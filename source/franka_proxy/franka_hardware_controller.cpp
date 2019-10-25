/**
 *************************************************************************
 *
 * @file franka_hardware_controller.cpp
 *
 * Classes to control a franka emika panda robot, implementation.
 *
 ************************************************************************/


#include "franka_hardware_controller.hpp"

#include <iostream>

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "franka_motion_generator.hpp"


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_hardware_controller
//
//////////////////////////////////////////////////////////////////////////


franka_hardware_controller::franka_hardware_controller
	(const std::string& controller_ip)
	:
	robot_(controller_ip, franka::RealtimeConfig::kIgnore),
	parameters_initialized_(false),
	stop_motion_(),
	speed_factor_(0.05),

	robot_state_(robot_.readOnce()),

	terminate_state_thread_(false),
	state_thread_([this]() { state_update_loop(); })
{
	try
	{
		gripper_ = std::make_unique<franka::Gripper>(controller_ip);
		max_width_ = gripper_->readOnce().max_width;
		gripper_state_ = gripper_->readOnce();
	}
	catch (...)
	{
		// todo
	}
}


franka_hardware_controller::~franka_hardware_controller() noexcept
{
	terminate_state_thread_ = true;
	state_thread_.join();
}


void franka_hardware_controller::apply_z_force
	(const double mass, const double duration)
{
	initialize_parameters();

	try
	{
		detail::force_motion_generator fmg(robot_, mass, duration);

		control_loop_running_.set(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard<std::mutex> state_guard(state_lock_);
		}

		// start real-time control loop
		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return fmg.callback(robot_state, period);
			});
	}
	catch (const detail::stop_motion_trigger&)
	{
	}
	catch (const franka::Exception&)
	{
		control_loop_running_.set(false);
		throw;
	}

	control_loop_running_.set(false);
}


void franka_hardware_controller::move_to(const robot_config_7dof& target)
{
	initialize_parameters();

	detail::motion_generator motion_generator
		(speed_factor_, target, state_lock_, robot_state_, stop_motion_, false);

	stop_motion_ = false;

	try
	{
		control_loop_running_.set(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard<std::mutex> state_guard(state_lock_);
		}

		robot_.control
			(motion_generator,
			 franka::ControllerMode::kJointImpedance,
			 true, 20.);
	}
	catch (const detail::stop_motion_trigger&)
	{
	}
	catch (const franka::Exception&)
	{
		control_loop_running_.set(false);
		throw;
	}

	control_loop_running_.set(false);
}


bool franka_hardware_controller::move_to_until_contact
	(const robot_config_7dof& target)
{
	initialize_parameters();

	detail::motion_generator motion_generator
		(speed_factor_, target, state_lock_, robot_state_, stop_motion_, true);

	stop_motion_ = false;
	set_contact_drive_collision_behaviour();

	try
	{
		control_loop_running_.set(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard<std::mutex> state_guard(state_lock_);
		}

		robot_.control
			(motion_generator,
			 franka::ControllerMode::kJointImpedance,
			 true, 20.);
	}
	catch (const detail::stop_motion_trigger&)
	{
	}
	catch (const detail::contact_stop_trigger&)
	{
		control_loop_running_.set(false);
		set_default_collision_behaviour();
		return false;
	}
	catch (const franka::Exception&)
	{
		control_loop_running_.set(false);
		throw;
	}
	
	control_loop_running_.set(false);
	set_default_collision_behaviour();
	return true;
}


void franka_hardware_controller::stop_movement()
{
	stop_motion_ = true;

	try
	{
		if (gripper_)
			gripper_->stop();
	}
	catch (const franka::Exception&)
	{}
}


void franka_hardware_controller::set_speed_factor(double speed_factor)
{
	std::lock_guard<std::mutex> state_guard(speed_factor_lock_);
	speed_factor_ = speed_factor;
}


franka::RobotState franka_hardware_controller::robot_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return robot_state_;
}


void franka_hardware_controller::open_gripper()
{
	if (!gripper_)
		return; // todo throw something usefull

	if (!gripper_->move(max_width_, gripper_speed))
	{
		std::cerr << "Gripper opening failed." << std::endl;
	}

	{
		std::lock_guard<std::mutex> state_guard(state_lock_);
		gripper_state_ = gripper_->readOnce();
	}
}


void franka_hardware_controller::close_gripper()
{
	if (!gripper_)
		return; // todo throw something usefull

	if (!gripper_->move(min_grasp_width, gripper_speed))
	{
		std::cerr << "Gripper closing failed." << std::endl;
	}

	{
		std::lock_guard<std::mutex> state_guard(state_lock_);
		gripper_state_ = gripper_->readOnce();
	}
}


void franka_hardware_controller::grasp_gripper(double speed, double force)
{
	throw std::exception("wrong implementation");

	if (!gripper_)
		return; // todo throw something usefull

	bool grasped = gripper_->grasp(min_grasp_width, speed, force, 0, 1);

	{
		std::lock_guard<std::mutex> state_guard(state_lock_);
		gripper_state_ = gripper_->readOnce();
	}
}


franka::GripperState franka_hardware_controller::gripper_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return gripper_state_;
}


void franka_hardware_controller::automatic_error_recovery()
{
	robot_.automaticErrorRecovery();
}


void franka_hardware_controller::state_update_loop()
{
	while (!terminate_state_thread_)
	{
		control_loop_running_.wait_for(false);
		if (control_loop_running_.get())
			continue;

		{
			std::lock_guard<std::mutex> state_guard(state_lock_);
			robot_state_ = robot_.readOnce();
			if (gripper_) 
				gripper_state_ = gripper_->readOnce();
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(33ms);
	}
}


void franka_hardware_controller::initialize_parameters()
{
	while (!parameters_initialized_)
	{
		set_default_collision_behaviour();

		robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
		robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

		parameters_initialized_ = true;
	}
}


void franka_hardware_controller::set_default_collision_behaviour()
{
	robot_.setCollisionBehavior(
		{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
		{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
		{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
		{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
}


void franka_hardware_controller::set_contact_drive_collision_behaviour()
{
	robot_.setCollisionBehavior(
		{ {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} }, { {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} },
		{ {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} }, { {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} },
		{ {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} }, { {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} },
		{ {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} }, { {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} });
}




} /* namespace franka_proxy */
