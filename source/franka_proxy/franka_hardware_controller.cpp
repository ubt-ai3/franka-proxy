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

#include "franka_motion_recorder.hpp"
#include "impedance_position_generator.hpp"
#include "motion_generator_force.hpp"
#include "motion_generator_impedance_hold_position.hpp"
#include "motion_generator_joint_max_accel.hpp"
#include "motion_generator_seq_cart_vel_tau.hpp"


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
		speed_factor_(0.05),

		motion_recorder_(10.0, robot_, robot_state_),

		robot_state_(robot_.readOnce()),

		control_loop_running_(false),

		terminate_state_threads_(false),
		robot_state_thread_([this]() { robot_state_update_loop(); }),
		gripper_state_thread_([this]() { gripper_state_update_loop(); })

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

		// todo JHa
		//robot_.setGuidingMode({ {true, true, true, false, false, true} }, false);
	}


	franka_hardware_controller::~franka_hardware_controller() noexcept
	{
		terminate_state_threads_ = true;
		robot_state_thread_.join();
		gripper_state_thread_.join();
	}


	void franka_hardware_controller::apply_z_force
	(const double mass, const double duration)
	{
		initialize_parameters();

		try
		{
			detail::force_motion_generator fmg(robot_, mass, duration);

			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			// start real-time control loop
			robot_.control(
				[&](const franka::RobotState& robot_state,
					franka::Duration period) -> franka::Torques
				{
					return fmg.callback(robot_state, period);
				}, true, 10.0);
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}

	void franka_hardware_controller::impedance_hold_position(const double duration)
	{
		detail::impedance_hold_position_motion_generator motion_generator(robot_, robot_state_lock_, robot_state_, duration);
		detail::impedance_position_generator position_generator(robot_state_, robot_state_lock_);

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control(
				[&](const franka::RobotState& robot_state,
					franka::Duration period) -> franka::Torques
				{
					return motion_generator.callback
						(robot_state, period,
							[&](const double time) -> Eigen::Vector3d
							{
								return position_generator.hold_current_position(time);
							}
						);
				}, true, 10.0
			);
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}

	void franka_hardware_controller::impedance_follow_positions(double duration)
	//void franka_hardware_controller::impedance_follow_positions(std::list<std::array<double, 3>>& positions, double duration)
	{
		detail::impedance_hold_position_motion_generator motion_generator(robot_, robot_state_lock_, robot_state_, duration);
		detail::impedance_position_generator position_generator(robot_state_, robot_state_lock_);
		//detail::impedance_position_generator position_generator(robot_state_, robot_state_lock_, positions, duration);

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control(
				[&](const franka::RobotState& robot_state,
					franka::Duration period) -> franka::Torques
				{
					return motion_generator.callback
					(robot_state, period,
						[&](const double time) -> Eigen::Vector3d
						{
							return position_generator.hold_current_position(time);
						}
					);
				}, true, 10.0
			);
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}


	void franka_hardware_controller::move_to(const robot_config_7dof& target)
	{
		initialize_parameters();

		detail::franka_joint_motion_generator motion_generator
		(speed_factor_, target, robot_state_lock_, robot_state_, stop_motion_, false);

		stop_motion_ = false;

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control
			(motion_generator,
				franka::ControllerMode::kJointImpedance,
				true, 20.);
		}
		catch (const detail::franka_joint_motion_generator::stop_motion_trigger&)
		{
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}


	bool franka_hardware_controller::move_to_until_contact
	(const robot_config_7dof& target)
	{
		initialize_parameters();

		detail::franka_joint_motion_generator motion_generator
		(speed_factor_, target, robot_state_lock_, robot_state_, stop_motion_, true);

		stop_motion_ = false;
		set_contact_drive_collision_behaviour();

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control
			(motion_generator,
				franka::ControllerMode::kJointImpedance,
				true, 20.);
		}
		catch (const detail::franka_joint_motion_generator::stop_motion_trigger&)
		{
		}
		catch (const detail::franka_joint_motion_generator::contact_stop_trigger&)
		{
			set_control_loop_running(false);
			set_default_collision_behaviour();
			return false;
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
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
		{
		}
	}


	void franka_hardware_controller::set_speed_factor(double speed_factor)
	{
		std::lock_guard<std::mutex> state_guard(speed_factor_lock_);
		speed_factor_ = speed_factor;
	}


	franka::RobotState franka_hardware_controller::robot_state() const
	{
		std::lock_guard<std::mutex> state_guard(robot_state_lock_);
		return robot_state_;
	}


	void franka_hardware_controller::open_gripper(double speed)
	{
		if (!gripper_)
			return; // todo throw something usefull

		if (!gripper_->move(max_width_, speed))
		{
			std::cerr << "Gripper opening failed." << std::endl;
		}
	}


	void franka_hardware_controller::close_gripper(double speed)
	{
		if (!gripper_)
			return; // todo throw something usefull

		if (!gripper_->move(min_grasp_width, speed))
		{
			std::cerr << "Gripper closing failed." << std::endl;
		}
	}


	bool franka_hardware_controller::grasp_gripper(double speed, double force)
	{
		if (!gripper_)
			return false; // todo throw something usefull

		bool grasped = gripper_->grasp(min_grasp_width, speed, force, 0, 1);

		return grasped;
	}


	franka::GripperState franka_hardware_controller::gripper_state() const
	{
		std::lock_guard<std::mutex> state_guard(gripper_state_lock_);
		return gripper_state_;
	}


	void franka_hardware_controller::automatic_error_recovery()
	{
		robot_.automaticErrorRecovery();
	}


	void franka_hardware_controller::start_recording()
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard<std::mutex> state_guard(robot_state_lock_);
		}

		motion_recorder_.start();
	}


	std::pair<std::vector<robot_config_7dof>, std::vector<robot_force_config>>
		franka_hardware_controller::stop_recording()
	{
		motion_recorder_.stop();
		set_control_loop_running(false);

		return { motion_recorder_.latest_record(), motion_recorder_.latest_fts_record() };
	}


	void franka_hardware_controller::move_sequence
	(const std::vector<std::array<double, 7>>& q_sequence)
	{
		initialize_parameters();
		set_default_collision_behaviour();

		std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), { 0,0,0,0,0,0 });
		std::vector<std::array<double, 6>> selection_vector_sequence(q_sequence.size(), { 1,1,1,1,1,1 });


		stop_motion_ = false;
		detail::seq_cart_vel_tau_generator motion_generator(robot_state_lock_, robot_state_, robot_, stop_motion_, q_sequence, f_sequence, selection_vector_sequence);


		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control(
				[&](const franka::RobotState& robot_state,
					franka::Duration period) -> franka::Torques
				{
					return motion_generator.step(robot_state, period);
				},
				true,
					1000.);

		}
		catch (const detail::seq_cart_vel_tau_generator::stop_motion_trigger&)
		{
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}


	void franka_hardware_controller::move_sequence
	(const std::vector<std::array<double, 7>>& q_sequence, double f_z)
	{
		initialize_parameters();
		set_default_collision_behaviour();

		// wrong implementation
		//detail::force_motion_generator force_motion_generator(robot_, 0.5, 10.0);
		//detail::sequence_joint_velocity_motion_generator joint_velocity_motion_generator(1., q_sequence, state_lock_, robot_state_, stop_motion_);



		std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), { 0,0,f_z,0,0,0 });
		std::vector<std::array<double, 6>> selection_vector_sequence(q_sequence.size(), { 1,1,0,1,1,1 });


		//double f_x = -5.0;

		//std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), { f_x,0,f_z,0,0,0 });
		//std::vector<std::array<double, 6>> selection_vector_sequence(q_sequence.size(), { 0,1,0,1,1,1 });


		stop_motion_ = false;
		detail::seq_cart_vel_tau_generator motion_generator(robot_state_lock_, robot_state_, robot_, stop_motion_, q_sequence, f_sequence, selection_vector_sequence);

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}


			// wrong implementation
			//robot_.control(
			//	[&](const franka::RobotState& robot_state,
			//		franka::Duration period) -> franka::Torques
			//	{
			//		return force_motion_generator.callback(robot_state, period);
			//	},
			//	joint_velocity_motion_generator,
			//	true,
			//	100.);

			robot_.control(
				[&](const franka::RobotState& robot_state,
					franka::Duration period) -> franka::Torques
				{
					return motion_generator.step(robot_state, period);
				},
				true,
					1000.);

		}
		catch (const detail::seq_cart_vel_tau_generator::stop_motion_trigger&)
		{
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}


	void franka_hardware_controller::move_sequence
	(const std::vector<std::array<double, 7>>& q_sequence,
		const std::vector<std::array<double, 6>>& f_sequence,
		const std::vector<std::array<double, 6>>& selection_vector)
	{
		initialize_parameters();
		set_default_collision_behaviour();

		stop_motion_ = false;
		detail::seq_cart_vel_tau_generator motion_generator(robot_state_lock_, robot_state_, robot_, stop_motion_, q_sequence, f_sequence, selection_vector);

		try
		{
			set_control_loop_running(true);
			{
				// Lock the current_state_lock_ to wait for state_thread_ to finish.
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
			}

			robot_.control([&](
				const franka::RobotState& robot_state,
				franka::Duration period) -> franka::Torques
				{
					return motion_generator.step(robot_state, period);
				},
				true,
					1000.);

		}
		catch (const detail::seq_cart_vel_tau_generator::stop_motion_trigger&)
		{
		}
		catch (const franka::Exception&)
		{
			set_control_loop_running(false);
			throw;
		}

		set_control_loop_running(false);
	}


	void franka_hardware_controller::robot_state_update_loop()
	{
		while (!terminate_state_threads_)
		{
			{
				std::unique_lock<std::mutex> lk(control_loop_running_mutex_);
				if (control_loop_running_)
				{
					control_loop_running_cv_.wait(lk);
					if (control_loop_running_)
						continue;
				}
			}

			try
			{
				std::lock_guard<std::mutex> state_guard(robot_state_lock_);
				robot_state_ = robot_.readOnce();
			}
			catch (...) {} // Don't propagate ugly error on robot shutdown...

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(33ms);
		}
	}


	void franka_hardware_controller::gripper_state_update_loop()
	{
		while (!terminate_state_threads_)
		{
			if (gripper_) {
				std::lock_guard<std::mutex> state_guard(gripper_state_lock_);
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

			robot_.setJointImpedance({ {3000, 3000, 3000, 2500, 2500, 2000, 2000} });
			robot_.setCartesianImpedance({ {3000, 3000, 3000, 300, 300, 300} });

			parameters_initialized_ = true;
		}
	}

	void franka_hardware_controller::set_default_collision_behaviour()
	{
		robot_.setCollisionBehavior(
			{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} }, { {40.0, 40.0, 38.0, 38.0, 36.0, 34.0, 32.0} },
			{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} }, { {40.0, 40.0, 38.0, 38.0, 36.0, 34.0, 32.0} },
			{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} }, { {40.0, 40.0, 40.0, 45.0, 45.0, 45.0} },
			{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} }, { {40.0, 40.0, 40.0, 45.0, 45.0, 45.0} });
	}


	void franka_hardware_controller::set_contact_drive_collision_behaviour()
	{
		robot_.setCollisionBehavior(
			{ {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} }, { {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} },
			{ {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} }, { {5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0} },
			{ {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} }, { {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} },
			{ {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} }, { {5.0, 5.0, 5.0, 6.25, 6.25, 6.25} });
	}


	void franka_hardware_controller::set_control_loop_running(bool running)
	{
		{
			std::lock_guard<std::mutex> lk(control_loop_running_mutex_);
			control_loop_running_ = running;
		}
		control_loop_running_cv_.notify_all();
	}
} /* namespace franka_proxy */
