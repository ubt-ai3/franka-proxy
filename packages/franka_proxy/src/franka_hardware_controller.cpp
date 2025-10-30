/**
 *************************************************************************
 *
 * @file franka_hardware_controller.cpp
 *
 * Classes to control a franka emika panda robot, implementation.
 *
 ************************************************************************/

#include "franka_hardware_controller.hpp"

#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "motion_generator_admittance.hpp"
#include "motion_generator_force.hpp"
#include "motion_generator_joint_impedance.hpp"
#include "motion_generator_joint_max_accel.hpp"
#include "motion_generator_seq_cart_vel_tau.hpp"
#include "motion_generator_joint_imp_ple.hpp"
#include "motion_recorder.hpp"

#include "ft_sensor/schunk_ft.hpp"

namespace franka_proxy
{
//////////////////////////////////////////////////////////////////////////
//
// franka_hardware_controller
//
//////////////////////////////////////////////////////////////////////////

franka_hardware_controller::franka_hardware_controller(
	const std::string& controller_ip,
	const bool enforce_realtime)
	: robot_(
		  controller_ip,
		  enforce_realtime ? franka::RealtimeConfig::kEnforce : franka::RealtimeConfig::kIgnore),
	  ft_sensor_(nullptr),
	  motion_recorder_(nullptr),
	  control_loop_running_(false),
	  robot_state_(robot_.readOnce()),
	  terminate_state_threads_(false)
{
	try
	{
		ft_sensor_ = std::make_unique<schunk_ft_sensor>(
			Eigen::Affine3f::Identity(), Eigen::Affine3f::Identity());
	}
	catch (const std::exception& e)
	{
		std::cout << "franka_proxy::franka_hardware_controller(const std::string & controller_ip): "
			<< "Connection to force/torque sensor could not be established:" << '\n'
			<< e.what() << '\n';
	}

	motion_recorder_ =
		std::make_unique<detail::motion_recorder>(
			robot_, robot_state_, ft_sensor_.get());

	try
	{
		gripper_ = std::make_unique<franka::Gripper>(controller_ip);
		max_width_ = gripper_->readOnce().max_width;
		gripper_state_ = gripper_->readOnce();
	}
	catch (const std::exception& e)
	{
		std::cout << "franka_proxy::franka_hardware_controller(const std::string & controller_ip): "
			<< "Connection to gripper could not be established\n"
			<< "Either, it is not attached or: "
			<< e.what() << '\n';
	}

	try
	{
		vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(controller_ip);
		vacuum_gripper_state_ = vacuum_gripper_->readOnce();
	}
	catch (const std::exception& e)
	{
		std::cout << "franka_proxy::franka_hardware_controller(const std::string & controller_ip): "
			<< "Connection to vacuum gripper could not be established.\n"
			<< "Either, it is not attached or: "
			<< e.what() << '\n';
	}

	set_guiding_mode({{true, true, true, true, true, true}}, false);

	robot_state_thread_ = std::thread([this]() { robot_state_update_loop(); });
	gripper_state_thread_ = std::thread([this]() { gripper_state_update_loop(); });
}


franka_hardware_controller::~franka_hardware_controller() noexcept
{
	try
	{
		if (vacuum_gripper_)
			vacuum_gripper_drop(std::chrono::milliseconds(100));
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << '\n';
	}

	terminate_state_threads_ = true;
	robot_state_thread_.join();
	gripper_state_thread_.join();
}


void franka_hardware_controller::automatic_error_recovery()
{
	robot_.automaticErrorRecovery();
}


void franka_hardware_controller::apply_z_force(
	const double mass,
	const double duration)
{
	try
	{
		detail::force_motion_generator fmg(robot_, mass, duration);

		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
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


void franka_hardware_controller::apply_admittance(
	double duration,
	double adm_rotational_stiffness,
	double adm_translational_stiffness,
	double imp_rotational_stiffness,
	double imp_translational_stiffness,
	const std::optional<std::string>& log_file_path)
{
	detail::admittance_motion_generator motion_generator(
		robot_, robot_state_lock_, robot_state_, duration,
		adm_rotational_stiffness, adm_translational_stiffness,
		imp_rotational_stiffness, imp_translational_stiffness,
		log_file_path);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback(robot_state, period);
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


void franka_hardware_controller::joint_impedance_hold_position(
	const double duration,
	const std::array<double, 49> stiffness,
	std::optional<std::string> log_file_path)
{
	detail::joint_impedance_motion_generator motion_generator(
		robot_, robot_state_lock_, robot_state_, duration, std::move(log_file_path));

	motion_generator.set_stiffness(stiffness);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback
				(robot_state, period,
				 [&](const double time) -> Eigen::Matrix<double, 7, 1>
				 {
					 return motion_generator.calculate_joint_position_error(robot_state, time);
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


void franka_hardware_controller::joint_impedance_positions(
	const std::list<std::array<double, 7>>& joint_positions,
	const double duration,
	const std::array<double, 49> stiffness,
	std::optional<std::string> log_file_path)
{
	detail::joint_impedance_motion_generator motion_generator(
		robot_, robot_state_lock_, robot_state_,
		joint_positions, duration, std::move(log_file_path));

	motion_generator.set_stiffness(stiffness);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback
				(robot_state, period,
				 [&](const double time) -> Eigen::Matrix<double, 7, 1>
				 {
					 return motion_generator.calculate_joint_position_error(robot_state, time);
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


void franka_hardware_controller::cartesian_impedance_hold_pose(
	const double duration,
	const bool use_stiff_damp_online_calc,
	const double rotational_stiffness,
	const double translational_stiffness,
	std::optional<std::string> log_file_path)
{
	detail::cartesian_impedance_motion_generator motion_generator(
		robot_, robot_state_lock_, robot_state_, duration,
		use_stiff_damp_online_calc, std::move(log_file_path));

	motion_generator.set_rotational_stiffness(rotational_stiffness);
	motion_generator.set_translational_stiffness(translational_stiffness);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback
				(robot_state, period,
				 [&](const double time) -> Eigen::Matrix<double, 6, 1>
				 {
					 return motion_generator.calculate_position_error(robot_state, time);
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


void franka_hardware_controller::cartesian_impedance_poses(
	const std::list<std::array<double, 16>>& poses,
	const double duration,
	const bool use_stiff_damp_online_calc,
	const double rotational_stiffness,
	const double translational_stiffness,
	std::optional<std::string> log_file_path)
{
	detail::cartesian_impedance_motion_generator motion_generator(
		robot_, robot_state_lock_, robot_state_, poses,
		duration, use_stiff_damp_online_calc, std::move(log_file_path));

	motion_generator.set_rotational_stiffness(rotational_stiffness);
	motion_generator.set_translational_stiffness(translational_stiffness);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback
				(robot_state, period,
				 [&](const double time) -> Eigen::Matrix<double, 6, 1>
				 {
					 return motion_generator.calculate_position_error(robot_state, time);
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


void franka_hardware_controller::run_payload_estimation(
	double speed,
	double duration,
	std::optional<std::string> log_file_path)
{
	detail::ple_motion_generator motion_generator(robot_, robot_state_lock_, robot_state_, speed, duration,
	                                              std::move(log_file_path));

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control(
			[&](const franka::RobotState& robot_state,
			    franka::Duration period) -> franka::Torques
			{
				return motion_generator.callback
				(robot_state, period,
				 [&](const double time) -> Eigen::Matrix<double, 7, 1>
				 {
					 return motion_generator.calculate_ple_motion(robot_state, time);
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


ft_sensor_response franka_hardware_controller::fts_state() const
{
	if (ft_sensor_)
		return ft_sensor_->read();
	else
		return ft_sensor_response{};
}


void franka_hardware_controller::move_to(const robot_config_7dof& target)
{
	set_default_impedance_and_collision_parameters();

	{
		// update robot_state to get the most current robot config for motion generation
		std::lock_guard state_guard(robot_state_lock_);
		robot_state_ = robot_.readOnce();
	}

	detail::franka_joint_motion_generator motion_generator
		(speed_factor_, target, robot_state_lock_, robot_state_, stop_motion_, false);

	stop_motion_ = false;

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control
		(motion_generator,
		 franka::ControllerMode::kJointImpedance,
		 true, 100.);
	}
	catch (const detail::franka_joint_motion_generator::stop_motion_trigger&)
	{
		std::cout << "motion stop triggered\n";
	}
	catch (const detail::franka_joint_motion_generator::contact_stop_trigger&)
	{
		std::cout << "contact occured\n";
	}
	catch (const franka::Exception& e)
	{
		set_control_loop_running(false);
		std::cout << "Error in move_to control loop :" << e.what() << '\n';
		throw;
	}

	set_control_loop_running(false);
}


bool franka_hardware_controller::move_to_until_contact(
	const robot_config_7dof& target)
{
	set_contact_move_impedance_and_collision_parameters();

	detail::franka_joint_motion_generator motion_generator
		(speed_factor_, target, robot_state_lock_, robot_state_, stop_motion_, true);

	stop_motion_ = false;

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control
		(motion_generator,
		 franka::ControllerMode::kJointImpedance,
		 true, 20.);
		/*robot_.control
		(motion_generator,
			franka::ControllerMode::kCartesianImpedance,
			true, 100.);*/
	}
	catch (const detail::franka_joint_motion_generator::stop_motion_trigger&)
	{
		std::cout << "motion stopped\n";
	}
	catch (const detail::franka_joint_motion_generator::contact_stop_trigger&)
	{
		std::cout << "contact occured\n";

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		automatic_error_recovery();

		set_control_loop_running(false);
		set_default_impedance_and_collision_parameters();

		return false;
	}
	catch (const franka::Exception&)
	{
		set_control_loop_running(false);
		std::cout << "error in move to until contact control loop\n";
		throw;
	}

	set_control_loop_running(false);
	set_default_impedance_and_collision_parameters();
	return true;
}


void franka_hardware_controller::stop_movement()
{
	stop_motion_ = true;

	try
	{
		if (gripper_)
			if (!gripper_->stop())
				std::cerr << "Gripper stop failed." << '\n';
	}
	catch (const franka::Exception& e)
	{
		std::cerr << "franka_hardware_controller::stop_movement(): Failed with: "
			<< e.what() << ".\n";
	}
}


void franka_hardware_controller::set_speed_factor(double speed_factor)
{
	std::lock_guard state_guard(speed_factor_lock_);
	speed_factor_ = std::max(0.001, std::min(1.0, speed_factor));
	std::cout << "franka_hardware_controller::set_speed_factor(double speed_factor): " << speed_factor_ << "\n";
}


void franka_hardware_controller::set_bias(const std::array<double, 6>& bias)
{
	if (ft_sensor_)
		ft_sensor_->set_bias(Eigen::Vector<double, 6>(bias.data()));
	else
		throw ft_sensor_connection_exception();
}


void franka_hardware_controller::set_load_mass(double load_mass)
{
	if (ft_sensor_)
		ft_sensor_->set_load_mass(load_mass);
	else
		throw ft_sensor_connection_exception();
}


void franka_hardware_controller::set_guiding_mode(const std::array<bool, 6>& guiding_mode, const bool elbow)
{
	robot_.setGuidingMode(guiding_mode, elbow);
}


franka::RobotState franka_hardware_controller::robot_state() const
{
	std::lock_guard state_guard(robot_state_lock_);
	return robot_state_;
}


void franka_hardware_controller::open_gripper(double speed)
{
	if (!gripper_)
		throw std::runtime_error("Tried to use non existent gripper, make sure you use the jaw gripper");

	if (!gripper_->move(max_width_, speed))
	{
		std::cerr << "Gripper opening failed." << '\n';
	}
}


void franka_hardware_controller::close_gripper(double speed)
{
	if (!gripper_)
		throw std::runtime_error("Tried to use non existent gripper, make sure you use the jaw gripper");

	if (!gripper_->move(min_grasp_width, speed))
	{
		std::cerr << "Gripper closing failed." << '\n';
	}
}


bool franka_hardware_controller::grasp_gripper(double speed, double force)
{
	if (!gripper_)
		throw std::runtime_error("Tried to use non existent gripper, make sure you use the jaw gripper");

	bool grasped = gripper_->grasp(min_grasp_width, speed, force, 0, 1);

	return grasped;
}


franka::GripperState franka_hardware_controller::gripper_state() const
{
	std::lock_guard state_guard(gripper_state_lock_);
	return gripper_state_;
}


bool franka_hardware_controller::vacuum_gripper_drop(std::chrono::milliseconds timeout)
{
	if (!vacuum_gripper_)
		throw std::runtime_error(
			"Tried to use non existent gripper, make sure you use the vacuum gripper");

	bool success = false;
	try
	{
		success = vacuum_gripper_->dropOff(timeout);
	}
	catch (const franka::CommandException& e)
	{
		std::cout << "franka_hardware_controller::vacuum_gripper_drop(): "
			<< e.what() << '\n';
	}

	{
		std::scoped_lock state_guard(gripper_state_lock_);
		vacuum_gripper_state_ = vacuum_gripper_->readOnce();
	}

	return success;
}


bool franka_hardware_controller::vacuum_gripper_vacuum(
	std::uint8_t vacuum_strength,
	std::chrono::milliseconds timeout)
{
	if (!vacuum_gripper_)
		throw std::runtime_error(
			"vacuum gripper not found, make sure you mounted the vacuum gripper");

	bool success = false;
	try
	{
		success = vacuum_gripper_->vacuum(vacuum_strength, timeout);
	}
	catch (const franka::CommandException& e)
	{
		std::cout << "franka_hardware_controller::vacuum_gripper_vacuum(): "
			<< e.what() << '\n';
	}

	{
		std::scoped_lock state_guard(gripper_state_lock_);
		vacuum_gripper_state_ = vacuum_gripper_->readOnce();
	}

	if (success != vacuum_gripper_state_.part_present)
		std::cout << "franka_hardware_controller::vacuum_gripper_vacuum(): "
			<< "Failed vacuum but attached object" << '\n';

	return vacuum_gripper_state_.part_present;
}


bool franka_hardware_controller::vacuum_gripper_stop()
{
	if (!vacuum_gripper_)
		throw std::runtime_error(
			"vacuum gripper not found, make sure you mounted the vacuum gripper");

	bool success = vacuum_gripper_->stop();

	{
		std::scoped_lock state_guard(gripper_state_lock_);
		vacuum_gripper_state_ = vacuum_gripper_->readOnce();
	}

	return success;
}


franka::VacuumGripperState franka_hardware_controller::vacuum_gripper_state() const
{
	std::scoped_lock guard(gripper_state_lock_);
	return vacuum_gripper_state_;
}


void franka_hardware_controller::start_recording(std::optional<std::string> log_file_path)
{
	if (!ft_sensor_)
		throw ft_sensor_connection_exception();

	set_control_loop_running(true);
	{
		// Lock the current_state_lock_ to wait for state_thread_ to finish.
		std::lock_guard state_guard(robot_state_lock_);
	}

	motion_recorder_->start(std::move("log_file_path.csv"));
}


std::pair<std::vector<robot_config_7dof>, std::vector<wrench>>
franka_hardware_controller::stop_recording()
{
	auto record = motion_recorder_->stop();
	set_control_loop_running(false);

	return record;
}


std::pair<std::vector<robot_config_7dof>, std::vector<wrench>>
franka_hardware_controller::recording_for(float seconds, std::optional<std::string> log_file_path)
{
	set_control_loop_running(true);
	{
		// Lock the current_state_lock_ to wait for state_thread_ to finish.
		std::lock_guard state_guard(robot_state_lock_);
	}

	auto record = motion_recorder_->start(seconds, std::move(log_file_path));
	set_control_loop_running(false);

	return record;
}


void franka_hardware_controller::move_sequence(
	const std::vector<std::array<double, 7>>& q_sequence)
{
	std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), {0, 0, 0, 0, 0, 0});
	std::vector<std::array<double, 6>> selection_vector_sequence(q_sequence.size(), {1, 1, 1, 1, 1, 1});

	stop_motion_ = false;
	detail::seq_cart_vel_tau_generator motion_generator(
		robot_state_lock_, robot_state_, robot_, stop_motion_,
		q_sequence, f_sequence, selection_vector_sequence);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
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


void franka_hardware_controller::move_sequence(
	const std::vector<std::array<double, 7>>& q_sequence, double f_z)
{

	std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), {0, 0, f_z, 0, 0, 0});
	std::vector<std::array<double, 6>> selection_vector_sequence(q_sequence.size(), {1, 1, 0, 1, 1, 1});

	stop_motion_ = false;
	detail::seq_cart_vel_tau_generator motion_generator(
		robot_state_lock_, robot_state_, robot_, stop_motion_,
		q_sequence, f_sequence, selection_vector_sequence);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
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


void franka_hardware_controller::move_sequence(
	const std::vector<std::array<double, 7>>& q_sequence,
	const std::vector<std::array<double, 6>>& f_sequence,
	const std::vector<std::array<double, 6>>& selection_vector,
	const std::array<double, 16>& offset_position,
	const std::array<double, 6>& offset_force)
{
	stop_motion_ = false;
	detail::seq_cart_vel_tau_generator motion_generator(
		robot_state_lock_, robot_state_, robot_, stop_motion_,
		q_sequence, f_sequence, selection_vector);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
		}

		robot_.control([&](
		               const franka::RobotState& robot_state,
		               franka::Duration period) -> franka::Torques
		               {
			               return motion_generator.step(robot_state, period, offset_position, offset_force);
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


void franka_hardware_controller::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_vector)
{
	stop_motion_ = false;
	detail::seq_cart_vel_tau_generator motion_generator(
		robot_state_lock_, robot_state_, robot_, stop_motion_,
		q_sequence, f_sequence, selection_vector);

	try
	{
		set_control_loop_running(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard state_guard(robot_state_lock_);
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
			std::unique_lock lk(control_loop_running_mutex_);
			if (control_loop_running_)
			{
				control_loop_running_cv_.wait(lk);
				if (control_loop_running_)
					continue;
			}
		}

		try
		{
			std::lock_guard state_guard(robot_state_lock_);
			robot_state_ = robot_.readOnce();

			if (gripper_)
				gripper_state_ = gripper_->readOnce();
			if (vacuum_gripper_)
				vacuum_gripper_state_ = vacuum_gripper_->readOnce();
		}
		catch (...)
		{
		} // Do not propagate ugly gripper errors on robot shutdown.

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(33ms);
	}
}


void franka_hardware_controller::gripper_state_update_loop()
{
	while (!terminate_state_threads_)
	{
		if (gripper_)
		{
			std::lock_guard state_guard(gripper_state_lock_);
			gripper_state_ = gripper_->readOnce();
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(33ms);
	}
}


void franka_hardware_controller::set_default_impedance_and_collision_parameters()
{
	try
	{
		robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
		robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

		robot_.setCollisionBehavior(
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{40.0, 40.0, 38.0, 38.0, 36.0, 34.0, 32.0}},
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{40.0, 40.0, 38.0, 38.0, 36.0, 34.0, 32.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}});
	}
	catch (franka::Exception& e)
	{
		std::cerr << "franka_hardware_controller::set_default_impedance_and_collision_parameters(): "
			<< e.what() << '\n';
	}
}


void franka_hardware_controller::set_contact_move_impedance_and_collision_parameters()
{
	try
	{
		robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
		robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

		robot_.setCollisionBehavior(
			{{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}}, {{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}},
			{{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}}, {{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}},
			{{5.0, 5.0, 5.0, 6.25, 6.25, 6.25}}, {{5.0, 5.0, 5.0, 6.25, 6.25, 6.25}},
			{{5.0, 5.0, 5.0, 6.25, 6.25, 6.25}}, {{5.0, 5.0, 5.0, 6.25, 6.25, 6.25}});
	}
	catch (franka::Exception& e)
	{
		std::cerr << "franka_hardware_controller::set_contact_move_impedance_and_collision_parameters(): "
			<< e.what() << '\n';
	}
}


void franka_hardware_controller::set_control_loop_running(bool running)
{
	{
		std::lock_guard lk(control_loop_running_mutex_);
		control_loop_running_ = running;
	}

	control_loop_running_cv_.notify_all();
}
} /* namespace franka_proxy */
