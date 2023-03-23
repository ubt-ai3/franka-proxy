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
#include "motion_generator_force.hpp"
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
	{}
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

	std::vector<std::array<double, 6>> f_sequence(q_sequence.size(), {0,0,0,0,0,0});
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

		robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
		robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

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

void franka_hardware_controller::set_impedance() {
	// TODO: rest of robot state

	// get robot model
	franka::Model model_ = robot_.loadModel();

	// get jacobian
	std::array<double, 42> jac_ar_ = model_.zeroJacobian(franka::Frame::kEndEffector, robot_state());
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_(jac_ar_.data());

	// get current position
	Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(robot_state().O_T_EE.data()));
	Eigen::Vector3d position_(po_transform_.translation());

	Eigen::Matrix<double, 6, 1> position_error_;

	// get current velocity
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_(robot_state().dq.data());
	Eigen::Matrix<double, 6, 1> velocity_ = jacobian_ * dq_; // dx = j(q)*dq

	// init acceleration variable
	Eigen::Matrix<double, 6, 1> acceleration_;

	// initialize impedance parameters
	if (!impedance_parameters_initialized_) {
		// calculate desired position
		Eigen::Affine3d pod_transform_(Eigen::Matrix4d::Map(robot_state().O_T_EE.data()));
		position_d_(pod_transform_.translation());

		// convert current velcoity to init measured velocities
		std::array<double, 6> new_measured_velocity_;

		for (int i = 0; i < velocity_.rows(); i++) {
			// get value
			double vel_ = velocity_(i, 0);

			// store it in new velocity array
			new_measured_velocity_[i] = vel_;
		}

		measured_velocities_.push_back(new_measured_velocity_);

		// calculate/set x0_max and derived_x0_max
		for (int i = 0; i < sizeof x0_max_ / sizeof x0_max_[0]; i++) {
			x0_max_[i] = std::max(std::abs(l_x0_[i]), u_x0_[i]);
			derived_x0_max_[i] = std::max(std::abs(l_derived_x0_[i]), u_derived_x0_[i]);
		}

		impedance_parameters_initialized_ = true;
	}

	// set position error
	position_error_.head(3) << position_ - position_d_; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix // TODO: what is head doing?

	// convert current velcoity and push it to measured velocities
	std::array<double, 6> new_measured_velocity_;

	for (int i = 0; i < velocity_.rows(); i++) {
		// get value
		double vel_ = velocity_(i, 0);

		// store it in new velocity array
		new_measured_velocity_[i] = vel_;
	}

	measured_velocities_.push_back(new_measured_velocity_);

	// remove first element of measured_velocitues_ if there are more then eleven elements to calculate the current acceleration by the current velocity and the velocity measured ten cycles ago
	if (measured_velocities_.size() > 11) {
		measured_velocities_.pop_front();
	}

	// calculate acceleration
	std::array<double, 6> acc_list_;
	double delta_time_ = measured_velocities_.size() * 0.001;

	for (int i = 0; i < acc_list_.size(); i++) {
		double delta_velocity_ = measured_velocities_.back()[i] - measured_velocities_.front()[i];
		acc_list_[i] = delta_velocity_ / delta_time_;
	}

	// set acceleration
	acceleration_(acc_list_.data());

	// TODO: move those elsewhere -> on each loop init with 0 will run the stability check on each loop
	// damping and stiffness matrix
	Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6,6>::Zero();
	Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

	// stiffness and damping
	for (int i = 0; i < position_error_.rows(); i++) {
		double mi = 0.0; // TODO: get mi value from inertia 
		
		// optimize damping
		double di = optimizeDamping(l_d_[i], u_d_[i], mi, b_[i], x0_max_[i], derived_x0_max_[i]);

		// TODO: stability check

		// get stiffness from new calculated damping value
		double ki = calculate_stiffness_from_damping(di, mi);

		// add new values to matrices
		damping_matrix_(i, i) = di;
		stiffness_matrix_(i, i) = ki;
	}

	// TEST STIFFNESS AND DAMPING FOR TESTING WITHOUT IMPEDANCE PLANNER
	// const double translational_stiffness{ 150.0 };
	// const double rotational_stiffness{ 10.0 };
	
	// stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	// stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	
	// damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
		// Eigen::MatrixXd::Identity(3, 3);
	// damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
		// Eigen::MatrixXd::Identity(3, 3);
	// ----- TEST STIFFNESS AND DAMPING FOR TESTING WITHOUT IMPEDANCE PLANNER
	
}

double franka_hardware_controller::optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max) {
	// di = min(max(...), ...);
	const double di_max_val_ = std::max(l_di,((2 * mi * derived_x0i_max)/((bi - x0i_max) * exp(1))));
	return std::min(di_max_val_, u_di);
}

double franka_hardware_controller::calculate_stiffness_from_damping(double di, double mi) {
	/**
		critically damped condition

		stiffness ki = (di)^2/4mmi
	*/

	double ki_ = di * di;

	if (mi <= 0) {
		// ki = ki/4;
		// do nothing and return (di^2) ???
	}
	else {
		ki_ = ki_ / (4 * mi);
	}

	return ki_;
}





} /* namespace franka_proxy */
