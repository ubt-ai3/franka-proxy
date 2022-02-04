/**
 *************************************************************************
 *
 * @file motion_generator_force.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "motion_generator_force.hpp"

#include <utility>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/model.h>


namespace franka_proxy
{
namespace detail
{


//////////////////////////////////////////////////////////////////////////
//
// force_motion_generator
//
//////////////////////////////////////////////////////////////////////////


force_motion_generator::force_motion_generator
	(franka::Robot& robot,
	 double mass,
	 double duration)
	:
	dq_d_({0., 0., 0., 0., 0., 0., 0.}),
	dq_buffer_(dq_filter_size_ * 7, 0),
	target_mass(mass),
	duration(duration),
	model(robot.loadModel())
{
	initial_state_ = robot.readOnce();
}


franka::Torques force_motion_generator::callback
	(const franka::RobotState& robot_state,
	 franka::Duration period)
{
	time_ += period.toSec();

	if (time_ > duration)
	{
		// todo this may be wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}


	Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

	std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());


	Eigen::VectorXd desired_force_torque(6), tau_existing(7), tau_desired(7), tau_command(7), tau_J_d(7);


	desired_force_torque.setZero();
	desired_force_torque(2) = desired_mass * -9.81;


	tau_existing = tau_measured - gravity;
	tau_desired = jacobian.transpose() * desired_force_torque;
	tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	// FF + PI control
	tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	//desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;
	desired_mass = target_mass;

	// updateDQFilter
	update_dq_filter(robot_state);

	// compute torques according to impedance control law
	// with assumption mass = 0 (robot state does not provide ddq and own measurement too noisy)
	for (int i = 0; i < 7; i++)
		tau_J_d[i] = K_P_[i] * (robot_state.q_d[i] - robot_state.q[i]) + K_D_[i] * (dq_d_[i] - compute_dq_filtered(i));


	//The final output vector
	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = (tau_command + tau_J_d) * 0.5;


	forces_z.push_back(robot_state.O_F_ext_hat_K[2]);
	des_mass.push_back(desired_mass * -9.81);


	return tau_d_array;
}

std::vector<double> force_motion_generator::give_forces() {
	return forces_z;
}
std::vector<double> force_motion_generator::give_desired_mass() {
	return des_mass;
}


void force_motion_generator::update_dq_filter(const franka::RobotState& robot_state)
{
	for (int i = 0; i < 7; i++)
		dq_buffer_[dq_current_filter_position_ * 7 + i] = robot_state.dq[i];

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


double force_motion_generator::compute_dq_filtered(int j)
{
	double value = 0.0;
	for (size_t i = j; i < 7 * dq_filter_size_; i += 7)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}

//-------------------------------------------------------------------------------------------------
//------------------Class Pid Force Control Motion Generator---------------------------------------
//-------------------------------------------------------------------------------------------------

//Constructor
pid_force_control_motion_generator::pid_force_control_motion_generator
(franka::Robot& robot,
	double mass,
	double duration)
	:
	tau_command_buffer_(tau_command_filter_size_ * 7, 0),
	force_error_diff_buffer_(force_error_diff_filter_size_ * 6, 0),
	position_error_diff_buffer_(position_error_diff_filter_size_ * 6, 0),
	target_mass_(mass),
	duration_(duration),
	model_(robot.loadModel())
{
	initial_state_ = robot.readOnce();
}

franka::Torques pid_force_control_motion_generator::callback
(const franka::RobotState& robot_state,
	franka::Duration period)
{
	time_ += period.toSec();

	if (time_ > duration_) //Finished
	{
		// todo this may be wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}

	std::array<double, 42> jacobian_array = model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_existing(robot_state.O_F_ext_hat_K.data());

	//Set desired cartesian Position as the first measured position (initial)
	if (count_loop_ == 0) {
		Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data()));
		position_desired_ = initial_transform.translation();
		orientation_desired_ = initial_transform.linear();
	}

	//Set force_desired
	Eigen::Matrix<double, 6, 1> force_desired;
	force_desired.setZero();
	force_desired(2, 0) = target_mass_ * -9.81;

	//force error
	Eigen::Matrix<double, 6, 1> force_error = force_desired - force_existing;

	//force Integral
	force_error_integral_ += period.toSec() * force_error;
	
	//force Differential
	Eigen::Matrix<double, 6, 1> force_error_diff_filtered;
	if (count_loop_ != 0) { //if period.toMSec = 0.0 the error_diff is infinite
		Eigen::Matrix<double, 6, 1> force_error_diff = (force_error - old_force_error_) / period.toMSec();
		update_force_error_diff_filter(force_error_diff);
		for (int i = 0; i < 6; i++) {
			force_error_diff_filtered(i, 0) = compute_force_error_diff_filtered(i);
		}
	}
	else {
		for (int i = 0; i < 6; i++) {
			force_error_diff_filtered(i, 0) = 0.0;
		}
	}
	old_force_error_ = force_error;
	
	//Pid Force control
	Eigen::Matrix<double, 6, 1> force_command;
	for (int i = 0; i < 6; i++) {
		force_command(i, 0) = k_p_f_[i] * force_error(i,0) + k_i_f_[i] * force_error_integral_(i, 0) + k_d_f_[i] * force_error_diff_filtered(i, 0);
	}

	//Position control
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());

	//Position error
	Eigen::Matrix<double, 6, 1> position_error;
	position_error.head(3) << position - position_desired_;
	// orientation error
	// "difference" quaternion
	if (orientation_desired_.coeffs().dot(orientation.coeffs()) < 0.0) {
		orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_desired_);
	position_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// Transform to base frame
	position_error.tail(3) << -transform.linear() * position_error.tail(3);

	//Position integral
	position_error_integral_ += period.toSec() * position_error;

	//Position Differential
	Eigen::Matrix<double, 6, 1> position_error_diff_filtered;
	if (count_loop_ != 0) { //if period.toMSec = 0.0 the error_diff is infinite
		Eigen::Matrix<double, 6, 1> position_error_diff = (position_error - old_position_error_) / period.toMSec();
		update_position_error_diff_filter(position_error_diff);
		for (int i = 0; i < 6; i++) {
			position_error_diff_filtered(i, 0) = compute_position_error_diff_filtered(i);
		}
	}
	else {
		for (int i = 0; i < 6; i++) {
			position_error_diff_filtered(i, 0) = 0.0;
		}
	}
	old_position_error_ = position_error;

	//Position PID Control
	Eigen::Matrix<double, 6, 1> position_command;
	for (int i = 0; i < 6; i++) {
		position_command(i, 0) = k_p_p_[i] * position_error(i, 0) + k_i_p_[i] * position_error_integral_(i, 0) + k_d_p_[i] * position_error_diff_filtered(i,0);
	}

	//Hybrid Control combines Force and Position commands
	Eigen::Matrix< double, 6, 1> s;
	s << 1, 1, 1, 1, 1, 1; //1 = Position controlled, 0 = force controlled
	Eigen::Matrix< double, 6, 6> compliance_selection_matrix = s.array().sqrt().matrix().asDiagonal();
	Eigen::Matrix< double, 6, 6> unit_matrix = Eigen::Matrix< double, 6, 6>::Identity();

	position_command = compliance_selection_matrix * position_command;
	force_command = (unit_matrix - compliance_selection_matrix) * force_command;

	Eigen::Matrix<double, 6, 1> hybrid_command;
	hybrid_command = position_command + force_command;

	//Convert in 7 joint space
	Eigen::Matrix<double, 7, 1> tau_command;
	tau_command = (jacobian.transpose() * hybrid_command);

	//Filter tau_command
	update_tau_command_filter(tau_command);
	for (int i = 0; i < 7; i++) {
		tau_command(i, 0) = compute_tau_command_filtered(i);
	}

	//Create and fill output array
	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_command;

	//Push data in export_data struct
	//my_data.measured_positions.push_back();
	my_data_.measured_forces.push_back(force_existing);
	my_data_.position_errors.push_back(position_error);
	my_data_.force_errors.push_back(force_error);
	my_data_.position_commands.push_back(position_command);
	my_data_.force_commands.push_back(force_command);


	count_loop_ ++;

	return tau_d_array;
}

void detail::pid_force_control_motion_generator::update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command) {
	for (int i = 0; i < 7; i ++) {
		tau_command_buffer_[tau_command_current_filter_position_ * 7 + i] = tau_command(i, 0);
	}
	tau_command_current_filter_position_ = (tau_command_current_filter_position_ + 1) % tau_command_filter_size_;
}

double detail::pid_force_control_motion_generator::compute_tau_command_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < j * tau_command_filter_size_; i += 7) {
		value += tau_command_buffer_[i];
	}
	return (value / tau_command_filter_size_);
}

void detail::pid_force_control_motion_generator::update_force_error_diff_filter(Eigen::Matrix<double, 6, 1> force_error_diff) {
	for (int i = 0; i < 6; i++) {
		force_error_diff_buffer_[force_error_diff_current_filter_position_ * 6 + i] = force_error_diff(i, 0);
	}
	force_error_diff_current_filter_position_ = (force_error_diff_current_filter_position_ + 1) % force_error_diff_filter_size_;
}

double detail::pid_force_control_motion_generator::compute_force_error_diff_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < j * force_error_diff_filter_size_; i += 6) {
		value += force_error_diff_buffer_[i];
	}
	return (value / force_error_diff_filter_size_);
}

void detail::pid_force_control_motion_generator::update_position_error_diff_filter(Eigen::Matrix<double, 6, 1> position_error_diff) {
	for (int i = 0; i < 6; i++) {
		position_error_diff_buffer_[position_error_diff_current_filter_position_ * 6 + i] = position_error_diff(i, 0);
	}
	position_error_diff_current_filter_position_ = (position_error_diff_current_filter_position_ + 1) % position_error_diff_filter_size_;
}

double detail::pid_force_control_motion_generator::compute_position_error_diff_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < j * position_error_diff_filter_size_; i += 6) {
		value += position_error_diff_buffer_[i];
	}
	return (value / position_error_diff_filter_size_);
}

detail::force_motion_generator::export_data pid_force_control_motion_generator::get_export_data() {
	return my_data_;
}

} /* namespace detail */
} /* namespace franka_proxy */
