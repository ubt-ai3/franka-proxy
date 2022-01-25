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


//Class Pid Force Control Motion Generator

//Constructor
pid_force_control_motion_generator::pid_force_control_motion_generator
(franka::Robot& robot,
	double mass,
	double duration)
	:
	tau_command_buffer(tau_command_filter_size * 7, 0),
	target_mass(mass),
	duration(duration),
	model(robot.loadModel())
{
	initial_state_ = robot.readOnce();
}

franka::Torques pid_force_control_motion_generator::callback
(const franka::RobotState& robot_state,
	franka::Duration period)
{
	time_ += period.toSec();

	if (time_ > duration) //Finished
	{
		// todo this may be wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}

	

	std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Matrix<double, 6, 1> force_command;
	Eigen::Matrix<double, 6, 1> force_desired;
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_existing(robot_state.O_F_ext_hat_K.data());

	//Set desired cartesian Position as the first measured position
	if (count_loop == 0) {
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> d_j_pos(robot_state.q.data());
		desired_cartesian_pos = (jacobian.transpose()).fullPivLu().solve(d_j_pos);
	}

	//Set force_desired
	force_desired.setZero();
	force_desired(2, 0) = target_mass * -9.81;

	//Integral
	force_error_integral += period.toSec() * (force_desired - force_existing);
	

	//Pid Force control
	for (int i = 0; i < 6; i++) {
		force_command(i, 0) = k_p_f[i] * (force_desired(i, 0) - force_existing(i, 0) + k_i_f[i] * force_error_integral(i, 0));
	}

	//Position control
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> j_pos(robot_state.q.data());
	Eigen::Matrix<double, 6, 1> measured_cartesian_pos = (jacobian.transpose()).fullPivLu().solve(j_pos);

	position_error_integral += period.toSec() * (desired_cartesian_pos - measured_cartesian_pos);
	
	Eigen::Matrix<double, 6, 1> position_command;
	for (int i = 0; i < 6; i ++) {
		position_command(i, 0) = k_p_p[i] * (desired_cartesian_pos(i, 0) - measured_cartesian_pos(i, 0)) + k_i_p[i] * position_error_integral(i, 0);
	}

	//Hybrid Control
	Eigen::Matrix< double, 6, 1> s;
	s << 1, 1, 0, 1, 1, 1;
	Eigen::Matrix< double, 6, 6> compliance_selection_matrix = s.array().sqrt().matrix().asDiagonal();

	Eigen::Matrix< double, 6, 1> e;
	e << 1, 1, 1, 1, 1, 1;
	Eigen::Matrix< double, 6, 6> unit_matrix = e.array().sqrt().matrix().asDiagonal();

	position_command = compliance_selection_matrix * position_command;
	force_command = (unit_matrix - compliance_selection_matrix) * force_command;

	Eigen::Matrix<double, 6, 1> hybrid_command;
	hybrid_command = position_command;
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
	my_data.desired_forces.push_back(force_desired);
	my_data.existing_forces.push_back(force_existing);
	my_data.command_forces.push_back(force_command);
	my_data.position_forces.push_back(position_command);
	my_data.hybrid_forces.push_back(hybrid_command);

	count_loop++;

	return tau_d_array;
}

void detail::pid_force_control_motion_generator::update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command) {
	for (int i = 0; i < 7; i ++) {
		tau_command_buffer[tau_command_current_filter_position * 7 + i] = tau_command(i, 0);
	}
	tau_command_current_filter_position = (tau_command_current_filter_position + 1) % tau_command_filter_size;
}

double detail::pid_force_control_motion_generator::compute_tau_command_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < j * tau_command_filter_size; i += 7) {
		value += tau_command_buffer[i];
	}
	return (value / tau_command_filter_size);
}


detail::force_motion_generator::export_data pid_force_control_motion_generator::get_export_data() {
	return my_data;
}

} /* namespace detail */
} /* namespace franka_proxy */
