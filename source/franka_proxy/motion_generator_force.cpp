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
	double duration,
	double k_p,
	double k_i,
	double k_d)
	:
	dq_d_({ 0., 0., 0., 0., 0., 0., 0. }),
	dq_buffer_(dq_filter_size_ * 7, 0),
	target_mass(mass),
	duration(duration),
	k_p(k_p),
	k_i(k_i),
	k_d(k_d),
	model(robot.loadModel()),
	tau_old_error(Eigen::Matrix<double, 7, 1>()),
	tau_new_error(Eigen::Matrix<double, 7, 1>())
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

	Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

	std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());


	Eigen::Matrix<double, 6, 1> desired_force_torque;

	Eigen::Matrix<double, 7, 1> tau_existing;
	Eigen::Matrix<double, 7, 1> tau_desired;
	Eigen::Matrix<double, 7, 1> tau_command;
	Eigen::Matrix<double, 7, 1> tau_J_d;
	Eigen::Matrix<double, 7, 1> tau_error_differential;
	Eigen::Matrix<double, 7, 1> tau_error_differential_sum; //Summe von den aufeinderfolgenden diff werten der errors
	Eigen::Matrix<double, 7, 1> tau_error_differential_filtered; //tau_error_differentials_sum wird durch die anzahl der Punkte geteilt, aus denen sich die Summe zusammensetzt

	//Setze gewünschte Drehmomente in Gelenken: Alle Richtungen gleich 0 außer z-Richtung
	desired_force_torque.setZero();
	desired_force_torque(2, 0) = target_mass * -9.81;


	tau_existing = tau_measured - gravity;
	tau_desired = jacobian.transpose() * desired_force_torque;
	tau_new_error = tau_desired - tau_existing;

	tau_error_integral += period.toSec() * tau_new_error;

	//calculate differential of error
	tau_error_differential = (tau_new_error - tau_old_error) / (0.001);
	tau_old_error = tau_new_error;
	points_derivative[count_loop % number_of_points_derivative] = tau_error_differential;	


	//Calculate values for the D Control
	for (int i = 0; i < number_of_points_derivative; i++) {
		tau_error_differential_sum += points_derivative[i];
	}
	if (count_loop < number_of_points_derivative) { //die ersten number_of_points_derivative werden auf 0 gesetzt
		tau_error_differential_filtered.setZero();
	}
	else {
		tau_error_differential_filtered = tau_error_differential_sum / number_of_points_derivative;
	}



	//Todo:
	//force_command = k_p * (force_desired - force_existing) + k_i * force_error_integral + k_d * force_differential_filtered;
	//force_command = (0, 0, -9.81, 0, 0, 0);
	//tau_command = jacobian.transpose() * force_command;
	//tau_command += tau_gravity;


	//New Part
	/*Eigen::Matrix<double, 6, 1> force_desired;
	Eigen::Matrix<double, 6, 1> force_command;
	Eigen::Matrix<double, 6, 1> force_error_integral;
	force_error_integral.setZero();
	Eigen::Matrix<double, 6, 1> force_differential_filtered;
	force_differential_filtered.setZero();

	Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_meausured(robot_state.O_F_ext_hat_K.data());

	force_desired.setZero();
	force_desired(2, 0) = -9.81 * target_mass;

	force_command = force_desired + k_p * (force_desired - force_meausured) + k_i * force_error_integral + k_d * force_differential_filtered;
	
	Eigen::Matrix<double, 7, 1> tau_command_new;
	tau_command_new = jacobian.transpose() * force_command;

	std::array<double, 7> tau_d_array_new{};
	Eigen::VectorXd::Map(&tau_d_array_new[0], 7) = tau_command_new;*/
	//End new part


	//FF? + PID-control
	//tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral + k_d * tau_error_differential_filtered;
	tau_command = k_p * (tau_desired - tau_existing) + k_i * tau_error_integral + k_d * tau_error_differential_filtered;
	
	// updateDQFilter
	update_dq_filter(robot_state);

	// compute torques according to impedance control law
	// with assumption mass = 0 (robot state does not provide ddq and own measurement too noisy)
	for (int i = 0; i < 7; i++)
		tau_J_d[i] = K_P_[i] * (robot_state.q_d[i] - robot_state.q[i]) + K_D_[i] * (dq_d_[i] - compute_dq_filtered(i));


	//the final output array
	std::array<double, 7> tau_d_array{};
	//Todo remove the following because this results in a constant error
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = (tau_command + tau_J_d) * 0.5;

	
	//Schreibe Werte in die lokalen privaten Variablen Arrays
	desired_forces.push_back(desired_force_torque); //Bereits in 6 Weltkoordinaten
	// A * x = b muss gelöst werden. A = J^T, x = F, b = tau. x ist der interessierende Vektor, der über verschiedene lineare Lösungssolver ermittelt werden kann
	measured_forces.push_back((jacobian.transpose()).fullPivLu().solve(tau_existing)); 
	command_forces.push_back((jacobian.transpose()).fullPivLu().solve(tau_command));
	force_errors.push_back((jacobian.transpose()).fullPivLu().solve(tau_new_error));
	force_errors_integral.push_back((jacobian.transpose()).fullPivLu().solve(tau_error_integral));
	force_errors_differential.push_back((jacobian.transpose()).fullPivLu().solve(tau_error_differential));
	force_errors_differential_sum.push_back((jacobian.transpose()).fullPivLu().solve(tau_error_differential_sum));
	force_errors_differential_filtered.push_back((jacobian.transpose()).fullPivLu().solve(tau_error_differential_filtered));


	count_loop++;

	/*if (count_loop & 100 == 0) {
		std::cout << "tau_d_array[0] = " << tau_d_array[0] << "tau_d_array_new[0]" << tau_d_array_new[0] << std::endl;
	}*/

	return tau_d_array;
}


//Get data vector functions
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_measured_forces() {
	return measured_forces;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_desired_forces() {
	return desired_forces;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_command_forces() {
	return command_forces;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_force_errors() {
	return force_errors;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_force_errors_integral() {
	return force_errors_integral;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_force_errors_differential() {
	return force_errors_differential;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_force_errors_differential_sum() {
	return force_errors_differential_sum;
}
std::vector<Eigen::Matrix<double, 6, 1>> pid_force_control_motion_generator::give_force_errors_differential_filtered() {
	return force_errors_differential_filtered;
}


void pid_force_control_motion_generator::update_dq_filter(const franka::RobotState& robot_state)
{
	for (int i = 0; i < 7; i++)
		dq_buffer_[dq_current_filter_position_ * 7 + i] = robot_state.dq[i];

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


double pid_force_control_motion_generator::compute_dq_filtered(int j)
{
	double value = 0.0;
	for (size_t i = j; i < 7 * dq_filter_size_; i += 7)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}


} /* namespace detail */
} /* namespace franka_proxy */
