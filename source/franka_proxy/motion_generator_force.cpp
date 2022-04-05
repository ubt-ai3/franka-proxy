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
//------------------Class Hybrid Control Motion Generator---------------------------------------
//-------------------------------------------------------------------------------------------------

//Constructor
hybrid_control_motion_generator::hybrid_control_motion_generator
(franka::Robot& robot,
	std::vector<Eigen::Vector3d> desired_positions,
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces,
	std::vector<Eigen::Quaterniond> desired_orientations,
	std::array<std::array<double, 6>, 6> control_parameters,
	csv_data& data)
	:
	tau_command_buffer_(tau_command_filter_size_ * 7, 0),
	force_error_diff_buffer_(force_error_diff_filter_size_ * 6, 0),
	position_error_diff_buffer_(position_error_diff_filter_size_ * 6, 0),
	model_(robot.loadModel()),
	stiffness_(6, 6),
	damping_(6, 6),
	dq_buffer_(dq_filter_size_, eigen_vector7d::Zero()),
	desired_positions_(desired_positions),
	desired_forces_(desired_forces),
	desired_orientations_(desired_orientations),
	k_p_p_(control_parameters[0]),
	k_i_p_(control_parameters[1]),
	k_d_p_(control_parameters[2]),
	k_p_f_(control_parameters[3]),
	k_i_f_(control_parameters[4]),
	k_d_f_(control_parameters[5]),
	data_(data)
{
	initial_state_ = robot.readOnce();

	//Set Stiffness and Damping matrices for springer damping position control
	stiffness_.setZero();
	stiffness_.topLeftCorner(3, 3) =
		translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
	stiffness_.bottomRightCorner(3, 3) =
		rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);

	damping_.setZero();
	damping_.topLeftCorner(3, 3) =
		.5 * sqrt(translational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
	damping_.bottomRightCorner(3, 3) =
		.5 * sqrt(rotational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
}

franka::Torques hybrid_control_motion_generator::callback
(const franka::RobotState& robot_state,
	franka::Duration period)
{
	time_ += period.toSec();

	if (count_loop_ == desired_positions_.size()) //Finished
	{
		// todo this may be wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}

	if (count_loop_ == 0) {
		data_.k_p_p = k_p_p_;
		data_.k_i_p = k_i_p_;
		data_.k_d_p = k_d_p_;
		data_.k_p_f = k_p_f_;
		data_.k_i_f = k_i_f_;
		data_.k_d_f = k_d_f_;
	}

	std::array<double, 42> jacobian_array = model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	data_.zero_jacobian.push_back(jacobian);
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_existing(robot_state.O_F_ext_hat_K.data());
	data_.o_F_ext_hat_K.push_back(force_existing);

	//Set desired cartesian Position as the first measured position (initial)
	/*if (count_loop_ == 0) {
		data_.duration = duration_;

		Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data()));
		position_desired_ = initial_transform.translation();
		position_desired_ = { 0.6, 0, 0.3 };
		orientation_desired_ = initial_transform.linear();
		orientation_desired_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
		std::cout << "orientation_desired.w: " << orientation_desired_.w() << std::endl;
		std::cout << "orientation_desired.vec: " << orientation_desired_.vec() << std::endl;
		std::cout << "position_desired_: " << position_desired_ << std::endl;
	}*/

	//Set force_desired
	Eigen::Matrix<double, 6, 1> force_desired = desired_forces_[count_loop_];
	data_.force_desired.push_back(force_desired);

	//force error
	Eigen::Matrix<double, 6, 1> force_error = force_desired - force_existing;
	data_.force_error.push_back(force_error);	

	//force Integral
	force_error_integral_ += period.toSec() * force_error;
	data_.force_error_integral.push_back(force_error_integral_);
	
	//force Differential
	Eigen::Matrix<double, 6, 1> force_error_diff_filtered;
	if (count_loop_ != 0) { //if period.toMSec = 0.0 the error_diff is infinite
		Eigen::Matrix<double, 6, 1> force_error_diff = (force_error - old_force_error_) * 1000.0; //period.toMSec()
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
	data_.force_error_diff_filtered.push_back(force_error_diff_filtered);
	
	//Pid Force control
	Eigen::Matrix<double, 6, 1> force_command;
	Eigen::Matrix<double, 6, 1> force_command_p;
	Eigen::Matrix<double, 6, 1> force_command_i;
	Eigen::Matrix<double, 6, 1> force_command_d;
	for (int i = 0; i < 6; i++) {
		force_command_p(i, 0) = k_p_f_[i] * force_error(i, 0);
		force_command_i(i, 0) = k_i_f_[i] * force_error_integral_(i, 0);
		force_command_d(i, 0) = k_d_f_[i] * force_error_diff_filtered(i, 0);

		force_command(i, 0) = force_command_p(i, 0) + force_command_i(i, 0) + force_command_d(i, 0);

	}
	data_.force_command.push_back(force_command);
	data_.force_command_p.push_back(force_command_p);
	data_.force_command_i.push_back(force_command_i);
	data_.force_command_d.push_back(force_command_d);

	

	//Position control
	update_dq_filter(robot_state);

	//Set desired position
	orientation_desired_ = desired_orientations_[count_loop_];
	position_desired_ = desired_positions_[count_loop_];

	//Current position
	auto current_pose = model_.pose(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data())); //was used in new spring damper control
	//Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); //was used in old PID control
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());


	Eigen::Matrix<double, 6, 1> pos;
	pos(0, 0) = position(0);
	pos(1, 0) = position(1);
	pos(2, 0) = position(2);
	pos(3, 0) = orientation.coeffs()(0);
	pos(4, 0) = orientation.coeffs()(1);
	pos(5, 0) = orientation.coeffs()(2);

	data_.position.push_back(pos);


	//Position error
	Eigen::Matrix<double, 6, 1> position_error;
	position_error.head(3) = position - position_desired_;

	//Orientation error
	if (orientation_desired_.coeffs().dot(orientation.coeffs()) < 0.0) {
		orientation.coeffs() = -orientation.coeffs(); //orientation.coeffs() << -orientation.coeffs(); bei pid control
	}
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_desired_);
	position_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// transform to base frame
	position_error.tail(3) = -transform.linear() * position_error.tail(3); //-transform_d.linear() * error.tail(3);
	data_.position_error.push_back(position_error);


	//Position integral
	position_error_integral_ += period.toSec() * position_error;
	data_.position_error_integral.push_back(position_error_integral_);

	//Position Differential
	Eigen::Matrix<double, 6, 1> position_error_diff_filtered;
	if (count_loop_ != 0) { //if period.toMSec = 0.0 the error_diff is infinite
		Eigen::Matrix<double, 6, 1> position_error_diff = (position_error - old_position_error_) * 1000.0; //period.toMSec()
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
	data_.position_error_diff_filtered.push_back(position_error_diff_filtered);

	// spring damper system with damping ratio=1 and filtered dq
	Eigen::Matrix<double, 6, 1> position_command_sd = -stiffness_ * position_error - damping_ * (jacobian * compute_dq_filtered());

	//Position PID Control
	Eigen::Matrix<double, 6, 1> position_command_pid;
	Eigen::Matrix<double, 6, 1> position_command_p;
	Eigen::Matrix<double, 6, 1> position_command_i;
	Eigen::Matrix<double, 6, 1> position_command_d;
	for (int i = 0; i < 6; i++) {
		position_command_p(i, 0) = k_p_p_[i] * position_error(i, 0);
		position_command_i(i, 0) = k_i_p_[i] * position_error_integral_(i, 0);
		position_command_d(i, 0) = k_d_p_[i] * position_error_diff_filtered(i, 0);
		position_command_pid(i, 0) = position_command_p(i, 0) + position_command_i(i, 0) + position_command_d(i, 0);
	}
	data_.position_command.push_back(position_command_pid);
	data_.position_command_p.push_back(position_command_p);
	data_.position_command_i.push_back(position_command_i);
	data_.position_command_d.push_back(position_command_d);

	//Hybrid Control combines Force and Position commands
	Eigen::Matrix< double, 6, 1> s;
	s << 1, 1, 0, 1, 1, 1; //1 = Position controlled, 0 = force controlled
	Eigen::Matrix< double, 6, 6> compliance_selection_matrix = s.array().matrix().asDiagonal();
	Eigen::Matrix< double, 6, 6> unit_matrix = Eigen::Matrix< double, 6, 6>::Identity();
	

	Eigen::Matrix<double, 6, 1> position_command = position_command_pid; //Change here to reimplement PID Position Control
	position_command = compliance_selection_matrix * position_command;
	force_command = (unit_matrix - compliance_selection_matrix) * force_command;

	Eigen::Matrix<double, 6, 1> hybrid_command;
	hybrid_command = position_command + force_command;

	//Convert in 7 joint space
	Eigen::Matrix<double, 7, 1> tau_command;
	tau_command = (jacobian.transpose() * hybrid_command);
	data_.tau_command.push_back(tau_command);

	//Coriolis
	std::array<double, 7> coriolis_array = model_.coriolis(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
	data_.coriolis.push_back(coriolis);

	tau_command += coriolis;

	//Filter tau_command
	update_tau_command_filter(tau_command);
	for (int i = 0; i < 7; i++) {
		tau_command(i, 0) = compute_tau_command_filtered(i);
	}
	data_.tau_command_filtered.push_back(tau_command);

	//Create and fill output array
	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_command;

	count_loop_ ++;

	return tau_d_array;
}

void hybrid_control_motion_generator::update_dq_filter(const franka::RobotState& robot_state)
{
	dq_buffer_[dq_current_filter_position_] = Eigen::Matrix<double,7,1>(robot_state.dq.data());

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


Eigen::Matrix<double, 7, 1> hybrid_control_motion_generator::compute_dq_filtered()
{
	eigen_vector7d value(eigen_vector7d::Zero());

	for (size_t i = 0; i < dq_filter_size_; ++i)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}

void detail::hybrid_control_motion_generator::update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command) {
	for (int i = 0; i < 7; i ++) {
		tau_command_buffer_[tau_command_current_filter_position_ * 7 + i] = tau_command(i, 0);
	}
	tau_command_current_filter_position_ = (tau_command_current_filter_position_ + 1) % tau_command_filter_size_;
}

double detail::hybrid_control_motion_generator::compute_tau_command_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < 7 * tau_command_filter_size_; i += 7) {
		value += tau_command_buffer_[i];
	}
	return (value / tau_command_filter_size_);
}

void detail::hybrid_control_motion_generator::update_force_error_diff_filter(Eigen::Matrix<double, 6, 1> force_error_diff) {
	for (int i = 0; i < 6; i++) {
		force_error_diff_buffer_[force_error_diff_current_filter_position_ * 6 + i] = force_error_diff(i, 0);
	}
	force_error_diff_current_filter_position_ = (force_error_diff_current_filter_position_ + 1) % force_error_diff_filter_size_;
}

double detail::hybrid_control_motion_generator::compute_force_error_diff_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < 6 * force_error_diff_filter_size_; i += 6) {
		value += force_error_diff_buffer_[i];
	}
	return (value / force_error_diff_filter_size_);
}

void detail::hybrid_control_motion_generator::update_position_error_diff_filter(Eigen::Matrix<double, 6, 1> position_error_diff) {
	for (int i = 0; i < 6; i++) {
		position_error_diff_buffer_[position_error_diff_current_filter_position_ * 6 + i] = position_error_diff(i, 0);
	}
	position_error_diff_current_filter_position_ = (position_error_diff_current_filter_position_ + 1) % position_error_diff_filter_size_;
}

double detail::hybrid_control_motion_generator::compute_position_error_diff_filtered(int j) {
	double value = 0.0;
	for (int i = j; i < 6 * position_error_diff_filter_size_; i += 6) {
		value += position_error_diff_buffer_[i];
	}
	return (value / position_error_diff_filter_size_);
}


} /* namespace detail */
} /* namespace franka_proxy */
