/**
 *************************************************************************
 *
 * @file franka_motion_generator.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/

#include "franka_motion_generator.hpp"
#include <franka/model.h>
#include <utility>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

namespace franka_proxy
{
namespace detail
{


//////////////////////////////////////////////////////////////////////////
//
// franka_motion_generator
//
//////////////////////////////////////////////////////////////////////////


franka_joint_motion_generator::franka_joint_motion_generator
(double speed_factor, const std::array<double, 7> q_goal,
	std::mutex& current_state_lock,
	franka::RobotState& current_state,
	const std::atomic_bool& stop_motion_flag,
	bool stop_on_contact)
	:
	q_goal_(q_goal.data()),

	current_state_lock_(current_state_lock),
	current_state_(current_state),

	stop_motion_(stop_motion_flag),
	stop_on_contact_(stop_on_contact)
{
	dq_max_ *= speed_factor;
	ddq_max_start_ *= speed_factor;
	ddq_max_goal_ *= speed_factor;
	dq_max_sync_.setZero();
	q_start_.setZero();
	delta_q_.setZero();
	t_1_sync_.setZero();
	t_2_sync_.setZero();
	t_f_sync_.setZero();
	q_1_.setZero();
}


bool franka_joint_motion_generator::calculateDesiredValues
(double t, Vector7d* delta_q_d) const
{
	Vector7i sign_delta_q;
	for (int i = 0; i < 7; i++)
	{
		if (delta_q_[i] < 0)
			sign_delta_q[i] = -1;
		else
			sign_delta_q[i] = 1;
	}

	Vector7d t_d = t_2_sync_ - t_1_sync_;
	Vector7d delta_t_2_sync_ = t_f_sync_ - t_2_sync_;
	std::array<bool, 7> joint_motion_finished{};

	for (size_t i = 0; i < 7; i++)
	{
		if (std::abs(delta_q_[i]) < kDeltaQMotionFinished)
		{
			(*delta_q_d)[i] = 0;
			joint_motion_finished[i] = true;
		}
		else
		{
			if (t < t_1_sync_[i])
			{
				(*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
					(0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
			}
			else if (t >= t_1_sync_[i] && t < t_2_sync_[i])
			{
				(*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
			}
			else if (t >= t_2_sync_[i] && t < t_f_sync_[i])
			{
				(*delta_q_d)[i] = delta_q_[i] +
					0.5 *
					(1.0 / std::pow(delta_t_2_sync_[i], 3.0) *
					(t - t_1_sync_[i] - 2.0 * delta_t_2_sync_[i] - t_d[i]) *
						std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
						(2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync_[i] - 2.0 * t_d[i])) *
					dq_max_sync_[i] * sign_delta_q[i];
			}
			else
			{
				(*delta_q_d)[i] = delta_q_[i];
				joint_motion_finished[i] = true;
			}
		}
	}
	return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
		[](bool x) { return x; });
}


void franka_joint_motion_generator::calculateSynchronizedValues()
{
	Vector7d dq_max_reach(dq_max_);
	Vector7d t_f = Vector7d::Zero();
	Vector7d delta_t_2 = Vector7d::Zero();
	Vector7d t_1 = Vector7d::Zero();
	Vector7d delta_t_2_sync = Vector7d::Zero();
	Vector7i sign_delta_q;
	for (int i = 0; i < 7; i++)
	{
		if (delta_q_[i] < 0)
			sign_delta_q[i] = -1;
		else
			sign_delta_q[i] = 1;
	}

	for (size_t i = 0; i < 7; i++)
	{
		if (std::abs(delta_q_[i]) > kDeltaQMotionFinished)
		{
			if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
				3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i])))
			{
				dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
					(ddq_max_start_[i] * ddq_max_goal_[i]) /
					(ddq_max_start_[i] + ddq_max_goal_[i]));
			}
			t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
			delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
			t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
		}
	}
	double max_t_f = t_f.maxCoeff();
	for (size_t i = 0; i < 7; i++)
	{
		if (std::abs(delta_q_[i]) > kDeltaQMotionFinished)
		{
			double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
			double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
			double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
			double delta = b * b - 4.0 * a * c;
			if (delta < 0.0)
			{
				delta = 0.0;
			}
			dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
			t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
			delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
			t_f_sync_[i] =
				(t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
			t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
			q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
		}
	}
}


bool franka_joint_motion_generator::colliding(const franka::RobotState& state)
{
	for (double v : state.joint_contact)
		if (v > 0) return true;
	for (double v : state.cartesian_contact)
		if (v > 0) return true;
	return false;
}


franka::JointPositions franka_joint_motion_generator::operator()
	(const franka::RobotState& robot_state, franka::Duration period)
{
	time_ += period.toSec();

	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();

	if (stop_on_contact_ && colliding(robot_state))
		throw contact_stop_trigger();


	if (time_ == 0.0)
	{
		q_start_ = Vector7d(robot_state.q_d.data());
		delta_q_ = q_goal_ - q_start_;
		calculateSynchronizedValues();
	}

	Vector7d delta_q_d;
	bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

	std::array<double, 7> joint_positions{};
	Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
	franka::JointPositions output(joint_positions);
	output.motion_finished = motion_finished;
	return output;
}




//////////////////////////////////////////////////////////////////////////
//
// force_motion_generator
//
//////////////////////////////////////////////////////////////////////////


force_motion_generator::force_motion_generator(
	franka::Robot& robot,
	double mass,
	double duration)
	: model(robot.loadModel()),
	target_mass(mass),
	duration(duration),
	dq_d_({ 0.,0.,0.,0.,0.,0.,0. }),
	dq_buffer_(dq_filter_size_ * 7, 0)
{
	initial_state_ = robot.readOnce();
}


franka::Torques force_motion_generator::callback(const franka::RobotState& robot_state, franka::Duration period)
{
	time_ += period.toSec();

	if (time_ > duration)
	{
		// todo this may be wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}


	Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());

	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

	std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());


	Eigen::VectorXd desired_force_torque(6), tau_existing(7), tau_desired(7), tau_command(7), tau_J_d(7);


	desired_force_torque.setZero();
	desired_force_torque(2) = desired_mass * -9.81;


	tau_existing = tau_measured - gravity;
	tau_desired = jacobian.transpose() * desired_force_torque;
	tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	// FF + PI control
	tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;


	// updateDQFilter
	update_dq_filter(robot_state);

	// compute torques according to impedance control law
	// with assumption mass = 0 (robot state does not provide ddq and own measurement too noisy)
	for (size_t i = 0; i < 7; i++)
		tau_J_d[i] = (K_P_[i] * (robot_state.q_d[i] - robot_state.q[i]) + K_D_[i] * (dq_d_[i] - compute_dq_filtered(i)));


	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = (tau_command + tau_J_d) * 0.5;


	forces_z.push_back(robot_state.O_F_ext_hat_K[2]);


	return tau_d_array;
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




//////////////////////////////////////////////////////////////////////////
//
// cartesian_impedance_controller
//
//////////////////////////////////////////////////////////////////////////


cartesian_impedance_controller::cartesian_impedance_controller
(franka::Robot& robot,
	double translational_stiffness,
	double rotational_stiffness)
	:
	model(robot.loadModel()),
	initial_state_(robot.readOnce()),
	initial_transform_(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data())),
	position_d_(initial_transform_.translation()),
	orientation_d_(initial_transform_.linear()),
	stiffness_(6, 6),
	damping_(6, 6)
{
	stiffness_.setZero();
	stiffness_.topLeftCorner(3, 3) <<
		translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	stiffness_.bottomRightCorner(3, 3) <<
		rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	damping_.setZero();
	damping_.topLeftCorner(3, 3) <<
		2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
	damping_.bottomRightCorner(3, 3) <<
		2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
}


franka::Torques cartesian_impedance_controller::callback
(const franka::RobotState& robot_state,
	franka::Duration)
{
	Eigen::Affine3d pose_d(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
	position_d_ = pose_d.translation();
	orientation_d_ = pose_d.linear();

	// get state variables
	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

	// convert to Eigen
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());

	// compute error to desired equilibrium pose
	// position error
	Eigen::Matrix<double, 6, 1> error;
	error.head(3) << position - position_d_;

	// orientation error
	// "difference" quaternion
	if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
	{
		orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// Transform to base frame
	error.tail(3) << -transform.linear() * error.tail(3);

	// compute control
	Eigen::VectorXd tau_task(7), tau_d(7);

	// Spring damper system with damping ratio=1
	tau_task << jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));
	tau_d << tau_task + coriolis;

	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
	return tau_d_array;
}


//////////////////////////////////////////////////////////////////////////
//
// cartesian_impedance_and_force_controller
//
//////////////////////////////////////////////////////////////////////////


cartesian_impedance_and_force_controller::cartesian_impedance_and_force_controller
(franka::Robot& robot,
	double translational_stiffness,
	double rotational_stiffness)
	:
	model(robot.loadModel()),
	initial_state_(robot.readOnce()),
	initial_transform_(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data())),
	position_d_(initial_transform_.translation()),
	orientation_d_(initial_transform_.linear()),
	stiffness_(6, 6),
	damping_(6, 6),
	target_mass(0.5)
{
	stiffness_.setZero();
	stiffness_.topLeftCorner(3, 3) <<
		translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	stiffness_.bottomRightCorner(3, 3) <<
		rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
	damping_.setZero();
	damping_.topLeftCorner(3, 3) <<
		2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
	damping_.bottomRightCorner(3, 3) <<
		2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
}


franka::Torques cartesian_impedance_and_force_controller::callback
(const franka::RobotState& robot_state,
	franka::Duration period)
{
	Eigen::Affine3d pose_d(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
	position_d_ = pose_d.translation();
	orientation_d_ = pose_d.linear();

	// get state variables
	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

	// convert to Eigen
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());

	// compute error to desired equilibrium pose
	// position error
	Eigen::Matrix<double, 6, 1> error;
	error.head(3) << position - position_d_;

	// orientation error
	// "difference" quaternion
	if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
	{
		orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// Transform to base frame
	error.tail(3) << -transform.linear() * error.tail(3);

	// compute control
	Eigen::VectorXd tau_task(7), tau_d(7);

	Eigen::Matrix<double, 6, 1> cart_pos_xy_error = error;
	cart_pos_xy_error[2] = cart_pos_xy_error[3] = cart_pos_xy_error[4] = 0;
	Eigen::Matrix<double, 6, 1> cart_vel_xy_error = jacobian * (dq_d - dq);
	cart_vel_xy_error[2] = cart_vel_xy_error[3] = cart_vel_xy_error[4] = 0;
	// Spring damper system with damping ratio=1
	Eigen::VectorXd desired_force_torque_cartesian_impedance = (-stiffness_ * cart_pos_xy_error + damping_ * cart_vel_xy_error);

	//static int i = 0;
	//if (++i % 100 == 0)
	//	std::cout << desired_force_torque_cartesian_impedance.transpose() << std::endl;




	Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());

	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());


	Eigen::VectorXd desired_force_torque_z_force(6), tau_existing(7), tau_desired(7), tau_command(7), tau_J_d(7);


	desired_force_torque_z_force.setZero();
	desired_force_torque_z_force(2) = desired_mass * -9.81;


	tau_existing = tau_measured - gravity;
	tau_existing;
	tau_desired = jacobian.transpose() * desired_force_torque_z_force;
	tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	// FF + PI control
	tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;


	forces_z.push_back(robot_state.O_F_ext_hat_K[2]);


	tau_task << (jacobian.transpose() * desired_force_torque_cartesian_impedance);/*  + tau_command) * 0.5;*/
	tau_d << tau_task + coriolis;


	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
	return tau_d_array;
}



//////////////////////////////////////////////////////////////////////////
//
// sequence_joint_position_motion_generator
//
//////////////////////////////////////////////////////////////////////////



sequence_joint_position_motion_generator::sequence_joint_position_motion_generator
(double speed_factor,
	std::vector<std::array<double, 7>> q_sequence,
	std::mutex& current_state_lock,
	franka::RobotState& current_state,
	const std::atomic_bool& stop_motion_flag)
	:
	q_sequence_(std::move(q_sequence)),
	current_state_lock_(current_state_lock),
	current_state_(current_state),
	stop_motion_(stop_motion_flag)
{ }


franka::JointPositions sequence_joint_position_motion_generator::operator()
	(const franka::RobotState& robot_state,
		franka::Duration period)
{
	time_ += period.toSec();

	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();  // NOLINT(hicpp-exception-baseclass)


	// start motion 
	if (time_ == 0.0)
		if ((Vector7d(robot_state.q_d.data()) - Vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");


	auto step = static_cast<unsigned int>(time_ * 1000.);
	// finish motion
	if (step >= q_sequence_.size())
	{
		franka::JointPositions output(q_sequence_.back());
		output.motion_finished = true;
		return output;
	}


	// motion
	franka::JointPositions output(q_sequence_[step]);
	output.motion_finished = false;
	return output;
}


sequence_joint_velocity_motion_generator::sequence_joint_velocity_motion_generator
(double speed_factor,
	std::vector<std::array<double, 7>> q_sequence,
	std::mutex& current_state_lock,
	franka::RobotState& current_state,
	const std::atomic_bool& stop_motion_flag)
	:
	q_sequence_(std::move(q_sequence)),
	current_state_lock_(current_state_lock),
	current_state_(current_state),
	stop_motion_(stop_motion_flag)
{ }


franka::JointVelocities sequence_joint_velocity_motion_generator::operator()
	(const franka::RobotState& robot_state,
		franka::Duration period)
{
	time_ += period.toSec();

	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();  // NOLINT(hicpp-exception-baseclass)


	// start motion 
	if (time_ == 0.0)
	{
		if ((Vector7d(robot_state.q_d.data()) - Vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");

		return franka::JointVelocities({ 0.,0.,0.,0.,0.,0.,0. });
	}

	auto step = static_cast<unsigned int>(time_ * 1000.);
	// finish motion
	if (step >= q_sequence_.size())
	{
		franka::JointVelocities output({ 0.,0.,0.,0.,0.,0.,0. });
		output.motion_finished = true;
		return output;
	}

	if (period.toMSec() < 1)
		throw("Period under 1ms.");

	// motion
	Vector7d q_(robot_state.q.data());
	Vector7d q_seq(q_sequence_[step].data());

	std::array<double, 7> vel{};
	Eigen::VectorXd::Map(&vel[0], 7) = k_p_ * (q_seq - q_);// *period.toMSec(); // todo hack

	franka::JointVelocities output(vel);
	output.motion_finished = false;
	return output;
}


sequence_cartesian_velocity_motion_generator::sequence_cartesian_velocity_motion_generator
(double speed_factor,
	std::vector<std::array<double, 7>> q_sequence,
	std::mutex& current_state_lock,
	franka::Robot& robot,
	const std::atomic_bool& stop_motion_flag)
	:
	model(robot.loadModel()),
	q_sequence_(std::move(q_sequence)),
	current_state_lock_(current_state_lock),
	current_state_(robot.readOnce()),
	stop_motion_(stop_motion_flag)
{ }

sequence_cartesian_velocity_motion_generator::~sequence_cartesian_velocity_motion_generator() = default;

franka::CartesianVelocities sequence_cartesian_velocity_motion_generator::operator()
	(const franka::RobotState& robot_state,
		franka::Duration period)
{
	time_ += period.toSec();

	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();  // NOLINT(hicpp-exception-baseclass)


	// start motion 
	if (time_ == 0.0)
	{
		if ((Vector7d(robot_state.q_d.data()) - Vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");

		return franka::CartesianVelocities({ 0.,0.,0.,0.,0.,0. });
	}

	auto step = static_cast<unsigned int>(time_ * 1000.);
	// finish motion
	if (step >= q_sequence_.size())
	{
		franka::CartesianVelocities output({ 0.,0.,0.,0.,0.,0. });
		output.motion_finished = true;
		return output;
	}

	// todo remove
	if (period.toMSec() < 1)
		throw std::exception("Period under 1ms.");

	// motion
	Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	//auto current_pose = model.pose(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
	//Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(current_pose.data()));
	pose_log_.emplace_back(current_transform);
	Eigen::Vector3d position(current_transform.translation());
	Eigen::Quaterniond orientation(current_transform.linear());

	// calculate pose from desired joints 
	auto desired_pose = model.pose(franka::Frame::kEndEffector, q_sequence_[step], robot_state.F_T_EE, robot_state.EE_T_K);

	Eigen::Affine3d desired_transform(Eigen::Matrix4d::Map(desired_pose.data()));
	pose_d_log_.emplace_back(desired_transform);
	Eigen::Vector3d position_d(desired_transform.translation());
	Eigen::Quaterniond orientation_d(desired_transform.linear());


	// compute error to desired pose
	Eigen::Matrix<double, 6, 1> error;


	// position error
	error.head(3) = position - position_d;


	// orientation error
	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
		orientation.coeffs() = -orientation.coeffs();
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();


	// Transform to base frame
	error.tail(3) = -desired_transform.linear() * error.tail(3);


	std::array<double, 6> vel{};
	Eigen::VectorXd::Map(&vel[0], 6) = -k_p_ * error;

	error_log_.emplace_back(vel);

	franka::CartesianVelocities output(vel);
	output.motion_finished = false;
	return output;
}


seq_cart_vel_tau_generator::seq_cart_vel_tau_generator
(std::mutex& current_state_lock,
	franka::RobotState& current_state,
	franka::Robot& robot,
	const std::atomic_bool& stop_motion_flag,
	std::vector<std::array<double, 7>> q_sequence,
	std::vector<std::array<double, 6>> f_sequence,
	std::vector<std::array<double, 6>> selection_vector_sequence)
	:
	model(robot.loadModel()),
	q_sequence_(std::move(q_sequence)),
	f_sequence_(std::move(f_sequence)),
	selection_vector_sequence_(std::move(selection_vector_sequence)),
	current_state_lock_(current_state_lock),
	current_state_(current_state),
	stop_motion_(stop_motion_flag),
	stiffness_(6, 6),
	damping_(6, 6),
	dq_buffer_(dq_filter_size_, eigen_vector7d::Zero()),
	ft_buffer_(ft_filter_size_, Eigen::Matrix<double, 6, 1>::Zero()),
	fts_()
{
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


seq_cart_vel_tau_generator::~seq_cart_vel_tau_generator()
{
	if (log_)
	{
		{
			std::ofstream csv("motion_log.csv");

			csv << "f0,f1,f2,e0,e1,e2,e3,e4,e5\n";
			for (int i = 0; i < ft_existing_log_.size(); ++i)
				csv << ft_existing_log_[i][0] << ","
				<< ft_existing_log_[i][1] << ","
				<< ft_existing_log_[i][2] << "," 
				<< error_log_[i][0] << ","
				<< error_log_[i][1] << ","
				<< error_log_[i][2] << ","
				<< error_log_[i][3] << ","
				<< error_log_[i][4] << ","
				<< error_log_[i][5] << "\n";
		}
		std::cout << "did logging" << std::endl;
	}
}


franka::Torques seq_cart_vel_tau_generator::step(const franka::RobotState& robot_state, franka::Duration period)
{
	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();  // NOLINT(hicpp-exception-baseclass)

	
	bool contact_change_motion = false;
	

	time_ += period.toSec();
	bool motion_finished = false;
	auto current_step = static_cast<size_t>(time_ * 1000.);


	// q_start
	if (time_ == 0.0)
	{
		if ((eigen_vector7d(robot_state.q.data())
			- eigen_vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");
	}

	// q_end
	if (current_step >= q_sequence_.size())
	{
		current_step = q_sequence_.size() - 1;
		motion_finished = true;
	}

	
	// get target variables
	std::array<double, 7> q_d = q_sequence_[current_step];
	std::array<double, 6> f_d = f_sequence_[current_step];
	std::array<double, 6> selection_vector = selection_vector_sequence_[current_step];


	// get state variables
	update_dq_filter(robot_state);
	Eigen::Map<const eigen_vector7d> tau_measured(robot_state.tau_J.data());

	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<eigen_vector7d> mass(mass_array.data());
	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	Eigen::Map<eigen_vector7d> coriolis(coriolis_array.data());
	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<eigen_vector7d> gravity(gravity_array.data());


	
	// --- cartesian motion ---

	auto current_pose = model.pose(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
	//Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());




	// calculate pose from desired joints 
	auto desired_pose = model.pose(franka::Frame::kEndEffector, q_d, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(desired_pose.data()));
	Eigen::Vector3d position_d(transform_d.translation());
	Eigen::Quaterniond orientation_d(transform_d.linear());


	// compute error to desired pose
	Eigen::Matrix<double, 6, 1> error;

	// position error
	error.head(3) = position - position_d;

	// orientation error
	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
		orientation.coeffs() = -orientation.coeffs();
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// transform to base frame
	error.tail(3) = -transform_d.linear() * error.tail(3);

	
	if (error.head(3).norm() > 0.005 &&
		selection_vector[0] == 1 && selection_vector[1] == 1 && selection_vector[2] == 1)
	{
		error.head(3) = error.head(3).normalized() * 0.005;
		contact_change_motion = true;
	}

	
	// spring damper system with damping ratio=1 and filtered dq
	Eigen::Matrix<double, 6, 1> ft_cartesian_motion =
		-stiffness_ * error -damping_ * (jacobian * compute_dq_filtered());

	// --- cartesian motion end --- 

	
	// --- force motion ---
	
	Eigen::Matrix<double, 6, 1> ft_desired(f_d.data());


	fts_.update();
	std::array<double, 6> current_fts_values = fts_.current_values();
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> ft_existing(current_fts_values.data());


	//tau_existing = tau_measured - gravity;
	////auto ft_existing = jacobian * tau_existing;
	//auto ft_existing_array = robot_state.O_F_ext_hat_K;
	//Eigen::Matrix<double, 6, 1> ft_existing_from_tau = 
	//	(jacobian * jacobian.transpose()).inverse() * jacobian * tau_existing;


	Eigen::Matrix<double, 6, 1> ft_force = ft_desired;

	// pi controller using fts for neg z-direction
	if (selection_vector[2] == 0)
	{
		int contact_dim = 2;

		if (ft_existing(2) > -1 && ft_desired(contact_dim) < -1) // move to no contact
		{
			contact_change_motion = true;
			ft_force(contact_dim) = -3.0;
		}
		else if (ft_existing(2) > -2.5) // ft sensor value not really useful
		{
			; // todo
		}
		else
		{
			double error_fz = (ft_desired(contact_dim) - ft_existing(2));
			double f_z_error_derivate = (error_fz - pre_error_fz_) / period.toSec();
			f_z_error_integral_ += error_fz * period.toSec();
			ft_force(contact_dim) += 0.3 * error_fz +30.0 * f_z_error_integral_;// +0.0001 * f_z_error_derivate;

			pre_error_fz_ = error_fz;
		}
	}

	// pi controller for neg z-direction	
	if (selection_vector[0] == 0)
	{
		int contact_dim = 0;

		if (ft_existing(2) > -1 && ft_desired(contact_dim) > 1) // move to no contact
		{
			//contact_change_motion = true;
			ft_force(contact_dim) = 5.0;
		}
		else if (ft_existing(2) > -3) // ft sensor value not really useful
		{
			; // todo
		}
		else
		{
			f_x_error_integral_ += (ft_desired(contact_dim) - (-ft_existing(2))) * period.toSec();
			ft_force(contact_dim) += 0.2 * (ft_desired(contact_dim) - (-ft_existing(2))) + 5.0 * f_x_error_integral_;
		}
	}

	update_ft_filter(ft_force); // todo use selection vector
	ft_force = compute_ft_filtered();

	//if (ft_existing_from_tau[2] < 0.0)
	//{
	//	force_error_integral_ += period.toSec() * (desired_force_torque_z_force - ft_existing_from_tau);
	//	ft_command += 1.0 * (desired_force_torque_z_force - ft_existing_from_tau) + 1.0 * force_error_integral_;
	//}
	//tau_desired = jacobian.transpose() * desired_force_torque_z_force;
	//tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	//// FF + PI control
	//tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	//desired_mass_ = filter_gain * target_mass + (1 - filter_gain) * desired_mass_;

	Eigen::Matrix<double, 6, 1> ft_task;
	for (int i = 0; i < 6; ++i)
		ft_task[i] = selection_vector[i] * ft_cartesian_motion[i] + (1 - selection_vector[i]) * ft_force[i];

	// --- force motion end ---

	
	// --- compute control ---
	Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * ft_task;
	Eigen::Matrix<double, 7, 1> tau_d = tau_task + coriolis;



	if (log_)
	{
		pose_log_.emplace_back(transform);
		pose_d_log_.emplace_back(transform_d);
		error_log_.emplace_back(error);
		ft_log_.emplace_back(ft_force);
		ft_existing_log_.emplace_back(ft_existing);
	}



	if (contact_change_motion)
		time_ -= 0.001; // period.toSec(); // stay at the same step  todo better doc/ hack atm



	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
	franka::Torques output(tau_d_array);
	output.motion_finished = motion_finished;
	return output;
}


void seq_cart_vel_tau_generator::update_dq_filter(const franka::RobotState& robot_state)
{
	dq_buffer_[dq_current_filter_position_] = eigen_vector7d(robot_state.dq.data());

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


Eigen::Matrix<double, 7, 1> seq_cart_vel_tau_generator::compute_dq_filtered()
{
	eigen_vector7d value(eigen_vector7d::Zero());

	for (size_t i = 0; i < dq_filter_size_; ++i)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}


void seq_cart_vel_tau_generator::update_ft_filter(const Eigen::Matrix<double, 6, 1>& current_ft)
{
	ft_buffer_[ft_current_filter_position_] = current_ft;

	ft_current_filter_position_ = (ft_current_filter_position_ + 1) % ft_filter_size_;
}


Eigen::Matrix<double, 6, 1> seq_cart_vel_tau_generator::compute_ft_filtered()
{
	Eigen::Matrix<double, 6, 1> value(Eigen::Matrix<double, 6, 1>::Zero());

	for (size_t i = 0; i < ft_filter_size_; ++i)
		value += ft_buffer_[i];

	return value / ft_filter_size_;
}


} /* namespace detail */
} /* namespace franka_proxy */