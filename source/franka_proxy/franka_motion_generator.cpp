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

#include <Eigen/Dense>


namespace franka_proxy
{
namespace detail
{


//////////////////////////////////////////////////////////////////////////
//
// franka_motion_generator
//
//////////////////////////////////////////////////////////////////////////


joint_motion_generator::joint_motion_generator
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


bool joint_motion_generator::calculateDesiredValues
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


void joint_motion_generator::calculateSynchronizedValues()
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


bool joint_motion_generator::colliding(const franka::RobotState& state)
{
	for (double v : state.joint_contact)
		if (v > 0) return true;
	for (double v : state.cartesian_contact)
		if (v > 0) return true;
	return false;
}


franka::JointPositions joint_motion_generator::operator()
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
	  dq_d_({0.,0.,0.,0.,0.,0.,0.}),
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


	gravity_array = model.gravity(robot_state);
	Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());
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


	// updateDQFilter
	for (size_t i = 0; i < 7; i++) {
		dq_buffer_[dq_current_filter_position_ * 7 + i] = robot_state.dq[i];
	}
	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;

	// compute torques
	for (size_t i = 0; i < 7; i++) {
		// compute dq filtered
		double value = 0;
		for (size_t j = i; j < 7 * dq_filter_size_; j += 7) {
			value += dq_buffer_[j];
		}
		value = value / dq_filter_size_;

		// impedance control law
		tau_J_d[i] = (K_P_[i] * (robot_state.q_d[i] - robot_state.q[i]) + K_D_[i] * (dq_d_[i] - value));
	}


	// Smoothly update the mass to reach the desired target value.
	desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;


	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = (tau_command + tau_J_d) * .5;


	forces_z.push_back(robot_state.O_F_ext_hat_K[2]);


	return tau_d_array;
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
		franka::JointVelocities output({0.,0.,0.,0.,0.,0.,0.});
		output.motion_finished = true;
		return output;
	}

	if (period.toMSec() < 1)
		throw("Period under 1ms.");

	// motion
	Vector7d q_(robot_state.q.data());
	Vector7d q_seq(q_sequence_[step].data());

	if ((q_seq - q_).norm() < 0.0001)
		return franka::JointVelocities({ 0.,0.,0.,0.,0.,0.,0. });


	std::array<double, 7> vel{};
	Eigen::VectorXd::Map(&vel[0], 7) = (q_seq - q_) * 1.1;// *period.toMSec(); // todo hack

	franka::JointVelocities output(vel);
	output.motion_finished = false;
	return output;
}


} /* namespace detail */
} /* namespace franka_proxy */