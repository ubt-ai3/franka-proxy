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
	  duration(duration)
{
	franka::RobotState initial_state = robot.readOnce();
	gravity_array = model.gravity(initial_state);

	Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
	initial_tau_ext = initial_tau_measured - initial_gravity;
}


franka::Torques force_motion_generator::callback(const franka::RobotState& robot_state, franka::Duration period)
{
	time_ += period.toSec();

	if (time_ == 0.0)
	{
		initial_position = get_position(robot_state);
	}

	if (time_ > duration)
	{
		// todo this is wrong!
		franka::Torques current_torques(robot_state.tau_J);
		current_torques.motion_finished = true;
		return current_torques;
	}

	//if (time_ > 0 && (get_position(robot_state) - initial_position).norm() > 0.01)
	//{
	//	throw std::runtime_error("Aborting; too far away from starting pose!"); // todo
	//}

	// get state variables
	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

	Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
	desired_force_torque.setZero();
	desired_force_torque(2) = desired_mass * -9.81;
	tau_ext << tau_measured - gravity - initial_tau_ext;
	tau_d << jacobian.transpose() * desired_force_torque;
	tau_error_integral += period.toSec() * (tau_d - tau_ext);
	// FF + PI control
	tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value
	desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;

	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;
	return tau_d_array;
}


Eigen::Vector3d force_motion_generator::get_position(const franka::RobotState& robot_state)
{
	return Eigen::Vector3d(
		robot_state.O_T_EE[12],
		robot_state.O_T_EE[13],
		robot_state.O_T_EE[14]);
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
	speed_factor_(speed_factor),
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
	speed_factor_(speed_factor),
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

	if ((q_seq - q_).norm() < 0.001)
		return franka::JointVelocities({ 0.,0.,0.,0.,0.,0.,0. });


	std::array<double, 7> vel{};
	Eigen::VectorXd::Map(&vel[0], 7) = (q_seq - q_);// *period.toMSec();

	franka::JointVelocities output(vel);
	output.motion_finished = false;
	return output;
}


} /* namespace detail */
} /* namespace franka_proxy */