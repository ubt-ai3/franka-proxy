/**
 *************************************************************************
 *
 * @file motion_generator_joint_max_accel.cpp
 *
 * Motion interpolator for franka movements that always uses maximum
 * acceleration, implementation.
 *
 ************************************************************************/


#include "motion_generator_joint_max_accel.hpp"

#include <cmath>
#include <ranges>
#include <algorithm>

#include <Eigen/Dense>
#include <franka/model.h>
#include <franka/exception.h>

#include "franka_proxy_share/franka_proxy_util.hpp"


namespace franka_proxy
{
namespace detail
{

bool JointMovement::isMotionFinished() const
{
	return std::ranges::all_of(joint_motion_finished, [](bool x) { return x; });
}

//////////////////////////////////////////////////////////////////////////
//
// franka_joint_motion_generator
//
//////////////////////////////////////////////////////////////////////////


franka_joint_motion_generator::franka_joint_motion_generator(
	double speed_factor,
	const std::array<double, 7>& q_goal, 
	std::mutex& current_state_lock, 
	franka::RobotState& current_state,
	const std::atomic_bool& stop_motion_flag, 
	bool stop_on_contact)
	: q_goal_(q_goal.data()),

	current_state_lock_(current_state_lock),
	current_state_(current_state),

	stop_motion_(stop_motion_flag),
	stop_on_contact_(stop_on_contact)
{
	if (speed_factor > 1.0)

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



JointMovement franka_joint_motion_generator::calculate_desired_values(double t) const
{
	JointMovement out;

	Vector7i sign_delta_q;
	for (int i = 0; i < 7; i++)
		sign_delta_q[i] = static_cast<int>(std::copysign(1.0, delta_q_[i]));

	for (Eigen::Index i = 0; i < 7; i++)
	{
		if (isMotionFinished(delta_q_[i]))
		{
			out.delta_q_d[i] = 0;
			out.joint_motion_finished[i] = true;
		}
		else
		{
			//uses the Formulas of Modeling, identification and control of robots
			//from page 327 to page 328
			if (t < t_1_sync_[i])
			{
				//formula 13.37
				out.delta_q_d[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
					(0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
			}
			else if (t >= t_1_sync_[i] && t < t_2_sync_[i])
			{
				//13.42
				out.delta_q_d[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
			}
			else if (t >= t_2_sync_[i] && t < t_f_sync_[i])
			{
				const double t_d = t_2_sync_[i] - t_1_sync_[i];
				const double delta_t_2_sync_ = t_f_sync_[i] - t_2_sync_[i];
				//13.43
				out.delta_q_d[i] = delta_q_[i] +
					0.5 *
					(1.0 / std::pow(delta_t_2_sync_, 3.0) *
						(t - t_1_sync_[i] - 2.0 * delta_t_2_sync_ - t_d) *
						std::pow((t - t_1_sync_[i] - t_d), 3.0) +
						(2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync_ - 2.0 * t_d)) *
					dq_max_sync_[i] * sign_delta_q[i];
			}
			else
			{
				out.delta_q_d[i] = delta_q_[i];
				out.joint_motion_finished[i] = true;
			}
		}
	}
	return out;
}

void franka_joint_motion_generator::calculate_synchronized_values()
{
	Vector7i sign_delta_q;
	for (int i = 0; i < 7; i++)
		sign_delta_q[i] = static_cast<int>(std::copysign(1.0, delta_q_[i]));

	constexpr double factor = 1.5;

	Vector7d t_f = Vector7d::Zero();
	for (Eigen::Index i = 0; i < 7; i++)
	{
		if (isMotionFinished(delta_q_[i]))
			continue;

		//max reachable speed
		double dq_max_reach = dq_max_[i];

		if (std::abs(delta_q_[i]) < //if overshooting target distance
			factor / 2.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
			factor / 2.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))
		{
			//reduce max speed
			dq_max_reach = std::sqrt
			(2.0 / factor * delta_q_[i] * sign_delta_q[i] *
				(ddq_max_start_[i] * ddq_max_goal_[i]) /
				(ddq_max_start_[i] + ddq_max_goal_[i]));
		}

		//acceleration time
		const double t_1 = factor * dq_max_reach / ddq_max_start_[i];
		//deceleration time
		const double delta_t_2 = factor * dq_max_reach / ddq_max_goal_[i];

		//final time = acceleration + deceleration + constant speed time?
		t_f[i] = (t_1 + delta_t_2) / 2.0 + std::abs(delta_q_[i]) / dq_max_reach;
	}
	const double max_t_f = t_f.maxCoeff();

	Vector7d delta_t_2_sync = Vector7d::Zero();
	for (Eigen::Index i = 0; i < 7; i++)
	{
		if (isMotionFinished(delta_q_[i]))
			continue;

		const double a = factor / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
		const double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
		const double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];

		dq_max_sync_[i] = calculateQuadraticSolution(a, b, c);
		t_1_sync_[i] = factor * dq_max_sync_[i] / ddq_max_start_[i];
		delta_t_2_sync[i] = factor * dq_max_sync_[i] / ddq_max_goal_[i];

		t_f_sync_[i] = (t_1_sync_[i] + delta_t_2_sync[i]) / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
		t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync[i];
		q_1_[i] = dq_max_sync_[i] * sign_delta_q[i] * 0.5 * t_1_sync_[i];
	}
}

double franka_joint_motion_generator::calculateQuadraticSolution(double a, double b, double c)
{
	double delta = b * b - 4.0 * a * c;
	//only consider positive solution
	delta = std::max(delta, 0.0);

	return (-b - std::sqrt(delta)) / (2.0 * a);
}

bool franka_joint_motion_generator::isMotionFinished(double delta)
{
	return std::abs(delta) <= kDeltaQMotionFinished;
}

bool franka_joint_motion_generator::colliding(const franka::RobotState& state)
{
	return std::ranges::any_of(state.joint_contact, [](const double& v) { return v > 0; }) ||
		std::ranges::any_of(state.cartesian_contact, [](const double& v) { return v > 0; });
}

//When a robot state is received, the callback function is used to calculate the response: the desired values for that time step. After sending back the response,
//the callback function will be called again with the most recently received robot state. Since the robot is controlled with a 1 kHz frequency,
//the callback functions have to compute their result in a short time frame in order to be accepted
franka::JointPositions franka_joint_motion_generator::operator()(
	const franka::RobotState& robot_state, franka::Duration period)
{
	time_ += period.toSec();

	{
		std::lock_guard state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();

	if (stop_on_contact_ && colliding(robot_state))
		throw contact_stop_trigger();


	if (time_ == 0.0) //the first invocation of the callback function
	{
		if (!franka_proxy_util::is_reachable(q_goal_)) {
			throw franka::InvalidOperationException("Target is not reachable");
		}

		q_start_ = Vector7d(robot_state.q_d.data());
		delta_q_ = q_goal_ - q_start_;
		calculate_synchronized_values();
	}
	
	const auto desiredValues = calculate_desired_values(time_);

	std::array<double, 7> joint_positions{};
	Eigen::VectorXd::Map(joint_positions.data(), 7) = (q_start_ + desiredValues.delta_q_d);
	franka::JointPositions output(joint_positions);
	output.motion_finished = desiredValues.isMotionFinished();
	return output;
}

} /* namespace detail */
} /* namespace franka_proxy */
