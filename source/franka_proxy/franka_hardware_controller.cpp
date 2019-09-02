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

#include <franka/exception.h>


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_hardware_controller
//
//////////////////////////////////////////////////////////////////////////


franka_hardware_controller::franka_hardware_controller
(const std::string& controller_ip)
	: robot_(controller_ip, franka::RealtimeConfig::kIgnore),
	  parameters_initialized_(false),
	  stop_motion_(),
	  speed_factor_(0.05),

	  gripper_(controller_ip),
	  max_width_(gripper_.readOnce().max_width),

	  robot_state_(robot_.readOnce()),
	  gripper_state_(gripper_.readOnce()),

	  terminate_state_thread_(false),
	  state_thread_([this]() { state_update_loop(); })
{
}


franka_hardware_controller::~franka_hardware_controller() noexcept
{
	terminate_state_thread_ = true;
	state_thread_.join();
}


void franka_hardware_controller::move_to(const robot_config_7dof& target)
{
	initialize_parameters();

	MotionGenerator motion_generator
		(speed_factor_, target, state_lock_, robot_state_, stop_motion_);

	stop_motion_ = false;

	try
	{
		control_loop_running_.set(true);
		{
			// Lock the current_state_lock_ to wait for state_thread_ to finish.
			std::lock_guard<std::mutex> state_guard(state_lock_);
		}

		robot_.control(motion_generator, franka::ControllerMode::kJointImpedance, true, 20.);
	}
	catch (const stop_motion_trigger&)
	{
		control_loop_running_.set(false);
	}
	catch (const franka::Exception&)
	{
		control_loop_running_.set(false);
		throw;
	}
}


void franka_hardware_controller::stop_movement()
{
	stop_motion_ = true;
	try
	{
		gripper_.stop();
	}
	catch (const franka::Exception&)
	{
	}
}


void franka_hardware_controller::set_speed_factor(double speed_factor)
{
	std::lock_guard<std::mutex> state_guard(speed_factor_lock_);
	speed_factor_ = speed_factor;
}


franka::RobotState franka_hardware_controller::robot_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return robot_state_;
}


void franka_hardware_controller::open_gripper()
{
	if (!gripper_.move(max_width_, gripper_speed))
	{
		std::cerr << "Gripper opening failed." << std::endl;
	}

	{
		std::lock_guard<std::mutex> state_guard(state_lock_);
		gripper_state_ = gripper_.readOnce();
	}
}


void franka_hardware_controller::close_gripper(double speed, double force)
{
	gripper_.grasp(min_grasp_width, speed, force, 0, 1);

	{
		std::lock_guard<std::mutex> state_guard(state_lock_);
		gripper_state_ = gripper_.readOnce();
	}
}


franka::GripperState franka_hardware_controller::gripper_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return gripper_state_;
}


void franka_hardware_controller::automatic_error_recovery()
{
	robot_.automaticErrorRecovery();
}


void franka_hardware_controller::state_update_loop()
{
	while (!terminate_state_thread_)
	{
		control_loop_running_.wait_for(false);
		if (control_loop_running_.get())
			continue;

		{
			std::lock_guard<std::mutex> state_guard(state_lock_);
			robot_state_ = robot_.readOnce();
			gripper_state_ = gripper_.readOnce();
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(20ms);
	}
}


void franka_hardware_controller::initialize_parameters()
{
	while (!parameters_initialized_)
	{
		robot_.setCollisionBehavior(
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

		robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
		robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

		parameters_initialized_ = true;
	}
}


//////////////////////////////////////////////////////////////////////////
//
// franka_hardware_controller::MotionGenerator
//
//////////////////////////////////////////////////////////////////////////


franka_hardware_controller::MotionGenerator::MotionGenerator
(double speed_factor, const std::array<double, 7> q_goal,
 std::mutex& current_state_lock, franka::RobotState& current_state,
 const std::atomic_bool& stop_motion_flag)
	: q_goal_(q_goal.data()),

	  current_state_lock_(current_state_lock),
	  current_state_(current_state),

	  stop_motion_(stop_motion_flag)
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


bool franka_hardware_controller::MotionGenerator::calculateDesiredValues
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


void franka_hardware_controller::MotionGenerator::calculateSynchronizedValues()
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


franka::JointPositions franka_hardware_controller::MotionGenerator::operator()
(const franka::RobotState& robot_state, franka::Duration period)
{
	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger();


	time_ += period.toSec();

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


} /* namespace franka_proxy */
