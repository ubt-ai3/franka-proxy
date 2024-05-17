/**
 *************************************************************************
 *
 * @file franka_controller_emulated.cpp
 *
 * Franka controller to emulate a robot, implementation.
 *
 ************************************************************************/


#include "franka_controller_emulated.hpp"

#include <iostream>

#include <franka_util/exception.hpp>


namespace franka_control
{


//////////////////////////////////////////////////////////////////////////
//
// franka_controller_emulated
//
//////////////////////////////////////////////////////////////////////////


franka_controller_emulated::franka_controller_emulated()
	:
	speed_factor_(0.f),
	gripper_open_(false),

	state_joint_values_
		((Eigen::Matrix<double, 7, 1>() <<
			0, 0, 0, -0.0698, 0, 0, 0).finished()),
	state_gripper_pos_(0)
{ }


franka_controller_emulated::~franka_controller_emulated() noexcept = default;


bool almost_equal(const robot_config_7dof& xes, const robot_config_7dof& array)
{
	for (int i = 0; i < 7; ++i)
	{
		if (abs(xes[i] - array[i]) >= 0.001)
			return false;
	}

	return true;
}


robot_config_7dof operator+
	(const robot_config_7dof& xes, const robot_config_7dof& rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] + rhs[i];
	return ret;
}


robot_config_7dof operator-
	(const robot_config_7dof& xes, const robot_config_7dof& rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] - rhs[i];
	return ret;
}


double length(const robot_config_7dof& xes)
{
	double len = 0;

	for (int i = 0; i < 7; ++i)
		len += xes[i] * xes[i];

	return sqrt(len);
}


robot_config_7dof operator*(const robot_config_7dof& xes, double rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] * rhs;
	return ret;
}


void franka_controller_emulated::move(const robot_config_7dof& target)
{
	robot_config_7dof current_joint_values = current_config();

	auto last_time = std::chrono::steady_clock::now();

	while (!almost_equal(target, current_joint_values))
	{
		auto next_timepoint =
			std::chrono::steady_clock::now() +
			std::chrono::duration_cast<std::chrono::milliseconds>
				(std::chrono::duration<double>(move_update_rate_));
		
		// Determine joint-space length each joint has moved
		// since the last iteration.
		auto now = std::chrono::steady_clock::now();
		double seconds_passed =
			std::chrono::duration_cast<std::chrono::duration<double>>
				(now - last_time).count();
		double move_length =
			seconds_passed *
			speed_factor() *
			max_speed_length_per_sec_;

		last_time = now;

		// Move robot joints by given length.
		double length_to_next =
			length(target - current_joint_values);

		if (length_to_next < move_length)
		{
			// If the next waymark is in reach, move there.
			current_joint_values = target;
		}
		else
		{
			// Move into the direction of the next waymark,
			// but don't actually reach it.
			current_joint_values = current_joint_values +
				(target - current_joint_values) *
					(move_length / length_to_next);
		}

		// Copy from process variables to exposed state.
		{
			std::lock_guard<std::mutex> lk(controller_mutex_);
			state_joint_values_ = current_joint_values;
		}

		std::this_thread::sleep_until(next_timepoint);
	}
}


bool franka_controller_emulated::move_until_contact
	(const robot_config_7dof& target)
{
	move(target);
	return true;
}


void franka_controller_emulated::open_gripper()
	{ move_gripper(max_gripper_pos_, gripper_default_speed_mps_); }


void franka_controller_emulated::close_gripper()
	{ move_gripper(0, gripper_default_speed_mps_); }


void franka_controller_emulated::grasp_gripper(double speed, double force)
	{ move_gripper(0, speed); }


bool franka_controller_emulated::gripper_grasped() const
	{ return false; }


double franka_controller_emulated::speed_factor() const
{
	std::lock_guard<std::mutex> lk(controller_mutex_);
	return speed_factor_;
}


void franka_controller_emulated::set_speed_factor(double speed_factor)
{
	std::lock_guard<std::mutex> lk(controller_mutex_);
	speed_factor_ = speed_factor;
}


void franka_controller_emulated::automatic_error_recovery() {}


robot_config_7dof franka_controller_emulated::current_config() const
{
	std::lock_guard<std::mutex> lk(controller_mutex_);
	return state_joint_values_;
}


int franka_controller_emulated::current_gripper_pos() const
{
	std::lock_guard<std::mutex> lk(controller_mutex_);
	return state_gripper_pos_;
}


int franka_controller_emulated::max_gripper_pos() const
{
	return max_gripper_pos_;
}


void franka_controller_emulated::update() {}

	
void franka_controller_emulated::start_recording()
{
	std::cerr << "franka_controller_emulated::start_recording(): Not implemented.";
	throw not_implemented();
}

	
std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_controller_emulated::stop_recording()
{
	std::cerr << "franka_controller_emulated::stop_recording(): Not implemented.";
	throw not_implemented();
}

	
void franka_controller_emulated::move_sequence
	(std::vector<std::array<double, 7>> q_sequence,
	 std::vector<std::array<double, 6>> f_sequence,
	 std::vector<std::array<double, 6>> selection_vector_sequence)
{
	std::cerr << "franka_controller_emulated::move_sequence(): Not implemented.";
	throw not_implemented();
}


void franka_controller_emulated::move_gripper(int target, double speed_mps)
{
	double current_pos = current_gripper_pos();

	auto last_time = std::chrono::steady_clock::now();

	while (abs(current_pos - target) > 0.001)
	{
		auto next_timepoint =
			std::chrono::steady_clock::now() +
			std::chrono::duration_cast<std::chrono::milliseconds>
				(std::chrono::duration<double>(move_update_rate_));
		
		double remaining_distance = target - current_pos;
		auto now = std::chrono::steady_clock::now();
		double seconds_passed =
			std::chrono::duration_cast<std::chrono::duration<double>>
				(now - last_time).count();
		double move_length = 
			seconds_passed * gripper_unit_per_m_ * speed_mps;

		last_time = now;

		if (abs(remaining_distance) <= move_length)
		{
			// If the target is in reach, move there.
			current_pos = target;
		}
		else
		{
			// Move into the direction of the target,
			// but don't actually reach it.
			if (remaining_distance < 0)
				current_pos -= move_length;
			else
				current_pos += move_length;
		}

		// Copy from process variables to exposed state.
		{
			std::lock_guard<std::mutex> lk(controller_mutex_);
			state_gripper_pos_ = static_cast<int>(current_pos);
		}

		std::this_thread::sleep_until(next_timepoint);
	}
	
	std::lock_guard<std::mutex> lk(controller_mutex_);
	state_gripper_pos_ = target;
}




} /* namespace franka_control */