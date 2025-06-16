/**
 *************************************************************************
 *
 * @file franka_controller_emulated.cpp
 *
 * Franka controller to emulate a robot, implementation.
 *
 ************************************************************************/

#include "franka_controller_emulated.hpp"


namespace franka_control
{
namespace
{
// Helper function in anonymous namespace
bool almost_equal(
	const robot_config_7dof& xes,
	const robot_config_7dof& other)
{
	for (int i = 0; i < 7; ++i)
	{
		if (abs(xes[i] - other[i]) >= 0.001)
			return false;
	}

	return true;
}
}

//////////////////////////////////////////////////////////////////////////
//
// franka_controller_emulated
//
//////////////////////////////////////////////////////////////////////////


franka_controller_emulated::franka_controller_emulated()
	: speed_factor_(0.1f),
	  state_joint_values_(
		  (robot_config_7dof() << 0, 0, 0, -0.0698, 0, 0, 0).finished()),
	  state_force_torque_values_(
		  (wrench() << 0, 0, 0, 0, 0, 0).finished()),
	  state_gripper_pos_(0)
{
}

franka_controller_emulated::~franka_controller_emulated() noexcept = default;

void franka_controller_emulated::move(const robot_config_7dof& target)
{
	robot_config_7dof current_joint_values = current_config();
	auto last_time = std::chrono::steady_clock::now();

	while (!almost_equal(target, current_joint_values))
	{
		auto next_time_point =
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
			(target - current_joint_values).norm();

		if (length_to_next < move_length)
		{
			// If the target is in reach, move there.
			current_joint_values = target;
		}
		else
		{
			// Move into the direction of the target,
			// but don't actually reach it.
			current_joint_values = current_joint_values +
				(target - current_joint_values) *
				(move_length / length_to_next);
		}

		// Copy from process variables to exposed state.
		{
			std::lock_guard lk(controller_mutex_);
			state_joint_values_ = current_joint_values;
		}

		std::this_thread::sleep_until(next_time_point);
	}
}


bool franka_controller_emulated::move_until_contact(
	const robot_config_7dof& target)
{
	move(target);
	return true;
}


void franka_controller_emulated::open_gripper()
{
	move_gripper(max_gripper_pos_, gripper_default_speed_mps_);
}


void franka_controller_emulated::close_gripper()
{
	move_gripper(0, gripper_default_speed_mps_);
}


void franka_controller_emulated::grasp_gripper(
	double speed, double force)
{
	move_gripper(0, speed);
}


bool franka_controller_emulated::gripper_grasped() const
{
	return false;
}


double franka_controller_emulated::speed_factor() const
{
	std::lock_guard lk(controller_mutex_);
	return speed_factor_;
}


void franka_controller_emulated::set_speed_factor(
	double speed_factor)
{
	std::lock_guard lk(controller_mutex_);
	speed_factor_ = speed_factor;
}

void franka_controller_emulated::set_guiding_mode(
	bool x, bool y, bool z,
	bool rx, bool ry, bool rz,
	bool elbow)
{
}


void franka_controller_emulated::automatic_error_recovery()
{
}


robot_config_7dof franka_controller_emulated::current_config() const
{
	return state_joint_values_;
}

wrench franka_controller_emulated::current_force_torque() const
{
	return state_force_torque_values_;
}


int franka_controller_emulated::current_gripper_pos() const
{
	std::lock_guard lk(controller_mutex_);
	return state_gripper_pos_;
}


int franka_controller_emulated::max_gripper_pos() const
{
	return max_gripper_pos_;
}


void franka_controller_emulated::update()
{
}


void franka_controller_emulated::start_recording(
	std::optional<std::string> log_file_path)
{
	std::lock_guard lk(controller_mutex_);
	recording_start_ = std::chrono::steady_clock::now();
}


std::pair<std::vector<robot_config_7dof>, std::vector<wrench>>
franka_controller_emulated::stop_recording()
{
	std::unique_lock lk(controller_mutex_);
	const auto recording_start = recording_start_;
	const auto jc = state_joint_values_;
	lk.unlock();

	const auto duration = std::chrono::steady_clock::now() - recording_start;
	const auto dur_ms =
		static_cast<size_t>(
			std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());

	std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> result;

	result.first = std::vector<robot_config_7dof>(
		dur_ms, {jc[0], jc[1], jc[2], jc[3], jc[4], jc[5], jc[6]});
	result.second = std::vector<wrench>(dur_ms, {0, 0, 0, 0, 0, 0});

	return result;
}

void franka_controller_emulated::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_vector_sequence)
{
	const auto start_time = std::chrono::steady_clock::now();

	while (true)
	{
		// Calculate time_point in sequence
		auto now = std::chrono::steady_clock::now();
		const auto next_time_point = now +
			std::chrono::duration_cast<std::chrono::milliseconds>
			(std::chrono::duration<double>(move_update_rate_));
		unsigned long long ticks_passed =
			std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();


		// Stop after sequence is finished
		if (ticks_passed >= q_sequence.size())
			break;

		// Copy from process variables to exposed state.
		{
			std::lock_guard lk(controller_mutex_);
			state_joint_values_ =
				Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_sequence[ticks_passed].data());
			state_force_torque_values_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
				f_sequence[ticks_passed].data());
		}

		std::this_thread::sleep_until(next_time_point);
	}

	std::lock_guard lk(controller_mutex_);
	state_joint_values_ =
		Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_sequence.back().data());
	state_force_torque_values_ =
		Eigen::Map<const Eigen::Matrix<double, 6, 1>>(f_sequence.back().data());
}

void franka_controller_emulated::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_vector_sequence,
	const std::array<double, 16> offset_cartesian,
	const std::array<double, 6> offset_force)
{
	move_sequence(q_sequence, f_sequence, selection_vector_sequence);
}

void franka_controller_emulated::move_gripper(int target, double speed_mps)
{
	double current_pos = current_gripper_pos();

	auto last_time = std::chrono::steady_clock::now();

	while (abs(current_pos - target) > 0.001)
	{
		auto next_time_point =
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
			std::lock_guard lk(controller_mutex_);
			state_gripper_pos_ = static_cast<int>(current_pos);
		}

		std::this_thread::sleep_until(next_time_point);
	}

	std::lock_guard lk(controller_mutex_);
	state_gripper_pos_ = target;
}
} /* namespace franka_control */
