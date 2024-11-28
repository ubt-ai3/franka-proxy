/**
 *************************************************************************
 *
 * @file franka_controller_emulated.hpp
 *
 * Franka controller to emulate a virtual robot.
 *
 ************************************************************************/

#pragma once


#include <mutex>
#include <optional>

#include "franka_controller.hpp"


namespace franka_control
{
/**
 *************************************************************************
 *
 * @class franka_controller_emulated
 *
 * A franka_controller that emulates robot movement.
 *
 * Usage notes:
 * - The robot_emulator does not have any acceleration limits
 *		and uses an implementation-specific maximum speed.
 *
 ************************************************************************/
class franka_controller_emulated : public franka_controller
{
public:
	franka_controller_emulated();
	~franka_controller_emulated() noexcept override;


	void move(const robot_config_7dof& target) override;
	void move_with_force(const robot_config_7dof& target,
	                     const wrench& target_force_torques) override;
	bool move_until_contact(const robot_config_7dof& target) override;

	void open_gripper() override;
	void close_gripper() override;
	void grasp_gripper(double speed = 0.025, double force = 0.05) override;
	bool gripper_grasped() const override;

	double speed_factor() const override;
	void set_speed_factor(double speed_factor) override;

	void set_guiding_mode(bool x, bool y, bool z, bool rx, bool ry, bool rz,bool elbow) const override;

	void automatic_error_recovery() override;

	robot_config_7dof current_config() const override;
	wrench current_force_torque() const override;
	int current_gripper_pos() const override;
	int max_gripper_pos() const override;

	void update() override;

	void start_recording(std::optional<std::string> log_file_path = std::nullopt) override;

	/**
	* Stop recording playback data, assumes that start_recording() has been called bevore.
	* 
	* This returns a motion sampled at 1kHz, but the robot always remains in the position
	* it was in when stop_playback() was called.
	**/
	std::pair<std::vector<robot_config_7dof>, std::vector<wrench>>
	stop_recording() override;

	/**
	* Simulates a playback movement with increment, but ignores force and selection values.
	**/
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector_sequence,
		std::array<double, 16> offset_cartesian ,
		std::array<double, 6> offset_force ) override;

	/**
	* Simulates a playback movement, but ignores force and selection values.
	**/
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector_sequence) override;
private:
	std::chrono::time_point<std::chrono::steady_clock> recording_start_;

	void move_gripper(int target, double speed_mps);


	static constexpr double max_speed_length_per_sec_ = 3.5; // ~200 deg
	static constexpr float move_update_rate_ = 0.01f;

	mutable std::mutex controller_mutex_;
	double speed_factor_;
	bool gripper_open_;

	robot_config_7dof state_joint_values_;
	wrench state_force_torque_values_;
	int state_gripper_pos_;
	int state_position_in_sequence_;

	static constexpr int max_gripper_pos_ = 50;
	static constexpr double gripper_default_speed_mps_ = 0.025;
};
} /* namespace franka_control */
