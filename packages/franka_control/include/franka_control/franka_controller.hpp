/**
 *************************************************************************
 *
 * @file franka_controller.hpp
 *
 * Base class for franka controllers.
 *
 ************************************************************************/

#pragma once


#include <atomic>
#include <thread>
#include <vector>
#include <optional>

#include <Eigen/Geometry>

namespace franka_control
{
using robot_config_7dof = Eigen::Matrix<double, 7, 1>;
using wrench = Eigen::Matrix<double, 6, 1>;
using selection_diagonal = Eigen::Matrix<double, 6, 1>;


/**
 *************************************************************************
 *
 * @class franka_controller
 *
 * Interface for controlling and monitoring a franka emika panda robot.
 *
 * Usage notes:
 * - To control a robot, clients send commands (e.g. set_speed_factor())
 *      via the robot_controller.
 * - To monitor a robot, clients must fetch franka_controller states
 *		from the actual robot by update().
 * - Without any update() call, the robot_controller may not change
 *		its exposed states.
 * - Clients should regularly call update() to avoid network timeouts.
 *		A franka_update_task conveniently automates update()s.
 *
 ************************************************************************/
class franka_controller
{
public:
	franka_controller();

	virtual ~franka_controller() noexcept;


	virtual void move(const robot_config_7dof& target) = 0;
	virtual void move_with_force(
		const robot_config_7dof& target,
		const wrench& target_force_torques) = 0;
	void move(const Eigen::Affine3d& target_world_T_tcp);

	/**
	 * Moves the robot to given target. If target is reached, returns true;
	 * In case of contact, the movement is aborted and returns false.
	 */
	virtual bool move_until_contact(const robot_config_7dof& target) = 0;
	bool move_until_contact(const Eigen::Affine3d& target_world_T_tcp);


	virtual void open_gripper() = 0;
	virtual void close_gripper() = 0;
	virtual void grasp_gripper(double speed = 0.025, double force = 0.05) = 0;
	virtual bool gripper_grasped() const = 0;

	virtual double speed_factor() const = 0;
	virtual void set_speed_factor(double speed_factor) = 0;

	virtual void set_guiding_mode(bool x, bool y, bool z, bool rx, bool ry, bool rz, bool elbow) const = 0;

	virtual void automatic_error_recovery() = 0;


	virtual robot_config_7dof current_config() const = 0;
	virtual wrench current_force_torque() const = 0;
	virtual int current_gripper_pos() const = 0; // in [mm]
	virtual int max_gripper_pos() const = 0; // in [mm]

	Eigen::Affine3d current_world_T_tcp() const;
	Eigen::Affine3d current_world_T_j7() const;
	Eigen::Affine3d current_world_T_flange() const;


	/**
	 * Fetch current state from the back-end robot.
	 * Call regularly to avoid overflow in network buffers,
	 * e.g. through a robot_controller_task.
	 */
	virtual void update() = 0;

	virtual void start_recording(std::optional<std::string> log_file_path = std::nullopt) = 0;
	virtual std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> stop_recording() = 0;
	virtual void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector_sequence) = 0;


	const Eigen::Affine3d j7_T_flange;
	const Eigen::Affine3d flange_T_tcp;
	const Eigen::Affine3d j7_T_tcp;
	const Eigen::Affine3d tcp_T_j7;

	// used to convert internal double gripper width in meters into an int
	static constexpr double gripper_unit_per_m_ = 1000.0;

private:
	static Eigen::Affine3d build_j7_T_flange();
	static Eigen::Affine3d build_flange_T_tcp();
	static Eigen::Affine3d build_j7_T_tcp();
	static Eigen::Affine3d build_tcp_T_j7();
};


/**
 *************************************************************************
 *
 * @class franka_update_task
 *
 * Updates a franka_controller instance in a separate thread.
 *
 ************************************************************************/

class franka_update_task
{
public:
	explicit franka_update_task(franka_controller& controller);
	~franka_update_task() noexcept;

private:
	void task_main();

	static const double update_time_step_secs_;
	franka_controller& controller_;

	std::thread internal_thread_;
	std::atomic_bool terminate_internal_thread_;
};
} /* namespace franka_control */
