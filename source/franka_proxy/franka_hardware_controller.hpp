	/**
 *************************************************************************
 *
 * @file franka_hardware_controller.hpp
 *
 * Classes to control a franka emika panda robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP


#include <atomic>
#include <string>

#include <viral_core/thread_synch.hpp>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <vector>

#include "franka_motion_recorder.hpp"


namespace franka_proxy
{


typedef std::array<double, 7> robot_config_7dof;


/**
 *************************************************************************
 *
 * @class franka_hardware_controller
 *
 * todo
 *
 ************************************************************************/
class franka_hardware_controller
{
public:

	franka_hardware_controller
		(const std::string& controller_ip);

	virtual ~franka_hardware_controller() noexcept;



	/**
	 * Moves the Panda robot to given target; In case
	 * of collision, the movement is retried and continued 
	 * after automatic error recovery
	 */
	void move_to(const robot_config_7dof& target);
	/**
	 * Moves the Panda robot to given target; In case
	 * of contact, the movement is aborted and false is returned.
	 */
	bool move_to_until_contact(const robot_config_7dof& target);

	void stop_movement();

	void set_speed_factor(double speed_factor);

	franka::RobotState robot_state() const;


	/** Move the gripper to gripper::max_width. */
	void open_gripper();
	/** Move the gripper to gripper::min_grasp_width. */
	void close_gripper();
	/** Grasp. */
	bool grasp_gripper(double speed, double force);

	franka::GripperState gripper_state() const;


	void automatic_error_recovery();



	void apply_z_force(
		double mass, 
		double duration);

	/**
	 * Starts/Stops the recording callback.
	 */
	void start_recording();
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop_recording();
	
	/**
	 * Moves the Panda robot along a given sequence.
	 */
	void move_sequence(
		const std::vector<std::array<double, 7>>& q_sequence);

	void move_sequence(
		const std::vector<std::array<double, 7>>& q_sequence,
		double f_z);

	void move_sequence(
		const std::vector<std::array<double, 7>>& q_sequence,
		const std::vector<std::array<double, 6>>& f_sequence,
		const std::vector<std::array<double, 6>>& selection_vector);


private:

	/**
	 * Used to update the current robot state while no control loop is
	 * running.
	 */
	void state_update_loop();


	/**
	 * Initialize parameters such as joint impedance and collision behavior.
	 */
	void initialize_parameters();

	void set_default_collision_behaviour();
	void set_contact_drive_collision_behaviour();


	// Robot
	mutable franka::Robot robot_;
	bool parameters_initialized_;

	std::atomic_bool stop_motion_;

	mutable std::mutex speed_factor_lock_;
	double speed_factor_;

	std::atomic_bool stop_recording_;
	detail::motion_recorder motion_recorder_;

	// Gripper
	mutable std::unique_ptr<franka::Gripper> gripper_;
	double max_width_;

	static constexpr double gripper_speed = 0.025;
	static constexpr double open_epsilon = 0.1;
	static constexpr double min_grasp_width = 0.003;


	mutable std::mutex state_lock_;
	franka::RobotState robot_state_;
	franka::GripperState gripper_state_;

	viral_core::signal control_loop_running_;
	std::atomic_bool terminate_state_thread_;
	std::thread state_thread_;
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP) */
