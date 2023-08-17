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
#include <condition_variable>
#include <string>
#include <vector>

#include <franka/robot.h>
#include <franka/gripper.h>

#include "ft_sensor/ft_sensor.hpp"


namespace franka_proxy
{


using robot_config_7dof = std::array<double, 7>;
using robot_force_config = std::array<double, 6>;
using robot_force_selection = std::array<double, 6>;

namespace detail
{
	class motion_recorder;
}

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
	void open_gripper(double speed = default_gripper_speed);
	/** Move the gripper to gripper::min_grasp_width. */
	void close_gripper(double speed = default_gripper_speed);
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
	std::pair<std::vector<robot_config_7dof>, std::vector<robot_force_config>> start_recording(float seconds);
	std::pair<std::vector<robot_config_7dof>, std::vector<robot_force_config>> stop_recording();
	
	/**
	 * Moves the Panda robot along a given sequence.
	 */
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence);

	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		double f_z);

	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<robot_force_config>& f_sequence,
		const std::vector<robot_force_selection>& selection_vector);


	static constexpr double default_gripper_speed = 0.025;



private:

	/**
	 * Used to update the current robot state while no control loop is
	 * running.
	 */
	void robot_state_update_loop();


	/**
	* Used to update the current gripper state, regardless if a control loop
	* is running or not.
	*/
	void gripper_state_update_loop();



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

	// Gripper
	mutable std::unique_ptr<franka::Gripper> gripper_;
	double max_width_;

	// FT-Sensor
	mutable std::unique_ptr<ft_sensor> ft_sensor_;

	std::unique_ptr<detail::motion_recorder> motion_recorder_;

	
	static constexpr double open_epsilon = 0.1;
	static constexpr double min_grasp_width = 0.003;


	mutable std::mutex robot_state_lock_;
	franka::RobotState robot_state_;

	mutable std::mutex gripper_state_lock_;
	franka::GripperState gripper_state_;

	void set_control_loop_running(bool running);
	bool control_loop_running_;
	std::mutex control_loop_running_mutex_;
	std::condition_variable control_loop_running_cv_;

	std::atomic_bool terminate_state_threads_;
	std::thread robot_state_thread_;
	std::thread gripper_state_thread_;


};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP) */
