#ifndef INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP
#define INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP
/**
 *************************************************************************
 *
 * @file franka_hardware_controller.hpp
 *
 * Classes to control a franka emika panda robot.
 *
 ************************************************************************/


#include <atomic>
#include <condition_variable>
#include <list>
#include <string>
#include <vector>
#include <optional>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/vacuum_gripper.h>

#include <ft_sensor/ft_sensor.hpp>


namespace franka_proxy
{
using robot_config_7dof = std::array<double, 7>;
using twist = std::array<double, 6>;
using wrench = std::array<double, 6>;
using selection_diagonal = std::array<double, 6>;


namespace detail
{
class motion_recorder;
}


/**
 *************************************************************************
 *
 * @class franka_hardware_controller
 *
 ************************************************************************/
class franka_hardware_controller
{
public:
	franka_hardware_controller(
		const std::string& controller_ip,
		bool enforce_realtime = false);


	~franka_hardware_controller() noexcept;


	void automatic_error_recovery();

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

	// @throws ft_sensor_connection_exception
	void set_bias(const std::array<double, 6>& bias);

	// @throws ft_sensor_connection_exception
	void set_load_mass(const std::array<double, 3>& load_mass);

	void set_guiding_mode(const std::array<bool, 6>& guiding_mode, bool elbow);


	franka::RobotState robot_state() const;

	//jaw gripper methods
	/** Move the gripper to gripper::max_width. */
	void open_gripper(double speed = default_gripper_speed);
	/** Move the gripper to gripper::min_grasp_width. */
	void close_gripper(double speed = default_gripper_speed);
	/** Grasp. */
	bool grasp_gripper(double speed, double force);

	franka::GripperState gripper_state() const;

	//vacuum gripper methods
	bool vacuum_gripper_drop(std::chrono::milliseconds timeout);
	bool vacuum_gripper_vacuum(
		std::uint8_t vacuum_strength, 
		std::chrono::milliseconds timeout);
	bool vacuum_gripper_stop();

	franka::VacuumGripperState vacuum_gripper_state() const;

	static constexpr double default_gripper_speed = 0.025;


	/**
	 * The robot applies force [mass * g] of the given
	 * @param mass [kg] downwards for the given @param duration
	 */
	void apply_z_force(
		double mass,
		double duration);


	/**
	 * Starts/Stops the recording callback.
	 */
	void start_recording(std::optional<std::string> log_file_path = std::nullopt);
	std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> stop_recording();
	std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> recording_for(
		float seconds,
		std::optional<std::string> log_file_path = std::nullopt);

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
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector,
		const std::array<double, 16>& offset_position,
		const std::array<double, 6>& offset_force);
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector);

	/**
	 * Admittance controller using desired admittance and impedance
	 * rotational and translational stiffness parameters.
	 */
	void apply_admittance(
		double duration, 
		double adm_rotational_stiffness,
		double adm_translational_stiffness, 
		double imp_rotational_stiffness,
		double imp_translational_stiffness,
		const std::optional<std::string>& log_file_path = std::nullopt);

	/**
	 * Cartesian impedance controller to hold the current pose
	 * using desired rotational and translational stiffness parameter.
	 */
	void cartesian_impedance_hold_pose(
		double duration, bool use_stiff_damp_online_calc,
		double rotational_stiffness, double translational_stiffness,
		std::optional<std::string> log_file_path = std::nullopt);
	/**
	 * Cartesian impedance controller to hold multiple poses resp. to follow path of multiple poses
	 * using desired rotational and translational stiffness parameter.
	*/
	void cartesian_impedance_poses(
		const std::list<std::array<double, 16>>& poses, double duration,
		bool use_stiff_damp_online_calc, double rotational_stiffness,
		double translational_stiffness, std::optional<std::string> log_file_path = std::nullopt);
	/**
	 * Joint space impedance controller to hold the current joint position
	 * using desired stiffness matrix parameter.
	 */
	void joint_impedance_hold_position(
		double duration, 
		std::array<double, 49> stiffness, 
		std::optional<std::string> log_file_path = std::nullopt);
	/**
	 * Joint space impedance controller to hold multiple joint positions resp.
	 * to follow path of multiple joint positions using desired stiffness matrix parameter.
	*/
	void joint_impedance_positions(
		const std::list<std::array<double, 7>>& joint_positions,
		double duration,
		std::array<double, 49> stiffness,
		std::optional<std::string> log_file_path = std::nullopt);

	/**
	 *  Runs the pre-defined motion for payload estimation
	*/
	void run_payload_estimation(
		double speed,
		double duration,
		std::optional<std::string> log_file_path = std::nullopt);

	ft_sensor_response fts_state() const;

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
	void set_default_impedance_and_collision_parameters();
	void set_contact_move_impedance_and_collision_parameters();


	// robot
	mutable franka::Robot robot_;

	std::atomic_bool stop_motion_;

	mutable std::mutex speed_factor_lock_;
	double speed_factor_ = 0.05;


	// gripper
	enum class effector
	{
		vacuum_gripper,
		gripper
	};

	mutable std::unique_ptr<franka::Gripper> gripper_;
	double max_width_;
	mutable std::unique_ptr<franka::VacuumGripper> vacuum_gripper_;

	static constexpr double open_epsilon = 0.1;
	static constexpr double min_grasp_width = 0.003;

	// fts
	mutable std::unique_ptr<ft_sensor> ft_sensor_;

	std::unique_ptr<detail::motion_recorder> motion_recorder_;

	// control/state management
	void set_control_loop_running(bool running);
	bool control_loop_running_;
	std::mutex control_loop_running_mutex_;
	std::condition_variable control_loop_running_cv_;

	mutable std::mutex robot_state_lock_;
	franka::RobotState robot_state_;

	mutable std::mutex gripper_state_lock_;
	franka::GripperState gripper_state_;
	franka::VacuumGripperState vacuum_gripper_state_;

	std::atomic_bool terminate_state_threads_;
	std::thread robot_state_thread_;
	std::thread gripper_state_thread_;
};
}

#endif // INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_CONTROLLER_HPP
