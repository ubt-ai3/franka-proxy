#ifndef INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_INTERFACE_HPP
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_INTERFACE_HPP
/**
 *************************************************************************
 *
 * @file franka_remote_interface.hpp
 *
 * Client side implementation of the franka_proxy.
 *
 ************************************************************************/


#include <array>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include <franka_proxy_share/franka_proxy_commands.hpp>

#include "franka_network_client.hpp"


namespace franka_proxy
{
using robot_config_7dof = std::array<double, 7>;

// Corresponds to libfranka::VacuumGripperState.
struct vacuum_gripper_state
{
	uint16_t actual_power_;
	uint16_t vacuum_level;
	bool part_detached_;
	bool part_present_;
	bool in_control_range_;
};

class franka_remote_interface
{
public:
	explicit franka_remote_interface(std::string proxy_ip);

	~franka_remote_interface() noexcept;


	/**
	 * Start control-loop to move the robot to given target.
	 *
	 * Returns if the movement was completed successfully.
	 * Throws some remote_exception on failure.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void move_to(const robot_config_7dof& target);
	void move_to(const Eigen::Vector<double, 7>& target);


	/**
	 * Start control-loop to move the robot to given target.
	 *
	 * Returns if the movement was completed successfully.
	 * Throws some remote_exception on failure.
	 *
	 * The motion will stop on contact. If the motions was
	 * stopped because contact, false is returned.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	bool move_to_until_contact(const robot_config_7dof& target);


	/**
	 * todo docu
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<std::array<double, 6>>& f_sequence,
		const std::vector<std::array<double, 6>>& selection_vector_sequence,
		const std::array<double, 16>& offset_position,
		const std::array<double, 6>& offset_force);


	/**
	 * todo docu
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<std::array<double, 6>>& f_sequence,
		const std::vector<std::array<double, 6>>& selection_vector_sequence);

	/**
	 * Admittance controller using desired rotational and translational stiffness within the admittance and the impedance controller
	*/
	void apply_admittance(
		double duration, double adm_rotational_stiffness,
		double adm_translational_stiffness, double imp_rotational_stiffness,
		double imp_translational_stiffness, std::optional<std::string> log_file_path = std::nullopt);

	/**
	 * Cartesian impedance controller to hold the current pose with desired rotational and translational stiffness
	*/
	void cartesian_impedance_hold_pose(
		double duration, bool use_stiff_damp_online_calc,
		double rotational_stiffness, double translational_stiffness,
		std::optional<std::string> log_file_path = std::nullopt);

	/**
	*  Cartesian impedance controller to follow path of poses with desired rotational and translational stiffness
	*  Duration parameter: duration to follow the complete path -> Example: 10s duration, 5 poses -> 2s per pose
	*/
	void cartesian_impedance_poses(
		std::list<std::array<double, 16>>& positions,
		double duration,
		bool use_stiff_damp_online_calc,
		double rotational_stiffness,
		double translational_stiffness,
		std::optional<std::string> log_file_path = std::nullopt);

	/**
	 * Joint space impedance controller to hold the current position with desired stiffness matrix parameter
	*/
	void joint_impedance_hold_position(
		double duration,
		std::array<double, 49> stiffness,
		std::optional<std::string> log_file_path = std::nullopt);

	/**
	*  Joint space impedance controller to follow path of positions with desired stiffness matrix parameter
	*	Duration parameter: duration to follow the complete path -> Example: 10s duration, 5 positions -> 2s per position
	*/
	void joint_impedance_positions(std::list<std::array<double, 7>>& joint_positions,
	                               double duration,
	                               std::array<double, 49> stiffness,
	                               std::optional<std::string> log_file_path = std::nullopt);

	/**
	 * Joint space impedance controller for executing a pre-defined motion for payload estimation
	*/
	void ple_motion(double speed, double duration, std::optional<std::string> log_file_path = std::nullopt);

	/**
	 * todo docu
	 * todo change to newton
	 */
	void apply_z_force(double mass, double duration);


	/**
	 * Open the gripper by moving it to max_width.
	 *
	 * Returns if the movement was completed successfully.
	 * Throws some remote_exception on failure.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void open_gripper(double speed = 0.025);


	/**
	 * todo
	 */
	void close_gripper(double speed = 0.025);


	/**
	 * Attempt to grasp an object.
	 * Returns if the grasp was completed successfully.
	 * Throws some remote_exception on failure.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @param[in] speed Closing speed. [m/s]
	 * @param[in] force Grasping force. [N]
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	bool grasp_gripper(double speed = 0.025, double force = 0.05);

	/**
	 * Drops objects by stopping then vacuum.
	 * Remote function for franka::VacuumGripper::drop
	 * 
	 * Returns if the movement was completed successfully.
	 * Throws some remote_exception on failure.
	 * 
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	bool vacuum_gripper_drop(
		std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

	/**
	 * Creates a vacuum with the vacuum gripper in order to grip objects.
	 * Remote function for franka::VacuumGripper::vacuum.

	 *	returns if the vacuum was created successful
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the movement was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	bool vacuum_gripper_vacuum(
		std::uint8_t vacuum_strength,
		std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

	/**
	* Stops the current vaccum gripper command
	*
	*	remote function for franka::VacuumGripper::stop
	*
	* @TODO: Check exceptions.
	*
	* @throw remote_exception if the movement was unsuccessful.
	* @throw viral_core::network_exception if the connection was lost.
	*/

	bool vacuum_gripper_stop();


	/**
	 * todo docu
	 */
	void start_recording(std::optional<std::string> log_file_path = std::nullopt);
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop_recording();


	/**
	 * Send new target speed to robot.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @param[in] speed_factor Target speed factor, normalized between 0 and 1.
	 *
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void set_speed_factor(double speed_factor);

	/**
	* Send new force/torque sensor bias
	*
	* @TODO: Check exceptions.
	*
	* @param[in] bias of the ft sensor (fx, fy, fz, tx, ty, tz)
	*
	*  @throw viral_core::force_torque_sensor_exception if force/torque sensor is unavailable.
	*/
	void set_fts_bias(const std::array<double, 6>& bias);


	/**
	* Send new load mass affecting the force/torque sensor
	*
	* @TODO: Check exceptions.
	*
	* @param[in] load_mass: force in world coordinates produced by load mass
	*
	* @throw viral_core::force_torque_sensor_exception if force/torque sensor is unavailable.
	*/
	void set_fts_load_mass(const std::array<double, 3>& load_mass);


	/**
	 * Runs automatic error recovery on the robot.
	 *
	 * Automatic error recovery e.g. resets the robot after a collision occurred.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw remote_exception if the command was unsuccessful.
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void automatic_error_recovery();

	/**
	* Send which cartesian DOF are mobile robot
	* 
	* @param[in] vector with bool values for every DOF; true equals mobile and false equals immobile
	* 
	*/
	void set_guiding_params(
		bool x, bool y, bool z,
		bool rx, bool ry, bool rz, bool elbow);

	robot_config_7dof current_config() const;
	std::array<double, 6> current_end_effector_wrench() const;
	double current_gripper_pos() const;
	double max_gripper_pos() const;
	bool gripper_grasped() const;


	/**
	 * Update internal robot state via the network.
	 * Should be called regularly.
	 *
	 * @TODO: Check exceptions.
	 *
	 * @throw viral_core::network_exception if the connection was lost.
	 */
	void update();

	/*
		returns the current vacuum gripper state
	*/
	vacuum_gripper_state get_vacuum_gripper_state() const;

private:
	template <typename TCommandType> using TResponseType = std::conditional_t<
		std::is_same_v<typename TCommandType::response_type, command_generic_response>,
		command_result,
		typename TCommandType::response_type>;

	/**
	 * Constructs an command in-place with given arguments, and checks the response code,
	 * if the response is of type `command_generic_response`.
	 *
	 * If the response type is 'command_generic_response', then the following exceptions are thrown in addition:
	 *
	 * Throws model_exception, if the response indicates an error of this type.
	 * Throws network_exception, if the response indicates an error of this type.
	 * Throws protocol_exception, if the response indicates an error of this type.
	 * Throws incompatible_version, if the response indicates an error of this type.
	 * Throws control_exception, if the response indicates an error of this type.
	 * Throws command_exception, if the response indicates an error of this type.
	 * Throws realtime_exception, if the response indicates an error of this type.
	 * Throws invalid_operation, if the response indicates an error of this type.
	 * Throws force_torque_sensor_exception, if the response indicates an error of this type.
	 * Throws unknown_command, if the response indicates an error of this type.
	 */
	template <typename TCommandType, typename... TArgs, typename TReturnType = TResponseType<TCommandType>> TReturnType
	send_command(TArgs&&... args)
	{
		const TCommandType cmd{std::forward<TArgs>(args)...};
		auto response = socket_control_->send_command(cmd);

		if constexpr (std::is_same_v<typename TCommandType::response_type, command_generic_response>)
			return check_response(response);
		else
			return response;
	}

	/**
	 * Checks whether the response indicates that the command was processed successfully.
	 * Otherwise, throws an exception indicated by the result code.
	 */
	static command_result check_response(const command_generic_response& response);


	void initialize_sockets();
	void shutdown_sockets() noexcept;


	const std::string franka_ip_;
	static constexpr unsigned short franka_control_port = 4711;
	static constexpr unsigned short franka_state_port = 4712;

	std::unique_ptr<franka_control_client> socket_control_;
	std::unique_ptr<franka_state_client> socket_state_;

	mutable std::mutex state_lock_;
	robot_config_7dof current_config_;
	std::array<double, 6> current_end_effector_wrench_;
	double current_gripper_pos_;
	double max_gripper_pos_;
	bool gripper_grasped_{false};
	vacuum_gripper_state vacuum_gripper_state_;
};
}

#endif // INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_INTERFACE_HPP
