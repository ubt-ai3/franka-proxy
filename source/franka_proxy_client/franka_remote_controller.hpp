/**
 *************************************************************************
 *
 * @file franka_remote_controller.hpp
 *
 * Client side implementation of the franka_proxy.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_CONTROLLER_HPP)
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_CONTROLLER_HPP


#include <array>
#include <mutex>
#include <string>

#include <franka_proxy_share/franka_proxy_commands.hpp>

#include "franka_network_client.hpp"


namespace franka_proxy
{


using robot_config_7dof = std::array<double, 7>;


class franka_remote_controller
{
public:

	franka_remote_controller
		(std::string proxy_ip);

	~franka_remote_controller() noexcept;


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
	void move_sequence
		(const std::vector<robot_config_7dof>& q_sequence,
		 const std::vector<std::array<double, 6>>& f_sequence,
		 const std::vector<std::array<double, 6>>& selection_vector_sequence);

	/**
	 * Impedance controller to hold the current position
	*/
	void admittance_apply_force(std::array<double, 6>& desired_force, double duration);

	/**
	 * Impedance controller to hold the current position
	*/
	void impedance_hold_position(double duration);

	/**
	*  Impedance controller to follow path of positions
	*/
	void impedance_follow_positions(std::list<std::array<double, 3>>& positions, double duration);

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
	 * todo docu
	 */
	void start_recording();

	/**
	 * todo docu
	 */
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


	robot_config_7dof current_config() const;
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


private:
	

	template<typename TCommandType>
	using TResponseType = std::conditional_t<
		std::is_same_v<typename TCommandType::response_type, command_generic_response>,
		command_result,
		typename TCommandType::response_type
	>;

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
	 * Throws unknown_command, if the response indicates an error of this type.
	 */
	template<typename TCommandType, typename... TArgs, typename TReturnType = TResponseType<TCommandType>>
	TReturnType send_command(TArgs&&... args)
	{
		const TCommandType cmd{ std::forward<TArgs>(args)... };
		auto response = socket_control_->send_command(cmd);

		if constexpr (std::is_same_v<typename TCommandType::response_type, command_generic_response>)
			return check_response(response);
		else
			return response;
	}

	/**
	 * Checks whether the response indicates that the command was processed successfully.
	 * Otherwise throws an exception indicated by the result code.
	 */
	static command_result check_response(command_generic_response& response);
	
	
	void initialize_sockets();
	void shutdown_sockets() noexcept;

	const std::string franka_ip_;

	std::unique_ptr<franka_control_client> socket_control_;
	std::unique_ptr<franka_state_client> socket_state_;


	mutable std::mutex state_lock_;
	robot_config_7dof current_config_;
	double current_gripper_pos_;
	double max_gripper_pos_;
	bool gripper_grasped_{false};


	static constexpr unsigned short franka_control_port = 4711;
	static constexpr unsigned short franka_state_port = 4712;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_CONTROLLER_HPP) */
