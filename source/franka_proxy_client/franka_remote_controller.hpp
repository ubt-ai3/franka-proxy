/**
 *************************************************************************
 *
 * @file franka_remote_controller.hpp
 *
 * Client side implementation of the franka_proxy.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP)
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP


#include <array>
#include <string>
#include <mutex>

#include <viral_core/network_forward.hpp>

#include "franka_proxy_share/franka_proxy_messages.hpp"

#include "franka_network_client.hpp"


namespace franka_proxy
{


typedef std::array<double, 7> robot_config_7dof;


class franka_remote_controller
{

public:

	franka_remote_controller
		(const std::string& proxy_ip,
		 viral_core::network_context& network);

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
	void open_gripper();


	/**
	 * Close the gripper.
	 *
	 * Returns if the movement was completed successfully.
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
	void close_gripper(double speed, double force);


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
	int current_gripper_pos() const;
	int max_gripper_pos() const;


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
	
	void initialize_sockets();
	void shutdown_sockets() noexcept;

	void check_response(franka_proxy_messages::feedback_type response);


	const std::string franka_ip_;

	viral_core::network_context& network_;

	std::unique_ptr<franka_control_client> socket_control_;
	std::unique_ptr<franka_state_client> socket_state_;


	mutable std::mutex state_lock_;
	robot_config_7dof current_config_;
	int current_gripper_pos_;
	int max_gripper_pos_;


	static constexpr unsigned short franka_control_port = 4711;
	static constexpr unsigned short franka_state_port = 4712;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP) */