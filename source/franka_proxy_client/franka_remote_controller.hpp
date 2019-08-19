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

#include "viral_core/network_forward.hpp"

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

	virtual ~franka_remote_controller() noexcept;


	/**
	 * Moves the Panda robot to given target; In case
	 * of collision, the movement is retried and continued 
	 * after automatic error recovery
	 */
	void move_to(const robot_config_7dof& target);

	void stop_movement();

	robot_config_7dof current_config() const;

	double speed_factor() const;
	void set_speed_factor(double speed_factor);


	/** Move the gripper to gripper::max_width. */
	void open_gripper();
	/** Grasp.... */
	void close_gripper();

	bool gripper_open();


	void update();


private:
	
	void initialize_sockets();
	void shutdown_sockets() noexcept;


	const std::string franka_ip_;

	viral_core::network_context& network_;

	std::unique_ptr<franka_control_client> socket_control_;
	std::unique_ptr<franka_state_client> socket_state_;


	mutable std::mutex state_lock_;
	robot_config_7dof current_config_;
	double current_speed_factor_;
	bool gripper_open_;
	int current_gripper_pos_;
	int max_gripper_pos_;
	int current_error_;


	static constexpr unsigned short franka_controll_port = 4711;
	static constexpr unsigned short franka_state_port = 4712;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP) */