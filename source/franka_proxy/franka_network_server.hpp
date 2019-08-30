/**
 *************************************************************************
 *
 * @file franka_network_server.hpp
 *
 * Network communication with a franka_proxy_client.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP


#include <viral_core/network_forward.hpp>


#include <viral_core/auto_pointer.hpp>
#include <viral_core/list.hpp>
#include <viral_core/string.hpp>
#include "viral_core/thread.hpp"
#include "franka_hardware_controller.hpp"
#include "franka_mover.hpp"


namespace franka {
struct GripperState;
struct RobotState;
}


namespace std {
class mutex;
}


namespace franka_proxy
{


/**
 *************************************************************************
 *
 * @class franka_control_server
 *
 * Sends commands to a Franka Emika Panda robot.
 *
 ************************************************************************/
class franka_control_server :
	public viral_core::threaded_task
{

public:

	franka_control_server
		(viral_core::network_context& network,
		 uint16 controll_port,
		 franka_hardware_controller& controller,
		 franka_mover& mover);

	~franka_control_server() NOTHROW;


private:

	void task_main() override;

	void receive_requests();
	void process_request
		(const viral_core::string& request);


	franka_hardware_controller& controller_;
	franka_mover& mover_;

	const viral_core::auto_pointer<viral_core::network_server> server_;
	viral_core::auto_pointer<viral_core::network_stream> stream_;

	static constexpr float sleep_seconds_disconnected_ = 0.01f;
	static constexpr float sleep_seconds_connected_ = 0.002f;
};




/**
 *************************************************************************
 *
 * @class franka_state_server
 *
 * Send the current state of the robot.
 *
 ************************************************************************/
class franka_state_server :
	public viral_core::threaded_task
{

public:

	franka_state_server
		(viral_core::network_context& network,
		 uint16 state_port,
		 const franka_hardware_controller& controller);

	~franka_state_server() noexcept;


private:

	void task_main() override;
	void send_status_message(const viral_core::string& command);


	const franka_hardware_controller& controller_;

	const uint16 state_port_;

	const viral_core::auto_pointer<viral_core::network_server> server_;
	viral_core::auto_pointer<viral_core::network_connection> connection_;

	static constexpr float sleep_seconds_disconnected_ = 0.01f;
	static constexpr float sleep_seconds_connected_ = 0.002f;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP) */
