#ifndef INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_STATE_SERVER_HPP
#define INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_STATE_SERVER_HPP
/**
 *************************************************************************
 *
 * @file franka_network_state_server.hpp
 *
 * Network communication with a franka_proxy_client.
 *
 ************************************************************************/



#include <asio/ip/tcp.hpp>

#include "franka_hardware_controller.hpp"


namespace franka_proxy
{


/**
 *************************************************************************
 *
 * @class franka_state_server
 *
 * Send the current state of the robot.
 *
 ************************************************************************/
class franka_state_server
{
public:

	franka_state_server
		(std::uint16_t state_port,
		 franka_hardware_controller& controller);

	~franka_state_server() noexcept;


private:

	void task_main();

	asio::ip::tcp::acceptor create_server(std::uint16_t control_port);


	franka_hardware_controller& controller_;

	const std::uint16_t state_port_;

	asio::io_context io_context_;
	asio::ip::tcp::acceptor server_;
	std::unique_ptr<asio::ip::tcp::socket> connection_;

	std::thread internal_thread_;
	std::atomic_bool terminate_internal_thread_;

	static constexpr float sleep_seconds_disconnected_ = 0.033f;
	static constexpr float sleep_seconds_connected_ = 0.033f;
};


} /* namespace franka_proxy */

#endif // INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_STATE_SERVER_HPP
