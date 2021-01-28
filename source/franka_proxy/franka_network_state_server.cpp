/**
 *************************************************************************
 *
 * @file franka_network_state_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_state_server.hpp"
#include <franka_proxy_share/franka_proxy_messages.hpp>

#include <string>
#include <iostream>

#include <asio/write.hpp>

#include <franka/robot_state.h>
#include <franka/gripper_state.h>

#include <nlohmann/json.hpp>


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_state_server
//
//////////////////////////////////////////////////////////////////////////


franka_state_server::franka_state_server
	(std::uint16_t state_port,
	 franka_hardware_controller& controller)
	:
	controller_(controller),

	state_port_(state_port),
	server_(create_server(state_port_)),

	terminate_internal_thread_(false)
{
	internal_thread_ = std::thread([this]{task_main();});
}


franka_state_server::~franka_state_server() noexcept
{
	terminate_internal_thread_ = true;
	try { internal_thread_.join(); }
	catch (...)
	{
		std::cerr << "franka_state_server::~franka_state_server(): " <<
			"Internal thread threw an exception on joining.";
	}
}


void franka_state_server::task_main()
{
	while (!terminate_internal_thread_)
	{
		// Possibly accept new connection
		// and drop preceding connection.
		std::error_code error;
		auto connection =
			std::make_unique<asio::ip::tcp::socket>(server_.accept(error));

		if(!error)
			connection_ = std::move(connection);

		if (!connection_)
		{
			std::this_thread::sleep_for
				(std::chrono::duration<double>(sleep_seconds_disconnected_));

			continue;
		}

		try
		{
			// Copy status
			franka::RobotState robot_state = controller_.robot_state();
			franka::GripperState gripper_state = controller_.gripper_state();

            message_robot_state msg;
            msg.q = robot_state.q;
            msg.width = gripper_state.width;
            msg.max_width = gripper_state.max_width;
            msg.is_grasped = gripper_state.is_grasped;
			
            std::string dump = nlohmann::json(msg).dump(); 
            const std::uint64_t content_length = dump.size();

            asio::write(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));
            asio::write(*connection_, asio::buffer(dump));
		}
		catch (...)
		{
			std::cerr << "franka_state_server::task_main(): " <<
				"Error while sending status, dropping stream and stopping robot.";
			controller_.stop_movement();
			connection_.reset();
		}

		std::this_thread::sleep_for
			(std::chrono::duration<double>(sleep_seconds_connected_));
	}
}


asio::ip::tcp::acceptor franka_state_server::create_server(std::uint16_t control_port)
{
	asio::ip::tcp::acceptor acceptor(io_context_);
	asio::ip::tcp::endpoint endpoint(asio::ip::tcp::v4(), control_port);
	acceptor.open(endpoint.protocol());
	acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true));
	acceptor.non_blocking(true);
	acceptor.bind(endpoint);
	acceptor.listen();

	return acceptor;
}




} /* namespace franka_proxy */
