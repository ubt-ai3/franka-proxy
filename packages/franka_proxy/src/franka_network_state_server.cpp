/**
 *************************************************************************
 *
 * @file franka_network_state_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_state_server.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <iostream>

#include <asio/write.hpp>

#include <franka/robot_state.h>
#include <franka/gripper_state.h>

#include <nlohmann/json.hpp>

#include <franka_proxy_share/franka_proxy_commands.hpp>


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
		auto next_iteration_time =
			std::chrono::steady_clock::now();
		// Possibly accept new connection
		// and drop preceding connection.
		std::error_code error;
		auto connection =
			std::make_unique<asio::ip::tcp::socket>(server_.accept(error));

		if(!error)
			connection_ = std::move(connection);

		if (!connection_)
		{
			next_iteration_time += step_duration_disconnected;
			std::this_thread::sleep_until(next_iteration_time);

			continue;
		}

		try
		{
			const franka::RobotState robot_state = controller_.robot_state();
			const franka::GripperState gripper_state = controller_.gripper_state();
			const franka::VacuumGripperState vacuum_gripper_state = controller_.vacuum_gripper_state();

			command_get_config_response response{};
			response.joint_configuration = robot_state.q;
			response.width = gripper_state.width;
			response.max_width = gripper_state.max_width;
			response.is_grasped = gripper_state.is_grasped;

			response.actual_power = vacuum_gripper_state.actual_power;
			response.part_detached = vacuum_gripper_state.part_detached;
			response.part_present = vacuum_gripper_state.part_present;
			response.vacuum = vacuum_gripper_state.vacuum;
			response.in_control_range = vacuum_gripper_state.in_control_range;

			std::string content = nlohmann::json(response).dump();
			const std::uint64_t content_length = content.size();

			write(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));
			write(*connection_, asio::buffer(content));
		}
		catch (const asio::system_error& e)
		{
			std::cout << "franka_state_server::task_main(): ";
			if (e.code() == asio::error::connection_reset)
				std::cout << "The connection was reset by the client. Dropping stream and stopping robot." << '\n';
			else if (e.code() == asio::error::connection_aborted)
				std::cout << "The connection was aborted. Dropping stream and stopping robot." << '\n';
			else if (e.code() == asio::error::timed_out)
				std::cout << "The connection timed out. Dropping stream and stopping robot." << '\n';
			else
				std::cout << "Unknown connection error. Dropping stream and stopping robot." << '\n';

			controller_.stop_movement();
			connection_.reset();
		}
		catch (const std::exception& e)
		{
			std::cerr << "franka_state_server::task_main(): " <<
				"An exception occurred while processing requests, " <<
				"dropping stream and stopping robot. \n" <<
				"Exception message: " << e.what() << '\n';

			controller_.stop_movement();
			connection_.reset();
		}
		catch (...)
		{
			std::cerr << "franka_control_server::task_main(): " <<
				"An unknown error occured while processing requests, dropping stream and stopping robot." << '\n';

			controller_.stop_movement();
			connection_.reset();
		}

		next_iteration_time += step_duration_connected;

		auto current_time_point_just_for_debugging =
			std::chrono::steady_clock::now();

		std::this_thread::sleep_until(next_iteration_time);
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
