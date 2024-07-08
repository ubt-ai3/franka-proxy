/**
 *************************************************************************
 *
 * @file franka_network_state_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_state_server.hpp"

#include <string>
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
			const franka::RobotState robot_state = controller_.robot_state();
			const franka::GripperState gripper_state = controller_.gripper_state();
			const franka::VacuumGripperState vacuum_gripper_state_ = controller_.vacuum_gripper_state();

			{	
				//DEBUG
				static bool object_present = false;
				if (vacuum_gripper_state_.part_present != object_present)
				{
					if (vacuum_gripper_state_.part_present)
						std::cout << "object attached\n";
					else
						std::cout << "object detached\n";

				}
				object_present = vacuum_gripper_state_.part_present;

				/*static bool part_detached = false;
				if (vacuum_gripper_state_.part_detached != part_detached)
				{
					if (vacuum_gripper_state_.part_detached)
						std::cout << "detached\n";
					else
						std::cout << "un-detached\n";
				}
				part_detached = vacuum_gripper_state_.part_detached;*/



				if (robot_state.last_motion_errors)
				{
					//these error flags are quite often set, i dont know why 
					//to avoid spamming console comment out
					
					/*auto errors = robot_state.last_motion_errors;
					if (errors.self_collision_avoidance_violation == false)
						std::cout << "no self collision, thats rare!\n";

					std::string error_description = (std::string)errors;
					if (error_description != "[\"self_collision_avoidance_violation\"]");
						std::cout << error_description<<"\n";*/

					//controller_.automatic_error_recovery();
				}
				bool collision = false;
				for (int i = 0; i < 7; i++)
				{
					if (robot_state.joint_collision[i])
					{
						std::cout << "Collision at joint " << i << "detected\n";
						collision = true;
					}
					if(robot_state.joint_contact[i])
						std::cout << "Contact at joint " << i << "detected\n";
				}
				if (collision)
					controller_.automatic_error_recovery();

				if (robot_state.control_command_success_rate  && robot_state.control_command_success_rate < 100)
					std::cout << "Command success rate: " << robot_state.control_command_success_rate << "\n";
				
			}
			command_get_config_response response{};
			response.joint_configuration = robot_state.q;
			response.width = gripper_state.width;
			response.max_width = gripper_state.max_width;
			response.is_grasped = gripper_state.is_grasped;

			response.actual_power = vacuum_gripper_state_.actual_power;
			response.part_detached = vacuum_gripper_state_.part_detached;
			response.part_present = vacuum_gripper_state_.part_present;
			response.vacuum = vacuum_gripper_state_.vacuum;
			response.in_control_range = vacuum_gripper_state_.in_control_range;

			std::string content = nlohmann::json(response).dump();
			const std::uint64_t content_length = content.size();

			asio::write(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));
			asio::write(*connection_, asio::buffer(content));
		}

		catch (const asio::system_error& exc)
		{
			std::cout << "franka_state_server::task_main(): ";
			if (exc.code() == asio::error::connection_reset)
				std::cout << " The connection was reset by the client. Dropping stream and stopping robot." << std::endl;
			else if (exc.code() == asio::error::connection_aborted)
				std::cout << " The connection was aborted. Dropping stream and stopping robot." << std::endl;
			else if (exc.code() == asio::error::timed_out)
				std::cout << " The connection timed out. Dropping stream and stopping robot." << std::endl;
			else
				std::cout << "Unknown connection error. Dropping stream and stopping robot." << std::endl;

			controller_.stop_movement();
			connection_.reset();
		}
		catch (const std::exception& exc)
		{
			std::cerr << "franka_state_server::task_main(): " <<
				"An exception occurred while processing requests, dropping stream and stopping robot. " <<
				std::endl << "Exception message: " << exc.what() << std::endl;

			controller_.stop_movement();
			connection_.reset();
		}
		catch (...)
		{
			std::cerr << "franka_control_server::task_main(): " <<
				"An unknown error occured while processing requests, dropping stream and stopping robot." << std::endl;

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
