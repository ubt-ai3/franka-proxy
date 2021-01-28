/**
 *************************************************************************
 *
 * @file franka_network_control_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_control_server.hpp"

#include <exception>
#include <string>
#include <string_view>
#include <iostream>

#include <asio/read.hpp>
#include <asio/write.hpp>

#include <franka/robot_state.h>
#include <franka/gripper_state.h>

#include <nlohmann/json.hpp>

#include "franka_proxy_share/franka_proxy_messages.hpp"


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_control_server
//
//////////////////////////////////////////////////////////////////////////


franka_control_server::franka_control_server
	(std::uint16_t control_port,
	 franka_hardware_controller& controller)
	:
	controller_(controller),
	server_(create_server(control_port)),
	terminate_internal_thread_(false)
{
    // Register message handlers.
    handles_message<message_move_ptp>();
    handles_message<message_move_contact>();
    handles_message<message_move_hybrid_sequence>();
    handles_message<message_force_z>();
    handles_message<message_open_gripper>();
    handles_message<message_close_gripper>();
    handles_message<message_grasping_gripper>();
    handles_message<message_start_recording>();
    handles_message<message_stop_recording>();
    handles_message<message_speed>();
    handles_message<message_error_recovery>();

	internal_thread_ = std::thread([this]{task_main();});
}


franka_control_server::~franka_control_server() noexcept
{
	terminate_internal_thread_ = true;
	try { internal_thread_.join(); }
	catch (...)
	{
		std::cerr << "franka_control_server::~franka_control_server(): " <<
			"Internal thread threw an exception on joining.";
	}
}

nlohmann::json franka_control_server::process_message(const message_move_ptp& msg)
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving"
                << std::endl;

    controller_.move_to(msg.config);  
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_move_hybrid_sequence& msg)
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving sequence"
                << std::endl;

	controller_.move_sequence(msg.q_data, msg.f_data, msg.s_data);
	return message_generic_response{ message_result::success };
} 

nlohmann::json franka_control_server::process_message(const message_move_contact& msg)
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving sensitive"
                << std::endl;

    const message_result res =  controller_.move_to_until_contact(msg.config)
            ? message_result::success
            : message_result::success_command_failed;

	return message_generic_response{ res };
}

nlohmann::json franka_control_server::process_message(const message_force_z& msg)
{
    controller_.apply_z_force(msg.mass, msg.duration);
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_open_gripper& gripper)
{
    std::cout << "franka_control_server::process_message(): " 
                << "Opening Gripper"
                << std::endl;

    controller_.open_gripper();
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_close_gripper& gripper)
{
    std::cout << "franka_control_server::process_message(): "
                << "Closing Gripper"
                << std::endl;

    controller_.close_gripper();
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_grasping_gripper& msg)
{
    std::cout << "franka_control_server::process_message(): Grasping with Gripper" << std::endl;

	const message_result result = controller_.grasp_gripper(msg.speed, msg.force)
		? message_result::success
		: message_result::success_command_failed;

	return message_generic_response{ result };
}

nlohmann::json franka_control_server::process_message(const message_start_recording& msg)
{
    std::cout << "franka_control_server::process_message(): Start recording" << std::endl;

    controller_.start_recording();
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_stop_recording& msg)
{
    std::cout << "franka_control_server::process_request(): Stop recording" << std::endl;

    std::vector<std::array<double, 7>> q_sequence;
    std::vector<std::array<double, 6>> f_sequence;
    
    message_stop_recording_response resp;
    std::tie(resp.q_sequence, resp.f_sequence) = controller_.stop_recording();

    controller_.stop_recording();

    return resp;
}

nlohmann::json franka_control_server::process_message(const message_speed& msg)
{
    std::cout << "franka_control_server::process_request(): Setting speed" << std::endl;
    controller_.set_speed_factor(msg.speed);
    // TODO: Bounds checking?
	return message_generic_response{ message_result::success };
}

nlohmann::json franka_control_server::process_message(const message_error_recovery& msg)
{
    std::cout << "franka_control_server::process_request(): Error recovery" << std::endl;
    controller_.automatic_error_recovery();
	return message_generic_response{ message_result::success };
}

void franka_control_server::task_main()
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
			// We have an active connection,
			// so handle incoming and outgoing network.
			receive_requests();
		}
		catch (const asio::system_error& exc)
		{
			std::cout << "franka_control_server::task_main(): ";
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
			std::cerr << "franka_control_server::task_main(): " <<
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


asio::ip::tcp::acceptor franka_control_server::create_server
	(std::uint16_t control_port)
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


void franka_control_server::receive_requests()
{
    while(true) {
        std::uint64_t content_length;
        std::size_t bytes_read = asio::read(
            *connection_, 
            asio::buffer(&content_length, sizeof(std::uint64_t))
        );

        std::string content{};
        content.resize(content_length);
        bytes_read = asio::read(*connection_, asio::buffer(content));

        // TODO: Check if content_length was written!

		const auto send_response = [&](const nlohmann::json& response) {
			const std::string dump = response.dump();
			const std::uint64_t content_length = dump.size();

			connection_->send(asio::buffer(&content_length, sizeof(std::uint64_t)));
			connection_->send(asio::buffer(dump));
		};
        
        try {
            auto message = nlohmann::json::parse(content); 
            const auto fit = _handlers.find(message["type"]);

			nlohmann::json response;

			if (fit == _handlers.end()) {
				message_generic_response re{message_result::unknown_operation};
				send_response(re);
				continue;
			}

			try {
				const auto& [type, handler] = *fit;
				response = handler(this, message);
			} 
			catch (const franka::ControlException&) { response = message_generic_response{ message_result::control_exception }; }
			catch (const franka::CommandException&) { response = message_generic_response{ message_result::command_exception }; }
			catch(const franka::NetworkException&) { response = message_generic_response{message_result::network_exception}; }
			catch(const franka::InvalidOperationException&) { response = message_generic_response{message_result::invalid_operation}; }
			catch(const franka::RealtimeException&) { response = message_generic_response{message_result::realtime_exception}; }
			catch(const franka::ModelException&) { response = message_generic_response{message_result::model_exception}; }
			catch(const franka::ProtocolException&) { response = message_generic_response{message_result::protocol_exception}; }
			catch (const franka::IncompatibleVersionException&) { response = message_generic_response{ message_result::incompatible_version}; }
			catch(const franka::Exception&) { response = message_generic_response{message_result::franka_exception}; }

			send_response(response);
		}
		catch(...) {
			// TODO: Handle exceptions!
        }
        
    }
}

} /* namespace franka_proxy */
