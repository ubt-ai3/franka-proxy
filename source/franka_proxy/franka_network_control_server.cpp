/**
 *************************************************************************
 *
 * @file franka_network_control_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include <franka_proxy/franka_network_control_server.hpp>

#include <exception>
#include <string>
#include <string_view>
#include <iostream>

#include <asio/read.hpp>
#include <asio/write.hpp>

#include <franka/robot_state.h>
#include <franka/gripper_state.h>

#include <nlohmann/json.hpp>

namespace
{
	using namespace franka_proxy;

	/**
	 * Executes a given functor with given arguments and decays exceptions to meaningful responses.
	 */
	template<typename TFunctor, typename... TArgs>
	nlohmann::json execute_safe(TFunctor functor, TArgs... args)
	{
		try
		{
			return functor(std::forward<TArgs>(args)...);
		}
		catch (const franka::ControlException&) { return command_generic_response{ command_result::control_exception }; }
		catch (const franka::CommandException&) { return command_generic_response{ command_result::command_exception }; }
		catch (const franka::NetworkException&) { return command_generic_response{ command_result::network_exception }; }
		catch (const franka::RealtimeException&) { return command_generic_response{ command_result::realtime_exception }; }
		catch (const franka::ModelException&) { return command_generic_response{ command_result::model_exception }; }
		catch (const franka::ProtocolException&) { return command_generic_response{ command_result::protocol_exception }; }
		catch (const franka::IncompatibleVersionException&) { return command_generic_response{ command_result::incompatible_version }; }
		catch (const franka::Exception&) { return command_generic_response{ command_result::franka_exception };  }
	}

	
}

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
    // Register command handlers.
	register_command_handler<command_move_to_config>();
	register_command_handler<command_move_hybrid_sequence>();
	register_command_handler<command_move_until_contact>();
	register_command_handler<command_force_z>();
	register_command_handler<command_open_gripper>();
	register_command_handler<command_close_gripper>();
	register_command_handler<command_grasp_gripper>();
	register_command_handler<command_start_recording>();
	register_command_handler<command_stop_recording>();
	register_command_handler<command_set_speed>();
	register_command_handler<command_recover_from_errors>();
		
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


command_generic_response franka_control_server::process_command(const command_move_to_config& cmd)
{
	controller_.move_to(cmd.target_joint_config);
	return command_result::success;
}


command_generic_response franka_control_server::process_command(const command_move_hybrid_sequence& cmd)
{
	controller_.move_sequence(cmd.joint_config_sequence, cmd.force_sequence, cmd.selection_sequence);
	return command_result::success;
}

command_generic_response franka_control_server::process_command(const command_move_until_contact& cmd)
{
	return controller_.move_to_until_contact(cmd.target_joint_config)
		? command_result::success
		: command_result::success_command_failed;
}

command_generic_response franka_control_server::process_command(const command_force_z& cmd)
{
	controller_.apply_z_force(cmd.mass, cmd.duration);
	return command_result::success;
}

command_generic_response franka_control_server::process_command(const command_open_gripper&)
{
	controller_.open_gripper();
	return command_result::success;
}

command_generic_response franka_control_server::process_command(const command_close_gripper&)
{
	controller_.close_gripper();
	return command_result::success;
}

command_generic_response franka_control_server::process_command(const command_grasp_gripper& cmd)
{
	return controller_.grasp_gripper(cmd.speed, cmd.force)
		? command_result::success
		: command_result::success_command_failed;
}

command_generic_response franka_control_server::process_command(const command_start_recording&)
{
	controller_.start_recording();
	return command_result::success;
}

command_stop_recording_response franka_control_server::process_command(const command_stop_recording&)
{
	const auto&[q_seq, f_seq] = controller_.stop_recording();
	return { q_seq, f_seq };
}

command_generic_response franka_control_server::process_command(const command_set_speed& cmd)
{
	controller_.set_speed_factor(cmd.speed);
	return command_result::success;
}

command_generic_response franka_control_server::process_command(const command_recover_from_errors&)
{
	controller_.automatic_error_recovery();
	return command_result::success;
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

    std::uint64_t content_length;
    asio::read(
        *connection_, 
        asio::buffer(&content_length, sizeof(std::uint64_t))
    );

    std::string content{};
    content.resize(content_length);
    asio::read(*connection_, asio::buffer(content));

	const auto send_response = [&](const nlohmann::json& response) {
		const std::string dump = response.dump();
		const std::uint64_t content_length = dump.size();

		connection_->send(asio::buffer(&content_length, sizeof(std::uint64_t)));
		connection_->send(asio::buffer(dump));
	};

	const auto message = nlohmann::json::parse(content);
	const auto fit = _command_handlers.find(message.value("type", ""));
	if (fit == _command_handlers.end()) {
		command_generic_response response{ command_result::unknown_command };
		send_response(response);
		return;
	}

	const auto response = execute_safe(fit->second, this, message);
	send_response(response);
}

} /* namespace franka_proxy */
