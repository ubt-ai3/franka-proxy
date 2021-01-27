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

message_result franka_control_server::process_message(const message_move_ptp& msg)
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving"
                << std::endl;

    controller_.move_to(msg.config);  
    return message_result::success;
}

message_result franka_control_server::process_message(const message_move_hybrid_sequence& msg) 
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving sequence"
                << std::endl;

	controller_.move_sequence(msg.q_data, msg.f_data, msg.s_data);
    return message_result::success;
} 

message_result franka_control_server::process_message(const message_move_contact& msg)
{
    std::cout << "franka_control_server::process_message(): "
                << "Moving sensitive"
                << std::endl;

    return controller_.move_to_until_contact(msg.config)
            ? message_result::success
            : message_result::success_command_failed;
}

message_result franka_control_server::process_message(const message_force_z& msg) 
{
    controller_.apply_z_force(msg.mass, msg.duration);
    return message_result::success;
}

message_result franka_control_server::process_message(const message_open_gripper& gripper)
{
    std::cout << "franka_control_server::process_message(): " 
                << "Opening Gripper"
                << std::endl;

    controller_.open_gripper();
    return message_result::success;
}

message_result franka_control_server::process_message(const message_close_gripper& gripper)
{
    std::cout << "franka_control_server::process_message(): "
                << "Closing Gripper"
                << std::endl;

    controller_.close_gripper();
    return message_result::success;
}

message_result franka_control_server::process_message(const message_grasping_gripper& msg)
{
    std::cout << "franka_control_server::process_message(): Grasping with Gripper" << std::endl;

    return controller_.grasp_gripper(msg.speed, msg.force)
        ? message_result::success
        : message_result::success_command_failed;
}

message_result franka_control_server::process_message(const message_start_recording& msg)
{
    std::cout << "franka_control_server::process_message(): Start recording" << std::endl;

    controller_.start_recording();
    return message_result::success;
}

message_result franka_control_server::process_message(const message_stop_recording& msg)
{
    std::cout << "franka_control_server::process_request(): Stop recording" << std::endl;

    std::vector<std::array<double, 7>> q_sequence;
    std::vector<std::array<double, 6>> f_sequence;
    
    std::tie(q_sequence, f_sequence) = controller_.stop_recording();
     
    // TODO: Serialize recording.

    controller_.stop_recording();
    return message_result::success;
}

message_result franka_control_server::process_message(const message_speed& msg) 
{
    std::cout << "franka_control_server::process_request(): Setting speed" << std::endl;
    controller_.set_speed_factor(msg.speed);
    // TODO: Bounds checking?
    return message_result::success;
}

message_result franka_control_server::process_message(const message_error_recovery& msg)
{
    std::cout << "franka_control_server::process_request(): Error recovery" << std::endl;
    controller_.automatic_error_recovery();
    return message_result::success;
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
		catch (...)
		{
			std::cerr << "franka_control_server::task_main(): " <<
				"Error while processing requests, dropping stream and stopping robot.";
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

        // TODO: Check if read `content_length` bytes!

        const auto send_response = [this](message_result response) {
            std::cout << "franka_control_server::process_request(): Sending response: " 
                        << static_cast<int>(response) 
                        << std::endl;
            connection_->send(asio::buffer(&response, sizeof(unsigned char)));
        };

        try {
            auto message = nlohmann::json::parse(content); 
            const auto fit = _handlers.find(message["type"]);
            if(fit == _handlers.end())
                ; // TODO: Handle unsupported action!


            const auto& [type, handler] = *fit;

			message_result result;

			try {
				result = handler(this, message);
			} 
			catch(const franka::ControlException&) { result = message_result::control_exception; }
			catch(const franka::CommandException&) { result = message_result::command_exception; }
			catch (const franka::NetworkException&) { result = message_result::network_exception; }

			std::cout << "franka_control_server::receive_requests(): Sending result: "
						<< static_cast<int>(result)
						<< std::endl;

			connection_->send(asio::buffer(&result, sizeof(message_result)));
		}
		catch(...) {
			// TODO: Handle exceptions!
        }
    }
}


void franka_control_server::process_request(const std::string& request)
{
	std::size_t pos = std::string::npos;
	franka_proxy_messages::command_type type = franka_proxy_messages::message_type_count;
	for (int i = 0; i < franka_proxy_messages::message_type_count; ++i)
	{
		pos = request.find(franka_proxy_messages::command_strings[i]);
		if (pos != std::string::npos)
		{
			type = franka_proxy_messages::command_type(i);
			break;
		}
	}

	if (type >= franka_proxy_messages::message_type_count)
	{
		std::cerr << "franka_control_server::process_request(): " <<
			"Invalid message: " << request;
		return;
	}

	std::cout << "franka_control_server::process_request(): " << request;

	std::string parameters = request.substr
		(pos + std::string(franka_proxy_messages::command_strings[type]).size() + 1);

	auto send_response = [this](unsigned char response)
	{
		unsigned char msg[1]; msg[0] = response;
		std::cout << "franka_control_server::process_request(): Sending response: " << static_cast<int>(msg[0]);
		connection_->send(asio::buffer(msg));
	};

	switch (type)
	{


		case franka_proxy_messages::stop_recording:
		{
			std::cout << "franka_control_server::process_request(): Stop recording";

			std::vector<std::array<double, 7>> q_sequence;
			std::vector<std::array<double, 6>> f_sequence;
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						std::tie(q_sequence, f_sequence) =
							controller_.stop_recording();
						return franka_proxy_messages::success;
					});

			std::size_t count[1]; count[0] = q_sequence.size();
			connection_->send(asio::buffer(count));

			for (const auto& p : q_sequence)
			{
				std::string message;
				message += std::to_string(p[0]) + ",";
				message += std::to_string(p[1]) + ",";
				message += std::to_string(p[2]) + ",";
				message += std::to_string(p[3]) + ",";
				message += std::to_string(p[4]) + ",";
				message += std::to_string(p[5]) + ",";
				message += std::to_string(p[6]);
				message += '\n';

				// send size and message
				// todo hton byteorder
				std::size_t size[1]; size[0] = message.size();
				connection_->send(asio::buffer(size));
				connection_->send(asio::buffer(message));

				//if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
				//{
				//	LOG_WARN("Network send buffer is 80 percent used.")
				//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
				//}
			}

			count[0] = f_sequence.size();
			connection_->send(asio::buffer(count));

			for (const auto& f : f_sequence)
			{
				std::string message;
				message += std::to_string(f[0]) + ",";
				message += std::to_string(f[1]) + ",";
				message += std::to_string(f[2]) + ",";
				message += std::to_string(f[3]) + ",";
				message += std::to_string(f[4]) + ",";
				message += std::to_string(f[5]);
				message += '\n';

				// send size and message
				// todo hton byteorder
				std::size_t size[1]; size[0] = message.size();
				connection_->send(asio::buffer(size));
				connection_->send(asio::buffer(message));

				//if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
				//{
				//	LOG_WARN("Network send buffer is 80 percent used.")
				//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
				//}
			}
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::message_type_count:
		default: throw std::exception();
	}
}


std::vector<std::string> franka_control_server::split_string
	(const std::string& s, const std::string& delim)
{
	std::size_t pos_start = 0;
	std::size_t pos_end;
	std::size_t delim_len = delim.length();

	std::string token;
	std::vector<std::string> res;

	while ((pos_end = s.find(delim, pos_start)) != std::string::npos)
	{
		token = s.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		res.push_back(token);
	}

	res.push_back(s.substr(pos_start));
	return res;
}


robot_config_7dof franka_control_server::string_to_robot_config
	(const std::string& s, const std::string& delim)
{
		std::vector<std::string> joint_values = split_string(s, delim);

		if (joint_values.size() != 7)
		{
			std::cerr << "franka_control_server::process_request(): " <<
				"Joint values message does not have 7 joint values.";
			throw std::out_of_range
				("Joint values message does not have 7 joint values.");
		}

		return robot_config_7dof
		{
			std::stod(joint_values[0]),
			std::stod(joint_values[1]),
			std::stod(joint_values[2]),
			std::stod(joint_values[3]),
			std::stod(joint_values[4]),
			std::stod(joint_values[5]),
			std::stod(joint_values[6])
		};
};


std::array<double, 6> franka_control_server::string_to_6_elements
	(const std::string& s, const std::string& delim)
{
		std::vector<std::string> joint_values = split_string(s, delim);

		if (joint_values.size() < 6)
		{
			std::cerr << "franka_control_server::process_request(): " <<
				"Force message does not have at least 6 elements.";
			throw std::out_of_range
				("Message does not have at least 6 elements.");
		}

		return std::array<double, 6>
		{
			std::stod(joint_values[0]),
			std::stod(joint_values[1]),
			std::stod(joint_values[2]),
			std::stod(joint_values[3]),
			std::stod(joint_values[4]),
			std::stod(joint_values[5])
		};
};




} /* namespace franka_proxy */
