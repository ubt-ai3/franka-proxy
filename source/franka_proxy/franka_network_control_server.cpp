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
	// Receive input into string buffer.
	std::string receive_buffer(receive_buffer_size_, '\0');
	std::size_t bytes_received =
		connection_->read_some(asio::buffer(receive_buffer));
	
	messages_buffer_ +=
		std::string(receive_buffer, 0, bytes_received);

	// Recover one completely transmitted request,
	// and remove it from the network stream.

	// Extract full request up to next end marker.
	// Transferring the request has not yet finished
	// if there is no (further) end marker.
	while(true)
	{
		std::size_t end_index =
			messages_buffer_.find(franka_proxy_messages::command_end_marker);

		if (end_index == std::string::npos)
			return;

		std::string request_string
			(messages_buffer_.substr(0, end_index));

		// Remove request from network stream.
		messages_buffer_ =
			messages_buffer_.substr(end_index + 1);


		process_request(request_string);
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
		case franka_proxy_messages::move_ptp:
		{
			std::cout << "franka_control_server::process_request(): " << "Moving";

			robot_config_7dof joint_config =
				string_to_robot_config(parameters, " ");

			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.move_to(joint_config);
						return franka_proxy_messages::success;
					});
			send_response(response);
			break;
		}

		case franka_proxy_messages::move_hybrid_sequence:
		{
			std::cout << "franka_control_server::process_request(): " << "Moving sequence";

			std::size_t buff[1];
			asio::read(*connection_, asio::buffer(buff));
			std::size_t count = buff[0];
			std::cout << "franka_control_server::process_request(): " << count;


			std::vector<std::array<double, 7>> q_data;
			q_data.reserve(count);
			for (std::size_t i = 0; i < count; ++i)
			{
				// size
				std::size_t size_buff[1];
				asio::read(*connection_, asio::buffer(size_buff));
				std::size_t size = size_buff[0];
				// todo ntoh byteorder

				// data
				std::string data_buff(size, '\0');
				asio::read(*connection_, asio::buffer(data_buff));

				// extract data
				q_data.emplace_back
					(string_to_robot_config(data_buff, ","));
			}

			std::vector<std::array<double, 6>> f_data;
			f_data.reserve(count);
			for (std::size_t i = 0; i < count; ++i)
			{
				// size
				std::size_t size_buff[1];
				asio::read(*connection_, asio::buffer(size_buff));
				std::size_t size = size_buff[0];
				// todo ntoh byteorder

				// data
				std::string data_buff(size, '\0');
				asio::read(*connection_, asio::buffer(data_buff));

				// extract data
				f_data.emplace_back
					(string_to_6_elements(data_buff, ","));
			}

			std::vector<std::array<double, 6>> s_data;
			s_data.reserve(count);
			for (std::size_t i = 0; i < count; ++i)
			{
				// size
				std::size_t size_buff[1];
				asio::read(*connection_, asio::buffer(size_buff));
				std::size_t size = size_buff[0];
				// todo ntoh byteorder

				// data
				std::string data_buff(size, '\0');
				asio::read(*connection_, asio::buffer(data_buff));

				// extract data
				s_data.emplace_back
					(string_to_6_elements(data_buff, ","));
			}

			std::cout << "franka_control_server::process_request(): " << "Received data.";

			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.move_sequence(q_data, f_data, s_data);
						//controller_.move_hybrid_sequence(data, -5.0);
						return franka_proxy_messages::success;
					});
			send_response(response);

			break;
		}

		case franka_proxy_messages::move_contact:
		{
			std::cout << "franka_control_server::process_request(): " << "Moving sensitive";

			robot_config_7dof joint_config =
				string_to_robot_config(parameters, " ");

			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						if (controller_.move_to_until_contact(joint_config))
							return franka_proxy_messages::success;
						return franka_proxy_messages::success_command_failed;
					});
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::force_z:
		{
			std::vector<std::string> parameters_split =
				split_string(parameters, " ");
			auto mass = std::stod(parameters_split[0]);
			auto duration = std::stod(parameters_split[1]);

			std::cout << "franka_control_server::process_request(): " << std::string("force_z " + std::to_string(mass) + " " + std::to_string(duration));
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.apply_z_force(mass, duration);
						return franka_proxy_messages::success;
					});
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::open_gripper:
		{
			std::cout << "franka_control_server::process_request(): " << "Opening Gripper";
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.open_gripper();
						return franka_proxy_messages::success;
					});
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::close_gripper:
		{
			std::cout << "franka_control_server::process_request(): " << "Closing Gripper";
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.close_gripper();
						return franka_proxy_messages::success;
					});
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::grasp_gripper:
		{
			std::cout << "franka_control_server::process_request(): Grasping with Gripper";
			
			std::vector<std::string> parameters_split =
				split_string(parameters, " ");
			auto speed = std::stod(parameters_split[0]);
			auto force = std::stod(parameters_split[1]);
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						if (controller_.grasp_gripper(speed, force))
							return franka_proxy_messages::success;
						return franka_proxy_messages::success_command_failed;
					});
			send_response(response);
			
			break;
		}

		case franka_proxy_messages::start_recording:
		{
			std::cout << "franka_control_server::process_request(): Start recording";
			
			unsigned char response = execute_exception_to_return_value
				([&]()
					{
						controller_.start_recording();
						return franka_proxy_messages::success;
					});
			send_response(response);
			
			break;
		}

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

		case franka_proxy_messages::speed:
		{
			std::cout << "franka_control_server::process_request(): Setting speed";
			controller_.set_speed_factor(std::stod(parameters));
			break;
		}

		case franka_proxy_messages::error_recovery:
		{
			std::cout << "franka_control_server::process_request(): Error recovery";
			controller_.automatic_error_recovery();
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