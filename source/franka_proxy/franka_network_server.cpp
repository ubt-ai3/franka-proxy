/**
 *************************************************************************
 *
 * @file franka_network_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_server.hpp"

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
		std::cerr << "franka_control_server::task_main(): " <<
			"Invalid message: " << request;
		return;
	}

	std::cout << "franka_control_server::task_main(): " << request;

	auto split = [](const std::string& s, const std::string& delim) -> std::vector<std::string>
	{
		std::size_t pos_start = 0;
		std::size_t pos_end;
		std::size_t delim_len = delim.length();

		std::string token;
		std::vector<std::string> res;

		while ((pos_end = s.find(delim, pos_start)) != std::string::npos)
		{
			token = s.substr (pos_start, pos_end - pos_start);
			pos_start = pos_end + delim_len;
			res.push_back (token);
		}

		res.push_back (s.substr (pos_start));
		return res;
	};

	switch (type)
	{
		case franka_proxy_messages::move_ptp:
		{
			std::cout << "franka_control_server::task_main(): " << "Moving";

			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size() + 1);
			std::vector<std::string> joint_values = split(rest, " ");
			robot_config_7dof joint_config
			{
				{
					std::stod(joint_values[0]),
					std::stod(joint_values[1]),
					std::stod(joint_values[2]),
					std::stod(joint_values[3]),
					std::stod(joint_values[4]),
					std::stod(joint_values[5]),
					std::stod(joint_values[6])
				}
			};

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.move_to(joint_config);
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::move_hybrid_sequence:
		{
			std::cout << "franka_control_server::task_main(): " << "Moving sequence";

			std::size_t buff[1];
			asio::read(*connection_, asio::buffer(buff));
			std::size_t count = buff[0];
			std::cout << "franka_control_server::task_main(): " << count;


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
				std::vector<std::string> joint_values_list = split(data_buff, ",");

				if (joint_values_list.size() != 7)
				{
					std::cerr << "franka_control_server::task_main(): " <<
						"Joint values message does not have 7 joint values";
					continue;
				}

				std::array<double, 7> joints
				{
					std::stod(joint_values_list[0]),
					std::stod(joint_values_list[1]),
					std::stod(joint_values_list[2]),
					std::stod(joint_values_list[3]),
					std::stod(joint_values_list[4]),
					std::stod(joint_values_list[5]),
					std::stod(joint_values_list[6])
				};


				// emplace position
				q_data.emplace_back(joints);
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
				std::vector<std::string> joint_values_list = split(data_buff, ",");

				if (joint_values_list.size() != 7)
				{
					std::cerr << "franka_control_server::task_main(): " <<
						"Force values message does not have 7 joint values";
					continue;
				}

				std::array<double, 6> joints
				{
					std::stod(joint_values_list[0]),
					std::stod(joint_values_list[1]),
					std::stod(joint_values_list[2]),
					std::stod(joint_values_list[3]),
					std::stod(joint_values_list[4]),
					std::stod(joint_values_list[5])
				};


				// emplace position
				f_data.emplace_back(joints);
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
				std::vector<std::string> joint_values_list = split(data_buff, ",");

				if (joint_values_list.size() != 7)
				{
					std::cerr << "franka_control_server::task_main(): " <<
						"Selection vector message does not have 7 joint values";
					continue;
				}

				std::array<double, 6> joints
				{
					std::stod(joint_values_list[0]),
					std::stod(joint_values_list[1]),
					std::stod(joint_values_list[2]),
					std::stod(joint_values_list[3]),
					std::stod(joint_values_list[4]),
					std::stod(joint_values_list[5])
				};


				// emplace position
				s_data.emplace_back(joints);
			}

			std::cout << "franka_control_server::task_main(): " << "Received data.";

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.move_sequence(q_data, f_data, s_data);
					//controller_.move_hybrid_sequence(data, -5.0);
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));

			break;
		}

		case franka_proxy_messages::move_contact:
		{
			std::cout << "franka_control_server::task_main(): " << "Moving sensitive";
			
			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size() + 1);
			std::vector<std::string> joint_values = split(rest, " ");
			robot_config_7dof joint_config
			{
				{
					std::stod(joint_values[0]),
					std::stod(joint_values[1]),
					std::stod(joint_values[2]),
					std::stod(joint_values[3]),
					std::stod(joint_values[4]),
					std::stod(joint_values[5]),
					std::stod(joint_values[6])
				}
			};

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					if (controller_.move_to_until_contact(joint_config))
						return franka_proxy_messages::success;
					return franka_proxy_messages::success_command_failed;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::force_z:
		{
			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size() + 1);
			std::vector<std::string> parameter = split(rest, " ");
			auto mass = std::stod(parameter[0]);
			auto duration = std::stod(parameter[1]);

			std::cout << "franka_control_server::task_main(): " << std::string("force_z " + std::to_string(mass) + " " + std::to_string(duration));

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.apply_z_force(mass, duration);
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::open_gripper:
		{
			std::cout << "franka_control_server::task_main(): " << "Opening Gripper";

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.open_gripper();
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::close_gripper:
		{
			std::cout << "franka_control_server::task_main(): " << "Closing Gripper";

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.close_gripper();
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::grasp_gripper:
		{
			std::cout << "franka_control_server::task_main(): " << "Grasping with Gripper";
			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size() + 1);
			std::vector<std::string> parameters = split(rest, " ");

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					if (controller_.grasp_gripper(std::stod(parameters[0]), std::stod(parameters[1])))
						return franka_proxy_messages::success;
					return franka_proxy_messages::success_command_failed;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::start_recording:
		{
			std::cout << "franka_control_server::task_main(): " << "Start recording";

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					controller_.start_recording();
					return franka_proxy_messages::success;
				});

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::stop_recording:
		{
			std::cout << "franka_control_server::task_main(): " << "Stop recording";

			std::vector<std::array<double, 7>> q_sequence;
			std::vector<std::array<double, 6>> f_sequence;

			unsigned char response[1];
			response[0] =
				execute_exception_to_return_value
				([&]()
				{
					auto tmp = controller_.stop_recording();
					q_sequence = std::move(tmp.first);
					f_sequence = std::move(tmp.second);
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

			std::cout << "franka_control_server::task_main(): " << "Sending response: " << static_cast<int>(response[0]);
			connection_->send(asio::buffer(response));
			break;
		}

		case franka_proxy_messages::speed:
		{
			std::cout << "franka_control_server::task_main(): " << "Setting speed";
			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size());
			controller_.set_speed_factor(std::stod(rest));
			break;
		}

		case franka_proxy_messages::error_recovery:
		{
			std::cout << "franka_control_server::task_main(): " << "Error recovery";
			std::string rest = request.substr
				(pos + std::string(franka_proxy_messages::command_strings[type]).size());
			controller_.automatic_error_recovery();
			break;
		}

		case franka_proxy_messages::message_type_count:
		default: throw std::exception("unhandled message type");
	}
}




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

			// Send state.
			// conf:j1,j2,j3,j4,j5,j6,j7$<gripper-position>$<gripper-max-position>$<gripper-is-grasped>
			std::string msg("conf:");

			msg += std::to_string(robot_state.q[0]) + ",";
			msg += std::to_string(robot_state.q[1]) + ",";
			msg += std::to_string(robot_state.q[2]) + ",";
			msg += std::to_string(robot_state.q[3]) + ",";
			msg += std::to_string(robot_state.q[4]) + ",";
			msg += std::to_string(robot_state.q[5]) + ",";
			msg += std::to_string(robot_state.q[6]);

			msg += '$';

			msg += std::to_string(gripper_state.width);

			msg += '$';

			msg += std::to_string(gripper_state.max_width);

			msg += '$';

			msg += std::to_string(gripper_state.is_grasped);

			msg += '\n';
			
			asio::write(*connection_, asio::buffer(msg));
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
