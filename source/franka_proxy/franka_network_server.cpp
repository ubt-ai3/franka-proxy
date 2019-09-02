/**
 *************************************************************************
 *
 * @file franka_network_server.cpp
 *
 * Network communication with a franka_proxy_client, implementation.
 *
 ************************************************************************/


#include "franka_network_server.hpp"

#include "viral_core/network.hpp"
#include "viral_core/network_stream.hpp"
#include "viral_core/log.hpp"
#include <mutex>
#include <franka/robot_state.h>
#include <franka/gripper_state.h>
#include "viral_core/network_transfer.hpp"
#include <string>
#include "franka_proxy_share/franka_proxy_messages.hpp"
#include <franka/exception.h>


namespace franka_proxy
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_control_server
//
//////////////////////////////////////////////////////////////////////////


franka_control_server::franka_control_server
	(network_context& network,
	 uint16 controll_port,
	 franka_hardware_controller& controller)
	:
	controller_(controller),

	server_(network.create_server(controll_port))
{
	start();
}


franka_control_server::~franka_control_server() noexcept
{	
	try { join(); }
	catch(...)
		{ LOG_CRITICAL("Internal thread threw an exception on joining."); }
}


void franka_control_server::task_main()
{
	while (!join_now())
	{
		// Possibly accept new connection
		// and drop preceding connection.
		auto_pointer<network_connection> connection
			(server_->try_accept_connection());

		if (connection)
			stream_.reset
				(new network_stream
					(std::move(connection)));
		
		if (!stream_)
		{
			thread_util::sleep_seconds
				(sleep_seconds_connected_);

			continue;
		}


		try
		{
			// We have an active connection,
			// so handle incoming and outgoing network.
			stream_->update();

			receive_requests();
		}
		catch (...)
		{
			LOG_ERROR("Error while receiving requests, dropping stream and stopping robot.");
			controller_.stop_movement();
			stream_.reset();
		}

		thread_util::sleep_seconds
			(sleep_seconds_disconnected_);
	}
}


void franka_control_server::receive_requests()
{
	// Early out if no input is pending.
	if (stream_->pending_receive_bytes() == 0)
		return;
	

	// Peek all pending input into string buffer.
	string buffer;
	buffer.resize(stream_->pending_receive_bytes());

	if (!stream_->try_peek_nonblocking
			(reinterpret_cast<unsigned char*>(buffer.data()),
			 buffer.size(), 0))
	{
		LOG_ERROR("Failed to fetch pending bytes from network stream.");
		throw assert_failed();
	}


	// Recover any completely transmitted requests,
	// and remove these from the network stream.
	while (true)
	{

		// Extract full request up to next end marker.
		// Transferring the request has not yet finished
		// if there is no (further) end marker.
		int64 end_index =
			buffer.seek(franka_proxy_messages::command_end_marker);

		if (end_index == string::invalid_index)
			break;

		string request_string
			(buffer.left(end_index).trim());


		process_request(request_string);


		// Remove request from local buffer
		// and from network stream.
		buffer =
			buffer.substring
				(end_index + string::size(franka_proxy_messages::command_end_marker));
	
		stream_->discard_receive
			(end_index + string::size(franka_proxy_messages::command_end_marker));
	}
}


void franka_control_server::process_request(const string& request)
{
	int64 pos = string::invalid_index;
	franka_proxy_messages::command_type type = franka_proxy_messages::message_type_count;
	for (int i = 0; i < franka_proxy_messages::message_type_count; ++i)
	{
		pos = request.seek(franka_proxy_messages::command_strings[i]);
		if (pos != string::invalid_index)
		{
			type = franka_proxy_messages::command_type(i);
			break;
		}
	}

	if (type == franka_proxy_messages::message_type_count)
	{
		LOG_WARN("Invalid message: " + request);
		return;
	}
	
	LOG_INFO(request)

	switch(type)
	{
		case franka_proxy_messages::move:
		{
			LOG_INFO("Moving")

			string rest = request.substring
				(pos + string(franka_proxy_messages::command_strings[type]).size() + 1);
			list<string> joint_values;
			rest.split(' ', joint_values);
			robot_config_7dof joint_config
				{{static_cast<double>(joint_values[0].to_float()),
				  static_cast<double>(joint_values[1].to_float()),
				  static_cast<double>(joint_values[2].to_float()),
				  static_cast<double>(joint_values[3].to_float()),
				  static_cast<double>(joint_values[4].to_float()),
				  static_cast<double>(joint_values[5].to_float()),
				  static_cast<double>(joint_values[6].to_float())}};

			unsigned char response =
				execute_exception_to_return_value([&](){controller_.move_to(joint_config);});

			stream_->send_nonblocking(&response, sizeof(unsigned char));
			break;
		}

		case franka_proxy_messages::open_gripper:
		{
			LOG_INFO("Opening Gripper")

			unsigned char response =
				execute_exception_to_return_value([&](){controller_.open_gripper();});

			stream_->send_nonblocking(&response, sizeof(unsigned char));
			break;
		}

		case franka_proxy_messages::close_gripper:
		{
			LOG_INFO("Closing Gripper")
			string rest = request.substring
				(pos + string(franka_proxy_messages::command_strings[type]).size() + 1);
			list<string> parameters;
			rest.split(' ', parameters);

			unsigned char response =
				execute_exception_to_return_value([&](){controller_.close_gripper(parameters[0].to_float(), parameters[1].to_float());});

			stream_->send_nonblocking(&response, sizeof(unsigned char));
			break;
		}

		case franka_proxy_messages::speed:
		{
			LOG_INFO("Setting speed")
			string rest = request.substring
				(pos + string(franka_proxy_messages::command_strings[type]).size());
			controller_.set_speed_factor(rest.to_float());
			break;
		}

		case franka_proxy_messages::error_recovery:
		{
			LOG_INFO("Error recovery")
			string rest = request.substring
				(pos + string(franka_proxy_messages::command_strings[type]).size());
			controller_.automatic_error_recovery();
			break;
		}

		default: ;
	}
}




//////////////////////////////////////////////////////////////////////////
//
// franka_state_server
//
//////////////////////////////////////////////////////////////////////////


franka_state_server::franka_state_server
	(network_context& network,
	 uint16 state_port,
	 franka_hardware_controller& controller)
	:
	controller_(controller),

	state_port_(state_port),
	server_(network.create_server(state_port_))
{ }


franka_state_server::~franka_state_server() NOTHROW
{	
	// Enforce explicit destructor instantiation.
}


void franka_state_server::task_main()
{
	while (!join_now())
	{
		// Possibly accept new connection
		// and drop preceding connection.
		auto_pointer<network_connection> connection
			(server_->try_accept_connection());

		if (connection)
			connection_ = std::move(connection);

		if (!connection_)
		{
			thread_util::sleep_seconds
				(sleep_seconds_disconnected_);

			continue;
		}

		try
		{
			// Copy status
			franka::RobotState robot_state = controller_.robot_state();
			franka::GripperState gripper_state = controller_.gripper_state();

			// Send state.
			// conf:j1,j2,j3,j4,j5,j6,j7$<gripper-position>$<gripper-max-position>
			string msg("conf:");

			msg += (std::to_string(robot_state.q[0]) + ',').data();
			msg += (std::to_string(robot_state.q[1]) + ',').data();
			msg += (std::to_string(robot_state.q[2]) + ',').data();
			msg += (std::to_string(robot_state.q[3]) + ',').data();
			msg += (std::to_string(robot_state.q[4]) + ',').data();
			msg += (std::to_string(robot_state.q[5]) + ',').data();
			msg += (std::to_string(robot_state.q[6]) + ',').data();

			msg += '$';

			msg += std::to_string(gripper_state.width).data();

			msg += '$';

			msg += std::to_string(gripper_state.max_width).data();

			msg += '\n';

			send_status_message(msg);
		}
		catch (...)
		{
			LOG_ERROR("Error while sending status, dropping stream and stopping robot.");
			controller_.stop_movement();
			connection_.reset();
		}

		thread_util::sleep_seconds
			(sleep_seconds_connected_);
	}
}


void franka_state_server::send_status_message(const string& command)
{
	network_buffer network_data
		(reinterpret_cast<const unsigned char*>(command.data()), command.size());

	network_buffer_progress progress(network_data);
	while (!progress.finished() && !join_now())
		network_transfer::send_partial_nonblocking
			(connection_.object(), network_data, progress);
}




} /* namespace franka_proxy */