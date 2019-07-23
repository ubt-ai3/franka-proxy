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


namespace franka_proxy
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_state_client
//
//////////////////////////////////////////////////////////////////////////


//franka_state_client::franka_state_client
//	(network_context& network,
//	 const string& remote_ip, 
//	 uint16 remote_port)
//	:
//	network_(network),
//	remote_ip_(remote_ip),
//	remote_port_(remote_port),
//	connection_
//		(network_.create_connection
//			(remote_ip_, remote_port_))
//{ }
//
//
//franka_state_client::~franka_state_client() NOTHROW
//{	
//	// Enforce explicit destructor instantiation.
//}
//
//
//void franka_state_client::update_messages()
//{
//	// Restore connection after any network error.
//	if (!connection_)
//		connection_ =
//			network_.create_connection
//				(remote_ip_, remote_port_);
//
//
//	// Append partial messages to buffer
//	// by reading from network connection.
//	// Drop connection (and still evaluate)
//	// on any network exception.
//	try
//	{
//		update_messages_buffer();
//	}
//	catch (const network_exception&)
//		{ connection_.reset(); }
//
//
//	// Extract any finished messages from message buffer
//	// and store all those messages in message list.
//	messages_.clear();
//
//	while (true)
//	{
//		string message(fetch_message());
//		if (message.empty()) break;
//
//		messages_.push_back(message);
//	}
//}
//
//
//list<string> franka_state_client::messages() const 
//	{ return messages_; }
//
//
//void franka_state_client::update_messages_buffer()
//{
//	/**
//	 * Fetch pending bytes from network
//	 * and accumulate in message buffer.
//	 */
//
//	unsigned char receive_buffer[receive_buffer_size_];
//
//	int64 bytes_received =
//		connection_.object().receive_nonblocking
//			(receive_buffer, receive_buffer_size_);
//
//	messages_buffer_ +=
//		string(reinterpret_cast<char*>(receive_buffer),
//			   bytes_received);
//}
//
//
//string franka_state_client::fetch_message()
//{
//	/**
//	 * Return a single \n-delimited state message
//	 * as read by a preceding update_messages_buffer()
//	 * and remove the message from the buffer string.
//	 * Multiple subsequent calls may return multiple
//	 * state messages (up to an empty() string).
//	 */
//
//	messages_buffer_.remove_all('\r');
//
//	int64 first_newline =
//		messages_buffer_.seek('\n');
//
//	if (first_newline == string::invalid_index)
//		return string();
//
//	string result
//		(messages_buffer_.substring(0, first_newline));
//
//	messages_buffer_ =
//		messages_buffer_.substring(first_newline + 1);
//
//	return result;
//}




//////////////////////////////////////////////////////////////////////////
//
// franka_state_server
//
//////////////////////////////////////////////////////////////////////////


franka_state_server::franka_state_server
	(network_context& network,
	 uint16 state_port,
	 std::mutex& state_lock,
	 franka::RobotState& robot_state,
	 franka::GripperState& gripper_state)
	:
	network_(network),
	state_port_(state_port),

	state_lock_(state_lock),
	robot_state_(robot_state),
	gripper_state_(gripper_state),

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


		try
		{
			// Copy status
			franka::RobotState robot_state;
			franka::GripperState gripper_state;
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				robot_state = robot_state_;
				gripper_state = gripper_state_;
			}

			// Send state.
			// conf:j1,j2,j3,j4,j5,j6,j7$<gripper-open>$<gripper-position>$<gripper-max-position>$<error-code>
			string msg("conf:");

			msg += (std::to_string(robot_state.q[0]) + ',').data();
			msg += (std::to_string(robot_state.q[1]) + ',').data();
			msg += (std::to_string(robot_state.q[2]) + ',').data();
			msg += (std::to_string(robot_state.q[3]) + ',').data();
			msg += (std::to_string(robot_state.q[4]) + ',').data();
			msg += (std::to_string(robot_state.q[5]) + ',').data();
			msg += (std::to_string(robot_state.q[6]) + ',').data();

			msg += '$';

			// TODO!

			msg += '$';

			msg += std::to_string(gripper_state.width).data();

			msg += '$';

			msg += std::to_string(gripper_state.max_width).data();

			msg += '$';

			msg += std::to_string(current_error_).data();

			msg += '\n';
		}
		catch (...)
		{
			LOG_ERROR("Error while sending status, dropping stream.");
			connection_.reset();
		}

		thread_util::sleep_seconds
			(sleep_seconds_disconnected_);
	}
}


void franka_state_server::send_status_message(const viral_core::string& command)
{
	network_buffer network_data
		(reinterpret_cast<const unsigned char*>(command.data()),
		 command.size());

	network_buffer_progress progress(network_data);
	while (!progress.finished())
		network_transfer::send_partial_nonblocking
			(connection_.object(), network_data, progress);
}




} /* namespace franka_proxy */