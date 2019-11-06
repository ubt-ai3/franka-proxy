/**
 *************************************************************************
 *
 * @file franka_network_client.cpp
 *
 * Network communication with a Franka Emika Panda robot, implementation.
 *
 ************************************************************************/


#include "franka_network_client.hpp"

#include <viral_core/buffer.hpp>
#include <viral_core/network.hpp>
#include <viral_core/network_transfer.hpp>
#include <viral_core/timer.hpp>
#include "viral_core/log.hpp"
#include "franka_proxy_share/franka_proxy_messages.hpp"


namespace franka_proxy
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_state_client
//
//////////////////////////////////////////////////////////////////////////


franka_state_client::franka_state_client
	(network_context& network,
	 const string& remote_ip,
	 uint16 remote_port)
	:
	network_(network),
	remote_ip_(remote_ip),
	remote_port_(remote_port),
	connection_
	(network_.create_connection
		(remote_ip_, remote_port_))
{}


franka_state_client::~franka_state_client() noexcept
{
	// Enforce explicit destructor instantiation.
}


void franka_state_client::update_messages()
{
	// Restore connection after any network error.
	if (!connection_)
		connection_ =
			network_.create_connection
			(remote_ip_, remote_port_);


	// Append partial messages to buffer
	// by reading from network connection.
	// Drop connection (and still evaluate)
	// on any network exception.
	try
	{
		update_messages_buffer();
	}
	catch (const network_exception&)
	{
		connection_.reset();
	}


	// Extract any finished messages from message buffer
	// and store all those messages in message list.
	messages_.clear();

	while (true)
	{
		string message(fetch_message());
		if (message.empty()) break;

		messages_.push_back(message);
	}
}


list<string> franka_state_client::messages() const
{
	return messages_;
}


void franka_state_client::update_messages_buffer()
{
	/**
	 * Fetch pending bytes from network
	 * and accumulate in message buffer.
	 */

	unsigned char receive_buffer[receive_buffer_size_];

	int64 bytes_received =
		connection_.object().receive_nonblocking
		(receive_buffer, receive_buffer_size_);

	messages_buffer_ +=
		string(reinterpret_cast<char*>(receive_buffer),
		       bytes_received);
}


string franka_state_client::fetch_message()
{
	/**
	 * Return a single \n-delimited state message
	 * as read by a preceding update_messages_buffer()
	 * and remove the message from the buffer string.
	 * Multiple subsequent calls may return multiple
	 * state messages (up to an empty() string).
	 */

	messages_buffer_.remove_all('\r');

	int64 first_newline =
		messages_buffer_.seek('\n');

	if (first_newline == string::invalid_index)
		return string();

	string result
		(messages_buffer_.substring(0, first_newline));

	messages_buffer_ =
		messages_buffer_.substring(first_newline + 1);

	return result;
}




//////////////////////////////////////////////////////////////////////////
//
// franka_control_client
//
//////////////////////////////////////////////////////////////////////////


franka_control_client::franka_control_client
	(network_context& network,
	 const string& remote_ip,
	 uint16 remote_port)
	:
	network_(network),

	remote_ip_(remote_ip),
	remote_port_(remote_port),

	connection_
		(network_.create_connection
			(remote_ip_, remote_port_))
{}


franka_control_client::~franka_control_client() noexcept
{
	// Enforce explicit destructor instantiation.
}


void franka_control_client::send_command
	(const string& command, float timeout_seconds)
{
	free_timer t;
	while (t.seconds_passed() < timeout_seconds)
	{
		try
		{
			if (!connection_)
				connection_ = network_.create_connection
					(remote_ip_, remote_port_);
			
			network_buffer network_data
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());
			network_transfer::send_blocking
				(connection_.object(), network_data, false, 0, false, 0);

			return;
		}
		catch (const network_exception&)
		{
			connection_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}


unsigned char franka_control_client::send_command_and_check_response
	(const string& command, float timeout_seconds)
{
	free_timer t;
	while (t.seconds_passed() < timeout_seconds)
	{
		try
		{
			if (!connection_)
				connection_ = network_.create_connection
					(remote_ip_, remote_port_);
			
			network_buffer network_data
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());
			network_transfer::send_blocking
				(connection_.object(), network_data, false, 0, false, 0);
			
			network_data = network_buffer();
			network_transfer::receive_blocking
				(connection_.object(), network_data,
				 sizeof(unsigned char),
				 false, 0, false, 0);

			return network_data[0];
		}
		catch (const network_exception&)
		{
			connection_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}


string franka_control_client::send_stop_recording_and_receive_squence(float timeout_seconds)
{
	const string command =
		(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::stop_recording]) +
			franka_proxy_messages::command_end_marker).data();

	free_timer t;
	while (t.seconds_passed() < timeout_seconds)
	{
		try
		{
			if (!connection_)
				connection_ = network_.create_connection
					(remote_ip_, remote_port_);
			
			network_buffer network_data
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());
			network_transfer::send_blocking
				(connection_.object(), network_data, false, 0, false, 0);
			
			// size
			network_data = network_buffer();
			network_transfer::receive_blocking
				(connection_.object(), network_data,
				 sizeof(unsigned char),
				 false, 0, false, 0);

			const unsigned char size = network_data[0];
			LOG_INFO(size);

			// data
			network_data = network_buffer();
			network_transfer::receive_blocking
				(connection_.object(), network_data,
				 size,
				 false, 0, false, 0);

			LOG_INFO(network_data.size());
			string data(reinterpret_cast<const char*>(network_data.data()));

			// response	
			network_transfer::receive_blocking
				(connection_.object(), network_data,
				 sizeof(unsigned char),
				 false, 0, false, 0);

			const unsigned char response = network_data[0];

			return data;

		}
		catch (const network_exception&)
		{
			connection_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}

} /* namespace franka_proxy */