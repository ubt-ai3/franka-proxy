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
#include <viral_core/timer.hpp>
#include "viral_core/log.hpp"
#include <vector>
#include <array>
#include "viral_core/network_stream.hpp"
#include "../franka_proxy_share/franka_proxy_messages.hpp"
#include <string>
#include <thread>


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

	stream_(new network_stream
	 (network_.create_connection
	  (remote_ip_, remote_port_),
	  16384, 1000000000, 16384, 1000000000))
{
	start();
}


franka_control_client::~franka_control_client() noexcept
{
	try { join(); }
	catch (...)
	{
		LOG_CRITICAL("Internal thread threw an exception on joining.");
	}
}


void franka_control_client::task_main()
{
	while (!joining_now())
	{
		if (!stream_)
		{
			thread_util::sleep_slice();
			continue;
		}

		try
		{
			// We have an active connection,
			// so handle incoming and outgoing network.
			stream_->update();
		}
		catch (...)
		{
			stream_.reset();
		}

		thread_util::sleep_seconds(0.002f);
	}
}


void franka_control_client::send_command
	(const string& command, float timeout_seconds)
{
	free_timer t;
	while (t.seconds_passed() < timeout_seconds)
	{
		try
		{
			if (!stream_)
				stream_.reset(new network_stream
				 (network_.create_connection
				  (remote_ip_, remote_port_),
				  16384, 1000000000, 16384, 1000000000));
			
			stream_->send_nonblocking
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());

			return;
		}
		catch (const network_exception&)
		{
			stream_.reset();
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
			if (!stream_)
				stream_.reset(new network_stream
				 (network_.create_connection
				  (remote_ip_, remote_port_),
				  16384, 1000000000, 16384, 1000000000));
			
			stream_->send_nonblocking
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());

			unsigned char return_value;
			while (!stream_->try_receive_nonblocking(&return_value, sizeof(unsigned char), false))
				thread_util::sleep_slice();

			return return_value;
		}
		catch (const network_exception&)
		{
			stream_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_control_client::send_stop_recording_and_receive_sequence(float timeout_seconds)
{
	const string command =
	(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::stop_recording]) +
		franka_proxy_messages::command_end_marker).data();

	free_timer t;
	while (t.seconds_passed() < timeout_seconds)
	{
		try
		{
			if (!stream_)
				stream_.reset
					(new network_stream
					 (network_.create_connection
					  (remote_ip_, remote_port_),
					  16384, 1037741824, 16384, 1037741824));

			// command
			stream_->send_nonblocking
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());


			// count
			int64 count;
			while (!stream_->try_receive_nonblocking
				(reinterpret_cast<unsigned char*>(&count), sizeof(int64), false))
				thread_util::sleep_slice();
			// todo ntoh byteorder


			std::vector<std::array<double, 7>> q_sequence;
			q_sequence.reserve(count);
			for (int64 i = 0; i < count; ++i)
			{
				// size
				int64 size;
				while (!stream_->try_receive_nonblocking
					(reinterpret_cast<unsigned char*>(&size), sizeof(int64), false))
					thread_util::sleep_slice();
				// todo ntoh byteorder


				// data
				network_buffer network_data = network_buffer();
				network_data.resize(size);
				while (!stream_->try_receive_nonblocking
					(network_data.data(), size, false))
					thread_util::sleep_slice();


				// extract data
				string tmp(reinterpret_cast<const char*>(network_data.data()), network_data.size());
				list<string> joint_values_list;
				tmp.split(',', joint_values_list);

				if (joint_values_list.size() != 7)
				{
					LOG_WARN("Joint values message does not have 7 joint values");
					continue;
				}

				std::array<double, 7> joints
				{
					{
						strtod(joint_values_list[0].data(), nullptr),
						strtod(joint_values_list[1].data(), nullptr),
						strtod(joint_values_list[2].data(), nullptr),
						strtod(joint_values_list[3].data(), nullptr),
						strtod(joint_values_list[4].data(), nullptr),
						strtod(joint_values_list[5].data(), nullptr),
						strtod(joint_values_list[6].data(), nullptr)
					}
				};


				// emplace position
				q_sequence.emplace_back(joints);
			}

			// count
			while (!stream_->try_receive_nonblocking
				(reinterpret_cast<unsigned char*>(&count), sizeof(int64), false))
				thread_util::sleep_slice();
			// todo ntoh byteorder


			std::vector<std::array<double, 6>> f_sequence;
			f_sequence.reserve(count);
			for (int64 i = 0; i < count; ++i)
			{
				// size
				int64 size;
				while (!stream_->try_receive_nonblocking
					(reinterpret_cast<unsigned char*>(&size), sizeof(int64), false))
					thread_util::sleep_slice();
				// todo ntoh byteorder


				// data
				network_buffer network_data = network_buffer();
				network_data.resize(size);
				while (!stream_->try_receive_nonblocking
					(network_data.data(), size, false))
					thread_util::sleep_slice();


				// extract data
				string tmp(reinterpret_cast<const char*>(network_data.data()), network_data.size());
				list<string> joint_values_list;
				tmp.split(',', joint_values_list);

				if (joint_values_list.size() != 6)
				{
					LOG_WARN("Joint values message does not have 7 joint values");
					continue;
				}

				std::array<double, 6> forces
				{
					{
						strtod(joint_values_list[0].data(), nullptr),
						strtod(joint_values_list[1].data(), nullptr),
						strtod(joint_values_list[2].data(), nullptr),
						strtod(joint_values_list[3].data(), nullptr),
						strtod(joint_values_list[4].data(), nullptr),
						strtod(joint_values_list[5].data(), nullptr)
					}
				};


				// emplace position
				f_sequence.emplace_back(forces);
			}

			// response	
			unsigned char return_value;
			while (!stream_->try_receive_nonblocking
				(&return_value, sizeof(unsigned char), false))
				thread_util::sleep_slice();

			LOG_INFO("received: " + count);


			return {q_sequence, f_sequence};
		}
		catch (const network_exception&)
		{
			stream_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}


void franka_control_client::send_move_sequence
	(const std::vector<std::array<double, 7>>& q_sequence,
	 const std::vector<std::array<double, 6>>& f_sequence,
	 const std::vector<std::array<double, 6>>& selection_vector_sequence,
	 float timeout_seconds)
{
	string command =
	(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move_hybrid_sequence]) +
		franka_proxy_messages::command_end_marker).data();

	free_timer t;
	while (t.seconds_passed() < 10000)
	{
		try
		{
			if (!stream_)
				stream_.reset
					(new network_stream
					 (network_.create_connection
					  (remote_ip_, remote_port_),
					  16384, 1037741824, 16384, 1037741824));

			// command
			stream_->send_nonblocking
				(reinterpret_cast<const unsigned char*>(command.data()),
				 command.size());

			// sequence size
			// todo hton
			int64 count = q_sequence.size();
			stream_->send_nonblocking
				(reinterpret_cast<const unsigned char*>(&count), sizeof(int64));

			// data
			for (const auto& p : q_sequence)
			{
				string message;
				message += (std::to_string(p[0]) + ",").data();
				message += (std::to_string(p[1]) + ",").data();
				message += (std::to_string(p[2]) + ",").data();
				message += (std::to_string(p[3]) + ",").data();
				message += (std::to_string(p[4]) + ",").data();
				message += (std::to_string(p[5]) + ",").data();
				message += (std::to_string(p[6])).data();
				message += '\n';

				// send size and message
				// todo hton byteorder
				int64 size = message.size();
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(message.data()), message.size());

				if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
				{
					LOG_WARN("Network send buffer is 80 percent used.")
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}

			// data
			for (const auto& p : f_sequence)
			{
				string message;
				message += (std::to_string(p[0]) + ",").data();
				message += (std::to_string(p[1]) + ",").data();
				message += (std::to_string(p[2]) + ",").data();
				message += (std::to_string(p[3]) + ",").data();
				message += (std::to_string(p[4]) + ",").data();
				message += (std::to_string(p[5])).data();
				message += '\n';

				// send size and message
				// todo hton byteorder
				int64 size = message.size();
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(message.data()), message.size());

				if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
				{
					LOG_WARN("Network send buffer is 80 percent used.")
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}

			// data
			for (const auto& p : selection_vector_sequence)
			{
				string message;
				message += (std::to_string(p[0]) + ",").data();
				message += (std::to_string(p[1]) + ",").data();
				message += (std::to_string(p[2]) + ",").data();
				message += (std::to_string(p[3]) + ",").data();
				message += (std::to_string(p[4]) + ",").data();
				message += (std::to_string(p[5])).data();
				message += '\n';

				// send size and message
				// todo hton byteorder
				int64 size = message.size();
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
				stream_->send_nonblocking
					(reinterpret_cast<const unsigned char*>(message.data()), message.size());

				if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
				{
					LOG_WARN("Network send buffer is 80 percent used.")
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}

			// response	
			unsigned char return_value;
			while (!stream_->try_receive_nonblocking
				(&return_value, sizeof(unsigned char), false))
				thread_util::sleep_slice();

			return;
		}
		catch (const network_exception&)
		{
			stream_.reset();
		}
	}

	LOG_ERROR("Failed to send.");
	throw network_exception();
}


} /* namespace franka_proxy */