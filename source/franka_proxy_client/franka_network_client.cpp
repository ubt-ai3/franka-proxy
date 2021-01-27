/**
 *************************************************************************
 *
 * @file franka_network_client.cpp
 *
 * Network communication with a Franka Emika Panda robot, implementation.
 *
 ************************************************************************/


#include "franka_network_client.hpp"

#include <array>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <iostream>

#include <asio/connect.hpp>
#include <asio/read.hpp>
#include <asio/write.hpp>
#include <asio/ip/tcp.hpp>


#include "exception.hpp"


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_state_client
//
//////////////////////////////////////////////////////////////////////////


franka_state_client::franka_state_client
	(std::string remote_ip,
	 std::uint16_t remote_port)
	:
	io_context_(new asio::io_context),

	remote_ip_(std::move(remote_ip)),
	remote_port_(remote_port),
	connection_(connect(remote_ip_, remote_port_))
{}


franka_state_client::~franka_state_client() noexcept
{
	// Enforce explicit destructor instantiation.
}


void franka_state_client::update_messages()
{
	// Restore connection after any network error.
	if (!connection_)
		connection_ = connect(remote_ip_, remote_port_);


	// Append partial messages to buffer
	// by reading from network connection.
	// Drop connection (and still evaluate)
	// on any network exception.
	try
	{
		update_messages_buffer();
	}
	catch (const asio::system_error&)
	{
		connection_.reset();
	}


	// Extract any finished messages from message buffer
	// and store all those messages in message list.
	messages_.clear();

	while (true)
	{
		std::string message(fetch_message());
		if (message.empty()) break;

		messages_.push_back(message);
	}
}


std::list<std::string> franka_state_client::messages() const
{
	return messages_;
}


void franka_state_client::update_messages_buffer()
{
	/**
	 * Fetch pending bytes from network
	 * and accumulate in message buffer.
	 */

	std::string receive_buffer(receive_buffer_size_, '\0');

	std::size_t bytes_received =
		connection_->read_some(asio::buffer(receive_buffer));

	messages_buffer_ +=
		std::string(receive_buffer, 0, bytes_received);
}


std::string franka_state_client::fetch_message()
{
	/**
	 * Return a single \n-delimited state message
	 * as read by a preceding update_messages_buffer()
	 * and remove the message from the buffer string.
	 * Multiple subsequent calls may return multiple
	 * state messages (up to an empty() string).
	 */

	messages_buffer_.erase
		(std::remove(messages_buffer_.begin(), messages_buffer_.end(), '\r'),
		 messages_buffer_.end());

	std::size_t first_newline =
		messages_buffer_.find('\n');

	if (first_newline == std::string::npos)
		return std::string();

	std::string result
		(messages_buffer_.substr(0, first_newline));

	messages_buffer_ =
		messages_buffer_.substr(first_newline + 1);

	return result;
}


std::unique_ptr<asio::ip::tcp::socket> franka_state_client::connect
	(const std::string& ip, std::uint16_t port)
{
	asio::ip::tcp::resolver resolver(*io_context_);
	asio::ip::tcp::resolver::results_type endpoints =
		resolver.resolve(ip, std::to_string(port));

	auto s = std::make_unique<asio::ip::tcp::socket>(*io_context_);
	asio::connect(*s, endpoints);

	return s;
}


//////////////////////////////////////////////////////////////////////////
//
// franka_control_client
//
//////////////////////////////////////////////////////////////////////////


franka_control_client::franka_control_client
	(const std::string& remote_ip,
	 std::uint16_t remote_port)
	:
	io_context_(new asio::io_context),

	remote_ip_(remote_ip),
	remote_port_(remote_port),
	connection_(connect(remote_ip_, remote_port_))
{}


franka_control_client::~franka_control_client() noexcept
{
	// Enforce explicit destructor instantiation.
}


void franka_control_client::send_command
	(const std::string& command, float timeout_seconds)
{
	try
	{
		if (!connection_)
			connection_ = connect(remote_ip_, remote_port_);

		asio::write(*connection_, asio::buffer(command));
	}
	catch (const asio::system_error&)
	{
		connection_.reset();
		std::cerr << "franka_control_client::send_command(): Failed to send.";
		throw network_exception();
	}
}

message_result franka_control_client::send_command(
    const nlohmann::json& json,
    float timeout_seconds
) {
    try 
    {
        if(!connection_)
            connection_ = connect(remote_ip_, remote_port_);     

        std::string message = json.dump();
        const std::uint64_t content_length = message.size();
        asio::write(
            *connection_, 
            asio::buffer(
                &content_length, 
                sizeof(std::uint64_t)
            )
        );
    
        asio::write(
            *connection_,
            asio::buffer(message)
		);

		message_result result;
		asio::read(*connection_, asio::buffer(&result, sizeof(message_result)));
    
		return result;
    } catch(const asio::system_error&)
    {
        connection_.reset();
        std::cerr << "franka_control_client::send_command(): Failed to send.";
        throw network_exception();
    }
}

unsigned char franka_control_client::send_command_and_check_response
	(const std::string& command, float timeout_seconds)
{
	try
	{
		if (!connection_)
			connection_ = connect(remote_ip_, remote_port_);

		asio::write(*connection_, asio::buffer(command));

		unsigned char return_value[1];
		asio::read(*connection_, asio::buffer(return_value));

		return return_value[0];
	}
	catch (const asio::system_error&)
	{
		connection_.reset();
		std::cerr << "franka_control_client::send_command_and_check_response(): Failed to send.";
		throw network_exception();
	}
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_control_client::send_stop_recording_and_receive_sequence(float timeout_seconds)
{
	//const std::string command =
	//(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::stop_recording]) +
	//	franka_proxy_messages::command_end_marker).data();

	//free_timer t;
	//while (t.seconds_passed() < timeout_seconds)
	//{
	//	try
	//	{
	//		if (!stream_)
	//			stream_.reset
	//				(new network_stream
	//				 (network_.create_connection
	//				  (remote_ip_, remote_port_),
	//				  16384, 1037741824, 16384, 1037741824));

	//		// command
	//		stream_->send_nonblocking
	//			(reinterpret_cast<const unsigned char*>(command.data()),
	//			 command.size());


	//		// count
	//		int64 count;
	//		while (!stream_->try_receive_nonblocking
	//			(reinterpret_cast<unsigned char*>(&count), sizeof(int64), false))
	//			thread_util::sleep_slice();
	//		// todo ntoh byteorder


	//		std::vector<std::array<double, 7>> q_sequence;
	//		q_sequence.reserve(count);
	//		for (int64 i = 0; i < count; ++i)
	//		{
	//			// size
	//			int64 size;
	//			while (!stream_->try_receive_nonblocking
	//				(reinterpret_cast<unsigned char*>(&size), sizeof(int64), false))
	//				thread_util::sleep_slice();
	//			// todo ntoh byteorder


	//			// data
	//			network_buffer network_data = network_buffer();
	//			network_data.resize(size);
	//			while (!stream_->try_receive_nonblocking
	//				(network_data.data(), size, false))
	//				thread_util::sleep_slice();


	//			// extract data
	//			string tmp(reinterpret_cast<const char*>(network_data.data()), network_data.size());
	//			list<string> joint_values_list;
	//			tmp.split(',', joint_values_list);

	//			if (joint_values_list.size() != 7)
	//			{
	//				LOG_WARN("Joint values message does not have 7 joint values");
	//				continue;
	//			}

	//			std::array<double, 7> joints
	//			{
	//				{
	//					strtod(joint_values_list[0].data(), nullptr),
	//					strtod(joint_values_list[1].data(), nullptr),
	//					strtod(joint_values_list[2].data(), nullptr),
	//					strtod(joint_values_list[3].data(), nullptr),
	//					strtod(joint_values_list[4].data(), nullptr),
	//					strtod(joint_values_list[5].data(), nullptr),
	//					strtod(joint_values_list[6].data(), nullptr)
	//				}
	//			};


	//			// emplace position
	//			q_sequence.emplace_back(joints);
	//		}

	//		// count
	//		while (!stream_->try_receive_nonblocking
	//			(reinterpret_cast<unsigned char*>(&count), sizeof(int64), false))
	//			thread_util::sleep_slice();
	//		// todo ntoh byteorder


	//		std::vector<std::array<double, 6>> f_sequence;
	//		f_sequence.reserve(count);
	//		for (int64 i = 0; i < count; ++i)
	//		{
	//			// size
	//			int64 size;
	//			while (!stream_->try_receive_nonblocking
	//				(reinterpret_cast<unsigned char*>(&size), sizeof(int64), false))
	//				thread_util::sleep_slice();
	//			// todo ntoh byteorder


	//			// data
	//			network_buffer network_data = network_buffer();
	//			network_data.resize(size);
	//			while (!stream_->try_receive_nonblocking
	//				(network_data.data(), size, false))
	//				thread_util::sleep_slice();


	//			// extract data
	//			string tmp(reinterpret_cast<const char*>(network_data.data()), network_data.size());
	//			list<string> joint_values_list;
	//			tmp.split(',', joint_values_list);

	//			if (joint_values_list.size() != 6)
	//			{
	//				LOG_WARN("Joint values message does not have 7 joint values");
	//				continue;
	//			}

	//			std::array<double, 6> forces
	//			{
	//				{
	//					strtod(joint_values_list[0].data(), nullptr),
	//					strtod(joint_values_list[1].data(), nullptr),
	//					strtod(joint_values_list[2].data(), nullptr),
	//					strtod(joint_values_list[3].data(), nullptr),
	//					strtod(joint_values_list[4].data(), nullptr),
	//					strtod(joint_values_list[5].data(), nullptr)
	//				}
	//			};


	//			// emplace position
	//			f_sequence.emplace_back(forces);
	//		}

	//		// response	
	//		unsigned char return_value;
	//		while (!stream_->try_receive_nonblocking
	//			(&return_value, sizeof(unsigned char), false))
	//			thread_util::sleep_slice();

	//		LOG_INFO("received: " + count);


	//		return {q_sequence, f_sequence};
	//	}
	//	catch (const network_exception&)
	//	{
	//		stream_.reset();
	//	}
	//}

	//LOG_ERROR("Failed to send.");
	//throw network_exception();

	return {};
}


void franka_control_client::send_move_sequence
	(const std::vector<std::array<double, 7>>& q_sequence,
	 const std::vector<std::array<double, 6>>& f_sequence,
	 const std::vector<std::array<double, 6>>& selection_vector_sequence,
	 float timeout_seconds)
{
	//string command =
	//(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move_hybrid_sequence]) +
	//	franka_proxy_messages::command_end_marker).data();

	//free_timer t;
	//while (t.seconds_passed() < 10000)
	//{
	//	try
	//	{
	//		if (!stream_)
	//			stream_.reset
	//				(new network_stream
	//				 (network_.create_connection
	//				  (remote_ip_, remote_port_),
	//				  16384, 1037741824, 16384, 1037741824));

	//		// command
	//		stream_->send_nonblocking
	//			(reinterpret_cast<const unsigned char*>(command.data()),
	//			 command.size());

	//		// sequence size
	//		// todo hton
	//		int64 count = q_sequence.size();
	//		stream_->send_nonblocking
	//			(reinterpret_cast<const unsigned char*>(&count), sizeof(int64));

	//		// data
	//		for (const auto& p : q_sequence)
	//		{
	//			string message;
	//			message += (std::to_string(p[0]) + ",").data();
	//			message += (std::to_string(p[1]) + ",").data();
	//			message += (std::to_string(p[2]) + ",").data();
	//			message += (std::to_string(p[3]) + ",").data();
	//			message += (std::to_string(p[4]) + ",").data();
	//			message += (std::to_string(p[5]) + ",").data();
	//			message += (std::to_string(p[6])).data();
	//			message += '\n';

	//			// send size and message
	//			// todo hton byteorder
	//			int64 size = message.size();
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(message.data()), message.size());

	//			if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
	//			{
	//				LOG_WARN("Network send buffer is 80 percent used.")
	//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//			}
	//		}

	//		// data
	//		for (const auto& p : f_sequence)
	//		{
	//			string message;
	//			message += (std::to_string(p[0]) + ",").data();
	//			message += (std::to_string(p[1]) + ",").data();
	//			message += (std::to_string(p[2]) + ",").data();
	//			message += (std::to_string(p[3]) + ",").data();
	//			message += (std::to_string(p[4]) + ",").data();
	//			message += (std::to_string(p[5])).data();
	//			message += '\n';

	//			// send size and message
	//			// todo hton byteorder
	//			int64 size = message.size();
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(message.data()), message.size());

	//			if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
	//			{
	//				LOG_WARN("Network send buffer is 80 percent used.")
	//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//			}
	//		}

	//		// data
	//		for (const auto& p : selection_vector_sequence)
	//		{
	//			string message;
	//			message += (std::to_string(p[0]) + ",").data();
	//			message += (std::to_string(p[1]) + ",").data();
	//			message += (std::to_string(p[2]) + ",").data();
	//			message += (std::to_string(p[3]) + ",").data();
	//			message += (std::to_string(p[4]) + ",").data();
	//			message += (std::to_string(p[5])).data();
	//			message += '\n';

	//			// send size and message
	//			// todo hton byteorder
	//			int64 size = message.size();
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(&size), sizeof(int64));
	//			stream_->send_nonblocking
	//				(reinterpret_cast<const unsigned char*>(message.data()), message.size());

	//			if (stream_->pending_send_bytes() > (stream_->buffer_max_size_send * 0.8))
	//			{
	//				LOG_WARN("Network send buffer is 80 percent used.")
	//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//			}
	//		}

	//		// response	
	//		unsigned char return_value;
	//		while (!stream_->try_receive_nonblocking
	//			(&return_value, sizeof(unsigned char), false))
	//			thread_util::sleep_slice();

	//		return;
	//	}
	//	catch (const network_exception&)
	//	{
	//		stream_.reset();
	//	}
	//}

	//LOG_ERROR("Failed to send.");
	//throw network_exception();
}


std::unique_ptr<asio::ip::tcp::socket> franka_control_client::connect
	(const std::string& ip, std::uint16_t port)
{
	asio::ip::tcp::resolver resolver(*io_context_);
	asio::ip::tcp::resolver::results_type endpoints =
		resolver.resolve(ip, std::to_string(port));

	auto s = std::make_unique<asio::ip::tcp::socket>(*io_context_);
	asio::connect(*s, endpoints);

	return s;
}




} /* namespace franka_proxy */
