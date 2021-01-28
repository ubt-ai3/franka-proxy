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

#include <nlohmann/json.hpp>

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

nlohmann::json franka_control_client::send_command(
    const nlohmann::json& json,
    float timeout_seconds
) {
    try 
    {
        if(!connection_)
            connection_ = connect(remote_ip_, remote_port_);     

        std::string message = json.dump();
        std::uint64_t content_length = message.size();
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

		asio::read(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));

		std::string response{};
		response.resize(content_length);
		asio::read(*connection_, asio::buffer(response));

		return nlohmann::json::parse(response);
    } 
	catch(const asio::system_error&)
    {
        connection_.reset();
        std::cerr << "franka_control_client::send_command(): Failed to send.";
        throw network_exception();
    }
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
