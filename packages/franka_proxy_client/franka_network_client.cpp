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
#include <iostream>
#include <string>
#include <thread>
#include <utility>

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
	: io_context_(new asio::io_context),

	  remote_ip_(std::move(remote_ip)),
	  remote_port_(remote_port),
	  connection_(connect(remote_ip_, remote_port_))
{
}


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
		states_.clear();
		update_messages_buffer();
	}
	catch (const asio::system_error&)
	{
		connection_.reset();
	}
}


const std::list<command_get_config_response>& franka_state_client::states() const noexcept
{
	return states_;
}


void franka_state_client::clear_states() noexcept
{
	states_.clear();
}


void franka_state_client::update_messages_buffer()
{
	std::uint64_t content_length;
	read(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));

	std::string buffer;
	buffer.resize(content_length);

	read(*connection_, asio::buffer(buffer));

	try
	{
		const auto state = nlohmann::json::parse(buffer).get<command_get_config_response>();
		states_.push_back(state);
	}
	catch (...)
	{
		std::cerr << "franka_state_client::update_messages_buffer(): "
			<< "State message discarted due to bad JSON."
			<< std::endl;
	}
}


std::unique_ptr<asio::ip::tcp::socket> franka_state_client::connect
(const std::string& ip, std::uint16_t port)
{
	asio::ip::tcp::resolver resolver(*io_context_);
	asio::ip::tcp::resolver::results_type endpoints =
		resolver.resolve(ip, std::to_string(port));

	auto s = std::make_unique<asio::ip::tcp::socket>(*io_context_);

	try
	{
		asio::connect(*s, endpoints);
	}
	catch (const std::system_error& e)
	{
		std::cerr << "franka_state_client::connect(): Failed to connect: "
			<< e.what() << std::endl;
		throw network_exception("");
	}

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
	: io_context_(new asio::io_context),

	  remote_ip_(remote_ip),
	  remote_port_(remote_port),
	  connection_(connect(remote_ip_, remote_port_))
{
}


franka_control_client::~franka_control_client() noexcept
{
	// Enforce explicit destructor instantiation.
}


nlohmann::json franka_control_client::send_json
(const nlohmann::json& json, float timeout_seconds)
{
	try
	{
		if (!connection_)
			connection_ = connect(remote_ip_, remote_port_);

		std::string message = json.dump();
		std::uint64_t content_length = message.size();
		write
		(*connection_,
		 asio::buffer(&content_length, sizeof(std::uint64_t)));

		write(*connection_, asio::buffer(message));

		read
			(*connection_, asio::buffer(&content_length, sizeof(std::uint64_t)));

		std::string response{};
		response.resize(content_length);
		read(*connection_, asio::buffer(response));

		return nlohmann::json::parse(response);
	}
	catch (const asio::system_error&)
	{
		connection_.reset();
		std::cerr << "franka_control_client::send_json(): Failed to send.";
		throw network_exception("Failed to send command.");
	}
	catch (const nlohmann::json::exception&)
	{
		connection_.reset();
		std::cerr << "franka_control_client::send_json(): Failed to parse response.";
		throw command_exception("Failed to parse response from server.");
	}
}


std::unique_ptr<asio::ip::tcp::socket> franka_control_client::connect
(const std::string& ip, std::uint16_t port)
{
	asio::ip::tcp::resolver resolver(*io_context_);
	asio::ip::tcp::resolver::results_type endpoints =
		resolver.resolve(ip, std::to_string(port));

	auto s = std::make_unique<asio::ip::tcp::socket>(*io_context_);

	try
	{
		asio::connect(*s, endpoints);
	}
	catch (const std::system_error& e)
	{
		std::cerr << "franka_control_client::connect(): Failed to connect: "
			<< e.what() << std::endl;

		throw network_exception("");
	}

	return s;
}
} /* namespace franka_proxy */
