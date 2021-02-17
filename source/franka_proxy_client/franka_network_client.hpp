/**
 *************************************************************************
 *
 * @file franka_network_client.hpp
 *
 * Network communication with a Franka Emika Panda robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP)
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP


#include <list>
#include <memory>
#include <string>
#include <vector>

#include <franka_proxy_share/asio_forward.hpp>
#include <franka_proxy_share/franka_proxy_commands.hpp>

#include <nlohmann/json.hpp>

namespace franka_proxy
{


/**
 *************************************************************************
 *
 * @class franka_state_client
 *
 * Fetch the current robot state from a Franka Emika Panda robot.
 *
 ************************************************************************/
class franka_state_client
{
public:

	franka_state_client
		(std::string remote_ip,
		 std::uint16_t remote_port);

	~franka_state_client() noexcept;


	void update_messages();

	const std::list<command_get_config_response>& states() const noexcept;
	void clear_states() noexcept;


private:

	void update_messages_buffer();

	std::unique_ptr<asio_tcp_socket> connect
		(const std::string& ip, std::uint16_t port);


	static const std::size_t receive_buffer_size_ = 1024;

	std::unique_ptr<asio::io_context> io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio_tcp_socket> connection_;

	std::string messages_buffer_;
	std::list<std::string> messages_;
	std::list<command_get_config_response> states_;
};




/**
 *************************************************************************
 *
 * @class franka_control_client
 *
 * Sends commands to a Franka Emika Panda robot.
 *
 ************************************************************************/
class franka_control_client
{
public:

	franka_control_client
		(const std::string& remote_ip,
		 std::uint16_t remote_port);

	~franka_control_client() noexcept;

	/**
	 * Sends a command to the server and awaits its reply.
	 *
	 * Commands are simple record types that are convertible to json using a custom 'to_json' function.
	 * Each command must specify a response type as `response_type` alias declaration. This type must be
	 * default constructable and json convertible with a 'from_json' function.
	 *
	 * Throws network_exception if the transmission fails.
	 * Throws command_exception if the response could not be parsed.
	 */
	template<typename TCommandType>
	typename TCommandType::response_type send_command(const TCommandType& command, float timeout_seconds = 1.f)
	{
		return send_json(command, timeout_seconds).get<typename TCommandType::response_type>();
	}

private:

	nlohmann::json send_json(
		const nlohmann::json& json,
		float timeout_seconds = 1.f
	);
	
	std::unique_ptr<asio_tcp_socket> connect
		(const std::string& ip, std::uint16_t port);
	

	std::unique_ptr<asio::io_context> io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio_tcp_socket> connection_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP) */
