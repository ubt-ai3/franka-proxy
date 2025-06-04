#ifndef INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP
/**
 *************************************************************************
 *
 * @file franka_network_client.hpp
 *
 * Network communication with a Franka Emika Panda robot.
 *
 ************************************************************************/

#include <list>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include <franka_proxy_share/asio_forward.hpp>
#include <franka_proxy_share/franka_proxy_commands.hpp>


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
	franka_state_client(
		std::string remote_ip,
		std::uint16_t remote_port);

	~franka_state_client() noexcept;


	void update_messages();

	const std::list<command_get_config_response>& states() const noexcept;
	void clear_states() noexcept;

private:
	void update_messages_buffer();

	std::unique_ptr<asio_tcp_socket> connect(
		const std::string& ip, std::uint16_t port);


	static constexpr std::size_t receive_buffer_size_ = 1024;

	std::unique_ptr<asio::io_context> io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio_tcp_socket> connection_;

	std::string messages_buffer_;
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
	franka_control_client(
		std::string remote_ip,
		std::uint16_t remote_port);

	~franka_control_client() noexcept;

	/**
	 * Sends a command to the server and awaits its response.
	 *
	 * Commands are simple record types that are JSON convertible by providing a simple "to_json" function.
	 * See "franka_proxy_commands.hpp" on details.
	 *
	 * Each command has a response type associated by declaring a "response_type" alias, which is then used
	 * to decode the response to.
	 *
	 * Throws network_exception if the command transmission failed.
	 * Throws bad_response_exception if the response failed to parse.
	 */
	template <typename TCommandType> typename TCommandType::response_type send_command(
		const TCommandType& command)
	{
		return send_json(command).template get<typename TCommandType::response_type>();
	}

private:
	nlohmann::json send_json(const nlohmann::json& json);

	std::unique_ptr<asio_tcp_socket> connect(
		const std::string& ip,
		std::uint16_t port);

	std::unique_ptr<asio::io_context> io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio_tcp_socket> connection_;
};
}

#endif // INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP
