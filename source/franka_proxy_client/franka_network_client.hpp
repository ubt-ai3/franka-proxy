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

#pragma warning(disable : 4242)
#include <asio/ip/tcp.hpp>
#pragma warning(pop)


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
	std::list<std::string> messages() const;


private:

	void update_messages_buffer();
	std::string fetch_message();

	std::unique_ptr<asio::ip::tcp::socket> connect
		(const std::string& ip, std::uint16_t port);


	static const std::size_t receive_buffer_size_ = 1024;

	asio::io_context io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio::ip::tcp::socket> connection_;

	std::string messages_buffer_;
	std::list<std::string> messages_;
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


	void send_command
		(const std::string& command,
		 float timeout_seconds = 1.f);
	unsigned char send_command_and_check_response
		(const std::string& command,
		 float timeout_seconds = 1.f);


	// todo split up function
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
		send_stop_recording_and_receive_sequence
			(float timeout_seconds = 1.f);
	void send_move_sequence
		(const std::vector<std::array<double, 7>>& q_sequence,
		 const std::vector<std::array<double, 6>>& f_sequence,
		 const std::vector<std::array<double, 6>>& selection_vector_sequence,
		 float timeout_seconds = 1.f);

private:

	std::unique_ptr<asio::ip::tcp::socket> connect
		(const std::string& ip, std::uint16_t port);
	

	asio::io_context io_context_;

	const std::string remote_ip_;
	const std::uint16_t remote_port_;

	std::unique_ptr<asio::ip::tcp::socket> connection_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP) */
