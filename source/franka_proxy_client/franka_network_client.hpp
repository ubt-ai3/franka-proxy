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

#include <vector>

#include <viral_core/auto_pointer.hpp>
#include <viral_core/list.hpp>
#include <viral_core/network_forward.hpp>
#include <viral_core/string.hpp>
#include <viral_core/thread.hpp>


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
		(viral_core::network_context& network,
		 const viral_core::string& remote_ip,
		 uint16 remote_port);

	~franka_state_client() noexcept;


	void update_messages();
	viral_core::list<viral_core::string> messages() const;


private:

	void update_messages_buffer();
	viral_core::string fetch_message();


	static const int64 receive_buffer_size_ = 1024;

	viral_core::network_context& network_;

	const viral_core::string remote_ip_;
	const uint16 remote_port_;

	viral_core::auto_pointer<viral_core::network_connection> connection_;

	viral_core::string messages_buffer_;
	viral_core::list<viral_core::string> messages_;
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
	: viral_core::threaded_task
{
public:

	franka_control_client
		(viral_core::network_context& network,
		 const viral_core::string& remote_ip,
		 uint16 remote_port);

	~franka_control_client() noexcept;


	void task_main() override;


	void send_command
		(const viral_core::string& command,
		 float timeout_seconds = 1.f);
	unsigned char send_command_and_check_response
		(const viral_core::string& command,
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

	viral_core::network_context& network_;

	const viral_core::string remote_ip_;
	const uint16 remote_port_;

	viral_core::auto_pointer<viral_core::network_stream> stream_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_NETWORK_CLIENT_HPP) */
