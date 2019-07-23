/**
 *************************************************************************
 *
 * @file franka_network_server.hpp
 *
 * Network communication with a franka_proxy_client.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP


#include <viral_core/network_forward.hpp>


#include <viral_core/auto_pointer.hpp>
#include <viral_core/list.hpp>
#include <viral_core/string.hpp>
#include "viral_core/thread.hpp"


namespace franka {
struct GripperState;
struct RobotState;
}


namespace std {
class mutex;
}


namespace franka_proxy
{


///**
// *************************************************************************
// *
// * @class franka_control_server
// *
// * Sends commands to a Franka Emika Panda robot.
// *
// ************************************************************************/
//class franka_control_server
//{
//
//	public:
//
//		franka_state_client
//			(viral_core::network_context& network,
//			 const viral_core::string& remote_ip,
//			 uint16 remote_port);
//
//		~franka_state_client() NOTHROW;
//
//
//		void update_messages();
//		viral_core::list<viral_core::string> messages() const;
//
//
//	private:
//
//		void update_messages_buffer();
//		viral_core::string fetch_message();
//
//
//		static const int64 receive_buffer_size_ = 1024;
//
//		viral_core::network_context& network_;
//
//		const viral_core::string remote_ip_;
//		const uint16 remote_port_;
//
//		viral_core::auto_pointer<viral_core::network_connection> connection_;
//
//		viral_core::string messages_buffer_;
//		viral_core::list<viral_core::string> messages_;
//
//};




/**
 *************************************************************************
 *
 * @class franka_state_server
 *
 * Send the current state of the robot.
 *
 ************************************************************************/
class franka_state_server :
	public viral_core::threaded_task
{

public:

	franka_state_server
		(viral_core::network_context& network,
		 uint16 state_port,
		 std::mutex& state_lock,
		 franka::RobotState& robot_state,
		 franka::GripperState& gripper_state);

	~franka_state_server() noexcept;


private:

	void task_main() override;

	void send_status_message(const viral_core::string& command);


	viral_core::network_context& network_;

	const uint16 state_port_;

	std::mutex& state_lock_;
	franka::RobotState& robot_state_;
	franka::GripperState& gripper_state_;
	
	const viral_core::auto_pointer<viral_core::network_server> server_;
	viral_core::auto_pointer<viral_core::network_connection> connection_;

	static constexpr float sleep_seconds_disconnected_ = 0.01f;
	static constexpr float sleep_seconds_connected_ = 0.002f;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP) */
