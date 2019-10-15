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


#include <viral_core/auto_pointer.hpp>
#include <viral_core/string.hpp>
#include <viral_core/network_forward.hpp>
#include <viral_core/thread.hpp>

#include <franka/exception.h>

#include "franka_proxy_share/franka_proxy_messages.hpp"
#include "franka_hardware_controller.hpp"


namespace franka
{
struct GripperState;
struct RobotState;
}


namespace std
{
class mutex;
}


namespace franka_proxy
{


/**
 *************************************************************************
 *
 * @class franka_control_server
 *
 * Sends commands to a Franka Emika Panda robot.
 *
 ************************************************************************/
class franka_control_server : 
	public viral_core::threaded_task
{
public:

	franka_control_server
	(viral_core::network_context& network,
	 uint16 control_port,
	 franka_hardware_controller& controller);

	~franka_control_server() noexcept;


private:

	void task_main() override;

	void receive_requests();
	void process_request(const viral_core::string& request);


	template <class Function>
	static unsigned char execute_exception_to_return_value(Function&& f)
	{
		try
		{
			return f();
		}
		catch (const franka::ControlException&)
		{
			return franka_proxy_messages::feedback_type::control_exception;
		}
		catch (const franka::CommandException&)
		{
			return franka_proxy_messages::feedback_type::command_exception;
		}
		catch (const franka::NetworkException&)
		{
			return franka_proxy_messages::feedback_type::network_exception;
		}
		catch (const franka::InvalidOperationException&)
		{
			return franka_proxy_messages::feedback_type::invalid_operation;
		}
		catch (const franka::RealtimeException&)
		{
			return franka_proxy_messages::feedback_type::realtime_exception;
		}
		catch (const franka::ModelException&)
		{
			return franka_proxy_messages::feedback_type::model_exception;
		}
		catch (const franka::ProtocolException&)
		{
			return franka_proxy_messages::feedback_type::protocol_exception;
		}
		catch (const franka::IncompatibleVersionException&)
		{
			return franka_proxy_messages::feedback_type::incompatible_version;
		}
		catch (const franka::Exception&)
		{
			return franka_proxy_messages::feedback_type::franka_exception;
		}
	}


	franka_hardware_controller& controller_;

	const viral_core::auto_pointer<viral_core::network_server> server_;
	viral_core::auto_pointer<viral_core::network_stream> stream_;

	static constexpr float sleep_seconds_disconnected_ = 0.033f; // todo 30hz?
	static constexpr float sleep_seconds_connected_ = 0.002f; // todo < 16ms?
};


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
	 franka_hardware_controller& controller);

	~franka_state_server() noexcept;


private:

	void task_main() override;
	void send_status_message(const viral_core::string& command);


	franka_hardware_controller& controller_;

	const uint16 state_port_;

	const viral_core::auto_pointer<viral_core::network_server> server_;
	viral_core::auto_pointer<viral_core::network_connection> connection_;

	static constexpr float sleep_seconds_disconnected_ = 0.033f; // todo 30hz?
	static constexpr float sleep_seconds_connected_ = 0.002f; // todo < 16ms?
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_SERVER_HPP) */
