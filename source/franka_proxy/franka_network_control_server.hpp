/**
 *************************************************************************
 *
 * @file franka_network_control_server.hpp
 *
 * Network communication with a franka_proxy_client.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_CONTROL_SERVER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_CONTROL_SERVER_HPP


#include <asio/ip/tcp.hpp>

#include <franka/exception.h>

#include <franka_proxy_share/franka_proxy_messages.hpp>
#include "franka_hardware_controller.hpp"


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
class franka_control_server
{
public:

	franka_control_server
		(std::uint16_t control_port,
		 franka_hardware_controller& controller);

	~franka_control_server() noexcept;


private:

	void task_main();

	asio::ip::tcp::acceptor create_server(std::uint16_t control_port_);

	void receive_requests();
	void process_request(const std::string& request);

	static std::vector<std::string> split_string
		(const std::string& s, const std::string& delim);


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

	asio::io_context io_context_;
	asio::ip::tcp::acceptor server_;
	std::unique_ptr<asio::ip::tcp::socket> connection_;
	
	std::string messages_buffer_;

	std::thread internal_thread_;
	std::atomic_bool terminate_internal_thread_;

	static constexpr float sleep_seconds_disconnected_ = 0.033f; // todo 30hz?
	static constexpr float sleep_seconds_connected_ = 0.002f; // todo <16ms?
	static constexpr std::size_t receive_buffer_size_ = 1000;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_CONTROL_SERVER_HPP) */
