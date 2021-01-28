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

#include <map>
#include <unordered_map>
#include <string_view>

#include <nlohmann/json_fwd.hpp>


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
    nlohmann::json process_message(const message_move_ptp&);
    nlohmann::json process_message(const message_move_hybrid_sequence&);
    nlohmann::json process_message(const message_move_contact&);
    nlohmann::json process_message(const message_force_z&);
    nlohmann::json process_message(const message_open_gripper&);
    nlohmann::json process_message(const message_close_gripper&);
    nlohmann::json process_message(const message_grasping_gripper&);
    nlohmann::json process_message(const message_start_recording&);
    nlohmann::json process_message(const message_stop_recording&);
    nlohmann::json process_message(const message_speed&);
    nlohmann::json process_message(const message_error_recovery&);

    template<class MessageType>
    static nlohmann::json process_message_stub(franka_control_server* self, const nlohmann::json& json) {
        const MessageType& msg = json.get<MessageType>();
        return self->process_message(msg);
    } 

    template<class MessageType>
    void handles_message() {
        std::string_view view{MessageType::type};
        _handlers[view] = &process_message_stub<MessageType>;
    }

    using message_handler = nlohmann::json(*)(franka_control_server* self, const nlohmann::json&);
    std::map<std::string_view, message_handler> _handlers; 


	void task_main();

	asio::ip::tcp::acceptor create_server(std::uint16_t control_port_);

	void receive_requests();

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
