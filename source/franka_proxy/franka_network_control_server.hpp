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


#include <map>

#include <asio/ip/tcp.hpp>

#include <nlohmann/json_fwd.hpp>

#include <franka_proxy_share/franka_proxy_commands.hpp>

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

	using command_handler = nlohmann::json(*)(franka_control_server* self, const nlohmann::json&);

	template<class TCommandType>
		void register_command_handler()
	{
		std::string_view type{ TCommandType::type };
		command_handler handler =
			[](franka_control_server* self, const nlohmann::json& json) -> nlohmann::json
		{
			static_assert
				(std::is_same_v
					<decltype(self->process_command(std::declval<TCommandType>())),
					 typename TCommandType::response_type>,
				 "A command handler's return type must match the response type associated with the command.");

			return self->process_command(json.get<TCommandType>());
		};

		command_handlers_[TCommandType::type] = handler;
	}

	void task_main();

	asio::ip::tcp::acceptor create_server(std::uint16_t control_port_);

	void receive_requests();

	command_generic_response process_command(const command_move_to_config&);
	command_generic_response process_command(const command_move_hybrid_sequence&);
	command_generic_response process_command(const command_move_until_contact&);
	command_generic_response process_command(const command_apply_admittance&);
	command_generic_response process_command(const command_apply_admittance_adm_desired_stiffness&);
	command_generic_response process_command(const command_apply_admittance_adm_imp_desired_stiffness&);
	command_generic_response process_command(const command_cartesian_impedance_hold_pose&);
	command_generic_response process_command(const command_cartesian_impedance_hold_pose_desired_stiffness&);
	command_generic_response process_command(const command_cartesian_impedance_poses&);
	command_generic_response process_command(const command_cartesian_impedance_poses_desired_stiffness&);
	command_generic_response process_command(const command_joint_impedance_hold_position&);
	command_generic_response process_command(const command_joint_impedance_hold_position_desired_stiffness&);
	command_generic_response process_command(const command_joint_impedance_positions&);
	command_generic_response process_command(const command_joint_impedance_positions_desired_stiffness&);
	command_generic_response process_command(const command_force_z&);
	command_generic_response process_command(const command_open_gripper&);
	command_generic_response process_command(const command_close_gripper&);
	command_generic_response process_command(const command_grasp_gripper&);
	command_generic_response process_command(const command_start_recording&);
	command_stop_recording_response process_command(const command_stop_recording&);
	command_generic_response process_command(const command_set_speed&);
	command_generic_response process_command(const command_recover_from_errors&);

	franka_hardware_controller& controller_;

	asio::io_context io_context_;
	asio::ip::tcp::acceptor server_;
	std::unique_ptr<asio::ip::tcp::socket> connection_;
	
	std::string messages_buffer_;

	std::thread internal_thread_;
	std::atomic_bool terminate_internal_thread_;

	std::map<std::string_view, command_handler> command_handlers_;

	static constexpr float sleep_seconds_disconnected_ = 0.033f; // todo 30hz?
	static constexpr float sleep_seconds_connected_ = 0.002f; // todo <16ms?
	static constexpr std::size_t receive_buffer_size_ = 1000;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_NETWORK_CONTROL_SERVER_HPP) */
