/**
 *************************************************************************
 *
 * @file franka_remote_controller.cpp
 *
 * Client side implementation of the franka_proxy, implementation.
 *
 ************************************************************************/

#include <franka_proxy_share/franka_proxy_commands.hpp>

#include <franka_proxy_client/franka_remote_controller.hpp>
#include <franka_proxy_client/exception.hpp>

#include <iostream>
#include <list>
#include <utility>

namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_remote_controller
//
//////////////////////////////////////////////////////////////////////////


franka_remote_controller::franka_remote_controller
	(std::string proxy_ip)
	:
	franka_ip_(std::move(proxy_ip)),
	current_config_(),
	current_gripper_pos_(),
	max_gripper_pos_()
{
	initialize_sockets();
}


franka_remote_controller::~franka_remote_controller() noexcept
{
	shutdown_sockets();
}


void franka_remote_controller::move_to(const robot_config_7dof& target)
{
	send_command<command_move_to_config>(target);
}


bool franka_remote_controller::move_to_until_contact
	(const robot_config_7dof& target)
{
	return send_command<command_move_until_contact>(target) == command_result::success;
}


void franka_remote_controller::move_sequence
	(const std::vector<robot_config_7dof>& q_sequence,
	 const std::vector<std::array<double, 6>>& f_sequence,
	 const std::vector<std::array<double, 6>>& selection_vector_sequence)
{
	send_command<command_move_hybrid_sequence>(q_sequence, f_sequence, selection_vector_sequence);
}


void franka_remote_controller::apply_z_force
	(double mass,
	 double duration)
{
	send_command<command_force_z>(mass, duration);
}


void franka_remote_controller::open_gripper()
{
	send_command<command_open_gripper>();
}


void franka_remote_controller::close_gripper()
{
	send_command<command_close_gripper>();
}


bool franka_remote_controller::grasp_gripper(double speed, double force)
{
	return send_command<command_grasp_gripper>(speed, force) == command_result::success;
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	send_command<command_set_speed>(speed_factor);
}


void franka_remote_controller::automatic_error_recovery()
{
	send_command<command_recover_from_errors>();
}


robot_config_7dof franka_remote_controller::current_config() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_config_;
}


int franka_remote_controller::current_gripper_pos() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_gripper_pos_;
}


int franka_remote_controller::max_gripper_pos() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return max_gripper_pos_;
}


bool franka_remote_controller::gripper_grasped() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return gripper_grasped_;
}


void franka_remote_controller::start_recording()
{
	send_command<command_start_recording>();
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_remote_controller::stop_recording()
{
	const auto&[q_sequence, f_sequence] = send_command<command_stop_recording>();	
	return {q_sequence, f_sequence};
}


void franka_remote_controller::update()
{
	while (socket_state_->states().empty()) {
		socket_state_->update_messages();
	}

	for (const auto& state : socket_state_->states())
	{
		std::lock_guard lck(state_lock_);
		current_config_ = state.q;
		current_gripper_pos_ = static_cast<int>(state.width);
		max_gripper_pos_ = static_cast<int>(state.max_width);
		gripper_grasped_ = state.is_grasped;
	}

	socket_state_->clear_states();
}


command_result franka_remote_controller::check_result(command_result result)
{
	switch (result) {
		case command_result::success:
		case command_result::success_command_failed:
			return result;
		case command_result::model_exception:
			throw model_exception{};
		case command_result::network_exception:
			throw network_exception{};
		case command_result::protocol_exception:
			throw protocol_exception{};
		case command_result::incompatible_version:
			throw incompatible_version_exception{};
		case command_result::control_exception:
			throw control_exception{};
		case command_result::command_exception:
			throw command_exception{};
		case command_result::realtime_exception:
			throw realtime_exception{};
		case command_result::invalid_operation:
			throw invalid_operation_exception{};
		default:
			throw remote_exception{};
	}
}


void franka_remote_controller::initialize_sockets()
{
	std::cout << "franka_remote_controller::initialize_sockets(): " <<
				"Creating network connections.";

	socket_control_.reset
		(new franka_control_client(franka_ip_.data(), franka_control_port));

	socket_state_.reset
		(new franka_state_client(franka_ip_.data(), franka_state_port));
}


void franka_remote_controller::shutdown_sockets() noexcept
{
	socket_control_.reset();
	socket_state_.reset();
}

} /* namespace franka_proxy */
