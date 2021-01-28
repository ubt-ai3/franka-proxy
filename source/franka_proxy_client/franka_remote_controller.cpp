/**
 *************************************************************************
 *
 * @file franka_remote_controller.cpp
 *
 * Client side implementation of the franka_proxy, implementation.
 *
 ************************************************************************/


#include "franka_remote_controller.hpp"

#include <iostream>
#include <list>
#include <utility>

#include "exception.hpp"


namespace 
{
	using namespace franka_proxy;

	message_result check_result(message_generic_response res)
	{
		switch (static_cast<message_result>(res.status_code)) {
			case message_result::success:
			case message_result::success_command_failed:
				return static_cast<message_result>(res.status_code);
			case message_result::model_exception:
				throw model_exception{};
			case message_result::network_exception:
				throw network_exception{};
			case message_result::protocol_exception:
				throw protocol_exception{};
			case message_result::incompatible_version:
				throw incompatible_version_exception{};
			case message_result::control_exception:
				throw control_exception{};
			case message_result::command_exception:
				throw command_exception{};
			case message_result::realtime_exception:
				throw realtime_exception{};
			case message_result::invalid_operation:
				throw invalid_operation_exception{};
			default:
				throw remote_exception{};
		}
	}

}

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
    message_move_ptp msg{}; 
    msg.config = target;

    const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


bool franka_remote_controller::move_to_until_contact
	(const robot_config_7dof& target)
{
	message_move_contact msg{};
	msg.config = target;

	const auto res = socket_control_->send_command<message_generic_response>(msg);
	return check_result(res) != message_result::success_command_failed;
}


void franka_remote_controller::move_sequence
	(const std::vector<robot_config_7dof>& q_sequence,
	 const std::vector<std::array<double, 6>>& f_sequence,
	 const std::vector<std::array<double, 6>>& selection_vector_sequence)
{
	message_move_hybrid_sequence msg{};
	msg.q_data = q_sequence;
	msg.f_data = f_sequence;
	msg.s_data = selection_vector_sequence;

	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


void franka_remote_controller::apply_z_force
	(double mass,
	 double duration)
{
	message_force_z msg{};
	msg.mass = mass;
	msg.duration = duration;
	
	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


void franka_remote_controller::open_gripper()
{
	message_open_gripper msg{};
	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


void franka_remote_controller::close_gripper()
{
	message_close_gripper msg{};
	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


bool franka_remote_controller::grasp_gripper(double speed, double force)
{
	message_grasping_gripper msg{};
	msg.speed = speed;
	msg.force = force;

	const auto res = socket_control_->send_command<message_generic_response>(msg);
	return check_result(res) != message_result::success_command_failed;
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	message_speed msg{};
	msg.speed = speed_factor;
	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
}


void franka_remote_controller::automatic_error_recovery()
{
	message_error_recovery msg{};
	const auto res = socket_control_->send_command<message_generic_response>(msg);
	check_result(res);
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
	message_start_recording msg{};
	const auto res = socket_control_->send_command(msg);
	check_result(res);
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_remote_controller::stop_recording()
{
	message_stop_recording msg{};
	auto resp = socket_control_->send_command<message_stop_recording_response>(msg);

	return { resp.q_sequence, resp.f_sequence };
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
