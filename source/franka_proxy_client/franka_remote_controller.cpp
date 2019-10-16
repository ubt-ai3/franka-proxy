/**
 *************************************************************************
 *
 * @file franka_remote_controller.cpp
 *
 * Client side implementation of the franka_proxy, implementation.
 *
 ************************************************************************/


#include "franka_remote_controller.hpp"

#include <viral_core/log.hpp>

#include "exception.hpp"


namespace franka_proxy
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_remote_controller
//
//////////////////////////////////////////////////////////////////////////


franka_remote_controller::franka_remote_controller
	(const std::string& proxy_ip,
	 network_context& network)
	:
	franka_ip_(proxy_ip),
	network_(network),
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


void franka_remote_controller::apply_z_force(double mass, double duration)
{
	string msg =
		(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::force_z]) + ' ' +
		 std::to_string(mass) + ' ' +
		 std::to_string(duration) +
		 franka_proxy_messages::command_end_marker).data();
	socket_control_->send_command(msg);

	check_response
		(franka_proxy_messages::feedback_type
			(socket_control_->receive_response()));
}


void franka_remote_controller::move_to(const robot_config_7dof& target)
{
	string msg =
		(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move]) + ' ' +
		 std::to_string(target[0]) + ' ' +
		 std::to_string(target[1]) + ' ' +
		 std::to_string(target[2]) + ' ' +
		 std::to_string(target[3]) + ' ' +
		 std::to_string(target[4]) + ' ' +
		 std::to_string(target[5]) + ' ' +
		 std::to_string(target[6]) +
		 franka_proxy_messages::command_end_marker).data();
	socket_control_->send_command(msg);

	check_response
		(franka_proxy_messages::feedback_type
			(socket_control_->receive_response()));
}


bool franka_remote_controller::move_to_until_contact
	(const robot_config_7dof& target)
{
	string msg =
		(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move_contact]) + ' ' +
		 std::to_string(target[0]) + ' ' +
		 std::to_string(target[1]) + ' ' +
		 std::to_string(target[2]) + ' ' +
		 std::to_string(target[3]) + ' ' +
		 std::to_string(target[4]) + ' ' +
		 std::to_string(target[5]) + ' ' +
		 std::to_string(target[6]) +
		 franka_proxy_messages::command_end_marker).data();
	socket_control_->send_command(msg);

	response_type response = check_response
		(franka_proxy_messages::feedback_type
			(socket_control_->receive_response()));

	if (response == response_type::success_contact)
		return false;
	return true;
}


void franka_remote_controller::open_gripper()
{
	socket_control_->send_command(string
		(franka_proxy_messages::command_strings[franka_proxy_messages::open_gripper]) +
		 franka_proxy_messages::command_end_marker);

	check_response
		(franka_proxy_messages::feedback_type
			(socket_control_->receive_response()));
}


void franka_remote_controller::close_gripper(double speed, double force)
{
	string msg =
		(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::close_gripper]) + ' ' +
		 std::to_string(speed) + ' ' + std::to_string(force) +
		 franka_proxy_messages::command_end_marker).data();
	socket_control_->send_command(msg);

	check_response
		(franka_proxy_messages::feedback_type
			(socket_control_->receive_response()));
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	socket_control_->send_command(string
		(franka_proxy_messages::command_strings[franka_proxy_messages::speed]) +
		 " " + static_cast<float>(speed_factor) +
		 franka_proxy_messages::command_end_marker);
}


void franka_remote_controller::automatic_error_recovery()
{
	socket_control_->send_command(string
		(franka_proxy_messages::command_strings[franka_proxy_messages::error_recovery]) +
		 franka_proxy_messages::command_end_marker);
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


void franka_remote_controller::update()
{
	std::lock_guard<std::mutex> state_guard(state_lock_);

	list<string> messages;
	while (messages.empty())
	{
		socket_state_->update_messages();
		messages = list<string>
			(socket_state_->messages());
	}

	for (list<string>::iterator it(messages.first()); it; ++it)
	{
		// Incoming format from TX90:
		// conf:j1,j2,j3,j4,j5,j6,j7$<gripper-position>$<gripper-max-position>$<gripper-is-grasped>


		// Separate state values from message string.
		int64 colon = it->seek(':');
		if (colon == string::invalid_index)
		{
			LOG_WARN("State message is missing colon: " + *it);
			continue;
		}

		string state_str
			(it->substring(colon + 1));

		list<string> state_list;
		state_str.split('$', state_list);

		if (state_list.size() != 4)
		{
			LOG_WARN("State message has invalid format: " + *it);
			continue;
		}


		// Fetch joint angles.
		list<string> joint_values_list;
		state_list[0].split(',', joint_values_list);

		if (joint_values_list.size() != 7)
		{
			LOG_WARN("State message does not have 7 joint values: " + *it);
			continue;
		}

		current_config_ =
		{
			strtod(joint_values_list[0].data(), nullptr),
			strtod(joint_values_list[1].data(), nullptr),
			strtod(joint_values_list[2].data(), nullptr),
			strtod(joint_values_list[3].data(), nullptr),
			strtod(joint_values_list[4].data(), nullptr),
			strtod(joint_values_list[5].data(), nullptr),
			strtod(joint_values_list[6].data(), nullptr)
		};


		// Fetch gripper state.
		current_gripper_pos_ =
			state_list[1].to_int32();
		max_gripper_pos_ =
			state_list[2].to_int32();
		gripper_grasped_ =
			state_list[3].to_int32() > 0;
	}
}


void franka_remote_controller::initialize_sockets()
{
	LOG_INFO("Creating network connections.");

	socket_control_.reset
	(new franka_control_client
		(network_, franka_ip_.data(), franka_control_port));

	socket_state_.reset
	(new franka_state_client
		(network_, franka_ip_.data(), franka_state_port));
}


void franka_remote_controller::shutdown_sockets() noexcept
{
	socket_control_.reset();
	socket_state_.reset();
}


franka_remote_controller::response_type
	franka_remote_controller::check_response
		(franka_proxy_messages::feedback_type response)
{
	switch (response)
	{
		case franka_proxy_messages::success:
			return response_type::success;
		case franka_proxy_messages::success_contact:
			return response_type::success_contact;
		case franka_proxy_messages::model_exception:
			throw model_exception();
		case franka_proxy_messages::network_exception:
			throw network_exception();
		case franka_proxy_messages::protocol_exception:
			throw protocol_exception();
		case franka_proxy_messages::incompatible_version:
			throw incompatible_version_exception();
		case franka_proxy_messages::control_exception:
			throw control_exception();
		case franka_proxy_messages::command_exception:
			throw command_exception();
		case franka_proxy_messages::realtime_exception:
			throw realtime_exception();
		case franka_proxy_messages::invalid_operation:
			throw invalid_operation_exception();
		default:
			throw remote_exception();
	}
}




} /* namespace franka_proxy */