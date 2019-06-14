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

#include "franka_proxy_share/franka_proxy_messages.hpp"

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
	network_(network)
{
	initialize_sockets();
}


franka_remote_controller::~franka_remote_controller() noexcept
{
	shutdown_sockets();
}


void franka_remote_controller::move_to(const robot_config_7dof& target)
{
	string msg = 
		(std::string(franka_proxy_messages::move) + ' ' +
		 std::to_string(target[0]) + ' ' +
		 std::to_string(target[0]) + ' ' +
		 std::to_string(target[1]) + ' ' +
		 std::to_string(target[2]) + ' ' +
		 std::to_string(target[3]) + ' ' +
		 std::to_string(target[4]) + ' ' +
		 std::to_string(target[5]) + ' ' +
		 std::to_string(target[6]) + '\n').data();
	socket_control_->send_command(msg);
}


void franka_remote_controller::stop_movement()
{
	socket_control_->send_command
		(string(franka_proxy_messages::stop_move) + '\n');
}


robot_config_7dof franka_remote_controller::current_config() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_config_;
}


double franka_remote_controller::speed_factor() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_speed_factor_;
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	socket_control_->send_command
		((std::string(franka_proxy_messages::speed) + ' ' +
		  std::to_string(speed_factor) + '\n').data());
}


void franka_remote_controller::open_gripper()
{
	socket_control_->send_command
		(string(franka_proxy_messages::open_gripper) + '\n');
}
void franka_remote_controller::close_gripper()
{
	socket_control_->send_command
		(string(franka_proxy_messages::close_gripper) + '\n');
}


bool franka_remote_controller::gripper_open()
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return gripper_open_;
}


void franka_remote_controller::stop_gripper_movement()
{
	socket_control_->send_command
		(string(franka_proxy_messages::stop_gripper) + '\n');
}


void franka_remote_controller::update()
{
	std::lock_guard<std::mutex> state_guard(state_lock_);

	socket_state_->update_messages();
	list<string> messages
		(socket_state_->messages());

	for (list<string>::iterator it(messages.first()); it; ++it)
	{
		// Incoming format from TX90:
		// HH-MM-SS conf:j1,j2,j3,j4,j5,j6,j7$<gripper-open>$<gripper-position>$<gripper-max-position>$<error-code>


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

		if (state_list.size() != 5)
		{
			LOG_WARN("State message has invalid format: " + *it);
			continue;
		}


		// Fetch joint angles.
		list<string> joint_values_list;
		state_list[0].split(',', joint_values_list);

		if (joint_values_list.size() < 7)
		{
			LOG_WARN("State message does not have 7 joint values: " + *it);
			continue;
		}

		current_config_ =
			{joint_values_list[0].to_float(),
			 joint_values_list[1].to_float(),
			 joint_values_list[2].to_float(),
			 joint_values_list[3].to_float(),
			 joint_values_list[4].to_float(),
			 joint_values_list[5].to_float(),
			 joint_values_list[6].to_float()};


		// Fetch gripper state.
		gripper_open_ =
			state_list[1].to_int32() != 0;
		current_gripper_pos_ =
			state_list[2].to_int32() != 0;
		max_gripper_pos_ =
			state_list[3].to_int32() != 0;
	

		// Fetch error
		current_error_ = state_list[4].to_int32();
	}

	switch (current_error_)
	{
		case 0: break;
		case 1: throw model_exception();
		case 2: throw network_exception();
		case 3: throw protocol_exception();
		case 4: throw incompatible_version_exception();
		case 5: throw control_exception();
		case 6: throw command_exception();
		case 7: throw realtime_exception();
		case 8: throw invalid_operation_exception();
		default: throw remote_exception();
	}

}


void franka_remote_controller::initialize_sockets()
{
	LOG_INFO("Creating network connections.");

	socket_control_.reset
		(new franka_control_client
			(network_, franka_ip, franka_controll_port));

	socket_state_.reset
		(new franka_state_client
			(network_, franka_ip, franka_state_port));
}


void franka_remote_controller::shutdown_sockets() noexcept
{
	socket_control_.reset();
	socket_state_.reset();
}




} /* namespace franka_proxy */