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
	std::string msg =
	std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move_ptp]) + ' ' +
		std::to_string(target[0]) + ' ' +
		std::to_string(target[1]) + ' ' +
		std::to_string(target[2]) + ' ' +
		std::to_string(target[3]) + ' ' +
		std::to_string(target[4]) + ' ' +
		std::to_string(target[5]) + ' ' +
		std::to_string(target[6]) +
		franka_proxy_messages::command_end_marker;

	check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));
}


bool franka_remote_controller::move_to_until_contact
	(const robot_config_7dof& target)
{
	std::string msg =
	std::string(franka_proxy_messages::command_strings[franka_proxy_messages::move_contact]) + ' ' +
		std::to_string(target[0]) + ' ' +
		std::to_string(target[1]) + ' ' +
		std::to_string(target[2]) + ' ' +
		std::to_string(target[3]) + ' ' +
		std::to_string(target[4]) + ' ' +
		std::to_string(target[5]) + ' ' +
		std::to_string(target[6]) +
		franka_proxy_messages::command_end_marker;

	const response_type response = check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));

	return response != response_type::success_command_failed;
}


void franka_remote_controller::move_sequence
	(const std::vector<robot_config_7dof>& q_sequence,
	 const std::vector<std::array<double, 6>>& f_sequence,
	 const std::vector<std::array<double, 6>>& selection_vector_sequence)
{
	return socket_control_->send_move_sequence(q_sequence, f_sequence, selection_vector_sequence);
}


void franka_remote_controller::apply_z_force
	(double mass,
	 double duration)
{
	std::string msg =
	std::string(franka_proxy_messages::command_strings[franka_proxy_messages::force_z]) + ' ' +
		std::to_string(mass) + ' ' +
		std::to_string(duration) +
		franka_proxy_messages::command_end_marker;

	check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));
}


void franka_remote_controller::open_gripper()
{
	std::string msg = std::string
		(franka_proxy_messages::command_strings[franka_proxy_messages::open_gripper]) +
		franka_proxy_messages::command_end_marker;
	unsigned char response =
		socket_control_->send_command_and_check_response(msg);
	check_response
		(franka_proxy_messages::feedback_type(response));
}


void franka_remote_controller::close_gripper()
{
	std::string msg = std::string
		(franka_proxy_messages::command_strings[franka_proxy_messages::close_gripper]) +
		franka_proxy_messages::command_end_marker;

	check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));
}


bool franka_remote_controller::grasp_gripper(double speed, double force)
{
	std::string msg =
	(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::grasp_gripper]) + ' ' +
		std::to_string(speed) + ' ' + std::to_string(force) +
		franka_proxy_messages::command_end_marker).data();

	response_type response = check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));

	return response != response_type::success_command_failed;
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	socket_control_->send_command
		(std::string
		 (franka_proxy_messages::command_strings[franka_proxy_messages::speed]) +
		 " " + std::to_string(speed_factor) +
		 franka_proxy_messages::command_end_marker);
}


void franka_remote_controller::automatic_error_recovery()
{
	socket_control_->send_command
		(std::string
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


void franka_remote_controller::start_recording()
{
	std::string msg =
	(std::string(franka_proxy_messages::command_strings[franka_proxy_messages::start_recording]) +
		franka_proxy_messages::command_end_marker).data();

	check_response
		(franka_proxy_messages::feedback_type
		 (socket_control_->send_command_and_check_response(msg)));
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
	franka_remote_controller::stop_recording()
{
	return socket_control_->send_stop_recording_and_receive_sequence();
}


void franka_remote_controller::update()
{
	std::list<std::string> messages;
	while (messages.empty())
	{
		socket_state_->update_messages();
		messages = std::list<std::string>
			(socket_state_->messages());
	}

	for (const std::string& message : messages)
	{
		// Incoming format from TX90:
		// conf:j1,j2,j3,j4,j5,j6,j7$<gripper-position>$<gripper-max-position>$<gripper-is-grasped>


		// Separate state values from message string.
		std::string::size_type colon = message.find(':');
		if (colon == std::string::npos)
		{
			std::cerr << "franka_remote_controller::update(): " <<
				"State message is missing colon: " << message;
			continue;
		}

		std::string state_str
			(message.substr(colon + 1));

		auto split = [](const std::string& s, const std::string& delim) -> std::vector<std::string>
		{
			std::size_t pos_start = 0;
			std::size_t pos_end;
			std::size_t delim_len = delim.length();

			std::string token;
			std::vector<std::string> res;

			while ((pos_end = s.find(delim, pos_start)) != std::string::npos)
			{
				token = s.substr (pos_start, pos_end - pos_start);
				pos_start = pos_end + delim_len;
				res.push_back (token);
			}

			res.push_back (s.substr (pos_start));
			return res;
		};

		std::vector<std::string> state_list = split(state_str, "$");

		if (state_list.size() != 4)
		{
			std::cerr << "franka_remote_controller::update(): " <<
				"State message has invalid format: " << message;
			continue;
		}


		// Fetch joint angles.
		std::vector<std::string> joint_values_list = split(state_list.front(), ",");

		if (joint_values_list.size() != 7)
		{
			std::cerr << "franka_remote_controller::update(): " <<
				"State message does not have 7 joint values: " << message;
			continue;
		}

		{
			std::lock_guard<std::mutex> state_guard(state_lock_);

			current_config_ =
			{
				stod(joint_values_list[0]),
				stod(joint_values_list[1]),
				stod(joint_values_list[2]),
				stod(joint_values_list[3]),
				stod(joint_values_list[4]),
				stod(joint_values_list[5]),
				stod(joint_values_list[6])
			};


			// Fetch gripper state.
			current_gripper_pos_ =
				std::stoi(state_list[1]);
			max_gripper_pos_ =
				std::stoi(state_list[2]);
			gripper_grasped_ =
				std::stoi(state_list[3]) > 0;
		}
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


franka_remote_controller::response_type
	franka_remote_controller::check_response
		(franka_proxy_messages::feedback_type response)
{
	switch (response)
	{
		case franka_proxy_messages::success:
			return response_type::success;
		case franka_proxy_messages::success_command_failed:
			return response_type::success_command_failed;
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
