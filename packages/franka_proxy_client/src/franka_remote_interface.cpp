/**
 *************************************************************************
 *
 * @file franka_remote_interface.cpp
 *
 * Client side implementation of the franka_proxy, implementation.
 *
 ************************************************************************/


#include "franka_remote_interface.hpp"

#include <iostream>
#include <list>
#include <utility>

#include <franka_proxy_share/franka_proxy_commands.hpp>
#include <franka_proxy_share/franka_proxy_util.hpp>

#include "exception.hpp"


namespace franka_proxy
{
//////////////////////////////////////////////////////////////////////////
//
// franka_remote_interface
//
//////////////////////////////////////////////////////////////////////////


franka_remote_interface::franka_remote_interface
(std::string proxy_ip)
	: franka_ip_(std::move(proxy_ip)),
	  current_config_(),
	  current_gripper_pos_(),
	  max_gripper_pos_()
{
	initialize_sockets();
}


franka_remote_interface::~franka_remote_interface() noexcept
{
	shutdown_sockets();
}


void franka_remote_interface::move_to(const robot_config_7dof& target)
{
	send_command<command_move_to_config>(target);
}


void franka_remote_interface::move_to(const Eigen::Vector<double, 7>& target)
{
	const std::array<double, 7> pos = franka_proxy_util::convert_to_std_array<double, 7>(target);
	move_to(pos);
}


bool franka_remote_interface::move_to_until_contact
(const robot_config_7dof& target)
{
	return send_command<command_move_until_contact>(target) == command_result::success;
}


void franka_remote_interface::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<std::array<double, 6>>& f_sequence,
	const std::vector<std::array<double, 6>>& selection_vector_sequence,
	const std::array<double,16>& offset_position,
	const std::array<double,6>& offset_force)
{
	send_command<command_move_hybrid_sequence_with_offset>
		(q_sequence, f_sequence, selection_vector_sequence,offset_position,offset_force);
}

void franka_remote_interface::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<std::array<double, 6>>& f_sequence,
	const std::vector<std::array<double, 6>>& selection_vector_sequence
	)
{
	send_command<command_move_hybrid_sequence>
		(q_sequence, f_sequence, selection_vector_sequence);
}



void franka_remote_interface::apply_admittance(
	double duration, double adm_rotational_stiffness,
	double adm_translational_stiffness, double imp_rotational_stiffness,
	double imp_translational_stiffness, std::optional<std::string> log_file_path)
{
	send_command<command_apply_admittance_adm_imp_desired_stiffness>(
		duration, adm_rotational_stiffness,
		adm_translational_stiffness,
		imp_rotational_stiffness,
		imp_translational_stiffness, log_file_path);
}

void franka_remote_interface::cartesian_impedance_hold_pose(
	double duration,
	bool use_stiff_damp_online_calc,
	double rotational_stiffness,
	double translational_stiffness,
	std::optional<std::string> log_file_path)
{
	send_command<command_cartesian_impedance_hold_pose_desired_stiffness>(
		duration,
		use_stiff_damp_online_calc,
		rotational_stiffness,
		translational_stiffness,
		log_file_path);
}


void franka_remote_interface::cartesian_impedance_poses(
	std::list<std::array<double, 16>>& poses,
	double duration,
	bool use_stiff_damp_online_calc,
	double rotational_stiffness,
	double translational_stiffness,
	std::optional<std::string> log_file_path)
{
	send_command<command_cartesian_impedance_poses_desired_stiffness>(
		poses,
		duration,
		use_stiff_damp_online_calc,
		rotational_stiffness,
		translational_stiffness,
		log_file_path);
}

void franka_remote_interface::joint_impedance_hold_position(
	double duration,
	std::array<double, 49> stiffness,
	std::optional<std::string> log_file_path)
{
	send_command<command_joint_impedance_hold_position_desired_stiffness>(
		duration, stiffness, log_file_path);
}

void franka_remote_interface::joint_impedance_positions(
	std::list<std::array<double, 7>>& joint_positions,
	double duration,
	std::array<double, 49> stiffness,
	std::optional<std::string> log_file_path)
{
	send_command<command_joint_impedance_positions_desired_stiffness>(
		joint_positions, duration, stiffness, log_file_path);
}

void franka_remote_interface::ple_motion(double speed, double duration, std::optional<std::string> log_file_path)
{
	send_command<command_ple_motion>(speed, duration, log_file_path);
}


void franka_remote_interface::apply_z_force(double mass, double duration)
{
	send_command<command_force_z>(mass, duration);
}

void franka_remote_interface::open_gripper(double speed)
{
	send_command<command_open_gripper>(speed);
}

void franka_remote_interface::close_gripper(double speed)
{
	send_command<command_close_gripper>(speed);
}


bool franka_remote_interface::grasp_gripper(double speed, double force)
{
	return send_command<command_grasp_gripper>(speed, force) == command_result::success;
}


void franka_remote_interface::set_speed_factor(double speed_factor)
{
	send_command<command_set_speed>(speed_factor);
}


void franka_remote_interface::set_fts_bias(const std::array<double, 6>& bias)
{
	send_command<command_set_fts_bias>(bias);
}


void franka_remote_interface::set_fts_load_mass(const std::array<double, 3>& load_mass)
{
	send_command<command_set_fts_load_mass>(load_mass);
}


void franka_remote_interface::automatic_error_recovery()
{
	send_command<command_recover_from_errors>();
}


void franka_remote_interface::set_guiding_params(bool x, bool y, bool z, bool rx, bool ry, bool rz, bool elbow)
	{ send_command<command_set_guiding_params>(std::array<bool, 6>{ x, y, z, rx, ry, rz }, elbow); }


robot_config_7dof franka_remote_interface::current_config() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_config_;
}


double franka_remote_interface::current_gripper_pos() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_gripper_pos_;
}


double franka_remote_interface::max_gripper_pos() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return max_gripper_pos_;
}


bool franka_remote_interface::gripper_grasped() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return gripper_grasped_;
}


void franka_remote_interface::start_recording(std::optional<std::string> log_file_path)
{
	send_command<command_start_recording>(log_file_path);
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
franka_remote_interface::stop_recording()
{
	auto [q_sequence, f_sequence] = send_command<command_stop_recording>();
	return {q_sequence, f_sequence};
}


void franka_remote_interface::update()
{
	while (socket_state_->states().empty())
		socket_state_->update_messages();

	for (const auto& state : socket_state_->states())
	{
		std::lock_guard lck(state_lock_);
		current_config_ = state.joint_configuration;
		current_gripper_pos_ = state.width;
		max_gripper_pos_ = state.max_width;
		gripper_grasped_ = state.is_grasped;

		//vacuum gripper
		vacuum_gripper_state_.actual_power_ = state.actual_power;
		vacuum_gripper_state_.vacuum_level = state.vacuum;
		vacuum_gripper_state_.part_detached_ = state.part_detached;
		vacuum_gripper_state_.part_present_ = state.part_present;
		vacuum_gripper_state_.in_control_range_ = state.in_control_range;
	}

	socket_state_->clear_states();
}

command_result franka_remote_interface::check_response(const command_generic_response& response)
{
	switch (response.result)
	{
	case command_result::success:
	case command_result::success_command_failed:
		return response.result;
	case command_result::model_exception:
		throw model_exception{response.reason};
	case command_result::network_exception:
		throw network_exception{response.reason};
	case command_result::protocol_exception:
		throw protocol_exception{response.reason};
	case command_result::incompatible_version:
		throw incompatible_version_exception{response.reason};
	case command_result::control_exception:
		throw control_exception{response.reason};
	case command_result::command_exception:
		throw command_exception{response.reason};
	case command_result::realtime_exception:
		throw realtime_exception{response.reason};
	case command_result::invalid_operation:
		throw invalid_operation_exception{response.reason};
	case command_result::force_torque_sensor_exception:
		throw ft_sensor_exception{};
	case command_result::unknown_command:
		throw unknown_command_exception{response.reason};
	default:
		throw bad_response_exception{};
	}
}


void franka_remote_interface::initialize_sockets()
{
	std::cout << "franka_remote_interface::initialize_sockets(): " <<
		"Creating network connections.\n";

	socket_control_.reset
		(new franka_control_client(franka_ip_, franka_control_port));

	socket_state_.reset
		(new franka_state_client(franka_ip_, franka_state_port));
}


void franka_remote_interface::shutdown_sockets() noexcept
{
	socket_control_.reset();
	socket_state_.reset();
}

bool franka_remote_interface::vacuum_gripper_drop(std::chrono::milliseconds timeout)
{
	return send_command<command_vacuum_gripper_drop>(timeout)==command_result::success;
}

bool franka_remote_interface::vacuum_gripper_vacuum(std::uint8_t vacuum_strength, std::chrono::milliseconds timeout)
{
	return send_command<command_vacuum_gripper_vacuum>(vacuum_strength,timeout) == command_result::success;
}

bool franka_remote_interface::vacuum_gripper_stop()
{
	return send_command<command_vacuum_gripper_stop>() == command_result::success;
}

vacuum_gripper_state franka_remote_interface::get_vacuum_gripper_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return vacuum_gripper_state_;
}

} /* namespace franka_proxy */
