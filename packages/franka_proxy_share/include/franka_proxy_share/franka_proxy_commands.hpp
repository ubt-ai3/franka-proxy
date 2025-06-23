#ifndef INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_COMMANDS_HPP
#define INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_COMMANDS_HPP
/**
 *************************************************************************
 *
 * @file franka_proxy_commands.hpp
 *
 * Commands/Responses that are sent to/received by the proxy server.
 *
 ************************************************************************/


#include <array>
#include <cstdint>
#include <list>
#include <vector>
#include <string>
#include <chrono>
#include <optional>

#include <nlohmann/json_fwd.hpp>


namespace franka_proxy
{
using robot_config_7dof = std::array<double, 7>;


struct command_stop_recording_response;

struct command_generic_response;
enum class command_result : std::uint8_t;

struct command_get_config_response;


/**
 *************************************************************************
 *
 * @class command_move_to_config
 *
 * Commands the robot to move to given config.
 *
 ************************************************************************/
struct command_move_to_config
{
	using response_type = command_generic_response;
	static constexpr char type[] = "move.config";

	std::array<double, 7> target_joint_config;
};

void to_json(nlohmann::json& json, const command_move_to_config& object);
void from_json(const nlohmann::json& json, command_move_to_config& object);


/**
 *************************************************************************
 *
 * @class command_move_until_contact
 *
 * Commands the robot to move to given config until contact with other
 * objects is being registered.
 *
 ************************************************************************/
struct command_move_until_contact
{
	using response_type = command_generic_response;
	static constexpr char type[] = "move.contact";

	std::array<double, 7> target_joint_config;
};

void to_json(nlohmann::json& json, const command_move_until_contact& object);
void from_json(const nlohmann::json& json, command_move_until_contact& object);


/**
 *************************************************************************
 *
 * @class command_move_hybrid_sequence
 *
 * Commands the robot to move following the given hybrid sequence
 * with a given position and force offset.
 *
 ************************************************************************/
struct command_move_hybrid_sequence_with_offset
{
	using response_type = command_generic_response;
	static constexpr char type[] = "move.hybrid-sequence-with-offset";

	std::vector<std::array<double, 7>> joint_config_sequence;
	std::vector<std::array<double, 6>> force_sequence;
	std::vector<std::array<double, 6>> selection_sequence;
	std::array<double, 16> offset_position;
	std::array<double, 6> offset_force;
};

void to_json(nlohmann::json& json, const command_move_hybrid_sequence_with_offset& object);
void from_json(const nlohmann::json& json, command_move_hybrid_sequence_with_offset& object);


/**
 *************************************************************************
 *
 * @class command_move_hybrid_sequence
 *
 * Commands the robot to move following the given hybrid sequence.
 *
 ************************************************************************/
struct command_move_hybrid_sequence
{
	using response_type = command_generic_response;
	static constexpr char type[] = "move.hybrid-sequence";

	std::vector<std::array<double, 7>> joint_config_sequence;
	std::vector<std::array<double, 6>> force_sequence;
	std::vector<std::array<double, 6>> selection_sequence;
};

void to_json(nlohmann::json& json, const command_move_hybrid_sequence& object);
void from_json(const nlohmann::json& json, command_move_hybrid_sequence& object);

/**
 *************************************************************************
 *
 * @class command_apply_admittance_adm_imp_desired_stiffness
 *
 * Commands the robot to use the admittance controller with desired
 * rotational and translational stiffness within the admittance and the
 * impedance controller
 *
 ************************************************************************/
struct command_apply_admittance_adm_imp_desired_stiffness
{
	using response_type = command_generic_response;
	static constexpr char type[] = "admittance.apply_adm_imp_desired_stiffness";

	double duration;
	double adm_rotational_stiffness;
	double adm_translational_stiffness;
	double imp_rotational_stiffness;
	double imp_translational_stiffness;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_apply_admittance_adm_imp_desired_stiffness& object);
void from_json(const nlohmann::json& json, command_apply_admittance_adm_imp_desired_stiffness& object);


/**
 *************************************************************************
 *
 * @class command_cartesian_impedance_hold_pose_desired_stiffness
 *
 * Commands the robot to use the cartesian impedance controller to hold the 
 * current pose for a given duration using desired rotational and 
 * translational stiffness
 *
 ************************************************************************/
struct command_cartesian_impedance_hold_pose_desired_stiffness
{
	using response_type = command_generic_response;
	static constexpr char type[] = "cartesian_impedance.hold_desired_stiffness";

	double duration;
	bool use_stiff_damp_online_calc;
	double rotational_stiffness;
	double translational_stiffness;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_cartesian_impedance_hold_pose_desired_stiffness& object);
void from_json(const nlohmann::json& json, command_cartesian_impedance_hold_pose_desired_stiffness& object);


/**
 *************************************************************************
 *
 * @class command_cartesian_impedance_poses_desired_stiffness
 *
 * Commands the robot to use the cartesian impedance controller to follow 
 * a path of given poses for a given duration using desired rotational and
 * translational stiffness
 *
 ************************************************************************/
struct command_cartesian_impedance_poses_desired_stiffness
{
	using response_type = command_generic_response;
	static constexpr char type[] = "cartesian_impedance.poses_desired_stiffness";

	std::list<std::array<double, 16>> poses;
	double duration;
	bool use_stiff_damp_online_calc;
	double rotational_stiffness;
	double translational_stiffness;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_cartesian_impedance_poses_desired_stiffness& object);
void from_json(const nlohmann::json& json, command_cartesian_impedance_poses_desired_stiffness& object);


/**
 *************************************************************************
 *
 * @class command_joint_impedance_hold_position_desired_stiffness
 *
 * Commands the robot to use the joint space impedance controller to hold 
 * the current position for a given duration using desired stiffness
 * matrix parameter
 *
 ************************************************************************/
struct command_joint_impedance_hold_position_desired_stiffness
{
	using response_type = command_generic_response;
	static constexpr char type[] = "joint_impedance.hold_desired_stiffness";

	double duration;
	std::array<double, 49> stiffness;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_joint_impedance_hold_position_desired_stiffness& object);
void from_json(const nlohmann::json& json, command_joint_impedance_hold_position_desired_stiffness& object);


/**
 *************************************************************************
 *
 * @class command_joint_impedance_positions_desired_stiffness
 *
 * Commands the robot to use the joint impedance controller to follow a
 * path of given positions for a given duration using desired stiffness
 * matrix parameter
 *
 ************************************************************************/
struct command_joint_impedance_positions_desired_stiffness
{
	using response_type = command_generic_response;
	static constexpr char type[] = "joint_impedance.positions_desired_stiffness";

	std::list<std::array<double, 7>> joint_positions;
	double duration;
	std::array<double, 49> stiffness;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_joint_impedance_positions_desired_stiffness& object);
void from_json(const nlohmann::json& json, command_joint_impedance_positions_desired_stiffness& object);


/**
 *************************************************************************
 *
 * @class command_ple_motion
 *
 * Commands the robot to execute the pre-defined motion for
 * payload estimation
 *
 ************************************************************************/
struct command_ple_motion
{
	using response_type = command_generic_response;
	static constexpr char type[] = "ple_motion";

	double speed;
	double duration;
	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json&, const command_ple_motion& object);
void from_json(const nlohmann::json& json, command_ple_motion& object);


/**
 *************************************************************************
 *
 * @class command_force_z
 *
 * Commands the robot to apply a force of given `mass` for `duration`
 * seconds along the end effectors z-axis. 
 *
 ************************************************************************/
struct command_force_z
{
	using response_type = command_generic_response;
	static constexpr char type[] = "force.z";

	double mass;
	double duration;
};

void to_json(nlohmann::json&, const command_force_z& object);
void from_json(const nlohmann::json& json, command_force_z& object);


/**
 *************************************************************************
 *
 * @class command_open_gripper
 *
 * Commands the robot to open its gripper.
 *
 ************************************************************************/
struct command_open_gripper
{
	using response_type = command_generic_response;
	static constexpr char type[] = "gripper.open";

	double speed;
};

void to_json(nlohmann::json& json, const command_open_gripper& object);
void from_json(const nlohmann::json& json, command_open_gripper& object);


/**
 *************************************************************************
 *
 * @class command_close_gripper
 *
 * Commands the robot to close its gripper.
 *
 ************************************************************************/
struct command_close_gripper
{
	using response_type = command_generic_response;
	static constexpr char type[] = "gripper.close";

	double speed;
};

void to_json(nlohmann::json& json, const command_close_gripper& object);
void from_json(const nlohmann::json& json, command_close_gripper& object);


/**
 *************************************************************************
 *
 * @class command_grasp_gripper
 *
 * Commands the robot to grasp using its gripper with given speed and force.
 *
 ************************************************************************/
struct command_grasp_gripper
{
	using response_type = command_generic_response;
	static constexpr char type[] = "gripper.grasp";

	double speed;
	double force;
};

void to_json(nlohmann::json& json, const command_grasp_gripper& object);
void from_json(const nlohmann::json& json, command_grasp_gripper& object);


/**
 *************************************************************************
 *
 * @class command_vacuum_gripper_drop
 *
 * Commands the robot to stop vacuum with the vacuum_gripper.
 *
 ************************************************************************/
struct command_vacuum_gripper_drop
{
	using response_type = command_generic_response;
	static constexpr char type[] = "vacuum_gripper.drop";
	std::chrono::milliseconds timeout;
};

void to_json(nlohmann::json& json, const command_vacuum_gripper_drop& object);
void from_json(const nlohmann::json& json, command_vacuum_gripper_drop& object);


/**
 *************************************************************************
 *
 * @class command_vacuum_gripper_vacuum
 *
 * Commands the robot to create a vacuum using the vacuum gripper.
 *
 ************************************************************************/
struct command_vacuum_gripper_vacuum
{
	using response_type = command_generic_response;
	static constexpr char type[] = "vacuum_gripper.vacuum";
	std::uint8_t vacuum_strength;
	std::chrono::milliseconds timeout;
};

void to_json(nlohmann::json& json, const command_vacuum_gripper_vacuum& object);
void from_json(const nlohmann::json& json, command_vacuum_gripper_vacuum& object);


/**
 *************************************************************************
 *
 * @class command_grasp_gripper
 *
 * Commands the robot to stop the current vacuum gripper command
 *
 ************************************************************************/
struct command_vacuum_gripper_stop
{
	using response_type = command_generic_response;
	static constexpr char type[] = "vacuum_gripper.stop";
};

void to_json(nlohmann::json& json, const command_vacuum_gripper_stop& object);
void from_json(const nlohmann::json& json, command_vacuum_gripper_stop& object);


/**
 *************************************************************************
 *
 * @class command_start_recording
 *
 * Commands the server to start recording the robot's state.
 *
 ************************************************************************/
struct command_start_recording
{
	using response_type = command_generic_response;
	static constexpr char type[] = "recording.start";

	std::optional<std::string> log_file_path;
};

void to_json(nlohmann::json& json, const command_start_recording& object);
void from_json(const nlohmann::json& json, command_start_recording& object);


/**
 *************************************************************************
 *
 * @class command_stop_recording
 *
 * Commands the server to stop recording the robot's state and to 
 * reply with the recorded data.
 *
 ************************************************************************/
struct command_stop_recording
{
	using response_type = command_stop_recording_response;
	static constexpr char type[] = "recording.stop";
};

void to_json(nlohmann::json& json, const command_stop_recording& object);
void from_json(const nlohmann::json& json, command_stop_recording& object);


/**
 *************************************************************************
 *
 * @class command_stop_recording_response
 *
 * Commands the server to stop recording the robot's state and to
 * reply with the recorded data.
 *
 ************************************************************************/
struct command_stop_recording_response
{
	static constexpr char type[] = "response.stop-recording";

	std::vector<std::array<double, 7>> joint_config_sequence;
	std::vector<std::array<double, 6>> force_sequence;
};

void to_json(nlohmann::json& json, const command_stop_recording_response& object);
void from_json(const nlohmann::json& json, command_stop_recording_response& object);


/**
 *************************************************************************
 *
 * @class command_set_speed
 *
 * Sets the speed factor at which the robot moves.
 *
 ************************************************************************/
struct command_set_speed
{
	using response_type = command_generic_response;
	static constexpr char type[] = "set.speed";

	double speed;
};

void to_json(nlohmann::json& json, const command_set_speed& object);
void from_json(const nlohmann::json& json, command_set_speed& object);

/**
 *************************************************************************
 *
 * @class command_set_fts_bias
 *
 * Sets the bias of the force/torque sensor
 *
 ************************************************************************/
struct command_set_fts_bias
{
	using response_type = command_generic_response;
	static constexpr char type[] = "set.fts_bias";

	std::array<double, 6> bias;
};

void to_json(nlohmann::json& json, const command_set_fts_bias& object);
void from_json(const nlohmann::json& json, command_set_fts_bias& object);

/**
 *************************************************************************
 *
 * @class command_set_fts_bias
 *
 * Sets the load_mass of the force/torque sensor
 *
 ************************************************************************/
struct command_set_fts_load_mass
{
	using response_type = command_generic_response;
	static constexpr char type[] = "set.fts_load_mass";

	std::array<double, 3> load_mass;
};

void to_json(nlohmann::json& json, const command_set_fts_load_mass& object);
void from_json(const nlohmann::json& json, command_set_fts_load_mass& object);


/**
 *************************************************************************
 *
 * @class command_set_guiding_params
 *
 * Sets the guiding vector selected in the application by the user.
 *
 ************************************************************************/
struct command_set_guiding_params
{
	using response_type = command_generic_response;
	static constexpr char type[] = "set.guiding";

	std::array<bool, 6> guiding_config;
	bool elbow;
};

void to_json(nlohmann::json& json, const command_set_guiding_params& objekt);
void from_json(const nlohmann::json& json, command_set_guiding_params& object);


/**
 *************************************************************************
 *
 * @class command_recover_from_errors
 *
 * Commands the server to try to recover from errors.
 *
 ************************************************************************/
struct command_recover_from_errors
{
	using response_type = command_generic_response;
	static constexpr char type[] = "error.recover";
};

void to_json(nlohmann::json& json, const command_recover_from_errors& object);
void from_json(const nlohmann::json& json, command_recover_from_errors& object);


/**
 *************************************************************************
 *
 * @enum command_result
 *
 * Result code sent in `command_generic_response`.
 *
 ************************************************************************/
enum class command_result
	: std::uint8_t
{
	success,
	success_command_failed,
	model_exception,
	network_exception,
	protocol_exception,
	incompatible_version,
	control_exception,
	command_exception,
	realtime_exception,
	invalid_operation,
	franka_exception,
	force_torque_sensor_exception,
	unknown_command
};


/**
 *************************************************************************
 *
 * @class command_generic_response
 *
 * Response sent by the server after processing a command.
 *
 ************************************************************************/
struct command_generic_response
{
	static constexpr char type[] = "response.generic";
	command_result result;
	std::string reason;


	command_generic_response() noexcept
		: result{command_result::success}
	{
	}


	command_generic_response(command_result result) noexcept
		: result{result}
	{
	}


	command_generic_response(command_result result, std::string reason)
		: result{result}
		  , reason{std::move(reason)}
	{
	}
};


void to_json(nlohmann::json& json, const command_generic_response& object);
void from_json(const nlohmann::json& json, command_generic_response& object);


/**
 *************************************************************************
 *
 * @class command_get_config
 *
 * Requests the current state of robot.
 *
 * Note: Currently not used.
 *
 ************************************************************************/
struct command_get_config
{
	using response_type = command_get_config_response;
	static constexpr char type[] = "get.config";
};

void to_json(nlohmann::json& json, const command_get_config& object);
void from_json(const nlohmann::json& json, command_get_config& object);


/**
 *************************************************************************
 *
 * @class command_get_config_response
 *
 * Response send by the server after requesting the current configuration.
 *
 ************************************************************************/
struct command_get_config_response
{
	static constexpr char type[] = "response.config";

	std::array<double, 7> joint_configuration;
	std::array<double, 6> end_effector_wrench;

	//jaw gripper
	double width;
	double max_width;
	bool is_grasped;

	//vacuum gripper
	uint16_t actual_power;
	uint16_t vacuum;
	bool part_detached;
	bool part_present;
	bool in_control_range;
};

void to_json(nlohmann::json& json, const command_get_config_response& object);
void from_json(const nlohmann::json& json, command_get_config_response& object);
} /* namespace franka_proxy */

#endif // INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_COMMANDS_HPP
