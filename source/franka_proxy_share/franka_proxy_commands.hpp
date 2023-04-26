/**
 *************************************************************************
 *
 * @file franka_proxy_commands.hpp
 *
 * Commands/Responses that are sent to/received by the proxy server.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_COMMANDS_HPP)
#define INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_COMMANDS_HPP


#include <array>
#include <cstdint>
#include <list>
#include <vector>
#include <string>

#include <nlohmann/json_fwd.hpp>


namespace franka_proxy
{


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

	std::array<double, 7>	target_joint_config;
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
	static constexpr char	type[] = "move.contact";

	std::array<double, 7>	target_joint_config;
};

void to_json(nlohmann::json& json, const command_move_until_contact& object);
void from_json(const nlohmann::json& json, command_move_until_contact& object);




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
	static constexpr char	type[] = "move.hybrid-sequence";

	std::vector<std::array<double, 7>>	joint_config_sequence;
	std::vector<std::array<double, 6>>	force_sequence;
	std::vector<std::array<double, 6>>	selection_sequence;
};

void to_json(nlohmann::json& json, const command_move_hybrid_sequence& object);
void from_json(const nlohmann::json& json, command_move_hybrid_sequence& object);




/**
 *************************************************************************
 *
 * @class command_apply_force
 *
 * Commands the robot to use the admittance controller 
 *
 ************************************************************************/
struct command_apply_admittance
{
	using response_type = command_generic_response;
	static constexpr char type[] = "admittance.apply";

	double duration;
};

void to_json(nlohmann::json&, const command_apply_admittance& object);
void from_json(const nlohmann::json& json, command_apply_admittance& object);




/**
 *************************************************************************
 *
 * @class command_impedance_hold_pose
 *
 * Commands the robot to use the impedance controller to hold the current
 * pose for a given duration
 *
 ************************************************************************/
struct command_impedance_hold_pose
{
	using response_type = command_generic_response;
	static constexpr char type[] = "impedance.hold";

	double duration;
};

void to_json(nlohmann::json&, const command_impedance_hold_pose& object);
void from_json(const nlohmann::json& json, command_impedance_hold_pose& object);




/**
 *************************************************************************
 *
 * @class command_impedance_follow_poses
 *
 * Commands the robot to use the impedance controller to follow a path of
 * given poses for a given duration
 *
 ************************************************************************/
struct command_impedance_follow_poses
{
	using response_type = command_generic_response;
	static constexpr char type[] = "impedance.follow";

	std::list<std::array<double, 16>> poses;
	double duration;
};

void to_json(nlohmann::json&, const command_impedance_follow_poses& object);
void from_json(const nlohmann::json& json, command_impedance_follow_poses& object);




/**
 *************************************************************************
 *
 * @class command_force_z
 *
 * Commands the robot to apply a force of given `mass` for `duration`
 * seconds along the endeffectors Z-axis. 
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
 * @class command_start_recording
 *
 * Commands the server to start recording the robot's state.
 *
 ************************************************************************/
struct command_start_recording
{
	using response_type = command_generic_response;
	static constexpr char type[] = "recording.start";
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
enum class command_result: std::uint8_t
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
	, reason{}
	{}

	command_generic_response(command_result result) noexcept
	: result{result}
	, reason{}
	{}

	command_generic_response(command_result result, std::string reason)
	: result{result}
	, reason{std::move(reason)}
	{}
	
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
	double width;
	double max_width;
	bool is_grasped;
};

void to_json(nlohmann::json& json, const command_get_config_response& object);
void from_json(const nlohmann::json& json, command_get_config_response& object);




}


#endif	// INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP