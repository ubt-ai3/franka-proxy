/**
 *************************************************************************
 *
 * @file franka_proxy_commands.cpp
 *
 * Commands/Responses that are sent to/received by the proxy server,
 * implementation.
 *
 ************************************************************************/


#include "franka_proxy_commands.hpp"

#include <nlohmann/json.hpp>


namespace franka_proxy
{
//////////////////////////////////////////////////////////////////////////
//
// command_move_to_config
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_move_to_config& object)
{
	json["type"] = command_move_to_config::type;
	json["target"] = object.target_joint_config;
}


void from_json(const nlohmann::json& json, command_move_to_config& object)
{
	json.at("target").get_to(object.target_joint_config);
}


//////////////////////////////////////////////////////////////////////////
//
// command_move_until_contact
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_move_until_contact& object)
{
	json["type"] = command_move_until_contact::type;
	json["target"] = object.target_joint_config;
}


void from_json(const nlohmann::json& json, command_move_until_contact& object)
{
	json.at("target").get_to(object.target_joint_config);
}


//////////////////////////////////////////////////////////////////////////
//
// command_move_hybrid_sequence
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_move_hybrid_sequence& object)
{
	json["type"] = command_move_hybrid_sequence::type;
	json["joint_config_sequence"] = object.joint_config_sequence;
	json["force_sequence"] = object.force_sequence;
	json["selection_sequence"] = object.selection_sequence;
	json["offset_position"] = object.offset_position;
	json["offset_force"] = object.offset_force;
}


void from_json(const nlohmann::json& json, command_move_hybrid_sequence& object)
{
	json.at("joint_config_sequence").get_to(object.joint_config_sequence);
	json.at("force_sequence").get_to(object.force_sequence);
	json.at("selection_sequence").get_to(object.selection_sequence);
	json.at("offset_position").get_to(object.offset_position);
	json.at("offset_force").get_to(object.offset_force);

}


//////////////////////////////////////////////////////////////////////////
//
// command_apply_admittance_adm_imp_desired_stiffness
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_apply_admittance_adm_imp_desired_stiffness& object)
{
	json["type"] = command_apply_admittance_adm_imp_desired_stiffness::type;
	json["duration"] = object.duration;
	json["adm_rotational_stiffness"] = object.adm_rotational_stiffness;
	json["adm_translational_stiffness"] = object.adm_translational_stiffness;
	json["imp_rotational_stiffness"] = object.imp_rotational_stiffness;
	json["imp_translational_stiffness"] = object.imp_translational_stiffness;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_apply_admittance_adm_imp_desired_stiffness& object)
{
	json.at("duration").get_to(object.duration);
	json.at("adm_rotational_stiffness").get_to(object.adm_rotational_stiffness);
	json.at("adm_translational_stiffness").get_to(object.adm_translational_stiffness);
	json.at("imp_rotational_stiffness").get_to(object.imp_rotational_stiffness);
	json.at("imp_translational_stiffness").get_to(object.imp_translational_stiffness);
	
	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_cartesian_impedance_hold_pose_desired_stiffness
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_cartesian_impedance_hold_pose_desired_stiffness& object)
{
	json["type"] = command_cartesian_impedance_hold_pose_desired_stiffness::type;
	json["duration"] = object.duration;
	json["use_stiff_damp_online_calc"] = object.use_stiff_damp_online_calc;
	json["rotational_stiffness"] = object.rotational_stiffness;
	json["translational_stiffness"] = object.translational_stiffness;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_cartesian_impedance_hold_pose_desired_stiffness& object)
{
	json.at("duration").get_to(object.duration);
	json.at("use_stiff_damp_online_calc").get_to(object.use_stiff_damp_online_calc);
	json.at("rotational_stiffness").get_to(object.rotational_stiffness);
	json.at("translational_stiffness").get_to(object.translational_stiffness);

	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_cartesian_impedance_poses_desired_stiffness
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_cartesian_impedance_poses_desired_stiffness& object)
{
	json["type"] = command_cartesian_impedance_poses_desired_stiffness::type;
	json["poses"] = object.poses;
	json["duration"] = object.duration;
	json["use_stiff_damp_online_calc"] = object.use_stiff_damp_online_calc;
	json["rotational_stiffness"] = object.rotational_stiffness;
	json["translational_stiffness"] = object.translational_stiffness;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_cartesian_impedance_poses_desired_stiffness& object)
{
	json.at("poses").get_to(object.poses);
	json.at("duration").get_to(object.duration);
	json.at("use_stiff_damp_online_calc").get_to(object.use_stiff_damp_online_calc);
	json.at("rotational_stiffness").get_to(object.rotational_stiffness);
	json.at("translational_stiffness").get_to(object.translational_stiffness);

	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_joint_impedance_hold_position_desired_stiffness
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_joint_impedance_hold_position_desired_stiffness& object)
{
	json["type"] = command_joint_impedance_hold_position_desired_stiffness::type;
	json["duration"] = object.duration;
	json["stiffness"] = object.stiffness;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_joint_impedance_hold_position_desired_stiffness& object)
{
	json.at("duration").get_to(object.duration);
	json.at("stiffness").get_to(object.stiffness);

	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_joint_impedance_positions_desired_stiffness
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_joint_impedance_positions_desired_stiffness& object)
{
	json["type"] = command_joint_impedance_positions_desired_stiffness::type;
	json["joint_positions"] = object.joint_positions;
	json["duration"] = object.duration;
	json["stiffness"] = object.stiffness;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_joint_impedance_positions_desired_stiffness& object)
{
	json.at("joint_positions").get_to(object.joint_positions);
	json.at("duration").get_to(object.duration);
	json.at("stiffness").get_to(object.stiffness);

	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_ple_motion
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_ple_motion& object)
{
	json["type"] = command_ple_motion::type;
	json["speed"] = object.speed;
	json["duration"] = object.duration;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_ple_motion& object)
{
	json.at("speed").get_to(object.speed);
	json.at("duration").get_to(object.duration);
	
	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_force_z
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_force_z& object)
{
	json["type"] = command_force_z::type;
	json["mass"] = object.mass;
	json["duration"] = object.duration;
}


void from_json(const nlohmann::json& json, command_force_z& object)
{
	json.at("mass").get_to(object.mass);
	json.at("duration").get_to(object.duration);
}


//////////////////////////////////////////////////////////////////////////
//
// command_open_gripper
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_open_gripper& object)
{
	json["type"] = command_open_gripper::type;
	json["speed"] = object.speed;
}


void from_json(const nlohmann::json& json, command_open_gripper& object)
{
	json.at("speed").get_to(object.speed);
}


//////////////////////////////////////////////////////////////////////////
//
// command_close_gripper
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_close_gripper& object)
{
	json["type"] = command_close_gripper::type;
	json["speed"] = object.speed;
}


void from_json(const nlohmann::json& json, command_close_gripper& object)
{
	json.at("speed").get_to(object.speed);
}


//////////////////////////////////////////////////////////////////////////
//
// command_close_gripper
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_grasp_gripper& object)
{
	json["type"] = command_grasp_gripper::type;
	json["speed"] = object.speed;
	json["force"] = object.force;
}


void from_json(const nlohmann::json& json, command_grasp_gripper& object)
{
	json.at("speed").get_to(object.speed);
	json.at("force").get_to(object.force);
}

void to_json(nlohmann::json& json, const command_vacuum_gripper_drop& object)
{
	json["type"] = command_vacuum_gripper_drop::type;
	json["timeout"] = object.timeout.count();
}

void from_json(const nlohmann::json& json, command_vacuum_gripper_drop& object)
{
	long long tick_count{};
	json.at("timeout").get_to(tick_count);
	object.timeout = std::chrono::milliseconds(tick_count);
}

void to_json(nlohmann::json& json, const command_vacuum_gripper_vacuum& object)
{
	json["type"] = command_vacuum_gripper_vacuum::type;
	json["timeout"] = object.timeout.count();
	json["vacuum"] = object.vacuum_strength;
}

void from_json(const nlohmann::json& json, command_vacuum_gripper_vacuum& object)
{
	long long tick_count{};
	json.at("timeout").get_to(tick_count);
	json.at("vacuum").get_to(object.vacuum_strength);
	object.timeout = std::chrono::milliseconds(tick_count);
}

void to_json(nlohmann::json& json, const command_vacuum_gripper_stop& object)
{
	json["type"] = command_vacuum_gripper_stop::type;
}

void from_json(const nlohmann::json& json, command_vacuum_gripper_stop& object)
{
	//do nothing
}


//////////////////////////////////////////////////////////////////////////
//
// command_start_recording
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_start_recording& object)
{
	json["type"] = command_start_recording::type;
	json["log_file_path"] = object.log_file_path.value_or("");
}


void from_json(const nlohmann::json& json, command_start_recording& object)
{
	std::string lfp = json.at("log_file_path");
	bool log = !lfp.empty();
	if (log)
		object.log_file_path = std::optional<std::string>(lfp);
	else
		object.log_file_path = std::optional<std::string>(std::nullopt);
}


//////////////////////////////////////////////////////////////////////////
//
// command_stop_recording
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_stop_recording& object)
{
	json["type"] = command_stop_recording::type;
}


void from_json(const nlohmann::json& json, command_stop_recording& object)
{
}


//////////////////////////////////////////////////////////////////////////
//
// command_stop_recording_response
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_stop_recording_response& object)
{
	json["type"] = command_stop_recording_response::type;
	json["joint_config_sequence"] = object.joint_config_sequence;
	json["force_sequence"] = object.force_sequence;
}

void from_json(const nlohmann::json& json, command_stop_recording_response& object)
{
	json.at("joint_config_sequence").get_to(object.joint_config_sequence);
	json.at("force_sequence").get_to(object.force_sequence);
}


//////////////////////////////////////////////////////////////////////////
//
// command_set_speed
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_set_speed& object)
{
	json["type"] = command_set_speed::type;
	json["speed"] = object.speed;
}


void from_json(const nlohmann::json& json, command_set_speed& object)
{
	json.at("speed").get_to(object.speed);
}


//////////////////////////////////////////////////////////////////////////
//
// command_set_fts_bias
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_set_fts_bias& object)
{
	json["type"] = command_set_fts_bias::type;
	json["bias"] = nlohmann::json::array();

	for (double bias : object.bias)
		json["bias"].push_back(bias);
}


void from_json(const nlohmann::json& json, command_set_fts_bias& object)
{
	object.bias = std::array<double, 6>();
	for (int i = 0; i < object.bias.size(); i++)
		object.bias[i] = json.at("bias").at(i);
}

//////////////////////////////////////////////////////////////////////////
//
// command_set_fts_load_mass
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_set_fts_load_mass& object)
{
	json["type"] = command_set_fts_load_mass::type;
	json["load_mass"] = nlohmann::json::array();

	for (double load_mass : object.load_mass)
		json["load_mass"].push_back(load_mass);
}


void from_json(const nlohmann::json& json, command_set_fts_load_mass& object)
{
	object.load_mass = std::array<double, 3>();
	for (int i = 0; i < object.load_mass.size(); i++)
		object.load_mass[i] = json.at("load_mass").at(i);
}

//////////////////////////////////////////////////////////////////////////
//
// command_set_guiding_params
//
//////////////////////////////////////////////////////////////////////////

void to_json(nlohmann::json& json, const command_set_guiding_params& object)
{
	json["type"] = command_set_guiding_params::type;
	json["guiding_config"] = object.guiding_config;
	json["elbow"] = object.elbow;
}
void from_json(const nlohmann::json& json, command_set_guiding_params& object)
{
	json.at("guiding_config").get_to(object.guiding_config);
	json.at("elbow").get_to(object.elbow);
}


//////////////////////////////////////////////////////////////////////////
//
// command_recover_from_errors
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_recover_from_errors& object)
{
	json["type"] = command_recover_from_errors::type;
}


void from_json(const nlohmann::json& json, command_recover_from_errors& object)
{
}


//////////////////////////////////////////////////////////////////////////
//
// command_generic_response
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_generic_response& object)
{
	json["type"] = command_generic_response::type;
	json["result"] = object.result;
	json["reason"] = object.reason;
}


void from_json(const nlohmann::json& json, command_generic_response& object)
{
	json.at("result").get_to(object.result);
	json.at("reason").get_to(object.reason);
}


//////////////////////////////////////////////////////////////////////////
//
// command_get_config
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_get_config& object)
{
	json["type"] = command_get_config::type;
}


void from_json(const nlohmann::json& json, command_get_config& object)
{
}


//////////////////////////////////////////////////////////////////////////
//
// command_get_config_response
//
//////////////////////////////////////////////////////////////////////////


void to_json(nlohmann::json& json, const command_get_config_response& object)
{
	json["type"] = command_get_config_response::type;
	json["joint_configuration"] = object.joint_configuration;
	json["width"] = object.width;
	json["max_width"] = object.max_width;
	json["is_grasped"] = object.is_grasped;

	json["actual_power"] = object.actual_power;
	json["vacuum_level"] = object.vacuum;
	json["in_control_range"] = object.in_control_range;
	json["part_detached"] = object.part_detached;
	json["part_present"] = object.part_present;

}


void from_json(const nlohmann::json& json, command_get_config_response& object)
{
	json.at("joint_configuration").get_to(object.joint_configuration);
	json.at("width").get_to(object.width);
	json.at("max_width").get_to(object.max_width);
	json.at("is_grasped").get_to(object.is_grasped);

	json.at("actual_power").get_to(object.actual_power);
	json.at("vacuum_level").get_to(object.vacuum);
	json.at("in_control_range").get_to(object.in_control_range);
	json.at("part_detached").get_to(object.part_detached);
	json.at("part_present").get_to(object.part_present);
}
} /* namespace franka_proxy */
