#include "franka_proxy_commands.hpp"

#include <nlohmann/json.hpp>

namespace franka_proxy {


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
	json["type"] = command_move_to_config::type;
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
}
	

void from_json(const nlohmann::json& json, command_move_hybrid_sequence& object)
{
	json.at("joint_config_sequence").get_to(object.joint_config_sequence);
	json.at("force_sequence").get_to(object.force_sequence);
	json.at("selection_sequence").get_to(object.selection_sequence);
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


//////////////////////////////////////////////////////////////////////////
//
// command_start_recording
//
//////////////////////////////////////////////////////////////////////////

	
void to_json(nlohmann::json& json, const command_start_recording& object)
{
	json["type"] = command_start_recording::type;
}


void from_json(const nlohmann::json& json, command_start_recording& object)
{
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
}

	
void from_json(const nlohmann::json& json, command_get_config_response& object)
{
	json.at("joint_configuration").get_to(object.joint_configuration);
	json.at("width").get_to(object.width);
	json.at("max_width").get_to(object.max_width);
	json.at("is_grasped").get_to(object.is_grasped);
}


} /* namespace franka_proxy */