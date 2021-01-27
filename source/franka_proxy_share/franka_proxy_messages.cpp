#include <franka_proxy_share/franka_proxy_messages.hpp>

#include <nlohmann/json.hpp>


namespace franka_proxy 
{ 

// ---------------------------- MESSAGE MOVE PTP -------------------------------

void to_json(nlohmann::json& j, const message_move_ptp& msg) 
{
    j["type"] = message_move_ptp::type;
    j["config"] = msg.config;
}

void from_json(const nlohmann::json& j, message_move_ptp& msg) 
{
    msg.config = j["config"].get<decltype(msg.config)>(); 
}

// -------------------------- MESSAGE MOVE CONTACT -----------------------------

void to_json(nlohmann::json& j, const message_move_contact& msg) 
{
    j["type"] = message_move_contact::type;
    j["config"] = msg.config;
}

void from_json(const nlohmann::json& j, message_move_contact& msg) 
{
    msg.config = j["config"].get<decltype(msg.config)>();
}

// ------------------------- MESSAGE MOVE SEQUENCE -----------------------------

void to_json(nlohmann::json& j, const message_move_hybrid_sequence& msg) 
{
    j["type"] = message_move_hybrid_sequence::type;
    j["q_data"] = msg.q_data;
    j["f_data"] = msg.f_data;
    j["s_data"] = msg.s_data;
}

void from_json(const nlohmann::json& j, message_move_hybrid_sequence& msg) 
{
    j.at("q_data").get_to(msg.q_data);
    j.at("f_data").get_to(msg.f_data);
    j.at("s_data").get_to(msg.s_data);
}

// ---------------------------- MESSAGE FORCE Z --------------------------------

void to_json(nlohmann::json& j, const message_force_z& msg)
{
    j["type"] = message_force_z::type;
    j["mass"] = msg.mass;
    j["duration"] = msg.duration;
}

void from_json(const nlohmann::json& j, message_force_z& msg)
{
    msg.mass = j["mass"].get<decltype(msg.mass)>();
    msg.duration = j["duration"].get<decltype(msg.duration)>();
}

// -------------------------- MESSAGE OPEN GRIPPER -----------------------------

void to_json(nlohmann::json& j, const message_open_gripper& msg)
{
    j["type"] = message_open_gripper::type;
}

void from_json(const nlohmann::json& j, message_open_gripper& msg)
{}

// -------------------------- MESSAGE CLOSE GRIPPER ----------------------------

void to_json(nlohmann::json& j, const message_close_gripper& msg)
{
    j["type"] = message_close_gripper::type;
}

void from_json(const nlohmann::json& j, message_close_gripper& msg)
{}

// ------------------------- MESSAGE GRASP GRIPPER -----------------------------

void to_json(nlohmann::json& j, const message_grasping_gripper& msg)
{
    j["type"] = message_grasping_gripper::type;
    j["speed"] = msg.speed;
    j["force"] = msg.force;
}

void from_json(const nlohmann::json& j, message_grasping_gripper& msg) 
{
    msg.speed = j["speed"].get<decltype(msg.speed)>();
    msg.force = j["force"].get<decltype(msg.force)>();
}

// ------------------------ MESSAGE START RECORDING ----------------------------

void to_json(nlohmann::json& j, const message_start_recording& msg)
{
    j["type"] = message_start_recording::type;
}

void from_json(const nlohmann::json& j, message_start_recording& msg)
{}

// ------------------------- MESSAGE STOP RECORDING ----------------------------

void to_json(nlohmann::json& j, const message_stop_recording& msg)
{
    j["type"] = message_stop_recording::type;
}

void from_json(const nlohmann::json& j, message_stop_recording& msg)
{}

// ----------------------------- MESSAGE SPEED ---------------------------------

void to_json(nlohmann::json& j, const message_speed& msg)
{
    j["type"] = message_speed::type;
    j["speed"] = msg.speed;
}

void from_json(const nlohmann::json& j, message_speed& msg)
{
    msg.speed = j["speed"].get<decltype(msg.speed)>();
}

// ------------------------- MESSAGE ERROR RECOVERY ----------------------------

void to_json(nlohmann::json& j, const message_error_recovery& msg)
{
    j["type"] = message_error_recovery::type;
}

void from_json(const nlohmann::json& j, message_error_recovery& msg)
{
}

}
