#include <franka_proxy_share/franka_proxy_messages.hpp>

#include <nlohmann/json.hpp>

namespace franka_proxy 
{ 

// -------------------------- MESSAGE ROBOT STATE ------------------------------

void to_json(nlohmann::json& j, const message_robot_state& msg) 
{
    j["type"] = message_robot_state::type;
    j["q"] = msg.q;
    j["width"] = msg.width;
    j["max_width"] = msg.max_width;
    j["is_grasped"] = msg.is_grasped;
}

void from_json(const nlohmann::json& j, message_robot_state& msg)
{
    j.at("q").get_to(msg.q);
    j.at("width").get_to(msg.width);
    j.at("max_width").get_to(msg.max_width);
    j.at("is_grasped").get_to(msg.is_grasped);
}

}
