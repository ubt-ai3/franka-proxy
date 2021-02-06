#if !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP)
#define INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP

/**
 * @file franka_proxy_messages.hpp
 * 
 * @brief Defines messages exchanged between a client and the proxy.
 * @details Messages are defined as struct, which are de-/serialized from/to
 *          JSON using the `from_json` and `to_json`. These MUST be implemented
 *          when adding new messages.
 */


#include <array>

#include <nlohmann/json_fwd.hpp>

namespace franka_proxy
{

struct message_robot_state
{
    static constexpr char type[] = "robot-state";

    std::array<double, 7>   q;
    double                  width;
    double                  max_width;
    bool                    is_grasped;

};


void to_json(nlohmann::json& j, const message_robot_state& msg);
void from_json(const nlohmann::json& j, message_robot_state& msg);


} /* namespace franka_proxy */



#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP) */
