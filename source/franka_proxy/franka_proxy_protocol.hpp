#ifndef FRANKA_PROXY_PROTOCOL_HPP
#define FRANKA_PROXY_PROTOCOL_HPP

#include <cstdint>

namespace franka_proxy
{
#pragma pack(0)
    
    template<std::uint8_t packet_id>
    struct packet_base {
        static constexpr std::uint8_t id = packet_id;
    };

    /**
     * @brief Instructs the robot to open its gripper.
     */
    struct open_gripper: packet_base<1> {};

    /**
     * @brief Instructs the robot to close its gripper.  
     */
    struct close_gripper: packet_base<2> {};

}

#endif  // FRANKA_PROXY_PROTOCOL_HPP