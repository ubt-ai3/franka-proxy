#ifndef INCLUDED__FRANKA_PROXY_CLIENT__FORWARD_HPP
#define INCLUDED__FRANKA_PROXY_CLIENT__FORWARD_HPP
/**
 *************************************************************************
 *
 * @file forward.hpp
 *
 * Forward declarations for all franka_proxy_client types.
 *
 ************************************************************************/



#include <array>


namespace franka_proxy
{
using robot_config_7dof = std::array<double, 7>;


class franka_state_client;
class franka_control_client;

class franka_remote_controller;
}

#endif // INCLUDED__FRANKA_PROXY_CLIENT__FORWARD_HPP
