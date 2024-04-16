/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/

#pragma once

#include "franka_hardware_controller.hpp"
#include "franka_network_control_server.hpp"
#include "franka_network_state_server.hpp"


namespace franka_proxy
{
/**
 *************************************************************************
 *
 * @class franka_proxy
 *
 * todo
 *
 ************************************************************************/

class franka_proxy
{
public:
	franka_proxy(const std::string& ip, bool enforce_realtime);

private:
	franka_hardware_controller controller_;

	franka_control_server control_server_;
	franka_state_server state_server_;

	static constexpr unsigned short franka_control_port = 4711;
	static constexpr unsigned short franka_state_port = 4712;
};
} /* namespace franka_proxy */
