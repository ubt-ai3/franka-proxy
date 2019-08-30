/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 *
 * ...
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP


#include <memory>

#include <viral_core/network.hpp>

#include "franka_network_server.hpp"
#include "franka_hardware_controller.hpp"


namespace franka_proxy
{


class franka_proxy
{

public:

	franka_proxy();


private:

	franka_hardware_controller controller_;

	std::unique_ptr<viral_core::network_context> network_;
	franka_control_server control_server_;
	franka_state_server state_server_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP) */