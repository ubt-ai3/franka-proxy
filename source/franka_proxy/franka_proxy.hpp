/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP


#include <memory>

#include "franka_hardware_controller.hpp"
#include "franka_network_server.hpp"


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

	franka_proxy();


private:

	franka_hardware_controller controller_;

	franka_control_server control_server_;
	franka_state_server state_server_;

    static constexpr unsigned short franka_control_port = 4711;
    static constexpr unsigned short franka_state_port = 4712;
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP) */
