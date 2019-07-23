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
#include <mutex>

#include <franka/robot_state.h>
#include <franka/gripper_state.h>

#include "franka_network_server.hpp"
#include "franka_hardware_controller.hpp"


namespace franka_proxy
{


class franka_proxy
{

public:

	franka_proxy();


private:

	std::mutex state_lock_;
	franka::RobotState robot_state_;
	franka::GripperState gripper_state_;

	std::unique_ptr<franka_control_server> control_server_;
	std::unique_ptr<franka_state_server> state_server_;


};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP) */