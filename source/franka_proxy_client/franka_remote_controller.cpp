/**
 *************************************************************************
 *
 * @file franka_remote_controller.cpp
 *
 * Client side implementation of the franka_proxy, implementation.
 *
 ************************************************************************/


#include "franka_remote_controller.hpp"

#include "franka_proxy_share/franka_proxy_messages.hpp"


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_remote_controller
//
//////////////////////////////////////////////////////////////////////////


franka_remote_controller::franka_remote_controller(const std::string& proxy_ip)
{}


franka_remote_controller::~franka_remote_controller() noexcept = default;


void franka_remote_controller::move_to(const robot_config_7dof& target)
{
	std::string msg = 
		franka_proxy_messages::move + ' ' +
		std::to_string(target[0]) + ' ' +
		std::to_string(target[0]) + ' ' +
		std::to_string(target[1]) + ' ' +
		std::to_string(target[2]) + ' ' +
		std::to_string(target[3]) + ' ' +
		std::to_string(target[4]) + ' ' +
		std::to_string(target[5]) + ' ' +
		std::to_string(target[6]);
	send_message(msg);
}


void franka_remote_controller::stop_movement()
	{ send_message(franka_proxy_messages::stop_move); }


franka::RobotState franka_remote_controller::current_state() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_state_;
}


double franka_remote_controller::speed_factor() const
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_speed_factor_;
}


void franka_remote_controller::set_speed_factor(double speed_factor)
{
	send_message
		(std::string(franka_proxy_messages::speed) + ' ' +
		 std::to_string(speed_factor));
}


void franka_remote_controller::open_gripper()
	{ send_message(franka_proxy_messages::open_gripper); }
void franka_remote_controller::close_gripper()
	{ send_message(franka_proxy_messages::close_gripper); }


franka::GripperState franka_remote_controller::current_gripper_state()
{
	std::lock_guard<std::mutex> state_guard(state_lock_);
	return current_gripper_state_;
}


void franka_remote_controller::stop_gripper_movement()
	{ send_message(franka_proxy_messages::stop_gripper); }


void franka_remote_controller::update()
{
	
}


void franka_remote_controller::send_message(const std::string& msg)
	{ send_message(msg.data()); }
void franka_remote_controller::send_message(const char* msg)
{
	
}




} /* namespace franka_proxy */