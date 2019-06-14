/**
 *************************************************************************
 *
 * @file franka_remote_controller.hpp
 *
 * Client side implementation of the franka_proxy.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP)
#define INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP


#include <atomic>
#include <string>

#include "libfranka_types/robot_state.h"
#include "libfranka_types/gripper_state.h"
#include <mutex>


namespace franka_proxy
{


typedef std::array<double, 7> robot_config_7dof;


class franka_remote_controller
{

public:

	franka_remote_controller
		(const std::string& proxy_ip = "192.168.1.1");

	virtual ~franka_remote_controller() noexcept;


	/**
	 * Moves the Panda robot to given target; In case
	 * of collision, the movement is retried and continued 
	 * after automatic error recovery
	 */
	void move_to(const robot_config_7dof& target);

	void stop_movement();

	franka::RobotState current_state() const;

	double speed_factor() const;
	void set_speed_factor(double speed_factor);


	/** Move the gripper to gripper::max_width. */
	void open_gripper();
	/** Grasp.... */
	void close_gripper();

	franka::GripperState current_gripper_state();

	void stop_gripper_movement();


	void update();


private:

	void send_message(const std::string& msg);
	void send_message(const char* msg);


	mutable std::mutex state_lock_;
	franka::RobotState current_state_;
	double current_speed_factor_;
	franka::GripperState current_gripper_state_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP) */