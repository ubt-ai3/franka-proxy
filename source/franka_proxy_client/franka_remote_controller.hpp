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

#include <franka/robot_state.h>
#include <franka/gripper_state.h>


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
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__FRANKA_REMOTE_KONTROLLER_HPP) */