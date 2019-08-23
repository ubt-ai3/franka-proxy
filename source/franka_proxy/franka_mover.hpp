/**
 *************************************************************************
 *
 * @file franka_mover.hpp
 *
 * Queued command execution.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOVER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOVER_HPP


#include <Eigen/Geometry>

#include <viral_core/auto_pointer.hpp>
#include <viral_core/owning_list.hpp>
#include <viral_core/thread.hpp>

#include <mutex>

#include "franka_hardware_controller.hpp"


namespace franka_proxy
{


class franka_command
{
public:

	enum command_type { cartesian_movement, joint_movement, gripper_movement };

	virtual ~franka_command() = default;

	virtual command_type type() = 0;
};


class cartesian_command :
	public franka_command
{
public:

	cartesian_command(const Eigen::Affine3d& target);

	command_type type() override;

	const Eigen::Affine3d target;
};


class joint_command :
	public franka_command
{
public:

	joint_command(const robot_config_7dof& target);

	command_type type() override;

	const robot_config_7dof target;
};


class gripper_command :
	public franka_command
{
public:

	gripper_command(bool open);

	command_type type() override;

	const bool open;
};




/**
 *************************************************************************
 *
 * @class franka_mover
 *
 * ...
 *
 ************************************************************************/
class franka_mover :
	public viral_core::threaded_task
{
public:

	franka_mover(franka_hardware_controller& controller);
	~franka_mover() NOTHROW;

	void task_main() override;

	void enqueue(viral_core::auto_pointer<franka_command> command);
	void clear_queue();


private:

	viral_core::signal has_command_;
	std::mutex command_list_lock_;
	viral_core::owning_list<franka_command> command_list_;

	franka_hardware_controller& controller_;
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOVER_HPP) */
