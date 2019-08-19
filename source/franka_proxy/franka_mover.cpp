/**
 *************************************************************************
 *
 * @file franka_mover.cpp
 *
 * Queued command execution, implementation.
 *
 ************************************************************************/


#include "franka_mover.hpp"

#include <viral_core/log.hpp>


namespace franka_proxy
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// cartesian_command
//
//////////////////////////////////////////////////////////////////////////


cartesian_command::cartesian_command(const Eigen::Affine3d& target)
	: target(target) {}

franka_command::command_type cartesian_command::type()
	{ return cartesian_movement; }




//////////////////////////////////////////////////////////////////////////
//
// joint_command
//
//////////////////////////////////////////////////////////////////////////


joint_command::joint_command(const robot_config_7dof& target)
	: target(target) {}

franka_command::command_type joint_command::type()
	{ return joint_movement; }




//////////////////////////////////////////////////////////////////////////
//
// joint_command
//
//////////////////////////////////////////////////////////////////////////


gripper_command::gripper_command(bool open)
	: open(open) {}

franka_command::command_type gripper_command::type()
	{ return gripper_movement; }


//////////////////////////////////////////////////////////////////////////
//
// franka_mover
//
//////////////////////////////////////////////////////////////////////////


franka_mover::franka_mover(franka_hardware_controller& controller)
	: controller_(controller) {}


franka_mover::~franka_mover() noexcept
{
	controller_.stop_movement();

	try
	{
		join();
	}
	catch(...)
	{
		LOG_CRITICAL("Thread joining threw exception.")
	}
}


void franka_mover::task_main()
{
	while (!join_now())
	{
		has_command_.wait_for(true);
		if (!has_command_.get())
			continue;

		if (join_now())
			return;

		std::unique_lock<std::mutex> command_list_guard(command_list_lock_);
		if (command_list_.empty())
			continue;
		auto_pointer<franka_command> command = command_list_.release_first();
		if (command_list_.empty())
			has_command_.set(false);
		command_list_guard.unlock();

		switch (command->type())
		{
			case franka_command::cartesian_movement:
			{
				// TODO!
				//cartesian_command& c_command = static_cast<cartesian_command&>(*command);
				//controller_.move_to(c_command.target);
				break;
			}

			case franka_command::joint_movement:
			{
				joint_command& j_command = static_cast<joint_command&>(*command);
				controller_.move_to(j_command.target);
				break;
			}

			case franka_command::gripper_movement:
			{
				gripper_command& g_command = static_cast<gripper_command&>(*command);
				if (g_command.open)
					controller_.open_gripper();
				else
					controller_.close_gripper();
				break;
			}

			default:
				break;
		}
	}
}


void franka_mover::enqueue(auto_pointer<franka_command> command)
{
	std::lock_guard<std::mutex> command_list_guard(command_list_lock_);
	command_list_.push_back(std::move(command));
	has_command_.set(true);
}




} /* namespace franka_proxy */