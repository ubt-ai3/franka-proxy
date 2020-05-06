/**
 *************************************************************************
 *
 * @file franka_controller.cpp
 *
 * Base class for franka controllers, implementation.
 *
 ************************************************************************/


#include "franka_controller.hpp"

#include <viral_core/log.hpp>
#include <viral_core/timer.hpp>
#include <vector>
#include "franka_util.hpp"


namespace franka_control
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_controller
//
//////////////////////////////////////////////////////////////////////////


franka_controller::franka_controller() {}


franka_controller::~franka_controller() noexcept = default;


void franka_controller::move_to(const Eigen::Affine3d& target)
{
	move_to(franka_util::ik_fast_closest(target, current_config()));
}


bool franka_controller::move_to_until_contact(const Eigen::Affine3d& target)
{
	return move_to_until_contact
		(franka_util::ik_fast_closest(target, current_config()));
}


Eigen::Affine3d franka_controller::current_nsa_T_world() const
{
	return franka_util::fk(current_config()).back().inverse();
}




//////////////////////////////////////////////////////////////////////////
//
// franka_update_task
//
//////////////////////////////////////////////////////////////////////////


franka_update_task::franka_update_task
	(franka_controller& controller)
	:
	controller_(controller)
{
	LOG_INFO("Creating and running robot controller task.");
	start(); 
}


franka_update_task::~franka_update_task() NOTHROW
{
	LOG_INFO("Destroying.");
	LOG_INFO("Terminating internal thread.");

	try { join(); }
	catch (...)
	{
		LOG_CRITICAL
			("Internal thread threw an exception on joining.");
	}

	LOG_INFO("Destroyed.");
}


void franka_update_task::task_main()
{
	step_timer timer(update_timestep_secs_, 1.f);

	while (true)
	{
		if (joining_now()) break;

		timer.try_sleep();
		timer.update();
		if (!timer.has_next_timestep()) continue;


		// While timesteps are not required for networking,
		// there are some backends that prefer time-locked
		// update() calls (e.g. to move a virtual robot).
		while (timer.next_timestep())
			controller_.update();
	}
}


const float franka_update_task::update_timestep_secs_ = 0.01f;




} /* namespace franka_control */