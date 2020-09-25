/**
 *************************************************************************
 *
 * @file franka_controller.cpp
 *
 * Base class for franka controllers, implementation.
 *
 ************************************************************************/


#include "franka_controller.hpp"

#include <vector>
#include <iostream>

#include "franka_util.hpp"


namespace franka_control
{


//////////////////////////////////////////////////////////////////////////
//
// franka_controller
//
//////////////////////////////////////////////////////////////////////////


franka_controller::franka_controller() = default;


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


Eigen::Affine3d franka_controller::current_flange_T_world() const
{
	return Eigen::Translation3d(0, 0, -0.107) 
		* current_nsa_T_world();
}


Eigen::Affine3d franka_controller::current_tcp_T_world() const
{
	return Eigen::AngleAxisd(-135.0/180.0 * franka_util::pi, Eigen::Vector3d(0,0,1))
		* Eigen::Translation3d(0, 0, -0.1564)
		* current_flange_T_world();
}


//////////////////////////////////////////////////////////////////////////
//
// franka_update_task
//
//////////////////////////////////////////////////////////////////////////


franka_update_task::franka_update_task
	(franka_controller& controller)
	:
	controller_(controller),
	terminate_internal_thread_(false)
{
	internal_thread_ = std::thread([this]{task_main();});
}


franka_update_task::~franka_update_task() noexcept
{
	terminate_internal_thread_ = true;
	try { internal_thread_.join(); }
	catch (...)
	{
		std::cerr << "franka_state_server::~franka_state_server(): " <<
			"Internal thread threw an exception on joining.";
	}
}


void franka_update_task::task_main()
{
	auto step_duration = std::chrono::duration_cast<std::chrono::microseconds>
		(std::chrono::duration<double>(update_timestep_secs_));
	
	while (!terminate_internal_thread_)
	{
		auto next_time_point = std::chrono::steady_clock::now() + step_duration;

		controller_.update();

		std::this_thread::sleep_until(next_time_point);
	}
}


const double franka_update_task::update_timestep_secs_ = 0.01667;




} /* namespace franka_control */
