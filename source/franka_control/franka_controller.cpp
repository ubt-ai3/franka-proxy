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


franka_controller::franka_controller()
	:
	j6_T_flange(build_j6_T_flange()),
	flange_T_tcp(build_flange_T_tcp()),
	j6_T_tcp(build_j6_T_tcp()),
	tcp_T_j6(build_tcp_T_j6())
{}


franka_controller::~franka_controller() noexcept = default;


void franka_controller::move(const Eigen::Affine3d& target_world_T_tcp)
{
	move(franka_util::ik_fast_closest
		(target_world_T_tcp * tcp_T_j6, current_config()));
}


bool franka_controller::move_until_contact
	(const Eigen::Affine3d& target_world_T_tcp)
{
	return move_until_contact
		(franka_util::ik_fast_closest
			(target_world_T_tcp * tcp_T_j6, current_config()));
}


Eigen::Affine3d franka_controller::current_world_T_tcp() const
	{ return current_world_T_j6() * j6_T_tcp; }


Eigen::Affine3d franka_controller::current_world_T_j6() const
	{ return franka_util::fk(current_config()).back(); }


Eigen::Affine3d franka_controller::current_world_T_flange() const
	{ return current_world_T_j6() * j6_T_flange; }


Eigen::Affine3d franka_controller::build_j6_T_flange() const
{
	return Eigen::Affine3d(Eigen::Translation3d(0, 0, 0.107));
}


Eigen::Affine3d franka_controller::build_flange_T_tcp() const
{
	Eigen::Affine3d flange_T_tcp(Eigen::AngleAxisd
		(-45. * franka_util::deg_to_rad, Eigen::Vector3d::UnitZ()));
	flange_T_tcp.translate(Eigen::Vector3d(0, 0, 0.1034));
	return flange_T_tcp;
}


Eigen::Affine3d franka_controller::build_j6_T_tcp() const
{
	return build_j6_T_flange() * build_flange_T_tcp();
}


Eigen::Affine3d franka_controller::build_tcp_T_j6() const
{
	return build_j6_T_tcp().inverse();
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
