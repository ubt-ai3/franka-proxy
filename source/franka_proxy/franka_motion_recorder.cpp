/**
 *************************************************************************
 *
 * @file franka_motion_recorder.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/

#include "franka_motion_recorder.hpp"


namespace franka_proxy
{
namespace detail
{


//////////////////////////////////////////////////////////////////////////
//
// franka_motion_recorder
//
//////////////////////////////////////////////////////////////////////////


motion_recorder::motion_recorder
	(double rate,
	 franka::Robot& robot,
	 franka::RobotState& robot_state)
	:
	robot_(robot),
	robot_state_(robot_state)
{ }


void motion_recorder::start()
{
	stop_ = false;
	record_.clear();

	t_ = std::thread
	([this]()
	{
		while (!stop_)
		{
			franka::RobotState current_state(robot_.readOnce());
			record_.emplace_back(current_state.q);
			robot_state_ = current_state;
		}
	});
}


void motion_recorder::stop()
{
	stop_ = true;
	t_.join();
}


std::vector<std::array<double, 7>> motion_recorder::latest_record()
{
	return record_;
}


} /* namespace detail */
} /* namespace franka_proxy */
