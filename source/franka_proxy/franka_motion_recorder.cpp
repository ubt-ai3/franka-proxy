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


motion_recorder::motion_recorder(double rate, franka::Robot& robot)
	: robot_(robot)
{
	
}


void motion_recorder::start()
{
	stop_ = false;
	record_.clear();

	t_ = std::thread([this]()
	{
		while (!stop_)
		{
			record_.emplace_back(robot_.readOnce().q);
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
