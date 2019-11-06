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
motion_recorder::motion_recorder(double rate)
{
	
}


void motion_recorder::start()
{
	record_.clear();

	for (int i = 0; i < 100000; ++i)
		record_.emplace_back(i);
}


void motion_recorder::stop()
{
}


std::vector<int> motion_recorder::latest_record()
{
	return record_;
}


} /* namespace detail */
} /* namespace franka_proxy */
