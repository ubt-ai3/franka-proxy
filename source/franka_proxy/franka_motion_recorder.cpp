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

	for (int i = 0; i < 10000; ++i)
	{
		std::array<double, 7> tmp{{0.,0.,0.,0.,0.,0.,0.}};
		record_.emplace_back(tmp);
	}
}


void motion_recorder::stop()
{
}


std::vector<std::array<double, 7>> motion_recorder::latest_record()
{
	return record_;
}


} /* namespace detail */
} /* namespace franka_proxy */
