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
	robot_state_(robot_state)/*,
	fts_()*/
{ }


void motion_recorder::start()
{
	stop_ = false;
	record_.clear();
	fts_record_.clear();

	//fts_.set_offsets_to_zero();

	t_ = std::thread
	([this]()
	{
		while (!stop_)
		{
			//fts_.update();
			//fts_record_.emplace_back(fts_.current_values());
			fts_record_.emplace_back(std::array<double, 6>{0, 0, 0, 0, 0, 0});

			franka::RobotState current_state(robot_.readOnce());
			record_.emplace_back(current_state.q);

			// todo add state mutex
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

std::vector<std::array<double, 6>> motion_recorder::latest_fts_record()
{
	return fts_record_;
}


} /* namespace detail */
} /* namespace franka_proxy */
