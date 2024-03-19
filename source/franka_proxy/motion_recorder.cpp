/**
 *************************************************************************
 *
 * @file motion_recorder.cpp
 *
 * todo..., implementation.
 *
 ************************************************************************/

#include "ft_sensor/ft_sensor.hpp"

#include "motion_recorder.hpp"


namespace franka_proxy
{
namespace detail
{
//////////////////////////////////////////////////////////////////////////
//
// franka_motion_recorder
//
//////////////////////////////////////////////////////////////////////////
motion_recorder::motion_recorder(franka::Robot& robot,
                                 franka::RobotState& robot_state,
                                 ft_sensor* fts)
	: robot_(robot),
	  robot_state_(robot_state),
	  fts_(fts)
{
}


void motion_recorder::start()
{
	stop_ = false;
	joints_record_.clear();
	fts_record_.clear();

	t_ = std::thread([this]()
	{
		while (!stop_)
		{
			franka::RobotState current_state(robot_.readOnce()); // sync call with approx. 1kHz
			joints_record_.emplace_back(current_state.q);

			if (fts_)
			{
				ft_sensor_response current_ft(fts_->read());
				fts_record_.emplace_back(current_ft.data);
			}

			robot_state_ = current_state;
		}
	});
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> motion_recorder::stop()
{
	stop_ = true;
	t_.join();
	return {joints_record_, fts_record_};
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> motion_recorder::start(float seconds)
{
	start();
	std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
	return stop();
}
} /* namespace detail */
} /* namespace franka_proxy */
