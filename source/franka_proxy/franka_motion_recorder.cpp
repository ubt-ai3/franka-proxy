/**
 *************************************************************************
 *
 * @file franka_motion_recorder.cpp
 *
 * todo..., implementation.
 *
 ************************************************************************/

#include "ft_sensor/ft_sensor.hpp"

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
motion_recorder::motion_recorder(franka::Robot& robot,
                                 franka::RobotState& robot_state,
                                 ft_sensor& fts)
	: robot_(robot),
	  robot_state_(robot_state),
	  fts_(fts)
{
}


void motion_recorder::start()
{
	stop_ = false;
	record_.clear();
	fts_record_.clear();

	t_ = std::thread
	([this]()
	{
		while (!stop_)
		{
			franka::RobotState current_state(robot_.readOnce()); // sync call with approx. 1kHz
			record_.emplace_back(current_state.q);

			ft_sensor_response current_ft(fts_.read());
			fts_record_.emplace_back(current_ft.data);

			// todo add state mutex?
			robot_state_ = current_state;
		}
	});
}

void motion_recorder::start(float seconds)
{
	std::thread([&]()
	{
		std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
		stop();
	}).detach();

	start();
}


void motion_recorder::stop()
{
	stop_ = true;
	t_.join();
}


std::vector<std::array<double, 7>> motion_recorder::latest_record()
{
	if (!stop_)
		throw std::exception("record is still running");

	return record_;
}

std::vector<std::array<double, 6>> motion_recorder::latest_fts_record()
{
	if (!stop_)
		throw std::exception("record is still running");

	return fts_record_;
}
} /* namespace detail */
} /* namespace franka_proxy */
