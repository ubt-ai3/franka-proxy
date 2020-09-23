/**
 *************************************************************************
 *
 * @file franka_motion_recorder.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP

#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <franka/robot.h>

#include "jr3_ft_sensor/force_torque_sensor.hpp"


namespace franka_proxy
{
namespace detail
{


/**
 *************************************************************************
 *
 * @class motion_recorder
 *
 * todo
 *
 ************************************************************************/
class motion_recorder
{
public:
	motion_recorder(
		double rate,
		franka::Robot& robot,
		franka::RobotState& robot_state);

	void start();

	// todo maybe add a timeout
	void stop();

	std::vector<std::array<double, 7>> latest_record();

	std::vector<std::array<double, 6>> latest_fts_record();

private:
	std::vector<std::array<double, 7>> record_;
	std::vector<std::array<double, 6>> fts_record_;

	std::thread t_{};
	std::atomic_bool stop_{false};
	franka::Robot& robot_;
	franka::RobotState& robot_state_;
	ft_sensor_jr3 fts_;
};


} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP) */
