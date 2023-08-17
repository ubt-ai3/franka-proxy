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


#include <array>
#include <atomic>
#include <thread>
#include <vector>

#include <franka/robot.h>


namespace franka_proxy
{
class ft_sensor;

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
		franka::Robot& robot,
		franka::RobotState& robot_state,
		ft_sensor& fts);


	void start();
	void stop();

	// this is blocking
	void start(float seconds);


	std::vector<std::array<double, 7>> latest_record();
	std::vector<std::array<double, 6>> latest_fts_record();

private:
	std::vector<std::array<double, 7>> record_;
	std::vector<std::array<double, 6>> fts_record_;

	franka::Robot& robot_;
	franka::RobotState& robot_state_;
	ft_sensor& fts_;

	std::thread t_{};
	std::atomic_bool stop_{false};
};
} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP) */
