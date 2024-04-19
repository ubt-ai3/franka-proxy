/**
 *************************************************************************
 *
 * @file motion_recorder.hpp
 * 
 * todo
 *
 ************************************************************************/

#pragma once


#include <array>
#include <atomic>
#include <thread>
#include <vector>

#include <franka/robot.h>

#include "logging/logger.hpp"


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
		ft_sensor* fts);

	void start(bool log, std::string& file);
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop();

	// this is blocking
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> start(float seconds, bool log, std::string& file);


private:
	std::vector<std::array<double, 7>> joints_record_;
	std::vector<std::array<double, 6>> fts_record_;

	franka::Robot& robot_;
	franka::RobotState& robot_state_;
	ft_sensor* fts_;

	std::thread t_{};
	std::atomic_bool stop_{false};

	// for logging
	std::string file_;
	std::array<std::string, 7> joints_ = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };
	std::vector<std::array<std::string, 7>> j_ = { joints_ };
	std::array<std::string, 6> ft_ = { "force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z" };
	std::vector<std::array<std::string, 6>> f_ = { ft_ };
};
} /* namespace detail */
} /* namespace franka_proxy */
