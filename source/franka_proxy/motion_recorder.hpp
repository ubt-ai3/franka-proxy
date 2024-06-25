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
#include <optional>

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
		ft_sensor* fts);

	void start(const std::optional<std::string>& log_file_path = std::nullopt);
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop();

	// this is blocking
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> start(
		float seconds, const std::optional<std::string>& log_file_path = std::nullopt);

private:
	std::vector<std::array<double, 7>> joints_record_;
	std::vector<std::array<double, 6>> fts_record_;

	franka::Robot& robot_;
	franka::RobotState& robot_state_;
	ft_sensor* fts_;

	std::thread t_{};
	std::atomic_bool stop_{false};

	// for logging
	bool log_ = false;
	std::string file_;
	std::vector<std::string> joints_ =
		{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
	std::vector<std::string> ft_ =
		{"force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"};
};
} /* namespace detail */
} /* namespace franka_proxy */
