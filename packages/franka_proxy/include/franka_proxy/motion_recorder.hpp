#ifndef INCLUDED__FRANKA_PROXY__MOTION_RECORDER_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_RECORDER_HPP
/**
 *************************************************************************
 *
 * @file motion_recorder.hpp
 * 
 * todo
 *
 ************************************************************************/

#include <array>
#include <atomic>
#include <thread>
#include <vector>
#include <optional>

#include <franka/model.h>
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

	void start(std::optional<std::string> log_file_path = std::nullopt);
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop();

	// this is blocking
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> start(
		float seconds, std::optional<std::string> log_file_path = std::nullopt);

	static std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>> compute_twist_and_acc(
		const Eigen::Affine3d& prev_transform,
		const Eigen::Affine3d& transform,
		double dt,
		const Eigen::Matrix<double, 6, 1>& prev_twist);

	// TODO not optimal this needs current velocity_ and acceleration_ to be correct
	std::array<double, 6> compensate_wrench(
		const ft_sensor_response& current_ft,
		const Eigen::Matrix3d& inv_rot);
private:
	std::vector<std::array<double, 7>> joints_record_;
	std::vector<std::array<double, 6>> fts_record_;

	franka::Robot& robot_;
	franka::RobotState& robot_state_;
	ft_sensor* fts_;

	franka::Model model_;

	bool prev_existing_ = false;
	Eigen::Matrix<double, 6, 1> prev_velocity_;
	Eigen::Affine3d prev_transform_;

	Eigen::Matrix<double, 6, 1> velocity_;
	Eigen::Matrix<double, 6, 1> acceleration_;

	// TODO hard coded load parameters atm
	double load_mass_;
	Eigen::Vector3d tool_com_;
	Eigen::Matrix3d tool_inertia_matrix_;
	const Eigen::Vector3d grav_ = Eigen::Vector3d(0.0, 0.0, -9.81);

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

#endif // INCLUDED__FRANKA_PROXY__MOTION_RECORDER_HPP
