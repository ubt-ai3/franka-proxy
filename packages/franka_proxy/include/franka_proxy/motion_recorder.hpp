#ifndef INCLUDED__FRANKA_PROXY__MOTION_RECORDER_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_RECORDER_HPP
/**
 *************************************************************************
 *
 * @file motion_recorder.hpp
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
 * Records hybrid force/motion data for generation of demonstration data.
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

	static std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>> compute_twist_and_acc_in_world(
		const Eigen::Affine3d& prev_transform,
		const Eigen::Affine3d& transform,
		double dt,
		const Eigen::Matrix<double, 6, 1>& prev_twist);

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

	// Exponentially weighted moving average filtering
	Eigen::Matrix<double, 6, 1> twist_filtered_ = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> twist_filtered_prev_ = Eigen::Matrix<double, 6, 1>::Zero();
	bool filter_initialized_ = false;
	double tau_v_lin_ = 0.02; // ~8 Hz cutoff for linear velocity
	double tau_v_ang_ = 0.05; // ~3 Hz cutoff for angular velocity (more smoothing)

	std::thread t_{};
	std::atomic_bool stop_{false};

	// logging
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
