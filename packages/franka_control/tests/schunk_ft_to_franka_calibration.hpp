#ifndef INCLUDED__TESTS__SCHUNK_FT_TO_FRANKA_CALIBRATION_HPP
#define INCLUDED__TESTS__SCHUNK_FT_TO_FRANKA_CALIBRATION_HPP

#include <array>

#include <Eigen/Geometry>

#include <franka_control/franka_controller_remote.hpp>

class schunk_ft_sensor_to_franka_calibration
{
public:
	// calculates and sets the bias of the kms used by franka
	// @return the current bias of the kms (depends on temperature & how mcuh the screws were tightened),
	// e. g. (-0.556029, -3.17169, 0.895239, -0.051808, -0.0642001, -0.0869519)
	// reads and writes from/to config_file if exists @TODO JHa this is not really the intended behaviour
	static franka_control::wrench calibrate_bias(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0,
		const std::string& config_file = "./assets/fts-config.json");

	// calculates and sets the load_mass of the kms used by franka
	// use only after bias of the kms was set
	// @return the current weight force of the load of the robot at the end effector in world coordinates
	// e.g. (0.246864, -0.209424, -8.24472)
	// reads and writes from/to config_file if exists @TODO JHa this is not really the intended behaviour
	static Eigen::Vector3d calibrate_load(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0,
		const std::string& config_file = "./assets/fts-config.json");

private:

	static std::array<Eigen::Affine3d, 24> calibration_poses_bias();
	static std::array<Eigen::Affine3d, 5> calibration_poses_load();
	static Eigen::Matrix3d get_axis_aligned_orientation(
		const Eigen::Vector3d& up, const Eigen::Vector3d& front);
};

#endif // INCLUDED__TESTS__SCHUNK_FT_TO_FRANKA_CALIBRATION_HPP
