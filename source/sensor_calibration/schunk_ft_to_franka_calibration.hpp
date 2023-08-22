#if !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPP)
#define INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HP

#include <array>
#include <Eigen/Geometry>
#include <franka_control/franka_controller_remote.hpp>

class schunk_ft_sensor_to_franka_calibration
{
public:
	// @return the current bias of the kms (depends on temperature & how mcuh the screws were tighntened), e. g. (-0.556029, -3.17169, 0.895239, -0.051808, -0.0642001, -0.0869519)
	static franka_control::force_torque_config_cartesian calibrate_bias(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0);

	//  use after bias of the kms was set
	// @return the current load of the robot at the endeffector in world coordinates e.g. (0.1492, -0.0427121, -8.58648)
	static Eigen::Vector3d calibrate_load(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0
	);

private:

	static std::array<Eigen::Affine3d, 24> calibration_poses_bias();
	static std::array<Eigen::Affine3d, 9> calibration_poses_load();
	static Eigen::Matrix3d get_axis_aligned_orientation(const Eigen::Vector3d& up, const Eigen::Vector3d& front);
};

#endif /* !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPPP) */
