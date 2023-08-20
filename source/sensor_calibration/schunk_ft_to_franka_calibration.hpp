#if !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPP)
#define INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HP

#include <array>
#include <Eigen/Geometry>
#include <franka_control/franka_controller_remote.hpp>

class schunk_ft_sensor_to_franka_calibration
{
public:
	// @return the current bias of the kms (depends on temperature & how mcuh the screws were tighntened), e. g. (-0.482146, -3.1187, 0.966813, -0.0460406, -0.077705, -0.080533)
	static franka_control::force_torque_config_cartesian calibrate_bias(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0);

	//  use after bias of the kms was set
	// @return the current load of the robot at the endeffector in world coordinates
	//static Eigen::Affine3d calibrate_load(
	static Eigen::Vector3d calibrate_load(
		franka_control::franka_controller_remote& franka,
		double record_time_per_pose_seconds = 2.0,
		double wait_time_seconds = 2.0
	);
	//todo: decide on return type

private:

	static std::array<Eigen::Affine3d, 24> calibration_poses_bias();
	static std::array<Eigen::Affine3d, 9> calibration_poses_load();
	static Eigen::Matrix3d get_axis_aligned_orientation(const Eigen::Vector3d& up, const Eigen::Vector3d& front);
};

#endif /* !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPPP) */
