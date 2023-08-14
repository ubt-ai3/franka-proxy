#define _USE_MATH_DEFINES

#include "schunk_ft_to_franka_calibration.hpp"

#include<math.h>
#include <iostream>
#include "franka_control/franka_util.hpp"

std::array<Eigen::Affine3d, 24> schunk_ft_sensor_to_franka_calibration::calibration_poses(
	const franka_control::robot_config_7dof& x_up_position,
	const franka_control::robot_config_7dof& y_up_position,
	const franka_control::robot_config_7dof& z_up_position)
{

	// 8 poses per up-axis a to use all other axis-aligned directions as front: 4 with a up and 4 with a down
	std::array<Eigen::Affine3d, 24> poses;

	//position of the endeffector
	Eigen::Affine3d x_up_pos(franka_control::franka_util::fk(x_up_position).back());
	Eigen::Affine3d y_up_pos(franka_control::franka_util::fk(y_up_position).back());
	Eigen::Affine3d z_up_pos(franka_control::franka_util::fk(z_up_position).back());

	std::fill_n(poses.begin(), 8, x_up_pos);
	std::fill_n(poses.begin() + 8, 8, y_up_pos);
	std::fill_n(poses.begin() + 16, 8, z_up_pos);

	//orientation of the endeffector
	std::array<Eigen::Vector3f, 3> axis_vecs({ Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ() });
	int axis_num = 0;
	int step = 4;
	for (int axis_num = 0; axis_num < axis_vecs.size(); axis_num++)
	{
		Eigen::Vector3f up = axis_vecs.at(axis_num);
		Eigen::Vector3f front = axis_vecs.at((axis_num + 1) % axis_vecs.size());

		poses.at(axis_num).linear() = get_axis_aligned_orientation(up, front);
		for (int i = 1; i < step; i++) {
			poses.at(axis_num*step + i).linear() = poses.at(i - 1).rotate(Eigen::AngleAxis(0.5 * M_PI, up)).linear();
		}
		poses.at(axis_num + step).linear() = get_axis_aligned_orientation(-1 * up, front);
		for (int i = step; i < 2*step; i++) {
			poses.at(i).linear() = poses.at(i - 1).rotate(Eigen::AngleAxis(0.5 * M_PI, up)).linear();
		}
	}

	return poses;
}

Eigen::Matrix3d schunk_ft_sensor_to_franka_calibration::get_axis_aligned_orientation(const Eigen::Vector3f& up, const Eigen::Vector3f& front)
{
	Eigen::Matrix3d orientation;
	orientation << front, front.cross(up), up;
	return orientation;
}
