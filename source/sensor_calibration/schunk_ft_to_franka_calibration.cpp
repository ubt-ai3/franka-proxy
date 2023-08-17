#define _USE_MATH_DEFINES

#include "schunk_ft_to_franka_calibration.hpp"

#include<math.h>
#include <iostream>
#include "franka_control/franka_util.hpp"
#include "franka_proxy/franka_motion_recorder.hpp"

franka_proxy::robot_force_config schunk_ft_sensor_to_franka_calibration::calibrate_bias(
	franka_proxy::franka_hardware_controller& franka,
	schunk_ft_sensor& ft_sensor,
	int record_time_per_pose_seconds,
	int wait_time_seconds)
{
	std::cout << "Starting Calibration." << std::endl;

	std::array<Eigen::Affine3d, 24> poses = calibration_poses();

	//franka_proxy::detail::motion_recorder recorder = franka_proxy::detail::motion_recorder(0.5, franka.robot_, franka.robot_state()); //to do: figure out a sensible way to reuse the robot here

	//calculate the avg forces/torques at rest for each pose to allow for even weighting
	std::array<std::array<double, 6>, 24> ft_avgs;
	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		ft_avgs[pose_idx] = { 0, 0, 0, 0, 0, 0 };
	}


	franka_control::robot_config_7dof prev_joint_config = Eigen::Matrix<double, 7, 1>(franka.robot_state().q);

	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		franka_proxy::robot_config_7dof q{};
		// get the closest config matching the pose
		auto ik_solution = franka_control::franka_util::ik_fast_closest(poses[pose_idx], prev_joint_config);
		Eigen::VectorXd::Map(&q[0], 7) = ik_solution;


		franka.move_to(q);

		auto start_record_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(wait_time_seconds));
		std::this_thread::sleep_until(start_record_time);

		auto end_record_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(record_time_per_pose_seconds));
		franka.start_recording();
		std::this_thread::sleep_until(end_record_time);
		std::pair<std::vector<franka_proxy::robot_config_7dof>, std::vector<franka_proxy::robot_force_config>> record_curr_pose = franka.stop_recording();
		std::vector<franka_proxy::robot_force_config> ft_record_curr_pose = record_curr_pose.second;

		for (int record_idx = 0; record_idx < ft_record_curr_pose.size(); record_idx++)
		{
			for (int ft_idx = 0; ft_idx < ft_record_curr_pose[record_idx].size(); ft_idx++)
			{
				ft_avgs[pose_idx][ft_idx] += ft_record_curr_pose[record_idx][ft_idx];
			}
		}

		for (int ft_idx = 0; ft_idx < ft_avgs[0].size(); ft_idx++)
		{
			ft_avgs[pose_idx][ft_idx] = ft_avgs[pose_idx][ft_idx] / ft_record_curr_pose.size();
		}

		prev_joint_config = Eigen::Matrix<double, 7, 1>(record_curr_pose.first.back());
	}

	franka_proxy::robot_force_config biases = { 0, 0, 0, 0 , 0, 0 };
	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		for (int ft_idx = 0; ft_idx < ft_avgs[pose_idx].size(); ft_idx++)
		{
			biases[ft_idx] += ft_avgs[pose_idx][ft_idx];
		}
	}

	for (int ft_idx = 0; ft_idx < biases.size(); ft_idx++)
	{
		biases[ft_idx] = biases[ft_idx] / ft_avgs.size();
	}

	return biases;

}

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
