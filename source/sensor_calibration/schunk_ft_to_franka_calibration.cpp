#include "schunk_ft_to_franka_calibration.hpp"

#include <iostream>

#include "franka_control/franka_util.hpp"
#include "franka_proxy/motion_recorder.hpp"

franka_control::force_torque_config_cartesian schunk_ft_sensor_to_franka_calibration::calibrate_bias(
	franka_control::franka_controller_remote& franka,
	double record_time_per_pose_seconds,
	double wait_time_seconds)
{
	std::cout << "Starting Calibration." << std::endl;

	std::array<Eigen::Affine3d, 24> poses = calibration_poses();

	//franka_proxy::detail::motion_recorder recorder = franka_proxy::detail::motion_recorder(0.5, franka.robot_, franka.robot_state()); //to do: figure out a sensible way to reuse the robot here

	//calculate the avg forces/torques at rest for each pose to allow for even weighting
	std::array<std::array<double, 6>, 24> ft_avgs;
	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		ft_avgs[pose_idx] = {0, 0, 0, 0, 0, 0};
	}

	franka.start_recording();
	std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
	auto prev_joint_config = franka.stop_recording().first.back();

	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		if (pose_idx % 4 == 0) std::cout << "change axis" << std::endl;
		std::cout << "pose idx" << pose_idx << std::endl;
		franka_control::robot_config_7dof q{};
		// get the closest config matching the pose
		auto ik_solution = franka_control::franka_util::ik_fast_closest(poses[pose_idx], prev_joint_config);
		Eigen::VectorXd::Map(&q[0], 7) = ik_solution;


		franka.move(q);


		std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
		franka.start_recording();

		std::this_thread::sleep_for(std::chrono::duration<double>(record_time_per_pose_seconds));
		auto [joint_record, force_record] = franka.stop_recording();


		for (int record_idx = 0; record_idx < force_record.size(); record_idx++)
			for (int ft_idx = 0; ft_idx < force_record[record_idx].size(); ft_idx++)
				ft_avgs[pose_idx][ft_idx] += force_record[record_idx][ft_idx];

		for (int ft_idx = 0; ft_idx < ft_avgs[0].size(); ft_idx++)
			ft_avgs[pose_idx][ft_idx] = ft_avgs[pose_idx][ft_idx] / force_record.size();


		prev_joint_config = Eigen::Matrix<double, 7, 1>(joint_record.back().data());
	}

	franka_control::force_torque_config_cartesian biases = {0, 0, 0, 0, 0, 0};
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

std::array<Eigen::Affine3d, 24> schunk_ft_sensor_to_franka_calibration::calibration_poses()
{

	const franka_control::robot_config_7dof x_up_position{
		1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602
	};

	const franka_control::robot_config_7dof y_up_position = {
		1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602
	};

	const franka_control::robot_config_7dof z_up_position = {
		1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602
	};
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
	std::array<Eigen::Vector3d, 3> axis_vecs({
		Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()
	});

	int step = 4;
	constexpr double pi = 3.14159265358979323846;
	
	for (int axis_num = 0; axis_num < axis_vecs.size(); axis_num++)
	{
		Eigen::Vector3d up = axis_vecs.at(axis_num);
		Eigen::Vector3d front = axis_vecs.at((axis_num + 1) % axis_vecs.size());
		int idx_first_pose_axis = 2 * axis_num * step;

		poses.at(idx_first_pose_axis).linear() = get_axis_aligned_orientation(up, front);
		for (int i = idx_first_pose_axis + 1; i < idx_first_pose_axis + step ; i++)
		{
			std::cout << "1. i " << i << std::endl;
			auto tmp = poses.at(i - 1);
			poses.at(i).linear() = tmp.rotate(Eigen::AngleAxis(0.5 * pi, up)).linear();
		}
		poses.at(idx_first_pose_axis + step).linear() = get_axis_aligned_orientation(-1 * up, front);
		for (int i = idx_first_pose_axis + step + 1; i < idx_first_pose_axis + 2*step; i++)
		{
			std::cout << "2. i " << i << std::endl;
			auto tmp = poses.at(i - 1);
			poses.at(i).linear() = tmp.rotate(Eigen::AngleAxis(0.5 * pi, up)).linear();
		}
	}

	return poses;
}

Eigen::Matrix3d schunk_ft_sensor_to_franka_calibration::get_axis_aligned_orientation(
	const Eigen::Vector3d& up, const Eigen::Vector3d& front)
{
	Eigen::Matrix3d orientation;
	orientation << front, up.cross(front), up;
	return orientation;
}
