#include "schunk_ft_to_franka_calibration.hpp"

#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>

#include <franka_control/franka_util.hpp>


franka_control::wrench schunk_ft_sensor_to_franka_calibration::calibrate_bias(
	franka_control::franka_controller_remote& franka,
	double record_time_per_pose_seconds,
	double wait_time_seconds)
{
	std::cout << "Starting bias calibration." << std::endl;

	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);

	franka.set_fts_bias({0, 0, 0, 0, 0, 0});
	std::array<Eigen::Affine3d, 24> poses = calibration_poses_bias();

	//calculate the avg forces/torques at rest for each pose to allow for even weighting
	std::array<std::array<double, 6>, 24> ft_avgs;
	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		ft_avgs[pose_idx] = {0, 0, 0, 0, 0, 0};
	}

	franka.start_recording(false, std::string(""));
	std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
	auto prev_joint_config = franka.stop_recording().first.back();

	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		std::cout << "calibrate_bias progress: " << pose_idx << " / " << ft_avgs.size() << std::endl;

		franka_control::robot_config_7dof q{};
		// get the closest config matching the pose
		auto ik_solution =
			franka_control::franka_util::ik_fast_closest(poses[pose_idx], prev_joint_config);
		Eigen::VectorXd::Map(&q[0], 7) = ik_solution;

		bool move_finished = false;
		while (!move_finished)
		{
			try
			{
				franka.move(q);
				move_finished = true;
			}
			catch (const std::exception&)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				franka.automatic_error_recovery();
			}
		}

		std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
		franka.start_recording(false, std::string(""));

		std::this_thread::sleep_for(std::chrono::duration<double>(record_time_per_pose_seconds));
		auto [joint_record, force_record] = franka.stop_recording();


		for (int record_idx = 0; record_idx < force_record.size(); record_idx++)
			for (int ft_idx = 0; ft_idx < force_record[record_idx].size(); ft_idx++)
				ft_avgs[pose_idx][ft_idx] += force_record[record_idx][ft_idx];

		for (int ft_idx = 0; ft_idx < ft_avgs[0].size(); ft_idx++)
			ft_avgs[pose_idx][ft_idx] = ft_avgs[pose_idx][ft_idx] / force_record.size();


		prev_joint_config = Eigen::Matrix<double, 7, 1>(joint_record.back().data());
	}

	franka_control::wrench biases = {0, 0, 0, 0, 0, 0};
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
		config["bias"].at(ft_idx) = biases[ft_idx];
	}
	std::ofstream out_stream(config_file);
	out_stream << std::setw(4) << config << std::endl;
	franka.set_fts_bias(biases);

	std::cout << "calibrated bias: " << biases.transpose() << std::endl;
	return biases;
}

Eigen::Vector3d schunk_ft_sensor_to_franka_calibration::calibrate_load(
	franka_control::franka_controller_remote& franka,
	double record_time_per_pose_seconds,
	double wait_time_seconds)
{
	std::cout << "Starting load calibration." << std::endl;

	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);

	std::array<Eigen::Affine3d, 5> poses = calibration_poses_load();

	std::array<Eigen::Vector3d, 5> force_world_avgs;
	for (int pose_idx = 0; pose_idx < force_world_avgs.size(); pose_idx++)
	{
		force_world_avgs[pose_idx] = {0, 0, 0};
	}

	franka.start_recording(false, std::string(""));
	std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
	auto prev_joint_config = franka.stop_recording().first.back();

	for (int pose_idx = 0; pose_idx < force_world_avgs.size(); pose_idx++)
	{
		std::cout << "calibrate_load progress: " << pose_idx << " / "
			<< force_world_avgs.size() << std::endl;

		franka_control::robot_config_7dof q{};
		// get the closest config matching the pose
		auto ik_solution =
			franka_control::franka_util::ik_fast_closest(poses[pose_idx], prev_joint_config);
		Eigen::VectorXd::Map(&q[0], 7) = ik_solution;


		bool move_finished = false;
		while (!move_finished)
		{
			try
			{
				franka.move(q);
				move_finished = true;
			}
			catch (const std::exception&)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				franka.automatic_error_recovery();
			}
		}


		std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
		franka.start_recording(false, std::string(""));

		std::this_thread::sleep_for(std::chrono::duration<double>(record_time_per_pose_seconds));
		auto [joint_record, force_record] = franka.stop_recording();

		Eigen::Affine3d world_rot_kms(Eigen::Affine3d::Identity());
		world_rot_kms *= (poses[pose_idx].rotation());
		world_rot_kms.rotate(Eigen::AngleAxisd(-pi, Eigen::Vector3d::UnitZ()));

		for (int record_idx = 0; record_idx < force_record.size(); record_idx++)
		{
			Eigen::Vector3d force_kms;
			force_kms << force_record[record_idx][0],
				force_record[record_idx][1],
				force_record[record_idx][2];
			force_world_avgs[pose_idx] += world_rot_kms * force_kms;
		}


		for (int force_idx = 0; force_idx < force_world_avgs[0].size(); force_idx++)
			force_world_avgs[pose_idx][force_idx] =
				force_world_avgs[pose_idx][force_idx] / force_record.size();

		prev_joint_config = Eigen::Matrix<double, 7, 1>(joint_record.back().data());
	}

	Eigen::Vector3d load;
	load << 0, 0, 0;
	for (int pose_idx = 0; pose_idx < force_world_avgs.size(); pose_idx++)
		load += force_world_avgs[pose_idx];

	load = load / force_world_avgs.size();

	config["load_mass"].clear();
	for (int i = 0; i < load.size(); i++)
		config["load_mass"].push_back(load[i]);

	std::ofstream out_stream(config_file);
	out_stream << std::setw(4) << config << std::endl;

	franka.set_fts_load_mass(load);
	std::cout << "calibrated load: " << load.transpose() << std::endl;

	return load;
}

std::array<Eigen::Affine3d, 24> schunk_ft_sensor_to_franka_calibration::calibration_poses_bias()
{
	const franka_control::robot_config_7dof position{
		1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602
	};

	// 8 poses per up-axis a to use all other axis-aligned directions as front:
	// 4 with a up and 4 with a down
	std::array<Eigen::Affine3d, 24> poses;

	//position of the endeffector
	Eigen::Affine3d pos(franka_control::franka_util::fk(position).back());


	std::fill_n(poses.begin(), 24, pos);


	//orientation of the endeffector
	std::array<Eigen::Vector3d, 3> axis_vecs({
		Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()
	});

	int step = 4;

	for (int axis_num = 0; axis_num < axis_vecs.size(); axis_num++)
	{
		Eigen::Vector3d up = axis_vecs.at(axis_num);
		Eigen::Vector3d front = axis_vecs.at((axis_num + 1) % axis_vecs.size());
		int idx_first_pose_axis = 2 * axis_num * step;

		poses.at(idx_first_pose_axis).linear() = get_axis_aligned_orientation(up, front);
		for (int i = idx_first_pose_axis + 1; i < idx_first_pose_axis + step; i++)
		{
			auto tmp = poses.at(i - 1);
			poses.at(i).linear() = tmp.rotate(Eigen::AngleAxis(0.5 * pi, up)).linear();
		}
		poses.at(idx_first_pose_axis + step).linear() =
			get_axis_aligned_orientation(-1 * up, front);
		for (int i = idx_first_pose_axis + step + 1; i < idx_first_pose_axis + 2 * step; i++)
		{
			auto tmp = poses.at(i - 1);
			poses.at(i).linear() = tmp.rotate(Eigen::AngleAxis(0.5 * pi, up)).linear();
		}
	}

	return poses;
}

std::array<Eigen::Affine3d, 5> schunk_ft_sensor_to_franka_calibration::calibration_poses_load()
{
	const franka_control::robot_config_7dof position{
		-1.31589, 0.305587, 1.48077, -1.818, -0.309624, 1.87622, 0.835527
	};

	// using 5 likely orientations of the robot for force/torque controlled motions
	std::array<Eigen::Affine3d, 5> poses;

	Eigen::Affine3d pos(franka_control::franka_util::fk(position).back());
	std::fill_n(poses.begin(), 5, pos);

	poses.at(0).linear() =
		get_axis_aligned_orientation(-1 * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
	poses.at(0) = poses.at(0).rotate(Eigen::AngleAxisd(0.25 * pi, Eigen::Vector3d::UnitZ()));

	std::array<Eigen::Vector3d, 2> axis_vecs(
		{Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()});

	double angle = 0.3 * pi;
	for (int i = 0; i < axis_vecs.size(); i++)
	{
		auto tmp = poses.at(0);
		poses.at(i + 1) = tmp.rotate(Eigen::AngleAxisd(angle, axis_vecs[i]));
		poses.at(axis_vecs.size() + i + 1) = poses.at(i + 1);
		poses.at(axis_vecs.size() + i + 1) = tmp.rotate(Eigen::AngleAxisd(-2 * angle, axis_vecs[i]));
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
