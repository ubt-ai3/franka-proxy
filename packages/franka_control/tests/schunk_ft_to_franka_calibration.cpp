#include "schunk_ft_to_franka_calibration.hpp"

#include <iostream>
#include <fstream>
#include <numbers>

#include <nlohmann/json.hpp>

#include <franka_proxy_share/franka_proxy_util.hpp>


franka_control::wrench schunk_ft_sensor_to_franka_calibration::calibrate_bias(
	franka_control::franka_controller_remote& franka,
	double record_time_per_pose_seconds,
	double wait_time_seconds,
	const std::string& config_file)
{
	std::cout << "Starting bias calibration." << '\n';

	std::ifstream in_stream(config_file); 
	nlohmann::json config_json;
	try
	{
		config_json = nlohmann::json::parse(in_stream);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Could not read fts config file: " << config_file
			<< " with " << e.what() << "\n";
	}

	franka.set_fts_bias({0, 0, 0, 0, 0, 0});
	std::array<Eigen::Affine3d, 24> poses = calibration_poses_bias();

	//calculate the avg forces/torques at rest for each pose to allow for even weighting
	std::array<std::array<double, 6>, 24> ft_avgs{ {0.,0.,0.,0.,0.,0.} };
	franka_control::robot_config_7dof prev_joint_config = franka.current_config();

	for (int pose_idx = 0; pose_idx < ft_avgs.size(); pose_idx++)
	{
		std::cout << "Progress calibrate_bias: " << pose_idx + 1 << " / " << ft_avgs.size() << '\n';

		franka_control::robot_config_7dof q =
			franka_proxy::franka_proxy_util::ik_fast_closest(poses[pose_idx], prev_joint_config);

		// Try moving to q until reached.
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
		franka.start_recording();
		std::this_thread::sleep_for(std::chrono::duration<double>(record_time_per_pose_seconds));
		auto [joint_record, force_record] = franka.stop_recording();

		for (auto& record_idx : force_record)
			for (int ft_idx = 0; ft_idx < record_idx.size(); ft_idx++)
				ft_avgs[pose_idx][ft_idx] += record_idx[ft_idx];

		for (int ft_idx = 0; ft_idx < ft_avgs[0].size(); ft_idx++)
			ft_avgs[pose_idx][ft_idx] /= static_cast<double>(force_record.size());


		prev_joint_config = franka.current_config();
	}

	franka_control::wrench bias = {0, 0, 0, 0, 0, 0};
	for (auto& ft_avg : ft_avgs)
		for (int ft_idx = 0; ft_idx < ft_avg.size(); ft_idx++)
			bias[ft_idx] += ft_avg[ft_idx];
	for (int ft_idx = 0; ft_idx < bias.size(); ft_idx++)
		bias[ft_idx] = bias[ft_idx] / ft_avgs.size();

	std::cout << "Calibrated bias: " << bias.transpose() << '\n';

	std::array<double, 6> biases_array;
	Eigen::Vector<double, 6>::Map(biases_array.data()) = bias;
	config_json["bias"] = biases_array;
	std::ofstream out_stream("./current_fts_config.json");
	out_stream << std::setw(4) << config_json << '\n';

	franka.set_fts_bias(bias);
	return bias;
}


double schunk_ft_sensor_to_franka_calibration::calibrate_load(
	franka_control::franka_controller_remote& franka,
	double record_time_per_pose_seconds,
	double wait_time_seconds,
	const std::string& config_file)
{
	std::cout << "Starting load calibration." << '\n';

	std::ifstream in_stream(config_file);
	nlohmann::json config_json;
	try
	{
		config_json = nlohmann::json::parse(in_stream);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Could not read fts config file: " << config_file
			<< " with " << e.what() << "\n";
	}

	std::array<Eigen::Affine3d, 5> poses = calibration_poses_load();
	std::array<Eigen::Vector3d, 5> force_world_avgs{ Eigen::Vector3d{0., 0., 0.} };
	franka_control::robot_config_7dof prev_joint_config = franka.current_config();

	for (int pose_idx = 0; pose_idx < poses.size(); pose_idx++)
	{
		std::cout << "Progress calibrate_load: " << pose_idx + 1 << " / "
			<< force_world_avgs.size() << '\n';

		franka_control::robot_config_7dof q = 
			franka_proxy::franka_proxy_util::ik_fast_closest(poses[pose_idx], prev_joint_config);

		// Try moving to q until reached.
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
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				franka.automatic_error_recovery();
			}
		}


		std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_seconds));
		franka.start_recording();
		std::this_thread::sleep_for(std::chrono::duration<double>(record_time_per_pose_seconds));
		auto [joint_record, force_record] = franka.stop_recording();

		//Eigen::Affine3d world_rot_kms(Eigen::Affine3d::Identity());
		//world_rot_kms *= (poses[pose_idx].rotation());
		//world_rot_kms.rotate(Eigen::AngleAxisd(-std::numbers::pi, Eigen::Vector3d::UnitZ()));

		for (auto& record_idx : force_record)
		{
			Eigen::Vector3d force_kms;
			force_kms << record_idx[0],
				record_idx[1],
				record_idx[2];
			force_world_avgs[pose_idx] += /*world_rot_kms * */force_kms;
		}

		for (int force_idx = 0; force_idx < force_world_avgs[0].size(); force_idx++)
			force_world_avgs[pose_idx][force_idx] /= static_cast<double>(force_record.size());


		prev_joint_config = franka.current_config();
	}

	double load_mass = 0.;
	for (const auto& force_world_avg : force_world_avgs)
		load_mass += force_world_avg.norm() / 9.81;
	load_mass /= poses.size();

	config_json["load_mass"] = load_mass;
	std::ofstream out_stream("./current_fts_config.json");
	out_stream << std::setw(4) << config_json << '\n';

	franka.set_fts_load_mass(load_mass);
	std::cout << "Calibrated load: " << load_mass << '\n';

	return load_mass;
}


std::array<Eigen::Affine3d, 24> schunk_ft_sensor_to_franka_calibration::calibration_poses_bias()
{
	const franka_control::robot_config_7dof initial_position{
		1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602
	};

	// Prepare 24 poses with fixed end-effector position.
	std::array<Eigen::Affine3d, 24> poses;
	std::fill_n(poses.begin(), poses.size(), franka_proxy::franka_proxy_util::fk(initial_position).back());

	// Define primary axis vectors for orientation.
	const std::array<Eigen::Vector3d, 3> axes = {
		Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()
	};

	// Iterate over each axis and set orientation.
	for (int axis_idx = 0; axis_idx < axes.size(); ++axis_idx)
	{
		constexpr int poses_per_axis = 8;

		const Eigen::Vector3d& up = axes[axis_idx];
		const Eigen::Vector3d& front = axes[(axis_idx + 1) % axes.size()];
		int start_idx = axis_idx * poses_per_axis;

		// Set four poses with "up" orientation.
		poses[start_idx].linear() = get_axis_aligned_orientation(up, front);
		for (int i = 1; i < poses_per_axis / 2; ++i)
			poses[start_idx + i].linear() = (poses[start_idx + i - 1] *
				Eigen::Affine3d(Eigen::AngleAxis(0.5 * std::numbers::pi, up))).linear();

		// Set four poses with "-up" orientation.
		poses[start_idx + poses_per_axis / 2].linear() = get_axis_aligned_orientation(-up, front);
		for (int i = 1; i < poses_per_axis / 2; ++i)
			poses[start_idx + poses_per_axis / 2 + i].linear() = (poses[start_idx + poses_per_axis / 2 + i - 1] *
				Eigen::Affine3d(Eigen::AngleAxis(0.5 * std::numbers::pi, up))).linear();
	}

	return poses;
}


std::array<Eigen::Affine3d, 5> schunk_ft_sensor_to_franka_calibration::calibration_poses_load()
{
	const franka_control::robot_config_7dof initial_position{
		-1.31589, 0.305587, 1.48077, -1.818, -0.309624, 1.87622, 0.835527
	};

	// Set 5 likely robot orientations for force/torque controlled motions.
	std::array<Eigen::Affine3d, 5> poses;
	std::fill_n(poses.begin(), poses.size(), franka_proxy::franka_proxy_util::fk(initial_position).back());

	// Set initial orientation with a small rotation around z-axis.
	poses[0].linear() = get_axis_aligned_orientation(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
	poses[0] = poses[0] * Eigen::Affine3d(Eigen::AngleAxisd(0.25 * std::numbers::pi, Eigen::Vector3d::UnitZ()));

	// Define axes for rotating orientations.
	const std::array<Eigen::Vector3d, 2> axes = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};
	constexpr double angle = 0.33 * std::numbers::pi;

	// Set additional orientations by applying rotations around each axis.
	for (int i = 0; i < axes.size(); ++i)
	{
		poses[i + 1] = poses[0] * Eigen::Affine3d(Eigen::AngleAxisd(angle, axes[i]));
		poses[i + 3] = poses[0] * Eigen::Affine3d(Eigen::AngleAxisd(-2 * angle, axes[i]));
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
