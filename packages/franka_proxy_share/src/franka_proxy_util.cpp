/**
 *************************************************************************
 *
 * @file franka_proxy_util.cpp
 *
 * Utility functions include FK/IK that are needed in multiple places.
 *
 ************************************************************************/
#include "franka_proxy_util.hpp"

#include <iostream>

#include "exception.hpp"
#include "ikfast.h"

namespace franka_proxy
{
joint_limit::joint_limit(
	double min,
	double max)
	: min(min),
	  max(max)
{
}


const joint_limit franka_proxy_util::joint_limits_[] =
{
	{-2.8973, 2.8973},
	{-1.7628, 1.7628},
	{-2.8973, 2.8973},
	{-3.0718, -0.069},
	{-2.8973, 2.8973},
	{-0.0175, 3.7525},
	{-2.8973, 2.8973}
};


std::vector<joint_limit> franka_proxy_util::joint_limits()
{
	return
	{
		joint_limits_[0],
		joint_limits_[1],
		joint_limits_[2],
		joint_limits_[3],
		joint_limits_[4],
		joint_limits_[5],
		joint_limits_[6]
	};
}


franka_proxy_util::robot_config_7dof franka_proxy_util::max_speed_per_joint()
{
	return {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
}


franka_proxy_util::robot_config_7dof franka_proxy_util::max_acc_per_joint()
{
	return {15., 7.5, 10., 12.5, 15., 20., 20.};
}


franka_proxy_util::robot_config_7dof franka_proxy_util::max_jerk_per_joint()
{
	return {7500., 3750., 5000., 6250., 7500., 10000., 10000.};
}


bool franka_proxy_util::is_reachable(
	const robot_config_7dof& target)
{
	auto limits = joint_limits();

	for (int i = 0; i < 7; i++)
		if (target[i] < limits[i].min || target[i] > limits[i].max)
			return false;

	return true;
}


std::vector<Eigen::Affine3d> franka_proxy_util::fk(
	const robot_config_7dof& configuration)
{
	std::array<double, 7> config;
	for (int i = 0; i < 7; ++i)
		config[i] = configuration[i];
	return fk(config);
}


std::vector<Eigen::Affine3d> franka_proxy_util::fk(
	const std::array<double, 7>& configuration)
{
	std::vector<Eigen::Affine3d> frames;
	frames.reserve(8);

	// initialize transformation matrix for FK
	Eigen::Affine3d trafo(Eigen::Affine3d::Identity());
	// trafo *= Eigen::AngleAxisd(3.1415, Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	// link 0 (just a default trafo)
	frames.emplace_back(trafo);

	// link 1 (translation to parent frame)
	trafo *= Eigen::Translation3d(0, 0, 0.333);
	// joint angle
	trafo *= Eigen::AngleAxisd(configuration[0], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 2 (rotation to parent frame)
	trafo *= Eigen::AngleAxisd(-std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[1], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 3 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0, -0.316, 0.);
	trafo *= Eigen::AngleAxisd(std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[2], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 4 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0.0825, 0., 0.);
	trafo *= Eigen::AngleAxisd(std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[3], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 5 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(-0.0825, 0.384, 0.);
	trafo *= Eigen::AngleAxisd(-std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[4], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 6 (rotation to parent frame)
	trafo *= Eigen::AngleAxisd(std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[5], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	// link 7 (rotation to parent frame)
	trafo *= Eigen::Translation3d(0.088, 0., 0.);
	trafo *= Eigen::AngleAxisd(std::numbers::pi / 2., Eigen::Vector3d(1., 0., 0.));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[6], Eigen::Vector3d(0., 0., 1.));
	frames.emplace_back(trafo);

	//// link 7 to flange
	//trafo *= Eigen::Translation3d(0.0f, 0.0f, 0.107);
	//frames.push_back(trafo);

	//// flange to standard end effector
	//trafo *= Eigen::Translation3d(0.0f, 0.0f, 0.1034f);
	//trafo *= Eigen::AngleAxisd(pi / 4., Eigen::Vector3d(0.0f, 0.0f, -1.0f));
	//frames.push_back(trafo);

	return frames;
}


std::vector<franka_proxy_util::robot_config_7dof> franka_proxy_util::ik_fast(
	const Eigen::Affine3d& target_world_T_j7, double joint_4_value)
{
	const Eigen::Affine3d last_segment_T_tcp
		(Eigen::Translation3d(0., 0., 0.107));

	Eigen::Affine3d target
		(target_world_T_j7 * last_segment_T_tcp);

	std::vector<robot_config_7dof> result;

	double rotation[9], translation[3];
	std::vector<double> free(1);
	rotation[0] = target(0, 0);
	rotation[1] = target(0, 1);
	rotation[2] = target(0, 2);
	rotation[3] = target(1, 0);
	rotation[4] = target(1, 1);
	rotation[5] = target(1, 2);
	rotation[6] = target(2, 0);
	rotation[7] = target(2, 1);
	rotation[8] = target(2, 2);

	translation[0] = target(0, 3);
	translation[1] = target(1, 3);
	translation[2] = target(2, 3);

	free[0] = joint_4_value;

	ikfast::IkSolutionList<double> solutions;
	if (ComputeIk(translation, rotation, free.data(), solutions))
	{
		for (int solution_index = 0;
		     solution_index < solutions.GetNumSolutions();
		     solution_index++)
		{
			const auto& solution = solutions.GetSolution(solution_index);
			double joint_angles[7];
			solution.GetSolution(joint_angles, nullptr);

			robot_config_7dof joints;
			for (int k = 0; k < 7; k++)
				joints[k] = joint_angles[k];

			if (is_reachable(joints))
				result.push_back(joints);
		}
	}

	return result;
}


std::vector<franka_proxy_util::robot_config_7dof> franka_proxy_util::ik_fast_robust(
	const Eigen::Affine3d& target_world_T_j7, double step_size)
{
	std::vector<robot_config_7dof> solutions = ik_fast(target_world_T_j7);
	double joint_4 = joint_limits_[4].min;
	while (solutions.empty() && joint_4 < joint_limits_[4].max)
	{
		solutions = ik_fast(target_world_T_j7, joint_4);
		joint_4 += step_size;
	}
	return solutions;
}


franka_proxy_util::robot_config_7dof franka_proxy_util::ik_fast_closest(
	const Eigen::Affine3d& target_world_T_j7,
	const robot_config_7dof& current_configuration,
	double step_size)
{
	// Calculate possible solutions
	std::vector<robot_config_7dof> solutions;
	for (double joint_4 = joint_limits_[4].min; joint_4 < joint_limits_[4].max; joint_4 += step_size)
	{
		std::vector<robot_config_7dof> new_solutions =
			ik_fast(target_world_T_j7, joint_4);
		for (const robot_config_7dof& solution : new_solutions)
			solutions.push_back(solution);
	}

	if (solutions.empty())
	{
		std::cerr << "No solution found." << '\n';
		throw ik_failed();
	}

	// Find closest solution
	robot_config_7dof closest = solutions.front();
	double squared_dist = (current_configuration - closest).squaredNorm();
	for (const robot_config_7dof& solution : solutions)
	{
		double current_squared_dist = (current_configuration - solution).squaredNorm();
		if (current_squared_dist < squared_dist)
		{
			squared_dist = current_squared_dist;
			closest = solution;
		}
	}

	return closest;
}


double franka_proxy_util::tool_mass()
{
	constexpr double
		m_print1 = 0.059,
		m_fts_top = 0.129,
		m_fts_plate = 0.151,
		m_print2 = 0.069,
		m_gripper = 0.73;

	return m_print1 + m_fts_top + m_fts_plate + m_print2 + m_gripper;
}


Eigen::Vector3d franka_proxy_util::tool_center_of_mass()
{
	constexpr double
		m_print1 = 0.059,
		m_fts_top = 0.129,
		m_fts_plate = 0.151,
		m_print2 = 0.069,
		m_gripper = 0.73;

	const Eigen::Vector3d
		c_print1(0, 0, 0.018),	// a bit shifted down because of the screws
		c_fts_top(0, 0, 0.0412),
		c_fts_plate(0, 0, 0.0537),
		c_print2(0, 0, 0.068),  // a bit shifted up because of the screws
		c_gripper(-0.007, -0.007, 0.137);

	return (m_print1 * c_print1 + m_fts_top * c_fts_top + m_fts_plate * c_fts_plate + m_print2 * c_print2 + m_gripper * c_gripper) / tool_mass();
}


Eigen::Matrix3d franka_proxy_util::tool_inertia()
{
	constexpr double
		m_print1 = 0.059,
		m_fts_top = 0.129,
		m_fts_plate = 0.151,
		m_print2 = 0.069,
		m_gripper = 0.73;

	const Eigen::Vector3d
		c_print1(0, 0, 0.018),	// a bit shifted down because of the screws
		c_fts_top(0, 0, 0.0412),
		c_fts_plate(0, 0, 0.0537),
		c_print2(0, 0, 0.068),  // a bit shifted up because of the screws
		c_gripper(-0.007, -0.007, 0.137);

	const Eigen::Vector3d c(tool_center_of_mass());


	// print1 as cylinder
	Eigen::Matrix3d inertia_print1(Eigen::Matrix3d::Zero());
	inertia_print1(0, 0) = inertia_print1(1, 1) = 1 / 12. * m_print1 * (3 * 0.041 * 0.041 + 0.032 * 0.032);
	inertia_print1(2, 2) = 0.5 * m_print1 * (0.032 * 0.032);

	auto a_tilde_print1 = a_tilde(c_print1 - c);
	const Eigen::Matrix3d inertia_print1_center_of_mass =
		inertia_print1 + m_print1 * a_tilde_print1.transpose() * a_tilde_print1;


	// fts as thick-walled cylindrical tube
	Eigen::Matrix3d inertia_fts_top(Eigen::Matrix3d::Zero());
	inertia_fts_top(0, 0) = inertia_fts_top(1, 1) = 1 / 12. * m_fts_top * (3 * (0.041 * 0.041 + 0.015 * 0.015) + 0.0184 * 0.0184);
	inertia_fts_top(2, 2) = 0.5 * m_fts_top * (0.041 * 0.041 + 0.015 * 0.015);

	auto a_tilde_fts_top = a_tilde(c_fts_top - c);
	const Eigen::Matrix3d inertia_fts_top_center_of_mass =
		inertia_fts_top + m_fts_top * a_tilde_fts_top.transpose() * a_tilde_fts_top;


	// fts plate as cylinder
	Eigen::Matrix3d inertia_fts_plate(Eigen::Matrix3d::Zero());
	inertia_fts_plate(0, 0) = inertia_fts_plate(1, 1) = 1. / 12. * m_fts_plate * (3 * 0.041 * 0.041 + 0.0066 * 0.0066);
	inertia_fts_plate(2, 2) = 0.5 * m_fts_plate * 0.041 * 0.041;

	auto a_tilde_fts_plate = a_tilde(c_fts_plate - c);
	const Eigen::Matrix3d inertia_fts_plate_center_of_mass =
		inertia_fts_plate + m_fts_plate * a_tilde_fts_plate.transpose() * a_tilde_fts_plate;


	// print2 as cylinder
	Eigen::Matrix3d inertia_print2(Eigen::Matrix3d::Zero());
	inertia_print2(0, 0) = inertia_print2(1, 1) = 1. / 12. * m_print2 * (3 * 0.041 * 0.041 + 0.025 * 0.025);
	inertia_print2(2, 2) = 0.5 * m_print2 * (0.041 * 0.041);

	auto a_tilde_print2 = a_tilde(c_print2 - c);
	const Eigen::Matrix3d inertia_print2_center_of_mass =
		inertia_print2 + m_print2 * a_tilde_print2.transpose() * a_tilde_print2;


	// gripper with rotation correction
	Eigen::Matrix3d inertia_gripper(Eigen::Matrix3d::Zero());
	inertia_gripper(0, 0) = 0.0025;
	inertia_gripper(1, 1) = 0.001;
	inertia_gripper(2, 2) = 0.0017;

	inertia_gripper =
		Eigen::Matrix3d(Eigen::AngleAxisd(
			std::numbers::pi, Eigen::Vector3d(0, 0, -1))).transpose()
		* inertia_gripper
		* Eigen::Matrix3d(Eigen::AngleAxisd(
			std::numbers::pi, Eigen::Vector3d(0, 0, -1)));

	auto a_tilde_gripper = a_tilde(c_gripper - c);
	const Eigen::Matrix3d inertia_gripper_center_of_mass =
		inertia_gripper + m_gripper * a_tilde_gripper.transpose() * a_tilde_gripper;


	return inertia_print1_center_of_mass
		+ inertia_fts_top_center_of_mass
		+ inertia_fts_plate_center_of_mass
		+ inertia_print2_center_of_mass
		+ inertia_gripper_center_of_mass;
}


double franka_proxy_util::tool_mass_from_fts()
{
	constexpr double
		m_fts = 0.151,		// what fts measures of "itself"
		m_print2 = 0.069,	// print + all screws
		m_gripper = 0.73;	// from franka docs

	return m_fts + m_print2 + m_gripper;
}


Eigen::Vector3d franka_proxy_util::tool_center_of_mass_from_fts()
{
	constexpr double
		m_fts = 0.151,		// what fts measures of "itself"
		m_print2 = 0.069,	// print + all screws
		m_gripper = 0.73;	// from franka docs

	const Eigen::Vector3d
		c_fts(0, 0, -0.0033),
		c_print2(0, 0, 0.011),	// a bit shifted up because of the screws
		c_gripper(0.007, 0.007, 0.055);

	return (m_fts * c_fts + m_print2 * c_print2 + m_gripper * c_gripper) / tool_mass_from_fts();
}


Eigen::Matrix3d franka_proxy_util::tool_inertia_from_fts()
{
	constexpr double
		m_fts = 0.151,		// what fts measures of "itself"
		m_print2 = 0.069,	// print + all screws
		m_gripper = 0.73;	// from franka docs

	const Eigen::Vector3d
		c_fts(0, 0, -0.0033),
		c_print2(0, 0, 0.011),	// a bit shifted up because of the screws
		c_gripper(0.007, 0.007, 0.055);

	const double m = tool_mass_from_fts();
	const Eigen::Vector3d c(tool_center_of_mass_from_fts());


	// fts plate as cylinder
	Eigen::Matrix3d inertia_fts(Eigen::Matrix3d::Zero());
	inertia_fts(0, 0) = inertia_fts(1, 1) = 1. / 12. * m_fts * (3 * 0.041 * 0.041 + 0.0066 * 0.0066);
	inertia_fts(2, 2) = 0.5 * m_fts * 0.041 * 0.041;

	auto a_tilde_fts = a_tilde(c_fts - c);
	const Eigen::Matrix3d inertia_fts_center_of_mass =
		inertia_fts + m_fts * a_tilde_fts.transpose() * a_tilde_fts;


	// print2 as cylinder
	Eigen::Matrix3d inertia_print2(Eigen::Matrix3d::Zero());
	inertia_print2(0, 0) = inertia_print2(1, 1) = 1. / 12. * m_print2 * (3 * 0.041 * 0.041 + 0.025 * 0.025);
	inertia_print2(2, 2) = 0.5 * m_print2 * (0.041 * 0.041);

	auto a_tilde_print2 = a_tilde(c_print2 - c);
	const Eigen::Matrix3d inertia_print2_center_of_mass =
		inertia_print2 + m_print2 * a_tilde_print2.transpose() * a_tilde_print2;


	// gripper with rotation correction
	Eigen::Matrix3d inertia_gripper(Eigen::Matrix3d::Zero());
	inertia_gripper(0, 0) = 0.0025;
	inertia_gripper(1, 1) = 0.001;
	inertia_gripper(2, 2) = 0.0017;

	inertia_gripper =
		Eigen::Matrix3d(Eigen::AngleAxisd(
			135. / 180. * std::numbers::pi, Eigen::Vector3d(0, 0, -1))).transpose()
		* inertia_gripper
		* Eigen::Matrix3d(Eigen::AngleAxisd(
			135. / 180. * std::numbers::pi, Eigen::Vector3d(0, 0, -1)));

	auto a_tilde_gripper = a_tilde(c_gripper - c);
	const Eigen::Matrix3d inertia_gripper_center_of_mass =
		inertia_gripper + m_gripper * a_tilde_gripper.transpose() * a_tilde_gripper;


	return inertia_fts_center_of_mass
		+ inertia_print2_center_of_mass
		+ inertia_gripper_center_of_mass;
}


Eigen::Matrix3d franka_proxy_util::a_tilde(const Eigen::Vector3d& a)
{
	Eigen::Matrix3d a_tilde;
	a_tilde << 0, -a.z(), a.y(),
		a.z(), 0, -a.x(),
		-a.y(), a.x(), 0;
	return a_tilde;
}




} /* namespace franka_proxy */
