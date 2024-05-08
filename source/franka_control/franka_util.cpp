/**
 *************************************************************************
 *
 * @file franka_util.cpp
 *
 * Static franka utilities, implementation.
 *
 ************************************************************************/

#include "franka_util.hpp"

#include <iostream>

#include "exception.hpp"
#include "ikfast.h"


namespace franka_control
{

//////////////////////////////////////////////////////////////////////////
//
// franka_util
//
//////////////////////////////////////////////////////////////////////////


std::vector<joint_limit> franka_util::joint_limits()
{
	return franka_proxy::franka_proxy_util::joint_limits();
}


robot_config_7dof franka_util::max_speed_per_joint()
{
	return franka_proxy::franka_proxy_util::max_speed_per_joint();
}


robot_config_7dof franka_util::max_acc_per_joint()
{
	return franka_proxy::franka_proxy_util::max_acc_per_joint();
}
robot_config_7dof franka_util::max_jerk_per_joint()
{
	return franka_proxy::franka_proxy_util::max_speed_per_joint();
}


bool franka_util::is_reachable
	(const robot_config_7dof& target)
{
	return franka_proxy::franka_proxy_util::is_reachable(target);
}

std::vector<Eigen::Affine3d> franka_util::fk
	(const robot_config_7dof& configuration)
{
	std::vector<Eigen::Affine3d> frames;

	// initialize transformation matrix for FK
	Eigen::Affine3d trafo(Eigen::Affine3d::Identity());
	// trafo *= Eigen::AngleAxisd(3.1415, Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	// link 0 (just a default trafo)
	frames.push_back(trafo);

	// link 1 (translation to parent frame)
	trafo *= Eigen::Translation3d(0, 0, 0.333f);
	// joint angle
	trafo *= Eigen::AngleAxisd(configuration[0], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 2 (rotation to parent frame)
	trafo *= Eigen::AngleAxisd(-pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[1], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 3 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0, -0.316f, 0.0f);
	trafo *= Eigen::AngleAxisd(pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[2], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 4 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0.0825f, 0.0f, 0.0f);
	trafo *= Eigen::AngleAxisd(pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[3], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 5 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(-0.0825f, 0.384f, 0.0f);
	trafo *= Eigen::AngleAxisd(-pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[4], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 6 (rotation to parent frame)
	trafo *= Eigen::AngleAxisd(pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[5], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 7 (rotation to parent frame)
	trafo *= Eigen::Translation3d(0.088f, 0.0f, 0.0f);
	trafo *= Eigen::AngleAxisd(pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[6], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	//// link 7 to flange
	//trafo *= Eigen::Translation3d(0.0f, 0.0f, 0.107);
	//frames.push_back(trafo);

	//// flange to standard end effector
	//trafo *= Eigen::Translation3d(0.0f, 0.0f, 0.1034f);
	//trafo *= Eigen::AngleAxisd(pi / 4., Eigen::Vector3d(0.0f, 0.0f, -1.0f));
	//frames.push_back(trafo);

	return frames;
}


std::vector<robot_config_7dof> franka_util::ik_fast
	(const Eigen::Affine3d& target_world_T_j7, double joint_4_value)
{
	const Eigen::Affine3d last_segment_T_tcp
		(Eigen::Translation3d(0.f, 0.f, 0.107f));

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
	if (ComputeIk(translation, rotation, &free[0], solutions))
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


std::vector<robot_config_7dof> franka_util::ik_fast_robust
	(const Eigen::Affine3d& target_world_T_j7, double step_size)
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


robot_config_7dof franka_util::ik_fast_closest
	(const Eigen::Affine3d& target_world_T_j7,
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
		std::cerr << "No solution found." << std::endl;
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


double franka_util::tool_mass()
{
	const double m_print1 = 0.01,
		m_fts = 0.372,
		m_print2 = 0.029,
		m_gripper = 0.73,
		m_camera = 0.09;

	return m_print1 + m_fts + m_print2 + m_gripper + m_camera;
}


Eigen::Vector3d franka_util::tool_center_of_mass()
{
	const double
		m_print1 = 0.01,
		m_fts = 0.372,
		m_print2 = 0.029,
		m_gripper = 0.73,
		m_camera = 0.09;

	const Eigen::Vector3d
		c_print1(0, 0, 0.001),
		c_fts(0, 0, 0.0175),
		c_print2(0, 0, 0.043),
		c_gripper(-0.007, -0.007, 0.083),
		c_camera(-0.028, 0.028, 0.103);

	return m_print1 * c_print1 + m_fts * c_fts + m_print2 * c_print2 + m_gripper * c_gripper + m_camera * c_camera;
}


Eigen::Matrix3d franka_util::tool_inertia()
{
	const double
		m_print1 = 0.01,
		m_fts = 0.372,
		m_print2 = 0.029,
		m_gripper = 0.73,
		m_camera = 0.09;

	const Eigen::Vector3d
		c_print1(0, 0, 0.001),
		c_fts(0, 0, 0.0175),
		c_print2(0, 0, 0.043),
		c_gripper(-0.00707107, -0.00707107, 0.083),
		c_camera(-0.028, 0.028, 0.103);

	const Eigen::Vector3d c(tool_center_of_mass());


	// print1 as cylinder
	Eigen::Matrix3d inertia_print1(Eigen::Matrix3d::Zero());
	inertia_print1(0, 0) = inertia_print1(1, 1) = 1 / 12. * m_print1 * (3 * 0.0315 * 0.0315 + 0.002 * 0.002);
	inertia_print1(2, 2) = 0.5 * m_print1 * (0.0315 * 0.0315);

	auto a_tilde_print1 = a_tilde(c_print1 - c);
	const Eigen::Matrix3d inertia_print1_center_of_mass =
		inertia_print1 + m_print1 * a_tilde_print1.transpose() * a_tilde_print1;
	
	
	// fts as thick-walled cylindrical tube
	Eigen::Matrix3d inertia_fts(Eigen::Matrix3d::Zero());
	inertia_fts(0, 0) = inertia_fts(1, 1) = 1 / 12. * m_fts * (3 * (0.04445 * 0.04445 + 0.015 * 0.015) + 0.031 * 0.031);
	inertia_fts(2, 2) = 0.5 * m_fts * (0.04445 * 0.04445 + 0.015 * 0.015);
	
	auto a_tilde_fts = a_tilde(c_fts - c);
	const Eigen::Matrix3d inertia_fts_center_of_mass =
		inertia_fts + m_fts * a_tilde_fts.transpose() * a_tilde_fts;


	// print2 as cylinder
	Eigen::Matrix3d inertia_print2(Eigen::Matrix3d::Zero());
	inertia_print2(0, 0) = inertia_print2(1, 1) = 1 / 12. * m_print2 * (3 * 0.0315 * 0.0315 + 0.02 * 0.02);
	inertia_print2(2, 2) = 0.5 * m_print2 * (0.0315 * 0.0315);

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
			135. / 180. * pi, Eigen::Vector3d(0, 0, 1))).transpose()
		* inertia_gripper
		* Eigen::Matrix3d(Eigen::AngleAxisd(
			135. / 180. * pi, Eigen::Vector3d(0, 0, 1)));
	
	auto a_tilde_gripper = a_tilde(c_gripper - c);
	const Eigen::Matrix3d inertia_gripper_center_of_mass =
		inertia_gripper + m_gripper * a_tilde_gripper.transpose() * a_tilde_gripper;


	// camera as solid cuboid (10 x 3 x 3)
	Eigen::Matrix3d inertia_camera(Eigen::Matrix3d::Zero());
	inertia_camera(0, 0) = 1 / 12. * m_camera * (0.1 * 0.1 + 0.03 * 0.03);
	inertia_camera(1, 1) = 1 / 12. * m_camera * (0.03 * 0.03 + 0.03 * 0.03);
	inertia_camera(2, 2) = 1 / 12. * m_camera * (0.03 * 0.03 + 0.1 * 0.1);

	inertia_camera =
		Eigen::Matrix3d(Eigen::AngleAxisd(
			135. / 180. * pi, Eigen::Vector3d(0, 0, 1))).transpose()
		* inertia_camera
		* Eigen::Matrix3d(Eigen::AngleAxisd(
			135. / 180. * pi, Eigen::Vector3d(0, 0, 1)));

	auto a_tilde_camera = a_tilde(c_camera - c);
	const Eigen::Matrix3d inertia_camera_center_of_mass =
		inertia_camera + m_camera * a_tilde_camera.transpose() * a_tilde_camera;
	

	return inertia_print1_center_of_mass
		+ inertia_fts_center_of_mass
		+ inertia_print2_center_of_mass
		+ inertia_gripper_center_of_mass
		+ inertia_camera_center_of_mass;
}


Eigen::Matrix3d franka_util::a_tilde(const Eigen::Vector3d& a)
{
	Eigen::Matrix3d a_tilde(Eigen::Matrix3d::Zero());
	
	a_tilde(1, 0) = a(2);
	a_tilde(2, 0) = -a(1);
	a_tilde(0, 1) = -a(2);
	a_tilde(2, 1) = a(0);
	a_tilde(0, 2) = a(1);
	a_tilde(1, 2) = -a(0);
	
	return a_tilde;
}


const joint_limit franka_util::joint_limits_[] =
{
	{-2.8973, 2.8973},
	{-1.7628, 1.7628},
	{-2.8973, 2.8973},
	{-3.0718, -0.069},
	{-2.8973, 2.8973},
	{-0.0175, 3.7525},
	{-2.8973, 2.8973}
};




} /* namespace franka_control */