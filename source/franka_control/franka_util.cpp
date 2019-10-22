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

#include <viral_core/geo_util.hpp>

#include "ikfast.h"
#include "exception.hpp"


namespace franka_control
{


//////////////////////////////////////////////////////////////////////////
//
// joint_limit
//
//////////////////////////////////////////////////////////////////////////


joint_limit::joint_limit
	(double min, double max)
	:
	min(min),
	max(max)
{ }




//////////////////////////////////////////////////////////////////////////
//
// franka_util
//
//////////////////////////////////////////////////////////////////////////


std::vector<joint_limit> franka_util::joint_limits()
{
	std::vector<joint_limit> limits;

	limits.emplace_back(-2.8973, 2.8973);
	limits.emplace_back(-1.7628, 1.7628);
	limits.emplace_back(-2.8973, 2.8973);
	limits.emplace_back(-3.0718, -0.0698);
	limits.emplace_back(-2.8973, 2.8973);
	limits.emplace_back(-0.0175, 3.7525);
	limits.emplace_back(-2.8973, 2.8973);

	return limits;
}


robot_config_7dof franka_util::max_speed_per_joint()
{
	return
		(robot_config_7dof() << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100).finished();
}


robot_config_7dof franka_util::max_acc_per_joint()
{
	return
		(robot_config_7dof() << 15., 7.5, 10., 12.5, 15., 20., 20.).finished();
}
robot_config_7dof franka_util::max_jerk_per_joint()
{
	return
		(robot_config_7dof() << 7500., 3750., 5000., 6250., 7500., 10000., 10000.).finished();
}


bool franka_util::is_reachable
	(const robot_config_7dof& target)
{
	auto limits = joint_limits();

	for (int i = 0; i < 7; i++)
		if (target[i] < limits[i].min || target[i] > limits[i].max)
			return false;

	return true;
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
	trafo *= Eigen::AngleAxisd(-viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[1], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 3 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0, -0.316f, 0.0f);
	trafo *= Eigen::AngleAxisd(viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[2], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 4 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(0.0825f, 0.0f, 0.0f);
	trafo *= Eigen::AngleAxisd(viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[3], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 5 (translation and rotation to parent frame)
	trafo *= Eigen::Translation3d(-0.0825f, 0.384f, 0.0f);
	trafo *= Eigen::AngleAxisd(-viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[4], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 6 (rotation to parent frame)
	trafo *= Eigen::AngleAxisd(viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[5], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	// link 7 (rotation to parent frame)
	trafo *= Eigen::Translation3d(0.088f, 0.0f, 0.0f);
	trafo *= Eigen::AngleAxisd(viral_core::geo_constants::pi / 2., Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	// link rotation
	trafo *= Eigen::AngleAxisd(configuration[6], Eigen::Vector3d(0.0f, 0.0f, 1.0f));
	frames.push_back(trafo);

	return frames;
}


std::vector<robot_config_7dof> franka_util::ik_fast
	(const Eigen::Affine3d& world_T_nsa, double joint_4_value)
{
	const Eigen::Affine3d last_segment_T_tcp
		(Eigen::Translation3d(0.f, 0.f, 0.107f));

	Eigen::Affine3d target
		(world_T_nsa * last_segment_T_tcp);

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
	(const Eigen::Affine3d& world_T_nsa, double stepsize)
{
	std::vector<robot_config_7dof> solutions = ik_fast(world_T_nsa);
	double joint_4 = joint_limits_[4].min;
	while (solutions.empty() && joint_4 < joint_limits_[4].max)
	{
		solutions = ik_fast(world_T_nsa, joint_4);
		joint_4 += stepsize;
	}
	return solutions;
}


robot_config_7dof franka_util::ik_fast_closest
	(const Eigen::Affine3d& target_world_T_nsa,
	 const robot_config_7dof& current_configuration,
	 double stepsize)
{
	// Calculate possible solutions
	std::vector<robot_config_7dof> solutions;
	for (double joint_4 = joint_limits_[4].min; joint_4 < joint_limits_[4].max; joint_4 += stepsize)
	{
		std::vector<robot_config_7dof> new_solutions =
			ik_fast(target_world_T_nsa, joint_4);
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