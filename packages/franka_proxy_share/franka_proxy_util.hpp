/*********************************
* 
* @file franka_proxy_util.hpp
* 
* Static utilities
* 
*********************************/

#pragma once

#include <vector>
#include <Eigen/Geometry>

namespace franka_proxy
{

	struct joint_limit
	{
		joint_limit(double min, double max);

		double min;
		double max;
	};

	class franka_proxy_util {

	using robot_config_7dof = Eigen::Matrix<double, 7, 1>;
	
	public:
		static std::vector<joint_limit> joint_limits();
		static robot_config_7dof max_speed_per_joint();
		static robot_config_7dof max_acc_per_joint();
		static robot_config_7dof max_jerk_per_joint();


		static bool is_reachable(const robot_config_7dof& target);
	};
}
/* namespace franka_proxy*/