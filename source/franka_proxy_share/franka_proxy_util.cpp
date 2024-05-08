#include "franka_proxy_util.hpp"

namespace franka_proxy {

	joint_limit::joint_limit
	(double min, double max)
		:
		min(min),
		max(max)
	{ }

	std::vector<joint_limit> franka_proxy_util::joint_limits()
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


	franka_proxy_util::robot_config_7dof franka_proxy_util::max_speed_per_joint()
	{
		return { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };
	}


	franka_proxy_util::robot_config_7dof franka_proxy_util::max_acc_per_joint()
	{
		return { 15., 7.5, 10., 12.5, 15., 20., 20. };
	}
	franka_proxy_util::robot_config_7dof franka_proxy_util::max_jerk_per_joint()
	{
		return { 7500., 3750., 5000., 6250., 7500., 10000., 10000. };
	}


	bool franka_proxy_util::is_reachable
	(const robot_config_7dof& target)
	{
		auto limits = joint_limits();

		for (int i = 0; i < 7; i++)
			if (target[i] < limits[i].min || target[i] > limits[i].max)
				return false;

		return true;
	}

} /* namespace franka_proxy */