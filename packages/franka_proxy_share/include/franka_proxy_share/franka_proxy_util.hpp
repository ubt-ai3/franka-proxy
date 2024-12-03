/*********************************
* 
* @file franka_proxy_util.hpp
* 
* Static utilities
* 
*********************************/

#pragma once

#include <vector>
#include <numbers>
#include <Eigen/Geometry>

namespace franka_proxy
{
	struct joint_limit
	{
		joint_limit(double min, double max);

		double min;
		double max;
	};

	class franka_proxy_util
	{
	public:

		typedef Eigen::Vector<double, 7> robot_config_7dof;

		static std::vector<joint_limit> joint_limits();
		static robot_config_7dof max_speed_per_joint();
		static robot_config_7dof max_acc_per_joint();
		static robot_config_7dof max_jerk_per_joint();


		static bool is_reachable(const robot_config_7dof& target);

		/**
		* For coordinate frames see:
		* https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
		*/
		static std::vector<Eigen::Affine3d> fk(
			const robot_config_7dof& configuration);


		static std::vector<robot_config_7dof> ik_fast(
			const Eigen::Affine3d& target_world_T_j7,
			double joint_4_value = 0.);

		static std::vector<Eigen::Affine3d> fk(
			const std::array<double, 7>& configuration);

		static std::vector<robot_config_7dof> ik_fast_robust(
			const Eigen::Affine3d& target_world_T_j7,
			double step_size = 0.174533);

		static robot_config_7dof ik_fast_closest(
			const Eigen::Affine3d& target_world_T_j7,
			const robot_config_7dof& current_configuration,
			double step_size = 0.174533);

		template<typename T, size_t N>
		static Eigen::Vector<T, N> convert_to_eigen(const std::array<T, N>& std_array)
		{
			Eigen::Vector<T, N> out = Eigen::Map<const Eigen::Vector<T, N>>(std_array.data());
			return out;
		}

		template<typename T, size_t N>
		static std::array<T, N> convert_to_std_array(const Eigen::Vector<T, N>& eigen_array)
		{
			std::array<T, N> out;
			for (int i = 0; i < N; i++)
				out[i] = eigen_array(i, 0);

			return out;
		}

		/**
		 * Helper functions to calculate tool mass, mass center and inertia tensor.
		 * Settings must be changed in franka web interface.
		 */
		static double tool_mass();
		static Eigen::Vector3d tool_center_of_mass();
		static Eigen::Matrix3d tool_inertia();


		static const joint_limit joint_limits_[];

		static constexpr double deg_to_rad = std::numbers::pi / 180.;
		static constexpr double rad_to_deg = 180. / std::numbers::pi;

	private:

		static Eigen::Matrix3d a_tilde(const Eigen::Vector3d& a);
	};
}
/* namespace franka_proxy*/