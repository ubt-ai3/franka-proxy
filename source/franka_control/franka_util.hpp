/**
 *************************************************************************
 *
 * @file franka_util.hpp
 *
 * Static franka utilities.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__FRANKA_UTIL_HPP)
#define INCLUDED__FRANKA_CONTROL__FRANKA_UTIL_HPP


#include <vector>

#include <Eigen/Geometry>

#include "franka_controller.hpp"


namespace franka_control
{


struct joint_limit
{
	joint_limit
		(double min, double max);

	double min;
	double max;
};


/**
 *************************************************************************
 *
 * @class franka_util
 *
 * Static franka utilities.
 *
 ************************************************************************/
class franka_util
{
public:

	static std::vector<joint_limit> joint_limits();
	static robot_config_7dof max_speed_per_joint();
	static robot_config_7dof max_acc_per_joint();
	static robot_config_7dof max_jerk_per_joint();


	static bool is_reachable
		(const robot_config_7dof& target);


	static std::vector<Eigen::Affine3d> fk
		(const robot_config_7dof& configuration);


	static std::vector<robot_config_7dof> ik_fast
		(const Eigen::Affine3d& world_T_nsa, double joint_4_value = 0.);
	
	static std::vector<robot_config_7dof> ik_fast_robust
		(const Eigen::Affine3d& world_T_nsa, double step_size = 0.174533);

	static robot_config_7dof ik_fast_closest
		(const Eigen::Affine3d& target_world_T_nsa,
		 const robot_config_7dof& current_configuration,
		 double step_size = 0.174533);


	/**
	 * Helper functions to calculate tool mass, mass center and inertia tensor.
	 * Settings must be changed in franka web interface.
	 */
	static double tool_mass();
	static Eigen::Vector3d tool_center_of_mass();
	static Eigen::Matrix3d tool_inertia();


	static const joint_limit joint_limits_[];


	static constexpr double pi = 3.14159265358979323846;
	static constexpr double deg_to_rad = pi / 180;
	static constexpr double rad_to_deg = 180 / pi;


private:
	
	static Eigen::Matrix3d a_tilde(const Eigen::Vector3d& a);
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_UTIL_HPP) */
