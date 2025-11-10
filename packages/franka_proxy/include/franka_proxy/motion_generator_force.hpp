#ifndef INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_FORCE_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_FORCE_HPP
/**
 *************************************************************************
 *
 * @file motion_generator_force.hpp
 * 
 * Heavily inspired by franka emika example code
 *
 ************************************************************************/


#include <vector>

#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>


namespace franka_proxy::detail
{
/**
 *************************************************************************
 *
 * @class force_motion_generator
 *
 ************************************************************************/
class force_motion_generator
{
public:
	force_motion_generator(
		franka::Robot& robot, double mass, double duration);

	franka::Torques callback(
		const franka::RobotState& robot_state,
		franka::Duration period);

	std::vector<double> get_resulting_forces_z_log();
	std::vector<double> get_desired_forces_log();

private:
	void update_dq_filter(const franka::RobotState& robot_state);
	double compute_dq_filtered(int j) const;

	double time_{0.0};
	double desired_mass{0.0};
	const double k_p_{1.0};
	const double k_i_{2.0};
	const double filter_gain{0.01};


	// Stiffness & Damping
	const std::array<double, 7> K_P_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
	const std::array<double, 7> K_D_ = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};


	size_t dq_current_filter_position_ = 0;
	const size_t dq_filter_size_ = 5;

	std::array<double, 7> dq_d_;
	std::vector<double> dq_buffer_;

	double target_mass;
	double duration;

	franka::Model model;

	Eigen::Matrix<double, 7, 1> initial_tau_ext;
	Eigen::Matrix<double, 7, 1> tau_error_integral;

	franka::RobotState initial_state_;

	std::vector<double> resulting_forces_z_{}; // debug purposes
	std::vector<double> desired_forces_{}; // debug purposes
};
}

#endif // INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_FORCE_HPP
