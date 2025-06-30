#ifndef INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP
/**
 *************************************************************************
 *
 * @file motion_generator_admittance.hpp
 *
 ************************************************************************/

#include <vector>
#include <optional>

#include <Eigen/Core>

#include <franka/robot.h>
#include <franka/model.h>

#include <franka_proxy_share/franka_proxy_logger.hpp>

#include "motion_generator_cartesian_impedance.hpp"


namespace franka_proxy::detail
{
/**
  *************************************************************************
  *
  * @class admittance_motion_generator
  * 
  * Cartesian admittance controller. This controller is using the cartesian
  * impedance controller of the motion_generator_cartesian_impedance class
  * to reach the calculated xi positions (xi = position output of the
  * admittance controller). The required position error calculation callback
  * of the impedance controller is handled within this class and differs from
  * the 'default' one presented in the motion_generator_cartesian_impedance
  * class.
  *
  ************************************************************************/
class admittance_motion_generator
{
public:
	admittance_motion_generator(
		franka::Robot& robot,
		std::mutex& state_lock,
		franka::RobotState& robot_state,
		double duration,
		double adm_rotational_stiffness,
		double adm_translational_stiffness,
		double imp_rotational_stiffness,
		double imp_translational_stiffness,
		const std::optional<std::string>& log_file_path = std::nullopt);

	franka::Torques callback(
		const franka::RobotState& robot_state,
		franka::Duration period);

private:
	void calculate_stiffness_and_damping();

	void initialize_logging();

	static constexpr double pi = 3.14159265358979323846;
	static constexpr double deg_to_rad = pi / 180.;
	static constexpr double rad_to_deg = 180. / pi;

	franka::Model model_;

	std::mutex& state_lock_;
	franka::RobotState& state_;

	double duration_;
	double time_ = 0.0;

	std::list<std::array<double, 6>> f_exts_;

	bool initialized_ = false;
	Eigen::Matrix<double, 6, 1> x_i_1_;
	Eigen::Matrix<double, 6, 1> x_i_2_;

	Eigen::Quaterniond previous_quaternion_;

	// damping and stiffness matrix
	double translational_stiffness_ = 150.0;
	double rotational_stiffness_ = 10.0;
	Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

	// impedance controller to command new desired position
	cartesian_impedance_motion_generator impedance_controller_;

	// csv logging
	bool logging_;
	logger logger_;
	std::string log_filename_ = "admittance_additional_log.csv";

	std::list<double> timestamps_;
};
}

#endif // INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP
