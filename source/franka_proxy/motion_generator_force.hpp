/**
 *************************************************************************
 *
 * @file motion_generator_force.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP


#include <vector>

#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>
#include <numeric>

//#include <jr3_ft_sensor/force_torque_sensor.hpp>



namespace franka_proxy
{
namespace detail
{



/**
 *************************************************************************
 *
 * @class force_motion_generator
 *
 * in use
 *
 ************************************************************************/
class force_motion_generator
{

	

public:

	struct export_data {
		std::array<double, 6> k_p_f;
		std::array<double, 6> k_i_f;
		std::array<double, 6> k_d_f;
		std::array<double, 6> k_p_p;
		std::array<double, 6> k_i_p;
		std::array<double, 6> k_d_p;

		std::vector<Eigen::Matrix<double, 6, 1>> measured_positions;
		std::vector<Eigen::Matrix<double, 6, 1>> measured_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> position_errors;
		std::vector<Eigen::Matrix<double, 6, 1>> force_errors;
		std::vector<Eigen::Matrix<double, 6, 1>> position_commands;
		std::vector<Eigen::Matrix<double, 6, 1>> force_commands;

	};

	force_motion_generator
		(franka::Robot& robot, double mass, double duration);

	franka::Torques callback
		(const franka::RobotState& robot_state,
		 franka::Duration period);

	std::vector<double> give_forces();
	std::vector<double> give_desired_mass();

private:

	void update_dq_filter(const franka::RobotState& robot_state);
	double compute_dq_filtered(int j);

	double time_{0.0};
	double desired_mass{0.0};
	const double k_p{1.0};
	const double k_i{2.0};
	const double filter_gain{0.01};


	// Stiffness & Damping
	const std::array<double, 7> K_P_ = { {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0} };
	const std::array<double, 7> K_D_ = { {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0} };


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

	std::vector<double> forces_z{}; // debug purpose
	std::vector<double> des_mass{}; //debug purpose
};


// ----------- PID FORCE CONTROL MOTION GENERATOR CLASS ------------
class pid_force_control_motion_generator
{
public:

	pid_force_control_motion_generator
	(franka::Robot& robot, double mass, double duration);

	franka::Torques callback
	(const franka::RobotState& robot_state,
		franka::Duration period);

	detail::force_motion_generator::export_data get_export_data();

private:
	detail::force_motion_generator::export_data my_data;

	franka::Model model;
	franka::RobotState initial_state_;

	std::array<double, 6> k_p_f = { 1.5, 1.5, 1.5, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_i_f = { 2.0, 2.0, 2.0, 1.0, 1.0, 1.0 };
	std::array<double, 6> k_d_f = { 100, 100, 100, 50, 50, 50 };
	double target_mass;
	double duration;

	std::array<double, 6> k_p_p = { -200, -200, -200, -20, -20, -20 };
	std::array<double, 6> k_i_p = { -200, -200, -200, -20, -20, -20 };
	std::array<double, 6> k_d_p = { -500, -500, -500, -50, -50, -50 };
	
	const size_t tau_command_filter_size = 5;
	size_t tau_command_current_filter_position = 0;
	std::vector<double> tau_command_buffer;

	Eigen::Matrix<double, 6, 1> old_force_error;
	const size_t force_error_diff_filter_size = 5;
	size_t force_error_diff_current_filter_position = 0;
	std::vector<double> force_error_diff_buffer;

	Eigen::Matrix<double, 6, 1> old_position_error;
	const size_t position_error_diff_filter_size = 5;
	size_t position_error_diff_current_filter_position = 0;
	std::vector<double> position_error_diff_buffer;

	void update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command);
	double compute_tau_command_filtered(int j);

	void update_force_error_diff_filter(Eigen::Matrix<double, 6, 1> force_error_diff);
	double compute_force_error_diff_filtered(int j);

	void update_position_error_diff_filter(Eigen::Matrix<double, 6, 1> position_error_diff);
	double compute_position_error_diff_filtered(int j);

	int count_loop = 0;
	double time_{ 0.0 };

	Eigen::Matrix<double, 6, 1> force_error_integral;
	Eigen::Matrix<double, 6, 1> position_error_integral;

	Eigen::Matrix<double, 6, 1> desired_cartesian_pos;

	//initial position and rotation values are desired values for the position control
	Eigen::Vector3d position_desired;
	Eigen::Quaterniond orientation_desired;
};

} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
