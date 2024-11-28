/**
 *************************************************************************
 *
 * @file motion_generator_force.hpp
 * 
 * todo
 *
 ************************************************************************/

#pragma once


#include <vector>

#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>



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
		
	force_motion_generator
		(franka::Robot& robot, double mass, double duration);

	force_motion_generator(
		franka::Robot& robot,
		double mass,
		double duration,
		const std::array<double, 6>& offset_force);

	franka::Torques callback
		(const franka::RobotState& robot_state,
		 franka::Duration period);

	std::vector<double> give_forces();
	std::vector<double> give_desired_mass();

private:

	void update_dq_filter(const franka::RobotState& robot_state);
	[[nodiscard]] double compute_dq_filtered(int j) const;

	double time_{0.0};
	double desired_mass{0.0};
	const double k_p{1.0};
	const double k_i{2.0};
	const double filter_gain{0.01};

	const std::array<double, 6> force_offset_{};

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


	// @todo work of Laurin Hecken
	/*
// ----------- HYBRID CONTROL MOTION GENERATOR CLASS ------------
class hybrid_control_motion_generator
{
public:

	hybrid_control_motion_generator
	(franka::Robot& robot,
		std::vector<Eigen::Vector3d> desired_positions,
		std::vector<Eigen::Matrix<double, 6, 1>> desired_forces, 
		std::vector<Eigen::Quaterniond> desired_orientations,
		std::array<std::array<double, 6>, 6> control_parameters,
		csv_data &data);

	franka::Torques callback
	(const franka::RobotState& robot_state,
		franka::Duration period);


private:
	franka::Model model_;
	franka::RobotState initial_state_;

	int count_loop_ = 0;
	double time_{ 0.0 };

	std::array<double, 6> k_p_p_;
	std::array<double, 6> k_i_p_;
	std::array<double, 6> k_d_p_;

	std::array<double, 6> k_p_f_;
	std::array<double, 6> k_i_f_;
	std::array<double, 6> k_d_f_;

	const size_t tau_command_filter_size_ = 5;
	size_t tau_command_current_filter_position_ = 0;
	std::vector<double> tau_command_buffer_;

	Eigen::Matrix<double, 6, 1> old_force_error_;
	const size_t force_error_diff_filter_size_ = 50;
	size_t force_error_diff_current_filter_position_ = 0;
	std::vector<double> force_error_diff_buffer_;

	Eigen::Matrix<double, 6, 1> old_position_error_;
	const size_t position_error_diff_filter_size_ = 50;
	size_t position_error_diff_current_filter_position_ = 0;
	std::vector<double> position_error_diff_buffer_;

	Eigen::Matrix<double, 6, 1> force_error_integral_;
	Eigen::Matrix<double, 6, 1> position_error_integral_;

	Eigen::Matrix<double, 6, 1> desired_cartesian_pos_;

	//current desired values are extracted from the value vectors
	Eigen::Vector3d position_desired_;
	Eigen::Quaterniond orientation_desired_;

	//Desired value vectors
	std::vector<Eigen::Vector3d> desired_positions_;
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces_;
	std::vector<Eigen::Quaterniond> desired_orientations_;


	void update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command);
	double compute_tau_command_filtered(int j);

	void update_force_error_diff_filter(Eigen::Matrix<double, 6, 1> force_error_diff);
	double compute_force_error_diff_filtered(int j);

	void update_position_error_diff_filter(Eigen::Matrix<double, 6, 1> position_error_diff);
	double compute_position_error_diff_filtered(int j);

	Eigen::Matrix<double, 6, 1> pid_force_control(double period, Eigen::Matrix<double, 6, 1> force_existing);
	Eigen::Matrix<double, 6, 1> pid_position_control(double period, Eigen::Affine3d transform);
	Eigen::Matrix<double, 6, 1> formatting_position(Eigen::Vector3d position, Eigen::Quaterniond orientation);

	//Spring Damping System Position related
	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;

	const std::vector<std::array<double, 7>> q_sequence_;

	void update_dq_filter(const franka::RobotState& robot_state);
	Eigen::Matrix<double, 7, 1> compute_dq_filtered();

	size_t dq_current_filter_position_ = 0;
	size_t dq_filter_size_ = 10;
	std::vector<eigen_vector7d> dq_buffer_;

	csv_data &data_;
};
*/

} /* namespace detail */
} /* namespace franka_proxy */
