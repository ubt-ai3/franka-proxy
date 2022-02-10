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


// ----------- HYBRID CONTROL MOTION GENERATOR CLASS ------------
class hybrid_control_motion_generator
{
public:

	hybrid_control_motion_generator
	(franka::Robot& robot, double mass, double duration);

	franka::Torques callback
	(const franka::RobotState& robot_state,
		franka::Duration period);


private:
	franka::Model model_;
	franka::RobotState initial_state_;

	int count_loop_ = 0;
	double time_{ 0.0 };
	double target_mass_;
	double duration_;

	std::array<double, 6> k_p_f_ = { 1.5, 1.5, 1.5, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_i_f_ = { 2.0, 2.0, 2.0, 1.0, 1.0, 1.0 };
	std::array<double, 6> k_d_f_ = { 100, 100, 100, 50, 50, 50 };

	std::array<double, 6> k_p_p_ = { -200, -200, -200, -20, -20, -20 };
	std::array<double, 6> k_i_p_ = { -200, -200, -200, -20, -20, -20 };
	std::array<double, 6> k_d_p_ = { -500, -500, -500, -50, -50, -50 };
	
	const size_t tau_command_filter_size_ = 5;
	size_t tau_command_current_filter_position_ = 0;
	std::vector<double> tau_command_buffer_;

	Eigen::Matrix<double, 6, 1> old_force_error_;
	const size_t force_error_diff_filter_size_ = 5;
	size_t force_error_diff_current_filter_position_ = 0;
	std::vector<double> force_error_diff_buffer_;

	Eigen::Matrix<double, 6, 1> old_position_error_;
	const size_t position_error_diff_filter_size_ = 5;
	size_t position_error_diff_current_filter_position_ = 0;
	std::vector<double> position_error_diff_buffer_;

	Eigen::Matrix<double, 6, 1> force_error_integral_;
	Eigen::Matrix<double, 6, 1> position_error_integral_;

	Eigen::Matrix<double, 6, 1> desired_cartesian_pos_;

	//initial position and rotation values are desired values for the position control
	Eigen::Vector3d position_desired_;
	Eigen::Quaterniond orientation_desired_;


	void update_tau_command_filter(Eigen::Matrix<double, 7, 1> tau_command);
	double compute_tau_command_filtered(int j);

	void update_force_error_diff_filter(Eigen::Matrix<double, 6, 1> force_error_diff);
	double compute_force_error_diff_filtered(int j);

	void update_position_error_diff_filter(Eigen::Matrix<double, 6, 1> position_error_diff);
	double compute_position_error_diff_filtered(int j);

	//Spring Damping System Position related
	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;

	const std::vector<std::array<double, 7>> q_sequence_;

	void update_dq_filter(const franka::RobotState& robot_state);
	Eigen::Matrix<double, 7, 1> compute_dq_filtered();

	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;
	const double translational_stiffness_{ 1000.0 };
	const double rotational_stiffness_{ 100.0 };

	size_t dq_current_filter_position_ = 0;
	size_t dq_filter_size_ = 10;
	std::vector<eigen_vector7d> dq_buffer_;
};

} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
