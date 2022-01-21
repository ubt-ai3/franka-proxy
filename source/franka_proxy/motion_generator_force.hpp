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
		double k_p;
		double k_i;
		double k_d;

		double duration;

		std::vector<Eigen::Matrix<double, 6, 7>> jacobi;


		Eigen::Matrix<double, 6, 1> masses; //the mass that should be achieved in each time step

		std::vector<Eigen::Matrix<double, 6, 1>> existing_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> command_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> position_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> hybrid_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> test_command_forces;

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
	(franka::Robot& robot, double mass, double duration, double k_p, double k_i, double k_d);

	franka::Torques callback
	(const franka::RobotState& robot_state,
		franka::Duration period);

	detail::force_motion_generator::export_data get_export_data();

private:
	detail::force_motion_generator::export_data my_data;

	franka::Model model;
	franka::RobotState initial_state_;

	double k_p;
	double k_i;
	double k_d;
	double target_mass;
	double duration;
	
	int count_loop = 0;
	double time_{ 0.0 };

	Eigen::Matrix<double, 6, 1> force_error_integral;

	Eigen::Matrix<double, 6, 1> desired_cartesian_pos;
};

} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
