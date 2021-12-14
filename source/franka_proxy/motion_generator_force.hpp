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

		//todo: check if columns can be initialized static with the use of the duration and the 1kHz frequenzy

		Eigen::Matrix<double, 6, Eigen::Dynamic> masses; //the mass that should be achieved in each time step
		std::vector<std::array<double, 6>> measured_forces;
		std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
		Eigen::Matrix<double, 6, Eigen::Dynamic> control_forces; //the calculated value - output from the pid-control
		Eigen::Matrix<double, 6, Eigen::Dynamic> error_integrals; //input for the i control - this value gets multiplied by k_i
		Eigen::Matrix<double, 6, Eigen::Dynamic> error_differentials; //input for the d control - this value gets multiplied by k_d

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

	std::vector<std::array<double, 6>> give_measured_forces();
	std::vector<Eigen::Matrix<double, 6, 1>> give_desired_forces();

private:

	void update_dq_filter(const franka::RobotState& robot_state);
	double compute_dq_filtered(int j);

	double time_{ 0.0 };


	// Stiffness & Damping
	const std::array<double, 7> K_P_ = { {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0} };
	const std::array<double, 7> K_D_ = { {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0} };


	size_t dq_current_filter_position_ = 0;
	const size_t dq_filter_size_ = 5;

	std::array<double, 7> dq_d_;
	std::vector<double> dq_buffer_;

	double k_p;
	double k_i;
	double k_d;
	double target_mass;
	double duration;

	//------------
	Eigen::VectorXd tau_new_error;
	Eigen::VectorXd tau_old_error;
	std::vector<std::array<double, 6>> measured_forces; //get über public get_measured_forces()
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;// get über public get_desired_forces()
	std::vector<Eigen::Matrix<double, 7, 1>> calculated_forces;
	std::vector<Eigen::Matrix<double, 6, 1>> error_integrals;
	std::vector<Eigen::Matrix<double, 6, 1>> error_differentials;

	//--------------

	franka::Model model;

	Eigen::Matrix<double, 7, 1> initial_tau_ext;
	Eigen::Matrix<double, 7, 1> tau_error_integral;

	franka::RobotState initial_state_;

	std::vector<double> forces_z{}; // debug purpose
	std::vector<double> des_mass{}; //debug purpose
};




} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
