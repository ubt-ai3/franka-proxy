/**
 *************************************************************************
 *
 * @file motion_generator_seq_cart_vel_tau.hpp
 * 
 * todo
 *
 ************************************************************************/

#pragma once


#include <atomic>
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
 * @class seq_cart_vel_tau_generator
 *
 * in use
 *
 ************************************************************************/
class seq_cart_vel_tau_generator
{
public:

	/**
	 * Thrown from motion_generators to terminate it.
	 */
	class stop_motion_trigger {};
	class contact_stop_trigger {};


	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	seq_cart_vel_tau_generator
	(std::mutex& current_state_lock,
		franka::RobotState& current_state,
		franka::Robot& robot,
		const std::atomic_bool& stop_motion_flag,
		std::vector<std::array<double, 7>> q_sequence,
		std::vector<std::array<double, 6>> f_sequence,
		std::vector<std::array<double, 6>> selection_vector_sequence);

	~seq_cart_vel_tau_generator();

	/**
	 * todo doc
	 */
	franka::Torques step
		(const franka::RobotState& robot_state,
			franka::Duration period);


private:


	void update_dq_filter(const franka::RobotState& robot_state);
	Eigen::Matrix<double, 7, 1> compute_dq_filtered();


	void update_ft_filter(const Eigen::Matrix<double, 6, 1>& current_ft);
	Eigen::Matrix<double, 6, 1> compute_ft_filtered();



	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;


	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;
	const std::atomic_bool& stop_motion_; // todo use it!


	franka::Model model;


	double time_ = 0.0;

	const std::vector<std::array<double, 7>> q_sequence_;
	const std::vector<std::array<double, 6>> f_sequence_;
	const std::vector<std::array<double, 6>> selection_vector_sequence_;


	size_t dq_current_filter_position_ = 0;
	size_t dq_filter_size_ = 10;
	std::vector<eigen_vector7d> dq_buffer_;


	size_t ft_current_filter_position_ = 0;
	size_t ft_filter_size_ = 20;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_buffer_;


	const double translational_stiffness_{3000.0};
	const double rotational_stiffness_{300.0};
	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	const double target_mass{0.0};
	double desired_mass_{0.0};
	double filter_gain{0.05};
	Eigen::Matrix<double, 6, 1> force_error_integral_{Eigen::Matrix<double, 6, 1>::Zero()};

	double f_x_error_integral_{0.0};
	double f_z_error_integral_{0.0};
	double pre_error_fz_{0.0};

	//ft_sensor_jr3 fts_;


	bool log_ = true;
	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> error_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_existing_log_;
};



/**
*************************************************************************
*
* @class seq_cart_vel_tau_generator_wo_fts
*
* in use
*
************************************************************************/
class seq_cart_vel_tau_generator_wo_fts
{
public:

	/**
	 * Thrown from motion_generators to terminate it.
	 */
	class stop_motion_trigger {};
	class contact_stop_trigger {};


	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	seq_cart_vel_tau_generator_wo_fts
	(std::mutex& current_state_lock,
		franka::RobotState& current_state,
		franka::Robot& robot,
		const std::atomic_bool& stop_motion_flag,
		std::vector<std::array<double, 7>> q_sequence,
		std::vector<std::array<double, 6>> f_sequence,
		std::vector<std::array<double, 6>> selection_vector_sequence);

	~seq_cart_vel_tau_generator_wo_fts();

	/**
	 * todo doc
	 */
	franka::Torques step
	(const franka::RobotState& robot_state,
		franka::Duration period);


private:


	void update_dq_filter(const franka::RobotState& robot_state);
	Eigen::Matrix<double, 7, 1> compute_dq_filtered();


	void update_ft_filter(const Eigen::Matrix<double, 6, 1>& current_ft);
	Eigen::Matrix<double, 6, 1> compute_ft_filtered();



	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;


	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;
	const std::atomic_bool& stop_motion_; // todo use it!


	franka::Model model;


	double time_ = 0.0;

	const std::vector<std::array<double, 7>> q_sequence_;
	const std::vector<std::array<double, 6>> f_sequence_;
	const std::vector<std::array<double, 6>> selection_vector_sequence_;


	size_t dq_current_filter_position_ = 0;
	size_t dq_filter_size_ = 10;
	std::vector<eigen_vector7d> dq_buffer_;


	size_t ft_current_filter_position_ = 0;
	size_t ft_filter_size_ = 20;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_buffer_;


	const double translational_stiffness_{ 3000.0 };
	const double rotational_stiffness_{ 300.0 };
	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	const double target_mass{ 0.0 };
	double desired_mass_{ 0.0 };
	double filter_gain{ 0.05 };
	Eigen::Matrix<double, 6, 1> force_error_integral_{ Eigen::Matrix<double, 6, 1>::Zero() };

	double f_x_error_integral_{ 0.0 };
	double f_z_error_integral_{ 0.0 };
	double pre_error_fz_{ 0.0 };


	bool log_ = true;
	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> error_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_log_;
};




} /* namespace detail */
} /* namespace franka_proxy */