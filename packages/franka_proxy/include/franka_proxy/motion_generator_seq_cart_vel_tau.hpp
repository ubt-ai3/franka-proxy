#ifndef INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_SEQ_CART_VEL_TAU_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_SEQ_CART_VEL_TAU_HPP
/**
 *************************************************************************
 *
 * @file motion_generator_seq_cart_vel_tau.hpp
 * 
 * todo
 *
 ************************************************************************/



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

	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;
	using eigen_vector6d = Eigen::Matrix<double, 6, 1>;
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
			std::vector<std::array<double, 6>> selection_vector_sequence
		);

		~seq_cart_vel_tau_generator();

		/**
		 * todo doc
		 */
		franka::Torques step
		(const franka::RobotState& robot_state,
			franka::Duration period,
			const std::array<double, 16>& offset_position ,
			const std::array<double, 6>& offset_force 
			);

		/*
		* todo doc
		*/
		franka::Torques step
		(const franka::RobotState& robot_state,
			franka::Duration period);
	


private:

	void update_dq_filter(const franka::RobotState& robot_state);
	[[nodiscard]] eigen_vector7d compute_dq_filtered() const;

	void update_ft_filter(const eigen_vector6d& current_ft);
	[[nodiscard]] eigen_vector6d compute_ft_filtered() const;

	std::array<double, 16> apply_pos_increment(const std::array<double, 16>& desired_pose,
		const std::array<double, 16>& increment);
	Eigen::Matrix<double, 6, 1> apply_force_increment(const Eigen::Matrix<double, 6, 1>& ft_desired,
		const std::array<double, 6> increment);

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
	std::vector<eigen_vector6d> ft_buffer_;


	const double translational_stiffness_{3000.0};
	const double rotational_stiffness_{300.0};
	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	const double target_mass{0.0};
	double desired_mass_{0.0};
	double filter_gain{0.05};
	eigen_vector6d force_error_integral_{eigen_vector6d::Zero()};

	double f_x_error_integral_{0.0};
	double f_z_error_integral_{0.0};
	double pre_error_fz_{0.0};

	//ft_sensor_jr3 fts_;


	bool log_ = true;
	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
	std::vector<eigen_vector6d> error_log_;
	std::vector<eigen_vector6d> ft_log_;
	std::vector<eigen_vector6d> ft_existing_log_;
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
	[[nodiscard]] eigen_vector7d compute_dq_filtered() const;


	void update_ft_filter(const eigen_vector6d& current_ft);
	[[nodiscard]] eigen_vector6d compute_ft_filtered() const;


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
	std::vector<eigen_vector6d> ft_buffer_;


	const double translational_stiffness_{ 3000.0 };
	const double rotational_stiffness_{ 300.0 };
	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	const double target_mass{ 0.0 };
	double desired_mass_{ 0.0 };
	double filter_gain{ 0.05 };
	eigen_vector6d force_error_integral_{ eigen_vector6d::Zero() };

	double f_x_error_integral_{ 0.0 };
	double f_z_error_integral_{ 0.0 };
	double pre_error_fz_{ 0.0 };


	bool log_ = true;
	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
	std::vector<eigen_vector6d> error_log_;
	std::vector<eigen_vector6d> ft_log_;
};




} /* namespace detail */
} /* namespace franka_proxy */
#endif // INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_SEQ_CART_VEL_TAU_HPP
