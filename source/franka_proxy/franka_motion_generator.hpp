/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP

#include <atomic>

#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>
#include <vector>


namespace franka_proxy
{
namespace detail
{


/**
 * Thrown from motion_generators to terminate it.
 */
class stop_motion_trigger {};
class contact_stop_trigger {};


/**
 *************************************************************************
 *
 * @class franka_joint_motion_generator
 *
 * An example showing how to generate a joint pose motion to a goal
 * position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and
 * Control of Robots (Kogan Page Science Paper edition).
 *
 ************************************************************************/
class franka_joint_motion_generator
{
public:
	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * @param[in] speed_factor General speed factor in range [0, 1].
	 * @param[in] q_goal Target joint positions.
	 */
	franka_joint_motion_generator
		(double speed_factor,
		 std::array<double, 7> q_goal,
		 std::mutex& current_state_lock,
		 franka::RobotState& current_state,
		 const std::atomic_bool& stop_motion_flag,
		 bool stop_on_contact);

	/**
	 * Sends joint position calculations
	 *
	 * @param[in] robot_state Current state of the robot.
	 * @param[in] period Duration of execution.
	 *
	 * @return Joint positions for use inside a control loop.
	 */
	franka::JointPositions operator()
		(const franka::RobotState& robot_state,
		 franka::Duration period);

	
private:
	
	using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
	using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

	bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
	void calculateSynchronizedValues();

	static bool colliding(const franka::RobotState& state);

	static constexpr double kDeltaQMotionFinished = 1e-6;
	const Vector7d q_goal_;

	Vector7d q_start_;
	Vector7d delta_q_;

	Vector7d dq_max_sync_;
	Vector7d t_1_sync_;
	Vector7d t_2_sync_;
	Vector7d t_f_sync_;
	Vector7d q_1_;

	double time_ = 0.0;

	Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
	Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
	Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();

	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;

	const std::atomic_bool& stop_motion_;
	const bool stop_on_contact_;
};




/**
 *************************************************************************
 *
 * @class force_motion_generator
 *
 * todo
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
};



/**
 *************************************************************************
 *
 * @class cartesian_impedance_controller
 *
 * todo
 *
 ************************************************************************/
class cartesian_impedance_controller
{
public:

	cartesian_impedance_controller
		(franka::Robot& robot,
		 double translational_stiffness = 500.0,
		 double rotational_stiffness = 50.0);

	franka::Torques callback
		(const franka::RobotState& robot_state,
		 franka::Duration);

private:
	
	franka::Model model;

	franka::RobotState initial_state_;
	Eigen::Affine3d initial_transform_;
	Eigen::Vector3d position_d_;
	Eigen::Quaterniond orientation_d_;

	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	std::vector<double> forces_z{}; // debug purpose
};


/**
 *************************************************************************
 *
 * @class cartesian_impedance_and_force_controller
 *
 * todo
 *
 ************************************************************************/
class cartesian_impedance_and_force_controller
{
public:

	cartesian_impedance_and_force_controller
	(franka::Robot& robot,
		double translational_stiffness = 150.0,
		double rotational_stiffness = 10.0);

	franka::Torques callback
	(const franka::RobotState& robot_state,
		franka::Duration period);

private:

	franka::Model model;

	franka::RobotState initial_state_;
	Eigen::Affine3d initial_transform_;
	Eigen::Vector3d position_d_;
	Eigen::Quaterniond orientation_d_;

	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;


	double time_{ 0.0 };
	double desired_mass{ 0.0 };
	const double k_p{ 1.0 };
	const double k_i{ 2.0 };
	const double filter_gain{ 0.01 };


	size_t dq_current_filter_position_ = 0;
	const size_t dq_filter_size_ = 5;

	std::array<double, 7> dq_d_;
	std::vector<double> dq_buffer_;

	double target_mass;
	double duration;

	Eigen::Matrix<double, 7, 1> initial_tau_ext;
	Eigen::Matrix<double, 7, 1> tau_error_integral;

	std::vector<double> forces_z{}; // debug purpose
};



/**
 *************************************************************************
 *
 * @class sequence_joint_position_motion_generator
 *
 * todo
 *
 ************************************************************************/
class sequence_joint_position_motion_generator
{
public:
	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	sequence_joint_position_motion_generator
		(double speed_factor,
		 std::vector<std::array<double, 7>> q_sequence,
		 std::mutex& current_state_lock,
		 franka::RobotState& current_state,
		 const std::atomic_bool& stop_motion_flag);

	/**
	 * Sends joint position calculations
	 *
	 * todo doc
	 */
	franka::JointPositions operator()
		(const franka::RobotState& robot_state,
		 franka::Duration period);

	
private:
	
	using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
	using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

	const std::vector<std::array<double, 7>> q_sequence_;

	double time_ = 0.0;

	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;

	const std::atomic_bool& stop_motion_;
};




/**
 *************************************************************************
 *
 * @class sequence_joint_velocity_motion_generator
 *
 * todo
 *
 ************************************************************************/
class sequence_joint_velocity_motion_generator
{
public:
	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	sequence_joint_velocity_motion_generator
		(double speed_factor,
		 std::vector<std::array<double, 7>> q_sequence,
		 std::mutex& current_state_lock,
		 franka::RobotState& current_state,
		 const std::atomic_bool& stop_motion_flag);

	/**
	 * Sends joint position calculations
	 *
	 * todo doc
	 */
	franka::JointVelocities operator()
		(const franka::RobotState& robot_state,
		 franka::Duration period);

	
private:
	
	using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
	using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

	const std::vector<std::array<double, 7>> q_sequence_;

	double time_ = 0.0;
	double k_p_ = 5.0;

	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;

	const std::atomic_bool& stop_motion_;
};


/**
 *************************************************************************
 *
 * @class sequence_cartesian_velocity_motion_generator
 *
 * todo
 *
 ************************************************************************/
class sequence_cartesian_velocity_motion_generator
{
public:
	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	sequence_cartesian_velocity_motion_generator
	(double speed_factor,
		std::vector<std::array<double, 7>> q_sequence,
		std::mutex& current_state_lock,
		franka::Robot& robot,
		const std::atomic_bool& stop_motion_flag);

	~sequence_cartesian_velocity_motion_generator();

	/**
	 * Sends cartesian position calculations
	 *
	 * todo doc
	 */
	franka::CartesianVelocities operator()
		(const franka::RobotState& robot_state,
			franka::Duration period);


private:

	franka::Model model;

	using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
	using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

	const std::vector<std::array<double, 7>> q_sequence_;

	double time_ = 0.0;
	double k_p_ = 2.0;

	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;

	const std::atomic_bool& stop_motion_;

	std::vector<std::array<double, 6>> error_log_;
	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
};



/**
 *************************************************************************
 *
 * @class seq_cart_vel_tau_generator
 *
 * todo
 *
 ************************************************************************/
class seq_cart_vel_tau_generator
{
public:
	/**
	 * Creates a new joint_motion_generator instance for a target q.
	 *
	 * todo doc
	 */
	seq_cart_vel_tau_generator
	(double speed_factor,
		std::vector<std::array<double, 7>> q_sequence,
		std::mutex& current_state_lock,
		franka::RobotState& current_state,
		franka::Robot& robot,
		const std::atomic_bool& stop_motion_flag);

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


	using eigen_vector7d = Eigen::Matrix<double, 7, 1>;

	std::mutex& current_state_lock_;
	franka::RobotState& current_state_;
	const std::atomic_bool& stop_motion_;


	franka::Model model;


	double time_ = 0.0;
	bool log_ = true;
	const std::vector<std::array<double, 7>> q_sequence_;


	size_t dq_current_filter_position_ = 0;
	size_t dq_filter_size_ = 10;
	std::vector<eigen_vector7d> dq_buffer_;


	const double translational_stiffness_{500.0};
	const double rotational_stiffness_{50.0};
	Eigen::MatrixXd stiffness_;
	Eigen::MatrixXd damping_;

	const double target_mass{0.5};
	double desired_mass_{0.0};
	double filter_gain{0.05};
	Eigen::Matrix<double, 6, 1> force_error_integral_{Eigen::Matrix<double, 6, 1>::Zero()};

	std::vector<Eigen::Affine3d> pose_log_;
	std::vector<Eigen::Affine3d> pose_d_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> error_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_log_;
	std::vector<Eigen::Matrix<double, 6, 1>> ft_existing_log_;
};


} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */