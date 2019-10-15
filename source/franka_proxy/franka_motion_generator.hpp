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

#include <Eigen/Core>

#include <franka/robot.h>
#include <franka/model.h>


namespace franka_proxy
{
namespace detail
{


/**
 * Thrown from motion_generator to terminate it.
 */
class stop_motion_trigger{};
class contact_stop_trigger{};


/**
 *************************************************************************
 *
 * @class motion_generator
 *
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 *
 ************************************************************************/
class motion_generator
{
public:
	/**
	 * Creates a new motion_generator instance for a target q.
	 *
	 * @param[in] speed_factor General speed factor in range [0, 1].
	 * @param[in] q_goal Target joint positions.
	 */
	motion_generator
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
	franka::JointPositions operator()(
		const franka::RobotState& robot_state,
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

	force_motion_generator(	franka::Robot& robot,double mass, double duration);

	franka::Torques callback(
		const franka::RobotState& robot_state,
		franka::Duration period);

private:

	Eigen::Vector3d get_position(const franka::RobotState& robot_state);;


	double time_{0.0};
	double k_p{1.0};
	double k_i{2.0};
	double filter_gain{0.01};
	double desired_mass{0.0};

	franka::Model model;

	double target_mass;
	double duration;

	std::array<double, 7> gravity_array;
	Eigen::Matrix<double, 7, 1> initial_tau_ext;
	Eigen::Matrix<double, 7, 1> tau_error_integral;

	Eigen::Vector3d initial_position;
};


} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
