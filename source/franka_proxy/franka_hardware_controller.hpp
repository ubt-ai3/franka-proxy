/**
 *************************************************************************
 *
 * @file franka_hardware_controller.hpp
 *
 * Classes to control a franka emika panda robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_KONTROLLER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_KONTROLLER_HPP


#include <atomic>
#include <string>

#include <franka/robot.h>
#include <franka/gripper.h>

#include <Eigen/Core>

#include <viral_core/thread_synch.hpp>


namespace franka_proxy
{


typedef std::array<double, 7> robot_config_7dof;


class franka_hardware_controller
{

public:

	franka_hardware_controller
		(const std::string& controller_ip);

	virtual ~franka_hardware_controller() noexcept;


	/**
	 * Moves the Panda robot to given target; In case
	 * of collision, the movement is retried and continued 
	 * after automatic error recovery
	 */
	void move_to(const robot_config_7dof& target);
	void stop_movement();

	void set_speed_factor(double speed_factor);

	franka::RobotState robot_state() const;


	/** Move the gripper to gripper::max_width. */
	void open_gripper();
	/** Grasp.... */
	void close_gripper(double speed, double force);

	franka::GripperState gripper_state() const;


	void automatic_error_recovery();


private:

	/**
	 * Used to update the current robot state while no control loop is
	 * running.
	 */
	void state_update_loop();

	
	/**
	 * Initialize parameters such as joint impedance and collision behavior.
	 */
	void initialize_parameters();


	// Robot
	mutable franka::Robot robot_;
	bool parameters_initialized_;

	std::atomic_bool stop_motion_;
	
	mutable std::mutex speed_factor_lock_;
	double speed_factor_;


	// Gripper
	mutable franka::Gripper gripper_;
	const double max_width_;

	static constexpr double gripper_speed = 0.025;
	static constexpr double open_epsilon = 0.1;
	static constexpr double min_grasp_width = 0.003;


	mutable std::mutex state_lock_;
	franka::RobotState robot_state_;
	franka::GripperState gripper_state_;

	viral_core::signal control_loop_running_;
	std::atomic_bool terminate_state_thread_;
	std::thread state_thread_;


			/**
			 * Thrown from MotionGenerator to terminate it.
			 */
			class stop_motion_trigger {};

			/**
			 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
			 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
			 * (Kogan Page Science Paper edition).
			 */
			class MotionGenerator {
			public:
				/**
				 * Creates a new MotionGenerator instance for a target q.
				 *
				 * @param[in] speed_factor General speed factor in range [0, 1].
				 * @param[in] q_goal Target joint positions.
				 */
				MotionGenerator
					(double speed_factor, const std::array<double, 7> q_goal,
					 std::mutex& current_state_lock, franka::RobotState& current_state,
					 const std::atomic_bool& stop_motion_flag);

				/**
				 * Sends joint position calculations
				 *
				 * @param[in] robot_state Current state of the robot.
				 * @param[in] period Duration of execution.
				 *
				 * @return Joint positions for use inside a control loop.
				 */
				franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

			private:
				using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
				using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

				bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
				void calculateSynchronizedValues();

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
			};


};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_HARDWARE_KONTROLLER_HPP) */