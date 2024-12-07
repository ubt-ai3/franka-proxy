#ifndef INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP
/**
 *************************************************************************
 *
 * @file motion_generator_joint_impedance.hpp
 *
 * todo
 *
 ************************************************************************/



#include <vector>
#include <optional>

#include <Eigen/Core>

#include <franka/robot.h>
#include <franka/model.h>

#include <franka_proxy_share/franka_proxy_logger.hpp>



namespace franka_proxy
{
	namespace detail
	{
		/**
		 *************************************************************************
		 *
		 * @class joint_impedance_motion_generator
		 *
		 * in use
		 * 
		 * Joint space based impedance controller. A position error calculation 
		 * callback is required within the callback function. The 
		 * calculate_position_error function within the 
		 * joint_impedance_motion_generator class is public and is designed to be 
		 * used to a) hold the current position or b) to follow a given path (saved 
		 * during constructor call). Otherwise it is possible to use a different 
		 * function to calcualte/retrieve the position error for a specific step 
		 * calculation.
		 *
		 ************************************************************************/
		class joint_impedance_motion_generator
		{
		public:
			joint_impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				double duration,
				std::optional<std::string> log_file_path);

			joint_impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				std::list<std::array<double, 7>> joint_positions,
				double duration,
				std::optional<std::string> log_file_path);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 7, 1>(const double)> get_joint_position_error);

			Eigen::Matrix<double, 7, 1> calculate_joint_position_error(const franka::RobotState& robot_state, double time);

			// getter and setter for 'default' stiffness and damping
			void set_stiffness(const std::array<double, 49>& stiffness);

			std::array<double, 49> get_stiffness();
			std::array<double, 49> get_damping();

		private:
			void calculate_default_stiffness_and_damping();
			void init_impedance_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state);
			double calculate_damping_from_stiffness(double ki);

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			bool initialized_ = false;

			double duration_;
			double time_ = 0.0;
			std::list<double> timestamps_;

			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 7, 7> damping_matrix_ = Eigen::Matrix<double, 7, 7>::Zero();
			Eigen::Matrix<double, 7, 7> stiffness_matrix_ = Eigen::Matrix<double, 7, 7>::Zero();

			// Stiffness & Damping
			const std::array<double, 7> K_P_ = { 300.0, 300.0, 300.0, 300.0, 50.0, 50.0, 50.0 };

			// joint position error calculation
			std::array<double, 7> current_joint_position_ = { 0 };
			std::list<std::array<double, 7>> joint_positions_;

			double joint_position_interval_;
			double next_joint_position_at_ = 0.0;

			// csv logging
			bool logging_;
			logger logger_;
			std::vector<std::string> j_head = { "d_joint_1", "d_joint_2", "d_joint_3", "d_joint_4", "d_joint_5", "d_joint_6", "d_joint_7",
						"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
						"tau_j1", "tau_j2", "tau_j3", "tau_j4", "tau_j5", "tau_j6", "tau_j7" };
			std::vector<std::string> f_head = { "s1", "s2", "s3", "s4", "s5", "s6", "s7", "d1", "d2", "d3", "d4", "d5", "d6", "d7" };
			std::vector<std::string> s_head = { "time" };
		};
	} /* namespace detail */
} /* namespace franka_proxy */

#endif // INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP
