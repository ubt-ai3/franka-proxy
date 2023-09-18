/**
 *************************************************************************
 *
 * @file motion_generator_joint_imp_ple.hpp
 *
 * Variant of of the joint space based impedance controller, for use
 * with the payload estimation from franka_control. Provides a
 * pre-defined motion and lacks the capabilities for processing
 * user-given paths.
 * Otherwise basically a copy of "motion_generator_joint_impedance.hpp".
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMP_PLE_HPP)
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMP_PLE_HPP


#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/robot.h>
#include <franka/model.h>



namespace franka_proxy
{
	namespace detail
	{
		/**
		 *************************************************************************
		 *
		 * @class ple_motion_generator
		 *
		 * in use
		 *
		 * Joint space based impedance controller, adapted for payload
		 * estimation. The position error calculation is replaced with a
		 * function representing the pre-defined motion.
		 ************************************************************************/
		class ple_motion_generator
		{
		public:
			ple_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				double duration,
				bool logging);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 7, 1>(const double)> get_joint_position_error);

			Eigen::Matrix<double, 7, 1> calculate_ple_motion(const franka::RobotState& robot_state, double time);

			// getter and setter for 'default' stiffness and damping
			void set_stiffness(const std::array<double, 49>& stiffness);

			std::array<double, 49> get_stiffness();
			std::array<double, 49> get_damping();

		private:
			void calculate_default_stiffness_and_damping();
			void init_impedance_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state);
			double calculate_damping_from_stiffness(double ki);
			void log_pos_error(Eigen::Matrix<double, 7, 1> q_d, Eigen::Matrix<double, 7, 1> q);
			void log(Eigen::Matrix<double, 7, 1> tau_d);

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

			// csv logging
			bool logging_;
			std::ofstream csv_log_1_;
			std::string csv_header1_ = "time; tau j1; tau j2; tau j3; tau j4; tau j5; tau j6; tau j7; s1; s2; s3; s4; s5; s6; s7; d1; d2; d3; d4; d5; d6; d7";
			std::ofstream csv_log_2_;
			std::string csv_header2_ = "time; d_joint 1; d_joint 2; d_joint 3; d_joint 4; d_joint 5; d_joint 6; d_joint 7; joint 1; joint 2; joint 3; joint 4; joint 5; joint 6; joint 7";
		};
	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP) */
