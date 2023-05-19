/**
 *************************************************************************
 *
 * @file motion_generator_joint_impedance.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP)
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP


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
		 * @class joint_impedance_motion_generator
		 *
		 * in use
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
				bool logging);

			joint_impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				std::list<std::array<double, 7>> joint_positions,
				double duration,
				bool logging);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 7, 1>(const double)> get_joint_position_error);

			Eigen::Matrix<double, 7, 1> calculate_joint_position_error(const franka::RobotState& robot_state, double time);

			// getter and setter for 'default' stiffness and damping
			bool set_stiffness(std::array<double, 49> stiffness);

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
			const std::array<double, 7> K_P_ = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
			const std::array<double, 7> K_D_ = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};

			// joint position error calculation
			std::array<double, 7> current_joint_position_ = { 0 };
			std::list<std::array<double, 7>> joint_positions_;

			double joint_position_interval_;
			double next_joint_position_at_ = 0.0;

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
