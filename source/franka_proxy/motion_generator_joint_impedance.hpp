/**
 *************************************************************************
 *
 * @file motion_generator_joint_impedance.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY___JOINT_MOTION_GENERATOR_IMPEDANCE_HPP)
#define INCLUDED__FRANKA_PROXY___JOINT_MOTION_GENERATOR_IMPEDANCE_HPP


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
				std::list<std::array<double, 16>> poses,
				double duration,
				bool logging);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 6, 1>(const double)> get_joint_position_error);

			Eigen::Matrix<double, 6, 1> calculate_position_error(const franka::RobotState& robot_state, double time);

			// getter and setter for 'default' stiffness and damping
			bool set_rotational_stiffness(double rotational_stiffness);
			bool set_translational_stiffness(double translational_stiffness);

			double get_rotational_stiffness();
			double get_translational_stiffness();

		private:
			void calculate_default_stiffness_and_damping();
			void init_impedance_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state);
			double calculate_stiffness_from_damping(double di, double mi);

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			bool initialized_ = false;

			double duration_;
			double time_ = 0.0;
			std::list<double> timestamps_;

			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			double translational_stiffness_ = 300.0;
			double rotational_stiffness_ = 50.0;
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero(); // 7 x 7
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero(); // 7 x 7

			// joint position error calculation
			std::array<double, 16> current_pose_ = { 0 };
			std::list<std::array<double, 16>> poses_;

			double pose_interval_;
			double next_pose_at_ = 0.0;

			// online stiffness and damping parameter calculation
			bool online_parameter_calc_ = false;

			// csv logging
			bool logging_;
			std::ofstream csv_log_1_;
			std::string csv_header1_ = "time; tau j1; tau j2; tau j3; tau j4; tau j5; tau j6; tau j7; s1; s2; s3; s4; s5; s6; s7; d1; d2; d3; d4; d5; d6; d7";
			std::ofstream csv_log_2_;
			std::string csv_header2_ = "time; d_joint 1; d_joint 2; d_joint 3; d_joint 4; d_joint 5; d_joint 6; d_joint 7; joint 1; joint 2; joint 3; joint 4; joint 5; joint 6; joint 7";
		};
	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HPP) */
