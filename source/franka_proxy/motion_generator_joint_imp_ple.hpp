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

#include "ft_sensor/schunk_ft.hpp"



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
			void init_ple_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state);
			double calculate_damping_from_stiffness(double ki);
			void log(std::array<double, 7> j, std::array<double, 6> ft, double time, Eigen::Matrix<double, 6, 1> sensor_velocities);

			franka::Model model_;

			franka_proxy::schunk_ft_sensor sensor_;
			Eigen::Affine3f placeholder_ = Eigen::Affine3f::Identity();
			Eigen::Vector3d no_mass_ = Eigen::Vector3d::Zero();

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
			std::ofstream csv_log_;
			std::string csv_header_ = "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7,force_x,force_y,force_z,torque_x,torque_y,torque_z,time,v_x,v_y,v_z,w_x,w_y,w_z";
		};
	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_JOINT_IMPEDANCE_HPP) */
