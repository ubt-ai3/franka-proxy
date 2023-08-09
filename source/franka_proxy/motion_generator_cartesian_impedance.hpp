/**
 *************************************************************************
 *
 * @file motion_generator_cartesian_impedance.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_CARTESIAN_IMPEDANCE_HPP)
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_CARTESIAN_IMPEDANCE_HPP


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
		 * @class cartesian_impedance_motion_generator
		 *
		 * in use
		 * 
		 * Cartesian impedance controller. A position error calculation callback
		 * is required within the callback function. The calculate_position_error
		 * function within the cartesian_impedance_motion_generator class is
		 * public and is designed to be used to a) hold the current position or
		 * b) to follow a given path (saved during constructor call). Otherwise it
		 * is possible to use a different function to calcualte/retrieve the 
		 * position error for a specific step calculation.
		 *
		 ************************************************************************/
		class cartesian_impedance_motion_generator
		{
		public:
			cartesian_impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				double duration,
				bool logging,
				bool use_online_parameter_calc);

			cartesian_impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				std::list<std::array<double, 16>> poses,
				double duration,
				bool logging,
				bool use_online_parameter_calc);

			franka::Torques callback
				(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 6, 1>(const double)> get_position_error);

			Eigen::Matrix<double, 6, 1> calculate_position_error(const franka::RobotState& robot_state, double time);

			// getter and setter for 'default' stiffness and damping
			void set_rotational_stiffness(double rotational_stiffness);
			void set_translational_stiffness(double translational_stiffness);

			double get_rotational_stiffness();
			double get_translational_stiffness();

		private:
			void calculate_default_stiffness_and_damping();
			void init_impedance_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state);
			double optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max);
			double calculate_stiffness_from_damping(double di, double mi);
			void log(Eigen::Matrix<double, 6, 1> f_ext, Eigen::Matrix<double, 6, 1> acceleration, Eigen::Matrix<double, 6, 1> velocity);
			void log_pos_error(Eigen::Vector3d position_d, Eigen::Quaterniond orientation_d, Eigen::Vector3d position, Eigen::Quaterniond orientation);

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			bool initialized_ = false;

			const double PI = 3.141592653589793238463;

			std::array<double, 6> b_ = { 0.04, 0.04, 0.04,  PI / 6,  PI / 6,  PI / 6 };

			std::array<double, 6> l_d_ = { 32.0, 32.0, 32.0, 5.1, 5.1, 5.1 }; 
			std::array<double, 6> u_d_ = { 150.0, 150.0, 150.0, 50.0, 50.0, 50.0 };

			std::array<double, 6> x0_max_ = { 0.0218, 0.0160, 0.0057, 0.0192, 0.0097, 0.0301 };
			std::array<double, 6> derived_x0_max_ = { 0.1729, 0.1240, 0.0635, 0.4331, 0.2396, 0.1633 };

			double duration_;
			double time_ = 0.0;
			std::list<double> timestamps_;

			std::list<std::array<double, 6>> measured_velocities_;
			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			double translational_stiffness_ = 300.0;
			double rotational_stiffness_ = 50.0;
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// position error calculation
			std::array<double, 16> current_pose_ = {0};
			std::list<std::array<double, 16>> poses_;

			std::list<Eigen::Quaterniond> orientations_;

			double pose_interval_;
			double next_pose_at_ = 0.0;

			// online stiffness and damping parameter calculation
			bool online_parameter_calc_ = false;

			// csv logging
			bool logging_;
			std::ofstream csv_log_1_;
			std::string csv_header1_ = "time; f_ext x; f_ext y; f_ext z; f_ext r_x; f_ext r_y; f_ext r_z; s1; s2; s3; s4; s5; s6; d1; d2; d3; d4; d5; d6";
			std::ofstream csv_log_2_;
			std::string csv_header2_ = "time; position_d x; position_d y; position_d z; rotation_d x; rotation_d y; rotation_d z; position x; position y; position z; rotation x; rotation y; rotation z ";
			std::ofstream acc_vel_log_;
		};
	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_CARTESIAN_IMPEDANCE_HPP) */
