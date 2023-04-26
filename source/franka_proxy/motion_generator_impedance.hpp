/**
 *************************************************************************
 *
 * @file motion_generator_impedance.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HPP)
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HPP


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
		 * @class impedance_motion_generator
		 *
		 * in use
		 *
		 ************************************************************************/
		class impedance_motion_generator
		{
		public:

			impedance_motion_generator
				(franka::Robot& robot,
					std::mutex& state_lock,
					franka::RobotState& robot_state,
					double duration);

			franka::Torques callback
				(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 6, 1>(const double)> get_position_error);

		private:
			double optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max);
			double calculate_stiffness_from_damping(double di, double mi);

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			std::array<double, 6> b_ = { 0.03, 0.03, 0.03, 0.036, 0.036, 0.036 };
			std::array<double, 6> l_d_ = { 1280.0, 460.0, -30.0, 1990.0, -880.0, 1350.0 };
			std::array<double, 6> u_d_ = { 5035.0, 2000.0, 330.0, 4800.0, 5500.0, 3300.0 };

			std::array<double, 6> l_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> l_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> x0_max_ = { 0.025, 0.025, 0.025, 0.025, 0.025, 0.025 };
			std::array<double, 6> derived_x0_max_ = { 0.03, 0.03, 0.03, 0.03, 0.03, 0.03 };

			double duration_;
			double time_ = 0.0;
			std::list<double> timestamps_;

			std::list<std::array<double, 6>> measured_velocities_;
			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// csv logging
			std::ofstream csv_log_;
			std::string csv_header = "time; f_ext j1; f_ext j2; f_ext j3; f_ext j4; f_ext j5; f_ext j6; position_d x; position_d y; position_d z; position x; position y; position z; s j1; s j2; s j3; s j4; s j5; s j6; d j1; d j2; d j3; d j4; d j5; d j6";
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HPP) */
