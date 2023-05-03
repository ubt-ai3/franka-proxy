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
		class impedance_position_error_calculator;

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

			impedance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				std::list<std::array<double, 16>> poses,
				double duration);

			franka::Torques callback
				(const franka::RobotState& robot_state,
				franka::Duration period,
				std::function<Eigen::Matrix<double, 6, 1>(const double)> get_position_error);

			Eigen::Matrix<double, 6, 1> calculate_position_error(const franka::RobotState& robot_state, double time);

		private:
			double optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max);
			double calculate_stiffness_from_damping(double di, double mi);

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			const double PI = 3.141592653589793238463;

			//std::array<double, 6> b_ = { 0.03, 0.03, 0.03, 0.036, 0.036, 0.036 };
			//std::array<double, 6> b_ = { 0.03, 0.03, 0.03, 0.0, 0.0, 0.0 };
			std::array<double, 6> b_ = { 0.04, 0.04, 0.04, PI/6, PI/6, PI/6 };
			//std::array<double, 6> l_d_ = { 102.0, 102.0, 102.0, 27.0, 27.0, 27.0 };
			//std::array<double, 6> l_d_ = { 134.0, 134.0, 134.0, 44.72, 44.72, 44.72 };
			//std::array<double, 6> l_d_ = { 230.0, 230.0, 230.0, 19.0, 36.0, 38.0 }; /////////////////////////////////////////////////////
			//std::array<double, 6> l_d_ = { 32.0, 32.0, 32.0, 5.1, 5.1, 5.1 };
			std::array<double, 6> l_d_ = { 81.241, 81.241, 81.241, 20.1, 20.1, 20.1 };
			//std::array<double, 6> l_d_ = { 100.0, 100.0, 100.0, 300.0, 300.0, 300.0 };
			//std::array<double, 6> l_d_ = { 1280.0, 460.0, -30.0, 1990.0, -880.0, 1350.0 };
			//std::array<double, 6> u_d_ = { 450.0, 450.0, 450.0, 34.0, 85.0, 69.0}; /////////////////////////////////////////////////////
			//std::array<double, 6> u_d_ = { 81.241, 81.241, 81.241, 20.1, 20.1, 20.1 };
			std::array<double, 6> u_d_ = { 150.0, 150.0, 150.0, 50.0, 50.0, 50.0 };
			//std::array<double, 6> u_d_ = { 134.0, 134.0, 134.0, 44.72, 44.72, 44.72 };
			//std::array<double, 6> u_d_ = { 102.0, 102.0, 102.0, 27.0, 27.0, 27.0 };
			//std::array<double, 6> u_d_ = { 5035.0, 2000.0, 330.0, 4800.0, 5500.0, 3300.0 };

			std::array<double, 6> l_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> l_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> x0_max_ = { 0.0218, 0.0160, 0.0057, 0.0192, 0.0097, 0.0301 };
			//std::array<double, 6> x0_max_ = { 0.025, 0.025, 0.025, 0.0, 0.0, 0.0 };
			std::array<double, 6> derived_x0_max_ = { 0.1729, 0.1240, 0.0635, 0.4331, 0.2396, 0.1633 };
			//std::array<double, 6> derived_x0_max_ = { 0.03, 0.03, 0.03, 0.0, 0.0, 0.0 };

			double duration_;
			double time_ = 0.0;
			std::list<double> timestamps_;

			std::list<std::array<double, 6>> measured_velocities_;
			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// position error calculation
			std::array<double, 16> current_pose_;
			std::list<std::array<double, 16>> poses_;

			double pose_interval_;
			double next_pose_at_ = 0.0;

			// csv logging
			std::ofstream csv_log_;
			std::string csv_header = "time; f_ext j1; f_ext j2; f_ext j3; f_ext j4; f_ext j5; f_ext j6; s1; s2; s3; s4; s5; s6; d1; d2; d3; d4; d5; d6";
			//std::string csv_header = "time; f_ext j1; f_ext j2; f_ext j3; f_ext j4; f_ext j5; f_ext j6; position_d x; position_d y; position_d z; position x; position y; position z; s j1; s j2; s j3; s j4; s j5; s j6; d j1; d j2; d j3; d j4; d j5; d j6";
			std::ofstream matrix_log_;
		};
	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HPP) */
