/**
 *************************************************************************
 *
 * @file motion_generator_admittance.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP)
#define INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP


#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/robot.h>
#include <franka/model.h>

#include "motion_generator_cartesian_impedance.hpp"


namespace franka_proxy
{
	namespace detail
	{


		/**
		 *************************************************************************
		 *
		 * @class admittance_motion_generator
		 *
		 * in use
		 * 
		 * Cartesian admittance controller. This controller is using the cartesian
		 * impedance controller of the motion_generator_cartesian_impedance class
		 * to reach the calculated xi positions (xi = position output of the
		 * admittance controller). The required position error calculation callback
		 * of the impedance controller is handled within this class and differs from
		 * the 'default' one presented in the motion_generator_cartesian_impedance
		 * class.
		 *
		 ************************************************************************/
		class admittance_motion_generator
		{
		public:

			admittance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				double duration,
				bool logging);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period);

			// getter and setter for 'default' stiffness and damping parameters
			void set_admittance_rotational_stiffness(double rotational_stiffness);
			void set_admittance_translational_stiffness(double translational_stiffness);

			void set_impedance_rotational_stiffness(double rotational_stiffness);
			void set_impedance_translational_stiffness(double translational_stiffness);

			double get_admittance_rotational_stiffness();
			double get_admittance_translational_stiffness();

			double get_impedance_rotational_stiffness();
			double get_impedance_translational_stiffness();

		private:
			void calculate_stiffness_and_damping();
			void log(
				Eigen::Matrix<double, 6, 1> f_ext,
				Eigen::Matrix<double, 6, 1> x_i,
				Eigen::Matrix<double, 6, 1> position_eq,
				Eigen::Matrix<double, 6, 1> x_i_prod2,
				Eigen::Matrix<double, 6, 1> current_force,
				const std::array<double, 6>& f_ext_middle,
				Eigen::Matrix<double, 6, 7> jacobian
			);

			std::vector<Eigen::Affine3d> fk(const Eigen::Matrix<double, 7, 1>& configuration);

			static constexpr double pi = 3.14159265358979323846;
			static constexpr double deg_to_rad = pi / 180;
			static constexpr double rad_to_deg = 180 / pi;

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			double duration_;
			double time_ = 0.0;

			std::list<std::array<double, 6>> f_exts_;

			bool initialized_ = false;
			Eigen::Matrix<double, 6, 1> x_i_1_;
			Eigen::Matrix<double, 6, 1> x_i_2_;

			Eigen::Quaterniond previous_quaternion_;

			// damping and stiffness matrix
			double translational_stiffness_ = 150.0;
			double rotational_stiffness_ = 10.0;
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// impedance controller to command new desired position
			cartesian_impedance_motion_generator impedance_controller_;

			// csv logging
			bool logging_;
			std::ofstream csv_log_;
			std::ofstream joint_log_;
			std::ofstream jacobian_log_;

			std::list<double> timestamps_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP) */