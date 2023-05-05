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

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/robot.h>
#include <franka/model.h>

#include "motion_generator_impedance.hpp"

#include <iostream>
#include <fstream>



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
			bool set_admittance_rotational_stiffness(double rotational_stiffness);
			bool set_admittance_translational_stiffness(double translational_stiffness);

			bool set_impedance_rotational_stiffness(double rotational_stiffness);
			bool set_impedance_translational_stiffness(double rotational_stiffness);

			double get_admittance_rotational_stiffness();
			double get_admittance_translational_stiffness();

			double get_impedance_rotational_stiffness();
			double get_impedance_translational_stiffness();

		private:
			void calculate_stiffness_and_damping();

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			double duration_;
			double time_ = 0.0;

			std::list<std::array<double, 6>> f_exts_;

			bool initialized_ = false;
			Eigen::Matrix<double, 6, 1> x_i_1_;
			Eigen::Matrix<double, 6, 1> x_i_2_;

			// damping and stiffness matrix
			double translational_stiffness_ = 150.0;
			double rotational_stiffness_ = 10.0;
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// impedance controller to command new desired position
			impedance_motion_generator impedance_controller_;

			// csv logging
			bool logging_;
			std::ofstream csv_log_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP) */