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
				std::array<double, 6> desired_force,
				double duration);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period);

		private:

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			double duration_;
			double time_ = 0.0;
			double last_time_ = 0.0;

			Eigen::Matrix<double, 6, 1> f_d_;

			// set positions
			std::list<Eigen::Matrix<double, 6, 1>> last_x_list_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// impedance controller to command new desired positin
			impedance_motion_generator impedance_controller_;

			// csv logging
			std::ofstream csv_log_;
			std::ofstream csv_prod1_log_;
			std::ofstream force_log_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP) */