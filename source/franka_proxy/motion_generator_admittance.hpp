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
				double duration);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period);

		private:

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			std::array<double, 6> b_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> l_d_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_d_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> l_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> l_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> u_derived_x0_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::array<double, 6> x0_max_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			std::array<double, 6> derived_x0_max_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			double duration_;
			double time_ = 0.0;
			double last_time_ = 0.0;

			///////////////////////////////////////////////
			Eigen::Vector3d position_eq_; // x_e interaction port equilibrium position - used as position_d_ in impedance

			// set positions
			std::list<Eigen::Matrix<double, 6, 1>> last_x_list_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			///////////////////////////////////////////////

			Eigen::Quaterniond orientation_d_;
			Eigen::Vector3d position_d_;

			std::list<std::array<double, 6>> measured_velocities_;
			std::list<std::array<double, 7>> measured_joint_velocities_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_ADMITTANCE_HPP) */
