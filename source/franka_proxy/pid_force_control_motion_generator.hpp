/**
 *************************************************************************
 *
 * @file motion_generator_force.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP


#include <vector>

#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>

 //#include <jr3_ft_sensor/force_torque_sensor.hpp>


namespace franka_proxy
{
	


		/**
		 *************************************************************************
		 *
		 * @class force_motion_generator
		 *
		 * in use
		 *
		 ************************************************************************/
		class pid_force_control_motion_generator
		{
		public:

			pid_force_control_motion_generator
			(franka::Robot& robot, double mass, double duration, double k_p, double k_i, double k_d);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period);

			std::vector<double> give_forces();
			std::vector<double> give_desired_mass();

		private:

			void update_dq_filter(const franka::RobotState& robot_state);
			double compute_dq_filtered(int j);

			double time_{ 0.0 };
			
			const double filter_gain{ 0.01 };


			// Stiffness & Damping
			const std::array<double, 7> K_P_ = { {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0} };
			const std::array<double, 7> K_D_ = { {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0} };


			size_t dq_current_filter_position_ = 0;
			const size_t dq_filter_size_ = 5;

			std::array<double, 7> dq_d_;
			std::vector<double> dq_buffer_;

			double k_p;
			double k_i;
			double k_d;

			double target_mass;
			double duration;

			franka::Model model;

			Eigen::Matrix<double, 7, 1> initial_tau_ext;
			Eigen::Matrix<double, 7, 1> tau_error_integral;

			franka::RobotState initial_state_;

			std::vector<double> forces_z{}; // debug purpose
			std::vector<double> des_mass{}; //debug purpose
		};





} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_GENERATOR_HPP) */
