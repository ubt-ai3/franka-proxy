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

#pragma once


#include <list>
#include <vector>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>

#include <franka_proxy_share/franka_proxy_logger.hpp>

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
				double speed,
				double duration,
				std::optional<std::string> log_file_path);

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

			double desired_speed_ = 0.33; // how fast the motion should be executed, 0.33 is a safe choice
			double speed_factor_ = 0.0; // adaptable speed factor, allows for gradual acceleration up to speeds beyond 0.4
			double x_ = 0.0;
			bool accelerate_ = false; // whether to accelerate or start at the desired speed
			const double acceleration_ = 0.01; // by how much the robot will accelerate every step until reaching desired speed
			const double decel_factor_ = 0.99; // how hard the robot will hit the brakes once duration is up (smaller = harder)
			bool done_ = false; // whether the motion is done up to a point where the robot can safely come to a complete stop immediately

			std::list<std::array<double, 7>> measured_joint_velocities_;

			// damping and stiffness matrix
			Eigen::Matrix<double, 7, 7> damping_matrix_ = Eigen::Matrix<double, 7, 7>::Zero();
			Eigen::Matrix<double, 7, 7> stiffness_matrix_ = Eigen::Matrix<double, 7, 7>::Zero();

			// Stiffness & Damping
			const std::array<double, 7> K_P_ = { 300.0, 300.0, 300.0, 300.0, 50.0, 50.0, 50.0 };

			// csv logging
			logger logger_;
			bool logging_;
			std::vector<std::string> cart_ = { "lin_v_x", "lin_v_y", "lin_v_z", "ang_v_x", "ang_v_y", "ang_v_z" };
			std::vector<std::string> grav_{ "g_x", "g_y", "g_z" };
			std::vector<std::string> ft_ = { "force_x", "force_y","force_z","torque_x","torque_y","torque_z" };
			std::vector<std::string> t_{ "time" };
		};
	} /* namespace detail */
} /* namespace franka_proxy */