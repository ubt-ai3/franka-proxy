/**
 *************************************************************************
 *
 * @file motion_generator_force.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "pid_force_control_motion_generator.hpp"

#include <utility>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/model.h>


namespace franka_proxy
{
	


		//////////////////////////////////////////////////////////////////////////
		//
		// pid_force_control_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		pid_force_control_motion_generator::pid_force_control_motion_generator
		(franka::Robot& robot,
			double mass,
			double duration,
			double k_p,
			double k_i,
			double k_d)
			:
			dq_d_({ 0., 0., 0., 0., 0., 0., 0. }),
			dq_buffer_(dq_filter_size_ * 7, 0),
			target_mass(mass),
			duration(duration),
			k_p(k_p),
			k_i(k_i),
			k_d(k_d),
			model(robot.loadModel())
		{
			initial_state_ = robot.readOnce();
		}


		franka::Torques pid_force_control_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period)
		{
			time_ += period.toSec();

			if (time_ > duration)
			{
				// todo this may be wrong!
				franka::Torques current_torques(robot_state.tau_J);
				current_torques.motion_finished = true;
				return current_torques;
			}


			Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());

			std::array<double, 49> mass_array = model.mass(robot_state);
			Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

			std::array<double, 7> gravity_array = model.gravity(robot_state);
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

			std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());


			Eigen::VectorXd desired_force_torque(6), tau_existing(7), tau_desired(7), tau_command(7), tau_J_d(7);


			desired_force_torque.setZero();
			desired_force_torque(2) = target_mass * -9.81;


			tau_existing = tau_measured - gravity;
			tau_desired = jacobian.transpose() * desired_force_torque;
			tau_error_integral += period.toSec() * (tau_desired - tau_existing);
			// FF + PI control
			tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;


			// updateDQFilter
			update_dq_filter(robot_state);

			// compute torques according to impedance control law
			// with assumption mass = 0 (robot state does not provide ddq and own measurement too noisy)
			for (int i = 0; i < 7; i++)
				tau_J_d[i] = K_P_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
				K_D_[i] * (dq_d_[i] - compute_dq_filtered(i));


			std::array<double, 7> tau_d_array{};
			Eigen::VectorXd::Map(&tau_d_array[0], 7) = (tau_command + tau_J_d) * 0.5;


			forces_z.push_back(robot_state.O_F_ext_hat_K[2]);
			des_mass.push_back(target_mass * -9.81);


			return tau_d_array;
		}

		std::vector<double> pid_force_control_motion_generator::give_forces() {
			return forces_z;
		}
		std::vector<double> pid_force_control_motion_generator::give_desired_mass() {
			return des_mass;
		}


		void pid_force_control_motion_generator::update_dq_filter(const franka::RobotState& robot_state)
		{
			for (int i = 0; i < 7; i++)
				dq_buffer_[dq_current_filter_position_ * 7 + i] = robot_state.dq[i];

			dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
		}


		double pid_force_control_motion_generator::compute_dq_filtered(int j)
		{
			double value = 0.0;
			for (size_t i = j; i < 7 * dq_filter_size_; i += 7)
				value += dq_buffer_[i];

			return value / dq_filter_size_;
		}





} /* namespace franka_proxy */
