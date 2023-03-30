/**
 *************************************************************************
 *
 * @file motion_generator_impedance_hold_position.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "motion_generator_impedance_hold_position.hpp"

#include <utility>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/model.h>


namespace franka_proxy
{
	namespace detail
	{


		//////////////////////////////////////////////////////////////////////////
		//
		// impedance_hold_position_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		impedance_hold_position_motion_generator::impedance_hold_position_motion_generator
			(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			double duration)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			// get jacobian
			std::array<double, 42> jac_ar_ = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_(jac_ar_.data());

			// get current velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_(state_.dq.data());
			Eigen::Matrix<double, 6, 1> dx_ = jacobian_ * dq_; // dx = j(q)*dq

			// convert current velcoity to init measured velocities
			std::array<double, 6> new_measured_velocity_;
			Eigen::VectorXd::Map(&new_measured_velocity_[0], 6) = dx_;

			measured_velocities_.push_back(new_measured_velocity_);

			// convert current joint velocity to init measured joint velocities
			std::array<double, 7> new_measured_joint_velocity_;
			Eigen::VectorXd::Map(&new_measured_joint_velocity_[0], 7) = dq_;

			measured_joint_velocities_.push_back(new_measured_joint_velocity_);

			// calculate desired position
			Eigen::Affine3d po_d_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			position_d_(po_d_transform_.translation());

			// calculate/set x0_max and derived_x0_max
			for (int i = 0; i < sizeof x0_max_ / sizeof x0_max_[0]; i++) {
				x0_max_[i] = std::max(std::abs(l_x0_[i]), u_x0_[i]);
				derived_x0_max_[i] = std::max(std::abs(l_derived_x0_[i]), u_derived_x0_[i]);
			}
		}


		franka::Torques impedance_hold_position_motion_generator::callback(const franka::RobotState& robot_state, franka::Duration period)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			time_ += period.toSec();

			if (time_ > duration_) {
				// motion finished
				// todo this may be wrong! -> comment from other motion generator
				franka::Torques current_torques_(robot_state.tau_J);
				current_torques_.motion_finished = true;

				return current_torques_;
			}

			// get coriolis matrix (coriolis_ = C x dq_)
			std::array<double, 7> coriolis_ar_ = model_.coriolis(state_);
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis_(coriolis_ar_.data());

			// get mass matrix
			std::array<double, 49> mass_ar_ = model_.mass(state_);
			Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix_(mass_ar_.data());

			// get jacobian
			std::array<double, 42> jac_ar_ = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_(jac_ar_.data());

			// calculate inertia matrix
			// intertia = (J(q)*B^(-1)(q)*J(q).transpose())^(-1)
			Eigen::Matrix<double, 6, 6> nd_inertia_matrix_ = (jacobian_ * mass_matrix_.inverse() * jacobian_.transpose()).inverse();
			// only using diagonal elements for calculations
			Eigen::Matrix<double, 6, 6> inertia_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			for (int i = 0; i < nd_inertia_matrix_.rows(); i++) {
				inertia_matrix_(i, i) = nd_inertia_matrix_(i, i);
			}

			// get current position
			Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d position_(po_transform_.translation());

			Eigen::Matrix<double, 6, 1> position_error_;

			// get current velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_(state_.dq.data());
			Eigen::Matrix<double, 6, 1> velocity_ = jacobian_ * dq_; // dx = j(q)*dq

			// init acceleration variable
			Eigen::Matrix<double, 6, 1> acceleration_;

			// init joint acceleration variable
			Eigen::Matrix<double, 7, 1> j_acceleration_;

			// set position error
			position_error_.head(3) << position_ - position_d_; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix // TODO: what is head doing?

			// convert current velcoity and push it to measured velocities
			std::array<double, 6> new_measured_velocity_;
			Eigen::VectorXd::Map(&new_measured_velocity_[0], 6) = velocity_;

			measured_velocities_.push_back(new_measured_velocity_);

			// convert current joint velocity to feed measured joint velocities
			std::array<double, 7> new_measured_joint_velocity_;
			Eigen::VectorXd::Map(&new_measured_joint_velocity_[0], 7) = dq_;

			measured_joint_velocities_.push_back(new_measured_joint_velocity_);

			// remove first element of measured_velocitues_ if there are more then eleven elements to calculate the current acceleration by the current velocity and the velocity measured ten cycles ago
			if (measured_velocities_.size() > 11) {
				measured_velocities_.pop_front();
			}

			// calculate acceleration
			std::array<double, 6> acc_list_;
			double delta_time_ = measured_velocities_.size() * 0.001;

			for (int i = 0; i < acc_list_.size(); i++) {
				double delta_velocity_ = measured_velocities_.back()[i] - measured_velocities_.front()[i];
				acc_list_[i] = delta_velocity_ / delta_time_;
			}

			// set acceleration
			acceleration_(acc_list_.data());

			// remove first element of measured_joint_velocitues_ if there are more then eleven elements to calculate the current joint acceleration by the current joint velocity and the joint velocity measured ten cycles ago
			if (measured_joint_velocities_.size() > 11) {
				measured_joint_velocities_.pop_front();
			}

			// calculate joint acceleration
			std::array<double, 6> j_acc_list_;
			double j_delta_time_ = measured_joint_velocities_.size() * 0.001; // TODO: use time?

			for (int i = 0; i < j_acc_list_.size(); i++) {
				double j_delta_velocity_ = measured_joint_velocities_.back()[i] - measured_joint_velocities_.front()[i];
				j_acc_list_[i] = j_delta_velocity_ / j_delta_time_;
			}

			// set joint acceleration
			j_acceleration_(j_acc_list_.data());

			/*
			// stiffness and damping
			for (int i = 0; i < inertia_matrix_.rows(); i++) {
				double mi = inertia_matrix_(i,i);

				// optimize damping
				double di = optimizeDamping(l_d_[i], u_d_[i], mi, b_[i], x0_max_[i], derived_x0_max_[i]);

				// TODO: stability check

				// get stiffness from new calculated damping value
				double ki = calculate_stiffness_from_damping(di, mi);

				// add new values to matrices
				damping_matrix_(i, i) = di;
				stiffness_matrix_(i, i) = ki;
			}*/

			// TEST STIFFNESS AND DAMPING FOR TESTING WITHOUT IMPEDANCE PLANNER
			const double translational_stiffness{ 150.0 };
			const double rotational_stiffness{ 10.0 };

			stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);

			damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);
			// ----- TEST STIFFNESS AND DAMPING FOR TESTING WITHOUT IMPEDANCE PLANNER

			// calculate external force
			Eigen::Matrix<double, 6, 1> f_ext_ = inertia_matrix_ * acceleration_ + damping_matrix_ * velocity_ + stiffness_matrix_ * position_error_;

			// calculate torque - without gravity as the robot handles it itself
			Eigen::VectorXd tau_d_ = mass_matrix_ * j_acceleration_ + coriolis_ - jacobian_.transpose() * f_ext_;

			std::array<double, 7> tau_d_ar_;
			Eigen::VectorXd::Map(&tau_d_ar_[0], 7) = tau_d_;
			
			return tau_d_ar_;
		}

		double impedance_hold_position_motion_generator::optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max) {
			// di = min(max(...), ...);
			const double di_max_val_ = std::max(l_di, ((2 * mi * derived_x0i_max) / ((bi - x0i_max) * exp(1))));
			return std::min(di_max_val_, u_di);
		}

		double impedance_hold_position_motion_generator::calculate_stiffness_from_damping(double di, double mi) {
			/**
				critically damped condition

				stiffness ki = (di)^2/4mi
			*/

			double ki_ = di * di;

			if (mi <= 0) {
				// ki = ki/4;
				// do nothing and return (di^2) ???
			}
			else {
				ki_ = ki_ / (4 * mi);
			}

			return ki_;
		}


	} /* namespace detail */
} /* namespace franka_proxy */
