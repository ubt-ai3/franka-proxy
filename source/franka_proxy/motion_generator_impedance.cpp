/**
 *************************************************************************
 *
 * @file motion_generator_impedance.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "motion_generator_impedance.hpp"

#include <utility>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/model.h>


namespace franka_proxy
{
	namespace detail
	{


		//////////////////////////////////////////////////////////////////////////
		//
		// impedance_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		impedance_motion_generator::impedance_motion_generator
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

			// load model
			model_ = robot.loadModel();

			robot.setCollisionBehavior({ {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },
				{ {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },
				{ {100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },
				{ {100.0, 100.0, 100.0, 100.0, 100.0, 100.0} });

			// equilibrium point is the current position
			Eigen::Affine3d init_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			position_d_ = init_transform_.translation();
			orientation_d_ = init_transform_.linear();

			const double translational_stiffness{ 150.0 };
			const double rotational_stiffness{ 10.0 };

			// initialize stiffness and damping matrix
			stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);

			// start logging to csv file
			csv_log_.open("impedance_log.csv");
			csv_log_ << csv_header << "\n";
		}

		franka::Torques impedance_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period,
			std::function<Eigen::Vector3d(const double)> get_desired_position)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			time_ += period.toSec();

			if (time_ > duration_) {
				// motion finished
				// todo this may be wrong! -> comment from other motion generator
				franka::Torques current_torques_(state_.tau_J);
				current_torques_.motion_finished = true;

				// close log file
				csv_log_.close();

				return current_torques_;
			}

			// save timestamp
			timestamps_.push_back(time_);

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
			Eigen::Matrix<double, 6, 6> inertia_matrix_ar = (jacobian_ * mass_matrix_.inverse() * jacobian_.transpose()).inverse();
			// only using diagonal elements for damping and stiffness optimization, using complete matrix for output calculations
			Eigen::Map<const Eigen::Matrix<double, 6, 6>> inertia_matrix_(inertia_matrix_ar.data());

			// get current desired position
			position_d_ = get_desired_position(time_);

			// get current position
			Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d position_(po_transform_.translation());

			// get current orientation
			Eigen::Quaterniond orientation_(po_transform_.linear());

			Eigen::Matrix<double, 6, 1> position_error_;

			// get current velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_(state_.dq.data());
			Eigen::Matrix<double, 6, 1> velocity_ = jacobian_ * dq_; // dx = j(q)*dq

			// set position error
			position_error_.head(3) << position_ - position_d_; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix

			// calculate orientation error
			if (orientation_d_.coeffs().dot(orientation_.coeffs()) < 0.0) {
				orientation_.coeffs() << -orientation_.coeffs();
			}

			// "difference" quaternion
			Eigen::Quaterniond diff_quaternion(orientation_.inverse() * orientation_d_);
			position_error_.tail(3) << diff_quaternion.x(), diff_quaternion.y(), diff_quaternion.z();
			// Transform to base frame
			position_error_.tail(3) << -po_transform_.linear() * position_error_.tail(3);

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

			// calculate delta time for acceleration and joint acceleration calculation
			double delta_time_ = timestamps_.back() - timestamps_.front();

			// calculate acceleration
			std::array<double, 6> acc_list_;

			// avoiding dividing by 0. Also: if no time has passed, no acceleration could have taken place
			if (delta_time_ == 0.0) {
				acc_list_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			}
			else {
				for (int i = 0; i < acc_list_.size(); i++) {
					double delta_velocity_ = measured_velocities_.back()[i] - measured_velocities_.front()[i];
					acc_list_[i] = delta_velocity_ / delta_time_;
				}
			}

			// init and set acceleration variable
			Eigen::Matrix<double, 6, 1> acceleration_(acc_list_.data());

			// remove first element of measured_joint_velocitues_ if there are more then eleven elements to calculate the current joint acceleration by the current joint velocity and the joint velocity measured ten cycles ago
			if (measured_joint_velocities_.size() > 11) {
				measured_joint_velocities_.pop_front();
			}

			// calculate joint acceleration
			std::array<double, 6> j_acc_list_;

			// avoiding dividing by 0. Also: if no time has passed, no (joint) acceleration could have taken place
			if (delta_time_ == 0.0) {
				j_acc_list_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			}
			else {
				for (int i = 0; i < j_acc_list_.size(); i++) {
					double j_delta_velocity_ = measured_joint_velocities_.back()[i] - measured_joint_velocities_.front()[i];
					j_acc_list_[i] = j_delta_velocity_ / delta_time_;
				}
			}

			// init and set joint acceleration variable
			Eigen::Matrix<double, 7, 1> j_acceleration_(j_acc_list_.data());

			// remove first element of timestamps_ if there are more then eleven timestamps, as it is not needed for further iterations and all calculations requiring timestamps_ are done at this point
			if (timestamps_.size() > 11) {
				timestamps_.pop_front();
			}

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

			// calculate external force
			Eigen::Matrix<double, 6, 1> f_ext_ = inertia_matrix_ * acceleration_ + damping_matrix_ * velocity_ + stiffness_matrix_ * position_error_;

			// calculate torque - without gravity as the robot handles it itself
			Eigen::VectorXd tau_d_ = mass_matrix_ * j_acceleration_ + coriolis_ - jacobian_.transpose() * f_ext_;

			std::array<double, 7> tau_d_ar_;
			Eigen::VectorXd::Map(&tau_d_ar_[0], 7) = tau_d_;

			// log to csv
			std::ostringstream f_ext_log_;
			f_ext_log_ << f_ext_(0) << "; " << f_ext_(1) << "; " << f_ext_(2) << "; " << f_ext_(3) << "; " << f_ext_(4) << "; " << f_ext_(5);
			std::ostringstream position_d_log_;
			position_d_log_ << position_d_.x() << "; " << position_d_.y() << "; " << position_d_.z();
			std::ostringstream position_log_;
			position_log_ << position_.x() << "; " << position_.y() << "; " << position_.z();
			std::ostringstream stiffness_matrix_log_;
			stiffness_matrix_log_ << stiffness_matrix_(0, 0) << "; " << stiffness_matrix_(1, 1) << "; " << stiffness_matrix_(2, 2) << "; " << stiffness_matrix_(3, 3) << "; " << stiffness_matrix_(4, 4) << "; " << stiffness_matrix_(5, 5);
			std::ostringstream damping_matrix_log_;
			damping_matrix_log_ << damping_matrix_(0, 0) << "; " << damping_matrix_(1, 1) << "; " << damping_matrix_(2, 2) << "; " << damping_matrix_(3, 3) << "; " << damping_matrix_(4, 4) << "; " << damping_matrix_(5, 5);

			std::ostringstream current_values;
			current_values << time_ << "; " << f_ext_log_.str() << "; " << position_d_log_.str() << "; " << position_log_.str() << "; " << stiffness_matrix_log_.str() << "; " << damping_matrix_log_.str();

			csv_log_ << current_values.str() << "\n";

			return tau_d_ar_;
		}

		double impedance_motion_generator::optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max) {
			// di = min(max(...), ...);
			const double di_max_val_ = std::max(l_di, ((2 * mi * derived_x0i_max) / ((bi - x0i_max) * exp(1))));
			return std::min(di_max_val_, u_di);
		}

		double impedance_motion_generator::calculate_stiffness_from_damping(double di, double mi) {
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