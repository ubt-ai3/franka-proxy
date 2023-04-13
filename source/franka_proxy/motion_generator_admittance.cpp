/**
 *************************************************************************
 *
 * @file motion_generator_admittance.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "motion_generator_admittance.hpp"

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
		// admittance_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		admittance_motion_generator::admittance_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			std::array<double, 6> desired_force,
			double duration)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration),
			impedance_controller_(robot, state_lock, robot_state, duration)
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

			const double translational_stiffness{ 150.0 };
			const double rotational_stiffness{ 10.0 };

			// initialize stiffness and damping matrix
			stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);

			// set desired force
			Eigen::Map<Eigen::Matrix<double, 6, 1>> f_mapped_(desired_force.data());
			f_d_ = f_mapped_;
		}

		franka::Torques admittance_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			time_ += period.toSec();

			// TODO: check return type of function
			if (time_ > duration_) {
				// motion finished
				// todo this may be wrong! -> comment from other motion generator
				franka::Torques current_torques_(state_.tau_J);
				current_torques_.motion_finished = true;

				return current_torques_;
			}

			// get current position
			Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d current_position_(po_transform_.translation());

			// get current orientation
			Eigen::Quaterniond orientation_(po_transform_.linear());

			// calculate/set current_x_
			Eigen::Matrix<double, 6, 1> position_eq_;
			position_eq_.head(3) << current_position_;
			position_eq_.tail(3) << orientation_.x(), orientation_.y(), orientation_.z();
			// Transform to base frame
			position_eq_.tail(3) << -po_transform_.linear() * position_eq_.tail(3); // TODO: NEEDED?

			// x_i-1 and x_i-2 are required for calculations
			// -> set them to the current position for initialization
			// -> sideeffect: enough timestamps to avoid having delta_time_ = 0
			if (last_x_list_.size() < 2) {
				// save current time as last_time_ for accurate delta_time calculations after the first two iterations
				last_time_ = time_;

				// add current position to last positions list
				last_x_list_.push_back(position_eq_);

				// todo this may be wrong! -> comment from other motion generator
				franka::Torques current_torques_(state_.tau_J);
				current_torques_.motion_finished = true;

				return current_torques_;
			}

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

			// get ext force
			std::array<double, 6> f_ext_ar_ = state_.O_F_ext_hat_K;
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_ext_(f_ext_ar_.data());
			// test
			std::array<double, 7> gravity_array = model_.gravity(state_);
			Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_(gravity_array.data());
			Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(state_.tau_J.data());
			Eigen::VectorXd tau_existing = tau_measured - gravity_;
			auto ft_existing = jacobian_ * tau_existing;

			// calculate force error
			Eigen::Matrix<double, 6, 1> f_error_ = f_ext_ - f_d_;

			// calculate delta time for position calculation
			double delta_time_ = last_time_ - time_;

			// calculate new position
			Eigen::Matrix<double, 6, 1> x_i_sum1_ = ((stiffness_matrix_ * (delta_time_ * delta_time_))
				+ (damping_matrix_ * delta_time_) + inertia_matrix_).inverse()
				* ((delta_time_ * delta_time_) * (f_error_ + stiffness_matrix_ * position_eq_));
			Eigen::Matrix<double, 6, 1> x_i_sum2_ = delta_time_ * damping_matrix_ * last_x_list_.front();
			Eigen::Matrix<double, 6, 1> x_i_sum3_ = inertia_matrix_ * ((2 * last_x_list_.front()) - last_x_list_.back());

			Eigen::Matrix<double, 6, 1> x_i_ = x_i_sum1_ + x_i_sum2_ + x_i_sum3_;

			// store new x_i_ in list and remove oldest entry
			last_x_list_.push_front(x_i_);
			last_x_list_.pop_back();

			// save current time as last_time_ for accurate delta_time calculations within next iteration
			last_time_ = time_;

			// HOW TO CALULATE POSITION_EQ_?????????????????????????????????? CURRENT POSITION AS POSITION_EQ!!!!!!
			// DESIRED FORCE F_EXT AS PARAMETER FOR CONSTRUCTOR -> NOT FROM STATE OF ROBOT -> DELTA FORCE
	
			return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Vector3d
				{
					return x_i_.head(3);
				}
			);
		}


	} /* namespace detail */
} /* namespace franka_proxy */