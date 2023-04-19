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

			/*for (int i = 0; i < stiffness_matrix_.cols(); i++) {
				for (int j = 0; j < stiffness_matrix_.rows(); j++) {
					stiffness_matrix_(i, j) = stiffness_matrix_(i, j) * 0.5;
					damping_matrix_(i, j) = damping_matrix_(i, j) * 0.5;
				}
			}*/

			std::array<double, 6> f_init_ar_ = state_.O_F_ext_hat_K;
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_init_(f_init_ar_.data());

			// set desired force
			Eigen::Map<Eigen::Matrix<double, 6, 1>> f_mapped_(desired_force.data());
			f_d_ = f_mapped_;

			// start logging to csv file
			csv_log_.open("admittance.csv");
			csv_prod1_log_.open("admittance_prod1.csv");
			force_log_.open("force_log.csv");
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
				csv_log_.close();
				csv_prod1_log_.close();
				force_log_.close();
				return current_torques_;
			}

			////////////////////////////
			/*franka::Torques c_(state_.tau_J);

			std::array<double, 6> force_ar_ = state_.O_F_ext_hat_K;
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_(force_ar_.data());

			std::ostringstream force_log2_;
			force_log2_ << force_(0) << "; " << force_(1) << "; " << force_(2) << "; " << force_(3) << "; " << force_(4) << "; " << force_(5);
			force_log_ << force_log2_.str() << "\n";

			return c_;*/
			////////////////////////////

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

				return current_torques_;
			}

			//csv_log_.close();

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
			//Eigen::Matrix<double, 6, 1> f_error_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			// calculate delta time for position calculation
			double delta_time_ = last_time_ - time_;

			// calculate new position
			const Eigen::Matrix<double, 6, 6> x_i_prod1_ = ((stiffness_matrix_ * (delta_time_ * delta_time_))
				+ (damping_matrix_ * delta_time_) + inertia_matrix_).inverse();

			const Eigen::Matrix<double, 6, 1> x_i_sum1_ = (delta_time_ * delta_time_) * (f_error_ + (stiffness_matrix_ * position_eq_));
			const Eigen::Matrix<double, 6, 1> x_i_sum2_ = delta_time_ * damping_matrix_ * last_x_list_.front();
			const Eigen::Matrix<double, 6, 1> x_i_sum3_ = inertia_matrix_ * ((2 * last_x_list_.front()) - last_x_list_.back());
			const Eigen::Matrix<double, 6, 1> x_i_prod2_ = x_i_sum1_ + x_i_sum2_ + x_i_sum3_;

			Eigen::Matrix<double, 6, 1> x_i_ = x_i_prod1_ * x_i_prod2_;

			std::string over = "";

			if (x_i_(0) < 1.0 && x_i_(0) > -1.0 && x_i_(1) < 1.0 && x_i_(1) > -1.0 && x_i_(2) < 1.0 && x_i_(2) > -1.0 && x_i_(3) < 1.0 && x_i_(3) > -1.0 && x_i_(4) < 1.0 && x_i_(4) > -1.0 && x_i_(5) < 1.0 && x_i_(5) > -1.0) {
				over = "false";
			}
			else {
				x_i_ = last_x_list_.front();

				over = "true";
			}

			// store new x_i_ in list and remove oldest entry
			last_x_list_.push_front(x_i_);
			last_x_list_.pop_back();

			// save current time as last_time_ for accurate delta_time calculations within next iteration
			last_time_ = time_;

			// log to csv
			std::ostringstream f_ext_log_;
			f_ext_log_ << f_ext_(0) << "; " << f_ext_(1) << "; " << f_ext_(2) << "; " << f_ext_(3) << "; " << f_ext_(4) << "; " << f_ext_(5);
			std::ostringstream ft_existing_log_;
			ft_existing_log_ << f_ext_(0) << "; " << f_ext_(1) << "; " << f_ext_(2) << "; " << f_ext_(3) << "; " << f_ext_(4) << "; " << f_ext_(5);
			std::ostringstream x_i_log_;
			x_i_log_ << x_i_(0) << "; " << x_i_(1) << "; " << x_i_(2) << "; " << x_i_(3) << "; " << x_i_(4) << "; " << x_i_(5);
			std::ostringstream x_e_log_;
			x_e_log_ << position_eq_(0) << "; " << position_eq_(1) << "; " << position_eq_(2) << "; " << position_eq_(3) << "; " << position_eq_(4) << "; " << position_eq_(5);
			std::ostringstream x_i_prod2_log_;
			x_i_prod2_log_ << x_i_prod2_(0) << "; " << x_i_prod2_(1) << "; " << x_i_prod2_(2) << "; " << x_i_prod2_(3) << "; " << x_i_prod2_(4) << "; " << x_i_prod2_(5);
			std::ostringstream f_error_log_;
			f_error_log_ << f_error_(0) << "; " << f_error_(1) << "; " << f_error_(2) << "; " << f_error_(3) << "; " << f_error_(4) << "; " << f_error_(5);

			Eigen::Vector3d x_head_(x_i_.head(3));
			std::ostringstream x_head_log_;
			x_head_log_ << x_head_(0) << "; " << x_head_(1) << "; " << x_head_(2);

			std::ostringstream current_values;
			current_values << time_ << "; " << "; " << f_ext_log_.str() << "; " << "; " << ft_existing_log_.str() << "; " << "; " << x_i_log_.str() << "; " << "; " << x_e_log_.str() << "; " << "; " << x_i_prod2_log_.str() << "; " << "; " << over << "; " << "; " << x_head_log_.str() << "; " << "; " << f_error_log_.str();

			csv_log_ << current_values.str() << "\n";

			std::ostringstream prod1_log_;
			prod1_log_ << time_;

			for (int i = 0; i < 6; i++) {
				prod1_log_ << "; " << x_i_prod1_(i, 0) << "; " << x_i_prod1_(i, 1) << "; " << x_i_prod1_(i, 2) << "; " << x_i_prod1_(i, 3) << "; " << x_i_prod1_(i, 4) << "; " << x_i_prod1_(i, 5) << "\n";
			}

			prod1_log_ << "\n";

			csv_prod1_log_ << prod1_log_.str();

			// HOW TO CALULATE POSITION_EQ_?????????????????????????????????? CURRENT POSITION AS POSITION_EQ!!!!!!
			// DESIRED FORCE F_EXT AS PARAMETER FOR CONSTRUCTOR -> NOT FROM STATE OF ROBOT -> DELTA FORCE

			/*return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Vector3d
				{
					return x_i_.head(3);
				}
			);*/

			// todo this may be wrong! -> comment from other motion generator
			franka::Torques current_torques_(state_.tau_J);

			Eigen::Affine3d po_transform2_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d current_return_position_ = po_transform2_.translation();

			return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Vector3d
				{
					return x_head_;
				}
			);
		}


	} /* namespace detail */
} /* namespace franka_proxy */