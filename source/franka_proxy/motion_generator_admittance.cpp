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
			std::array<double, 6> desired_force, // TODO: remove parameter
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

			// start logging to csv file
			csv_log_.open("admittance.csv");
			csv_prod1_log_.open("admittance_prod1.csv");
			force_log_.open("force_log.csv");
			noise_log_.open("force_noise_log.csv");
			x_i_log_.open("x_i_log.csv");
			x_i_log_ << "time" << "; " << "x_i x" << "; " << "x_i y" << "; " << "x_i z" << "; " << "x_i dx" << "; " << "x_i dy" << "; " << "x_i dz" << "; " << "x_i_1 x" << "; " << "x_i_1 y" << "; " << "x_i_1 z" << "; " << "x_i_1 dx" << "; " << "x_i_1 dy" << "; " << "x_i_1 dz" << "; " << "x_i_2 x" << "; " << "x_i_2 y" << "; " << "x_i_2 z" << "; " << "x_i_2 dx" << "; " << "x_i_2 dy" << "; " << "x_i_2 dz" << "; " << "red x_i x" << "; " << "red x_i y" << "; " << "red x_i z" << "; " << "red x_i dx" << "; " << "red x_i dy" << "; " << "red x_i dz" << "; " << "red x_i_1 x" << "; " << "red x_i_1 y" << "; " << "red x_i_1 z" << "; " << "red x_i_1 dx" << "; " << "red x_i_1 dy" << "; " << "red x_i_1 dz" << "; " << "red x_i_2 x" << "; " << "red x_i_2 y" << "; " << "red x_i_2 z" << "; " << "red x_i_2 dx" << "; " << "red x_i_2 dy" << "; " << "red x_i_2 dz" << "\n";
	
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
				noise_log_.close();
				x_i_log_.close();

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
			if (last_x_i_list_.size() < 2) {
				// add current position to last positions list
				last_x_i_list_.push_front(position_eq_);

				xi1 = position_eq_;
				xi2 = position_eq_;
				

				// todo this may be wrong! -> comment from other motion generator
				franka::Torques current_torques_(state_.tau_J);

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

			// todo
			std::array<double, 16> ee_T_k = state_.EE_T_K;
			Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_T_k_matrix(ee_T_k.data());

			// get ext force
			std::array<double, 6> f_ext_ar_ = state_.O_F_ext_hat_K;
			
			// add measured f_ext to array
			f_exts_.push_front(f_ext_ar_);

			// calculate f_ext from last measurements
			std::array<double, 6> f_ext_middle = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			std::list<std::array<double, 6>> f_exts_it(f_exts_);

			for (int i = 0; i < f_exts_.size(); i++) {
				std::array<double, 6> current_el = f_exts_it.front();
				f_exts_it.pop_front();

				f_ext_middle[0] = f_ext_middle[0] + current_el[0];
				f_ext_middle[1] = f_ext_middle[1] + current_el[1];
				f_ext_middle[2] = f_ext_middle[2] + current_el[2];
				f_ext_middle[3] = f_ext_middle[3] + current_el[3];
				f_ext_middle[4] = f_ext_middle[4] + current_el[4];
				f_ext_middle[5] = f_ext_middle[5] + current_el[5];

				if (i == f_exts_.size() - 1) {
					auto size = f_exts_.size();

					f_ext_middle[0] = f_ext_middle[0] / size;
					f_ext_middle[1] = f_ext_middle[1] / size;
					f_ext_middle[2] = f_ext_middle[2] / size;
					f_ext_middle[3] = f_ext_middle[3] / size;
					f_ext_middle[4] = f_ext_middle[4] / size;
					f_ext_middle[5] = f_ext_middle[5] / size;
				}
			}

			if (f_exts_.size() == 41) {
				f_exts_.pop_back();
			}

			// x_i-1 and x_i_2 noise reduction
			std::list<Eigen::Matrix<double, 6, 1>> x_is_it = {};
			Eigen::Matrix<double, 6, 1> x_i_1 = Eigen::Matrix<double, 6, 1>::Zero();
			Eigen::Matrix<double, 6, 1> x_i_2 = Eigen::Matrix<double, 6, 1>::Zero();

			for (int i = 0; i < last_x_i_list_.size(); i++) {
				auto current_el = last_x_i_list_.front();

				if (i < last_x_i_list_.size() - 1) {
					x_i_1(0, 0) = x_i_1(0, 0) + current_el(0, 0);
					x_i_1(1, 0) = x_i_1(1, 0) + current_el(1, 0);
					x_i_1(2, 0) = x_i_1(2, 0) + current_el(2, 0);
					x_i_1(3, 0) = x_i_1(3, 0) + current_el(3, 0);
					x_i_1(4, 0) = x_i_1(4, 0) + current_el(4, 0);
					x_i_1(5, 0) = x_i_1(5, 0) + current_el(5, 0);
				}
				

				if (i > 0) {
					x_i_2(0, 0) = x_i_2(0, 0) * current_el(0, 0);
					x_i_2(1, 0) = x_i_2(1, 0) * current_el(1, 0);
					x_i_2(2, 0) = x_i_2(2, 0) * current_el(2, 0);
					x_i_2(3, 0) = x_i_2(3, 0) * current_el(3, 0);
					x_i_2(4, 0) = x_i_2(4, 0) * current_el(4, 0);
					x_i_2(5, 0) = x_i_2(5, 0) * current_el(5, 0);
				}

				x_is_it.push_back(current_el);
				last_x_i_list_.pop_front();

				if (i == last_x_i_list_.size() - 1) {
					auto size = last_x_i_list_.size();

					x_i_1(0, 0) = x_i_1(0, 0) / size;
					x_i_1(1, 0) = x_i_1(1, 0) / size;
					x_i_1(2, 0) = x_i_1(2, 0) / size;
					x_i_1(3, 0) = x_i_1(3, 0) / size;
					x_i_1(4, 0) = x_i_1(4, 0) / size;
					x_i_1(5, 0) = x_i_1(5, 0) / size;

					x_i_2(0, 0) = x_i_2(0, 0) / size;
					x_i_2(1, 0) = x_i_2(1, 0) / size;
					x_i_2(2, 0) = x_i_2(2, 0) / size;
					x_i_2(3, 0) = x_i_2(3, 0) / size;
					x_i_2(4, 0) = x_i_2(4, 0) / size;
					x_i_2(5, 0) = x_i_2(5, 0) / size;
				}
			}

			last_x_i_list_ = x_is_it;

			// set current force for further calculations
			Eigen::Map<Eigen::Matrix<double, 6, 1>> current_force(f_ext_middle.data());

			// using constant as using actual timestamps causing too much noise
			double delta_time_ = 0.001;

			// calculate new position
			const Eigen::Matrix<double, 6, 6> x_i_prod1_ = ((stiffness_matrix_ * (delta_time_ * delta_time_))
				+ (damping_matrix_ * delta_time_) + inertia_matrix_).inverse();

			const Eigen::Matrix<double, 6, 1> x_i_sum1_ = (delta_time_ * delta_time_) * (-current_force + (stiffness_matrix_ * position_eq_));
			const Eigen::Matrix<double, 6, 1> x_i_sum2_ = delta_time_ * damping_matrix_ * x_i_1;
			const Eigen::Matrix<double, 6, 1> x_i_sum3_ = inertia_matrix_ * ((2 * x_i_1) - x_i_2);
			const Eigen::Matrix<double, 6, 1> x_i_prod2_ = x_i_sum1_ + x_i_sum2_ + x_i_sum3_;

			Eigen::Matrix<double, 6, 1> x_i_ = x_i_prod1_ * x_i_prod2_;

			// test new position
			const Eigen::Matrix<double, 6, 6> nx_i_prod1_ = ((stiffness_matrix_ * (delta_time_ * delta_time_))
				+ (damping_matrix_ * delta_time_) + inertia_matrix_).inverse();

			const Eigen::Matrix<double, 6, 1> nx_i_sum1_ = (delta_time_ * delta_time_) * (-current_force + (stiffness_matrix_ * position_eq_));
			const Eigen::Matrix<double, 6, 1> nx_i_sum2_ = delta_time_ * damping_matrix_ * xi1;
			const Eigen::Matrix<double, 6, 1> nx_i_sum3_ = inertia_matrix_ * ((2 * xi1) - xi2);
			const Eigen::Matrix<double, 6, 1> nx_i_prod2_ = nx_i_sum1_ + nx_i_sum2_ + nx_i_sum3_;

			Eigen::Matrix<double, 6, 1> nx_i_ = nx_i_prod1_ * nx_i_prod2_;

			// store new x_i_ in list and remove oldest entry
			last_x_i_list_.push_front(x_i_);

			if (last_x_i_list_.size() > 11) {
				last_x_i_list_.pop_back();
			}
			

			// test
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_ext_(f_ext_ar_.data());
			std::array<double, 7> gravity_array = model_.gravity(state_);
			Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_(gravity_array.data());
			Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(state_.tau_J.data());
			Eigen::VectorXd tau_existing = tau_measured - gravity_;
			auto ft_existing = jacobian_ * tau_existing;

			// log to csv
			std::ostringstream f_ext_log_;
			f_ext_log_ << f_ext_(0) << "; " << f_ext_(1) << "; " << f_ext_(2) << "; " << f_ext_(3) << "; " << f_ext_(4) << "; " << f_ext_(5);
			std::ostringstream ft_existing_log_;
			ft_existing_log_ << ft_existing(0) << "; " << ft_existing(1) << "; " << ft_existing(2) << "; " << ft_existing(3) << "; " << ft_existing(4) << "; " << ft_existing(5);
			std::ostringstream x_i_log;
			x_i_log << x_i_(0) << "; " << x_i_(1) << "; " << x_i_(2) << "; " << x_i_(3) << "; " << x_i_(4) << "; " << x_i_(5);
			std::ostringstream x_e_log_;
			x_e_log_ << position_eq_(0) << "; " << position_eq_(1) << "; " << position_eq_(2) << "; " << position_eq_(3) << "; " << position_eq_(4) << "; " << position_eq_(5);
			std::ostringstream x_i_prod2_log_;
			x_i_prod2_log_ << x_i_prod2_(0) << "; " << x_i_prod2_(1) << "; " << x_i_prod2_(2) << "; " << x_i_prod2_(3) << "; " << x_i_prod2_(4) << "; " << x_i_prod2_(5);
			std::ostringstream current_force_log;
			current_force_log << current_force(0) << "; " << current_force(1) << "; " << current_force(2) << "; " << current_force(3) << "; " << current_force(4) << "; " << current_force(5);
			std::ostringstream f_ext_middle_log;
			f_ext_middle_log << f_ext_middle[0] << "; " << f_ext_middle[1] << "; " << f_ext_middle[2] << "; " << f_ext_middle[3] << "; " << f_ext_middle[4] << "; " << f_ext_middle[5];

			Eigen::Vector3d x_head_(x_i_.head(3));
			std::ostringstream x_head_log_;
			x_head_log_ << x_head_(0) << "; " << x_head_(1) << "; " << x_head_(2);

			std::ostringstream current_values;
			current_values << time_ << "; " << "; " << f_ext_log_.str() << "; " << "; " << ft_existing_log_.str() << "; " << "; " << x_i_log.str() << "; " << "; " << x_e_log_.str() << "; " << "; " << current_force_log.str();

			csv_log_ << current_values.str() << "\n";

			std::ostringstream prod1_log_;
			prod1_log_ << time_;

			for (int i = 0; i < 6; i++) {
				prod1_log_ << "; " << x_i_prod1_(i, 0) << "; " << x_i_prod1_(i, 1) << "; " << x_i_prod1_(i, 2) << "; " << x_i_prod1_(i, 3) << "; " << x_i_prod1_(i, 4) << "; " << x_i_prod1_(i, 5) << "\n";
			}

			prod1_log_ << "\n";

			csv_prod1_log_ << prod1_log_.str();

			std::ostringstream current_noise_values;
			current_noise_values << time_ << "; " << "; " << f_ext_log_.str() << "; " << "; " << f_ext_middle_log.str();
			noise_log_ << current_noise_values.str() << "\n";

			std::ostringstream nxi_log;
			nxi_log << nx_i_(0) << "; " << nx_i_(1) << "; " << nx_i_(2) << "; " << nx_i_(3) << "; " << nx_i_(4) << "; " << nx_i_(5);
			std::ostringstream nxi1_log;
			nxi1_log << xi1(0) << "; " << xi1(1) << "; " << xi1(2) << "; " << xi1(3) << "; " << xi1(4) << "; " << xi1(5);
			std::ostringstream nxi2_log;
			nxi2_log << xi2(0) << "; " << xi2(1) << "; " << xi2(2) << "; " << xi2(3) << "; " << xi2(4) << "; " << xi2(5);
			std::ostringstream rxi_log;
			rxi_log << x_i_(0) << "; " << x_i_(1) << "; " << x_i_(2) << "; " << x_i_(3) << "; " << x_i_(4) << "; " << x_i_(5);
			std::ostringstream rxi1_log;
			rxi1_log << x_i_1(0) << "; " << x_i_1(1) << "; " << x_i_1(2) << "; " << x_i_1(3) << "; " << x_i_1(4) << "; " << x_i_1(5);
			std::ostringstream rxi2_log;
			rxi2_log << xi2(0) << "; " << x_i_2(1) << "; " << x_i_2(2) << "; " << x_i_2(3) << "; " << x_i_2(4) << "; " << x_i_2(5);

			xi2 = xi1;
			xi1 = nx_i_;

			std::ostringstream current_x_values;
			current_x_values << time_ << "; " << nxi_log.str() << "; " << nxi1_log.str() << "; " << nxi2_log.str() << "; " << rxi_log.str() << "; " << rxi1_log.str() << "; " << rxi2_log.str();
			x_i_log_ << current_x_values.str() << "\n";

			return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Vector3d
				{
					return nx_i_.head(3); // TODO: Change Impedance callback to use all 6 components
				}
			);

			/*return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Vector3d
				{
					return x_head_;
				}
			);*/
		}


	} /* namespace detail */
} /* namespace franka_proxy */