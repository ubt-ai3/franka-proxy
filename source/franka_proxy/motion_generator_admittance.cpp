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
				franka::Torques current_torques(state_.tau_J);
				current_torques.motion_finished = true;
				csv_log_.close();
				csv_prod1_log_.close();
				force_log_.close();
				noise_log_.close();

				return current_torques;
			}

			//// get current desired position and orientation
			//Eigen::Affine3d po_d_transform(Eigen::Matrix4d::Map(current_pose_.data()));
			//Eigen::Vector3d position_d(po_d_transform.translation());
			//Eigen::Quaterniond orientation_d(po_d_transform.linear());

			//// get current position and orientation
			//Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			//Eigen::Vector3d position(po_transform.translation());
			//Eigen::Quaterniond orientation(po_transform.linear());

			//Eigen::Matrix<double, 6, 1> position_error;

			//// calculate the position error
			//position_error.head(3) << position - position_d; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix

			//// calculate orientation error
			//if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
			//	orientation.coeffs() << -orientation.coeffs();
			//}

			//// "difference" quaternion
			//Eigen::Quaterniond diff_quaternion(orientation.inverse() * orientation_d);
			//position_error.tail(3) << diff_quaternion.x(), diff_quaternion.y(), diff_quaternion.z();
			//// Transform to base frame
			//position_error.tail(3) << -po_transform.linear() * position_error.tail(3);

			// get current position and orientation
			Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d current_position(po_transform.translation());
			Eigen::Quaterniond orientation(po_transform.linear());

			// calculate/set current_x_
			Eigen::Matrix<double, 6, 1> position_eq;
			position_eq.head(3) << current_position;

			// calculate orientation
			if (orientation.coeffs().dot(orientation.coeffs()) < 0.0) {
				orientation.coeffs() << -orientation.coeffs();
			}

			// "difference" quaternion
			Eigen::Quaterniond diff_quaternion(orientation.inverse() * orientation);
			position_eq.tail(3) << diff_quaternion.x(), diff_quaternion.y(), diff_quaternion.z();
			// Transform to base frame
			position_eq.tail(3) << -po_transform.linear() * position_eq.tail(3);

			//position_eq.tail(3) << orientation.x(), orientation.y(), orientation.z();
			// Transform to base frame
			//position_eq.tail(3) << -po_transform.linear() * position_eq.tail(3);

			// x_i-1 and x_i-2 are required for further calculations
			if (!initialized_) {
				// add current position to last positions list
				x_i_1_ = position_eq;
				x_i_2_ = position_eq;

				initialized_ = true;
			}

			// get mass matrix
			std::array<double, 49> mass_ar = model_.mass(state_);
			Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(mass_ar.data());

			// get jacobian
			std::array<double, 42> jac_ar = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jac_ar.data());

			// calculate inertia matrix
			// intertia = (J(q)*B^(-1)(q)*J(q).transpose())^(-1)
			Eigen::Matrix<double, 6, 6> inertia_matrix_ar = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse();
			// only using diagonal elements for damping and stiffness optimization, using complete matrix for output calculations
			Eigen::Map<const Eigen::Matrix<double, 6, 6>> inertia_matrix(inertia_matrix_ar.data());

			// get ext 
			std::array<double, 6> f_ext_ar = state_.O_F_ext_hat_K;

			// add measured f_ext to array
			f_exts_.push_front(f_ext_ar);

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

			// set current force for further calculations
			Eigen::Map<Eigen::Matrix<double, 6, 1>> current_force(f_ext_middle.data());

			// using constant as using actual timestamps causing too much noise
			double delta_time = 0.001;

			// calculate new position
			const Eigen::Matrix<double, 6, 6> x_i_prod1 = ((stiffness_matrix_ * (delta_time * delta_time))
				+ (damping_matrix_ * delta_time) + inertia_matrix).inverse();

			const Eigen::Matrix<double, 6, 1> x_i_sum1 = (delta_time * delta_time) * (-current_force + (stiffness_matrix_ * position_eq));
			const Eigen::Matrix<double, 6, 1> x_i_sum2 = delta_time * damping_matrix_ * x_i_1_;
			const Eigen::Matrix<double, 6, 1> x_i_sum3 = inertia_matrix * ((2 * x_i_1_) - x_i_2_);
			const Eigen::Matrix<double, 6, 1> x_i_prod2 = x_i_sum1 + x_i_sum2 + x_i_sum3;

			Eigen::Matrix<double, 6, 1> x_i = x_i_prod1 * x_i_prod2;

			// set new x_i-1 and x_i-2
			x_i_2_ = x_i_1_;
			x_i_1_ = x_i;


			// test
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_ext_(f_ext_ar.data());
			std::array<double, 7> gravity_array = model_.gravity(state_);
			Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
			Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(state_.tau_J.data());
			Eigen::VectorXd tau_existing = tau_measured - gravity;
			auto ft_existing = jacobian * tau_existing;

			// log to csv
			std::ostringstream f_ext_log;
			f_ext_log << f_ext_(0) << "; " << f_ext_(1) << "; " << f_ext_(2) << "; " << f_ext_(3) << "; " << f_ext_(4) << "; " << f_ext_(5);
			std::ostringstream ft_existing_log_;
			ft_existing_log_ << ft_existing(0) << "; " << ft_existing(1) << "; " << ft_existing(2) << "; " << ft_existing(3) << "; " << ft_existing(4) << "; " << ft_existing(5);
			std::ostringstream x_i_log;
			x_i_log << x_i(0) << "; " << x_i(1) << "; " << x_i(2) << "; " << x_i(3) << "; " << x_i(4) << "; " << x_i(5);
			std::ostringstream x_e_log;
			x_e_log << position_eq(0) << "; " << position_eq(1) << "; " << position_eq(2) << "; " << position_eq(3) << "; " << position_eq(4) << "; " << position_eq(5);
			std::ostringstream x_i_prod2_log;
			x_i_prod2_log << x_i_prod2(0) << "; " << x_i_prod2(1) << "; " << x_i_prod2(2) << "; " << x_i_prod2(3) << "; " << x_i_prod2(4) << "; " << x_i_prod2(5);
			std::ostringstream current_force_log;
			current_force_log << current_force(0) << "; " << current_force(1) << "; " << current_force(2) << "; " << current_force(3) << "; " << current_force(4) << "; " << current_force(5);
			std::ostringstream f_ext_middle_log;
			f_ext_middle_log << f_ext_middle[0] << "; " << f_ext_middle[1] << "; " << f_ext_middle[2] << "; " << f_ext_middle[3] << "; " << f_ext_middle[4] << "; " << f_ext_middle[5];

			std::ostringstream current_values;
			current_values << time_ << "; " << "; " << f_ext_log.str() << "; " << "; " << ft_existing_log_.str() << "; " << "; " << x_i_log.str() << "; " << "; " << x_e_log.str() << "; " << "; " << current_force_log.str();

			csv_log_ << current_values.str() << "\n";

			std::ostringstream prod1_log_;
			prod1_log_ << time_;

			for (int i = 0; i < 6; i++) {
				prod1_log_ << "; " << x_i_prod1(i, 0) << "; " << x_i_prod1(i, 1) << "; " << x_i_prod1(i, 2) << "; " << x_i_prod1(i, 3) << "; " << x_i_prod1(i, 4) << "; " << x_i_prod1(i, 5) << "\n";
			}

			prod1_log_ << "\n";

			csv_prod1_log_ << prod1_log_.str();

			std::ostringstream current_noise_values;
			current_noise_values << time_ << "; " << "; " << f_ext_log.str() << "; " << "; " << f_ext_middle_log.str();
			noise_log_ << current_noise_values.str() << "\n";

			return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Matrix<double, 6, 1>
				{
					return x_i;
				}
			);
		}


	} /* namespace detail */
} /* namespace franka_proxy */