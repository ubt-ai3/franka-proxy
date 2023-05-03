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

			const double translational_stiffness{300.0};
			const double rotational_stiffness{ 50.0 };

			// initialize stiffness and damping matrix
			//stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			//stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			//damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
			//	Eigen::MatrixXd::Identity(3, 3);
			//damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
			//	Eigen::MatrixXd::Identity(3, 3);

			// position error calculation initialization - initial pose
			current_pose_ = state_.O_T_EE;
			pose_interval_ = 0.0;

			// start logging to csv file
			csv_log_.open("impedance_log.csv");
			csv_log_ << csv_header << "\n";

			matrix_log_.open("impedance_matrix_log.csv");

			/////////////////////////////////////////////////////////////
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

			
			double delta_time = 0.001;
			// stiffness and damping
			for (int i = 0; i < inertia_matrix.rows(); i++) {
				double mi = inertia_matrix(i,i);

				// optimize damping
				double di = optimizeDamping(l_d_[i], u_d_[i], mi, b_[i], x0_max_[i], derived_x0_max_[i]);

				// stability check
				auto current_damping = damping_matrix_(i, i);

				if (di < current_damping) {
					auto stability_condition = current_damping - ((current_damping * current_damping * delta_time) / mi) + 1.0; // ((current_damping * 0.0002 * 0.001) / mi);

					if (di <= stability_condition) {
						di = stability_condition;
					}
				}

				// get stiffness from new calculated damping value
				double ki = calculate_stiffness_from_damping(di, mi);

				// add new values to matrices
				damping_matrix_(i, i) = di;
				stiffness_matrix_(i, i) = ki;

			}
			
		};

		impedance_motion_generator::impedance_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			std::list<std::array<double, 16>> poses,
			double duration)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			poses_(poses),
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
			
			/*const double translational_stiffness{300.0};
			const double rotational_stiffness{ 50.0 };

			// initialize stiffness and damping matrix
			stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
				Eigen::MatrixXd::Identity(3, 3);*/

			// position error calculation initialization - initial pose
			current_pose_ = state_.O_T_EE;

			if (duration > 0.0) {
				pose_interval_ = duration / poses.size();
			}
			else {
				pose_interval_ = 0.0;
			}

			// start logging to csv file
			csv_log_.open("impedance_log.csv");
			csv_log_ << csv_header << "\n";

			matrix_log_.open("impedance_matrix_log.csv");

			/////////////////////////////////////////////////////////////
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
			
			 double delta_time = 0.001;
			// stiffness and damping
			 for (int i = 0; i < inertia_matrix.rows(); i++) {
				double mi = inertia_matrix(i,i);

				// optimize damping
				double di = optimizeDamping(l_d_[i], u_d_[i], mi, b_[i], x0_max_[i], derived_x0_max_[i]);

				// stability check
				auto current_damping = damping_matrix_(i, i);

				//if (di < current_damping) {
				auto stability_condition = current_damping - ((current_damping * current_damping * delta_time) / mi) + 0.0; // ((current_damping * 0.0002 * 0.001) / mi);

					if (di <= stability_condition) {
						di = stability_condition;
					}
				//}

				// get stiffness from new calculated damping value
				double ki = calculate_stiffness_from_damping(di, mi);

				// add new values to matrices
				damping_matrix_(i, i) = di;
				stiffness_matrix_(i, i) = ki;
			}
		}

		franka::Torques impedance_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period,
			std::function<Eigen::Matrix<double, 6, 1>(const double)> get_position_error)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			time_ += period.toSec();

			if (time_ > duration_) {
				// motion finished
				franka::Torques current_torques(state_.tau_J);
				current_torques.motion_finished = true;

				// close log file
				csv_log_.close();
				matrix_log_.close();

				return current_torques;
			}

			// save timestamp
			timestamps_.push_back(time_);

			// get coriolis matrix (coriolis_ = C x dq_)
			std::array<double, 7> coriolis_ar = model_.coriolis(state_);
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_ar.data());

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

			// get current velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state_.dq.data());
			Eigen::Matrix<double, 6, 1> velocity = jacobian * dq; // dx = j(q)*dq

			// convert current velcoity and push it to measured velocities
			std::array<double, 6> new_measured_velocity_;
			Eigen::VectorXd::Map(&new_measured_velocity_[0], 6) = velocity;

			measured_velocities_.push_back(new_measured_velocity_);

			// convert current joint velocity to feed measured joint velocities
			std::array<double, 7> new_measured_joint_velocity;
			Eigen::VectorXd::Map(&new_measured_joint_velocity[0], 7) = dq;

			measured_joint_velocities_.push_back(new_measured_joint_velocity);

			// remove first element of measured_velocitues_ if there are more then eleven elements to calculate the current acceleration by the current velocity and the velocity measured ten cycles ago
			if (measured_velocities_.size() > 11) {
				measured_velocities_.pop_front();
			}

			// calculate delta time for acceleration and joint acceleration calculation
			double delta_time = timestamps_.back() - timestamps_.front();
			//double delta_time = 0.001;

			// calculate acceleration
			std::array<double, 6> acc_list_;

			// avoiding dividing by 0. Also: if no time has passed, no acceleration could have taken place
			if (delta_time == 0.0) {
				acc_list_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			}
			else {
				for (int i = 0; i < acc_list_.size(); i++) {
					double delta_velocity = measured_velocities_.back()[i] - measured_velocities_.front()[i];
					acc_list_[i] = delta_velocity / delta_time;
				}
			}

			// init and set acceleration variable
			Eigen::Matrix<double, 6, 1> acceleration(acc_list_.data());

			// remove first element of measured_joint_velocitues_ if there are more then eleven elements to calculate the current joint acceleration by the current joint velocity and the joint velocity measured ten cycles ago
			if (measured_joint_velocities_.size() > 11) {
				measured_joint_velocities_.pop_front();
			}

			// calculate joint acceleration
			std::array<double, 6> j_acc_list;

			// avoiding dividing by 0. Also: if no time has passed, no (joint) acceleration could have taken place
			if (delta_time == 0.0) {
				j_acc_list = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			}
			else {
				for (int i = 0; i < j_acc_list.size(); i++) {
					double j_delta_velocity = measured_joint_velocities_.back()[i] - measured_joint_velocities_.front()[i];
					j_acc_list[i] = j_delta_velocity / delta_time;
				}
			}

			// init and set joint acceleration variable
			Eigen::Matrix<double, 7, 1> j_acceleration(j_acc_list.data());

			// remove first element of timestamps_ if there are more then eleven timestamps, as it is not needed for further iterations and all calculations requiring timestamps_ are done at this point
			if (timestamps_.size() > 11) {
				timestamps_.pop_front();
			}

			/*
			// stiffness and damping
			for (int i = 0; i < inertia_matrix.rows(); i++) {
				double mi = inertia_matrix(i,i);

				// optimize damping
				double di = optimizeDamping(l_d_[i], u_d_[i], mi, b_[i], x0_max_[i], derived_x0_max_[i]);

				// stability check
				auto current_damping = damping_matrix_(i, i);

				//if (di < current_damping) {
					auto stability_condition = current_damping - ((current_damping * current_damping * delta_time) / mi) + ((current_damping * 0.0002 * 0.001) / mi);

					if (di <= stability_condition) {
						di = stability_condition;
					}
				//}

				// get stiffness from new calculated damping value
				double ki = calculate_stiffness_from_damping(di, mi);

				// add new values to matrices
				damping_matrix_(i, i) = di;
				stiffness_matrix_(i, i) = ki;
			}*/

			Eigen::Matrix<double, 6, 1> position_error(get_position_error(time_));

			// calculate external force
			Eigen::Matrix<double, 6, 1> f_ext = inertia_matrix * acceleration + damping_matrix_ * velocity + stiffness_matrix_ * position_error;

			// calculate torque - without gravity as the robot handles it itself
			Eigen::VectorXd tau_d = mass_matrix * j_acceleration + coriolis - jacobian.transpose() * f_ext;

			std::array<double, 7> tau_d_ar;
			Eigen::VectorXd::Map(&tau_d_ar[0], 7) = tau_d;

			// log to csv
			std::ostringstream f_ext_log;
			f_ext_log << f_ext(0) << "; " << f_ext(1) << "; " << f_ext(2) << "; " << f_ext(3) << "; " << f_ext(4) << "; " << f_ext(5);
			/*std::ostringstream position_d_log;
			position_d_log << position_d.x() << "; " << position_d.y() << "; " << position_d.z();
			std::ostringstream position_log;
			position_log << position.x() << "; " << position.y() << "; " << position.z();*/
			std::ostringstream stiffness_matrix_log;
			stiffness_matrix_log << stiffness_matrix_(0, 0) << "; " << stiffness_matrix_(1, 1) << "; " << stiffness_matrix_(2, 2) << "; " << stiffness_matrix_(3, 3) << "; " << stiffness_matrix_(4, 4) << "; " << stiffness_matrix_(5, 5);
			std::ostringstream damping_matrix_log;
			damping_matrix_log << damping_matrix_(0, 0) << "; " << damping_matrix_(1, 1) << "; " << damping_matrix_(2, 2) << "; " << damping_matrix_(3, 3) << "; " << damping_matrix_(4, 4) << "; " << damping_matrix_(5, 5);

			std::ostringstream current_values;
			current_values << time_ << "; " << f_ext_log.str() << "; " << stiffness_matrix_log.str() << "; " << damping_matrix_log.str();
			//current_values << time_ << "; " << f_ext_log.str() << "; " << position_d_log.str() << "; " << position_log.str() << "; " << stiffness_matrix_log.str() << "; " << damping_matrix_log.str();

			csv_log_ << current_values.str() << "\n";

			matrix_log_ << "\n" << "Stiffness Matrix" << "\n";
			std::ostringstream stiff_mat_log;

			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					stiff_mat_log << stiffness_matrix_(i, j) << "; ";
				}

				stiff_mat_log << "\n";
			}

			matrix_log_ << stiff_mat_log.str();

			matrix_log_ << "\n" << "Damping Matrix" << "\n";
			std::ostringstream damp_mat_log;

			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					damp_mat_log << damping_matrix_(i, j) << "; ";
				}

				damp_mat_log << "\n";
			}

			matrix_log_ << damp_mat_log.str();

			return tau_d_ar;
		}

		Eigen::Matrix<double, 6, 1> impedance_motion_generator::calculate_position_error(const franka::RobotState& robot_state, double time) {
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			if (time >= next_pose_at_ && !poses_.empty()) {
				// get new pose from list
				current_pose_ = poses_.front();
				poses_.pop_front();

				// set next position interval
				next_pose_at_ = next_pose_at_ + pose_interval_;
			}

			// get current desired position and orientation
			Eigen::Affine3d po_d_transform(Eigen::Matrix4d::Map(current_pose_.data()));
			Eigen::Vector3d position_d(po_d_transform.translation());
			Eigen::Quaterniond orientation_d(po_d_transform.linear());

			// get current position and orientation
			Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d position(po_transform.translation());
			Eigen::Quaterniond orientation(po_transform.linear());

			Eigen::Matrix<double, 6, 1> position_error;

			// calculate the position error
			position_error.head(3) << position - position_d; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix

			// calculate orientation error
			if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
				orientation.coeffs() << -orientation.coeffs();
			}

			// "difference" quaternion
			Eigen::Quaterniond diff_quaternion(orientation.inverse() * orientation_d);
			position_error.tail(3) << diff_quaternion.x(), diff_quaternion.y(), diff_quaternion.z();
			// Transform to base frame
			position_error.tail(3) << -po_transform.linear() * position_error.tail(3);

			return position_error;
		}

		double impedance_motion_generator::optimizeDamping(double l_di, double u_di, double mi, double bi, double x0i_max, double derived_x0i_max) {
			// di = min(max(...), ...);
			const double di_max_val = std::max(l_di, ((2 * mi * derived_x0i_max) / ((bi - x0i_max) * exp(1))));
			return std::min(di_max_val, u_di);
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