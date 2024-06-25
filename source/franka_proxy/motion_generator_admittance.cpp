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
			double duration,
			const std::optional<std::string>& log_file_path)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration),
			impedance_controller_(robot, state_lock, robot_state, duration, false, log_file_path),
			logging_(log_file_path.has_value()),
			logger_(log_file_path.value_or("none"), 1, 3, 3, 1, 42)
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

			// initialize stiffness and damping matrix
			calculate_stiffness_and_damping();

			// set impedance controller default stiffness and damping parameter
			impedance_controller_.set_rotational_stiffness(10); // always true
			impedance_controller_.set_translational_stiffness(150); // always true

			if (logging_) {
				// start logging to csv file
				logger_.start_logging(&j_head, &c_head, &f_head, &s_head, &a_head);
			}
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

			if (time_ > duration_) {
				// motion finished
				franka::Torques current_torques(state_.tau_J);
				current_torques.motion_finished = true;

				if (logging_) {
					logger_.stop_logging();
				}

				return current_torques;
			}

			// save timestamp
			timestamps_.push_back(time_);

			// get current joint position
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state_.q.data());


			// get current position and orientation
			Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));

			Eigen::Vector3d current_position(po_transform.translation());
			Eigen::Quaterniond orientation(po_transform.linear());
			
			// todo fix this init
			if (!initialized_) 
				previous_quaternion_ = orientation;
			
			double flip = ((previous_quaternion_ * orientation.conjugate()).w() > 0 ? -1 : 1);

			Eigen::Matrix<double, 6, 1> position_eq;
			position_eq.head(3) << current_position;
			position_eq.tail(3) << orientation.x(), orientation.y(), orientation.z();

			position_eq.tail(3) << po_transform.linear() 
				* flip
				* position_eq.tail(3);

			previous_quaternion_.coeffs() << -flip * orientation.coeffs();

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
			Eigen::Matrix<double, 6, 6> inertia_matrix = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse();

			// get f ext 
			std::array<double, 6> f_ext_ar = state_.O_F_ext_hat_K;
			Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_ext_(f_ext_ar.data());
			
			// filter model discrepancy noise
			for (int i = 0; i < 6; i++) {
				if (i < 3) {
					if (f_ext_ar[i] < 0.0) {
						f_ext_ar[i] = f_ext_ar[i] + std::min(2.0, abs(f_ext_ar[i]));
					}
					else {
						f_ext_ar[i] = f_ext_ar[i] - std::min(2.0, abs(f_ext_ar[i]));
					}
				}
				else {
					if (f_ext_ar[i] < 0.0) {
						f_ext_ar[i] = f_ext_ar[i] + std::min(1.0, abs(f_ext_ar[i]));
					}
					else {
						f_ext_ar[i] = f_ext_ar[i] - std::min(1.0, abs(f_ext_ar[i]));
					}
				}
			}

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
			//double delta_time = 0.001;
			// calculate delta time for acceleration and joint acceleration calculation
			double delta_time = timestamps_.back() - timestamps_.front();

			if (timestamps_.size() > 1) {
			timestamps_.pop_front();
			}

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

			if (logging_) {
				// log to csv
				logger_.add_joint_data(state_.q);
				logger_.add_cart_data(x_i);
				logger_.add_cart_data(position_eq);
				logger_.add_cart_data(x_i_prod2);
				logger_.add_ft_data(f_ext_);
				logger_.add_ft_data(current_force);
				logger_.add_ft_data(f_ext_middle);
				logger_.add_single_data(time_);
				for (int i = 0; i < 6; i++) {
					for (int j = 0; j < 7; j++) {
						logger_.add_arbitrary_data(std::to_string(jacobian(i, j)));
					}
				}
			}

			return impedance_controller_.callback
			(state_, period,
				[&](const double time) -> Eigen::Matrix<double, 6, 1>
				{
					return position_eq - x_i;
				}
			);
		}

		void admittance_motion_generator::calculate_stiffness_and_damping() {
			stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
			stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness_) *
				Eigen::MatrixXd::Identity(3, 3);
			damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness_) *
				Eigen::MatrixXd::Identity(3, 3);
		}

		void admittance_motion_generator::set_admittance_rotational_stiffness(double rotational_stiffness) {
			if (!initialized_) {
				rotational_stiffness_ = rotational_stiffness;
				calculate_stiffness_and_damping();

				// operation succeeded
			}
			else {
				throw std::runtime_error("Setting admittance rotational stiffness after initialization is not allowed!");
			}
		}

		void admittance_motion_generator::set_admittance_translational_stiffness(double translational_stiffness) {
			if (!initialized_) {
				translational_stiffness_ = translational_stiffness;
				calculate_stiffness_and_damping();

				// operation succeeded
			}
			else {
				throw std::runtime_error("Setting admittance translational stiffness after initialization is not allowed!");
			}
		}

		void admittance_motion_generator::set_impedance_rotational_stiffness(double rotational_stiffness) {
			if (!initialized_) {
				impedance_controller_.set_rotational_stiffness(rotational_stiffness);
			}
			else {
				throw std::runtime_error("(Admittance controller) Setting impedance rotational stiffness after initialization is not allowed!");
			}
		}

		void admittance_motion_generator::set_impedance_translational_stiffness(double translational_stiffness) {
			if (!initialized_) {
				impedance_controller_.set_translational_stiffness(translational_stiffness);
			}
			else {
				throw std::runtime_error("(Admittance controller) Setting impedance translational stiffness after initialization is not allowed!");
			}
		}

		double admittance_motion_generator::get_admittance_rotational_stiffness() {
			return rotational_stiffness_;
		}

		double admittance_motion_generator::get_admittance_translational_stiffness() {
			return translational_stiffness_;
		}

		double admittance_motion_generator::get_impedance_rotational_stiffness() {
			return impedance_controller_.get_rotational_stiffness();
		}

		double admittance_motion_generator::get_impedance_translational_stiffness() {
			return impedance_controller_.get_translational_stiffness();
		}

	} /* namespace detail */
} /* namespace franka_proxy */