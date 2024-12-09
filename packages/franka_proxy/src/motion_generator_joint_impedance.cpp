/**
 *************************************************************************
 *
 * @file motion_generator_joint_impedance.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "motion_generator_joint_impedance.hpp"

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
		// joint_impedance_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		joint_impedance_motion_generator::joint_impedance_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			double duration,
			std::optional<std::string> log_file_path)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration),
			logging_(log_file_path.has_value()),
			logger_(log_file_path.value_or("none"), 3, 0, 2, 1, 0)
		{
			init_impedance_motion_generator(robot, state_lock, robot_state);

			joint_position_interval_ = 0.0;

			if (logging_) {
				// start logging to csv file
				logger_.start_logging(&j_head, nullptr, &f_head, &s_head, nullptr);
			}
		};

		joint_impedance_motion_generator::joint_impedance_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			const std::list<std::array<double, 7>>& joint_positions,
			double duration,
			std::optional<std::string> log_file_path)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration),
			joint_positions_(joint_positions),
			logging_(log_file_path.has_value()),
			logger_(log_file_path.value_or("none"), 3, 0, 2, 1, 0)
		{
			init_impedance_motion_generator(robot, state_lock, robot_state);

			if (duration > 0.0) {
				joint_position_interval_ = duration / joint_positions.size();
			}
			else {
				joint_position_interval_ = 0.0;
			}

			if (logging_) {
				// start logging to csv file
				logger_.start_logging(&j_head, nullptr, &f_head, &s_head, nullptr);
			}
		}

		void joint_impedance_motion_generator::init_impedance_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state) {
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
			calculate_default_stiffness_and_damping();

			// joint position error calculation initialization - initial pose
			current_joint_position_ = state_.q;
		}

		franka::Torques joint_impedance_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period,
			const std::function<Eigen::Matrix<double, 7, 1>(const double)>& get_joint_position_error)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			time_ += period.toSec();

			if (!initialized_) {
				// first call of callback -> no more rotational / translational stiffness changes are allowed
				initialized_ = true;
			}

			if (time_ > duration_) {
				// motion finished
				franka::Torques current_torques(state_.tau_J);
				current_torques.motion_finished = true;

				if (logging_) {
					// close log file
					logger_.stop_logging();
				}

				return current_torques;
			}

			// save timestamp
			timestamps_.push_back(time_);

			// get coriolis matrix (coriolis_ = C x dq_)
			std::array<double, 7> coriolis_ar = model_.coriolis(state_);
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_ar.data());

			// get mass matrix
			std::array<double, 49> mass_ar = model_.mass(state_);
			//Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(mass_ar.data());
			Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(mass_ar.data());

			// get jacobian
			std::array<double, 42> jac_ar = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jac_ar.data());

			// get current joint velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_velocity(state_.dq.data());

			// convert current joint velocity to feed measured joint velocities
			std::array<double, 7> new_measured_joint_velocity;
			Eigen::VectorXd::Map(new_measured_joint_velocity.data(), 7) = joint_velocity;

			measured_joint_velocities_.push_back(new_measured_joint_velocity);

			// calculate delta time for joint acceleration calculation
			double delta_time = timestamps_.back() - timestamps_.front();

			// remove first element of measured_joint_velocitues_ if there are more then eleven elements to calculate the current joint acceleration by the current joint velocity and the joint velocity measured ten cycles ago
			if (measured_joint_velocities_.size() > 11) {
				measured_joint_velocities_.pop_front();
			}

			// calculate joint acceleration
			std::array<double, 7> j_acc_list;

			// avoiding dividing by 0. Also: if no time has passed, no (joint) acceleration could have taken place
			if (delta_time == 0.0) {
				j_acc_list = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
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

			Eigen::Matrix<double, 7, 1> joint_position_error(get_joint_position_error(time_));

			// calculate torque - without gravity as the robot handles it itself
			Eigen::VectorXd tau_d = coriolis - (stiffness_matrix_ * joint_position_error + damping_matrix_ * joint_velocity);

			std::array<double, 7> tau_d_ar;
			Eigen::VectorXd::Map(tau_d_ar.data(), 7) = tau_d;

			if (logging_) {
				// log to csv
				logger_.add_joint_data(tau_d);
				logger_.add_ft_data(stiffness_matrix_(0, 0), stiffness_matrix_(1, 1), stiffness_matrix_(2, 2), stiffness_matrix_(3, 3), stiffness_matrix_(4, 4), stiffness_matrix_(5, 5));
				logger_.add_ft_data(damping_matrix_(0, 0), damping_matrix_(1, 1), damping_matrix_(2, 2), damping_matrix_(3, 3), damping_matrix_(4, 4), damping_matrix_(5, 5));
				logger_.add_single_data(time_);
				logger_.log();
			}

			return tau_d_ar;
		}

		Eigen::Matrix<double, 7, 1> joint_impedance_motion_generator::calculate_joint_position_error(const franka::RobotState& robot_state, double time) {
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			if (time >= next_joint_position_at_ && !joint_positions_.empty()) {
				// get new pose from list
				current_joint_position_ = joint_positions_.front();
				joint_positions_.pop_front();

				// set next position interval
				next_joint_position_at_ = next_joint_position_at_ + joint_position_interval_;
			}

			// get current joint position
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state_.q.data());

			Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_d(current_joint_position_.data());

			Eigen::Matrix<double, 7, 1> joint_position_error = q - q_d;

			if (logging_) {
				logger_.add_joint_data(q_d);
				logger_.add_joint_data(q);
			}

			return joint_position_error;
		}

		double joint_impedance_motion_generator::calculate_damping_from_stiffness(double ki) {
			/**
				critically damped condition

				stiffness ki = (di)^2/4mi
			*/

			return 2.0 * sqrt(ki); // mi = 1
		}

		void joint_impedance_motion_generator::calculate_default_stiffness_and_damping() {
			for (int i = 0; i < stiffness_matrix_.rows(); i++) {
				stiffness_matrix_(i, i) = K_P_[i];
				damping_matrix_(i, i) = calculate_damping_from_stiffness(K_P_[i]);
			}
		}
		
		void joint_impedance_motion_generator::set_stiffness(const std::array<double, 49>& stiffness) {
			if (!initialized_) {
				// set new value
				Eigen::Map<const Eigen::Matrix<double, 7, 7>> new_stiffness_matrix(stiffness.data());
				stiffness_matrix_ = new_stiffness_matrix;

				// operation succeeded
			}
			else {
				throw std::runtime_error("(Joint impedance controller) Setting impedance stiffness after initialization is not allowed!");
			}
		}

		std::array<double, 49> joint_impedance_motion_generator::get_stiffness() {
			std::array<double, 49> stiffness_matrix_ar;

			for (int i = 0; i < 7; i++) {
				for (int j = 0; j < 7; j++) {
					stiffness_matrix_ar[7 * i + j] = stiffness_matrix_(i, j);
				}
			}
			
			return stiffness_matrix_ar;
		}

		std::array<double, 49> joint_impedance_motion_generator::get_damping() {
			std::array<double, 49> damping_matrix_ar;

			for (int i = 0; i < 7; i++) {
				for (int j = 0; j < 7; j++) {
					damping_matrix_ar[7 * i + j] = damping_matrix_(i, j);
				}
			}

			return damping_matrix_ar;
		}

	} /* namespace detail */
} /* namespace franka_proxy */