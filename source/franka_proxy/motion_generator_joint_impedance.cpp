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
			bool logging)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			duration_(duration),
			logging_(logging)
		{
			init_impedance_motion_generator(robot, state_lock, robot_state);

			pose_interval_ = 0.0;

			if (logging_) {
				// start logging to csv file
				csv_log_1_.open("joint_impedance_log_1.csv");
				csv_log_1_ << csv_header1_ << "\n";
				csv_log_2_.open("joint_impedance_log_2.csv");
				csv_log_2_ << csv_header2_ << "\n";
			}
		};

		joint_impedance_motion_generator::joint_impedance_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			std::list<std::array<double, 7>> joint_positions,
			double duration,
			bool logging)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			joint_positions_(joint_positions),
			duration_(duration),
			logging_(logging)
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
				csv_log_1_.open("joint_impedance_log_1.csv");
				csv_log_1_ << csv_header1_ << "\n";
				csv_log_2_.open("joint_impedance_log_2.csv");
				csv_log_2_ << csv_header2_ << "\n";
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
			current_pose_ = state_.O_T_EE;
		}

		franka::Torques joint_impedance_motion_generator::callback
		(const franka::RobotState& robot_state,
			franka::Duration period,
			std::function<Eigen::Matrix<double, 7, 1>(const double)> get_joint_position_error)
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
					csv_log_1_.close();
					csv_log_2_.close();
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

			// calculate inertia matrix !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// intertia = (J(q)*B^(-1)(q)*J(q).transpose())^(-1)
			// Eigen::Matrix<double, 6, 6> inertia_matrix = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse();

			// get current joint velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state_.dq.data());

			// convert current joint velocity to feed measured joint velocities
			std::array<double, 7> new_measured_joint_velocity;
			Eigen::VectorXd::Map(&new_measured_joint_velocity[0], 7) = dq;

			measured_joint_velocities_.push_back(new_measured_joint_velocity);

			// calculate delta time for joint acceleration calculation
			double delta_time = timestamps_.back() - timestamps_.front();

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

			Eigen::Matrix<double, 7, 1> joint_position_error(get_joint_position_error(time_));

			// calculate external force
			/*Eigen::Matrix<double, 6, 1> f_ext = inertia_matrix * acceleration + damping_matrix_ * velocity + stiffness_matrix_ * position_error;

			for (int i = 3; i < 6; i++) {
				f_ext(i) = f_ext(i) * 0.5;
			}*/

			// calculate torque - without gravity as the robot handles it itself !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// Eigen::VectorXd tau_d = mass_matrix * j_acceleration + coriolis - jacobian.transpose() * f_ext;
			Eigen::VectorXd tau_d = stiffness_matrix_ * joint_position_error + damping_matrix_ * joint_velocity; // inertia_matrix * joint_acceleration;

			std::array<double, 7> tau_d_ar;
			Eigen::VectorXd::Map(&tau_d_ar[0], 7) = tau_d;
			
			if (logging_) {
				// log to csv
				std::ostringstream tau_d_log;
				tau_d_log << tau_d(0) << "; " << tau_d(1) << "; " << tau_d(2) << "; " << tau_d(3) << "; " << tau_d(4) << "; " << tau_d(5) << "; " << tau_d(6);
				std::ostringstream stiffness_matrix_log;
				stiffness_matrix_log << stiffness_matrix_(0, 0) << "; " << stiffness_matrix_(1, 1) << "; " << stiffness_matrix_(2, 2) << "; " << stiffness_matrix_(3, 3) << "; " << stiffness_matrix_(4, 4) << "; " << stiffness_matrix_(5, 5);
				std::ostringstream damping_matrix_log;
				damping_matrix_log << damping_matrix_(0, 0) << "; " << damping_matrix_(1, 1) << "; " << damping_matrix_(2, 2) << "; " << damping_matrix_(3, 3) << "; " << damping_matrix_(4, 4) << "; " << damping_matrix_(5, 5);

				std::ostringstream current_values;
				current_values << time_ << "; " << tau_d_log.str() << "; " << stiffness_matrix_log.str() << "; " << damping_matrix_log.str();

				csv_log_1_ << current_values.str() << "\n";
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
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state_.dq.data());

			Eigen::Matrix<double, 7, 1> joint_position_error = dq - current_joint_position_;

			if (logging_) {
				std::ostringstream joint_position_d_log;
				std::ostringstream joint_position_log;

				joint_position_d_log << current_joint_position_(0) << "; " << current_joint_position_(1) << "; " << current_joint_position_(2) << "; " << current_joint_position_(3) << "; " << current_joint_position_(4) << "; " << current_joint_position_(5) << "; " << current_joint_position_(6);
				joint_position_log << dq(0) << "; " << dq(1) << "; " << dq(2) << "; " << dq(3) << "; " << dq(4) << "; " << dq(5) << "; " << dq(6);

				std::ostringstream current_values;
				current_values << time_ << "; " << joint_position_d_log.str() << "; " << joint_position_log.str();

				csv_log_2_ << current_values.str() << "\n";
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
				stiffness_matrix_(i, i) = K_P[i];
				damping_matrix_(i, i) = K_D[i]; // TODO: use calculate damping from stiffness method
			}
		}

		bool joint_impedance_motion_generator::set_stiffness(std::array<double, 49> stiffness) {
			if (initialized_) {
				// no changes allowed -> return false as operation failed
				return false;
			}
			else {
				// set new value
				Eigen::Map<const Eigen::Matrix<double, 7, 7>> new_stiffness_matrix(stiffness.data());
				stiffness_matrix_ = new_stiffness_matrix;

				// operation succeeded -> return true
				return true;
			}
		}

		std::array<double, 49> joint_impedance_motion_generator::get_stiffness() {
			std::array<double, 49> stiffness_matrix_ar;
			Eigen::VectorXd::Map(&stiffness_matrix_ar[0], 49) = stiffness_matrix_;

			return stiffness_matrix_ar;
		}

		bool joint_impedance_motion_generator::set_damping(std::array<double, 49> damping) {
			if (initialized_) {
				// no changes allowed -> return false as operation failed
				return false;
			}
			else {
				// set new value
				Eigen::Map<const Eigen::Matrix<double, 7, 7>> new_damping_matrix(damping.data());
				damping_matrix_ = new_damping_matrix;

				// operation succeeded -> return true
				return true;
			}
		}

		std::array<double, 49> joint_impedance_motion_generator::get_damping() {
			std::array<double, 49> damping_matrix_ar;
			Eigen::VectorXd::Map(&damping_matrix_ar[0], 49) = damping_matrix_;

			return damping_matrix_ar;
		}


	} /* namespace detail */
} /* namespace franka_proxy */