
#include "motion_generator_joint_imp_ple.hpp"

#include <utility>
#include <cmath>

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
		// ple_motion_generator
		//
		//////////////////////////////////////////////////////////////////////////


		ple_motion_generator::ple_motion_generator
		(franka::Robot& robot,
			std::mutex& state_lock,
			franka::RobotState& robot_state,
			double speed,
			double duration,
			const std::optional<std::string>& log_file_path)
			:
			model_(robot.loadModel()),
			state_lock_(state_lock),
			state_(robot_state),
			desired_speed_(speed),
			duration_(duration),
			logging_(log_file_path.has_value()),
			sensor_(placeholder_, placeholder_, "./assets/fts-config.json"),
			logger_(log_file_path.value_or("none"), 0, 1, 1, 1, 3)
		{
			sensor_.set_load_mass(no_mass_);

			init_ple_motion_generator(robot, state_lock, robot_state);

			if (logging_) {
				logger_.start_logging(nullptr, &cart_, &ft_, &t_, &grav_);
			}
		};

		void ple_motion_generator::init_ple_motion_generator(franka::Robot& robot, std::mutex& state_lock, franka::RobotState& robot_state) {
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
		}

		franka::Torques ple_motion_generator::callback
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

				if (desired_speed_ < 0.4) {
					speed_factor_ = desired_speed_;
				}
				else
				{
					speed_factor_ = 0.3;
					accelerate_ = true;
				}
			}

			if (done_) {
				// motion finished
				franka::Torques current_torques(state_.tau_J);
				current_torques.motion_finished = true;

				if (logging_) {
					// write log file
					logger_.stop_logging();
				}

				return current_torques;
			}

			if (accelerate_) {
				speed_factor_ += acceleration_;
				if (speed_factor_ > desired_speed_)
				{
					speed_factor_ = desired_speed_;
					accelerate_ = false;
				}
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

			// get jacobian EE
			std::array<double, 42> jac_ar = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jac_ar.data());

			// get current joint velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_velocity(state_.dq.data());

			// convert current joint velocity to feed measured joint velocities
			std::array<double, 7> new_measured_joint_velocity;
			Eigen::VectorXd::Map(&new_measured_joint_velocity[0], 7) = joint_velocity;

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

			// keep going normally for specified duration, then gradually slow down to a stop
			if (time_ < duration_)
				x_ += speed_factor_ * period.toSec();
			else
			{
				speed_factor_ *= decel_factor_;
				if (speed_factor_ < 0.01) 
					done_ = true;
				x_ += speed_factor_ * period.toSec();
			}

			Eigen::Matrix<double, 7, 1> joint_position_error(get_joint_position_error(time_));

			// calculate torque - without gravity as the robot handles it itself
			Eigen::VectorXd tau_d = coriolis - (stiffness_matrix_ * joint_position_error + damping_matrix_ * joint_velocity);

			std::array<double, 7> tau_d_ar;
			Eigen::VectorXd::Map(&tau_d_ar[0], 7) = tau_d;

			return tau_d_ar;
		}


		Eigen::Matrix<double, 7, 1> ple_motion_generator::calculate_ple_motion(const franka::RobotState& robot_state, double time) 
		{
			// get current joint position and measurements
			std::array<double, 7> j = state_.q;
			std::array<double, 6> ft = sensor_.read().data;


			// get jacobian flange
			std::array<double, 42> jac_ar = model_.zeroJacobian(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_flange(jac_ar.data());
			// get current joint velocity
			Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_velocity(state_.dq.data());

			Eigen::Matrix<double, 6, 1> flange_velocities = jacobian_flange * joint_velocity;

			Eigen::Matrix<double, 3, 1> v;
			v << flange_velocities(0), flange_velocities(1), flange_velocities(2);
			Eigen::Matrix<double, 3, 1> w;
			w << flange_velocities(3), flange_velocities(4), flange_velocities(5);

			Eigen::Matrix<double, 3, 1> g;
			g << 0, 0, -1;
			

			std::array<double, 16> ee_p = model_.pose(franka::Frame::kEndEffector, state_);
			Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_pose(ee_p.data());

			auto trafo = (ee_pose * Eigen::Affine3d::Identity()).rotation();

			g = trafo * g;
			v = trafo * v;
			w = trafo * w;

			Eigen::Matrix<double, 6, 1> sensor_velos;
			sensor_velos << v, w;

			//static int i = 900;
			//i++;
			//if (i % 1000 == 0)
			//	std::cout << g.transpose() << "\n";

			if (logging_) {
				logger_.add_ft_data(ft);
				logger_.add_cart_data(sensor_velos);
				logger_.add_single_data(time);
				std::vector<std::string> grav = { std::to_string(g(0)), std::to_string(g(1)), std::to_string(g(2)) };
				logger_.add_arbitrary_data(grav);

				logger_.log();
			}

			Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(j.data());

			const double x = x_;
			
			// calculate next motion step (i.e., this is where the pre-defined motion is implemented)
			Eigen::Matrix<double, 7, 1> q_d{0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461};
			Eigen::Matrix<double, 7, 1> start_offset{ 0.0800004 , 0.200001   ,  0.4 , 0. ,0.0300006  , 0.0300007, 0. };
			Eigen::Matrix<double, 7, 1> offset;
			offset << 0.4 * (0.3 * std::sin(x * 0.6 * M_PI) - 0.3 * std::cos(x * 0.5 * M_PI) + 0.2 * std::sin(x * M_PI) + 0.5 * std::cos(x * M_PI)),
				0.4 * (0.2 * std::sin(x * 0.5 * M_PI) + 0.1 * std::cos(x * 0.5 * M_PI) + 0.2 * std::sin(x * M_PI) + 0.3 * std::cos(x * M_PI) + 0.2 * std::sin(x * 1.5 * M_PI) + 0.1 * std::cos(x * 1.5 * M_PI)),
				0.4 * (0.2 * std::sin(x * 0.7 * M_PI) + 0.3 * std::cos(x * 0.5 * M_PI) - 0.2 * std::sin(x * M_PI) + 0.2 * std::cos(x * M_PI) + 0.1 * std::sin(x * 1.5 * M_PI) + 0.3 * std::cos(x * 1.5 * M_PI) - 0.1 * std::sin(x * 2.0 * M_PI) + 0.2 * std::cos(x * 2.0 * M_PI)),
				2. * (0.4 * (0.3 * std::sin(x * 0.8 * M_PI) - 0.2 * std::cos(x * 0.5 * M_PI) + 0.4 * std::sin(x * M_PI) + 0.2 * std::cos(x * M_PI) - 0.2 * std::sin(x * 1.5 * M_PI) + 0.1 * std::cos(x * 1.5 * M_PI) - 0.1 * std::cos(x * 2.0 * M_PI))),
				0.3 * (-0.1 * std::cos(x * 0.5 * M_PI) - 0.1 * std::sin(x * M_PI) + 0.3 * std::cos(x * M_PI) + 0.2 * std::sin(x * 1.5 * M_PI) + 0.1 * std::cos(x * 1.5 * M_PI) + 0.2 * std::sin(x * 2.0 * M_PI) - 0.2 * std::cos(x * 2.0 * M_PI) + 0.3 * std::sin(x * 3.0 * M_PI)),
				0.3 * (0.1 * std::sin(x * 0.5 * M_PI) + 0.2 * std::sin(x * M_PI) - 0.2 * std::cos(x * M_PI) - 0.1 * std::sin(x * 1.5 * M_PI) + 0.2 * std::cos(x * 1.5 * M_PI) + 0.3 * std::sin(x * 2.0 * M_PI) + 0.1 * std::cos(x * 2.0 * M_PI) + 0.2 * std::sin(x * 3.0 * M_PI) * std::sin(x * 3.0 * M_PI)),
				0.3 * (0.3 * std::sin(x * M_PI) + 0.1 * std::cos(x * M_PI) - 0.3 * std::cos(x * 1.5 * M_PI) + 0.1 * std::sin(x * 2.0 * M_PI) + 0.2 * std::cos(x * 2.0 * M_PI) + 0.2 * std::sin(x * 3.0 * M_PI));


			q_d = q_d + offset - start_offset;


			Eigen::Matrix<double, 7, 1> joint_position_error = q - q_d;

			return joint_position_error;
		}

		double ple_motion_generator::calculate_damping_from_stiffness(double ki) {
			/**
				critically damped condition

				stiffness ki = (di)^2/4mi
			*/

			return 2.0 * sqrt(ki); // mi = 1
		}

		void ple_motion_generator::calculate_default_stiffness_and_damping() {
			for (int i = 0; i < stiffness_matrix_.rows(); i++) {
				stiffness_matrix_(i, i) = K_P_[i];
				damping_matrix_(i, i) = calculate_damping_from_stiffness(K_P_[i]);
			}
		}

		void ple_motion_generator::set_stiffness(const std::array<double, 49>& stiffness) {
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

		std::array<double, 49> ple_motion_generator::get_stiffness() {
			std::array<double, 49> stiffness_matrix_ar;

			for (int i = 0; i < 7; i++) {
				for (int j = 0; j < 7; j++) {
					stiffness_matrix_ar[7 * i + j] = stiffness_matrix_(i, j);
				}
			}

			return stiffness_matrix_ar;
		}

		std::array<double, 49> ple_motion_generator::get_damping() {
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