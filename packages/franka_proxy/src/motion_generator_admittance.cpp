/**
 *************************************************************************
 *
 * @file motion_generator_admittance.cpp
 *
 ************************************************************************/

#include "motion_generator_admittance.hpp"

#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/model.h>


namespace franka_proxy::detail
{
//////////////////////////////////////////////////////////////////////////
//
// admittance_motion_generator
//
//////////////////////////////////////////////////////////////////////////
admittance_motion_generator::admittance_motion_generator(
	franka::Robot& robot,
	std::mutex& state_lock,
	franka::RobotState& robot_state,
	double duration,
	double adm_rotational_stiffness,
	double adm_translational_stiffness,
	double imp_rotational_stiffness,
	double imp_translational_stiffness,
	const std::optional<std::string>& log_file_path)
	: model_(robot.loadModel()),
	  state_lock_(state_lock),
	  state_(robot_state),
	  duration_(duration),
	  translational_stiffness_(adm_translational_stiffness),
	  rotational_stiffness_(adm_rotational_stiffness),
	  impedance_controller_(robot, state_lock, robot_state, duration, false, log_file_path),
	  logging_(log_file_path.has_value()),
	  logger_(log_file_path.value_or("none"), 1, 3, 3, 1, 42)
{
	{
		std::lock_guard state_guard(state_lock_);
		state_ = robot_state;
	}

	// load model and set collisions
	model_ = robot.loadModel();
	robot.setCollisionBehavior(
		{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
		{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
		{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
		{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

	// initialize stiffness and damping matrix
	calculate_stiffness_and_damping();

	// set impedance controller default stiffness and damping parameter
	impedance_controller_.set_rotational_stiffness(imp_rotational_stiffness);
	impedance_controller_.set_translational_stiffness(imp_translational_stiffness);

	if (logging_)
		initialize_logging();
}


franka::Torques admittance_motion_generator::callback(
	const franka::RobotState& robot_state,
	franka::Duration period)
{
	{
		std::lock_guard state_guard(state_lock_);
		state_ = robot_state;
	}

	time_ += period.toSec();

	if (time_ > duration_)
	{
		// motion finished
		franka::Torques current_torques(state_.tau_J);
		current_torques.motion_finished = true;

		if (logging_)
		{
			logger_.stop_logging();
		}

		return current_torques;
	}

	// save timestamp
	timestamps_.push_back(time_);

	// get current position and orientation
	Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));

	Eigen::Vector3d current_position(po_transform.translation());
	Eigen::Quaterniond orientation(po_transform.linear());

	if (!initialized_)
		previous_quaternion_ = orientation;

	double flip = (previous_quaternion_ * orientation.conjugate()).w() > 0 ? -1 : 1;

	Eigen::Matrix<double, 6, 1> position_eq;
	position_eq.head(3) << current_position;
	position_eq.tail(3) << orientation.x(), orientation.y(), orientation.z();
	position_eq.tail(3) << po_transform.linear()
		* flip
		* position_eq.tail(3);

	previous_quaternion_.coeffs() << -flip * orientation.coeffs();

	// x_i-1 and x_i-2 are required for further calculations
	if (!initialized_)
	{
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
	for (int i = 0; i < 6; i++)
	{
		if (i < 3)
			if (f_ext_ar[i] < 0.0)
				f_ext_ar[i] = f_ext_ar[i] + std::min(2.0, abs(f_ext_ar[i]));
			else
				f_ext_ar[i] = f_ext_ar[i] - std::min(2.0, abs(f_ext_ar[i]));
		else if (f_ext_ar[i] < 0.0)
			f_ext_ar[i] = f_ext_ar[i] + std::min(1.0, abs(f_ext_ar[i]));
		else
			f_ext_ar[i] = f_ext_ar[i] - std::min(1.0, abs(f_ext_ar[i]));
	}

	// add measured f_ext to array
	f_exts_.push_front(f_ext_ar);

	// calculate f_ext from last measurements
	std::array f_ext_middle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	std::list f_exts_it(f_exts_);

	for (int i = 0; i < f_exts_.size(); i++)
	{
		std::array<double, 6> current_el = f_exts_it.front();
		f_exts_it.pop_front();

		f_ext_middle[0] = f_ext_middle[0] + current_el[0];
		f_ext_middle[1] = f_ext_middle[1] + current_el[1];
		f_ext_middle[2] = f_ext_middle[2] + current_el[2];
		f_ext_middle[3] = f_ext_middle[3] + current_el[3];
		f_ext_middle[4] = f_ext_middle[4] + current_el[4];
		f_ext_middle[5] = f_ext_middle[5] + current_el[5];

		if (i == f_exts_.size() - 1)
		{
			auto size = f_exts_.size();

			f_ext_middle[0] = f_ext_middle[0] / size;
			f_ext_middle[1] = f_ext_middle[1] / size;
			f_ext_middle[2] = f_ext_middle[2] / size;
			f_ext_middle[3] = f_ext_middle[3] / size;
			f_ext_middle[4] = f_ext_middle[4] / size;
			f_ext_middle[5] = f_ext_middle[5] / size;
		}
	}

	if (f_exts_.size() == 41)
	{
		f_exts_.pop_back();
	}

	// set current force for further calculations
	Eigen::Map<Eigen::Matrix<double, 6, 1>> current_force(f_ext_middle.data());

	// using constant as using actual timestamps causing too much noise
	//double delta_time = 0.001;
	// calculate delta time for acceleration and joint acceleration calculation
	double delta_time = timestamps_.back() - timestamps_.front();

	if (timestamps_.size() > 1)
		timestamps_.pop_front();

	// calculate new position
	const Eigen::Matrix<double, 6, 6> x_i_prod1 = (stiffness_matrix_ * (delta_time * delta_time)
		+ damping_matrix_ * delta_time + inertia_matrix).inverse();

	const Eigen::Matrix<double, 6, 1> x_i_sum1 = delta_time * delta_time * (-current_force + stiffness_matrix_ *
		position_eq);
	const Eigen::Matrix<double, 6, 1> x_i_sum2 = delta_time * damping_matrix_ * x_i_1_;
	const Eigen::Matrix<double, 6, 1> x_i_sum3 = inertia_matrix * (2 * x_i_1_ - x_i_2_);
	const Eigen::Matrix<double, 6, 1> x_i_prod2 = x_i_sum1 + x_i_sum2 + x_i_sum3;

	Eigen::Matrix<double, 6, 1> x_i = x_i_prod1 * x_i_prod2;

	// set new x_i-1 and x_i-2
	x_i_2_ = x_i_1_;
	x_i_1_ = x_i;

	if (logging_)
	{
		logger_.add_joint_data(state_.q);
		logger_.add_cart_data(x_i);
		logger_.add_cart_data(position_eq);
		logger_.add_cart_data(x_i_prod2);
		logger_.add_ft_data(f_ext_);
		logger_.add_ft_data(current_force);
		logger_.add_ft_data(f_ext_middle);
		logger_.add_single_data(time_);
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 7; j++)
				logger_.add_arbitrary_data(std::to_string(jacobian(i, j)));
	}

	return impedance_controller_.callback(
		state_, period, [&](const double time) -> Eigen::Matrix<double, 6, 1>
		{
			return position_eq - x_i;
		}
	);
}


void admittance_motion_generator::calculate_stiffness_and_damping()
{
	stiffness_matrix_.topLeftCorner(3, 3) << translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
	stiffness_matrix_.bottomRightCorner(3, 3) << rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
	damping_matrix_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness_) *
		Eigen::MatrixXd::Identity(3, 3);
	damping_matrix_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness_) *
		Eigen::MatrixXd::Identity(3, 3);
}


void admittance_motion_generator::initialize_logging()
{
	std::vector<std::string> j_head = {"q1", "q2", "q3", "q4", "q5", "q7", "q6"};
	std::vector<std::string> c_head = {
		"x_i_0", "x_i_1", "x_i_2", "x_i_3", "x_i_4", "x_i_5",
		"pos_eq_0", "pos_eq_1", "pos_eq_2", "pos_eq_3", "pos_eq_4", "pos_eq_5",
		"x_i_prod2_0", "x_i_prod2_1", "x_i_prod2_2", "x_i_prod2_3", "x_i_prod2_4", "x_i_prod2_5"
	};
	std::vector<std::string> f_head = {
		"f_ext_0", "f_ext_1", "f_ext_2", "f_ext_3", "f_ext_4", "f_ext_5",
		"cur_f_0", "cur_f_1", "cur_f_2", "cur_f_3", "cur_f_4", "cur_f_5",
		"f_ext_m_0", "f_ext_m_1", "f_ext_m_2", "f_ext_m_3", "f_ext_m_4", "f_ext_m_5"
	};
	std::vector<std::string> s_head = {"time"};
	std::vector<std::string> a_head = {
		"jac_0_0", "jac_0_1", "jac_0_2", "jac_0_3", "jac_0_4", "jac_0_5", "jac_0_6",
		"jac_1_0", "jac_1_1", "jac_1_2", "jac_1_3", "jac_1_4", "jac_1_5", "jac_1_6",
		"jac_2_0", "jac_2_1", "jac_2_2", "jac_2_3", "jac_2_4", "jac_2_5", "jac_2_6",
		"jac_3_0", "jac_3_1", "jac_3_2", "jac_3_3", "jac_3_4", "jac_3_5", "jac_3_6",
		"jac_4_0", "jac_4_1", "jac_4_2", "jac_4_3", "jac_4_4", "jac_4_5", "jac_4_6",
		"jac_5_0", "jac_5_1", "jac_5_2", "jac_5_3", "jac_5_4", "jac_5_5", "jac_5_6"
	};
	// start logging to csv file
	logger_.start_logging(&j_head, &c_head, &f_head, &s_head, &a_head);
}
}
