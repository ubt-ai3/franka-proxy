/**
 *************************************************************************
 *
 * @file motion_generator_seq_cart_vel_tau.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include <franka_proxy/motion_generator_seq_cart_vel_tau.hpp>

#include <utility>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/model.h>


namespace franka_proxy
{
namespace detail
{


//////////////////////////////////////////////////////////////////////////
//
// seq_cart_vel_tau_generator
//
//////////////////////////////////////////////////////////////////////////


seq_cart_vel_tau_generator::seq_cart_vel_tau_generator
	(std::mutex& current_state_lock,
	 franka::RobotState& current_state,
	 franka::Robot& robot,
	 const std::atomic_bool& stop_motion_flag,
	 std::vector<std::array<double, 7>> q_sequence,
	 std::vector<std::array<double, 6>> f_sequence,
	 std::vector<std::array<double, 6>> selection_vector_sequence)
	:
	current_state_lock_(current_state_lock),
	current_state_(current_state),
	stop_motion_(stop_motion_flag),
	model(robot.loadModel()),
	q_sequence_(std::move(q_sequence)),
	f_sequence_(std::move(f_sequence)),
	selection_vector_sequence_(std::move(selection_vector_sequence)),
	dq_buffer_(dq_filter_size_, eigen_vector7d::Zero()),
	ft_buffer_(ft_filter_size_, Eigen::Matrix<double, 6, 1>::Zero()),
	stiffness_(6, 6),
	damping_(6, 6)/*,
	fts_()*/
{
	stiffness_.setZero();
	stiffness_.topLeftCorner(3, 3) =
		translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
	stiffness_.bottomRightCorner(3, 3) =
		rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);

	damping_.setZero();
	damping_.topLeftCorner(3, 3) =
		.5 * sqrt(translational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
	damping_.bottomRightCorner(3, 3) =
		.5 * sqrt(rotational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
}


seq_cart_vel_tau_generator::~seq_cart_vel_tau_generator()
{
	if (log_)
	{
		try
		{
			std::ofstream csv("motion_log.csv");

			csv << "f0,f1,f2,e0,e1,e2,e3,e4,e5\n";
			for (int i = 0; i < ft_existing_log_.size(); ++i)
				csv << ft_existing_log_[i][0] << ","
					<< ft_existing_log_[i][1] << ","
					<< ft_existing_log_[i][2] << ","
					<< error_log_[i][0] << ","
					<< error_log_[i][1] << ","
					<< error_log_[i][2] << ","
					<< error_log_[i][3] << ","
					<< error_log_[i][4] << ","
					<< error_log_[i][5] << "\n";
			
			std::cout << "did logging" << std::endl;
		}
		catch (std::exception&)
		{
		}
	}
}


franka::Torques seq_cart_vel_tau_generator::step
	(const franka::RobotState& robot_state,
	 franka::Duration period)
{
	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger(); // NOLINT(hicpp-exception-baseclass)


	bool contact_change_motion = false;


	time_ += period.toSec();
	bool motion_finished = false;
	auto current_step = static_cast<size_t>(time_ * 1000.);


	// q_start
	if (time_ == 0.0)
	{
		if ((eigen_vector7d(robot_state.q.data())
			- eigen_vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");
	}

	// q_end
	if (current_step >= q_sequence_.size())
	{
		current_step = q_sequence_.size() - 1;
		motion_finished = true;
	}


	// get target variables
	std::array<double, 7> q_d = q_sequence_[current_step];
	std::array<double, 6> f_d = f_sequence_[current_step];
	std::array<double, 6> selection_vector = selection_vector_sequence_[current_step];


	// get state variables
	update_dq_filter(robot_state);
	Eigen::Map<const eigen_vector7d> tau_measured(robot_state.tau_J.data());

	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<eigen_vector7d> mass(mass_array.data());
	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	Eigen::Map<eigen_vector7d> coriolis(coriolis_array.data());
	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<eigen_vector7d> gravity(gravity_array.data());


	// --- cartesian motion ---

	auto current_pose = model.pose(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
	//Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());


	// calculate pose from desired joints 
	auto desired_pose = model.pose(franka::Frame::kEndEffector, q_d, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(desired_pose.data()));
	Eigen::Vector3d position_d(transform_d.translation());
	Eigen::Quaterniond orientation_d(transform_d.linear());


	// compute error to desired pose
	Eigen::Matrix<double, 6, 1> error;

	// position error
	error.head(3) = position - position_d;

	// orientation error
	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
		orientation.coeffs() = -orientation.coeffs();
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// transform to base frame
	error.tail(3) = -transform_d.linear() * error.tail(3);


	if (error.head(3).norm() > 0.005 &&
		selection_vector[0] == 1 && selection_vector[1] == 1 && selection_vector[2] == 1)
	{
		error.head(3) = error.head(3).normalized() * 0.005;
		contact_change_motion = true;
	}


	// spring damper system with damping ratio=1 and filtered dq
	Eigen::Matrix<double, 6, 1> ft_cartesian_motion =
		-stiffness_ * error - damping_ * (jacobian * compute_dq_filtered());

	// --- cartesian motion end --- 


	// --- force motion ---

	Eigen::Matrix<double, 6, 1> ft_desired(f_d.data());


	//fts_.update();
	//std::array<double, 6> current_fts_values = fts_.current_values();
	//Eigen::Map<const Eigen::Matrix<double, 6, 1>> ft_existing(current_fts_values.data());


	//tau_existing = tau_measured - gravity;
	////auto ft_existing = jacobian * tau_existing;
	//auto ft_existing_array = robot_state.O_F_ext_hat_K;
	//Eigen::Matrix<double, 6, 1> ft_existing_from_tau = 
	//	(jacobian * jacobian.transpose()).inverse() * jacobian * tau_existing;

	// ff
	Eigen::Matrix<double, 6, 1> ft_force = ft_desired;

/*
	// pi controller using fts for neg z-direction
	if (selection_vector[2] == 0)
	{
		int contact_dim = 2;

		if (ft_existing(2) > -1 && ft_desired(contact_dim) < -1) // move to no contact
		{
			contact_change_motion = true;
			ft_force(contact_dim) = -3.0;
		}
		else if (ft_existing(2) > -2.5) // ft sensor value not really useful
		{ }
		else
		{
			double error_fz = (ft_desired(contact_dim) - ft_existing(2));
			double f_z_error_derivate = (error_fz - pre_error_fz_) / period.toSec();
			f_z_error_integral_ += error_fz * period.toSec();
			ft_force(contact_dim) += 0.3 * error_fz + 30.0 * f_z_error_integral_; // +0.0001 * f_z_error_derivate;

			pre_error_fz_ = error_fz;
		}
	}

	// pi controller for neg z-direction	
	if (selection_vector[0] == 0)
	{
		int contact_dim = 0;

		if (ft_existing(2) > -1 && ft_desired(contact_dim) > 1) // move to no contact
		{
			//contact_change_motion = true;
			ft_force(contact_dim) = 5.0;
		}
		else if (ft_existing(2) > -3) // ft sensor value not really useful
		{ }
		else
		{
			f_x_error_integral_ += (ft_desired(contact_dim) - (-ft_existing(2))) * period.toSec();
			ft_force(contact_dim) += 0.2 * (ft_desired(contact_dim) - (-ft_existing(2))) + 5.0 * f_x_error_integral_;
		}
	}
*/
	update_ft_filter(ft_force); // todo use selection vector
	ft_force = compute_ft_filtered();

	//if (ft_existing_from_tau[2] < 0.0)
	//{
	//	force_error_integral_ += period.toSec() * (desired_force_torque_z_force - ft_existing_from_tau);
	//	ft_command += 1.0 * (desired_force_torque_z_force - ft_existing_from_tau) + 1.0 * force_error_integral_;
	//}
	//tau_desired = jacobian.transpose() * desired_force_torque_z_force;
	//tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	//// FF + PI control
	//tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	//desired_mass_ = filter_gain * target_mass + (1 - filter_gain) * desired_mass_;

	Eigen::Matrix<double, 6, 1> ft_task;
	for (int i = 0; i < 6; ++i)
		ft_task[i] = selection_vector[i] * ft_cartesian_motion[i] + (1 - selection_vector[i]) * ft_force[i];

	// --- force motion end ---


	// --- compute control ---
	Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * ft_task;
	Eigen::Matrix<double, 7, 1> tau_d = tau_task + coriolis;


	if (log_)
	{
		pose_log_.emplace_back(transform);
		pose_d_log_.emplace_back(transform_d);
		error_log_.emplace_back(error);
		ft_log_.emplace_back(ft_force);
		//ft_existing_log_.emplace_back(ft_existing);
	}


	if (contact_change_motion)
		time_ -= 0.001; // period.toSec(); // stay at the same step  todo better doc/ hack atm


	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
	franka::Torques output(tau_d_array);
	output.motion_finished = motion_finished;
	return output;
}


void seq_cart_vel_tau_generator::update_dq_filter(const franka::RobotState& robot_state)
{
	dq_buffer_[dq_current_filter_position_] = eigen_vector7d(robot_state.dq.data());

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


Eigen::Matrix<double, 7, 1> seq_cart_vel_tau_generator::compute_dq_filtered()
{
	eigen_vector7d value(eigen_vector7d::Zero());

	for (size_t i = 0; i < dq_filter_size_; ++i)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}


void seq_cart_vel_tau_generator::update_ft_filter(const Eigen::Matrix<double, 6, 1>& current_ft)
{
	ft_buffer_[ft_current_filter_position_] = current_ft;

	ft_current_filter_position_ = (ft_current_filter_position_ + 1) % ft_filter_size_;
}


Eigen::Matrix<double, 6, 1> seq_cart_vel_tau_generator::compute_ft_filtered()
{
	Eigen::Matrix<double, 6, 1> value(Eigen::Matrix<double, 6, 1>::Zero());

	for (size_t i = 0; i < ft_filter_size_; ++i)
		value += ft_buffer_[i];

	return value / ft_filter_size_;
}




//////////////////////////////////////////////////////////////////////////
//
// seq_cart_vel_tau_generator_wo_fts
//
//////////////////////////////////////////////////////////////////////////


seq_cart_vel_tau_generator_wo_fts::seq_cart_vel_tau_generator_wo_fts
(std::mutex& current_state_lock,
	franka::RobotState& current_state,
	franka::Robot& robot,
	const std::atomic_bool& stop_motion_flag,
	std::vector<std::array<double, 7>> q_sequence,
	std::vector<std::array<double, 6>> f_sequence,
	std::vector<std::array<double, 6>> selection_vector_sequence)
	:
	current_state_lock_(current_state_lock),
	current_state_(current_state),
	stop_motion_(stop_motion_flag),
	model(robot.loadModel()),
	q_sequence_(std::move(q_sequence)),
	f_sequence_(std::move(f_sequence)),
	selection_vector_sequence_(std::move(selection_vector_sequence)),
	dq_buffer_(dq_filter_size_, eigen_vector7d::Zero()),
	ft_buffer_(ft_filter_size_, Eigen::Matrix<double, 6, 1>::Zero()),
	stiffness_(6, 6),
	damping_(6, 6)
{
	stiffness_.setZero();
	stiffness_.topLeftCorner(3, 3) =
		translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
	stiffness_.bottomRightCorner(3, 3) =
		rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);

	damping_.setZero();
	damping_.topLeftCorner(3, 3) =
		.5 * sqrt(translational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
	damping_.bottomRightCorner(3, 3) =
		.5 * sqrt(rotational_stiffness_) * Eigen::MatrixXd::Identity(3, 3);
}


seq_cart_vel_tau_generator_wo_fts::~seq_cart_vel_tau_generator_wo_fts()
{
	if (log_)
		std::cout << "~seq_cart_vel_tau_generator_wo_fts" << std::endl;
}


franka::Torques seq_cart_vel_tau_generator_wo_fts::step
(const franka::RobotState& robot_state,
	franka::Duration period)
{
	{
		std::lock_guard<std::mutex> state_guard(current_state_lock_);
		current_state_ = robot_state;
	}

	if (stop_motion_)
		throw stop_motion_trigger(); // NOLINT(hicpp-exception-baseclass)


	bool contact_change_motion = false;


	time_ += period.toSec();
	bool motion_finished = false;
	auto current_step = static_cast<size_t>(time_ * 1000.);


	// q_start
	if (time_ == 0.0)
	{
		if ((eigen_vector7d(robot_state.q.data())
			- eigen_vector7d(q_sequence_.front().data())).norm() > 0.01)
			throw std::runtime_error("Aborting; too far away from starting pose!");
	}

	// q_end
	if (current_step >= q_sequence_.size())
	{
		current_step = q_sequence_.size() - 1;
		motion_finished = true;
	}


	// get target variables
	std::array<double, 7> q_d = q_sequence_[current_step];
	std::array<double, 6> f_d = f_sequence_[current_step];
	std::array<double, 6> selection_vector = selection_vector_sequence_[current_step];


	// get state variables
	update_dq_filter(robot_state);
	Eigen::Map<const eigen_vector7d> tau_measured(robot_state.tau_J.data());

	std::array<double, 42> jacobian_array =
		model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

	std::array<double, 49> mass_array = model.mass(robot_state);
	Eigen::Map<eigen_vector7d> mass(mass_array.data());
	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	Eigen::Map<eigen_vector7d> coriolis(coriolis_array.data());
	std::array<double, 7> gravity_array = model.gravity(robot_state);
	Eigen::Map<eigen_vector7d> gravity(gravity_array.data());


	// --- cartesian motion ---

	auto current_pose = model.pose(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
	//Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());


	// calculate pose from desired joints 
	auto desired_pose = model.pose(franka::Frame::kEndEffector, q_d, robot_state.F_T_EE, robot_state.EE_T_K);
	Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(desired_pose.data()));
	Eigen::Vector3d position_d(transform_d.translation());
	Eigen::Quaterniond orientation_d(transform_d.linear());


	// compute error to desired pose
	Eigen::Matrix<double, 6, 1> error;

	// position error
	error.head(3) = position - position_d;

	// orientation error
	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
		orientation.coeffs() = -orientation.coeffs();
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// transform to base frame
	error.tail(3) = -transform_d.linear() * error.tail(3);


	if (error.head(3).norm() > 0.005 &&
		selection_vector[0] == 1 && selection_vector[1] == 1 && selection_vector[2] == 1)
	{
		error.head(3) = error.head(3).normalized() * 0.005;
		contact_change_motion = true;
	}


	// spring damper system with damping ratio=1 and filtered dq
	Eigen::Matrix<double, 6, 1> ft_cartesian_motion =
		-stiffness_ * error - damping_ * (jacobian * compute_dq_filtered());

	// --- cartesian motion end --- 


	// --- force motion ---

	Eigen::Matrix<double, 6, 1> ft_desired(f_d.data());

	// ff
	Eigen::Matrix<double, 6, 1> ft_force = ft_desired;


	update_ft_filter(ft_force); // todo use selection vector
	ft_force = compute_ft_filtered();

	//if (ft_existing_from_tau[2] < 0.0)
	//{
	//	force_error_integral_ += period.toSec() * (desired_force_torque_z_force - ft_existing_from_tau);
	//	ft_command += 1.0 * (desired_force_torque_z_force - ft_existing_from_tau) + 1.0 * force_error_integral_;
	//}
	//tau_desired = jacobian.transpose() * desired_force_torque_z_force;
	//tau_error_integral += period.toSec() * (tau_desired - tau_existing);
	//// FF + PI control
	//tau_command = tau_desired + k_p * (tau_desired - tau_existing) + k_i * tau_error_integral;

	// Smoothly update the mass to reach the desired target value.
	//desired_mass_ = filter_gain * target_mass + (1 - filter_gain) * desired_mass_;

	Eigen::Matrix<double, 6, 1> ft_task;
	for (int i = 0; i < 6; ++i)
		ft_task[i] = selection_vector[i] * ft_cartesian_motion[i] + (1 - selection_vector[i]) * ft_force[i];

	// --- force motion end ---


	// --- compute control ---
	Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * ft_task;
	Eigen::Matrix<double, 7, 1> tau_d = tau_task + coriolis;


	if (log_)
	{
		pose_log_.emplace_back(transform);
		pose_d_log_.emplace_back(transform_d);
		error_log_.emplace_back(error);
		ft_log_.emplace_back(ft_force);
	}


	if (contact_change_motion)
		time_ -= 0.001; // period.toSec(); // stay at the same step  todo better doc/ hack atm


	std::array<double, 7> tau_d_array{};
	Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
	franka::Torques output(tau_d_array);
	output.motion_finished = motion_finished;
	return output;
}


void seq_cart_vel_tau_generator_wo_fts::update_dq_filter(const franka::RobotState& robot_state)
{
	dq_buffer_[dq_current_filter_position_] = eigen_vector7d(robot_state.dq.data());

	dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}


Eigen::Matrix<double, 7, 1> seq_cart_vel_tau_generator_wo_fts::compute_dq_filtered()
{
	eigen_vector7d value(eigen_vector7d::Zero());

	for (size_t i = 0; i < dq_filter_size_; ++i)
		value += dq_buffer_[i];

	return value / dq_filter_size_;
}


void seq_cart_vel_tau_generator_wo_fts::update_ft_filter(const Eigen::Matrix<double, 6, 1>& current_ft)
{
	ft_buffer_[ft_current_filter_position_] = current_ft;

	ft_current_filter_position_ = (ft_current_filter_position_ + 1) % ft_filter_size_;
}


Eigen::Matrix<double, 6, 1> seq_cart_vel_tau_generator_wo_fts::compute_ft_filtered()
{
	Eigen::Matrix<double, 6, 1> value(Eigen::Matrix<double, 6, 1>::Zero());

	for (size_t i = 0; i < ft_filter_size_; ++i)
		value += ft_buffer_[i];

	return value / ft_filter_size_;
}




} /* namespace detail */
} /* namespace franka_proxy */
