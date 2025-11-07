/**
 *************************************************************************
 *
 * @file motion_recorder.cpp
 *
 * todo..., implementation.
 *
 ************************************************************************/

#include "ft_sensor/ft_sensor.hpp"

#include <algorithm>
#include <utility>

#include <franka_proxy_share/franka_proxy_logger.hpp>

#include "motion_recorder.hpp"

#include "franka_proxy_util.hpp"


namespace franka_proxy::detail
{
//////////////////////////////////////////////////////////////////////////
//
// franka_motion_recorder
//
//////////////////////////////////////////////////////////////////////////
motion_recorder::motion_recorder(
	franka::Robot& robot,
	franka::RobotState& robot_state,
	ft_sensor* fts)
	: robot_(robot),
	  robot_state_(robot_state),
	  fts_(fts),
	  model_(robot.loadModel())
{
}


void motion_recorder::start(std::optional<std::string> log_file_path)
{
	stop_ = false;
	joints_record_.clear();
	fts_record_.clear();

	if (log_file_path.has_value())
	{
		log_ = true;
		file_ = log_file_path.value();
	}


	t_ = std::thread([this]()
	{
		while (!stop_)
		{
			franka::RobotState current_state(robot_.readOnce()); // sync call with approx. 1kHz

			auto current_pose = 
				model_.pose(franka::Frame::kFlange, current_state.q, current_state.F_T_EE, current_state.EE_T_K);
			Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
			transform *= Eigen::Translation3d(0.0, 0.0, 0.068); // robot flange to fts flange
			transform *= Eigen::AngleAxisd(45.0 * franka_proxy_util::deg_to_rad, Eigen::Vector3d(0.0, 0.0, -1.0));
			Eigen::Matrix3d inv_rot = transform.inverse().linear();

			if (!prev_existing_)
			{
				prev_transform_ = transform;
				prev_velocity_ = Eigen::Matrix<double, 6, 1>::Zero();
				velocity_ = Eigen::Matrix<double, 6, 1>::Zero();
				acceleration_ = Eigen::Matrix<double, 6, 1>::Zero();
				prev_existing_ = true;
				continue;
			}


			prev_velocity_ = velocity_;
			constexpr double dt = 0.001;
			auto p = compute_twist_and_acc_in_world(prev_transform_, transform, dt, prev_velocity_);

			auto alpha_from_tau = [](double dt_local, double tau)-> double
			{
				if (tau <= 0.0) return 1.0;
				if (dt_local <= 0.0) return 0.0;
				double a = dt_local / (tau + dt_local);
				a = std::max(a, 0.0);
				a = std::min(a, 1.0);
				return a;
			};

			const double a_lin = alpha_from_tau(dt, tau_v_lin_);
			const double a_ang = alpha_from_tau(dt, tau_v_ang_);

			Eigen::Matrix<double, 6, 1> twist_raw = p.first;

			if (!filter_initialized_)
			{
				twist_filtered_ = twist_raw;
				twist_filtered_prev_ = twist_filtered_;
				filter_initialized_ = true;
				velocity_ = twist_filtered_;
				acceleration_.setZero();
			}
			else
			{
				twist_filtered_prev_ = twist_filtered_;
				twist_filtered_.segment<3>(0) = (1.0 - a_lin) * twist_filtered_.segment<3>(0) + a_lin * twist_raw.segment<3>(0);
				twist_filtered_.segment<3>(3) = (1.0 - a_ang) * twist_filtered_.segment<3>(3) + a_ang * twist_raw.segment<3>(3);

				constexpr double safe_dt = dt > 1e-9 ? dt : 1e-9;
				Eigen::Matrix<double, 6, 1> acc_from_filt = Eigen::Matrix<double, 6, 1>::Zero();
				acc_from_filt.segment<3>(0) = (twist_filtered_.segment<3>(0) - twist_filtered_prev_.segment<3>(0)) / safe_dt;
				acc_from_filt.segment<3>(3) = (twist_filtered_.segment<3>(3) - twist_filtered_prev_.segment<3>(3)) / safe_dt;

				velocity_ = twist_filtered_;
				acceleration_ = acc_from_filt;
			}

			prev_transform_ = transform;


			if (fts_)
			{
				ft_sensor_response current_ft(fts_->read());
				std::array ft_world_arr = fts_->compensate_tool_wrench(current_ft, inv_rot, velocity_, acceleration_);
				fts_record_.emplace_back(ft_world_arr);
			}

			joints_record_.emplace_back(current_state.q);
			robot_state_ = current_state;
		}
	});
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> motion_recorder::stop()
{
	stop_ = true;
	t_.join();

	// remove first 100 elements because of noise and filter init
	if (joints_record_.size() < 101) return {};
	joints_record_.erase(joints_record_.begin(), joints_record_.begin() + 100);
	if (fts_) fts_record_.erase(fts_record_.begin(), fts_record_.begin() + 100);

	if (log_)
	{
		if (fts_)
		{
			logger logger_(file_, 1, 0, 1, 0, 0);
			logger_.start_logging(&joints_, nullptr, &ft_, nullptr, nullptr);

			for (int i = 0; i < joints_record_.size(); i++)
			{
				logger_.add_joint_data(joints_record_.at(i));
				logger_.add_ft_data(fts_record_.at(i));
				logger_.log();
			}

			logger_.stop_logging();
		}
		else
		{
			logger logger_(file_, 1, 0, 0, 0, 0);
			logger_.start_logging(&joints_, nullptr, nullptr, nullptr, nullptr);

			for (const auto& joint_data : joints_record_)
			{
				logger_.add_joint_data(joint_data);
				logger_.log();
			}

			logger_.stop_logging();
		}
	}

	return {joints_record_, fts_record_};
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> motion_recorder::start(
	float seconds,
	std::optional<std::string> log_file_path)
{
	start(std::move(log_file_path));
	std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
	return stop();
}


std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>> motion_recorder::compute_twist_and_acc_in_world(
	const Eigen::Affine3d& prev_transform,
	const Eigen::Affine3d& transform,
	double dt,
	const Eigen::Matrix<double, 6, 1>& prev_twist)
{
	Eigen::Matrix<double, 6, 1> twist;
	twist.setZero();
	Eigen::Matrix<double, 6, 1> accel;
	accel.setZero();

	if (dt <= 0.0)
		return {twist, accel};

	const Eigen::Vector3d p = transform.translation();
	const Eigen::Vector3d p_prev = prev_transform.translation();
	const Eigen::Matrix3d R = transform.linear();
	const Eigen::Matrix3d R_prev = prev_transform.linear();

	Eigen::Vector3d linear_vel = (p - p_prev) / dt;

	Eigen::Matrix3d R_rel = R_prev.transpose() * R;

	Eigen::JacobiSVD svd(R_rel, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d R_rel_orth = svd.matrixU() * svd.matrixV().transpose();

	Eigen::AngleAxisd aa(R_rel_orth);
	double angle = aa.angle();
	constexpr double EPS_ANGLE = 1e-8;

	Eigen::Vector3d angular_vel_prev_body;
	if (std::abs(angle) < EPS_ANGLE)
	{
		// small-angle approximation: omega \approx vee((R_rel - R_rel^T)/2) / dt
		Eigen::Matrix3d skew = 0.5 * (R_rel_orth - R_rel_orth.transpose());
		Eigen::Vector3d vee;
		vee.x() = skew(2, 1);
		vee.y() = skew(0, 2);
		vee.z() = skew(1, 0);
		angular_vel_prev_body = vee / dt;
	}
	else
	{
		angular_vel_prev_body = aa.axis() * angle / dt;
	}

	Eigen::Vector3d angular_vel_world = R_prev * angular_vel_prev_body;

	twist.block<3, 1>(0, 0) = linear_vel;
	twist.block<3, 1>(3, 0) = angular_vel_world;

	Eigen::Vector3d prev_linear = prev_twist.block<3, 1>(0, 0);
	Eigen::Vector3d prev_angular = prev_twist.block<3, 1>(3, 0);

	Eigen::Vector3d linear_acc = (linear_vel - prev_linear) / dt;
	Eigen::Vector3d angular_acc = (angular_vel_world - prev_angular) / dt;

	accel.block<3, 1>(0, 0) = linear_acc;
	accel.block<3, 1>(3, 0) = angular_acc;

	return {twist, accel};
}
}
