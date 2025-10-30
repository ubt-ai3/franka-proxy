/**
 *************************************************************************
 *
 * @file motion_recorder.cpp
 *
 * todo..., implementation.
 *
 ************************************************************************/

#include "ft_sensor/ft_sensor.hpp"

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
	if (fts_)
	{
		load_mass_ = fts_->load_mass();
		tool_com_ = franka_proxy_util::tool_center_of_mass_from_fts();
		tool_inertia_matrix_ = franka_proxy_util::tool_inertia_from_fts();
	}
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

			auto current_pose = model_.pose(franka::Frame::kFlange, current_state.q, current_state.F_T_EE,
			                                current_state.EE_T_K);
			Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
			transform *= Eigen::Translation3d(0.0, 0.0, 0.068); // robot flange to DMP-used fts flange
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
			auto p = compute_twist_and_acc(transform, transform, 0.001, prev_velocity_);
			velocity_ = p.first;
			acceleration_ = p.second;

			if (fts_)
			{
				ft_sensor_response current_ft(fts_->read());
				std::array ft_world_arr = compensate_wrench(current_ft, inv_rot);
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


std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>> motion_recorder::compute_twist_and_acc(
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

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_rel, Eigen::ComputeFullU | Eigen::ComputeFullV);
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

	Eigen::Vector3d angular_vel = angular_vel_prev_body;

	twist.block<3, 1>(0, 0) = linear_vel;
	twist.block<3, 1>(3, 0) = angular_vel;

	Eigen::Vector3d prev_linear = prev_twist.block<3, 1>(0, 0);
	Eigen::Vector3d prev_angular = prev_twist.block<3, 1>(3, 0);

	Eigen::Vector3d linear_acc = (linear_vel - prev_linear) / dt;
	Eigen::Vector3d angular_acc = (angular_vel - prev_angular) / dt;

	accel.block<3, 1>(0, 0) = linear_acc;
	accel.block<3, 1>(3, 0) = angular_acc;

	return {twist, accel};
}


std::array<double, 6> motion_recorder::compensate_wrench(
	const ft_sensor_response& current_ft,
	const Eigen::Matrix3d& inv_rot)
{
	using Vector6d = Eigen::Matrix<double, 6, 1>;

	std::array<double, 6> ft_measured = current_ft.data;
	Vector6d ft_vec = Eigen::Map<const Vector6d>(ft_measured.data());

	Eigen::Vector3d c = inv_rot * tool_com_ * load_mass_;
	Eigen::Matrix3d in = inv_rot * tool_inertia_matrix_;

	Eigen::Vector3d f_world = inv_rot * ft_vec.head<3>();

	double a0 = acceleration_(0), a1 = acceleration_(1), a2 = acceleration_(2);
	double a3 = acceleration_(3), a4 = acceleration_(4), a5 = acceleration_(5);

	double v3 = velocity_(3), v4 = velocity_(4), v5 = velocity_(5);

	double g0 = grav_(0), g1 = grav_(1), g2 = grav_(2);

	Eigen::Vector3d f_inertia;
	f_inertia.x() =
		(a0 + g0) * load_mass_ +
		(-(v4 * v4) - v5 * v5) * c.x() +
		(v3 * v4 - a5) * c.y() +
		(v3 * v5 + a4) * c.z();

	f_inertia.y() =
		(a1 + g1) * load_mass_ +
		(v3 * v4 + a5) * c.x() +
		(-(v3 * v3) - v5 * v5) * c.y() +
		(v4 * v5 - a3) * c.z();

	f_inertia.z() =
		(a2 + g2) * load_mass_ +
		(v3 * v5 - a4) * c.x() +
		(v4 * v5 + a3) * c.y() +
		(-(v4 * v4) - v3 * v3) * c.z();

	f_world -= f_inertia;


	Eigen::Vector3d t_world = inv_rot * ft_vec.tail<3>();

	Eigen::Vector3d t_inertia;
	t_inertia.x() =
		(a2 + g2) * c.y() + (-a1 - g1) * c.z() +
		a3 * in(0, 0) + (a4 - v3 * v5) * in(0, 1) + (a5 + v3 * v4) * in(0, 2) +
		-(v4 * v5) * in(1, 1) + (v4 * v4 - v5 * v5) * in(1, 2) + v4 * v5 * in(2, 2);

	t_inertia.y() =
		(-a2 - g2) * c.x() + (a0 + g0) * c.z() +
		v3 * v5 * in(0, 0) + (a3 + v4 * v5) * in(0, 1) + (v5 * v5 - v3 * v3) * in(0, 2) +
		a4 * in(1, 1) + (a5 - v3 * v5) * in(1, 2) + -v3 * v5 * in(2, 2);

	t_inertia.z() =
		(a1 + g1) * c.x() + (-a0 - g0) * c.y() +
		-v3 * v4 * in(0, 0) + (v3 * v3 - v4 * v4) * in(0, 1) + (a3 - v4 * v5) * in(0, 2) +
		v3 * v4 * in(1, 1) + (a4 + v3 * v5) * in(1, 2) + a5 * in(2, 2);

	t_world -= t_inertia;

	std::array ft_world_arr = {
		f_world.x(), f_world.y(), f_world.z(),
		t_world.x(), t_world.y(), t_world.z()
	};

	return ft_world_arr;
}
}
