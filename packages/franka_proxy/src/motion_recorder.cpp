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
			load_mass_ = franka_proxy_util::tool_mass_from_fts();
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
					const double dt = 0.001;
					auto p = compute_twist_and_acc_in_world(prev_transform_, transform, dt, prev_velocity_);

					// === Begin EWMA filtering of twist; derive acceleration from filtered twist ===
					static Eigen::Matrix<double, 6, 1> twist_filt = Eigen::Matrix<double, 6, 1>::Zero();
					static Eigen::Matrix<double, 6, 1> twist_filt_prev = Eigen::Matrix<double, 6, 1>::Zero();
					static bool filt_init = false;

					// Tunable time constants (seconds)
					static double tau_v_lin = 0.02; // 8 Hz
					static double tau_v_ang = 0.05; // 3 Hz

					auto alpha_from_tau = [](double dt_local, double tau)->double {
						if (tau <= 0.0) return 1.0;
						if (dt_local <= 0.0) return 0.0;
						double a = dt_local / (tau + dt_local);
						if (a < 0.0) a = 0.0;
						if (a > 1.0) a = 1.0;
						return a;
						};

					const double a_lin = alpha_from_tau(dt, tau_v_lin);
					const double a_ang = alpha_from_tau(dt, tau_v_ang);

					Eigen::Matrix<double, 6, 1> twist_raw = p.first;

					if (!filt_init) {
						twist_filt = twist_raw;
						twist_filt_prev = twist_filt;
						filt_init = true;
						velocity_ = twist_filt;
						acceleration_.setZero();
					}
					else {
						twist_filt_prev = twist_filt;
						// Low-pass linear components
						twist_filt.segment<3>(0) = (1.0 - a_lin) * twist_filt.segment<3>(0) + a_lin * twist_raw.segment<3>(0);
						// Low-pass angular components
						twist_filt.segment<3>(3) = (1.0 - a_ang) * twist_filt.segment<3>(3) + a_ang * twist_raw.segment<3>(3);

						const double safe_dt = (dt > 1e-9) ? dt : 1e-9;
						Eigen::Matrix<double, 6, 1> acc_from_filt = Eigen::Matrix<double, 6, 1>::Zero();
						acc_from_filt.segment<3>(0) = (twist_filt.segment<3>(0) - twist_filt_prev.segment<3>(0)) / safe_dt;
						acc_from_filt.segment<3>(3) = (twist_filt.segment<3>(3) - twist_filt_prev.segment<3>(3)) / safe_dt;

						velocity_ = twist_filt;
						acceleration_ = acc_from_filt;
					}
					// === End EWMA filtering ===

					prev_transform_ = transform;


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

		return { joints_record_, fts_record_ };
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
			return { twist, accel };

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

		Eigen::Vector3d angular_vel_world = R_prev * angular_vel_prev_body;

		twist.block<3, 1>(0, 0) = linear_vel;
		twist.block<3, 1>(3, 0) = angular_vel_world;

		Eigen::Vector3d prev_linear = prev_twist.block<3, 1>(0, 0);
		Eigen::Vector3d prev_angular = prev_twist.block<3, 1>(3, 0);

		Eigen::Vector3d linear_acc = (linear_vel - prev_linear) / dt;
		Eigen::Vector3d angular_acc = (angular_vel_world - prev_angular) / dt;

		accel.block<3, 1>(0, 0) = linear_acc;
		accel.block<3, 1>(3, 0) = angular_acc;

		return { twist, accel };
	}


	std::array<double, 6> motion_recorder::compensate_wrench(
		const ft_sensor_response& current_ft,
		const Eigen::Matrix3d& inv_rot)
	{
		using Vector6d = Eigen::Matrix<double, 6, 1>;

		// Map measured wrench (tool frame) to Eigen vector
		std::array<double, 6> ft_measured = current_ft.data;
		const Vector6d ft_vec = Eigen::Map<const Vector6d>(ft_measured.data());

		// Recover R = transform.linear() (rotation tool->world)
		// inv_rot = R^{-1}  =>  R = inv_rot.transpose()
		const Eigen::Matrix3d R = inv_rot.transpose();

		// COM in world (meters)  position only (do NOT multiply by mass here)
		const Eigen::Vector3d d = R * tool_com_; // sensor origin -> COM (world)

		// Rotate inertia to world: I_world = R * I_tool * R^T
		Eigen::Matrix3d I_world = R * tool_inertia_matrix_ * R.transpose();

		// Shift inertia to sensor origin if tool_inertia_matrix_ is about COM (parallel-axis theorem)
		const double m = load_mass_;
		const double d2 = d.squaredNorm();
		const Eigen::Matrix3d parallel = m * (d2 * Eigen::Matrix3d::Identity() - d * d.transpose());
		const Eigen::Matrix3d I_about_sensor = I_world + parallel;

		// Rotate measured wrench into world frame
		const Eigen::Vector3d f_meas_world = R * ft_vec.head<3>();
		const Eigen::Vector3d t_meas_world = R * ft_vec.tail<3>();

		// Extract linear acc, angular vel/acc (all assumed already in world frame)
		const Eigen::Vector3d a_world = acceleration_.block<3, 1>(0, 0);
		const Eigen::Vector3d omega_world = velocity_.block<3, 1>(3, 0);
		const Eigen::Vector3d alpha_world = acceleration_.block<3, 1>(3, 0);

		// COM acceleration: a_c = a + alpha x d + omega x (omega x d)
		const Eigen::Vector3d a_c = a_world + alpha_world.cross(d) + omega_world.cross(omega_world.cross(d));

		// Inertial wrench components
		const Eigen::Vector3d f_inertial = m * a_c;
		const Eigen::Vector3d tau_inertial = I_about_sensor * alpha_world
			+ omega_world.cross(I_about_sensor * omega_world)
			+ d.cross(m * a_c);

		// Compensated wrench in world frame
		const Eigen::Vector3d f_sensor = inv_rot * (f_meas_world - f_inertial);
		const Eigen::Vector3d t_sensor = inv_rot * (t_meas_world - tau_inertial);

		// Pack as [f_x,f_y,f_z,t_x,t_y,t_z]
		return { f_sensor.x(), f_sensor.y(), f_sensor.z(),
				 t_sensor.x(), t_sensor.y(), t_sensor.z() };
	}

}
