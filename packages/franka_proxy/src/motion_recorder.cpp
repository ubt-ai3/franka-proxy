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


namespace franka_proxy
{
namespace detail
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
	  fts_(fts)
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
			joints_record_.emplace_back(current_state.q);

			auto current_pose = model.pose(franka::Frame::kFlange, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
			Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_pose.data()));
			// robot flange to DMP-used fts flange
			transform *= Eigen::Translation3d(0.0, 0.0, 0.068);
			transform *= Eigen::AngleAxisd(45.0 * franka_proxy_util::deg_to_rad, Eigen::Vector3d(0.0, 0.0, -1.0));
			Eigen::Vector3d position(transform.translation());
			Eigen::Matrix3d curr_rot = transform.linear();
			Eigen::Matrix3d inv_rot = transform.inverse().linear();
			Eigen::Quaterniond orientation(curr_rot);
			Eigen::Vector3d rotation = franka_proxy_util::get_euler_angles(curr_rot);
			Eigen::Matrix<double, 6, 1> cartesian;
			cartesian << position, rotation;

			if (fts_)
			{
				ft_sensor_response current_ft(fts_->read());

				std::array<double, 6> ft_measured = current_ft.data;
				Eigen::Vector<double, 6> ft_vec = Eigen::Vector<double, 6>::Map(ft_measured.data());
				Eigen::Vector3d c = (inv_rot * tool_com_) * load_mass_;
				Eigen::Matrix3d in = inv_rot * tool_iner_;
				Eigen::Vector3d f_world = inv_rot * ft_vec.head(3);
				Eigen::Vector3d f_inertia;
				f_inertia << (((acceleration_(0) + grav_(0)) * load_mass_) + ((-pow(velocity_(4), 2) - pow(velocity_(5), 2)) * c(0)) + (((velocity_(3) * velocity_(4)) - acceleration_(5)) * c(1)) + (((velocity_(3) * velocity_(5)) + acceleration_(4)) * c(2))),
					(((acceleration_(1) + grav_(1)) * load_mass_) + (((velocity_(3) * velocity_(4)) + acceleration_(5)) * c(0)) + ((-pow(velocity_(3), 2) - pow(velocity_(5), 2)) * c(1)) + (((velocity_(4) * velocity_(5)) - acceleration_(3)) * c(2))),
					(((acceleration_(2) + grav_(2)) * load_mass_) + (((velocity_(3) * velocity_(5)) - acceleration_(4)) * c(0)) + (((velocity_(4) * velocity_(5)) + acceleration_(3)) * c(1)) + ((-pow(velocity_(4), 2) - pow(velocity_(3), 2)) * c(2)));
				f_world -= f_inertia;
				Eigen::Vector3d f_meas = curr_dmp_.get_world_to_task() * f_world;
				Eigen::Vector3d t_world = inv_rot * ft_vec.tail(3);
				Eigen::Vector3d t_inertia;
				t_inertia << (((acceleration_(2) + grav_(2)) * c(1)) + ((-acceleration_(1) - grav_(1)) * c(2)) + (acceleration_(3) * in(0, 0)) + ((acceleration_(4) - (velocity_(3) * velocity_(5))) * in(0, 1)) + ((acceleration_(5) + (velocity_(3) * velocity_(4))) * in(0, 2)) + ((-(velocity_(4) * velocity_(5))) * in(1, 1)) + ((pow(velocity_(4), 2) - pow(velocity_(5), 2)) * in(1, 2)) + ((velocity_(4) * velocity_(5)) * in(2, 2))),
					(((-acceleration_(2) - grav_(2)) * c(0)) + ((acceleration_(0) + grav_(0)) * c(2)) + ((velocity_(3) * velocity_(5)) * in(0, 0)) + ((acceleration_(3) + (velocity_(4) * velocity_(5))) * in(0, 1)) + ((pow(velocity_(5), 2) - pow(velocity_(3), 2)) * in(0, 2)) + (acceleration_(4) * in(1, 1)) + ((acceleration_(5) - (velocity_(3) * velocity_(5))) * in(1, 2)) + ((-velocity_(3) * velocity_(5)) * in(2, 2))),
					(((acceleration_(1) + grav_(1)) * c(0)) + ((-acceleration_(0) - grav_(0)) * c(1)) + ((-velocity_(3) * velocity_(4)) * in(0, 0)) + ((pow(velocity_(3), 2) - pow(velocity_(4), 2)) * in(0, 1)) + ((acceleration_(3) - (velocity_(4) * velocity_(5))) * in(0, 2)) + ((velocity_(3) * velocity_(4)) * in(1, 1)) + ((acceleration_(4) + (velocity_(3) * velocity_(5))) * in(1, 2)) + (acceleration_(5) * in(2, 2)));
				t_world -= t_inertia;
				Eigen::Vector3d t_meas = curr_dmp_.get_world_to_task() * t_world;

				fts_record_.emplace_back(current_ft.data);
			}
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
} /* namespace detail */
} /* namespace franka_proxy */
