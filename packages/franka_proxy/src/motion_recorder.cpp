/**
 *************************************************************************
 *
 * @file motion_recorder.cpp
 *
 * todo..., implementation.
 *
 ************************************************************************/

#include "ft_sensor/ft_sensor.hpp"

#include "motion_recorder.hpp"

#include <franka_proxy_share/franka_proxy_logger.hpp>


namespace franka_proxy
{
namespace detail
{
//////////////////////////////////////////////////////////////////////////
//
// franka_motion_recorder
//
//////////////////////////////////////////////////////////////////////////
motion_recorder::motion_recorder(franka::Robot& robot,
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

	if (log_file_path.has_value()) {
		log_ = true;
		file_ = log_file_path.value();
	}


	t_ = std::thread([this]()
	{
		while (!stop_)
		{
			franka::RobotState current_state(robot_.readOnce()); // sync call with approx. 1kHz
			joints_record_.emplace_back(current_state.q);

			if (fts_)
			{
				ft_sensor_response current_ft(fts_->read());
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

	if (log_) {
		if (fts_) {
			logger logger_(file_, 1, 0, 1, 0, 0);
			logger_.start_logging(&joints_, nullptr, &ft_, nullptr, nullptr);

			for (int i = 0; i < joints_record_.size(); i++) {
				logger_.add_joint_data(joints_record_.at(i));
				logger_.add_ft_data(fts_record_.at(i));
				logger_.log();
			}

			logger_.stop_logging();
		}
		else {
			logger logger_(file_, 1, 0, 0, 0, 0);
			logger_.start_logging(&joints_, nullptr, nullptr, nullptr, nullptr);

			for (int i = 0; i < joints_record_.size(); i++) {
				logger_.add_joint_data(joints_record_.at(i));
				logger_.log();
			}

			logger_.stop_logging();
		}
	}

	return {joints_record_, fts_record_};
}


std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> motion_recorder::start(float seconds, std::optional<std::string> log_file_path)
{
	start(log_file_path);
	std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
	return stop();
}
} /* namespace detail */
} /* namespace franka_proxy */
