/**
 *************************************************************************
 *
 * @file impedance_position_generator.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "impedance_position_generator.hpp"

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
		// impedance_position_generator
		//
		//////////////////////////////////////////////////////////////////////////


		impedance_position_generator::impedance_position_generator
			(franka::RobotState& robot_state,
			std::mutex& state_lock)
			:
			state_lock_(state_lock),
			state_(robot_state)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			// get initial position
			Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			current_position_ = po_transform_.translation();

			position_interval_ = 0.0;
		}

		impedance_position_generator::impedance_position_generator
		(franka::RobotState& robot_state,
			std::mutex& state_lock,
			std::list<std::array<double, 3>>& positions,
			double duration)
			:
			state_lock_(state_lock),
			state_(robot_state),
			positions_(positions)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			// get initial position
			Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			current_position_ = po_transform_.translation();

			if (duration > 0.0) {
				position_interval_ = positions.size() / duration;
			}
			else {
				position_interval_ = 0.0;
			}
		}

		Eigen::Vector3d impedance_position_generator::hold_current_position(double time) {
			if (std::fmod(time, position_interval_) == 0 && !positions_.empty()) {
				// get new position from list and map position to Vector3d
				std::array<double, 3> position_ar_= positions_.front();
				Eigen::Map<const Eigen::Vector3d> position_(position_ar_.data());

				current_position_ = position_;
				positions_.pop_front();
			}

			return current_position_;
		}



	} /* namespace detail */
} /* namespace franka_proxy */