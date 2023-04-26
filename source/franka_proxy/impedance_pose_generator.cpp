/**
 *************************************************************************
 *
 * @file impedance_pose_generator.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "impedance_pose_generator.hpp"

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
		// impedance_pose_generator
		//
		//////////////////////////////////////////////////////////////////////////


		impedance_pose_generator::impedance_pose_generator
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

			// get initial pose
			current_pose_ = state_.O_T_EE;

			pose_interval_ = 0.0;
		}

		impedance_pose_generator::impedance_pose_generator
		(franka::RobotState& robot_state,
			std::mutex& state_lock,
			std::list<std::array<double, 16>> poses,
			double duration)
			:
			state_lock_(state_lock),
			state_(robot_state),
			poses_(poses)
		{
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			// get initial position
			//Eigen::Affine3d po_transform_(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			//current_position_ = po_transform_.translation();
			current_pose_ = state_.O_T_EE;

			if (duration > 0.0) {
				pose_interval_ = duration / poses.size();
			}
			else {
				pose_interval_ = 0.0;
			}
		}

		std::array<double, 16> impedance_pose_generator::hold_current_pose(double time) {
			if (time >= next_pose_at_ && !poses_.empty()) {
				// get new pose from list
				current_pose_ = poses_.front();
				poses_.pop_front();

				// set next position interval
				next_pose_at_ = next_pose_at_ + pose_interval_;
			}

			return current_pose_;
		}



	} /* namespace detail */
} /* namespace franka_proxy */