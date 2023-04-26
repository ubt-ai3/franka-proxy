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

			// get initial pose
			current_pose_ = state_.O_T_EE;

			pose_interval_ = 0.0;
		}

		impedance_position_generator::impedance_position_generator
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

		Eigen::Matrix<double, 6, 1> impedance_position_generator::hold_current_pose(double time) {
			{
				std::lock_guard<std::mutex> state_guard(state_lock_);
				state_ = robot_state;
			}

			if (time >= next_pose_at_ && !poses_.empty()) {
				// get new pose from list
				current_pose_ = poses_.front();
				poses_.pop_front();

				// set next position interval
				next_pose_at_ = next_pose_at_ + pose_interval_;
			}

			// get current desired position and orientation
			Eigen::Affine3d po_d_transform(Eigen::Matrix4d::Map(current_pose_.data()));
			Eigen::Vector3d position_d(po_d_transform.translation());
			Eigen::Quaterniond orientation_d(po_d_transform.linear());

			// get current position and orientation
			Eigen::Affine3d po_transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
			Eigen::Vector3d position(po_transform.translation());
			Eigen::Quaterniond orientation(po_transform.linear());

			Eigen::Matrix<double, 6, 1> position_error;

			// calculate the position error
			position_error.head(3) << position - position_d; // transforming to 6x6 as the position error will be mulitplied with the stiffness matrix

			// calculate orientation error
			if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
				orientation.coeffs() << -orientation.coeffs();
			}

			// "difference" quaternion
			Eigen::Quaterniond diff_quaternion(orientation.inverse() * orientation_d);
			position_error.tail(3) << diff_quaternion.x(), diff_quaternion.y(), diff_quaternion.z();
			// Transform to base frame
			position_error.tail(3) << -po_transform.linear() * position_error.tail(3);

			return position_error;
		}



	} /* namespace detail */
} /* namespace franka_proxy */