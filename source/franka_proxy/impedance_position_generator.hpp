/**
 *************************************************************************
 *
 * @file impedance_position_generator.hpp
 *
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__IMPEDANCE_POSITION_GENERATOR_HPP)
#define INCLUDED__FRANKA_PROXY__IMPEDANCE_POSITION_GENERATOR_HPP


#include <Eigen/Core>

#include <franka/robot.h>



namespace franka_proxy
{
	namespace detail
	{


		/**
		 *************************************************************************
		 *
		 * @class impedance_position_generator
		 *
		 * in use
		 *
		 ************************************************************************/
		class impedance_position_generator
		{
		public:
			impedance_position_generator(franka::RobotState& robot_state, std::mutex& state_lock);
			impedance_position_generator
				(franka::RobotState& robot_state,
					std::mutex& state_lock,
					std::list<std::array<double, 16>> poses,
					double duration);

			Eigen::Matrix<double, 6, 1> hold_current_pose(const franka::RobotState& robot_state, double time);

		private:
			std::mutex& state_lock_;
			franka::RobotState& state_;

			std::array<double, 16> current_pose_;
			std::list<std::array<double, 16>> poses_;

			double pose_interval_;
			double next_pose_at_ = 0.0;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HOLD_POSE_HPP) */
