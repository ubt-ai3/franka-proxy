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
			Eigen::Vector3d hold_current_position(double time);

		private:
			std::mutex& state_lock_;
			franka::RobotState& state_;

			Eigen::Vector3d initial_position_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__MOTION_GENERATOR_IMPEDANCE_HOLD_POSITION_HPP) */
