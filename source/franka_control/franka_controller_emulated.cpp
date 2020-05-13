/**
 *************************************************************************
 *
 * @file franka_controller_emulated.cpp
 *
 * Franka controller to emulate a robot, implementation.
 *
 ************************************************************************/


#include "franka_controller_emulated.hpp"

#include <viral_core/log.hpp>
#include <viral_core/timer.hpp>

#include "exception.hpp"


namespace franka_control
{


using namespace viral_core;


//////////////////////////////////////////////////////////////////////////
//
// franka_controller_emulated
//
//////////////////////////////////////////////////////////////////////////


franka_controller_emulated::franka_controller_emulated()
	:
	speed_factor_(0.f),
	gripper_open_(false),

	state_joint_values_
		((Eigen::Matrix<double, 7, 1>() <<
			0, 0, 0, -0.0698, 0, 0, 0).finished()),

	max_gripper_pos_(50)
{ }


franka_controller_emulated::~franka_controller_emulated() noexcept = default;


void franka_controller_emulated::apply_z_force(double mass, double duration)
{
	LOG_ERROR("Not implemented.");
	throw not_implemented(); // TODO: Throw something else.
}


bool almost_equal(const robot_config_7dof& xes, const robot_config_7dof& array)
{
	for (int64 i = 0; i < 7; ++i)
	{
		if (abs(xes[i] - array[i]) >= 0.1)
			return false;
	}

	return true;
}


robot_config_7dof operator+
	(const robot_config_7dof& xes, const robot_config_7dof& rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] + rhs[i];
	return ret;
}


robot_config_7dof operator-
	(const robot_config_7dof& xes, const robot_config_7dof& rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] - rhs[i];
	return ret;
}


double length(const robot_config_7dof& xes)
{
	double len = 0;

	for (int64 i = 0; i < 7; ++i)
		len += xes[i] * xes[i];

	return sqrt(len);
}


robot_config_7dof operator*(const robot_config_7dof& xes, double rhs)
{
	robot_config_7dof ret;
	for (int i = 0; i < 7; ++i)
		ret[i] = xes[i] * rhs;
	return ret;
}


void franka_controller_emulated::move_to(const robot_config_7dof& target)
{
	robot_config_7dof current_joint_values = current_config();

	step_timer tick_timer(move_update_rate_, 1.f);
	free_timer timer;

	while (!almost_equal(target, current_joint_values))
	{
		tick_timer.try_sleep();
		tick_timer.update();
		if (!tick_timer.has_next_timestep()) continue;

		while (tick_timer.next_timestep())
		{
			// Determine joint-space length each joint has moved
			// since the last call to update().
			double move_length =
				timer.seconds_passed() *
				speed_factor_ *
				max_speed_length_per_sec_;

			timer.restart();

			// Move robot joints by given length.
			double length_to_next =
				length(target - current_joint_values);

			if (length_to_next < move_length)
			{
				// If the next waymark is in reach, move there.
				current_joint_values = target;
			}
			else
			{
				// Move into the direction of the next waymark,
				// but don't actually reach it.
				current_joint_values = current_joint_values +
					(target - current_joint_values) *
						(move_length / length_to_next);
			}

			// Copy from process variables to exposed state.
			// This enables changes of state only on update(),
			// as required by the robot_controller interface.
			MUTEX_SCOPE(controller_mutex_);
			state_joint_values_ = current_joint_values;
		}
	}
}


bool franka_controller_emulated::move_to_until_contact
	(const robot_config_7dof& target)
{
	move_to(target);
	return true;
}


void franka_controller_emulated::open_gripper()
{
	MUTEX_SCOPE(controller_mutex_);
	gripper_open_ = true;
}


void franka_controller_emulated::close_gripper()
{
	MUTEX_SCOPE(controller_mutex_);
	gripper_open_ = false;
}


void franka_controller_emulated::grasp_gripper(double speed, double force)
{
	MUTEX_SCOPE(controller_mutex_);
	gripper_open_ = false;
}


bool franka_controller_emulated::gripper_grasped() const
	{ return false; }


double franka_controller_emulated::speed_factor() const
{
	MUTEX_SCOPE(controller_mutex_);
	return speed_factor_;
}


void franka_controller_emulated::set_speed_factor(double speed_factor)
{
	MUTEX_SCOPE(controller_mutex_);
	speed_factor_ = speed_factor;
}


void franka_controller_emulated::automatic_error_recovery() {}


robot_config_7dof franka_controller_emulated::current_config() const
{
	MUTEX_SCOPE(controller_mutex_);
	return state_joint_values_;
}


int franka_controller_emulated::current_gripper_pos() const
{
	MUTEX_SCOPE(controller_mutex_);
	return gripper_open_ ? max_gripper_pos_ : 0;
}


int franka_controller_emulated::max_gripper_pos() const
{
	return max_gripper_pos_;
}


void franka_controller_emulated::update() {}




} /* namespace franka_control */