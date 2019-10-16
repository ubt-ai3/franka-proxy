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

#include "exception.hpp"
#include "viral_core/timer.hpp"


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
	speed_normalized_(0.f),
	gripper_open_(false),

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


void franka_controller_emulated::move_to(const robot_config_7dof& target)
{
	next_waymark_ = target;

	while (!almost_equal(next_waymark_, current_joint_values_))
		viral_core::thread_util::sleep_seconds(0.01f);
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


void franka_controller_emulated::close_gripper(double speed, double force)
{
	MUTEX_SCOPE(controller_mutex_);
	gripper_open_ = false;
}


bool franka_controller_emulated::gripper_grasped() const
	{ return false; }


double franka_controller_emulated::speed_factor() const
{
	MUTEX_SCOPE(controller_mutex_);
	return speed_normalized_;
}


void franka_controller_emulated::set_speed_factor(double speed_factor)
{
	MUTEX_SCOPE(controller_mutex_);
	speed_normalized_ = speed_factor;
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


void franka_controller_emulated::update()
{
	MUTEX_SCOPE(controller_mutex_);


	// Determine joint-space length each joint has moved
	// since the last call to update().
	double move_length =
		timer_.seconds_passed() *
		speed_normalized_ *
		max_speed_length_per_sec_;

	timer_.restart();


	// Move robot joints by given length.
	// We possibly must fetch new waymarks,
	// maybe multiple times in one update().
	while (true)
	{
		double length_to_next =
			length(next_waymark_ - current_joint_values_);


		// If the next waymark is in reach, move there.
		// Further handling in the above.
		if (length_to_next < move_length)
		{
			current_joint_values_ = next_waymark_;
			move_length -= length_to_next;
			continue;
		}


		// Move into the direction of the next waymark,
		// but don't actually reach it.
		current_joint_values_ = current_joint_values_ +
			(next_waymark_ - current_joint_values_) *
				(move_length / length_to_next);

		break;
	}


	// Copy from process variables to exposed state.
	// This enables changes of state only on update(),
	// as required by the robot_controller interface.
	state_joint_values_ = current_joint_values_;
}




} /* namespace franka_control */