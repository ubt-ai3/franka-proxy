/**
 *************************************************************************
 *
 * @file franka_controller_remote.cpp
 *
 * Franka controller to emulate a robot, implementation.
 *
 ************************************************************************/


#include "franka_controller_remote.hpp"

#include <franka_proxy_client/franka_remote_interface.hpp>


namespace franka_control
{


//////////////////////////////////////////////////////////////////////////
//
// franka_emulated_controller
//
//////////////////////////////////////////////////////////////////////////


franka_controller_remote::franka_controller_remote
	(const std::string& ip)
	:
	controller_(new franka_proxy::franka_remote_interface(ip)),

	speed_factor_(0.1)
{
	controller_->set_speed_factor(speed_factor_);
}


franka_controller_remote::~franka_controller_remote() noexcept = default;


void franka_controller_remote::move(const robot_config_7dof& target)
{
	controller_->move_to
		(franka_proxy::robot_config_7dof
			{target[0], target[1], target[2],
			 target[3], target[4], target[5], target[6]});
}

void franka_controller_remote::move_with_force(const robot_config_7dof& target, const force_torque_config_cartesian& target_force_torques)
{
	move(target);
}


bool franka_controller_remote::move_until_contact
	(const robot_config_7dof& target)
{
	return controller_->move_to_until_contact
		(franka_proxy::robot_config_7dof
			{target[0], target[1], target[2],
			 target[3], target[4], target[5], target[6]});
}


void franka_controller_remote::open_gripper()
	{ controller_->open_gripper(); }
void franka_controller_remote::close_gripper()
	{ controller_->close_gripper(); }
void franka_controller_remote::grasp_gripper(double speed, double force)
	{ controller_->grasp_gripper(speed, force); }


bool franka_controller_remote::gripper_grasped() const
	{ return controller_->gripper_grasped(); }


double franka_controller_remote::speed_factor() const
{
	std::lock_guard<std::mutex> l(state_lock_);
	return speed_factor_;
}


void franka_controller_remote::set_speed_factor(double speed_factor)
{
	{
		std::lock_guard<std::mutex> l(state_lock_);
		speed_factor_ = speed_factor;
	}
	controller_->set_speed_factor(speed_factor_);
}


void franka_controller_remote::automatic_error_recovery()
	{ controller_->automatic_error_recovery(); }


robot_config_7dof franka_controller_remote::current_config() const
{
	robot_config_7dof ret;
	franka_proxy::robot_config_7dof config = controller_->current_config();
	for (int i = 0; i < 7; ++i)
		ret[i] = config[i];
	return ret;
}

force_torque_config_cartesian franka_controller_remote::current_force_torque() const
{
	return force_torque_config_cartesian();
}

int franka_controller_remote::current_gripper_pos() const
	{ return static_cast<int>( controller_->current_gripper_pos() * gripper_unit_per_m_); }
int franka_controller_remote::max_gripper_pos() const
	{ return static_cast<int>(controller_->max_gripper_pos() * gripper_unit_per_m_); }
void franka_controller_remote::update()
	{ controller_->update(); }


void franka_controller_remote::start_recording()
{
	controller_->start_recording();
}


std::pair<std::vector<robot_config_7dof>, std::vector<force_torque_config_cartesian>> franka_controller_remote::stop_recording()
{
	std::vector<robot_config_7dof> joints;
	std::vector<force_torque_config_cartesian> forces;

	auto [recorded_joints, recorded_forces] = controller_->stop_recording();

	joints.reserve(recorded_joints.size());
	for (auto datum : recorded_joints)
		joints.emplace_back(datum.data());

	forces.reserve(recorded_forces.size());
	for (auto datum : recorded_forces)
		forces.emplace_back(datum.data());

	return {joints, forces};
}


void franka_controller_remote::move_sequence(
	std::vector<std::array<double, 7>> q_sequence,
	std::vector<std::array<double, 6>> f_sequence,
	std::vector<std::array<double, 6>> selection_vector_sequence)
{
	controller_->move_to(q_sequence.front());
	controller_->move_sequence(q_sequence, f_sequence, selection_vector_sequence);
	controller_->move_to(q_sequence.back());
}

void franka_controller_remote::set_fts_bias(const Eigen::Vector<double, 6>& bias)
{
	controller_->set_fts_bias({ bias[0], bias[1], bias[2], bias[3], bias[4], bias[5] });
}

void franka_controller_remote::set_fts_load_mass(const Eigen::Vector3d& load_mass)
{
	controller_->set_fts_load_mass({ load_mass[0], load_mass[1], load_mass[2] });
}



} /* namespace franka_control */