/**
 *************************************************************************
 *
 * @file franka_controller_remote.cpp
 *
 * Franka controller to emulate a robot, implementation.
 *
 ************************************************************************/

#include <optional>
#include <execution>

#include "franka_controller_remote.hpp"

#include <franka_proxy_client/franka_remote_interface.hpp>
#include "franka_proxy_share/franka_proxy_util.hpp"


namespace franka_control
{
//////////////////////////////////////////////////////////////////////////
//
// franka_emulated_controller
//
//////////////////////////////////////////////////////////////////////////


franka_controller_remote::franka_controller_remote(
	const std::string& ip)
	: controller_(new franka_proxy::franka_remote_interface(ip)),
	  speed_factor_(0.1)
{
	controller_->set_speed_factor(speed_factor_);
	controller_->update();
}


franka_controller_remote::~franka_controller_remote() noexcept = default;


void franka_controller_remote::move(const robot_config_7dof& target)
{
	franka_proxy::robot_config_7dof array;
	Eigen::VectorXd::Map(array.data(), array.size()) = target;
	controller_->move_to(array);
}


bool franka_controller_remote::move_until_contact(
	const robot_config_7dof& target)
{
	franka_proxy::robot_config_7dof array;
	Eigen::VectorXd::Map(array.data(), array.size()) = target;
	return controller_->move_to_until_contact(array);
}


void franka_controller_remote::open_gripper()
{
	controller_->open_gripper();
}


void franka_controller_remote::close_gripper()
{
	controller_->close_gripper();
}


void franka_controller_remote::grasp_gripper(double speed, double force)
{
	controller_->grasp_gripper(speed, force);
}


bool franka_controller_remote::gripper_grasped() const
{
	return controller_->gripper_grasped();
}


bool franka_controller_remote::vacuum_gripper_vacuum(std::uint8_t vacuum_strength, std::chrono::milliseconds timeout)
{
	return controller_->vacuum_gripper_vacuum(vacuum_strength, timeout);
}


bool franka_controller_remote::vacuum_gripper_drop(std::chrono::milliseconds timeout)
{
	return controller_->vacuum_gripper_drop(timeout);
}


bool franka_controller_remote::vacuum_gripper_stop()
{
	return controller_->vacuum_gripper_stop();
}


double franka_controller_remote::speed_factor() const
{
	std::lock_guard l(state_lock_);
	return speed_factor_;
}


void franka_controller_remote::set_speed_factor(double speed_factor)
{
	{
		std::lock_guard l(state_lock_);
		speed_factor_ = speed_factor;
	}
	controller_->set_speed_factor(speed_factor_);
}


void franka_controller_remote::automatic_error_recovery()
{
	controller_->automatic_error_recovery();
}


robot_config_7dof franka_controller_remote::current_config() const
{
	robot_config_7dof result;
	const franka_proxy::robot_config_7dof config =
		controller_->current_config();

	for (int i = 0; i < config.size(); ++i)
		result[i] = config[i];

	return result;
}


wrench franka_controller_remote::current_force_torque() const
{
	wrench result;
	const std::array<double, 6> wrench =
		controller_->current_end_effector_wrench();

	for (int i = 0; i < wrench.size(); ++i)
		result[i] = wrench[i];

	return result;
}


int franka_controller_remote::current_gripper_pos() const
{
	return static_cast<int>(controller_->current_gripper_pos() * gripper_unit_per_m_);
}


int franka_controller_remote::max_gripper_pos() const
{
	return static_cast<int>(controller_->max_gripper_pos() * gripper_unit_per_m_);
}


void franka_controller_remote::update()
{
	controller_->update();
}


void franka_controller_remote::start_recording(std::optional<std::string> log_file_path)
{
	controller_->start_recording(log_file_path);
}


std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> franka_controller_remote::stop_recording()
{
	std::vector<robot_config_7dof> joints;
	std::vector<wrench> force_torque;

	auto [recorded_joints, recorded_forces] =
		controller_->stop_recording();

	joints.reserve(recorded_joints.size());
	for (auto datum : recorded_joints)
		joints.emplace_back(datum.data());

	force_torque.reserve(recorded_forces.size());
	for (auto datum : recorded_forces)
		force_torque.emplace_back(datum.data());

	return {joints, force_torque};
}


void franka_controller_remote::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_sequence,
	const std::optional<std::array<double, 16>>& offset_cartesian,
	const std::optional<std::array<double, 6>>& offset_force)
{
	std::vector<std::array<double, 7>> joints(q_sequence.size());
	std::vector<std::array<double, 6>> forces(f_sequence.size());
	std::vector<std::array<double, 6>> selections(selection_sequence.size());

	std::transform(std::execution::par, q_sequence.begin(), q_sequence.end(), joints.begin(),
		[](const auto& datum) { return franka_proxy::franka_proxy_util::convert_to_std_array<double, 7>(datum); });

	std::transform(std::execution::par, f_sequence.begin(), f_sequence.end(), forces.begin(),
		[](const auto& datum) { return franka_proxy::franka_proxy_util::convert_to_std_array<double, 6>(datum); });

	std::transform(std::execution::par, selection_sequence.begin(), selection_sequence.end(), selections.begin(),
		[](const auto& datum) { return franka_proxy::franka_proxy_util::convert_to_std_array<double, 6>(datum); });

	if (offset_cartesian && offset_force)
	{
		const Eigen::Matrix4d offset_matrix = Eigen::Map<const Eigen::Matrix4d>(offset_cartesian->data());
		const Eigen::Affine3d offset_transform(offset_matrix);

		auto apply_offset = [&](const robot_config_7dof& q) -> robot_config_7dof {
			const Eigen::Affine3d original_pose = franka_proxy::franka_proxy_util::fk(q).front();
			const Eigen::Affine3d new_pose = offset_transform * original_pose;
			return franka_proxy::franka_proxy_util::ik_fast_closest(new_pose, q);
			};

		const robot_config_7dof result_front = apply_offset(q_sequence.front());
		const robot_config_7dof result_back = apply_offset(q_sequence.back());

		controller_->move_to(result_front);
		controller_->move_sequence(joints, forces, selections, *offset_cartesian, *offset_force);
		controller_->move_to(result_back);
	}
	else
	{
		controller_->move_to(joints.front());
		controller_->move_sequence(joints, forces, selections);
		controller_->move_to(joints.back());
	}
}


void franka_controller_remote::set_fts_bias(
	const wrench& bias)
{
	controller_->set_fts_bias({bias[0], bias[1], bias[2], bias[3], bias[4], bias[5]});
}


void franka_controller_remote::set_fts_load_mass(
	const Eigen::Vector3d& load_mass)
{
	controller_->set_fts_load_mass({load_mass[0], load_mass[1], load_mass[2]});
}


void franka_controller_remote::set_guiding_mode(
	bool x, bool y, bool z,
	bool rx, bool ry, bool rz, bool elbow)
{
	controller_->set_guiding_params(x, y, z, rx, ry, rz, elbow);
}
} /* namespace franka_control */
