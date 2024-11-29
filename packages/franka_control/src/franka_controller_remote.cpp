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

void franka_controller_remote::move_with_force(const robot_config_7dof& target, const wrench& target_force_torques)
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

wrench franka_controller_remote::current_force_torque() const
{
	return wrench();
}

int franka_controller_remote::current_gripper_pos() const
	{ return static_cast<int>( controller_->current_gripper_pos() * gripper_unit_per_m_); }
int franka_controller_remote::max_gripper_pos() const
	{ return static_cast<int>(controller_->max_gripper_pos() * gripper_unit_per_m_); }
void franka_controller_remote::update()
	{ controller_->update(); }


void franka_controller_remote::start_recording(std::optional<std::string> log_file_path)
{
	controller_->start_recording(log_file_path);
}


std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> franka_controller_remote::stop_recording()
{
	std::vector<robot_config_7dof> joints;
	std::vector<wrench> forces;

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
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_vector_sequence,
	const std::array<double,16> offset_cartesian,
	const std::array<double,6> offset_force)
{
	std::vector<std::array<double, 7>> joints;
	std::vector<std::array<double, 6>> forces;
	std::vector<std::array<double, 6>> selection;

	joints.resize(q_sequence.size());
	forces.resize(f_sequence.size());
	selection.resize(selection_vector_sequence.size());

	std::for_each(std::execution::par, q_sequence.begin(), q_sequence.end(),
		[&joints, &q_sequence](const auto& datum) {
			size_t idx = &datum - &q_sequence[0];
			/*joints[idx] = std::array<double, 7>{
				datum(0), datum(1), datum(2), datum(3),
					datum(4), datum(5), datum(6)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			joints[idx] = *reinterpret_cast<std::array<double, 7>*>(&datum);
		});

	std::for_each(std::execution::par, f_sequence.begin(), f_sequence.end(),
		[&forces, &f_sequence](const auto& datum) {
			size_t idx = &datum - &f_sequence[0];
			/*forces[idx] = std::array<double, 6>{
				datum(0), datum(1), datum(2),
					datum(3), datum(4), datum(5)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			forces[idx] = *reinterpret_cast<std::array<double, 6>*>(&datum);
		});

	std::for_each(std::execution::par, selection_vector_sequence.begin(), selection_vector_sequence.end(),
		[&selection, &selection_vector_sequence](const auto& datum) {
			size_t idx = &datum - &selection_vector_sequence[0];
			/*selection[idx] = std::array<double, 6>{
				datum(0), datum(1), datum(2),
					datum(3), datum(4), datum(5)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			selection[idx] = *reinterpret_cast<std::array<double, 6>*>(&datum);
		});

	//apply offset for front()
	Eigen::Affine3d offset_transform;

	offset_transform.matrix() = Eigen::Map<const Eigen::Matrix4d>(offset_cartesian.data());

	std::vector<Eigen::Affine3d> original_pose = franka_proxy::franka_proxy_util::fk(q_sequence.front());

	Eigen::Affine3d new_pose = offset_transform * original_pose.front();

	robot_config_7dof result = franka_proxy::franka_proxy_util::ik_fast_closest(new_pose, q_sequence.front());

	//apply offset for back()
	Eigen::Affine3d offset_transform;

	offset_transform.matrix() = Eigen::Map<const Eigen::Matrix4d>(offset_cartesian.data());

	std::vector<Eigen::Affine3d> original_pose = franka_proxy::franka_proxy_util::fk(q_sequence.back());

	Eigen::Affine3d new_pose = offset_transform * original_pose.front();

	robot_config_7dof result = franka_proxy::franka_proxy_util::ik_fast_closest(new_pose, q_sequence.back());

	controller_->move_to(joints.front());
	controller_->move_sequence(joints, forces, selection,offset_cartesian,offset_force);
	controller_->move_to(joints.back());
}

void franka_controller_remote::move_sequence(
	const std::vector<robot_config_7dof>& q_sequence,
	const std::vector<wrench>& f_sequence,
	const std::vector<selection_diagonal>& selection_vector_sequence)
{
	std::vector<std::array<double, 7>> joints;
	std::vector<std::array<double, 6>> forces;
	std::vector<std::array<double, 6>> selection;

	joints.resize(q_sequence.size());
	forces.resize(f_sequence.size());
	selection.resize(selection_vector_sequence.size());

	std::for_each(std::execution::par, q_sequence.begin(), q_sequence.end(),
		[&joints, &q_sequence](const auto& datum) {
			size_t idx = &datum - &q_sequence[0];
			/*joints[idx] = std::array<double, 7>{
				datum(0), datum(1), datum(2), datum(3),
					datum(4), datum(5), datum(6)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			joints[idx] = *reinterpret_cast<std::array<double, 7>*>(&datum);
		});

	std::for_each(std::execution::par, f_sequence.begin(), f_sequence.end(),
		[&forces, &f_sequence](const auto& datum) {
			size_t idx = &datum - &f_sequence[0];
			/*forces[idx] = std::array<double, 6>{
				datum(0), datum(1), datum(2),
					datum(3), datum(4), datum(5)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			forces[idx] = *reinterpret_cast<std::array<double, 6>*>(&datum);
		});

	std::for_each(std::execution::par, selection_vector_sequence.begin(), selection_vector_sequence.end(),
		[&selection, &selection_vector_sequence](const auto& datum) {
			size_t idx = &datum - &selection_vector_sequence[0];
			/*selection[idx] = std::array<double, 6>{
				datum(0), datum(1), datum(2),
					datum(3), datum(4), datum(5)
			};*/
			// Reinterpret the Eigen::Matrix as std::array directly instead of copying
			selection[idx] = *reinterpret_cast<std::array<double, 6>*>(&datum);
		});


	
	
	controller_->move_to(joints.front());
	controller_->move_sequence(joints, forces, selection);
	controller_->move_to(joints.back());
}


void franka_controller_remote::set_fts_bias(const wrench& bias)
{
	controller_->set_fts_bias({ bias[0], bias[1], bias[2], bias[3], bias[4], bias[5] });
}

void franka_controller_remote::set_fts_load_mass(const Eigen::Vector3d& load_mass)
{
	controller_->set_fts_load_mass({ load_mass[0], load_mass[1], load_mass[2] });
}

void franka_controller_remote::set_guiding_mode(bool x, bool y, bool z, bool rx, bool ry, bool rz, bool elbow) const
{
	controller_->set_guiding_params(x,y,z,rx,ry,rz,elbow);
}



} /* namespace franka_control */