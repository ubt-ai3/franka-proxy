/**
 *************************************************************************
 *
 * @file franka_controller_remote.hpp
 *
 * Controlling of a remote controlled robot.
 *
 ************************************************************************/

#pragma once


#include <mutex>

#include <Eigen/Geometry>

#include "franka_controller.hpp"


namespace franka_proxy
{
class franka_remote_interface;
}


namespace franka_control
{
/**
 *************************************************************************
 *
 * @class franka_controller_remote
 *
 * Control a franka emika panda robot via the franka_proxy server.
 *
 ************************************************************************/
class franka_controller_remote : public franka_controller
{
public:
	franka_controller_remote
	(const std::string& ip);
	~franka_controller_remote() noexcept override;


	void move(const robot_config_7dof& target) override;
	void move_with_force(const robot_config_7dof& target,
	                     const wrench& target_force_torques) override;
	bool move_until_contact(const robot_config_7dof& target) override;

	void open_gripper() override;
	void close_gripper() override;
	void grasp_gripper(double speed = 0.025, double force = 0.05) override;
	bool gripper_grasped() const override;

	double speed_factor() const override;
	void set_speed_factor(double speed_factor) override;
	void automatic_error_recovery() override;

	robot_config_7dof current_config() const override;
	wrench current_force_torque() const override;
	int current_gripper_pos() const override;
	int max_gripper_pos() const override;

	void update() override;


	void start_recording(bool log, std::string& file) override;
	std::pair<std::vector<robot_config_7dof>, std::vector<wrench>> stop_recording() override;
	void move_sequence(
		const std::vector<robot_config_7dof>& q_sequence,
		const std::vector<wrench>& f_sequence,
		const std::vector<selection_diagonal>& selection_vector_sequence) override;

	void set_fts_bias(const wrench& bias);
	void set_fts_load_mass(const Eigen::Vector3d& load_mass);

	void set_guiding_mode(bool x, bool y, bool z, bool rx, bool ry, bool rz, bool elbow) const override;
	
private:
	std::unique_ptr<franka_proxy::franka_remote_interface> controller_;

	mutable std::mutex state_lock_;
	double speed_factor_;
};
} /* namespace franka_control */
