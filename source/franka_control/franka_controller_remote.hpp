/**
 *************************************************************************
 *
 * @file franka_controller_remote.hpp
 *
 * Controlling of a remote controlled robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_REMOTE_HPP)
#define INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_REMOTE_HPP


#include <mutex>

#include "franka_controller.hpp"


namespace franka_proxy
{
	class franka_remote_controller;
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
class franka_controller_remote :
	public franka_controller
{
public:

	franka_controller_remote
		(const std::string& ip);
	~franka_controller_remote() noexcept override;


	void move(const robot_config_7dof& target) override;
	void move_with_force(const robot_config_7dof& target, const force_torque_config_cartesian& target_force_torques) override;
	bool move_until_contact(const robot_config_7dof& target) override;

	void open_gripper() override;
	void close_gripper() override;
	void grasp_gripper(double speed = 0.025, double force = 0.05) override;
	bool gripper_grasped() const override;

	double speed_factor() const override;
	void set_speed_factor(double speed_factor) override;
	void automatic_error_recovery() override;

	robot_config_7dof current_config() const override;
	force_torque_config_cartesian current_force_torque() const override;
	int current_position_in_sequence() const override;
	int current_gripper_pos() const override;
	int max_gripper_pos() const override;

	void update() override;


	void start_recording() override;
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop_recording() override;
	void move_sequence(
		std::vector<std::array<double, 7>> q_sequence,
		std::vector<std::array<double, 6>> f_sequence,
		std::vector<std::array<double, 6>> selection_vector_sequence) override;
	
private:

	std::unique_ptr<franka_proxy::franka_remote_controller> controller_;

	mutable std::mutex state_lock_;
	double speed_factor_;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_REMOTE_HPP) */
