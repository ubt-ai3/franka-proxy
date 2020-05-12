/**
 *************************************************************************
 *
 * @file franka_controller_emulated.hpp
 *
 * Franka controller to emulate a robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP)
#define INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP


#include <viral_core/timer.hpp>

#include "franka_controller.hpp"


namespace franka_control
{


/**
 *************************************************************************
 *
 * @class franka_controller_emulated
 *
 * TODO!
 *
 ************************************************************************/
class franka_controller_emulated :
	public franka_controller
{
public:

	franka_controller_emulated();
	~franka_controller_emulated() noexcept override;


	void apply_z_force(double mass, double duration) override;

	void move_to(const robot_config_7dof& target) override;
	bool move_to_until_contact(const robot_config_7dof& target) override;

	void open_gripper() override;
	void close_gripper() override;
	void grasp_gripper(double speed = 0.025, double force = 0.05) override;
	bool gripper_grasped() const override;

	double speed_factor() const override;
	void set_speed_factor(double speed_factor) override;

	void automatic_error_recovery() override;

	robot_config_7dof current_config() const override;
	int current_gripper_pos() const override;
	int max_gripper_pos() const override;

	void update() override;


private:

	static const float max_speed_length_per_sec_;
	static const float update_timestep_secs_;

	mutable viral_core::mutex controller_mutex_;
	double speed_normalized_;
	bool gripper_open_;

	robot_config_7dof state_joint_values_;

	int max_gripper_pos_;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP) */
