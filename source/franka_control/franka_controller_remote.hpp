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


#include "franka_controller.hpp"

#include <mutex>


namespace viral_core
{
class network_context;
}


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
 * TODO!
 *
 ************************************************************************/
class franka_controller_remote :
	public franka_controller
{
public:

	franka_controller_remote
		(const std::string& ip, viral_core::network_context& network);
	~franka_controller_remote() noexcept override;


	void apply_z_force(double mass, double duration) override;

	void move_to(const robot_config_7dof& target) override;
	bool move_to_until_contact(const robot_config_7dof& target) override;

	void open_gripper() override;
	void close_gripper(double speed = 0.025, double force = 0.05) override;
	bool gripper_grasped() const override;

	double speed_factor() const override;
	void set_speed_factor(double speed_factor) override;

	void automatic_error_recovery() override;

	robot_config_7dof current_config() const override;
	int current_gripper_pos() const override;
	int max_gripper_pos() const override;

	void update() override;


private:

	std::unique_ptr<franka_proxy::franka_remote_controller> controller_;

	mutable std::mutex state_lock_;
	double speed_factor_;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_REMOTE_HPP) */
