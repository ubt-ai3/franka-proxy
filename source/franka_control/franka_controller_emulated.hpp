/**
 *************************************************************************
 *
 * @file franka_controller_emulated.hpp
 *
 * Franka controller to emulate a virtual robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP)
#define INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP


#include "franka_controller.hpp"


namespace franka_control
{


/**
 *************************************************************************
 *
 * @class franka_controller_emulated
 *
 * A franka_controller that emulates robot movement.
 *
 * Usage notes:
 * - The robot_emulator does not have any acceleration limits
 *		and uses an implementation-specific maximum speed.
 *
 ************************************************************************/
class franka_controller_emulated :
	public franka_controller
{
public:

	franka_controller_emulated();
	~franka_controller_emulated() noexcept override;


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


	void start_recording() override;
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>>
		stop_recording() override;
	void move_sequence
		(std::vector<std::array<double, 7>> q_sequence,
		 std::vector<std::array<double, 6>> f_sequence,
		 std::vector<std::array<double, 6>> selection_vector_sequence) override;
	
private:

	void move_gripper(int target, double speed_mps);


	static constexpr double max_speed_length_per_sec_ = 3.5; // ~200 deg
	static constexpr float move_update_rate_ = 0.01f;

	mutable std::mutex controller_mutex_;
	double speed_factor_;
	bool gripper_open_;

	robot_config_7dof state_joint_values_;
	int state_gripper_pos_;

	static constexpr int max_gripper_pos_ = 50;
	static constexpr int gripper_unit_per_m_ = 1000;
	static constexpr double gripper_default_speed_mps_ = 0.025;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_EMULATED_HPP) */
