/**
 *************************************************************************
 *
 * @file franka_controller.hpp
 *
 * Base class for franka controllers.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP)
#define INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP


#include <array>
#include <atomic>
#include <thread>
#include <vector>

#include <Eigen/Geometry>


namespace franka_control
{


typedef Eigen::Matrix<double, 7, 1> robot_config_7dof;


/**
 *************************************************************************
 *
 * @class franka_controller
 *
 * Interface for controlling and monitoring a franka emika panda robot.
 *
 * Usage notes:
 * - To control a robot, clients send commands (e.g. set_speed_factor())
 *      via the robot_controller.
 * - To monitor a robot, clients must fetch franka_controller states
 *		from the actual robot by update().
 * - Without any update() call, the robot_controller may not change
 *		its exposed states.
 * - Clients should regularly call update() to avoid network timeouts.
 *		A franka_update_task conveniently automates update()s.
 *
 ************************************************************************/
class franka_controller
{
public:

	franka_controller();

	virtual ~franka_controller() noexcept;

	
	virtual void move_to(const robot_config_7dof& target) = 0;
	void move_to(const Eigen::Affine3d& target_world_T_nsa);
	virtual bool move_to_until_contact(const robot_config_7dof& target) = 0;
	bool move_to_until_contact(const Eigen::Affine3d& target_world_T_nsa);

	virtual void open_gripper() = 0;
	virtual void close_gripper() = 0;
	virtual void grasp_gripper(double speed = 0.025, double force = 0.05) = 0;
	virtual bool gripper_grasped() const = 0;

	virtual double speed_factor() const = 0;
	virtual void set_speed_factor(double speed_factor) = 0;

	virtual void automatic_error_recovery() = 0;

	virtual robot_config_7dof current_config() const = 0;
	virtual int current_gripper_pos() const = 0;
	virtual int max_gripper_pos() const = 0;

	/**
	 * Fetch current state from the back-end robot.
	 * Call regularly to avoid overflow in network buffers,
	 * e.g. through a robot_controller_task.
	 */
	virtual void update() = 0;

	Eigen::Affine3d current_nsa_T_world() const;
	Eigen::Affine3d current_flange_T_world() const;
	Eigen::Affine3d current_tcp_T_world() const;
	
	virtual void start_recording() = 0;
	virtual std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> stop_recording() = 0;
	virtual void move_sequence(
		std::vector<std::array<double, 7>> q_sequence,
		std::vector<std::array<double, 6>> f_sequence, 
		std::vector<std::array<double, 6>> selection_vector_sequence) = 0;
};




/**
 *************************************************************************
 *
 * @class franka_update_task
 *
 * Updates a franka_controller instance in a separate thread.
 *
 ************************************************************************/
class franka_update_task
{

public:

	franka_update_task(franka_controller& controller);
	~franka_update_task() noexcept;


private:

	void task_main();

	static const double update_timestep_secs_;
	franka_controller& controller_;

	std::thread internal_thread_;
	std::atomic_bool terminate_internal_thread_;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP) */
