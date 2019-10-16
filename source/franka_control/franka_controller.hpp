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

#include <Eigen/Geometry>

#include <viral_core/thread.hpp>


namespace franka_control
{


typedef Eigen::Matrix<double, 7, 1> robot_config_7dof;


/**
 *************************************************************************
 *
 * @class franka_controller
 *
 * TODO!
 *
 ************************************************************************/
class franka_controller
{
public:

	franka_controller();

	virtual ~franka_controller() noexcept;

	
	virtual void apply_z_force(double mass, double duration) = 0;

	virtual void move_to(const robot_config_7dof& target) = 0;
	void move_to(const Eigen::Affine3d& target);
	virtual bool move_to_until_contact(const robot_config_7dof& target) = 0;
	bool move_to_until_contact(const Eigen::Affine3d& target);

	virtual void open_gripper() = 0;
	virtual void close_gripper(double speed = 0.025, double force = 0.05) = 0;
	virtual bool gripper_grasped() const = 0;

	virtual double speed_factor() const = 0;
	virtual void set_speed_factor(double speed_factor) = 0;

	virtual void automatic_error_recovery() = 0;

	virtual robot_config_7dof current_config() const = 0;
	virtual int current_gripper_pos() const = 0;
	virtual int max_gripper_pos() const = 0;

	virtual void update() = 0;

	Eigen::Affine3d current_nsa_T_world() const;
};




/**
 *************************************************************************
 *
 * @class franka_update_task
 *
 * Updates a franka_controller instance in a separate thread.
 *
 ************************************************************************/
class franka_update_task :
	private viral_core::threaded_task
{

	public:

		franka_update_task(franka_controller& controller);
		~franka_update_task() noexcept;


	private:

		void task_main() override;

		static const float update_timestep_secs_;
		franka_controller& controller_;
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP) */
