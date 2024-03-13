#include <atomic>
#include <iostream>
#include <thread>

#include <franka_control/franka_controller_emulated.hpp>
#include <franka_control/franka_controller_remote.hpp>
#include <franka_control/franka_util.hpp>

#include "sensor_calibration/schunk_ft_to_franka_calibration.hpp"


void franka_controller_remote_test(const std::string& ip);
void franka_controller_emulated_test();
void franka_fts_calibration_test(const std::string& ip);

void print_status(const franka_control::franka_controller& controller);


int main()
{
	std::string ip("127.0.0.1");
	//std::string ip("132.180.194.112"); // franka1-proxy@resy-lab

	franka_controller_emulated_test();
	//franka_controller_remote_test(ip);
	//franka_fts_calibration_test(ip);

	std::cout << "Press Enter to end test exe." << std::endl;
	std::cin.get();
	return 0;
}


void franka_controller_remote_test(const std::string& ip)
{
	std::unique_ptr<franka_control::franka_controller> robot =
		std::make_unique<franka_control::franka_controller_remote>(ip);
	franka_control::franka_update_task update_task(*robot);

	// todo: real robot should not be dependent on start position
	const auto joints_start = robot->current_config();
	// todo: this config (and IK pose) needs to be tested on real robot
	const auto joints_test((franka_control::robot_config_7dof()
		<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished());


	std::cout << "Status tests: 5 seconds.\n"; // todo: design a useful test function
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &robot]()
	{
		while (!stop)
		{
			print_status(*robot);
			std::this_thread::sleep_for(std::chrono::duration<double>(1));
		}
	});

	std::this_thread::sleep_for(std::chrono::duration<double>(5));
	stop = true;
	t.join();


	std::cout << "Move tests: ... ";

	robot->move(joints_test);

	Eigen::Affine3d pose(franka_control::franka_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution = franka_control::franka_util::ik_fast_closest(
		pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution);

	std::cout << "result unknown." << std::endl; // todo: design a useful test function
}

void franka_controller_emulated_test()
{
	std::unique_ptr<franka_control::franka_controller> robot =
		std::make_unique<franka_control::franka_controller_emulated>();
	franka_control::franka_update_task update_task(*robot);

	const auto joints_start = robot->current_config();
	const auto joints_test((franka_control::robot_config_7dof()
		<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished());

	std::cout << "Status tests: Moving to test config and back.\n"; // todo: design a useful test function
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &robot]()
		{
			while (!stop)
			{
				print_status(*robot);
				std::this_thread::sleep_for(std::chrono::duration<double>(1));
			}
		});
	robot->move(joints_test);
	robot->move(joints_start);
	stop = true;
	t.join();


	std::cout << "Move tests: ... ";
	robot->move(joints_test);

	Eigen::Affine3d pose(franka_control::franka_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution_joints = franka_control::franka_util::ik_fast_closest(
		pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution_joints);

	bool target_reached = robot->move_until_contact(ik_solution_joints);
	std::cout << "result unknown." << std::endl; // todo: design a useful test function


	std::cout << "Gripper tests: ... ";
	robot->close_gripper();
	robot->open_gripper();
	robot->grasp_gripper();
	bool gripper_grasped = robot->gripper_grasped();
	robot->open_gripper();
	std::cout << "result unknown." << std::endl;  // todo: design a useful test function


	std::cout << "Speed tests: ... ";
	robot->set_speed_factor(0.1);
	double speed = robot->speed_factor();
	std::cout << "result unknown." << std::endl; // todo: design a useful test function
}

void print_status(const franka_control::franka_controller& controller)
{
	const Eigen::IOFormat format(3, 0, ", ", "\n", "[ ", " ]");
	std::cout << "Current robot joints: "
		<< controller.current_config().transpose().format(format) << std::endl;
}

void franka_fts_calibration_test(const std::string& ip)
{
	franka_control::franka_controller_remote controller(ip);
	schunk_ft_sensor_to_franka_calibration::calibrate_bias(controller);
	schunk_ft_sensor_to_franka_calibration::calibrate_load(controller);
}