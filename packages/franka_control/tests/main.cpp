#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>

#include <argparse/argparse.hpp>

#include <franka_control/franka_controller_emulated.hpp>
#include <franka_control/franka_controller_remote.hpp>
#include <franka_proxy_share/franka_proxy_util.hpp>

// todo: this sensor calibration should not be used from here
#include "schunk_ft_to_franka_calibration.hpp"

using namespace franka_proxy;

void franka_controller_remote_test(const std::string& ip);
void franka_controller_emulated_test();
void franka_fts_calibration(const std::string& ip);
void guiding_mode_test(const std::string& ip);

void print_status(const franka_control::franka_controller& controller);


int main(int argc, char* argv[])
{
	argparse::ArgumentParser program("franka_control_test");

	program.add_argument("-e", "--emulate")
	       .help("execute the emulate controller test")
	       .flag();

	program.add_argument("-r", "--remote")
	       .help("specify ip for franka-proxy remote connection")
	       .default_value(std::string{"127.0.0.1"})
	       .metavar("IP");

	// todo: this sensor calibration should not be used from here
	program.add_argument("-c", "--calibrate-fts")
	       .help("specify ip for franka-proxy connection to calibrate fts")
	       .default_value(std::string{"127.0.0.1"})
	       .metavar("IP");

	program.add_argument("-g", "--guiding-mode")
	       .help("guiding mode test")
	       .default_value(std::string("127.0.0.1"))
	       .metavar("IP");


	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cerr << program;
		return -1;
	}


	if (program.is_used("-e"))
	{
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing franka controller emulated test: " << '\n';
		franka_controller_emulated_test();
	}
	if (program.is_used("-r"))
	{
		const auto ip = program.get<std::string>("-r");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing franka controller remote test with IP " << ip << ": " << '\n';
		//std::string ip("132.180.194.112"); // franka1-proxy@resy-lab
		franka_controller_remote_test(ip);
	}
	if (program.is_used("--calibrate-fts"))
	{
		const auto ip = program.get<std::string>("-r");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing franka-schunk-fts calibration with IP " << ip << ": " << '\n';
		//std::string ip("132.180.194.112"); // franka1-proxy@resy-lab
		franka_fts_calibration(ip);
	}
	if (program.is_used("-g"))
	{
		const auto ip = program.get<std::string>("-g");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing guiding mode test: " << '\n';
		guiding_mode_test(ip);
	}
	std::cout << "\nPress Enter to end test exe." << '\n';
	std::cin.get();
	return 0;
}


void franka_controller_remote_test(const std::string& ip)
{
	std::unique_ptr<franka_control::franka_controller> robot;
	try
	{
		robot =
			std::make_unique<franka_control::franka_controller_remote>(ip);
	}
	catch (const std::exception&)
	{
		std::cerr << "Could not connect to franka-proxy with IP " << ip << "." << '\n';
		return;
	}
	franka_control::franka_update_task update_task(*robot);

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

	Eigen::Affine3d pose(franka_proxy_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution = franka_proxy_util::ik_fast_closest(
		pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution);

	std::cout << "result unknown." << '\n'; // todo: design a useful test function
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

	Eigen::Affine3d pose(franka_proxy_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution_joints = franka_proxy_util::ik_fast_closest(
		pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution_joints);

	[[maybe_unused]] bool target_reached = robot->move_until_contact(ik_solution_joints);
	std::cout << "result unknown." << '\n'; // todo: design a useful test function


	std::cout << "Gripper tests: ... ";
	robot->close_gripper();
	robot->open_gripper();
	robot->grasp_gripper();
	[[maybe_unused]] bool gripper_grasped = robot->gripper_grasped();
	robot->open_gripper();
	std::cout << "result unknown." << '\n'; // todo: design a useful test function


	std::cout << "Speed tests: ... ";
	robot->set_speed_factor(0.1);
	[[maybe_unused]] double speed = robot->speed_factor();
	std::cout << "result unknown." << '\n'; // todo: design a useful test function
}

void print_status(const franka_control::franka_controller& controller)
{
	const Eigen::IOFormat format(3, 0, ", ", "\n", "[ ", " ]");
	std::cout << "Current robot joints: "
		<< controller.current_config().transpose().format(format) << '\n';
}

void franka_fts_calibration(const std::string& ip)
{
	franka_control::franka_controller_remote controller(ip);
	schunk_ft_sensor_to_franka_calibration::calibrate_bias(controller);
	schunk_ft_sensor_to_franka_calibration::calibrate_load(controller);
}

void guiding_mode_test(const std::string& ip)
{
	std::unique_ptr<franka_control::franka_controller> robot;
	try
	{
		robot =
			std::make_unique<franka_control::franka_controller_remote>(ip);
	}
	catch (const std::exception&)
	{
		std::cerr << "Could not connect to franka-proxy with IP " << ip << "." << '\n';
		return;
	}
	franka_control::franka_update_task update_task(*robot);

	std::chrono::seconds duration(20);
	//set to default before test
	robot->set_guiding_mode(true, true, true, true, true, true, false);

	std::cout << "Robot is now for 20 secs in guiding mode with guidable DOF (1,1,1,0,0,0) and elbow is false" << '\n';
	robot->set_guiding_mode(true, true, true, false, false, false, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for 20 secs in default guiding mode" << '\n';
	robot->set_guiding_mode(true, true, true, true, true, true, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for 20 secs in guiding mode with guidable DOF (1,1,0,1,1,0) and elbow is false" << '\n';
	robot->set_guiding_mode(true, true, false, true, true, false, false);
	std::this_thread::sleep_for(duration);

	//set back to default after test
	robot->set_guiding_mode(true, true, true, true, true, true, false);
	std::cout << "Finished guiding mode test" << '\n';
}
