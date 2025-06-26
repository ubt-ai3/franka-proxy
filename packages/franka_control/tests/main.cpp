#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>

#include <argparse/argparse.hpp>

#include <franka_proxy_share/franka_proxy_util.hpp>
#include <franka_control/forward.hpp> // included for IDE visibility of the forward header
#include <franka_control/franka_controller_emulated.hpp>
#include <franka_control/franka_controller_remote.hpp>

// todo: this sensor calibration should not be used from here
#include "schunk_ft_to_franka_calibration.hpp"


void franka_controller_remote_test(const std::string& ip);
void franka_controller_emulated_test();
void franka_fts_calibration(const std::string& ip);
void guiding_mode_test(const std::string& ip);
void test_generic_function(const std::string& ip);

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

	// todo: this sensor calibration should not be here
	program.add_argument("-c", "--calibrate-fts")
	       .help("specify ip for franka-proxy connection to calibrate schunk net-box fts")
	       .default_value(std::string{"127.0.0.1"})
	       .metavar("IP");

	program.add_argument("-g", "--guiding-mode")
	       .help("guiding mode test")
	       .default_value(std::string("127.0.0.1"))
	       .metavar("IP");

	program.add_argument("-t", "--test-generic")
	       .help("generic test function")
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
		franka_controller_remote_test(ip);
	}
	if (program.is_used("--calibrate-fts"))
	{
		const auto ip = program.get<std::string>("-r");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing franka-schunk-fts calibration with IP " << ip << ": " << '\n';
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
	if (program.is_used("-t"))
	{
		const auto ip = program.get<std::string>("-t");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing generic test: " << std::endl;
		test_generic_function(ip);
	}
	std::cout << "\nPress Enter to end test exe." << std::endl;
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


	std::cout << "Status tests: 5 seconds.\n";
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

	Eigen::Affine3d pose(franka_proxy::franka_proxy_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution = franka_proxy::franka_proxy_util::ik_fast_closest(
		pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution);

	std::cout << "result unknown." << '\n';
}


void franka_controller_emulated_test()
{
	std::unique_ptr<franka_control::franka_controller> robot =
		std::make_unique<franka_control::franka_controller_emulated>();
	franka_control::franka_update_task update_task(*robot);

	const auto joints_start = robot->current_config();
	const auto joints_test((franka_control::robot_config_7dof()
		<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished());

	std::cout << "Status tests: Moving to test config and back.\n";
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


	std::cout << "Move test using IK: ... ";
	robot->move(joints_test);

	Eigen::Affine3d pose(franka_proxy::franka_proxy_util::fk(joints_test).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution_joints =
		franka_proxy::franka_proxy_util::ik_fast_closest(
			pose, franka_control::robot_config_7dof(joints_test));
	robot->move(ik_solution_joints);
	// todo: design a useful test function
	std::cout << "result needs to be checked manually." << '\n';


	std::cout << "Gripper tests: ... ";
	robot->close_gripper();
	robot->open_gripper();
	robot->grasp_gripper();
	[[maybe_unused]] bool gripper_grasped = robot->gripper_grasped();
	robot->open_gripper();
	// todo: design a useful test function
	std::cout << "result needs to be checked manually." << '\n';


	std::cout << "Speed tests: ... ";
	robot->set_speed_factor(0.1);
	[[maybe_unused]] double speed = robot->speed_factor();
	// todo: design a useful test function
	std::cout << "result needs to be checked manually." << '\n';
}


namespace
{
template <typename D> void print_fixed_format(
	const std::string& label,
	const Eigen::MatrixBase<D>& vec)
{
	std::cout << label << "[ ";
	for (int i = 0; i < vec.size(); ++i)
	{
		std::cout << std::fixed << std::setprecision(4)
			<< std::setw(7) << std::setfill(' ')
			<< vec(i);
		if (i != vec.size() - 1)
			std::cout << ", ";
	}
	std::cout << " ]\n";
}
}


void print_status(const franka_control::franka_controller& controller)
{
	print_fixed_format("Current robot joints: ", controller.current_config().transpose());
	print_fixed_format("Current wrench: ", controller.current_force_torque().transpose());
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

	std::chrono::seconds duration(10);
	//set to default before test
	robot->set_guiding_mode(true, true, true, true, true, true, false);

	std::cout << "Robot is now for " << duration
		<< " in guiding mode with guidable DOF (1,1,1,0,0,0) and elbow is false" << '\n';
	robot->set_guiding_mode(true, true, true, false, false, false, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for " << duration
		<< " in default guiding mode" << '\n';
	robot->set_guiding_mode(true, true, true, true, true, true, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for " << duration
		<< "  in guiding mode with guidable DOF (1,1,0,1,1,0) and elbow is false" << '\n';
	robot->set_guiding_mode(true, true, false, true, true, false, false);
	std::this_thread::sleep_for(duration);

	//set back to default after test
	robot->set_guiding_mode(true, true, true, true, true, true, false);
	std::cout << "Finished guiding mode test." << '\n';
}


void test_generic_function(const std::string& ip)
{
	std::unique_ptr<franka_control::franka_controller> robot;
	try
	{
		robot =
			std::make_unique<franka_control::franka_controller_remote>(ip);
	}
	catch (const std::exception&)
	{
		std::cerr << "Could not connect to franka-proxy with IP "
			<< ip << ".\n";
		return;
	}
	franka_control::franka_update_task update_task(*robot);

	std::chrono::minutes duration(1);
	std::cout << "Generic tests called: The status test runs for "
		<< duration << ".\n";
	std::atomic_bool stop(false);
	std::vector<franka_control::wrench> ft_values;
	std::thread t
	([&stop, &robot, &ft_values]()
	{
		while (!stop)
		{
			ft_values.emplace_back(robot->current_force_torque());
			std::this_thread::sleep_for(
				std::chrono::seconds(1));
		}
	});

	std::this_thread::sleep_for(duration);
	stop = true;
	t.join();

	for (const auto& value : ft_values)
		print_fixed_format("Wrench: ", value.transpose());
}
