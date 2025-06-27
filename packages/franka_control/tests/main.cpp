#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <argparse/argparse.hpp>

#include <franka_proxy_share/franka_proxy_util.hpp>
#include <franka_control/forward.hpp> // included for IDE visibility of the forward header
#include <franka_control/franka_controller_emulated.hpp>
#include <franka_control/franka_controller_remote.hpp>

#include "schunk_ft_to_franka_calibration.hpp"


static void franka_controller_remote_test(const std::string& ip);
static void franka_controller_emulated_test();
static void franka_fts_calibration(const std::string& ip);
static void test_generic_function(const std::string& ip); // for quick custom tests


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


void print_status(const franka_control::franka_controller& controller)
{
	print_fixed_format("Current robot joints: ", controller.current_config().transpose());
	print_fixed_format("Current wrench: ", controller.current_force_torque().transpose());
}
}


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
	program.add_argument("-c", "--calibrate-fts")
	       .help("specify ip for franka-proxy connection to calibrate schunk net-box fts")
	       .default_value(std::string{"127.0.0.1"})
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
	if (program.is_used("-c"))
	{
		const auto ip = program.get<std::string>("-c");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing franka-schunk-fts calibration with IP " << ip << ": " << '\n';
		franka_fts_calibration(ip);
	}
	if (program.is_used("-t"))
	{
		const auto ip = program.get<std::string>("-t");
		std::cout <<
			"--------------------------------------------------------------------------------\n"
			"Executing generic test: \n";
		test_generic_function(ip);
	}


	std::cout << "\nPress Enter to end test exe.\n";
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
	std::cout << "result needs to be checked manually." << '\n';


	std::cout << "Gripper tests: ... ";
	robot->close_gripper();
	robot->open_gripper();
	robot->grasp_gripper();
	bool gripper_grasped = robot->gripper_grasped();
	robot->open_gripper();
	std::cout << "result needs to be checked manually." << '\n';


	std::cout << "Speed tests: ... ";
	robot->set_speed_factor(0.1);
	double speed = robot->speed_factor();
	std::cout << "result needs to be checked manually." << '\n';
}


void franka_fts_calibration(const std::string& ip)
{
	franka_control::franka_controller_remote controller(ip);
	schunk_ft_sensor_to_franka_calibration::calibrate_bias(controller);
	schunk_ft_sensor_to_franka_calibration::calibrate_load(controller);
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

	std::chrono::hours duration(12);
	std::cout << "Generic tests called: The status test runs for "
		<< duration << ".\n";
	std::atomic_bool stop(false);
	std::vector<franka_control::wrench> ft_values;
	std::thread t
	([&stop, &robot, &ft_values]()
	{
		while (!stop)
		{
			auto zoned_time = 
				std::chrono::current_zone()->to_local(std::chrono::system_clock::now());
			std::cout << std::format("{:%a %b %d %H:%M:%S %Y}\n", zoned_time);
			print_fixed_format("Current wrench: ", robot->current_force_torque());

			ft_values.emplace_back(robot->current_force_torque());

			std::this_thread::sleep_for(
				std::chrono::minutes(1));
		}
	});

	std::this_thread::sleep_for(duration);
	stop = true;
	t.join();

	// Write wrenches to *.csv.
	std::string out_filename = "ft_values.csv";
	std::ofstream out_stream(out_filename);

	if (!out_stream.is_open())
	{
		std::cerr << "Error: Could not open file "
			<< out_filename << " for writing.\n";
		return;
	}

	out_stream << "fx,fy,fz,tx,ty,tz\n";
	out_stream << std::fixed << std::setprecision(6);

	for (const auto& wrench : ft_values)
	{
		for (int i = 0; i < 6; ++i)
		{
			out_stream << wrench[i];
			if (i < 5)
				out_stream << ",";
		}
		out_stream << "\n";
	}
	out_stream.close();
}
