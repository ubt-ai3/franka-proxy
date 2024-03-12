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
void franka_control_ik_test(franka_control::franka_controller& controller);


int main()
{
	std::string ip("127.0.0.1");
	//std::string ip("132.180.194.112");

	franka_fts_calibration_test(ip);
	//franka_controller_emulated_test();
	//franka_controller_remote_test(ip);

	std::cout << "Press Enter to end program." << std::endl;
	return std::cin.get();
}


void franka_controller_remote_test(const std::string& ip)
{
	std::unique_ptr<franka_control::franka_controller> robot =
		std::make_unique<franka_control::franka_controller_remote>(ip);
	franka_control::franka_update_task controller_status_update_task(*robot);

	// status test
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &robot]()
	{
		while (!stop)
		{
			print_status(*robot);

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(std::chrono::duration<double>(1));
		}
	});

	// TODO add test code, so no user interaction is needed.
	std::cout << "Press Enter to end test." << std::endl;
	std::cin.get();
	stop = true;
	t.join();
}

void franka_controller_emulated_test()
{
	using namespace std::chrono_literals;

	std::unique_ptr<franka_control::franka_controller> robot =
		std::make_unique<franka_control::franka_controller_emulated>();

	// status test
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

	franka_control_ik_test(*robot);

	std::this_thread::sleep_for(std::chrono::duration<double>(3));
	stop = true;
	t.join();
}

void print_status(const franka_control::franka_controller& controller)
{
	const auto joints = controller.current_config();
	std::cout << "POS JOINTS: ";
	std::cout << joints.transpose() << std::endl;
	std::cout << "CART: \n";
	std::cout << franka_control::franka_util::fk(controller.current_config()).rbegin()->matrix() << std::endl;

	std::cout << std::endl;
}

void franka_fts_calibration_test(const std::string& ip)
{
	franka_control::franka_controller_remote controller(ip);
	schunk_ft_sensor_to_franka_calibration::calibrate_bias(controller);
	schunk_ft_sensor_to_franka_calibration::calibrate_load(controller);
}

void franka_control_ik_test(franka_control::franka_controller& controller)
{
	std::cout << "--- Starting FK/IK Test. ---" << std::endl;

	const franka_control::robot_config_7dof joints((franka_control::robot_config_7dof()
		<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished());
	controller.move(joints);

	Eigen::Affine3d pose(franka_control::franka_util::fk(joints).back());
	pose.linear()
		<< 0.707107, 0.707107, 0,
		0.707107, -0.707107, 0,
		0, 0, -1;

	const auto ik_solution =
		franka_control::franka_util::ik_fast_closest(pose, franka_control::robot_config_7dof(joints));
	controller.move(ik_solution);

	std::cout << "--- Finished FK/IK Test. ---" << std::endl;
}
