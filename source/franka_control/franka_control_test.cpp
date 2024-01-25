#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
#include <utility>

#include <franka_control/franka_controller_remote.hpp>
#include <franka_control/franka_util.hpp>

#include "sensor_calibration/schunk_ft_to_franka_calibration.hpp"


void franka_controller_test(const std::string& ip);
void print_status(const franka_control::franka_controller_remote& controller);
void franka_fts_calibration_test(const std::string& ip);

int main()
{
	std::string ip("127.0.0.1");
	//std::string ip("132.180.194.112");

	franka_fts_calibration_test(ip);
	//franka_controller_test(ip);

	return std::cin.get();

	// TODO this testcode was last tested in franka client not in control and is not adapted!
	//std::cout << "Starting FK/IK Test." << std::endl;

	//Eigen::Affine3d pose
	//(franka_control::franka_util::fk
	//(
	//	(franka_control::robot_config_7dof()
	//		<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished()).back());

	//pose.linear() << 0.707107, 0.707107, 0,
	//	0.707107, -0.707107, -0,
	//	0, 0, -1;

	//auto ik_solution = franka_control::franka_util::ik_fast_closest
	//(pose,
	//	franka_control::robot_config_7dof(robot.current_config().data()));

	//franka_proxy::robot_config_7dof q{};
	//Eigen::VectorXd::Map(&q[0], 7) = ik_solution;
	//robot.move_to(q);

	//std::cout << ("Finished FK/IK Test.");
}


void franka_controller_test(const std::string& ip)
{
	franka_control::franka_controller_remote controller(ip);
	// status test
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &controller]()
	{
		int i = 0;
		while (!stop)
		{
			controller.update();

			if (i++ % 30 == 0)
				print_status(controller);

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(std::chrono::duration<double>(0.016));
		}
	});

	t.join();
}

void print_status(const franka_control::franka_controller_remote& controller)
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
