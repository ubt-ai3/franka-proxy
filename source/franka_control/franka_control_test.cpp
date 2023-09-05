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
	//auto biases = schunk_ft_sensor_to_franka_calibration::calibrate_bias(controller);
	auto load = schunk_ft_sensor_to_franka_calibration::calibrate_load(controller);
}
