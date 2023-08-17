#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
#include <utility>

#include "franka_control/franka_controller_remote.hpp"
#include "franka_control/franka_util.hpp"


void franka_controller_test(const std::string& ip);
void print_status(const franka_control::franka_controller_remote& controller);


int main()
{
	//franka_controller_test("132.180.194.112");
	franka_controller_test("127.0.0.1");
	return 0;
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
			std::this_thread::sleep_for(0.016s);
		}
	});
}

void print_status(const franka_control::franka_controller_remote& controller)
{
	const auto joints = controller.current_config();
	std::cout << "POS JOINTS: ";
	std::cout << joints << std::endl;
	std::cout << "CART: ";
	std::cout << franka_control::franka_util::fk(controller.current_config()).rbegin()->matrix() << std::endl;

	std::cout << std::endl;
}
