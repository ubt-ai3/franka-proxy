


#include "franka_proxy_client/franka_remote_controller.hpp"
#include "viral_core/ms_network.hpp"
#include <iostream>
#include "franka_proxy_client/exception.hpp"
#include "viral_core/thread_util.hpp"
#include "viral_core/log.hpp"


void print(const franka_proxy::robot_config_7dof& config)
{
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}

void print_status(const franka_proxy::franka_remote_controller& controller)
{
	std::cout << "POS: ";
	auto config = controller.current_config();
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}

double dist_squared(const franka_proxy::robot_config_7dof& c1, const franka_proxy::robot_config_7dof& c2)
{
	double sum = 0;
	for (int i = 0; i < 7; ++i)
		sum += (c1[i] - c2[i]) * (c1[i] - c2[i]);
	return sum;
}

template <class Function>
	void execute_retry(Function&& f, franka_proxy::franka_remote_controller& controller)
{
	bool finished = false;
	while (!finished)
	{
		try
		{
			f();
			finished = true;
		}
		catch (const franka_proxy::control_exception&)
		{
			// for some reason, automatic error recovery
			// is only possible after waiting some time...
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			controller.automatic_error_recovery();
		}
		catch (const franka_proxy::command_exception&)
		{
			LOG_INFO("Encountered command exception. Probably because of wrong working mode. Waiting before retry.")
			viral_core::thread_util::sleep_seconds(1);
		}
	}
}

int main()
{
	viral_core::ms_network_context network("network");
	franka_proxy::franka_remote_controller controller("127.0.0.1", network);

	execute_retry([&]{controller.open_gripper();}, controller);
	execute_retry([&]{controller.close_gripper();}, controller);
	
	controller.update();
	print_status(controller);

	franka_proxy::robot_config_7dof pos1 {{
			2.4673210167983628,
			-1.053636035616098,
			-0.935180716967433,
			-1.670424119447617,
			0.1367540113528485,
			1.4206203791091001,
			0.3347107737734215}};
	franka_proxy::robot_config_7dof pos2 {{
			-0.002421978837257,
			1.2362939888338829,
			2.4654171861844083,
			-1.264853222554674,
			-0.001813626296555,
			1.9141426016730037,
			-1.063268839608126}};
	
	execute_retry([&]{controller.move_to(pos1);}, controller);

	while (dist_squared(controller.current_config(), pos1) > 0.001)
	{
		controller.update();
		print_status(controller);
	}
	
	execute_retry([&]{controller.move_to(pos2);}, controller);

	while (dist_squared(controller.current_config(), pos2) > 0.001)
	{
		controller.update();
		print_status(controller);
	}

	std::cin.ignore();
}