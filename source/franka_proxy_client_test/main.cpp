


#include "franka_proxy_client/franka_remote_controller.hpp"
#include "viral_core/ms_network.hpp"
#include <iostream>


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

int main()
{
	viral_core::ms_network_context network("network");
	franka_proxy::franka_remote_controller controller("127.0.0.1", network);

	controller.open_gripper();
	controller.close_gripper();
	
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

	controller.move_to(pos1);

	while (dist_squared(controller.current_config(), pos1) > 0.001)
	{
		controller.update();
		print_status(controller);
	}

	controller.move_to(pos2);

	while (dist_squared(controller.current_config(), pos2) > 0.001)
	{
		controller.update();
		print_status(controller);
	}

	std::cin.ignore();
}