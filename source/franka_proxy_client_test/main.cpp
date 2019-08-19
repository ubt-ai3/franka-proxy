


#include "franka_proxy_client/franka_remote_controller.hpp"
#include "viral_core/ms_network.hpp"
#include <iostream>


void print(const franka_proxy::robot_config_7dof& config)
{
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}

int main()
{
	viral_core::ms_network_context network("network");
	franka_proxy::franka_remote_controller controller("127.0.0.1", network);

	controller.open_gripper();
	controller.close_gripper();

	print(controller.current_config());

	controller.move_to({2.4673210167983628,
			-1.053636035616098,
			-0.935180716967433,
			-1.670424119447617,
			0.1367540113528485,
			1.4206203791091001,
					   0.3347107737734215});

	print(controller.current_config());

	controller.move_to({-0.002421978837257,
			1.2362939888338829,
			2.4654171861844083,
			-1.264853222554674,
			-0.001813626296555,
			1.9141426016730037,
			-1.063268839608126});

	print(controller.current_config());
}