/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "franka_proxy.hpp"
#include "motion_generator_force.hpp"
#include "csv_data_struct.hpp"
#include "simulated_annealing.hpp"


#include <iostream>
#include <iomanip>
#include <random>
#include<Eigen/StdVector>


namespace franka_proxy
{


	//////////////////////////////////////////////////////////////////////////
	//
	// franka_proxy
	//
	//////////////////////////////////////////////////////////////////////////


	franka_proxy::franka_proxy()
		:
		controller_("192.168.1.1"),

		control_server_(franka_control_port, controller_),
		state_server_(franka_state_port, controller_)
	{}


} /* namespace franka_proxy */





int main() {

	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	franka_proxy::hyb_con_pid_optimizer params_optimizer(h_controller);
	params_optimizer.start();

	std::cout << "Press Esc to abort the the optimization early!" << std::endl;
	while (params_optimizer.is_running()) {
		if (GetAsyncKeyState(0x1B)) {
			params_optimizer.stop();
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return 0;
}




