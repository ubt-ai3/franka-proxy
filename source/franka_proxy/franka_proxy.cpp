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

	int dim[12] = {
		1,1,0,1,1,1, //position (x, y, z, mx, my, mz)
		0,0,1,0,0,0 //force (x, y, z, mx, my, mz)
	};

	for (int i = 0; i < 6; i++) {
		if (dim[i] + dim[i + 6] > 1) {
			std::cout << "Invalid Dimension Array!" << std::endl;
			return 0;
		}
	}
	simulatedAnnnealing(h_controller, dim);

	return 0;
}




