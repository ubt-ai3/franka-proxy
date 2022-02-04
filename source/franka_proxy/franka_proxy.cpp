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

#include <iostream>


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

void gripper_test(franka_proxy::franka_hardware_controller& h_controller) {
	h_controller.open_gripper(0.1);
	h_controller.close_gripper(0.1);
	h_controller.open_gripper(0.1);
}

void move_test(franka_proxy::franka_hardware_controller& h_controller) {
	franka_proxy::robot_config_7dof posIdle
	{ {0.166, 0.078, -0.152, -2.615, 0.020, 2.714, -2.390} };
	franka_proxy::robot_config_7dof pos1
	{ {2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471} };

	h_controller.set_speed_factor(0.2);

	std::cout << "Moving to position one:" << std::endl;
	h_controller.move_to(pos1);

	std::cout << "Moving back to idle position:" << std::endl;
	h_controller.move_to(posIdle);
}

//This function parses the force_motion_generator::export_data (which is returned from the apply_z_force_pid call in the main function) to a csv file
void data_to_csv(franka_proxy::detail::force_motion_generator::export_data data) {

	std::ofstream data_file("testExportData.csv");
	//todo: print all necessary parts from export_data in csv file
	for (int n = 0; n < data.measured_forces.size(); n++) {
		for (int i = 0; i < 6; i++) {
			data_file << data.measured_forces[n][i] << ",";
		}
		data_file << "\n";
	}	
	data_file.close();
}

void debug_export_data(franka_proxy::detail::force_motion_generator::export_data data) {

	if (!(data.force_commands.size() > 0)) {
		std::cout << "No measured values to print." << std::endl;
		return;
	}
	for (int i = 0; i < data.force_commands.size(); i++) {
		if (i % 500 == 0) {
			std::cout << "i: " << i << std::endl;
			for (int j = 0; j < 6; j++) {
				std::cout << "Dim: " << j+1 << std::endl;
				std::cout << "measured_force = " << data.measured_forces[i][j] << ", ";
				std::cout << "position_error = " << data.position_errors[i][j] << ", ";
				std::cout << "force_error = " << data.force_errors[i][j] << ", ";
				std::cout << "position_command = " << data.position_commands[i][j] << ", ";
				std::cout << "force_command = " << data.force_commands[i][j] << std::endl;
			}
			std::cout << std::endl;
		}
		
	}	
}

int main() {
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Applying z-force in 2 seconds..." << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(2));

	franka_proxy::detail::force_motion_generator::export_data data;

	try {
		//This function calls creates a pid_force_control_motion_generator which is defined in motion_generator_force.cpp
		//In this function a force_motion_generator::export_data is created and filled with the measured values etc. and returns this data
		data = h_controller.apply_z_force_pid(2.0, 10);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}

	/*std::cout << "Writing the data to a csv file..." << std::endl;
	data_to_csv(data);
	std::cout << "Writing in csv file finished. Closing in 1 second..." << std::endl;*/
	std::this_thread::sleep_for(std::chrono::seconds(1));
	debug_export_data(data);
	return 0;
}




