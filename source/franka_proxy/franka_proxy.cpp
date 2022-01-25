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

void print_status(const franka_proxy::franka_hardware_controller& h_controller)  {
	std::cout << "Joint position of the robot: " << std::endl;
	auto config = h_controller.robot_state();
	for (int i = 0; i < 7; i++) {
		std::cout << config.q[i] << "  ";
	}
	std::cout << std::endl;
}

void print_joint_pos(franka_proxy::robot_config_7dof pos) {
	for (int i = 0; i < 7; i++) {
		std::cout << pos[i] << "  ";
	}
	std::cout << std:: endl;
}

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
	print_joint_pos(pos1);
	h_controller.move_to(pos1);
	print_status(h_controller);

	std::cout << "Moving back to idle position:" << std::endl;
	print_joint_pos(posIdle);
	h_controller.move_to(posIdle);
	print_status(h_controller);
}

void test_measured_z_force(franka_proxy::franka_hardware_controller& robot) {
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	for (int i = 0; i < 50; i++) {

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		for (int j = 0; j < 7; j++) {
			std::cout << robot.robot_state().tau_J[j] << "  ";
		}
		std::cout << std::endl;
		
	}
}

void test_force_control_z(franka_proxy::franka_hardware_controller& h_controller) {

}

//This function parses the force_motion_generator::export_data (which is returned from the apply_z_force_pid call in the main function) to a csv file
//void data_to_csv(franka_proxy::detail::force_motion_generator::export_data data) {
//	std::cout << "PID Parameters: " << data.k_p << ", " << data.k_i << ", " << data.k_d << std::endl;
//	std::cout << "Amount measured forces: " << data.existing_forces.size() << std::endl;
//	std::cout << "Amount desired forces: " << data.desired_forces.size() << std::endl;
//	std::cout << "Amount command forces: " << data.command_forces.size() << std::endl;
//
//	std::ofstream data_file("testExportData.csv");
//	for (int i = 1; i <= 6; i ++) {
//		data_file << std::to_string(data.masses[i]) << ",";
//	}
//	data_file << std::to_string(data.duration) << "," << std::to_string(data.k_p) << "," << std::to_string(data.k_i) << "," << std::to_string(data.k_d) << "\n";
//
//	for (int n = 0; n < data.measured_forces.size(); n++) {
//		for (int i = 0; i < 6; i++) {
//			data_file << data.measured_forces[n][i] << "," << data.desired_forces[n][i] << "," << data.command_forces[n][i] << "," << data.force_errors[n][i] << ",";
//			data_file << data.force_errors_integrals[n][i] << "," << data.force_errors_differentials[n][i] << "," << data.force_errors_differentials_sum[n][i] << "," << data.force_errors_differentials_filtered[n][i] << ",";
//		}
//		data_file << "\n";
//	}
//	
//	data_file.close();
//}

void debug_export_data(franka_proxy::detail::force_motion_generator::export_data data) {

	if (!(data.existing_forces.size() > 0)) {
		std::cout << "No measured values to print." << std::endl;
		return;
	}
	for (int i = 0; i < data.existing_forces.size(); i++) {
		if (i % 100 == 0) {
			std::cout << "i: " << i << std::endl;
			for (int j = 0; j < 6; j++) {
				std::cout << "Dim: " << j+1 << std::endl;
				std::cout << "force_desired = " << data.desired_forces[i][j] << ", ";
				std::cout << "force_existing = " << data.existing_forces[i][j] << std::endl;
				std::cout << "force_command = " << data.command_forces[i][j] << ", ";
				std::cout << "position_command = " << data.position_forces[i][j] << ", ";
				std::cout << "hybrid_command = " << data.hybrid_forces[i][j] << std::endl;
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
		data = h_controller.apply_z_force_pid(1.0, 5);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}

	//std::this_thread::sleep_for(std::chrono::seconds(1));



	//franka_proxy::franka_proxy proxy;
	//std::cout << "Press enter to close..." << std::endl;
	/*std::cout << "Writing the data to a csv file..." << std::endl;
	data_to_csv(data);
	std::cout << "Writing in csv file finished. Closing in 1 second..." << std::endl;*/
	std::this_thread::sleep_for(std::chrono::seconds(1));
	debug_export_data(data);
	return 0;
}




