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
void data_to_csv(franka_proxy::detail::force_motion_generator::export_data data) {
	std::cout << "PID Parameters: " << data.k_p << ", " << data.k_i << ", " << data.k_d << std::endl;
	std::cout << "Amount measured forces: " << data.measured_forces.size() << std::endl;
	std::cout << "Amount desired forces: " << data.desired_forces.size() << std::endl;
	std::cout << "Amount command forces: " << data.command_forces.size() << std::endl;

	std::ofstream data_file("testExportData.csv");
	data_file << std::to_string(data.k_p) << "," << std::to_string(data.k_i) << "," << std::to_string(data.k_d) << "\n";
}


int main() {
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");



	//Move test
	//std::cout << "Starting move test..." << std::endl;
	//move_test(h_controller);
	
	//Gripper test
	//std::cout << "Starting Gripper Test..." << std::endl;
	//gripper_test(h_controller);

	/*z-Force measurement Test*/
	//std::cout << "Starting z-Force measurement test..." << std::endl;
	//test_measured_z_force(h_controller);

	std::cout << "Applying z-force in 3 seconds..." << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(3));

	franka_proxy::detail::force_motion_generator::export_data data;

	try {
		//This function calls creates a pid_force_control_motion_generator which is defined in motion_generator_force.cpp
		//In this function a force_motion_generator::export_data is created and filled with the measured values etc. and returns this data
		data = h_controller.apply_z_force_pid(1, 5, 1.0, 2.0, 0.0);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	for (int i = 0; i < 20; i++){
		if (true) {
			/*std::cout << "Measured x-Force " << i << ": " << data.measured_forces[i][0] << std::endl;
			std::cout << "Measured y-Force " << i << ": " << data.measured_forces[i][1] << std::endl;
			std::cout << "Measured z-Force " << i << ": " << data.measured_forces[i][2] << std::endl;
			std::cout << "Desired x-Force" << i << ": " << data.desired_forces[i][0] << std::endl;
			std::cout << "Desired y-Force" << i << ": " << data.desired_forces[i][1] << std::endl;
			std::cout << "Desired z-Force" << i << ": " << data.desired_forces[i][2] << std::endl;
			std::cout << "Error x-Force" << i << ": " << data.force_errors[i][0] << std::endl;
			std::cout << "Error y-Force" << i << ": " << data.force_errors[i][1] << std::endl;
			std::cout << "Error z-Force" << i << ": " << data.force_errors[i][2] << std::endl;
			std::cout << "Error integral x-Force" << i << ": " << data.force_errors_integrals[i][0] << std::endl;
			std::cout << "Error integral y-Force" << i << ": " << data.force_errors_integrals[i][1] << std::endl;
			std::cout << "Error integral z-Force" << i << ": " << data.force_errors_integrals[i][2] << std::endl;
			std::cout << "Error differential x-Force" << i << ": " << data.force_errors_differentials[i][0] << std::endl;
			std::cout << "Error differential y-Force" << i << ": " << data.force_errors_differentials[i][1] << std::endl;*/
			std::cout << "Error differential z-Force" << i << ": " << data.force_errors_differentials[i][2] << std::endl;
			/*std::cout << "Error differentia sum x-Force" << i << ": " << data.force_errors_differentials_sum[i][0] << std::endl;
			std::cout << "Error differential sum y-Force" << i << ": " << data.force_errors_differentials_sum[i][1] << std::endl;*/
			std::cout << "Error differential sum z-Force" << i << ": " << data.force_errors_differentials_sum[i][2] << std::endl;
			/*std::cout << "Error differential filtered x-Force" << i << ": " << data.force_errors_differentials_filtered[i][0] << std::endl;
			std::cout << "Error differential filtered y-Force" << i << ": " << data.force_errors_differentials_filtered[i][1] << std::endl;*/
			std::cout << "Error differential filtered z-Force" << i << ": " << data.force_errors_differentials_filtered[i][2] << std::endl;
			/*std::cout << "Control x-Force" << i << ": " << data.command_forces[i][0] << std::endl;
			std::cout << "Control y-Force" << i << ": " << data.command_forces[i][1] << std::endl;
			std::cout << "Control z-Force" << i << ": " << data.command_forces[i][2] << std::endl;*/
			std::cout << std::endl << std::endl;
		}
	}

	//franka_proxy::franka_proxy proxy;
	std::cout << "Press enter to close..." << std::endl;
	return std::cin.get();
}


