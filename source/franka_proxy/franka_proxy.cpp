/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "franka_proxy.hpp"

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

	try {
		h_controller.apply_z_force_pid(1, 5, 1.0, 2.0, 0.0);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	//franka_proxy::franka_proxy proxy;
	std::cout << "Press enter to close..." << std::endl;
	return std::cin.get();
}


