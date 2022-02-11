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


void put_data_in_csv(std::vector<Eigen::Matrix<double, 6, 1>> data, std::string file_name) {
	int length = data.size();

	std::ofstream data_file(file_name);

	data_file << "n, x, y, z, mx, my, mz \n";

	for (int n = 0; n < length; n++) {
		data_file << (n + 1) << ", ";
		for (int i = 0; i < 6; i++) {
			data_file << data[n][i];
			if (i != 5) {
				data_file << ", ";
			}
		}
		data_file << "\n";
	}
	data_file.close();
}

void create_overview_csv(csv_data data, std::string file_name) {
	std::ofstream data_file(file_name);

	data_file << "duration: " << data.duration << "\n";

}

//This function parses the force_motion_generator::export_data (which is returned from the apply_z_force_pid call in the main function) to a csv file
void csv_parser(csv_data data) {
	int length = data.force_command.size();

	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto time_string = oss.str();

	std::string path = "C:/Users/hecken/Desktop/BA_Hecken/csv_data_files/" + time_string + "/";
	CreateDirectoryA(path.c_str(), NULL);

	create_overview_csv(data, path + "overview.csv");

	put_data_in_csv(data.o_F_ext_hat_K, path + "o_F_ext_hat_K.csv");
	put_data_in_csv(data.force_desired, path + "force_desired.csv");
	put_data_in_csv(data.force_error, path + "force_error.csv");
	put_data_in_csv(data.force_error_integral, path + "force_error_integral.csv");
	put_data_in_csv(data.force_error_diff_filtered, path + "force_error_diff_filtered.csv");

	put_data_in_csv(data.force_command, path + "force_command.csv");
	put_data_in_csv(data.force_command_p, path + "force_command_p.csv");
	put_data_in_csv(data.force_command_i, path + "force_command_i.csv");
	put_data_in_csv(data.force_command_d, path + "force_command_d.csv");

	put_data_in_csv(data.position_error, path + "position_error.csv");
	put_data_in_csv(data.position_error_integral, path + "position_error_integral.csv");
	put_data_in_csv(data.position_error_diff_filtered, path + "position_error_diff_filtered.csv");

	put_data_in_csv(data.position_command, path + "position_command.csv");
	put_data_in_csv(data.position_command_p, path + "position_command_p.csv");
	put_data_in_csv(data.position_command_i, path + "position_command_i.csv");
	put_data_in_csv(data.position_command_d, path + "position_command_d.csv");
}



void print_curent_joint_pos(franka_proxy::franka_hardware_controller& h_controller) {
	for (int i = 0; i < 7; i++) {
		std::cout << h_controller.robot_state().q[i] << ", ";
	}
	std::cout << std::endl;
}

int main() {
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Moving to start position in 2 seconds..." << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(2));

	csv_data data = {};

	std::array<double, 7> pos1 = { -0.148474, 0.66516, 0.0542239, -1.97034, -0.0870424, 2.68921, 0.648864 }; //kontakt auf holzplatte mitte
	std::array<double, 7> pos2 = { -0.149195, 0.657161, 0.0538761, -1.97476, -0.0856461, 2.68536, 0.664352 }; //knapp oberhalb der holzplatte mitte
	std::array<double, 7> pos3 = { -0.538555, 0.570877, -0.182261, -2.12031, 0.223319, 2.70837, 0.762102 }; //knapp oberhalb der holzplatte rechts
	std::array<double, 7> pos4 = { -0.542976, 0.596388, -0.182899, -2.11954, 0.222944, 2.70741, 0.74313 }; //knapp oberhalb der holzplatte rechts

	try {
		//This function calls creates a pid_force_control_motion_generator which is defined in motion_generator_force.cpp
		//In this function a force_motion_generator::export_data is created and filled with the measured values etc. and returns this data
		//h_controller.move_to(pos1);
		//h_controller.move_to_until_contact(pos2);
		std::cout << "Hybrid Force/ Position control in 1 second..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
		h_controller.hybrid_control(data, 2.0, 5);
		
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}

	std::cout << "Writing the data to a csv file..." << std::endl;
	csv_parser(data);
	std::cout << "Writing in csv file finished. Closing in 1 second..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	return 0;
}




