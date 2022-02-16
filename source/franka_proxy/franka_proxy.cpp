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



void print_cur_joint_pos(franka_proxy::franka_hardware_controller& h_controller) {
	for (int i = 0; i < 7; i++) {
		std::cout << h_controller.robot_state().q[i] << ", ";
	}
	std::cout << std::endl;
}

void print_cur_cartesian_pos(franka_proxy::franka_hardware_controller& h_controller) {
	std::array<double, 16> o_T_EE = h_controller.robot_state().O_T_EE;
	std::cout << "x= " << o_T_EE[12] << ", y= " << o_T_EE[13] << ", z= " << o_T_EE[14] << std::endl;
}

void apply_z_force(franka_proxy::franka_hardware_controller& h_controller) {
	csv_data data = {};
	std::array<double, 7> pos_air = { 0.00808741, 0.224202, -0.0017594, -1.84288, -0.0287991, 2.03331, 0.807086 };

	std::cout << "Applying z-force..." << std::endl;
	try {
		h_controller.move_to(pos_air);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception when moving to start position: " << e.what() << std::endl;
	}
	//Get start position
	std::array<double, 16> o_T_EE = h_controller.robot_state().O_T_EE;
	Eigen::Vector3d start_pos(o_T_EE[12], o_T_EE[13], o_T_EE[14]);

	//Get start orientation
	Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(h_controller.robot_state().O_T_EE.data()));
	Eigen::Quaterniond start_orientation;
	start_orientation = initial_transform.linear();

	//Set forces
	Eigen::Matrix<double, 6, 1> des_force;
	des_force.setZero();
	des_force(2) = -1.0 * 9.81;

	std::vector<Eigen::Vector3d> desired_positions;
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
	std::vector<Eigen::Quaterniond> desired_orientations;
	for (int i = 0; i < 5000; i++) {
		desired_orientations.push_back(start_orientation);
		desired_positions.push_back(start_pos);
		desired_forces.push_back(des_force);
	}

	try {
		std::cout << "Hybrid Force/ Position control in 1 second..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));

		h_controller.hybrid_control(data, 2.0, 10, desired_positions, desired_forces, desired_orientations);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
	}
}

int main() {
	//Positionen
	std::array<double, 7> pos_air = { 0.00808741, 0.224202, -0.0017594, -1.84288, -0.0287991, 2.03331, 0.807086 }; //in der luft bei (0.6, 0, 0.3)
	std::array<double, 7> pos_contact = { -0.00684334, 0.832551, 0.0170616, -1.55377, -0.0087478, 2.47243, 0.742217 }; //mit blauem Teil knapp oberhalb Tischplatte
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");
	csv_data data = {};

	std::cout << "Moving to start position in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	

	std::cout << "Enter 'y' if you want to save the data in a csv file: " << std::endl;
	std::string answer_string;
	std::getline(std::cin, answer_string);
	if (answer_string == "y") {
		std::cout << "Writing the data to a csv file..." << std::endl;
		csv_parser(data);
		std::cout << "Writing in csv file finished." << std::endl;
	}
	return 0;
}




