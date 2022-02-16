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

std::array<double, 6> calculate_square_error_integral(std::vector<Eigen::Matrix<double, 6, 1>> values) {
	std::array<double, 6> square_error_integral = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	for (int i = 0; i < 6; i++) {
		for (int n = 0; n < values.size(); n++) {
			square_error_integral[i] += pow((values[n](i, 0)), 2);
		}
	}
	return square_error_integral;
}

bool apply_z_force(franka_proxy::franka_hardware_controller& h_controller, std::array<std::array<double, 6>, 6> control_parameters) {
	csv_data data = {};
	std::array<double, 7> pos_30 = { 0.0141143, 0.744292, -0.0176676, -1.6264, 0.0207479, 2.41293, 0.724183 }; //30mm above wood plate with blue part
	std::array<double, 7> pos_10 = { 0.0166511, 0.777224, -0.0173561, -1.61821, 0.020813, 2.45273, 0.724666 }; //10mm above wood plate with blue part
	std::array<double, 7> pos_2 = { 0.0173603, 0.782011, -0.0172685, -1.61904, 0.0207746, 2.45505, 0.724928 }; //2mm above wood plate with blue part
	std::array<double, 7> pos_0 = { 0.0159666, 0.784819, -0.0174431, -1.6166, 0.0208109, 2.45508, 0.724577 }; //0mm above wood plate with blue part

	try {
		h_controller.move_to(pos_10);
		h_controller.move_to(pos_0);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception when moving to start position: " << e.what() << std::endl;
		return false;
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
		h_controller.hybrid_control(data, desired_positions, desired_forces, desired_orientations, control_parameters);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
		return false;
	}

	csv_parser(data);

	std::cout << "P: " << control_parameters[3][2] << std::endl;
	std::cout << "Square Error Integral force z: " << calculate_square_error_integral(data.force_error)[2] << std::endl;
	std::cout << "Square Error Integral position z: " << calculate_square_error_integral(data.position_error)[2] << std::endl << std::endl;

	return true;
	/*std::cout << "Enter 'y' if you want to save the data in a csv file: " << std::endl;
	std::string answer_string;
	std::getline(std::cin, answer_string);
	if (answer_string == "y") {
		std::cout << "Writing the data to a csv file..." << std::endl;
		csv_parser(data);
		std::cout << "Writing in csv file finished." << std::endl;
	}*/
}

int main() {
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");
	csv_data data = {};

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	std::array<double, 6> k_i_p = { -30, -30, -30, -5, -5, -5 };
	std::array<double, 6> k_d_p = { 0, 0, 0, 0, 0, 0 };
	std::array<double, 6> k_p_p = { -200, -200, -200, -30, -30, -20 };

	std::array<double, 6> k_p_f = { 0.5, 0.5, 0.55, 0.05, 0.05, 0.05 };
	std::array<double, 6> k_i_f = { 5, 5, 0, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_d_f = { 0, 0, 0, 0, 0, 0 };

	std::array<std::array<double, 6>, 6> control_parameters;
	control_parameters = { k_p_p, k_i_p, k_d_p, k_p_f, k_i_f, k_d_f };

	bool continue_control = true;
	while (continue_control) {
		continue_control = apply_z_force(h_controller, control_parameters);
		control_parameters[3][2] += 0.005;
	}

	return 0;
}




