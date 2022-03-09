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

	data_file << "Measured Steps in O_F_ext_hat: " << data.o_F_ext_hat_K.size() << "\n\n";
	data_file << "Control Parameters:\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i + 1) << "\t";
		data_file << "P Pos: " << data.k_p_p[i] << "\tI Pos: " << data.k_i_p[i] << "\tD Pos: " << data.k_d_p[i];
		data_file << "\tP Force: " << data.k_p_f[i] << "\tI Force: " << data.k_i_f[i] << "\tD Force: " << data.k_d_f[i] << "\n";
	}

	data_file << "\nSquare Error Integral Median from Position\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i+1) << "\t" << data.square_error_integral_median_position[i] << "\n";
	}
	data_file << "\nSquare Error Integral Median from Force\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i+1) << "\t" << data.square_error_integral_median_force[i] << "\n";
	}

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

std::array<double, 6> calculate_square_error_integral_median(std::vector<Eigen::Matrix<double, 6, 1>> values) {
	std::array<double, 6> square_error_integral_median = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	for (int d = 0; d < 6; d++) { //Dimensions
		for (int n = 0; n < values.size(); n++) { //Step
			square_error_integral_median[d] += ((values[n](d, 0)) * (values[n](d, 0)));
		}
		square_error_integral_median[d] = square_error_integral_median[d] / values.size();
	}
	return square_error_integral_median;
}

csv_data apply_z_force(franka_proxy::franka_hardware_controller& h_controller, std::array<std::array<double, 6>, 6> control_parameters, csv_data& data) {
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
	des_force(2) = 0 * -1.0 * 9.81;

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
	}

	data.square_error_integral_median_position = calculate_square_error_integral_median(data.position_error);
	data.square_error_integral_median_force = calculate_square_error_integral_median(data.force_error);
	csv_parser(data);

	return data;
}

void simulatedAnnnealing(franka_proxy::franka_hardware_controller& h_controller){
	//Position Parameters
	std::array<double, 6> k_p_p = { -200, -200, -200, -30, -30, -20 };
	std::array<double, 6> k_i_p = { -30, -30, -30, -5, -5, -5 };
	std::array<double, 6> k_d_p = { 0, 0, 0, 0, 0, 0 };

	//Ziegler Nichols Method Force Parameters
	std::array<double, 6> k_p_f = { 0.5, 0.5, 0.366, 0.05, 0.05, 0.05 };
	std::array<double, 6> k_i_f = { 5, 5, 8.04, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_d_f = { 0, 0, 0.004163, 0, 0, 0 };

	std::array<std::array<double, 6>, 6> control_parameters;
	control_parameters = { k_p_p, k_i_p, k_d_p, k_p_f, k_i_f, k_d_f };

	//random initialization of parameter set
	double kPmax = 0.6;
	double kImax = 0;
	double kDmax = 0;

	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::normal_distribution<double> d(0, 1);
	double kPr = d(gen); //kp Random [0,1]
	double kIr = d(gen); //ki Random [0,1]
	double kDr = d(gen); //kd Random [0,1]

	Eigen::Vector3d parameterVector = { kPr * kPmax, kIr * kImax, kDr * kDmax};
	std::cout << "Initial Parameters: " << parameterVector[0] << ", " << parameterVector[1] << ", " << parameterVector[2] << std::endl;
	control_parameters[3][0] = kPr * kPmax;

	//call hybrid control with parameterSet and calculate ISE
	csv_data data{};
	apply_z_force(h_controller, control_parameters, data);
	double ISE;
	if (!data.square_error_integral_median_force.empty()) {
		ISE = data.square_error_integral_median_force[2];
		std::cout << "Initial ISE = " << ISE << std::endl;
	}
	else {
		return;
	}

	//calculate (or choose) initial Temperature
	double worst = 50; //TODO
	double best = 0;
	double Pr = 0.85;
	double T = -(worst - best) / log(0.85); //~308

	double eta = 1.0; //initial eta

	int k = 0;
	int k_max = 20;
	double minISE = 0.00001;

	while (ISE > minISE && k < k_max){ // && Delta ISE did not change much in the last n iterations

		//calculate new parameter set (neighbour) based on current paramter set
		std::random_device rd;
		std::mt19937 gen{ rd() };
		std::normal_distribution<double> d(0, 1);
		double lambda1 = d(gen);
		double lambda2 = d(gen);
		double lambda3 = d(gen);
		Eigen::Vector3d lambdaVector = { lambda1, lambda2, lambda3 };
		Eigen::Vector3d newParameterVector = parameterVector + eta * lambdaVector;

		//printing
		std::cout << "lambdaVector: " << lambda1 << ", " << lambda2 << ", " << lambda3 << std::endl;
		std::cout << "newParameterVector = " << newParameterVector(0) << ", " << newParameterVector(1) << ", " << newParameterVector(2) << std::endl;

		//call hybrid_control with newParameterVector and calculate new ISE
		control_parameters[3][0] = newParameterVector(0);
		csv_data data{};
		apply_z_force(h_controller, control_parameters, data);
		double newISE;
		if (!data.square_error_integral_median_force.empty()) {
			newISE = data.square_error_integral_median_force[2];
			std::cout << "Initial ISE = " << ISE << std::endl;
		}
		else {
			newISE = 10000000;
		}
		std::cout << "ISE = " << ISE << ", newISE = " << newISE << std::endl;

		if (newISE < ISE) { //new parameterVector is better then accept always
			ISE = newISE;
			parameterVector = newParameterVector;
			std::cout << "newParameterVector was better" << std::endl;
		}
		else { //if old parameterVector was better, then only accept with boltzmann-related chance
			std::random_device rd;
			std::mt19937 gen{ rd() };
			std::uniform_real_distribution<double> d(0, 1);
			double r = d(gen); //rand[0,1]

			std::cout << "r = " << r << " has to be smaller than: " << (exp(-(newISE - ISE) / T)) << std::endl;

			if (r < exp(-(newISE-ISE)/T)) {
				ISE = newISE;
				parameterVector = newParameterVector;
				std::cout << "newParameterVector was worse but accepted" << std::endl;
			}
			std::cout << "newParameterVector was worse and not accepted" << std::endl;
		}

		//Decrease T and eta
		double l = 0.9;
		T = l * T;
		eta = l * eta;

		k++;
		std::cout << "T: " << T << ", k: " << k << std::endl << std::endl;
	}

}

int main() {
	
	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	simulatedAnnnealing(h_controller);

	return 0;
}




