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

	data_file << "\nISE from Position\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i + 1) << "\t" << data.ise_position[i] << "\n";
	}
	data_file << "\nISE from Force\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i + 1) << "\t" << data.ise_force[i] << "\n";
	}
	data_file << "\nITAE from Position\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i + 1) << "\t" << data.itae_position[i] << "\n";
	}
	data_file << "\nITAE from Force\n";
	for (int i = 0; i < 6; i++) {
		data_file << "Dimension: " << (i + 1) << "\t" << data.itae_force[i] << "\n";
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

//integral squared error: Force in [N^2 s] or Position in ?[m^2 s]?
Eigen::Matrix<double, 6, 1> calculate_ISE(std::vector<Eigen::Matrix<double, 6, 1>>& values) {
	Eigen::Matrix<double, 6, 1> ise;
	ise.setZero();
	for (int n = 0; n < values.size(); n++) {
		ise += ((values[n].array() * values[n].array()) * 0.001).matrix(); //Integral(e(t)*e(t) dt)
	}
	return ise;
}

//integral time absolute error: Force in [N s^2] or Position in ?[m s^2]?
Eigen::Matrix<double, 6, 1> calculate_ITAE(std::vector<Eigen::Matrix<double, 6, 1>>& values) {
	Eigen::Matrix<double, 6, 1> itae;
	itae.setZero();
	for (int n = 0; n < values.size(); n++) {
		itae += ((n / 1000.0) * values[n].array().abs() * 0.001).matrix();  //Integral(t*|e(t)| dt)
	}
	return itae;
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
	des_force(2) = -10.0;

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
		h_controller.automatic_error_recovery();
		throw;
	}

	data.ise_position = calculate_ISE(data.position_error);
	data.ise_force = calculate_ISE(data.force_error);
	data.itae_position = calculate_ITAE(data.position_error);
	data.itae_force = calculate_ITAE(data.force_error);

	csv_parser(data);

	return data;
}


void simulatedAnnnealing(franka_proxy::franka_hardware_controller& h_controller) {
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto time_string = oss.str();

	std::string path = "C:/Users/hecken/Desktop/BA_Hecken/SA/" + time_string + "/";
	CreateDirectoryA(path.c_str(), NULL);
	std::string filename = "C:/Users/hecken/Desktop/BA_Hecken/SA/" + time_string + "/sa_overview.csv";

	std::ofstream sa_data_file;

	sa_data_file.open(filename, std::ofstream::out | std::ofstream::app);
	sa_data_file << "k,T,eta,best_F,new_F,best_Kp,new_Kp,best_Ki,new_Ki,best_Kd,new_Kd\n";
	sa_data_file.close();

	//Position Parameters
	std::array<double, 6> k_p_p = { -200, -200, -200, -30, -30, -20 };
	std::array<double, 6> k_i_p = { -30, -30, -30, -5, -5, -5 };
	std::array<double, 6> k_d_p = { 0, 0, 0, 0, 0, 0 };

	//Ziegler Nichols Method Force Parameters
	std::array<double, 6> k_p_f = { 0.5, 0.5, 0 , 0.05, 0.05, 0.05 };
	std::array<double, 6> k_i_f = { 5, 5, 0, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_d_f = { 0, 0, 0, 0, 0, 0 };

	std::array<std::array<double, 6>, 6> control_parameters;
	control_parameters = { k_p_p, k_i_p, k_d_p, k_p_f, k_i_f, k_d_f };

	//random initialization of parameter set
	Eigen::Vector3d max_parameters(0.6, 20.0, 0.01);

	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::uniform_real_distribution<double> d(0, 1);
	Eigen::Vector3d rand_vector(d(gen), d(gen), d(gen));

	Eigen::Vector3d best_parameter_vector = (rand_vector.array() * max_parameters.array()).matrix();
	control_parameters[3][2] = best_parameter_vector(0);
	control_parameters[4][2] = best_parameter_vector(1);
	control_parameters[4][2] = best_parameter_vector(2);

	//call hybrid control with parameterSet and calculate F
	csv_data data{};
	double best_F;
	double old_best_F; // Difference between to adjacent best F values
	try {
		apply_z_force(h_controller, control_parameters, data);
		best_F = data.itae_force(2); //cost Function
	}
	catch (const franka::Exception& e) {
		best_F = 100000.0;
	}
	old_best_F = best_F;

	double T = 20; //initial T
	double eta = 1.0; //initial eta

	double target_F = 0.1; //This value of F will be accepted
	double epsilon = 0.05; //two adjacent best_F values have to be under this value for c_max consecutive steps. Then the SA Alg stops.
	int c = 0; //consecutive falling below min_delta_F
	int c_max = 10;
	int k = 1;
	double mu = 0.0;
	double sigma = 0.25;

	while (best_F > target_F && c < c_max) { // && Delta ISE did not change much in the last n iterations

		//calculate new parameter set (neighbour) based on current paramter set
		std::normal_distribution<double> nd(mu, sigma);
		Eigen::Vector3d lambda_vector = { nd(gen), nd(gen), nd(gen) };
		Eigen::Vector3d new_parameter_vector = best_parameter_vector + eta * (lambda_vector.array() * max_parameters.array()).matrix();

		//set newParameterVector in necessary boundaries
		new_parameter_vector(0) = std::max(0.0, std::min(new_parameter_vector(0), max_parameters(0)));
		new_parameter_vector(1) = std::max(0.0, std::min(new_parameter_vector(1), max_parameters(1)));
		new_parameter_vector(2) = std::max(0.0, std::min(new_parameter_vector(2), max_parameters(2)));

		//call hybrid_control with newParameterVector and calculate new F
		control_parameters[3][2] = new_parameter_vector(0);
		control_parameters[4][2] = new_parameter_vector(1);
		control_parameters[5][2] = new_parameter_vector(2);

		csv_data data{};
		double new_F;
		try {
			apply_z_force(h_controller, control_parameters, data);
			new_F = data.itae_force(2);
		}
		catch (const franka::Exception& e) {
			new_F = 100000.0;
		}

		//write values in sa_overview.csv
		sa_data_file.open(filename, std::ofstream::out | std::ofstream::app);
		sa_data_file << k << "," << T << "," << eta << "," << best_F << "," << new_F << "," << best_parameter_vector(0) << "," << new_parameter_vector(0)
			<< "," << best_parameter_vector(1) << "," << new_parameter_vector(1) << "," << best_parameter_vector(2) << "," << new_parameter_vector(2) << "\n";
		sa_data_file.close();

		//K_D is printed with a factor 1000!
		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "k=" << k << "\tT = " << T <<  "\tbest F=" << best_F << "\tnew_F=" << new_F << "\tc=" << c
			<< "\tbestParams = (" << best_parameter_vector(0) << "," << best_parameter_vector(1) << "," << 1000 * best_parameter_vector(2) << ")"
			<< "\tnewParams = (" << new_parameter_vector(0) << "," << new_parameter_vector(1) << "," << 1000.0 * new_parameter_vector(2) << ")"
			<< "\n";

		if (new_F < best_F) { //new parameterVector is better then accept always
			best_F = new_F;
			best_parameter_vector = new_parameter_vector;
		}
		else { //if old parameterVector was better, then only accept with boltzmann-related chance
			double r = d(gen); //rand[0,1]

			if (r < exp(-(new_F - best_F) / T)) {
				best_F = new_F;
				best_parameter_vector = new_parameter_vector;
			}
		}

		//check delta_best_F as acceptance criteria
		if (std::abs(old_best_F - best_F) < epsilon) {
			c++;
		}
		else {
			c = 0;
		}

		old_best_F = best_F;

		//Decrease T and eta
		double l = 0.95;
		T = l * T;
		eta = l * eta;

		k++;
	}

}

int main() {

	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	simulatedAnnnealing(h_controller);

	return 0;
}




