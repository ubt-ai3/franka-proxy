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

std::vector<std::vector<double>> read_csv(std::string filename) {
	std::vector<std::vector<double>> m;
	std::ifstream in(filename);
	std::string line;
	while (std::getline(in, line)) {
		std::stringstream ss(line);
		std::vector<double> row;
		std::string data;
		while (std::getline(ss, data, ',')) {
			row.push_back(std::stod(data));
		}
		if (row.size() > 0) {
			m.push_back(row);
		}
	}
	return m;
}


//This function parses the force_motion_generator::export_data (which is returned from the apply_z_force_pid call in the main function) to a csv file
void csv_parser(csv_data data) {
	int length = data.force_command.size();

	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto time_string = oss.str();

	std::string path = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/csv_output/" + time_string + "/";
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

	put_data_in_csv(data.position, path + "position.csv");
	put_data_in_csv(data.position_desired, path + "position_desired.csv");
	put_data_in_csv(data.position_error, path + "position_error.csv");
	put_data_in_csv(data.position_error_integral, path + "position_error_integral.csv");
	put_data_in_csv(data.position_error_diff_filtered, path + "position_error_diff_filtered.csv");

	put_data_in_csv(data.position_command, path + "position_command.csv");
	put_data_in_csv(data.position_command_p, path + "position_command_p.csv");
	put_data_in_csv(data.position_command_i, path + "position_command_i.csv");
	put_data_in_csv(data.position_command_d, path + "position_command_d.csv");
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> give_x_movement_positions(Eigen::Vector3d start_pos) {
	//desired x position
	double a = -0.025; //[m/s^2]
	double t = 8.0; // must be greater than 2.0
	Eigen::Vector3d pos;
	pos = start_pos;
	std::vector<Eigen::Vector3d> des_pos;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> desired_positions;

	for (double i = 0; i < 1.0; i += 0.001) { // 1s linear increasing velocity
		pos(0) = 0.5 * a * i * i + start_pos(0); // s(t=1) = 0.5*a
		des_pos.push_back(pos);
	}
	for (double i = 0; i < (t-2.0); i += 0.001) { // t - 2s constant velocity
		pos(0) = a * i + 0.5 * a + start_pos(0); //s = a * t + s(t=1)
		des_pos.push_back(pos);
	}
	for (double i = 0; i < 1.0; i += 0.001) { // 1s linear decreasing velocity
		pos(0) = -0.5 * a * i * i + a * i + 0.5 * a + (t-2.0) * a + start_pos(0); // s = -0.5 a * t^2 + v(t=4) * t + s(t=4)
		des_pos.push_back(pos);
	}

	for (int i = 0; i < (t*1000); i++) {
		desired_positions.push_back(des_pos[i]);
	}
	for (int i = 0; i < 1000; i++) {
		desired_positions.push_back(des_pos[(t*1000)-1]);
	}
	return desired_positions;
}

std::vector<Eigen::Vector3d> give_circle_positions(Eigen::Vector3d start_pos) {
	double r = 0.1; //radius of circle
	std::vector<Eigen::Vector3d> desired_positions;
	Eigen::Vector3d des_pos = start_pos;
	for (int i = 0; i < 5000; i++) {
		des_pos(0) = (cos(i / 5000.0 * 2 * 3.14159265) - 1.0) * r;
		des_pos(1) = sin(i / 5000.0 * 2 * 3.14159265) * r;
		desired_positions.push_back(des_pos);
	}
}

std::vector<Eigen::Vector3d> give_triangle_positions(Eigen::Vector3d start_pos) {
	std::vector<Eigen::Vector3d> desired_positions;
	Eigen::Vector3d pos = start_pos;
	double a = 0.01;

	for (int i = 0; i < 1000; i++) {
		pos(0) += 0.0001; //10cm
		pos(1) += 0.0001;
		desired_positions.push_back(pos);
	}
	for (int i = 0; i < 2000; i++) {
		pos(0) -= 0.0001;
		desired_positions.push_back(pos);
	}
	for (int i = 0; i < 1000; i++) {
		pos(0) += 0.0001;
		pos(1) -= 0.0001;
		desired_positions.push_back(pos);
	}
	return desired_positions;
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
	std::array<double, 7> pos_0_b = { 0.00167622, 0.583144, -0.00122744, -2.04601, 0.008703, 2.69809, 0.716798 }; //0mm above wood plate with bearing for x_movement
	std::array<double, 7> pos_5_b = { 0.000821532, 0.563336, -0.00267677, -2.05768, 0.00858097, 2.69578, 0.716801 }; //5mm above wood plate with bearing for x_movement
	std::array<double, 7> pos_air_x = { 0.0445302, 0.157452, -0.0226488, -2.44288, 0.0199934, 2.7444, 0.778894 }; //in air (middle) for linear x movement
	std::array<double, 7> pos_air_x_b = { 0.0311876, 0.256293, -0.00171765, -2.62387, 0.00780726, 3.03017, 0.7313 }; //in air (middle) for linear x movement with bearing (lower)

	
	try {
		h_controller.move_to(pos_5_b);
		h_controller.move_to(pos_0_b);
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
	//std::vector<std::vector<double>> m;
	//m = read_csv("C:/Users/hecken/Desktop/BA_Hecken/JohannesDaten/recording16457996444757387.csv");
	Eigen::Matrix<double, 6, 1> des_force;
	des_force.setZero();
	des_force(2) = -10.0;


	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> desired_positions_all;
	std::vector<Eigen::Vector3d> desired_positions;
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
	std::vector<Eigen::Quaterniond> desired_orientations;

	desired_positions_all = give_x_movement_positions(start_pos);

	for (int i = 0; i < desired_positions_all.size(); i++) {
		desired_forces.push_back(des_force);
		desired_orientations.push_back(start_orientation);
		desired_positions.push_back(desired_positions_all[i]);
	}

	/*for (int i = 0; i < 5000; i++) {
		desired_orientations.push_back(start_orientation);
		desired_positions.push_back(start_pos);
		desired_forces.push_back(des_force);
	}*/

	try {
		h_controller.hybrid_control(data, desired_positions, desired_forces, desired_orientations, control_parameters);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception: " << e.what() << std::endl;
		h_controller.automatic_error_recovery();
		throw;
	}

	try {
		h_controller.move_to(pos_air_x_b);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception when moving to air position: " << e.what() << std::endl;
	}

	data.ise_position = calculate_ISE(data.position_error);
	data.ise_force = calculate_ISE(data.force_error);
	data.itae_position = calculate_ITAE(data.position_error);
	data.itae_force = calculate_ITAE(data.force_error);

	csv_parser(data);

	return data;
}

void print_2d_array(std::array<std::array<double, 6>, 6> a) {
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			std::cout << a[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void simulatedAnnnealing(franka_proxy::franka_hardware_controller& h_controller, int dim[12]) {

	//create csv files and write header lines
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto time_string = oss.str();
	std::string path = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/";
	CreateDirectoryA(path.c_str(), NULL);
	std::string filenames[12];
	std::ofstream sa_data_files[12];
	for (int d = 0; d < 12; d++) {
		if (dim[d] != 1) continue;

		filenames[d] = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/sa_overview_" + std::to_string(d) + ".csv";
		sa_data_files[d].open(filenames[d], std::ofstream::out | std::ofstream::app);
		sa_data_files[d] << "k,T,eta,best_F,current_F,new_F,best_Kp,current_Kp,new_Kp,best_Ki,current_Ki,new_Ki,best_Kd,current_Kd,new_Kd,c\n";
		sa_data_files[d].close();
	}
	

	//starting the simulated annealing algorithm
	Eigen::Vector3d x_pos (200, 30, 0);
	Eigen::Vector3d y_pos(200, 30, 0);
	Eigen::Vector3d z_pos(200, 30, 0);
	Eigen::Vector3d mx_pos(30, 5, 0);
	Eigen::Vector3d my_pos(30, 5, 0);
	Eigen::Vector3d mz_pos(20, 5, 0);
	Eigen::Vector3d x_f(0.5, 5, 0);
	Eigen::Vector3d y_f(0.5, 5, 0);
	Eigen::Vector3d z_f(0.3, 10, 0);
	Eigen::Vector3d mx_f(0.05, 0.5, 0);
	Eigen::Vector3d my_f(0.05, 0.5, 0);
	Eigen::Vector3d mz_f(0.05, 0.5, 0);


	std::array<Eigen::Vector3d, 12> initial_parameters = {
	x_pos, y_pos, z_pos, mx_pos, my_pos, mz_pos, x_f, y_f, z_f, mx_f, my_f, mz_f
	};

	Eigen::Vector3d x_pos_max(7500, 5000, 50.0);
	Eigen::Vector3d y_pos_max(200, 30, 0);
	Eigen::Vector3d z_pos_max(200, 30, 0);
	Eigen::Vector3d mx_pos_max(30, 5, 0);
	Eigen::Vector3d my_pos_max(30, 5, 0);
	Eigen::Vector3d mz_pos_max(20, 5, 0);
	Eigen::Vector3d x_f_max(0.5, 5, 0);
	Eigen::Vector3d y_f_max(0.5, 5, 0);
	Eigen::Vector3d z_f_max(0.6, 20, 0.01);
	Eigen::Vector3d mx_f_max(0.05, 0.5, 0);
	Eigen::Vector3d my_f_max(0.05, 0.5, 0);
	Eigen::Vector3d mz_f_max(0.05, 0.5, 0);


	std::array<Eigen::Vector3d, 12> max_parameters = {
	x_pos_max, y_pos_max, z_pos_max, mx_pos_max, my_pos_max, mz_pos_max, x_f_max, y_f_max, z_f_max, mx_f_max, my_f_max, mz_f_max
	};
	
	//random initialization of first parameters
	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::uniform_real_distribution<double> d(0, 1);

	for (int i = 0; i < 12; i++) {
		if (dim[i] != 1) continue;

		Eigen::Vector3d rand (d(gen), d(gen), d(gen));
		initial_parameters[i] = (rand.array() * max_parameters[i].array()).matrix();
	}

	//call hybrid control with first random parameterSet and calculate start F
	csv_data data{};

	double initial_F = 0.0;
	double p_factor = 100.0;
	double f_factor = 1.0;

	std::cout << "Evaluating first random parameter set..." << std::endl;
	try {
		//This 2d array is used by the robot
		std::array<std::array<double, 6>, 6> control_parameters;
		control_parameters[0] = { initial_parameters[0](0), initial_parameters[1](0), initial_parameters[2](0), initial_parameters[3](0), initial_parameters[4](0), initial_parameters[5](0) };
		control_parameters[1] = { initial_parameters[0](1), initial_parameters[1](1), initial_parameters[2](1), initial_parameters[3](1), initial_parameters[4](1), initial_parameters[5](1) };
		control_parameters[2] = { initial_parameters[0](2), initial_parameters[1](2), initial_parameters[2](2), initial_parameters[3](2), initial_parameters[4](2), initial_parameters[5](2) };
		control_parameters[3] = { initial_parameters[6](0), initial_parameters[7](0), initial_parameters[8](0), initial_parameters[9](0), initial_parameters[10](0), initial_parameters[11](0) };
		control_parameters[4] = { initial_parameters[6](1), initial_parameters[7](1), initial_parameters[8](1), initial_parameters[9](1), initial_parameters[10](1), initial_parameters[11](1) };
		control_parameters[5] = { initial_parameters[6](2), initial_parameters[7](2), initial_parameters[8](2), initial_parameters[9](2), initial_parameters[10](2), initial_parameters[11](2) };


		apply_z_force(h_controller, control_parameters, data);
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			if (d < 6) {
				initial_F += p_factor * data.itae_position(d, 0);
			}
			else {
				initial_F += f_factor * data.itae_force(d - 6, 0);
			}
		}
	}
	catch (const franka::Exception& e) {
		std::cout << "First random set of parameters caused an exception..." << std::endl;
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;
			std::cout << " Dim: " << d << " (" <<
				initial_parameters[d](0) << ", " << initial_parameters[d](1) << ", " << initial_parameters[d](2) << ")";
		}
		throw;
	}

	double current_F = initial_F;
	double best_F = initial_F;
	std::array<Eigen::Vector3d, 12> current_parameters = initial_parameters;
	std::array<Eigen::Vector3d, 12> best_parameters = initial_parameters;

	double T = 5.0; //initial T
	double eta = 0.25; //initial eta
	int c = 0; //counter for consecutive remaining parameterVector
	int c_max = 10;
	int exc = 0; //counter for consecutive exceptions
	int exc_max = 10; //the programm will abort if this number of consecutive exceptions happen
	int k = 1; //only used in csv file
	double mu = 0.0;

	double sigma = 1.0;
	double l = 0.97;
	double delta_F = 0.1;

	int m = 3; //the median of m F values will be used

	while (c < c_max && exc < exc_max) {

		//calculate new parameter set (neighbour) based on current paramter set
		std::normal_distribution<double> nd(mu, sigma);
		std::array<Eigen::Vector3d, 12> new_parameters = current_parameters;

		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			for (int j = 0; j < 3; j++) {
				do {
					new_parameters[d](j) = current_parameters[d](j) + eta * nd(gen) * max_parameters[d](j);
				} while (new_parameters[d](j) > max_parameters[d](j) || new_parameters[d](j) < 0);
			}
		}
				
		bool catched_e = false;
		double new_F_sum = 0.0;

		for (int i = 0; i < m; i++) {
			csv_data data{};
			try {
				//This 2d array is used by the robot
				std::array<std::array<double, 6>, 6> control_parameters;
				control_parameters[0] = { new_parameters[0](0), new_parameters[1](0), new_parameters[2](0), new_parameters[3](0), new_parameters[4](0), new_parameters[5](0) };
				control_parameters[1] = { new_parameters[0](1), new_parameters[1](1), new_parameters[2](1), new_parameters[3](1), new_parameters[4](1), new_parameters[5](1) };
				control_parameters[2] = { new_parameters[0](2), new_parameters[1](2), new_parameters[2](2), new_parameters[3](2), new_parameters[4](2), new_parameters[5](2) };
				control_parameters[3] = { new_parameters[6](0), new_parameters[7](0), new_parameters[8](0), new_parameters[9](0), new_parameters[10](0), new_parameters[11](0) };
				control_parameters[4] = { new_parameters[6](1), new_parameters[7](1), new_parameters[8](1), new_parameters[9](1), new_parameters[10](1), new_parameters[11](1) };
				control_parameters[5] = { new_parameters[6](2), new_parameters[7](2), new_parameters[8](2), new_parameters[9](2), new_parameters[10](2), new_parameters[11](2) };

				apply_z_force(h_controller, control_parameters, data);

				for (int d = 0; d < 12; d++) {
					if (dim[d] != 1) continue;

					if (d < 6) {
						new_F_sum += p_factor * data.itae_position(d, 0);
					}
					else {
						new_F_sum += f_factor * data.itae_force(d - 6, 0);
					}
				}
				exc = 0;
			}
			catch (const franka::Exception& e) {
				c = 0;
				exc++;
				std::cout << "The following parameter set caused an exception! ";
				for (int d = 0; d < 12; d++) {
					if (dim[d] != 1) continue;
					std::cout << " Dim: " << d << " (" <<
						new_parameters[d](0) << ", " << new_parameters[d](1) << ", " << new_parameters[d](2) << ")";
				}
				std::cout << std::endl;
				catched_e = true;
				break;
			}
		}
		if (catched_e) {
			continue;
		}

		double new_F = new_F_sum / m;		


		//printing on console (K_D is printed on console with a factor 1000)
		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "k=" << k << "\tT=" << T << "\tbestF=" << best_F << "\tcurrentF=" << current_F << "\tnewF=" << new_F << "\tc=" << c;
		if ((exp(-(new_F - current_F) / T)) < 1.0) {
			std::cout << "\tprop_worse = " << (exp(-(new_F - current_F) / T));
		}
		std::cout << "\n";
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			std::cout << "Dim=" << d << "\tcurrentParams=(" << current_parameters[d](0) << "," << current_parameters[d](1) << "," << 1000 * current_parameters[d](2) << ")"
				<< "\tnewParams=(" << new_parameters[d](0) << "," << new_parameters[d](1) << "," << 1000.0 * new_parameters[d](2) << ")\n";
		}
		std::cout << "\n";
		

		//write values in sa_overview.csv
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			sa_data_files[d].open(filenames[d], std::ofstream::out | std::ofstream::app);
			sa_data_files[d] << k << "," << T << "," << eta << ","
				<< best_F << "," << current_F << "," << new_F << ","
				<< best_parameters[d](0) << "," << current_parameters[d](0) << "," << new_parameters[d](0) << ","
				<< best_parameters[d](1) << "," << current_parameters[d](1) << "," << new_parameters[d](1) << ","
				<< best_parameters[d](2) << "," << current_parameters[d](2) << "," << new_parameters[d](2) << ","
				<< c << "\n";
			sa_data_files[d].close();
		}
		

		if (new_F < current_F) { //new parameterVector is better then change to this parameterVector
			current_parameters = new_parameters;
			if (std::abs(current_F - new_F) > delta_F) { //only set termination criteria c to zero if the change in F is significant
				c = 0;
			}
			else {
				c++;
			}
			current_F = new_F;
			
		}
		else { //if current parameterVector was better, then only accept with boltzmann-related chance
			double r = d(gen); //rand[0,1]

			if (r < exp(-(new_F - current_F) / T)) { //Boltzmann
				current_parameters = new_parameters;
				if (std::abs(current_F - new_F) > delta_F) { //only set termination criteria c to zero if the change in F is significant
					c = 0;
				}
				else {
					c++;
				}
				current_F = new_F;
			}
			else {
				c++; // if nothing changes increase c for termination criterium
			}
		}

		//Save the overall best value
		if (new_F < best_F) {
			best_F = new_F;
			best_parameters = new_parameters;
		}


		//Decrease T and eta
		T = l * T;
		eta = l * eta;

		k++;
	}
}

int main() {

	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	int dim[12] = {
		1,0,0,0,0,0, //position (x, y, z, mx, my, mz)
		0,0,1,0,0,0 //force (x, y, z, mx, my, mz)
	};
	simulatedAnnnealing(h_controller, dim);

	//print_cur_joint_pos(h_controller);

	return 0;
}




