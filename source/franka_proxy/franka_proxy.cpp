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
	//std::vector<std::vector<double>> m;
	//m = read_csv("C:/Users/hecken/Desktop/BA_Hecken/JohannesDaten/recording16457996444757387.csv");
	Eigen::Matrix<double, 6, 1> des_force;
	des_force.setZero();
	des_force(2) = -1.0;


	std::vector<Eigen::Vector3d> desired_positions;
	std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
	std::vector<Eigen::Quaterniond> desired_orientations;

	//desired x position
	double a = -0.02; //[m/s^2]
	double t = 8.0; // must be greater than 2.0
	Eigen::Vector3d pos;
	pos = start_pos;
	std::vector<Eigen::Vector3d> des_pos;

	for (double i = 0; i < 1.0; i += 0.001) { // 1s linear increasing velocity
		pos(0) = 0.5 * a * i * i + start_pos(0);
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
		desired_orientations.push_back(start_orientation);
		//desired_positions.push_back(start_pos);
		desired_positions.push_back(des_pos[i]);
		//des_force(2) = -m[i][3];
		desired_forces.push_back(des_force);
	}
	for (int i = 0; i < 1000; i++) {
		desired_orientations.push_back(start_orientation);
		desired_positions.push_back(des_pos[(t*1000)-1]);
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


Eigen::Vector3d simulatedAnnnealing(franka_proxy::franka_hardware_controller& h_controller) {
	//create ccs file and write header line
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto time_string = oss.str();
	std::string path = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/";
	CreateDirectoryA(path.c_str(), NULL);
	std::string filename = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/sa_overview.csv";
	std::ofstream sa_data_file;
	sa_data_file.open(filename, std::ofstream::out | std::ofstream::app);
	sa_data_file << "k,T,eta,best_F,current_F,new_F,best_Kp,current_Kp,new_Kp,best_Ki,current_Ki,new_Ki,best_Kd,current_Kd,new_Kd,c\n";
	sa_data_file.close();

	//Position Parameters which are based on experience
	std::array<double, 6> k_p_p = { -200, -200, -200, -30, -30, -20 };
	std::array<double, 6> k_i_p = { -30, -30, -30, -5, -5, -5 };
	std::array<double, 6> k_d_p = { 0, 0, 0, 0, 0, 0 };

	//Force Parameters which are based on experience - z-Dimension will be set by SA Alg
	std::array<double, 6> k_p_f = { 0.5, 0.5, 0 , 0.05, 0.05, 0.05 };
	std::array<double, 6> k_i_f = { 5, 5, 0, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_d_f = { 0, 0, 0, 0, 0, 0 };

	//This array will be modified by SA Alg and is used by the robot
	std::array<std::array<double, 6>, 6> control_parameters;
	control_parameters = { k_p_p, k_i_p, k_d_p, k_p_f, k_i_f, k_d_f };

	//starting the simulated annealing algorithm
	
	//setting max values for the z-force parameters
	Eigen::Vector3d max_parameters(0.6, 20.0, 0.01);

	//random initialization of first parameter set
	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::uniform_real_distribution<double> d(0, 1);
	Eigen::Vector3d rand_vector(d(gen), d(gen), d(gen));
	Eigen::Vector3d initial_parameter_vector = (rand_vector.array() * max_parameters.array()).matrix();
	control_parameters[3][2] = initial_parameter_vector(0);
	control_parameters[4][2] = initial_parameter_vector(1);
	control_parameters[4][2] = initial_parameter_vector(2);

	//call hybrid control with first random parameterSet and calculate start F
	csv_data data{};
	double initial_F;
	std::cout << "Evaluating first random parameter set..." << std::endl;
	try {
		apply_z_force(h_controller, control_parameters, data);
		initial_F = data.itae_force(2); //cost Function
	}
	catch (const franka::Exception& e) {
		std::cout << "First random set of parameters (" << initial_parameter_vector(0) << ", " << initial_parameter_vector(1) << ", " 
			<< initial_parameter_vector(2) << ") caused exception..." << std::endl;
		throw;
	}
	double current_F = initial_F;
	double best_F = initial_F;
	Eigen::Vector3d current_parameter_vector = initial_parameter_vector;
	Eigen::Vector3d best_parameter_vector = initial_parameter_vector;

	double T = 0.1; //initial T
	double eta = 0.25; //initial eta
	int c = 0; //counter for consecutive remaining parameterVector
	int c_max = 20;
	int exc = 0; //counter for consecutive exceptions
	int exc_max = 10; //the programm will abort if this number of consecutive exceptions happen
	int k = 1; //used in csv file
	double mu = 0.0;

	double sigma = 1.0;
	double l = 0.99;
	double delta_F = 0.01;

	while (c < c_max && exc < exc_max) {

		//calculate new parameter set (neighbour) based on current paramter set
		std::normal_distribution<double> nd(mu, sigma);
		Eigen::Vector3d new_parameter_vector;
		for (int i = 0; i < 3; i++) {
			do {
				new_parameter_vector(i) = current_parameter_vector(i) + eta * nd(gen) * max_parameters(i);
			} while (new_parameter_vector(i) > max_parameters(i) || new_parameter_vector(i) < 0);
		}

		//call hybrid_control with newParameterVector and calculate new F
		control_parameters[3][2] = new_parameter_vector(0);
		control_parameters[4][2] = new_parameter_vector(1);
		control_parameters[5][2] = new_parameter_vector(2);

		csv_data data{};
		double new_F;
		try {
			apply_z_force(h_controller, control_parameters, data);
			new_F = data.itae_force(2);
			exc = 0;
		}
		catch (const franka::Exception& e) {
			c = 0;
			exc++;
			std::cout << "The parameter set: ("
				<< new_parameter_vector(0) << ", " << new_parameter_vector(1) << ", " << new_parameter_vector(2)*1000
				<< ") caused an exception!" << std::endl;
			continue;
		}

		//printing on console (K_D is printed on console with a factor 1000)
		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "k=" << k << "\tT=" << T
			<< "\tbestF=" << best_F << "\tcurrentF=" << current_F << "\tnewF=" << new_F
			<< "\tcurrentParams=(" << current_parameter_vector(0) << "," << current_parameter_vector(1) << "," << 1000 * current_parameter_vector(2) << ")"
			<< "\tnewParams=(" << new_parameter_vector(0) << "," << new_parameter_vector(1) << "," << 1000.0 * new_parameter_vector(2) << ")"
			<< "    c=" << c;
		if ((exp(-(new_F - current_F) / T)) < 1.0) {
			std::cout << "    prop_worse = " << (exp(-(new_F - current_F) / T));
		}
		std::cout << "\n";

		//write values in sa_overview.csv
		sa_data_file.open(filename, std::ofstream::out | std::ofstream::app);
		sa_data_file << k << "," << T << "," << eta << "," 
			<< best_F << "," << current_F << "," << new_F << ","
			<< best_parameter_vector(0) << "," << current_parameter_vector(0) << "," << new_parameter_vector(0)	<< ","
			<< best_parameter_vector(1) << "," << current_parameter_vector(1) << "," << new_parameter_vector(1) << "," 
			<< best_parameter_vector(2) << "," << current_parameter_vector(2) << "," << new_parameter_vector(2) << ","
			<< c << "\n";
		sa_data_file.close();

		if (new_F < current_F) { //new parameterVector is better then change to this parameterVector
			current_parameter_vector = new_parameter_vector;
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
				current_parameter_vector = new_parameter_vector;
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
			best_parameter_vector = new_parameter_vector;
		}


		//Decrease T and eta
		T = l * T;
		eta = l * eta;

		k++;
	}
	return best_parameter_vector;
}

int main() {

	franka_proxy::franka_hardware_controller h_controller("192.168.1.1");

	std::cout << "Starting in 2 seconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));

	//Eigen::Vector3d param_set = simulatedAnnnealing(h_controller);
	//std::cout << "SA Alg gives parameter set: " << param_set << std::endl;


	//Position Parameters
	std::array<double, 6> k_p_p = { -3000, -100, -100, -100, -1500, -100 };
	std::array<double, 6> k_i_p = { 0, 0, 0, 0, 0, 0 };
	std::array<double, 6> k_d_p = { 0, 0, 0, 0, 0, 0 };

	//Ziegler Nichols Method Force Parameters
	std::array<double, 6> k_p_f = { 0.2, 0.2, 0.02, 0.02, 0.05, 0.05 };
	std::array<double, 6> k_i_f = { 5, 5, 2, 0.5, 0.5, 0.5 };
	std::array<double, 6> k_d_f = { 0, 0, 0, 0, 0, 0 };

	std::array<std::array<double, 6>, 6> control_parameters;
	control_parameters = { k_p_p, k_i_p, k_d_p, k_p_f, k_i_f, k_d_f };
	

	csv_data data{};
	apply_z_force(h_controller, control_parameters, data);

	/*for (int i = 0; i < 20; i++) {
		csv_data data{};
		apply_z_force(h_controller, control_parameters, data);
		std::cout << data.itae_force(2) << std::endl;
	}*/

	std::array<double, 7> pos_air_x = { 0.0445302, 0.157452, -0.0226488, -2.44288, 0.0199934, 2.7444, 0.778894 }; //in air (middle) for linear x movement
	try {
		h_controller.move_to(pos_air_x);
	}
	catch (const franka::Exception& e) {
		std::cout << "catched Exception when moving to air position: " << e.what() << std::endl;
	}

	return 0;
}




