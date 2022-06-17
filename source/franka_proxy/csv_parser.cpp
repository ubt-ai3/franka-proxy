/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/



#include "csv_parser.hpp"
#include "csv_data_struct.hpp"

#include <iostream>
#include <iomanip>
#include <random>
#include<Eigen/StdVector>


namespace franka_proxy
{
	void csv_parser::create_header(std::array<int, 12> dim) {
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		std::ostringstream oss;
		oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
		auto time_string = oss.str();
		std::string path = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/";
		CreateDirectoryA(path.c_str(), NULL);
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			filenames_[d] = "H:/DB_Forschung/flexPro/11.Unterprojekte/BA_Laurin_Hecken/05_Rohdaten/sa_overview_output/" + time_string + "/sa_overview_" + std::to_string(d) + ".csv";
			sa_data_files_[d].open(filenames_[d], std::ofstream::out | std::ofstream::app);
			sa_data_files_[d] << "k,T,eta,best_F,current_F,new_F,best_Kp,current_Kp,new_Kp,best_Ki,current_Ki,new_Ki,best_Kd,current_Kd,new_Kd,c\n";
			sa_data_files_[d].close();
		}
	}

	void csv_parser::write_in_overview(std::array<int, 12> dim, int k, int c, double T, double eta, double best_F, double current_F, double new_F, std::array<Eigen::Vector3d, 12> best_parameters, std::array<Eigen::Vector3d, 12> current_parameters, std::array<Eigen::Vector3d, 12> new_parameters) {
		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;

			sa_data_files_[d].open(filenames_[d], std::ofstream::out | std::ofstream::app);
			sa_data_files_[d] << k << "," << T << "," << eta << ","
				<< best_F << "," << current_F << "," << new_F << ","
				<< best_parameters[d](0) << "," << current_parameters[d](0) << "," << new_parameters[d](0) << ","
				<< best_parameters[d](1) << "," << current_parameters[d](1) << "," << new_parameters[d](1) << ","
				<< best_parameters[d](2) << "," << current_parameters[d](2) << "," << new_parameters[d](2) << ","
				<< c << "\n";
			sa_data_files_[d].close();
		}
	}

	
	void csv_parser::put_data_in_csv(std::vector<Eigen::Matrix<double, 6, 1>> data, std::string file_name) {
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

	void csv_parser::create_overview_csv(csv_data data, std::string file_name) {
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
	void csv_parser::parsing(csv_data data) {
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

}





