/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__CSV_PARSER_HPP)
#define INCLUDED__FRANKA_PROXY__SIMULATED_ANNEALING_HPP


#include "franka_hardware_controller.hpp"
#include "franka_network_control_server.hpp"
#include "franka_network_state_server.hpp"
#include "csv_data_struct.hpp"
#include <atomic>
#include <thread>



#include <franka/exception.h>


namespace franka_proxy
{

	class csv_parser {

	public:
		
		void parsing(csv_data data);
		void write_in_overview(std::array<int, 12> dim, int k, int c, double T, double eta, double best_F, double current_F, double new_F, std::array<Eigen::Vector3d, 12> best_parameters, std::array<Eigen::Vector3d, 12> current_parameters, std::array<Eigen::Vector3d, 12> new_parameters);
		void create_header(std::array<int, 12> dim);

	private:
		void put_data_in_csv(std::vector<Eigen::Matrix<double, 6, 1>> data, std::string file_name);
		void create_overview_csv(csv_data data, std::string file_name);

		std::array<std::string, 12> filenames_;
		std::array<std::ofstream, 12> sa_data_files_;

	};



} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__CSV_PARSER_HPP) */
