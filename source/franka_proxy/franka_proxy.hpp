/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP


#include "franka_hardware_controller.hpp"
#include "franka_network_control_server.hpp"
#include "franka_network_state_server.hpp"
#include "motion_generator_force.hpp" //necessary for the export_data struct

#include <franka/exception.h>


namespace franka_proxy
{
/**
 *************************************************************************
 *
 * @class franka_proxy
 *
 * todo
 *
 ************************************************************************/
	struct csv_data {
		std::array<double, 6> k_p_f;
		std::array<double, 6> k_i_f;
		std::array<double, 6> k_d_f;
		std::array<double, 6> k_p_p;
		std::array<double, 6> k_i_p;
		std::array<double, 6> k_d_p;

		std::vector<Eigen::Matrix<double, 6, 7 >> zero_jacobian;
		std::vector<Eigen::Matrix<double, 6, 1 >> o_F_ext_hat_K;
		std::vector<Eigen::Matrix<double, 6, 1>> force_desired;
		std::vector<Eigen::Matrix<double, 6, 1>> force_error;
		std::vector<Eigen::Matrix<double, 6, 1>> force_error_integral;
		std::vector<Eigen::Matrix<double, 6, 1>> force_error_diff_filtered;

		std::vector<Eigen::Matrix<double, 6, 1>> force_command;
		std::vector<Eigen::Matrix<double, 6, 1>> force_command_p;
		std::vector<Eigen::Matrix<double, 6, 1>> force_command_i;
		std::vector<Eigen::Matrix<double, 6, 1>> force_command_d;

		std::vector<Eigen::Matrix<double, 6, 1>> position_error;
		std::vector<Eigen::Matrix<double, 6, 1>> position_error_integral;
		std::vector<Eigen::Matrix<double, 6, 1>> position_error_diff_filtered;

		std::vector<Eigen::Matrix<double, 6, 1>> position_command;
		std::vector<Eigen::Matrix<double, 6, 1>> position_command_p;
		std::vector<Eigen::Matrix<double, 6, 1>> position_command_i;
		std::vector<Eigen::Matrix<double, 6, 1>> position_command_d;

		std::vector<Eigen::Matrix<double, 7, 1>> tau_command;
		std::vector<Eigen::Matrix<double, 7, 1>> tau_command_filtered;
		std::vector<Eigen::Matrix<double, 7, 1>> coriolis;
	};
class franka_proxy
{
	

public:

	

	franka_proxy();


private:

	franka_hardware_controller controller_;

	franka_control_server control_server_;
	franka_state_server state_server_;

    static constexpr unsigned short franka_control_port = 4711;
    static constexpr unsigned short franka_state_port = 4712;
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_PROXY_HPP) */
