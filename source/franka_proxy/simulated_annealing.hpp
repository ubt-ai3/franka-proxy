/**
 *************************************************************************
 *
 * @file franka_proxy.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__SIMULATED_ANNEALING_HPP)
#define INCLUDED__FRANKA_PROXY__SIMULATED_ANNEALING_HPP


#include "franka_hardware_controller.hpp"
#include "franka_network_control_server.hpp"
#include "franka_network_state_server.hpp"
#include <atomic>
#include <thread>



#include <franka/exception.h>


namespace franka_proxy
{


class hyb_con_pid_optimizer {

public:

	hyb_con_pid_optimizer(franka_hardware_controller& h_controller);
	~hyb_con_pid_optimizer() noexcept;	

	void start();
	void stop();
	bool is_running();

	void simulated_annealing();

private:


	std::atomic_bool run_;
	std::thread sa_thread_;

	franka_hardware_controller& hc_;


	// the int dim_[12] array indicated which pid controllers of which dimension are optimized (0 not to be optimized || 1 to be optimized):
	int dim_[12] = {
		1,1,0,1,1,1, //position (x, y, z, mx, my, mz)
		0,0,1,0,0,0 //force (x, y, z, mx, my, mz)
	};

	double T_start_ = 30.0; //starting Temperature
	double l_start_ = 0.98; //starting cooling factor
	double eta_start_ = 0.25; //starting scaling factor

	int m_ = 5; //the median of m-times of the F values will be used

	int c_max_ = 10; //the programm will abort if this number of consecutive steps result in none or minor changes in F
	int exc_max_ = 10; //the programm will abort if this number of consecutive exceptions happen
	double delta_F_ = 2.0; //this difference in F will not reset the termination criteria

	double mu_ = 0.0; //mean value of normal distribution
	double sigma_ = 1.0; //variance value of normal distribution		

	std::array<double, 7> pos_30_ = { 0.0141143, 0.744292, -0.0176676, -1.6264, 0.0207479, 2.41293, 0.724183 }; //30mm above wood plate with blue part
	std::array<double, 7> pos_10_ = { 0.0166511, 0.777224, -0.0173561, -1.61821, 0.020813, 2.45273, 0.724666 }; //10mm above wood plate with blue part
	std::array<double, 7> pos_2_ = { 0.0173603, 0.782011, -0.0172685, -1.61904, 0.0207746, 2.45505, 0.724928 }; //2mm above wood plate with blue part
	std::array<double, 7> pos_0_ = { 0.0159666, 0.784819, -0.0174431, -1.6166, 0.0208109, 2.45508, 0.724577 }; //0mm above wood plate with blue part
	std::array<double, 7> pos_0_b_ = { 0.00167622, 0.583144, -0.00122744, -2.04601, 0.008703, 2.69809, 0.716798 }; //0mm above wood plate with bearing for x_movement
	std::array<double, 7> pos_5_b_ = { 0.000821532, 0.563336, -0.00267677, -2.05768, 0.00858097, 2.69578, 0.716801 }; //5mm above wood plate with bearing for x_movement
	std::array<double, 7> pos_air_x_ = { 0.0445302, 0.157452, -0.0226488, -2.44288, 0.0199934, 2.7444, 0.778894 }; //in air (middle) for linear x movement
	std::array<double, 7> pos_air_x_b_ = { 0.0311876, 0.256293, -0.00171765, -2.62387, 0.00780726, 3.03017, 0.7313 }; //in air (middle) for linear x movement with bearing (lower)

	
	csv_data evaluate_params(franka_hardware_controller& h_controller, std::array<std::array<double, 6>, 6> control_parameters, csv_data& data);
	double calculate_F(int dim[12], csv_data& data);
	void set_sa_params(double T, double l, double eta);
	std::array<std::array<double, 6>, 6> format_control_parameters(std::array<Eigen::Vector3d, 12> eigen_parameters);
	Eigen::Matrix<double, 6, 1> calculate_ISE(std::vector<Eigen::Matrix<double, 6, 1>>& values);
	Eigen::Matrix<double, 6, 1> calculate_ITAE(std::vector<Eigen::Matrix<double, 6, 1>>& values);
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__SIMULATED_ANNEALING_HPP) */
