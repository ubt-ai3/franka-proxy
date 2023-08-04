/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "simulated_annealing.hpp"
#include "motion_generator_force.hpp"
#include "csv_data_struct.hpp"

#include <iostream>
#include <iomanip>
#include <random>
#include<Eigen/StdVector>

namespace franka_proxy
{


	//This function is only needed for temporary reasons

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
		for (double i = 0; i < (t - 2.0); i += 0.001) { // t - 2s constant velocity
			pos(0) = a * i + 0.5 * a + start_pos(0); //s = a * t + s(t=1)
			des_pos.push_back(pos);
		}
		for (double i = 0; i < 1.0; i += 0.001) { // 1s linear decreasing velocity
			pos(0) = -0.5 * a * i * i + a * i + 0.5 * a + (t - 2.0) * a + start_pos(0); // s = -0.5 a * t^2 + v(t=4) * t + s(t=4)
			des_pos.push_back(pos);
		}

		for (int i = 0; i < (t * 1000); i++) {
			desired_positions.push_back(des_pos[i]);
		}
		for (int i = 0; i < 1000; i++) {
			desired_positions.push_back(des_pos[(t * 1000) - 1]);
		}
		return desired_positions;
	}



	//This function is only needed for temporary reasons

	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> give_circle_positions(Eigen::Vector3d start_pos) {
		double r = 0.1; //radius of circle
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> desired_positions;
		Eigen::Vector3d des_pos = start_pos;
		int m = 10000;
		for (int i = 0; i < m; i++) {
			des_pos(0) = (cos((double)i / (double)m * 2 * 3.14159265) - 1.0) * r + start_pos(0);
			des_pos(1) = sin((double)i / (double)m * 2 * 3.14159265) * r + start_pos(1);
			desired_positions.push_back(des_pos);
		}
		for (int i = 0; i < 1000; i++) {
			desired_positions.push_back(des_pos);
		}
		return desired_positions;
	}





	
	//------------------------------------------HYB_CON_PID_OPTIMIZER CLASS------------------------------------------------
	// 
	//optimizes pid control parameters for the hybrid control of the franka via an automatic simulated annealing algorithm
	//arguments: franka hardware controller reference, dimension array (which dimension should be optimized), bool should there be data_tracking ion csv files

	//___________________________________PUBLIC____________________________________________________________________________

	hyb_con_pid_optimizer::hyb_con_pid_optimizer(franka_hardware_controller& h_controller, std::array<int, 12> dim, bool data_tracking, std::string base_path) :
		hc_(h_controller),
		dim_(dim),
		csv_parser_(base_path),
		data_tracking_(data_tracking)
	{}

	hyb_con_pid_optimizer::~hyb_con_pid_optimizer() noexcept{
		run_ = false;
		if (sa_thread_.joinable())
			sa_thread_.join();
	}

	//checks for a correct optimization dimension vector and then starts the sa_thread
	void hyb_con_pid_optimizer::start() {
		for (int i = 0; i <= 5; i++) {
			if (dim_[i] == 1 && dim_[i + 6]) {
				throw std::invalid_argument("invalid optimization dimension vector!");
			}
		}
		run_ = true;
		sa_thread_ = std::thread(&hyb_con_pid_optimizer::simulated_annealing, this);
		std::cout << "start" << std::endl;
	}


	void hyb_con_pid_optimizer::stop() {
		run_ = false;
		std::cout << "stop.. waiting for join" << std::endl;
		if (sa_thread_.joinable())
			sa_thread_.join();
		std::cout << "stop" << std::endl;
	}


	bool hyb_con_pid_optimizer::is_running() {
		return run_;
	}

	
	//Basic framework for the optimization of the hybrid pid control.
	//This prodecure is based on the simulated annealing algorithm, which tries to find the global optimum of an optimization problem
	//A Simulated annnealing algorithm accepts worse solutions with a certain propability based on the current Temperature.
	//The Temperature cools down after every loop pass

	void hyb_con_pid_optimizer::simulated_annealing() {

		//create csv files and write header lines
		if (data_tracking_) {
			csv_parser_.create_header(dim_);
		}
		


		//starting the simulated annealing algorithm
		Eigen::Vector3d x_pos(200, 30, 0);
		Eigen::Vector3d y_pos(200, 30, 0);
		Eigen::Vector3d z_pos(200, 30, 0);
		Eigen::Vector3d mx_pos(30, 5, 0);
		Eigen::Vector3d my_pos(30, 5, 0);
		Eigen::Vector3d mz_pos(20, 5, 0);
		Eigen::Vector3d x_f(0.5, 5, 0);
		Eigen::Vector3d y_f(0.5, 5, 0);
		Eigen::Vector3d z_f(0.2, 10, 0);
		Eigen::Vector3d mx_f(0.05, 0.5, 0);
		Eigen::Vector3d my_f(0.05, 0.5, 0);
		Eigen::Vector3d mz_f(0.05, 0.5, 0);

		std::array<Eigen::Vector3d, 12> initial_parameters = {
		x_pos, y_pos, z_pos, mx_pos, my_pos, mz_pos, x_f, y_f, z_f, mx_f, my_f, mz_f
		};


		Eigen::Vector3d x_pos_max(1000, 1000, 0);
		Eigen::Vector3d y_pos_max(1000, 1000, 0);
		Eigen::Vector3d z_pos_max(200, 30, 0);
		Eigen::Vector3d mx_pos_max(100, 20, 0);
		Eigen::Vector3d my_pos_max(100, 20, 0);
		Eigen::Vector3d mz_pos_max(100, 20, 0);
		Eigen::Vector3d x_f_max(0.5, 5, 0);
		Eigen::Vector3d y_f_max(0.5, 5, 0);
		Eigen::Vector3d z_f_max(0.4, 20, 0.01);
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
			if (dim_[i] != 1) continue;

			Eigen::Vector3d rand(d(gen), d(gen), d(gen));
			initial_parameters[i] = (rand.array() * max_parameters[i].array()).matrix();
		}


		//call hybrid control with first random parameterSet and calculate cost (F) of initial parameter set
		csv_data initial_data{};

		std::cout << "Evaluating first random parameter set..." << std::endl;
		try {
			std::array<std::array<double, 6>, 6> control_parameters = format_control_parameters(initial_parameters); //This 2d array is used by the robot

			evaluate_params(hc_, control_parameters, initial_data);
		}
		catch (const franka::Exception& e) {
			std::cout << "First random set of parameters caused an exception..." << std::endl;
			for (int d = 0; d < 12; d++) {
				if (dim_[d] != 1) continue;
				std::cout << " Dim: " << d << " (" <<
					initial_parameters[d](0) << ", " << initial_parameters[d](1) << ", " << initial_parameters[d](2) << ")\n";
			}
			return;
		}

		double current_F = calculate_F(dim_, initial_data);
		double best_F = current_F;

		std::array<Eigen::Vector3d, 12> current_parameters = initial_parameters;
		std::array<Eigen::Vector3d, 12> best_parameters = initial_parameters;

		double T = T_start_;
		double l = l_start_;
		double eta = eta_start_;

		int c = 0; //counter for consecutive remaining parameterVector
		int exc = 0; //counter for consecutive exceptions
		int k = 1; //only used in csv file

		


		//--------------------------Starting the SA loop-------------------------------
		while ((c < c_max_ && exc < exc_max_) && run_) {

			//calculate new parameter set (neighbour) based on current paramter set
			std::normal_distribution<double> nd(mu_, sigma_);
			std::array<Eigen::Vector3d, 12> new_parameters = current_parameters;

			for (int d = 0; d < 12; d++) {
				if (dim_[d] != 1) continue;

				for (int j = 0; j < 3; j++) {
					do {
						new_parameters[d](j) = current_parameters[d](j) + eta * nd(gen) * max_parameters[d](j);
					} while (new_parameters[d](j) > max_parameters[d](j) || new_parameters[d](j) < 0);
				}
			}
			bool catched_e = false;


			//calculate the cost F from m consecutive measurements to reduce impact of measurement noise
			double new_F_sum = 0.0;

			for (int i = 0; i < m_; i++) {
				csv_data data{};
				try {
					std::array<std::array<double, 6>, 6> control_parameters = format_control_parameters(new_parameters); //This 2d array is used by the robot

					evaluate_params(hc_, control_parameters, data);

					new_F_sum += calculate_F(dim_, data);

					exc = 0;
				}

				catch (const franka::Exception& e) {
					c = 0;
					exc++;

					std::cout << "The following parameter set caused an exception!\n";

					for (int d = 0; d < 12; d++) {
						if (dim_[d] != 1) continue;
						std::cout << " Dim: " << d << " (" <<
							new_parameters[d](0) << ", " << new_parameters[d](1) << ", " << new_parameters[d](2) << ")\n";
					}
					std::cout << std::endl;
					catched_e = true;
					std::this_thread::sleep_for(std::chrono::seconds(5)); //use this time to reset the robot arms position
					break;
				}
			}
			if (catched_e) {
				continue;
			}

			double new_F = new_F_sum / m_;


			//printing on console (K_D is printed on console with a factor 1000)
			std::cout << std::fixed;
			std::cout << std::setprecision(4);
			std::cout << "k=" << k << "\tT=" << T << "\tbestF=" << best_F << "\tcurrentF=" << current_F << "\tnewF=" << new_F << "\tc=" << c;
			if ((exp(-(new_F - current_F) / T)) < 1.0) {
				std::cout << "\tprop_worse = " << (exp(-(new_F - current_F) / T));
			}
			std::cout << "\n";
			for (int d = 0; d < 12; d++) {
				if (dim_[d] != 1) continue;

				std::cout << "Dim=" << d;

				std::cout << "\tcurrentParams=(" << current_parameters[d](0) << "," << current_parameters[d](1) << "," << 1000 * current_parameters[d](2) << ")"
					<< "\tnewParams=(" << new_parameters[d](0) << "," << new_parameters[d](1) << "," << 1000.0 * new_parameters[d](2) << ")\n";
			}
			std::cout << "\n";

			
			//write values in sa_overview.csv
			if (data_tracking_) {
				csv_parser_.write_in_overview(dim_, k, c, T, eta, best_F, current_F, new_F, best_parameters, current_parameters, new_parameters);
			}
			
			


			if (new_F < current_F) { //new parameterVector is better then change to this parameterVector
				current_parameters = new_parameters;
				if (std::abs(current_F - new_F) > delta_F_) { //only set termination criteria c to zero if the change in F is significant
					c = 0;
				}
				else {
					c++; // if nothing changes increase c for termination criterium
				}
				current_F = new_F;

			}

			else { //if current parameterVector was better, then only accept with boltzmann-related chance
				double r = d(gen); //rand[0,1]

				if (r < exp(-(new_F - current_F) / T)) { //Boltzmann
					current_parameters = new_parameters;
					if (std::abs(current_F - new_F) > delta_F_) { //only set termination criteria c to zero if the change in F is significant
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

			//Save the overall best value (this has no influence on the SA Algorithm)
			if (new_F < best_F) {
				best_F = new_F;
				best_parameters = new_parameters;
			}


			//Cooling of T and eta
			T = l * T;
			eta = l * eta;

			k++;
		}
	}

	


	//___________________________________PRIVATE____________________________________________________________________________

	//This function assigns a cost value to a parameter set, by executing the hybrid control with the given parameter set
	//The cost value put into the csv_data struct and then this struct is returned
	csv_data hyb_con_pid_optimizer::evaluate_params(franka_proxy::franka_hardware_controller& h_controller, std::array<std::array<double, 6>, 6> control_parameters, csv_data& data) {

		//move to starting position
		try {
			h_controller.move_to(pos_5_b_);
			h_controller.move_to(pos_0_b_);
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


		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> desired_positions_all;
		std::vector<Eigen::Vector3d> desired_positions;
		std::vector<Eigen::Matrix<double, 6, 1>> desired_forces;
		std::vector<Eigen::Quaterniond> desired_orientations;

		desired_positions_all = give_circle_positions(start_pos);

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

		//try {
		//	h_controller.move_to(pos_air_x_b);
		//}
		//catch (const franka::Exception& e) {
		//	std::cout << "catched Exception when moving to air position: " << e.what() << std::endl;
		//}

		data.ise_position = calculate_ISE(data.position_error);
		data.ise_force = calculate_ISE(data.force_error);
		data.itae_position = calculate_ITAE(data.position_error);
		data.itae_force = calculate_ITAE(data.force_error);

		if (data_tracking_) {
			csv_parser_.parsing(data);
		}		

		return data;
	}



	//change the current Simulated Annealing parameter
	void hyb_con_pid_optimizer::set_sa_params(double T, double l, double eta) {
		hyb_con_pid_optimizer::T_start_ = T;
		hyb_con_pid_optimizer::l_start_ = l;
		hyb_con_pid_optimizer::eta_start_ = eta;
	}


	//Auxiliary function
	std::array<std::array<double, 6>, 6> hyb_con_pid_optimizer::format_control_parameters(std::array<Eigen::Vector3d, 12> eigen_parameters) {
		std::array<std::array<double, 6>, 6> array_parameters;
		array_parameters[0] = { eigen_parameters[0](0), eigen_parameters[1](0), eigen_parameters[2](0), eigen_parameters[3](0), eigen_parameters[4](0), eigen_parameters[5](0) };
		array_parameters[1] = { eigen_parameters[0](1), eigen_parameters[1](1), eigen_parameters[2](1), eigen_parameters[3](1), eigen_parameters[4](1), eigen_parameters[5](1) };
		array_parameters[2] = { eigen_parameters[0](2), eigen_parameters[1](2), eigen_parameters[2](2), eigen_parameters[3](2), eigen_parameters[4](2), eigen_parameters[5](2) };
		array_parameters[3] = { eigen_parameters[6](0), eigen_parameters[7](0), eigen_parameters[8](0), eigen_parameters[9](0), eigen_parameters[10](0), eigen_parameters[11](0) };
		array_parameters[4] = { eigen_parameters[6](1), eigen_parameters[7](1), eigen_parameters[8](1), eigen_parameters[9](1), eigen_parameters[10](1), eigen_parameters[11](1) };
		array_parameters[5] = { eigen_parameters[6](2), eigen_parameters[7](2), eigen_parameters[8](2), eigen_parameters[9](2), eigen_parameters[10](2), eigen_parameters[11](2) };

		return array_parameters;
	}

	//Auxiliary function
	double hyb_con_pid_optimizer::calculate_F(std::array<int,12> dim, csv_data& data) {
		double F = 0.0;
		double p_factor = 100.0; //increase the impact of the position dimension errors on the final cost value F
		double phi_factor = 10.0; //increase the impact of the position rotation errors on the final cost value F
		double f_factor = 1.0;

		for (int d = 0; d < 12; d++) {
			if (dim[d] != 1) continue;
			if (d <= 2) {
				F += p_factor * data.itae_position(d, 0);
			}
			else if (d > 2 && d < 6) {
				F += phi_factor * data.itae_position(d, 0);
			}
			else {
				F += f_factor * data.itae_force(d - 6, 0);
			}
		}

		return F;
	}

	//calculate integral of the squared error: Force in [N^2 s] or Position in [m^2 s]
	Eigen::Matrix<double, 6, 1> hyb_con_pid_optimizer::calculate_ISE(std::vector<Eigen::Matrix<double, 6, 1>>& values) {
		Eigen::Matrix<double, 6, 1> ise;
		ise.setZero();
		for (int n = 0; n < values.size(); n++) {
			ise += ((values[n].array() * values[n].array()) * 0.001).matrix(); //Integral(e(t)*e(t) dt)
		}
		return ise;
	}

	//calculate integral of the time multiplied absolute error: Force in [N s^2] or Position in [m s^2]
	Eigen::Matrix<double, 6, 1> hyb_con_pid_optimizer::calculate_ITAE(std::vector<Eigen::Matrix<double, 6, 1>>& values) {
		Eigen::Matrix<double, 6, 1> itae;
		itae.setZero();
		for (int n = 0; n < values.size(); n++) {
			itae += ((n / 1000.0) * values[n].array().abs() * 0.001).matrix();  //Integral(t*|e(t)| dt)
		}
		return itae;
	}
}





