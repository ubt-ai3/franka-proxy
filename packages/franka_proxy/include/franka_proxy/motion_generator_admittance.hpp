/**
 *************************************************************************
 *
 * @file motion_generator_admittance.hpp
 *
 * todo
 *
 ************************************************************************/

#pragma once


#include <vector>
#include <optional>

#include <Eigen/Core>

#include <franka/robot.h>
#include <franka/model.h>

#include <franka_proxy_share/franka_proxy_logger.hpp>

#include "motion_generator_cartesian_impedance.hpp"


namespace franka_proxy
{
	namespace detail
	{


		/**
		 *************************************************************************
		 *
		 * @class admittance_motion_generator
		 *
		 * in use
		 * 
		 * Cartesian admittance controller. This controller is using the cartesian
		 * impedance controller of the motion_generator_cartesian_impedance class
		 * to reach the calculated xi positions (xi = position output of the
		 * admittance controller). The required position error calculation callback
		 * of the impedance controller is handled within this class and differs from
		 * the 'default' one presented in the motion_generator_cartesian_impedance
		 * class.
		 *
		 ************************************************************************/
		class admittance_motion_generator
		{
		public:

			admittance_motion_generator
			(franka::Robot& robot,
				std::mutex& state_lock,
				franka::RobotState& robot_state,
				double duration,
				std::optional<std::string> log_file_path);

			franka::Torques callback
			(const franka::RobotState& robot_state,
				franka::Duration period);

			// getter and setter for 'default' stiffness and damping parameters
			void set_admittance_rotational_stiffness(double rotational_stiffness);
			void set_admittance_translational_stiffness(double translational_stiffness);

			void set_impedance_rotational_stiffness(double rotational_stiffness);
			void set_impedance_translational_stiffness(double translational_stiffness);

			double get_admittance_rotational_stiffness();
			double get_admittance_translational_stiffness();

			double get_impedance_rotational_stiffness();
			double get_impedance_translational_stiffness();

		private:
			void calculate_stiffness_and_damping();

			//std::vector<Eigen::Affine3d> fk(const Eigen::Matrix<double, 7, 1>& configuration);

			static constexpr double pi = 3.14159265358979323846;
			static constexpr double deg_to_rad = pi / 180.;
			static constexpr double rad_to_deg = 180. / pi;

			franka::Model model_;

			std::mutex& state_lock_;
			franka::RobotState& state_;

			double duration_;
			double time_ = 0.0;

			std::list<std::array<double, 6>> f_exts_;

			bool initialized_ = false;
			Eigen::Matrix<double, 6, 1> x_i_1_;
			Eigen::Matrix<double, 6, 1> x_i_2_;

			Eigen::Quaterniond previous_quaternion_;

			// damping and stiffness matrix
			double translational_stiffness_ = 150.0;
			double rotational_stiffness_ = 10.0;
			Eigen::Matrix<double, 6, 6> damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 6> stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();

			// impedance controller to command new desired position
			cartesian_impedance_motion_generator impedance_controller_;

			// csv logging
			bool logging_;
			std::string file_ = "admittance_addtional_log.csv";
			logger logger_;
			std::vector<std::string> j_head = {"q0", "q1", "q2", "q3", "q4", "q5"};
			std::vector<std::string> c_head = { "x_i_0", "x_i_1", "x_i_2", "x_i_3", "x_i_4", "x_i_5",
				"pos_eq_0", "pos_eq_1", "pos_eq_2", "pos_eq_3", "pos_eq_4", "pos_eq_5",
				"x_i_prod2_0", "x_i_prod2_1", "x_i_prod2_2", "x_i_prod2_3", "x_i_prod2_4", "x_i_prod2_5"};
			std::vector<std::string> f_head = { "f_ext_0", "f_ext_1", "f_ext_2", "f_ext_3", "f_ext_4", "f_ext_5",
				"cur_f_0", "cur_f_1", "cur_f_2", "cur_f_3", "cur_f_4", "cur_f_5", 
				"f_ext_m_0", "f_ext_m_1", "f_ext_m_2", "f_ext_m_3", "f_ext_m_4", "f_ext_m_5"};
			std::vector<std::string> s_head = { "time" };
			std::vector<std::string> a_head = { "jac_0_0", "jac_0_1", "jac_0_2", "jac_0_3", "jac_0_4", "jac_0_5", "jac_0_6",
				"jac_1_0", "jac_1_1", "jac_1_2", "jac_1_3", "jac_1_4", "jac_1_5", "jac_1_6",
				"jac_2_0", "jac_2_1", "jac_2_2", "jac_2_3", "jac_2_4", "jac_2_5", "jac_2_6",
				"jac_3_0", "jac_3_1", "jac_3_2", "jac_3_3", "jac_3_4", "jac_3_5", "jac_3_6",
				"jac_4_0", "jac_4_1", "jac_4_2", "jac_4_3", "jac_4_4", "jac_4_5", "jac_4_6"
				"jac_5_0", "jac_5_1", "jac_5_2", "jac_5_3", "jac_5_4", "jac_5_5", "jac_5_6" };

			std::list<double> timestamps_;
		};




	} /* namespace detail */
} /* namespace franka_proxy */
