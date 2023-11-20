/**
 *************************************************************************
 *
 * @file payload_estimation.hpp
 *
 * Utility class for estimating the inertial parameters of a payload.
 * 
 * The approach implemented here follows the one described in
 * "On-Line Estimation of Inertial Parameters Using a Recursive Total
 * Least-Squares Approach", Kubus et al., 2008,
 * https://doi.org/10.1109/IROS.2008.4650672
 *
 ************************************************************************/
#if !defined(INCLUDED__FRANKA_CONTROL__PAYLOAD_ESTIMATION)
#define INCLUDED__FRANKA_CONTROL__PAYLOAD_ESTIMATION

#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>
#include <vector>
#include <array>

#include "franka_util.hpp"


namespace payload_estimation
{

	static Eigen::Matrix<double, 3, 1> gravity(0.0, 0.0, -9.81); //global gravity vector

	//some intital values for payload estimation, will be set whenever running any estimation function
	static Eigen::EulerAnglesXYZd ang_init; //euler angles for starting position
	static Eigen::Matrix<double, 3, 1> g_init; //gravity vector for starting position
	static Eigen::Matrix<double, 11, 6> M_g; //matrix built from g_init for TLS
	static std::array<double, 6> ft_init; //initial sensor values
	static double t_init; //starting time

	//internal data interchange format
	struct inter {
		Eigen::Matrix<double, 3, 1> v = Eigen::Matrix<double, 3, 1>(); //angular velocity
		Eigen::Matrix<double, 3, 1> a = Eigen::Matrix<double, 3, 1>(); //angular acceleration
		Eigen::Matrix<double, 3, 1> l = Eigen::Matrix<double, 3, 1>(); //linear acceleration
		Eigen::Matrix<double, 3, 1> g = Eigen::Matrix<double, 3, 1>(); //gravity

		Eigen::EulerAnglesXYZd angles;
	};

	//output data format
	struct results {
		double mass = 0.0;
		Eigen::Matrix<double, 3, 1> com = Eigen::Matrix<double, 3, 1>();
		Eigen::Matrix<double, 3, 3> inertia = Eigen::Matrix<double, 3, 3>();
	};

	//input data format
	typedef std::vector<std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, double>> data;


	/*************************************************************
	* 
	* @class ple
	* 
	* Provides different payload estimation methods:
	* - Total Least Squares
	* - Usage of the Ceres solver
	* 
	*************************************************************/
	class ple
	{
	private:
		static void init(std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, double> sample);

		static inter preprocess(std::array<double, 7> &q, Eigen::EulerAnglesXYZd &old_ang, Eigen::Matrix<double, 3, 1> &old_v, double seconds);
		
		static results compute_tls_solution(Eigen::MatrixXd &S, Eigen::MatrixXd &U);

		static Eigen::Affine3d fk(Eigen::Matrix<double, 7, 1> &config);

		static franka_control::franka_util util;

		
	public: 
		static results estimate_ceres(data &input);

		static results estimate_tls(data &input, bool fast, int step);

		static data read_from_csv(std::string &filename);

	};

} /* namespace payload_estimation */

#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP) */