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

namespace payload_estimation
{

	static Eigen::Matrix<double, 3, 1> gravity(0.0, 0.0, -9.81); //global gravity vector

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
	typedef std::vector<std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, int>> data;


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
		static void preprocess(inter &store, std::array<double, 7> &q, Eigen::EulerAnglesXYZd &old_ang, Eigen::Matrix<double, 3, 1> &old_v, double seconds);
		
		static void compute_tls_solution(results &res, Eigen::MatrixXd &S, Eigen::MatrixXd &U);

		
	public: 
		static void estimate_ceres(data &input, results &res);

		static void estimate_tls(data &input, results &res);
	};

} /* namespace payload_estimation */

#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP) */