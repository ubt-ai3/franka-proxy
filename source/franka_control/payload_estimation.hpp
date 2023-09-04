#if !defined(INCLUDED__FRANKA_CONTROL__PAYLOAD_ESTIMATION)
#define INCLUDED__FRANKA_CONTROL__PAYLOAD_ESTIMATION

#include <Eigen/Geometry>
#include <Eigen/EulerAngles>
#include <vector>
#include <array>

Eigen::Matrix<double, 3, 1> gravity(0.0, 0.0, -9.81); //global gravity vector

struct inter {
	Eigen::Matrix<double, 3, 1> v; //angular velocity
	Eigen::Matrix<double, 3, 1> a; //angular acceleration
	Eigen::Matrix<double, 3, 1> l; //linear acceleration
	Eigen::Matrix<double, 3, 1> g; //gravity

	Eigen::EulerAnglesXYZd angles;
};


struct results {
	double mass;
	Eigen::Matrix<double, 3, 1> com;
	Eigen::Matrix<double, 3, 3> inertia;
};


typedef std::vector<std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, int>> data;


void preprocess(inter& store, std::array<double, 7>& q, Eigen::EulerAnglesXYZd& old_ang, Eigen::Matrix<double, 3, 1>& old_v, double seconds);

void estimate_ceres(data& input, results& res);

void estimate_tls(data& input, results& res);

#endif /* !defined(INCLUDED__FRANKA_CONTROL__FRANKA_CONTROLLER_HPP) */