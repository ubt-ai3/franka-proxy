#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "payload_estimation.hpp"

#include <cmath>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <ceres/ceres.h>

namespace payload_estimation
{
	/***************************************************
	* Utility function for reading data from a CSV file,
	* expects the header as definded in
	* "motion_generator_joint_imp_ple.hpp"
	***************************************************/

	data ple::read_from_csv(std::string &filename) {

		std::ifstream input(filename);

		if (!input.is_open()) { throw std::runtime_error("Could not open CSV file: " + filename); }

		data content;
		std::string line;
		std::string val;

		std::array<double, 7> q;
		std::array<double, 6> ft;
		double time;

		//throw away the header line
		std::getline(input, line);

		while (std::getline(input, line)) {
			std::stringstream stream(line);
			int count = 0;
			while (std::getline(stream, val, ',')) {
				if (count < 7){ 
					q[count] = std::stod(val);
					count++;
				}
				else if (count < 13) {
					ft[(count - 7)] = std::stod(val);
					count++;
				}
				else {
					time = std::stod(val);
					break;
				}
			}
			std::pair<std::array<double, 7>, std::array<double, 6>> measurements(q, ft);
			std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, double> entry(measurements, time);
			content.push_back(entry);
		}

		return content;
	}


	/**********************************
	* Utility function to calculate the
	* forward transformation into the
	* sensor frame
	**********************************/
	Eigen::Affine3d ple::fk(Eigen::Matrix<double, 7, 1> &config) {
		
		//fk until last joint
		Eigen::Affine3d trafo = util.fk(config).back();
		//last joint to flange (c.f. "franka_util.hpp")
		trafo *= Eigen::Translation3d(0.0, 0.0, 0.107);
		//flange to fts (offset and rotation)
		trafo *= Eigen::Translation3d(0.0, 0.0, 0.055);
		trafo *= Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d(0.0, 0.0, 1.0));

		return trafo;
	}


	/************************************************
	* Initialization for the starting position of the
	* measured robot motion, needs to be run before
	* any estimations can be made and will be run
	* automatically by any estimation method, using
	* the first sample of data, if necessary
	************************************************/

	void ple::init(std::pair<std::pair<std::array<double, 7>, std::array<double, 6>>, double> sample) {
		
		ft_init = sample.first.second;
		t_init = sample.second;

		Eigen::Matrix<double, 7, 1> config(sample.first.first.data());
		Eigen::Affine3d trafo = fk(config);
		Eigen::Matrix<double, 3, 3> M = trafo.rotation();

		Eigen::EulerAnglesXYZd euler(M);
		ang_init = euler;

		g_init = M * gravity;
		M_g << g_init(0), g_init(1), g_init(2), 0, 0, 0,
				0, 0, 0, 0, -g_init(2), g_init(1),
				0, 0, 0, g_init(2), 0, -g_init(0),
				0, 0, 0, -g_init(1), g_init(0), 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0;

		is_init = true;
	}


	/*****************************************
	* Common preprocessing for all PLE methods
	*****************************************/

	inter ple::preprocess(std::array<double, 7> &q, Eigen::EulerAnglesXYZd &old_ang, Eigen::Matrix<double, 3, 1> &old_v, double seconds) {
		
		Eigen::Matrix<double, 7, 1> config(q.data());
		Eigen::Affine3d trafo = fk(config);
		Eigen::Matrix<double, 3, 3> M = trafo.rotation();

		Eigen::Matrix<double, 3, 1> grav = M * gravity;

		Eigen::EulerAnglesXYZd euler(M);

		double dz = euler.alpha() - old_ang.alpha();
		if (dz > M_PI) { dz = dz - (2 * M_PI); }
		else if (dz < -M_PI) { dz = dz + (2 * M_PI); }
		double dy = euler.beta() - old_ang.beta();
		if (dy > M_PI) { dy = dy - (2 * M_PI); }
		else if (dy < -M_PI) { dy = dy + (2 * M_PI); }
		double dx = euler.gamma() - old_ang.gamma();
		if (dx > M_PI) { dx = dx - (2 * M_PI); }
		else if (dx < -M_PI) { dx = dx + (2 * M_PI); }

		double vx = (dx * std::sin(euler.beta()) * std::sin(euler.alpha()) + dy * std::cos(euler.alpha())) / seconds;
		double vy = (dx * std::sin(euler.beta()) * std::cos(euler.alpha()) - dy * std::sin(euler.alpha())) / seconds;
		double vz = (dx * std::cos(euler.beta()) + dz) / seconds;

		Eigen::Matrix<double, 3, 1> velo(vx, vy, vz);

		Eigen::Matrix<double, 3, 1> acc = (velo - old_v) / seconds;

		Eigen::Matrix<double, 3, 1> x(1.0, 0.0, 0.0);
		Eigen::Matrix<double, 3, 1> y(0.0, 1.0, 0.0);
		Eigen::Matrix<double, 3, 1> z(0.0, 0.0, 1.0);

		Eigen::Matrix<double, 3, 1> lin = velo.cross(x) + velo.cross(y) + velo.cross(z);

		inter store;

		store.v = velo;
		store.a = acc;
		store.l = lin;
		store.g = grav;
		store.angles = euler;

		return store;
	}


	/*********************************************
	* Utility function that outputs preprocessed
	* data into a .csv file for use in ML training
	*********************************************/

	void ple::generate_training_data(data &input) {
		//initial setup
		auto sample = input[0];
		init(sample);
		Eigen::EulerAnglesXYZd old_ang = ang_init;
		Eigen::Matrix<double, 3, 1> old_v(0.0, 0.0, 0.0);
		double time = t_init;
		std::array<double, 7> q;
		std::array<double, 6> ft;
		inter store;

		std::string outfile = "ple_training.csv";
		std::string header = "ang_v_x,ang_v_y,ang_v_z,ang_a_x,ang_a_y,ang_a_z,lin_a_x,lin_a_y,lin_a_z,grav_x,grav_y,grav_z,f_x,f_y,f_z,t_x,t_y,t_z,time";
		std::ofstream logger(outfile, std::ofstream::out);
		logger << header << "\n";

		for (int i = 1; i < input.size(); i++) {
			double next = input[i].second;
			double dt = next - time;
			if (dt <= 0.0) { continue; }

			q = input[i].first.first;
			ft = input[i].first.second;
			store = preprocess(q, old_ang, old_v, dt);
			old_ang = store.angles;
			old_v = store.v;
			time = next;

			logger << store.v(0) << "," << store.v(1) << "," << store.v(2) << "," 
				<< store.a(0) << "," << store.a(1) << "," << store.a(2) << "," 
				<< store.l(0) << "," << store.l(1) << "," << store.l(2) << "," 
				<< store.g(0) << "," << store.g(1) << "," << store.g(2) << "," 
				<< ft[0] << "," << ft[1] << "," << ft[2] << "," << ft[3] 
				<< "," << ft[4] << "," << ft[5] << "," << time << "\n";
		}
	}


	/*******************************
	* Payload estimation using Ceres
	*******************************/

	struct FX {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double fx;
		FX(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double fx)
			: v(v), a(a), l(l), g(g), fx(fx) {};
		template <typename T>
		bool operator()(const T* const m, const T* const cx, const T* const cy, const T* const cz, T* residual) const {
			//without zeroing
			//residual[0] = (l(0) - g(0)) * m[0] + (-pow(v(1), 2) - pow(v(2), 2)) * cx[0] + ((v(0) * v(1)) - a(2)) * cy[0] + ((v(0) * v(1)) + a(1)) * cz[0] - fx;

			residual[0] = (l(0) - g(0) + g_init(0)) * m[0] + (-pow(v(1), 2) - pow(v(2), 2)) * cx[0] + ((v(0) * v(1)) - a(2)) * cy[0] + ((v(0) * v(1)) + a(1)) * cz[0] - (fx - ft_init[0]);
			return true;
		}
	};

	struct FY {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double fy;
		FY(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double fy)
			: v(v), a(a), l(l), g(g), fy(fy) {};
		template <typename T>
		bool operator()(const T* const m, const T* const cx, const T* const cy, const T* const cz, T* residual) const {
			//without zeroing
			//residual[0] = (l(1) - g(1)) * m[0] + ((v(0) * v(1)) + a(2)) * cx[0] + (-pow(v(0), 2) - pow(v(2), 2)) * cy[0] + ((v(1) * v(2)) - a(0)) * cz[0] - fy;

			residual[0] = (l(1) - g(1) + g_init(1)) * m[0] + ((v(0) * v(1)) + a(2)) * cx[0] + (-pow(v(0), 2) - pow(v(2), 2)) * cy[0] + ((v(1) * v(2)) - a(0)) * cz[0] - (fy - ft_init[1]);
			return true;
		}
	};

	struct FZ {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double fz;
		FZ(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double fz)
			: v(v), a(a), l(l), g(g), fz(fz) {};
		template <typename T>
		bool operator()(const T* const m, const T* const cx, const T* const cy, const T* const cz, T* residual) const {
			//without zeroing
			//residual[0] = (l(2) - g(2)) * m[0] + ((v(0) * v(2)) - a(1)) * cx[0] + ((v(1) * v(2)) + a(0)) * cy[0] + (-pow(v(1), 2) - pow(v(0), 2)) * cz[0] - fz;

			residual[0] = (l(2) - g(2) + g_init(2)) * m[0] + ((v(0) * v(2)) - a(1)) * cx[0] + ((v(1) * v(2)) + a(0)) * cy[0] + (-pow(v(1), 2) - pow(v(0), 2)) * cz[0] - (fz - ft_init[2]);
			return true;
		}
	};

	struct TX {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double tx;
		TX(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double tx)
			: v(v), a(a), l(l), g(g), tx(tx) {};
		template <typename T>
		bool operator()(const T* const cy, const T* const cz,
			const T* const ixx, const T* const ixy, const T* const ixz, const T*
			const iyy, const T* const iyz, const T* const izz, T* residual) const {
			//without zeroing
			//residual[0] = (l(2) - g(2)) * cy[0] + (g(1) - l(1)) * cz[0] + (a(0)) * ixx[0] + (a(1) - (v(0) * v(2))) * ixy[0] + (a(2) + (v(0) * v(1))) * ixz[0] + (-(v(1) * v(2))) * iyy[0] + (pow(v(1), 2) - pow(v(2), 2)) * iyz[0] + (v(1) * v(2)) * izz[0] - tx;
			
			residual[0] = (l(2) - g(2) + g_init(2)) * cy[0] + (g(1) - l(1) - g_init(1)) * cz[0] + (a(0)) * ixx[0] + (a(1) - (v(0) * v(2))) * ixy[0] + (a(2) + (v(0) * v(1))) * ixz[0] + (-(v(1) * v(2))) * iyy[0] + (pow(v(1), 2) - pow(v(2), 2)) * iyz[0] + (v(1) * v(2)) * izz[0] - (tx - ft_init[3]);
			return true;
		}
	};

	struct TY {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double ty;
		TY(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double ty)
			: v(v), a(a), l(l), g(g), ty(ty) {};
		template <typename T>
		bool operator()(const T* const cx, const T* const cz,
			const T* const ixx, const T* const ixy, const T* const ixz, const T*
			const iyy, const T* const iyz, const T* const izz, T* residual) const {
			//without zeroing
#			//residual[0] = (g(2) - l(2)) * cx[0] + (l(0) - g(0)) * cz[0] + (v(0) * v(2)) * ixx[0] + (a(0) + (v(1) * v(2))) * ixy[0] + (pow(v(2), 2) - pow(v(0), 2)) * ixz[0] + (a(1)) * iyy[0] + (a(2) - (v(0) * v(1))) * iyz[0] + (-(v(0) * v(2))) * izz[0] - ty;

			residual[0] = (g(2) - l(2) - g_init(2)) * cx[0] + (l(0) - g(0) + g_init(0)) * cz[0] + (v(0) * v(2)) * ixx[0] + (a(0) + (v(1) * v(2))) * ixy[0] + (pow(v(2), 2) - pow(v(0), 2)) * ixz[0] + (a(1)) * iyy[0] + (a(2) - (v(0) * v(1))) * iyz[0] + (-(v(0) * v(2))) * izz[0] - (ty - ft_init[4]);
			return true;
		}
	};

	struct TZ {
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> a;
		Eigen::Matrix<double, 3, 1> l;
		Eigen::Matrix<double, 3, 1> g;
		double tz;
		TZ(Eigen::Matrix<double, 3, 1> v, Eigen::Matrix<double, 3, 1> a,
			Eigen::Matrix<double, 3, 1> l, Eigen::Matrix<double, 3, 1> g, double tz)
			: v(v), a(a), l(l), g(g), tz(tz) {};
		template <typename T>
		bool operator()(const T* const cx, const T* const cy,
			const T* const ixx, const T* const ixy, const T* const ixz, const T*
			const iyy, const T* const iyz, const T* const izz, T* residual) const {
			//without zeroing
			//residual[0] = (l(1) - g(1)) * cx[0] + (g(0) - l(0)) * cy[0] + (-(v(0) * v(1))) * ixx[0] + (pow(v(0), 2) - pow(v(1), 2)) * ixy[0] + (a(0) - (v(1) * v(2))) * ixz[0] + (v(0) * v(1)) * iyy[0] + (a(1) + (v(0) * v(2))) * iyz[0] + (a(2)) * izz[0] - tz;

			residual[0] = (l(1) - g(1) + g_init(1)) * cx[0] + (g(0) - l(0) - g_init(0)) * cy[0] + (-(v(0) * v(1))) * ixx[0] + (pow(v(0), 2) - pow(v(1), 2)) * ixy[0] + (a(0) - (v(1) * v(2))) * ixz[0] + (v(0) * v(1)) * iyy[0] + (a(1) + (v(0) * v(2))) * iyz[0] + (a(2)) * izz[0] - (tz - ft_init[5]);
			return true;
		}
	};

	results ple::estimate_ceres(data &input) {
		//initial setup
		if (!is_init) {
			auto sample = input[0];
			init(sample);
		}
		Eigen::EulerAnglesXYZd old_ang = ang_init;
		Eigen::Matrix<double, 3, 1> old_v(0.0, 0.0, 0.0);
		double time = t_init;
		std::array<double, 7> q;
		std::array<double, 6> ft;
		inter store;

		//defining the problem
		ceres::Problem problem;
		double m = 0.0;
		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		double ixx = 0.0;
		double ixy = 0.0;
		double ixz = 0.0;
		double iyy = 0.0;
		double iyz = 0.0;
		double izz = 0.0;

		for (int i = 1; i < input.size(); i++) {
			double next = input[i].second;
			double dt = next - time;
			if (dt <= 0.0) { continue; }

			q = input[i].first.first;
			ft = input[i].first.second;
			store= preprocess(q, old_ang, old_v, dt);
			old_ang = store.angles;
			old_v = store.v;
			time = next;

			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FX, 1, 1, 1, 1, 1>(new FX(store.v, store.a, store.l, store.g, ft[0])), nullptr, &m, &cx, &cy, &cz);
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FY, 1, 1, 1, 1, 1>(new FY(store.v, store.a, store.l, store.g, ft[1])), nullptr, &m, &cx, &cy, &cz);
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FZ, 1, 1, 1, 1, 1>(new FZ(store.v, store.a, store.l, store.g, ft[2])), nullptr, &m, &cx, &cy, &cz);
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<TX, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new TX(store.v, store.a, store.l, store.g, ft[3])), nullptr, &cy, &cz, &ixx, &ixy, &ixz, &iyy, &iyz, &izz);
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<TY, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new TY(store.v, store.a, store.l, store.g, ft[4])), nullptr, &cx, &cz, &ixx, &ixy, &ixz, &iyy, &iyz, &izz);
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<TZ, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new TZ(store.v, store.a, store.l, store.g, ft[5])), nullptr, &cx, &cy, &ixx, &ixy, &ixz, &iyy, &iyz, &izz);
		}

		//solving the problem
		ceres::Solver::Options options;
		options.max_num_iterations = 1000;

		ceres::Solver::Summary summary;

		ceres::Solve(options, &problem, &summary);

		results res;
		res.mass = m;
		res.com = Eigen::Matrix<double, 3, 1>(cx, cy, cz);
		res.inertia = Eigen::Matrix<double, 3, 3>();
		res.inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

		return res;
	}


	/*****************************
	* Payload estimation using TLS
	*****************************/

	results ple::compute_tls_solution(Eigen::MatrixXd &S, Eigen::MatrixXd &U) {
		//check for problems with the TLS solution
		if (U(U.rows() - 1, U.cols() - 1) == 0) { throw std::runtime_error("No TLS solution exists for this data"); }
		if (S(S.rows() - 1, S.cols() - 1) == S(S.rows() - 2, S.cols() - 2)) { std::cerr << "WARNING: TLS solution may be inaccurate or incorrect" << std::endl; }

		//obtain the actual solution
		std::array<double, 10> sol{0.0};
		for (int i = 0; i < sol.size(); i++) {
			sol[i] = U(i, i) / U(U.rows() - 1, U.cols() - 1);
		}
		results res;
		res.mass = sol[0];
		res.com << sol[1], sol[2], sol[3];
		res.inertia << sol[4], sol[5], sol[6],
					sol[5], sol[7], sol[8],
					sol[6], sol[8], sol[9];
		return res;
	}

	results ple::estimate_tls(data &input, bool fast, int step) {
		//initial setup
		if (step < 1) { step = 1;  }
		if (!is_init) {
			auto sample = input[0];
			init(sample);
		}

		//initial SVD
		std::array<double, 6> ft0 = input[1].first.second;
		std::array<double, 7> q0 = input[1].first.first;
		std::array<double, 6> ft1;
		std::array<double, 7> q1;
		double t0 = input[1].second - t_init;
		inter i0;
		inter i1;
		Eigen::EulerAnglesXYZd old_ang = ang_init;
		Eigen::Matrix<double, 3, 1> old_v(0.0, 0.0, 0.0);
		i0 = preprocess(q0, old_ang, old_v, t0);
		old_ang = i0.angles;
		old_v = i0.v;

		int entry = 0;
		for (int i = 2; i < input.size(); i++) {
			double t = input[i].second;
			double dt = t - t0;
			
			if (dt > 0.0) {
				entry = i + 1;
				ft1 = input[i].first.second;
				q1 = input[i].first.first;
				t0 = t;
				i1 = preprocess(q1, old_ang, old_v, dt);
				old_ang = i1.angles;
				old_v = i1.v;
				break;
			}
		}

		Eigen::Matrix<double, 11, 12> Minit;
		Minit << (i0.l(0) - i0.g(0)), (i0.l(1) - i0.g(1)), (i0.l(2) - i0.g(2)), 0, 0, 0, (i1.l(0) - i1.g(0)), (i1.l(1) - i1.g(1)), (i1.l(2) - i1.g(2)), 0, 0, 0,
				(-pow(i0.v(1), 2) - pow(i0.v(2), 2)), ((i0.v(0) * i0.v(1)) + i0.a(2)), ((i0.v(0) * i0.v(2)) - i0.a(1)), 0, (i0.g(2) - i0.l(2)), (i0.l(1) - i0.g(1)), (-pow(i1.v(1), 2) - pow(i1.v(2), 2)), ((i1.v(0) * i1.v(1)) + i1.a(2)), ((i1.v(0) * i1.v(2)) - i1.a(1)), 0, (i1.g(2) - i1.l(2)), (i1.l(1) - i1.g(1)),
				((i0.v(0) - i0.v(1)) - i0.a(2)), (-pow(i0.v(0), 2) - pow(i0.v(2), 2)), ((i0.v(1) * i0.v(2)) + i0.a(0)), (i0.l(2) - i0.g(2)), 0, (i0.g(0) - i0.l(0)), ((i1.v(0) - i1.v(1)) - i1.a(2)), (-pow(i1.v(0), 2) - pow(i1.v(2), 2)), ((i1.v(1) * i1.v(2)) + i1.a(0)), (i1.l(2) - i1.g(2)), 0, (i1.g(0) - i1.l(0)),
				((i0.v(0) * i0.v(2)) + i0.a(1)), ((i0.v(1) * i0.v(2)) - i0.a(0)), (-pow(i0.v(1), 2) - pow(i0.v(0), 2)), (i0.g(1) - i0.l(1)), (i0.l(0) - i0.g(0)), 0, ((i1.v(0) * i1.v(2)) + i1.a(1)), ((i1.v(1) * i1.v(2)) - i1.a(0)), (-pow(i1.v(1), 2) - pow(i1.v(0), 2)), (i1.g(1) - i1.l(1)), (i1.l(0) - i1.g(0)), 0,
				0, 0, 0, (i0.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) * i1.v(1)), 0, 0, 0, (i1.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) * i1.v(1)),
				0, 0, 0, (i0.a(1) - (i0.v(0) * i0.v(2))), (i0.a(0) + (i0.v(1) * i0.v(2))), (pow(i0.v(0), 2) - pow(i0.v(1), 2)), 0, 0, 0, (i1.a(1) - (i1.v(0) * i1.v(2))), (i1.a(0) + (i1.v(1) * i1.v(2))), (pow(i1.v(0), 2) - pow(i1.v(1), 2)),
				0, 0, 0, (i0.a(2) + (i0.v(0) * i0.v(1))), (pow(i0.v(2), 2) - pow(i0.v(0), 2)), (i0.a(0) - (i0.v(1) * i0.v(2))), 0, 0, 0, (i1.a(2) + (i1.v(0) * i1.v(1))), (pow(i1.v(2), 2) - pow(i1.v(0), 2)), (i1.a(0) - (i1.v(1) * i1.v(2))),
				0, 0, 0, (-i0.v(1) * i0.v(2)), (i0.a(1)), (i0.v(0) * i0.v(1)), 0, 0, 0, (-i1.v(1) * i1.v(2)), (i1.a(1)), (i1.v(0) * i1.v(1)),
				0, 0, 0, (pow(i0.v(1), 2) - pow(i0.v(2), 2)), (i0.a(2) - (i0.v(0) * i0.v(1))), (i0.a(1) + (i0.v(0) * i0.v(2))), 0, 0, 0, (pow(i1.v(1), 2) - pow(i1.v(2), 2)), (i1.a(2) - (i1.v(0) * i1.v(1))), (i1.a(1) + (i1.v(0) * i1.v(2))),
				0, 0, 0, (i0.v(1) * i0.v(2)), (-i0.v(0) * i0.v(2)), (i0.a(2)), 0, 0, 0, (i1.v(1) * i1.v(2)), (-i1.v(0) * i1.v(2)), (i1.a(2)),
				(ft0[0] - ft_init[0]), (ft0[1] - ft_init[1]), (ft0[2] - ft_init[2]), (ft0[3] - ft_init[3]), (ft0[4] - ft_init[4]), (ft0[5] - ft_init[5]), (ft1[0] - ft_init[0]), (ft1[1] - ft_init[1]), (ft1[2] - ft_init[2]), (ft1[3] - ft_init[3]), (ft1[4] - ft_init[4]), (ft1[5] - ft_init[5]);
		
		
		Eigen::Matrix<double, 11, 12> Ginit;
		Ginit << M_g, M_g;
		Minit = Minit + Ginit;
		
		
		Eigen::MatrixXd S;
		Eigen::MatrixXd U;
		if (fast) {
			Eigen::BDCSVD<Eigen::MatrixXd> svd(Minit, Eigen::ComputeThinU);
			S = svd.singularValues().asDiagonal();
			U = svd.matrixU();
		}
		else {
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Minit, Eigen::ComputeThinU);
			S = svd.singularValues().asDiagonal();
			U = svd.matrixU();
		}

		//this is just in case - should not trigger with real input data
		if (entry >= input.size()) {
			return compute_tls_solution(S, U);
		}

		//update SVD, incorporating remaining data
		for (int i = entry; i < input.size(); i = i + step) {
			double t = input[i].second;
			double dt = t - t0;
			if (dt <= 0.0) { continue; }

			q0 = input[i].first.first;
			ft0 = input[i].first.second;
			i0 = preprocess(q0, old_ang, old_v, dt);
			Eigen::Matrix<double, 11, 6> N;
			N << (i0.l(0) - i0.g(0)), (i0.l(1) - i0.g(1)), (i0.l(2) - i0.g(2)), 0, 0, 0,
					(-pow(i0.v(1), 2) - pow(i0.v(2), 2)), ((i0.v(0) * i0.v(1)) + i0.a(2)), ((i0.v(0) * i0.v(2)) - i0.a(1)), 0, (i0.g(2) - i0.l(2)), (i0.l(1) - i0.g(1)),
					((i0.v(0) - i0.v(1)) - i0.a(2)), (-pow(i0.v(0), 2) - pow(i0.v(2), 2)), ((i0.v(1) * i0.v(2)) + i0.a(0)), (i0.l(2) - i0.g(2)), 0, (i0.g(0) - i0.l(0)),
					((i0.v(0) * i0.v(2)) + i0.a(1)), ((i0.v(1) * i0.v(2)) - i0.a(0)), (-pow(i0.v(1), 2) - pow(i0.v(0), 2)), (i0.g(1) - i0.l(1)), (i0.l(0) - i0.g(0)), 0,
					0, 0, 0, (i0.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) * i1.v(1)),
					0, 0, 0, (i0.a(1) - (i0.v(0) * i0.v(2))), (i0.a(0) + (i0.v(1) * i0.v(2))), (pow(i0.v(0), 2) - pow(i0.v(1), 2)),
					0, 0, 0, (i0.a(2) + (i0.v(0) * i0.v(1))), (pow(i0.v(2), 2) - pow(i0.v(0), 2)), (i0.a(0) - (i0.v(1) * i0.v(2))),
					0, 0, 0, (-i0.v(1) * i0.v(2)), (i0.a(1)), (i0.v(0) * i0.v(1)),
					0, 0, 0, (pow(i0.v(1), 2) - pow(i0.v(2), 2)), (i0.a(2) - (i0.v(0) * i0.v(1))), (i0.a(1) + (i0.v(0) * i0.v(2))),
					0, 0, 0, (i0.v(1) * i0.v(2)), (-i0.v(0) * i0.v(2)), (i0.a(2)),
					(ft0[0] - ft_init[0]), (ft0[1] - ft_init[1]), (ft0[2] - ft_init[2]), (ft0[3] - ft_init[3]), (ft0[4] - ft_init[4]), (ft0[5] - ft_init[5]);
			
			
			N = N + M_g;
			
			
			Eigen::MatrixXd L = U.transpose() * N;
			Eigen::MatrixXd M = N - (U * L);
			Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
			Eigen::MatrixXd J = qr.householderQ();
			Eigen::MatrixXd G = qr.matrixQR();
			Eigen::MatrixXd F = Eigen::MatrixXd::Zero(G.rows(), S.cols());
			Eigen::MatrixXd Z((L.rows() + G.rows()), (S.cols() + L.cols()));
			Z << S, L,
				F, G;

			Eigen::MatrixXd sz;
			Eigen::MatrixXd uz;
			if (fast) {
				Eigen::BDCSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU);
				sz = svd.singularValues().asDiagonal();
				uz = svd.matrixU();
			}
			else {
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU);
				sz = svd.singularValues().asDiagonal();
				uz = svd.matrixU();
			}

			Eigen::MatrixXd UJ(U.rows(), (U.cols() + J.cols()));
			UJ << U, J;
			Eigen::MatrixXd nu = UJ * uz;
			U = nu;
			S = sz;
		}

		return compute_tls_solution(S, U);
	}


} /* namespace payload_estimation */