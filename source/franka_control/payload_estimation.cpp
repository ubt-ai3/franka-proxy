#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif

#include "payload_estimation.hpp"

#include "ikfast.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <ceres/ceres.h>

namespace payload_estimation
{

	/*****************************************
	* Common preprocessing for all PLE methods
	*****************************************/

	void ple::preprocess(inter& store, std::array<double, 7>& q, Eigen::EulerAnglesXYZd& old_ang, Eigen::Matrix<double, 3, 1>& old_v, double seconds) {
		//targets for ikfast ComputerFk()
		double* trans = new double[3];
		double* rot = new double[9];

		double*joints = new double[7];

		ComputeFk(joints, trans, rot);

		Eigen::Matrix<double, 3, 3> M = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(rot);

		Eigen::Matrix<double, 3, 1> grav = M * gravity;

		Eigen::EulerAnglesXYZd euler(M);

		double dz = euler.alpha() - old_ang.alpha();
		double dy = euler.beta() - old_ang.beta();
		double dx = euler.gamma() - old_ang.gamma();

		double vx = (dx * std::sin(euler.beta()) * std::sin(euler.alpha()) + dy * std::cos(euler.alpha())) / seconds;
		double vy = (dx * std::sin(euler.beta()) * std::cos(euler.alpha()) - dy * std::sin(euler.alpha())) / seconds;
		double vz = (dx * std::cos(euler.beta()) + dz) / seconds;

		Eigen::Matrix<double, 3, 1> velo(vx, vy, vz);

		Eigen::Matrix<double, 3, 1> acc = (velo - old_v) / seconds;

		Eigen::Matrix<double, 3, 1> x(1.0, 0.0, 0.0);
		Eigen::Matrix<double, 3, 1> y(0.0, 1.0, 0.0);
		Eigen::Matrix<double, 3, 1> z(0.0, 0.0, 1.0);

		Eigen::Matrix<double, 3, 1> lin = velo.cross(x) + velo.cross(y) + velo.cross(z);

		store.v = velo;
		store.a = acc;
		store.l = lin;
		store.g = grav;
		store.angles = euler;
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
			residual[0] = (l(0) - g(0)) * m[0] + (-pow(v(1), 2) - pow(v(2), 2)) * cx[0] + ((v(0) * v(1)) - a(2)) * cy[0] + ((v(0) * v(1)) + a(1)) * cz[0] - fx;
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
			residual[0] = (l(1) - g(1)) * m[0] + ((v(0) * v(1)) + a(2)) * cx[0] + (-pow(v(0), 2) - pow(v(2), 2)) * cy[0] + ((v(1) * v(2)) - a(1)) * cz[0] - fy;
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
			residual[0] = (l(2) - g(2)) * m[0] + ((v(0) * v(2)) - a(1)) * cx[0] + ((v(1) * v(2)) + a(0)) * cy[0] + (-pow(v(1), 2) - pow(v(0), 2)) * cz[0] - fz;
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
			residual[0] = (l(2) - g(2)) * cy[0] + (g(1) - l(1)) * cz[0] + (a(0)) * ixx[0] + (a(1) - (v(0) * v(2))) * ixy[0] + (a(2) + (v(0) * v(1))) * ixz[0] + (-(v(1) * v(2))) * iyy[0] + (pow(v(1), 2) - pow(v(2), 2)) * iyz[0] + (v(1) * v(2)) * izz[0] - tx;
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
			residual[0] = (g(2) - l(2)) * cx[0] + (l(0) - g(0)) * cz[0] + (v(0) * v(2)) * ixx[0] + (a(0) + (v(1) * v(2))) * ixy[0] + (pow(v(2), 2) - pow(v(0), 2)) * ixz[0] + (a(1)) * iyy[0] + (a(2) - (v(0) * v(1))) * iyz[0] + (-(v(0) * v(2))) * izz[0] - ty;
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
			residual[0] = (l(1) - g(1)) * cx[0] + (g(0) - l(0)) * cy[0] + (-(v(0) * v(1))) * ixx[0] + (pow(v(0), 2) - pow(v(1), 2)) * ixy[0] + (a(0) - (v(1) * v(2))) * ixz[0] + (v(0) * v(1)) * iyy[0] + (a(1) + (v(0) * v(2))) * iyz[0] + (a(2)) * izz[0] - tz;
			return true;
		}
	};

	void ple::estimate_ceres(data& input, results& res) {
		//initial setup
		Eigen::EulerAnglesXYZd old_ang(0.0, 0.0, 0.0);
		Eigen::Matrix<double, 3, 1> old_v(0.0, 0.0, 0.0);
		int time = input[0].second;
		std::array<double, 7> q = input[0].first.first;
		std::array<double, 6> ft;
		inter store;
		preprocess(store, q, old_ang, old_v, 1.0);
		old_ang = store.angles;
		old_v = store.v;

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
			int next = input[i].second;
			int dt = next - time;
			if (dt <= 0) { continue; }

			q = input[i].first.first;
			ft = input[i].first.second;
			preprocess(store, q, old_ang, old_v, (dt * 0.001));
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
		options.max_num_iterations = 100; //rather low number for quick testing

		ceres::Solver::Summary summary;

		ceres::Solve(options, &problem, &summary);

		res.mass = m;
		res.com = Eigen::Matrix<double, 3, 1>(cx, cy, cz);
		res.inertia = Eigen::Matrix<double, 3, 3>();
		res.inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

	}


	/*****************************
	* Payload estimation using TLS
	*****************************/

	void ple::compute_tls_solution(results& res, Eigen::MatrixXd& S, Eigen::MatrixXd& U) {
		//check for problems with the TLS solution
		if (U(U.rows() - 1, U.cols() - 1) == 0) { throw std::runtime_error("No TLS solution exists for this data"); }
		if (S(S.rows() - 1, S.cols() - 1) == S(S.rows() - 2, S.cols() - 2)) { std::cerr << "WARNING: TLS solution may be inaccurate or incorrect"; }

		//obtain the actual solution
		std::array<double, 10> sol{};
		for (int i = 0; i < sol.size(); i++) {
			sol[i] = U(i, i) / U(U.rows() - 1, U.cols() - 1);
		}
		res.mass = sol[0];
		res.com << sol[1], sol[2], sol[3];
		res.inertia << sol[4], sol[5], sol[6],
					sol[5], sol[7], sol[8],
					sol[6], sol[8], sol[9];
	}

	void ple::estimate_tls(data& input, results& res) {
		//initial setup
		std::array<double, 6> ft0 = input[0].first.second;
		std::array<double, 7> q0 = input[0].first.first;
		std::array<double, 6> ft1;
		std::array<double, 7> q1;
		int t0 = input[0].second;
		inter i0;
		inter i1;
		Eigen::EulerAnglesXYZd old_ang(0.0, 0.0, 0.0);
		Eigen::Matrix<double, 3, 1> old_v(0.0, 0.0, 0.0);
		preprocess(i0, q0, old_ang, old_v, 1.0);
		old_ang = i0.angles;
		old_v = i0.v;

		//initial SVD
		int entry = 0;
		for (int i = 1; i < input.size(); i++) {
			int t = input[i].second;
			int dt = t - t0;
			if (dt > 0) {
				entry = i + 1;
				ft1 = input[i].first.second;
				q1 = input[i].first.first;
				t0 = t;
				preprocess(i1, q1, old_ang, old_v, dt * 0.001);
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
				0, 0, 0, (i0.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) + i1.v(1)), 0, 0, 0, (i1.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) + i1.v(1)),
				0, 0, 0, (i0.a(1) - (i0.v(0) * i0.v(2))), (i0.a(0) + (i0.v(1) * i0.v(2))), (pow(i0.v(0), 2) - pow(i0.v(1), 2)), 0, 0, 0, (i1.a(1) - (i1.v(0) * i1.v(2))), (i1.a(0) + (i1.v(1) * i1.v(2))), (pow(i1.v(0), 2) - pow(i1.v(1), 2)),
				0, 0, 0, (i0.a(2) + (i0.v(0) * i0.v(1))), (pow(i0.v(2), 2) - pow(i0.v(0), 2)), (i0.a(0) - (i0.v(1) * i0.v(2))), 0, 0, 0, (i1.a(2) + (i1.v(0) * i1.v(1))), (pow(i1.v(2), 2) - pow(i1.v(0), 2)), (i1.a(0) - (i1.v(1) * i1.v(2))),
				0, 0, 0, (-i0.v(1) * i0.v(2)), (i0.a(1)), (i0.v(0) * i0.v(1)), 0, 0, 0, (-i1.v(1) * i1.v(2)), (i1.a(1)), (i1.v(0) * i1.v(1)),
				0, 0, 0, (pow(i0.v(1), 2) - pow(i0.v(2), 2)), (i0.a(2) - (i0.v(0) * i0.v(1))), (i0.a(1) - (i0.v(0) * i0.v(2))), 0, 0, 0, (pow(i1.v(1), 2) - pow(i1.v(2), 2)), (i1.a(2) - (i1.v(0) * i1.v(1))), (i1.a(1) - (i1.v(0) * i1.v(2))),
				0, 0, 0, (i0.v(1) * i0.v(2)), (-i0.v(0) * i0.v(2)), (i0.a(2)), 0, 0, 0, (i1.v(1) * i1.v(2)), (-i1.v(0) * i1.v(2)), (i1.a(2)),
				ft0[0], ft0[1], ft0[2], ft0[3], ft0[4], ft0[5], ft1[0], ft1[1], ft1[2], ft1[3], ft1[4], ft1[5];

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Minit, Eigen::ComputeThinU);
		Eigen::MatrixXd S = svd.singularValues().asDiagonal();
		Eigen::MatrixXd U = svd.matrixU();

		//this is just in case - should not trigger with real input data
		if (entry >= input.size()) {
			compute_tls_solution(res, S, U);
			return;
		}

		//update SVD, incorporating remaining data
		for (int i = entry; i < input.size(); i++) {
			int t = input[i].second;
			int dt = t - t0;
			if (dt <= 0) { continue; }

			q0 = input[i].first.first;
			ft0 = input[i].first.second;
			preprocess(i0, q0, old_ang, old_v, dt * 0.001);
			Eigen::Matrix<double, 11, 6> N;
			N << (i0.l(0) - i0.g(0)), (i0.l(1) - i0.g(1)), (i0.l(2) - i0.g(2)), 0, 0, 0,
					(-pow(i0.v(1), 2) - pow(i0.v(2), 2)), ((i0.v(0) * i0.v(1)) + i0.a(2)), ((i0.v(0) * i0.v(2)) - i0.a(1)), 0, (i0.g(2) - i0.l(2)), (i0.l(1) - i0.g(1)),
					((i0.v(0) - i0.v(1)) - i0.a(2)), (-pow(i0.v(0), 2) - pow(i0.v(2), 2)), ((i0.v(1) * i0.v(2)) + i0.a(0)), (i0.l(2) - i0.g(2)), 0, (i0.g(0) - i0.l(0)),
					((i0.v(0) * i0.v(2)) + i0.a(1)), ((i0.v(1) * i0.v(2)) - i0.a(0)), (-pow(i0.v(1), 2) - pow(i0.v(0), 2)), (i0.g(1) - i0.l(1)), (i0.l(0) - i0.g(0)), 0,
					0, 0, 0, (i0.a(0)), (i1.v(0) * i1.v(2)), (-i1.v(0) + i1.v(1)),
					0, 0, 0, (i0.a(1) - (i0.v(0) * i0.v(2))), (i0.a(0) + (i0.v(1) * i0.v(2))), (pow(i0.v(0), 2) - pow(i0.v(1), 2)),
					0, 0, 0, (i0.a(2) + (i0.v(0) * i0.v(1))), (pow(i0.v(2), 2) - pow(i0.v(0), 2)), (i0.a(0) - (i0.v(1) * i0.v(2))),
					0, 0, 0, (-i0.v(1) * i0.v(2)), (i0.a(1)), (i0.v(0) * i0.v(1)),
					0, 0, 0, (pow(i0.v(1), 2) - pow(i0.v(2), 2)), (i0.a(2) - (i0.v(0) * i0.v(1))), (i0.a(1) - (i0.v(0) * i0.v(2))),
					0, 0, 0, (i0.v(1) * i0.v(2)), (-i0.v(0) * i0.v(2)), (i0.a(2)),
					ft0[0], ft0[1], ft0[2], ft0[3], ft0[4], ft0[5];

			Eigen::MatrixXd L = U.transpose() * N;
			Eigen::MatrixXd M = N - (U * L);
			Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
			Eigen::MatrixXd J = qr.householderQ();
			Eigen::MatrixXd G = qr.matrixQR();
			Eigen::MatrixXd F = Eigen::MatrixXd::Zero(G.rows(), S.cols());
			Eigen::MatrixXd Z((L.rows() + G.rows()), (S.cols() + L.cols()));
			Z << S, L,
				F, G;

			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU);
			Eigen::MatrixXd sz = svd.singularValues().asDiagonal();
			Eigen::MatrixXd uz = svd.matrixU();

			Eigen::MatrixXd nu = (U * J) * uz;
			U = nu;
			S = sz;
		}

		compute_tls_solution(res, S, U);
	}


} /* namespace payload_estimation */