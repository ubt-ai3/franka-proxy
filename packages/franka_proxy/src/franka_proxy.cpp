/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "franka_proxy.hpp"

#include <iostream>

#include <argparse/argparse.hpp>


namespace franka_proxy
{
	//////////////////////////////////////////////////////////////////////////
	//
	// franka_proxy
	//
	//////////////////////////////////////////////////////////////////////////


	franka_proxy::franka_proxy(
		const std::string& ip,
		const bool enforce_realtime)
		: controller_(ip, enforce_realtime),

		  control_server_(franka_control_port, controller_),
		  state_server_(franka_state_port, controller_)
	{
	}
} /* namespace franka_proxy */


int main(int argc, char* argv[])
{
	argparse::ArgumentParser program("franka_proxy");
	//todo: crashes quietly when ip doesn't match
	program.add_argument("--ip")
	       .help("specify ip for franka-proxy to fci "
		       "(otherwise uses default 192.168.1.1)");

	program.add_argument("--enforce-realtime")
	       .help("activates fci realtime mode for control-loops")
	       .flag();

	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cout << program;
		return -1;
	}

	std::string ip("192.168.1.1");
	if (program.is_used("--ip"))
		ip = program.get<std::string>("--ip");

	const bool enforce_realtime = program["--enforce-realtime"] == true;
	std::cout << "Starting franka-proxy with ip: " << ip << '\n';

	if (enforce_realtime)
		std::cout << "Enabled realtime control loops. "
			"(This will only work on a realtime linux system or on Windows with administrator privileges)" << '\n';

	try
	{
		franka_proxy::franka_proxy proxy(ip, enforce_realtime);

		std::cout << "\nPress Enter to stop proxy." << '\n';
		return std::cin.get();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Connection to fci is not possible with error:\n" << e.what() << '\n';
		std::cout << "Use franka_proxy --help for more program options." << '\n';
		return -1;
	}
}

std::vector<std::array<double, 7>> find_derivative(
	const std::vector<std::array<double, 7>>& trajectory)
{
	std::vector<std::array<double, 7>> d;
	for (int i = 0; i < trajectory.size() - 1; i++)
	{
		std::array<double, 7> pos;
		for (int joint = 0; joint < pos.size(); joint++)
		{
			pos[joint] = (trajectory[i + 1][joint] - trajectory[i][joint]) / 0.001;
		}
		d.push_back(pos);
	}
	return d;
}

std::vector<double> find_abs_max(
	const std::vector<std::array<double, 7>>& curve)
{
	std::array<double, 7> out{0};
	int index = 0;
	for (int a = 0; a < curve.size(); a++)
	{
		auto& arr = curve[a];
		for (int i = 0; i < 7; i++)
		{
			double abs = std::abs(arr[i]);
			if (abs > out[i])
			{
				out[i] = abs;
				index = a;
			}
		}
	}
	std::cout << "Max at " << index << "\n";
	return std::vector(out.begin(), out.end());
}

void print_vec(const std::vector<double>& vec)
{
	for (const auto e : vec)
		std::cout << e << " ";

	std::cout << "\n";
}

#include "motion_generator_joint_max_accel.hpp"
#include <fstream>

int main2(int argc, char* argv[])
{
	using namespace franka_proxy;
	std::array<double, 7> q_goal = {1, 1, 1, 1, 1, 1, 1};
	std::mutex state_lock;
	franka::RobotState cur_state;
	cur_state.q_d = {0};
	std::atomic_bool stop_motion;
	bool stop_on_contact = false;
	detail::franka_joint_motion_generator gen(1, q_goal, state_lock, cur_state, stop_motion, stop_on_contact);
	std::vector<std::array<double, 7>> trajectory;
	for (int i = 0; i < 10000; i++)
	{
		auto pose = gen(cur_state, franka::Duration(i ? 1 : 0));
		if (pose.motion_finished)
		{
			std::cout << "motion finished at: " << i << "\n";
			break;
		}
		trajectory.push_back(pose.q);
	}

	auto vel = find_derivative(trajectory);

	auto acc = find_derivative(vel);

	auto jerk = find_derivative(acc);

	auto djerk = find_derivative(jerk);
	auto max_djerk = find_abs_max(djerk);

	auto max_jerk = find_abs_max(jerk);

	auto max_acc(find_abs_max(acc));

	auto max_vel = find_abs_max(vel);

	auto max_loc = find_abs_max(trajectory);
	std::cout << "dJerk:\n";
	print_vec(max_djerk);

	std::cout << "Jerk:\n";
	print_vec(max_jerk);

	std::cout << "Acc:\n";
	print_vec(max_acc);

	std::cout << "Vel:\n";
	print_vec(max_vel);

	std::cout << "x:\n";
	print_vec(max_loc);

	std::ofstream f("./motion.csv", std::ios::trunc);
	f << "time,x,vel,acc,jerk\n";
	for (int i = 0; i < jerk.size(); i++)
	{
		f << i << "," << trajectory[i][0] << "," << vel[i][0] << "," << acc[i][0] << "," << jerk[i][0] << "\n";
	}
	f.close();
	std::cout << argv[0] << "wrote to file";

	return 0;
}
