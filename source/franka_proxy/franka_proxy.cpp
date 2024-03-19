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
	program.add_argument("--ip")
	       .help("specify ip for franka-proxy to fci "
		       "(otherwise uses default 192.168.0.1)");

	program.add_argument("--enforce-realtime")
	       .help("activates fci realtime mode for control-loops")
	       .flag();

	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		std::cout << program;
		return -1;
	}

	std::string ip("192.168.0.1");
	if (program.is_used("--ip"))
		ip = program.get<std::string>("--ip");

	const bool enforce_realtime = program["--enforce-realtime"] == true;
	std::cout << "Starting franka-proxy with ip: " << ip << std::endl;

	if (enforce_realtime)
		std::cout << "Enabled realtime control loops. "
			"(This will only work on a realtime linux system.)" << std::endl;

	try
	{
		franka_proxy::franka_proxy proxy(ip, enforce_realtime);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Connection to fci is not possible with error:\n" << e.what() << std::endl;
		std::cout << "Use franka_proxy --help for more program options." << std::endl;
		return -1;
	}

	std::cout << "\nPress Enter to stop proxy." << std::endl;
	return std::cin.get();
}
