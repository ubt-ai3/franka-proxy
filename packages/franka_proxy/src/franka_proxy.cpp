/**
 *************************************************************************
 *
 * @file franka_proxy.cpp
 *
 * ..., implementation.
 *
 ************************************************************************/


#include "franka_proxy.hpp"

#include <cstdlib>
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

namespace
{
std::tuple<std::string, bool> parse_arguments(int argc, char* argv[]);
}


int main(int argc, char* argv[])
{
	try
	{
		auto [ip, enforce_realtime] = parse_arguments(argc, argv);

		std::cout << "Starting franka-proxy with ip: " << ip << ".\n";
		if (enforce_realtime)
			std::cout << "Enabled realtime control loops. "
				<< "(This will only work on a realtime linux system "
				<< "- or more or less - using process priority on "
				<< "windows with admin privileges)." << '\n';

		franka_proxy::franka_proxy proxy(ip, enforce_realtime);

		std::cout << '\n' << "Press Enter to stop proxy." << std::endl;
		std::cin.get();
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Connection to FCI is not possible with error:\n"
			<< e.what() << '\n';
		std::cout << "Use franka_proxy --help for more program options." << '\n';
		return EXIT_FAILURE;
	}
}


namespace
{
std::tuple<std::string, bool> parse_arguments(int argc, char* argv[])
{
	argparse::ArgumentParser program("franka_proxy", FRANKA_PROXY_VERSION);

	program.add_argument("--ip")
	       .help("specify IP for franka-proxy to FCI (default: 192.168.1.1)");

	program.add_argument("--enforce-realtime")
	       .help("activates FCI realtime mode for control-loops")
	       .flag();

	program.parse_args(argc, argv);

	std::string ip = program.present("--ip").value_or("192.168.1.1");
	bool enforce_realtime = program["--enforce-realtime"] == true;

	return {ip, enforce_realtime};
}
}
