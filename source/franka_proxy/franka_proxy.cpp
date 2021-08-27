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


namespace franka_proxy
{


//////////////////////////////////////////////////////////////////////////
//
// franka_proxy
//
//////////////////////////////////////////////////////////////////////////


franka_proxy::franka_proxy()
	:
	controller_("192.168.1.1"),

	control_server_(franka_control_port, controller_),
	state_server_(franka_state_port, controller_)
{}


} /* namespace franka_proxy */


int main()
{
	try {
		franka_proxy::franka_proxy proxy;
		return std::cin.get();
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}
}
