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

#include <viral_core/ms_network.hpp>


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

	mover_(controller_),

	network_(new viral_core::ms_network_context("network")),
	control_server_(*network_, 4711, controller_, mover_),
	state_server_(*network_, 4712, controller_)
{
	state_server_.start();
	mover_.start();

	while(true)
	{
		viral_core::thread_util::sleep_seconds(1);
	}
}




} /* namespace franka_proxy */



int main()
{
	franka_proxy::franka_proxy proxy;
	std::cin.ignore();
}