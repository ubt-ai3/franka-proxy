/**
 *************************************************************************
 *
 * @file signal.hpp
 *
 * Signal class from viral_core by Tobias Werner, implementation.
 * Didn't want to use the whole library.
 *
 ************************************************************************/


#include "signal.hpp"

#include <algorithm>
#include <iostream>


namespace viral_core_lite
{


//////////////////////////////////////////////////////////////////////////
//
//	signal, C++11 std implementation
//
//////////////////////////////////////////////////////////////////////////


signal::signal() : signal_value_(false) { }
signal::~signal() noexcept { }


void signal::set(bool value) noexcept
{
	std::lock_guard<std::mutex> signal_guard(signal_mutex_);
	signal_value_ = value;
	condition_toggle_.notify_all();
}


bool signal::get() const noexcept
{
	std::lock_guard<std::mutex> signal_guard(signal_mutex_);
	return signal_value_;
}


bool signal::get_and_set(bool value) noexcept
{
	std::lock_guard<std::mutex> signal_guard(signal_mutex_);
	bool oldvalue = signal_value_;
	signal_value_ = value;
	condition_toggle_.notify_all();
	return oldvalue;
}


void signal::wait_for(bool value) const noexcept
	{ wait_for(value, default_timeout_seconds_); }


void signal::wait_for
	(bool value, float timeout_seconds) const noexcept
{
	// Check current value to avoid waiting
	// if the signal already has the desired value.
	{
		std::lock_guard<std::mutex> signal_guard(signal_mutex_);
		if (signal_value_ == value) return;
	}


	timeout_seconds =
		std::max(0.f, std::min(max_timeout_seconds_, timeout_seconds));

	
	// Notes:
	// - We use but a single waiting condition to wait
	//		for both signal states. This is not a problem,
	//		since premature timeouts anyways enforce clients
	//		to recheck the current signal value.
	// - We ensure NOTHROW in the case of any lock errors.
	//		We can't do anything here, but the client must
	//		anyways recheck the current signal value on return.
	try
	{
		std::unique_lock<std::mutex> lock(condition_mutex_);
		condition_toggle_.wait_for
			(lock,
			 std::chrono::milliseconds
				(static_cast<long long>(timeout_seconds * 1000)));
	}
	catch (...)
		{ std::cerr << "Signal waiting failed." << std::endl; }
}


const float signal::default_timeout_seconds_ = 0.25f;
const float signal::max_timeout_seconds_ = 60;




} /* namespace viral_core */