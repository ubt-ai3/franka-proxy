/**
 *************************************************************************
 *
 * @file exception.hpp
 *
 * Header for the Exception Module.
 *
 ************************************************************************/

#pragma once


#include <exception>


namespace franka_control
{
/**
 *************************************************************************
 *
 * @class exception
 *
 * Groups together all custom exceptions thrown by the entire hardware
 * library, so these may be caught at a single place within high level
 * code.
 *
 ************************************************************************/
class exception : public std::exception
{
public:
	exception() noexcept = default;

	/**
	 * Retrieve (static, constant) description string via
	 * the standard C++ exception interface.
	 */
	[[nodiscard]] const char* what() const noexcept override
	{
		return "General franka control exception occurred.";
	}
};


/**
 *************************************************************************
 *
 * Thrown if no solution was found for an inverse kinematic problem.
 *
 ************************************************************************/
class ik_failed final : public exception
{
public:
	[[nodiscard]] const char* what() const noexcept override
	{
		return "No solution was found for inverse kinematics.";
	}
};


/**
 *************************************************************************
 *
 * Thrown if a requested operation is not yet implemented.
 * 
 ************************************************************************/
class not_implemented final : public exception
{
public:
	[[nodiscard]] const char* what() const noexcept override
	{
		return "Not yet implemented operation requested.";
	}
};


/**
 *************************************************************************
 *
 * Thrown if an error occurred during robot interpolation.
 * 
 ************************************************************************/
class interpolation_error final : public exception
{
public:
	[[nodiscard]] const char* what() const noexcept override
	{
		return "Interpolation failed.";
	}
};


/**
 *************************************************************************
 *
 * Thrown if some general API method fails.
 * 
 ************************************************************************/
class api_call_failed final : public exception
{
public:
	[[nodiscard]] const char* what() const noexcept override
	{
		return "API call failed.";
	}
};
} /* namespace franka_control */
