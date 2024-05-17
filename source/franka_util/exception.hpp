/**
 *************************************************************************
 *
 * @file exception.hpp
 *
 * Header for the Exception Module.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_CONTROL__EXCEPTION_HPP)
#define INCLUDED__FRANKA_CONTROL__EXCEPTION_HPP


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
class exception: public std::exception
{
	public:

		exception() noexcept = default;
		~exception() noexcept override = default;


		/**
		 * Retrieve (static, constant) description string via
		 * the standard C++ exception interface.
		 */
		const char* what() const noexcept override
			{ return "General hardware exception"; }
};




/**
 *************************************************************************
 *
 * Thrown if no solution was found for an inverse kinematic problem.
 *
 ************************************************************************/
class ik_failed : public exception
{
	public:
		const char* what() const noexcept override
			{ return "No solution was found for inverse kinematics."; }
};


/**
 *************************************************************************
 *
 * Thrown if a requested operation is not yet implemented.
 * 
 ************************************************************************/
class not_implemented : public exception
{
	public:
		const char* what() const noexcept override
			{ return "Invalid operation requested."; }
};


/**
 *************************************************************************
 *
 * Thrown if an error occured during robot interpolation.
 * 
 ************************************************************************/
class interpolation_error : public exception
{
	public:
		const char* what() const noexcept override
			{ return "Interpolation failed."; }
};


/**
 *************************************************************************
 *
 * Thrown if some general API method fails.
 * 
 ************************************************************************/
class api_call_failed : public exception
{
	public:
		const char* what() const noexcept override
			{ return "API call failed."; }
};




} /* namespace franka_control */


#endif /* !defined(INCLUDED__FRANKA_CONTROL__EXCEPTION_HPP) */
