/**
 *************************************************************************
 *
 * @file exception.hpp
 *
 * Exceptions thrown during controlling the robot.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_CLIENT__EXCEPTION_HPP)
#define INCLUDED__FRANKA_PROXY_CLIENT__EXCEPTION_HPP


#include <exception>


namespace franka_proxy
{


/**
 *************************************************************************
 *
 * @class exception
 *
 * Groups together all custom exceptions thrown by the entire framework,
 * so these may be caught at a single place within high level code.
 *
 ************************************************************************/
class exception : public std::exception
{
public:

	exception() = default;
	~exception() noexcept override = default;


	/**
	 * Retrieve (static, constant) description string via
	 * the standard C++ exception interface.
	 */
	const char* what() const noexcept override
	{
		return "General exception";
	}
};


/**
 *************************************************************************
 *
 * @class remote_exception
 *
 * Groups together all exceptions thrown due to exceptions on the
 * remote side.
 *
 ************************************************************************/
class remote_exception : public exception
{
public:
	const char* what() const noexcept override
	{
		return "Exception thrown on remote side.";
	}
};


/**
 *************************************************************************
 * Thrown if an error occurs when loading the model library.
 ************************************************************************/
class model_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "Error while loading the model library.";
	}
};


/**
 *************************************************************************
 * Thrown if a connection to the robot cannot be established, or when
 * a timeout occurs.
 ************************************************************************/
class network_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "Connection to the robot cannot be established, or timeout occured.";
	}
};


/**
 *************************************************************************
 * Thrown if the robot returns an incorrect message.
 ************************************************************************/
class protocol_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "Incorrect message returned by robot.";
	}
};


/**
 *************************************************************************
 * Thrown if the robot does not support this version of libfranka.
 ************************************************************************/
class incompatible_version_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "Incompatible version of libfranka used on remote side.";
	}
};


/**
 *************************************************************************
 * Thrown if an error occurs during motion generation or torque control.
 ************************************************************************/
class control_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "An error occured during motion generation or torque control.";
	}
};


/**
 *************************************************************************
 * Thrown if an error occurs during command execution.
 ************************************************************************/
class command_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "An error occured during command execution.";
	}
};


/**
 *************************************************************************
 * Thrown if realtime priority cannot be set.
 ************************************************************************/
class realtime_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "Realtime priority cannot be set.";
	}
};


/**
 *************************************************************************
 * Thrown if an operation cannot be performed.
 ************************************************************************/
class invalid_operation_exception : public remote_exception
{
public:
	const char* what() const noexcept override
	{
		return "An operation cannot be performed.";
	}
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__EXCEPTION_HPP) */
