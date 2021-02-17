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
	
	explicit remote_exception(std::string reason) noexcept
		: reason_(std::move(reason))
		{}

	
	const char* what() const noexcept override
	{
		return reason_.c_str();
	}

private:
	std::string reason_;
};




/**
 *************************************************************************
 * Thrown if an error occurs when loading the model library.
 ************************************************************************/
class model_exception : public remote_exception
{
public:
	
	explicit model_exception(std::string reason) noexcept
		: remote_exception{std::move(reason)}
		{}
		
};




/**
 *************************************************************************
 * Thrown if a connection to the robot cannot be established, or when
 * a timeout occurs.
 ************************************************************************/
class network_exception : public remote_exception
{
public:
	
	explicit network_exception(std::string reason) noexcept
		: remote_exception{std::move(reason)}
		{}
};




/**
 *************************************************************************
 * Thrown if the robot returns an incorrect message.
 ************************************************************************/
class protocol_exception : public remote_exception
{
public:
	
	explicit protocol_exception(std::string reason) noexcept
		: remote_exception{std::move(reason)}
		{}
};




/**
 *************************************************************************
 * Thrown if the robot does not support this version of libfranka.
 ************************************************************************/
class incompatible_version_exception : public remote_exception
{
public:
	
	explicit incompatible_version_exception(std::string reason) noexcept
		: remote_exception{std::move(reason)}
		{}
};




/**
 *************************************************************************
 * Thrown if an error occurs during motion generation or torque control.
 ************************************************************************/
class control_exception : public remote_exception
{
public:
	
	explicit control_exception(std::string reason) noexcept
		: remote_exception{std::move(reason)}
		{}
};




/**
 *************************************************************************
 * Thrown if an error occurs during command execution.
 ************************************************************************/
class command_exception : public remote_exception
{
public:
	
	explicit command_exception(std::string reason) noexcept
		: remote_exception{ std::move(reason) }
		{}
};




/**
 *************************************************************************
 * Thrown if realtime priority cannot be set.
 ************************************************************************/
class realtime_exception : public remote_exception
{
public:
	
	explicit realtime_exception(std::string reason) noexcept
		: remote_exception{ std::move(reason) }
		{}
};




/**
 *************************************************************************
 * Thrown if an operation cannot be performed.
 ************************************************************************/
class invalid_operation_exception : public remote_exception
{
public:
	
	explicit invalid_operation_exception(std::string reason) noexcept
		: remote_exception{ std::move(reason) }
		{}
};



	
/**
 *************************************************************************
 * Thrown if a command is unknown to the server.
 ************************************************************************/
class unknown_command_exception: public remote_exception
{
public:

	explicit unknown_command_exception(std::string reason) noexcept
		: remote_exception{ std::move(reason) }
	{}
};


	
	
/**
*************************************************************************
* Thrown if the response received was bad.
************************************************************************/
class bad_response_exception: public exception
{
public:
	
	const char* what() const noexcept override
	{
		return "Bad response sent by the server.";
	}
};


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_CLIENT__EXCEPTION_HPP) */
