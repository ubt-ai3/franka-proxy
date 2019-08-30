/**
 *************************************************************************
 *
 * @file franka_proxy_messages.hpp
 *
 * Messages sent between proxy and client.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP)
#define INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP


namespace franka_proxy
{


class franka_proxy_messages
{

public:

	enum command_type { move, open_gripper, close_gripper, speed, error_recovery, message_type_count };
	static constexpr const char* command_strings[5] = {"MOVE", "OPEN GRIPPER", "CLOSE GRIPPER", "SPEED", "ERRORRECOVERY"};
	static constexpr const char* command_end_marker = ";";

	enum feedback_type
	{
		success,
		model_exception,
		network_exception,
		protocol_exception,
		incompatible_version,
		control_exception,
		command_exception,
		realtime_exception,
		invalid_operation,
		franka_exception
	};
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP) */