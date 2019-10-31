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

	static constexpr const char* command_strings[10] =
	{
		"MOVE",
		"MOVE_CONTACT",
		"FORCE_Z",
		"OPEN_GRIPPER",
		"CLOSE_GRIPPER",
		"GRASPING_GRIPPER",
		"START_RECORDING",
		"STOP_RECORDING",
		"SPEED",
		"ERRORRECOVERY"
	};

	
	static constexpr const char* command_end_marker = ";";


	enum command_type
	{
		move,
		move_contact,
		force_z,
		open_gripper,
		close_gripper,
		grasp_gripper,
		start_recording,
		stop_recording,
		speed,
		error_recovery,
		message_type_count
	};


	enum feedback_type
	{
		success,
		success_command_failed,
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