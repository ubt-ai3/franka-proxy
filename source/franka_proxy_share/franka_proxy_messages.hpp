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

	enum message_type { move, stop, speed, open_gripper, close_gripper, message_type_count };

	static constexpr const char* message_strings[5] = {"MOVE", "STOP", "SPEED", "OPEN GRIPPER", "CLOSE GRIPPER"};

	static constexpr const char* message_end_marker = ";";
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP) */