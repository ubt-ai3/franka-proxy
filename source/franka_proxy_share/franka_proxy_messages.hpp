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

	static constexpr const char* move = "MOVE";
	static constexpr const char* stop_move = "STOP MOVE";
	static constexpr const char* speed = "SPEED";
	static constexpr const char* open_gripper = "OPEN GRIPPER";
	static constexpr const char* close_gripper = "CLOSE GRIPPER";
	static constexpr const char* stop_gripper = "STOP GRIPPER";
};




} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP) */