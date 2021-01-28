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

#include <array>
#include <cstdint>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace franka_proxy
{

struct message_move_ptp
{
    static constexpr char type[] = "move-ptp";
    std::array<double, 7> config;
};

void to_json(nlohmann::json& j, const message_move_ptp&);
void from_json(const nlohmann::json& j, message_move_ptp&);

struct message_move_contact
{
    static constexpr char type[] = "move-contact";
    std::array<double, 7> config;
};

void to_json(nlohmann::json& j, const message_move_contact&);
void from_json(const nlohmann::json& j, message_move_contact&);

struct message_move_hybrid_sequence
{
    static constexpr char type[] = "move-sequence";

	std::vector<std::array<double, 7>> q_data;
	std::vector<std::array<double, 6>> f_data;
	std::vector<std::array<double, 6>> s_data;
	
};

void to_json(nlohmann::json& j, const message_move_hybrid_sequence&);
void from_json(const nlohmann::json& j, message_move_hybrid_sequence&);

struct message_force_z{
    static constexpr char type[] = "force-z";

    double mass;
    double duration;
};

void to_json(nlohmann::json& j, const message_force_z&);
void from_json(const nlohmann::json& j, message_force_z&);

struct message_open_gripper
{
    static constexpr char type[] = "open-gripper";
};

void to_json(nlohmann::json& j, const message_open_gripper&);
void from_json(const nlohmann::json& j, message_open_gripper&);

struct message_close_gripper{
    static constexpr char type[] = "close-gripper";
};

void to_json(nlohmann::json& j, const message_close_gripper&);
void from_json(const nlohmann::json& j, message_close_gripper&);

struct message_grasping_gripper{
    static constexpr char type[] = "grasping-gripper";

    double speed;
    double force;
};

void to_json(nlohmann::json& j, const message_grasping_gripper&);
void from_json(const nlohmann::json& j, message_grasping_gripper&);

struct message_start_recording{
    static constexpr char type[]= "start-recording";
};

void to_json(nlohmann::json& j, const message_start_recording&);
void from_json(const nlohmann::json& j, message_start_recording&);

struct message_stop_recording
{
    static constexpr char type[] = "stop-recording";
};

void to_json(nlohmann::json& j, const message_stop_recording&);
void from_json(const nlohmann::json& j, message_stop_recording&);

struct message_stop_recording_response
{
	static constexpr char type[] = "stop-recording-response";
	std::vector<std::array<double, 7>> q_sequence;
	std::vector<std::array<double, 6>> f_sequence;
};

void to_json(nlohmann::json& j, const message_stop_recording_response&);
void from_json(const nlohmann::json& j, message_stop_recording_response&);

struct message_speed{
    static constexpr char type[] = "speed"; 

    double speed;
};

void to_json(nlohmann::json& j, const message_speed&);
void from_json(const nlohmann::json& j, message_speed&);

struct message_error_recovery 
{
    static constexpr char type[] = "error-recovery";
};

void to_json(nlohmann::json& j, const message_error_recovery&);
void from_json(const nlohmann::json& j, message_error_recovery&);

enum class message_result: std::uint8_t 
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
    franka_exception,
	unknown_operation,
};

struct message_generic_response 
{
	static constexpr char type[] = "generic-response";
	std::uint8_t status_code;

    message_generic_response() noexcept
    : status_code{ 0 }
    {}

	message_generic_response(message_result result) noexcept
	: status_code{ static_cast<std::uint8_t>(result) }
	{}
};

void to_json(nlohmann::json& j, const message_generic_response& msg);
void from_json(const nlohmann::json& j, message_generic_response& msg);

} /* namespace franka_proxy */



#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_MESSAGES_HPP) */
