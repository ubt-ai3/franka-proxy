#ifndef INCLUDED__FT_SENSOR__SCHUNK_FT_HPP
#define INCLUDED__FT_SENSOR__SCHUNK_FT_HPP

#include <mutex>
#include <thread>

#include <asio/io_service.hpp>
#include <asio/ip/udp.hpp>

#include <Eigen/Geometry>

#include "ft_sensor.hpp"


namespace franka_proxy
{
class schunk_ft_sensor final
	: public ft_sensor
{
public:
	schunk_ft_sensor(
		const Eigen::Affine3f& kms_T_flange,
		const Eigen::Affine3f& EE_T_kms,
		const Eigen::Vector<double, 6>& bias,
		double load_mass);

	schunk_ft_sensor(
		const Eigen::Affine3f& kms_T_flange,
		const Eigen::Affine3f& EE_T_kms,
		const std::string& = "./assets/fts-config.json");

	~schunk_ft_sensor() override;

	void update_calibration(const std::string& config_file = "./assets/fts-config.json");

private:
	void set_response_handler(const std::function<void(const ft_sensor_response&)>& functor);
	void remove_response_handler();

	void run();

	void setup_connection();

	std::string ip_ = "192.168.2.1"; // hardcoded ip
	unsigned short port_ = 49152; // port the net ft sensor always uses


	//If in doubt, check current configuration in the web interface.
	double cpf_ = 1000000; // count to force (count --> N)
	double cpt_ = 1000000; // count to torque (count --> Nm)
	double data_rate_ = 7000;


	asio::io_service io_service_;
	asio::ip::udp::socket socket_;
	asio::ip::udp::endpoint sender_endpoint_;
	asio::ip::udp::endpoint receiver_endpoint_;


	std::atomic_bool stopped_ = false;
	std::thread worker_;
	std::mutex functor_lock_;
	std::function<void(const ft_sensor_response&)> handle_data_{
		[](const ft_sensor_response&)
		{
		}
	};


	/* see  table 9.1 in Net F/T user manual. */
	std::array<unsigned char, 8> start_streaming_msg_ = []()
	{
		std::array<unsigned char, 8> msg;
		*reinterpret_cast<uint16_t*>(&msg[0]) = htons(0x1234); // standard header
		*reinterpret_cast<uint16_t*>(&msg[2]) = htons(2); // start realtime-streaming
		*reinterpret_cast<uint32_t*>(&msg[4]) = htonl(0); // forever

		return msg;
	}();

	std::array<unsigned char, 8> end_streaming_msg_ = []()
	{
		std::array<unsigned char, 8> msg;
		*reinterpret_cast<uint16_t*>(&msg[0]) = htons(0x1234); // standard header
		*reinterpret_cast<uint16_t*>(&msg[2]) = htons(0); // end streaming
		*reinterpret_cast<uint32_t*>(&msg[4]) = htonl(0); // unused

		return msg;
	}();

	Eigen::Vector<double, 6> bias_from_config(const std::string& config_file) const;
	double load_mass_from_config(const std::string& config_file) const;
};
} /* namespace franka_proxy */

#endif // INCLUDED__FT_SENSOR__SCHUNK_FT_HPP
