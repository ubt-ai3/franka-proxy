#if !defined(INCLUDED__FT_SENSOR__SCHUNK_FT_HPP)
#define INCLUDED__FT_SENSOR__SCHUNK_FT_HPP

#include <mutex>
#include <winsock2.h>

#include <asio/io_service.hpp>
#include <asio/ip/udp.hpp>

#include "ft_sensor/ft_sensor.hpp"
#include <Eigen/Geometry>

class schunk_ft_sensor final : public ft_sensor
{
public:
	schunk_ft_sensor(const Eigen::Affine3f& kms_T_flange,
	                 const Eigen::Affine3f& EE_T_kms,
	                 const std::array<double, 6>& bias,
	                 const Eigen::Affine3f& load);

	~schunk_ft_sensor() override;

	void set_response_handler(const std::function<void(const response&)>& functor) override;
	void remove_response_handler() override;

	double get_data_rate();
	/*
	 * Set the amount of responses per second. The rate is rounded up to fraction of 7000 with a whole number denominator, eg.
	 * 7000/1, 7000/2 , 7000/3, ...
	 */
	void set_data_rate(int rate);


	std::array<double, 6> bias();
	Eigen::Affine3f load();

private:
	void run();

	//std::string read_current_config();
	//void change_setting(const std::string& target);
	//void parse_config(const std::string& xml_config);


	const std::string ip_ = "192.168.2.1";
	const unsigned short port_ = 49152;
	const std::string http_port_ = "80";
	const int html_version_ = 10;


	std::mutex counts_lock_;
	double cpf_ = std::numeric_limits<double>::quiet_NaN();
	double cpt_ = std::numeric_limits<double>::quiet_NaN();

	std::mutex data_rate_lock;
	double data_rate_ = std::numeric_limits<double>::quiet_NaN();

	asio::io_service io_service_;
	asio::ip::udp::socket socket_;
	asio::ip::udp::endpoint sender_endpoint_;
	asio::ip::udp::endpoint receiver_endpoint_;

	std::atomic_bool stopped_ = false;
	std::thread worker_;

	std::mutex functor_lock_;
	std::function<void(const response&)> handle_data{
		[](const response&)
		{
		}
	};
	response current_response_{0, 0, 0, {}};

	/* see  table 9.1 in Net F/T user manual. */
	const std::array<unsigned char, 8> start_streaming_msg_ = []()
	{
		std::array<unsigned char, 8> msg;
		*(uint16_t*)&msg[0] = htons(0x1234); // standard header
		*(uint16_t*)&msg[2] = htons(2); // start realtime-streaming
		*(uint32_t*)&msg[4] = htonl(0); // forever

		return msg;
	}();


	const std::array<unsigned char, 8> end_streaming_msg_ = []()
	{
		std::array<unsigned char, 8> msg;
		*(uint16_t*)&msg[0] = htons(0x1234); // standard header
		*(uint16_t*)&msg[2] = htons(0); // end streaming
		*(uint32_t*)&msg[4] = htonl(0); // unused

		return msg;
	}();
};

#endif /* !defined(INCLUDED__FT_SENSOR__SCHUNK_FT_HPP) */
