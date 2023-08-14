#if !defined(INCLUDED__FT_SENSOR__SCHUNK_FT_HPP)
#define INCLUDED__FT_SENSOR__SCHUNK_FT_HPP

#include <mutex>

#include <asio/io_service.hpp>
#include <asio/ip/udp.hpp>

#include <Eigen/Geometry>

#include "ft_sensor/ft_sensor.hpp"


class schunk_ft_sensor final
	: public ft_sensor
{
public:
	schunk_ft_sensor(const Eigen::Affine3f& kms_T_flange,
	                 const Eigen::Affine3f& EE_T_kms,
	                 const std::array<double, 6>& bias,
	                 const Eigen::Affine3f& load);

	~schunk_ft_sensor() override;

	void set_response_handler(const std::function<void(const response&)>& functor) override;
	void remove_response_handler() override;

	double data_rate() const;

	std::array<double, 6> bias() const;
	Eigen::Affine3f load();

private:
	void run();

	std::string ip_ = "192.168.2.1";
	unsigned short port_ = 49152; // port the net ft sensor always uses


	//If in doubt, check current configuration in the web interface.
	double cpf_ = 1000000; // count to force (count --> N)
	double cpt_ = 1000000; // count to torque (count --> Nm)
	std::mutex data_rate_lock;
	double data_rate_ = 7000;


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
		*reinterpret_cast<uint16_t*>(&msg[0]) = htons(0x1234);	// standard header
		*reinterpret_cast<uint16_t*>(&msg[2]) = htons(2);		// start realtime-streaming
		*reinterpret_cast<uint32_t*>(&msg[4]) = htonl(0);		// forever

		return msg;
	}();

	const std::array<unsigned char, 8> end_streaming_msg_ = []()
	{
		std::array<unsigned char, 8> msg;
		*reinterpret_cast<uint16_t*>(&msg[0]) = htons(0x1234);	// standard header
		*reinterpret_cast<uint16_t*>(&msg[2]) = htons(0);		// end streaming
		*reinterpret_cast<uint32_t*>(&msg[4]) = htonl(0);		// unused

		return msg;
	}();
};

#endif /* !defined(INCLUDED__FT_SENSOR__SCHUNK_FT_HPP) */
