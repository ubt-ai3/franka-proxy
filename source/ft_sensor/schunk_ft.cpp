#include "schunk_ft.hpp"

#include <iostream>



schunk_ft_sensor::schunk_ft_sensor(
	const Eigen::Affine3f& kms_T_flange,
	const Eigen::Affine3f& EE_T_kms,
	const std::array<double, 6>& bias,
	const Eigen::Affine3f& load)
	: ft_sensor(kms_T_flange, EE_T_kms, bias, load),
	  socket_(io_service_),
	  receiver_endpoint_(asio::ip::make_address(ip_), port_)
{
	socket_.open(asio::ip::udp::v4());
	socket_.send_to(asio::buffer(start_streaming_msg_), receiver_endpoint_);
	worker_ = std::thread(&schunk_ft_sensor::run, this);
}

schunk_ft_sensor::~schunk_ft_sensor()
{
	try
	{
		stopped_ = true;
		worker_.join();

		socket_.send_to(asio::buffer(end_streaming_msg_), receiver_endpoint_);
		socket_.shutdown(asio::ip::udp::socket::shutdown_both);
		socket_.close();


	}
	catch (const asio::system_error& e)
	{
		if (e.code().value() != 10054)
			std::cerr << "Unhandled exception, netbox might still be streaming data. " << e.what();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Unhandled exception, netbox might still be streaming data. " << e.what();
	}
}

void schunk_ft_sensor::set_response_handler(const std::function<void(const response&)>& functor)
{
	std::unique_lock<std::mutex> lock(functor_lock_);
	handle_data = functor;
}

void schunk_ft_sensor::remove_response_handler()
{
	std::unique_lock<std::mutex> lock(functor_lock_);
	handle_data = [](const response&) {}; // no-op
}

double schunk_ft_sensor::data_rate() const
{
	return data_rate_;
}

std::array<double, 6> schunk_ft_sensor::bias() const
{
	return bias_;
}

Eigen::Affine3f schunk_ft_sensor::load()
{
	return load_;
}

void schunk_ft_sensor::run()
{
	try
	{
		std::array<unsigned char, 36> recv_buf{ '\0' };
		asio::ip::udp::endpoint sender_endpoint;

		while (!stopped_)
		{
			using namespace std::chrono_literals;

			size_t len = socket_.receive_from(
				asio::buffer(recv_buf), sender_endpoint);

			response resp;
			resp.rdt_sequence = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[0]));
			resp.ft_sequence = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[4]));
			resp.status = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[8]));

			for (int i = 0; i < 3; i++) 
				resp.FTData[i] = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&recv_buf[12 + i * 4]))) / cpf_;
			for (int i = 3; i < 6; i++) 
				resp.FTData[i] = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&recv_buf[12 + i * 4]))) / cpt_;

			std::unique_lock<std::mutex> lock(functor_lock_);
			handle_data(resp);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

}
