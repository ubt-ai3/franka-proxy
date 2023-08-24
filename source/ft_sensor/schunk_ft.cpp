#include "schunk_ft.hpp"

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include <asio/connect.hpp>
#include <asio/registered_buffer.hpp>
#include <asio/ip/address.hpp>
#include <asio/system_error.hpp>
#include <asio/ip/tcp.hpp>


namespace franka_proxy
{
schunk_ft_sensor::schunk_ft_sensor(const Eigen::Affine3f& kms_T_flange,
                                   const Eigen::Affine3f& EE_T_kms,
                                   const Eigen::Vector<double, 6>& bias,
                                   const Eigen::Vector3d& load_mass)
	: ft_sensor(kms_T_flange, EE_T_kms, bias, load_mass),
	  socket_(io_service_),
	  receiver_endpoint_(asio::ip::make_address(ip_), port_)
{
	try {
		socket_.open(asio::ip::udp::v4());
	}
	catch(...)
	{
		throw ft_sensor_connection_exception();
	}

	socket_.send_to(asio::buffer(start_streaming_msg_), receiver_endpoint_);
	worker_ = std::thread(&schunk_ft_sensor::run, this);

	set_response_handler([&](const ft_sensor_response& response) { current_ft_sensor_response_.store(response); });

}

schunk_ft_sensor::schunk_ft_sensor(const Eigen::Affine3f& kms_T_flange,
	const Eigen::Affine3f& EE_T_kms,
	const std::string config_file)
	: schunk_ft_sensor::schunk_ft_sensor(kms_T_flange, EE_T_kms, bias_from_config(config_file), load_mass_from_config(config_file))
{
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

void schunk_ft_sensor::update_calibration(const std::string config_file)
{
	bias_ = bias_from_config(config_file);
	load_mass_ = load_mass_from_config(config_file);
}

void schunk_ft_sensor::set_response_handler(const std::function<void(const ft_sensor_response&)>& functor)
{
	std::unique_lock<std::mutex> lock(functor_lock_);
	handle_data_ = functor;
}

void schunk_ft_sensor::remove_response_handler()
{
	std::unique_lock<std::mutex> lock(functor_lock_);
	handle_data_ = [](const ft_sensor_response&)
	{
	}; //no-op
}

void schunk_ft_sensor::run()
{
	try
	{
		std::array<unsigned char, 36> recv_buf{'\0'};
		asio::ip::udp::endpoint sender_endpoint;

		while (!stopped_)
		{
			size_t len = socket_.receive_from(
				asio::buffer(recv_buf), sender_endpoint);

			ft_sensor_response resp;

			// meta data
			//resp.sequence_number = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[0]));		// sequence number, counting send packages
			resp.ft_sequence_number = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[4]));
			// sequence number, counting internal control-loop
			//resp.status = ntohl(*reinterpret_cast<uint32_t*>(&recv_buf[8]));				//www.ati-ia.com/app_content/documents/9610-05-1022.pdf

			// data
			for (int i = 0; i < 3; i++)
				resp.data[i] = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&recv_buf[12 + i * 4]))) / cpf_;
			for (int i = 3; i < 6; i++)
				resp.data[i] = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&recv_buf[12 + i * 4]))) / cpt_;

			std::unique_lock<std::mutex> lock(functor_lock_);
			handle_data_(resp);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}
Eigen::Vector<double, 6> schunk_ft_sensor::bias_from_config(const std::string config_file) const
{
	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);

	Eigen::Vector<double, 6> bias;
	for (int i = 0; i < bias.size(); i++)
		bias[i] = config["bias"].at(i);

	return bias;
}

Eigen::Vector3d schunk_ft_sensor::load_mass_from_config(const std::string config_file) const
{
	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);
	Eigen::Vector3d load_mass;

	for (int i = 0; i < load_mass.size(); i++)
		load_mass[i] = config["load_mass"].at(i);

	return load_mass;
}
} /* namespace franka_proxy */
