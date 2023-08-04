#include "schunk_ft.hpp"

#include <iostream>
#include <fstream>

#include <asio/connect.hpp>
#include <asio/registered_buffer.hpp>
#include <asio/ip/address.hpp>
#include <asio/system_error.hpp>
#include <asio/ip/tcp.hpp>


schunk_ft_sensor::schunk_ft_sensor(const Eigen::Affine3f& kms_T_flange, const Eigen::Affine3f& EE_T_kms, const std::array<double, 6>& bias, const Eigen::Affine3f& load)
: ft_sensor(kms_T_flange,EE_T_kms, bias, load),
	socket_(io_service_),
	receiver_endpoint_(asio::ip::make_address(ip_), port_)
{
	using asio::ip::udp;
	using asio::buffer;

	//parse_config(read_current_config());

	socket_.open(udp::v4());
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
	handle_data = [](const response&) {}; //no-op
}

double schunk_ft_sensor::get_data_rate()
{
	throw std::exception("unsupported");

	//std::unique_lock<std::mutex> lock(data_rate_lock);
	//return data_rate_;
}

void schunk_ft_sensor::set_data_rate(int rate)
{
	throw std::exception("unsupported");

	//using namespace std::literals;
	//change_setting("/comm.cgi?comrdtrate="s + std::to_string(rate));
}

std::array<double, 6> schunk_ft_sensor::bias()
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
			resp.rdt_sequence = ntohl(*(uint32_t*)&recv_buf[0]);
			resp.ft_sequence = ntohl(*(uint32_t*)&recv_buf[4]);
			resp.status = ntohl(*(uint32_t*)&recv_buf[8]);


			{
				std::unique_lock<std::mutex> lock(counts_lock_);

				for (int i = 0; i < 3; i++) {
					resp.FTData[i] = (int32_t)(ntohl(*(int32_t*)&recv_buf[12 + i * 4])) / cpf_;
				}
				for (int i = 3; i < 6; i++) {
					resp.FTData[i] = (int32_t)(ntohl(*(int32_t*)&recv_buf[12 + i * 4])) / cpt_;
				}
			}

			std::unique_lock<std::mutex> lock(functor_lock_);
			handle_data(resp);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}

}

// todo JHa this contains boost-beast code (not header only..)
//
//std::string schunk_ft_sensor::read_current_config()
//{
//	namespace http = beast::http;
//	using tcp = asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
//
//	auto const target = "/netftapi2.xml";
//
//
//	tcp::resolver resolver{ io_service_ };
//	tcp::socket socket{ io_service_ };
//	auto const results = resolver.resolve(ip_, http_port_);
//	asio::connect(socket, results.begin(), results.end());
//
//	http::request<http::string_body> req{ http::verb::get, target, html_version_ };
//	req.set(http::field::host, ip_);
//	req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
//	http::write(socket, req);
//
//
//	beast::flat_buffer buffer;
//	http::response<http::dynamic_body> res;
//	http::read(socket, buffer, res);
//
//	return buffers_to_string(res.body().data());
//}
//
//void schunk_ft_sensor::change_setting(const std::string& target)
//{
//	// Possible settings see https://www.ati-ia.com/app_content/documents/9610-05-1022.pdf, Page 64 ff.
//
//	namespace http = beast::http;
//	using tcp = asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
//
//
//	tcp::resolver resolver{ io_service_ };
//	tcp::socket socket{ io_service_ };
//	auto const results = resolver.resolve(ip_, http_port_);
//	asio::connect(socket, results.begin(), results.end());
//
//	http::request<http::string_body> req{ http::verb::get, target, html_version_ };
//	req.set(http::field::host, ip_);
//	req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
//	http::write(socket, req);
//}
//
//void schunk_ft_sensor::parse_config(const std::string& xml_config)
//{
//	std::stringstream ss;
//	ss << xml_config;
//
//	property_tree::ptree pt;
//	read_xml(ss, pt);
//
//	{
//		std::unique_lock<std::mutex> lock(counts_lock_);
//		cpf_ = pt.get<double>("netft.cfgcpf"); // counts per force
//		cpt_ = pt.get<double>("netft.cfgcpt"); // counts per torque
//	}
//
//	std::unique_lock<std::mutex> lock(data_rate_lock);
//	data_rate_ = pt.get<double>("netft.comrdtrate");
//}



