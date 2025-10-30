#include <ft_sensor/schunk_ft.hpp>

#include <chrono>
#include <fstream>
#include <future>
#include <iostream>

#include <asio/connect.hpp>
#include <asio/registered_buffer.hpp>
#include <asio/ip/address.hpp>
#include <asio/system_error.hpp>
#include <asio/ip/tcp.hpp>
#include <nlohmann/json.hpp>

#include "franka_proxy_util.hpp"


namespace franka_proxy
{
schunk_ft_sensor::schunk_ft_sensor(
	const Eigen::Affine3f& kms_T_flange,
	const Eigen::Affine3f& EE_T_kms,
	const Eigen::Vector<double, 6>& bias,
	double load_mass)
	: ft_sensor(kms_T_flange, EE_T_kms, bias, load_mass),
	  socket_(io_service_),
	  receiver_endpoint_(asio::ip::make_address(ip_), port_)
{
	load_mass_ = franka_proxy_util::tool_mass_from_fts();
	tool_com_ = franka_proxy_util::tool_center_of_mass_from_fts();
	tool_inertia_matrix_ = franka_proxy_util::tool_inertia_from_fts();

	setup_connection();
}


schunk_ft_sensor::schunk_ft_sensor(
	const Eigen::Affine3f& kms_T_flange,
	const Eigen::Affine3f& EE_T_kms,
	const std::string& config_file)
	: ft_sensor(kms_T_flange, EE_T_kms, read_bias_from_config(config_file), read_load_mass_from_config(config_file)),
	  socket_(io_service_),
	  receiver_endpoint_(asio::ip::make_address(ip_), port_)
{
	setup_connection();
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


void schunk_ft_sensor::update_calibration(const std::string& config_file)
{
	bias_ = read_bias_from_config(config_file);
	load_mass_ = read_load_mass_from_config(config_file);
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
			socket_.receive_from(
				asio::buffer(recv_buf), sender_endpoint);

			ft_sensor_response resp{};

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
		std::cerr << "schunk_ft_sensor::run"
			<< e.what() << '\n';
	}
}


void schunk_ft_sensor::setup_connection()
{
	socket_.open(asio::ip::udp::v4());
	socket_.send_to(asio::buffer(start_streaming_msg_), receiver_endpoint_);

	// read one response to check if the sensor is actively sending data
	std::array<unsigned char, 36> recv_buf{'\0'};
	asio::ip::udp::endpoint sender_endpoint;

	socket_.async_receive_from(asio::buffer(recv_buf), sender_endpoint,
	                           [](const asio::error_code& error, std::size_t bytes_transferred)
	                           {
	                           });
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	ft_sensor_response resp{};

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

	if (resp.ft_sequence_number == 0)
		throw ft_sensor_connection_exception();

	// Set first value here by hand.
	{
		std::lock_guard lock(current_ft_sensor_response_mutex_);
		current_ft_sensor_response_ = resp;
	}

	set_response_handler([&](const ft_sensor_response& response)
	{
		std::lock_guard lock(current_ft_sensor_response_mutex_);
		current_ft_sensor_response_ = response;
	});
	worker_ = std::thread(&schunk_ft_sensor::run, this);
}


Eigen::Vector<double, 6> schunk_ft_sensor::read_bias_from_config(const std::string& config_file) const
{
	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);

	Eigen::Vector<double, 6> bias;
	for (int i = 0; i < bias.size(); i++)
		bias[i] = config["bias"].at(i);

	return bias;
}


double schunk_ft_sensor::read_load_mass_from_config(const std::string& config_file) const
{
	std::ifstream in_stream(config_file);
	nlohmann::json config = nlohmann::json::parse(in_stream);
	return config["load_mass"].get<double>();
}


std::array<double, 6> schunk_ft_sensor::compensate_tool_wrench(
	const ft_sensor_response& current_ft,
	const Eigen::Matrix3d& inv_rot,
	const Eigen::Matrix<double, 6, 1>& velocity, 
	const Eigen::Matrix<double, 6, 1>& acceleration) const
{
	std::array<double, 6> ft_measured = current_ft.data;
	const Eigen::Matrix<double, 6, 1> ft_vec = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(ft_measured.data());

	const Eigen::Matrix3d R = inv_rot.transpose();
	const Eigen::Vector3d d = R * tool_com_;

	Eigen::Matrix3d I_world = R * tool_inertia_matrix_ * R.transpose();

	const double m = tool_mass_;
	const double d2 = d.squaredNorm();
	const Eigen::Matrix3d parallel = m * (d2 * Eigen::Matrix3d::Identity() - d * d.transpose());
	const Eigen::Matrix3d I_about_sensor = I_world + parallel;

	const Eigen::Vector3d f_meas_world = R * ft_vec.head<3>();
	const Eigen::Vector3d t_meas_world = R * ft_vec.tail<3>();

	const Eigen::Vector3d a_world = acceleration.block<3, 1>(0, 0);
	const Eigen::Vector3d omega_world = velocity.block<3, 1>(3, 0);
	const Eigen::Vector3d alpha_world = acceleration.block<3, 1>(3, 0);

	const Eigen::Vector3d a_c = a_world + alpha_world.cross(d) + omega_world.cross(omega_world.cross(d));

	const Eigen::Vector3d f_inertial = m * a_c;
	const Eigen::Vector3d tau_inertial = I_about_sensor * alpha_world
		+ omega_world.cross(I_about_sensor * omega_world)
		+ d.cross(m * a_c);

	const Eigen::Vector3d f_sensor = inv_rot * (f_meas_world - f_inertial);
	const Eigen::Vector3d t_sensor = inv_rot * (t_meas_world - tau_inertial);

	return {
		f_sensor.x(), f_sensor.y(), f_sensor.z(),
		t_sensor.x(), t_sensor.y(), t_sensor.z()
	};
}


std::array<double, 6> schunk_ft_sensor::compensate_only_tool_mass(
	const ft_sensor_response& current_ft,
	const Eigen::Matrix3d& inv_rot) const
{
	std::array<double, 6> ft_measured = current_ft.data;
	const Eigen::Matrix<double, 6, 1> ft_vec =
		Eigen::Map<const Eigen::Matrix<double, 6, 1>>(ft_measured.data());

	const Eigen::Matrix3d R = inv_rot.transpose();
	const Eigen::Vector3d d = R * tool_com_;

	const Eigen::Vector3d f_meas_world = R * ft_vec.head<3>();
	const Eigen::Vector3d t_meas_world = R * ft_vec.tail<3>();

	const double m = tool_mass_;
	const Eigen::Vector3d f_grav = m * grav_;
	const Eigen::Vector3d tau_grav = d.cross(f_grav);

	const Eigen::Vector3d f_sensor = inv_rot * (f_meas_world - f_grav);
	const Eigen::Vector3d t_sensor = inv_rot * (t_meas_world - tau_grav);

	return {
		f_sensor.x(), f_sensor.y(), f_sensor.z(),
		t_sensor.x(), t_sensor.y(), t_sensor.z()
	};
}
} /* namespace franka_proxy */
