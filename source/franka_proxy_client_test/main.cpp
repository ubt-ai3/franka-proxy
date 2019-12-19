#include <iostream>

#include <atomic>

#include <viral_core/log.hpp>
#include <viral_core/ms_network.hpp>
#include <viral_core/thread_util.hpp>

#include "franka_proxy_client/exception.hpp"
#include "franka_proxy_client/franka_remote_controller.hpp"
#include "FftComplex.hpp"
#include <random>
#include <fstream>
#include <Eigen/Core>
#include "franka_control/franka_util.hpp"
#include "viral_core/geo_util.hpp"


void print(const franka_proxy::robot_config_7dof& config)
{
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}


void print_status(const franka_proxy::franka_remote_controller& controller)
{
	std::cout << "POS: ";
	auto config = controller.current_config();
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}


double dist_squared(const franka_proxy::robot_config_7dof& c1, const franka_proxy::robot_config_7dof& c2)
{
	double sum = 0;
	for (int i = 0; i < 7; ++i)
		sum += (c1[i] - c2[i]) * (c1[i] - c2[i]);
	return sum;
}


template <class Function>
void execute_retry(Function&& f, franka_proxy::franka_remote_controller& controller)
{
	bool finished = false;
	while (!finished)
	{
		try
		{
			f();
			finished = true;
		}
		catch (const franka_proxy::control_exception&)
		{
			// for some reason, automatic error recovery
			// is only possible after waiting some time...
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			controller.automatic_error_recovery();
		}
		catch (const franka_proxy::command_exception&)
		{
			LOG_INFO("Encountered command exception. Probably because of wrong working mode. Waiting before retry.")
			viral_core::thread_util::sleep_seconds(1);
		}
	}
}


std::vector<std::array<double, 7>> roll_mean(
	const std::vector<std::array<double, 7>>& signal, 
	int interval)
{
	std::vector<std::array<double, 7>> result(signal.size());

	for (int i = 0; i < signal.size(); ++i)
	{
		std::array<double, 7> val{};

		for (int d = 0; d < 7; ++d)
		{
			int points = 0;
			for (int j = i - interval/2; j < i + interval/2 && j < static_cast<int>(signal.size()); ++j)
			{
				if (j < 0)
					continue;

				val[d] += signal[j][d];
				++points;
			}
			val[d] /= points;
		}

		result[i] = val;
	}

	return result;
}


// Return RC low-pass filter output samples, given input samples,
// time interval dt, and time constant RC
std::vector<std::array<double, 7>> lowpass(
	const std::vector<std::array<double, 7>>& signal,
	double dt,
	double RC)
{
	double a = dt / (RC + dt);

	std::vector<std::array<double, 7>> result(signal.size());
	result[0] = signal[0];

	for (int d = 0; d < 7; ++d)
		for (int i = 1; i < signal.size(); ++i)
			result[i][d] = a * signal[i][d] + (1. - a) * result[i - 1][d];

	return result;
}


std::vector<std::string> get_next_line_and_split_into_cells(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ','))
    {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}


int main()
{
	//std::default_random_engine randGen((std::random_device())());
	//std::uniform_real_distribution<double> valueDist(-1.0, 1.0);
	//std::vector<std::complex<double>> result, save, trans;

	//for (int i = 0; i < 1024; i++)
	//	result.emplace_back
	//	((std::sin(static_cast<double>(i) * 2. * M_PI / 1024.) 
	//		+ 0.5 * std::cos((static_cast<double>(i) + 1024./4.) * 4. *  M_PI / 1024.) + valueDist(randGen) /*+ i * 0.02*/), 0.);

	//save = result;

	//Fft::transform(result);

	//trans = result;

	//std::vector<std::complex<double>> test(1024, std::complex<double>(0.,0.));
	//test[0] = result[0];
	//int n = 100;
	//for (int i = 0; i < n; ++i)
	//{
	//	*(test.begin() + 1 + i) = *(result.begin() + 1 + i); 
	//	*(test.rbegin() + i) = *(result.rbegin() + i); 
	//}

	//Fft::inverseTransform(result);
	//Fft::inverseTransform(test);

	//{
	//	std::ofstream csv;
	//	csv.open("test123.csv");
	//	csv << "test,res\n";
	//	for (int i = 0; i < 1024; ++i)
	//		csv << test[i].real()/1024. << "," << save[i].real() << "\n";
	//	csv.close();
	//}


	//std::vector<std::array<double, 7>> filter_test2;
	//for (int i = 0; i < 1024; i++)
	//	filter_test2.emplace_back
	//	(std::array<double, 7>{
	//		(std::sin(static_cast<double>(i) * 2. * M_PI / 1024.)
	//			+ 0.5 * std::cos((static_cast<double>(i) + 1024. / 4.) * 4. * M_PI / 1024.)
	//			+ valueDist(randGen) 
	//			+ i * 0.02),
	//		0.,
	//		0.,
	//		0.,
	//		0.,
	//		0.,
	//		0.
	//	});

	//auto filter_test2_filtered = lowpass(filter_test2, 0.001, 0.05);
	//{
	//	std::ofstream csv;
	//	csv.open("lowpass.csv");
	//	csv << "test,filtered\n";
	//	for (int i = 0; i < 1024; ++i)
	//		csv << filter_test2[i][0] << "," << filter_test2_filtered[i][0] << "\n";
	//	csv.close();
	//}
	//
	//auto filter_test3_filtered = roll_mean(filter_test2, 200);
	//{
	//	std::ofstream csv;
	//	csv.open("roll_mean.csv");
	//	csv << "test,filtered\n";
	//	for (int i = 0; i < 1024; ++i)
	//		csv << filter_test2[i][0] << "," << filter_test3_filtered[i][0] << "\n";
	//	csv.close();
	//}


	viral_core::ms_network_context network("network");
	//franka_proxy::franka_remote_controller controller("127.0.0.1", network);
	franka_proxy::franka_remote_controller controller("132.180.194.141", network);

	// status test

	std::atomic_bool stop(false);
	std::thread t([&stop, &controller]()
	{
		int i = 0;
		while (!stop)
		{
			controller.update();

			if (i++ % 30 == 0)
				print_status(controller);

			viral_core::thread_util::sleep_seconds(0.016f);
		}
	});


	LOG_INFO("Starting Gripper Test.");

	controller.grasp_gripper();
	//controller.open_gripper();
	//controller.close_gripper();
	//controller.open_gripper();

	LOG_INFO("Finished Gripper Test.");

	
	LOG_INFO("Starting PTP-Movement Test.");

	//franka_proxy::robot_config_7dof pos1
	//	{{2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471}};
	//franka_proxy::robot_config_7dof pos2
	//	{{-0.00242, 1.236293, 2.465417, -1.26485, -0.00181, 1.914142, -1.06326}};

	//controller.set_speed_factor(0.2);
	//execute_retry([&] { controller.move_to(pos1); }, controller);
	//execute_retry([&] { controller.move_to(pos2); }, controller);

	LOG_INFO("Finished PTP-Movement Test.");

	
	LOG_INFO("Starting Force Test.");

	//franka_proxy::robot_config_7dof pos_with_scale
	//	{{1.09452, 0.475923, 0.206959, -2.33289, -0.289467, 2.7587, 0.830083}};
	//franka_proxy::robot_config_7dof pos_above_table
	//	{{1.09703, 0.505084, 0.216472, -2.29691, -0.302112, 2.72655, 0.817159}};
	//	//{{1.10689, 0.660073, 0.240198, -2.03228, -0.33317, 2.63551, 0.784704}};

	//controller.set_speed_factor(0.2);
	//controller.move_to(pos_with_scale);
	//controller.move_to_until_contact(pos_above_table);

	//controller.apply_z_force(0.0, 5.0);
	//controller.apply_z_force(1.0, 5.0);

	
	LOG_INFO("Finished Force Test.");


	LOG_INFO("Starting Playback Test.");
	

	std::vector<std::array<double, 7>> record;

	Eigen::Affine3d pose
		(franka_control::franka_util::fk(
		(franka_control::robot_config_7dof() 
			<< 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished()).back());

	pose.linear() << 0.707107, 0.707107, 0,
		0.707107, - 0.707107, -0,
		0, 0, -1; 

	auto ik_solution = franka_control::franka_util::ik_fast_closest
		(pose,
		 franka_control::robot_config_7dof(controller.current_config().data()));

	franka_proxy::robot_config_7dof q{};
	Eigen::VectorXd::Map(&q[0], 7) = ik_solution;
	controller.move_to(q);


	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	LOG_INFO("$$$$$ START RECORDING $$$$$");
	controller.start_recording();

	std::this_thread::sleep_for(std::chrono::seconds(10));
	
	LOG_INFO("$$$$$ STOP RECORDING $$$$$");
	record = controller.stop_recording();

	{
		std::ofstream csv("record.csv");

		csv << "j0,j1,j2,j3,j4,j5,j6\n";
		for (int i = 0; i < record.size(); ++i)
			csv << record[i][0] << ","
				<< record[i][1] << ","
				<< record[i][2] << ","
				<< record[i][3] << ","
				<< record[i][4] << ","
				<< record[i][5] << ","
				<< record[i][6] << "\n";
	}

	//{
	//	std::ifstream csv("record.csv");

	//	auto header = get_next_line_and_split_into_cells(csv);
	//	if (header[0] != "j0")
	//		throw std::exception("wrong header");

	//	int i = 0;
	//	while (!csv.eof())
	//	{
	//		auto line = get_next_line_and_split_into_cells(csv);

	//		if (line.size() != 7)
	//			continue;
	//		++i;

	//		std::array<double, 7> joints{};
	//		for (int i = 0; i < 7; ++i)
	//			joints[i] = std::stod(line[i]);
	//			
	//		record.emplace_back(joints);
	//	}
	//	LOG_INFO(i + " lines read.")
	//}
	
	std::cin.get();

	//std::vector<std::array<double, 7>> reserve_record(record.rbegin(), record.rend());
	//record = lowpass(record, 0.001, 0.016);
	//controller.move_to({{1.0882, 0.221298, 0.241497, -2.37185, -0.155255, 2.51272, 0.717536}});
	//controller.move_to({{1.08554, 0.035344, 0.225367, -2.28033, -0.053305, 2.29159, 0.602587}});
	controller.move_to(record.front());
	controller.move_sequence(record);

	LOG_INFO("Finished Playback Test.");


	// cleanup status test
	stop = true;
	t.join();


	return 0;
}
