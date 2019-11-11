#include <iostream>

#include <atomic>

#include <viral_core/log.hpp>
#include <viral_core/ms_network.hpp>
#include <viral_core/thread_util.hpp>

#include "franka_proxy_client/exception.hpp"
#include "franka_proxy_client/franka_remote_controller.hpp"


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


void smooth_record(std::vector<std::array<double, 7>>& record, int interval)
{
	for (int i = 0; i < record.size(); ++i)
	{
		std::array<double, 7> result{};

		for (int d = 0; d < 7; ++d)
		{
			int points = 0;
			for (int j = i; j < i + interval && j < record.size(); ++j)
			{
				result[d] += record[j][d];
				++points;
			}
			result[d] /= points;
		}

		record[i] = result;
	}
}


int main()
{
	viral_core::ms_network_context network("network");
	//franka_proxy::franka_remote_controller controller("127.0.0.1", network);
	franka_proxy::franka_remote_controller controller("132.180.194.141", network);

	std::atomic_bool stop(false);
	std::thread t([&stop, &controller]()
	{
		int i = 0;
		while (!stop)
		{
			controller.update();

			if (i++ % 30 == 0)
				print_status(controller);

			std::this_thread::sleep_for(std::chrono::milliseconds(16));
		}
	});

	// gripper test

	//execute_retry([&] { controller.open_gripper(); }, controller);
	//execute_retry([&] { controller.close_gripper(); }, controller);
	//execute_retry([&] { controller.close_gripper(); }, controller);

	//std::this_thread::sleep_for(std::chrono::seconds(5));

	//execute_retry([&] { controller.open_gripper(); }, controller);
	//execute_retry([&] { controller.grasp_gripper(); }, controller);
	//execute_retry([&] { controller.grasp_gripper(); }, controller);

	//std::this_thread::sleep_for(std::chrono::seconds(5));

	//execute_retry([&] { controller.open_gripper(); }, controller);
	//execute_retry([&] { controller.open_gripper(); }, controller);

	LOG_INFO("Finished Gripper Test");


	// motion test

	//franka_proxy::robot_config_7dof pos1{
	//	{
	//		2.4673210167983628,
	//		-1.053636035616098,
	//		-0.935180716967433,
	//		-1.670424119447617,
	//		0.1367540113528485,
	//		1.4206203791091001,
	//		0.3347107737734215
	//	}
	//};
	//franka_proxy::robot_config_7dof pos2{
	//	{
	//		-0.002421978837257,
	//		1.2362939888338829,
	//		2.4654171861844083,
	//		-1.264853222554674,
	//		-0.001813626296555,
	//		1.9141426016730037,
	//		-1.063268839608126
	//	}
	//};

	//controller.set_speed_factor(0.25);
	//execute_retry([&] { controller.move_to(pos1); }, controller);
	//execute_retry([&] { controller.move_to(pos2); }, controller);

	LOG_INFO("Finished ptp-Movement Test");


	// playback test

	std::this_thread::sleep_for(std::chrono::seconds(5));
	
	LOG_INFO("$$$$$ START RECORDING $$$$$");
	controller.start_recording();

	std::this_thread::sleep_for(std::chrono::seconds(10));
	
	LOG_INFO("$$$$$ STOP RECORDING $$$$$");
	std::vector<std::array<double, 7>> record(controller.stop_recording());

	std::cin.get();

	smooth_record(record, 200);

	controller.move_to(record.front());
	controller.move_sequence(record);

	LOG_INFO("Finished Playback Test");


	stop = true;
	t.join();
}
