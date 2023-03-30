#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_controller.hpp>
//#include "franka_control/franka_util.hpp"


void print_status(const franka_proxy::franka_remote_controller& controller)
{
	std::cout << "POS: ";
	auto config = controller.current_config();
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
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
			std::cout << "Encountered command exception. Probably because of wrong working mode. Waiting before retry." << std::endl;
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1s);
		}
	}
}


int main()
{
	franka_proxy::franka_remote_controller controller("127.0.0.1");
	//franka_proxy::franka_remote_controller controller("132.180.194.112");

	// status test
	std::atomic_bool stop(false);
	std::thread t
		([&stop, &controller]()
		{
 			int i = 0;
			while (!stop)
			{
				controller.update();

				if (i++ % 30 == 0)
					print_status(controller);

				using namespace std::chrono_literals;
				std::this_thread::sleep_for(0.016s);
			}
		});


	std::cout << "Starting Gripper Test." << std::endl;

	controller.grasp_gripper(0.1);
	controller.open_gripper(0.1);
	controller.close_gripper(1);
	controller.open_gripper(1);

	std::cout << "Finished Gripper Test." << std::endl;


	std::cout << "Starting PTP-Movement Test." << std::endl;

	franka_proxy::robot_config_7dof pos1
		{{2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471}};
	franka_proxy::robot_config_7dof pos2
		{{-0.00242, 1.236293, 2.465417, -1.26485, -0.00181, 1.914142, -1.06326}};

	controller.set_speed_factor(0.2);
	execute_retry([&] { controller.move_to(pos1); }, controller);
	execute_retry([&] { controller.move_to(pos2); }, controller);

	std::cout << "Finished PTP-Movement Test." << std::endl;
	std::cout << "Starting Force Test." << std::endl;

	

	franka_proxy::robot_config_7dof pos_with_scale
		{{1.09452, 0.475923, 0.206959, -2.33289, -0.289467, 2.7587, 0.830083}};
	franka_proxy::robot_config_7dof pos_above_table
		{{1.09703, 0.505084, 0.216472, -2.29691, -0.302112, 2.72655, 0.817159}};
	//{{1.10689, 0.660073, 0.240198, -2.03228, -0.33317, 2.63551, 0.784704}};

	controller.set_speed_factor(0.2);
	controller.move_to(pos_with_scale);
	controller.move_to_until_contact(pos_above_table);

	//controller.apply_z_force(0.0, 5.0);
	//controller.apply_z_force(1.0, 5.0);
	
	std::cout << "Finished Force Test." << std::endl;

	std::cout << "Starting Impedance - Hold Position Test." << std::endl;

	controller.impedance_hold_position(60);

	std::cout << "Finished Impedance - Hold Position Test." << std::endl;


	std::cout << "Starting FK/IK Test." << std::endl;

	//Eigen::Affine3d pose
	//	(franka_control::franka_util::fk
	//	 (
	//	  (franka_control::robot_config_7dof()
	//		  << 1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723).finished()).back());

	//pose.linear() << 0.707107, 0.707107, 0,
	//	0.707107, - 0.707107, -0,
	//	0, 0, -1;

	//auto ik_solution = franka_control::franka_util::ik_fast_closest
	//	(pose,
	//	 franka_control::robot_config_7dof(controller.current_config().data()));

	//franka_proxy::robot_config_7dof q{};
	//Eigen::VectorXd::Map(&q[0], 7) = ik_solution;
	//controller.move_to(q);

	std::cout << ("Finished FK/IK Test.");


	std::cout << ("Starting Playback Test.");

	franka_proxy::robot_config_7dof q
		{{1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723}};
	controller.move_to(q);
	
	std::cout << ("--- press to start in 3s ---");
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << ("--- starting demonstration ---");
	controller.start_recording();
	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::cout << ("--- stopped demonstration ---");
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> record
		(
		 controller.stop_recording());

	std::cout << ("--- press to start reproduction in 3s ---");
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	controller.move_to(q);
	controller.move_to(record.first.front());
	const std::vector<std::array<double, 6>> selection_vectors
		(
		 record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	controller.move_sequence(record.first, record.second, selection_vectors);

	std::cout << ("Finished Playback Test.");


	// cleanup status test
	stop = true;
	t.join();


	return 0;
}
