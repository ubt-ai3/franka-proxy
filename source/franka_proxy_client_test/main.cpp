#include <atomic>
#include <iostream>
#include <thread>
#include <utility>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>

#include <franka_control/payload_estimation.hpp>


void franka_proxy_client_test(const std::string& ip);
void print_status(const franka_proxy::franka_remote_interface& robot);
template <class Function> void execute_retry(Function&& f, franka_proxy::franka_remote_interface& robot);


int main()
{
	//franka_proxy_client_test("132.180.194.112");
	franka_proxy_client_test("127.0.0.1");
	return 0;
}


void franka_proxy_client_test(const std::string& ip)
{
	franka_proxy::franka_remote_interface robot(ip);
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &robot]()
		{
			int i = 0;
			while (!stop)
			{
				robot.update();

				if (i++ % 30 == 0)
					print_status(robot);

				using namespace std::chrono_literals;
				std::this_thread::sleep_for(0.016s);
			}
		});

	// PLE MOTION TEST
	franka_proxy::robot_config_7dof sp{ {0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461} };
	robot.move_to(sp);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.ple_motion(30.0, false);

	if (true) {
		stop = true;
		t.join(); 
		return; 
	}
	// PLE MOTION TEST

	// VERY DIRTY PLE TEST, REMOVE BEFORE FLIGHT
	std::array<double, 6> f1 = { 0.030176    ,-2.97307    ,10.9486     ,-0.188621    ,-0.166592    ,-0.092337 };
	std::array<double, 7> j1 = { 2.60759 ,-0.688177 ,-2.04893 ,-1.19268 ,-0.69501  ,1.64645 ,1.16381 };
	std::array<double, 6> f2 = { 0.041172    ,-2.96951    ,10.9455     ,-0.189406    ,-0.165091    ,-0.091925 };
	std::array<double, 7> j2 = { 2.60759 ,-0.688177 ,-2.04893 ,-1.19268 ,-0.69501  ,1.64645 ,1.16381 };
	std::array<double, 6> f3 = { 0.022531    ,-2.96242    ,10.9556     ,-0.190145    ,-0.165851    ,-0.091962 };
	std::array<double, 7> j3 = { 2.60758 ,-0.688178 ,-2.04893 ,-1.19268 ,-0.695011 ,1.64645 ,1.16381 };
	std::array<double, 6> f4 = { 0.08, -3.5, 13.97, -0.4, -0.2, -0.01 };
	std::array<double, 7> j4 = { 2.75, -0.8, -2.5, -1.4, -0.3, 2.2, 0.7 };
	std::array<double, 6> f5 = { 0.9, -1.8, 6.5, -0.96 , 0.0, -0.9 };
	std::array<double, 7> j5 = { 1.9, 0.0, -3.0, -0.8, 0.5, 3.7, 0.1 };

	payload_estimation::data stuff = {
		std::make_pair(std::make_pair(j1,f1),1),
		std::make_pair(std::make_pair(j2,f2),1),
		std::make_pair(std::make_pair(j3,f3),1),
		std::make_pair(std::make_pair(j4,f4),1),
		std::make_pair(std::make_pair(j5,f5),1),
	};

	payload_estimation::results rtls;
	payload_estimation::results rcer;

	payload_estimation::ple::estimate_ceres(stuff, rcer);
	std::cout << "Ceres estimates a mass of: " + std::to_string(rcer.mass) << std::endl;
	
	try {
		payload_estimation::ple::estimate_tls(stuff, rtls);
		std::cout << "TLS estimates a mass of: " + std::to_string(rtls.mass) << std::endl;
	}
	catch (std::runtime_error e)
	{
		std::cout << e.what() << std::endl;
	}

	if (true) { return; }
	// VERY DIRTY PLE TEST, REMOVE BEFORE FLIGHT



	// status test



	std::cout << "Starting Gripper Test." << std::endl;

	robot.grasp_gripper(0.1);
	robot.open_gripper(0.1);
	robot.close_gripper(1);
	robot.open_gripper(1);

	std::cout << "Finished Gripper Test." << std::endl;


	std::cout << "Starting PTP-Movement Test." << std::endl;

	franka_proxy::robot_config_7dof pos1
		{{2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471}};
	franka_proxy::robot_config_7dof pos2
		{{-0.00242, 1.236293, 2.465417, -1.26485, -0.00181, 1.914142, -1.06326}};

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(pos1); }, robot);
	execute_retry([&] { robot.move_to(pos2); }, robot);

	std::cout << "Finished PTP-Movement Test." << std::endl;
	std::cout << "Starting Force Test." << std::endl;


	franka_proxy::robot_config_7dof pos_with_scale
		{{1.09452, 0.475923, 0.206959, -2.33289, -0.289467, 2.7587, 0.830083}};
	franka_proxy::robot_config_7dof pos_above_table
		{{1.09703, 0.505084, 0.216472, -2.29691, -0.302112, 2.72655, 0.817159}};
	//{{1.10689, 0.660073, 0.240198, -2.03228, -0.33317, 2.63551, 0.784704}};

	robot.set_speed_factor(0.2);
	robot.move_to(pos_with_scale);
	robot.move_to_until_contact(pos_above_table);

	//robot.apply_z_force(0.0, 5.0);
	//robot.apply_z_force(1.0, 5.0);

	std::cout << "Finished Force Test." << std::endl;


	std::cout << "Starting Impedance - Hold Position Test." << std::endl;

	std::array<double, 49> stiffness{0.};
	stiffness[0] = stiffness[8] = stiffness[16] = stiffness[24] = 600.; // first four joints
	stiffness[32] = 250.; 
	stiffness[40] = 150.; 
	stiffness[48] = 50.; 
	robot.joint_impedance_hold_position(10, false, stiffness);

	std::cout << "Finished Impedance - Hold Position Test." << std::endl;


	std::cout << "Starting Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;
	// positions
	std::list<std::array<double, 16>> poses_for_cart = {
		{0.321529, 0.8236, 0.467208, 0, 0.931889, -0.187754, -0.310343, 0, -0.167882, 0.53518, -0.827888, 0, 0.426976, 0.382873, 0.324984, 1},
		{0.323711, 0.604326, 0.727999, 0, 0.698631, -0.671549, 0.246814, 0, 0.638056, 0.428714, -0.639601, 0, 0.600692, 0.372768, 0.415227, 1},
		{0.826378, 0.559426, 0.0642109, 0, 0.562775, -0.824385, -0.0604543, 0, 0.0191152, 0.086096, -0.996103, 0, 0.503131, 0.2928, 0.296891, 1}
	};

	robot.cartesian_impedance_poses(poses_for_cart, 10, false, false, 50., 300.);

	std::cout << "Finished Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;


	std::cout << "Starting Impedance - Follow Poses with Joint Impedance Test." << std::endl;
	// positions
	std::list<std::array<double, 16>> poses_for_joint = {
		{0.321529, 0.8236, 0.467208, 0, 0.931889, -0.187754, -0.310343, 0, -0.167882, 0.53518, -0.827888, 0, 0.426976, 0.382873, 0.324984, 1},
		{0.323711, 0.604326, 0.727999, 0, 0.698631, -0.671549, 0.246814, 0, 0.638056, 0.428714, -0.639601, 0, 0.600692, 0.372768, 0.415227, 1},
		{0.826378, 0.559426, 0.0642109, 0, 0.562775, -0.824385, -0.0604543, 0, 0.0191152, 0.086096, -0.996103, 0, 0.503131, 0.2928, 0.296891, 1}
	};

	robot.cartesian_impedance_poses(poses_for_joint, 10, false, false, 50., 300.);

	std::cout << "Finished Impedance - Follow Poses with Joint Impedance Test." << std::endl;


	std::cout << "Starting Admittance - Apply Force Test." << std::endl;

	robot.apply_admittance(10, true, 10., 150., 10., 150.);

	std::cout << "Finished Admittance - Apply Force Test." << std::endl;


	//std::cout << "Starting FK/IK Test." << std::endl;

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
	//	 franka_control::robot_config_7dof(robot.current_config().data()));

	//franka_proxy::robot_config_7dof q{};
	//Eigen::VectorXd::Map(&q[0], 7) = ik_solution;
	//robot.move_to(q);

	//std::cout << ("Finished FK/IK Test.");


	std::cout << ("Starting Playback Test.");

	franka_proxy::robot_config_7dof q
		{{1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723}};
	robot.move_to(q);

	std::cout << ("--- press to start in 3s ---");
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << ("--- starting demonstration ---");
	robot.start_recording();
	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::cout << ("--- stopped demonstration ---");
	std::pair<std::vector<std::array<double, 7>>, std::vector<std::array<double, 6>>> record
	(
		robot.stop_recording());

	std::cout << ("--- press to start reproduction in 3s ---");
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	robot.move_to(q);
	robot.move_to(record.first.front());
	const std::vector<std::array<double, 6>> selection_vectors
	(
		record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	robot.move_sequence(record.first, record.second, selection_vectors);

	std::cout << ("Finished Playback Test.");


	// cleanup status test
	stop = true;
	t.join();
}


void print_status(const franka_proxy::franka_remote_interface& robot)
{
	std::cout << "POS: ";
	auto config = robot.current_config();
	for (int i = 0; i < 7; ++i)
		std::cout << config[i] << " ";

	std::cout << std::endl;
}


template <class Function> void execute_retry(Function&& f, franka_proxy::franka_remote_interface& robot)
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
			robot.automatic_error_recovery();
		}
		catch (const franka_proxy::command_exception&)
		{
			std::cout << "Encountered command exception. Probably because of wrong working mode. Waiting before retry."
				<< std::endl;
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1s);
		}
	}
}
