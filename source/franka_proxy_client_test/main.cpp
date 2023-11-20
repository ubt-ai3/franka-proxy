#include <atomic>
#include <iostream>
#include <thread>
#include <utility>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>

#include <franka_control/payload_estimation.hpp>
#include <fstream>

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
	//PLE CALCULATION TEST
	std::cout << "Starting PLE calculation test" << std::endl;

	std::string infile = "ple_log_gripper.csv";
	payload_estimation::data indata = payload_estimation::ple::read_from_csv(infile);
	int samples = indata.size();
	auto front = indata.begin();
	auto back = front;
	std::cout << "Total number of samples: " << samples << std::endl;

	std::cout << "Preparing data sets for testing...";
	std::vector<payload_estimation::data> sets;
	int small = 0;
	int medium = 0;
	for (int i = 0; i < samples - 200; i = i + 200) {
		front = back;
		back += 200;
		payload_estimation::data set(front, back);
		sets.push_back(set);
		small++;
	}
	front = indata.begin();
	back = front;
	for (int i = 0; i < samples - 1000; i = i + 1000) {
		front = back;
		back += 1000;
		payload_estimation::data set(front, back);
		sets.push_back(set);
		medium++;
	}
	sets.push_back(indata);
	std::cout << "Done" << std::endl;

	std::string outfile = "ple_test.csv";
	std::string header = "mode,samples,steps,mass,com_x,com_y,com_z,i_xx,i_xy,i_xz,i_yy,i_yz,i_zz,time";
	std::ofstream logger(outfile, std::ofstream::out);
	logger << header << "\n";

	payload_estimation::results res;
	
	std::cout << "Calculating PLE solutions for " << small << " small data sets (200 samples), " << medium << " medium data sets (1000 samples), and the full data set" << std::endl;
	for (int i = 0; i < sets.size(); i++) {
		if (i < small) {
			std::cout << "Small data set " << (i + 1) << " of " << small << "..." << std::endl;
		}
		else if (i < (small + medium)) {
			std::cout << "Medium data set " << (i - small + 1) << " of " << medium << "..." << std::endl;
		}
		else {
			std::cout << "Full data set..." << std::endl;
		}

		std::cout << "Exact TLS..." << std::endl;

		auto t0 = std::chrono::high_resolution_clock::now();
		try {
			res = payload_estimation::ple::estimate_tls(sets[i], false, 1);
		}
		catch (std::runtime_error e) {
			std::cout << e.what() << std::endl;
		}
		auto t1 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> time = t1 - t0;
		logger << "exact," << sets[i].size() << ",1," << res.mass << "," << res.com(0) << "," << res.com(1) << "," << res.com(2) << "," << res.inertia.coeff(0, 0) << "," << res.inertia.coeff(0, 1) << "," << res.inertia.coeff(0, 2) << "," << res.inertia.coeff(1, 1) << "," << res.inertia.coeff(1, 2) << "," << res.inertia.coeff(2, 2) << "," << time.count() << "\n";

		std::cout << "Time taken: " << time.count() << std::endl;
		
		std::cout << "Fast TLS..." << std::endl;

		t0 = std::chrono::high_resolution_clock::now();
		try {
			res = payload_estimation::ple::estimate_tls(sets[i], true, 1);
		}
		catch (std::runtime_error e) {
			std::cout << e.what() << std::endl;
		}
		t1 = std::chrono::high_resolution_clock::now();
		time = t1 - t0;
		logger << "fast," << sets[i].size() << ",1," << res.mass << "," << res.com(0) << "," << res.com(1) << "," << res.com(2) << "," << res.inertia.coeff(0, 0) << "," << res.inertia.coeff(0, 1) << "," << res.inertia.coeff(0, 2) << "," << res.inertia.coeff(1, 1) << "," << res.inertia.coeff(1, 2) << "," << res.inertia.coeff(2, 2) << "," << time.count() << "\n";

		std::cout << "Time taken: " << time.count() << std::endl;

		std::cout << "Ceres..." << std::endl;

		t0 = std::chrono::high_resolution_clock::now();
		res = payload_estimation::ple::estimate_ceres(sets[i]);
		t1 = std::chrono::high_resolution_clock::now();
		time = t1 - t0;
		logger << "ceres," << sets[i].size() << ",1," << res.mass << "," << res.com(0) << "," << res.com(1) << "," << res.com(2) << "," << res.inertia.coeff(0, 0) << "," << res.inertia.coeff(0, 1) << "," << res.inertia.coeff(0, 2) << "," << res.inertia.coeff(1, 1) << "," << res.inertia.coeff(1, 2) << "," << res.inertia.coeff(2, 2) << "," << time.count() << "\n";

		std::cout << "Time taken: " << time.count() << std::endl;
	}
	
	if (true) { return; }
	//PLE CALCULATION TEST



	franka_proxy::franka_remote_interface robot(ip);
	std::atomic_bool stop(false);
	std::thread t
	([&stop, &robot]()
		{
			int i = 0;
			while (!stop)
			{
				robot.update();

				/**
				if (i++ % 30 == 0)
					print_status(robot);
				**/

				using namespace std::chrono_literals;
				std::this_thread::sleep_for(0.016s);
			}
		});

	// PLE TEST
	franka_proxy::robot_config_7dof sp{ {0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461} };
	robot.move_to(sp);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.ple_motion(10.0, true);

	std::string filename = "ple_log.csv";
	payload_estimation::data input = payload_estimation::ple::read_from_csv(filename);
	std::cout << "Number of data points:" << input.size() << std::endl;

	payload_estimation::results rcer = payload_estimation::ple::estimate_ceres(input);
	std::cout << "Ceres estimates a mass of: " + std::to_string(rcer.mass) << std::endl;
	
	/**
	payload_estimation::results rtls;
	try {
		rtls = payload_estimation::ple::estimate_tls(input, false, 1);
		std::cout << "TLS estimates a mass of: " + std::to_string(rtls.mass) << std::endl;
	}
	catch (std::runtime_error e)
	{
		std::cout << e.what() << std::endl;
	}
	**/

	if (true) {
		stop = true;
		t.join();
		return;
	}
	
	// PLE TEST



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
