#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>

#include <argparse/argparse.hpp>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>


enum mode{none, ple, playback, gripper, ptp, force, ermer};

void franka_proxy_client_test(const std::string& ip, mode test, std::vector<std::string>& params);


void print_status(const franka_proxy::franka_remote_interface& robot);
template <class Function> void execute_retry(
	Function&& f, franka_proxy::franka_remote_interface& robot);


// todo: motion should stop gracefully --- should do now. lienhardt
void ple_motion_record_test(franka_proxy::franka_remote_interface& robot, double speed, double duration, bool log, std::string file);
// todo: not tested on robot
[[deprecated("Revise test code before execution on real robot!")]]
void playback_test(franka_proxy::franka_remote_interface& robot);
void gripper_test(franka_proxy::franka_remote_interface& robot);
// todo: used poses may be unsafe robot
[[deprecated("Revise test code before execution on real robot!")]]
void ptp_test(franka_proxy::franka_remote_interface& robot);
// todo: used poses may be unsafe robot
[[deprecated("Revise test code before execution on real robot!")]]
void force_test(franka_proxy::franka_remote_interface& robot);
// todo: untested since bachelor thesis d. ermer; handle with care.
[[deprecated("Revise test code before execution on real robot!")]]
void impedance_admittance_ermer_ba_tests(franka_proxy::franka_remote_interface& robot);


// todo: we need a csv-motion-logger for franka-proxy and tests.
void log(std::ofstream& csv_log, std::array<double, 7> j, std::array<double, 6> ft, double time);


int main(int argc, char* argv[])
{
	argparse::ArgumentParser program("franka_client_test");

	program.add_argument("-ip")
		.help("specify IP for franka-proxy remote connection")
		.default_value(std::string{ "127.0.0.1" });


	argparse::ArgumentParser base("fct_base", "1.0", argparse::default_arguments::none);
	
	base.add_argument("-l", "-log")
		.help("enable logging")
		.default_value("false")
		.implicit_value("true");

	base.add_argument("-f", "-file")
		.help("specify file to write log into")
		.default_value("franka_client_test_log.csv");


	argparse::ArgumentParser ple_test("ple");

	ple_test.add_argument("speed")
		.help("specify speed for ple motion")
		.default_value("0.3");

	ple_test.add_argument("-d", "-duration")
		.help("specify duration for ple motion")
		.default_value("10.0");

	ple_test.add_parents(base);


	program.add_subparser(ple_test);

	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		std::cerr << program;
		return -1;
	}

	const auto ip = program.get<std::string>("-ip");
	std::cout <<
		"--------------------------------------------------------------------------------\n"
		"Executing franka client test with IP " << ip << ": " << std::endl;
	//std::string ip("132.180.194.112"); // franka1-proxy@resy-lab

	mode test = mode::none;
	std::vector<std::string> params;
	
	if (ple_test) {
		std::cout << "Running PLE motion record test..." << std::endl;

		std::string speed = ple_test.get<std::string>("speed");
		std::string duration = ple_test.get<std::string>("-d");
		std::string log = ple_test.get<std::string>("-l");
		std::string file = ple_test.get<std::string>("-f");

		params.push_back(speed);
		params.push_back(duration);
		params.push_back(log);
		params.push_back(file);

		test = mode::ple;
	}
	

	//todo: add cases for other tests


	franka_proxy_client_test(ip, test, params);

	std::cout << "Press Enter to end test exe." << std::endl;
	std::cin.get();
	return 0;
}


void franka_proxy_client_test(const std::string& ip, mode test, std::vector<std::string>& params)
{
	std::unique_ptr<franka_proxy::franka_remote_interface> robot;
	try
	{
		robot =
			std::make_unique<franka_proxy::franka_remote_interface>(ip);
	}
	catch (const std::exception&)
	{
		std::cerr << "Could not connect to franka-proxy with IP " << ip << "." << std::endl;
		return;
	}

	// --- mandatory status thread with debug output ---
	int print_every_ith_status = 30;
	std::atomic_bool stop(false);
	std::thread t([&stop, &robot, print_every_ith_status]()
	{
		int i = 0;
		while (!stop)
		{
			robot->update();

			if (print_every_ith_status && i++ % print_every_ith_status == 0)
				print_status(*robot);

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(0.016s);
		}
	});

	// --- franka motion tests ---
	//gripper_test(robot);
	//ptp_test(robot);
	//force_test(robot); 
	//impedance_admittance_ermer_ba_tests(robot);
	//playback_test(robot);

	if (test == ple) {
		double speed = stod(params[0]);
		double duration = stod(params[1]);
		bool log = (params[2] == "true");
		std::string file = params[3];
		ple_motion_record_test(*robot, speed, duration, log, file);

		std::cout << "PLE motion record test finished." << std::endl;
	}
	
	// --- cleanup status thread ---
	stop = true;
	t.join();
}


void print_status(const franka_proxy::franka_remote_interface& robot)
{
	const auto config = robot.current_config();

	std::cout << "Current robot joints: ";
	for (int i = 0; i < 6; ++i)
		std::cout << config[i] << ", ";
	std::cout << config[6] << std::endl;
}


template <class Function> void execute_retry(
	Function&& f, franka_proxy::franka_remote_interface& robot)
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
			std::cout << "Encountered command exception. "
				"Probably because of wrong working mode. "
				"Waiting before retry." << std::endl;
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1s);
		}
	}
}


void ple_motion_record_test(franka_proxy::franka_remote_interface& robot, double speed, double duration, bool log, std::string file)
{
	constexpr franka_proxy::robot_config_7dof joints_start
		{0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461};
	robot.move_to(joints_start);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.ple_motion(speed, duration, log, file);
}


void playback_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << ("Starting Playback Test.") << std::endl;

	std::cout << ("--- press to start in 3s ---") << std::endl;
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << ("--- starting demonstration ---") << std::endl;
	robot.start_recording();
	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::cout << ("--- stopped demonstration ---") << std::endl;
	const std::pair record(robot.stop_recording());

	std::cout << ("--- press to start reproduction in 3s ---") << std::endl;
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));


	robot.move_to(record.first.front());
	const std::vector selection_vectors(
		record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	robot.move_sequence(record.first, record.second, selection_vectors);

	std::cout << ("Finished Playback Test.");

	std::ofstream csv_log;
	csv_log.open("hand_guided_log.csv");
	const std::string csv_header_ =
		"joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7,"
		"force_x,force_y,force_z,torque_x,torque_y,torque_z,time";
	csv_log << csv_header_ << "\n";
	for (size_t i = 0; i < record.first.size(); i++)
		log(csv_log, record.first.at(i), record.second.at(i), 0.001 * static_cast<double>(i));
}

void log(std::ofstream& csv_log, std::array<double, 7> j, std::array<double, 6> ft, double time)
{
	std::ostringstream j_log;
	j_log << j[0] << "," << j[1] << "," << j[2] << "," << j[3]
		<< "," << j[4] << "," << j[5] << "," << j[6];
	std::ostringstream ft_log;
	ft_log << ft[0] << "," << ft[1] << "," << ft[2] << "," << ft[3] << "," << ft[4] << "," << ft[5];

	double sum = ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2];
	std::ostringstream current_values;
	current_values << j_log.str() << "," << ft_log.str() << "," << time << "," << sum;

	csv_log << current_values.str() << "\n";
}


void gripper_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting Gripper Test." << std::endl;

	robot.grasp_gripper(0.1);
	robot.open_gripper(0.1);
	robot.close_gripper(1);
	robot.open_gripper(1);

	std::cout << "Finished Gripper Test." << std::endl;
}


void ptp_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting PTP-Movement Test." << std::endl;

	constexpr franka_proxy::robot_config_7dof pos1
		{2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471};
	constexpr franka_proxy::robot_config_7dof pos2
		{-0.00242, 1.236293, 2.465417, -1.26485, -0.00181, 1.914142, -1.06326};

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(pos1); }, robot);
	execute_retry([&] { robot.move_to(pos2); }, robot);

	std::cout << "Finished PTP-Movement Test." << std::endl;
}


void force_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting Force Test." << std::endl;

	//franka_proxy::robot_config_7dof pos_with_scale
	//	{{1.09452, 0.475923, 0.206959, -2.33289, -0.289467, 2.7587, 0.830083}};
	//franka_proxy::robot_config_7dof pos_above_table
	//	{{1.09703, 0.505084, 0.216472, -2.29691, -0.302112, 2.72655, 0.817159}};
	////{{1.10689, 0.660073, 0.240198, -2.03228, -0.33317, 2.63551, 0.784704}};

	//robot.set_speed_factor(0.2);
	//robot.move_to(pos_with_scale);
	//robot.move_to_until_contact(pos_above_table);

	////robot.apply_z_force(0.0, 5.0);
	////robot.apply_z_force(1.0, 5.0);
	
	std::cout << "Finished Force Test." << std::endl;
}


void impedance_admittance_ermer_ba_tests(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting Impedance - Hold Position Test." << std::endl;

	std::array<double, 49> stiffness{0.};
	stiffness[0] = stiffness[8] = stiffness[16] = stiffness[24] = 600.; // first four joints
	stiffness[32] = 250.;
	stiffness[40] = 150.;
	stiffness[48] = 50.;
	robot.joint_impedance_hold_position(10, false, stiffness);

	//std::this_thread::sleep_for(std::chrono::seconds(3));
	////TODO move to start pose
	//robot.cartesian_impedance_poses(cart_test_positions, 15, false, false, 50., 300.);

	//std::cout << "Finished Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;

	std::cout << "Starting Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;
	// positions
	std::list<std::array<double, 16>> poses_for_cart = {
		{
			0.321529, 0.8236, 0.467208, 0,
			0.931889, -0.187754, -0.310343, 0,
			-0.167882, 0.53518, -0.827888, 0,
			0.426976, 0.382873, 0.324984, 1
		},
		{
			0.323711, 0.604326, 0.727999, 0,
			0.698631, -0.671549, 0.246814, 0,
			0.638056, 0.428714, -0.639601, 0,
			0.600692, 0.372768, 0.415227, 1
		},
		{
			0.826378, 0.559426, 0.0642109, 0,
			0.562775, -0.824385, -0.0604543, 0,
			0.0191152, 0.086096, -0.996103, 0,
			0.503131, 0.2928, 0.296891, 1
		}
	};

	std::cout << "Starting Joint Impedance Tests." << std::endl << "Holding position with Joint Impedance in 3s..";


	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.joint_impedance_hold_position(10, false, stiffness);

	std::cout << " .. finished." << std::endl << "Following Poses with Joint Impedance in 3s..";
	// positions
	std::list<std::array<double, 16>> poses_for_joint = {
		{
			0.321529, 0.8236, 0.467208, 0,
			0.931889, -0.187754, -0.310343, 0,
			-0.167882, 0.53518, -0.827888, 0,
			0.426976, 0.382873, 0.324984, 1
		},
		{
			0.323711, 0.604326, 0.727999, 0,
			0.698631, -0.671549, 0.246814, 0,
			0.638056, 0.428714, -0.639601, 0,
			0.600692, 0.372768, 0.415227, 1
		},
		{
			0.826378, 0.559426, 0.0642109, 0,
			0.562775, -0.824385, -0.0604543, 0,
			0.0191152, 0.086096, -0.996103, 0,
			0.503131, 0.2928, 0.296891, 1
		}
	};

	robot.cartesian_impedance_poses(poses_for_joint, 10, false, false, 50., 300.);

	std::cout << "Finished Impedance - Follow Poses with Joint Impedance Test." << std::endl;


	std::cout << "Starting Admittance - Apply Force Test." << std::endl;

	robot.apply_admittance(10, true, 10., 150., 10., 150.);

	std::cout << "Finished Admittance - Apply Force Test." << std::endl;
}
