#include <atomic>
#include <iostream>
#include <optional>
#include <thread>

#include <argparse/argparse.hpp>

#include <franka_proxy_share/franka_proxy_logger.hpp>
#include <franka_proxy_client/forward.hpp> // included for IDE visibility of the forward header
#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>


// use this to specify which test to run within the franka_proxy_client_test method
// for new tests, add a new, distinct and sensible entry here, a new subparser to the main method,
// and a new case to franka_proxy_client_test
enum test_mode { none, ple, playback, gripper, ptp, force, vacuum };


// general purpose starting position, centered above table with some good space in all directions
constexpr franka_proxy::robot_config_7dof starting_pos
	{0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461};


void franka_proxy_client_test(
	const std::string& ip,
	test_mode test,
	const std::vector<std::string>& params);


void print_status(const franka_proxy::franka_remote_interface& robot);
template <class Function> void execute_retry(
	Function f, 
	franka_proxy::franka_remote_interface& robot);


void ple_motion_record_test(
	franka_proxy::franka_remote_interface& robot, double speed, double duration, bool log, const std::string& file);
void ptp_test(
	franka_proxy::franka_remote_interface& robot, double margin, bool log, const std::string& file);
void gripper_test(
	franka_proxy::franka_remote_interface& robot, double margin, bool grasp);
void playback_test(
	franka_proxy::franka_remote_interface& robot, bool log, const std::string& file);
void force_test(
	franka_proxy::franka_remote_interface& robot, double mass, double duration);
void vacuum_test(
	franka_proxy::franka_remote_interface& robot);
void guiding_test(
	franka_proxy::franka_remote_interface& robot);


double relative_config_error(
	const franka_proxy::robot_config_7dof& desired_config,
	const franka_proxy::robot_config_7dof& current_config);


int main(int argc, char* argv[])
{
	// this is the main parser, which will only handle specifying an IP and calling individual subparsers
	argparse::ArgumentParser program("franka_client_test");
	program.add_argument("-ip")
	       .help("specify IP for franka-proxy remote connection")
	       .default_value(std::string{"127.0.0.1"});


	// this is a base parser, including optional arguments for logging; use as parent for all subparsers that need it
	argparse::ArgumentParser base("fct_base", "1.0", argparse::default_arguments::none);
	base.add_argument("-l", "-log")
	    .help("enable logging")
	    .flag();
	base.add_argument("-f", "-file")
	    .help("specify file to write log into")
	    .default_value("franka_client_test_log.csv");


	// subparsers for individual tests
	argparse::ArgumentParser ple_test("ple");
	ple_test.add_argument("speed")
	        .help("specify speed factor for ple motion")
	        .default_value("0.3");
	ple_test.add_argument("-d", "-duration")
	        .help("specify duration (sec) for ple motion")
	        .default_value("10.0");
	ple_test.add_parents(base);


	argparse::ArgumentParser gripper_test("gripper");
	gripper_test.add_argument("margin")
	            .help(
		            "specify margin below which gripper is considered closed/grasped (useful for different gripper configs)")
	            .default_value("0.01");
	gripper_test.add_argument("-g")
	            .help("enable grasping of a provided object (gripper will close normally otherwise)")
	            .flag();
	// logs nothing


	argparse::ArgumentParser ptp_test("ptp");
	ptp_test.add_argument("margin")
	        .help("specify margin below which relative pose error is ignored and pose is considered reached")
	        .default_value("0.1");
	ptp_test.add_parents(base);


	argparse::ArgumentParser force_test("force");
	force_test.add_argument("-m")
	          .help("specify mass (kg) with which to press downward")
	          .default_value("0.5");
	force_test.add_argument("-d")
	          .help("specify duration (sec) for which to press downward")
	          .default_value("10.0");
	// logs nothing

	argparse::ArgumentParser vacuum_test("vacuum");


	argparse::ArgumentParser playback_test("playback");
	// has no further arguments
	playback_test.add_parents(base);


	program.add_subparser(vacuum_test);
	program.add_subparser(playback_test);
	program.add_subparser(force_test);
	program.add_subparser(ptp_test);
	program.add_subparser(gripper_test);
	program.add_subparser(ple_test);


	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cerr << program;
		return -1;
	}


	const auto ip = program.get<std::string>("-ip");
	std::cout <<
		"--------------------------------------------------------------------------------\n"
		"Executing franka client test with IP " << ip << ": " << '\n';


	test_mode test = none;
	std::vector<std::string> params;

	// case distinction for individual tests / subparsers, remember to set "test" to the corresponding mode
	// this is where input arguments are handled, which will be passed to franka_proxy_client_test as a vector of strings
	// note that only one test can be run at the same time, so add new tests with an "else if" block
	if (program.is_subcommand_used(ple_test))
	{
		test = ple;

		auto speed = ple_test.get<std::string>("speed");
		auto duration = ple_test.get<std::string>("-d");
		bool log_flag = ple_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		auto file = ple_test.get<std::string>("-f");

		params.push_back(speed);
		params.push_back(duration);
		params.push_back(log);
		params.push_back(file);
	}
	else if (program.is_subcommand_used(gripper_test))
	{
		test = gripper;

		auto margin = gripper_test.get<std::string>("margin");
		bool grasp_flag = gripper_test.get<bool>("-g");
		std::string grasp("false");
		if (grasp_flag) grasp = "true";

		params.push_back(margin);
		params.push_back(grasp);
	}
	else if (program.is_subcommand_used(ptp_test))
	{
		test = ptp;

		auto margin = ptp_test.get<std::string>("margin");
		bool log_flag = ptp_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		auto file = ptp_test.get<std::string>("-f");

		params.push_back(margin);
		params.push_back(log);
		params.push_back(file);
	}
	else if (program.is_subcommand_used(force_test))
	{
		test = force;

		auto mass = force_test.get<std::string>("-m");
		auto duration = force_test.get<std::string>("-d");

		params.push_back(mass);
		params.push_back(duration);
	}
	else if (program.is_subcommand_used(playback_test))
	{
		test = playback;

		bool log_flag = playback_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		auto file = playback_test.get<std::string>("-f");

		params.push_back(log);
		params.push_back(file);
	}
	else if (program.is_subcommand_used(vacuum_test))
	{
		test = vacuum;
		std::cout << "Vacuum gripper tests currently not implemented.";
	}


	franka_proxy_client_test(ip, test, params);
	std::cout << "Press Enter to end test exe." << '\n';
	std::cin.get();
	return 0;
}

// todo PLi: What is this second nesting of tests for if only one can be called at a time?
void franka_proxy_client_test(
	const std::string& ip,
	const test_mode test,
	const std::vector<std::string>& params)
{
	std::unique_ptr<franka_proxy::franka_remote_interface> robot;
	try
	{
		robot =
			std::make_unique<franka_proxy::franka_remote_interface>(ip);
	}
	catch (const std::exception&)
	{
		std::cerr << "Could not connect to franka-proxy with IP " << ip << "." << '\n';
		return;
	}


	// Mandatory status thread with debug output
	int print_every_ith_status = 10;
	std::atomic_bool stop(false);
	std::thread t([&stop, &robot, print_every_ith_status]()
	{
		int i = 0;
		while (!stop)
		{
			robot->update();

			if (print_every_ith_status && i % print_every_ith_status == 0)
			{
				std::cout << "Status #" << i << ": ";
				print_status(*robot);
			}
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(0.016s);

			++i;
		}
	});


	// Franka motion tests

	// case distinction for individual tests (add new tests with an "else if" block)
	// parameters for test methods arrive here as a vector of strings, so convert accordingly
	if (test == none)
	{
		using namespace std::chrono_literals;
		std::chrono::duration duration(10s);
		std::cout << "Only testing status socket for "
			<< duration << ".\n";
		std::this_thread::sleep_for(duration);
	}
	else if (test == ple)
	{
		double speed = stod(params[0]);
		double duration = stod(params[1]);
		bool log = (params[2] == "true");
		std::string file = params[3];

		ple_motion_record_test(*robot, speed, duration, log, file);
	}
	else if (test == gripper)
	{
		double margin = stod(params[0]);
		bool grasp = (params[1] == "true");

		gripper_test(*robot, margin, grasp);
	}
	else if (test == ptp)
	{
		double margin = stod(params[0]);
		bool log = (params[1] == "true");
		std::string file = params[2];

		ptp_test(*robot, margin, log, file);
	}
	else if (test == force)
	{
		double mass = stod(params[0]);
		double duration = stod(params[1]);

		force_test(*robot, mass, duration);
	}
	else if (test == playback)
	{
		bool log = (params[0] == "true");
		std::string file = params[1];

		playback_test(*robot, log, file);
	}
	else if (test == vacuum)
	{
		vacuum_test(*robot);
	}

	// Cleanup status thread
	stop = true;
	t.join();
}


void print_status(const franka_proxy::franka_remote_interface& robot)
{
	const auto config = robot.current_config();

	std::cout << " [ ";
	for (int i = 0; i < config.size(); ++i)
	{
		std::cout << std::fixed << std::setprecision(4)
			<< std::setw(7) << std::setfill(' ')
			<< config[i];
		if (i != config.size() - 1)
			std::cout << ", ";
	}
	std::cout << " ]\n";
}


template <class Function> void execute_retry(
	Function f, franka_proxy::franka_remote_interface& robot)
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
				"Waiting before retry." << '\n';
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}


void ple_motion_record_test(
	franka_proxy::franka_remote_interface& robot,
	double speed,
	double duration,
	bool log,
	const std::string& file)
{
	std::cout << "Starting PLE motion record test." << '\n';

	execute_retry([&] { robot.move_to(starting_pos); }, robot);

	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::optional<std::string> log_file_path = std::nullopt;
	if (log) log_file_path = file;
	robot.ple_motion(speed, duration, log_file_path);

	std::cout << "Finished PLE motion record test." << '\n';
}


void playback_test(
	franka_proxy::franka_remote_interface& robot,
	bool log,
	const std::string& file)
{
	std::cout << ("Starting Playback Test.") << '\n';

	std::optional<std::string> log_file_path = std::nullopt;
	if (log) log_file_path = file;

	std::cout << ("--- press to start in 3s (must light white) ---") << '\n';
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << ("--- starting demonstration for 10s ---") << '\n';
	robot.start_recording(log_file_path);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::cout << ("--- stopped demonstration ---") << '\n';
	const std::pair record(robot.stop_recording());


	std::cout << ("--- press to start reproduction in 3s (must light blue) ---") << '\n';
	std::cin.get();

	std::this_thread::sleep_for(std::chrono::seconds(3));

	franka_proxy::robot_config_7dof start_pos = record.first.front();
	franka_proxy::robot_config_7dof stop_pos = record.first.back();

	robot.move_to(start_pos);

	franka_proxy::robot_config_7dof curr_pos = robot.current_config();
	double error = relative_config_error(start_pos, curr_pos);
	std::cout << "Starting position reached with a relative error of: " << error << '\n';

	const std::vector selection_vectors(
		record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	robot.move_sequence(record.first, record.second, selection_vectors);

	curr_pos = robot.current_config();
	error = relative_config_error(stop_pos, curr_pos);
	std::cout << "Final position reached with a relative error of: " << error << '\n';

	std::cout << ("Finished Playback Test.") << '\n';
}


void gripper_test(
	franka_proxy::franka_remote_interface& robot,
	double margin,
	bool grasp)
{
	std::cout << "Starting Gripper Test." << '\n';


	std::cout << "Grasping with gripper." << '\n';
	robot.grasp_gripper(0.1);
	double gripper_pos = robot.current_gripper_pos(); // if there's nothing to grasp, grasp will be treated like close
	if (!((!grasp && gripper_pos < margin) || robot.gripper_grasped()))
	{
		std::cout << "Failed to grasp with gripper, aborting test." << '\n';
		return;
	}
	std::cout << "Success." << '\n';


	std::cout << "Opening gripper." << '\n';
	robot.open_gripper(0.1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin)
	{
		std::cout << "Failed to open gripper, aborting test." << '\n';
		return;
	}
	std::cout << "Success." << '\n';


	std::cout << "Closing gripper." << '\n';
	robot.close_gripper(1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos >= margin)
	{
		std::cout << "Failed to close gripper, aborting test." << '\n';
		return;
	}
	std::cout << "Success." << '\n';


	std::cout << "Opening gripper." << '\n';
	robot.open_gripper(1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin)
	{
		std::cout << "Failed to open gripper, aborting test." << '\n';
		return;
	}
	std::cout << "Success." << '\n';


	std::cout << "Finished Gripper Test." << '\n';
}


void ptp_test(
	franka_proxy::franka_remote_interface& robot,
	double margin,
	bool log,
	const std::string& file)
{
	std::cout << "Starting PTP-Movement Test." << '\n';

	franka_proxy::logger logger(file, 2, 0, 0, 1, 1);
	std::string succ;
	if (log)
	{
		std::vector<std::string> j = {
			"q0", "q1", "q2", "q3", "q4", "q5", "q6", "q0_d", "q1_d", "q2_d", "q3_d", "q4_d", "q5_d", "q6_d"
		};
		std::vector<std::string> e = {"error"};
		std::vector<std::string> s = {"success"};

		logger.start_logging(&j, nullptr, nullptr, &e, &s);
	}


	// poses for testing, starting pose is re-used as third testing pose
	constexpr franka_proxy::robot_config_7dof pos1
		{0.47604, 0.650999, 1.14681, -0.747477, 0.166399, 3.03934, -0.619102};
	constexpr franka_proxy::robot_config_7dof pos2
		{-0.713442, 0.744363, 0.543357, -1.40935, -2.06861, 1.6925, -2.46015};

	// trivially unreachable, as fourth joint must be negative
	constexpr franka_proxy::robot_config_7dof unreachable_pos
		{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};


	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);


	execute_retry([&] { robot.move_to(pos1); }, robot);
	franka_proxy::robot_config_7dof posc = robot.current_config();
	double error = relative_config_error(pos1, posc);
	if (error < margin)
	{
		std::cout << "Pose 1 of 4 reached successfully (relative error: " << error << ")." << '\n';
		succ = "yes";
	}
	else
	{
		std::cout << "Pose 1 of 4 missed with a relative error of: " << error << '\n';
		succ = "no";
	}
	if (log)
	{
		logger.add_joint_data(posc);
		logger.add_joint_data(pos1);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}


	execute_retry([&] { robot.move_to(pos2); }, robot);
	posc = robot.current_config();
	error = relative_config_error(pos2, posc);
	if (error < margin)
	{
		std::cout << "Pose 2 of 4 reached successfully (relative error: " << error << ")." << '\n';
		succ = "yes";
	}
	else
	{
		std::cout << "Pose 2 of 4 missed with a relative error of: " << error << '\n';
		succ = "no";
	}
	if (log)
	{
		logger.add_joint_data(posc);
		logger.add_joint_data(pos2);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}


	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	posc = robot.current_config();
	error = relative_config_error(starting_pos, posc);
	if (error < margin)
	{
		std::cout << "Pose 3 of 4 reached successfully (relative error: " << error << ")." << '\n';
		succ = "yes";
	}
	else
	{
		std::cout << "Pose 3 of 4 missed with a relative error of: " << error << '\n';
		succ = "no";
	}
	if (log)
	{
		logger.add_joint_data(posc);
		logger.add_joint_data(starting_pos);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}


	try
	{
		robot.move_to(unreachable_pos);
	}
	catch (franka_proxy::invalid_operation_exception& e)
	{
		std::cout << e.what() << '\n';
		std::cout << "Pose 4 of 4 successfully identified as unreachable." << '\n';
	}
	catch (franka_proxy::exception& f)
	{
		std::cout << "A franka exception ocurred (not related to pose 4 of 4 being unreachable)." << '\n';
		std::cout << f.what() << '\n';
		std::cout << typeid(f).name() << '\n';
		std::cout << "Pose 4 not identified as unreachable. Test failed." << '\n';
	}
	catch (std::exception& g)
	{
		std::cout << "A general exception occured (not related to pose 4 of 4 being unreachable)." << '\n';
		std::cout << g.what() << '\n';
		std::cout << typeid(g).name() << '\n';
		std::cout << "Pose 4 not identified as unreachable. Test failed." << '\n';
	}


	if (log) logger.stop_logging();

	std::cout << "Finished PTP-Movement Test." << '\n';
}


void force_test(
	franka_proxy::franka_remote_interface& robot,
	const double mass,
	const double duration)
{
	std::cout << "Starting Force Test with mass " << mass
		<< " for " << duration << " seconds." << '\n';

	// this one is down from starting_pos by (0.0, 0.0, 0.5)
	constexpr franka_proxy::robot_config_7dof tgt_pos
		{0.0226325, 0.980913, -0.0348997, -1.99411, 0.069761, 2.98885, 0.708856};


	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	robot.close_gripper();
	robot.set_speed_factor(0.05);
	try
	{
		if (!robot.move_to_until_contact(tgt_pos))
		{
			std::cout << "Aborting Force Test, not able to make contact." << '\n';
			return;
		}
	}
	catch (franka_proxy::remote_exception& e)
	{
		std::cout << e.what() << '\n';
		std::cout << "Aborting Force Test, please try again." << '\n';
		return;
	}

	// move out of contact again to prevent robot from jerking left/right - twitching still occurs, but robot doesn't move as much (most of the time)
	const auto contact = robot.current_config();
	franka_proxy::robot_config_7dof relax{
		contact[0], (contact[1] - 0.01), contact[2], contact[3], contact[4], (contact[5] - 0.01), contact[6]
	};
	execute_retry([&] { robot.move_to(relax); }, robot);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	try
	{
		robot.apply_z_force(mass, duration);
	}
	catch (franka_proxy::remote_exception& e)
	{
		std::cout << e.what() << '\n';
	}

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);

	std::cout << "Finished Force Test." << '\n';
}


void vacuum_test(franka_proxy::franka_remote_interface& robot)
{
	execute_retry([&] { robot.vacuum_gripper_stop(); }, robot);
	std::this_thread::sleep_for(std::chrono::seconds(10));
	execute_retry([&] { robot.vacuum_gripper_vacuum(100, std::chrono::milliseconds(1000)); }, robot);
	std::cout << "pause\n";
	std::this_thread::sleep_for(std::chrono::seconds(10));
	execute_retry([&] { robot.vacuum_gripper_stop(); }, robot);
}


void guiding_test(franka_proxy::franka_remote_interface& robot)
{
	std::chrono::seconds duration(10);
	//set to default before test
	robot.set_guiding_params(true, true, true, true, true, true, false);

	std::cout << "Robot is now for " << duration
		<< " in guiding mode with guidable DOF (1,1,1,0,0,0) and elbow is false" << '\n';
	robot.set_guiding_params(true, true, true, false, false, false, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for " << duration
		<< " in default guiding mode" << '\n';
	robot.set_guiding_params(true, true, true, true, true, true, false);
	std::this_thread::sleep_for(duration);

	std::cout << "Robot is now for " << duration
		<< "  in guiding mode with guidable DOF (1,1,0,1,1,0) and elbow is false" << '\n';
	robot.set_guiding_params(true, true, false, true, true, false, false);
	std::this_thread::sleep_for(duration);

	//set back to default after test
	robot.set_guiding_params(true, true, true, true, true, true, false);
	std::cout << "Finished guiding mode test." << '\n';
}


double relative_config_error(
	const franka_proxy::robot_config_7dof& desired_config,
	const franka_proxy::robot_config_7dof& current_config)
{
	double error = 0.0;

	for (size_t i = 0; i < desired_config.size(); i++)
		error += std::abs((desired_config[i] - current_config[i]) / desired_config[i]);

	return error / static_cast<double>(desired_config.size());
}
