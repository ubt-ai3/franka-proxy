#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <argparse/argparse.hpp>

#include <franka_proxy_share/franka_proxy_logger.hpp>
#include <franka_proxy_client/forward.hpp> // included for IDE visibility of the forward header
#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>


enum test_mode : std::uint8_t
{
	none, ple, playback, gripper, ptp, force, vacuum, guiding
};

// General purpose starting position, centered above table with some good space in all directions.
constexpr franka_proxy::robot_config_7dof starting_pos{
	0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461
};

namespace time_test_constants
{
constexpr auto short_delay = std::chrono::seconds(1);
constexpr auto medium_delay = std::chrono::seconds(3);
constexpr auto long_delay = std::chrono::seconds(10);
constexpr auto update_sleep_ms = std::chrono::milliseconds(16); // Approx. 60 Hz update rate
constexpr int print_status_interval = 10;
}


void franka_proxy_client_test(
	const std::string& ip,
	test_mode test,
	const std::vector<std::string>& params);


namespace
{
template <std::size_t D>
void print_fixed_format(const std::string& label, const std::array<double, D>& vec)
{
	std::cout << label << "[ ";
	for (int i = 0; i < vec.size(); ++i)
	{
		std::cout << std::fixed << std::setprecision(4)
			<< std::setw(7) << std::setfill(' ')
			<< vec[i];
		if (i != vec.size() - 1)
			std::cout << ", ";
	}
	std::cout << " ]\n";
}


void print_status(const franka_proxy::franka_remote_interface& robot)
{
	print_fixed_format("Current robot joints: ", robot.current_config());
	//print_fixed_format("Current wrench: ", robot.current_end_effector_wrench());
}
}


template <class Function>
void execute_retry(Function f, franka_proxy::franka_remote_interface& robot);


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
	argparse::ArgumentParser program("franka_client_test");
	program.add_argument("-ip")
	       .help("specify IP for franka-proxy remote connection")
	       .default_value(std::string{"127.0.0.1"});

	argparse::ArgumentParser base("fct_base", "1.0", argparse::default_arguments::none);
	base.add_argument("-l", "-log")
	    .help("enable logging")
	    .flag();
	base.add_argument("-f", "-file")
	    .help("specify file to write log into")
	    .default_value("franka_client_test_log.csv");


	argparse::ArgumentParser ple_test_parser("ple");
	ple_test_parser.add_argument("speed")
	               .help("specify speed factor for ple motion")
	               .default_value("0.3");
	ple_test_parser.add_argument("-d", "-duration")
	               .help("specify duration (sec) for ple motion")
	               .default_value("10.0");
	ple_test_parser.add_parents(base);

	argparse::ArgumentParser gripper_test_parser("gripper");
	gripper_test_parser.add_argument("margin")
	                   .help("specify margin below which gripper is considered closed/grasped (useful for different gripper configs)")
	                   .default_value("0.01");
	gripper_test_parser.add_argument("-g")
	                   .help("enable grasping of a provided object (gripper will close normally otherwise)")
	                   .flag();

	argparse::ArgumentParser ptp_test_parser("ptp");
	ptp_test_parser.add_argument("margin")
	               .help("specify margin below which relative pose error is ignored and pose is considered reached")
	               .default_value("0.1");
	ptp_test_parser.add_parents(base);

	argparse::ArgumentParser force_test_parser("force");
	force_test_parser.add_argument("-m")
	                 .help("specify mass (kg) with which to press downward")
	                 .default_value("0.5");
	force_test_parser.add_argument("-d")
	                 .help("specify duration (sec) for which to press downward")
	                 .default_value("10.0");

	argparse::ArgumentParser vacuum_test_parser("vacuum");
	vacuum_test_parser.add_parents(base);

	argparse::ArgumentParser playback_test_parser("playback");
	playback_test_parser.add_parents(base);

	argparse::ArgumentParser guiding_test_parser("guiding");
	guiding_test_parser.add_parents(base);

	program.add_subparser(vacuum_test_parser);
	program.add_subparser(playback_test_parser);
	program.add_subparser(force_test_parser);
	program.add_subparser(ptp_test_parser);
	program.add_subparser(gripper_test_parser);
	program.add_subparser(ple_test_parser);
	program.add_subparser(guiding_test_parser);

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
	std::cout << "--------------------------------------------------------------------------------\n"
		<< "Executing Franka client test with IP " << ip << ": \n";

	test_mode selected_test = none;
	std::vector<std::string> params;

	if (program.is_subcommand_used(ple_test_parser))
	{
		selected_test = ple;
		params.emplace_back(ple_test_parser.get<std::string>("speed"));
		params.emplace_back(ple_test_parser.get<std::string>("-d"));
		params.emplace_back(ple_test_parser.get<bool>("-l") ? "true" : "false");
		params.emplace_back(ple_test_parser.get<std::string>("-f"));
	}
	else if (program.is_subcommand_used(gripper_test_parser))
	{
		selected_test = gripper;
		params.emplace_back(gripper_test_parser.get<std::string>("margin"));
		params.emplace_back(gripper_test_parser.get<bool>("-g") ? "true" : "false");
	}
	else if (program.is_subcommand_used(ptp_test_parser))
	{
		selected_test = ptp;
		params.emplace_back(ptp_test_parser.get<std::string>("margin"));
		params.emplace_back(ptp_test_parser.get<bool>("-l") ? "true" : "false");
		params.emplace_back(ptp_test_parser.get<std::string>("-f"));
	}
	else if (program.is_subcommand_used(force_test_parser))
	{
		selected_test = force;
		params.emplace_back(force_test_parser.get<std::string>("-m"));
		params.emplace_back(force_test_parser.get<std::string>("-d"));
	}
	else if (program.is_subcommand_used(playback_test_parser))
	{
		selected_test = playback;
		params.emplace_back(playback_test_parser.get<bool>("-l") ? "true" : "false");
		params.emplace_back(playback_test_parser.get<std::string>("-f"));
	}
	else if (program.is_subcommand_used(vacuum_test_parser))
	{
		selected_test = vacuum;
	}
	else if (program.is_subcommand_used(guiding_test_parser))
	{
		selected_test = guiding;
	}


	franka_proxy_client_test(ip, selected_test, params);
	std::cout << "Press Enter to end test program." << '\n';
	std::cin.get();
	return 0;
}


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
	std::atomic_bool stop(false);
	std::thread status_thread([&stop, &robot]()
	{
		int i = 0;
		while (!stop)
		{
			robot->update();

			if (i % time_test_constants::print_status_interval == 0)
			{
				std::cout << "Status #" << i << ": ";
				print_status(*robot);
			}
			std::this_thread::sleep_for(time_test_constants::update_sleep_ms);
			++i;
		}
	});


	// Franka motion tests - only one test can be run at a time.
	// Parameters for test methods arrive here as a vector of strings, so convert accordingly.
	if (test == none)
	{
		std::cout << "No specific test selected. Only testing status socket for "
			<< time_test_constants::long_delay.count() << " seconds.\n";
		std::this_thread::sleep_for(time_test_constants::long_delay);
	}
	else if (test == ple)
	{
		double speed = std::stod(params[0]);
		double duration = std::stod(params[1]);
		bool log = (params[2] == "true");
		const std::string& file = params[3];
		ple_motion_record_test(*robot, speed, duration, log, file);
	}
	else if (test == gripper)
	{
		double margin = std::stod(params[0]);
		bool grasp = (params[1] == "true");
		gripper_test(*robot, margin, grasp);
	}
	else if (test == ptp)
	{
		double margin = std::stod(params[0]);
		bool log = (params[1] == "true");
		const std::string& file = params[2];
		ptp_test(*robot, margin, log, file);
	}
	else if (test == force)
	{
		double mass = std::stod(params[0]);
		double duration = std::stod(params[1]);
		force_test(*robot, mass, duration);
	}
	else if (test == playback)
	{
		bool log = (params[0] == "true");
		const std::string& file = params[1];
		playback_test(*robot, log, file);
	}
	else if (test == vacuum)
	{
		vacuum_test(*robot);
	}
	else if (test == guiding)
	{
		guiding_test(*robot);
	}

	// Cleanup status thread
	stop = true;
	status_thread.join();
}


template <class Function>
void execute_retry(Function f, franka_proxy::franka_remote_interface& robot)
{
	bool finished = false;
	while (!finished)
	{
		try
		{
			f();
			finished = true;
		}
		catch (const franka_proxy::control_exception& e)
		{
			std::cerr << "Caught control exception: " << e.what() << ". Attempting automatic error recovery.\n";
			std::this_thread::sleep_for(time_test_constants::short_delay);
			robot.automatic_error_recovery();
		}
		catch (const franka_proxy::command_exception& e)
		{
			std::cerr << "Caught command exception: " << e.what()
				<< ". Probably because of wrong working mode. Waiting before retry.\n";
			std::this_thread::sleep_for(time_test_constants::medium_delay);
		}
		catch (const std::exception& e)
		{
			std::cerr << "Caught unexpected exception during execution: " << e.what() << '\n';
			throw;
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
	std::cout << "Starting PLE motion record test.\n";

	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	std::this_thread::sleep_for(time_test_constants::short_delay);

	std::optional<std::string> log_file_path = std::nullopt;
	if (log)
		log_file_path = file;

	robot.ple_motion(speed, duration, log_file_path);
	std::cout << "Finished PLE motion record test.\n";
}


void playback_test(
	franka_proxy::franka_remote_interface& robot,
	bool log,
	const std::string& file)
{
	std::cout << "Starting Playback Test.\n";

	std::optional<std::string> log_file_path = std::nullopt;
	if (log)
		log_file_path = file;

	std::cout << "--- Press Enter to start recording in " << time_test_constants::short_delay.count()
		<< "s (robot lights white) ---\n";
	std::cin.get();
	std::this_thread::sleep_for(time_test_constants::short_delay);

	std::cout << "--- Starting demonstration for " << time_test_constants::long_delay.count() << "s ---\n";
	robot.start_recording(log_file_path);
	std::this_thread::sleep_for(time_test_constants::long_delay);

	std::cout << "--- Stopped demonstration ---\n";
	const auto record = robot.stop_recording();

	std::cout << "--- Press Enter to start reproduction in " << time_test_constants::short_delay.count()
		<< "s (robot lights blue) ---\n";
	std::cin.get();
	std::this_thread::sleep_for(time_test_constants::short_delay);

	franka_proxy::robot_config_7dof start_pos = record.first.front();
	franka_proxy::robot_config_7dof stop_pos = record.first.back();

	robot.move_to(start_pos);

	franka_proxy::robot_config_7dof curr_pos = robot.current_config();
	double error = relative_config_error(start_pos, curr_pos);
	std::cout << "Starting position reached with a relative joint offset of: " << error << '\n';

	const std::vector selection_vectors(
		record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	robot.move_sequence(record.first, record.second, selection_vectors);

	curr_pos = robot.current_config();
	error = relative_config_error(stop_pos, curr_pos);
	std::cout << "Final position reached with a relative joint offset of: " << error << '\n';

	std::cout << "Finished Playback Test.\n";
}


void gripper_test(
	franka_proxy::franka_remote_interface& robot,
	double margin,
	bool grasp)
{
	std::cout << "Starting Gripper Test.\n";

	std::cout << "Attempting grasp with gripper.\n";
	robot.grasp_gripper(0.1);
	double gripper_pos = robot.current_gripper_pos();
	if (!((!grasp && gripper_pos < margin) || robot.gripper_grasped()))
	{
		std::cout << "Failed to grasp with gripper. Aborting test.\n";
		return;
	}
	std::cout << "Grasp successful.\n";

	std::cout << "Opening gripper.\n";
	robot.open_gripper(0.1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin)
	{
		std::cout << "Failed to open gripper. Aborting test.\n";
		return;
	}
	std::cout << "Gripper opened successfully.\n";

	std::cout << "Closing gripper.\n";
	robot.close_gripper(1.0);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos >= margin)
	{
		std::cout << "Failed to close gripper. Aborting test.\n";
		return;
	}
	std::cout << "Gripper closed successfully.\n";

	std::cout << "Opening gripper.\n";
	robot.open_gripper(0.5);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin)
	{
		std::cout << "Failed to open gripper. Aborting test.\n";
		return;
	}
	std::cout << "Gripper opened successfully.\n";

	std::cout << "Finished Gripper Test.\n";
}


void ptp_test(
	franka_proxy::franka_remote_interface& robot,
	double margin,
	bool log,
	const std::string& file)
{
	std::cout << "Starting PTP-Movement Test.\n";

	franka_proxy::logger logger(file, 2, 0, 0, 1, 1);
	std::string succ_status;
	if (log)
	{
		std::vector<std::string> j = {
			"q0", "q1", "q2", "q3", "q4", "q5", "q6", "q0_d", "q1_d", "q2_d", "q3_d", "q4_d", "q5_d", "q6_d"
		};
		std::vector<std::string> e = {"error"};
		std::vector<std::string> s = {"success"};

		logger.start_logging(&j, nullptr, nullptr, &e, &s);
	}

	constexpr franka_proxy::robot_config_7dof pos1{
		0.47604, 0.650999, 1.14681, -0.747477, 0.166399, 3.03934, -0.619102
	};
	constexpr franka_proxy::robot_config_7dof pos2{
		-0.713442, 0.744363, 0.543357, -1.40935, -2.06861, 1.6925, -2.46015
	};
	constexpr franka_proxy::robot_config_7dof unreachable_pos{
		1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
	};

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);


	execute_retry([&] { robot.move_to(pos1); }, robot);
	franka_proxy::robot_config_7dof current_config = robot.current_config();
	double error = relative_config_error(pos1, current_config);
	if (error < margin)
	{
		std::cout << "Pose 1 of 4 reached successfully (relative error: " << error << ").\n";
		succ_status = "yes";
	}
	else
	{
		std::cout << "Pose 1 of 4 missed with a relative error of: " << error << ".\n";
		succ_status = "no";
	}
	if (log)
	{
		logger.add_joint_data(current_config);
		logger.add_joint_data(pos1);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ_status);
		logger.log();
	}


	execute_retry([&] { robot.move_to(pos2); }, robot);
	current_config = robot.current_config();
	error = relative_config_error(pos2, current_config);
	if (error < margin)
	{
		std::cout << "Pose 2 of 4 reached successfully (relative error: " << error << ").\n";
		succ_status = "yes";
	}
	else
	{
		std::cout << "Pose 2 of 4 missed with a relative error of: " << error << ".\n";
		succ_status = "no";
	}
	if (log)
	{
		logger.add_joint_data(current_config);
		logger.add_joint_data(pos2);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ_status);
		logger.log();
	}


	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	current_config = robot.current_config();
	error = relative_config_error(starting_pos, current_config);
	if (error < margin)
	{
		std::cout << "Pose 3 of 4 reached successfully (relative error: " << error << ").\n";
		succ_status = "yes";
	}
	else
	{
		std::cout << "Pose 3 of 4 missed with a relative error of: " << error << ".\n";
		succ_status = "no";
	}
	if (log)
	{
		logger.add_joint_data(current_config);
		logger.add_joint_data(starting_pos);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ_status);
		logger.log();
	}


	try
	{
		robot.move_to(unreachable_pos);
		std::cout << "Error: Unreachable pose 4 of 4 was reached. Test failed.\n";
	}
	catch (franka_proxy::invalid_operation_exception& e)
	{
		std::cout << e.what() << '\n';
		std::cout << "Pose 4 of 4 successfully identified as unreachable.\n";
	}
	catch (franka_proxy::exception& e)
	{
		std::cerr << "A Franka proxy exception occurred (not related to pose 4 of 4 being unreachable): "
			<< e.what() << '\n';
		std::cerr << "Type: " << typeid(e).name() << '\n';
		std::cout << "Pose 4 not identified as unreachable. Test failed.\n";
	}
	catch (std::exception& e)
	{
		std::cerr << "A general exception occurred (not related to pose 4 of 4 being unreachable): "
			<< e.what() << '\n';
		std::cerr << "Type: " << typeid(e).name() << '\n';
		std::cout << "Pose 4 not identified as unreachable. Test failed.\n";
	}

	if (log)
		logger.stop_logging();

	std::cout << "Finished PTP-Movement Test.\n";
}


void force_test(
	franka_proxy::franka_remote_interface& robot,
	const double mass,
	const double duration)
{
	std::cout << "Starting Force Test with mass " << mass
		<< " for " << duration << " seconds.\n";

	// Target position for contact, below starting_pos, slightly above our table
	constexpr franka_proxy::robot_config_7dof target_contact_pos{
		0.0226325, 0.980913, -0.0348997, -1.99411, 0.069761, 2.98885, 0.708856
	};

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	robot.close_gripper();
	robot.set_speed_factor(0.05);
	try
	{
		if (!robot.move_to_until_contact(target_contact_pos))
		{
			std::cout << "Aborting Force Test: Robot not able to make contact.\n";
			return;
		}
		std::cout << "Contact detected.\n";
	}
	catch (franka_proxy::remote_exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cout << "Aborting Force Test due to remote exception. Please try again.\n";
		return;
	}

	// Move out of contact again to prevent robot from jerking left/right.
	// Twitching sometimes still occurs, but robot doesn't move as much (most of the time).
	const auto contact_config = robot.current_config();
	franka_proxy::robot_config_7dof relax_config{
		contact_config[0], (contact_config[1] - 0.01), contact_config[2], contact_config[3],
		contact_config[4], (contact_config[5] - 0.01), contact_config[6]
	};
	execute_retry([&] { robot.move_to(relax_config); }, robot);
	std::this_thread::sleep_for(time_test_constants::medium_delay);

	try
	{
		std::cout << "Applying z-force...\n";
		robot.apply_z_force(mass, duration);
		std::cout << "z-force application complete.\n";
	}
	catch (franka_proxy::remote_exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cout << "Force application interrupted by remote exception.\n";
	}

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);

	std::cout << "Finished Force Test.\n";
}


void vacuum_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting Vacuum Gripper Test.\n";

	robot.vacuum_gripper_stop();
	std::this_thread::sleep_for(std::chrono::seconds(10));
	std::cout << "Vacuuming for " << time_test_constants::short_delay << ".\n";
	robot.vacuum_gripper_vacuum(100, time_test_constants::short_delay);
	std::cout << "Short pause\n";
	std::this_thread::sleep_for(time_test_constants::medium_delay);
	std::cout << "Vacuuming for " << time_test_constants::medium_delay << ".\n";
	robot.vacuum_gripper_vacuum(100, time_test_constants::short_delay);
	robot.vacuum_gripper_stop();

	std::cout << "Finished Vacuum Gripper Test.\n";
}


void guiding_test(franka_proxy::franka_remote_interface& robot)
{
	std::cout << "Starting Guiding Mode Test.\n";

	// Set to default before test
	robot.set_guiding_params(true, true, true, true, true, true, false);
	std::cout << "Robot set to default guiding mode "
		<< "(all DOFs guidable, elbow false).\n";
	std::this_thread::sleep_for(time_test_constants::short_delay);


	std::cout << "Robot in guiding mode with guidable "
		<< "DOFs(X, Y, Z), elbow false for "
		<< time_test_constants::long_delay.count() << " seconds.\n";
	robot.set_guiding_params(true, true, true, false, false, false, false);
	std::this_thread::sleep_for(time_test_constants::long_delay);

	std::cout << "Robot back to default guiding mode for "
		<< time_test_constants::long_delay.count() << " seconds.\n";
	robot.set_guiding_params(true, true, true, true, true, true, false);
	std::this_thread::sleep_for(time_test_constants::long_delay);

	std::cout << "Robot in guiding mode with guidable "
		<< "DOFs (X, Y, Rx, Ry), elbow false for "
		<< time_test_constants::long_delay.count() << " seconds.\n";
	robot.set_guiding_params(true, true, false, true, true, false, false);
	std::this_thread::sleep_for(time_test_constants::long_delay);

	// Set back to default after test
	robot.set_guiding_params(true, true, true, true, true, true, false);
	std::cout << "Finished guiding mode test. "
		<< "Robot returned to default guiding mode.\n";
}


double relative_config_error(
	const franka_proxy::robot_config_7dof& desired_config,
	const franka_proxy::robot_config_7dof& current_config)
{
	double error = 0.0;
	for (size_t i = 0; i < desired_config.size(); i++)
		if (!(std::abs(desired_config[i]) < 1e-9))
			error += std::abs((desired_config[i] - current_config[i]) / desired_config[i]);
	return error / static_cast<double>(desired_config.size());
}
