#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
#include <optional>

#include <argparse/argparse.hpp>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>

#include <franka_proxy_share/franka_proxy_logger.hpp>



// use this to specify which test to run within the franka_proxy_client_test method
// for new tests, add a new, distinct and sensible entry here, a new subparser to the main method,
// and a new case to franka_proxy_client_test
enum mode{none, ple, playback, gripper, ptp, force, ermer};


// general purpose starting position, centered above table with some good space in all directions
// if you don't know where to have the robot start, use this
constexpr franka_proxy::robot_config_7dof starting_pos
{ 0.0346044, -0.0666144, -0.0398886, -2.04985, -0.0229875, 1.99782, 0.778461 };


void franka_proxy_client_test(const std::string& ip, mode test, std::vector<std::string>& params);


void print_status(const franka_proxy::franka_remote_interface& robot);
template <class Function> void execute_retry(
	Function&& f, franka_proxy::franka_remote_interface& robot);
double calculate_pose_error(const franka_proxy::robot_config_7dof& pose_d, const franka_proxy::robot_config_7dof& pose_c);


void ple_motion_record_test(franka_proxy::franka_remote_interface& robot, double speed, double duration, bool log, std::string file);
void ptp_test(franka_proxy::franka_remote_interface& robot, double margin, bool log, std::string& file);
void gripper_test(franka_proxy::franka_remote_interface& robot, double margin, bool grasp);
void playback_test(franka_proxy::franka_remote_interface& robot, bool log, std::string& file);
void force_test(franka_proxy::franka_remote_interface& robot, double mass, double duration);


// todo: untested since bachelor thesis d. ermer; handle with care.
[[deprecated("Revise test code before execution on real robot!")]]
void impedance_admittance_ermer_ba_tests(franka_proxy::franka_remote_interface& robot, bool log);


int main(int argc, char* argv[])
{
	// this is the main parser, which will only handle specifying an IP and calling individual subparsers
	argparse::ArgumentParser program("franka_client_test");
	program.add_argument("-ip")
		.help("specify IP for franka-proxy remote connection")
		.default_value(std::string{ "127.0.0.1" });


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
		.help("specify margin below which gripper is considered closed/grasped (useful for different gripper configs)")
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


	argparse::ArgumentParser playback_test("playback");
	// has no further arguments
	playback_test.add_parents(base);

	
	//argparse::ArgumentParser ermer_test("ermer");
	//// has no further arguments and logs into multiple files (ignores "-f" argument)
	//ermer_test.add_parents(base);


	//program.add_subparser(ermer_test);
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
	
	// case distinction for individual tests / subparsers, remember to set "test" to the corresponding mode
	// this is where input arguments are handled, which will be passed to franka_proxy_client_test as a vector of strings
	// note that only one test can be run at the same time, so add new tests with an "else if" block
	if (program.is_subcommand_used(ple_test)) {
		test = mode::ple;

		std::string speed = ple_test.get<std::string>("speed");
		std::string duration = ple_test.get<std::string>("-d");
		bool log_flag = ple_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		std::string file = ple_test.get<std::string>("-f");

		params.push_back(speed);
		params.push_back(duration);
		params.push_back(log);
		params.push_back(file);
	}
	else if (program.is_subcommand_used(gripper_test)) {
		test = mode::gripper;

		std::string margin = gripper_test.get<std::string>("margin");
		bool grasp_flag = gripper_test.get<bool>("-g");
		std::string grasp("false");
		if (grasp_flag) grasp = "true";

		params.push_back(margin);
		params.push_back(grasp);
	}
	else if (program.is_subcommand_used(ptp_test)) {
		test = mode::ptp;

		std::string margin = ptp_test.get<std::string>("margin");
		bool log_flag = ptp_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		std::string file = ptp_test.get<std::string>("-f");
		
		params.push_back(margin);
		params.push_back(log);
		params.push_back(file);
	}
	else if (program.is_subcommand_used(force_test)) {
		test = mode::force;

		std::string mass = force_test.get<std::string>("-m");
		std::string duration = force_test.get<std::string>("-d");

		params.push_back(mass);
		params.push_back(duration);
	}
	else if (program.is_subcommand_used(playback_test)) {
		test = mode::playback;

		bool log_flag = playback_test.get<bool>("-l");
		std::string log("false");
		if (log_flag) log = "true";
		std::string file = playback_test.get<std::string>("-f");

		params.push_back(log);
		params.push_back(file);
	}
	/*else if (program.is_subcommand_used(ermer_test)) {
		test = mode::ermer;

		bool log_flag = ermer_test.get<bool>("-l");
		std::string log = "false";
		if (log_flag) log = "true";

		params.push_back(log);
	}*/


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

	// case distinction for individual tests (add new tests with an "else if" block)
	// parameters for test methods arrive here as a vector of strings, so convert accordingly
	if (test == mode::ple) {
		double speed = stod(params[0]);
		double duration = stod(params[1]);
		bool log = (params[2] == "true");
		std::string file = params[3];

		ple_motion_record_test(*robot, speed, duration, log, file);
	}
	else if (test == mode::gripper) {
		double margin = stod(params[0]);
		bool grasp = (params[1] == "true");

		gripper_test(*robot, margin, grasp);
	}
	else if (test == mode::ptp) {
		double margin = stod(params[0]);
		bool log = (params[1] == "true");
		std::string file = params[2];

		ptp_test(*robot, margin, log, file);
	}
	else if (test == mode::force) {
		double mass = stod(params[0]);
		double duration = stod(params[1]);

		force_test(*robot, mass, duration);
	}
	else if (test == mode::playback) {
		bool log = (params[0] == "true");
		std::string file = params[1];

		playback_test(*robot, log, file);
	}
	/*else if (test == mode::ermer) {
		bool log = (params[0] == "true");

		impedance_admittance_ermer_ba_tests(*robot, log);
	}*/
	

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
	std::cout << "Starting PLE motion record test." << std::endl;

	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::optional<std::string> log_file_path = std::nullopt;
	if (log) log_file_path = file;
	robot.ple_motion(speed, duration, log_file_path);

	std::cout << "Finished PLE motion record test." << std::endl;
}


void playback_test(franka_proxy::franka_remote_interface& robot, bool log, std::string& file)
{
	std::cout << ("Starting Playback Test.") << std::endl;

	std::optional<std::string> log_file_path = std::nullopt;
	if (log) log_file_path = file;
	
	std::cout << ("--- press to start in 3s (lights white) ---") << std::endl;
	std::cin.get();
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << ("--- starting demonstration ---") << std::endl;
	robot.start_recording(log_file_path);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::cout << ("--- stopped demonstration ---") << std::endl;
	const std::pair record(robot.stop_recording());


	std::cout << ("--- press to start reproduction in 3s (lights blue) ---") << std::endl;
	std::cin.get();
	
	std::this_thread::sleep_for(std::chrono::seconds(3));

	franka_proxy::robot_config_7dof start_pos = record.first.front();
	franka_proxy::robot_config_7dof stop_pos = record.first.back();

	robot.move_to(start_pos);

	franka_proxy::robot_config_7dof curr_pos = robot.current_config();
	double error = calculate_pose_error(start_pos, curr_pos);
	std::cout << "Starting position reached with a relative error of: " << error << std::endl;

	const std::vector selection_vectors(
		record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	robot.move_sequence(record.first, record.second, selection_vectors);

	curr_pos = robot.current_config();
	error = calculate_pose_error(stop_pos, curr_pos);
	std::cout << "Final position reached with a relative error of: " << error << std::endl;
	
	std::cout << ("Finished Playback Test.") << std::endl;
}


void gripper_test(franka_proxy::franka_remote_interface& robot, double margin, bool grasp)
{
	std::cout << "Starting Gripper Test." << std::endl;


	std::cout << "Grasping with gripper." << std::endl;
	robot.grasp_gripper(0.1);
	double gripper_pos = robot.current_gripper_pos(); // if there's nothing to grasp, grasp will be treated like close
	if (!((!grasp && gripper_pos < margin) || robot.gripper_grasped())) {
		std::cout << "Failed to grasp with gripper, aborting test." << std::endl;
		return;
	}
	std::cout << "Success." << std::endl;


	std::cout << "Opening gripper." << std::endl;
	robot.open_gripper(0.1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin) {
		std::cout << "Failed to open gripper, aborting test." << std::endl;
		return;
	}
	std::cout << "Success." << std::endl;


	std::cout << "Closing gripper." << std::endl;
	robot.close_gripper(1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos >= margin) {
		std::cout << "Failed to close gripper, aborting test." << std::endl;
		return;
	}
	std::cout << "Success." << std::endl;


	std::cout << "Opening gripper." << std::endl;
	robot.open_gripper(1);
	gripper_pos = robot.current_gripper_pos();
	if (gripper_pos < margin) {
		std::cout << "Failed to open gripper, aborting test." << std::endl;
		return;
	}
	std::cout << "Success." << std::endl;


	std::cout << "Finished Gripper Test." << std::endl;
}


void ptp_test(franka_proxy::franka_remote_interface& robot, double margin, bool log, std::string& file)
{
	std::cout << "Starting PTP-Movement Test." << std::endl;

	franka_proxy::logger logger(file, 2, 0, 0, 1, 1);
	std::string succ;
	if (log) {
		std::vector<std::string> j = { "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q0_d", "q1_d", "q2_d", "q3_d", "q4_d", "q5_d", "q6_d" };
		std::vector<std::string> e = {"error"};
		std::vector<std::string> s = { "success" };
		
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
	double error = calculate_pose_error(pos1, posc);
	if (error < margin) {
		std::cout << "Pose 1 of 4 reached successfully (relative error: " << error << ")." << std::endl;
		succ = "yes";
	}
	else {
		std::cout << "Pose 1 of 4 missed with a relative error of: " << error << std::endl;
		succ = "no";
	}
	if (log) {
		logger.add_joint_data(posc);
		logger.add_joint_data(pos1);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}


	execute_retry([&] { robot.move_to(pos2); }, robot);
	posc = robot.current_config();
	error = calculate_pose_error(pos2, posc);
	if (error < margin) {
		std::cout << "Pose 2 of 4 reached successfully (relative error: " << error << ")." << std::endl;
		succ = "yes";
	}
	else {
		std::cout << "Pose 2 of 4 missed with a relative error of: " << error << std::endl;
		succ = "no";
	}
	if (log) {
		logger.add_joint_data(posc);
		logger.add_joint_data(pos2);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}


	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	posc = robot.current_config();
	error = calculate_pose_error(starting_pos, posc);
	if (error < margin) {
		std::cout << "Pose 3 of 4 reached successfully (relative error: " << error << ")." << std::endl;
		succ = "yes";
	}
	else {
		std::cout << "Pose 3 of 4 missed with a relative error of: " << error << std::endl;
		succ = "no";
	}
	if (log) {
		logger.add_joint_data(posc);
		logger.add_joint_data(starting_pos);
		logger.add_single_data(error);
		logger.add_arbitrary_data(succ);

		logger.log();
	}

	
	try {
		robot.move_to(unreachable_pos);
	}
	catch (franka_proxy::invalid_operation_exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "Pose 4 of 4 successfully identified as unreachable." << std::endl;
	}
	catch (franka_proxy::exception& f) {
		std::cout << "A franka exception ocurred (not related to pose 4 of 4 being unreachable)." << std::endl;
		std::cout << f.what() << std::endl;
		std::cout << typeid(f).name() << std::endl;
		std::cout << "Pose 4 not identified as unreachable. Test failed." << std::endl;
	}
	catch (std::exception& g) {
		std::cout << "A general exception occured (not related to pose 4 of 4 being unreachable)." << std::endl;
		std::cout << g.what() << std::endl;
		std::cout << typeid(g).name() << std::endl;
		std::cout << "Pose 4 not identified as unreachable. Test failed." << std::endl;
	}


	if (log) logger.stop_logging();

	std::cout << "Finished PTP-Movement Test." << std::endl;
}


void force_test(franka_proxy::franka_remote_interface& robot, double mass, double duration)
{
	std::cout << "Starting Force Test." << std::endl;


	// this one is down from starting_pos by (0.0, 0.0, 0.5)
	constexpr franka_proxy::robot_config_7dof tgt_pos
	{ 0.0226325, 0.980913, - 0.0348997, - 1.99411, 0.069761, 2.98885, 0.708856 };


	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	robot.close_gripper();
	robot.set_speed_factor(0.05);
	try {
		robot.move_to_until_contact(tgt_pos);
	}
	catch (franka_proxy::remote_exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "Aborting Force Test, please try again." << std::endl;
		return;
	}

	// move out of contact again to prevent robot from jerking left/right - twitching still occurs, but robot doesn't move as much (most of the time)
	franka_proxy::robot_config_7dof contact = robot.current_config();
	franka_proxy::robot_config_7dof relax{ contact[0], (contact[1] - 0.01), contact[2], contact[3], contact[4], (contact[5] - 0.01), contact[6] };
	execute_retry([&] { robot.move_to(relax); }, robot);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	try {
		robot.apply_z_force(mass, duration);
	}
	catch (franka_proxy::remote_exception& e) {
		std::cout << e.what() << std::endl;
	}

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	
	std::cout << "Finished Force Test." << std::endl;
}


void impedance_admittance_ermer_ba_tests(franka_proxy::franka_remote_interface& robot, bool log)
{
	// original poses for the cartesian impedance follow test - not sure if safe
	std::list<std::array<double, 16>> original_poses = {
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


	// new poses, mirroring those of the ptp and joint impedance tests
	std::list<std::array<double, 16>> new_poses = {
		{0.999834,0.011222,0.014328,0.000000,
		0.010956,-0.999769,0.018504,0.000000,
		0.014532,-0.018343,-0.999726,0.000000,
		0.537203,-0.006444,0.366354,1.000000},

		{-0.632624,-0.121834,0.764816,0.000000,
		0.734050,-0.409155,0.541999,0.000000,
		0.246894,0.904295,0.348273,0.000000,
		0.422637,0.606252,0.913909,1.000000},

		{-0.834496,-0.024363,0.550476,0.000000,
		-0.548973,-0.049154,-0.834393,0.000000,
		0.047386,-0.998494,0.027644,0.000000,
		0.688281,-0.420545,0.376022,1.000000}
	};


	// poses for the joint impedance follow test, mirroring those of the ptp test
	std::list<std::array<double, 7>> joint_poses = {
		starting_pos,

		{0.47604, 0.650999, 1.14681, -0.747477, 0.166399, 3.03934, -0.619102},

		{-0.713442, 0.744363, 0.543357, -1.40935, -2.06861, 1.6925, -2.46015}
	};


	// stiffness for joint tests
	std::array<double, 49> stiffness{ 0. };
	stiffness[0] = stiffness[8] = stiffness[16] = stiffness[24] = 600.; // first four joints
	stiffness[32] = 250.;
	stiffness[40] = 150.;
	stiffness[48] = 50.;

	// stiffness for cartesian tests
	double rotational_stiffness = 50.;
	double translational_stiffness = 300.;


	std::cout << "Starting Impedance and Admittance Tests." << std::endl;


	std::cout << "Moving into starting position." << std::endl;
	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	std::cout << "Starting position reached." << std::endl;


	std::this_thread::sleep_for(std::chrono::seconds(3));


	std::cout << "Starting Impedance - Hold Position with Cartesian Impedance Test." << std::endl;
	std::string file = "ermer_cart_impedance_hold.csv";

	// this likes to cause trouble every now and then
	try {
		std::optional<std::string> log_file_path = std::nullopt;
		if (log) log_file_path = file;
		robot.cartesian_impedance_hold_pose(10., false, rotational_stiffness, translational_stiffness, log_file_path);
	}
	catch (franka_proxy::command_exception& e) {
		std::cout << e.what() << std::endl;
	}
	catch (franka_proxy::control_exception& f) {
		std::cout << f.what() << std::endl;
	}
	catch (franka_proxy::network_exception& g) {
		std::cout << g.what() << std::endl;
	}

	std::cout << "Finished Impedance - Hold Position with Cartesian Impedance Test." << std::endl;


	std::this_thread::sleep_for(std::chrono::seconds(3));


	std::cout << "Starting Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;
	file = "ermer_cart_impedance_follow.csv";
	
	//TODO: Robot apparently tries to snap into position, weird things happen with execute_retry wrap (plus lots of exceptions)
	// maybe related to position error? trying to fix in one go rather than over time?
	//execute_retry([&] { robot.cartesian_impedance_poses(new_poses, 30., log, file, false, 50., 300.); }, robot);
	try {
		std::optional<std::string> log_file_path = std::nullopt;
		if (log) log_file_path = file;
		robot.cartesian_impedance_poses(new_poses, 30., false, rotational_stiffness, translational_stiffness, log_file_path);
	}
	catch (franka_proxy::command_exception& e) {
		std::cout << e.what() << std::endl;
	}
	catch (franka_proxy::control_exception& f) {
		std::cout << f.what() << std::endl;
	}
	catch (franka_proxy::network_exception& g) {
		std::cout << g.what() << std::endl;
	}

	std::cout << "Finished Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;


	std::cout << "Moving back to starting position." << std::endl;
	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(starting_pos); }, robot);
	std::cout << "Starting position reached." << std::endl;


	std::this_thread::sleep_for(std::chrono::seconds(3));


	std::cout << "Starting Impedance - Hold Position with Joint Impedance Test." << std::endl;
	file = "ermer_joint_impedance_hold.csv";

	// see above
	try {
		std::optional<std::string> log_file_path = std::nullopt;
		if (log) log_file_path = file;
		robot.joint_impedance_hold_position(10., stiffness, log_file_path);
	}
	catch (franka_proxy::command_exception& e) {
		std::cout << e.what() << std::endl;
	}
	catch (franka_proxy::control_exception& f) {
		std::cout << f.what() << std::endl;
	}
	catch (franka_proxy::network_exception& g) {
		std::cout << g.what() << std::endl;
	}

	std::cout << "Finished Impedance - Hold Position with Joint Impedance Test." << std::endl;


	std::this_thread::sleep_for(std::chrono::seconds(3));


	std::cout << "Starting Impedance - Follow Poses with Joint Impedance Test." << std::endl;
	file = "ermer_joint_impedance_follow.csv";

	//TODO: same as above, with more violent effect after a delay
	//execute_retry([&] { robot.cartesian_impedance_poses(joint_poses, 30., log, file, stiffness); }, robot);
	try {
		std::optional<std::string> log_file_path = std::nullopt;
		if (log) log_file_path = file;
		robot.joint_impedance_positions(joint_poses, 30., stiffness, log_file_path);
	}
	catch (franka_proxy::command_exception& e) {
		std::cout << e.what() << std::endl;
	}
	catch (franka_proxy::control_exception& f) {
		std::cout << f.what() << std::endl;
	}
	catch (franka_proxy::network_exception& g) {
		std::cout << g.what() << std::endl;
	}

	std::cout << "Finished Impedance - Follow Poses with Joint Impedance Test." << std::endl;


	std::this_thread::sleep_for(std::chrono::seconds(3));


	std::cout << "Starting Admittance - Apply Force Test." << std::endl;
	file = "ermer_apply_admittance.csv";

	//TODO: this fails to send command constantly
	try {
		std::optional<std::string> log_file_path = std::nullopt;
		if (log) log_file_path = file;
		robot.apply_admittance(10., 10., 150., 10., 150., log_file_path);
	}
	catch (franka_proxy::command_exception& e) {
		std::cout << e.what() << std::endl;
	}
	catch (franka_proxy::control_exception& f) {
		std::cout << f.what() << std::endl;
	}
	catch (franka_proxy::network_exception& g) {
		std::cout << g.what() << std::endl;
	}

	std::cout << "Finished Admittance - Apply Force Test." << std::endl;


	std::cout << "Finished Impedance and Admittance Tests." << std::endl;
}


double calculate_pose_error(const franka_proxy::robot_config_7dof& pose_d, const franka_proxy::robot_config_7dof& pose_c) {
	double error = 0.0;
	for (int i = 0; i < 7; i++) {
		error += std::abs((pose_d[i] - pose_c[i]) / pose_d[i]);
	}

	return error / 7.0;
}
