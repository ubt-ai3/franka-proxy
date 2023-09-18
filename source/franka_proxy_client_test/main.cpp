#include <atomic>
#include <iostream>
#include <thread>

#include <franka_proxy_client/exception.hpp>
#include <franka_proxy_client/franka_remote_interface.hpp>


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

	// status test
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


	//std::cout << "Starting Gripper Test." << std::endl;

	//robot.grasp_gripper(0.1);
	//robot.open_gripper(0.1);
	//robot.close_gripper(1);
	//robot.open_gripper(1);

	//std::cout << "Finished Gripper Test." << std::endl;


	std::cout << "Starting PTP-Movement Test." << std::endl;

	franka_proxy::robot_config_7dof pos1
		{{2.46732, -1.0536, -0.9351, -1.6704, 0.13675, 1.42062, 0.33471}};
	franka_proxy::robot_config_7dof pos2
		{{-0.00242, 1.236293, 2.465417, -1.26485, -0.00181, 1.914142, -1.06326}};

	robot.set_speed_factor(0.2);
	execute_retry([&] { robot.move_to(pos1); }, robot);
	execute_retry([&] { robot.move_to(pos2); }, robot);

	std::cout << "Finished PTP-Movement Test." << std::endl;


	//std::cout << "Starting Force Test." << std::endl;

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

	//std::cout << "Finished Force Test." << std::endl;


	//std::cout << "Starting Cartesian Impedance Tests." << std::endl << "Holding position with Cartesian Impedance in 3s..";
	////TODO
	//std::cout << " .. finished." << std::endl << "Following Poses with Cartesian Impedance in 3s..";
	//// positions
	//std::list<std::array<double, 16>> cart_test_positions = {
	//	{-0.0377676, 0.998866, -0.0286638, 0, 0.996603, 0.039748, 0.0719952, 0, 0.0730542, -0.0258478, -0.996993, 0, 0.748937, 0.201267, 0.111803, 1},
	//	{-0.0368831, 0.999194, -0.0151943, 0, 0.997798, 0.0376591, 0.0544201, 0, 0.0549495, -0.0131539, -0.998402, 0, 0.749475, 0.160782, 0.114082, 1},
	//	{-0.0453064, 0.998409, -0.0332923, 0, 0.996905, 0.0473265, 0.0626274, 0, 0.0641046, -0.0303524, -0.997481, 0, 0.748593, 0.132102, 0.112398, 1},
	//	{-0.0281149, 0.998188, -0.0530236, 0, 0.997021, 0.0318075, 0.0701315, 0, 0.0716923, -0.0508949, -0.996127, 0, 0.747817, 0.100581, 0.112815, 1},
	//	{-0.0307227, 0.997898, -0.0568839, 0, 0.997925, 0.033836, 0.0546024, 0, 0.0564135, -0.0550894, -0.996886, 0, 0.749112, 0.0609631, 0.110317, 1},
	//	{-0.0398797, 0.995946, -0.0805132, 0, 0.996891, 0.0451271, 0.0644415, 0, 0.0678148, -0.0776944, -0.994668, 0, 0.749454, 0.0151884, 0.108793, 1},
	//	{-0.0395714, 0.997657, -0.0556449, 0, 0.997577, 0.0426249, 0.0548037, 0, 0.0570482, -0.0533425, -0.996945, 0, 0.74799, -0.0188109, 0.108772, 1},
	//	{-0.036773, 0.996965, -0.0684852, 0, 0.997704, 0.0405157, 0.0540862, 0, 0.0566979, -0.0663404, -0.996185, 0, 0.748505, -0.051716, 0.109057, 1},
	//	{-0.0294348, 0.999175, -0.0276389, 0, 0.998475, 0.030678, 0.0456902, 0, 0.0465013, -0.0262524, -0.998573, 0, 0.747186, -0.0821434, 0.110378, 1},
	//	{-0.0541067, 0.997864, -0.0363526, 0, 0.997717, 0.0554919, 0.0382416, 0, 0.040178, -0.0342012, -0.998607, 0, 0.749775, -0.122523, 0.107051, 1},
	//	{-0.0408069, 0.997338, -0.060273, 0, 0.997717, 0.0439114, 0.0511144, 0, 0.0536261, -0.0580508, -0.996872, 0, 0.748725, -0.15092, 0.108648, 1},
	//	{-0.025413, 0.998547, -0.0473167, 0, 0.998125, 0.0279739, 0.054271, 0, 0.0555169, -0.0458496, -0.997404, 0, 0.749576, -0.167958, 0.112784, 1},
	//	{-0.0526972, 0.997159, -0.0536456, 0, 0.99692, 0.0556483, 0.0550895, 0, 0.0579194, -0.0505783, -0.997039, 0, 0.743498, -0.210845, 0.110186, 1},
	//};

	//std::this_thread::sleep_for(std::chrono::seconds(3));
	////TODO move to start pose
	//robot.cartesian_impedance_poses(cart_test_positions, 15, false, false, 50., 300.);

	//std::cout << "Finished Impedance - Follow Poses with Cartesian Impedance Test." << std::endl;


	std::cout << "Starting Joint Impedance Tests." << std::endl << "Holding position with Joint Impedance in 3s..";

	std::array<double, 49> stiffness{0.};
	stiffness[0] = stiffness[8] = stiffness[16] = stiffness[24] = 600.; // first four joints
	stiffness[32] = 250.;
	stiffness[40] = 150.;
	stiffness[48] = 50.;

	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.joint_impedance_hold_position(10, false, stiffness);

	std::cout << " .. finished." << std::endl << "Following Poses with Joint Impedance in 3s..";
	// positions
	std::list<std::array<double, 7>> joint_test_positions = {
		{0.182553, 0.955365, 0.134328, -1.17653, -0.18769, 2.18457, -0.463218},
		{0.132181, 0.921525, 0.12326, -1.22838, -0.150379, 2.19111, -0.53363},
		{0.111017, 0.892178, 0.10027, -1.28424, -0.150402, 2.22764, -0.570988},
		{0.091614, 0.866344, 0.073094, -1.33213, -0.149832, 2.25516, -0.58708},
		{0.0603157, 0.865643, 0.0436494, -1.33446, -0.1196, 2.2478, -0.657233},
		{0.0174238, 0.852878, 0.0236156, -1.3657, -0.12886, 2.27983, -0.71515},
		{-0.036821, 0.851742, 0.0278518, -1.36561, -0.0960243, 2.27401, -0.78531},
		{-0.0676432, 0.857071, 0.013558, -1.35602, -0.0957613, 2.27019, -0.823591},
		{-0.122377, 0.866343, 0.0229133, -1.32912, -0.0494514, 2.24661, -0.893418},
		{-0.148367, 0.904683, -0.0118949, -1.2703, -0.0228553, 2.21967, -0.982407},
		{-0.169821, 0.90625, -0.0290059, -1.2683, -0.0308711, 2.23793, -0.996487},
		{-0.196575, 0.917356, -0.0253515, -1.24132, -0.0163876, 2.21699, -1.01426},
		{-0.243285, 0.936971, -0.0404504, -1.21004, -0.00125092, 2.21594, -1.10456},
	};

	std::this_thread::sleep_for(std::chrono::seconds(3));
	robot.move_to(joint_test_positions.front());
	robot.joint_impedance_positions(joint_test_positions, 15, false, stiffness);
	robot.move_to(joint_test_positions.back());

	std::cout << " .. finished." << std::endl << "Finished Joint Impedance Tests." << std::endl;


	std::cout << "Starting Admittance - Test." << std::endl;

	robot.apply_admittance(10, true, 10., 150., 10., 150.);

	std::cout << "Finished Admittance - Test." << std::endl;


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


	//std::cout << ("Starting Playback Test.");

	//franka_proxy::robot_config_7dof q
	//	{{1.08615, 0.044619, 0.227112, -2.26678, -0.059792, 2.27532, 0.605723}};
	//robot.move_to(q);

	//std::cout << ("--- press to start in 3s ---");
	//std::cin.get();
	//std::this_thread::sleep_for(std::chrono::seconds(3));

	//std::cout << ("--- starting demonstration ---");
	//robot.start_recording();
	//std::this_thread::sleep_for(std::chrono::seconds(10));

	//std::cout << ("--- stopped demonstration ---");
	//std::pair record
	//(
	//	robot.stop_recording());

	//std::cout << ("--- press to start reproduction in 3s ---");
	//std::cin.get();
	//std::this_thread::sleep_for(std::chrono::seconds(3));

	//robot.move_to(q);
	//robot.move_to(record.first.front());
	//const std::vector<std::array<double, 6>> selection_vectors
	//(
	//	record.second.size(), std::array<double, 6>{1, 1, 1, 1, 1, 1});
	//robot.move_sequence(record.first, record.second, selection_vectors);

	//std::cout << ("Finished Playback Test.");


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
