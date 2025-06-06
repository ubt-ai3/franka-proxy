/**
 *************************************************************************
 *
 * @file py_franka_control.cpp
 *
 * Python bindings to control a franka emika panda robot.
 *
 *
 * ! Caution ! only functionality from the python example is tested.
 * E.g., python cannot handle all of Eigen's data types.
 * For Eigen::Affine3d have to use an Eigen::Matrix4d.
 * See py_* wrapper classes.
 *
 ************************************************************************/

#include <optional>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <franka_proxy_share/franka_proxy_util.hpp>
#include <franka_control/franka_controller.hpp>
#include <franka_control/franka_controller_remote.hpp>
#include <franka_control/franka_controller_emulated.hpp>


using robot_config_7dof = Eigen::Matrix<double, 7, 1>;
using wrench = Eigen::Matrix<double, 6, 1>;
using selection_diagonal = Eigen::Matrix<double, 6, 1>;

//////////////////////////////////////////////////////////////////////////
//
// py_franka_proxy_util
//
//////////////////////////////////////////////////////////////////////////
class py_franka_proxy_util
	: public franka_proxy::franka_proxy_util
{
public:
	static std::vector<Eigen::Matrix4d> fk_wrapper(
		const robot_config_7dof& config)
	{
		std::vector<Eigen::Affine3d> fk_affine3d = fk(config);

		std::vector<Eigen::Matrix4d> fk_matrix4d;
		fk_matrix4d.reserve(fk_affine3d.size());
		for (const auto& t : fk_affine3d)
			fk_matrix4d.emplace_back(t.matrix());

		return fk_matrix4d;
	}

	static robot_config_7dof ik_fast_closest_wrapper(
		const Eigen::Matrix4d& target_robot_base_T_j7,
		const robot_config_7dof& current_configuration,
		double step_size = 0.0174533) // 1 deg
	{
		Eigen::Affine3d robot_base_T_j7(target_robot_base_T_j7);
		return ik_fast_closest(
			robot_base_T_j7, current_configuration, step_size);
	}
};


PYBIND11_MODULE(py_franka_control, m)
{
	//////////////////////////////////////////////////////////////////////////
	//
	// pybind11 FrankaController
    //
	// Again ! Caution ! only functionality from the python example is tested.
	// E.g., python cannot handle all of Eigen's data types.
	// For Eigen::Affine3d have to use an Eigen::Matrix4d.
	// See py_ * wrapper classes.
	//
	//////////////////////////////////////////////////////////////////////////
	pybind11::class_<
			franka_control::franka_controller,
			std::shared_ptr<franka_control::franka_controller>>(m, "FrankaController")
		.def("move", pybind11::overload_cast<const Eigen::Affine3d&>(&franka_control::franka_controller::move))
		.def("move_until_contact",
		     pybind11::overload_cast<const Eigen::Affine3d&>(&franka_control::franka_controller::move_until_contact))
		.def("current_robot_base_T_tcp", &franka_control::franka_controller::current_robot_base_T_tcp)
		.def("current_robot_base_T_j7", &franka_control::franka_controller::current_robot_base_T_j7)
		.def("current_robot_base_T_flange", &franka_control::franka_controller::current_robot_base_T_flange)
		.def_readonly("j7_T_flange", &franka_control::franka_controller::j7_T_flange)
		.def_readonly("flange_T_tcp", &franka_control::franka_controller::flange_T_tcp)
		.def_readonly("j7_T_tcp", &franka_control::franka_controller::j7_T_tcp)
		.def_readonly("tcp_T_j7", &franka_control::franka_controller::tcp_T_j7)
		.def_readonly_static("gripper_unit_per_m", &franka_control::franka_controller::gripper_unit_per_m_);

	pybind11::class_<
			franka_control::franka_update_task,
			std::shared_ptr<franka_control::franka_update_task>>(m, "FrankaUpdateTask")
		.def(pybind11::init<franka_control::franka_controller&>());

	pybind11::class_<
			franka_control::franka_controller_remote, franka_control::franka_controller,
			std::shared_ptr<franka_control::franka_controller_remote>>(m, "FrankaControllerRemote")
		.def(pybind11::init<const std::string&>())
		.def("move", &franka_control::franka_controller_remote::move)
		.def("move_until_contact", &franka_control::franka_controller_remote::move_until_contact)
		.def("open_gripper", &franka_control::franka_controller_remote::open_gripper)
		.def("close_gripper", &franka_control::franka_controller_remote::close_gripper)
		.def("grasp_gripper", &franka_control::franka_controller_remote::grasp_gripper,
		     pybind11::arg("speed") = 0.025,
		     pybind11::arg("force") = 0.05)
		.def("gripper_grasped", &franka_control::franka_controller_remote::gripper_grasped)
		.def("speed_factor", &franka_control::franka_controller_remote::speed_factor)
		.def("set_speed_factor", &franka_control::franka_controller_remote::set_speed_factor)
		.def("automatic_error_recovery", &franka_control::franka_controller_remote::automatic_error_recovery)
		.def("current_config", &franka_control::franka_controller_remote::current_config)
		.def("current_force_torque", &franka_control::franka_controller_remote::current_force_torque)
		.def("current_gripper_pos", &franka_control::franka_controller_remote::current_gripper_pos)
		.def("max_gripper_pos", &franka_control::franka_controller_remote::max_gripper_pos)
		.def("update", &franka_control::franka_controller_remote::update)
		.def("start_recording", &franka_control::franka_controller_remote::start_recording,
		     pybind11::arg("log_file_path") = std::nullopt)
		.def("stop_recording", &franka_control::franka_controller_remote::stop_recording)
		.def("move_sequence",
		     pybind11::overload_cast<
			     const std::vector<franka_control::robot_config_7dof>&,
			     const std::vector<franka_control::wrench>&,
			     const std::vector<franka_control::selection_diagonal>&>(
			     &franka_control::franka_controller_remote::move_sequence))
		.def("move_sequence",
		     pybind11::overload_cast<
			     const std::vector<franka_control::robot_config_7dof>&,
			     const std::vector<franka_control::wrench>&,
			     const std::vector<franka_control::selection_diagonal>&,
			     std::array<double, 16>,
			     std::array<double, 6>
		     >(&franka_control::franka_controller_remote::move_sequence))
		.def("set_fts_bias", &franka_control::franka_controller_remote::set_fts_bias)
		.def("set_fts_load_mass", &franka_control::franka_controller_remote::set_fts_load_mass)
		.def("set_guiding_mode", &franka_control::franka_controller_remote::set_guiding_mode);

	pybind11::class_<
			franka_control::franka_controller_emulated, franka_control::franka_controller,
			std::shared_ptr<franka_control::franka_controller_emulated>>(m, "FrankaControllerEmulated")
		.def(pybind11::init<>())
		.def("update", &franka_control::franka_controller_emulated::update)
		.def("automatic_error_recovery", &franka_control::franka_controller_emulated::automatic_error_recovery)
		.def("move", &franka_control::franka_controller_emulated::move)
		.def("move_until_contact", &franka_control::franka_controller_emulated::move_until_contact)
		.def("open_gripper", &franka_control::franka_controller_emulated::open_gripper)
		.def("close_gripper", &franka_control::franka_controller_emulated::close_gripper)
		.def("grasp_gripper", &franka_control::franka_controller_emulated::grasp_gripper,
		     pybind11::arg("speed") = 0.025,
		     pybind11::arg("force") = 0.05)
		.def("gripper_grasped", &franka_control::franka_controller_emulated::gripper_grasped)
		.def("speed_factor", &franka_control::franka_controller_emulated::speed_factor)
		.def("set_speed_factor", &franka_control::franka_controller_emulated::set_speed_factor)
		.def("set_guiding_mode", &franka_control::franka_controller_emulated::set_guiding_mode)
		.def("current_config", &franka_control::franka_controller_emulated::current_config)
		.def("current_force_torque", &franka_control::franka_controller_emulated::current_force_torque)
		.def("current_gripper_pos", &franka_control::franka_controller_emulated::current_gripper_pos)
		.def("max_gripper_pos", &franka_control::franka_controller_emulated::max_gripper_pos)
		.def("start_recording", &franka_control::franka_controller_emulated::start_recording)
		.def("stop_recording", &franka_control::franka_controller_emulated::stop_recording)
		.def("move_sequence",
		     pybind11::overload_cast<
			     const std::vector<franka_control::robot_config_7dof>&,
			     const std::vector<franka_control::wrench>&,
			     const std::vector<franka_control::selection_diagonal>&>(
			     &franka_control::franka_controller_emulated::move_sequence))
		.def("move_sequence",
		     pybind11::overload_cast<
			     const std::vector<franka_control::robot_config_7dof>&,
			     const std::vector<franka_control::wrench>&,
			     const std::vector<franka_control::selection_diagonal>&,
			     std::array<double, 16>,
			     std::array<double, 6>
		     >(&franka_control::franka_controller_emulated::move_sequence));


	//////////////////////////////////////////////////////////////////////////
	//
	// pybind11 FrankaProxyUtil
	//
	//////////////////////////////////////////////////////////////////////////
	pybind11::class_<franka_proxy::joint_limit>(m, "JointLimit")
		.def(pybind11::init<double, double>())
		.def_readwrite("min", &franka_proxy::joint_limit::min)
		.def_readwrite("max", &franka_proxy::joint_limit::max);

	pybind11::class_<py_franka_proxy_util>(m, "FrankaProxyUtil")
		.def_static("fk", &py_franka_proxy_util::fk_wrapper,
		            pybind11::arg("config"))
		.def_static("ik_fast_closest", &py_franka_proxy_util::ik_fast_closest_wrapper,
		            pybind11::arg("target_world_T_j7"),
		            pybind11::arg("current_configuration"),
		            pybind11::arg("step_size") = 0.0174533)
		.def_static("joint_limits", &py_franka_proxy_util::joint_limits)
		.def_static("max_speed_per_joint", &py_franka_proxy_util::max_speed_per_joint)
		.def_static("max_acc_per_joint", &py_franka_proxy_util::max_acc_per_joint)
		.def_static("max_jerk_per_joint", &py_franka_proxy_util::max_jerk_per_joint)
		.def_static("is_reachable", &py_franka_proxy_util::is_reachable)
		.def_static("tool_mass", &py_franka_proxy_util::tool_mass)
		.def_static("tool_center_of_mass", &py_franka_proxy_util::tool_center_of_mass)
		.def_static("tool_inertia", &py_franka_proxy_util::tool_inertia)
		.def_readonly_static("deg_to_rad", &py_franka_proxy_util::deg_to_rad)
		.def_readonly_static("rad_to_deg", &py_franka_proxy_util::rad_to_deg);
} /* PYBIND11_MODULE(py_franka_control, m) */
