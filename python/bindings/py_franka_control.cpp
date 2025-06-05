/**
 *************************************************************************
 *
 * @file py_franka_control.cpp
 *
 * Python bindings to control a franka emika panda robot.
 *
 ************************************************************************/


#include <optional>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <franka_controller.hpp>
#include <franka_controller_remote.hpp>
#include <franka_controller_emulated.hpp>


PYBIND11_MODULE(py_franka_control, m)
{
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

	pybind11::class_<franka_control::franka_controller_remote, franka_control::franka_controller,
	                 std::shared_ptr<franka_control::franka_controller_remote>>(m, "FrankaControllerRemote")
		.def(pybind11::init<const std::string&>())
		.def("move", &franka_control::franka_controller_remote::move)
		.def("move_until_contact", &franka_control::franka_controller_remote::move_until_contact)
		.def("open_gripper", &franka_control::franka_controller_remote::open_gripper)
		.def("close_gripper", &franka_control::franka_controller_remote::close_gripper)
		.def("grasp_gripper", &franka_control::franka_controller_remote::grasp_gripper, pybind11::arg("speed") = 0.025,
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
}
