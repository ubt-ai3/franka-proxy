#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <optional>

#include <franka_controller.hpp>
#include <franka_controller_remote.hpp>
#include <franka_controller_emulated.hpp>

namespace py = pybind11;
namespace fc = franka_control;

PYBIND11_MODULE(py_franka_control, m)
{
	py::class_<fc::franka_controller, std::shared_ptr<fc::franka_controller>>(m, "FrankaController")
		// Base class methods not overridden in derived classes (remote and emulated controllers)
		.def("move", py::overload_cast<const Eigen::Affine3d&>(&fc::franka_controller::move))
		.def("move_until_contact",
		     py::overload_cast<const Eigen::Affine3d&>(&fc::franka_controller::move_until_contact))
		.def("current_world_T_tcp", &fc::franka_controller::current_world_T_tcp)
		.def("current_world_T_j7", &fc::franka_controller::current_world_T_j7)
		.def("current_world_T_flange", &fc::franka_controller::current_world_T_flange)
		.def_readonly("j7_T_flange", &fc::franka_controller::j7_T_flange)
		.def_readonly("flange_T_tcp", &fc::franka_controller::flange_T_tcp)
		.def_readonly("j7_T_tcp", &fc::franka_controller::j7_T_tcp)
		.def_readonly("tcp_T_j7", &fc::franka_controller::tcp_T_j7)
		.def_readonly_static("gripper_unit_per_m", &fc::franka_controller::gripper_unit_per_m_);

	py::class_<fc::franka_update_task, std::shared_ptr<fc::franka_update_task>>(m, "FrankaUpdateTask")
		.def(py::init<fc::franka_controller&>());

	py::class_<fc::franka_controller_remote, fc::franka_controller, std::shared_ptr<fc::franka_controller_remote>>(
			m, "FrankaControllerRemote")
		.def(py::init<const std::string&>())
		.def("move", &fc::franka_controller_remote::move)
		.def("move_until_contact", &fc::franka_controller_remote::move_until_contact)
		.def("open_gripper", &fc::franka_controller_remote::open_gripper)
		.def("close_gripper", &fc::franka_controller_remote::close_gripper)
		.def("grasp_gripper", &fc::franka_controller_remote::grasp_gripper, py::arg("speed") = 0.025,
		     py::arg("force") = 0.05)
		.def("gripper_grasped", &fc::franka_controller_remote::gripper_grasped)
		.def("speed_factor", &fc::franka_controller_remote::speed_factor)
		.def("set_speed_factor", &fc::franka_controller_remote::set_speed_factor)
		.def("automatic_error_recovery", &fc::franka_controller_remote::automatic_error_recovery)
		.def("current_config", &fc::franka_controller_remote::current_config)
		.def("current_force_torque", &fc::franka_controller_remote::current_force_torque)
		.def("current_gripper_pos", &fc::franka_controller_remote::current_gripper_pos)
		.def("max_gripper_pos", &fc::franka_controller_remote::max_gripper_pos)
		.def("update", &fc::franka_controller_remote::update)
		.def("start_recording", &fc::franka_controller_remote::start_recording, py::arg("log_file_path") = std::nullopt)
		.def("stop_recording", &fc::franka_controller_remote::stop_recording)
		.def("move_sequence",
		     py::overload_cast<
			     const std::vector<fc::robot_config_7dof>&,
			     const std::vector<fc::wrench>&,
			     const std::vector<fc::selection_diagonal>&>(&fc::franka_controller_remote::move_sequence))
		.def("move_sequence",
		     py::overload_cast<
			     const std::vector<fc::robot_config_7dof>&,
			     const std::vector<fc::wrench>&,
			     const std::vector<fc::selection_diagonal>&,
			     std::array<double, 16>,
			     std::array<double, 6>
		     >(&fc::franka_controller_remote::move_sequence))
		.def("set_fts_bias", &fc::franka_controller_remote::set_fts_bias)
		.def("set_fts_load_mass", &fc::franka_controller_remote::set_fts_load_mass)
		.def("set_guiding_mode", &fc::franka_controller_remote::set_guiding_mode);

	py::class_<fc::franka_controller_emulated, fc::franka_controller, std::shared_ptr<fc::franka_controller_emulated>>(
			m, "FrankaControllerEmulated")
		.def(py::init<>())
		.def("update", &fc::franka_controller_emulated::update)
		.def("automatic_error_recovery", &fc::franka_controller_emulated::automatic_error_recovery)
		.def("move", &fc::franka_controller_emulated::move)
		.def("move_until_contact", &fc::franka_controller_emulated::move_until_contact)
		.def("open_gripper", &fc::franka_controller_emulated::open_gripper)
		.def("close_gripper", &fc::franka_controller_emulated::close_gripper)
		.def("grasp_gripper", &fc::franka_controller_emulated::grasp_gripper, py::arg("speed") = 0.025,
		     py::arg("force") = 0.05)
		.def("gripper_grasped", &fc::franka_controller_emulated::gripper_grasped)
		.def("speed_factor", &fc::franka_controller_emulated::speed_factor)
		.def("set_speed_factor", &fc::franka_controller_emulated::set_speed_factor)
		.def("set_guiding_mode", &fc::franka_controller_emulated::set_guiding_mode)
		.def("current_config", &fc::franka_controller_emulated::current_config)
		.def("current_force_torque", &fc::franka_controller_emulated::current_force_torque)
		.def("current_gripper_pos", &fc::franka_controller_emulated::current_gripper_pos)
		.def("max_gripper_pos", &fc::franka_controller_emulated::max_gripper_pos)
		.def("start_recording", &fc::franka_controller_emulated::start_recording)
		.def("stop_recording", &fc::franka_controller_emulated::stop_recording)
		.def("move_sequence",
		     py::overload_cast<
			     const std::vector<fc::robot_config_7dof>&,
			     const std::vector<fc::wrench>&,
			     const std::vector<fc::selection_diagonal>&>(&fc::franka_controller_emulated::move_sequence))
		.def("move_sequence",
		     py::overload_cast<
			     const std::vector<fc::robot_config_7dof>&,
			     const std::vector<fc::wrench>&,
			     const std::vector<fc::selection_diagonal>&,
			     std::array<double, 16>,
			     std::array<double, 6>
		     >(&fc::franka_controller_emulated::move_sequence));
}
