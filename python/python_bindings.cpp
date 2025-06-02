#include <optional>
#include <nlohmann/json.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include <franka_control/franka_controller_remote.hpp>
#include <franka_proxy_commands.hpp>
#include <franka_proxy_client/exception.hpp>
#include <franka_network_client.hpp>
#include <franka_remote_interface.hpp>
#include <franka_proxy_util.hpp>

/**
 *************************************************************************
 *
 * @file python_bindings.cpp
 *
 * Bindings for basically all classes in the franka_proxy project
 * Abstract classes need so called trampoline classes if you want to override them from python. It was also necessary because the file wouldnt compile otherwise
 * Some functions need wrappers especially because some datastructures are not supported by python (e.g. to_json)
 * Some functions need either wrappers or use lambda expressions to directly change datastructures in the binding itself (e.g. franka_proxy_util.fk)
 * Some functions need a new class inheriting from the orignial class with just one function redefined because for some reason the two ways above did not work (e.g. franka_proxy_util.ik_fast_closest -> py_franka_proxy_util.ik_fast_closest_wrapper)
 *
 * 
 * In general you bind each class/function or whatever you want in your python module
 * Binding in general works like this: 
 * (franka_proxy_module is the name of the output module in this case and m is the internal reference to the module)
 * 
 * PYBIND11_MODULE(franka_proxy_module, m){
 *      py::class_<example::example>(m, "franka_controller")
                .def(py::init<std::string>(), py::arg("ip"))                                                     //binding for constructor with its parameters    
                .def("move_to_until_contact", &example::example::move_to_until_contact,                          //binding of class function
                     py::arg("target"))
                .def("move", static_cast<void (example::example::*)(const Eigen::Matrix<double, 7, 1>&)>         //binding for function with override have to specifiy each override you want in your module
                     (&example::example::move))
                .def_readwrite("robot_wrench", &example::robot_config_7dof_wrench::second);                      //binding for public attribute, class stopped by ;

        m.def("to_json", &to_json_wrapper<example::command_move_to_config>, py::arg("object"));                  //binding for non class function
 * }
 ************************************************************************/

using robot_config_7dof = Eigen::Matrix<double, 7, 1>;
using wrench = Eigen::Matrix<double, 6, 1>;
using selection_diagonal = Eigen::Matrix<double, 6, 1>;

//PYBIND11_MAKE_OPAQUE(std::pair<std::vector<robot_config_7dof>, std::vector<wrench>>);

namespace py = pybind11;

template <typename T>
nlohmann::json to_json_wrapper(const T& obj){
        nlohmann::json json;
        franka_proxy::to_json(json, obj);
        return json;
}

template <typename T>
nlohmann::json from_json_wrapper(const nlohmann::json& json){
        T obj;
        franka_proxy::from_json(json, obj);
        return obj;
}

template <typename TCommandType>
typename TCommandType::response_type send_command_wrapper(const TCommandType& command){
        return franka_proxy::franka_control_client::send_command(command);
}

class py_franka_proxy_util : public franka_proxy::franka_proxy_util{
        public:

                static robot_config_7dof ik_fast_closest_wrapper
                (const Eigen::Matrix4d& target_world_T_j7, 
                        robot_config_7dof& current_configuration,
                        double step_size = 0.174533){
                                Eigen::Affine3d transfo(target_world_T_j7);
                                return ik_fast_closest(transfo, current_configuration, step_size);
                        }
};

class py_franka_controller : public franka_control::franka_controller {
	public:
		using franka_control::franka_controller::franka_controller;
		

		void move(const robot_config_7dof& target) override {
        	PYBIND11_OVERRIDE_PURE(void, franka_controller, move, target);
    	}

		bool move_until_contact(const robot_config_7dof& target) override {
			PYBIND11_OVERRIDE_PURE(bool, franka_controller, move_until_contact, target);
		}

		void open_gripper() override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, open_gripper);
		}

		void close_gripper() override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, close_gripper);
		}

		void grasp_gripper(double speed, double force) override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, grasp_gripper, speed, force);
		}

		bool gripper_grasped() const override {
			PYBIND11_OVERRIDE_PURE(bool, franka_controller, gripper_grasped);
		}

		double speed_factor() const override {
			PYBIND11_OVERRIDE_PURE(double, franka_controller, speed_factor);
		}

		void set_speed_factor(double speed_factor) override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, set_speed_factor, speed_factor);
		}

		void set_guiding_mode(bool x, bool y, bool z, bool rx, bool ry, bool rz, bool elbow) const override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, set_guiding_mode, x, y, z, rx, ry, rz, elbow);
		}

		void automatic_error_recovery() override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, automatic_error_recovery);
		}

		robot_config_7dof current_config() const override {
			PYBIND11_OVERRIDE_PURE(robot_config_7dof, franka_controller, current_config);
		}

		wrench current_force_torque() const override {
			PYBIND11_OVERRIDE_PURE(wrench, franka_controller, current_force_torque);
		}

		int current_gripper_pos() const override {
			PYBIND11_OVERRIDE_PURE(int, franka_controller, current_gripper_pos);
		}

		int max_gripper_pos() const override {
			PYBIND11_OVERRIDE_PURE(int, franka_controller, max_gripper_pos);
		}		

		void update() override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, update);
		}

		void start_recording(std::optional<std::string> log_file_path) override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, start_recording, log_file_path);
		}

		franka_control::robot_config_7dof_wrench stop_recording() override {
			PYBIND11_OVERRIDE_PURE(
				franka_control::robot_config_7dof_wrench,
				franka_controller,
				stop_recording
			);
		}

		void move_sequence(const std::vector<robot_config_7dof>& q_sequence,
						const std::vector<wrench>& f_sequence,
						const std::vector<selection_diagonal>& selection_vector_sequence) override {
			PYBIND11_OVERRIDE_PURE(void, franka_controller, move_sequence, q_sequence, f_sequence, selection_vector_sequence);
		}
};

PYBIND11_MODULE(franka_proxy_module, m){

	py::class_<franka_control::robot_config_7dof_wrench>(m, "robot_config_7dof_wrench")
		.def(py::init<std::vector<robot_config_7dof>, std::vector<wrench>>())
                .def_readwrite("robot_config", &franka_control::robot_config_7dof_wrench::first)
                .def_readwrite("robot_wrench", &franka_control::robot_config_7dof_wrench::second);
	
	py::class_<franka_control::franka_controller, py_franka_controller>(m, "franka_controller")
		.def(py::init<>())
        //.def("move", py::overload_cast<const robot_config_7dof&>(&franka_control::franka_controller::move))
		.def("move", static_cast<void (franka_control::franka_controller::*)(const Eigen::Matrix<double, 7, 1>&)>(&franka_control::franka_controller::move))
                .def("move_until_contact", static_cast<bool (franka_control::franka_controller::*)(const Eigen::Matrix<double, 7, 1>&)>(&franka_control::franka_controller::move_until_contact))
                .def("open_gripper", &franka_control::franka_controller::open_gripper)
                .def("close_gripper", &franka_control::franka_controller::close_gripper)
                .def("grasp_gripper", &franka_control::franka_controller::grasp_gripper, py::arg("speed") = 0.025, py::arg("force") = 0.05)
                .def("gripper_grasped", &franka_control::franka_controller::gripper_grasped)
                .def("speed_factor", &franka_control::franka_controller::speed_factor)
                .def("set_speed_factor", &franka_control::franka_controller::set_speed_factor)
                .def("set_guiding_mode", &franka_control::franka_controller::set_guiding_mode)
                .def("automatic_error_recovery", &franka_control::franka_controller::automatic_error_recovery)
                .def("current_config", &franka_control::franka_controller::current_config)
                .def("current_force_torque", &franka_control::franka_controller::current_force_torque)
                .def("current_gripper_pos", &franka_control::franka_controller::current_gripper_pos)
                .def("max_gripper_pos", &franka_control::franka_controller::max_gripper_pos)
                .def("update", &franka_control::franka_controller::update)
                .def("start_recording", &franka_control::franka_controller::start_recording, py::arg("log_file_path") = std::nullopt)
                .def("stop_recording", &franka_control::franka_controller::stop_recording)
                .def("move_sequence", &franka_control::franka_controller::move_sequence)
                .def("current_world_T_tcp", &franka_control::franka_controller::current_world_T_tcp)
                .def("current_world_T_j7", &franka_control::franka_controller::current_world_T_j7)
                .def("current_world_T_flange", &franka_control::franka_controller::current_world_T_flange)
                .def_readonly("j7_T_flange", &franka_control::franka_controller::j7_T_flange)
                .def_readonly("flange_T_tcp", &franka_control::franka_controller::flange_T_tcp)
                .def_readonly("j7_T_tcp", &franka_control::franka_controller::j7_T_tcp)
                .def_readonly("tcp_T_j7", &franka_control::franka_controller::tcp_T_j7);

	py::class_<franka_control::franka_update_task>(m, "franka_update_task")
                .def(py::init<franka_control::franka_controller&>());

	py::class_<franka_control::franka_controller_remote, franka_control::franka_controller>(m, "franka_controller_remote")
		.def(py::init<const std::string &>())
                .def("move", &franka_control::franka_controller_remote::move)
                .def("move_until_contact", &franka_control::franka_controller_remote::move_until_contact)
                .def("open_gripper", &franka_control::franka_controller_remote::open_gripper)
                .def("close_gripper", &franka_control::franka_controller_remote::close_gripper)
                .def("grasp_gripper", &franka_control::franka_controller_remote::grasp_gripper,
                py::arg("speed") = 0.025, py::arg("force") = 0.05)
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
                py::arg("log_file_path") = std::nullopt)
                .def("stop_recording", &franka_control::franka_controller_remote::stop_recording)
                .def("move_sequence", &franka_control::franka_controller_remote::move_sequence)
                .def("set_fts_bias", &franka_control::franka_controller_remote::set_fts_bias)
                .def("set_fts_load_mass", &franka_control::franka_controller_remote::set_fts_load_mass)
                .def("set_guiding_mode", &franka_control::franka_controller_remote::set_guiding_mode);

        py::class_<franka_proxy::command_move_to_config>(m, "command_move_to_config")
                .def(py::init<>())
                .def_readwrite("target_joint_config", &franka_proxy::command_move_to_config::target_joint_config)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_move_to_config::type;
                });

        py::class_<franka_proxy::command_move_until_contact>(m, "command_move_until_contact")
                .def(py::init<>())
                .def_readwrite("target_joint_config", &franka_proxy::command_move_until_contact::target_joint_config)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_move_until_contact::type;
                });

        py::class_<franka_proxy::command_move_hybrid_sequence>(m, "command_move_hybrid_sequence")
                .def(py::init<>())
                .def_readwrite("joint_config_sequence", &franka_proxy::command_move_hybrid_sequence::joint_config_sequence)
                .def_readwrite("force_sequence", &franka_proxy::command_move_hybrid_sequence::force_sequence)
                .def_readwrite("selection_sequence", &franka_proxy::command_move_hybrid_sequence::selection_sequence)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_move_hybrid_sequence::type;
                });
        
        py::class_<franka_proxy::command_apply_admittance_adm_imp_desired_stiffness>(m, "command_apply_admittance_adm_imp_desired_stiffness")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::duration)
                .def_readwrite("adm_rotational_stiffness", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::adm_rotational_stiffness)
                .def_readwrite("adm_translational_stiffness", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::adm_translational_stiffness)
                .def_readwrite("imp_rotational_stiffness", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::imp_rotational_stiffness)
                .def_readwrite("imp_translational_stiffness", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::imp_translational_stiffness)
                .def_readwrite("log_file_path", &franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_apply_admittance_adm_imp_desired_stiffness::type;
                });

        py::class_<franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness>(m, "command_cartesian_impedance_hold_pose_desired_stiffness")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::duration)
                .def_readwrite("use_stiff_damp_online_calc", &franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::use_stiff_damp_online_calc)
                .def_readwrite("rotational_stiffness", &franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::rotational_stiffness)
                .def_readwrite("translational_stiffness", &franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::translational_stiffness)
                .def_readwrite("log_file_path", &franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness::type;
                });

        py::class_<franka_proxy::command_cartesian_impedance_poses_desired_stiffness>(m, "command_cartesian_impedance_poses_desired_stiffness")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_cartesian_impedance_poses_desired_stiffness::duration)
                .def_readwrite("use_stiff_damp_online_calc", &franka_proxy::command_cartesian_impedance_poses_desired_stiffness::use_stiff_damp_online_calc)
                .def_readwrite("rotational_stiffness", &franka_proxy::command_cartesian_impedance_poses_desired_stiffness::rotational_stiffness)
                .def_readwrite("translational_stiffness", &franka_proxy::command_cartesian_impedance_poses_desired_stiffness::translational_stiffness)
                .def_readwrite("log_file_path", &franka_proxy::command_cartesian_impedance_poses_desired_stiffness::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_cartesian_impedance_poses_desired_stiffness::type;
                });

        py::class_<franka_proxy::command_joint_impedance_hold_position_desired_stiffness>(m, "command_joint_impedance_hold_position_desired_stiffness")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_joint_impedance_hold_position_desired_stiffness::duration)
                .def_readwrite("stiffness", &franka_proxy::command_joint_impedance_hold_position_desired_stiffness::stiffness)
                .def_readwrite("log_file_path", &franka_proxy::command_joint_impedance_hold_position_desired_stiffness::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_joint_impedance_hold_position_desired_stiffness::type;
                });

        py::class_<franka_proxy::command_joint_impedance_positions_desired_stiffness>(m, "command_joint_impedance_positions_desired_stiffness")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_joint_impedance_positions_desired_stiffness::duration)
                .def_readwrite("joint_positions", &franka_proxy::command_joint_impedance_positions_desired_stiffness::joint_positions)
                .def_readwrite("stiffness", &franka_proxy::command_joint_impedance_positions_desired_stiffness::stiffness)
                .def_readwrite("log_file_path", &franka_proxy::command_joint_impedance_positions_desired_stiffness::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_joint_impedance_positions_desired_stiffness::type;
                });


        py::class_<franka_proxy::command_ple_motion>(m, "command_ple_motion")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_ple_motion::duration)
                .def_readwrite("speed", &franka_proxy::command_ple_motion::speed)
                .def_readwrite("log_file_path", &franka_proxy::command_ple_motion::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_ple_motion::type;
                });

        py::class_<franka_proxy::command_force_z>(m, "command_force_z")
                .def(py::init<>())
                .def_readwrite("duration", &franka_proxy::command_force_z::duration)
                .def_readwrite("mass", &franka_proxy::command_force_z::mass)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_force_z::type;
                });

        py::class_<franka_proxy::command_open_gripper>(m, "command_open_gripper")
                .def(py::init<>())
                .def_readwrite("speed", &franka_proxy::command_open_gripper::speed)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_open_gripper::type;
                });

        py::class_<franka_proxy::command_close_gripper>(m, "command_close_gripper")
                .def(py::init<>())
                .def_readwrite("speed", &franka_proxy::command_close_gripper::speed)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_close_gripper::type;
                });

        py::class_<franka_proxy::command_grasp_gripper>(m, "command_grasp_gripper")
                .def(py::init<>())
                .def_readwrite("speed", &franka_proxy::command_grasp_gripper::speed)
                .def_readwrite("force", &franka_proxy::command_grasp_gripper::force)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_grasp_gripper::type;
                });

        py::class_<franka_proxy::command_vacuum_gripper_drop>(m, "command_vacuum_gripper_drop")
                .def(py::init<>())
                .def_readwrite("timeout", &franka_proxy::command_vacuum_gripper_drop::timeout)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_vacuum_gripper_drop::type;
                });

        py::class_<franka_proxy::command_vacuum_gripper_vacuum>(m, "command_vacuum_gripper_vacuum")
                .def(py::init<>())
                .def_readwrite("timeout", &franka_proxy::command_vacuum_gripper_vacuum::timeout)
                .def_readwrite("vacuum_strength", &franka_proxy::command_vacuum_gripper_vacuum::vacuum_strength)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_vacuum_gripper_vacuum::type;
                });

        py::class_<franka_proxy::command_vacuum_gripper_stop>(m, "command_vacuum_gripper_stop")
                .def(py::init<>())
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_vacuum_gripper_stop::type;
                });

        py::class_<franka_proxy::command_start_recording>(m, "command_start_recording")
                .def(py::init<>())
                .def_readwrite("log_file_path", &franka_proxy::command_start_recording::log_file_path)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_start_recording::type;
                });

        py::class_<franka_proxy::command_stop_recording>(m, "command_stop_recording")
                .def(py::init<>())
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_stop_recording::type;
                });

        py::class_<franka_proxy::command_stop_recording_response>(m, "command_stop_recording_response")
                .def(py::init<>())
                .def_readwrite("joint_config_sequence", &franka_proxy::command_stop_recording_response::joint_config_sequence)
                .def_readwrite("force_sequence", &franka_proxy::command_stop_recording_response::force_sequence)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_stop_recording_response::type;
                });
        
        py::class_<franka_proxy::command_set_speed>(m, "command_set_speed")
                .def(py::init<>())
                .def_readwrite("speed", &franka_proxy::command_set_speed::speed)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_set_speed::type;
                });

        py::class_<franka_proxy::command_set_fts_bias>(m, "command_set_fts_bias")
                .def(py::init<>())
                .def_readwrite("bias", &franka_proxy::command_set_fts_bias::bias)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_set_fts_bias::type;
                });

        py::class_<franka_proxy::command_set_fts_load_mass>(m, "command_set_fts_load_mass")
                .def(py::init<>())
                .def_readwrite("load_mass", &franka_proxy::command_set_fts_load_mass::load_mass)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_set_fts_load_mass::type;
                });

        py::class_<franka_proxy::command_set_guiding_params>(m, "command_set_guiding_params")
                .def(py::init<>())
                .def_readwrite("guiding_config", &franka_proxy::command_set_guiding_params::guiding_config)
                .def_readwrite("elbow", &franka_proxy::command_set_guiding_params::elbow)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_set_guiding_params::type;
                });

        py::class_<franka_proxy::command_recover_from_errors>(m, "command_recover_from_errors")
                .def(py::init<>())
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_recover_from_errors::type;
                });

        py::enum_<franka_proxy::command_result>(m, "command_result", py::arithmetic())
                .value("SUCCESS", franka_proxy::command_result::success)
                .value("SUCCESS_COMMAND_FAILED", franka_proxy::command_result::success_command_failed)
                .value("MODEL_EXCEPTION", franka_proxy::command_result::model_exception)
                .value("NETWORK_EXCEPTION", franka_proxy::command_result::network_exception)
                .value("PROTOCOL_EXCEPTION", franka_proxy::command_result::protocol_exception)
                .value("INCOMPATIBLE_VERSION", franka_proxy::command_result::incompatible_version)
                .value("CONTROL_EXCEPTION", franka_proxy::command_result::control_exception)
                .value("COMMAND_EXCEPTION", franka_proxy::command_result::command_exception)
                .value("REALTIME_EXCEPTION", franka_proxy::command_result::realtime_exception)
                .value("INVALID_OPERATION", franka_proxy::command_result::invalid_operation)
                .value("FRANKA_EXCEPTION", franka_proxy::command_result::franka_exception)
                .value("FORCE_TORQUE_SENSOR_EXCEPTION", franka_proxy::command_result::force_torque_sensor_exception)
                .value("UNKNOWN_COMMAND", franka_proxy::command_result::unknown_command)
                .export_values();

        py::class_<franka_proxy::command_generic_response>(m, "command_generic_response")
                .def(py::init<>())
                .def(py::init<franka_proxy::command_result>())
                .def(py::init<franka_proxy::command_result, std::string>())
                .def_readwrite("result", &franka_proxy::command_generic_response::result)
                .def_readwrite("reason", &franka_proxy::command_generic_response::reason)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_generic_response::type;
                });

        py::class_<franka_proxy::command_get_config>(m, "command_get_config")
                .def(py::init<>())
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_get_config::type;
                });

        py::class_<franka_proxy::command_get_config_response>(m, "command_get_config_response")
                .def(py::init<>())
                .def_readwrite("joint_configuration", &franka_proxy::command_get_config_response::joint_configuration)
                .def_readwrite("width", &franka_proxy::command_get_config_response::width)
                .def_readwrite("max_width", &franka_proxy::command_get_config_response::max_width)
                .def_readwrite("is_grasped", &franka_proxy::command_get_config_response::is_grasped)
                .def_readwrite("actual_power", &franka_proxy::command_get_config_response::actual_power)
                .def_readwrite("vacuum", &franka_proxy::command_get_config_response::vacuum)
                .def_readwrite("part_detached", &franka_proxy::command_get_config_response::part_detached)
                .def_readwrite("part_present", &franka_proxy::command_get_config_response::part_present)
                .def_readwrite("in_control_range", &franka_proxy::command_get_config_response::in_control_range)
                .def_property_readonly_static("type", [](py::object){
                        return franka_proxy::command_get_config_response::type;
                });

        m.def("to_json", &to_json_wrapper<franka_proxy::command_move_to_config>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_move_until_contact>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_move_hybrid_sequence>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_apply_admittance_adm_imp_desired_stiffness>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_cartesian_impedance_poses_desired_stiffness>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_joint_impedance_hold_position_desired_stiffness>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_joint_impedance_positions_desired_stiffness>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_ple_motion>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_force_z>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_open_gripper>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_close_gripper>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_grasp_gripper>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_vacuum_gripper_drop>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_vacuum_gripper_vacuum>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_vacuum_gripper_stop>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_start_recording>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_stop_recording>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_stop_recording_response>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_set_speed>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_set_fts_bias>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_set_fts_load_mass>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_set_guiding_params>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_recover_from_errors>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_generic_response>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_get_config>, py::arg("object"));
        m.def("to_json", &to_json_wrapper<franka_proxy::command_get_config_response>, py::arg("object"));

        m.def("from_json", &from_json_wrapper<franka_proxy::command_move_to_config>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_move_until_contact>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_move_hybrid_sequence>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_apply_admittance_adm_imp_desired_stiffness>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_cartesian_impedance_poses_desired_stiffness>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_joint_impedance_hold_position_desired_stiffness>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_joint_impedance_positions_desired_stiffness>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_ple_motion>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_force_z>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_open_gripper>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_close_gripper>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_grasp_gripper>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_vacuum_gripper_drop>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_vacuum_gripper_vacuum>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_vacuum_gripper_stop>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_start_recording>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_stop_recording>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_stop_recording_response>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_set_speed>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_set_fts_bias>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_set_fts_load_mass>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_set_guiding_params>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_recover_from_errors>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_generic_response>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_get_config>, py::arg("object"));
        m.def("from_json", &from_json_wrapper<franka_proxy::command_get_config_response>, py::arg("object"));

        py::register_exception<franka_proxy::exception>(m, "exception");
        py::register_exception<franka_proxy::remote_exception>(m, "remote_exception", m.attr("exception").ptr());
        py::register_exception<franka_proxy::model_exception>(m, "model_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::network_exception>(m, "network_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::protocol_exception>(m, "protocol_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::incompatible_version_exception>(m, "incompatible_version_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::control_exception>(m, "control_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::command_exception>(m, "command_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::realtime_exception>(m, "realtime_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::invalid_operation_exception>(m, "invalid_operation_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::ft_sensor_exception>(m, "ft_sensor_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::unknown_command_exception>(m, "unknown_command_exception", m.attr("remote_exception").ptr());
        py::register_exception<franka_proxy::bad_response_exception>(m, "bad_response_exception", m.attr("exception").ptr());

        py::class_<franka_proxy::franka_state_client>(m, "franka_state_client")
                .def(py::init<std::string, std::uint16_t>(), py::arg("remote_ip"), py::arg("remote_port"))
                .def("update_messages", &franka_proxy::franka_state_client::update_messages)
                .def("states", &franka_proxy::franka_state_client::states)
                .def("clear_states", &franka_proxy::franka_state_client::clear_states);

        py::class_<franka_proxy::franka_control_client>(m, "franka_control_client")
                .def(py::init<std::string, std::uint16_t>(), py::arg("remote_ip"), py::arg("remote_port"))
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_move_to_config>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_move_until_contact>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_move_hybrid_sequence>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_apply_admittance_adm_imp_desired_stiffness>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_cartesian_impedance_hold_pose_desired_stiffness>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_cartesian_impedance_poses_desired_stiffness>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_joint_impedance_hold_position_desired_stiffness>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_joint_impedance_positions_desired_stiffness>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_ple_motion>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_force_z>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_open_gripper>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_close_gripper>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_grasp_gripper>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_vacuum_gripper_drop>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_vacuum_gripper_vacuum>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_vacuum_gripper_stop>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_start_recording>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_stop_recording>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)                
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_set_speed>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_set_fts_bias>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_set_fts_load_mass>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_set_guiding_params>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_recover_from_errors>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f)
                .def("send_command", &franka_proxy::franka_control_client::send_command<franka_proxy::command_get_config>,
                     py::arg("command"), py::arg("timeout_seconds") = 1.f);

        py::class_<franka_proxy::vacuum_gripper_state>(m, "vacuum_gripper_state")
                .def(py::init<>())
                .def_readwrite("actual_power", &franka_proxy::vacuum_gripper_state::actual_power_)
                .def_readwrite("vacuum_level", &franka_proxy::vacuum_gripper_state::vacuum_level)
                .def_readwrite("part_detached_", &franka_proxy::vacuum_gripper_state::part_detached_)
                .def_readwrite("part_present_", &franka_proxy::vacuum_gripper_state::part_present_)
                .def_readwrite("in_control_range_", &franka_proxy::vacuum_gripper_state::in_control_range_);
        
        py::class_<franka_proxy::franka_remote_interface>(m, "franka_remote_interface")
                .def(py::init<std::string>(), py::arg("proxy_ip"))
                .def("move_to", py::overload_cast<const robot_config_7dof&>(&franka_proxy::franka_remote_interface::move_to),
                     py::arg("target"))
                .def("move_to", py::overload_cast<const Eigen::Vector<double, 7>&>(&franka_proxy::franka_remote_interface::move_to),
                     py::arg("target"))
                .def("move_to_until_contact", &franka_proxy::franka_remote_interface::move_to_until_contact,
                     py::arg("target"))
                .def("move_sequence", &franka_proxy::franka_remote_interface::move_sequence,
                     py::arg("q_sequence"), py::arg("f_sequence"), py::arg("selection_vector_sequence"))
                .def("apply_admittance", &franka_proxy::franka_remote_interface::apply_admittance,
                     py::arg("duration"), py::arg("adm_rotational_stiffness"), py::arg("adm_translational_stiffness"),
                     py::arg("imp_rotational_stiffness"), py::arg("imp_translational_stiffness"), py::arg("log_file_path"))
                .def("cartesian_impedance_hold_pose", &franka_proxy::franka_remote_interface::cartesian_impedance_hold_pose,
                     py::arg("duration"), py::arg("use_stiff_damp_online_calc"), py::arg("rotational_stiffness"),
                     py::arg("translational_stiffness"), py::arg("log_file_path"))
                .def("cartesian_impedance_poses", &franka_proxy::franka_remote_interface::cartesian_impedance_poses,
                     py::arg("positions"), py::arg("duration"), py::arg("use_stiff_damp_online_calc"),
                     py::arg("rotational_stiffness"), py::arg("translational_stiffness"), py::arg("log_file_path"))
                .def("joint_impedance_hold_position", &franka_proxy::franka_remote_interface::joint_impedance_hold_position,
                     py::arg("duration"), py::arg("stiffness"), py::arg("rotational_stiffness"))
                .def("joint_impedance_positions", &franka_proxy::franka_remote_interface::joint_impedance_positions,
                     py::arg("joint_positions"), py::arg("duration"), py::arg("stiffness"),
                     py::arg("log_file_path"))
                .def("ple_motion", &franka_proxy::franka_remote_interface::ple_motion,
                     py::arg("speed"), py::arg("duration"), py::arg("log_file_path"))
                .def("apply_z_force", &franka_proxy::franka_remote_interface::apply_z_force,
                     py::arg("mass"), py::arg("duration"))
                .def("open_gripper", &franka_proxy::franka_remote_interface::open_gripper,
                     py::arg("speed"))
                .def("close_gripper", &franka_proxy::franka_remote_interface::close_gripper,
                     py::arg("speed"))
                .def("grasp_gripper", &franka_proxy::franka_remote_interface::grasp_gripper,
                     py::arg("speed"), py::arg("force"))
                .def("vacuum_gripper_drop", &franka_proxy::franka_remote_interface::vacuum_gripper_drop,
                     py::arg("timeout"))
                .def("vacuum_gripper_vacuum", &franka_proxy::franka_remote_interface::vacuum_gripper_vacuum,
                     py::arg("vacuum_strength"), py::arg("timeout"))
                .def("vacuum_gripper_stop", &franka_proxy::franka_remote_interface::vacuum_gripper_stop)
                .def("start_recording", &franka_proxy::franka_remote_interface::start_recording,
                        py::arg("log_file_path"))
                .def("stop_recording", &franka_proxy::franka_remote_interface::stop_recording)
                .def("set_speed_factor", &franka_proxy::franka_remote_interface::set_speed_factor,
                        py::arg("speed_factor"))
                .def("set_fts_bias", &franka_proxy::franka_remote_interface::set_fts_bias,
                        py::arg("bias"))
                .def("set_fts_load_mass", &franka_proxy::franka_remote_interface::set_fts_load_mass,
                        py::arg("load_mass"))
                .def("automatic_error_recovery", &franka_proxy::franka_remote_interface::automatic_error_recovery)
                .def("set_guiding_params", &franka_proxy::franka_remote_interface::set_guiding_params,
                        py::arg("x"), py::arg("y"), py::arg("z"), py::arg("rx"), py::arg("ry"), py::arg("rz"),
                        py::arg("elbow"))
                .def("current_config", &franka_proxy::franka_remote_interface::current_config)
                .def("current_gripper_pos", &franka_proxy::franka_remote_interface::current_gripper_pos)
                .def("max_gripper_pos", &franka_proxy::franka_remote_interface::max_gripper_pos)
                .def("gripper_grasped", &franka_proxy::franka_remote_interface::gripper_grasped)
                .def("update", &franka_proxy::franka_remote_interface::update)
                .def("get_vacuum_gripper_state", &franka_proxy::franka_remote_interface::get_vacuum_gripper_state);

        // // py::register_exception<franka_proxy::ik_failed>(m, "ik_failed", m.attr("exception").ptr());
        // // py::register_exception<franka_proxy::not_implemented>(m, "not_implemented", m.attr("exception").ptr());
        // // py::register_exception<franka_proxy::interpolation_error>(m, "interpolation_error", m.attr("exception").ptr());
        // // py::register_exception<franka_proxy::api_call_failed>(m, "api_call_failed", m.attr("exception").ptr());
        
        py::class_<franka_proxy::joint_limit>(m, "joint_limit")
                .def(py::init<double, double>(), py::arg("min"), py::arg("max"))
                .def_readwrite("min", &franka_proxy::joint_limit::min)
                .def_readwrite("max", &franka_proxy::joint_limit::max);

        py::class_<franka_proxy::franka_proxy_util>(m, "franka_proxy_util_old");
        
        py::class_<py_franka_proxy_util, franka_proxy::franka_proxy_util>(m, "franka_proxy_util")
                .def(py::init<>())
                .def_static("ik_fast_closest", &py_franka_proxy_util::ik_fast_closest_wrapper,
                        py::arg("target_world_T_j7"), py::arg("current_configuration"), py::arg("step_size") = 0.174533)
                .def_static("joint_limits", &py_franka_proxy_util::joint_limits)
                .def_static("max_speed_per_joint", &py_franka_proxy_util::max_speed_per_joint)
                .def_static("max_acc_per_joint", &py_franka_proxy_util::max_acc_per_joint)
                .def_static("max_jerk_per_joint", &py_franka_proxy_util::max_jerk_per_joint)
                .def_static("is_reachable", &py_franka_proxy_util::is_reachable, py::arg("target"))
                //.def_static("fk", py::overload_cast<const robot_config_7dof&>(&franka_proxy::franka_proxy_util::fk),
                //            py::arg("configuration"))
                .def_static("fk", [](const robot_config_7dof& config){
                        std::vector<Eigen::Affine3d> result = py_franka_proxy_util::fk(config);

                        std::vector<Eigen::Matrix4d> matrices;
                        matrices.reserve(result.size());
                        for(const auto& trafo : result){
                                matrices.push_back(trafo.matrix());
                        }

                        return matrices;
                }, py::arg("configuration"))
                //.def_static("fk", py::overload_cast<const std::array<double, 7>&>(&py_franka_proxy_util::fk),
                //            py::arg("configuration"))
                .def_static("ik_fast", &py_franka_proxy_util::ik_fast,
                            py::arg("target_world_T_j7"), py::arg("joint_4_value") = 0.)
                .def_static("ik_fast_robust", &py_franka_proxy_util::ik_fast_robust,
                             py::arg("target_world_T_j7"), py::arg("step_size") = 0.174533)
                .def_static("ik_fast_closest", &py_franka_proxy_util::ik_fast_closest,
                             py::arg("target_world_T_j7"), py::arg("current_configuration"), py::arg("step_size") = 0.174533)
                .def_static("tool_mass", &py_franka_proxy_util::tool_mass)
                .def_static("tool_center_of_mass", &py_franka_proxy_util::tool_center_of_mass)
                .def_static("tool_inertia", &py_franka_proxy_util::tool_inertia)
                .def_readonly_static("deg_to_rad", &py_franka_proxy_util::deg_to_rad)
                .def_readonly_static("rad_to_deg", &py_franka_proxy_util::rad_to_deg);
        
}
// .def("start", py::overload_cast<std::optional<std::string>>(&franka_proxy::detail::motion_recorder::start),
//                      py::arg("log_file_path"))
// py::register_exception<franka_proxy::bad_response_exception>(m, "bad_response_exception", m.attr("exception").ptr());