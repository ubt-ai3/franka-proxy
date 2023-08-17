#if !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPP)
#define INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HP

#include <array>
#include <Eigen/Geometry>
#include <franka_proxy/franka_hardware_controller.hpp>
#include <ft_sensor/schunk_ft.hpp>

class schunk_ft_sensor_to_franka_calibration
{

public:
    //todo: implement these
    static franka_proxy::robot_force_config calibrate_bias(
        franka_proxy::franka_hardware_controller& franka,
        double record_time_per_pose_seconds = 2.0,
        double wait_time_seconds = 0.5);
    static Eigen::Affine3f calibrate_load(franka_control::franka_controller& franka);

private:
    static std::array<Eigen::Affine3d, 24> calibration_poses(
        const franka_control::robot_config_7dof& x_up_position = {1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602 },
        const franka_control::robot_config_7dof& y_up_position = { 1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602 },
        const franka_control::robot_config_7dof& z_up_position = { 1.88336, 0.0335908, -1.86277, -1.26855, 0.0206543, 1.34875, 0.706602 });

    static Eigen::Matrix3d get_axis_aligned_orientation(const Eigen::Vector3f& up, const Eigen::Vector3f& front);
};

#endif /* !defined(INCLUDED__CALIBRATION__SHUNK_FT_TO_FRANKA_CALIBRATION_HPPP) */