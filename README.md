# Setup
## Externals via vcpkg

./bootstrap-vcpkg.bat

vcpkg install asio:x64-windows franka:x64-windows

mkdir build
cd build
cmake .. "-DCMAKE_TOOLCHAIN_FILE=C:\Users\hartwig\Desktop\franka_proxy_without_viral\tools\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build .


# Project structure
```mermaid
classDiagram
    class franka_state_server{
        -franka_hardware_controller& controller_
    }
    class franka_control_server{
        -franka_hardware_controller& controller_
    }

    class Robot
    <<franka>> Robot
    class Gripper
    <<franka>> Gripper

    franka_proxy *-- franka_control_server
    franka_proxy *-- franka_state_server
    franka_proxy *-- franka_hardware_controller
    franka_control_server .. franka_control_client
    franka_state_server .. franka_state_client
    franka_control_client --* franka_remote_controller
    franka_state_client --* franka_remote_controller
    franka_hardware_controller *-- Robot
    franka_hardware_controller *-- Gripper

    class franka_controller_emulated{
        -franka_proxy::franka_remote_controller& controller
    }

    class franka_controller
    <<abstract>> franka_controller
    
    franka_controller_emulated --|> franka_controller
    franka_controller_remote --|> franka_controller
    franka_remote_controller -- franka_controller_remote
```