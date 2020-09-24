cmake_minimum_required(VERSION 3.11)
project(franka_proxy)


find_package(asio CONFIG REQUIRED)
if(MSVC)
	add_compile_definitions(_WIN32_WINNT=0x0501)
endif()
find_package(Eigen3 CONFIG REQUIRED)
find_package(Franka CONFIG REQUIRED)
find_package(Poco CONFIG REQUIRED COMPONENTS Net Foundation)
find_package(Threads REQUIRED)


## franka_share
add_library(
    franka_share
    INTERFACE
)

target_include_directories(
    franka_share
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/source
)

target_link_libraries(
    franka_share
    INTERFACE
)


## franka_proxy
add_executable(
    franka_proxy
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_hardware_controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_motion_generator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_motion_recorder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_network_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_proxy.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/jr3_ft_sensor/force_torque_sensor.cpp
)

target_include_directories(
    franka_proxy
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source
        ${franka_SOURCE_DIR}/include
        ${eigen3_SOURCE_DIR}
)

target_link_libraries(
    franka_proxy
    PUBLIC
        asio::asio
        Eigen3::Eigen
		Poco::Foundation
		Poco::Net
        Threads::Threads
		Franka::Franka
        franka_share
)

target_compile_definitions(
    franka_proxy
    PUBLIC
		std_cxx_17
)


## franka_proxy_client
add_library(
    franka_proxy_client STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client/franka_network_client.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client/franka_remote_controller.cpp
)

target_include_directories(
    franka_proxy_client
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source
)

target_link_libraries(
    franka_proxy_client
    PUBLIC
        asio::asio
        Eigen3::Eigen
		franka_share
)

target_compile_definitions(
    franka_proxy_client
    PUBLIC
        std_cxx_17
)


## franka_proxy_client_test
add_executable(
    franka_proxy_client_test
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client_test/main.cpp
)

target_include_directories(
    franka_proxy_client_test
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source
        ${eigen3_SOURCE_DIR}
)

target_link_libraries(
    franka_proxy_client_test
    PUBLIC
        asio::asio
        Eigen3::Eigen
		franka_proxy_client
)

target_compile_definitions(
    franka_proxy_client_test
    PUBLIC
		std_cxx_17
)


## franka_control
add_library(
    franka_control
    STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/franka_controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/franka_controller_emulated.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/franka_controller_remote.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/franka_util.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/ikfast.cpp
)

target_link_libraries(
    franka_control
    PUBLIC
        asio::asio
        Eigen3::Eigen
        franka_proxy_client
)

target_compile_definitions(
    franka_control
    PUBLIC
		std_cxx_17
)
