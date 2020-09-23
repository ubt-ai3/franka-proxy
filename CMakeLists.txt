cmake_minimum_required(VERSION 3.11)
project(franka_proxy)

include(cmake/CPM.cmake)

CPMAddPackage(
    NAME viral
    GIT_REPOSITORY https://resy-gitlab.inf.uni-bayreuth.de/verbbot/viral.git
    GIT_TAG 609393cc8a64f53608460aedf2108c13bfaa5bef
)

CPMAddPackage(
    NAME franka
    GIT_REPOSITORY https://resy-gitlab.inf.uni-bayreuth.de/libfranka/libfranka.git
    GIT_TAG cf58da4152f07a50ff859012f3d94629783c633b
)

CPMAddPackage(
    NAME eigen3_wrapper
    GIT_REPOSITORY https://resy-gitlab.inf.uni-bayreuth.de/verbbot/eigen3_wrapper.git
    GIT_TAG ed9aa62a6cabfb8a7f1137a7b213eb0f3c538ee2
)

find_package(Threads)


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
        viral::core
        eigen3
        franka_proxy_client
)

target_compile_definitions(
    franka_control
    PUBLIC
    std_cxx_17
)

add_library(
    franka_proxy
    STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_hardware_controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_motion_generator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_motion_recorder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_network_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_proxy.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/jr3_fts/force_torque_sensor.cpp
)

target_include_directories(
    franka_proxy
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source
        ${franka_SOURCE_DIR}/include
        ${eigen3_SOURCE_DIR}
)


add_library(
    franka_proxy_client
    STATIC
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
        viral::core
        franka_proxy
        eigen3
)

target_compile_definitions(
    franka_proxy_client
    PUBLIC
        std_cxx_17
)


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
        viral::core
        franka
        Threads::Threads
)

target_link_libraries(
    franka_proxy
    PUBLIC
        viral::core
        eigen3
        franka
        Threads::Threads
        franka_share
)

target_compile_definitions(
    franka_proxy
    PUBLIC
    std_cxx_17
)
