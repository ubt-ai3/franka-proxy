cmake_minimum_required(VERSION 3.20)
project(franka-proxy
    LANGUAGES CXX)

include(GNUInstallDirs)


option(BUILD_SERVER "Build franka_proxy executable" ON)
option(BUILD_TESTS "Build franka_proxy tests" ON)


if(BUILD_SERVER)
	find_package(Franka CONFIG REQUIRED)
    find_package(Poco COMPONENTS Net Foundation REQUIRED)
endif()

if(BUILD_SERVER OR BUILD_TESTS)
    find_package(argparse CONFIG REQUIRED)
endif()

find_package(asio CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)


if (WIN32)
	add_compile_definitions(_WIN32_WINNT=0x0A00)
	add_compile_definitions(NOMINMAX)
endif()


set(PROXY_TARGETS
    franka_proxy_share
    franka_proxy_client
    franka_control
	ft_sensor
)

add_subdirectory(packages/franka_proxy_share)
add_subdirectory(packages/franka_proxy_client)
add_subdirectory(packages/franka_control)
add_subdirectory(packages/ft_sensor)

if(BUILD_SERVER)
    add_subdirectory(packages/franka_proxy)
    list(APPEND PROXY_TARGETS franka_proxy)
endif()

if(BUILD_TESTS)
    list(APPEND PROXY_TARGETS 
		franka_proxy_client_test 
		franka_control_test)
endif()


configure_file(${CMAKE_SOURCE_DIR}/cmake/franka-proxyConfig.cmake.in
               ${CMAKE_BINARY_DIR}/cmake/franka-proxyConfig.cmake @ONLY)

install(FILES "${CMAKE_BINARY_DIR}/cmake/franka-proxyConfig.cmake"
        DESTINATION share)

install(EXPORT franka-proxyConfig
        DESTINATION share
        FILE franka-proxyTargets.cmake)