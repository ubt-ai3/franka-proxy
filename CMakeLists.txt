cmake_minimum_required(VERSION 3.11)
project(franka-proxy)


option(BUILD_SERVER "Build franka_proxy executable" ON)


if(BUILD_SERVER)
	find_package(Franka CONFIG REQUIRED)
    find_package(Poco COMPONENTS Net Foundation REQUIRED)
endif()

find_package(asio CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)


## TODO convert to modern cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	add_compile_definitions(_WIN32_WINNT=0x0601)
	add_compile_definitions(NOMINMAX)
endif()


add_library(
	franka_proxy_share
	STATIC
	""
)

target_sources(
	franka_proxy_share
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_share/franka_proxy_commands.cpp
)

target_include_directories(
	franka_proxy_share
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
)

target_link_libraries(
	franka_proxy_share
	PUBLIC
		nlohmann_json::nlohmann_json
)


if(BUILD_SERVER)
## franka_proxy
add_executable(
    franka_proxy
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_hardware_controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_network_control_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_network_state_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/franka_proxy.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_admittance.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_cartesian_impedance.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_force.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_joint_impedance.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_joint_max_accel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_generator_seq_cart_vel_tau.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy/motion_recorder.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/source/ft_sensor/schunk_ft.cpp

)

target_include_directories(
    franka_proxy
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
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
		franka_proxy_share
)

target_compile_definitions(
    franka_proxy
    PUBLIC
		std_cxx_17
)
endif()


## franka_proxy_client
add_library(
    franka_proxy_client STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client/franka_network_client.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client/franka_remote_interface.cpp
)

target_include_directories(
    franka_proxy_client
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
)

target_link_libraries(
    franka_proxy_client
    PUBLIC
        asio::asio
        Eigen3::Eigen
        franka_proxy_share
)

target_compile_definitions(
    franka_proxy_client
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

target_include_directories(
    franka_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
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



## franka_proxy_client_test
add_executable(
    franka_proxy_client_test
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client_test/main.cpp
)

target_include_directories(
    franka_proxy_client_test
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
)

target_link_libraries(
    franka_proxy_client_test
    PUBLIC
        asio::asio
        Eigen3::Eigen
        Threads::Threads
		franka_proxy_share
		franka_proxy_client
)

target_compile_definitions(
    franka_proxy_client_test
    PUBLIC
		std_cxx_17
)

## franka_control_test
add_executable(
    franka_control_test
        ${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/franka_control_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/sensor_calibration/schunk_ft_to_franka_calibration.cpp
)

target_include_directories(
    franka_control_test
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
)

target_link_libraries(
    franka_control_test
    PUBLIC
        asio::asio
        Eigen3::Eigen
        Threads::Threads
		franka_proxy_share
		franka_proxy_client
		franka_control
)

target_compile_definitions(
    franka_control_test
    PUBLIC
		std_cxx_17
)




## install
file(GLOB franka_proxy_share_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_share/*.hpp"
)
file(GLOB franka_proxy_client_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/source/franka_proxy_client/*.hpp"
)
file(GLOB franka_control_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/source/franka_control/*.hpp"
)

if(BUILD_SERVER)
	install(
		TARGETS franka_control franka_proxy franka_proxy_client franka_proxy_share
		EXPORT franka-proxyConfig
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
else()
	install(
		TARGETS franka_control franka_proxy_client franka_proxy_share
		EXPORT franka-proxyConfig
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()
	
install(
	FILES ${franka_proxy_share_headers}
	DESTINATION include/franka_proxy_share
)
install(
	FILES ${franka_proxy_client_headers}
	DESTINATION include/franka_proxy_client
)
install(
	FILES ${franka_control_headers}
	DESTINATION include/franka_control
)

install(EXPORT franka-proxyConfig DESTINATION share FILE franka-proxyTargets.cmake)


# assets
if(BUILD_SERVER)
    file(GLOB franka_assets
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/*"
    )

    file(
        GENERATE 
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/CopyConfigurationData$<CONFIG>.cmake"
            CONTENT 
                "if (NOT EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/assets\")
                    execute_process(
                         COMMAND \"${CMAKE_COMMAND}\" -E copy_directory
                             \"${PROJECT_SOURCE_DIR}/assets\"
                             \"${CMAKE_CURRENT_BINARY_DIR}/assets\"
                    )
                endif()"
    )

    add_custom_command(
        TARGET franka_proxy
        POST_BUILD 
        COMMAND "${CMAKE_COMMAND}" -P "CopyConfigurationData$<CONFIG>.cmake"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )

    install(
	FILES ${franka_assets}
	DESTINATION bin/assets
)
endif()