set(EXE_NAME franka_proxy)

add_executable(${EXE_NAME})

set(INCLUDE_DIR include/${EXE_NAME})

target_sources(${EXE_NAME}
  PRIVATE    
    src/franka_hardware_controller.cpp
    src/franka_network_control_server.cpp
    src/franka_network_state_server.cpp
    src/franka_proxy.cpp
    src/motion_generator_admittance.cpp
    src/motion_generator_cartesian_impedance.cpp
    src/motion_generator_force.cpp
    src/motion_generator_joint_impedance.cpp
    src/motion_generator_joint_imp_ple.cpp
    src/motion_generator_joint_max_accel.cpp
    src/motion_generator_seq_cart_vel_tau.cpp
    src/motion_recorder.cpp
  PRIVATE FILE_SET HEADERS 
  BASE_DIRS ${INCLUDE_DIR}
  FILES
    ${INCLUDE_DIR}/franka_hardware_controller.hpp
    ${INCLUDE_DIR}/franka_network_control_server.hpp
    ${INCLUDE_DIR}/franka_network_state_server.hpp
    ${INCLUDE_DIR}/franka_proxy.hpp
    ${INCLUDE_DIR}/motion_generator_admittance.hpp
    ${INCLUDE_DIR}/motion_generator_cartesian_impedance.hpp
    ${INCLUDE_DIR}/motion_generator_force.hpp
    ${INCLUDE_DIR}/motion_generator_joint_imp_ple.hpp
    ${INCLUDE_DIR}/motion_generator_joint_impedance.hpp
    ${INCLUDE_DIR}/motion_generator_joint_max_accel.hpp
    ${INCLUDE_DIR}/motion_generator_seq_cart_vel_tau.hpp
    ${INCLUDE_DIR}/motion_recorder.hpp
)

target_link_libraries(${EXE_NAME}
    PUBLIC
        argparse::argparse
        asio::asio
        Eigen3::Eigen
        Threads::Threads
        Franka::Franka
        franka_proxy_share
		ft_sensor
)

target_compile_features(${EXE_NAME} PUBLIC cxx_std_20)

install(
  TARGETS 
    ${EXE_NAME}
  EXPORT franka-proxyConfig
  LIBRARY
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME 
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  FILE_SET HEADERS
    DESTINATION ${INCLUDE_DIR}
)