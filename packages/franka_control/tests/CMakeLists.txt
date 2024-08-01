set(EXE_NAME franka_control_test)

add_executable(${EXE_NAME})

target_sources(${EXE_NAME}
  PRIVATE    
    main.cpp
)

if (BUILD_FT_SENSOR)
  target_sources(${EXE_NAME}
    PRIVATE    
      schunk_ft_to_franka_calibration.cpp
      schunk_ft_to_franka_calibration.hpp
  )
  #linking here is just for the compile_definitions of ft_sensor
  target_link_libraries(${EXE_NAME} PRIVATE ft_sensor)
endif()


target_link_libraries(${EXE_NAME}
    PUBLIC
        argparse::argparse
        asio::asio
        Eigen3::Eigen
        Threads::Threads
        franka_proxy_share
        franka_proxy_client
        franka_control
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