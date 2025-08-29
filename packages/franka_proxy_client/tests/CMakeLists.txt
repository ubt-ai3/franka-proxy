set(EXE_NAME franka_proxy_client_test)

add_executable(${EXE_NAME})

target_sources(${EXE_NAME}
    PRIVATE    
        main.cpp
)

target_link_libraries(${EXE_NAME}
    PUBLIC
        argparse::argparse
        asio::asio
        Eigen3::Eigen
        Threads::Threads
        franka_proxy_share
        franka_proxy_client
)

target_compile_features(${EXE_NAME} PUBLIC cxx_std_20)

target_compile_definitions(${EXE_NAME} 
    PRIVATE
        FRANKA_PROXY_VERSION="${PROJECT_VERSION}")

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