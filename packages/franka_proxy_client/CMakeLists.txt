set(LIB_NAME franka_proxy_client)

add_library(${LIB_NAME} STATIC)

set(INCLUDE_DIR include/${LIB_NAME})

target_include_directories(${LIB_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)

target_sources(${LIB_NAME}
  PRIVATE    
    src/franka_network_client.cpp
    src/franka_remote_interface.cpp
  PUBLIC FILE_SET HEADERS 
  BASE_DIRS ${INCLUDE_DIR}
  FILES
    ${INCLUDE_DIR}/exception.hpp
    ${INCLUDE_DIR}/forward.hpp
    ${INCLUDE_DIR}/franka_network_client.hpp
    ${INCLUDE_DIR}/franka_remote_interface.hpp
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        asio::asio
        Eigen3::Eigen
        franka_proxy_share
)

target_compile_features(${LIB_NAME} PUBLIC cxx_std_20)

install(
  TARGETS 
    ${LIB_NAME}
  EXPORT franka-proxyConfig
  LIBRARY
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME 
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  FILE_SET HEADERS
    DESTINATION ${INCLUDE_DIR}
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()