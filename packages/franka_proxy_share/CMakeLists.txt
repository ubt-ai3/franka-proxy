set(LIB_NAME franka_proxy_share)

add_library(${LIB_NAME} STATIC)

set(INCLUDE_DIR include/${LIB_NAME})

target_include_directories(${LIB_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)

target_sources(${LIB_NAME}
  PRIVATE    
    src/franka_proxy_commands.cpp
    src/franka_proxy_logger.cpp
    src/franka_proxy_util.cpp
    src/ikfast.cpp
  PUBLIC FILE_SET HEADERS 
  BASE_DIRS ${INCLUDE_DIR}
  FILES
    ${INCLUDE_DIR}/asio_forward.hpp
    ${INCLUDE_DIR}/exception.hpp
    ${INCLUDE_DIR}/franka_proxy_commands.hpp
    ${INCLUDE_DIR}/franka_proxy_logger.hpp
    ${INCLUDE_DIR}/franka_proxy_util.hpp
    ${INCLUDE_DIR}/ikfast.h
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
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