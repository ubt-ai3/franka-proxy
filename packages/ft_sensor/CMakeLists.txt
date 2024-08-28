set(LIB_NAME ft_sensor)

add_library(${LIB_NAME} STATIC)

set(INCLUDE_DIR include/${LIB_NAME})

target_include_directories(${LIB_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)

target_sources(${LIB_NAME}
  PRIVATE    
    src/schunk_ft.cpp
  PUBLIC FILE_SET HEADERS 
  BASE_DIRS ${INCLUDE_DIR}
  FILES
    ${INCLUDE_DIR}/ft_sensor.hpp
    ${INCLUDE_DIR}/schunk_ft.hpp
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        asio::asio
        Eigen3::Eigen
        franka_proxy_share
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