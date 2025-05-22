set(MODULE_NAME franka_control_module)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Development component is required, else we get an error: https://github.com/pybind/pybind11/issues/3996
find_package (Python3 COMPONENTS Interpreter Development REQUIRED)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(pybind11_DIR ${PYBIND11_CMAKE_DIR})

message(STATUS "Using pybind11 found at: ${pybind11_DIR}")
find_package(pybind11 REQUIRED)
set_target_properties(Python3::Module PROPERTIES  MAP_IMPORTED_CONFIG_DEBUG ";RELEASE")


# Create Python module
pybind11_add_module(${MODULE_NAME} SHARED 
    test.cpp
    ../packages/franka_control/src/franka_controller.cpp
)

target_include_directories(${MODULE_NAME} PRIVATE ${ALL_INCLUDE_DIRECTORIES})
target_include_directories(${MODULE_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(${MODULE_NAME} PRIVATE ${Python3_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/packages/franka_proxy/include
)
include_directories(${CMAKE_SOURCE_DIR}/packages/franka_proxy/include/franka_proxy)
target_link_libraries(${MODULE_NAME} PRIVATE ${Python3_LIBRARIES})
target_link_libraries(${MODULE_NAME} PRIVATE franka_control)


set_target_properties(${MODULE_NAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

# Needed for importing the "simple_module" module from the build directory
set(INIT_PY "${CMAKE_CURRENT_BINARY_DIR}/__init__.py")
if(NOT EXISTS ${INIT_PY})
    file(WRITE ${INIT_PY} "")
endif()

add_custom_command(TARGET ${MODULE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE_DIR:${MODULE_NAME}>/$<TARGET_FILE_NAME:${MODULE_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:${MODULE_NAME}>
)

GET_TARGET_PROPERTY(_lib Python3::Python IMPORTED_IMPLIB)
MESSAGE(STATUS "Python3::Python ${_lib}")

GET_TARGET_PROPERTY(_lib Python3::Python IMPORTED_IMPLIB_DEBUG)
MESSAGE(STATUS "Python3::Python (Debug) ${_lib}")

GET_TARGET_PROPERTY(_lib Python3::Python IMPORTED_IMPLIB_RELEASE)
MESSAGE(STATUS "Python3::Python (Release) ${_lib}")
