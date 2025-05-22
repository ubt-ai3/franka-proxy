set(MODULE_NAME franka_control_module)

# Development component is required, else we get an error: https://github.com/pybind/pybind11/issues/3996
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Find pybind11 via Python module
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
find_package(pybind11 REQUIRED)

message(STATUS "Using pybind11 found at: ${pybind11_DIR}")

# Define the module
pybind11_add_module(${MODULE_NAME} SHARED
    test.cpp
    ../packages/franka_control/src/franka_controller.cpp
)
target_compile_features(${MODULE_NAME} PUBLIC cxx_std_20)

# Include directories
target_include_directories(${MODULE_NAME} PRIVATE
    ${ALL_INCLUDE_DIRECTORIES}
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/packages/franka_proxy/include
    ${CMAKE_SOURCE_DIR}/packages/franka_proxy/include/franka_proxy
)

# Link libraries
target_link_libraries(${MODULE_NAME} PRIVATE
    ${Python3_LIBRARIES}
    franka_control
)

# Post-build: copy module to binary directory
add_custom_command(TARGET ${MODULE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:${MODULE_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:${MODULE_NAME}>
)

# Ensure __init__.py exists
set(INIT_PY "${CMAKE_CURRENT_BINARY_DIR}/__init__.py")
if(NOT EXISTS ${INIT_PY})
    file(WRITE ${INIT_PY} "")
endif()

# Debug info (optional, remove if not needed)
foreach(config IN ITEMS "" "_DEBUG" "_RELEASE")
    get_target_property(_lib Python3::Python IMPORTED_IMPLIB${config})
    message(STATUS "Python3::Python${config} = ${_lib}")
endforeach()