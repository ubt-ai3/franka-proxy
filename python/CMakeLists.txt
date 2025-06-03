set(MODULE_NAME franka_control_python_bindings)

# Development component is required (see https://github.com/pybind/pybind11/issues/3996)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Define the module
pybind11_add_module(${MODULE_NAME} SHARED
    franka_control_python_bindings.cpp
)
target_compile_features(${MODULE_NAME} PUBLIC cxx_std_20)

# Include directories
target_include_directories(${MODULE_NAME} PRIVATE
    ${ALL_INCLUDE_DIRECTORIES}
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${MODULE_NAME} PRIVATE
    ${Python3_LIBRARIES}
    franka_proxy_share
    franka_proxy_client
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

add_custom_command(
    TARGET franka_control_python_bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/controller_test.py
            ${CMAKE_CURRENT_BINARY_DIR}/controller_test.py
)