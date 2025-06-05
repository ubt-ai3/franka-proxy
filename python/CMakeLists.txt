set(MODULE_NAME py_franka_control)

pybind11_add_module(${MODULE_NAME} SHARED
    bindings/py_franka_control.cpp
)

target_compile_features(${MODULE_NAME} PUBLIC cxx_std_20)

target_include_directories(${MODULE_NAME} PRIVATE
    ${ALL_INCLUDE_DIRECTORIES}
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${MODULE_NAME} PRIVATE
    ${Python3_LIBRARIES}
    franka_proxy_share
    franka_proxy_client
    franka_control
)

# Copy module to binary dir
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

# Copy python example to binary dir
add_custom_command(
    TARGET ${MODULE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/controller_test.py
            ${CMAKE_CURRENT_BINARY_DIR}/controller_test.py
)