include(CMakeFindDependencyMacro)

find_package(argparse CONFIG REQUIRED)
find_package(asio CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/franka-proxyTargets.cmake")