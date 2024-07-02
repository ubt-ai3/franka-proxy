include(CMakeFindDependencyMacro)

find_dependency(asio CONFIG)
find_dependency(nlohmann_json CONFIG)
find_dependency(Eigen3 CONFIG)
find_dependency(Threads REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/franka-proxyTargets.cmake")