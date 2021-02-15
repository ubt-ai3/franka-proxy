include(CMakeFindDependencyMacro)

find_dependency(asio CONFIG)
find_dependency(Eigen3 CONFIG)

include("${CMAKE_CURRENT_LIST_DIR}/franka-proxyTargets.cmake")