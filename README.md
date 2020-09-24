## Externals via vcpkg

run bootstrap-vcpkg.bat

vcpkg install asio:x64-windows libfranka:x64-windows


mkdir build
cd build
cmake .. "-DCMAKE_TOOLCHAIN_FILE=C:\Users\hartwig\Desktop\franka_proxy_without_viral\tools\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build .