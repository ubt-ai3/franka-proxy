# Common Ambient Variables:
#   CURRENT_BUILDTREES_DIR    = ${VCPKG_ROOT_DIR}\buildtrees\${PORT}
#   CURRENT_PACKAGES_DIR      = ${VCPKG_ROOT_DIR}\packages\${PORT}_${TARGET_TRIPLET}
#   CURRENT_PORT_DIR          = ${VCPKG_ROOT_DIR}\ports\${PORT}
#   CURRENT_INSTALLED_DIR     = ${VCPKG_ROOT_DIR}\installed\${TRIPLET}
#   DOWNLOADS                 = ${VCPKG_ROOT_DIR}\downloads
#   PORT                      = current port name (zlib, etc)
#   TARGET_TRIPLET            = current triplet (x86-windows, x64-windows-static, etc)
#   VCPKG_CRT_LINKAGE         = C runtime linkage type (static, dynamic)
#   VCPKG_LIBRARY_LINKAGE     = target library linkage type (static, dynamic)
#   VCPKG_ROOT_DIR            = <C:\path\to\current\vcpkg>
#   VCPKG_TARGET_ARCHITECTURE = target architecture (x64, x86, arm)
#   VCPKG_TOOLCHAIN           = ON OFF
#   TRIPLET_SYSTEM_ARCH       = arm x86 x64
#   BUILD_ARCH                = "Win32" "x64" "ARM"
#   MSBUILD_PLATFORM          = "Win32"/"x64"/${TRIPLET_SYSTEM_ARCH}
#   DEBUG_CONFIG              = "Debug Static" "Debug Dll"
#   RELEASE_CONFIG            = "Release Static"" "Release DLL"
#   VCPKG_TARGET_IS_WINDOWS
#   VCPKG_TARGET_IS_UWP
#   VCPKG_TARGET_IS_LINUX
#   VCPKG_TARGET_IS_OSX
#   VCPKG_TARGET_IS_FREEBSD
#   VCPKG_TARGET_IS_ANDROID
#   VCPKG_TARGET_IS_MINGW
#   VCPKG_TARGET_EXECUTABLE_SUFFIX
#   VCPKG_TARGET_STATIC_LIBRARY_SUFFIX
#   VCPKG_TARGET_SHARED_LIBRARY_SUFFIX
#
# 	See additional helpful variables in /docs/maintainers/vcpkg_common_definitions.md

# # Specifies if the port install should fail immediately given a condition
# vcpkg_fail_port_install(MESSAGE "libfranka currently only supports Linux and Mac platforms" ON_TARGET "Windows")

#vcpkg_from_gitlab(
#    GITLAB_URL https://resy-gitlab.inf.uni-bayreuth.de
#    OUT_SOURCE_PATH SOURCE_PATH
#    REPO libfranka/franka_proxy
#    REF 8f563f9e7b06ddd8ee71837c204c07927f182b1d
#    SHA512 59eea1acbbe9ddfa8b5f611410fb8018d39fcb15959554e7e7985fb0a673f54d2dade7e054320bfe02e167cb11d0db9252e2e94304830273db13c6925cd41240
#)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://resy-gitlab.inf.uni-bayreuth.de/libfranka/franka_proxy
    REF 2d27d25ff1110c969fba797a621cc4479d68564d
    PATCHES
        sth.patch
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    #PREFER_NINJA # Disable this option if project cannot be built with Ninja
    # OPTIONS -DUSE_THIS_IN_ALL_BUILDS=1 -DUSE_THIS_TOO=2
    # OPTIONS_RELEASE -DOPTIMIZE=1
    # OPTIONS_DEBUG -DDEBUGGABLE=1
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(
    CONFIG_PATH share 
    #TARGET_PATH share/${PORT}
)
vcpkg_copy_pdbs()

#file(COPY ${SOURCE_PATH}/build/x64_Release/libfranka.lib DESTINATION ${CURRENT_PACKAGES_DIR}/lib)
#file(COPY ${SOURCE_PATH}/build/x64_Debug/libfranka.lib DESTINATION ${CURRENT_PACKAGES_DIR}/debug/lib)
#file(COPY ${SOURCE_PATH}/build/x64_Debug/libfranka.pdb DESTINATION ${CURRENT_PACKAGES_DIR}/debug/lib)
#file(COPY ${SOURCE_PATH}/include/ DESTINATION ${CURRENT_PACKAGES_DIR}/include)

file(COPY ${CURRENT_PACKAGES_DIR}/bin/franka_proxy.exe DESTINATION ${CURRENT_PACKAGES_DIR}/tools)

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/bin")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/bin")

file(INSTALL ${SOURCE_PATH}/LICENSE DESTINATION ${CURRENT_PACKAGES_DIR}/share/${PORT} RENAME copyright)