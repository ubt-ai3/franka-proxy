#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "franka_control" for configuration "Debug"
set_property(TARGET franka_control APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(franka_control PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/franka_control.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS franka_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_franka_control "${_IMPORT_PREFIX}/lib/franka_control.lib" )

# Import target "franka_proxy" for configuration "Debug"
set_property(TARGET franka_proxy APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(franka_proxy PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/franka_proxy.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS franka_proxy )
list(APPEND _IMPORT_CHECK_FILES_FOR_franka_proxy "${_IMPORT_PREFIX}/bin/franka_proxy.exe" )

# Import target "franka_proxy_client" for configuration "Debug"
set_property(TARGET franka_proxy_client APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(franka_proxy_client PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/franka_proxy_client.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS franka_proxy_client )
list(APPEND _IMPORT_CHECK_FILES_FOR_franka_proxy_client "${_IMPORT_PREFIX}/lib/franka_proxy_client.lib" )

# Import target "franka_proxy_share" for configuration "Debug"
set_property(TARGET franka_proxy_share APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(franka_proxy_share PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/franka_proxy_share.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS franka_proxy_share )
list(APPEND _IMPORT_CHECK_FILES_FOR_franka_proxy_share "${_IMPORT_PREFIX}/lib/franka_proxy_share.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
