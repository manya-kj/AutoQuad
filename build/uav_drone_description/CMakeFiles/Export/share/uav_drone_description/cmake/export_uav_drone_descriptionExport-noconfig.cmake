#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "uav_drone_description::plugin_drone" for configuration ""
set_property(TARGET uav_drone_description::plugin_drone APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(uav_drone_description::plugin_drone PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libplugin_drone.so"
  IMPORTED_SONAME_NOCONFIG "libplugin_drone.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS uav_drone_description::plugin_drone )
list(APPEND _IMPORT_CHECK_FILES_FOR_uav_drone_description::plugin_drone "${_IMPORT_PREFIX}/lib/libplugin_drone.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
