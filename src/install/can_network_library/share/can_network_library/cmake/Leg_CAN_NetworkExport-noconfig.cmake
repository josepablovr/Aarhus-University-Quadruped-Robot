#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "can_network_library::Leg_CAN_Network" for configuration ""
set_property(TARGET can_network_library::Leg_CAN_Network APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(can_network_library::Leg_CAN_Network PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C;CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libLeg_CAN_Network.a"
  )

list(APPEND _cmake_import_check_targets can_network_library::Leg_CAN_Network )
list(APPEND _cmake_import_check_files_for_can_network_library::Leg_CAN_Network "${_IMPORT_PREFIX}/lib/libLeg_CAN_Network.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
