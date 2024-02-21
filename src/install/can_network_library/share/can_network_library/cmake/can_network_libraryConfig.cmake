# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_can_network_library_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED can_network_library_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(can_network_library_FOUND FALSE)
  elseif(NOT can_network_library_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(can_network_library_FOUND FALSE)
  endif()
  return()
endif()
set(_can_network_library_CONFIG_INCLUDED TRUE)

# output package information
if(NOT can_network_library_FIND_QUIETLY)
  message(STATUS "Found can_network_library: 0.0.0 (${can_network_library_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'can_network_library' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT can_network_library_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(can_network_library_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${can_network_library_DIR}/${_extra}")
endforeach()
