# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_teleop_twist_listener_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED teleop_twist_listener_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(teleop_twist_listener_FOUND FALSE)
  elseif(NOT teleop_twist_listener_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(teleop_twist_listener_FOUND FALSE)
  endif()
  return()
endif()
set(_teleop_twist_listener_CONFIG_INCLUDED TRUE)

# output package information
if(NOT teleop_twist_listener_FIND_QUIETLY)
  message(STATUS "Found teleop_twist_listener: 0.0.1 (${teleop_twist_listener_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'teleop_twist_listener' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${teleop_twist_listener_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(teleop_twist_listener_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${teleop_twist_listener_DIR}/${_extra}")
endforeach()
