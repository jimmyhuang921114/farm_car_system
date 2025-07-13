# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_obs_filiter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED obs_filiter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(obs_filiter_FOUND FALSE)
  elseif(NOT obs_filiter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(obs_filiter_FOUND FALSE)
  endif()
  return()
endif()
set(_obs_filiter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT obs_filiter_FIND_QUIETLY)
  message(STATUS "Found obs_filiter: 0.1.0 (${obs_filiter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'obs_filiter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${obs_filiter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(obs_filiter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${obs_filiter_DIR}/${_extra}")
endforeach()
