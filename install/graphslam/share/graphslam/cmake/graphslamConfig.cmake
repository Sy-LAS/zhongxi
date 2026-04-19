# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_graphslam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED graphslam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(graphslam_FOUND FALSE)
  elseif(NOT graphslam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(graphslam_FOUND FALSE)
  endif()
  return()
endif()
set(_graphslam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT graphslam_FIND_QUIETLY)
  message(STATUS "Found graphslam: 0.0.0 (${graphslam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'graphslam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${graphslam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(graphslam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${graphslam_DIR}/${_extra}")
endforeach()
