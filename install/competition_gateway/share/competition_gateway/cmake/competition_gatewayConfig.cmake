# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_competition_gateway_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED competition_gateway_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(competition_gateway_FOUND FALSE)
  elseif(NOT competition_gateway_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(competition_gateway_FOUND FALSE)
  endif()
  return()
endif()
set(_competition_gateway_CONFIG_INCLUDED TRUE)

# output package information
if(NOT competition_gateway_FIND_QUIETLY)
  message(STATUS "Found competition_gateway: 0.1.0 (${competition_gateway_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'competition_gateway' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT competition_gateway_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(competition_gateway_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${competition_gateway_DIR}/${_extra}")
endforeach()
