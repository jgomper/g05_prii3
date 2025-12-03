# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gXX_prii3_WS_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gXX_prii3_WS_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gXX_prii3_WS_FOUND FALSE)
  elseif(NOT gXX_prii3_WS_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gXX_prii3_WS_FOUND FALSE)
  endif()
  return()
endif()
set(_gXX_prii3_WS_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gXX_prii3_WS_FIND_QUIETLY)
  message(STATUS "Found gXX_prii3_WS: 0.0.0 (${gXX_prii3_WS_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gXX_prii3_WS' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gXX_prii3_WS_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gXX_prii3_WS_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gXX_prii3_WS_DIR}/${_extra}")
endforeach()
