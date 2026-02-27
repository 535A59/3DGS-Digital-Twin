# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mtc_dynamic_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mtc_dynamic_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mtc_dynamic_FOUND FALSE)
  elseif(NOT mtc_dynamic_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mtc_dynamic_FOUND FALSE)
  endif()
  return()
endif()
set(_mtc_dynamic_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mtc_dynamic_FIND_QUIETLY)
  message(STATUS "Found mtc_dynamic: 0.1.3 (${mtc_dynamic_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mtc_dynamic' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mtc_dynamic_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mtc_dynamic_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mtc_dynamic_DIR}/${_extra}")
endforeach()
