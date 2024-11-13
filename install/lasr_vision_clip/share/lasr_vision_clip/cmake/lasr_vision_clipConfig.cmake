# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lasr_vision_clip_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lasr_vision_clip_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lasr_vision_clip_FOUND FALSE)
  elseif(NOT lasr_vision_clip_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lasr_vision_clip_FOUND FALSE)
  endif()
  return()
endif()
set(_lasr_vision_clip_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lasr_vision_clip_FIND_QUIETLY)
  message(STATUS "Found lasr_vision_clip: 0.0.0 (${lasr_vision_clip_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lasr_vision_clip' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lasr_vision_clip_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lasr_vision_clip_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lasr_vision_clip_DIR}/${_extra}")
endforeach()
