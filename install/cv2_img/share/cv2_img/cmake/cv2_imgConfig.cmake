# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cv2_img_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cv2_img_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cv2_img_FOUND FALSE)
  elseif(NOT cv2_img_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cv2_img_FOUND FALSE)
  endif()
  return()
endif()
set(_cv2_img_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cv2_img_FIND_QUIETLY)
  message(STATUS "Found cv2_img: 0.0.0 (${cv2_img_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cv2_img' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cv2_img_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cv2_img_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cv2_img_DIR}/${_extra}")
endforeach()
