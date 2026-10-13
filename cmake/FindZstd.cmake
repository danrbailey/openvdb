# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: Apache-2.0
#
#[=======================================================================[.rst:

FindZstd
--------

Find Zstd include dirs and libraries

This module is intended as a fallback for zstd installs that do not ship the
zstd CMake config files. Prefer find_package(zstd CONFIG) where available.

Use this module by invoking find_package with the form::

  find_package(Zstd
    [version] [EXACT]      # Minimum or EXACT version
    [REQUIRED]             # Fail with error if Zstd is not found
    )


IMPORTED Targets
^^^^^^^^^^^^^^^^

``Zstd::zstd``
  This module defines IMPORTED target Zstd::zstd, if Zstd has been found.

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Zstd_FOUND``
  True if the system has the Zstd library.
``Zstd_VERSION``
  The version of the Zstd library which was found.
``Zstd_INCLUDE_DIRS``
  Include directories needed to use Zstd.
``Zstd_RELEASE_LIBRARIES``
  Libraries needed to link to the release version of Zstd.
``Zstd_RELEASE_LIBRARY_DIRS``
  Zstd release library directories.
``Zstd_DEBUG_LIBRARIES``
  Libraries needed to link to the debug version of Zstd.
``Zstd_DEBUG_LIBRARY_DIRS``
  Zstd debug library directories.
``Zstd_LIBRARIES``
  Libraries needed to link to Zstd.
``Zstd_LIBRARY_DIRS``
  Zstd library directories.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``Zstd_INCLUDE_DIR``
  The directory containing ``zstd.h``.
``Zstd_LIBRARY``
  The path to the Zstd library. may include target_link_libraries() debug/optimized keywords.
``Zstd_LIBRARY_RELEASE``
  The path to the Zstd release library.
``Zstd_LIBRARY_DEBUG``
  The path to the Zstd debug library.

Hints
^^^^^

Instead of explicitly setting the cache variables, the following variables
may be provided to tell this module where to look.

``Zstd_ROOT``
  Preferred installation prefix.
``ZSTD_INCLUDEDIR``
  Preferred include directory e.g. <prefix>/include
``ZSTD_LIBRARYDIR``
  Preferred library directory e.g. <prefix>/lib
``ZSTD_DEBUG_SUFFIX``
  Suffix of the debug version of the zstd lib. Defaults to "d".
``SYSTEM_LIBRARY_PATHS``
  Global list of library paths intended to be searched by and find_xxx call
``ZSTD_USE_STATIC_LIBS``
  Only search for static zstd libraries
``DISABLE_CMAKE_SEARCH_PATHS``
  Disable CMakes default search paths for find_xxx calls in this module

#]=======================================================================]

cmake_minimum_required(VERSION 3.24)
include(GNUInstallDirs)

mark_as_advanced(
  Zstd_INCLUDE_DIR
  Zstd_LIBRARY
)

set(_FIND_ZSTD_ADDITIONAL_OPTIONS "")
if(DISABLE_CMAKE_SEARCH_PATHS)
  set(_FIND_ZSTD_ADDITIONAL_OPTIONS NO_DEFAULT_PATH)
endif()

# Set _ZSTD_ROOT based on a user provided root var. Xxx_ROOT and ENV{Xxx_ROOT}
# are prioritised over the capitalized XXX_ROOT variables
if(Zstd_ROOT)
  set(_ZSTD_ROOT ${Zstd_ROOT})
elseif(DEFINED ENV{Zstd_ROOT})
  set(_ZSTD_ROOT $ENV{Zstd_ROOT})
elseif(ZSTD_ROOT)
  set(_ZSTD_ROOT ${ZSTD_ROOT})
elseif(DEFINED ENV{ZSTD_ROOT})
  set(_ZSTD_ROOT $ENV{ZSTD_ROOT})
endif()

# Additionally try and use pkconfig to find zstd
if(USE_PKGCONFIG)
  if(NOT DEFINED PKG_CONFIG_FOUND)
    find_package(PkgConfig)
  endif()
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_Zstd QUIET libzstd)
  endif()
endif()

# ------------------------------------------------------------------------
#  Search for Zstd include DIR
# ------------------------------------------------------------------------

set(_ZSTD_INCLUDE_SEARCH_DIRS "")
list(APPEND _ZSTD_INCLUDE_SEARCH_DIRS
  ${ZSTD_INCLUDEDIR}
  ${_ZSTD_ROOT}
  ${PC_Zstd_INCLUDEDIR}
  ${SYSTEM_LIBRARY_PATHS}
)

# Look for a standard zstd header file.
find_path(Zstd_INCLUDE_DIR zstd.h
  ${_FIND_ZSTD_ADDITIONAL_OPTIONS}
  PATHS ${_ZSTD_INCLUDE_SEARCH_DIRS}
  PATH_SUFFIXES ${CMAKE_INSTALL_INCLUDEDIR} include
)

if(EXISTS "${Zstd_INCLUDE_DIR}/zstd.h")
  foreach(_ZSTD_VERSION_PART MAJOR MINOR RELEASE)
    file(STRINGS "${Zstd_INCLUDE_DIR}/zstd.h"
      _zstd_version_string REGEX "#define ZSTD_VERSION_${_ZSTD_VERSION_PART} +[0-9]+"
    )
    string(REGEX REPLACE "#define ZSTD_VERSION_${_ZSTD_VERSION_PART} +([0-9]+).*$" "\\1"
      Zstd_VERSION_${_ZSTD_VERSION_PART} "${_zstd_version_string}"
    )
  endforeach()
  unset(_zstd_version_string)
  unset(_ZSTD_VERSION_PART)

  set(Zstd_VERSION ${Zstd_VERSION_MAJOR}.${Zstd_VERSION_MINOR}.${Zstd_VERSION_RELEASE})
endif()

# ------------------------------------------------------------------------
#  Search for Zstd lib DIR
# ------------------------------------------------------------------------

if(NOT DEFINED ZSTD_DEBUG_SUFFIX)
  set(ZSTD_DEBUG_SUFFIX d)
endif()

set(_ZSTD_LIBRARYDIR_SEARCH_DIRS "")
list(APPEND _ZSTD_LIBRARYDIR_SEARCH_DIRS
  ${ZSTD_LIBRARYDIR}
  ${_ZSTD_ROOT}
  ${PC_Zstd_LIBDIR}
  ${SYSTEM_LIBRARY_PATHS}
)

# Library suffix handling

set(_ZSTD_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

if(MSVC)
  if(ZSTD_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  endif()
else()
  if(ZSTD_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  endif()
endif()

# On Windows the zstd CMake build names the static library zstd_static.lib
# and the DLL import library zstd.lib
set(_ZSTD_LIB_NAMES zstd)
if(MSVC AND ZSTD_USE_STATIC_LIBS)
  set(_ZSTD_LIB_NAMES zstd_static zstd)
endif()

set(Zstd_LIB_COMPONENTS "")
list(APPEND ZSTD_BUILD_TYPES RELEASE DEBUG)

foreach(BUILD_TYPE ${ZSTD_BUILD_TYPES})
  set(_ZSTD_BUILD_TYPE_LIB_NAMES "")
  foreach(_ZSTD_LIB_NAME ${_ZSTD_LIB_NAMES})
    if(BUILD_TYPE STREQUAL DEBUG)
      set(_ZSTD_LIB_NAME "${_ZSTD_LIB_NAME}${ZSTD_DEBUG_SUFFIX}")
    endif()
    list(APPEND _ZSTD_BUILD_TYPE_LIB_NAMES ${_ZSTD_LIB_NAME})
  endforeach()

  find_library(Zstd_LIBRARY_${BUILD_TYPE}
    NAMES ${_ZSTD_BUILD_TYPE_LIB_NAMES}
    NAMES_PER_DIR
    ${_FIND_ZSTD_ADDITIONAL_OPTIONS}
    PATHS ${_ZSTD_LIBRARYDIR_SEARCH_DIRS}
    PATH_SUFFIXES ${CMAKE_INSTALL_LIBDIR} lib64 lib
  )

  list(APPEND Zstd_LIB_COMPONENTS ${Zstd_LIBRARY_${BUILD_TYPE}})
endforeach()

unset(_ZSTD_LIB_NAME)
unset(_ZSTD_LIB_NAMES)
unset(_ZSTD_BUILD_TYPE_LIB_NAMES)

# Reset library suffix

set(CMAKE_FIND_LIBRARY_SUFFIXES ${_ZSTD_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
unset(_ZSTD_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)

if(Zstd_LIBRARY_DEBUG AND Zstd_LIBRARY_RELEASE)
  # if the generator is multi-config or if CMAKE_BUILD_TYPE is set for
  # single-config generators, set optimized and debug libraries
  get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(_isMultiConfig OR CMAKE_BUILD_TYPE)
    set(Zstd_LIBRARY optimized ${Zstd_LIBRARY_RELEASE} debug ${Zstd_LIBRARY_DEBUG})
  else()
    # For single-config generators where CMAKE_BUILD_TYPE has no value,
    # just use the release libraries
    set(Zstd_LIBRARY ${Zstd_LIBRARY_RELEASE})
  endif()
  set(Zstd_LIBRARIES optimized ${Zstd_LIBRARY_RELEASE} debug ${Zstd_LIBRARY_DEBUG})
endif()

# if only the release version was found, set the debug variable also to the release version
if(Zstd_LIBRARY_RELEASE AND NOT Zstd_LIBRARY_DEBUG)
  set(Zstd_LIBRARY_DEBUG ${Zstd_LIBRARY_RELEASE})
  set(Zstd_LIBRARY       ${Zstd_LIBRARY_RELEASE})
  set(Zstd_LIBRARIES     ${Zstd_LIBRARY_RELEASE})
endif()

# if only the debug version was found, set the release variable also to the debug version
if(Zstd_LIBRARY_DEBUG AND NOT Zstd_LIBRARY_RELEASE)
  set(Zstd_LIBRARY_RELEASE ${Zstd_LIBRARY_DEBUG})
  set(Zstd_LIBRARY         ${Zstd_LIBRARY_DEBUG})
  set(Zstd_LIBRARIES       ${Zstd_LIBRARY_DEBUG})
endif()

# If the debug & release library ends up being the same, omit the keywords
if("${Zstd_LIBRARY_RELEASE}" STREQUAL "${Zstd_LIBRARY_DEBUG}")
  set(Zstd_LIBRARY   ${Zstd_LIBRARY_RELEASE} )
  set(Zstd_LIBRARIES ${Zstd_LIBRARY_RELEASE} )
endif()

if(Zstd_LIBRARY)
  set(Zstd_FOUND TRUE)
else()
  set(Zstd_FOUND FALSE)
endif()

# ------------------------------------------------------------------------
#  Cache and set Zstd_FOUND
# ------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Zstd
  FOUND_VAR Zstd_FOUND
  REQUIRED_VARS
    Zstd_LIBRARY
    Zstd_INCLUDE_DIR
  VERSION_VAR Zstd_VERSION
)

if(NOT Zstd_FOUND)
  if(Zstd_FIND_REQUIRED)
    message(FATAL_ERROR "Unable to find Zstd")
  endif()
  return()
endif()

# Partition release/debug lib vars

set(Zstd_RELEASE_LIBRARIES ${Zstd_LIBRARY_RELEASE})
get_filename_component(Zstd_RELEASE_LIBRARY_DIRS ${Zstd_RELEASE_LIBRARIES} DIRECTORY)
set(Zstd_DEBUG_LIBRARIES ${Zstd_LIBRARY_DEBUG})
get_filename_component(Zstd_DEBUG_LIBRARY_DIRS ${Zstd_DEBUG_LIBRARIES} DIRECTORY)
set(Zstd_LIBRARIES ${Zstd_RELEASE_LIBRARIES})
set(Zstd_LIBRARY_DIRS ${Zstd_RELEASE_LIBRARY_DIRS})
set(Zstd_INCLUDE_DIRS ${Zstd_INCLUDE_DIR})

# Configure lib type. If XXX_USE_STATIC_LIBS, we always assume a static
# lib is in use. If win32, we can't mark the import .libs as shared, so
# these are always marked as UNKNOWN. Otherwise, infer from extension.
set(ZSTD_LIB_TYPE UNKNOWN)
if(ZSTD_USE_STATIC_LIBS)
  set(ZSTD_LIB_TYPE STATIC)
elseif(UNIX)
  get_filename_component(_ZSTD_EXT ${Zstd_LIBRARY_RELEASE} EXT)
  if(_ZSTD_EXT STREQUAL ".a")
    set(ZSTD_LIB_TYPE STATIC)
  elseif(_ZSTD_EXT STREQUAL ".so" OR
         _ZSTD_EXT STREQUAL ".dylib")
    set(ZSTD_LIB_TYPE SHARED)
  endif()
endif()

get_filename_component(Zstd_LIBRARY_DIRS ${Zstd_LIBRARY_RELEASE} DIRECTORY)

if(NOT TARGET Zstd::zstd)
  add_library(Zstd::zstd ${ZSTD_LIB_TYPE} IMPORTED)
  set_target_properties(Zstd::zstd PROPERTIES
    INTERFACE_COMPILE_OPTIONS "${PC_Zstd_CFLAGS_OTHER}"
    INTERFACE_INCLUDE_DIRECTORIES "${Zstd_INCLUDE_DIRS}")

  # Standard location
  set_target_properties(Zstd::zstd PROPERTIES
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${Zstd_LIBRARY}")

  # Release location
  if(EXISTS "${Zstd_LIBRARY_RELEASE}")
    set_property(TARGET Zstd::zstd APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(Zstd::zstd PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
      IMPORTED_LOCATION_RELEASE "${Zstd_LIBRARY_RELEASE}")
  endif()

  # Debug location
  if(EXISTS "${Zstd_LIBRARY_DEBUG}")
    set_property(TARGET Zstd::zstd APPEND PROPERTY
      IMPORTED_CONFIGURATIONS DEBUG)
    set_target_properties(Zstd::zstd PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
      IMPORTED_LOCATION_DEBUG "${Zstd_LIBRARY_DEBUG}")
  endif()

  # A static zstd built with multithreading needs pthreads at link time
  if(ZSTD_LIB_TYPE STREQUAL STATIC AND UNIX)
    find_package(Threads REQUIRED)
    set_property(TARGET Zstd::zstd APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES Threads::Threads)
  endif()
endif()
