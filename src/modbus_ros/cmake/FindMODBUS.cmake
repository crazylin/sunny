# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindMODBUS
-------

Finds the MODBUS library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``MODBUS::MODBUS``
  The modbus library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``MODBUS_FOUND``
  True if the system has the MODBUS library.
``MODBUS_VERSION`` and ``MODBUS_VERSION_STRING``
  The version of the MODBUS library which was found.
``MODBUS_INCLUDE_DIRS``
  The MODBUS include directories.
``MODBUS_LIBRARIES``
  The MODBUS libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``MODBUS_INCLUDE_DIR``
  The directory containing MODBUS.h.
``MODBUS_LIBRARY``
  The path to the MODBUS library.

#]=======================================================================]
# Try to find the MODBUS library

# Find the include path which includes modbus.h
find_path(MODBUS_INCLUDE_DIR
  NAMES "modbus.h"
  PATH_SUFFIXES "modbus"
  DOC "The directory containing modbus.h."
)

# Find the specific libary under <prefix>/lib/[x64|x86]/
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_LIBRARY_ARCHITECTURE x64)
  else()
    set(CMAKE_LIBRARY_ARCHITECTURE x86)
  endif()
endif()

# Find the specific libary
find_library(MODBUS_LIBRARY
  NAMES "modbus"
  DOC "The path to the MODBUS library."
)

unset(CMAKE_LIBRARY_ARCHITECTURE)

# Extract version information
file(STRINGS "${MODBUS_INCLUDE_DIR}/modbus-version.h" MODBUS_VERSION REGEX "LIBMODBUS_VERSION_STRING")
string(REGEX MATCH "[0-9.]+" MODBUS_VERSION "${MODBUS_VERSION}")

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MODBUS
  REQUIRED_VARS
    MODBUS_LIBRARY
    MODBUS_INCLUDE_DIR
  VERSION_VAR
    MODBUS_VERSION
)

# Setup import target which can be utilized by target_link_libraries
if(MODBUS_FOUND AND NOT TARGET MODBUS::MODBUS)
  add_library(MODBUS::MODBUS SHARED IMPORTED)
  set_target_properties(MODBUS::MODBUS PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MODBUS_INCLUDE_DIR}"
    IMPORTED_LOCATION "${MODBUS_LIBRARY}"
  )
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(MODBUS_INCLUDE_DIRS ${MODBUS_INCLUDE_DIR})
set(MODBUS_LIBRARIES ${MODBUS_LIBRARY})

mark_as_advanced(
  MODBUS_INCLUDE_DIR
  MODBUS_LIBRARY
)

# compatibility variables
set(MODBUS_VERSION_STRING ${MODBUS_VERSION})
