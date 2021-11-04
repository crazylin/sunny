# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindGPIOD
-------

Finds the GPIOD library (libgpiod).

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``GPIOD::GPIOD``
  The GPIOD library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``GPIOD_FOUND``
  True if the system has the GPIOD library.
``GPIOD_INCLUDE_DIRS``
  The GPIOD include directories.
``GPIOD_LIBRARIES``
  The GPIOD libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``GPIOD_INCLUDE_DIR``
  The directory containing GxIAPI.h.
``GPIOD_LIBRARY``
  The path to the GxIAPI library.

#]=======================================================================]
# Try to find the GPIOD library

# Find the include path which includes gpiod.h
find_path(GPIOD_INCLUDE_DIR
  NAMES "gpiod.h"
  DOC "The directory containing gpiod.h."
)

# Find the specific libary
find_library(GPIOD_LIBRARY
  NAMES "gpiod"
  DOC "The path to the GPIOD library file."
)

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GPIOD
  REQUIRED_VARS
    GPIOD_LIBRARY
    GPIOD_INCLUDE_DIR
)

# Setup import target which can be utilized by target_link_libraries
if(GPIOD_FOUND AND NOT TARGET GPIOD::GPIOD)
  add_library(GPIOD::GPIOD SHARED IMPORTED)
  set_target_properties(GPIOD::GPIOD PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GPIOD_INCLUDE_DIR}"
    IMPORTED_LOCATION "${GPIOD_LIBRARY}"
  )
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(GPIOD_INCLUDE_DIRS ${GPIOD_INCLUDE_DIR})
set(GPIOD_LIBRARIES ${GPIOD_LIBRARY})

mark_as_advanced(
  GPIOD_INCLUDE_DIR
  GPIOD_LIBRARY
)
