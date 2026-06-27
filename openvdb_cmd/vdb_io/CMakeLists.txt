# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: Apache-2.0
#
#[=======================================================================[

  CMake Configuration for VDB IO

#]=======================================================================]

cmake_minimum_required(VERSION 3.24)
project(VDBIO LANGUAGES CXX)

include(GNUInstallDirs)

set(SOURCE_FILES main.cc)
add_executable(vdb_io ${SOURCE_FILES})
target_link_libraries(vdb_io ${OPENVDB_BINARIES_DEPENDENT_LIBS})

install(TARGETS vdb_io RUNTIME DESTINATION ${OPENVDB_INSTALL_BINDIR})
