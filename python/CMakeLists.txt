################################################################################
#
#  Copyright (c) Honda Research Institute Europe GmbH.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

SET(PYBIND_DIR_HINT1 "${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11")
SET(PYBIND_DIR_HINT2 "/opt/ros/galactic/share/cmake/pybind11")
FIND_PACKAGE(pybind11 CONFIG HINTS ${PYBIND_DIR_HINT1} ${PYBIND_DIR_HINT2})

IF (pybind11_FOUND)

  MESSAGE(STATUS "pybind11 found - creating python wrapper")

  # The link-time optimization has an internal error when used from ToolBos, so disable it.
  PYBIND11_ADD_MODULE(pyAffaction module.cpp)
  TARGET_COMPILE_DEFINITIONS(pyAffaction PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(pyAffaction PUBLIC AffAction)

  # Mark pybind11 include dirs as system to supress warnings in pybind11 code
  TARGET_INCLUDE_DIRECTORIES(pyAffaction SYSTEM BEFORE PUBLIC ${PYBIND11_INCLUDE_DIR} ../external)

ELSE(pybind11_FOUND)
  
  MESSAGE(STATUS "pybind11 not found - skipping python wrapper")
  
ENDIF(pybind11_FOUND)
