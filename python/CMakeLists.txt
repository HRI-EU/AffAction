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

SET(PYBIND11_CPP_STANDARD -std=c++14)
SET(PYBIND_DIR_HINT "${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11")

IF (WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # Put module where dlls are located
ENDIF(WIN32)

IF (EXISTS "${PYBIND_DIR_HINT}" AND IS_DIRECTORY "${PYBIND_DIR_HINT}")

  FIND_PACKAGE(Python3 REQUIRED COMPONENTS Interpreter Development)
  MESSAGE(STATUS "pybind11 found - creating python wrapper")
  ADD_SUBDIRECTORY("${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11" "${CMAKE_BINARY_DIR}/pybind11")
  SET(MODULE_SRCS module.cpp)

  # The link-time optimization has an internal error when used from ToolBos, so disable it.
  # Mark pybind11 include dirs as system to supress warnings in pybind11 code
  PYBIND11_ADD_MODULE(pyAffaction ${MODULE_SRCS})# NO_EXTRAS)
  TARGET_COMPILE_DEFINITIONS(pyAffaction PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(pyAffaction PUBLIC AffAction)
  TARGET_INCLUDE_DIRECTORIES(pyAffaction SYSTEM BEFORE PUBLIC ${PYBIND11_INCLUDE_DIR} ../external)

  PYBIND11_ADD_MODULE(pyPushT pyPushT.cpp)# NO_EXTRAS)
  TARGET_COMPILE_DEFINITIONS(pyPushT PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(pyPushT PUBLIC AffAction)
  TARGET_INCLUDE_DIRECTORIES(pyPushT SYSTEM BEFORE PUBLIC ${PYBIND11_INCLUDE_DIR} ../external)

  PYBIND11_ADD_MODULE(pyTeleOp module_teleop.cpp)# NO_EXTRAS)
  TARGET_COMPILE_DEFINITIONS(pyTeleOp PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(pyTeleOp PUBLIC AffAction)
  TARGET_INCLUDE_DIRECTORIES(pyTeleOp SYSTEM BEFORE PUBLIC ${PYBIND11_INCLUDE_DIR} ../external)

ELSE()

  MESSAGE(STATUS "pybind11 not found - skipping python wrapper")
  
ENDIF()

IF(UNIX)
  INSTALL(TARGETS pyAffaction pyPushT pyTeleOp
          RUNTIME DESTINATION bin 
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib/AffAction)
ELSE()
  INSTALL(TARGETS pyAffaction pyPushT pyTeleOp
          RUNTIME DESTINATION bin 
          LIBRARY DESTINATION bin
          ARCHIVE DESTINATION lib/AffAction)
ENDIF()
