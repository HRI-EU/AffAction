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

SET(PYBIND_DIR_HINT "${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11")

# Only build Python modules if the pybind11 source exists.
IF(NOT EXISTS "${PYBIND_DIR_HINT}/CMakeLists.txt")

  MESSAGE(STATUS "pybind11 not found - skipping Python wrappers")

ELSE()

  # Use CMake's modern FindPython mechanism.
  SET(PYBIND11_FINDPYTHON ON)

  # Use CMake's built-in Python discovery on Windows,
  # bypassing the vcpkg Python wrapper.
  IF(WIN32)
    SET(Python3_FIND_COMPONENTS Interpreter Development.Module)
    SET(Python3_FIND_REQUIRED TRUE)
    SET(Python3_FIND_REQUIRED_Interpreter TRUE)
    SET("Python3_FIND_REQUIRED_Development.Module" TRUE)
    INCLUDE("${CMAKE_ROOT}/Modules/FindPython3.cmake")
  ELSE()
    FIND_PACKAGE(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  ENDIF()

  # Verify that Python module development is available
  IF(NOT Python3_FOUND OR
      NOT TARGET Python3::Module OR
      NOT COMMAND Python3_add_library)
    MESSAGE(FATAL_ERROR "Python module development not available for ${Python3_EXECUTABLE}")
  ENDIF()

  MESSAGE(STATUS "pybind11 found - creating python wrapper")
  MESSAGE(STATUS "Python executable: ${Python3_EXECUTABLE}")
  MESSAGE(STATUS "Python includes: ${Python3_INCLUDE_DIRS}")
  MESSAGE(STATUS "Python libraries: ${Python3_LIBRARIES}")

  ADD_SUBDIRECTORY("${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11" "${CMAKE_BINARY_DIR}/pybind11")

  IF(WIN32)
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  ENDIF()

  PYBIND11_ADD_MODULE(pyAffaction module.cpp)# NO_EXTRAS)
  PYBIND11_ADD_MODULE(pyPushT pyPushT.cpp)# NO_EXTRAS)
  PYBIND11_ADD_MODULE(pyTeleOp module_teleop.cpp)# NO_EXTRAS)

  FOREACH(target IN ITEMS pyAffaction pyPushT pyTeleOp)
    TARGET_COMPILE_FEATURES(${target} PRIVATE cxx_std_14)
    TARGET_COMPILE_DEFINITIONS(${target} PRIVATE RCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
    TARGET_LINK_LIBRARIES(${target} PRIVATE AffAction)
    TARGET_INCLUDE_DIRECTORIES(${target} SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../external")
  ENDFOREACH()

  # Installation
  IF(WIN32)
    INSTALL(TARGETS pyAffaction pyPushT pyTeleOp
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
        ARCHIVE DESTINATION lib/AffAction)
  ELSE()
    INSTALL(TARGETS pyAffaction pyPushT pyTeleOp
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/AffAction)
  ENDIF()

ENDIF()
