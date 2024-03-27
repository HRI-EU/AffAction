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

IF (EXISTS "${PYBIND_DIR_HINT}" AND IS_DIRECTORY "${PYBIND_DIR_HINT}")

  MESSAGE(STATUS "pybind11 found - creating python wrapper")
  ADD_SUBDIRECTORY("${CMAKE_CURRENT_SOURCE_DIR}/../../pybind11" "${CMAKE_BINARY_DIR}/pybind11")
  SET(MODULE_SRCS module.cpp)

  # The link-time optimization has an internal error when used from ToolBos, so disable it.
  PYBIND11_ADD_MODULE(pyAffaction ${MODULE_SRCS})# NO_EXTRAS)
  TARGET_COMPILE_DEFINITIONS(pyAffaction PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")

  TARGET_LINK_LIBRARIES(pyAffaction PUBLIC AffAction)

  # Mark pybind11 include dirs as system to supress warnings in pybind11 code
  TARGET_INCLUDE_DIRECTORIES(pyAffaction SYSTEM BEFORE PUBLIC ${PYBIND11_INCLUDE_DIR} ../external)

  # Setup site-packages link for python build mode. This has issues if you don't have write permissions
  OPTION(SETUP_PYTHON_DEVEL "Add the build output dir to the pythonpath")
  IF (SETUP_PYTHON_DEVEL)
    # Find site-packages dir
    EXECUTE_PROCESS(
        COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    SET(RCSPYSIM_PTH "${PYTHON_SITE_PACKAGES_DIR}/rcspysim.pth")
  
    MESSAGE(STATUS "-- Linking output directory onto python path")
    FILE(WRITE "${RCSPYSIM_PTH}" "\
${LIBRARY_OUTPUT_PATH}
${CMAKE_SOURCE_DIR}/src/python")

  ENDIF(SETUP_PYTHON_DEVEL)

ELSE()

  SET(PYBIND_DIR_HINT_ROS "/opt/ros/galactic/share/cmake/pybind11")
  FIND_PACKAGE(pybind11 CONFIG HINTS ${PYBIND_DIR_HINT_ROS})

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
  
ENDIF()

IF(UNIX)
  INSTALL(TARGETS pyAffaction
          RUNTIME DESTINATION bin 
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib/AffAction)
ELSE()
  INSTALL(TARGETS pyAffaction
          RUNTIME DESTINATION bin 
          LIBRARY DESTINATION bin
          ARCHIVE DESTINATION lib/AffAction)
ENDIF()
