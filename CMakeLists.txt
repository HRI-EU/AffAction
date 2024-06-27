################################################################################
#
#  Copyright (c) 2020, Honda Research Institute Europe GmbH.
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
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
SET(BST_INSTALL_CATEGORY Libraries)
STRING (REPLACE "\\" "/" PIPER_PATH "$ENV{SIT}/External/piper")
SET(AFFACTION_PIPER_PATH "${PIPER_PATH}" CACHE STRING "Path to piper executable")

OPTION(AFFACTION_WITH_OPTIM "Build with optimization" OFF)

################################################################################
# Macro to copy dlls into bin directory
################################################################################
IF (NOT COMMAND COPY_DLL)
MACRO(COPY_DLL libname)
IF (WIN32)
  ADD_CUSTOM_COMMAND(
      TARGET ${libname} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/${libname}.dll
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/${libname}.dll)
ENDIF(WIN32)
ENDMACRO()
ENDIF(NOT COMMAND COPY_DLL)

################################################################################
# Macro to copy directories
################################################################################
IF (NOT COMMAND COPY_CONFIG_DIR)
MACRO(COPY_CONFIG_DIR CFG_FROM CFG_TO_BASEDIR CFG_TO_SUBDIRNAME)
  SET(CFG_TO ${CFG_TO_BASEDIR}/${CFG_TO_SUBDIRNAME})

  IF (WIN32 AND (NOT IS_DIRECTORY "${CFG_TO}") AND (EXISTS "${CFG_TO}"))
    MESSAGE(STATUS "Removing non-directory ${CONFIG_DST_DIR}")
    FILE(REMOVE ${CONFIG_DST_DIR})
  ENDIF()

  IF(NOT EXISTS "${CFG_TO}")

    IF (NOT EXISTS ${CFG_TO_BASEDIR})
      FILE(MAKE_DIRECTORY ${CFG_TO_BASEDIR})
    ENDIF()

    IF(UNIX)
      SET(CONFIGDIR_COPY_COMMAND create_symlink)
      MESSAGE(STATUS "Linking config directory from ${CFG_FROM} to ${CFG_TO}")
    ELSE()
      SET(CONFIGDIR_COPY_COMMAND copy_directory)
      MESSAGE(STATUS "Copying config directory from ${CFG_FROM} to ${CFG_TO}")
    ENDIF()

    EXECUTE_PROCESS (COMMAND
     ${CMAKE_COMMAND} -E ${CONFIGDIR_COPY_COMMAND} ${CFG_FROM} ${CFG_TO})
    MESSAGE(STATUS "Config directory ${CFG_TO} already exists - skipping")
  ELSE()
    MESSAGE(STATUS "Config directory ${CFG_TO} already exists - skipping")
  ENDIF()
ENDMACRO()
ENDIF(NOT COMMAND COPY_CONFIG_DIR)

################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(AffAction)
  SET(OpenGL_GL_PREFERENCE GLVND) # Removes warning OpenGL_GL_PREFERENCE has not been set to "GLVND" or "LEGACY"
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release." FORCE)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  SET(RCS_CONFIG_DIR "$ENV{SIT}/Libraries/Rcs/1.0/config")
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Rcs/1.0/share/cmake/Rcs)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Tropic/1.0/share/cmake/Tropic)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/ESLib/1.0/share/cmake/EventSystem)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/External/MuJoCo/1.0/$ENV{MAKEFILE_PLATFORM}/lib/cmake/mujoco)
  FIND_PACKAGE(Rcs REQUIRED)
  FIND_PACKAGE(Tropic REQUIRED)
  FIND_PACKAGE(mujoco QUIET)
  MESSAGE(STATUS "Compiling AffAction as stand-alone project")
  SET(STANDALONE_PROJECT ON)
  FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  SET(ESLIB_INCLUDE_DIR "$ENV{SIT}/Libraries/ESLib/1.0/include")
  SET(WEBSOCKET_INCLUDE_DIR "$ENV{SIT}/Libraries/DexterousCooperation/1.0/include/websocketpp/include")
  INCLUDE($ENV{SIT}/Libraries/ESLib/1.0/share/ESLibValidation/cmake/EventValidation.cmake)
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/AffAction/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  STRING(LENGTH "${CMAKE_SOURCE_DIR}/" RCS_BASE_PATH_LENGTH)
  ADD_DEFINITIONS("-DRCS_BASE_PATH_LENGTH=${RCS_BASE_PATH_LENGTH}")
ELSE()
  SET(RCS_CONFIG_DIR "config")
  SET(ESLIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ESLib/include")
  SET(WEBSOCKET_INCLUDE_HINT1 "${CMAKE_CURRENT_SOURCE_DIR}/../websocketpp")
  SET(WEBSOCKET_INCLUDE_HINT2 "${CMAKE_CURRENT_SOURCE_DIR}/../DexterousCooperation/external/websocketpp/include")
  IF (EXISTS ${WEBSOCKET_INCLUDE_HINT1})
    SET(WEBSOCKET_INCLUDE_DIR ${WEBSOCKET_INCLUDE_HINT1})
  ELSEIF (EXISTS ${WEBSOCKET_INCLUDE_HINT2})
    SET(WEBSOCKET_INCLUDE_DIR ${WEBSOCKET_INCLUDE_HINT2})
  ELSE()
    MESSAGE(STATUS "Could not find websocket header-only library")
  ENDIF()
  LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as in-tree project")
ENDIF()

ADD_DEFINITIONS(-DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")

################################################################################
# Compile settings
################################################################################
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_CXX_STANDARD 14)

################################################################################
# Setup exported target collection
################################################################################

# Setup AFFACTION_EXPORT_LIBRARIES so that CMakeLists.txt can modify it
SET(AFFACTION_EXPORT_LIBRARIES "" CACHE INTERNAL "")

# Create a macro for adding an export library
FUNCTION(AFFACTION_ADD_EXPORT_LIBRARY library)
  SET(AFFACTION_EXPORT_LIBRARIES ${AFFACTION_EXPORT_LIBRARIES} ${library} CACHE INTERNAL "")
ENDFUNCTION()

IF (WIN32)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)   # M_PI etc. for windows
ENDIF(WIN32)

################################################################################
# Compile settings
################################################################################
IF(UNIX)
  # SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC -Wno-long-long -Wno-variadic-macros -std=c99")
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC -Wno-format -Wno-long-long -Wno-variadic-macros")
  # ADD_DEFINITIONS(-D__linux__)

  # Flag -rdynamic needed for backtrace()
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -rdynamic")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
  ENDIF()

  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    # setting march to core2 to enable valgrind debugging (also good for Xeon)
    # Low level optimization for debug mode, flag for checking stack corruption, flag for debug output
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
  ELSE()
    # setting march to native for optimal performance on local machine
    # Strong optimization for release mode
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ggdb")
  ENDIF()

ENDIF()

################################################################################
#
# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH. Required for Vortex and other
# dependencies from the SIT.
# Note: This only works with CMake >= 3.0. With 2.8, the library directories of
# the dependencies must be added to the LD_LIBRARY_PATH manually in order to
# use the installed executables. This has no effect on using the package in
# another CMake project.
#
################################################################################
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

################################################################################
# ROS
################################################################################
OPTION(USE_AFFACTION_ROS "Enable ROS for the retro project" OFF)

IF(USE_AFFACTION_ROS)
  LIST(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic) #$ENV{ROS_DISTRO})
  SET(SMILE_WS_PATH "$ENV{SMILE_WORKSPACE_PATH}" CACHE STRING "Path to smile ROS workspace")
  FIND_PACKAGE(roscpp QUIET)
  IF(NOT roscpp_FOUND)
    MESSAGE(STATUS "Couldn't find roscpp, so not building ROS-specific functionality. Did you source the ROS setup.sh file?")
  ELSE()
    MESSAGE(STATUS "Found roscpp installed at ${roscpp_DIR}, so building ROS-specific functionality.")
  ENDIF()
  FIND_PACKAGE(actionlib REQUIRED)
ENDIF(USE_AFFACTION_ROS)

###############################################################################
# Library
###############################################################################
FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
SET(CMAKE_AUTOMOC ON)

SET(ACTION_SRCS
src/ActionResult.cpp
src/ActionBase.cpp
src/ActionFromXML.cpp
src/ActionGazeAtHand.cpp
src/ActionGet.cpp
src/ActionPass.cpp
src/ActionPoint.cpp
src/ActionPush.cpp
src/ActionPut.cpp
src/ActionPour.cpp
src/ActionPose.cpp
src/ActionGaze.cpp
src/ActionPanTilt.cpp
src/ActionDoor.cpp
src/ActionDrop.cpp
src/ActionWait.cpp
src/ActionScrew.cpp
src/ActionShake.cpp
src/ActionFingerPush.cpp
src/ActionComposite.cpp
src/Affordance.cpp
src/AnimationSequence.cpp
src/Capability.cpp
src/ActionSequence.cpp
src/AffordanceEntity.cpp
src/Manipulator.cpp
src/Agent.cpp
src/ActionScene.cpp
src/SceneJsonHelpers.cpp
src/ActionFactory.cpp
src/TrajectoryPredictor.cpp
src/TrackerBase.cpp
src/PredictionTree.cpp
src/ArucoTracker.cpp
src/ArucoMultiCamTracker.cpp
src/AzureSkeletonTracker.cpp
src/LandmarkBase.cpp
src/ConcurrentExecutor.cpp
src/ConcurrentSceneQuery.cpp
)

SET(ECS_SRCS
src/EntityBase.cpp
src/GraphicsWindow.cpp
src/GraphComponent.cpp
src/IKComponent.cpp
src/TrajectoryComponent.cpp
src/EventGui.cpp
src/TaskGuiComponent.cpp
src/TextEditComponent.cpp
src/ActionComponent.cpp
src/HardwareComponent.cpp
src/WebsocketActionComponent.cpp
)

SET(EXAMPLE_SRCS
examples/ExampleActionsECS.cpp
src/JacoShmComponent.cpp
src/TTSComponent.cpp
)

IF (USE_AFFACTION_ROS)
  SET(ROS_SRCS
  src/ros/PtuComponent.cpp
  src/ros/PtuActionComponent.cpp
  src/ros/RespeakerComponent.cpp
  src/ros/NuanceTTSComponent.cpp
)
ENDIF(USE_AFFACTION_ROS)

SET_PROPERTY(SOURCE src/TTSComponent.cpp PROPERTY COMPILE_DEFINITIONS AFFACTION_PIPER_PATH="${AFFACTION_PIPER_PATH}")

ADD_LIBRARY(AffAction ${ACTION_SRCS} ${ECS_SRCS} ${EXAMPLE_SRCS} ${ROS_SRCS})

TARGET_INCLUDE_DIRECTORIES(AffAction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
    $<BUILD_INTERFACE:${ESLIB_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples>
    $<BUILD_INTERFACE:${WEBSOCKET_INCLUDE_DIR}>
    $<BUILD_INTERFACE:$ENV{SIT}/External/asio-1.12.1/include>
    $<INSTALL_INTERFACE:include/AffAction>)

TARGET_LINK_LIBRARIES(AffAction Tropic RcsGraphics RcsGui RcsPhysics)

# zmq library is used to communicate with python from MediaPipeComponent
IF (WIN32)
  FIND_PACKAGE(cppzmq REQUIRED)
  TARGET_LINK_LIBRARIES(AffAction cppzmq)
ELSE()
  TARGET_LINK_LIBRARIES(AffAction zmq)
ENDIF()

# We add this option to force the linker to export the symbols. This
# solves the issue that it will not be exposed, since all initialization
# happens inside statically. Under Unix, we can use -Wl,--no-as-needed
IF (WIN32)
  TARGET_LINK_OPTIONS(AffAction PUBLIC /INCLUDE:AffActionExampleInfo)
ENDIF()

IF(AFFACTION_WITH_OPTIM)
  IF(TARGET RcsOptim)
    MESSAGE(STATUS "Building AffAction with optimization support")
    #LIST(APPEND ActionBase src/OptimizationSet.cpp)
    TARGET_COMPILE_DEFINITIONS(AffAction PUBLIC -DSMILEACTIONS_WITH_OPTIM)
    TARGET_LINK_LIBRARIES(AffAction RcsOptim)
  ELSE()
    MESSAGE(STATUS "Building AffAction without optimization support")
  ENDIF()

ENDIF()

# PTUComponent currently uses ROS
IF (USE_AFFACTION_ROS)
  TARGET_LINK_LIBRARIES(AffAction ${roscpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
  TARGET_INCLUDE_DIRECTORIES(AffAction SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
  TARGET_COMPILE_DEFINITIONS(AffAction PUBLIC -DUSE_ROS)  
  TARGET_INCLUDE_DIRECTORIES(AffAction PUBLIC ${SMILE_WS_PATH}/devel/include)
  TARGET_LINK_DIRECTORIES(AffAction PUBLIC /opt/ros/noetic/lib)
  TARGET_LINK_LIBRARIES(AffAction actionlib)
ENDIF(USE_AFFACTION_ROS)

################################################################################
# Executables
################################################################################
ADD_EXECUTABLE(ExampleRunnerAff examples/ExampleRunner.cpp)
IF (UNIX)
  TARGET_LINK_LIBRARIES(ExampleRunnerAff -Wl,--no-as-needed RcsExamples AffAction -Wl,--as-needed)
ELSE()
  TARGET_LINK_LIBRARIES(ExampleRunnerAff RcsExamples AffAction)
ENDIF()

TARGET_COMPILE_DEFINITIONS(ExampleRunnerAff PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")

ADD_EXECUTABLE(TestAffordance examples/TestAffordance.cpp)
TARGET_LINK_LIBRARIES(TestAffordance AffAction)

ADD_EXECUTABLE(TestLLMSim examples/TestLLMSim.cpp)
TARGET_LINK_LIBRARIES(TestLLMSim AffAction)

ADD_EXECUTABLE(TreeTest examples/TreeTest.cpp)
TARGET_LINK_LIBRARIES(TreeTest RcsCore)

# To fix issues with remote graphics, we call XInitThreads in TestLLMSim.
# That's why we need to link against X11 under Linux.
IF (UNIX)
  TARGET_LINK_LIBRARIES(TestLLMSim X11)
ENDIF()

################################################################################
# Python module with pybind11
################################################################################
ADD_SUBDIRECTORY(python)

################################################################################
# Doxygen html documentation
################################################################################
ADD_SUBDIRECTORY (doc)

###############################################################################
# Install the libraries and the binaries
###############################################################################
#INSTALL(TARGETS AffAction EXPORT AffActionExport DESTINATION lib)
INSTALL(TARGETS AffAction EXPORT AffActionExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/AffAction)

INSTALL(TARGETS TestLLMSim TestAffordance 
        RUNTIME DESTINATION bin 
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/AffAction)

###############################################################################
# Install the headers
###############################################################################
INSTALL(
 DIRECTORY src/
 DESTINATION include/AffAction
 FILES_MATCHING PATTERN "*.h"
)

###############################################################################
# Mark for export
###############################################################################
#SMILEACTIONS_ADD_EXPORT_LIBRARY(SmileActions)

###############################################################################
# copy the .dll file to the same folder as the executable
###############################################################################
COPY_DLL(AffAction)

################################################################################
# Copy / link config directories
################################################################################
COPY_CONFIG_DIR("${CMAKE_CURRENT_SOURCE_DIR}/config" "${PROJECT_BINARY_DIR}/config/xml" "AffAction")

################################################################################
# Target export script
################################################################################
INCLUDE(ExportAffAction)
