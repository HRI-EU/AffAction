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
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

IF (APPLE)
  CMAKE_MINIMUM_REQUIRED(VERSION 3.25)
  CMAKE_POLICY(SET CMP0142 NEW)   # Skip duplicate build type library paths
ENDIF(APPLE)

SET(BST_INSTALL_CATEGORY Libraries)
STRING (REPLACE "\\" "/" PIPER_PATH "$ENV{SIT}/External/piper")
SET(AFFACTION_PIPER_PATH "${PIPER_PATH}" CACHE STRING "Path to piper executable")

OPTION(AFFACTION_WITH_OPTIM "Build with optimization" OFF)
OPTION(AFFACTION_WITH_MQTT "Enable Mqtt communication" OFF)
OPTION(AFFACTION_WITH_ROS "Enable ROS for the AffAction project" OFF)
OPTION(AFFACTION_WITH_RESPEAKER "Enable respeaker" OFF)
IF (WIN32)
  OPTION(AFFACTION_WITH_PCAN_BASIC "Enable PCan-Basic PTU driver" OFF)
ENDIF()

################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(AffAction)
  SET(OpenGL_GL_PREFERENCE GLVND) # Removes warning OpenGL_GL_PREFERENCE has not been set to "GLVND" or "LEGACY"
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release." FORCE)
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
  STRING(LENGTH "${CMAKE_CURRENT_SOURCE_DIR}/" RCS_BASE_PATH_LENGTH)
  ADD_DEFINITIONS("-DRCS_BASE_PATH_LENGTH=${RCS_BASE_PATH_LENGTH}")
ELSE()
  SET(RCS_CONFIG_DIR "config")
  SET(ESLIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ESLib/include")
  SET(WEBSOCKET_INCLUDE_HINT1 "${CMAKE_CURRENT_SOURCE_DIR}/../websocketpp")
  IF (EXISTS ${WEBSOCKET_INCLUDE_HINT1})
    SET(WEBSOCKET_INCLUDE_DIR ${WEBSOCKET_INCLUDE_HINT1})
  ELSE()
    MESSAGE(STATUS "Could not find websocket header-only library")
  ENDIF()
  LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../Rcs/cmake)
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
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)   # for dlls

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

  IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Suppress the "implicit conversion loses integer precision" warning
    add_compile_options(-Wno-conversion)
    #add_compile_options(-Wno-unused-function)
    #add_compile_options(-Wno-overloaded-virtual)
    #add_compile_options(-Wno-unused-but-set-variable)
    ##add_compile_options(-Wno-inconsistent-missing-override)
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

IF(AFFACTION_WITH_ROS)
  LIST(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic) #$ENV{ROS_DISTRO})
  SET(SMILE_WS_PATH "$ENV{SMILE_WORKSPACE_PATH}" CACHE STRING "Path to smile ROS workspace")
  FIND_PACKAGE(roscpp QUIET)
  IF(NOT roscpp_FOUND)
    MESSAGE(STATUS "Couldn't find roscpp, so not building ROS-specific functionality. Did you source the ROS setup.sh file?")
  ELSE()
    MESSAGE(STATUS "Found roscpp installed at ${roscpp_DIR}, so building ROS-specific functionality.")
  ENDIF()
  FIND_PACKAGE(actionlib REQUIRED)
ENDIF(AFFACTION_WITH_ROS)

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
src/ActionEyeGaze.cpp
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
src/ComponentBase.cpp
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
src/FaceTracker.cpp
src/YoloTracker.cpp
src/ImageTracker.cpp
src/ImageHelpers.cpp
src/VirtualCamera.cpp
src/SceneHelpers.cpp
src/StringParserTools.cpp
src/HeadGesture.cpp
src/SceneTransformationDataRecorder.cpp
src/SceneTransformationDataPlayer.cpp
)

SET(ECS_SRCS
src/ComponentFactory.cpp
src/EntityBase.cpp
src/GraphicsWindow.cpp
src/GraphComponent.cpp
src/IKComponent.cpp
src/EyeModelIKComponent.cpp
src/TrajectoryComponent.cpp
src/ActionComponent.cpp
src/HardwareComponent.cpp
src/WebsocketActionComponent.cpp
src/LandmarkZmqComponent.cpp
src/ZmqRouterComponent.cpp
src/FaceGestureComponent.cpp
src/GazeComponent.cpp
src/CameraViewComponent.cpp
src/PhysicsComponent.cpp
src/PW70CANInterface.cpp
src/PW70Component.cpp
src/VirtualCameraWindow.cpp
src/TTSComponent.cpp
src/RespeakerSoundDirComponent.cpp
src/RetargetComponent.cpp
src/JointGuiComponent.cpp
)

SET(GUI_SRCS
src/EventGui.cpp
src/TaskGuiComponent.cpp
src/TextEditComponent.cpp
)

SET(EXAMPLE_SRCS
examples/ExampleActionsECS.cpp
examples/ExampleRetarget.cpp
examples/ExampleFlowMatching.cpp
examples/ExampleViapoints.cpp
examples/ExampleJointControl.cpp
src/JacoShmComponent.cpp
src/AzureBodyTrackingComponent.cpp
)

IF (AFFACTION_WITH_ROS)
  SET(ROS_SRCS
  src/ros/PtuComponent.cpp
  src/ros/PtuActionComponent.cpp
  src/ros/RespeakerComponent.cpp
  src/ros/NuanceTTSComponent.cpp
  src/ros/MirrorEyeComponent.cpp
)
ENDIF(AFFACTION_WITH_ROS)

SET_PROPERTY(SOURCE src/TTSComponent.cpp PROPERTY COMPILE_DEFINITIONS AFFACTION_PIPER_PATH="${AFFACTION_PIPER_PATH}")

ADD_LIBRARY(AffAction ${ACTION_SRCS} ${ECS_SRCS} ${GUI_SRCS} ${EXAMPLE_SRCS} ${ROS_SRCS})

TARGET_INCLUDE_DIRECTORIES(AffAction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
    $<BUILD_INTERFACE:${ESLIB_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples>
    $<BUILD_INTERFACE:${WEBSOCKET_INCLUDE_DIR}>
    $<BUILD_INTERFACE:$ENV{SIT}/External/asio-1.12.1/include>
    $<INSTALL_INTERFACE:include/AffAction>)

TARGET_LINK_LIBRARIES(AffAction PUBLIC Tropic RcsGraphics RcsGui RcsPhysics)

IF (TARGET AzureTracker)
  TARGET_LINK_LIBRARIES(AffAction PUBLIC AzureTracker)
ENDIF()

IF (AFFACTION_WITH_PCAN_BASIC)
  FIND_PACKAGE(PCAN-Basic REQUIRED)
  IF(TARGET PCAN-Basic::PCAN-Basic)
    TARGET_LINK_LIBRARIES(AffAction PRIVATE PCAN-Basic::PCAN-Basic)
    TARGET_COMPILE_DEFINITIONS(AffAction PRIVATE -DAFFACTION_WITH_PCAN_BASIC)
    MESSAGE(STATUS "Linking AffAction with PCAN_BASIC_DLLS: ${PCAN_BASIC_DLLS}")
  ENDIF()
ENDIF(AFFACTION_WITH_PCAN_BASIC)

# zmq library is used to communicate with python from MediaPipeComponent
IF (WIN32)
  FIND_PACKAGE(cppzmq REQUIRED)
  SET(ZMQ_LIBRARIES cppzmq)
ELSEIF(APPLE)
  FIND_PACKAGE(PkgConfig REQUIRED)
  PKG_CHECK_MODULES(ZMQ REQUIRED libzmq)
  TARGET_LINK_DIRECTORIES(AffAction PRIVATE ${ZMQ_LIBRARY_DIRS})
ELSE()
  SET(ZMQ_LIBRARIES zmq)
ENDIF()
TARGET_LINK_LIBRARIES(AffAction PRIVATE ${ZMQ_LIBRARIES})

# Experimental respeaker c++ class
# LIBUSB_1_INCLUDE_DIRS, LIBUSB_1_LIBRARIES
# PORTAUDIO_INCLUDE_DIR, PORTAUDIO_LIBRARIES
IF (AFFACTION_WITH_RESPEAKER)
  IF (UNIX)
    FIND_PACKAGE(PkgConfig REQUIRED)
    PKG_CHECK_MODULES(libusb-1.0 REQUIRED libusb-1.0)
    SET(PORTAUDIO_INCLUDE_DIR $ENV{SIT}/External/portaudio/1.9/include)
    SET(PORTAUDIO_LIBRARIES $ENV{SIT}/External/portaudio/1.9/lib/focal64/libportaudio.so)
    SET(LIBUSB_1_INCLUDE_DIRS )
    SET(LIBUSB_1_LIBRARIES ${libusb-1.0_LIBRARIES})
  ELSEIF (WIN32)
    FIND_PACKAGE(libusb REQUIRED)
	SET(LIBUSB_1_LIBRARIES ${LIBUSB_LIBRARIES})
	SET(LIBUSB_1_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIRS})
	#SET(LIBUSB_1_LIBRARIES hid setupapi)
  ENDIF()

  TARGET_LINK_LIBRARIES(AffAction PRIVATE ${LIBUSB_1_LIBRARIES} ${PORTAUDIO_LIBRARIES})
  TARGET_INCLUDE_DIRECTORIES(AffAction PRIVATE ${LIBUSB_1_INCLUDE_DIRS} ${PORTAUDIO_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(AffAction PRIVATE -DAFFACTION_WITH_RESPEAKER)
ENDIF(AFFACTION_WITH_RESPEAKER)

# We add this option to force the linker to export the symbols. This
# solves the issue that it will not be exposed, since all initialization
# happens inside statically. Under Unix, we can use -Wl,--no-as-needed
IF (WIN32)
  TARGET_LINK_OPTIONS(AffAction PUBLIC /INCLUDE:AffActionExampleInfo)
ENDIF()

IF(AFFACTION_WITH_OPTIM)
  IF(TARGET RcsOptim)
    MESSAGE(STATUS "Compiling AffAction with optimization support")
    #LIST(APPEND ActionBase src/OptimizationSet.cpp)
    TARGET_COMPILE_DEFINITIONS(AffAction PUBLIC -DSMILEACTIONS_WITH_OPTIM)
    TARGET_LINK_LIBRARIES(AffAction PRIVATE RcsOptim)
  ELSE()
    MESSAGE(STATUS "Compiling AffAction without optimization support")
  ENDIF()

ENDIF()

# PTUComponent currently uses ROS
IF (AFFACTION_WITH_ROS)
  TARGET_LINK_LIBRARIES(AffAction PRIVATE ${roscpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
  TARGET_INCLUDE_DIRECTORIES(AffAction SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
  TARGET_COMPILE_DEFINITIONS(AffAction PUBLIC -DUSE_ROS)  
  TARGET_INCLUDE_DIRECTORIES(AffAction PUBLIC ${SMILE_WS_PATH}/devel/include)
  TARGET_LINK_DIRECTORIES(AffAction PUBLIC /opt/ros/noetic/lib)
  TARGET_LINK_LIBRARIES(AffAction PRIVATE actionlib)
ENDIF(AFFACTION_WITH_ROS)

################################################################################
# Executables
################################################################################
ADD_EXECUTABLE(ExampleRunnerAff examples/ExampleRunner.cpp)
IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  TARGET_LINK_LIBRARIES(ExampleRunnerAff -Wl,--no-as-needed RcsExamples AffAction -Wl,--as-needed)
ELSE()
  TARGET_LINK_LIBRARIES(ExampleRunnerAff RcsExamples AffAction)
ENDIF()

TARGET_COMPILE_DEFINITIONS(ExampleRunnerAff PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")

ADD_EXECUTABLE(TestAffordance examples/TestAffordance.cpp)
TARGET_LINK_LIBRARIES(TestAffordance AffAction)

# To fix issues with remote graphics, we call XInitThreads in TestLLMSim.
# That's why we need to link against X11 under Linux.
ADD_EXECUTABLE(TestLLMSim examples/TestLLMSim.cpp)
TARGET_LINK_LIBRARIES(TestLLMSim AffAction)
IF (UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(TestLLMSim X11)
ENDIF()

ADD_EXECUTABLE(TreeTest examples/TreeTest.cpp)
TARGET_LINK_LIBRARIES(TreeTest RcsCore)

ADD_EXECUTABLE(RunPTU examples/RunPTU.cpp)
IF (APPLE)
  FIND_PACKAGE(PkgConfig REQUIRED)
  PKG_CHECK_MODULES(ZMQ REQUIRED libzmq)
  TARGET_LINK_DIRECTORIES(RunPTU PRIVATE ${ZMQ_LIBRARY_DIRS})
ENDIF()
TARGET_LINK_LIBRARIES(RunPTU AffAction ${ZMQ_LIBRARIES})


IF (WIN32 AND AFFACTION_WITH_MQTT)
  ADD_EXECUTABLE(MqttTest examples/MqttTest.cpp)
  TARGET_INCLUDE_DIRECTORIES(MqttTest PUBLIC $ENV{SIT}/External/mosquitto/devel)
  TARGET_LINK_DIRECTORIES(MqttTest PUBLIC $ENV{SIT}/External/mosquitto/devel)
  TARGET_LINK_LIBRARIES(MqttTest PUBLIC RcsCore mosquitto mosquittopp)
#  ADD_CUSTOM_COMMAND(
#    TARGET MqttTest POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy
#    $ENV{SIT}/External/mosquitto/mosquitto.dll
#    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/mosquitto.dll)
ENDIF()

################################################################################
# Python module with pybind11
################################################################################
ADD_SUBDIRECTORY(python)

################################################################################
# Robotics components
################################################################################
ADD_SUBDIRECTORY(src/robo)

################################################################################
# Doxygen html documentation
################################################################################
ADD_SUBDIRECTORY (doc)

###############################################################################
# Install the libraries and the binaries
###############################################################################
INSTALL(TARGETS AffAction EXPORT AffActionExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/AffAction)

IF (TARGET AzureTracker)
  INSTALL(TARGETS AzureTracker EXPORT AffActionExport
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib/AffAction)
ENDIF()

INSTALL(TARGETS TestLLMSim TestAffordance ExampleRunnerAff
        RUNTIME DESTINATION bin 
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/AffAction)

###############################################################################
# Install the headers and the dependent dlls (for Windows)
###############################################################################
INSTALL(
 DIRECTORY src/
 DESTINATION include/AffAction
 FILES_MATCHING PATTERN "*.h"
)

IF (WIN32)
  FILE(GLOB DLL_FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/*.dll)
  INSTALL(FILES ${DLL_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
ENDIF(WIN32)

###############################################################################
# Mark for export
###############################################################################
#AFFACTION_ADD_EXPORT_LIBRARY(AzureTracker)

INCLUDE(CopyOrLinkConfig)
RCS_COPY_OR_LINK_CONFIG(${CMAKE_CURRENT_SOURCE_DIR}/config ${PROJECT_BINARY_DIR}/config)

################################################################################
# Target export script
################################################################################
INCLUDE(ExportAffAction)
