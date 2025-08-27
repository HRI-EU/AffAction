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

OPTION(AFFACTION_WITH_KINOVA_GEN3 "Enable Kinova Gen3 Kortex driver" OFF)
OPTION(AFFACTION_WITH_LIBFRANKA "Enable libfranka driver" OFF)



##############################################################################
# Kinova Kortex Gen3 driver
##############################################################################
ADD_EXECUTABLE(KortexDriver KortexDriver.cpp)
TARGET_LINK_LIBRARIES(KortexDriver RcsCore ${ZMQ_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../external") # for json

IF(APPLE)
  FIND_PACKAGE(PkgConfig REQUIRED)
  PKG_CHECK_MODULES(ZMQ REQUIRED libzmq)
  TARGET_LINK_DIRECTORIES(KortexDriver PRIVATE ${ZMQ_LIBRARY_DIRS})
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${ZMQ_INCLUDE_DIRS})
ELSEIF(UNIX)
  TARGET_COMPILE_DEFINITIONS(KortexDriver PRIVATE -D_OS_UNIX)
  TARGET_COMPILE_OPTIONS(KortexDriver PRIVATE "-Wall")
  TARGET_COMPILE_OPTIONS(KortexDriver PRIVATE "-Wno-reorder")
ELSEIF(WIN32)
  TARGET_COMPILE_DEFINITIONS(KortexDriver PRIVATE -D_OS_WINDOWS -D_USE_MATH_DEFINES)
ENDIF()

IF (AFFACTION_WITH_KINOVA_GEN3)
  
  SET(KORTEX_BASE_DIR "$ENV{SIT}/External/Kinova-kortex2_Gen3_G3L")
  IF (UNIX)
    SET(KORTEX_DIR "${KORTEX_BASE_DIR}/api_cpp/examples/kortex_api")
  ELSE()
    SET(KORTEX_DIR "${KORTEX_BASE_DIR}/api_cpp/examples/kortex_api/MD")
  ENDIF()

  IF(UNIX)
    TARGET_LINK_LIBRARIES(KortexDriver ${KORTEX_DIR}/lib/release/libKortexApiCpp.a)
  ELSEIF(WIN32)
    TARGET_LINK_LIBRARIES(KortexDriver ${KORTEX_DIR}/lib/release/KortexApiCpp.lib)
  ENDIF()

  # Add Include Directories
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_DIR}/include)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_DIR}/include/client)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_DIR}/include/common)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_DIR}/include/messages)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_DIR}/include/client_stubs)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_BASE_DIR}/api_cpp/examples)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PRIVATE ${KORTEX_BASE_DIR}/api_cpp/examples/thirdParty/cxxopts)

  TARGET_COMPILE_DEFINITIONS(KortexDriver PRIVATE -DAFFACTION_WITH_KINOVA_GEN3)

ENDIF(AFFACTION_WITH_KINOVA_GEN3)


##############################################################################
# Socket CAN Schunk PW70 driver
##############################################################################
ADD_EXECUTABLE(PTUDriver PTUDriver.cpp ../PW70CANInterface.cpp)
TARGET_INCLUDE_DIRECTORIES(PTUDriver PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..") # for PW70
TARGET_INCLUDE_DIRECTORIES(PTUDriver PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../external") # for json

IF (APPLE)
  TARGET_LINK_DIRECTORIES(PTUDriver PRIVATE ${ZMQ_LIBRARY_DIRS})
  TARGET_INCLUDE_DIRECTORIES(PTUDriver PRIVATE ${ZMQ_INCLUDE_DIRS})
ENDIF()

TARGET_LINK_LIBRARIES(PTUDriver RcsCore ${ZMQ_LIBRARIES})



##############################################################################
# libfranka driver. Here's the cmake vars for pinocchio and Franka:
# PINOCCHIO_INCLUDE_DIRS PINOCCHIO_LIBRARIES
# Franka_INCLUDE_DIRS Franka_LIBRARIES
##############################################################################
IF (AFFACTION_WITH_LIBFRANKA AND UNIX AND NOT APPLE)
  #SET(CMAKE_CXX_STANDARD 17)

  ADD_EXECUTABLE(FrankaDriver FrankaDriver.cpp)
  SET(PINOCCHIO_DIR "/hri/storage/user/mgienger/Repos/pinocchio/install")
  SET(LIBFRANKA_DIR "/hri/storage/user/mgienger/Repos/libfranka/install")
  FIND_PACKAGE(pinocchio REQUIRED HINTS "${PINOCCHIO_DIR}/lib/cmake/pinocchio")
  FIND_PACKAGE(Franka REQUIRED HINTS "${LIBFRANKA_DIR}/lib/cmake/Franka")
  FIND_PACKAGE(Eigen3 REQUIRED)

  TARGET_INCLUDE_DIRECTORIES(FrankaDriver PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../external") # for json
  TARGET_INCLUDE_DIRECTORIES(FrankaDriver PRIVATE "${EIGEN3_INCLUDE_DIR}")
  TARGET_INCLUDE_DIRECTORIES(FrankaDriver PRIVATE "${PINOCCHIO_INCLUDE_DIRS}")
  TARGET_INCLUDE_DIRECTORIES(FrankaDriver PRIVATE "${Franka_INCLUDE_DIRS}")
  TARGET_LINK_LIBRARIES(FrankaDriver RcsCore ${ZMQ_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${Franka_LIBRARIES})

  IF(APPLE)
    FIND_PACKAGE(PkgConfig REQUIRED)
    PKG_CHECK_MODULES(ZMQ REQUIRED libzmq)
    TARGET_LINK_DIRECTORIES(FrankaDriver PRIVATE ${ZMQ_LIBRARY_DIRS})
    TARGET_INCLUDE_DIRECTORIES(FrankaDriver PRIVATE ${ZMQ_INCLUDE_DIRS})
  ELSEIF(UNIX)
    TARGET_COMPILE_DEFINITIONS(FrankaDriver PRIVATE -D_OS_UNIX)
    TARGET_COMPILE_OPTIONS(FrankaDriver PRIVATE "-Wall")
    TARGET_COMPILE_OPTIONS(FrankaDriver PRIVATE "-Wno-reorder")
  ELSEIF(WIN32)
    TARGET_COMPILE_DEFINITIONS(FrankaDriver PRIVATE -D_OS_WINDOWS -D_USE_MATH_DEFINES)
  ENDIF()
ENDIF(AFFACTION_WITH_LIBFRANKA AND UNIX AND NOT APPLE)
