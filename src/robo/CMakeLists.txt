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



##############################################################################
# Kinova Kortex Gen3 driver
##############################################################################
ADD_EXECUTABLE(KortexDriver KortexDriver.cpp)
TARGET_LINK_LIBRARIES(KortexDriver RcsCore ${ZMQ_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../external") # for json

IF(UNIX)
  TARGET_COMPILE_DEFINITIONS(KortexDriver PUBLIC -D_OS_UNIX)
  TARGET_COMPILE_OPTIONS(KortexDriver PRIVATE "-Wall")
  TARGET_COMPILE_OPTIONS(KortexDriver PRIVATE "-Wno-reorder")
ELSEIF(WIN32)
  TARGET_COMPILE_DEFINITIONS(KortexDriver PUBLIC -D_OS_WINDOWS -D_USE_MATH_DEFINES)
ELSEIF(APPLE)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${ZMQ_INCLUDE_DIRS})
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
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_DIR}/include)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_DIR}/include/client)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_DIR}/include/common)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_DIR}/include/messages)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_DIR}/include/client_stubs)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_BASE_DIR}/api_cpp/examples)
  TARGET_INCLUDE_DIRECTORIES(KortexDriver PUBLIC ${KORTEX_BASE_DIR}/api_cpp/examples/thirdParty/cxxopts)

  TARGET_COMPILE_DEFINITIONS(KortexDriver PUBLIC -DAFFACTION_WITH_KINOVA_GEN3)

ENDIF(AFFACTION_WITH_KINOVA_GEN3)


##############################################################################
# Socket CAN Schunk PW70 driver
##############################################################################
ADD_EXECUTABLE(PTUDriver PTUDriver.cpp)
IF (APPLE)
  FIND_PACKAGE(PkgConfig REQUIRED)
  PKG_CHECK_MODULES(ZMQ REQUIRED libzmq)
  TARGET_LINK_DIRECTORIES(PTUDriver PRIVATE ${ZMQ_LIBRARY_DIRS})
ENDIF()
TARGET_LINK_LIBRARIES(PTUDriver AffAction ${ZMQ_LIBRARIES})