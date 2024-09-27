# FindPCAN-Basic.cmake
# CMake module to find PCAN-Basic library

# Variables defined by this script:
#   PCAN_BASIC_FOUND         - True if found
#   PCAN_BASIC_INCLUDE_DIRS  - Include directories for PCAN-Basic
#   PCAN_BASIC_LIBRARIES     - Libraries to link against
#   PCAN_BASIC_DLLS          - DLLs required at runtime
#   PCAN-Basic::PCAN-Basic   - Imported target

# User may specify:
#   PCAN_BASIC_ROOT_DIR      - Root directory of PCAN-Basic installation

# How to use

# Old-school:
#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
#find_package(PCAN-Basic REQUIRED)
#if(PCAN-Basic_FOUND)
#    target_include_directories(your_target PRIVATE ${PCAN_BASIC_INCLUDE_DIRS})
#    target_link_libraries(your_target PRIVATE ${PCAN_BASIC_LIBRARIES})
#endif()

# Imported target:
#find_package(PCAN-Basic REQUIRED)
#if(TARGET PCAN-Basic::PCAN-Basic)
#    target_link_libraries(your_target PRIVATE PCAN-Basic::PCAN-Basic)
#endif()




# Locate the root directory
find_path(PCAN_BASIC_ROOT_DIR
    NAMES Include/PCANBasic.h
    HINTS ENV PCAN_BASIC_ROOT_DIR
    PATHS
        "[HKEY_CURRENT_USER\\Software\\PCAN-Basic;InstallDir]"
        "[HKEY_LOCAL_MACHINE\\Software\\PCAN-Basic;InstallDir]"
		"$ENV{USERPROFILE}/Documents/Software/Repos/PCAN-Basic"
        "C:/PCAN-Basic"
        "C:/Program Files/PCAN-Basic"
        "C:/Program Files (x86)/PCAN-Basic"
    DOC "Root directory of the PCAN-Basic library"
)

# Ensure the root directory was found
if(NOT PCAN_BASIC_ROOT_DIR)
    message(STATUS "Could not find PCAN-Basic root directory.")
    set(PCAN_BASIC_FOUND FALSE)
    return()
endif()

# Locate the include directory
set(PCAN_BASIC_INCLUDE_DIR "${PCAN_BASIC_ROOT_DIR}/Include")
if(NOT EXISTS "${PCAN_BASIC_INCLUDE_DIR}/PCANBasic.h")
    message(STATUS "Could not find PCANBasic.h in ${PCAN_BASIC_INCLUDE_DIR}")
    set(PCAN_BASIC_FOUND FALSE)
    return()
endif()

# Determine the architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PCAN_ARCH_DIR "x64")
else()
    set(PCAN_ARCH_DIR "x86")
endif()

# Locate the library directory
set(PCAN_BASIC_LIBRARY_DIR "${PCAN_BASIC_ROOT_DIR}/${PCAN_ARCH_DIR}/VC_LIB")
find_library(PCAN_BASIC_LIBRARY
    NAMES PCANBasic
    HINTS "${PCAN_BASIC_LIBRARY_DIR}"
)

if(NOT PCAN_BASIC_LIBRARY)
    message(STATUS "Could not find PCANBasic library in ${PCAN_BASIC_LIBRARY_DIR}")
    set(PCAN_BASIC_FOUND FALSE)
    return()
endif()

# Locate the DLLs (optional)
set(PCAN_BASIC_DLL_DIR "${PCAN_BASIC_ROOT_DIR}/${PCAN_ARCH_DIR}")
file(GLOB PCAN_BASIC_DLLS "${PCAN_BASIC_DLL_DIR}/PCANBasic.dll")

# Set the include directories and libraries
set(PCAN_BASIC_INCLUDE_DIRS "${PCAN_BASIC_INCLUDE_DIR}")
set(PCAN_BASIC_LIBRARIES "${PCAN_BASIC_LIBRARY}")

# Set the FOUND variable
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCAN-Basic REQUIRED_VARS PCAN_BASIC_INCLUDE_DIRS PCAN_BASIC_LIBRARIES)

# Create an imported target
if(PCAN-Basic_FOUND AND NOT TARGET PCAN-Basic::PCAN-Basic)
    add_library(PCAN-Basic::PCAN-Basic UNKNOWN IMPORTED)
    set_target_properties(PCAN-Basic::PCAN-Basic PROPERTIES
        IMPORTED_LOCATION "${PCAN_BASIC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PCAN_BASIC_INCLUDE_DIRS}"
    )
endif()
