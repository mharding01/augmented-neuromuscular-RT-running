#
# author: Nicolas Van der Noot
#
# Visualization project for Robotran, with OpenGL
#

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       PROJECT MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


# CMake minimum version
cmake_minimum_required(VERSION 2.8.12)

# project name
project (OpenGLRobotran)


# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

# for Unix: display all the warnings, except the ones related to /* -- */
if (UNIX)
  set(CMAKE_C_FLAGS "-g -Wall -Wno-comment")
endif (UNIX)

# mbsysc project path
#set(MBSYSC_PATH /data/nvander/Documents/Robotran/Projects/robotran/mbsysc)
set(MBSYSC_PATH /home/nico/Documents/Robotran/Projects/robotran/mbsysc)

# additional CMake functions
set(CMAKE_AUX ${MBSYSC_PATH}/MBsysC/cmake_aux)
set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)

add_subdirectory ( ${CMAKE_AUX}/listing/   ${CMAKE_AUX_BIN}/listing/   )
add_subdirectory ( ${CMAKE_AUX}/libraries/ ${CMAKE_AUX_BIN}/libraries/ )

# Windows M_PI definitions
if (WIN32)
    add_definitions(-D _USE_MATH_DEFINES)
endif (WIN32)

# project own definition
add_definitions(-D SEPARATE_OPEN_GL)



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           USER ABSOLUTE PATHS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# configure header files to pass some of the CMake settings to the source code
set (ROBOTRAN_SOURCE_DIR "${MBSYSC_PATH}/MBsysC")

configure_file (
  "${MBSYSC_PATH}/MBsysC/conf/cmake_config.h.in"
  "${PROJECT_BINARY_DIR}/conf/cmake_config.h"
)

set(OPEN_GL_FILES_PATH ${MBSYSC_PATH}/MBsysC/mbs_common/mbs_realtime/open_gl/)

configure_file (
    "${MBSYSC_PATH}/MBsysC/mbs_common/mbs_realtime/open_gl/conf/cmake_open_gl_config.h.in"
    "${PROJECT_BINARY_DIR}/conf/cmake_open_gl_config.h"
)

# 'cmake_config.h.in' and 'cmake_open_gl_config.h.in' is in the 'conf' folder
include_directories (${PROJECT_BINARY_DIR}/conf)



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           EXTERNAL LIBRARIES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

set(CMAKE_MODULE_PATH ${MBSYSC_PATH}/MBsysC/conf)

# Libxml2
xml_lib(main)

# OpenGL
open_gl_lib(main)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           LIST FILES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

 
# list source files to compile
init_src()

increment_src("${PROJECT_SOURCE_DIR}/src")
increment_src("${MBSYSC_PATH}/MBsysC/mbs_common/mbs_realtime/open_gl")


# list include directories (to find headers)
init_include()

increment_include("${PROJECT_SOURCE_DIR}/src")
increment_include("${MBSYSC_PATH}/MBsysC/mbs_common/mbs_realtime/open_gl")



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         CONFIGURE EXECUTABLE
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# include these directories
include_directories (${INCLUDE_DIR})

# name of the executable
set (Executable Exec)

# generate the executable
add_executable (${Executable} ${SOURCE_FILES})

# link library: Libxml2
target_link_libraries ( ${Executable} ${LIBXML2_LIBRARIES} )

# link library: OpenGL
target_link_libraries ( ${Executable} ${OPEN_GL_LIBS})

# math external library (for Unix): linkage
if (UNIX)
    target_link_libraries (${Executable} m)
endif (UNIX)
