# Copyright: (C)
# Authors: Timothee Habra
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

project(MBsysC_utilities)

# project configuration
set( CMAKE_C_FLAGS_RELEASE   "-O3" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

# release of debug
release_debug()

# list source files to compile
init_src()
increment_src( ${PROJECT_SOURCE_DIR} )

# list include directories (to find headers)
init_include()
set(INCLUDE_DIR ${INCLUDE_DIR} ${PROJECT_SOURCE_DIR} PARENT_SCOPE)
set(INCLUDE_DIR ${INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/auto_output PARENT_SCOPE)

if(FLAG_SHARED_LIB)
    add_library(MBsysC_utilities SHARED ${SOURCE_FILES} ${INCLUDE_DIR})
    target_link_libraries(MBsysC_struct)
else( )
    add_library(MBsysC_utilities STATIC ${SOURCE_FILES} ${INCLUDE_DIR})
endif( )

# include directories
include_directories("./")
include_directories("./auto_output")
include_directories(../mbs_struct)
