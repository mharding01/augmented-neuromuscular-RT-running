# Copyright: (C)
# Authors: Timothee Habra
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

project(MBsysC_loadXML)

# project configuration
set( CMAKE_C_FLAGS_RELEASE   "-O3" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

# release of debug
release_debug()

# Libxml2
xml_lib(mbs_load_xml)
set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES} PARENT_SCOPE)

# list source files to compile
init_src()
increment_src( ${PROJECT_SOURCE_DIR} )

# list include directories (to find headers)
init_include()
set(INCLUDE_DIR ${INCLUDE_DIR} ${PROJECT_SOURCE_DIR} PARENT_SCOPE)

if(FLAG_SHARED_LIB)
    add_library(MBsysC_loadXML SHARED ${SOURCE_FILES} ${INCLUDE_DIR})
    target_link_libraries(MBsysC_loadXML ${LIBXML2_LIBRARIES})
else( )
    add_library(MBsysC_loadXML STATIC ${SOURCE_FILES} ${INCLUDE_DIR})
endif( )

# include directories
include_directories("./")
include_directories("../mbs_utilities")
include_directories("../mbs_struct")
