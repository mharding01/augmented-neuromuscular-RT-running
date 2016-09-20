# Copyright: (C)
# Authors: Timothee Habra & Nicolas Van der Noot
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

project(MBsysC)

## --------------- ##

# project configuration
set( CMAKE_C_FLAGS_RELEASE   "-O3" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

# link to useful CMake functions
set(CMAKE_AUX "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_aux")
set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../conf/")

add_subdirectory (${CMAKE_AUX}/listing/ ${CMAKE_AUX_BIN}/listing/)
add_subdirectory (${CMAKE_AUX}/flags/ ${CMAKE_AUX_BIN}/flags/)
add_subdirectory (${CMAKE_AUX}/libraries/ ${CMAKE_AUX_BIN}/libraries/)

flags_check()
release_debug()
definitions()

# flags
if (UNIX)
    set(CMAKE_C_FLAGS "-fPIC")
    set(CMAKE_EXE_LINKER_FLAGS "-fPIC")
endif (UNIX)

# add compilation CMake modules
add_subdirectory ( mbs_numerics/  ${CMAKE_CURRENT_BINARY_DIR}/mbs_numerics/  )
add_subdirectory ( mbs_struct/    ${CMAKE_CURRENT_BINARY_DIR}/mbs_struct/    )
add_subdirectory ( mbs_utilities/ ${CMAKE_CURRENT_BINARY_DIR}/mbs_utilities/ )
add_subdirectory ( mbs_load_xml/  ${CMAKE_CURRENT_BINARY_DIR}/mbs_load_xml/  )
add_subdirectory ( mbs_module/    ${CMAKE_CURRENT_BINARY_DIR}/mbs_module/    )
add_subdirectory ( mbs_realtime/  ${CMAKE_CURRENT_BINARY_DIR}/mbs_realtime/  )

# add parent scope if needed
get_directory_property(hasParent PARENT_DIRECTORY)

if(hasParent)
    set( LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES} PARENT_SCOPE )
    set( GSL_LIBRARIES     ${GSL_LIBRARIES}     PARENT_SCOPE )
    set( SDL2_LIBRARIES    ${SDL2_LIBRARIES}    PARENT_SCOPE )
    set( SDL2TTF_LIBRARIES ${SDL2TTF_LIBRARIES} PARENT_SCOPE )
    set( JNI_LIBRARIES     ${JNI_LIBRARIES}     PARENT_SCOPE )
    set( J3D_PATH          ${J3D_PATH}          PARENT_SCOPE )
endif()
