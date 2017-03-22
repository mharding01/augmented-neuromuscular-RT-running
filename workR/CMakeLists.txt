#
# Authors: Nicolas Van der Noot, Nicolas Docquier, Timothee Habra
# September 2015
#
# CMake for the COMAN project 
#


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                        PROJECT SPECIFIC OPTIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# possibilities: init_feet ; long_feet ; flex_feet ; short_feet ; short_feet_ball ; spring_toe_feet ; spring_toe_short_feet
set( COMAN_VERSION "spring_toe_short_feet" )

# 0: normal run ; 1: evaluation run ; 2: test opti ; 3: BioRob opti ; 4: MPI opti
set( MAIN_RUN 0 )

# speed reference
set (CMAKE_VAR_1 1.5)

# initial forward speed
set (CMAKE_VAR_2 1.5)

# not used
set (CMAKE_VAR_3 0.0)
set (CMAKE_VAR_4 0.0)
set (CMAKE_VAR_5 0.0)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       PROJECT MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# project name
project (projectRobotran)

# project configuration
set( CMAKE_C_FLAGS_RELEASE   "-O3" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

# use GLPK library
option (FLAG_GLPK 
        "GLPK" ON)

# Variable for storing the path to Robotran common files (should be adapted depending on the location of those source)
set(TRIAL_PATHS_MBSYSC
	#"C:/Users/Bruno/Documents/Robotran/mbsysc/MBsysC" # Bruno Windows
	"C:/Users/Bruno/Documents/Robotran/mbsysc/MBsysC" # Bruno Windows last release
	"${PROJECT_SOURCE_DIR}/../mbsysCopy" # copy
)
find_path(ROBOTRAN_SOURCE_DIR mbs_common ${TRIAL_PATHS_MBSYSC})

# force mbsysCopy for optimization
if ( MAIN_RUN GREATER 1 )
	set (ROBOTRAN_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mbsysCopy")
endif ( )

# message to display the project name and the CoMan version
message(STATUS "Processing ${PROJECT_NAME}")
message(STATUS "CoMan version: ${COMAN_VERSION}")
message(STATUS "MBsysC dir: ${ROBOTRAN_SOURCE_DIR}")

# for Unix: display all the warnings, except the ones related to /* -- */ and for the unused variables
if (UNIX)
	set( CMAKE_C_FLAGS "-g -Wall -Wno-unused-but-set-variable -Wno-unused-variable -Wno-comment" )
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         Additional CMakelists.txt
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

## --- WIN32 or WIN64 DETECTION --- ##

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )   # 64bit Windows
	SET(WIN_LIB_DIRECTORY win64_include_lib)
else( )  # 32bit Windows
	SET(WIN_LIB_DIRECTORY win32_include_lib)
endif( )

# additional CMake functions
set(CMAKE_AUX ${ROBOTRAN_SOURCE_DIR}/cmake_aux)
set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)

add_subdirectory ( ${CMAKE_AUX}/flags/     ${CMAKE_AUX_BIN}/flags/     )
add_subdirectory ( ${CMAKE_AUX}/listing/   ${CMAKE_AUX_BIN}/listing/   )
add_subdirectory ( ${CMAKE_AUX}/libraries/ ${CMAKE_AUX_BIN}/libraries/ )
add_subdirectory ( ${CMAKE_AUX}/make_opt/  ${CMAKE_AUX_BIN}/make_opt/  )

# find libraries paths
set(CMAKE_MODULE_PATH ${ROBOTRAN_SOURCE_DIR}/conf ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_conf)

# libraries check
set ( SYMBOLIC_PATH symbolicR/${COMAN_VERSION} )
find_package( LibSymbolicR )
find_package( LibRobotranC )

# real-time when not performing an optimization
if ( MAIN_RUN LESS 2 )

	# real-time
	set_flag( FLAG_REAL_TIME ON )

	# separate symbolic
	if ( ${PROJ_SYMB_FOUND} )
		set_flag( FLAG_SEPARATE_SYMBOLIC ON )
		message(" Precompiled symbolic library found, FLAG_SEPARATE_SYMBOLIC set to ON.")
	else ( )
		set_flag( FLAG_SEPARATE_SYMBOLIC OFF )
		message(" No precompiled symbolic library found, FLAG_SEPARATE_SYMBOLIC set to OFF.")
	endif ( )

	# separate MBsysC
	if ( ${ROBOTRAN_C_FOUND} )
		set_flag( FLAG_SHARED_LIB        ON )
		set_flag( FLAG_SEPARATE_BUILD    ON )
		message(" Precompiled MBsysC libraries found, FLAG_SEPARATE_BUILD set to ON.")
	else ( )
		set_flag( FLAG_SHARED_LIB        OFF )
		set_flag( FLAG_SEPARATE_BUILD    OFF )
		message(" No precompiled MBsysC libraries found, FLAG_SEPARATE_BUILD set to OFF.")
	endif ( )
endif ( )

# CMake functions
flags_check()
release_debug()
definitions()
make_options()

# MBsysC files to compile
if (NOT FLAG_SEPARATE_BUILD)
	add_subdirectory( ${ROBOTRAN_SOURCE_DIR}/mbs_common ${CMAKE_CURRENT_BINARY_DIR}/mbs_common )
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                             EXTRA LIBRARIES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# Biorob Cluster
if ( MAIN_RUN EQUAL 3 )
	find_package( BiorobOpti REQUIRED )
	if (BIOROB_OPTI_FOUND)
		include_directories (${BIOROB_OPTI_INCLUDE_PATH})
		message( "\n  >> main: BiorobOpti found")
	else ( )
		message( "\n  >> main: BiorobOpti not found, modify ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_conf/FindBiorobOpti.cmake")
	endif ( )

	message("      -> INC : ${BIOROB_OPTI_INCLUDE_PATH}")
	message("      -> LIB : ${BIOROB_OPTI_LIBRARIES}\n")
endif ( )

# MPI library
if (MAIN_RUN EQUAL 4)
	find_package(MPI REQUIRED)
	if (MPI_FOUND)
		include_directories (${MPI_INCLUDE_PATH})
		message( "\n  >> main: MPI found")
	else ( )
		message( "\n  >> main: MPI not found")
	endif ( )

	message("      -> INC : ${MPI_INCLUDE_PATH}")
	message("      -> LIB : ${MPI_LIBRARIES}\n")
endif ( )

# GLPK library
if (FLAG_GLPK)
	add_definitions( -DGLPK_LIB )
	find_package( GLPK REQUIRED )
	if (GLPK_FOUND)
		include_directories (${GLPK_INCLUDE_PATH})
		message( "\n  >> main: GLPK found")
	else ( )
		message( "\n  >> main: GLPK not found, modify ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_conf/FindGLPK.cmake")
	endif ( )

	message("      -> INC : ${GLPK_INCLUDE_PATH}")
	message("      -> LIB : ${GLPK_LIBRARIES}\n")
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         FLAGS RELATED FUNCTIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


# debug or release
if( CMAKE_BUILD_TYPE MATCHES "Debug" )
	add_definitions( -DDEBUG_VERSION )
	message(STATUS "Debug version !")
endif( )

# evaluation run
if ( MAIN_RUN EQUAL 1 )
	add_definitions( -D EVAL_RUN )
	message(STATUS "Evaluation run !")
endif (  )

# optimization run
if ( MAIN_RUN GREATER 1 )
	add_definitions( -D OPTI_RUN )
	message(STATUS "Running optimization !")
endif (  )

# BioRob optimization
if ( MAIN_RUN EQUAL 3 )
	add_definitions( -D OPTI_BIOROB )
	message(STATUS "BioRob cluster !")
endif (  )

# MPI optimization
if ( MAIN_RUN EQUAL 4 )
	add_definitions( -D OPTI_MPI )
	message(STATUS "MPI computation !")
endif (  )


# Windows M_PI definitions
if (WIN32)
	add_definitions(-D_USE_MATH_DEFINES)
endif (WIN32)

# Robotran simulation flag
add_definitions ( -DROBOTRAN_SIMU )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           USER ABSOLUTE PATHS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# upper-case letters of COMAN version
string(TOUPPER ${COMAN_VERSION} COMAN_VERSION_UPPER)

# configure a header file to pass some of the CMake settings to the source code
configure_file (
	"${ROBOTRAN_SOURCE_DIR}/conf/cmake_config.h.in"
	"${PROJECT_BINARY_DIR}/conf/cmake_config.h"
)

configure_file (
	"${PROJECT_SOURCE_DIR}/src/conf/cmake_define.h.in"
	"${PROJECT_BINARY_DIR}/conf/cmake_define.h"
)

include_directories (${PROJECT_BINARY_DIR}/conf)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           LIST FILES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 
# list source files to compile
init_src()

increment_src( ${PROJECT_SOURCE_DIR}/src )
increment_src( ${PROJECT_SOURCE_DIR}/../userfctR )
increment_src( ${PROJECT_SOURCE_DIR}/../userFiles )
increment_src( ${PROJECT_SOURCE_DIR}/../dataR/index/${COMAN_VERSION} )

if (NOT FLAG_SEPARATE_SYMBOLIC)
	increment_src( ${PROJECT_SOURCE_DIR}/../symbolicR/${COMAN_VERSION} )
endif ( )

increment_void_symbolic( symbolicR/${COMAN_VERSION} )


# list include directories (to find headers)
init_include()

increment_include( ${PROJECT_SOURCE_DIR}/src )
increment_include( ${PROJECT_SOURCE_DIR}/../userfctR )
increment_include( ${PROJECT_SOURCE_DIR}/../userFiles )
increment_include( ${PROJECT_SOURCE_DIR}/../symbolicR/${COMAN_VERSION} )

increment_include( ${ROBOTRAN_SOURCE_DIR}/mbs_common )

# SDL.h header
if (FLAG_PLOT)
	sdl_header_lib(project)
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           EXTERNAL LIBRARIES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

## ---- WINDOWS DLL FILES ---- ##

# copy all the dll (except 'jvm.dll') used for Windows
# these dll files are copied in the Executable directory (Debug or Release)
if (WIN32)
	file(COPY ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Debug)
	file(COPY ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Release)
	if (FLAG_GLPK)
		file(COPY ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_win/glpk/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Debug)
		file(COPY ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_win/glpk/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Release)
	endif ( )
endif ( )

## ---- PYTHON SCRIPTS FOR OPTI ---- ##

# BioRob
if (MAIN_RUN EQUAL 3)
	file(COPY ${PROJECT_SOURCE_DIR}/../opti/python/xml_gen.py DESTINATION ${CMAKE_BINARY_DIR})
	file(COPY ${PROJECT_SOURCE_DIR}/../opti/python/biorob_res_gen.py DESTINATION ${CMAKE_BINARY_DIR})
	file(COPY ${PROJECT_SOURCE_DIR}/../opti/python/nb_opti_params.py DESTINATION ${CMAKE_BINARY_DIR})
endif ( )

# MPI
if (MAIN_RUN EQUAL 4)
	file(COPY ${PROJECT_SOURCE_DIR}/../opti/python/mpi_res_gen.py DESTINATION ${CMAKE_BINARY_DIR})
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         EXECUTABLE COMPILATION
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# include these directories
include_directories (${INCLUDE_DIR})

# name of the executable
if ( MAIN_RUN GREATER 2 ) # opti cluster
	set (Executable dispatcher_opti)
else ( )
	set (Executable exe_${PROJECT_NAME})
endif ( )

# generate the executable
add_executable ( ${Executable} ${SOURCE_FILES} )



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         EXECUTABLE LINKING
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


if (FLAG_SEPARATE_BUILD) # find MBSysC dynamic libraries

	find_package( LibRobotranC REQUIRED )
	target_link_libraries( ${Executable} ${LIB_MBSYSC_MODULES} ${LIB_MBSYSC_LOAD} ${LIB_MBSYSC_UTILITIES} )

else ( ) 

	target_link_libraries( ${Executable} MBsysC_loadXML MBsysC_module )

	if (NOT FLAG_SHARED_LIB)  

		# find MBSysC static libraries
		target_link_libraries( ${Executable} MBsysC_struct MBsysC_numerics MBsysC_realtime MBsysC_utilities )
		
		#Libxml2 and GSL external libraries
		target_link_libraries ( ${Executable} ${LIBXML2_LIBRARIES} ${GSL_LIBRARIES} )
		
		# SDL external library
		if (FLAG_PLOT)
			target_link_libraries ( ${Executable} ${SDL2_LIBRARIES} ${SDL2TTF_LIBRARIES} )
		endif ( )
		
		# Java external library
		if (FLAG_VISU)
			target_link_libraries ( ${Executable} ${JNI_LIBRARIES} )
		endif ( )
		
	endif ( )

endif ( )

# separate symbolic files compilation
if ( FLAG_SEPARATE_SYMBOLIC )
	set ( SYMBOLIC_PATH symbolicR/${COMAN_VERSION} )
	find_package( LibSymbolicR REQUIRED )
	target_link_libraries( ${Executable} ${LIB_PROJ_SYMB} )
endif ( )

# math external library (for Unix)
if (UNIX)
	target_link_libraries ( ${Executable} m )
endif ( )

# Biorob Cluster
if ( MAIN_RUN EQUAL 3 )
	target_link_libraries (${Executable} ${BIOROB_OPTI_LIBRARIES})
endif ( )

# MPI library
if ( MAIN_RUN EQUAL 4 )
	target_link_libraries (${Executable} ${MPI_LIBRARIES})
endif ( )

# GLPK library
if (FLAG_GLPK)
	target_link_libraries (${Executable} ${GLPK_LIBRARIES})
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         ORGANISE FILES FOR VS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 
# organize the files in visual studio folders, according to the organization in the file systems
# from http://cmake.3232098.n2.nabble.com/Keep-folder-structure-in-Visual-Studio-Project-generated-with-CMake-td7586044.html
macro(GroupSources curdir)
   file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)
   foreach(child ${children})
          if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
                  GroupSources(${curdir}/${child})
          else()
            string(REPLACE "/" "\\" groupname ${curdir})
			string(REPLACE ".." "coman_bruno" groupname ${groupname})
            source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
          endif()
   endforeach()
endmacro()
   
# Execute the macro
GroupSources(..)
