#
# author: Timothee Habra
# Sept 7 2015
#
# This file finds the libraries
# related to Robotran MBSysC
#
# ROBOTRAN_C_FOUND :  1 if all required files found (0 otherwise)
# SDL2_INCLUDE_PATH : include paths (for the header files) -> for compilation
# SDL2_LIBRARIES :    libraries -> for linkage
#


# possible paths for: 'libMBsysC_module.so'
set(TRIAL_PATHS_LIB_MBSYSC_MODULES
  "${ROBOTRAN_SOURCE_DIR}/build/mbs_module"
)

# possible paths for: 'libMBsysC_loadXML.so'
set(TRIAL_PATHS_LIB_MBSYSC_LOAD
  "${ROBOTRAN_SOURCE_DIR}/build/mbs_load_xml"
)

# possible paths for: 'libMBsysC_numerics.so'
set(TRIAL_PATHS_LIB_MBSYSC_UTILITIES
  "${ROBOTRAN_SOURCE_DIR}/build/mbs_utilities"
)


find_library(LIB_MBSYSC_MODULES libMBsysC_module.so ${TRIAL_PATHS_LIB_MBSYSC_MODULES})
find_library(LIB_MBSYSC_LOAD libMBsysC_loadXML.so ${TRIAL_PATHS_LIB_MBSYSC_LOAD})
find_library(LIB_MBSYSC_UTILITIES libMBsysC_utilities.so ${TRIAL_PATHS_LIB_MBSYSC_UTILITIES})

#set(SDL2_LIBRARIES ${SDL2_LIBRARIES_SDL2})


# flag put to 1 if all required files are found
if (LIB_MBSYSC_MODULES AND LIB_MBSYSC_LOAD AND LIB_MBSYSC_UTILITIES)
  set(ROBOTRAN_C_FOUND 1)
else ( )
  set(ROBOTRAN_C_FOUND 0)
endif ( )

# outputs
mark_as_advanced(
  ROBOTRAN_C_FOUND
  #SDL2_INCLUDE_PATH
  #SDL2_LIBRARIES 
)
