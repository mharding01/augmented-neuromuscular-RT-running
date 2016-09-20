#
# author: Nicolas Van der Noot
# September 2015
#
# This file finds the static libraries
# related to the project symbolic files
#
# PROJ_SYMB_FOUND :  1 if all required files found (0 otherwise)
# LIB_PROJ_SYMB   :  static library -> for linkage
#


# possible paths for: 'libMBproj_symbolicR.a'
set(TRIAL_PATHS_LIB_PROJ_SYMB
    "${PROJECT_SOURCE_DIR}/../${SYMBOLIC_PATH}/build"
    "${PROJECT_SOURCE_DIR}/../symbolicR/build"
)

# first trial
set (LIB_PROJ_SYMB ${PROJECT_SOURCE_DIR}/../${SYMBOLIC_PATH}/build/libMBproj_symbolicR.a)

# in case the first trial is not successful
if (NOT EXISTS ${LIB_PROJ_SYMB})
	set (LIB_PROJ_SYMB) # clear variable
	find_library (LIB_PROJ_SYMB libMBproj_symbolicR.a ${TRIAL_PATHS_LIB_PROJ_SYMB})
endif ( )


# flag put to 1 if all required files are found
if ( LIB_PROJ_SYMB )
    set(PROJ_SYMB_FOUND 1)
else ( )
    set(PROJ_SYMB_FOUND 0)
endif ( )

# outputs
mark_as_advanced(
    PROJ_SYMB_FOUND
)
