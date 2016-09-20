#
# author: Nicolas Van der Noot
# December 1 2015
#
# This file finds the include folders (header files) and the libraries
# related to the GLPK library to solve large-scale linear programming problems
#
# GLPK_FOUND :        1 if all required files found (0 otherwise)
# GLPK_INCLUDE_PATH : include paths (for the header files) -> for compilation
# GLPK_LIBRARIES :    libraries -> for linkage
#

if (UNIX)

if (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'glpk.h'
set (TRIAL_PATHS_INC_GLPK
  /usr/local/include
)

# possible paths for: 'libglpk.so'
set (TRIAL_PATHS_LIB_GLPK
  /usr/local/lib
)

find_path (GLPK_INCLUDE_PATH glpk.h ${TRIAL_PATHS_INC_GLPK})
find_library (GLPK_LIBRARIES libglpk.dylib ${TRIAL_PATHS_LIB_GLPK})

## ---------------- ##


else (APPLE)


## ---- LINUX ---- ##	

# possible paths for: 'glpk.h'
set (TRIAL_PATHS_INC_GLPK
  /home/ucl/mctr/bsomers/.local/include
  /home/ucl/mctr
  /usr/local/include
  /home/nvander/glpk-4.57/src # BioRob
  /home/ucl/mctr/nvandernoot/glpk-4.57/src # nico
  /home/ucl/mctr/bsomers/glpk-4.57/src # bruno
  /home/ucl/mctr/heremans/glpk-4.57/src # françois
  /home/ucl/mctr/thabra/glpk-4.57/src # tim
  /home/ucl/mctr/everarts/glpk-4.57/src # chris
)

# possible paths for: 'libglpk.so'
set (TRIAL_PATHS_LIB_GLPK
  /home/ucl/mctr/bsomers/.local/lib
  /usr/local/lib
  /home/nvander/glpk-4.57/src/.libs # BioRob
  /home/ucl/mctr/nvandernoot/glpk-4.57/src/.libs # nico
  /home/ucl/mctr/bsomers/glpk-4.57/src/.libs # bruno
  /home/ucl/mctr/heremans/glpk-4.57/src/.libs # françois
  /home/ucl/mctr/thabra/glpk-4.57/src/.libs # tim
  /home/ucl/mctr/everarts/glpk-4.57/src/.libs # chris
)

find_path (GLPK_INCLUDE_PATH glpk.h ${TRIAL_PATHS_INC_GLPK})
find_library (GLPK_LIBRARIES libglpk.so ${TRIAL_PATHS_LIB_GLPK} NO_DEFAULT_PATH)


## --------------- ##


endif (APPLE)

else (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'glpk.h'
set (TRIAL_PATHS_INC_GLPK
  ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_win/glpk/${WIN_LIB_DIRECTORY}
)

# possible paths for: 'glpk_4_57.lib'
set (TRIAL_PATHS_LIB_GLPK
  ${PROJECT_SOURCE_DIR}/../mbsysCopy/extra_win/glpk/${WIN_LIB_DIRECTORY}
)

find_path (GLPK_INCLUDE_PATH glpk.h ${TRIAL_PATHS_INC_GLPK})
find_library (GLPK_LIBRARIES glpk_4_57.lib ${TRIAL_PATHS_LIB_GLPK})

## ----------------- ##


endif (UNIX)


# flag put to 1 if all required files are found
if (GLPK_INCLUDE_PATH AND GLPK_LIBRARIES)
  set (GLPK_FOUND 1)
else ()
  set (GLPK_FOUND 0)
endif  ()

# outputs
mark_as_advanced(
  GLPK_FOUND
  GLPK_INCLUDE_PATH
  GLPK_LIBRARIES 
)
