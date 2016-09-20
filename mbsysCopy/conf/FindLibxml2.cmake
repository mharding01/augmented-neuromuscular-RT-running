#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to Libxml2
#
# LIBXML2_FOUND :        1 if all required files found (0 otherwise)
# LIBXML2_INCLUDE_PATH : include paths (for the header files) -> for compilation
# LIBXML2_LIBRARIES :    libraries -> for linkage
#


if (UNIX)

if (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'tree.h'
set(TRIAL_PATHS_INC_LIBXML_1
  /usr/bin/libxml 2.framework/Headers/libxml
  /usr/include/libxml2/libxml
  /usr/local/include/libxml2/libxml
) 

# possible paths for: 'libxml/tree.h'
set(TRIAL_PATHS_INC_LIBXML_2
  /usr/bin/libxml 2.framework/Headers
  /usr/include/libxml2
  /usr/local/include/libxml2
) 

# possible paths for: 'libxml2.dylib'
set(TRIAL_PATHS_LIB_LIBXML2
  /usr/lib
) 

find_path(LIBXML2_INCLUDE_PATH_1 tree.h ${TRIAL_PATHS_INC_LIBXML_1})
find_path(LIBXML2_INCLUDE_PATH_2 libxml/tree.h ${TRIAL_PATHS_INC_LIBXML_2})

set(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_1} ${LIBXML2_INCLUDE_PATH_2})

find_library(LIBXML2_LIBRARIES libxml2.dylib ${TRIAL_PATHS_LIB_LIBXML2})


## ---------------- ##


else (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'iconv.h'
set(TRIAL_PATHS_INC_ICONV
  /usr/include
)

# possible paths for: 'tree.h'
set(TRIAL_PATHS_INC_LIBXML_1
  /usr/include/libxml2/libxml
)

# possible paths for: 'libxml/tree.h'
set(TRIAL_PATHS_INC_LIBXML_2
  /usr/include/libxml2
)

# possible paths for: 'libxml2.so'
set(TRIAL_PATHS_LIB_LIBXML2
  /usr/lib/i386-linux-gnu
  /usr/lib/lib64
)

find_path(LIBXML2_INCLUDE_PATH_ICONV iconv.h ${TRIAL_PATHS_INC_ICONV})
find_path(LIBXML2_INCLUDE_PATH_LIBXML_1 tree.h ${TRIAL_PATHS_INC_LIBXML_1})
find_path(LIBXML2_INCLUDE_PATH_LIBXML_2 libxml/tree.h ${TRIAL_PATHS_INC_LIBXML_2})

set(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_ICONV} ${LIBXML2_INCLUDE_PATH_LIBXML_1} ${LIBXML2_INCLUDE_PATH_LIBXML_2})

find_library(LIBXML2_LIBRARIES libxml2.so ${TRIAL_PATHS_LIB_LIBXML2})


## --------------- ##


endif (APPLE)

else (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'iconv.h'
set(TRIAL_PATHS_INC_ICONV
  ${PROJECT_SOURCE_DIR}/src/other/${WIN_LIB_DIRECTORY}/include/libxml
  ${PROJECT_ABS_PATH}/src/other/${WIN_LIB_DIRECTORY}/include/libxml
  ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/include/libxml
) 

# possible paths for: 'tree.h'
set(TRIAL_PATHS_INC_LIBXML_1
  ${PROJECT_SOURCE_DIR}/src/other/${WIN_LIB_DIRECTORY}/include/libxml
  ${PROJECT_ABS_PATH}/src/other/${WIN_LIB_DIRECTORY}/include/libxml
  ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/include/libxml
) 

# possible paths for: 'libxml/tree.h'
set(TRIAL_PATHS_INC_LIBXML_2
  ${PROJECT_SOURCE_DIR}/src/other/${WIN_LIB_DIRECTORY}/include
  ${PROJECT_ABS_PATH}/src/other/${WIN_LIB_DIRECTORY}/include
  ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/include
) 

# possible paths for: 'iconv.lib'
set(TRIAL_PATHS_LIB_ICONV
  ${PROJECT_SOURCE_DIR}/src/other/${WIN_LIB_DIRECTORY}/lib
  ${PROJECT_ABS_PATH}/src/other/${WIN_LIB_DIRECTORY}/lib
  ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/lib
) 

# possible paths for: 'libxml2.lib'
set(TRIAL_PATHS_LIB_LIBXML2
  ${PROJECT_SOURCE_DIR}/src/other/${WIN_LIB_DIRECTORY}/lib
  ${PROJECT_ABS_PATH}/src/other/${WIN_LIB_DIRECTORY}/lib
  ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/lib
) 

find_path(LIBXML2_INCLUDE_PATH_ICONV iconv.h ${TRIAL_PATHS_INC_ICONV})
find_path(LIBXML2_INCLUDE_PATH_LIBXML_1 tree.h ${TRIAL_PATHS_INC_LIBXML_1})
find_path(LIBXML2_INCLUDE_PATH_LIBXML_2 libxml/tree.h ${TRIAL_PATHS_INC_LIBXML_2})

find_library(LIBXML2_LIBRARIES_ICONV iconv.lib ${TRIAL_PATHS_LIB_ICONV})
find_library(LIBXML2_LIBRARIES_LIBXML2 libxml2.lib ${TRIAL_PATHS_LIB_LIBXML2})

set(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_ICONV} ${LIBXML2_INCLUDE_PATH_LIBXML_1} ${LIBXML2_INCLUDE_PATH_LIBXML_2})
set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES_ICONV} ${LIBXML2_LIBRARIES_LIBXML2})

## ----------------- ##


endif (UNIX)


# flag put to 1 if all required files are found
if (LIBXML2_INCLUDE_PATH AND LIBXML2_LIBRARIES)
  set(LIBXML2_FOUND 1)
else ( )
  set(LIBXML2_FOUND 0)
endif ( )

# outputs
mark_as_advanced(
  LIBXML2_FOUND
  LIBXML2_INCLUDE_PATH
  LIBXML2_LIBRARIES 
)
