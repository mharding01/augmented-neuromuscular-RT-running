#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to the Biorob cluster for optimization
#
# BIOROB_OPTI_FOUND :        1 if all required files found (0 otherwise)
# BIOROB_OPTI_INCLUDE_PATH : include paths (for the header files) -> for compilation
# BIOROB_OPTI_LIBRARIES :    libraries -> for linkage
#


if  (UNIX)

if  (APPLE)


## ---- MAC OS ---- ##



## ---------------- ##


else  (APPLE)


## ---- LINUX ---- ##	


# -- Include -- #

# possible paths for: 'optimization/messages.hh'
set (TRIAL_PATHS_INC_OPT
	/usr/include/liboptimization-2.0
)

# possible paths for: 'jessevdk/os/os.hh'
set (TRIAL_PATHS_INC_JES
	/usr/include/libjessevdk-1.0
)

# possible paths for: 'glib.h'
set (TRIAL_PATHS_INC_GLIB
	/usr/include/glib-2.0
)

# possible paths for: 'glibconfig.h'
set (TRIAL_PATHS_INC_GLIB_CONFIG
	/usr/lib/x86_64-linux-gnu/glib-2.0/include
)

# possible paths for: 'glibmm.h'
set (TRIAL_PATHS_INC_GLIBMM
	/usr/include/glibmm-2.4
)

# possible paths for: 'glibmmconfig.h'
set (TRIAL_PATHS_INC_GLIBMM_CONFIG
	/usr/lib/x86_64-linux-gnu/glibmm-2.4/include
)

# possible paths for: 'sigc++/sigc++.h'
set (TRIAL_PATHS_INC_SIGC
	/usr/include/sigc++-2.0
)

# possible paths for: 'sigc++config.h'
set (TRIAL_PATHS_INC_SIGC_CONFIG
	/usr/lib/x86_64-linux-gnu/sigc++-2.0/include
)


# -- Lib -- #

# possible paths for: 'liboptimization-2.0.so'
set (TRIAL_PATHS_LIB_BIOROB_OPT
	/usr/lib
)

# possible paths for: 'libjessevdk-1.0.so'
set (TRIAL_PATHS_LIB_BIOROB_JES
	/usr/lib
)

# possible paths for: 'libprotobuf.so'
set (TRIAL_PATHS_LIB_BIOROB_PRO
	/usr/lib
)

# possible paths for: 'libglib-2.0.so'
set (TRIAL_PATHS_LIB_GLIB
	/usr/lib/x86_64-linux-gnu
)

# possible paths for: 'libglibmm-2.4.so'
set (TRIAL_PATHS_LIB_GLIBMM
	/usr/lib/x86_64-linux-gnu/
)


# -- Set result paths -- #

# include
find_path (BIOROB_OPTI_INCLUDE_PATH_OPT optimization/messages.hh ${TRIAL_PATHS_INC_OPT})
find_path (BIOROB_OPTI_INCLUDE_PATH_JES jessevdk/os/os.hh ${TRIAL_PATHS_INC_JES})
find_path (BIOROB_OPTI_INCLUDE_PATH_GLIB glib.h ${TRIAL_PATHS_INC_GLIB})
find_path (BIOROB_OPTI_INCLUDE_PATH_GLIB_CONFIG glibconfig.h ${TRIAL_PATHS_INC_GLIB_CONFIG})
find_path (BIOROB_OPTI_INCLUDE_PATH_GLIBMM glibmm.h ${TRIAL_PATHS_INC_GLIBMM})
find_path (BIOROB_OPTI_INCLUDE_PATH_GLIBMM_CONFIG glibmmconfig.h ${TRIAL_PATHS_INC_GLIBMM_CONFIG})
find_path (BIOROB_OPTI_INCLUDE_PATH_SIGC sigc++/sigc++.h ${TRIAL_PATHS_INC_SIGC})
find_path (BIOROB_OPTI_INCLUDE_PATH_SIGC_CONFIG sigc++config.h ${TRIAL_PATHS_INC_SIGC_CONFIG})

set (BIOROB_OPTI_INCLUDE_PATH ${BIOROB_OPTI_INCLUDE_PATH_OPT} ${BIOROB_OPTI_INCLUDE_PATH_JES} ${BIOROB_OPTI_INCLUDE_PATH_GLIB} ${BIOROB_OPTI_INCLUDE_PATH_GLIB_CONFIG})
set (BIOROB_OPTI_INCLUDE_PATH ${BIOROB_OPTI_INCLUDE_PATH} ${BIOROB_OPTI_INCLUDE_PATH_GLIBMM} ${BIOROB_OPTI_INCLUDE_PATH_GLIBMM_CONFIG})
set (BIOROB_OPTI_INCLUDE_PATH ${BIOROB_OPTI_INCLUDE_PATH} ${BIOROB_OPTI_INCLUDE_PATH_SIGC} ${BIOROB_OPTI_INCLUDE_PATH_SIGC_CONFIG})

# lib
find_library (BIOROB_OPTI_LIBRARIES_BIOROB_OPT liboptimization-2.0.so ${TRIAL_PATHS_LIB_BIOROB_OPT})
find_library (BIOROB_OPTI_LIBRARIES_BIOROB_JES libjessevdk-1.0.so ${TRIAL_PATHS_LIB_BIOROB_JES})
find_library (BIOROB_OPTI_LIBRARIES_BIOROB_PRO libprotobuf.so ${TRIAL_PATHS_LIB_BIOROB_PRO})
find_library (BIOROB_OPTI_LIBRARIES_GLIB libglib-2.0.so ${TRIAL_PATHS_LIB_GLIB})
find_library (BIOROB_OPTI_LIBRARIES_GLIBMM libglibmm-2.4.so ${TRIAL_PATHS_LIB_GLIBMM})

set (BIOROB_OPTI_LIBRARIES ${BIOROB_OPTI_LIBRARIES_BIOROB_OPT} ${BIOROB_OPTI_LIBRARIES_BIOROB_JES} ${BIOROB_OPTI_LIBRARIES_BIOROB_PRO})
set (BIOROB_OPTI_LIBRARIES ${BIOROB_OPTI_LIBRARIES} ${BIOROB_OPTI_LIBRARIES_GLIB} ${BIOROB_OPTI_LIBRARIES_GLIBMM})

## --------------- ##


endif   (APPLE)

else  (UNIX)


## ---- WINDOWS ---- ##


## ----------------- ##


endif   (UNIX)


# flag put to 1 if all required files are found
if (BIOROB_OPTI_INCLUDE_PATH AND BIOROB_OPTI_LIBRARIES)
	set (BIOROB_OPTI_FOUND 1)
else ()
	set (BIOROB_OPTI_FOUND 0)
endif  ()

# outputs
MARK_AS_ADVANCED(
	BIOROB_OPTI_FOUND
	BIOROB_OPTI_INCLUDE_PATH
	BIOROB_OPTI_LIBRARIES 
)
