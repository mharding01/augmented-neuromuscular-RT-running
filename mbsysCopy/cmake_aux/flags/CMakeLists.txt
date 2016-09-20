#
# Author: Nicolas Van der Noot & Timothee Habra
# September 2015
#
# Flags functions 
#

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                                   FLAGS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

## -- Release or debug -- ##

# use Release version (otherwise: Debug)
option (FLAG_RELEASE 
        "Release" ON)


## -- Real-time features related -- ##

# run the simulation in real-time
option (FLAG_REAL_TIME 
        "Real time" OFF)

# use the SDL library to plot real-time graphs and to handle the user inputs (keyboard...)
option (FLAG_PLOT
        "SDL" ON)

# use the Java library to display in real-time the animation
option (FLAG_VISU
        "Java visualization" ON)


## -- Static, dynamic libraries -- ##

# Compile Robotran MBSysC as shared lib (.so / .dll)
option (FLAG_SHARED_LIB
        "Compile as dynamic lib" OFF)

# Link to MBSysC lib already compiled
option (FLAG_SEPARATE_BUILD
        "Link to MBSysC lib already compiled" OFF)

# Link to symbolic files already compiled
option (FLAG_SEPARATE_SYMBOLIC
        "Link to symbolic files already compiled" OFF)



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         SET FLAG VALUE
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

function(set_flag arg0 arg1)
    set(${arg0} ${arg1} PARENT_SCOPE)
endfunction()


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                    SAFETY: INCOMPATIBLE FLAGS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

function(flags_check)

    # real-time
    if (NOT FLAG_REAL_TIME)
        set( FLAG_PLOT OFF )
        set( FLAG_VISU OFF )
    endif( )

    # separate build
    if (FLAG_SEPARATE_BUILD)
        set ( FLAG_SHARED_LIB ON )
    endif ( )

    # parent scope
    set( FLAG_RELEASE           ${FLAG_RELEASE}           PARENT_SCOPE )
    set( FLAG_REAL_TIME         ${FLAG_REAL_TIME}         PARENT_SCOPE )
    set( FLAG_PLOT              ${FLAG_PLOT}              PARENT_SCOPE )
    set( FLAG_VISU              ${FLAG_VISU}              PARENT_SCOPE )
    set( FLAG_SHARED_LIB        ${FLAG_SHARED_LIB}        PARENT_SCOPE )
    set( FLAG_SEPARATE_BUILD    ${FLAG_SEPARATE_BUILD}    PARENT_SCOPE )
    set( FLAG_SEPARATE_SYMBOLIC ${FLAG_SEPARATE_SYMBOLIC} PARENT_SCOPE )

endfunction()


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       RELEASE OR DEBUG
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

function(release_debug)
    if (FLAG_RELEASE)
        set(CMAKE_BUILD_TYPE Release PARENT_SCOPE)
    else ( )
        set(CMAKE_BUILD_TYPE Debug PARENT_SCOPE)
    endif( )
endfunction()


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       ADD DEFINITIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

function(definitions)
    if(UNIX)
        add_definitions( -DUNIX )
    endif( )

    add_definitions( -DDIRDYNARED )
    add_definitions( -DCMEX ) # note: CMEX was defined for Simulink stuff, it should be modified because the Standalone version was added !

    if(FLAG_REAL_TIME)
        add_definitions( -DREAL_TIME )
    endif( )

    if(FLAG_PLOT)
        add_definitions( -DSDL )
    endif( )

    if(FLAG_VISU)
        add_definitions( -DJAVA )
    endif( )
endfunction()
