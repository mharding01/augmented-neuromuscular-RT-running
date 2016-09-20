#
# Author: Nicolas Van der Noot
# September 2015
#
# Make options functions 
#

# Make options
function(make_options)

    if (UNIX)

        # general variables
        set( SCRIPTS_PATH ${ROBOTRAN_SOURCE_DIR}/cmake_aux/scripts )

        # scripts executable
        execute_process(COMMAND chmod +x ${SCRIPTS_PATH}/mbsysc_build)
        execute_process(COMMAND chmod +x ${SCRIPTS_PATH}/symbolic_lib)

        # mbsysc_options (options listing with an example of cmake script)
        add_custom_target(mbsysc_options COMMAND ${CMAKE_COMMAND} -P ${SCRIPTS_PATH}/mbsysc_options )

        # mbsysc_basic (SHARED_LIB, SEPARATE_BUILD, RELEASE, REAL_TIME, PLOT, VISU)
        add_custom_target(mbsysc_basic COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./mbsysc_build ${ROBOTRAN_SOURCE_DIR} ON ON ON OFF OFF OFF)

        # mbsysc_plot (SHARED_LIB, SEPARATE_BUILD, RELEASE, REAL_TIME, PLOT, VISU)
        add_custom_target(mbsysc_plot COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./mbsysc_build ${ROBOTRAN_SOURCE_DIR} ON ON ON ON ON OFF)

        # mbsysc_visu (SHARED_LIB, SEPARATE_BUILD, RELEASE, REAL_TIME, PLOT, VISU)
        add_custom_target(mbsysc_visu COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./mbsysc_build ${ROBOTRAN_SOURCE_DIR} ON ON ON ON OFF ON)

        # mbsysc_realtime (SHARED_LIB, SEPARATE_BUILD, RELEASE, REAL_TIME, PLOT, VISU)
        add_custom_target(mbsysc_realtime COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./mbsysc_build ${ROBOTRAN_SOURCE_DIR} ON ON ON ON ON ON)

        # mbsysc_clean (SHARED_LIB, SEPARATE_BUILD, RELEASE, REAL_TIME, PLOT, VISU)
        add_custom_target(mbsysc_clean COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./mbsysc_build ${ROBOTRAN_SOURCE_DIR} "clean")

        # symb_lib
        add_custom_target(symb_lib COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./symbolic_lib ${PROJECT_SOURCE_DIR}/../symbolicR ${ROBOTRAN_SOURCE_DIR}/mbs_app/symbolic_lib)

        # symb_clean
        add_custom_target(symb_clean COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPTS_PATH} ./symbolic_lib ${PROJECT_SOURCE_DIR}/../symbolicR ${ROBOTRAN_SOURCE_DIR}/mbs_app/symbolic_lib "clean")

    endif ( )

endfunction()
