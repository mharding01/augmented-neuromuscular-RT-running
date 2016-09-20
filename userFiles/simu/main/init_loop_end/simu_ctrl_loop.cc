/*! 
 * \author Nicolas Van der Noot
 * \file simu_ctrl_loop.cc
 * \brief Main loop of the controller and user simulation files
 */

#include "CppInterface.hh"
#include "user_simu_functions.h"
#include "user_model.h"

/*! \brief ineterface to run the main loop of the simulation (also calling the controller)
 * 
 * \param[in] mbs_data Robotran strcuture
 */
void simu_controller_loop(MbsData *mbs_data)
{
    CppInterface *cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

    cppInterface->get_simu_ctrl()->compute();
}
