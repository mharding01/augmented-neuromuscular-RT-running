/*! 
 * \author Nicolas Van der Noot
 * \file simu_init.cc
 * \brief Handle the optimization inputs (not the ones related to the controller).
 */

#include "user_simu_functions.h"
#include "CppInterface.hh"
#include "CtrlInterface.hh"
#include "user_model.h"

/*! \brief initialization of the simulation
 * 
 * \param[in] mbs_data Robotran structure
 */
void simu_init(MbsData *mbs_data)
{
	mbs_data->user_model->cppInterface = new CtrlInterface(mbs_data);
}

/*! \brief initialization of the simulation when the contactGeom class is initialized
 * 
 * \param[in] mbs_data Robotran structure
 */
void simu_init_contact(MbsData *mbs_data)
{
	static_cast<CppInterface*>(mbs_data->user_model->cppInterface)->get_simu_ctrl()->init_contactGeom();
}
