#include "compute_driven.h"
#include "CppInterface.hh"
#include "user_model.h"

/*! \brief compyte driven joints (interface with C)
 * 
 * \param[in] mbs_data Robotran structure
 */
void compute_driven(MbsData *mbs_data)
{
	static_cast<CppInterface*>(mbs_data->user_model->cppInterface)->get_simu_ctrl()->compute_driven();
}
