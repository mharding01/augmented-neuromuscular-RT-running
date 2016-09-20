#include "link_compute.h"
#include "LinksRobot.hh"
#include "CppInterface.hh"
#include "user_model.h"

/*! \brief links of the model computation (interface with C)
 * 
 * \param[in] Z length of the link [m]
 * \param[in] Zd derivative of Z [m/s]
 * \param[in] mbs_data Robotran structure
 * \param[in] tsim simulation time [s]
 * \param[in] ilnk ID of the link
 * \return force of the corresponding link [N]
 */
double link_compute(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk)
{
	return static_cast<CppInterface*>(mbs_data->user_model->cppInterface)->get_simu_ctrl()->get_links()->compute(Z, Zd, tsim, ilnk);
}
