#include "contact_interface.h"
#include "ContactGestion.hh"
#include "user_IO.h"

/*! \brief extract ContactGestion from mbs_data
 * 
 * \param[in] mbs_data Robotran structure
 * \return ContactGestion
 */
ContactGeom::ContactGestion* get_ContactGestion(MbsData *mbs_data)
{
    return static_cast<ContactGeom::ContactGestion*>(mbs_data->user_IO->contactGestion);
}

/*! \brief initialize ContactGestion
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void init_contact_geom(MbsData *mbs_data)
{
    mbs_data->user_IO->contactGestion = new ContactGeom::ContactGestion(mbs_data);
}

void user_state_contact_geom(MbsData *mbs_data)
{
    get_ContactGestion(mbs_data)->update_user_state();
}

/*! \brief delete ContactGestion
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void close_contact_geom(MbsData *mbs_data)
{
    delete get_ContactGestion(mbs_data);
}

/*! \brief update the kinematics of the shapes in ContactGestion
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void update_contact_geom_kinematics(MbsData *mbs_data)
{
    get_ContactGestion(mbs_data)->update_kinematics();
}

/*! \brief update the forces with ContactGestion
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void update_contact_geom_F_T(MbsData *mbs_data)
{
    get_ContactGestion(mbs_data)->update_F_T();
}

/*! \brief update the kinematics and force-torque computation
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void update_contact_geom(MbsData *mbs_data)
{
    update_contact_geom_kinematics(mbs_data);
    update_contact_geom_F_T(mbs_data);
}

/*! \brief apply the forces and torques computed by the force-contact model
 * 
 * \param[in,out] mbs_data Robotran structure
 * \param[in] ixF ID of the Fsensor
 * \param[out] Fx force in the x direction
 * \param[out] Fy force in the y direction
 * \param[out] Fz force in the z direction
 * \param[out] Mx moment in the x direction
 * \param[out] My moment in the y direction
 * \param[out] Mz moment in the z direction
 */
void apply_contact_geom(MbsData *mbs_data, int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz)
{
    get_ContactGestion(mbs_data)->get_main_union()->apply_F_T(ixF, Fx, Fy, Fz, Mx, My, Mz);
}
