/*! 
 * \author Nicolas Van der Noot
 * \file contact_interface.h
 * \brief interface between C++ and C to use the contact library with Robotran
 */

#ifndef _CONTACT_INTERFACE_H_
#define _CONTACT_INTERFACE_H_

#include "mbs_data.h"

#ifdef __cplusplus
extern "C" {
#endif
    void init_contact_geom(MbsData *mbs_data);
    void user_state_contact_geom(MbsData *mbs_data);
    void close_contact_geom(MbsData *mbs_data);
    void update_contact_geom_kinematics(MbsData *mbs_data);
    void update_contact_geom_F_T(MbsData *mbs_data);
    void update_contact_geom(MbsData *mbs_data);
    void apply_contact_geom(MbsData *mbs_data, int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
#ifdef __cplusplus
}
#endif


#endif
