/*! 
 * \author Nicolas Van der Noot
 * \file user_contact_kinematics.hh
 * \brief kinematics of the contact bodies
 */

#ifndef _USER_CONTACT_KINEMATICS_HH_
#define _USER_CONTACT_KINEMATICS_HH_

extern "C" {
	#include "mbs_sensor.h"
}
#include <vector>
#include "mbs_data.h"

namespace ContactGeom{

// forward declaration
class RigidShape;
class MainUnionShape;

/*! \brief kinematics information
 */ 
typedef struct KinInfo
{
	double P[3]; ///< position [m]
	double V[3]; ///< velocity [m/s]
	double OM[3]; ///< angular velocity [rad/s]
	double R[3][3]; ///< rotation matrix [-]

	int isens; ///< Robotran ID in 'mbs_sensor'
	RigidShape *rigid_shape; ///< RigidShape pointer

	MbsSensor sens; ///< Robotran sensor

} KinInfo;

//  function prototype
void user_contact_kinematics(std::vector<KinInfo> &kin_info_list, MainUnionShape *main_union, MbsData *mbs_data,
	double **PxF_tab, double **VxF_tab, double **OMxF_tab, double ***RxF_tab);
void update_kin_info(double *PxF, double **RxF, double *VxF, double *OMxF, std::vector<KinInfo> &kin_info_list, int isens);

}
#endif
