/*! 
 * \author Nicolas Van der Noot
 * \file controller_io.hh
 * \brief Controller IO structures
 */

#ifndef _CONTROLLER_IO_HH_
#define _CONTROLLER_IO_HH_

#include "mbs_data.h"

/*! \brief inputs structure
 */
typedef struct Inputs_ctrl
{
	double t;       ///< time [s]
	double *q;      ///< position [rad]
	double *qd;     ///< velocity [rad/s]
	double *Qq;     ///< torques [Nm]
	double *q_mot;  ///< position of the motors [rad]
	double *qd_mot; ///< velocity of the motors [rad/s]
	double F_Rfoot[3]; ///< force under the right foot [N]
	double F_Lfoot[3]; ///< force under the left foot [N]
	double F_RToe_IF[3]; ///< force under the right toe in inertial frame [N] (not existing in real robot)
	double F_LToe_IF[3]; ///< force under the left toe in inertial frame [N] (not existing in real robot)
	double T_Rfoot[3]; ///< torque under the right foot [Nm]
	double T_Lfoot[3]; ///< torque under the left foot [Nm]
	double IMU_Orientation[9];  ///< IMU rotation matrix [-]
	double IMU_Angular_Rate[3]; ///< IMU angular veloicty [rad/s]
	double IMU_Acceleration[3]; ///< IMU angular acceleration [rad/s^2]

} Inputs_ctrl;

/*! \brief outputs structure
 */
typedef struct Outputs_ctrl
{
	double *q_ref;  ///< position references [rad]
	double *Qq_ref; ///< torque references [Nm]

} Outputs_ctrl;


#ifdef __cplusplus
extern "C" {
#endif
	void controller_init_interface(MbsData *mbs_data);
	void controller_loop_interface(MbsData *mbs_data);
	void controller_finish_interface(MbsData *mbs_data);
#ifdef __cplusplus
}
#endif

Inputs_ctrl* init_Inputs_ctrl(int nb_mot);
Outputs_ctrl* init_Outputs_ctrl(int nb_mot);

void free_Inputs_ctrl(Inputs_ctrl *inputs);
void free_Outputs_ctrl(Outputs_ctrl *outputs);

#endif
