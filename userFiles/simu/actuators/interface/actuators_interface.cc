#include "actuators_interface.h"
#include "CppInterface.hh"
#include "user_model.h"
#include "RobotActuators.hh"

/*! \brief compute uxd (user derivative with MbsData)
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ux vector integrated by Robotran
 * \param[out] uxd vector to integrate with Robotran
 */
void compute_actuators_derivative(MbsData* mbs_data, double *ux, double *uxd)
{
	CppInterface *cpp_int;
	RobotActuators *actuators;
	
	cpp_int = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	actuators = cpp_int->get_simu_ctrl()->get_actuators();

	actuators->motor_compute_der(ux, uxd);
}

/*! \brief compute the torque due to the motors
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ux vector integrated by Robotran
 * \param[in] uxd vector to integrate with Robotran
 * \param[out] Qq torque vector (with the MBS file indexes)
 */
void compute_actuators_torque(MbsData* mbs_data, double *ux, double *uxd, double *Qq)
{
	CppInterface *cpp_int;
	RobotActuators *actuators;
	
	cpp_int = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	actuators = cpp_int->get_simu_ctrl()->get_actuators();

	actuators->motor_compute_tor(ux, uxd, Qq);
}
