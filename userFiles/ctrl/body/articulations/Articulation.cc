
#include "Articulation.hh"

#define TAU_QQ_SOFT 0.05 ///< time constant for tau low filter [s]

// torque ratio = 0.1376 -> 0.3*0.1376*(180/pi)*(180/pi) = 135.514 [Nm*s/rad^2] (see sagittal leg)

#define KD_THRES_SOFT_LIM 0.01745 ///< threshold for soft contribution [rad/s] = 1.0 [deg/s]

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] ctrl_index indexes of the controller
 * \param[in] art_id ID of the articulation
 * \param[in] body_part_id ID of the arm
 */
Articulation::Articulation(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int art_id, int body_part_id)
{
	this->inputs = inputs;
	this->ctrl_index = ctrl_index;	
	this->art_id = art_id;
	this->body_part_id = body_part_id;

	q  = 0.0;
	qd = 0.0;
	Qq = 0.0;
	Qq_ref = 0.0;

	filter_Qq_soft = new LowFilter(TAU_QQ_SOFT, 0.0, inputs->get_t());
}

/*! \brief destructor
 */
Articulation::~Articulation()
{
	delete filter_Qq_soft;
}

/*! \brief compute the contribution of the soft limit
 * 
 * \param[in] k_qq proportional constant [s/rad^2]
 * \return requested contribution [Nm]
 */
double Articulation::compute_Qq_soft_contribution(double k_qq)
{
	double delta_min   = q_min - q; // [rad]
	double delta_max   = q - q_max; // [rad]

	if ((delta_min > 0.0) && (qd < KD_THRES_SOFT_LIM))
	{
		return k_qq * delta_min * (KD_THRES_SOFT_LIM - qd);
	}
	else if ((delta_max > 0.0) && (qd > -KD_THRES_SOFT_LIM))
	{
		return -k_qq * delta_max * (KD_THRES_SOFT_LIM + qd);			
	}
	else
	{
		return 0.0;
	}
}
