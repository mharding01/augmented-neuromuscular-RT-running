#include "StimTrapezoid.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimTrapezoid::StimTrapezoid(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options):
	StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	time_A = 1.0;
	time_B = 2.0;
	time_C = 3.0;
}

/*! \brief destructor
 */
StimTrapezoid::~StimTrapezoid()
{

}

/*! \brief main stimulation computations
 *
 * // body parts
 * enum{RIGHT_LEG_BODY, LEFT_LEG_BODY, TORSO_BODY, RIGHT_ARM_BODY, LEFT_ARM_BODY, NB_BODY_PARTS};
 *
 * // leg muscles
 * enum{SOL_MUSCLE, TA_MUSCLE, GAS_MUSCLE, VAS_MUSCLE, HAM_MUSCLE, GLU_MUSCLE, HFL_MUSCLE, RF_MUSCLE,
 *   HAB_MUSCLE, HAD_MUSCLE, HER_MUSCLE, HIR_MUSCLE, EVE_MUSCLE, INV_MUSCLE, NB_LEG_MUSCLES};
 * 
 * // torso muscles
 * enum{BFL_MUSCLE, BET_MUSCLE, BTR_MUSCLE, BTL_MUSCLE, BRR_MUSCLE, BRL_MUSCLE, NB_TORSO_MUSCLES};
 * 
 * // arm muscles
 * enum{EFL_MUSCLE, EET_MUSCLE, SFL_MUSCLE, SET_MUSCLE, 
 *   SAB_MUSCLE, SAD_MUSCLE, SER_MUSCLE, SIR_MUSCLE, NB_ARM_MUSCLES};
 */
void StimTrapezoid::compute()
{
	// reset all stimulations
	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		for (int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i][j] = S_MIN;
		}
	}
	stim_all(1.0, 4.0);
	//Stim[0][RF_MUSCLE] = trap_cur_value(inputs->get_t(), 2.0, time_A, time_B, time_C);
	//Stim[1][RF_MUSCLE] = trap_cur_value(inputs->get_t(), 12.0, time_A, time_B, time_C);
}

/*! \brief get trapezoidal value starting: (0.0,S_MIN) ; (time_A,S_MAX) ; (time_B,S_MAX) ; (time_C,S_MIN)
 * 
 * \param[in] cur_t current time [s]
 * \param[in] init_t initial time to start the trapezoidal shape [s]
 * \param[in] time_A first special time [s]
 * \param[in] time_B second special time [s]
 * \param[in] time_C third special time [s]
 * \return requested trapezoidal value
 */
double StimTrapezoid::trap_cur_value(double cur_t, double init_t, double time_A, double time_B, double time_C)
{
	double delta_t = cur_t - init_t;

	if (delta_t <= 0.0)
	{
		return S_MIN;
	}
	else if (delta_t <= time_A)
	{
		return linear_interpol(S_MIN, S_MAX, 0.0, time_A, delta_t);
	}
	else if (delta_t <= time_B)
	{
		return S_MAX;
	}
	else if (delta_t <= time_C)
	{
		return linear_interpol(S_MAX, S_MIN, time_B, time_C, delta_t);
	}
	else
	{
		return S_MIN;
	}
}

/*! \brief stimulate all muscles from one member
 *
 * \param[in] part_id ID of the body part
 * \param[in] init_t initial time [s]
 * \param[in] delta_t time interval for each
 */
void StimTrapezoid::stim_member(int part_id, double init_t, double delta_t)
{
	double cur_t;

	cur_t = inputs->get_t();

	// loop on all the part IDs
	for(int i=0; i<parts[part_id]->get_nb_muscles(); i++)
	{
		Stim[part_id][i] = trap_cur_value(cur_t, init_t+i*delta_t, time_A, time_B, time_C);
	}
}

/*! \brief stimulate all muscles from all members
 *
 * \param[in] init_t initial time [s]
 * \param[in] delta_t time interval for each
 */
void StimTrapezoid::stim_all(double init_t, double delta_t)
{
	double cur_init_t;

	cur_init_t = init_t;

	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		stim_member(i, cur_init_t, delta_t);

		cur_init_t += delta_t*parts[i]->get_nb_muscles();
	}
}
