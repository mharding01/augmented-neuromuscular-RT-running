#include "StimInit.hh"
#include "ctrl_functions.hh"

inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }
inline double neg(double x){ return (x < 0.0) ? -x : 0.0; }

inline double limit_stim(double x){ return (x < S_MIN) ? S_MIN : (x > S_MAX) ? S_MAX : x; }

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimInit::StimInit(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options): StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	m_st = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	for (int i=0; i<NB_LEGS; i++)
	{
		pitch_knee[i] = parts[i]->get_articulations(PITCH_KNEE_ART);
		pitch_hip[i]  = parts[i]->get_articulations(PITCH_HIP_ART);
		roll_hip[i]   = parts[i]->get_articulations(ROLL_HIP_ART);
		roll_foot[i]  = parts[i]->get_articulations(ROLL_FOOT_ART);
		yaw_hip[i]    = parts[i]->get_articulations(YAW_HIP_ART);
	}

	r_first_swing = inputs->get_options()->is_r_first_swing();

	x_com_ref = 0.0;

	x_com_foot = 0.0;
	y_com_foot = 0.0;

	xp_com_foot = 0.0;
	yp_com_foot = 0.0;

	flag_done = 0;

	// fixed gains
	kp_x_com = 20.0;
	kd_x_com = 10.0;

	kp_pitch_knee = 10.0;
	kp_pitch_hip  = 10.0;
	kp_yaw_hip    = 10.0;

	// fixed timing
	t0 = 0.0;
	t1 = 1.0;

	// parameters to optimize
	target_x_com = 0.04;

	roll_stim    = 0.2;
	y_com_thres  = 0.05;

	// for optimization
	inputs->get_opti_inputs()->set_stim_init(this);
}

/*! \brief destructor
 */
StimInit::~StimInit()
{

}

/*! \brief main computation
 */
void StimInit::compute()
{
	// variables declaration
	double x_com_term;
	double pitch_knee_term, pitch_hip_term;
	double r_yaw_term, l_yaw_term;

	// com position computation
	com_position();

	// com position reference
	com_ref();

	// com regulation
	x_com_term = kp_x_com * (x_com_ref - x_com_foot) - kd_x_com * xp_com_foot;

	// pitch
	for(int i=0; i<NB_LEGS; i++)
	{
		pitch_knee_term = kp_pitch_knee * (0.0 - pitch_knee[i]->get_q());
		pitch_hip_term  = kp_pitch_hip  * (0.0 - pitch_hip[i]->get_q() );

		Stim[i][SOL_MUSCLE] = neg(x_com_term);
		Stim[i][TA_MUSCLE]  = pos(x_com_term);

		Stim[i][GAS_MUSCLE] = S_MIN;

		Stim[i][VAS_MUSCLE] = neg(pitch_knee_term);
		Stim[i][HAM_MUSCLE] = pos(pitch_knee_term);

		Stim[i][GLU_MUSCLE] = pos(pitch_hip_term);
		Stim[i][HFL_MUSCLE] = neg(pitch_hip_term);
	}

	// roll
	if (r_first_swing)
	{
		Stim[R_ID][EVE_MUSCLE] = S_MIN;
		Stim[R_ID][INV_MUSCLE] = roll_stim;

		Stim[L_ID][EVE_MUSCLE] = roll_stim;
		Stim[L_ID][INV_MUSCLE] = S_MIN;
	}
	else
	{
		Stim[R_ID][EVE_MUSCLE] = roll_stim;
		Stim[R_ID][INV_MUSCLE] = S_MIN;

		Stim[L_ID][EVE_MUSCLE] = S_MIN;
		Stim[L_ID][INV_MUSCLE] = roll_stim;
	}
		

	Stim[R_ID][HAB_MUSCLE] = S_MIN;
	Stim[R_ID][HAD_MUSCLE] = S_MIN;

	Stim[L_ID][HAB_MUSCLE] = S_MIN;
	Stim[L_ID][HAD_MUSCLE] = S_MIN;

	// yaw
	r_yaw_term = kp_yaw_hip * (0.0 - yaw_hip[R_ID]->get_q());
	l_yaw_term = kp_yaw_hip * (0.0 - yaw_hip[L_ID]->get_q());

	Stim[R_ID][HER_MUSCLE] = neg(r_yaw_term);
	Stim[R_ID][HIR_MUSCLE] = pos(r_yaw_term);

	Stim[L_ID][HER_MUSCLE] = pos(l_yaw_term);
	Stim[L_ID][HIR_MUSCLE] = neg(l_yaw_term);

	// limiting stimulations
	for(int i=0; i<NB_LEGS; i++)
	{
		for(int j=0; j<NB_LEG_MUSCLES; j++)
		{
			Stim[i][j] = limit_stim(Stim[i][j]);
		}
	}

	if ( (( r_first_swing) && (y_com_foot > -y_com_thres)) || 
		 ((!r_first_swing) && (y_com_foot <  y_com_thres)) )
	{
		flag_done = 1;
	}
}

/*! \brief compute the COM position (relative to the first supporting foot)
 */
void StimInit::com_position()
{
	if (r_first_swing)
	{
		x_com_foot = fwd_kin->get_r_COM_Lfoot_yaw(0);
		y_com_foot = fwd_kin->get_r_COM_Lfoot_yaw(1);

		xp_com_foot = fwd_kin->get_rp_COM_Lfoot_yaw(0);
		yp_com_foot = fwd_kin->get_rp_COM_Lfoot_yaw(1);
	}
	else
	{
		x_com_foot = fwd_kin->get_r_COM_Rfoot_yaw(0);
		y_com_foot = fwd_kin->get_r_COM_Rfoot_yaw(1);

		xp_com_foot = fwd_kin->get_rp_COM_Rfoot_yaw(0);
		yp_com_foot = fwd_kin->get_rp_COM_Rfoot_yaw(1);
	}
}

/*! \brief compute the reference position of the COM
 */
void StimInit::com_ref()
{
	x_com_ref = linear_interpol(0.01, target_x_com, t0, t1, inputs->get_t());
}
