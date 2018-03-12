#include "StimWalkCtrl.hh"
#include "DelayWalk.hh"
#include "user_realtime.h"
#include "coman_properties.hh"

#define THRESHOLD_DOUBLE_SUP_OSC 0.05 ///< threshold for VAS limitation (releated to oscillators)
#define MAX_TRUNK_THRESHOLD 0.25      ///< threshold for the torso angle [rad]

inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }
inline double neg(double x){ return (x < 0.0) ? -x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimWalkCtrl::StimWalkCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options): StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));

	delay_manager = new DelayWalk();

	osc = new MatsuokaSixN(6, inputs->get_t(), ws, inputs, options);

	for (int i=0; i<NB_LEGS; i++)
	{
		pk[i] = parts[i]->get_articulations(PITCH_KNEE_ART);
		pa[i] = parts[i]->get_articulations(PITCH_FOOT_ART); 

		sol[i] = parts[i]->get_muscle(SOL_MUSCLE);
		ta[i]  = parts[i]->get_muscle(TA_MUSCLE);
		vas[i] = parts[i]->get_muscle(VAS_MUSCLE);

		// feet forces (delay)
		F_foot[i] = 0.0;

		// angles (delay)
		phi_k[i]  = 0.0;
		phip_k[i] = 0.0;

		// l.ce (delay)
		lce_ta[i] = 0.0;

		// Fm (delay)
		F_sol[i] = 0.0;
		F_vas[i] = 0.0;

		roll_hip[i]  = parts[i]->get_articulations(ROLL_HIP_ART);
		roll_foot[i] = parts[i]->get_articulations(ROLL_FOOT_ART);
		yaw_hip[i]   = parts[i]->get_articulations(YAW_HIP_ART);
	}

	flag_3D = options->is_flag_3D();
	supporting_r_leg = options->is_r_first_swing() ? 0 : 1;

	com_position();

	body_orientations();

	// torso (delay)
	theta_torso = 0.0;
	omega_torso = 0.0;

	theta_ref = 0.0;

	k_GLU  = 0.0;
	k_HFL  = 0.0;
	k_HAM1 = 0.0;
	k_HAM2 = 0.0;

	// fixed parameters
	F_max_sol = sol[R_ID]->get_Fmax();
	F_max_vas = vas[R_ID]->get_Fmax();
	l_opt_ta  = ta[R_ID]->get_lopt();
	l_opt_vas = vas[R_ID]->get_lopt();

	// pitch opti
	S0_vas = 0.011107;

	G_sol = 1.222391;
	G_ta_sw = 0.0;
	G_ta_st = 0.0;
	G_vas = 12.092056;

	l_off_ta_sw = 0.0;
	l_off_ta_st = 0.0;
	phi_off_pk = 3.018712;

	ksi_1 = 6.212713;
	ksi_2 = 0.057676;

	// vas extra term
	l_off_vas_sw = 0.0;
	G_vas_sw  = 0.0;

	// roll opti
	kp_torso  = 5.0;
	kd_torso  = 0.0;
	torso_ref = 0.1;

	kp_y_com  = 7.0;
	kd_y_com  = 0.0;
	y_com_ref = 0.04;

	kp_hip = 20.0;
	kd_hip = 0.0;

	kp_sp_foot = 10.0;
	kd_sp_foot = 1.0;

	kp_sw_foot = 5.0;
	kd_sw_foot = 1.0;

	// yaw opti
	kp_yaw = 10.0;
	kd_yaw = 1.0;

	// for optimization
	inputs->get_opti_inputs()->set_stim_ctrl(this);
}

/*! \brief destructor
 */
StimWalkCtrl::~StimWalkCtrl()
{
	delete osc;

	delete delay_manager;
}

/*! \brief main computation
 */
void StimWalkCtrl::compute()
{
	com_position();

	body_orientations();

	compute_delay();

	update_oscillators();

	compute_stimulation();
}

/*! \brief compute the COM position (relative to the supporting foot)
 */
void StimWalkCtrl::com_position()
{
	supporting_r_leg = sw_st->is_supporting_r_leg();

	if (supporting_r_leg)
	{
		x_com_foot = fwd_kin->get_r_COM_Rfoot_yaw(0);
		y_com_foot = fwd_kin->get_r_COM_Rfoot_yaw(1);

		xp_com_foot = fwd_kin->get_rp_COM_Rfoot_yaw(0);
		yp_com_foot = fwd_kin->get_rp_COM_Rfoot_yaw(1);
	}
	else
	{
		x_com_foot = fwd_kin->get_r_COM_Lfoot_yaw(0);
		y_com_foot = fwd_kin->get_r_COM_Lfoot_yaw(1);

		xp_com_foot = fwd_kin->get_rp_COM_Lfoot_yaw(0);
		yp_com_foot = fwd_kin->get_rp_COM_Lfoot_yaw(1);
	}
}

/*! \brief compute the torso and the feet orientation angles and derivatives
 */
void StimWalkCtrl::body_orientations()
{
	// torso roll
	theta_roll_torso = inputs->get_theta_torso(0);
	omega_roll_torso = inputs->get_omega_torso(0);

	// feet roll
	theta_roll_Rfoot = inputs->get_theta_Rfoot(0);
	theta_roll_Lfoot = inputs->get_theta_Lfoot(0);

	omega_roll_Rfoot = inputs->get_omega_Rfoot(0);
	omega_roll_Lfoot = inputs->get_omega_Lfoot(0);
}

/*! \brief compute delays in signals
 */
void StimWalkCtrl::compute_delay()
{
	// update time
	delay_manager->set_t(inputs->get_t());

	// feet forces (delay)
	F_foot[R_ID] = delay_manager->update_and_get(F_FOOT_R, inputs->get_mean_Fz_feet(R_ID));
	F_foot[L_ID] = delay_manager->update_and_get(F_FOOT_L, inputs->get_mean_Fz_feet(L_ID));

	// angles (delay)
	phi_k[R_ID]  = delay_manager->update_and_get(PHI_K_R,  pk[R_ID]->get_q());
	phi_k[L_ID]  = delay_manager->update_and_get(PHI_K_L,  pk[L_ID]->get_q());
	phip_k[R_ID] = delay_manager->update_and_get(PHIP_K_R, pk[R_ID]->get_qd());
	phip_k[L_ID] = delay_manager->update_and_get(PHIP_K_L, pk[L_ID]->get_qd());

	// l.ce (delay)
	lce_ta[R_ID]  = delay_manager->update_and_get(LCE_TA_R, ta[R_ID]->get_lce());
	lce_ta[L_ID]  = delay_manager->update_and_get(LCE_TA_L, ta[L_ID]->get_lce());
	lce_vas[R_ID] = delay_manager->update_and_get(LCE_VAS_R, vas[R_ID]->get_lce());
	lce_vas[L_ID] = delay_manager->update_and_get(LCE_VAS_L, vas[L_ID]->get_lce());

	// Fm (delay)
	F_sol[R_ID] = delay_manager->update_and_get(F_SOL_R, sol[R_ID]->get_Fm());
	F_sol[L_ID] = delay_manager->update_and_get(F_SOL_L, sol[L_ID]->get_Fm());
	F_vas[R_ID] = delay_manager->update_and_get(F_VAS_R, vas[R_ID]->get_Fm());
	F_vas[L_ID] = delay_manager->update_and_get(F_VAS_L, vas[L_ID]->get_Fm());

	// torso (delay)
	theta_torso = delay_manager->update_and_get(THETA_TORSO, inputs->get_theta_torso(1));
	omega_torso = delay_manager->update_and_get(OMEGA_TORSO, inputs->get_omega_torso(1));
}

/*! \brief update the oscillators
 */
void StimWalkCtrl::update_oscillators()
{
	osc->update(inputs->get_t());

	//theta_ref = osc->get_theta_ref();

	k_GLU = osc->get_k_GLU();
	k_HFL = osc->get_k_HFL();
	k_HAM1 = osc->get_k_HAM1();
	k_HAM2 = osc->get_k_HAM2();
}

/*! \brief compute the stimulations
 */
void StimWalkCtrl::compute_stimulation()
{
	pitch_compute();

	if (flag_3D)
	{
		roll_compute();
		yaw_compute();
	}
	else
	{
		roll_compute_min();
		yaw_compute_min();
	}
}

/*! \brief pitch muscles computation
 */
void StimWalkCtrl::pitch_compute()
{
	// variables declaration
	double osc_knee;
	double PD_torso, P_torso_max;

	// torso backward (to pull with HFL)
	PD_torso = -(theta_torso - theta_ref) - ksi_2 * omega_torso;

	// torso forward (to pull with GLU)
	P_torso_max = theta_torso - MAX_TRUNK_THRESHOLD;

	// oscillators
	Stim[R_ID][HFL_MUSCLE] = k_HFL  * osc->get_y_pos(3);
	Stim[R_ID][GLU_MUSCLE] = k_GLU  * osc->get_y_pos(0);
	Stim[R_ID][HAM_MUSCLE] = k_HAM1 * osc->get_y_pos(0) + k_HAM2 * osc->get_y_pos(2);
	
	Stim[L_ID][HFL_MUSCLE] = k_HFL  * osc->get_y_pos(1);
	Stim[L_ID][GLU_MUSCLE] = k_GLU  * osc->get_y_pos(2);
	Stim[L_ID][HAM_MUSCLE] = k_HAM1 * osc->get_y_pos(2) + k_HAM2 * osc->get_y_pos(0);

	for (int i=0; i<NB_LEGS; i++)
	{
		// swing
		if (sw_st->is_swing_leg(i))
		{
			// SOL
			Stim[i][SOL_MUSCLE] = S_MIN;

			// TA
			Stim[i][TA_MUSCLE] = S_MIN + G_ta_sw * ( (lce_ta[i] / l_opt_ta) - l_off_ta_sw);

			// GAS
			Stim[i][GAS_MUSCLE] = S_MIN;

			// VAS
			Stim[i][VAS_MUSCLE] = S0_vas;

			if (options->is_extra_knee())
			{
				if (i == R_ID)
				{
					osc_knee = osc->get_x_pos(4);
				}
				else
				{
					osc_knee = osc->get_x_pos(1);
				}

				Stim[i][VAS_MUSCLE] += G_vas_sw * osc_knee * pos( (lce_vas[i] / l_opt_vas) - l_off_vas_sw);
			}
		}
		// stance
		else
		{
			// SOL
			Stim[i][SOL_MUSCLE] = S_MIN + G_sol * (F_sol[i] / F_max_sol);

			// TA
			Stim[i][TA_MUSCLE] = S_MIN + G_ta_st * ( (lce_ta[i] / l_opt_ta) - l_off_ta_st);

			// GAS
			Stim[i][GAS_MUSCLE] = S_MIN;

			// VAS
			Stim[i][VAS_MUSCLE] = S0_vas + G_vas * (F_vas[i] / F_max_vas);

			if ((phi_k[i] < phi_off_pk) && (phip_k[i] < 0.0))
			{
				Stim[i][VAS_MUSCLE] = S0_vas;
			}

			if ( ((i == R_ID) && (osc->get_x(3) > THRESHOLD_DOUBLE_SUP_OSC)) || ((i == L_ID) && (osc->get_x(0) > THRESHOLD_DOUBLE_SUP_OSC)) ) // trailing leg in double support
			{
				Stim[i][VAS_MUSCLE] = S0_vas;
			}

			// GLU
			if (P_torso_max > 0.0)
			{
				Stim[i][GLU_MUSCLE] += P_torso_max * ksi_1 * (F_foot[i] / WEIGHT_COMAN);
			}

			// HFL
			if (PD_torso > 0.0)
			{
				Stim[i][HFL_MUSCLE] += PD_torso * ksi_1 * (F_foot[i] / WEIGHT_COMAN);
			}
		}
	}
}

/*! \brief roll muscles computation
 */
void StimWalkCtrl::roll_compute()
{
	double torso_term, com_term, com_angle;
	double sp_foot_term, sw_foot_term;

	if (supporting_r_leg) // right leg in supporting phase
	{
		// hip
		torso_term =  (kp_torso * (torso_ref - theta_roll_torso       ) - kd_torso * omega_roll_torso) * (F_foot[R_ID] / WEIGHT_COMAN);
		com_angle  = -(kp_y_com * (y_com_ref - y_com_foot             ) - kd_y_com * yp_com_foot);
		com_term   =   kp_hip   * (com_angle - roll_hip[L_ID]->get_q()) - kd_hip   * roll_hip[L_ID]->get_qd();

		Stim[R_ID][HAB_MUSCLE] = pos(torso_term);
		Stim[R_ID][HAD_MUSCLE] = neg(torso_term);

		Stim[L_ID][HAB_MUSCLE] = pos(com_term);
		Stim[L_ID][HAD_MUSCLE] = neg(com_term);

		// foot
		sp_foot_term = kp_sp_foot * (0.0 - roll_foot[R_ID]->get_q()) - kd_sp_foot * roll_foot[R_ID]->get_qd();
		sw_foot_term = kp_sw_foot * (0.0 - theta_roll_Lfoot        ) - kd_sw_foot * omega_roll_Lfoot;

		Stim[R_ID][EVE_MUSCLE] = neg(sp_foot_term);
		Stim[R_ID][INV_MUSCLE] = pos(sp_foot_term);

		Stim[L_ID][EVE_MUSCLE] = pos(sw_foot_term);
		Stim[L_ID][INV_MUSCLE] = neg(sw_foot_term);
	}
	else // left leg in supporting phase
	{
		// hip
		torso_term =  (kp_torso * (-torso_ref - theta_roll_torso       ) - kd_torso * omega_roll_torso) * (F_foot[L_ID] / WEIGHT_COMAN);
		com_angle  = -(kp_y_com * (-y_com_ref - y_com_foot             ) - kd_y_com * yp_com_foot);
		com_term   =   kp_hip   * (com_angle  - roll_hip[R_ID]->get_q()) - kd_hip   * roll_hip[R_ID]->get_qd();

		Stim[R_ID][HAB_MUSCLE] = neg(com_term);
		Stim[R_ID][HAD_MUSCLE] = pos(com_term);

		Stim[L_ID][HAB_MUSCLE] = neg(torso_term);
		Stim[L_ID][HAD_MUSCLE] = pos(torso_term);

		// foot
		sp_foot_term = kp_sp_foot * (0.0 - roll_foot[L_ID]->get_q()) - kd_sp_foot * roll_foot[L_ID]->get_qd();
		sw_foot_term = kp_sw_foot * (0.0 - theta_roll_Rfoot        ) - kd_sw_foot * omega_roll_Rfoot;

		Stim[R_ID][EVE_MUSCLE] = neg(sw_foot_term);
		Stim[R_ID][INV_MUSCLE] = pos(sw_foot_term);

		Stim[L_ID][EVE_MUSCLE] = pos(sp_foot_term);
		Stim[L_ID][INV_MUSCLE] = neg(sp_foot_term);
	}
}

/*! \brief yaw muscles computation
 */
void StimWalkCtrl::yaw_compute()
{
	double r_yaw_term, l_yaw_term;

	r_yaw_term = kp_yaw * (0.0 - yaw_hip[R_ID]->get_q()) - kd_yaw * yaw_hip[R_ID]->get_qd();
	l_yaw_term = kp_yaw * (0.0 - yaw_hip[L_ID]->get_q()) - kd_yaw * yaw_hip[L_ID]->get_qd();

	Stim[R_ID][HER_MUSCLE] = neg(r_yaw_term);
	Stim[R_ID][HIR_MUSCLE] = pos(r_yaw_term);

	Stim[L_ID][HER_MUSCLE] = pos(l_yaw_term);
	Stim[L_ID][HIR_MUSCLE] = neg(l_yaw_term);

}

/*! \brief pitch muscles computation: minimal stimulations
 */
void StimWalkCtrl::pitch_compute_min()
{
	for (int i=0; i<NB_LEGS; i++)
	{
		Stim[i][SOL_MUSCLE] = S_MIN;
		Stim[i][TA_MUSCLE]  = S_MIN;
		Stim[i][GAS_MUSCLE] = S_MIN;
		Stim[i][VAS_MUSCLE] = S_MIN;
		Stim[i][HAM_MUSCLE] = S_MIN;
		Stim[i][GLU_MUSCLE] = S_MIN;
		Stim[i][HFL_MUSCLE] = S_MIN;
	}
}

/*! \brief roll muscles computation: minimal stimulations
 */
void StimWalkCtrl::roll_compute_min()
{
	for (int i=0; i<NB_LEGS; i++)
	{
		Stim[i][HAB_MUSCLE] = S_MIN;
		Stim[i][HAD_MUSCLE] = S_MIN;
		Stim[i][EVE_MUSCLE] = S_MIN;
		Stim[i][INV_MUSCLE] = S_MIN;
	}
}

/*! \brief yaw muscles computation: minimal stimulations
 */
void StimWalkCtrl::yaw_compute_min()
{
	for (int i=0; i<NB_LEGS; i++)
	{
		Stim[i][HER_MUSCLE] = S_MIN;
		Stim[i][HIR_MUSCLE] = S_MIN;
	}
}
