
#include "StimGeyerCtrl.hh"
#include "DelayGeyer.hh"
#include "coman_properties.hh"

#define TA_EXTRA_K 4.0
#define TA_EXTRA_THRESHOLD -0.1
#define EXTRA_TA

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimGeyerCtrl::StimGeyerCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options): StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));
	tr_st = static_cast<TrailingState*>(ws->get_state(TRAILING_STATE));

	delay_manager = new DelayGeyer();

	flag_3D = options->is_flag_3D();

	if (flag_3D)
	{
		std::cout << "Error: 3D walking not implemented for Geyer controller" << std::endl;
		exit(EXIT_FAILURE);
	}

	for (int i=0; i<NB_LEGS; i++)
	{
		pk[i] = parts[i]->get_articulations(PITCH_KNEE_ART);
		pa[i] = parts[i]->get_articulations(PITCH_FOOT_ART);
		
		sol[i] = parts[i]->get_muscle(SOL_MUSCLE);
		ta[i]  = parts[i]->get_muscle(TA_MUSCLE);
		vas[i] = parts[i]->get_muscle(VAS_MUSCLE);
		ham[i] = parts[i]->get_muscle(HAM_MUSCLE);
		hfl[i] = parts[i]->get_muscle(HFL_MUSCLE);

		// feet forces (delay)
		F_foot[i] = 0.0;

		// angles (delay)
		phi_k[i]  = 0.0;
		phip_k[i] = 0.0;

		// l.ce (delay)
		lce_ta[i]  = 0.0;
		lce_vas[i] = 0.0;
		lce_ham[i] = 0.0;
		lce_hfl[i] = 0.0;

		// Fm (delay)
		F_sol[i] = 0.0;
		F_vas[i] = 0.0;
	}

	// torso (delay)
	theta_torso = 0.0;
	omega_torso = 0.0;

	// fixed parameters
	F_max_sol = sol[R_ID]->get_Fmax();
	F_max_vas = vas[R_ID]->get_Fmax();
	l_opt_ta  = ta[R_ID]->get_lopt();
	l_opt_vas = vas[R_ID]->get_lopt();
	l_opt_ham = ham[R_ID]->get_lopt();
	l_opt_hfl = hfl[R_ID]->get_lopt();

	// opti parameters
	S0_vas = 0.01067721;
	G_sol = 1.00268000;
	G_ta_sw = 1.28686400;
	G_ta_st = 1.86938000;
	G_vas = 9.09615400;
	G_ham_hfl = 3.59343500;
	phi_off_pk = 2.99840060;
	l_off_ta_sw = 0.89909385;
	l_off_ta_st = 0.55937220;
	l_off_ham = 1.03233670;
	k_bw = 0.40899500;
	k_p = 10.00000000;
	k_d = 0.26193767;
	theta_ref = 0.17472449;

	// vas extra term
	l_off_vas_sw = 1.0;
	G_vas_sw  = 1.2;

	l_off_ham_sw = 0.8;
	G_vas_sw  = 1.2;

	// ham extra term
	S_ham_tr = 1.0;

	// for optimization
	inputs->get_opti_inputs()->set_stim_ctrl(this);
}

/*! \brief destructor
 */
StimGeyerCtrl::~StimGeyerCtrl()
{
	delete delay_manager;
}

/*! \brief main computation
 */
void StimGeyerCtrl::compute()
{
	compute_delay();

	compute_stimulation();
}

/*! \brief compute delays in signals
 */
void StimGeyerCtrl::compute_delay()
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
	lce_ta[R_ID]  = delay_manager->update_and_get(LCE_TA_R,  ta[R_ID]->get_lce());
	lce_ta[L_ID]  = delay_manager->update_and_get(LCE_TA_L,  ta[L_ID]->get_lce());
	lce_vas[R_ID] = delay_manager->update_and_get(LCE_VAS_R, ham[R_ID]->get_lce());
	lce_vas[L_ID] = delay_manager->update_and_get(LCE_VAS_L, ham[L_ID]->get_lce());
	lce_ham[R_ID] = delay_manager->update_and_get(LCE_HAM_R, ham[R_ID]->get_lce());
	lce_ham[L_ID] = delay_manager->update_and_get(LCE_HAM_L, ham[L_ID]->get_lce());
	lce_hfl[R_ID] = delay_manager->update_and_get(LCE_HFL_R, hfl[R_ID]->get_lce());
	lce_hfl[L_ID] = delay_manager->update_and_get(LCE_HFL_L, hfl[L_ID]->get_lce());

	// Fm (delay)
	F_sol[R_ID] = delay_manager->update_and_get(F_SOL_R, sol[R_ID]->get_Fm());
	F_sol[L_ID] = delay_manager->update_and_get(F_SOL_L, sol[L_ID]->get_Fm());
	F_vas[R_ID] = delay_manager->update_and_get(F_VAS_R, vas[R_ID]->get_Fm());
	F_vas[L_ID] = delay_manager->update_and_get(F_VAS_L, vas[L_ID]->get_Fm());

	// torso (delay)
	theta_torso = delay_manager->update_and_get(THETA_TORSO, inputs->get_theta_torso(1));
	omega_torso = delay_manager->update_and_get(OMEGA_TORSO, inputs->get_omega_torso(1));
}

/*! \brief compute the stimulations
 */
void StimGeyerCtrl::compute_stimulation()
{
	// set stimulations to minimal value
	for(int i=RIGHT_LEG_BODY; i<=LEFT_LEG_BODY; i++)
	{
		for(int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i][j] = S_MIN;
		}
	}

	pitch_compute();

	roll_compute_min();

	yaw_compute_min();

	// limit stimulations
	for(int i=RIGHT_LEG_BODY; i<=LEFT_LEG_BODY; i++)
	{
		for(int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i][j] = limit_range(Stim[i][j], S_MIN, S_MAX);
		}
	}
}

/*! \brief pitch muscles computation
 */
void StimGeyerCtrl::pitch_compute()
{
	double PD_torso_1, PD_torso_2;

	PD_torso_1 =        k_p * (theta_torso - theta_ref) + k_d * omega_torso;
	PD_torso_2 = 0.68 * k_p * (theta_torso - theta_ref) + k_d * omega_torso;

	for (int i=0; i<NB_LEGS; i++)
	{
		// swing
		if (sw_st->is_swing_leg(i))
		{
			// SOL
			Stim[i][SOL_MUSCLE] = S_MIN;

			// TA
			Stim[i][TA_MUSCLE] = S_MIN + G_ta_sw * ( (lce_ta[i] / l_opt_ta) - l_off_ta_sw);

			if ( options->is_extra_ta_reflex() && (pa[i]->get_q() > TA_EXTRA_THRESHOLD) )
			{
				Stim[i][TA_MUSCLE] += TA_EXTRA_K * (pa[i]->get_q() - TA_EXTRA_THRESHOLD);
			}

			// GAS
			Stim[i][GAS_MUSCLE] = S_MIN;

			// VAS
			Stim[i][VAS_MUSCLE] = S0_vas;

			if (options->is_extra_knee())
			{
				Stim[i][VAS_MUSCLE] += G_vas_sw * ( (lce_vas[i] / l_opt_vas) - l_off_vas_sw);
			}

			// HAM
			Stim[i][HAM_MUSCLE] = S_MIN;

			// GLU
			Stim[i][GLU_MUSCLE] = S_MIN;

			// HFL
			Stim[i][HFL_MUSCLE] = S_MIN - G_ham_hfl * ( (lce_ham[i] / l_opt_ham) - l_off_ham);
		}
		// stance
		else
		{
			// SOL
			Stim[i][SOL_MUSCLE] = S_MIN + G_sol * (F_sol[i] / F_max_sol);

			// TA
			Stim[i][TA_MUSCLE] = S_MIN + G_ta_st * ( (lce_ta[i] / l_opt_ta) - l_off_ta_st);

			if(sw_st->get_nb_strikes() < 1) // first step
			{
				Stim[i][TA_MUSCLE] = S_MAX;
			}

			// GAS
			Stim[i][GAS_MUSCLE] = S_MIN;

			// VAS
			Stim[i][VAS_MUSCLE] = S0_vas + G_vas * (F_vas[i] / F_max_vas);

			if ((phi_k[i] < phi_off_pk) && (phip_k[i] < 0.0))
			{
				Stim[i][VAS_MUSCLE] = S0_vas;
			}

			if (tr_st->get_trailing_leg(i)) // trailing leg in double support
			{
				Stim[i][VAS_MUSCLE] = S0_vas;
			}

			// HAM
			Stim[i][HAM_MUSCLE] = S_MIN;
			
			if (PD_torso_1 > 0.0)
			{
				Stim[i][HAM_MUSCLE] += PD_torso_1 * k_bw * (F_foot[i] / WEIGHT_COMAN);
			}

			if (options->is_extra_ham_reflex() && tr_st->get_trailing_leg(i)) // trailing leg in double support (extra reflex)
			{
				Stim[i][HAM_MUSCLE] = S_ham_tr;
			}

			// GLU
			Stim[i][GLU_MUSCLE] = S_MIN;
			
			if(PD_torso_2 > 0.0)
			{
				Stim[i][GLU_MUSCLE] += PD_torso_2 * k_bw * (F_foot[i] / WEIGHT_COMAN);
			}
			
			if (tr_st->get_trailing_leg(i)) // trailing leg in double support
			{
				Stim[i][GLU_MUSCLE] = S_MIN;
			}

			// HFL
			Stim[i][HFL_MUSCLE] = S_MIN;

			if (PD_torso_1 < 0.0)
			{
				Stim[i][HFL_MUSCLE] -= PD_torso_1 * k_bw * (F_foot[i] / WEIGHT_COMAN);
			}
			
			if (tr_st->get_trailing_leg(i)) // trailing leg in double support
			{
				Stim[i][HFL_MUSCLE] = S_MAX;
			}
		}
	}
}

/*! \brief pitch muscles computation: minimal stimulations
 */
void StimGeyerCtrl::pitch_compute_min()
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
void StimGeyerCtrl::roll_compute_min()
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
void StimGeyerCtrl::yaw_compute_min()
{
	for (int i=0; i<NB_LEGS; i++)
	{
		Stim[i][HER_MUSCLE] = S_MIN;
		Stim[i][HIR_MUSCLE] = S_MIN;
	}
}
