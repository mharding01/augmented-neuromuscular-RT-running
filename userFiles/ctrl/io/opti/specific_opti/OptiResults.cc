#include "OptiResults.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OptiResults::OptiResults()
{

}

/*! \brief destructor
 */
OptiResults::~OptiResults()
{

}

/*! \brief set optimization parameters
 */
void OptiResults::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Oscillator fields
    g_osc->set_gamma_A(1.56979859);
    g_osc->set_gamma_B(2.49723515);
    g_osc->set_gamma_C(2.69045600);

    g_osc->set_eta_A(5.48543727);  
    g_osc->set_eta_B(4.85188411);  
    g_osc->set_eta_C(5.94055309);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.75602226);  
    g_osc->set_eta_E(3.66501723);  

    g_osc->set_P_tau(0.04829689 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 1.97013575);
    g_osc->set_k_HFLrun2( 4.94740384);
    g_osc->set_k_HAMrun3( 3.28465598);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.02194817 );
	stim_wang->set_S0_ta_st( 0.02562719 );
	stim_wang->set_S0_gas_st( 0.01810459 );
	stim_wang->set_S0_vas_st( 0.44042045 );
	stim_wang->set_S0_ham_st( 0.20237877 );
	stim_wang->set_S0_rf_st( 0.10120247 );
	stim_wang->set_S0_glu_st( 0.29770641 );
	stim_wang->set_S0_hfl_st( 0.18072312 );
	stim_wang->set_S0_sol_sw( 0.02209875 );
	stim_wang->set_S0_ta_sw( 0.02128742 );
	stim_wang->set_S0_gas_sw( 0.16021302 );
	stim_wang->set_S0_vas_sw( 0.02544146 );
	stim_wang->set_S0_ham_sw( 0.09626436 );
	stim_wang->set_S0_rf_sw( 0.13831479 );
	stim_wang->set_S0_glu_sw( 0.02297107 );
	stim_wang->set_S0_hfl_sw( 0.03559792 );
	stim_wang->set_G_sol( 3.86143565 );
	stim_wang->set_G_sol_ta( 2.23527349 );
	stim_wang->set_G_gas( 16.12240268 );
	stim_wang->set_G_vas( 1.31822437 );
	stim_wang->set_G_ham( 1.50051293 );
	stim_wang->set_G_glu( 0.33246666 );
	stim_wang->set_G_ta_sw( 2.92715246 );
	stim_wang->set_G_ta_st( 3.42944165 );
	stim_wang->set_G_hfl( 1.67669726 );
	stim_wang->set_G_ham_hfl( 3.80544781 );
	stim_wang->set_l_off_ta_sw( 0.55343540 );
	stim_wang->set_l_off_ta_st( 0.64861688 );
	stim_wang->set_l_off_ham_hfl( 0.48237644 );
	stim_wang->set_l_off_hfl( 0.28690527 );
	stim_wang->set_K_ham( 4.91877910 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 7.92362812 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.90032020 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.09565679 );
	stim_wang->set_D_glu( 0.31915933 );
	stim_wang->set_D_hfl( 0.19707590 );
	stim_wang->set_theta_ref( 0.02358341 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.90530405 );
	stim_wang->set_si_rf( 0.47380585 );
	stim_wang->set_si_glu( 0.59193452 );
	stim_wang->set_si_hfl( 0.09751235 );
	stim_wang->set_K_sp_vas( 3.83015083 );
	stim_wang->set_K_sp_glu( 0.63644002 );
	stim_wang->set_K_sp_hfl( 2.46860713 );
	stim_wang->set_D_sp_vas( 0.04409413 );
	stim_wang->set_D_sp_glu( 0.03368846 );
	stim_wang->set_D_sp_hfl( 0.05510572 );
	stim_wang->set_theta_k_ref( 0.08557284 );
	stim_wang->set_theta_h_ref0( 0.64557474 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.16744413 );
	stim_wang->set_d_si( 0.31075125 );
	stim_wang->set_k_THETA( 2.77354147 );
	stim_wang->set_k_theta( 10.53640192 );
	stim_wang->set_phi_off_pk( 0.08459034 );

	//init pos
	joints_init->set_T3( 0.50150535 );
	joints_init->set_T3_p( 0.11369304 );
	joints_init->set_R2( 0.17464433 );
	joints_init->set_R2_p( 1.02029611 );
	joints_init->set_r_sh_p( -2.24530040 );
	joints_init->set_r_hip( -0.68812954 );
	joints_init->set_r_hip_p( 4.78889217 );
	joints_init->set_r_knee( 0.44390555 );
	joints_init->set_r_knee_p( 0.82832996 );
	joints_init->set_r_ankle( 0.05231890 );
	joints_init->set_r_ankle_p( 2.04323971 );
	joints_init->set_l_hip( -0.03572839 );
	joints_init->set_l_hip_p( -2.92095395 );
	joints_init->set_l_knee( 1.03811341 );
	joints_init->set_l_knee_p( -1.26551026 );
	joints_init->set_l_ankle( 0.29137364 );
	joints_init->set_l_ankle_p( 4.57176218 );
}
