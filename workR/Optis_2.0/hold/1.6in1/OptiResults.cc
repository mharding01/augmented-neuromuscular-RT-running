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
    g_osc->set_gamma_A(1.59347264);
    g_osc->set_gamma_B(2.45153750);
    g_osc->set_gamma_C(2.64187032);

    g_osc->set_eta_A(5.47390070);  
    g_osc->set_eta_B(4.80300807);  
    g_osc->set_eta_C(5.82844905);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.76019890);  
    g_osc->set_eta_E(3.67985920);  

    g_osc->set_P_tau(0.04614132 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 4.09441294);
    g_osc->set_k_HFLrun2( 5.06855865);
    g_osc->set_k_HAMrun( 4.49230369);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.01159407 );
	stim_wang->set_S0_ta_st( 0.02295275 );
	stim_wang->set_S0_gas_st( 0.01993975 );
	stim_wang->set_S0_vas_st( 0.34857885 );
	stim_wang->set_S0_ham_st( 0.27629355 );
	stim_wang->set_S0_rf_st( 0.17693013 );
	stim_wang->set_S0_glu_st( 0.07450415 );
	stim_wang->set_S0_hfl_st( 0.01487152 );
	stim_wang->set_S0_sol_sw( 0.01452059 );
	stim_wang->set_S0_ta_sw( 0.04136364 );
	stim_wang->set_S0_gas_sw( 0.09443023 );
	stim_wang->set_S0_vas_sw( 0.02920788 );
	stim_wang->set_S0_ham_sw( 0.07607386 );
	stim_wang->set_S0_rf_sw( 0.26409921 );
	stim_wang->set_S0_glu_sw( 0.03269991 );
	stim_wang->set_S0_hfl_sw( 0.02864639 );
	stim_wang->set_G_sol( 2.29461551 );
	stim_wang->set_G_sol_ta( 3.53066044 );
	stim_wang->set_G_gas( 5.43955167 );
	stim_wang->set_G_vas( 4.06938620 );
	stim_wang->set_G_ham( 0.96540640 );
	stim_wang->set_G_glu( 0.84082477 );
	stim_wang->set_G_ta_sw( 3.00924890 );
	stim_wang->set_G_ta_st( 1.58646132 );
	stim_wang->set_G_hfl( 1.18626361 );
	stim_wang->set_G_ham_hfl( 3.58423209 );
	stim_wang->set_l_off_ta_sw( 0.65896015 );
	stim_wang->set_l_off_ta_st( 0.54884404 );
	stim_wang->set_l_off_ham_hfl( 0.45579096 );
	stim_wang->set_l_off_hfl( 0.55900207 );
	stim_wang->set_K_ham( 1.85146565 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 13.08781353 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 2.25505118 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.16076146 );
	stim_wang->set_D_glu( 0.15672698 );
	stim_wang->set_D_hfl( 0.68838107 );
	stim_wang->set_theta_ref( 0.01882422 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.80406235 );
	stim_wang->set_si_rf( 0.04944915 );
	stim_wang->set_si_glu( 0.59692421 );
	stim_wang->set_si_hfl( 0.90901939 );
	stim_wang->set_K_sp_vas( 0.14354995 );
	stim_wang->set_K_sp_glu( 2.79379413 );
	stim_wang->set_K_sp_hfl( 3.08480893 );
	stim_wang->set_D_sp_vas( 0.02421477 );
	stim_wang->set_D_sp_glu( 0.05064783 );
	stim_wang->set_D_sp_hfl( 0.03567101 );
	stim_wang->set_theta_k_ref( 0.11093368 );
	stim_wang->set_theta_h_ref0( 0.20182283 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( 0.03238600 );
	stim_wang->set_d_si( 0.34067805 );
	stim_wang->set_k_THETA( 1.51368859 );
	stim_wang->set_k_theta( 7.72386645 );
	stim_wang->set_phi_off_pk( 0.11685187 );

	//init pos
	joints_init->set_T3( 0.51109681 );
	joints_init->set_T3_p( 0.24677900 );
	joints_init->set_R2( 0.10184367 );
	joints_init->set_R2_p( -0.82178786 );
	joints_init->set_r_sh_p( -1.31449076 );
	joints_init->set_r_hip( -0.69504564 );
	joints_init->set_r_hip_p( 3.59856812 );
	joints_init->set_r_knee( 0.64640667 );
	joints_init->set_r_knee_p( -0.07566438 );
	joints_init->set_r_ankle( 0.03535752 );
	joints_init->set_r_ankle_p( 4.31608752 );
	joints_init->set_l_hip( 0.11025396 );
	joints_init->set_l_hip_p( 0.73417263 );
	joints_init->set_l_knee( 1.67665764 );
	joints_init->set_l_knee_p( 3.12292182 );
	joints_init->set_l_ankle( 0.28095838 );
	joints_init->set_l_ankle_p( -3.94777584 );
}
