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
	/*
    g_osc->set_gamma_A(1.59030347);
    g_osc->set_gamma_B(2.39640033);
    g_osc->set_gamma_C(2.84350615);

    g_osc->set_eta_A(5.46761414);  
    g_osc->set_eta_B(4.88355275);  
    g_osc->set_eta_C(5.66314746);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.77916453);  
    g_osc->set_eta_E(3.70464392);  

    g_osc->set_P_tau(0.04676336 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 4.17021254);
    g_osc->set_k_HFLrun2( 9.16549638);
    g_osc->set_k_HAMrun3( 3.35982891);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.04144467 );//
	stim_wang->set_S0_ta_st( 0.03442493 );//
	stim_wang->set_S0_gas_st( 0.01144743 );//
	stim_wang->set_S0_vas_st( 0.27724696 );//
	stim_wang->set_S0_ham_st( 0.12211856 );//
	stim_wang->set_S0_rf_st( 0.03856261 );//
	stim_wang->set_S0_glu_st( 0.19368160 );//
	stim_wang->set_S0_hfl_st( 0.08483506 );//
	stim_wang->set_S0_sol_sw( 0.03201217 );//
	stim_wang->set_S0_ta_sw( 0.02341347 ); //
	stim_wang->set_S0_gas_sw( 0.05998866 );//
	stim_wang->set_S0_vas_sw( 0.07705747 );//
	stim_wang->set_S0_ham_sw( 0.08176211 );//
	stim_wang->set_S0_rf_sw( 0.29242350 ); //
	stim_wang->set_S0_glu_sw( 0.01436186 );//
	stim_wang->set_S0_hfl_sw( 0.03742033 );//
	stim_wang->set_G_sol( 3.20496133 );//
	stim_wang->set_G_sol_ta( 3.15520137 );//
	stim_wang->set_G_gas( 9.63917261 );//
	stim_wang->set_G_vas( 1.64607262 );//
	stim_wang->set_G_ham( 1.11480642 );//
	stim_wang->set_G_glu( 0.07998873 );//
	stim_wang->set_G_ta_sw( 2.71763514 );//
	stim_wang->set_G_ta_st( 3.70742092 );//
	stim_wang->set_G_hfl( 0.82179641 );//
	stim_wang->set_G_ham_hfl( 3.85110420 );//
	stim_wang->set_l_off_ta_sw( 0.63260836 );//
	stim_wang->set_l_off_ta_st( 0.50250814 );//
	stim_wang->set_l_off_ham_hfl( 0.00736140 );//
	stim_wang->set_l_off_hfl( 0.53396689 );//
	stim_wang->set_K_ham( 4.06202134 ); // // TODO: want less HAM in stance
	stim_wang->set_K_glu( 13.99331892 );// // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 5.05426992 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.30785626 );
	stim_wang->set_D_glu( 0.49261734 );
	stim_wang->set_D_hfl( 0.12076677 );//
	stim_wang->set_theta_ref( 0.06586455 );// // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.42227731 );//
	stim_wang->set_si_rf( 0.32872840 ); //
	stim_wang->set_si_glu( 0.65210655 );//
	stim_wang->set_si_hfl( 0.59414450 );//
	stim_wang->set_K_sp_vas( 0.13819734 );//
	stim_wang->set_K_sp_glu( 2.95331736 );//
	stim_wang->set_K_sp_hfl( 0.91644294 );//
	stim_wang->set_D_sp_vas( 0.03033318 );//
	stim_wang->set_D_sp_glu( 0.04191062 );//
	stim_wang->set_D_sp_hfl( 0.02470569 );//
	stim_wang->set_theta_k_ref( 0.11244578 );//
	stim_wang->set_theta_h_ref0( 0.16986034 );// // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.07115247 );//
	stim_wang->set_d_si( 0.37523836 ); //
	stim_wang->set_k_THETA( 5.15958768 );//
	stim_wang->set_k_theta( 6.68941640 );//
	stim_wang->set_phi_off_pk( 0.12978405 );//

	//init pos
	joints_init->set_T3( 0.50856120 );//
	joints_init->set_T3_p( -0.13185847 );//
	joints_init->set_R2( 0.19244314 );//
	joints_init->set_R2_p( -0.10833274 );//
	joints_init->set_r_sh_p( 0.69375329 );//
	joints_init->set_r_hip( -0.85556491 );//
	joints_init->set_r_hip_p( 4.98169968 );//
	joints_init->set_r_knee( 0.69257087 );//
	joints_init->set_r_knee_p( -0.97761252 );//
	joints_init->set_r_ankle( -0.02803309 );//
	joints_init->set_r_ankle_p( -0.12132465 );//j
	joints_init->set_l_hip( 0.14972826 );//
	joints_init->set_l_hip_p( -3.48415321 );//
	joints_init->set_l_knee( 0.47043970 );//
	joints_init->set_l_knee_p( -2.58307740 );//
	joints_init->set_l_ankle( 0.16824578 );//
	joints_init->set_l_ankle_p( 4.42269471 );//
	*/
}
