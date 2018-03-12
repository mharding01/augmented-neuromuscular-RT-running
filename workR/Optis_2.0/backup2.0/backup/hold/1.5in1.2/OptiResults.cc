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
    g_osc->set_gamma_A(1.56990621);
    g_osc->set_gamma_B(2.39385933);
    g_osc->set_gamma_C(2.97528401);

    g_osc->set_eta_A(5.48447338);  
    g_osc->set_eta_B(4.83108079);  
    g_osc->set_eta_C(5.68360419);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.75589060);  
    g_osc->set_eta_E(3.67915001);  

    g_osc->set_P_tau(0.04629644 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 2.56954913);
    g_osc->set_k_HFLrun2( 9.65862864);
    g_osc->set_k_HAMrun( 6.13780900);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.04678285 );
	stim_wang->set_S0_ta_st( 0.02899115 );
	stim_wang->set_S0_gas_st( 0.02680461 );
	stim_wang->set_S0_vas_st( 0.28578744 );
	stim_wang->set_S0_ham_st( 0.03148691 );
	stim_wang->set_S0_rf_st( 0.15867039 );
	stim_wang->set_S0_glu_st( 0.12466513 );
	stim_wang->set_S0_hfl_st( 0.06144631 );
	stim_wang->set_S0_sol_sw( 0.01848283 );
	stim_wang->set_S0_ta_sw( 0.02693546 );
	stim_wang->set_S0_gas_sw( 0.15084608 );
	stim_wang->set_S0_vas_sw( 0.05628207 );
	stim_wang->set_S0_ham_sw( 0.04072476 );
	stim_wang->set_S0_rf_sw( 0.28064547 );
	stim_wang->set_S0_glu_sw( 0.03507747 );
	stim_wang->set_S0_hfl_sw( 0.03261636 );
	stim_wang->set_G_sol( 1.70926580 );
	stim_wang->set_G_sol_ta( 2.25933284 );
	stim_wang->set_G_gas( 3.28599946 );
	stim_wang->set_G_vas( 1.84072957 );
	stim_wang->set_G_ham( 1.28672625 );
	stim_wang->set_G_glu( 1.18070909 );
	stim_wang->set_G_ta_sw( 3.33372854 );
	stim_wang->set_G_ta_st( 4.67973169 );
	stim_wang->set_G_hfl( 0.40062735 );
	stim_wang->set_G_ham_hfl( 1.38034397 );
	stim_wang->set_l_off_ta_sw( 0.64052075 );
	stim_wang->set_l_off_ta_st( 0.55866489 );
	stim_wang->set_l_off_ham_hfl( 0.08338940 );
	stim_wang->set_l_off_hfl( 0.38258374 );
	stim_wang->set_K_ham( 4.38550720 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 11.38193250 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 7.48224607 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.37615659 );
	stim_wang->set_D_glu( 0.14842360 );
	stim_wang->set_D_hfl( 0.22163786 );
	stim_wang->set_theta_ref( 0.09388319 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.52426975 );
	stim_wang->set_si_rf( 0.50759588 );
	stim_wang->set_si_glu( 0.26689854 );
	stim_wang->set_si_hfl( 0.25410880 );
	stim_wang->set_K_sp_vas( 0.81914847 );
	stim_wang->set_K_sp_glu( 3.70163900 );
	stim_wang->set_K_sp_hfl( 2.92821192 );
	stim_wang->set_D_sp_vas( 0.05457945 );
	stim_wang->set_D_sp_glu( 0.03410639 );
	stim_wang->set_D_sp_hfl( 0.07061881 );
	stim_wang->set_theta_k_ref( 0.28760183 );
	stim_wang->set_theta_h_ref0( 0.13607346 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.06693516 );
	stim_wang->set_d_si( 0.35356776 );
	stim_wang->set_k_THETA( 4.73276886 );
	stim_wang->set_k_theta( 8.55934086 );
	stim_wang->set_phi_off_pk( 0.17739057 );

	//init pos
	joints_init->set_T3( 0.52366052 );
	joints_init->set_T3_p( -0.16366136 );
	joints_init->set_R2( 0.03933181 );
	joints_init->set_R2_p( 0.64845873 );
	joints_init->set_r_sh_p( 0.06459492 );
	joints_init->set_r_hip( -0.46930059 );
	joints_init->set_r_hip_p( 1.69796411 );
	joints_init->set_r_knee( 0.27924002 );
	joints_init->set_r_knee_p( 2.30156445 );
	joints_init->set_r_ankle( -0.03814217 );
	joints_init->set_r_ankle_p( 2.81289127 );
	joints_init->set_l_hip( 0.08699595 );
	joints_init->set_l_hip_p( -4.04834756 );
	joints_init->set_l_knee( 0.91034122 );
	joints_init->set_l_knee_p( -3.19147574 );
	joints_init->set_l_ankle( 0.15679849 );
	joints_init->set_l_ankle_p( 3.00878691 );
}
