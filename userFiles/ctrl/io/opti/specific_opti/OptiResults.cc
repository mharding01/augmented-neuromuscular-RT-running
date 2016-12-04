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
    g_osc->set_gamma_A(1.14333323);
    g_osc->set_gamma_B(2.40468504);
    g_osc->set_gamma_C(2.89107084);

    g_osc->set_eta_A(5.50735421);  
    g_osc->set_eta_B(4.86900302);  
    g_osc->set_eta_C(6.12814786);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73317193);  
    g_osc->set_eta_E(3.67642072);  

    g_osc->set_P_tau(0.05710128 );    // TODO: Scaled down range by factor of min ~3, max 2 

    g_osc->set_k_HFLrun1( 3.60898382);
    g_osc->set_k_HFLrun2( 5.97781485);
    g_osc->set_k_HAMrun3( 3.88822619);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 0.200); //0.48881866 );    // TODO: cpg control set to overwrite just after oscillo init. @ .2 sec
	stim_wang->set_S0_sol_st( 0.02709393 );
	stim_wang->set_S0_ta_st( 0.01781570 );
	stim_wang->set_S0_gas_st( 0.02593461 );
	stim_wang->set_S0_vas_st( 0.32014068 );
	stim_wang->set_S0_ham_st( 0.18277660 );
	stim_wang->set_S0_rf_st( 0.12060820 );
	stim_wang->set_S0_glu_st( 0.23975573 );
	stim_wang->set_S0_hfl_st( 0.17528595 );
	stim_wang->set_S0_sol_sw( 0.01801547 );
	stim_wang->set_S0_ta_sw( 0.01398394 );
	stim_wang->set_S0_gas_sw( 0.06358098 );
	stim_wang->set_S0_vas_sw( 0.02881286 );
	stim_wang->set_S0_ham_sw( 0.01741034 );
	stim_wang->set_S0_rf_sw( 0.48835452 );
	stim_wang->set_S0_glu_sw( 0.02503411 );
	stim_wang->set_S0_hfl_sw( 0.01815202 );
	stim_wang->set_G_sol( 3.07813943 );
	stim_wang->set_G_sol_ta( 4.92394801 );
	stim_wang->set_G_gas( 7.22595528 );
	stim_wang->set_G_vas( 1.74563596 );
	stim_wang->set_G_ham( 0.08303629 );
	stim_wang->set_G_glu( 0.06954024 );
	stim_wang->set_G_ta_sw( 3.76905948 );
	stim_wang->set_G_ta_st( 2.24945036 );
	stim_wang->set_G_hfl( 1.18259564 );
	stim_wang->set_G_ham_hfl( 4.89501562 );
	stim_wang->set_l_off_ta_sw( 0.69501884 );
	stim_wang->set_l_off_ta_st( 0.59682676 );
	stim_wang->set_l_off_ham_hfl( 0.54092289 );
	stim_wang->set_l_off_hfl( 0.58112994 );
	stim_wang->set_K_ham( 3.68335115 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 8.68618195 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 3.91937018 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.24108599 );
	stim_wang->set_D_glu( 0.33112581 );
	stim_wang->set_D_hfl( 0.57212492 );
	stim_wang->set_theta_ref( 0.32861726 ); // TODO: trunk lean, realistic max i pi/6=30deg.
	stim_wang->set_si_vas( 0.66485555 );
	stim_wang->set_si_rf( 0.99821085 );
	stim_wang->set_si_glu( 0.40937975 );
	stim_wang->set_si_hfl( 0.31122778 );
	stim_wang->set_K_sp_vas( 1.93097061 );
	stim_wang->set_K_sp_glu( 4.11583124 );
	stim_wang->set_K_sp_hfl( 2.77931788 );
	stim_wang->set_D_sp_vas( 0.06074333 );
	stim_wang->set_D_sp_glu( 0.09118567 );
	stim_wang->set_D_sp_hfl( 0.02521028 );
	stim_wang->set_theta_k_ref( 0.19885233 );
	stim_wang->set_theta_h_ref0( 0.06532548 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.13110697 );
	stim_wang->set_d_si( 0.60151790 );
	stim_wang->set_k_THETA( 4.22811666 );
	stim_wang->set_k_theta( 3.18034374 );
	stim_wang->set_phi_off_pk( 0.10525001 );

	//init pos
	joints_init->set_T3( 0.55713652 );
	joints_init->set_T3_p( -0.01999569 );
	joints_init->set_R2( 0.09096396 );
	joints_init->set_R2_p( 3.20281272 );
	joints_init->set_r_sh_p( 2.15794728 );
	joints_init->set_r_hip( -0.82231284 );
	joints_init->set_r_hip_p( -3.64800956 );
	joints_init->set_r_knee( 0.35854359 );
	joints_init->set_r_knee_p( 0.48053136 );
	joints_init->set_r_ankle( 0.01540442 );
	joints_init->set_r_ankle_p( -3.49250795 );
	joints_init->set_l_hip( 0.13139186 );
	joints_init->set_l_hip_p( -3.29024422 );
	joints_init->set_l_knee( 0.74382874 );
	joints_init->set_l_knee_p( 0.13840313 );
	joints_init->set_l_ankle( 0.38577617 );
	joints_init->set_l_ankle_p( -0.67479602 );
}
