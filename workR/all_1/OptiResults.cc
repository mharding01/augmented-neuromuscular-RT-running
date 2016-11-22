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
    g_osc->set_gamma_A(1.55562128);
    g_osc->set_gamma_B(2.39149780);
    g_osc->set_gamma_C(2.95178254);

    g_osc->set_eta_A(5.52057215);  
    g_osc->set_eta_B(4.89250780);  
    g_osc->set_eta_C(5.71468752);  
    g_osc->set_eta_D(3.75063166);  
    g_osc->set_eta_E(3.64762410);  

    g_osc->set_P_tau(0.04965805 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 4.49013598);
    g_osc->set_k_HFLrun2( 4.48522365);
    g_osc->set_k_HAMrun3( 3.00341761);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 5.51325970 );
	stim_wang->set_S0_sol_st( 0.04912714 );
	stim_wang->set_S0_ta_st( 0.02298887 );
	stim_wang->set_S0_gas_st( 0.02451966 );
	stim_wang->set_S0_vas_st( 0.63444752 );
	stim_wang->set_S0_ham_st( 0.08161838 );
	stim_wang->set_S0_rf_st( 0.29834706 );
	stim_wang->set_S0_glu_st( 0.14448499 );
	stim_wang->set_S0_hfl_st( 0.07402319 );
	stim_wang->set_S0_sol_sw( 0.01266374 );
	stim_wang->set_S0_ta_sw( 0.03914513 );
	stim_wang->set_S0_gas_sw( 0.02120764 );
	stim_wang->set_S0_vas_sw( 0.02773252 );
	stim_wang->set_S0_ham_sw( 0.06562289 );
	stim_wang->set_S0_rf_sw( 0.22482205 );
	stim_wang->set_S0_glu_sw( 0.03808044 );
	stim_wang->set_S0_hfl_sw( 0.04747252 );
	stim_wang->set_G_sol( 2.29290715 );
	stim_wang->set_G_sol_ta( 3.32211564 );
	stim_wang->set_G_gas( 16.90273802 );
	stim_wang->set_G_vas( 3.03498474 );
	stim_wang->set_G_ham( 1.09681248 );
	stim_wang->set_G_glu( 0.33200026 );
	stim_wang->set_G_ta_sw( 4.70970166 );
	stim_wang->set_G_ta_st( 2.85174268 );
	stim_wang->set_G_hfl( 2.07614987 );
	stim_wang->set_G_ham_hfl( 0.31879382 );
	stim_wang->set_l_off_ta_sw( 0.50646532 );
	stim_wang->set_l_off_ta_st( 0.67418211 );
	stim_wang->set_l_off_ham_hfl( 0.26755293 );
	stim_wang->set_l_off_hfl( 0.34415530 );
	stim_wang->set_K_ham( 4.07932228 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 12.00056904 );
	stim_wang->set_K_hfl( 12.97106140 );
	stim_wang->set_D_ham( 0.47011426 );
	stim_wang->set_D_glu( 0.16606311 );
	stim_wang->set_D_hfl( 0.30503601 );
	stim_wang->set_theta_ref( 0.06897563 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.76498421 );
	stim_wang->set_si_rf( 0.69376994 );
	stim_wang->set_si_glu( 0.88623664 );
	stim_wang->set_si_hfl( 0.76920902 );
	stim_wang->set_K_sp_vas( 2.77259909 );
	stim_wang->set_K_sp_glu( 4.67318822 );
	stim_wang->set_K_sp_hfl( 2.18159337 );
	stim_wang->set_D_sp_vas( 0.04674870 );
	stim_wang->set_D_sp_glu( 0.09063557 );
	stim_wang->set_D_sp_hfl( 0.05150494 );
	stim_wang->set_theta_k_ref( 0.16806338 );
	stim_wang->set_theta_h_ref0( 0.34247473 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.04959957 );
	stim_wang->set_d_si( 0.52427986 );
	stim_wang->set_k_THETA( 2.88493323 );
	stim_wang->set_k_theta( 5.17425637 );
	stim_wang->set_phi_off_pk( 0.13274202 );

	//init pos
	joints_init->set_T3( 0.49997884 );
	joints_init->set_T3_p( -0.45791812 );
	joints_init->set_R2( 0.09688429 );
	joints_init->set_R2_p( -0.67760675 );
	joints_init->set_r_sh_p( -0.42294104 );
	joints_init->set_r_hip( -0.56409221 );
	joints_init->set_r_hip_p( 2.93087171 );
	joints_init->set_r_knee( 0.62649155 );
	joints_init->set_r_knee_p( -0.51810435 );
	joints_init->set_r_ankle( -0.04902148 );
	joints_init->set_r_ankle_p( -3.43570587 );
	joints_init->set_l_hip( 0.18033625 );
	joints_init->set_l_hip_p( 0.13331469 );
	joints_init->set_l_knee( 0.51515221 );
	joints_init->set_l_knee_p( -3.57989599 );
	joints_init->set_l_ankle( 0.26213576 );
	joints_init->set_l_ankle_p( 0.91520728 );
}
