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
    g_osc->set_gamma_A(1.55428473);
    g_osc->set_gamma_B(2.50439254);
    g_osc->set_gamma_C(2.81330277);

    g_osc->set_eta_A(5.49629366);  
    g_osc->set_eta_B(4.88655430);  
    g_osc->set_eta_C(5.66249720);  
    g_osc->set_eta_D(3.75511755);  
    g_osc->set_eta_E(3.68674669);  

    g_osc->set_P_tau(0.05228981 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 3.35785362);
    g_osc->set_k_HFLrun2( 4.47299716);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 5.55519366 );
	stim_wang->set_S0_sol_st( 0.01297286 );
	stim_wang->set_S0_ta_st( 0.04275306 );
	stim_wang->set_S0_gas_st( 0.01254764 );
	stim_wang->set_S0_vas_st( 0.47103684 );
	stim_wang->set_S0_ham_st( 0.09934494 );
	stim_wang->set_S0_rf_st( 0.14101836 );
	stim_wang->set_S0_glu_st( 0.02011663 );
	stim_wang->set_S0_hfl_st( 0.19237332 );
	stim_wang->set_S0_sol_sw( 0.01570168 );
	stim_wang->set_S0_ta_sw( 0.03647418 );
	stim_wang->set_S0_gas_sw( 0.02942171 );
	stim_wang->set_S0_vas_sw( 0.08270768 );
	stim_wang->set_S0_ham_sw( 0.08580918 );
	stim_wang->set_S0_rf_sw( 0.25609627 );
	stim_wang->set_S0_glu_sw( 0.01656156 );
	stim_wang->set_S0_hfl_sw( 0.04658742 );
	stim_wang->set_G_sol( 4.40641367 );
	stim_wang->set_G_sol_ta( 3.91356252 );
	stim_wang->set_G_gas( 4.76027703 );
	stim_wang->set_G_vas( 2.77889873 );
	stim_wang->set_G_ham( 0.68713984 );
	stim_wang->set_G_glu( 0.00167770 );
	stim_wang->set_G_ta_sw( 2.17262960 );
	stim_wang->set_G_ta_st( 2.55029374 );
	stim_wang->set_G_hfl( 2.70388151 );
	stim_wang->set_G_ham_hfl( 5.06352787 );
	stim_wang->set_l_off_ta_sw( 0.52656354 );
	stim_wang->set_l_off_ta_st( 0.68585852 );
	stim_wang->set_l_off_ham_hfl( 0.54264011 );
	stim_wang->set_l_off_hfl( 0.41670460 );
	stim_wang->set_K_ham( 3.01035031 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 4.54464156 );
	stim_wang->set_K_hfl( 2.42982471 );
	stim_wang->set_D_ham( 0.32750241 );
	stim_wang->set_D_glu( 0.05520323 );
	stim_wang->set_D_hfl( 0.36473302 );
	stim_wang->set_theta_ref( 0.02234267 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.73849588 );
	stim_wang->set_si_rf( 0.74499424 );
	stim_wang->set_si_glu( 0.70740866 );
	stim_wang->set_si_hfl( 0.52506255 );
	stim_wang->set_K_sp_vas( 0.49544572 );
	stim_wang->set_K_sp_glu( 4.27018793 );
	stim_wang->set_K_sp_hfl( 2.20738880 );
	stim_wang->set_D_sp_vas( 0.07957417 );
	stim_wang->set_D_sp_glu( 0.09761327 );
	stim_wang->set_D_sp_hfl( 0.04498949 );
	stim_wang->set_theta_k_ref( 0.20568565 );
	stim_wang->set_theta_h_ref0( 0.28988335 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.02241938 );
	stim_wang->set_d_si( 0.46695045 );
	stim_wang->set_k_THETA( 3.38823962 );
	stim_wang->set_k_theta( 9.88123746 );
	stim_wang->set_phi_off_pk( 0.10351874 );

	//init pos
	joints_init->set_T3( 0.49726246 );
	joints_init->set_T3_p( -0.06088822 );
	joints_init->set_R2( 0.13455446 );
	joints_init->set_R2_p( -1.41184301 );
	joints_init->set_r_sh_p( -0.24645724 );
	joints_init->set_r_hip( -0.46386077 );
	joints_init->set_r_hip_p( 2.74046968 );
	joints_init->set_r_knee( 0.16272597 );
	joints_init->set_r_knee_p( 2.17200721 );
	joints_init->set_r_ankle( -0.06856865 );
	joints_init->set_r_ankle_p( -1.91441294 );
	joints_init->set_l_hip( 0.05638992 );
	joints_init->set_l_hip_p( 1.64909184 );
	joints_init->set_l_knee( 0.76976232 );
	joints_init->set_l_knee_p( 1.04406057 );
	joints_init->set_l_ankle( 0.29834026 );
	joints_init->set_l_ankle_p( 2.36728010 );
}
