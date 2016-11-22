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
    g_osc->set_gamma_A(1.55716968);
    g_osc->set_gamma_B(2.50181579);
    g_osc->set_gamma_C(2.90701189);

    g_osc->set_eta_A(5.46207308);  
    g_osc->set_eta_B(4.84135592);  
    g_osc->set_eta_C(5.64758945);  
    g_osc->set_eta_D(3.76372044);  
    g_osc->set_eta_E(3.68982655);  

    g_osc->set_P_tau(0.05138131 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 5.71459516);
    g_osc->set_k_HFLrun2( 6.25575835);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 2.92044155 );
	stim_wang->set_S0_sol_st( 0.03215361 );
	stim_wang->set_S0_ta_st( 0.03869692 );
	stim_wang->set_S0_gas_st( 0.03003649 );
	stim_wang->set_S0_vas_st( 0.49933690 );
	stim_wang->set_S0_ham_st( 0.28014963 );
	stim_wang->set_S0_rf_st( 0.06574346 );
	stim_wang->set_S0_glu_st( 0.11285433 );
	stim_wang->set_S0_hfl_st( 0.31796960 );
	stim_wang->set_S0_sol_sw( 0.01574172 );
	stim_wang->set_S0_ta_sw( 0.01843768 );
	stim_wang->set_S0_gas_sw( 0.02564638 );
	stim_wang->set_S0_vas_sw( 0.05097068 );
	stim_wang->set_S0_ham_sw( 0.05193404 );
	stim_wang->set_S0_rf_sw( 0.28920646 );
	stim_wang->set_S0_glu_sw( 0.03685598 );
	stim_wang->set_S0_hfl_sw( 0.01793314 );
	stim_wang->set_G_sol( 2.52847605 );
	stim_wang->set_G_sol_ta( 5.69524709 );
	stim_wang->set_G_gas( 19.92208422 );
	stim_wang->set_G_vas( 1.05173639 );
	stim_wang->set_G_ham( 0.80453974 );
	stim_wang->set_G_glu( 0.49511888 );
	stim_wang->set_G_ta_sw( 4.21386979 );
	stim_wang->set_G_ta_st( 3.14586437 );
	stim_wang->set_G_hfl( 0.53131724 );
	stim_wang->set_G_ham_hfl( 8.15864695 );
	stim_wang->set_l_off_ta_sw( 0.57591613 );
	stim_wang->set_l_off_ta_st( 0.64997752 );
	stim_wang->set_l_off_ham_hfl( 0.66976967 );
	stim_wang->set_l_off_hfl( 0.29750273 );
	stim_wang->set_K_ham( 6.95623366 );
	stim_wang->set_K_glu( 10.99578164 );
	stim_wang->set_K_hfl( 6.93659067 );
	stim_wang->set_D_ham( 0.49992994 );
	stim_wang->set_D_glu( 0.13896644 );
	stim_wang->set_D_hfl( 0.43871501 );
	stim_wang->set_theta_ref( 0.04675575 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.54204888 );
	stim_wang->set_si_rf( 0.26401257 );
	stim_wang->set_si_glu( 0.90684877 );
	stim_wang->set_si_hfl( 0.74492850 );
	stim_wang->set_K_sp_vas( 3.93646662 );
	stim_wang->set_K_sp_glu( 0.85798506 );
	stim_wang->set_K_sp_hfl( 0.71582476 );
	stim_wang->set_D_sp_vas( 0.04029262 );
	stim_wang->set_D_sp_glu( 0.08579967 );
	stim_wang->set_D_sp_hfl( 0.08917151 );
	stim_wang->set_theta_k_ref( 0.10481513 );
	stim_wang->set_theta_h_ref0( 0.87899760 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( -0.03373470 );
	stim_wang->set_d_si( 0.59756799 );
	stim_wang->set_k_THETA( 3.74592037 );
	stim_wang->set_k_theta( 9.28579240 );
	stim_wang->set_phi_off_pk( 0.08417610 );

	//init pos
	joints_init->set_T3( 0.50822373 );
	joints_init->set_T3_p( -0.08020968 );
	joints_init->set_R2( 0.26431842 );
	joints_init->set_R2_p( -0.92604312 );
	joints_init->set_r_sh_p( -3.15059690 );
	joints_init->set_r_hip( -0.71110124 );
	joints_init->set_r_hip_p( 3.56111363 );
	joints_init->set_r_knee( 0.20967267 );
	joints_init->set_r_knee_p( 1.07723726 );
	joints_init->set_r_ankle( -0.05780951 );
	joints_init->set_r_ankle_p( 3.83270807 );
	joints_init->set_l_hip( -0.05616461 );
	joints_init->set_l_hip_p( 0.11613135 );
	joints_init->set_l_knee( 1.36000638 );
	joints_init->set_l_knee_p( 3.97809078 );
	joints_init->set_l_ankle( 0.34975101 );
	joints_init->set_l_ankle_p( 0.74570457 );
}
