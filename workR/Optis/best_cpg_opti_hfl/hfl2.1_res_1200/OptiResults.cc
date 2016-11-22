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
    g_osc->set_gamma_A(1.56031323);
    g_osc->set_gamma_B(2.51047210);
    g_osc->set_gamma_C(2.83530806);

    g_osc->set_eta_A(5.46685077);  
    g_osc->set_eta_B(4.85791283);  
    g_osc->set_eta_C(5.67273560);  
    g_osc->set_eta_D(3.75479794);  
    g_osc->set_eta_E(3.69846791);  

    g_osc->set_P_tau(0.05261852 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 5.85794909);
    g_osc->set_k_HFLrun2( 6.82742058);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 1.90902048 );
	stim_wang->set_S0_sol_st( 0.03759074 );
	stim_wang->set_S0_ta_st( 0.03465263 );
	stim_wang->set_S0_gas_st( 0.04869250 );
	stim_wang->set_S0_vas_st( 0.51253837 );
	stim_wang->set_S0_ham_st( 0.17396502 );
	stim_wang->set_S0_rf_st( 0.23919273 );
	stim_wang->set_S0_glu_st( 0.10011257 );
	stim_wang->set_S0_hfl_st( 0.04872497 );
	stim_wang->set_S0_sol_sw( 0.02062448 );
	stim_wang->set_S0_ta_sw( 0.02122153 );
	stim_wang->set_S0_gas_sw( 0.03310280 );
	stim_wang->set_S0_vas_sw( 0.09052949 );
	stim_wang->set_S0_ham_sw( 0.05026565 );
	stim_wang->set_S0_rf_sw( 0.28180832 );
	stim_wang->set_S0_glu_sw( 0.04049157 );
	stim_wang->set_S0_hfl_sw( 0.03153666 );
	stim_wang->set_G_sol( 2.00535371 );
	stim_wang->set_G_sol_ta( 4.35192675 );
	stim_wang->set_G_gas( 12.34244941 );
	stim_wang->set_G_vas( 2.63772222 );
	stim_wang->set_G_ham( 2.29782437 );
	stim_wang->set_G_glu( 0.38768646 );
	stim_wang->set_G_ta_sw( 3.99743989 );
	stim_wang->set_G_ta_st( 3.63171239 );
	stim_wang->set_G_hfl( 2.27676311 );
	stim_wang->set_G_ham_hfl( 1.17957229 );
	stim_wang->set_l_off_ta_sw( 0.57912193 );
	stim_wang->set_l_off_ta_st( 0.73051924 );
	stim_wang->set_l_off_ham_hfl( 0.41583080 );
	stim_wang->set_l_off_hfl( 0.33736411 );
	stim_wang->set_K_ham( 4.29178070 );
	stim_wang->set_K_glu( 11.67281636 );
	stim_wang->set_K_hfl( 7.39183487 );
	stim_wang->set_D_ham( 0.31145554 );
	stim_wang->set_D_glu( 0.23859221 );
	stim_wang->set_D_hfl( 0.39203575 );
	stim_wang->set_theta_ref( 0.03163433 );
	stim_wang->set_si_vas( 0.88750683 );
	stim_wang->set_si_rf( 0.70692158 );
	stim_wang->set_si_glu( 0.77111676 );
	stim_wang->set_si_hfl( 0.90223598 );
	stim_wang->set_K_sp_vas( 0.95025632 );
	stim_wang->set_K_sp_glu( 3.56373532 );
	stim_wang->set_K_sp_hfl( 1.65599999 );
	stim_wang->set_D_sp_vas( 0.08340740 );
	stim_wang->set_D_sp_glu( 0.04787745 );
	stim_wang->set_D_sp_hfl( 0.04854313 );
	stim_wang->set_theta_k_ref( 0.09599978 );
	stim_wang->set_theta_h_ref0( 0.50739069 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.04400011 );
	stim_wang->set_d_si( 0.51097020 );
	stim_wang->set_k_THETA( 4.00431532 );
	stim_wang->set_k_theta( 6.30309475 );
	stim_wang->set_phi_off_pk( 0.10441704 );

	//init pos
	joints_init->set_T3( 0.53137367 );
	joints_init->set_T3_p( 0.14958633 );
	joints_init->set_R2( 0.09599003 );
	joints_init->set_R2_p( 0.57515908 );
	joints_init->set_r_sh_p( 2.73285026 );
	joints_init->set_r_hip( -0.79987200 );
	joints_init->set_r_hip_p( -0.23136247 );
	joints_init->set_r_knee( 0.44068270 );
	joints_init->set_r_knee_p( 2.46340384 );
	joints_init->set_r_ankle( -0.02344828 );
	joints_init->set_r_ankle_p( 1.89374665 );
	joints_init->set_l_hip( 0.03207123 );
	joints_init->set_l_hip_p( 2.89636381 );
	joints_init->set_l_knee( 0.89386932 );
	joints_init->set_l_knee_p( 0.12554261 );
	joints_init->set_l_ankle( 0.37999125 );
	joints_init->set_l_ankle_p( -2.21260810 );
}
