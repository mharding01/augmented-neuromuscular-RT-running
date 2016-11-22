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
    g_osc->set_gamma_A(1.52320082);
    g_osc->set_gamma_B(2.47035427);
    g_osc->set_gamma_C(2.99063266);

    g_osc->set_eta_A(5.52378569);  
    g_osc->set_eta_B(4.88389403);  
    g_osc->set_eta_C(5.67494696);  
    g_osc->set_eta_D(3.75183152);  
    g_osc->set_eta_E(3.69098638);  

    g_osc->set_P_tau(0.05485446 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 4.45622892);
    g_osc->set_k_HFLrun2( 3.36274822);
    g_osc->set_k_HFLrun3( 3.44810427);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.04100858 );
	stim_wang->set_S0_ta_st( 0.02404314 );
	stim_wang->set_S0_gas_st( 0.02820139 );
	stim_wang->set_S0_vas_st( 0.12225753 );
	stim_wang->set_S0_ham_st( 0.27440895 );
	stim_wang->set_S0_rf_st( 0.28580627 );
	stim_wang->set_S0_glu_st( 0.07110309 );
	stim_wang->set_S0_hfl_st( 0.17841966 );
	stim_wang->set_S0_sol_sw( 0.01179856 );
	stim_wang->set_S0_ta_sw( 0.02213212 );
	stim_wang->set_S0_gas_sw( 0.14187984 );
	stim_wang->set_S0_vas_sw( 0.05834280 );
	stim_wang->set_S0_ham_sw( 0.09844590 );
	stim_wang->set_S0_rf_sw( 0.36730537 );
	stim_wang->set_S0_glu_sw( 0.04736273 );
	stim_wang->set_S0_hfl_sw( 0.01518768 );
	stim_wang->set_G_sol( 2.72874636 );
	stim_wang->set_G_sol_ta( 5.42431832 );
	stim_wang->set_G_gas( 13.26210357 );
	stim_wang->set_G_vas( 3.37755215 );
	stim_wang->set_G_ham( 0.79497374 );
	stim_wang->set_G_glu( 0.06984378 );
	stim_wang->set_G_ta_sw( 4.60994634 );
	stim_wang->set_G_ta_st( 1.22658631 );
	stim_wang->set_G_hfl( 1.56560844 );
	stim_wang->set_G_ham_hfl( 6.30788886 );
	stim_wang->set_l_off_ta_sw( 0.61363795 );
	stim_wang->set_l_off_ta_st( 0.60575187 );
	stim_wang->set_l_off_ham_hfl( 0.54964889 );
	stim_wang->set_l_off_hfl( 0.00661504 );
	stim_wang->set_K_ham( 4.49774820 );
	stim_wang->set_K_glu( 11.38946017 );
	stim_wang->set_K_hfl( 4.68229152 );
	stim_wang->set_D_ham( 0.30570602 );
	stim_wang->set_D_glu( 0.36989146 );
	stim_wang->set_D_hfl( 0.25472776 );
	stim_wang->set_theta_ref( 0.06386951 );
	stim_wang->set_si_vas( 0.76237034 );
	stim_wang->set_si_rf( 0.07476094 );
	stim_wang->set_si_glu( 0.87225198 );
	stim_wang->set_si_hfl( 0.64656298 );
	stim_wang->set_K_sp_vas( 0.19348406 );
	stim_wang->set_K_sp_glu( 3.17787003 );
	stim_wang->set_K_sp_hfl( 2.53738762 );
	stim_wang->set_D_sp_vas( 0.02391881 );
	stim_wang->set_D_sp_glu( 0.04419354 );
	stim_wang->set_D_sp_hfl( 0.04060406 );
	stim_wang->set_theta_k_ref( 0.10372832 );
	stim_wang->set_theta_h_ref0( 0.34642088 );
	stim_wang->set_d_sp( 0.02379370 );
	stim_wang->set_d_si( 0.49347182 );
	stim_wang->set_k_THETA( 3.77240854 );
	stim_wang->set_k_theta( 3.29741283 );
	stim_wang->set_phi_off_pk( 0.07281024 );

	//init pos
	joints_init->set_T3( 0.50840828 );
	joints_init->set_T3_p( 0.43923204 );
	joints_init->set_R2( 0.10014263 );
	joints_init->set_R2_p( 0.87954727 );
	joints_init->set_r_sh_p( 1.55992521 );
	joints_init->set_r_hip( -0.65035066 );
	joints_init->set_r_hip_p( 3.36981332 );
	joints_init->set_r_knee( 0.06768688 );
	joints_init->set_r_knee_p( -4.07786713 );
	joints_init->set_r_ankle( 0.02349523 );
	joints_init->set_r_ankle_p( -3.76663700 );
	joints_init->set_l_hip( 0.15643639 );
	joints_init->set_l_hip_p( -0.00902199 );
	joints_init->set_l_knee( 1.19572606 );
	joints_init->set_l_knee_p( 2.26127195 );
	joints_init->set_l_ankle( 0.23484686 );
	joints_init->set_l_ankle_p( 3.43053840 );
}
