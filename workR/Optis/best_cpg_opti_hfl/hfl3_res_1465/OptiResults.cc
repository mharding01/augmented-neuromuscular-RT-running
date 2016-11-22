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
    g_osc->set_gamma_A(1.54575285);
    g_osc->set_gamma_B(2.41104391);
    g_osc->set_gamma_C(2.59592197);

    g_osc->set_eta_A(5.48606605);  
    g_osc->set_eta_B(4.82045137);  
    g_osc->set_eta_C(5.64747812);  
    g_osc->set_eta_D(3.74819398);  
    g_osc->set_eta_E(3.65073597);  

    g_osc->set_P_tau(0.05687071 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 3.70975647);
    g_osc->set_k_HFLrun2( 5.63038657);
    g_osc->set_k_HFLrun3( 3.08583186);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.01002830 );
	stim_wang->set_S0_ta_st( 0.01600132 );
	stim_wang->set_S0_gas_st( 0.02298934 );
	stim_wang->set_S0_vas_st( 0.48672398 );
	stim_wang->set_S0_ham_st( 0.01994810 );
	stim_wang->set_S0_rf_st( 0.17069730 );
	stim_wang->set_S0_glu_st( 0.28799031 );
	stim_wang->set_S0_hfl_st( 0.12587452 );
	stim_wang->set_S0_sol_sw( 0.01244392 );
	stim_wang->set_S0_ta_sw( 0.01712262 );
	stim_wang->set_S0_gas_sw( 0.11271817 );
	stim_wang->set_S0_vas_sw( 0.03364618 );
	stim_wang->set_S0_ham_sw( 0.05924651 );
	stim_wang->set_S0_rf_sw( 0.21358261 );
	stim_wang->set_S0_glu_sw( 0.02704641 );
	stim_wang->set_S0_hfl_sw( 0.01271326 );
	stim_wang->set_G_sol( 2.84780963 );
	stim_wang->set_G_sol_ta( 5.69288107 );
	stim_wang->set_G_gas( 3.84732186 );
	stim_wang->set_G_vas( 1.37807378 );
	stim_wang->set_G_ham( 1.14319268 );
	stim_wang->set_G_glu( 0.61236149 );
	stim_wang->set_G_ta_sw( 4.52940802 );
	stim_wang->set_G_ta_st( 3.61745149 );
	stim_wang->set_G_hfl( 2.68825790 );
	stim_wang->set_G_ham_hfl( 0.69882240 );
	stim_wang->set_l_off_ta_sw( 0.53616369 );
	stim_wang->set_l_off_ta_st( 0.58827799 );
	stim_wang->set_l_off_ham_hfl( 0.14777484 );
	stim_wang->set_l_off_hfl( 0.18567622 );
	stim_wang->set_K_ham( 3.64929195 );
	stim_wang->set_K_glu( 2.03973033 );
	stim_wang->set_K_hfl( 6.46869059 );
	stim_wang->set_D_ham( 0.43600463 );
	stim_wang->set_D_glu( 0.22987942 );
	stim_wang->set_D_hfl( 0.43454391 );
	stim_wang->set_theta_ref( 0.03822489 );
	stim_wang->set_si_vas( 0.87199556 );
	stim_wang->set_si_rf( 0.26561199 );
	stim_wang->set_si_glu( 0.92174250 );
	stim_wang->set_si_hfl( 0.30205449 );
	stim_wang->set_K_sp_vas( 2.12083647 );
	stim_wang->set_K_sp_glu( 2.51250324 );
	stim_wang->set_K_sp_hfl( 4.93792447 );
	stim_wang->set_D_sp_vas( 0.06281252 );
	stim_wang->set_D_sp_glu( 0.01012074 );
	stim_wang->set_D_sp_hfl( 0.03815217 );
	stim_wang->set_theta_k_ref( 0.22652702 );
	stim_wang->set_theta_h_ref0( 0.26994711 );
	stim_wang->set_d_sp( 0.00859860 );
	stim_wang->set_d_si( 0.49614396 );
	stim_wang->set_k_THETA( 1.81003198 );
	stim_wang->set_k_theta( 2.93446150 );
	stim_wang->set_phi_off_pk( 0.13167790 );

	//init pos
	joints_init->set_T3( 0.47779264 );
	joints_init->set_T3_p( -0.08715330 );
	joints_init->set_R2( 0.03333061 );
	joints_init->set_R2_p( 1.49163167 );
	joints_init->set_r_sh_p( 1.16847337 );
	joints_init->set_r_hip( -0.78137383 );
	joints_init->set_r_hip_p( 3.05066997 );
	joints_init->set_r_knee( 0.54613651 );
	joints_init->set_r_knee_p( -3.22715630 );
	joints_init->set_r_ankle( -0.02070949 );
	joints_init->set_r_ankle_p( 2.31919010 );
	joints_init->set_l_hip( 0.12330031 );
	joints_init->set_l_hip_p( 1.57547360 );
	joints_init->set_l_knee( 0.53981417 );
	joints_init->set_l_knee_p( -1.68727470 );
	joints_init->set_l_ankle( 0.41155853 );
	joints_init->set_l_ankle_p( -0.76421215 );
}
