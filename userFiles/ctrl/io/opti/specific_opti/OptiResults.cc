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
    g_osc->set_gamma_A(1.59841828);
    g_osc->set_gamma_B(2.53495424);
    g_osc->set_gamma_C(2.85077996);

    g_osc->set_eta_A(5.48042911);  
    g_osc->set_eta_B(4.88008844);  
    g_osc->set_eta_C(5.87208327);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.77481912);  
    g_osc->set_eta_E(3.68370766);  

    g_osc->set_P_tau(0.04099378 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 2.70286746);
    g_osc->set_k_HFLrun2( 1.73092776);
    g_osc->set_k_HAMrun3( 5.90948878);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.01501363 );
	stim_wang->set_S0_ta_st( 0.04323499 );
	stim_wang->set_S0_gas_st( 0.03581910 );
	stim_wang->set_S0_vas_st( 0.25504045 );
	stim_wang->set_S0_ham_st( 0.28572141 );
	stim_wang->set_S0_rf_st( 0.07422153 );
	stim_wang->set_S0_glu_st( 0.19266169 );
	stim_wang->set_S0_hfl_st( 0.03639353 );
	stim_wang->set_S0_sol_sw( 0.01926744 );
	stim_wang->set_S0_ta_sw( 0.03646658 );
	stim_wang->set_S0_gas_sw( 0.10328166 );
	stim_wang->set_S0_vas_sw( 0.08797712 );
	stim_wang->set_S0_ham_sw( 0.07113375 );
	stim_wang->set_S0_rf_sw( 0.36318710 );
	stim_wang->set_S0_glu_sw( 0.03544825 );
	stim_wang->set_S0_hfl_sw( 0.03542112 );
	stim_wang->set_G_sol( 2.94102259 );
	stim_wang->set_G_sol_ta( 4.08447278 );
	stim_wang->set_G_gas( 10.93986801 );
	stim_wang->set_G_vas( 2.01629450 );
	stim_wang->set_G_ham( 0.22047074 );
	stim_wang->set_G_glu( 0.21197331 );
	stim_wang->set_G_ta_sw( 3.00335609 );
	stim_wang->set_G_ta_st( 1.18909418 );
	stim_wang->set_G_hfl( 0.67344844 );
	stim_wang->set_G_ham_hfl( 2.44081960 );
	stim_wang->set_l_off_ta_sw( 0.50165296 );
	stim_wang->set_l_off_ta_st( 0.63073457 );
	stim_wang->set_l_off_ham_hfl( 0.09993807 );
	stim_wang->set_l_off_hfl( 0.60277484 );
	stim_wang->set_K_ham( 2.26061147 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 8.88204609 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 5.55503960 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.34832090 );
	stim_wang->set_D_glu( 0.27652610 );
	stim_wang->set_D_hfl( 0.59302621 );
	stim_wang->set_theta_ref( 0.02132347 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.75524519 );
	stim_wang->set_si_rf( 0.51132049 );
	stim_wang->set_si_glu( 0.58379131 );
	stim_wang->set_si_hfl( 0.29230433 );
	stim_wang->set_K_sp_vas( 3.01144506 );
	stim_wang->set_K_sp_glu( 2.86262348 );
	stim_wang->set_K_sp_hfl( 3.40413750 );
	stim_wang->set_D_sp_vas( 0.03633052 );
	stim_wang->set_D_sp_glu( 0.02009767 );
	stim_wang->set_D_sp_hfl( 0.05728372 );
	stim_wang->set_theta_k_ref( 0.18843177 );
	stim_wang->set_theta_h_ref0( 0.42435419 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( 0.01496919 );
	stim_wang->set_d_si( 0.36271278 );
	stim_wang->set_k_THETA( 5.06402856 );
	stim_wang->set_k_theta( 12.43603099 );
	stim_wang->set_phi_off_pk( 0.16669310 );

	//init pos
	joints_init->set_T3( 0.50434594 );
	joints_init->set_T3_p( 0.23357081 );
	joints_init->set_R2( 0.27936242 );
	joints_init->set_R2_p( 0.42896828 );
	joints_init->set_r_sh_p( -3.51438050 );
	joints_init->set_r_hip( -0.67329057 );
	joints_init->set_r_hip_p( -4.38830750 );
	joints_init->set_r_knee( 0.15326265 );
	joints_init->set_r_knee_p( 4.18429017 );
	joints_init->set_r_ankle( -0.03992061 );
	joints_init->set_r_ankle_p( -3.93996825 );
	joints_init->set_l_hip( 0.24374040 );
	joints_init->set_l_hip_p( -0.82022714 );
	joints_init->set_l_knee( 1.52373490 );
	joints_init->set_l_knee_p( -0.88198876 );
	joints_init->set_l_ankle( 0.17743332 );
	joints_init->set_l_ankle_p( -3.16605273 );
}
