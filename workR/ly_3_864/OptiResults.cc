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
    g_osc->set_gamma_A(1.56461583);
    g_osc->set_gamma_B(2.49714245);
    g_osc->set_gamma_C(2.58197305);

    g_osc->set_eta_A(5.45370511);  
    g_osc->set_eta_B(4.87768067);  
    g_osc->set_eta_C(5.67042748);  
    g_osc->set_eta_D(3.73905285);  
    g_osc->set_eta_E(3.66498983);  

    g_osc->set_P_tau(0.05742989 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 3.33441881);
    g_osc->set_k_HFLrun2( 5.05866222);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 5.38847553 );
	stim_wang->set_S0_sol_st( 0.01840597 );
	stim_wang->set_S0_ta_st( 0.02099010 );
	stim_wang->set_S0_gas_st( 0.01922406 );
	stim_wang->set_S0_vas_st( 0.46293638 );
	stim_wang->set_S0_ham_st( 0.28813106 );
	stim_wang->set_S0_rf_st( 0.03635220 );
	stim_wang->set_S0_glu_st( 0.18666987 );
	stim_wang->set_S0_hfl_st( 0.23977543 );
	stim_wang->set_S0_sol_sw( 0.02993653 );
	stim_wang->set_S0_ta_sw( 0.04310548 );
	stim_wang->set_S0_gas_sw( 0.17052732 );
	stim_wang->set_S0_vas_sw( 0.07533518 );
	stim_wang->set_S0_ham_sw( 0.02868629 );
	stim_wang->set_S0_rf_sw( 0.45470351 );
	stim_wang->set_S0_glu_sw( 0.03555566 );
	stim_wang->set_S0_hfl_sw( 0.02214042 );
	stim_wang->set_G_sol( 3.56140144 );
	stim_wang->set_G_sol_ta( 1.58991440 );
	stim_wang->set_G_gas( 10.76640789 );
	stim_wang->set_G_vas( 4.69634393 );
	stim_wang->set_G_ham( 0.45886068 );
	stim_wang->set_G_glu( 0.24914361 );
	stim_wang->set_G_ta_sw( 3.11998624 );
	stim_wang->set_G_ta_st( 2.05225989 );
	stim_wang->set_G_hfl( 2.74717328 );
	stim_wang->set_G_ham_hfl( 4.34827933 );
	stim_wang->set_l_off_ta_sw( 0.51547415 );
	stim_wang->set_l_off_ta_st( 0.75072890 );
	stim_wang->set_l_off_ham_hfl( 0.28804334 );
	stim_wang->set_l_off_hfl( 0.79675755 );
	stim_wang->set_K_ham( 6.85000540 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 7.06450993 );
	stim_wang->set_K_hfl( 1.26126379 );
	stim_wang->set_D_ham( 0.03238919 );
	stim_wang->set_D_glu( 0.41359094 );
	stim_wang->set_D_hfl( 0.49778275 );
	stim_wang->set_theta_ref( 0.06958741 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.96869778 );
	stim_wang->set_si_rf( 0.46046203 );
	stim_wang->set_si_glu( 0.84865994 );
	stim_wang->set_si_hfl( 0.50079465 );
	stim_wang->set_K_sp_vas( 0.72582108 );
	stim_wang->set_K_sp_glu( 2.30892716 );
	stim_wang->set_K_sp_hfl( 2.53004019 );
	stim_wang->set_D_sp_vas( 0.04550266 );
	stim_wang->set_D_sp_glu( 0.04817944 );
	stim_wang->set_D_sp_hfl( 0.08898658 );
	stim_wang->set_theta_k_ref( 0.14439216 );
	stim_wang->set_theta_h_ref0( 0.16644183 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( -0.06940489 );
	stim_wang->set_d_si( 0.51112059 );
	stim_wang->set_k_THETA( 4.43266284 );
	stim_wang->set_k_theta( 6.79164663 );
	stim_wang->set_phi_off_pk( 0.08129644 );

	//init pos
	joints_init->set_T3( 0.53570510 );
	joints_init->set_T3_p( -0.49972032 );
	joints_init->set_R2( 0.09885094 );
	joints_init->set_R2_p( -0.92787394 );
	joints_init->set_r_sh_p( 3.13666870 );
	joints_init->set_r_hip( -0.50983945 );
	joints_init->set_r_hip_p( 3.97421296 );
	joints_init->set_r_knee( 0.38350974 );
	joints_init->set_r_knee_p( -2.32338023 );
	joints_init->set_r_ankle( -0.04183395 );
	joints_init->set_r_ankle_p( 2.27628052 );
	joints_init->set_l_hip( 0.01127655 );
	joints_init->set_l_hip_p( -4.84977411 );
	joints_init->set_l_knee( 0.93703623 );
	joints_init->set_l_knee_p( -3.92083535 );
	joints_init->set_l_ankle( 0.37013777 );
	joints_init->set_l_ankle_p( -4.61854315 );
}
