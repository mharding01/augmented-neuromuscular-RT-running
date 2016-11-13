#include "OptiGeneration.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OptiGeneration::OptiGeneration()
{

}

/*! \brief destructor
 */
OptiGeneration::~OptiGeneration()
{

}

/*! \brief set optimization parameters
 */
void OptiGeneration::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Oscillator fields
    g_osc->set_gamma_A(optiParams[0]);
    g_osc->set_gamma_B(optiParams[1]);
    g_osc->set_gamma_C(optiParams[2]);

    g_osc->set_eta_A(optiParams[3]);  
    g_osc->set_eta_B(optiParams[4]);  
    g_osc->set_eta_C(optiParams[5]);  
    g_osc->set_eta_D(optiParams[6]);  
    g_osc->set_eta_E(optiParams[7]);  

    g_osc->set_P_tau(optiParams[8] );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( optiParams[9]);
    g_osc->set_k_HFLrun2( optiParams[10]);
    g_osc->set_k_HFLrun3( optiParams[11]);

	// Wang stimulations
	stim_wang->set_S0_sol_st( optiParams[12] );
	stim_wang->set_S0_ta_st( optiParams[13] );
	stim_wang->set_S0_gas_st( optiParams[14] );
	stim_wang->set_S0_vas_st( optiParams[15] );
	stim_wang->set_S0_ham_st( optiParams[16] );
	stim_wang->set_S0_rf_st( optiParams[17] );
	stim_wang->set_S0_glu_st( optiParams[18] );
	stim_wang->set_S0_hfl_st( optiParams[19] );
	stim_wang->set_S0_sol_sw( optiParams[20] );
	stim_wang->set_S0_ta_sw( optiParams[21] );
	stim_wang->set_S0_gas_sw( optiParams[22] );
	stim_wang->set_S0_vas_sw( optiParams[23] );
	stim_wang->set_S0_ham_sw( optiParams[24] );
	stim_wang->set_S0_rf_sw( optiParams[25] );
	stim_wang->set_S0_glu_sw( optiParams[26] );
	stim_wang->set_S0_hfl_sw( optiParams[27] );
	stim_wang->set_G_sol( optiParams[28] );
	stim_wang->set_G_sol_ta( optiParams[29] );
	stim_wang->set_G_gas( optiParams[30] );
	stim_wang->set_G_vas( optiParams[31] );
	stim_wang->set_G_ham( optiParams[32] );
	stim_wang->set_G_glu( optiParams[33] );
	stim_wang->set_G_ta_sw( optiParams[34] );
	stim_wang->set_G_ta_st( optiParams[35] );
	stim_wang->set_G_hfl( optiParams[36] );
	stim_wang->set_G_ham_hfl( optiParams[37] );
	stim_wang->set_l_off_ta_sw( optiParams[38] );
	stim_wang->set_l_off_ta_st( optiParams[39] );
	stim_wang->set_l_off_ham_hfl( optiParams[40] );
	stim_wang->set_l_off_hfl( optiParams[41] );
	stim_wang->set_K_ham( optiParams[42] );
	stim_wang->set_K_glu( optiParams[43] );
	stim_wang->set_K_hfl( optiParams[44] );
	stim_wang->set_D_ham( optiParams[45] );
	stim_wang->set_D_glu( optiParams[46] );
	stim_wang->set_D_hfl( optiParams[47] );
	stim_wang->set_theta_ref( optiParams[48] );
	stim_wang->set_si_vas( optiParams[49] );
	stim_wang->set_si_rf( optiParams[50] );
	stim_wang->set_si_glu( optiParams[51] );
	stim_wang->set_si_hfl( optiParams[52] );
	stim_wang->set_K_sp_vas( optiParams[53] );
	stim_wang->set_K_sp_glu( optiParams[54] );
	stim_wang->set_K_sp_hfl( optiParams[55] );
	stim_wang->set_D_sp_vas( optiParams[56] );
	stim_wang->set_D_sp_glu( optiParams[57] );
	stim_wang->set_D_sp_hfl( optiParams[58] );
	stim_wang->set_theta_k_ref( optiParams[59] );
	stim_wang->set_theta_h_ref0( optiParams[60] );
	stim_wang->set_d_sp( optiParams[61] );
	stim_wang->set_d_si( optiParams[62] );
	stim_wang->set_k_THETA( optiParams[63] );
	stim_wang->set_k_theta( optiParams[64] );
	stim_wang->set_phi_off_pk( optiParams[65] );

	//init pos
	joints_init->set_T3( optiParams[66] );
	joints_init->set_T3_p( optiParams[67] );
	joints_init->set_R2( optiParams[68] );
	joints_init->set_R2_p( optiParams[69] );
	joints_init->set_r_sh_p( optiParams[70] );
	joints_init->set_r_hip( optiParams[71] );
	joints_init->set_r_hip_p( optiParams[72] );
	joints_init->set_r_knee( optiParams[73] );
	joints_init->set_r_knee_p( optiParams[74] );
	joints_init->set_r_ankle( optiParams[75] );
	joints_init->set_r_ankle_p( optiParams[76] );
	joints_init->set_l_hip( optiParams[77] );
	joints_init->set_l_hip_p( optiParams[78] );
	joints_init->set_l_knee( optiParams[79] );
	joints_init->set_l_knee_p( optiParams[80] );
	joints_init->set_l_ankle( optiParams[81] );
	joints_init->set_l_ankle_p( optiParams[82] );
}
