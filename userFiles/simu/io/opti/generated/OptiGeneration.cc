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

    g_osc->set_beta_A(optiParams[3]);
    g_osc->set_beta_B(optiParams[4]);
    g_osc->set_beta_C(optiParams[5]);

    g_osc->set_eta_A(optiParams[6]);  
    g_osc->set_eta_B(optiParams[7]);  
    g_osc->set_eta_C(optiParams[8]);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(optiParams[9]);  
    g_osc->set_eta_E(optiParams[10]);

    g_osc->set_eta_F(optiParams[11]);
    g_osc->set_eta_G(optiParams[12]); 

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(optiParams[13]); // 0.0609
    g_osc->set_P_theta_hip(optiParams[14]); // 0.1616
    g_osc->set_P_k_theta(optiParams[15]); // 8.6158
    
    // Linear
    g_osc->set_P_k_HFLrun1(optiParams[16]); // 5.1025
    g_osc->set_p_k_HFLrun1(optiParams[17]); // -3.1544

    g_osc->set_P_G_SOL_TA(optiParams[18]); // 5.3225
    g_osc->set_p_G_SOL_TA(optiParams[19]); // 9.0977

    g_osc->set_P_G_GAS(optiParams[20]); // 7.5656
    g_osc->set_p_G_GAS(optiParams[21]); // -13.7575

    g_osc->set_P_G_VAS(optiParams[22]); // 1.6058
    g_osc->set_p_G_VAS(optiParams[23]); // 2.0708
    
    // Quadratic
    g_osc->set_P_tau(optiParams[24]); // 0.0465
    g_osc->set_p_tau(optiParams[25]); // -.0168
    g_osc->set_p2_tau(optiParams[26]);// -.0277
    
    g_osc->set_P_k_HFLrun2(optiParams[27]); // 5.8581
    g_osc->set_p_k_HFLrun2(optiParams[28]);// 29.9698
    g_osc->set_p2_k_HFLrun2(optiParams[29]);//84.8007
    
    g_osc->set_P_k_HAMrun(optiParams[30]); // 3.8158
    g_osc->set_p_k_HAMrun(optiParams[31]); // 8.7087
    g_osc->set_p2_k_HAMrun(optiParams[32]);//34.0513

    g_osc->set_P_G_SOL(optiParams[33]); // 2.9365
    g_osc->set_p_G_SOL(optiParams[34]); // 3.4425
    g_osc->set_p2_G_SOL(optiParams[35]); // 13.5518

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( optiParams[36] );
	stim_wang->set_S0_ta_st( optiParams[37] );
	stim_wang->set_S0_gas_st( optiParams[38] );
	stim_wang->set_S0_vas_st( optiParams[39] );
	stim_wang->set_S0_ham_st( optiParams[40] );
	stim_wang->set_S0_rf_st( optiParams[41] );
	stim_wang->set_S0_glu_st( optiParams[42] );
	stim_wang->set_S0_hfl_st( optiParams[43] );
	stim_wang->set_S0_sol_sw( optiParams[44] );
	stim_wang->set_S0_ta_sw( optiParams[45] );
	stim_wang->set_S0_gas_sw( optiParams[46] );
	stim_wang->set_S0_vas_sw( optiParams[47] );
	stim_wang->set_S0_ham_sw( optiParams[48] );
	stim_wang->set_S0_rf_sw( optiParams[49] );
	stim_wang->set_S0_glu_sw( optiParams[50] );
	stim_wang->set_S0_hfl_sw( optiParams[51] );
	stim_wang->set_G_ham( optiParams[52] );
	stim_wang->set_G_glu( optiParams[53] );
	stim_wang->set_G_ta_sw( optiParams[54] );
	stim_wang->set_G_ta_st( optiParams[55] );
	stim_wang->set_l_off_ta_sw( optiParams[56] );
	stim_wang->set_l_off_ta_st( optiParams[57] );
	stim_wang->set_K_ham( optiParams[58] );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( optiParams[59] ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( optiParams[60] ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( optiParams[61] );
	stim_wang->set_D_glu( optiParams[62] );
	stim_wang->set_D_hfl( optiParams[63] );
	stim_wang->set_si_vas( optiParams[64] );
	stim_wang->set_si_rf( optiParams[65] );
	stim_wang->set_K_sp_vas( optiParams[66] );
	stim_wang->set_K_sp_glu( optiParams[67] );
	stim_wang->set_K_sp_hfl( optiParams[68] );
	stim_wang->set_D_sp_vas( optiParams[69] );
	stim_wang->set_D_sp_glu( optiParams[70] );
	stim_wang->set_D_sp_hfl( optiParams[71] );
	stim_wang->set_theta_k_ref( optiParams[72] );
	stim_wang->set_d_sp( optiParams[73] );
	stim_wang->set_d_si( optiParams[74] );
	stim_wang->set_k_THETA( optiParams[75] );
	stim_wang->set_phi_off_pk( optiParams[76] );
}
