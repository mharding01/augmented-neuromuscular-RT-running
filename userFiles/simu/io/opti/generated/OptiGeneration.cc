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
    g_osc->set_eta_C(optiParams[5]);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(optiParams[6]);  
    g_osc->set_eta_E(optiParams[7]);  

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(optiParams[8]);
    g_osc->set_P_theta_hip(optiParams[9]);
    g_osc->set_P_k_theta(optiParams[10]);
    
    // Linear
    g_osc->set_P_k_HFLrun1(optiParams[11]);
    g_osc->set_p_k_HFLrun1(optiParams[12]);

    g_osc->set_P_G_SOL_TA(optiParams[13]);
    g_osc->set_p_G_SOL_TA(optiParams[14]);

    g_osc->set_P_G_GAS(optiParams[15]);
    g_osc->set_p_G_GAS(optiParams[16]);

    g_osc->set_P_G_VAS(optiParams[17]);
    g_osc->set_p_G_VAS(optiParams[18]);
    
    // Quadratic
    g_osc->set_P_tau(optiParams[19]);
    g_osc->set_p_tau(optiParams[20]);
    g_osc->set_p2_tau(optiParams[21]);
    
    g_osc->set_P_k_HFLrun2(optiParams[22]);
    g_osc->set_p_k_HFLrun2(optiParams[23]);
    g_osc->set_p2_k_HFLrun2(optiParams[24]);
    
    g_osc->set_P_k_HAMrun(optiParams[25]);
    g_osc->set_p_k_HAMrun(optiParams[26]);
    g_osc->set_p2_k_HAMrun(optiParams[27]);

    g_osc->set_P_G_SOL(optiParams[28]);
    g_osc->set_p_G_SOL(optiParams[29]);
    g_osc->set_p2_G_SOL(optiParams[30]);

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( optiParams[31] );
	stim_wang->set_S0_ta_st( optiParams[32] );
	stim_wang->set_S0_gas_st( optiParams[33] );
	stim_wang->set_S0_vas_st( optiParams[34] );
	stim_wang->set_S0_ham_st( optiParams[35] );
	stim_wang->set_S0_rf_st( optiParams[36] );
	stim_wang->set_S0_glu_st( optiParams[37] );
	stim_wang->set_S0_hfl_st( optiParams[38] );
	stim_wang->set_S0_sol_sw( optiParams[39] );
	stim_wang->set_S0_ta_sw( optiParams[40] );
	stim_wang->set_S0_gas_sw( optiParams[41] );
	stim_wang->set_S0_vas_sw( optiParams[42] );
	stim_wang->set_S0_ham_sw( optiParams[43] );
	stim_wang->set_S0_rf_sw( optiParams[44] );
	stim_wang->set_S0_glu_sw( optiParams[45] );
	stim_wang->set_S0_hfl_sw( optiParams[46] );
	stim_wang->set_G_ham( optiParams[47] );
	stim_wang->set_G_glu( optiParams[48] );
	stim_wang->set_G_ta_sw( optiParams[49] );
	stim_wang->set_G_ta_st( optiParams[50] );
	stim_wang->set_G_hfl( optiParams[51] );
	stim_wang->set_G_ham_hfl( optiParams[52] );
	stim_wang->set_l_off_ta_sw( optiParams[53] );
	stim_wang->set_l_off_ta_st( optiParams[54] );
	stim_wang->set_l_off_ham_hfl( optiParams[55] );
	stim_wang->set_l_off_hfl( optiParams[56] );
	stim_wang->set_K_ham( optiParams[57] );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( optiParams[58] ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( optiParams[59] ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( optiParams[60] );
	stim_wang->set_D_glu( optiParams[61] );
	stim_wang->set_D_hfl( optiParams[62] );
	stim_wang->set_si_vas( optiParams[63] );
	stim_wang->set_si_rf( optiParams[64] );
	stim_wang->set_si_glu( optiParams[65] );
	stim_wang->set_si_hfl( optiParams[66] );
	stim_wang->set_K_sp_vas( optiParams[67] );
	stim_wang->set_K_sp_glu( optiParams[68] );
	stim_wang->set_K_sp_hfl( optiParams[69] );
	stim_wang->set_D_sp_vas( optiParams[70] );
	stim_wang->set_D_sp_glu( optiParams[71] );
	stim_wang->set_D_sp_hfl( optiParams[72] );
	stim_wang->set_theta_k_ref( optiParams[73] );
	stim_wang->set_d_sp( optiParams[74] );
	stim_wang->set_d_si( optiParams[75] );
	stim_wang->set_k_THETA( optiParams[76] );
	stim_wang->set_phi_off_pk( optiParams[77] );
}
