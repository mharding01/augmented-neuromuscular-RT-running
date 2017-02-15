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
    g_osc->set_gamma_A(1.46874083);
    g_osc->set_gamma_B(2.48369634);
    g_osc->set_gamma_C(2.85076784);

    g_osc->set_eta_A(5.46799006);  
    g_osc->set_eta_B(4.83376136);  
    g_osc->set_eta_C(5.78087383);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.74734351);  
    g_osc->set_eta_E(3.67944872);  

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(0.09816749);
    g_osc->set_P_theta_hip(0.24083103);
    g_osc->set_P_k_theta(11.26876542);
    
    // Linear
    g_osc->set_P_k_HFLrun1(6.96719573);
    g_osc->set_p_k_HFLrun1(-3.03990666);

    g_osc->set_P_G_SOL_TA(6.93727478);
    g_osc->set_p_G_SOL_TA(7.01990501);

    g_osc->set_P_G_GAS(10.85855341);
    g_osc->set_p_G_GAS(-6.21455481);

    g_osc->set_P_G_VAS(2.22888245);
    g_osc->set_p_G_VAS(4.57103267);
    
    // Quadratic
    g_osc->set_P_tau(0.05351429);
    g_osc->set_p_tau(-0.02200861);
    g_osc->set_p2_tau(-0.06957676);
    
    g_osc->set_P_k_HFLrun2(10.78284186);
    g_osc->set_p_k_HFLrun2(26.37058032);
    g_osc->set_p2_k_HFLrun2(40.76674021);
    
    g_osc->set_P_k_HAMrun(5.99751237);
    g_osc->set_p_k_HAMrun(1.41347886);
    g_osc->set_p2_k_HAMrun(20.10226237);

    g_osc->set_P_G_SOL(4.52922122);
    g_osc->set_p_G_SOL(4.29233683);
    g_osc->set_p2_G_SOL(5.80546771);

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.01753195 );
	stim_wang->set_S0_ta_st( 0.02204223 );
	stim_wang->set_S0_gas_st( 0.02554709 );
	stim_wang->set_S0_vas_st( 0.67218875 );
	stim_wang->set_S0_ham_st( 0.12248562 );
	stim_wang->set_S0_rf_st( 0.04755801 );
	stim_wang->set_S0_glu_st( 0.28470727 );
	stim_wang->set_S0_hfl_st( 0.04430844 );
	stim_wang->set_S0_sol_sw( 0.01498389 );
	stim_wang->set_S0_ta_sw( 0.04266880 );
	stim_wang->set_S0_gas_sw( 0.14747085 );
	stim_wang->set_S0_vas_sw( 0.02984441 );
	stim_wang->set_S0_ham_sw( 0.08619052 );
	stim_wang->set_S0_rf_sw( 0.22464248 );
	stim_wang->set_S0_glu_sw( 0.01703052 );
	stim_wang->set_S0_hfl_sw( 0.02586521 );
	stim_wang->set_G_ham( 0.94035820 );
	stim_wang->set_G_glu( 0.31005847 );
	stim_wang->set_G_ta_sw( 4.25623040 );
	stim_wang->set_G_ta_st( 0.77832616 );
	stim_wang->set_G_hfl( 1.92199919 );
	stim_wang->set_G_ham_hfl( 4.96678020 );
	stim_wang->set_l_off_ta_sw( 0.54161859 );
	stim_wang->set_l_off_ta_st( 0.69328677 );
	stim_wang->set_l_off_ham_hfl( 0.20852504 );
	stim_wang->set_l_off_hfl( 0.17038076 );
	stim_wang->set_K_ham( 5.45405875 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 11.37648811 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.84723500 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.11140799 );
	stim_wang->set_D_glu( 0.23088320 );
	stim_wang->set_D_hfl( 0.19553304 );
	stim_wang->set_si_vas( 0.78486783 );
	stim_wang->set_si_rf( 0.68875310 );
	stim_wang->set_si_glu( 0.58461900 );
	stim_wang->set_si_hfl( 0.57898144 );
	stim_wang->set_K_sp_vas( 0.20971720 );
	stim_wang->set_K_sp_glu( 3.11960234 );
	stim_wang->set_K_sp_hfl( 1.23363754 );
	stim_wang->set_D_sp_vas( 0.04633572 );
	stim_wang->set_D_sp_glu( 0.02580766 );
	stim_wang->set_D_sp_hfl( 0.08285719 );
	stim_wang->set_theta_k_ref( 0.01491167 );
	stim_wang->set_d_sp( 0.03962495 );
	stim_wang->set_d_si( 0.36946983 );
	stim_wang->set_k_THETA( 1.56085226 );
	stim_wang->set_phi_off_pk( 0.08998778 );
}
