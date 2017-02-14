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
    g_osc->set_gamma_A(1.54342853);
    g_osc->set_gamma_B(2.52247928);
    g_osc->set_gamma_C(2.58841846);

    g_osc->set_eta_A(5.46959229);  
    g_osc->set_eta_B(4.89004444);  
    g_osc->set_eta_C(5.98686131);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73465877);  
    g_osc->set_eta_E(3.68666009);  

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(-0.03638933);
    g_osc->set_P_theta_hip(0.17218969);
    g_osc->set_P_k_theta(10.14132788);
    
    // Linear
    g_osc->set_P_k_HFLrun1(5.81100059);
    g_osc->set_p_k_HFLrun1(-3.49579518);

    g_osc->set_P_G_SOL_TA(7.32924734);
    g_osc->set_p_G_SOL_TA(6.38126200);

    g_osc->set_P_G_GAS(8.18220099);
    g_osc->set_p_G_GAS(-14.26031216);

    g_osc->set_P_G_VAS(1.45490958);
    g_osc->set_p_G_VAS(0.44084504);
    
    // Quadratic
    g_osc->set_P_tau(0.07039286);
    g_osc->set_p_tau(0.00344288);
    g_osc->set_p2_tau(-0.34213891);
    
    g_osc->set_P_k_HFLrun2(9.66263131);
    g_osc->set_p_k_HFLrun2(39.16047834);
    g_osc->set_p2_k_HFLrun2(21.11652307);
    
    g_osc->set_P_k_HAMrun(6.75572624);
    g_osc->set_p_k_HAMrun(0.96823090);
    g_osc->set_p2_k_HAMrun(20.83475103);

    g_osc->set_P_G_SOL(3.46974623);
    g_osc->set_p_G_SOL(4.48271183);
    g_osc->set_p2_G_SOL(11.81920647);

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03446639 );
	stim_wang->set_S0_ta_st( 0.02349686 );
	stim_wang->set_S0_gas_st( 0.02246540 );
	stim_wang->set_S0_vas_st( 0.40024992 );
	stim_wang->set_S0_ham_st( 0.19091668 );
	stim_wang->set_S0_rf_st( 0.46634284 );
	stim_wang->set_S0_glu_st( 0.25711767 );
	stim_wang->set_S0_hfl_st( 0.06443106 );
	stim_wang->set_S0_sol_sw( 0.02461646 );
	stim_wang->set_S0_ta_sw( 0.02450263 );
	stim_wang->set_S0_gas_sw( 0.02664023 );
	stim_wang->set_S0_vas_sw( 0.08248610 );
	stim_wang->set_S0_ham_sw( 0.05705112 );
	stim_wang->set_S0_rf_sw( 0.23731131 );
	stim_wang->set_S0_glu_sw( 0.02238267 );
	stim_wang->set_S0_hfl_sw( 0.01413455 );
	stim_wang->set_G_ham( 0.96924249 );
	stim_wang->set_G_glu( 0.90805030 );
	stim_wang->set_G_ta_sw( 3.53824841 );
	stim_wang->set_G_ta_st( 1.68194463 );
	stim_wang->set_G_hfl( 2.29094108 );
	stim_wang->set_G_ham_hfl( 4.80553252 );
	stim_wang->set_l_off_ta_sw( 0.73563871 );
	stim_wang->set_l_off_ta_st( 0.61832672 );
	stim_wang->set_l_off_ham_hfl( 0.44663053 );
	stim_wang->set_l_off_hfl( 0.27578244 );
	stim_wang->set_K_ham( 3.90352166 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 4.69766016 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 6.38026235 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.16899946 );
	stim_wang->set_D_glu( 0.15022309 );
	stim_wang->set_D_hfl( 0.71696047 );
	stim_wang->set_si_vas( 0.81864135 );
	stim_wang->set_si_rf( 0.46469697 );
	stim_wang->set_si_glu( 0.66109741 );
	stim_wang->set_si_hfl( 0.75136641 );
	stim_wang->set_K_sp_vas( 0.24608867 );
	stim_wang->set_K_sp_glu( 3.38635614 );
	stim_wang->set_K_sp_hfl( 0.95556288 );
	stim_wang->set_D_sp_vas( 0.06177686 );
	stim_wang->set_D_sp_glu( 0.04217671 );
	stim_wang->set_D_sp_hfl( 0.07331029 );
	stim_wang->set_theta_k_ref( 0.18914053 );
	stim_wang->set_d_sp( -0.01685162 );
	stim_wang->set_d_si( 0.63588791 );
	stim_wang->set_k_THETA( 2.86725451 );
	stim_wang->set_phi_off_pk( 0.12150555 );

	//init pos
	joints_init->set_T3( 0.51223863 );
	joints_init->set_T3_p( 0.00185412 );
	joints_init->set_R2( 0.06613346 );
	joints_init->set_R2_p( 3.91008657 );
	joints_init->set_r_sh_p( 4.87004676 );
	joints_init->set_r_hip( -0.77071918 );
	joints_init->set_r_hip_p( -3.83071851 );
	joints_init->set_r_knee( 0.24857749 );
	joints_init->set_r_knee_p( -3.64650428 );
	joints_init->set_r_ankle( 0.07297330 );
	joints_init->set_r_ankle_p( -3.85941459 );
	joints_init->set_l_hip( 0.06109374 );
	joints_init->set_l_hip_p( -1.23523796 );
	joints_init->set_l_knee( 1.22608835 );
	joints_init->set_l_knee_p( 3.78344275 );
	joints_init->set_l_ankle( 0.30323805 );
	joints_init->set_l_ankle_p( 4.18894087 );
}
