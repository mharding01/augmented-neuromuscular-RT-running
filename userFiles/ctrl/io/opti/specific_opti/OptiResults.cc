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
    g_osc->set_gamma_A(1.31466772);
    g_osc->set_gamma_B(2.40343304);
    g_osc->set_gamma_C(2.78442275);

    g_osc->set_beta_A(5.78212310);
    g_osc->set_beta_B(5.55493185);
    g_osc->set_beta_C(3.60503114);

    g_osc->set_eta_A(5.48931652);  
    g_osc->set_eta_B(4.89257413);  
    g_osc->set_eta_C(5.42287258);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73940070);  
    g_osc->set_eta_E(3.69522122);

    g_osc->set_eta_F(4.73163386);
    g_osc->set_eta_G(3.52516749); 

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(0.04352992); // 0.0609
    g_osc->set_P_theta_hip(0.13607615); // 0.1616
    g_osc->set_P_k_theta(7.57313438); // 8.6158
    
    // Linear
    g_osc->set_P_k_HFLrun1(7.63949718); // 5.1025
    g_osc->set_p_k_HFLrun1(-4.31105036); // -3.1544

    g_osc->set_P_G_SOL_TA(6.74090020); // 5.3225
    g_osc->set_p_G_SOL_TA(10.57195677); // 9.0977

    g_osc->set_P_G_GAS(8.80164898); // 7.5656
    g_osc->set_p_G_GAS(-14.96919228); // -13.7575

    g_osc->set_P_G_VAS(2.71664392); // 1.6058
    g_osc->set_p_G_VAS(1.29140329); // 2.0708
    
    // Quadratic
    g_osc->set_P_tau(0.05645322); // 0.0465
    g_osc->set_p_tau(-0.00373714); // -.0168
    g_osc->set_p2_tau(0.00861792);// -.0277
    
    g_osc->set_P_k_HFLrun2(8.79112856); // 5.8581
    g_osc->set_p_k_HFLrun2(29.07043842);// 29.9698
    g_osc->set_p2_k_HFLrun2(85.01290082);//84.8007
    
    g_osc->set_P_k_HAMrun(5.58157540); // 3.8158
    g_osc->set_p_k_HAMrun(8.80416757); // 8.7087
    g_osc->set_p2_k_HAMrun(31.86747170);//34.0513

    g_osc->set_P_G_SOL(4.77975898); // 2.9365
    g_osc->set_p_G_SOL(5.02438920); // 3.4425
    g_osc->set_p2_G_SOL(14.02293313); // 13.5518

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03818302 );
	stim_wang->set_S0_ta_st( 0.04154003 );
	stim_wang->set_S0_gas_st( 0.04980694 );
	stim_wang->set_S0_vas_st( 0.23074307 );
	stim_wang->set_S0_ham_st( 0.20799120 );
	stim_wang->set_S0_rf_st( 0.06717290 );
	stim_wang->set_S0_glu_st( 0.30088037 );
	stim_wang->set_S0_hfl_st( 0.08220173 );
	stim_wang->set_S0_sol_sw( 0.02868729 );
	stim_wang->set_S0_ta_sw( 0.04480496 );
	stim_wang->set_S0_gas_sw( 0.10223686 );
	stim_wang->set_S0_vas_sw( 0.01282052 );
	stim_wang->set_S0_ham_sw( 0.08402490 );
	stim_wang->set_S0_rf_sw( 0.47692941 );
	stim_wang->set_S0_glu_sw( 0.04719897 );
	stim_wang->set_S0_hfl_sw( 0.03901564 );
	stim_wang->set_G_ham( 1.19918457 );
	stim_wang->set_G_glu( 0.64059899 );
	stim_wang->set_G_ta_sw( 3.49518872 );
	stim_wang->set_G_ta_st( 2.54519658 );
	stim_wang->set_l_off_ta_sw( 0.54057781 );
	stim_wang->set_l_off_ta_st( 0.64669994 );
	stim_wang->set_K_ham( 5.37770470 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 8.85066706 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 3.01257080 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.19354081 );
	stim_wang->set_D_glu( 0.40696855 );
	stim_wang->set_D_hfl( 0.12420056 );
	stim_wang->set_si_vas( 0.75524048 );
	stim_wang->set_si_rf( 0.64097198 );
	stim_wang->set_K_sp_vas( 0.58630839 );
	stim_wang->set_K_sp_glu( 2.82446922 );
	stim_wang->set_K_sp_hfl( 4.96323862 );
	stim_wang->set_D_sp_vas( 0.07211924 );
	stim_wang->set_D_sp_glu( 0.07267726 );
	stim_wang->set_D_sp_hfl( 0.03362618 );
	stim_wang->set_theta_k_ref( 0.22824145 );
	stim_wang->set_d_sp( -0.05599871 );
	stim_wang->set_d_si( 0.68096545 );
	stim_wang->set_k_THETA( 2.92964455 );
	stim_wang->set_phi_off_pk( 0.16679902 );
}
