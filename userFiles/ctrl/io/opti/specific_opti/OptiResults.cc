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
    g_osc->set_gamma_A(1.57553931);
    g_osc->set_gamma_B(2.53633384);
    g_osc->set_gamma_C(2.65514190);

    g_osc->set_eta_A(5.49681774);  
    g_osc->set_eta_B(4.86147345);  
    g_osc->set_eta_C(5.68391234);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73377651);  
    g_osc->set_eta_E(3.66622041);  

    g_osc->set_P_tau(0.04747445 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 1.59619401);
    g_osc->set_k_HFLrun2( 7.01533279);
    g_osc->set_k_HAMrun3( 6.20870389);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.04051781 );
	stim_wang->set_S0_ta_st( 0.01605854 );
	stim_wang->set_S0_gas_st( 0.03672691 );
	stim_wang->set_S0_vas_st( 0.32842455 );
	stim_wang->set_S0_ham_st( 0.29797363 );
	stim_wang->set_S0_rf_st( 0.09781730 );
	stim_wang->set_S0_glu_st( 0.11264627 );
	stim_wang->set_S0_hfl_st( 0.09101347 );
	stim_wang->set_S0_sol_sw( 0.02880403 );
	stim_wang->set_S0_ta_sw( 0.02961106 );
	stim_wang->set_S0_gas_sw( 0.09458508 );
	stim_wang->set_S0_vas_sw( 0.02181909 );
	stim_wang->set_S0_ham_sw( 0.06652838 );
	stim_wang->set_S0_rf_sw( 0.37252351 );
	stim_wang->set_S0_glu_sw( 0.01657156 );
	stim_wang->set_S0_hfl_sw( 0.04202100 );
	stim_wang->set_G_sol( 2.44040651 );
	stim_wang->set_G_sol_ta( 4.73210470 );
	stim_wang->set_G_gas( 16.29604047 );
	stim_wang->set_G_vas( 1.05551413 );
	stim_wang->set_G_ham( 2.28251452 );
	stim_wang->set_G_glu( 1.07624631 ); 
	stim_wang->set_G_ta_sw( 3.46270266 );
	stim_wang->set_G_ta_st( 2.19688800 );
	stim_wang->set_G_hfl( 2.14613290 );
	stim_wang->set_G_ham_hfl( 6.10776576 );
	stim_wang->set_l_off_ta_sw( 0.61752060 );
	stim_wang->set_l_off_ta_st( 0.60032765 );
	stim_wang->set_l_off_ham_hfl( 0.47799085 );
	stim_wang->set_l_off_hfl( 0.53284490 );
	stim_wang->set_K_ham( 3.30099978 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 6.97141634 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 8.51399488 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.28472525 );
	stim_wang->set_D_glu( 0.05289260 );
	stim_wang->set_D_hfl( 0.70371939 );
	stim_wang->set_theta_ref( 0.02114376 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.78000072 );
	stim_wang->set_si_rf( 0.10492337 );
	stim_wang->set_si_glu( 0.62043390 );
	stim_wang->set_si_hfl( 0.27594038 );
	stim_wang->set_K_sp_vas( 1.59802488 );
	stim_wang->set_K_sp_glu( 3.11768317 );
	stim_wang->set_K_sp_hfl( 1.88241262 );
	stim_wang->set_D_sp_vas( 0.09571155 );
	stim_wang->set_D_sp_glu( 0.02186125 );
	stim_wang->set_D_sp_hfl( 0.08392030 );
	stim_wang->set_theta_k_ref( 0.29975759 );
	stim_wang->set_theta_h_ref0( 0.60620732 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.08317890 );
	stim_wang->set_d_si( 0.32710358 );
	stim_wang->set_k_THETA( 3.37682482 );
	stim_wang->set_k_theta( 2.53505258 );
	stim_wang->set_phi_off_pk( 0.08940935 );

	//init pos
    /*
	joints_init->set_T3( 0.51434532 );
	joints_init->set_T3_p( -0.19917298 );
	joints_init->set_R2( 0.15931929 );
	joints_init->set_R2_p( 0.12469546 );
	joints_init->set_r_sh_p( 1.78067287 );
	joints_init->set_r_hip( -0.53664085 );
	joints_init->set_r_hip_p( 2.35038869 );
	joints_init->set_r_knee( 0.50076082 );
	joints_init->set_r_knee_p( 1.65588274 );
	joints_init->set_r_ankle( 0.04930992 );
	joints_init->set_r_ankle_p( 4.63256804 );
	joints_init->set_l_hip( -0.01129314 );
	joints_init->set_l_hip_p( -1.50775015 );
	joints_init->set_l_knee( 0.96870977 );
	joints_init->set_l_knee_p( -0.15919546 );
	joints_init->set_l_ankle( 0.27641770 );
	joints_init->set_l_ankle_p( -1.33132669 );
    */
}
