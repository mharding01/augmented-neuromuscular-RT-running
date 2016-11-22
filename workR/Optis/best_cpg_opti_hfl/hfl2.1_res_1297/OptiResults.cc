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
    g_osc->set_gamma_A(1.50055851);
    g_osc->set_gamma_B(2.44740994);
    g_osc->set_gamma_C(2.69474060);

    g_osc->set_eta_A(5.51172005);  
    g_osc->set_eta_B(4.82738857);  
    g_osc->set_eta_C(5.63461842);  
    g_osc->set_eta_D(3.76405886);  
    g_osc->set_eta_E(3.64409737);  

    g_osc->set_P_tau(0.05641256 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 4.48858251);
    g_osc->set_k_HFLrun2( 5.09498090);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 2.22826992 );
	stim_wang->set_S0_sol_st( 0.04150142 );
	stim_wang->set_S0_ta_st( 0.02494345 );
	stim_wang->set_S0_gas_st( 0.01918588 );
	stim_wang->set_S0_vas_st( 0.28469995 );
	stim_wang->set_S0_ham_st( 0.21234984 );
	stim_wang->set_S0_rf_st( 0.28955137 );
	stim_wang->set_S0_glu_st( 0.10209518 );
	stim_wang->set_S0_hfl_st( 0.14737035 );
	stim_wang->set_S0_sol_sw( 0.01958790 );
	stim_wang->set_S0_ta_sw( 0.02642059 );
	stim_wang->set_S0_gas_sw( 0.05887592 );
	stim_wang->set_S0_vas_sw( 0.07853863 );
	stim_wang->set_S0_ham_sw( 0.06481000 );
	stim_wang->set_S0_rf_sw( 0.27913460 );
	stim_wang->set_S0_glu_sw( 0.03466372 );
	stim_wang->set_S0_hfl_sw( 0.02529774 );
	stim_wang->set_G_sol( 2.65662374 );
	stim_wang->set_G_sol_ta( 2.78146380 );
	stim_wang->set_G_gas( 11.22066516 );
	stim_wang->set_G_vas( 4.03662938 );
	stim_wang->set_G_ham( 1.16812170 );
	stim_wang->set_G_glu( 0.45534772 );
	stim_wang->set_G_ta_sw( 4.70663555 );
	stim_wang->set_G_ta_st( 4.38460731 );
	stim_wang->set_G_hfl( 2.89537123 );
	stim_wang->set_G_ham_hfl( 1.34305162 );
	stim_wang->set_l_off_ta_sw( 0.53339315 );
	stim_wang->set_l_off_ta_st( 0.72250945 );
	stim_wang->set_l_off_ham_hfl( 0.66117971 );
	stim_wang->set_l_off_hfl( 0.24736968 );
	stim_wang->set_K_ham( 8.51045007 );
	stim_wang->set_K_glu( 10.79671604 );
	stim_wang->set_K_hfl( 14.30803098 );
	stim_wang->set_D_ham( 0.35538869 );
	stim_wang->set_D_glu( 0.42467173 );
	stim_wang->set_D_hfl( 0.43690682 );
	stim_wang->set_theta_ref( 0.07537490 );
	stim_wang->set_si_vas( 0.83694254 );
	stim_wang->set_si_rf( 0.40232503 );
	stim_wang->set_si_glu( 0.62844386 );
	stim_wang->set_si_hfl( 0.92929311 );
	stim_wang->set_K_sp_vas( 1.24344725 );
	stim_wang->set_K_sp_glu( 3.26026273 );
	stim_wang->set_K_sp_hfl( 2.26369720 );
	stim_wang->set_D_sp_vas( 0.05665517 );
	stim_wang->set_D_sp_glu( 0.08120476 );
	stim_wang->set_D_sp_hfl( 0.05380159 );
	stim_wang->set_theta_k_ref( 0.16749730 );
	stim_wang->set_theta_h_ref0( 0.50958783 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.09218105 );
	stim_wang->set_d_si( 0.59562025 );
	stim_wang->set_k_THETA( 3.92860211 );
	stim_wang->set_k_theta( 9.64556494 );
	stim_wang->set_phi_off_pk( 0.14602602 );

	//init pos
	joints_init->set_T3( 0.49825234 );
	joints_init->set_T3_p( -0.39939354 );
	joints_init->set_R2( 0.18055430 );
	joints_init->set_R2_p( 0.14746524 );
	joints_init->set_r_sh_p( -3.57953849 );
	joints_init->set_r_hip( -0.51685074 );
	joints_init->set_r_hip_p( -0.89699301 );
	joints_init->set_r_knee( 0.08442704 );
	joints_init->set_r_knee_p( 1.02389065 );
	joints_init->set_r_ankle( 0.03022198 );
	joints_init->set_r_ankle_p( -0.04129424 );
	joints_init->set_l_hip( 0.28131846 );
	joints_init->set_l_hip_p( 0.38315228 );
	joints_init->set_l_knee( 1.54491631 );
	joints_init->set_l_knee_p( 4.22048414 );
	joints_init->set_l_ankle( 0.38681896 );
	joints_init->set_l_ankle_p( -1.61284226 );
}
