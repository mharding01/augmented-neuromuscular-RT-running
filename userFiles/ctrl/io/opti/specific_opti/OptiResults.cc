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
    g_osc->set_gamma_A(0.34639743); // Longer or shorter than 50% of %50 of gait cycle
    g_osc->set_gamma_B(1.27622469);
    g_osc->set_gamma_C(2.73156693);

    g_osc->set_eta_A(5.48697143);  
    g_osc->set_eta_B(4.85904703);  
    g_osc->set_eta_C(5.68190810);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.74236770);  
    g_osc->set_eta_E(3.71289230);  

    g_osc->set_P_tau(0.11654366 );    // TODO: 

    g_osc->set_k_HFLrun1( 2.97624841);
    g_osc->set_k_HFLrun2( 8.64446827);
    g_osc->set_k_HAMrun3( 3.04913570);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03115479 );
	stim_wang->set_S0_ta_st( 0.02197836 );
	stim_wang->set_S0_gas_st( 0.02350046 );
	stim_wang->set_S0_vas_st( 0.27421670 );
	stim_wang->set_S0_ham_st( 0.11748006 );
	stim_wang->set_S0_rf_st( 0.34656157 );
	stim_wang->set_S0_glu_st( 0.11966928 );
	stim_wang->set_S0_hfl_st( 0.26199135 );
	stim_wang->set_S0_sol_sw( 0.02581696 );
	stim_wang->set_S0_ta_sw( 0.03297640 );
	stim_wang->set_S0_gas_sw( 0.07442018 );
	stim_wang->set_S0_vas_sw( 0.05359915 );
	stim_wang->set_S0_ham_sw( 0.07307360 );
	stim_wang->set_S0_rf_sw( 0.16749611 );
	stim_wang->set_S0_glu_sw( 0.04191842 );
	stim_wang->set_S0_hfl_sw( 0.02341097 );
	stim_wang->set_G_sol( 4.70454955 );
	stim_wang->set_G_sol_ta( 4.83310634 );
	stim_wang->set_G_gas( 15.00198317 );
	stim_wang->set_G_vas( 2.33809740 );
	stim_wang->set_G_ham( 1.12170214 );
	stim_wang->set_G_glu( 1.36230708 );
	stim_wang->set_G_ta_sw( 4.47866587 );
	stim_wang->set_G_ta_st( 3.82033550 );
	stim_wang->set_G_hfl( 1.74694648 );
	stim_wang->set_G_ham_hfl( 8.40012516 );
	stim_wang->set_l_off_ta_sw( 0.52756536 );
	stim_wang->set_l_off_ta_st( 0.67016364 );
	stim_wang->set_l_off_ham_hfl( 0.42613999 );
	stim_wang->set_l_off_hfl( 0.30831995 );
	stim_wang->set_K_ham( 4.26645446 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 12.43170326 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 5.53253873 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.42780352 );
	stim_wang->set_D_glu( 0.14711968 );
	stim_wang->set_D_hfl( 0.68958070 );
	stim_wang->set_theta_ref( 0.05742870 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.77507733 );
	stim_wang->set_si_rf( 0.82967468 );
	stim_wang->set_si_glu( 0.71598670 );
	stim_wang->set_si_hfl( 0.82278060 );
	stim_wang->set_K_sp_vas( 4.44985207 );
	stim_wang->set_K_sp_glu( 3.36767380 );
	stim_wang->set_K_sp_hfl( 0.42430263 );
	stim_wang->set_D_sp_vas( 0.03401853 );
	stim_wang->set_D_sp_glu( 0.03049704 );
	stim_wang->set_D_sp_hfl( 0.06200927 );
	stim_wang->set_theta_k_ref( 0.07180202 );
	stim_wang->set_theta_h_ref0( 0.20617916 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.13813388 );
	stim_wang->set_d_si( 0.66789365 );
	stim_wang->set_k_THETA( 2.24908646 );
	stim_wang->set_k_theta( 12.68038236 );
	stim_wang->set_phi_off_pk( 0.14709330 );

	//init pos
	joints_init->set_T3( 0.51274772 );
	joints_init->set_T3_p( 0.43316763 );
	joints_init->set_R2( 0.06440920 );
	joints_init->set_R2_p( 2.37088206 );
	joints_init->set_r_sh_p( -4.29145591 );
	joints_init->set_r_hip( -0.77370179 );
	joints_init->set_r_hip_p( 2.31662166 );
	joints_init->set_r_knee( 0.34845990 );
	joints_init->set_r_knee_p( -1.78802286 );
	joints_init->set_r_ankle( 0.05229381 );
	joints_init->set_r_ankle_p( 0.23703220 );
	joints_init->set_l_hip( 0.22378707 );
	joints_init->set_l_hip_p( -3.99432126 );
	joints_init->set_l_knee( 0.78040915 );
	joints_init->set_l_knee_p( 4.47157410 );
	joints_init->set_l_ankle( 0.37953762 );
	joints_init->set_l_ankle_p( 0.82900825 );
}
