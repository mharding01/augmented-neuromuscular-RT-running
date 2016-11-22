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
    g_osc->set_gamma_A(1.56551109);
    g_osc->set_gamma_B(2.38363148);
    g_osc->set_gamma_C(2.84861107);

    g_osc->set_eta_A(5.50518447);  
    g_osc->set_eta_B(4.82111397);  
    g_osc->set_eta_C(5.65738235);  
    g_osc->set_eta_D(3.73848452);  
    g_osc->set_eta_E(3.67710330);  

    g_osc->set_P_tau(0.04949891 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 3.97124703);
    g_osc->set_k_HFLrun2( 6.39695769);
    g_osc->set_k_HAMrun3( 0.65838671);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 5.11137999 );
	stim_wang->set_S0_sol_st( 0.01227052 );
	stim_wang->set_S0_ta_st( 0.04971914 );
	stim_wang->set_S0_gas_st( 0.02561523 );
	stim_wang->set_S0_vas_st( 0.51519880 );
	stim_wang->set_S0_ham_st( 0.19431216 );
	stim_wang->set_S0_rf_st( 0.15204551 );
	stim_wang->set_S0_glu_st( 0.06903039 );
	stim_wang->set_S0_hfl_st( 0.21964406 );
	stim_wang->set_S0_sol_sw( 0.01452694 );
	stim_wang->set_S0_ta_sw( 0.04929693 );
	stim_wang->set_S0_gas_sw( 0.04527688 );
	stim_wang->set_S0_vas_sw( 0.08896354 );
	stim_wang->set_S0_ham_sw( 0.06846763 );
	stim_wang->set_S0_rf_sw( 0.26401277 );
	stim_wang->set_S0_glu_sw( 0.01257405 );
	stim_wang->set_S0_hfl_sw( 0.03538940 );
	stim_wang->set_G_sol( 2.66344067 );
	stim_wang->set_G_sol_ta( 2.81462984 );
	stim_wang->set_G_gas( 5.41535935 );
	stim_wang->set_G_vas( 2.25229855 );
	stim_wang->set_G_ham( 0.87907301 );
	stim_wang->set_G_glu( 0.29483814 );
	stim_wang->set_G_ta_sw( 4.35356391 );
	stim_wang->set_G_ta_st( 3.93005373 );
	stim_wang->set_G_hfl( 1.73354612 );
	stim_wang->set_G_ham_hfl( 6.05225152 );
	stim_wang->set_l_off_ta_sw( 0.62838861 );
	stim_wang->set_l_off_ta_st( 0.67308197 );
	stim_wang->set_l_off_ham_hfl( 0.67413758 );
	stim_wang->set_l_off_hfl( 0.39666631 );
	stim_wang->set_K_ham( 4.75289279 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 9.88402165 );
	stim_wang->set_K_hfl( 9.68191006 );
	stim_wang->set_D_ham( 0.31391452 );
	stim_wang->set_D_glu( 0.17579505 );
	stim_wang->set_D_hfl( 0.46744765 );
	stim_wang->set_theta_ref( 0.06296305 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.92228868 );
	stim_wang->set_si_rf( 0.16132062 );
	stim_wang->set_si_glu( 0.80131784 );
	stim_wang->set_si_hfl( 0.59629352 );
	stim_wang->set_K_sp_vas( 0.13455421 );
	stim_wang->set_K_sp_glu( 3.40982200 );
	stim_wang->set_K_sp_hfl( 3.81842745 );
	stim_wang->set_D_sp_vas( 0.06688372 );
	stim_wang->set_D_sp_glu( 0.04442026 );
	stim_wang->set_D_sp_hfl( 0.04129171 );
	stim_wang->set_theta_k_ref( 0.15015617 );
	stim_wang->set_theta_h_ref0( 0.31835327 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.02345817 );
	stim_wang->set_d_si( 0.44146734 );
	stim_wang->set_k_THETA( 5.12471034 );
	stim_wang->set_k_theta( 2.28901342 );
	stim_wang->set_phi_off_pk( 0.14474719 );

	//init pos
	joints_init->set_T3( 0.50644827 );
	joints_init->set_T3_p( 0.39463852 );
	joints_init->set_R2( 0.05089726 );
	joints_init->set_R2_p( -0.05863734 );
	joints_init->set_r_sh_p( -2.79143247 );
	joints_init->set_r_hip( -0.38467721 );
	joints_init->set_r_hip_p( -2.35915692 );
	joints_init->set_r_knee( 0.58468035 );
	joints_init->set_r_knee_p( -1.78715971 );
	joints_init->set_r_ankle( -0.03986721 );
	joints_init->set_r_ankle_p( -4.02914827 );
	joints_init->set_l_hip( 0.06751969 );
	joints_init->set_l_hip_p( 1.44246560 );
	joints_init->set_l_knee( 1.18415685 );
	joints_init->set_l_knee_p( 2.10592280 );
	joints_init->set_l_ankle( 0.23065829 );
	joints_init->set_l_ankle_p( 0.15629573 );
}
