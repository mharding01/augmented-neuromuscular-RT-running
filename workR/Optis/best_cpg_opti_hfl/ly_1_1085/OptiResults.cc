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
    g_osc->set_gamma_A(1.52875156);
    g_osc->set_gamma_B(2.46553687);
    g_osc->set_gamma_C(2.96911196);

    g_osc->set_eta_A(5.46632143);  
    g_osc->set_eta_B(4.84031739);  
    g_osc->set_eta_C(5.65685067);  
    g_osc->set_eta_D(3.74929216);  
    g_osc->set_eta_E(3.68785481);  

    g_osc->set_P_tau(0.05877190 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 2.93757957);
    g_osc->set_k_HFLrun2( 5.91811934);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 5.49278043 );
	stim_wang->set_S0_sol_st( 0.02444064 );
	stim_wang->set_S0_ta_st( 0.03244077 );
	stim_wang->set_S0_gas_st( 0.04567259 );
	stim_wang->set_S0_vas_st( 0.22487439 );
	stim_wang->set_S0_ham_st( 0.18234013 );
	stim_wang->set_S0_rf_st( 0.06160283 );
	stim_wang->set_S0_glu_st( 0.23347080 );
	stim_wang->set_S0_hfl_st( 0.12699928 );
	stim_wang->set_S0_sol_sw( 0.02112320 );
	stim_wang->set_S0_ta_sw( 0.03390124 );
	stim_wang->set_S0_gas_sw( 0.04991128 );
	stim_wang->set_S0_vas_sw( 0.04197238 );
	stim_wang->set_S0_ham_sw( 0.06194205 );
	stim_wang->set_S0_rf_sw( 0.24415433 );
	stim_wang->set_S0_glu_sw( 0.03650065 );
	stim_wang->set_S0_hfl_sw( 0.03303515 );
	stim_wang->set_G_sol( 3.67422759 );
	stim_wang->set_G_sol_ta( 4.08077106 );
	stim_wang->set_G_gas( 12.79982562 );
	stim_wang->set_G_vas( 3.40946581 );
	stim_wang->set_G_ham( 1.14294208 );
	stim_wang->set_G_glu( 0.09335177 );
	stim_wang->set_G_ta_sw( 3.50001634 );
	stim_wang->set_G_ta_st( 3.65829637 );
	stim_wang->set_G_hfl( 2.29733485 );
	stim_wang->set_G_ham_hfl( 3.23192740 );
	stim_wang->set_l_off_ta_sw( 0.58533422 );
	stim_wang->set_l_off_ta_st( 0.58182585 );
	stim_wang->set_l_off_ham_hfl( 0.54603710 );
	stim_wang->set_l_off_hfl( 0.38599008 );
	stim_wang->set_K_ham( 3.25131974 );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( 4.86501258 );
	stim_wang->set_K_hfl( 8.96733910 );
	stim_wang->set_D_ham( 0.24147505 );
	stim_wang->set_D_glu( 0.08859911 );
	stim_wang->set_D_hfl( 0.38764702 );
	stim_wang->set_theta_ref( 0.06079723 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.77081810 );
	stim_wang->set_si_rf( 0.06036041 );
	stim_wang->set_si_glu( 0.38977916 );
	stim_wang->set_si_hfl( 0.49032514 );
	stim_wang->set_K_sp_vas( 3.72004031 );
	stim_wang->set_K_sp_glu( 2.65754504 );
	stim_wang->set_K_sp_hfl( 3.34446557 );
	stim_wang->set_D_sp_vas( 0.00305964 );
	stim_wang->set_D_sp_glu( 0.03502322 );
	stim_wang->set_D_sp_hfl( 0.04708788 );
	stim_wang->set_theta_k_ref( 0.13116625 );
	stim_wang->set_theta_h_ref0( 0.26917627 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.04585654 );
	stim_wang->set_d_si( 0.40633534 );
	stim_wang->set_k_THETA( 3.23572902 );
	stim_wang->set_k_theta( 6.11257220 );
	stim_wang->set_phi_off_pk( 0.15090566 );

	//init pos
	joints_init->set_T3( 0.49366555 );
	joints_init->set_T3_p( 0.44659499 );
	joints_init->set_R2( 0.05902043 );
	joints_init->set_R2_p( -0.26001614 );
	joints_init->set_r_sh_p( 3.59483145 );
	joints_init->set_r_hip( -0.59527178 );
	joints_init->set_r_hip_p( 3.79746617 );
	joints_init->set_r_knee( 0.35842983 );
	joints_init->set_r_knee_p( 2.14640145 );
	joints_init->set_r_ankle( -0.01341656 );
	joints_init->set_r_ankle_p( -1.83396432 );
	joints_init->set_l_hip( 0.07235296 );
	joints_init->set_l_hip_p( 2.34244933 );
	joints_init->set_l_knee( 0.73266126 );
	joints_init->set_l_knee_p( -1.89124346 );
	joints_init->set_l_ankle( 0.39427577 );
	joints_init->set_l_ankle_p( -1.37191140 );
}
