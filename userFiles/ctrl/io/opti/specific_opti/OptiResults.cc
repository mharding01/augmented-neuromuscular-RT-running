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
    g_osc->set_gamma_A(1.57481492);
    g_osc->set_gamma_B(2.42292289);
    g_osc->set_gamma_C(2.80724348);

    g_osc->set_eta_A(5.49607969);  
    g_osc->set_eta_B(4.80979787);  
    g_osc->set_eta_C(5.66861122);  
    g_osc->set_eta_D(3.74768032);  
    g_osc->set_eta_E(3.70539153);  

    g_osc->set_P_tau(0.05620997 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 3.24576633);
    g_osc->set_k_HFLrun2( 6.03128862);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
    stim_wang->set_cpg_ctrl_thresh_t( 2.0);
	stim_wang->set_S0_sol_st( 0.01342274 );
	stim_wang->set_S0_ta_st( 0.04327635 );
	stim_wang->set_S0_gas_st( 0.03055984 );
	stim_wang->set_S0_vas_st( 0.33708271 );
	stim_wang->set_S0_ham_st( 0.15338699 );
	stim_wang->set_S0_rf_st( 0.41272986 );
	stim_wang->set_S0_glu_st( 0.16740316 );
	stim_wang->set_S0_hfl_st( 0.16358407 );
	stim_wang->set_S0_sol_sw( 0.01064221 );
	stim_wang->set_S0_ta_sw( 0.01689723 );
	stim_wang->set_S0_gas_sw( 0.02947748 );
	stim_wang->set_S0_vas_sw( 0.04289643 );
	stim_wang->set_S0_ham_sw( 0.07886212 );
	stim_wang->set_S0_rf_sw( 0.09689076 );
	stim_wang->set_S0_glu_sw( 0.01831303 );
	stim_wang->set_S0_hfl_sw( 0.02238741 );
	stim_wang->set_G_sol( 2.46876618 );
	stim_wang->set_G_sol_ta( 4.89275307 );
	stim_wang->set_G_gas( 7.58467348 );
	stim_wang->set_G_vas( 2.11762041 );
	stim_wang->set_G_ham( 0.69385174 );
	stim_wang->set_G_glu( 0.25365423 );
	stim_wang->set_G_ta_sw( 3.15903862 );
	stim_wang->set_G_ta_st( 2.72258881 );
	stim_wang->set_G_hfl( 1.40737689 );
	stim_wang->set_G_ham_hfl( 2.37100555 );
	stim_wang->set_l_off_ta_sw( 0.64427246 );
	stim_wang->set_l_off_ta_st( 0.76310567 );
	stim_wang->set_l_off_ham_hfl( 0.45523415 );
	stim_wang->set_l_off_hfl( 0.04989315 );
	stim_wang->set_K_ham( 6.000 ); //8.62803799 );
	stim_wang->set_K_glu( 3.85820695 );
	stim_wang->set_K_hfl( 14.15241326 );
	stim_wang->set_D_ham( 0.31109949 );
	stim_wang->set_D_glu( 0.15076655 );
	stim_wang->set_D_hfl( 0.33315609 );
	stim_wang->set_theta_ref( 0.07152984 ); // TODO: made this smaller as stimwang runner was unstable
	stim_wang->set_si_vas( 0.88257734 );
	stim_wang->set_si_rf( 0.22914131 );
	stim_wang->set_si_glu( 0.42110543 );
	stim_wang->set_si_hfl( 0.74789922 );
	stim_wang->set_K_sp_vas( 0.34178559 );
	stim_wang->set_K_sp_glu( 2.72229058 );
	stim_wang->set_K_sp_hfl( 4.97099601 );
	stim_wang->set_D_sp_vas( 0.01351337 );
	stim_wang->set_D_sp_glu( 0.06949506 );
	stim_wang->set_D_sp_hfl( 0.01950122 );
	stim_wang->set_theta_k_ref( 0.12988652 );
	stim_wang->set_theta_h_ref0( 0.37958984 ); // TODO: made its lower bound smaller
	stim_wang->set_d_sp( 0.09847997 );
	stim_wang->set_d_si( 0.50439297 );
	stim_wang->set_k_THETA( 3.15436072 );
	stim_wang->set_k_theta( 6.40349221 );
	stim_wang->set_phi_off_pk( 0.07757075 );

	//init pos
	joints_init->set_T3( 0.55569041 );
	joints_init->set_T3_p( -0.40494533 );
	joints_init->set_R2( 0.17574419 );
	joints_init->set_R2_p( -2.07824217 );
	joints_init->set_r_sh_p( -2.40250490 );
	joints_init->set_r_hip( -0.55283592 );
	joints_init->set_r_hip_p( 2.46097238 );
	joints_init->set_r_knee( 0.43047202 );
	joints_init->set_r_knee_p( 0.55480295 );
	joints_init->set_r_ankle( -0.08233351 );
	joints_init->set_r_ankle_p( 2.33133470 );
	joints_init->set_l_hip( 0.16905580 );
	joints_init->set_l_hip_p( 1.09006415 );
	joints_init->set_l_knee( 1.38285894 );
	joints_init->set_l_knee_p( 4.93610257 );
	joints_init->set_l_ankle( 0.13399933 );
	joints_init->set_l_ankle_p( -0.26499658 );
}
