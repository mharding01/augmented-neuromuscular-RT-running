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
    g_osc->set_gamma_A(1.57284454);
    g_osc->set_gamma_B(2.45405997);
    g_osc->set_gamma_C(2.67938032);

    g_osc->set_eta_A(5.47405451);  
    g_osc->set_eta_B(4.84326006);  
    g_osc->set_eta_C(5.94686033);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73597291);  
    g_osc->set_eta_E(3.67744054);  

    g_osc->set_P_tau(0.04506615 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 3.64027485);
    g_osc->set_k_HFLrun2( 6.69972053);
    g_osc->set_k_HAMrun( 6.20919127);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03549640 );
	stim_wang->set_S0_ta_st( 0.01385281 );
	stim_wang->set_S0_gas_st( 0.02272210 );
	stim_wang->set_S0_vas_st( 0.46785768 );
	stim_wang->set_S0_ham_st( 0.22933120 );
	stim_wang->set_S0_rf_st( 0.10404763 );
	stim_wang->set_S0_glu_st( 0.13281718 );
	stim_wang->set_S0_hfl_st( 0.09884110 );
	stim_wang->set_S0_sol_sw( 0.01672568 );
	stim_wang->set_S0_ta_sw( 0.01422752 );
	stim_wang->set_S0_gas_sw( 0.07298648 );
	stim_wang->set_S0_vas_sw( 0.06137100 );
	stim_wang->set_S0_ham_sw( 0.07297537 );
	stim_wang->set_S0_rf_sw( 0.49998897 );
	stim_wang->set_S0_glu_sw( 0.01552216 );
	stim_wang->set_S0_hfl_sw( 0.03424812 );
	stim_wang->set_G_sol( 1.95654250 );
	stim_wang->set_G_sol_ta( 2.14963061 );
	stim_wang->set_G_gas( 12.21368971 );
	stim_wang->set_G_vas( 2.89822737 );
	stim_wang->set_G_ham( 0.44398825 );
	stim_wang->set_G_glu( 0.55817780 );
	stim_wang->set_G_ta_sw( 4.14129715 );
	stim_wang->set_G_ta_st( 1.92015903 );
	stim_wang->set_G_hfl( 1.37569551 );
	stim_wang->set_G_ham_hfl( 7.23903898 );
	stim_wang->set_l_off_ta_sw( 0.72440666 );
	stim_wang->set_l_off_ta_st( 0.55433606 );
	stim_wang->set_l_off_ham_hfl( 0.62341933 );
	stim_wang->set_l_off_hfl( 0.75211098 );
	stim_wang->set_K_ham( 3.77578511 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 12.78948832 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.38098711 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.23062641 );
	stim_wang->set_D_glu( 0.27511060 );
	stim_wang->set_D_hfl( 0.10109232 );
	stim_wang->set_theta_ref( 0.05009541 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.86249140 );
	stim_wang->set_si_rf( 0.10405186 );
	stim_wang->set_si_glu( 0.52270510 );
	stim_wang->set_si_hfl( 0.26721774 );
	stim_wang->set_K_sp_vas( 0.71144024 );
	stim_wang->set_K_sp_glu( 3.99731413 );
	stim_wang->set_K_sp_hfl( 3.40289466 );
	stim_wang->set_D_sp_vas( 0.03684102 );
	stim_wang->set_D_sp_glu( 0.06050738 );
	stim_wang->set_D_sp_hfl( 0.03614367 );
	stim_wang->set_theta_k_ref( 0.13155785 );
	stim_wang->set_theta_h_ref0( 0.36720498 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.22978568 );
	stim_wang->set_d_si( 0.41614757 );
	stim_wang->set_k_THETA( 3.60531071 );
	stim_wang->set_k_theta( 7.94021067 );
	stim_wang->set_phi_off_pk( 0.13259622 );

	//init pos
	joints_init->set_T3( 0.49708324 );
	joints_init->set_T3_p( -0.23324899 );
	joints_init->set_R2( 0.22176592 );
	joints_init->set_R2_p( -0.45637130 );
	joints_init->set_r_sh_p( -4.75884488 );
	joints_init->set_r_hip( -0.74800592 );
	joints_init->set_r_hip_p( 0.30801618 );
	joints_init->set_r_knee( 0.56595035 );
	joints_init->set_r_knee_p( 3.01108854 );
	joints_init->set_r_ankle( 0.06633811 );
	joints_init->set_r_ankle_p( 3.58106893 );
	joints_init->set_l_hip( 0.30562051 );
	joints_init->set_l_hip_p( -2.87585070 );
	joints_init->set_l_knee( 0.71733129 );
	joints_init->set_l_knee_p( 0.07188957 );
	joints_init->set_l_ankle( 0.13404878 );
	joints_init->set_l_ankle_p( 0.52596502 );
}
