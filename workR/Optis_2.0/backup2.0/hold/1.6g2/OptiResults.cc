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
    g_osc->set_gamma_A(1.52185376);
    g_osc->set_gamma_B(2.50837137);
    g_osc->set_gamma_C(2.86904367);

    g_osc->set_eta_A(5.45650149);  
    g_osc->set_eta_B(4.88097194);  
    g_osc->set_eta_C(5.83264208);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73937799);  
    g_osc->set_eta_E(3.67266431);  

    g_osc->set_P_tau(0.05572105 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 5.80381001);
    g_osc->set_k_HFLrun2( 5.13912547);
    g_osc->set_k_HAMrun( 1.39502570);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03248779 );
	stim_wang->set_S0_ta_st( 0.02907832 );
	stim_wang->set_S0_gas_st( 0.04137696 );
	stim_wang->set_S0_vas_st( 0.31387869 );
	stim_wang->set_S0_ham_st( 0.11367431 );
	stim_wang->set_S0_rf_st( 0.32829679 );
	stim_wang->set_S0_glu_st( 0.18332393 );
	stim_wang->set_S0_hfl_st( 0.01398852 );
	stim_wang->set_S0_sol_sw( 0.01546136 );
	stim_wang->set_S0_ta_sw( 0.02770919 );
	stim_wang->set_S0_gas_sw( 0.06678464 );
	stim_wang->set_S0_vas_sw( 0.03449787 );
	stim_wang->set_S0_ham_sw( 0.07389618 );
	stim_wang->set_S0_rf_sw( 0.26316298 );
	stim_wang->set_S0_glu_sw( 0.03832461 );
	stim_wang->set_S0_hfl_sw( 0.04861854 );
	stim_wang->set_G_sol( 3.52228566 );
	stim_wang->set_G_sol_ta( 1.81616602 );
	stim_wang->set_G_gas( 5.00762836 );
	stim_wang->set_G_vas( 1.64510935 );
	stim_wang->set_G_ham( 0.89785789 );
	stim_wang->set_G_glu( 0.94009105 );
	stim_wang->set_G_ta_sw( 2.64344226 );
	stim_wang->set_G_ta_st( 1.54449566 );
	stim_wang->set_G_hfl( 2.95205179 );
	stim_wang->set_G_ham_hfl( 4.05655136 );
	stim_wang->set_l_off_ta_sw( 0.56542431 );
	stim_wang->set_l_off_ta_st( 0.62927006 );
	stim_wang->set_l_off_ham_hfl( 0.06385190 );
	stim_wang->set_l_off_hfl( 0.56513174 );
	stim_wang->set_K_ham( 4.40366929 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 11.32840743 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.97038006 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.06487235 );
	stim_wang->set_D_glu( 0.35899181 );
	stim_wang->set_D_hfl( 0.37512277 );
	stim_wang->set_theta_ref( 0.09109704 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.72225274 );
	stim_wang->set_si_rf( 0.80745579 );
	stim_wang->set_si_glu( 0.64732538 );
	stim_wang->set_si_hfl( 0.68511190 );
	stim_wang->set_K_sp_vas( 3.16219557 );
	stim_wang->set_K_sp_glu( 2.44336603 );
	stim_wang->set_K_sp_hfl( 0.52102600 );
	stim_wang->set_D_sp_vas( 0.03472973 );
	stim_wang->set_D_sp_glu( 0.09063653 );
	stim_wang->set_D_sp_hfl( 0.02135524 );
	stim_wang->set_theta_k_ref( 0.05870901 );
	stim_wang->set_theta_h_ref0( 0.57047536 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( -0.22901182 );
	stim_wang->set_d_si( 0.35689973 );
	stim_wang->set_k_THETA( 3.33702213 );
	stim_wang->set_k_theta( 9.62430097 );
	stim_wang->set_phi_off_pk( 0.09185164 );

	//init pos
	joints_init->set_T3( 0.50294443 );
	joints_init->set_T3_p( 0.23626531 );
	joints_init->set_R2( 0.12682485 );
	joints_init->set_R2_p( 0.80511795 );
	joints_init->set_r_sh_p( 2.53544439 );
	joints_init->set_r_hip( -0.71056800 );
	joints_init->set_r_hip_p( -1.53180318 );
	joints_init->set_r_knee( 0.79956251 );
	joints_init->set_r_knee_p( -3.34208176 );
	joints_init->set_r_ankle( -0.01540988 );
	joints_init->set_r_ankle_p( 3.21756052 );
	joints_init->set_l_hip( 0.16873244 );
	joints_init->set_l_hip_p( 1.78830149 );
	joints_init->set_l_knee( 1.00176359 );
	joints_init->set_l_knee_p( -2.91526485 );
	joints_init->set_l_ankle( 0.15320303 );
	joints_init->set_l_ankle_p( -0.55997066 );
}
