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
    g_osc->set_gamma_A(1.55044962);
    g_osc->set_gamma_B(2.51059438);
    g_osc->set_gamma_C(2.84312094);

    g_osc->set_eta_A(5.47554574);  
    g_osc->set_eta_B(4.80296459);  
    g_osc->set_eta_C(5.88121396);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.76496277);  
    g_osc->set_eta_E(3.68886888);  

    g_osc->set_P_tau(0.04669179 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 3.57318589);
    g_osc->set_k_HFLrun2( 2.61672734);
    g_osc->set_k_HAMrun( 5.34024805);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.01560274 );
	stim_wang->set_S0_ta_st( 0.03734385 );
	stim_wang->set_S0_gas_st( 0.03738314 );
	stim_wang->set_S0_vas_st( 0.19433172 );
	stim_wang->set_S0_ham_st( 0.17992431 );
	stim_wang->set_S0_rf_st( 0.09106773 );
	stim_wang->set_S0_glu_st( 0.07377309 );
	stim_wang->set_S0_hfl_st( 0.10027111 );
	stim_wang->set_S0_sol_sw( 0.03595114 );
	stim_wang->set_S0_ta_sw( 0.04058931 );
	stim_wang->set_S0_gas_sw( 0.04201207 );
	stim_wang->set_S0_vas_sw( 0.01383497 );
	stim_wang->set_S0_ham_sw( 0.07828039 );
	stim_wang->set_S0_rf_sw( 0.38340967 );
	stim_wang->set_S0_glu_sw( 0.01449282 );
	stim_wang->set_S0_hfl_sw( 0.04293133 );
	stim_wang->set_G_sol( 2.49124408 );
	stim_wang->set_G_sol_ta( 5.28611210 );
	stim_wang->set_G_gas( 10.36703878 );
	stim_wang->set_G_vas( 1.38131082 );
	stim_wang->set_G_ham( 1.80372463 );
	stim_wang->set_G_glu( 1.67414316 );
	stim_wang->set_G_ta_sw( 4.77593695 );
	stim_wang->set_G_ta_st( 2.81481530 );
	stim_wang->set_G_hfl( 2.53792686 );
	stim_wang->set_G_ham_hfl( 5.01062107 );
	stim_wang->set_l_off_ta_sw( 0.64156075 );
	stim_wang->set_l_off_ta_st( 0.77895771 );
	stim_wang->set_l_off_ham_hfl( 0.31456088 );
	stim_wang->set_l_off_hfl( 0.40979921 );
	stim_wang->set_K_ham( 3.34462949 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 7.47797755 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 5.27898904 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.23075193 );
	stim_wang->set_D_glu( 0.18215873 );
	stim_wang->set_D_hfl( 0.29516296 );
	stim_wang->set_theta_ref( 0.03610345 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.55839219 );
	stim_wang->set_si_rf( 0.13290048 );
	stim_wang->set_si_glu( 0.74263710 );
	stim_wang->set_si_hfl( 0.82134241 );
	stim_wang->set_K_sp_vas( 2.32038087 );
	stim_wang->set_K_sp_glu( 4.44580597 );
	stim_wang->set_K_sp_hfl( 1.58305485 );
	stim_wang->set_D_sp_vas( 0.07529897 );
	stim_wang->set_D_sp_glu( 0.01790749 );
	stim_wang->set_D_sp_hfl( 0.02576595 );
	stim_wang->set_theta_k_ref( 0.27846301 );
	stim_wang->set_theta_h_ref0( 0.46240720 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( 0.02524741 );
	stim_wang->set_d_si( 0.45192451 );
	stim_wang->set_k_THETA( 5.58488654 );
	stim_wang->set_k_theta( 11.31531866 );
	stim_wang->set_phi_off_pk( 0.08104193 );

	//init pos
	joints_init->set_T3( 0.50588215 );
	joints_init->set_T3_p( -0.28694451 );
	joints_init->set_R2( 0.18712496 );
	joints_init->set_R2_p( 0.45385369 );
	joints_init->set_r_sh_p( -2.31176743 );
	joints_init->set_r_hip( -0.62450269 );
	joints_init->set_r_hip_p( -3.07102766 );
	joints_init->set_r_knee( 0.60518502 );
	joints_init->set_r_knee_p( -0.69816064 );
	joints_init->set_r_ankle( -0.07221544 );
	joints_init->set_r_ankle_p( 4.00862328 );
	joints_init->set_l_hip( 0.18418751 );
	joints_init->set_l_hip_p( -2.14556346 );
	joints_init->set_l_knee( 0.35198752 );
	joints_init->set_l_knee_p( 2.27297170 );
	joints_init->set_l_ankle( 0.16750927 );
	joints_init->set_l_ankle_p( 3.73506353 );
}
