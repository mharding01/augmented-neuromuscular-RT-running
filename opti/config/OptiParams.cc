#include "OPTI_NAME.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OPTI_NAME::OPTI_NAME()
{

}

/*! \brief destructor
 */
OPTI_NAME::~OPTI_NAME()
{

}

/*! \brief set optimization parameters
 */
void OPTI_NAME::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Oscillator fields
    g_osc->set_gamma_A({.33; 3.0  }); // Longer or shorter than 50% of %50 of gait cycle
    g_osc->set_gamma_B({.33; 3.0 });
    g_osc->set_gamma_C({0.33; 3.0  });

    g_osc->set_eta_A({5.45; 5.53  });  
    g_osc->set_eta_B({4.80; 4.90 });  
    g_osc->set_eta_C({5.62; 6.62 });  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D({3.73; 3.78 });  
    g_osc->set_eta_E({3.64; 3.72 });  

    g_osc->set_P_tau({0.00015 ; 0.102} );    // TODO: Scaled down range by factor of min ~3, max 2 

    g_osc->set_k_HFLrun1( {1.0 ; 6.1});
    g_osc->set_k_HFLrun2( {1.5 ; 10.0});
    g_osc->set_k_HAMrun3( {0.5 ; 7.0});
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_ta_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_gas_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_vas_st( { 0.047 ; 0.71 } );
	stim_wang->set_S0_ham_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_rf_st( { 0.01 ; 0.5 } );
	stim_wang->set_S0_glu_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_hfl_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_sol_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_ta_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_gas_sw( { 0.01 ; 0.2 } );
	stim_wang->set_S0_vas_sw( { 0.01 ; 0.1 } );
	stim_wang->set_S0_ham_sw( { 0.01 ; 0.1 } );
	stim_wang->set_S0_rf_sw( { 0.01 ; 0.5 } );
	stim_wang->set_S0_glu_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_hfl_sw( { 0.01 ; 0.05 } );
	stim_wang->set_G_sol( { 0.97 ; 5.0 } );
	stim_wang->set_G_sol_ta( { 0.4 ; 7.0 } );
	stim_wang->set_G_gas( { 0.0 ; 20.0 } );
	stim_wang->set_G_vas( { 0.82 ; 5.0 } );
	stim_wang->set_G_ham( { 0.0 ; 3.0 } );
	stim_wang->set_G_glu( { 0.0 ; 2.0 } );
	stim_wang->set_G_ta_sw( { 0.55 ; 5.0 } );
	stim_wang->set_G_ta_st( { 0.55 ; 5.0 } );
	stim_wang->set_G_hfl( { 0.17 ; 3.0 } );
	stim_wang->set_G_ham_hfl( { 0.0 ; 10.0 } );
	stim_wang->set_l_off_ta_sw( { 0.50 ; 0.8 } );
	stim_wang->set_l_off_ta_st( { 0.50 ; 0.8 } );
	stim_wang->set_l_off_ham_hfl( { 0.0 ; 0.71 } );
	stim_wang->set_l_off_hfl( { 0.0 ; 0.85 } );
	stim_wang->set_K_ham( { 3.5 ; 6.0 } );  // TODO: shortened, want less HAM in stance
	stim_wang->set_K_glu( { 0.5 ; 14.0 } ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( { 1.0 ; 10.0 } ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( { 0.0 ; 0.5 } );
	stim_wang->set_D_glu( { 0.0 ; 0.5 } );
	stim_wang->set_D_hfl( { 0.0 ; 0.8 } );
	stim_wang->set_theta_ref( { 0.005 ; 0.21 } ); // TODO: trunk lean, realistic max i pi/12=15deg.
	stim_wang->set_si_vas( { 0.3 ; 1.0 } );
	stim_wang->set_si_rf( { 0.0 ; 1.0 } );
	stim_wang->set_si_glu( { 0.2 ; 1.0 } );
	stim_wang->set_si_hfl( { 0.0 ; 1.0 } );
	stim_wang->set_K_sp_vas( { 0.0 ; 5.0 } );
	stim_wang->set_K_sp_glu( { 0.0 ; 5.0 } );
	stim_wang->set_K_sp_hfl( { 0.0 ; 5.0 } );
	stim_wang->set_D_sp_vas( { 0.0 ; 0.1 } );
	stim_wang->set_D_sp_glu( { 0.0 ; 0.1 } );
	stim_wang->set_D_sp_hfl( { 0.0 ; 0.1 } );
	stim_wang->set_theta_k_ref( { 0.0 ; 0.3 } );
	stim_wang->set_theta_h_ref0( { 0.05 ; 0.9 } ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( { -0.3 ; 0.1 } );
	stim_wang->set_d_si( { 0.2 ; 0.7 } );
	stim_wang->set_k_THETA( { 1.0 ; 5.7 } );
	stim_wang->set_k_theta( { 1.0 ; 15.0 } );
	stim_wang->set_phi_off_pk( { 0.05 ; 0.2 } );

	//init pos
	joints_init->set_T3( { 0.46 ; 0.58 } );
	joints_init->set_T3_p( { -0.5 ; 0.5 } );
	joints_init->set_R2( { 0.0 ; 0.3 } );
	joints_init->set_R2_p( { -5.0 ; 5.0 } );
	joints_init->set_r_sh_p( { -5.0 ; 5.0 } );
	joints_init->set_r_hip( { -0.87 ; -0.35 } );
	joints_init->set_r_hip_p( { -5.0 ; 5.0 } );
	joints_init->set_r_knee( { 0.0 ; 0.87 } );
	joints_init->set_r_knee_p( { -5.0 ; 5.0 } );
	joints_init->set_r_ankle( { -0.09 ; 0.09 } );
	joints_init->set_r_ankle_p( { -5.0 ; 5.0 } );
	joints_init->set_l_hip( { -0.09 ; 0.35 } );
	joints_init->set_l_hip_p( { -5.0 ; 5.0 } );
	joints_init->set_l_knee( { 0.35 ; 1.75 } );
	joints_init->set_l_knee_p( { -5.0 ; 5.0 } );
	joints_init->set_l_ankle( { 0.09 ; 0.45 } );
	joints_init->set_l_ankle_p( { -5.0 ; 5.0 } );
}
