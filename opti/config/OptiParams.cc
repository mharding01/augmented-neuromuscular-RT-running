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
    g_osc->set_gamma_A({1.30; 1.48});
    g_osc->set_gamma_B({2.35; 2.55 });
    g_osc->set_gamma_C({2.75; 2.95 });

    g_osc->set_eta_A({5.45; 5.53  });  
    g_osc->set_eta_B({4.80; 4.90 });  
    g_osc->set_eta_C({5.2; 5.8});  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D({3.73; 3.78 });  
    g_osc->set_eta_E({3.64; 3.72 });  

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk({0.10; .19});
    g_osc->set_P_theta_hip({0.35; 0.45});
    g_osc->set_P_k_theta({12.0; 14.0});
    
    // Linear
    g_osc->set_P_k_HFLrun1({7.5; 9.0});
    g_osc->set_p_k_HFLrun1({-3.5; -2.0});

    g_osc->set_P_G_SOL_TA({6.0;8.5});
    g_osc->set_p_G_SOL_TA({6.0; 7.5});

    g_osc->set_P_G_GAS({12.0; 13.5});
    g_osc->set_p_G_GAS({-7.0; -5.0});

    g_osc->set_P_G_VAS({2.0; 3.0});
    g_osc->set_p_G_VAS({5.0; 6.5});
    
    // Quadratic
    g_osc->set_P_tau({0.010; .10});
    g_osc->set_p_tau({-.2; 0.2});
    g_osc->set_p2_tau({-0.3;0.1});
    
    g_osc->set_P_k_HFLrun2({11.0;13.0});
    g_osc->set_p_k_HFLrun2({11.0;14.0});
    g_osc->set_p2_k_HFLrun2({25.0;31.0});
    
    g_osc->set_P_k_HAMrun({7.0;7.7});
    g_osc->set_p_k_HAMrun({0.2;1.5});
    g_osc->set_p2_k_HAMrun({16.0;20.0});

    g_osc->set_P_G_SOL({5.2;5.5});
    g_osc->set_p_G_SOL({6.0;7.5});
    g_osc->set_p2_G_SOL({0.0;2.0});

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
	stim_wang->set_K_ham( { 0.5 ; 6.0 } );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( { 0.5 ; 14.0 } ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( { 1.0 ; 10.0 } ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( { 0.0 ; 0.5 } );
	stim_wang->set_D_glu( { 0.0 ; 0.5 } );
	stim_wang->set_D_hfl( { 0.0 ; 0.8 } );
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
	stim_wang->set_d_sp( { -0.3 ; 0.1 } );
	stim_wang->set_d_si( { 0.2 ; 0.7 } );
	stim_wang->set_k_THETA( { 1.0 ; 5.7 } );
	stim_wang->set_phi_off_pk( { 0.05 ; 0.2 } );
}
