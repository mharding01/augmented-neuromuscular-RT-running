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

    g_osc->set_beta_A({4.5; 6.5});
    g_osc->set_beta_B({4.0; 6.0 });
    g_osc->set_beta_C({3.0; 6.0 });

    g_osc->set_eta_A({5.45; 5.53  });  
    g_osc->set_eta_B({4.80; 4.90 });  
    g_osc->set_eta_C({5.2; 5.8});  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D({3.73; 3.78 });  
    g_osc->set_eta_E({3.64; 3.72 });

    g_osc->set_eta_F({2.5 ; 5.0 });
    g_osc->set_eta_G({3.0 ; 5.5 }); 

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk({0.01; 0.2}); // 0.0609
    g_osc->set_P_theta_hip({0.05; 0.25}); // 0.1616
    g_osc->set_P_k_theta({4.5; 10.5}); // 8.6158
    
    // Linear
    g_osc->set_P_k_HFLrun1({2.5; 8.0}); // 5.1025
    g_osc->set_p_k_HFLrun1({-5.5; -2.0}); // -3.1544

    g_osc->set_P_G_SOL_TA({3.0;8.5}); // 5.3225
    g_osc->set_p_G_SOL_TA({7.5; 12.5}); // 9.0977

    g_osc->set_P_G_GAS({5.5; 10.5}); // 7.5656
    g_osc->set_p_G_GAS({-15.5; -10.0}); // -13.7575

    g_osc->set_P_G_VAS({1.0; 3.0}); // 1.6058
    g_osc->set_p_G_VAS({1.0; 3.5}); // 2.0708
    
    // Quadratic
    g_osc->set_P_tau({0.010; .10}); // 0.0465
    g_osc->set_p_tau({-.2; 0.2}); // -.0168
    g_osc->set_p2_tau({-0.3;0.1});// -.0277
    
    g_osc->set_P_k_HFLrun2({3.0;9.0}); // 5.8581
    g_osc->set_p_k_HFLrun2({27.0;33.0});// 29.9698
    g_osc->set_p2_k_HFLrun2({80.0;90.0});//84.8007
    
    g_osc->set_P_k_HAMrun({2.0;5.7}); // 3.8158
    g_osc->set_p_k_HAMrun({7.2;10.5}); // 8.7087
    g_osc->set_p2_k_HAMrun({29.0;37.0});//34.0513

    g_osc->set_P_G_SOL({1.2;5.5}); // 2.9365
    g_osc->set_p_G_SOL({1.0;6.5}); // 3.4425
    g_osc->set_p2_G_SOL({10.0;16.0}); // 13.5518

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
	stim_wang->set_l_off_ta_sw( { 0.50 ; 0.8 } );
	stim_wang->set_l_off_ta_st( { 0.50 ; 0.8 } );
	stim_wang->set_K_ham( { 0.5 ; 6.0 } );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( { 0.5 ; 14.0 } ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( { 1.0 ; 10.0 } ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( { 0.0 ; 0.5 } );
	stim_wang->set_D_glu( { 0.0 ; 0.5 } );
	stim_wang->set_D_hfl( { 0.0 ; 0.8 } );
	stim_wang->set_si_vas( { 0.3 ; 1.0 } );
	stim_wang->set_si_rf( { 0.0 ; 1.0 } );
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
