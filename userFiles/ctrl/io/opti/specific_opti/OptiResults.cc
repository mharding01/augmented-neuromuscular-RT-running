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
    g_osc->set_gamma_A(1.50566969);
    g_osc->set_gamma_B(2.48852412);
    g_osc->set_gamma_C(2.81611728);

    g_osc->set_eta_A(5.47598250);  
    g_osc->set_eta_B(4.83709023);  
    g_osc->set_eta_C(5.74443359);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.73251892);  
    g_osc->set_eta_E(3.67559364);  

    // Fitting parameters for improved vel tracking
    g_osc->set_P_theta_trunk(0.10156826);
    g_osc->set_P_theta_hip(0.28365856);
    g_osc->set_P_k_theta(7.98175890);
    
    // Linear
    g_osc->set_P_k_HFLrun1(6.63614326);
    g_osc->set_p_k_HFLrun1(-3.46380059);

    g_osc->set_P_G_SOL_TA(6.91348931);
    g_osc->set_p_G_SOL_TA(7.18550717);

    g_osc->set_P_G_GAS(10.80375366);
    g_osc->set_p_G_GAS(-6.24075094);

    g_osc->set_P_G_VAS(2.53682663);
    g_osc->set_p_G_VAS(4.67265811);
    
    // Quadratic
    g_osc->set_P_tau(0.05286006);
    g_osc->set_p_tau(-0.01738798);
    g_osc->set_p2_tau(-0.01692750);
    
    g_osc->set_P_k_HFLrun2(9.50334867);
    g_osc->set_p_k_HFLrun2(24.64874783);
    g_osc->set_p2_k_HFLrun2(60.97031920);
    
    g_osc->set_P_k_HAMrun(5.99355253);
    g_osc->set_p_k_HAMrun(1.16341140);
    g_osc->set_p2_k_HAMrun(25.30023408);

    g_osc->set_P_G_SOL(4.60461364);
    g_osc->set_p_G_SOL(4.91556049);
    g_osc->set_p2_G_SOL(5.91220672);

	// Wang stimulations
    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.01505076 );
	stim_wang->set_S0_ta_st( 0.02757871 );
	stim_wang->set_S0_gas_st( 0.01370363 );
	stim_wang->set_S0_vas_st( 0.67385526 );
	stim_wang->set_S0_ham_st( 0.12891059 );
	stim_wang->set_S0_rf_st( 0.07379027 );
	stim_wang->set_S0_glu_st( 0.29912969 );
	stim_wang->set_S0_hfl_st( 0.02124526 );
	stim_wang->set_S0_sol_sw( 0.01554241 );
	stim_wang->set_S0_ta_sw( 0.02663306 );
	stim_wang->set_S0_gas_sw( 0.13651294 );
	stim_wang->set_S0_vas_sw( 0.02688331 );
	stim_wang->set_S0_ham_sw( 0.08888485 );
	stim_wang->set_S0_rf_sw( 0.23686505 );
	stim_wang->set_S0_glu_sw( 0.01867327 );
	stim_wang->set_S0_hfl_sw( 0.02575706 );
	stim_wang->set_G_ham( 0.97596149 );
	stim_wang->set_G_glu( 0.32903988 );
	stim_wang->set_G_ta_sw( 3.71697456 );
	stim_wang->set_G_ta_st( 1.02220560 );
	stim_wang->set_G_hfl( 1.98667896 );
	stim_wang->set_G_ham_hfl( 5.20267861 );
	stim_wang->set_l_off_ta_sw( 0.54843458 );
	stim_wang->set_l_off_ta_st( 0.60285498 );
	stim_wang->set_l_off_ham_hfl( 0.23192924 );
	stim_wang->set_l_off_hfl( 0.18255622 );
	stim_wang->set_K_ham( 5.44161892 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 10.95547689 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.94857112 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.10817993 );
	stim_wang->set_D_glu( 0.33313422 );
	stim_wang->set_D_hfl( 0.20307032 );
	stim_wang->set_si_vas( 0.78205670 );
	stim_wang->set_si_rf( 0.49977394 );
	stim_wang->set_si_glu( 0.55566786 );
	stim_wang->set_si_hfl( 0.41336175 );
	stim_wang->set_K_sp_vas( 0.21182800 );
	stim_wang->set_K_sp_glu( 4.13861354 );
	stim_wang->set_K_sp_hfl( 1.34631739 );
	stim_wang->set_D_sp_vas( 0.04788856 );
	stim_wang->set_D_sp_glu( 0.02669805 );
	stim_wang->set_D_sp_hfl( 0.08271770 );
	stim_wang->set_theta_k_ref( 0.01609366 );
	stim_wang->set_d_sp( 0.09988371 );
	stim_wang->set_d_si( 0.35941048 );
	stim_wang->set_k_THETA( 1.73604641 );
	stim_wang->set_phi_off_pk( 0.05373537 );
}
