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
    g_osc->set_gamma_A(1.51058861);
    g_osc->set_gamma_B(2.47834232);
    g_osc->set_gamma_C(2.93742890);

    g_osc->set_eta_A(5.52652176);  
    g_osc->set_eta_B(4.81115287);  
    g_osc->set_eta_C(5.79932476);  // TODO: N3 and N6, controls how soon appear 
    g_osc->set_eta_D(3.76204015);  
    g_osc->set_eta_E(3.71710006);  

    g_osc->set_P_tau(0.06951043 ); // Taken from master commit: "All_1 results, 1459 score"

    g_osc->set_k_HFLrun1( 0.48272872);
    g_osc->set_k_HFLrun2( 7.43730138);
    g_osc->set_k_HAMrun( 3.38334245);
    
	// Wang stimulations

    // TODO: added cpg_ctrl_thresh_t parameter for optimization
	stim_wang->set_S0_sol_st( 0.03886870 );
	stim_wang->set_S0_ta_st( 0.02171512 );
	stim_wang->set_S0_gas_st( 0.03075254 );
	stim_wang->set_S0_vas_st( 0.44819239 );
	stim_wang->set_S0_ham_st( 0.26716848 );
	stim_wang->set_S0_rf_st( 0.39070959 );
	stim_wang->set_S0_glu_st( 0.03956022 );
	stim_wang->set_S0_hfl_st( 0.03204395 );
	stim_wang->set_S0_sol_sw( 0.03575322 );
	stim_wang->set_S0_ta_sw( 0.02842026 );
	stim_wang->set_S0_gas_sw( 0.15987940 );
	stim_wang->set_S0_vas_sw( 0.06230985 );
	stim_wang->set_S0_ham_sw( 0.04441489 );
	stim_wang->set_S0_rf_sw( 0.37011824 );
	stim_wang->set_S0_glu_sw( 0.02905096 );
	stim_wang->set_S0_hfl_sw( 0.04939852 );
	stim_wang->set_G_sol( 3.09312214 );
	stim_wang->set_G_sol_ta( 5.41931445 );
	stim_wang->set_G_gas( 1.62238298 );
	stim_wang->set_G_vas( 2.30794205 );
	stim_wang->set_G_ham( 2.67817885 );
	stim_wang->set_G_glu( 1.80256314 );
	stim_wang->set_G_ta_sw( 4.99992001 );
	stim_wang->set_G_ta_st( 3.09771449 );
	stim_wang->set_G_hfl( 1.19679976 );
	stim_wang->set_G_ham_hfl( 3.56436578 );
	stim_wang->set_l_off_ta_sw( 0.66142119 );
	stim_wang->set_l_off_ta_st( 0.63427583 );
	stim_wang->set_l_off_ham_hfl( 0.08485718 );
	stim_wang->set_l_off_hfl( 0.46517465 );
	stim_wang->set_K_ham( 5.70544745 );  // TODO: want less HAM in stance
	stim_wang->set_K_glu( 6.96539516 ); // Prefer GLU to handle much of trunk lean
	stim_wang->set_K_hfl( 4.84701374 ); // Prefer HFL not to be too active
	stim_wang->set_D_ham( 0.47882854 );
	stim_wang->set_D_glu( 0.35849814 );
	stim_wang->set_D_hfl( 0.19123199 );
	stim_wang->set_theta_ref( 0.09330898 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	stim_wang->set_si_vas( 0.86868440 );
	stim_wang->set_si_rf( 0.09978235 );
	stim_wang->set_si_glu( 0.70070334 );
	stim_wang->set_si_hfl( 0.37918894 );
	stim_wang->set_K_sp_vas( 1.85969115 );
	stim_wang->set_K_sp_glu( 2.37561404 );
	stim_wang->set_K_sp_hfl( 3.39378766 );
	stim_wang->set_D_sp_vas( 0.05627901 );
	stim_wang->set_D_sp_glu( 0.07712219 );
	stim_wang->set_D_sp_hfl( 0.00069816 );
	stim_wang->set_theta_k_ref( 0.06615299 );
	stim_wang->set_theta_h_ref0( 0.25420279 ); // TODO: made its lower bound smaller, come back to this later
	stim_wang->set_d_sp( 0.06240429 );
	stim_wang->set_d_si( 0.37298095 );
	stim_wang->set_k_THETA( 5.53843535 );
	stim_wang->set_k_theta( 11.97284889 );
	stim_wang->set_phi_off_pk( 0.11808692 );

	//init pos
	joints_init->set_T3( 0.46698324 );
	joints_init->set_T3_p( -0.16312731 );
	joints_init->set_R2( 0.05760236 );
	joints_init->set_R2_p( 1.28785977 );
	joints_init->set_r_sh_p( 0.41362214 );
	joints_init->set_r_hip( -0.64218595 );
	joints_init->set_r_hip_p( 1.69184095 );
	joints_init->set_r_knee( 0.50490467 );
	joints_init->set_r_knee_p( -2.59394115 );
	joints_init->set_r_ankle( -0.01457556 );
	joints_init->set_r_ankle_p( -3.10882213 );
	joints_init->set_l_hip( 0.12575733 );
	joints_init->set_l_hip_p( -3.10763319 );
	joints_init->set_l_knee( 1.47682491 );
	joints_init->set_l_knee_p( 0.93732960 );
	joints_init->set_l_ankle( 0.27502998 );
	joints_init->set_l_ankle_p( -1.82856530 );
}
