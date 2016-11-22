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
    g_osc->set_gamma_A(1.56850348);
    g_osc->set_gamma_B(2.39567561);
    g_osc->set_gamma_C(2.85809945);

    g_osc->set_eta_A(5.47973339);  
    g_osc->set_eta_B(4.86021965);  
    g_osc->set_eta_C(5.68556845);  
    g_osc->set_eta_D(3.73115505);  
    g_osc->set_eta_E(3.67409776);  

    g_osc->set_P_tau(0.05650028 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 4.83565368);
    g_osc->set_k_HFLrun2( 3.18855063);
    g_osc->set_k_HFLrun3( 1.63895273);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.02071842 );
	stim_wang->set_S0_ta_st( 0.03086535 );
	stim_wang->set_S0_gas_st( 0.04997144 );
	stim_wang->set_S0_vas_st( 0.23263577 );
	stim_wang->set_S0_ham_st( 0.22770965 );
	stim_wang->set_S0_rf_st( 0.16418810 );
	stim_wang->set_S0_glu_st( 0.15845287 );
	stim_wang->set_S0_hfl_st( 0.07635223 );
	stim_wang->set_S0_sol_sw( 0.01244428 );
	stim_wang->set_S0_ta_sw( 0.03658818 );
	stim_wang->set_S0_gas_sw( 0.08664081 );
	stim_wang->set_S0_vas_sw( 0.02797499 );
	stim_wang->set_S0_ham_sw( 0.08434195 );
	stim_wang->set_S0_rf_sw( 0.21277983 );
	stim_wang->set_S0_glu_sw( 0.03680804 );
	stim_wang->set_S0_hfl_sw( 0.01706187 );
	stim_wang->set_G_sol( 2.61977585 );
	stim_wang->set_G_sol_ta( 4.09308436 );
	stim_wang->set_G_gas( 5.74566499 );
	stim_wang->set_G_vas( 1.18666554 );
	stim_wang->set_G_ham( 1.03211902 );
	stim_wang->set_G_glu( 0.08289130 );
	stim_wang->set_G_ta_sw( 3.82825297 );
	stim_wang->set_G_ta_st( 1.24360521 );
	stim_wang->set_G_hfl( 2.55253628 );
	stim_wang->set_G_ham_hfl( 1.78906314 );
	stim_wang->set_l_off_ta_sw( 0.53789562 );
	stim_wang->set_l_off_ta_st( 0.55230480 );
	stim_wang->set_l_off_ham_hfl( 0.69316680 );
	stim_wang->set_l_off_hfl( 0.31257903 );
	stim_wang->set_K_ham( 4.16753347 );
	stim_wang->set_K_glu( 6.42981904 );
	stim_wang->set_K_hfl( 7.82402222 );
	stim_wang->set_D_ham( 0.37159767 );
	stim_wang->set_D_glu( 0.26143899 );
	stim_wang->set_D_hfl( 0.26086884 );
	stim_wang->set_theta_ref( 0.10230319 );
	stim_wang->set_si_vas( 0.77112029 );
	stim_wang->set_si_rf( 0.00428991 );
	stim_wang->set_si_glu( 0.59002933 );
	stim_wang->set_si_hfl( 0.18741617 );
	stim_wang->set_K_sp_vas( 2.60284228 );
	stim_wang->set_K_sp_glu( 2.74364806 );
	stim_wang->set_K_sp_hfl( 2.64262703 );
	stim_wang->set_D_sp_vas( 0.03040695 );
	stim_wang->set_D_sp_glu( 0.00324867 );
	stim_wang->set_D_sp_hfl( 0.05819196 );
	stim_wang->set_theta_k_ref( 0.24423110 );
	stim_wang->set_theta_h_ref0( 0.16504324 );
	stim_wang->set_d_sp( 0.02673103 );
	stim_wang->set_d_si( 0.58236287 );
	stim_wang->set_k_THETA( 4.60857213 );
	stim_wang->set_k_theta( 11.20795615 );
	stim_wang->set_phi_off_pk( 0.14024130 );

	//init pos
	joints_init->set_T3( 0.52946477 );
	joints_init->set_T3_p( 0.25341436 );
	joints_init->set_R2( 0.24137369 );
	joints_init->set_R2_p( -0.65454933 );
	joints_init->set_r_sh_p( -2.35545254 );
	joints_init->set_r_hip( -0.56867945 );
	joints_init->set_r_hip_p( 0.95925193 );
	joints_init->set_r_knee( 0.20539201 );
	joints_init->set_r_knee_p( -0.83371614 );
	joints_init->set_r_ankle( 0.03292537 );
	joints_init->set_r_ankle_p( 1.67396772 );
	joints_init->set_l_hip( -0.00894789 );
	joints_init->set_l_hip_p( 4.55738902 );
	joints_init->set_l_knee( 1.07985372 );
	joints_init->set_l_knee_p( 1.83225495 );
	joints_init->set_l_ankle( 0.31495684 );
	joints_init->set_l_ankle_p( 1.35304225 );
}
