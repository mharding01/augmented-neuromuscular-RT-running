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
    g_osc->set_gamma_A(1.51266623);
    g_osc->set_gamma_B(2.42979814);
    g_osc->set_gamma_C(2.88511450);

    g_osc->set_eta_A(5.50068958);  
    g_osc->set_eta_B(4.87883031);  
    g_osc->set_eta_C(5.66782244);  
    g_osc->set_eta_D(3.76101422);  
    g_osc->set_eta_E(3.66572344);  

    g_osc->set_P_tau(0.05574846 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 4.65346270);
    g_osc->set_k_HFLrun2( 7.49289295);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.03241136 );
	stim_wang->set_S0_ta_st( 0.03638972 );
	stim_wang->set_S0_gas_st( 0.03151137 );
	stim_wang->set_S0_vas_st( 0.41804559 );
	stim_wang->set_S0_ham_st( 0.12259760 );
	stim_wang->set_S0_rf_st( 0.12835663 );
	stim_wang->set_S0_glu_st( 0.16455500 );
	stim_wang->set_S0_hfl_st( 0.09959168 );
	stim_wang->set_S0_sol_sw( 0.01866390 );
	stim_wang->set_S0_ta_sw( 0.01660701 );
	stim_wang->set_S0_gas_sw( 0.11540703 );
	stim_wang->set_S0_vas_sw( 0.06574179 );
	stim_wang->set_S0_ham_sw( 0.08134159 );
	stim_wang->set_S0_rf_sw( 0.44116974 );
	stim_wang->set_S0_glu_sw( 0.02054306 );
	stim_wang->set_S0_hfl_sw( 0.04508776 );
	stim_wang->set_G_sol( 3.75047085 );
	stim_wang->set_G_sol_ta( 5.89192044 );
	stim_wang->set_G_gas( 12.83819992 );
	stim_wang->set_G_vas( 2.53868026 );
	stim_wang->set_G_ham( 1.03490682 );
	stim_wang->set_G_glu( 0.16835715 );
	stim_wang->set_G_ta_sw( 3.79253914 );
	stim_wang->set_G_ta_st( 3.37713368 );
	stim_wang->set_G_hfl( 2.76030406 );
	stim_wang->set_G_ham_hfl( 1.79259197 );
	stim_wang->set_l_off_ta_sw( 0.51891587 );
	stim_wang->set_l_off_ta_st( 0.71439387 );
	stim_wang->set_l_off_ham_hfl( 0.69919258 );
	stim_wang->set_l_off_hfl( 0.58522616 );
	stim_wang->set_K_ham( 8.96509536 );
	stim_wang->set_K_glu( 8.05188176 );
	stim_wang->set_K_hfl( 17.93786486 );
	stim_wang->set_D_ham( 0.39163915 );
	stim_wang->set_D_glu( 0.30197359 );
	stim_wang->set_D_hfl( 0.36079207 );
	stim_wang->set_theta_ref( 0.04617243 );
	stim_wang->set_si_vas( 0.59026919 );
	stim_wang->set_si_rf( 0.80725243 );
	stim_wang->set_si_glu( 0.59850558 );
	stim_wang->set_si_hfl( 0.94802856 );
	stim_wang->set_K_sp_vas( 0.86888715 );
	stim_wang->set_K_sp_glu( 3.44732507 );
	stim_wang->set_K_sp_hfl( 4.08095783 );
	stim_wang->set_D_sp_vas( 0.08172700 );
	stim_wang->set_D_sp_glu( 0.08243571 );
	stim_wang->set_D_sp_hfl( 0.09650892 );
	stim_wang->set_theta_k_ref( 0.14225990 );
	stim_wang->set_theta_h_ref0( 0.38352963 );
	stim_wang->set_d_sp( -0.01185803 );
	stim_wang->set_d_si( 0.50606586 );
	stim_wang->set_k_THETA( 3.49356149 );
	stim_wang->set_k_theta( 11.92184307 );
	stim_wang->set_phi_off_pk( 0.12875805 );

	//init pos
	joints_init->set_T3( 0.51710229 );
	joints_init->set_T3_p( 0.38067749 );
	joints_init->set_R2( 0.23218523 );
	joints_init->set_R2_p( 1.68843214 );
	joints_init->set_r_sh_p( 1.85401762 );
	joints_init->set_r_hip( -0.57306585 );
	joints_init->set_r_hip_p( -2.43889098 );
	joints_init->set_r_knee( 0.29285858 );
	joints_init->set_r_knee_p( -4.05543046 );
	joints_init->set_r_ankle( 0.00865157 );
	joints_init->set_r_ankle_p( 3.12922124 );
	joints_init->set_l_hip( 0.34471621 );
	joints_init->set_l_hip_p( -2.94408932 );
	joints_init->set_l_knee( 0.65227520 );
	joints_init->set_l_knee_p( 2.21837676 );
	joints_init->set_l_ankle( 0.38534277 );
	joints_init->set_l_ankle_p( 2.85945321 );
}
