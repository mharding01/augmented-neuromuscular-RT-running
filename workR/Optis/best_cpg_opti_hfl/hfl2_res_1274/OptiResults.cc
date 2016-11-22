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
    g_osc->set_gamma_A(1.56864066);
    g_osc->set_gamma_B(2.46232952);
    g_osc->set_gamma_C(2.94139352);

    g_osc->set_eta_A(5.45789353);  
    g_osc->set_eta_B(4.84511153);  
    g_osc->set_eta_C(5.65398608);  
    g_osc->set_eta_D(3.75172105);  
    g_osc->set_eta_E(3.68029441);  

    g_osc->set_P_tau(0.05430483 );    // TODO: not using v_diff

    g_osc->set_k_HFLrun1( 5.68569836);
    g_osc->set_k_HFLrun2( 5.42569114);
    g_osc->set_k_HFLrun3( 3.26833659);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.01827984 );
	stim_wang->set_S0_ta_st( 0.03668745 );
	stim_wang->set_S0_gas_st( 0.01146552 );
	stim_wang->set_S0_vas_st( 0.45216266 );
	stim_wang->set_S0_ham_st( 0.23784600 );
	stim_wang->set_S0_rf_st( 0.29314408 );
	stim_wang->set_S0_glu_st( 0.29819257 );
	stim_wang->set_S0_hfl_st( 0.15375022 );
	stim_wang->set_S0_sol_sw( 0.01968270 );
	stim_wang->set_S0_ta_sw( 0.03157519 );
	stim_wang->set_S0_gas_sw( 0.06600710 );
	stim_wang->set_S0_vas_sw( 0.03833717 );
	stim_wang->set_S0_ham_sw( 0.04147778 );
	stim_wang->set_S0_rf_sw( 0.19443608 );
	stim_wang->set_S0_glu_sw( 0.01054136 );
	stim_wang->set_S0_hfl_sw( 0.03824912 );
	stim_wang->set_G_sol( 2.30410489 );
	stim_wang->set_G_sol_ta( 2.32860675 );
	stim_wang->set_G_gas( 18.66488700 );
	stim_wang->set_G_vas( 3.79730211 );
	stim_wang->set_G_ham( 0.99342985 );
	stim_wang->set_G_glu( 0.17548112 );
	stim_wang->set_G_ta_sw( 4.45737422 );
	stim_wang->set_G_ta_st( 4.10335393 );
	stim_wang->set_G_hfl( 2.57142311 );
	stim_wang->set_G_ham_hfl( 0.71997733 );
	stim_wang->set_l_off_ta_sw( 0.52731023 );
	stim_wang->set_l_off_ta_st( 0.75783631 );
	stim_wang->set_l_off_ham_hfl( 0.52355669 );
	stim_wang->set_l_off_hfl( 0.24739006 );
	stim_wang->set_K_ham( 2.61967549 );
	stim_wang->set_K_glu( 6.96591444 );
	stim_wang->set_K_hfl( 13.04894780 );
	stim_wang->set_D_ham( 0.19244114 );
	stim_wang->set_D_glu( 0.04607496 );
	stim_wang->set_D_hfl( 0.33205876 );
	stim_wang->set_theta_ref( 0.10062236 );
	stim_wang->set_si_vas( 0.66041250 );
	stim_wang->set_si_rf( 0.00943271 );
	stim_wang->set_si_glu( 0.85406304 );
	stim_wang->set_si_hfl( 0.25970543 );
	stim_wang->set_K_sp_vas( 4.31054452 );
	stim_wang->set_K_sp_glu( 4.80358381 );
	stim_wang->set_K_sp_hfl( 1.00243545 );
	stim_wang->set_D_sp_vas( 0.04680553 );
	stim_wang->set_D_sp_glu( 0.01937024 );
	stim_wang->set_D_sp_hfl( 0.00405168 );
	stim_wang->set_theta_k_ref( 0.27040360 );
	stim_wang->set_theta_h_ref0( 0.56020467 );
	stim_wang->set_d_sp( 0.04550590 );
	stim_wang->set_d_si( 0.58301073 );
	stim_wang->set_k_THETA( 2.35756787 );
	stim_wang->set_k_theta( 13.66384240 );
	stim_wang->set_phi_off_pk( 0.12082398 );

	//init pos
	joints_init->set_T3( 0.55221861 );
	joints_init->set_T3_p( -0.45474661 );
	joints_init->set_R2( 0.02838587 );
	joints_init->set_R2_p( -0.35564961 );
	joints_init->set_r_sh_p( 2.02695881 );
	joints_init->set_r_hip( -0.48969151 );
	joints_init->set_r_hip_p( -0.02756574 );
	joints_init->set_r_knee( 0.64475093 );
	joints_init->set_r_knee_p( 0.79057812 );
	joints_init->set_r_ankle( 0.00329890 );
	joints_init->set_r_ankle_p( -0.25018301 );
	joints_init->set_l_hip( 0.14834840 );
	joints_init->set_l_hip_p( 3.76061867 );
	joints_init->set_l_knee( 0.86399494 );
	joints_init->set_l_knee_p( 3.79062051 );
	joints_init->set_l_ankle( 0.28443006 );
	joints_init->set_l_ankle_p( -2.81942246 );
}
