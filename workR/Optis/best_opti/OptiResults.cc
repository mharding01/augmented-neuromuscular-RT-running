#include "OptiResults.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"

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

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.01821622 );
	stim_wang->set_S0_ta_st( 0.02934691 );
	stim_wang->set_S0_gas_st( 0.02306751 );
	stim_wang->set_S0_vas_st( 0.37581340 );
	stim_wang->set_S0_ham_st( 0.06437070 );
	stim_wang->set_S0_rf_st( 0.45308626 );
	stim_wang->set_S0_glu_st( 0.27716083 );
	stim_wang->set_S0_hfl_st( 0.18990279 );
	stim_wang->set_S0_sol_sw( 0.02196479 );
	stim_wang->set_S0_ta_sw( 0.03418070 );
	stim_wang->set_S0_gas_sw( 0.11830941 );
	stim_wang->set_S0_vas_sw( 0.07509333 );
	stim_wang->set_S0_ham_sw( 0.01658044 );
	stim_wang->set_S0_rf_sw( 0.16277237 );
	stim_wang->set_S0_glu_sw( 0.01353357 );
	stim_wang->set_S0_hfl_sw( 0.01278032 );
	stim_wang->set_G_sol( 3.01996740 );
	stim_wang->set_G_sol_ta( 1.94436751 );
	stim_wang->set_G_gas( 2.07542609 );
	stim_wang->set_G_vas( 3.05052881 );
	stim_wang->set_G_ham( 1.33484603 );
	stim_wang->set_G_glu( 0.60992137 );
	stim_wang->set_G_ta_sw( 4.78986196 );
	stim_wang->set_G_ta_st( 2.67346753 );
	stim_wang->set_G_hfl( 2.88617230 );
	stim_wang->set_G_ham_hfl( 4.83540672 );
	stim_wang->set_l_off_ta_sw( 0.60048553 );
	stim_wang->set_l_off_ta_st( 0.60113361 );
	stim_wang->set_l_off_ham_hfl( 0.61594341 );
	stim_wang->set_l_off_hfl( 0.07361972 );
	stim_wang->set_K_ham( 9.96806507 );
	stim_wang->set_K_glu( 12.62738680 );
	stim_wang->set_K_hfl( 3.39471198 );
	stim_wang->set_D_ham( 0.31541490 );
	stim_wang->set_D_glu( 0.08526154 );
	stim_wang->set_D_hfl( 0.18676656 );
	stim_wang->set_theta_ref( 0.07024470 );
	stim_wang->set_si_vas( 0.65801971 );
	stim_wang->set_si_rf( 0.69459752 );
	stim_wang->set_si_glu( 0.92662565 );
	stim_wang->set_si_hfl( 0.51336055 );
	stim_wang->set_K_sp_vas( 0.44314468 );
	stim_wang->set_K_sp_glu( 1.70576476 );
	stim_wang->set_K_sp_hfl( 2.92504185 );
	stim_wang->set_D_sp_vas( 0.04160829 );
	stim_wang->set_D_sp_glu( 0.05394762 );
	stim_wang->set_D_sp_hfl( 0.06554153 );
	stim_wang->set_theta_k_ref( 0.27345535 );
	stim_wang->set_theta_h_ref0( 0.54939395 );
	stim_wang->set_d_sp( 0.07064843 );
	stim_wang->set_d_si( 0.49408563 );
	stim_wang->set_k_THETA( 1.85819214 );
	stim_wang->set_k_theta( 6.86674446 );
	stim_wang->set_phi_off_pk( 0.13679013 );

	//init pos
	joints_init->set_T3( 0.48088311 );
	joints_init->set_T3_p( 0.15143504 );
	joints_init->set_R2( 0.13122466 );
	joints_init->set_R2_p( 1.67801148 );
	joints_init->set_r_sh_p( 1.73658144 );
	joints_init->set_r_hip( -0.70179734 );
	joints_init->set_r_hip_p( -2.63926232 );
	joints_init->set_r_knee( 0.15773517 );
	joints_init->set_r_knee_p( -2.63078159 );
	joints_init->set_r_ankle( -0.03679161 );
	joints_init->set_r_ankle_p( 4.73898315 );
	joints_init->set_l_hip( 0.04889869 );
	joints_init->set_l_hip_p( 2.52969697 );
	joints_init->set_l_knee( 1.32866350 );
	joints_init->set_l_knee_p( 3.67791084 );
	joints_init->set_l_ankle( 0.26511839 );
	joints_init->set_l_ankle_p( -3.41326666 );
}
