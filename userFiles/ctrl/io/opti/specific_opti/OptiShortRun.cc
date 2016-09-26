#include "OptiShortRun.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"

/*! \brief constructor
 */
OptiShortRun::OptiShortRun()
{

}

/*! \brief destructor
 */
OptiShortRun::~OptiShortRun()
{

}

/*! \brief set optimization parameters
 */
void OptiShortRun::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.01505988 );
	stim_wang->set_S0_ta_st( 0.03339436 );
	stim_wang->set_S0_gas_st( 0.04163660 );
	stim_wang->set_S0_vas_st( 0.52803501 );
	stim_wang->set_S0_ham_st( 0.01067587 );
	stim_wang->set_S0_rf_st( 0.01027389 );
	stim_wang->set_S0_glu_st( 0.29916087 );
	stim_wang->set_S0_hfl_st( 0.01229380 );
	stim_wang->set_S0_sol_sw( 0.01119018 );
	stim_wang->set_S0_ta_sw( 0.03181172 );
	stim_wang->set_S0_gas_sw( 0.01048316 );
	stim_wang->set_S0_vas_sw( 0.01828385 );
	stim_wang->set_S0_ham_sw( 0.07783750 );
	stim_wang->set_S0_rf_sw( 0.03097185 );
	stim_wang->set_S0_glu_sw( 0.02286668 );
	stim_wang->set_S0_hfl_sw( 0.01191430 );
	stim_wang->set_G_sol( 3.80842169 );
	stim_wang->set_G_sol_ta( 1.01989600 );
	stim_wang->set_G_gas( 3.70360000 );
	stim_wang->set_G_vas( 3.10458318 );
	stim_wang->set_G_ham( 1.08090300 );
	stim_wang->set_G_glu( 0.08200720 );
	stim_wang->set_G_ta_sw( 2.52710830 );
	stim_wang->set_G_ta_st( 3.39094675 );
	stim_wang->set_G_hfl( 2.46012656 );
	stim_wang->set_G_ham_hfl( 1.01901000 );
	stim_wang->set_l_off_ta_sw( 0.74453840 );
	stim_wang->set_l_off_ta_st( 0.78011270 );
	stim_wang->set_l_off_ham_hfl( 0.64268987 );
	stim_wang->set_l_off_hfl( 0.12636695 );
	stim_wang->set_K_ham( 1.18302570 );
	stim_wang->set_K_glu( 11.79063050 );
	stim_wang->set_K_hfl( 9.27456300 );
	stim_wang->set_D_ham( 0.00504475 );
	stim_wang->set_D_glu( 0.02871265 );
	stim_wang->set_D_hfl( 0.24215250 );
	stim_wang->set_theta_ref( 0.01750215 );
	stim_wang->set_si_vas( 0.81033850 );
	stim_wang->set_si_rf( 0.00069802 );
	stim_wang->set_si_glu( 0.99838640 );
	stim_wang->set_si_hfl( 0.39981800 );
	stim_wang->set_K_sp_vas( 1.67942500 );
	stim_wang->set_K_sp_glu( 2.95712000 );
	stim_wang->set_K_sp_hfl( 4.42688000 );
	stim_wang->set_D_sp_vas( 0.45190950 );
	stim_wang->set_D_sp_glu( 0.16748050 );
	stim_wang->set_D_sp_hfl( 0.00321490 );
	stim_wang->set_theta_k_ref( 0.15579450 );
	stim_wang->set_theta_h_ref0( 0.33077610 );
	stim_wang->set_d_sp( -0.00157140 );
	stim_wang->set_d_si( 0.28735850 );
	stim_wang->set_k_THETA( 3.75169490 );
	stim_wang->set_k_theta( 1.35157920 );
	stim_wang->set_phi_off_pk( 0.06189914 );

	//init pos
	joints_init->set_T3( 0.48155709 );
	joints_init->set_T3_p( -0.04242300 );
	joints_init->set_R2( 0.07895220 );
	joints_init->set_R2_p( 3.98429000 );
	joints_init->set_r_sh_p( 3.06285000 );
	joints_init->set_r_hip( -0.77381820 );
	joints_init->set_r_hip_p( -4.27134000 );
	joints_init->set_r_knee( 0.15443370 );
	joints_init->set_r_knee_p( -3.01509000 );
	joints_init->set_r_ankle( -0.08428844 );
	joints_init->set_r_ankle_p( -4.99558258 );
	joints_init->set_l_hip( 0.24766040 );
	joints_init->set_l_hip_p( -2.53122000 );
	joints_init->set_l_knee( 1.70106020 );
	joints_init->set_l_knee_p( 1.67706000 );
	joints_init->set_l_ankle( 0.09051899 );
	joints_init->set_l_ankle_p( 1.89495000 );
}
