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
	stim_wang->set_S0_sol_st( 0.02539681 );
	stim_wang->set_S0_ta_st( 0.04180907 );
	stim_wang->set_S0_gas_st( 0.04889095 );
	stim_wang->set_S0_vas_st( 0.50336741 );
	stim_wang->set_S0_ham_st( 0.23079810 );
	stim_wang->set_S0_rf_st( 0.02909083 );
	stim_wang->set_S0_glu_st( 0.17102555 );
	stim_wang->set_S0_hfl_st( 0.21818242 );
	stim_wang->set_S0_sol_sw( 0.02317278 );
	stim_wang->set_S0_ta_sw( 0.03810165 );
	stim_wang->set_S0_gas_sw( 0.07028861 );
	stim_wang->set_S0_vas_sw( 0.04045333 );
	stim_wang->set_S0_ham_sw( 0.05727623 );
	stim_wang->set_S0_rf_sw( 0.42913579 );
	stim_wang->set_S0_glu_sw( 0.02682093 );
	stim_wang->set_S0_hfl_sw( 0.02992623 );
	stim_wang->set_G_sol( 3.36153686 );
	stim_wang->set_G_sol_ta( 4.05320849 );
	stim_wang->set_G_gas( 10.52174112 );
	stim_wang->set_G_vas( 2.44574316 );
	stim_wang->set_G_ham( 0.85910501 );
	stim_wang->set_G_glu( 0.53421296 );
	stim_wang->set_G_ta_sw( 1.73749907 );
	stim_wang->set_G_ta_st( 3.75146240 );
	stim_wang->set_G_hfl( 0.97420560 );
	stim_wang->set_G_ham_hfl( 5.79816674 );
	stim_wang->set_l_off_ta_sw( 0.59935155 );
	stim_wang->set_l_off_ta_st( 0.65295382 );
	stim_wang->set_l_off_ham_hfl( 0.54809544 );
	stim_wang->set_l_off_hfl( 0.33321662 );
	stim_wang->set_K_ham( 7.83368163 );
	stim_wang->set_K_glu( 0.86311441 );
	stim_wang->set_K_hfl( 9.30504352 );
	stim_wang->set_D_ham( 0.33109583 );
	stim_wang->set_D_glu( 0.45908604 );
	stim_wang->set_D_hfl( 0.22135304 );
	stim_wang->set_theta_ref( 0.02739944 );
	stim_wang->set_si_vas( 0.76203972 );
	stim_wang->set_si_rf( 0.35318328 );
	stim_wang->set_si_glu( 0.72271405 );
	stim_wang->set_si_hfl( 0.62309361 );
	stim_wang->set_K_sp_vas( 0.48495882 );
	stim_wang->set_K_sp_glu( 1.43820442 );
	stim_wang->set_K_sp_hfl( 1.88576777 );
	stim_wang->set_D_sp_vas( 0.00895858 );
	stim_wang->set_D_sp_glu( 0.00911448 );
	stim_wang->set_D_sp_hfl( 0.01747260 );
	stim_wang->set_theta_k_ref( 0.17024008 );
	stim_wang->set_theta_h_ref0( 0.39581987 );
	stim_wang->set_d_sp( 0.05264151 );
	stim_wang->set_d_si( 0.53833047 );
	stim_wang->set_k_THETA( 5.21631110 );
	stim_wang->set_k_theta( 10.74509763 );
	stim_wang->set_phi_off_pk( 0.06209830 );

	//init pos
	joints_init->set_T3( 0.46553102 );
	joints_init->set_T3_p( 0.47629866 );
	joints_init->set_R2( 0.06852014 );
	joints_init->set_R2_p( 0.42889930 );
	joints_init->set_r_sh_p( 1.77534118 );
	joints_init->set_r_hip( -0.75737884 );
	joints_init->set_r_hip_p( 1.96939369 );
	joints_init->set_r_knee( 0.34689495 );
	joints_init->set_r_knee_p( 3.11683324 );
	joints_init->set_r_ankle( -0.01164247 );
	joints_init->set_r_ankle_p( 0.72412737 );
	joints_init->set_l_hip( 0.12097076 );
	joints_init->set_l_hip_p( -0.71540625 );
	joints_init->set_l_knee( 0.84032490 );
	joints_init->set_l_knee_p( -2.77564306 );
	joints_init->set_l_ankle( 0.36850263 );
	joints_init->set_l_ankle_p( 1.76051536 );
}
