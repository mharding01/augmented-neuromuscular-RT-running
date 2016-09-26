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
	stim_wang->set_S0_sol_st( 0.01340756 );
	stim_wang->set_S0_ta_st( 0.01365652 );
	stim_wang->set_S0_gas_st( 0.01292869 );
	stim_wang->set_S0_vas_st( 0.06363898 );
	stim_wang->set_S0_ham_st( 0.01333396 );
	stim_wang->set_S0_rf_st( 0.01037046 );
	stim_wang->set_S0_glu_st( 0.02637039 );
	stim_wang->set_S0_hfl_st( 0.01009040 );
	stim_wang->set_S0_sol_sw( 0.02450600 );
	stim_wang->set_S0_ta_sw( 0.01007776 );
	stim_wang->set_S0_gas_sw( 0.01068173 );
	stim_wang->set_S0_vas_sw( 0.01000799 );
	stim_wang->set_S0_ham_sw( 0.02238508 );
	stim_wang->set_S0_rf_sw( 0.04960209 );
	stim_wang->set_S0_glu_sw( 0.01045520 );
	stim_wang->set_S0_hfl_sw( 0.01163426 );
	stim_wang->set_G_sol( 1.61224095 );
	stim_wang->set_G_sol_ta( 1.90853320 );
	stim_wang->set_G_gas( 0.16533600 );
	stim_wang->set_G_vas( 0.94725007 );
	stim_wang->set_G_ham( 0.01793268 );
	stim_wang->set_G_glu( 0.18862380 );
	stim_wang->set_G_ta_sw( 0.56135831 );
	stim_wang->set_G_ta_st( 1.19434665 );
	stim_wang->set_G_hfl( 0.22201172 );
	stim_wang->set_G_ham_hfl( 2.48127000 );
	stim_wang->set_l_off_ta_sw( 0.66948590 );
	stim_wang->set_l_off_ta_st( 0.79091570 );
	stim_wang->set_l_off_ham_hfl( 0.64224612 );
	stim_wang->set_l_off_hfl( 0.83652495 );
	stim_wang->set_K_ham( 2.04731900 );
	stim_wang->set_K_glu( 6.74250800 );
	stim_wang->set_K_hfl( 1.42629030 );
	stim_wang->set_D_ham( 0.09859350 );
	stim_wang->set_D_glu( 0.23453050 );
	stim_wang->set_D_hfl( 0.04545265 );
	stim_wang->set_theta_ref( 0.03701806 );
	stim_wang->set_si_vas( 0.97733890 );
	stim_wang->set_si_rf( 0.41614800 );
	stim_wang->set_si_glu( 0.99121840 );
	stim_wang->set_si_hfl( 0.41041600 );
	stim_wang->set_K_sp_vas( 1.62407000 );
	stim_wang->set_K_sp_glu( 0.23123150 );
	stim_wang->set_K_sp_hfl( 3.34021500 );
	stim_wang->set_D_sp_vas( 0.28034550 );
	stim_wang->set_D_sp_glu( 0.12253250 );
	stim_wang->set_D_sp_hfl( 0.00605375 );
	stim_wang->set_theta_k_ref( 0.08881170 );
	stim_wang->set_theta_h_ref0( 0.32930750 );
	stim_wang->set_d_sp( -0.05624880 );
	stim_wang->set_d_si( 0.47355600 );
	stim_wang->set_k_THETA( 1.00391323 );
	stim_wang->set_k_theta( 3.90211600 );
	stim_wang->set_phi_off_pk( 0.17516600 );

	//init pos
	joints_init->set_T3( 0.54001440 );
	joints_init->set_T3_p( 0.14063700 );
	joints_init->set_R2( 0.09754230 );
	joints_init->set_R2_p( -1.47786000 );
	joints_init->set_r_sh_p( -1.47128000 );
	joints_init->set_r_hip( -0.55958340 );
	joints_init->set_r_hip_p( 2.72561000 );
	joints_init->set_r_knee( 0.00032810 );
	joints_init->set_r_knee_p( 0.21770000 );
	joints_init->set_r_ankle( -0.01276290 );
	joints_init->set_r_ankle_p( -0.40108000 );
	joints_init->set_l_hip( 0.14305524 );
	joints_init->set_l_hip_p( -1.35761000 );
	joints_init->set_l_knee( 0.35015730 );
	joints_init->set_l_knee_p( -2.87972000 );
	joints_init->set_l_ankle( 0.09355191 );
	joints_init->set_l_ankle_p( 0.32305000 );
}
