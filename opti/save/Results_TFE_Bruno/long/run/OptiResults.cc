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
	stim_wang->set_S0_sol_st( 0.01528884 );
	stim_wang->set_S0_ta_st( 0.03165072 );
	stim_wang->set_S0_gas_st( 0.03219700 );
	stim_wang->set_S0_vas_st( 0.32380979 );
	stim_wang->set_S0_ham_st( 0.05033379 );
	stim_wang->set_S0_rf_st( 0.02505662 );
	stim_wang->set_S0_glu_st( 0.31918780 );
	stim_wang->set_S0_hfl_st( 0.01000068 );
	stim_wang->set_S0_sol_sw( 0.01015243 );
	stim_wang->set_S0_ta_sw( 0.01130838 );
	stim_wang->set_S0_gas_sw( 0.01034117 );
	stim_wang->set_S0_vas_sw( 0.01000005 );
	stim_wang->set_S0_ham_sw( 0.07130125 );
	stim_wang->set_S0_rf_sw( 0.13031558 );
	stim_wang->set_S0_glu_sw( 0.03396396 );
	stim_wang->set_S0_hfl_sw( 0.04740248 );
	stim_wang->set_G_sol( 4.97912863 );
	stim_wang->set_G_sol_ta( 4.93623480 );
	stim_wang->set_G_gas( 4.60562000 );
	stim_wang->set_G_vas( 0.85177134 );
	stim_wang->set_G_ham( 0.99674700 );
	stim_wang->set_G_glu( 0.00004399 );
	stim_wang->set_G_ta_sw( 3.83146560 );
	stim_wang->set_G_ta_st( 2.75783190 );
	stim_wang->set_G_hfl( 2.99997453 );
	stim_wang->set_G_ham_hfl( 8.28798000 );
	stim_wang->set_l_off_ta_sw( 0.64548770 );
	stim_wang->set_l_off_ta_st( 0.55824380 );
	stim_wang->set_l_off_ham_hfl( 0.68353759 );
	stim_wang->set_l_off_hfl( 0.65121730 );
	stim_wang->set_K_ham( 5.12399000 );
	stim_wang->set_K_glu( 6.09703250 );
	stim_wang->set_K_hfl( 7.47720400 );
	stim_wang->set_D_ham( 0.20265600 );
	stim_wang->set_D_glu( 0.49216050 );
	stim_wang->set_D_hfl( 0.48977550 );
	stim_wang->set_theta_ref( 0.03882003 );
	stim_wang->set_si_vas( 0.63736500 );
	stim_wang->set_si_rf( 0.63408400 );
	stim_wang->set_si_glu( 0.98768560 );
	stim_wang->set_si_hfl( 0.96445100 );
	stim_wang->set_K_sp_vas( 1.36866500 );
	stim_wang->set_K_sp_glu( 3.99221000 );
	stim_wang->set_K_sp_hfl( 4.51641000 );
	stim_wang->set_D_sp_vas( 0.48574350 );
	stim_wang->set_D_sp_glu( 0.39298150 );
	stim_wang->set_D_sp_hfl( 0.49966200 );
	stim_wang->set_theta_k_ref( 0.02633541 );
	stim_wang->set_theta_h_ref0( 0.20536068 );
	stim_wang->set_d_sp( -0.00024480 );
	stim_wang->set_d_si( 0.63522100 );
	stim_wang->set_k_THETA( 3.19163350 );
	stim_wang->set_k_theta( 12.29647400 );
	stim_wang->set_phi_off_pk( 0.19913795 );

	//init pos
	joints_init->set_T3( 0.48871788 );
	joints_init->set_T3_p( 0.18881200 );
	joints_init->set_R2( 0.14403030 );
	joints_init->set_R2_p( 1.99415000 );
	joints_init->set_r_sh_p( 1.62949000 );
	joints_init->set_r_hip( -0.83079340 );
	joints_init->set_r_hip_p( 1.21247000 );
	joints_init->set_r_knee( 0.18265302 );
	joints_init->set_r_knee_p( 0.72403000 );
	joints_init->set_r_ankle( -0.07651557 );
	joints_init->set_r_ankle_p( -2.51525000 );
	joints_init->set_l_hip( 0.34607080 );
	joints_init->set_l_hip_p( -4.91734030 );
	joints_init->set_l_knee( 1.58567640 );
	joints_init->set_l_knee_p( 3.45950000 );
	joints_init->set_l_ankle( 0.12390005 );
	joints_init->set_l_ankle_p( -4.94404040 );
}
