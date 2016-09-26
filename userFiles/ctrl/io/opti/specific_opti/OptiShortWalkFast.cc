#include "OptiShortWalkFast.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"

/*! \brief constructor
 */
OptiShortWalkFast::OptiShortWalkFast()
{

}

/*! \brief destructor
 */
OptiShortWalkFast::~OptiShortWalkFast()
{

}

/*! \brief set optimization parameters
 */
void OptiShortWalkFast::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);

	// Wang stimulations
	stim_wang->set_S0_sol_st( 0.03904388 );
	stim_wang->set_S0_ta_st( 0.04779608 );
	stim_wang->set_S0_gas_st( 0.04636976 );
	stim_wang->set_S0_vas_st( 0.55425600 );
	stim_wang->set_S0_ham_st( 0.29808269 );
	stim_wang->set_S0_rf_st( 0.01119879 );
	stim_wang->set_S0_glu_st( 0.06400913 );
	stim_wang->set_S0_hfl_st( 0.30182966 );
	stim_wang->set_S0_sol_sw( 0.01731868 );
	stim_wang->set_S0_ta_sw( 0.01250043 );
	stim_wang->set_S0_gas_sw( 0.18507303 );
	stim_wang->set_S0_vas_sw( 0.07047667 );
	stim_wang->set_S0_ham_sw( 0.03915019 );
	stim_wang->set_S0_rf_sw( 0.49610401 );
	stim_wang->set_S0_glu_sw( 0.01711260 );
	stim_wang->set_S0_hfl_sw( 0.04360116 );
	stim_wang->set_G_sol( 2.47753837 );
	stim_wang->set_G_sol_ta( 4.78651180 );
	stim_wang->set_G_gas( 14.84888000 );
	stim_wang->set_G_vas( 0.87592924 );
	stim_wang->set_G_ham( 0.01396110 );
	stim_wang->set_G_glu( 0.03645840 );
	stim_wang->set_G_ta_sw( 4.76705140 );
	stim_wang->set_G_ta_st( 4.34714940 );
	stim_wang->set_G_hfl( 1.60283749 );
	stim_wang->set_G_ham_hfl( 0.02923650 );
	stim_wang->set_l_off_ta_sw( 0.51767591 );
	stim_wang->set_l_off_ta_st( 0.66503030 );
	stim_wang->set_l_off_ham_hfl( 0.70978558 );
	stim_wang->set_l_off_hfl( 0.39617310 );
	stim_wang->set_K_ham( 1.01206492 );
	stim_wang->set_K_glu( 3.33351500 );
	stim_wang->set_K_hfl( 10.58385200 );
	stim_wang->set_D_ham( 0.03265760 );
	stim_wang->set_D_glu( 0.35423850 );
	stim_wang->set_D_hfl( 0.38981100 );
	stim_wang->set_theta_ref( 0.10990012 );
	stim_wang->set_si_vas( 0.44432810 );
	stim_wang->set_si_rf( 0.12553500 );
	stim_wang->set_si_glu( 0.67032400 );
	stim_wang->set_si_hfl( 0.95249600 );
	stim_wang->set_K_sp_vas( 0.02032690 );
	stim_wang->set_K_sp_glu( 3.62705000 );
	stim_wang->set_K_sp_hfl( 0.54166500 );
	stim_wang->set_D_sp_vas( 0.06673590 );
	stim_wang->set_D_sp_glu( 0.00536783 );
	stim_wang->set_D_sp_hfl( 0.04253720 );
	stim_wang->set_theta_k_ref( 0.25335960 );
	stim_wang->set_theta_h_ref0( 0.34282650 );
	stim_wang->set_d_sp( 0.09258160 );
	stim_wang->set_d_si( 0.68558400 );
	stim_wang->set_k_THETA( 2.95798710 );
	stim_wang->set_k_theta( 1.85931160 );
	stim_wang->set_phi_off_pk( 0.19295885 );

	//init pos
	joints_init->set_T3( 0.47407408 );
	joints_init->set_T3_p( 0.48867300 );
	joints_init->set_R2( 0.16069080 );
	joints_init->set_R2_p( -0.12308000 );
	joints_init->set_r_sh_p( -3.98346000 );
	joints_init->set_r_hip( -0.68971184 );
	joints_init->set_r_hip_p( -1.77102000 );
	joints_init->set_r_knee( 0.46706820 );
	joints_init->set_r_knee_p( -1.68480000 );
	joints_init->set_r_ankle( -0.00603162 );
	joints_init->set_r_ankle_p( 2.09645000 );
	joints_init->set_l_hip( 0.30291780 );
	joints_init->set_l_hip_p( -3.79971000 );
	joints_init->set_l_knee( 0.52716440 );
	joints_init->set_l_knee_p( 1.36427000 );
	joints_init->set_l_ankle( 0.44243712 );
	joints_init->set_l_ankle_p( 1.76805000 );
}
