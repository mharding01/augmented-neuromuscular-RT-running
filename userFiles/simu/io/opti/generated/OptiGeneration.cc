#include "OptiGeneration.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"

/*! \brief constructor
 */
OptiGeneration::OptiGeneration()
{

}

/*! \brief destructor
 */
OptiGeneration::~OptiGeneration()
{

}

/*! \brief set optimization parameters
 */
void OptiGeneration::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);

	// Wang stimulations
	stim_wang->set_S0_sol_st( optiParams[0] );
	stim_wang->set_S0_ta_st( optiParams[1] );
	stim_wang->set_S0_gas_st( optiParams[2] );
	stim_wang->set_S0_vas_st( optiParams[3] );
	stim_wang->set_S0_ham_st( optiParams[4] );
	stim_wang->set_S0_rf_st( optiParams[5] );
	stim_wang->set_S0_glu_st( optiParams[6] );
	stim_wang->set_S0_hfl_st( optiParams[7] );
	stim_wang->set_S0_sol_sw( optiParams[8] );
	stim_wang->set_S0_ta_sw( optiParams[9] );
	stim_wang->set_S0_gas_sw( optiParams[10] );
	stim_wang->set_S0_vas_sw( optiParams[11] );
	stim_wang->set_S0_ham_sw( optiParams[12] );
	stim_wang->set_S0_rf_sw( optiParams[13] );
	stim_wang->set_S0_glu_sw( optiParams[14] );
	stim_wang->set_S0_hfl_sw( optiParams[15] );
	stim_wang->set_G_sol( optiParams[16] );
	stim_wang->set_G_sol_ta( optiParams[17] );
	stim_wang->set_G_gas( optiParams[18] );
	stim_wang->set_G_vas( optiParams[19] );
	stim_wang->set_G_ham( optiParams[20] );
	stim_wang->set_G_glu( optiParams[21] );
	stim_wang->set_G_ta_sw( optiParams[22] );
	stim_wang->set_G_ta_st( optiParams[23] );
	stim_wang->set_G_hfl( optiParams[24] );
	stim_wang->set_G_ham_hfl( optiParams[25] );
	stim_wang->set_l_off_ta_sw( optiParams[26] );
	stim_wang->set_l_off_ta_st( optiParams[27] );
	stim_wang->set_l_off_ham_hfl( optiParams[28] );
	stim_wang->set_l_off_hfl( optiParams[29] );
	stim_wang->set_K_ham( optiParams[30] );
	stim_wang->set_K_glu( optiParams[31] );
	stim_wang->set_K_hfl( optiParams[32] );
	stim_wang->set_D_ham( optiParams[33] );
	stim_wang->set_D_glu( optiParams[34] );
	stim_wang->set_D_hfl( optiParams[35] );
	stim_wang->set_theta_ref( optiParams[36] );
	stim_wang->set_si_vas( optiParams[37] );
	stim_wang->set_si_rf( optiParams[38] );
	stim_wang->set_si_glu( optiParams[39] );
	stim_wang->set_si_hfl( optiParams[40] );
	stim_wang->set_K_sp_vas( optiParams[41] );
	stim_wang->set_K_sp_glu( optiParams[42] );
	stim_wang->set_K_sp_hfl( optiParams[43] );
	stim_wang->set_D_sp_vas( optiParams[44] );
	stim_wang->set_D_sp_glu( optiParams[45] );
	stim_wang->set_D_sp_hfl( optiParams[46] );
	stim_wang->set_theta_k_ref( optiParams[47] );
	stim_wang->set_theta_h_ref0( optiParams[48] );
	stim_wang->set_d_sp( optiParams[49] );
	stim_wang->set_d_si( optiParams[50] );
	stim_wang->set_k_THETA( optiParams[51] );
	stim_wang->set_k_theta( optiParams[52] );
	stim_wang->set_phi_off_pk( optiParams[53] );

	//init pos
	joints_init->set_T3( optiParams[54] );
	joints_init->set_T3_p( optiParams[55] );
	joints_init->set_R2( optiParams[56] );
	joints_init->set_R2_p( optiParams[57] );
	joints_init->set_r_sh_p( optiParams[58] );
	joints_init->set_r_hip( optiParams[59] );
	joints_init->set_r_hip_p( optiParams[60] );
	joints_init->set_r_knee( optiParams[61] );
	joints_init->set_r_knee_p( optiParams[62] );
	joints_init->set_r_ankle( optiParams[63] );
	joints_init->set_r_ankle_p( optiParams[64] );
	joints_init->set_l_hip( optiParams[65] );
	joints_init->set_l_hip_p( optiParams[66] );
	joints_init->set_l_knee( optiParams[67] );
	joints_init->set_l_knee_p( optiParams[68] );
	joints_init->set_l_ankle( optiParams[69] );
	joints_init->set_l_ankle_p( optiParams[70] );
}
