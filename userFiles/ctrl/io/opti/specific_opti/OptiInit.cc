#include "OptiInit.hh"
#include "StimWangCtrl.hh"
#include "ImpedanceCtrlInit.hh"

/*! \brief constructor
 */
OptiInit::OptiInit()
{

}

/*! \brief destructor
 */
OptiInit::~OptiInit()
{

}

/*! \brief set optimization parameters
 */
void OptiInit::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);

	stim_wang->set_S0_sol_st( 0.03103356 );
	stim_wang->set_S0_ta_st( 0.01549300 );
	stim_wang->set_S0_gas_st( 0.01004285 );
	stim_wang->set_S0_vas_st( 0.69724521 );
	stim_wang->set_S0_ham_st( 0.30795929 );
	stim_wang->set_S0_rf_st( 0.38416204 );
	stim_wang->set_S0_glu_st( 0.18781414 );
	stim_wang->set_S0_hfl_st( 0.01600662 );
	stim_wang->set_S0_sol_sw( 0.02306808 );
	stim_wang->set_S0_ta_sw( 0.03062988 );
	stim_wang->set_S0_gas_sw( 0.01529006 );
	stim_wang->set_S0_vas_sw( 0.09026542 );
	stim_wang->set_S0_ham_sw( 0.09698581 );
	stim_wang->set_S0_rf_sw( 0.07174147 );
	stim_wang->set_S0_glu_sw( 0.04380480 );
	stim_wang->set_S0_hfl_sw( 0.04271192 );
	stim_wang->set_G_sol( 1.91340688 );
	stim_wang->set_G_sol_ta( 0.50606033 );
	stim_wang->set_G_gas( 1.00602853 );
	stim_wang->set_G_vas( 0.83702932 );
	stim_wang->set_G_ham( 0.90432900 );
	stim_wang->set_G_glu( 0.24398800 );
	stim_wang->set_G_ta_sw( 1.55496130 );
	stim_wang->set_G_ta_st( 5.00000000 );
	stim_wang->set_G_hfl( 1.98290083 );
	stim_wang->set_G_ham_hfl( 1.46387000 );
	stim_wang->set_l_off_ta_sw( 0.58753460 );
	stim_wang->set_l_off_ta_st( 0.50002699 );
	stim_wang->set_l_off_ham_hfl( 0.69668821 );
	stim_wang->set_l_off_hfl( 0.30266000 );
	stim_wang->set_K_ham( 9.07951300 );
	stim_wang->set_K_glu( 10.89342050 );
	stim_wang->set_K_hfl( 17.30918800 );
	stim_wang->set_D_ham( 0.16279000 );
	stim_wang->set_D_glu( 0.38228250 );
	stim_wang->set_D_hfl( 0.25967300 );
	stim_wang->set_theta_ref( 0.03818707 );
	stim_wang->set_si_vas( 0.99762560 );
	stim_wang->set_si_rf( 0.99872300 );
	stim_wang->set_si_glu( 0.95953920 );
	stim_wang->set_si_hfl( 0.95517700 );
	stim_wang->set_K_sp_vas( 1.60392500 );
	stim_wang->set_K_sp_glu( 0.01876810 );
	stim_wang->set_K_sp_hfl( 0.00498767 );
	stim_wang->set_D_sp_vas( 0.30494500 );
	stim_wang->set_D_sp_glu( 0.04883995 );
	stim_wang->set_D_sp_hfl( 0.46693050 );
	stim_wang->set_theta_k_ref( 0.29146260 );
	stim_wang->set_theta_h_ref0( 0.41031640 );
	stim_wang->set_c_d( 0.00000000 );
	stim_wang->set_c_v( 0.00000000 );
	stim_wang->set_d_sp( -0.11457980 );
	stim_wang->set_d_si( 0.62835450 );
	stim_wang->set_k_THETA( 3.04232390 );
	stim_wang->set_k_theta( 13.40236200 );
	stim_wang->set_phi_off_pk( 0.05650066 );
}
