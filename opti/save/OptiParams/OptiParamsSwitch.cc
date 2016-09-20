#include "OPTI_NAME.hh"
#include "StimGeyerCtrl.hh"
#include "StimWangCtrl.hh"
#include "StimWalkCtrl.hh"
#include "ImpedanceCtrlInit.hh"

/*! \brief constructor
 */
OPTI_NAME::OPTI_NAME()
{

}

/*! \brief destructor
 */
OPTI_NAME::~OPTI_NAME()
{

}

/*! \brief set optimization parameters
 */
void OPTI_NAME::set_opti()
{
	StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);

	// Wang stimulations
	stim_wang->set_S0_sol_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_ta_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_gas_st( { 0.01 ; 0.05 } );
	stim_wang->set_S0_vas_st( { 0.047 ; 0.71 } );
	stim_wang->set_S0_ham_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_rf_st( { 0.01 ; 0.5 } );
	stim_wang->set_S0_glu_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_hfl_st( { 0.01 ; 0.32 } );
	stim_wang->set_S0_sol_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_ta_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_gas_sw( { 0.01 ; 0.2 } );
	stim_wang->set_S0_vas_sw( { 0.01 ; 0.1 } );
	stim_wang->set_S0_ham_sw( { 0.01 ; 0.1 } );
	stim_wang->set_S0_rf_sw( { 0.01 ; 0.5 } );
	stim_wang->set_S0_glu_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_hfl_sw( { 0.01 ; 0.05 } );
	stim_wang->set_G_sol( { 0.97 ; 5.0 } );
	stim_wang->set_G_sol_ta( { 0.4 ; 5.0 } );
	stim_wang->set_G_gas( { 0.0 ; 15.0 } );
	stim_wang->set_G_vas( { 0.82 ; 5.0 } );
	stim_wang->set_G_ham( { 0.0 ; 5.0 } );
	stim_wang->set_G_glu( { 0.0 ; 5.0 } );
	stim_wang->set_G_ta_sw( { 0.55 ; 5.0 } );
	stim_wang->set_G_ta_st( { 0.55 ; 5.0 } );
	stim_wang->set_G_hfl( { 0.17 ; 3.0 } );
	stim_wang->set_G_ham_hfl( { 0.0 ; 10.0 } );
	stim_wang->set_l_off_ta_sw( { 0.50 ; 0.8 } );
	stim_wang->set_l_off_ta_st( { 0.50 ; 0.8 } );
	stim_wang->set_l_off_ham_hfl( { 0.0 ; 0.71 } );
	stim_wang->set_l_off_hfl( { 0.0 ; 0.85 } );
	//stim_wang->set_K_ham( { 1.0 ; 14.0 } );
	//stim_wang->set_K_glu( { 0.5 ; 14.0 } );
	//stim_wang->set_K_hfl( { 1.0 ; 18.0 } );
	//stim_wang->set_D_ham( { 0.0 ; 0.5 } );
	//stim_wang->set_D_glu( { 0.0 ; 0.5 } );
	//stim_wang->set_D_hfl( { 0.0 ; 0.5 } );
	//stim_wang->set_theta_ref( { 0.017 ; 0.11 } );
	stim_wang->set_si_vas( { 0.3 ; 1.0 } );
	stim_wang->set_si_rf( { 0.0 ; 1.0 } );
	stim_wang->set_si_glu( { 0.2 ; 1.0 } );
	stim_wang->set_si_hfl( { 0.0 ; 1.0 } );
	stim_wang->set_K_sp_vas( { 0.0 ; 5.0 } );
	stim_wang->set_K_sp_glu( { 0.0 ; 5.0 } );
	stim_wang->set_K_sp_hfl( { 0.0 ; 5.0 } );
	stim_wang->set_D_sp_vas( { 0.0 ; 0.5 } );
	stim_wang->set_D_sp_glu( { 0.0 ; 0.5 } );
	stim_wang->set_D_sp_hfl( { 0.0 ; 0.5 } );
	stim_wang->set_theta_k_ref( { 0.0 ; 0.3 } );
	stim_wang->set_theta_h_ref0( { 0.2 ; 0.9 } );
	//stim_wang->set_c_d( { 0.0 ; 0.0 } );
	//stim_wang->set_c_v( { 0.0 ; 0.0 } );
	stim_wang->set_d_sp( { -0.2 ; 0.0 } );
	stim_wang->set_d_si( { 0.2 ; 0.7 } );
	//stim_wang->set_k_THETA( { 1.0 ; 5.7 } );
	stim_wang->set_k_theta( { 1.0 ; 15.0 } );
	stim_wang->set_phi_off_pk( { 0.05 ; 0.2 } );

	//switch controler 
	//stim_wang->set_t_switch( { 4.745 ; 5.087 } );
}
