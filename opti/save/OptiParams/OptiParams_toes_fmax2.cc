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
	stim_wang->set_S0_ta_st( { 0.01 ; 0.15 } );
	stim_wang->set_S0_gas_st( { 0.01 ; 0.15 } );
	stim_wang->set_S0_vas_st( { 0.01 ; 0.8 } );
	stim_wang->set_S0_ham_st( { 0.01 ; 0.8 } );
	stim_wang->set_S0_rf_st( { 0.01 ; 0.075 } );
	stim_wang->set_S0_glu_st( { 0.01 ; 0.8 } );
	stim_wang->set_S0_hfl_st( { 0.01 ; 0.02 } );
	stim_wang->set_S0_sol_sw( { 0.01 ; 0.2 } );
	stim_wang->set_S0_ta_sw( { 0.01 ; 0.35 } );
	stim_wang->set_S0_gas_sw( { 0.01 ; 0.8 } );
	stim_wang->set_S0_vas_sw( { 0.01 ; 0.25 } );
	stim_wang->set_S0_ham_sw( { 0.01 ; 0.05 } );
	stim_wang->set_S0_rf_sw( { 0.01 ; 0.25 } );
	stim_wang->set_S0_glu_sw( { 0.01 ; 0.1 } );
	stim_wang->set_S0_hfl_sw( { 0.01 ; 0.6 } );
	stim_wang->set_G_sol( { 0.0 ; 1.5 } );
	stim_wang->set_G_sol_ta( { 0.0 ; 2.0 } );
	stim_wang->set_G_gas( { 0.0 ; 2.0 } );
	stim_wang->set_G_vas( { 0.0 ; 20.0 } );
	stim_wang->set_G_ham( { 0.0 ; 0.5 } );
	stim_wang->set_G_glu( { 0.0 ; 0.35 } );
	stim_wang->set_G_ta_sw( { 0.5 ; 12.5 } );
	stim_wang->set_G_ta_st( { 0.5 ; 30.0 } );
	stim_wang->set_G_hfl( { 0.5 ; 20.0 } );
	stim_wang->set_G_ham_hfl( { 0.0 ; 50.0 } );
	stim_wang->set_l_off_ta_sw( { 0.0 ; 2.0 } );
	stim_wang->set_l_off_ta_st( { 0.0 ; 2.0 } );
	stim_wang->set_l_off_ham_hfl( { 0.0 ; 10.0 } );
	stim_wang->set_l_off_hfl( { 0.0 ; 1.0 } );
	stim_wang->set_K_ham( { 0.0 ; 10.0 } );
	stim_wang->set_K_glu( { 0.0 ; 30.0 } );
	stim_wang->set_K_hfl( { 0.0 ; 7.5 } );
	stim_wang->set_D_ham( { 0.0 ; 2.0 } );
	stim_wang->set_D_glu( { 0.0 ; 1.5 } );
	stim_wang->set_D_hfl( { 0.0 ; 1.0 } );
	stim_wang->set_theta_ref( { 0.0 ; 0.4 } );
	stim_wang->set_si_vas( { 0.3 ; 1.0 } );
	stim_wang->set_si_rf( { 0.0 ; 1.0 } );
	stim_wang->set_si_glu( { 0.2 ; 1.0 } );
	stim_wang->set_si_hfl( { 0.1 ; 1.0 } );
	stim_wang->set_K_sp_vas( { 0.0 ; 4.0 } );
	stim_wang->set_K_sp_glu( { 0.0 ; 5.0 } );
	stim_wang->set_K_sp_hfl( { 0.0 ; 2.0 } );
	stim_wang->set_D_sp_vas( { 0.0 ; 3.0 } );
	stim_wang->set_D_sp_glu( { 0.0 ; 0.5 } );
	stim_wang->set_D_sp_hfl( { 0.0 ; 7.0 } );
	stim_wang->set_theta_k_ref( { 0.0 ; 8.0 } );
	stim_wang->set_theta_h_ref0( { 0.5 ; 8.0 } );
	stim_wang->set_c_d( { 0.0 ; 10.0 } );
	stim_wang->set_c_v( { 0.0 ; 5.0 } );
	stim_wang->set_d_sp( { -0.2 ; 0.1 } );
	stim_wang->set_d_si( { 0.2 ; 0.7 } );
	stim_wang->set_k_THETA( { 0.0 ; 2.5 } );
	stim_wang->set_k_theta( { 0.0 ; 25.0 } );
	stim_wang->set_phi_off_pk( { 0.0 ; 30.0 } );

	// initial motion
	impedance_init->set_x_ref_end( {0.0 ; 0.08} );
}
