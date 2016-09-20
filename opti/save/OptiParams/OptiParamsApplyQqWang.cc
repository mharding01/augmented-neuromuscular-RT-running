#include "OPTI_NAME.hh"
#include "JointsInit.hh"
#include "Body.hh"

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
	//init pos
	joints_init->set_T1_p( { 0.8 ; 2.0} );
	joints_init->set_T3( { 0.48 ; 0.58 } );
	joints_init->set_T3_p( { -0.5 ; 0.5 } );
	joints_init->set_R2( { 0.0 ; 0.3 } );
	joints_init->set_R2_p( { -5.0 ; 5.0 } );
	joints_init->set_r_sh_p( { -5.0 ; 5.0 } );
	joints_init->set_r_hip( { -0.87 ; -0.35 } );
	joints_init->set_r_hip_p( { -5.0 ; 5.0 } );
	joints_init->set_r_knee( { 0.0 ; 0.87 } );
	joints_init->set_r_knee_p( { -5.0 ; 5.0 } );
	joints_init->set_r_ankle( { -0.09 ; 0.09 } );
	joints_init->set_r_ankle_p( { -5.0 ; 5.0 } );
	joints_init->set_l_hip( { -0.09 ; 0.35 } );
	joints_init->set_l_hip_p( { -5.0 ; 5.0 } );
	joints_init->set_l_knee( { 0.35 ; 1.75 } );
	joints_init->set_l_knee_p( { -5.0 ; 5.0 } );
	joints_init->set_l_ankle( { 0.09 ; 0.45 } );
	joints_init->set_l_ankle_p( { -5.0 ; 5.0 } );

	//scale and shift of torque data
	body->set_arh( { 0.5 ; 2.0 } );
	body->set_ark( { 0.5 ; 2.0 } );
	body->set_ara( { 0.5 ; 2.0 } );
	body->set_crh( { 0.5 ; 2.0 } );
	body->set_crk( { 0.5 ; 2.0 } );
	body->set_cra( { 0.5 ; 2.0 } );
	body->set_brh( { 0.0 ; 1.0 } );
	body->set_brk( { 0.0 ; 1.0 } );
	body->set_bra( { 0.0 ; 1.0 } );
	body->set_alh( { 0.5 ; 2.0 } );
	body->set_alk( { 0.5 ; 2.0 } );
	body->set_ala( { 0.5 ; 2.0 } );
	body->set_clh( { 0.5 ; 2.0 } );
	body->set_clk( { 0.5 ; 2.0 } );
	body->set_cla( { 0.5 ; 2.0 } );
	body->set_blh( { 0.0 ; 1.0 } );
	body->set_blk( { 0.0 ; 1.0 } );
	body->set_bla( { 0.0 ; 1.0 } );
}
