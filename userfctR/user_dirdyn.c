//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2015
// Last update : 08/05/2015
//---------------------------

#include "math.h"
#include "MBSdef.h"
#include "mbs_data.h"
#include "mbs_dirdyn.h"
#include "user_simu_functions.h"
#include "contact_interface.h"

/*! \brief user own initialization functions
 * 
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
	// init simulation
	simu_init(mbs_data);

	// controller initialization
	controller_init_interface(mbs_data);

	// 3D contact initialization
	init_contact_geom(mbs_data);

	// adding 3D contact to main simulation class
	simu_init_contact(mbs_data);
}

/*! \brief user own loop functions
 * 
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
	// 3D contact user state
	user_state_contact_geom(mbs_data);

	// main loop user
	simu_controller_loop(mbs_data);
}

/*! \brief user own finishing functions
 * 
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
	// close 3D contact model
	close_contact_geom(mbs_data);

	// close main user
	simu_finish(mbs_data);
}
