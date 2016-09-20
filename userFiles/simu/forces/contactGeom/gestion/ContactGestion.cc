#include "ContactGestion.hh"
#include "user_shapes.hh"

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
ContactGestion::ContactGestion(MbsData *mbs_data)
{
    this->mbs_data = mbs_data;

    // create global list of shapes (main_union)
    main_union = new MainUnionShape(mbs_data);

    // user main_union configuration
    config_shapes(main_union, mbs_data);

    // create tabulars for user kinematics
    main_union->create_tabs();

    // update the check_flag flags
    main_union->update_check_flag();

    // check for correct lists
    main_union->check_list();
}

/*! \brief destructor
 */
ContactGestion::~ContactGestion()
{
    delete main_union;
}

/*! \brief update the bodies kinematics
 */
void ContactGestion::update_kinematics()
{
    user_contact_kinematics(main_union->get_kin_info_list(), main_union, mbs_data,
        main_union->get_PxF_tab(), main_union->get_VxF_tab(), main_union->get_OMxF_tab(), main_union->get_RxF_tab());

    main_union->kin_info_apply();

    main_union->update_kinematics();
}

/*! \brief update the user possible states
 */
void ContactGestion::update_user_state()
{
    main_union->user_states();
}

/*! \brief update the forces and torques of all bodies
 */
void ContactGestion::update_F_T()
{
    main_union->reset_F_T();
    main_union->update_F_T_main();
    main_union->gather_F_T();
}

}
