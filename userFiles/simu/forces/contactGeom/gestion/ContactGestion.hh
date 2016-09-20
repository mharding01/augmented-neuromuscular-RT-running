/*! 
 * \author Nicolas Van der Noot
 * \file ContactGestion.hh
 * \brief ContactGestion class
 */

#ifndef _CONTACT_GESTION_HH_
#define _CONTACT_GESTION_HH_

#include "mbs_data.h"
#include "GeometryShape.hh"
#include "MainUnionShape.hh"
#include "RigidShape.hh"

namespace ContactGeom{

/*! \brief main contact gestion class
 */
class ContactGestion
{
    public:
        ContactGestion(MbsData *mbs_data);
        ~ContactGestion();

        void update_kinematics();
        void update_user_state();
        void update_F_T();

        MainUnionShape* get_main_union() const { return main_union; }

    private:
        MbsData *mbs_data; ///< Robotran structure
        MainUnionShape *main_union; ///< list of all geometric shapes
};

}
#endif
