/*! 
 * \author Nicolas Van der Noot
 * \file UnionShape.hh
 * \brief UnionShape class
 */

#ifndef _UNION_SHAPE_HH_
#define _UNION_SHAPE_HH_

#include "GeometryShape.hh"
#include "mbs_data.h"
#include <vector>

namespace ContactGeom{

class UnionShape: public GeometryShape
{
    public:
        UnionShape(MbsData *mbs_data, GeometryShape *parent_shape);
        virtual ~UnionShape();
    
        UnionShape* add_union();

        virtual void update_kinematics();
        virtual void update_check();
        virtual void update_check_flag();
        virtual void check_list();
        virtual void user_states();

        void update_F_T_main();
        virtual void reset_F_T();
        virtual void update_F_T(GeometryShape *other_shape);
        virtual void add_child_recursively(GeometryShape *child);

        GeometryShape* add_geometry(GeometryShape *geom);

        MbsData* get_mbs_data() const { return mbs_data; }

        virtual void set_all_contact_default();
        virtual void set_no_contact_default();

        virtual void gather_F_T();
        virtual void list_genealogy();

        // get shapes
        int get_list_geom_size() const { return list_geom.size(); }
        GeometryShape* get_geom(int i) { return list_geom[i]; }

    protected:
        MbsData *mbs_data; ///< Robotran structure

        std::vector<GeometryShape*> list_geom; ///< list with geometry shapes
};

}
#endif
