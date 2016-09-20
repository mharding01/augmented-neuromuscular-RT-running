/*! 
 * \author Nicolas Van der Noot
 * \file MainUnionShape.hh
 * \brief MainUnionShape class
 */

#ifndef _MAIN_UNION_SHAPE_HH_
#define _MAIN_UNION_SHAPE_HH_

#include "UnionShape.hh"
#include "RigidShape.hh"
#include "user_contact_kinematics.hh"
#include <vector>

namespace ContactGeom{

/*! \brief main union of shapes
 */
class MainUnionShape: public UnionShape
{
    public:
        MainUnionShape(MbsData *mbs_data);
        virtual ~MainUnionShape();

        void add_rigid_F(RigidShape *new_rigid);
        void add_rigid_S(RigidShape *new_rigid);
        void add_kin_info(RigidShape *rigid_shape);

        void apply_F_T(int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
        void mbs_sensor_compute();
        void kin_info_apply();
        void create_tabs();

        double**  get_PxF_tab()  { return PxF_tab;  }
        double**  get_VxF_tab()  { return VxF_tab;  }
        double**  get_OMxF_tab() { return OMxF_tab; }
        double*** get_RxF_tab()  { return RxF_tab;  }

        std::vector<KinInfo>& get_kin_info_list() { return kin_info_list; }

        // get shapes
        int get_rigid_F_size() const { return rigid_F_list.size(); }
        int get_rigid_S_size() const { return rigid_S_list.size(); }

        RigidShape* get_rigid_F(int i) { return rigid_F_list[i]; }
        RigidShape* get_rigid_S(int i) { return rigid_S_list[i]; }

    private:
        int nb_joints; ///< number of joints in Robotran

        std::vector<RigidShape*> rigid_F_list; ///< list of all rigid shapes with Fsensor
        std::vector<RigidShape*> rigid_S_list; ///< list of all rigid shapes with Ssensor
        std::vector<KinInfo> kin_info_list; ///< kinematic info computed

        double **PxF_tab;  ///< 'PxF'  tabular for user kinematics computation
        double **VxF_tab;  ///< 'VxF'  tabular for user kinematics computation
        double **OMxF_tab; ///< 'OMxF' tabular for user kinematics computation
        double ***RxF_tab; ///< 'RxF'  tabular for user kinematics computation

        // functions prototype
        double** create_tab_2d(int x, int y);
        double*** create_tab_3d(int x, int y, int z);
        void free_tab_2d(double **tab, int x);
        void free_tab_3d(double ***tab, int x, int y);
};

}
#endif
