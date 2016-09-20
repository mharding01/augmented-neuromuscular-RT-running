#include "MainUnionShape.hh"

extern "C" {
    #include "mbs_project_interface.h"
}

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
MainUnionShape::MainUnionShape(MbsData *mbs_data): UnionShape(mbs_data, NULL)
{
    nb_joints = mbs_data->njoint;
}

/*! \brief destructor
 */
MainUnionShape::~MainUnionShape()
{
    int size;

    size = kin_info_list.size();

    if (PxF_tab != NULL)
    {
        free_tab_2d(PxF_tab, size);
    }

    if (VxF_tab != NULL)
    {
        free_tab_2d(VxF_tab, size);
    }
    
    if (OMxF_tab != NULL)
    {
        free_tab_2d(OMxF_tab, size);
    }

    if (RxF_tab != NULL)
    {
        free_tab_3d(RxF_tab, size, 4);
    }    

    for(int i=0; i<size; i++)
    {
        free_sensor(&(kin_info_list[i].sens));
    }
}

/*! \brief add a new Rigid Shape with S sensor
 * 
 * \param[in] new_rigid new rigid shape
 */
void MainUnionShape::add_rigid_S(RigidShape *new_rigid)
{
    rigid_S_list.push_back(new_rigid);
    add_kin_info(new_rigid);
}

/*! \brief add a new Rigid Shape with F sensor
 * 
 * \param[in] new_rigid new rigid shape
 */
void MainUnionShape::add_rigid_F(RigidShape *new_rigid)
{
    rigid_F_list.push_back(new_rigid);
    add_kin_info(new_rigid);
}

/*! \brief add a new ID used to get sensors kinematics
 * 
 * \param[in] rigid_shape rigid shape added
 */
void MainUnionShape::add_kin_info(RigidShape *rigid_shape)
{
    // increment list
    kin_info_list.push_back(KinInfo());

    // initialize with 0 values
    for(int i=0; i<3; i++)
    {
        kin_info_list.back().P[i]  = 0.0;
        kin_info_list.back().V[i]  = 0.0;
        kin_info_list.back().OM[i] = 0.0;

        for(int j=0; j<3; j++)
        {
            kin_info_list.back().R[i][j] = 0.0;
        }
    }

    // isens related
    kin_info_list.back().isens = rigid_shape->get_isens();
    kin_info_list.back().rigid_shape = rigid_shape;

    // Robotran sensor
    allocate_sensor(&(kin_info_list.back().sens), nb_joints);
    init_sensor(&(kin_info_list.back().sens), nb_joints);
}

/*! \brief apply the computed force on the Fsensor if its id is in the list of rigid F shapes
 * 
 * \param[in] ixF ID of the Fsensor
 * \param[out] Fx force in the x direction
 * \param[out] Fy force in the y direction
 * \param[out] Fz force in the z direction
 * \param[out] Mx moment in the x direction
 * \param[out] My moment in the y direction
 * \param[out] Mz moment in the z direction
 */
void MainUnionShape::apply_F_T(int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz)
{
    RigidShape *cur_rigid;

    for(unsigned int i=0; i<rigid_F_list.size(); i++)
    {
        cur_rigid = rigid_F_list[i];

        if ( (cur_rigid->get_flag() == RIGID_F_SENS) && (cur_rigid->get_Fsens() == ixF) )
        {
            cur_rigid->apply_F_T(Fx, Fy, Fz, Mx, My, Mz);

            break;
        }
    }
}

/*! \brief fill the kinematics info with the Robotran 'mbs_sensor' function
 */
void MainUnionShape::mbs_sensor_compute()
{
    int cur_isens;

    for(unsigned int i=0; i<kin_info_list.size(); i++)
    {
        cur_isens = kin_info_list[i].isens;

        if (cur_isens <= 0)
        {
            std::cout << "Error, S sens ID " << cur_isens << " should be strictly positive !" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Robotran kinematics computation
        init_sensor(&(kin_info_list[i].sens), nb_joints);

        mbs_sensor(&(kin_info_list[i].sens), mbs_data, cur_isens);

        // copy values computed from Robotran
        for(int j=0; j<3; j++)
        {
            kin_info_list[i].P[j]  = kin_info_list[i].sens.P[1+j];
            kin_info_list[i].V[j]  = kin_info_list[i].sens.V[1+j];
            kin_info_list[i].OM[j] = kin_info_list[i].sens.OM[1+j];

            for(int k=0; k<3; k++)
            {
                kin_info_list[i].R[j][k] = kin_info_list[i].sens.R[1+j][1+k];
            }
        }
    }
}

/*! \brief apply the kinematics info computed in kin_info_list
 */
void MainUnionShape::kin_info_apply()
{
    for(unsigned int i=0; i<kin_info_list.size(); i++)
    {
        kin_info_list[i].rigid_shape->update_kinematics(kin_info_list[i]);
    }
}

/*! \brief allocate memory for the kinematics tabulars
 */
void MainUnionShape::create_tabs()
{
    int size;

    size = kin_info_list.size();

    PxF_tab  = create_tab_2d(size, 4);
    VxF_tab  = create_tab_2d(size, 4);
    OMxF_tab = create_tab_2d(size, 4);
    RxF_tab  = create_tab_3d(size, 4, 4);
}

/*! \brief create a 2D tabular of 'double'
 * 
 * \param[in] x first index size
 * \param[in] y second index size
 * \return allocated tabular
 */
double** MainUnionShape::create_tab_2d(int x, int y)
{
    double **result;

    result = (double**) malloc(x*sizeof(double*));

    for(int i=0; i<x; i++)
    {
        result[i] = (double*) malloc(y*sizeof(double));
    }

    return result;
}

/*! \brief create a 3D tabular of 'double'
 * 
 * \param[in] x first index size
 * \param[in] y second index size
 * \param[in] z third index size
 * \return allocated tabular
 */
double*** MainUnionShape::create_tab_3d(int x, int y, int z)
{
    double ***result;

    result = (double***) malloc(x*sizeof(double**));

    for(int i=0; i<x; i++)
    {
        result[i] = create_tab_2d(y, z);
    }

    return result;
}

/*! \brief release the memory of a 2D tabular
 * 
 * \param[out] tab tabular to release
 * \param[in] x first index size
 */
void MainUnionShape::free_tab_2d(double **tab, int x)
{
    for(int i=0; i<x; i++)
    {
        free(tab[i]);
    }
    free(tab);
}

/*! \brief release the memory of a 3D tabular
 * 
 * \param[out] tab tabular to release
 * \param[in] x first index size
 * \param[in] y second index size
 */
void MainUnionShape::free_tab_3d(double ***tab, int x, int y)
{
    for(int i=0; i<x; i++)
    {
        free_tab_2d(tab[i], y);
    }
    free(tab);
}

}
