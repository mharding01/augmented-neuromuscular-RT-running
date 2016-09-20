
#include "compute_forces.h"
#include "CppInterface.hh"
#include "ComputeGCM.hh"
#include "ModelSimuIndex.hh"
#include "SimuIndex.hh"
#include "SimuOptions.h"
#include "user_model.h"
#include "user_IO.h"
#include "contact_interface.h"

/*! \brief compute the external ground reactions related to one ext force sensor
 * 
 * \param[out] F external force [N]
 * \param[out] T external torque [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the external sensor
 */
void compute_gcm(double F[3], double T[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], MbsData* mbs_data, int index)
{
	double F_tot[3], T_tot[3];

	CppInterface *cpp_int;
	ComputeGCM *gcm_mesh;

	cpp_int = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	gcm_mesh = cpp_int->get_simu_ctrl()->get_gcm_mesh();

	gcm_mesh->compute_F_T(F_tot, T_tot, PxF, RxF, VxF, OMxF, index);

	for (int i=0; i<3; i++)
	{
		F[i] = F_tot[i];
		T[i] = T_tot[i];
	}
}

/*! \brief user_ExtForces call interface with c++
 * 
 * \param[out] SWr outputs (Fx, Fy, Fz, Mx, My, Mz, dxF, dyF, dzF)
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] AxF
 * \param[in] OMPxF
 * \param[in] mbs_data Robotran structure
 * \param[in] tsim simulation time [s]
 * \param[in] ixF index of the external sensor (as in the MBS file)
 */
void compute_ext_forces(double *SWr, double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], double AxF[4], double OMPxF[4], MbsData *mbs_data, double tsim, int ixF)
{
	// variables declaration
	int Fsens_id, idpt;

	SimuCtrl *simu_ctrl;
	ModelSimuIndex *simu_index;
	SimuOptions *options;

	double Fx = 0.0, Fy = 0.0, Fz = 0.0;
	double Mx = 0.0, My = 0.0, Mz = 0.0;

	double F[3] = {0.0, 0.0, 0.0};
	double T[3] = {0.0, 0.0, 0.0};

	// options
	options = mbs_data->user_IO->options;


	// -- Ext Forces init -- //

	for(int i=1; i<=9; i++)
	{
		SWr[i] = 0.0;
	}

	idpt = mbs_data->xfidpt[ixF];

	for(int i=0; i<3; i++)
	{
		SWr[7+i] = mbs_data->dpt[1+i][idpt];
	}


	// -- 3D contact model -- //

	// update the contact primitives model: kinematics and force-torque of the shapes
	if (ixF == 1)
	{
		update_contact_geom(mbs_data);
	}

	// apply force-torque computed by the contact primitives model
	apply_contact_geom(mbs_data, ixF, &Fx, &Fy, &Fz, &Mx, &My, &Mz);

	SWr[1] = Fx;
	SWr[2] = Fy;
	SWr[3] = Fz;
	SWr[4] = Mx;
	SWr[5] = My;
	SWr[6] = Mz;

	// -- Mesh GCM -- //

	if (options->gcm_model == MESH_GCM_MODEL)
	{
		simu_ctrl = static_cast<CppInterface*>(mbs_data->user_model->cppInterface)->get_simu_ctrl();

		simu_index = simu_ctrl->get_simu_index();

		Fsens_id = simu_index->get_ind_F(ixF);

		switch (Fsens_id)
		{
			case SimuFsensIndex::RightFoot :
			case SimuFsensIndex::LeftFoot  :
			case SimuFsensIndex::RightFlexProx :
			case SimuFsensIndex::LeftFlexProx  :
			case SimuFsensIndex::RightFlexDist :
			case SimuFsensIndex::LeftFlexDist  :
			case SimuFsensIndex::RightToe : 
			case SimuFsensIndex::LeftToe  :
				compute_gcm(F, T, PxF, RxF, VxF, OMxF, mbs_data, Fsens_id);

				// gather inside SWr
				for(int i=0; i<3; i++)
				{
					SWr[1+i] = F[i];
					SWr[4+i] = T[i];
				}
				break;
		
			default:
				break;
		}	
	}
}
