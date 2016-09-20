//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//
//
//---------------------------

#include "MBSdef.h"
#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "compute_forces.h"

double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MbsData *mbs_data, double tsim,int ixF)
{
	double *SWr = mbs_data->SWr[ixF];

	compute_ext_forces(SWr, PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF);

	return SWr;
}

 
