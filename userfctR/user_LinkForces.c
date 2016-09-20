//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "link_compute.h"

double user_LinkForces(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk)
{
	return link_compute(Z, Zd, mbs_data, tsim, ilnk);
}
