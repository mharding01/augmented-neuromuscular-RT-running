/*! 
 * \author Nicolas Van der Noot
 * \file ComputeGCM.hh
 * \brief ComputeGCM class
 */

#ifndef _COMPUTE_GCM_HH_
#define _COMPUTE_GCM_HH_

#include "WholeFeet.hh"
#include "GaitFeatures.hh"
#include "SensorsInfo.hh"
#include "GroundModel.hh"

/*! \brief Computation of the Ground Contact Model (GCM) forces
 */
class ComputeGCM
{
	public:
		ComputeGCM(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info);
		~ComputeGCM();

		void compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index);
		void state_compute();

		/// get whole_feet
		inline WholeFeet* get_whole_feet() { return whole_feet; }

	private:
		int coman_model; ///< COMAN model

		MbsData *mbs_data;           ///< Robotran structure
		WholeFeet *whole_feet;       ///< class with all the feet
		GroundModel *ground;         ///< ground model
		GaitFeatures *gait_features; ///< gait features
		SensorsInfo *sens_info;      ///< info from sensors
};

#endif
