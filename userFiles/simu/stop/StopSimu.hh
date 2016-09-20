/*! 
 * \author Nicolas Van der Noot
 * \file StopSimu.hh
 * \brief StopSimu class
 */

#ifndef _STOP_SIMU_HH_
#define _STOP_SIMU_HH_

#include "ComputationSimu.hh"
#include "SensorsInfo.hh"
#include "JointsLimitRobot.hh"
#include "RigidShape.hh"

/*! \brief Stop the simulation according to some rules
 */
class StopSimu: public ComputationSimu
{
	public:
		StopSimu(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info);
		virtual ~StopSimu();

		virtual void compute();

		void set_rfoot_shape(ContactGeom::RigidShape *r_foot) { rfoot_shape = r_foot; }
		void set_lfoot_shape(ContactGeom::RigidShape *l_foot) { lfoot_shape = l_foot; }

	private:
		int n_joint;   ///< number of joints in the simulation
		int gcm_model; ///< ground contact model

		SensorsInfo *sens_info; ///< info from sensors
		
		JointsLimitRobot *limit_joints; ///< joint limits

		ContactGeom::RigidShape *rfoot_shape; ///< right foot shape
		ContactGeom::RigidShape *lfoot_shape; ///< left foot shape
};

#endif
