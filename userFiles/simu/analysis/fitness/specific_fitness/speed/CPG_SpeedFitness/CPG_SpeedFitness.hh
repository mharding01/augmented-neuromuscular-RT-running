/*! 
 * \author Matthew Harding
 * \file CPG_SpeedFitness.hh
 * \brief CPG_SpeedFitness class
 */

#ifndef _CPG_SPEED_FITNESS_HH_
#define _CPG_SPEED_FITNESS_HH_

#include "SpeedFitness.hh"
#include "SensorsInfo.hh"
#include "Ctrl.hh"
#include "StimWangCtrl.hh"

/*! \brief One stage of the fitness function computation:
 * velocity target matching
 */
class CPG_SpeedFitness: public SpeedFitness 
{
	public:
		CPG_SpeedFitness(MbsData *mbs_data, Ctrl *ctrl, SensorsInfo *sens_info);
		virtual ~CPG_SpeedFitness();
		virtual void compute();
	private:

        StimWangCtrl *stims;     // Pointer to stims for controller in use
};

#endif