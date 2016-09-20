/*! 
 * \author Nicolas Van der Noot
 * \file FitnessRun.hh
 * \brief FitnessRun class
 */

#ifndef _FITNESS_RUN_HH_
#define _FITNESS_RUN_HH_

#include "Fitness.hh"


/*! \brief Computation of the fitness function for optimization
 */
class FitnessRun: public Fitness
{
	public:
		FitnessRun(MbsData *mbs_data, Ctrl *ctrl, SensorsInfo *sens_info);
		virtual ~FitnessRun();

	private:
};

#endif
