/*! 
 * \author Nicolas Van der Noot
 * \file MinDistFitness.hh
 * \brief MinDistFitness class
 */

#ifndef _MIN_DIST_FITNESS_HH_
#define _MIN_DIST_FITNESS_HH_

#include "FitnessStage.hh"
#include "SensorsInfo.hh"

/*! \brief One stage of the fitness function computation:
 * walk a minimal distance
 */
class MinDistFitness: public FitnessStage
{
	public:
		MinDistFitness(MbsData *mbs_data, SensorsInfo *sens_info);
		virtual ~MinDistFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

		double get_max_x_before_fall() { return max_x_before_fall; }

	private:
		double max_x_before_fall; ///< max x distance before falling [m]

		SensorsInfo *sens_info; ///< sensors info

		double tf; ///< simulation final time [s]
		double lim_x_travel; ///< distance to travel to get the maximal score
};

#endif
