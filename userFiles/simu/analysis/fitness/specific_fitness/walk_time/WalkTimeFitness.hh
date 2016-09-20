/*! 
 * \author Nicolas Van der Noot
 * \file WalkTimeFitness.hh
 * \brief WalkTimeFitness class
 */

#ifndef _WALK_TIME_FITNESS_HH_
#define _WALK_TIME_FITNESS_HH_

#include "FitnessStage.hh"

/*! \brief One stage of the fitness function computation:
 * walking time before falling
 */
class WalkTimeFitness: public FitnessStage
{
	public:
		WalkTimeFitness(MbsData *mbs_data);
		virtual ~WalkTimeFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		double tf; ///< final simulation time [s]
};

#endif
