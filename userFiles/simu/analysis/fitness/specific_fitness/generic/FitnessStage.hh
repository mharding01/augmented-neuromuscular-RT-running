/*! 
 * \author Nicolas Van der Noot
 * \file FitnessStage.hh
 * \brief FitnessStage class
 */

#ifndef _FITNESS_STAGE_HH_
#define _FITNESS_STAGE_HH_

#ifdef __cplusplus
extern "C" {
#endif
	#include "mbs_data.h"
#ifdef __cplusplus
}
#endif

#include <cmath>

/*! \brief One stage of the fitness function computation
 */
class FitnessStage
{
	public:
		FitnessStage(MbsData *mbs_data);
		virtual ~FitnessStage();

		inline double compute_gaussian_fitness(double error, double mult_gain, double expo_gain)
		{
			return mult_gain*exp(- error * error * expo_gain);
		}

		virtual void compute() = 0;
		virtual double get_fitness() = 0;
		virtual int next_stage_unlocked() = 0;

	protected:
		MbsData *mbs_data; ///< Robotran structure
		double max_fitness;
};

#endif
