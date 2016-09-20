/*! 
 * \author Nicolas Van der Noot
 * \file FitnessQq.hh
 * \brief FitnessQq class
 */

#ifndef _FITNESS_QQ_HH_
#define _FITNESS_QQ_HH_

#include "Fitness.hh"


/*! \brief Computation of the fitness function for optimization
 */
class FitnessQq: public Fitness
{
	public:
		FitnessQq(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~FitnessQq();

	private:
};

#endif
