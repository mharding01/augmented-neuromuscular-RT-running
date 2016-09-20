/*! 
 * \author Nicolas Van der Noot
 * \file FlightFitness.hh
 * \brief FlightFitness class
 */

#ifndef _FLIGHT_FITNESS_HH_
#define _FLIGHT_FITNESS_HH_

#include "FitnessStage.hh"
#include "SwingStanceState.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 *  maximise flight time
 */
class FlightFitness: public FitnessStage
{
	public:
		FlightFitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~FlightFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		SwingStanceState *sw_st; ///< swing-stance state

		double tf; ///< simulation final time [s]
		double dt; ///< time step [s]
		double time_flight; ///< time of flight phase [s]
		double flight_per_cycle; ///< mean time of flight phaFLIGHT
		int flight_opti_started; ///< 1 if computation started, 0 otherwise
		double flight_opti_t_start; ///< time when computation started [s]
};

#endif
