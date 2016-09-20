/*! 
 * \author Nicolas Van der Noot
 * \file SpeedFitness.hh
 * \brief SpeedFitness class
 */

#ifndef _SPEED_FITNESS_HH_
#define _SPEED_FITNESS_HH_

#include "FitnessStage.hh"
#include "SensorsInfo.hh"

/*! \brief One stage of the fitness function computation:
 * velocity target matching
 */
class SpeedFitness: public FitnessStage
{
	public:
		SpeedFitness(MbsData *mbs_data, SensorsInfo *sens_info);
		virtual ~SpeedFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

		void set_speed_ref(double value) { speed_ref = value; }

	private:
		SensorsInfo *sens_info; ///< info from the sensors

		int speed_opti_started; ///< 1 if computation started, 0 otherwise

		double speed_opti_x_start; ///< x position when computation started [m]
		double speed_opti_t_start; ///< time when computation started [s]

		double speed_opti; ///< speed for opti [m/s]

		double speed_ref; ///< speed reference (target) [m/s]
};

#endif
