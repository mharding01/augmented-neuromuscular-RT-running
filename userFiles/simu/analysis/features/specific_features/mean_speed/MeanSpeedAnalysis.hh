/*! 
 * \author Nicolas Van der Noot
 * \file MeanSpeedAnalysis.hh
 * \brief MeanSpeedAnalysis class
 */

#ifndef _MEAN_SPEED_ANALYSIS_HH_
#define _MEAN_SPEED_ANALYSIS_HH_

#include "FeatureAnalysis.hh"
#include "RunningAverage.hh"
#include "SensorsInfo.hh"
#include "SimuOptions.h"

/*! \brief Mean speed computation
 */
class MeanSpeedAnalysis: public FeatureAnalysis
{
	public:
		MeanSpeedAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info);
		virtual ~MeanSpeedAnalysis();

		virtual void compute();

		/// get mean_speed
		inline double get_mean_speed() { return mean_speed; }

	private:
		int flag_speed_started; ///< 1 if started to compute speed, 0 otherwise
		int flag_speed_end;     ///< 1 if finished to computed speed, 0 otherwise

		int FJ_T1_id; ///< ID of the floating joint T1 in MBS

		double mean_speed;    ///< mean speed on a huge period [m/s]
		double running_speed; ///< mean speed as running average [m/s]

		double speed_x_start; ///< position to compute speed at start [m]
		double speed_t_start; ///< time to compute speed at start [t]

		RunningAverage *speed_av_compute; ///< running average to compute the speed [m/s]
		SensorsInfo *sens_info;           ///< info from the sensors
		
		SimuOptions *options; ///< simulation options
};

#endif
