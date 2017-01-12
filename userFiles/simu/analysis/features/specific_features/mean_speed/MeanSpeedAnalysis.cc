#include "MeanSpeedAnalysis.hh"
#include "SimuIndex.hh"
#include "user_IO.h"
#include "user_realtime.h"

#define DIST_VELOCITY_START 15.0 ///< distance to start speed computation TODO: Hardcoded, not important
#define DIST_VELOCITY_END 20.0  ///< distance to finish speed computation

#define SIZE_VEL_AVERAGE 4000 ///< size of the running average to compute speed

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
MeanSpeedAnalysis::MeanSpeedAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info):
	FeatureAnalysis(mbs_data, simu_index)
{
	flag_speed_started = 0;
	flag_speed_end     = 0;

	mean_speed    = 0.0;
	running_speed = 0.0;

	speed_x_start = 0.0;
	speed_t_start = 0.0;

	FJ_T1_id = simu_index->get_mbs_jt(SimuJointIndex::FloatingT1);

	speed_av_compute = new RunningAverage(SIZE_VEL_AVERAGE, mbs_data->qd[FJ_T1_id]);

	this->sens_info = sens_info;

	options = mbs_data->user_IO->options;
}

/*! \brief destructor
 */
MeanSpeedAnalysis::~MeanSpeedAnalysis()
{
	delete speed_av_compute;
}

/*! \brief compute this gait feature
 */
void MeanSpeedAnalysis::compute()
{
	double x_coman;

	// running speed
	running_speed = speed_av_compute->update_and_get(mbs_data->qd[FJ_T1_id]);

	set_plot(running_speed, "real speed [m/s]");

	// mean speed
	x_coman = mbs_data->q[FJ_T1_id];

	if (x_coman > DIST_VELOCITY_START)
	{
		if (!flag_speed_started)
		{
			flag_speed_started = 1;
			speed_x_start = sens_info->get_S_MidWaist_P(0);
			speed_t_start = mbs_data->tsim;
		}
		else if ( (!flag_speed_end) && (x_coman > DIST_VELOCITY_END) )
		{
			flag_speed_end = 1;
			mean_speed = (sens_info->get_S_MidWaist_P(0) - speed_x_start) / (mbs_data->tsim - speed_t_start); // mean speed

			if (options->print)
			{
				std::cout << "speed : " << mean_speed << " [m/s]" << std::endl;
			}
		}
	}
}
