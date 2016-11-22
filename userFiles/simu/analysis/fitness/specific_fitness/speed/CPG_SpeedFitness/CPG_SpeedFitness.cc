#include "CPG_SpeedFitness.hh"
#include "cmake_config.h"
#include "user_IO.h"
#include "NicoCtrl.hh"
#include "Body.hh"

#define VELOCITY_MARGIN 0.05

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
CPG_SpeedFitness::CPG_SpeedFitness(MbsData *mbs_data, Ctrl *ctrl, SensorsInfo *sens_info): SpeedFitness(mbs_data, sens_info)
{
    /* Extract the Stims controller object */
    NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);
    Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));
	stims = static_cast<StimWangCtrl*>(lower_body->get_stim_ctrl());
}

/*! \brief destructor
 */
CPG_SpeedFitness::~CPG_SpeedFitness()
{

}

/*! \brief compute variables at each time step
 */
void CPG_SpeedFitness::compute()
{
	double t;

	if (stims->is_cpg_ctrl_active()) /* Only start when cpg contrl active */
	{
		t = mbs_data->tsim;

		if (!speed_opti_started)
		{
			speed_opti_started = 1;
			speed_opti_x_start = sens_info->get_S_MidWaist_P(0);
			speed_opti_t_start = t;
			speed_opti = 0.0;
		}
		else if (t > speed_opti_t_start)
		{
			speed_opti = (sens_info->get_S_MidWaist_P(0) - speed_opti_x_start) / (t - speed_opti_t_start); // mean velocity
		}
	}
}
