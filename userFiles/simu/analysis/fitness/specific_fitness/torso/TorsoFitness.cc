#include "TorsoFitness.hh"
#include "cmake_config.h"
#include "user_IO.h"
#include "NicoCtrl.hh"

#define DIST_TORSO_START 2.0 //[m]
#define TORSO_MARGIN 0.1 //[rad]

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
TorsoFitness::TorsoFitness(MbsData *mbs_data, SensorsInfo *sens_info, Ctrl *ctrl): FitnessStage(mbs_data)
{
	this->sens_info = sens_info;
	this->ctrl = ctrl;

	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);
	inputs = static_cast<CtrlInputs*>(nico_ctrl->get_inputs());

	torso_av = new AverageIncrement();
	torso_mean = 0.0;

	opti_started = 0;
	
	max_fitness = 250.0;
}

/*! \brief destructor
 */
TorsoFitness::~TorsoFitness()
{
	delete torso_av;
}

/*! \brief compute variables at each time step
 */
void TorsoFitness::compute()
{
	double t;

	if (mbs_data->q[1] > DIST_TORSO_START)
	{
		opti_started = 1;
        // Increment mean of torso's theta along y-axis, forward tilt
		torso_mean = torso_av->update_and_get(inputs->get_theta_torso(1)); 
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double TorsoFitness::get_fitness()
{
	if(opti_started) 
	{
		if (fabs(torso_mean) > TORSO_MARGIN)
		{
			return compute_gaussian_fitness(torso_mean, max_fitness, 20);
		}
		else
		{
			return max_fitness;
		}
	}
	else
	{
		return 0.0;
	}
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int TorsoFitness::next_stage_unlocked()
{
	return 1;
}
