
#include "FitnessQq.hh"
#include "TorqueFitness.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
FitnessQq::FitnessQq(MbsData *mbs_data, Ctrl *ctrl): Fitness(mbs_data, simu_index, ctrl, sens_info)
{
	if (ctrl->get_ctrl_id() == NICO_CTRL)
	{
		fitness_tab.push_back(new TorqueFitness(mbs_data, ctrl));
	}
}

/*! \brief destructor
 */
FitnessQq::~FitnessQq()
{
	//already done in mother class
}