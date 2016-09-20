#include "ComputationSimu.hh"
#include "user_IO.h"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
ComputationSimu::ComputationSimu(MbsData *mbs_data, ModelSimuIndex *simu_index)
{
	this->mbs_data   = mbs_data;
	this->simu_index = simu_index;

	options = mbs_data->user_IO->options;
}

/*! \brief destructor
 */
ComputationSimu::~ComputationSimu()
{

}
