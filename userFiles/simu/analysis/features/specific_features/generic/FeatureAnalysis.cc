
#include "FeatureAnalysis.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
FeatureAnalysis::FeatureAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index)
{
	this->mbs_data   = mbs_data;
	this->simu_index = simu_index;
}

/*! \brief destructor
 */
FeatureAnalysis::~FeatureAnalysis()
{

}
