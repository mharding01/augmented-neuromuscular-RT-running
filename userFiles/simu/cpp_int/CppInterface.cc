
#include "CppInterface.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
CppInterface::CppInterface(MbsData *mbs_data)
{
	this->mbs_data = mbs_data;
}

/*! \brief destructor
 */
CppInterface::~CppInterface()
{
	delete simu_ctrl;

	delete ctrl;
}
