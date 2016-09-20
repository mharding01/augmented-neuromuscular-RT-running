
#include "CtrlOutputs.hh"

/*! \brief constructor
 */
CtrlOutputs::CtrlOutputs(MotorCtrlIndex *ctrl_index)
{
	nb_mot = ctrl_index->get_nb_mot();

	for(int i=0; i<nb_mot; i++)
	{
		q_ref.push_back(0.0);
		Qq_ref.push_back(0.0);
	}

	this->ctrl_index = ctrl_index;
}

/*! \brief destructor
 */
CtrlOutputs::~CtrlOutputs()
{

}
