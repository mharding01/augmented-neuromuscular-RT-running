
#include "StimulationCtrl.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimulationCtrl::StimulationCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options)
{
	this->inputs = inputs;
	this->parts = parts;
	this->ws = ws;
	this->fwd_kin = fwd_kin;
	this->options = options;

	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		for (int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i].push_back(S_MIN);	
		}
	}
}

/*! \brief destructor
 */
StimulationCtrl::~StimulationCtrl()
{

}
