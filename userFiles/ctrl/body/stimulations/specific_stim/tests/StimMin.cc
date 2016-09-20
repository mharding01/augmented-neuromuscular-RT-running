#include "StimMin.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimMin::StimMin(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options):
	StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{

}

/*! \brief destructor
 */
StimMin::~StimMin()
{

}

/*! \brief main stimulations computation
 */
void StimMin::compute()
{
	// nothing to do, already S_MIN
}
