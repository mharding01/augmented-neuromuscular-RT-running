#include "WalkState.hh"

/*! \brief constructor
 */
WalkState::WalkState(CtrlInputs *inputs, CtrlOptions *options)
{
	this->inputs = inputs;
	this->options = options;
}

/*! \brief destructor
 */
WalkState::~WalkState()
{

}
