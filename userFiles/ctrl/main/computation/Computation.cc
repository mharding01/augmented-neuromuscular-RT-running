
#include "Computation.hh"

/*! \brief constructor
 *
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 */
Computation::Computation(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index)
{
	this->inputs  = inputs;
	this->options = options;
	this->ctrl_index = ctrl_index;
}

/*! \brief destructor
 */
Computation::~Computation()
{

}
