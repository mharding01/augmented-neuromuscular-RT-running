
#include "HandleOutputs.hh"
#include "ctrl_functions.hh"

#define MIN_TORQUE -30.0 ///< min torque value [Nm]
#define MAX_TORQUE  30.0 ///< max torque value [Nm]

/*
 * uncomment the following line to limit the motors torques
 */
#define LIMIT_TORQUE

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs outputs of the controller
 */
HandleOutputs::HandleOutputs(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs):
	Computation(inputs, options, ctrl_index)
{
	this->outputs = outputs;
}

/*! \brief destructor
 */
HandleOutputs::~HandleOutputs()
{

}

/*! \brief main computation to handle the outputs
 */
void HandleOutputs::compute()
{
	if (options->is_torque_limit())
	{
		if (inputs->get_t() > 5.0)
		{
			for(int i=0; i<inputs->get_indexes()->get_nb_mot(); i++)
			{
				outputs->set_Qq_ref(i, limit_range(outputs->get_Qq_ref(i), MIN_TORQUE, MAX_TORQUE));
			}
		}
	}
}
