
#include "ComputeManager.hh"
#include "HandleInputs.hh"
#include "HandleOutputs.hh"
#include "WalkStates.hh"
#include "ForwardKinematics.hh"
#include "Body.hh"
#include "UpperBody.hh"
#include "ImpedanceCtrlStanding.hh"
#include "ImpedanceCtrlInit.hh"
#include "ImpedanceCtrlJoystick.hh"
#include <stdlib.h>

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[out] outputs controller outputs
 */
ComputeManager::ComputeManager(CtrlInputs *inputs, CtrlOutputs *outputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index)
{
	int flag_fwd_kin, flag_ws;

	WalkStates *ws;
	ForwardKinematics *fwd_kin;

	std::vector<int> active_mod;

	this->inputs  = inputs;
	this->outputs = outputs;
	this->options = options;
	this->ctrl_index = ctrl_index;

	flag_fwd_kin = 0;
	flag_ws = 0;

	fwd_kin = NULL;
	ws = NULL;

	// computation modules to activate
	active_mod.push_back(WALK_STATES);
	active_mod.push_back(KINEMATICS_COMPUTE);
	active_mod.push_back(HANDLE_INPUTS);
	active_mod.push_back(LOWER_BODY);
	active_mod.push_back(UPPER_BODY);
	active_mod.push_back(IMPEDANCE_CTRL);
	active_mod.push_back(HANDLE_OUTPUTS);

	for(unsigned int i=0; i<active_mod.size(); i++)
	{
		switch (active_mod[i])
		{
			// robot states
			case WALK_STATES:
				ws = new WalkStates(inputs, options, ctrl_index);
				compute_tab.push_back(ws);
				flag_ws = 1;
				break;

			// computes the COM position (and derivative)
			case KINEMATICS_COMPUTE:
				if (!flag_ws)
				{
					std::cout << "Error: WalkStates not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				fwd_kin = new ForwardKinematics(inputs, options, ctrl_index, ws);
				compute_tab.push_back(fwd_kin);
				flag_fwd_kin = 1;
				break;

			// handle the inputs
			case HANDLE_INPUTS:
				if (!flag_fwd_kin)
				{
					std::cout << "Error: ForwardKinematics not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new HandleInputs(inputs, options, ctrl_index, fwd_kin));
				break;

			// lower body
			case LOWER_BODY:
				if (!flag_ws)
				{
					std::cout << "Error: WalkStates not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				if (!flag_fwd_kin)
				{
					std::cout << "Error: ForwardKinematics not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new Body(inputs, options, ctrl_index, outputs, ws, fwd_kin));
				break;

			// upper body for POSITION_TRACKING
			case UPPER_BODY:
				if (!flag_ws)
				{
					std::cout << "Error: WalkStates not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new UpperBody(inputs, options, ctrl_index, outputs, ws));
				break;

			// impedance controller
			case IMPEDANCE_CTRL:
				if (!flag_fwd_kin)
				{
					std::cout << "Error: ForwardKinematics not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new ImpedanceCtrlInit(inputs, options, ctrl_index, outputs, fwd_kin));
				break;

			// handle the outputs
			case HANDLE_OUTPUTS:
				compute_tab.push_back(new HandleOutputs(inputs, options, ctrl_index, outputs));
				break;

			default:
				std::cout << "Error: unknown computation model !" << std::endl;
				exit(EXIT_FAILURE);
				break;
		}
	}
}

/*! \brief destructor
 */
ComputeManager::~ComputeManager()
{
	for(unsigned int i=0; i<compute_tab.size(); i++)
	{
		delete compute_tab[i];
	}
}

/*! \brief run main controller modules
 */
void ComputeManager::run_computations()
{
	for(unsigned int i=0; i<compute_tab.size(); i++)
	{
		compute_tab[i]->compute();
	}
}
