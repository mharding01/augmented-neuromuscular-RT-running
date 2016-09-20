#include "CtrlInterface.hh"

#include "ExampleCtrl.hh"
#include "NicoCtrl.hh"

#include "CtrlOptions.hh"
#include "CtrlInputs.hh"

#include "OptiClass.hh"
#include "OptiGeneration.hh"
#include "user_IO.h"

CtrlInterface::CtrlInterface(MbsData *mbs_data): CppInterface(mbs_data)
{
	// variables declaration
	NicoCtrl *nico_ctrl;
	OptiGeneration *opti_gen;
	OptiClass *optiClass;

	CtrlOptions *options;
	CtrlInputs *inputs;

	// choose your controller here (currently available: 'ExampleCtrl' and 'NicoCtrl')
	ctrl = new NicoCtrl();

	nico_ctrl = static_cast<NicoCtrl*>(ctrl);
	options = static_cast<CtrlOptions*>(nico_ctrl->get_options());
	inputs = static_cast<CtrlInputs*>(nico_ctrl->get_inputs());

	//inital conditions for joints
	if (options->is_initial_pos())
	{
		joints_init = new JointsInit(mbs_data,inputs);
	}

	// erase the optimization parameters with the new ones
	if (ctrl->get_ctrl_id() == NICO_CTRL)
	{
		if (options->is_opti())
		{
			opti_gen = static_cast<OptiGeneration*>(inputs->get_opti_inputs());
			optiClass = static_cast<OptiClass*>(mbs_data->user_IO->optiClass);
			opti_gen->set_optiParams(optiClass->get_param());
			opti_gen->set_opti(); //to assign t_switch
		}
		else
		{
			inputs->get_opti_inputs()->set_opti(); //use OPtiResults()
		}
	}

	//apply initial condition
	if (options->is_initial_pos())
	{
		joints_init->set_joints_init();
	}

	// simu-ctrl gestion
	simu_ctrl = new SimuCtrl(mbs_data, ctrl);
}

CtrlInterface::~CtrlInterface()
{
	// memory release already done in mother class (CppInterface) fot ctrl and simu_ctrl
	
	NicoCtrl *nico_ctrl;
	CtrlOptions *options;

	nico_ctrl = static_cast<NicoCtrl*>(ctrl);
	options = static_cast<CtrlOptions*>(nico_ctrl->get_options());

	if (options->is_initial_pos())
	{
		delete joints_init;
	}
}
