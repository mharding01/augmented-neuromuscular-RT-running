/*! 
 * \author Nicolas Van der Noot
 * \file NicoCtrl.hh
 * \brief NicoCtrl class
 */

#ifndef _NICO_CTRL_HH_
#define _NICO_CTRL_HH_	

#include "Ctrl.hh"

#include "CtrlInputs.hh"
#include "CtrlOutputs.hh"
#include "ComputeManager.hh"
#include "MotorCtrlIndex.hh"

/*! \brief Main Controller designed by Nico
 */
class NicoCtrl: public Ctrl
{
	public:
		NicoCtrl();
		virtual ~NicoCtrl();

		virtual void init_ctrl();
		virtual void loop_ctrl();
		virtual void finish_ctrl();

		virtual void set_inputs(Inputs_ctrl *inputsCtrl);
		virtual void set_outputs(Outputs_ctrl *outputsCtrl);

		CtrlInputs* get_inputs()      const { return inputs;  }
		CtrlOutputs* get_outputs()    const { return outputs; }
		ComputeManager* get_manager() const { return manager; }
		MotorCtrlIndex* get_motor_index() const { return motor_index; }

		CtrlOptions* get_options() const { return options; }

	private:
		CtrlInputs *inputs;      ///< controller inputs
		CtrlOutputs *outputs;    ///< controller outputs
		ComputeManager *manager; ///< manager for main loop

		CtrlOptions *options; ///< options for the controller

		MotorCtrlIndex *motor_index; ///< indexes for implemented motors controller

		int get_ctrl_index(int coman_index);
		int get_coman_index(int ctrl_index);
};

#endif
