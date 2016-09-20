/*! 
 * \author Nicolas Van der Noot
 * \file OptiInputs.hh
 * \brief OptiInputs class
 */

#ifndef _OPTI_INPUTS_HH_
#define _OPTI_INPUTS_HH_

// forward declaration
class ImpedanceCtrlInit;
class StimulationCtrl;
class StimInit;
class Oscillators;
class SwingStanceState;
class JointsInit;
class Body;

/*! \brief Inputs of the optimization
 */
class OptiInputs
{
	public:
		OptiInputs();
		virtual ~OptiInputs();

		virtual void set_opti() = 0;

		void set_opti_init();
		
		void set_impedance_init(ImpedanceCtrlInit *impedance_init) { this->impedance_init = impedance_init; }
		void set_stim_ctrl(StimulationCtrl *stim_ctrl) { this->stim_ctrl = stim_ctrl; }
		void set_stim_init(StimInit *stim_init) { this->stim_init = stim_init; }
		void set_osc(Oscillators *osc) { this->osc = osc; }
		void set_sw_st(SwingStanceState *sw_st) { this->sw_st = sw_st; }
		void set_joints_init(JointsInit *joints_init) { this->joints_init = joints_init; }
		void set_body(Body *body) { this->body = body; }

	protected:
		ImpedanceCtrlInit *impedance_init; ///< impedance controller for initialization

		StimulationCtrl *stim_ctrl; ///< computation of the muscles stimulations class
		StimInit *stim_init;        ///< computation of the initial muscles simulations class

		Oscillators *osc; ///< oscillators

		SwingStanceState *sw_st; ///< swing-stance states

		JointsInit *joints_init; ///< initial positions and velocities for some joints

		Body *body; ///< to apply wang torque (from files)
};

#endif
