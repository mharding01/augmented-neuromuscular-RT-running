/*! 
 * \author Nicolas Van der Noot
 * \file ImpedanceCtrlStanding.hh
 * \brief ImpedanceCtrlStanding class
 */

#ifndef _IMPEDANCE_CTRL_STANDING_HH_
#define _IMPEDANCE_CTRL_STANDING_HH_

#include "ImpedanceCtrl.hh"

class ImpedanceCtrlStanding: public ImpedanceCtrl
{
	public:
		ImpedanceCtrlStanding(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin);
		virtual ~ImpedanceCtrlStanding();

	private:
		int nb_cont_foot; ///< number of contacts per foot

		// function prototypes
		virtual void update_matrices();
		virtual void compute_base();
		virtual void compute_jacob();
};

#endif
