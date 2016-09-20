/*! 
 * \author Nicolas Van der Noot
 * \file Ctrl.hh
 * \brief Ctrl class
 */

#ifndef _CTROLLER_HH_
#define _CTROLLER_HH_	

#include "controller_io.hh"
#include "UserCtrl.hh"
#include <iostream>

#ifdef ROBOTRAN_SIMU
	#include "user_realtime.h"
#endif

enum{BASIC_CTRL, EXAMPLE_CTRL, NICO_CTRL};

/*! \brief Main controller
 */
class Ctrl
{
	public:
		Ctrl(int ctrl_id);
		virtual ~Ctrl();

		virtual void init_ctrl()   = 0;
		virtual void loop_ctrl()   = 0;
		virtual void finish_ctrl() = 0;

		virtual void set_inputs(Inputs_ctrl *inputsCtrl)    = 0;
		virtual void set_outputs(Outputs_ctrl *outputsCtrl) = 0;

		/// get ctrl_id
		inline int get_ctrl_id() const { return ctrl_id; }

		/// get user_ctrl
		inline UserCtrl* get_user_ctrl() const { return user_ctrl; }

	protected:
		int ctrl_id;         ///< controller ID
		UserCtrl *user_ctrl; ///< user control structure
};

#endif
