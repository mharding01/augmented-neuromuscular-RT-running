/*! 
 * \author Nicolas Van der Noot
 * \file MainState.hh
 * \brief MainState class
 */

#ifndef _MAIN_STATE_HH_
#define _MAIN_STATE_HH_

#include "WalkState.hh"

enum{TEST_STATE, INIT_UPRIGHT_STATE, WALK_COMAN_STATE};

/*! \brief main state of the COMAN during its walk
 */
class MainState: public WalkState
{
	public:
		MainState(CtrlInputs *inputs, CtrlOptions *options);
		virtual ~MainState();

		virtual void compute();

		void set_coman_state(int state) { coman_state = state; }

		int get_coman_state() const { return coman_state; }

		double get_init_t_walk() const { return init_t_walk; }

	private:
		double t; ///< time [s]

		int coman_state; ///< state of the COMAN

		double t_walk_start;   ///< time to trigger walk initiation
		double init_t_walk;    ///< time where walk initiation was applied
};

#endif
