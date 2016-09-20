/*! 
 * \author Nicolas Van der Noot
 * \file SwingStanceState.hh
 * \brief SwingStanceState class
 */

#ifndef _SWING_STATE_STATE_HH_
#define _SWING_STATE_STATE_HH_

#include "WalkState.hh"
#include "coman_properties.hh"
#include "MainState.hh"

/*! \brief state of each leg as swing or stance
 */
class SwingStanceState: public WalkState
{
	public:
		SwingStanceState(CtrlInputs *inputs, CtrlOptions *options, MainState *main_state);
		virtual ~SwingStanceState();

		virtual void compute();

		int is_swing_leg(int i) const { return swing_leg[i]; }
		int get_nb_strikes() const { return nb_strikes; }

		int is_double_support() const {return (!is_swing_leg(R_ID) && !is_swing_leg(L_ID)); }
		int is_flight_phase() const {return (is_swing_leg(R_ID) && is_swing_leg(L_ID)); }

		int get_flag_strike() const { return flag_strike; }
		int get_flag_strike_leg(int i) { return flag_strike_leg[i]; }
		int is_supporting_r_leg() const { return supporting_r_leg; }

		double get_t_start_last_stance() const { return t_start_last_stance; }

		void set_per_swing_wang(double value)   { per_swing_wang = value; }

	private:
		int Qq_wang;
		int initial_pos;
		double per_swing_wang; ///< percent of cycle for transition stance/swing for Wang data [s]

		int nb_strikes; ///< number of feet strikes

		int supporting_r_leg; ///< 1 if right leg is supporting leg, 0 otherwise

		int flag_strike;              ///< 1 just after strike (one iteration), 0 after
		int flag_strike_leg[NB_LEGS]; ///< 1 after strike, for each leg (one iteration), 0 after

		int swing_leg[NB_LEGS]; ///< 1 if corresponding leg in swing, 0 otherwise

		double t; ///< time [s]
		double foot_threshold; ///< foot force threshold to detect possible swing
		double toe_threshold; ///< toe force threshold to detect possible swing

		double t_start_last_stance; ///< starting time for last stance [s]
		double t_start_last_swing;  ///< starting time for last swing [s]

		double t_start_last_stance_leg[NB_LEGS]; ///< starting time for last stance (for each leg) [s]
		double t_start_last_swing_leg[NB_LEGS];  ///< starting time for last swing (for each leg) [s]

		double last_t_stance[NB_LEGS]; ///< last time leg was in stance (for each leg) [s]

		double safety_time_walk; ///< safety on the time for last stance [s]

		MainState *main_state; ///< main state of the COMAN

		// private functions
		int possible_swing(int leg_id);
};

#endif
