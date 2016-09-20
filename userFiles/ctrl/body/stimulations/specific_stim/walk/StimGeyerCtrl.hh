/*! 
 * \author Nicolas Van der Noot
 * \file StimGeyerCtrl.hh
 * \brief StimGeyerCtrl class
 */

#ifndef _STIM_GEYER_CTRL_HH_
#define _STIM_GEYER_CTRL_HH_

#include "StimulationCtrl.hh"
#include "DelayManager.hh"
#include "SwingStanceState.hh"
#include "TrailingState.hh"

/*! \brief Controller generating the stimulations sent to the muscles computed with Geyer controller
 */
class StimGeyerCtrl: public StimulationCtrl
{
	public:
		StimGeyerCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimGeyerCtrl();

		virtual void compute();

		void compute_delay();
		void compute_stimulation();

		void set_S0_vas(double value)      { S0_vas      = value; }
		void set_G_sol(double value)       { G_sol       = value; }
		void set_G_ta_sw(double value)     { G_ta_sw     = value; }
		void set_G_ta_st(double value)     { G_ta_st     = value; }
		void set_G_vas(double value)       { G_vas       = value; }
		void set_G_ham_hfl(double value)   { G_ham_hfl   = value; }
		void set_phi_off_pk(double value)  { phi_off_pk  = value; }
		void set_l_off_ta_sw(double value) { l_off_ta_sw = value; }
		void set_l_off_ta_st(double value) { l_off_ta_st = value; }
		void set_l_off_ham(double value)   { l_off_ham   = value; }
		void set_k_bw(double value)        { k_bw        = value; }
		void set_k_p(double value)         { k_p         = value; }
		void set_k_d(double value)         { k_d         = value; }
		void set_theta_ref(double value)   { theta_ref   = value; }
		void set_S_ham_tr(double value)    { S_ham_tr    = value; }

	private:
		SwingStanceState *sw_st; ///< swing-stance state
		TrailingState *tr_st;    ///< trailing state

		DelayManager *delay_manager; ///< delay manager
		Articulation *pk[NB_LEGS];   ///< articulation: pitch knee
		Articulation *pa[NB_LEGS];   ///< articulation: pitch foot

		Muscle *sol[NB_LEGS]; ///< SOL muscle
		Muscle *ta[NB_LEGS];  ///< TA muscle
		Muscle *vas[NB_LEGS]; ///< VAS muscle
		Muscle *ham[NB_LEGS]; ///< HAM muscle
		Muscle *hfl[NB_LEGS]; ///< HFL muscle

		int flag_3D; ///< 1 for 3D walking, 0 otherwise

		// feet forces (delay)
		double F_foot[NB_LEGS]; ///< feet forces [N]

		// angles (delay)
		double phi_k[NB_LEGS];  ///< knee angle [rad]
		double phip_k[NB_LEGS]; ///< knee angle derivative [rad/s]

		// torso (delay)
		double theta_torso; ///< torso angle [rad]
		double omega_torso; ///< torso angle derivative [rad/s]

		// l.ce (dalay)
		double lce_ta[NB_LEGS];  ///< contractile element derivative for TA [m]
		double lce_vas[NB_LEGS]; ///< contractile element derivative for VAS [m]
		double lce_ham[NB_LEGS]; ///< contractile element derivative for HAM [m]
		double lce_hfl[NB_LEGS]; ///< contractile element derivative for HFL [m]

		// Fm (dalay)
		double F_sol[NB_LEGS]; ///< force for SOL [N]
		double F_vas[NB_LEGS]; ///< force for VAS [N]

		// fixed parameters
		double F_max_sol; ///< maximal force, SOL muscle [N]
		double F_max_vas; ///< maximal force, VAS muscle [N]
		double l_opt_ta;  ///< l.opt: TA muscle [m]
		double l_opt_vas; ///< l.opt: VAS muscle [m]
		double l_opt_ham; ///< l.opt: HAM muscle [m]
		double l_opt_hfl; ///< l.opt: HFL muscle [m]

		// opti parameters
		double S0_vas;
		
		double G_sol;
		double G_ta_sw;
		double G_ta_st;
		double G_vas;
		double G_ham_hfl;

		double phi_off_pk;
		double l_off_ta_sw;
		double l_off_ta_st;
		double l_off_ham;

		double k_bw;
		double k_p;
		double k_d;

		double theta_ref;

		// vas extra term
		double l_off_vas_sw;
		double G_vas_sw;

		double l_off_ham_sw;
		double G_ham_sw;

		// ham extra term
		double S_ham_tr;

		void pitch_compute();

		void pitch_compute_min();
		void roll_compute_min();
		void yaw_compute_min();
};

#endif
