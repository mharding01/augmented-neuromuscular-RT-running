/*! 
 * \author Nicolas Van der Noot
 * \file StimWangCtrl.hh
 * \brief StimWangCtrl class
 */

#ifndef _STIM_WANG_CTRL_HH_
#define _STIM_WANG_CTRL_HH_

#include "StimulationCtrl.hh"
#include "DelayManager.hh"
#include "SwingStanceState.hh"
#include "TrailingState.hh"
#include "OptiInit.hh"

/*! \brief Controller generating the stimulations sent to the muscles computed with Wang controller
 */
class StimWangCtrl: public StimulationCtrl
{
	public:
		StimWangCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimWangCtrl();

		virtual void compute();

		void compute_delay();
		void compute_stimulation();
		void switch_results();

		void set_S0_sol_st(double value)   { S0_sol_st   = value; }
		void set_S0_ta_st(double value)    { S0_ta_st    = value; }
		void set_S0_gas_st(double value)   { S0_gas_st   = value; }
		void set_S0_vas_st(double value)   { S0_vas_st   = value; }
		void set_S0_ham_st(double value)   { S0_ham_st   = value; }
		void set_S0_rf_st(double value)    { S0_rf_st    = value; }
		void set_S0_glu_st(double value)   { S0_glu_st   = value; }
		void set_S0_hfl_st(double value)   { S0_hfl_st   = value; }
		void set_S0_sol_sw(double value)   { S0_sol_sw   = value; }
		void set_S0_ta_sw(double value)    { S0_ta_sw    = value; }
		void set_S0_gas_sw(double value)   { S0_gas_sw   = value; }
		void set_S0_vas_sw(double value)   { S0_vas_sw   = value; }
		void set_S0_ham_sw(double value)   { S0_ham_sw   = value; }
		void set_S0_rf_sw(double value)    { S0_rf_sw    = value; }
		void set_S0_glu_sw(double value)   { S0_glu_sw   = value; }
		void set_S0_hfl_sw(double value)   { S0_hfl_sw   = value; }

		void set_G_sol(double value)       { G_sol       = value; }
		void set_G_sol_ta(double value)    { G_sol_ta    = value; }
		void set_G_gas(double value)       { G_gas       = value; }
		void set_G_vas(double value)       { G_vas       = value; }
		void set_G_ham(double value)       { G_ham       = value; }
		void set_G_glu(double value)       { G_glu       = value; }

		void set_G_ta_sw(double value)     { G_ta_sw     = value; }
		void set_G_ta_st(double value)     { G_ta_st     = value; }
		void set_G_hfl(double value)       { G_hfl       = value; }
		void set_G_ham_hfl(double value)   { G_ham_hfl   = value; }
		
		void set_l_off_ta_sw(double value) { l_off_ta_sw = value; }
		void set_l_off_ta_st(double value) { l_off_ta_st = value; }
		void set_l_off_ham_hfl(double value) { l_off_ham_hfl = value; }
		void set_l_off_hfl(double value)   { l_off_hfl   = value; }

		void set_K_ham(double value)       { K_ham       = value; }
		void set_K_glu(double value)       { K_glu       = value; }
		void set_K_hfl(double value)       { K_hfl       = value; }
		void set_D_ham(double value)       { D_ham       = value; }
		void set_D_glu(double value)       { D_glu       = value; }
		void set_D_hfl(double value)       { D_hfl       = value; }
		void set_theta_ref(double value)   { theta_ref   = value; }

		void set_si_vas(double value)      { si_vas      = value; }
		void set_si_rf(double value)       { si_rf       = value; }
		void set_si_glu(double value)      { si_glu      = value; }
		void set_si_hfl(double value)      { si_hfl      = value; }

		void set_K_sp_vas(double value)    { K_sp_vas    = value; }
		void set_K_sp_glu(double value)    { K_sp_glu    = value; }
		void set_K_sp_hfl(double value)    { K_sp_hfl    = value; }
		void set_D_sp_vas(double value)    { D_sp_vas    = value; }
		void set_D_sp_glu(double value)    { D_sp_glu    = value; }
		void set_D_sp_hfl(double value)    { D_sp_hfl    = value; }
		void set_theta_k_ref(double value) { theta_k_ref = value; }

		void set_theta_h_ref0(double value){ theta_h_ref0= value; }
		void set_c_d(double value)         { c_d         = value; }
		void set_c_v(double value)         { c_v         = value; }

		void set_d_sp(double value)        { d_sp        = value; }
		void set_d_si(double value)        { d_si        = value; }

		void set_k_THETA(double value)     { k_THETA     = value; }
		void set_k_theta(double value)     { k_theta     = value; }
		void set_phi_off_pk(double value)  { phi_off_pk  = value; }

		void set_t_switch(double value)    { t_switch    = value; }

	private:
		OptiInit *opti_init; ///< initial optimization parameters

		SwingStanceState *sw_st; ///< swing-stance state
		TrailingState *tr_st;    ///< trailing state

		DelayManager *delay_manager; ///< delay manager
		Articulation *pk[NB_LEGS];   ///< articulation: pitch knee
		Articulation *ph[NB_LEGS];   ///< articulation: pitch hip

		Muscle *sol[NB_LEGS]; ///< SOL muscle
		Muscle *ta[NB_LEGS];  ///< TA muscle
		Muscle *vas[NB_LEGS]; ///< VAS muscle
		Muscle *ham[NB_LEGS]; ///< HAM muscle
		Muscle *hfl[NB_LEGS]; ///< HFL muscle
		Muscle *gas[NB_LEGS]; ///< GAS muscle
		Muscle *glu[NB_LEGS]; ///< GLU muscle

		int flag_3D; ///< 1 for 3D walking, 0 otherwise
		int ctrl_two_parts; ///< 1 for ctrl in two parts : first with know results and after with opti's results, 0 otherwise
		int flag_part1;
		int flag_part2;
		int Qq_match_wang;
		int inital_pos;

		// angles (delay)
		double phi_k[NB_LEGS];  ///< knee angle [rad]
		double phip_k[NB_LEGS]; ///< knee angle derivative [rad/s]
		double phi_h[NB_LEGS];  ///< hip angle [rad]
		double phip_h[NB_LEGS]; ///< hip angle derivative [rad/s]

		// torso (delay)
		double theta_torso; ///< torso angle [rad]
		double omega_torso; ///< torso angle derivative [rad/s]

		double theta_toro_sw0; ///< torso angle at the beginning of the swing phase [rad]
		int first_swing[NB_LEGS]; ///< flag to enter only one in first swing contact 

		// l.ce (dalay)
		double lce_ta[NB_LEGS];  ///< contractile element derivative for TA [m]
		double lce_ham[NB_LEGS]; ///< contractile element derivative for HAM [m]
		double lce_hfl[NB_LEGS]; ///< contractile element derivative for HFL [m]

		// Fm (dalay)
		double F_sol[NB_LEGS]; ///< force for SOL [N]
		double F_vas[NB_LEGS]; ///< force for VAS [N]
		double F_gas[NB_LEGS]; ///< force for GAS [N]
		double F_ham[NB_LEGS]; ///< force for HAM [N]
		double F_glu[NB_LEGS]; ///< force for GLU [N]

		// fixed parameters
		double F_max_sol; ///< maximal force, SOL muscle [N]
		double F_max_vas; ///< maximal force, VAS muscle [N]
		double F_max_gas; ///< maximal force, GAS muscle [N]
		double F_max_ham; ///< maximal force, HAM muscle [N]
		double F_max_glu; ///< maximal force, GLU muscle [N]
		double l_opt_ta;  ///< l.opt: TA muscle [m]
		double l_opt_ham; ///< l.opt: HAM muscle [m]
		double l_opt_hfl; ///< l.opt: HFL muscle [m]

		// --- opti parameters ---

    	// pre-stimulations
    	double S0_sol_st;
    	double S0_ta_st;
    	double S0_gas_st;
    	double S0_vas_st;
    	double S0_ham_st;
    	double S0_rf_st;
    	double S0_glu_st;
    	double S0_hfl_st;
    	double S0_sol_sw;
    	double S0_ta_sw;
    	double S0_gas_sw;
    	double S0_vas_sw;
    	double S0_ham_sw;
    	double S0_rf_sw;
    	double S0_glu_sw;
    	double S0_hfl_sw;
    	// gains of positive force feedback laws
    	double G_sol;
    	double G_sol_ta;
    	double G_gas;
    	double G_vas;
    	double G_ham;
    	double G_glu;
    	// gains of positive length feedback laws
    	double G_ta_sw;
    	double G_ta_st;
    	double G_hfl;
    	double G_ham_hfl;
		// offsets of positive length feedback laws
    	double l_off_ta_sw;
    	double l_off_ta_st;
    	double l_off_hfl;
    	double l_off_ham_hfl;
    	// stance phase PD-control parameters
    	double K_ham;
    	double K_glu;
    	double K_hfl;
    	double D_ham;
    	double D_glu;
    	double D_hfl;
    	double theta_ref;
    	// swing initiation parameters
    	double si_rf;
    	double si_vas;
    	double si_glu;
    	double si_hfl;
    	// stance preparation muscle PD-control parameters
    	double K_sp_vas;
		double K_sp_glu;
		double K_sp_hfl;
		double D_sp_vas;
		double D_sp_glu;
		double D_sp_hfl;
		double theta_k_ref;
		// double stance preparation SIMBICON-style feedback parameters
		double theta_h_ref0;
		double c_d;
		double c_v;
		// double swing initiation and stance preparation offsets
    	double d_si;
    	double d_sp;
    	//additional parameters
    	double k_THETA;
    	double k_theta;
    	double phi_off_pk;
    	//switch controller
    	double t_switch;

		void pitch_compute();

		void pitch_compute_min();
		void roll_compute_min();
		void yaw_compute_min();

		int stance_preparation(int swing_leg_id);
		int swing_initiation(int stance_leg_id);
};

#endif
