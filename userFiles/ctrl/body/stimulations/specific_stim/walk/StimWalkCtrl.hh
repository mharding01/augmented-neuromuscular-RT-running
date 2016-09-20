/*! 
 * \author Nicolas Van der Noot
 * \file StimWalkCtrl.hh
 * \brief StimWalkCtrl class
 */
#ifndef _STIM_WALK_CTRL_HH_
#define _STIM_WALK_CTRL_HH_

#include "StimulationCtrl.hh"
#include "DelayManager.hh"
#include "MatsuokaSixN.hh"
#include "SwingStanceState.hh"
#include "Articulation.hh"

/*! \brief Controller generating the stimulations sent to the muscles to make the robot walk combining reflexes and feedforward signals
 */
class StimWalkCtrl: public StimulationCtrl
{
	public:
		StimWalkCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimWalkCtrl();

		virtual void compute();

		MatsuokaSixN* get_osc() { return osc; }

		// pitch opti
		void set_S0_vas(double value)      { S0_vas      = value; }
		void set_G_sol(double value)       { G_sol       = value; }
		void set_G_ta_sw(double value)     { G_ta_sw     = value; }
		void set_G_ta_st(double value)     { G_ta_st     = value; }
		void set_G_vas(double value)       { G_vas       = value; }
		void set_l_off_ta_sw(double value) { l_off_ta_sw = value; }
		void set_l_off_ta_st(double value) { l_off_ta_st = value; }
		void set_phi_off_pk(double value)  { phi_off_pk  = value; }
		void set_ksi_1(double value)       { ksi_1       = value; }
		void set_ksi_2(double value)       { ksi_2       = value; }

		// roll opti
		void set_kp_torso(double value)   { kp_torso = value;   }
		void set_kd_torso(double value)   { kd_torso = value;   }
		void set_torso_ref(double value)  { torso_ref = value;  }
		void set_kp_y_com(double value)   { kp_y_com = value;   }
		void set_kd_y_com(double value)   { kd_y_com = value;   }
		void set_y_com_ref(double value)  { y_com_ref = value;  }
		void set_kp_hip(double value)     { kp_hip = value;     }
		void set_kd_hip(double value)     { kd_hip = value;     }
		void set_kp_sp_foot(double value) { kp_sp_foot = value; }
		void set_kd_sp_foot(double value) { kd_sp_foot = value; }
		void set_kp_sw_foot(double value) { kp_sw_foot = value; }
		void set_kd_sw_foot(double value) { kd_sw_foot = value; }

		// yaw opti
		void set_kp_yaw(double value) { kp_yaw = value; }
		void set_kd_yaw(double value) { kd_yaw = value; }
		
	private:
		SwingStanceState *sw_st;     ///< swing-stance state
		DelayManager *delay_manager; ///< delay manager
		MatsuokaSixN *osc;           ///< oscillators
		Articulation *pk[NB_LEGS];   ///< articulation: pitch knee
		Articulation *pa[NB_LEGS];   ///< articulation: pitch foot

		Muscle *sol[NB_LEGS]; ///< SOL muscle
		Muscle *ta[NB_LEGS];  ///< TA muscle
		Muscle *vas[NB_LEGS]; ///< VAS muscle

		Articulation *roll_hip[NB_LEGS];  ///< roll hip articulations
		Articulation *roll_foot[NB_LEGS]; ///< roll foot articulations
		Articulation *yaw_hip[NB_LEGS];   ///< yaw hip articulations

		int supporting_r_leg; ///< 1 if right leg is supporting leg, 0 otherwise
		int flag_3D; ///< 1 for 3D walking, 0 otherwise

		// COM position
		double x_com_foot;  ///< x COM position (relative to the supporting foot) [m]
		double y_com_foot;  ///< y COM position (relative to the supporting foot) [m]
		double xp_com_foot; ///< x COM position derivative (relative to the supporting foot) [m/s]
		double yp_com_foot; ///< y COM position derivative (relative to the supporting foot) [m/s]

		// body orientation angles and derivatives
		double theta_roll_torso; ///< torso absolute roll angle [rad]
		double theta_roll_Rfoot; ///< right foot absolute roll angle [rad]
		double theta_roll_Lfoot; ///< left foot absolute roll angle [rad]

		double omega_roll_torso; ///< torso absolute roll angle derivative [rad/s]
		double omega_roll_Rfoot; ///< right foot absolute roll angle derivative [rad/s]
		double omega_roll_Lfoot; ///< left foot absolute roll angle derivative [rad/s]

		// feet forces (delay)
		double F_foot[NB_LEGS]; ///< feet forces [N]

		// angles (delay)
		double phi_k[NB_LEGS];  ///< knee angle [rad]
		double phip_k[NB_LEGS]; ///< knee angle derivative [rad/s]

		// torso (delay)
		double theta_torso; ///< torso angle [rad]
		double omega_torso; ///< torso angle derivative [rad/s]

		// l.ce (dalay)
		double lce_ta[NB_LEGS]; ///< contractile element derivative for TA [m]
		double lce_vas[NB_LEGS]; ///< contractile element derivative for VAS [m]

		// Fm (dalay)
		double F_sol[NB_LEGS]; ///< force for SOL [N]
		double F_vas[NB_LEGS]; ///< force for VAS [N]

		// fixed parameters
		double F_max_sol; ///< maximal force, SOL muscle [N]
		double F_max_vas; ///< maximal force, VAS muscle [N]
		double l_opt_ta;  ///< l.opt: TA muscle [m]
		double l_opt_vas; ///< l.opt: VAS muscle [m]

		// opti parameters
		double S0_vas;
		
		double G_sol;
		double G_ta_sw;
		double G_ta_st;
		double G_vas;

		double l_off_ta_sw;
		double l_off_ta_st;

		double phi_off_pk;
		
		double ksi_1;
		double ksi_2;

		// roll
		double kp_torso;
		double kd_torso;
		double torso_ref;

		double kp_y_com;
		double kd_y_com;
		double y_com_ref;

		double kp_hip;
		double kd_hip;

		double kp_sp_foot;
		double kd_sp_foot;

		double kp_sw_foot;
		double kd_sw_foot;

		// yaw
		double kp_yaw;
		double kd_yaw;

		// theta ref
		double theta_ref;

		// velocity tracking
		double k_GLU;
		double k_HFL;
		double k_HAM1;
		double k_HAM2;

		// vas extra term
		double l_off_vas_sw;
		double G_vas_sw;

		// functions prototypes
		void com_position();
		void body_orientations();
		void compute_delay();
		void update_oscillators();
		void compute_stimulation();

		void pitch_compute();
		void roll_compute();
		void yaw_compute();

		void pitch_compute_min();
		void roll_compute_min();
		void yaw_compute_min();
};

#endif

