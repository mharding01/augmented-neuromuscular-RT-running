/*! 
 * \author Nicolas Van der Noot
 * \file Muscle.hh
 * \brief Muscle class
 */

#ifndef _MUSCLE_HH_
#define _MUSCLE_HH_

#include "Articulation.hh"
#include "Noise.hh"
#include "LowFilter.hh"
#include "body_parts.hh"
#include <cmath>

class StimulationCtrl;

#define L_MTU_INTERP_SIZE 5
#define A_INTERP_SIZE 5

#define S_MIN 0.01
#define S_MAX 1.0

#define L_CE_FINITE_NB 3

// leg muscles
enum{SOL_MUSCLE, TA_MUSCLE, GAS_MUSCLE, VAS_MUSCLE, HAM_MUSCLE, GLU_MUSCLE, HFL_MUSCLE, RF_MUSCLE,
	 HAB_MUSCLE, HAD_MUSCLE, HER_MUSCLE, HIR_MUSCLE, EVE_MUSCLE, INV_MUSCLE, NB_LEG_MUSCLES};

// torso muscles
enum{BFL_MUSCLE, BET_MUSCLE, BTR_MUSCLE, BTL_MUSCLE, BRR_MUSCLE, BRL_MUSCLE, NB_TORSO_MUSCLES};

// arm muscles
enum{EFL_MUSCLE, EET_MUSCLE, SFL_MUSCLE, SET_MUSCLE, 
	 SAB_MUSCLE, SAD_MUSCLE, SER_MUSCLE, SIR_MUSCLE, NB_ARM_MUSCLES};

// lengths of the muscles
typedef struct MuscleLengths
{
	double mtu; ///< total length of the MTU (muscle tendon unit) [m]
	double ce;  ///< length of the contractile element [m]
	double se;  ///< length of the series element [m]

} MuscleLengths;

// speeds of the contractile element
typedef struct MuscleSpeeds
{
	double ce; ///< speed of the contractile element [m/s]
	double ce_norm; ///< normalized speed of the contractile element [1/s]

} MuscleSpeeds;

// force-velocity & force-length relationships
typedef struct MuscleForceRelations
{
	double v; ///< force-velocity relation
	double l; ///< force-length relation

} MuscleForceRelations;

// muscles forces
typedef struct MuscleForces
{
	double se;      ///< force produced by the series elastic element [N]
	double be;      ///< force produced by the BE element [N]
	double pe_star; ///< force produced by the parallel element, scaled [-]
	double ce;      ///< force produced by the contractile element [N]
	double m;       ///< force produced by the muscle [N]

} MuscleForces;

// muscles properties
typedef struct MuscleProperties
{
	double l_opt;   ///< length l_opt of the muscle [m]
	double l_slack; ///< length l_slack of the muscle [m]
	double l_min;   ///< length min of the muscle [m]
	double F_max;   ///< force max of the muscle [N]
	double v_max;   ///< speed max of the muscle [1/s]
	double m_mtu;   ///< mass of the muscle [kg]
	double t_I;     ///< t_I of the muscle []

} MuscleProperties;

// inverse model constants
typedef struct MuscleInverseCst
{
	double K1; ///< constant 1
	double K2; ///< constant 2
	double K3; ///< constant 3
	double K4; ///< constant 4
	double K5; ///< constant 5
	double K6; ///< constant 6
	double K7; ///< constant 7
	
} MuscleInverseCst;


/*! \brief Muscle class
 */
class Muscle
{
	public:
		Muscle(CtrlInputs *inputs, int muscle_id, int body_part_id, double l_opt, double l_slack, double F_max, double v_max, double m_mtu, double t_I);
		virtual ~Muscle();
		
		int get_id() { return muscle_id; }

		double get_S()          const { return S;          }
		double get_lce()        const { return l.ce;       }
		double get_vce()        const { return v.ce;       }
		double get_Fm()         const { return F.m;        }
		double get_Fmax()       const { return p.F_max;    }
		double get_lopt()       const { return p.l_opt;    }
		double get_met_energy() const { return met_energy; }

		void compute(StimulationCtrl *stim_ctrl);

		virtual void rm_compute()      = 0;
		virtual void lmtu_compute()    = 0;
		virtual void torques_compute() = 0;

		int compute_F_min(double &l_ce_star, double &F_min);
		int l_ce_steady_state(double &l_ce_star, double cur_l_mtu, double cur_A);
		void l_ce_Newton_Raphson(double &l_ce_star, double cur_l_mtu, double cur_A);

		double l_ce_interp(double A, double l_mtu, double A_vec[A_INTERP_SIZE], double l_mtu_vec[L_MTU_INTERP_SIZE], double tab_A_l_mtu[A_INTERP_SIZE][L_MTU_INTERP_SIZE]);
		
		double inverse_compute(double Fm_des);


	protected:
		CtrlInputs *inputs; ///< controller inputs

		int muscle_noise; ///< 1 to add noise on the muscle stimulations, 0 otherwise
		int ctrl_two_parts; ///< 1 for ctrl in two parts : first with know results and after with opti's results, 0 otherwise

		int body_part_id; ///< body part ID
		int muscle_id; ///< muscle ID

		MuscleLengths l;        ///< lengths of the muscles
		MuscleSpeeds v;         ///< speeds of the muscles
		MuscleForceRelations f; ///< force-velocity & force-length relationships
		MuscleForces F;         ///< muscles forces
		MuscleProperties p;     ///< muscles properties
		MuscleInverseCst c;     ///< muscles inverse constants
		
		double A; ///< activation [-]
		double S; ///< stimulation [-]

		Noise *n; ///< noise [-]

		LowFilter *A_filter; ///< low pass filter to get the activation

		double last_t;     ///< last time muscle updated [s]
		double last_t_inv; ///< last time inverse muscle updated [s]

		double met_energy; ///< metabolic energy [J]

		double prev_lce[L_CE_FINITE_NB]; ///< previous l_ce values for finite difference

		double coef_diff_1; ///< first coefficient for the finite difference order 3
		double coef_diff_2; ///< second coefficient for the finite difference order 3
		double coef_diff_3; ///< third coefficient for the finite difference order 3
		double coef_diff_4; ///< fourth coefficient for the finite difference order 3

		// function prototypes
		void forces_compute(double cur_l_ce);
		void fl_compute(double cur_l_ce);
		void fv_compute();
		void v_ce_compute(double &cur_v_ce);
		void derivative(double cur_l_ce, double &cur_v_ce);

		double metabolic_energy_dot();
		double g_l_ce(double l_ce_tilde);
		double fl_compute_inv(double l_ce);

		void euler_iterations(double diff_t, int nb_iter);
		void runge_kutta(double diff_t);
};

#endif
