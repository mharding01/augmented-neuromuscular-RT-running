/*! 
 * \author Nicolas Van der Noot
 * \file Body.hh
 * \brief Body class
 */

#ifndef _LOWER_BODY_HH_
#define _LOWER_BODY_HH_

#include "Computation.hh"
#include "CtrlInputs.hh"
#include "CtrlOutputs.hh"
#include "BodyPart.hh"
#include "StimulationCtrl.hh"
#include "StimWalkCtrl.hh" // TODO: for cpg ghost controller
#include "StimInit.hh"
#include "ForwardKinematics.hh"
#include "WalkStates.hh"
#include "MainState.hh"
#include "StimUpperPos.hh"

/*! \brief Lower body computations
 */
class Body: public Computation
{
	public:
		Body(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, WalkStates *ws, ForwardKinematics *fwd_kin);
		virtual ~Body();

		virtual void compute();
		
		void send_references();

		/// get stim_ctrl
		StimulationCtrl* get_stim_ctrl() { return stim_ctrl; }

		/// get met_energy_total
		double get_met_energy_total() const { return met_energy_total; }

		/// get met_energy_legs
		double get_met_energy_legs() const { return met_energy_legs; }

		WalkStates* get_walk_state() { return ws; }

		void set_arh(double value) { arh = value; }
		void set_ark(double value) { ark = value; }
		void set_ara(double value) { ara = value; }
		void set_crh(double value) { arh = value; }
		void set_crk(double value) { ark = value; }
		void set_cra(double value) { ara = value; }
		void set_brh(double value) { brh = value; }
		void set_brk(double value) { brk = value; }
		void set_bra(double value) { bra = value; }
		void set_alh(double value) { alh = value; }
		void set_alk(double value) { alk = value; }
		void set_ala(double value) { ala = value; }
		void set_clh(double value) { alh = value; }
		void set_clk(double value) { alk = value; }
		void set_cla(double value) { ala = value; }
		void set_blh(double value) { blh = value; }
		void set_blk(double value) { blk = value; }
		void set_bla(double value) { bla = value; }

	private:
		CtrlOutputs *outputs; ///< controller outputs

		WalkStates *ws;  ///< walk states
		MainState *m_st; ///< main state
		ForwardKinematics *fwd_kin; ///< forward kinematics

		BodyPart* parts[NB_BODY_PARTS]; ///< body parts
		
		StimulationCtrl *stim_init; ///< stimulations to initialize the motion
		StimulationCtrl *stim_ctrl; ///< main stimulations (for the muscles) controller during walking
		StimulationCtrl *stim_upper; ///< stimulations for the upper body
		StimWalkCtrl *ghost_stim_cpg; ///< stimulations from cpg (ghost controller for testing)

		double met_energy_total; ///< total metabolic enrgy
		double met_energy_legs; ///< energy for the legs

		int flag_ctrl; ///< flag indicating the requested controller
		int flag_apply_Qq_wang;

		double t; ///< time [s]

		// i/o vector IDs
		int RightHipPitch_id;  ///< right hip pitch joint
		int RightHipRoll_id;   ///< right hip roll joint
		int RightHipYaw_id;    ///< right hip yaw joint
		int RightKneePitch_id; ///< right knee pitch joint
		int RightFootRoll_id;  ///< right foot roll joint
		int RightFootPitch_id; ///< right foot pitch joint

		int LeftHipPitch_id;  ///< left hip pitch joint
		int LeftHipRoll_id;   ///< left hip roll joint
		int LeftHipYaw_id;    ///< left hip yaw joint
		int LeftKneePitch_id; ///< left knee pitch joint
		int LeftFootRoll_id;  ///< left foot roll joint
		int LeftFootPitch_id; ///< left foot pitch joint

		int RightShPitch_id;  ///< right shoulder pitch joint
		int RightShRoll_id;   ///< right shoulder roll joint
		int RightShYaw_id;    ///< right shoulder yaw joint
		int RightElbPitch_id; ///< right elbow pitch joint

		int LeftShPitch_id;   ///< left shoulder pitch joint
		int LeftShRoll_id;    ///< left shoulder roll joint
		int LeftShYaw_id;     ///< left shoulder yaw joint
		int LeftElbPitch_id;  ///< left elbow pitch joint

		int TorsoRoll_id;	  ///< torso roll joint
		int TorsoPitch_id;	  ///< torso pitch joint
		int TorsoYaw_id;	  ///< torso yaw joint

		std::vector<double> RHipQq; ///< Wang torque data for hip (2 cycles)
		std::vector<double> RKneeQq; ///< Wang torque data for knee (2 cycles)
		std::vector<double> RAnkleQq; ///< Wang torque data for ankle (2 cycles)

		double arh; ///< scaling of Wang torque for right hip
		double ark; ///< scaling of Wang torque for right knee
		double cra; ///< scaling of Wang time cycle for right ankle
		double crh; ///< scaling of Wang time cycle for right hip
		double crk; ///< scaling of Wang time cycle for right knee
		double ara; ///< scaling of Wang torque for right ankle
		double brh; ///< shift of Wang time cycle for right hip
		double brk; ///< shift of Wang time cycle for right knee
		double bra; ///< shift of Wang time cycle for right ankle

		double alh; ///< scaling of Wang torque for left hip
		double alk; ///< scaling of Wang torque for left knee
		double ala; ///< scaling of Wang torque for left ankle
		double clh; ///< scaling of Wang time cycle for left hip
		double clk; ///< scaling of Wang time cycle for left knee
		double cla; ///< scaling of Wang time cycle for left ankle
		double blh; ///< shift of Wang time cycle for left hip
		double blk; ///< shift of Wang time cycle for left knee
		double bla; ///< shift of Wang time cycle for left ankle
};

#endif
