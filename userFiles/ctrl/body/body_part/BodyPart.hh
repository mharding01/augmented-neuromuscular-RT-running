/*! 
 * \author Nicolas Van der Noot & Bruno Somers
 * \file BodyPart.hh
 * \brief BodyPart class
 */
#ifndef _BODY_PART_HH_
#define _BODY_PART_HH_

#include "Articulation.hh"
#include "Muscle.hh"
#include "MotorCtrlIndex.hh"
#include <vector>

#ifdef GLPK_LIB
	#include "glpk.h"
#endif

/*! \brief body part with muscles and articulations
 */
class BodyPart
{
	public:
		BodyPart(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~BodyPart();

		virtual void update_S_ref() = 0;

		void update_inputs();
		
		void set_Qq_ref_art();

		virtual void update_body_part(StimulationCtrl *stim_ctrl);

		Articulation* get_articulations(int index) { return articulations[index]; }
		Muscle* get_muscle(int index) { return muscles[index]; }
		double get_Qq(int art_index) { return articulations[art_index]->get_Qq(); }
		double get_Qq_ref(int art_index) { return articulations[art_index]->get_Qq_ref(); }
		double get_q(int art_index) { return articulations[art_index]->get_q(); }
		double get_qd(int art_index) { return articulations[art_index]->get_qd(); }
		double get_met_energy() { return met_energy; }

		double get_S_ref(int i) const { return S_ref[i]; }

		int get_nb_muscles() const { return muscles.size(); }
		int get_nb_articulations() const { return articulations.size(); }

		int get_body_part_id() { return body_part_id; }

	protected:
		int body_part_id; ///< body part ID

		int flag_glp; ///< 1 if GLP initialized, 0 otherwise

		double met_energy; ///< metabolic energy consumption for this body part [J]

		CtrlInputs *inputs; ///< controller inputs
		MotorCtrlIndex *ctrl_index; ///< joint indexes

		std::vector<Articulation*> articulations; ///< articulations
		std::vector<Articulation*> pitch_articulations; ///< pitch articulations
		std::vector<Articulation*> roll_articulations;  ///< roll articulations
		std::vector<Articulation*> yaw_articulations;   ///< yaw articulations

		std::vector<Muscle*> muscles; ///< muscles
		std::vector<Muscle*> pitch_muscles; ///< pitch muscles
		std::vector<Muscle*> roll_muscles;  ///< roll muscles
		std::vector<Muscle*> yaw_muscles;   ///< yaw muscles

		std::vector<double> Fm_ref; ///< force references to track Qq_ref [-]
		std::vector<double> S_ref;  ///< stimulation references to track Qq_ref [-]

		// linear optimization
		int *ia; ///< constrait matrix: first index
		int *ja; ///< constrait matrix: second index
		double *ar; ///< constraint matrix

		#ifdef GLPK_LIB
		glp_prob *lp;
		#endif

		// function prototype
		void init_glp(int nb_inputs, int nb_outputs);
		void end_init();
};

#endif
