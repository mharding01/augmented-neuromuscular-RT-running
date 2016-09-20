#include "Leg.hh"
#include "Pitch_Foot_Art.hh"
#include "Roll_Foot_Art.hh"
#include "Pitch_Knee_Art.hh"
#include "Yaw_Hip_Art.hh"
#include "Roll_Hip_Art.hh"
#include "Pitch_Hip_Art.hh"

// orders of the pitch muscles, for optimization
enum {SOL_ID, TA_ID, GAS_ID, VAS_ID, HAM_ID, GLU_ID, HFL_ID, RF_ID, NB_PITCH_MUSCLE_IDS};

#define SAFETY_MIN 1.2 ///< security factor for minimal muscle value

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the body part
 */
Leg::Leg(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): BodyPart(inputs, ctrl_index, body_part_id)
{
	double cur_Fmax;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Leg error: body part ID (" << body_part_id << ") is not correct for leg !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// articulations
	for(int i=0; i<LEG_ART_NB; i++)
	{
		switch (i)
		{
			case PITCH_FOOT_ART : articulations.push_back(new Pitch_Foot_Art(inputs, ctrl_index, body_part_id)); break;
			case ROLL_FOOT_ART  : articulations.push_back(new Roll_Foot_Art( inputs, ctrl_index, body_part_id)); break;
			case PITCH_KNEE_ART : articulations.push_back(new Pitch_Knee_Art(inputs, ctrl_index, body_part_id)); break;
			case YAW_HIP_ART    : articulations.push_back(new Yaw_Hip_Art(   inputs, ctrl_index, body_part_id)); break;
			case ROLL_HIP_ART   : articulations.push_back(new Roll_Hip_Art(  inputs, ctrl_index, body_part_id)); break;
			case PITCH_HIP_ART  : articulations.push_back(new Pitch_Hip_Art( inputs, ctrl_index, body_part_id)); break;
		
			default:
				std::cout << "Leg error: articulation " << i << " is not correct for leg !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch
	pitch_articulations.push_back(articulations[PITCH_FOOT_ART]);
	pitch_articulations.push_back(articulations[PITCH_KNEE_ART]);
	pitch_articulations.push_back(articulations[PITCH_HIP_ART]);

	// roll
	roll_articulations.push_back(articulations[ROLL_FOOT_ART]);
	roll_articulations.push_back(articulations[ROLL_HIP_ART]);

	// yaw
	yaw_articulations.push_back(articulations[YAW_HIP_ART]);

	// muscles
	for(int i=0; i<NB_LEG_MUSCLES; i++)
	{
		switch (i)
		{
			// pitch
			case SOL_MUSCLE : muscles.push_back(new SOL_Muscle(inputs, articulations[PITCH_FOOT_ART], body_part_id)); break;
			case TA_MUSCLE  : muscles.push_back(new  TA_Muscle(inputs, articulations[PITCH_FOOT_ART], body_part_id)); break;
			case GAS_MUSCLE : muscles.push_back(new GAS_Muscle(inputs, articulations[PITCH_FOOT_ART], articulations[PITCH_KNEE_ART], body_part_id)); break;
			case VAS_MUSCLE : muscles.push_back(new VAS_Muscle(inputs, articulations[PITCH_KNEE_ART], body_part_id)); break;
			case HAM_MUSCLE : muscles.push_back(new HAM_Muscle(inputs, articulations[PITCH_KNEE_ART], articulations[PITCH_HIP_ART], body_part_id)); break;
			case GLU_MUSCLE : muscles.push_back(new GLU_Muscle(inputs, articulations[PITCH_HIP_ART], body_part_id)); break;
			case HFL_MUSCLE : muscles.push_back(new HFL_Muscle(inputs, articulations[PITCH_HIP_ART], body_part_id)); break;
			case RF_MUSCLE  : muscles.push_back(new  RF_Muscle(inputs, articulations[PITCH_KNEE_ART], articulations[PITCH_HIP_ART], body_part_id)); break;

			// roll
			case HAB_MUSCLE : muscles.push_back(new HAB_Muscle(inputs, articulations[ROLL_HIP_ART], body_part_id)); break;
			case HAD_MUSCLE : muscles.push_back(new HAD_Muscle(inputs, articulations[ROLL_HIP_ART], body_part_id)); break;
			case EVE_MUSCLE : muscles.push_back(new EVE_Muscle(inputs, articulations[ROLL_FOOT_ART], body_part_id)); break;
			case INV_MUSCLE : muscles.push_back(new INV_Muscle(inputs, articulations[ROLL_FOOT_ART], body_part_id)); break;

			// yaw
			case HER_MUSCLE : muscles.push_back(new HER_Muscle(inputs, articulations[YAW_HIP_ART], body_part_id)); break;
			case HIR_MUSCLE : muscles.push_back(new HIR_Muscle(inputs, articulations[YAW_HIP_ART], body_part_id)); break;

			default:
				std::cout << "Leg error: muscle " << i << " is not correct for leg !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch
	pitch_muscles.push_back(muscles[SOL_MUSCLE]);
	pitch_muscles.push_back(muscles[TA_MUSCLE]);
	pitch_muscles.push_back(muscles[GAS_MUSCLE]);
	pitch_muscles.push_back(muscles[VAS_MUSCLE]);
	pitch_muscles.push_back(muscles[HAM_MUSCLE]);
	pitch_muscles.push_back(muscles[GLU_MUSCLE]);
	pitch_muscles.push_back(muscles[HFL_MUSCLE]);
	pitch_muscles.push_back(muscles[RF_MUSCLE]);

	// roll
	roll_muscles.push_back(muscles[HAB_MUSCLE]);
	roll_muscles.push_back(muscles[HAD_MUSCLE]);
	roll_muscles.push_back(muscles[EVE_MUSCLE]);
	roll_muscles.push_back(muscles[INV_MUSCLE]);

	// yaw
	yaw_muscles.push_back(muscles[HER_MUSCLE]);
	yaw_muscles.push_back(muscles[HIR_MUSCLE]);

	// specific muscles : pitch
	sol = static_cast<SOL_Muscle*>(muscles[SOL_MUSCLE]);
	ta  = static_cast<TA_Muscle*>(muscles[TA_MUSCLE]);
	gas = static_cast<GAS_Muscle*>(muscles[GAS_MUSCLE]);
	vas = static_cast<VAS_Muscle*>(muscles[VAS_MUSCLE]);
	ham = static_cast<HAM_Muscle*>(muscles[HAM_MUSCLE]);
	glu = static_cast<GLU_Muscle*>(muscles[GLU_MUSCLE]);
	hfl = static_cast<HFL_Muscle*>(muscles[HFL_MUSCLE]);
	rf =  static_cast<RF_Muscle*>(muscles[RF_MUSCLE]);

	// specific muscles : roll
	hab = static_cast<HAB_Muscle*>(muscles[HAB_MUSCLE]);
	had = static_cast<HAD_Muscle*>(muscles[HAD_MUSCLE]);
	eve = static_cast<EVE_Muscle*>(muscles[EVE_MUSCLE]);
	inv = static_cast<INV_Muscle*>(muscles[INV_MUSCLE]);

	// specific muscles : yaw
	her = static_cast<HER_Muscle*>(muscles[HER_MUSCLE]);
	hir = static_cast<HIR_Muscle*>(muscles[HIR_MUSCLE]);


	// -- Invert muscles -- //

	// generic glp initialization
	init_glp(pitch_articulations.size(), pitch_muscles.size());

	// personnal glp initialization: output bounds and weights
	#ifdef GLPK_LIB
	for (int i=0; i<pitch_muscles.size(); i++)
	{
		cur_Fmax = pitch_muscles[i]->get_Fmax();

		// set columns (outputs) bounds: double-bounded variable
		glp_set_col_bnds(lp, i+1, GLP_DB, 0.0, cur_Fmax);

		/* set objective coefficient or constant term: objective function
		 * is the sum of the outputs multiplied by these coefficients
		 */
		glp_set_obj_coef(lp, i+1, 1.0/cur_Fmax);
	}
	#endif

	// minimal values for muscles outputs (linear opti)
	for(unsigned int i=0; i<pitch_muscles.size(); i++)
	{
		F_min_pitch.push_back(0.0);
		l_ce_star_pitch.push_back(pitch_muscles[i]->get_lce());
	}

	// end initialization
	end_init();
}

/*! \brief destructor
 */
Leg::~Leg()
{

}

/*! \brief update the stimulation references to get Qq_ref
 */
void Leg::update_S_ref()
{
	#ifdef GLPK_LIB
	int index;
	double cur_Qq_ref, cur_Fm_ref;

	// -- pitch muscles -- //

	// set row (inputs) bounds: fixed input: lb == ub
	for(int i=0; i<pitch_articulations.size(); i++)
	{
		cur_Qq_ref = pitch_articulations[i]->get_Qq_ref();

		glp_set_row_bnds(lp, 1+i, GLP_FX, cur_Qq_ref, cur_Qq_ref);
	}

	// set columns (outputs) bounds: double-bounded variable
	for (int i=0; i<pitch_muscles.size(); i++)
	{
		pitch_muscles[i]->compute_F_min(l_ce_star_pitch[i], F_min_pitch[i]);

		if (0.0 < F_min_pitch[i] && F_min_pitch[i] < 0.1*pitch_muscles[i]->get_Fmax()) // safety
		{
			glp_set_col_bnds(lp, i+1, GLP_DB, SAFETY_MIN*F_min_pitch[i], pitch_muscles[i]->get_Fmax());
		}
		else
		{
			glp_set_col_bnds(lp, i+1, GLP_DB, 0.0, pitch_muscles[i]->get_Fmax());
		}
	}

	// lever arms
	for(int i=0; i<pitch_articulations.size(); i++)
	{
		index = 1+pitch_muscles.size()*i;

		switch (i)
		{
			case 0:
				ar[index+SOL_ID] =  sol->get_rm_pa();
				ar[index+TA_ID]  = -ta->get_rm_pa();
				ar[index+GAS_ID] =  gas->get_rm_pa();
				break;

			case 1:
				ar[index+GAS_ID] =  gas->get_rm_pk();
				ar[index+VAS_ID] = -vas->get_rm_pk();
				ar[index+HAM_ID] =  ham->get_rm_pk();
				ar[index+RF_ID]  =  -rf->get_rm_pk();
				break;

			case 2:
				ar[index+HAM_ID] =  ham->get_rm_ph();
				ar[index+RF_ID]  =  -rf->get_rm_ph();
				ar[index+GLU_ID] =  glu->get_rm_ph();
				ar[index+HFL_ID] = -hfl->get_rm_ph();
				break;

			default:
				std::cout << "Error: index impossible for lever arms !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// load (replace) the whole constraint matrix
	glp_load_matrix(lp, pitch_articulations.size()*pitch_muscles.size(), ia, ja, ar);

	// solve LP problem with the primal or dual simplex method
	glp_simplex(lp, NULL);

	// get back all forces: pitch
	for(int i=0; i<pitch_muscles.size(); i++)
	{
		cur_Fm_ref = glp_get_col_prim(lp, i+1);

		switch (i)
		{
			case SOL_ID : Fm_ref[SOL_MUSCLE] = cur_Fm_ref; break;
			case TA_ID  : Fm_ref[TA_MUSCLE]  = cur_Fm_ref; break;
			case GAS_ID : Fm_ref[GAS_MUSCLE] = cur_Fm_ref; break;
			case VAS_ID : Fm_ref[VAS_MUSCLE] = cur_Fm_ref; break;
			case HAM_ID : Fm_ref[HAM_MUSCLE] = cur_Fm_ref; break;
			case GLU_ID : Fm_ref[GLU_MUSCLE] = cur_Fm_ref; break;
			case HFL_ID : Fm_ref[HFL_MUSCLE] = cur_Fm_ref; break;
			case RF_ID  : Fm_ref[RF_MUSCLE]  = cur_Fm_ref; break;
		
			default:
				std::cout << "Error: muscle index impossible for GLPK output !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// -- roll muscle -- //

	for(unsigned int i=0; i<roll_articulations.size(); i++)
	{
		switch (i)
		{
			case 0 :
				cur_Qq_ref = articulations[ROLL_FOOT_ART]->get_Qq_ref();

				if (body_part_id == RIGHT_LEG_BODY)
				{
					Fm_ref[EVE_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / eve->get_rm_ra() : 0.0;
					Fm_ref[INV_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / inv->get_rm_ra() : 0.0;
				}
				else
				{
					Fm_ref[EVE_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / eve->get_rm_ra() : 0.0;
					Fm_ref[INV_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / inv->get_rm_ra() : 0.0;
				}
					
				break;

			case 1 :
				cur_Qq_ref = articulations[ROLL_HIP_ART]->get_Qq_ref();

				if (body_part_id == RIGHT_LEG_BODY)
				{
					Fm_ref[HAB_MUSCLE] = cur_Qq_ref  < 0.0 ? -cur_Qq_ref  / hab->get_rm_rh() : 0.0;
					Fm_ref[HAD_MUSCLE] = cur_Qq_ref  > 0.0 ?  cur_Qq_ref  / had->get_rm_rh() : 0.0;
				}
				else
				{
					Fm_ref[HAB_MUSCLE] = cur_Qq_ref  > 0.0 ?  cur_Qq_ref  / hab->get_rm_rh() : 0.0;
					Fm_ref[HAD_MUSCLE] = cur_Qq_ref  < 0.0 ? -cur_Qq_ref  / had->get_rm_rh() : 0.0;
				}
				
				break;
		
			default:
				std::cout << "Error: roll articulation impossible for inverse stimulation !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}


	// -- yaw muscle -- //

	cur_Qq_ref = articulations[YAW_HIP_ART]->get_Qq_ref();

	if (body_part_id == RIGHT_LEG_BODY)
	{
		Fm_ref[HER_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / her->get_rm_yh() : 0.0;
		Fm_ref[HIR_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / hir->get_rm_yh() : 0.0;
	}
	else
	{
		Fm_ref[HER_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / her->get_rm_yh() : 0.0;
		Fm_ref[HIR_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / hir->get_rm_yh() : 0.0;
	}

	// compute stimulation inverse
	for(unsigned int i=0; i<S_ref.size(); i++)
	{
		S_ref[i] = muscles[i]->inverse_compute(Fm_ref[i]);
	}

	#else
	std::cout << "Error: set FLAG_GLPK to ON to use GLPK !" << std::endl;
	exit(EXIT_FAILURE);
	#endif
}
