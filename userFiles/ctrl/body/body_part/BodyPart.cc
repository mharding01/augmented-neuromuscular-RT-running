#include "BodyPart.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the body part
 */
BodyPart::BodyPart(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id)
{
	this->inputs = inputs;
	this->ctrl_index = ctrl_index;
	this->body_part_id = body_part_id;

	// metabolic energy
	met_energy = 0.0;

	flag_glp = 0;
}

/*! \brief destructor
 */
BodyPart::~BodyPart()
{
	// delete problem object
	if (flag_glp)
	{
		free(ar);
		free(ja);
		free(ia);

		#ifdef GLPK_LIB
		glp_delete_prob(lp);
		#endif
	}

	for(unsigned int i=0; i<muscles.size(); i++)
	{
		delete muscles[i];
	}

	for(unsigned int i=0; i<articulations.size(); i++)
	{
		delete articulations[i];
	}
}

/*! \brief initialize the glp class to use GLPK optimization tools
 * 
 * \param[in] nb_inputs number of inputs [-]
 * \param[in] nb_outputs number of outputs [-]
 */
void BodyPart::init_glp(int nb_inputs, int nb_outputs)
{
	int size_mat;

	// GLPK library to get stimulations from reference torques
	#ifdef GLPK_LIB

	flag_glp = 1;

	// create problem object
	lp = glp_create_prob();

	// disable terminal output
	glp_term_out(GLP_OFF);

	// optimization direction flag
	glp_set_obj_dir(lp, GLP_MIN);

	// add new rows (inputs) to problem object
	glp_add_rows(lp, nb_inputs);

	// set row (inputs) bounds: fixed input: lb == ub
	for(int i=0; i<nb_inputs; i++)
	{
		glp_set_row_bnds(lp, 1+i, GLP_FX, 0.0, 0.0);
	}

	// add new columns (outputs) to problem object
	glp_add_cols(lp, nb_outputs);
	#endif

	size_mat = 1+nb_inputs*nb_outputs;

	ia = (int*) calloc(size_mat,sizeof(int));
	ja = (int*) calloc(size_mat,sizeof(int));
	ar = (double*) calloc(size_mat,sizeof(double));

	// constraints matrices
	for(int i=0; i<nb_inputs; i++)
	{
		for(int j=0; j<nb_outputs; j++)
		{
			ia[i*nb_outputs+(j+1)] = i+1;
			ja[i*nb_outputs+(j+1)] = j+1;
			ar[i*nb_outputs+(j+1)] = 0.0;
		}
	}
}

/*! \brief final steps of the initialization
 */
void BodyPart::end_init()
{
	// stimulation references for Qq_ref
	for(unsigned int i=0; i<muscles.size(); i++)
	{
		Fm_ref.push_back(0.0);
		S_ref.push_back(0.0);
	}
}

/*! \brief update the leg kinematics and reset the torque
 */
void BodyPart::update_inputs()
{
	for(unsigned int i=0; i<articulations.size(); i++)
	{
		articulations[i]->update_q_qd();
		articulations[i]->reset_Qq();
	}
}

/*! \brief computations related to the body part
 * 
 * \param[in] stim_ctrl stimulations for the muscles controller
 */
void BodyPart::update_body_part(StimulationCtrl *stim_ctrl)
{
	met_energy = 0.0;

	// apply stimulations
	for(unsigned int i=0; i<muscles.size(); i++)
	{
		muscles[i]->compute(stim_ctrl);

		met_energy += muscles[i]->get_met_energy();
	}

	// add soft articulation limits
	for(unsigned int i=0; i<articulations.size(); i++)
	{
		articulations[i]->add_Qq_soft_lim();
	}
}

/*! \brief set the torque references from CtrlInputs to the articulations
 */
void BodyPart::set_Qq_ref_art()
{
	Articulation *cur_art;

	for(unsigned int i=0; i<articulations.size(); i++)
	{
		cur_art = articulations[i];

		cur_art->set_Qq_ref(inputs->get_Qq_ref(cur_art->get_joint_id()));
	}
}
