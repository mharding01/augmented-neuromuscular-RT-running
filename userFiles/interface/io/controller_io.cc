#include "controller_io.hh"

/*! \brief initialize the Inputs_ctrl structure
 * 
 * \param[in] nb_mot number of motors
 * \return initialized structure
 */
Inputs_ctrl* init_Inputs_ctrl(int nb_mot)
{
	Inputs_ctrl *inputs;

	inputs = (Inputs_ctrl*)  malloc(sizeof(Inputs_ctrl));

	inputs->t = 0.0;

	inputs->q      = (double*) malloc(nb_mot*sizeof(double));
	inputs->qd     = (double*) malloc(nb_mot*sizeof(double));
	inputs->Qq     = (double*) malloc(nb_mot*sizeof(double));
	inputs->q_mot  = (double*) malloc(nb_mot*sizeof(double));
	inputs->qd_mot = (double*) malloc(nb_mot*sizeof(double));

	for(int i=0; i<nb_mot; i++)
	{
		inputs->q[i]      = 0.0;
		inputs->qd[i]     = 0.0;
		inputs->Qq[i]     = 0.0;
		inputs->q_mot[i]  = 0.0;
		inputs->qd_mot[i] = 0.0;
	}

	for(int i=0; i<3; i++)
	{
		inputs->F_Rfoot[i] = 0.0;
		inputs->F_Lfoot[i] = 0.0;
		inputs->F_RToe_IF[i] = 0.0;
		inputs->F_LToe_IF[i] = 0.0;
		inputs->T_Rfoot[i] = 0.0;
		inputs->T_Lfoot[i] = 0.0;

		inputs->IMU_Angular_Rate[i] = 0.0;
		inputs->IMU_Acceleration[i] = 0.0;
	}

	for(int i=0; i<9; i++)
	{
		inputs->IMU_Orientation[i] = 0.0;
	}

	return inputs;
}

/*! \brief release memory for the Inputs_ctrl structure
 * 
 * \param[out] inputs structure to release
 */
void free_Inputs_ctrl(Inputs_ctrl *inputs)
{
	free(inputs->q);
	free(inputs->qd);
	free(inputs->Qq);
	free(inputs->q_mot);
	free(inputs->qd_mot);

	free(inputs);
}

/*! \brief initialize the Outputs_ctrl structure
 * 
 * \param[in] nb_mot number of motors
 * \return initialized structure
 */
Outputs_ctrl* init_Outputs_ctrl(int nb_mot)
{
	Outputs_ctrl *outputs;

	outputs = (Outputs_ctrl*)  malloc(sizeof(Outputs_ctrl));

	outputs->q_ref  = (double*) malloc(nb_mot*sizeof(double));
	outputs->Qq_ref = (double*) malloc(nb_mot*sizeof(double));

	for(int i=0; i<nb_mot; i++)
	{
		outputs->q_ref[i]  = 0.0;
		outputs->Qq_ref[i] = 0.0;
	}

	return outputs;
}

/*! \brief release memory for the Outputs_ctrl structure
 * 
 * \param[out] inputs structure to release
 */
void free_Outputs_ctrl(Outputs_ctrl *outputs)
{
	free(outputs->q_ref);
	free(outputs->Qq_ref);

	free(outputs);
}
