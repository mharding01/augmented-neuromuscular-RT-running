#include "Arm.hh"

#include "Pitch_Shoulder_Art.hh"
#include "Roll_Shoulder_Art.hh"
#include "Yaw_Shoulder_Art.hh"
#include "Pitch_Elbow_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the body part
 */
Arm::Arm(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): BodyPart(inputs, ctrl_index, body_part_id)
{
	// safety
	if ((body_part_id != RIGHT_ARM_BODY) && (body_part_id != LEFT_ARM_BODY))
	{
		std::cout << "Arm error: body part ID (" << body_part_id << ") is not correct for arm !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// articulations
	for(int i=0; i<ARM_ART_NB; i++)
	{
		switch (i)
		{
			case PITCH_SHOULDER_ART : articulations.push_back(new Pitch_Shoulder_Art(inputs, ctrl_index, body_part_id)); break;
			case ROLL_SHOULDER_ART  : articulations.push_back(new Roll_Shoulder_Art( inputs, ctrl_index, body_part_id)); break;
			case YAW_SHOULDER_ART   : articulations.push_back(new Yaw_Shoulder_Art(  inputs, ctrl_index, body_part_id)); break;
			case PITCH_ELBOW_ART    : articulations.push_back(new Pitch_Elbow_Art(   inputs, ctrl_index, body_part_id)); break;
		
			default:
				std::cout << "Arm error: articulation " << i << " is not correct for arm !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch articulations
	pitch_articulations.push_back(articulations[PITCH_SHOULDER_ART]);
	pitch_articulations.push_back(articulations[PITCH_ELBOW_ART]);

	// roll articulation
	roll_articulations.push_back(articulations[ROLL_SHOULDER_ART]);

	// yaw articulation
	yaw_articulations.push_back(articulations[YAW_SHOULDER_ART]);

	//muscles
	for (int i=0; i<NB_ARM_MUSCLES; i++)
	{
		switch(i)
		{
			case EFL_MUSCLE : muscles.push_back(new EFL_Muscle(inputs, articulations[PITCH_ELBOW_ART]   , body_part_id)); break;
			case EET_MUSCLE : muscles.push_back(new EET_Muscle(inputs, articulations[PITCH_ELBOW_ART]   , body_part_id)); break;
			case SFL_MUSCLE : muscles.push_back(new SFL_Muscle(inputs, articulations[PITCH_SHOULDER_ART], body_part_id)); break;
			case SET_MUSCLE : muscles.push_back(new SET_Muscle(inputs, articulations[PITCH_SHOULDER_ART], body_part_id)); break;
			case SAB_MUSCLE : muscles.push_back(new SAB_Muscle(inputs, articulations[ROLL_SHOULDER_ART] , body_part_id)); break;
			case SAD_MUSCLE : muscles.push_back(new SAD_Muscle(inputs, articulations[ROLL_SHOULDER_ART] , body_part_id)); break;
			case SER_MUSCLE : muscles.push_back(new SER_Muscle(inputs, articulations[YAW_SHOULDER_ART]  , body_part_id)); break;
			case SIR_MUSCLE : muscles.push_back(new SIR_Muscle(inputs, articulations[YAW_SHOULDER_ART]  , body_part_id)); break;
			default:
				std::cout << "Arm error: articulation " << i << " is not correct for arm !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch muscles
	pitch_muscles.push_back(muscles[EET_MUSCLE]);
	pitch_muscles.push_back(muscles[EFL_MUSCLE]);
	pitch_muscles.push_back(muscles[SFL_MUSCLE]);
	pitch_muscles.push_back(muscles[SET_MUSCLE]);

	// roll muscles
	roll_muscles.push_back(muscles[SAB_MUSCLE]);
	roll_muscles.push_back(muscles[SAD_MUSCLE]);

	// yaw muscles
	yaw_muscles.push_back(muscles[SER_MUSCLE]);
	yaw_muscles.push_back(muscles[SIR_MUSCLE]);

	// specific muscles : pitch
	esb = static_cast<EET_Muscle*>(muscles[EET_MUSCLE]);
	esf = static_cast<EFL_Muscle*>(muscles[EFL_MUSCLE]);
	see = static_cast<SFL_Muscle*>(muscles[SFL_MUSCLE]);
	sei = static_cast<SET_Muscle*>(muscles[SET_MUSCLE]);

	// specific muscles : roll
	sae = static_cast<SAB_Muscle*>(muscles[SAB_MUSCLE]);
	sai = static_cast<SAD_Muscle*>(muscles[SAD_MUSCLE]);

	// specific muscles : yaw
	srr = static_cast<SER_Muscle*>(muscles[SER_MUSCLE]);
	srl = static_cast<SIR_Muscle*>(muscles[SIR_MUSCLE]);

	// end initialization
	end_init();
}

/*! \brief destructor
 */
Arm::~Arm()
{
	
}

/*! \brief update the stimulation references to get Qq_ref
 */
void Arm::update_S_ref()
{
	#ifdef GLPK_LIB
	double cur_Qq_ref;

	// -- pitch muscle -- //

	std::cout << "Error: pitch muscle inversion still not implemented for arms !" << std::endl;
	exit(EXIT_FAILURE);


	// -- roll muscle -- //

	cur_Qq_ref = articulations[ROLL_SHOULDER_ART]->get_Qq_ref();

	if (body_part_id == RIGHT_ARM_BODY)
	{
		Fm_ref[SAD_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / sai->get_rm_rs() : 0.0;
		Fm_ref[SAB_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / sae->get_rm_rs() : 0.0;
	}
	else
	{
		Fm_ref[SAD_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / sai->get_rm_rs() : 0.0;
		Fm_ref[SAB_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / sae->get_rm_rs() : 0.0;
	}


	// -- yaw muscle -- //

	cur_Qq_ref = articulations[YAW_SHOULDER_ART]->get_Qq_ref();

	if (body_part_id == RIGHT_ARM_BODY)
	{
		Fm_ref[SER_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / srr->get_rm_ys() : 0.0;
		Fm_ref[SIR_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / srl->get_rm_ys() : 0.0;
	}
	else
	{
		Fm_ref[SER_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / srr->get_rm_ys() : 0.0;
		Fm_ref[SIR_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / srl->get_rm_ys() : 0.0;
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
