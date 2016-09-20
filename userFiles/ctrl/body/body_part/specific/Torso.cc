#include "Torso.hh"

#include "Pitch_Torso_Art.hh"
#include "Roll_Torso_Art.hh"
#include "Yaw_Torso_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 */
Torso::Torso(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index): BodyPart(inputs, ctrl_index, TORSO_BODY)
{
	// articulations
	for(int i=0; i<TORSO_ART_NB; i++)
	{
		switch (i)
		{
			case PITCH_TORSO_ART : articulations.push_back(new Pitch_Torso_Art(inputs, ctrl_index)); break;
			case ROLL_TORSO_ART  : articulations.push_back(new Roll_Torso_Art( inputs, ctrl_index)); break;
			case YAW_TORSO_ART   : articulations.push_back(new Yaw_Torso_Art(  inputs, ctrl_index)); break;
		
			default:
				std::cout << "Torso error: articulation " << i << " is not correct for torso !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch articulation
	pitch_articulations.push_back(articulations[PITCH_TORSO_ART]);
	
	// roll articulation
	roll_articulations.push_back(articulations[ROLL_TORSO_ART]);
	
	// yaw articulation
	yaw_articulations.push_back(articulations[YAW_TORSO_ART]);

	//muscles
	for (int i=0; i<NB_TORSO_MUSCLES; i++)
	{
		switch(i)
		{
			case BFL_MUSCLE : muscles.push_back(new BFL_Muscle(inputs, articulations[PITCH_TORSO_ART])); break;
			case BET_MUSCLE : muscles.push_back(new BET_Muscle(inputs, articulations[PITCH_TORSO_ART])); break;
			case BTR_MUSCLE : muscles.push_back(new BTR_Muscle(inputs, articulations[ROLL_TORSO_ART])); break;
			case BTL_MUSCLE : muscles.push_back(new BTL_Muscle(inputs, articulations[ROLL_TORSO_ART])); break;
			case BRR_MUSCLE : muscles.push_back(new BRR_Muscle(inputs, articulations[YAW_TORSO_ART])); break;
			case BRL_MUSCLE : muscles.push_back(new BRL_Muscle(inputs, articulations[YAW_TORSO_ART])); break;

			default:
				std::cout << "Torso error: muscle " << i << " is not correct for torso !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	// pitch muscles
	pitch_muscles.push_back(muscles[BFL_MUSCLE]);
	pitch_muscles.push_back(muscles[BET_MUSCLE]);
	
	// roll muscles
	roll_muscles.push_back(muscles[BTR_MUSCLE]);
	roll_muscles.push_back(muscles[BTL_MUSCLE]);

	// yaw muscles
	yaw_muscles.push_back(muscles[BRR_MUSCLE]);
	yaw_muscles.push_back(muscles[BRL_MUSCLE]);

	// specific muscles : pitch
	brf = static_cast<BFL_Muscle*>(muscles[BFL_MUSCLE]);
	brb = static_cast<BET_Muscle*>(muscles[BET_MUSCLE]);

	// specific muscles : roll
	btr = static_cast<BTR_Muscle*>(muscles[BTR_MUSCLE]);
	btl = static_cast<BTL_Muscle*>(muscles[BTL_MUSCLE]);

	// specific muscles : yaw
	blr = static_cast<BRR_Muscle*>(muscles[BRR_MUSCLE]);
	bll = static_cast<BRL_Muscle*>(muscles[BRL_MUSCLE]);

	// end initialization
	end_init();
}

/*! \brief destructor
 */
Torso::~Torso()
{

}

/*! \brief update the stimulation references to get Qq_ref
 */
void Torso::update_S_ref()
{
	#ifdef GLPK_LIB
	double cur_Qq_ref;

	// -- pitch muscle -- //

	cur_Qq_ref = articulations[PITCH_TORSO_ART]->get_Qq_ref();

	Fm_ref[BET_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / brb->get_rm_pt() : 0.0;
	Fm_ref[BFL_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / brf->get_rm_pt() : 0.0;


	// -- roll muscle -- //

	cur_Qq_ref = articulations[ROLL_TORSO_ART]->get_Qq_ref();

	Fm_ref[BTL_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / btl->get_rm_rt() : 0.0;
	Fm_ref[BTR_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / btr->get_rm_rt() : 0.0;


	// -- yaw muscle -- //

	cur_Qq_ref = articulations[YAW_TORSO_ART]->get_Qq_ref();

	Fm_ref[BRR_MUSCLE] = cur_Qq_ref < 0.0 ? -cur_Qq_ref / blr->get_rm_yt() : 0.0;
	Fm_ref[BRL_MUSCLE] = cur_Qq_ref > 0.0 ?  cur_Qq_ref / bll->get_rm_yt() : 0.0;


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
