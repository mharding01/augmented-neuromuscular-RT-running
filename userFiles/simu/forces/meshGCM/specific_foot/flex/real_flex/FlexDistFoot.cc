
#include "FlexDistFoot.hh"

#include <cstdlib>

#define NB_SEMI_POINTS 10
#define NB_POINTS (2 * NB_SEMI_POINTS)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
FlexDistFoot::FlexDistFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
{
	// x axis
	for(int i=0; i<2; i++)
	{
		rn[0 + (i*NB_SEMI_POINTS)][0] = -0.071;
		rn[1 + (i*NB_SEMI_POINTS)][0] = -0.0609;
		rn[2 + (i*NB_SEMI_POINTS)][0] = -0.0507;
		rn[3 + (i*NB_SEMI_POINTS)][0] = -0.0405;
		rn[4 + (i*NB_SEMI_POINTS)][0] = -0.0303;
		rn[5 + (i*NB_SEMI_POINTS)][0] = -0.02015;
		rn[6 + (i*NB_SEMI_POINTS)][0] = -0.01;
		rn[7 + (i*NB_SEMI_POINTS)][0] =  0.0;
		rn[8 + (i*NB_SEMI_POINTS)][0] =  0.01;
		rn[9 + (i*NB_SEMI_POINTS)][0] =  0.01995;
	}

	// y axis
	for(int i=0; i<NB_SEMI_POINTS; i++)
	{
		rn[0 + i][1] = -0.02;
		rn[NB_SEMI_POINTS + i][1] =  0.02;
	}

	// z axis
	for(int i=0; i<2; i++)
	{
		rn[0 + (i*NB_SEMI_POINTS)][2] = 0.00647;
		rn[1 + (i*NB_SEMI_POINTS)][2] = 0.00597;
		rn[2 + (i*NB_SEMI_POINTS)][2] = 0.00448;
		rn[3 + (i*NB_SEMI_POINTS)][2] = 0.00299;
		rn[4 + (i*NB_SEMI_POINTS)][2] = 0.0015;
		rn[5 + (i*NB_SEMI_POINTS)][2] = 0.000498;
		rn[6 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[7 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[8 + (i*NB_SEMI_POINTS)][2] = 0.000498;
		rn[9 + (i*NB_SEMI_POINTS)][2] = 0.00204;
	}
}

/*! \brief destructor
 */
FlexDistFoot::~FlexDistFoot()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* FlexDistFoot::get_F_Sens_body()
{
	switch (foot_id)
	{
		case R_FOOT_ID:
			return sens_info->get_F_Sens_RFlexDist();

		case L_FOOT_ID:
			return sens_info->get_F_Sens_LFlexDist();

		default:
			std::cout << "Error: unkown foot ID: " << foot_id << " !" << std::endl;
			exit(EXIT_FAILURE);
	}
}
