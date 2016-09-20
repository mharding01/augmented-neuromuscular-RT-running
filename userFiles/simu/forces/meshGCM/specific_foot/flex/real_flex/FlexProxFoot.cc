
#include "FlexProxFoot.hh"

#include <cstdlib>

#define NB_SEMI_POINTS 6
#define NB_POINTS (2 * NB_SEMI_POINTS)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
FlexProxFoot::FlexProxFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
{
	// x axis
	for(int i=0; i<2; i++)
	{
		rn[0 + (i*NB_SEMI_POINTS)][0] = -0.01005;
		rn[1 + (i*NB_SEMI_POINTS)][0] =  0.0;
		rn[2 + (i*NB_SEMI_POINTS)][0] =  0.0099;
		rn[3 + (i*NB_SEMI_POINTS)][0] =  0.0197;
		rn[4 + (i*NB_SEMI_POINTS)][0] =  0.0296;
		rn[5 + (i*NB_SEMI_POINTS)][0] =  0.0396;
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
		rn[0 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[1 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[2 + (i*NB_SEMI_POINTS)][2] = 0.00149;
		rn[3 + (i*NB_SEMI_POINTS)][2] = 0.00398;
		rn[4 + (i*NB_SEMI_POINTS)][2] = 0.00547;
		rn[5 + (i*NB_SEMI_POINTS)][2] = 0.00597;
	}
}

/*! \brief destructor
 */
FlexProxFoot::~FlexProxFoot()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* FlexProxFoot::get_F_Sens_body()
{
	switch (foot_id)
	{
		case R_FOOT_ID:
			return sens_info->get_F_Sens_RFlexProx();

		case L_FOOT_ID:
			return sens_info->get_F_Sens_LFlexProx();

		default:
			std::cout << "Error: unkown foot ID: " << foot_id << " !" << std::endl;
			exit(EXIT_FAILURE);
	}
}
