
#include "RectFlextFoot.hh"

#include <cstdlib>

#define NB_X_POINTS 16
#define NB_Y_POINTS 2
#define NB_POINTS (NB_X_POINTS * NB_Y_POINTS)

#define X_MIN -0.03
#define Y_MIN -0.02

#define X_RES 0.01
#define Y_RES 0.04

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
RectFlextFoot::RectFlextFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
{
	int index;

	for (int i=0; i<NB_X_POINTS; i++)
	{
		for (int j=0; j<NB_Y_POINTS; j++)
		{
			index = j + i*NB_Y_POINTS;

			rn[index][0] = X_MIN + i*X_RES;
			rn[index][1] = Y_MIN + j*Y_RES;
			rn[index][2] = 0.0;
		}	
	}
}

/*! \brief destructor
 */
RectFlextFoot::~RectFlextFoot()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* RectFlextFoot::get_F_Sens_body()
{
	switch (foot_id)
	{
		case R_FOOT_ID:
			return sens_info->get_F_Sens_RFoot();

		case L_FOOT_ID:
			return sens_info->get_F_Sens_LFoot();

		default:
			std::cout << "Error: unkown foot ID: " << foot_id << " !" << std::endl;
			exit(EXIT_FAILURE);
	}
}
