
#include "RectToeShortFoot.hh"

#include <cstdlib>

/*
 * Foot dimensions:
 * x: 0.0  to 0.035  = 0.035
 * y: -0.045 to 0.045 = 0.09
 */
#define NB_X_POINTS  3
#define NB_Y_POINTS 4
#define NB_POINTS (NB_X_POINTS * NB_Y_POINTS)

#define X_MIN  0.0
#define Y_MIN -0.045

#define X_RES 0.0175
#define Y_RES 0.03


/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
RectToeShortFoot::RectToeShortFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
{
	int index;

	for (int i=0; i<NB_X_POINTS; i++)
	{
		for (int j=0; j<NB_Y_POINTS; j++)
		{
			index = j + i*NB_Y_POINTS;

			rn[index][0] = X_MIN + i*X_RES; ///< x indexes
			rn[index][1] = Y_MIN + j*Y_RES; ///< y indexes
			rn[index][2] = 0.0;             ///< z indexes
		}	
	}
}


/*! \brief destructor
 */
RectToeShortFoot::~RectToeShortFoot()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* RectToeShortFoot::get_F_Sens_body()
{
	switch (foot_id)
	{
		case R_FOOT_ID:
			return sens_info->get_F_Sens_RToe();

		case L_FOOT_ID:
			return sens_info->get_F_Sens_LToe();
	
		default:
			std::cout << "Error: unkown foot ID: " << foot_id << " !" << std::endl;
			exit(EXIT_FAILURE);
	}
}