
#include "ShortFoot.hh"

#include <cstdlib>

/*
 * Foot dimensions:
 * x: -0.06  to 0.08  = 0.14
 * y: -0.045 to 0.045 = 0.09
 */
#define NB_X_POINTS 5
#define NB_Y_POINTS 4
#define NB_POINTS (NB_X_POINTS * NB_Y_POINTS)

#define X_MIN -0.06  ///< minimal x position [m]
#define Y_MIN -0.045 ///< minimal y posiiton [m]

#define X_RES 0.035 ///< x increment [m]
#define Y_RES 0.03  ///< y increment [m]

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
ShortFoot::ShortFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
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
ShortFoot::~ShortFoot()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* ShortFoot::get_F_Sens_body()
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
