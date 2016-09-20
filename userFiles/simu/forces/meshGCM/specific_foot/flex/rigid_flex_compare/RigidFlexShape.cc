
#include "RigidFlexShape.hh"

#include <cstdlib>

#define NB_SEMI_POINTS 16
#define NB_POINTS (2 * NB_SEMI_POINTS)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 */
RigidFlexShape::RigidFlexShape(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id): ContactFoot(mbs_data, gait_features, sens_info, foot_id, NB_POINTS)
{
	// x axis
	for(int i=0; i<2; i++)
	{
		rn[0  + (i*NB_SEMI_POINTS)][0] = -0.02572;
		rn[1  + (i*NB_SEMI_POINTS)][0] = -0.01567;
		rn[2  + (i*NB_SEMI_POINTS)][0] = -0.00577;
		rn[3  + (i*NB_SEMI_POINTS)][0] =  0.00403;
		rn[4  + (i*NB_SEMI_POINTS)][0] =  0.01393;
		rn[5  + (i*NB_SEMI_POINTS)][0] =  0.02393;
		rn[6  + (i*NB_SEMI_POINTS)][0] =  0.0339;
		rn[7  + (i*NB_SEMI_POINTS)][0] =  0.044;
		rn[8  + (i*NB_SEMI_POINTS)][0] =  0.0542;
		rn[9  + (i*NB_SEMI_POINTS)][0] =  0.0644;
		rn[10 + (i*NB_SEMI_POINTS)][0] =  0.0746;
		rn[11 + (i*NB_SEMI_POINTS)][0] =  0.08475;
		rn[12 + (i*NB_SEMI_POINTS)][0] =  0.0949;
		rn[13 + (i*NB_SEMI_POINTS)][0] =  0.1049;
		rn[14 + (i*NB_SEMI_POINTS)][0] =  0.1149;
		rn[15 + (i*NB_SEMI_POINTS)][0] =  0.12485;
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
		rn[0  + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[1  + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[2  + (i*NB_SEMI_POINTS)][2] = 0.00149;
		rn[3  + (i*NB_SEMI_POINTS)][2] = 0.00398;
		rn[4  + (i*NB_SEMI_POINTS)][2] = 0.00547;
		rn[5  + (i*NB_SEMI_POINTS)][2] = 0.00597;
		rn[6  + (i*NB_SEMI_POINTS)][2] = 0.00647;
		rn[7  + (i*NB_SEMI_POINTS)][2] = 0.00597;
		rn[8  + (i*NB_SEMI_POINTS)][2] = 0.00448;
		rn[9  + (i*NB_SEMI_POINTS)][2] = 0.00299;
		rn[10 + (i*NB_SEMI_POINTS)][2] = 0.0015;
		rn[11 + (i*NB_SEMI_POINTS)][2] = 0.000498;
		rn[12 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[13 + (i*NB_SEMI_POINTS)][2] = 0.0;
		rn[14 + (i*NB_SEMI_POINTS)][2] = 0.000498;
		rn[15 + (i*NB_SEMI_POINTS)][2] = 0.00204;
	}
}

/*! \brief destructor
 */
RigidFlexShape::~RigidFlexShape()
{
	// free already done in mother class
}

/*! \brief get the Fsens_info struct related to this foot
 * 
 * \return requested Fsens_info struct
 */
Fsens_info* RigidFlexShape::get_F_Sens_body()
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
