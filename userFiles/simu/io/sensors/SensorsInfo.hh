/*! 
 * \author Nicolas Van der Noot
 * \file SensorsInfo.hh
 * \brief SensorsInfo class
 */

#ifndef _SENSORS_INFO_HH_
#define _SENSORS_INFO_HH_

#include "mbs_data.h"

#include "ComputationSimu.hh"
#include "SimuOptions.h"
#include "ModelSimuIndex.hh"
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
	#include "mbs_sensor.h"
#ifdef __cplusplus
}
#endif

/*! \brief Fsensor info
 */
typedef struct Fsens_info
{
	double P[3];    ///< position [m]
	double V[3];    ///< velocity [m/s]
	double OM[3];   ///< angular velocity [rad/s]
	double R[3][3]; ///< rotational matrix [-]

} Fsens_info;

/*! \brief Get sensors info from Robotran sensors
 */
class SensorsInfo: public ComputationSimu
{
	public:
		SensorsInfo(MbsData *mbs_data, ModelSimuIndex *simu_index);
		virtual ~SensorsInfo();

		virtual void compute();

		double get_S_MidWaist_R(int index)   { return S_MidWaist_R[index];   }
		double get_S_MidWaist_OM(int index)  { return S_MidWaist_OM[index];  }
		double get_S_MidWaist_OMP(int index) { return S_MidWaist_OMP[index]; }
		double get_S_MidWaist_P(int index)   { return S_MidWaist_P[index];   }

		double get_S_RFoots_R(int index) { return S_RFoots_R[index];  }
		double get_S_RFoots_P(int index) { return S_RFoots_P[index];  }
		double get_S_LFoots_R(int index) { return S_LFoots_R[index];  }
		double get_S_LFoots_P(int index) { return S_LFoots_P[index];  }
		
		double get_S_RToe_R(int index) { return S_RToe_R[index];  }
		double get_S_LToe_R(int index) { return S_LToe_R[index];  }

		double get_S_RWrist_P(int index)   { return S_RWrist_P[index]; }
		double get_S_LWrist_P(int index)   { return S_LWrist_P[index]; }
		double get_S_Ball_P(int index)     { return S_Ball_P[index];   }

		double get_S_RWrist_V(int index)   { return S_RWrist_V[index]; }
		double get_S_LWrist_V(int index)   { return S_LWrist_V[index]; }
		double get_S_Ball_V(int index)     { return S_Ball_V[index];   }

		double get_waist_to_feet() { return waist_to_feet; }

		Fsens_info* get_F_Sens_RFoot() { return F_Sens_RFoot; }
		Fsens_info* get_F_Sens_LFoot() { return F_Sens_LFoot; }

		Fsens_info* get_F_Sens_RToe() { return F_Sens_RToe; }
		Fsens_info* get_F_Sens_LToe() { return F_Sens_LToe; }

		Fsens_info* get_F_Sens_RFlexProx() { return F_Sens_RFlexProx; }
		Fsens_info* get_F_Sens_RFlexDist() { return F_Sens_RFlexDist; }
		Fsens_info* get_F_Sens_LFlexProx() { return F_Sens_LFlexProx; }
		Fsens_info* get_F_Sens_LFlexDist() { return F_Sens_LFlexDist; }

	private:
		SimuOptions *options; ///< simulation options

		std::vector<int> com_ids;  ///< center of mass sensor indexes
		std::vector<int> body_ids; ///< id of all the bodies, coherent with 'com_ids'

		int coman_model;  ///< COMAN model

		int nb_joints; ///< number of joints in the MBS file

		int S_MidWaist_id; ///< id of the S sensor index of 'MidWaist' (as in the MBS file)

		int S_RFoots_id;   ///< id of the S sensor index of 'RFoots' (as in the MBS file)
		int S_LFoots_id;   ///< id of the S sensor index of 'LFoots' (as in the MBS file)

		int S_RFlexProx_id;   ///< id of the S sensor index of 'RightFlexProx' (as in the MBS file)
		int S_RFlexDist_id;   ///< id of the S sensor index of 'RightFlexDist' (as in the MBS file)
		int S_LFlexProx_id;   ///< id of the S sensor index of 'LeftFlexProx' (as in the MBS file)
		int S_LFlexDist_id;   ///< id of the S sensor index of 'LeftFlexDist' (as in the MBS file)

		int S_RToe_id;   ///< id of the S sensor index of 'RToe' (as in the MBS file)
		int S_LToe_id;   ///< id of the S sensor index of 'LToe' (as in the MBS file)

		int F_RWrist_id; ///< id of the F sensor index of 'RWrist' (as in the MBS file)
		int F_LWrist_id; ///< id of the F sensor index of 'LWrist' (as in the MBS file)
	
		double S_MidWaist_R[9];   ///< rotation matrix for IMU
		double S_MidWaist_OM[3];  ///< angular velocity for IMU
		double S_MidWaist_OMP[3]; ///< angular acceleration for IMU

		double S_MidWaist_P[3]; ///< MidWaist position [m]
		double S_MidWaist_V[3]; ///< MidWaist velocity [m]

		double S_RFoots_P[3]; ///< right foot position [m]
		double S_LFoots_P[3]; ///< left foot position [m]

		double S_RFoots_R[9]; ///< right foot rotation matrix [-]
		double S_LFoots_R[9]; ///< left foot rotation matrix [-]

		double S_RToe_R[9]; ///< right foot toes rotation matrix [-]
		double S_LToe_R[9]; ///< left foot toes rotation matrix [-]

		double waist_to_feet; ///< distance from waist to feet [m]

		// for objects handling
		double S_RWrist_P[3]; ///< right wrist position [m]
		double S_LWrist_P[3]; ///< left wrist position [m]

		double S_RWrist_V[3]; ///< right wrist velocity [m/s]
		double S_LWrist_V[3]; ///< left wrist velocity [m/s]

		double S_Ball_P[3]; ///< ball position [m]
		double S_Ball_V[3]; ///< ball velocity [m/s]

		double S_Box_P[3]; ///< ball position [m]
		double S_Box_V[3]; ///< ball velocity [m/s]

		// basic feet
		Fsens_info *F_Sens_RFoot; ///< F sensor info: right foot
		Fsens_info *F_Sens_LFoot; ///< F sensor info: left foot		

		// spring toe feet
		Fsens_info *F_Sens_RToe; ///< F sensor info: right toe
		Fsens_info *F_Sens_LToe; ///< F sensor info: left toe		

		// flex feet
		Fsens_info *F_Sens_RFlexProx; ///< F sensor info: RightFlexProx
		Fsens_info *F_Sens_RFlexDist; ///< F sensor info: RightFlexDist
		Fsens_info *F_Sens_LFlexProx; ///< F sensor info: LeftFlexProx
		Fsens_info *F_Sens_LFlexDist; ///< F sensor info: LeftFlexDist

		// sensors declaration

		MbsSensor S_MidWaist; ///< mid-waist sensor

		MbsSensor S_RFoots; ///< right foot sensor
		MbsSensor S_LFoots; ///< left foot sensor

		MbsSensor S_RToe; ///< right toe sensor
		MbsSensor S_LToe; ///< left toe sensor

		MbsSensor S_RFlexProx; ///< right proximal flex foot sensor
		MbsSensor S_RFlexDist; ///< right distal flex foot sensor
		MbsSensor S_LFlexProx; ///< left proximal flex foot sensor
		MbsSensor S_LFlexDist; ///< left distal flex foot sensor

		// functions prototypes
		void sensor_kinematics(int sens_id, int rel_waist);
		void rel_sensor_kinematics(int sens_1, int sens_2);
		void com_kinematics(int rel_waist);
		
		Fsens_info* new_Fsens_info();
		void fill_Fsens_info(MbsSensor cur_Fsens, Fsens_info *cur_Fsens_info);

		void com_body_init();
};

#endif
