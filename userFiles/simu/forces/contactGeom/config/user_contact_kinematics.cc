#include "user_contact_kinematics.hh"
#include "RigidShape.hh"
#include "MainUnionShape.hh"

#include "user_IO.h"

// function prototype
extern "C" { void user_kinematics_model(MbsData *mbs_data, double **PxF_tab, double ***RxF_tab, double **VxF_tab, double **OMxF_tab); }

namespace ContactGeom{

/*! \brief kinematics update
 * 
 * \param[out] kin_info_list kinematic information of all moving bodies
 * \param[in] main_union main union of shapes
 * \param[in] mbs_data Robotran structure
 * \param[in,out] PxF_tab tabular with 'PxF' computed with 'user_kinematics_model'
 * \param[in,out] VxF_tab tabular with 'VxF' computed with 'user_kinematics_model'
 * \param[in,out] OMxF_tab tabular with 'OMxF' computed with 'user_kinematics_model'
 * \param[in,out] RxF_tab tabular with 'RxF' computed with 'user_kinematics_model'
 *
 * This function is used to compute the kinematics of all bodies involved in the contact model.
 * Using the line 'main_union->mbs_sensor_compute();' is enough to automatically get the correct result.
 * Unfortunately, this function can be quite slow because it uses the 'mbs_sensor' function, which recomputes
 * all the kinematics of each body, each time starting from scratch.
 *
 * For more advance users who want to improve the computational efficiency, you can remove this line and
 * perform yourself the computation. The purpose of this function is to fill 'kin_info_list', as can
 * be seen in the 'update_kin_info' function (see bottom of this file).
 *
 * An easy solution to handle this computation in an efficient way is to do as follows:
 * - Create a new .mbs file (similar to the main one), but where all bodies with a changing kinematics
 *   (i.e. bodies added with the lines 'add_rigid_Ssens' or 'add_rigid_Fsens' in 'user_shapes.cc') are
 *   assigned to a F sensor (even the ones which should be with a S sensor, due to 'add_rigid_Ssens').
 *   Like in 'user_shapes.cc', check only the fields 'Position', 'Rotation Matrix' and 'Velocity' for
 *   the F sensors. Also, remove all the other F sensors in this special .mbs file.
 * - Generate the accelred symbolic equations and open the function 'mbs_accelred'
 * - Copy these two lines: 'q = s->q;' and 'qd = s->qd;'
 * - Copy the sine and cosine equations involved in the requested kinematics computations from 'mbs_accelred'.
 * - Copy the whole block 'Sensor Kinematics' from 'mbs_accelred' (stop just before 'SWr1 = user_ExtForces(...);')
 * - Add the requested declarations found in 'mbs_accelred_xxx.c' and 'mbs_accelred_xxx.h'
 * - For each body where a 'F sensor' was added in the special .mbs file, add this line:
 *         update_kin_info(PxFi, RxFi, VxFi, OMxFi, kin_info_list, j);
 *   where 'i' is the index of the F sensor (i.e. in the order of definition in the special .mbs file, starting with '1')
 *   and 'j' is 'isens', i.e. the index which would be called in the 'mbs_sensor' function (greater or equal to '1') for
 *   the main .mbs file. If it is a F sensor (in the main .mbs file), its index is 'mbs_data->Nsensor + k', where
 *   'k' is the F sensor index.
 *
 * A script called 'kinematics_gen.py' was developed in the folder 'contactGeom/python' to do this automatically.
 * To use it, locate the accelred file generated with the special .mbs file. Then, launch it with two arguments:
 *    - the first one is the path to this accelred file
 *    - the second one is the output file (.c), like 'user_kinematics_model.c'
 * When, it is done, you can for instance place this file inside the 'symbolicR' folder.
 * Then, you just need to call the function 'user_kinematics_model(mbs_data, PxF_tab, RxF_tab, VxF_tab, OMxF_tab);'.
 * The outputs of the function will be located inside 'PxF_tab', 'RxF_tab', 'VxF_tab' and 'OMxF_tab'.
 * Here are the relashionships:
 *    PxF_tab[i-1]  is PxFi
 *    RxF_tab[i-1]  is RxFi
 *    OMxF_tab[i-1] is OMxFi
 *    VxF_tab[i-1]  is VxFi
 * So, you can call this line:
 *         update_kin_info(PxF_tab[i-1], RxF_tab[i-1], OMxF_tab[i-1], VxF_tab[i-1], kin_info_list, j);
 * instead of this one:
 *         update_kin_info(PxFi, RxFi, VxFi, OMxFi, kin_info_list, j);
 */
void user_contact_kinematics(std::vector<KinInfo> &kin_info_list, MainUnionShape *main_union, MbsData *mbs_data,
	double **PxF_tab, double **VxF_tab, double **OMxF_tab, double ***RxF_tab)
{
	int nb_sens;
	SimuOptions *options;

	options = mbs_data->user_IO->options;

	switch (options->coman_model)
	{
		case INIT_FEET_COMAN:
		case LONG_FEET_COMAN:
		case SHORT_FEET_COMAN:
			nb_sens = 12;
			break;

		case SHORT_FEET_BALL_COMAN:
			nb_sens = 13;
			break;

		case FLEX_FEET_COMAN:
		case SPRING_TOE_SHORT_FEET_COMAN:
		case SPRING_TOE_FEET_COMAN:
			nb_sens = 14;
			break;

		default:
			std::cout << "Error: unknown COMAN model: " << options->coman_model << " !" << std::endl;
			exit(EXIT_FAILURE);
	}

	/*if (kin_info_list.size() == nb_sens)
	{
		user_kinematics_model(mbs_data, PxF_tab, RxF_tab, VxF_tab, OMxF_tab);

		for(int i=0; i<nb_sens; i++)
		{
			update_kin_info(PxF_tab[i], RxF_tab[i], VxF_tab[i], OMxF_tab[i], kin_info_list, mbs_data->Nsensor + 1 + i);
		}
	}
	else
	{
		main_union->mbs_sensor_compute();
	}*/ 
	main_union->mbs_sensor_compute(); //to remove
	
}

/*! \brief update 'kin_info_list'
 * 
 * \param[in] PxF position [m]
 * \param[in] RxF rotation matrix [-]
 * \param[in] VxF velocity [m/s]
 * \param[in] OMxF angular velocity [rad/s]
 * \param[out] kin_info_list kinematic information of all moving bodies
 * \param[in] isens sensor index as it appears in 'mbs_sensor'
 */
void update_kin_info(double *PxF, double **RxF, double *VxF, double *OMxF, std::vector<KinInfo> &kin_info_list, int isens)
{
	int index;

	index = -1;

	// find good index
	for(unsigned int i=0; i<kin_info_list.size(); i++)
	{
		if (isens == kin_info_list[i].isens)
		{
			index = i;
			break;
		}
	}

	// safety
	if (index < 0)
	{
		std::cout << "Error: isens " << isens << " not found in 'kin_info_list' !" << std::endl;
		exit(EXIT_FAILURE);
	}

	for(int i=0; i<3; i++)
	{
		kin_info_list[index].P[i]  = PxF[1+i];  ///< position [m]
		kin_info_list[index].V[i]  = VxF[1+i];  ///< velocity [m/s]
		kin_info_list[index].OM[i] = OMxF[1+i]; ///< angular velocity [rad/s]

		for(int j=0; j<3; j++)
		{
			kin_info_list[index].R[i][j] = RxF[1+i][1+j]; ///< rotation matrix [-]
		}
	}
}

}
