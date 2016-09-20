
#include "StopSimu.hh"
#include "CppInterface.hh"
#include "WholeFeet.hh"
#include "user_model.h"
#include "SimuIndex.hh"
#include "ContactGestion.hh"
#include "user_IO.h"

#include <cmath>

#define FALL_THRESHOLD 0.15 ///< threshold to detect fall [m]

#define GROUND_FORCES_THRESHOLD 1.0e4 ///< threshold for ground reaction forces [N]

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
StopSimu::StopSimu(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info): ComputationSimu(mbs_data, simu_index)
{
	this->sens_info = sens_info;

	limit_joints = new JointsLimitRobot(mbs_data, simu_index);

	gcm_model = options->gcm_model;

	n_joint = mbs_data->njoint;
}

/*! \brief destructor
 */
StopSimu::~StopSimu()
{
	delete limit_joints;
}

/*! \brief computations
 */
void StopSimu::compute()
{
	// variables declaration
	CppInterface *cppInterface;

	WholeFeet *whole_feet;

	// variables initialization
	UserModel *um;

	um = mbs_data->user_model;

	cppInterface = static_cast<CppInterface*>(um->cppInterface);


	// --- Event detection fall --- //

	mbs_data->flag_stop = 0;

	if (sens_info->get_waist_to_feet() <= FALL_THRESHOLD)
	{
		mbs_data->flag_stop = 1;

		if (options->print)
		{
			std::cout << "Stopping simulation: detection fall !" << std::endl;
		}
	}


	// ---- Stopping simulation if ground forces too high ---- //

	switch (gcm_model)
	{
		case NO_GCM_MODEL:
			break;

		case MESH_GCM_MODEL:
			whole_feet = cppInterface->get_simu_ctrl()->get_gcm_mesh()->get_whole_feet();

			if((fabs(whole_feet->get_leg_feet_forces(RIGHT_ID, 2)) > GROUND_FORCES_THRESHOLD) ||
			   (fabs(whole_feet->get_leg_feet_forces(LEFT_ID, 2))  > GROUND_FORCES_THRESHOLD))
			{
				mbs_data->flag_stop = 1;

				if (options->print)
				{
					std::cout << "Stopping simulation: ground forces too important !" << std::endl;
				}
			}
			break;
	
		case PRIM_GCM_MODEL:
			if((fabs(rfoot_shape->get_F_tot_z()) > GROUND_FORCES_THRESHOLD) ||
			   (fabs(lfoot_shape->get_F_tot_z()) > GROUND_FORCES_THRESHOLD))
			{
				mbs_data->flag_stop = 1;

				if (options->print)
				{
					std::cout << "Stopping simulation: ground forces too important !" << std::endl;
				}
			}
			break;

		default:
			std::cout << "Error: unknown GCM model : " << gcm_model << " !" << std::endl;
			exit(EXIT_FAILURE);
	}

	
	// --- Stopping simulation if joints are out of bounds --- //

	limit_joints->check();


	// --- Stopping simulation if nan detected --- //

	for(int i=1; i<=n_joint; i++)
	{
		if (std::isnan(mbs_data->q[i]))
		{
			mbs_data->flag_stop = 1;

			if (options->print)
			{
				std::cout << "Stopping simulation: joint " << simu_index->get_name_mbs_jt(i) << " is nan !" << std::endl;
			}
		}
	}


	/*
	 * Uncomment the following line to remove the possibility to automatically
	 * stop the simulation before the end of the initially scheduled simulation time
	 */
	//mbs_data->flag_stop = 0;
}
