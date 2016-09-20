#include "SimuCtrl.hh"
#include "SensorsInfo.hh"
#include "GaitFeatures.hh"
#include "FitnessRun.hh"
#include "FitnessQq.hh"
#include "StopSimu.hh"
#include "SwingStanceAnalysis.hh"
#include "user_IO.h"
#include "SimuOptions.h"
#include "ContactGestion.hh"
#include "SimuIndex.hh"

#include "user_all_id.h"
#include "cmake_config.h"
#include <fstream>

#define PERIOD_CTRL 1.0e-3  ///< period of the controller [s]
#define TIME_EPSILON 1.0e-5 ///< safety to check the period of the controller [s]

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
SimuCtrl::SimuCtrl(MbsData *mbs_data, Ctrl *ctrl)
{
	int flag_info, flag_features;

	SensorsInfo *sens_info;
	GaitFeatures *gait_features;

	this->ctrl = ctrl;

	this->mbs_data = mbs_data;

	options = mbs_data->user_IO->options;

	gcm_model = options->gcm_model;

	simu_constraint = options->simu_constraint;

	last_t_ctrl = 0.0;

	flag_info     = 0;
	flag_features = 0;

	sens_info = NULL;
	gait_features = NULL;

	simu_index = new ModelSimuIndex(mbs_data, options);

	actuators = new RobotActuators(mbs_data, simu_index);

	for(int i=0; i<SIMU_CTRL_NB; i++)
	{
		switch (i)
		{
			case SENSORS_INFO:
				compute_tab.push_back(new SensorsInfo(mbs_data, simu_index));
				sens_info = static_cast<SensorsInfo*>(compute_tab.back());
				flag_info = 1;
				break;

			case GAIT_FEATURES:
				if (!flag_info)
				{
					std::cout << "Error: SensorsInfo not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new GaitFeatures(mbs_data, simu_index, sens_info));
				gait_features = static_cast<GaitFeatures*>(compute_tab.back());
				flag_features = 1;
				break;

			case STOP_SIMULATION:
				if (!flag_info)
				{
					std::cout << "Error: SensorsInfo not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				compute_tab.push_back(new StopSimu(mbs_data, simu_index, sens_info));
				break;

			case FITNESS:
				if (!flag_info)
				{
					std::cout << "Error: SensorsInfo not initialized !" << std::endl;
					exit(EXIT_FAILURE);
				}
				if (simu_constraint == SIMU_Qq_MATCH_WANG) 
				{
					compute_tab.push_back(new FitnessQq(mbs_data, ctrl));
				}
				else
				{
					compute_tab.push_back(new FitnessRun(mbs_data, ctrl, sens_info));
				}
				break;
		
			default:
				break;
		}
	}
	
	if (!flag_info)
	{
		std::cout << "Error: SensorsInfo not initialized !" << std::endl;
		exit(EXIT_FAILURE);
	}
	if (!flag_features)
	{
		std::cout << "Error: GaitFeatures not initialized !" << std::endl;
		exit(EXIT_FAILURE);
	}

	if (gcm_model == MESH_GCM_MODEL)
	{
		gcm_mesh = new ComputeGCM(mbs_data, gait_features, sens_info);
	}

	links = new LinksRobot(mbs_data, simu_index, options);

	contact_shapes = NULL;
	rfoot_shape = NULL;
	lfoot_shape = NULL;

	dt = mbs_data->dt0;

	if (simu_constraint == SIMU_Qq_MATCH_WANG)
	{
		// position of driven joint    
		std::ifstream ihip(PROJECT_SOURCE_DIR"/../userFiles/dataWang/hip.txt", std::ios::in);
		std::ifstream ihipp(PROJECT_SOURCE_DIR"/../userFiles/dataWang/hipp.txt", std::ios::in);
		std::ifstream ihippp(PROJECT_SOURCE_DIR"/../userFiles/dataWang/hippp.txt", std::ios::in);
		std::ifstream iknee(PROJECT_SOURCE_DIR"/../userFiles/dataWang/knee.txt", std::ios::in);
		std::ifstream ikneep(PROJECT_SOURCE_DIR"/../userFiles/dataWang/kneep.txt", std::ios::in);
		std::ifstream ikneepp(PROJECT_SOURCE_DIR"/../userFiles/dataWang/kneepp.txt", std::ios::in);
		std::ifstream iankle(PROJECT_SOURCE_DIR"/../userFiles/dataWang/ankle.txt", std::ios::in);
		std::ifstream ianklep(PROJECT_SOURCE_DIR"/../userFiles/dataWang/anklep.txt", std::ios::in);
		std::ifstream ianklepp(PROJECT_SOURCE_DIR"/../userFiles/dataWang/anklepp.txt", std::ios::in);
		if (!ihip.is_open() || !ihipp.is_open() || !ihippp.is_open())
		{
	        std::cout << "Error: cant open wang hip files" << std::endl;
	        exit(1);
	    }
		double num = 0.0;
		while (ihip >> num)
			RHipPos.push_back(num);
		num = 0.0;
		while (ihipp >> num)
			RHipVit.push_back(num);
		num = 0.0;
		while (ihippp >> num)
			RHipAcc.push_back(num);
		
		if (!iknee.is_open() || !ikneep.is_open() || !ikneepp.is_open())
		{
	        std::cout << "Error: cant open wang knee files" << std::endl;
	        exit(1);
	    }
	    num = 0.0;
		while (iknee >> num)
			RKneePos.push_back(num);
		while (ikneep >> num)
			RKneeVit.push_back(num);
		num = 0.0;
		while (ikneepp >> num)
			RKneeAcc.push_back(num);

		if (!iankle.is_open() || !ianklep.is_open() || !ianklepp.is_open())
		{
	        std::cout << "Error: cant open wang ankle files" << std::endl;
	        exit(1);
	    }
		num = 0.0;
		while (iankle >> num)
			RAnklePos.push_back(num);
		while (ianklep >> num)
			RAnkleVit.push_back(num);
		num = 0.0;
		while (ianklepp >> num)
			RAnkleAcc.push_back(num);
	}
}

/*! \brief destructor
 */
SimuCtrl::~SimuCtrl()
{
	if (gcm_model == MESH_GCM_MODEL)
	{
		delete gcm_mesh;
	}
	
	delete simu_index;
	delete actuators;
	delete links;

	for(unsigned int i=0; i<compute_tab.size(); i++)
	{
		delete compute_tab[i];
	}
}

/*! \brief initialize the contactGeom parameters
 */
void SimuCtrl::init_contactGeom()
{
	StopSimu *stop_simu;
	SwingStanceAnalysis *swing_stance;
	ContactGeom::RigidShape *cur_rigid;

	contact_shapes = static_cast<ContactGeom::ContactGestion*>(mbs_data->user_IO->contactGestion)->get_main_union();

	if (gcm_model == PRIM_GCM_MODEL)
	{
		for(int i=0; i<contact_shapes->get_rigid_F_size(); i++)
		{
			cur_rigid = contact_shapes->get_rigid_F(i);

			if (cur_rigid->get_Fsens() == simu_index->get_mbs_F(SimuFsensIndex::RightFoot))
			{
				rfoot_shape = cur_rigid;
			}
			else if (cur_rigid->get_Fsens() == simu_index->get_mbs_F(SimuFsensIndex::LeftFoot))
			{
				lfoot_shape = cur_rigid;
			}		
		}

		stop_simu = static_cast<StopSimu*>(compute_tab[STOP_SIMULATION]);
		swing_stance = static_cast<SwingStanceAnalysis*>(static_cast<GaitFeatures*>(compute_tab[GAIT_FEATURES])->get_feature(SWING_STANCE_FEAT));

		stop_simu->set_rfoot_shape(rfoot_shape);
		stop_simu->set_lfoot_shape(lfoot_shape);

		swing_stance->set_rfoot_shape(rfoot_shape);
		swing_stance->set_lfoot_shape(lfoot_shape);
	}
}

/*! \brief computations
 */
void SimuCtrl::compute()
{
	double t = mbs_data->tsim;

	compute_tab[SENSORS_INFO]->compute();

	// controller
	if (t >= last_t_ctrl + PERIOD_CTRL - TIME_EPSILON)
	{
		last_t_ctrl = t;
		
		controller_loop_interface(mbs_data);
	}

	// normal computations
	for(unsigned int i=GAIT_FEATURES; i<compute_tab.size(); i++)
	{
		compute_tab[i]->compute();
	}

	// GCM mesh
	if (gcm_model == MESH_GCM_MODEL)
	{
		gcm_mesh->state_compute();
	}
}

/*! \brief driven variables
 */
void SimuCtrl::compute_driven()
{
	if (simu_constraint == SIMU_Qq_MATCH_WANG)
	{
		double t = mbs_data->tsim;
		double index = t/dt;
		
		// position, speed, accel of hip from Wang data
		mbs_data->q[RHipSag_id]   = RHipPos[index];
		mbs_data->qd[RHipSag_id]  = RHipVit[index];
		mbs_data->qdd[RHipSag_id] = RHipAcc[index];
		// position, speed, accel of knee from Wang data
		mbs_data->q[RKneeSag_id]  = RKneePos[index];
		mbs_data->qd[RKneeSag_id] = RKneeVit[index];
		mbs_data->qdd[RKneeSag_id]= RKneeAcc[index];
		// position, speed, accel of ankle from Wang data
		mbs_data->q[RAnkSag_id]   = RAnklePos[index];
		mbs_data->qd[RAnkSag_id]  = RAnkleVit[index];
		mbs_data->qdd[RAnkSag_id] = RAnkleAcc[index];
	}
}