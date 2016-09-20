/*! 
 * \author Nicolas Van der Noot
 * \file SimuCtrl.hh
 * \brief SimuCtrl class
 */

#ifndef _SIMU_CTRL_HH_
#define _SIMU_CTRL_HH_


#include "mbs_data.h"
#include "user_realtime.h"
#include "Ctrl.hh"
#include "ComputationSimu.hh"
#include "ComputeGCM.hh"
#include "ModelSimuIndex.hh"
#include "RobotActuators.hh"
#include "LinksRobot.hh"
#include "MainUnionShape.hh"
#include "RigidShape.hh"

#include <vector>
#include <iostream>

/// simulation control structures
enum{SENSORS_INFO, GAIT_FEATURES, STOP_SIMULATION, FITNESS, SIMU_CTRL_NB};

/*! \brief Main class to handle the controller and the user simulation functions
 */
class SimuCtrl
{
	public:
		SimuCtrl(MbsData *mbs_data, Ctrl *ctrl);
		~SimuCtrl();

		void compute();
		void init_contactGeom();
		void compute_driven();

		ComputationSimu* get_computation(int index) { return compute_tab[index]; }
		ComputeGCM* get_gcm_mesh() { return gcm_mesh; }
		Ctrl* get_ctrl() { return ctrl; }

		RobotActuators* get_actuators() const { return actuators; }
		ModelSimuIndex* get_simu_index() const { return simu_index; }
		LinksRobot* get_links() const { return links; }

		ContactGeom::RigidShape* get_rfoot_shape() { return rfoot_shape; }
		ContactGeom::RigidShape* get_lfoot_shape() { return lfoot_shape; }

	private:
		int gcm_model; ///< 1 for GCM mesh, 0 otherwise
		double last_t_ctrl; ///< last time controller was called [s]

		MbsData *mbs_data; ///< Robotran structure
		Ctrl *ctrl; ///< controller
		ComputeGCM *gcm_mesh; ///< ground contact model

		std::vector<ComputationSimu*> compute_tab; ///< all computations

		ModelSimuIndex *simu_index; ///< indexes for the simulation
		SimuOptions *options; ///< simulation options

		RobotActuators *actuators; ///< actuators implemented in this COMAN model

		LinksRobot *links; ///< links of the robot

		ContactGeom::MainUnionShape *contact_shapes; ///< main union for contact shapes
		ContactGeom::RigidShape *rfoot_shape; ///< right foot shape
		ContactGeom::RigidShape *lfoot_shape; ///< left foot shape

		std::vector<double> RHipPos;
		std::vector<double> RHipVit;
		std::vector<double> RHipAcc;
		std::vector<double> RKneePos;
		std::vector<double> RKneeVit;
		std::vector<double> RKneeAcc;
		std::vector<double> RAnklePos;
		std::vector<double> RAnkleVit;
		std::vector<double> RAnkleAcc;
		double dt;
		int simu_constraint;
};

#endif
