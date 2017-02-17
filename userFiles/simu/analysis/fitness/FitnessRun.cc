
#include "FitnessRun.hh"
#include "MinDistFitness.hh"
#include "WalkTimeFitness.hh"
#include "CPG_SpeedFitness.hh"
#include "OscillosFitness.hh"
#include "DSFitness.hh"
#include "FlightFitness.hh"
#include "MetEnergyFitness.hh"
#include "FootFitness.hh"
#include "TorsoFitness.hh"
#include "PolyRegL1Fitness.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 * \param[in] sens_info info from the sensor
 */
FitnessRun::FitnessRun(MbsData *mbs_data, Ctrl *ctrl, SensorsInfo *sens_info): Fitness(mbs_data, simu_index, ctrl, sens_info)
{
	if (ctrl->get_ctrl_id() == NICO_CTRL)
	{   
		if (options->speed_opti)
		{   // 700 ...
			// want to still reward staying as close as possible to within 0.025
            CPG_SpeedFitness *cpg_speed_fitness = new CPG_SpeedFitness(mbs_data, ctrl, sens_info);
            speed_fitness = static_cast<SpeedFitness*>(cpg_speed_fitness);
			fitness_tab.push_back(cpg_speed_fitness); 
		}
        fitness_tab.push_back(new OscillosFitness(mbs_data, ctrl));     // 500

		//TODO: TorsoFitness favors no forward lean, needed for faster running
		fitness_tab.push_back(new TorsoFitness(mbs_data, sens_info, ctrl)); // 350 

		// Flat optimization: 
		fitness_tab.push_back(new WalkTimeFitness(mbs_data));           // 300
		fitness_tab.push_back(new MinDistFitness(mbs_data, sens_info)); // 300
		
		fitness_tab.push_back(new MetEnergyFitness(mbs_data, ctrl));    // 200


		// TODO: flightfitness not necessary, doesn't allow speed-walking
		fitness_tab.push_back(new FlightFitness(mbs_data, ctrl));       // 250 
		//TODO: foot fitness favors maximizing total stance time per gait cycle
		//fitness_tab.push_back(new FootFitness(mbs_data, sens_info, ctrl));  // 25
	}
}

/*! \brief destructor
 */
FitnessRun::~FitnessRun()
{
	//already done in mother class
}
