/*! 
 * \author Nicolas Van der Noot
 * \file RobotActuators.hh
 * \brief RobotActuators class
 */

#ifndef _ROBOT_ACTUATORS_HH_
#define _ROBOT_ACTUATORS_HH_

#include "Actuator.hh"
#include "ModelSimuIndex.hh"

#include <vector>

/*! \brief actuators implemented in this COMAN model
 */
class RobotActuators
{
	public:
		RobotActuators(MbsData *mbs_data, ModelSimuIndex *simu_index);
		~RobotActuators();

		void motor_compute_der(double *ux, double *uxd);
		void motor_compute_tor(double *ux, double *uxd, double *Qq);

		int get_nb_actuators() const { return actuators_list.size(); }

		int get_nb_mot() const { return nb_mot; }

		int get_inv_actuators_index(int i) const { return inv_actuators_index[i]; }

		Actuator* get_actuator(int i) const { return actuators_list[i]; }
		Actuator* get_actuator_from_index(int index) const { return actuators_list[get_inv_actuators_index(index)]; }

	private:
		int nb_mot;    ///< number of motors
		int index_max; ///< maximal motor index

		ModelSimuIndex *simu_index; ///< indexes of the simulation

		std::vector<Actuator*> actuators_list; ///< list of actuators implemented in the current COMAN model
		std::vector<int> inv_actuators_index;  ///< inverted list of actuators_list simulation indexes

		MbsData *mbs_data; ///< Robotran structure

		SimuOptions *options; ///< simulation options
};

#endif
