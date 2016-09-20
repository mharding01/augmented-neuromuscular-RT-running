/*! 
 * \author Nicolas Van der Noot
 * \file Actuator.hh
 * \brief Actuator class
 */
#ifndef _ACTUATOR_HH_
#define _ACTUATOR_HH_

#include "mbs_data.h"
#include "ModelSimuIndex.hh"
#include "NoiseSimu.hh"
#include "SimuOptions.h"

/*! \brief actuator of the COMAN
 */
class Actuator
{
	public:
		Actuator(MbsData *mbs_data, ModelSimuIndex *simu_index, int index, double K_s, double D_s, double J, double D, double V_T, double Aco, double Bco);
		~Actuator();

		void set_pos_vel(double q_m, double qd_m);
		void set_stiffness_damping(double P_stiffness, double D_damping);
		void set_PI_gains(double P, double I);

		void compute_acceleration();
		void compute_torque();

		double get_qdd_m() const { return qdd_m; }
		double get_Qq() const { return Qq; }

		int get_mbs_index() const { return mbs_index; }
		int get_simu_index() const { return index; }

		void low_level_controller();

		void set_voltage(double voltage) { u = voltage; }
		void set_q_ref(double value) { q_ref = value; }
		void set_Qq_ref(double value) { Qq_ref = value; }

	private:
		MbsData *mbs_data; ///< Robotran strucure

		NoiseSimu *torque_noise; ///< noise on the torque reading

		SimuOptions *options; ///< simulation options

		double t_switch; ///< swithcing time bewteen controller
		int torque_noisee; 
		int ctrl_two_parts;

		int mbs_index;  ///< index of the joint as descrbed in the MBS file
		int index; ///< index of the joint (simulation list)

		// joint kinematics
		double q;  ///< joint position [rad]
		double qd; ///< joint velocity [rad/s]

		// motor kinematics
		double q_m;   ///< motor position  [rad]
		double qd_m;  ///< motor velocity [rad/s]
		double qdd_m; ///< motor acceleration [rad/s^2]

		double Qq; ///< motor torque
	
		// motor characteristics
		double K_s;   ///< passive stiffness [Nm/rad]
		double D_s;   ///< passive damping [Nm*s/rad]
		double J;     ///< motor inertia [kg*m^2]
		double D;     ///< motor back EMF constant and the rotor friction [Nm*s/rad]
		double V_T;   ///< voltage to torque ratio [Nm/V]
		double J_inv; ///< inverse of J [1/(kg*m^2)]

		double u; ///< voltage [V]

		// friction model
		double Aco; ///< friction model step factor [Nm]
		double Bco; ///< friction model proportional factor [Nm*s/rad]

		// low-level impedance controller
		double P_stiffness;  ///< position proportional gain = stiffness [Nm/rad]
		double D_damping;    ///< position derivative gain = damping [(Nm s)/rad]
		double P;  ///< torque proportional gain [V/Nm]
		double I;  ///< torque integral gain [V/(Nm s)]

		double q_ref;  ///< position reference [rad]
		double Qq_ref; ///< torque reference [Nm]

		double old_err_Qq; ///< last torque error [Nm]
		double int_err_Qq; ///< integral of the torque error [Nm s]
		double last_t_err_Qq; ///< last time the torque error was computed
};

#endif
