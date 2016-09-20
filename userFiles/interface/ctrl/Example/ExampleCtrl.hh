/*! 
 * \author Nicolas Van der Noot
 * \file ExampleCtrl.hh
 * \brief ExampleCtrl class
 */

#ifndef _EXAMPLE_CTRL_HH_
#define _EXAMPLE_CTRL_HH_	

#include "Ctrl.hh"
#include "controller_io.hh"
#include <vector>

/*! \brief Ctrl example to present the interface
 */
class ExampleCtrl: public Ctrl
{
	public:
		ExampleCtrl();
		virtual ~ExampleCtrl();

		virtual void init_ctrl();
		virtual void loop_ctrl();
		virtual void finish_ctrl();

		virtual void set_inputs(Inputs_ctrl *inputsCtrl);
		virtual void set_outputs(Outputs_ctrl *outputsCtrl);

	private:
		int nb_mot; ///< number of motors
		int torque_ctrl; ///< 1 for torque control, 0 fot position control

		double kp_tor; ///< proportional gain for the torque reference

		// time input
		double t; ///< time [s]

		// joint inputs
		std::vector<double> q;  ///< joint position [rad]
		std::vector<double> qd; ///< joint velocity [rad/s]
		std::vector<double> Qq; ///< joint torque [Nm]
		std::vector<double> q_mot;  ///< motor position [rad]
		std::vector<double> qd_mot; ///< motor velocity [rad/s]

		// ground contact forces inputs
		std::vector<double> F_Rfoot; ///< right foot force [N]
		std::vector<double> F_Lfoot; ///< left foot force [N] 
		std::vector<double> T_Rfoot; ///< right foot torque [Nm]
		std::vector<double> T_Lfoot; ///< left foot torque [Nm]

		// IMU inputs
		std::vector<double> IMU_Angular_Rate; ///< IMU angular rate [rad/s]
		std::vector<double> IMU_Acceleration; ///< IMU angular acceleration [rad/s^2]
		std::vector<double> IMU_Orientation;  ///< IMU orientation matrix [-]

		// joints outputs
		std::vector<double> q_ref;  ///< joint position reference [rad]
		std::vector<double> Qq_ref; ///< joint torque reference [Nm]
};

#endif
