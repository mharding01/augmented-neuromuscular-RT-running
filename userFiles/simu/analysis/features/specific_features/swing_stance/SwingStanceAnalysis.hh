/*! 
 * \author Nicolas Van der Noot
 * \file SwingStanceAnalysis.hh
 * \brief SwingStanceAnalysis class
 */

#ifndef _SWING_STANCE_ANALYSIS_HH_
#define _SWING_STANCE_ANALYSIS_HH_

#include "FeatureAnalysis.hh"
#include "SensorsInfo.hh"
#include "AverageIncrement.hh"
#include "SimuOptions.h"
#include "RigidShape.hh"

/*! \brief Analysis of the events happening at swing and stance switch (strike and toe push-off)
 */
class SwingStanceAnalysis: public FeatureAnalysis
{
	public:
		SwingStanceAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info);
		virtual ~SwingStanceAnalysis();

		virtual void compute();

		double get_x_last_strike_step(int foot_id) { return x_last_strike_step[foot_id]; }
		int get_nb_steps() const { return nb_steps; }
		int get_flag_strike() const { return flag_strike; }
		int get_flag_strike_leg(int p) const { return flag_strike_leg[p]; }

		void set_rfoot_shape(ContactGeom::RigidShape *r_foot) { rfoot_shape = r_foot; }
		void set_lfoot_shape(ContactGeom::RigidShape *l_foot) { lfoot_shape = l_foot; }

		double get_stride_period_mean() const { return stride_period_mean; }
		double get_stride_length_mean() const { return stride_length_mean; }
		double get_take_off_mean() const { return take_off_mean; }
		double get_ds_cycle_mean() const { return ds_cycle_mean; }
		double get_flight_cycle_mean() const { return flight_cycle_mean; }

	private:
		SensorsInfo *sens_info; ///< info from the sensors

		UserModel *um; ///< user model

		SimuOptions *options; ///< simulation options

		AverageIncrement *str_period_av; ///< average for the stride period
		AverageIncrement *str_length_av; ///< average for the stride length
		AverageIncrement *ds_cycle_av; ///< average for the double support percent per cycle
		AverageIncrement *flight_cycle_av; ///< average for the flight phase percent per cycle
		AverageIncrement *take_off_av; ///< average for starting take-off

		int cur_obstacle_r;   ///< 1 if current obstacle is for right leg, 0 otherwise
		int supporting_r_leg; ///< 1 if current supporting leg is the right one, 0 otherwise

		int gcm_model; ///< ground contact model

		int flag_print; ///< 1 if already printed, 0 otherwise

		int nb_steps; ///< number of steps performed [-]

		int swing_leg[N_LEGS]; ///< 1 if corresponding leg in swing, 0 othewise

		int flag_strike; ///< 1 if strike just happened (one iteration)
		int flag_strike_leg[N_LEGS]; ///< 1 if strike just happened for the corresponding leg (one iteration)

		double step_length;        ///< step length [m]
		double cur_x_obstacle;     ///< current x position for teh obstacle [m]
		double stride_period_mean; ///< mean of the stride period [s]
		double stride_length_mean; ///< mean of the stride length [m]
		double ds_cycle_mean;      ///< mean of the double support per cycle [%]
		double flight_cycle_mean;  ///< mean of the flight phase per cycle [%]
		double take_off_mean;      ///< mean for starting take off [%]

		double ds_time; ///< double support time per cycle [s]
		double flight_time; ///< flight time per cycle [s]
		double take_off_pourc[N_LEGS]; ///< starting take off [%]

		double dt; ///< time step of simulation [s]

		double last_t_strike_leg[N_LEGS];   ///< last time strike for the corresponding leg [s]
		double last_t_take_off_leg[N_LEGS]; ///< last time take-off for the corresponding leg [s]
		double x_last_strike[N_LEGS];       ///< last strike position for the corresponding leg [m]
		double x_last_strike_step[N_LEGS];  ///< last strike position for the corresponding leg (with safety) [m]
		double stride_period[N_LEGS];	    ///< stride period for the corresponding leg [s]	
		double stride_length[N_LEGS];       ///< stride length for the corresponding leg [s]
		double t_last_force_leg[N_LEGS];    ///< last time huge force applied for the corresponding leg [s]

		ContactGeom::RigidShape *rfoot_shape; ///< right foot shape
		ContactGeom::RigidShape *lfoot_shape; ///< left foot shape
};

#endif
