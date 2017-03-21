
#include "SwingStanceAnalysis.hh"
#include "CppInterface.hh"
#include "WholeFeet.hh"
#include "user_model.h"
#include "user_IO.h"
#include "ContactGestion.hh"
#include "user_IO.h"
#include "user_realtime.h"

#define TIME_SAFETY 0.1             ///< time safety (for strike) [s]
#define TIME_SAFETY_2 0.05          ///< time safety (for take-off) [s]
#define TIME_TAKE_OFF_SAFETY 2.5e-3 ///< time safety to wait after take-off [s]
#define FZ_THRESHOLD_STRIKE 5.0   ///< normal force threshold to detect strike [N]
#define THRESHOLD_X_OBSTACLE 0.15 ///< minimal distance between two obstacles [m]

#define MIN_T_MEAN 9.0 ///< start time to compute stride periods and lengths
#define MAX_T_MEAN 29.0 ///< finish time to compute stride periods and lengths

#define MIN_DIST_STEP 0.1 ///< minimal distance between two steps

#define THRESHOLD_ANGLE_STRIKE 0.6 ///< threshold for the foot angle at strike

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
SwingStanceAnalysis::SwingStanceAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info):
	FeatureAnalysis(mbs_data, simu_index)
{
	this->sens_info = sens_info;

	um = mbs_data->user_model;

	for(int i=0; i<N_LEGS; i++)
	{
		swing_leg[i] = 0;

		last_t_strike_leg[i]   = 0.0;
		last_t_take_off_leg[i] = 0.0;

		x_last_strike[i] = 0.0;
		stride_length[i] = 0.0;
		stride_period[i] = 0.0;
		x_last_strike_step[i] = 0.0;
		t_last_force_leg[i] = 0.0;

		flag_strike_leg[i] = 0;

		take_off_pourc[i] = 0.0;
	}

	nb_steps = 0;

	flag_strike = 0;

	supporting_r_leg = 0;
	cur_obstacle_r   = 0;

	step_length    = 0.0;
	cur_x_obstacle = 0.0;

	stride_period_mean = 0.0;
	stride_length_mean = 0.0;
	ds_cycle_mean = 0.0;
	flight_cycle_mean = 0.0;
	take_off_mean = 0.0;

	ds_time = 0.0;
	flight_time = 0.0;

	str_period_av = new AverageIncrement();
	str_length_av = new AverageIncrement();
	ds_cycle_av = new AverageIncrement();
	flight_cycle_av = new AverageIncrement();
	take_off_av = new AverageIncrement();

	flag_print = 0;

	dt = mbs_data->dt0;
	options = mbs_data->user_IO->options;
	gcm_model = options->gcm_model;
}

/*! \brief destructor
 */
SwingStanceAnalysis::~SwingStanceAnalysis()
{
	delete str_period_av;
	delete str_length_av;
	delete ds_cycle_av;
	delete flight_cycle_av;
	delete take_off_av;
}

/*! \brief compute this gait feature
 */
void SwingStanceAnalysis::compute()
{
	double x_foot, t;
	double Fz[N_LEGS];
	double angle_foot;
	double gh_t_flight, flight_per_cycle;


	CppInterface *cppInterface;

	WholeFeet *whole_feet;

	t = mbs_data->tsim;

	cppInterface = static_cast<CppInterface*>(um->cppInterface);

	switch (gcm_model)
	{
		case NO_GCM_MODEL:
			Fz[RIGHT_ID] = 0.0;
			Fz[LEFT_ID]  = 0.0;
			break;

		case MESH_GCM_MODEL:
			whole_feet = cppInterface->get_simu_ctrl()->get_gcm_mesh()->get_whole_feet();

			Fz[RIGHT_ID] = whole_feet->get_leg_feet_forces(RIGHT_ID, 2);
			Fz[LEFT_ID]  = whole_feet->get_leg_feet_forces(LEFT_ID, 2);
			break;
	
		case PRIM_GCM_MODEL:
			Fz[RIGHT_ID] = rfoot_shape->get_F_tot_z();
			Fz[LEFT_ID]  = lfoot_shape->get_F_tot_z();
			break;

		default:
			std::cout << "Error: unknown GCM model : " << gcm_model << " !" << std::endl;
			exit(EXIT_FAILURE);
	}

	flag_strike = 0;

	flag_strike = 0;
	
	for (int i=0; i<N_LEGS; i++)
	{
		flag_strike_leg[i] = 0;	
	}

	for(int i=0; i<N_LEGS; i++)
	{
		// swing
		if (swing_leg[i])
		{
			if (i == RIGHT_ID)
			{
				angle_foot = atan2(sens_info->get_S_RFoots_R(6), sqrt(sens_info->get_S_RFoots_R(7)*sens_info->get_S_RFoots_R(7) + sens_info->get_S_RFoots_R(8)*sens_info->get_S_RFoots_R(8)));
			}
			else
			{
				angle_foot = atan2(sens_info->get_S_LFoots_R(6), sqrt(sens_info->get_S_LFoots_R(7)*sens_info->get_S_LFoots_R(7) + sens_info->get_S_LFoots_R(8)*sens_info->get_S_LFoots_R(8)));
			}

			// strike
			if ( (Fz[i] > FZ_THRESHOLD_STRIKE)  && (t - last_t_take_off_leg[i] > TIME_SAFETY) && (angle_foot < THRESHOLD_ANGLE_STRIKE) )
			{
				swing_leg[i] = 0;

				flag_strike = 1;

				if (i == RIGHT_ID)
				{
					supporting_r_leg = 1;
					flag_strike_leg[RIGHT_ID] = 1;
				}
				else
				{
					supporting_r_leg = 0;
					flag_strike_leg[LEFT_ID] = 1;
				}

				stride_period[i] = t - last_t_strike_leg[i];

				take_off_pourc[i] = 100.0*(last_t_take_off_leg[i] - last_t_strike_leg[i])/stride_period[i];

				last_t_strike_leg[i] = t;

				// right foot x position [m]
				if (i == RIGHT_ID)
				{
					x_foot = sens_info->get_S_RFoots_P(0);
				}
				else
				{
					x_foot = sens_info->get_S_LFoots_P(0);
				}	

				step_length      = x_foot - x_last_strike[!i];
				stride_length[i] = x_foot - x_last_strike[i];
				x_last_strike[i] = x_foot;

				if (fabs(x_foot - x_last_strike_step[!i]) > MIN_DIST_STEP)
				{
					x_last_strike_step[i] = x_foot;
				}

				// obstacle position
				if (x_foot - cur_x_obstacle > THRESHOLD_X_OBSTACLE)
				{
					cur_x_obstacle = x_foot;

					if (i == RIGHT_ID)
					{
						cur_obstacle_r = 0;
					}
					else
					{
						cur_obstacle_r = 1;
					}
				}

				// mean values
				if ( (MIN_T_MEAN < t) && (t < MAX_T_MEAN) )
				{
					stride_period_mean = str_period_av->update_and_get(stride_period[i]);
					stride_length_mean = str_length_av->update_and_get(stride_length[i]);
					// Stance-time of leg i, only
					take_off_mean = take_off_av->update_and_get(take_off_pourc[i]);
					if (i == RIGHT_ID) 
					{
						ds_cycle_mean = ds_cycle_av->update_and_get(100*ds_time/stride_period[i]);
						// Flight total time over one gait cycle
						flight_cycle_mean = flight_cycle_av->update_and_get(100*flight_time/stride_period[i]);
					}
				}

				if ( (nb_steps < 3) || (fabs(x_last_strike[RIGHT_ID] - x_last_strike[LEFT_ID]) > MIN_DIST_STEP) )
				{
					nb_steps++;
				}

				if (i == RIGHT_ID) //reset of ds_time and flight_time at each right foot strike
				{
					ds_time = 0.0;
					flight_time = 0.0;
				}
			}	
		}
		// stance
		else
		{
			if (Fz[i] >= FZ_THRESHOLD_STRIKE)
			{
				t_last_force_leg[i] = t;
			}

			// take-off
			if ( (Fz[i] <= 0.0) && (t - last_t_strike_leg[i] > TIME_SAFETY_2) && (t - t_last_force_leg[i] > TIME_TAKE_OFF_SAFETY) )
			{
				swing_leg[i] = 1;

				last_t_take_off_leg[i] = t;

				if (i == RIGHT_ID)
				{
					supporting_r_leg = 0;
				}
				else
				{
					supporting_r_leg = 1;
				}
			}
		}
	}

	if ( ((MIN_T_MEAN - 5.0) < t) && (t < (MAX_T_MEAN)) ) 
	{
		if(!swing_leg[0] && !swing_leg[1]) //computation double support time 
		{
			ds_time += dt;
		}
		else if(swing_leg[0] && swing_leg[1]) //computation flight time 
		{
			flight_time += dt;
		}
	}	

	if (options->print && !flag_print && mbs_data->tsim > 19.5)
	{
		flag_print = 1;

		std::cout << "period: " << stride_period_mean << " [s]" << std::endl;
		std::cout << "length: " << stride_length_mean << " [m]" <<std::endl;
		std::cout << "one leg stance phase: " << take_off_mean << " [%]" <<std::endl;
		std::cout << "double support: " << ds_cycle_mean << " [%]" <<std::endl;
		std::cout << "tot flight: " << flight_cycle_mean << " [%]" <<std::endl;
	}
}
