/*! 
 * \author Nicolas Van der Noot
 * \file TrainingGround.hh
 * \brief TrainingGround class
 */

#ifndef _TRAINING_GROUND_HH_
#define _TRAINING_GROUND_HH_

#include "EquationGround.hh"


#define HALF_BASIS 0.02

#define X1 0.005
#define X2 0.035

#define OBS_MAX_H 30e-3 // (before: 10e-3 to 40e-3) 
#define OBST_T_INIT 10.0

#define DENOM (2.0*HALF_BASIS - X2)

/*! \brief specific ground model: flat ground with obstacles next to the stance foot 
 * to increase foot clearance with the ground during the optimization process
 */
class TrainingGround: public EquationGround
{
	public:
		TrainingGround(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info);
		virtual ~TrainingGround();

		virtual void compute();

	private:
		/*! \brief description of the ground for ground equations models
		 * 
		 * \param[in] x x position [m]
		 * \param[in] y y position [m]
		 * \param[out] h height of the ground at point (x,y) [m]
		 * \param[out] hx derivative along x of the height of the ground at point (x,y) [-]
		 * \param[out] hy derivative along y of the height of the ground at point (x,y) [-]
		 */
		inline void ground_description(double x, double y, double *h, double *hx, double *hy)
		{
			double str_x_pos;
			double x_star;
			double x_min, x_max;
			double obs_h;

			if (t < OBST_T_INIT)
			{
				*h  = 0.0;
				*hx = 0.0;
				*hy = 0.0;

				return;
			}

			// right foot
			if (y < 0.0)
			{
				str_x_pos = gait_features->get_x_last_strike_step(LEFT_ID);
			}
			// left foot
			else
			{
				str_x_pos = gait_features->get_x_last_strike_step(RIGHT_ID);
			}
			
			if (str_x_pos > 0.0)
			{
				x_min = str_x_pos - HALF_BASIS;
				x_max = str_x_pos + HALF_BASIS;

				// trapezoidal obstacle
				if ( (x_min < x) && (x < x_max) )
				{
					x_star = x - x_min;

					// height of the obstacle
					obs_h = ( (t - OBST_T_INIT) / (mbs_data->tf - OBST_T_INIT) ) * OBS_MAX_H;

					if (x_star < X1)
					{
						*h  = obs_h * (x_star/X1);
						*hx = obs_h/X1;
					}
					else if (x_star < X2)
					{
						*h  = obs_h;
						*hx = 0.0;
					}
					else
					{
						*h  = obs_h * (1.0 - (x_star - X2)/DENOM);
						*hx = -obs_h/DENOM;
					}
				}
				else // flat ground
				{
					*h  = 0.0;
					*hx = 0.0;
				}			
			}
			else // flat ground
			{
				*h  = 0.0;
				*hx = 0.0;
			}

			*hy = 0.0;
		}
};

#endif
