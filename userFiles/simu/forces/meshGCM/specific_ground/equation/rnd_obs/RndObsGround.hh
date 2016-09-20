/*! 
 * \author Nicolas Van der Noot
 * \file RndObsGround.hh
 * \brief RndObsGround class
 */

#ifndef _RND_OBS_GROUND_HH_
#define _RND_OBS_GROUND_HH_

#include "EquationGround.hh"
#include "RotatingListSimu.hh"

/*! \brief specific ground model: flat ground with rnadom obstacles positions
 */
class RndObsGround: public EquationGround
{
	public:
		RndObsGround(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info);
		virtual ~RndObsGround();

		virtual void compute();

		double get_random_dist();
		void init_hardcode();

	private:
		int flag_first;

		double l_obs;
		double h_obs;

		RotatingListSimu x_obs_r;
		RotatingListSimu x_obs_l;

		/*! \brief description of the ground for ground equations models
		 * 
		 * \param[in] x x position [m]
		 * \param[in] y y position [m]
		 * \param[out] h height of the ground at point (x,y) [m]
		 * \param[out] hx derivative along x of the height of the ground at point (x,y) [-]
		 * \param[out] hy derivative along y of the height of the ground at point (x,y) [-]
		 */
		virtual inline void ground_description(double x, double y, double *h, double *hx, double *hy)
		{
			double cur_x_obs, delta_x;
			double arg, pi_l;

			*h = 0.0;
			*hx = 0.0;
			*hy = 0.0;

			if (y > 0.0) ///< left foot
			{
				for(int i=0; i<x_obs_l.get_nb(); i++)
				{
					cur_x_obs = x_obs_l.get_from_last(i);			
					
					delta_x = x - cur_x_obs;

					if (fabs(delta_x) < l_obs)
					{
						pi_l = M_PI / l_obs;
						arg = pi_l * delta_x;

						*h  =  0.5 * h_obs * (1.0 + cos(arg));
						*hx = -0.5 * h_obs * pi_l * sin(arg); 

						break;
					}
				}
			}
			else ///< right foot
			{
				for(int i=0; i<x_obs_r.get_nb(); i++)
				{
					cur_x_obs = x_obs_r.get_from_last(i);
					
					delta_x = x - cur_x_obs;

					if (fabs(delta_x) < l_obs)
					{
						pi_l = M_PI / l_obs;
						arg = pi_l * delta_x;

						*h  =  0.5 * h_obs * (1.0 + cos(arg));
						*hx = -0.5 * h_obs * pi_l * sin(arg); 

						break;
					}
				}
			}
		}
};

#endif
