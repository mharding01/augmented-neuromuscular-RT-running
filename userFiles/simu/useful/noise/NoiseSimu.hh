/*! 
 * \author Nicolas Van der Noot
 * \file NoiseSimu.hh
 * \brief NoiseSimu class
 */

#ifndef _NOISE_SIMU_HH_
#define _NOISE_SIMU_HH_

#include "ctrl_functions.hh"

/*! \brief NoiseSimu of a signal signal with linear interpolation
 */
class NoiseSimu
{
	public:
		NoiseSimu(double t, double max_noise, double t_period);
		~NoiseSimu();

		void update(double t);

		/*! \brief get current noise level
		 * 
		 * \return current noise value, bounded in [-max_noise ; max_noise]
		 */
		inline double get()
		{
			return cur_n;
		}

		/*! \brief update and get current noise level
		 * 
		 * \param[in] t current time [s]
		 * \return current noise value, bounded in [-max_noise ; max_noise]
		 */
		inline double update_and_get(double t)
		{
			update(t);
			return get();
		}

		/*! \brief get new noise value
		 * 
		 * \return new random value, bounded in [-max_noise ; max_noise]
		 */
		inline double get_new_noise()
		{
			return (rnd() - 0.5) * (2.0 * max_noise);
		}

	private:
		double last_n; ///< last noise node
		double next_n; ///< next noise node
		double cur_n;  ///< current noise level

		double last_t; ///< time corresponding to last_n node [s]

		double max_noise; ///< noise maximum noise amplitude (noise bounded in [-max_noise ; max_noise])
		double t_period;  ///< period of the noise before computing a new node [s]
};

#endif
