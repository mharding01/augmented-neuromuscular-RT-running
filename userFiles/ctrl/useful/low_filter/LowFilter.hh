/*! 
 * \author Nicolas Van der Noot
 * \file LowFilter.hh
 * \brief LowFilter class
 */

#ifndef _LOW_FILTER_HH_
#define _LOW_FILTER_HH_

/*! \brief First order low pass filter
 */
class LowFilter
{
	public:
		LowFilter(double tau, double x, double cur_t);
		~LowFilter();

		void update(double x, double cur_t);

		/*! \brief get output
		 * 
		 * \return current output
		 */
		double get()
		{
			return y;
		}

		/*! \brief update and get output
		 * 
		 * \param[in] x new value
		 * \param[in] cur_t current time [s]
		 * \return current output
		 */
		double update_and_get(double x, double cur_t)
		{
			update(x, cur_t);
			
			return get();
		}

	private:
		double y;      ///< output value
		double last_t; ///< last time value added
		double tau;    ///< time constant [s]
};

#endif
