/*! 
 * \author Nicolas Van der Noot
 * \file AverageInc.hh
 * \brief AverageInc class
 */

#ifndef _AVERAGE_INC_HH_
#define _AVERAGE_INC_HH_

/*! \brief Compute the average by incrementing new values (controller version)
 */
class AverageInc
{
	public:
		AverageInc();
		~AverageInc();

		void update(double new_value);

		/*! \brief return current average value
		 */
		double get()
		{
			return mean_value;
		}

		/*! \brief update with a new value and return the current average
		 * 
		 * \param[in] new_value new value to add to this average
		 */
		double update_and_get(double new_value)
		{
			update(new_value);

			return get();
		}

	private:
		int mean_count;    ///< counter of values added
		double mean_value; ///< current mean value
};

#endif
