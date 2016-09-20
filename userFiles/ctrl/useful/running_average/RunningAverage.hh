/*! 
 * \author Nicolas Van der Noot
 * \file RunningAverage.hh
 * \brief RunningAverage class
 */

#ifndef _RUNNING_AVERAGE_HH_
#define _RUNNING_AVERAGE_HH_

/*! \brief Signal running average computation
 */
class RunningAverage
{
	public:
		RunningAverage(int size_vec, double init_value);
		~RunningAverage();

		void new_value(double new_val);

		/*! \brief get the current average
		 * 
		 * \return current average
		 */
		double get_average()
		{
			return sum / size_vec;
		}

		/*! \brief update and get the current average
		 * 
		 * \param[in] new_val new value to add
		 * \return current average
		 */
		double update_and_get(double new_val)
		{
			new_value(new_val);

			return get_average();
		}

	private:
		double *vec;  ///< vector with values
		int index;    ///< index in the vector
		int size_vec; ///< size of the vector
		double sum;   ///< sum of the vector
};

#endif
