/*! 
 * \author Nicolas Van der Noot
 * \file DelayManager.hh
 * \brief DelayManager class
 */

#ifndef _DELAY_MANAGER_HH_
#define _DELAY_MANAGER_HH_

#include "Delay.hh"
#include <vector>

/*! \brief Manage delays on signals
 */
class DelayManager
{
	public:
		DelayManager();
		virtual ~DelayManager();

		/*! \brief update and get new value
		 * 
		 * \param[in] index index of the tabular
		 * \param[in] cur_val current new value
		 * \return current delayed value
		 */
		double update_and_get(int index, double cur_val)
		{
			return delay_tab[index]->update_and_get(t, cur_val);
		}

		/*! \brief get new value
		 * 
		 * \param[in] index index of the tabular
		 * \return current delayed value
		 */
		double get(int index)
		{
			return delay_tab[index]->get_value_delay(t);
		}

		/*! \brief update new value
		 * 
		 * \param[in] index index of the tabular
		 * \param[in] cur_val current new value
		 */
		void update(int index, double cur_val)
		{
			delay_tab[index]->new_value_time(t, cur_val);
		}

		/*! \brief set time
		 * 
		 * \param[in] value new value for time [s]
		 */
		void set_t(double value)
		{
			t = value;
		}

	protected:
		std::vector<Delay*> delay_tab; ///< delay tabular

		double t; ///< time [s]
};

#endif
