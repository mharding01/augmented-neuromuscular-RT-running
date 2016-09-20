/*! 
 * \author Nicolas Van der Noot
 * \file RotatingListSimu.hh
 * \brief RotatingListSimu class
 */

#ifndef _ROTATING_LIST_SIMU_HH_
#define _ROTATING_LIST_SIMU_HH_

#include <vector>

/*! \brief Rotating list
 */
class RotatingListSimu
{
	public:
		RotatingListSimu(int size_list);
		RotatingListSimu(int size_list, double init_val);
		virtual ~RotatingListSimu();

		void add_value(double val);
		double get_from_last(int p);
		void set_all_values(double val);

		/// get the size of the list
		inline int get_nb() const { return list.size(); }

		/// get the last element added
		inline double get_last() const { return list[last_index]; }

	private:
		std::vector<double> list; ///< list of all elements

		int next_index; ///< next index
		int last_index; ///< last index
};

#endif
