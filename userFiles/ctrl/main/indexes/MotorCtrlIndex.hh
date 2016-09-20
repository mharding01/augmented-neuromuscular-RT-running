/*! 
 * \author Nicolas Van der Noot
 * \file MotorCtrlIndex.hh
 * \brief MotorCtrlIndex class
 */
#ifndef _MOTOR_CTRL_INDEX_HH_
#define _MOTOR_CTRL_INDEX_HH_

#include <vector>
#include <stdio.h>
#include "CtrlOptions.hh"

/*! \brief indexes corresponding to the implemented motors
 *
 * In comparison, "CtrlIndex.hh" provides all the motor indexes, even the ones not used
 */
class MotorCtrlIndex
{
	public:
		MotorCtrlIndex(CtrlOptions *options);
		~MotorCtrlIndex();

		/// get the number of motors
		int get_nb_mot() const { return indexes.size(); }

		/// iterate on the implemented motors
		int get_index(int i) const { return indexes[i]; }

		/// get indexes used by inputs and outputs controller classes
		int get_inv_index(int i) const { return inv_indexes[i]; }

	private:
		std::vector<int> indexes; ///< indexes of the running motors
		std::vector<int> inv_indexes; ///< inverse of the 'indexes' vector

		CtrlOptions *options; ///< controller options

		int index_max; ///< maximal index
};

#endif
