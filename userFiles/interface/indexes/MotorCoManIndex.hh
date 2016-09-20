/*! 
 * \author Nicolas Van der Noot
 * \file MotorCoManIndex.hh
 * \brief MotorCoManIndex class
 */
#ifndef _MOTOR_COMAN_INDEX_HH_
#define _MOTOR_COMAN_INDEX_HH_

#include <vector>

/*! \brief indexes corresponding to the implemented motors with the real COMAN interface (interface with the controller)
 *
 * In comparison, "CoManIndex.hh" provides all the motor indexes, even the ones not used
 */
class MotorCoManIndex
{
	public:
		MotorCoManIndex(int coman_model);
		~MotorCoManIndex();

		/// get the number of motors
		int get_nb_mot() const { return indexes.size(); }

		/// iterate on the implemented motors
		int get_index(int i) const { return indexes[i]; }

		/// get indexes used by inputs and outputs controller classes
		int get_inv_index(int i) const { return inv_indexes[i]; }

		/// get coman_model
		int get_coman_model() const { return coman_model; }

	private:
		std::vector<int> indexes; ///< indexes of the running motors
		std::vector<int> inv_indexes; ///< inverse of the 'indexes' vector

		int index_max; ///< maximal index
		int coman_model; ///< ID of the COMAN model
};

#endif
