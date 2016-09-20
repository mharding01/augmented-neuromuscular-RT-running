#include "MotorCoManIndex.hh"
#include "CoManIndex.hh"
#include "SimuOptions.h"

#include <iostream>

/*! \brief constructor
 * 
 * \param[in] coman_model ID of the COMAN model used
 */
MotorCoManIndex::MotorCoManIndex(int coman_model)
{
	int cur_index;

	this->coman_model = coman_model;

	// right leg
	indexes.push_back(CoManIndex::RightHipPitch);
	indexes.push_back(CoManIndex::RightHipRoll);
	indexes.push_back(CoManIndex::RightHipYaw);
	indexes.push_back(CoManIndex::RightKneePitch);
	indexes.push_back(CoManIndex::RightFootRoll);
	indexes.push_back(CoManIndex::RightFootPitch);

	// left leg
	indexes.push_back(CoManIndex::LeftHipPitch);
	indexes.push_back(CoManIndex::LeftHipRoll);
	indexes.push_back(CoManIndex::LeftHipYaw);
	indexes.push_back(CoManIndex::LeftKneePitch);
	indexes.push_back(CoManIndex::LeftFootRoll);
	indexes.push_back(CoManIndex::LeftFootPitch);

	// torso
	indexes.push_back(CoManIndex::TorsoRoll);
	indexes.push_back(CoManIndex::TorsoPitch);
	indexes.push_back(CoManIndex::TorsoYaw);

	// right arm
	indexes.push_back(CoManIndex::RightShPitch);
	indexes.push_back(CoManIndex::RightShRoll);
	indexes.push_back(CoManIndex::RightShYaw);
	indexes.push_back(CoManIndex::RightElbPitch);

	// left arm
	indexes.push_back(CoManIndex::LeftShPitch);
	indexes.push_back(CoManIndex::LeftShRoll);
	indexes.push_back(CoManIndex::LeftShYaw);
	indexes.push_back(CoManIndex::LeftElbPitch);

	// get maximal index
	index_max = -1;

	for(unsigned int i=0; i<indexes.size(); i++)
	{
		cur_index = indexes[i];

		if (cur_index > index_max)
		{
			index_max = cur_index;
		}
	}

	for(int i=0; i<=index_max; i++)
	{
		inv_indexes.push_back(-1);
	}

	for(unsigned int i=0; i<indexes.size(); i++)
	{
		cur_index = indexes[i];
		inv_indexes[cur_index] = i;
	}
}

/*! \brief destructor
 */
MotorCoManIndex::~MotorCoManIndex()
{

}
