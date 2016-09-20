#include "MotorCtrlIndex.hh"
#include "CtrlIndex.hh"

#include <iostream>

/*! \brief constructor
 * 
 * \param[in] options controller options
 */
MotorCtrlIndex::MotorCtrlIndex(CtrlOptions *options)
{
	int cur_index;

	this->options = options;

	// right leg
	indexes.push_back(CtrlIndex::RightHipPitch);
	indexes.push_back(CtrlIndex::RightHipRoll);
	indexes.push_back(CtrlIndex::RightHipYaw);
	indexes.push_back(CtrlIndex::RightKneePitch);
	indexes.push_back(CtrlIndex::RightFootRoll);
	indexes.push_back(CtrlIndex::RightFootPitch);

	// left leg
	indexes.push_back(CtrlIndex::LeftHipPitch);
	indexes.push_back(CtrlIndex::LeftHipRoll);
	indexes.push_back(CtrlIndex::LeftHipYaw);
	indexes.push_back(CtrlIndex::LeftKneePitch);
	indexes.push_back(CtrlIndex::LeftFootRoll);
	indexes.push_back(CtrlIndex::LeftFootPitch);

	// torso
	indexes.push_back(CtrlIndex::TorsoRoll);
	indexes.push_back(CtrlIndex::TorsoPitch);
	indexes.push_back(CtrlIndex::TorsoYaw);

	// right arm
	indexes.push_back(CtrlIndex::RightShPitch);
	indexes.push_back(CtrlIndex::RightShRoll);
	indexes.push_back(CtrlIndex::RightShYaw);
	indexes.push_back(CtrlIndex::RightElbPitch);

	// left arm
	indexes.push_back(CtrlIndex::LeftShPitch);
	indexes.push_back(CtrlIndex::LeftShRoll);
	indexes.push_back(CtrlIndex::LeftShYaw);
	indexes.push_back(CtrlIndex::LeftElbPitch);

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
MotorCtrlIndex::~MotorCtrlIndex()
{

}
