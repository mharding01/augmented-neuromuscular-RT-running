
#include "DelayManager.hh"
#include <iostream>

/*! \brief constructor
 */
DelayManager::DelayManager()
{
	t = 0.0;
}

/*! \brief destructor
 */
DelayManager::~DelayManager()
{
	for (unsigned int i=0; i<delay_tab.size(); i++)
	{
		delete delay_tab[i];
	}
}
