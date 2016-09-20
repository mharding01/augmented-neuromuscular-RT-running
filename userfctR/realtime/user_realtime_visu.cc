/*!
 * \author Nicolas Van der Noot
 * \file user_realtime_visu.c
 * \brief Java visualization configuration
 * 
 * \attention xxx
 * \pre xxx
 * \param[in/out/in,out] name xxx
 * \return xxx
 *
 * In order to use C++ features, you just need to change the extension of this file (.c) to .cc (or .cpp).
 */

#ifdef JAVA

#include "user_realtime.h"

/*! \brief Java visualization user configuration
 * 
 * \param[in] mbs_data Robotran main structure
 * \param[in] nb_q number of joints in the .mbs file
 * \param[out] q_vec vector to fill with the positions
 *
 * Fill q_vec[i] with the corresponding values for the joints ([rad] for rotational joints,
 *      [m] for translational joints). i-1 is the index of the joint indicated in the .mbs file
 *      used for the visualization (pay attention to the -1 !). If you want to know the number
 *      of joints to fill, print 'nb_q'. Do not use indexes below '0' or above 'nb_q-1'.
 *
 * This file is initially configured to work with the .mbs file located in dataR.
 * Modifying this file is only useful if you want to display something else than
 * the model used to compute the dynamics with Robotran.
 * For instance, it can be used to display extra bodies (like moving arrows)
 * on top of the ones used by Robotran dynamics.
 * In this case, create a new .mbs file (on top of the one in dataR) only used for visualization.
 * Place it somewhere in your project and indicate its path in 'user_realtime_options.c' as
 * 'options->mbs_file = NEW_PATH_WITH_MBS_FILE;'. You must also adapt 'options->nb_q'.
 * Finally, modify the 'user_realtime_visu' function accordingly.
 */
void user_realtime_visu(MbsData* mbs_data, int nb_q, double *q_vec)
{
	int i;

	for(i=1; i<=nb_q; i++)
	{
		q_vec[i-1] = mbs_data->q[i];
	}
}

#endif
