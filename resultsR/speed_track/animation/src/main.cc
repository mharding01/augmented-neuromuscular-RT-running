/*! 
 * \author Nicolas Van der Noot
 * \file main.cc
 * \brief file with the main function
 */
#include "MbsWorld3D.hh"
#include "cmake_config.h"
#include "AnimReader.hh"
#include <iostream>

#define NB_JOINTS_COMAN 31 ///< number of joints (COMAN)

/*! \brief main function
 */
int main(int argc, char **argv)
{
	// variables declaration
	double tsim;
	double *joint_vec;
	OpenGLMbs::AnimReader *reader;

	// main window for 3D world
	OpenGLMbs::MbsWorld3D world_3d(1024, 768);

	// load mbs files
	world_3d.AddMbs(PROJECT_SOURCE_DIR"/dataR/coman_spring_toe_short_feet.mbs");

	// read animation
	reader = new OpenGLMbs::AnimReader(PROJECT_SOURCE_DIR"/animationR/dirdyn_q.anim");

	// initialize joints vectors
	joint_vec = new double[NB_JOINTS_COMAN];

	// loop
	do
	{
		// update the timing
		world_3d.UpdateTime();

		tsim = world_3d.GetT() - 5.0;

		// fill Q values by reading the anim
		reader->FillQ(tsim, NB_JOINTS_COMAN, joint_vec);

		// update joints
		world_3d.UpdateJoints(0, NB_JOINTS_COMAN, joint_vec);

		// update the window
		world_3d.Update();
	}
	while( world_3d.CheckClose() ); // check if the ESC key was pressed or the world_3d was closed

	// release the joint vector
	delete reader;
	delete[] joint_vec;

	return 0;
}
