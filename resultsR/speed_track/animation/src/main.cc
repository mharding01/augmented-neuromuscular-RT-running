/*! 
 * \author Nicolas Van der Noot
 * \file main.cc
 * \brief file with the main function
 */
#include "MbsWorld3D.hh"
#include "cmake_config.h"
#include "AnimReader.hh"
#include "shader_multisample.hh"
#include <iostream>

#define NB_JOINTS_COMAN 31 ///< number of joints (COMAN)

/*! \brief get the time for the video, depending on the real time
 * 
 * This function slows down the video to get a better recording (later speed up to get normal speed)
 *
 * \param[in] t real time [s]
 * \return video time [s]
 */
double time_video_slow_record(double t)
{
	return (t - 5.0) / 16.0;
}

/*! \brief main function
 */
int main(int argc, char **argv)
{
	// variables declaration
	double tsim;
	double *joint_vec;
	int shader_flag, multi_samp;
	OpenGLMbs::AnimReader *reader;

    // get the shader flag and the mutlisampling option from .mbs file
    OpenGLMbs::shader_multisample(PROJECT_SOURCE_DIR"/dataR/coman_spring_toe_short_feet.mbs", shader_flag, multi_samp);

	// main window for 3D world
	//OpenGLMbs::MbsWorld3D world_3d(1024, 768, shader_flag, multi_samp);
	OpenGLMbs::MbsWorld3D world_3d(1280, 720, shader_flag, multi_samp);

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

		tsim = time_video_slow_record(world_3d.GetT());

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
