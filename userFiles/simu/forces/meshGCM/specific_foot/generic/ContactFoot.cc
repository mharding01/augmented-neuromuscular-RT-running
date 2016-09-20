
#include "ContactFoot.hh"
#include "simu_functions.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait class
 * \param[in] foot_id ID of the foot
 * \param[in] nb_points number of mesh points [-]
 */
ContactFoot::ContactFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id, int nb_points)
{
	double tsim;

	this->foot_id   = foot_id;
	this->nb_points = nb_points;

	this->mbs_data = mbs_data;
	this->gait_features = gait_features;
	this->sens_info = sens_info;

	for (int i=0; i<3; i++)
	{
		P[i] = 0.0;
		V[i] = 0.0;
		OM[i] = 0.0;

		F_tot[i] = 0.0;
		T_tot[i] = 0.0;

		for (int j=0; j<3; j++)
		{
			R[i][j] = 0.0;
		}
	}

	// foot mesh
	rn = (double**) malloc(nb_points*sizeof(double*));
	rs = (double**) malloc(nb_points*sizeof(double*));

	state = (int*) malloc(nb_points*sizeof(int));
	
	Fn_lp = (double*) malloc(nb_points*sizeof(double));
	fx_lp = (double*) malloc(nb_points*sizeof(double));
	fy_lp = (double*) malloc(nb_points*sizeof(double));
	vx_lp = (double*) malloc(nb_points*sizeof(double));
	vy_lp = (double*) malloc(nb_points*sizeof(double));

	for (int i=0; i<nb_points; i++)
	{
		rn[i] = (double*) malloc(3*sizeof(double));
		rs[i] = (double*) malloc(3*sizeof(double));

		for (int j=0; j<3; j++)
		{
			rn[i][j] = 0.0;
			rs[i][j] = 0.0;
		}

		state[i] = SLIDING_GCM;

		Fn_lp[i] = 0.0;
		fx_lp[i] = 0.0;
		fy_lp[i] = 0.0;
		vx_lp[i] = 0.0;
		vy_lp[i] = 0.0;
	}

	// low pass filters
	tsim = mbs_data->tsim;

	low_filters[LOW_FILT_FN] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_FX] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_FY] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_VX] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_VY] = new LowFilterSimu(1.0e-2, tsim);
}

/*! \brief destructor
 */
ContactFoot::~ContactFoot()
{
	for(int i=0; i<NB_LOW_FILT_MESH; i++)
	{
		delete low_filters[i];
	}

	for (int i=0; i<nb_points; i++)
	{
		free(rn[i]);
		free(rs[i]);
	}

	free(rn);
	free(rs);

	free(state);

	free(Fn_lp);
	free(fx_lp);
	free(fy_lp);
	free(vx_lp);
	free(vy_lp);
}

/*! \brief update the pose (position, rotation and derivatives) with the user_ExtForces inputs
 * 
 * \param[in] PxF absolute position (provided by Robotran)
 * \param[in] RxF absolute rotation matrix (provided by Robotran)
 * \param[in] VxF position derivative (provided by Robotran)
 * \param[in] OMxF orientation derivative (provided by Robotran)
 */
void ContactFoot::update_pose(double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4])
{
	for (int i=0; i<3; i++)
	{
		P[i]  = PxF[i+1];
		V[i]  = VxF[i+1];
		OM[i] = OMxF[i+1];

		for (int j=0; j<3; j++)
		{
			R[i][j] = RxF[i+1][j+1];
		}
	}
}

/*! \brief update the pose (position, rotation and derivatives) with the model F sensors
 */
void ContactFoot::update_pose_Fsens()
{
	Fsens_info *f_sens_info;

	f_sens_info = get_F_Sens_body();

	for (int i=0; i<3; i++)
	{
		P[i]  = f_sens_info->P[i];
		V[i]  = f_sens_info->V[i];
		OM[i] = f_sens_info->OM[i];

		for (int j=0; j<3; j++)
		{
			R[i][j] = f_sens_info->R[i][j];
		}
	}
}

/*! \brief update the low-pass filter parameters
 * 
 * \param[in] new_t new time [s]
 */
void ContactFoot::update_low_pass_params(double new_t)
{
	for(int i=0; i<NB_LOW_FILT_MESH; i++)
	{
		low_filters[i]->update_params(mbs_data->tsim);
	}
}
