//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°37
//
#ifndef mbs_data_h
#define mbs_data_h
/*--------------------*/

#include "stdio.h"
#include "stdlib.h"

#include "mbs_user_interface.h"

typedef struct MbsData
{
    // MATLAB FIELDS //

    // Données géométriques et dynamiques //
    int npt; ///< Number of anchor points.
    double *dpt[3+1]; ///< Array containing the coordinate of all anchor points.
    double *l[3+1]; ///< Array containing the centre of mass coordinates (in the body fixed frame, one column per body)
    double *m; ///< Array containing the mass of each body
    double *In[9+1]; ///< Array containing the inertia tensor component of each body (in the body fixed frame, relative to the center of mass). 1 column containing 9 rows for each body as follow: [I11;I12;I13;I21;I22;I23;I31;I32;I33]
    double g[3+1]; ///< The 3 gravity components

    int nbody, njoint;

    // Infos partitionnement //
    int nqu, nqc, nqlocked, nqdriven, nqa, nqv, nhu; // JFC : 15/01/2008 : ajout de nhu
    int *qu, *qc, *qlocked, *qdriven, *qa, *qv, *hu; // JFC : 15/01/2008 : ajout de hu

    // Variables articulaires, valeurs initiales et limites //
    double *q, *qd, *qdd;
    double *q0, *qd0, *qdd0;

    double *qmin, *qmax;

    // Frc, Trq, Qq, tsim
    double *frc[3+1], *trq[3+1], *Qq;
    double tsim;

    /// initial time of the simulation [s] (for the user)
    double t0;
    /// final time of the simulation [s] (for the user)
    double tf;
    /// initial value of the integration step size [s] (for the user)
    double dt0;

    // set to 1 to stop the simulation
    int flag_stop;

    // Constraints
    double *lrod;
    int Nloopc, Ncons, Nuserc;
    double NRerr;
    double *lambda;

    // Links
    int Nlink, Nlink3D;
    double *Z, *Zd, *Fl;
    double **l3DWr; // JFC : Attention: la convention des indices est inversée par rapport aux habitudes Robotran

    // Sensors
    int Nsensor;

    // Ext Forces
    int Nxfrc, *xfidpt;
    double **SWr;    // JFC : Attention: la convention des indices est inversée par rapport aux habitudes Robotran

    // Wheel
    int Nwheel;
    double *rnom;

#if !defined SENSORKIN
    // User Model
    int Nuser_model;
    UserModel *user_model;

//#ifndef CMEX
    // User Variable
    UserIO *user_IO;
//#endif

#endif

    // User State
    double *ux, *uxd;
    double *ux0;
    int Nux;

    // OTHER FIELDS //
    double *udd;

    int DonePart;
    int DoneEquil;
    int DoneModal;

    int process; // 1 = part, 2 = equil, 3 = dirdynared, 
    int simu_end;

    char *mbs_filename;

    #ifdef REAL_TIME
    void *realtime;
    #endif

} MbsData;


/*
 * Print values of mbs_data (used for debug)
 */
#ifdef __cplusplus
extern "C" {
#endif
    void mbs_print_data(MbsData *mbs_data);
#ifdef __cplusplus
}
#endif

/*--------------------*/
#endif
