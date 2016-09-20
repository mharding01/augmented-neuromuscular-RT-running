
/**
 * 
 * Declaration of functions that are project dependent 
 *
 */

#include "mbs_data.h"
#include "mbs_aux.h"
#include "mbs_dirdyn.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                    User functions                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * */


void user_dirdyn_init(MbsData *MBSdata, MbsDirdyn *mbs_dd);
void user_dirdyn_loop(MbsData *MBSdata, MbsDirdyn *mbs_dd);
void user_dirdyn_finish(MbsData *MBSdata, MbsDirdyn *mbs_dd);


void user_cons_hJ(double *h, double **Jac, MbsData *s, double tsim);

void user_cons_jdqd(double *jdqd, MbsData *s, double tsim);

void user_Derivative(MbsData *s);

void user_DrivenJoints(MbsData *s, double tsim);

double* user_ExtForces(double PxF[4], double RxF[4][4],
                       double VxF[4], double OMxF[4],
                       double AxF[4], double OMPxF[4],
                       MbsData *s, double tsim, int ixF);

double* user_GenExtForces(double PxF[4], double RxF[4][4],
                       double VxF[4], double OMxF[4],
                       double AxF[4], double OMPxF[4],
                       MbsData *s, double tsim,int iBody);

double* user_JointForces(MbsData *s, double tsim);

double user_LinkForces(double Z, double Zd, MbsData *s, double tsim, int ilnk);

double* user_Link3DForces(double PxF[4], double RxF[4][4],
                       double VxF[4], double OMxF[4],
                       double AxF[4], double OMPxF[4],
                       MbsData *s, double tsim,int ixF);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                  Symbolic functions                   */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * */


void mbs_cons_hJ(double *h,double **Jac,MbsData *s, double tsim);

void mbs_cons_jdqd(double *Jdqd,MbsData *s, double tsim);

void mbs_link(double **frc,double **trq,double *Flnk,double *Z,double *Zd,MbsData *s, double tsim);

void mbs_link3D(double **frc,double **trq, MbsData *s, double tsim);

void mbs_extforces(double **frc, double **trq, MbsData *s, double tsim);

int mbs_accelred(MbsData *s, double tsim);

void  mbs_gensensor(MbsSensor *sens,
              MbsData *s,
              int isens);

void  mbs_sensor(MbsSensor *sens,
              MbsData *s,
              int isens);

void mbs_invdyna(double *Qq,
             MbsData *s, double tsim);

void mbs_dirdyna(double **M,double *c,
             MbsData *s, double tsim);
             
             


