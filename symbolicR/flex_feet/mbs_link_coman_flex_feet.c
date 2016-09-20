//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Thu Nov 26 12:45:15 2015
//
//	==> Project name : coman_flex_feet
//	==> using XML input file 
//
//	==> Number of joints : 33
//
//	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
//	==> Flops complexity : 20
//
//	==> Generation Time :  0.020 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_link(double **frc,double **trq,double *Flnk,double *Z,double *Zd,
MbsData *s, double tsim)

// double frc[3][33];
// double trq[3][33];
// double Flnk[4];
// double Z[4];
// double Zd[4];
{ 
 
#include "mbs_link_coman_flex_feet.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_1_0_1_1_3 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Z1 = q[13];
  Zd1 = qd[13];
 
// Link Force Computation 

  Flink1 = user_LinkForces(Z1,Zd1,s,tsim,1);

// = = Block_0_1_0_1_2_4 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Z2 = q[14];
  Zd2 = qd[14];
 
// Link Force Computation 

  Flink2 = user_LinkForces(Z2,Zd2,s,tsim,2);

// = = Block_0_1_0_1_3_6 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Z3 = q[21];
  Zd3 = qd[21];
 
// Link Force Computation 

  Flink3 = user_LinkForces(Z3,Zd3,s,tsim,3);

// = = Block_0_1_0_1_4_7 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Z4 = q[22];
  Zd4 = qd[22];
 
// Link Force Computation 

  Flink4 = user_LinkForces(Z4,Zd4,s,tsim,4);

// = = Block_0_1_0_2_2_2 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk31 = user_LinkForces(Z1,Zd1,s,tsim,1);
  s->frc[3][12] = s->frc[3][12]+fPlnk31;
  s->trq[2][12] = s->trq[2][12]-fPlnk31*(s->dpt[1][20]-s->l[1][12]);

// = = Block_0_1_0_2_2_3 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk31 = user_LinkForces(Z1,Zd1,s,tsim,1);
  frc[3][13] = s->frc[3][13]-fSlnk31;

// = = Block_0_1_0_2_3_2 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk32 = user_LinkForces(Z2,Zd2,s,tsim,2);
  frc[3][12] = fPlnk32+s->frc[3][12];
  trq[2][12] = s->trq[2][12]-fPlnk32*(s->dpt[1][21]-s->l[1][12]);

// = = Block_0_1_0_2_3_4 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk32 = user_LinkForces(Z2,Zd2,s,tsim,2);
  frc[3][14] = s->frc[3][14]-fSlnk32;

// = = Block_0_1_0_2_4_5 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk33 = user_LinkForces(Z3,Zd3,s,tsim,3);
  s->frc[3][20] = s->frc[3][20]+fPlnk33;
  s->trq[2][20] = s->trq[2][20]-fPlnk33*(s->dpt[1][38]-s->l[1][20]);

// = = Block_0_1_0_2_4_6 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk33 = user_LinkForces(Z3,Zd3,s,tsim,3);
  frc[3][21] = s->frc[3][21]-fSlnk33;

// = = Block_0_1_0_2_5_5 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk34 = user_LinkForces(Z4,Zd4,s,tsim,4);
  frc[3][20] = fPlnk34+s->frc[3][20];
  trq[2][20] = s->trq[2][20]-fPlnk34*(s->dpt[1][39]-s->l[1][20]);

// = = Block_0_1_0_2_5_7 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk34 = user_LinkForces(Z4,Zd4,s,tsim,4);
  frc[3][22] = s->frc[3][22]-fSlnk34;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  frc[1][6] = s->frc[1][6];
  frc[2][6] = s->frc[2][6];
  frc[3][6] = s->frc[3][6];
  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][9] = s->frc[1][9];
  frc[2][9] = s->frc[2][9];
  frc[3][9] = s->frc[3][9];
  frc[1][10] = s->frc[1][10];
  frc[2][10] = s->frc[2][10];
  frc[3][10] = s->frc[3][10];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][12] = s->frc[1][12];
  frc[2][12] = s->frc[2][12];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[1][14] = s->frc[1][14];
  frc[2][14] = s->frc[2][14];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][17] = s->frc[1][17];
  frc[2][17] = s->frc[2][17];
  frc[3][17] = s->frc[3][17];
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
  frc[1][32] = s->frc[1][32];
  frc[2][32] = s->frc[2][32];
  frc[3][32] = s->frc[3][32];
  frc[1][33] = s->frc[1][33];
  frc[2][33] = s->frc[2][33];
  frc[3][33] = s->frc[3][33];
  trq[1][6] = s->trq[1][6];
  trq[2][6] = s->trq[2][6];
  trq[3][6] = s->trq[3][6];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][9] = s->trq[1][9];
  trq[2][9] = s->trq[2][9];
  trq[3][9] = s->trq[3][9];
  trq[1][10] = s->trq[1][10];
  trq[2][10] = s->trq[2][10];
  trq[3][10] = s->trq[3][10];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][12] = s->trq[1][12];
  trq[3][12] = s->trq[3][12];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][14] = s->trq[1][14];
  trq[2][14] = s->trq[2][14];
  trq[3][14] = s->trq[3][14];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][17] = s->trq[1][17];
  trq[2][17] = s->trq[2][17];
  trq[3][17] = s->trq[3][17];
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[3][20] = s->trq[3][20];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];
  trq[1][32] = s->trq[1][32];
  trq[2][32] = s->trq[2][32];
  trq[3][32] = s->trq[3][32];
  trq[1][33] = s->trq[1][33];
  trq[2][33] = s->trq[2][33];
  trq[3][33] = s->trq[3][33];
  Flnk[1] = Flink1;
  Flnk[2] = Flink2;
  Flnk[3] = Flink3;
  Flnk[4] = Flink4;
  Z[1] = q[13];
  Z[2] = q[14];
  Z[3] = q[21];
  Z[4] = q[22];
  Zd[1] = qd[13];
  Zd[2] = qd[14];
  Zd[3] = qd[21];
  Zd[4] = qd[22];

// ====== END Task 0 ====== 


}
 

