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
//	==> Generation Date : Thu Nov 26 12:45:14 2015
//
//	==> Project name : coman_flex_feet
//	==> using XML input file 
//
//	==> Number of joints : 33
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 7333
//
//	==> Generation Time :  0.240 seconds
//	==> Post-Processing :  0.250 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][33];
// double trq[3][33];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
double PxF5[4]; 
double RxF5[4][4]; 
double VxF5[4]; 
double OMxF5[4]; 
double AxF5[4]; 
double OMPxF5[4]; 
double *SWr5; 
double PxF6[4]; 
double RxF6[4][4]; 
double VxF6[4]; 
double OMxF6[4]; 
double AxF6[4]; 
double OMPxF6[4]; 
double *SWr6; 
double PxF7[4]; 
double RxF7[4][4]; 
double VxF7[4]; 
double OMxF7[4]; 
double AxF7[4]; 
double OMPxF7[4]; 
double *SWr7; 
double PxF8[4]; 
double RxF8[4][4]; 
double VxF8[4]; 
double OMxF8[4]; 
double AxF8[4]; 
double OMPxF8[4]; 
double *SWr8; 
double PxF9[4]; 
double RxF9[4][4]; 
double VxF9[4]; 
double OMxF9[4]; 
double AxF9[4]; 
double OMPxF9[4]; 
double *SWr9; 
double PxF10[4]; 
double RxF10[4][4]; 
double VxF10[4]; 
double OMxF10[4]; 
double AxF10[4]; 
double OMPxF10[4]; 
double *SWr10; 
double PxF11[4]; 
double RxF11[4][4]; 
double VxF11[4]; 
double OMxF11[4]; 
double AxF11[4]; 
double OMPxF11[4]; 
double *SWr11; 
double PxF12[4]; 
double RxF12[4][4]; 
double VxF12[4]; 
double OMxF12[4]; 
double AxF12[4]; 
double OMPxF12[4]; 
double *SWr12; 
double PxF13[4]; 
double RxF13[4][4]; 
double VxF13[4]; 
double OMxF13[4]; 
double AxF13[4]; 
double OMPxF13[4]; 
double *SWr13; 
double PxF14[4]; 
double RxF14[4][4]; 
double VxF14[4]; 
double OMxF14[4]; 
double AxF14[4]; 
double OMPxF14[4]; 
double *SWr14; 
 
#include "mbs_extforces_coman_flex_feet.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_3 = = 
 
// Augmented Joint Position Vectors   

  Dz133 = q[13]+s->dpt[3][20];

// = = Block_0_0_0_0_0_4 = = 
 
// Augmented Joint Position Vectors   

  Dz143 = q[14]+s->dpt[3][21];

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);
  C18 = cos(q[18]);
  S18 = sin(q[18]);
  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_6 = = 
 
// Augmented Joint Position Vectors   

  Dz213 = q[21]+s->dpt[3][38];

// = = Block_0_0_0_0_0_7 = = 
 
// Augmented Joint Position Vectors   

  Dz223 = q[22]+s->dpt[3][39];

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);

// = = Block_0_0_1_1_0_1 = = 
 
// Sensor Kinematics 


  ROcp26_25 = S4*S5;
  ROcp26_35 = -C4*S5;
  ROcp26_85 = -S4*C5;
  ROcp26_95 = C4*C5;
  ROcp26_16 = C5*C6;
  ROcp26_26 = ROcp26_25*C6+C4*S6;
  ROcp26_36 = ROcp26_35*C6+S4*S6;
  ROcp26_46 = -C5*S6;
  ROcp26_56 = -(ROcp26_25*S6-C4*C6);
  ROcp26_66 = -(ROcp26_35*S6-S4*C6);
  OMcp26_25 = qd[5]*C4;
  OMcp26_35 = qd[5]*S4;
  PxF1[1] = q[1];
  PxF1[2] = q[2];
  PxF1[3] = q[3];
  RxF1[1][1] = ROcp26_16;
  RxF1[1][2] = ROcp26_26;
  RxF1[1][3] = ROcp26_36;
  RxF1[2][1] = ROcp26_46;
  RxF1[2][2] = ROcp26_56;
  RxF1[2][3] = ROcp26_66;
  RxF1[3][1] = S5;
  RxF1[3][2] = ROcp26_85;
  RxF1[3][3] = ROcp26_95;
  VxF1[1] = qd[1];
  VxF1[2] = qd[2];
  VxF1[3] = qd[3];
  OMxF1[1] = qd[4]+qd[6]*S5;
  OMxF1[2] = OMcp26_25+qd[6]*ROcp26_85;
  OMxF1[3] = OMcp26_35+qd[6]*ROcp26_95;
  AxF1[1] = qdd[1];
  AxF1[2] = qdd[2];
  AxF1[3] = qdd[3];
  OMPxF1[1] = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OMPxF1[2] = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp26_95-OMcp26_35*S5)-qdd[5]*C4-qdd[6]*ROcp26_85);
  OMPxF1[3] = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp26_85-OMcp26_25*S5)+qdd[5]*S4+qdd[6]*ROcp26_95;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc127 = ROcp26_16*SWr1[1]+ROcp26_26*SWr1[2]+ROcp26_36*SWr1[3];
  xfrc227 = ROcp26_46*SWr1[1]+ROcp26_56*SWr1[2]+ROcp26_66*SWr1[3];
  xfrc327 = ROcp26_85*SWr1[2]+ROcp26_95*SWr1[3]+SWr1[1]*S5;
  frc[1][6] = s->frc[1][6]+xfrc127;
  frc[2][6] = s->frc[2][6]+xfrc227;
  frc[3][6] = s->frc[3][6]+xfrc327;
  xtrq127 = ROcp26_16*SWr1[4]+ROcp26_26*SWr1[5]+ROcp26_36*SWr1[6];
  xtrq227 = ROcp26_46*SWr1[4]+ROcp26_56*SWr1[5]+ROcp26_66*SWr1[6];
  xtrq327 = ROcp26_85*SWr1[5]+ROcp26_95*SWr1[6]+SWr1[4]*S5;
  trq[1][6] = s->trq[1][6]+xtrq127-xfrc227*(SWr1[9]-s->l[3][6])+xfrc327*(SWr1[8]-s->l[2][6]);
  trq[2][6] = s->trq[2][6]+xtrq227+xfrc127*(SWr1[9]-s->l[3][6])-xfrc327*(SWr1[7]-s->l[1][6]);
  trq[3][6] = s->trq[3][6]+xtrq327-xfrc127*(SWr1[8]-s->l[2][6])+xfrc227*(SWr1[7]-s->l[1][6]);

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp27_25 = S4*S5;
  ROcp27_35 = -C4*S5;
  ROcp27_85 = -S4*C5;
  ROcp27_95 = C4*C5;
  ROcp27_16 = C5*C6;
  ROcp27_26 = ROcp27_25*C6+C4*S6;
  ROcp27_36 = ROcp27_35*C6+S4*S6;
  ROcp27_46 = -C5*S6;
  ROcp27_56 = -(ROcp27_25*S6-C4*C6);
  ROcp27_66 = -(ROcp27_35*S6-S4*C6);
  OMcp27_25 = qd[5]*C4;
  OMcp27_35 = qd[5]*S4;
  OMcp27_16 = qd[4]+qd[6]*S5;
  OMcp27_26 = OMcp27_25+qd[6]*ROcp27_85;
  OMcp27_36 = OMcp27_35+qd[6]*ROcp27_95;
  OPcp27_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp27_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp27_95-OMcp27_35*S5)-qdd[5]*C4-qdd[6]*ROcp27_85);
  OPcp27_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp27_85-OMcp27_25*S5)+qdd[5]*S4+qdd[6]*ROcp27_95;

// = = Block_0_0_1_2_0_2 = = 
 
// Sensor Kinematics 


  ROcp27_17 = ROcp27_16*C7-S5*S7;
  ROcp27_27 = ROcp27_26*C7-ROcp27_85*S7;
  ROcp27_37 = ROcp27_36*C7-ROcp27_95*S7;
  ROcp27_77 = ROcp27_16*S7+S5*C7;
  ROcp27_87 = ROcp27_26*S7+ROcp27_85*C7;
  ROcp27_97 = ROcp27_36*S7+ROcp27_95*C7;
  ROcp27_48 = ROcp27_46*C8+ROcp27_77*S8;
  ROcp27_58 = ROcp27_56*C8+ROcp27_87*S8;
  ROcp27_68 = ROcp27_66*C8+ROcp27_97*S8;
  ROcp27_78 = -(ROcp27_46*S8-ROcp27_77*C8);
  ROcp27_88 = -(ROcp27_56*S8-ROcp27_87*C8);
  ROcp27_98 = -(ROcp27_66*S8-ROcp27_97*C8);
  ROcp27_19 = ROcp27_17*C9+ROcp27_48*S9;
  ROcp27_29 = ROcp27_27*C9+ROcp27_58*S9;
  ROcp27_39 = ROcp27_37*C9+ROcp27_68*S9;
  ROcp27_49 = -(ROcp27_17*S9-ROcp27_48*C9);
  ROcp27_59 = -(ROcp27_27*S9-ROcp27_58*C9);
  ROcp27_69 = -(ROcp27_37*S9-ROcp27_68*C9);
  RLcp27_17 = ROcp27_46*s->dpt[2][2];
  RLcp27_27 = ROcp27_56*s->dpt[2][2];
  RLcp27_37 = ROcp27_66*s->dpt[2][2];
  OMcp27_17 = OMcp27_16+qd[7]*ROcp27_46;
  OMcp27_27 = OMcp27_26+qd[7]*ROcp27_56;
  OMcp27_37 = OMcp27_36+qd[7]*ROcp27_66;
  ORcp27_17 = OMcp27_26*RLcp27_37-OMcp27_36*RLcp27_27;
  ORcp27_27 = -(OMcp27_16*RLcp27_37-OMcp27_36*RLcp27_17);
  ORcp27_37 = OMcp27_16*RLcp27_27-OMcp27_26*RLcp27_17;
  OPcp27_17 = OPcp27_16+qd[7]*(OMcp27_26*ROcp27_66-OMcp27_36*ROcp27_56)+qdd[7]*ROcp27_46;
  OPcp27_27 = OPcp27_26-qd[7]*(OMcp27_16*ROcp27_66-OMcp27_36*ROcp27_46)+qdd[7]*ROcp27_56;
  OPcp27_37 = OPcp27_36+qd[7]*(OMcp27_16*ROcp27_56-OMcp27_26*ROcp27_46)+qdd[7]*ROcp27_66;
  RLcp27_18 = ROcp27_46*s->dpt[2][6];
  RLcp27_28 = ROcp27_56*s->dpt[2][6];
  RLcp27_38 = ROcp27_66*s->dpt[2][6];
  OMcp27_18 = OMcp27_17+qd[8]*ROcp27_17;
  OMcp27_28 = OMcp27_27+qd[8]*ROcp27_27;
  OMcp27_38 = OMcp27_37+qd[8]*ROcp27_37;
  ORcp27_18 = OMcp27_27*RLcp27_38-OMcp27_37*RLcp27_28;
  ORcp27_28 = -(OMcp27_17*RLcp27_38-OMcp27_37*RLcp27_18);
  ORcp27_38 = OMcp27_17*RLcp27_28-OMcp27_27*RLcp27_18;
  OPcp27_18 = OPcp27_17+qd[8]*(OMcp27_27*ROcp27_37-OMcp27_37*ROcp27_27)+qdd[8]*ROcp27_17;
  OPcp27_28 = OPcp27_27-qd[8]*(OMcp27_17*ROcp27_37-OMcp27_37*ROcp27_17)+qdd[8]*ROcp27_27;
  OPcp27_38 = OPcp27_37+qd[8]*(OMcp27_17*ROcp27_27-OMcp27_27*ROcp27_17)+qdd[8]*ROcp27_37;
  RLcp27_19 = ROcp27_78*s->dpt[3][8];
  RLcp27_29 = ROcp27_88*s->dpt[3][8];
  RLcp27_39 = ROcp27_98*s->dpt[3][8];
  ORcp27_19 = OMcp27_28*RLcp27_39-OMcp27_38*RLcp27_29;
  ORcp27_29 = -(OMcp27_18*RLcp27_39-OMcp27_38*RLcp27_19);
  ORcp27_39 = OMcp27_18*RLcp27_29-OMcp27_28*RLcp27_19;
  PxF2[1] = q[1]+RLcp27_17+RLcp27_18+RLcp27_19;
  PxF2[2] = q[2]+RLcp27_27+RLcp27_28+RLcp27_29;
  PxF2[3] = q[3]+RLcp27_37+RLcp27_38+RLcp27_39;
  RxF2[1][1] = ROcp27_19;
  RxF2[1][2] = ROcp27_29;
  RxF2[1][3] = ROcp27_39;
  RxF2[2][1] = ROcp27_49;
  RxF2[2][2] = ROcp27_59;
  RxF2[2][3] = ROcp27_69;
  RxF2[3][1] = ROcp27_78;
  RxF2[3][2] = ROcp27_88;
  RxF2[3][3] = ROcp27_98;
  VxF2[1] = qd[1]+ORcp27_17+ORcp27_18+ORcp27_19;
  VxF2[2] = qd[2]+ORcp27_27+ORcp27_28+ORcp27_29;
  VxF2[3] = qd[3]+ORcp27_37+ORcp27_38+ORcp27_39;
  OMxF2[1] = OMcp27_18+qd[9]*ROcp27_78;
  OMxF2[2] = OMcp27_28+qd[9]*ROcp27_88;
  OMxF2[3] = OMcp27_38+qd[9]*ROcp27_98;
  AxF2[1] = qdd[1]+OMcp27_26*ORcp27_37+OMcp27_27*ORcp27_38+OMcp27_28*ORcp27_39-OMcp27_36*ORcp27_27-OMcp27_37*ORcp27_28-
 OMcp27_38*ORcp27_29+OPcp27_26*RLcp27_37+OPcp27_27*RLcp27_38+OPcp27_28*RLcp27_39-OPcp27_36*RLcp27_27-OPcp27_37*RLcp27_28-
 OPcp27_38*RLcp27_29;
  AxF2[2] = qdd[2]-OMcp27_16*ORcp27_37-OMcp27_17*ORcp27_38-OMcp27_18*ORcp27_39+OMcp27_36*ORcp27_17+OMcp27_37*ORcp27_18+
 OMcp27_38*ORcp27_19-OPcp27_16*RLcp27_37-OPcp27_17*RLcp27_38-OPcp27_18*RLcp27_39+OPcp27_36*RLcp27_17+OPcp27_37*RLcp27_18+
 OPcp27_38*RLcp27_19;
  AxF2[3] = qdd[3]+OMcp27_16*ORcp27_27+OMcp27_17*ORcp27_28+OMcp27_18*ORcp27_29-OMcp27_26*ORcp27_17-OMcp27_27*ORcp27_18-
 OMcp27_28*ORcp27_19+OPcp27_16*RLcp27_27+OPcp27_17*RLcp27_28+OPcp27_18*RLcp27_29-OPcp27_26*RLcp27_17-OPcp27_27*RLcp27_18-
 OPcp27_28*RLcp27_19;
  OMPxF2[1] = OPcp27_18+qd[9]*(OMcp27_28*ROcp27_98-OMcp27_38*ROcp27_88)+qdd[9]*ROcp27_78;
  OMPxF2[2] = OPcp27_28-qd[9]*(OMcp27_18*ROcp27_98-OMcp27_38*ROcp27_78)+qdd[9]*ROcp27_88;
  OMPxF2[3] = OPcp27_38+qd[9]*(OMcp27_18*ROcp27_88-OMcp27_28*ROcp27_78)+qdd[9]*ROcp27_98;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc128 = ROcp27_19*SWr2[1]+ROcp27_29*SWr2[2]+ROcp27_39*SWr2[3];
  xfrc228 = ROcp27_49*SWr2[1]+ROcp27_59*SWr2[2]+ROcp27_69*SWr2[3];
  xfrc328 = ROcp27_78*SWr2[1]+ROcp27_88*SWr2[2]+ROcp27_98*SWr2[3];
  frc[1][9] = s->frc[1][9]+xfrc128;
  frc[2][9] = s->frc[2][9]+xfrc228;
  frc[3][9] = s->frc[3][9]+xfrc328;
  xtrq128 = ROcp27_19*SWr2[4]+ROcp27_29*SWr2[5]+ROcp27_39*SWr2[6];
  xtrq228 = ROcp27_49*SWr2[4]+ROcp27_59*SWr2[5]+ROcp27_69*SWr2[6];
  xtrq328 = ROcp27_78*SWr2[4]+ROcp27_88*SWr2[5]+ROcp27_98*SWr2[6];
  trq[1][9] = s->trq[1][9]+xtrq128-xfrc228*(SWr2[9]-s->l[3][9])+xfrc328*(SWr2[8]-s->l[2][9]);
  trq[2][9] = s->trq[2][9]+xtrq228+xfrc128*(SWr2[9]-s->l[3][9])-xfrc328*(SWr2[7]-s->l[1][9]);
  trq[3][9] = s->trq[3][9]+xtrq328-xfrc128*(SWr2[8]-s->l[2][9])+xfrc228*(SWr2[7]-s->l[1][9]);

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


  ROcp28_25 = S4*S5;
  ROcp28_35 = -C4*S5;
  ROcp28_85 = -S4*C5;
  ROcp28_95 = C4*C5;
  ROcp28_16 = C5*C6;
  ROcp28_26 = ROcp28_25*C6+C4*S6;
  ROcp28_36 = ROcp28_35*C6+S4*S6;
  ROcp28_46 = -C5*S6;
  ROcp28_56 = -(ROcp28_25*S6-C4*C6);
  ROcp28_66 = -(ROcp28_35*S6-S4*C6);
  OMcp28_25 = qd[5]*C4;
  OMcp28_35 = qd[5]*S4;
  OMcp28_16 = qd[4]+qd[6]*S5;
  OMcp28_26 = OMcp28_25+qd[6]*ROcp28_85;
  OMcp28_36 = OMcp28_35+qd[6]*ROcp28_95;
  OPcp28_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp28_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp28_95-OMcp28_35*S5)-qdd[5]*C4-qdd[6]*ROcp28_85);
  OPcp28_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp28_85-OMcp28_25*S5)+qdd[5]*S4+qdd[6]*ROcp28_95;

// = = Block_0_0_1_3_0_2 = = 
 
// Sensor Kinematics 


  ROcp28_17 = ROcp28_16*C7-S5*S7;
  ROcp28_27 = ROcp28_26*C7-ROcp28_85*S7;
  ROcp28_37 = ROcp28_36*C7-ROcp28_95*S7;
  ROcp28_77 = ROcp28_16*S7+S5*C7;
  ROcp28_87 = ROcp28_26*S7+ROcp28_85*C7;
  ROcp28_97 = ROcp28_36*S7+ROcp28_95*C7;
  ROcp28_48 = ROcp28_46*C8+ROcp28_77*S8;
  ROcp28_58 = ROcp28_56*C8+ROcp28_87*S8;
  ROcp28_68 = ROcp28_66*C8+ROcp28_97*S8;
  ROcp28_78 = -(ROcp28_46*S8-ROcp28_77*C8);
  ROcp28_88 = -(ROcp28_56*S8-ROcp28_87*C8);
  ROcp28_98 = -(ROcp28_66*S8-ROcp28_97*C8);
  ROcp28_19 = ROcp28_17*C9+ROcp28_48*S9;
  ROcp28_29 = ROcp28_27*C9+ROcp28_58*S9;
  ROcp28_39 = ROcp28_37*C9+ROcp28_68*S9;
  ROcp28_49 = -(ROcp28_17*S9-ROcp28_48*C9);
  ROcp28_59 = -(ROcp28_27*S9-ROcp28_58*C9);
  ROcp28_69 = -(ROcp28_37*S9-ROcp28_68*C9);
  ROcp28_110 = ROcp28_19*C10-ROcp28_78*S10;
  ROcp28_210 = ROcp28_29*C10-ROcp28_88*S10;
  ROcp28_310 = ROcp28_39*C10-ROcp28_98*S10;
  ROcp28_710 = ROcp28_19*S10+ROcp28_78*C10;
  ROcp28_810 = ROcp28_29*S10+ROcp28_88*C10;
  ROcp28_910 = ROcp28_39*S10+ROcp28_98*C10;
  RLcp28_17 = ROcp28_46*s->dpt[2][2];
  RLcp28_27 = ROcp28_56*s->dpt[2][2];
  RLcp28_37 = ROcp28_66*s->dpt[2][2];
  OMcp28_17 = OMcp28_16+qd[7]*ROcp28_46;
  OMcp28_27 = OMcp28_26+qd[7]*ROcp28_56;
  OMcp28_37 = OMcp28_36+qd[7]*ROcp28_66;
  ORcp28_17 = OMcp28_26*RLcp28_37-OMcp28_36*RLcp28_27;
  ORcp28_27 = -(OMcp28_16*RLcp28_37-OMcp28_36*RLcp28_17);
  ORcp28_37 = OMcp28_16*RLcp28_27-OMcp28_26*RLcp28_17;
  OPcp28_17 = OPcp28_16+qd[7]*(OMcp28_26*ROcp28_66-OMcp28_36*ROcp28_56)+qdd[7]*ROcp28_46;
  OPcp28_27 = OPcp28_26-qd[7]*(OMcp28_16*ROcp28_66-OMcp28_36*ROcp28_46)+qdd[7]*ROcp28_56;
  OPcp28_37 = OPcp28_36+qd[7]*(OMcp28_16*ROcp28_56-OMcp28_26*ROcp28_46)+qdd[7]*ROcp28_66;
  RLcp28_18 = ROcp28_46*s->dpt[2][6];
  RLcp28_28 = ROcp28_56*s->dpt[2][6];
  RLcp28_38 = ROcp28_66*s->dpt[2][6];
  OMcp28_18 = OMcp28_17+qd[8]*ROcp28_17;
  OMcp28_28 = OMcp28_27+qd[8]*ROcp28_27;
  OMcp28_38 = OMcp28_37+qd[8]*ROcp28_37;
  ORcp28_18 = OMcp28_27*RLcp28_38-OMcp28_37*RLcp28_28;
  ORcp28_28 = -(OMcp28_17*RLcp28_38-OMcp28_37*RLcp28_18);
  ORcp28_38 = OMcp28_17*RLcp28_28-OMcp28_27*RLcp28_18;
  OPcp28_18 = OPcp28_17+qd[8]*(OMcp28_27*ROcp28_37-OMcp28_37*ROcp28_27)+qdd[8]*ROcp28_17;
  OPcp28_28 = OPcp28_27-qd[8]*(OMcp28_17*ROcp28_37-OMcp28_37*ROcp28_17)+qdd[8]*ROcp28_27;
  OPcp28_38 = OPcp28_37+qd[8]*(OMcp28_17*ROcp28_27-OMcp28_27*ROcp28_17)+qdd[8]*ROcp28_37;
  RLcp28_19 = ROcp28_78*s->dpt[3][8];
  RLcp28_29 = ROcp28_88*s->dpt[3][8];
  RLcp28_39 = ROcp28_98*s->dpt[3][8];
  OMcp28_19 = OMcp28_18+qd[9]*ROcp28_78;
  OMcp28_29 = OMcp28_28+qd[9]*ROcp28_88;
  OMcp28_39 = OMcp28_38+qd[9]*ROcp28_98;
  ORcp28_19 = OMcp28_28*RLcp28_39-OMcp28_38*RLcp28_29;
  ORcp28_29 = -(OMcp28_18*RLcp28_39-OMcp28_38*RLcp28_19);
  ORcp28_39 = OMcp28_18*RLcp28_29-OMcp28_28*RLcp28_19;
  OPcp28_19 = OPcp28_18+qd[9]*(OMcp28_28*ROcp28_98-OMcp28_38*ROcp28_88)+qdd[9]*ROcp28_78;
  OPcp28_29 = OPcp28_28-qd[9]*(OMcp28_18*ROcp28_98-OMcp28_38*ROcp28_78)+qdd[9]*ROcp28_88;
  OPcp28_39 = OPcp28_38+qd[9]*(OMcp28_18*ROcp28_88-OMcp28_28*ROcp28_78)+qdd[9]*ROcp28_98;
  RLcp28_110 = ROcp28_78*s->dpt[3][11];
  RLcp28_210 = ROcp28_88*s->dpt[3][11];
  RLcp28_310 = ROcp28_98*s->dpt[3][11];
  ORcp28_110 = OMcp28_29*RLcp28_310-OMcp28_39*RLcp28_210;
  ORcp28_210 = -(OMcp28_19*RLcp28_310-OMcp28_39*RLcp28_110);
  ORcp28_310 = OMcp28_19*RLcp28_210-OMcp28_29*RLcp28_110;
  PxF3[1] = q[1]+RLcp28_110+RLcp28_17+RLcp28_18+RLcp28_19;
  PxF3[2] = q[2]+RLcp28_210+RLcp28_27+RLcp28_28+RLcp28_29;
  PxF3[3] = q[3]+RLcp28_310+RLcp28_37+RLcp28_38+RLcp28_39;
  RxF3[1][1] = ROcp28_110;
  RxF3[1][2] = ROcp28_210;
  RxF3[1][3] = ROcp28_310;
  RxF3[2][1] = ROcp28_49;
  RxF3[2][2] = ROcp28_59;
  RxF3[2][3] = ROcp28_69;
  RxF3[3][1] = ROcp28_710;
  RxF3[3][2] = ROcp28_810;
  RxF3[3][3] = ROcp28_910;
  VxF3[1] = qd[1]+ORcp28_110+ORcp28_17+ORcp28_18+ORcp28_19;
  VxF3[2] = qd[2]+ORcp28_210+ORcp28_27+ORcp28_28+ORcp28_29;
  VxF3[3] = qd[3]+ORcp28_310+ORcp28_37+ORcp28_38+ORcp28_39;
  OMxF3[1] = OMcp28_19+qd[10]*ROcp28_49;
  OMxF3[2] = OMcp28_29+qd[10]*ROcp28_59;
  OMxF3[3] = OMcp28_39+qd[10]*ROcp28_69;
  AxF3[1] = qdd[1]+OMcp28_26*ORcp28_37+OMcp28_27*ORcp28_38+OMcp28_28*ORcp28_39+OMcp28_29*ORcp28_310-OMcp28_36*ORcp28_27-
 OMcp28_37*ORcp28_28-OMcp28_38*ORcp28_29-OMcp28_39*ORcp28_210+OPcp28_26*RLcp28_37+OPcp28_27*RLcp28_38+OPcp28_28*RLcp28_39+
 OPcp28_29*RLcp28_310-OPcp28_36*RLcp28_27-OPcp28_37*RLcp28_28-OPcp28_38*RLcp28_29-OPcp28_39*RLcp28_210;
  AxF3[2] = qdd[2]-OMcp28_16*ORcp28_37-OMcp28_17*ORcp28_38-OMcp28_18*ORcp28_39-OMcp28_19*ORcp28_310+OMcp28_36*ORcp28_17+
 OMcp28_37*ORcp28_18+OMcp28_38*ORcp28_19+OMcp28_39*ORcp28_110-OPcp28_16*RLcp28_37-OPcp28_17*RLcp28_38-OPcp28_18*RLcp28_39-
 OPcp28_19*RLcp28_310+OPcp28_36*RLcp28_17+OPcp28_37*RLcp28_18+OPcp28_38*RLcp28_19+OPcp28_39*RLcp28_110;
  AxF3[3] = qdd[3]+OMcp28_16*ORcp28_27+OMcp28_17*ORcp28_28+OMcp28_18*ORcp28_29+OMcp28_19*ORcp28_210-OMcp28_26*ORcp28_17-
 OMcp28_27*ORcp28_18-OMcp28_28*ORcp28_19-OMcp28_29*ORcp28_110+OPcp28_16*RLcp28_27+OPcp28_17*RLcp28_28+OPcp28_18*RLcp28_29+
 OPcp28_19*RLcp28_210-OPcp28_26*RLcp28_17-OPcp28_27*RLcp28_18-OPcp28_28*RLcp28_19-OPcp28_29*RLcp28_110;
  OMPxF3[1] = OPcp28_19+qd[10]*(OMcp28_29*ROcp28_69-OMcp28_39*ROcp28_59)+qdd[10]*ROcp28_49;
  OMPxF3[2] = OPcp28_29-qd[10]*(OMcp28_19*ROcp28_69-OMcp28_39*ROcp28_49)+qdd[10]*ROcp28_59;
  OMPxF3[3] = OPcp28_39+qd[10]*(OMcp28_19*ROcp28_59-OMcp28_29*ROcp28_49)+qdd[10]*ROcp28_69;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc129 = ROcp28_110*SWr3[1]+ROcp28_210*SWr3[2]+ROcp28_310*SWr3[3];
  xfrc229 = ROcp28_49*SWr3[1]+ROcp28_59*SWr3[2]+ROcp28_69*SWr3[3];
  xfrc329 = ROcp28_710*SWr3[1]+ROcp28_810*SWr3[2]+ROcp28_910*SWr3[3];
  frc[1][10] = s->frc[1][10]+xfrc129;
  frc[2][10] = s->frc[2][10]+xfrc229;
  frc[3][10] = s->frc[3][10]+xfrc329;
  xtrq129 = ROcp28_110*SWr3[4]+ROcp28_210*SWr3[5]+ROcp28_310*SWr3[6];
  xtrq229 = ROcp28_49*SWr3[4]+ROcp28_59*SWr3[5]+ROcp28_69*SWr3[6];
  xtrq329 = ROcp28_710*SWr3[4]+ROcp28_810*SWr3[5]+ROcp28_910*SWr3[6];
  trq[1][10] = s->trq[1][10]+xtrq129-xfrc229*(SWr3[9]-s->l[3][10])+xfrc329*(SWr3[8]-s->l[2][10]);
  trq[2][10] = s->trq[2][10]+xtrq229+xfrc129*(SWr3[9]-s->l[3][10])-xfrc329*(SWr3[7]-s->l[1][10]);
  trq[3][10] = s->trq[3][10]+xtrq329-xfrc129*(SWr3[8]-s->l[2][10])+xfrc229*(SWr3[7]-s->l[1][10]);

// = = Block_0_0_1_4_0_1 = = 
 
// Sensor Kinematics 


  ROcp29_25 = S4*S5;
  ROcp29_35 = -C4*S5;
  ROcp29_85 = -S4*C5;
  ROcp29_95 = C4*C5;
  ROcp29_16 = C5*C6;
  ROcp29_26 = ROcp29_25*C6+C4*S6;
  ROcp29_36 = ROcp29_35*C6+S4*S6;
  ROcp29_46 = -C5*S6;
  ROcp29_56 = -(ROcp29_25*S6-C4*C6);
  ROcp29_66 = -(ROcp29_35*S6-S4*C6);
  OMcp29_25 = qd[5]*C4;
  OMcp29_35 = qd[5]*S4;
  OMcp29_16 = qd[4]+qd[6]*S5;
  OMcp29_26 = OMcp29_25+qd[6]*ROcp29_85;
  OMcp29_36 = OMcp29_35+qd[6]*ROcp29_95;
  OPcp29_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp29_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp29_95-OMcp29_35*S5)-qdd[5]*C4-qdd[6]*ROcp29_85);
  OPcp29_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp29_85-OMcp29_25*S5)+qdd[5]*S4+qdd[6]*ROcp29_95;

// = = Block_0_0_1_4_0_2 = = 
 
// Sensor Kinematics 


  ROcp29_17 = ROcp29_16*C7-S5*S7;
  ROcp29_27 = ROcp29_26*C7-ROcp29_85*S7;
  ROcp29_37 = ROcp29_36*C7-ROcp29_95*S7;
  ROcp29_77 = ROcp29_16*S7+S5*C7;
  ROcp29_87 = ROcp29_26*S7+ROcp29_85*C7;
  ROcp29_97 = ROcp29_36*S7+ROcp29_95*C7;
  ROcp29_48 = ROcp29_46*C8+ROcp29_77*S8;
  ROcp29_58 = ROcp29_56*C8+ROcp29_87*S8;
  ROcp29_68 = ROcp29_66*C8+ROcp29_97*S8;
  ROcp29_78 = -(ROcp29_46*S8-ROcp29_77*C8);
  ROcp29_88 = -(ROcp29_56*S8-ROcp29_87*C8);
  ROcp29_98 = -(ROcp29_66*S8-ROcp29_97*C8);
  ROcp29_19 = ROcp29_17*C9+ROcp29_48*S9;
  ROcp29_29 = ROcp29_27*C9+ROcp29_58*S9;
  ROcp29_39 = ROcp29_37*C9+ROcp29_68*S9;
  ROcp29_49 = -(ROcp29_17*S9-ROcp29_48*C9);
  ROcp29_59 = -(ROcp29_27*S9-ROcp29_58*C9);
  ROcp29_69 = -(ROcp29_37*S9-ROcp29_68*C9);
  ROcp29_110 = ROcp29_19*C10-ROcp29_78*S10;
  ROcp29_210 = ROcp29_29*C10-ROcp29_88*S10;
  ROcp29_310 = ROcp29_39*C10-ROcp29_98*S10;
  ROcp29_710 = ROcp29_19*S10+ROcp29_78*C10;
  ROcp29_810 = ROcp29_29*S10+ROcp29_88*C10;
  ROcp29_910 = ROcp29_39*S10+ROcp29_98*C10;
  ROcp29_411 = ROcp29_49*C11+ROcp29_710*S11;
  ROcp29_511 = ROcp29_59*C11+ROcp29_810*S11;
  ROcp29_611 = ROcp29_69*C11+ROcp29_910*S11;
  ROcp29_711 = -(ROcp29_49*S11-ROcp29_710*C11);
  ROcp29_811 = -(ROcp29_59*S11-ROcp29_810*C11);
  ROcp29_911 = -(ROcp29_69*S11-ROcp29_910*C11);
  ROcp29_112 = ROcp29_110*C12-ROcp29_711*S12;
  ROcp29_212 = ROcp29_210*C12-ROcp29_811*S12;
  ROcp29_312 = ROcp29_310*C12-ROcp29_911*S12;
  ROcp29_712 = ROcp29_110*S12+ROcp29_711*C12;
  ROcp29_812 = ROcp29_210*S12+ROcp29_811*C12;
  ROcp29_912 = ROcp29_310*S12+ROcp29_911*C12;
  RLcp29_17 = ROcp29_46*s->dpt[2][2];
  RLcp29_27 = ROcp29_56*s->dpt[2][2];
  RLcp29_37 = ROcp29_66*s->dpt[2][2];
  OMcp29_17 = OMcp29_16+qd[7]*ROcp29_46;
  OMcp29_27 = OMcp29_26+qd[7]*ROcp29_56;
  OMcp29_37 = OMcp29_36+qd[7]*ROcp29_66;
  ORcp29_17 = OMcp29_26*RLcp29_37-OMcp29_36*RLcp29_27;
  ORcp29_27 = -(OMcp29_16*RLcp29_37-OMcp29_36*RLcp29_17);
  ORcp29_37 = OMcp29_16*RLcp29_27-OMcp29_26*RLcp29_17;
  OPcp29_17 = OPcp29_16+qd[7]*(OMcp29_26*ROcp29_66-OMcp29_36*ROcp29_56)+qdd[7]*ROcp29_46;
  OPcp29_27 = OPcp29_26-qd[7]*(OMcp29_16*ROcp29_66-OMcp29_36*ROcp29_46)+qdd[7]*ROcp29_56;
  OPcp29_37 = OPcp29_36+qd[7]*(OMcp29_16*ROcp29_56-OMcp29_26*ROcp29_46)+qdd[7]*ROcp29_66;
  RLcp29_18 = ROcp29_46*s->dpt[2][6];
  RLcp29_28 = ROcp29_56*s->dpt[2][6];
  RLcp29_38 = ROcp29_66*s->dpt[2][6];
  OMcp29_18 = OMcp29_17+qd[8]*ROcp29_17;
  OMcp29_28 = OMcp29_27+qd[8]*ROcp29_27;
  OMcp29_38 = OMcp29_37+qd[8]*ROcp29_37;
  ORcp29_18 = OMcp29_27*RLcp29_38-OMcp29_37*RLcp29_28;
  ORcp29_28 = -(OMcp29_17*RLcp29_38-OMcp29_37*RLcp29_18);
  ORcp29_38 = OMcp29_17*RLcp29_28-OMcp29_27*RLcp29_18;
  OPcp29_18 = OPcp29_17+qd[8]*(OMcp29_27*ROcp29_37-OMcp29_37*ROcp29_27)+qdd[8]*ROcp29_17;
  OPcp29_28 = OPcp29_27-qd[8]*(OMcp29_17*ROcp29_37-OMcp29_37*ROcp29_17)+qdd[8]*ROcp29_27;
  OPcp29_38 = OPcp29_37+qd[8]*(OMcp29_17*ROcp29_27-OMcp29_27*ROcp29_17)+qdd[8]*ROcp29_37;
  RLcp29_19 = ROcp29_78*s->dpt[3][8];
  RLcp29_29 = ROcp29_88*s->dpt[3][8];
  RLcp29_39 = ROcp29_98*s->dpt[3][8];
  OMcp29_19 = OMcp29_18+qd[9]*ROcp29_78;
  OMcp29_29 = OMcp29_28+qd[9]*ROcp29_88;
  OMcp29_39 = OMcp29_38+qd[9]*ROcp29_98;
  ORcp29_19 = OMcp29_28*RLcp29_39-OMcp29_38*RLcp29_29;
  ORcp29_29 = -(OMcp29_18*RLcp29_39-OMcp29_38*RLcp29_19);
  ORcp29_39 = OMcp29_18*RLcp29_29-OMcp29_28*RLcp29_19;
  OPcp29_19 = OPcp29_18+qd[9]*(OMcp29_28*ROcp29_98-OMcp29_38*ROcp29_88)+qdd[9]*ROcp29_78;
  OPcp29_29 = OPcp29_28-qd[9]*(OMcp29_18*ROcp29_98-OMcp29_38*ROcp29_78)+qdd[9]*ROcp29_88;
  OPcp29_39 = OPcp29_38+qd[9]*(OMcp29_18*ROcp29_88-OMcp29_28*ROcp29_78)+qdd[9]*ROcp29_98;
  RLcp29_110 = ROcp29_78*s->dpt[3][11];
  RLcp29_210 = ROcp29_88*s->dpt[3][11];
  RLcp29_310 = ROcp29_98*s->dpt[3][11];
  OMcp29_110 = OMcp29_19+qd[10]*ROcp29_49;
  OMcp29_210 = OMcp29_29+qd[10]*ROcp29_59;
  OMcp29_310 = OMcp29_39+qd[10]*ROcp29_69;
  ORcp29_110 = OMcp29_29*RLcp29_310-OMcp29_39*RLcp29_210;
  ORcp29_210 = -(OMcp29_19*RLcp29_310-OMcp29_39*RLcp29_110);
  ORcp29_310 = OMcp29_19*RLcp29_210-OMcp29_29*RLcp29_110;
  OPcp29_110 = OPcp29_19+qd[10]*(OMcp29_29*ROcp29_69-OMcp29_39*ROcp29_59)+qdd[10]*ROcp29_49;
  OPcp29_210 = OPcp29_29-qd[10]*(OMcp29_19*ROcp29_69-OMcp29_39*ROcp29_49)+qdd[10]*ROcp29_59;
  OPcp29_310 = OPcp29_39+qd[10]*(OMcp29_19*ROcp29_59-OMcp29_29*ROcp29_49)+qdd[10]*ROcp29_69;
  RLcp29_111 = ROcp29_710*s->dpt[3][14];
  RLcp29_211 = ROcp29_810*s->dpt[3][14];
  RLcp29_311 = ROcp29_910*s->dpt[3][14];
  OMcp29_111 = OMcp29_110+qd[11]*ROcp29_110;
  OMcp29_211 = OMcp29_210+qd[11]*ROcp29_210;
  OMcp29_311 = OMcp29_310+qd[11]*ROcp29_310;
  ORcp29_111 = OMcp29_210*RLcp29_311-OMcp29_310*RLcp29_211;
  ORcp29_211 = -(OMcp29_110*RLcp29_311-OMcp29_310*RLcp29_111);
  ORcp29_311 = OMcp29_110*RLcp29_211-OMcp29_210*RLcp29_111;
  OMcp29_112 = OMcp29_111+qd[12]*ROcp29_411;
  OMcp29_212 = OMcp29_211+qd[12]*ROcp29_511;
  OMcp29_312 = OMcp29_311+qd[12]*ROcp29_611;
  OPcp29_112 = OPcp29_110+qd[11]*(OMcp29_210*ROcp29_310-OMcp29_310*ROcp29_210)+qd[12]*(OMcp29_211*ROcp29_611-OMcp29_311*
 ROcp29_511)+qdd[11]*ROcp29_110+qdd[12]*ROcp29_411;
  OPcp29_212 = OPcp29_210-qd[11]*(OMcp29_110*ROcp29_310-OMcp29_310*ROcp29_110)-qd[12]*(OMcp29_111*ROcp29_611-OMcp29_311*
 ROcp29_411)+qdd[11]*ROcp29_210+qdd[12]*ROcp29_511;
  OPcp29_312 = OPcp29_310+qd[11]*(OMcp29_110*ROcp29_210-OMcp29_210*ROcp29_110)+qd[12]*(OMcp29_111*ROcp29_511-OMcp29_211*
 ROcp29_411)+qdd[11]*ROcp29_310+qdd[12]*ROcp29_611;

// = = Block_0_0_1_4_0_3 = = 
 
// Sensor Kinematics 


  RLcp29_113 = Dz133*ROcp29_712+ROcp29_112*s->dpt[1][20];
  RLcp29_213 = Dz133*ROcp29_812+ROcp29_212*s->dpt[1][20];
  RLcp29_313 = Dz133*ROcp29_912+ROcp29_312*s->dpt[1][20];
  ORcp29_113 = OMcp29_212*RLcp29_313-OMcp29_312*RLcp29_213;
  ORcp29_213 = -(OMcp29_112*RLcp29_313-OMcp29_312*RLcp29_113);
  ORcp29_313 = OMcp29_112*RLcp29_213-OMcp29_212*RLcp29_113;
  PxF4[1] = q[1]+RLcp29_110+RLcp29_111+RLcp29_113+RLcp29_17+RLcp29_18+RLcp29_19;
  PxF4[2] = q[2]+RLcp29_210+RLcp29_211+RLcp29_213+RLcp29_27+RLcp29_28+RLcp29_29;
  PxF4[3] = q[3]+RLcp29_310+RLcp29_311+RLcp29_313+RLcp29_37+RLcp29_38+RLcp29_39;
  RxF4[1][1] = ROcp29_112;
  RxF4[1][2] = ROcp29_212;
  RxF4[1][3] = ROcp29_312;
  RxF4[2][1] = ROcp29_411;
  RxF4[2][2] = ROcp29_511;
  RxF4[2][3] = ROcp29_611;
  RxF4[3][1] = ROcp29_712;
  RxF4[3][2] = ROcp29_812;
  RxF4[3][3] = ROcp29_912;
  VxF4[1] = qd[1]+ORcp29_110+ORcp29_111+ORcp29_113+ORcp29_17+ORcp29_18+ORcp29_19+qd[13]*ROcp29_712;
  VxF4[2] = qd[2]+ORcp29_210+ORcp29_211+ORcp29_213+ORcp29_27+ORcp29_28+ORcp29_29+qd[13]*ROcp29_812;
  VxF4[3] = qd[3]+ORcp29_310+ORcp29_311+ORcp29_313+ORcp29_37+ORcp29_38+ORcp29_39+qd[13]*ROcp29_912;
  OMxF4[1] = OMcp29_112;
  OMxF4[2] = OMcp29_212;
  OMxF4[3] = OMcp29_312;
  AxF4[1] = qdd[1]+(2.0)*qd[13]*(OMcp29_212*ROcp29_912-OMcp29_312*ROcp29_812)+qdd[13]*ROcp29_712+OMcp29_210*ORcp29_311+
 OMcp29_212*ORcp29_313+OMcp29_26*ORcp29_37+OMcp29_27*ORcp29_38+OMcp29_28*ORcp29_39+OMcp29_29*ORcp29_310-OMcp29_310*ORcp29_211
 -OMcp29_312*ORcp29_213-OMcp29_36*ORcp29_27-OMcp29_37*ORcp29_28-OMcp29_38*ORcp29_29-OMcp29_39*ORcp29_210+OPcp29_210*
 RLcp29_311+OPcp29_212*RLcp29_313+OPcp29_26*RLcp29_37+OPcp29_27*RLcp29_38+OPcp29_28*RLcp29_39+OPcp29_29*RLcp29_310-OPcp29_310
 *RLcp29_211-OPcp29_312*RLcp29_213-OPcp29_36*RLcp29_27-OPcp29_37*RLcp29_28-OPcp29_38*RLcp29_29-OPcp29_39*RLcp29_210;
  AxF4[2] = qdd[2]-(2.0)*qd[13]*(OMcp29_112*ROcp29_912-OMcp29_312*ROcp29_712)+qdd[13]*ROcp29_812-OMcp29_110*ORcp29_311-
 OMcp29_112*ORcp29_313-OMcp29_16*ORcp29_37-OMcp29_17*ORcp29_38-OMcp29_18*ORcp29_39-OMcp29_19*ORcp29_310+OMcp29_310*ORcp29_111
 +OMcp29_312*ORcp29_113+OMcp29_36*ORcp29_17+OMcp29_37*ORcp29_18+OMcp29_38*ORcp29_19+OMcp29_39*ORcp29_110-OPcp29_110*
 RLcp29_311-OPcp29_112*RLcp29_313-OPcp29_16*RLcp29_37-OPcp29_17*RLcp29_38-OPcp29_18*RLcp29_39-OPcp29_19*RLcp29_310+OPcp29_310
 *RLcp29_111+OPcp29_312*RLcp29_113+OPcp29_36*RLcp29_17+OPcp29_37*RLcp29_18+OPcp29_38*RLcp29_19+OPcp29_39*RLcp29_110;
  AxF4[3] = qdd[3]+(2.0)*qd[13]*(OMcp29_112*ROcp29_812-OMcp29_212*ROcp29_712)+qdd[13]*ROcp29_912+OMcp29_110*ORcp29_211+
 OMcp29_112*ORcp29_213+OMcp29_16*ORcp29_27+OMcp29_17*ORcp29_28+OMcp29_18*ORcp29_29+OMcp29_19*ORcp29_210-OMcp29_210*ORcp29_111
 -OMcp29_212*ORcp29_113-OMcp29_26*ORcp29_17-OMcp29_27*ORcp29_18-OMcp29_28*ORcp29_19-OMcp29_29*ORcp29_110+OPcp29_110*
 RLcp29_211+OPcp29_112*RLcp29_213+OPcp29_16*RLcp29_27+OPcp29_17*RLcp29_28+OPcp29_18*RLcp29_29+OPcp29_19*RLcp29_210-OPcp29_210
 *RLcp29_111-OPcp29_212*RLcp29_113-OPcp29_26*RLcp29_17-OPcp29_27*RLcp29_18-OPcp29_28*RLcp29_19-OPcp29_29*RLcp29_110;
  OMPxF4[1] = OPcp29_112;
  OMPxF4[2] = OPcp29_212;
  OMPxF4[3] = OPcp29_312;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc130 = ROcp29_112*SWr4[1]+ROcp29_212*SWr4[2]+ROcp29_312*SWr4[3];
  xfrc230 = ROcp29_411*SWr4[1]+ROcp29_511*SWr4[2]+ROcp29_611*SWr4[3];
  xfrc330 = ROcp29_712*SWr4[1]+ROcp29_812*SWr4[2]+ROcp29_912*SWr4[3];
  frc[1][13] = s->frc[1][13]+xfrc130;
  frc[2][13] = s->frc[2][13]+xfrc230;
  frc[3][13] = s->frc[3][13]+xfrc330;
  xtrq130 = ROcp29_112*SWr4[4]+ROcp29_212*SWr4[5]+ROcp29_312*SWr4[6];
  xtrq230 = ROcp29_411*SWr4[4]+ROcp29_511*SWr4[5]+ROcp29_611*SWr4[6];
  xtrq330 = ROcp29_712*SWr4[4]+ROcp29_812*SWr4[5]+ROcp29_912*SWr4[6];
  trq[1][13] = s->trq[1][13]+xtrq130-xfrc230*SWr4[9]+xfrc330*SWr4[8];
  trq[2][13] = s->trq[2][13]+xtrq230+xfrc130*SWr4[9]-xfrc330*SWr4[7];
  trq[3][13] = s->trq[3][13]+xtrq330-xfrc130*SWr4[8]+xfrc230*SWr4[7];

// = = Block_0_0_1_5_0_1 = = 
 
// Sensor Kinematics 


  ROcp30_25 = S4*S5;
  ROcp30_35 = -C4*S5;
  ROcp30_85 = -S4*C5;
  ROcp30_95 = C4*C5;
  ROcp30_16 = C5*C6;
  ROcp30_26 = ROcp30_25*C6+C4*S6;
  ROcp30_36 = ROcp30_35*C6+S4*S6;
  ROcp30_46 = -C5*S6;
  ROcp30_56 = -(ROcp30_25*S6-C4*C6);
  ROcp30_66 = -(ROcp30_35*S6-S4*C6);
  OMcp30_25 = qd[5]*C4;
  OMcp30_35 = qd[5]*S4;
  OMcp30_16 = qd[4]+qd[6]*S5;
  OMcp30_26 = OMcp30_25+qd[6]*ROcp30_85;
  OMcp30_36 = OMcp30_35+qd[6]*ROcp30_95;
  OPcp30_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp30_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp30_95-OMcp30_35*S5)-qdd[5]*C4-qdd[6]*ROcp30_85);
  OPcp30_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp30_85-OMcp30_25*S5)+qdd[5]*S4+qdd[6]*ROcp30_95;

// = = Block_0_0_1_5_0_2 = = 
 
// Sensor Kinematics 


  ROcp30_17 = ROcp30_16*C7-S5*S7;
  ROcp30_27 = ROcp30_26*C7-ROcp30_85*S7;
  ROcp30_37 = ROcp30_36*C7-ROcp30_95*S7;
  ROcp30_77 = ROcp30_16*S7+S5*C7;
  ROcp30_87 = ROcp30_26*S7+ROcp30_85*C7;
  ROcp30_97 = ROcp30_36*S7+ROcp30_95*C7;
  ROcp30_48 = ROcp30_46*C8+ROcp30_77*S8;
  ROcp30_58 = ROcp30_56*C8+ROcp30_87*S8;
  ROcp30_68 = ROcp30_66*C8+ROcp30_97*S8;
  ROcp30_78 = -(ROcp30_46*S8-ROcp30_77*C8);
  ROcp30_88 = -(ROcp30_56*S8-ROcp30_87*C8);
  ROcp30_98 = -(ROcp30_66*S8-ROcp30_97*C8);
  ROcp30_19 = ROcp30_17*C9+ROcp30_48*S9;
  ROcp30_29 = ROcp30_27*C9+ROcp30_58*S9;
  ROcp30_39 = ROcp30_37*C9+ROcp30_68*S9;
  ROcp30_49 = -(ROcp30_17*S9-ROcp30_48*C9);
  ROcp30_59 = -(ROcp30_27*S9-ROcp30_58*C9);
  ROcp30_69 = -(ROcp30_37*S9-ROcp30_68*C9);
  ROcp30_110 = ROcp30_19*C10-ROcp30_78*S10;
  ROcp30_210 = ROcp30_29*C10-ROcp30_88*S10;
  ROcp30_310 = ROcp30_39*C10-ROcp30_98*S10;
  ROcp30_710 = ROcp30_19*S10+ROcp30_78*C10;
  ROcp30_810 = ROcp30_29*S10+ROcp30_88*C10;
  ROcp30_910 = ROcp30_39*S10+ROcp30_98*C10;
  ROcp30_411 = ROcp30_49*C11+ROcp30_710*S11;
  ROcp30_511 = ROcp30_59*C11+ROcp30_810*S11;
  ROcp30_611 = ROcp30_69*C11+ROcp30_910*S11;
  ROcp30_711 = -(ROcp30_49*S11-ROcp30_710*C11);
  ROcp30_811 = -(ROcp30_59*S11-ROcp30_810*C11);
  ROcp30_911 = -(ROcp30_69*S11-ROcp30_910*C11);
  ROcp30_112 = ROcp30_110*C12-ROcp30_711*S12;
  ROcp30_212 = ROcp30_210*C12-ROcp30_811*S12;
  ROcp30_312 = ROcp30_310*C12-ROcp30_911*S12;
  ROcp30_712 = ROcp30_110*S12+ROcp30_711*C12;
  ROcp30_812 = ROcp30_210*S12+ROcp30_811*C12;
  ROcp30_912 = ROcp30_310*S12+ROcp30_911*C12;
  RLcp30_17 = ROcp30_46*s->dpt[2][2];
  RLcp30_27 = ROcp30_56*s->dpt[2][2];
  RLcp30_37 = ROcp30_66*s->dpt[2][2];
  OMcp30_17 = OMcp30_16+qd[7]*ROcp30_46;
  OMcp30_27 = OMcp30_26+qd[7]*ROcp30_56;
  OMcp30_37 = OMcp30_36+qd[7]*ROcp30_66;
  ORcp30_17 = OMcp30_26*RLcp30_37-OMcp30_36*RLcp30_27;
  ORcp30_27 = -(OMcp30_16*RLcp30_37-OMcp30_36*RLcp30_17);
  ORcp30_37 = OMcp30_16*RLcp30_27-OMcp30_26*RLcp30_17;
  OPcp30_17 = OPcp30_16+qd[7]*(OMcp30_26*ROcp30_66-OMcp30_36*ROcp30_56)+qdd[7]*ROcp30_46;
  OPcp30_27 = OPcp30_26-qd[7]*(OMcp30_16*ROcp30_66-OMcp30_36*ROcp30_46)+qdd[7]*ROcp30_56;
  OPcp30_37 = OPcp30_36+qd[7]*(OMcp30_16*ROcp30_56-OMcp30_26*ROcp30_46)+qdd[7]*ROcp30_66;
  RLcp30_18 = ROcp30_46*s->dpt[2][6];
  RLcp30_28 = ROcp30_56*s->dpt[2][6];
  RLcp30_38 = ROcp30_66*s->dpt[2][6];
  OMcp30_18 = OMcp30_17+qd[8]*ROcp30_17;
  OMcp30_28 = OMcp30_27+qd[8]*ROcp30_27;
  OMcp30_38 = OMcp30_37+qd[8]*ROcp30_37;
  ORcp30_18 = OMcp30_27*RLcp30_38-OMcp30_37*RLcp30_28;
  ORcp30_28 = -(OMcp30_17*RLcp30_38-OMcp30_37*RLcp30_18);
  ORcp30_38 = OMcp30_17*RLcp30_28-OMcp30_27*RLcp30_18;
  OPcp30_18 = OPcp30_17+qd[8]*(OMcp30_27*ROcp30_37-OMcp30_37*ROcp30_27)+qdd[8]*ROcp30_17;
  OPcp30_28 = OPcp30_27-qd[8]*(OMcp30_17*ROcp30_37-OMcp30_37*ROcp30_17)+qdd[8]*ROcp30_27;
  OPcp30_38 = OPcp30_37+qd[8]*(OMcp30_17*ROcp30_27-OMcp30_27*ROcp30_17)+qdd[8]*ROcp30_37;
  RLcp30_19 = ROcp30_78*s->dpt[3][8];
  RLcp30_29 = ROcp30_88*s->dpt[3][8];
  RLcp30_39 = ROcp30_98*s->dpt[3][8];
  OMcp30_19 = OMcp30_18+qd[9]*ROcp30_78;
  OMcp30_29 = OMcp30_28+qd[9]*ROcp30_88;
  OMcp30_39 = OMcp30_38+qd[9]*ROcp30_98;
  ORcp30_19 = OMcp30_28*RLcp30_39-OMcp30_38*RLcp30_29;
  ORcp30_29 = -(OMcp30_18*RLcp30_39-OMcp30_38*RLcp30_19);
  ORcp30_39 = OMcp30_18*RLcp30_29-OMcp30_28*RLcp30_19;
  OPcp30_19 = OPcp30_18+qd[9]*(OMcp30_28*ROcp30_98-OMcp30_38*ROcp30_88)+qdd[9]*ROcp30_78;
  OPcp30_29 = OPcp30_28-qd[9]*(OMcp30_18*ROcp30_98-OMcp30_38*ROcp30_78)+qdd[9]*ROcp30_88;
  OPcp30_39 = OPcp30_38+qd[9]*(OMcp30_18*ROcp30_88-OMcp30_28*ROcp30_78)+qdd[9]*ROcp30_98;
  RLcp30_110 = ROcp30_78*s->dpt[3][11];
  RLcp30_210 = ROcp30_88*s->dpt[3][11];
  RLcp30_310 = ROcp30_98*s->dpt[3][11];
  OMcp30_110 = OMcp30_19+qd[10]*ROcp30_49;
  OMcp30_210 = OMcp30_29+qd[10]*ROcp30_59;
  OMcp30_310 = OMcp30_39+qd[10]*ROcp30_69;
  ORcp30_110 = OMcp30_29*RLcp30_310-OMcp30_39*RLcp30_210;
  ORcp30_210 = -(OMcp30_19*RLcp30_310-OMcp30_39*RLcp30_110);
  ORcp30_310 = OMcp30_19*RLcp30_210-OMcp30_29*RLcp30_110;
  OPcp30_110 = OPcp30_19+qd[10]*(OMcp30_29*ROcp30_69-OMcp30_39*ROcp30_59)+qdd[10]*ROcp30_49;
  OPcp30_210 = OPcp30_29-qd[10]*(OMcp30_19*ROcp30_69-OMcp30_39*ROcp30_49)+qdd[10]*ROcp30_59;
  OPcp30_310 = OPcp30_39+qd[10]*(OMcp30_19*ROcp30_59-OMcp30_29*ROcp30_49)+qdd[10]*ROcp30_69;
  RLcp30_111 = ROcp30_710*s->dpt[3][14];
  RLcp30_211 = ROcp30_810*s->dpt[3][14];
  RLcp30_311 = ROcp30_910*s->dpt[3][14];
  OMcp30_111 = OMcp30_110+qd[11]*ROcp30_110;
  OMcp30_211 = OMcp30_210+qd[11]*ROcp30_210;
  OMcp30_311 = OMcp30_310+qd[11]*ROcp30_310;
  ORcp30_111 = OMcp30_210*RLcp30_311-OMcp30_310*RLcp30_211;
  ORcp30_211 = -(OMcp30_110*RLcp30_311-OMcp30_310*RLcp30_111);
  ORcp30_311 = OMcp30_110*RLcp30_211-OMcp30_210*RLcp30_111;
  OMcp30_112 = OMcp30_111+qd[12]*ROcp30_411;
  OMcp30_212 = OMcp30_211+qd[12]*ROcp30_511;
  OMcp30_312 = OMcp30_311+qd[12]*ROcp30_611;
  OPcp30_112 = OPcp30_110+qd[11]*(OMcp30_210*ROcp30_310-OMcp30_310*ROcp30_210)+qd[12]*(OMcp30_211*ROcp30_611-OMcp30_311*
 ROcp30_511)+qdd[11]*ROcp30_110+qdd[12]*ROcp30_411;
  OPcp30_212 = OPcp30_210-qd[11]*(OMcp30_110*ROcp30_310-OMcp30_310*ROcp30_110)-qd[12]*(OMcp30_111*ROcp30_611-OMcp30_311*
 ROcp30_411)+qdd[11]*ROcp30_210+qdd[12]*ROcp30_511;
  OPcp30_312 = OPcp30_310+qd[11]*(OMcp30_110*ROcp30_210-OMcp30_210*ROcp30_110)+qd[12]*(OMcp30_111*ROcp30_511-OMcp30_211*
 ROcp30_411)+qdd[11]*ROcp30_310+qdd[12]*ROcp30_611;

// = = Block_0_0_1_5_0_4 = = 
 
// Sensor Kinematics 


  RLcp30_114 = Dz143*ROcp30_712+ROcp30_112*s->dpt[1][21];
  RLcp30_214 = Dz143*ROcp30_812+ROcp30_212*s->dpt[1][21];
  RLcp30_314 = Dz143*ROcp30_912+ROcp30_312*s->dpt[1][21];
  ORcp30_114 = OMcp30_212*RLcp30_314-OMcp30_312*RLcp30_214;
  ORcp30_214 = -(OMcp30_112*RLcp30_314-OMcp30_312*RLcp30_114);
  ORcp30_314 = OMcp30_112*RLcp30_214-OMcp30_212*RLcp30_114;
  PxF5[1] = q[1]+RLcp30_110+RLcp30_111+RLcp30_114+RLcp30_17+RLcp30_18+RLcp30_19;
  PxF5[2] = q[2]+RLcp30_210+RLcp30_211+RLcp30_214+RLcp30_27+RLcp30_28+RLcp30_29;
  PxF5[3] = q[3]+RLcp30_310+RLcp30_311+RLcp30_314+RLcp30_37+RLcp30_38+RLcp30_39;
  RxF5[1][1] = ROcp30_112;
  RxF5[1][2] = ROcp30_212;
  RxF5[1][3] = ROcp30_312;
  RxF5[2][1] = ROcp30_411;
  RxF5[2][2] = ROcp30_511;
  RxF5[2][3] = ROcp30_611;
  RxF5[3][1] = ROcp30_712;
  RxF5[3][2] = ROcp30_812;
  RxF5[3][3] = ROcp30_912;
  VxF5[1] = qd[1]+ORcp30_110+ORcp30_111+ORcp30_114+ORcp30_17+ORcp30_18+ORcp30_19+qd[14]*ROcp30_712;
  VxF5[2] = qd[2]+ORcp30_210+ORcp30_211+ORcp30_214+ORcp30_27+ORcp30_28+ORcp30_29+qd[14]*ROcp30_812;
  VxF5[3] = qd[3]+ORcp30_310+ORcp30_311+ORcp30_314+ORcp30_37+ORcp30_38+ORcp30_39+qd[14]*ROcp30_912;
  OMxF5[1] = OMcp30_112;
  OMxF5[2] = OMcp30_212;
  OMxF5[3] = OMcp30_312;
  AxF5[1] = qdd[1]+(2.0)*qd[14]*(OMcp30_212*ROcp30_912-OMcp30_312*ROcp30_812)+qdd[14]*ROcp30_712+OMcp30_210*ORcp30_311+
 OMcp30_212*ORcp30_314+OMcp30_26*ORcp30_37+OMcp30_27*ORcp30_38+OMcp30_28*ORcp30_39+OMcp30_29*ORcp30_310-OMcp30_310*ORcp30_211
 -OMcp30_312*ORcp30_214-OMcp30_36*ORcp30_27-OMcp30_37*ORcp30_28-OMcp30_38*ORcp30_29-OMcp30_39*ORcp30_210+OPcp30_210*
 RLcp30_311+OPcp30_212*RLcp30_314+OPcp30_26*RLcp30_37+OPcp30_27*RLcp30_38+OPcp30_28*RLcp30_39+OPcp30_29*RLcp30_310-OPcp30_310
 *RLcp30_211-OPcp30_312*RLcp30_214-OPcp30_36*RLcp30_27-OPcp30_37*RLcp30_28-OPcp30_38*RLcp30_29-OPcp30_39*RLcp30_210;
  AxF5[2] = qdd[2]-(2.0)*qd[14]*(OMcp30_112*ROcp30_912-OMcp30_312*ROcp30_712)+qdd[14]*ROcp30_812-OMcp30_110*ORcp30_311-
 OMcp30_112*ORcp30_314-OMcp30_16*ORcp30_37-OMcp30_17*ORcp30_38-OMcp30_18*ORcp30_39-OMcp30_19*ORcp30_310+OMcp30_310*ORcp30_111
 +OMcp30_312*ORcp30_114+OMcp30_36*ORcp30_17+OMcp30_37*ORcp30_18+OMcp30_38*ORcp30_19+OMcp30_39*ORcp30_110-OPcp30_110*
 RLcp30_311-OPcp30_112*RLcp30_314-OPcp30_16*RLcp30_37-OPcp30_17*RLcp30_38-OPcp30_18*RLcp30_39-OPcp30_19*RLcp30_310+OPcp30_310
 *RLcp30_111+OPcp30_312*RLcp30_114+OPcp30_36*RLcp30_17+OPcp30_37*RLcp30_18+OPcp30_38*RLcp30_19+OPcp30_39*RLcp30_110;
  AxF5[3] = qdd[3]+(2.0)*qd[14]*(OMcp30_112*ROcp30_812-OMcp30_212*ROcp30_712)+qdd[14]*ROcp30_912+OMcp30_110*ORcp30_211+
 OMcp30_112*ORcp30_214+OMcp30_16*ORcp30_27+OMcp30_17*ORcp30_28+OMcp30_18*ORcp30_29+OMcp30_19*ORcp30_210-OMcp30_210*ORcp30_111
 -OMcp30_212*ORcp30_114-OMcp30_26*ORcp30_17-OMcp30_27*ORcp30_18-OMcp30_28*ORcp30_19-OMcp30_29*ORcp30_110+OPcp30_110*
 RLcp30_211+OPcp30_112*RLcp30_214+OPcp30_16*RLcp30_27+OPcp30_17*RLcp30_28+OPcp30_18*RLcp30_29+OPcp30_19*RLcp30_210-OPcp30_210
 *RLcp30_111-OPcp30_212*RLcp30_114-OPcp30_26*RLcp30_17-OPcp30_27*RLcp30_18-OPcp30_28*RLcp30_19-OPcp30_29*RLcp30_110;
  OMPxF5[1] = OPcp30_112;
  OMPxF5[2] = OPcp30_212;
  OMPxF5[3] = OPcp30_312;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc131 = ROcp30_112*SWr5[1]+ROcp30_212*SWr5[2]+ROcp30_312*SWr5[3];
  xfrc231 = ROcp30_411*SWr5[1]+ROcp30_511*SWr5[2]+ROcp30_611*SWr5[3];
  xfrc331 = ROcp30_712*SWr5[1]+ROcp30_812*SWr5[2]+ROcp30_912*SWr5[3];
  frc[1][14] = s->frc[1][14]+xfrc131;
  frc[2][14] = s->frc[2][14]+xfrc231;
  frc[3][14] = s->frc[3][14]+xfrc331;
  xtrq131 = ROcp30_112*SWr5[4]+ROcp30_212*SWr5[5]+ROcp30_312*SWr5[6];
  xtrq231 = ROcp30_411*SWr5[4]+ROcp30_511*SWr5[5]+ROcp30_611*SWr5[6];
  xtrq331 = ROcp30_712*SWr5[4]+ROcp30_812*SWr5[5]+ROcp30_912*SWr5[6];
  trq[1][14] = s->trq[1][14]+xtrq131-xfrc231*SWr5[9]+xfrc331*SWr5[8];
  trq[2][14] = s->trq[2][14]+xtrq231+xfrc131*SWr5[9]-xfrc331*SWr5[7];
  trq[3][14] = s->trq[3][14]+xtrq331-xfrc131*SWr5[8]+xfrc231*SWr5[7];

// = = Block_0_0_1_6_0_1 = = 
 
// Sensor Kinematics 


  ROcp31_25 = S4*S5;
  ROcp31_35 = -C4*S5;
  ROcp31_85 = -S4*C5;
  ROcp31_95 = C4*C5;
  ROcp31_16 = C5*C6;
  ROcp31_26 = ROcp31_25*C6+C4*S6;
  ROcp31_36 = ROcp31_35*C6+S4*S6;
  ROcp31_46 = -C5*S6;
  ROcp31_56 = -(ROcp31_25*S6-C4*C6);
  ROcp31_66 = -(ROcp31_35*S6-S4*C6);
  OMcp31_25 = qd[5]*C4;
  OMcp31_35 = qd[5]*S4;
  OMcp31_16 = qd[4]+qd[6]*S5;
  OMcp31_26 = OMcp31_25+qd[6]*ROcp31_85;
  OMcp31_36 = OMcp31_35+qd[6]*ROcp31_95;
  OPcp31_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp31_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp31_95-OMcp31_35*S5)-qdd[5]*C4-qdd[6]*ROcp31_85);
  OPcp31_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp31_85-OMcp31_25*S5)+qdd[5]*S4+qdd[6]*ROcp31_95;

// = = Block_0_0_1_6_0_5 = = 
 
// Sensor Kinematics 


  ROcp31_115 = ROcp31_16*C15-S15*S5;
  ROcp31_215 = ROcp31_26*C15-ROcp31_85*S15;
  ROcp31_315 = ROcp31_36*C15-ROcp31_95*S15;
  ROcp31_715 = ROcp31_16*S15+C15*S5;
  ROcp31_815 = ROcp31_26*S15+ROcp31_85*C15;
  ROcp31_915 = ROcp31_36*S15+ROcp31_95*C15;
  ROcp31_416 = ROcp31_46*C16+ROcp31_715*S16;
  ROcp31_516 = ROcp31_56*C16+ROcp31_815*S16;
  ROcp31_616 = ROcp31_66*C16+ROcp31_915*S16;
  ROcp31_716 = -(ROcp31_46*S16-ROcp31_715*C16);
  ROcp31_816 = -(ROcp31_56*S16-ROcp31_815*C16);
  ROcp31_916 = -(ROcp31_66*S16-ROcp31_915*C16);
  ROcp31_117 = ROcp31_115*C17+ROcp31_416*S17;
  ROcp31_217 = ROcp31_215*C17+ROcp31_516*S17;
  ROcp31_317 = ROcp31_315*C17+ROcp31_616*S17;
  ROcp31_417 = -(ROcp31_115*S17-ROcp31_416*C17);
  ROcp31_517 = -(ROcp31_215*S17-ROcp31_516*C17);
  ROcp31_617 = -(ROcp31_315*S17-ROcp31_616*C17);
  RLcp31_115 = ROcp31_46*s->dpt[2][3];
  RLcp31_215 = ROcp31_56*s->dpt[2][3];
  RLcp31_315 = ROcp31_66*s->dpt[2][3];
  OMcp31_115 = OMcp31_16+qd[15]*ROcp31_46;
  OMcp31_215 = OMcp31_26+qd[15]*ROcp31_56;
  OMcp31_315 = OMcp31_36+qd[15]*ROcp31_66;
  ORcp31_115 = OMcp31_26*RLcp31_315-OMcp31_36*RLcp31_215;
  ORcp31_215 = -(OMcp31_16*RLcp31_315-OMcp31_36*RLcp31_115);
  ORcp31_315 = OMcp31_16*RLcp31_215-OMcp31_26*RLcp31_115;
  OPcp31_115 = OPcp31_16+qd[15]*(OMcp31_26*ROcp31_66-OMcp31_36*ROcp31_56)+qdd[15]*ROcp31_46;
  OPcp31_215 = OPcp31_26-qd[15]*(OMcp31_16*ROcp31_66-OMcp31_36*ROcp31_46)+qdd[15]*ROcp31_56;
  OPcp31_315 = OPcp31_36+qd[15]*(OMcp31_16*ROcp31_56-OMcp31_26*ROcp31_46)+qdd[15]*ROcp31_66;
  RLcp31_116 = ROcp31_46*s->dpt[2][24];
  RLcp31_216 = ROcp31_56*s->dpt[2][24];
  RLcp31_316 = ROcp31_66*s->dpt[2][24];
  OMcp31_116 = OMcp31_115+qd[16]*ROcp31_115;
  OMcp31_216 = OMcp31_215+qd[16]*ROcp31_215;
  OMcp31_316 = OMcp31_315+qd[16]*ROcp31_315;
  ORcp31_116 = OMcp31_215*RLcp31_316-OMcp31_315*RLcp31_216;
  ORcp31_216 = -(OMcp31_115*RLcp31_316-OMcp31_315*RLcp31_116);
  ORcp31_316 = OMcp31_115*RLcp31_216-OMcp31_215*RLcp31_116;
  OPcp31_116 = OPcp31_115+qd[16]*(OMcp31_215*ROcp31_315-OMcp31_315*ROcp31_215)+qdd[16]*ROcp31_115;
  OPcp31_216 = OPcp31_215-qd[16]*(OMcp31_115*ROcp31_315-OMcp31_315*ROcp31_115)+qdd[16]*ROcp31_215;
  OPcp31_316 = OPcp31_315+qd[16]*(OMcp31_115*ROcp31_215-OMcp31_215*ROcp31_115)+qdd[16]*ROcp31_315;
  RLcp31_117 = ROcp31_716*s->dpt[3][26];
  RLcp31_217 = ROcp31_816*s->dpt[3][26];
  RLcp31_317 = ROcp31_916*s->dpt[3][26];
  ORcp31_117 = OMcp31_216*RLcp31_317-OMcp31_316*RLcp31_217;
  ORcp31_217 = -(OMcp31_116*RLcp31_317-OMcp31_316*RLcp31_117);
  ORcp31_317 = OMcp31_116*RLcp31_217-OMcp31_216*RLcp31_117;
  PxF6[1] = q[1]+RLcp31_115+RLcp31_116+RLcp31_117;
  PxF6[2] = q[2]+RLcp31_215+RLcp31_216+RLcp31_217;
  PxF6[3] = q[3]+RLcp31_315+RLcp31_316+RLcp31_317;
  RxF6[1][1] = ROcp31_117;
  RxF6[1][2] = ROcp31_217;
  RxF6[1][3] = ROcp31_317;
  RxF6[2][1] = ROcp31_417;
  RxF6[2][2] = ROcp31_517;
  RxF6[2][3] = ROcp31_617;
  RxF6[3][1] = ROcp31_716;
  RxF6[3][2] = ROcp31_816;
  RxF6[3][3] = ROcp31_916;
  VxF6[1] = qd[1]+ORcp31_115+ORcp31_116+ORcp31_117;
  VxF6[2] = qd[2]+ORcp31_215+ORcp31_216+ORcp31_217;
  VxF6[3] = qd[3]+ORcp31_315+ORcp31_316+ORcp31_317;
  OMxF6[1] = OMcp31_116+qd[17]*ROcp31_716;
  OMxF6[2] = OMcp31_216+qd[17]*ROcp31_816;
  OMxF6[3] = OMcp31_316+qd[17]*ROcp31_916;
  AxF6[1] = qdd[1]+OMcp31_215*ORcp31_316+OMcp31_216*ORcp31_317+OMcp31_26*ORcp31_315-OMcp31_315*ORcp31_216-OMcp31_316*
 ORcp31_217-OMcp31_36*ORcp31_215+OPcp31_215*RLcp31_316+OPcp31_216*RLcp31_317+OPcp31_26*RLcp31_315-OPcp31_315*RLcp31_216-
 OPcp31_316*RLcp31_217-OPcp31_36*RLcp31_215;
  AxF6[2] = qdd[2]-OMcp31_115*ORcp31_316-OMcp31_116*ORcp31_317-OMcp31_16*ORcp31_315+OMcp31_315*ORcp31_116+OMcp31_316*
 ORcp31_117+OMcp31_36*ORcp31_115-OPcp31_115*RLcp31_316-OPcp31_116*RLcp31_317-OPcp31_16*RLcp31_315+OPcp31_315*RLcp31_116+
 OPcp31_316*RLcp31_117+OPcp31_36*RLcp31_115;
  AxF6[3] = qdd[3]+OMcp31_115*ORcp31_216+OMcp31_116*ORcp31_217+OMcp31_16*ORcp31_215-OMcp31_215*ORcp31_116-OMcp31_216*
 ORcp31_117-OMcp31_26*ORcp31_115+OPcp31_115*RLcp31_216+OPcp31_116*RLcp31_217+OPcp31_16*RLcp31_215-OPcp31_215*RLcp31_116-
 OPcp31_216*RLcp31_117-OPcp31_26*RLcp31_115;
  OMPxF6[1] = OPcp31_116+qd[17]*(OMcp31_216*ROcp31_916-OMcp31_316*ROcp31_816)+qdd[17]*ROcp31_716;
  OMPxF6[2] = OPcp31_216-qd[17]*(OMcp31_116*ROcp31_916-OMcp31_316*ROcp31_716)+qdd[17]*ROcp31_816;
  OMPxF6[3] = OPcp31_316+qd[17]*(OMcp31_116*ROcp31_816-OMcp31_216*ROcp31_716)+qdd[17]*ROcp31_916;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc132 = ROcp31_117*SWr6[1]+ROcp31_217*SWr6[2]+ROcp31_317*SWr6[3];
  xfrc232 = ROcp31_417*SWr6[1]+ROcp31_517*SWr6[2]+ROcp31_617*SWr6[3];
  xfrc332 = ROcp31_716*SWr6[1]+ROcp31_816*SWr6[2]+ROcp31_916*SWr6[3];
  frc[1][17] = s->frc[1][17]+xfrc132;
  frc[2][17] = s->frc[2][17]+xfrc232;
  frc[3][17] = s->frc[3][17]+xfrc332;
  xtrq132 = ROcp31_117*SWr6[4]+ROcp31_217*SWr6[5]+ROcp31_317*SWr6[6];
  xtrq232 = ROcp31_417*SWr6[4]+ROcp31_517*SWr6[5]+ROcp31_617*SWr6[6];
  xtrq332 = ROcp31_716*SWr6[4]+ROcp31_816*SWr6[5]+ROcp31_916*SWr6[6];
  trq[1][17] = s->trq[1][17]+xtrq132-xfrc232*(SWr6[9]-s->l[3][17])+xfrc332*(SWr6[8]-s->l[2][17]);
  trq[2][17] = s->trq[2][17]+xtrq232+xfrc132*(SWr6[9]-s->l[3][17])-xfrc332*(SWr6[7]-s->l[1][17]);
  trq[3][17] = s->trq[3][17]+xtrq332-xfrc132*(SWr6[8]-s->l[2][17])+xfrc232*(SWr6[7]-s->l[1][17]);

// = = Block_0_0_1_7_0_1 = = 
 
// Sensor Kinematics 


  ROcp32_25 = S4*S5;
  ROcp32_35 = -C4*S5;
  ROcp32_85 = -S4*C5;
  ROcp32_95 = C4*C5;
  ROcp32_16 = C5*C6;
  ROcp32_26 = ROcp32_25*C6+C4*S6;
  ROcp32_36 = ROcp32_35*C6+S4*S6;
  ROcp32_46 = -C5*S6;
  ROcp32_56 = -(ROcp32_25*S6-C4*C6);
  ROcp32_66 = -(ROcp32_35*S6-S4*C6);
  OMcp32_25 = qd[5]*C4;
  OMcp32_35 = qd[5]*S4;
  OMcp32_16 = qd[4]+qd[6]*S5;
  OMcp32_26 = OMcp32_25+qd[6]*ROcp32_85;
  OMcp32_36 = OMcp32_35+qd[6]*ROcp32_95;
  OPcp32_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp32_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp32_95-OMcp32_35*S5)-qdd[5]*C4-qdd[6]*ROcp32_85);
  OPcp32_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp32_85-OMcp32_25*S5)+qdd[5]*S4+qdd[6]*ROcp32_95;

// = = Block_0_0_1_7_0_5 = = 
 
// Sensor Kinematics 


  ROcp32_115 = ROcp32_16*C15-S15*S5;
  ROcp32_215 = ROcp32_26*C15-ROcp32_85*S15;
  ROcp32_315 = ROcp32_36*C15-ROcp32_95*S15;
  ROcp32_715 = ROcp32_16*S15+C15*S5;
  ROcp32_815 = ROcp32_26*S15+ROcp32_85*C15;
  ROcp32_915 = ROcp32_36*S15+ROcp32_95*C15;
  ROcp32_416 = ROcp32_46*C16+ROcp32_715*S16;
  ROcp32_516 = ROcp32_56*C16+ROcp32_815*S16;
  ROcp32_616 = ROcp32_66*C16+ROcp32_915*S16;
  ROcp32_716 = -(ROcp32_46*S16-ROcp32_715*C16);
  ROcp32_816 = -(ROcp32_56*S16-ROcp32_815*C16);
  ROcp32_916 = -(ROcp32_66*S16-ROcp32_915*C16);
  ROcp32_117 = ROcp32_115*C17+ROcp32_416*S17;
  ROcp32_217 = ROcp32_215*C17+ROcp32_516*S17;
  ROcp32_317 = ROcp32_315*C17+ROcp32_616*S17;
  ROcp32_417 = -(ROcp32_115*S17-ROcp32_416*C17);
  ROcp32_517 = -(ROcp32_215*S17-ROcp32_516*C17);
  ROcp32_617 = -(ROcp32_315*S17-ROcp32_616*C17);
  ROcp32_118 = ROcp32_117*C18-ROcp32_716*S18;
  ROcp32_218 = ROcp32_217*C18-ROcp32_816*S18;
  ROcp32_318 = ROcp32_317*C18-ROcp32_916*S18;
  ROcp32_718 = ROcp32_117*S18+ROcp32_716*C18;
  ROcp32_818 = ROcp32_217*S18+ROcp32_816*C18;
  ROcp32_918 = ROcp32_317*S18+ROcp32_916*C18;
  RLcp32_115 = ROcp32_46*s->dpt[2][3];
  RLcp32_215 = ROcp32_56*s->dpt[2][3];
  RLcp32_315 = ROcp32_66*s->dpt[2][3];
  OMcp32_115 = OMcp32_16+qd[15]*ROcp32_46;
  OMcp32_215 = OMcp32_26+qd[15]*ROcp32_56;
  OMcp32_315 = OMcp32_36+qd[15]*ROcp32_66;
  ORcp32_115 = OMcp32_26*RLcp32_315-OMcp32_36*RLcp32_215;
  ORcp32_215 = -(OMcp32_16*RLcp32_315-OMcp32_36*RLcp32_115);
  ORcp32_315 = OMcp32_16*RLcp32_215-OMcp32_26*RLcp32_115;
  OPcp32_115 = OPcp32_16+qd[15]*(OMcp32_26*ROcp32_66-OMcp32_36*ROcp32_56)+qdd[15]*ROcp32_46;
  OPcp32_215 = OPcp32_26-qd[15]*(OMcp32_16*ROcp32_66-OMcp32_36*ROcp32_46)+qdd[15]*ROcp32_56;
  OPcp32_315 = OPcp32_36+qd[15]*(OMcp32_16*ROcp32_56-OMcp32_26*ROcp32_46)+qdd[15]*ROcp32_66;
  RLcp32_116 = ROcp32_46*s->dpt[2][24];
  RLcp32_216 = ROcp32_56*s->dpt[2][24];
  RLcp32_316 = ROcp32_66*s->dpt[2][24];
  OMcp32_116 = OMcp32_115+qd[16]*ROcp32_115;
  OMcp32_216 = OMcp32_215+qd[16]*ROcp32_215;
  OMcp32_316 = OMcp32_315+qd[16]*ROcp32_315;
  ORcp32_116 = OMcp32_215*RLcp32_316-OMcp32_315*RLcp32_216;
  ORcp32_216 = -(OMcp32_115*RLcp32_316-OMcp32_315*RLcp32_116);
  ORcp32_316 = OMcp32_115*RLcp32_216-OMcp32_215*RLcp32_116;
  OPcp32_116 = OPcp32_115+qd[16]*(OMcp32_215*ROcp32_315-OMcp32_315*ROcp32_215)+qdd[16]*ROcp32_115;
  OPcp32_216 = OPcp32_215-qd[16]*(OMcp32_115*ROcp32_315-OMcp32_315*ROcp32_115)+qdd[16]*ROcp32_215;
  OPcp32_316 = OPcp32_315+qd[16]*(OMcp32_115*ROcp32_215-OMcp32_215*ROcp32_115)+qdd[16]*ROcp32_315;
  RLcp32_117 = ROcp32_716*s->dpt[3][26];
  RLcp32_217 = ROcp32_816*s->dpt[3][26];
  RLcp32_317 = ROcp32_916*s->dpt[3][26];
  OMcp32_117 = OMcp32_116+qd[17]*ROcp32_716;
  OMcp32_217 = OMcp32_216+qd[17]*ROcp32_816;
  OMcp32_317 = OMcp32_316+qd[17]*ROcp32_916;
  ORcp32_117 = OMcp32_216*RLcp32_317-OMcp32_316*RLcp32_217;
  ORcp32_217 = -(OMcp32_116*RLcp32_317-OMcp32_316*RLcp32_117);
  ORcp32_317 = OMcp32_116*RLcp32_217-OMcp32_216*RLcp32_117;
  OPcp32_117 = OPcp32_116+qd[17]*(OMcp32_216*ROcp32_916-OMcp32_316*ROcp32_816)+qdd[17]*ROcp32_716;
  OPcp32_217 = OPcp32_216-qd[17]*(OMcp32_116*ROcp32_916-OMcp32_316*ROcp32_716)+qdd[17]*ROcp32_816;
  OPcp32_317 = OPcp32_316+qd[17]*(OMcp32_116*ROcp32_816-OMcp32_216*ROcp32_716)+qdd[17]*ROcp32_916;
  RLcp32_118 = ROcp32_716*s->dpt[3][29];
  RLcp32_218 = ROcp32_816*s->dpt[3][29];
  RLcp32_318 = ROcp32_916*s->dpt[3][29];
  ORcp32_118 = OMcp32_217*RLcp32_318-OMcp32_317*RLcp32_218;
  ORcp32_218 = -(OMcp32_117*RLcp32_318-OMcp32_317*RLcp32_118);
  ORcp32_318 = OMcp32_117*RLcp32_218-OMcp32_217*RLcp32_118;
  PxF7[1] = q[1]+RLcp32_115+RLcp32_116+RLcp32_117+RLcp32_118;
  PxF7[2] = q[2]+RLcp32_215+RLcp32_216+RLcp32_217+RLcp32_218;
  PxF7[3] = q[3]+RLcp32_315+RLcp32_316+RLcp32_317+RLcp32_318;
  RxF7[1][1] = ROcp32_118;
  RxF7[1][2] = ROcp32_218;
  RxF7[1][3] = ROcp32_318;
  RxF7[2][1] = ROcp32_417;
  RxF7[2][2] = ROcp32_517;
  RxF7[2][3] = ROcp32_617;
  RxF7[3][1] = ROcp32_718;
  RxF7[3][2] = ROcp32_818;
  RxF7[3][3] = ROcp32_918;
  VxF7[1] = qd[1]+ORcp32_115+ORcp32_116+ORcp32_117+ORcp32_118;
  VxF7[2] = qd[2]+ORcp32_215+ORcp32_216+ORcp32_217+ORcp32_218;
  VxF7[3] = qd[3]+ORcp32_315+ORcp32_316+ORcp32_317+ORcp32_318;
  OMxF7[1] = OMcp32_117+qd[18]*ROcp32_417;
  OMxF7[2] = OMcp32_217+qd[18]*ROcp32_517;
  OMxF7[3] = OMcp32_317+qd[18]*ROcp32_617;
  AxF7[1] = qdd[1]+OMcp32_215*ORcp32_316+OMcp32_216*ORcp32_317+OMcp32_217*ORcp32_318+OMcp32_26*ORcp32_315-OMcp32_315*
 ORcp32_216-OMcp32_316*ORcp32_217-OMcp32_317*ORcp32_218-OMcp32_36*ORcp32_215+OPcp32_215*RLcp32_316+OPcp32_216*RLcp32_317+
 OPcp32_217*RLcp32_318+OPcp32_26*RLcp32_315-OPcp32_315*RLcp32_216-OPcp32_316*RLcp32_217-OPcp32_317*RLcp32_218-OPcp32_36*
 RLcp32_215;
  AxF7[2] = qdd[2]-OMcp32_115*ORcp32_316-OMcp32_116*ORcp32_317-OMcp32_117*ORcp32_318-OMcp32_16*ORcp32_315+OMcp32_315*
 ORcp32_116+OMcp32_316*ORcp32_117+OMcp32_317*ORcp32_118+OMcp32_36*ORcp32_115-OPcp32_115*RLcp32_316-OPcp32_116*RLcp32_317-
 OPcp32_117*RLcp32_318-OPcp32_16*RLcp32_315+OPcp32_315*RLcp32_116+OPcp32_316*RLcp32_117+OPcp32_317*RLcp32_118+OPcp32_36*
 RLcp32_115;
  AxF7[3] = qdd[3]+OMcp32_115*ORcp32_216+OMcp32_116*ORcp32_217+OMcp32_117*ORcp32_218+OMcp32_16*ORcp32_215-OMcp32_215*
 ORcp32_116-OMcp32_216*ORcp32_117-OMcp32_217*ORcp32_118-OMcp32_26*ORcp32_115+OPcp32_115*RLcp32_216+OPcp32_116*RLcp32_217+
 OPcp32_117*RLcp32_218+OPcp32_16*RLcp32_215-OPcp32_215*RLcp32_116-OPcp32_216*RLcp32_117-OPcp32_217*RLcp32_118-OPcp32_26*
 RLcp32_115;
  OMPxF7[1] = OPcp32_117+qd[18]*(OMcp32_217*ROcp32_617-OMcp32_317*ROcp32_517)+qdd[18]*ROcp32_417;
  OMPxF7[2] = OPcp32_217-qd[18]*(OMcp32_117*ROcp32_617-OMcp32_317*ROcp32_417)+qdd[18]*ROcp32_517;
  OMPxF7[3] = OPcp32_317+qd[18]*(OMcp32_117*ROcp32_517-OMcp32_217*ROcp32_417)+qdd[18]*ROcp32_617;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc133 = ROcp32_118*SWr7[1]+ROcp32_218*SWr7[2]+ROcp32_318*SWr7[3];
  xfrc233 = ROcp32_417*SWr7[1]+ROcp32_517*SWr7[2]+ROcp32_617*SWr7[3];
  xfrc333 = ROcp32_718*SWr7[1]+ROcp32_818*SWr7[2]+ROcp32_918*SWr7[3];
  frc[1][18] = s->frc[1][18]+xfrc133;
  frc[2][18] = s->frc[2][18]+xfrc233;
  frc[3][18] = s->frc[3][18]+xfrc333;
  xtrq133 = ROcp32_118*SWr7[4]+ROcp32_218*SWr7[5]+ROcp32_318*SWr7[6];
  xtrq233 = ROcp32_417*SWr7[4]+ROcp32_517*SWr7[5]+ROcp32_617*SWr7[6];
  xtrq333 = ROcp32_718*SWr7[4]+ROcp32_818*SWr7[5]+ROcp32_918*SWr7[6];
  trq[1][18] = s->trq[1][18]+xtrq133-xfrc233*(SWr7[9]-s->l[3][18])+xfrc333*(SWr7[8]-s->l[2][18]);
  trq[2][18] = s->trq[2][18]+xtrq233+xfrc133*(SWr7[9]-s->l[3][18])-xfrc333*(SWr7[7]-s->l[1][18]);
  trq[3][18] = s->trq[3][18]+xtrq333-xfrc133*(SWr7[8]-s->l[2][18])+xfrc233*(SWr7[7]-s->l[1][18]);

// = = Block_0_0_1_8_0_1 = = 
 
// Sensor Kinematics 


  ROcp33_25 = S4*S5;
  ROcp33_35 = -C4*S5;
  ROcp33_85 = -S4*C5;
  ROcp33_95 = C4*C5;
  ROcp33_16 = C5*C6;
  ROcp33_26 = ROcp33_25*C6+C4*S6;
  ROcp33_36 = ROcp33_35*C6+S4*S6;
  ROcp33_46 = -C5*S6;
  ROcp33_56 = -(ROcp33_25*S6-C4*C6);
  ROcp33_66 = -(ROcp33_35*S6-S4*C6);
  OMcp33_25 = qd[5]*C4;
  OMcp33_35 = qd[5]*S4;
  OMcp33_16 = qd[4]+qd[6]*S5;
  OMcp33_26 = OMcp33_25+qd[6]*ROcp33_85;
  OMcp33_36 = OMcp33_35+qd[6]*ROcp33_95;
  OPcp33_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp33_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp33_95-OMcp33_35*S5)-qdd[5]*C4-qdd[6]*ROcp33_85);
  OPcp33_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp33_85-OMcp33_25*S5)+qdd[5]*S4+qdd[6]*ROcp33_95;

// = = Block_0_0_1_8_0_5 = = 
 
// Sensor Kinematics 


  ROcp33_115 = ROcp33_16*C15-S15*S5;
  ROcp33_215 = ROcp33_26*C15-ROcp33_85*S15;
  ROcp33_315 = ROcp33_36*C15-ROcp33_95*S15;
  ROcp33_715 = ROcp33_16*S15+C15*S5;
  ROcp33_815 = ROcp33_26*S15+ROcp33_85*C15;
  ROcp33_915 = ROcp33_36*S15+ROcp33_95*C15;
  ROcp33_416 = ROcp33_46*C16+ROcp33_715*S16;
  ROcp33_516 = ROcp33_56*C16+ROcp33_815*S16;
  ROcp33_616 = ROcp33_66*C16+ROcp33_915*S16;
  ROcp33_716 = -(ROcp33_46*S16-ROcp33_715*C16);
  ROcp33_816 = -(ROcp33_56*S16-ROcp33_815*C16);
  ROcp33_916 = -(ROcp33_66*S16-ROcp33_915*C16);
  ROcp33_117 = ROcp33_115*C17+ROcp33_416*S17;
  ROcp33_217 = ROcp33_215*C17+ROcp33_516*S17;
  ROcp33_317 = ROcp33_315*C17+ROcp33_616*S17;
  ROcp33_417 = -(ROcp33_115*S17-ROcp33_416*C17);
  ROcp33_517 = -(ROcp33_215*S17-ROcp33_516*C17);
  ROcp33_617 = -(ROcp33_315*S17-ROcp33_616*C17);
  ROcp33_118 = ROcp33_117*C18-ROcp33_716*S18;
  ROcp33_218 = ROcp33_217*C18-ROcp33_816*S18;
  ROcp33_318 = ROcp33_317*C18-ROcp33_916*S18;
  ROcp33_718 = ROcp33_117*S18+ROcp33_716*C18;
  ROcp33_818 = ROcp33_217*S18+ROcp33_816*C18;
  ROcp33_918 = ROcp33_317*S18+ROcp33_916*C18;
  ROcp33_419 = ROcp33_417*C19+ROcp33_718*S19;
  ROcp33_519 = ROcp33_517*C19+ROcp33_818*S19;
  ROcp33_619 = ROcp33_617*C19+ROcp33_918*S19;
  ROcp33_719 = -(ROcp33_417*S19-ROcp33_718*C19);
  ROcp33_819 = -(ROcp33_517*S19-ROcp33_818*C19);
  ROcp33_919 = -(ROcp33_617*S19-ROcp33_918*C19);
  ROcp33_120 = ROcp33_118*C20-ROcp33_719*S20;
  ROcp33_220 = ROcp33_218*C20-ROcp33_819*S20;
  ROcp33_320 = ROcp33_318*C20-ROcp33_919*S20;
  ROcp33_720 = ROcp33_118*S20+ROcp33_719*C20;
  ROcp33_820 = ROcp33_218*S20+ROcp33_819*C20;
  ROcp33_920 = ROcp33_318*S20+ROcp33_919*C20;
  RLcp33_115 = ROcp33_46*s->dpt[2][3];
  RLcp33_215 = ROcp33_56*s->dpt[2][3];
  RLcp33_315 = ROcp33_66*s->dpt[2][3];
  OMcp33_115 = OMcp33_16+qd[15]*ROcp33_46;
  OMcp33_215 = OMcp33_26+qd[15]*ROcp33_56;
  OMcp33_315 = OMcp33_36+qd[15]*ROcp33_66;
  ORcp33_115 = OMcp33_26*RLcp33_315-OMcp33_36*RLcp33_215;
  ORcp33_215 = -(OMcp33_16*RLcp33_315-OMcp33_36*RLcp33_115);
  ORcp33_315 = OMcp33_16*RLcp33_215-OMcp33_26*RLcp33_115;
  OPcp33_115 = OPcp33_16+qd[15]*(OMcp33_26*ROcp33_66-OMcp33_36*ROcp33_56)+qdd[15]*ROcp33_46;
  OPcp33_215 = OPcp33_26-qd[15]*(OMcp33_16*ROcp33_66-OMcp33_36*ROcp33_46)+qdd[15]*ROcp33_56;
  OPcp33_315 = OPcp33_36+qd[15]*(OMcp33_16*ROcp33_56-OMcp33_26*ROcp33_46)+qdd[15]*ROcp33_66;
  RLcp33_116 = ROcp33_46*s->dpt[2][24];
  RLcp33_216 = ROcp33_56*s->dpt[2][24];
  RLcp33_316 = ROcp33_66*s->dpt[2][24];
  OMcp33_116 = OMcp33_115+qd[16]*ROcp33_115;
  OMcp33_216 = OMcp33_215+qd[16]*ROcp33_215;
  OMcp33_316 = OMcp33_315+qd[16]*ROcp33_315;
  ORcp33_116 = OMcp33_215*RLcp33_316-OMcp33_315*RLcp33_216;
  ORcp33_216 = -(OMcp33_115*RLcp33_316-OMcp33_315*RLcp33_116);
  ORcp33_316 = OMcp33_115*RLcp33_216-OMcp33_215*RLcp33_116;
  OPcp33_116 = OPcp33_115+qd[16]*(OMcp33_215*ROcp33_315-OMcp33_315*ROcp33_215)+qdd[16]*ROcp33_115;
  OPcp33_216 = OPcp33_215-qd[16]*(OMcp33_115*ROcp33_315-OMcp33_315*ROcp33_115)+qdd[16]*ROcp33_215;
  OPcp33_316 = OPcp33_315+qd[16]*(OMcp33_115*ROcp33_215-OMcp33_215*ROcp33_115)+qdd[16]*ROcp33_315;
  RLcp33_117 = ROcp33_716*s->dpt[3][26];
  RLcp33_217 = ROcp33_816*s->dpt[3][26];
  RLcp33_317 = ROcp33_916*s->dpt[3][26];
  OMcp33_117 = OMcp33_116+qd[17]*ROcp33_716;
  OMcp33_217 = OMcp33_216+qd[17]*ROcp33_816;
  OMcp33_317 = OMcp33_316+qd[17]*ROcp33_916;
  ORcp33_117 = OMcp33_216*RLcp33_317-OMcp33_316*RLcp33_217;
  ORcp33_217 = -(OMcp33_116*RLcp33_317-OMcp33_316*RLcp33_117);
  ORcp33_317 = OMcp33_116*RLcp33_217-OMcp33_216*RLcp33_117;
  OPcp33_117 = OPcp33_116+qd[17]*(OMcp33_216*ROcp33_916-OMcp33_316*ROcp33_816)+qdd[17]*ROcp33_716;
  OPcp33_217 = OPcp33_216-qd[17]*(OMcp33_116*ROcp33_916-OMcp33_316*ROcp33_716)+qdd[17]*ROcp33_816;
  OPcp33_317 = OPcp33_316+qd[17]*(OMcp33_116*ROcp33_816-OMcp33_216*ROcp33_716)+qdd[17]*ROcp33_916;
  RLcp33_118 = ROcp33_716*s->dpt[3][29];
  RLcp33_218 = ROcp33_816*s->dpt[3][29];
  RLcp33_318 = ROcp33_916*s->dpt[3][29];
  OMcp33_118 = OMcp33_117+qd[18]*ROcp33_417;
  OMcp33_218 = OMcp33_217+qd[18]*ROcp33_517;
  OMcp33_318 = OMcp33_317+qd[18]*ROcp33_617;
  ORcp33_118 = OMcp33_217*RLcp33_318-OMcp33_317*RLcp33_218;
  ORcp33_218 = -(OMcp33_117*RLcp33_318-OMcp33_317*RLcp33_118);
  ORcp33_318 = OMcp33_117*RLcp33_218-OMcp33_217*RLcp33_118;
  OPcp33_118 = OPcp33_117+qd[18]*(OMcp33_217*ROcp33_617-OMcp33_317*ROcp33_517)+qdd[18]*ROcp33_417;
  OPcp33_218 = OPcp33_217-qd[18]*(OMcp33_117*ROcp33_617-OMcp33_317*ROcp33_417)+qdd[18]*ROcp33_517;
  OPcp33_318 = OPcp33_317+qd[18]*(OMcp33_117*ROcp33_517-OMcp33_217*ROcp33_417)+qdd[18]*ROcp33_617;
  RLcp33_119 = ROcp33_718*s->dpt[3][32];
  RLcp33_219 = ROcp33_818*s->dpt[3][32];
  RLcp33_319 = ROcp33_918*s->dpt[3][32];
  OMcp33_119 = OMcp33_118+qd[19]*ROcp33_118;
  OMcp33_219 = OMcp33_218+qd[19]*ROcp33_218;
  OMcp33_319 = OMcp33_318+qd[19]*ROcp33_318;
  ORcp33_119 = OMcp33_218*RLcp33_319-OMcp33_318*RLcp33_219;
  ORcp33_219 = -(OMcp33_118*RLcp33_319-OMcp33_318*RLcp33_119);
  ORcp33_319 = OMcp33_118*RLcp33_219-OMcp33_218*RLcp33_119;
  OMcp33_120 = OMcp33_119+qd[20]*ROcp33_419;
  OMcp33_220 = OMcp33_219+qd[20]*ROcp33_519;
  OMcp33_320 = OMcp33_319+qd[20]*ROcp33_619;
  OPcp33_120 = OPcp33_118+qd[19]*(OMcp33_218*ROcp33_318-OMcp33_318*ROcp33_218)+qd[20]*(OMcp33_219*ROcp33_619-OMcp33_319*
 ROcp33_519)+qdd[19]*ROcp33_118+qdd[20]*ROcp33_419;
  OPcp33_220 = OPcp33_218-qd[19]*(OMcp33_118*ROcp33_318-OMcp33_318*ROcp33_118)-qd[20]*(OMcp33_119*ROcp33_619-OMcp33_319*
 ROcp33_419)+qdd[19]*ROcp33_218+qdd[20]*ROcp33_519;
  OPcp33_320 = OPcp33_318+qd[19]*(OMcp33_118*ROcp33_218-OMcp33_218*ROcp33_118)+qd[20]*(OMcp33_119*ROcp33_519-OMcp33_219*
 ROcp33_419)+qdd[19]*ROcp33_318+qdd[20]*ROcp33_619;

// = = Block_0_0_1_8_0_6 = = 
 
// Sensor Kinematics 


  RLcp33_121 = Dz213*ROcp33_720+ROcp33_120*s->dpt[1][38];
  RLcp33_221 = Dz213*ROcp33_820+ROcp33_220*s->dpt[1][38];
  RLcp33_321 = Dz213*ROcp33_920+ROcp33_320*s->dpt[1][38];
  ORcp33_121 = OMcp33_220*RLcp33_321-OMcp33_320*RLcp33_221;
  ORcp33_221 = -(OMcp33_120*RLcp33_321-OMcp33_320*RLcp33_121);
  ORcp33_321 = OMcp33_120*RLcp33_221-OMcp33_220*RLcp33_121;
  PxF8[1] = q[1]+RLcp33_115+RLcp33_116+RLcp33_117+RLcp33_118+RLcp33_119+RLcp33_121;
  PxF8[2] = q[2]+RLcp33_215+RLcp33_216+RLcp33_217+RLcp33_218+RLcp33_219+RLcp33_221;
  PxF8[3] = q[3]+RLcp33_315+RLcp33_316+RLcp33_317+RLcp33_318+RLcp33_319+RLcp33_321;
  RxF8[1][1] = ROcp33_120;
  RxF8[1][2] = ROcp33_220;
  RxF8[1][3] = ROcp33_320;
  RxF8[2][1] = ROcp33_419;
  RxF8[2][2] = ROcp33_519;
  RxF8[2][3] = ROcp33_619;
  RxF8[3][1] = ROcp33_720;
  RxF8[3][2] = ROcp33_820;
  RxF8[3][3] = ROcp33_920;
  VxF8[1] = qd[1]+ORcp33_115+ORcp33_116+ORcp33_117+ORcp33_118+ORcp33_119+ORcp33_121+qd[21]*ROcp33_720;
  VxF8[2] = qd[2]+ORcp33_215+ORcp33_216+ORcp33_217+ORcp33_218+ORcp33_219+ORcp33_221+qd[21]*ROcp33_820;
  VxF8[3] = qd[3]+ORcp33_315+ORcp33_316+ORcp33_317+ORcp33_318+ORcp33_319+ORcp33_321+qd[21]*ROcp33_920;
  OMxF8[1] = OMcp33_120;
  OMxF8[2] = OMcp33_220;
  OMxF8[3] = OMcp33_320;
  AxF8[1] = qdd[1]+(2.0)*qd[21]*(OMcp33_220*ROcp33_920-OMcp33_320*ROcp33_820)+qdd[21]*ROcp33_720+OMcp33_215*ORcp33_316+
 OMcp33_216*ORcp33_317+OMcp33_217*ORcp33_318+OMcp33_218*ORcp33_319+OMcp33_220*ORcp33_321+OMcp33_26*ORcp33_315-OMcp33_315*
 ORcp33_216-OMcp33_316*ORcp33_217-OMcp33_317*ORcp33_218-OMcp33_318*ORcp33_219-OMcp33_320*ORcp33_221-OMcp33_36*ORcp33_215+
 OPcp33_215*RLcp33_316+OPcp33_216*RLcp33_317+OPcp33_217*RLcp33_318+OPcp33_218*RLcp33_319+OPcp33_220*RLcp33_321+OPcp33_26*
 RLcp33_315-OPcp33_315*RLcp33_216-OPcp33_316*RLcp33_217-OPcp33_317*RLcp33_218-OPcp33_318*RLcp33_219-OPcp33_320*RLcp33_221-
 OPcp33_36*RLcp33_215;
  AxF8[2] = qdd[2]-(2.0)*qd[21]*(OMcp33_120*ROcp33_920-OMcp33_320*ROcp33_720)+qdd[21]*ROcp33_820-OMcp33_115*ORcp33_316-
 OMcp33_116*ORcp33_317-OMcp33_117*ORcp33_318-OMcp33_118*ORcp33_319-OMcp33_120*ORcp33_321-OMcp33_16*ORcp33_315+OMcp33_315*
 ORcp33_116+OMcp33_316*ORcp33_117+OMcp33_317*ORcp33_118+OMcp33_318*ORcp33_119+OMcp33_320*ORcp33_121+OMcp33_36*ORcp33_115-
 OPcp33_115*RLcp33_316-OPcp33_116*RLcp33_317-OPcp33_117*RLcp33_318-OPcp33_118*RLcp33_319-OPcp33_120*RLcp33_321-OPcp33_16*
 RLcp33_315+OPcp33_315*RLcp33_116+OPcp33_316*RLcp33_117+OPcp33_317*RLcp33_118+OPcp33_318*RLcp33_119+OPcp33_320*RLcp33_121+
 OPcp33_36*RLcp33_115;
  AxF8[3] = qdd[3]+(2.0)*qd[21]*(OMcp33_120*ROcp33_820-OMcp33_220*ROcp33_720)+qdd[21]*ROcp33_920+OMcp33_115*ORcp33_216+
 OMcp33_116*ORcp33_217+OMcp33_117*ORcp33_218+OMcp33_118*ORcp33_219+OMcp33_120*ORcp33_221+OMcp33_16*ORcp33_215-OMcp33_215*
 ORcp33_116-OMcp33_216*ORcp33_117-OMcp33_217*ORcp33_118-OMcp33_218*ORcp33_119-OMcp33_220*ORcp33_121-OMcp33_26*ORcp33_115+
 OPcp33_115*RLcp33_216+OPcp33_116*RLcp33_217+OPcp33_117*RLcp33_218+OPcp33_118*RLcp33_219+OPcp33_120*RLcp33_221+OPcp33_16*
 RLcp33_215-OPcp33_215*RLcp33_116-OPcp33_216*RLcp33_117-OPcp33_217*RLcp33_118-OPcp33_218*RLcp33_119-OPcp33_220*RLcp33_121-
 OPcp33_26*RLcp33_115;
  OMPxF8[1] = OPcp33_120;
  OMPxF8[2] = OPcp33_220;
  OMPxF8[3] = OPcp33_320;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc134 = ROcp33_120*SWr8[1]+ROcp33_220*SWr8[2]+ROcp33_320*SWr8[3];
  xfrc234 = ROcp33_419*SWr8[1]+ROcp33_519*SWr8[2]+ROcp33_619*SWr8[3];
  xfrc334 = ROcp33_720*SWr8[1]+ROcp33_820*SWr8[2]+ROcp33_920*SWr8[3];
  frc[1][21] = s->frc[1][21]+xfrc134;
  frc[2][21] = s->frc[2][21]+xfrc234;
  frc[3][21] = s->frc[3][21]+xfrc334;
  xtrq134 = ROcp33_120*SWr8[4]+ROcp33_220*SWr8[5]+ROcp33_320*SWr8[6];
  xtrq234 = ROcp33_419*SWr8[4]+ROcp33_519*SWr8[5]+ROcp33_619*SWr8[6];
  xtrq334 = ROcp33_720*SWr8[4]+ROcp33_820*SWr8[5]+ROcp33_920*SWr8[6];
  trq[1][21] = s->trq[1][21]+xtrq134-xfrc234*SWr8[9]+xfrc334*SWr8[8];
  trq[2][21] = s->trq[2][21]+xtrq234+xfrc134*SWr8[9]-xfrc334*SWr8[7];
  trq[3][21] = s->trq[3][21]+xtrq334-xfrc134*SWr8[8]+xfrc234*SWr8[7];

// = = Block_0_0_1_9_0_1 = = 
 
// Sensor Kinematics 


  ROcp34_25 = S4*S5;
  ROcp34_35 = -C4*S5;
  ROcp34_85 = -S4*C5;
  ROcp34_95 = C4*C5;
  ROcp34_16 = C5*C6;
  ROcp34_26 = ROcp34_25*C6+C4*S6;
  ROcp34_36 = ROcp34_35*C6+S4*S6;
  ROcp34_46 = -C5*S6;
  ROcp34_56 = -(ROcp34_25*S6-C4*C6);
  ROcp34_66 = -(ROcp34_35*S6-S4*C6);
  OMcp34_25 = qd[5]*C4;
  OMcp34_35 = qd[5]*S4;
  OMcp34_16 = qd[4]+qd[6]*S5;
  OMcp34_26 = OMcp34_25+qd[6]*ROcp34_85;
  OMcp34_36 = OMcp34_35+qd[6]*ROcp34_95;
  OPcp34_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp34_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp34_95-OMcp34_35*S5)-qdd[5]*C4-qdd[6]*ROcp34_85);
  OPcp34_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp34_85-OMcp34_25*S5)+qdd[5]*S4+qdd[6]*ROcp34_95;

// = = Block_0_0_1_9_0_5 = = 
 
// Sensor Kinematics 


  ROcp34_115 = ROcp34_16*C15-S15*S5;
  ROcp34_215 = ROcp34_26*C15-ROcp34_85*S15;
  ROcp34_315 = ROcp34_36*C15-ROcp34_95*S15;
  ROcp34_715 = ROcp34_16*S15+C15*S5;
  ROcp34_815 = ROcp34_26*S15+ROcp34_85*C15;
  ROcp34_915 = ROcp34_36*S15+ROcp34_95*C15;
  ROcp34_416 = ROcp34_46*C16+ROcp34_715*S16;
  ROcp34_516 = ROcp34_56*C16+ROcp34_815*S16;
  ROcp34_616 = ROcp34_66*C16+ROcp34_915*S16;
  ROcp34_716 = -(ROcp34_46*S16-ROcp34_715*C16);
  ROcp34_816 = -(ROcp34_56*S16-ROcp34_815*C16);
  ROcp34_916 = -(ROcp34_66*S16-ROcp34_915*C16);
  ROcp34_117 = ROcp34_115*C17+ROcp34_416*S17;
  ROcp34_217 = ROcp34_215*C17+ROcp34_516*S17;
  ROcp34_317 = ROcp34_315*C17+ROcp34_616*S17;
  ROcp34_417 = -(ROcp34_115*S17-ROcp34_416*C17);
  ROcp34_517 = -(ROcp34_215*S17-ROcp34_516*C17);
  ROcp34_617 = -(ROcp34_315*S17-ROcp34_616*C17);
  ROcp34_118 = ROcp34_117*C18-ROcp34_716*S18;
  ROcp34_218 = ROcp34_217*C18-ROcp34_816*S18;
  ROcp34_318 = ROcp34_317*C18-ROcp34_916*S18;
  ROcp34_718 = ROcp34_117*S18+ROcp34_716*C18;
  ROcp34_818 = ROcp34_217*S18+ROcp34_816*C18;
  ROcp34_918 = ROcp34_317*S18+ROcp34_916*C18;
  ROcp34_419 = ROcp34_417*C19+ROcp34_718*S19;
  ROcp34_519 = ROcp34_517*C19+ROcp34_818*S19;
  ROcp34_619 = ROcp34_617*C19+ROcp34_918*S19;
  ROcp34_719 = -(ROcp34_417*S19-ROcp34_718*C19);
  ROcp34_819 = -(ROcp34_517*S19-ROcp34_818*C19);
  ROcp34_919 = -(ROcp34_617*S19-ROcp34_918*C19);
  ROcp34_120 = ROcp34_118*C20-ROcp34_719*S20;
  ROcp34_220 = ROcp34_218*C20-ROcp34_819*S20;
  ROcp34_320 = ROcp34_318*C20-ROcp34_919*S20;
  ROcp34_720 = ROcp34_118*S20+ROcp34_719*C20;
  ROcp34_820 = ROcp34_218*S20+ROcp34_819*C20;
  ROcp34_920 = ROcp34_318*S20+ROcp34_919*C20;
  RLcp34_115 = ROcp34_46*s->dpt[2][3];
  RLcp34_215 = ROcp34_56*s->dpt[2][3];
  RLcp34_315 = ROcp34_66*s->dpt[2][3];
  OMcp34_115 = OMcp34_16+qd[15]*ROcp34_46;
  OMcp34_215 = OMcp34_26+qd[15]*ROcp34_56;
  OMcp34_315 = OMcp34_36+qd[15]*ROcp34_66;
  ORcp34_115 = OMcp34_26*RLcp34_315-OMcp34_36*RLcp34_215;
  ORcp34_215 = -(OMcp34_16*RLcp34_315-OMcp34_36*RLcp34_115);
  ORcp34_315 = OMcp34_16*RLcp34_215-OMcp34_26*RLcp34_115;
  OPcp34_115 = OPcp34_16+qd[15]*(OMcp34_26*ROcp34_66-OMcp34_36*ROcp34_56)+qdd[15]*ROcp34_46;
  OPcp34_215 = OPcp34_26-qd[15]*(OMcp34_16*ROcp34_66-OMcp34_36*ROcp34_46)+qdd[15]*ROcp34_56;
  OPcp34_315 = OPcp34_36+qd[15]*(OMcp34_16*ROcp34_56-OMcp34_26*ROcp34_46)+qdd[15]*ROcp34_66;
  RLcp34_116 = ROcp34_46*s->dpt[2][24];
  RLcp34_216 = ROcp34_56*s->dpt[2][24];
  RLcp34_316 = ROcp34_66*s->dpt[2][24];
  OMcp34_116 = OMcp34_115+qd[16]*ROcp34_115;
  OMcp34_216 = OMcp34_215+qd[16]*ROcp34_215;
  OMcp34_316 = OMcp34_315+qd[16]*ROcp34_315;
  ORcp34_116 = OMcp34_215*RLcp34_316-OMcp34_315*RLcp34_216;
  ORcp34_216 = -(OMcp34_115*RLcp34_316-OMcp34_315*RLcp34_116);
  ORcp34_316 = OMcp34_115*RLcp34_216-OMcp34_215*RLcp34_116;
  OPcp34_116 = OPcp34_115+qd[16]*(OMcp34_215*ROcp34_315-OMcp34_315*ROcp34_215)+qdd[16]*ROcp34_115;
  OPcp34_216 = OPcp34_215-qd[16]*(OMcp34_115*ROcp34_315-OMcp34_315*ROcp34_115)+qdd[16]*ROcp34_215;
  OPcp34_316 = OPcp34_315+qd[16]*(OMcp34_115*ROcp34_215-OMcp34_215*ROcp34_115)+qdd[16]*ROcp34_315;
  RLcp34_117 = ROcp34_716*s->dpt[3][26];
  RLcp34_217 = ROcp34_816*s->dpt[3][26];
  RLcp34_317 = ROcp34_916*s->dpt[3][26];
  OMcp34_117 = OMcp34_116+qd[17]*ROcp34_716;
  OMcp34_217 = OMcp34_216+qd[17]*ROcp34_816;
  OMcp34_317 = OMcp34_316+qd[17]*ROcp34_916;
  ORcp34_117 = OMcp34_216*RLcp34_317-OMcp34_316*RLcp34_217;
  ORcp34_217 = -(OMcp34_116*RLcp34_317-OMcp34_316*RLcp34_117);
  ORcp34_317 = OMcp34_116*RLcp34_217-OMcp34_216*RLcp34_117;
  OPcp34_117 = OPcp34_116+qd[17]*(OMcp34_216*ROcp34_916-OMcp34_316*ROcp34_816)+qdd[17]*ROcp34_716;
  OPcp34_217 = OPcp34_216-qd[17]*(OMcp34_116*ROcp34_916-OMcp34_316*ROcp34_716)+qdd[17]*ROcp34_816;
  OPcp34_317 = OPcp34_316+qd[17]*(OMcp34_116*ROcp34_816-OMcp34_216*ROcp34_716)+qdd[17]*ROcp34_916;
  RLcp34_118 = ROcp34_716*s->dpt[3][29];
  RLcp34_218 = ROcp34_816*s->dpt[3][29];
  RLcp34_318 = ROcp34_916*s->dpt[3][29];
  OMcp34_118 = OMcp34_117+qd[18]*ROcp34_417;
  OMcp34_218 = OMcp34_217+qd[18]*ROcp34_517;
  OMcp34_318 = OMcp34_317+qd[18]*ROcp34_617;
  ORcp34_118 = OMcp34_217*RLcp34_318-OMcp34_317*RLcp34_218;
  ORcp34_218 = -(OMcp34_117*RLcp34_318-OMcp34_317*RLcp34_118);
  ORcp34_318 = OMcp34_117*RLcp34_218-OMcp34_217*RLcp34_118;
  OPcp34_118 = OPcp34_117+qd[18]*(OMcp34_217*ROcp34_617-OMcp34_317*ROcp34_517)+qdd[18]*ROcp34_417;
  OPcp34_218 = OPcp34_217-qd[18]*(OMcp34_117*ROcp34_617-OMcp34_317*ROcp34_417)+qdd[18]*ROcp34_517;
  OPcp34_318 = OPcp34_317+qd[18]*(OMcp34_117*ROcp34_517-OMcp34_217*ROcp34_417)+qdd[18]*ROcp34_617;
  RLcp34_119 = ROcp34_718*s->dpt[3][32];
  RLcp34_219 = ROcp34_818*s->dpt[3][32];
  RLcp34_319 = ROcp34_918*s->dpt[3][32];
  OMcp34_119 = OMcp34_118+qd[19]*ROcp34_118;
  OMcp34_219 = OMcp34_218+qd[19]*ROcp34_218;
  OMcp34_319 = OMcp34_318+qd[19]*ROcp34_318;
  ORcp34_119 = OMcp34_218*RLcp34_319-OMcp34_318*RLcp34_219;
  ORcp34_219 = -(OMcp34_118*RLcp34_319-OMcp34_318*RLcp34_119);
  ORcp34_319 = OMcp34_118*RLcp34_219-OMcp34_218*RLcp34_119;
  OMcp34_120 = OMcp34_119+qd[20]*ROcp34_419;
  OMcp34_220 = OMcp34_219+qd[20]*ROcp34_519;
  OMcp34_320 = OMcp34_319+qd[20]*ROcp34_619;
  OPcp34_120 = OPcp34_118+qd[19]*(OMcp34_218*ROcp34_318-OMcp34_318*ROcp34_218)+qd[20]*(OMcp34_219*ROcp34_619-OMcp34_319*
 ROcp34_519)+qdd[19]*ROcp34_118+qdd[20]*ROcp34_419;
  OPcp34_220 = OPcp34_218-qd[19]*(OMcp34_118*ROcp34_318-OMcp34_318*ROcp34_118)-qd[20]*(OMcp34_119*ROcp34_619-OMcp34_319*
 ROcp34_419)+qdd[19]*ROcp34_218+qdd[20]*ROcp34_519;
  OPcp34_320 = OPcp34_318+qd[19]*(OMcp34_118*ROcp34_218-OMcp34_218*ROcp34_118)+qd[20]*(OMcp34_119*ROcp34_519-OMcp34_219*
 ROcp34_419)+qdd[19]*ROcp34_318+qdd[20]*ROcp34_619;

// = = Block_0_0_1_9_0_7 = = 
 
// Sensor Kinematics 


  RLcp34_122 = Dz223*ROcp34_720+ROcp34_120*s->dpt[1][39];
  RLcp34_222 = Dz223*ROcp34_820+ROcp34_220*s->dpt[1][39];
  RLcp34_322 = Dz223*ROcp34_920+ROcp34_320*s->dpt[1][39];
  ORcp34_122 = OMcp34_220*RLcp34_322-OMcp34_320*RLcp34_222;
  ORcp34_222 = -(OMcp34_120*RLcp34_322-OMcp34_320*RLcp34_122);
  ORcp34_322 = OMcp34_120*RLcp34_222-OMcp34_220*RLcp34_122;
  PxF9[1] = q[1]+RLcp34_115+RLcp34_116+RLcp34_117+RLcp34_118+RLcp34_119+RLcp34_122;
  PxF9[2] = q[2]+RLcp34_215+RLcp34_216+RLcp34_217+RLcp34_218+RLcp34_219+RLcp34_222;
  PxF9[3] = q[3]+RLcp34_315+RLcp34_316+RLcp34_317+RLcp34_318+RLcp34_319+RLcp34_322;
  RxF9[1][1] = ROcp34_120;
  RxF9[1][2] = ROcp34_220;
  RxF9[1][3] = ROcp34_320;
  RxF9[2][1] = ROcp34_419;
  RxF9[2][2] = ROcp34_519;
  RxF9[2][3] = ROcp34_619;
  RxF9[3][1] = ROcp34_720;
  RxF9[3][2] = ROcp34_820;
  RxF9[3][3] = ROcp34_920;
  VxF9[1] = qd[1]+ORcp34_115+ORcp34_116+ORcp34_117+ORcp34_118+ORcp34_119+ORcp34_122+qd[22]*ROcp34_720;
  VxF9[2] = qd[2]+ORcp34_215+ORcp34_216+ORcp34_217+ORcp34_218+ORcp34_219+ORcp34_222+qd[22]*ROcp34_820;
  VxF9[3] = qd[3]+ORcp34_315+ORcp34_316+ORcp34_317+ORcp34_318+ORcp34_319+ORcp34_322+qd[22]*ROcp34_920;
  OMxF9[1] = OMcp34_120;
  OMxF9[2] = OMcp34_220;
  OMxF9[3] = OMcp34_320;
  AxF9[1] = qdd[1]+(2.0)*qd[22]*(OMcp34_220*ROcp34_920-OMcp34_320*ROcp34_820)+qdd[22]*ROcp34_720+OMcp34_215*ORcp34_316+
 OMcp34_216*ORcp34_317+OMcp34_217*ORcp34_318+OMcp34_218*ORcp34_319+OMcp34_220*ORcp34_322+OMcp34_26*ORcp34_315-OMcp34_315*
 ORcp34_216-OMcp34_316*ORcp34_217-OMcp34_317*ORcp34_218-OMcp34_318*ORcp34_219-OMcp34_320*ORcp34_222-OMcp34_36*ORcp34_215+
 OPcp34_215*RLcp34_316+OPcp34_216*RLcp34_317+OPcp34_217*RLcp34_318+OPcp34_218*RLcp34_319+OPcp34_220*RLcp34_322+OPcp34_26*
 RLcp34_315-OPcp34_315*RLcp34_216-OPcp34_316*RLcp34_217-OPcp34_317*RLcp34_218-OPcp34_318*RLcp34_219-OPcp34_320*RLcp34_222-
 OPcp34_36*RLcp34_215;
  AxF9[2] = qdd[2]-(2.0)*qd[22]*(OMcp34_120*ROcp34_920-OMcp34_320*ROcp34_720)+qdd[22]*ROcp34_820-OMcp34_115*ORcp34_316-
 OMcp34_116*ORcp34_317-OMcp34_117*ORcp34_318-OMcp34_118*ORcp34_319-OMcp34_120*ORcp34_322-OMcp34_16*ORcp34_315+OMcp34_315*
 ORcp34_116+OMcp34_316*ORcp34_117+OMcp34_317*ORcp34_118+OMcp34_318*ORcp34_119+OMcp34_320*ORcp34_122+OMcp34_36*ORcp34_115-
 OPcp34_115*RLcp34_316-OPcp34_116*RLcp34_317-OPcp34_117*RLcp34_318-OPcp34_118*RLcp34_319-OPcp34_120*RLcp34_322-OPcp34_16*
 RLcp34_315+OPcp34_315*RLcp34_116+OPcp34_316*RLcp34_117+OPcp34_317*RLcp34_118+OPcp34_318*RLcp34_119+OPcp34_320*RLcp34_122+
 OPcp34_36*RLcp34_115;
  AxF9[3] = qdd[3]+(2.0)*qd[22]*(OMcp34_120*ROcp34_820-OMcp34_220*ROcp34_720)+qdd[22]*ROcp34_920+OMcp34_115*ORcp34_216+
 OMcp34_116*ORcp34_217+OMcp34_117*ORcp34_218+OMcp34_118*ORcp34_219+OMcp34_120*ORcp34_222+OMcp34_16*ORcp34_215-OMcp34_215*
 ORcp34_116-OMcp34_216*ORcp34_117-OMcp34_217*ORcp34_118-OMcp34_218*ORcp34_119-OMcp34_220*ORcp34_122-OMcp34_26*ORcp34_115+
 OPcp34_115*RLcp34_216+OPcp34_116*RLcp34_217+OPcp34_117*RLcp34_218+OPcp34_118*RLcp34_219+OPcp34_120*RLcp34_222+OPcp34_16*
 RLcp34_215-OPcp34_215*RLcp34_116-OPcp34_216*RLcp34_117-OPcp34_217*RLcp34_118-OPcp34_218*RLcp34_119-OPcp34_220*RLcp34_122-
 OPcp34_26*RLcp34_115;
  OMPxF9[1] = OPcp34_120;
  OMPxF9[2] = OPcp34_220;
  OMPxF9[3] = OPcp34_320;
 
// Sensor Forces Computation 

  SWr9 = user_ExtForces(PxF9,RxF9,VxF9,OMxF9,AxF9,OMPxF9,s,tsim,9);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc135 = ROcp34_120*SWr9[1]+ROcp34_220*SWr9[2]+ROcp34_320*SWr9[3];
  xfrc235 = ROcp34_419*SWr9[1]+ROcp34_519*SWr9[2]+ROcp34_619*SWr9[3];
  xfrc335 = ROcp34_720*SWr9[1]+ROcp34_820*SWr9[2]+ROcp34_920*SWr9[3];
  frc[1][22] = s->frc[1][22]+xfrc135;
  frc[2][22] = s->frc[2][22]+xfrc235;
  frc[3][22] = s->frc[3][22]+xfrc335;
  xtrq135 = ROcp34_120*SWr9[4]+ROcp34_220*SWr9[5]+ROcp34_320*SWr9[6];
  xtrq235 = ROcp34_419*SWr9[4]+ROcp34_519*SWr9[5]+ROcp34_619*SWr9[6];
  xtrq335 = ROcp34_720*SWr9[4]+ROcp34_820*SWr9[5]+ROcp34_920*SWr9[6];
  trq[1][22] = s->trq[1][22]+xtrq135-xfrc235*SWr9[9]+xfrc335*SWr9[8];
  trq[2][22] = s->trq[2][22]+xtrq235+xfrc135*SWr9[9]-xfrc335*SWr9[7];
  trq[3][22] = s->trq[3][22]+xtrq335-xfrc135*SWr9[8]+xfrc235*SWr9[7];

// = = Block_0_0_1_10_0_1 = = 
 
// Sensor Kinematics 


  ROcp35_25 = S4*S5;
  ROcp35_35 = -C4*S5;
  ROcp35_85 = -S4*C5;
  ROcp35_95 = C4*C5;
  ROcp35_16 = C5*C6;
  ROcp35_26 = ROcp35_25*C6+C4*S6;
  ROcp35_36 = ROcp35_35*C6+S4*S6;
  ROcp35_46 = -C5*S6;
  ROcp35_56 = -(ROcp35_25*S6-C4*C6);
  ROcp35_66 = -(ROcp35_35*S6-S4*C6);
  OMcp35_25 = qd[5]*C4;
  OMcp35_35 = qd[5]*S4;
  OMcp35_16 = qd[4]+qd[6]*S5;
  OMcp35_26 = OMcp35_25+qd[6]*ROcp35_85;
  OMcp35_36 = OMcp35_35+qd[6]*ROcp35_95;
  OPcp35_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp35_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp35_95-OMcp35_35*S5)-qdd[5]*C4-qdd[6]*ROcp35_85);
  OPcp35_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp35_85-OMcp35_25*S5)+qdd[5]*S4+qdd[6]*ROcp35_95;

// = = Block_0_0_1_10_0_8 = = 
 
// Sensor Kinematics 


  ROcp35_423 = ROcp35_46*C23+S23*S5;
  ROcp35_523 = ROcp35_56*C23+ROcp35_85*S23;
  ROcp35_623 = ROcp35_66*C23+ROcp35_95*S23;
  ROcp35_723 = -(ROcp35_46*S23-C23*S5);
  ROcp35_823 = -(ROcp35_56*S23-ROcp35_85*C23);
  ROcp35_923 = -(ROcp35_66*S23-ROcp35_95*C23);
  ROcp35_124 = ROcp35_16*C24-ROcp35_723*S24;
  ROcp35_224 = ROcp35_26*C24-ROcp35_823*S24;
  ROcp35_324 = ROcp35_36*C24-ROcp35_923*S24;
  ROcp35_724 = ROcp35_16*S24+ROcp35_723*C24;
  ROcp35_824 = ROcp35_26*S24+ROcp35_823*C24;
  ROcp35_924 = ROcp35_36*S24+ROcp35_923*C24;
  ROcp35_125 = ROcp35_124*C25+ROcp35_423*S25;
  ROcp35_225 = ROcp35_224*C25+ROcp35_523*S25;
  ROcp35_325 = ROcp35_324*C25+ROcp35_623*S25;
  ROcp35_425 = -(ROcp35_124*S25-ROcp35_423*C25);
  ROcp35_525 = -(ROcp35_224*S25-ROcp35_523*C25);
  ROcp35_625 = -(ROcp35_324*S25-ROcp35_623*C25);
  RLcp35_123 = ROcp35_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp35_223 = ROcp35_26*s->dpt[1][4]+ROcp35_85*s->dpt[3][4];
  RLcp35_323 = ROcp35_36*s->dpt[1][4]+ROcp35_95*s->dpt[3][4];
  OMcp35_123 = OMcp35_16+qd[23]*ROcp35_16;
  OMcp35_223 = OMcp35_26+qd[23]*ROcp35_26;
  OMcp35_323 = OMcp35_36+qd[23]*ROcp35_36;
  ORcp35_123 = OMcp35_26*RLcp35_323-OMcp35_36*RLcp35_223;
  ORcp35_223 = -(OMcp35_16*RLcp35_323-OMcp35_36*RLcp35_123);
  ORcp35_323 = OMcp35_16*RLcp35_223-OMcp35_26*RLcp35_123;
  OMcp35_124 = OMcp35_123+qd[24]*ROcp35_423;
  OMcp35_224 = OMcp35_223+qd[24]*ROcp35_523;
  OMcp35_324 = OMcp35_323+qd[24]*ROcp35_623;
  OPcp35_124 = OPcp35_16+qd[23]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26)+qd[24]*(OMcp35_223*ROcp35_623-OMcp35_323*
 ROcp35_523)+qdd[23]*ROcp35_16+qdd[24]*ROcp35_423;
  OPcp35_224 = OPcp35_26-qd[23]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16)-qd[24]*(OMcp35_123*ROcp35_623-OMcp35_323*
 ROcp35_423)+qdd[23]*ROcp35_26+qdd[24]*ROcp35_523;
  OPcp35_324 = OPcp35_36+qd[23]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16)+qd[24]*(OMcp35_123*ROcp35_523-OMcp35_223*
 ROcp35_423)+qdd[23]*ROcp35_36+qdd[24]*ROcp35_623;
  RLcp35_125 = ROcp35_724*s->dpt[3][44];
  RLcp35_225 = ROcp35_824*s->dpt[3][44];
  RLcp35_325 = ROcp35_924*s->dpt[3][44];
  ORcp35_125 = OMcp35_224*RLcp35_325-OMcp35_324*RLcp35_225;
  ORcp35_225 = -(OMcp35_124*RLcp35_325-OMcp35_324*RLcp35_125);
  ORcp35_325 = OMcp35_124*RLcp35_225-OMcp35_224*RLcp35_125;
  PxF10[1] = q[1]+RLcp35_123+RLcp35_125;
  PxF10[2] = q[2]+RLcp35_223+RLcp35_225;
  PxF10[3] = q[3]+RLcp35_323+RLcp35_325;
  RxF10[1][1] = ROcp35_125;
  RxF10[1][2] = ROcp35_225;
  RxF10[1][3] = ROcp35_325;
  RxF10[2][1] = ROcp35_425;
  RxF10[2][2] = ROcp35_525;
  RxF10[2][3] = ROcp35_625;
  RxF10[3][1] = ROcp35_724;
  RxF10[3][2] = ROcp35_824;
  RxF10[3][3] = ROcp35_924;
  VxF10[1] = qd[1]+ORcp35_123+ORcp35_125;
  VxF10[2] = qd[2]+ORcp35_223+ORcp35_225;
  VxF10[3] = qd[3]+ORcp35_323+ORcp35_325;
  OMxF10[1] = OMcp35_124+qd[25]*ROcp35_724;
  OMxF10[2] = OMcp35_224+qd[25]*ROcp35_824;
  OMxF10[3] = OMcp35_324+qd[25]*ROcp35_924;
  AxF10[1] = qdd[1]+OMcp35_224*ORcp35_325+OMcp35_26*ORcp35_323-OMcp35_324*ORcp35_225-OMcp35_36*ORcp35_223+OPcp35_224*
 RLcp35_325+OPcp35_26*RLcp35_323-OPcp35_324*RLcp35_225-OPcp35_36*RLcp35_223;
  AxF10[2] = qdd[2]-OMcp35_124*ORcp35_325-OMcp35_16*ORcp35_323+OMcp35_324*ORcp35_125+OMcp35_36*ORcp35_123-OPcp35_124*
 RLcp35_325-OPcp35_16*RLcp35_323+OPcp35_324*RLcp35_125+OPcp35_36*RLcp35_123;
  AxF10[3] = qdd[3]+OMcp35_124*ORcp35_225+OMcp35_16*ORcp35_223-OMcp35_224*ORcp35_125-OMcp35_26*ORcp35_123+OPcp35_124*
 RLcp35_225+OPcp35_16*RLcp35_223-OPcp35_224*RLcp35_125-OPcp35_26*RLcp35_123;
  OMPxF10[1] = OPcp35_124+qd[25]*(OMcp35_224*ROcp35_924-OMcp35_324*ROcp35_824)+qdd[25]*ROcp35_724;
  OMPxF10[2] = OPcp35_224-qd[25]*(OMcp35_124*ROcp35_924-OMcp35_324*ROcp35_724)+qdd[25]*ROcp35_824;
  OMPxF10[3] = OPcp35_324+qd[25]*(OMcp35_124*ROcp35_824-OMcp35_224*ROcp35_724)+qdd[25]*ROcp35_924;
 
// Sensor Forces Computation 

  SWr10 = user_ExtForces(PxF10,RxF10,VxF10,OMxF10,AxF10,OMPxF10,s,tsim,10);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc136 = ROcp35_125*SWr10[1]+ROcp35_225*SWr10[2]+ROcp35_325*SWr10[3];
  xfrc236 = ROcp35_425*SWr10[1]+ROcp35_525*SWr10[2]+ROcp35_625*SWr10[3];
  xfrc336 = ROcp35_724*SWr10[1]+ROcp35_824*SWr10[2]+ROcp35_924*SWr10[3];
  frc[1][25] = s->frc[1][25]+xfrc136;
  frc[2][25] = s->frc[2][25]+xfrc236;
  frc[3][25] = s->frc[3][25]+xfrc336;
  xtrq136 = ROcp35_125*SWr10[4]+ROcp35_225*SWr10[5]+ROcp35_325*SWr10[6];
  xtrq236 = ROcp35_425*SWr10[4]+ROcp35_525*SWr10[5]+ROcp35_625*SWr10[6];
  xtrq336 = ROcp35_724*SWr10[4]+ROcp35_824*SWr10[5]+ROcp35_924*SWr10[6];
  trq[1][25] = s->trq[1][25]+xtrq136-xfrc236*(SWr10[9]-s->l[3][25])+xfrc336*(SWr10[8]-s->l[2][25]);
  trq[2][25] = s->trq[2][25]+xtrq236+xfrc136*(SWr10[9]-s->l[3][25])-xfrc336*(SWr10[7]-s->l[1][25]);
  trq[3][25] = s->trq[3][25]+xtrq336-xfrc136*(SWr10[8]-s->l[2][25])+xfrc236*(SWr10[7]-s->l[1][25]);

// = = Block_0_0_1_11_0_1 = = 
 
// Sensor Kinematics 


  ROcp36_25 = S4*S5;
  ROcp36_35 = -C4*S5;
  ROcp36_85 = -S4*C5;
  ROcp36_95 = C4*C5;
  ROcp36_16 = C5*C6;
  ROcp36_26 = ROcp36_25*C6+C4*S6;
  ROcp36_36 = ROcp36_35*C6+S4*S6;
  ROcp36_46 = -C5*S6;
  ROcp36_56 = -(ROcp36_25*S6-C4*C6);
  ROcp36_66 = -(ROcp36_35*S6-S4*C6);
  OMcp36_25 = qd[5]*C4;
  OMcp36_35 = qd[5]*S4;
  OMcp36_16 = qd[4]+qd[6]*S5;
  OMcp36_26 = OMcp36_25+qd[6]*ROcp36_85;
  OMcp36_36 = OMcp36_35+qd[6]*ROcp36_95;
  OPcp36_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp36_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp36_95-OMcp36_35*S5)-qdd[5]*C4-qdd[6]*ROcp36_85);
  OPcp36_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp36_85-OMcp36_25*S5)+qdd[5]*S4+qdd[6]*ROcp36_95;

// = = Block_0_0_1_11_0_8 = = 
 
// Sensor Kinematics 


  ROcp36_423 = ROcp36_46*C23+S23*S5;
  ROcp36_523 = ROcp36_56*C23+ROcp36_85*S23;
  ROcp36_623 = ROcp36_66*C23+ROcp36_95*S23;
  ROcp36_723 = -(ROcp36_46*S23-C23*S5);
  ROcp36_823 = -(ROcp36_56*S23-ROcp36_85*C23);
  ROcp36_923 = -(ROcp36_66*S23-ROcp36_95*C23);
  ROcp36_124 = ROcp36_16*C24-ROcp36_723*S24;
  ROcp36_224 = ROcp36_26*C24-ROcp36_823*S24;
  ROcp36_324 = ROcp36_36*C24-ROcp36_923*S24;
  ROcp36_724 = ROcp36_16*S24+ROcp36_723*C24;
  ROcp36_824 = ROcp36_26*S24+ROcp36_823*C24;
  ROcp36_924 = ROcp36_36*S24+ROcp36_923*C24;
  ROcp36_125 = ROcp36_124*C25+ROcp36_423*S25;
  ROcp36_225 = ROcp36_224*C25+ROcp36_523*S25;
  ROcp36_325 = ROcp36_324*C25+ROcp36_623*S25;
  ROcp36_425 = -(ROcp36_124*S25-ROcp36_423*C25);
  ROcp36_525 = -(ROcp36_224*S25-ROcp36_523*C25);
  ROcp36_625 = -(ROcp36_324*S25-ROcp36_623*C25);
  RLcp36_123 = ROcp36_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp36_223 = ROcp36_26*s->dpt[1][4]+ROcp36_85*s->dpt[3][4];
  RLcp36_323 = ROcp36_36*s->dpt[1][4]+ROcp36_95*s->dpt[3][4];
  OMcp36_123 = OMcp36_16+qd[23]*ROcp36_16;
  OMcp36_223 = OMcp36_26+qd[23]*ROcp36_26;
  OMcp36_323 = OMcp36_36+qd[23]*ROcp36_36;
  ORcp36_123 = OMcp36_26*RLcp36_323-OMcp36_36*RLcp36_223;
  ORcp36_223 = -(OMcp36_16*RLcp36_323-OMcp36_36*RLcp36_123);
  ORcp36_323 = OMcp36_16*RLcp36_223-OMcp36_26*RLcp36_123;
  OMcp36_124 = OMcp36_123+qd[24]*ROcp36_423;
  OMcp36_224 = OMcp36_223+qd[24]*ROcp36_523;
  OMcp36_324 = OMcp36_323+qd[24]*ROcp36_623;
  OPcp36_124 = OPcp36_16+qd[23]*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26)+qd[24]*(OMcp36_223*ROcp36_623-OMcp36_323*
 ROcp36_523)+qdd[23]*ROcp36_16+qdd[24]*ROcp36_423;
  OPcp36_224 = OPcp36_26-qd[23]*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16)-qd[24]*(OMcp36_123*ROcp36_623-OMcp36_323*
 ROcp36_423)+qdd[23]*ROcp36_26+qdd[24]*ROcp36_523;
  OPcp36_324 = OPcp36_36+qd[23]*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16)+qd[24]*(OMcp36_123*ROcp36_523-OMcp36_223*
 ROcp36_423)+qdd[23]*ROcp36_36+qdd[24]*ROcp36_623;
  RLcp36_125 = ROcp36_724*s->dpt[3][44];
  RLcp36_225 = ROcp36_824*s->dpt[3][44];
  RLcp36_325 = ROcp36_924*s->dpt[3][44];
  OMcp36_125 = OMcp36_124+qd[25]*ROcp36_724;
  OMcp36_225 = OMcp36_224+qd[25]*ROcp36_824;
  OMcp36_325 = OMcp36_324+qd[25]*ROcp36_924;
  ORcp36_125 = OMcp36_224*RLcp36_325-OMcp36_324*RLcp36_225;
  ORcp36_225 = -(OMcp36_124*RLcp36_325-OMcp36_324*RLcp36_125);
  ORcp36_325 = OMcp36_124*RLcp36_225-OMcp36_224*RLcp36_125;
  OPcp36_125 = OPcp36_124+qd[25]*(OMcp36_224*ROcp36_924-OMcp36_324*ROcp36_824)+qdd[25]*ROcp36_724;
  OPcp36_225 = OPcp36_224-qd[25]*(OMcp36_124*ROcp36_924-OMcp36_324*ROcp36_724)+qdd[25]*ROcp36_824;
  OPcp36_325 = OPcp36_324+qd[25]*(OMcp36_124*ROcp36_824-OMcp36_224*ROcp36_724)+qdd[25]*ROcp36_924;

// = = Block_0_0_1_11_0_9 = = 
 
// Sensor Kinematics 


  ROcp36_126 = ROcp36_125*C26-ROcp36_724*S26;
  ROcp36_226 = ROcp36_225*C26-ROcp36_824*S26;
  ROcp36_326 = ROcp36_325*C26-ROcp36_924*S26;
  ROcp36_726 = ROcp36_125*S26+ROcp36_724*C26;
  ROcp36_826 = ROcp36_225*S26+ROcp36_824*C26;
  ROcp36_926 = ROcp36_325*S26+ROcp36_924*C26;
  ROcp36_427 = ROcp36_425*C27+ROcp36_726*S27;
  ROcp36_527 = ROcp36_525*C27+ROcp36_826*S27;
  ROcp36_627 = ROcp36_625*C27+ROcp36_926*S27;
  ROcp36_727 = -(ROcp36_425*S27-ROcp36_726*C27);
  ROcp36_827 = -(ROcp36_525*S27-ROcp36_826*C27);
  ROcp36_927 = -(ROcp36_625*S27-ROcp36_926*C27);
  ROcp36_128 = ROcp36_126*C28+ROcp36_427*S28;
  ROcp36_228 = ROcp36_226*C28+ROcp36_527*S28;
  ROcp36_328 = ROcp36_326*C28+ROcp36_627*S28;
  ROcp36_428 = -(ROcp36_126*S28-ROcp36_427*C28);
  ROcp36_528 = -(ROcp36_226*S28-ROcp36_527*C28);
  ROcp36_628 = -(ROcp36_326*S28-ROcp36_627*C28);
  RLcp36_126 = ROcp36_125*s->dpt[1][48]+ROcp36_425*s->dpt[2][48]+ROcp36_724*s->dpt[3][48];
  RLcp36_226 = ROcp36_225*s->dpt[1][48]+ROcp36_525*s->dpt[2][48]+ROcp36_824*s->dpt[3][48];
  RLcp36_326 = ROcp36_325*s->dpt[1][48]+ROcp36_625*s->dpt[2][48]+ROcp36_924*s->dpt[3][48];
  OMcp36_126 = OMcp36_125+qd[26]*ROcp36_425;
  OMcp36_226 = OMcp36_225+qd[26]*ROcp36_525;
  OMcp36_326 = OMcp36_325+qd[26]*ROcp36_625;
  ORcp36_126 = OMcp36_225*RLcp36_326-OMcp36_325*RLcp36_226;
  ORcp36_226 = -(OMcp36_125*RLcp36_326-OMcp36_325*RLcp36_126);
  ORcp36_326 = OMcp36_125*RLcp36_226-OMcp36_225*RLcp36_126;
  OPcp36_126 = OPcp36_125+qd[26]*(OMcp36_225*ROcp36_625-OMcp36_325*ROcp36_525)+qdd[26]*ROcp36_425;
  OPcp36_226 = OPcp36_225-qd[26]*(OMcp36_125*ROcp36_625-OMcp36_325*ROcp36_425)+qdd[26]*ROcp36_525;
  OPcp36_326 = OPcp36_325+qd[26]*(OMcp36_125*ROcp36_525-OMcp36_225*ROcp36_425)+qdd[26]*ROcp36_625;
  RLcp36_127 = ROcp36_425*s->dpt[2][51];
  RLcp36_227 = ROcp36_525*s->dpt[2][51];
  RLcp36_327 = ROcp36_625*s->dpt[2][51];
  OMcp36_127 = OMcp36_126+qd[27]*ROcp36_126;
  OMcp36_227 = OMcp36_226+qd[27]*ROcp36_226;
  OMcp36_327 = OMcp36_326+qd[27]*ROcp36_326;
  ORcp36_127 = OMcp36_226*RLcp36_327-OMcp36_326*RLcp36_227;
  ORcp36_227 = -(OMcp36_126*RLcp36_327-OMcp36_326*RLcp36_127);
  ORcp36_327 = OMcp36_126*RLcp36_227-OMcp36_226*RLcp36_127;
  OPcp36_127 = OPcp36_126+qd[27]*(OMcp36_226*ROcp36_326-OMcp36_326*ROcp36_226)+qdd[27]*ROcp36_126;
  OPcp36_227 = OPcp36_226-qd[27]*(OMcp36_126*ROcp36_326-OMcp36_326*ROcp36_126)+qdd[27]*ROcp36_226;
  OPcp36_327 = OPcp36_326+qd[27]*(OMcp36_126*ROcp36_226-OMcp36_226*ROcp36_126)+qdd[27]*ROcp36_326;
  RLcp36_128 = ROcp36_727*s->dpt[3][53];
  RLcp36_228 = ROcp36_827*s->dpt[3][53];
  RLcp36_328 = ROcp36_927*s->dpt[3][53];
  ORcp36_128 = OMcp36_227*RLcp36_328-OMcp36_327*RLcp36_228;
  ORcp36_228 = -(OMcp36_127*RLcp36_328-OMcp36_327*RLcp36_128);
  ORcp36_328 = OMcp36_127*RLcp36_228-OMcp36_227*RLcp36_128;
  PxF11[1] = q[1]+RLcp36_123+RLcp36_125+RLcp36_126+RLcp36_127+RLcp36_128;
  PxF11[2] = q[2]+RLcp36_223+RLcp36_225+RLcp36_226+RLcp36_227+RLcp36_228;
  PxF11[3] = q[3]+RLcp36_323+RLcp36_325+RLcp36_326+RLcp36_327+RLcp36_328;
  RxF11[1][1] = ROcp36_128;
  RxF11[1][2] = ROcp36_228;
  RxF11[1][3] = ROcp36_328;
  RxF11[2][1] = ROcp36_428;
  RxF11[2][2] = ROcp36_528;
  RxF11[2][3] = ROcp36_628;
  RxF11[3][1] = ROcp36_727;
  RxF11[3][2] = ROcp36_827;
  RxF11[3][3] = ROcp36_927;
  VxF11[1] = qd[1]+ORcp36_123+ORcp36_125+ORcp36_126+ORcp36_127+ORcp36_128;
  VxF11[2] = qd[2]+ORcp36_223+ORcp36_225+ORcp36_226+ORcp36_227+ORcp36_228;
  VxF11[3] = qd[3]+ORcp36_323+ORcp36_325+ORcp36_326+ORcp36_327+ORcp36_328;
  OMxF11[1] = OMcp36_127+qd[28]*ROcp36_727;
  OMxF11[2] = OMcp36_227+qd[28]*ROcp36_827;
  OMxF11[3] = OMcp36_327+qd[28]*ROcp36_927;
  AxF11[1] = qdd[1]+OMcp36_224*ORcp36_325+OMcp36_225*ORcp36_326+OMcp36_226*ORcp36_327+OMcp36_227*ORcp36_328+OMcp36_26*
 ORcp36_323-OMcp36_324*ORcp36_225-OMcp36_325*ORcp36_226-OMcp36_326*ORcp36_227-OMcp36_327*ORcp36_228-OMcp36_36*ORcp36_223+
 OPcp36_224*RLcp36_325+OPcp36_225*RLcp36_326+OPcp36_226*RLcp36_327+OPcp36_227*RLcp36_328+OPcp36_26*RLcp36_323-OPcp36_324*
 RLcp36_225-OPcp36_325*RLcp36_226-OPcp36_326*RLcp36_227-OPcp36_327*RLcp36_228-OPcp36_36*RLcp36_223;
  AxF11[2] = qdd[2]-OMcp36_124*ORcp36_325-OMcp36_125*ORcp36_326-OMcp36_126*ORcp36_327-OMcp36_127*ORcp36_328-OMcp36_16*
 ORcp36_323+OMcp36_324*ORcp36_125+OMcp36_325*ORcp36_126+OMcp36_326*ORcp36_127+OMcp36_327*ORcp36_128+OMcp36_36*ORcp36_123-
 OPcp36_124*RLcp36_325-OPcp36_125*RLcp36_326-OPcp36_126*RLcp36_327-OPcp36_127*RLcp36_328-OPcp36_16*RLcp36_323+OPcp36_324*
 RLcp36_125+OPcp36_325*RLcp36_126+OPcp36_326*RLcp36_127+OPcp36_327*RLcp36_128+OPcp36_36*RLcp36_123;
  AxF11[3] = qdd[3]+OMcp36_124*ORcp36_225+OMcp36_125*ORcp36_226+OMcp36_126*ORcp36_227+OMcp36_127*ORcp36_228+OMcp36_16*
 ORcp36_223-OMcp36_224*ORcp36_125-OMcp36_225*ORcp36_126-OMcp36_226*ORcp36_127-OMcp36_227*ORcp36_128-OMcp36_26*ORcp36_123+
 OPcp36_124*RLcp36_225+OPcp36_125*RLcp36_226+OPcp36_126*RLcp36_227+OPcp36_127*RLcp36_228+OPcp36_16*RLcp36_223-OPcp36_224*
 RLcp36_125-OPcp36_225*RLcp36_126-OPcp36_226*RLcp36_127-OPcp36_227*RLcp36_128-OPcp36_26*RLcp36_123;
  OMPxF11[1] = OPcp36_127+qd[28]*(OMcp36_227*ROcp36_927-OMcp36_327*ROcp36_827)+qdd[28]*ROcp36_727;
  OMPxF11[2] = OPcp36_227-qd[28]*(OMcp36_127*ROcp36_927-OMcp36_327*ROcp36_727)+qdd[28]*ROcp36_827;
  OMPxF11[3] = OPcp36_327+qd[28]*(OMcp36_127*ROcp36_827-OMcp36_227*ROcp36_727)+qdd[28]*ROcp36_927;
 
// Sensor Forces Computation 

  SWr11 = user_ExtForces(PxF11,RxF11,VxF11,OMxF11,AxF11,OMPxF11,s,tsim,11);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc137 = ROcp36_128*SWr11[1]+ROcp36_228*SWr11[2]+ROcp36_328*SWr11[3];
  xfrc237 = ROcp36_428*SWr11[1]+ROcp36_528*SWr11[2]+ROcp36_628*SWr11[3];
  xfrc337 = ROcp36_727*SWr11[1]+ROcp36_827*SWr11[2]+ROcp36_927*SWr11[3];
  frc[1][28] = s->frc[1][28]+xfrc137;
  frc[2][28] = s->frc[2][28]+xfrc237;
  frc[3][28] = s->frc[3][28]+xfrc337;
  xtrq137 = ROcp36_128*SWr11[4]+ROcp36_228*SWr11[5]+ROcp36_328*SWr11[6];
  xtrq237 = ROcp36_428*SWr11[4]+ROcp36_528*SWr11[5]+ROcp36_628*SWr11[6];
  xtrq337 = ROcp36_727*SWr11[4]+ROcp36_827*SWr11[5]+ROcp36_927*SWr11[6];
  trq[1][28] = s->trq[1][28]+xtrq137-xfrc237*(SWr11[9]-s->l[3][28])+xfrc337*(SWr11[8]-s->l[2][28]);
  trq[2][28] = s->trq[2][28]+xtrq237+xfrc137*(SWr11[9]-s->l[3][28])-xfrc337*(SWr11[7]-s->l[1][28]);
  trq[3][28] = s->trq[3][28]+xtrq337-xfrc137*(SWr11[8]-s->l[2][28])+xfrc237*(SWr11[7]-s->l[1][28]);

// = = Block_0_0_1_12_0_1 = = 
 
// Sensor Kinematics 


  ROcp37_25 = S4*S5;
  ROcp37_35 = -C4*S5;
  ROcp37_85 = -S4*C5;
  ROcp37_95 = C4*C5;
  ROcp37_16 = C5*C6;
  ROcp37_26 = ROcp37_25*C6+C4*S6;
  ROcp37_36 = ROcp37_35*C6+S4*S6;
  ROcp37_46 = -C5*S6;
  ROcp37_56 = -(ROcp37_25*S6-C4*C6);
  ROcp37_66 = -(ROcp37_35*S6-S4*C6);
  OMcp37_25 = qd[5]*C4;
  OMcp37_35 = qd[5]*S4;
  OMcp37_16 = qd[4]+qd[6]*S5;
  OMcp37_26 = OMcp37_25+qd[6]*ROcp37_85;
  OMcp37_36 = OMcp37_35+qd[6]*ROcp37_95;
  OPcp37_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp37_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp37_95-OMcp37_35*S5)-qdd[5]*C4-qdd[6]*ROcp37_85);
  OPcp37_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp37_85-OMcp37_25*S5)+qdd[5]*S4+qdd[6]*ROcp37_95;

// = = Block_0_0_1_12_0_8 = = 
 
// Sensor Kinematics 


  ROcp37_423 = ROcp37_46*C23+S23*S5;
  ROcp37_523 = ROcp37_56*C23+ROcp37_85*S23;
  ROcp37_623 = ROcp37_66*C23+ROcp37_95*S23;
  ROcp37_723 = -(ROcp37_46*S23-C23*S5);
  ROcp37_823 = -(ROcp37_56*S23-ROcp37_85*C23);
  ROcp37_923 = -(ROcp37_66*S23-ROcp37_95*C23);
  ROcp37_124 = ROcp37_16*C24-ROcp37_723*S24;
  ROcp37_224 = ROcp37_26*C24-ROcp37_823*S24;
  ROcp37_324 = ROcp37_36*C24-ROcp37_923*S24;
  ROcp37_724 = ROcp37_16*S24+ROcp37_723*C24;
  ROcp37_824 = ROcp37_26*S24+ROcp37_823*C24;
  ROcp37_924 = ROcp37_36*S24+ROcp37_923*C24;
  ROcp37_125 = ROcp37_124*C25+ROcp37_423*S25;
  ROcp37_225 = ROcp37_224*C25+ROcp37_523*S25;
  ROcp37_325 = ROcp37_324*C25+ROcp37_623*S25;
  ROcp37_425 = -(ROcp37_124*S25-ROcp37_423*C25);
  ROcp37_525 = -(ROcp37_224*S25-ROcp37_523*C25);
  ROcp37_625 = -(ROcp37_324*S25-ROcp37_623*C25);
  RLcp37_123 = ROcp37_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp37_223 = ROcp37_26*s->dpt[1][4]+ROcp37_85*s->dpt[3][4];
  RLcp37_323 = ROcp37_36*s->dpt[1][4]+ROcp37_95*s->dpt[3][4];
  OMcp37_123 = OMcp37_16+qd[23]*ROcp37_16;
  OMcp37_223 = OMcp37_26+qd[23]*ROcp37_26;
  OMcp37_323 = OMcp37_36+qd[23]*ROcp37_36;
  ORcp37_123 = OMcp37_26*RLcp37_323-OMcp37_36*RLcp37_223;
  ORcp37_223 = -(OMcp37_16*RLcp37_323-OMcp37_36*RLcp37_123);
  ORcp37_323 = OMcp37_16*RLcp37_223-OMcp37_26*RLcp37_123;
  OMcp37_124 = OMcp37_123+qd[24]*ROcp37_423;
  OMcp37_224 = OMcp37_223+qd[24]*ROcp37_523;
  OMcp37_324 = OMcp37_323+qd[24]*ROcp37_623;
  OPcp37_124 = OPcp37_16+qd[23]*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26)+qd[24]*(OMcp37_223*ROcp37_623-OMcp37_323*
 ROcp37_523)+qdd[23]*ROcp37_16+qdd[24]*ROcp37_423;
  OPcp37_224 = OPcp37_26-qd[23]*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16)-qd[24]*(OMcp37_123*ROcp37_623-OMcp37_323*
 ROcp37_423)+qdd[23]*ROcp37_26+qdd[24]*ROcp37_523;
  OPcp37_324 = OPcp37_36+qd[23]*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16)+qd[24]*(OMcp37_123*ROcp37_523-OMcp37_223*
 ROcp37_423)+qdd[23]*ROcp37_36+qdd[24]*ROcp37_623;
  RLcp37_125 = ROcp37_724*s->dpt[3][44];
  RLcp37_225 = ROcp37_824*s->dpt[3][44];
  RLcp37_325 = ROcp37_924*s->dpt[3][44];
  OMcp37_125 = OMcp37_124+qd[25]*ROcp37_724;
  OMcp37_225 = OMcp37_224+qd[25]*ROcp37_824;
  OMcp37_325 = OMcp37_324+qd[25]*ROcp37_924;
  ORcp37_125 = OMcp37_224*RLcp37_325-OMcp37_324*RLcp37_225;
  ORcp37_225 = -(OMcp37_124*RLcp37_325-OMcp37_324*RLcp37_125);
  ORcp37_325 = OMcp37_124*RLcp37_225-OMcp37_224*RLcp37_125;
  OPcp37_125 = OPcp37_124+qd[25]*(OMcp37_224*ROcp37_924-OMcp37_324*ROcp37_824)+qdd[25]*ROcp37_724;
  OPcp37_225 = OPcp37_224-qd[25]*(OMcp37_124*ROcp37_924-OMcp37_324*ROcp37_724)+qdd[25]*ROcp37_824;
  OPcp37_325 = OPcp37_324+qd[25]*(OMcp37_124*ROcp37_824-OMcp37_224*ROcp37_724)+qdd[25]*ROcp37_924;

// = = Block_0_0_1_12_0_9 = = 
 
// Sensor Kinematics 


  ROcp37_126 = ROcp37_125*C26-ROcp37_724*S26;
  ROcp37_226 = ROcp37_225*C26-ROcp37_824*S26;
  ROcp37_326 = ROcp37_325*C26-ROcp37_924*S26;
  ROcp37_726 = ROcp37_125*S26+ROcp37_724*C26;
  ROcp37_826 = ROcp37_225*S26+ROcp37_824*C26;
  ROcp37_926 = ROcp37_325*S26+ROcp37_924*C26;
  ROcp37_427 = ROcp37_425*C27+ROcp37_726*S27;
  ROcp37_527 = ROcp37_525*C27+ROcp37_826*S27;
  ROcp37_627 = ROcp37_625*C27+ROcp37_926*S27;
  ROcp37_727 = -(ROcp37_425*S27-ROcp37_726*C27);
  ROcp37_827 = -(ROcp37_525*S27-ROcp37_826*C27);
  ROcp37_927 = -(ROcp37_625*S27-ROcp37_926*C27);
  ROcp37_128 = ROcp37_126*C28+ROcp37_427*S28;
  ROcp37_228 = ROcp37_226*C28+ROcp37_527*S28;
  ROcp37_328 = ROcp37_326*C28+ROcp37_627*S28;
  ROcp37_428 = -(ROcp37_126*S28-ROcp37_427*C28);
  ROcp37_528 = -(ROcp37_226*S28-ROcp37_527*C28);
  ROcp37_628 = -(ROcp37_326*S28-ROcp37_627*C28);
  ROcp37_129 = ROcp37_128*C29-ROcp37_727*S29;
  ROcp37_229 = ROcp37_228*C29-ROcp37_827*S29;
  ROcp37_329 = ROcp37_328*C29-ROcp37_927*S29;
  ROcp37_729 = ROcp37_128*S29+ROcp37_727*C29;
  ROcp37_829 = ROcp37_228*S29+ROcp37_827*C29;
  ROcp37_929 = ROcp37_328*S29+ROcp37_927*C29;
  RLcp37_126 = ROcp37_125*s->dpt[1][48]+ROcp37_425*s->dpt[2][48]+ROcp37_724*s->dpt[3][48];
  RLcp37_226 = ROcp37_225*s->dpt[1][48]+ROcp37_525*s->dpt[2][48]+ROcp37_824*s->dpt[3][48];
  RLcp37_326 = ROcp37_325*s->dpt[1][48]+ROcp37_625*s->dpt[2][48]+ROcp37_924*s->dpt[3][48];
  OMcp37_126 = OMcp37_125+qd[26]*ROcp37_425;
  OMcp37_226 = OMcp37_225+qd[26]*ROcp37_525;
  OMcp37_326 = OMcp37_325+qd[26]*ROcp37_625;
  ORcp37_126 = OMcp37_225*RLcp37_326-OMcp37_325*RLcp37_226;
  ORcp37_226 = -(OMcp37_125*RLcp37_326-OMcp37_325*RLcp37_126);
  ORcp37_326 = OMcp37_125*RLcp37_226-OMcp37_225*RLcp37_126;
  OPcp37_126 = OPcp37_125+qd[26]*(OMcp37_225*ROcp37_625-OMcp37_325*ROcp37_525)+qdd[26]*ROcp37_425;
  OPcp37_226 = OPcp37_225-qd[26]*(OMcp37_125*ROcp37_625-OMcp37_325*ROcp37_425)+qdd[26]*ROcp37_525;
  OPcp37_326 = OPcp37_325+qd[26]*(OMcp37_125*ROcp37_525-OMcp37_225*ROcp37_425)+qdd[26]*ROcp37_625;
  RLcp37_127 = ROcp37_425*s->dpt[2][51];
  RLcp37_227 = ROcp37_525*s->dpt[2][51];
  RLcp37_327 = ROcp37_625*s->dpt[2][51];
  OMcp37_127 = OMcp37_126+qd[27]*ROcp37_126;
  OMcp37_227 = OMcp37_226+qd[27]*ROcp37_226;
  OMcp37_327 = OMcp37_326+qd[27]*ROcp37_326;
  ORcp37_127 = OMcp37_226*RLcp37_327-OMcp37_326*RLcp37_227;
  ORcp37_227 = -(OMcp37_126*RLcp37_327-OMcp37_326*RLcp37_127);
  ORcp37_327 = OMcp37_126*RLcp37_227-OMcp37_226*RLcp37_127;
  OPcp37_127 = OPcp37_126+qd[27]*(OMcp37_226*ROcp37_326-OMcp37_326*ROcp37_226)+qdd[27]*ROcp37_126;
  OPcp37_227 = OPcp37_226-qd[27]*(OMcp37_126*ROcp37_326-OMcp37_326*ROcp37_126)+qdd[27]*ROcp37_226;
  OPcp37_327 = OPcp37_326+qd[27]*(OMcp37_126*ROcp37_226-OMcp37_226*ROcp37_126)+qdd[27]*ROcp37_326;
  RLcp37_128 = ROcp37_727*s->dpt[3][53];
  RLcp37_228 = ROcp37_827*s->dpt[3][53];
  RLcp37_328 = ROcp37_927*s->dpt[3][53];
  OMcp37_128 = OMcp37_127+qd[28]*ROcp37_727;
  OMcp37_228 = OMcp37_227+qd[28]*ROcp37_827;
  OMcp37_328 = OMcp37_327+qd[28]*ROcp37_927;
  ORcp37_128 = OMcp37_227*RLcp37_328-OMcp37_327*RLcp37_228;
  ORcp37_228 = -(OMcp37_127*RLcp37_328-OMcp37_327*RLcp37_128);
  ORcp37_328 = OMcp37_127*RLcp37_228-OMcp37_227*RLcp37_128;
  OPcp37_128 = OPcp37_127+qd[28]*(OMcp37_227*ROcp37_927-OMcp37_327*ROcp37_827)+qdd[28]*ROcp37_727;
  OPcp37_228 = OPcp37_227-qd[28]*(OMcp37_127*ROcp37_927-OMcp37_327*ROcp37_727)+qdd[28]*ROcp37_827;
  OPcp37_328 = OPcp37_327+qd[28]*(OMcp37_127*ROcp37_827-OMcp37_227*ROcp37_727)+qdd[28]*ROcp37_927;
  RLcp37_129 = ROcp37_727*s->dpt[3][56];
  RLcp37_229 = ROcp37_827*s->dpt[3][56];
  RLcp37_329 = ROcp37_927*s->dpt[3][56];
  ORcp37_129 = OMcp37_228*RLcp37_329-OMcp37_328*RLcp37_229;
  ORcp37_229 = -(OMcp37_128*RLcp37_329-OMcp37_328*RLcp37_129);
  ORcp37_329 = OMcp37_128*RLcp37_229-OMcp37_228*RLcp37_129;
  PxF12[1] = q[1]+RLcp37_123+RLcp37_125+RLcp37_126+RLcp37_127+RLcp37_128+RLcp37_129;
  PxF12[2] = q[2]+RLcp37_223+RLcp37_225+RLcp37_226+RLcp37_227+RLcp37_228+RLcp37_229;
  PxF12[3] = q[3]+RLcp37_323+RLcp37_325+RLcp37_326+RLcp37_327+RLcp37_328+RLcp37_329;
  RxF12[1][1] = ROcp37_129;
  RxF12[1][2] = ROcp37_229;
  RxF12[1][3] = ROcp37_329;
  RxF12[2][1] = ROcp37_428;
  RxF12[2][2] = ROcp37_528;
  RxF12[2][3] = ROcp37_628;
  RxF12[3][1] = ROcp37_729;
  RxF12[3][2] = ROcp37_829;
  RxF12[3][3] = ROcp37_929;
  VxF12[1] = qd[1]+ORcp37_123+ORcp37_125+ORcp37_126+ORcp37_127+ORcp37_128+ORcp37_129;
  VxF12[2] = qd[2]+ORcp37_223+ORcp37_225+ORcp37_226+ORcp37_227+ORcp37_228+ORcp37_229;
  VxF12[3] = qd[3]+ORcp37_323+ORcp37_325+ORcp37_326+ORcp37_327+ORcp37_328+ORcp37_329;
  OMxF12[1] = OMcp37_128+qd[29]*ROcp37_428;
  OMxF12[2] = OMcp37_228+qd[29]*ROcp37_528;
  OMxF12[3] = OMcp37_328+qd[29]*ROcp37_628;
  AxF12[1] = qdd[1]+OMcp37_224*ORcp37_325+OMcp37_225*ORcp37_326+OMcp37_226*ORcp37_327+OMcp37_227*ORcp37_328+OMcp37_228*
 ORcp37_329+OMcp37_26*ORcp37_323-OMcp37_324*ORcp37_225-OMcp37_325*ORcp37_226-OMcp37_326*ORcp37_227-OMcp37_327*ORcp37_228-
 OMcp37_328*ORcp37_229-OMcp37_36*ORcp37_223+OPcp37_224*RLcp37_325+OPcp37_225*RLcp37_326+OPcp37_226*RLcp37_327+OPcp37_227*
 RLcp37_328+OPcp37_228*RLcp37_329+OPcp37_26*RLcp37_323-OPcp37_324*RLcp37_225-OPcp37_325*RLcp37_226-OPcp37_326*RLcp37_227-
 OPcp37_327*RLcp37_228-OPcp37_328*RLcp37_229-OPcp37_36*RLcp37_223;
  AxF12[2] = qdd[2]-OMcp37_124*ORcp37_325-OMcp37_125*ORcp37_326-OMcp37_126*ORcp37_327-OMcp37_127*ORcp37_328-OMcp37_128*
 ORcp37_329-OMcp37_16*ORcp37_323+OMcp37_324*ORcp37_125+OMcp37_325*ORcp37_126+OMcp37_326*ORcp37_127+OMcp37_327*ORcp37_128+
 OMcp37_328*ORcp37_129+OMcp37_36*ORcp37_123-OPcp37_124*RLcp37_325-OPcp37_125*RLcp37_326-OPcp37_126*RLcp37_327-OPcp37_127*
 RLcp37_328-OPcp37_128*RLcp37_329-OPcp37_16*RLcp37_323+OPcp37_324*RLcp37_125+OPcp37_325*RLcp37_126+OPcp37_326*RLcp37_127+
 OPcp37_327*RLcp37_128+OPcp37_328*RLcp37_129+OPcp37_36*RLcp37_123;
  AxF12[3] = qdd[3]+OMcp37_124*ORcp37_225+OMcp37_125*ORcp37_226+OMcp37_126*ORcp37_227+OMcp37_127*ORcp37_228+OMcp37_128*
 ORcp37_229+OMcp37_16*ORcp37_223-OMcp37_224*ORcp37_125-OMcp37_225*ORcp37_126-OMcp37_226*ORcp37_127-OMcp37_227*ORcp37_128-
 OMcp37_228*ORcp37_129-OMcp37_26*ORcp37_123+OPcp37_124*RLcp37_225+OPcp37_125*RLcp37_226+OPcp37_126*RLcp37_227+OPcp37_127*
 RLcp37_228+OPcp37_128*RLcp37_229+OPcp37_16*RLcp37_223-OPcp37_224*RLcp37_125-OPcp37_225*RLcp37_126-OPcp37_226*RLcp37_127-
 OPcp37_227*RLcp37_128-OPcp37_228*RLcp37_129-OPcp37_26*RLcp37_123;
  OMPxF12[1] = OPcp37_128+qd[29]*(OMcp37_228*ROcp37_628-OMcp37_328*ROcp37_528)+qdd[29]*ROcp37_428;
  OMPxF12[2] = OPcp37_228-qd[29]*(OMcp37_128*ROcp37_628-OMcp37_328*ROcp37_428)+qdd[29]*ROcp37_528;
  OMPxF12[3] = OPcp37_328+qd[29]*(OMcp37_128*ROcp37_528-OMcp37_228*ROcp37_428)+qdd[29]*ROcp37_628;
 
// Sensor Forces Computation 

  SWr12 = user_ExtForces(PxF12,RxF12,VxF12,OMxF12,AxF12,OMPxF12,s,tsim,12);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc138 = ROcp37_129*SWr12[1]+ROcp37_229*SWr12[2]+ROcp37_329*SWr12[3];
  xfrc238 = ROcp37_428*SWr12[1]+ROcp37_528*SWr12[2]+ROcp37_628*SWr12[3];
  xfrc338 = ROcp37_729*SWr12[1]+ROcp37_829*SWr12[2]+ROcp37_929*SWr12[3];
  frc[1][29] = s->frc[1][29]+xfrc138;
  frc[2][29] = s->frc[2][29]+xfrc238;
  frc[3][29] = s->frc[3][29]+xfrc338;
  xtrq138 = ROcp37_129*SWr12[4]+ROcp37_229*SWr12[5]+ROcp37_329*SWr12[6];
  xtrq238 = ROcp37_428*SWr12[4]+ROcp37_528*SWr12[5]+ROcp37_628*SWr12[6];
  xtrq338 = ROcp37_729*SWr12[4]+ROcp37_829*SWr12[5]+ROcp37_929*SWr12[6];
  trq[1][29] = s->trq[1][29]+xtrq138-xfrc238*(SWr12[9]-s->l[3][29])+xfrc338*(SWr12[8]-s->l[2][29]);
  trq[2][29] = s->trq[2][29]+xtrq238+xfrc138*(SWr12[9]-s->l[3][29])-xfrc338*(SWr12[7]-s->l[1][29]);
  trq[3][29] = s->trq[3][29]+xtrq338-xfrc138*(SWr12[8]-s->l[2][29])+xfrc238*(SWr12[7]-s->l[1][29]);

// = = Block_0_0_1_13_0_1 = = 
 
// Sensor Kinematics 


  ROcp38_25 = S4*S5;
  ROcp38_35 = -C4*S5;
  ROcp38_85 = -S4*C5;
  ROcp38_95 = C4*C5;
  ROcp38_16 = C5*C6;
  ROcp38_26 = ROcp38_25*C6+C4*S6;
  ROcp38_36 = ROcp38_35*C6+S4*S6;
  ROcp38_46 = -C5*S6;
  ROcp38_56 = -(ROcp38_25*S6-C4*C6);
  ROcp38_66 = -(ROcp38_35*S6-S4*C6);
  OMcp38_25 = qd[5]*C4;
  OMcp38_35 = qd[5]*S4;
  OMcp38_16 = qd[4]+qd[6]*S5;
  OMcp38_26 = OMcp38_25+qd[6]*ROcp38_85;
  OMcp38_36 = OMcp38_35+qd[6]*ROcp38_95;
  OPcp38_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp38_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp38_95-OMcp38_35*S5)-qdd[5]*C4-qdd[6]*ROcp38_85);
  OPcp38_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp38_85-OMcp38_25*S5)+qdd[5]*S4+qdd[6]*ROcp38_95;

// = = Block_0_0_1_13_0_8 = = 
 
// Sensor Kinematics 


  ROcp38_423 = ROcp38_46*C23+S23*S5;
  ROcp38_523 = ROcp38_56*C23+ROcp38_85*S23;
  ROcp38_623 = ROcp38_66*C23+ROcp38_95*S23;
  ROcp38_723 = -(ROcp38_46*S23-C23*S5);
  ROcp38_823 = -(ROcp38_56*S23-ROcp38_85*C23);
  ROcp38_923 = -(ROcp38_66*S23-ROcp38_95*C23);
  ROcp38_124 = ROcp38_16*C24-ROcp38_723*S24;
  ROcp38_224 = ROcp38_26*C24-ROcp38_823*S24;
  ROcp38_324 = ROcp38_36*C24-ROcp38_923*S24;
  ROcp38_724 = ROcp38_16*S24+ROcp38_723*C24;
  ROcp38_824 = ROcp38_26*S24+ROcp38_823*C24;
  ROcp38_924 = ROcp38_36*S24+ROcp38_923*C24;
  ROcp38_125 = ROcp38_124*C25+ROcp38_423*S25;
  ROcp38_225 = ROcp38_224*C25+ROcp38_523*S25;
  ROcp38_325 = ROcp38_324*C25+ROcp38_623*S25;
  ROcp38_425 = -(ROcp38_124*S25-ROcp38_423*C25);
  ROcp38_525 = -(ROcp38_224*S25-ROcp38_523*C25);
  ROcp38_625 = -(ROcp38_324*S25-ROcp38_623*C25);
  RLcp38_123 = ROcp38_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp38_223 = ROcp38_26*s->dpt[1][4]+ROcp38_85*s->dpt[3][4];
  RLcp38_323 = ROcp38_36*s->dpt[1][4]+ROcp38_95*s->dpt[3][4];
  OMcp38_123 = OMcp38_16+qd[23]*ROcp38_16;
  OMcp38_223 = OMcp38_26+qd[23]*ROcp38_26;
  OMcp38_323 = OMcp38_36+qd[23]*ROcp38_36;
  ORcp38_123 = OMcp38_26*RLcp38_323-OMcp38_36*RLcp38_223;
  ORcp38_223 = -(OMcp38_16*RLcp38_323-OMcp38_36*RLcp38_123);
  ORcp38_323 = OMcp38_16*RLcp38_223-OMcp38_26*RLcp38_123;
  OMcp38_124 = OMcp38_123+qd[24]*ROcp38_423;
  OMcp38_224 = OMcp38_223+qd[24]*ROcp38_523;
  OMcp38_324 = OMcp38_323+qd[24]*ROcp38_623;
  OPcp38_124 = OPcp38_16+qd[23]*(OMcp38_26*ROcp38_36-OMcp38_36*ROcp38_26)+qd[24]*(OMcp38_223*ROcp38_623-OMcp38_323*
 ROcp38_523)+qdd[23]*ROcp38_16+qdd[24]*ROcp38_423;
  OPcp38_224 = OPcp38_26-qd[23]*(OMcp38_16*ROcp38_36-OMcp38_36*ROcp38_16)-qd[24]*(OMcp38_123*ROcp38_623-OMcp38_323*
 ROcp38_423)+qdd[23]*ROcp38_26+qdd[24]*ROcp38_523;
  OPcp38_324 = OPcp38_36+qd[23]*(OMcp38_16*ROcp38_26-OMcp38_26*ROcp38_16)+qd[24]*(OMcp38_123*ROcp38_523-OMcp38_223*
 ROcp38_423)+qdd[23]*ROcp38_36+qdd[24]*ROcp38_623;
  RLcp38_125 = ROcp38_724*s->dpt[3][44];
  RLcp38_225 = ROcp38_824*s->dpt[3][44];
  RLcp38_325 = ROcp38_924*s->dpt[3][44];
  OMcp38_125 = OMcp38_124+qd[25]*ROcp38_724;
  OMcp38_225 = OMcp38_224+qd[25]*ROcp38_824;
  OMcp38_325 = OMcp38_324+qd[25]*ROcp38_924;
  ORcp38_125 = OMcp38_224*RLcp38_325-OMcp38_324*RLcp38_225;
  ORcp38_225 = -(OMcp38_124*RLcp38_325-OMcp38_324*RLcp38_125);
  ORcp38_325 = OMcp38_124*RLcp38_225-OMcp38_224*RLcp38_125;
  OPcp38_125 = OPcp38_124+qd[25]*(OMcp38_224*ROcp38_924-OMcp38_324*ROcp38_824)+qdd[25]*ROcp38_724;
  OPcp38_225 = OPcp38_224-qd[25]*(OMcp38_124*ROcp38_924-OMcp38_324*ROcp38_724)+qdd[25]*ROcp38_824;
  OPcp38_325 = OPcp38_324+qd[25]*(OMcp38_124*ROcp38_824-OMcp38_224*ROcp38_724)+qdd[25]*ROcp38_924;

// = = Block_0_0_1_13_0_10 = = 
 
// Sensor Kinematics 


  ROcp38_130 = ROcp38_125*C30-ROcp38_724*S30;
  ROcp38_230 = ROcp38_225*C30-ROcp38_824*S30;
  ROcp38_330 = ROcp38_325*C30-ROcp38_924*S30;
  ROcp38_730 = ROcp38_125*S30+ROcp38_724*C30;
  ROcp38_830 = ROcp38_225*S30+ROcp38_824*C30;
  ROcp38_930 = ROcp38_325*S30+ROcp38_924*C30;
  ROcp38_431 = ROcp38_425*C31+ROcp38_730*S31;
  ROcp38_531 = ROcp38_525*C31+ROcp38_830*S31;
  ROcp38_631 = ROcp38_625*C31+ROcp38_930*S31;
  ROcp38_731 = -(ROcp38_425*S31-ROcp38_730*C31);
  ROcp38_831 = -(ROcp38_525*S31-ROcp38_830*C31);
  ROcp38_931 = -(ROcp38_625*S31-ROcp38_930*C31);
  ROcp38_132 = ROcp38_130*C32+ROcp38_431*S32;
  ROcp38_232 = ROcp38_230*C32+ROcp38_531*S32;
  ROcp38_332 = ROcp38_330*C32+ROcp38_631*S32;
  ROcp38_432 = -(ROcp38_130*S32-ROcp38_431*C32);
  ROcp38_532 = -(ROcp38_230*S32-ROcp38_531*C32);
  ROcp38_632 = -(ROcp38_330*S32-ROcp38_631*C32);
  RLcp38_130 = ROcp38_125*s->dpt[1][49]+ROcp38_425*s->dpt[2][49]+ROcp38_724*s->dpt[3][49];
  RLcp38_230 = ROcp38_225*s->dpt[1][49]+ROcp38_525*s->dpt[2][49]+ROcp38_824*s->dpt[3][49];
  RLcp38_330 = ROcp38_325*s->dpt[1][49]+ROcp38_625*s->dpt[2][49]+ROcp38_924*s->dpt[3][49];
  OMcp38_130 = OMcp38_125+qd[30]*ROcp38_425;
  OMcp38_230 = OMcp38_225+qd[30]*ROcp38_525;
  OMcp38_330 = OMcp38_325+qd[30]*ROcp38_625;
  ORcp38_130 = OMcp38_225*RLcp38_330-OMcp38_325*RLcp38_230;
  ORcp38_230 = -(OMcp38_125*RLcp38_330-OMcp38_325*RLcp38_130);
  ORcp38_330 = OMcp38_125*RLcp38_230-OMcp38_225*RLcp38_130;
  OPcp38_130 = OPcp38_125+qd[30]*(OMcp38_225*ROcp38_625-OMcp38_325*ROcp38_525)+qdd[30]*ROcp38_425;
  OPcp38_230 = OPcp38_225-qd[30]*(OMcp38_125*ROcp38_625-OMcp38_325*ROcp38_425)+qdd[30]*ROcp38_525;
  OPcp38_330 = OPcp38_325+qd[30]*(OMcp38_125*ROcp38_525-OMcp38_225*ROcp38_425)+qdd[30]*ROcp38_625;
  RLcp38_131 = ROcp38_425*s->dpt[2][60];
  RLcp38_231 = ROcp38_525*s->dpt[2][60];
  RLcp38_331 = ROcp38_625*s->dpt[2][60];
  OMcp38_131 = OMcp38_130+qd[31]*ROcp38_130;
  OMcp38_231 = OMcp38_230+qd[31]*ROcp38_230;
  OMcp38_331 = OMcp38_330+qd[31]*ROcp38_330;
  ORcp38_131 = OMcp38_230*RLcp38_331-OMcp38_330*RLcp38_231;
  ORcp38_231 = -(OMcp38_130*RLcp38_331-OMcp38_330*RLcp38_131);
  ORcp38_331 = OMcp38_130*RLcp38_231-OMcp38_230*RLcp38_131;
  OPcp38_131 = OPcp38_130+qd[31]*(OMcp38_230*ROcp38_330-OMcp38_330*ROcp38_230)+qdd[31]*ROcp38_130;
  OPcp38_231 = OPcp38_230-qd[31]*(OMcp38_130*ROcp38_330-OMcp38_330*ROcp38_130)+qdd[31]*ROcp38_230;
  OPcp38_331 = OPcp38_330+qd[31]*(OMcp38_130*ROcp38_230-OMcp38_230*ROcp38_130)+qdd[31]*ROcp38_330;
  RLcp38_132 = ROcp38_731*s->dpt[3][62];
  RLcp38_232 = ROcp38_831*s->dpt[3][62];
  RLcp38_332 = ROcp38_931*s->dpt[3][62];
  ORcp38_132 = OMcp38_231*RLcp38_332-OMcp38_331*RLcp38_232;
  ORcp38_232 = -(OMcp38_131*RLcp38_332-OMcp38_331*RLcp38_132);
  ORcp38_332 = OMcp38_131*RLcp38_232-OMcp38_231*RLcp38_132;
  PxF13[1] = q[1]+RLcp38_123+RLcp38_125+RLcp38_130+RLcp38_131+RLcp38_132;
  PxF13[2] = q[2]+RLcp38_223+RLcp38_225+RLcp38_230+RLcp38_231+RLcp38_232;
  PxF13[3] = q[3]+RLcp38_323+RLcp38_325+RLcp38_330+RLcp38_331+RLcp38_332;
  RxF13[1][1] = ROcp38_132;
  RxF13[1][2] = ROcp38_232;
  RxF13[1][3] = ROcp38_332;
  RxF13[2][1] = ROcp38_432;
  RxF13[2][2] = ROcp38_532;
  RxF13[2][3] = ROcp38_632;
  RxF13[3][1] = ROcp38_731;
  RxF13[3][2] = ROcp38_831;
  RxF13[3][3] = ROcp38_931;
  VxF13[1] = qd[1]+ORcp38_123+ORcp38_125+ORcp38_130+ORcp38_131+ORcp38_132;
  VxF13[2] = qd[2]+ORcp38_223+ORcp38_225+ORcp38_230+ORcp38_231+ORcp38_232;
  VxF13[3] = qd[3]+ORcp38_323+ORcp38_325+ORcp38_330+ORcp38_331+ORcp38_332;
  OMxF13[1] = OMcp38_131+qd[32]*ROcp38_731;
  OMxF13[2] = OMcp38_231+qd[32]*ROcp38_831;
  OMxF13[3] = OMcp38_331+qd[32]*ROcp38_931;
  AxF13[1] = qdd[1]+OMcp38_224*ORcp38_325+OMcp38_225*ORcp38_330+OMcp38_230*ORcp38_331+OMcp38_231*ORcp38_332+OMcp38_26*
 ORcp38_323-OMcp38_324*ORcp38_225-OMcp38_325*ORcp38_230-OMcp38_330*ORcp38_231-OMcp38_331*ORcp38_232-OMcp38_36*ORcp38_223+
 OPcp38_224*RLcp38_325+OPcp38_225*RLcp38_330+OPcp38_230*RLcp38_331+OPcp38_231*RLcp38_332+OPcp38_26*RLcp38_323-OPcp38_324*
 RLcp38_225-OPcp38_325*RLcp38_230-OPcp38_330*RLcp38_231-OPcp38_331*RLcp38_232-OPcp38_36*RLcp38_223;
  AxF13[2] = qdd[2]-OMcp38_124*ORcp38_325-OMcp38_125*ORcp38_330-OMcp38_130*ORcp38_331-OMcp38_131*ORcp38_332-OMcp38_16*
 ORcp38_323+OMcp38_324*ORcp38_125+OMcp38_325*ORcp38_130+OMcp38_330*ORcp38_131+OMcp38_331*ORcp38_132+OMcp38_36*ORcp38_123-
 OPcp38_124*RLcp38_325-OPcp38_125*RLcp38_330-OPcp38_130*RLcp38_331-OPcp38_131*RLcp38_332-OPcp38_16*RLcp38_323+OPcp38_324*
 RLcp38_125+OPcp38_325*RLcp38_130+OPcp38_330*RLcp38_131+OPcp38_331*RLcp38_132+OPcp38_36*RLcp38_123;
  AxF13[3] = qdd[3]+OMcp38_124*ORcp38_225+OMcp38_125*ORcp38_230+OMcp38_130*ORcp38_231+OMcp38_131*ORcp38_232+OMcp38_16*
 ORcp38_223-OMcp38_224*ORcp38_125-OMcp38_225*ORcp38_130-OMcp38_230*ORcp38_131-OMcp38_231*ORcp38_132-OMcp38_26*ORcp38_123+
 OPcp38_124*RLcp38_225+OPcp38_125*RLcp38_230+OPcp38_130*RLcp38_231+OPcp38_131*RLcp38_232+OPcp38_16*RLcp38_223-OPcp38_224*
 RLcp38_125-OPcp38_225*RLcp38_130-OPcp38_230*RLcp38_131-OPcp38_231*RLcp38_132-OPcp38_26*RLcp38_123;
  OMPxF13[1] = OPcp38_131+qd[32]*(OMcp38_231*ROcp38_931-OMcp38_331*ROcp38_831)+qdd[32]*ROcp38_731;
  OMPxF13[2] = OPcp38_231-qd[32]*(OMcp38_131*ROcp38_931-OMcp38_331*ROcp38_731)+qdd[32]*ROcp38_831;
  OMPxF13[3] = OPcp38_331+qd[32]*(OMcp38_131*ROcp38_831-OMcp38_231*ROcp38_731)+qdd[32]*ROcp38_931;
 
// Sensor Forces Computation 

  SWr13 = user_ExtForces(PxF13,RxF13,VxF13,OMxF13,AxF13,OMPxF13,s,tsim,13);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc139 = ROcp38_132*SWr13[1]+ROcp38_232*SWr13[2]+ROcp38_332*SWr13[3];
  xfrc239 = ROcp38_432*SWr13[1]+ROcp38_532*SWr13[2]+ROcp38_632*SWr13[3];
  xfrc339 = ROcp38_731*SWr13[1]+ROcp38_831*SWr13[2]+ROcp38_931*SWr13[3];
  frc[1][32] = s->frc[1][32]+xfrc139;
  frc[2][32] = s->frc[2][32]+xfrc239;
  frc[3][32] = s->frc[3][32]+xfrc339;
  xtrq139 = ROcp38_132*SWr13[4]+ROcp38_232*SWr13[5]+ROcp38_332*SWr13[6];
  xtrq239 = ROcp38_432*SWr13[4]+ROcp38_532*SWr13[5]+ROcp38_632*SWr13[6];
  xtrq339 = ROcp38_731*SWr13[4]+ROcp38_831*SWr13[5]+ROcp38_931*SWr13[6];
  trq[1][32] = s->trq[1][32]+xtrq139-xfrc239*(SWr13[9]-s->l[3][32])+xfrc339*(SWr13[8]-s->l[2][32]);
  trq[2][32] = s->trq[2][32]+xtrq239+xfrc139*(SWr13[9]-s->l[3][32])-xfrc339*(SWr13[7]-s->l[1][32]);
  trq[3][32] = s->trq[3][32]+xtrq339-xfrc139*(SWr13[8]-s->l[2][32])+xfrc239*(SWr13[7]-s->l[1][32]);

// = = Block_0_0_1_14_0_1 = = 
 
// Sensor Kinematics 


  ROcp39_25 = S4*S5;
  ROcp39_35 = -C4*S5;
  ROcp39_85 = -S4*C5;
  ROcp39_95 = C4*C5;
  ROcp39_16 = C5*C6;
  ROcp39_26 = ROcp39_25*C6+C4*S6;
  ROcp39_36 = ROcp39_35*C6+S4*S6;
  ROcp39_46 = -C5*S6;
  ROcp39_56 = -(ROcp39_25*S6-C4*C6);
  ROcp39_66 = -(ROcp39_35*S6-S4*C6);
  OMcp39_25 = qd[5]*C4;
  OMcp39_35 = qd[5]*S4;
  OMcp39_16 = qd[4]+qd[6]*S5;
  OMcp39_26 = OMcp39_25+qd[6]*ROcp39_85;
  OMcp39_36 = OMcp39_35+qd[6]*ROcp39_95;
  OPcp39_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp39_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp39_95-OMcp39_35*S5)-qdd[5]*C4-qdd[6]*ROcp39_85);
  OPcp39_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp39_85-OMcp39_25*S5)+qdd[5]*S4+qdd[6]*ROcp39_95;

// = = Block_0_0_1_14_0_8 = = 
 
// Sensor Kinematics 


  ROcp39_423 = ROcp39_46*C23+S23*S5;
  ROcp39_523 = ROcp39_56*C23+ROcp39_85*S23;
  ROcp39_623 = ROcp39_66*C23+ROcp39_95*S23;
  ROcp39_723 = -(ROcp39_46*S23-C23*S5);
  ROcp39_823 = -(ROcp39_56*S23-ROcp39_85*C23);
  ROcp39_923 = -(ROcp39_66*S23-ROcp39_95*C23);
  ROcp39_124 = ROcp39_16*C24-ROcp39_723*S24;
  ROcp39_224 = ROcp39_26*C24-ROcp39_823*S24;
  ROcp39_324 = ROcp39_36*C24-ROcp39_923*S24;
  ROcp39_724 = ROcp39_16*S24+ROcp39_723*C24;
  ROcp39_824 = ROcp39_26*S24+ROcp39_823*C24;
  ROcp39_924 = ROcp39_36*S24+ROcp39_923*C24;
  ROcp39_125 = ROcp39_124*C25+ROcp39_423*S25;
  ROcp39_225 = ROcp39_224*C25+ROcp39_523*S25;
  ROcp39_325 = ROcp39_324*C25+ROcp39_623*S25;
  ROcp39_425 = -(ROcp39_124*S25-ROcp39_423*C25);
  ROcp39_525 = -(ROcp39_224*S25-ROcp39_523*C25);
  ROcp39_625 = -(ROcp39_324*S25-ROcp39_623*C25);
  RLcp39_123 = ROcp39_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp39_223 = ROcp39_26*s->dpt[1][4]+ROcp39_85*s->dpt[3][4];
  RLcp39_323 = ROcp39_36*s->dpt[1][4]+ROcp39_95*s->dpt[3][4];
  OMcp39_123 = OMcp39_16+qd[23]*ROcp39_16;
  OMcp39_223 = OMcp39_26+qd[23]*ROcp39_26;
  OMcp39_323 = OMcp39_36+qd[23]*ROcp39_36;
  ORcp39_123 = OMcp39_26*RLcp39_323-OMcp39_36*RLcp39_223;
  ORcp39_223 = -(OMcp39_16*RLcp39_323-OMcp39_36*RLcp39_123);
  ORcp39_323 = OMcp39_16*RLcp39_223-OMcp39_26*RLcp39_123;
  OMcp39_124 = OMcp39_123+qd[24]*ROcp39_423;
  OMcp39_224 = OMcp39_223+qd[24]*ROcp39_523;
  OMcp39_324 = OMcp39_323+qd[24]*ROcp39_623;
  OPcp39_124 = OPcp39_16+qd[23]*(OMcp39_26*ROcp39_36-OMcp39_36*ROcp39_26)+qd[24]*(OMcp39_223*ROcp39_623-OMcp39_323*
 ROcp39_523)+qdd[23]*ROcp39_16+qdd[24]*ROcp39_423;
  OPcp39_224 = OPcp39_26-qd[23]*(OMcp39_16*ROcp39_36-OMcp39_36*ROcp39_16)-qd[24]*(OMcp39_123*ROcp39_623-OMcp39_323*
 ROcp39_423)+qdd[23]*ROcp39_26+qdd[24]*ROcp39_523;
  OPcp39_324 = OPcp39_36+qd[23]*(OMcp39_16*ROcp39_26-OMcp39_26*ROcp39_16)+qd[24]*(OMcp39_123*ROcp39_523-OMcp39_223*
 ROcp39_423)+qdd[23]*ROcp39_36+qdd[24]*ROcp39_623;
  RLcp39_125 = ROcp39_724*s->dpt[3][44];
  RLcp39_225 = ROcp39_824*s->dpt[3][44];
  RLcp39_325 = ROcp39_924*s->dpt[3][44];
  OMcp39_125 = OMcp39_124+qd[25]*ROcp39_724;
  OMcp39_225 = OMcp39_224+qd[25]*ROcp39_824;
  OMcp39_325 = OMcp39_324+qd[25]*ROcp39_924;
  ORcp39_125 = OMcp39_224*RLcp39_325-OMcp39_324*RLcp39_225;
  ORcp39_225 = -(OMcp39_124*RLcp39_325-OMcp39_324*RLcp39_125);
  ORcp39_325 = OMcp39_124*RLcp39_225-OMcp39_224*RLcp39_125;
  OPcp39_125 = OPcp39_124+qd[25]*(OMcp39_224*ROcp39_924-OMcp39_324*ROcp39_824)+qdd[25]*ROcp39_724;
  OPcp39_225 = OPcp39_224-qd[25]*(OMcp39_124*ROcp39_924-OMcp39_324*ROcp39_724)+qdd[25]*ROcp39_824;
  OPcp39_325 = OPcp39_324+qd[25]*(OMcp39_124*ROcp39_824-OMcp39_224*ROcp39_724)+qdd[25]*ROcp39_924;

// = = Block_0_0_1_14_0_10 = = 
 
// Sensor Kinematics 


  ROcp39_130 = ROcp39_125*C30-ROcp39_724*S30;
  ROcp39_230 = ROcp39_225*C30-ROcp39_824*S30;
  ROcp39_330 = ROcp39_325*C30-ROcp39_924*S30;
  ROcp39_730 = ROcp39_125*S30+ROcp39_724*C30;
  ROcp39_830 = ROcp39_225*S30+ROcp39_824*C30;
  ROcp39_930 = ROcp39_325*S30+ROcp39_924*C30;
  ROcp39_431 = ROcp39_425*C31+ROcp39_730*S31;
  ROcp39_531 = ROcp39_525*C31+ROcp39_830*S31;
  ROcp39_631 = ROcp39_625*C31+ROcp39_930*S31;
  ROcp39_731 = -(ROcp39_425*S31-ROcp39_730*C31);
  ROcp39_831 = -(ROcp39_525*S31-ROcp39_830*C31);
  ROcp39_931 = -(ROcp39_625*S31-ROcp39_930*C31);
  ROcp39_132 = ROcp39_130*C32+ROcp39_431*S32;
  ROcp39_232 = ROcp39_230*C32+ROcp39_531*S32;
  ROcp39_332 = ROcp39_330*C32+ROcp39_631*S32;
  ROcp39_432 = -(ROcp39_130*S32-ROcp39_431*C32);
  ROcp39_532 = -(ROcp39_230*S32-ROcp39_531*C32);
  ROcp39_632 = -(ROcp39_330*S32-ROcp39_631*C32);
  ROcp39_133 = ROcp39_132*C33-ROcp39_731*S33;
  ROcp39_233 = ROcp39_232*C33-ROcp39_831*S33;
  ROcp39_333 = ROcp39_332*C33-ROcp39_931*S33;
  ROcp39_733 = ROcp39_132*S33+ROcp39_731*C33;
  ROcp39_833 = ROcp39_232*S33+ROcp39_831*C33;
  ROcp39_933 = ROcp39_332*S33+ROcp39_931*C33;
  RLcp39_130 = ROcp39_125*s->dpt[1][49]+ROcp39_425*s->dpt[2][49]+ROcp39_724*s->dpt[3][49];
  RLcp39_230 = ROcp39_225*s->dpt[1][49]+ROcp39_525*s->dpt[2][49]+ROcp39_824*s->dpt[3][49];
  RLcp39_330 = ROcp39_325*s->dpt[1][49]+ROcp39_625*s->dpt[2][49]+ROcp39_924*s->dpt[3][49];
  OMcp39_130 = OMcp39_125+qd[30]*ROcp39_425;
  OMcp39_230 = OMcp39_225+qd[30]*ROcp39_525;
  OMcp39_330 = OMcp39_325+qd[30]*ROcp39_625;
  ORcp39_130 = OMcp39_225*RLcp39_330-OMcp39_325*RLcp39_230;
  ORcp39_230 = -(OMcp39_125*RLcp39_330-OMcp39_325*RLcp39_130);
  ORcp39_330 = OMcp39_125*RLcp39_230-OMcp39_225*RLcp39_130;
  OPcp39_130 = OPcp39_125+qd[30]*(OMcp39_225*ROcp39_625-OMcp39_325*ROcp39_525)+qdd[30]*ROcp39_425;
  OPcp39_230 = OPcp39_225-qd[30]*(OMcp39_125*ROcp39_625-OMcp39_325*ROcp39_425)+qdd[30]*ROcp39_525;
  OPcp39_330 = OPcp39_325+qd[30]*(OMcp39_125*ROcp39_525-OMcp39_225*ROcp39_425)+qdd[30]*ROcp39_625;
  RLcp39_131 = ROcp39_425*s->dpt[2][60];
  RLcp39_231 = ROcp39_525*s->dpt[2][60];
  RLcp39_331 = ROcp39_625*s->dpt[2][60];
  OMcp39_131 = OMcp39_130+qd[31]*ROcp39_130;
  OMcp39_231 = OMcp39_230+qd[31]*ROcp39_230;
  OMcp39_331 = OMcp39_330+qd[31]*ROcp39_330;
  ORcp39_131 = OMcp39_230*RLcp39_331-OMcp39_330*RLcp39_231;
  ORcp39_231 = -(OMcp39_130*RLcp39_331-OMcp39_330*RLcp39_131);
  ORcp39_331 = OMcp39_130*RLcp39_231-OMcp39_230*RLcp39_131;
  OPcp39_131 = OPcp39_130+qd[31]*(OMcp39_230*ROcp39_330-OMcp39_330*ROcp39_230)+qdd[31]*ROcp39_130;
  OPcp39_231 = OPcp39_230-qd[31]*(OMcp39_130*ROcp39_330-OMcp39_330*ROcp39_130)+qdd[31]*ROcp39_230;
  OPcp39_331 = OPcp39_330+qd[31]*(OMcp39_130*ROcp39_230-OMcp39_230*ROcp39_130)+qdd[31]*ROcp39_330;
  RLcp39_132 = ROcp39_731*s->dpt[3][62];
  RLcp39_232 = ROcp39_831*s->dpt[3][62];
  RLcp39_332 = ROcp39_931*s->dpt[3][62];
  OMcp39_132 = OMcp39_131+qd[32]*ROcp39_731;
  OMcp39_232 = OMcp39_231+qd[32]*ROcp39_831;
  OMcp39_332 = OMcp39_331+qd[32]*ROcp39_931;
  ORcp39_132 = OMcp39_231*RLcp39_332-OMcp39_331*RLcp39_232;
  ORcp39_232 = -(OMcp39_131*RLcp39_332-OMcp39_331*RLcp39_132);
  ORcp39_332 = OMcp39_131*RLcp39_232-OMcp39_231*RLcp39_132;
  OPcp39_132 = OPcp39_131+qd[32]*(OMcp39_231*ROcp39_931-OMcp39_331*ROcp39_831)+qdd[32]*ROcp39_731;
  OPcp39_232 = OPcp39_231-qd[32]*(OMcp39_131*ROcp39_931-OMcp39_331*ROcp39_731)+qdd[32]*ROcp39_831;
  OPcp39_332 = OPcp39_331+qd[32]*(OMcp39_131*ROcp39_831-OMcp39_231*ROcp39_731)+qdd[32]*ROcp39_931;
  RLcp39_133 = ROcp39_731*s->dpt[3][65];
  RLcp39_233 = ROcp39_831*s->dpt[3][65];
  RLcp39_333 = ROcp39_931*s->dpt[3][65];
  ORcp39_133 = OMcp39_232*RLcp39_333-OMcp39_332*RLcp39_233;
  ORcp39_233 = -(OMcp39_132*RLcp39_333-OMcp39_332*RLcp39_133);
  ORcp39_333 = OMcp39_132*RLcp39_233-OMcp39_232*RLcp39_133;
  PxF14[1] = q[1]+RLcp39_123+RLcp39_125+RLcp39_130+RLcp39_131+RLcp39_132+RLcp39_133;
  PxF14[2] = q[2]+RLcp39_223+RLcp39_225+RLcp39_230+RLcp39_231+RLcp39_232+RLcp39_233;
  PxF14[3] = q[3]+RLcp39_323+RLcp39_325+RLcp39_330+RLcp39_331+RLcp39_332+RLcp39_333;
  RxF14[1][1] = ROcp39_133;
  RxF14[1][2] = ROcp39_233;
  RxF14[1][3] = ROcp39_333;
  RxF14[2][1] = ROcp39_432;
  RxF14[2][2] = ROcp39_532;
  RxF14[2][3] = ROcp39_632;
  RxF14[3][1] = ROcp39_733;
  RxF14[3][2] = ROcp39_833;
  RxF14[3][3] = ROcp39_933;
  VxF14[1] = qd[1]+ORcp39_123+ORcp39_125+ORcp39_130+ORcp39_131+ORcp39_132+ORcp39_133;
  VxF14[2] = qd[2]+ORcp39_223+ORcp39_225+ORcp39_230+ORcp39_231+ORcp39_232+ORcp39_233;
  VxF14[3] = qd[3]+ORcp39_323+ORcp39_325+ORcp39_330+ORcp39_331+ORcp39_332+ORcp39_333;
  OMxF14[1] = OMcp39_132+qd[33]*ROcp39_432;
  OMxF14[2] = OMcp39_232+qd[33]*ROcp39_532;
  OMxF14[3] = OMcp39_332+qd[33]*ROcp39_632;
  AxF14[1] = qdd[1]+OMcp39_224*ORcp39_325+OMcp39_225*ORcp39_330+OMcp39_230*ORcp39_331+OMcp39_231*ORcp39_332+OMcp39_232*
 ORcp39_333+OMcp39_26*ORcp39_323-OMcp39_324*ORcp39_225-OMcp39_325*ORcp39_230-OMcp39_330*ORcp39_231-OMcp39_331*ORcp39_232-
 OMcp39_332*ORcp39_233-OMcp39_36*ORcp39_223+OPcp39_224*RLcp39_325+OPcp39_225*RLcp39_330+OPcp39_230*RLcp39_331+OPcp39_231*
 RLcp39_332+OPcp39_232*RLcp39_333+OPcp39_26*RLcp39_323-OPcp39_324*RLcp39_225-OPcp39_325*RLcp39_230-OPcp39_330*RLcp39_231-
 OPcp39_331*RLcp39_232-OPcp39_332*RLcp39_233-OPcp39_36*RLcp39_223;
  AxF14[2] = qdd[2]-OMcp39_124*ORcp39_325-OMcp39_125*ORcp39_330-OMcp39_130*ORcp39_331-OMcp39_131*ORcp39_332-OMcp39_132*
 ORcp39_333-OMcp39_16*ORcp39_323+OMcp39_324*ORcp39_125+OMcp39_325*ORcp39_130+OMcp39_330*ORcp39_131+OMcp39_331*ORcp39_132+
 OMcp39_332*ORcp39_133+OMcp39_36*ORcp39_123-OPcp39_124*RLcp39_325-OPcp39_125*RLcp39_330-OPcp39_130*RLcp39_331-OPcp39_131*
 RLcp39_332-OPcp39_132*RLcp39_333-OPcp39_16*RLcp39_323+OPcp39_324*RLcp39_125+OPcp39_325*RLcp39_130+OPcp39_330*RLcp39_131+
 OPcp39_331*RLcp39_132+OPcp39_332*RLcp39_133+OPcp39_36*RLcp39_123;
  AxF14[3] = qdd[3]+OMcp39_124*ORcp39_225+OMcp39_125*ORcp39_230+OMcp39_130*ORcp39_231+OMcp39_131*ORcp39_232+OMcp39_132*
 ORcp39_233+OMcp39_16*ORcp39_223-OMcp39_224*ORcp39_125-OMcp39_225*ORcp39_130-OMcp39_230*ORcp39_131-OMcp39_231*ORcp39_132-
 OMcp39_232*ORcp39_133-OMcp39_26*ORcp39_123+OPcp39_124*RLcp39_225+OPcp39_125*RLcp39_230+OPcp39_130*RLcp39_231+OPcp39_131*
 RLcp39_232+OPcp39_132*RLcp39_233+OPcp39_16*RLcp39_223-OPcp39_224*RLcp39_125-OPcp39_225*RLcp39_130-OPcp39_230*RLcp39_131-
 OPcp39_231*RLcp39_132-OPcp39_232*RLcp39_133-OPcp39_26*RLcp39_123;
  OMPxF14[1] = OPcp39_132+qd[33]*(OMcp39_232*ROcp39_632-OMcp39_332*ROcp39_532)+qdd[33]*ROcp39_432;
  OMPxF14[2] = OPcp39_232-qd[33]*(OMcp39_132*ROcp39_632-OMcp39_332*ROcp39_432)+qdd[33]*ROcp39_532;
  OMPxF14[3] = OPcp39_332+qd[33]*(OMcp39_132*ROcp39_532-OMcp39_232*ROcp39_432)+qdd[33]*ROcp39_632;
 
// Sensor Forces Computation 

  SWr14 = user_ExtForces(PxF14,RxF14,VxF14,OMxF14,AxF14,OMPxF14,s,tsim,14);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc140 = ROcp39_133*SWr14[1]+ROcp39_233*SWr14[2]+ROcp39_333*SWr14[3];
  xfrc240 = ROcp39_432*SWr14[1]+ROcp39_532*SWr14[2]+ROcp39_632*SWr14[3];
  xfrc340 = ROcp39_733*SWr14[1]+ROcp39_833*SWr14[2]+ROcp39_933*SWr14[3];
  frc[1][33] = s->frc[1][33]+xfrc140;
  frc[2][33] = s->frc[2][33]+xfrc240;
  frc[3][33] = s->frc[3][33]+xfrc340;
  xtrq140 = ROcp39_133*SWr14[4]+ROcp39_233*SWr14[5]+ROcp39_333*SWr14[6];
  xtrq240 = ROcp39_432*SWr14[4]+ROcp39_532*SWr14[5]+ROcp39_632*SWr14[6];
  xtrq340 = ROcp39_733*SWr14[4]+ROcp39_833*SWr14[5]+ROcp39_933*SWr14[6];
  trq[1][33] = s->trq[1][33]+xtrq140-xfrc240*(SWr14[9]-s->l[3][33])+xfrc340*(SWr14[8]-s->l[2][33]);
  trq[2][33] = s->trq[2][33]+xtrq240+xfrc140*(SWr14[9]-s->l[3][33])-xfrc340*(SWr14[7]-s->l[1][33]);
  trq[3][33] = s->trq[3][33]+xtrq340-xfrc140*(SWr14[8]-s->l[2][33])+xfrc240*(SWr14[7]-s->l[1][33]);

// = = Block_0_0_1_14_1_0 = = 
 
// Symbolic Outputs  

  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][12] = s->frc[1][12];
  frc[2][12] = s->frc[2][12];
  frc[3][12] = s->frc[3][12];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][12] = s->trq[1][12];
  trq[2][12] = s->trq[2][12];
  trq[3][12] = s->trq[3][12];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];

// ====== END Task 0 ====== 


}
 

