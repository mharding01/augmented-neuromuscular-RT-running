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
//	==> Generation Date : Thu Nov 26 12:45:46 2015
//
//	==> Project name : coman_init_feet
//	==> using XML input file 
//
//	==> Number of joints : 29
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 5931
//
//	==> Generation Time :  0.100 seconds
//	==> Post-Processing :  0.110 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][29];
// double trq[3][29];
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
 
#include "mbs_extforces_coman_init_feet.h" 
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
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);
  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);

// = = Block_0_0_1_1_0_1 = = 
 
// Sensor Kinematics 


  ROcp24_25 = S4*S5;
  ROcp24_35 = -C4*S5;
  ROcp24_85 = -S4*C5;
  ROcp24_95 = C4*C5;
  ROcp24_16 = C5*C6;
  ROcp24_26 = ROcp24_25*C6+C4*S6;
  ROcp24_36 = ROcp24_35*C6+S4*S6;
  ROcp24_46 = -C5*S6;
  ROcp24_56 = -(ROcp24_25*S6-C4*C6);
  ROcp24_66 = -(ROcp24_35*S6-S4*C6);
  OMcp24_25 = qd[5]*C4;
  OMcp24_35 = qd[5]*S4;
  PxF1[1] = q[1];
  PxF1[2] = q[2];
  PxF1[3] = q[3];
  RxF1[1][1] = ROcp24_16;
  RxF1[1][2] = ROcp24_26;
  RxF1[1][3] = ROcp24_36;
  RxF1[2][1] = ROcp24_46;
  RxF1[2][2] = ROcp24_56;
  RxF1[2][3] = ROcp24_66;
  RxF1[3][1] = S5;
  RxF1[3][2] = ROcp24_85;
  RxF1[3][3] = ROcp24_95;
  VxF1[1] = qd[1];
  VxF1[2] = qd[2];
  VxF1[3] = qd[3];
  OMxF1[1] = qd[4]+qd[6]*S5;
  OMxF1[2] = OMcp24_25+qd[6]*ROcp24_85;
  OMxF1[3] = OMcp24_35+qd[6]*ROcp24_95;
  AxF1[1] = qdd[1];
  AxF1[2] = qdd[2];
  AxF1[3] = qdd[3];
  OMPxF1[1] = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OMPxF1[2] = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp24_95-OMcp24_35*S5)-qdd[5]*C4-qdd[6]*ROcp24_85);
  OMPxF1[3] = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp24_85-OMcp24_25*S5)+qdd[5]*S4+qdd[6]*ROcp24_95;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc125 = ROcp24_16*SWr1[1]+ROcp24_26*SWr1[2]+ROcp24_36*SWr1[3];
  xfrc225 = ROcp24_46*SWr1[1]+ROcp24_56*SWr1[2]+ROcp24_66*SWr1[3];
  xfrc325 = ROcp24_85*SWr1[2]+ROcp24_95*SWr1[3]+SWr1[1]*S5;
  frc[1][6] = s->frc[1][6]+xfrc125;
  frc[2][6] = s->frc[2][6]+xfrc225;
  frc[3][6] = s->frc[3][6]+xfrc325;
  xtrq125 = ROcp24_16*SWr1[4]+ROcp24_26*SWr1[5]+ROcp24_36*SWr1[6];
  xtrq225 = ROcp24_46*SWr1[4]+ROcp24_56*SWr1[5]+ROcp24_66*SWr1[6];
  xtrq325 = ROcp24_85*SWr1[5]+ROcp24_95*SWr1[6]+SWr1[4]*S5;
  trq[1][6] = s->trq[1][6]+xtrq125-xfrc225*(SWr1[9]-s->l[3][6])+xfrc325*(SWr1[8]-s->l[2][6]);
  trq[2][6] = s->trq[2][6]+xtrq225+xfrc125*(SWr1[9]-s->l[3][6])-xfrc325*(SWr1[7]-s->l[1][6]);
  trq[3][6] = s->trq[3][6]+xtrq325-xfrc125*(SWr1[8]-s->l[2][6])+xfrc225*(SWr1[7]-s->l[1][6]);

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp25_25 = S4*S5;
  ROcp25_35 = -C4*S5;
  ROcp25_85 = -S4*C5;
  ROcp25_95 = C4*C5;
  ROcp25_16 = C5*C6;
  ROcp25_26 = ROcp25_25*C6+C4*S6;
  ROcp25_36 = ROcp25_35*C6+S4*S6;
  ROcp25_46 = -C5*S6;
  ROcp25_56 = -(ROcp25_25*S6-C4*C6);
  ROcp25_66 = -(ROcp25_35*S6-S4*C6);
  OMcp25_25 = qd[5]*C4;
  OMcp25_35 = qd[5]*S4;
  OMcp25_16 = qd[4]+qd[6]*S5;
  OMcp25_26 = OMcp25_25+qd[6]*ROcp25_85;
  OMcp25_36 = OMcp25_35+qd[6]*ROcp25_95;
  OPcp25_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp25_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp25_95-OMcp25_35*S5)-qdd[5]*C4-qdd[6]*ROcp25_85);
  OPcp25_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp25_85-OMcp25_25*S5)+qdd[5]*S4+qdd[6]*ROcp25_95;

// = = Block_0_0_1_2_0_2 = = 
 
// Sensor Kinematics 


  ROcp25_17 = ROcp25_16*C7-S5*S7;
  ROcp25_27 = ROcp25_26*C7-ROcp25_85*S7;
  ROcp25_37 = ROcp25_36*C7-ROcp25_95*S7;
  ROcp25_77 = ROcp25_16*S7+S5*C7;
  ROcp25_87 = ROcp25_26*S7+ROcp25_85*C7;
  ROcp25_97 = ROcp25_36*S7+ROcp25_95*C7;
  ROcp25_48 = ROcp25_46*C8+ROcp25_77*S8;
  ROcp25_58 = ROcp25_56*C8+ROcp25_87*S8;
  ROcp25_68 = ROcp25_66*C8+ROcp25_97*S8;
  ROcp25_78 = -(ROcp25_46*S8-ROcp25_77*C8);
  ROcp25_88 = -(ROcp25_56*S8-ROcp25_87*C8);
  ROcp25_98 = -(ROcp25_66*S8-ROcp25_97*C8);
  ROcp25_19 = ROcp25_17*C9+ROcp25_48*S9;
  ROcp25_29 = ROcp25_27*C9+ROcp25_58*S9;
  ROcp25_39 = ROcp25_37*C9+ROcp25_68*S9;
  ROcp25_49 = -(ROcp25_17*S9-ROcp25_48*C9);
  ROcp25_59 = -(ROcp25_27*S9-ROcp25_58*C9);
  ROcp25_69 = -(ROcp25_37*S9-ROcp25_68*C9);
  RLcp25_17 = ROcp25_46*s->dpt[2][2];
  RLcp25_27 = ROcp25_56*s->dpt[2][2];
  RLcp25_37 = ROcp25_66*s->dpt[2][2];
  OMcp25_17 = OMcp25_16+qd[7]*ROcp25_46;
  OMcp25_27 = OMcp25_26+qd[7]*ROcp25_56;
  OMcp25_37 = OMcp25_36+qd[7]*ROcp25_66;
  ORcp25_17 = OMcp25_26*RLcp25_37-OMcp25_36*RLcp25_27;
  ORcp25_27 = -(OMcp25_16*RLcp25_37-OMcp25_36*RLcp25_17);
  ORcp25_37 = OMcp25_16*RLcp25_27-OMcp25_26*RLcp25_17;
  OPcp25_17 = OPcp25_16+qd[7]*(OMcp25_26*ROcp25_66-OMcp25_36*ROcp25_56)+qdd[7]*ROcp25_46;
  OPcp25_27 = OPcp25_26-qd[7]*(OMcp25_16*ROcp25_66-OMcp25_36*ROcp25_46)+qdd[7]*ROcp25_56;
  OPcp25_37 = OPcp25_36+qd[7]*(OMcp25_16*ROcp25_56-OMcp25_26*ROcp25_46)+qdd[7]*ROcp25_66;
  RLcp25_18 = ROcp25_46*s->dpt[2][6];
  RLcp25_28 = ROcp25_56*s->dpt[2][6];
  RLcp25_38 = ROcp25_66*s->dpt[2][6];
  OMcp25_18 = OMcp25_17+qd[8]*ROcp25_17;
  OMcp25_28 = OMcp25_27+qd[8]*ROcp25_27;
  OMcp25_38 = OMcp25_37+qd[8]*ROcp25_37;
  ORcp25_18 = OMcp25_27*RLcp25_38-OMcp25_37*RLcp25_28;
  ORcp25_28 = -(OMcp25_17*RLcp25_38-OMcp25_37*RLcp25_18);
  ORcp25_38 = OMcp25_17*RLcp25_28-OMcp25_27*RLcp25_18;
  OPcp25_18 = OPcp25_17+qd[8]*(OMcp25_27*ROcp25_37-OMcp25_37*ROcp25_27)+qdd[8]*ROcp25_17;
  OPcp25_28 = OPcp25_27-qd[8]*(OMcp25_17*ROcp25_37-OMcp25_37*ROcp25_17)+qdd[8]*ROcp25_27;
  OPcp25_38 = OPcp25_37+qd[8]*(OMcp25_17*ROcp25_27-OMcp25_27*ROcp25_17)+qdd[8]*ROcp25_37;
  RLcp25_19 = ROcp25_78*s->dpt[3][8];
  RLcp25_29 = ROcp25_88*s->dpt[3][8];
  RLcp25_39 = ROcp25_98*s->dpt[3][8];
  ORcp25_19 = OMcp25_28*RLcp25_39-OMcp25_38*RLcp25_29;
  ORcp25_29 = -(OMcp25_18*RLcp25_39-OMcp25_38*RLcp25_19);
  ORcp25_39 = OMcp25_18*RLcp25_29-OMcp25_28*RLcp25_19;
  PxF2[1] = q[1]+RLcp25_17+RLcp25_18+RLcp25_19;
  PxF2[2] = q[2]+RLcp25_27+RLcp25_28+RLcp25_29;
  PxF2[3] = q[3]+RLcp25_37+RLcp25_38+RLcp25_39;
  RxF2[1][1] = ROcp25_19;
  RxF2[1][2] = ROcp25_29;
  RxF2[1][3] = ROcp25_39;
  RxF2[2][1] = ROcp25_49;
  RxF2[2][2] = ROcp25_59;
  RxF2[2][3] = ROcp25_69;
  RxF2[3][1] = ROcp25_78;
  RxF2[3][2] = ROcp25_88;
  RxF2[3][3] = ROcp25_98;
  VxF2[1] = qd[1]+ORcp25_17+ORcp25_18+ORcp25_19;
  VxF2[2] = qd[2]+ORcp25_27+ORcp25_28+ORcp25_29;
  VxF2[3] = qd[3]+ORcp25_37+ORcp25_38+ORcp25_39;
  OMxF2[1] = OMcp25_18+qd[9]*ROcp25_78;
  OMxF2[2] = OMcp25_28+qd[9]*ROcp25_88;
  OMxF2[3] = OMcp25_38+qd[9]*ROcp25_98;
  AxF2[1] = qdd[1]+OMcp25_26*ORcp25_37+OMcp25_27*ORcp25_38+OMcp25_28*ORcp25_39-OMcp25_36*ORcp25_27-OMcp25_37*ORcp25_28-
 OMcp25_38*ORcp25_29+OPcp25_26*RLcp25_37+OPcp25_27*RLcp25_38+OPcp25_28*RLcp25_39-OPcp25_36*RLcp25_27-OPcp25_37*RLcp25_28-
 OPcp25_38*RLcp25_29;
  AxF2[2] = qdd[2]-OMcp25_16*ORcp25_37-OMcp25_17*ORcp25_38-OMcp25_18*ORcp25_39+OMcp25_36*ORcp25_17+OMcp25_37*ORcp25_18+
 OMcp25_38*ORcp25_19-OPcp25_16*RLcp25_37-OPcp25_17*RLcp25_38-OPcp25_18*RLcp25_39+OPcp25_36*RLcp25_17+OPcp25_37*RLcp25_18+
 OPcp25_38*RLcp25_19;
  AxF2[3] = qdd[3]+OMcp25_16*ORcp25_27+OMcp25_17*ORcp25_28+OMcp25_18*ORcp25_29-OMcp25_26*ORcp25_17-OMcp25_27*ORcp25_18-
 OMcp25_28*ORcp25_19+OPcp25_16*RLcp25_27+OPcp25_17*RLcp25_28+OPcp25_18*RLcp25_29-OPcp25_26*RLcp25_17-OPcp25_27*RLcp25_18-
 OPcp25_28*RLcp25_19;
  OMPxF2[1] = OPcp25_18+qd[9]*(OMcp25_28*ROcp25_98-OMcp25_38*ROcp25_88)+qdd[9]*ROcp25_78;
  OMPxF2[2] = OPcp25_28-qd[9]*(OMcp25_18*ROcp25_98-OMcp25_38*ROcp25_78)+qdd[9]*ROcp25_88;
  OMPxF2[3] = OPcp25_38+qd[9]*(OMcp25_18*ROcp25_88-OMcp25_28*ROcp25_78)+qdd[9]*ROcp25_98;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc126 = ROcp25_19*SWr2[1]+ROcp25_29*SWr2[2]+ROcp25_39*SWr2[3];
  xfrc226 = ROcp25_49*SWr2[1]+ROcp25_59*SWr2[2]+ROcp25_69*SWr2[3];
  xfrc326 = ROcp25_78*SWr2[1]+ROcp25_88*SWr2[2]+ROcp25_98*SWr2[3];
  frc[1][9] = s->frc[1][9]+xfrc126;
  frc[2][9] = s->frc[2][9]+xfrc226;
  frc[3][9] = s->frc[3][9]+xfrc326;
  xtrq126 = ROcp25_19*SWr2[4]+ROcp25_29*SWr2[5]+ROcp25_39*SWr2[6];
  xtrq226 = ROcp25_49*SWr2[4]+ROcp25_59*SWr2[5]+ROcp25_69*SWr2[6];
  xtrq326 = ROcp25_78*SWr2[4]+ROcp25_88*SWr2[5]+ROcp25_98*SWr2[6];
  trq[1][9] = s->trq[1][9]+xtrq126-xfrc226*(SWr2[9]-s->l[3][9])+xfrc326*(SWr2[8]-s->l[2][9]);
  trq[2][9] = s->trq[2][9]+xtrq226+xfrc126*(SWr2[9]-s->l[3][9])-xfrc326*(SWr2[7]-s->l[1][9]);
  trq[3][9] = s->trq[3][9]+xtrq326-xfrc126*(SWr2[8]-s->l[2][9])+xfrc226*(SWr2[7]-s->l[1][9]);

// = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp26_16 = qd[4]+qd[6]*S5;
  OMcp26_26 = OMcp26_25+qd[6]*ROcp26_85;
  OMcp26_36 = OMcp26_35+qd[6]*ROcp26_95;
  OPcp26_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp26_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp26_95-OMcp26_35*S5)-qdd[5]*C4-qdd[6]*ROcp26_85);
  OPcp26_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp26_85-OMcp26_25*S5)+qdd[5]*S4+qdd[6]*ROcp26_95;

// = = Block_0_0_1_3_0_2 = = 
 
// Sensor Kinematics 


  ROcp26_17 = ROcp26_16*C7-S5*S7;
  ROcp26_27 = ROcp26_26*C7-ROcp26_85*S7;
  ROcp26_37 = ROcp26_36*C7-ROcp26_95*S7;
  ROcp26_77 = ROcp26_16*S7+S5*C7;
  ROcp26_87 = ROcp26_26*S7+ROcp26_85*C7;
  ROcp26_97 = ROcp26_36*S7+ROcp26_95*C7;
  ROcp26_48 = ROcp26_46*C8+ROcp26_77*S8;
  ROcp26_58 = ROcp26_56*C8+ROcp26_87*S8;
  ROcp26_68 = ROcp26_66*C8+ROcp26_97*S8;
  ROcp26_78 = -(ROcp26_46*S8-ROcp26_77*C8);
  ROcp26_88 = -(ROcp26_56*S8-ROcp26_87*C8);
  ROcp26_98 = -(ROcp26_66*S8-ROcp26_97*C8);
  ROcp26_19 = ROcp26_17*C9+ROcp26_48*S9;
  ROcp26_29 = ROcp26_27*C9+ROcp26_58*S9;
  ROcp26_39 = ROcp26_37*C9+ROcp26_68*S9;
  ROcp26_49 = -(ROcp26_17*S9-ROcp26_48*C9);
  ROcp26_59 = -(ROcp26_27*S9-ROcp26_58*C9);
  ROcp26_69 = -(ROcp26_37*S9-ROcp26_68*C9);
  ROcp26_110 = ROcp26_19*C10-ROcp26_78*S10;
  ROcp26_210 = ROcp26_29*C10-ROcp26_88*S10;
  ROcp26_310 = ROcp26_39*C10-ROcp26_98*S10;
  ROcp26_710 = ROcp26_19*S10+ROcp26_78*C10;
  ROcp26_810 = ROcp26_29*S10+ROcp26_88*C10;
  ROcp26_910 = ROcp26_39*S10+ROcp26_98*C10;
  RLcp26_17 = ROcp26_46*s->dpt[2][2];
  RLcp26_27 = ROcp26_56*s->dpt[2][2];
  RLcp26_37 = ROcp26_66*s->dpt[2][2];
  OMcp26_17 = OMcp26_16+qd[7]*ROcp26_46;
  OMcp26_27 = OMcp26_26+qd[7]*ROcp26_56;
  OMcp26_37 = OMcp26_36+qd[7]*ROcp26_66;
  ORcp26_17 = OMcp26_26*RLcp26_37-OMcp26_36*RLcp26_27;
  ORcp26_27 = -(OMcp26_16*RLcp26_37-OMcp26_36*RLcp26_17);
  ORcp26_37 = OMcp26_16*RLcp26_27-OMcp26_26*RLcp26_17;
  OPcp26_17 = OPcp26_16+qd[7]*(OMcp26_26*ROcp26_66-OMcp26_36*ROcp26_56)+qdd[7]*ROcp26_46;
  OPcp26_27 = OPcp26_26-qd[7]*(OMcp26_16*ROcp26_66-OMcp26_36*ROcp26_46)+qdd[7]*ROcp26_56;
  OPcp26_37 = OPcp26_36+qd[7]*(OMcp26_16*ROcp26_56-OMcp26_26*ROcp26_46)+qdd[7]*ROcp26_66;
  RLcp26_18 = ROcp26_46*s->dpt[2][6];
  RLcp26_28 = ROcp26_56*s->dpt[2][6];
  RLcp26_38 = ROcp26_66*s->dpt[2][6];
  OMcp26_18 = OMcp26_17+qd[8]*ROcp26_17;
  OMcp26_28 = OMcp26_27+qd[8]*ROcp26_27;
  OMcp26_38 = OMcp26_37+qd[8]*ROcp26_37;
  ORcp26_18 = OMcp26_27*RLcp26_38-OMcp26_37*RLcp26_28;
  ORcp26_28 = -(OMcp26_17*RLcp26_38-OMcp26_37*RLcp26_18);
  ORcp26_38 = OMcp26_17*RLcp26_28-OMcp26_27*RLcp26_18;
  OPcp26_18 = OPcp26_17+qd[8]*(OMcp26_27*ROcp26_37-OMcp26_37*ROcp26_27)+qdd[8]*ROcp26_17;
  OPcp26_28 = OPcp26_27-qd[8]*(OMcp26_17*ROcp26_37-OMcp26_37*ROcp26_17)+qdd[8]*ROcp26_27;
  OPcp26_38 = OPcp26_37+qd[8]*(OMcp26_17*ROcp26_27-OMcp26_27*ROcp26_17)+qdd[8]*ROcp26_37;
  RLcp26_19 = ROcp26_78*s->dpt[3][8];
  RLcp26_29 = ROcp26_88*s->dpt[3][8];
  RLcp26_39 = ROcp26_98*s->dpt[3][8];
  OMcp26_19 = OMcp26_18+qd[9]*ROcp26_78;
  OMcp26_29 = OMcp26_28+qd[9]*ROcp26_88;
  OMcp26_39 = OMcp26_38+qd[9]*ROcp26_98;
  ORcp26_19 = OMcp26_28*RLcp26_39-OMcp26_38*RLcp26_29;
  ORcp26_29 = -(OMcp26_18*RLcp26_39-OMcp26_38*RLcp26_19);
  ORcp26_39 = OMcp26_18*RLcp26_29-OMcp26_28*RLcp26_19;
  OPcp26_19 = OPcp26_18+qd[9]*(OMcp26_28*ROcp26_98-OMcp26_38*ROcp26_88)+qdd[9]*ROcp26_78;
  OPcp26_29 = OPcp26_28-qd[9]*(OMcp26_18*ROcp26_98-OMcp26_38*ROcp26_78)+qdd[9]*ROcp26_88;
  OPcp26_39 = OPcp26_38+qd[9]*(OMcp26_18*ROcp26_88-OMcp26_28*ROcp26_78)+qdd[9]*ROcp26_98;
  RLcp26_110 = ROcp26_78*s->dpt[3][11];
  RLcp26_210 = ROcp26_88*s->dpt[3][11];
  RLcp26_310 = ROcp26_98*s->dpt[3][11];
  ORcp26_110 = OMcp26_29*RLcp26_310-OMcp26_39*RLcp26_210;
  ORcp26_210 = -(OMcp26_19*RLcp26_310-OMcp26_39*RLcp26_110);
  ORcp26_310 = OMcp26_19*RLcp26_210-OMcp26_29*RLcp26_110;
  PxF3[1] = q[1]+RLcp26_110+RLcp26_17+RLcp26_18+RLcp26_19;
  PxF3[2] = q[2]+RLcp26_210+RLcp26_27+RLcp26_28+RLcp26_29;
  PxF3[3] = q[3]+RLcp26_310+RLcp26_37+RLcp26_38+RLcp26_39;
  RxF3[1][1] = ROcp26_110;
  RxF3[1][2] = ROcp26_210;
  RxF3[1][3] = ROcp26_310;
  RxF3[2][1] = ROcp26_49;
  RxF3[2][2] = ROcp26_59;
  RxF3[2][3] = ROcp26_69;
  RxF3[3][1] = ROcp26_710;
  RxF3[3][2] = ROcp26_810;
  RxF3[3][3] = ROcp26_910;
  VxF3[1] = qd[1]+ORcp26_110+ORcp26_17+ORcp26_18+ORcp26_19;
  VxF3[2] = qd[2]+ORcp26_210+ORcp26_27+ORcp26_28+ORcp26_29;
  VxF3[3] = qd[3]+ORcp26_310+ORcp26_37+ORcp26_38+ORcp26_39;
  OMxF3[1] = OMcp26_19+qd[10]*ROcp26_49;
  OMxF3[2] = OMcp26_29+qd[10]*ROcp26_59;
  OMxF3[3] = OMcp26_39+qd[10]*ROcp26_69;
  AxF3[1] = qdd[1]+OMcp26_26*ORcp26_37+OMcp26_27*ORcp26_38+OMcp26_28*ORcp26_39+OMcp26_29*ORcp26_310-OMcp26_36*ORcp26_27-
 OMcp26_37*ORcp26_28-OMcp26_38*ORcp26_29-OMcp26_39*ORcp26_210+OPcp26_26*RLcp26_37+OPcp26_27*RLcp26_38+OPcp26_28*RLcp26_39+
 OPcp26_29*RLcp26_310-OPcp26_36*RLcp26_27-OPcp26_37*RLcp26_28-OPcp26_38*RLcp26_29-OPcp26_39*RLcp26_210;
  AxF3[2] = qdd[2]-OMcp26_16*ORcp26_37-OMcp26_17*ORcp26_38-OMcp26_18*ORcp26_39-OMcp26_19*ORcp26_310+OMcp26_36*ORcp26_17+
 OMcp26_37*ORcp26_18+OMcp26_38*ORcp26_19+OMcp26_39*ORcp26_110-OPcp26_16*RLcp26_37-OPcp26_17*RLcp26_38-OPcp26_18*RLcp26_39-
 OPcp26_19*RLcp26_310+OPcp26_36*RLcp26_17+OPcp26_37*RLcp26_18+OPcp26_38*RLcp26_19+OPcp26_39*RLcp26_110;
  AxF3[3] = qdd[3]+OMcp26_16*ORcp26_27+OMcp26_17*ORcp26_28+OMcp26_18*ORcp26_29+OMcp26_19*ORcp26_210-OMcp26_26*ORcp26_17-
 OMcp26_27*ORcp26_18-OMcp26_28*ORcp26_19-OMcp26_29*ORcp26_110+OPcp26_16*RLcp26_27+OPcp26_17*RLcp26_28+OPcp26_18*RLcp26_29+
 OPcp26_19*RLcp26_210-OPcp26_26*RLcp26_17-OPcp26_27*RLcp26_18-OPcp26_28*RLcp26_19-OPcp26_29*RLcp26_110;
  OMPxF3[1] = OPcp26_19+qd[10]*(OMcp26_29*ROcp26_69-OMcp26_39*ROcp26_59)+qdd[10]*ROcp26_49;
  OMPxF3[2] = OPcp26_29-qd[10]*(OMcp26_19*ROcp26_69-OMcp26_39*ROcp26_49)+qdd[10]*ROcp26_59;
  OMPxF3[3] = OPcp26_39+qd[10]*(OMcp26_19*ROcp26_59-OMcp26_29*ROcp26_49)+qdd[10]*ROcp26_69;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc127 = ROcp26_110*SWr3[1]+ROcp26_210*SWr3[2]+ROcp26_310*SWr3[3];
  xfrc227 = ROcp26_49*SWr3[1]+ROcp26_59*SWr3[2]+ROcp26_69*SWr3[3];
  xfrc327 = ROcp26_710*SWr3[1]+ROcp26_810*SWr3[2]+ROcp26_910*SWr3[3];
  frc[1][10] = s->frc[1][10]+xfrc127;
  frc[2][10] = s->frc[2][10]+xfrc227;
  frc[3][10] = s->frc[3][10]+xfrc327;
  xtrq127 = ROcp26_110*SWr3[4]+ROcp26_210*SWr3[5]+ROcp26_310*SWr3[6];
  xtrq227 = ROcp26_49*SWr3[4]+ROcp26_59*SWr3[5]+ROcp26_69*SWr3[6];
  xtrq327 = ROcp26_710*SWr3[4]+ROcp26_810*SWr3[5]+ROcp26_910*SWr3[6];
  trq[1][10] = s->trq[1][10]+xtrq127-xfrc227*(SWr3[9]-s->l[3][10])+xfrc327*(SWr3[8]-s->l[2][10]);
  trq[2][10] = s->trq[2][10]+xtrq227+xfrc127*(SWr3[9]-s->l[3][10])-xfrc327*(SWr3[7]-s->l[1][10]);
  trq[3][10] = s->trq[3][10]+xtrq327-xfrc127*(SWr3[8]-s->l[2][10])+xfrc227*(SWr3[7]-s->l[1][10]);

// = = Block_0_0_1_4_0_1 = = 
 
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

// = = Block_0_0_1_4_0_2 = = 
 
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
  ROcp27_110 = ROcp27_19*C10-ROcp27_78*S10;
  ROcp27_210 = ROcp27_29*C10-ROcp27_88*S10;
  ROcp27_310 = ROcp27_39*C10-ROcp27_98*S10;
  ROcp27_710 = ROcp27_19*S10+ROcp27_78*C10;
  ROcp27_810 = ROcp27_29*S10+ROcp27_88*C10;
  ROcp27_910 = ROcp27_39*S10+ROcp27_98*C10;
  ROcp27_411 = ROcp27_49*C11+ROcp27_710*S11;
  ROcp27_511 = ROcp27_59*C11+ROcp27_810*S11;
  ROcp27_611 = ROcp27_69*C11+ROcp27_910*S11;
  ROcp27_711 = -(ROcp27_49*S11-ROcp27_710*C11);
  ROcp27_811 = -(ROcp27_59*S11-ROcp27_810*C11);
  ROcp27_911 = -(ROcp27_69*S11-ROcp27_910*C11);
  ROcp27_112 = ROcp27_110*C12-ROcp27_711*S12;
  ROcp27_212 = ROcp27_210*C12-ROcp27_811*S12;
  ROcp27_312 = ROcp27_310*C12-ROcp27_911*S12;
  ROcp27_712 = ROcp27_110*S12+ROcp27_711*C12;
  ROcp27_812 = ROcp27_210*S12+ROcp27_811*C12;
  ROcp27_912 = ROcp27_310*S12+ROcp27_911*C12;
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
  OMcp27_19 = OMcp27_18+qd[9]*ROcp27_78;
  OMcp27_29 = OMcp27_28+qd[9]*ROcp27_88;
  OMcp27_39 = OMcp27_38+qd[9]*ROcp27_98;
  ORcp27_19 = OMcp27_28*RLcp27_39-OMcp27_38*RLcp27_29;
  ORcp27_29 = -(OMcp27_18*RLcp27_39-OMcp27_38*RLcp27_19);
  ORcp27_39 = OMcp27_18*RLcp27_29-OMcp27_28*RLcp27_19;
  OPcp27_19 = OPcp27_18+qd[9]*(OMcp27_28*ROcp27_98-OMcp27_38*ROcp27_88)+qdd[9]*ROcp27_78;
  OPcp27_29 = OPcp27_28-qd[9]*(OMcp27_18*ROcp27_98-OMcp27_38*ROcp27_78)+qdd[9]*ROcp27_88;
  OPcp27_39 = OPcp27_38+qd[9]*(OMcp27_18*ROcp27_88-OMcp27_28*ROcp27_78)+qdd[9]*ROcp27_98;
  RLcp27_110 = ROcp27_78*s->dpt[3][11];
  RLcp27_210 = ROcp27_88*s->dpt[3][11];
  RLcp27_310 = ROcp27_98*s->dpt[3][11];
  OMcp27_110 = OMcp27_19+qd[10]*ROcp27_49;
  OMcp27_210 = OMcp27_29+qd[10]*ROcp27_59;
  OMcp27_310 = OMcp27_39+qd[10]*ROcp27_69;
  ORcp27_110 = OMcp27_29*RLcp27_310-OMcp27_39*RLcp27_210;
  ORcp27_210 = -(OMcp27_19*RLcp27_310-OMcp27_39*RLcp27_110);
  ORcp27_310 = OMcp27_19*RLcp27_210-OMcp27_29*RLcp27_110;
  OPcp27_110 = OPcp27_19+qd[10]*(OMcp27_29*ROcp27_69-OMcp27_39*ROcp27_59)+qdd[10]*ROcp27_49;
  OPcp27_210 = OPcp27_29-qd[10]*(OMcp27_19*ROcp27_69-OMcp27_39*ROcp27_49)+qdd[10]*ROcp27_59;
  OPcp27_310 = OPcp27_39+qd[10]*(OMcp27_19*ROcp27_59-OMcp27_29*ROcp27_49)+qdd[10]*ROcp27_69;
  RLcp27_111 = ROcp27_710*s->dpt[3][14];
  RLcp27_211 = ROcp27_810*s->dpt[3][14];
  RLcp27_311 = ROcp27_910*s->dpt[3][14];
  OMcp27_111 = OMcp27_110+qd[11]*ROcp27_110;
  OMcp27_211 = OMcp27_210+qd[11]*ROcp27_210;
  OMcp27_311 = OMcp27_310+qd[11]*ROcp27_310;
  ORcp27_111 = OMcp27_210*RLcp27_311-OMcp27_310*RLcp27_211;
  ORcp27_211 = -(OMcp27_110*RLcp27_311-OMcp27_310*RLcp27_111);
  ORcp27_311 = OMcp27_110*RLcp27_211-OMcp27_210*RLcp27_111;
  OMcp27_112 = OMcp27_111+qd[12]*ROcp27_411;
  OMcp27_212 = OMcp27_211+qd[12]*ROcp27_511;
  OMcp27_312 = OMcp27_311+qd[12]*ROcp27_611;
  OPcp27_112 = OPcp27_110+qd[11]*(OMcp27_210*ROcp27_310-OMcp27_310*ROcp27_210)+qd[12]*(OMcp27_211*ROcp27_611-OMcp27_311*
 ROcp27_511)+qdd[11]*ROcp27_110+qdd[12]*ROcp27_411;
  OPcp27_212 = OPcp27_210-qd[11]*(OMcp27_110*ROcp27_310-OMcp27_310*ROcp27_110)-qd[12]*(OMcp27_111*ROcp27_611-OMcp27_311*
 ROcp27_411)+qdd[11]*ROcp27_210+qdd[12]*ROcp27_511;
  OPcp27_312 = OPcp27_310+qd[11]*(OMcp27_110*ROcp27_210-OMcp27_210*ROcp27_110)+qd[12]*(OMcp27_111*ROcp27_511-OMcp27_211*
 ROcp27_411)+qdd[11]*ROcp27_310+qdd[12]*ROcp27_611;
  RLcp27_157 = ROcp27_712*s->dpt[3][18];
  RLcp27_257 = ROcp27_812*s->dpt[3][18];
  RLcp27_357 = ROcp27_912*s->dpt[3][18];
  ORcp27_157 = OMcp27_212*RLcp27_357-OMcp27_312*RLcp27_257;
  ORcp27_257 = -(OMcp27_112*RLcp27_357-OMcp27_312*RLcp27_157);
  ORcp27_357 = OMcp27_112*RLcp27_257-OMcp27_212*RLcp27_157;
  PxF4[1] = q[1]+RLcp27_110+RLcp27_111+RLcp27_157+RLcp27_17+RLcp27_18+RLcp27_19;
  PxF4[2] = q[2]+RLcp27_210+RLcp27_211+RLcp27_257+RLcp27_27+RLcp27_28+RLcp27_29;
  PxF4[3] = q[3]+RLcp27_310+RLcp27_311+RLcp27_357+RLcp27_37+RLcp27_38+RLcp27_39;
  RxF4[1][1] = ROcp27_112;
  RxF4[1][2] = ROcp27_212;
  RxF4[1][3] = ROcp27_312;
  RxF4[2][1] = ROcp27_411;
  RxF4[2][2] = ROcp27_511;
  RxF4[2][3] = ROcp27_611;
  RxF4[3][1] = ROcp27_712;
  RxF4[3][2] = ROcp27_812;
  RxF4[3][3] = ROcp27_912;
  VxF4[1] = qd[1]+ORcp27_110+ORcp27_111+ORcp27_157+ORcp27_17+ORcp27_18+ORcp27_19;
  VxF4[2] = qd[2]+ORcp27_210+ORcp27_211+ORcp27_257+ORcp27_27+ORcp27_28+ORcp27_29;
  VxF4[3] = qd[3]+ORcp27_310+ORcp27_311+ORcp27_357+ORcp27_37+ORcp27_38+ORcp27_39;
  OMxF4[1] = OMcp27_112;
  OMxF4[2] = OMcp27_212;
  OMxF4[3] = OMcp27_312;
  AxF4[1] = qdd[1]+OMcp27_210*ORcp27_311+OMcp27_212*ORcp27_357+OMcp27_26*ORcp27_37+OMcp27_27*ORcp27_38+OMcp27_28*
 ORcp27_39+OMcp27_29*ORcp27_310-OMcp27_310*ORcp27_211-OMcp27_312*ORcp27_257-OMcp27_36*ORcp27_27-OMcp27_37*ORcp27_28-OMcp27_38
 *ORcp27_29-OMcp27_39*ORcp27_210+OPcp27_210*RLcp27_311+OPcp27_212*RLcp27_357+OPcp27_26*RLcp27_37+OPcp27_27*RLcp27_38+
 OPcp27_28*RLcp27_39+OPcp27_29*RLcp27_310-OPcp27_310*RLcp27_211-OPcp27_312*RLcp27_257-OPcp27_36*RLcp27_27-OPcp27_37*RLcp27_28
 -OPcp27_38*RLcp27_29-OPcp27_39*RLcp27_210;
  AxF4[2] = qdd[2]-OMcp27_110*ORcp27_311-OMcp27_112*ORcp27_357-OMcp27_16*ORcp27_37-OMcp27_17*ORcp27_38-OMcp27_18*
 ORcp27_39-OMcp27_19*ORcp27_310+OMcp27_310*ORcp27_111+OMcp27_312*ORcp27_157+OMcp27_36*ORcp27_17+OMcp27_37*ORcp27_18+OMcp27_38
 *ORcp27_19+OMcp27_39*ORcp27_110-OPcp27_110*RLcp27_311-OPcp27_112*RLcp27_357-OPcp27_16*RLcp27_37-OPcp27_17*RLcp27_38-
 OPcp27_18*RLcp27_39-OPcp27_19*RLcp27_310+OPcp27_310*RLcp27_111+OPcp27_312*RLcp27_157+OPcp27_36*RLcp27_17+OPcp27_37*RLcp27_18
 +OPcp27_38*RLcp27_19+OPcp27_39*RLcp27_110;
  AxF4[3] = qdd[3]+OMcp27_110*ORcp27_211+OMcp27_112*ORcp27_257+OMcp27_16*ORcp27_27+OMcp27_17*ORcp27_28+OMcp27_18*
 ORcp27_29+OMcp27_19*ORcp27_210-OMcp27_210*ORcp27_111-OMcp27_212*ORcp27_157-OMcp27_26*ORcp27_17-OMcp27_27*ORcp27_18-OMcp27_28
 *ORcp27_19-OMcp27_29*ORcp27_110+OPcp27_110*RLcp27_211+OPcp27_112*RLcp27_257+OPcp27_16*RLcp27_27+OPcp27_17*RLcp27_28+
 OPcp27_18*RLcp27_29+OPcp27_19*RLcp27_210-OPcp27_210*RLcp27_111-OPcp27_212*RLcp27_157-OPcp27_26*RLcp27_17-OPcp27_27*RLcp27_18
 -OPcp27_28*RLcp27_19-OPcp27_29*RLcp27_110;
  OMPxF4[1] = OPcp27_112;
  OMPxF4[2] = OPcp27_212;
  OMPxF4[3] = OPcp27_312;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc128 = ROcp27_112*SWr4[1]+ROcp27_212*SWr4[2]+ROcp27_312*SWr4[3];
  xfrc228 = ROcp27_411*SWr4[1]+ROcp27_511*SWr4[2]+ROcp27_611*SWr4[3];
  xfrc328 = ROcp27_712*SWr4[1]+ROcp27_812*SWr4[2]+ROcp27_912*SWr4[3];
  frc[1][12] = s->frc[1][12]+xfrc128;
  frc[2][12] = s->frc[2][12]+xfrc228;
  frc[3][12] = s->frc[3][12]+xfrc328;
  xtrq128 = ROcp27_112*SWr4[4]+ROcp27_212*SWr4[5]+ROcp27_312*SWr4[6];
  xtrq228 = ROcp27_411*SWr4[4]+ROcp27_511*SWr4[5]+ROcp27_611*SWr4[6];
  xtrq328 = ROcp27_712*SWr4[4]+ROcp27_812*SWr4[5]+ROcp27_912*SWr4[6];
  trq[1][12] = s->trq[1][12]+xtrq128-xfrc228*(SWr4[9]-s->l[3][12])+xfrc328*(SWr4[8]-s->l[2][12]);
  trq[2][12] = s->trq[2][12]+xtrq228+xfrc128*(SWr4[9]-s->l[3][12])-xfrc328*(SWr4[7]-s->l[1][12]);
  trq[3][12] = s->trq[3][12]+xtrq328-xfrc128*(SWr4[8]-s->l[2][12])+xfrc228*(SWr4[7]-s->l[1][12]);

// = = Block_0_0_1_5_0_1 = = 
 
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

// = = Block_0_0_1_5_0_3 = = 
 
// Sensor Kinematics 


  ROcp28_113 = ROcp28_16*C13-S13*S5;
  ROcp28_213 = ROcp28_26*C13-ROcp28_85*S13;
  ROcp28_313 = ROcp28_36*C13-ROcp28_95*S13;
  ROcp28_713 = ROcp28_16*S13+C13*S5;
  ROcp28_813 = ROcp28_26*S13+ROcp28_85*C13;
  ROcp28_913 = ROcp28_36*S13+ROcp28_95*C13;
  ROcp28_414 = ROcp28_46*C14+ROcp28_713*S14;
  ROcp28_514 = ROcp28_56*C14+ROcp28_813*S14;
  ROcp28_614 = ROcp28_66*C14+ROcp28_913*S14;
  ROcp28_714 = -(ROcp28_46*S14-ROcp28_713*C14);
  ROcp28_814 = -(ROcp28_56*S14-ROcp28_813*C14);
  ROcp28_914 = -(ROcp28_66*S14-ROcp28_913*C14);
  ROcp28_115 = ROcp28_113*C15+ROcp28_414*S15;
  ROcp28_215 = ROcp28_213*C15+ROcp28_514*S15;
  ROcp28_315 = ROcp28_313*C15+ROcp28_614*S15;
  ROcp28_415 = -(ROcp28_113*S15-ROcp28_414*C15);
  ROcp28_515 = -(ROcp28_213*S15-ROcp28_514*C15);
  ROcp28_615 = -(ROcp28_313*S15-ROcp28_614*C15);
  RLcp28_113 = ROcp28_46*s->dpt[2][3];
  RLcp28_213 = ROcp28_56*s->dpt[2][3];
  RLcp28_313 = ROcp28_66*s->dpt[2][3];
  OMcp28_113 = OMcp28_16+qd[13]*ROcp28_46;
  OMcp28_213 = OMcp28_26+qd[13]*ROcp28_56;
  OMcp28_313 = OMcp28_36+qd[13]*ROcp28_66;
  ORcp28_113 = OMcp28_26*RLcp28_313-OMcp28_36*RLcp28_213;
  ORcp28_213 = -(OMcp28_16*RLcp28_313-OMcp28_36*RLcp28_113);
  ORcp28_313 = OMcp28_16*RLcp28_213-OMcp28_26*RLcp28_113;
  OPcp28_113 = OPcp28_16+qd[13]*(OMcp28_26*ROcp28_66-OMcp28_36*ROcp28_56)+qdd[13]*ROcp28_46;
  OPcp28_213 = OPcp28_26-qd[13]*(OMcp28_16*ROcp28_66-OMcp28_36*ROcp28_46)+qdd[13]*ROcp28_56;
  OPcp28_313 = OPcp28_36+qd[13]*(OMcp28_16*ROcp28_56-OMcp28_26*ROcp28_46)+qdd[13]*ROcp28_66;
  RLcp28_114 = ROcp28_46*s->dpt[2][20];
  RLcp28_214 = ROcp28_56*s->dpt[2][20];
  RLcp28_314 = ROcp28_66*s->dpt[2][20];
  OMcp28_114 = OMcp28_113+qd[14]*ROcp28_113;
  OMcp28_214 = OMcp28_213+qd[14]*ROcp28_213;
  OMcp28_314 = OMcp28_313+qd[14]*ROcp28_313;
  ORcp28_114 = OMcp28_213*RLcp28_314-OMcp28_313*RLcp28_214;
  ORcp28_214 = -(OMcp28_113*RLcp28_314-OMcp28_313*RLcp28_114);
  ORcp28_314 = OMcp28_113*RLcp28_214-OMcp28_213*RLcp28_114;
  OPcp28_114 = OPcp28_113+qd[14]*(OMcp28_213*ROcp28_313-OMcp28_313*ROcp28_213)+qdd[14]*ROcp28_113;
  OPcp28_214 = OPcp28_213-qd[14]*(OMcp28_113*ROcp28_313-OMcp28_313*ROcp28_113)+qdd[14]*ROcp28_213;
  OPcp28_314 = OPcp28_313+qd[14]*(OMcp28_113*ROcp28_213-OMcp28_213*ROcp28_113)+qdd[14]*ROcp28_313;
  RLcp28_115 = ROcp28_714*s->dpt[3][22];
  RLcp28_215 = ROcp28_814*s->dpt[3][22];
  RLcp28_315 = ROcp28_914*s->dpt[3][22];
  ORcp28_115 = OMcp28_214*RLcp28_315-OMcp28_314*RLcp28_215;
  ORcp28_215 = -(OMcp28_114*RLcp28_315-OMcp28_314*RLcp28_115);
  ORcp28_315 = OMcp28_114*RLcp28_215-OMcp28_214*RLcp28_115;
  PxF5[1] = q[1]+RLcp28_113+RLcp28_114+RLcp28_115;
  PxF5[2] = q[2]+RLcp28_213+RLcp28_214+RLcp28_215;
  PxF5[3] = q[3]+RLcp28_313+RLcp28_314+RLcp28_315;
  RxF5[1][1] = ROcp28_115;
  RxF5[1][2] = ROcp28_215;
  RxF5[1][3] = ROcp28_315;
  RxF5[2][1] = ROcp28_415;
  RxF5[2][2] = ROcp28_515;
  RxF5[2][3] = ROcp28_615;
  RxF5[3][1] = ROcp28_714;
  RxF5[3][2] = ROcp28_814;
  RxF5[3][3] = ROcp28_914;
  VxF5[1] = qd[1]+ORcp28_113+ORcp28_114+ORcp28_115;
  VxF5[2] = qd[2]+ORcp28_213+ORcp28_214+ORcp28_215;
  VxF5[3] = qd[3]+ORcp28_313+ORcp28_314+ORcp28_315;
  OMxF5[1] = OMcp28_114+qd[15]*ROcp28_714;
  OMxF5[2] = OMcp28_214+qd[15]*ROcp28_814;
  OMxF5[3] = OMcp28_314+qd[15]*ROcp28_914;
  AxF5[1] = qdd[1]+OMcp28_213*ORcp28_314+OMcp28_214*ORcp28_315+OMcp28_26*ORcp28_313-OMcp28_313*ORcp28_214-OMcp28_314*
 ORcp28_215-OMcp28_36*ORcp28_213+OPcp28_213*RLcp28_314+OPcp28_214*RLcp28_315+OPcp28_26*RLcp28_313-OPcp28_313*RLcp28_214-
 OPcp28_314*RLcp28_215-OPcp28_36*RLcp28_213;
  AxF5[2] = qdd[2]-OMcp28_113*ORcp28_314-OMcp28_114*ORcp28_315-OMcp28_16*ORcp28_313+OMcp28_313*ORcp28_114+OMcp28_314*
 ORcp28_115+OMcp28_36*ORcp28_113-OPcp28_113*RLcp28_314-OPcp28_114*RLcp28_315-OPcp28_16*RLcp28_313+OPcp28_313*RLcp28_114+
 OPcp28_314*RLcp28_115+OPcp28_36*RLcp28_113;
  AxF5[3] = qdd[3]+OMcp28_113*ORcp28_214+OMcp28_114*ORcp28_215+OMcp28_16*ORcp28_213-OMcp28_213*ORcp28_114-OMcp28_214*
 ORcp28_115-OMcp28_26*ORcp28_113+OPcp28_113*RLcp28_214+OPcp28_114*RLcp28_215+OPcp28_16*RLcp28_213-OPcp28_213*RLcp28_114-
 OPcp28_214*RLcp28_115-OPcp28_26*RLcp28_113;
  OMPxF5[1] = OPcp28_114+qd[15]*(OMcp28_214*ROcp28_914-OMcp28_314*ROcp28_814)+qdd[15]*ROcp28_714;
  OMPxF5[2] = OPcp28_214-qd[15]*(OMcp28_114*ROcp28_914-OMcp28_314*ROcp28_714)+qdd[15]*ROcp28_814;
  OMPxF5[3] = OPcp28_314+qd[15]*(OMcp28_114*ROcp28_814-OMcp28_214*ROcp28_714)+qdd[15]*ROcp28_914;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc129 = ROcp28_115*SWr5[1]+ROcp28_215*SWr5[2]+ROcp28_315*SWr5[3];
  xfrc229 = ROcp28_415*SWr5[1]+ROcp28_515*SWr5[2]+ROcp28_615*SWr5[3];
  xfrc329 = ROcp28_714*SWr5[1]+ROcp28_814*SWr5[2]+ROcp28_914*SWr5[3];
  frc[1][15] = s->frc[1][15]+xfrc129;
  frc[2][15] = s->frc[2][15]+xfrc229;
  frc[3][15] = s->frc[3][15]+xfrc329;
  xtrq129 = ROcp28_115*SWr5[4]+ROcp28_215*SWr5[5]+ROcp28_315*SWr5[6];
  xtrq229 = ROcp28_415*SWr5[4]+ROcp28_515*SWr5[5]+ROcp28_615*SWr5[6];
  xtrq329 = ROcp28_714*SWr5[4]+ROcp28_814*SWr5[5]+ROcp28_914*SWr5[6];
  trq[1][15] = s->trq[1][15]+xtrq129-xfrc229*(SWr5[9]-s->l[3][15])+xfrc329*(SWr5[8]-s->l[2][15]);
  trq[2][15] = s->trq[2][15]+xtrq229+xfrc129*(SWr5[9]-s->l[3][15])-xfrc329*(SWr5[7]-s->l[1][15]);
  trq[3][15] = s->trq[3][15]+xtrq329-xfrc129*(SWr5[8]-s->l[2][15])+xfrc229*(SWr5[7]-s->l[1][15]);

// = = Block_0_0_1_6_0_1 = = 
 
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

// = = Block_0_0_1_6_0_3 = = 
 
// Sensor Kinematics 


  ROcp29_113 = ROcp29_16*C13-S13*S5;
  ROcp29_213 = ROcp29_26*C13-ROcp29_85*S13;
  ROcp29_313 = ROcp29_36*C13-ROcp29_95*S13;
  ROcp29_713 = ROcp29_16*S13+C13*S5;
  ROcp29_813 = ROcp29_26*S13+ROcp29_85*C13;
  ROcp29_913 = ROcp29_36*S13+ROcp29_95*C13;
  ROcp29_414 = ROcp29_46*C14+ROcp29_713*S14;
  ROcp29_514 = ROcp29_56*C14+ROcp29_813*S14;
  ROcp29_614 = ROcp29_66*C14+ROcp29_913*S14;
  ROcp29_714 = -(ROcp29_46*S14-ROcp29_713*C14);
  ROcp29_814 = -(ROcp29_56*S14-ROcp29_813*C14);
  ROcp29_914 = -(ROcp29_66*S14-ROcp29_913*C14);
  ROcp29_115 = ROcp29_113*C15+ROcp29_414*S15;
  ROcp29_215 = ROcp29_213*C15+ROcp29_514*S15;
  ROcp29_315 = ROcp29_313*C15+ROcp29_614*S15;
  ROcp29_415 = -(ROcp29_113*S15-ROcp29_414*C15);
  ROcp29_515 = -(ROcp29_213*S15-ROcp29_514*C15);
  ROcp29_615 = -(ROcp29_313*S15-ROcp29_614*C15);
  ROcp29_116 = ROcp29_115*C16-ROcp29_714*S16;
  ROcp29_216 = ROcp29_215*C16-ROcp29_814*S16;
  ROcp29_316 = ROcp29_315*C16-ROcp29_914*S16;
  ROcp29_716 = ROcp29_115*S16+ROcp29_714*C16;
  ROcp29_816 = ROcp29_215*S16+ROcp29_814*C16;
  ROcp29_916 = ROcp29_315*S16+ROcp29_914*C16;
  RLcp29_113 = ROcp29_46*s->dpt[2][3];
  RLcp29_213 = ROcp29_56*s->dpt[2][3];
  RLcp29_313 = ROcp29_66*s->dpt[2][3];
  OMcp29_113 = OMcp29_16+qd[13]*ROcp29_46;
  OMcp29_213 = OMcp29_26+qd[13]*ROcp29_56;
  OMcp29_313 = OMcp29_36+qd[13]*ROcp29_66;
  ORcp29_113 = OMcp29_26*RLcp29_313-OMcp29_36*RLcp29_213;
  ORcp29_213 = -(OMcp29_16*RLcp29_313-OMcp29_36*RLcp29_113);
  ORcp29_313 = OMcp29_16*RLcp29_213-OMcp29_26*RLcp29_113;
  OPcp29_113 = OPcp29_16+qd[13]*(OMcp29_26*ROcp29_66-OMcp29_36*ROcp29_56)+qdd[13]*ROcp29_46;
  OPcp29_213 = OPcp29_26-qd[13]*(OMcp29_16*ROcp29_66-OMcp29_36*ROcp29_46)+qdd[13]*ROcp29_56;
  OPcp29_313 = OPcp29_36+qd[13]*(OMcp29_16*ROcp29_56-OMcp29_26*ROcp29_46)+qdd[13]*ROcp29_66;
  RLcp29_114 = ROcp29_46*s->dpt[2][20];
  RLcp29_214 = ROcp29_56*s->dpt[2][20];
  RLcp29_314 = ROcp29_66*s->dpt[2][20];
  OMcp29_114 = OMcp29_113+qd[14]*ROcp29_113;
  OMcp29_214 = OMcp29_213+qd[14]*ROcp29_213;
  OMcp29_314 = OMcp29_313+qd[14]*ROcp29_313;
  ORcp29_114 = OMcp29_213*RLcp29_314-OMcp29_313*RLcp29_214;
  ORcp29_214 = -(OMcp29_113*RLcp29_314-OMcp29_313*RLcp29_114);
  ORcp29_314 = OMcp29_113*RLcp29_214-OMcp29_213*RLcp29_114;
  OPcp29_114 = OPcp29_113+qd[14]*(OMcp29_213*ROcp29_313-OMcp29_313*ROcp29_213)+qdd[14]*ROcp29_113;
  OPcp29_214 = OPcp29_213-qd[14]*(OMcp29_113*ROcp29_313-OMcp29_313*ROcp29_113)+qdd[14]*ROcp29_213;
  OPcp29_314 = OPcp29_313+qd[14]*(OMcp29_113*ROcp29_213-OMcp29_213*ROcp29_113)+qdd[14]*ROcp29_313;
  RLcp29_115 = ROcp29_714*s->dpt[3][22];
  RLcp29_215 = ROcp29_814*s->dpt[3][22];
  RLcp29_315 = ROcp29_914*s->dpt[3][22];
  OMcp29_115 = OMcp29_114+qd[15]*ROcp29_714;
  OMcp29_215 = OMcp29_214+qd[15]*ROcp29_814;
  OMcp29_315 = OMcp29_314+qd[15]*ROcp29_914;
  ORcp29_115 = OMcp29_214*RLcp29_315-OMcp29_314*RLcp29_215;
  ORcp29_215 = -(OMcp29_114*RLcp29_315-OMcp29_314*RLcp29_115);
  ORcp29_315 = OMcp29_114*RLcp29_215-OMcp29_214*RLcp29_115;
  OPcp29_115 = OPcp29_114+qd[15]*(OMcp29_214*ROcp29_914-OMcp29_314*ROcp29_814)+qdd[15]*ROcp29_714;
  OPcp29_215 = OPcp29_214-qd[15]*(OMcp29_114*ROcp29_914-OMcp29_314*ROcp29_714)+qdd[15]*ROcp29_814;
  OPcp29_315 = OPcp29_314+qd[15]*(OMcp29_114*ROcp29_814-OMcp29_214*ROcp29_714)+qdd[15]*ROcp29_914;
  RLcp29_116 = ROcp29_714*s->dpt[3][25];
  RLcp29_216 = ROcp29_814*s->dpt[3][25];
  RLcp29_316 = ROcp29_914*s->dpt[3][25];
  ORcp29_116 = OMcp29_215*RLcp29_316-OMcp29_315*RLcp29_216;
  ORcp29_216 = -(OMcp29_115*RLcp29_316-OMcp29_315*RLcp29_116);
  ORcp29_316 = OMcp29_115*RLcp29_216-OMcp29_215*RLcp29_116;
  PxF6[1] = q[1]+RLcp29_113+RLcp29_114+RLcp29_115+RLcp29_116;
  PxF6[2] = q[2]+RLcp29_213+RLcp29_214+RLcp29_215+RLcp29_216;
  PxF6[3] = q[3]+RLcp29_313+RLcp29_314+RLcp29_315+RLcp29_316;
  RxF6[1][1] = ROcp29_116;
  RxF6[1][2] = ROcp29_216;
  RxF6[1][3] = ROcp29_316;
  RxF6[2][1] = ROcp29_415;
  RxF6[2][2] = ROcp29_515;
  RxF6[2][3] = ROcp29_615;
  RxF6[3][1] = ROcp29_716;
  RxF6[3][2] = ROcp29_816;
  RxF6[3][3] = ROcp29_916;
  VxF6[1] = qd[1]+ORcp29_113+ORcp29_114+ORcp29_115+ORcp29_116;
  VxF6[2] = qd[2]+ORcp29_213+ORcp29_214+ORcp29_215+ORcp29_216;
  VxF6[3] = qd[3]+ORcp29_313+ORcp29_314+ORcp29_315+ORcp29_316;
  OMxF6[1] = OMcp29_115+qd[16]*ROcp29_415;
  OMxF6[2] = OMcp29_215+qd[16]*ROcp29_515;
  OMxF6[3] = OMcp29_315+qd[16]*ROcp29_615;
  AxF6[1] = qdd[1]+OMcp29_213*ORcp29_314+OMcp29_214*ORcp29_315+OMcp29_215*ORcp29_316+OMcp29_26*ORcp29_313-OMcp29_313*
 ORcp29_214-OMcp29_314*ORcp29_215-OMcp29_315*ORcp29_216-OMcp29_36*ORcp29_213+OPcp29_213*RLcp29_314+OPcp29_214*RLcp29_315+
 OPcp29_215*RLcp29_316+OPcp29_26*RLcp29_313-OPcp29_313*RLcp29_214-OPcp29_314*RLcp29_215-OPcp29_315*RLcp29_216-OPcp29_36*
 RLcp29_213;
  AxF6[2] = qdd[2]-OMcp29_113*ORcp29_314-OMcp29_114*ORcp29_315-OMcp29_115*ORcp29_316-OMcp29_16*ORcp29_313+OMcp29_313*
 ORcp29_114+OMcp29_314*ORcp29_115+OMcp29_315*ORcp29_116+OMcp29_36*ORcp29_113-OPcp29_113*RLcp29_314-OPcp29_114*RLcp29_315-
 OPcp29_115*RLcp29_316-OPcp29_16*RLcp29_313+OPcp29_313*RLcp29_114+OPcp29_314*RLcp29_115+OPcp29_315*RLcp29_116+OPcp29_36*
 RLcp29_113;
  AxF6[3] = qdd[3]+OMcp29_113*ORcp29_214+OMcp29_114*ORcp29_215+OMcp29_115*ORcp29_216+OMcp29_16*ORcp29_213-OMcp29_213*
 ORcp29_114-OMcp29_214*ORcp29_115-OMcp29_215*ORcp29_116-OMcp29_26*ORcp29_113+OPcp29_113*RLcp29_214+OPcp29_114*RLcp29_215+
 OPcp29_115*RLcp29_216+OPcp29_16*RLcp29_213-OPcp29_213*RLcp29_114-OPcp29_214*RLcp29_115-OPcp29_215*RLcp29_116-OPcp29_26*
 RLcp29_113;
  OMPxF6[1] = OPcp29_115+qd[16]*(OMcp29_215*ROcp29_615-OMcp29_315*ROcp29_515)+qdd[16]*ROcp29_415;
  OMPxF6[2] = OPcp29_215-qd[16]*(OMcp29_115*ROcp29_615-OMcp29_315*ROcp29_415)+qdd[16]*ROcp29_515;
  OMPxF6[3] = OPcp29_315+qd[16]*(OMcp29_115*ROcp29_515-OMcp29_215*ROcp29_415)+qdd[16]*ROcp29_615;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc130 = ROcp29_116*SWr6[1]+ROcp29_216*SWr6[2]+ROcp29_316*SWr6[3];
  xfrc230 = ROcp29_415*SWr6[1]+ROcp29_515*SWr6[2]+ROcp29_615*SWr6[3];
  xfrc330 = ROcp29_716*SWr6[1]+ROcp29_816*SWr6[2]+ROcp29_916*SWr6[3];
  frc[1][16] = s->frc[1][16]+xfrc130;
  frc[2][16] = s->frc[2][16]+xfrc230;
  frc[3][16] = s->frc[3][16]+xfrc330;
  xtrq130 = ROcp29_116*SWr6[4]+ROcp29_216*SWr6[5]+ROcp29_316*SWr6[6];
  xtrq230 = ROcp29_415*SWr6[4]+ROcp29_515*SWr6[5]+ROcp29_615*SWr6[6];
  xtrq330 = ROcp29_716*SWr6[4]+ROcp29_816*SWr6[5]+ROcp29_916*SWr6[6];
  trq[1][16] = s->trq[1][16]+xtrq130-xfrc230*(SWr6[9]-s->l[3][16])+xfrc330*(SWr6[8]-s->l[2][16]);
  trq[2][16] = s->trq[2][16]+xtrq230+xfrc130*(SWr6[9]-s->l[3][16])-xfrc330*(SWr6[7]-s->l[1][16]);
  trq[3][16] = s->trq[3][16]+xtrq330-xfrc130*(SWr6[8]-s->l[2][16])+xfrc230*(SWr6[7]-s->l[1][16]);

// = = Block_0_0_1_7_0_1 = = 
 
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

// = = Block_0_0_1_7_0_3 = = 
 
// Sensor Kinematics 


  ROcp30_113 = ROcp30_16*C13-S13*S5;
  ROcp30_213 = ROcp30_26*C13-ROcp30_85*S13;
  ROcp30_313 = ROcp30_36*C13-ROcp30_95*S13;
  ROcp30_713 = ROcp30_16*S13+C13*S5;
  ROcp30_813 = ROcp30_26*S13+ROcp30_85*C13;
  ROcp30_913 = ROcp30_36*S13+ROcp30_95*C13;
  ROcp30_414 = ROcp30_46*C14+ROcp30_713*S14;
  ROcp30_514 = ROcp30_56*C14+ROcp30_813*S14;
  ROcp30_614 = ROcp30_66*C14+ROcp30_913*S14;
  ROcp30_714 = -(ROcp30_46*S14-ROcp30_713*C14);
  ROcp30_814 = -(ROcp30_56*S14-ROcp30_813*C14);
  ROcp30_914 = -(ROcp30_66*S14-ROcp30_913*C14);
  ROcp30_115 = ROcp30_113*C15+ROcp30_414*S15;
  ROcp30_215 = ROcp30_213*C15+ROcp30_514*S15;
  ROcp30_315 = ROcp30_313*C15+ROcp30_614*S15;
  ROcp30_415 = -(ROcp30_113*S15-ROcp30_414*C15);
  ROcp30_515 = -(ROcp30_213*S15-ROcp30_514*C15);
  ROcp30_615 = -(ROcp30_313*S15-ROcp30_614*C15);
  ROcp30_116 = ROcp30_115*C16-ROcp30_714*S16;
  ROcp30_216 = ROcp30_215*C16-ROcp30_814*S16;
  ROcp30_316 = ROcp30_315*C16-ROcp30_914*S16;
  ROcp30_716 = ROcp30_115*S16+ROcp30_714*C16;
  ROcp30_816 = ROcp30_215*S16+ROcp30_814*C16;
  ROcp30_916 = ROcp30_315*S16+ROcp30_914*C16;
  ROcp30_417 = ROcp30_415*C17+ROcp30_716*S17;
  ROcp30_517 = ROcp30_515*C17+ROcp30_816*S17;
  ROcp30_617 = ROcp30_615*C17+ROcp30_916*S17;
  ROcp30_717 = -(ROcp30_415*S17-ROcp30_716*C17);
  ROcp30_817 = -(ROcp30_515*S17-ROcp30_816*C17);
  ROcp30_917 = -(ROcp30_615*S17-ROcp30_916*C17);
  ROcp30_118 = ROcp30_116*C18-ROcp30_717*S18;
  ROcp30_218 = ROcp30_216*C18-ROcp30_817*S18;
  ROcp30_318 = ROcp30_316*C18-ROcp30_917*S18;
  ROcp30_718 = ROcp30_116*S18+ROcp30_717*C18;
  ROcp30_818 = ROcp30_216*S18+ROcp30_817*C18;
  ROcp30_918 = ROcp30_316*S18+ROcp30_917*C18;
  RLcp30_113 = ROcp30_46*s->dpt[2][3];
  RLcp30_213 = ROcp30_56*s->dpt[2][3];
  RLcp30_313 = ROcp30_66*s->dpt[2][3];
  OMcp30_113 = OMcp30_16+qd[13]*ROcp30_46;
  OMcp30_213 = OMcp30_26+qd[13]*ROcp30_56;
  OMcp30_313 = OMcp30_36+qd[13]*ROcp30_66;
  ORcp30_113 = OMcp30_26*RLcp30_313-OMcp30_36*RLcp30_213;
  ORcp30_213 = -(OMcp30_16*RLcp30_313-OMcp30_36*RLcp30_113);
  ORcp30_313 = OMcp30_16*RLcp30_213-OMcp30_26*RLcp30_113;
  OPcp30_113 = OPcp30_16+qd[13]*(OMcp30_26*ROcp30_66-OMcp30_36*ROcp30_56)+qdd[13]*ROcp30_46;
  OPcp30_213 = OPcp30_26-qd[13]*(OMcp30_16*ROcp30_66-OMcp30_36*ROcp30_46)+qdd[13]*ROcp30_56;
  OPcp30_313 = OPcp30_36+qd[13]*(OMcp30_16*ROcp30_56-OMcp30_26*ROcp30_46)+qdd[13]*ROcp30_66;
  RLcp30_114 = ROcp30_46*s->dpt[2][20];
  RLcp30_214 = ROcp30_56*s->dpt[2][20];
  RLcp30_314 = ROcp30_66*s->dpt[2][20];
  OMcp30_114 = OMcp30_113+qd[14]*ROcp30_113;
  OMcp30_214 = OMcp30_213+qd[14]*ROcp30_213;
  OMcp30_314 = OMcp30_313+qd[14]*ROcp30_313;
  ORcp30_114 = OMcp30_213*RLcp30_314-OMcp30_313*RLcp30_214;
  ORcp30_214 = -(OMcp30_113*RLcp30_314-OMcp30_313*RLcp30_114);
  ORcp30_314 = OMcp30_113*RLcp30_214-OMcp30_213*RLcp30_114;
  OPcp30_114 = OPcp30_113+qd[14]*(OMcp30_213*ROcp30_313-OMcp30_313*ROcp30_213)+qdd[14]*ROcp30_113;
  OPcp30_214 = OPcp30_213-qd[14]*(OMcp30_113*ROcp30_313-OMcp30_313*ROcp30_113)+qdd[14]*ROcp30_213;
  OPcp30_314 = OPcp30_313+qd[14]*(OMcp30_113*ROcp30_213-OMcp30_213*ROcp30_113)+qdd[14]*ROcp30_313;
  RLcp30_115 = ROcp30_714*s->dpt[3][22];
  RLcp30_215 = ROcp30_814*s->dpt[3][22];
  RLcp30_315 = ROcp30_914*s->dpt[3][22];
  OMcp30_115 = OMcp30_114+qd[15]*ROcp30_714;
  OMcp30_215 = OMcp30_214+qd[15]*ROcp30_814;
  OMcp30_315 = OMcp30_314+qd[15]*ROcp30_914;
  ORcp30_115 = OMcp30_214*RLcp30_315-OMcp30_314*RLcp30_215;
  ORcp30_215 = -(OMcp30_114*RLcp30_315-OMcp30_314*RLcp30_115);
  ORcp30_315 = OMcp30_114*RLcp30_215-OMcp30_214*RLcp30_115;
  OPcp30_115 = OPcp30_114+qd[15]*(OMcp30_214*ROcp30_914-OMcp30_314*ROcp30_814)+qdd[15]*ROcp30_714;
  OPcp30_215 = OPcp30_214-qd[15]*(OMcp30_114*ROcp30_914-OMcp30_314*ROcp30_714)+qdd[15]*ROcp30_814;
  OPcp30_315 = OPcp30_314+qd[15]*(OMcp30_114*ROcp30_814-OMcp30_214*ROcp30_714)+qdd[15]*ROcp30_914;
  RLcp30_116 = ROcp30_714*s->dpt[3][25];
  RLcp30_216 = ROcp30_814*s->dpt[3][25];
  RLcp30_316 = ROcp30_914*s->dpt[3][25];
  OMcp30_116 = OMcp30_115+qd[16]*ROcp30_415;
  OMcp30_216 = OMcp30_215+qd[16]*ROcp30_515;
  OMcp30_316 = OMcp30_315+qd[16]*ROcp30_615;
  ORcp30_116 = OMcp30_215*RLcp30_316-OMcp30_315*RLcp30_216;
  ORcp30_216 = -(OMcp30_115*RLcp30_316-OMcp30_315*RLcp30_116);
  ORcp30_316 = OMcp30_115*RLcp30_216-OMcp30_215*RLcp30_116;
  OPcp30_116 = OPcp30_115+qd[16]*(OMcp30_215*ROcp30_615-OMcp30_315*ROcp30_515)+qdd[16]*ROcp30_415;
  OPcp30_216 = OPcp30_215-qd[16]*(OMcp30_115*ROcp30_615-OMcp30_315*ROcp30_415)+qdd[16]*ROcp30_515;
  OPcp30_316 = OPcp30_315+qd[16]*(OMcp30_115*ROcp30_515-OMcp30_215*ROcp30_415)+qdd[16]*ROcp30_615;
  RLcp30_117 = ROcp30_716*s->dpt[3][28];
  RLcp30_217 = ROcp30_816*s->dpt[3][28];
  RLcp30_317 = ROcp30_916*s->dpt[3][28];
  OMcp30_117 = OMcp30_116+qd[17]*ROcp30_116;
  OMcp30_217 = OMcp30_216+qd[17]*ROcp30_216;
  OMcp30_317 = OMcp30_316+qd[17]*ROcp30_316;
  ORcp30_117 = OMcp30_216*RLcp30_317-OMcp30_316*RLcp30_217;
  ORcp30_217 = -(OMcp30_116*RLcp30_317-OMcp30_316*RLcp30_117);
  ORcp30_317 = OMcp30_116*RLcp30_217-OMcp30_216*RLcp30_117;
  OMcp30_118 = OMcp30_117+qd[18]*ROcp30_417;
  OMcp30_218 = OMcp30_217+qd[18]*ROcp30_517;
  OMcp30_318 = OMcp30_317+qd[18]*ROcp30_617;
  OPcp30_118 = OPcp30_116+qd[17]*(OMcp30_216*ROcp30_316-OMcp30_316*ROcp30_216)+qd[18]*(OMcp30_217*ROcp30_617-OMcp30_317*
 ROcp30_517)+qdd[17]*ROcp30_116+qdd[18]*ROcp30_417;
  OPcp30_218 = OPcp30_216-qd[17]*(OMcp30_116*ROcp30_316-OMcp30_316*ROcp30_116)-qd[18]*(OMcp30_117*ROcp30_617-OMcp30_317*
 ROcp30_417)+qdd[17]*ROcp30_216+qdd[18]*ROcp30_517;
  OPcp30_318 = OPcp30_316+qd[17]*(OMcp30_116*ROcp30_216-OMcp30_216*ROcp30_116)+qd[18]*(OMcp30_117*ROcp30_517-OMcp30_217*
 ROcp30_417)+qdd[17]*ROcp30_316+qdd[18]*ROcp30_617;
  RLcp30_160 = ROcp30_718*s->dpt[3][33];
  RLcp30_260 = ROcp30_818*s->dpt[3][33];
  RLcp30_360 = ROcp30_918*s->dpt[3][33];
  ORcp30_160 = OMcp30_218*RLcp30_360-OMcp30_318*RLcp30_260;
  ORcp30_260 = -(OMcp30_118*RLcp30_360-OMcp30_318*RLcp30_160);
  ORcp30_360 = OMcp30_118*RLcp30_260-OMcp30_218*RLcp30_160;
  PxF7[1] = q[1]+RLcp30_113+RLcp30_114+RLcp30_115+RLcp30_116+RLcp30_117+RLcp30_160;
  PxF7[2] = q[2]+RLcp30_213+RLcp30_214+RLcp30_215+RLcp30_216+RLcp30_217+RLcp30_260;
  PxF7[3] = q[3]+RLcp30_313+RLcp30_314+RLcp30_315+RLcp30_316+RLcp30_317+RLcp30_360;
  RxF7[1][1] = ROcp30_118;
  RxF7[1][2] = ROcp30_218;
  RxF7[1][3] = ROcp30_318;
  RxF7[2][1] = ROcp30_417;
  RxF7[2][2] = ROcp30_517;
  RxF7[2][3] = ROcp30_617;
  RxF7[3][1] = ROcp30_718;
  RxF7[3][2] = ROcp30_818;
  RxF7[3][3] = ROcp30_918;
  VxF7[1] = qd[1]+ORcp30_113+ORcp30_114+ORcp30_115+ORcp30_116+ORcp30_117+ORcp30_160;
  VxF7[2] = qd[2]+ORcp30_213+ORcp30_214+ORcp30_215+ORcp30_216+ORcp30_217+ORcp30_260;
  VxF7[3] = qd[3]+ORcp30_313+ORcp30_314+ORcp30_315+ORcp30_316+ORcp30_317+ORcp30_360;
  OMxF7[1] = OMcp30_118;
  OMxF7[2] = OMcp30_218;
  OMxF7[3] = OMcp30_318;
  AxF7[1] = qdd[1]+OMcp30_213*ORcp30_314+OMcp30_214*ORcp30_315+OMcp30_215*ORcp30_316+OMcp30_216*ORcp30_317+OMcp30_218*
 ORcp30_360+OMcp30_26*ORcp30_313-OMcp30_313*ORcp30_214-OMcp30_314*ORcp30_215-OMcp30_315*ORcp30_216-OMcp30_316*ORcp30_217-
 OMcp30_318*ORcp30_260-OMcp30_36*ORcp30_213+OPcp30_213*RLcp30_314+OPcp30_214*RLcp30_315+OPcp30_215*RLcp30_316+OPcp30_216*
 RLcp30_317+OPcp30_218*RLcp30_360+OPcp30_26*RLcp30_313-OPcp30_313*RLcp30_214-OPcp30_314*RLcp30_215-OPcp30_315*RLcp30_216-
 OPcp30_316*RLcp30_217-OPcp30_318*RLcp30_260-OPcp30_36*RLcp30_213;
  AxF7[2] = qdd[2]-OMcp30_113*ORcp30_314-OMcp30_114*ORcp30_315-OMcp30_115*ORcp30_316-OMcp30_116*ORcp30_317-OMcp30_118*
 ORcp30_360-OMcp30_16*ORcp30_313+OMcp30_313*ORcp30_114+OMcp30_314*ORcp30_115+OMcp30_315*ORcp30_116+OMcp30_316*ORcp30_117+
 OMcp30_318*ORcp30_160+OMcp30_36*ORcp30_113-OPcp30_113*RLcp30_314-OPcp30_114*RLcp30_315-OPcp30_115*RLcp30_316-OPcp30_116*
 RLcp30_317-OPcp30_118*RLcp30_360-OPcp30_16*RLcp30_313+OPcp30_313*RLcp30_114+OPcp30_314*RLcp30_115+OPcp30_315*RLcp30_116+
 OPcp30_316*RLcp30_117+OPcp30_318*RLcp30_160+OPcp30_36*RLcp30_113;
  AxF7[3] = qdd[3]+OMcp30_113*ORcp30_214+OMcp30_114*ORcp30_215+OMcp30_115*ORcp30_216+OMcp30_116*ORcp30_217+OMcp30_118*
 ORcp30_260+OMcp30_16*ORcp30_213-OMcp30_213*ORcp30_114-OMcp30_214*ORcp30_115-OMcp30_215*ORcp30_116-OMcp30_216*ORcp30_117-
 OMcp30_218*ORcp30_160-OMcp30_26*ORcp30_113+OPcp30_113*RLcp30_214+OPcp30_114*RLcp30_215+OPcp30_115*RLcp30_216+OPcp30_116*
 RLcp30_217+OPcp30_118*RLcp30_260+OPcp30_16*RLcp30_213-OPcp30_213*RLcp30_114-OPcp30_214*RLcp30_115-OPcp30_215*RLcp30_116-
 OPcp30_216*RLcp30_117-OPcp30_218*RLcp30_160-OPcp30_26*RLcp30_113;
  OMPxF7[1] = OPcp30_118;
  OMPxF7[2] = OPcp30_218;
  OMPxF7[3] = OPcp30_318;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc131 = ROcp30_118*SWr7[1]+ROcp30_218*SWr7[2]+ROcp30_318*SWr7[3];
  xfrc231 = ROcp30_417*SWr7[1]+ROcp30_517*SWr7[2]+ROcp30_617*SWr7[3];
  xfrc331 = ROcp30_718*SWr7[1]+ROcp30_818*SWr7[2]+ROcp30_918*SWr7[3];
  frc[1][18] = s->frc[1][18]+xfrc131;
  frc[2][18] = s->frc[2][18]+xfrc231;
  frc[3][18] = s->frc[3][18]+xfrc331;
  xtrq131 = ROcp30_118*SWr7[4]+ROcp30_218*SWr7[5]+ROcp30_318*SWr7[6];
  xtrq231 = ROcp30_417*SWr7[4]+ROcp30_517*SWr7[5]+ROcp30_617*SWr7[6];
  xtrq331 = ROcp30_718*SWr7[4]+ROcp30_818*SWr7[5]+ROcp30_918*SWr7[6];
  trq[1][18] = s->trq[1][18]+xtrq131-xfrc231*(SWr7[9]-s->l[3][18])+xfrc331*(SWr7[8]-s->l[2][18]);
  trq[2][18] = s->trq[2][18]+xtrq231+xfrc131*(SWr7[9]-s->l[3][18])-xfrc331*(SWr7[7]-s->l[1][18]);
  trq[3][18] = s->trq[3][18]+xtrq331-xfrc131*(SWr7[8]-s->l[2][18])+xfrc231*(SWr7[7]-s->l[1][18]);

// = = Block_0_0_1_8_0_1 = = 
 
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

// = = Block_0_0_1_8_0_4 = = 
 
// Sensor Kinematics 


  ROcp31_419 = ROcp31_46*C19+S19*S5;
  ROcp31_519 = ROcp31_56*C19+ROcp31_85*S19;
  ROcp31_619 = ROcp31_66*C19+ROcp31_95*S19;
  ROcp31_719 = -(ROcp31_46*S19-C19*S5);
  ROcp31_819 = -(ROcp31_56*S19-ROcp31_85*C19);
  ROcp31_919 = -(ROcp31_66*S19-ROcp31_95*C19);
  ROcp31_120 = ROcp31_16*C20-ROcp31_719*S20;
  ROcp31_220 = ROcp31_26*C20-ROcp31_819*S20;
  ROcp31_320 = ROcp31_36*C20-ROcp31_919*S20;
  ROcp31_720 = ROcp31_16*S20+ROcp31_719*C20;
  ROcp31_820 = ROcp31_26*S20+ROcp31_819*C20;
  ROcp31_920 = ROcp31_36*S20+ROcp31_919*C20;
  ROcp31_121 = ROcp31_120*C21+ROcp31_419*S21;
  ROcp31_221 = ROcp31_220*C21+ROcp31_519*S21;
  ROcp31_321 = ROcp31_320*C21+ROcp31_619*S21;
  ROcp31_421 = -(ROcp31_120*S21-ROcp31_419*C21);
  ROcp31_521 = -(ROcp31_220*S21-ROcp31_519*C21);
  ROcp31_621 = -(ROcp31_320*S21-ROcp31_619*C21);
  RLcp31_119 = ROcp31_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp31_219 = ROcp31_26*s->dpt[1][4]+ROcp31_85*s->dpt[3][4];
  RLcp31_319 = ROcp31_36*s->dpt[1][4]+ROcp31_95*s->dpt[3][4];
  OMcp31_119 = OMcp31_16+qd[19]*ROcp31_16;
  OMcp31_219 = OMcp31_26+qd[19]*ROcp31_26;
  OMcp31_319 = OMcp31_36+qd[19]*ROcp31_36;
  ORcp31_119 = OMcp31_26*RLcp31_319-OMcp31_36*RLcp31_219;
  ORcp31_219 = -(OMcp31_16*RLcp31_319-OMcp31_36*RLcp31_119);
  ORcp31_319 = OMcp31_16*RLcp31_219-OMcp31_26*RLcp31_119;
  OMcp31_120 = OMcp31_119+qd[20]*ROcp31_419;
  OMcp31_220 = OMcp31_219+qd[20]*ROcp31_519;
  OMcp31_320 = OMcp31_319+qd[20]*ROcp31_619;
  OPcp31_120 = OPcp31_16+qd[19]*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26)+qd[20]*(OMcp31_219*ROcp31_619-OMcp31_319*
 ROcp31_519)+qdd[19]*ROcp31_16+qdd[20]*ROcp31_419;
  OPcp31_220 = OPcp31_26-qd[19]*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16)-qd[20]*(OMcp31_119*ROcp31_619-OMcp31_319*
 ROcp31_419)+qdd[19]*ROcp31_26+qdd[20]*ROcp31_519;
  OPcp31_320 = OPcp31_36+qd[19]*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16)+qd[20]*(OMcp31_119*ROcp31_519-OMcp31_219*
 ROcp31_419)+qdd[19]*ROcp31_36+qdd[20]*ROcp31_619;
  RLcp31_121 = ROcp31_720*s->dpt[3][36];
  RLcp31_221 = ROcp31_820*s->dpt[3][36];
  RLcp31_321 = ROcp31_920*s->dpt[3][36];
  ORcp31_121 = OMcp31_220*RLcp31_321-OMcp31_320*RLcp31_221;
  ORcp31_221 = -(OMcp31_120*RLcp31_321-OMcp31_320*RLcp31_121);
  ORcp31_321 = OMcp31_120*RLcp31_221-OMcp31_220*RLcp31_121;
  PxF8[1] = q[1]+RLcp31_119+RLcp31_121;
  PxF8[2] = q[2]+RLcp31_219+RLcp31_221;
  PxF8[3] = q[3]+RLcp31_319+RLcp31_321;
  RxF8[1][1] = ROcp31_121;
  RxF8[1][2] = ROcp31_221;
  RxF8[1][3] = ROcp31_321;
  RxF8[2][1] = ROcp31_421;
  RxF8[2][2] = ROcp31_521;
  RxF8[2][3] = ROcp31_621;
  RxF8[3][1] = ROcp31_720;
  RxF8[3][2] = ROcp31_820;
  RxF8[3][3] = ROcp31_920;
  VxF8[1] = qd[1]+ORcp31_119+ORcp31_121;
  VxF8[2] = qd[2]+ORcp31_219+ORcp31_221;
  VxF8[3] = qd[3]+ORcp31_319+ORcp31_321;
  OMxF8[1] = OMcp31_120+qd[21]*ROcp31_720;
  OMxF8[2] = OMcp31_220+qd[21]*ROcp31_820;
  OMxF8[3] = OMcp31_320+qd[21]*ROcp31_920;
  AxF8[1] = qdd[1]+OMcp31_220*ORcp31_321+OMcp31_26*ORcp31_319-OMcp31_320*ORcp31_221-OMcp31_36*ORcp31_219+OPcp31_220*
 RLcp31_321+OPcp31_26*RLcp31_319-OPcp31_320*RLcp31_221-OPcp31_36*RLcp31_219;
  AxF8[2] = qdd[2]-OMcp31_120*ORcp31_321-OMcp31_16*ORcp31_319+OMcp31_320*ORcp31_121+OMcp31_36*ORcp31_119-OPcp31_120*
 RLcp31_321-OPcp31_16*RLcp31_319+OPcp31_320*RLcp31_121+OPcp31_36*RLcp31_119;
  AxF8[3] = qdd[3]+OMcp31_120*ORcp31_221+OMcp31_16*ORcp31_219-OMcp31_220*ORcp31_121-OMcp31_26*ORcp31_119+OPcp31_120*
 RLcp31_221+OPcp31_16*RLcp31_219-OPcp31_220*RLcp31_121-OPcp31_26*RLcp31_119;
  OMPxF8[1] = OPcp31_120+qd[21]*(OMcp31_220*ROcp31_920-OMcp31_320*ROcp31_820)+qdd[21]*ROcp31_720;
  OMPxF8[2] = OPcp31_220-qd[21]*(OMcp31_120*ROcp31_920-OMcp31_320*ROcp31_720)+qdd[21]*ROcp31_820;
  OMPxF8[3] = OPcp31_320+qd[21]*(OMcp31_120*ROcp31_820-OMcp31_220*ROcp31_720)+qdd[21]*ROcp31_920;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc132 = ROcp31_121*SWr8[1]+ROcp31_221*SWr8[2]+ROcp31_321*SWr8[3];
  xfrc232 = ROcp31_421*SWr8[1]+ROcp31_521*SWr8[2]+ROcp31_621*SWr8[3];
  xfrc332 = ROcp31_720*SWr8[1]+ROcp31_820*SWr8[2]+ROcp31_920*SWr8[3];
  frc[1][21] = s->frc[1][21]+xfrc132;
  frc[2][21] = s->frc[2][21]+xfrc232;
  frc[3][21] = s->frc[3][21]+xfrc332;
  xtrq132 = ROcp31_121*SWr8[4]+ROcp31_221*SWr8[5]+ROcp31_321*SWr8[6];
  xtrq232 = ROcp31_421*SWr8[4]+ROcp31_521*SWr8[5]+ROcp31_621*SWr8[6];
  xtrq332 = ROcp31_720*SWr8[4]+ROcp31_820*SWr8[5]+ROcp31_920*SWr8[6];
  trq[1][21] = s->trq[1][21]+xtrq132-xfrc232*(SWr8[9]-s->l[3][21])+xfrc332*(SWr8[8]-s->l[2][21]);
  trq[2][21] = s->trq[2][21]+xtrq232+xfrc132*(SWr8[9]-s->l[3][21])-xfrc332*(SWr8[7]-s->l[1][21]);
  trq[3][21] = s->trq[3][21]+xtrq332-xfrc132*(SWr8[8]-s->l[2][21])+xfrc232*(SWr8[7]-s->l[1][21]);

// = = Block_0_0_1_9_0_1 = = 
 
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

// = = Block_0_0_1_9_0_4 = = 
 
// Sensor Kinematics 


  ROcp32_419 = ROcp32_46*C19+S19*S5;
  ROcp32_519 = ROcp32_56*C19+ROcp32_85*S19;
  ROcp32_619 = ROcp32_66*C19+ROcp32_95*S19;
  ROcp32_719 = -(ROcp32_46*S19-C19*S5);
  ROcp32_819 = -(ROcp32_56*S19-ROcp32_85*C19);
  ROcp32_919 = -(ROcp32_66*S19-ROcp32_95*C19);
  ROcp32_120 = ROcp32_16*C20-ROcp32_719*S20;
  ROcp32_220 = ROcp32_26*C20-ROcp32_819*S20;
  ROcp32_320 = ROcp32_36*C20-ROcp32_919*S20;
  ROcp32_720 = ROcp32_16*S20+ROcp32_719*C20;
  ROcp32_820 = ROcp32_26*S20+ROcp32_819*C20;
  ROcp32_920 = ROcp32_36*S20+ROcp32_919*C20;
  ROcp32_121 = ROcp32_120*C21+ROcp32_419*S21;
  ROcp32_221 = ROcp32_220*C21+ROcp32_519*S21;
  ROcp32_321 = ROcp32_320*C21+ROcp32_619*S21;
  ROcp32_421 = -(ROcp32_120*S21-ROcp32_419*C21);
  ROcp32_521 = -(ROcp32_220*S21-ROcp32_519*C21);
  ROcp32_621 = -(ROcp32_320*S21-ROcp32_619*C21);
  RLcp32_119 = ROcp32_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp32_219 = ROcp32_26*s->dpt[1][4]+ROcp32_85*s->dpt[3][4];
  RLcp32_319 = ROcp32_36*s->dpt[1][4]+ROcp32_95*s->dpt[3][4];
  OMcp32_119 = OMcp32_16+qd[19]*ROcp32_16;
  OMcp32_219 = OMcp32_26+qd[19]*ROcp32_26;
  OMcp32_319 = OMcp32_36+qd[19]*ROcp32_36;
  ORcp32_119 = OMcp32_26*RLcp32_319-OMcp32_36*RLcp32_219;
  ORcp32_219 = -(OMcp32_16*RLcp32_319-OMcp32_36*RLcp32_119);
  ORcp32_319 = OMcp32_16*RLcp32_219-OMcp32_26*RLcp32_119;
  OMcp32_120 = OMcp32_119+qd[20]*ROcp32_419;
  OMcp32_220 = OMcp32_219+qd[20]*ROcp32_519;
  OMcp32_320 = OMcp32_319+qd[20]*ROcp32_619;
  OPcp32_120 = OPcp32_16+qd[19]*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26)+qd[20]*(OMcp32_219*ROcp32_619-OMcp32_319*
 ROcp32_519)+qdd[19]*ROcp32_16+qdd[20]*ROcp32_419;
  OPcp32_220 = OPcp32_26-qd[19]*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16)-qd[20]*(OMcp32_119*ROcp32_619-OMcp32_319*
 ROcp32_419)+qdd[19]*ROcp32_26+qdd[20]*ROcp32_519;
  OPcp32_320 = OPcp32_36+qd[19]*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16)+qd[20]*(OMcp32_119*ROcp32_519-OMcp32_219*
 ROcp32_419)+qdd[19]*ROcp32_36+qdd[20]*ROcp32_619;
  RLcp32_121 = ROcp32_720*s->dpt[3][36];
  RLcp32_221 = ROcp32_820*s->dpt[3][36];
  RLcp32_321 = ROcp32_920*s->dpt[3][36];
  OMcp32_121 = OMcp32_120+qd[21]*ROcp32_720;
  OMcp32_221 = OMcp32_220+qd[21]*ROcp32_820;
  OMcp32_321 = OMcp32_320+qd[21]*ROcp32_920;
  ORcp32_121 = OMcp32_220*RLcp32_321-OMcp32_320*RLcp32_221;
  ORcp32_221 = -(OMcp32_120*RLcp32_321-OMcp32_320*RLcp32_121);
  ORcp32_321 = OMcp32_120*RLcp32_221-OMcp32_220*RLcp32_121;
  OPcp32_121 = OPcp32_120+qd[21]*(OMcp32_220*ROcp32_920-OMcp32_320*ROcp32_820)+qdd[21]*ROcp32_720;
  OPcp32_221 = OPcp32_220-qd[21]*(OMcp32_120*ROcp32_920-OMcp32_320*ROcp32_720)+qdd[21]*ROcp32_820;
  OPcp32_321 = OPcp32_320+qd[21]*(OMcp32_120*ROcp32_820-OMcp32_220*ROcp32_720)+qdd[21]*ROcp32_920;

// = = Block_0_0_1_9_0_5 = = 
 
// Sensor Kinematics 


  ROcp32_122 = ROcp32_121*C22-ROcp32_720*S22;
  ROcp32_222 = ROcp32_221*C22-ROcp32_820*S22;
  ROcp32_322 = ROcp32_321*C22-ROcp32_920*S22;
  ROcp32_722 = ROcp32_121*S22+ROcp32_720*C22;
  ROcp32_822 = ROcp32_221*S22+ROcp32_820*C22;
  ROcp32_922 = ROcp32_321*S22+ROcp32_920*C22;
  ROcp32_423 = ROcp32_421*C23+ROcp32_722*S23;
  ROcp32_523 = ROcp32_521*C23+ROcp32_822*S23;
  ROcp32_623 = ROcp32_621*C23+ROcp32_922*S23;
  ROcp32_723 = -(ROcp32_421*S23-ROcp32_722*C23);
  ROcp32_823 = -(ROcp32_521*S23-ROcp32_822*C23);
  ROcp32_923 = -(ROcp32_621*S23-ROcp32_922*C23);
  ROcp32_124 = ROcp32_122*C24+ROcp32_423*S24;
  ROcp32_224 = ROcp32_222*C24+ROcp32_523*S24;
  ROcp32_324 = ROcp32_322*C24+ROcp32_623*S24;
  ROcp32_424 = -(ROcp32_122*S24-ROcp32_423*C24);
  ROcp32_524 = -(ROcp32_222*S24-ROcp32_523*C24);
  ROcp32_624 = -(ROcp32_322*S24-ROcp32_623*C24);
  RLcp32_122 = ROcp32_121*s->dpt[1][40]+ROcp32_421*s->dpt[2][40]+ROcp32_720*s->dpt[3][40];
  RLcp32_222 = ROcp32_221*s->dpt[1][40]+ROcp32_521*s->dpt[2][40]+ROcp32_820*s->dpt[3][40];
  RLcp32_322 = ROcp32_321*s->dpt[1][40]+ROcp32_621*s->dpt[2][40]+ROcp32_920*s->dpt[3][40];
  OMcp32_122 = OMcp32_121+qd[22]*ROcp32_421;
  OMcp32_222 = OMcp32_221+qd[22]*ROcp32_521;
  OMcp32_322 = OMcp32_321+qd[22]*ROcp32_621;
  ORcp32_122 = OMcp32_221*RLcp32_322-OMcp32_321*RLcp32_222;
  ORcp32_222 = -(OMcp32_121*RLcp32_322-OMcp32_321*RLcp32_122);
  ORcp32_322 = OMcp32_121*RLcp32_222-OMcp32_221*RLcp32_122;
  OPcp32_122 = OPcp32_121+qd[22]*(OMcp32_221*ROcp32_621-OMcp32_321*ROcp32_521)+qdd[22]*ROcp32_421;
  OPcp32_222 = OPcp32_221-qd[22]*(OMcp32_121*ROcp32_621-OMcp32_321*ROcp32_421)+qdd[22]*ROcp32_521;
  OPcp32_322 = OPcp32_321+qd[22]*(OMcp32_121*ROcp32_521-OMcp32_221*ROcp32_421)+qdd[22]*ROcp32_621;
  RLcp32_123 = ROcp32_421*s->dpt[2][43];
  RLcp32_223 = ROcp32_521*s->dpt[2][43];
  RLcp32_323 = ROcp32_621*s->dpt[2][43];
  OMcp32_123 = OMcp32_122+qd[23]*ROcp32_122;
  OMcp32_223 = OMcp32_222+qd[23]*ROcp32_222;
  OMcp32_323 = OMcp32_322+qd[23]*ROcp32_322;
  ORcp32_123 = OMcp32_222*RLcp32_323-OMcp32_322*RLcp32_223;
  ORcp32_223 = -(OMcp32_122*RLcp32_323-OMcp32_322*RLcp32_123);
  ORcp32_323 = OMcp32_122*RLcp32_223-OMcp32_222*RLcp32_123;
  OPcp32_123 = OPcp32_122+qd[23]*(OMcp32_222*ROcp32_322-OMcp32_322*ROcp32_222)+qdd[23]*ROcp32_122;
  OPcp32_223 = OPcp32_222-qd[23]*(OMcp32_122*ROcp32_322-OMcp32_322*ROcp32_122)+qdd[23]*ROcp32_222;
  OPcp32_323 = OPcp32_322+qd[23]*(OMcp32_122*ROcp32_222-OMcp32_222*ROcp32_122)+qdd[23]*ROcp32_322;
  RLcp32_124 = ROcp32_723*s->dpt[3][45];
  RLcp32_224 = ROcp32_823*s->dpt[3][45];
  RLcp32_324 = ROcp32_923*s->dpt[3][45];
  ORcp32_124 = OMcp32_223*RLcp32_324-OMcp32_323*RLcp32_224;
  ORcp32_224 = -(OMcp32_123*RLcp32_324-OMcp32_323*RLcp32_124);
  ORcp32_324 = OMcp32_123*RLcp32_224-OMcp32_223*RLcp32_124;
  PxF9[1] = q[1]+RLcp32_119+RLcp32_121+RLcp32_122+RLcp32_123+RLcp32_124;
  PxF9[2] = q[2]+RLcp32_219+RLcp32_221+RLcp32_222+RLcp32_223+RLcp32_224;
  PxF9[3] = q[3]+RLcp32_319+RLcp32_321+RLcp32_322+RLcp32_323+RLcp32_324;
  RxF9[1][1] = ROcp32_124;
  RxF9[1][2] = ROcp32_224;
  RxF9[1][3] = ROcp32_324;
  RxF9[2][1] = ROcp32_424;
  RxF9[2][2] = ROcp32_524;
  RxF9[2][3] = ROcp32_624;
  RxF9[3][1] = ROcp32_723;
  RxF9[3][2] = ROcp32_823;
  RxF9[3][3] = ROcp32_923;
  VxF9[1] = qd[1]+ORcp32_119+ORcp32_121+ORcp32_122+ORcp32_123+ORcp32_124;
  VxF9[2] = qd[2]+ORcp32_219+ORcp32_221+ORcp32_222+ORcp32_223+ORcp32_224;
  VxF9[3] = qd[3]+ORcp32_319+ORcp32_321+ORcp32_322+ORcp32_323+ORcp32_324;
  OMxF9[1] = OMcp32_123+qd[24]*ROcp32_723;
  OMxF9[2] = OMcp32_223+qd[24]*ROcp32_823;
  OMxF9[3] = OMcp32_323+qd[24]*ROcp32_923;
  AxF9[1] = qdd[1]+OMcp32_220*ORcp32_321+OMcp32_221*ORcp32_322+OMcp32_222*ORcp32_323+OMcp32_223*ORcp32_324+OMcp32_26*
 ORcp32_319-OMcp32_320*ORcp32_221-OMcp32_321*ORcp32_222-OMcp32_322*ORcp32_223-OMcp32_323*ORcp32_224-OMcp32_36*ORcp32_219+
 OPcp32_220*RLcp32_321+OPcp32_221*RLcp32_322+OPcp32_222*RLcp32_323+OPcp32_223*RLcp32_324+OPcp32_26*RLcp32_319-OPcp32_320*
 RLcp32_221-OPcp32_321*RLcp32_222-OPcp32_322*RLcp32_223-OPcp32_323*RLcp32_224-OPcp32_36*RLcp32_219;
  AxF9[2] = qdd[2]-OMcp32_120*ORcp32_321-OMcp32_121*ORcp32_322-OMcp32_122*ORcp32_323-OMcp32_123*ORcp32_324-OMcp32_16*
 ORcp32_319+OMcp32_320*ORcp32_121+OMcp32_321*ORcp32_122+OMcp32_322*ORcp32_123+OMcp32_323*ORcp32_124+OMcp32_36*ORcp32_119-
 OPcp32_120*RLcp32_321-OPcp32_121*RLcp32_322-OPcp32_122*RLcp32_323-OPcp32_123*RLcp32_324-OPcp32_16*RLcp32_319+OPcp32_320*
 RLcp32_121+OPcp32_321*RLcp32_122+OPcp32_322*RLcp32_123+OPcp32_323*RLcp32_124+OPcp32_36*RLcp32_119;
  AxF9[3] = qdd[3]+OMcp32_120*ORcp32_221+OMcp32_121*ORcp32_222+OMcp32_122*ORcp32_223+OMcp32_123*ORcp32_224+OMcp32_16*
 ORcp32_219-OMcp32_220*ORcp32_121-OMcp32_221*ORcp32_122-OMcp32_222*ORcp32_123-OMcp32_223*ORcp32_124-OMcp32_26*ORcp32_119+
 OPcp32_120*RLcp32_221+OPcp32_121*RLcp32_222+OPcp32_122*RLcp32_223+OPcp32_123*RLcp32_224+OPcp32_16*RLcp32_219-OPcp32_220*
 RLcp32_121-OPcp32_221*RLcp32_122-OPcp32_222*RLcp32_123-OPcp32_223*RLcp32_124-OPcp32_26*RLcp32_119;
  OMPxF9[1] = OPcp32_123+qd[24]*(OMcp32_223*ROcp32_923-OMcp32_323*ROcp32_823)+qdd[24]*ROcp32_723;
  OMPxF9[2] = OPcp32_223-qd[24]*(OMcp32_123*ROcp32_923-OMcp32_323*ROcp32_723)+qdd[24]*ROcp32_823;
  OMPxF9[3] = OPcp32_323+qd[24]*(OMcp32_123*ROcp32_823-OMcp32_223*ROcp32_723)+qdd[24]*ROcp32_923;
 
// Sensor Forces Computation 

  SWr9 = user_ExtForces(PxF9,RxF9,VxF9,OMxF9,AxF9,OMPxF9,s,tsim,9);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc133 = ROcp32_124*SWr9[1]+ROcp32_224*SWr9[2]+ROcp32_324*SWr9[3];
  xfrc233 = ROcp32_424*SWr9[1]+ROcp32_524*SWr9[2]+ROcp32_624*SWr9[3];
  xfrc333 = ROcp32_723*SWr9[1]+ROcp32_823*SWr9[2]+ROcp32_923*SWr9[3];
  frc[1][24] = s->frc[1][24]+xfrc133;
  frc[2][24] = s->frc[2][24]+xfrc233;
  frc[3][24] = s->frc[3][24]+xfrc333;
  xtrq133 = ROcp32_124*SWr9[4]+ROcp32_224*SWr9[5]+ROcp32_324*SWr9[6];
  xtrq233 = ROcp32_424*SWr9[4]+ROcp32_524*SWr9[5]+ROcp32_624*SWr9[6];
  xtrq333 = ROcp32_723*SWr9[4]+ROcp32_823*SWr9[5]+ROcp32_923*SWr9[6];
  trq[1][24] = s->trq[1][24]+xtrq133-xfrc233*(SWr9[9]-s->l[3][24])+xfrc333*(SWr9[8]-s->l[2][24]);
  trq[2][24] = s->trq[2][24]+xtrq233+xfrc133*(SWr9[9]-s->l[3][24])-xfrc333*(SWr9[7]-s->l[1][24]);
  trq[3][24] = s->trq[3][24]+xtrq333-xfrc133*(SWr9[8]-s->l[2][24])+xfrc233*(SWr9[7]-s->l[1][24]);

// = = Block_0_0_1_10_0_1 = = 
 
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

// = = Block_0_0_1_10_0_4 = = 
 
// Sensor Kinematics 


  ROcp33_419 = ROcp33_46*C19+S19*S5;
  ROcp33_519 = ROcp33_56*C19+ROcp33_85*S19;
  ROcp33_619 = ROcp33_66*C19+ROcp33_95*S19;
  ROcp33_719 = -(ROcp33_46*S19-C19*S5);
  ROcp33_819 = -(ROcp33_56*S19-ROcp33_85*C19);
  ROcp33_919 = -(ROcp33_66*S19-ROcp33_95*C19);
  ROcp33_120 = ROcp33_16*C20-ROcp33_719*S20;
  ROcp33_220 = ROcp33_26*C20-ROcp33_819*S20;
  ROcp33_320 = ROcp33_36*C20-ROcp33_919*S20;
  ROcp33_720 = ROcp33_16*S20+ROcp33_719*C20;
  ROcp33_820 = ROcp33_26*S20+ROcp33_819*C20;
  ROcp33_920 = ROcp33_36*S20+ROcp33_919*C20;
  ROcp33_121 = ROcp33_120*C21+ROcp33_419*S21;
  ROcp33_221 = ROcp33_220*C21+ROcp33_519*S21;
  ROcp33_321 = ROcp33_320*C21+ROcp33_619*S21;
  ROcp33_421 = -(ROcp33_120*S21-ROcp33_419*C21);
  ROcp33_521 = -(ROcp33_220*S21-ROcp33_519*C21);
  ROcp33_621 = -(ROcp33_320*S21-ROcp33_619*C21);
  RLcp33_119 = ROcp33_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp33_219 = ROcp33_26*s->dpt[1][4]+ROcp33_85*s->dpt[3][4];
  RLcp33_319 = ROcp33_36*s->dpt[1][4]+ROcp33_95*s->dpt[3][4];
  OMcp33_119 = OMcp33_16+qd[19]*ROcp33_16;
  OMcp33_219 = OMcp33_26+qd[19]*ROcp33_26;
  OMcp33_319 = OMcp33_36+qd[19]*ROcp33_36;
  ORcp33_119 = OMcp33_26*RLcp33_319-OMcp33_36*RLcp33_219;
  ORcp33_219 = -(OMcp33_16*RLcp33_319-OMcp33_36*RLcp33_119);
  ORcp33_319 = OMcp33_16*RLcp33_219-OMcp33_26*RLcp33_119;
  OMcp33_120 = OMcp33_119+qd[20]*ROcp33_419;
  OMcp33_220 = OMcp33_219+qd[20]*ROcp33_519;
  OMcp33_320 = OMcp33_319+qd[20]*ROcp33_619;
  OPcp33_120 = OPcp33_16+qd[19]*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26)+qd[20]*(OMcp33_219*ROcp33_619-OMcp33_319*
 ROcp33_519)+qdd[19]*ROcp33_16+qdd[20]*ROcp33_419;
  OPcp33_220 = OPcp33_26-qd[19]*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16)-qd[20]*(OMcp33_119*ROcp33_619-OMcp33_319*
 ROcp33_419)+qdd[19]*ROcp33_26+qdd[20]*ROcp33_519;
  OPcp33_320 = OPcp33_36+qd[19]*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16)+qd[20]*(OMcp33_119*ROcp33_519-OMcp33_219*
 ROcp33_419)+qdd[19]*ROcp33_36+qdd[20]*ROcp33_619;
  RLcp33_121 = ROcp33_720*s->dpt[3][36];
  RLcp33_221 = ROcp33_820*s->dpt[3][36];
  RLcp33_321 = ROcp33_920*s->dpt[3][36];
  OMcp33_121 = OMcp33_120+qd[21]*ROcp33_720;
  OMcp33_221 = OMcp33_220+qd[21]*ROcp33_820;
  OMcp33_321 = OMcp33_320+qd[21]*ROcp33_920;
  ORcp33_121 = OMcp33_220*RLcp33_321-OMcp33_320*RLcp33_221;
  ORcp33_221 = -(OMcp33_120*RLcp33_321-OMcp33_320*RLcp33_121);
  ORcp33_321 = OMcp33_120*RLcp33_221-OMcp33_220*RLcp33_121;
  OPcp33_121 = OPcp33_120+qd[21]*(OMcp33_220*ROcp33_920-OMcp33_320*ROcp33_820)+qdd[21]*ROcp33_720;
  OPcp33_221 = OPcp33_220-qd[21]*(OMcp33_120*ROcp33_920-OMcp33_320*ROcp33_720)+qdd[21]*ROcp33_820;
  OPcp33_321 = OPcp33_320+qd[21]*(OMcp33_120*ROcp33_820-OMcp33_220*ROcp33_720)+qdd[21]*ROcp33_920;

// = = Block_0_0_1_10_0_5 = = 
 
// Sensor Kinematics 


  ROcp33_122 = ROcp33_121*C22-ROcp33_720*S22;
  ROcp33_222 = ROcp33_221*C22-ROcp33_820*S22;
  ROcp33_322 = ROcp33_321*C22-ROcp33_920*S22;
  ROcp33_722 = ROcp33_121*S22+ROcp33_720*C22;
  ROcp33_822 = ROcp33_221*S22+ROcp33_820*C22;
  ROcp33_922 = ROcp33_321*S22+ROcp33_920*C22;
  ROcp33_423 = ROcp33_421*C23+ROcp33_722*S23;
  ROcp33_523 = ROcp33_521*C23+ROcp33_822*S23;
  ROcp33_623 = ROcp33_621*C23+ROcp33_922*S23;
  ROcp33_723 = -(ROcp33_421*S23-ROcp33_722*C23);
  ROcp33_823 = -(ROcp33_521*S23-ROcp33_822*C23);
  ROcp33_923 = -(ROcp33_621*S23-ROcp33_922*C23);
  ROcp33_124 = ROcp33_122*C24+ROcp33_423*S24;
  ROcp33_224 = ROcp33_222*C24+ROcp33_523*S24;
  ROcp33_324 = ROcp33_322*C24+ROcp33_623*S24;
  ROcp33_424 = -(ROcp33_122*S24-ROcp33_423*C24);
  ROcp33_524 = -(ROcp33_222*S24-ROcp33_523*C24);
  ROcp33_624 = -(ROcp33_322*S24-ROcp33_623*C24);
  ROcp33_125 = ROcp33_124*C25-ROcp33_723*S25;
  ROcp33_225 = ROcp33_224*C25-ROcp33_823*S25;
  ROcp33_325 = ROcp33_324*C25-ROcp33_923*S25;
  ROcp33_725 = ROcp33_124*S25+ROcp33_723*C25;
  ROcp33_825 = ROcp33_224*S25+ROcp33_823*C25;
  ROcp33_925 = ROcp33_324*S25+ROcp33_923*C25;
  RLcp33_122 = ROcp33_121*s->dpt[1][40]+ROcp33_421*s->dpt[2][40]+ROcp33_720*s->dpt[3][40];
  RLcp33_222 = ROcp33_221*s->dpt[1][40]+ROcp33_521*s->dpt[2][40]+ROcp33_820*s->dpt[3][40];
  RLcp33_322 = ROcp33_321*s->dpt[1][40]+ROcp33_621*s->dpt[2][40]+ROcp33_920*s->dpt[3][40];
  OMcp33_122 = OMcp33_121+qd[22]*ROcp33_421;
  OMcp33_222 = OMcp33_221+qd[22]*ROcp33_521;
  OMcp33_322 = OMcp33_321+qd[22]*ROcp33_621;
  ORcp33_122 = OMcp33_221*RLcp33_322-OMcp33_321*RLcp33_222;
  ORcp33_222 = -(OMcp33_121*RLcp33_322-OMcp33_321*RLcp33_122);
  ORcp33_322 = OMcp33_121*RLcp33_222-OMcp33_221*RLcp33_122;
  OPcp33_122 = OPcp33_121+qd[22]*(OMcp33_221*ROcp33_621-OMcp33_321*ROcp33_521)+qdd[22]*ROcp33_421;
  OPcp33_222 = OPcp33_221-qd[22]*(OMcp33_121*ROcp33_621-OMcp33_321*ROcp33_421)+qdd[22]*ROcp33_521;
  OPcp33_322 = OPcp33_321+qd[22]*(OMcp33_121*ROcp33_521-OMcp33_221*ROcp33_421)+qdd[22]*ROcp33_621;
  RLcp33_123 = ROcp33_421*s->dpt[2][43];
  RLcp33_223 = ROcp33_521*s->dpt[2][43];
  RLcp33_323 = ROcp33_621*s->dpt[2][43];
  OMcp33_123 = OMcp33_122+qd[23]*ROcp33_122;
  OMcp33_223 = OMcp33_222+qd[23]*ROcp33_222;
  OMcp33_323 = OMcp33_322+qd[23]*ROcp33_322;
  ORcp33_123 = OMcp33_222*RLcp33_323-OMcp33_322*RLcp33_223;
  ORcp33_223 = -(OMcp33_122*RLcp33_323-OMcp33_322*RLcp33_123);
  ORcp33_323 = OMcp33_122*RLcp33_223-OMcp33_222*RLcp33_123;
  OPcp33_123 = OPcp33_122+qd[23]*(OMcp33_222*ROcp33_322-OMcp33_322*ROcp33_222)+qdd[23]*ROcp33_122;
  OPcp33_223 = OPcp33_222-qd[23]*(OMcp33_122*ROcp33_322-OMcp33_322*ROcp33_122)+qdd[23]*ROcp33_222;
  OPcp33_323 = OPcp33_322+qd[23]*(OMcp33_122*ROcp33_222-OMcp33_222*ROcp33_122)+qdd[23]*ROcp33_322;
  RLcp33_124 = ROcp33_723*s->dpt[3][45];
  RLcp33_224 = ROcp33_823*s->dpt[3][45];
  RLcp33_324 = ROcp33_923*s->dpt[3][45];
  OMcp33_124 = OMcp33_123+qd[24]*ROcp33_723;
  OMcp33_224 = OMcp33_223+qd[24]*ROcp33_823;
  OMcp33_324 = OMcp33_323+qd[24]*ROcp33_923;
  ORcp33_124 = OMcp33_223*RLcp33_324-OMcp33_323*RLcp33_224;
  ORcp33_224 = -(OMcp33_123*RLcp33_324-OMcp33_323*RLcp33_124);
  ORcp33_324 = OMcp33_123*RLcp33_224-OMcp33_223*RLcp33_124;
  OPcp33_124 = OPcp33_123+qd[24]*(OMcp33_223*ROcp33_923-OMcp33_323*ROcp33_823)+qdd[24]*ROcp33_723;
  OPcp33_224 = OPcp33_223-qd[24]*(OMcp33_123*ROcp33_923-OMcp33_323*ROcp33_723)+qdd[24]*ROcp33_823;
  OPcp33_324 = OPcp33_323+qd[24]*(OMcp33_123*ROcp33_823-OMcp33_223*ROcp33_723)+qdd[24]*ROcp33_923;
  RLcp33_125 = ROcp33_723*s->dpt[3][48];
  RLcp33_225 = ROcp33_823*s->dpt[3][48];
  RLcp33_325 = ROcp33_923*s->dpt[3][48];
  ORcp33_125 = OMcp33_224*RLcp33_325-OMcp33_324*RLcp33_225;
  ORcp33_225 = -(OMcp33_124*RLcp33_325-OMcp33_324*RLcp33_125);
  ORcp33_325 = OMcp33_124*RLcp33_225-OMcp33_224*RLcp33_125;
  PxF10[1] = q[1]+RLcp33_119+RLcp33_121+RLcp33_122+RLcp33_123+RLcp33_124+RLcp33_125;
  PxF10[2] = q[2]+RLcp33_219+RLcp33_221+RLcp33_222+RLcp33_223+RLcp33_224+RLcp33_225;
  PxF10[3] = q[3]+RLcp33_319+RLcp33_321+RLcp33_322+RLcp33_323+RLcp33_324+RLcp33_325;
  RxF10[1][1] = ROcp33_125;
  RxF10[1][2] = ROcp33_225;
  RxF10[1][3] = ROcp33_325;
  RxF10[2][1] = ROcp33_424;
  RxF10[2][2] = ROcp33_524;
  RxF10[2][3] = ROcp33_624;
  RxF10[3][1] = ROcp33_725;
  RxF10[3][2] = ROcp33_825;
  RxF10[3][3] = ROcp33_925;
  VxF10[1] = qd[1]+ORcp33_119+ORcp33_121+ORcp33_122+ORcp33_123+ORcp33_124+ORcp33_125;
  VxF10[2] = qd[2]+ORcp33_219+ORcp33_221+ORcp33_222+ORcp33_223+ORcp33_224+ORcp33_225;
  VxF10[3] = qd[3]+ORcp33_319+ORcp33_321+ORcp33_322+ORcp33_323+ORcp33_324+ORcp33_325;
  OMxF10[1] = OMcp33_124+qd[25]*ROcp33_424;
  OMxF10[2] = OMcp33_224+qd[25]*ROcp33_524;
  OMxF10[3] = OMcp33_324+qd[25]*ROcp33_624;
  AxF10[1] = qdd[1]+OMcp33_220*ORcp33_321+OMcp33_221*ORcp33_322+OMcp33_222*ORcp33_323+OMcp33_223*ORcp33_324+OMcp33_224*
 ORcp33_325+OMcp33_26*ORcp33_319-OMcp33_320*ORcp33_221-OMcp33_321*ORcp33_222-OMcp33_322*ORcp33_223-OMcp33_323*ORcp33_224-
 OMcp33_324*ORcp33_225-OMcp33_36*ORcp33_219+OPcp33_220*RLcp33_321+OPcp33_221*RLcp33_322+OPcp33_222*RLcp33_323+OPcp33_223*
 RLcp33_324+OPcp33_224*RLcp33_325+OPcp33_26*RLcp33_319-OPcp33_320*RLcp33_221-OPcp33_321*RLcp33_222-OPcp33_322*RLcp33_223-
 OPcp33_323*RLcp33_224-OPcp33_324*RLcp33_225-OPcp33_36*RLcp33_219;
  AxF10[2] = qdd[2]-OMcp33_120*ORcp33_321-OMcp33_121*ORcp33_322-OMcp33_122*ORcp33_323-OMcp33_123*ORcp33_324-OMcp33_124*
 ORcp33_325-OMcp33_16*ORcp33_319+OMcp33_320*ORcp33_121+OMcp33_321*ORcp33_122+OMcp33_322*ORcp33_123+OMcp33_323*ORcp33_124+
 OMcp33_324*ORcp33_125+OMcp33_36*ORcp33_119-OPcp33_120*RLcp33_321-OPcp33_121*RLcp33_322-OPcp33_122*RLcp33_323-OPcp33_123*
 RLcp33_324-OPcp33_124*RLcp33_325-OPcp33_16*RLcp33_319+OPcp33_320*RLcp33_121+OPcp33_321*RLcp33_122+OPcp33_322*RLcp33_123+
 OPcp33_323*RLcp33_124+OPcp33_324*RLcp33_125+OPcp33_36*RLcp33_119;
  AxF10[3] = qdd[3]+OMcp33_120*ORcp33_221+OMcp33_121*ORcp33_222+OMcp33_122*ORcp33_223+OMcp33_123*ORcp33_224+OMcp33_124*
 ORcp33_225+OMcp33_16*ORcp33_219-OMcp33_220*ORcp33_121-OMcp33_221*ORcp33_122-OMcp33_222*ORcp33_123-OMcp33_223*ORcp33_124-
 OMcp33_224*ORcp33_125-OMcp33_26*ORcp33_119+OPcp33_120*RLcp33_221+OPcp33_121*RLcp33_222+OPcp33_122*RLcp33_223+OPcp33_123*
 RLcp33_224+OPcp33_124*RLcp33_225+OPcp33_16*RLcp33_219-OPcp33_220*RLcp33_121-OPcp33_221*RLcp33_122-OPcp33_222*RLcp33_123-
 OPcp33_223*RLcp33_124-OPcp33_224*RLcp33_125-OPcp33_26*RLcp33_119;
  OMPxF10[1] = OPcp33_124+qd[25]*(OMcp33_224*ROcp33_624-OMcp33_324*ROcp33_524)+qdd[25]*ROcp33_424;
  OMPxF10[2] = OPcp33_224-qd[25]*(OMcp33_124*ROcp33_624-OMcp33_324*ROcp33_424)+qdd[25]*ROcp33_524;
  OMPxF10[3] = OPcp33_324+qd[25]*(OMcp33_124*ROcp33_524-OMcp33_224*ROcp33_424)+qdd[25]*ROcp33_624;
 
// Sensor Forces Computation 

  SWr10 = user_ExtForces(PxF10,RxF10,VxF10,OMxF10,AxF10,OMPxF10,s,tsim,10);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc134 = ROcp33_125*SWr10[1]+ROcp33_225*SWr10[2]+ROcp33_325*SWr10[3];
  xfrc234 = ROcp33_424*SWr10[1]+ROcp33_524*SWr10[2]+ROcp33_624*SWr10[3];
  xfrc334 = ROcp33_725*SWr10[1]+ROcp33_825*SWr10[2]+ROcp33_925*SWr10[3];
  frc[1][25] = s->frc[1][25]+xfrc134;
  frc[2][25] = s->frc[2][25]+xfrc234;
  frc[3][25] = s->frc[3][25]+xfrc334;
  xtrq134 = ROcp33_125*SWr10[4]+ROcp33_225*SWr10[5]+ROcp33_325*SWr10[6];
  xtrq234 = ROcp33_424*SWr10[4]+ROcp33_524*SWr10[5]+ROcp33_624*SWr10[6];
  xtrq334 = ROcp33_725*SWr10[4]+ROcp33_825*SWr10[5]+ROcp33_925*SWr10[6];
  trq[1][25] = s->trq[1][25]+xtrq134-xfrc234*(SWr10[9]-s->l[3][25])+xfrc334*(SWr10[8]-s->l[2][25]);
  trq[2][25] = s->trq[2][25]+xtrq234+xfrc134*(SWr10[9]-s->l[3][25])-xfrc334*(SWr10[7]-s->l[1][25]);
  trq[3][25] = s->trq[3][25]+xtrq334-xfrc134*(SWr10[8]-s->l[2][25])+xfrc234*(SWr10[7]-s->l[1][25]);

// = = Block_0_0_1_11_0_1 = = 
 
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

// = = Block_0_0_1_11_0_4 = = 
 
// Sensor Kinematics 


  ROcp34_419 = ROcp34_46*C19+S19*S5;
  ROcp34_519 = ROcp34_56*C19+ROcp34_85*S19;
  ROcp34_619 = ROcp34_66*C19+ROcp34_95*S19;
  ROcp34_719 = -(ROcp34_46*S19-C19*S5);
  ROcp34_819 = -(ROcp34_56*S19-ROcp34_85*C19);
  ROcp34_919 = -(ROcp34_66*S19-ROcp34_95*C19);
  ROcp34_120 = ROcp34_16*C20-ROcp34_719*S20;
  ROcp34_220 = ROcp34_26*C20-ROcp34_819*S20;
  ROcp34_320 = ROcp34_36*C20-ROcp34_919*S20;
  ROcp34_720 = ROcp34_16*S20+ROcp34_719*C20;
  ROcp34_820 = ROcp34_26*S20+ROcp34_819*C20;
  ROcp34_920 = ROcp34_36*S20+ROcp34_919*C20;
  ROcp34_121 = ROcp34_120*C21+ROcp34_419*S21;
  ROcp34_221 = ROcp34_220*C21+ROcp34_519*S21;
  ROcp34_321 = ROcp34_320*C21+ROcp34_619*S21;
  ROcp34_421 = -(ROcp34_120*S21-ROcp34_419*C21);
  ROcp34_521 = -(ROcp34_220*S21-ROcp34_519*C21);
  ROcp34_621 = -(ROcp34_320*S21-ROcp34_619*C21);
  RLcp34_119 = ROcp34_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp34_219 = ROcp34_26*s->dpt[1][4]+ROcp34_85*s->dpt[3][4];
  RLcp34_319 = ROcp34_36*s->dpt[1][4]+ROcp34_95*s->dpt[3][4];
  OMcp34_119 = OMcp34_16+qd[19]*ROcp34_16;
  OMcp34_219 = OMcp34_26+qd[19]*ROcp34_26;
  OMcp34_319 = OMcp34_36+qd[19]*ROcp34_36;
  ORcp34_119 = OMcp34_26*RLcp34_319-OMcp34_36*RLcp34_219;
  ORcp34_219 = -(OMcp34_16*RLcp34_319-OMcp34_36*RLcp34_119);
  ORcp34_319 = OMcp34_16*RLcp34_219-OMcp34_26*RLcp34_119;
  OMcp34_120 = OMcp34_119+qd[20]*ROcp34_419;
  OMcp34_220 = OMcp34_219+qd[20]*ROcp34_519;
  OMcp34_320 = OMcp34_319+qd[20]*ROcp34_619;
  OPcp34_120 = OPcp34_16+qd[19]*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26)+qd[20]*(OMcp34_219*ROcp34_619-OMcp34_319*
 ROcp34_519)+qdd[19]*ROcp34_16+qdd[20]*ROcp34_419;
  OPcp34_220 = OPcp34_26-qd[19]*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16)-qd[20]*(OMcp34_119*ROcp34_619-OMcp34_319*
 ROcp34_419)+qdd[19]*ROcp34_26+qdd[20]*ROcp34_519;
  OPcp34_320 = OPcp34_36+qd[19]*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16)+qd[20]*(OMcp34_119*ROcp34_519-OMcp34_219*
 ROcp34_419)+qdd[19]*ROcp34_36+qdd[20]*ROcp34_619;
  RLcp34_121 = ROcp34_720*s->dpt[3][36];
  RLcp34_221 = ROcp34_820*s->dpt[3][36];
  RLcp34_321 = ROcp34_920*s->dpt[3][36];
  OMcp34_121 = OMcp34_120+qd[21]*ROcp34_720;
  OMcp34_221 = OMcp34_220+qd[21]*ROcp34_820;
  OMcp34_321 = OMcp34_320+qd[21]*ROcp34_920;
  ORcp34_121 = OMcp34_220*RLcp34_321-OMcp34_320*RLcp34_221;
  ORcp34_221 = -(OMcp34_120*RLcp34_321-OMcp34_320*RLcp34_121);
  ORcp34_321 = OMcp34_120*RLcp34_221-OMcp34_220*RLcp34_121;
  OPcp34_121 = OPcp34_120+qd[21]*(OMcp34_220*ROcp34_920-OMcp34_320*ROcp34_820)+qdd[21]*ROcp34_720;
  OPcp34_221 = OPcp34_220-qd[21]*(OMcp34_120*ROcp34_920-OMcp34_320*ROcp34_720)+qdd[21]*ROcp34_820;
  OPcp34_321 = OPcp34_320+qd[21]*(OMcp34_120*ROcp34_820-OMcp34_220*ROcp34_720)+qdd[21]*ROcp34_920;

// = = Block_0_0_1_11_0_6 = = 
 
// Sensor Kinematics 


  ROcp34_126 = ROcp34_121*C26-ROcp34_720*S26;
  ROcp34_226 = ROcp34_221*C26-ROcp34_820*S26;
  ROcp34_326 = ROcp34_321*C26-ROcp34_920*S26;
  ROcp34_726 = ROcp34_121*S26+ROcp34_720*C26;
  ROcp34_826 = ROcp34_221*S26+ROcp34_820*C26;
  ROcp34_926 = ROcp34_321*S26+ROcp34_920*C26;
  ROcp34_427 = ROcp34_421*C27+ROcp34_726*S27;
  ROcp34_527 = ROcp34_521*C27+ROcp34_826*S27;
  ROcp34_627 = ROcp34_621*C27+ROcp34_926*S27;
  ROcp34_727 = -(ROcp34_421*S27-ROcp34_726*C27);
  ROcp34_827 = -(ROcp34_521*S27-ROcp34_826*C27);
  ROcp34_927 = -(ROcp34_621*S27-ROcp34_926*C27);
  ROcp34_128 = ROcp34_126*C28+ROcp34_427*S28;
  ROcp34_228 = ROcp34_226*C28+ROcp34_527*S28;
  ROcp34_328 = ROcp34_326*C28+ROcp34_627*S28;
  ROcp34_428 = -(ROcp34_126*S28-ROcp34_427*C28);
  ROcp34_528 = -(ROcp34_226*S28-ROcp34_527*C28);
  ROcp34_628 = -(ROcp34_326*S28-ROcp34_627*C28);
  RLcp34_126 = ROcp34_121*s->dpt[1][41]+ROcp34_421*s->dpt[2][41]+ROcp34_720*s->dpt[3][41];
  RLcp34_226 = ROcp34_221*s->dpt[1][41]+ROcp34_521*s->dpt[2][41]+ROcp34_820*s->dpt[3][41];
  RLcp34_326 = ROcp34_321*s->dpt[1][41]+ROcp34_621*s->dpt[2][41]+ROcp34_920*s->dpt[3][41];
  OMcp34_126 = OMcp34_121+qd[26]*ROcp34_421;
  OMcp34_226 = OMcp34_221+qd[26]*ROcp34_521;
  OMcp34_326 = OMcp34_321+qd[26]*ROcp34_621;
  ORcp34_126 = OMcp34_221*RLcp34_326-OMcp34_321*RLcp34_226;
  ORcp34_226 = -(OMcp34_121*RLcp34_326-OMcp34_321*RLcp34_126);
  ORcp34_326 = OMcp34_121*RLcp34_226-OMcp34_221*RLcp34_126;
  OPcp34_126 = OPcp34_121+qd[26]*(OMcp34_221*ROcp34_621-OMcp34_321*ROcp34_521)+qdd[26]*ROcp34_421;
  OPcp34_226 = OPcp34_221-qd[26]*(OMcp34_121*ROcp34_621-OMcp34_321*ROcp34_421)+qdd[26]*ROcp34_521;
  OPcp34_326 = OPcp34_321+qd[26]*(OMcp34_121*ROcp34_521-OMcp34_221*ROcp34_421)+qdd[26]*ROcp34_621;
  RLcp34_127 = ROcp34_421*s->dpt[2][52];
  RLcp34_227 = ROcp34_521*s->dpt[2][52];
  RLcp34_327 = ROcp34_621*s->dpt[2][52];
  OMcp34_127 = OMcp34_126+qd[27]*ROcp34_126;
  OMcp34_227 = OMcp34_226+qd[27]*ROcp34_226;
  OMcp34_327 = OMcp34_326+qd[27]*ROcp34_326;
  ORcp34_127 = OMcp34_226*RLcp34_327-OMcp34_326*RLcp34_227;
  ORcp34_227 = -(OMcp34_126*RLcp34_327-OMcp34_326*RLcp34_127);
  ORcp34_327 = OMcp34_126*RLcp34_227-OMcp34_226*RLcp34_127;
  OPcp34_127 = OPcp34_126+qd[27]*(OMcp34_226*ROcp34_326-OMcp34_326*ROcp34_226)+qdd[27]*ROcp34_126;
  OPcp34_227 = OPcp34_226-qd[27]*(OMcp34_126*ROcp34_326-OMcp34_326*ROcp34_126)+qdd[27]*ROcp34_226;
  OPcp34_327 = OPcp34_326+qd[27]*(OMcp34_126*ROcp34_226-OMcp34_226*ROcp34_126)+qdd[27]*ROcp34_326;
  RLcp34_128 = ROcp34_727*s->dpt[3][54];
  RLcp34_228 = ROcp34_827*s->dpt[3][54];
  RLcp34_328 = ROcp34_927*s->dpt[3][54];
  ORcp34_128 = OMcp34_227*RLcp34_328-OMcp34_327*RLcp34_228;
  ORcp34_228 = -(OMcp34_127*RLcp34_328-OMcp34_327*RLcp34_128);
  ORcp34_328 = OMcp34_127*RLcp34_228-OMcp34_227*RLcp34_128;
  PxF11[1] = q[1]+RLcp34_119+RLcp34_121+RLcp34_126+RLcp34_127+RLcp34_128;
  PxF11[2] = q[2]+RLcp34_219+RLcp34_221+RLcp34_226+RLcp34_227+RLcp34_228;
  PxF11[3] = q[3]+RLcp34_319+RLcp34_321+RLcp34_326+RLcp34_327+RLcp34_328;
  RxF11[1][1] = ROcp34_128;
  RxF11[1][2] = ROcp34_228;
  RxF11[1][3] = ROcp34_328;
  RxF11[2][1] = ROcp34_428;
  RxF11[2][2] = ROcp34_528;
  RxF11[2][3] = ROcp34_628;
  RxF11[3][1] = ROcp34_727;
  RxF11[3][2] = ROcp34_827;
  RxF11[3][3] = ROcp34_927;
  VxF11[1] = qd[1]+ORcp34_119+ORcp34_121+ORcp34_126+ORcp34_127+ORcp34_128;
  VxF11[2] = qd[2]+ORcp34_219+ORcp34_221+ORcp34_226+ORcp34_227+ORcp34_228;
  VxF11[3] = qd[3]+ORcp34_319+ORcp34_321+ORcp34_326+ORcp34_327+ORcp34_328;
  OMxF11[1] = OMcp34_127+qd[28]*ROcp34_727;
  OMxF11[2] = OMcp34_227+qd[28]*ROcp34_827;
  OMxF11[3] = OMcp34_327+qd[28]*ROcp34_927;
  AxF11[1] = qdd[1]+OMcp34_220*ORcp34_321+OMcp34_221*ORcp34_326+OMcp34_226*ORcp34_327+OMcp34_227*ORcp34_328+OMcp34_26*
 ORcp34_319-OMcp34_320*ORcp34_221-OMcp34_321*ORcp34_226-OMcp34_326*ORcp34_227-OMcp34_327*ORcp34_228-OMcp34_36*ORcp34_219+
 OPcp34_220*RLcp34_321+OPcp34_221*RLcp34_326+OPcp34_226*RLcp34_327+OPcp34_227*RLcp34_328+OPcp34_26*RLcp34_319-OPcp34_320*
 RLcp34_221-OPcp34_321*RLcp34_226-OPcp34_326*RLcp34_227-OPcp34_327*RLcp34_228-OPcp34_36*RLcp34_219;
  AxF11[2] = qdd[2]-OMcp34_120*ORcp34_321-OMcp34_121*ORcp34_326-OMcp34_126*ORcp34_327-OMcp34_127*ORcp34_328-OMcp34_16*
 ORcp34_319+OMcp34_320*ORcp34_121+OMcp34_321*ORcp34_126+OMcp34_326*ORcp34_127+OMcp34_327*ORcp34_128+OMcp34_36*ORcp34_119-
 OPcp34_120*RLcp34_321-OPcp34_121*RLcp34_326-OPcp34_126*RLcp34_327-OPcp34_127*RLcp34_328-OPcp34_16*RLcp34_319+OPcp34_320*
 RLcp34_121+OPcp34_321*RLcp34_126+OPcp34_326*RLcp34_127+OPcp34_327*RLcp34_128+OPcp34_36*RLcp34_119;
  AxF11[3] = qdd[3]+OMcp34_120*ORcp34_221+OMcp34_121*ORcp34_226+OMcp34_126*ORcp34_227+OMcp34_127*ORcp34_228+OMcp34_16*
 ORcp34_219-OMcp34_220*ORcp34_121-OMcp34_221*ORcp34_126-OMcp34_226*ORcp34_127-OMcp34_227*ORcp34_128-OMcp34_26*ORcp34_119+
 OPcp34_120*RLcp34_221+OPcp34_121*RLcp34_226+OPcp34_126*RLcp34_227+OPcp34_127*RLcp34_228+OPcp34_16*RLcp34_219-OPcp34_220*
 RLcp34_121-OPcp34_221*RLcp34_126-OPcp34_226*RLcp34_127-OPcp34_227*RLcp34_128-OPcp34_26*RLcp34_119;
  OMPxF11[1] = OPcp34_127+qd[28]*(OMcp34_227*ROcp34_927-OMcp34_327*ROcp34_827)+qdd[28]*ROcp34_727;
  OMPxF11[2] = OPcp34_227-qd[28]*(OMcp34_127*ROcp34_927-OMcp34_327*ROcp34_727)+qdd[28]*ROcp34_827;
  OMPxF11[3] = OPcp34_327+qd[28]*(OMcp34_127*ROcp34_827-OMcp34_227*ROcp34_727)+qdd[28]*ROcp34_927;
 
// Sensor Forces Computation 

  SWr11 = user_ExtForces(PxF11,RxF11,VxF11,OMxF11,AxF11,OMPxF11,s,tsim,11);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc135 = ROcp34_128*SWr11[1]+ROcp34_228*SWr11[2]+ROcp34_328*SWr11[3];
  xfrc235 = ROcp34_428*SWr11[1]+ROcp34_528*SWr11[2]+ROcp34_628*SWr11[3];
  xfrc335 = ROcp34_727*SWr11[1]+ROcp34_827*SWr11[2]+ROcp34_927*SWr11[3];
  frc[1][28] = s->frc[1][28]+xfrc135;
  frc[2][28] = s->frc[2][28]+xfrc235;
  frc[3][28] = s->frc[3][28]+xfrc335;
  xtrq135 = ROcp34_128*SWr11[4]+ROcp34_228*SWr11[5]+ROcp34_328*SWr11[6];
  xtrq235 = ROcp34_428*SWr11[4]+ROcp34_528*SWr11[5]+ROcp34_628*SWr11[6];
  xtrq335 = ROcp34_727*SWr11[4]+ROcp34_827*SWr11[5]+ROcp34_927*SWr11[6];
  trq[1][28] = s->trq[1][28]+xtrq135-xfrc235*(SWr11[9]-s->l[3][28])+xfrc335*(SWr11[8]-s->l[2][28]);
  trq[2][28] = s->trq[2][28]+xtrq235+xfrc135*(SWr11[9]-s->l[3][28])-xfrc335*(SWr11[7]-s->l[1][28]);
  trq[3][28] = s->trq[3][28]+xtrq335-xfrc135*(SWr11[8]-s->l[2][28])+xfrc235*(SWr11[7]-s->l[1][28]);

// = = Block_0_0_1_12_0_1 = = 
 
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

// = = Block_0_0_1_12_0_4 = = 
 
// Sensor Kinematics 


  ROcp35_419 = ROcp35_46*C19+S19*S5;
  ROcp35_519 = ROcp35_56*C19+ROcp35_85*S19;
  ROcp35_619 = ROcp35_66*C19+ROcp35_95*S19;
  ROcp35_719 = -(ROcp35_46*S19-C19*S5);
  ROcp35_819 = -(ROcp35_56*S19-ROcp35_85*C19);
  ROcp35_919 = -(ROcp35_66*S19-ROcp35_95*C19);
  ROcp35_120 = ROcp35_16*C20-ROcp35_719*S20;
  ROcp35_220 = ROcp35_26*C20-ROcp35_819*S20;
  ROcp35_320 = ROcp35_36*C20-ROcp35_919*S20;
  ROcp35_720 = ROcp35_16*S20+ROcp35_719*C20;
  ROcp35_820 = ROcp35_26*S20+ROcp35_819*C20;
  ROcp35_920 = ROcp35_36*S20+ROcp35_919*C20;
  ROcp35_121 = ROcp35_120*C21+ROcp35_419*S21;
  ROcp35_221 = ROcp35_220*C21+ROcp35_519*S21;
  ROcp35_321 = ROcp35_320*C21+ROcp35_619*S21;
  ROcp35_421 = -(ROcp35_120*S21-ROcp35_419*C21);
  ROcp35_521 = -(ROcp35_220*S21-ROcp35_519*C21);
  ROcp35_621 = -(ROcp35_320*S21-ROcp35_619*C21);
  RLcp35_119 = ROcp35_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp35_219 = ROcp35_26*s->dpt[1][4]+ROcp35_85*s->dpt[3][4];
  RLcp35_319 = ROcp35_36*s->dpt[1][4]+ROcp35_95*s->dpt[3][4];
  OMcp35_119 = OMcp35_16+qd[19]*ROcp35_16;
  OMcp35_219 = OMcp35_26+qd[19]*ROcp35_26;
  OMcp35_319 = OMcp35_36+qd[19]*ROcp35_36;
  ORcp35_119 = OMcp35_26*RLcp35_319-OMcp35_36*RLcp35_219;
  ORcp35_219 = -(OMcp35_16*RLcp35_319-OMcp35_36*RLcp35_119);
  ORcp35_319 = OMcp35_16*RLcp35_219-OMcp35_26*RLcp35_119;
  OMcp35_120 = OMcp35_119+qd[20]*ROcp35_419;
  OMcp35_220 = OMcp35_219+qd[20]*ROcp35_519;
  OMcp35_320 = OMcp35_319+qd[20]*ROcp35_619;
  OPcp35_120 = OPcp35_16+qd[19]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26)+qd[20]*(OMcp35_219*ROcp35_619-OMcp35_319*
 ROcp35_519)+qdd[19]*ROcp35_16+qdd[20]*ROcp35_419;
  OPcp35_220 = OPcp35_26-qd[19]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16)-qd[20]*(OMcp35_119*ROcp35_619-OMcp35_319*
 ROcp35_419)+qdd[19]*ROcp35_26+qdd[20]*ROcp35_519;
  OPcp35_320 = OPcp35_36+qd[19]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16)+qd[20]*(OMcp35_119*ROcp35_519-OMcp35_219*
 ROcp35_419)+qdd[19]*ROcp35_36+qdd[20]*ROcp35_619;
  RLcp35_121 = ROcp35_720*s->dpt[3][36];
  RLcp35_221 = ROcp35_820*s->dpt[3][36];
  RLcp35_321 = ROcp35_920*s->dpt[3][36];
  OMcp35_121 = OMcp35_120+qd[21]*ROcp35_720;
  OMcp35_221 = OMcp35_220+qd[21]*ROcp35_820;
  OMcp35_321 = OMcp35_320+qd[21]*ROcp35_920;
  ORcp35_121 = OMcp35_220*RLcp35_321-OMcp35_320*RLcp35_221;
  ORcp35_221 = -(OMcp35_120*RLcp35_321-OMcp35_320*RLcp35_121);
  ORcp35_321 = OMcp35_120*RLcp35_221-OMcp35_220*RLcp35_121;
  OPcp35_121 = OPcp35_120+qd[21]*(OMcp35_220*ROcp35_920-OMcp35_320*ROcp35_820)+qdd[21]*ROcp35_720;
  OPcp35_221 = OPcp35_220-qd[21]*(OMcp35_120*ROcp35_920-OMcp35_320*ROcp35_720)+qdd[21]*ROcp35_820;
  OPcp35_321 = OPcp35_320+qd[21]*(OMcp35_120*ROcp35_820-OMcp35_220*ROcp35_720)+qdd[21]*ROcp35_920;

// = = Block_0_0_1_12_0_6 = = 
 
// Sensor Kinematics 


  ROcp35_126 = ROcp35_121*C26-ROcp35_720*S26;
  ROcp35_226 = ROcp35_221*C26-ROcp35_820*S26;
  ROcp35_326 = ROcp35_321*C26-ROcp35_920*S26;
  ROcp35_726 = ROcp35_121*S26+ROcp35_720*C26;
  ROcp35_826 = ROcp35_221*S26+ROcp35_820*C26;
  ROcp35_926 = ROcp35_321*S26+ROcp35_920*C26;
  ROcp35_427 = ROcp35_421*C27+ROcp35_726*S27;
  ROcp35_527 = ROcp35_521*C27+ROcp35_826*S27;
  ROcp35_627 = ROcp35_621*C27+ROcp35_926*S27;
  ROcp35_727 = -(ROcp35_421*S27-ROcp35_726*C27);
  ROcp35_827 = -(ROcp35_521*S27-ROcp35_826*C27);
  ROcp35_927 = -(ROcp35_621*S27-ROcp35_926*C27);
  ROcp35_128 = ROcp35_126*C28+ROcp35_427*S28;
  ROcp35_228 = ROcp35_226*C28+ROcp35_527*S28;
  ROcp35_328 = ROcp35_326*C28+ROcp35_627*S28;
  ROcp35_428 = -(ROcp35_126*S28-ROcp35_427*C28);
  ROcp35_528 = -(ROcp35_226*S28-ROcp35_527*C28);
  ROcp35_628 = -(ROcp35_326*S28-ROcp35_627*C28);
  ROcp35_129 = ROcp35_128*C29-ROcp35_727*S29;
  ROcp35_229 = ROcp35_228*C29-ROcp35_827*S29;
  ROcp35_329 = ROcp35_328*C29-ROcp35_927*S29;
  ROcp35_729 = ROcp35_128*S29+ROcp35_727*C29;
  ROcp35_829 = ROcp35_228*S29+ROcp35_827*C29;
  ROcp35_929 = ROcp35_328*S29+ROcp35_927*C29;
  RLcp35_126 = ROcp35_121*s->dpt[1][41]+ROcp35_421*s->dpt[2][41]+ROcp35_720*s->dpt[3][41];
  RLcp35_226 = ROcp35_221*s->dpt[1][41]+ROcp35_521*s->dpt[2][41]+ROcp35_820*s->dpt[3][41];
  RLcp35_326 = ROcp35_321*s->dpt[1][41]+ROcp35_621*s->dpt[2][41]+ROcp35_920*s->dpt[3][41];
  OMcp35_126 = OMcp35_121+qd[26]*ROcp35_421;
  OMcp35_226 = OMcp35_221+qd[26]*ROcp35_521;
  OMcp35_326 = OMcp35_321+qd[26]*ROcp35_621;
  ORcp35_126 = OMcp35_221*RLcp35_326-OMcp35_321*RLcp35_226;
  ORcp35_226 = -(OMcp35_121*RLcp35_326-OMcp35_321*RLcp35_126);
  ORcp35_326 = OMcp35_121*RLcp35_226-OMcp35_221*RLcp35_126;
  OPcp35_126 = OPcp35_121+qd[26]*(OMcp35_221*ROcp35_621-OMcp35_321*ROcp35_521)+qdd[26]*ROcp35_421;
  OPcp35_226 = OPcp35_221-qd[26]*(OMcp35_121*ROcp35_621-OMcp35_321*ROcp35_421)+qdd[26]*ROcp35_521;
  OPcp35_326 = OPcp35_321+qd[26]*(OMcp35_121*ROcp35_521-OMcp35_221*ROcp35_421)+qdd[26]*ROcp35_621;
  RLcp35_127 = ROcp35_421*s->dpt[2][52];
  RLcp35_227 = ROcp35_521*s->dpt[2][52];
  RLcp35_327 = ROcp35_621*s->dpt[2][52];
  OMcp35_127 = OMcp35_126+qd[27]*ROcp35_126;
  OMcp35_227 = OMcp35_226+qd[27]*ROcp35_226;
  OMcp35_327 = OMcp35_326+qd[27]*ROcp35_326;
  ORcp35_127 = OMcp35_226*RLcp35_327-OMcp35_326*RLcp35_227;
  ORcp35_227 = -(OMcp35_126*RLcp35_327-OMcp35_326*RLcp35_127);
  ORcp35_327 = OMcp35_126*RLcp35_227-OMcp35_226*RLcp35_127;
  OPcp35_127 = OPcp35_126+qd[27]*(OMcp35_226*ROcp35_326-OMcp35_326*ROcp35_226)+qdd[27]*ROcp35_126;
  OPcp35_227 = OPcp35_226-qd[27]*(OMcp35_126*ROcp35_326-OMcp35_326*ROcp35_126)+qdd[27]*ROcp35_226;
  OPcp35_327 = OPcp35_326+qd[27]*(OMcp35_126*ROcp35_226-OMcp35_226*ROcp35_126)+qdd[27]*ROcp35_326;
  RLcp35_128 = ROcp35_727*s->dpt[3][54];
  RLcp35_228 = ROcp35_827*s->dpt[3][54];
  RLcp35_328 = ROcp35_927*s->dpt[3][54];
  OMcp35_128 = OMcp35_127+qd[28]*ROcp35_727;
  OMcp35_228 = OMcp35_227+qd[28]*ROcp35_827;
  OMcp35_328 = OMcp35_327+qd[28]*ROcp35_927;
  ORcp35_128 = OMcp35_227*RLcp35_328-OMcp35_327*RLcp35_228;
  ORcp35_228 = -(OMcp35_127*RLcp35_328-OMcp35_327*RLcp35_128);
  ORcp35_328 = OMcp35_127*RLcp35_228-OMcp35_227*RLcp35_128;
  OPcp35_128 = OPcp35_127+qd[28]*(OMcp35_227*ROcp35_927-OMcp35_327*ROcp35_827)+qdd[28]*ROcp35_727;
  OPcp35_228 = OPcp35_227-qd[28]*(OMcp35_127*ROcp35_927-OMcp35_327*ROcp35_727)+qdd[28]*ROcp35_827;
  OPcp35_328 = OPcp35_327+qd[28]*(OMcp35_127*ROcp35_827-OMcp35_227*ROcp35_727)+qdd[28]*ROcp35_927;
  RLcp35_129 = ROcp35_727*s->dpt[3][57];
  RLcp35_229 = ROcp35_827*s->dpt[3][57];
  RLcp35_329 = ROcp35_927*s->dpt[3][57];
  ORcp35_129 = OMcp35_228*RLcp35_329-OMcp35_328*RLcp35_229;
  ORcp35_229 = -(OMcp35_128*RLcp35_329-OMcp35_328*RLcp35_129);
  ORcp35_329 = OMcp35_128*RLcp35_229-OMcp35_228*RLcp35_129;
  PxF12[1] = q[1]+RLcp35_119+RLcp35_121+RLcp35_126+RLcp35_127+RLcp35_128+RLcp35_129;
  PxF12[2] = q[2]+RLcp35_219+RLcp35_221+RLcp35_226+RLcp35_227+RLcp35_228+RLcp35_229;
  PxF12[3] = q[3]+RLcp35_319+RLcp35_321+RLcp35_326+RLcp35_327+RLcp35_328+RLcp35_329;
  RxF12[1][1] = ROcp35_129;
  RxF12[1][2] = ROcp35_229;
  RxF12[1][3] = ROcp35_329;
  RxF12[2][1] = ROcp35_428;
  RxF12[2][2] = ROcp35_528;
  RxF12[2][3] = ROcp35_628;
  RxF12[3][1] = ROcp35_729;
  RxF12[3][2] = ROcp35_829;
  RxF12[3][3] = ROcp35_929;
  VxF12[1] = qd[1]+ORcp35_119+ORcp35_121+ORcp35_126+ORcp35_127+ORcp35_128+ORcp35_129;
  VxF12[2] = qd[2]+ORcp35_219+ORcp35_221+ORcp35_226+ORcp35_227+ORcp35_228+ORcp35_229;
  VxF12[3] = qd[3]+ORcp35_319+ORcp35_321+ORcp35_326+ORcp35_327+ORcp35_328+ORcp35_329;
  OMxF12[1] = OMcp35_128+qd[29]*ROcp35_428;
  OMxF12[2] = OMcp35_228+qd[29]*ROcp35_528;
  OMxF12[3] = OMcp35_328+qd[29]*ROcp35_628;
  AxF12[1] = qdd[1]+OMcp35_220*ORcp35_321+OMcp35_221*ORcp35_326+OMcp35_226*ORcp35_327+OMcp35_227*ORcp35_328+OMcp35_228*
 ORcp35_329+OMcp35_26*ORcp35_319-OMcp35_320*ORcp35_221-OMcp35_321*ORcp35_226-OMcp35_326*ORcp35_227-OMcp35_327*ORcp35_228-
 OMcp35_328*ORcp35_229-OMcp35_36*ORcp35_219+OPcp35_220*RLcp35_321+OPcp35_221*RLcp35_326+OPcp35_226*RLcp35_327+OPcp35_227*
 RLcp35_328+OPcp35_228*RLcp35_329+OPcp35_26*RLcp35_319-OPcp35_320*RLcp35_221-OPcp35_321*RLcp35_226-OPcp35_326*RLcp35_227-
 OPcp35_327*RLcp35_228-OPcp35_328*RLcp35_229-OPcp35_36*RLcp35_219;
  AxF12[2] = qdd[2]-OMcp35_120*ORcp35_321-OMcp35_121*ORcp35_326-OMcp35_126*ORcp35_327-OMcp35_127*ORcp35_328-OMcp35_128*
 ORcp35_329-OMcp35_16*ORcp35_319+OMcp35_320*ORcp35_121+OMcp35_321*ORcp35_126+OMcp35_326*ORcp35_127+OMcp35_327*ORcp35_128+
 OMcp35_328*ORcp35_129+OMcp35_36*ORcp35_119-OPcp35_120*RLcp35_321-OPcp35_121*RLcp35_326-OPcp35_126*RLcp35_327-OPcp35_127*
 RLcp35_328-OPcp35_128*RLcp35_329-OPcp35_16*RLcp35_319+OPcp35_320*RLcp35_121+OPcp35_321*RLcp35_126+OPcp35_326*RLcp35_127+
 OPcp35_327*RLcp35_128+OPcp35_328*RLcp35_129+OPcp35_36*RLcp35_119;
  AxF12[3] = qdd[3]+OMcp35_120*ORcp35_221+OMcp35_121*ORcp35_226+OMcp35_126*ORcp35_227+OMcp35_127*ORcp35_228+OMcp35_128*
 ORcp35_229+OMcp35_16*ORcp35_219-OMcp35_220*ORcp35_121-OMcp35_221*ORcp35_126-OMcp35_226*ORcp35_127-OMcp35_227*ORcp35_128-
 OMcp35_228*ORcp35_129-OMcp35_26*ORcp35_119+OPcp35_120*RLcp35_221+OPcp35_121*RLcp35_226+OPcp35_126*RLcp35_227+OPcp35_127*
 RLcp35_228+OPcp35_128*RLcp35_229+OPcp35_16*RLcp35_219-OPcp35_220*RLcp35_121-OPcp35_221*RLcp35_126-OPcp35_226*RLcp35_127-
 OPcp35_227*RLcp35_128-OPcp35_228*RLcp35_129-OPcp35_26*RLcp35_119;
  OMPxF12[1] = OPcp35_128+qd[29]*(OMcp35_228*ROcp35_628-OMcp35_328*ROcp35_528)+qdd[29]*ROcp35_428;
  OMPxF12[2] = OPcp35_228-qd[29]*(OMcp35_128*ROcp35_628-OMcp35_328*ROcp35_428)+qdd[29]*ROcp35_528;
  OMPxF12[3] = OPcp35_328+qd[29]*(OMcp35_128*ROcp35_528-OMcp35_228*ROcp35_428)+qdd[29]*ROcp35_628;
 
// Sensor Forces Computation 

  SWr12 = user_ExtForces(PxF12,RxF12,VxF12,OMxF12,AxF12,OMPxF12,s,tsim,12);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc136 = ROcp35_129*SWr12[1]+ROcp35_229*SWr12[2]+ROcp35_329*SWr12[3];
  xfrc236 = ROcp35_428*SWr12[1]+ROcp35_528*SWr12[2]+ROcp35_628*SWr12[3];
  xfrc336 = ROcp35_729*SWr12[1]+ROcp35_829*SWr12[2]+ROcp35_929*SWr12[3];
  frc[1][29] = s->frc[1][29]+xfrc136;
  frc[2][29] = s->frc[2][29]+xfrc236;
  frc[3][29] = s->frc[3][29]+xfrc336;
  xtrq136 = ROcp35_129*SWr12[4]+ROcp35_229*SWr12[5]+ROcp35_329*SWr12[6];
  xtrq236 = ROcp35_428*SWr12[4]+ROcp35_528*SWr12[5]+ROcp35_628*SWr12[6];
  xtrq336 = ROcp35_729*SWr12[4]+ROcp35_829*SWr12[5]+ROcp35_929*SWr12[6];
  trq[1][29] = s->trq[1][29]+xtrq136-xfrc236*(SWr12[9]-s->l[3][29])+xfrc336*(SWr12[8]-s->l[2][29]);
  trq[2][29] = s->trq[2][29]+xtrq236+xfrc136*(SWr12[9]-s->l[3][29])-xfrc336*(SWr12[7]-s->l[1][29]);
  trq[3][29] = s->trq[3][29]+xtrq336-xfrc136*(SWr12[8]-s->l[2][29])+xfrc236*(SWr12[7]-s->l[1][29]);

// = = Block_0_0_1_12_1_0 = = 
 
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
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][14] = s->frc[1][14];
  frc[2][14] = s->frc[2][14];
  frc[3][14] = s->frc[3][14];
  frc[1][17] = s->frc[1][17];
  frc[2][17] = s->frc[2][17];
  frc[3][17] = s->frc[3][17];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][14] = s->trq[1][14];
  trq[2][14] = s->trq[2][14];
  trq[3][14] = s->trq[3][14];
  trq[1][17] = s->trq[1][17];
  trq[2][17] = s->trq[2][17];
  trq[3][17] = s->trq[3][17];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];

// ====== END Task 0 ====== 


}
 

