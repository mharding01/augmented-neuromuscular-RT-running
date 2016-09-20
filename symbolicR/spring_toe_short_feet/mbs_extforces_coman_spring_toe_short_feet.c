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
//	==> Generation Date : Tue May 24 15:03:39 2016
//
//	==> Project name : coman_spring_toe_short_feet
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 7291
//
//	==> Generation Time :  0.130 seconds
//	==> Post-Processing :  0.130 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][31];
// double trq[3][31];
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
 
#include "mbs_extforces_coman_spring_toe_short_feet.h" 
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
  C13 = cos(q[13]);
  S13 = sin(q[13]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

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
  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);
  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

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
  RLcp27_159 = ROcp27_712*s->dpt[3][19];
  RLcp27_259 = ROcp27_812*s->dpt[3][19];
  RLcp27_359 = ROcp27_912*s->dpt[3][19];
  ORcp27_159 = OMcp27_212*RLcp27_359-OMcp27_312*RLcp27_259;
  ORcp27_259 = -(OMcp27_112*RLcp27_359-OMcp27_312*RLcp27_159);
  ORcp27_359 = OMcp27_112*RLcp27_259-OMcp27_212*RLcp27_159;
  PxF4[1] = q[1]+RLcp27_110+RLcp27_111+RLcp27_159+RLcp27_17+RLcp27_18+RLcp27_19;
  PxF4[2] = q[2]+RLcp27_210+RLcp27_211+RLcp27_259+RLcp27_27+RLcp27_28+RLcp27_29;
  PxF4[3] = q[3]+RLcp27_310+RLcp27_311+RLcp27_359+RLcp27_37+RLcp27_38+RLcp27_39;
  RxF4[1][1] = ROcp27_112;
  RxF4[1][2] = ROcp27_212;
  RxF4[1][3] = ROcp27_312;
  RxF4[2][1] = ROcp27_411;
  RxF4[2][2] = ROcp27_511;
  RxF4[2][3] = ROcp27_611;
  RxF4[3][1] = ROcp27_712;
  RxF4[3][2] = ROcp27_812;
  RxF4[3][3] = ROcp27_912;
  VxF4[1] = qd[1]+ORcp27_110+ORcp27_111+ORcp27_159+ORcp27_17+ORcp27_18+ORcp27_19;
  VxF4[2] = qd[2]+ORcp27_210+ORcp27_211+ORcp27_259+ORcp27_27+ORcp27_28+ORcp27_29;
  VxF4[3] = qd[3]+ORcp27_310+ORcp27_311+ORcp27_359+ORcp27_37+ORcp27_38+ORcp27_39;
  OMxF4[1] = OMcp27_112;
  OMxF4[2] = OMcp27_212;
  OMxF4[3] = OMcp27_312;
  AxF4[1] = qdd[1]+OMcp27_210*ORcp27_311+OMcp27_212*ORcp27_359+OMcp27_26*ORcp27_37+OMcp27_27*ORcp27_38+OMcp27_28*
 ORcp27_39+OMcp27_29*ORcp27_310-OMcp27_310*ORcp27_211-OMcp27_312*ORcp27_259-OMcp27_36*ORcp27_27-OMcp27_37*ORcp27_28-OMcp27_38
 *ORcp27_29-OMcp27_39*ORcp27_210+OPcp27_210*RLcp27_311+OPcp27_212*RLcp27_359+OPcp27_26*RLcp27_37+OPcp27_27*RLcp27_38+
 OPcp27_28*RLcp27_39+OPcp27_29*RLcp27_310-OPcp27_310*RLcp27_211-OPcp27_312*RLcp27_259-OPcp27_36*RLcp27_27-OPcp27_37*RLcp27_28
 -OPcp27_38*RLcp27_29-OPcp27_39*RLcp27_210;
  AxF4[2] = qdd[2]-OMcp27_110*ORcp27_311-OMcp27_112*ORcp27_359-OMcp27_16*ORcp27_37-OMcp27_17*ORcp27_38-OMcp27_18*
 ORcp27_39-OMcp27_19*ORcp27_310+OMcp27_310*ORcp27_111+OMcp27_312*ORcp27_159+OMcp27_36*ORcp27_17+OMcp27_37*ORcp27_18+OMcp27_38
 *ORcp27_19+OMcp27_39*ORcp27_110-OPcp27_110*RLcp27_311-OPcp27_112*RLcp27_359-OPcp27_16*RLcp27_37-OPcp27_17*RLcp27_38-
 OPcp27_18*RLcp27_39-OPcp27_19*RLcp27_310+OPcp27_310*RLcp27_111+OPcp27_312*RLcp27_159+OPcp27_36*RLcp27_17+OPcp27_37*RLcp27_18
 +OPcp27_38*RLcp27_19+OPcp27_39*RLcp27_110;
  AxF4[3] = qdd[3]+OMcp27_110*ORcp27_211+OMcp27_112*ORcp27_259+OMcp27_16*ORcp27_27+OMcp27_17*ORcp27_28+OMcp27_18*
 ORcp27_29+OMcp27_19*ORcp27_210-OMcp27_210*ORcp27_111-OMcp27_212*ORcp27_159-OMcp27_26*ORcp27_17-OMcp27_27*ORcp27_18-OMcp27_28
 *ORcp27_19-OMcp27_29*ORcp27_110+OPcp27_110*RLcp27_211+OPcp27_112*RLcp27_259+OPcp27_16*RLcp27_27+OPcp27_17*RLcp27_28+
 OPcp27_18*RLcp27_29+OPcp27_19*RLcp27_210-OPcp27_210*RLcp27_111-OPcp27_212*RLcp27_159-OPcp27_26*RLcp27_17-OPcp27_27*RLcp27_18
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
  trq[1][12] = s->trq[1][12]+xtrq128-xfrc228*(SWr4[9]-s->l[3][12])+xfrc328*SWr4[8];
  trq[2][12] = s->trq[2][12]+xtrq228+xfrc128*(SWr4[9]-s->l[3][12])-xfrc328*(SWr4[7]-s->l[1][12]);
  trq[3][12] = s->trq[3][12]+xtrq328-xfrc128*SWr4[8]+xfrc228*(SWr4[7]-s->l[1][12]);

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

// = = Block_0_0_1_5_0_2 = = 
 
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
  ROcp28_411 = ROcp28_49*C11+ROcp28_710*S11;
  ROcp28_511 = ROcp28_59*C11+ROcp28_810*S11;
  ROcp28_611 = ROcp28_69*C11+ROcp28_910*S11;
  ROcp28_711 = -(ROcp28_49*S11-ROcp28_710*C11);
  ROcp28_811 = -(ROcp28_59*S11-ROcp28_810*C11);
  ROcp28_911 = -(ROcp28_69*S11-ROcp28_910*C11);
  ROcp28_112 = ROcp28_110*C12-ROcp28_711*S12;
  ROcp28_212 = ROcp28_210*C12-ROcp28_811*S12;
  ROcp28_312 = ROcp28_310*C12-ROcp28_911*S12;
  ROcp28_712 = ROcp28_110*S12+ROcp28_711*C12;
  ROcp28_812 = ROcp28_210*S12+ROcp28_811*C12;
  ROcp28_912 = ROcp28_310*S12+ROcp28_911*C12;
  ROcp28_113 = ROcp28_112*C13-ROcp28_712*S13;
  ROcp28_213 = ROcp28_212*C13-ROcp28_812*S13;
  ROcp28_313 = ROcp28_312*C13-ROcp28_912*S13;
  ROcp28_713 = ROcp28_112*S13+ROcp28_712*C13;
  ROcp28_813 = ROcp28_212*S13+ROcp28_812*C13;
  ROcp28_913 = ROcp28_312*S13+ROcp28_912*C13;
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
  OMcp28_110 = OMcp28_19+qd[10]*ROcp28_49;
  OMcp28_210 = OMcp28_29+qd[10]*ROcp28_59;
  OMcp28_310 = OMcp28_39+qd[10]*ROcp28_69;
  ORcp28_110 = OMcp28_29*RLcp28_310-OMcp28_39*RLcp28_210;
  ORcp28_210 = -(OMcp28_19*RLcp28_310-OMcp28_39*RLcp28_110);
  ORcp28_310 = OMcp28_19*RLcp28_210-OMcp28_29*RLcp28_110;
  OPcp28_110 = OPcp28_19+qd[10]*(OMcp28_29*ROcp28_69-OMcp28_39*ROcp28_59)+qdd[10]*ROcp28_49;
  OPcp28_210 = OPcp28_29-qd[10]*(OMcp28_19*ROcp28_69-OMcp28_39*ROcp28_49)+qdd[10]*ROcp28_59;
  OPcp28_310 = OPcp28_39+qd[10]*(OMcp28_19*ROcp28_59-OMcp28_29*ROcp28_49)+qdd[10]*ROcp28_69;
  RLcp28_111 = ROcp28_710*s->dpt[3][14];
  RLcp28_211 = ROcp28_810*s->dpt[3][14];
  RLcp28_311 = ROcp28_910*s->dpt[3][14];
  OMcp28_111 = OMcp28_110+qd[11]*ROcp28_110;
  OMcp28_211 = OMcp28_210+qd[11]*ROcp28_210;
  OMcp28_311 = OMcp28_310+qd[11]*ROcp28_310;
  ORcp28_111 = OMcp28_210*RLcp28_311-OMcp28_310*RLcp28_211;
  ORcp28_211 = -(OMcp28_110*RLcp28_311-OMcp28_310*RLcp28_111);
  ORcp28_311 = OMcp28_110*RLcp28_211-OMcp28_210*RLcp28_111;
  OMcp28_112 = OMcp28_111+qd[12]*ROcp28_411;
  OMcp28_212 = OMcp28_211+qd[12]*ROcp28_511;
  OMcp28_312 = OMcp28_311+qd[12]*ROcp28_611;
  OPcp28_112 = OPcp28_110+qd[11]*(OMcp28_210*ROcp28_310-OMcp28_310*ROcp28_210)+qd[12]*(OMcp28_211*ROcp28_611-OMcp28_311*
 ROcp28_511)+qdd[11]*ROcp28_110+qdd[12]*ROcp28_411;
  OPcp28_212 = OPcp28_210-qd[11]*(OMcp28_110*ROcp28_310-OMcp28_310*ROcp28_110)-qd[12]*(OMcp28_111*ROcp28_611-OMcp28_311*
 ROcp28_411)+qdd[11]*ROcp28_210+qdd[12]*ROcp28_511;
  OPcp28_312 = OPcp28_310+qd[11]*(OMcp28_110*ROcp28_210-OMcp28_210*ROcp28_110)+qd[12]*(OMcp28_111*ROcp28_511-OMcp28_211*
 ROcp28_411)+qdd[11]*ROcp28_310+qdd[12]*ROcp28_611;
  RLcp28_113 = ROcp28_112*s->dpt[1][20]+ROcp28_712*s->dpt[3][20];
  RLcp28_213 = ROcp28_212*s->dpt[1][20]+ROcp28_812*s->dpt[3][20];
  RLcp28_313 = ROcp28_312*s->dpt[1][20]+ROcp28_912*s->dpt[3][20];
  ORcp28_113 = OMcp28_212*RLcp28_313-OMcp28_312*RLcp28_213;
  ORcp28_213 = -(OMcp28_112*RLcp28_313-OMcp28_312*RLcp28_113);
  ORcp28_313 = OMcp28_112*RLcp28_213-OMcp28_212*RLcp28_113;
  PxF5[1] = q[1]+RLcp28_110+RLcp28_111+RLcp28_113+RLcp28_17+RLcp28_18+RLcp28_19;
  PxF5[2] = q[2]+RLcp28_210+RLcp28_211+RLcp28_213+RLcp28_27+RLcp28_28+RLcp28_29;
  PxF5[3] = q[3]+RLcp28_310+RLcp28_311+RLcp28_313+RLcp28_37+RLcp28_38+RLcp28_39;
  RxF5[1][1] = ROcp28_113;
  RxF5[1][2] = ROcp28_213;
  RxF5[1][3] = ROcp28_313;
  RxF5[2][1] = ROcp28_411;
  RxF5[2][2] = ROcp28_511;
  RxF5[2][3] = ROcp28_611;
  RxF5[3][1] = ROcp28_713;
  RxF5[3][2] = ROcp28_813;
  RxF5[3][3] = ROcp28_913;
  VxF5[1] = qd[1]+ORcp28_110+ORcp28_111+ORcp28_113+ORcp28_17+ORcp28_18+ORcp28_19;
  VxF5[2] = qd[2]+ORcp28_210+ORcp28_211+ORcp28_213+ORcp28_27+ORcp28_28+ORcp28_29;
  VxF5[3] = qd[3]+ORcp28_310+ORcp28_311+ORcp28_313+ORcp28_37+ORcp28_38+ORcp28_39;
  OMxF5[1] = OMcp28_112+qd[13]*ROcp28_411;
  OMxF5[2] = OMcp28_212+qd[13]*ROcp28_511;
  OMxF5[3] = OMcp28_312+qd[13]*ROcp28_611;
  AxF5[1] = qdd[1]+OMcp28_210*ORcp28_311+OMcp28_212*ORcp28_313+OMcp28_26*ORcp28_37+OMcp28_27*ORcp28_38+OMcp28_28*
 ORcp28_39+OMcp28_29*ORcp28_310-OMcp28_310*ORcp28_211-OMcp28_312*ORcp28_213-OMcp28_36*ORcp28_27-OMcp28_37*ORcp28_28-OMcp28_38
 *ORcp28_29-OMcp28_39*ORcp28_210+OPcp28_210*RLcp28_311+OPcp28_212*RLcp28_313+OPcp28_26*RLcp28_37+OPcp28_27*RLcp28_38+
 OPcp28_28*RLcp28_39+OPcp28_29*RLcp28_310-OPcp28_310*RLcp28_211-OPcp28_312*RLcp28_213-OPcp28_36*RLcp28_27-OPcp28_37*RLcp28_28
 -OPcp28_38*RLcp28_29-OPcp28_39*RLcp28_210;
  AxF5[2] = qdd[2]-OMcp28_110*ORcp28_311-OMcp28_112*ORcp28_313-OMcp28_16*ORcp28_37-OMcp28_17*ORcp28_38-OMcp28_18*
 ORcp28_39-OMcp28_19*ORcp28_310+OMcp28_310*ORcp28_111+OMcp28_312*ORcp28_113+OMcp28_36*ORcp28_17+OMcp28_37*ORcp28_18+OMcp28_38
 *ORcp28_19+OMcp28_39*ORcp28_110-OPcp28_110*RLcp28_311-OPcp28_112*RLcp28_313-OPcp28_16*RLcp28_37-OPcp28_17*RLcp28_38-
 OPcp28_18*RLcp28_39-OPcp28_19*RLcp28_310+OPcp28_310*RLcp28_111+OPcp28_312*RLcp28_113+OPcp28_36*RLcp28_17+OPcp28_37*RLcp28_18
 +OPcp28_38*RLcp28_19+OPcp28_39*RLcp28_110;
  AxF5[3] = qdd[3]+OMcp28_110*ORcp28_211+OMcp28_112*ORcp28_213+OMcp28_16*ORcp28_27+OMcp28_17*ORcp28_28+OMcp28_18*
 ORcp28_29+OMcp28_19*ORcp28_210-OMcp28_210*ORcp28_111-OMcp28_212*ORcp28_113-OMcp28_26*ORcp28_17-OMcp28_27*ORcp28_18-OMcp28_28
 *ORcp28_19-OMcp28_29*ORcp28_110+OPcp28_110*RLcp28_211+OPcp28_112*RLcp28_213+OPcp28_16*RLcp28_27+OPcp28_17*RLcp28_28+
 OPcp28_18*RLcp28_29+OPcp28_19*RLcp28_210-OPcp28_210*RLcp28_111-OPcp28_212*RLcp28_113-OPcp28_26*RLcp28_17-OPcp28_27*RLcp28_18
 -OPcp28_28*RLcp28_19-OPcp28_29*RLcp28_110;
  OMPxF5[1] = OPcp28_112+qd[13]*(OMcp28_212*ROcp28_611-OMcp28_312*ROcp28_511)+qdd[13]*ROcp28_411;
  OMPxF5[2] = OPcp28_212-qd[13]*(OMcp28_112*ROcp28_611-OMcp28_312*ROcp28_411)+qdd[13]*ROcp28_511;
  OMPxF5[3] = OPcp28_312+qd[13]*(OMcp28_112*ROcp28_511-OMcp28_212*ROcp28_411)+qdd[13]*ROcp28_611;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc129 = ROcp28_113*SWr5[1]+ROcp28_213*SWr5[2]+ROcp28_313*SWr5[3];
  xfrc229 = ROcp28_411*SWr5[1]+ROcp28_511*SWr5[2]+ROcp28_611*SWr5[3];
  xfrc329 = ROcp28_713*SWr5[1]+ROcp28_813*SWr5[2]+ROcp28_913*SWr5[3];
  frc[1][13] = s->frc[1][13]+xfrc129;
  frc[2][13] = s->frc[2][13]+xfrc229;
  frc[3][13] = s->frc[3][13]+xfrc329;
  xtrq129 = ROcp28_113*SWr5[4]+ROcp28_213*SWr5[5]+ROcp28_313*SWr5[6];
  xtrq229 = ROcp28_411*SWr5[4]+ROcp28_511*SWr5[5]+ROcp28_611*SWr5[6];
  xtrq329 = ROcp28_713*SWr5[4]+ROcp28_813*SWr5[5]+ROcp28_913*SWr5[6];
  trq[1][13] = s->trq[1][13]+xtrq129-xfrc229*(SWr5[9]-s->l[3][13])+xfrc329*SWr5[8];
  trq[2][13] = s->trq[2][13]+xtrq229+xfrc129*(SWr5[9]-s->l[3][13])-xfrc329*(SWr5[7]-s->l[1][13]);
  trq[3][13] = s->trq[3][13]+xtrq329-xfrc129*SWr5[8]+xfrc229*(SWr5[7]-s->l[1][13]);

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


  ROcp29_114 = ROcp29_16*C14-S14*S5;
  ROcp29_214 = ROcp29_26*C14-ROcp29_85*S14;
  ROcp29_314 = ROcp29_36*C14-ROcp29_95*S14;
  ROcp29_714 = ROcp29_16*S14+C14*S5;
  ROcp29_814 = ROcp29_26*S14+ROcp29_85*C14;
  ROcp29_914 = ROcp29_36*S14+ROcp29_95*C14;
  ROcp29_415 = ROcp29_46*C15+ROcp29_714*S15;
  ROcp29_515 = ROcp29_56*C15+ROcp29_814*S15;
  ROcp29_615 = ROcp29_66*C15+ROcp29_914*S15;
  ROcp29_715 = -(ROcp29_46*S15-ROcp29_714*C15);
  ROcp29_815 = -(ROcp29_56*S15-ROcp29_814*C15);
  ROcp29_915 = -(ROcp29_66*S15-ROcp29_914*C15);
  ROcp29_116 = ROcp29_114*C16+ROcp29_415*S16;
  ROcp29_216 = ROcp29_214*C16+ROcp29_515*S16;
  ROcp29_316 = ROcp29_314*C16+ROcp29_615*S16;
  ROcp29_416 = -(ROcp29_114*S16-ROcp29_415*C16);
  ROcp29_516 = -(ROcp29_214*S16-ROcp29_515*C16);
  ROcp29_616 = -(ROcp29_314*S16-ROcp29_615*C16);
  RLcp29_114 = ROcp29_46*s->dpt[2][3];
  RLcp29_214 = ROcp29_56*s->dpt[2][3];
  RLcp29_314 = ROcp29_66*s->dpt[2][3];
  OMcp29_114 = OMcp29_16+qd[14]*ROcp29_46;
  OMcp29_214 = OMcp29_26+qd[14]*ROcp29_56;
  OMcp29_314 = OMcp29_36+qd[14]*ROcp29_66;
  ORcp29_114 = OMcp29_26*RLcp29_314-OMcp29_36*RLcp29_214;
  ORcp29_214 = -(OMcp29_16*RLcp29_314-OMcp29_36*RLcp29_114);
  ORcp29_314 = OMcp29_16*RLcp29_214-OMcp29_26*RLcp29_114;
  OPcp29_114 = OPcp29_16+qd[14]*(OMcp29_26*ROcp29_66-OMcp29_36*ROcp29_56)+qdd[14]*ROcp29_46;
  OPcp29_214 = OPcp29_26-qd[14]*(OMcp29_16*ROcp29_66-OMcp29_36*ROcp29_46)+qdd[14]*ROcp29_56;
  OPcp29_314 = OPcp29_36+qd[14]*(OMcp29_16*ROcp29_56-OMcp29_26*ROcp29_46)+qdd[14]*ROcp29_66;
  RLcp29_115 = ROcp29_46*s->dpt[2][22];
  RLcp29_215 = ROcp29_56*s->dpt[2][22];
  RLcp29_315 = ROcp29_66*s->dpt[2][22];
  OMcp29_115 = OMcp29_114+qd[15]*ROcp29_114;
  OMcp29_215 = OMcp29_214+qd[15]*ROcp29_214;
  OMcp29_315 = OMcp29_314+qd[15]*ROcp29_314;
  ORcp29_115 = OMcp29_214*RLcp29_315-OMcp29_314*RLcp29_215;
  ORcp29_215 = -(OMcp29_114*RLcp29_315-OMcp29_314*RLcp29_115);
  ORcp29_315 = OMcp29_114*RLcp29_215-OMcp29_214*RLcp29_115;
  OPcp29_115 = OPcp29_114+qd[15]*(OMcp29_214*ROcp29_314-OMcp29_314*ROcp29_214)+qdd[15]*ROcp29_114;
  OPcp29_215 = OPcp29_214-qd[15]*(OMcp29_114*ROcp29_314-OMcp29_314*ROcp29_114)+qdd[15]*ROcp29_214;
  OPcp29_315 = OPcp29_314+qd[15]*(OMcp29_114*ROcp29_214-OMcp29_214*ROcp29_114)+qdd[15]*ROcp29_314;
  RLcp29_116 = ROcp29_715*s->dpt[3][24];
  RLcp29_216 = ROcp29_815*s->dpt[3][24];
  RLcp29_316 = ROcp29_915*s->dpt[3][24];
  ORcp29_116 = OMcp29_215*RLcp29_316-OMcp29_315*RLcp29_216;
  ORcp29_216 = -(OMcp29_115*RLcp29_316-OMcp29_315*RLcp29_116);
  ORcp29_316 = OMcp29_115*RLcp29_216-OMcp29_215*RLcp29_116;
  PxF6[1] = q[1]+RLcp29_114+RLcp29_115+RLcp29_116;
  PxF6[2] = q[2]+RLcp29_214+RLcp29_215+RLcp29_216;
  PxF6[3] = q[3]+RLcp29_314+RLcp29_315+RLcp29_316;
  RxF6[1][1] = ROcp29_116;
  RxF6[1][2] = ROcp29_216;
  RxF6[1][3] = ROcp29_316;
  RxF6[2][1] = ROcp29_416;
  RxF6[2][2] = ROcp29_516;
  RxF6[2][3] = ROcp29_616;
  RxF6[3][1] = ROcp29_715;
  RxF6[3][2] = ROcp29_815;
  RxF6[3][3] = ROcp29_915;
  VxF6[1] = qd[1]+ORcp29_114+ORcp29_115+ORcp29_116;
  VxF6[2] = qd[2]+ORcp29_214+ORcp29_215+ORcp29_216;
  VxF6[3] = qd[3]+ORcp29_314+ORcp29_315+ORcp29_316;
  OMxF6[1] = OMcp29_115+qd[16]*ROcp29_715;
  OMxF6[2] = OMcp29_215+qd[16]*ROcp29_815;
  OMxF6[3] = OMcp29_315+qd[16]*ROcp29_915;
  AxF6[1] = qdd[1]+OMcp29_214*ORcp29_315+OMcp29_215*ORcp29_316+OMcp29_26*ORcp29_314-OMcp29_314*ORcp29_215-OMcp29_315*
 ORcp29_216-OMcp29_36*ORcp29_214+OPcp29_214*RLcp29_315+OPcp29_215*RLcp29_316+OPcp29_26*RLcp29_314-OPcp29_314*RLcp29_215-
 OPcp29_315*RLcp29_216-OPcp29_36*RLcp29_214;
  AxF6[2] = qdd[2]-OMcp29_114*ORcp29_315-OMcp29_115*ORcp29_316-OMcp29_16*ORcp29_314+OMcp29_314*ORcp29_115+OMcp29_315*
 ORcp29_116+OMcp29_36*ORcp29_114-OPcp29_114*RLcp29_315-OPcp29_115*RLcp29_316-OPcp29_16*RLcp29_314+OPcp29_314*RLcp29_115+
 OPcp29_315*RLcp29_116+OPcp29_36*RLcp29_114;
  AxF6[3] = qdd[3]+OMcp29_114*ORcp29_215+OMcp29_115*ORcp29_216+OMcp29_16*ORcp29_214-OMcp29_214*ORcp29_115-OMcp29_215*
 ORcp29_116-OMcp29_26*ORcp29_114+OPcp29_114*RLcp29_215+OPcp29_115*RLcp29_216+OPcp29_16*RLcp29_214-OPcp29_214*RLcp29_115-
 OPcp29_215*RLcp29_116-OPcp29_26*RLcp29_114;
  OMPxF6[1] = OPcp29_115+qd[16]*(OMcp29_215*ROcp29_915-OMcp29_315*ROcp29_815)+qdd[16]*ROcp29_715;
  OMPxF6[2] = OPcp29_215-qd[16]*(OMcp29_115*ROcp29_915-OMcp29_315*ROcp29_715)+qdd[16]*ROcp29_815;
  OMPxF6[3] = OPcp29_315+qd[16]*(OMcp29_115*ROcp29_815-OMcp29_215*ROcp29_715)+qdd[16]*ROcp29_915;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc130 = ROcp29_116*SWr6[1]+ROcp29_216*SWr6[2]+ROcp29_316*SWr6[3];
  xfrc230 = ROcp29_416*SWr6[1]+ROcp29_516*SWr6[2]+ROcp29_616*SWr6[3];
  xfrc330 = ROcp29_715*SWr6[1]+ROcp29_815*SWr6[2]+ROcp29_915*SWr6[3];
  frc[1][16] = s->frc[1][16]+xfrc130;
  frc[2][16] = s->frc[2][16]+xfrc230;
  frc[3][16] = s->frc[3][16]+xfrc330;
  xtrq130 = ROcp29_116*SWr6[4]+ROcp29_216*SWr6[5]+ROcp29_316*SWr6[6];
  xtrq230 = ROcp29_416*SWr6[4]+ROcp29_516*SWr6[5]+ROcp29_616*SWr6[6];
  xtrq330 = ROcp29_715*SWr6[4]+ROcp29_815*SWr6[5]+ROcp29_915*SWr6[6];
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


  ROcp30_114 = ROcp30_16*C14-S14*S5;
  ROcp30_214 = ROcp30_26*C14-ROcp30_85*S14;
  ROcp30_314 = ROcp30_36*C14-ROcp30_95*S14;
  ROcp30_714 = ROcp30_16*S14+C14*S5;
  ROcp30_814 = ROcp30_26*S14+ROcp30_85*C14;
  ROcp30_914 = ROcp30_36*S14+ROcp30_95*C14;
  ROcp30_415 = ROcp30_46*C15+ROcp30_714*S15;
  ROcp30_515 = ROcp30_56*C15+ROcp30_814*S15;
  ROcp30_615 = ROcp30_66*C15+ROcp30_914*S15;
  ROcp30_715 = -(ROcp30_46*S15-ROcp30_714*C15);
  ROcp30_815 = -(ROcp30_56*S15-ROcp30_814*C15);
  ROcp30_915 = -(ROcp30_66*S15-ROcp30_914*C15);
  ROcp30_116 = ROcp30_114*C16+ROcp30_415*S16;
  ROcp30_216 = ROcp30_214*C16+ROcp30_515*S16;
  ROcp30_316 = ROcp30_314*C16+ROcp30_615*S16;
  ROcp30_416 = -(ROcp30_114*S16-ROcp30_415*C16);
  ROcp30_516 = -(ROcp30_214*S16-ROcp30_515*C16);
  ROcp30_616 = -(ROcp30_314*S16-ROcp30_615*C16);
  ROcp30_117 = ROcp30_116*C17-ROcp30_715*S17;
  ROcp30_217 = ROcp30_216*C17-ROcp30_815*S17;
  ROcp30_317 = ROcp30_316*C17-ROcp30_915*S17;
  ROcp30_717 = ROcp30_116*S17+ROcp30_715*C17;
  ROcp30_817 = ROcp30_216*S17+ROcp30_815*C17;
  ROcp30_917 = ROcp30_316*S17+ROcp30_915*C17;
  RLcp30_114 = ROcp30_46*s->dpt[2][3];
  RLcp30_214 = ROcp30_56*s->dpt[2][3];
  RLcp30_314 = ROcp30_66*s->dpt[2][3];
  OMcp30_114 = OMcp30_16+qd[14]*ROcp30_46;
  OMcp30_214 = OMcp30_26+qd[14]*ROcp30_56;
  OMcp30_314 = OMcp30_36+qd[14]*ROcp30_66;
  ORcp30_114 = OMcp30_26*RLcp30_314-OMcp30_36*RLcp30_214;
  ORcp30_214 = -(OMcp30_16*RLcp30_314-OMcp30_36*RLcp30_114);
  ORcp30_314 = OMcp30_16*RLcp30_214-OMcp30_26*RLcp30_114;
  OPcp30_114 = OPcp30_16+qd[14]*(OMcp30_26*ROcp30_66-OMcp30_36*ROcp30_56)+qdd[14]*ROcp30_46;
  OPcp30_214 = OPcp30_26-qd[14]*(OMcp30_16*ROcp30_66-OMcp30_36*ROcp30_46)+qdd[14]*ROcp30_56;
  OPcp30_314 = OPcp30_36+qd[14]*(OMcp30_16*ROcp30_56-OMcp30_26*ROcp30_46)+qdd[14]*ROcp30_66;
  RLcp30_115 = ROcp30_46*s->dpt[2][22];
  RLcp30_215 = ROcp30_56*s->dpt[2][22];
  RLcp30_315 = ROcp30_66*s->dpt[2][22];
  OMcp30_115 = OMcp30_114+qd[15]*ROcp30_114;
  OMcp30_215 = OMcp30_214+qd[15]*ROcp30_214;
  OMcp30_315 = OMcp30_314+qd[15]*ROcp30_314;
  ORcp30_115 = OMcp30_214*RLcp30_315-OMcp30_314*RLcp30_215;
  ORcp30_215 = -(OMcp30_114*RLcp30_315-OMcp30_314*RLcp30_115);
  ORcp30_315 = OMcp30_114*RLcp30_215-OMcp30_214*RLcp30_115;
  OPcp30_115 = OPcp30_114+qd[15]*(OMcp30_214*ROcp30_314-OMcp30_314*ROcp30_214)+qdd[15]*ROcp30_114;
  OPcp30_215 = OPcp30_214-qd[15]*(OMcp30_114*ROcp30_314-OMcp30_314*ROcp30_114)+qdd[15]*ROcp30_214;
  OPcp30_315 = OPcp30_314+qd[15]*(OMcp30_114*ROcp30_214-OMcp30_214*ROcp30_114)+qdd[15]*ROcp30_314;
  RLcp30_116 = ROcp30_715*s->dpt[3][24];
  RLcp30_216 = ROcp30_815*s->dpt[3][24];
  RLcp30_316 = ROcp30_915*s->dpt[3][24];
  OMcp30_116 = OMcp30_115+qd[16]*ROcp30_715;
  OMcp30_216 = OMcp30_215+qd[16]*ROcp30_815;
  OMcp30_316 = OMcp30_315+qd[16]*ROcp30_915;
  ORcp30_116 = OMcp30_215*RLcp30_316-OMcp30_315*RLcp30_216;
  ORcp30_216 = -(OMcp30_115*RLcp30_316-OMcp30_315*RLcp30_116);
  ORcp30_316 = OMcp30_115*RLcp30_216-OMcp30_215*RLcp30_116;
  OPcp30_116 = OPcp30_115+qd[16]*(OMcp30_215*ROcp30_915-OMcp30_315*ROcp30_815)+qdd[16]*ROcp30_715;
  OPcp30_216 = OPcp30_215-qd[16]*(OMcp30_115*ROcp30_915-OMcp30_315*ROcp30_715)+qdd[16]*ROcp30_815;
  OPcp30_316 = OPcp30_315+qd[16]*(OMcp30_115*ROcp30_815-OMcp30_215*ROcp30_715)+qdd[16]*ROcp30_915;
  RLcp30_117 = ROcp30_715*s->dpt[3][27];
  RLcp30_217 = ROcp30_815*s->dpt[3][27];
  RLcp30_317 = ROcp30_915*s->dpt[3][27];
  ORcp30_117 = OMcp30_216*RLcp30_317-OMcp30_316*RLcp30_217;
  ORcp30_217 = -(OMcp30_116*RLcp30_317-OMcp30_316*RLcp30_117);
  ORcp30_317 = OMcp30_116*RLcp30_217-OMcp30_216*RLcp30_117;
  PxF7[1] = q[1]+RLcp30_114+RLcp30_115+RLcp30_116+RLcp30_117;
  PxF7[2] = q[2]+RLcp30_214+RLcp30_215+RLcp30_216+RLcp30_217;
  PxF7[3] = q[3]+RLcp30_314+RLcp30_315+RLcp30_316+RLcp30_317;
  RxF7[1][1] = ROcp30_117;
  RxF7[1][2] = ROcp30_217;
  RxF7[1][3] = ROcp30_317;
  RxF7[2][1] = ROcp30_416;
  RxF7[2][2] = ROcp30_516;
  RxF7[2][3] = ROcp30_616;
  RxF7[3][1] = ROcp30_717;
  RxF7[3][2] = ROcp30_817;
  RxF7[3][3] = ROcp30_917;
  VxF7[1] = qd[1]+ORcp30_114+ORcp30_115+ORcp30_116+ORcp30_117;
  VxF7[2] = qd[2]+ORcp30_214+ORcp30_215+ORcp30_216+ORcp30_217;
  VxF7[3] = qd[3]+ORcp30_314+ORcp30_315+ORcp30_316+ORcp30_317;
  OMxF7[1] = OMcp30_116+qd[17]*ROcp30_416;
  OMxF7[2] = OMcp30_216+qd[17]*ROcp30_516;
  OMxF7[3] = OMcp30_316+qd[17]*ROcp30_616;
  AxF7[1] = qdd[1]+OMcp30_214*ORcp30_315+OMcp30_215*ORcp30_316+OMcp30_216*ORcp30_317+OMcp30_26*ORcp30_314-OMcp30_314*
 ORcp30_215-OMcp30_315*ORcp30_216-OMcp30_316*ORcp30_217-OMcp30_36*ORcp30_214+OPcp30_214*RLcp30_315+OPcp30_215*RLcp30_316+
 OPcp30_216*RLcp30_317+OPcp30_26*RLcp30_314-OPcp30_314*RLcp30_215-OPcp30_315*RLcp30_216-OPcp30_316*RLcp30_217-OPcp30_36*
 RLcp30_214;
  AxF7[2] = qdd[2]-OMcp30_114*ORcp30_315-OMcp30_115*ORcp30_316-OMcp30_116*ORcp30_317-OMcp30_16*ORcp30_314+OMcp30_314*
 ORcp30_115+OMcp30_315*ORcp30_116+OMcp30_316*ORcp30_117+OMcp30_36*ORcp30_114-OPcp30_114*RLcp30_315-OPcp30_115*RLcp30_316-
 OPcp30_116*RLcp30_317-OPcp30_16*RLcp30_314+OPcp30_314*RLcp30_115+OPcp30_315*RLcp30_116+OPcp30_316*RLcp30_117+OPcp30_36*
 RLcp30_114;
  AxF7[3] = qdd[3]+OMcp30_114*ORcp30_215+OMcp30_115*ORcp30_216+OMcp30_116*ORcp30_217+OMcp30_16*ORcp30_214-OMcp30_214*
 ORcp30_115-OMcp30_215*ORcp30_116-OMcp30_216*ORcp30_117-OMcp30_26*ORcp30_114+OPcp30_114*RLcp30_215+OPcp30_115*RLcp30_216+
 OPcp30_116*RLcp30_217+OPcp30_16*RLcp30_214-OPcp30_214*RLcp30_115-OPcp30_215*RLcp30_116-OPcp30_216*RLcp30_117-OPcp30_26*
 RLcp30_114;
  OMPxF7[1] = OPcp30_116+qd[17]*(OMcp30_216*ROcp30_616-OMcp30_316*ROcp30_516)+qdd[17]*ROcp30_416;
  OMPxF7[2] = OPcp30_216-qd[17]*(OMcp30_116*ROcp30_616-OMcp30_316*ROcp30_416)+qdd[17]*ROcp30_516;
  OMPxF7[3] = OPcp30_316+qd[17]*(OMcp30_116*ROcp30_516-OMcp30_216*ROcp30_416)+qdd[17]*ROcp30_616;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc131 = ROcp30_117*SWr7[1]+ROcp30_217*SWr7[2]+ROcp30_317*SWr7[3];
  xfrc231 = ROcp30_416*SWr7[1]+ROcp30_516*SWr7[2]+ROcp30_616*SWr7[3];
  xfrc331 = ROcp30_717*SWr7[1]+ROcp30_817*SWr7[2]+ROcp30_917*SWr7[3];
  frc[1][17] = s->frc[1][17]+xfrc131;
  frc[2][17] = s->frc[2][17]+xfrc231;
  frc[3][17] = s->frc[3][17]+xfrc331;
  xtrq131 = ROcp30_117*SWr7[4]+ROcp30_217*SWr7[5]+ROcp30_317*SWr7[6];
  xtrq231 = ROcp30_416*SWr7[4]+ROcp30_516*SWr7[5]+ROcp30_616*SWr7[6];
  xtrq331 = ROcp30_717*SWr7[4]+ROcp30_817*SWr7[5]+ROcp30_917*SWr7[6];
  trq[1][17] = s->trq[1][17]+xtrq131-xfrc231*(SWr7[9]-s->l[3][17])+xfrc331*(SWr7[8]-s->l[2][17]);
  trq[2][17] = s->trq[2][17]+xtrq231+xfrc131*(SWr7[9]-s->l[3][17])-xfrc331*(SWr7[7]-s->l[1][17]);
  trq[3][17] = s->trq[3][17]+xtrq331-xfrc131*(SWr7[8]-s->l[2][17])+xfrc231*(SWr7[7]-s->l[1][17]);

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

// = = Block_0_0_1_8_0_3 = = 
 
// Sensor Kinematics 


  ROcp31_114 = ROcp31_16*C14-S14*S5;
  ROcp31_214 = ROcp31_26*C14-ROcp31_85*S14;
  ROcp31_314 = ROcp31_36*C14-ROcp31_95*S14;
  ROcp31_714 = ROcp31_16*S14+C14*S5;
  ROcp31_814 = ROcp31_26*S14+ROcp31_85*C14;
  ROcp31_914 = ROcp31_36*S14+ROcp31_95*C14;
  ROcp31_415 = ROcp31_46*C15+ROcp31_714*S15;
  ROcp31_515 = ROcp31_56*C15+ROcp31_814*S15;
  ROcp31_615 = ROcp31_66*C15+ROcp31_914*S15;
  ROcp31_715 = -(ROcp31_46*S15-ROcp31_714*C15);
  ROcp31_815 = -(ROcp31_56*S15-ROcp31_814*C15);
  ROcp31_915 = -(ROcp31_66*S15-ROcp31_914*C15);
  ROcp31_116 = ROcp31_114*C16+ROcp31_415*S16;
  ROcp31_216 = ROcp31_214*C16+ROcp31_515*S16;
  ROcp31_316 = ROcp31_314*C16+ROcp31_615*S16;
  ROcp31_416 = -(ROcp31_114*S16-ROcp31_415*C16);
  ROcp31_516 = -(ROcp31_214*S16-ROcp31_515*C16);
  ROcp31_616 = -(ROcp31_314*S16-ROcp31_615*C16);
  ROcp31_117 = ROcp31_116*C17-ROcp31_715*S17;
  ROcp31_217 = ROcp31_216*C17-ROcp31_815*S17;
  ROcp31_317 = ROcp31_316*C17-ROcp31_915*S17;
  ROcp31_717 = ROcp31_116*S17+ROcp31_715*C17;
  ROcp31_817 = ROcp31_216*S17+ROcp31_815*C17;
  ROcp31_917 = ROcp31_316*S17+ROcp31_915*C17;
  ROcp31_418 = ROcp31_416*C18+ROcp31_717*S18;
  ROcp31_518 = ROcp31_516*C18+ROcp31_817*S18;
  ROcp31_618 = ROcp31_616*C18+ROcp31_917*S18;
  ROcp31_718 = -(ROcp31_416*S18-ROcp31_717*C18);
  ROcp31_818 = -(ROcp31_516*S18-ROcp31_817*C18);
  ROcp31_918 = -(ROcp31_616*S18-ROcp31_917*C18);
  ROcp31_119 = ROcp31_117*C19-ROcp31_718*S19;
  ROcp31_219 = ROcp31_217*C19-ROcp31_818*S19;
  ROcp31_319 = ROcp31_317*C19-ROcp31_918*S19;
  ROcp31_719 = ROcp31_117*S19+ROcp31_718*C19;
  ROcp31_819 = ROcp31_217*S19+ROcp31_818*C19;
  ROcp31_919 = ROcp31_317*S19+ROcp31_918*C19;
  RLcp31_114 = ROcp31_46*s->dpt[2][3];
  RLcp31_214 = ROcp31_56*s->dpt[2][3];
  RLcp31_314 = ROcp31_66*s->dpt[2][3];
  OMcp31_114 = OMcp31_16+qd[14]*ROcp31_46;
  OMcp31_214 = OMcp31_26+qd[14]*ROcp31_56;
  OMcp31_314 = OMcp31_36+qd[14]*ROcp31_66;
  ORcp31_114 = OMcp31_26*RLcp31_314-OMcp31_36*RLcp31_214;
  ORcp31_214 = -(OMcp31_16*RLcp31_314-OMcp31_36*RLcp31_114);
  ORcp31_314 = OMcp31_16*RLcp31_214-OMcp31_26*RLcp31_114;
  OPcp31_114 = OPcp31_16+qd[14]*(OMcp31_26*ROcp31_66-OMcp31_36*ROcp31_56)+qdd[14]*ROcp31_46;
  OPcp31_214 = OPcp31_26-qd[14]*(OMcp31_16*ROcp31_66-OMcp31_36*ROcp31_46)+qdd[14]*ROcp31_56;
  OPcp31_314 = OPcp31_36+qd[14]*(OMcp31_16*ROcp31_56-OMcp31_26*ROcp31_46)+qdd[14]*ROcp31_66;
  RLcp31_115 = ROcp31_46*s->dpt[2][22];
  RLcp31_215 = ROcp31_56*s->dpt[2][22];
  RLcp31_315 = ROcp31_66*s->dpt[2][22];
  OMcp31_115 = OMcp31_114+qd[15]*ROcp31_114;
  OMcp31_215 = OMcp31_214+qd[15]*ROcp31_214;
  OMcp31_315 = OMcp31_314+qd[15]*ROcp31_314;
  ORcp31_115 = OMcp31_214*RLcp31_315-OMcp31_314*RLcp31_215;
  ORcp31_215 = -(OMcp31_114*RLcp31_315-OMcp31_314*RLcp31_115);
  ORcp31_315 = OMcp31_114*RLcp31_215-OMcp31_214*RLcp31_115;
  OPcp31_115 = OPcp31_114+qd[15]*(OMcp31_214*ROcp31_314-OMcp31_314*ROcp31_214)+qdd[15]*ROcp31_114;
  OPcp31_215 = OPcp31_214-qd[15]*(OMcp31_114*ROcp31_314-OMcp31_314*ROcp31_114)+qdd[15]*ROcp31_214;
  OPcp31_315 = OPcp31_314+qd[15]*(OMcp31_114*ROcp31_214-OMcp31_214*ROcp31_114)+qdd[15]*ROcp31_314;
  RLcp31_116 = ROcp31_715*s->dpt[3][24];
  RLcp31_216 = ROcp31_815*s->dpt[3][24];
  RLcp31_316 = ROcp31_915*s->dpt[3][24];
  OMcp31_116 = OMcp31_115+qd[16]*ROcp31_715;
  OMcp31_216 = OMcp31_215+qd[16]*ROcp31_815;
  OMcp31_316 = OMcp31_315+qd[16]*ROcp31_915;
  ORcp31_116 = OMcp31_215*RLcp31_316-OMcp31_315*RLcp31_216;
  ORcp31_216 = -(OMcp31_115*RLcp31_316-OMcp31_315*RLcp31_116);
  ORcp31_316 = OMcp31_115*RLcp31_216-OMcp31_215*RLcp31_116;
  OPcp31_116 = OPcp31_115+qd[16]*(OMcp31_215*ROcp31_915-OMcp31_315*ROcp31_815)+qdd[16]*ROcp31_715;
  OPcp31_216 = OPcp31_215-qd[16]*(OMcp31_115*ROcp31_915-OMcp31_315*ROcp31_715)+qdd[16]*ROcp31_815;
  OPcp31_316 = OPcp31_315+qd[16]*(OMcp31_115*ROcp31_815-OMcp31_215*ROcp31_715)+qdd[16]*ROcp31_915;
  RLcp31_117 = ROcp31_715*s->dpt[3][27];
  RLcp31_217 = ROcp31_815*s->dpt[3][27];
  RLcp31_317 = ROcp31_915*s->dpt[3][27];
  OMcp31_117 = OMcp31_116+qd[17]*ROcp31_416;
  OMcp31_217 = OMcp31_216+qd[17]*ROcp31_516;
  OMcp31_317 = OMcp31_316+qd[17]*ROcp31_616;
  ORcp31_117 = OMcp31_216*RLcp31_317-OMcp31_316*RLcp31_217;
  ORcp31_217 = -(OMcp31_116*RLcp31_317-OMcp31_316*RLcp31_117);
  ORcp31_317 = OMcp31_116*RLcp31_217-OMcp31_216*RLcp31_117;
  OPcp31_117 = OPcp31_116+qd[17]*(OMcp31_216*ROcp31_616-OMcp31_316*ROcp31_516)+qdd[17]*ROcp31_416;
  OPcp31_217 = OPcp31_216-qd[17]*(OMcp31_116*ROcp31_616-OMcp31_316*ROcp31_416)+qdd[17]*ROcp31_516;
  OPcp31_317 = OPcp31_316+qd[17]*(OMcp31_116*ROcp31_516-OMcp31_216*ROcp31_416)+qdd[17]*ROcp31_616;
  RLcp31_118 = ROcp31_717*s->dpt[3][30];
  RLcp31_218 = ROcp31_817*s->dpt[3][30];
  RLcp31_318 = ROcp31_917*s->dpt[3][30];
  OMcp31_118 = OMcp31_117+qd[18]*ROcp31_117;
  OMcp31_218 = OMcp31_217+qd[18]*ROcp31_217;
  OMcp31_318 = OMcp31_317+qd[18]*ROcp31_317;
  ORcp31_118 = OMcp31_217*RLcp31_318-OMcp31_317*RLcp31_218;
  ORcp31_218 = -(OMcp31_117*RLcp31_318-OMcp31_317*RLcp31_118);
  ORcp31_318 = OMcp31_117*RLcp31_218-OMcp31_217*RLcp31_118;
  OMcp31_119 = OMcp31_118+qd[19]*ROcp31_418;
  OMcp31_219 = OMcp31_218+qd[19]*ROcp31_518;
  OMcp31_319 = OMcp31_318+qd[19]*ROcp31_618;
  OPcp31_119 = OPcp31_117+qd[18]*(OMcp31_217*ROcp31_317-OMcp31_317*ROcp31_217)+qd[19]*(OMcp31_218*ROcp31_618-OMcp31_318*
 ROcp31_518)+qdd[18]*ROcp31_117+qdd[19]*ROcp31_418;
  OPcp31_219 = OPcp31_217-qd[18]*(OMcp31_117*ROcp31_317-OMcp31_317*ROcp31_117)-qd[19]*(OMcp31_118*ROcp31_618-OMcp31_318*
 ROcp31_418)+qdd[18]*ROcp31_217+qdd[19]*ROcp31_518;
  OPcp31_319 = OPcp31_317+qd[18]*(OMcp31_117*ROcp31_217-OMcp31_217*ROcp31_117)+qd[19]*(OMcp31_118*ROcp31_518-OMcp31_218*
 ROcp31_418)+qdd[18]*ROcp31_317+qdd[19]*ROcp31_618;
  RLcp31_163 = ROcp31_719*s->dpt[3][35];
  RLcp31_263 = ROcp31_819*s->dpt[3][35];
  RLcp31_363 = ROcp31_919*s->dpt[3][35];
  ORcp31_163 = OMcp31_219*RLcp31_363-OMcp31_319*RLcp31_263;
  ORcp31_263 = -(OMcp31_119*RLcp31_363-OMcp31_319*RLcp31_163);
  ORcp31_363 = OMcp31_119*RLcp31_263-OMcp31_219*RLcp31_163;
  PxF8[1] = q[1]+RLcp31_114+RLcp31_115+RLcp31_116+RLcp31_117+RLcp31_118+RLcp31_163;
  PxF8[2] = q[2]+RLcp31_214+RLcp31_215+RLcp31_216+RLcp31_217+RLcp31_218+RLcp31_263;
  PxF8[3] = q[3]+RLcp31_314+RLcp31_315+RLcp31_316+RLcp31_317+RLcp31_318+RLcp31_363;
  RxF8[1][1] = ROcp31_119;
  RxF8[1][2] = ROcp31_219;
  RxF8[1][3] = ROcp31_319;
  RxF8[2][1] = ROcp31_418;
  RxF8[2][2] = ROcp31_518;
  RxF8[2][3] = ROcp31_618;
  RxF8[3][1] = ROcp31_719;
  RxF8[3][2] = ROcp31_819;
  RxF8[3][3] = ROcp31_919;
  VxF8[1] = qd[1]+ORcp31_114+ORcp31_115+ORcp31_116+ORcp31_117+ORcp31_118+ORcp31_163;
  VxF8[2] = qd[2]+ORcp31_214+ORcp31_215+ORcp31_216+ORcp31_217+ORcp31_218+ORcp31_263;
  VxF8[3] = qd[3]+ORcp31_314+ORcp31_315+ORcp31_316+ORcp31_317+ORcp31_318+ORcp31_363;
  OMxF8[1] = OMcp31_119;
  OMxF8[2] = OMcp31_219;
  OMxF8[3] = OMcp31_319;
  AxF8[1] = qdd[1]+OMcp31_214*ORcp31_315+OMcp31_215*ORcp31_316+OMcp31_216*ORcp31_317+OMcp31_217*ORcp31_318+OMcp31_219*
 ORcp31_363+OMcp31_26*ORcp31_314-OMcp31_314*ORcp31_215-OMcp31_315*ORcp31_216-OMcp31_316*ORcp31_217-OMcp31_317*ORcp31_218-
 OMcp31_319*ORcp31_263-OMcp31_36*ORcp31_214+OPcp31_214*RLcp31_315+OPcp31_215*RLcp31_316+OPcp31_216*RLcp31_317+OPcp31_217*
 RLcp31_318+OPcp31_219*RLcp31_363+OPcp31_26*RLcp31_314-OPcp31_314*RLcp31_215-OPcp31_315*RLcp31_216-OPcp31_316*RLcp31_217-
 OPcp31_317*RLcp31_218-OPcp31_319*RLcp31_263-OPcp31_36*RLcp31_214;
  AxF8[2] = qdd[2]-OMcp31_114*ORcp31_315-OMcp31_115*ORcp31_316-OMcp31_116*ORcp31_317-OMcp31_117*ORcp31_318-OMcp31_119*
 ORcp31_363-OMcp31_16*ORcp31_314+OMcp31_314*ORcp31_115+OMcp31_315*ORcp31_116+OMcp31_316*ORcp31_117+OMcp31_317*ORcp31_118+
 OMcp31_319*ORcp31_163+OMcp31_36*ORcp31_114-OPcp31_114*RLcp31_315-OPcp31_115*RLcp31_316-OPcp31_116*RLcp31_317-OPcp31_117*
 RLcp31_318-OPcp31_119*RLcp31_363-OPcp31_16*RLcp31_314+OPcp31_314*RLcp31_115+OPcp31_315*RLcp31_116+OPcp31_316*RLcp31_117+
 OPcp31_317*RLcp31_118+OPcp31_319*RLcp31_163+OPcp31_36*RLcp31_114;
  AxF8[3] = qdd[3]+OMcp31_114*ORcp31_215+OMcp31_115*ORcp31_216+OMcp31_116*ORcp31_217+OMcp31_117*ORcp31_218+OMcp31_119*
 ORcp31_263+OMcp31_16*ORcp31_214-OMcp31_214*ORcp31_115-OMcp31_215*ORcp31_116-OMcp31_216*ORcp31_117-OMcp31_217*ORcp31_118-
 OMcp31_219*ORcp31_163-OMcp31_26*ORcp31_114+OPcp31_114*RLcp31_215+OPcp31_115*RLcp31_216+OPcp31_116*RLcp31_217+OPcp31_117*
 RLcp31_218+OPcp31_119*RLcp31_263+OPcp31_16*RLcp31_214-OPcp31_214*RLcp31_115-OPcp31_215*RLcp31_116-OPcp31_216*RLcp31_117-
 OPcp31_217*RLcp31_118-OPcp31_219*RLcp31_163-OPcp31_26*RLcp31_114;
  OMPxF8[1] = OPcp31_119;
  OMPxF8[2] = OPcp31_219;
  OMPxF8[3] = OPcp31_319;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc132 = ROcp31_119*SWr8[1]+ROcp31_219*SWr8[2]+ROcp31_319*SWr8[3];
  xfrc232 = ROcp31_418*SWr8[1]+ROcp31_518*SWr8[2]+ROcp31_618*SWr8[3];
  xfrc332 = ROcp31_719*SWr8[1]+ROcp31_819*SWr8[2]+ROcp31_919*SWr8[3];
  frc[1][19] = s->frc[1][19]+xfrc132;
  frc[2][19] = s->frc[2][19]+xfrc232;
  frc[3][19] = s->frc[3][19]+xfrc332;
  xtrq132 = ROcp31_119*SWr8[4]+ROcp31_219*SWr8[5]+ROcp31_319*SWr8[6];
  xtrq232 = ROcp31_418*SWr8[4]+ROcp31_518*SWr8[5]+ROcp31_618*SWr8[6];
  xtrq332 = ROcp31_719*SWr8[4]+ROcp31_819*SWr8[5]+ROcp31_919*SWr8[6];
  trq[1][19] = s->trq[1][19]+xtrq132-xfrc232*(SWr8[9]-s->l[3][19])+xfrc332*SWr8[8];
  trq[2][19] = s->trq[2][19]+xtrq232+xfrc132*(SWr8[9]-s->l[3][19])-xfrc332*(SWr8[7]-s->l[1][19]);
  trq[3][19] = s->trq[3][19]+xtrq332-xfrc132*SWr8[8]+xfrc232*(SWr8[7]-s->l[1][19]);

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

// = = Block_0_0_1_9_0_3 = = 
 
// Sensor Kinematics 


  ROcp32_114 = ROcp32_16*C14-S14*S5;
  ROcp32_214 = ROcp32_26*C14-ROcp32_85*S14;
  ROcp32_314 = ROcp32_36*C14-ROcp32_95*S14;
  ROcp32_714 = ROcp32_16*S14+C14*S5;
  ROcp32_814 = ROcp32_26*S14+ROcp32_85*C14;
  ROcp32_914 = ROcp32_36*S14+ROcp32_95*C14;
  ROcp32_415 = ROcp32_46*C15+ROcp32_714*S15;
  ROcp32_515 = ROcp32_56*C15+ROcp32_814*S15;
  ROcp32_615 = ROcp32_66*C15+ROcp32_914*S15;
  ROcp32_715 = -(ROcp32_46*S15-ROcp32_714*C15);
  ROcp32_815 = -(ROcp32_56*S15-ROcp32_814*C15);
  ROcp32_915 = -(ROcp32_66*S15-ROcp32_914*C15);
  ROcp32_116 = ROcp32_114*C16+ROcp32_415*S16;
  ROcp32_216 = ROcp32_214*C16+ROcp32_515*S16;
  ROcp32_316 = ROcp32_314*C16+ROcp32_615*S16;
  ROcp32_416 = -(ROcp32_114*S16-ROcp32_415*C16);
  ROcp32_516 = -(ROcp32_214*S16-ROcp32_515*C16);
  ROcp32_616 = -(ROcp32_314*S16-ROcp32_615*C16);
  ROcp32_117 = ROcp32_116*C17-ROcp32_715*S17;
  ROcp32_217 = ROcp32_216*C17-ROcp32_815*S17;
  ROcp32_317 = ROcp32_316*C17-ROcp32_915*S17;
  ROcp32_717 = ROcp32_116*S17+ROcp32_715*C17;
  ROcp32_817 = ROcp32_216*S17+ROcp32_815*C17;
  ROcp32_917 = ROcp32_316*S17+ROcp32_915*C17;
  ROcp32_418 = ROcp32_416*C18+ROcp32_717*S18;
  ROcp32_518 = ROcp32_516*C18+ROcp32_817*S18;
  ROcp32_618 = ROcp32_616*C18+ROcp32_917*S18;
  ROcp32_718 = -(ROcp32_416*S18-ROcp32_717*C18);
  ROcp32_818 = -(ROcp32_516*S18-ROcp32_817*C18);
  ROcp32_918 = -(ROcp32_616*S18-ROcp32_917*C18);
  ROcp32_119 = ROcp32_117*C19-ROcp32_718*S19;
  ROcp32_219 = ROcp32_217*C19-ROcp32_818*S19;
  ROcp32_319 = ROcp32_317*C19-ROcp32_918*S19;
  ROcp32_719 = ROcp32_117*S19+ROcp32_718*C19;
  ROcp32_819 = ROcp32_217*S19+ROcp32_818*C19;
  ROcp32_919 = ROcp32_317*S19+ROcp32_918*C19;
  ROcp32_120 = ROcp32_119*C20-ROcp32_719*S20;
  ROcp32_220 = ROcp32_219*C20-ROcp32_819*S20;
  ROcp32_320 = ROcp32_319*C20-ROcp32_919*S20;
  ROcp32_720 = ROcp32_119*S20+ROcp32_719*C20;
  ROcp32_820 = ROcp32_219*S20+ROcp32_819*C20;
  ROcp32_920 = ROcp32_319*S20+ROcp32_919*C20;
  RLcp32_114 = ROcp32_46*s->dpt[2][3];
  RLcp32_214 = ROcp32_56*s->dpt[2][3];
  RLcp32_314 = ROcp32_66*s->dpt[2][3];
  OMcp32_114 = OMcp32_16+qd[14]*ROcp32_46;
  OMcp32_214 = OMcp32_26+qd[14]*ROcp32_56;
  OMcp32_314 = OMcp32_36+qd[14]*ROcp32_66;
  ORcp32_114 = OMcp32_26*RLcp32_314-OMcp32_36*RLcp32_214;
  ORcp32_214 = -(OMcp32_16*RLcp32_314-OMcp32_36*RLcp32_114);
  ORcp32_314 = OMcp32_16*RLcp32_214-OMcp32_26*RLcp32_114;
  OPcp32_114 = OPcp32_16+qd[14]*(OMcp32_26*ROcp32_66-OMcp32_36*ROcp32_56)+qdd[14]*ROcp32_46;
  OPcp32_214 = OPcp32_26-qd[14]*(OMcp32_16*ROcp32_66-OMcp32_36*ROcp32_46)+qdd[14]*ROcp32_56;
  OPcp32_314 = OPcp32_36+qd[14]*(OMcp32_16*ROcp32_56-OMcp32_26*ROcp32_46)+qdd[14]*ROcp32_66;
  RLcp32_115 = ROcp32_46*s->dpt[2][22];
  RLcp32_215 = ROcp32_56*s->dpt[2][22];
  RLcp32_315 = ROcp32_66*s->dpt[2][22];
  OMcp32_115 = OMcp32_114+qd[15]*ROcp32_114;
  OMcp32_215 = OMcp32_214+qd[15]*ROcp32_214;
  OMcp32_315 = OMcp32_314+qd[15]*ROcp32_314;
  ORcp32_115 = OMcp32_214*RLcp32_315-OMcp32_314*RLcp32_215;
  ORcp32_215 = -(OMcp32_114*RLcp32_315-OMcp32_314*RLcp32_115);
  ORcp32_315 = OMcp32_114*RLcp32_215-OMcp32_214*RLcp32_115;
  OPcp32_115 = OPcp32_114+qd[15]*(OMcp32_214*ROcp32_314-OMcp32_314*ROcp32_214)+qdd[15]*ROcp32_114;
  OPcp32_215 = OPcp32_214-qd[15]*(OMcp32_114*ROcp32_314-OMcp32_314*ROcp32_114)+qdd[15]*ROcp32_214;
  OPcp32_315 = OPcp32_314+qd[15]*(OMcp32_114*ROcp32_214-OMcp32_214*ROcp32_114)+qdd[15]*ROcp32_314;
  RLcp32_116 = ROcp32_715*s->dpt[3][24];
  RLcp32_216 = ROcp32_815*s->dpt[3][24];
  RLcp32_316 = ROcp32_915*s->dpt[3][24];
  OMcp32_116 = OMcp32_115+qd[16]*ROcp32_715;
  OMcp32_216 = OMcp32_215+qd[16]*ROcp32_815;
  OMcp32_316 = OMcp32_315+qd[16]*ROcp32_915;
  ORcp32_116 = OMcp32_215*RLcp32_316-OMcp32_315*RLcp32_216;
  ORcp32_216 = -(OMcp32_115*RLcp32_316-OMcp32_315*RLcp32_116);
  ORcp32_316 = OMcp32_115*RLcp32_216-OMcp32_215*RLcp32_116;
  OPcp32_116 = OPcp32_115+qd[16]*(OMcp32_215*ROcp32_915-OMcp32_315*ROcp32_815)+qdd[16]*ROcp32_715;
  OPcp32_216 = OPcp32_215-qd[16]*(OMcp32_115*ROcp32_915-OMcp32_315*ROcp32_715)+qdd[16]*ROcp32_815;
  OPcp32_316 = OPcp32_315+qd[16]*(OMcp32_115*ROcp32_815-OMcp32_215*ROcp32_715)+qdd[16]*ROcp32_915;
  RLcp32_117 = ROcp32_715*s->dpt[3][27];
  RLcp32_217 = ROcp32_815*s->dpt[3][27];
  RLcp32_317 = ROcp32_915*s->dpt[3][27];
  OMcp32_117 = OMcp32_116+qd[17]*ROcp32_416;
  OMcp32_217 = OMcp32_216+qd[17]*ROcp32_516;
  OMcp32_317 = OMcp32_316+qd[17]*ROcp32_616;
  ORcp32_117 = OMcp32_216*RLcp32_317-OMcp32_316*RLcp32_217;
  ORcp32_217 = -(OMcp32_116*RLcp32_317-OMcp32_316*RLcp32_117);
  ORcp32_317 = OMcp32_116*RLcp32_217-OMcp32_216*RLcp32_117;
  OPcp32_117 = OPcp32_116+qd[17]*(OMcp32_216*ROcp32_616-OMcp32_316*ROcp32_516)+qdd[17]*ROcp32_416;
  OPcp32_217 = OPcp32_216-qd[17]*(OMcp32_116*ROcp32_616-OMcp32_316*ROcp32_416)+qdd[17]*ROcp32_516;
  OPcp32_317 = OPcp32_316+qd[17]*(OMcp32_116*ROcp32_516-OMcp32_216*ROcp32_416)+qdd[17]*ROcp32_616;
  RLcp32_118 = ROcp32_717*s->dpt[3][30];
  RLcp32_218 = ROcp32_817*s->dpt[3][30];
  RLcp32_318 = ROcp32_917*s->dpt[3][30];
  OMcp32_118 = OMcp32_117+qd[18]*ROcp32_117;
  OMcp32_218 = OMcp32_217+qd[18]*ROcp32_217;
  OMcp32_318 = OMcp32_317+qd[18]*ROcp32_317;
  ORcp32_118 = OMcp32_217*RLcp32_318-OMcp32_317*RLcp32_218;
  ORcp32_218 = -(OMcp32_117*RLcp32_318-OMcp32_317*RLcp32_118);
  ORcp32_318 = OMcp32_117*RLcp32_218-OMcp32_217*RLcp32_118;
  OMcp32_119 = OMcp32_118+qd[19]*ROcp32_418;
  OMcp32_219 = OMcp32_218+qd[19]*ROcp32_518;
  OMcp32_319 = OMcp32_318+qd[19]*ROcp32_618;
  OPcp32_119 = OPcp32_117+qd[18]*(OMcp32_217*ROcp32_317-OMcp32_317*ROcp32_217)+qd[19]*(OMcp32_218*ROcp32_618-OMcp32_318*
 ROcp32_518)+qdd[18]*ROcp32_117+qdd[19]*ROcp32_418;
  OPcp32_219 = OPcp32_217-qd[18]*(OMcp32_117*ROcp32_317-OMcp32_317*ROcp32_117)-qd[19]*(OMcp32_118*ROcp32_618-OMcp32_318*
 ROcp32_418)+qdd[18]*ROcp32_217+qdd[19]*ROcp32_518;
  OPcp32_319 = OPcp32_317+qd[18]*(OMcp32_117*ROcp32_217-OMcp32_217*ROcp32_117)+qd[19]*(OMcp32_118*ROcp32_518-OMcp32_218*
 ROcp32_418)+qdd[18]*ROcp32_317+qdd[19]*ROcp32_618;
  RLcp32_120 = ROcp32_119*s->dpt[1][36]+ROcp32_719*s->dpt[3][36];
  RLcp32_220 = ROcp32_219*s->dpt[1][36]+ROcp32_819*s->dpt[3][36];
  RLcp32_320 = ROcp32_319*s->dpt[1][36]+ROcp32_919*s->dpt[3][36];
  ORcp32_120 = OMcp32_219*RLcp32_320-OMcp32_319*RLcp32_220;
  ORcp32_220 = -(OMcp32_119*RLcp32_320-OMcp32_319*RLcp32_120);
  ORcp32_320 = OMcp32_119*RLcp32_220-OMcp32_219*RLcp32_120;
  PxF9[1] = q[1]+RLcp32_114+RLcp32_115+RLcp32_116+RLcp32_117+RLcp32_118+RLcp32_120;
  PxF9[2] = q[2]+RLcp32_214+RLcp32_215+RLcp32_216+RLcp32_217+RLcp32_218+RLcp32_220;
  PxF9[3] = q[3]+RLcp32_314+RLcp32_315+RLcp32_316+RLcp32_317+RLcp32_318+RLcp32_320;
  RxF9[1][1] = ROcp32_120;
  RxF9[1][2] = ROcp32_220;
  RxF9[1][3] = ROcp32_320;
  RxF9[2][1] = ROcp32_418;
  RxF9[2][2] = ROcp32_518;
  RxF9[2][3] = ROcp32_618;
  RxF9[3][1] = ROcp32_720;
  RxF9[3][2] = ROcp32_820;
  RxF9[3][3] = ROcp32_920;
  VxF9[1] = qd[1]+ORcp32_114+ORcp32_115+ORcp32_116+ORcp32_117+ORcp32_118+ORcp32_120;
  VxF9[2] = qd[2]+ORcp32_214+ORcp32_215+ORcp32_216+ORcp32_217+ORcp32_218+ORcp32_220;
  VxF9[3] = qd[3]+ORcp32_314+ORcp32_315+ORcp32_316+ORcp32_317+ORcp32_318+ORcp32_320;
  OMxF9[1] = OMcp32_119+qd[20]*ROcp32_418;
  OMxF9[2] = OMcp32_219+qd[20]*ROcp32_518;
  OMxF9[3] = OMcp32_319+qd[20]*ROcp32_618;
  AxF9[1] = qdd[1]+OMcp32_214*ORcp32_315+OMcp32_215*ORcp32_316+OMcp32_216*ORcp32_317+OMcp32_217*ORcp32_318+OMcp32_219*
 ORcp32_320+OMcp32_26*ORcp32_314-OMcp32_314*ORcp32_215-OMcp32_315*ORcp32_216-OMcp32_316*ORcp32_217-OMcp32_317*ORcp32_218-
 OMcp32_319*ORcp32_220-OMcp32_36*ORcp32_214+OPcp32_214*RLcp32_315+OPcp32_215*RLcp32_316+OPcp32_216*RLcp32_317+OPcp32_217*
 RLcp32_318+OPcp32_219*RLcp32_320+OPcp32_26*RLcp32_314-OPcp32_314*RLcp32_215-OPcp32_315*RLcp32_216-OPcp32_316*RLcp32_217-
 OPcp32_317*RLcp32_218-OPcp32_319*RLcp32_220-OPcp32_36*RLcp32_214;
  AxF9[2] = qdd[2]-OMcp32_114*ORcp32_315-OMcp32_115*ORcp32_316-OMcp32_116*ORcp32_317-OMcp32_117*ORcp32_318-OMcp32_119*
 ORcp32_320-OMcp32_16*ORcp32_314+OMcp32_314*ORcp32_115+OMcp32_315*ORcp32_116+OMcp32_316*ORcp32_117+OMcp32_317*ORcp32_118+
 OMcp32_319*ORcp32_120+OMcp32_36*ORcp32_114-OPcp32_114*RLcp32_315-OPcp32_115*RLcp32_316-OPcp32_116*RLcp32_317-OPcp32_117*
 RLcp32_318-OPcp32_119*RLcp32_320-OPcp32_16*RLcp32_314+OPcp32_314*RLcp32_115+OPcp32_315*RLcp32_116+OPcp32_316*RLcp32_117+
 OPcp32_317*RLcp32_118+OPcp32_319*RLcp32_120+OPcp32_36*RLcp32_114;
  AxF9[3] = qdd[3]+OMcp32_114*ORcp32_215+OMcp32_115*ORcp32_216+OMcp32_116*ORcp32_217+OMcp32_117*ORcp32_218+OMcp32_119*
 ORcp32_220+OMcp32_16*ORcp32_214-OMcp32_214*ORcp32_115-OMcp32_215*ORcp32_116-OMcp32_216*ORcp32_117-OMcp32_217*ORcp32_118-
 OMcp32_219*ORcp32_120-OMcp32_26*ORcp32_114+OPcp32_114*RLcp32_215+OPcp32_115*RLcp32_216+OPcp32_116*RLcp32_217+OPcp32_117*
 RLcp32_218+OPcp32_119*RLcp32_220+OPcp32_16*RLcp32_214-OPcp32_214*RLcp32_115-OPcp32_215*RLcp32_116-OPcp32_216*RLcp32_117-
 OPcp32_217*RLcp32_118-OPcp32_219*RLcp32_120-OPcp32_26*RLcp32_114;
  OMPxF9[1] = OPcp32_119+qd[20]*(OMcp32_219*ROcp32_618-OMcp32_319*ROcp32_518)+qdd[20]*ROcp32_418;
  OMPxF9[2] = OPcp32_219-qd[20]*(OMcp32_119*ROcp32_618-OMcp32_319*ROcp32_418)+qdd[20]*ROcp32_518;
  OMPxF9[3] = OPcp32_319+qd[20]*(OMcp32_119*ROcp32_518-OMcp32_219*ROcp32_418)+qdd[20]*ROcp32_618;
 
// Sensor Forces Computation 

  SWr9 = user_ExtForces(PxF9,RxF9,VxF9,OMxF9,AxF9,OMPxF9,s,tsim,9);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc133 = ROcp32_120*SWr9[1]+ROcp32_220*SWr9[2]+ROcp32_320*SWr9[3];
  xfrc233 = ROcp32_418*SWr9[1]+ROcp32_518*SWr9[2]+ROcp32_618*SWr9[3];
  xfrc333 = ROcp32_720*SWr9[1]+ROcp32_820*SWr9[2]+ROcp32_920*SWr9[3];
  frc[1][20] = s->frc[1][20]+xfrc133;
  frc[2][20] = s->frc[2][20]+xfrc233;
  frc[3][20] = s->frc[3][20]+xfrc333;
  xtrq133 = ROcp32_120*SWr9[4]+ROcp32_220*SWr9[5]+ROcp32_320*SWr9[6];
  xtrq233 = ROcp32_418*SWr9[4]+ROcp32_518*SWr9[5]+ROcp32_618*SWr9[6];
  xtrq333 = ROcp32_720*SWr9[4]+ROcp32_820*SWr9[5]+ROcp32_920*SWr9[6];
  trq[1][20] = s->trq[1][20]+xtrq133-xfrc233*(SWr9[9]-s->l[3][20])+xfrc333*SWr9[8];
  trq[2][20] = s->trq[2][20]+xtrq233+xfrc133*(SWr9[9]-s->l[3][20])-xfrc333*(SWr9[7]-s->l[1][20]);
  trq[3][20] = s->trq[3][20]+xtrq333-xfrc133*SWr9[8]+xfrc233*(SWr9[7]-s->l[1][20]);

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


  ROcp33_421 = ROcp33_46*C21+S21*S5;
  ROcp33_521 = ROcp33_56*C21+ROcp33_85*S21;
  ROcp33_621 = ROcp33_66*C21+ROcp33_95*S21;
  ROcp33_721 = -(ROcp33_46*S21-C21*S5);
  ROcp33_821 = -(ROcp33_56*S21-ROcp33_85*C21);
  ROcp33_921 = -(ROcp33_66*S21-ROcp33_95*C21);
  ROcp33_122 = ROcp33_16*C22-ROcp33_721*S22;
  ROcp33_222 = ROcp33_26*C22-ROcp33_821*S22;
  ROcp33_322 = ROcp33_36*C22-ROcp33_921*S22;
  ROcp33_722 = ROcp33_16*S22+ROcp33_721*C22;
  ROcp33_822 = ROcp33_26*S22+ROcp33_821*C22;
  ROcp33_922 = ROcp33_36*S22+ROcp33_921*C22;
  ROcp33_123 = ROcp33_122*C23+ROcp33_421*S23;
  ROcp33_223 = ROcp33_222*C23+ROcp33_521*S23;
  ROcp33_323 = ROcp33_322*C23+ROcp33_621*S23;
  ROcp33_423 = -(ROcp33_122*S23-ROcp33_421*C23);
  ROcp33_523 = -(ROcp33_222*S23-ROcp33_521*C23);
  ROcp33_623 = -(ROcp33_322*S23-ROcp33_621*C23);
  RLcp33_121 = ROcp33_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp33_221 = ROcp33_26*s->dpt[1][4]+ROcp33_85*s->dpt[3][4];
  RLcp33_321 = ROcp33_36*s->dpt[1][4]+ROcp33_95*s->dpt[3][4];
  OMcp33_121 = OMcp33_16+qd[21]*ROcp33_16;
  OMcp33_221 = OMcp33_26+qd[21]*ROcp33_26;
  OMcp33_321 = OMcp33_36+qd[21]*ROcp33_36;
  ORcp33_121 = OMcp33_26*RLcp33_321-OMcp33_36*RLcp33_221;
  ORcp33_221 = -(OMcp33_16*RLcp33_321-OMcp33_36*RLcp33_121);
  ORcp33_321 = OMcp33_16*RLcp33_221-OMcp33_26*RLcp33_121;
  OMcp33_122 = OMcp33_121+qd[22]*ROcp33_421;
  OMcp33_222 = OMcp33_221+qd[22]*ROcp33_521;
  OMcp33_322 = OMcp33_321+qd[22]*ROcp33_621;
  OPcp33_122 = OPcp33_16+qd[21]*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26)+qd[22]*(OMcp33_221*ROcp33_621-OMcp33_321*
 ROcp33_521)+qdd[21]*ROcp33_16+qdd[22]*ROcp33_421;
  OPcp33_222 = OPcp33_26-qd[21]*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16)-qd[22]*(OMcp33_121*ROcp33_621-OMcp33_321*
 ROcp33_421)+qdd[21]*ROcp33_26+qdd[22]*ROcp33_521;
  OPcp33_322 = OPcp33_36+qd[21]*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16)+qd[22]*(OMcp33_121*ROcp33_521-OMcp33_221*
 ROcp33_421)+qdd[21]*ROcp33_36+qdd[22]*ROcp33_621;
  RLcp33_123 = ROcp33_722*s->dpt[3][40];
  RLcp33_223 = ROcp33_822*s->dpt[3][40];
  RLcp33_323 = ROcp33_922*s->dpt[3][40];
  ORcp33_123 = OMcp33_222*RLcp33_323-OMcp33_322*RLcp33_223;
  ORcp33_223 = -(OMcp33_122*RLcp33_323-OMcp33_322*RLcp33_123);
  ORcp33_323 = OMcp33_122*RLcp33_223-OMcp33_222*RLcp33_123;
  PxF10[1] = q[1]+RLcp33_121+RLcp33_123;
  PxF10[2] = q[2]+RLcp33_221+RLcp33_223;
  PxF10[3] = q[3]+RLcp33_321+RLcp33_323;
  RxF10[1][1] = ROcp33_123;
  RxF10[1][2] = ROcp33_223;
  RxF10[1][3] = ROcp33_323;
  RxF10[2][1] = ROcp33_423;
  RxF10[2][2] = ROcp33_523;
  RxF10[2][3] = ROcp33_623;
  RxF10[3][1] = ROcp33_722;
  RxF10[3][2] = ROcp33_822;
  RxF10[3][3] = ROcp33_922;
  VxF10[1] = qd[1]+ORcp33_121+ORcp33_123;
  VxF10[2] = qd[2]+ORcp33_221+ORcp33_223;
  VxF10[3] = qd[3]+ORcp33_321+ORcp33_323;
  OMxF10[1] = OMcp33_122+qd[23]*ROcp33_722;
  OMxF10[2] = OMcp33_222+qd[23]*ROcp33_822;
  OMxF10[3] = OMcp33_322+qd[23]*ROcp33_922;
  AxF10[1] = qdd[1]+OMcp33_222*ORcp33_323+OMcp33_26*ORcp33_321-OMcp33_322*ORcp33_223-OMcp33_36*ORcp33_221+OPcp33_222*
 RLcp33_323+OPcp33_26*RLcp33_321-OPcp33_322*RLcp33_223-OPcp33_36*RLcp33_221;
  AxF10[2] = qdd[2]-OMcp33_122*ORcp33_323-OMcp33_16*ORcp33_321+OMcp33_322*ORcp33_123+OMcp33_36*ORcp33_121-OPcp33_122*
 RLcp33_323-OPcp33_16*RLcp33_321+OPcp33_322*RLcp33_123+OPcp33_36*RLcp33_121;
  AxF10[3] = qdd[3]+OMcp33_122*ORcp33_223+OMcp33_16*ORcp33_221-OMcp33_222*ORcp33_123-OMcp33_26*ORcp33_121+OPcp33_122*
 RLcp33_223+OPcp33_16*RLcp33_221-OPcp33_222*RLcp33_123-OPcp33_26*RLcp33_121;
  OMPxF10[1] = OPcp33_122+qd[23]*(OMcp33_222*ROcp33_922-OMcp33_322*ROcp33_822)+qdd[23]*ROcp33_722;
  OMPxF10[2] = OPcp33_222-qd[23]*(OMcp33_122*ROcp33_922-OMcp33_322*ROcp33_722)+qdd[23]*ROcp33_822;
  OMPxF10[3] = OPcp33_322+qd[23]*(OMcp33_122*ROcp33_822-OMcp33_222*ROcp33_722)+qdd[23]*ROcp33_922;
 
// Sensor Forces Computation 

  SWr10 = user_ExtForces(PxF10,RxF10,VxF10,OMxF10,AxF10,OMPxF10,s,tsim,10);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc134 = ROcp33_123*SWr10[1]+ROcp33_223*SWr10[2]+ROcp33_323*SWr10[3];
  xfrc234 = ROcp33_423*SWr10[1]+ROcp33_523*SWr10[2]+ROcp33_623*SWr10[3];
  xfrc334 = ROcp33_722*SWr10[1]+ROcp33_822*SWr10[2]+ROcp33_922*SWr10[3];
  frc[1][23] = s->frc[1][23]+xfrc134;
  frc[2][23] = s->frc[2][23]+xfrc234;
  frc[3][23] = s->frc[3][23]+xfrc334;
  xtrq134 = ROcp33_123*SWr10[4]+ROcp33_223*SWr10[5]+ROcp33_323*SWr10[6];
  xtrq234 = ROcp33_423*SWr10[4]+ROcp33_523*SWr10[5]+ROcp33_623*SWr10[6];
  xtrq334 = ROcp33_722*SWr10[4]+ROcp33_822*SWr10[5]+ROcp33_922*SWr10[6];
  trq[1][23] = s->trq[1][23]+xtrq134-xfrc234*(SWr10[9]-s->l[3][23])+xfrc334*(SWr10[8]-s->l[2][23]);
  trq[2][23] = s->trq[2][23]+xtrq234+xfrc134*(SWr10[9]-s->l[3][23])-xfrc334*(SWr10[7]-s->l[1][23]);
  trq[3][23] = s->trq[3][23]+xtrq334-xfrc134*(SWr10[8]-s->l[2][23])+xfrc234*(SWr10[7]-s->l[1][23]);

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


  ROcp34_421 = ROcp34_46*C21+S21*S5;
  ROcp34_521 = ROcp34_56*C21+ROcp34_85*S21;
  ROcp34_621 = ROcp34_66*C21+ROcp34_95*S21;
  ROcp34_721 = -(ROcp34_46*S21-C21*S5);
  ROcp34_821 = -(ROcp34_56*S21-ROcp34_85*C21);
  ROcp34_921 = -(ROcp34_66*S21-ROcp34_95*C21);
  ROcp34_122 = ROcp34_16*C22-ROcp34_721*S22;
  ROcp34_222 = ROcp34_26*C22-ROcp34_821*S22;
  ROcp34_322 = ROcp34_36*C22-ROcp34_921*S22;
  ROcp34_722 = ROcp34_16*S22+ROcp34_721*C22;
  ROcp34_822 = ROcp34_26*S22+ROcp34_821*C22;
  ROcp34_922 = ROcp34_36*S22+ROcp34_921*C22;
  ROcp34_123 = ROcp34_122*C23+ROcp34_421*S23;
  ROcp34_223 = ROcp34_222*C23+ROcp34_521*S23;
  ROcp34_323 = ROcp34_322*C23+ROcp34_621*S23;
  ROcp34_423 = -(ROcp34_122*S23-ROcp34_421*C23);
  ROcp34_523 = -(ROcp34_222*S23-ROcp34_521*C23);
  ROcp34_623 = -(ROcp34_322*S23-ROcp34_621*C23);
  RLcp34_121 = ROcp34_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp34_221 = ROcp34_26*s->dpt[1][4]+ROcp34_85*s->dpt[3][4];
  RLcp34_321 = ROcp34_36*s->dpt[1][4]+ROcp34_95*s->dpt[3][4];
  OMcp34_121 = OMcp34_16+qd[21]*ROcp34_16;
  OMcp34_221 = OMcp34_26+qd[21]*ROcp34_26;
  OMcp34_321 = OMcp34_36+qd[21]*ROcp34_36;
  ORcp34_121 = OMcp34_26*RLcp34_321-OMcp34_36*RLcp34_221;
  ORcp34_221 = -(OMcp34_16*RLcp34_321-OMcp34_36*RLcp34_121);
  ORcp34_321 = OMcp34_16*RLcp34_221-OMcp34_26*RLcp34_121;
  OMcp34_122 = OMcp34_121+qd[22]*ROcp34_421;
  OMcp34_222 = OMcp34_221+qd[22]*ROcp34_521;
  OMcp34_322 = OMcp34_321+qd[22]*ROcp34_621;
  OPcp34_122 = OPcp34_16+qd[21]*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26)+qd[22]*(OMcp34_221*ROcp34_621-OMcp34_321*
 ROcp34_521)+qdd[21]*ROcp34_16+qdd[22]*ROcp34_421;
  OPcp34_222 = OPcp34_26-qd[21]*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16)-qd[22]*(OMcp34_121*ROcp34_621-OMcp34_321*
 ROcp34_421)+qdd[21]*ROcp34_26+qdd[22]*ROcp34_521;
  OPcp34_322 = OPcp34_36+qd[21]*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16)+qd[22]*(OMcp34_121*ROcp34_521-OMcp34_221*
 ROcp34_421)+qdd[21]*ROcp34_36+qdd[22]*ROcp34_621;
  RLcp34_123 = ROcp34_722*s->dpt[3][40];
  RLcp34_223 = ROcp34_822*s->dpt[3][40];
  RLcp34_323 = ROcp34_922*s->dpt[3][40];
  OMcp34_123 = OMcp34_122+qd[23]*ROcp34_722;
  OMcp34_223 = OMcp34_222+qd[23]*ROcp34_822;
  OMcp34_323 = OMcp34_322+qd[23]*ROcp34_922;
  ORcp34_123 = OMcp34_222*RLcp34_323-OMcp34_322*RLcp34_223;
  ORcp34_223 = -(OMcp34_122*RLcp34_323-OMcp34_322*RLcp34_123);
  ORcp34_323 = OMcp34_122*RLcp34_223-OMcp34_222*RLcp34_123;
  OPcp34_123 = OPcp34_122+qd[23]*(OMcp34_222*ROcp34_922-OMcp34_322*ROcp34_822)+qdd[23]*ROcp34_722;
  OPcp34_223 = OPcp34_222-qd[23]*(OMcp34_122*ROcp34_922-OMcp34_322*ROcp34_722)+qdd[23]*ROcp34_822;
  OPcp34_323 = OPcp34_322+qd[23]*(OMcp34_122*ROcp34_822-OMcp34_222*ROcp34_722)+qdd[23]*ROcp34_922;

// = = Block_0_0_1_11_0_5 = = 
 
// Sensor Kinematics 


  ROcp34_124 = ROcp34_123*C24-ROcp34_722*S24;
  ROcp34_224 = ROcp34_223*C24-ROcp34_822*S24;
  ROcp34_324 = ROcp34_323*C24-ROcp34_922*S24;
  ROcp34_724 = ROcp34_123*S24+ROcp34_722*C24;
  ROcp34_824 = ROcp34_223*S24+ROcp34_822*C24;
  ROcp34_924 = ROcp34_323*S24+ROcp34_922*C24;
  ROcp34_425 = ROcp34_423*C25+ROcp34_724*S25;
  ROcp34_525 = ROcp34_523*C25+ROcp34_824*S25;
  ROcp34_625 = ROcp34_623*C25+ROcp34_924*S25;
  ROcp34_725 = -(ROcp34_423*S25-ROcp34_724*C25);
  ROcp34_825 = -(ROcp34_523*S25-ROcp34_824*C25);
  ROcp34_925 = -(ROcp34_623*S25-ROcp34_924*C25);
  ROcp34_126 = ROcp34_124*C26+ROcp34_425*S26;
  ROcp34_226 = ROcp34_224*C26+ROcp34_525*S26;
  ROcp34_326 = ROcp34_324*C26+ROcp34_625*S26;
  ROcp34_426 = -(ROcp34_124*S26-ROcp34_425*C26);
  ROcp34_526 = -(ROcp34_224*S26-ROcp34_525*C26);
  ROcp34_626 = -(ROcp34_324*S26-ROcp34_625*C26);
  RLcp34_124 = ROcp34_123*s->dpt[1][44]+ROcp34_423*s->dpt[2][44]+ROcp34_722*s->dpt[3][44];
  RLcp34_224 = ROcp34_223*s->dpt[1][44]+ROcp34_523*s->dpt[2][44]+ROcp34_822*s->dpt[3][44];
  RLcp34_324 = ROcp34_323*s->dpt[1][44]+ROcp34_623*s->dpt[2][44]+ROcp34_922*s->dpt[3][44];
  OMcp34_124 = OMcp34_123+qd[24]*ROcp34_423;
  OMcp34_224 = OMcp34_223+qd[24]*ROcp34_523;
  OMcp34_324 = OMcp34_323+qd[24]*ROcp34_623;
  ORcp34_124 = OMcp34_223*RLcp34_324-OMcp34_323*RLcp34_224;
  ORcp34_224 = -(OMcp34_123*RLcp34_324-OMcp34_323*RLcp34_124);
  ORcp34_324 = OMcp34_123*RLcp34_224-OMcp34_223*RLcp34_124;
  OPcp34_124 = OPcp34_123+qd[24]*(OMcp34_223*ROcp34_623-OMcp34_323*ROcp34_523)+qdd[24]*ROcp34_423;
  OPcp34_224 = OPcp34_223-qd[24]*(OMcp34_123*ROcp34_623-OMcp34_323*ROcp34_423)+qdd[24]*ROcp34_523;
  OPcp34_324 = OPcp34_323+qd[24]*(OMcp34_123*ROcp34_523-OMcp34_223*ROcp34_423)+qdd[24]*ROcp34_623;
  RLcp34_125 = ROcp34_423*s->dpt[2][47];
  RLcp34_225 = ROcp34_523*s->dpt[2][47];
  RLcp34_325 = ROcp34_623*s->dpt[2][47];
  OMcp34_125 = OMcp34_124+qd[25]*ROcp34_124;
  OMcp34_225 = OMcp34_224+qd[25]*ROcp34_224;
  OMcp34_325 = OMcp34_324+qd[25]*ROcp34_324;
  ORcp34_125 = OMcp34_224*RLcp34_325-OMcp34_324*RLcp34_225;
  ORcp34_225 = -(OMcp34_124*RLcp34_325-OMcp34_324*RLcp34_125);
  ORcp34_325 = OMcp34_124*RLcp34_225-OMcp34_224*RLcp34_125;
  OPcp34_125 = OPcp34_124+qd[25]*(OMcp34_224*ROcp34_324-OMcp34_324*ROcp34_224)+qdd[25]*ROcp34_124;
  OPcp34_225 = OPcp34_224-qd[25]*(OMcp34_124*ROcp34_324-OMcp34_324*ROcp34_124)+qdd[25]*ROcp34_224;
  OPcp34_325 = OPcp34_324+qd[25]*(OMcp34_124*ROcp34_224-OMcp34_224*ROcp34_124)+qdd[25]*ROcp34_324;
  RLcp34_126 = ROcp34_725*s->dpt[3][49];
  RLcp34_226 = ROcp34_825*s->dpt[3][49];
  RLcp34_326 = ROcp34_925*s->dpt[3][49];
  ORcp34_126 = OMcp34_225*RLcp34_326-OMcp34_325*RLcp34_226;
  ORcp34_226 = -(OMcp34_125*RLcp34_326-OMcp34_325*RLcp34_126);
  ORcp34_326 = OMcp34_125*RLcp34_226-OMcp34_225*RLcp34_126;
  PxF11[1] = q[1]+RLcp34_121+RLcp34_123+RLcp34_124+RLcp34_125+RLcp34_126;
  PxF11[2] = q[2]+RLcp34_221+RLcp34_223+RLcp34_224+RLcp34_225+RLcp34_226;
  PxF11[3] = q[3]+RLcp34_321+RLcp34_323+RLcp34_324+RLcp34_325+RLcp34_326;
  RxF11[1][1] = ROcp34_126;
  RxF11[1][2] = ROcp34_226;
  RxF11[1][3] = ROcp34_326;
  RxF11[2][1] = ROcp34_426;
  RxF11[2][2] = ROcp34_526;
  RxF11[2][3] = ROcp34_626;
  RxF11[3][1] = ROcp34_725;
  RxF11[3][2] = ROcp34_825;
  RxF11[3][3] = ROcp34_925;
  VxF11[1] = qd[1]+ORcp34_121+ORcp34_123+ORcp34_124+ORcp34_125+ORcp34_126;
  VxF11[2] = qd[2]+ORcp34_221+ORcp34_223+ORcp34_224+ORcp34_225+ORcp34_226;
  VxF11[3] = qd[3]+ORcp34_321+ORcp34_323+ORcp34_324+ORcp34_325+ORcp34_326;
  OMxF11[1] = OMcp34_125+qd[26]*ROcp34_725;
  OMxF11[2] = OMcp34_225+qd[26]*ROcp34_825;
  OMxF11[3] = OMcp34_325+qd[26]*ROcp34_925;
  AxF11[1] = qdd[1]+OMcp34_222*ORcp34_323+OMcp34_223*ORcp34_324+OMcp34_224*ORcp34_325+OMcp34_225*ORcp34_326+OMcp34_26*
 ORcp34_321-OMcp34_322*ORcp34_223-OMcp34_323*ORcp34_224-OMcp34_324*ORcp34_225-OMcp34_325*ORcp34_226-OMcp34_36*ORcp34_221+
 OPcp34_222*RLcp34_323+OPcp34_223*RLcp34_324+OPcp34_224*RLcp34_325+OPcp34_225*RLcp34_326+OPcp34_26*RLcp34_321-OPcp34_322*
 RLcp34_223-OPcp34_323*RLcp34_224-OPcp34_324*RLcp34_225-OPcp34_325*RLcp34_226-OPcp34_36*RLcp34_221;
  AxF11[2] = qdd[2]-OMcp34_122*ORcp34_323-OMcp34_123*ORcp34_324-OMcp34_124*ORcp34_325-OMcp34_125*ORcp34_326-OMcp34_16*
 ORcp34_321+OMcp34_322*ORcp34_123+OMcp34_323*ORcp34_124+OMcp34_324*ORcp34_125+OMcp34_325*ORcp34_126+OMcp34_36*ORcp34_121-
 OPcp34_122*RLcp34_323-OPcp34_123*RLcp34_324-OPcp34_124*RLcp34_325-OPcp34_125*RLcp34_326-OPcp34_16*RLcp34_321+OPcp34_322*
 RLcp34_123+OPcp34_323*RLcp34_124+OPcp34_324*RLcp34_125+OPcp34_325*RLcp34_126+OPcp34_36*RLcp34_121;
  AxF11[3] = qdd[3]+OMcp34_122*ORcp34_223+OMcp34_123*ORcp34_224+OMcp34_124*ORcp34_225+OMcp34_125*ORcp34_226+OMcp34_16*
 ORcp34_221-OMcp34_222*ORcp34_123-OMcp34_223*ORcp34_124-OMcp34_224*ORcp34_125-OMcp34_225*ORcp34_126-OMcp34_26*ORcp34_121+
 OPcp34_122*RLcp34_223+OPcp34_123*RLcp34_224+OPcp34_124*RLcp34_225+OPcp34_125*RLcp34_226+OPcp34_16*RLcp34_221-OPcp34_222*
 RLcp34_123-OPcp34_223*RLcp34_124-OPcp34_224*RLcp34_125-OPcp34_225*RLcp34_126-OPcp34_26*RLcp34_121;
  OMPxF11[1] = OPcp34_125+qd[26]*(OMcp34_225*ROcp34_925-OMcp34_325*ROcp34_825)+qdd[26]*ROcp34_725;
  OMPxF11[2] = OPcp34_225-qd[26]*(OMcp34_125*ROcp34_925-OMcp34_325*ROcp34_725)+qdd[26]*ROcp34_825;
  OMPxF11[3] = OPcp34_325+qd[26]*(OMcp34_125*ROcp34_825-OMcp34_225*ROcp34_725)+qdd[26]*ROcp34_925;
 
// Sensor Forces Computation 

  SWr11 = user_ExtForces(PxF11,RxF11,VxF11,OMxF11,AxF11,OMPxF11,s,tsim,11);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc135 = ROcp34_126*SWr11[1]+ROcp34_226*SWr11[2]+ROcp34_326*SWr11[3];
  xfrc235 = ROcp34_426*SWr11[1]+ROcp34_526*SWr11[2]+ROcp34_626*SWr11[3];
  xfrc335 = ROcp34_725*SWr11[1]+ROcp34_825*SWr11[2]+ROcp34_925*SWr11[3];
  frc[1][26] = s->frc[1][26]+xfrc135;
  frc[2][26] = s->frc[2][26]+xfrc235;
  frc[3][26] = s->frc[3][26]+xfrc335;
  xtrq135 = ROcp34_126*SWr11[4]+ROcp34_226*SWr11[5]+ROcp34_326*SWr11[6];
  xtrq235 = ROcp34_426*SWr11[4]+ROcp34_526*SWr11[5]+ROcp34_626*SWr11[6];
  xtrq335 = ROcp34_725*SWr11[4]+ROcp34_825*SWr11[5]+ROcp34_925*SWr11[6];
  trq[1][26] = s->trq[1][26]+xtrq135-xfrc235*(SWr11[9]-s->l[3][26])+xfrc335*(SWr11[8]-s->l[2][26]);
  trq[2][26] = s->trq[2][26]+xtrq235+xfrc135*(SWr11[9]-s->l[3][26])-xfrc335*(SWr11[7]-s->l[1][26]);
  trq[3][26] = s->trq[3][26]+xtrq335-xfrc135*(SWr11[8]-s->l[2][26])+xfrc235*(SWr11[7]-s->l[1][26]);

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


  ROcp35_421 = ROcp35_46*C21+S21*S5;
  ROcp35_521 = ROcp35_56*C21+ROcp35_85*S21;
  ROcp35_621 = ROcp35_66*C21+ROcp35_95*S21;
  ROcp35_721 = -(ROcp35_46*S21-C21*S5);
  ROcp35_821 = -(ROcp35_56*S21-ROcp35_85*C21);
  ROcp35_921 = -(ROcp35_66*S21-ROcp35_95*C21);
  ROcp35_122 = ROcp35_16*C22-ROcp35_721*S22;
  ROcp35_222 = ROcp35_26*C22-ROcp35_821*S22;
  ROcp35_322 = ROcp35_36*C22-ROcp35_921*S22;
  ROcp35_722 = ROcp35_16*S22+ROcp35_721*C22;
  ROcp35_822 = ROcp35_26*S22+ROcp35_821*C22;
  ROcp35_922 = ROcp35_36*S22+ROcp35_921*C22;
  ROcp35_123 = ROcp35_122*C23+ROcp35_421*S23;
  ROcp35_223 = ROcp35_222*C23+ROcp35_521*S23;
  ROcp35_323 = ROcp35_322*C23+ROcp35_621*S23;
  ROcp35_423 = -(ROcp35_122*S23-ROcp35_421*C23);
  ROcp35_523 = -(ROcp35_222*S23-ROcp35_521*C23);
  ROcp35_623 = -(ROcp35_322*S23-ROcp35_621*C23);
  RLcp35_121 = ROcp35_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp35_221 = ROcp35_26*s->dpt[1][4]+ROcp35_85*s->dpt[3][4];
  RLcp35_321 = ROcp35_36*s->dpt[1][4]+ROcp35_95*s->dpt[3][4];
  OMcp35_121 = OMcp35_16+qd[21]*ROcp35_16;
  OMcp35_221 = OMcp35_26+qd[21]*ROcp35_26;
  OMcp35_321 = OMcp35_36+qd[21]*ROcp35_36;
  ORcp35_121 = OMcp35_26*RLcp35_321-OMcp35_36*RLcp35_221;
  ORcp35_221 = -(OMcp35_16*RLcp35_321-OMcp35_36*RLcp35_121);
  ORcp35_321 = OMcp35_16*RLcp35_221-OMcp35_26*RLcp35_121;
  OMcp35_122 = OMcp35_121+qd[22]*ROcp35_421;
  OMcp35_222 = OMcp35_221+qd[22]*ROcp35_521;
  OMcp35_322 = OMcp35_321+qd[22]*ROcp35_621;
  OPcp35_122 = OPcp35_16+qd[21]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26)+qd[22]*(OMcp35_221*ROcp35_621-OMcp35_321*
 ROcp35_521)+qdd[21]*ROcp35_16+qdd[22]*ROcp35_421;
  OPcp35_222 = OPcp35_26-qd[21]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16)-qd[22]*(OMcp35_121*ROcp35_621-OMcp35_321*
 ROcp35_421)+qdd[21]*ROcp35_26+qdd[22]*ROcp35_521;
  OPcp35_322 = OPcp35_36+qd[21]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16)+qd[22]*(OMcp35_121*ROcp35_521-OMcp35_221*
 ROcp35_421)+qdd[21]*ROcp35_36+qdd[22]*ROcp35_621;
  RLcp35_123 = ROcp35_722*s->dpt[3][40];
  RLcp35_223 = ROcp35_822*s->dpt[3][40];
  RLcp35_323 = ROcp35_922*s->dpt[3][40];
  OMcp35_123 = OMcp35_122+qd[23]*ROcp35_722;
  OMcp35_223 = OMcp35_222+qd[23]*ROcp35_822;
  OMcp35_323 = OMcp35_322+qd[23]*ROcp35_922;
  ORcp35_123 = OMcp35_222*RLcp35_323-OMcp35_322*RLcp35_223;
  ORcp35_223 = -(OMcp35_122*RLcp35_323-OMcp35_322*RLcp35_123);
  ORcp35_323 = OMcp35_122*RLcp35_223-OMcp35_222*RLcp35_123;
  OPcp35_123 = OPcp35_122+qd[23]*(OMcp35_222*ROcp35_922-OMcp35_322*ROcp35_822)+qdd[23]*ROcp35_722;
  OPcp35_223 = OPcp35_222-qd[23]*(OMcp35_122*ROcp35_922-OMcp35_322*ROcp35_722)+qdd[23]*ROcp35_822;
  OPcp35_323 = OPcp35_322+qd[23]*(OMcp35_122*ROcp35_822-OMcp35_222*ROcp35_722)+qdd[23]*ROcp35_922;

// = = Block_0_0_1_12_0_5 = = 
 
// Sensor Kinematics 


  ROcp35_124 = ROcp35_123*C24-ROcp35_722*S24;
  ROcp35_224 = ROcp35_223*C24-ROcp35_822*S24;
  ROcp35_324 = ROcp35_323*C24-ROcp35_922*S24;
  ROcp35_724 = ROcp35_123*S24+ROcp35_722*C24;
  ROcp35_824 = ROcp35_223*S24+ROcp35_822*C24;
  ROcp35_924 = ROcp35_323*S24+ROcp35_922*C24;
  ROcp35_425 = ROcp35_423*C25+ROcp35_724*S25;
  ROcp35_525 = ROcp35_523*C25+ROcp35_824*S25;
  ROcp35_625 = ROcp35_623*C25+ROcp35_924*S25;
  ROcp35_725 = -(ROcp35_423*S25-ROcp35_724*C25);
  ROcp35_825 = -(ROcp35_523*S25-ROcp35_824*C25);
  ROcp35_925 = -(ROcp35_623*S25-ROcp35_924*C25);
  ROcp35_126 = ROcp35_124*C26+ROcp35_425*S26;
  ROcp35_226 = ROcp35_224*C26+ROcp35_525*S26;
  ROcp35_326 = ROcp35_324*C26+ROcp35_625*S26;
  ROcp35_426 = -(ROcp35_124*S26-ROcp35_425*C26);
  ROcp35_526 = -(ROcp35_224*S26-ROcp35_525*C26);
  ROcp35_626 = -(ROcp35_324*S26-ROcp35_625*C26);
  ROcp35_127 = ROcp35_126*C27-ROcp35_725*S27;
  ROcp35_227 = ROcp35_226*C27-ROcp35_825*S27;
  ROcp35_327 = ROcp35_326*C27-ROcp35_925*S27;
  ROcp35_727 = ROcp35_126*S27+ROcp35_725*C27;
  ROcp35_827 = ROcp35_226*S27+ROcp35_825*C27;
  ROcp35_927 = ROcp35_326*S27+ROcp35_925*C27;
  RLcp35_124 = ROcp35_123*s->dpt[1][44]+ROcp35_423*s->dpt[2][44]+ROcp35_722*s->dpt[3][44];
  RLcp35_224 = ROcp35_223*s->dpt[1][44]+ROcp35_523*s->dpt[2][44]+ROcp35_822*s->dpt[3][44];
  RLcp35_324 = ROcp35_323*s->dpt[1][44]+ROcp35_623*s->dpt[2][44]+ROcp35_922*s->dpt[3][44];
  OMcp35_124 = OMcp35_123+qd[24]*ROcp35_423;
  OMcp35_224 = OMcp35_223+qd[24]*ROcp35_523;
  OMcp35_324 = OMcp35_323+qd[24]*ROcp35_623;
  ORcp35_124 = OMcp35_223*RLcp35_324-OMcp35_323*RLcp35_224;
  ORcp35_224 = -(OMcp35_123*RLcp35_324-OMcp35_323*RLcp35_124);
  ORcp35_324 = OMcp35_123*RLcp35_224-OMcp35_223*RLcp35_124;
  OPcp35_124 = OPcp35_123+qd[24]*(OMcp35_223*ROcp35_623-OMcp35_323*ROcp35_523)+qdd[24]*ROcp35_423;
  OPcp35_224 = OPcp35_223-qd[24]*(OMcp35_123*ROcp35_623-OMcp35_323*ROcp35_423)+qdd[24]*ROcp35_523;
  OPcp35_324 = OPcp35_323+qd[24]*(OMcp35_123*ROcp35_523-OMcp35_223*ROcp35_423)+qdd[24]*ROcp35_623;
  RLcp35_125 = ROcp35_423*s->dpt[2][47];
  RLcp35_225 = ROcp35_523*s->dpt[2][47];
  RLcp35_325 = ROcp35_623*s->dpt[2][47];
  OMcp35_125 = OMcp35_124+qd[25]*ROcp35_124;
  OMcp35_225 = OMcp35_224+qd[25]*ROcp35_224;
  OMcp35_325 = OMcp35_324+qd[25]*ROcp35_324;
  ORcp35_125 = OMcp35_224*RLcp35_325-OMcp35_324*RLcp35_225;
  ORcp35_225 = -(OMcp35_124*RLcp35_325-OMcp35_324*RLcp35_125);
  ORcp35_325 = OMcp35_124*RLcp35_225-OMcp35_224*RLcp35_125;
  OPcp35_125 = OPcp35_124+qd[25]*(OMcp35_224*ROcp35_324-OMcp35_324*ROcp35_224)+qdd[25]*ROcp35_124;
  OPcp35_225 = OPcp35_224-qd[25]*(OMcp35_124*ROcp35_324-OMcp35_324*ROcp35_124)+qdd[25]*ROcp35_224;
  OPcp35_325 = OPcp35_324+qd[25]*(OMcp35_124*ROcp35_224-OMcp35_224*ROcp35_124)+qdd[25]*ROcp35_324;
  RLcp35_126 = ROcp35_725*s->dpt[3][49];
  RLcp35_226 = ROcp35_825*s->dpt[3][49];
  RLcp35_326 = ROcp35_925*s->dpt[3][49];
  OMcp35_126 = OMcp35_125+qd[26]*ROcp35_725;
  OMcp35_226 = OMcp35_225+qd[26]*ROcp35_825;
  OMcp35_326 = OMcp35_325+qd[26]*ROcp35_925;
  ORcp35_126 = OMcp35_225*RLcp35_326-OMcp35_325*RLcp35_226;
  ORcp35_226 = -(OMcp35_125*RLcp35_326-OMcp35_325*RLcp35_126);
  ORcp35_326 = OMcp35_125*RLcp35_226-OMcp35_225*RLcp35_126;
  OPcp35_126 = OPcp35_125+qd[26]*(OMcp35_225*ROcp35_925-OMcp35_325*ROcp35_825)+qdd[26]*ROcp35_725;
  OPcp35_226 = OPcp35_225-qd[26]*(OMcp35_125*ROcp35_925-OMcp35_325*ROcp35_725)+qdd[26]*ROcp35_825;
  OPcp35_326 = OPcp35_325+qd[26]*(OMcp35_125*ROcp35_825-OMcp35_225*ROcp35_725)+qdd[26]*ROcp35_925;
  RLcp35_127 = ROcp35_725*s->dpt[3][52];
  RLcp35_227 = ROcp35_825*s->dpt[3][52];
  RLcp35_327 = ROcp35_925*s->dpt[3][52];
  ORcp35_127 = OMcp35_226*RLcp35_327-OMcp35_326*RLcp35_227;
  ORcp35_227 = -(OMcp35_126*RLcp35_327-OMcp35_326*RLcp35_127);
  ORcp35_327 = OMcp35_126*RLcp35_227-OMcp35_226*RLcp35_127;
  PxF12[1] = q[1]+RLcp35_121+RLcp35_123+RLcp35_124+RLcp35_125+RLcp35_126+RLcp35_127;
  PxF12[2] = q[2]+RLcp35_221+RLcp35_223+RLcp35_224+RLcp35_225+RLcp35_226+RLcp35_227;
  PxF12[3] = q[3]+RLcp35_321+RLcp35_323+RLcp35_324+RLcp35_325+RLcp35_326+RLcp35_327;
  RxF12[1][1] = ROcp35_127;
  RxF12[1][2] = ROcp35_227;
  RxF12[1][3] = ROcp35_327;
  RxF12[2][1] = ROcp35_426;
  RxF12[2][2] = ROcp35_526;
  RxF12[2][3] = ROcp35_626;
  RxF12[3][1] = ROcp35_727;
  RxF12[3][2] = ROcp35_827;
  RxF12[3][3] = ROcp35_927;
  VxF12[1] = qd[1]+ORcp35_121+ORcp35_123+ORcp35_124+ORcp35_125+ORcp35_126+ORcp35_127;
  VxF12[2] = qd[2]+ORcp35_221+ORcp35_223+ORcp35_224+ORcp35_225+ORcp35_226+ORcp35_227;
  VxF12[3] = qd[3]+ORcp35_321+ORcp35_323+ORcp35_324+ORcp35_325+ORcp35_326+ORcp35_327;
  OMxF12[1] = OMcp35_126+qd[27]*ROcp35_426;
  OMxF12[2] = OMcp35_226+qd[27]*ROcp35_526;
  OMxF12[3] = OMcp35_326+qd[27]*ROcp35_626;
  AxF12[1] = qdd[1]+OMcp35_222*ORcp35_323+OMcp35_223*ORcp35_324+OMcp35_224*ORcp35_325+OMcp35_225*ORcp35_326+OMcp35_226*
 ORcp35_327+OMcp35_26*ORcp35_321-OMcp35_322*ORcp35_223-OMcp35_323*ORcp35_224-OMcp35_324*ORcp35_225-OMcp35_325*ORcp35_226-
 OMcp35_326*ORcp35_227-OMcp35_36*ORcp35_221+OPcp35_222*RLcp35_323+OPcp35_223*RLcp35_324+OPcp35_224*RLcp35_325+OPcp35_225*
 RLcp35_326+OPcp35_226*RLcp35_327+OPcp35_26*RLcp35_321-OPcp35_322*RLcp35_223-OPcp35_323*RLcp35_224-OPcp35_324*RLcp35_225-
 OPcp35_325*RLcp35_226-OPcp35_326*RLcp35_227-OPcp35_36*RLcp35_221;
  AxF12[2] = qdd[2]-OMcp35_122*ORcp35_323-OMcp35_123*ORcp35_324-OMcp35_124*ORcp35_325-OMcp35_125*ORcp35_326-OMcp35_126*
 ORcp35_327-OMcp35_16*ORcp35_321+OMcp35_322*ORcp35_123+OMcp35_323*ORcp35_124+OMcp35_324*ORcp35_125+OMcp35_325*ORcp35_126+
 OMcp35_326*ORcp35_127+OMcp35_36*ORcp35_121-OPcp35_122*RLcp35_323-OPcp35_123*RLcp35_324-OPcp35_124*RLcp35_325-OPcp35_125*
 RLcp35_326-OPcp35_126*RLcp35_327-OPcp35_16*RLcp35_321+OPcp35_322*RLcp35_123+OPcp35_323*RLcp35_124+OPcp35_324*RLcp35_125+
 OPcp35_325*RLcp35_126+OPcp35_326*RLcp35_127+OPcp35_36*RLcp35_121;
  AxF12[3] = qdd[3]+OMcp35_122*ORcp35_223+OMcp35_123*ORcp35_224+OMcp35_124*ORcp35_225+OMcp35_125*ORcp35_226+OMcp35_126*
 ORcp35_227+OMcp35_16*ORcp35_221-OMcp35_222*ORcp35_123-OMcp35_223*ORcp35_124-OMcp35_224*ORcp35_125-OMcp35_225*ORcp35_126-
 OMcp35_226*ORcp35_127-OMcp35_26*ORcp35_121+OPcp35_122*RLcp35_223+OPcp35_123*RLcp35_224+OPcp35_124*RLcp35_225+OPcp35_125*
 RLcp35_226+OPcp35_126*RLcp35_227+OPcp35_16*RLcp35_221-OPcp35_222*RLcp35_123-OPcp35_223*RLcp35_124-OPcp35_224*RLcp35_125-
 OPcp35_225*RLcp35_126-OPcp35_226*RLcp35_127-OPcp35_26*RLcp35_121;
  OMPxF12[1] = OPcp35_126+qd[27]*(OMcp35_226*ROcp35_626-OMcp35_326*ROcp35_526)+qdd[27]*ROcp35_426;
  OMPxF12[2] = OPcp35_226-qd[27]*(OMcp35_126*ROcp35_626-OMcp35_326*ROcp35_426)+qdd[27]*ROcp35_526;
  OMPxF12[3] = OPcp35_326+qd[27]*(OMcp35_126*ROcp35_526-OMcp35_226*ROcp35_426)+qdd[27]*ROcp35_626;
 
// Sensor Forces Computation 

  SWr12 = user_ExtForces(PxF12,RxF12,VxF12,OMxF12,AxF12,OMPxF12,s,tsim,12);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc136 = ROcp35_127*SWr12[1]+ROcp35_227*SWr12[2]+ROcp35_327*SWr12[3];
  xfrc236 = ROcp35_426*SWr12[1]+ROcp35_526*SWr12[2]+ROcp35_626*SWr12[3];
  xfrc336 = ROcp35_727*SWr12[1]+ROcp35_827*SWr12[2]+ROcp35_927*SWr12[3];
  frc[1][27] = s->frc[1][27]+xfrc136;
  frc[2][27] = s->frc[2][27]+xfrc236;
  frc[3][27] = s->frc[3][27]+xfrc336;
  xtrq136 = ROcp35_127*SWr12[4]+ROcp35_227*SWr12[5]+ROcp35_327*SWr12[6];
  xtrq236 = ROcp35_426*SWr12[4]+ROcp35_526*SWr12[5]+ROcp35_626*SWr12[6];
  xtrq336 = ROcp35_727*SWr12[4]+ROcp35_827*SWr12[5]+ROcp35_927*SWr12[6];
  trq[1][27] = s->trq[1][27]+xtrq136-xfrc236*(SWr12[9]-s->l[3][27])+xfrc336*(SWr12[8]-s->l[2][27]);
  trq[2][27] = s->trq[2][27]+xtrq236+xfrc136*(SWr12[9]-s->l[3][27])-xfrc336*(SWr12[7]-s->l[1][27]);
  trq[3][27] = s->trq[3][27]+xtrq336-xfrc136*(SWr12[8]-s->l[2][27])+xfrc236*(SWr12[7]-s->l[1][27]);

// = = Block_0_0_1_13_0_1 = = 
 
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

// = = Block_0_0_1_13_0_4 = = 
 
// Sensor Kinematics 


  ROcp36_421 = ROcp36_46*C21+S21*S5;
  ROcp36_521 = ROcp36_56*C21+ROcp36_85*S21;
  ROcp36_621 = ROcp36_66*C21+ROcp36_95*S21;
  ROcp36_721 = -(ROcp36_46*S21-C21*S5);
  ROcp36_821 = -(ROcp36_56*S21-ROcp36_85*C21);
  ROcp36_921 = -(ROcp36_66*S21-ROcp36_95*C21);
  ROcp36_122 = ROcp36_16*C22-ROcp36_721*S22;
  ROcp36_222 = ROcp36_26*C22-ROcp36_821*S22;
  ROcp36_322 = ROcp36_36*C22-ROcp36_921*S22;
  ROcp36_722 = ROcp36_16*S22+ROcp36_721*C22;
  ROcp36_822 = ROcp36_26*S22+ROcp36_821*C22;
  ROcp36_922 = ROcp36_36*S22+ROcp36_921*C22;
  ROcp36_123 = ROcp36_122*C23+ROcp36_421*S23;
  ROcp36_223 = ROcp36_222*C23+ROcp36_521*S23;
  ROcp36_323 = ROcp36_322*C23+ROcp36_621*S23;
  ROcp36_423 = -(ROcp36_122*S23-ROcp36_421*C23);
  ROcp36_523 = -(ROcp36_222*S23-ROcp36_521*C23);
  ROcp36_623 = -(ROcp36_322*S23-ROcp36_621*C23);
  RLcp36_121 = ROcp36_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp36_221 = ROcp36_26*s->dpt[1][4]+ROcp36_85*s->dpt[3][4];
  RLcp36_321 = ROcp36_36*s->dpt[1][4]+ROcp36_95*s->dpt[3][4];
  OMcp36_121 = OMcp36_16+qd[21]*ROcp36_16;
  OMcp36_221 = OMcp36_26+qd[21]*ROcp36_26;
  OMcp36_321 = OMcp36_36+qd[21]*ROcp36_36;
  ORcp36_121 = OMcp36_26*RLcp36_321-OMcp36_36*RLcp36_221;
  ORcp36_221 = -(OMcp36_16*RLcp36_321-OMcp36_36*RLcp36_121);
  ORcp36_321 = OMcp36_16*RLcp36_221-OMcp36_26*RLcp36_121;
  OMcp36_122 = OMcp36_121+qd[22]*ROcp36_421;
  OMcp36_222 = OMcp36_221+qd[22]*ROcp36_521;
  OMcp36_322 = OMcp36_321+qd[22]*ROcp36_621;
  OPcp36_122 = OPcp36_16+qd[21]*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26)+qd[22]*(OMcp36_221*ROcp36_621-OMcp36_321*
 ROcp36_521)+qdd[21]*ROcp36_16+qdd[22]*ROcp36_421;
  OPcp36_222 = OPcp36_26-qd[21]*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16)-qd[22]*(OMcp36_121*ROcp36_621-OMcp36_321*
 ROcp36_421)+qdd[21]*ROcp36_26+qdd[22]*ROcp36_521;
  OPcp36_322 = OPcp36_36+qd[21]*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16)+qd[22]*(OMcp36_121*ROcp36_521-OMcp36_221*
 ROcp36_421)+qdd[21]*ROcp36_36+qdd[22]*ROcp36_621;
  RLcp36_123 = ROcp36_722*s->dpt[3][40];
  RLcp36_223 = ROcp36_822*s->dpt[3][40];
  RLcp36_323 = ROcp36_922*s->dpt[3][40];
  OMcp36_123 = OMcp36_122+qd[23]*ROcp36_722;
  OMcp36_223 = OMcp36_222+qd[23]*ROcp36_822;
  OMcp36_323 = OMcp36_322+qd[23]*ROcp36_922;
  ORcp36_123 = OMcp36_222*RLcp36_323-OMcp36_322*RLcp36_223;
  ORcp36_223 = -(OMcp36_122*RLcp36_323-OMcp36_322*RLcp36_123);
  ORcp36_323 = OMcp36_122*RLcp36_223-OMcp36_222*RLcp36_123;
  OPcp36_123 = OPcp36_122+qd[23]*(OMcp36_222*ROcp36_922-OMcp36_322*ROcp36_822)+qdd[23]*ROcp36_722;
  OPcp36_223 = OPcp36_222-qd[23]*(OMcp36_122*ROcp36_922-OMcp36_322*ROcp36_722)+qdd[23]*ROcp36_822;
  OPcp36_323 = OPcp36_322+qd[23]*(OMcp36_122*ROcp36_822-OMcp36_222*ROcp36_722)+qdd[23]*ROcp36_922;

// = = Block_0_0_1_13_0_6 = = 
 
// Sensor Kinematics 


  ROcp36_128 = ROcp36_123*C28-ROcp36_722*S28;
  ROcp36_228 = ROcp36_223*C28-ROcp36_822*S28;
  ROcp36_328 = ROcp36_323*C28-ROcp36_922*S28;
  ROcp36_728 = ROcp36_123*S28+ROcp36_722*C28;
  ROcp36_828 = ROcp36_223*S28+ROcp36_822*C28;
  ROcp36_928 = ROcp36_323*S28+ROcp36_922*C28;
  ROcp36_429 = ROcp36_423*C29+ROcp36_728*S29;
  ROcp36_529 = ROcp36_523*C29+ROcp36_828*S29;
  ROcp36_629 = ROcp36_623*C29+ROcp36_928*S29;
  ROcp36_729 = -(ROcp36_423*S29-ROcp36_728*C29);
  ROcp36_829 = -(ROcp36_523*S29-ROcp36_828*C29);
  ROcp36_929 = -(ROcp36_623*S29-ROcp36_928*C29);
  ROcp36_130 = ROcp36_128*C30+ROcp36_429*S30;
  ROcp36_230 = ROcp36_228*C30+ROcp36_529*S30;
  ROcp36_330 = ROcp36_328*C30+ROcp36_629*S30;
  ROcp36_430 = -(ROcp36_128*S30-ROcp36_429*C30);
  ROcp36_530 = -(ROcp36_228*S30-ROcp36_529*C30);
  ROcp36_630 = -(ROcp36_328*S30-ROcp36_629*C30);
  RLcp36_128 = ROcp36_123*s->dpt[1][45]+ROcp36_423*s->dpt[2][45]+ROcp36_722*s->dpt[3][45];
  RLcp36_228 = ROcp36_223*s->dpt[1][45]+ROcp36_523*s->dpt[2][45]+ROcp36_822*s->dpt[3][45];
  RLcp36_328 = ROcp36_323*s->dpt[1][45]+ROcp36_623*s->dpt[2][45]+ROcp36_922*s->dpt[3][45];
  OMcp36_128 = OMcp36_123+qd[28]*ROcp36_423;
  OMcp36_228 = OMcp36_223+qd[28]*ROcp36_523;
  OMcp36_328 = OMcp36_323+qd[28]*ROcp36_623;
  ORcp36_128 = OMcp36_223*RLcp36_328-OMcp36_323*RLcp36_228;
  ORcp36_228 = -(OMcp36_123*RLcp36_328-OMcp36_323*RLcp36_128);
  ORcp36_328 = OMcp36_123*RLcp36_228-OMcp36_223*RLcp36_128;
  OPcp36_128 = OPcp36_123+qd[28]*(OMcp36_223*ROcp36_623-OMcp36_323*ROcp36_523)+qdd[28]*ROcp36_423;
  OPcp36_228 = OPcp36_223-qd[28]*(OMcp36_123*ROcp36_623-OMcp36_323*ROcp36_423)+qdd[28]*ROcp36_523;
  OPcp36_328 = OPcp36_323+qd[28]*(OMcp36_123*ROcp36_523-OMcp36_223*ROcp36_423)+qdd[28]*ROcp36_623;
  RLcp36_129 = ROcp36_423*s->dpt[2][56];
  RLcp36_229 = ROcp36_523*s->dpt[2][56];
  RLcp36_329 = ROcp36_623*s->dpt[2][56];
  OMcp36_129 = OMcp36_128+qd[29]*ROcp36_128;
  OMcp36_229 = OMcp36_228+qd[29]*ROcp36_228;
  OMcp36_329 = OMcp36_328+qd[29]*ROcp36_328;
  ORcp36_129 = OMcp36_228*RLcp36_329-OMcp36_328*RLcp36_229;
  ORcp36_229 = -(OMcp36_128*RLcp36_329-OMcp36_328*RLcp36_129);
  ORcp36_329 = OMcp36_128*RLcp36_229-OMcp36_228*RLcp36_129;
  OPcp36_129 = OPcp36_128+qd[29]*(OMcp36_228*ROcp36_328-OMcp36_328*ROcp36_228)+qdd[29]*ROcp36_128;
  OPcp36_229 = OPcp36_228-qd[29]*(OMcp36_128*ROcp36_328-OMcp36_328*ROcp36_128)+qdd[29]*ROcp36_228;
  OPcp36_329 = OPcp36_328+qd[29]*(OMcp36_128*ROcp36_228-OMcp36_228*ROcp36_128)+qdd[29]*ROcp36_328;
  RLcp36_130 = ROcp36_729*s->dpt[3][58];
  RLcp36_230 = ROcp36_829*s->dpt[3][58];
  RLcp36_330 = ROcp36_929*s->dpt[3][58];
  ORcp36_130 = OMcp36_229*RLcp36_330-OMcp36_329*RLcp36_230;
  ORcp36_230 = -(OMcp36_129*RLcp36_330-OMcp36_329*RLcp36_130);
  ORcp36_330 = OMcp36_129*RLcp36_230-OMcp36_229*RLcp36_130;
  PxF13[1] = q[1]+RLcp36_121+RLcp36_123+RLcp36_128+RLcp36_129+RLcp36_130;
  PxF13[2] = q[2]+RLcp36_221+RLcp36_223+RLcp36_228+RLcp36_229+RLcp36_230;
  PxF13[3] = q[3]+RLcp36_321+RLcp36_323+RLcp36_328+RLcp36_329+RLcp36_330;
  RxF13[1][1] = ROcp36_130;
  RxF13[1][2] = ROcp36_230;
  RxF13[1][3] = ROcp36_330;
  RxF13[2][1] = ROcp36_430;
  RxF13[2][2] = ROcp36_530;
  RxF13[2][3] = ROcp36_630;
  RxF13[3][1] = ROcp36_729;
  RxF13[3][2] = ROcp36_829;
  RxF13[3][3] = ROcp36_929;
  VxF13[1] = qd[1]+ORcp36_121+ORcp36_123+ORcp36_128+ORcp36_129+ORcp36_130;
  VxF13[2] = qd[2]+ORcp36_221+ORcp36_223+ORcp36_228+ORcp36_229+ORcp36_230;
  VxF13[3] = qd[3]+ORcp36_321+ORcp36_323+ORcp36_328+ORcp36_329+ORcp36_330;
  OMxF13[1] = OMcp36_129+qd[30]*ROcp36_729;
  OMxF13[2] = OMcp36_229+qd[30]*ROcp36_829;
  OMxF13[3] = OMcp36_329+qd[30]*ROcp36_929;
  AxF13[1] = qdd[1]+OMcp36_222*ORcp36_323+OMcp36_223*ORcp36_328+OMcp36_228*ORcp36_329+OMcp36_229*ORcp36_330+OMcp36_26*
 ORcp36_321-OMcp36_322*ORcp36_223-OMcp36_323*ORcp36_228-OMcp36_328*ORcp36_229-OMcp36_329*ORcp36_230-OMcp36_36*ORcp36_221+
 OPcp36_222*RLcp36_323+OPcp36_223*RLcp36_328+OPcp36_228*RLcp36_329+OPcp36_229*RLcp36_330+OPcp36_26*RLcp36_321-OPcp36_322*
 RLcp36_223-OPcp36_323*RLcp36_228-OPcp36_328*RLcp36_229-OPcp36_329*RLcp36_230-OPcp36_36*RLcp36_221;
  AxF13[2] = qdd[2]-OMcp36_122*ORcp36_323-OMcp36_123*ORcp36_328-OMcp36_128*ORcp36_329-OMcp36_129*ORcp36_330-OMcp36_16*
 ORcp36_321+OMcp36_322*ORcp36_123+OMcp36_323*ORcp36_128+OMcp36_328*ORcp36_129+OMcp36_329*ORcp36_130+OMcp36_36*ORcp36_121-
 OPcp36_122*RLcp36_323-OPcp36_123*RLcp36_328-OPcp36_128*RLcp36_329-OPcp36_129*RLcp36_330-OPcp36_16*RLcp36_321+OPcp36_322*
 RLcp36_123+OPcp36_323*RLcp36_128+OPcp36_328*RLcp36_129+OPcp36_329*RLcp36_130+OPcp36_36*RLcp36_121;
  AxF13[3] = qdd[3]+OMcp36_122*ORcp36_223+OMcp36_123*ORcp36_228+OMcp36_128*ORcp36_229+OMcp36_129*ORcp36_230+OMcp36_16*
 ORcp36_221-OMcp36_222*ORcp36_123-OMcp36_223*ORcp36_128-OMcp36_228*ORcp36_129-OMcp36_229*ORcp36_130-OMcp36_26*ORcp36_121+
 OPcp36_122*RLcp36_223+OPcp36_123*RLcp36_228+OPcp36_128*RLcp36_229+OPcp36_129*RLcp36_230+OPcp36_16*RLcp36_221-OPcp36_222*
 RLcp36_123-OPcp36_223*RLcp36_128-OPcp36_228*RLcp36_129-OPcp36_229*RLcp36_130-OPcp36_26*RLcp36_121;
  OMPxF13[1] = OPcp36_129+qd[30]*(OMcp36_229*ROcp36_929-OMcp36_329*ROcp36_829)+qdd[30]*ROcp36_729;
  OMPxF13[2] = OPcp36_229-qd[30]*(OMcp36_129*ROcp36_929-OMcp36_329*ROcp36_729)+qdd[30]*ROcp36_829;
  OMPxF13[3] = OPcp36_329+qd[30]*(OMcp36_129*ROcp36_829-OMcp36_229*ROcp36_729)+qdd[30]*ROcp36_929;
 
// Sensor Forces Computation 

  SWr13 = user_ExtForces(PxF13,RxF13,VxF13,OMxF13,AxF13,OMPxF13,s,tsim,13);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc137 = ROcp36_130*SWr13[1]+ROcp36_230*SWr13[2]+ROcp36_330*SWr13[3];
  xfrc237 = ROcp36_430*SWr13[1]+ROcp36_530*SWr13[2]+ROcp36_630*SWr13[3];
  xfrc337 = ROcp36_729*SWr13[1]+ROcp36_829*SWr13[2]+ROcp36_929*SWr13[3];
  frc[1][30] = s->frc[1][30]+xfrc137;
  frc[2][30] = s->frc[2][30]+xfrc237;
  frc[3][30] = s->frc[3][30]+xfrc337;
  xtrq137 = ROcp36_130*SWr13[4]+ROcp36_230*SWr13[5]+ROcp36_330*SWr13[6];
  xtrq237 = ROcp36_430*SWr13[4]+ROcp36_530*SWr13[5]+ROcp36_630*SWr13[6];
  xtrq337 = ROcp36_729*SWr13[4]+ROcp36_829*SWr13[5]+ROcp36_929*SWr13[6];
  trq[1][30] = s->trq[1][30]+xtrq137-xfrc237*(SWr13[9]-s->l[3][30])+xfrc337*(SWr13[8]-s->l[2][30]);
  trq[2][30] = s->trq[2][30]+xtrq237+xfrc137*(SWr13[9]-s->l[3][30])-xfrc337*(SWr13[7]-s->l[1][30]);
  trq[3][30] = s->trq[3][30]+xtrq337-xfrc137*(SWr13[8]-s->l[2][30])+xfrc237*(SWr13[7]-s->l[1][30]);

// = = Block_0_0_1_14_0_1 = = 
 
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

// = = Block_0_0_1_14_0_4 = = 
 
// Sensor Kinematics 


  ROcp37_421 = ROcp37_46*C21+S21*S5;
  ROcp37_521 = ROcp37_56*C21+ROcp37_85*S21;
  ROcp37_621 = ROcp37_66*C21+ROcp37_95*S21;
  ROcp37_721 = -(ROcp37_46*S21-C21*S5);
  ROcp37_821 = -(ROcp37_56*S21-ROcp37_85*C21);
  ROcp37_921 = -(ROcp37_66*S21-ROcp37_95*C21);
  ROcp37_122 = ROcp37_16*C22-ROcp37_721*S22;
  ROcp37_222 = ROcp37_26*C22-ROcp37_821*S22;
  ROcp37_322 = ROcp37_36*C22-ROcp37_921*S22;
  ROcp37_722 = ROcp37_16*S22+ROcp37_721*C22;
  ROcp37_822 = ROcp37_26*S22+ROcp37_821*C22;
  ROcp37_922 = ROcp37_36*S22+ROcp37_921*C22;
  ROcp37_123 = ROcp37_122*C23+ROcp37_421*S23;
  ROcp37_223 = ROcp37_222*C23+ROcp37_521*S23;
  ROcp37_323 = ROcp37_322*C23+ROcp37_621*S23;
  ROcp37_423 = -(ROcp37_122*S23-ROcp37_421*C23);
  ROcp37_523 = -(ROcp37_222*S23-ROcp37_521*C23);
  ROcp37_623 = -(ROcp37_322*S23-ROcp37_621*C23);
  RLcp37_121 = ROcp37_16*s->dpt[1][4]+s->dpt[3][4]*S5;
  RLcp37_221 = ROcp37_26*s->dpt[1][4]+ROcp37_85*s->dpt[3][4];
  RLcp37_321 = ROcp37_36*s->dpt[1][4]+ROcp37_95*s->dpt[3][4];
  OMcp37_121 = OMcp37_16+qd[21]*ROcp37_16;
  OMcp37_221 = OMcp37_26+qd[21]*ROcp37_26;
  OMcp37_321 = OMcp37_36+qd[21]*ROcp37_36;
  ORcp37_121 = OMcp37_26*RLcp37_321-OMcp37_36*RLcp37_221;
  ORcp37_221 = -(OMcp37_16*RLcp37_321-OMcp37_36*RLcp37_121);
  ORcp37_321 = OMcp37_16*RLcp37_221-OMcp37_26*RLcp37_121;
  OMcp37_122 = OMcp37_121+qd[22]*ROcp37_421;
  OMcp37_222 = OMcp37_221+qd[22]*ROcp37_521;
  OMcp37_322 = OMcp37_321+qd[22]*ROcp37_621;
  OPcp37_122 = OPcp37_16+qd[21]*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26)+qd[22]*(OMcp37_221*ROcp37_621-OMcp37_321*
 ROcp37_521)+qdd[21]*ROcp37_16+qdd[22]*ROcp37_421;
  OPcp37_222 = OPcp37_26-qd[21]*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16)-qd[22]*(OMcp37_121*ROcp37_621-OMcp37_321*
 ROcp37_421)+qdd[21]*ROcp37_26+qdd[22]*ROcp37_521;
  OPcp37_322 = OPcp37_36+qd[21]*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16)+qd[22]*(OMcp37_121*ROcp37_521-OMcp37_221*
 ROcp37_421)+qdd[21]*ROcp37_36+qdd[22]*ROcp37_621;
  RLcp37_123 = ROcp37_722*s->dpt[3][40];
  RLcp37_223 = ROcp37_822*s->dpt[3][40];
  RLcp37_323 = ROcp37_922*s->dpt[3][40];
  OMcp37_123 = OMcp37_122+qd[23]*ROcp37_722;
  OMcp37_223 = OMcp37_222+qd[23]*ROcp37_822;
  OMcp37_323 = OMcp37_322+qd[23]*ROcp37_922;
  ORcp37_123 = OMcp37_222*RLcp37_323-OMcp37_322*RLcp37_223;
  ORcp37_223 = -(OMcp37_122*RLcp37_323-OMcp37_322*RLcp37_123);
  ORcp37_323 = OMcp37_122*RLcp37_223-OMcp37_222*RLcp37_123;
  OPcp37_123 = OPcp37_122+qd[23]*(OMcp37_222*ROcp37_922-OMcp37_322*ROcp37_822)+qdd[23]*ROcp37_722;
  OPcp37_223 = OPcp37_222-qd[23]*(OMcp37_122*ROcp37_922-OMcp37_322*ROcp37_722)+qdd[23]*ROcp37_822;
  OPcp37_323 = OPcp37_322+qd[23]*(OMcp37_122*ROcp37_822-OMcp37_222*ROcp37_722)+qdd[23]*ROcp37_922;

// = = Block_0_0_1_14_0_6 = = 
 
// Sensor Kinematics 


  ROcp37_128 = ROcp37_123*C28-ROcp37_722*S28;
  ROcp37_228 = ROcp37_223*C28-ROcp37_822*S28;
  ROcp37_328 = ROcp37_323*C28-ROcp37_922*S28;
  ROcp37_728 = ROcp37_123*S28+ROcp37_722*C28;
  ROcp37_828 = ROcp37_223*S28+ROcp37_822*C28;
  ROcp37_928 = ROcp37_323*S28+ROcp37_922*C28;
  ROcp37_429 = ROcp37_423*C29+ROcp37_728*S29;
  ROcp37_529 = ROcp37_523*C29+ROcp37_828*S29;
  ROcp37_629 = ROcp37_623*C29+ROcp37_928*S29;
  ROcp37_729 = -(ROcp37_423*S29-ROcp37_728*C29);
  ROcp37_829 = -(ROcp37_523*S29-ROcp37_828*C29);
  ROcp37_929 = -(ROcp37_623*S29-ROcp37_928*C29);
  ROcp37_130 = ROcp37_128*C30+ROcp37_429*S30;
  ROcp37_230 = ROcp37_228*C30+ROcp37_529*S30;
  ROcp37_330 = ROcp37_328*C30+ROcp37_629*S30;
  ROcp37_430 = -(ROcp37_128*S30-ROcp37_429*C30);
  ROcp37_530 = -(ROcp37_228*S30-ROcp37_529*C30);
  ROcp37_630 = -(ROcp37_328*S30-ROcp37_629*C30);
  ROcp37_131 = ROcp37_130*C31-ROcp37_729*S31;
  ROcp37_231 = ROcp37_230*C31-ROcp37_829*S31;
  ROcp37_331 = ROcp37_330*C31-ROcp37_929*S31;
  ROcp37_731 = ROcp37_130*S31+ROcp37_729*C31;
  ROcp37_831 = ROcp37_230*S31+ROcp37_829*C31;
  ROcp37_931 = ROcp37_330*S31+ROcp37_929*C31;
  RLcp37_128 = ROcp37_123*s->dpt[1][45]+ROcp37_423*s->dpt[2][45]+ROcp37_722*s->dpt[3][45];
  RLcp37_228 = ROcp37_223*s->dpt[1][45]+ROcp37_523*s->dpt[2][45]+ROcp37_822*s->dpt[3][45];
  RLcp37_328 = ROcp37_323*s->dpt[1][45]+ROcp37_623*s->dpt[2][45]+ROcp37_922*s->dpt[3][45];
  OMcp37_128 = OMcp37_123+qd[28]*ROcp37_423;
  OMcp37_228 = OMcp37_223+qd[28]*ROcp37_523;
  OMcp37_328 = OMcp37_323+qd[28]*ROcp37_623;
  ORcp37_128 = OMcp37_223*RLcp37_328-OMcp37_323*RLcp37_228;
  ORcp37_228 = -(OMcp37_123*RLcp37_328-OMcp37_323*RLcp37_128);
  ORcp37_328 = OMcp37_123*RLcp37_228-OMcp37_223*RLcp37_128;
  OPcp37_128 = OPcp37_123+qd[28]*(OMcp37_223*ROcp37_623-OMcp37_323*ROcp37_523)+qdd[28]*ROcp37_423;
  OPcp37_228 = OPcp37_223-qd[28]*(OMcp37_123*ROcp37_623-OMcp37_323*ROcp37_423)+qdd[28]*ROcp37_523;
  OPcp37_328 = OPcp37_323+qd[28]*(OMcp37_123*ROcp37_523-OMcp37_223*ROcp37_423)+qdd[28]*ROcp37_623;
  RLcp37_129 = ROcp37_423*s->dpt[2][56];
  RLcp37_229 = ROcp37_523*s->dpt[2][56];
  RLcp37_329 = ROcp37_623*s->dpt[2][56];
  OMcp37_129 = OMcp37_128+qd[29]*ROcp37_128;
  OMcp37_229 = OMcp37_228+qd[29]*ROcp37_228;
  OMcp37_329 = OMcp37_328+qd[29]*ROcp37_328;
  ORcp37_129 = OMcp37_228*RLcp37_329-OMcp37_328*RLcp37_229;
  ORcp37_229 = -(OMcp37_128*RLcp37_329-OMcp37_328*RLcp37_129);
  ORcp37_329 = OMcp37_128*RLcp37_229-OMcp37_228*RLcp37_129;
  OPcp37_129 = OPcp37_128+qd[29]*(OMcp37_228*ROcp37_328-OMcp37_328*ROcp37_228)+qdd[29]*ROcp37_128;
  OPcp37_229 = OPcp37_228-qd[29]*(OMcp37_128*ROcp37_328-OMcp37_328*ROcp37_128)+qdd[29]*ROcp37_228;
  OPcp37_329 = OPcp37_328+qd[29]*(OMcp37_128*ROcp37_228-OMcp37_228*ROcp37_128)+qdd[29]*ROcp37_328;
  RLcp37_130 = ROcp37_729*s->dpt[3][58];
  RLcp37_230 = ROcp37_829*s->dpt[3][58];
  RLcp37_330 = ROcp37_929*s->dpt[3][58];
  OMcp37_130 = OMcp37_129+qd[30]*ROcp37_729;
  OMcp37_230 = OMcp37_229+qd[30]*ROcp37_829;
  OMcp37_330 = OMcp37_329+qd[30]*ROcp37_929;
  ORcp37_130 = OMcp37_229*RLcp37_330-OMcp37_329*RLcp37_230;
  ORcp37_230 = -(OMcp37_129*RLcp37_330-OMcp37_329*RLcp37_130);
  ORcp37_330 = OMcp37_129*RLcp37_230-OMcp37_229*RLcp37_130;
  OPcp37_130 = OPcp37_129+qd[30]*(OMcp37_229*ROcp37_929-OMcp37_329*ROcp37_829)+qdd[30]*ROcp37_729;
  OPcp37_230 = OPcp37_229-qd[30]*(OMcp37_129*ROcp37_929-OMcp37_329*ROcp37_729)+qdd[30]*ROcp37_829;
  OPcp37_330 = OPcp37_329+qd[30]*(OMcp37_129*ROcp37_829-OMcp37_229*ROcp37_729)+qdd[30]*ROcp37_929;
  RLcp37_131 = ROcp37_729*s->dpt[3][61];
  RLcp37_231 = ROcp37_829*s->dpt[3][61];
  RLcp37_331 = ROcp37_929*s->dpt[3][61];
  ORcp37_131 = OMcp37_230*RLcp37_331-OMcp37_330*RLcp37_231;
  ORcp37_231 = -(OMcp37_130*RLcp37_331-OMcp37_330*RLcp37_131);
  ORcp37_331 = OMcp37_130*RLcp37_231-OMcp37_230*RLcp37_131;
  PxF14[1] = q[1]+RLcp37_121+RLcp37_123+RLcp37_128+RLcp37_129+RLcp37_130+RLcp37_131;
  PxF14[2] = q[2]+RLcp37_221+RLcp37_223+RLcp37_228+RLcp37_229+RLcp37_230+RLcp37_231;
  PxF14[3] = q[3]+RLcp37_321+RLcp37_323+RLcp37_328+RLcp37_329+RLcp37_330+RLcp37_331;
  RxF14[1][1] = ROcp37_131;
  RxF14[1][2] = ROcp37_231;
  RxF14[1][3] = ROcp37_331;
  RxF14[2][1] = ROcp37_430;
  RxF14[2][2] = ROcp37_530;
  RxF14[2][3] = ROcp37_630;
  RxF14[3][1] = ROcp37_731;
  RxF14[3][2] = ROcp37_831;
  RxF14[3][3] = ROcp37_931;
  VxF14[1] = qd[1]+ORcp37_121+ORcp37_123+ORcp37_128+ORcp37_129+ORcp37_130+ORcp37_131;
  VxF14[2] = qd[2]+ORcp37_221+ORcp37_223+ORcp37_228+ORcp37_229+ORcp37_230+ORcp37_231;
  VxF14[3] = qd[3]+ORcp37_321+ORcp37_323+ORcp37_328+ORcp37_329+ORcp37_330+ORcp37_331;
  OMxF14[1] = OMcp37_130+qd[31]*ROcp37_430;
  OMxF14[2] = OMcp37_230+qd[31]*ROcp37_530;
  OMxF14[3] = OMcp37_330+qd[31]*ROcp37_630;
  AxF14[1] = qdd[1]+OMcp37_222*ORcp37_323+OMcp37_223*ORcp37_328+OMcp37_228*ORcp37_329+OMcp37_229*ORcp37_330+OMcp37_230*
 ORcp37_331+OMcp37_26*ORcp37_321-OMcp37_322*ORcp37_223-OMcp37_323*ORcp37_228-OMcp37_328*ORcp37_229-OMcp37_329*ORcp37_230-
 OMcp37_330*ORcp37_231-OMcp37_36*ORcp37_221+OPcp37_222*RLcp37_323+OPcp37_223*RLcp37_328+OPcp37_228*RLcp37_329+OPcp37_229*
 RLcp37_330+OPcp37_230*RLcp37_331+OPcp37_26*RLcp37_321-OPcp37_322*RLcp37_223-OPcp37_323*RLcp37_228-OPcp37_328*RLcp37_229-
 OPcp37_329*RLcp37_230-OPcp37_330*RLcp37_231-OPcp37_36*RLcp37_221;
  AxF14[2] = qdd[2]-OMcp37_122*ORcp37_323-OMcp37_123*ORcp37_328-OMcp37_128*ORcp37_329-OMcp37_129*ORcp37_330-OMcp37_130*
 ORcp37_331-OMcp37_16*ORcp37_321+OMcp37_322*ORcp37_123+OMcp37_323*ORcp37_128+OMcp37_328*ORcp37_129+OMcp37_329*ORcp37_130+
 OMcp37_330*ORcp37_131+OMcp37_36*ORcp37_121-OPcp37_122*RLcp37_323-OPcp37_123*RLcp37_328-OPcp37_128*RLcp37_329-OPcp37_129*
 RLcp37_330-OPcp37_130*RLcp37_331-OPcp37_16*RLcp37_321+OPcp37_322*RLcp37_123+OPcp37_323*RLcp37_128+OPcp37_328*RLcp37_129+
 OPcp37_329*RLcp37_130+OPcp37_330*RLcp37_131+OPcp37_36*RLcp37_121;
  AxF14[3] = qdd[3]+OMcp37_122*ORcp37_223+OMcp37_123*ORcp37_228+OMcp37_128*ORcp37_229+OMcp37_129*ORcp37_230+OMcp37_130*
 ORcp37_231+OMcp37_16*ORcp37_221-OMcp37_222*ORcp37_123-OMcp37_223*ORcp37_128-OMcp37_228*ORcp37_129-OMcp37_229*ORcp37_130-
 OMcp37_230*ORcp37_131-OMcp37_26*ORcp37_121+OPcp37_122*RLcp37_223+OPcp37_123*RLcp37_228+OPcp37_128*RLcp37_229+OPcp37_129*
 RLcp37_230+OPcp37_130*RLcp37_231+OPcp37_16*RLcp37_221-OPcp37_222*RLcp37_123-OPcp37_223*RLcp37_128-OPcp37_228*RLcp37_129-
 OPcp37_229*RLcp37_130-OPcp37_230*RLcp37_131-OPcp37_26*RLcp37_121;
  OMPxF14[1] = OPcp37_130+qd[31]*(OMcp37_230*ROcp37_630-OMcp37_330*ROcp37_530)+qdd[31]*ROcp37_430;
  OMPxF14[2] = OPcp37_230-qd[31]*(OMcp37_130*ROcp37_630-OMcp37_330*ROcp37_430)+qdd[31]*ROcp37_530;
  OMPxF14[3] = OPcp37_330+qd[31]*(OMcp37_130*ROcp37_530-OMcp37_230*ROcp37_430)+qdd[31]*ROcp37_630;
 
// Sensor Forces Computation 

  SWr14 = user_ExtForces(PxF14,RxF14,VxF14,OMxF14,AxF14,OMPxF14,s,tsim,14);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc138 = ROcp37_131*SWr14[1]+ROcp37_231*SWr14[2]+ROcp37_331*SWr14[3];
  xfrc238 = ROcp37_430*SWr14[1]+ROcp37_530*SWr14[2]+ROcp37_630*SWr14[3];
  xfrc338 = ROcp37_731*SWr14[1]+ROcp37_831*SWr14[2]+ROcp37_931*SWr14[3];
  frc[1][31] = s->frc[1][31]+xfrc138;
  frc[2][31] = s->frc[2][31]+xfrc238;
  frc[3][31] = s->frc[3][31]+xfrc338;
  xtrq138 = ROcp37_131*SWr14[4]+ROcp37_231*SWr14[5]+ROcp37_331*SWr14[6];
  xtrq238 = ROcp37_430*SWr14[4]+ROcp37_530*SWr14[5]+ROcp37_630*SWr14[6];
  xtrq338 = ROcp37_731*SWr14[4]+ROcp37_831*SWr14[5]+ROcp37_931*SWr14[6];
  trq[1][31] = s->trq[1][31]+xtrq138-xfrc238*(SWr14[9]-s->l[3][31])+xfrc338*(SWr14[8]-s->l[2][31]);
  trq[2][31] = s->trq[2][31]+xtrq238+xfrc138*(SWr14[9]-s->l[3][31])-xfrc338*(SWr14[7]-s->l[1][31]);
  trq[3][31] = s->trq[3][31]+xtrq338-xfrc138*(SWr14[8]-s->l[2][31])+xfrc238*(SWr14[7]-s->l[1][31]);

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
  frc[1][14] = s->frc[1][14];
  frc[2][14] = s->frc[2][14];
  frc[3][14] = s->frc[3][14];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][14] = s->trq[1][14];
  trq[2][14] = s->trq[2][14];
  trq[3][14] = s->trq[3][14];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];

// ====== END Task 0 ====== 


}
 

