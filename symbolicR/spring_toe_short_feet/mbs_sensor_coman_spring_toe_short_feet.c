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
//	==> Generation Date : Tue May 24 15:03:38 2016
//
//	==> Project name : coman_spring_toe_short_feet
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 9492
//
//	==> Generation Time :  0.310 seconds
//	==> Post-Processing :  0.200 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void  mbs_sensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_sensor_coman_spring_toe_short_feet.h" 
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

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    OMcp0_16 = qd[4]+qd[6]*S5;
    OMcp0_26 = ROcp0_85*qd[6]+qd[5]*C4;
    OMcp0_36 = ROcp0_95*qd[6]+qd[5]*S4;
    RLcp0_132 = s->dpt[3][5]*S5+C5*(s->dpt[1][5]*C6-s->dpt[2][5]*S6);
    RLcp0_232 = ROcp0_85*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_25*C6+C4*S6)-s->dpt[2][5]*(ROcp0_25*S6-C4*C6);
    RLcp0_332 = ROcp0_95*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_35*C6+S4*S6)-s->dpt[2][5]*(ROcp0_35*S6-S4*C6);
    POcp0_132 = RLcp0_132+q[1];
    POcp0_232 = RLcp0_232+q[2];
    POcp0_332 = RLcp0_332+q[3];
    VIcp0_132 = qd[1]+OMcp0_26*RLcp0_332-OMcp0_36*RLcp0_232;
    VIcp0_232 = qd[2]-OMcp0_16*RLcp0_332+OMcp0_36*RLcp0_132;
    VIcp0_332 = qd[3]+OMcp0_16*RLcp0_232-OMcp0_26*RLcp0_132;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_132;
    sens->P[2] = POcp0_232;
    sens->P[3] = POcp0_332;
    sens->V[1] = VIcp0_132;
    sens->V[2] = VIcp0_232;
    sens->V[3] = VIcp0_332;
    sens->OM[1] = OMcp0_16;
    sens->OM[2] = OMcp0_26;
    sens->OM[3] = OMcp0_36;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_16 = qd[4]+qd[6]*S5;
    OMcp1_26 = ROcp1_85*qd[6]+qd[5]*C4;
    OMcp1_36 = ROcp1_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_2_0_2 = = 
 
// Sensor Kinematics 


    RLcp1_17 = ROcp1_46*s->dpt[2][2];
    RLcp1_27 = ROcp1_56*s->dpt[2][2];
    RLcp1_37 = ROcp1_66*s->dpt[2][2];
    OMcp1_17 = OMcp1_16+ROcp1_46*qd[7];
    OMcp1_27 = OMcp1_26+ROcp1_56*qd[7];
    OMcp1_37 = OMcp1_36+ROcp1_66*qd[7];
    RLcp1_133 = ROcp1_46*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_16*C7-S5*S7)+s->dpt[3][7]*(ROcp1_16*S7+S5*C7);
    RLcp1_233 = ROcp1_56*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_26*C7-ROcp1_85*S7)+s->dpt[3][7]*(ROcp1_26*S7+ROcp1_85*C7);
    RLcp1_333 = ROcp1_66*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_36*C7-ROcp1_95*S7)+s->dpt[3][7]*(ROcp1_36*S7+ROcp1_95*C7);
    POcp1_133 = RLcp1_133+RLcp1_17+q[1];
    POcp1_233 = RLcp1_233+RLcp1_27+q[2];
    POcp1_333 = RLcp1_333+RLcp1_37+q[3];
    VIcp1_133 = qd[1]+OMcp1_26*RLcp1_37+OMcp1_27*RLcp1_333-OMcp1_36*RLcp1_27-OMcp1_37*RLcp1_233;
    VIcp1_233 = qd[2]-OMcp1_16*RLcp1_37-OMcp1_17*RLcp1_333+OMcp1_36*RLcp1_17+OMcp1_37*RLcp1_133;
    VIcp1_333 = qd[3]+OMcp1_16*RLcp1_27+OMcp1_17*RLcp1_233-OMcp1_26*RLcp1_17-OMcp1_27*RLcp1_133;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_133;
    sens->P[2] = POcp1_233;
    sens->P[3] = POcp1_333;
    sens->V[1] = VIcp1_133;
    sens->V[2] = VIcp1_233;
    sens->V[3] = VIcp1_333;
    sens->OM[1] = OMcp1_17;
    sens->OM[2] = OMcp1_27;
    sens->OM[3] = OMcp1_37;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_16 = qd[4]+qd[6]*S5;
    OMcp2_26 = ROcp2_85*qd[6]+qd[5]*C4;
    OMcp2_36 = ROcp2_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_3_0_2 = = 
 
// Sensor Kinematics 


    ROcp2_17 = ROcp2_16*C7-S5*S7;
    ROcp2_27 = ROcp2_26*C7-ROcp2_85*S7;
    ROcp2_37 = ROcp2_36*C7-ROcp2_95*S7;
    ROcp2_77 = ROcp2_16*S7+S5*C7;
    ROcp2_87 = ROcp2_26*S7+ROcp2_85*C7;
    ROcp2_97 = ROcp2_36*S7+ROcp2_95*C7;
    RLcp2_17 = ROcp2_46*s->dpt[2][2];
    RLcp2_27 = ROcp2_56*s->dpt[2][2];
    RLcp2_37 = ROcp2_66*s->dpt[2][2];
    OMcp2_17 = OMcp2_16+ROcp2_46*qd[7];
    OMcp2_27 = OMcp2_26+ROcp2_56*qd[7];
    OMcp2_37 = OMcp2_36+ROcp2_66*qd[7];
    RLcp2_18 = ROcp2_46*s->dpt[2][6];
    RLcp2_28 = ROcp2_56*s->dpt[2][6];
    RLcp2_38 = ROcp2_66*s->dpt[2][6];
    OMcp2_18 = OMcp2_17+ROcp2_17*qd[8];
    OMcp2_28 = OMcp2_27+ROcp2_27*qd[8];
    OMcp2_38 = OMcp2_37+ROcp2_37*qd[8];
    RLcp2_134 = ROcp2_17*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_46*C8+ROcp2_77*S8)-s->dpt[3][9]*(ROcp2_46*S8-ROcp2_77*C8);
    RLcp2_234 = ROcp2_27*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_56*C8+ROcp2_87*S8)-s->dpt[3][9]*(ROcp2_56*S8-ROcp2_87*C8);
    RLcp2_334 = ROcp2_37*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_66*C8+ROcp2_97*S8)-s->dpt[3][9]*(ROcp2_66*S8-ROcp2_97*C8);
    POcp2_134 = RLcp2_134+RLcp2_17+RLcp2_18+q[1];
    POcp2_234 = RLcp2_234+RLcp2_27+RLcp2_28+q[2];
    POcp2_334 = RLcp2_334+RLcp2_37+RLcp2_38+q[3];
    VIcp2_134 = qd[1]+OMcp2_26*RLcp2_37+OMcp2_27*RLcp2_38+OMcp2_28*RLcp2_334-OMcp2_36*RLcp2_27-OMcp2_37*RLcp2_28-OMcp2_38*
 RLcp2_234;
    VIcp2_234 = qd[2]-OMcp2_16*RLcp2_37-OMcp2_17*RLcp2_38-OMcp2_18*RLcp2_334+OMcp2_36*RLcp2_17+OMcp2_37*RLcp2_18+OMcp2_38*
 RLcp2_134;
    VIcp2_334 = qd[3]+OMcp2_16*RLcp2_27+OMcp2_17*RLcp2_28+OMcp2_18*RLcp2_234-OMcp2_26*RLcp2_17-OMcp2_27*RLcp2_18-OMcp2_28*
 RLcp2_134;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_134;
    sens->P[2] = POcp2_234;
    sens->P[3] = POcp2_334;
    sens->V[1] = VIcp2_134;
    sens->V[2] = VIcp2_234;
    sens->V[3] = VIcp2_334;
    sens->OM[1] = OMcp2_18;
    sens->OM[2] = OMcp2_28;
    sens->OM[3] = OMcp2_38;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_16 = qd[4]+qd[6]*S5;
    OMcp3_26 = ROcp3_85*qd[6]+qd[5]*C4;
    OMcp3_36 = ROcp3_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_4_0_2 = = 
 
// Sensor Kinematics 


    ROcp3_17 = ROcp3_16*C7-S5*S7;
    ROcp3_27 = ROcp3_26*C7-ROcp3_85*S7;
    ROcp3_37 = ROcp3_36*C7-ROcp3_95*S7;
    ROcp3_77 = ROcp3_16*S7+S5*C7;
    ROcp3_87 = ROcp3_26*S7+ROcp3_85*C7;
    ROcp3_97 = ROcp3_36*S7+ROcp3_95*C7;
    ROcp3_48 = ROcp3_46*C8+ROcp3_77*S8;
    ROcp3_58 = ROcp3_56*C8+ROcp3_87*S8;
    ROcp3_68 = ROcp3_66*C8+ROcp3_97*S8;
    ROcp3_78 = -(ROcp3_46*S8-ROcp3_77*C8);
    ROcp3_88 = -(ROcp3_56*S8-ROcp3_87*C8);
    ROcp3_98 = -(ROcp3_66*S8-ROcp3_97*C8);
    RLcp3_17 = ROcp3_46*s->dpt[2][2];
    RLcp3_27 = ROcp3_56*s->dpt[2][2];
    RLcp3_37 = ROcp3_66*s->dpt[2][2];
    OMcp3_17 = OMcp3_16+ROcp3_46*qd[7];
    OMcp3_27 = OMcp3_26+ROcp3_56*qd[7];
    OMcp3_37 = OMcp3_36+ROcp3_66*qd[7];
    RLcp3_18 = ROcp3_46*s->dpt[2][6];
    RLcp3_28 = ROcp3_56*s->dpt[2][6];
    RLcp3_38 = ROcp3_66*s->dpt[2][6];
    OMcp3_18 = OMcp3_17+ROcp3_17*qd[8];
    OMcp3_28 = OMcp3_27+ROcp3_27*qd[8];
    OMcp3_38 = OMcp3_37+ROcp3_37*qd[8];
    RLcp3_19 = ROcp3_78*s->dpt[3][8];
    RLcp3_29 = ROcp3_88*s->dpt[3][8];
    RLcp3_39 = ROcp3_98*s->dpt[3][8];
    OMcp3_19 = OMcp3_18+ROcp3_78*qd[9];
    OMcp3_29 = OMcp3_28+ROcp3_88*qd[9];
    OMcp3_39 = OMcp3_38+ROcp3_98*qd[9];
    RLcp3_135 = ROcp3_78*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_17*C9+ROcp3_48*S9)-s->dpt[2][12]*(ROcp3_17*S9-ROcp3_48*C9);
    RLcp3_235 = ROcp3_88*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_27*C9+ROcp3_58*S9)-s->dpt[2][12]*(ROcp3_27*S9-ROcp3_58*C9);
    RLcp3_335 = ROcp3_98*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_37*C9+ROcp3_68*S9)-s->dpt[2][12]*(ROcp3_37*S9-ROcp3_68*C9);
    POcp3_135 = RLcp3_135+RLcp3_17+RLcp3_18+RLcp3_19+q[1];
    POcp3_235 = RLcp3_235+RLcp3_27+RLcp3_28+RLcp3_29+q[2];
    POcp3_335 = RLcp3_335+RLcp3_37+RLcp3_38+RLcp3_39+q[3];
    VIcp3_135 = qd[1]+OMcp3_26*RLcp3_37+OMcp3_27*RLcp3_38+OMcp3_28*RLcp3_39+OMcp3_29*RLcp3_335-OMcp3_36*RLcp3_27-OMcp3_37*
 RLcp3_28-OMcp3_38*RLcp3_29-OMcp3_39*RLcp3_235;
    VIcp3_235 = qd[2]-OMcp3_16*RLcp3_37-OMcp3_17*RLcp3_38-OMcp3_18*RLcp3_39-OMcp3_19*RLcp3_335+OMcp3_36*RLcp3_17+OMcp3_37*
 RLcp3_18+OMcp3_38*RLcp3_19+OMcp3_39*RLcp3_135;
    VIcp3_335 = qd[3]+OMcp3_16*RLcp3_27+OMcp3_17*RLcp3_28+OMcp3_18*RLcp3_29+OMcp3_19*RLcp3_235-OMcp3_26*RLcp3_17-OMcp3_27*
 RLcp3_18-OMcp3_28*RLcp3_19-OMcp3_29*RLcp3_135;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_135;
    sens->P[2] = POcp3_235;
    sens->P[3] = POcp3_335;
    sens->V[1] = VIcp3_135;
    sens->V[2] = VIcp3_235;
    sens->V[3] = VIcp3_335;
    sens->OM[1] = OMcp3_19;
    sens->OM[2] = OMcp3_29;
    sens->OM[3] = OMcp3_39;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_16 = qd[4]+qd[6]*S5;
    OMcp4_26 = ROcp4_85*qd[6]+qd[5]*C4;
    OMcp4_36 = ROcp4_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_5_0_2 = = 
 
// Sensor Kinematics 


    ROcp4_17 = ROcp4_16*C7-S5*S7;
    ROcp4_27 = ROcp4_26*C7-ROcp4_85*S7;
    ROcp4_37 = ROcp4_36*C7-ROcp4_95*S7;
    ROcp4_77 = ROcp4_16*S7+S5*C7;
    ROcp4_87 = ROcp4_26*S7+ROcp4_85*C7;
    ROcp4_97 = ROcp4_36*S7+ROcp4_95*C7;
    ROcp4_48 = ROcp4_46*C8+ROcp4_77*S8;
    ROcp4_58 = ROcp4_56*C8+ROcp4_87*S8;
    ROcp4_68 = ROcp4_66*C8+ROcp4_97*S8;
    ROcp4_78 = -(ROcp4_46*S8-ROcp4_77*C8);
    ROcp4_88 = -(ROcp4_56*S8-ROcp4_87*C8);
    ROcp4_98 = -(ROcp4_66*S8-ROcp4_97*C8);
    ROcp4_19 = ROcp4_17*C9+ROcp4_48*S9;
    ROcp4_29 = ROcp4_27*C9+ROcp4_58*S9;
    ROcp4_39 = ROcp4_37*C9+ROcp4_68*S9;
    ROcp4_49 = -(ROcp4_17*S9-ROcp4_48*C9);
    ROcp4_59 = -(ROcp4_27*S9-ROcp4_58*C9);
    ROcp4_69 = -(ROcp4_37*S9-ROcp4_68*C9);
    RLcp4_17 = ROcp4_46*s->dpt[2][2];
    RLcp4_27 = ROcp4_56*s->dpt[2][2];
    RLcp4_37 = ROcp4_66*s->dpt[2][2];
    OMcp4_17 = OMcp4_16+ROcp4_46*qd[7];
    OMcp4_27 = OMcp4_26+ROcp4_56*qd[7];
    OMcp4_37 = OMcp4_36+ROcp4_66*qd[7];
    RLcp4_18 = ROcp4_46*s->dpt[2][6];
    RLcp4_28 = ROcp4_56*s->dpt[2][6];
    RLcp4_38 = ROcp4_66*s->dpt[2][6];
    OMcp4_18 = OMcp4_17+ROcp4_17*qd[8];
    OMcp4_28 = OMcp4_27+ROcp4_27*qd[8];
    OMcp4_38 = OMcp4_37+ROcp4_37*qd[8];
    RLcp4_19 = ROcp4_78*s->dpt[3][8];
    RLcp4_29 = ROcp4_88*s->dpt[3][8];
    RLcp4_39 = ROcp4_98*s->dpt[3][8];
    OMcp4_19 = OMcp4_18+ROcp4_78*qd[9];
    OMcp4_29 = OMcp4_28+ROcp4_88*qd[9];
    OMcp4_39 = OMcp4_38+ROcp4_98*qd[9];
    RLcp4_110 = ROcp4_78*s->dpt[3][11];
    RLcp4_210 = ROcp4_88*s->dpt[3][11];
    RLcp4_310 = ROcp4_98*s->dpt[3][11];
    OMcp4_110 = OMcp4_19+ROcp4_49*qd[10];
    OMcp4_210 = OMcp4_29+ROcp4_59*qd[10];
    OMcp4_310 = OMcp4_39+ROcp4_69*qd[10];
    RLcp4_136 = ROcp4_49*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_19*C10-ROcp4_78*S10)+s->dpt[3][15]*(ROcp4_19*S10+ROcp4_78*C10);
    RLcp4_236 = ROcp4_59*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_29*C10-ROcp4_88*S10)+s->dpt[3][15]*(ROcp4_29*S10+ROcp4_88*C10);
    RLcp4_336 = ROcp4_69*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_39*C10-ROcp4_98*S10)+s->dpt[3][15]*(ROcp4_39*S10+ROcp4_98*C10);
    POcp4_136 = RLcp4_110+RLcp4_136+RLcp4_17+RLcp4_18+RLcp4_19+q[1];
    POcp4_236 = RLcp4_210+RLcp4_236+RLcp4_27+RLcp4_28+RLcp4_29+q[2];
    POcp4_336 = RLcp4_310+RLcp4_336+RLcp4_37+RLcp4_38+RLcp4_39+q[3];
    VIcp4_136 = qd[1]+OMcp4_210*RLcp4_336+OMcp4_26*RLcp4_37+OMcp4_27*RLcp4_38+OMcp4_28*RLcp4_39+OMcp4_29*RLcp4_310-
 OMcp4_310*RLcp4_236-OMcp4_36*RLcp4_27-OMcp4_37*RLcp4_28-OMcp4_38*RLcp4_29-OMcp4_39*RLcp4_210;
    VIcp4_236 = qd[2]-OMcp4_110*RLcp4_336-OMcp4_16*RLcp4_37-OMcp4_17*RLcp4_38-OMcp4_18*RLcp4_39-OMcp4_19*RLcp4_310+
 OMcp4_310*RLcp4_136+OMcp4_36*RLcp4_17+OMcp4_37*RLcp4_18+OMcp4_38*RLcp4_19+OMcp4_39*RLcp4_110;
    VIcp4_336 = qd[3]+OMcp4_110*RLcp4_236+OMcp4_16*RLcp4_27+OMcp4_17*RLcp4_28+OMcp4_18*RLcp4_29+OMcp4_19*RLcp4_210-
 OMcp4_210*RLcp4_136-OMcp4_26*RLcp4_17-OMcp4_27*RLcp4_18-OMcp4_28*RLcp4_19-OMcp4_29*RLcp4_110;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_136;
    sens->P[2] = POcp4_236;
    sens->P[3] = POcp4_336;
    sens->V[1] = VIcp4_136;
    sens->V[2] = VIcp4_236;
    sens->V[3] = VIcp4_336;
    sens->OM[1] = OMcp4_110;
    sens->OM[2] = OMcp4_210;
    sens->OM[3] = OMcp4_310;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_16 = qd[4]+qd[6]*S5;
    OMcp5_26 = ROcp5_85*qd[6]+qd[5]*C4;
    OMcp5_36 = ROcp5_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_6_0_2 = = 
 
// Sensor Kinematics 


    ROcp5_17 = ROcp5_16*C7-S5*S7;
    ROcp5_27 = ROcp5_26*C7-ROcp5_85*S7;
    ROcp5_37 = ROcp5_36*C7-ROcp5_95*S7;
    ROcp5_77 = ROcp5_16*S7+S5*C7;
    ROcp5_87 = ROcp5_26*S7+ROcp5_85*C7;
    ROcp5_97 = ROcp5_36*S7+ROcp5_95*C7;
    ROcp5_48 = ROcp5_46*C8+ROcp5_77*S8;
    ROcp5_58 = ROcp5_56*C8+ROcp5_87*S8;
    ROcp5_68 = ROcp5_66*C8+ROcp5_97*S8;
    ROcp5_78 = -(ROcp5_46*S8-ROcp5_77*C8);
    ROcp5_88 = -(ROcp5_56*S8-ROcp5_87*C8);
    ROcp5_98 = -(ROcp5_66*S8-ROcp5_97*C8);
    ROcp5_19 = ROcp5_17*C9+ROcp5_48*S9;
    ROcp5_29 = ROcp5_27*C9+ROcp5_58*S9;
    ROcp5_39 = ROcp5_37*C9+ROcp5_68*S9;
    ROcp5_49 = -(ROcp5_17*S9-ROcp5_48*C9);
    ROcp5_59 = -(ROcp5_27*S9-ROcp5_58*C9);
    ROcp5_69 = -(ROcp5_37*S9-ROcp5_68*C9);
    ROcp5_110 = ROcp5_19*C10-ROcp5_78*S10;
    ROcp5_210 = ROcp5_29*C10-ROcp5_88*S10;
    ROcp5_310 = ROcp5_39*C10-ROcp5_98*S10;
    ROcp5_710 = ROcp5_19*S10+ROcp5_78*C10;
    ROcp5_810 = ROcp5_29*S10+ROcp5_88*C10;
    ROcp5_910 = ROcp5_39*S10+ROcp5_98*C10;
    RLcp5_17 = ROcp5_46*s->dpt[2][2];
    RLcp5_27 = ROcp5_56*s->dpt[2][2];
    RLcp5_37 = ROcp5_66*s->dpt[2][2];
    OMcp5_17 = OMcp5_16+ROcp5_46*qd[7];
    OMcp5_27 = OMcp5_26+ROcp5_56*qd[7];
    OMcp5_37 = OMcp5_36+ROcp5_66*qd[7];
    RLcp5_18 = ROcp5_46*s->dpt[2][6];
    RLcp5_28 = ROcp5_56*s->dpt[2][6];
    RLcp5_38 = ROcp5_66*s->dpt[2][6];
    OMcp5_18 = OMcp5_17+ROcp5_17*qd[8];
    OMcp5_28 = OMcp5_27+ROcp5_27*qd[8];
    OMcp5_38 = OMcp5_37+ROcp5_37*qd[8];
    RLcp5_19 = ROcp5_78*s->dpt[3][8];
    RLcp5_29 = ROcp5_88*s->dpt[3][8];
    RLcp5_39 = ROcp5_98*s->dpt[3][8];
    OMcp5_19 = OMcp5_18+ROcp5_78*qd[9];
    OMcp5_29 = OMcp5_28+ROcp5_88*qd[9];
    OMcp5_39 = OMcp5_38+ROcp5_98*qd[9];
    RLcp5_110 = ROcp5_78*s->dpt[3][11];
    RLcp5_210 = ROcp5_88*s->dpt[3][11];
    RLcp5_310 = ROcp5_98*s->dpt[3][11];
    OMcp5_110 = OMcp5_19+ROcp5_49*qd[10];
    OMcp5_210 = OMcp5_29+ROcp5_59*qd[10];
    OMcp5_310 = OMcp5_39+ROcp5_69*qd[10];
    RLcp5_111 = ROcp5_710*s->dpt[3][14];
    RLcp5_211 = ROcp5_810*s->dpt[3][14];
    RLcp5_311 = ROcp5_910*s->dpt[3][14];
    OMcp5_111 = OMcp5_110+ROcp5_110*qd[11];
    OMcp5_211 = OMcp5_210+ROcp5_210*qd[11];
    OMcp5_311 = OMcp5_310+ROcp5_310*qd[11];
    RLcp5_137 = ROcp5_110*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_49*C11+ROcp5_710*S11)-s->dpt[3][17]*(ROcp5_49*S11-ROcp5_710*
 C11);
    RLcp5_237 = ROcp5_210*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_59*C11+ROcp5_810*S11)-s->dpt[3][17]*(ROcp5_59*S11-ROcp5_810*
 C11);
    RLcp5_337 = ROcp5_310*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_69*C11+ROcp5_910*S11)-s->dpt[3][17]*(ROcp5_69*S11-ROcp5_910*
 C11);
    POcp5_137 = RLcp5_110+RLcp5_111+RLcp5_137+RLcp5_17+RLcp5_18+RLcp5_19+q[1];
    POcp5_237 = RLcp5_210+RLcp5_211+RLcp5_237+RLcp5_27+RLcp5_28+RLcp5_29+q[2];
    POcp5_337 = RLcp5_310+RLcp5_311+RLcp5_337+RLcp5_37+RLcp5_38+RLcp5_39+q[3];
    VIcp5_137 = qd[1]+OMcp5_210*RLcp5_311+OMcp5_211*RLcp5_337+OMcp5_26*RLcp5_37+OMcp5_27*RLcp5_38+OMcp5_28*RLcp5_39+
 OMcp5_29*RLcp5_310-OMcp5_310*RLcp5_211-OMcp5_311*RLcp5_237-OMcp5_36*RLcp5_27-OMcp5_37*RLcp5_28-OMcp5_38*RLcp5_29-OMcp5_39*
 RLcp5_210;
    VIcp5_237 = qd[2]-OMcp5_110*RLcp5_311-OMcp5_111*RLcp5_337-OMcp5_16*RLcp5_37-OMcp5_17*RLcp5_38-OMcp5_18*RLcp5_39-
 OMcp5_19*RLcp5_310+OMcp5_310*RLcp5_111+OMcp5_311*RLcp5_137+OMcp5_36*RLcp5_17+OMcp5_37*RLcp5_18+OMcp5_38*RLcp5_19+OMcp5_39*
 RLcp5_110;
    VIcp5_337 = qd[3]+OMcp5_110*RLcp5_211+OMcp5_111*RLcp5_237+OMcp5_16*RLcp5_27+OMcp5_17*RLcp5_28+OMcp5_18*RLcp5_29+
 OMcp5_19*RLcp5_210-OMcp5_210*RLcp5_111-OMcp5_211*RLcp5_137-OMcp5_26*RLcp5_17-OMcp5_27*RLcp5_18-OMcp5_28*RLcp5_19-OMcp5_29*
 RLcp5_110;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_137;
    sens->P[2] = POcp5_237;
    sens->P[3] = POcp5_337;
    sens->V[1] = VIcp5_137;
    sens->V[2] = VIcp5_237;
    sens->V[3] = VIcp5_337;
    sens->OM[1] = OMcp5_111;
    sens->OM[2] = OMcp5_211;
    sens->OM[3] = OMcp5_311;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_16 = qd[4]+qd[6]*S5;
    OMcp6_26 = ROcp6_85*qd[6]+qd[5]*C4;
    OMcp6_36 = ROcp6_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_7_0_2 = = 
 
// Sensor Kinematics 


    ROcp6_17 = ROcp6_16*C7-S5*S7;
    ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
    ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
    ROcp6_77 = ROcp6_16*S7+S5*C7;
    ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
    ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
    ROcp6_48 = ROcp6_46*C8+ROcp6_77*S8;
    ROcp6_58 = ROcp6_56*C8+ROcp6_87*S8;
    ROcp6_68 = ROcp6_66*C8+ROcp6_97*S8;
    ROcp6_78 = -(ROcp6_46*S8-ROcp6_77*C8);
    ROcp6_88 = -(ROcp6_56*S8-ROcp6_87*C8);
    ROcp6_98 = -(ROcp6_66*S8-ROcp6_97*C8);
    ROcp6_19 = ROcp6_17*C9+ROcp6_48*S9;
    ROcp6_29 = ROcp6_27*C9+ROcp6_58*S9;
    ROcp6_39 = ROcp6_37*C9+ROcp6_68*S9;
    ROcp6_49 = -(ROcp6_17*S9-ROcp6_48*C9);
    ROcp6_59 = -(ROcp6_27*S9-ROcp6_58*C9);
    ROcp6_69 = -(ROcp6_37*S9-ROcp6_68*C9);
    ROcp6_110 = ROcp6_19*C10-ROcp6_78*S10;
    ROcp6_210 = ROcp6_29*C10-ROcp6_88*S10;
    ROcp6_310 = ROcp6_39*C10-ROcp6_98*S10;
    ROcp6_710 = ROcp6_19*S10+ROcp6_78*C10;
    ROcp6_810 = ROcp6_29*S10+ROcp6_88*C10;
    ROcp6_910 = ROcp6_39*S10+ROcp6_98*C10;
    ROcp6_711 = -(ROcp6_49*S11-ROcp6_710*C11);
    ROcp6_811 = -(ROcp6_59*S11-ROcp6_810*C11);
    ROcp6_911 = -(ROcp6_69*S11-ROcp6_910*C11);
    RLcp6_17 = ROcp6_46*s->dpt[2][2];
    RLcp6_27 = ROcp6_56*s->dpt[2][2];
    RLcp6_37 = ROcp6_66*s->dpt[2][2];
    OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
    OMcp6_37 = OMcp6_36+ROcp6_66*qd[7];
    RLcp6_18 = ROcp6_46*s->dpt[2][6];
    RLcp6_28 = ROcp6_56*s->dpt[2][6];
    RLcp6_38 = ROcp6_66*s->dpt[2][6];
    OMcp6_18 = OMcp6_17+ROcp6_17*qd[8];
    OMcp6_28 = OMcp6_27+ROcp6_27*qd[8];
    OMcp6_38 = OMcp6_37+ROcp6_37*qd[8];
    RLcp6_19 = ROcp6_78*s->dpt[3][8];
    RLcp6_29 = ROcp6_88*s->dpt[3][8];
    RLcp6_39 = ROcp6_98*s->dpt[3][8];
    OMcp6_19 = OMcp6_18+ROcp6_78*qd[9];
    OMcp6_29 = OMcp6_28+ROcp6_88*qd[9];
    OMcp6_39 = OMcp6_38+ROcp6_98*qd[9];
    RLcp6_110 = ROcp6_78*s->dpt[3][11];
    RLcp6_210 = ROcp6_88*s->dpt[3][11];
    RLcp6_310 = ROcp6_98*s->dpt[3][11];
    OMcp6_110 = OMcp6_19+ROcp6_49*qd[10];
    OMcp6_210 = OMcp6_29+ROcp6_59*qd[10];
    OMcp6_310 = OMcp6_39+ROcp6_69*qd[10];
    RLcp6_111 = ROcp6_710*s->dpt[3][14];
    RLcp6_211 = ROcp6_810*s->dpt[3][14];
    RLcp6_311 = ROcp6_910*s->dpt[3][14];
    OMcp6_112 = OMcp6_110+ROcp6_110*qd[11]+qd[12]*(ROcp6_49*C11+ROcp6_710*S11);
    OMcp6_212 = OMcp6_210+ROcp6_210*qd[11]+qd[12]*(ROcp6_59*C11+ROcp6_810*S11);
    OMcp6_312 = OMcp6_310+ROcp6_310*qd[11]+qd[12]*(ROcp6_69*C11+ROcp6_910*S11);
    RLcp6_138 = s->dpt[1][18]*(ROcp6_110*C12-ROcp6_711*S12)+s->dpt[3][18]*(ROcp6_110*S12+ROcp6_711*C12);
    RLcp6_238 = s->dpt[1][18]*(ROcp6_210*C12-ROcp6_811*S12)+s->dpt[3][18]*(ROcp6_210*S12+ROcp6_811*C12);
    RLcp6_338 = s->dpt[1][18]*(ROcp6_310*C12-ROcp6_911*S12)+s->dpt[3][18]*(ROcp6_310*S12+ROcp6_911*C12);
    POcp6_138 = RLcp6_110+RLcp6_111+RLcp6_138+RLcp6_17+RLcp6_18+RLcp6_19+q[1];
    POcp6_238 = RLcp6_210+RLcp6_211+RLcp6_238+RLcp6_27+RLcp6_28+RLcp6_29+q[2];
    POcp6_338 = RLcp6_310+RLcp6_311+RLcp6_338+RLcp6_37+RLcp6_38+RLcp6_39+q[3];
    VIcp6_138 = qd[1]+OMcp6_210*RLcp6_311+OMcp6_212*RLcp6_338+OMcp6_26*RLcp6_37+OMcp6_27*RLcp6_38+OMcp6_28*RLcp6_39+
 OMcp6_29*RLcp6_310-OMcp6_310*RLcp6_211-OMcp6_312*RLcp6_238-OMcp6_36*RLcp6_27-OMcp6_37*RLcp6_28-OMcp6_38*RLcp6_29-OMcp6_39*
 RLcp6_210;
    VIcp6_238 = qd[2]-OMcp6_110*RLcp6_311-OMcp6_112*RLcp6_338-OMcp6_16*RLcp6_37-OMcp6_17*RLcp6_38-OMcp6_18*RLcp6_39-
 OMcp6_19*RLcp6_310+OMcp6_310*RLcp6_111+OMcp6_312*RLcp6_138+OMcp6_36*RLcp6_17+OMcp6_37*RLcp6_18+OMcp6_38*RLcp6_19+OMcp6_39*
 RLcp6_110;
    VIcp6_338 = qd[3]+OMcp6_110*RLcp6_211+OMcp6_112*RLcp6_238+OMcp6_16*RLcp6_27+OMcp6_17*RLcp6_28+OMcp6_18*RLcp6_29+
 OMcp6_19*RLcp6_210-OMcp6_210*RLcp6_111-OMcp6_212*RLcp6_138-OMcp6_26*RLcp6_17-OMcp6_27*RLcp6_18-OMcp6_28*RLcp6_19-OMcp6_29*
 RLcp6_110;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_138;
    sens->P[2] = POcp6_238;
    sens->P[3] = POcp6_338;
    sens->V[1] = VIcp6_138;
    sens->V[2] = VIcp6_238;
    sens->V[3] = VIcp6_338;
    sens->OM[1] = OMcp6_112;
    sens->OM[2] = OMcp6_212;
    sens->OM[3] = OMcp6_312;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_16 = qd[4]+qd[6]*S5;
    OMcp7_26 = ROcp7_85*qd[6]+qd[5]*C4;
    OMcp7_36 = ROcp7_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_8_0_3 = = 
 
// Sensor Kinematics 


    RLcp7_114 = ROcp7_46*s->dpt[2][3];
    RLcp7_214 = ROcp7_56*s->dpt[2][3];
    RLcp7_314 = ROcp7_66*s->dpt[2][3];
    OMcp7_114 = OMcp7_16+ROcp7_46*qd[14];
    OMcp7_214 = OMcp7_26+ROcp7_56*qd[14];
    OMcp7_314 = OMcp7_36+ROcp7_66*qd[14];
    RLcp7_139 = ROcp7_46*s->dpt[2][23]+s->dpt[1][23]*(ROcp7_16*C14-S14*S5)+s->dpt[3][23]*(ROcp7_16*S14+C14*S5);
    RLcp7_239 = ROcp7_56*s->dpt[2][23]+s->dpt[1][23]*(ROcp7_26*C14-ROcp7_85*S14)+s->dpt[3][23]*(ROcp7_26*S14+ROcp7_85*C14);
    RLcp7_339 = ROcp7_66*s->dpt[2][23]+s->dpt[1][23]*(ROcp7_36*C14-ROcp7_95*S14)+s->dpt[3][23]*(ROcp7_36*S14+ROcp7_95*C14);
    POcp7_139 = RLcp7_114+RLcp7_139+q[1];
    POcp7_239 = RLcp7_214+RLcp7_239+q[2];
    POcp7_339 = RLcp7_314+RLcp7_339+q[3];
    VIcp7_139 = qd[1]+OMcp7_214*RLcp7_339+OMcp7_26*RLcp7_314-OMcp7_314*RLcp7_239-OMcp7_36*RLcp7_214;
    VIcp7_239 = qd[2]-OMcp7_114*RLcp7_339-OMcp7_16*RLcp7_314+OMcp7_314*RLcp7_139+OMcp7_36*RLcp7_114;
    VIcp7_339 = qd[3]+OMcp7_114*RLcp7_239+OMcp7_16*RLcp7_214-OMcp7_214*RLcp7_139-OMcp7_26*RLcp7_114;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_139;
    sens->P[2] = POcp7_239;
    sens->P[3] = POcp7_339;
    sens->V[1] = VIcp7_139;
    sens->V[2] = VIcp7_239;
    sens->V[3] = VIcp7_339;
    sens->OM[1] = OMcp7_114;
    sens->OM[2] = OMcp7_214;
    sens->OM[3] = OMcp7_314;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


    ROcp8_25 = S4*S5;
    ROcp8_35 = -C4*S5;
    ROcp8_85 = -S4*C5;
    ROcp8_95 = C4*C5;
    ROcp8_16 = C5*C6;
    ROcp8_26 = ROcp8_25*C6+C4*S6;
    ROcp8_36 = ROcp8_35*C6+S4*S6;
    ROcp8_46 = -C5*S6;
    ROcp8_56 = -(ROcp8_25*S6-C4*C6);
    ROcp8_66 = -(ROcp8_35*S6-S4*C6);
    OMcp8_16 = qd[4]+qd[6]*S5;
    OMcp8_26 = ROcp8_85*qd[6]+qd[5]*C4;
    OMcp8_36 = ROcp8_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_9_0_3 = = 
 
// Sensor Kinematics 


    ROcp8_114 = ROcp8_16*C14-S14*S5;
    ROcp8_214 = ROcp8_26*C14-ROcp8_85*S14;
    ROcp8_314 = ROcp8_36*C14-ROcp8_95*S14;
    ROcp8_714 = ROcp8_16*S14+C14*S5;
    ROcp8_814 = ROcp8_26*S14+ROcp8_85*C14;
    ROcp8_914 = ROcp8_36*S14+ROcp8_95*C14;
    RLcp8_114 = ROcp8_46*s->dpt[2][3];
    RLcp8_214 = ROcp8_56*s->dpt[2][3];
    RLcp8_314 = ROcp8_66*s->dpt[2][3];
    OMcp8_114 = OMcp8_16+ROcp8_46*qd[14];
    OMcp8_214 = OMcp8_26+ROcp8_56*qd[14];
    OMcp8_314 = OMcp8_36+ROcp8_66*qd[14];
    RLcp8_115 = ROcp8_46*s->dpt[2][22];
    RLcp8_215 = ROcp8_56*s->dpt[2][22];
    RLcp8_315 = ROcp8_66*s->dpt[2][22];
    OMcp8_115 = OMcp8_114+ROcp8_114*qd[15];
    OMcp8_215 = OMcp8_214+ROcp8_214*qd[15];
    OMcp8_315 = OMcp8_314+ROcp8_314*qd[15];
    RLcp8_140 = ROcp8_114*s->dpt[1][25]+s->dpt[2][25]*(ROcp8_46*C15+ROcp8_714*S15)-s->dpt[3][25]*(ROcp8_46*S15-ROcp8_714*
 C15);
    RLcp8_240 = ROcp8_214*s->dpt[1][25]+s->dpt[2][25]*(ROcp8_56*C15+ROcp8_814*S15)-s->dpt[3][25]*(ROcp8_56*S15-ROcp8_814*
 C15);
    RLcp8_340 = ROcp8_314*s->dpt[1][25]+s->dpt[2][25]*(ROcp8_66*C15+ROcp8_914*S15)-s->dpt[3][25]*(ROcp8_66*S15-ROcp8_914*
 C15);
    POcp8_140 = RLcp8_114+RLcp8_115+RLcp8_140+q[1];
    POcp8_240 = RLcp8_214+RLcp8_215+RLcp8_240+q[2];
    POcp8_340 = RLcp8_314+RLcp8_315+RLcp8_340+q[3];
    VIcp8_140 = qd[1]+OMcp8_214*RLcp8_315+OMcp8_215*RLcp8_340+OMcp8_26*RLcp8_314-OMcp8_314*RLcp8_215-OMcp8_315*RLcp8_240-
 OMcp8_36*RLcp8_214;
    VIcp8_240 = qd[2]-OMcp8_114*RLcp8_315-OMcp8_115*RLcp8_340-OMcp8_16*RLcp8_314+OMcp8_314*RLcp8_115+OMcp8_315*RLcp8_140+
 OMcp8_36*RLcp8_114;
    VIcp8_340 = qd[3]+OMcp8_114*RLcp8_215+OMcp8_115*RLcp8_240+OMcp8_16*RLcp8_214-OMcp8_214*RLcp8_115-OMcp8_215*RLcp8_140-
 OMcp8_26*RLcp8_114;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_140;
    sens->P[2] = POcp8_240;
    sens->P[3] = POcp8_340;
    sens->V[1] = VIcp8_140;
    sens->V[2] = VIcp8_240;
    sens->V[3] = VIcp8_340;
    sens->OM[1] = OMcp8_115;
    sens->OM[2] = OMcp8_215;
    sens->OM[3] = OMcp8_315;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


    ROcp9_25 = S4*S5;
    ROcp9_35 = -C4*S5;
    ROcp9_85 = -S4*C5;
    ROcp9_95 = C4*C5;
    ROcp9_16 = C5*C6;
    ROcp9_26 = ROcp9_25*C6+C4*S6;
    ROcp9_36 = ROcp9_35*C6+S4*S6;
    ROcp9_46 = -C5*S6;
    ROcp9_56 = -(ROcp9_25*S6-C4*C6);
    ROcp9_66 = -(ROcp9_35*S6-S4*C6);
    OMcp9_16 = qd[4]+qd[6]*S5;
    OMcp9_26 = ROcp9_85*qd[6]+qd[5]*C4;
    OMcp9_36 = ROcp9_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_10_0_3 = = 
 
// Sensor Kinematics 


    ROcp9_114 = ROcp9_16*C14-S14*S5;
    ROcp9_214 = ROcp9_26*C14-ROcp9_85*S14;
    ROcp9_314 = ROcp9_36*C14-ROcp9_95*S14;
    ROcp9_714 = ROcp9_16*S14+C14*S5;
    ROcp9_814 = ROcp9_26*S14+ROcp9_85*C14;
    ROcp9_914 = ROcp9_36*S14+ROcp9_95*C14;
    ROcp9_415 = ROcp9_46*C15+ROcp9_714*S15;
    ROcp9_515 = ROcp9_56*C15+ROcp9_814*S15;
    ROcp9_615 = ROcp9_66*C15+ROcp9_914*S15;
    ROcp9_715 = -(ROcp9_46*S15-ROcp9_714*C15);
    ROcp9_815 = -(ROcp9_56*S15-ROcp9_814*C15);
    ROcp9_915 = -(ROcp9_66*S15-ROcp9_914*C15);
    RLcp9_114 = ROcp9_46*s->dpt[2][3];
    RLcp9_214 = ROcp9_56*s->dpt[2][3];
    RLcp9_314 = ROcp9_66*s->dpt[2][3];
    OMcp9_114 = OMcp9_16+ROcp9_46*qd[14];
    OMcp9_214 = OMcp9_26+ROcp9_56*qd[14];
    OMcp9_314 = OMcp9_36+ROcp9_66*qd[14];
    RLcp9_115 = ROcp9_46*s->dpt[2][22];
    RLcp9_215 = ROcp9_56*s->dpt[2][22];
    RLcp9_315 = ROcp9_66*s->dpt[2][22];
    OMcp9_115 = OMcp9_114+ROcp9_114*qd[15];
    OMcp9_215 = OMcp9_214+ROcp9_214*qd[15];
    OMcp9_315 = OMcp9_314+ROcp9_314*qd[15];
    RLcp9_116 = ROcp9_715*s->dpt[3][24];
    RLcp9_216 = ROcp9_815*s->dpt[3][24];
    RLcp9_316 = ROcp9_915*s->dpt[3][24];
    OMcp9_116 = OMcp9_115+ROcp9_715*qd[16];
    OMcp9_216 = OMcp9_215+ROcp9_815*qd[16];
    OMcp9_316 = OMcp9_315+ROcp9_915*qd[16];
    RLcp9_141 = ROcp9_715*s->dpt[3][28]+s->dpt[1][28]*(ROcp9_114*C16+ROcp9_415*S16)-s->dpt[2][28]*(ROcp9_114*S16-ROcp9_415
 *C16);
    RLcp9_241 = ROcp9_815*s->dpt[3][28]+s->dpt[1][28]*(ROcp9_214*C16+ROcp9_515*S16)-s->dpt[2][28]*(ROcp9_214*S16-ROcp9_515
 *C16);
    RLcp9_341 = ROcp9_915*s->dpt[3][28]+s->dpt[1][28]*(ROcp9_314*C16+ROcp9_615*S16)-s->dpt[2][28]*(ROcp9_314*S16-ROcp9_615
 *C16);
    POcp9_141 = RLcp9_114+RLcp9_115+RLcp9_116+RLcp9_141+q[1];
    POcp9_241 = RLcp9_214+RLcp9_215+RLcp9_216+RLcp9_241+q[2];
    POcp9_341 = RLcp9_314+RLcp9_315+RLcp9_316+RLcp9_341+q[3];
    VIcp9_141 = qd[1]+OMcp9_214*RLcp9_315+OMcp9_215*RLcp9_316+OMcp9_216*RLcp9_341+OMcp9_26*RLcp9_314-OMcp9_314*RLcp9_215-
 OMcp9_315*RLcp9_216-OMcp9_316*RLcp9_241-OMcp9_36*RLcp9_214;
    VIcp9_241 = qd[2]-OMcp9_114*RLcp9_315-OMcp9_115*RLcp9_316-OMcp9_116*RLcp9_341-OMcp9_16*RLcp9_314+OMcp9_314*RLcp9_115+
 OMcp9_315*RLcp9_116+OMcp9_316*RLcp9_141+OMcp9_36*RLcp9_114;
    VIcp9_341 = qd[3]+OMcp9_114*RLcp9_215+OMcp9_115*RLcp9_216+OMcp9_116*RLcp9_241+OMcp9_16*RLcp9_214-OMcp9_214*RLcp9_115-
 OMcp9_215*RLcp9_116-OMcp9_216*RLcp9_141-OMcp9_26*RLcp9_114;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_141;
    sens->P[2] = POcp9_241;
    sens->P[3] = POcp9_341;
    sens->V[1] = VIcp9_141;
    sens->V[2] = VIcp9_241;
    sens->V[3] = VIcp9_341;
    sens->OM[1] = OMcp9_116;
    sens->OM[2] = OMcp9_216;
    sens->OM[3] = OMcp9_316;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


    ROcp10_25 = S4*S5;
    ROcp10_35 = -C4*S5;
    ROcp10_85 = -S4*C5;
    ROcp10_95 = C4*C5;
    ROcp10_16 = C5*C6;
    ROcp10_26 = ROcp10_25*C6+C4*S6;
    ROcp10_36 = ROcp10_35*C6+S4*S6;
    ROcp10_46 = -C5*S6;
    ROcp10_56 = -(ROcp10_25*S6-C4*C6);
    ROcp10_66 = -(ROcp10_35*S6-S4*C6);
    OMcp10_16 = qd[4]+qd[6]*S5;
    OMcp10_26 = ROcp10_85*qd[6]+qd[5]*C4;
    OMcp10_36 = ROcp10_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_11_0_3 = = 
 
// Sensor Kinematics 


    ROcp10_114 = ROcp10_16*C14-S14*S5;
    ROcp10_214 = ROcp10_26*C14-ROcp10_85*S14;
    ROcp10_314 = ROcp10_36*C14-ROcp10_95*S14;
    ROcp10_714 = ROcp10_16*S14+C14*S5;
    ROcp10_814 = ROcp10_26*S14+ROcp10_85*C14;
    ROcp10_914 = ROcp10_36*S14+ROcp10_95*C14;
    ROcp10_415 = ROcp10_46*C15+ROcp10_714*S15;
    ROcp10_515 = ROcp10_56*C15+ROcp10_814*S15;
    ROcp10_615 = ROcp10_66*C15+ROcp10_914*S15;
    ROcp10_715 = -(ROcp10_46*S15-ROcp10_714*C15);
    ROcp10_815 = -(ROcp10_56*S15-ROcp10_814*C15);
    ROcp10_915 = -(ROcp10_66*S15-ROcp10_914*C15);
    ROcp10_116 = ROcp10_114*C16+ROcp10_415*S16;
    ROcp10_216 = ROcp10_214*C16+ROcp10_515*S16;
    ROcp10_316 = ROcp10_314*C16+ROcp10_615*S16;
    ROcp10_416 = -(ROcp10_114*S16-ROcp10_415*C16);
    ROcp10_516 = -(ROcp10_214*S16-ROcp10_515*C16);
    ROcp10_616 = -(ROcp10_314*S16-ROcp10_615*C16);
    RLcp10_114 = ROcp10_46*s->dpt[2][3];
    RLcp10_214 = ROcp10_56*s->dpt[2][3];
    RLcp10_314 = ROcp10_66*s->dpt[2][3];
    OMcp10_114 = OMcp10_16+ROcp10_46*qd[14];
    OMcp10_214 = OMcp10_26+ROcp10_56*qd[14];
    OMcp10_314 = OMcp10_36+ROcp10_66*qd[14];
    RLcp10_115 = ROcp10_46*s->dpt[2][22];
    RLcp10_215 = ROcp10_56*s->dpt[2][22];
    RLcp10_315 = ROcp10_66*s->dpt[2][22];
    OMcp10_115 = OMcp10_114+ROcp10_114*qd[15];
    OMcp10_215 = OMcp10_214+ROcp10_214*qd[15];
    OMcp10_315 = OMcp10_314+ROcp10_314*qd[15];
    RLcp10_116 = ROcp10_715*s->dpt[3][24];
    RLcp10_216 = ROcp10_815*s->dpt[3][24];
    RLcp10_316 = ROcp10_915*s->dpt[3][24];
    OMcp10_116 = OMcp10_115+ROcp10_715*qd[16];
    OMcp10_216 = OMcp10_215+ROcp10_815*qd[16];
    OMcp10_316 = OMcp10_315+ROcp10_915*qd[16];
    RLcp10_117 = ROcp10_715*s->dpt[3][27];
    RLcp10_217 = ROcp10_815*s->dpt[3][27];
    RLcp10_317 = ROcp10_915*s->dpt[3][27];
    OMcp10_117 = OMcp10_116+ROcp10_416*qd[17];
    OMcp10_217 = OMcp10_216+ROcp10_516*qd[17];
    OMcp10_317 = OMcp10_316+ROcp10_616*qd[17];
    RLcp10_142 = ROcp10_416*s->dpt[2][31]+s->dpt[1][31]*(ROcp10_116*C17-ROcp10_715*S17)+s->dpt[3][31]*(ROcp10_116*S17+
 ROcp10_715*C17);
    RLcp10_242 = ROcp10_516*s->dpt[2][31]+s->dpt[1][31]*(ROcp10_216*C17-ROcp10_815*S17)+s->dpt[3][31]*(ROcp10_216*S17+
 ROcp10_815*C17);
    RLcp10_342 = ROcp10_616*s->dpt[2][31]+s->dpt[1][31]*(ROcp10_316*C17-ROcp10_915*S17)+s->dpt[3][31]*(ROcp10_316*S17+
 ROcp10_915*C17);
    POcp10_142 = RLcp10_114+RLcp10_115+RLcp10_116+RLcp10_117+RLcp10_142+q[1];
    POcp10_242 = RLcp10_214+RLcp10_215+RLcp10_216+RLcp10_217+RLcp10_242+q[2];
    POcp10_342 = RLcp10_314+RLcp10_315+RLcp10_316+RLcp10_317+RLcp10_342+q[3];
    VIcp10_142 = qd[1]+OMcp10_214*RLcp10_315+OMcp10_215*RLcp10_316+OMcp10_216*RLcp10_317+OMcp10_217*RLcp10_342+OMcp10_26*
 RLcp10_314-OMcp10_314*RLcp10_215-OMcp10_315*RLcp10_216-OMcp10_316*RLcp10_217-OMcp10_317*RLcp10_242-OMcp10_36*RLcp10_214;
    VIcp10_242 = qd[2]-OMcp10_114*RLcp10_315-OMcp10_115*RLcp10_316-OMcp10_116*RLcp10_317-OMcp10_117*RLcp10_342-OMcp10_16*
 RLcp10_314+OMcp10_314*RLcp10_115+OMcp10_315*RLcp10_116+OMcp10_316*RLcp10_117+OMcp10_317*RLcp10_142+OMcp10_36*RLcp10_114;
    VIcp10_342 = qd[3]+OMcp10_114*RLcp10_215+OMcp10_115*RLcp10_216+OMcp10_116*RLcp10_217+OMcp10_117*RLcp10_242+OMcp10_16*
 RLcp10_214-OMcp10_214*RLcp10_115-OMcp10_215*RLcp10_116-OMcp10_216*RLcp10_117-OMcp10_217*RLcp10_142-OMcp10_26*RLcp10_114;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_142;
    sens->P[2] = POcp10_242;
    sens->P[3] = POcp10_342;
    sens->V[1] = VIcp10_142;
    sens->V[2] = VIcp10_242;
    sens->V[3] = VIcp10_342;
    sens->OM[1] = OMcp10_117;
    sens->OM[2] = OMcp10_217;
    sens->OM[3] = OMcp10_317;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


    ROcp11_25 = S4*S5;
    ROcp11_35 = -C4*S5;
    ROcp11_85 = -S4*C5;
    ROcp11_95 = C4*C5;
    ROcp11_16 = C5*C6;
    ROcp11_26 = ROcp11_25*C6+C4*S6;
    ROcp11_36 = ROcp11_35*C6+S4*S6;
    ROcp11_46 = -C5*S6;
    ROcp11_56 = -(ROcp11_25*S6-C4*C6);
    ROcp11_66 = -(ROcp11_35*S6-S4*C6);
    OMcp11_16 = qd[4]+qd[6]*S5;
    OMcp11_26 = ROcp11_85*qd[6]+qd[5]*C4;
    OMcp11_36 = ROcp11_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_12_0_3 = = 
 
// Sensor Kinematics 


    ROcp11_114 = ROcp11_16*C14-S14*S5;
    ROcp11_214 = ROcp11_26*C14-ROcp11_85*S14;
    ROcp11_314 = ROcp11_36*C14-ROcp11_95*S14;
    ROcp11_714 = ROcp11_16*S14+C14*S5;
    ROcp11_814 = ROcp11_26*S14+ROcp11_85*C14;
    ROcp11_914 = ROcp11_36*S14+ROcp11_95*C14;
    ROcp11_415 = ROcp11_46*C15+ROcp11_714*S15;
    ROcp11_515 = ROcp11_56*C15+ROcp11_814*S15;
    ROcp11_615 = ROcp11_66*C15+ROcp11_914*S15;
    ROcp11_715 = -(ROcp11_46*S15-ROcp11_714*C15);
    ROcp11_815 = -(ROcp11_56*S15-ROcp11_814*C15);
    ROcp11_915 = -(ROcp11_66*S15-ROcp11_914*C15);
    ROcp11_116 = ROcp11_114*C16+ROcp11_415*S16;
    ROcp11_216 = ROcp11_214*C16+ROcp11_515*S16;
    ROcp11_316 = ROcp11_314*C16+ROcp11_615*S16;
    ROcp11_416 = -(ROcp11_114*S16-ROcp11_415*C16);
    ROcp11_516 = -(ROcp11_214*S16-ROcp11_515*C16);
    ROcp11_616 = -(ROcp11_314*S16-ROcp11_615*C16);
    ROcp11_117 = ROcp11_116*C17-ROcp11_715*S17;
    ROcp11_217 = ROcp11_216*C17-ROcp11_815*S17;
    ROcp11_317 = ROcp11_316*C17-ROcp11_915*S17;
    ROcp11_717 = ROcp11_116*S17+ROcp11_715*C17;
    ROcp11_817 = ROcp11_216*S17+ROcp11_815*C17;
    ROcp11_917 = ROcp11_316*S17+ROcp11_915*C17;
    RLcp11_114 = ROcp11_46*s->dpt[2][3];
    RLcp11_214 = ROcp11_56*s->dpt[2][3];
    RLcp11_314 = ROcp11_66*s->dpt[2][3];
    OMcp11_114 = OMcp11_16+ROcp11_46*qd[14];
    OMcp11_214 = OMcp11_26+ROcp11_56*qd[14];
    OMcp11_314 = OMcp11_36+ROcp11_66*qd[14];
    RLcp11_115 = ROcp11_46*s->dpt[2][22];
    RLcp11_215 = ROcp11_56*s->dpt[2][22];
    RLcp11_315 = ROcp11_66*s->dpt[2][22];
    OMcp11_115 = OMcp11_114+ROcp11_114*qd[15];
    OMcp11_215 = OMcp11_214+ROcp11_214*qd[15];
    OMcp11_315 = OMcp11_314+ROcp11_314*qd[15];
    RLcp11_116 = ROcp11_715*s->dpt[3][24];
    RLcp11_216 = ROcp11_815*s->dpt[3][24];
    RLcp11_316 = ROcp11_915*s->dpt[3][24];
    OMcp11_116 = OMcp11_115+ROcp11_715*qd[16];
    OMcp11_216 = OMcp11_215+ROcp11_815*qd[16];
    OMcp11_316 = OMcp11_315+ROcp11_915*qd[16];
    RLcp11_117 = ROcp11_715*s->dpt[3][27];
    RLcp11_217 = ROcp11_815*s->dpt[3][27];
    RLcp11_317 = ROcp11_915*s->dpt[3][27];
    OMcp11_117 = OMcp11_116+ROcp11_416*qd[17];
    OMcp11_217 = OMcp11_216+ROcp11_516*qd[17];
    OMcp11_317 = OMcp11_316+ROcp11_616*qd[17];
    RLcp11_118 = ROcp11_717*s->dpt[3][30];
    RLcp11_218 = ROcp11_817*s->dpt[3][30];
    RLcp11_318 = ROcp11_917*s->dpt[3][30];
    OMcp11_118 = OMcp11_117+ROcp11_117*qd[18];
    OMcp11_218 = OMcp11_217+ROcp11_217*qd[18];
    OMcp11_318 = OMcp11_317+ROcp11_317*qd[18];
    RLcp11_143 = ROcp11_117*s->dpt[1][33]+s->dpt[2][33]*(ROcp11_416*C18+ROcp11_717*S18)-s->dpt[3][33]*(ROcp11_416*S18-
 ROcp11_717*C18);
    RLcp11_243 = ROcp11_217*s->dpt[1][33]+s->dpt[2][33]*(ROcp11_516*C18+ROcp11_817*S18)-s->dpt[3][33]*(ROcp11_516*S18-
 ROcp11_817*C18);
    RLcp11_343 = ROcp11_317*s->dpt[1][33]+s->dpt[2][33]*(ROcp11_616*C18+ROcp11_917*S18)-s->dpt[3][33]*(ROcp11_616*S18-
 ROcp11_917*C18);
    POcp11_143 = RLcp11_114+RLcp11_115+RLcp11_116+RLcp11_117+RLcp11_118+RLcp11_143+q[1];
    POcp11_243 = RLcp11_214+RLcp11_215+RLcp11_216+RLcp11_217+RLcp11_218+RLcp11_243+q[2];
    POcp11_343 = RLcp11_314+RLcp11_315+RLcp11_316+RLcp11_317+RLcp11_318+RLcp11_343+q[3];
    VIcp11_143 = qd[1]+OMcp11_214*RLcp11_315+OMcp11_215*RLcp11_316+OMcp11_216*RLcp11_317+OMcp11_217*RLcp11_318+OMcp11_218*
 RLcp11_343+OMcp11_26*RLcp11_314-OMcp11_314*RLcp11_215-OMcp11_315*RLcp11_216-OMcp11_316*RLcp11_217-OMcp11_317*RLcp11_218-
 OMcp11_318*RLcp11_243-OMcp11_36*RLcp11_214;
    VIcp11_243 = qd[2]-OMcp11_114*RLcp11_315-OMcp11_115*RLcp11_316-OMcp11_116*RLcp11_317-OMcp11_117*RLcp11_318-OMcp11_118*
 RLcp11_343-OMcp11_16*RLcp11_314+OMcp11_314*RLcp11_115+OMcp11_315*RLcp11_116+OMcp11_316*RLcp11_117+OMcp11_317*RLcp11_118+
 OMcp11_318*RLcp11_143+OMcp11_36*RLcp11_114;
    VIcp11_343 = qd[3]+OMcp11_114*RLcp11_215+OMcp11_115*RLcp11_216+OMcp11_116*RLcp11_217+OMcp11_117*RLcp11_218+OMcp11_118*
 RLcp11_243+OMcp11_16*RLcp11_214-OMcp11_214*RLcp11_115-OMcp11_215*RLcp11_116-OMcp11_216*RLcp11_117-OMcp11_217*RLcp11_118-
 OMcp11_218*RLcp11_143-OMcp11_26*RLcp11_114;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_143;
    sens->P[2] = POcp11_243;
    sens->P[3] = POcp11_343;
    sens->V[1] = VIcp11_143;
    sens->V[2] = VIcp11_243;
    sens->V[3] = VIcp11_343;
    sens->OM[1] = OMcp11_118;
    sens->OM[2] = OMcp11_218;
    sens->OM[3] = OMcp11_318;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


    ROcp12_25 = S4*S5;
    ROcp12_35 = -C4*S5;
    ROcp12_85 = -S4*C5;
    ROcp12_95 = C4*C5;
    ROcp12_16 = C5*C6;
    ROcp12_26 = ROcp12_25*C6+C4*S6;
    ROcp12_36 = ROcp12_35*C6+S4*S6;
    ROcp12_46 = -C5*S6;
    ROcp12_56 = -(ROcp12_25*S6-C4*C6);
    ROcp12_66 = -(ROcp12_35*S6-S4*C6);
    OMcp12_16 = qd[4]+qd[6]*S5;
    OMcp12_26 = ROcp12_85*qd[6]+qd[5]*C4;
    OMcp12_36 = ROcp12_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_13_0_3 = = 
 
// Sensor Kinematics 


    ROcp12_114 = ROcp12_16*C14-S14*S5;
    ROcp12_214 = ROcp12_26*C14-ROcp12_85*S14;
    ROcp12_314 = ROcp12_36*C14-ROcp12_95*S14;
    ROcp12_714 = ROcp12_16*S14+C14*S5;
    ROcp12_814 = ROcp12_26*S14+ROcp12_85*C14;
    ROcp12_914 = ROcp12_36*S14+ROcp12_95*C14;
    ROcp12_415 = ROcp12_46*C15+ROcp12_714*S15;
    ROcp12_515 = ROcp12_56*C15+ROcp12_814*S15;
    ROcp12_615 = ROcp12_66*C15+ROcp12_914*S15;
    ROcp12_715 = -(ROcp12_46*S15-ROcp12_714*C15);
    ROcp12_815 = -(ROcp12_56*S15-ROcp12_814*C15);
    ROcp12_915 = -(ROcp12_66*S15-ROcp12_914*C15);
    ROcp12_116 = ROcp12_114*C16+ROcp12_415*S16;
    ROcp12_216 = ROcp12_214*C16+ROcp12_515*S16;
    ROcp12_316 = ROcp12_314*C16+ROcp12_615*S16;
    ROcp12_416 = -(ROcp12_114*S16-ROcp12_415*C16);
    ROcp12_516 = -(ROcp12_214*S16-ROcp12_515*C16);
    ROcp12_616 = -(ROcp12_314*S16-ROcp12_615*C16);
    ROcp12_117 = ROcp12_116*C17-ROcp12_715*S17;
    ROcp12_217 = ROcp12_216*C17-ROcp12_815*S17;
    ROcp12_317 = ROcp12_316*C17-ROcp12_915*S17;
    ROcp12_717 = ROcp12_116*S17+ROcp12_715*C17;
    ROcp12_817 = ROcp12_216*S17+ROcp12_815*C17;
    ROcp12_917 = ROcp12_316*S17+ROcp12_915*C17;
    ROcp12_718 = -(ROcp12_416*S18-ROcp12_717*C18);
    ROcp12_818 = -(ROcp12_516*S18-ROcp12_817*C18);
    ROcp12_918 = -(ROcp12_616*S18-ROcp12_917*C18);
    RLcp12_114 = ROcp12_46*s->dpt[2][3];
    RLcp12_214 = ROcp12_56*s->dpt[2][3];
    RLcp12_314 = ROcp12_66*s->dpt[2][3];
    OMcp12_114 = OMcp12_16+ROcp12_46*qd[14];
    OMcp12_214 = OMcp12_26+ROcp12_56*qd[14];
    OMcp12_314 = OMcp12_36+ROcp12_66*qd[14];
    RLcp12_115 = ROcp12_46*s->dpt[2][22];
    RLcp12_215 = ROcp12_56*s->dpt[2][22];
    RLcp12_315 = ROcp12_66*s->dpt[2][22];
    OMcp12_115 = OMcp12_114+ROcp12_114*qd[15];
    OMcp12_215 = OMcp12_214+ROcp12_214*qd[15];
    OMcp12_315 = OMcp12_314+ROcp12_314*qd[15];
    RLcp12_116 = ROcp12_715*s->dpt[3][24];
    RLcp12_216 = ROcp12_815*s->dpt[3][24];
    RLcp12_316 = ROcp12_915*s->dpt[3][24];
    OMcp12_116 = OMcp12_115+ROcp12_715*qd[16];
    OMcp12_216 = OMcp12_215+ROcp12_815*qd[16];
    OMcp12_316 = OMcp12_315+ROcp12_915*qd[16];
    RLcp12_117 = ROcp12_715*s->dpt[3][27];
    RLcp12_217 = ROcp12_815*s->dpt[3][27];
    RLcp12_317 = ROcp12_915*s->dpt[3][27];
    OMcp12_117 = OMcp12_116+ROcp12_416*qd[17];
    OMcp12_217 = OMcp12_216+ROcp12_516*qd[17];
    OMcp12_317 = OMcp12_316+ROcp12_616*qd[17];
    RLcp12_118 = ROcp12_717*s->dpt[3][30];
    RLcp12_218 = ROcp12_817*s->dpt[3][30];
    RLcp12_318 = ROcp12_917*s->dpt[3][30];
    OMcp12_119 = OMcp12_117+ROcp12_117*qd[18]+qd[19]*(ROcp12_416*C18+ROcp12_717*S18);
    OMcp12_219 = OMcp12_217+ROcp12_217*qd[18]+qd[19]*(ROcp12_516*C18+ROcp12_817*S18);
    OMcp12_319 = OMcp12_317+ROcp12_317*qd[18]+qd[19]*(ROcp12_616*C18+ROcp12_917*S18);
    RLcp12_144 = s->dpt[1][34]*(ROcp12_117*C19-ROcp12_718*S19)+s->dpt[3][34]*(ROcp12_117*S19+ROcp12_718*C19);
    RLcp12_244 = s->dpt[1][34]*(ROcp12_217*C19-ROcp12_818*S19)+s->dpt[3][34]*(ROcp12_217*S19+ROcp12_818*C19);
    RLcp12_344 = s->dpt[1][34]*(ROcp12_317*C19-ROcp12_918*S19)+s->dpt[3][34]*(ROcp12_317*S19+ROcp12_918*C19);
    POcp12_144 = RLcp12_114+RLcp12_115+RLcp12_116+RLcp12_117+RLcp12_118+RLcp12_144+q[1];
    POcp12_244 = RLcp12_214+RLcp12_215+RLcp12_216+RLcp12_217+RLcp12_218+RLcp12_244+q[2];
    POcp12_344 = RLcp12_314+RLcp12_315+RLcp12_316+RLcp12_317+RLcp12_318+RLcp12_344+q[3];
    VIcp12_144 = qd[1]+OMcp12_214*RLcp12_315+OMcp12_215*RLcp12_316+OMcp12_216*RLcp12_317+OMcp12_217*RLcp12_318+OMcp12_219*
 RLcp12_344+OMcp12_26*RLcp12_314-OMcp12_314*RLcp12_215-OMcp12_315*RLcp12_216-OMcp12_316*RLcp12_217-OMcp12_317*RLcp12_218-
 OMcp12_319*RLcp12_244-OMcp12_36*RLcp12_214;
    VIcp12_244 = qd[2]-OMcp12_114*RLcp12_315-OMcp12_115*RLcp12_316-OMcp12_116*RLcp12_317-OMcp12_117*RLcp12_318-OMcp12_119*
 RLcp12_344-OMcp12_16*RLcp12_314+OMcp12_314*RLcp12_115+OMcp12_315*RLcp12_116+OMcp12_316*RLcp12_117+OMcp12_317*RLcp12_118+
 OMcp12_319*RLcp12_144+OMcp12_36*RLcp12_114;
    VIcp12_344 = qd[3]+OMcp12_114*RLcp12_215+OMcp12_115*RLcp12_216+OMcp12_116*RLcp12_217+OMcp12_117*RLcp12_218+OMcp12_119*
 RLcp12_244+OMcp12_16*RLcp12_214-OMcp12_214*RLcp12_115-OMcp12_215*RLcp12_116-OMcp12_216*RLcp12_117-OMcp12_217*RLcp12_118-
 OMcp12_219*RLcp12_144-OMcp12_26*RLcp12_114;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_144;
    sens->P[2] = POcp12_244;
    sens->P[3] = POcp12_344;
    sens->V[1] = VIcp12_144;
    sens->V[2] = VIcp12_244;
    sens->V[3] = VIcp12_344;
    sens->OM[1] = OMcp12_119;
    sens->OM[2] = OMcp12_219;
    sens->OM[3] = OMcp12_319;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


    ROcp13_25 = S4*S5;
    ROcp13_35 = -C4*S5;
    ROcp13_85 = -S4*C5;
    ROcp13_95 = C4*C5;
    ROcp13_16 = C5*C6;
    ROcp13_26 = ROcp13_25*C6+C4*S6;
    ROcp13_36 = ROcp13_35*C6+S4*S6;
    ROcp13_46 = -C5*S6;
    ROcp13_56 = -(ROcp13_25*S6-C4*C6);
    ROcp13_66 = -(ROcp13_35*S6-S4*C6);
    OMcp13_16 = qd[4]+qd[6]*S5;
    OMcp13_26 = ROcp13_85*qd[6]+qd[5]*C4;
    OMcp13_36 = ROcp13_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_14_0_4 = = 
 
// Sensor Kinematics 


    RLcp13_121 = ROcp13_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp13_221 = ROcp13_26*s->dpt[1][4]+ROcp13_85*s->dpt[3][4];
    RLcp13_321 = ROcp13_36*s->dpt[1][4]+ROcp13_95*s->dpt[3][4];
    OMcp13_121 = OMcp13_16+ROcp13_16*qd[21];
    OMcp13_221 = OMcp13_26+ROcp13_26*qd[21];
    OMcp13_321 = OMcp13_36+ROcp13_36*qd[21];
    RLcp13_145 = ROcp13_16*s->dpt[1][39]+s->dpt[2][39]*(ROcp13_46*C21+S21*S5)-s->dpt[3][39]*(ROcp13_46*S21-C21*S5);
    RLcp13_245 = ROcp13_26*s->dpt[1][39]+s->dpt[2][39]*(ROcp13_56*C21+ROcp13_85*S21)-s->dpt[3][39]*(ROcp13_56*S21-
 ROcp13_85*C21);
    RLcp13_345 = ROcp13_36*s->dpt[1][39]+s->dpt[2][39]*(ROcp13_66*C21+ROcp13_95*S21)-s->dpt[3][39]*(ROcp13_66*S21-
 ROcp13_95*C21);
    POcp13_145 = RLcp13_121+RLcp13_145+q[1];
    POcp13_245 = RLcp13_221+RLcp13_245+q[2];
    POcp13_345 = RLcp13_321+RLcp13_345+q[3];
    VIcp13_145 = qd[1]+OMcp13_221*RLcp13_345+OMcp13_26*RLcp13_321-OMcp13_321*RLcp13_245-OMcp13_36*RLcp13_221;
    VIcp13_245 = qd[2]-OMcp13_121*RLcp13_345-OMcp13_16*RLcp13_321+OMcp13_321*RLcp13_145+OMcp13_36*RLcp13_121;
    VIcp13_345 = qd[3]+OMcp13_121*RLcp13_245+OMcp13_16*RLcp13_221-OMcp13_221*RLcp13_145-OMcp13_26*RLcp13_121;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_145;
    sens->P[2] = POcp13_245;
    sens->P[3] = POcp13_345;
    sens->V[1] = VIcp13_145;
    sens->V[2] = VIcp13_245;
    sens->V[3] = VIcp13_345;
    sens->OM[1] = OMcp13_121;
    sens->OM[2] = OMcp13_221;
    sens->OM[3] = OMcp13_321;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


    ROcp14_25 = S4*S5;
    ROcp14_35 = -C4*S5;
    ROcp14_85 = -S4*C5;
    ROcp14_95 = C4*C5;
    ROcp14_16 = C5*C6;
    ROcp14_26 = ROcp14_25*C6+C4*S6;
    ROcp14_36 = ROcp14_35*C6+S4*S6;
    ROcp14_46 = -C5*S6;
    ROcp14_56 = -(ROcp14_25*S6-C4*C6);
    ROcp14_66 = -(ROcp14_35*S6-S4*C6);
    OMcp14_16 = qd[4]+qd[6]*S5;
    OMcp14_26 = ROcp14_85*qd[6]+qd[5]*C4;
    OMcp14_36 = ROcp14_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_15_0_4 = = 
 
// Sensor Kinematics 


    ROcp14_421 = ROcp14_46*C21+S21*S5;
    ROcp14_521 = ROcp14_56*C21+ROcp14_85*S21;
    ROcp14_621 = ROcp14_66*C21+ROcp14_95*S21;
    ROcp14_721 = -(ROcp14_46*S21-C21*S5);
    ROcp14_821 = -(ROcp14_56*S21-ROcp14_85*C21);
    ROcp14_921 = -(ROcp14_66*S21-ROcp14_95*C21);
    RLcp14_121 = ROcp14_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp14_221 = ROcp14_26*s->dpt[1][4]+ROcp14_85*s->dpt[3][4];
    RLcp14_321 = ROcp14_36*s->dpt[1][4]+ROcp14_95*s->dpt[3][4];
    OMcp14_122 = OMcp14_16+ROcp14_16*qd[21]+ROcp14_421*qd[22];
    OMcp14_222 = OMcp14_26+ROcp14_26*qd[21]+ROcp14_521*qd[22];
    OMcp14_322 = OMcp14_36+ROcp14_36*qd[21]+ROcp14_621*qd[22];
    RLcp14_146 = ROcp14_421*s->dpt[2][41]+s->dpt[1][41]*(ROcp14_16*C22-ROcp14_721*S22)+s->dpt[3][41]*(ROcp14_16*S22+
 ROcp14_721*C22);
    RLcp14_246 = ROcp14_521*s->dpt[2][41]+s->dpt[1][41]*(ROcp14_26*C22-ROcp14_821*S22)+s->dpt[3][41]*(ROcp14_26*S22+
 ROcp14_821*C22);
    RLcp14_346 = ROcp14_621*s->dpt[2][41]+s->dpt[1][41]*(ROcp14_36*C22-ROcp14_921*S22)+s->dpt[3][41]*(ROcp14_36*S22+
 ROcp14_921*C22);
    POcp14_146 = RLcp14_121+RLcp14_146+q[1];
    POcp14_246 = RLcp14_221+RLcp14_246+q[2];
    POcp14_346 = RLcp14_321+RLcp14_346+q[3];
    VIcp14_146 = qd[1]+OMcp14_222*RLcp14_346+OMcp14_26*RLcp14_321-OMcp14_322*RLcp14_246-OMcp14_36*RLcp14_221;
    VIcp14_246 = qd[2]-OMcp14_122*RLcp14_346-OMcp14_16*RLcp14_321+OMcp14_322*RLcp14_146+OMcp14_36*RLcp14_121;
    VIcp14_346 = qd[3]+OMcp14_122*RLcp14_246+OMcp14_16*RLcp14_221-OMcp14_222*RLcp14_146-OMcp14_26*RLcp14_121;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_146;
    sens->P[2] = POcp14_246;
    sens->P[3] = POcp14_346;
    sens->V[1] = VIcp14_146;
    sens->V[2] = VIcp14_246;
    sens->V[3] = VIcp14_346;
    sens->OM[1] = OMcp14_122;
    sens->OM[2] = OMcp14_222;
    sens->OM[3] = OMcp14_322;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


    ROcp15_25 = S4*S5;
    ROcp15_35 = -C4*S5;
    ROcp15_85 = -S4*C5;
    ROcp15_95 = C4*C5;
    ROcp15_16 = C5*C6;
    ROcp15_26 = ROcp15_25*C6+C4*S6;
    ROcp15_36 = ROcp15_35*C6+S4*S6;
    ROcp15_46 = -C5*S6;
    ROcp15_56 = -(ROcp15_25*S6-C4*C6);
    ROcp15_66 = -(ROcp15_35*S6-S4*C6);
    OMcp15_16 = qd[4]+qd[6]*S5;
    OMcp15_26 = ROcp15_85*qd[6]+qd[5]*C4;
    OMcp15_36 = ROcp15_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_16_0_4 = = 
 
// Sensor Kinematics 


    ROcp15_421 = ROcp15_46*C21+S21*S5;
    ROcp15_521 = ROcp15_56*C21+ROcp15_85*S21;
    ROcp15_621 = ROcp15_66*C21+ROcp15_95*S21;
    ROcp15_721 = -(ROcp15_46*S21-C21*S5);
    ROcp15_821 = -(ROcp15_56*S21-ROcp15_85*C21);
    ROcp15_921 = -(ROcp15_66*S21-ROcp15_95*C21);
    ROcp15_122 = ROcp15_16*C22-ROcp15_721*S22;
    ROcp15_222 = ROcp15_26*C22-ROcp15_821*S22;
    ROcp15_322 = ROcp15_36*C22-ROcp15_921*S22;
    ROcp15_722 = ROcp15_16*S22+ROcp15_721*C22;
    ROcp15_822 = ROcp15_26*S22+ROcp15_821*C22;
    ROcp15_922 = ROcp15_36*S22+ROcp15_921*C22;
    RLcp15_121 = ROcp15_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp15_221 = ROcp15_26*s->dpt[1][4]+ROcp15_85*s->dpt[3][4];
    RLcp15_321 = ROcp15_36*s->dpt[1][4]+ROcp15_95*s->dpt[3][4];
    OMcp15_122 = OMcp15_16+ROcp15_16*qd[21]+ROcp15_421*qd[22];
    OMcp15_222 = OMcp15_26+ROcp15_26*qd[21]+ROcp15_521*qd[22];
    OMcp15_322 = OMcp15_36+ROcp15_36*qd[21]+ROcp15_621*qd[22];
    RLcp15_123 = ROcp15_722*s->dpt[3][40];
    RLcp15_223 = ROcp15_822*s->dpt[3][40];
    RLcp15_323 = ROcp15_922*s->dpt[3][40];
    OMcp15_123 = OMcp15_122+ROcp15_722*qd[23];
    OMcp15_223 = OMcp15_222+ROcp15_822*qd[23];
    OMcp15_323 = OMcp15_322+ROcp15_922*qd[23];
    RLcp15_147 = ROcp15_722*s->dpt[3][43]+s->dpt[1][43]*(ROcp15_122*C23+ROcp15_421*S23)-s->dpt[2][43]*(ROcp15_122*S23-
 ROcp15_421*C23);
    RLcp15_247 = ROcp15_822*s->dpt[3][43]+s->dpt[1][43]*(ROcp15_222*C23+ROcp15_521*S23)-s->dpt[2][43]*(ROcp15_222*S23-
 ROcp15_521*C23);
    RLcp15_347 = ROcp15_922*s->dpt[3][43]+s->dpt[1][43]*(ROcp15_322*C23+ROcp15_621*S23)-s->dpt[2][43]*(ROcp15_322*S23-
 ROcp15_621*C23);
    POcp15_147 = RLcp15_121+RLcp15_123+RLcp15_147+q[1];
    POcp15_247 = RLcp15_221+RLcp15_223+RLcp15_247+q[2];
    POcp15_347 = RLcp15_321+RLcp15_323+RLcp15_347+q[3];
    VIcp15_147 = qd[1]+OMcp15_222*RLcp15_323+OMcp15_223*RLcp15_347+OMcp15_26*RLcp15_321-OMcp15_322*RLcp15_223-OMcp15_323*
 RLcp15_247-OMcp15_36*RLcp15_221;
    VIcp15_247 = qd[2]-OMcp15_122*RLcp15_323-OMcp15_123*RLcp15_347-OMcp15_16*RLcp15_321+OMcp15_322*RLcp15_123+OMcp15_323*
 RLcp15_147+OMcp15_36*RLcp15_121;
    VIcp15_347 = qd[3]+OMcp15_122*RLcp15_223+OMcp15_123*RLcp15_247+OMcp15_16*RLcp15_221-OMcp15_222*RLcp15_123-OMcp15_223*
 RLcp15_147-OMcp15_26*RLcp15_121;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_147;
    sens->P[2] = POcp15_247;
    sens->P[3] = POcp15_347;
    sens->V[1] = VIcp15_147;
    sens->V[2] = VIcp15_247;
    sens->V[3] = VIcp15_347;
    sens->OM[1] = OMcp15_123;
    sens->OM[2] = OMcp15_223;
    sens->OM[3] = OMcp15_323;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


    ROcp16_25 = S4*S5;
    ROcp16_35 = -C4*S5;
    ROcp16_85 = -S4*C5;
    ROcp16_95 = C4*C5;
    ROcp16_16 = C5*C6;
    ROcp16_26 = ROcp16_25*C6+C4*S6;
    ROcp16_36 = ROcp16_35*C6+S4*S6;
    ROcp16_46 = -C5*S6;
    ROcp16_56 = -(ROcp16_25*S6-C4*C6);
    ROcp16_66 = -(ROcp16_35*S6-S4*C6);
    OMcp16_16 = qd[4]+qd[6]*S5;
    OMcp16_26 = ROcp16_85*qd[6]+qd[5]*C4;
    OMcp16_36 = ROcp16_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_17_0_4 = = 
 
// Sensor Kinematics 


    ROcp16_421 = ROcp16_46*C21+S21*S5;
    ROcp16_521 = ROcp16_56*C21+ROcp16_85*S21;
    ROcp16_621 = ROcp16_66*C21+ROcp16_95*S21;
    ROcp16_721 = -(ROcp16_46*S21-C21*S5);
    ROcp16_821 = -(ROcp16_56*S21-ROcp16_85*C21);
    ROcp16_921 = -(ROcp16_66*S21-ROcp16_95*C21);
    ROcp16_122 = ROcp16_16*C22-ROcp16_721*S22;
    ROcp16_222 = ROcp16_26*C22-ROcp16_821*S22;
    ROcp16_322 = ROcp16_36*C22-ROcp16_921*S22;
    ROcp16_722 = ROcp16_16*S22+ROcp16_721*C22;
    ROcp16_822 = ROcp16_26*S22+ROcp16_821*C22;
    ROcp16_922 = ROcp16_36*S22+ROcp16_921*C22;
    ROcp16_123 = ROcp16_122*C23+ROcp16_421*S23;
    ROcp16_223 = ROcp16_222*C23+ROcp16_521*S23;
    ROcp16_323 = ROcp16_322*C23+ROcp16_621*S23;
    ROcp16_423 = -(ROcp16_122*S23-ROcp16_421*C23);
    ROcp16_523 = -(ROcp16_222*S23-ROcp16_521*C23);
    ROcp16_623 = -(ROcp16_322*S23-ROcp16_621*C23);
    RLcp16_121 = ROcp16_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp16_221 = ROcp16_26*s->dpt[1][4]+ROcp16_85*s->dpt[3][4];
    RLcp16_321 = ROcp16_36*s->dpt[1][4]+ROcp16_95*s->dpt[3][4];
    OMcp16_122 = OMcp16_16+ROcp16_16*qd[21]+ROcp16_421*qd[22];
    OMcp16_222 = OMcp16_26+ROcp16_26*qd[21]+ROcp16_521*qd[22];
    OMcp16_322 = OMcp16_36+ROcp16_36*qd[21]+ROcp16_621*qd[22];
    RLcp16_123 = ROcp16_722*s->dpt[3][40];
    RLcp16_223 = ROcp16_822*s->dpt[3][40];
    RLcp16_323 = ROcp16_922*s->dpt[3][40];
    OMcp16_123 = OMcp16_122+ROcp16_722*qd[23];
    OMcp16_223 = OMcp16_222+ROcp16_822*qd[23];
    OMcp16_323 = OMcp16_322+ROcp16_922*qd[23];

// = = Block_1_0_0_17_0_5 = = 
 
// Sensor Kinematics 


    RLcp16_124 = ROcp16_123*s->dpt[1][44]+ROcp16_423*s->dpt[2][44]+ROcp16_722*s->dpt[3][44];
    RLcp16_224 = ROcp16_223*s->dpt[1][44]+ROcp16_523*s->dpt[2][44]+ROcp16_822*s->dpt[3][44];
    RLcp16_324 = ROcp16_323*s->dpt[1][44]+ROcp16_623*s->dpt[2][44]+ROcp16_922*s->dpt[3][44];
    OMcp16_124 = OMcp16_123+ROcp16_423*qd[24];
    OMcp16_224 = OMcp16_223+ROcp16_523*qd[24];
    OMcp16_324 = OMcp16_323+ROcp16_623*qd[24];
    RLcp16_148 = ROcp16_423*s->dpt[2][46]+s->dpt[1][46]*(ROcp16_123*C24-ROcp16_722*S24)+s->dpt[3][46]*(ROcp16_123*S24+
 ROcp16_722*C24);
    RLcp16_248 = ROcp16_523*s->dpt[2][46]+s->dpt[1][46]*(ROcp16_223*C24-ROcp16_822*S24)+s->dpt[3][46]*(ROcp16_223*S24+
 ROcp16_822*C24);
    RLcp16_348 = ROcp16_623*s->dpt[2][46]+s->dpt[1][46]*(ROcp16_323*C24-ROcp16_922*S24)+s->dpt[3][46]*(ROcp16_323*S24+
 ROcp16_922*C24);
    POcp16_148 = RLcp16_121+RLcp16_123+RLcp16_124+RLcp16_148+q[1];
    POcp16_248 = RLcp16_221+RLcp16_223+RLcp16_224+RLcp16_248+q[2];
    POcp16_348 = RLcp16_321+RLcp16_323+RLcp16_324+RLcp16_348+q[3];
    VIcp16_148 = qd[1]+OMcp16_222*RLcp16_323+OMcp16_223*RLcp16_324+OMcp16_224*RLcp16_348+OMcp16_26*RLcp16_321-OMcp16_322*
 RLcp16_223-OMcp16_323*RLcp16_224-OMcp16_324*RLcp16_248-OMcp16_36*RLcp16_221;
    VIcp16_248 = qd[2]-OMcp16_122*RLcp16_323-OMcp16_123*RLcp16_324-OMcp16_124*RLcp16_348-OMcp16_16*RLcp16_321+OMcp16_322*
 RLcp16_123+OMcp16_323*RLcp16_124+OMcp16_324*RLcp16_148+OMcp16_36*RLcp16_121;
    VIcp16_348 = qd[3]+OMcp16_122*RLcp16_223+OMcp16_123*RLcp16_224+OMcp16_124*RLcp16_248+OMcp16_16*RLcp16_221-OMcp16_222*
 RLcp16_123-OMcp16_223*RLcp16_124-OMcp16_224*RLcp16_148-OMcp16_26*RLcp16_121;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_148;
    sens->P[2] = POcp16_248;
    sens->P[3] = POcp16_348;
    sens->V[1] = VIcp16_148;
    sens->V[2] = VIcp16_248;
    sens->V[3] = VIcp16_348;
    sens->OM[1] = OMcp16_124;
    sens->OM[2] = OMcp16_224;
    sens->OM[3] = OMcp16_324;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


    ROcp17_25 = S4*S5;
    ROcp17_35 = -C4*S5;
    ROcp17_85 = -S4*C5;
    ROcp17_95 = C4*C5;
    ROcp17_16 = C5*C6;
    ROcp17_26 = ROcp17_25*C6+C4*S6;
    ROcp17_36 = ROcp17_35*C6+S4*S6;
    ROcp17_46 = -C5*S6;
    ROcp17_56 = -(ROcp17_25*S6-C4*C6);
    ROcp17_66 = -(ROcp17_35*S6-S4*C6);
    OMcp17_16 = qd[4]+qd[6]*S5;
    OMcp17_26 = ROcp17_85*qd[6]+qd[5]*C4;
    OMcp17_36 = ROcp17_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_18_0_4 = = 
 
// Sensor Kinematics 


    ROcp17_421 = ROcp17_46*C21+S21*S5;
    ROcp17_521 = ROcp17_56*C21+ROcp17_85*S21;
    ROcp17_621 = ROcp17_66*C21+ROcp17_95*S21;
    ROcp17_721 = -(ROcp17_46*S21-C21*S5);
    ROcp17_821 = -(ROcp17_56*S21-ROcp17_85*C21);
    ROcp17_921 = -(ROcp17_66*S21-ROcp17_95*C21);
    ROcp17_122 = ROcp17_16*C22-ROcp17_721*S22;
    ROcp17_222 = ROcp17_26*C22-ROcp17_821*S22;
    ROcp17_322 = ROcp17_36*C22-ROcp17_921*S22;
    ROcp17_722 = ROcp17_16*S22+ROcp17_721*C22;
    ROcp17_822 = ROcp17_26*S22+ROcp17_821*C22;
    ROcp17_922 = ROcp17_36*S22+ROcp17_921*C22;
    ROcp17_123 = ROcp17_122*C23+ROcp17_421*S23;
    ROcp17_223 = ROcp17_222*C23+ROcp17_521*S23;
    ROcp17_323 = ROcp17_322*C23+ROcp17_621*S23;
    ROcp17_423 = -(ROcp17_122*S23-ROcp17_421*C23);
    ROcp17_523 = -(ROcp17_222*S23-ROcp17_521*C23);
    ROcp17_623 = -(ROcp17_322*S23-ROcp17_621*C23);
    RLcp17_121 = ROcp17_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp17_221 = ROcp17_26*s->dpt[1][4]+ROcp17_85*s->dpt[3][4];
    RLcp17_321 = ROcp17_36*s->dpt[1][4]+ROcp17_95*s->dpt[3][4];
    OMcp17_122 = OMcp17_16+ROcp17_16*qd[21]+ROcp17_421*qd[22];
    OMcp17_222 = OMcp17_26+ROcp17_26*qd[21]+ROcp17_521*qd[22];
    OMcp17_322 = OMcp17_36+ROcp17_36*qd[21]+ROcp17_621*qd[22];
    RLcp17_123 = ROcp17_722*s->dpt[3][40];
    RLcp17_223 = ROcp17_822*s->dpt[3][40];
    RLcp17_323 = ROcp17_922*s->dpt[3][40];
    OMcp17_123 = OMcp17_122+ROcp17_722*qd[23];
    OMcp17_223 = OMcp17_222+ROcp17_822*qd[23];
    OMcp17_323 = OMcp17_322+ROcp17_922*qd[23];

// = = Block_1_0_0_18_0_5 = = 
 
// Sensor Kinematics 


    ROcp17_124 = ROcp17_123*C24-ROcp17_722*S24;
    ROcp17_224 = ROcp17_223*C24-ROcp17_822*S24;
    ROcp17_324 = ROcp17_323*C24-ROcp17_922*S24;
    ROcp17_724 = ROcp17_123*S24+ROcp17_722*C24;
    ROcp17_824 = ROcp17_223*S24+ROcp17_822*C24;
    ROcp17_924 = ROcp17_323*S24+ROcp17_922*C24;
    RLcp17_124 = ROcp17_123*s->dpt[1][44]+ROcp17_423*s->dpt[2][44]+ROcp17_722*s->dpt[3][44];
    RLcp17_224 = ROcp17_223*s->dpt[1][44]+ROcp17_523*s->dpt[2][44]+ROcp17_822*s->dpt[3][44];
    RLcp17_324 = ROcp17_323*s->dpt[1][44]+ROcp17_623*s->dpt[2][44]+ROcp17_922*s->dpt[3][44];
    OMcp17_124 = OMcp17_123+ROcp17_423*qd[24];
    OMcp17_224 = OMcp17_223+ROcp17_523*qd[24];
    OMcp17_324 = OMcp17_323+ROcp17_623*qd[24];
    RLcp17_125 = ROcp17_423*s->dpt[2][47];
    RLcp17_225 = ROcp17_523*s->dpt[2][47];
    RLcp17_325 = ROcp17_623*s->dpt[2][47];
    OMcp17_125 = OMcp17_124+ROcp17_124*qd[25];
    OMcp17_225 = OMcp17_224+ROcp17_224*qd[25];
    OMcp17_325 = OMcp17_324+ROcp17_324*qd[25];
    RLcp17_149 = ROcp17_124*s->dpt[1][48]+s->dpt[2][48]*(ROcp17_423*C25+ROcp17_724*S25)-s->dpt[3][48]*(ROcp17_423*S25-
 ROcp17_724*C25);
    RLcp17_249 = ROcp17_224*s->dpt[1][48]+s->dpt[2][48]*(ROcp17_523*C25+ROcp17_824*S25)-s->dpt[3][48]*(ROcp17_523*S25-
 ROcp17_824*C25);
    RLcp17_349 = ROcp17_324*s->dpt[1][48]+s->dpt[2][48]*(ROcp17_623*C25+ROcp17_924*S25)-s->dpt[3][48]*(ROcp17_623*S25-
 ROcp17_924*C25);
    POcp17_149 = RLcp17_121+RLcp17_123+RLcp17_124+RLcp17_125+RLcp17_149+q[1];
    POcp17_249 = RLcp17_221+RLcp17_223+RLcp17_224+RLcp17_225+RLcp17_249+q[2];
    POcp17_349 = RLcp17_321+RLcp17_323+RLcp17_324+RLcp17_325+RLcp17_349+q[3];
    VIcp17_149 = qd[1]+OMcp17_222*RLcp17_323+OMcp17_223*RLcp17_324+OMcp17_224*RLcp17_325+OMcp17_225*RLcp17_349+OMcp17_26*
 RLcp17_321-OMcp17_322*RLcp17_223-OMcp17_323*RLcp17_224-OMcp17_324*RLcp17_225-OMcp17_325*RLcp17_249-OMcp17_36*RLcp17_221;
    VIcp17_249 = qd[2]-OMcp17_122*RLcp17_323-OMcp17_123*RLcp17_324-OMcp17_124*RLcp17_325-OMcp17_125*RLcp17_349-OMcp17_16*
 RLcp17_321+OMcp17_322*RLcp17_123+OMcp17_323*RLcp17_124+OMcp17_324*RLcp17_125+OMcp17_325*RLcp17_149+OMcp17_36*RLcp17_121;
    VIcp17_349 = qd[3]+OMcp17_122*RLcp17_223+OMcp17_123*RLcp17_224+OMcp17_124*RLcp17_225+OMcp17_125*RLcp17_249+OMcp17_16*
 RLcp17_221-OMcp17_222*RLcp17_123-OMcp17_223*RLcp17_124-OMcp17_224*RLcp17_125-OMcp17_225*RLcp17_149-OMcp17_26*RLcp17_121;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_149;
    sens->P[2] = POcp17_249;
    sens->P[3] = POcp17_349;
    sens->V[1] = VIcp17_149;
    sens->V[2] = VIcp17_249;
    sens->V[3] = VIcp17_349;
    sens->OM[1] = OMcp17_125;
    sens->OM[2] = OMcp17_225;
    sens->OM[3] = OMcp17_325;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


    ROcp18_25 = S4*S5;
    ROcp18_35 = -C4*S5;
    ROcp18_85 = -S4*C5;
    ROcp18_95 = C4*C5;
    ROcp18_16 = C5*C6;
    ROcp18_26 = ROcp18_25*C6+C4*S6;
    ROcp18_36 = ROcp18_35*C6+S4*S6;
    ROcp18_46 = -C5*S6;
    ROcp18_56 = -(ROcp18_25*S6-C4*C6);
    ROcp18_66 = -(ROcp18_35*S6-S4*C6);
    OMcp18_16 = qd[4]+qd[6]*S5;
    OMcp18_26 = ROcp18_85*qd[6]+qd[5]*C4;
    OMcp18_36 = ROcp18_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_19_0_4 = = 
 
// Sensor Kinematics 


    ROcp18_421 = ROcp18_46*C21+S21*S5;
    ROcp18_521 = ROcp18_56*C21+ROcp18_85*S21;
    ROcp18_621 = ROcp18_66*C21+ROcp18_95*S21;
    ROcp18_721 = -(ROcp18_46*S21-C21*S5);
    ROcp18_821 = -(ROcp18_56*S21-ROcp18_85*C21);
    ROcp18_921 = -(ROcp18_66*S21-ROcp18_95*C21);
    ROcp18_122 = ROcp18_16*C22-ROcp18_721*S22;
    ROcp18_222 = ROcp18_26*C22-ROcp18_821*S22;
    ROcp18_322 = ROcp18_36*C22-ROcp18_921*S22;
    ROcp18_722 = ROcp18_16*S22+ROcp18_721*C22;
    ROcp18_822 = ROcp18_26*S22+ROcp18_821*C22;
    ROcp18_922 = ROcp18_36*S22+ROcp18_921*C22;
    ROcp18_123 = ROcp18_122*C23+ROcp18_421*S23;
    ROcp18_223 = ROcp18_222*C23+ROcp18_521*S23;
    ROcp18_323 = ROcp18_322*C23+ROcp18_621*S23;
    ROcp18_423 = -(ROcp18_122*S23-ROcp18_421*C23);
    ROcp18_523 = -(ROcp18_222*S23-ROcp18_521*C23);
    ROcp18_623 = -(ROcp18_322*S23-ROcp18_621*C23);
    RLcp18_121 = ROcp18_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp18_221 = ROcp18_26*s->dpt[1][4]+ROcp18_85*s->dpt[3][4];
    RLcp18_321 = ROcp18_36*s->dpt[1][4]+ROcp18_95*s->dpt[3][4];
    OMcp18_122 = OMcp18_16+ROcp18_16*qd[21]+ROcp18_421*qd[22];
    OMcp18_222 = OMcp18_26+ROcp18_26*qd[21]+ROcp18_521*qd[22];
    OMcp18_322 = OMcp18_36+ROcp18_36*qd[21]+ROcp18_621*qd[22];
    RLcp18_123 = ROcp18_722*s->dpt[3][40];
    RLcp18_223 = ROcp18_822*s->dpt[3][40];
    RLcp18_323 = ROcp18_922*s->dpt[3][40];
    OMcp18_123 = OMcp18_122+ROcp18_722*qd[23];
    OMcp18_223 = OMcp18_222+ROcp18_822*qd[23];
    OMcp18_323 = OMcp18_322+ROcp18_922*qd[23];

// = = Block_1_0_0_19_0_5 = = 
 
// Sensor Kinematics 


    ROcp18_124 = ROcp18_123*C24-ROcp18_722*S24;
    ROcp18_224 = ROcp18_223*C24-ROcp18_822*S24;
    ROcp18_324 = ROcp18_323*C24-ROcp18_922*S24;
    ROcp18_724 = ROcp18_123*S24+ROcp18_722*C24;
    ROcp18_824 = ROcp18_223*S24+ROcp18_822*C24;
    ROcp18_924 = ROcp18_323*S24+ROcp18_922*C24;
    ROcp18_425 = ROcp18_423*C25+ROcp18_724*S25;
    ROcp18_525 = ROcp18_523*C25+ROcp18_824*S25;
    ROcp18_625 = ROcp18_623*C25+ROcp18_924*S25;
    ROcp18_725 = -(ROcp18_423*S25-ROcp18_724*C25);
    ROcp18_825 = -(ROcp18_523*S25-ROcp18_824*C25);
    ROcp18_925 = -(ROcp18_623*S25-ROcp18_924*C25);
    RLcp18_124 = ROcp18_123*s->dpt[1][44]+ROcp18_423*s->dpt[2][44]+ROcp18_722*s->dpt[3][44];
    RLcp18_224 = ROcp18_223*s->dpt[1][44]+ROcp18_523*s->dpt[2][44]+ROcp18_822*s->dpt[3][44];
    RLcp18_324 = ROcp18_323*s->dpt[1][44]+ROcp18_623*s->dpt[2][44]+ROcp18_922*s->dpt[3][44];
    OMcp18_124 = OMcp18_123+ROcp18_423*qd[24];
    OMcp18_224 = OMcp18_223+ROcp18_523*qd[24];
    OMcp18_324 = OMcp18_323+ROcp18_623*qd[24];
    RLcp18_125 = ROcp18_423*s->dpt[2][47];
    RLcp18_225 = ROcp18_523*s->dpt[2][47];
    RLcp18_325 = ROcp18_623*s->dpt[2][47];
    OMcp18_125 = OMcp18_124+ROcp18_124*qd[25];
    OMcp18_225 = OMcp18_224+ROcp18_224*qd[25];
    OMcp18_325 = OMcp18_324+ROcp18_324*qd[25];
    RLcp18_126 = ROcp18_725*s->dpt[3][49];
    RLcp18_226 = ROcp18_825*s->dpt[3][49];
    RLcp18_326 = ROcp18_925*s->dpt[3][49];
    OMcp18_126 = OMcp18_125+ROcp18_725*qd[26];
    OMcp18_226 = OMcp18_225+ROcp18_825*qd[26];
    OMcp18_326 = OMcp18_325+ROcp18_925*qd[26];
    RLcp18_150 = ROcp18_725*s->dpt[3][51]+s->dpt[1][51]*(ROcp18_124*C26+ROcp18_425*S26)-s->dpt[2][51]*(ROcp18_124*S26-
 ROcp18_425*C26);
    RLcp18_250 = ROcp18_825*s->dpt[3][51]+s->dpt[1][51]*(ROcp18_224*C26+ROcp18_525*S26)-s->dpt[2][51]*(ROcp18_224*S26-
 ROcp18_525*C26);
    RLcp18_350 = ROcp18_925*s->dpt[3][51]+s->dpt[1][51]*(ROcp18_324*C26+ROcp18_625*S26)-s->dpt[2][51]*(ROcp18_324*S26-
 ROcp18_625*C26);
    POcp18_150 = RLcp18_121+RLcp18_123+RLcp18_124+RLcp18_125+RLcp18_126+RLcp18_150+q[1];
    POcp18_250 = RLcp18_221+RLcp18_223+RLcp18_224+RLcp18_225+RLcp18_226+RLcp18_250+q[2];
    POcp18_350 = RLcp18_321+RLcp18_323+RLcp18_324+RLcp18_325+RLcp18_326+RLcp18_350+q[3];
    VIcp18_150 = qd[1]+OMcp18_222*RLcp18_323+OMcp18_223*RLcp18_324+OMcp18_224*RLcp18_325+OMcp18_225*RLcp18_326+OMcp18_226*
 RLcp18_350+OMcp18_26*RLcp18_321-OMcp18_322*RLcp18_223-OMcp18_323*RLcp18_224-OMcp18_324*RLcp18_225-OMcp18_325*RLcp18_226-
 OMcp18_326*RLcp18_250-OMcp18_36*RLcp18_221;
    VIcp18_250 = qd[2]-OMcp18_122*RLcp18_323-OMcp18_123*RLcp18_324-OMcp18_124*RLcp18_325-OMcp18_125*RLcp18_326-OMcp18_126*
 RLcp18_350-OMcp18_16*RLcp18_321+OMcp18_322*RLcp18_123+OMcp18_323*RLcp18_124+OMcp18_324*RLcp18_125+OMcp18_325*RLcp18_126+
 OMcp18_326*RLcp18_150+OMcp18_36*RLcp18_121;
    VIcp18_350 = qd[3]+OMcp18_122*RLcp18_223+OMcp18_123*RLcp18_224+OMcp18_124*RLcp18_225+OMcp18_125*RLcp18_226+OMcp18_126*
 RLcp18_250+OMcp18_16*RLcp18_221-OMcp18_222*RLcp18_123-OMcp18_223*RLcp18_124-OMcp18_224*RLcp18_125-OMcp18_225*RLcp18_126-
 OMcp18_226*RLcp18_150-OMcp18_26*RLcp18_121;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_150;
    sens->P[2] = POcp18_250;
    sens->P[3] = POcp18_350;
    sens->V[1] = VIcp18_150;
    sens->V[2] = VIcp18_250;
    sens->V[3] = VIcp18_350;
    sens->OM[1] = OMcp18_126;
    sens->OM[2] = OMcp18_226;
    sens->OM[3] = OMcp18_326;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


    ROcp19_25 = S4*S5;
    ROcp19_35 = -C4*S5;
    ROcp19_85 = -S4*C5;
    ROcp19_95 = C4*C5;
    ROcp19_16 = C5*C6;
    ROcp19_26 = ROcp19_25*C6+C4*S6;
    ROcp19_36 = ROcp19_35*C6+S4*S6;
    ROcp19_46 = -C5*S6;
    ROcp19_56 = -(ROcp19_25*S6-C4*C6);
    ROcp19_66 = -(ROcp19_35*S6-S4*C6);
    OMcp19_16 = qd[4]+qd[6]*S5;
    OMcp19_26 = ROcp19_85*qd[6]+qd[5]*C4;
    OMcp19_36 = ROcp19_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_20_0_4 = = 
 
// Sensor Kinematics 


    ROcp19_421 = ROcp19_46*C21+S21*S5;
    ROcp19_521 = ROcp19_56*C21+ROcp19_85*S21;
    ROcp19_621 = ROcp19_66*C21+ROcp19_95*S21;
    ROcp19_721 = -(ROcp19_46*S21-C21*S5);
    ROcp19_821 = -(ROcp19_56*S21-ROcp19_85*C21);
    ROcp19_921 = -(ROcp19_66*S21-ROcp19_95*C21);
    ROcp19_122 = ROcp19_16*C22-ROcp19_721*S22;
    ROcp19_222 = ROcp19_26*C22-ROcp19_821*S22;
    ROcp19_322 = ROcp19_36*C22-ROcp19_921*S22;
    ROcp19_722 = ROcp19_16*S22+ROcp19_721*C22;
    ROcp19_822 = ROcp19_26*S22+ROcp19_821*C22;
    ROcp19_922 = ROcp19_36*S22+ROcp19_921*C22;
    ROcp19_123 = ROcp19_122*C23+ROcp19_421*S23;
    ROcp19_223 = ROcp19_222*C23+ROcp19_521*S23;
    ROcp19_323 = ROcp19_322*C23+ROcp19_621*S23;
    ROcp19_423 = -(ROcp19_122*S23-ROcp19_421*C23);
    ROcp19_523 = -(ROcp19_222*S23-ROcp19_521*C23);
    ROcp19_623 = -(ROcp19_322*S23-ROcp19_621*C23);
    RLcp19_121 = ROcp19_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp19_221 = ROcp19_26*s->dpt[1][4]+ROcp19_85*s->dpt[3][4];
    RLcp19_321 = ROcp19_36*s->dpt[1][4]+ROcp19_95*s->dpt[3][4];
    OMcp19_122 = OMcp19_16+ROcp19_16*qd[21]+ROcp19_421*qd[22];
    OMcp19_222 = OMcp19_26+ROcp19_26*qd[21]+ROcp19_521*qd[22];
    OMcp19_322 = OMcp19_36+ROcp19_36*qd[21]+ROcp19_621*qd[22];
    RLcp19_123 = ROcp19_722*s->dpt[3][40];
    RLcp19_223 = ROcp19_822*s->dpt[3][40];
    RLcp19_323 = ROcp19_922*s->dpt[3][40];
    OMcp19_123 = OMcp19_122+ROcp19_722*qd[23];
    OMcp19_223 = OMcp19_222+ROcp19_822*qd[23];
    OMcp19_323 = OMcp19_322+ROcp19_922*qd[23];

// = = Block_1_0_0_20_0_5 = = 
 
// Sensor Kinematics 


    ROcp19_124 = ROcp19_123*C24-ROcp19_722*S24;
    ROcp19_224 = ROcp19_223*C24-ROcp19_822*S24;
    ROcp19_324 = ROcp19_323*C24-ROcp19_922*S24;
    ROcp19_724 = ROcp19_123*S24+ROcp19_722*C24;
    ROcp19_824 = ROcp19_223*S24+ROcp19_822*C24;
    ROcp19_924 = ROcp19_323*S24+ROcp19_922*C24;
    ROcp19_425 = ROcp19_423*C25+ROcp19_724*S25;
    ROcp19_525 = ROcp19_523*C25+ROcp19_824*S25;
    ROcp19_625 = ROcp19_623*C25+ROcp19_924*S25;
    ROcp19_725 = -(ROcp19_423*S25-ROcp19_724*C25);
    ROcp19_825 = -(ROcp19_523*S25-ROcp19_824*C25);
    ROcp19_925 = -(ROcp19_623*S25-ROcp19_924*C25);
    ROcp19_126 = ROcp19_124*C26+ROcp19_425*S26;
    ROcp19_226 = ROcp19_224*C26+ROcp19_525*S26;
    ROcp19_326 = ROcp19_324*C26+ROcp19_625*S26;
    ROcp19_426 = -(ROcp19_124*S26-ROcp19_425*C26);
    ROcp19_526 = -(ROcp19_224*S26-ROcp19_525*C26);
    ROcp19_626 = -(ROcp19_324*S26-ROcp19_625*C26);
    RLcp19_124 = ROcp19_123*s->dpt[1][44]+ROcp19_423*s->dpt[2][44]+ROcp19_722*s->dpt[3][44];
    RLcp19_224 = ROcp19_223*s->dpt[1][44]+ROcp19_523*s->dpt[2][44]+ROcp19_822*s->dpt[3][44];
    RLcp19_324 = ROcp19_323*s->dpt[1][44]+ROcp19_623*s->dpt[2][44]+ROcp19_922*s->dpt[3][44];
    OMcp19_124 = OMcp19_123+ROcp19_423*qd[24];
    OMcp19_224 = OMcp19_223+ROcp19_523*qd[24];
    OMcp19_324 = OMcp19_323+ROcp19_623*qd[24];
    RLcp19_125 = ROcp19_423*s->dpt[2][47];
    RLcp19_225 = ROcp19_523*s->dpt[2][47];
    RLcp19_325 = ROcp19_623*s->dpt[2][47];
    OMcp19_125 = OMcp19_124+ROcp19_124*qd[25];
    OMcp19_225 = OMcp19_224+ROcp19_224*qd[25];
    OMcp19_325 = OMcp19_324+ROcp19_324*qd[25];
    RLcp19_126 = ROcp19_725*s->dpt[3][49];
    RLcp19_226 = ROcp19_825*s->dpt[3][49];
    RLcp19_326 = ROcp19_925*s->dpt[3][49];
    OMcp19_126 = OMcp19_125+ROcp19_725*qd[26];
    OMcp19_226 = OMcp19_225+ROcp19_825*qd[26];
    OMcp19_326 = OMcp19_325+ROcp19_925*qd[26];
    RLcp19_127 = ROcp19_725*s->dpt[3][52];
    RLcp19_227 = ROcp19_825*s->dpt[3][52];
    RLcp19_327 = ROcp19_925*s->dpt[3][52];
    OMcp19_127 = OMcp19_126+ROcp19_426*qd[27];
    OMcp19_227 = OMcp19_226+ROcp19_526*qd[27];
    OMcp19_327 = OMcp19_326+ROcp19_626*qd[27];
    RLcp19_151 = ROcp19_426*s->dpt[2][54]+s->dpt[1][54]*(ROcp19_126*C27-ROcp19_725*S27)+s->dpt[3][54]*(ROcp19_126*S27+
 ROcp19_725*C27);
    RLcp19_251 = ROcp19_526*s->dpt[2][54]+s->dpt[1][54]*(ROcp19_226*C27-ROcp19_825*S27)+s->dpt[3][54]*(ROcp19_226*S27+
 ROcp19_825*C27);
    RLcp19_351 = ROcp19_626*s->dpt[2][54]+s->dpt[1][54]*(ROcp19_326*C27-ROcp19_925*S27)+s->dpt[3][54]*(ROcp19_326*S27+
 ROcp19_925*C27);
    POcp19_151 = RLcp19_121+RLcp19_123+RLcp19_124+RLcp19_125+RLcp19_126+RLcp19_127+RLcp19_151+q[1];
    POcp19_251 = RLcp19_221+RLcp19_223+RLcp19_224+RLcp19_225+RLcp19_226+RLcp19_227+RLcp19_251+q[2];
    POcp19_351 = RLcp19_321+RLcp19_323+RLcp19_324+RLcp19_325+RLcp19_326+RLcp19_327+RLcp19_351+q[3];
    VIcp19_151 = qd[1]+OMcp19_222*RLcp19_323+OMcp19_223*RLcp19_324+OMcp19_224*RLcp19_325+OMcp19_225*RLcp19_326+OMcp19_226*
 RLcp19_327+OMcp19_227*RLcp19_351+OMcp19_26*RLcp19_321-OMcp19_322*RLcp19_223-OMcp19_323*RLcp19_224-OMcp19_324*RLcp19_225-
 OMcp19_325*RLcp19_226-OMcp19_326*RLcp19_227-OMcp19_327*RLcp19_251-OMcp19_36*RLcp19_221;
    VIcp19_251 = qd[2]-OMcp19_122*RLcp19_323-OMcp19_123*RLcp19_324-OMcp19_124*RLcp19_325-OMcp19_125*RLcp19_326-OMcp19_126*
 RLcp19_327-OMcp19_127*RLcp19_351-OMcp19_16*RLcp19_321+OMcp19_322*RLcp19_123+OMcp19_323*RLcp19_124+OMcp19_324*RLcp19_125+
 OMcp19_325*RLcp19_126+OMcp19_326*RLcp19_127+OMcp19_327*RLcp19_151+OMcp19_36*RLcp19_121;
    VIcp19_351 = qd[3]+OMcp19_122*RLcp19_223+OMcp19_123*RLcp19_224+OMcp19_124*RLcp19_225+OMcp19_125*RLcp19_226+OMcp19_126*
 RLcp19_227+OMcp19_127*RLcp19_251+OMcp19_16*RLcp19_221-OMcp19_222*RLcp19_123-OMcp19_223*RLcp19_124-OMcp19_224*RLcp19_125-
 OMcp19_225*RLcp19_126-OMcp19_226*RLcp19_127-OMcp19_227*RLcp19_151-OMcp19_26*RLcp19_121;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_151;
    sens->P[2] = POcp19_251;
    sens->P[3] = POcp19_351;
    sens->V[1] = VIcp19_151;
    sens->V[2] = VIcp19_251;
    sens->V[3] = VIcp19_351;
    sens->OM[1] = OMcp19_127;
    sens->OM[2] = OMcp19_227;
    sens->OM[3] = OMcp19_327;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


    ROcp20_25 = S4*S5;
    ROcp20_35 = -C4*S5;
    ROcp20_85 = -S4*C5;
    ROcp20_95 = C4*C5;
    ROcp20_16 = C5*C6;
    ROcp20_26 = ROcp20_25*C6+C4*S6;
    ROcp20_36 = ROcp20_35*C6+S4*S6;
    ROcp20_46 = -C5*S6;
    ROcp20_56 = -(ROcp20_25*S6-C4*C6);
    ROcp20_66 = -(ROcp20_35*S6-S4*C6);
    OMcp20_16 = qd[4]+qd[6]*S5;
    OMcp20_26 = ROcp20_85*qd[6]+qd[5]*C4;
    OMcp20_36 = ROcp20_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_21_0_4 = = 
 
// Sensor Kinematics 


    ROcp20_421 = ROcp20_46*C21+S21*S5;
    ROcp20_521 = ROcp20_56*C21+ROcp20_85*S21;
    ROcp20_621 = ROcp20_66*C21+ROcp20_95*S21;
    ROcp20_721 = -(ROcp20_46*S21-C21*S5);
    ROcp20_821 = -(ROcp20_56*S21-ROcp20_85*C21);
    ROcp20_921 = -(ROcp20_66*S21-ROcp20_95*C21);
    ROcp20_122 = ROcp20_16*C22-ROcp20_721*S22;
    ROcp20_222 = ROcp20_26*C22-ROcp20_821*S22;
    ROcp20_322 = ROcp20_36*C22-ROcp20_921*S22;
    ROcp20_722 = ROcp20_16*S22+ROcp20_721*C22;
    ROcp20_822 = ROcp20_26*S22+ROcp20_821*C22;
    ROcp20_922 = ROcp20_36*S22+ROcp20_921*C22;
    ROcp20_123 = ROcp20_122*C23+ROcp20_421*S23;
    ROcp20_223 = ROcp20_222*C23+ROcp20_521*S23;
    ROcp20_323 = ROcp20_322*C23+ROcp20_621*S23;
    ROcp20_423 = -(ROcp20_122*S23-ROcp20_421*C23);
    ROcp20_523 = -(ROcp20_222*S23-ROcp20_521*C23);
    ROcp20_623 = -(ROcp20_322*S23-ROcp20_621*C23);
    RLcp20_121 = ROcp20_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp20_221 = ROcp20_26*s->dpt[1][4]+ROcp20_85*s->dpt[3][4];
    RLcp20_321 = ROcp20_36*s->dpt[1][4]+ROcp20_95*s->dpt[3][4];
    OMcp20_122 = OMcp20_16+ROcp20_16*qd[21]+ROcp20_421*qd[22];
    OMcp20_222 = OMcp20_26+ROcp20_26*qd[21]+ROcp20_521*qd[22];
    OMcp20_322 = OMcp20_36+ROcp20_36*qd[21]+ROcp20_621*qd[22];
    RLcp20_123 = ROcp20_722*s->dpt[3][40];
    RLcp20_223 = ROcp20_822*s->dpt[3][40];
    RLcp20_323 = ROcp20_922*s->dpt[3][40];
    OMcp20_123 = OMcp20_122+ROcp20_722*qd[23];
    OMcp20_223 = OMcp20_222+ROcp20_822*qd[23];
    OMcp20_323 = OMcp20_322+ROcp20_922*qd[23];

// = = Block_1_0_0_21_0_6 = = 
 
// Sensor Kinematics 


    RLcp20_128 = ROcp20_123*s->dpt[1][45]+ROcp20_423*s->dpt[2][45]+ROcp20_722*s->dpt[3][45];
    RLcp20_228 = ROcp20_223*s->dpt[1][45]+ROcp20_523*s->dpt[2][45]+ROcp20_822*s->dpt[3][45];
    RLcp20_328 = ROcp20_323*s->dpt[1][45]+ROcp20_623*s->dpt[2][45]+ROcp20_922*s->dpt[3][45];
    OMcp20_128 = OMcp20_123+ROcp20_423*qd[28];
    OMcp20_228 = OMcp20_223+ROcp20_523*qd[28];
    OMcp20_328 = OMcp20_323+ROcp20_623*qd[28];
    RLcp20_152 = ROcp20_423*s->dpt[2][55]+s->dpt[1][55]*(ROcp20_123*C28-ROcp20_722*S28)+s->dpt[3][55]*(ROcp20_123*S28+
 ROcp20_722*C28);
    RLcp20_252 = ROcp20_523*s->dpt[2][55]+s->dpt[1][55]*(ROcp20_223*C28-ROcp20_822*S28)+s->dpt[3][55]*(ROcp20_223*S28+
 ROcp20_822*C28);
    RLcp20_352 = ROcp20_623*s->dpt[2][55]+s->dpt[1][55]*(ROcp20_323*C28-ROcp20_922*S28)+s->dpt[3][55]*(ROcp20_323*S28+
 ROcp20_922*C28);
    POcp20_152 = RLcp20_121+RLcp20_123+RLcp20_128+RLcp20_152+q[1];
    POcp20_252 = RLcp20_221+RLcp20_223+RLcp20_228+RLcp20_252+q[2];
    POcp20_352 = RLcp20_321+RLcp20_323+RLcp20_328+RLcp20_352+q[3];
    VIcp20_152 = qd[1]+OMcp20_222*RLcp20_323+OMcp20_223*RLcp20_328+OMcp20_228*RLcp20_352+OMcp20_26*RLcp20_321-OMcp20_322*
 RLcp20_223-OMcp20_323*RLcp20_228-OMcp20_328*RLcp20_252-OMcp20_36*RLcp20_221;
    VIcp20_252 = qd[2]-OMcp20_122*RLcp20_323-OMcp20_123*RLcp20_328-OMcp20_128*RLcp20_352-OMcp20_16*RLcp20_321+OMcp20_322*
 RLcp20_123+OMcp20_323*RLcp20_128+OMcp20_328*RLcp20_152+OMcp20_36*RLcp20_121;
    VIcp20_352 = qd[3]+OMcp20_122*RLcp20_223+OMcp20_123*RLcp20_228+OMcp20_128*RLcp20_252+OMcp20_16*RLcp20_221-OMcp20_222*
 RLcp20_123-OMcp20_223*RLcp20_128-OMcp20_228*RLcp20_152-OMcp20_26*RLcp20_121;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_152;
    sens->P[2] = POcp20_252;
    sens->P[3] = POcp20_352;
    sens->V[1] = VIcp20_152;
    sens->V[2] = VIcp20_252;
    sens->V[3] = VIcp20_352;
    sens->OM[1] = OMcp20_128;
    sens->OM[2] = OMcp20_228;
    sens->OM[3] = OMcp20_328;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


    ROcp21_25 = S4*S5;
    ROcp21_35 = -C4*S5;
    ROcp21_85 = -S4*C5;
    ROcp21_95 = C4*C5;
    ROcp21_16 = C5*C6;
    ROcp21_26 = ROcp21_25*C6+C4*S6;
    ROcp21_36 = ROcp21_35*C6+S4*S6;
    ROcp21_46 = -C5*S6;
    ROcp21_56 = -(ROcp21_25*S6-C4*C6);
    ROcp21_66 = -(ROcp21_35*S6-S4*C6);
    OMcp21_16 = qd[4]+qd[6]*S5;
    OMcp21_26 = ROcp21_85*qd[6]+qd[5]*C4;
    OMcp21_36 = ROcp21_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_22_0_4 = = 
 
// Sensor Kinematics 


    ROcp21_421 = ROcp21_46*C21+S21*S5;
    ROcp21_521 = ROcp21_56*C21+ROcp21_85*S21;
    ROcp21_621 = ROcp21_66*C21+ROcp21_95*S21;
    ROcp21_721 = -(ROcp21_46*S21-C21*S5);
    ROcp21_821 = -(ROcp21_56*S21-ROcp21_85*C21);
    ROcp21_921 = -(ROcp21_66*S21-ROcp21_95*C21);
    ROcp21_122 = ROcp21_16*C22-ROcp21_721*S22;
    ROcp21_222 = ROcp21_26*C22-ROcp21_821*S22;
    ROcp21_322 = ROcp21_36*C22-ROcp21_921*S22;
    ROcp21_722 = ROcp21_16*S22+ROcp21_721*C22;
    ROcp21_822 = ROcp21_26*S22+ROcp21_821*C22;
    ROcp21_922 = ROcp21_36*S22+ROcp21_921*C22;
    ROcp21_123 = ROcp21_122*C23+ROcp21_421*S23;
    ROcp21_223 = ROcp21_222*C23+ROcp21_521*S23;
    ROcp21_323 = ROcp21_322*C23+ROcp21_621*S23;
    ROcp21_423 = -(ROcp21_122*S23-ROcp21_421*C23);
    ROcp21_523 = -(ROcp21_222*S23-ROcp21_521*C23);
    ROcp21_623 = -(ROcp21_322*S23-ROcp21_621*C23);
    RLcp21_121 = ROcp21_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp21_221 = ROcp21_26*s->dpt[1][4]+ROcp21_85*s->dpt[3][4];
    RLcp21_321 = ROcp21_36*s->dpt[1][4]+ROcp21_95*s->dpt[3][4];
    OMcp21_122 = OMcp21_16+ROcp21_16*qd[21]+ROcp21_421*qd[22];
    OMcp21_222 = OMcp21_26+ROcp21_26*qd[21]+ROcp21_521*qd[22];
    OMcp21_322 = OMcp21_36+ROcp21_36*qd[21]+ROcp21_621*qd[22];
    RLcp21_123 = ROcp21_722*s->dpt[3][40];
    RLcp21_223 = ROcp21_822*s->dpt[3][40];
    RLcp21_323 = ROcp21_922*s->dpt[3][40];
    OMcp21_123 = OMcp21_122+ROcp21_722*qd[23];
    OMcp21_223 = OMcp21_222+ROcp21_822*qd[23];
    OMcp21_323 = OMcp21_322+ROcp21_922*qd[23];

// = = Block_1_0_0_22_0_6 = = 
 
// Sensor Kinematics 


    ROcp21_128 = ROcp21_123*C28-ROcp21_722*S28;
    ROcp21_228 = ROcp21_223*C28-ROcp21_822*S28;
    ROcp21_328 = ROcp21_323*C28-ROcp21_922*S28;
    ROcp21_728 = ROcp21_123*S28+ROcp21_722*C28;
    ROcp21_828 = ROcp21_223*S28+ROcp21_822*C28;
    ROcp21_928 = ROcp21_323*S28+ROcp21_922*C28;
    RLcp21_128 = ROcp21_123*s->dpt[1][45]+ROcp21_423*s->dpt[2][45]+ROcp21_722*s->dpt[3][45];
    RLcp21_228 = ROcp21_223*s->dpt[1][45]+ROcp21_523*s->dpt[2][45]+ROcp21_822*s->dpt[3][45];
    RLcp21_328 = ROcp21_323*s->dpt[1][45]+ROcp21_623*s->dpt[2][45]+ROcp21_922*s->dpt[3][45];
    OMcp21_128 = OMcp21_123+ROcp21_423*qd[28];
    OMcp21_228 = OMcp21_223+ROcp21_523*qd[28];
    OMcp21_328 = OMcp21_323+ROcp21_623*qd[28];
    RLcp21_129 = ROcp21_423*s->dpt[2][56];
    RLcp21_229 = ROcp21_523*s->dpt[2][56];
    RLcp21_329 = ROcp21_623*s->dpt[2][56];
    OMcp21_129 = OMcp21_128+ROcp21_128*qd[29];
    OMcp21_229 = OMcp21_228+ROcp21_228*qd[29];
    OMcp21_329 = OMcp21_328+ROcp21_328*qd[29];
    RLcp21_153 = ROcp21_128*s->dpt[1][57]+s->dpt[2][57]*(ROcp21_423*C29+ROcp21_728*S29)-s->dpt[3][57]*(ROcp21_423*S29-
 ROcp21_728*C29);
    RLcp21_253 = ROcp21_228*s->dpt[1][57]+s->dpt[2][57]*(ROcp21_523*C29+ROcp21_828*S29)-s->dpt[3][57]*(ROcp21_523*S29-
 ROcp21_828*C29);
    RLcp21_353 = ROcp21_328*s->dpt[1][57]+s->dpt[2][57]*(ROcp21_623*C29+ROcp21_928*S29)-s->dpt[3][57]*(ROcp21_623*S29-
 ROcp21_928*C29);
    POcp21_153 = RLcp21_121+RLcp21_123+RLcp21_128+RLcp21_129+RLcp21_153+q[1];
    POcp21_253 = RLcp21_221+RLcp21_223+RLcp21_228+RLcp21_229+RLcp21_253+q[2];
    POcp21_353 = RLcp21_321+RLcp21_323+RLcp21_328+RLcp21_329+RLcp21_353+q[3];
    VIcp21_153 = qd[1]+OMcp21_222*RLcp21_323+OMcp21_223*RLcp21_328+OMcp21_228*RLcp21_329+OMcp21_229*RLcp21_353+OMcp21_26*
 RLcp21_321-OMcp21_322*RLcp21_223-OMcp21_323*RLcp21_228-OMcp21_328*RLcp21_229-OMcp21_329*RLcp21_253-OMcp21_36*RLcp21_221;
    VIcp21_253 = qd[2]-OMcp21_122*RLcp21_323-OMcp21_123*RLcp21_328-OMcp21_128*RLcp21_329-OMcp21_129*RLcp21_353-OMcp21_16*
 RLcp21_321+OMcp21_322*RLcp21_123+OMcp21_323*RLcp21_128+OMcp21_328*RLcp21_129+OMcp21_329*RLcp21_153+OMcp21_36*RLcp21_121;
    VIcp21_353 = qd[3]+OMcp21_122*RLcp21_223+OMcp21_123*RLcp21_228+OMcp21_128*RLcp21_229+OMcp21_129*RLcp21_253+OMcp21_16*
 RLcp21_221-OMcp21_222*RLcp21_123-OMcp21_223*RLcp21_128-OMcp21_228*RLcp21_129-OMcp21_229*RLcp21_153-OMcp21_26*RLcp21_121;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_153;
    sens->P[2] = POcp21_253;
    sens->P[3] = POcp21_353;
    sens->V[1] = VIcp21_153;
    sens->V[2] = VIcp21_253;
    sens->V[3] = VIcp21_353;
    sens->OM[1] = OMcp21_129;
    sens->OM[2] = OMcp21_229;
    sens->OM[3] = OMcp21_329;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


    ROcp22_25 = S4*S5;
    ROcp22_35 = -C4*S5;
    ROcp22_85 = -S4*C5;
    ROcp22_95 = C4*C5;
    ROcp22_16 = C5*C6;
    ROcp22_26 = ROcp22_25*C6+C4*S6;
    ROcp22_36 = ROcp22_35*C6+S4*S6;
    ROcp22_46 = -C5*S6;
    ROcp22_56 = -(ROcp22_25*S6-C4*C6);
    ROcp22_66 = -(ROcp22_35*S6-S4*C6);
    OMcp22_16 = qd[4]+qd[6]*S5;
    OMcp22_26 = ROcp22_85*qd[6]+qd[5]*C4;
    OMcp22_36 = ROcp22_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_23_0_4 = = 
 
// Sensor Kinematics 


    ROcp22_421 = ROcp22_46*C21+S21*S5;
    ROcp22_521 = ROcp22_56*C21+ROcp22_85*S21;
    ROcp22_621 = ROcp22_66*C21+ROcp22_95*S21;
    ROcp22_721 = -(ROcp22_46*S21-C21*S5);
    ROcp22_821 = -(ROcp22_56*S21-ROcp22_85*C21);
    ROcp22_921 = -(ROcp22_66*S21-ROcp22_95*C21);
    ROcp22_122 = ROcp22_16*C22-ROcp22_721*S22;
    ROcp22_222 = ROcp22_26*C22-ROcp22_821*S22;
    ROcp22_322 = ROcp22_36*C22-ROcp22_921*S22;
    ROcp22_722 = ROcp22_16*S22+ROcp22_721*C22;
    ROcp22_822 = ROcp22_26*S22+ROcp22_821*C22;
    ROcp22_922 = ROcp22_36*S22+ROcp22_921*C22;
    ROcp22_123 = ROcp22_122*C23+ROcp22_421*S23;
    ROcp22_223 = ROcp22_222*C23+ROcp22_521*S23;
    ROcp22_323 = ROcp22_322*C23+ROcp22_621*S23;
    ROcp22_423 = -(ROcp22_122*S23-ROcp22_421*C23);
    ROcp22_523 = -(ROcp22_222*S23-ROcp22_521*C23);
    ROcp22_623 = -(ROcp22_322*S23-ROcp22_621*C23);
    RLcp22_121 = ROcp22_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp22_221 = ROcp22_26*s->dpt[1][4]+ROcp22_85*s->dpt[3][4];
    RLcp22_321 = ROcp22_36*s->dpt[1][4]+ROcp22_95*s->dpt[3][4];
    OMcp22_122 = OMcp22_16+ROcp22_16*qd[21]+ROcp22_421*qd[22];
    OMcp22_222 = OMcp22_26+ROcp22_26*qd[21]+ROcp22_521*qd[22];
    OMcp22_322 = OMcp22_36+ROcp22_36*qd[21]+ROcp22_621*qd[22];
    RLcp22_123 = ROcp22_722*s->dpt[3][40];
    RLcp22_223 = ROcp22_822*s->dpt[3][40];
    RLcp22_323 = ROcp22_922*s->dpt[3][40];
    OMcp22_123 = OMcp22_122+ROcp22_722*qd[23];
    OMcp22_223 = OMcp22_222+ROcp22_822*qd[23];
    OMcp22_323 = OMcp22_322+ROcp22_922*qd[23];

// = = Block_1_0_0_23_0_6 = = 
 
// Sensor Kinematics 


    ROcp22_128 = ROcp22_123*C28-ROcp22_722*S28;
    ROcp22_228 = ROcp22_223*C28-ROcp22_822*S28;
    ROcp22_328 = ROcp22_323*C28-ROcp22_922*S28;
    ROcp22_728 = ROcp22_123*S28+ROcp22_722*C28;
    ROcp22_828 = ROcp22_223*S28+ROcp22_822*C28;
    ROcp22_928 = ROcp22_323*S28+ROcp22_922*C28;
    ROcp22_429 = ROcp22_423*C29+ROcp22_728*S29;
    ROcp22_529 = ROcp22_523*C29+ROcp22_828*S29;
    ROcp22_629 = ROcp22_623*C29+ROcp22_928*S29;
    ROcp22_729 = -(ROcp22_423*S29-ROcp22_728*C29);
    ROcp22_829 = -(ROcp22_523*S29-ROcp22_828*C29);
    ROcp22_929 = -(ROcp22_623*S29-ROcp22_928*C29);
    RLcp22_128 = ROcp22_123*s->dpt[1][45]+ROcp22_423*s->dpt[2][45]+ROcp22_722*s->dpt[3][45];
    RLcp22_228 = ROcp22_223*s->dpt[1][45]+ROcp22_523*s->dpt[2][45]+ROcp22_822*s->dpt[3][45];
    RLcp22_328 = ROcp22_323*s->dpt[1][45]+ROcp22_623*s->dpt[2][45]+ROcp22_922*s->dpt[3][45];
    OMcp22_128 = OMcp22_123+ROcp22_423*qd[28];
    OMcp22_228 = OMcp22_223+ROcp22_523*qd[28];
    OMcp22_328 = OMcp22_323+ROcp22_623*qd[28];
    RLcp22_129 = ROcp22_423*s->dpt[2][56];
    RLcp22_229 = ROcp22_523*s->dpt[2][56];
    RLcp22_329 = ROcp22_623*s->dpt[2][56];
    OMcp22_129 = OMcp22_128+ROcp22_128*qd[29];
    OMcp22_229 = OMcp22_228+ROcp22_228*qd[29];
    OMcp22_329 = OMcp22_328+ROcp22_328*qd[29];
    RLcp22_130 = ROcp22_729*s->dpt[3][58];
    RLcp22_230 = ROcp22_829*s->dpt[3][58];
    RLcp22_330 = ROcp22_929*s->dpt[3][58];
    OMcp22_130 = OMcp22_129+ROcp22_729*qd[30];
    OMcp22_230 = OMcp22_229+ROcp22_829*qd[30];
    OMcp22_330 = OMcp22_329+ROcp22_929*qd[30];
    RLcp22_154 = ROcp22_729*s->dpt[3][60]+s->dpt[1][60]*(ROcp22_128*C30+ROcp22_429*S30)-s->dpt[2][60]*(ROcp22_128*S30-
 ROcp22_429*C30);
    RLcp22_254 = ROcp22_829*s->dpt[3][60]+s->dpt[1][60]*(ROcp22_228*C30+ROcp22_529*S30)-s->dpt[2][60]*(ROcp22_228*S30-
 ROcp22_529*C30);
    RLcp22_354 = ROcp22_929*s->dpt[3][60]+s->dpt[1][60]*(ROcp22_328*C30+ROcp22_629*S30)-s->dpt[2][60]*(ROcp22_328*S30-
 ROcp22_629*C30);
    POcp22_154 = RLcp22_121+RLcp22_123+RLcp22_128+RLcp22_129+RLcp22_130+RLcp22_154+q[1];
    POcp22_254 = RLcp22_221+RLcp22_223+RLcp22_228+RLcp22_229+RLcp22_230+RLcp22_254+q[2];
    POcp22_354 = RLcp22_321+RLcp22_323+RLcp22_328+RLcp22_329+RLcp22_330+RLcp22_354+q[3];
    VIcp22_154 = qd[1]+OMcp22_222*RLcp22_323+OMcp22_223*RLcp22_328+OMcp22_228*RLcp22_329+OMcp22_229*RLcp22_330+OMcp22_230*
 RLcp22_354+OMcp22_26*RLcp22_321-OMcp22_322*RLcp22_223-OMcp22_323*RLcp22_228-OMcp22_328*RLcp22_229-OMcp22_329*RLcp22_230-
 OMcp22_330*RLcp22_254-OMcp22_36*RLcp22_221;
    VIcp22_254 = qd[2]-OMcp22_122*RLcp22_323-OMcp22_123*RLcp22_328-OMcp22_128*RLcp22_329-OMcp22_129*RLcp22_330-OMcp22_130*
 RLcp22_354-OMcp22_16*RLcp22_321+OMcp22_322*RLcp22_123+OMcp22_323*RLcp22_128+OMcp22_328*RLcp22_129+OMcp22_329*RLcp22_130+
 OMcp22_330*RLcp22_154+OMcp22_36*RLcp22_121;
    VIcp22_354 = qd[3]+OMcp22_122*RLcp22_223+OMcp22_123*RLcp22_228+OMcp22_128*RLcp22_229+OMcp22_129*RLcp22_230+OMcp22_130*
 RLcp22_254+OMcp22_16*RLcp22_221-OMcp22_222*RLcp22_123-OMcp22_223*RLcp22_128-OMcp22_228*RLcp22_129-OMcp22_229*RLcp22_130-
 OMcp22_230*RLcp22_154-OMcp22_26*RLcp22_121;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_154;
    sens->P[2] = POcp22_254;
    sens->P[3] = POcp22_354;
    sens->V[1] = VIcp22_154;
    sens->V[2] = VIcp22_254;
    sens->V[3] = VIcp22_354;
    sens->OM[1] = OMcp22_130;
    sens->OM[2] = OMcp22_230;
    sens->OM[3] = OMcp22_330;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


    ROcp23_25 = S4*S5;
    ROcp23_35 = -C4*S5;
    ROcp23_85 = -S4*C5;
    ROcp23_95 = C4*C5;
    ROcp23_16 = C5*C6;
    ROcp23_26 = ROcp23_25*C6+C4*S6;
    ROcp23_36 = ROcp23_35*C6+S4*S6;
    ROcp23_46 = -C5*S6;
    ROcp23_56 = -(ROcp23_25*S6-C4*C6);
    ROcp23_66 = -(ROcp23_35*S6-S4*C6);
    OMcp23_16 = qd[4]+qd[6]*S5;
    OMcp23_26 = ROcp23_85*qd[6]+qd[5]*C4;
    OMcp23_36 = ROcp23_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_24_0_4 = = 
 
// Sensor Kinematics 


    ROcp23_421 = ROcp23_46*C21+S21*S5;
    ROcp23_521 = ROcp23_56*C21+ROcp23_85*S21;
    ROcp23_621 = ROcp23_66*C21+ROcp23_95*S21;
    ROcp23_721 = -(ROcp23_46*S21-C21*S5);
    ROcp23_821 = -(ROcp23_56*S21-ROcp23_85*C21);
    ROcp23_921 = -(ROcp23_66*S21-ROcp23_95*C21);
    ROcp23_122 = ROcp23_16*C22-ROcp23_721*S22;
    ROcp23_222 = ROcp23_26*C22-ROcp23_821*S22;
    ROcp23_322 = ROcp23_36*C22-ROcp23_921*S22;
    ROcp23_722 = ROcp23_16*S22+ROcp23_721*C22;
    ROcp23_822 = ROcp23_26*S22+ROcp23_821*C22;
    ROcp23_922 = ROcp23_36*S22+ROcp23_921*C22;
    ROcp23_123 = ROcp23_122*C23+ROcp23_421*S23;
    ROcp23_223 = ROcp23_222*C23+ROcp23_521*S23;
    ROcp23_323 = ROcp23_322*C23+ROcp23_621*S23;
    ROcp23_423 = -(ROcp23_122*S23-ROcp23_421*C23);
    ROcp23_523 = -(ROcp23_222*S23-ROcp23_521*C23);
    ROcp23_623 = -(ROcp23_322*S23-ROcp23_621*C23);
    RLcp23_121 = ROcp23_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp23_221 = ROcp23_26*s->dpt[1][4]+ROcp23_85*s->dpt[3][4];
    RLcp23_321 = ROcp23_36*s->dpt[1][4]+ROcp23_95*s->dpt[3][4];
    OMcp23_122 = OMcp23_16+ROcp23_16*qd[21]+ROcp23_421*qd[22];
    OMcp23_222 = OMcp23_26+ROcp23_26*qd[21]+ROcp23_521*qd[22];
    OMcp23_322 = OMcp23_36+ROcp23_36*qd[21]+ROcp23_621*qd[22];
    RLcp23_123 = ROcp23_722*s->dpt[3][40];
    RLcp23_223 = ROcp23_822*s->dpt[3][40];
    RLcp23_323 = ROcp23_922*s->dpt[3][40];
    OMcp23_123 = OMcp23_122+ROcp23_722*qd[23];
    OMcp23_223 = OMcp23_222+ROcp23_822*qd[23];
    OMcp23_323 = OMcp23_322+ROcp23_922*qd[23];

// = = Block_1_0_0_24_0_6 = = 
 
// Sensor Kinematics 


    ROcp23_128 = ROcp23_123*C28-ROcp23_722*S28;
    ROcp23_228 = ROcp23_223*C28-ROcp23_822*S28;
    ROcp23_328 = ROcp23_323*C28-ROcp23_922*S28;
    ROcp23_728 = ROcp23_123*S28+ROcp23_722*C28;
    ROcp23_828 = ROcp23_223*S28+ROcp23_822*C28;
    ROcp23_928 = ROcp23_323*S28+ROcp23_922*C28;
    ROcp23_429 = ROcp23_423*C29+ROcp23_728*S29;
    ROcp23_529 = ROcp23_523*C29+ROcp23_828*S29;
    ROcp23_629 = ROcp23_623*C29+ROcp23_928*S29;
    ROcp23_729 = -(ROcp23_423*S29-ROcp23_728*C29);
    ROcp23_829 = -(ROcp23_523*S29-ROcp23_828*C29);
    ROcp23_929 = -(ROcp23_623*S29-ROcp23_928*C29);
    ROcp23_130 = ROcp23_128*C30+ROcp23_429*S30;
    ROcp23_230 = ROcp23_228*C30+ROcp23_529*S30;
    ROcp23_330 = ROcp23_328*C30+ROcp23_629*S30;
    ROcp23_430 = -(ROcp23_128*S30-ROcp23_429*C30);
    ROcp23_530 = -(ROcp23_228*S30-ROcp23_529*C30);
    ROcp23_630 = -(ROcp23_328*S30-ROcp23_629*C30);
    RLcp23_128 = ROcp23_123*s->dpt[1][45]+ROcp23_423*s->dpt[2][45]+ROcp23_722*s->dpt[3][45];
    RLcp23_228 = ROcp23_223*s->dpt[1][45]+ROcp23_523*s->dpt[2][45]+ROcp23_822*s->dpt[3][45];
    RLcp23_328 = ROcp23_323*s->dpt[1][45]+ROcp23_623*s->dpt[2][45]+ROcp23_922*s->dpt[3][45];
    OMcp23_128 = OMcp23_123+ROcp23_423*qd[28];
    OMcp23_228 = OMcp23_223+ROcp23_523*qd[28];
    OMcp23_328 = OMcp23_323+ROcp23_623*qd[28];
    RLcp23_129 = ROcp23_423*s->dpt[2][56];
    RLcp23_229 = ROcp23_523*s->dpt[2][56];
    RLcp23_329 = ROcp23_623*s->dpt[2][56];
    OMcp23_129 = OMcp23_128+ROcp23_128*qd[29];
    OMcp23_229 = OMcp23_228+ROcp23_228*qd[29];
    OMcp23_329 = OMcp23_328+ROcp23_328*qd[29];
    RLcp23_130 = ROcp23_729*s->dpt[3][58];
    RLcp23_230 = ROcp23_829*s->dpt[3][58];
    RLcp23_330 = ROcp23_929*s->dpt[3][58];
    OMcp23_130 = OMcp23_129+ROcp23_729*qd[30];
    OMcp23_230 = OMcp23_229+ROcp23_829*qd[30];
    OMcp23_330 = OMcp23_329+ROcp23_929*qd[30];
    RLcp23_131 = ROcp23_729*s->dpt[3][61];
    RLcp23_231 = ROcp23_829*s->dpt[3][61];
    RLcp23_331 = ROcp23_929*s->dpt[3][61];
    OMcp23_131 = OMcp23_130+ROcp23_430*qd[31];
    OMcp23_231 = OMcp23_230+ROcp23_530*qd[31];
    OMcp23_331 = OMcp23_330+ROcp23_630*qd[31];
    RLcp23_155 = ROcp23_430*s->dpt[2][63]+s->dpt[1][63]*(ROcp23_130*C31-ROcp23_729*S31)+s->dpt[3][63]*(ROcp23_130*S31+
 ROcp23_729*C31);
    RLcp23_255 = ROcp23_530*s->dpt[2][63]+s->dpt[1][63]*(ROcp23_230*C31-ROcp23_829*S31)+s->dpt[3][63]*(ROcp23_230*S31+
 ROcp23_829*C31);
    RLcp23_355 = ROcp23_630*s->dpt[2][63]+s->dpt[1][63]*(ROcp23_330*C31-ROcp23_929*S31)+s->dpt[3][63]*(ROcp23_330*S31+
 ROcp23_929*C31);
    POcp23_155 = RLcp23_121+RLcp23_123+RLcp23_128+RLcp23_129+RLcp23_130+RLcp23_131+RLcp23_155+q[1];
    POcp23_255 = RLcp23_221+RLcp23_223+RLcp23_228+RLcp23_229+RLcp23_230+RLcp23_231+RLcp23_255+q[2];
    POcp23_355 = RLcp23_321+RLcp23_323+RLcp23_328+RLcp23_329+RLcp23_330+RLcp23_331+RLcp23_355+q[3];
    VIcp23_155 = qd[1]+OMcp23_222*RLcp23_323+OMcp23_223*RLcp23_328+OMcp23_228*RLcp23_329+OMcp23_229*RLcp23_330+OMcp23_230*
 RLcp23_331+OMcp23_231*RLcp23_355+OMcp23_26*RLcp23_321-OMcp23_322*RLcp23_223-OMcp23_323*RLcp23_228-OMcp23_328*RLcp23_229-
 OMcp23_329*RLcp23_230-OMcp23_330*RLcp23_231-OMcp23_331*RLcp23_255-OMcp23_36*RLcp23_221;
    VIcp23_255 = qd[2]-OMcp23_122*RLcp23_323-OMcp23_123*RLcp23_328-OMcp23_128*RLcp23_329-OMcp23_129*RLcp23_330-OMcp23_130*
 RLcp23_331-OMcp23_131*RLcp23_355-OMcp23_16*RLcp23_321+OMcp23_322*RLcp23_123+OMcp23_323*RLcp23_128+OMcp23_328*RLcp23_129+
 OMcp23_329*RLcp23_130+OMcp23_330*RLcp23_131+OMcp23_331*RLcp23_155+OMcp23_36*RLcp23_121;
    VIcp23_355 = qd[3]+OMcp23_122*RLcp23_223+OMcp23_123*RLcp23_228+OMcp23_128*RLcp23_229+OMcp23_129*RLcp23_230+OMcp23_130*
 RLcp23_231+OMcp23_131*RLcp23_255+OMcp23_16*RLcp23_221-OMcp23_222*RLcp23_123-OMcp23_223*RLcp23_128-OMcp23_228*RLcp23_129-
 OMcp23_229*RLcp23_130-OMcp23_230*RLcp23_131-OMcp23_231*RLcp23_155-OMcp23_26*RLcp23_121;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_155;
    sens->P[2] = POcp23_255;
    sens->P[3] = POcp23_355;
    sens->V[1] = VIcp23_155;
    sens->V[2] = VIcp23_255;
    sens->V[3] = VIcp23_355;
    sens->OM[1] = OMcp23_131;
    sens->OM[2] = OMcp23_231;
    sens->OM[3] = OMcp23_331;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
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
    OMcp24_16 = qd[4]+qd[6]*S5;
    OMcp24_26 = ROcp24_85*qd[6]+qd[5]*C4;
    OMcp24_36 = ROcp24_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = ROcp24_16;
    sens->R[1][2] = ROcp24_26;
    sens->R[1][3] = ROcp24_36;
    sens->R[2][1] = ROcp24_46;
    sens->R[2][2] = ROcp24_56;
    sens->R[2][3] = ROcp24_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp24_85;
    sens->R[3][3] = ROcp24_95;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = OMcp24_16;
    sens->OM[2] = OMcp24_26;
    sens->OM[3] = OMcp24_36;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
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
    OMcp25_16 = qd[4]+qd[6]*S5;
    OMcp25_26 = ROcp25_85*qd[6]+qd[5]*C4;
    OMcp25_36 = ROcp25_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_26_0_2 = = 
 
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
    OMcp25_17 = OMcp25_16+ROcp25_46*qd[7];
    OMcp25_27 = OMcp25_26+ROcp25_56*qd[7];
    OMcp25_37 = OMcp25_36+ROcp25_66*qd[7];
    RLcp25_18 = ROcp25_46*s->dpt[2][6];
    RLcp25_28 = ROcp25_56*s->dpt[2][6];
    RLcp25_38 = ROcp25_66*s->dpt[2][6];
    OMcp25_18 = OMcp25_17+ROcp25_17*qd[8];
    OMcp25_28 = OMcp25_27+ROcp25_27*qd[8];
    OMcp25_38 = OMcp25_37+ROcp25_37*qd[8];
    RLcp25_19 = ROcp25_78*s->dpt[3][8];
    RLcp25_29 = ROcp25_88*s->dpt[3][8];
    RLcp25_39 = ROcp25_98*s->dpt[3][8];
    POcp25_19 = RLcp25_17+RLcp25_18+RLcp25_19+q[1];
    POcp25_29 = RLcp25_27+RLcp25_28+RLcp25_29+q[2];
    POcp25_39 = RLcp25_37+RLcp25_38+RLcp25_39+q[3];
    OMcp25_19 = OMcp25_18+ROcp25_78*qd[9];
    OMcp25_29 = OMcp25_28+ROcp25_88*qd[9];
    OMcp25_39 = OMcp25_38+ROcp25_98*qd[9];
    VIcp25_19 = qd[1]+OMcp25_26*RLcp25_37+OMcp25_27*RLcp25_38+OMcp25_28*RLcp25_39-OMcp25_36*RLcp25_27-OMcp25_37*RLcp25_28-
 OMcp25_38*RLcp25_29;
    VIcp25_29 = qd[2]-OMcp25_16*RLcp25_37-OMcp25_17*RLcp25_38-OMcp25_18*RLcp25_39+OMcp25_36*RLcp25_17+OMcp25_37*RLcp25_18+
 OMcp25_38*RLcp25_19;
    VIcp25_39 = qd[3]+OMcp25_16*RLcp25_27+OMcp25_17*RLcp25_28+OMcp25_18*RLcp25_29-OMcp25_26*RLcp25_17-OMcp25_27*RLcp25_18-
 OMcp25_28*RLcp25_19;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_19;
    sens->P[2] = POcp25_29;
    sens->P[3] = POcp25_39;
    sens->R[1][1] = ROcp25_19;
    sens->R[1][2] = ROcp25_29;
    sens->R[1][3] = ROcp25_39;
    sens->R[2][1] = ROcp25_49;
    sens->R[2][2] = ROcp25_59;
    sens->R[2][3] = ROcp25_69;
    sens->R[3][1] = ROcp25_78;
    sens->R[3][2] = ROcp25_88;
    sens->R[3][3] = ROcp25_98;
    sens->V[1] = VIcp25_19;
    sens->V[2] = VIcp25_29;
    sens->V[3] = VIcp25_39;
    sens->OM[1] = OMcp25_19;
    sens->OM[2] = OMcp25_29;
    sens->OM[3] = OMcp25_39;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
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
    OMcp26_16 = qd[4]+qd[6]*S5;
    OMcp26_26 = ROcp26_85*qd[6]+qd[5]*C4;
    OMcp26_36 = ROcp26_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_27_0_2 = = 
 
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
    OMcp26_17 = OMcp26_16+ROcp26_46*qd[7];
    OMcp26_27 = OMcp26_26+ROcp26_56*qd[7];
    OMcp26_37 = OMcp26_36+ROcp26_66*qd[7];
    RLcp26_18 = ROcp26_46*s->dpt[2][6];
    RLcp26_28 = ROcp26_56*s->dpt[2][6];
    RLcp26_38 = ROcp26_66*s->dpt[2][6];
    OMcp26_18 = OMcp26_17+ROcp26_17*qd[8];
    OMcp26_28 = OMcp26_27+ROcp26_27*qd[8];
    OMcp26_38 = OMcp26_37+ROcp26_37*qd[8];
    RLcp26_19 = ROcp26_78*s->dpt[3][8];
    RLcp26_29 = ROcp26_88*s->dpt[3][8];
    RLcp26_39 = ROcp26_98*s->dpt[3][8];
    OMcp26_19 = OMcp26_18+ROcp26_78*qd[9];
    OMcp26_29 = OMcp26_28+ROcp26_88*qd[9];
    OMcp26_39 = OMcp26_38+ROcp26_98*qd[9];
    RLcp26_110 = ROcp26_78*s->dpt[3][11];
    RLcp26_210 = ROcp26_88*s->dpt[3][11];
    RLcp26_310 = ROcp26_98*s->dpt[3][11];
    POcp26_110 = RLcp26_110+RLcp26_17+RLcp26_18+RLcp26_19+q[1];
    POcp26_210 = RLcp26_210+RLcp26_27+RLcp26_28+RLcp26_29+q[2];
    POcp26_310 = RLcp26_310+RLcp26_37+RLcp26_38+RLcp26_39+q[3];
    OMcp26_110 = OMcp26_19+ROcp26_49*qd[10];
    OMcp26_210 = OMcp26_29+ROcp26_59*qd[10];
    OMcp26_310 = OMcp26_39+ROcp26_69*qd[10];
    VIcp26_110 = qd[1]+OMcp26_26*RLcp26_37+OMcp26_27*RLcp26_38+OMcp26_28*RLcp26_39+OMcp26_29*RLcp26_310-OMcp26_36*
 RLcp26_27-OMcp26_37*RLcp26_28-OMcp26_38*RLcp26_29-OMcp26_39*RLcp26_210;
    VIcp26_210 = qd[2]-OMcp26_16*RLcp26_37-OMcp26_17*RLcp26_38-OMcp26_18*RLcp26_39-OMcp26_19*RLcp26_310+OMcp26_36*
 RLcp26_17+OMcp26_37*RLcp26_18+OMcp26_38*RLcp26_19+OMcp26_39*RLcp26_110;
    VIcp26_310 = qd[3]+OMcp26_16*RLcp26_27+OMcp26_17*RLcp26_28+OMcp26_18*RLcp26_29+OMcp26_19*RLcp26_210-OMcp26_26*
 RLcp26_17-OMcp26_27*RLcp26_18-OMcp26_28*RLcp26_19-OMcp26_29*RLcp26_110;

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_110;
    sens->P[2] = POcp26_210;
    sens->P[3] = POcp26_310;
    sens->R[1][1] = ROcp26_110;
    sens->R[1][2] = ROcp26_210;
    sens->R[1][3] = ROcp26_310;
    sens->R[2][1] = ROcp26_49;
    sens->R[2][2] = ROcp26_59;
    sens->R[2][3] = ROcp26_69;
    sens->R[3][1] = ROcp26_710;
    sens->R[3][2] = ROcp26_810;
    sens->R[3][3] = ROcp26_910;
    sens->V[1] = VIcp26_110;
    sens->V[2] = VIcp26_210;
    sens->V[3] = VIcp26_310;
    sens->OM[1] = OMcp26_110;
    sens->OM[2] = OMcp26_210;
    sens->OM[3] = OMcp26_310;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
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
    OMcp27_16 = qd[4]+qd[6]*S5;
    OMcp27_26 = ROcp27_85*qd[6]+qd[5]*C4;
    OMcp27_36 = ROcp27_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_28_0_2 = = 
 
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
    OMcp27_17 = OMcp27_16+ROcp27_46*qd[7];
    OMcp27_27 = OMcp27_26+ROcp27_56*qd[7];
    OMcp27_37 = OMcp27_36+ROcp27_66*qd[7];
    RLcp27_18 = ROcp27_46*s->dpt[2][6];
    RLcp27_28 = ROcp27_56*s->dpt[2][6];
    RLcp27_38 = ROcp27_66*s->dpt[2][6];
    OMcp27_18 = OMcp27_17+ROcp27_17*qd[8];
    OMcp27_28 = OMcp27_27+ROcp27_27*qd[8];
    OMcp27_38 = OMcp27_37+ROcp27_37*qd[8];
    RLcp27_19 = ROcp27_78*s->dpt[3][8];
    RLcp27_29 = ROcp27_88*s->dpt[3][8];
    RLcp27_39 = ROcp27_98*s->dpt[3][8];
    OMcp27_19 = OMcp27_18+ROcp27_78*qd[9];
    OMcp27_29 = OMcp27_28+ROcp27_88*qd[9];
    OMcp27_39 = OMcp27_38+ROcp27_98*qd[9];
    RLcp27_110 = ROcp27_78*s->dpt[3][11];
    RLcp27_210 = ROcp27_88*s->dpt[3][11];
    RLcp27_310 = ROcp27_98*s->dpt[3][11];
    OMcp27_110 = OMcp27_19+ROcp27_49*qd[10];
    OMcp27_210 = OMcp27_29+ROcp27_59*qd[10];
    OMcp27_310 = OMcp27_39+ROcp27_69*qd[10];
    RLcp27_111 = ROcp27_710*s->dpt[3][14];
    RLcp27_211 = ROcp27_810*s->dpt[3][14];
    RLcp27_311 = ROcp27_910*s->dpt[3][14];
    OMcp27_112 = OMcp27_110+ROcp27_110*qd[11]+ROcp27_411*qd[12];
    OMcp27_212 = OMcp27_210+ROcp27_210*qd[11]+ROcp27_511*qd[12];
    OMcp27_312 = OMcp27_310+ROcp27_310*qd[11]+ROcp27_611*qd[12];
    RLcp27_159 = ROcp27_712*s->dpt[3][19];
    RLcp27_259 = ROcp27_812*s->dpt[3][19];
    RLcp27_359 = ROcp27_912*s->dpt[3][19];
    POcp27_159 = RLcp27_110+RLcp27_111+RLcp27_159+RLcp27_17+RLcp27_18+RLcp27_19+q[1];
    POcp27_259 = RLcp27_210+RLcp27_211+RLcp27_259+RLcp27_27+RLcp27_28+RLcp27_29+q[2];
    POcp27_359 = RLcp27_310+RLcp27_311+RLcp27_359+RLcp27_37+RLcp27_38+RLcp27_39+q[3];
    VIcp27_159 = qd[1]+OMcp27_210*RLcp27_311+OMcp27_212*RLcp27_359+OMcp27_26*RLcp27_37+OMcp27_27*RLcp27_38+OMcp27_28*
 RLcp27_39+OMcp27_29*RLcp27_310-OMcp27_310*RLcp27_211-OMcp27_312*RLcp27_259-OMcp27_36*RLcp27_27-OMcp27_37*RLcp27_28-OMcp27_38
 *RLcp27_29-OMcp27_39*RLcp27_210;
    VIcp27_259 = qd[2]-OMcp27_110*RLcp27_311-OMcp27_112*RLcp27_359-OMcp27_16*RLcp27_37-OMcp27_17*RLcp27_38-OMcp27_18*
 RLcp27_39-OMcp27_19*RLcp27_310+OMcp27_310*RLcp27_111+OMcp27_312*RLcp27_159+OMcp27_36*RLcp27_17+OMcp27_37*RLcp27_18+OMcp27_38
 *RLcp27_19+OMcp27_39*RLcp27_110;
    VIcp27_359 = qd[3]+OMcp27_110*RLcp27_211+OMcp27_112*RLcp27_259+OMcp27_16*RLcp27_27+OMcp27_17*RLcp27_28+OMcp27_18*
 RLcp27_29+OMcp27_19*RLcp27_210-OMcp27_210*RLcp27_111-OMcp27_212*RLcp27_159-OMcp27_26*RLcp27_17-OMcp27_27*RLcp27_18-OMcp27_28
 *RLcp27_19-OMcp27_29*RLcp27_110;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_159;
    sens->P[2] = POcp27_259;
    sens->P[3] = POcp27_359;
    sens->R[1][1] = ROcp27_112;
    sens->R[1][2] = ROcp27_212;
    sens->R[1][3] = ROcp27_312;
    sens->R[2][1] = ROcp27_411;
    sens->R[2][2] = ROcp27_511;
    sens->R[2][3] = ROcp27_611;
    sens->R[3][1] = ROcp27_712;
    sens->R[3][2] = ROcp27_812;
    sens->R[3][3] = ROcp27_912;
    sens->V[1] = VIcp27_159;
    sens->V[2] = VIcp27_259;
    sens->V[3] = VIcp27_359;
    sens->OM[1] = OMcp27_112;
    sens->OM[2] = OMcp27_212;
    sens->OM[3] = OMcp27_312;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
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
    OMcp28_26 = OMcp28_25+ROcp28_85*qd[6];
    OMcp28_36 = OMcp28_35+ROcp28_95*qd[6];
    OPcp28_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp28_26 = ROcp28_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp28_35*S5-ROcp28_95*qd[4]);
    OPcp28_36 = ROcp28_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp28_25*S5-ROcp28_85*qd[4]);

// = = Block_1_0_0_29_0_2 = = 
 
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
    OMcp28_17 = OMcp28_16+ROcp28_46*qd[7];
    OMcp28_27 = OMcp28_26+ROcp28_56*qd[7];
    OMcp28_37 = OMcp28_36+ROcp28_66*qd[7];
    ORcp28_17 = OMcp28_26*RLcp28_37-OMcp28_36*RLcp28_27;
    ORcp28_27 = -(OMcp28_16*RLcp28_37-OMcp28_36*RLcp28_17);
    ORcp28_37 = OMcp28_16*RLcp28_27-OMcp28_26*RLcp28_17;
    OPcp28_17 = OPcp28_16+ROcp28_46*qdd[7]+qd[7]*(OMcp28_26*ROcp28_66-OMcp28_36*ROcp28_56);
    OPcp28_27 = OPcp28_26+ROcp28_56*qdd[7]-qd[7]*(OMcp28_16*ROcp28_66-OMcp28_36*ROcp28_46);
    OPcp28_37 = OPcp28_36+ROcp28_66*qdd[7]+qd[7]*(OMcp28_16*ROcp28_56-OMcp28_26*ROcp28_46);
    RLcp28_18 = ROcp28_46*s->dpt[2][6];
    RLcp28_28 = ROcp28_56*s->dpt[2][6];
    RLcp28_38 = ROcp28_66*s->dpt[2][6];
    OMcp28_18 = OMcp28_17+ROcp28_17*qd[8];
    OMcp28_28 = OMcp28_27+ROcp28_27*qd[8];
    OMcp28_38 = OMcp28_37+ROcp28_37*qd[8];
    ORcp28_18 = OMcp28_27*RLcp28_38-OMcp28_37*RLcp28_28;
    ORcp28_28 = -(OMcp28_17*RLcp28_38-OMcp28_37*RLcp28_18);
    ORcp28_38 = OMcp28_17*RLcp28_28-OMcp28_27*RLcp28_18;
    OPcp28_18 = OPcp28_17+ROcp28_17*qdd[8]+qd[8]*(OMcp28_27*ROcp28_37-OMcp28_37*ROcp28_27);
    OPcp28_28 = OPcp28_27+ROcp28_27*qdd[8]-qd[8]*(OMcp28_17*ROcp28_37-OMcp28_37*ROcp28_17);
    OPcp28_38 = OPcp28_37+ROcp28_37*qdd[8]+qd[8]*(OMcp28_17*ROcp28_27-OMcp28_27*ROcp28_17);
    RLcp28_19 = ROcp28_78*s->dpt[3][8];
    RLcp28_29 = ROcp28_88*s->dpt[3][8];
    RLcp28_39 = ROcp28_98*s->dpt[3][8];
    OMcp28_19 = OMcp28_18+ROcp28_78*qd[9];
    OMcp28_29 = OMcp28_28+ROcp28_88*qd[9];
    OMcp28_39 = OMcp28_38+ROcp28_98*qd[9];
    ORcp28_19 = OMcp28_28*RLcp28_39-OMcp28_38*RLcp28_29;
    ORcp28_29 = -(OMcp28_18*RLcp28_39-OMcp28_38*RLcp28_19);
    ORcp28_39 = OMcp28_18*RLcp28_29-OMcp28_28*RLcp28_19;
    OPcp28_19 = OPcp28_18+ROcp28_78*qdd[9]+qd[9]*(OMcp28_28*ROcp28_98-OMcp28_38*ROcp28_88);
    OPcp28_29 = OPcp28_28+ROcp28_88*qdd[9]-qd[9]*(OMcp28_18*ROcp28_98-OMcp28_38*ROcp28_78);
    OPcp28_39 = OPcp28_38+ROcp28_98*qdd[9]+qd[9]*(OMcp28_18*ROcp28_88-OMcp28_28*ROcp28_78);
    RLcp28_110 = ROcp28_78*s->dpt[3][11];
    RLcp28_210 = ROcp28_88*s->dpt[3][11];
    RLcp28_310 = ROcp28_98*s->dpt[3][11];
    OMcp28_110 = OMcp28_19+ROcp28_49*qd[10];
    OMcp28_210 = OMcp28_29+ROcp28_59*qd[10];
    OMcp28_310 = OMcp28_39+ROcp28_69*qd[10];
    ORcp28_110 = OMcp28_29*RLcp28_310-OMcp28_39*RLcp28_210;
    ORcp28_210 = -(OMcp28_19*RLcp28_310-OMcp28_39*RLcp28_110);
    ORcp28_310 = OMcp28_19*RLcp28_210-OMcp28_29*RLcp28_110;
    OPcp28_110 = OPcp28_19+ROcp28_49*qdd[10]+qd[10]*(OMcp28_29*ROcp28_69-OMcp28_39*ROcp28_59);
    OPcp28_210 = OPcp28_29+ROcp28_59*qdd[10]-qd[10]*(OMcp28_19*ROcp28_69-OMcp28_39*ROcp28_49);
    OPcp28_310 = OPcp28_39+ROcp28_69*qdd[10]+qd[10]*(OMcp28_19*ROcp28_59-OMcp28_29*ROcp28_49);
    RLcp28_111 = ROcp28_710*s->dpt[3][14];
    RLcp28_211 = ROcp28_810*s->dpt[3][14];
    RLcp28_311 = ROcp28_910*s->dpt[3][14];
    OMcp28_111 = OMcp28_110+ROcp28_110*qd[11];
    OMcp28_211 = OMcp28_210+ROcp28_210*qd[11];
    OMcp28_311 = OMcp28_310+ROcp28_310*qd[11];
    ORcp28_111 = OMcp28_210*RLcp28_311-OMcp28_310*RLcp28_211;
    ORcp28_211 = -(OMcp28_110*RLcp28_311-OMcp28_310*RLcp28_111);
    ORcp28_311 = OMcp28_110*RLcp28_211-OMcp28_210*RLcp28_111;
    OMcp28_112 = OMcp28_111+ROcp28_411*qd[12];
    OMcp28_212 = OMcp28_211+ROcp28_511*qd[12];
    OMcp28_312 = OMcp28_311+ROcp28_611*qd[12];
    OPcp28_112 = OPcp28_110+ROcp28_110*qdd[11]+ROcp28_411*qdd[12]+qd[11]*(OMcp28_210*ROcp28_310-OMcp28_310*ROcp28_210)+
 qd[12]*(OMcp28_211*ROcp28_611-OMcp28_311*ROcp28_511);
    OPcp28_212 = OPcp28_210+ROcp28_210*qdd[11]+ROcp28_511*qdd[12]-qd[11]*(OMcp28_110*ROcp28_310-OMcp28_310*ROcp28_110)-
 qd[12]*(OMcp28_111*ROcp28_611-OMcp28_311*ROcp28_411);
    OPcp28_312 = OPcp28_310+ROcp28_310*qdd[11]+ROcp28_611*qdd[12]+qd[11]*(OMcp28_110*ROcp28_210-OMcp28_210*ROcp28_110)+
 qd[12]*(OMcp28_111*ROcp28_511-OMcp28_211*ROcp28_411);
    RLcp28_113 = ROcp28_112*s->dpt[1][20]+ROcp28_712*s->dpt[3][20];
    RLcp28_213 = ROcp28_212*s->dpt[1][20]+ROcp28_812*s->dpt[3][20];
    RLcp28_313 = ROcp28_312*s->dpt[1][20]+ROcp28_912*s->dpt[3][20];
    POcp28_113 = RLcp28_110+RLcp28_111+RLcp28_113+RLcp28_17+RLcp28_18+RLcp28_19+q[1];
    POcp28_213 = RLcp28_210+RLcp28_211+RLcp28_213+RLcp28_27+RLcp28_28+RLcp28_29+q[2];
    POcp28_313 = RLcp28_310+RLcp28_311+RLcp28_313+RLcp28_37+RLcp28_38+RLcp28_39+q[3];
    OMcp28_113 = OMcp28_112+ROcp28_411*qd[13];
    OMcp28_213 = OMcp28_212+ROcp28_511*qd[13];
    OMcp28_313 = OMcp28_312+ROcp28_611*qd[13];
    ORcp28_113 = OMcp28_212*RLcp28_313-OMcp28_312*RLcp28_213;
    ORcp28_213 = -(OMcp28_112*RLcp28_313-OMcp28_312*RLcp28_113);
    ORcp28_313 = OMcp28_112*RLcp28_213-OMcp28_212*RLcp28_113;
    VIcp28_113 = ORcp28_110+ORcp28_111+ORcp28_113+ORcp28_17+ORcp28_18+ORcp28_19+qd[1];
    VIcp28_213 = ORcp28_210+ORcp28_211+ORcp28_213+ORcp28_27+ORcp28_28+ORcp28_29+qd[2];
    VIcp28_313 = ORcp28_310+ORcp28_311+ORcp28_313+ORcp28_37+ORcp28_38+ORcp28_39+qd[3];
    OPcp28_113 = OPcp28_112+ROcp28_411*qdd[13]+qd[13]*(OMcp28_212*ROcp28_611-OMcp28_312*ROcp28_511);
    OPcp28_213 = OPcp28_212+ROcp28_511*qdd[13]-qd[13]*(OMcp28_112*ROcp28_611-OMcp28_312*ROcp28_411);
    OPcp28_313 = OPcp28_312+ROcp28_611*qdd[13]+qd[13]*(OMcp28_112*ROcp28_511-OMcp28_212*ROcp28_411);
    ACcp28_113 = qdd[1]+OMcp28_210*ORcp28_311+OMcp28_212*ORcp28_313+OMcp28_26*ORcp28_37+OMcp28_27*ORcp28_38+OMcp28_28*
 ORcp28_39+OMcp28_29*ORcp28_310-OMcp28_310*ORcp28_211-OMcp28_312*ORcp28_213-OMcp28_36*ORcp28_27-OMcp28_37*ORcp28_28-OMcp28_38
 *ORcp28_29-OMcp28_39*ORcp28_210+OPcp28_210*RLcp28_311+OPcp28_212*RLcp28_313+OPcp28_26*RLcp28_37+OPcp28_27*RLcp28_38+
 OPcp28_28*RLcp28_39+OPcp28_29*RLcp28_310-OPcp28_310*RLcp28_211-OPcp28_312*RLcp28_213-OPcp28_36*RLcp28_27-OPcp28_37*RLcp28_28
 -OPcp28_38*RLcp28_29-OPcp28_39*RLcp28_210;
    ACcp28_213 = qdd[2]-OMcp28_110*ORcp28_311-OMcp28_112*ORcp28_313-OMcp28_16*ORcp28_37-OMcp28_17*ORcp28_38-OMcp28_18*
 ORcp28_39-OMcp28_19*ORcp28_310+OMcp28_310*ORcp28_111+OMcp28_312*ORcp28_113+OMcp28_36*ORcp28_17+OMcp28_37*ORcp28_18+OMcp28_38
 *ORcp28_19+OMcp28_39*ORcp28_110-OPcp28_110*RLcp28_311-OPcp28_112*RLcp28_313-OPcp28_16*RLcp28_37-OPcp28_17*RLcp28_38-
 OPcp28_18*RLcp28_39-OPcp28_19*RLcp28_310+OPcp28_310*RLcp28_111+OPcp28_312*RLcp28_113+OPcp28_36*RLcp28_17+OPcp28_37*RLcp28_18
 +OPcp28_38*RLcp28_19+OPcp28_39*RLcp28_110;
    ACcp28_313 = qdd[3]+OMcp28_110*ORcp28_211+OMcp28_112*ORcp28_213+OMcp28_16*ORcp28_27+OMcp28_17*ORcp28_28+OMcp28_18*
 ORcp28_29+OMcp28_19*ORcp28_210-OMcp28_210*ORcp28_111-OMcp28_212*ORcp28_113-OMcp28_26*ORcp28_17-OMcp28_27*ORcp28_18-OMcp28_28
 *ORcp28_19-OMcp28_29*ORcp28_110+OPcp28_110*RLcp28_211+OPcp28_112*RLcp28_213+OPcp28_16*RLcp28_27+OPcp28_17*RLcp28_28+
 OPcp28_18*RLcp28_29+OPcp28_19*RLcp28_210-OPcp28_210*RLcp28_111-OPcp28_212*RLcp28_113-OPcp28_26*RLcp28_17-OPcp28_27*RLcp28_18
 -OPcp28_28*RLcp28_19-OPcp28_29*RLcp28_110;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_113;
    sens->P[2] = POcp28_213;
    sens->P[3] = POcp28_313;
    sens->R[1][1] = ROcp28_113;
    sens->R[1][2] = ROcp28_213;
    sens->R[1][3] = ROcp28_313;
    sens->R[2][1] = ROcp28_411;
    sens->R[2][2] = ROcp28_511;
    sens->R[2][3] = ROcp28_611;
    sens->R[3][1] = ROcp28_713;
    sens->R[3][2] = ROcp28_813;
    sens->R[3][3] = ROcp28_913;
    sens->V[1] = VIcp28_113;
    sens->V[2] = VIcp28_213;
    sens->V[3] = VIcp28_313;
    sens->OM[1] = OMcp28_113;
    sens->OM[2] = OMcp28_213;
    sens->OM[3] = OMcp28_313;
    sens->A[1] = ACcp28_113;
    sens->A[2] = ACcp28_213;
    sens->A[3] = ACcp28_313;
    sens->OMP[1] = OPcp28_113;
    sens->OMP[2] = OPcp28_213;
    sens->OMP[3] = OPcp28_313;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
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
    OMcp29_16 = qd[4]+qd[6]*S5;
    OMcp29_26 = ROcp29_85*qd[6]+qd[5]*C4;
    OMcp29_36 = ROcp29_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_30_0_3 = = 
 
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
    OMcp29_114 = OMcp29_16+ROcp29_46*qd[14];
    OMcp29_214 = OMcp29_26+ROcp29_56*qd[14];
    OMcp29_314 = OMcp29_36+ROcp29_66*qd[14];
    RLcp29_115 = ROcp29_46*s->dpt[2][22];
    RLcp29_215 = ROcp29_56*s->dpt[2][22];
    RLcp29_315 = ROcp29_66*s->dpt[2][22];
    OMcp29_115 = OMcp29_114+ROcp29_114*qd[15];
    OMcp29_215 = OMcp29_214+ROcp29_214*qd[15];
    OMcp29_315 = OMcp29_314+ROcp29_314*qd[15];
    RLcp29_116 = ROcp29_715*s->dpt[3][24];
    RLcp29_216 = ROcp29_815*s->dpt[3][24];
    RLcp29_316 = ROcp29_915*s->dpt[3][24];
    POcp29_116 = RLcp29_114+RLcp29_115+RLcp29_116+q[1];
    POcp29_216 = RLcp29_214+RLcp29_215+RLcp29_216+q[2];
    POcp29_316 = RLcp29_314+RLcp29_315+RLcp29_316+q[3];
    OMcp29_116 = OMcp29_115+ROcp29_715*qd[16];
    OMcp29_216 = OMcp29_215+ROcp29_815*qd[16];
    OMcp29_316 = OMcp29_315+ROcp29_915*qd[16];
    VIcp29_116 = qd[1]+OMcp29_214*RLcp29_315+OMcp29_215*RLcp29_316+OMcp29_26*RLcp29_314-OMcp29_314*RLcp29_215-OMcp29_315*
 RLcp29_216-OMcp29_36*RLcp29_214;
    VIcp29_216 = qd[2]-OMcp29_114*RLcp29_315-OMcp29_115*RLcp29_316-OMcp29_16*RLcp29_314+OMcp29_314*RLcp29_115+OMcp29_315*
 RLcp29_116+OMcp29_36*RLcp29_114;
    VIcp29_316 = qd[3]+OMcp29_114*RLcp29_215+OMcp29_115*RLcp29_216+OMcp29_16*RLcp29_214-OMcp29_214*RLcp29_115-OMcp29_215*
 RLcp29_116-OMcp29_26*RLcp29_114;

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_116;
    sens->P[2] = POcp29_216;
    sens->P[3] = POcp29_316;
    sens->R[1][1] = ROcp29_116;
    sens->R[1][2] = ROcp29_216;
    sens->R[1][3] = ROcp29_316;
    sens->R[2][1] = ROcp29_416;
    sens->R[2][2] = ROcp29_516;
    sens->R[2][3] = ROcp29_616;
    sens->R[3][1] = ROcp29_715;
    sens->R[3][2] = ROcp29_815;
    sens->R[3][3] = ROcp29_915;
    sens->V[1] = VIcp29_116;
    sens->V[2] = VIcp29_216;
    sens->V[3] = VIcp29_316;
    sens->OM[1] = OMcp29_116;
    sens->OM[2] = OMcp29_216;
    sens->OM[3] = OMcp29_316;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
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
    OMcp30_16 = qd[4]+qd[6]*S5;
    OMcp30_26 = ROcp30_85*qd[6]+qd[5]*C4;
    OMcp30_36 = ROcp30_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_31_0_3 = = 
 
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
    OMcp30_114 = OMcp30_16+ROcp30_46*qd[14];
    OMcp30_214 = OMcp30_26+ROcp30_56*qd[14];
    OMcp30_314 = OMcp30_36+ROcp30_66*qd[14];
    RLcp30_115 = ROcp30_46*s->dpt[2][22];
    RLcp30_215 = ROcp30_56*s->dpt[2][22];
    RLcp30_315 = ROcp30_66*s->dpt[2][22];
    OMcp30_115 = OMcp30_114+ROcp30_114*qd[15];
    OMcp30_215 = OMcp30_214+ROcp30_214*qd[15];
    OMcp30_315 = OMcp30_314+ROcp30_314*qd[15];
    RLcp30_116 = ROcp30_715*s->dpt[3][24];
    RLcp30_216 = ROcp30_815*s->dpt[3][24];
    RLcp30_316 = ROcp30_915*s->dpt[3][24];
    OMcp30_116 = OMcp30_115+ROcp30_715*qd[16];
    OMcp30_216 = OMcp30_215+ROcp30_815*qd[16];
    OMcp30_316 = OMcp30_315+ROcp30_915*qd[16];
    RLcp30_117 = ROcp30_715*s->dpt[3][27];
    RLcp30_217 = ROcp30_815*s->dpt[3][27];
    RLcp30_317 = ROcp30_915*s->dpt[3][27];
    POcp30_117 = RLcp30_114+RLcp30_115+RLcp30_116+RLcp30_117+q[1];
    POcp30_217 = RLcp30_214+RLcp30_215+RLcp30_216+RLcp30_217+q[2];
    POcp30_317 = RLcp30_314+RLcp30_315+RLcp30_316+RLcp30_317+q[3];
    OMcp30_117 = OMcp30_116+ROcp30_416*qd[17];
    OMcp30_217 = OMcp30_216+ROcp30_516*qd[17];
    OMcp30_317 = OMcp30_316+ROcp30_616*qd[17];
    VIcp30_117 = qd[1]+OMcp30_214*RLcp30_315+OMcp30_215*RLcp30_316+OMcp30_216*RLcp30_317+OMcp30_26*RLcp30_314-OMcp30_314*
 RLcp30_215-OMcp30_315*RLcp30_216-OMcp30_316*RLcp30_217-OMcp30_36*RLcp30_214;
    VIcp30_217 = qd[2]-OMcp30_114*RLcp30_315-OMcp30_115*RLcp30_316-OMcp30_116*RLcp30_317-OMcp30_16*RLcp30_314+OMcp30_314*
 RLcp30_115+OMcp30_315*RLcp30_116+OMcp30_316*RLcp30_117+OMcp30_36*RLcp30_114;
    VIcp30_317 = qd[3]+OMcp30_114*RLcp30_215+OMcp30_115*RLcp30_216+OMcp30_116*RLcp30_217+OMcp30_16*RLcp30_214-OMcp30_214*
 RLcp30_115-OMcp30_215*RLcp30_116-OMcp30_216*RLcp30_117-OMcp30_26*RLcp30_114;

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_117;
    sens->P[2] = POcp30_217;
    sens->P[3] = POcp30_317;
    sens->R[1][1] = ROcp30_117;
    sens->R[1][2] = ROcp30_217;
    sens->R[1][3] = ROcp30_317;
    sens->R[2][1] = ROcp30_416;
    sens->R[2][2] = ROcp30_516;
    sens->R[2][3] = ROcp30_616;
    sens->R[3][1] = ROcp30_717;
    sens->R[3][2] = ROcp30_817;
    sens->R[3][3] = ROcp30_917;
    sens->V[1] = VIcp30_117;
    sens->V[2] = VIcp30_217;
    sens->V[3] = VIcp30_317;
    sens->OM[1] = OMcp30_117;
    sens->OM[2] = OMcp30_217;
    sens->OM[3] = OMcp30_317;
 
// 
break;
case 32:
 


// = = Block_1_0_0_32_0_1 = = 
 
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
    OMcp31_16 = qd[4]+qd[6]*S5;
    OMcp31_26 = ROcp31_85*qd[6]+qd[5]*C4;
    OMcp31_36 = ROcp31_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_32_0_3 = = 
 
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
    OMcp31_114 = OMcp31_16+ROcp31_46*qd[14];
    OMcp31_214 = OMcp31_26+ROcp31_56*qd[14];
    OMcp31_314 = OMcp31_36+ROcp31_66*qd[14];
    RLcp31_115 = ROcp31_46*s->dpt[2][22];
    RLcp31_215 = ROcp31_56*s->dpt[2][22];
    RLcp31_315 = ROcp31_66*s->dpt[2][22];
    OMcp31_115 = OMcp31_114+ROcp31_114*qd[15];
    OMcp31_215 = OMcp31_214+ROcp31_214*qd[15];
    OMcp31_315 = OMcp31_314+ROcp31_314*qd[15];
    RLcp31_116 = ROcp31_715*s->dpt[3][24];
    RLcp31_216 = ROcp31_815*s->dpt[3][24];
    RLcp31_316 = ROcp31_915*s->dpt[3][24];
    OMcp31_116 = OMcp31_115+ROcp31_715*qd[16];
    OMcp31_216 = OMcp31_215+ROcp31_815*qd[16];
    OMcp31_316 = OMcp31_315+ROcp31_915*qd[16];
    RLcp31_117 = ROcp31_715*s->dpt[3][27];
    RLcp31_217 = ROcp31_815*s->dpt[3][27];
    RLcp31_317 = ROcp31_915*s->dpt[3][27];
    OMcp31_117 = OMcp31_116+ROcp31_416*qd[17];
    OMcp31_217 = OMcp31_216+ROcp31_516*qd[17];
    OMcp31_317 = OMcp31_316+ROcp31_616*qd[17];
    RLcp31_118 = ROcp31_717*s->dpt[3][30];
    RLcp31_218 = ROcp31_817*s->dpt[3][30];
    RLcp31_318 = ROcp31_917*s->dpt[3][30];
    OMcp31_119 = OMcp31_117+ROcp31_117*qd[18]+ROcp31_418*qd[19];
    OMcp31_219 = OMcp31_217+ROcp31_217*qd[18]+ROcp31_518*qd[19];
    OMcp31_319 = OMcp31_317+ROcp31_317*qd[18]+ROcp31_618*qd[19];
    RLcp31_163 = ROcp31_719*s->dpt[3][35];
    RLcp31_263 = ROcp31_819*s->dpt[3][35];
    RLcp31_363 = ROcp31_919*s->dpt[3][35];
    POcp31_163 = RLcp31_114+RLcp31_115+RLcp31_116+RLcp31_117+RLcp31_118+RLcp31_163+q[1];
    POcp31_263 = RLcp31_214+RLcp31_215+RLcp31_216+RLcp31_217+RLcp31_218+RLcp31_263+q[2];
    POcp31_363 = RLcp31_314+RLcp31_315+RLcp31_316+RLcp31_317+RLcp31_318+RLcp31_363+q[3];
    VIcp31_163 = qd[1]+OMcp31_214*RLcp31_315+OMcp31_215*RLcp31_316+OMcp31_216*RLcp31_317+OMcp31_217*RLcp31_318+OMcp31_219*
 RLcp31_363+OMcp31_26*RLcp31_314-OMcp31_314*RLcp31_215-OMcp31_315*RLcp31_216-OMcp31_316*RLcp31_217-OMcp31_317*RLcp31_218-
 OMcp31_319*RLcp31_263-OMcp31_36*RLcp31_214;
    VIcp31_263 = qd[2]-OMcp31_114*RLcp31_315-OMcp31_115*RLcp31_316-OMcp31_116*RLcp31_317-OMcp31_117*RLcp31_318-OMcp31_119*
 RLcp31_363-OMcp31_16*RLcp31_314+OMcp31_314*RLcp31_115+OMcp31_315*RLcp31_116+OMcp31_316*RLcp31_117+OMcp31_317*RLcp31_118+
 OMcp31_319*RLcp31_163+OMcp31_36*RLcp31_114;
    VIcp31_363 = qd[3]+OMcp31_114*RLcp31_215+OMcp31_115*RLcp31_216+OMcp31_116*RLcp31_217+OMcp31_117*RLcp31_218+OMcp31_119*
 RLcp31_263+OMcp31_16*RLcp31_214-OMcp31_214*RLcp31_115-OMcp31_215*RLcp31_116-OMcp31_216*RLcp31_117-OMcp31_217*RLcp31_118-
 OMcp31_219*RLcp31_163-OMcp31_26*RLcp31_114;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp31_163;
    sens->P[2] = POcp31_263;
    sens->P[3] = POcp31_363;
    sens->R[1][1] = ROcp31_119;
    sens->R[1][2] = ROcp31_219;
    sens->R[1][3] = ROcp31_319;
    sens->R[2][1] = ROcp31_418;
    sens->R[2][2] = ROcp31_518;
    sens->R[2][3] = ROcp31_618;
    sens->R[3][1] = ROcp31_719;
    sens->R[3][2] = ROcp31_819;
    sens->R[3][3] = ROcp31_919;
    sens->V[1] = VIcp31_163;
    sens->V[2] = VIcp31_263;
    sens->V[3] = VIcp31_363;
    sens->OM[1] = OMcp31_119;
    sens->OM[2] = OMcp31_219;
    sens->OM[3] = OMcp31_319;
 
// 
break;
case 33:
 


// = = Block_1_0_0_33_0_1 = = 
 
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
    OMcp32_26 = OMcp32_25+ROcp32_85*qd[6];
    OMcp32_36 = OMcp32_35+ROcp32_95*qd[6];
    OPcp32_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp32_26 = ROcp32_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp32_35*S5-ROcp32_95*qd[4]);
    OPcp32_36 = ROcp32_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp32_25*S5-ROcp32_85*qd[4]);

// = = Block_1_0_0_33_0_3 = = 
 
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
    OMcp32_114 = OMcp32_16+ROcp32_46*qd[14];
    OMcp32_214 = OMcp32_26+ROcp32_56*qd[14];
    OMcp32_314 = OMcp32_36+ROcp32_66*qd[14];
    ORcp32_114 = OMcp32_26*RLcp32_314-OMcp32_36*RLcp32_214;
    ORcp32_214 = -(OMcp32_16*RLcp32_314-OMcp32_36*RLcp32_114);
    ORcp32_314 = OMcp32_16*RLcp32_214-OMcp32_26*RLcp32_114;
    OPcp32_114 = OPcp32_16+ROcp32_46*qdd[14]+qd[14]*(OMcp32_26*ROcp32_66-OMcp32_36*ROcp32_56);
    OPcp32_214 = OPcp32_26+ROcp32_56*qdd[14]-qd[14]*(OMcp32_16*ROcp32_66-OMcp32_36*ROcp32_46);
    OPcp32_314 = OPcp32_36+ROcp32_66*qdd[14]+qd[14]*(OMcp32_16*ROcp32_56-OMcp32_26*ROcp32_46);
    RLcp32_115 = ROcp32_46*s->dpt[2][22];
    RLcp32_215 = ROcp32_56*s->dpt[2][22];
    RLcp32_315 = ROcp32_66*s->dpt[2][22];
    OMcp32_115 = OMcp32_114+ROcp32_114*qd[15];
    OMcp32_215 = OMcp32_214+ROcp32_214*qd[15];
    OMcp32_315 = OMcp32_314+ROcp32_314*qd[15];
    ORcp32_115 = OMcp32_214*RLcp32_315-OMcp32_314*RLcp32_215;
    ORcp32_215 = -(OMcp32_114*RLcp32_315-OMcp32_314*RLcp32_115);
    ORcp32_315 = OMcp32_114*RLcp32_215-OMcp32_214*RLcp32_115;
    OPcp32_115 = OPcp32_114+ROcp32_114*qdd[15]+qd[15]*(OMcp32_214*ROcp32_314-OMcp32_314*ROcp32_214);
    OPcp32_215 = OPcp32_214+ROcp32_214*qdd[15]-qd[15]*(OMcp32_114*ROcp32_314-OMcp32_314*ROcp32_114);
    OPcp32_315 = OPcp32_314+ROcp32_314*qdd[15]+qd[15]*(OMcp32_114*ROcp32_214-OMcp32_214*ROcp32_114);
    RLcp32_116 = ROcp32_715*s->dpt[3][24];
    RLcp32_216 = ROcp32_815*s->dpt[3][24];
    RLcp32_316 = ROcp32_915*s->dpt[3][24];
    OMcp32_116 = OMcp32_115+ROcp32_715*qd[16];
    OMcp32_216 = OMcp32_215+ROcp32_815*qd[16];
    OMcp32_316 = OMcp32_315+ROcp32_915*qd[16];
    ORcp32_116 = OMcp32_215*RLcp32_316-OMcp32_315*RLcp32_216;
    ORcp32_216 = -(OMcp32_115*RLcp32_316-OMcp32_315*RLcp32_116);
    ORcp32_316 = OMcp32_115*RLcp32_216-OMcp32_215*RLcp32_116;
    OPcp32_116 = OPcp32_115+ROcp32_715*qdd[16]+qd[16]*(OMcp32_215*ROcp32_915-OMcp32_315*ROcp32_815);
    OPcp32_216 = OPcp32_215+ROcp32_815*qdd[16]-qd[16]*(OMcp32_115*ROcp32_915-OMcp32_315*ROcp32_715);
    OPcp32_316 = OPcp32_315+ROcp32_915*qdd[16]+qd[16]*(OMcp32_115*ROcp32_815-OMcp32_215*ROcp32_715);
    RLcp32_117 = ROcp32_715*s->dpt[3][27];
    RLcp32_217 = ROcp32_815*s->dpt[3][27];
    RLcp32_317 = ROcp32_915*s->dpt[3][27];
    OMcp32_117 = OMcp32_116+ROcp32_416*qd[17];
    OMcp32_217 = OMcp32_216+ROcp32_516*qd[17];
    OMcp32_317 = OMcp32_316+ROcp32_616*qd[17];
    ORcp32_117 = OMcp32_216*RLcp32_317-OMcp32_316*RLcp32_217;
    ORcp32_217 = -(OMcp32_116*RLcp32_317-OMcp32_316*RLcp32_117);
    ORcp32_317 = OMcp32_116*RLcp32_217-OMcp32_216*RLcp32_117;
    OPcp32_117 = OPcp32_116+ROcp32_416*qdd[17]+qd[17]*(OMcp32_216*ROcp32_616-OMcp32_316*ROcp32_516);
    OPcp32_217 = OPcp32_216+ROcp32_516*qdd[17]-qd[17]*(OMcp32_116*ROcp32_616-OMcp32_316*ROcp32_416);
    OPcp32_317 = OPcp32_316+ROcp32_616*qdd[17]+qd[17]*(OMcp32_116*ROcp32_516-OMcp32_216*ROcp32_416);
    RLcp32_118 = ROcp32_717*s->dpt[3][30];
    RLcp32_218 = ROcp32_817*s->dpt[3][30];
    RLcp32_318 = ROcp32_917*s->dpt[3][30];
    OMcp32_118 = OMcp32_117+ROcp32_117*qd[18];
    OMcp32_218 = OMcp32_217+ROcp32_217*qd[18];
    OMcp32_318 = OMcp32_317+ROcp32_317*qd[18];
    ORcp32_118 = OMcp32_217*RLcp32_318-OMcp32_317*RLcp32_218;
    ORcp32_218 = -(OMcp32_117*RLcp32_318-OMcp32_317*RLcp32_118);
    ORcp32_318 = OMcp32_117*RLcp32_218-OMcp32_217*RLcp32_118;
    OMcp32_119 = OMcp32_118+ROcp32_418*qd[19];
    OMcp32_219 = OMcp32_218+ROcp32_518*qd[19];
    OMcp32_319 = OMcp32_318+ROcp32_618*qd[19];
    OPcp32_119 = OPcp32_117+ROcp32_117*qdd[18]+ROcp32_418*qdd[19]+qd[18]*(OMcp32_217*ROcp32_317-OMcp32_317*ROcp32_217)+
 qd[19]*(OMcp32_218*ROcp32_618-OMcp32_318*ROcp32_518);
    OPcp32_219 = OPcp32_217+ROcp32_217*qdd[18]+ROcp32_518*qdd[19]-qd[18]*(OMcp32_117*ROcp32_317-OMcp32_317*ROcp32_117)-
 qd[19]*(OMcp32_118*ROcp32_618-OMcp32_318*ROcp32_418);
    OPcp32_319 = OPcp32_317+ROcp32_317*qdd[18]+ROcp32_618*qdd[19]+qd[18]*(OMcp32_117*ROcp32_217-OMcp32_217*ROcp32_117)+
 qd[19]*(OMcp32_118*ROcp32_518-OMcp32_218*ROcp32_418);
    RLcp32_120 = ROcp32_119*s->dpt[1][36]+ROcp32_719*s->dpt[3][36];
    RLcp32_220 = ROcp32_219*s->dpt[1][36]+ROcp32_819*s->dpt[3][36];
    RLcp32_320 = ROcp32_319*s->dpt[1][36]+ROcp32_919*s->dpt[3][36];
    POcp32_120 = RLcp32_114+RLcp32_115+RLcp32_116+RLcp32_117+RLcp32_118+RLcp32_120+q[1];
    POcp32_220 = RLcp32_214+RLcp32_215+RLcp32_216+RLcp32_217+RLcp32_218+RLcp32_220+q[2];
    POcp32_320 = RLcp32_314+RLcp32_315+RLcp32_316+RLcp32_317+RLcp32_318+RLcp32_320+q[3];
    OMcp32_120 = OMcp32_119+ROcp32_418*qd[20];
    OMcp32_220 = OMcp32_219+ROcp32_518*qd[20];
    OMcp32_320 = OMcp32_319+ROcp32_618*qd[20];
    ORcp32_120 = OMcp32_219*RLcp32_320-OMcp32_319*RLcp32_220;
    ORcp32_220 = -(OMcp32_119*RLcp32_320-OMcp32_319*RLcp32_120);
    ORcp32_320 = OMcp32_119*RLcp32_220-OMcp32_219*RLcp32_120;
    VIcp32_120 = ORcp32_114+ORcp32_115+ORcp32_116+ORcp32_117+ORcp32_118+ORcp32_120+qd[1];
    VIcp32_220 = ORcp32_214+ORcp32_215+ORcp32_216+ORcp32_217+ORcp32_218+ORcp32_220+qd[2];
    VIcp32_320 = ORcp32_314+ORcp32_315+ORcp32_316+ORcp32_317+ORcp32_318+ORcp32_320+qd[3];
    OPcp32_120 = OPcp32_119+ROcp32_418*qdd[20]+qd[20]*(OMcp32_219*ROcp32_618-OMcp32_319*ROcp32_518);
    OPcp32_220 = OPcp32_219+ROcp32_518*qdd[20]-qd[20]*(OMcp32_119*ROcp32_618-OMcp32_319*ROcp32_418);
    OPcp32_320 = OPcp32_319+ROcp32_618*qdd[20]+qd[20]*(OMcp32_119*ROcp32_518-OMcp32_219*ROcp32_418);
    ACcp32_120 = qdd[1]+OMcp32_214*ORcp32_315+OMcp32_215*ORcp32_316+OMcp32_216*ORcp32_317+OMcp32_217*ORcp32_318+OMcp32_219
 *ORcp32_320+OMcp32_26*ORcp32_314-OMcp32_314*ORcp32_215-OMcp32_315*ORcp32_216-OMcp32_316*ORcp32_217-OMcp32_317*ORcp32_218-
 OMcp32_319*ORcp32_220-OMcp32_36*ORcp32_214+OPcp32_214*RLcp32_315+OPcp32_215*RLcp32_316+OPcp32_216*RLcp32_317+OPcp32_217*
 RLcp32_318+OPcp32_219*RLcp32_320+OPcp32_26*RLcp32_314-OPcp32_314*RLcp32_215-OPcp32_315*RLcp32_216-OPcp32_316*RLcp32_217-
 OPcp32_317*RLcp32_218-OPcp32_319*RLcp32_220-OPcp32_36*RLcp32_214;
    ACcp32_220 = qdd[2]-OMcp32_114*ORcp32_315-OMcp32_115*ORcp32_316-OMcp32_116*ORcp32_317-OMcp32_117*ORcp32_318-OMcp32_119
 *ORcp32_320-OMcp32_16*ORcp32_314+OMcp32_314*ORcp32_115+OMcp32_315*ORcp32_116+OMcp32_316*ORcp32_117+OMcp32_317*ORcp32_118+
 OMcp32_319*ORcp32_120+OMcp32_36*ORcp32_114-OPcp32_114*RLcp32_315-OPcp32_115*RLcp32_316-OPcp32_116*RLcp32_317-OPcp32_117*
 RLcp32_318-OPcp32_119*RLcp32_320-OPcp32_16*RLcp32_314+OPcp32_314*RLcp32_115+OPcp32_315*RLcp32_116+OPcp32_316*RLcp32_117+
 OPcp32_317*RLcp32_118+OPcp32_319*RLcp32_120+OPcp32_36*RLcp32_114;
    ACcp32_320 = qdd[3]+OMcp32_114*ORcp32_215+OMcp32_115*ORcp32_216+OMcp32_116*ORcp32_217+OMcp32_117*ORcp32_218+OMcp32_119
 *ORcp32_220+OMcp32_16*ORcp32_214-OMcp32_214*ORcp32_115-OMcp32_215*ORcp32_116-OMcp32_216*ORcp32_117-OMcp32_217*ORcp32_118-
 OMcp32_219*ORcp32_120-OMcp32_26*ORcp32_114+OPcp32_114*RLcp32_215+OPcp32_115*RLcp32_216+OPcp32_116*RLcp32_217+OPcp32_117*
 RLcp32_218+OPcp32_119*RLcp32_220+OPcp32_16*RLcp32_214-OPcp32_214*RLcp32_115-OPcp32_215*RLcp32_116-OPcp32_216*RLcp32_117-
 OPcp32_217*RLcp32_118-OPcp32_219*RLcp32_120-OPcp32_26*RLcp32_114;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp32_120;
    sens->P[2] = POcp32_220;
    sens->P[3] = POcp32_320;
    sens->R[1][1] = ROcp32_120;
    sens->R[1][2] = ROcp32_220;
    sens->R[1][3] = ROcp32_320;
    sens->R[2][1] = ROcp32_418;
    sens->R[2][2] = ROcp32_518;
    sens->R[2][3] = ROcp32_618;
    sens->R[3][1] = ROcp32_720;
    sens->R[3][2] = ROcp32_820;
    sens->R[3][3] = ROcp32_920;
    sens->V[1] = VIcp32_120;
    sens->V[2] = VIcp32_220;
    sens->V[3] = VIcp32_320;
    sens->OM[1] = OMcp32_120;
    sens->OM[2] = OMcp32_220;
    sens->OM[3] = OMcp32_320;
    sens->A[1] = ACcp32_120;
    sens->A[2] = ACcp32_220;
    sens->A[3] = ACcp32_320;
    sens->OMP[1] = OPcp32_120;
    sens->OMP[2] = OPcp32_220;
    sens->OMP[3] = OPcp32_320;
 
// 
break;
case 34:
 


// = = Block_1_0_0_34_0_1 = = 
 
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
    OMcp33_16 = qd[4]+qd[6]*S5;
    OMcp33_26 = ROcp33_85*qd[6]+qd[5]*C4;
    OMcp33_36 = ROcp33_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_34_0_4 = = 
 
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
    OMcp33_122 = OMcp33_16+ROcp33_16*qd[21]+ROcp33_421*qd[22];
    OMcp33_222 = OMcp33_26+ROcp33_26*qd[21]+ROcp33_521*qd[22];
    OMcp33_322 = OMcp33_36+ROcp33_36*qd[21]+ROcp33_621*qd[22];
    RLcp33_123 = ROcp33_722*s->dpt[3][40];
    RLcp33_223 = ROcp33_822*s->dpt[3][40];
    RLcp33_323 = ROcp33_922*s->dpt[3][40];
    POcp33_123 = RLcp33_121+RLcp33_123+q[1];
    POcp33_223 = RLcp33_221+RLcp33_223+q[2];
    POcp33_323 = RLcp33_321+RLcp33_323+q[3];
    OMcp33_123 = OMcp33_122+ROcp33_722*qd[23];
    OMcp33_223 = OMcp33_222+ROcp33_822*qd[23];
    OMcp33_323 = OMcp33_322+ROcp33_922*qd[23];
    VIcp33_123 = qd[1]+OMcp33_222*RLcp33_323+OMcp33_26*RLcp33_321-OMcp33_322*RLcp33_223-OMcp33_36*RLcp33_221;
    VIcp33_223 = qd[2]-OMcp33_122*RLcp33_323-OMcp33_16*RLcp33_321+OMcp33_322*RLcp33_123+OMcp33_36*RLcp33_121;
    VIcp33_323 = qd[3]+OMcp33_122*RLcp33_223+OMcp33_16*RLcp33_221-OMcp33_222*RLcp33_123-OMcp33_26*RLcp33_121;

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp33_123;
    sens->P[2] = POcp33_223;
    sens->P[3] = POcp33_323;
    sens->R[1][1] = ROcp33_123;
    sens->R[1][2] = ROcp33_223;
    sens->R[1][3] = ROcp33_323;
    sens->R[2][1] = ROcp33_423;
    sens->R[2][2] = ROcp33_523;
    sens->R[2][3] = ROcp33_623;
    sens->R[3][1] = ROcp33_722;
    sens->R[3][2] = ROcp33_822;
    sens->R[3][3] = ROcp33_922;
    sens->V[1] = VIcp33_123;
    sens->V[2] = VIcp33_223;
    sens->V[3] = VIcp33_323;
    sens->OM[1] = OMcp33_123;
    sens->OM[2] = OMcp33_223;
    sens->OM[3] = OMcp33_323;
 
// 
break;
case 35:
 


// = = Block_1_0_0_35_0_1 = = 
 
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
    OMcp34_16 = qd[4]+qd[6]*S5;
    OMcp34_26 = ROcp34_85*qd[6]+qd[5]*C4;
    OMcp34_36 = ROcp34_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_35_0_4 = = 
 
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
    OMcp34_122 = OMcp34_16+ROcp34_16*qd[21]+ROcp34_421*qd[22];
    OMcp34_222 = OMcp34_26+ROcp34_26*qd[21]+ROcp34_521*qd[22];
    OMcp34_322 = OMcp34_36+ROcp34_36*qd[21]+ROcp34_621*qd[22];
    RLcp34_123 = ROcp34_722*s->dpt[3][40];
    RLcp34_223 = ROcp34_822*s->dpt[3][40];
    RLcp34_323 = ROcp34_922*s->dpt[3][40];
    OMcp34_123 = OMcp34_122+ROcp34_722*qd[23];
    OMcp34_223 = OMcp34_222+ROcp34_822*qd[23];
    OMcp34_323 = OMcp34_322+ROcp34_922*qd[23];

// = = Block_1_0_0_35_0_5 = = 
 
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
    OMcp34_124 = OMcp34_123+ROcp34_423*qd[24];
    OMcp34_224 = OMcp34_223+ROcp34_523*qd[24];
    OMcp34_324 = OMcp34_323+ROcp34_623*qd[24];
    RLcp34_125 = ROcp34_423*s->dpt[2][47];
    RLcp34_225 = ROcp34_523*s->dpt[2][47];
    RLcp34_325 = ROcp34_623*s->dpt[2][47];
    OMcp34_125 = OMcp34_124+ROcp34_124*qd[25];
    OMcp34_225 = OMcp34_224+ROcp34_224*qd[25];
    OMcp34_325 = OMcp34_324+ROcp34_324*qd[25];
    RLcp34_126 = ROcp34_725*s->dpt[3][49];
    RLcp34_226 = ROcp34_825*s->dpt[3][49];
    RLcp34_326 = ROcp34_925*s->dpt[3][49];
    POcp34_126 = RLcp34_121+RLcp34_123+RLcp34_124+RLcp34_125+RLcp34_126+q[1];
    POcp34_226 = RLcp34_221+RLcp34_223+RLcp34_224+RLcp34_225+RLcp34_226+q[2];
    POcp34_326 = RLcp34_321+RLcp34_323+RLcp34_324+RLcp34_325+RLcp34_326+q[3];
    OMcp34_126 = OMcp34_125+ROcp34_725*qd[26];
    OMcp34_226 = OMcp34_225+ROcp34_825*qd[26];
    OMcp34_326 = OMcp34_325+ROcp34_925*qd[26];
    VIcp34_126 = qd[1]+OMcp34_222*RLcp34_323+OMcp34_223*RLcp34_324+OMcp34_224*RLcp34_325+OMcp34_225*RLcp34_326+OMcp34_26*
 RLcp34_321-OMcp34_322*RLcp34_223-OMcp34_323*RLcp34_224-OMcp34_324*RLcp34_225-OMcp34_325*RLcp34_226-OMcp34_36*RLcp34_221;
    VIcp34_226 = qd[2]-OMcp34_122*RLcp34_323-OMcp34_123*RLcp34_324-OMcp34_124*RLcp34_325-OMcp34_125*RLcp34_326-OMcp34_16*
 RLcp34_321+OMcp34_322*RLcp34_123+OMcp34_323*RLcp34_124+OMcp34_324*RLcp34_125+OMcp34_325*RLcp34_126+OMcp34_36*RLcp34_121;
    VIcp34_326 = qd[3]+OMcp34_122*RLcp34_223+OMcp34_123*RLcp34_224+OMcp34_124*RLcp34_225+OMcp34_125*RLcp34_226+OMcp34_16*
 RLcp34_221-OMcp34_222*RLcp34_123-OMcp34_223*RLcp34_124-OMcp34_224*RLcp34_125-OMcp34_225*RLcp34_126-OMcp34_26*RLcp34_121;

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp34_126;
    sens->P[2] = POcp34_226;
    sens->P[3] = POcp34_326;
    sens->R[1][1] = ROcp34_126;
    sens->R[1][2] = ROcp34_226;
    sens->R[1][3] = ROcp34_326;
    sens->R[2][1] = ROcp34_426;
    sens->R[2][2] = ROcp34_526;
    sens->R[2][3] = ROcp34_626;
    sens->R[3][1] = ROcp34_725;
    sens->R[3][2] = ROcp34_825;
    sens->R[3][3] = ROcp34_925;
    sens->V[1] = VIcp34_126;
    sens->V[2] = VIcp34_226;
    sens->V[3] = VIcp34_326;
    sens->OM[1] = OMcp34_126;
    sens->OM[2] = OMcp34_226;
    sens->OM[3] = OMcp34_326;
 
// 
break;
case 36:
 


// = = Block_1_0_0_36_0_1 = = 
 
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
    OMcp35_16 = qd[4]+qd[6]*S5;
    OMcp35_26 = ROcp35_85*qd[6]+qd[5]*C4;
    OMcp35_36 = ROcp35_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_36_0_4 = = 
 
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
    OMcp35_122 = OMcp35_16+ROcp35_16*qd[21]+ROcp35_421*qd[22];
    OMcp35_222 = OMcp35_26+ROcp35_26*qd[21]+ROcp35_521*qd[22];
    OMcp35_322 = OMcp35_36+ROcp35_36*qd[21]+ROcp35_621*qd[22];
    RLcp35_123 = ROcp35_722*s->dpt[3][40];
    RLcp35_223 = ROcp35_822*s->dpt[3][40];
    RLcp35_323 = ROcp35_922*s->dpt[3][40];
    OMcp35_123 = OMcp35_122+ROcp35_722*qd[23];
    OMcp35_223 = OMcp35_222+ROcp35_822*qd[23];
    OMcp35_323 = OMcp35_322+ROcp35_922*qd[23];

// = = Block_1_0_0_36_0_5 = = 
 
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
    OMcp35_124 = OMcp35_123+ROcp35_423*qd[24];
    OMcp35_224 = OMcp35_223+ROcp35_523*qd[24];
    OMcp35_324 = OMcp35_323+ROcp35_623*qd[24];
    RLcp35_125 = ROcp35_423*s->dpt[2][47];
    RLcp35_225 = ROcp35_523*s->dpt[2][47];
    RLcp35_325 = ROcp35_623*s->dpt[2][47];
    OMcp35_125 = OMcp35_124+ROcp35_124*qd[25];
    OMcp35_225 = OMcp35_224+ROcp35_224*qd[25];
    OMcp35_325 = OMcp35_324+ROcp35_324*qd[25];
    RLcp35_126 = ROcp35_725*s->dpt[3][49];
    RLcp35_226 = ROcp35_825*s->dpt[3][49];
    RLcp35_326 = ROcp35_925*s->dpt[3][49];
    OMcp35_126 = OMcp35_125+ROcp35_725*qd[26];
    OMcp35_226 = OMcp35_225+ROcp35_825*qd[26];
    OMcp35_326 = OMcp35_325+ROcp35_925*qd[26];
    RLcp35_127 = ROcp35_725*s->dpt[3][52];
    RLcp35_227 = ROcp35_825*s->dpt[3][52];
    RLcp35_327 = ROcp35_925*s->dpt[3][52];
    POcp35_127 = RLcp35_121+RLcp35_123+RLcp35_124+RLcp35_125+RLcp35_126+RLcp35_127+q[1];
    POcp35_227 = RLcp35_221+RLcp35_223+RLcp35_224+RLcp35_225+RLcp35_226+RLcp35_227+q[2];
    POcp35_327 = RLcp35_321+RLcp35_323+RLcp35_324+RLcp35_325+RLcp35_326+RLcp35_327+q[3];
    OMcp35_127 = OMcp35_126+ROcp35_426*qd[27];
    OMcp35_227 = OMcp35_226+ROcp35_526*qd[27];
    OMcp35_327 = OMcp35_326+ROcp35_626*qd[27];
    VIcp35_127 = qd[1]+OMcp35_222*RLcp35_323+OMcp35_223*RLcp35_324+OMcp35_224*RLcp35_325+OMcp35_225*RLcp35_326+OMcp35_226*
 RLcp35_327+OMcp35_26*RLcp35_321-OMcp35_322*RLcp35_223-OMcp35_323*RLcp35_224-OMcp35_324*RLcp35_225-OMcp35_325*RLcp35_226-
 OMcp35_326*RLcp35_227-OMcp35_36*RLcp35_221;
    VIcp35_227 = qd[2]-OMcp35_122*RLcp35_323-OMcp35_123*RLcp35_324-OMcp35_124*RLcp35_325-OMcp35_125*RLcp35_326-OMcp35_126*
 RLcp35_327-OMcp35_16*RLcp35_321+OMcp35_322*RLcp35_123+OMcp35_323*RLcp35_124+OMcp35_324*RLcp35_125+OMcp35_325*RLcp35_126+
 OMcp35_326*RLcp35_127+OMcp35_36*RLcp35_121;
    VIcp35_327 = qd[3]+OMcp35_122*RLcp35_223+OMcp35_123*RLcp35_224+OMcp35_124*RLcp35_225+OMcp35_125*RLcp35_226+OMcp35_126*
 RLcp35_227+OMcp35_16*RLcp35_221-OMcp35_222*RLcp35_123-OMcp35_223*RLcp35_124-OMcp35_224*RLcp35_125-OMcp35_225*RLcp35_126-
 OMcp35_226*RLcp35_127-OMcp35_26*RLcp35_121;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp35_127;
    sens->P[2] = POcp35_227;
    sens->P[3] = POcp35_327;
    sens->R[1][1] = ROcp35_127;
    sens->R[1][2] = ROcp35_227;
    sens->R[1][3] = ROcp35_327;
    sens->R[2][1] = ROcp35_426;
    sens->R[2][2] = ROcp35_526;
    sens->R[2][3] = ROcp35_626;
    sens->R[3][1] = ROcp35_727;
    sens->R[3][2] = ROcp35_827;
    sens->R[3][3] = ROcp35_927;
    sens->V[1] = VIcp35_127;
    sens->V[2] = VIcp35_227;
    sens->V[3] = VIcp35_327;
    sens->OM[1] = OMcp35_127;
    sens->OM[2] = OMcp35_227;
    sens->OM[3] = OMcp35_327;
 
// 
break;
case 37:
 


// = = Block_1_0_0_37_0_1 = = 
 
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
    OMcp36_16 = qd[4]+qd[6]*S5;
    OMcp36_26 = ROcp36_85*qd[6]+qd[5]*C4;
    OMcp36_36 = ROcp36_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_37_0_4 = = 
 
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
    OMcp36_122 = OMcp36_16+ROcp36_16*qd[21]+ROcp36_421*qd[22];
    OMcp36_222 = OMcp36_26+ROcp36_26*qd[21]+ROcp36_521*qd[22];
    OMcp36_322 = OMcp36_36+ROcp36_36*qd[21]+ROcp36_621*qd[22];
    RLcp36_123 = ROcp36_722*s->dpt[3][40];
    RLcp36_223 = ROcp36_822*s->dpt[3][40];
    RLcp36_323 = ROcp36_922*s->dpt[3][40];
    OMcp36_123 = OMcp36_122+ROcp36_722*qd[23];
    OMcp36_223 = OMcp36_222+ROcp36_822*qd[23];
    OMcp36_323 = OMcp36_322+ROcp36_922*qd[23];

// = = Block_1_0_0_37_0_6 = = 
 
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
    OMcp36_128 = OMcp36_123+ROcp36_423*qd[28];
    OMcp36_228 = OMcp36_223+ROcp36_523*qd[28];
    OMcp36_328 = OMcp36_323+ROcp36_623*qd[28];
    RLcp36_129 = ROcp36_423*s->dpt[2][56];
    RLcp36_229 = ROcp36_523*s->dpt[2][56];
    RLcp36_329 = ROcp36_623*s->dpt[2][56];
    OMcp36_129 = OMcp36_128+ROcp36_128*qd[29];
    OMcp36_229 = OMcp36_228+ROcp36_228*qd[29];
    OMcp36_329 = OMcp36_328+ROcp36_328*qd[29];
    RLcp36_130 = ROcp36_729*s->dpt[3][58];
    RLcp36_230 = ROcp36_829*s->dpt[3][58];
    RLcp36_330 = ROcp36_929*s->dpt[3][58];
    POcp36_130 = RLcp36_121+RLcp36_123+RLcp36_128+RLcp36_129+RLcp36_130+q[1];
    POcp36_230 = RLcp36_221+RLcp36_223+RLcp36_228+RLcp36_229+RLcp36_230+q[2];
    POcp36_330 = RLcp36_321+RLcp36_323+RLcp36_328+RLcp36_329+RLcp36_330+q[3];
    OMcp36_130 = OMcp36_129+ROcp36_729*qd[30];
    OMcp36_230 = OMcp36_229+ROcp36_829*qd[30];
    OMcp36_330 = OMcp36_329+ROcp36_929*qd[30];
    VIcp36_130 = qd[1]+OMcp36_222*RLcp36_323+OMcp36_223*RLcp36_328+OMcp36_228*RLcp36_329+OMcp36_229*RLcp36_330+OMcp36_26*
 RLcp36_321-OMcp36_322*RLcp36_223-OMcp36_323*RLcp36_228-OMcp36_328*RLcp36_229-OMcp36_329*RLcp36_230-OMcp36_36*RLcp36_221;
    VIcp36_230 = qd[2]-OMcp36_122*RLcp36_323-OMcp36_123*RLcp36_328-OMcp36_128*RLcp36_329-OMcp36_129*RLcp36_330-OMcp36_16*
 RLcp36_321+OMcp36_322*RLcp36_123+OMcp36_323*RLcp36_128+OMcp36_328*RLcp36_129+OMcp36_329*RLcp36_130+OMcp36_36*RLcp36_121;
    VIcp36_330 = qd[3]+OMcp36_122*RLcp36_223+OMcp36_123*RLcp36_228+OMcp36_128*RLcp36_229+OMcp36_129*RLcp36_230+OMcp36_16*
 RLcp36_221-OMcp36_222*RLcp36_123-OMcp36_223*RLcp36_128-OMcp36_228*RLcp36_129-OMcp36_229*RLcp36_130-OMcp36_26*RLcp36_121;

// = = Block_1_0_0_37_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp36_130;
    sens->P[2] = POcp36_230;
    sens->P[3] = POcp36_330;
    sens->R[1][1] = ROcp36_130;
    sens->R[1][2] = ROcp36_230;
    sens->R[1][3] = ROcp36_330;
    sens->R[2][1] = ROcp36_430;
    sens->R[2][2] = ROcp36_530;
    sens->R[2][3] = ROcp36_630;
    sens->R[3][1] = ROcp36_729;
    sens->R[3][2] = ROcp36_829;
    sens->R[3][3] = ROcp36_929;
    sens->V[1] = VIcp36_130;
    sens->V[2] = VIcp36_230;
    sens->V[3] = VIcp36_330;
    sens->OM[1] = OMcp36_130;
    sens->OM[2] = OMcp36_230;
    sens->OM[3] = OMcp36_330;
 
// 
break;
case 38:
 


// = = Block_1_0_0_38_0_1 = = 
 
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
    OMcp37_16 = qd[4]+qd[6]*S5;
    OMcp37_26 = ROcp37_85*qd[6]+qd[5]*C4;
    OMcp37_36 = ROcp37_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_38_0_4 = = 
 
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
    OMcp37_122 = OMcp37_16+ROcp37_16*qd[21]+ROcp37_421*qd[22];
    OMcp37_222 = OMcp37_26+ROcp37_26*qd[21]+ROcp37_521*qd[22];
    OMcp37_322 = OMcp37_36+ROcp37_36*qd[21]+ROcp37_621*qd[22];
    RLcp37_123 = ROcp37_722*s->dpt[3][40];
    RLcp37_223 = ROcp37_822*s->dpt[3][40];
    RLcp37_323 = ROcp37_922*s->dpt[3][40];
    OMcp37_123 = OMcp37_122+ROcp37_722*qd[23];
    OMcp37_223 = OMcp37_222+ROcp37_822*qd[23];
    OMcp37_323 = OMcp37_322+ROcp37_922*qd[23];

// = = Block_1_0_0_38_0_6 = = 
 
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
    OMcp37_128 = OMcp37_123+ROcp37_423*qd[28];
    OMcp37_228 = OMcp37_223+ROcp37_523*qd[28];
    OMcp37_328 = OMcp37_323+ROcp37_623*qd[28];
    RLcp37_129 = ROcp37_423*s->dpt[2][56];
    RLcp37_229 = ROcp37_523*s->dpt[2][56];
    RLcp37_329 = ROcp37_623*s->dpt[2][56];
    OMcp37_129 = OMcp37_128+ROcp37_128*qd[29];
    OMcp37_229 = OMcp37_228+ROcp37_228*qd[29];
    OMcp37_329 = OMcp37_328+ROcp37_328*qd[29];
    RLcp37_130 = ROcp37_729*s->dpt[3][58];
    RLcp37_230 = ROcp37_829*s->dpt[3][58];
    RLcp37_330 = ROcp37_929*s->dpt[3][58];
    OMcp37_130 = OMcp37_129+ROcp37_729*qd[30];
    OMcp37_230 = OMcp37_229+ROcp37_829*qd[30];
    OMcp37_330 = OMcp37_329+ROcp37_929*qd[30];
    RLcp37_131 = ROcp37_729*s->dpt[3][61];
    RLcp37_231 = ROcp37_829*s->dpt[3][61];
    RLcp37_331 = ROcp37_929*s->dpt[3][61];
    POcp37_131 = RLcp37_121+RLcp37_123+RLcp37_128+RLcp37_129+RLcp37_130+RLcp37_131+q[1];
    POcp37_231 = RLcp37_221+RLcp37_223+RLcp37_228+RLcp37_229+RLcp37_230+RLcp37_231+q[2];
    POcp37_331 = RLcp37_321+RLcp37_323+RLcp37_328+RLcp37_329+RLcp37_330+RLcp37_331+q[3];
    OMcp37_131 = OMcp37_130+ROcp37_430*qd[31];
    OMcp37_231 = OMcp37_230+ROcp37_530*qd[31];
    OMcp37_331 = OMcp37_330+ROcp37_630*qd[31];
    VIcp37_131 = qd[1]+OMcp37_222*RLcp37_323+OMcp37_223*RLcp37_328+OMcp37_228*RLcp37_329+OMcp37_229*RLcp37_330+OMcp37_230*
 RLcp37_331+OMcp37_26*RLcp37_321-OMcp37_322*RLcp37_223-OMcp37_323*RLcp37_228-OMcp37_328*RLcp37_229-OMcp37_329*RLcp37_230-
 OMcp37_330*RLcp37_231-OMcp37_36*RLcp37_221;
    VIcp37_231 = qd[2]-OMcp37_122*RLcp37_323-OMcp37_123*RLcp37_328-OMcp37_128*RLcp37_329-OMcp37_129*RLcp37_330-OMcp37_130*
 RLcp37_331-OMcp37_16*RLcp37_321+OMcp37_322*RLcp37_123+OMcp37_323*RLcp37_128+OMcp37_328*RLcp37_129+OMcp37_329*RLcp37_130+
 OMcp37_330*RLcp37_131+OMcp37_36*RLcp37_121;
    VIcp37_331 = qd[3]+OMcp37_122*RLcp37_223+OMcp37_123*RLcp37_228+OMcp37_128*RLcp37_229+OMcp37_129*RLcp37_230+OMcp37_130*
 RLcp37_231+OMcp37_16*RLcp37_221-OMcp37_222*RLcp37_123-OMcp37_223*RLcp37_128-OMcp37_228*RLcp37_129-OMcp37_229*RLcp37_130-
 OMcp37_230*RLcp37_131-OMcp37_26*RLcp37_121;

// = = Block_1_0_0_38_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp37_131;
    sens->P[2] = POcp37_231;
    sens->P[3] = POcp37_331;
    sens->R[1][1] = ROcp37_131;
    sens->R[1][2] = ROcp37_231;
    sens->R[1][3] = ROcp37_331;
    sens->R[2][1] = ROcp37_430;
    sens->R[2][2] = ROcp37_530;
    sens->R[2][3] = ROcp37_630;
    sens->R[3][1] = ROcp37_731;
    sens->R[3][2] = ROcp37_831;
    sens->R[3][3] = ROcp37_931;
    sens->V[1] = VIcp37_131;
    sens->V[2] = VIcp37_231;
    sens->V[3] = VIcp37_331;
    sens->OM[1] = OMcp37_131;
    sens->OM[2] = OMcp37_231;
    sens->OM[3] = OMcp37_331;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

