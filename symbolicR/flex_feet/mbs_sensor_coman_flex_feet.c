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
//	==> Generation Date : Thu Nov 26 12:45:13 2015
//
//	==> Project name : coman_flex_feet
//	==> using XML input file 
//
//	==> Number of joints : 33
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 9380
//
//	==> Generation Time :  0.340 seconds
//	==> Post-Processing :  0.220 seconds
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
 
#include "mbs_sensor_coman_flex_feet.h" 
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
    RLcp0_142 = s->dpt[3][5]*S5+C5*(s->dpt[1][5]*C6-s->dpt[2][5]*S6);
    RLcp0_242 = ROcp0_85*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_25*C6+C4*S6)-s->dpt[2][5]*(ROcp0_25*S6-C4*C6);
    RLcp0_342 = ROcp0_95*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_35*C6+S4*S6)-s->dpt[2][5]*(ROcp0_35*S6-S4*C6);
    POcp0_142 = RLcp0_142+q[1];
    POcp0_242 = RLcp0_242+q[2];
    POcp0_342 = RLcp0_342+q[3];
    VIcp0_142 = qd[1]+OMcp0_26*RLcp0_342-OMcp0_36*RLcp0_242;
    VIcp0_242 = qd[2]-OMcp0_16*RLcp0_342+OMcp0_36*RLcp0_142;
    VIcp0_342 = qd[3]+OMcp0_16*RLcp0_242-OMcp0_26*RLcp0_142;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_142;
    sens->P[2] = POcp0_242;
    sens->P[3] = POcp0_342;
    sens->V[1] = VIcp0_142;
    sens->V[2] = VIcp0_242;
    sens->V[3] = VIcp0_342;
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
    RLcp1_143 = ROcp1_46*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_16*C7-S5*S7)+s->dpt[3][7]*(ROcp1_16*S7+S5*C7);
    RLcp1_243 = ROcp1_56*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_26*C7-ROcp1_85*S7)+s->dpt[3][7]*(ROcp1_26*S7+ROcp1_85*C7);
    RLcp1_343 = ROcp1_66*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_36*C7-ROcp1_95*S7)+s->dpt[3][7]*(ROcp1_36*S7+ROcp1_95*C7);
    POcp1_143 = RLcp1_143+RLcp1_17+q[1];
    POcp1_243 = RLcp1_243+RLcp1_27+q[2];
    POcp1_343 = RLcp1_343+RLcp1_37+q[3];
    VIcp1_143 = qd[1]+OMcp1_26*RLcp1_37+OMcp1_27*RLcp1_343-OMcp1_36*RLcp1_27-OMcp1_37*RLcp1_243;
    VIcp1_243 = qd[2]-OMcp1_16*RLcp1_37-OMcp1_17*RLcp1_343+OMcp1_36*RLcp1_17+OMcp1_37*RLcp1_143;
    VIcp1_343 = qd[3]+OMcp1_16*RLcp1_27+OMcp1_17*RLcp1_243-OMcp1_26*RLcp1_17-OMcp1_27*RLcp1_143;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_143;
    sens->P[2] = POcp1_243;
    sens->P[3] = POcp1_343;
    sens->V[1] = VIcp1_143;
    sens->V[2] = VIcp1_243;
    sens->V[3] = VIcp1_343;
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
    RLcp2_144 = ROcp2_17*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_46*C8+ROcp2_77*S8)-s->dpt[3][9]*(ROcp2_46*S8-ROcp2_77*C8);
    RLcp2_244 = ROcp2_27*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_56*C8+ROcp2_87*S8)-s->dpt[3][9]*(ROcp2_56*S8-ROcp2_87*C8);
    RLcp2_344 = ROcp2_37*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_66*C8+ROcp2_97*S8)-s->dpt[3][9]*(ROcp2_66*S8-ROcp2_97*C8);
    POcp2_144 = RLcp2_144+RLcp2_17+RLcp2_18+q[1];
    POcp2_244 = RLcp2_244+RLcp2_27+RLcp2_28+q[2];
    POcp2_344 = RLcp2_344+RLcp2_37+RLcp2_38+q[3];
    VIcp2_144 = qd[1]+OMcp2_26*RLcp2_37+OMcp2_27*RLcp2_38+OMcp2_28*RLcp2_344-OMcp2_36*RLcp2_27-OMcp2_37*RLcp2_28-OMcp2_38*
 RLcp2_244;
    VIcp2_244 = qd[2]-OMcp2_16*RLcp2_37-OMcp2_17*RLcp2_38-OMcp2_18*RLcp2_344+OMcp2_36*RLcp2_17+OMcp2_37*RLcp2_18+OMcp2_38*
 RLcp2_144;
    VIcp2_344 = qd[3]+OMcp2_16*RLcp2_27+OMcp2_17*RLcp2_28+OMcp2_18*RLcp2_244-OMcp2_26*RLcp2_17-OMcp2_27*RLcp2_18-OMcp2_28*
 RLcp2_144;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_144;
    sens->P[2] = POcp2_244;
    sens->P[3] = POcp2_344;
    sens->V[1] = VIcp2_144;
    sens->V[2] = VIcp2_244;
    sens->V[3] = VIcp2_344;
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
    RLcp3_145 = ROcp3_78*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_17*C9+ROcp3_48*S9)-s->dpt[2][12]*(ROcp3_17*S9-ROcp3_48*C9);
    RLcp3_245 = ROcp3_88*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_27*C9+ROcp3_58*S9)-s->dpt[2][12]*(ROcp3_27*S9-ROcp3_58*C9);
    RLcp3_345 = ROcp3_98*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_37*C9+ROcp3_68*S9)-s->dpt[2][12]*(ROcp3_37*S9-ROcp3_68*C9);
    POcp3_145 = RLcp3_145+RLcp3_17+RLcp3_18+RLcp3_19+q[1];
    POcp3_245 = RLcp3_245+RLcp3_27+RLcp3_28+RLcp3_29+q[2];
    POcp3_345 = RLcp3_345+RLcp3_37+RLcp3_38+RLcp3_39+q[3];
    VIcp3_145 = qd[1]+OMcp3_26*RLcp3_37+OMcp3_27*RLcp3_38+OMcp3_28*RLcp3_39+OMcp3_29*RLcp3_345-OMcp3_36*RLcp3_27-OMcp3_37*
 RLcp3_28-OMcp3_38*RLcp3_29-OMcp3_39*RLcp3_245;
    VIcp3_245 = qd[2]-OMcp3_16*RLcp3_37-OMcp3_17*RLcp3_38-OMcp3_18*RLcp3_39-OMcp3_19*RLcp3_345+OMcp3_36*RLcp3_17+OMcp3_37*
 RLcp3_18+OMcp3_38*RLcp3_19+OMcp3_39*RLcp3_145;
    VIcp3_345 = qd[3]+OMcp3_16*RLcp3_27+OMcp3_17*RLcp3_28+OMcp3_18*RLcp3_29+OMcp3_19*RLcp3_245-OMcp3_26*RLcp3_17-OMcp3_27*
 RLcp3_18-OMcp3_28*RLcp3_19-OMcp3_29*RLcp3_145;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_145;
    sens->P[2] = POcp3_245;
    sens->P[3] = POcp3_345;
    sens->V[1] = VIcp3_145;
    sens->V[2] = VIcp3_245;
    sens->V[3] = VIcp3_345;
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
    RLcp4_146 = ROcp4_49*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_19*C10-ROcp4_78*S10)+s->dpt[3][15]*(ROcp4_19*S10+ROcp4_78*C10);
    RLcp4_246 = ROcp4_59*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_29*C10-ROcp4_88*S10)+s->dpt[3][15]*(ROcp4_29*S10+ROcp4_88*C10);
    RLcp4_346 = ROcp4_69*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_39*C10-ROcp4_98*S10)+s->dpt[3][15]*(ROcp4_39*S10+ROcp4_98*C10);
    POcp4_146 = RLcp4_110+RLcp4_146+RLcp4_17+RLcp4_18+RLcp4_19+q[1];
    POcp4_246 = RLcp4_210+RLcp4_246+RLcp4_27+RLcp4_28+RLcp4_29+q[2];
    POcp4_346 = RLcp4_310+RLcp4_346+RLcp4_37+RLcp4_38+RLcp4_39+q[3];
    VIcp4_146 = qd[1]+OMcp4_210*RLcp4_346+OMcp4_26*RLcp4_37+OMcp4_27*RLcp4_38+OMcp4_28*RLcp4_39+OMcp4_29*RLcp4_310-
 OMcp4_310*RLcp4_246-OMcp4_36*RLcp4_27-OMcp4_37*RLcp4_28-OMcp4_38*RLcp4_29-OMcp4_39*RLcp4_210;
    VIcp4_246 = qd[2]-OMcp4_110*RLcp4_346-OMcp4_16*RLcp4_37-OMcp4_17*RLcp4_38-OMcp4_18*RLcp4_39-OMcp4_19*RLcp4_310+
 OMcp4_310*RLcp4_146+OMcp4_36*RLcp4_17+OMcp4_37*RLcp4_18+OMcp4_38*RLcp4_19+OMcp4_39*RLcp4_110;
    VIcp4_346 = qd[3]+OMcp4_110*RLcp4_246+OMcp4_16*RLcp4_27+OMcp4_17*RLcp4_28+OMcp4_18*RLcp4_29+OMcp4_19*RLcp4_210-
 OMcp4_210*RLcp4_146-OMcp4_26*RLcp4_17-OMcp4_27*RLcp4_18-OMcp4_28*RLcp4_19-OMcp4_29*RLcp4_110;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_146;
    sens->P[2] = POcp4_246;
    sens->P[3] = POcp4_346;
    sens->V[1] = VIcp4_146;
    sens->V[2] = VIcp4_246;
    sens->V[3] = VIcp4_346;
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
    RLcp5_147 = ROcp5_110*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_49*C11+ROcp5_710*S11)-s->dpt[3][17]*(ROcp5_49*S11-ROcp5_710*
 C11);
    RLcp5_247 = ROcp5_210*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_59*C11+ROcp5_810*S11)-s->dpt[3][17]*(ROcp5_59*S11-ROcp5_810*
 C11);
    RLcp5_347 = ROcp5_310*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_69*C11+ROcp5_910*S11)-s->dpt[3][17]*(ROcp5_69*S11-ROcp5_910*
 C11);
    POcp5_147 = RLcp5_110+RLcp5_111+RLcp5_147+RLcp5_17+RLcp5_18+RLcp5_19+q[1];
    POcp5_247 = RLcp5_210+RLcp5_211+RLcp5_247+RLcp5_27+RLcp5_28+RLcp5_29+q[2];
    POcp5_347 = RLcp5_310+RLcp5_311+RLcp5_347+RLcp5_37+RLcp5_38+RLcp5_39+q[3];
    VIcp5_147 = qd[1]+OMcp5_210*RLcp5_311+OMcp5_211*RLcp5_347+OMcp5_26*RLcp5_37+OMcp5_27*RLcp5_38+OMcp5_28*RLcp5_39+
 OMcp5_29*RLcp5_310-OMcp5_310*RLcp5_211-OMcp5_311*RLcp5_247-OMcp5_36*RLcp5_27-OMcp5_37*RLcp5_28-OMcp5_38*RLcp5_29-OMcp5_39*
 RLcp5_210;
    VIcp5_247 = qd[2]-OMcp5_110*RLcp5_311-OMcp5_111*RLcp5_347-OMcp5_16*RLcp5_37-OMcp5_17*RLcp5_38-OMcp5_18*RLcp5_39-
 OMcp5_19*RLcp5_310+OMcp5_310*RLcp5_111+OMcp5_311*RLcp5_147+OMcp5_36*RLcp5_17+OMcp5_37*RLcp5_18+OMcp5_38*RLcp5_19+OMcp5_39*
 RLcp5_110;
    VIcp5_347 = qd[3]+OMcp5_110*RLcp5_211+OMcp5_111*RLcp5_247+OMcp5_16*RLcp5_27+OMcp5_17*RLcp5_28+OMcp5_18*RLcp5_29+
 OMcp5_19*RLcp5_210-OMcp5_210*RLcp5_111-OMcp5_211*RLcp5_147-OMcp5_26*RLcp5_17-OMcp5_27*RLcp5_18-OMcp5_28*RLcp5_19-OMcp5_29*
 RLcp5_110;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_147;
    sens->P[2] = POcp5_247;
    sens->P[3] = POcp5_347;
    sens->V[1] = VIcp5_147;
    sens->V[2] = VIcp5_247;
    sens->V[3] = VIcp5_347;
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
    ROcp6_411 = ROcp6_49*C11+ROcp6_710*S11;
    ROcp6_511 = ROcp6_59*C11+ROcp6_810*S11;
    ROcp6_611 = ROcp6_69*C11+ROcp6_910*S11;
    ROcp6_711 = -(ROcp6_49*S11-ROcp6_710*C11);
    ROcp6_811 = -(ROcp6_59*S11-ROcp6_810*C11);
    ROcp6_911 = -(ROcp6_69*S11-ROcp6_910*C11);
    ROcp6_112 = ROcp6_110*C12-ROcp6_711*S12;
    ROcp6_212 = ROcp6_210*C12-ROcp6_811*S12;
    ROcp6_312 = ROcp6_310*C12-ROcp6_911*S12;
    ROcp6_712 = ROcp6_110*S12+ROcp6_711*C12;
    ROcp6_812 = ROcp6_210*S12+ROcp6_811*C12;
    ROcp6_912 = ROcp6_310*S12+ROcp6_911*C12;
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
    POcp6_111 = RLcp6_110+RLcp6_111+RLcp6_17+RLcp6_18+RLcp6_19+q[1];
    POcp6_211 = RLcp6_210+RLcp6_211+RLcp6_27+RLcp6_28+RLcp6_29+q[2];
    POcp6_311 = RLcp6_310+RLcp6_311+RLcp6_37+RLcp6_38+RLcp6_39+q[3];
    VIcp6_111 = qd[1]+OMcp6_210*RLcp6_311+OMcp6_26*RLcp6_37+OMcp6_27*RLcp6_38+OMcp6_28*RLcp6_39+OMcp6_29*RLcp6_310-
 OMcp6_310*RLcp6_211-OMcp6_36*RLcp6_27-OMcp6_37*RLcp6_28-OMcp6_38*RLcp6_29-OMcp6_39*RLcp6_210;
    VIcp6_211 = qd[2]-OMcp6_110*RLcp6_311-OMcp6_16*RLcp6_37-OMcp6_17*RLcp6_38-OMcp6_18*RLcp6_39-OMcp6_19*RLcp6_310+
 OMcp6_310*RLcp6_111+OMcp6_36*RLcp6_17+OMcp6_37*RLcp6_18+OMcp6_38*RLcp6_19+OMcp6_39*RLcp6_110;
    VIcp6_311 = qd[3]+OMcp6_110*RLcp6_211+OMcp6_16*RLcp6_27+OMcp6_17*RLcp6_28+OMcp6_18*RLcp6_29+OMcp6_19*RLcp6_210-
 OMcp6_210*RLcp6_111-OMcp6_26*RLcp6_17-OMcp6_27*RLcp6_18-OMcp6_28*RLcp6_19-OMcp6_29*RLcp6_110;
    OMcp6_112 = OMcp6_110+ROcp6_110*qd[11]+ROcp6_411*qd[12];
    OMcp6_212 = OMcp6_210+ROcp6_210*qd[11]+ROcp6_511*qd[12];
    OMcp6_312 = OMcp6_310+ROcp6_310*qd[11]+ROcp6_611*qd[12];

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_111;
    sens->P[2] = POcp6_211;
    sens->P[3] = POcp6_311;
    sens->R[1][1] = ROcp6_112;
    sens->R[1][2] = ROcp6_212;
    sens->R[1][3] = ROcp6_312;
    sens->R[2][1] = ROcp6_411;
    sens->R[2][2] = ROcp6_511;
    sens->R[2][3] = ROcp6_611;
    sens->R[3][1] = ROcp6_712;
    sens->R[3][2] = ROcp6_812;
    sens->R[3][3] = ROcp6_912;
    sens->V[1] = VIcp6_111;
    sens->V[2] = VIcp6_211;
    sens->V[3] = VIcp6_311;
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

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


    ROcp7_17 = ROcp7_16*C7-S5*S7;
    ROcp7_27 = ROcp7_26*C7-ROcp7_85*S7;
    ROcp7_37 = ROcp7_36*C7-ROcp7_95*S7;
    ROcp7_77 = ROcp7_16*S7+S5*C7;
    ROcp7_87 = ROcp7_26*S7+ROcp7_85*C7;
    ROcp7_97 = ROcp7_36*S7+ROcp7_95*C7;
    ROcp7_48 = ROcp7_46*C8+ROcp7_77*S8;
    ROcp7_58 = ROcp7_56*C8+ROcp7_87*S8;
    ROcp7_68 = ROcp7_66*C8+ROcp7_97*S8;
    ROcp7_78 = -(ROcp7_46*S8-ROcp7_77*C8);
    ROcp7_88 = -(ROcp7_56*S8-ROcp7_87*C8);
    ROcp7_98 = -(ROcp7_66*S8-ROcp7_97*C8);
    ROcp7_19 = ROcp7_17*C9+ROcp7_48*S9;
    ROcp7_29 = ROcp7_27*C9+ROcp7_58*S9;
    ROcp7_39 = ROcp7_37*C9+ROcp7_68*S9;
    ROcp7_49 = -(ROcp7_17*S9-ROcp7_48*C9);
    ROcp7_59 = -(ROcp7_27*S9-ROcp7_58*C9);
    ROcp7_69 = -(ROcp7_37*S9-ROcp7_68*C9);
    ROcp7_110 = ROcp7_19*C10-ROcp7_78*S10;
    ROcp7_210 = ROcp7_29*C10-ROcp7_88*S10;
    ROcp7_310 = ROcp7_39*C10-ROcp7_98*S10;
    ROcp7_710 = ROcp7_19*S10+ROcp7_78*C10;
    ROcp7_810 = ROcp7_29*S10+ROcp7_88*C10;
    ROcp7_910 = ROcp7_39*S10+ROcp7_98*C10;
    ROcp7_711 = -(ROcp7_49*S11-ROcp7_710*C11);
    ROcp7_811 = -(ROcp7_59*S11-ROcp7_810*C11);
    ROcp7_911 = -(ROcp7_69*S11-ROcp7_910*C11);
    RLcp7_17 = ROcp7_46*s->dpt[2][2];
    RLcp7_27 = ROcp7_56*s->dpt[2][2];
    RLcp7_37 = ROcp7_66*s->dpt[2][2];
    OMcp7_17 = OMcp7_16+ROcp7_46*qd[7];
    OMcp7_27 = OMcp7_26+ROcp7_56*qd[7];
    OMcp7_37 = OMcp7_36+ROcp7_66*qd[7];
    RLcp7_18 = ROcp7_46*s->dpt[2][6];
    RLcp7_28 = ROcp7_56*s->dpt[2][6];
    RLcp7_38 = ROcp7_66*s->dpt[2][6];
    OMcp7_18 = OMcp7_17+ROcp7_17*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_27*qd[8];
    OMcp7_38 = OMcp7_37+ROcp7_37*qd[8];
    RLcp7_19 = ROcp7_78*s->dpt[3][8];
    RLcp7_29 = ROcp7_88*s->dpt[3][8];
    RLcp7_39 = ROcp7_98*s->dpt[3][8];
    OMcp7_19 = OMcp7_18+ROcp7_78*qd[9];
    OMcp7_29 = OMcp7_28+ROcp7_88*qd[9];
    OMcp7_39 = OMcp7_38+ROcp7_98*qd[9];
    RLcp7_110 = ROcp7_78*s->dpt[3][11];
    RLcp7_210 = ROcp7_88*s->dpt[3][11];
    RLcp7_310 = ROcp7_98*s->dpt[3][11];
    OMcp7_110 = OMcp7_19+ROcp7_49*qd[10];
    OMcp7_210 = OMcp7_29+ROcp7_59*qd[10];
    OMcp7_310 = OMcp7_39+ROcp7_69*qd[10];
    RLcp7_111 = ROcp7_710*s->dpt[3][14];
    RLcp7_211 = ROcp7_810*s->dpt[3][14];
    RLcp7_311 = ROcp7_910*s->dpt[3][14];
    OMcp7_112 = OMcp7_110+ROcp7_110*qd[11]+qd[12]*(ROcp7_49*C11+ROcp7_710*S11);
    OMcp7_212 = OMcp7_210+ROcp7_210*qd[11]+qd[12]*(ROcp7_59*C11+ROcp7_810*S11);
    OMcp7_312 = OMcp7_310+ROcp7_310*qd[11]+qd[12]*(ROcp7_69*C11+ROcp7_910*S11);
    RLcp7_149 = s->dpt[1][19]*(ROcp7_110*C12-ROcp7_711*S12)+s->dpt[3][19]*(ROcp7_110*S12+ROcp7_711*C12);
    RLcp7_249 = s->dpt[1][19]*(ROcp7_210*C12-ROcp7_811*S12)+s->dpt[3][19]*(ROcp7_210*S12+ROcp7_811*C12);
    RLcp7_349 = s->dpt[1][19]*(ROcp7_310*C12-ROcp7_911*S12)+s->dpt[3][19]*(ROcp7_310*S12+ROcp7_911*C12);
    POcp7_149 = RLcp7_110+RLcp7_111+RLcp7_149+RLcp7_17+RLcp7_18+RLcp7_19+q[1];
    POcp7_249 = RLcp7_210+RLcp7_211+RLcp7_249+RLcp7_27+RLcp7_28+RLcp7_29+q[2];
    POcp7_349 = RLcp7_310+RLcp7_311+RLcp7_349+RLcp7_37+RLcp7_38+RLcp7_39+q[3];
    VIcp7_149 = qd[1]+OMcp7_210*RLcp7_311+OMcp7_212*RLcp7_349+OMcp7_26*RLcp7_37+OMcp7_27*RLcp7_38+OMcp7_28*RLcp7_39+
 OMcp7_29*RLcp7_310-OMcp7_310*RLcp7_211-OMcp7_312*RLcp7_249-OMcp7_36*RLcp7_27-OMcp7_37*RLcp7_28-OMcp7_38*RLcp7_29-OMcp7_39*
 RLcp7_210;
    VIcp7_249 = qd[2]-OMcp7_110*RLcp7_311-OMcp7_112*RLcp7_349-OMcp7_16*RLcp7_37-OMcp7_17*RLcp7_38-OMcp7_18*RLcp7_39-
 OMcp7_19*RLcp7_310+OMcp7_310*RLcp7_111+OMcp7_312*RLcp7_149+OMcp7_36*RLcp7_17+OMcp7_37*RLcp7_18+OMcp7_38*RLcp7_19+OMcp7_39*
 RLcp7_110;
    VIcp7_349 = qd[3]+OMcp7_110*RLcp7_211+OMcp7_112*RLcp7_249+OMcp7_16*RLcp7_27+OMcp7_17*RLcp7_28+OMcp7_18*RLcp7_29+
 OMcp7_19*RLcp7_210-OMcp7_210*RLcp7_111-OMcp7_212*RLcp7_149-OMcp7_26*RLcp7_17-OMcp7_27*RLcp7_18-OMcp7_28*RLcp7_19-OMcp7_29*
 RLcp7_110;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_149;
    sens->P[2] = POcp7_249;
    sens->P[3] = POcp7_349;
    sens->V[1] = VIcp7_149;
    sens->V[2] = VIcp7_249;
    sens->V[3] = VIcp7_349;
    sens->OM[1] = OMcp7_112;
    sens->OM[2] = OMcp7_212;
    sens->OM[3] = OMcp7_312;
 
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

// = = Block_1_0_0_9_0_5 = = 
 
// Sensor Kinematics 


    RLcp8_115 = ROcp8_46*s->dpt[2][3];
    RLcp8_215 = ROcp8_56*s->dpt[2][3];
    RLcp8_315 = ROcp8_66*s->dpt[2][3];
    OMcp8_115 = OMcp8_16+ROcp8_46*qd[15];
    OMcp8_215 = OMcp8_26+ROcp8_56*qd[15];
    OMcp8_315 = OMcp8_36+ROcp8_66*qd[15];
    RLcp8_150 = ROcp8_46*s->dpt[2][25]+s->dpt[1][25]*(ROcp8_16*C15-S15*S5)+s->dpt[3][25]*(ROcp8_16*S15+C15*S5);
    RLcp8_250 = ROcp8_56*s->dpt[2][25]+s->dpt[1][25]*(ROcp8_26*C15-ROcp8_85*S15)+s->dpt[3][25]*(ROcp8_26*S15+ROcp8_85*C15);
    RLcp8_350 = ROcp8_66*s->dpt[2][25]+s->dpt[1][25]*(ROcp8_36*C15-ROcp8_95*S15)+s->dpt[3][25]*(ROcp8_36*S15+ROcp8_95*C15);
    POcp8_150 = RLcp8_115+RLcp8_150+q[1];
    POcp8_250 = RLcp8_215+RLcp8_250+q[2];
    POcp8_350 = RLcp8_315+RLcp8_350+q[3];
    VIcp8_150 = qd[1]+OMcp8_215*RLcp8_350+OMcp8_26*RLcp8_315-OMcp8_315*RLcp8_250-OMcp8_36*RLcp8_215;
    VIcp8_250 = qd[2]-OMcp8_115*RLcp8_350-OMcp8_16*RLcp8_315+OMcp8_315*RLcp8_150+OMcp8_36*RLcp8_115;
    VIcp8_350 = qd[3]+OMcp8_115*RLcp8_250+OMcp8_16*RLcp8_215-OMcp8_215*RLcp8_150-OMcp8_26*RLcp8_115;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_150;
    sens->P[2] = POcp8_250;
    sens->P[3] = POcp8_350;
    sens->V[1] = VIcp8_150;
    sens->V[2] = VIcp8_250;
    sens->V[3] = VIcp8_350;
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

// = = Block_1_0_0_10_0_5 = = 
 
// Sensor Kinematics 


    ROcp9_115 = ROcp9_16*C15-S15*S5;
    ROcp9_215 = ROcp9_26*C15-ROcp9_85*S15;
    ROcp9_315 = ROcp9_36*C15-ROcp9_95*S15;
    ROcp9_715 = ROcp9_16*S15+C15*S5;
    ROcp9_815 = ROcp9_26*S15+ROcp9_85*C15;
    ROcp9_915 = ROcp9_36*S15+ROcp9_95*C15;
    RLcp9_115 = ROcp9_46*s->dpt[2][3];
    RLcp9_215 = ROcp9_56*s->dpt[2][3];
    RLcp9_315 = ROcp9_66*s->dpt[2][3];
    OMcp9_115 = OMcp9_16+ROcp9_46*qd[15];
    OMcp9_215 = OMcp9_26+ROcp9_56*qd[15];
    OMcp9_315 = OMcp9_36+ROcp9_66*qd[15];
    RLcp9_116 = ROcp9_46*s->dpt[2][24];
    RLcp9_216 = ROcp9_56*s->dpt[2][24];
    RLcp9_316 = ROcp9_66*s->dpt[2][24];
    OMcp9_116 = OMcp9_115+ROcp9_115*qd[16];
    OMcp9_216 = OMcp9_215+ROcp9_215*qd[16];
    OMcp9_316 = OMcp9_315+ROcp9_315*qd[16];
    RLcp9_151 = ROcp9_115*s->dpt[1][27]+s->dpt[2][27]*(ROcp9_46*C16+ROcp9_715*S16)-s->dpt[3][27]*(ROcp9_46*S16-ROcp9_715*
 C16);
    RLcp9_251 = ROcp9_215*s->dpt[1][27]+s->dpt[2][27]*(ROcp9_56*C16+ROcp9_815*S16)-s->dpt[3][27]*(ROcp9_56*S16-ROcp9_815*
 C16);
    RLcp9_351 = ROcp9_315*s->dpt[1][27]+s->dpt[2][27]*(ROcp9_66*C16+ROcp9_915*S16)-s->dpt[3][27]*(ROcp9_66*S16-ROcp9_915*
 C16);
    POcp9_151 = RLcp9_115+RLcp9_116+RLcp9_151+q[1];
    POcp9_251 = RLcp9_215+RLcp9_216+RLcp9_251+q[2];
    POcp9_351 = RLcp9_315+RLcp9_316+RLcp9_351+q[3];
    VIcp9_151 = qd[1]+OMcp9_215*RLcp9_316+OMcp9_216*RLcp9_351+OMcp9_26*RLcp9_315-OMcp9_315*RLcp9_216-OMcp9_316*RLcp9_251-
 OMcp9_36*RLcp9_215;
    VIcp9_251 = qd[2]-OMcp9_115*RLcp9_316-OMcp9_116*RLcp9_351-OMcp9_16*RLcp9_315+OMcp9_315*RLcp9_116+OMcp9_316*RLcp9_151+
 OMcp9_36*RLcp9_115;
    VIcp9_351 = qd[3]+OMcp9_115*RLcp9_216+OMcp9_116*RLcp9_251+OMcp9_16*RLcp9_215-OMcp9_215*RLcp9_116-OMcp9_216*RLcp9_151-
 OMcp9_26*RLcp9_115;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_151;
    sens->P[2] = POcp9_251;
    sens->P[3] = POcp9_351;
    sens->V[1] = VIcp9_151;
    sens->V[2] = VIcp9_251;
    sens->V[3] = VIcp9_351;
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

// = = Block_1_0_0_11_0_5 = = 
 
// Sensor Kinematics 


    ROcp10_115 = ROcp10_16*C15-S15*S5;
    ROcp10_215 = ROcp10_26*C15-ROcp10_85*S15;
    ROcp10_315 = ROcp10_36*C15-ROcp10_95*S15;
    ROcp10_715 = ROcp10_16*S15+C15*S5;
    ROcp10_815 = ROcp10_26*S15+ROcp10_85*C15;
    ROcp10_915 = ROcp10_36*S15+ROcp10_95*C15;
    ROcp10_416 = ROcp10_46*C16+ROcp10_715*S16;
    ROcp10_516 = ROcp10_56*C16+ROcp10_815*S16;
    ROcp10_616 = ROcp10_66*C16+ROcp10_915*S16;
    ROcp10_716 = -(ROcp10_46*S16-ROcp10_715*C16);
    ROcp10_816 = -(ROcp10_56*S16-ROcp10_815*C16);
    ROcp10_916 = -(ROcp10_66*S16-ROcp10_915*C16);
    RLcp10_115 = ROcp10_46*s->dpt[2][3];
    RLcp10_215 = ROcp10_56*s->dpt[2][3];
    RLcp10_315 = ROcp10_66*s->dpt[2][3];
    OMcp10_115 = OMcp10_16+ROcp10_46*qd[15];
    OMcp10_215 = OMcp10_26+ROcp10_56*qd[15];
    OMcp10_315 = OMcp10_36+ROcp10_66*qd[15];
    RLcp10_116 = ROcp10_46*s->dpt[2][24];
    RLcp10_216 = ROcp10_56*s->dpt[2][24];
    RLcp10_316 = ROcp10_66*s->dpt[2][24];
    OMcp10_116 = OMcp10_115+ROcp10_115*qd[16];
    OMcp10_216 = OMcp10_215+ROcp10_215*qd[16];
    OMcp10_316 = OMcp10_315+ROcp10_315*qd[16];
    RLcp10_117 = ROcp10_716*s->dpt[3][26];
    RLcp10_217 = ROcp10_816*s->dpt[3][26];
    RLcp10_317 = ROcp10_916*s->dpt[3][26];
    OMcp10_117 = OMcp10_116+ROcp10_716*qd[17];
    OMcp10_217 = OMcp10_216+ROcp10_816*qd[17];
    OMcp10_317 = OMcp10_316+ROcp10_916*qd[17];
    RLcp10_152 = ROcp10_716*s->dpt[3][30]+s->dpt[1][30]*(ROcp10_115*C17+ROcp10_416*S17)-s->dpt[2][30]*(ROcp10_115*S17-
 ROcp10_416*C17);
    RLcp10_252 = ROcp10_816*s->dpt[3][30]+s->dpt[1][30]*(ROcp10_215*C17+ROcp10_516*S17)-s->dpt[2][30]*(ROcp10_215*S17-
 ROcp10_516*C17);
    RLcp10_352 = ROcp10_916*s->dpt[3][30]+s->dpt[1][30]*(ROcp10_315*C17+ROcp10_616*S17)-s->dpt[2][30]*(ROcp10_315*S17-
 ROcp10_616*C17);
    POcp10_152 = RLcp10_115+RLcp10_116+RLcp10_117+RLcp10_152+q[1];
    POcp10_252 = RLcp10_215+RLcp10_216+RLcp10_217+RLcp10_252+q[2];
    POcp10_352 = RLcp10_315+RLcp10_316+RLcp10_317+RLcp10_352+q[3];
    VIcp10_152 = qd[1]+OMcp10_215*RLcp10_316+OMcp10_216*RLcp10_317+OMcp10_217*RLcp10_352+OMcp10_26*RLcp10_315-OMcp10_315*
 RLcp10_216-OMcp10_316*RLcp10_217-OMcp10_317*RLcp10_252-OMcp10_36*RLcp10_215;
    VIcp10_252 = qd[2]-OMcp10_115*RLcp10_316-OMcp10_116*RLcp10_317-OMcp10_117*RLcp10_352-OMcp10_16*RLcp10_315+OMcp10_315*
 RLcp10_116+OMcp10_316*RLcp10_117+OMcp10_317*RLcp10_152+OMcp10_36*RLcp10_115;
    VIcp10_352 = qd[3]+OMcp10_115*RLcp10_216+OMcp10_116*RLcp10_217+OMcp10_117*RLcp10_252+OMcp10_16*RLcp10_215-OMcp10_215*
 RLcp10_116-OMcp10_216*RLcp10_117-OMcp10_217*RLcp10_152-OMcp10_26*RLcp10_115;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_152;
    sens->P[2] = POcp10_252;
    sens->P[3] = POcp10_352;
    sens->V[1] = VIcp10_152;
    sens->V[2] = VIcp10_252;
    sens->V[3] = VIcp10_352;
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

// = = Block_1_0_0_12_0_5 = = 
 
// Sensor Kinematics 


    ROcp11_115 = ROcp11_16*C15-S15*S5;
    ROcp11_215 = ROcp11_26*C15-ROcp11_85*S15;
    ROcp11_315 = ROcp11_36*C15-ROcp11_95*S15;
    ROcp11_715 = ROcp11_16*S15+C15*S5;
    ROcp11_815 = ROcp11_26*S15+ROcp11_85*C15;
    ROcp11_915 = ROcp11_36*S15+ROcp11_95*C15;
    ROcp11_416 = ROcp11_46*C16+ROcp11_715*S16;
    ROcp11_516 = ROcp11_56*C16+ROcp11_815*S16;
    ROcp11_616 = ROcp11_66*C16+ROcp11_915*S16;
    ROcp11_716 = -(ROcp11_46*S16-ROcp11_715*C16);
    ROcp11_816 = -(ROcp11_56*S16-ROcp11_815*C16);
    ROcp11_916 = -(ROcp11_66*S16-ROcp11_915*C16);
    ROcp11_117 = ROcp11_115*C17+ROcp11_416*S17;
    ROcp11_217 = ROcp11_215*C17+ROcp11_516*S17;
    ROcp11_317 = ROcp11_315*C17+ROcp11_616*S17;
    ROcp11_417 = -(ROcp11_115*S17-ROcp11_416*C17);
    ROcp11_517 = -(ROcp11_215*S17-ROcp11_516*C17);
    ROcp11_617 = -(ROcp11_315*S17-ROcp11_616*C17);
    RLcp11_115 = ROcp11_46*s->dpt[2][3];
    RLcp11_215 = ROcp11_56*s->dpt[2][3];
    RLcp11_315 = ROcp11_66*s->dpt[2][3];
    OMcp11_115 = OMcp11_16+ROcp11_46*qd[15];
    OMcp11_215 = OMcp11_26+ROcp11_56*qd[15];
    OMcp11_315 = OMcp11_36+ROcp11_66*qd[15];
    RLcp11_116 = ROcp11_46*s->dpt[2][24];
    RLcp11_216 = ROcp11_56*s->dpt[2][24];
    RLcp11_316 = ROcp11_66*s->dpt[2][24];
    OMcp11_116 = OMcp11_115+ROcp11_115*qd[16];
    OMcp11_216 = OMcp11_215+ROcp11_215*qd[16];
    OMcp11_316 = OMcp11_315+ROcp11_315*qd[16];
    RLcp11_117 = ROcp11_716*s->dpt[3][26];
    RLcp11_217 = ROcp11_816*s->dpt[3][26];
    RLcp11_317 = ROcp11_916*s->dpt[3][26];
    OMcp11_117 = OMcp11_116+ROcp11_716*qd[17];
    OMcp11_217 = OMcp11_216+ROcp11_816*qd[17];
    OMcp11_317 = OMcp11_316+ROcp11_916*qd[17];
    RLcp11_118 = ROcp11_716*s->dpt[3][29];
    RLcp11_218 = ROcp11_816*s->dpt[3][29];
    RLcp11_318 = ROcp11_916*s->dpt[3][29];
    OMcp11_118 = OMcp11_117+ROcp11_417*qd[18];
    OMcp11_218 = OMcp11_217+ROcp11_517*qd[18];
    OMcp11_318 = OMcp11_317+ROcp11_617*qd[18];
    RLcp11_153 = ROcp11_417*s->dpt[2][33]+s->dpt[1][33]*(ROcp11_117*C18-ROcp11_716*S18)+s->dpt[3][33]*(ROcp11_117*S18+
 ROcp11_716*C18);
    RLcp11_253 = ROcp11_517*s->dpt[2][33]+s->dpt[1][33]*(ROcp11_217*C18-ROcp11_816*S18)+s->dpt[3][33]*(ROcp11_217*S18+
 ROcp11_816*C18);
    RLcp11_353 = ROcp11_617*s->dpt[2][33]+s->dpt[1][33]*(ROcp11_317*C18-ROcp11_916*S18)+s->dpt[3][33]*(ROcp11_317*S18+
 ROcp11_916*C18);
    POcp11_153 = RLcp11_115+RLcp11_116+RLcp11_117+RLcp11_118+RLcp11_153+q[1];
    POcp11_253 = RLcp11_215+RLcp11_216+RLcp11_217+RLcp11_218+RLcp11_253+q[2];
    POcp11_353 = RLcp11_315+RLcp11_316+RLcp11_317+RLcp11_318+RLcp11_353+q[3];
    VIcp11_153 = qd[1]+OMcp11_215*RLcp11_316+OMcp11_216*RLcp11_317+OMcp11_217*RLcp11_318+OMcp11_218*RLcp11_353+OMcp11_26*
 RLcp11_315-OMcp11_315*RLcp11_216-OMcp11_316*RLcp11_217-OMcp11_317*RLcp11_218-OMcp11_318*RLcp11_253-OMcp11_36*RLcp11_215;
    VIcp11_253 = qd[2]-OMcp11_115*RLcp11_316-OMcp11_116*RLcp11_317-OMcp11_117*RLcp11_318-OMcp11_118*RLcp11_353-OMcp11_16*
 RLcp11_315+OMcp11_315*RLcp11_116+OMcp11_316*RLcp11_117+OMcp11_317*RLcp11_118+OMcp11_318*RLcp11_153+OMcp11_36*RLcp11_115;
    VIcp11_353 = qd[3]+OMcp11_115*RLcp11_216+OMcp11_116*RLcp11_217+OMcp11_117*RLcp11_218+OMcp11_118*RLcp11_253+OMcp11_16*
 RLcp11_215-OMcp11_215*RLcp11_116-OMcp11_216*RLcp11_117-OMcp11_217*RLcp11_118-OMcp11_218*RLcp11_153-OMcp11_26*RLcp11_115;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_153;
    sens->P[2] = POcp11_253;
    sens->P[3] = POcp11_353;
    sens->V[1] = VIcp11_153;
    sens->V[2] = VIcp11_253;
    sens->V[3] = VIcp11_353;
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

// = = Block_1_0_0_13_0_5 = = 
 
// Sensor Kinematics 


    ROcp12_115 = ROcp12_16*C15-S15*S5;
    ROcp12_215 = ROcp12_26*C15-ROcp12_85*S15;
    ROcp12_315 = ROcp12_36*C15-ROcp12_95*S15;
    ROcp12_715 = ROcp12_16*S15+C15*S5;
    ROcp12_815 = ROcp12_26*S15+ROcp12_85*C15;
    ROcp12_915 = ROcp12_36*S15+ROcp12_95*C15;
    ROcp12_416 = ROcp12_46*C16+ROcp12_715*S16;
    ROcp12_516 = ROcp12_56*C16+ROcp12_815*S16;
    ROcp12_616 = ROcp12_66*C16+ROcp12_915*S16;
    ROcp12_716 = -(ROcp12_46*S16-ROcp12_715*C16);
    ROcp12_816 = -(ROcp12_56*S16-ROcp12_815*C16);
    ROcp12_916 = -(ROcp12_66*S16-ROcp12_915*C16);
    ROcp12_117 = ROcp12_115*C17+ROcp12_416*S17;
    ROcp12_217 = ROcp12_215*C17+ROcp12_516*S17;
    ROcp12_317 = ROcp12_315*C17+ROcp12_616*S17;
    ROcp12_417 = -(ROcp12_115*S17-ROcp12_416*C17);
    ROcp12_517 = -(ROcp12_215*S17-ROcp12_516*C17);
    ROcp12_617 = -(ROcp12_315*S17-ROcp12_616*C17);
    ROcp12_118 = ROcp12_117*C18-ROcp12_716*S18;
    ROcp12_218 = ROcp12_217*C18-ROcp12_816*S18;
    ROcp12_318 = ROcp12_317*C18-ROcp12_916*S18;
    ROcp12_718 = ROcp12_117*S18+ROcp12_716*C18;
    ROcp12_818 = ROcp12_217*S18+ROcp12_816*C18;
    ROcp12_918 = ROcp12_317*S18+ROcp12_916*C18;
    RLcp12_115 = ROcp12_46*s->dpt[2][3];
    RLcp12_215 = ROcp12_56*s->dpt[2][3];
    RLcp12_315 = ROcp12_66*s->dpt[2][3];
    OMcp12_115 = OMcp12_16+ROcp12_46*qd[15];
    OMcp12_215 = OMcp12_26+ROcp12_56*qd[15];
    OMcp12_315 = OMcp12_36+ROcp12_66*qd[15];
    RLcp12_116 = ROcp12_46*s->dpt[2][24];
    RLcp12_216 = ROcp12_56*s->dpt[2][24];
    RLcp12_316 = ROcp12_66*s->dpt[2][24];
    OMcp12_116 = OMcp12_115+ROcp12_115*qd[16];
    OMcp12_216 = OMcp12_215+ROcp12_215*qd[16];
    OMcp12_316 = OMcp12_315+ROcp12_315*qd[16];
    RLcp12_117 = ROcp12_716*s->dpt[3][26];
    RLcp12_217 = ROcp12_816*s->dpt[3][26];
    RLcp12_317 = ROcp12_916*s->dpt[3][26];
    OMcp12_117 = OMcp12_116+ROcp12_716*qd[17];
    OMcp12_217 = OMcp12_216+ROcp12_816*qd[17];
    OMcp12_317 = OMcp12_316+ROcp12_916*qd[17];
    RLcp12_118 = ROcp12_716*s->dpt[3][29];
    RLcp12_218 = ROcp12_816*s->dpt[3][29];
    RLcp12_318 = ROcp12_916*s->dpt[3][29];
    OMcp12_118 = OMcp12_117+ROcp12_417*qd[18];
    OMcp12_218 = OMcp12_217+ROcp12_517*qd[18];
    OMcp12_318 = OMcp12_317+ROcp12_617*qd[18];
    RLcp12_119 = ROcp12_718*s->dpt[3][32];
    RLcp12_219 = ROcp12_818*s->dpt[3][32];
    RLcp12_319 = ROcp12_918*s->dpt[3][32];
    OMcp12_119 = OMcp12_118+ROcp12_118*qd[19];
    OMcp12_219 = OMcp12_218+ROcp12_218*qd[19];
    OMcp12_319 = OMcp12_318+ROcp12_318*qd[19];
    RLcp12_154 = ROcp12_118*s->dpt[1][35]+s->dpt[2][35]*(ROcp12_417*C19+ROcp12_718*S19)-s->dpt[3][35]*(ROcp12_417*S19-
 ROcp12_718*C19);
    RLcp12_254 = ROcp12_218*s->dpt[1][35]+s->dpt[2][35]*(ROcp12_517*C19+ROcp12_818*S19)-s->dpt[3][35]*(ROcp12_517*S19-
 ROcp12_818*C19);
    RLcp12_354 = ROcp12_318*s->dpt[1][35]+s->dpt[2][35]*(ROcp12_617*C19+ROcp12_918*S19)-s->dpt[3][35]*(ROcp12_617*S19-
 ROcp12_918*C19);
    POcp12_154 = RLcp12_115+RLcp12_116+RLcp12_117+RLcp12_118+RLcp12_119+RLcp12_154+q[1];
    POcp12_254 = RLcp12_215+RLcp12_216+RLcp12_217+RLcp12_218+RLcp12_219+RLcp12_254+q[2];
    POcp12_354 = RLcp12_315+RLcp12_316+RLcp12_317+RLcp12_318+RLcp12_319+RLcp12_354+q[3];
    VIcp12_154 = qd[1]+OMcp12_215*RLcp12_316+OMcp12_216*RLcp12_317+OMcp12_217*RLcp12_318+OMcp12_218*RLcp12_319+OMcp12_219*
 RLcp12_354+OMcp12_26*RLcp12_315-OMcp12_315*RLcp12_216-OMcp12_316*RLcp12_217-OMcp12_317*RLcp12_218-OMcp12_318*RLcp12_219-
 OMcp12_319*RLcp12_254-OMcp12_36*RLcp12_215;
    VIcp12_254 = qd[2]-OMcp12_115*RLcp12_316-OMcp12_116*RLcp12_317-OMcp12_117*RLcp12_318-OMcp12_118*RLcp12_319-OMcp12_119*
 RLcp12_354-OMcp12_16*RLcp12_315+OMcp12_315*RLcp12_116+OMcp12_316*RLcp12_117+OMcp12_317*RLcp12_118+OMcp12_318*RLcp12_119+
 OMcp12_319*RLcp12_154+OMcp12_36*RLcp12_115;
    VIcp12_354 = qd[3]+OMcp12_115*RLcp12_216+OMcp12_116*RLcp12_217+OMcp12_117*RLcp12_218+OMcp12_118*RLcp12_219+OMcp12_119*
 RLcp12_254+OMcp12_16*RLcp12_215-OMcp12_215*RLcp12_116-OMcp12_216*RLcp12_117-OMcp12_217*RLcp12_118-OMcp12_218*RLcp12_119-
 OMcp12_219*RLcp12_154-OMcp12_26*RLcp12_115;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_154;
    sens->P[2] = POcp12_254;
    sens->P[3] = POcp12_354;
    sens->V[1] = VIcp12_154;
    sens->V[2] = VIcp12_254;
    sens->V[3] = VIcp12_354;
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

// = = Block_1_0_0_14_0_5 = = 
 
// Sensor Kinematics 


    ROcp13_115 = ROcp13_16*C15-S15*S5;
    ROcp13_215 = ROcp13_26*C15-ROcp13_85*S15;
    ROcp13_315 = ROcp13_36*C15-ROcp13_95*S15;
    ROcp13_715 = ROcp13_16*S15+C15*S5;
    ROcp13_815 = ROcp13_26*S15+ROcp13_85*C15;
    ROcp13_915 = ROcp13_36*S15+ROcp13_95*C15;
    ROcp13_416 = ROcp13_46*C16+ROcp13_715*S16;
    ROcp13_516 = ROcp13_56*C16+ROcp13_815*S16;
    ROcp13_616 = ROcp13_66*C16+ROcp13_915*S16;
    ROcp13_716 = -(ROcp13_46*S16-ROcp13_715*C16);
    ROcp13_816 = -(ROcp13_56*S16-ROcp13_815*C16);
    ROcp13_916 = -(ROcp13_66*S16-ROcp13_915*C16);
    ROcp13_117 = ROcp13_115*C17+ROcp13_416*S17;
    ROcp13_217 = ROcp13_215*C17+ROcp13_516*S17;
    ROcp13_317 = ROcp13_315*C17+ROcp13_616*S17;
    ROcp13_417 = -(ROcp13_115*S17-ROcp13_416*C17);
    ROcp13_517 = -(ROcp13_215*S17-ROcp13_516*C17);
    ROcp13_617 = -(ROcp13_315*S17-ROcp13_616*C17);
    ROcp13_118 = ROcp13_117*C18-ROcp13_716*S18;
    ROcp13_218 = ROcp13_217*C18-ROcp13_816*S18;
    ROcp13_318 = ROcp13_317*C18-ROcp13_916*S18;
    ROcp13_718 = ROcp13_117*S18+ROcp13_716*C18;
    ROcp13_818 = ROcp13_217*S18+ROcp13_816*C18;
    ROcp13_918 = ROcp13_317*S18+ROcp13_916*C18;
    ROcp13_419 = ROcp13_417*C19+ROcp13_718*S19;
    ROcp13_519 = ROcp13_517*C19+ROcp13_818*S19;
    ROcp13_619 = ROcp13_617*C19+ROcp13_918*S19;
    ROcp13_719 = -(ROcp13_417*S19-ROcp13_718*C19);
    ROcp13_819 = -(ROcp13_517*S19-ROcp13_818*C19);
    ROcp13_919 = -(ROcp13_617*S19-ROcp13_918*C19);
    ROcp13_120 = ROcp13_118*C20-ROcp13_719*S20;
    ROcp13_220 = ROcp13_218*C20-ROcp13_819*S20;
    ROcp13_320 = ROcp13_318*C20-ROcp13_919*S20;
    ROcp13_720 = ROcp13_118*S20+ROcp13_719*C20;
    ROcp13_820 = ROcp13_218*S20+ROcp13_819*C20;
    ROcp13_920 = ROcp13_318*S20+ROcp13_919*C20;
    RLcp13_115 = ROcp13_46*s->dpt[2][3];
    RLcp13_215 = ROcp13_56*s->dpt[2][3];
    RLcp13_315 = ROcp13_66*s->dpt[2][3];
    OMcp13_115 = OMcp13_16+ROcp13_46*qd[15];
    OMcp13_215 = OMcp13_26+ROcp13_56*qd[15];
    OMcp13_315 = OMcp13_36+ROcp13_66*qd[15];
    RLcp13_116 = ROcp13_46*s->dpt[2][24];
    RLcp13_216 = ROcp13_56*s->dpt[2][24];
    RLcp13_316 = ROcp13_66*s->dpt[2][24];
    OMcp13_116 = OMcp13_115+ROcp13_115*qd[16];
    OMcp13_216 = OMcp13_215+ROcp13_215*qd[16];
    OMcp13_316 = OMcp13_315+ROcp13_315*qd[16];
    RLcp13_117 = ROcp13_716*s->dpt[3][26];
    RLcp13_217 = ROcp13_816*s->dpt[3][26];
    RLcp13_317 = ROcp13_916*s->dpt[3][26];
    OMcp13_117 = OMcp13_116+ROcp13_716*qd[17];
    OMcp13_217 = OMcp13_216+ROcp13_816*qd[17];
    OMcp13_317 = OMcp13_316+ROcp13_916*qd[17];
    RLcp13_118 = ROcp13_716*s->dpt[3][29];
    RLcp13_218 = ROcp13_816*s->dpt[3][29];
    RLcp13_318 = ROcp13_916*s->dpt[3][29];
    OMcp13_118 = OMcp13_117+ROcp13_417*qd[18];
    OMcp13_218 = OMcp13_217+ROcp13_517*qd[18];
    OMcp13_318 = OMcp13_317+ROcp13_617*qd[18];
    RLcp13_119 = ROcp13_718*s->dpt[3][32];
    RLcp13_219 = ROcp13_818*s->dpt[3][32];
    RLcp13_319 = ROcp13_918*s->dpt[3][32];
    POcp13_119 = RLcp13_115+RLcp13_116+RLcp13_117+RLcp13_118+RLcp13_119+q[1];
    POcp13_219 = RLcp13_215+RLcp13_216+RLcp13_217+RLcp13_218+RLcp13_219+q[2];
    POcp13_319 = RLcp13_315+RLcp13_316+RLcp13_317+RLcp13_318+RLcp13_319+q[3];
    VIcp13_119 = qd[1]+OMcp13_215*RLcp13_316+OMcp13_216*RLcp13_317+OMcp13_217*RLcp13_318+OMcp13_218*RLcp13_319+OMcp13_26*
 RLcp13_315-OMcp13_315*RLcp13_216-OMcp13_316*RLcp13_217-OMcp13_317*RLcp13_218-OMcp13_318*RLcp13_219-OMcp13_36*RLcp13_215;
    VIcp13_219 = qd[2]-OMcp13_115*RLcp13_316-OMcp13_116*RLcp13_317-OMcp13_117*RLcp13_318-OMcp13_118*RLcp13_319-OMcp13_16*
 RLcp13_315+OMcp13_315*RLcp13_116+OMcp13_316*RLcp13_117+OMcp13_317*RLcp13_118+OMcp13_318*RLcp13_119+OMcp13_36*RLcp13_115;
    VIcp13_319 = qd[3]+OMcp13_115*RLcp13_216+OMcp13_116*RLcp13_217+OMcp13_117*RLcp13_218+OMcp13_118*RLcp13_219+OMcp13_16*
 RLcp13_215-OMcp13_215*RLcp13_116-OMcp13_216*RLcp13_117-OMcp13_217*RLcp13_118-OMcp13_218*RLcp13_119-OMcp13_26*RLcp13_115;
    OMcp13_120 = OMcp13_118+ROcp13_118*qd[19]+ROcp13_419*qd[20];
    OMcp13_220 = OMcp13_218+ROcp13_218*qd[19]+ROcp13_519*qd[20];
    OMcp13_320 = OMcp13_318+ROcp13_318*qd[19]+ROcp13_619*qd[20];

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_119;
    sens->P[2] = POcp13_219;
    sens->P[3] = POcp13_319;
    sens->R[1][1] = ROcp13_120;
    sens->R[1][2] = ROcp13_220;
    sens->R[1][3] = ROcp13_320;
    sens->R[2][1] = ROcp13_419;
    sens->R[2][2] = ROcp13_519;
    sens->R[2][3] = ROcp13_619;
    sens->R[3][1] = ROcp13_720;
    sens->R[3][2] = ROcp13_820;
    sens->R[3][3] = ROcp13_920;
    sens->V[1] = VIcp13_119;
    sens->V[2] = VIcp13_219;
    sens->V[3] = VIcp13_319;
    sens->OM[1] = OMcp13_120;
    sens->OM[2] = OMcp13_220;
    sens->OM[3] = OMcp13_320;
 
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

// = = Block_1_0_0_15_0_5 = = 
 
// Sensor Kinematics 


    ROcp14_115 = ROcp14_16*C15-S15*S5;
    ROcp14_215 = ROcp14_26*C15-ROcp14_85*S15;
    ROcp14_315 = ROcp14_36*C15-ROcp14_95*S15;
    ROcp14_715 = ROcp14_16*S15+C15*S5;
    ROcp14_815 = ROcp14_26*S15+ROcp14_85*C15;
    ROcp14_915 = ROcp14_36*S15+ROcp14_95*C15;
    ROcp14_416 = ROcp14_46*C16+ROcp14_715*S16;
    ROcp14_516 = ROcp14_56*C16+ROcp14_815*S16;
    ROcp14_616 = ROcp14_66*C16+ROcp14_915*S16;
    ROcp14_716 = -(ROcp14_46*S16-ROcp14_715*C16);
    ROcp14_816 = -(ROcp14_56*S16-ROcp14_815*C16);
    ROcp14_916 = -(ROcp14_66*S16-ROcp14_915*C16);
    ROcp14_117 = ROcp14_115*C17+ROcp14_416*S17;
    ROcp14_217 = ROcp14_215*C17+ROcp14_516*S17;
    ROcp14_317 = ROcp14_315*C17+ROcp14_616*S17;
    ROcp14_417 = -(ROcp14_115*S17-ROcp14_416*C17);
    ROcp14_517 = -(ROcp14_215*S17-ROcp14_516*C17);
    ROcp14_617 = -(ROcp14_315*S17-ROcp14_616*C17);
    ROcp14_118 = ROcp14_117*C18-ROcp14_716*S18;
    ROcp14_218 = ROcp14_217*C18-ROcp14_816*S18;
    ROcp14_318 = ROcp14_317*C18-ROcp14_916*S18;
    ROcp14_718 = ROcp14_117*S18+ROcp14_716*C18;
    ROcp14_818 = ROcp14_217*S18+ROcp14_816*C18;
    ROcp14_918 = ROcp14_317*S18+ROcp14_916*C18;
    ROcp14_719 = -(ROcp14_417*S19-ROcp14_718*C19);
    ROcp14_819 = -(ROcp14_517*S19-ROcp14_818*C19);
    ROcp14_919 = -(ROcp14_617*S19-ROcp14_918*C19);
    RLcp14_115 = ROcp14_46*s->dpt[2][3];
    RLcp14_215 = ROcp14_56*s->dpt[2][3];
    RLcp14_315 = ROcp14_66*s->dpt[2][3];
    OMcp14_115 = OMcp14_16+ROcp14_46*qd[15];
    OMcp14_215 = OMcp14_26+ROcp14_56*qd[15];
    OMcp14_315 = OMcp14_36+ROcp14_66*qd[15];
    RLcp14_116 = ROcp14_46*s->dpt[2][24];
    RLcp14_216 = ROcp14_56*s->dpt[2][24];
    RLcp14_316 = ROcp14_66*s->dpt[2][24];
    OMcp14_116 = OMcp14_115+ROcp14_115*qd[16];
    OMcp14_216 = OMcp14_215+ROcp14_215*qd[16];
    OMcp14_316 = OMcp14_315+ROcp14_315*qd[16];
    RLcp14_117 = ROcp14_716*s->dpt[3][26];
    RLcp14_217 = ROcp14_816*s->dpt[3][26];
    RLcp14_317 = ROcp14_916*s->dpt[3][26];
    OMcp14_117 = OMcp14_116+ROcp14_716*qd[17];
    OMcp14_217 = OMcp14_216+ROcp14_816*qd[17];
    OMcp14_317 = OMcp14_316+ROcp14_916*qd[17];
    RLcp14_118 = ROcp14_716*s->dpt[3][29];
    RLcp14_218 = ROcp14_816*s->dpt[3][29];
    RLcp14_318 = ROcp14_916*s->dpt[3][29];
    OMcp14_118 = OMcp14_117+ROcp14_417*qd[18];
    OMcp14_218 = OMcp14_217+ROcp14_517*qd[18];
    OMcp14_318 = OMcp14_317+ROcp14_617*qd[18];
    RLcp14_119 = ROcp14_718*s->dpt[3][32];
    RLcp14_219 = ROcp14_818*s->dpt[3][32];
    RLcp14_319 = ROcp14_918*s->dpt[3][32];
    OMcp14_120 = OMcp14_118+ROcp14_118*qd[19]+qd[20]*(ROcp14_417*C19+ROcp14_718*S19);
    OMcp14_220 = OMcp14_218+ROcp14_218*qd[19]+qd[20]*(ROcp14_517*C19+ROcp14_818*S19);
    OMcp14_320 = OMcp14_318+ROcp14_318*qd[19]+qd[20]*(ROcp14_617*C19+ROcp14_918*S19);
    RLcp14_156 = s->dpt[1][37]*(ROcp14_118*C20-ROcp14_719*S20)+s->dpt[3][37]*(ROcp14_118*S20+ROcp14_719*C20);
    RLcp14_256 = s->dpt[1][37]*(ROcp14_218*C20-ROcp14_819*S20)+s->dpt[3][37]*(ROcp14_218*S20+ROcp14_819*C20);
    RLcp14_356 = s->dpt[1][37]*(ROcp14_318*C20-ROcp14_919*S20)+s->dpt[3][37]*(ROcp14_318*S20+ROcp14_919*C20);
    POcp14_156 = RLcp14_115+RLcp14_116+RLcp14_117+RLcp14_118+RLcp14_119+RLcp14_156+q[1];
    POcp14_256 = RLcp14_215+RLcp14_216+RLcp14_217+RLcp14_218+RLcp14_219+RLcp14_256+q[2];
    POcp14_356 = RLcp14_315+RLcp14_316+RLcp14_317+RLcp14_318+RLcp14_319+RLcp14_356+q[3];
    VIcp14_156 = qd[1]+OMcp14_215*RLcp14_316+OMcp14_216*RLcp14_317+OMcp14_217*RLcp14_318+OMcp14_218*RLcp14_319+OMcp14_220*
 RLcp14_356+OMcp14_26*RLcp14_315-OMcp14_315*RLcp14_216-OMcp14_316*RLcp14_217-OMcp14_317*RLcp14_218-OMcp14_318*RLcp14_219-
 OMcp14_320*RLcp14_256-OMcp14_36*RLcp14_215;
    VIcp14_256 = qd[2]-OMcp14_115*RLcp14_316-OMcp14_116*RLcp14_317-OMcp14_117*RLcp14_318-OMcp14_118*RLcp14_319-OMcp14_120*
 RLcp14_356-OMcp14_16*RLcp14_315+OMcp14_315*RLcp14_116+OMcp14_316*RLcp14_117+OMcp14_317*RLcp14_118+OMcp14_318*RLcp14_119+
 OMcp14_320*RLcp14_156+OMcp14_36*RLcp14_115;
    VIcp14_356 = qd[3]+OMcp14_115*RLcp14_216+OMcp14_116*RLcp14_217+OMcp14_117*RLcp14_218+OMcp14_118*RLcp14_219+OMcp14_120*
 RLcp14_256+OMcp14_16*RLcp14_215-OMcp14_215*RLcp14_116-OMcp14_216*RLcp14_117-OMcp14_217*RLcp14_118-OMcp14_218*RLcp14_119-
 OMcp14_220*RLcp14_156-OMcp14_26*RLcp14_115;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_156;
    sens->P[2] = POcp14_256;
    sens->P[3] = POcp14_356;
    sens->V[1] = VIcp14_156;
    sens->V[2] = VIcp14_256;
    sens->V[3] = VIcp14_356;
    sens->OM[1] = OMcp14_120;
    sens->OM[2] = OMcp14_220;
    sens->OM[3] = OMcp14_320;
 
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

// = = Block_1_0_0_16_0_8 = = 
 
// Sensor Kinematics 


    RLcp15_123 = ROcp15_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp15_223 = ROcp15_26*s->dpt[1][4]+ROcp15_85*s->dpt[3][4];
    RLcp15_323 = ROcp15_36*s->dpt[1][4]+ROcp15_95*s->dpt[3][4];
    OMcp15_123 = OMcp15_16+ROcp15_16*qd[23];
    OMcp15_223 = OMcp15_26+ROcp15_26*qd[23];
    OMcp15_323 = OMcp15_36+ROcp15_36*qd[23];
    RLcp15_157 = ROcp15_16*s->dpt[1][43]+s->dpt[2][43]*(ROcp15_46*C23+S23*S5)-s->dpt[3][43]*(ROcp15_46*S23-C23*S5);
    RLcp15_257 = ROcp15_26*s->dpt[1][43]+s->dpt[2][43]*(ROcp15_56*C23+ROcp15_85*S23)-s->dpt[3][43]*(ROcp15_56*S23-
 ROcp15_85*C23);
    RLcp15_357 = ROcp15_36*s->dpt[1][43]+s->dpt[2][43]*(ROcp15_66*C23+ROcp15_95*S23)-s->dpt[3][43]*(ROcp15_66*S23-
 ROcp15_95*C23);
    POcp15_157 = RLcp15_123+RLcp15_157+q[1];
    POcp15_257 = RLcp15_223+RLcp15_257+q[2];
    POcp15_357 = RLcp15_323+RLcp15_357+q[3];
    VIcp15_157 = qd[1]+OMcp15_223*RLcp15_357+OMcp15_26*RLcp15_323-OMcp15_323*RLcp15_257-OMcp15_36*RLcp15_223;
    VIcp15_257 = qd[2]-OMcp15_123*RLcp15_357-OMcp15_16*RLcp15_323+OMcp15_323*RLcp15_157+OMcp15_36*RLcp15_123;
    VIcp15_357 = qd[3]+OMcp15_123*RLcp15_257+OMcp15_16*RLcp15_223-OMcp15_223*RLcp15_157-OMcp15_26*RLcp15_123;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_157;
    sens->P[2] = POcp15_257;
    sens->P[3] = POcp15_357;
    sens->V[1] = VIcp15_157;
    sens->V[2] = VIcp15_257;
    sens->V[3] = VIcp15_357;
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

// = = Block_1_0_0_17_0_8 = = 
 
// Sensor Kinematics 


    ROcp16_423 = ROcp16_46*C23+S23*S5;
    ROcp16_523 = ROcp16_56*C23+ROcp16_85*S23;
    ROcp16_623 = ROcp16_66*C23+ROcp16_95*S23;
    ROcp16_723 = -(ROcp16_46*S23-C23*S5);
    ROcp16_823 = -(ROcp16_56*S23-ROcp16_85*C23);
    ROcp16_923 = -(ROcp16_66*S23-ROcp16_95*C23);
    RLcp16_123 = ROcp16_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp16_223 = ROcp16_26*s->dpt[1][4]+ROcp16_85*s->dpt[3][4];
    RLcp16_323 = ROcp16_36*s->dpt[1][4]+ROcp16_95*s->dpt[3][4];
    OMcp16_124 = OMcp16_16+ROcp16_16*qd[23]+ROcp16_423*qd[24];
    OMcp16_224 = OMcp16_26+ROcp16_26*qd[23]+ROcp16_523*qd[24];
    OMcp16_324 = OMcp16_36+ROcp16_36*qd[23]+ROcp16_623*qd[24];
    RLcp16_158 = ROcp16_423*s->dpt[2][45]+s->dpt[1][45]*(ROcp16_16*C24-ROcp16_723*S24)+s->dpt[3][45]*(ROcp16_16*S24+
 ROcp16_723*C24);
    RLcp16_258 = ROcp16_523*s->dpt[2][45]+s->dpt[1][45]*(ROcp16_26*C24-ROcp16_823*S24)+s->dpt[3][45]*(ROcp16_26*S24+
 ROcp16_823*C24);
    RLcp16_358 = ROcp16_623*s->dpt[2][45]+s->dpt[1][45]*(ROcp16_36*C24-ROcp16_923*S24)+s->dpt[3][45]*(ROcp16_36*S24+
 ROcp16_923*C24);
    POcp16_158 = RLcp16_123+RLcp16_158+q[1];
    POcp16_258 = RLcp16_223+RLcp16_258+q[2];
    POcp16_358 = RLcp16_323+RLcp16_358+q[3];
    VIcp16_158 = qd[1]+OMcp16_224*RLcp16_358+OMcp16_26*RLcp16_323-OMcp16_324*RLcp16_258-OMcp16_36*RLcp16_223;
    VIcp16_258 = qd[2]-OMcp16_124*RLcp16_358-OMcp16_16*RLcp16_323+OMcp16_324*RLcp16_158+OMcp16_36*RLcp16_123;
    VIcp16_358 = qd[3]+OMcp16_124*RLcp16_258+OMcp16_16*RLcp16_223-OMcp16_224*RLcp16_158-OMcp16_26*RLcp16_123;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_158;
    sens->P[2] = POcp16_258;
    sens->P[3] = POcp16_358;
    sens->V[1] = VIcp16_158;
    sens->V[2] = VIcp16_258;
    sens->V[3] = VIcp16_358;
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

// = = Block_1_0_0_18_0_8 = = 
 
// Sensor Kinematics 


    ROcp17_423 = ROcp17_46*C23+S23*S5;
    ROcp17_523 = ROcp17_56*C23+ROcp17_85*S23;
    ROcp17_623 = ROcp17_66*C23+ROcp17_95*S23;
    ROcp17_723 = -(ROcp17_46*S23-C23*S5);
    ROcp17_823 = -(ROcp17_56*S23-ROcp17_85*C23);
    ROcp17_923 = -(ROcp17_66*S23-ROcp17_95*C23);
    ROcp17_124 = ROcp17_16*C24-ROcp17_723*S24;
    ROcp17_224 = ROcp17_26*C24-ROcp17_823*S24;
    ROcp17_324 = ROcp17_36*C24-ROcp17_923*S24;
    ROcp17_724 = ROcp17_16*S24+ROcp17_723*C24;
    ROcp17_824 = ROcp17_26*S24+ROcp17_823*C24;
    ROcp17_924 = ROcp17_36*S24+ROcp17_923*C24;
    RLcp17_123 = ROcp17_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp17_223 = ROcp17_26*s->dpt[1][4]+ROcp17_85*s->dpt[3][4];
    RLcp17_323 = ROcp17_36*s->dpt[1][4]+ROcp17_95*s->dpt[3][4];
    OMcp17_124 = OMcp17_16+ROcp17_16*qd[23]+ROcp17_423*qd[24];
    OMcp17_224 = OMcp17_26+ROcp17_26*qd[23]+ROcp17_523*qd[24];
    OMcp17_324 = OMcp17_36+ROcp17_36*qd[23]+ROcp17_623*qd[24];
    RLcp17_125 = ROcp17_724*s->dpt[3][44];
    RLcp17_225 = ROcp17_824*s->dpt[3][44];
    RLcp17_325 = ROcp17_924*s->dpt[3][44];
    OMcp17_125 = OMcp17_124+ROcp17_724*qd[25];
    OMcp17_225 = OMcp17_224+ROcp17_824*qd[25];
    OMcp17_325 = OMcp17_324+ROcp17_924*qd[25];
    RLcp17_159 = ROcp17_724*s->dpt[3][47]+s->dpt[1][47]*(ROcp17_124*C25+ROcp17_423*S25)-s->dpt[2][47]*(ROcp17_124*S25-
 ROcp17_423*C25);
    RLcp17_259 = ROcp17_824*s->dpt[3][47]+s->dpt[1][47]*(ROcp17_224*C25+ROcp17_523*S25)-s->dpt[2][47]*(ROcp17_224*S25-
 ROcp17_523*C25);
    RLcp17_359 = ROcp17_924*s->dpt[3][47]+s->dpt[1][47]*(ROcp17_324*C25+ROcp17_623*S25)-s->dpt[2][47]*(ROcp17_324*S25-
 ROcp17_623*C25);
    POcp17_159 = RLcp17_123+RLcp17_125+RLcp17_159+q[1];
    POcp17_259 = RLcp17_223+RLcp17_225+RLcp17_259+q[2];
    POcp17_359 = RLcp17_323+RLcp17_325+RLcp17_359+q[3];
    VIcp17_159 = qd[1]+OMcp17_224*RLcp17_325+OMcp17_225*RLcp17_359+OMcp17_26*RLcp17_323-OMcp17_324*RLcp17_225-OMcp17_325*
 RLcp17_259-OMcp17_36*RLcp17_223;
    VIcp17_259 = qd[2]-OMcp17_124*RLcp17_325-OMcp17_125*RLcp17_359-OMcp17_16*RLcp17_323+OMcp17_324*RLcp17_125+OMcp17_325*
 RLcp17_159+OMcp17_36*RLcp17_123;
    VIcp17_359 = qd[3]+OMcp17_124*RLcp17_225+OMcp17_125*RLcp17_259+OMcp17_16*RLcp17_223-OMcp17_224*RLcp17_125-OMcp17_225*
 RLcp17_159-OMcp17_26*RLcp17_123;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_159;
    sens->P[2] = POcp17_259;
    sens->P[3] = POcp17_359;
    sens->V[1] = VIcp17_159;
    sens->V[2] = VIcp17_259;
    sens->V[3] = VIcp17_359;
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

// = = Block_1_0_0_19_0_8 = = 
 
// Sensor Kinematics 


    ROcp18_423 = ROcp18_46*C23+S23*S5;
    ROcp18_523 = ROcp18_56*C23+ROcp18_85*S23;
    ROcp18_623 = ROcp18_66*C23+ROcp18_95*S23;
    ROcp18_723 = -(ROcp18_46*S23-C23*S5);
    ROcp18_823 = -(ROcp18_56*S23-ROcp18_85*C23);
    ROcp18_923 = -(ROcp18_66*S23-ROcp18_95*C23);
    ROcp18_124 = ROcp18_16*C24-ROcp18_723*S24;
    ROcp18_224 = ROcp18_26*C24-ROcp18_823*S24;
    ROcp18_324 = ROcp18_36*C24-ROcp18_923*S24;
    ROcp18_724 = ROcp18_16*S24+ROcp18_723*C24;
    ROcp18_824 = ROcp18_26*S24+ROcp18_823*C24;
    ROcp18_924 = ROcp18_36*S24+ROcp18_923*C24;
    ROcp18_125 = ROcp18_124*C25+ROcp18_423*S25;
    ROcp18_225 = ROcp18_224*C25+ROcp18_523*S25;
    ROcp18_325 = ROcp18_324*C25+ROcp18_623*S25;
    ROcp18_425 = -(ROcp18_124*S25-ROcp18_423*C25);
    ROcp18_525 = -(ROcp18_224*S25-ROcp18_523*C25);
    ROcp18_625 = -(ROcp18_324*S25-ROcp18_623*C25);
    RLcp18_123 = ROcp18_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp18_223 = ROcp18_26*s->dpt[1][4]+ROcp18_85*s->dpt[3][4];
    RLcp18_323 = ROcp18_36*s->dpt[1][4]+ROcp18_95*s->dpt[3][4];
    OMcp18_124 = OMcp18_16+ROcp18_16*qd[23]+ROcp18_423*qd[24];
    OMcp18_224 = OMcp18_26+ROcp18_26*qd[23]+ROcp18_523*qd[24];
    OMcp18_324 = OMcp18_36+ROcp18_36*qd[23]+ROcp18_623*qd[24];
    RLcp18_125 = ROcp18_724*s->dpt[3][44];
    RLcp18_225 = ROcp18_824*s->dpt[3][44];
    RLcp18_325 = ROcp18_924*s->dpt[3][44];
    OMcp18_125 = OMcp18_124+ROcp18_724*qd[25];
    OMcp18_225 = OMcp18_224+ROcp18_824*qd[25];
    OMcp18_325 = OMcp18_324+ROcp18_924*qd[25];

// = = Block_1_0_0_19_0_9 = = 
 
// Sensor Kinematics 


    RLcp18_126 = ROcp18_125*s->dpt[1][48]+ROcp18_425*s->dpt[2][48]+ROcp18_724*s->dpt[3][48];
    RLcp18_226 = ROcp18_225*s->dpt[1][48]+ROcp18_525*s->dpt[2][48]+ROcp18_824*s->dpt[3][48];
    RLcp18_326 = ROcp18_325*s->dpt[1][48]+ROcp18_625*s->dpt[2][48]+ROcp18_924*s->dpt[3][48];
    OMcp18_126 = OMcp18_125+ROcp18_425*qd[26];
    OMcp18_226 = OMcp18_225+ROcp18_525*qd[26];
    OMcp18_326 = OMcp18_325+ROcp18_625*qd[26];
    RLcp18_160 = ROcp18_425*s->dpt[2][50]+s->dpt[1][50]*(ROcp18_125*C26-ROcp18_724*S26)+s->dpt[3][50]*(ROcp18_125*S26+
 ROcp18_724*C26);
    RLcp18_260 = ROcp18_525*s->dpt[2][50]+s->dpt[1][50]*(ROcp18_225*C26-ROcp18_824*S26)+s->dpt[3][50]*(ROcp18_225*S26+
 ROcp18_824*C26);
    RLcp18_360 = ROcp18_625*s->dpt[2][50]+s->dpt[1][50]*(ROcp18_325*C26-ROcp18_924*S26)+s->dpt[3][50]*(ROcp18_325*S26+
 ROcp18_924*C26);
    POcp18_160 = RLcp18_123+RLcp18_125+RLcp18_126+RLcp18_160+q[1];
    POcp18_260 = RLcp18_223+RLcp18_225+RLcp18_226+RLcp18_260+q[2];
    POcp18_360 = RLcp18_323+RLcp18_325+RLcp18_326+RLcp18_360+q[3];
    VIcp18_160 = qd[1]+OMcp18_224*RLcp18_325+OMcp18_225*RLcp18_326+OMcp18_226*RLcp18_360+OMcp18_26*RLcp18_323-OMcp18_324*
 RLcp18_225-OMcp18_325*RLcp18_226-OMcp18_326*RLcp18_260-OMcp18_36*RLcp18_223;
    VIcp18_260 = qd[2]-OMcp18_124*RLcp18_325-OMcp18_125*RLcp18_326-OMcp18_126*RLcp18_360-OMcp18_16*RLcp18_323+OMcp18_324*
 RLcp18_125+OMcp18_325*RLcp18_126+OMcp18_326*RLcp18_160+OMcp18_36*RLcp18_123;
    VIcp18_360 = qd[3]+OMcp18_124*RLcp18_225+OMcp18_125*RLcp18_226+OMcp18_126*RLcp18_260+OMcp18_16*RLcp18_223-OMcp18_224*
 RLcp18_125-OMcp18_225*RLcp18_126-OMcp18_226*RLcp18_160-OMcp18_26*RLcp18_123;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_160;
    sens->P[2] = POcp18_260;
    sens->P[3] = POcp18_360;
    sens->V[1] = VIcp18_160;
    sens->V[2] = VIcp18_260;
    sens->V[3] = VIcp18_360;
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

// = = Block_1_0_0_20_0_8 = = 
 
// Sensor Kinematics 


    ROcp19_423 = ROcp19_46*C23+S23*S5;
    ROcp19_523 = ROcp19_56*C23+ROcp19_85*S23;
    ROcp19_623 = ROcp19_66*C23+ROcp19_95*S23;
    ROcp19_723 = -(ROcp19_46*S23-C23*S5);
    ROcp19_823 = -(ROcp19_56*S23-ROcp19_85*C23);
    ROcp19_923 = -(ROcp19_66*S23-ROcp19_95*C23);
    ROcp19_124 = ROcp19_16*C24-ROcp19_723*S24;
    ROcp19_224 = ROcp19_26*C24-ROcp19_823*S24;
    ROcp19_324 = ROcp19_36*C24-ROcp19_923*S24;
    ROcp19_724 = ROcp19_16*S24+ROcp19_723*C24;
    ROcp19_824 = ROcp19_26*S24+ROcp19_823*C24;
    ROcp19_924 = ROcp19_36*S24+ROcp19_923*C24;
    ROcp19_125 = ROcp19_124*C25+ROcp19_423*S25;
    ROcp19_225 = ROcp19_224*C25+ROcp19_523*S25;
    ROcp19_325 = ROcp19_324*C25+ROcp19_623*S25;
    ROcp19_425 = -(ROcp19_124*S25-ROcp19_423*C25);
    ROcp19_525 = -(ROcp19_224*S25-ROcp19_523*C25);
    ROcp19_625 = -(ROcp19_324*S25-ROcp19_623*C25);
    RLcp19_123 = ROcp19_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp19_223 = ROcp19_26*s->dpt[1][4]+ROcp19_85*s->dpt[3][4];
    RLcp19_323 = ROcp19_36*s->dpt[1][4]+ROcp19_95*s->dpt[3][4];
    OMcp19_124 = OMcp19_16+ROcp19_16*qd[23]+ROcp19_423*qd[24];
    OMcp19_224 = OMcp19_26+ROcp19_26*qd[23]+ROcp19_523*qd[24];
    OMcp19_324 = OMcp19_36+ROcp19_36*qd[23]+ROcp19_623*qd[24];
    RLcp19_125 = ROcp19_724*s->dpt[3][44];
    RLcp19_225 = ROcp19_824*s->dpt[3][44];
    RLcp19_325 = ROcp19_924*s->dpt[3][44];
    OMcp19_125 = OMcp19_124+ROcp19_724*qd[25];
    OMcp19_225 = OMcp19_224+ROcp19_824*qd[25];
    OMcp19_325 = OMcp19_324+ROcp19_924*qd[25];

// = = Block_1_0_0_20_0_9 = = 
 
// Sensor Kinematics 


    ROcp19_126 = ROcp19_125*C26-ROcp19_724*S26;
    ROcp19_226 = ROcp19_225*C26-ROcp19_824*S26;
    ROcp19_326 = ROcp19_325*C26-ROcp19_924*S26;
    ROcp19_726 = ROcp19_125*S26+ROcp19_724*C26;
    ROcp19_826 = ROcp19_225*S26+ROcp19_824*C26;
    ROcp19_926 = ROcp19_325*S26+ROcp19_924*C26;
    RLcp19_126 = ROcp19_125*s->dpt[1][48]+ROcp19_425*s->dpt[2][48]+ROcp19_724*s->dpt[3][48];
    RLcp19_226 = ROcp19_225*s->dpt[1][48]+ROcp19_525*s->dpt[2][48]+ROcp19_824*s->dpt[3][48];
    RLcp19_326 = ROcp19_325*s->dpt[1][48]+ROcp19_625*s->dpt[2][48]+ROcp19_924*s->dpt[3][48];
    OMcp19_126 = OMcp19_125+ROcp19_425*qd[26];
    OMcp19_226 = OMcp19_225+ROcp19_525*qd[26];
    OMcp19_326 = OMcp19_325+ROcp19_625*qd[26];
    RLcp19_127 = ROcp19_425*s->dpt[2][51];
    RLcp19_227 = ROcp19_525*s->dpt[2][51];
    RLcp19_327 = ROcp19_625*s->dpt[2][51];
    OMcp19_127 = OMcp19_126+ROcp19_126*qd[27];
    OMcp19_227 = OMcp19_226+ROcp19_226*qd[27];
    OMcp19_327 = OMcp19_326+ROcp19_326*qd[27];
    RLcp19_161 = ROcp19_126*s->dpt[1][52]+s->dpt[2][52]*(ROcp19_425*C27+ROcp19_726*S27)-s->dpt[3][52]*(ROcp19_425*S27-
 ROcp19_726*C27);
    RLcp19_261 = ROcp19_226*s->dpt[1][52]+s->dpt[2][52]*(ROcp19_525*C27+ROcp19_826*S27)-s->dpt[3][52]*(ROcp19_525*S27-
 ROcp19_826*C27);
    RLcp19_361 = ROcp19_326*s->dpt[1][52]+s->dpt[2][52]*(ROcp19_625*C27+ROcp19_926*S27)-s->dpt[3][52]*(ROcp19_625*S27-
 ROcp19_926*C27);
    POcp19_161 = RLcp19_123+RLcp19_125+RLcp19_126+RLcp19_127+RLcp19_161+q[1];
    POcp19_261 = RLcp19_223+RLcp19_225+RLcp19_226+RLcp19_227+RLcp19_261+q[2];
    POcp19_361 = RLcp19_323+RLcp19_325+RLcp19_326+RLcp19_327+RLcp19_361+q[3];
    VIcp19_161 = qd[1]+OMcp19_224*RLcp19_325+OMcp19_225*RLcp19_326+OMcp19_226*RLcp19_327+OMcp19_227*RLcp19_361+OMcp19_26*
 RLcp19_323-OMcp19_324*RLcp19_225-OMcp19_325*RLcp19_226-OMcp19_326*RLcp19_227-OMcp19_327*RLcp19_261-OMcp19_36*RLcp19_223;
    VIcp19_261 = qd[2]-OMcp19_124*RLcp19_325-OMcp19_125*RLcp19_326-OMcp19_126*RLcp19_327-OMcp19_127*RLcp19_361-OMcp19_16*
 RLcp19_323+OMcp19_324*RLcp19_125+OMcp19_325*RLcp19_126+OMcp19_326*RLcp19_127+OMcp19_327*RLcp19_161+OMcp19_36*RLcp19_123;
    VIcp19_361 = qd[3]+OMcp19_124*RLcp19_225+OMcp19_125*RLcp19_226+OMcp19_126*RLcp19_227+OMcp19_127*RLcp19_261+OMcp19_16*
 RLcp19_223-OMcp19_224*RLcp19_125-OMcp19_225*RLcp19_126-OMcp19_226*RLcp19_127-OMcp19_227*RLcp19_161-OMcp19_26*RLcp19_123;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_161;
    sens->P[2] = POcp19_261;
    sens->P[3] = POcp19_361;
    sens->V[1] = VIcp19_161;
    sens->V[2] = VIcp19_261;
    sens->V[3] = VIcp19_361;
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

// = = Block_1_0_0_21_0_8 = = 
 
// Sensor Kinematics 


    ROcp20_423 = ROcp20_46*C23+S23*S5;
    ROcp20_523 = ROcp20_56*C23+ROcp20_85*S23;
    ROcp20_623 = ROcp20_66*C23+ROcp20_95*S23;
    ROcp20_723 = -(ROcp20_46*S23-C23*S5);
    ROcp20_823 = -(ROcp20_56*S23-ROcp20_85*C23);
    ROcp20_923 = -(ROcp20_66*S23-ROcp20_95*C23);
    ROcp20_124 = ROcp20_16*C24-ROcp20_723*S24;
    ROcp20_224 = ROcp20_26*C24-ROcp20_823*S24;
    ROcp20_324 = ROcp20_36*C24-ROcp20_923*S24;
    ROcp20_724 = ROcp20_16*S24+ROcp20_723*C24;
    ROcp20_824 = ROcp20_26*S24+ROcp20_823*C24;
    ROcp20_924 = ROcp20_36*S24+ROcp20_923*C24;
    ROcp20_125 = ROcp20_124*C25+ROcp20_423*S25;
    ROcp20_225 = ROcp20_224*C25+ROcp20_523*S25;
    ROcp20_325 = ROcp20_324*C25+ROcp20_623*S25;
    ROcp20_425 = -(ROcp20_124*S25-ROcp20_423*C25);
    ROcp20_525 = -(ROcp20_224*S25-ROcp20_523*C25);
    ROcp20_625 = -(ROcp20_324*S25-ROcp20_623*C25);
    RLcp20_123 = ROcp20_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp20_223 = ROcp20_26*s->dpt[1][4]+ROcp20_85*s->dpt[3][4];
    RLcp20_323 = ROcp20_36*s->dpt[1][4]+ROcp20_95*s->dpt[3][4];
    OMcp20_124 = OMcp20_16+ROcp20_16*qd[23]+ROcp20_423*qd[24];
    OMcp20_224 = OMcp20_26+ROcp20_26*qd[23]+ROcp20_523*qd[24];
    OMcp20_324 = OMcp20_36+ROcp20_36*qd[23]+ROcp20_623*qd[24];
    RLcp20_125 = ROcp20_724*s->dpt[3][44];
    RLcp20_225 = ROcp20_824*s->dpt[3][44];
    RLcp20_325 = ROcp20_924*s->dpt[3][44];
    OMcp20_125 = OMcp20_124+ROcp20_724*qd[25];
    OMcp20_225 = OMcp20_224+ROcp20_824*qd[25];
    OMcp20_325 = OMcp20_324+ROcp20_924*qd[25];

// = = Block_1_0_0_21_0_9 = = 
 
// Sensor Kinematics 


    ROcp20_126 = ROcp20_125*C26-ROcp20_724*S26;
    ROcp20_226 = ROcp20_225*C26-ROcp20_824*S26;
    ROcp20_326 = ROcp20_325*C26-ROcp20_924*S26;
    ROcp20_726 = ROcp20_125*S26+ROcp20_724*C26;
    ROcp20_826 = ROcp20_225*S26+ROcp20_824*C26;
    ROcp20_926 = ROcp20_325*S26+ROcp20_924*C26;
    ROcp20_427 = ROcp20_425*C27+ROcp20_726*S27;
    ROcp20_527 = ROcp20_525*C27+ROcp20_826*S27;
    ROcp20_627 = ROcp20_625*C27+ROcp20_926*S27;
    ROcp20_727 = -(ROcp20_425*S27-ROcp20_726*C27);
    ROcp20_827 = -(ROcp20_525*S27-ROcp20_826*C27);
    ROcp20_927 = -(ROcp20_625*S27-ROcp20_926*C27);
    RLcp20_126 = ROcp20_125*s->dpt[1][48]+ROcp20_425*s->dpt[2][48]+ROcp20_724*s->dpt[3][48];
    RLcp20_226 = ROcp20_225*s->dpt[1][48]+ROcp20_525*s->dpt[2][48]+ROcp20_824*s->dpt[3][48];
    RLcp20_326 = ROcp20_325*s->dpt[1][48]+ROcp20_625*s->dpt[2][48]+ROcp20_924*s->dpt[3][48];
    OMcp20_126 = OMcp20_125+ROcp20_425*qd[26];
    OMcp20_226 = OMcp20_225+ROcp20_525*qd[26];
    OMcp20_326 = OMcp20_325+ROcp20_625*qd[26];
    RLcp20_127 = ROcp20_425*s->dpt[2][51];
    RLcp20_227 = ROcp20_525*s->dpt[2][51];
    RLcp20_327 = ROcp20_625*s->dpt[2][51];
    OMcp20_127 = OMcp20_126+ROcp20_126*qd[27];
    OMcp20_227 = OMcp20_226+ROcp20_226*qd[27];
    OMcp20_327 = OMcp20_326+ROcp20_326*qd[27];
    RLcp20_128 = ROcp20_727*s->dpt[3][53];
    RLcp20_228 = ROcp20_827*s->dpt[3][53];
    RLcp20_328 = ROcp20_927*s->dpt[3][53];
    OMcp20_128 = OMcp20_127+ROcp20_727*qd[28];
    OMcp20_228 = OMcp20_227+ROcp20_827*qd[28];
    OMcp20_328 = OMcp20_327+ROcp20_927*qd[28];
    RLcp20_162 = ROcp20_727*s->dpt[3][55]+s->dpt[1][55]*(ROcp20_126*C28+ROcp20_427*S28)-s->dpt[2][55]*(ROcp20_126*S28-
 ROcp20_427*C28);
    RLcp20_262 = ROcp20_827*s->dpt[3][55]+s->dpt[1][55]*(ROcp20_226*C28+ROcp20_527*S28)-s->dpt[2][55]*(ROcp20_226*S28-
 ROcp20_527*C28);
    RLcp20_362 = ROcp20_927*s->dpt[3][55]+s->dpt[1][55]*(ROcp20_326*C28+ROcp20_627*S28)-s->dpt[2][55]*(ROcp20_326*S28-
 ROcp20_627*C28);
    POcp20_162 = RLcp20_123+RLcp20_125+RLcp20_126+RLcp20_127+RLcp20_128+RLcp20_162+q[1];
    POcp20_262 = RLcp20_223+RLcp20_225+RLcp20_226+RLcp20_227+RLcp20_228+RLcp20_262+q[2];
    POcp20_362 = RLcp20_323+RLcp20_325+RLcp20_326+RLcp20_327+RLcp20_328+RLcp20_362+q[3];
    VIcp20_162 = qd[1]+OMcp20_224*RLcp20_325+OMcp20_225*RLcp20_326+OMcp20_226*RLcp20_327+OMcp20_227*RLcp20_328+OMcp20_228*
 RLcp20_362+OMcp20_26*RLcp20_323-OMcp20_324*RLcp20_225-OMcp20_325*RLcp20_226-OMcp20_326*RLcp20_227-OMcp20_327*RLcp20_228-
 OMcp20_328*RLcp20_262-OMcp20_36*RLcp20_223;
    VIcp20_262 = qd[2]-OMcp20_124*RLcp20_325-OMcp20_125*RLcp20_326-OMcp20_126*RLcp20_327-OMcp20_127*RLcp20_328-OMcp20_128*
 RLcp20_362-OMcp20_16*RLcp20_323+OMcp20_324*RLcp20_125+OMcp20_325*RLcp20_126+OMcp20_326*RLcp20_127+OMcp20_327*RLcp20_128+
 OMcp20_328*RLcp20_162+OMcp20_36*RLcp20_123;
    VIcp20_362 = qd[3]+OMcp20_124*RLcp20_225+OMcp20_125*RLcp20_226+OMcp20_126*RLcp20_227+OMcp20_127*RLcp20_228+OMcp20_128*
 RLcp20_262+OMcp20_16*RLcp20_223-OMcp20_224*RLcp20_125-OMcp20_225*RLcp20_126-OMcp20_226*RLcp20_127-OMcp20_227*RLcp20_128-
 OMcp20_228*RLcp20_162-OMcp20_26*RLcp20_123;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_162;
    sens->P[2] = POcp20_262;
    sens->P[3] = POcp20_362;
    sens->V[1] = VIcp20_162;
    sens->V[2] = VIcp20_262;
    sens->V[3] = VIcp20_362;
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

// = = Block_1_0_0_22_0_8 = = 
 
// Sensor Kinematics 


    ROcp21_423 = ROcp21_46*C23+S23*S5;
    ROcp21_523 = ROcp21_56*C23+ROcp21_85*S23;
    ROcp21_623 = ROcp21_66*C23+ROcp21_95*S23;
    ROcp21_723 = -(ROcp21_46*S23-C23*S5);
    ROcp21_823 = -(ROcp21_56*S23-ROcp21_85*C23);
    ROcp21_923 = -(ROcp21_66*S23-ROcp21_95*C23);
    ROcp21_124 = ROcp21_16*C24-ROcp21_723*S24;
    ROcp21_224 = ROcp21_26*C24-ROcp21_823*S24;
    ROcp21_324 = ROcp21_36*C24-ROcp21_923*S24;
    ROcp21_724 = ROcp21_16*S24+ROcp21_723*C24;
    ROcp21_824 = ROcp21_26*S24+ROcp21_823*C24;
    ROcp21_924 = ROcp21_36*S24+ROcp21_923*C24;
    ROcp21_125 = ROcp21_124*C25+ROcp21_423*S25;
    ROcp21_225 = ROcp21_224*C25+ROcp21_523*S25;
    ROcp21_325 = ROcp21_324*C25+ROcp21_623*S25;
    ROcp21_425 = -(ROcp21_124*S25-ROcp21_423*C25);
    ROcp21_525 = -(ROcp21_224*S25-ROcp21_523*C25);
    ROcp21_625 = -(ROcp21_324*S25-ROcp21_623*C25);
    RLcp21_123 = ROcp21_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp21_223 = ROcp21_26*s->dpt[1][4]+ROcp21_85*s->dpt[3][4];
    RLcp21_323 = ROcp21_36*s->dpt[1][4]+ROcp21_95*s->dpt[3][4];
    OMcp21_124 = OMcp21_16+ROcp21_16*qd[23]+ROcp21_423*qd[24];
    OMcp21_224 = OMcp21_26+ROcp21_26*qd[23]+ROcp21_523*qd[24];
    OMcp21_324 = OMcp21_36+ROcp21_36*qd[23]+ROcp21_623*qd[24];
    RLcp21_125 = ROcp21_724*s->dpt[3][44];
    RLcp21_225 = ROcp21_824*s->dpt[3][44];
    RLcp21_325 = ROcp21_924*s->dpt[3][44];
    OMcp21_125 = OMcp21_124+ROcp21_724*qd[25];
    OMcp21_225 = OMcp21_224+ROcp21_824*qd[25];
    OMcp21_325 = OMcp21_324+ROcp21_924*qd[25];

// = = Block_1_0_0_22_0_9 = = 
 
// Sensor Kinematics 


    ROcp21_126 = ROcp21_125*C26-ROcp21_724*S26;
    ROcp21_226 = ROcp21_225*C26-ROcp21_824*S26;
    ROcp21_326 = ROcp21_325*C26-ROcp21_924*S26;
    ROcp21_726 = ROcp21_125*S26+ROcp21_724*C26;
    ROcp21_826 = ROcp21_225*S26+ROcp21_824*C26;
    ROcp21_926 = ROcp21_325*S26+ROcp21_924*C26;
    ROcp21_427 = ROcp21_425*C27+ROcp21_726*S27;
    ROcp21_527 = ROcp21_525*C27+ROcp21_826*S27;
    ROcp21_627 = ROcp21_625*C27+ROcp21_926*S27;
    ROcp21_727 = -(ROcp21_425*S27-ROcp21_726*C27);
    ROcp21_827 = -(ROcp21_525*S27-ROcp21_826*C27);
    ROcp21_927 = -(ROcp21_625*S27-ROcp21_926*C27);
    ROcp21_128 = ROcp21_126*C28+ROcp21_427*S28;
    ROcp21_228 = ROcp21_226*C28+ROcp21_527*S28;
    ROcp21_328 = ROcp21_326*C28+ROcp21_627*S28;
    ROcp21_428 = -(ROcp21_126*S28-ROcp21_427*C28);
    ROcp21_528 = -(ROcp21_226*S28-ROcp21_527*C28);
    ROcp21_628 = -(ROcp21_326*S28-ROcp21_627*C28);
    RLcp21_126 = ROcp21_125*s->dpt[1][48]+ROcp21_425*s->dpt[2][48]+ROcp21_724*s->dpt[3][48];
    RLcp21_226 = ROcp21_225*s->dpt[1][48]+ROcp21_525*s->dpt[2][48]+ROcp21_824*s->dpt[3][48];
    RLcp21_326 = ROcp21_325*s->dpt[1][48]+ROcp21_625*s->dpt[2][48]+ROcp21_924*s->dpt[3][48];
    OMcp21_126 = OMcp21_125+ROcp21_425*qd[26];
    OMcp21_226 = OMcp21_225+ROcp21_525*qd[26];
    OMcp21_326 = OMcp21_325+ROcp21_625*qd[26];
    RLcp21_127 = ROcp21_425*s->dpt[2][51];
    RLcp21_227 = ROcp21_525*s->dpt[2][51];
    RLcp21_327 = ROcp21_625*s->dpt[2][51];
    OMcp21_127 = OMcp21_126+ROcp21_126*qd[27];
    OMcp21_227 = OMcp21_226+ROcp21_226*qd[27];
    OMcp21_327 = OMcp21_326+ROcp21_326*qd[27];
    RLcp21_128 = ROcp21_727*s->dpt[3][53];
    RLcp21_228 = ROcp21_827*s->dpt[3][53];
    RLcp21_328 = ROcp21_927*s->dpt[3][53];
    OMcp21_128 = OMcp21_127+ROcp21_727*qd[28];
    OMcp21_228 = OMcp21_227+ROcp21_827*qd[28];
    OMcp21_328 = OMcp21_327+ROcp21_927*qd[28];
    RLcp21_129 = ROcp21_727*s->dpt[3][56];
    RLcp21_229 = ROcp21_827*s->dpt[3][56];
    RLcp21_329 = ROcp21_927*s->dpt[3][56];
    OMcp21_129 = OMcp21_128+ROcp21_428*qd[29];
    OMcp21_229 = OMcp21_228+ROcp21_528*qd[29];
    OMcp21_329 = OMcp21_328+ROcp21_628*qd[29];
    RLcp21_163 = ROcp21_428*s->dpt[2][58]+s->dpt[1][58]*(ROcp21_128*C29-ROcp21_727*S29)+s->dpt[3][58]*(ROcp21_128*S29+
 ROcp21_727*C29);
    RLcp21_263 = ROcp21_528*s->dpt[2][58]+s->dpt[1][58]*(ROcp21_228*C29-ROcp21_827*S29)+s->dpt[3][58]*(ROcp21_228*S29+
 ROcp21_827*C29);
    RLcp21_363 = ROcp21_628*s->dpt[2][58]+s->dpt[1][58]*(ROcp21_328*C29-ROcp21_927*S29)+s->dpt[3][58]*(ROcp21_328*S29+
 ROcp21_927*C29);
    POcp21_163 = RLcp21_123+RLcp21_125+RLcp21_126+RLcp21_127+RLcp21_128+RLcp21_129+RLcp21_163+q[1];
    POcp21_263 = RLcp21_223+RLcp21_225+RLcp21_226+RLcp21_227+RLcp21_228+RLcp21_229+RLcp21_263+q[2];
    POcp21_363 = RLcp21_323+RLcp21_325+RLcp21_326+RLcp21_327+RLcp21_328+RLcp21_329+RLcp21_363+q[3];
    VIcp21_163 = qd[1]+OMcp21_224*RLcp21_325+OMcp21_225*RLcp21_326+OMcp21_226*RLcp21_327+OMcp21_227*RLcp21_328+OMcp21_228*
 RLcp21_329+OMcp21_229*RLcp21_363+OMcp21_26*RLcp21_323-OMcp21_324*RLcp21_225-OMcp21_325*RLcp21_226-OMcp21_326*RLcp21_227-
 OMcp21_327*RLcp21_228-OMcp21_328*RLcp21_229-OMcp21_329*RLcp21_263-OMcp21_36*RLcp21_223;
    VIcp21_263 = qd[2]-OMcp21_124*RLcp21_325-OMcp21_125*RLcp21_326-OMcp21_126*RLcp21_327-OMcp21_127*RLcp21_328-OMcp21_128*
 RLcp21_329-OMcp21_129*RLcp21_363-OMcp21_16*RLcp21_323+OMcp21_324*RLcp21_125+OMcp21_325*RLcp21_126+OMcp21_326*RLcp21_127+
 OMcp21_327*RLcp21_128+OMcp21_328*RLcp21_129+OMcp21_329*RLcp21_163+OMcp21_36*RLcp21_123;
    VIcp21_363 = qd[3]+OMcp21_124*RLcp21_225+OMcp21_125*RLcp21_226+OMcp21_126*RLcp21_227+OMcp21_127*RLcp21_228+OMcp21_128*
 RLcp21_229+OMcp21_129*RLcp21_263+OMcp21_16*RLcp21_223-OMcp21_224*RLcp21_125-OMcp21_225*RLcp21_126-OMcp21_226*RLcp21_127-
 OMcp21_227*RLcp21_128-OMcp21_228*RLcp21_129-OMcp21_229*RLcp21_163-OMcp21_26*RLcp21_123;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_163;
    sens->P[2] = POcp21_263;
    sens->P[3] = POcp21_363;
    sens->V[1] = VIcp21_163;
    sens->V[2] = VIcp21_263;
    sens->V[3] = VIcp21_363;
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

// = = Block_1_0_0_23_0_8 = = 
 
// Sensor Kinematics 


    ROcp22_423 = ROcp22_46*C23+S23*S5;
    ROcp22_523 = ROcp22_56*C23+ROcp22_85*S23;
    ROcp22_623 = ROcp22_66*C23+ROcp22_95*S23;
    ROcp22_723 = -(ROcp22_46*S23-C23*S5);
    ROcp22_823 = -(ROcp22_56*S23-ROcp22_85*C23);
    ROcp22_923 = -(ROcp22_66*S23-ROcp22_95*C23);
    ROcp22_124 = ROcp22_16*C24-ROcp22_723*S24;
    ROcp22_224 = ROcp22_26*C24-ROcp22_823*S24;
    ROcp22_324 = ROcp22_36*C24-ROcp22_923*S24;
    ROcp22_724 = ROcp22_16*S24+ROcp22_723*C24;
    ROcp22_824 = ROcp22_26*S24+ROcp22_823*C24;
    ROcp22_924 = ROcp22_36*S24+ROcp22_923*C24;
    ROcp22_125 = ROcp22_124*C25+ROcp22_423*S25;
    ROcp22_225 = ROcp22_224*C25+ROcp22_523*S25;
    ROcp22_325 = ROcp22_324*C25+ROcp22_623*S25;
    ROcp22_425 = -(ROcp22_124*S25-ROcp22_423*C25);
    ROcp22_525 = -(ROcp22_224*S25-ROcp22_523*C25);
    ROcp22_625 = -(ROcp22_324*S25-ROcp22_623*C25);
    RLcp22_123 = ROcp22_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp22_223 = ROcp22_26*s->dpt[1][4]+ROcp22_85*s->dpt[3][4];
    RLcp22_323 = ROcp22_36*s->dpt[1][4]+ROcp22_95*s->dpt[3][4];
    OMcp22_124 = OMcp22_16+ROcp22_16*qd[23]+ROcp22_423*qd[24];
    OMcp22_224 = OMcp22_26+ROcp22_26*qd[23]+ROcp22_523*qd[24];
    OMcp22_324 = OMcp22_36+ROcp22_36*qd[23]+ROcp22_623*qd[24];
    RLcp22_125 = ROcp22_724*s->dpt[3][44];
    RLcp22_225 = ROcp22_824*s->dpt[3][44];
    RLcp22_325 = ROcp22_924*s->dpt[3][44];
    OMcp22_125 = OMcp22_124+ROcp22_724*qd[25];
    OMcp22_225 = OMcp22_224+ROcp22_824*qd[25];
    OMcp22_325 = OMcp22_324+ROcp22_924*qd[25];

// = = Block_1_0_0_23_0_10 = = 
 
// Sensor Kinematics 


    RLcp22_130 = ROcp22_125*s->dpt[1][49]+ROcp22_425*s->dpt[2][49]+ROcp22_724*s->dpt[3][49];
    RLcp22_230 = ROcp22_225*s->dpt[1][49]+ROcp22_525*s->dpt[2][49]+ROcp22_824*s->dpt[3][49];
    RLcp22_330 = ROcp22_325*s->dpt[1][49]+ROcp22_625*s->dpt[2][49]+ROcp22_924*s->dpt[3][49];
    OMcp22_130 = OMcp22_125+ROcp22_425*qd[30];
    OMcp22_230 = OMcp22_225+ROcp22_525*qd[30];
    OMcp22_330 = OMcp22_325+ROcp22_625*qd[30];
    RLcp22_164 = ROcp22_425*s->dpt[2][59]+s->dpt[1][59]*(ROcp22_125*C30-ROcp22_724*S30)+s->dpt[3][59]*(ROcp22_125*S30+
 ROcp22_724*C30);
    RLcp22_264 = ROcp22_525*s->dpt[2][59]+s->dpt[1][59]*(ROcp22_225*C30-ROcp22_824*S30)+s->dpt[3][59]*(ROcp22_225*S30+
 ROcp22_824*C30);
    RLcp22_364 = ROcp22_625*s->dpt[2][59]+s->dpt[1][59]*(ROcp22_325*C30-ROcp22_924*S30)+s->dpt[3][59]*(ROcp22_325*S30+
 ROcp22_924*C30);
    POcp22_164 = RLcp22_123+RLcp22_125+RLcp22_130+RLcp22_164+q[1];
    POcp22_264 = RLcp22_223+RLcp22_225+RLcp22_230+RLcp22_264+q[2];
    POcp22_364 = RLcp22_323+RLcp22_325+RLcp22_330+RLcp22_364+q[3];
    VIcp22_164 = qd[1]+OMcp22_224*RLcp22_325+OMcp22_225*RLcp22_330+OMcp22_230*RLcp22_364+OMcp22_26*RLcp22_323-OMcp22_324*
 RLcp22_225-OMcp22_325*RLcp22_230-OMcp22_330*RLcp22_264-OMcp22_36*RLcp22_223;
    VIcp22_264 = qd[2]-OMcp22_124*RLcp22_325-OMcp22_125*RLcp22_330-OMcp22_130*RLcp22_364-OMcp22_16*RLcp22_323+OMcp22_324*
 RLcp22_125+OMcp22_325*RLcp22_130+OMcp22_330*RLcp22_164+OMcp22_36*RLcp22_123;
    VIcp22_364 = qd[3]+OMcp22_124*RLcp22_225+OMcp22_125*RLcp22_230+OMcp22_130*RLcp22_264+OMcp22_16*RLcp22_223-OMcp22_224*
 RLcp22_125-OMcp22_225*RLcp22_130-OMcp22_230*RLcp22_164-OMcp22_26*RLcp22_123;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_164;
    sens->P[2] = POcp22_264;
    sens->P[3] = POcp22_364;
    sens->V[1] = VIcp22_164;
    sens->V[2] = VIcp22_264;
    sens->V[3] = VIcp22_364;
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

// = = Block_1_0_0_24_0_8 = = 
 
// Sensor Kinematics 


    ROcp23_423 = ROcp23_46*C23+S23*S5;
    ROcp23_523 = ROcp23_56*C23+ROcp23_85*S23;
    ROcp23_623 = ROcp23_66*C23+ROcp23_95*S23;
    ROcp23_723 = -(ROcp23_46*S23-C23*S5);
    ROcp23_823 = -(ROcp23_56*S23-ROcp23_85*C23);
    ROcp23_923 = -(ROcp23_66*S23-ROcp23_95*C23);
    ROcp23_124 = ROcp23_16*C24-ROcp23_723*S24;
    ROcp23_224 = ROcp23_26*C24-ROcp23_823*S24;
    ROcp23_324 = ROcp23_36*C24-ROcp23_923*S24;
    ROcp23_724 = ROcp23_16*S24+ROcp23_723*C24;
    ROcp23_824 = ROcp23_26*S24+ROcp23_823*C24;
    ROcp23_924 = ROcp23_36*S24+ROcp23_923*C24;
    ROcp23_125 = ROcp23_124*C25+ROcp23_423*S25;
    ROcp23_225 = ROcp23_224*C25+ROcp23_523*S25;
    ROcp23_325 = ROcp23_324*C25+ROcp23_623*S25;
    ROcp23_425 = -(ROcp23_124*S25-ROcp23_423*C25);
    ROcp23_525 = -(ROcp23_224*S25-ROcp23_523*C25);
    ROcp23_625 = -(ROcp23_324*S25-ROcp23_623*C25);
    RLcp23_123 = ROcp23_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp23_223 = ROcp23_26*s->dpt[1][4]+ROcp23_85*s->dpt[3][4];
    RLcp23_323 = ROcp23_36*s->dpt[1][4]+ROcp23_95*s->dpt[3][4];
    OMcp23_124 = OMcp23_16+ROcp23_16*qd[23]+ROcp23_423*qd[24];
    OMcp23_224 = OMcp23_26+ROcp23_26*qd[23]+ROcp23_523*qd[24];
    OMcp23_324 = OMcp23_36+ROcp23_36*qd[23]+ROcp23_623*qd[24];
    RLcp23_125 = ROcp23_724*s->dpt[3][44];
    RLcp23_225 = ROcp23_824*s->dpt[3][44];
    RLcp23_325 = ROcp23_924*s->dpt[3][44];
    OMcp23_125 = OMcp23_124+ROcp23_724*qd[25];
    OMcp23_225 = OMcp23_224+ROcp23_824*qd[25];
    OMcp23_325 = OMcp23_324+ROcp23_924*qd[25];

// = = Block_1_0_0_24_0_10 = = 
 
// Sensor Kinematics 


    ROcp23_130 = ROcp23_125*C30-ROcp23_724*S30;
    ROcp23_230 = ROcp23_225*C30-ROcp23_824*S30;
    ROcp23_330 = ROcp23_325*C30-ROcp23_924*S30;
    ROcp23_730 = ROcp23_125*S30+ROcp23_724*C30;
    ROcp23_830 = ROcp23_225*S30+ROcp23_824*C30;
    ROcp23_930 = ROcp23_325*S30+ROcp23_924*C30;
    RLcp23_130 = ROcp23_125*s->dpt[1][49]+ROcp23_425*s->dpt[2][49]+ROcp23_724*s->dpt[3][49];
    RLcp23_230 = ROcp23_225*s->dpt[1][49]+ROcp23_525*s->dpt[2][49]+ROcp23_824*s->dpt[3][49];
    RLcp23_330 = ROcp23_325*s->dpt[1][49]+ROcp23_625*s->dpt[2][49]+ROcp23_924*s->dpt[3][49];
    OMcp23_130 = OMcp23_125+ROcp23_425*qd[30];
    OMcp23_230 = OMcp23_225+ROcp23_525*qd[30];
    OMcp23_330 = OMcp23_325+ROcp23_625*qd[30];
    RLcp23_131 = ROcp23_425*s->dpt[2][60];
    RLcp23_231 = ROcp23_525*s->dpt[2][60];
    RLcp23_331 = ROcp23_625*s->dpt[2][60];
    OMcp23_131 = OMcp23_130+ROcp23_130*qd[31];
    OMcp23_231 = OMcp23_230+ROcp23_230*qd[31];
    OMcp23_331 = OMcp23_330+ROcp23_330*qd[31];
    RLcp23_165 = ROcp23_130*s->dpt[1][61]+s->dpt[2][61]*(ROcp23_425*C31+ROcp23_730*S31)-s->dpt[3][61]*(ROcp23_425*S31-
 ROcp23_730*C31);
    RLcp23_265 = ROcp23_230*s->dpt[1][61]+s->dpt[2][61]*(ROcp23_525*C31+ROcp23_830*S31)-s->dpt[3][61]*(ROcp23_525*S31-
 ROcp23_830*C31);
    RLcp23_365 = ROcp23_330*s->dpt[1][61]+s->dpt[2][61]*(ROcp23_625*C31+ROcp23_930*S31)-s->dpt[3][61]*(ROcp23_625*S31-
 ROcp23_930*C31);
    POcp23_165 = RLcp23_123+RLcp23_125+RLcp23_130+RLcp23_131+RLcp23_165+q[1];
    POcp23_265 = RLcp23_223+RLcp23_225+RLcp23_230+RLcp23_231+RLcp23_265+q[2];
    POcp23_365 = RLcp23_323+RLcp23_325+RLcp23_330+RLcp23_331+RLcp23_365+q[3];
    VIcp23_165 = qd[1]+OMcp23_224*RLcp23_325+OMcp23_225*RLcp23_330+OMcp23_230*RLcp23_331+OMcp23_231*RLcp23_365+OMcp23_26*
 RLcp23_323-OMcp23_324*RLcp23_225-OMcp23_325*RLcp23_230-OMcp23_330*RLcp23_231-OMcp23_331*RLcp23_265-OMcp23_36*RLcp23_223;
    VIcp23_265 = qd[2]-OMcp23_124*RLcp23_325-OMcp23_125*RLcp23_330-OMcp23_130*RLcp23_331-OMcp23_131*RLcp23_365-OMcp23_16*
 RLcp23_323+OMcp23_324*RLcp23_125+OMcp23_325*RLcp23_130+OMcp23_330*RLcp23_131+OMcp23_331*RLcp23_165+OMcp23_36*RLcp23_123;
    VIcp23_365 = qd[3]+OMcp23_124*RLcp23_225+OMcp23_125*RLcp23_230+OMcp23_130*RLcp23_231+OMcp23_131*RLcp23_265+OMcp23_16*
 RLcp23_223-OMcp23_224*RLcp23_125-OMcp23_225*RLcp23_130-OMcp23_230*RLcp23_131-OMcp23_231*RLcp23_165-OMcp23_26*RLcp23_123;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_165;
    sens->P[2] = POcp23_265;
    sens->P[3] = POcp23_365;
    sens->V[1] = VIcp23_165;
    sens->V[2] = VIcp23_265;
    sens->V[3] = VIcp23_365;
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

// = = Block_1_0_0_25_0_8 = = 
 
// Sensor Kinematics 


    ROcp24_423 = ROcp24_46*C23+S23*S5;
    ROcp24_523 = ROcp24_56*C23+ROcp24_85*S23;
    ROcp24_623 = ROcp24_66*C23+ROcp24_95*S23;
    ROcp24_723 = -(ROcp24_46*S23-C23*S5);
    ROcp24_823 = -(ROcp24_56*S23-ROcp24_85*C23);
    ROcp24_923 = -(ROcp24_66*S23-ROcp24_95*C23);
    ROcp24_124 = ROcp24_16*C24-ROcp24_723*S24;
    ROcp24_224 = ROcp24_26*C24-ROcp24_823*S24;
    ROcp24_324 = ROcp24_36*C24-ROcp24_923*S24;
    ROcp24_724 = ROcp24_16*S24+ROcp24_723*C24;
    ROcp24_824 = ROcp24_26*S24+ROcp24_823*C24;
    ROcp24_924 = ROcp24_36*S24+ROcp24_923*C24;
    ROcp24_125 = ROcp24_124*C25+ROcp24_423*S25;
    ROcp24_225 = ROcp24_224*C25+ROcp24_523*S25;
    ROcp24_325 = ROcp24_324*C25+ROcp24_623*S25;
    ROcp24_425 = -(ROcp24_124*S25-ROcp24_423*C25);
    ROcp24_525 = -(ROcp24_224*S25-ROcp24_523*C25);
    ROcp24_625 = -(ROcp24_324*S25-ROcp24_623*C25);
    RLcp24_123 = ROcp24_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp24_223 = ROcp24_26*s->dpt[1][4]+ROcp24_85*s->dpt[3][4];
    RLcp24_323 = ROcp24_36*s->dpt[1][4]+ROcp24_95*s->dpt[3][4];
    OMcp24_124 = OMcp24_16+ROcp24_16*qd[23]+ROcp24_423*qd[24];
    OMcp24_224 = OMcp24_26+ROcp24_26*qd[23]+ROcp24_523*qd[24];
    OMcp24_324 = OMcp24_36+ROcp24_36*qd[23]+ROcp24_623*qd[24];
    RLcp24_125 = ROcp24_724*s->dpt[3][44];
    RLcp24_225 = ROcp24_824*s->dpt[3][44];
    RLcp24_325 = ROcp24_924*s->dpt[3][44];
    OMcp24_125 = OMcp24_124+ROcp24_724*qd[25];
    OMcp24_225 = OMcp24_224+ROcp24_824*qd[25];
    OMcp24_325 = OMcp24_324+ROcp24_924*qd[25];

// = = Block_1_0_0_25_0_10 = = 
 
// Sensor Kinematics 


    ROcp24_130 = ROcp24_125*C30-ROcp24_724*S30;
    ROcp24_230 = ROcp24_225*C30-ROcp24_824*S30;
    ROcp24_330 = ROcp24_325*C30-ROcp24_924*S30;
    ROcp24_730 = ROcp24_125*S30+ROcp24_724*C30;
    ROcp24_830 = ROcp24_225*S30+ROcp24_824*C30;
    ROcp24_930 = ROcp24_325*S30+ROcp24_924*C30;
    ROcp24_431 = ROcp24_425*C31+ROcp24_730*S31;
    ROcp24_531 = ROcp24_525*C31+ROcp24_830*S31;
    ROcp24_631 = ROcp24_625*C31+ROcp24_930*S31;
    ROcp24_731 = -(ROcp24_425*S31-ROcp24_730*C31);
    ROcp24_831 = -(ROcp24_525*S31-ROcp24_830*C31);
    ROcp24_931 = -(ROcp24_625*S31-ROcp24_930*C31);
    RLcp24_130 = ROcp24_125*s->dpt[1][49]+ROcp24_425*s->dpt[2][49]+ROcp24_724*s->dpt[3][49];
    RLcp24_230 = ROcp24_225*s->dpt[1][49]+ROcp24_525*s->dpt[2][49]+ROcp24_824*s->dpt[3][49];
    RLcp24_330 = ROcp24_325*s->dpt[1][49]+ROcp24_625*s->dpt[2][49]+ROcp24_924*s->dpt[3][49];
    OMcp24_130 = OMcp24_125+ROcp24_425*qd[30];
    OMcp24_230 = OMcp24_225+ROcp24_525*qd[30];
    OMcp24_330 = OMcp24_325+ROcp24_625*qd[30];
    RLcp24_131 = ROcp24_425*s->dpt[2][60];
    RLcp24_231 = ROcp24_525*s->dpt[2][60];
    RLcp24_331 = ROcp24_625*s->dpt[2][60];
    OMcp24_131 = OMcp24_130+ROcp24_130*qd[31];
    OMcp24_231 = OMcp24_230+ROcp24_230*qd[31];
    OMcp24_331 = OMcp24_330+ROcp24_330*qd[31];
    RLcp24_132 = ROcp24_731*s->dpt[3][62];
    RLcp24_232 = ROcp24_831*s->dpt[3][62];
    RLcp24_332 = ROcp24_931*s->dpt[3][62];
    OMcp24_132 = OMcp24_131+ROcp24_731*qd[32];
    OMcp24_232 = OMcp24_231+ROcp24_831*qd[32];
    OMcp24_332 = OMcp24_331+ROcp24_931*qd[32];
    RLcp24_166 = ROcp24_731*s->dpt[3][64]+s->dpt[1][64]*(ROcp24_130*C32+ROcp24_431*S32)-s->dpt[2][64]*(ROcp24_130*S32-
 ROcp24_431*C32);
    RLcp24_266 = ROcp24_831*s->dpt[3][64]+s->dpt[1][64]*(ROcp24_230*C32+ROcp24_531*S32)-s->dpt[2][64]*(ROcp24_230*S32-
 ROcp24_531*C32);
    RLcp24_366 = ROcp24_931*s->dpt[3][64]+s->dpt[1][64]*(ROcp24_330*C32+ROcp24_631*S32)-s->dpt[2][64]*(ROcp24_330*S32-
 ROcp24_631*C32);
    POcp24_166 = RLcp24_123+RLcp24_125+RLcp24_130+RLcp24_131+RLcp24_132+RLcp24_166+q[1];
    POcp24_266 = RLcp24_223+RLcp24_225+RLcp24_230+RLcp24_231+RLcp24_232+RLcp24_266+q[2];
    POcp24_366 = RLcp24_323+RLcp24_325+RLcp24_330+RLcp24_331+RLcp24_332+RLcp24_366+q[3];
    VIcp24_166 = qd[1]+OMcp24_224*RLcp24_325+OMcp24_225*RLcp24_330+OMcp24_230*RLcp24_331+OMcp24_231*RLcp24_332+OMcp24_232*
 RLcp24_366+OMcp24_26*RLcp24_323-OMcp24_324*RLcp24_225-OMcp24_325*RLcp24_230-OMcp24_330*RLcp24_231-OMcp24_331*RLcp24_232-
 OMcp24_332*RLcp24_266-OMcp24_36*RLcp24_223;
    VIcp24_266 = qd[2]-OMcp24_124*RLcp24_325-OMcp24_125*RLcp24_330-OMcp24_130*RLcp24_331-OMcp24_131*RLcp24_332-OMcp24_132*
 RLcp24_366-OMcp24_16*RLcp24_323+OMcp24_324*RLcp24_125+OMcp24_325*RLcp24_130+OMcp24_330*RLcp24_131+OMcp24_331*RLcp24_132+
 OMcp24_332*RLcp24_166+OMcp24_36*RLcp24_123;
    VIcp24_366 = qd[3]+OMcp24_124*RLcp24_225+OMcp24_125*RLcp24_230+OMcp24_130*RLcp24_231+OMcp24_131*RLcp24_232+OMcp24_132*
 RLcp24_266+OMcp24_16*RLcp24_223-OMcp24_224*RLcp24_125-OMcp24_225*RLcp24_130-OMcp24_230*RLcp24_131-OMcp24_231*RLcp24_132-
 OMcp24_232*RLcp24_166-OMcp24_26*RLcp24_123;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_166;
    sens->P[2] = POcp24_266;
    sens->P[3] = POcp24_366;
    sens->V[1] = VIcp24_166;
    sens->V[2] = VIcp24_266;
    sens->V[3] = VIcp24_366;
    sens->OM[1] = OMcp24_132;
    sens->OM[2] = OMcp24_232;
    sens->OM[3] = OMcp24_332;
 
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

// = = Block_1_0_0_26_0_8 = = 
 
// Sensor Kinematics 


    ROcp25_423 = ROcp25_46*C23+S23*S5;
    ROcp25_523 = ROcp25_56*C23+ROcp25_85*S23;
    ROcp25_623 = ROcp25_66*C23+ROcp25_95*S23;
    ROcp25_723 = -(ROcp25_46*S23-C23*S5);
    ROcp25_823 = -(ROcp25_56*S23-ROcp25_85*C23);
    ROcp25_923 = -(ROcp25_66*S23-ROcp25_95*C23);
    ROcp25_124 = ROcp25_16*C24-ROcp25_723*S24;
    ROcp25_224 = ROcp25_26*C24-ROcp25_823*S24;
    ROcp25_324 = ROcp25_36*C24-ROcp25_923*S24;
    ROcp25_724 = ROcp25_16*S24+ROcp25_723*C24;
    ROcp25_824 = ROcp25_26*S24+ROcp25_823*C24;
    ROcp25_924 = ROcp25_36*S24+ROcp25_923*C24;
    ROcp25_125 = ROcp25_124*C25+ROcp25_423*S25;
    ROcp25_225 = ROcp25_224*C25+ROcp25_523*S25;
    ROcp25_325 = ROcp25_324*C25+ROcp25_623*S25;
    ROcp25_425 = -(ROcp25_124*S25-ROcp25_423*C25);
    ROcp25_525 = -(ROcp25_224*S25-ROcp25_523*C25);
    ROcp25_625 = -(ROcp25_324*S25-ROcp25_623*C25);
    RLcp25_123 = ROcp25_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp25_223 = ROcp25_26*s->dpt[1][4]+ROcp25_85*s->dpt[3][4];
    RLcp25_323 = ROcp25_36*s->dpt[1][4]+ROcp25_95*s->dpt[3][4];
    OMcp25_124 = OMcp25_16+ROcp25_16*qd[23]+ROcp25_423*qd[24];
    OMcp25_224 = OMcp25_26+ROcp25_26*qd[23]+ROcp25_523*qd[24];
    OMcp25_324 = OMcp25_36+ROcp25_36*qd[23]+ROcp25_623*qd[24];
    RLcp25_125 = ROcp25_724*s->dpt[3][44];
    RLcp25_225 = ROcp25_824*s->dpt[3][44];
    RLcp25_325 = ROcp25_924*s->dpt[3][44];
    OMcp25_125 = OMcp25_124+ROcp25_724*qd[25];
    OMcp25_225 = OMcp25_224+ROcp25_824*qd[25];
    OMcp25_325 = OMcp25_324+ROcp25_924*qd[25];

// = = Block_1_0_0_26_0_10 = = 
 
// Sensor Kinematics 


    ROcp25_130 = ROcp25_125*C30-ROcp25_724*S30;
    ROcp25_230 = ROcp25_225*C30-ROcp25_824*S30;
    ROcp25_330 = ROcp25_325*C30-ROcp25_924*S30;
    ROcp25_730 = ROcp25_125*S30+ROcp25_724*C30;
    ROcp25_830 = ROcp25_225*S30+ROcp25_824*C30;
    ROcp25_930 = ROcp25_325*S30+ROcp25_924*C30;
    ROcp25_431 = ROcp25_425*C31+ROcp25_730*S31;
    ROcp25_531 = ROcp25_525*C31+ROcp25_830*S31;
    ROcp25_631 = ROcp25_625*C31+ROcp25_930*S31;
    ROcp25_731 = -(ROcp25_425*S31-ROcp25_730*C31);
    ROcp25_831 = -(ROcp25_525*S31-ROcp25_830*C31);
    ROcp25_931 = -(ROcp25_625*S31-ROcp25_930*C31);
    ROcp25_132 = ROcp25_130*C32+ROcp25_431*S32;
    ROcp25_232 = ROcp25_230*C32+ROcp25_531*S32;
    ROcp25_332 = ROcp25_330*C32+ROcp25_631*S32;
    ROcp25_432 = -(ROcp25_130*S32-ROcp25_431*C32);
    ROcp25_532 = -(ROcp25_230*S32-ROcp25_531*C32);
    ROcp25_632 = -(ROcp25_330*S32-ROcp25_631*C32);
    RLcp25_130 = ROcp25_125*s->dpt[1][49]+ROcp25_425*s->dpt[2][49]+ROcp25_724*s->dpt[3][49];
    RLcp25_230 = ROcp25_225*s->dpt[1][49]+ROcp25_525*s->dpt[2][49]+ROcp25_824*s->dpt[3][49];
    RLcp25_330 = ROcp25_325*s->dpt[1][49]+ROcp25_625*s->dpt[2][49]+ROcp25_924*s->dpt[3][49];
    OMcp25_130 = OMcp25_125+ROcp25_425*qd[30];
    OMcp25_230 = OMcp25_225+ROcp25_525*qd[30];
    OMcp25_330 = OMcp25_325+ROcp25_625*qd[30];
    RLcp25_131 = ROcp25_425*s->dpt[2][60];
    RLcp25_231 = ROcp25_525*s->dpt[2][60];
    RLcp25_331 = ROcp25_625*s->dpt[2][60];
    OMcp25_131 = OMcp25_130+ROcp25_130*qd[31];
    OMcp25_231 = OMcp25_230+ROcp25_230*qd[31];
    OMcp25_331 = OMcp25_330+ROcp25_330*qd[31];
    RLcp25_132 = ROcp25_731*s->dpt[3][62];
    RLcp25_232 = ROcp25_831*s->dpt[3][62];
    RLcp25_332 = ROcp25_931*s->dpt[3][62];
    OMcp25_132 = OMcp25_131+ROcp25_731*qd[32];
    OMcp25_232 = OMcp25_231+ROcp25_831*qd[32];
    OMcp25_332 = OMcp25_331+ROcp25_931*qd[32];
    RLcp25_133 = ROcp25_731*s->dpt[3][65];
    RLcp25_233 = ROcp25_831*s->dpt[3][65];
    RLcp25_333 = ROcp25_931*s->dpt[3][65];
    OMcp25_133 = OMcp25_132+ROcp25_432*qd[33];
    OMcp25_233 = OMcp25_232+ROcp25_532*qd[33];
    OMcp25_333 = OMcp25_332+ROcp25_632*qd[33];
    RLcp25_167 = ROcp25_432*s->dpt[2][67]+s->dpt[1][67]*(ROcp25_132*C33-ROcp25_731*S33)+s->dpt[3][67]*(ROcp25_132*S33+
 ROcp25_731*C33);
    RLcp25_267 = ROcp25_532*s->dpt[2][67]+s->dpt[1][67]*(ROcp25_232*C33-ROcp25_831*S33)+s->dpt[3][67]*(ROcp25_232*S33+
 ROcp25_831*C33);
    RLcp25_367 = ROcp25_632*s->dpt[2][67]+s->dpt[1][67]*(ROcp25_332*C33-ROcp25_931*S33)+s->dpt[3][67]*(ROcp25_332*S33+
 ROcp25_931*C33);
    POcp25_167 = RLcp25_123+RLcp25_125+RLcp25_130+RLcp25_131+RLcp25_132+RLcp25_133+RLcp25_167+q[1];
    POcp25_267 = RLcp25_223+RLcp25_225+RLcp25_230+RLcp25_231+RLcp25_232+RLcp25_233+RLcp25_267+q[2];
    POcp25_367 = RLcp25_323+RLcp25_325+RLcp25_330+RLcp25_331+RLcp25_332+RLcp25_333+RLcp25_367+q[3];
    VIcp25_167 = qd[1]+OMcp25_224*RLcp25_325+OMcp25_225*RLcp25_330+OMcp25_230*RLcp25_331+OMcp25_231*RLcp25_332+OMcp25_232*
 RLcp25_333+OMcp25_233*RLcp25_367+OMcp25_26*RLcp25_323-OMcp25_324*RLcp25_225-OMcp25_325*RLcp25_230-OMcp25_330*RLcp25_231-
 OMcp25_331*RLcp25_232-OMcp25_332*RLcp25_233-OMcp25_333*RLcp25_267-OMcp25_36*RLcp25_223;
    VIcp25_267 = qd[2]-OMcp25_124*RLcp25_325-OMcp25_125*RLcp25_330-OMcp25_130*RLcp25_331-OMcp25_131*RLcp25_332-OMcp25_132*
 RLcp25_333-OMcp25_133*RLcp25_367-OMcp25_16*RLcp25_323+OMcp25_324*RLcp25_125+OMcp25_325*RLcp25_130+OMcp25_330*RLcp25_131+
 OMcp25_331*RLcp25_132+OMcp25_332*RLcp25_133+OMcp25_333*RLcp25_167+OMcp25_36*RLcp25_123;
    VIcp25_367 = qd[3]+OMcp25_124*RLcp25_225+OMcp25_125*RLcp25_230+OMcp25_130*RLcp25_231+OMcp25_131*RLcp25_232+OMcp25_132*
 RLcp25_233+OMcp25_133*RLcp25_267+OMcp25_16*RLcp25_223-OMcp25_224*RLcp25_125-OMcp25_225*RLcp25_130-OMcp25_230*RLcp25_131-
 OMcp25_231*RLcp25_132-OMcp25_232*RLcp25_133-OMcp25_233*RLcp25_167-OMcp25_26*RLcp25_123;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_167;
    sens->P[2] = POcp25_267;
    sens->P[3] = POcp25_367;
    sens->V[1] = VIcp25_167;
    sens->V[2] = VIcp25_267;
    sens->V[3] = VIcp25_367;
    sens->OM[1] = OMcp25_133;
    sens->OM[2] = OMcp25_233;
    sens->OM[3] = OMcp25_333;
 
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

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = ROcp26_16;
    sens->R[1][2] = ROcp26_26;
    sens->R[1][3] = ROcp26_36;
    sens->R[2][1] = ROcp26_46;
    sens->R[2][2] = ROcp26_56;
    sens->R[2][3] = ROcp26_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp26_85;
    sens->R[3][3] = ROcp26_95;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = OMcp26_16;
    sens->OM[2] = OMcp26_26;
    sens->OM[3] = OMcp26_36;
 
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
    POcp27_19 = RLcp27_17+RLcp27_18+RLcp27_19+q[1];
    POcp27_29 = RLcp27_27+RLcp27_28+RLcp27_29+q[2];
    POcp27_39 = RLcp27_37+RLcp27_38+RLcp27_39+q[3];
    OMcp27_19 = OMcp27_18+ROcp27_78*qd[9];
    OMcp27_29 = OMcp27_28+ROcp27_88*qd[9];
    OMcp27_39 = OMcp27_38+ROcp27_98*qd[9];
    VIcp27_19 = qd[1]+OMcp27_26*RLcp27_37+OMcp27_27*RLcp27_38+OMcp27_28*RLcp27_39-OMcp27_36*RLcp27_27-OMcp27_37*RLcp27_28-
 OMcp27_38*RLcp27_29;
    VIcp27_29 = qd[2]-OMcp27_16*RLcp27_37-OMcp27_17*RLcp27_38-OMcp27_18*RLcp27_39+OMcp27_36*RLcp27_17+OMcp27_37*RLcp27_18+
 OMcp27_38*RLcp27_19;
    VIcp27_39 = qd[3]+OMcp27_16*RLcp27_27+OMcp27_17*RLcp27_28+OMcp27_18*RLcp27_29-OMcp27_26*RLcp27_17-OMcp27_27*RLcp27_18-
 OMcp27_28*RLcp27_19;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_19;
    sens->P[2] = POcp27_29;
    sens->P[3] = POcp27_39;
    sens->R[1][1] = ROcp27_19;
    sens->R[1][2] = ROcp27_29;
    sens->R[1][3] = ROcp27_39;
    sens->R[2][1] = ROcp27_49;
    sens->R[2][2] = ROcp27_59;
    sens->R[2][3] = ROcp27_69;
    sens->R[3][1] = ROcp27_78;
    sens->R[3][2] = ROcp27_88;
    sens->R[3][3] = ROcp27_98;
    sens->V[1] = VIcp27_19;
    sens->V[2] = VIcp27_29;
    sens->V[3] = VIcp27_39;
    sens->OM[1] = OMcp27_19;
    sens->OM[2] = OMcp27_29;
    sens->OM[3] = OMcp27_39;
 
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
    OMcp28_16 = qd[4]+qd[6]*S5;
    OMcp28_26 = ROcp28_85*qd[6]+qd[5]*C4;
    OMcp28_36 = ROcp28_95*qd[6]+qd[5]*S4;

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
    RLcp28_17 = ROcp28_46*s->dpt[2][2];
    RLcp28_27 = ROcp28_56*s->dpt[2][2];
    RLcp28_37 = ROcp28_66*s->dpt[2][2];
    OMcp28_17 = OMcp28_16+ROcp28_46*qd[7];
    OMcp28_27 = OMcp28_26+ROcp28_56*qd[7];
    OMcp28_37 = OMcp28_36+ROcp28_66*qd[7];
    RLcp28_18 = ROcp28_46*s->dpt[2][6];
    RLcp28_28 = ROcp28_56*s->dpt[2][6];
    RLcp28_38 = ROcp28_66*s->dpt[2][6];
    OMcp28_18 = OMcp28_17+ROcp28_17*qd[8];
    OMcp28_28 = OMcp28_27+ROcp28_27*qd[8];
    OMcp28_38 = OMcp28_37+ROcp28_37*qd[8];
    RLcp28_19 = ROcp28_78*s->dpt[3][8];
    RLcp28_29 = ROcp28_88*s->dpt[3][8];
    RLcp28_39 = ROcp28_98*s->dpt[3][8];
    OMcp28_19 = OMcp28_18+ROcp28_78*qd[9];
    OMcp28_29 = OMcp28_28+ROcp28_88*qd[9];
    OMcp28_39 = OMcp28_38+ROcp28_98*qd[9];
    RLcp28_110 = ROcp28_78*s->dpt[3][11];
    RLcp28_210 = ROcp28_88*s->dpt[3][11];
    RLcp28_310 = ROcp28_98*s->dpt[3][11];
    POcp28_110 = RLcp28_110+RLcp28_17+RLcp28_18+RLcp28_19+q[1];
    POcp28_210 = RLcp28_210+RLcp28_27+RLcp28_28+RLcp28_29+q[2];
    POcp28_310 = RLcp28_310+RLcp28_37+RLcp28_38+RLcp28_39+q[3];
    OMcp28_110 = OMcp28_19+ROcp28_49*qd[10];
    OMcp28_210 = OMcp28_29+ROcp28_59*qd[10];
    OMcp28_310 = OMcp28_39+ROcp28_69*qd[10];
    VIcp28_110 = qd[1]+OMcp28_26*RLcp28_37+OMcp28_27*RLcp28_38+OMcp28_28*RLcp28_39+OMcp28_29*RLcp28_310-OMcp28_36*
 RLcp28_27-OMcp28_37*RLcp28_28-OMcp28_38*RLcp28_29-OMcp28_39*RLcp28_210;
    VIcp28_210 = qd[2]-OMcp28_16*RLcp28_37-OMcp28_17*RLcp28_38-OMcp28_18*RLcp28_39-OMcp28_19*RLcp28_310+OMcp28_36*
 RLcp28_17+OMcp28_37*RLcp28_18+OMcp28_38*RLcp28_19+OMcp28_39*RLcp28_110;
    VIcp28_310 = qd[3]+OMcp28_16*RLcp28_27+OMcp28_17*RLcp28_28+OMcp28_18*RLcp28_29+OMcp28_19*RLcp28_210-OMcp28_26*
 RLcp28_17-OMcp28_27*RLcp28_18-OMcp28_28*RLcp28_19-OMcp28_29*RLcp28_110;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_110;
    sens->P[2] = POcp28_210;
    sens->P[3] = POcp28_310;
    sens->R[1][1] = ROcp28_110;
    sens->R[1][2] = ROcp28_210;
    sens->R[1][3] = ROcp28_310;
    sens->R[2][1] = ROcp28_49;
    sens->R[2][2] = ROcp28_59;
    sens->R[2][3] = ROcp28_69;
    sens->R[3][1] = ROcp28_710;
    sens->R[3][2] = ROcp28_810;
    sens->R[3][3] = ROcp28_910;
    sens->V[1] = VIcp28_110;
    sens->V[2] = VIcp28_210;
    sens->V[3] = VIcp28_310;
    sens->OM[1] = OMcp28_110;
    sens->OM[2] = OMcp28_210;
    sens->OM[3] = OMcp28_310;
 
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

// = = Block_1_0_0_30_0_2 = = 
 
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
    OMcp29_17 = OMcp29_16+ROcp29_46*qd[7];
    OMcp29_27 = OMcp29_26+ROcp29_56*qd[7];
    OMcp29_37 = OMcp29_36+ROcp29_66*qd[7];
    RLcp29_18 = ROcp29_46*s->dpt[2][6];
    RLcp29_28 = ROcp29_56*s->dpt[2][6];
    RLcp29_38 = ROcp29_66*s->dpt[2][6];
    OMcp29_18 = OMcp29_17+ROcp29_17*qd[8];
    OMcp29_28 = OMcp29_27+ROcp29_27*qd[8];
    OMcp29_38 = OMcp29_37+ROcp29_37*qd[8];
    RLcp29_19 = ROcp29_78*s->dpt[3][8];
    RLcp29_29 = ROcp29_88*s->dpt[3][8];
    RLcp29_39 = ROcp29_98*s->dpt[3][8];
    OMcp29_19 = OMcp29_18+ROcp29_78*qd[9];
    OMcp29_29 = OMcp29_28+ROcp29_88*qd[9];
    OMcp29_39 = OMcp29_38+ROcp29_98*qd[9];
    RLcp29_110 = ROcp29_78*s->dpt[3][11];
    RLcp29_210 = ROcp29_88*s->dpt[3][11];
    RLcp29_310 = ROcp29_98*s->dpt[3][11];
    OMcp29_110 = OMcp29_19+ROcp29_49*qd[10];
    OMcp29_210 = OMcp29_29+ROcp29_59*qd[10];
    OMcp29_310 = OMcp29_39+ROcp29_69*qd[10];
    RLcp29_111 = ROcp29_710*s->dpt[3][14];
    RLcp29_211 = ROcp29_810*s->dpt[3][14];
    RLcp29_311 = ROcp29_910*s->dpt[3][14];
    OMcp29_112 = OMcp29_110+ROcp29_110*qd[11]+ROcp29_411*qd[12];
    OMcp29_212 = OMcp29_210+ROcp29_210*qd[11]+ROcp29_511*qd[12];
    OMcp29_312 = OMcp29_310+ROcp29_310*qd[11]+ROcp29_611*qd[12];

// = = Block_1_0_0_30_0_3 = = 
 
// Sensor Kinematics 


    RLcp29_113 = Dz133*ROcp29_712+ROcp29_112*s->dpt[1][20];
    RLcp29_213 = Dz133*ROcp29_812+ROcp29_212*s->dpt[1][20];
    RLcp29_313 = Dz133*ROcp29_912+ROcp29_312*s->dpt[1][20];
    POcp29_113 = RLcp29_110+RLcp29_111+RLcp29_113+RLcp29_17+RLcp29_18+RLcp29_19+q[1];
    POcp29_213 = RLcp29_210+RLcp29_211+RLcp29_213+RLcp29_27+RLcp29_28+RLcp29_29+q[2];
    POcp29_313 = RLcp29_310+RLcp29_311+RLcp29_313+RLcp29_37+RLcp29_38+RLcp29_39+q[3];
    VIcp29_113 = qd[1]+OMcp29_210*RLcp29_311+OMcp29_212*RLcp29_313+OMcp29_26*RLcp29_37+OMcp29_27*RLcp29_38+OMcp29_28*
 RLcp29_39+OMcp29_29*RLcp29_310-OMcp29_310*RLcp29_211-OMcp29_312*RLcp29_213-OMcp29_36*RLcp29_27-OMcp29_37*RLcp29_28-OMcp29_38
 *RLcp29_29-OMcp29_39*RLcp29_210+ROcp29_712*qd[13];
    VIcp29_213 = qd[2]-OMcp29_110*RLcp29_311-OMcp29_112*RLcp29_313-OMcp29_16*RLcp29_37-OMcp29_17*RLcp29_38-OMcp29_18*
 RLcp29_39-OMcp29_19*RLcp29_310+OMcp29_310*RLcp29_111+OMcp29_312*RLcp29_113+OMcp29_36*RLcp29_17+OMcp29_37*RLcp29_18+OMcp29_38
 *RLcp29_19+OMcp29_39*RLcp29_110+ROcp29_812*qd[13];
    VIcp29_313 = qd[3]+OMcp29_110*RLcp29_211+OMcp29_112*RLcp29_213+OMcp29_16*RLcp29_27+OMcp29_17*RLcp29_28+OMcp29_18*
 RLcp29_29+OMcp29_19*RLcp29_210-OMcp29_210*RLcp29_111-OMcp29_212*RLcp29_113-OMcp29_26*RLcp29_17-OMcp29_27*RLcp29_18-OMcp29_28
 *RLcp29_19-OMcp29_29*RLcp29_110+ROcp29_912*qd[13];

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_113;
    sens->P[2] = POcp29_213;
    sens->P[3] = POcp29_313;
    sens->R[1][1] = ROcp29_112;
    sens->R[1][2] = ROcp29_212;
    sens->R[1][3] = ROcp29_312;
    sens->R[2][1] = ROcp29_411;
    sens->R[2][2] = ROcp29_511;
    sens->R[2][3] = ROcp29_611;
    sens->R[3][1] = ROcp29_712;
    sens->R[3][2] = ROcp29_812;
    sens->R[3][3] = ROcp29_912;
    sens->V[1] = VIcp29_113;
    sens->V[2] = VIcp29_213;
    sens->V[3] = VIcp29_313;
    sens->OM[1] = OMcp29_112;
    sens->OM[2] = OMcp29_212;
    sens->OM[3] = OMcp29_312;
 
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

// = = Block_1_0_0_31_0_2 = = 
 
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
    OMcp30_17 = OMcp30_16+ROcp30_46*qd[7];
    OMcp30_27 = OMcp30_26+ROcp30_56*qd[7];
    OMcp30_37 = OMcp30_36+ROcp30_66*qd[7];
    RLcp30_18 = ROcp30_46*s->dpt[2][6];
    RLcp30_28 = ROcp30_56*s->dpt[2][6];
    RLcp30_38 = ROcp30_66*s->dpt[2][6];
    OMcp30_18 = OMcp30_17+ROcp30_17*qd[8];
    OMcp30_28 = OMcp30_27+ROcp30_27*qd[8];
    OMcp30_38 = OMcp30_37+ROcp30_37*qd[8];
    RLcp30_19 = ROcp30_78*s->dpt[3][8];
    RLcp30_29 = ROcp30_88*s->dpt[3][8];
    RLcp30_39 = ROcp30_98*s->dpt[3][8];
    OMcp30_19 = OMcp30_18+ROcp30_78*qd[9];
    OMcp30_29 = OMcp30_28+ROcp30_88*qd[9];
    OMcp30_39 = OMcp30_38+ROcp30_98*qd[9];
    RLcp30_110 = ROcp30_78*s->dpt[3][11];
    RLcp30_210 = ROcp30_88*s->dpt[3][11];
    RLcp30_310 = ROcp30_98*s->dpt[3][11];
    OMcp30_110 = OMcp30_19+ROcp30_49*qd[10];
    OMcp30_210 = OMcp30_29+ROcp30_59*qd[10];
    OMcp30_310 = OMcp30_39+ROcp30_69*qd[10];
    RLcp30_111 = ROcp30_710*s->dpt[3][14];
    RLcp30_211 = ROcp30_810*s->dpt[3][14];
    RLcp30_311 = ROcp30_910*s->dpt[3][14];
    OMcp30_112 = OMcp30_110+ROcp30_110*qd[11]+ROcp30_411*qd[12];
    OMcp30_212 = OMcp30_210+ROcp30_210*qd[11]+ROcp30_511*qd[12];
    OMcp30_312 = OMcp30_310+ROcp30_310*qd[11]+ROcp30_611*qd[12];

// = = Block_1_0_0_31_0_4 = = 
 
// Sensor Kinematics 


    RLcp30_114 = Dz143*ROcp30_712+ROcp30_112*s->dpt[1][21];
    RLcp30_214 = Dz143*ROcp30_812+ROcp30_212*s->dpt[1][21];
    RLcp30_314 = Dz143*ROcp30_912+ROcp30_312*s->dpt[1][21];
    POcp30_114 = RLcp30_110+RLcp30_111+RLcp30_114+RLcp30_17+RLcp30_18+RLcp30_19+q[1];
    POcp30_214 = RLcp30_210+RLcp30_211+RLcp30_214+RLcp30_27+RLcp30_28+RLcp30_29+q[2];
    POcp30_314 = RLcp30_310+RLcp30_311+RLcp30_314+RLcp30_37+RLcp30_38+RLcp30_39+q[3];
    VIcp30_114 = qd[1]+OMcp30_210*RLcp30_311+OMcp30_212*RLcp30_314+OMcp30_26*RLcp30_37+OMcp30_27*RLcp30_38+OMcp30_28*
 RLcp30_39+OMcp30_29*RLcp30_310-OMcp30_310*RLcp30_211-OMcp30_312*RLcp30_214-OMcp30_36*RLcp30_27-OMcp30_37*RLcp30_28-OMcp30_38
 *RLcp30_29-OMcp30_39*RLcp30_210+ROcp30_712*qd[14];
    VIcp30_214 = qd[2]-OMcp30_110*RLcp30_311-OMcp30_112*RLcp30_314-OMcp30_16*RLcp30_37-OMcp30_17*RLcp30_38-OMcp30_18*
 RLcp30_39-OMcp30_19*RLcp30_310+OMcp30_310*RLcp30_111+OMcp30_312*RLcp30_114+OMcp30_36*RLcp30_17+OMcp30_37*RLcp30_18+OMcp30_38
 *RLcp30_19+OMcp30_39*RLcp30_110+ROcp30_812*qd[14];
    VIcp30_314 = qd[3]+OMcp30_110*RLcp30_211+OMcp30_112*RLcp30_214+OMcp30_16*RLcp30_27+OMcp30_17*RLcp30_28+OMcp30_18*
 RLcp30_29+OMcp30_19*RLcp30_210-OMcp30_210*RLcp30_111-OMcp30_212*RLcp30_114-OMcp30_26*RLcp30_17-OMcp30_27*RLcp30_18-OMcp30_28
 *RLcp30_19-OMcp30_29*RLcp30_110+ROcp30_912*qd[14];

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_114;
    sens->P[2] = POcp30_214;
    sens->P[3] = POcp30_314;
    sens->R[1][1] = ROcp30_112;
    sens->R[1][2] = ROcp30_212;
    sens->R[1][3] = ROcp30_312;
    sens->R[2][1] = ROcp30_411;
    sens->R[2][2] = ROcp30_511;
    sens->R[2][3] = ROcp30_611;
    sens->R[3][1] = ROcp30_712;
    sens->R[3][2] = ROcp30_812;
    sens->R[3][3] = ROcp30_912;
    sens->V[1] = VIcp30_114;
    sens->V[2] = VIcp30_214;
    sens->V[3] = VIcp30_314;
    sens->OM[1] = OMcp30_112;
    sens->OM[2] = OMcp30_212;
    sens->OM[3] = OMcp30_312;
 
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

// = = Block_1_0_0_32_0_5 = = 
 
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
    OMcp31_115 = OMcp31_16+ROcp31_46*qd[15];
    OMcp31_215 = OMcp31_26+ROcp31_56*qd[15];
    OMcp31_315 = OMcp31_36+ROcp31_66*qd[15];
    RLcp31_116 = ROcp31_46*s->dpt[2][24];
    RLcp31_216 = ROcp31_56*s->dpt[2][24];
    RLcp31_316 = ROcp31_66*s->dpt[2][24];
    OMcp31_116 = OMcp31_115+ROcp31_115*qd[16];
    OMcp31_216 = OMcp31_215+ROcp31_215*qd[16];
    OMcp31_316 = OMcp31_315+ROcp31_315*qd[16];
    RLcp31_117 = ROcp31_716*s->dpt[3][26];
    RLcp31_217 = ROcp31_816*s->dpt[3][26];
    RLcp31_317 = ROcp31_916*s->dpt[3][26];
    POcp31_117 = RLcp31_115+RLcp31_116+RLcp31_117+q[1];
    POcp31_217 = RLcp31_215+RLcp31_216+RLcp31_217+q[2];
    POcp31_317 = RLcp31_315+RLcp31_316+RLcp31_317+q[3];
    OMcp31_117 = OMcp31_116+ROcp31_716*qd[17];
    OMcp31_217 = OMcp31_216+ROcp31_816*qd[17];
    OMcp31_317 = OMcp31_316+ROcp31_916*qd[17];
    VIcp31_117 = qd[1]+OMcp31_215*RLcp31_316+OMcp31_216*RLcp31_317+OMcp31_26*RLcp31_315-OMcp31_315*RLcp31_216-OMcp31_316*
 RLcp31_217-OMcp31_36*RLcp31_215;
    VIcp31_217 = qd[2]-OMcp31_115*RLcp31_316-OMcp31_116*RLcp31_317-OMcp31_16*RLcp31_315+OMcp31_315*RLcp31_116+OMcp31_316*
 RLcp31_117+OMcp31_36*RLcp31_115;
    VIcp31_317 = qd[3]+OMcp31_115*RLcp31_216+OMcp31_116*RLcp31_217+OMcp31_16*RLcp31_215-OMcp31_215*RLcp31_116-OMcp31_216*
 RLcp31_117-OMcp31_26*RLcp31_115;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp31_117;
    sens->P[2] = POcp31_217;
    sens->P[3] = POcp31_317;
    sens->R[1][1] = ROcp31_117;
    sens->R[1][2] = ROcp31_217;
    sens->R[1][3] = ROcp31_317;
    sens->R[2][1] = ROcp31_417;
    sens->R[2][2] = ROcp31_517;
    sens->R[2][3] = ROcp31_617;
    sens->R[3][1] = ROcp31_716;
    sens->R[3][2] = ROcp31_816;
    sens->R[3][3] = ROcp31_916;
    sens->V[1] = VIcp31_117;
    sens->V[2] = VIcp31_217;
    sens->V[3] = VIcp31_317;
    sens->OM[1] = OMcp31_117;
    sens->OM[2] = OMcp31_217;
    sens->OM[3] = OMcp31_317;
 
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
    OMcp32_16 = qd[4]+qd[6]*S5;
    OMcp32_26 = ROcp32_85*qd[6]+qd[5]*C4;
    OMcp32_36 = ROcp32_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_33_0_5 = = 
 
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
    OMcp32_115 = OMcp32_16+ROcp32_46*qd[15];
    OMcp32_215 = OMcp32_26+ROcp32_56*qd[15];
    OMcp32_315 = OMcp32_36+ROcp32_66*qd[15];
    RLcp32_116 = ROcp32_46*s->dpt[2][24];
    RLcp32_216 = ROcp32_56*s->dpt[2][24];
    RLcp32_316 = ROcp32_66*s->dpt[2][24];
    OMcp32_116 = OMcp32_115+ROcp32_115*qd[16];
    OMcp32_216 = OMcp32_215+ROcp32_215*qd[16];
    OMcp32_316 = OMcp32_315+ROcp32_315*qd[16];
    RLcp32_117 = ROcp32_716*s->dpt[3][26];
    RLcp32_217 = ROcp32_816*s->dpt[3][26];
    RLcp32_317 = ROcp32_916*s->dpt[3][26];
    OMcp32_117 = OMcp32_116+ROcp32_716*qd[17];
    OMcp32_217 = OMcp32_216+ROcp32_816*qd[17];
    OMcp32_317 = OMcp32_316+ROcp32_916*qd[17];
    RLcp32_118 = ROcp32_716*s->dpt[3][29];
    RLcp32_218 = ROcp32_816*s->dpt[3][29];
    RLcp32_318 = ROcp32_916*s->dpt[3][29];
    POcp32_118 = RLcp32_115+RLcp32_116+RLcp32_117+RLcp32_118+q[1];
    POcp32_218 = RLcp32_215+RLcp32_216+RLcp32_217+RLcp32_218+q[2];
    POcp32_318 = RLcp32_315+RLcp32_316+RLcp32_317+RLcp32_318+q[3];
    OMcp32_118 = OMcp32_117+ROcp32_417*qd[18];
    OMcp32_218 = OMcp32_217+ROcp32_517*qd[18];
    OMcp32_318 = OMcp32_317+ROcp32_617*qd[18];
    VIcp32_118 = qd[1]+OMcp32_215*RLcp32_316+OMcp32_216*RLcp32_317+OMcp32_217*RLcp32_318+OMcp32_26*RLcp32_315-OMcp32_315*
 RLcp32_216-OMcp32_316*RLcp32_217-OMcp32_317*RLcp32_218-OMcp32_36*RLcp32_215;
    VIcp32_218 = qd[2]-OMcp32_115*RLcp32_316-OMcp32_116*RLcp32_317-OMcp32_117*RLcp32_318-OMcp32_16*RLcp32_315+OMcp32_315*
 RLcp32_116+OMcp32_316*RLcp32_117+OMcp32_317*RLcp32_118+OMcp32_36*RLcp32_115;
    VIcp32_318 = qd[3]+OMcp32_115*RLcp32_216+OMcp32_116*RLcp32_217+OMcp32_117*RLcp32_218+OMcp32_16*RLcp32_215-OMcp32_215*
 RLcp32_116-OMcp32_216*RLcp32_117-OMcp32_217*RLcp32_118-OMcp32_26*RLcp32_115;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp32_118;
    sens->P[2] = POcp32_218;
    sens->P[3] = POcp32_318;
    sens->R[1][1] = ROcp32_118;
    sens->R[1][2] = ROcp32_218;
    sens->R[1][3] = ROcp32_318;
    sens->R[2][1] = ROcp32_417;
    sens->R[2][2] = ROcp32_517;
    sens->R[2][3] = ROcp32_617;
    sens->R[3][1] = ROcp32_718;
    sens->R[3][2] = ROcp32_818;
    sens->R[3][3] = ROcp32_918;
    sens->V[1] = VIcp32_118;
    sens->V[2] = VIcp32_218;
    sens->V[3] = VIcp32_318;
    sens->OM[1] = OMcp32_118;
    sens->OM[2] = OMcp32_218;
    sens->OM[3] = OMcp32_318;
 
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

// = = Block_1_0_0_34_0_5 = = 
 
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
    OMcp33_115 = OMcp33_16+ROcp33_46*qd[15];
    OMcp33_215 = OMcp33_26+ROcp33_56*qd[15];
    OMcp33_315 = OMcp33_36+ROcp33_66*qd[15];
    RLcp33_116 = ROcp33_46*s->dpt[2][24];
    RLcp33_216 = ROcp33_56*s->dpt[2][24];
    RLcp33_316 = ROcp33_66*s->dpt[2][24];
    OMcp33_116 = OMcp33_115+ROcp33_115*qd[16];
    OMcp33_216 = OMcp33_215+ROcp33_215*qd[16];
    OMcp33_316 = OMcp33_315+ROcp33_315*qd[16];
    RLcp33_117 = ROcp33_716*s->dpt[3][26];
    RLcp33_217 = ROcp33_816*s->dpt[3][26];
    RLcp33_317 = ROcp33_916*s->dpt[3][26];
    OMcp33_117 = OMcp33_116+ROcp33_716*qd[17];
    OMcp33_217 = OMcp33_216+ROcp33_816*qd[17];
    OMcp33_317 = OMcp33_316+ROcp33_916*qd[17];
    RLcp33_118 = ROcp33_716*s->dpt[3][29];
    RLcp33_218 = ROcp33_816*s->dpt[3][29];
    RLcp33_318 = ROcp33_916*s->dpt[3][29];
    OMcp33_118 = OMcp33_117+ROcp33_417*qd[18];
    OMcp33_218 = OMcp33_217+ROcp33_517*qd[18];
    OMcp33_318 = OMcp33_317+ROcp33_617*qd[18];
    RLcp33_119 = ROcp33_718*s->dpt[3][32];
    RLcp33_219 = ROcp33_818*s->dpt[3][32];
    RLcp33_319 = ROcp33_918*s->dpt[3][32];
    OMcp33_120 = OMcp33_118+ROcp33_118*qd[19]+ROcp33_419*qd[20];
    OMcp33_220 = OMcp33_218+ROcp33_218*qd[19]+ROcp33_519*qd[20];
    OMcp33_320 = OMcp33_318+ROcp33_318*qd[19]+ROcp33_619*qd[20];

// = = Block_1_0_0_34_0_6 = = 
 
// Sensor Kinematics 


    RLcp33_121 = Dz213*ROcp33_720+ROcp33_120*s->dpt[1][38];
    RLcp33_221 = Dz213*ROcp33_820+ROcp33_220*s->dpt[1][38];
    RLcp33_321 = Dz213*ROcp33_920+ROcp33_320*s->dpt[1][38];
    POcp33_121 = RLcp33_115+RLcp33_116+RLcp33_117+RLcp33_118+RLcp33_119+RLcp33_121+q[1];
    POcp33_221 = RLcp33_215+RLcp33_216+RLcp33_217+RLcp33_218+RLcp33_219+RLcp33_221+q[2];
    POcp33_321 = RLcp33_315+RLcp33_316+RLcp33_317+RLcp33_318+RLcp33_319+RLcp33_321+q[3];
    VIcp33_121 = qd[1]+OMcp33_215*RLcp33_316+OMcp33_216*RLcp33_317+OMcp33_217*RLcp33_318+OMcp33_218*RLcp33_319+OMcp33_220*
 RLcp33_321+OMcp33_26*RLcp33_315-OMcp33_315*RLcp33_216-OMcp33_316*RLcp33_217-OMcp33_317*RLcp33_218-OMcp33_318*RLcp33_219-
 OMcp33_320*RLcp33_221-OMcp33_36*RLcp33_215+ROcp33_720*qd[21];
    VIcp33_221 = qd[2]-OMcp33_115*RLcp33_316-OMcp33_116*RLcp33_317-OMcp33_117*RLcp33_318-OMcp33_118*RLcp33_319-OMcp33_120*
 RLcp33_321-OMcp33_16*RLcp33_315+OMcp33_315*RLcp33_116+OMcp33_316*RLcp33_117+OMcp33_317*RLcp33_118+OMcp33_318*RLcp33_119+
 OMcp33_320*RLcp33_121+OMcp33_36*RLcp33_115+ROcp33_820*qd[21];
    VIcp33_321 = qd[3]+OMcp33_115*RLcp33_216+OMcp33_116*RLcp33_217+OMcp33_117*RLcp33_218+OMcp33_118*RLcp33_219+OMcp33_120*
 RLcp33_221+OMcp33_16*RLcp33_215-OMcp33_215*RLcp33_116-OMcp33_216*RLcp33_117-OMcp33_217*RLcp33_118-OMcp33_218*RLcp33_119-
 OMcp33_220*RLcp33_121-OMcp33_26*RLcp33_115+ROcp33_920*qd[21];

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp33_121;
    sens->P[2] = POcp33_221;
    sens->P[3] = POcp33_321;
    sens->R[1][1] = ROcp33_120;
    sens->R[1][2] = ROcp33_220;
    sens->R[1][3] = ROcp33_320;
    sens->R[2][1] = ROcp33_419;
    sens->R[2][2] = ROcp33_519;
    sens->R[2][3] = ROcp33_619;
    sens->R[3][1] = ROcp33_720;
    sens->R[3][2] = ROcp33_820;
    sens->R[3][3] = ROcp33_920;
    sens->V[1] = VIcp33_121;
    sens->V[2] = VIcp33_221;
    sens->V[3] = VIcp33_321;
    sens->OM[1] = OMcp33_120;
    sens->OM[2] = OMcp33_220;
    sens->OM[3] = OMcp33_320;
 
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

// = = Block_1_0_0_35_0_5 = = 
 
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
    OMcp34_115 = OMcp34_16+ROcp34_46*qd[15];
    OMcp34_215 = OMcp34_26+ROcp34_56*qd[15];
    OMcp34_315 = OMcp34_36+ROcp34_66*qd[15];
    RLcp34_116 = ROcp34_46*s->dpt[2][24];
    RLcp34_216 = ROcp34_56*s->dpt[2][24];
    RLcp34_316 = ROcp34_66*s->dpt[2][24];
    OMcp34_116 = OMcp34_115+ROcp34_115*qd[16];
    OMcp34_216 = OMcp34_215+ROcp34_215*qd[16];
    OMcp34_316 = OMcp34_315+ROcp34_315*qd[16];
    RLcp34_117 = ROcp34_716*s->dpt[3][26];
    RLcp34_217 = ROcp34_816*s->dpt[3][26];
    RLcp34_317 = ROcp34_916*s->dpt[3][26];
    OMcp34_117 = OMcp34_116+ROcp34_716*qd[17];
    OMcp34_217 = OMcp34_216+ROcp34_816*qd[17];
    OMcp34_317 = OMcp34_316+ROcp34_916*qd[17];
    RLcp34_118 = ROcp34_716*s->dpt[3][29];
    RLcp34_218 = ROcp34_816*s->dpt[3][29];
    RLcp34_318 = ROcp34_916*s->dpt[3][29];
    OMcp34_118 = OMcp34_117+ROcp34_417*qd[18];
    OMcp34_218 = OMcp34_217+ROcp34_517*qd[18];
    OMcp34_318 = OMcp34_317+ROcp34_617*qd[18];
    RLcp34_119 = ROcp34_718*s->dpt[3][32];
    RLcp34_219 = ROcp34_818*s->dpt[3][32];
    RLcp34_319 = ROcp34_918*s->dpt[3][32];
    OMcp34_120 = OMcp34_118+ROcp34_118*qd[19]+ROcp34_419*qd[20];
    OMcp34_220 = OMcp34_218+ROcp34_218*qd[19]+ROcp34_519*qd[20];
    OMcp34_320 = OMcp34_318+ROcp34_318*qd[19]+ROcp34_619*qd[20];

// = = Block_1_0_0_35_0_7 = = 
 
// Sensor Kinematics 


    RLcp34_122 = Dz223*ROcp34_720+ROcp34_120*s->dpt[1][39];
    RLcp34_222 = Dz223*ROcp34_820+ROcp34_220*s->dpt[1][39];
    RLcp34_322 = Dz223*ROcp34_920+ROcp34_320*s->dpt[1][39];
    POcp34_122 = RLcp34_115+RLcp34_116+RLcp34_117+RLcp34_118+RLcp34_119+RLcp34_122+q[1];
    POcp34_222 = RLcp34_215+RLcp34_216+RLcp34_217+RLcp34_218+RLcp34_219+RLcp34_222+q[2];
    POcp34_322 = RLcp34_315+RLcp34_316+RLcp34_317+RLcp34_318+RLcp34_319+RLcp34_322+q[3];
    VIcp34_122 = qd[1]+OMcp34_215*RLcp34_316+OMcp34_216*RLcp34_317+OMcp34_217*RLcp34_318+OMcp34_218*RLcp34_319+OMcp34_220*
 RLcp34_322+OMcp34_26*RLcp34_315-OMcp34_315*RLcp34_216-OMcp34_316*RLcp34_217-OMcp34_317*RLcp34_218-OMcp34_318*RLcp34_219-
 OMcp34_320*RLcp34_222-OMcp34_36*RLcp34_215+ROcp34_720*qd[22];
    VIcp34_222 = qd[2]-OMcp34_115*RLcp34_316-OMcp34_116*RLcp34_317-OMcp34_117*RLcp34_318-OMcp34_118*RLcp34_319-OMcp34_120*
 RLcp34_322-OMcp34_16*RLcp34_315+OMcp34_315*RLcp34_116+OMcp34_316*RLcp34_117+OMcp34_317*RLcp34_118+OMcp34_318*RLcp34_119+
 OMcp34_320*RLcp34_122+OMcp34_36*RLcp34_115+ROcp34_820*qd[22];
    VIcp34_322 = qd[3]+OMcp34_115*RLcp34_216+OMcp34_116*RLcp34_217+OMcp34_117*RLcp34_218+OMcp34_118*RLcp34_219+OMcp34_120*
 RLcp34_222+OMcp34_16*RLcp34_215-OMcp34_215*RLcp34_116-OMcp34_216*RLcp34_117-OMcp34_217*RLcp34_118-OMcp34_218*RLcp34_119-
 OMcp34_220*RLcp34_122-OMcp34_26*RLcp34_115+ROcp34_920*qd[22];

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp34_122;
    sens->P[2] = POcp34_222;
    sens->P[3] = POcp34_322;
    sens->R[1][1] = ROcp34_120;
    sens->R[1][2] = ROcp34_220;
    sens->R[1][3] = ROcp34_320;
    sens->R[2][1] = ROcp34_419;
    sens->R[2][2] = ROcp34_519;
    sens->R[2][3] = ROcp34_619;
    sens->R[3][1] = ROcp34_720;
    sens->R[3][2] = ROcp34_820;
    sens->R[3][3] = ROcp34_920;
    sens->V[1] = VIcp34_122;
    sens->V[2] = VIcp34_222;
    sens->V[3] = VIcp34_322;
    sens->OM[1] = OMcp34_120;
    sens->OM[2] = OMcp34_220;
    sens->OM[3] = OMcp34_320;
 
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

// = = Block_1_0_0_36_0_8 = = 
 
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
    OMcp35_124 = OMcp35_16+ROcp35_16*qd[23]+ROcp35_423*qd[24];
    OMcp35_224 = OMcp35_26+ROcp35_26*qd[23]+ROcp35_523*qd[24];
    OMcp35_324 = OMcp35_36+ROcp35_36*qd[23]+ROcp35_623*qd[24];
    RLcp35_125 = ROcp35_724*s->dpt[3][44];
    RLcp35_225 = ROcp35_824*s->dpt[3][44];
    RLcp35_325 = ROcp35_924*s->dpt[3][44];
    POcp35_125 = RLcp35_123+RLcp35_125+q[1];
    POcp35_225 = RLcp35_223+RLcp35_225+q[2];
    POcp35_325 = RLcp35_323+RLcp35_325+q[3];
    OMcp35_125 = OMcp35_124+ROcp35_724*qd[25];
    OMcp35_225 = OMcp35_224+ROcp35_824*qd[25];
    OMcp35_325 = OMcp35_324+ROcp35_924*qd[25];
    VIcp35_125 = qd[1]+OMcp35_224*RLcp35_325+OMcp35_26*RLcp35_323-OMcp35_324*RLcp35_225-OMcp35_36*RLcp35_223;
    VIcp35_225 = qd[2]-OMcp35_124*RLcp35_325-OMcp35_16*RLcp35_323+OMcp35_324*RLcp35_125+OMcp35_36*RLcp35_123;
    VIcp35_325 = qd[3]+OMcp35_124*RLcp35_225+OMcp35_16*RLcp35_223-OMcp35_224*RLcp35_125-OMcp35_26*RLcp35_123;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp35_125;
    sens->P[2] = POcp35_225;
    sens->P[3] = POcp35_325;
    sens->R[1][1] = ROcp35_125;
    sens->R[1][2] = ROcp35_225;
    sens->R[1][3] = ROcp35_325;
    sens->R[2][1] = ROcp35_425;
    sens->R[2][2] = ROcp35_525;
    sens->R[2][3] = ROcp35_625;
    sens->R[3][1] = ROcp35_724;
    sens->R[3][2] = ROcp35_824;
    sens->R[3][3] = ROcp35_924;
    sens->V[1] = VIcp35_125;
    sens->V[2] = VIcp35_225;
    sens->V[3] = VIcp35_325;
    sens->OM[1] = OMcp35_125;
    sens->OM[2] = OMcp35_225;
    sens->OM[3] = OMcp35_325;
 
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

// = = Block_1_0_0_37_0_8 = = 
 
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
    OMcp36_124 = OMcp36_16+ROcp36_16*qd[23]+ROcp36_423*qd[24];
    OMcp36_224 = OMcp36_26+ROcp36_26*qd[23]+ROcp36_523*qd[24];
    OMcp36_324 = OMcp36_36+ROcp36_36*qd[23]+ROcp36_623*qd[24];
    RLcp36_125 = ROcp36_724*s->dpt[3][44];
    RLcp36_225 = ROcp36_824*s->dpt[3][44];
    RLcp36_325 = ROcp36_924*s->dpt[3][44];
    OMcp36_125 = OMcp36_124+ROcp36_724*qd[25];
    OMcp36_225 = OMcp36_224+ROcp36_824*qd[25];
    OMcp36_325 = OMcp36_324+ROcp36_924*qd[25];

// = = Block_1_0_0_37_0_9 = = 
 
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
    OMcp36_126 = OMcp36_125+ROcp36_425*qd[26];
    OMcp36_226 = OMcp36_225+ROcp36_525*qd[26];
    OMcp36_326 = OMcp36_325+ROcp36_625*qd[26];
    RLcp36_127 = ROcp36_425*s->dpt[2][51];
    RLcp36_227 = ROcp36_525*s->dpt[2][51];
    RLcp36_327 = ROcp36_625*s->dpt[2][51];
    OMcp36_127 = OMcp36_126+ROcp36_126*qd[27];
    OMcp36_227 = OMcp36_226+ROcp36_226*qd[27];
    OMcp36_327 = OMcp36_326+ROcp36_326*qd[27];
    RLcp36_128 = ROcp36_727*s->dpt[3][53];
    RLcp36_228 = ROcp36_827*s->dpt[3][53];
    RLcp36_328 = ROcp36_927*s->dpt[3][53];
    POcp36_128 = RLcp36_123+RLcp36_125+RLcp36_126+RLcp36_127+RLcp36_128+q[1];
    POcp36_228 = RLcp36_223+RLcp36_225+RLcp36_226+RLcp36_227+RLcp36_228+q[2];
    POcp36_328 = RLcp36_323+RLcp36_325+RLcp36_326+RLcp36_327+RLcp36_328+q[3];
    OMcp36_128 = OMcp36_127+ROcp36_727*qd[28];
    OMcp36_228 = OMcp36_227+ROcp36_827*qd[28];
    OMcp36_328 = OMcp36_327+ROcp36_927*qd[28];
    VIcp36_128 = qd[1]+OMcp36_224*RLcp36_325+OMcp36_225*RLcp36_326+OMcp36_226*RLcp36_327+OMcp36_227*RLcp36_328+OMcp36_26*
 RLcp36_323-OMcp36_324*RLcp36_225-OMcp36_325*RLcp36_226-OMcp36_326*RLcp36_227-OMcp36_327*RLcp36_228-OMcp36_36*RLcp36_223;
    VIcp36_228 = qd[2]-OMcp36_124*RLcp36_325-OMcp36_125*RLcp36_326-OMcp36_126*RLcp36_327-OMcp36_127*RLcp36_328-OMcp36_16*
 RLcp36_323+OMcp36_324*RLcp36_125+OMcp36_325*RLcp36_126+OMcp36_326*RLcp36_127+OMcp36_327*RLcp36_128+OMcp36_36*RLcp36_123;
    VIcp36_328 = qd[3]+OMcp36_124*RLcp36_225+OMcp36_125*RLcp36_226+OMcp36_126*RLcp36_227+OMcp36_127*RLcp36_228+OMcp36_16*
 RLcp36_223-OMcp36_224*RLcp36_125-OMcp36_225*RLcp36_126-OMcp36_226*RLcp36_127-OMcp36_227*RLcp36_128-OMcp36_26*RLcp36_123;

// = = Block_1_0_0_37_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp36_128;
    sens->P[2] = POcp36_228;
    sens->P[3] = POcp36_328;
    sens->R[1][1] = ROcp36_128;
    sens->R[1][2] = ROcp36_228;
    sens->R[1][3] = ROcp36_328;
    sens->R[2][1] = ROcp36_428;
    sens->R[2][2] = ROcp36_528;
    sens->R[2][3] = ROcp36_628;
    sens->R[3][1] = ROcp36_727;
    sens->R[3][2] = ROcp36_827;
    sens->R[3][3] = ROcp36_927;
    sens->V[1] = VIcp36_128;
    sens->V[2] = VIcp36_228;
    sens->V[3] = VIcp36_328;
    sens->OM[1] = OMcp36_128;
    sens->OM[2] = OMcp36_228;
    sens->OM[3] = OMcp36_328;
 
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

// = = Block_1_0_0_38_0_8 = = 
 
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
    OMcp37_124 = OMcp37_16+ROcp37_16*qd[23]+ROcp37_423*qd[24];
    OMcp37_224 = OMcp37_26+ROcp37_26*qd[23]+ROcp37_523*qd[24];
    OMcp37_324 = OMcp37_36+ROcp37_36*qd[23]+ROcp37_623*qd[24];
    RLcp37_125 = ROcp37_724*s->dpt[3][44];
    RLcp37_225 = ROcp37_824*s->dpt[3][44];
    RLcp37_325 = ROcp37_924*s->dpt[3][44];
    OMcp37_125 = OMcp37_124+ROcp37_724*qd[25];
    OMcp37_225 = OMcp37_224+ROcp37_824*qd[25];
    OMcp37_325 = OMcp37_324+ROcp37_924*qd[25];

// = = Block_1_0_0_38_0_9 = = 
 
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
    OMcp37_126 = OMcp37_125+ROcp37_425*qd[26];
    OMcp37_226 = OMcp37_225+ROcp37_525*qd[26];
    OMcp37_326 = OMcp37_325+ROcp37_625*qd[26];
    RLcp37_127 = ROcp37_425*s->dpt[2][51];
    RLcp37_227 = ROcp37_525*s->dpt[2][51];
    RLcp37_327 = ROcp37_625*s->dpt[2][51];
    OMcp37_127 = OMcp37_126+ROcp37_126*qd[27];
    OMcp37_227 = OMcp37_226+ROcp37_226*qd[27];
    OMcp37_327 = OMcp37_326+ROcp37_326*qd[27];
    RLcp37_128 = ROcp37_727*s->dpt[3][53];
    RLcp37_228 = ROcp37_827*s->dpt[3][53];
    RLcp37_328 = ROcp37_927*s->dpt[3][53];
    OMcp37_128 = OMcp37_127+ROcp37_727*qd[28];
    OMcp37_228 = OMcp37_227+ROcp37_827*qd[28];
    OMcp37_328 = OMcp37_327+ROcp37_927*qd[28];
    RLcp37_129 = ROcp37_727*s->dpt[3][56];
    RLcp37_229 = ROcp37_827*s->dpt[3][56];
    RLcp37_329 = ROcp37_927*s->dpt[3][56];
    POcp37_129 = RLcp37_123+RLcp37_125+RLcp37_126+RLcp37_127+RLcp37_128+RLcp37_129+q[1];
    POcp37_229 = RLcp37_223+RLcp37_225+RLcp37_226+RLcp37_227+RLcp37_228+RLcp37_229+q[2];
    POcp37_329 = RLcp37_323+RLcp37_325+RLcp37_326+RLcp37_327+RLcp37_328+RLcp37_329+q[3];
    OMcp37_129 = OMcp37_128+ROcp37_428*qd[29];
    OMcp37_229 = OMcp37_228+ROcp37_528*qd[29];
    OMcp37_329 = OMcp37_328+ROcp37_628*qd[29];
    VIcp37_129 = qd[1]+OMcp37_224*RLcp37_325+OMcp37_225*RLcp37_326+OMcp37_226*RLcp37_327+OMcp37_227*RLcp37_328+OMcp37_228*
 RLcp37_329+OMcp37_26*RLcp37_323-OMcp37_324*RLcp37_225-OMcp37_325*RLcp37_226-OMcp37_326*RLcp37_227-OMcp37_327*RLcp37_228-
 OMcp37_328*RLcp37_229-OMcp37_36*RLcp37_223;
    VIcp37_229 = qd[2]-OMcp37_124*RLcp37_325-OMcp37_125*RLcp37_326-OMcp37_126*RLcp37_327-OMcp37_127*RLcp37_328-OMcp37_128*
 RLcp37_329-OMcp37_16*RLcp37_323+OMcp37_324*RLcp37_125+OMcp37_325*RLcp37_126+OMcp37_326*RLcp37_127+OMcp37_327*RLcp37_128+
 OMcp37_328*RLcp37_129+OMcp37_36*RLcp37_123;
    VIcp37_329 = qd[3]+OMcp37_124*RLcp37_225+OMcp37_125*RLcp37_226+OMcp37_126*RLcp37_227+OMcp37_127*RLcp37_228+OMcp37_128*
 RLcp37_229+OMcp37_16*RLcp37_223-OMcp37_224*RLcp37_125-OMcp37_225*RLcp37_126-OMcp37_226*RLcp37_127-OMcp37_227*RLcp37_128-
 OMcp37_228*RLcp37_129-OMcp37_26*RLcp37_123;

// = = Block_1_0_0_38_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp37_129;
    sens->P[2] = POcp37_229;
    sens->P[3] = POcp37_329;
    sens->R[1][1] = ROcp37_129;
    sens->R[1][2] = ROcp37_229;
    sens->R[1][3] = ROcp37_329;
    sens->R[2][1] = ROcp37_428;
    sens->R[2][2] = ROcp37_528;
    sens->R[2][3] = ROcp37_628;
    sens->R[3][1] = ROcp37_729;
    sens->R[3][2] = ROcp37_829;
    sens->R[3][3] = ROcp37_929;
    sens->V[1] = VIcp37_129;
    sens->V[2] = VIcp37_229;
    sens->V[3] = VIcp37_329;
    sens->OM[1] = OMcp37_129;
    sens->OM[2] = OMcp37_229;
    sens->OM[3] = OMcp37_329;
 
// 
break;
case 39:
 


// = = Block_1_0_0_39_0_1 = = 
 
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
    OMcp38_16 = qd[4]+qd[6]*S5;
    OMcp38_26 = ROcp38_85*qd[6]+qd[5]*C4;
    OMcp38_36 = ROcp38_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_39_0_8 = = 
 
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
    OMcp38_124 = OMcp38_16+ROcp38_16*qd[23]+ROcp38_423*qd[24];
    OMcp38_224 = OMcp38_26+ROcp38_26*qd[23]+ROcp38_523*qd[24];
    OMcp38_324 = OMcp38_36+ROcp38_36*qd[23]+ROcp38_623*qd[24];
    RLcp38_125 = ROcp38_724*s->dpt[3][44];
    RLcp38_225 = ROcp38_824*s->dpt[3][44];
    RLcp38_325 = ROcp38_924*s->dpt[3][44];
    OMcp38_125 = OMcp38_124+ROcp38_724*qd[25];
    OMcp38_225 = OMcp38_224+ROcp38_824*qd[25];
    OMcp38_325 = OMcp38_324+ROcp38_924*qd[25];

// = = Block_1_0_0_39_0_10 = = 
 
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
    OMcp38_130 = OMcp38_125+ROcp38_425*qd[30];
    OMcp38_230 = OMcp38_225+ROcp38_525*qd[30];
    OMcp38_330 = OMcp38_325+ROcp38_625*qd[30];
    RLcp38_131 = ROcp38_425*s->dpt[2][60];
    RLcp38_231 = ROcp38_525*s->dpt[2][60];
    RLcp38_331 = ROcp38_625*s->dpt[2][60];
    OMcp38_131 = OMcp38_130+ROcp38_130*qd[31];
    OMcp38_231 = OMcp38_230+ROcp38_230*qd[31];
    OMcp38_331 = OMcp38_330+ROcp38_330*qd[31];
    RLcp38_132 = ROcp38_731*s->dpt[3][62];
    RLcp38_232 = ROcp38_831*s->dpt[3][62];
    RLcp38_332 = ROcp38_931*s->dpt[3][62];
    POcp38_132 = RLcp38_123+RLcp38_125+RLcp38_130+RLcp38_131+RLcp38_132+q[1];
    POcp38_232 = RLcp38_223+RLcp38_225+RLcp38_230+RLcp38_231+RLcp38_232+q[2];
    POcp38_332 = RLcp38_323+RLcp38_325+RLcp38_330+RLcp38_331+RLcp38_332+q[3];
    OMcp38_132 = OMcp38_131+ROcp38_731*qd[32];
    OMcp38_232 = OMcp38_231+ROcp38_831*qd[32];
    OMcp38_332 = OMcp38_331+ROcp38_931*qd[32];
    VIcp38_132 = qd[1]+OMcp38_224*RLcp38_325+OMcp38_225*RLcp38_330+OMcp38_230*RLcp38_331+OMcp38_231*RLcp38_332+OMcp38_26*
 RLcp38_323-OMcp38_324*RLcp38_225-OMcp38_325*RLcp38_230-OMcp38_330*RLcp38_231-OMcp38_331*RLcp38_232-OMcp38_36*RLcp38_223;
    VIcp38_232 = qd[2]-OMcp38_124*RLcp38_325-OMcp38_125*RLcp38_330-OMcp38_130*RLcp38_331-OMcp38_131*RLcp38_332-OMcp38_16*
 RLcp38_323+OMcp38_324*RLcp38_125+OMcp38_325*RLcp38_130+OMcp38_330*RLcp38_131+OMcp38_331*RLcp38_132+OMcp38_36*RLcp38_123;
    VIcp38_332 = qd[3]+OMcp38_124*RLcp38_225+OMcp38_125*RLcp38_230+OMcp38_130*RLcp38_231+OMcp38_131*RLcp38_232+OMcp38_16*
 RLcp38_223-OMcp38_224*RLcp38_125-OMcp38_225*RLcp38_130-OMcp38_230*RLcp38_131-OMcp38_231*RLcp38_132-OMcp38_26*RLcp38_123;

// = = Block_1_0_0_39_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp38_132;
    sens->P[2] = POcp38_232;
    sens->P[3] = POcp38_332;
    sens->R[1][1] = ROcp38_132;
    sens->R[1][2] = ROcp38_232;
    sens->R[1][3] = ROcp38_332;
    sens->R[2][1] = ROcp38_432;
    sens->R[2][2] = ROcp38_532;
    sens->R[2][3] = ROcp38_632;
    sens->R[3][1] = ROcp38_731;
    sens->R[3][2] = ROcp38_831;
    sens->R[3][3] = ROcp38_931;
    sens->V[1] = VIcp38_132;
    sens->V[2] = VIcp38_232;
    sens->V[3] = VIcp38_332;
    sens->OM[1] = OMcp38_132;
    sens->OM[2] = OMcp38_232;
    sens->OM[3] = OMcp38_332;
 
// 
break;
case 40:
 


// = = Block_1_0_0_40_0_1 = = 
 
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
    OMcp39_16 = qd[4]+qd[6]*S5;
    OMcp39_26 = ROcp39_85*qd[6]+qd[5]*C4;
    OMcp39_36 = ROcp39_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_40_0_8 = = 
 
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
    OMcp39_124 = OMcp39_16+ROcp39_16*qd[23]+ROcp39_423*qd[24];
    OMcp39_224 = OMcp39_26+ROcp39_26*qd[23]+ROcp39_523*qd[24];
    OMcp39_324 = OMcp39_36+ROcp39_36*qd[23]+ROcp39_623*qd[24];
    RLcp39_125 = ROcp39_724*s->dpt[3][44];
    RLcp39_225 = ROcp39_824*s->dpt[3][44];
    RLcp39_325 = ROcp39_924*s->dpt[3][44];
    OMcp39_125 = OMcp39_124+ROcp39_724*qd[25];
    OMcp39_225 = OMcp39_224+ROcp39_824*qd[25];
    OMcp39_325 = OMcp39_324+ROcp39_924*qd[25];

// = = Block_1_0_0_40_0_10 = = 
 
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
    OMcp39_130 = OMcp39_125+ROcp39_425*qd[30];
    OMcp39_230 = OMcp39_225+ROcp39_525*qd[30];
    OMcp39_330 = OMcp39_325+ROcp39_625*qd[30];
    RLcp39_131 = ROcp39_425*s->dpt[2][60];
    RLcp39_231 = ROcp39_525*s->dpt[2][60];
    RLcp39_331 = ROcp39_625*s->dpt[2][60];
    OMcp39_131 = OMcp39_130+ROcp39_130*qd[31];
    OMcp39_231 = OMcp39_230+ROcp39_230*qd[31];
    OMcp39_331 = OMcp39_330+ROcp39_330*qd[31];
    RLcp39_132 = ROcp39_731*s->dpt[3][62];
    RLcp39_232 = ROcp39_831*s->dpt[3][62];
    RLcp39_332 = ROcp39_931*s->dpt[3][62];
    OMcp39_132 = OMcp39_131+ROcp39_731*qd[32];
    OMcp39_232 = OMcp39_231+ROcp39_831*qd[32];
    OMcp39_332 = OMcp39_331+ROcp39_931*qd[32];
    RLcp39_133 = ROcp39_731*s->dpt[3][65];
    RLcp39_233 = ROcp39_831*s->dpt[3][65];
    RLcp39_333 = ROcp39_931*s->dpt[3][65];
    POcp39_133 = RLcp39_123+RLcp39_125+RLcp39_130+RLcp39_131+RLcp39_132+RLcp39_133+q[1];
    POcp39_233 = RLcp39_223+RLcp39_225+RLcp39_230+RLcp39_231+RLcp39_232+RLcp39_233+q[2];
    POcp39_333 = RLcp39_323+RLcp39_325+RLcp39_330+RLcp39_331+RLcp39_332+RLcp39_333+q[3];
    OMcp39_133 = OMcp39_132+ROcp39_432*qd[33];
    OMcp39_233 = OMcp39_232+ROcp39_532*qd[33];
    OMcp39_333 = OMcp39_332+ROcp39_632*qd[33];
    VIcp39_133 = qd[1]+OMcp39_224*RLcp39_325+OMcp39_225*RLcp39_330+OMcp39_230*RLcp39_331+OMcp39_231*RLcp39_332+OMcp39_232*
 RLcp39_333+OMcp39_26*RLcp39_323-OMcp39_324*RLcp39_225-OMcp39_325*RLcp39_230-OMcp39_330*RLcp39_231-OMcp39_331*RLcp39_232-
 OMcp39_332*RLcp39_233-OMcp39_36*RLcp39_223;
    VIcp39_233 = qd[2]-OMcp39_124*RLcp39_325-OMcp39_125*RLcp39_330-OMcp39_130*RLcp39_331-OMcp39_131*RLcp39_332-OMcp39_132*
 RLcp39_333-OMcp39_16*RLcp39_323+OMcp39_324*RLcp39_125+OMcp39_325*RLcp39_130+OMcp39_330*RLcp39_131+OMcp39_331*RLcp39_132+
 OMcp39_332*RLcp39_133+OMcp39_36*RLcp39_123;
    VIcp39_333 = qd[3]+OMcp39_124*RLcp39_225+OMcp39_125*RLcp39_230+OMcp39_130*RLcp39_231+OMcp39_131*RLcp39_232+OMcp39_132*
 RLcp39_233+OMcp39_16*RLcp39_223-OMcp39_224*RLcp39_125-OMcp39_225*RLcp39_130-OMcp39_230*RLcp39_131-OMcp39_231*RLcp39_132-
 OMcp39_232*RLcp39_133-OMcp39_26*RLcp39_123;

// = = Block_1_0_0_40_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp39_133;
    sens->P[2] = POcp39_233;
    sens->P[3] = POcp39_333;
    sens->R[1][1] = ROcp39_133;
    sens->R[1][2] = ROcp39_233;
    sens->R[1][3] = ROcp39_333;
    sens->R[2][1] = ROcp39_432;
    sens->R[2][2] = ROcp39_532;
    sens->R[2][3] = ROcp39_632;
    sens->R[3][1] = ROcp39_733;
    sens->R[3][2] = ROcp39_833;
    sens->R[3][3] = ROcp39_933;
    sens->V[1] = VIcp39_133;
    sens->V[2] = VIcp39_233;
    sens->V[3] = VIcp39_333;
    sens->OM[1] = OMcp39_133;
    sens->OM[2] = OMcp39_233;
    sens->OM[3] = OMcp39_333;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

