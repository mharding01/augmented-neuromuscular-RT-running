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
//	==> Generation Date : Thu Nov 26 12:46:17 2015
//
//	==> Project name : coman_short_feet
//	==> using XML input file 
//
//	==> Number of joints : 29
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 8196
//
//	==> Generation Time :  0.290 seconds
//	==> Post-Processing :  0.180 seconds
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
 
#include "mbs_sensor_coman_short_feet.h" 
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
    RLcp0_130 = s->dpt[3][5]*S5+C5*(s->dpt[1][5]*C6-s->dpt[2][5]*S6);
    RLcp0_230 = ROcp0_85*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_25*C6+C4*S6)-s->dpt[2][5]*(ROcp0_25*S6-C4*C6);
    RLcp0_330 = ROcp0_95*s->dpt[3][5]+s->dpt[1][5]*(ROcp0_35*C6+S4*S6)-s->dpt[2][5]*(ROcp0_35*S6-S4*C6);
    POcp0_130 = RLcp0_130+q[1];
    POcp0_230 = RLcp0_230+q[2];
    POcp0_330 = RLcp0_330+q[3];
    VIcp0_130 = qd[1]+OMcp0_26*RLcp0_330-OMcp0_36*RLcp0_230;
    VIcp0_230 = qd[2]-OMcp0_16*RLcp0_330+OMcp0_36*RLcp0_130;
    VIcp0_330 = qd[3]+OMcp0_16*RLcp0_230-OMcp0_26*RLcp0_130;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_130;
    sens->P[2] = POcp0_230;
    sens->P[3] = POcp0_330;
    sens->V[1] = VIcp0_130;
    sens->V[2] = VIcp0_230;
    sens->V[3] = VIcp0_330;
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
    RLcp1_131 = ROcp1_46*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_16*C7-S5*S7)+s->dpt[3][7]*(ROcp1_16*S7+S5*C7);
    RLcp1_231 = ROcp1_56*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_26*C7-ROcp1_85*S7)+s->dpt[3][7]*(ROcp1_26*S7+ROcp1_85*C7);
    RLcp1_331 = ROcp1_66*s->dpt[2][7]+s->dpt[1][7]*(ROcp1_36*C7-ROcp1_95*S7)+s->dpt[3][7]*(ROcp1_36*S7+ROcp1_95*C7);
    POcp1_131 = RLcp1_131+RLcp1_17+q[1];
    POcp1_231 = RLcp1_231+RLcp1_27+q[2];
    POcp1_331 = RLcp1_331+RLcp1_37+q[3];
    VIcp1_131 = qd[1]+OMcp1_26*RLcp1_37+OMcp1_27*RLcp1_331-OMcp1_36*RLcp1_27-OMcp1_37*RLcp1_231;
    VIcp1_231 = qd[2]-OMcp1_16*RLcp1_37-OMcp1_17*RLcp1_331+OMcp1_36*RLcp1_17+OMcp1_37*RLcp1_131;
    VIcp1_331 = qd[3]+OMcp1_16*RLcp1_27+OMcp1_17*RLcp1_231-OMcp1_26*RLcp1_17-OMcp1_27*RLcp1_131;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_131;
    sens->P[2] = POcp1_231;
    sens->P[3] = POcp1_331;
    sens->V[1] = VIcp1_131;
    sens->V[2] = VIcp1_231;
    sens->V[3] = VIcp1_331;
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
    RLcp2_132 = ROcp2_17*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_46*C8+ROcp2_77*S8)-s->dpt[3][9]*(ROcp2_46*S8-ROcp2_77*C8);
    RLcp2_232 = ROcp2_27*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_56*C8+ROcp2_87*S8)-s->dpt[3][9]*(ROcp2_56*S8-ROcp2_87*C8);
    RLcp2_332 = ROcp2_37*s->dpt[1][9]+s->dpt[2][9]*(ROcp2_66*C8+ROcp2_97*S8)-s->dpt[3][9]*(ROcp2_66*S8-ROcp2_97*C8);
    POcp2_132 = RLcp2_132+RLcp2_17+RLcp2_18+q[1];
    POcp2_232 = RLcp2_232+RLcp2_27+RLcp2_28+q[2];
    POcp2_332 = RLcp2_332+RLcp2_37+RLcp2_38+q[3];
    VIcp2_132 = qd[1]+OMcp2_26*RLcp2_37+OMcp2_27*RLcp2_38+OMcp2_28*RLcp2_332-OMcp2_36*RLcp2_27-OMcp2_37*RLcp2_28-OMcp2_38*
 RLcp2_232;
    VIcp2_232 = qd[2]-OMcp2_16*RLcp2_37-OMcp2_17*RLcp2_38-OMcp2_18*RLcp2_332+OMcp2_36*RLcp2_17+OMcp2_37*RLcp2_18+OMcp2_38*
 RLcp2_132;
    VIcp2_332 = qd[3]+OMcp2_16*RLcp2_27+OMcp2_17*RLcp2_28+OMcp2_18*RLcp2_232-OMcp2_26*RLcp2_17-OMcp2_27*RLcp2_18-OMcp2_28*
 RLcp2_132;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_132;
    sens->P[2] = POcp2_232;
    sens->P[3] = POcp2_332;
    sens->V[1] = VIcp2_132;
    sens->V[2] = VIcp2_232;
    sens->V[3] = VIcp2_332;
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
    RLcp3_133 = ROcp3_78*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_17*C9+ROcp3_48*S9)-s->dpt[2][12]*(ROcp3_17*S9-ROcp3_48*C9);
    RLcp3_233 = ROcp3_88*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_27*C9+ROcp3_58*S9)-s->dpt[2][12]*(ROcp3_27*S9-ROcp3_58*C9);
    RLcp3_333 = ROcp3_98*s->dpt[3][12]+s->dpt[1][12]*(ROcp3_37*C9+ROcp3_68*S9)-s->dpt[2][12]*(ROcp3_37*S9-ROcp3_68*C9);
    POcp3_133 = RLcp3_133+RLcp3_17+RLcp3_18+RLcp3_19+q[1];
    POcp3_233 = RLcp3_233+RLcp3_27+RLcp3_28+RLcp3_29+q[2];
    POcp3_333 = RLcp3_333+RLcp3_37+RLcp3_38+RLcp3_39+q[3];
    VIcp3_133 = qd[1]+OMcp3_26*RLcp3_37+OMcp3_27*RLcp3_38+OMcp3_28*RLcp3_39+OMcp3_29*RLcp3_333-OMcp3_36*RLcp3_27-OMcp3_37*
 RLcp3_28-OMcp3_38*RLcp3_29-OMcp3_39*RLcp3_233;
    VIcp3_233 = qd[2]-OMcp3_16*RLcp3_37-OMcp3_17*RLcp3_38-OMcp3_18*RLcp3_39-OMcp3_19*RLcp3_333+OMcp3_36*RLcp3_17+OMcp3_37*
 RLcp3_18+OMcp3_38*RLcp3_19+OMcp3_39*RLcp3_133;
    VIcp3_333 = qd[3]+OMcp3_16*RLcp3_27+OMcp3_17*RLcp3_28+OMcp3_18*RLcp3_29+OMcp3_19*RLcp3_233-OMcp3_26*RLcp3_17-OMcp3_27*
 RLcp3_18-OMcp3_28*RLcp3_19-OMcp3_29*RLcp3_133;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_133;
    sens->P[2] = POcp3_233;
    sens->P[3] = POcp3_333;
    sens->V[1] = VIcp3_133;
    sens->V[2] = VIcp3_233;
    sens->V[3] = VIcp3_333;
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
    RLcp4_134 = ROcp4_49*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_19*C10-ROcp4_78*S10)+s->dpt[3][15]*(ROcp4_19*S10+ROcp4_78*C10);
    RLcp4_234 = ROcp4_59*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_29*C10-ROcp4_88*S10)+s->dpt[3][15]*(ROcp4_29*S10+ROcp4_88*C10);
    RLcp4_334 = ROcp4_69*s->dpt[2][15]+s->dpt[1][15]*(ROcp4_39*C10-ROcp4_98*S10)+s->dpt[3][15]*(ROcp4_39*S10+ROcp4_98*C10);
    POcp4_134 = RLcp4_110+RLcp4_134+RLcp4_17+RLcp4_18+RLcp4_19+q[1];
    POcp4_234 = RLcp4_210+RLcp4_234+RLcp4_27+RLcp4_28+RLcp4_29+q[2];
    POcp4_334 = RLcp4_310+RLcp4_334+RLcp4_37+RLcp4_38+RLcp4_39+q[3];
    VIcp4_134 = qd[1]+OMcp4_210*RLcp4_334+OMcp4_26*RLcp4_37+OMcp4_27*RLcp4_38+OMcp4_28*RLcp4_39+OMcp4_29*RLcp4_310-
 OMcp4_310*RLcp4_234-OMcp4_36*RLcp4_27-OMcp4_37*RLcp4_28-OMcp4_38*RLcp4_29-OMcp4_39*RLcp4_210;
    VIcp4_234 = qd[2]-OMcp4_110*RLcp4_334-OMcp4_16*RLcp4_37-OMcp4_17*RLcp4_38-OMcp4_18*RLcp4_39-OMcp4_19*RLcp4_310+
 OMcp4_310*RLcp4_134+OMcp4_36*RLcp4_17+OMcp4_37*RLcp4_18+OMcp4_38*RLcp4_19+OMcp4_39*RLcp4_110;
    VIcp4_334 = qd[3]+OMcp4_110*RLcp4_234+OMcp4_16*RLcp4_27+OMcp4_17*RLcp4_28+OMcp4_18*RLcp4_29+OMcp4_19*RLcp4_210-
 OMcp4_210*RLcp4_134-OMcp4_26*RLcp4_17-OMcp4_27*RLcp4_18-OMcp4_28*RLcp4_19-OMcp4_29*RLcp4_110;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_134;
    sens->P[2] = POcp4_234;
    sens->P[3] = POcp4_334;
    sens->V[1] = VIcp4_134;
    sens->V[2] = VIcp4_234;
    sens->V[3] = VIcp4_334;
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
    RLcp5_135 = ROcp5_110*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_49*C11+ROcp5_710*S11)-s->dpt[3][17]*(ROcp5_49*S11-ROcp5_710*
 C11);
    RLcp5_235 = ROcp5_210*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_59*C11+ROcp5_810*S11)-s->dpt[3][17]*(ROcp5_59*S11-ROcp5_810*
 C11);
    RLcp5_335 = ROcp5_310*s->dpt[1][17]+s->dpt[2][17]*(ROcp5_69*C11+ROcp5_910*S11)-s->dpt[3][17]*(ROcp5_69*S11-ROcp5_910*
 C11);
    POcp5_135 = RLcp5_110+RLcp5_111+RLcp5_135+RLcp5_17+RLcp5_18+RLcp5_19+q[1];
    POcp5_235 = RLcp5_210+RLcp5_211+RLcp5_235+RLcp5_27+RLcp5_28+RLcp5_29+q[2];
    POcp5_335 = RLcp5_310+RLcp5_311+RLcp5_335+RLcp5_37+RLcp5_38+RLcp5_39+q[3];
    VIcp5_135 = qd[1]+OMcp5_210*RLcp5_311+OMcp5_211*RLcp5_335+OMcp5_26*RLcp5_37+OMcp5_27*RLcp5_38+OMcp5_28*RLcp5_39+
 OMcp5_29*RLcp5_310-OMcp5_310*RLcp5_211-OMcp5_311*RLcp5_235-OMcp5_36*RLcp5_27-OMcp5_37*RLcp5_28-OMcp5_38*RLcp5_29-OMcp5_39*
 RLcp5_210;
    VIcp5_235 = qd[2]-OMcp5_110*RLcp5_311-OMcp5_111*RLcp5_335-OMcp5_16*RLcp5_37-OMcp5_17*RLcp5_38-OMcp5_18*RLcp5_39-
 OMcp5_19*RLcp5_310+OMcp5_310*RLcp5_111+OMcp5_311*RLcp5_135+OMcp5_36*RLcp5_17+OMcp5_37*RLcp5_18+OMcp5_38*RLcp5_19+OMcp5_39*
 RLcp5_110;
    VIcp5_335 = qd[3]+OMcp5_110*RLcp5_211+OMcp5_111*RLcp5_235+OMcp5_16*RLcp5_27+OMcp5_17*RLcp5_28+OMcp5_18*RLcp5_29+
 OMcp5_19*RLcp5_210-OMcp5_210*RLcp5_111-OMcp5_211*RLcp5_135-OMcp5_26*RLcp5_17-OMcp5_27*RLcp5_18-OMcp5_28*RLcp5_19-OMcp5_29*
 RLcp5_110;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_135;
    sens->P[2] = POcp5_235;
    sens->P[3] = POcp5_335;
    sens->V[1] = VIcp5_135;
    sens->V[2] = VIcp5_235;
    sens->V[3] = VIcp5_335;
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
    RLcp6_136 = s->dpt[1][18]*(ROcp6_110*C12-ROcp6_711*S12)+s->dpt[3][18]*(ROcp6_110*S12+ROcp6_711*C12);
    RLcp6_236 = s->dpt[1][18]*(ROcp6_210*C12-ROcp6_811*S12)+s->dpt[3][18]*(ROcp6_210*S12+ROcp6_811*C12);
    RLcp6_336 = s->dpt[1][18]*(ROcp6_310*C12-ROcp6_911*S12)+s->dpt[3][18]*(ROcp6_310*S12+ROcp6_911*C12);
    POcp6_136 = RLcp6_110+RLcp6_111+RLcp6_136+RLcp6_17+RLcp6_18+RLcp6_19+q[1];
    POcp6_236 = RLcp6_210+RLcp6_211+RLcp6_236+RLcp6_27+RLcp6_28+RLcp6_29+q[2];
    POcp6_336 = RLcp6_310+RLcp6_311+RLcp6_336+RLcp6_37+RLcp6_38+RLcp6_39+q[3];
    VIcp6_136 = qd[1]+OMcp6_210*RLcp6_311+OMcp6_212*RLcp6_336+OMcp6_26*RLcp6_37+OMcp6_27*RLcp6_38+OMcp6_28*RLcp6_39+
 OMcp6_29*RLcp6_310-OMcp6_310*RLcp6_211-OMcp6_312*RLcp6_236-OMcp6_36*RLcp6_27-OMcp6_37*RLcp6_28-OMcp6_38*RLcp6_29-OMcp6_39*
 RLcp6_210;
    VIcp6_236 = qd[2]-OMcp6_110*RLcp6_311-OMcp6_112*RLcp6_336-OMcp6_16*RLcp6_37-OMcp6_17*RLcp6_38-OMcp6_18*RLcp6_39-
 OMcp6_19*RLcp6_310+OMcp6_310*RLcp6_111+OMcp6_312*RLcp6_136+OMcp6_36*RLcp6_17+OMcp6_37*RLcp6_18+OMcp6_38*RLcp6_19+OMcp6_39*
 RLcp6_110;
    VIcp6_336 = qd[3]+OMcp6_110*RLcp6_211+OMcp6_112*RLcp6_236+OMcp6_16*RLcp6_27+OMcp6_17*RLcp6_28+OMcp6_18*RLcp6_29+
 OMcp6_19*RLcp6_210-OMcp6_210*RLcp6_111-OMcp6_212*RLcp6_136-OMcp6_26*RLcp6_17-OMcp6_27*RLcp6_18-OMcp6_28*RLcp6_19-OMcp6_29*
 RLcp6_110;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_136;
    sens->P[2] = POcp6_236;
    sens->P[3] = POcp6_336;
    sens->V[1] = VIcp6_136;
    sens->V[2] = VIcp6_236;
    sens->V[3] = VIcp6_336;
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


    RLcp7_113 = ROcp7_46*s->dpt[2][3];
    RLcp7_213 = ROcp7_56*s->dpt[2][3];
    RLcp7_313 = ROcp7_66*s->dpt[2][3];
    OMcp7_113 = OMcp7_16+ROcp7_46*qd[13];
    OMcp7_213 = OMcp7_26+ROcp7_56*qd[13];
    OMcp7_313 = OMcp7_36+ROcp7_66*qd[13];
    RLcp7_137 = ROcp7_46*s->dpt[2][21]+s->dpt[1][21]*(ROcp7_16*C13-S13*S5)+s->dpt[3][21]*(ROcp7_16*S13+C13*S5);
    RLcp7_237 = ROcp7_56*s->dpt[2][21]+s->dpt[1][21]*(ROcp7_26*C13-ROcp7_85*S13)+s->dpt[3][21]*(ROcp7_26*S13+ROcp7_85*C13);
    RLcp7_337 = ROcp7_66*s->dpt[2][21]+s->dpt[1][21]*(ROcp7_36*C13-ROcp7_95*S13)+s->dpt[3][21]*(ROcp7_36*S13+ROcp7_95*C13);
    POcp7_137 = RLcp7_113+RLcp7_137+q[1];
    POcp7_237 = RLcp7_213+RLcp7_237+q[2];
    POcp7_337 = RLcp7_313+RLcp7_337+q[3];
    VIcp7_137 = qd[1]+OMcp7_213*RLcp7_337+OMcp7_26*RLcp7_313-OMcp7_313*RLcp7_237-OMcp7_36*RLcp7_213;
    VIcp7_237 = qd[2]-OMcp7_113*RLcp7_337-OMcp7_16*RLcp7_313+OMcp7_313*RLcp7_137+OMcp7_36*RLcp7_113;
    VIcp7_337 = qd[3]+OMcp7_113*RLcp7_237+OMcp7_16*RLcp7_213-OMcp7_213*RLcp7_137-OMcp7_26*RLcp7_113;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_137;
    sens->P[2] = POcp7_237;
    sens->P[3] = POcp7_337;
    sens->V[1] = VIcp7_137;
    sens->V[2] = VIcp7_237;
    sens->V[3] = VIcp7_337;
    sens->OM[1] = OMcp7_113;
    sens->OM[2] = OMcp7_213;
    sens->OM[3] = OMcp7_313;
 
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


    ROcp8_113 = ROcp8_16*C13-S13*S5;
    ROcp8_213 = ROcp8_26*C13-ROcp8_85*S13;
    ROcp8_313 = ROcp8_36*C13-ROcp8_95*S13;
    ROcp8_713 = ROcp8_16*S13+C13*S5;
    ROcp8_813 = ROcp8_26*S13+ROcp8_85*C13;
    ROcp8_913 = ROcp8_36*S13+ROcp8_95*C13;
    RLcp8_113 = ROcp8_46*s->dpt[2][3];
    RLcp8_213 = ROcp8_56*s->dpt[2][3];
    RLcp8_313 = ROcp8_66*s->dpt[2][3];
    OMcp8_113 = OMcp8_16+ROcp8_46*qd[13];
    OMcp8_213 = OMcp8_26+ROcp8_56*qd[13];
    OMcp8_313 = OMcp8_36+ROcp8_66*qd[13];
    RLcp8_114 = ROcp8_46*s->dpt[2][20];
    RLcp8_214 = ROcp8_56*s->dpt[2][20];
    RLcp8_314 = ROcp8_66*s->dpt[2][20];
    OMcp8_114 = OMcp8_113+ROcp8_113*qd[14];
    OMcp8_214 = OMcp8_213+ROcp8_213*qd[14];
    OMcp8_314 = OMcp8_313+ROcp8_313*qd[14];
    RLcp8_138 = ROcp8_113*s->dpt[1][23]+s->dpt[2][23]*(ROcp8_46*C14+ROcp8_713*S14)-s->dpt[3][23]*(ROcp8_46*S14-ROcp8_713*
 C14);
    RLcp8_238 = ROcp8_213*s->dpt[1][23]+s->dpt[2][23]*(ROcp8_56*C14+ROcp8_813*S14)-s->dpt[3][23]*(ROcp8_56*S14-ROcp8_813*
 C14);
    RLcp8_338 = ROcp8_313*s->dpt[1][23]+s->dpt[2][23]*(ROcp8_66*C14+ROcp8_913*S14)-s->dpt[3][23]*(ROcp8_66*S14-ROcp8_913*
 C14);
    POcp8_138 = RLcp8_113+RLcp8_114+RLcp8_138+q[1];
    POcp8_238 = RLcp8_213+RLcp8_214+RLcp8_238+q[2];
    POcp8_338 = RLcp8_313+RLcp8_314+RLcp8_338+q[3];
    VIcp8_138 = qd[1]+OMcp8_213*RLcp8_314+OMcp8_214*RLcp8_338+OMcp8_26*RLcp8_313-OMcp8_313*RLcp8_214-OMcp8_314*RLcp8_238-
 OMcp8_36*RLcp8_213;
    VIcp8_238 = qd[2]-OMcp8_113*RLcp8_314-OMcp8_114*RLcp8_338-OMcp8_16*RLcp8_313+OMcp8_313*RLcp8_114+OMcp8_314*RLcp8_138+
 OMcp8_36*RLcp8_113;
    VIcp8_338 = qd[3]+OMcp8_113*RLcp8_214+OMcp8_114*RLcp8_238+OMcp8_16*RLcp8_213-OMcp8_213*RLcp8_114-OMcp8_214*RLcp8_138-
 OMcp8_26*RLcp8_113;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_138;
    sens->P[2] = POcp8_238;
    sens->P[3] = POcp8_338;
    sens->V[1] = VIcp8_138;
    sens->V[2] = VIcp8_238;
    sens->V[3] = VIcp8_338;
    sens->OM[1] = OMcp8_114;
    sens->OM[2] = OMcp8_214;
    sens->OM[3] = OMcp8_314;
 
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


    ROcp9_113 = ROcp9_16*C13-S13*S5;
    ROcp9_213 = ROcp9_26*C13-ROcp9_85*S13;
    ROcp9_313 = ROcp9_36*C13-ROcp9_95*S13;
    ROcp9_713 = ROcp9_16*S13+C13*S5;
    ROcp9_813 = ROcp9_26*S13+ROcp9_85*C13;
    ROcp9_913 = ROcp9_36*S13+ROcp9_95*C13;
    ROcp9_414 = ROcp9_46*C14+ROcp9_713*S14;
    ROcp9_514 = ROcp9_56*C14+ROcp9_813*S14;
    ROcp9_614 = ROcp9_66*C14+ROcp9_913*S14;
    ROcp9_714 = -(ROcp9_46*S14-ROcp9_713*C14);
    ROcp9_814 = -(ROcp9_56*S14-ROcp9_813*C14);
    ROcp9_914 = -(ROcp9_66*S14-ROcp9_913*C14);
    RLcp9_113 = ROcp9_46*s->dpt[2][3];
    RLcp9_213 = ROcp9_56*s->dpt[2][3];
    RLcp9_313 = ROcp9_66*s->dpt[2][3];
    OMcp9_113 = OMcp9_16+ROcp9_46*qd[13];
    OMcp9_213 = OMcp9_26+ROcp9_56*qd[13];
    OMcp9_313 = OMcp9_36+ROcp9_66*qd[13];
    RLcp9_114 = ROcp9_46*s->dpt[2][20];
    RLcp9_214 = ROcp9_56*s->dpt[2][20];
    RLcp9_314 = ROcp9_66*s->dpt[2][20];
    OMcp9_114 = OMcp9_113+ROcp9_113*qd[14];
    OMcp9_214 = OMcp9_213+ROcp9_213*qd[14];
    OMcp9_314 = OMcp9_313+ROcp9_313*qd[14];
    RLcp9_115 = ROcp9_714*s->dpt[3][22];
    RLcp9_215 = ROcp9_814*s->dpt[3][22];
    RLcp9_315 = ROcp9_914*s->dpt[3][22];
    OMcp9_115 = OMcp9_114+ROcp9_714*qd[15];
    OMcp9_215 = OMcp9_214+ROcp9_814*qd[15];
    OMcp9_315 = OMcp9_314+ROcp9_914*qd[15];
    RLcp9_139 = ROcp9_714*s->dpt[3][26]+s->dpt[1][26]*(ROcp9_113*C15+ROcp9_414*S15)-s->dpt[2][26]*(ROcp9_113*S15-ROcp9_414
 *C15);
    RLcp9_239 = ROcp9_814*s->dpt[3][26]+s->dpt[1][26]*(ROcp9_213*C15+ROcp9_514*S15)-s->dpt[2][26]*(ROcp9_213*S15-ROcp9_514
 *C15);
    RLcp9_339 = ROcp9_914*s->dpt[3][26]+s->dpt[1][26]*(ROcp9_313*C15+ROcp9_614*S15)-s->dpt[2][26]*(ROcp9_313*S15-ROcp9_614
 *C15);
    POcp9_139 = RLcp9_113+RLcp9_114+RLcp9_115+RLcp9_139+q[1];
    POcp9_239 = RLcp9_213+RLcp9_214+RLcp9_215+RLcp9_239+q[2];
    POcp9_339 = RLcp9_313+RLcp9_314+RLcp9_315+RLcp9_339+q[3];
    VIcp9_139 = qd[1]+OMcp9_213*RLcp9_314+OMcp9_214*RLcp9_315+OMcp9_215*RLcp9_339+OMcp9_26*RLcp9_313-OMcp9_313*RLcp9_214-
 OMcp9_314*RLcp9_215-OMcp9_315*RLcp9_239-OMcp9_36*RLcp9_213;
    VIcp9_239 = qd[2]-OMcp9_113*RLcp9_314-OMcp9_114*RLcp9_315-OMcp9_115*RLcp9_339-OMcp9_16*RLcp9_313+OMcp9_313*RLcp9_114+
 OMcp9_314*RLcp9_115+OMcp9_315*RLcp9_139+OMcp9_36*RLcp9_113;
    VIcp9_339 = qd[3]+OMcp9_113*RLcp9_214+OMcp9_114*RLcp9_215+OMcp9_115*RLcp9_239+OMcp9_16*RLcp9_213-OMcp9_213*RLcp9_114-
 OMcp9_214*RLcp9_115-OMcp9_215*RLcp9_139-OMcp9_26*RLcp9_113;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_139;
    sens->P[2] = POcp9_239;
    sens->P[3] = POcp9_339;
    sens->V[1] = VIcp9_139;
    sens->V[2] = VIcp9_239;
    sens->V[3] = VIcp9_339;
    sens->OM[1] = OMcp9_115;
    sens->OM[2] = OMcp9_215;
    sens->OM[3] = OMcp9_315;
 
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


    ROcp10_113 = ROcp10_16*C13-S13*S5;
    ROcp10_213 = ROcp10_26*C13-ROcp10_85*S13;
    ROcp10_313 = ROcp10_36*C13-ROcp10_95*S13;
    ROcp10_713 = ROcp10_16*S13+C13*S5;
    ROcp10_813 = ROcp10_26*S13+ROcp10_85*C13;
    ROcp10_913 = ROcp10_36*S13+ROcp10_95*C13;
    ROcp10_414 = ROcp10_46*C14+ROcp10_713*S14;
    ROcp10_514 = ROcp10_56*C14+ROcp10_813*S14;
    ROcp10_614 = ROcp10_66*C14+ROcp10_913*S14;
    ROcp10_714 = -(ROcp10_46*S14-ROcp10_713*C14);
    ROcp10_814 = -(ROcp10_56*S14-ROcp10_813*C14);
    ROcp10_914 = -(ROcp10_66*S14-ROcp10_913*C14);
    ROcp10_115 = ROcp10_113*C15+ROcp10_414*S15;
    ROcp10_215 = ROcp10_213*C15+ROcp10_514*S15;
    ROcp10_315 = ROcp10_313*C15+ROcp10_614*S15;
    ROcp10_415 = -(ROcp10_113*S15-ROcp10_414*C15);
    ROcp10_515 = -(ROcp10_213*S15-ROcp10_514*C15);
    ROcp10_615 = -(ROcp10_313*S15-ROcp10_614*C15);
    RLcp10_113 = ROcp10_46*s->dpt[2][3];
    RLcp10_213 = ROcp10_56*s->dpt[2][3];
    RLcp10_313 = ROcp10_66*s->dpt[2][3];
    OMcp10_113 = OMcp10_16+ROcp10_46*qd[13];
    OMcp10_213 = OMcp10_26+ROcp10_56*qd[13];
    OMcp10_313 = OMcp10_36+ROcp10_66*qd[13];
    RLcp10_114 = ROcp10_46*s->dpt[2][20];
    RLcp10_214 = ROcp10_56*s->dpt[2][20];
    RLcp10_314 = ROcp10_66*s->dpt[2][20];
    OMcp10_114 = OMcp10_113+ROcp10_113*qd[14];
    OMcp10_214 = OMcp10_213+ROcp10_213*qd[14];
    OMcp10_314 = OMcp10_313+ROcp10_313*qd[14];
    RLcp10_115 = ROcp10_714*s->dpt[3][22];
    RLcp10_215 = ROcp10_814*s->dpt[3][22];
    RLcp10_315 = ROcp10_914*s->dpt[3][22];
    OMcp10_115 = OMcp10_114+ROcp10_714*qd[15];
    OMcp10_215 = OMcp10_214+ROcp10_814*qd[15];
    OMcp10_315 = OMcp10_314+ROcp10_914*qd[15];
    RLcp10_116 = ROcp10_714*s->dpt[3][25];
    RLcp10_216 = ROcp10_814*s->dpt[3][25];
    RLcp10_316 = ROcp10_914*s->dpt[3][25];
    OMcp10_116 = OMcp10_115+ROcp10_415*qd[16];
    OMcp10_216 = OMcp10_215+ROcp10_515*qd[16];
    OMcp10_316 = OMcp10_315+ROcp10_615*qd[16];
    RLcp10_140 = ROcp10_415*s->dpt[2][29]+s->dpt[1][29]*(ROcp10_115*C16-ROcp10_714*S16)+s->dpt[3][29]*(ROcp10_115*S16+
 ROcp10_714*C16);
    RLcp10_240 = ROcp10_515*s->dpt[2][29]+s->dpt[1][29]*(ROcp10_215*C16-ROcp10_814*S16)+s->dpt[3][29]*(ROcp10_215*S16+
 ROcp10_814*C16);
    RLcp10_340 = ROcp10_615*s->dpt[2][29]+s->dpt[1][29]*(ROcp10_315*C16-ROcp10_914*S16)+s->dpt[3][29]*(ROcp10_315*S16+
 ROcp10_914*C16);
    POcp10_140 = RLcp10_113+RLcp10_114+RLcp10_115+RLcp10_116+RLcp10_140+q[1];
    POcp10_240 = RLcp10_213+RLcp10_214+RLcp10_215+RLcp10_216+RLcp10_240+q[2];
    POcp10_340 = RLcp10_313+RLcp10_314+RLcp10_315+RLcp10_316+RLcp10_340+q[3];
    VIcp10_140 = qd[1]+OMcp10_213*RLcp10_314+OMcp10_214*RLcp10_315+OMcp10_215*RLcp10_316+OMcp10_216*RLcp10_340+OMcp10_26*
 RLcp10_313-OMcp10_313*RLcp10_214-OMcp10_314*RLcp10_215-OMcp10_315*RLcp10_216-OMcp10_316*RLcp10_240-OMcp10_36*RLcp10_213;
    VIcp10_240 = qd[2]-OMcp10_113*RLcp10_314-OMcp10_114*RLcp10_315-OMcp10_115*RLcp10_316-OMcp10_116*RLcp10_340-OMcp10_16*
 RLcp10_313+OMcp10_313*RLcp10_114+OMcp10_314*RLcp10_115+OMcp10_315*RLcp10_116+OMcp10_316*RLcp10_140+OMcp10_36*RLcp10_113;
    VIcp10_340 = qd[3]+OMcp10_113*RLcp10_214+OMcp10_114*RLcp10_215+OMcp10_115*RLcp10_216+OMcp10_116*RLcp10_240+OMcp10_16*
 RLcp10_213-OMcp10_213*RLcp10_114-OMcp10_214*RLcp10_115-OMcp10_215*RLcp10_116-OMcp10_216*RLcp10_140-OMcp10_26*RLcp10_113;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_140;
    sens->P[2] = POcp10_240;
    sens->P[3] = POcp10_340;
    sens->V[1] = VIcp10_140;
    sens->V[2] = VIcp10_240;
    sens->V[3] = VIcp10_340;
    sens->OM[1] = OMcp10_116;
    sens->OM[2] = OMcp10_216;
    sens->OM[3] = OMcp10_316;
 
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


    ROcp11_113 = ROcp11_16*C13-S13*S5;
    ROcp11_213 = ROcp11_26*C13-ROcp11_85*S13;
    ROcp11_313 = ROcp11_36*C13-ROcp11_95*S13;
    ROcp11_713 = ROcp11_16*S13+C13*S5;
    ROcp11_813 = ROcp11_26*S13+ROcp11_85*C13;
    ROcp11_913 = ROcp11_36*S13+ROcp11_95*C13;
    ROcp11_414 = ROcp11_46*C14+ROcp11_713*S14;
    ROcp11_514 = ROcp11_56*C14+ROcp11_813*S14;
    ROcp11_614 = ROcp11_66*C14+ROcp11_913*S14;
    ROcp11_714 = -(ROcp11_46*S14-ROcp11_713*C14);
    ROcp11_814 = -(ROcp11_56*S14-ROcp11_813*C14);
    ROcp11_914 = -(ROcp11_66*S14-ROcp11_913*C14);
    ROcp11_115 = ROcp11_113*C15+ROcp11_414*S15;
    ROcp11_215 = ROcp11_213*C15+ROcp11_514*S15;
    ROcp11_315 = ROcp11_313*C15+ROcp11_614*S15;
    ROcp11_415 = -(ROcp11_113*S15-ROcp11_414*C15);
    ROcp11_515 = -(ROcp11_213*S15-ROcp11_514*C15);
    ROcp11_615 = -(ROcp11_313*S15-ROcp11_614*C15);
    ROcp11_116 = ROcp11_115*C16-ROcp11_714*S16;
    ROcp11_216 = ROcp11_215*C16-ROcp11_814*S16;
    ROcp11_316 = ROcp11_315*C16-ROcp11_914*S16;
    ROcp11_716 = ROcp11_115*S16+ROcp11_714*C16;
    ROcp11_816 = ROcp11_215*S16+ROcp11_814*C16;
    ROcp11_916 = ROcp11_315*S16+ROcp11_914*C16;
    RLcp11_113 = ROcp11_46*s->dpt[2][3];
    RLcp11_213 = ROcp11_56*s->dpt[2][3];
    RLcp11_313 = ROcp11_66*s->dpt[2][3];
    OMcp11_113 = OMcp11_16+ROcp11_46*qd[13];
    OMcp11_213 = OMcp11_26+ROcp11_56*qd[13];
    OMcp11_313 = OMcp11_36+ROcp11_66*qd[13];
    RLcp11_114 = ROcp11_46*s->dpt[2][20];
    RLcp11_214 = ROcp11_56*s->dpt[2][20];
    RLcp11_314 = ROcp11_66*s->dpt[2][20];
    OMcp11_114 = OMcp11_113+ROcp11_113*qd[14];
    OMcp11_214 = OMcp11_213+ROcp11_213*qd[14];
    OMcp11_314 = OMcp11_313+ROcp11_313*qd[14];
    RLcp11_115 = ROcp11_714*s->dpt[3][22];
    RLcp11_215 = ROcp11_814*s->dpt[3][22];
    RLcp11_315 = ROcp11_914*s->dpt[3][22];
    OMcp11_115 = OMcp11_114+ROcp11_714*qd[15];
    OMcp11_215 = OMcp11_214+ROcp11_814*qd[15];
    OMcp11_315 = OMcp11_314+ROcp11_914*qd[15];
    RLcp11_116 = ROcp11_714*s->dpt[3][25];
    RLcp11_216 = ROcp11_814*s->dpt[3][25];
    RLcp11_316 = ROcp11_914*s->dpt[3][25];
    OMcp11_116 = OMcp11_115+ROcp11_415*qd[16];
    OMcp11_216 = OMcp11_215+ROcp11_515*qd[16];
    OMcp11_316 = OMcp11_315+ROcp11_615*qd[16];
    RLcp11_117 = ROcp11_716*s->dpt[3][28];
    RLcp11_217 = ROcp11_816*s->dpt[3][28];
    RLcp11_317 = ROcp11_916*s->dpt[3][28];
    OMcp11_117 = OMcp11_116+ROcp11_116*qd[17];
    OMcp11_217 = OMcp11_216+ROcp11_216*qd[17];
    OMcp11_317 = OMcp11_316+ROcp11_316*qd[17];
    RLcp11_141 = ROcp11_116*s->dpt[1][31]+s->dpt[2][31]*(ROcp11_415*C17+ROcp11_716*S17)-s->dpt[3][31]*(ROcp11_415*S17-
 ROcp11_716*C17);
    RLcp11_241 = ROcp11_216*s->dpt[1][31]+s->dpt[2][31]*(ROcp11_515*C17+ROcp11_816*S17)-s->dpt[3][31]*(ROcp11_515*S17-
 ROcp11_816*C17);
    RLcp11_341 = ROcp11_316*s->dpt[1][31]+s->dpt[2][31]*(ROcp11_615*C17+ROcp11_916*S17)-s->dpt[3][31]*(ROcp11_615*S17-
 ROcp11_916*C17);
    POcp11_141 = RLcp11_113+RLcp11_114+RLcp11_115+RLcp11_116+RLcp11_117+RLcp11_141+q[1];
    POcp11_241 = RLcp11_213+RLcp11_214+RLcp11_215+RLcp11_216+RLcp11_217+RLcp11_241+q[2];
    POcp11_341 = RLcp11_313+RLcp11_314+RLcp11_315+RLcp11_316+RLcp11_317+RLcp11_341+q[3];
    VIcp11_141 = qd[1]+OMcp11_213*RLcp11_314+OMcp11_214*RLcp11_315+OMcp11_215*RLcp11_316+OMcp11_216*RLcp11_317+OMcp11_217*
 RLcp11_341+OMcp11_26*RLcp11_313-OMcp11_313*RLcp11_214-OMcp11_314*RLcp11_215-OMcp11_315*RLcp11_216-OMcp11_316*RLcp11_217-
 OMcp11_317*RLcp11_241-OMcp11_36*RLcp11_213;
    VIcp11_241 = qd[2]-OMcp11_113*RLcp11_314-OMcp11_114*RLcp11_315-OMcp11_115*RLcp11_316-OMcp11_116*RLcp11_317-OMcp11_117*
 RLcp11_341-OMcp11_16*RLcp11_313+OMcp11_313*RLcp11_114+OMcp11_314*RLcp11_115+OMcp11_315*RLcp11_116+OMcp11_316*RLcp11_117+
 OMcp11_317*RLcp11_141+OMcp11_36*RLcp11_113;
    VIcp11_341 = qd[3]+OMcp11_113*RLcp11_214+OMcp11_114*RLcp11_215+OMcp11_115*RLcp11_216+OMcp11_116*RLcp11_217+OMcp11_117*
 RLcp11_241+OMcp11_16*RLcp11_213-OMcp11_213*RLcp11_114-OMcp11_214*RLcp11_115-OMcp11_215*RLcp11_116-OMcp11_216*RLcp11_117-
 OMcp11_217*RLcp11_141-OMcp11_26*RLcp11_113;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_141;
    sens->P[2] = POcp11_241;
    sens->P[3] = POcp11_341;
    sens->V[1] = VIcp11_141;
    sens->V[2] = VIcp11_241;
    sens->V[3] = VIcp11_341;
    sens->OM[1] = OMcp11_117;
    sens->OM[2] = OMcp11_217;
    sens->OM[3] = OMcp11_317;
 
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


    ROcp12_113 = ROcp12_16*C13-S13*S5;
    ROcp12_213 = ROcp12_26*C13-ROcp12_85*S13;
    ROcp12_313 = ROcp12_36*C13-ROcp12_95*S13;
    ROcp12_713 = ROcp12_16*S13+C13*S5;
    ROcp12_813 = ROcp12_26*S13+ROcp12_85*C13;
    ROcp12_913 = ROcp12_36*S13+ROcp12_95*C13;
    ROcp12_414 = ROcp12_46*C14+ROcp12_713*S14;
    ROcp12_514 = ROcp12_56*C14+ROcp12_813*S14;
    ROcp12_614 = ROcp12_66*C14+ROcp12_913*S14;
    ROcp12_714 = -(ROcp12_46*S14-ROcp12_713*C14);
    ROcp12_814 = -(ROcp12_56*S14-ROcp12_813*C14);
    ROcp12_914 = -(ROcp12_66*S14-ROcp12_913*C14);
    ROcp12_115 = ROcp12_113*C15+ROcp12_414*S15;
    ROcp12_215 = ROcp12_213*C15+ROcp12_514*S15;
    ROcp12_315 = ROcp12_313*C15+ROcp12_614*S15;
    ROcp12_415 = -(ROcp12_113*S15-ROcp12_414*C15);
    ROcp12_515 = -(ROcp12_213*S15-ROcp12_514*C15);
    ROcp12_615 = -(ROcp12_313*S15-ROcp12_614*C15);
    ROcp12_116 = ROcp12_115*C16-ROcp12_714*S16;
    ROcp12_216 = ROcp12_215*C16-ROcp12_814*S16;
    ROcp12_316 = ROcp12_315*C16-ROcp12_914*S16;
    ROcp12_716 = ROcp12_115*S16+ROcp12_714*C16;
    ROcp12_816 = ROcp12_215*S16+ROcp12_814*C16;
    ROcp12_916 = ROcp12_315*S16+ROcp12_914*C16;
    ROcp12_717 = -(ROcp12_415*S17-ROcp12_716*C17);
    ROcp12_817 = -(ROcp12_515*S17-ROcp12_816*C17);
    ROcp12_917 = -(ROcp12_615*S17-ROcp12_916*C17);
    RLcp12_113 = ROcp12_46*s->dpt[2][3];
    RLcp12_213 = ROcp12_56*s->dpt[2][3];
    RLcp12_313 = ROcp12_66*s->dpt[2][3];
    OMcp12_113 = OMcp12_16+ROcp12_46*qd[13];
    OMcp12_213 = OMcp12_26+ROcp12_56*qd[13];
    OMcp12_313 = OMcp12_36+ROcp12_66*qd[13];
    RLcp12_114 = ROcp12_46*s->dpt[2][20];
    RLcp12_214 = ROcp12_56*s->dpt[2][20];
    RLcp12_314 = ROcp12_66*s->dpt[2][20];
    OMcp12_114 = OMcp12_113+ROcp12_113*qd[14];
    OMcp12_214 = OMcp12_213+ROcp12_213*qd[14];
    OMcp12_314 = OMcp12_313+ROcp12_313*qd[14];
    RLcp12_115 = ROcp12_714*s->dpt[3][22];
    RLcp12_215 = ROcp12_814*s->dpt[3][22];
    RLcp12_315 = ROcp12_914*s->dpt[3][22];
    OMcp12_115 = OMcp12_114+ROcp12_714*qd[15];
    OMcp12_215 = OMcp12_214+ROcp12_814*qd[15];
    OMcp12_315 = OMcp12_314+ROcp12_914*qd[15];
    RLcp12_116 = ROcp12_714*s->dpt[3][25];
    RLcp12_216 = ROcp12_814*s->dpt[3][25];
    RLcp12_316 = ROcp12_914*s->dpt[3][25];
    OMcp12_116 = OMcp12_115+ROcp12_415*qd[16];
    OMcp12_216 = OMcp12_215+ROcp12_515*qd[16];
    OMcp12_316 = OMcp12_315+ROcp12_615*qd[16];
    RLcp12_117 = ROcp12_716*s->dpt[3][28];
    RLcp12_217 = ROcp12_816*s->dpt[3][28];
    RLcp12_317 = ROcp12_916*s->dpt[3][28];
    OMcp12_118 = OMcp12_116+ROcp12_116*qd[17]+qd[18]*(ROcp12_415*C17+ROcp12_716*S17);
    OMcp12_218 = OMcp12_216+ROcp12_216*qd[17]+qd[18]*(ROcp12_515*C17+ROcp12_816*S17);
    OMcp12_318 = OMcp12_316+ROcp12_316*qd[17]+qd[18]*(ROcp12_615*C17+ROcp12_916*S17);
    RLcp12_142 = s->dpt[1][32]*(ROcp12_116*C18-ROcp12_717*S18)+s->dpt[3][32]*(ROcp12_116*S18+ROcp12_717*C18);
    RLcp12_242 = s->dpt[1][32]*(ROcp12_216*C18-ROcp12_817*S18)+s->dpt[3][32]*(ROcp12_216*S18+ROcp12_817*C18);
    RLcp12_342 = s->dpt[1][32]*(ROcp12_316*C18-ROcp12_917*S18)+s->dpt[3][32]*(ROcp12_316*S18+ROcp12_917*C18);
    POcp12_142 = RLcp12_113+RLcp12_114+RLcp12_115+RLcp12_116+RLcp12_117+RLcp12_142+q[1];
    POcp12_242 = RLcp12_213+RLcp12_214+RLcp12_215+RLcp12_216+RLcp12_217+RLcp12_242+q[2];
    POcp12_342 = RLcp12_313+RLcp12_314+RLcp12_315+RLcp12_316+RLcp12_317+RLcp12_342+q[3];
    VIcp12_142 = qd[1]+OMcp12_213*RLcp12_314+OMcp12_214*RLcp12_315+OMcp12_215*RLcp12_316+OMcp12_216*RLcp12_317+OMcp12_218*
 RLcp12_342+OMcp12_26*RLcp12_313-OMcp12_313*RLcp12_214-OMcp12_314*RLcp12_215-OMcp12_315*RLcp12_216-OMcp12_316*RLcp12_217-
 OMcp12_318*RLcp12_242-OMcp12_36*RLcp12_213;
    VIcp12_242 = qd[2]-OMcp12_113*RLcp12_314-OMcp12_114*RLcp12_315-OMcp12_115*RLcp12_316-OMcp12_116*RLcp12_317-OMcp12_118*
 RLcp12_342-OMcp12_16*RLcp12_313+OMcp12_313*RLcp12_114+OMcp12_314*RLcp12_115+OMcp12_315*RLcp12_116+OMcp12_316*RLcp12_117+
 OMcp12_318*RLcp12_142+OMcp12_36*RLcp12_113;
    VIcp12_342 = qd[3]+OMcp12_113*RLcp12_214+OMcp12_114*RLcp12_215+OMcp12_115*RLcp12_216+OMcp12_116*RLcp12_217+OMcp12_118*
 RLcp12_242+OMcp12_16*RLcp12_213-OMcp12_213*RLcp12_114-OMcp12_214*RLcp12_115-OMcp12_215*RLcp12_116-OMcp12_216*RLcp12_117-
 OMcp12_218*RLcp12_142-OMcp12_26*RLcp12_113;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_142;
    sens->P[2] = POcp12_242;
    sens->P[3] = POcp12_342;
    sens->V[1] = VIcp12_142;
    sens->V[2] = VIcp12_242;
    sens->V[3] = VIcp12_342;
    sens->OM[1] = OMcp12_118;
    sens->OM[2] = OMcp12_218;
    sens->OM[3] = OMcp12_318;
 
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


    RLcp13_119 = ROcp13_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp13_219 = ROcp13_26*s->dpt[1][4]+ROcp13_85*s->dpt[3][4];
    RLcp13_319 = ROcp13_36*s->dpt[1][4]+ROcp13_95*s->dpt[3][4];
    OMcp13_119 = OMcp13_16+ROcp13_16*qd[19];
    OMcp13_219 = OMcp13_26+ROcp13_26*qd[19];
    OMcp13_319 = OMcp13_36+ROcp13_36*qd[19];
    RLcp13_143 = ROcp13_16*s->dpt[1][35]+s->dpt[2][35]*(ROcp13_46*C19+S19*S5)-s->dpt[3][35]*(ROcp13_46*S19-C19*S5);
    RLcp13_243 = ROcp13_26*s->dpt[1][35]+s->dpt[2][35]*(ROcp13_56*C19+ROcp13_85*S19)-s->dpt[3][35]*(ROcp13_56*S19-
 ROcp13_85*C19);
    RLcp13_343 = ROcp13_36*s->dpt[1][35]+s->dpt[2][35]*(ROcp13_66*C19+ROcp13_95*S19)-s->dpt[3][35]*(ROcp13_66*S19-
 ROcp13_95*C19);
    POcp13_143 = RLcp13_119+RLcp13_143+q[1];
    POcp13_243 = RLcp13_219+RLcp13_243+q[2];
    POcp13_343 = RLcp13_319+RLcp13_343+q[3];
    VIcp13_143 = qd[1]+OMcp13_219*RLcp13_343+OMcp13_26*RLcp13_319-OMcp13_319*RLcp13_243-OMcp13_36*RLcp13_219;
    VIcp13_243 = qd[2]-OMcp13_119*RLcp13_343-OMcp13_16*RLcp13_319+OMcp13_319*RLcp13_143+OMcp13_36*RLcp13_119;
    VIcp13_343 = qd[3]+OMcp13_119*RLcp13_243+OMcp13_16*RLcp13_219-OMcp13_219*RLcp13_143-OMcp13_26*RLcp13_119;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_143;
    sens->P[2] = POcp13_243;
    sens->P[3] = POcp13_343;
    sens->V[1] = VIcp13_143;
    sens->V[2] = VIcp13_243;
    sens->V[3] = VIcp13_343;
    sens->OM[1] = OMcp13_119;
    sens->OM[2] = OMcp13_219;
    sens->OM[3] = OMcp13_319;
 
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


    ROcp14_419 = ROcp14_46*C19+S19*S5;
    ROcp14_519 = ROcp14_56*C19+ROcp14_85*S19;
    ROcp14_619 = ROcp14_66*C19+ROcp14_95*S19;
    ROcp14_719 = -(ROcp14_46*S19-C19*S5);
    ROcp14_819 = -(ROcp14_56*S19-ROcp14_85*C19);
    ROcp14_919 = -(ROcp14_66*S19-ROcp14_95*C19);
    RLcp14_119 = ROcp14_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp14_219 = ROcp14_26*s->dpt[1][4]+ROcp14_85*s->dpt[3][4];
    RLcp14_319 = ROcp14_36*s->dpt[1][4]+ROcp14_95*s->dpt[3][4];
    OMcp14_120 = OMcp14_16+ROcp14_16*qd[19]+ROcp14_419*qd[20];
    OMcp14_220 = OMcp14_26+ROcp14_26*qd[19]+ROcp14_519*qd[20];
    OMcp14_320 = OMcp14_36+ROcp14_36*qd[19]+ROcp14_619*qd[20];
    RLcp14_144 = ROcp14_419*s->dpt[2][37]+s->dpt[1][37]*(ROcp14_16*C20-ROcp14_719*S20)+s->dpt[3][37]*(ROcp14_16*S20+
 ROcp14_719*C20);
    RLcp14_244 = ROcp14_519*s->dpt[2][37]+s->dpt[1][37]*(ROcp14_26*C20-ROcp14_819*S20)+s->dpt[3][37]*(ROcp14_26*S20+
 ROcp14_819*C20);
    RLcp14_344 = ROcp14_619*s->dpt[2][37]+s->dpt[1][37]*(ROcp14_36*C20-ROcp14_919*S20)+s->dpt[3][37]*(ROcp14_36*S20+
 ROcp14_919*C20);
    POcp14_144 = RLcp14_119+RLcp14_144+q[1];
    POcp14_244 = RLcp14_219+RLcp14_244+q[2];
    POcp14_344 = RLcp14_319+RLcp14_344+q[3];
    VIcp14_144 = qd[1]+OMcp14_220*RLcp14_344+OMcp14_26*RLcp14_319-OMcp14_320*RLcp14_244-OMcp14_36*RLcp14_219;
    VIcp14_244 = qd[2]-OMcp14_120*RLcp14_344-OMcp14_16*RLcp14_319+OMcp14_320*RLcp14_144+OMcp14_36*RLcp14_119;
    VIcp14_344 = qd[3]+OMcp14_120*RLcp14_244+OMcp14_16*RLcp14_219-OMcp14_220*RLcp14_144-OMcp14_26*RLcp14_119;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_144;
    sens->P[2] = POcp14_244;
    sens->P[3] = POcp14_344;
    sens->V[1] = VIcp14_144;
    sens->V[2] = VIcp14_244;
    sens->V[3] = VIcp14_344;
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

// = = Block_1_0_0_16_0_4 = = 
 
// Sensor Kinematics 


    ROcp15_419 = ROcp15_46*C19+S19*S5;
    ROcp15_519 = ROcp15_56*C19+ROcp15_85*S19;
    ROcp15_619 = ROcp15_66*C19+ROcp15_95*S19;
    ROcp15_719 = -(ROcp15_46*S19-C19*S5);
    ROcp15_819 = -(ROcp15_56*S19-ROcp15_85*C19);
    ROcp15_919 = -(ROcp15_66*S19-ROcp15_95*C19);
    ROcp15_120 = ROcp15_16*C20-ROcp15_719*S20;
    ROcp15_220 = ROcp15_26*C20-ROcp15_819*S20;
    ROcp15_320 = ROcp15_36*C20-ROcp15_919*S20;
    ROcp15_720 = ROcp15_16*S20+ROcp15_719*C20;
    ROcp15_820 = ROcp15_26*S20+ROcp15_819*C20;
    ROcp15_920 = ROcp15_36*S20+ROcp15_919*C20;
    RLcp15_119 = ROcp15_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp15_219 = ROcp15_26*s->dpt[1][4]+ROcp15_85*s->dpt[3][4];
    RLcp15_319 = ROcp15_36*s->dpt[1][4]+ROcp15_95*s->dpt[3][4];
    OMcp15_120 = OMcp15_16+ROcp15_16*qd[19]+ROcp15_419*qd[20];
    OMcp15_220 = OMcp15_26+ROcp15_26*qd[19]+ROcp15_519*qd[20];
    OMcp15_320 = OMcp15_36+ROcp15_36*qd[19]+ROcp15_619*qd[20];
    RLcp15_121 = ROcp15_720*s->dpt[3][36];
    RLcp15_221 = ROcp15_820*s->dpt[3][36];
    RLcp15_321 = ROcp15_920*s->dpt[3][36];
    OMcp15_121 = OMcp15_120+ROcp15_720*qd[21];
    OMcp15_221 = OMcp15_220+ROcp15_820*qd[21];
    OMcp15_321 = OMcp15_320+ROcp15_920*qd[21];
    RLcp15_145 = ROcp15_720*s->dpt[3][39]+s->dpt[1][39]*(ROcp15_120*C21+ROcp15_419*S21)-s->dpt[2][39]*(ROcp15_120*S21-
 ROcp15_419*C21);
    RLcp15_245 = ROcp15_820*s->dpt[3][39]+s->dpt[1][39]*(ROcp15_220*C21+ROcp15_519*S21)-s->dpt[2][39]*(ROcp15_220*S21-
 ROcp15_519*C21);
    RLcp15_345 = ROcp15_920*s->dpt[3][39]+s->dpt[1][39]*(ROcp15_320*C21+ROcp15_619*S21)-s->dpt[2][39]*(ROcp15_320*S21-
 ROcp15_619*C21);
    POcp15_145 = RLcp15_119+RLcp15_121+RLcp15_145+q[1];
    POcp15_245 = RLcp15_219+RLcp15_221+RLcp15_245+q[2];
    POcp15_345 = RLcp15_319+RLcp15_321+RLcp15_345+q[3];
    VIcp15_145 = qd[1]+OMcp15_220*RLcp15_321+OMcp15_221*RLcp15_345+OMcp15_26*RLcp15_319-OMcp15_320*RLcp15_221-OMcp15_321*
 RLcp15_245-OMcp15_36*RLcp15_219;
    VIcp15_245 = qd[2]-OMcp15_120*RLcp15_321-OMcp15_121*RLcp15_345-OMcp15_16*RLcp15_319+OMcp15_320*RLcp15_121+OMcp15_321*
 RLcp15_145+OMcp15_36*RLcp15_119;
    VIcp15_345 = qd[3]+OMcp15_120*RLcp15_221+OMcp15_121*RLcp15_245+OMcp15_16*RLcp15_219-OMcp15_220*RLcp15_121-OMcp15_221*
 RLcp15_145-OMcp15_26*RLcp15_119;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_145;
    sens->P[2] = POcp15_245;
    sens->P[3] = POcp15_345;
    sens->V[1] = VIcp15_145;
    sens->V[2] = VIcp15_245;
    sens->V[3] = VIcp15_345;
    sens->OM[1] = OMcp15_121;
    sens->OM[2] = OMcp15_221;
    sens->OM[3] = OMcp15_321;
 
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


    ROcp16_419 = ROcp16_46*C19+S19*S5;
    ROcp16_519 = ROcp16_56*C19+ROcp16_85*S19;
    ROcp16_619 = ROcp16_66*C19+ROcp16_95*S19;
    ROcp16_719 = -(ROcp16_46*S19-C19*S5);
    ROcp16_819 = -(ROcp16_56*S19-ROcp16_85*C19);
    ROcp16_919 = -(ROcp16_66*S19-ROcp16_95*C19);
    ROcp16_120 = ROcp16_16*C20-ROcp16_719*S20;
    ROcp16_220 = ROcp16_26*C20-ROcp16_819*S20;
    ROcp16_320 = ROcp16_36*C20-ROcp16_919*S20;
    ROcp16_720 = ROcp16_16*S20+ROcp16_719*C20;
    ROcp16_820 = ROcp16_26*S20+ROcp16_819*C20;
    ROcp16_920 = ROcp16_36*S20+ROcp16_919*C20;
    ROcp16_121 = ROcp16_120*C21+ROcp16_419*S21;
    ROcp16_221 = ROcp16_220*C21+ROcp16_519*S21;
    ROcp16_321 = ROcp16_320*C21+ROcp16_619*S21;
    ROcp16_421 = -(ROcp16_120*S21-ROcp16_419*C21);
    ROcp16_521 = -(ROcp16_220*S21-ROcp16_519*C21);
    ROcp16_621 = -(ROcp16_320*S21-ROcp16_619*C21);
    RLcp16_119 = ROcp16_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp16_219 = ROcp16_26*s->dpt[1][4]+ROcp16_85*s->dpt[3][4];
    RLcp16_319 = ROcp16_36*s->dpt[1][4]+ROcp16_95*s->dpt[3][4];
    OMcp16_120 = OMcp16_16+ROcp16_16*qd[19]+ROcp16_419*qd[20];
    OMcp16_220 = OMcp16_26+ROcp16_26*qd[19]+ROcp16_519*qd[20];
    OMcp16_320 = OMcp16_36+ROcp16_36*qd[19]+ROcp16_619*qd[20];
    RLcp16_121 = ROcp16_720*s->dpt[3][36];
    RLcp16_221 = ROcp16_820*s->dpt[3][36];
    RLcp16_321 = ROcp16_920*s->dpt[3][36];
    OMcp16_121 = OMcp16_120+ROcp16_720*qd[21];
    OMcp16_221 = OMcp16_220+ROcp16_820*qd[21];
    OMcp16_321 = OMcp16_320+ROcp16_920*qd[21];

// = = Block_1_0_0_17_0_5 = = 
 
// Sensor Kinematics 


    RLcp16_122 = ROcp16_121*s->dpt[1][40]+ROcp16_421*s->dpt[2][40]+ROcp16_720*s->dpt[3][40];
    RLcp16_222 = ROcp16_221*s->dpt[1][40]+ROcp16_521*s->dpt[2][40]+ROcp16_820*s->dpt[3][40];
    RLcp16_322 = ROcp16_321*s->dpt[1][40]+ROcp16_621*s->dpt[2][40]+ROcp16_920*s->dpt[3][40];
    OMcp16_122 = OMcp16_121+ROcp16_421*qd[22];
    OMcp16_222 = OMcp16_221+ROcp16_521*qd[22];
    OMcp16_322 = OMcp16_321+ROcp16_621*qd[22];
    RLcp16_146 = ROcp16_421*s->dpt[2][42]+s->dpt[1][42]*(ROcp16_121*C22-ROcp16_720*S22)+s->dpt[3][42]*(ROcp16_121*S22+
 ROcp16_720*C22);
    RLcp16_246 = ROcp16_521*s->dpt[2][42]+s->dpt[1][42]*(ROcp16_221*C22-ROcp16_820*S22)+s->dpt[3][42]*(ROcp16_221*S22+
 ROcp16_820*C22);
    RLcp16_346 = ROcp16_621*s->dpt[2][42]+s->dpt[1][42]*(ROcp16_321*C22-ROcp16_920*S22)+s->dpt[3][42]*(ROcp16_321*S22+
 ROcp16_920*C22);
    POcp16_146 = RLcp16_119+RLcp16_121+RLcp16_122+RLcp16_146+q[1];
    POcp16_246 = RLcp16_219+RLcp16_221+RLcp16_222+RLcp16_246+q[2];
    POcp16_346 = RLcp16_319+RLcp16_321+RLcp16_322+RLcp16_346+q[3];
    VIcp16_146 = qd[1]+OMcp16_220*RLcp16_321+OMcp16_221*RLcp16_322+OMcp16_222*RLcp16_346+OMcp16_26*RLcp16_319-OMcp16_320*
 RLcp16_221-OMcp16_321*RLcp16_222-OMcp16_322*RLcp16_246-OMcp16_36*RLcp16_219;
    VIcp16_246 = qd[2]-OMcp16_120*RLcp16_321-OMcp16_121*RLcp16_322-OMcp16_122*RLcp16_346-OMcp16_16*RLcp16_319+OMcp16_320*
 RLcp16_121+OMcp16_321*RLcp16_122+OMcp16_322*RLcp16_146+OMcp16_36*RLcp16_119;
    VIcp16_346 = qd[3]+OMcp16_120*RLcp16_221+OMcp16_121*RLcp16_222+OMcp16_122*RLcp16_246+OMcp16_16*RLcp16_219-OMcp16_220*
 RLcp16_121-OMcp16_221*RLcp16_122-OMcp16_222*RLcp16_146-OMcp16_26*RLcp16_119;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_146;
    sens->P[2] = POcp16_246;
    sens->P[3] = POcp16_346;
    sens->V[1] = VIcp16_146;
    sens->V[2] = VIcp16_246;
    sens->V[3] = VIcp16_346;
    sens->OM[1] = OMcp16_122;
    sens->OM[2] = OMcp16_222;
    sens->OM[3] = OMcp16_322;
 
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


    ROcp17_419 = ROcp17_46*C19+S19*S5;
    ROcp17_519 = ROcp17_56*C19+ROcp17_85*S19;
    ROcp17_619 = ROcp17_66*C19+ROcp17_95*S19;
    ROcp17_719 = -(ROcp17_46*S19-C19*S5);
    ROcp17_819 = -(ROcp17_56*S19-ROcp17_85*C19);
    ROcp17_919 = -(ROcp17_66*S19-ROcp17_95*C19);
    ROcp17_120 = ROcp17_16*C20-ROcp17_719*S20;
    ROcp17_220 = ROcp17_26*C20-ROcp17_819*S20;
    ROcp17_320 = ROcp17_36*C20-ROcp17_919*S20;
    ROcp17_720 = ROcp17_16*S20+ROcp17_719*C20;
    ROcp17_820 = ROcp17_26*S20+ROcp17_819*C20;
    ROcp17_920 = ROcp17_36*S20+ROcp17_919*C20;
    ROcp17_121 = ROcp17_120*C21+ROcp17_419*S21;
    ROcp17_221 = ROcp17_220*C21+ROcp17_519*S21;
    ROcp17_321 = ROcp17_320*C21+ROcp17_619*S21;
    ROcp17_421 = -(ROcp17_120*S21-ROcp17_419*C21);
    ROcp17_521 = -(ROcp17_220*S21-ROcp17_519*C21);
    ROcp17_621 = -(ROcp17_320*S21-ROcp17_619*C21);
    RLcp17_119 = ROcp17_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp17_219 = ROcp17_26*s->dpt[1][4]+ROcp17_85*s->dpt[3][4];
    RLcp17_319 = ROcp17_36*s->dpt[1][4]+ROcp17_95*s->dpt[3][4];
    OMcp17_120 = OMcp17_16+ROcp17_16*qd[19]+ROcp17_419*qd[20];
    OMcp17_220 = OMcp17_26+ROcp17_26*qd[19]+ROcp17_519*qd[20];
    OMcp17_320 = OMcp17_36+ROcp17_36*qd[19]+ROcp17_619*qd[20];
    RLcp17_121 = ROcp17_720*s->dpt[3][36];
    RLcp17_221 = ROcp17_820*s->dpt[3][36];
    RLcp17_321 = ROcp17_920*s->dpt[3][36];
    OMcp17_121 = OMcp17_120+ROcp17_720*qd[21];
    OMcp17_221 = OMcp17_220+ROcp17_820*qd[21];
    OMcp17_321 = OMcp17_320+ROcp17_920*qd[21];

// = = Block_1_0_0_18_0_5 = = 
 
// Sensor Kinematics 


    ROcp17_122 = ROcp17_121*C22-ROcp17_720*S22;
    ROcp17_222 = ROcp17_221*C22-ROcp17_820*S22;
    ROcp17_322 = ROcp17_321*C22-ROcp17_920*S22;
    ROcp17_722 = ROcp17_121*S22+ROcp17_720*C22;
    ROcp17_822 = ROcp17_221*S22+ROcp17_820*C22;
    ROcp17_922 = ROcp17_321*S22+ROcp17_920*C22;
    RLcp17_122 = ROcp17_121*s->dpt[1][40]+ROcp17_421*s->dpt[2][40]+ROcp17_720*s->dpt[3][40];
    RLcp17_222 = ROcp17_221*s->dpt[1][40]+ROcp17_521*s->dpt[2][40]+ROcp17_820*s->dpt[3][40];
    RLcp17_322 = ROcp17_321*s->dpt[1][40]+ROcp17_621*s->dpt[2][40]+ROcp17_920*s->dpt[3][40];
    OMcp17_122 = OMcp17_121+ROcp17_421*qd[22];
    OMcp17_222 = OMcp17_221+ROcp17_521*qd[22];
    OMcp17_322 = OMcp17_321+ROcp17_621*qd[22];
    RLcp17_123 = ROcp17_421*s->dpt[2][43];
    RLcp17_223 = ROcp17_521*s->dpt[2][43];
    RLcp17_323 = ROcp17_621*s->dpt[2][43];
    OMcp17_123 = OMcp17_122+ROcp17_122*qd[23];
    OMcp17_223 = OMcp17_222+ROcp17_222*qd[23];
    OMcp17_323 = OMcp17_322+ROcp17_322*qd[23];
    RLcp17_147 = ROcp17_122*s->dpt[1][44]+s->dpt[2][44]*(ROcp17_421*C23+ROcp17_722*S23)-s->dpt[3][44]*(ROcp17_421*S23-
 ROcp17_722*C23);
    RLcp17_247 = ROcp17_222*s->dpt[1][44]+s->dpt[2][44]*(ROcp17_521*C23+ROcp17_822*S23)-s->dpt[3][44]*(ROcp17_521*S23-
 ROcp17_822*C23);
    RLcp17_347 = ROcp17_322*s->dpt[1][44]+s->dpt[2][44]*(ROcp17_621*C23+ROcp17_922*S23)-s->dpt[3][44]*(ROcp17_621*S23-
 ROcp17_922*C23);
    POcp17_147 = RLcp17_119+RLcp17_121+RLcp17_122+RLcp17_123+RLcp17_147+q[1];
    POcp17_247 = RLcp17_219+RLcp17_221+RLcp17_222+RLcp17_223+RLcp17_247+q[2];
    POcp17_347 = RLcp17_319+RLcp17_321+RLcp17_322+RLcp17_323+RLcp17_347+q[3];
    VIcp17_147 = qd[1]+OMcp17_220*RLcp17_321+OMcp17_221*RLcp17_322+OMcp17_222*RLcp17_323+OMcp17_223*RLcp17_347+OMcp17_26*
 RLcp17_319-OMcp17_320*RLcp17_221-OMcp17_321*RLcp17_222-OMcp17_322*RLcp17_223-OMcp17_323*RLcp17_247-OMcp17_36*RLcp17_219;
    VIcp17_247 = qd[2]-OMcp17_120*RLcp17_321-OMcp17_121*RLcp17_322-OMcp17_122*RLcp17_323-OMcp17_123*RLcp17_347-OMcp17_16*
 RLcp17_319+OMcp17_320*RLcp17_121+OMcp17_321*RLcp17_122+OMcp17_322*RLcp17_123+OMcp17_323*RLcp17_147+OMcp17_36*RLcp17_119;
    VIcp17_347 = qd[3]+OMcp17_120*RLcp17_221+OMcp17_121*RLcp17_222+OMcp17_122*RLcp17_223+OMcp17_123*RLcp17_247+OMcp17_16*
 RLcp17_219-OMcp17_220*RLcp17_121-OMcp17_221*RLcp17_122-OMcp17_222*RLcp17_123-OMcp17_223*RLcp17_147-OMcp17_26*RLcp17_119;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_147;
    sens->P[2] = POcp17_247;
    sens->P[3] = POcp17_347;
    sens->V[1] = VIcp17_147;
    sens->V[2] = VIcp17_247;
    sens->V[3] = VIcp17_347;
    sens->OM[1] = OMcp17_123;
    sens->OM[2] = OMcp17_223;
    sens->OM[3] = OMcp17_323;
 
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


    ROcp18_419 = ROcp18_46*C19+S19*S5;
    ROcp18_519 = ROcp18_56*C19+ROcp18_85*S19;
    ROcp18_619 = ROcp18_66*C19+ROcp18_95*S19;
    ROcp18_719 = -(ROcp18_46*S19-C19*S5);
    ROcp18_819 = -(ROcp18_56*S19-ROcp18_85*C19);
    ROcp18_919 = -(ROcp18_66*S19-ROcp18_95*C19);
    ROcp18_120 = ROcp18_16*C20-ROcp18_719*S20;
    ROcp18_220 = ROcp18_26*C20-ROcp18_819*S20;
    ROcp18_320 = ROcp18_36*C20-ROcp18_919*S20;
    ROcp18_720 = ROcp18_16*S20+ROcp18_719*C20;
    ROcp18_820 = ROcp18_26*S20+ROcp18_819*C20;
    ROcp18_920 = ROcp18_36*S20+ROcp18_919*C20;
    ROcp18_121 = ROcp18_120*C21+ROcp18_419*S21;
    ROcp18_221 = ROcp18_220*C21+ROcp18_519*S21;
    ROcp18_321 = ROcp18_320*C21+ROcp18_619*S21;
    ROcp18_421 = -(ROcp18_120*S21-ROcp18_419*C21);
    ROcp18_521 = -(ROcp18_220*S21-ROcp18_519*C21);
    ROcp18_621 = -(ROcp18_320*S21-ROcp18_619*C21);
    RLcp18_119 = ROcp18_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp18_219 = ROcp18_26*s->dpt[1][4]+ROcp18_85*s->dpt[3][4];
    RLcp18_319 = ROcp18_36*s->dpt[1][4]+ROcp18_95*s->dpt[3][4];
    OMcp18_120 = OMcp18_16+ROcp18_16*qd[19]+ROcp18_419*qd[20];
    OMcp18_220 = OMcp18_26+ROcp18_26*qd[19]+ROcp18_519*qd[20];
    OMcp18_320 = OMcp18_36+ROcp18_36*qd[19]+ROcp18_619*qd[20];
    RLcp18_121 = ROcp18_720*s->dpt[3][36];
    RLcp18_221 = ROcp18_820*s->dpt[3][36];
    RLcp18_321 = ROcp18_920*s->dpt[3][36];
    OMcp18_121 = OMcp18_120+ROcp18_720*qd[21];
    OMcp18_221 = OMcp18_220+ROcp18_820*qd[21];
    OMcp18_321 = OMcp18_320+ROcp18_920*qd[21];

// = = Block_1_0_0_19_0_5 = = 
 
// Sensor Kinematics 


    ROcp18_122 = ROcp18_121*C22-ROcp18_720*S22;
    ROcp18_222 = ROcp18_221*C22-ROcp18_820*S22;
    ROcp18_322 = ROcp18_321*C22-ROcp18_920*S22;
    ROcp18_722 = ROcp18_121*S22+ROcp18_720*C22;
    ROcp18_822 = ROcp18_221*S22+ROcp18_820*C22;
    ROcp18_922 = ROcp18_321*S22+ROcp18_920*C22;
    ROcp18_423 = ROcp18_421*C23+ROcp18_722*S23;
    ROcp18_523 = ROcp18_521*C23+ROcp18_822*S23;
    ROcp18_623 = ROcp18_621*C23+ROcp18_922*S23;
    ROcp18_723 = -(ROcp18_421*S23-ROcp18_722*C23);
    ROcp18_823 = -(ROcp18_521*S23-ROcp18_822*C23);
    ROcp18_923 = -(ROcp18_621*S23-ROcp18_922*C23);
    RLcp18_122 = ROcp18_121*s->dpt[1][40]+ROcp18_421*s->dpt[2][40]+ROcp18_720*s->dpt[3][40];
    RLcp18_222 = ROcp18_221*s->dpt[1][40]+ROcp18_521*s->dpt[2][40]+ROcp18_820*s->dpt[3][40];
    RLcp18_322 = ROcp18_321*s->dpt[1][40]+ROcp18_621*s->dpt[2][40]+ROcp18_920*s->dpt[3][40];
    OMcp18_122 = OMcp18_121+ROcp18_421*qd[22];
    OMcp18_222 = OMcp18_221+ROcp18_521*qd[22];
    OMcp18_322 = OMcp18_321+ROcp18_621*qd[22];
    RLcp18_123 = ROcp18_421*s->dpt[2][43];
    RLcp18_223 = ROcp18_521*s->dpt[2][43];
    RLcp18_323 = ROcp18_621*s->dpt[2][43];
    OMcp18_123 = OMcp18_122+ROcp18_122*qd[23];
    OMcp18_223 = OMcp18_222+ROcp18_222*qd[23];
    OMcp18_323 = OMcp18_322+ROcp18_322*qd[23];
    RLcp18_124 = ROcp18_723*s->dpt[3][45];
    RLcp18_224 = ROcp18_823*s->dpt[3][45];
    RLcp18_324 = ROcp18_923*s->dpt[3][45];
    OMcp18_124 = OMcp18_123+ROcp18_723*qd[24];
    OMcp18_224 = OMcp18_223+ROcp18_823*qd[24];
    OMcp18_324 = OMcp18_323+ROcp18_923*qd[24];
    RLcp18_148 = ROcp18_723*s->dpt[3][47]+s->dpt[1][47]*(ROcp18_122*C24+ROcp18_423*S24)-s->dpt[2][47]*(ROcp18_122*S24-
 ROcp18_423*C24);
    RLcp18_248 = ROcp18_823*s->dpt[3][47]+s->dpt[1][47]*(ROcp18_222*C24+ROcp18_523*S24)-s->dpt[2][47]*(ROcp18_222*S24-
 ROcp18_523*C24);
    RLcp18_348 = ROcp18_923*s->dpt[3][47]+s->dpt[1][47]*(ROcp18_322*C24+ROcp18_623*S24)-s->dpt[2][47]*(ROcp18_322*S24-
 ROcp18_623*C24);
    POcp18_148 = RLcp18_119+RLcp18_121+RLcp18_122+RLcp18_123+RLcp18_124+RLcp18_148+q[1];
    POcp18_248 = RLcp18_219+RLcp18_221+RLcp18_222+RLcp18_223+RLcp18_224+RLcp18_248+q[2];
    POcp18_348 = RLcp18_319+RLcp18_321+RLcp18_322+RLcp18_323+RLcp18_324+RLcp18_348+q[3];
    VIcp18_148 = qd[1]+OMcp18_220*RLcp18_321+OMcp18_221*RLcp18_322+OMcp18_222*RLcp18_323+OMcp18_223*RLcp18_324+OMcp18_224*
 RLcp18_348+OMcp18_26*RLcp18_319-OMcp18_320*RLcp18_221-OMcp18_321*RLcp18_222-OMcp18_322*RLcp18_223-OMcp18_323*RLcp18_224-
 OMcp18_324*RLcp18_248-OMcp18_36*RLcp18_219;
    VIcp18_248 = qd[2]-OMcp18_120*RLcp18_321-OMcp18_121*RLcp18_322-OMcp18_122*RLcp18_323-OMcp18_123*RLcp18_324-OMcp18_124*
 RLcp18_348-OMcp18_16*RLcp18_319+OMcp18_320*RLcp18_121+OMcp18_321*RLcp18_122+OMcp18_322*RLcp18_123+OMcp18_323*RLcp18_124+
 OMcp18_324*RLcp18_148+OMcp18_36*RLcp18_119;
    VIcp18_348 = qd[3]+OMcp18_120*RLcp18_221+OMcp18_121*RLcp18_222+OMcp18_122*RLcp18_223+OMcp18_123*RLcp18_224+OMcp18_124*
 RLcp18_248+OMcp18_16*RLcp18_219-OMcp18_220*RLcp18_121-OMcp18_221*RLcp18_122-OMcp18_222*RLcp18_123-OMcp18_223*RLcp18_124-
 OMcp18_224*RLcp18_148-OMcp18_26*RLcp18_119;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_148;
    sens->P[2] = POcp18_248;
    sens->P[3] = POcp18_348;
    sens->V[1] = VIcp18_148;
    sens->V[2] = VIcp18_248;
    sens->V[3] = VIcp18_348;
    sens->OM[1] = OMcp18_124;
    sens->OM[2] = OMcp18_224;
    sens->OM[3] = OMcp18_324;
 
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


    ROcp19_419 = ROcp19_46*C19+S19*S5;
    ROcp19_519 = ROcp19_56*C19+ROcp19_85*S19;
    ROcp19_619 = ROcp19_66*C19+ROcp19_95*S19;
    ROcp19_719 = -(ROcp19_46*S19-C19*S5);
    ROcp19_819 = -(ROcp19_56*S19-ROcp19_85*C19);
    ROcp19_919 = -(ROcp19_66*S19-ROcp19_95*C19);
    ROcp19_120 = ROcp19_16*C20-ROcp19_719*S20;
    ROcp19_220 = ROcp19_26*C20-ROcp19_819*S20;
    ROcp19_320 = ROcp19_36*C20-ROcp19_919*S20;
    ROcp19_720 = ROcp19_16*S20+ROcp19_719*C20;
    ROcp19_820 = ROcp19_26*S20+ROcp19_819*C20;
    ROcp19_920 = ROcp19_36*S20+ROcp19_919*C20;
    ROcp19_121 = ROcp19_120*C21+ROcp19_419*S21;
    ROcp19_221 = ROcp19_220*C21+ROcp19_519*S21;
    ROcp19_321 = ROcp19_320*C21+ROcp19_619*S21;
    ROcp19_421 = -(ROcp19_120*S21-ROcp19_419*C21);
    ROcp19_521 = -(ROcp19_220*S21-ROcp19_519*C21);
    ROcp19_621 = -(ROcp19_320*S21-ROcp19_619*C21);
    RLcp19_119 = ROcp19_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp19_219 = ROcp19_26*s->dpt[1][4]+ROcp19_85*s->dpt[3][4];
    RLcp19_319 = ROcp19_36*s->dpt[1][4]+ROcp19_95*s->dpt[3][4];
    OMcp19_120 = OMcp19_16+ROcp19_16*qd[19]+ROcp19_419*qd[20];
    OMcp19_220 = OMcp19_26+ROcp19_26*qd[19]+ROcp19_519*qd[20];
    OMcp19_320 = OMcp19_36+ROcp19_36*qd[19]+ROcp19_619*qd[20];
    RLcp19_121 = ROcp19_720*s->dpt[3][36];
    RLcp19_221 = ROcp19_820*s->dpt[3][36];
    RLcp19_321 = ROcp19_920*s->dpt[3][36];
    OMcp19_121 = OMcp19_120+ROcp19_720*qd[21];
    OMcp19_221 = OMcp19_220+ROcp19_820*qd[21];
    OMcp19_321 = OMcp19_320+ROcp19_920*qd[21];

// = = Block_1_0_0_20_0_5 = = 
 
// Sensor Kinematics 


    ROcp19_122 = ROcp19_121*C22-ROcp19_720*S22;
    ROcp19_222 = ROcp19_221*C22-ROcp19_820*S22;
    ROcp19_322 = ROcp19_321*C22-ROcp19_920*S22;
    ROcp19_722 = ROcp19_121*S22+ROcp19_720*C22;
    ROcp19_822 = ROcp19_221*S22+ROcp19_820*C22;
    ROcp19_922 = ROcp19_321*S22+ROcp19_920*C22;
    ROcp19_423 = ROcp19_421*C23+ROcp19_722*S23;
    ROcp19_523 = ROcp19_521*C23+ROcp19_822*S23;
    ROcp19_623 = ROcp19_621*C23+ROcp19_922*S23;
    ROcp19_723 = -(ROcp19_421*S23-ROcp19_722*C23);
    ROcp19_823 = -(ROcp19_521*S23-ROcp19_822*C23);
    ROcp19_923 = -(ROcp19_621*S23-ROcp19_922*C23);
    ROcp19_124 = ROcp19_122*C24+ROcp19_423*S24;
    ROcp19_224 = ROcp19_222*C24+ROcp19_523*S24;
    ROcp19_324 = ROcp19_322*C24+ROcp19_623*S24;
    ROcp19_424 = -(ROcp19_122*S24-ROcp19_423*C24);
    ROcp19_524 = -(ROcp19_222*S24-ROcp19_523*C24);
    ROcp19_624 = -(ROcp19_322*S24-ROcp19_623*C24);
    RLcp19_122 = ROcp19_121*s->dpt[1][40]+ROcp19_421*s->dpt[2][40]+ROcp19_720*s->dpt[3][40];
    RLcp19_222 = ROcp19_221*s->dpt[1][40]+ROcp19_521*s->dpt[2][40]+ROcp19_820*s->dpt[3][40];
    RLcp19_322 = ROcp19_321*s->dpt[1][40]+ROcp19_621*s->dpt[2][40]+ROcp19_920*s->dpt[3][40];
    OMcp19_122 = OMcp19_121+ROcp19_421*qd[22];
    OMcp19_222 = OMcp19_221+ROcp19_521*qd[22];
    OMcp19_322 = OMcp19_321+ROcp19_621*qd[22];
    RLcp19_123 = ROcp19_421*s->dpt[2][43];
    RLcp19_223 = ROcp19_521*s->dpt[2][43];
    RLcp19_323 = ROcp19_621*s->dpt[2][43];
    OMcp19_123 = OMcp19_122+ROcp19_122*qd[23];
    OMcp19_223 = OMcp19_222+ROcp19_222*qd[23];
    OMcp19_323 = OMcp19_322+ROcp19_322*qd[23];
    RLcp19_124 = ROcp19_723*s->dpt[3][45];
    RLcp19_224 = ROcp19_823*s->dpt[3][45];
    RLcp19_324 = ROcp19_923*s->dpt[3][45];
    OMcp19_124 = OMcp19_123+ROcp19_723*qd[24];
    OMcp19_224 = OMcp19_223+ROcp19_823*qd[24];
    OMcp19_324 = OMcp19_323+ROcp19_923*qd[24];
    RLcp19_125 = ROcp19_723*s->dpt[3][48];
    RLcp19_225 = ROcp19_823*s->dpt[3][48];
    RLcp19_325 = ROcp19_923*s->dpt[3][48];
    OMcp19_125 = OMcp19_124+ROcp19_424*qd[25];
    OMcp19_225 = OMcp19_224+ROcp19_524*qd[25];
    OMcp19_325 = OMcp19_324+ROcp19_624*qd[25];
    RLcp19_149 = ROcp19_424*s->dpt[2][50]+s->dpt[1][50]*(ROcp19_124*C25-ROcp19_723*S25)+s->dpt[3][50]*(ROcp19_124*S25+
 ROcp19_723*C25);
    RLcp19_249 = ROcp19_524*s->dpt[2][50]+s->dpt[1][50]*(ROcp19_224*C25-ROcp19_823*S25)+s->dpt[3][50]*(ROcp19_224*S25+
 ROcp19_823*C25);
    RLcp19_349 = ROcp19_624*s->dpt[2][50]+s->dpt[1][50]*(ROcp19_324*C25-ROcp19_923*S25)+s->dpt[3][50]*(ROcp19_324*S25+
 ROcp19_923*C25);
    POcp19_149 = RLcp19_119+RLcp19_121+RLcp19_122+RLcp19_123+RLcp19_124+RLcp19_125+RLcp19_149+q[1];
    POcp19_249 = RLcp19_219+RLcp19_221+RLcp19_222+RLcp19_223+RLcp19_224+RLcp19_225+RLcp19_249+q[2];
    POcp19_349 = RLcp19_319+RLcp19_321+RLcp19_322+RLcp19_323+RLcp19_324+RLcp19_325+RLcp19_349+q[3];
    VIcp19_149 = qd[1]+OMcp19_220*RLcp19_321+OMcp19_221*RLcp19_322+OMcp19_222*RLcp19_323+OMcp19_223*RLcp19_324+OMcp19_224*
 RLcp19_325+OMcp19_225*RLcp19_349+OMcp19_26*RLcp19_319-OMcp19_320*RLcp19_221-OMcp19_321*RLcp19_222-OMcp19_322*RLcp19_223-
 OMcp19_323*RLcp19_224-OMcp19_324*RLcp19_225-OMcp19_325*RLcp19_249-OMcp19_36*RLcp19_219;
    VIcp19_249 = qd[2]-OMcp19_120*RLcp19_321-OMcp19_121*RLcp19_322-OMcp19_122*RLcp19_323-OMcp19_123*RLcp19_324-OMcp19_124*
 RLcp19_325-OMcp19_125*RLcp19_349-OMcp19_16*RLcp19_319+OMcp19_320*RLcp19_121+OMcp19_321*RLcp19_122+OMcp19_322*RLcp19_123+
 OMcp19_323*RLcp19_124+OMcp19_324*RLcp19_125+OMcp19_325*RLcp19_149+OMcp19_36*RLcp19_119;
    VIcp19_349 = qd[3]+OMcp19_120*RLcp19_221+OMcp19_121*RLcp19_222+OMcp19_122*RLcp19_223+OMcp19_123*RLcp19_224+OMcp19_124*
 RLcp19_225+OMcp19_125*RLcp19_249+OMcp19_16*RLcp19_219-OMcp19_220*RLcp19_121-OMcp19_221*RLcp19_122-OMcp19_222*RLcp19_123-
 OMcp19_223*RLcp19_124-OMcp19_224*RLcp19_125-OMcp19_225*RLcp19_149-OMcp19_26*RLcp19_119;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_149;
    sens->P[2] = POcp19_249;
    sens->P[3] = POcp19_349;
    sens->V[1] = VIcp19_149;
    sens->V[2] = VIcp19_249;
    sens->V[3] = VIcp19_349;
    sens->OM[1] = OMcp19_125;
    sens->OM[2] = OMcp19_225;
    sens->OM[3] = OMcp19_325;
 
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


    ROcp20_419 = ROcp20_46*C19+S19*S5;
    ROcp20_519 = ROcp20_56*C19+ROcp20_85*S19;
    ROcp20_619 = ROcp20_66*C19+ROcp20_95*S19;
    ROcp20_719 = -(ROcp20_46*S19-C19*S5);
    ROcp20_819 = -(ROcp20_56*S19-ROcp20_85*C19);
    ROcp20_919 = -(ROcp20_66*S19-ROcp20_95*C19);
    ROcp20_120 = ROcp20_16*C20-ROcp20_719*S20;
    ROcp20_220 = ROcp20_26*C20-ROcp20_819*S20;
    ROcp20_320 = ROcp20_36*C20-ROcp20_919*S20;
    ROcp20_720 = ROcp20_16*S20+ROcp20_719*C20;
    ROcp20_820 = ROcp20_26*S20+ROcp20_819*C20;
    ROcp20_920 = ROcp20_36*S20+ROcp20_919*C20;
    ROcp20_121 = ROcp20_120*C21+ROcp20_419*S21;
    ROcp20_221 = ROcp20_220*C21+ROcp20_519*S21;
    ROcp20_321 = ROcp20_320*C21+ROcp20_619*S21;
    ROcp20_421 = -(ROcp20_120*S21-ROcp20_419*C21);
    ROcp20_521 = -(ROcp20_220*S21-ROcp20_519*C21);
    ROcp20_621 = -(ROcp20_320*S21-ROcp20_619*C21);
    RLcp20_119 = ROcp20_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp20_219 = ROcp20_26*s->dpt[1][4]+ROcp20_85*s->dpt[3][4];
    RLcp20_319 = ROcp20_36*s->dpt[1][4]+ROcp20_95*s->dpt[3][4];
    OMcp20_120 = OMcp20_16+ROcp20_16*qd[19]+ROcp20_419*qd[20];
    OMcp20_220 = OMcp20_26+ROcp20_26*qd[19]+ROcp20_519*qd[20];
    OMcp20_320 = OMcp20_36+ROcp20_36*qd[19]+ROcp20_619*qd[20];
    RLcp20_121 = ROcp20_720*s->dpt[3][36];
    RLcp20_221 = ROcp20_820*s->dpt[3][36];
    RLcp20_321 = ROcp20_920*s->dpt[3][36];
    OMcp20_121 = OMcp20_120+ROcp20_720*qd[21];
    OMcp20_221 = OMcp20_220+ROcp20_820*qd[21];
    OMcp20_321 = OMcp20_320+ROcp20_920*qd[21];

// = = Block_1_0_0_21_0_6 = = 
 
// Sensor Kinematics 


    RLcp20_126 = ROcp20_121*s->dpt[1][41]+ROcp20_421*s->dpt[2][41]+ROcp20_720*s->dpt[3][41];
    RLcp20_226 = ROcp20_221*s->dpt[1][41]+ROcp20_521*s->dpt[2][41]+ROcp20_820*s->dpt[3][41];
    RLcp20_326 = ROcp20_321*s->dpt[1][41]+ROcp20_621*s->dpt[2][41]+ROcp20_920*s->dpt[3][41];
    OMcp20_126 = OMcp20_121+ROcp20_421*qd[26];
    OMcp20_226 = OMcp20_221+ROcp20_521*qd[26];
    OMcp20_326 = OMcp20_321+ROcp20_621*qd[26];
    RLcp20_150 = ROcp20_421*s->dpt[2][51]+s->dpt[1][51]*(ROcp20_121*C26-ROcp20_720*S26)+s->dpt[3][51]*(ROcp20_121*S26+
 ROcp20_720*C26);
    RLcp20_250 = ROcp20_521*s->dpt[2][51]+s->dpt[1][51]*(ROcp20_221*C26-ROcp20_820*S26)+s->dpt[3][51]*(ROcp20_221*S26+
 ROcp20_820*C26);
    RLcp20_350 = ROcp20_621*s->dpt[2][51]+s->dpt[1][51]*(ROcp20_321*C26-ROcp20_920*S26)+s->dpt[3][51]*(ROcp20_321*S26+
 ROcp20_920*C26);
    POcp20_150 = RLcp20_119+RLcp20_121+RLcp20_126+RLcp20_150+q[1];
    POcp20_250 = RLcp20_219+RLcp20_221+RLcp20_226+RLcp20_250+q[2];
    POcp20_350 = RLcp20_319+RLcp20_321+RLcp20_326+RLcp20_350+q[3];
    VIcp20_150 = qd[1]+OMcp20_220*RLcp20_321+OMcp20_221*RLcp20_326+OMcp20_226*RLcp20_350+OMcp20_26*RLcp20_319-OMcp20_320*
 RLcp20_221-OMcp20_321*RLcp20_226-OMcp20_326*RLcp20_250-OMcp20_36*RLcp20_219;
    VIcp20_250 = qd[2]-OMcp20_120*RLcp20_321-OMcp20_121*RLcp20_326-OMcp20_126*RLcp20_350-OMcp20_16*RLcp20_319+OMcp20_320*
 RLcp20_121+OMcp20_321*RLcp20_126+OMcp20_326*RLcp20_150+OMcp20_36*RLcp20_119;
    VIcp20_350 = qd[3]+OMcp20_120*RLcp20_221+OMcp20_121*RLcp20_226+OMcp20_126*RLcp20_250+OMcp20_16*RLcp20_219-OMcp20_220*
 RLcp20_121-OMcp20_221*RLcp20_126-OMcp20_226*RLcp20_150-OMcp20_26*RLcp20_119;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_150;
    sens->P[2] = POcp20_250;
    sens->P[3] = POcp20_350;
    sens->V[1] = VIcp20_150;
    sens->V[2] = VIcp20_250;
    sens->V[3] = VIcp20_350;
    sens->OM[1] = OMcp20_126;
    sens->OM[2] = OMcp20_226;
    sens->OM[3] = OMcp20_326;
 
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


    ROcp21_419 = ROcp21_46*C19+S19*S5;
    ROcp21_519 = ROcp21_56*C19+ROcp21_85*S19;
    ROcp21_619 = ROcp21_66*C19+ROcp21_95*S19;
    ROcp21_719 = -(ROcp21_46*S19-C19*S5);
    ROcp21_819 = -(ROcp21_56*S19-ROcp21_85*C19);
    ROcp21_919 = -(ROcp21_66*S19-ROcp21_95*C19);
    ROcp21_120 = ROcp21_16*C20-ROcp21_719*S20;
    ROcp21_220 = ROcp21_26*C20-ROcp21_819*S20;
    ROcp21_320 = ROcp21_36*C20-ROcp21_919*S20;
    ROcp21_720 = ROcp21_16*S20+ROcp21_719*C20;
    ROcp21_820 = ROcp21_26*S20+ROcp21_819*C20;
    ROcp21_920 = ROcp21_36*S20+ROcp21_919*C20;
    ROcp21_121 = ROcp21_120*C21+ROcp21_419*S21;
    ROcp21_221 = ROcp21_220*C21+ROcp21_519*S21;
    ROcp21_321 = ROcp21_320*C21+ROcp21_619*S21;
    ROcp21_421 = -(ROcp21_120*S21-ROcp21_419*C21);
    ROcp21_521 = -(ROcp21_220*S21-ROcp21_519*C21);
    ROcp21_621 = -(ROcp21_320*S21-ROcp21_619*C21);
    RLcp21_119 = ROcp21_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp21_219 = ROcp21_26*s->dpt[1][4]+ROcp21_85*s->dpt[3][4];
    RLcp21_319 = ROcp21_36*s->dpt[1][4]+ROcp21_95*s->dpt[3][4];
    OMcp21_120 = OMcp21_16+ROcp21_16*qd[19]+ROcp21_419*qd[20];
    OMcp21_220 = OMcp21_26+ROcp21_26*qd[19]+ROcp21_519*qd[20];
    OMcp21_320 = OMcp21_36+ROcp21_36*qd[19]+ROcp21_619*qd[20];
    RLcp21_121 = ROcp21_720*s->dpt[3][36];
    RLcp21_221 = ROcp21_820*s->dpt[3][36];
    RLcp21_321 = ROcp21_920*s->dpt[3][36];
    OMcp21_121 = OMcp21_120+ROcp21_720*qd[21];
    OMcp21_221 = OMcp21_220+ROcp21_820*qd[21];
    OMcp21_321 = OMcp21_320+ROcp21_920*qd[21];

// = = Block_1_0_0_22_0_6 = = 
 
// Sensor Kinematics 


    ROcp21_126 = ROcp21_121*C26-ROcp21_720*S26;
    ROcp21_226 = ROcp21_221*C26-ROcp21_820*S26;
    ROcp21_326 = ROcp21_321*C26-ROcp21_920*S26;
    ROcp21_726 = ROcp21_121*S26+ROcp21_720*C26;
    ROcp21_826 = ROcp21_221*S26+ROcp21_820*C26;
    ROcp21_926 = ROcp21_321*S26+ROcp21_920*C26;
    RLcp21_126 = ROcp21_121*s->dpt[1][41]+ROcp21_421*s->dpt[2][41]+ROcp21_720*s->dpt[3][41];
    RLcp21_226 = ROcp21_221*s->dpt[1][41]+ROcp21_521*s->dpt[2][41]+ROcp21_820*s->dpt[3][41];
    RLcp21_326 = ROcp21_321*s->dpt[1][41]+ROcp21_621*s->dpt[2][41]+ROcp21_920*s->dpt[3][41];
    OMcp21_126 = OMcp21_121+ROcp21_421*qd[26];
    OMcp21_226 = OMcp21_221+ROcp21_521*qd[26];
    OMcp21_326 = OMcp21_321+ROcp21_621*qd[26];
    RLcp21_127 = ROcp21_421*s->dpt[2][52];
    RLcp21_227 = ROcp21_521*s->dpt[2][52];
    RLcp21_327 = ROcp21_621*s->dpt[2][52];
    OMcp21_127 = OMcp21_126+ROcp21_126*qd[27];
    OMcp21_227 = OMcp21_226+ROcp21_226*qd[27];
    OMcp21_327 = OMcp21_326+ROcp21_326*qd[27];
    RLcp21_151 = ROcp21_126*s->dpt[1][53]+s->dpt[2][53]*(ROcp21_421*C27+ROcp21_726*S27)-s->dpt[3][53]*(ROcp21_421*S27-
 ROcp21_726*C27);
    RLcp21_251 = ROcp21_226*s->dpt[1][53]+s->dpt[2][53]*(ROcp21_521*C27+ROcp21_826*S27)-s->dpt[3][53]*(ROcp21_521*S27-
 ROcp21_826*C27);
    RLcp21_351 = ROcp21_326*s->dpt[1][53]+s->dpt[2][53]*(ROcp21_621*C27+ROcp21_926*S27)-s->dpt[3][53]*(ROcp21_621*S27-
 ROcp21_926*C27);
    POcp21_151 = RLcp21_119+RLcp21_121+RLcp21_126+RLcp21_127+RLcp21_151+q[1];
    POcp21_251 = RLcp21_219+RLcp21_221+RLcp21_226+RLcp21_227+RLcp21_251+q[2];
    POcp21_351 = RLcp21_319+RLcp21_321+RLcp21_326+RLcp21_327+RLcp21_351+q[3];
    VIcp21_151 = qd[1]+OMcp21_220*RLcp21_321+OMcp21_221*RLcp21_326+OMcp21_226*RLcp21_327+OMcp21_227*RLcp21_351+OMcp21_26*
 RLcp21_319-OMcp21_320*RLcp21_221-OMcp21_321*RLcp21_226-OMcp21_326*RLcp21_227-OMcp21_327*RLcp21_251-OMcp21_36*RLcp21_219;
    VIcp21_251 = qd[2]-OMcp21_120*RLcp21_321-OMcp21_121*RLcp21_326-OMcp21_126*RLcp21_327-OMcp21_127*RLcp21_351-OMcp21_16*
 RLcp21_319+OMcp21_320*RLcp21_121+OMcp21_321*RLcp21_126+OMcp21_326*RLcp21_127+OMcp21_327*RLcp21_151+OMcp21_36*RLcp21_119;
    VIcp21_351 = qd[3]+OMcp21_120*RLcp21_221+OMcp21_121*RLcp21_226+OMcp21_126*RLcp21_227+OMcp21_127*RLcp21_251+OMcp21_16*
 RLcp21_219-OMcp21_220*RLcp21_121-OMcp21_221*RLcp21_126-OMcp21_226*RLcp21_127-OMcp21_227*RLcp21_151-OMcp21_26*RLcp21_119;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_151;
    sens->P[2] = POcp21_251;
    sens->P[3] = POcp21_351;
    sens->V[1] = VIcp21_151;
    sens->V[2] = VIcp21_251;
    sens->V[3] = VIcp21_351;
    sens->OM[1] = OMcp21_127;
    sens->OM[2] = OMcp21_227;
    sens->OM[3] = OMcp21_327;
 
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


    ROcp22_419 = ROcp22_46*C19+S19*S5;
    ROcp22_519 = ROcp22_56*C19+ROcp22_85*S19;
    ROcp22_619 = ROcp22_66*C19+ROcp22_95*S19;
    ROcp22_719 = -(ROcp22_46*S19-C19*S5);
    ROcp22_819 = -(ROcp22_56*S19-ROcp22_85*C19);
    ROcp22_919 = -(ROcp22_66*S19-ROcp22_95*C19);
    ROcp22_120 = ROcp22_16*C20-ROcp22_719*S20;
    ROcp22_220 = ROcp22_26*C20-ROcp22_819*S20;
    ROcp22_320 = ROcp22_36*C20-ROcp22_919*S20;
    ROcp22_720 = ROcp22_16*S20+ROcp22_719*C20;
    ROcp22_820 = ROcp22_26*S20+ROcp22_819*C20;
    ROcp22_920 = ROcp22_36*S20+ROcp22_919*C20;
    ROcp22_121 = ROcp22_120*C21+ROcp22_419*S21;
    ROcp22_221 = ROcp22_220*C21+ROcp22_519*S21;
    ROcp22_321 = ROcp22_320*C21+ROcp22_619*S21;
    ROcp22_421 = -(ROcp22_120*S21-ROcp22_419*C21);
    ROcp22_521 = -(ROcp22_220*S21-ROcp22_519*C21);
    ROcp22_621 = -(ROcp22_320*S21-ROcp22_619*C21);
    RLcp22_119 = ROcp22_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp22_219 = ROcp22_26*s->dpt[1][4]+ROcp22_85*s->dpt[3][4];
    RLcp22_319 = ROcp22_36*s->dpt[1][4]+ROcp22_95*s->dpt[3][4];
    OMcp22_120 = OMcp22_16+ROcp22_16*qd[19]+ROcp22_419*qd[20];
    OMcp22_220 = OMcp22_26+ROcp22_26*qd[19]+ROcp22_519*qd[20];
    OMcp22_320 = OMcp22_36+ROcp22_36*qd[19]+ROcp22_619*qd[20];
    RLcp22_121 = ROcp22_720*s->dpt[3][36];
    RLcp22_221 = ROcp22_820*s->dpt[3][36];
    RLcp22_321 = ROcp22_920*s->dpt[3][36];
    OMcp22_121 = OMcp22_120+ROcp22_720*qd[21];
    OMcp22_221 = OMcp22_220+ROcp22_820*qd[21];
    OMcp22_321 = OMcp22_320+ROcp22_920*qd[21];

// = = Block_1_0_0_23_0_6 = = 
 
// Sensor Kinematics 


    ROcp22_126 = ROcp22_121*C26-ROcp22_720*S26;
    ROcp22_226 = ROcp22_221*C26-ROcp22_820*S26;
    ROcp22_326 = ROcp22_321*C26-ROcp22_920*S26;
    ROcp22_726 = ROcp22_121*S26+ROcp22_720*C26;
    ROcp22_826 = ROcp22_221*S26+ROcp22_820*C26;
    ROcp22_926 = ROcp22_321*S26+ROcp22_920*C26;
    ROcp22_427 = ROcp22_421*C27+ROcp22_726*S27;
    ROcp22_527 = ROcp22_521*C27+ROcp22_826*S27;
    ROcp22_627 = ROcp22_621*C27+ROcp22_926*S27;
    ROcp22_727 = -(ROcp22_421*S27-ROcp22_726*C27);
    ROcp22_827 = -(ROcp22_521*S27-ROcp22_826*C27);
    ROcp22_927 = -(ROcp22_621*S27-ROcp22_926*C27);
    RLcp22_126 = ROcp22_121*s->dpt[1][41]+ROcp22_421*s->dpt[2][41]+ROcp22_720*s->dpt[3][41];
    RLcp22_226 = ROcp22_221*s->dpt[1][41]+ROcp22_521*s->dpt[2][41]+ROcp22_820*s->dpt[3][41];
    RLcp22_326 = ROcp22_321*s->dpt[1][41]+ROcp22_621*s->dpt[2][41]+ROcp22_920*s->dpt[3][41];
    OMcp22_126 = OMcp22_121+ROcp22_421*qd[26];
    OMcp22_226 = OMcp22_221+ROcp22_521*qd[26];
    OMcp22_326 = OMcp22_321+ROcp22_621*qd[26];
    RLcp22_127 = ROcp22_421*s->dpt[2][52];
    RLcp22_227 = ROcp22_521*s->dpt[2][52];
    RLcp22_327 = ROcp22_621*s->dpt[2][52];
    OMcp22_127 = OMcp22_126+ROcp22_126*qd[27];
    OMcp22_227 = OMcp22_226+ROcp22_226*qd[27];
    OMcp22_327 = OMcp22_326+ROcp22_326*qd[27];
    RLcp22_128 = ROcp22_727*s->dpt[3][54];
    RLcp22_228 = ROcp22_827*s->dpt[3][54];
    RLcp22_328 = ROcp22_927*s->dpt[3][54];
    OMcp22_128 = OMcp22_127+ROcp22_727*qd[28];
    OMcp22_228 = OMcp22_227+ROcp22_827*qd[28];
    OMcp22_328 = OMcp22_327+ROcp22_927*qd[28];
    RLcp22_152 = ROcp22_727*s->dpt[3][56]+s->dpt[1][56]*(ROcp22_126*C28+ROcp22_427*S28)-s->dpt[2][56]*(ROcp22_126*S28-
 ROcp22_427*C28);
    RLcp22_252 = ROcp22_827*s->dpt[3][56]+s->dpt[1][56]*(ROcp22_226*C28+ROcp22_527*S28)-s->dpt[2][56]*(ROcp22_226*S28-
 ROcp22_527*C28);
    RLcp22_352 = ROcp22_927*s->dpt[3][56]+s->dpt[1][56]*(ROcp22_326*C28+ROcp22_627*S28)-s->dpt[2][56]*(ROcp22_326*S28-
 ROcp22_627*C28);
    POcp22_152 = RLcp22_119+RLcp22_121+RLcp22_126+RLcp22_127+RLcp22_128+RLcp22_152+q[1];
    POcp22_252 = RLcp22_219+RLcp22_221+RLcp22_226+RLcp22_227+RLcp22_228+RLcp22_252+q[2];
    POcp22_352 = RLcp22_319+RLcp22_321+RLcp22_326+RLcp22_327+RLcp22_328+RLcp22_352+q[3];
    VIcp22_152 = qd[1]+OMcp22_220*RLcp22_321+OMcp22_221*RLcp22_326+OMcp22_226*RLcp22_327+OMcp22_227*RLcp22_328+OMcp22_228*
 RLcp22_352+OMcp22_26*RLcp22_319-OMcp22_320*RLcp22_221-OMcp22_321*RLcp22_226-OMcp22_326*RLcp22_227-OMcp22_327*RLcp22_228-
 OMcp22_328*RLcp22_252-OMcp22_36*RLcp22_219;
    VIcp22_252 = qd[2]-OMcp22_120*RLcp22_321-OMcp22_121*RLcp22_326-OMcp22_126*RLcp22_327-OMcp22_127*RLcp22_328-OMcp22_128*
 RLcp22_352-OMcp22_16*RLcp22_319+OMcp22_320*RLcp22_121+OMcp22_321*RLcp22_126+OMcp22_326*RLcp22_127+OMcp22_327*RLcp22_128+
 OMcp22_328*RLcp22_152+OMcp22_36*RLcp22_119;
    VIcp22_352 = qd[3]+OMcp22_120*RLcp22_221+OMcp22_121*RLcp22_226+OMcp22_126*RLcp22_227+OMcp22_127*RLcp22_228+OMcp22_128*
 RLcp22_252+OMcp22_16*RLcp22_219-OMcp22_220*RLcp22_121-OMcp22_221*RLcp22_126-OMcp22_226*RLcp22_127-OMcp22_227*RLcp22_128-
 OMcp22_228*RLcp22_152-OMcp22_26*RLcp22_119;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_152;
    sens->P[2] = POcp22_252;
    sens->P[3] = POcp22_352;
    sens->V[1] = VIcp22_152;
    sens->V[2] = VIcp22_252;
    sens->V[3] = VIcp22_352;
    sens->OM[1] = OMcp22_128;
    sens->OM[2] = OMcp22_228;
    sens->OM[3] = OMcp22_328;
 
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


    ROcp23_419 = ROcp23_46*C19+S19*S5;
    ROcp23_519 = ROcp23_56*C19+ROcp23_85*S19;
    ROcp23_619 = ROcp23_66*C19+ROcp23_95*S19;
    ROcp23_719 = -(ROcp23_46*S19-C19*S5);
    ROcp23_819 = -(ROcp23_56*S19-ROcp23_85*C19);
    ROcp23_919 = -(ROcp23_66*S19-ROcp23_95*C19);
    ROcp23_120 = ROcp23_16*C20-ROcp23_719*S20;
    ROcp23_220 = ROcp23_26*C20-ROcp23_819*S20;
    ROcp23_320 = ROcp23_36*C20-ROcp23_919*S20;
    ROcp23_720 = ROcp23_16*S20+ROcp23_719*C20;
    ROcp23_820 = ROcp23_26*S20+ROcp23_819*C20;
    ROcp23_920 = ROcp23_36*S20+ROcp23_919*C20;
    ROcp23_121 = ROcp23_120*C21+ROcp23_419*S21;
    ROcp23_221 = ROcp23_220*C21+ROcp23_519*S21;
    ROcp23_321 = ROcp23_320*C21+ROcp23_619*S21;
    ROcp23_421 = -(ROcp23_120*S21-ROcp23_419*C21);
    ROcp23_521 = -(ROcp23_220*S21-ROcp23_519*C21);
    ROcp23_621 = -(ROcp23_320*S21-ROcp23_619*C21);
    RLcp23_119 = ROcp23_16*s->dpt[1][4]+s->dpt[3][4]*S5;
    RLcp23_219 = ROcp23_26*s->dpt[1][4]+ROcp23_85*s->dpt[3][4];
    RLcp23_319 = ROcp23_36*s->dpt[1][4]+ROcp23_95*s->dpt[3][4];
    OMcp23_120 = OMcp23_16+ROcp23_16*qd[19]+ROcp23_419*qd[20];
    OMcp23_220 = OMcp23_26+ROcp23_26*qd[19]+ROcp23_519*qd[20];
    OMcp23_320 = OMcp23_36+ROcp23_36*qd[19]+ROcp23_619*qd[20];
    RLcp23_121 = ROcp23_720*s->dpt[3][36];
    RLcp23_221 = ROcp23_820*s->dpt[3][36];
    RLcp23_321 = ROcp23_920*s->dpt[3][36];
    OMcp23_121 = OMcp23_120+ROcp23_720*qd[21];
    OMcp23_221 = OMcp23_220+ROcp23_820*qd[21];
    OMcp23_321 = OMcp23_320+ROcp23_920*qd[21];

// = = Block_1_0_0_24_0_6 = = 
 
// Sensor Kinematics 


    ROcp23_126 = ROcp23_121*C26-ROcp23_720*S26;
    ROcp23_226 = ROcp23_221*C26-ROcp23_820*S26;
    ROcp23_326 = ROcp23_321*C26-ROcp23_920*S26;
    ROcp23_726 = ROcp23_121*S26+ROcp23_720*C26;
    ROcp23_826 = ROcp23_221*S26+ROcp23_820*C26;
    ROcp23_926 = ROcp23_321*S26+ROcp23_920*C26;
    ROcp23_427 = ROcp23_421*C27+ROcp23_726*S27;
    ROcp23_527 = ROcp23_521*C27+ROcp23_826*S27;
    ROcp23_627 = ROcp23_621*C27+ROcp23_926*S27;
    ROcp23_727 = -(ROcp23_421*S27-ROcp23_726*C27);
    ROcp23_827 = -(ROcp23_521*S27-ROcp23_826*C27);
    ROcp23_927 = -(ROcp23_621*S27-ROcp23_926*C27);
    ROcp23_128 = ROcp23_126*C28+ROcp23_427*S28;
    ROcp23_228 = ROcp23_226*C28+ROcp23_527*S28;
    ROcp23_328 = ROcp23_326*C28+ROcp23_627*S28;
    ROcp23_428 = -(ROcp23_126*S28-ROcp23_427*C28);
    ROcp23_528 = -(ROcp23_226*S28-ROcp23_527*C28);
    ROcp23_628 = -(ROcp23_326*S28-ROcp23_627*C28);
    RLcp23_126 = ROcp23_121*s->dpt[1][41]+ROcp23_421*s->dpt[2][41]+ROcp23_720*s->dpt[3][41];
    RLcp23_226 = ROcp23_221*s->dpt[1][41]+ROcp23_521*s->dpt[2][41]+ROcp23_820*s->dpt[3][41];
    RLcp23_326 = ROcp23_321*s->dpt[1][41]+ROcp23_621*s->dpt[2][41]+ROcp23_920*s->dpt[3][41];
    OMcp23_126 = OMcp23_121+ROcp23_421*qd[26];
    OMcp23_226 = OMcp23_221+ROcp23_521*qd[26];
    OMcp23_326 = OMcp23_321+ROcp23_621*qd[26];
    RLcp23_127 = ROcp23_421*s->dpt[2][52];
    RLcp23_227 = ROcp23_521*s->dpt[2][52];
    RLcp23_327 = ROcp23_621*s->dpt[2][52];
    OMcp23_127 = OMcp23_126+ROcp23_126*qd[27];
    OMcp23_227 = OMcp23_226+ROcp23_226*qd[27];
    OMcp23_327 = OMcp23_326+ROcp23_326*qd[27];
    RLcp23_128 = ROcp23_727*s->dpt[3][54];
    RLcp23_228 = ROcp23_827*s->dpt[3][54];
    RLcp23_328 = ROcp23_927*s->dpt[3][54];
    OMcp23_128 = OMcp23_127+ROcp23_727*qd[28];
    OMcp23_228 = OMcp23_227+ROcp23_827*qd[28];
    OMcp23_328 = OMcp23_327+ROcp23_927*qd[28];
    RLcp23_129 = ROcp23_727*s->dpt[3][57];
    RLcp23_229 = ROcp23_827*s->dpt[3][57];
    RLcp23_329 = ROcp23_927*s->dpt[3][57];
    OMcp23_129 = OMcp23_128+ROcp23_428*qd[29];
    OMcp23_229 = OMcp23_228+ROcp23_528*qd[29];
    OMcp23_329 = OMcp23_328+ROcp23_628*qd[29];
    RLcp23_153 = ROcp23_428*s->dpt[2][59]+s->dpt[1][59]*(ROcp23_128*C29-ROcp23_727*S29)+s->dpt[3][59]*(ROcp23_128*S29+
 ROcp23_727*C29);
    RLcp23_253 = ROcp23_528*s->dpt[2][59]+s->dpt[1][59]*(ROcp23_228*C29-ROcp23_827*S29)+s->dpt[3][59]*(ROcp23_228*S29+
 ROcp23_827*C29);
    RLcp23_353 = ROcp23_628*s->dpt[2][59]+s->dpt[1][59]*(ROcp23_328*C29-ROcp23_927*S29)+s->dpt[3][59]*(ROcp23_328*S29+
 ROcp23_927*C29);
    POcp23_153 = RLcp23_119+RLcp23_121+RLcp23_126+RLcp23_127+RLcp23_128+RLcp23_129+RLcp23_153+q[1];
    POcp23_253 = RLcp23_219+RLcp23_221+RLcp23_226+RLcp23_227+RLcp23_228+RLcp23_229+RLcp23_253+q[2];
    POcp23_353 = RLcp23_319+RLcp23_321+RLcp23_326+RLcp23_327+RLcp23_328+RLcp23_329+RLcp23_353+q[3];
    VIcp23_153 = qd[1]+OMcp23_220*RLcp23_321+OMcp23_221*RLcp23_326+OMcp23_226*RLcp23_327+OMcp23_227*RLcp23_328+OMcp23_228*
 RLcp23_329+OMcp23_229*RLcp23_353+OMcp23_26*RLcp23_319-OMcp23_320*RLcp23_221-OMcp23_321*RLcp23_226-OMcp23_326*RLcp23_227-
 OMcp23_327*RLcp23_228-OMcp23_328*RLcp23_229-OMcp23_329*RLcp23_253-OMcp23_36*RLcp23_219;
    VIcp23_253 = qd[2]-OMcp23_120*RLcp23_321-OMcp23_121*RLcp23_326-OMcp23_126*RLcp23_327-OMcp23_127*RLcp23_328-OMcp23_128*
 RLcp23_329-OMcp23_129*RLcp23_353-OMcp23_16*RLcp23_319+OMcp23_320*RLcp23_121+OMcp23_321*RLcp23_126+OMcp23_326*RLcp23_127+
 OMcp23_327*RLcp23_128+OMcp23_328*RLcp23_129+OMcp23_329*RLcp23_153+OMcp23_36*RLcp23_119;
    VIcp23_353 = qd[3]+OMcp23_120*RLcp23_221+OMcp23_121*RLcp23_226+OMcp23_126*RLcp23_227+OMcp23_127*RLcp23_228+OMcp23_128*
 RLcp23_229+OMcp23_129*RLcp23_253+OMcp23_16*RLcp23_219-OMcp23_220*RLcp23_121-OMcp23_221*RLcp23_126-OMcp23_226*RLcp23_127-
 OMcp23_227*RLcp23_128-OMcp23_228*RLcp23_129-OMcp23_229*RLcp23_153-OMcp23_26*RLcp23_119;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_153;
    sens->P[2] = POcp23_253;
    sens->P[3] = POcp23_353;
    sens->V[1] = VIcp23_153;
    sens->V[2] = VIcp23_253;
    sens->V[3] = VIcp23_353;
    sens->OM[1] = OMcp23_129;
    sens->OM[2] = OMcp23_229;
    sens->OM[3] = OMcp23_329;
 
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
    RLcp27_157 = ROcp27_712*s->dpt[3][19];
    RLcp27_257 = ROcp27_812*s->dpt[3][19];
    RLcp27_357 = ROcp27_912*s->dpt[3][19];
    POcp27_157 = RLcp27_110+RLcp27_111+RLcp27_157+RLcp27_17+RLcp27_18+RLcp27_19+q[1];
    POcp27_257 = RLcp27_210+RLcp27_211+RLcp27_257+RLcp27_27+RLcp27_28+RLcp27_29+q[2];
    POcp27_357 = RLcp27_310+RLcp27_311+RLcp27_357+RLcp27_37+RLcp27_38+RLcp27_39+q[3];
    VIcp27_157 = qd[1]+OMcp27_210*RLcp27_311+OMcp27_212*RLcp27_357+OMcp27_26*RLcp27_37+OMcp27_27*RLcp27_38+OMcp27_28*
 RLcp27_39+OMcp27_29*RLcp27_310-OMcp27_310*RLcp27_211-OMcp27_312*RLcp27_257-OMcp27_36*RLcp27_27-OMcp27_37*RLcp27_28-OMcp27_38
 *RLcp27_29-OMcp27_39*RLcp27_210;
    VIcp27_257 = qd[2]-OMcp27_110*RLcp27_311-OMcp27_112*RLcp27_357-OMcp27_16*RLcp27_37-OMcp27_17*RLcp27_38-OMcp27_18*
 RLcp27_39-OMcp27_19*RLcp27_310+OMcp27_310*RLcp27_111+OMcp27_312*RLcp27_157+OMcp27_36*RLcp27_17+OMcp27_37*RLcp27_18+OMcp27_38
 *RLcp27_19+OMcp27_39*RLcp27_110;
    VIcp27_357 = qd[3]+OMcp27_110*RLcp27_211+OMcp27_112*RLcp27_257+OMcp27_16*RLcp27_27+OMcp27_17*RLcp27_28+OMcp27_18*
 RLcp27_29+OMcp27_19*RLcp27_210-OMcp27_210*RLcp27_111-OMcp27_212*RLcp27_157-OMcp27_26*RLcp27_17-OMcp27_27*RLcp27_18-OMcp27_28
 *RLcp27_19-OMcp27_29*RLcp27_110;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_157;
    sens->P[2] = POcp27_257;
    sens->P[3] = POcp27_357;
    sens->R[1][1] = ROcp27_112;
    sens->R[1][2] = ROcp27_212;
    sens->R[1][3] = ROcp27_312;
    sens->R[2][1] = ROcp27_411;
    sens->R[2][2] = ROcp27_511;
    sens->R[2][3] = ROcp27_611;
    sens->R[3][1] = ROcp27_712;
    sens->R[3][2] = ROcp27_812;
    sens->R[3][3] = ROcp27_912;
    sens->V[1] = VIcp27_157;
    sens->V[2] = VIcp27_257;
    sens->V[3] = VIcp27_357;
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
    OMcp28_16 = qd[4]+qd[6]*S5;
    OMcp28_26 = ROcp28_85*qd[6]+qd[5]*C4;
    OMcp28_36 = ROcp28_95*qd[6]+qd[5]*S4;

// = = Block_1_0_0_29_0_3 = = 
 
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
    OMcp28_113 = OMcp28_16+ROcp28_46*qd[13];
    OMcp28_213 = OMcp28_26+ROcp28_56*qd[13];
    OMcp28_313 = OMcp28_36+ROcp28_66*qd[13];
    RLcp28_114 = ROcp28_46*s->dpt[2][20];
    RLcp28_214 = ROcp28_56*s->dpt[2][20];
    RLcp28_314 = ROcp28_66*s->dpt[2][20];
    OMcp28_114 = OMcp28_113+ROcp28_113*qd[14];
    OMcp28_214 = OMcp28_213+ROcp28_213*qd[14];
    OMcp28_314 = OMcp28_313+ROcp28_313*qd[14];
    RLcp28_115 = ROcp28_714*s->dpt[3][22];
    RLcp28_215 = ROcp28_814*s->dpt[3][22];
    RLcp28_315 = ROcp28_914*s->dpt[3][22];
    POcp28_115 = RLcp28_113+RLcp28_114+RLcp28_115+q[1];
    POcp28_215 = RLcp28_213+RLcp28_214+RLcp28_215+q[2];
    POcp28_315 = RLcp28_313+RLcp28_314+RLcp28_315+q[3];
    OMcp28_115 = OMcp28_114+ROcp28_714*qd[15];
    OMcp28_215 = OMcp28_214+ROcp28_814*qd[15];
    OMcp28_315 = OMcp28_314+ROcp28_914*qd[15];
    VIcp28_115 = qd[1]+OMcp28_213*RLcp28_314+OMcp28_214*RLcp28_315+OMcp28_26*RLcp28_313-OMcp28_313*RLcp28_214-OMcp28_314*
 RLcp28_215-OMcp28_36*RLcp28_213;
    VIcp28_215 = qd[2]-OMcp28_113*RLcp28_314-OMcp28_114*RLcp28_315-OMcp28_16*RLcp28_313+OMcp28_313*RLcp28_114+OMcp28_314*
 RLcp28_115+OMcp28_36*RLcp28_113;
    VIcp28_315 = qd[3]+OMcp28_113*RLcp28_214+OMcp28_114*RLcp28_215+OMcp28_16*RLcp28_213-OMcp28_213*RLcp28_114-OMcp28_214*
 RLcp28_115-OMcp28_26*RLcp28_113;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_115;
    sens->P[2] = POcp28_215;
    sens->P[3] = POcp28_315;
    sens->R[1][1] = ROcp28_115;
    sens->R[1][2] = ROcp28_215;
    sens->R[1][3] = ROcp28_315;
    sens->R[2][1] = ROcp28_415;
    sens->R[2][2] = ROcp28_515;
    sens->R[2][3] = ROcp28_615;
    sens->R[3][1] = ROcp28_714;
    sens->R[3][2] = ROcp28_814;
    sens->R[3][3] = ROcp28_914;
    sens->V[1] = VIcp28_115;
    sens->V[2] = VIcp28_215;
    sens->V[3] = VIcp28_315;
    sens->OM[1] = OMcp28_115;
    sens->OM[2] = OMcp28_215;
    sens->OM[3] = OMcp28_315;
 
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
    OMcp29_113 = OMcp29_16+ROcp29_46*qd[13];
    OMcp29_213 = OMcp29_26+ROcp29_56*qd[13];
    OMcp29_313 = OMcp29_36+ROcp29_66*qd[13];
    RLcp29_114 = ROcp29_46*s->dpt[2][20];
    RLcp29_214 = ROcp29_56*s->dpt[2][20];
    RLcp29_314 = ROcp29_66*s->dpt[2][20];
    OMcp29_114 = OMcp29_113+ROcp29_113*qd[14];
    OMcp29_214 = OMcp29_213+ROcp29_213*qd[14];
    OMcp29_314 = OMcp29_313+ROcp29_313*qd[14];
    RLcp29_115 = ROcp29_714*s->dpt[3][22];
    RLcp29_215 = ROcp29_814*s->dpt[3][22];
    RLcp29_315 = ROcp29_914*s->dpt[3][22];
    OMcp29_115 = OMcp29_114+ROcp29_714*qd[15];
    OMcp29_215 = OMcp29_214+ROcp29_814*qd[15];
    OMcp29_315 = OMcp29_314+ROcp29_914*qd[15];
    RLcp29_116 = ROcp29_714*s->dpt[3][25];
    RLcp29_216 = ROcp29_814*s->dpt[3][25];
    RLcp29_316 = ROcp29_914*s->dpt[3][25];
    POcp29_116 = RLcp29_113+RLcp29_114+RLcp29_115+RLcp29_116+q[1];
    POcp29_216 = RLcp29_213+RLcp29_214+RLcp29_215+RLcp29_216+q[2];
    POcp29_316 = RLcp29_313+RLcp29_314+RLcp29_315+RLcp29_316+q[3];
    OMcp29_116 = OMcp29_115+ROcp29_415*qd[16];
    OMcp29_216 = OMcp29_215+ROcp29_515*qd[16];
    OMcp29_316 = OMcp29_315+ROcp29_615*qd[16];
    VIcp29_116 = qd[1]+OMcp29_213*RLcp29_314+OMcp29_214*RLcp29_315+OMcp29_215*RLcp29_316+OMcp29_26*RLcp29_313-OMcp29_313*
 RLcp29_214-OMcp29_314*RLcp29_215-OMcp29_315*RLcp29_216-OMcp29_36*RLcp29_213;
    VIcp29_216 = qd[2]-OMcp29_113*RLcp29_314-OMcp29_114*RLcp29_315-OMcp29_115*RLcp29_316-OMcp29_16*RLcp29_313+OMcp29_313*
 RLcp29_114+OMcp29_314*RLcp29_115+OMcp29_315*RLcp29_116+OMcp29_36*RLcp29_113;
    VIcp29_316 = qd[3]+OMcp29_113*RLcp29_214+OMcp29_114*RLcp29_215+OMcp29_115*RLcp29_216+OMcp29_16*RLcp29_213-OMcp29_213*
 RLcp29_114-OMcp29_214*RLcp29_115-OMcp29_215*RLcp29_116-OMcp29_26*RLcp29_113;

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_116;
    sens->P[2] = POcp29_216;
    sens->P[3] = POcp29_316;
    sens->R[1][1] = ROcp29_116;
    sens->R[1][2] = ROcp29_216;
    sens->R[1][3] = ROcp29_316;
    sens->R[2][1] = ROcp29_415;
    sens->R[2][2] = ROcp29_515;
    sens->R[2][3] = ROcp29_615;
    sens->R[3][1] = ROcp29_716;
    sens->R[3][2] = ROcp29_816;
    sens->R[3][3] = ROcp29_916;
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
    OMcp30_113 = OMcp30_16+ROcp30_46*qd[13];
    OMcp30_213 = OMcp30_26+ROcp30_56*qd[13];
    OMcp30_313 = OMcp30_36+ROcp30_66*qd[13];
    RLcp30_114 = ROcp30_46*s->dpt[2][20];
    RLcp30_214 = ROcp30_56*s->dpt[2][20];
    RLcp30_314 = ROcp30_66*s->dpt[2][20];
    OMcp30_114 = OMcp30_113+ROcp30_113*qd[14];
    OMcp30_214 = OMcp30_213+ROcp30_213*qd[14];
    OMcp30_314 = OMcp30_313+ROcp30_313*qd[14];
    RLcp30_115 = ROcp30_714*s->dpt[3][22];
    RLcp30_215 = ROcp30_814*s->dpt[3][22];
    RLcp30_315 = ROcp30_914*s->dpt[3][22];
    OMcp30_115 = OMcp30_114+ROcp30_714*qd[15];
    OMcp30_215 = OMcp30_214+ROcp30_814*qd[15];
    OMcp30_315 = OMcp30_314+ROcp30_914*qd[15];
    RLcp30_116 = ROcp30_714*s->dpt[3][25];
    RLcp30_216 = ROcp30_814*s->dpt[3][25];
    RLcp30_316 = ROcp30_914*s->dpt[3][25];
    OMcp30_116 = OMcp30_115+ROcp30_415*qd[16];
    OMcp30_216 = OMcp30_215+ROcp30_515*qd[16];
    OMcp30_316 = OMcp30_315+ROcp30_615*qd[16];
    RLcp30_117 = ROcp30_716*s->dpt[3][28];
    RLcp30_217 = ROcp30_816*s->dpt[3][28];
    RLcp30_317 = ROcp30_916*s->dpt[3][28];
    OMcp30_118 = OMcp30_116+ROcp30_116*qd[17]+ROcp30_417*qd[18];
    OMcp30_218 = OMcp30_216+ROcp30_216*qd[17]+ROcp30_517*qd[18];
    OMcp30_318 = OMcp30_316+ROcp30_316*qd[17]+ROcp30_617*qd[18];
    RLcp30_160 = ROcp30_718*s->dpt[3][33];
    RLcp30_260 = ROcp30_818*s->dpt[3][33];
    RLcp30_360 = ROcp30_918*s->dpt[3][33];
    POcp30_160 = RLcp30_113+RLcp30_114+RLcp30_115+RLcp30_116+RLcp30_117+RLcp30_160+q[1];
    POcp30_260 = RLcp30_213+RLcp30_214+RLcp30_215+RLcp30_216+RLcp30_217+RLcp30_260+q[2];
    POcp30_360 = RLcp30_313+RLcp30_314+RLcp30_315+RLcp30_316+RLcp30_317+RLcp30_360+q[3];
    VIcp30_160 = qd[1]+OMcp30_213*RLcp30_314+OMcp30_214*RLcp30_315+OMcp30_215*RLcp30_316+OMcp30_216*RLcp30_317+OMcp30_218*
 RLcp30_360+OMcp30_26*RLcp30_313-OMcp30_313*RLcp30_214-OMcp30_314*RLcp30_215-OMcp30_315*RLcp30_216-OMcp30_316*RLcp30_217-
 OMcp30_318*RLcp30_260-OMcp30_36*RLcp30_213;
    VIcp30_260 = qd[2]-OMcp30_113*RLcp30_314-OMcp30_114*RLcp30_315-OMcp30_115*RLcp30_316-OMcp30_116*RLcp30_317-OMcp30_118*
 RLcp30_360-OMcp30_16*RLcp30_313+OMcp30_313*RLcp30_114+OMcp30_314*RLcp30_115+OMcp30_315*RLcp30_116+OMcp30_316*RLcp30_117+
 OMcp30_318*RLcp30_160+OMcp30_36*RLcp30_113;
    VIcp30_360 = qd[3]+OMcp30_113*RLcp30_214+OMcp30_114*RLcp30_215+OMcp30_115*RLcp30_216+OMcp30_116*RLcp30_217+OMcp30_118*
 RLcp30_260+OMcp30_16*RLcp30_213-OMcp30_213*RLcp30_114-OMcp30_214*RLcp30_115-OMcp30_215*RLcp30_116-OMcp30_216*RLcp30_117-
 OMcp30_218*RLcp30_160-OMcp30_26*RLcp30_113;

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_160;
    sens->P[2] = POcp30_260;
    sens->P[3] = POcp30_360;
    sens->R[1][1] = ROcp30_118;
    sens->R[1][2] = ROcp30_218;
    sens->R[1][3] = ROcp30_318;
    sens->R[2][1] = ROcp30_417;
    sens->R[2][2] = ROcp30_517;
    sens->R[2][3] = ROcp30_617;
    sens->R[3][1] = ROcp30_718;
    sens->R[3][2] = ROcp30_818;
    sens->R[3][3] = ROcp30_918;
    sens->V[1] = VIcp30_160;
    sens->V[2] = VIcp30_260;
    sens->V[3] = VIcp30_360;
    sens->OM[1] = OMcp30_118;
    sens->OM[2] = OMcp30_218;
    sens->OM[3] = OMcp30_318;
 
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

// = = Block_1_0_0_32_0_4 = = 
 
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
    OMcp31_120 = OMcp31_16+ROcp31_16*qd[19]+ROcp31_419*qd[20];
    OMcp31_220 = OMcp31_26+ROcp31_26*qd[19]+ROcp31_519*qd[20];
    OMcp31_320 = OMcp31_36+ROcp31_36*qd[19]+ROcp31_619*qd[20];
    RLcp31_121 = ROcp31_720*s->dpt[3][36];
    RLcp31_221 = ROcp31_820*s->dpt[3][36];
    RLcp31_321 = ROcp31_920*s->dpt[3][36];
    POcp31_121 = RLcp31_119+RLcp31_121+q[1];
    POcp31_221 = RLcp31_219+RLcp31_221+q[2];
    POcp31_321 = RLcp31_319+RLcp31_321+q[3];
    OMcp31_121 = OMcp31_120+ROcp31_720*qd[21];
    OMcp31_221 = OMcp31_220+ROcp31_820*qd[21];
    OMcp31_321 = OMcp31_320+ROcp31_920*qd[21];
    VIcp31_121 = qd[1]+OMcp31_220*RLcp31_321+OMcp31_26*RLcp31_319-OMcp31_320*RLcp31_221-OMcp31_36*RLcp31_219;
    VIcp31_221 = qd[2]-OMcp31_120*RLcp31_321-OMcp31_16*RLcp31_319+OMcp31_320*RLcp31_121+OMcp31_36*RLcp31_119;
    VIcp31_321 = qd[3]+OMcp31_120*RLcp31_221+OMcp31_16*RLcp31_219-OMcp31_220*RLcp31_121-OMcp31_26*RLcp31_119;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp31_121;
    sens->P[2] = POcp31_221;
    sens->P[3] = POcp31_321;
    sens->R[1][1] = ROcp31_121;
    sens->R[1][2] = ROcp31_221;
    sens->R[1][3] = ROcp31_321;
    sens->R[2][1] = ROcp31_421;
    sens->R[2][2] = ROcp31_521;
    sens->R[2][3] = ROcp31_621;
    sens->R[3][1] = ROcp31_720;
    sens->R[3][2] = ROcp31_820;
    sens->R[3][3] = ROcp31_920;
    sens->V[1] = VIcp31_121;
    sens->V[2] = VIcp31_221;
    sens->V[3] = VIcp31_321;
    sens->OM[1] = OMcp31_121;
    sens->OM[2] = OMcp31_221;
    sens->OM[3] = OMcp31_321;
 
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

// = = Block_1_0_0_33_0_4 = = 
 
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
    OMcp32_120 = OMcp32_16+ROcp32_16*qd[19]+ROcp32_419*qd[20];
    OMcp32_220 = OMcp32_26+ROcp32_26*qd[19]+ROcp32_519*qd[20];
    OMcp32_320 = OMcp32_36+ROcp32_36*qd[19]+ROcp32_619*qd[20];
    RLcp32_121 = ROcp32_720*s->dpt[3][36];
    RLcp32_221 = ROcp32_820*s->dpt[3][36];
    RLcp32_321 = ROcp32_920*s->dpt[3][36];
    OMcp32_121 = OMcp32_120+ROcp32_720*qd[21];
    OMcp32_221 = OMcp32_220+ROcp32_820*qd[21];
    OMcp32_321 = OMcp32_320+ROcp32_920*qd[21];

// = = Block_1_0_0_33_0_5 = = 
 
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
    OMcp32_122 = OMcp32_121+ROcp32_421*qd[22];
    OMcp32_222 = OMcp32_221+ROcp32_521*qd[22];
    OMcp32_322 = OMcp32_321+ROcp32_621*qd[22];
    RLcp32_123 = ROcp32_421*s->dpt[2][43];
    RLcp32_223 = ROcp32_521*s->dpt[2][43];
    RLcp32_323 = ROcp32_621*s->dpt[2][43];
    OMcp32_123 = OMcp32_122+ROcp32_122*qd[23];
    OMcp32_223 = OMcp32_222+ROcp32_222*qd[23];
    OMcp32_323 = OMcp32_322+ROcp32_322*qd[23];
    RLcp32_124 = ROcp32_723*s->dpt[3][45];
    RLcp32_224 = ROcp32_823*s->dpt[3][45];
    RLcp32_324 = ROcp32_923*s->dpt[3][45];
    POcp32_124 = RLcp32_119+RLcp32_121+RLcp32_122+RLcp32_123+RLcp32_124+q[1];
    POcp32_224 = RLcp32_219+RLcp32_221+RLcp32_222+RLcp32_223+RLcp32_224+q[2];
    POcp32_324 = RLcp32_319+RLcp32_321+RLcp32_322+RLcp32_323+RLcp32_324+q[3];
    OMcp32_124 = OMcp32_123+ROcp32_723*qd[24];
    OMcp32_224 = OMcp32_223+ROcp32_823*qd[24];
    OMcp32_324 = OMcp32_323+ROcp32_923*qd[24];
    VIcp32_124 = qd[1]+OMcp32_220*RLcp32_321+OMcp32_221*RLcp32_322+OMcp32_222*RLcp32_323+OMcp32_223*RLcp32_324+OMcp32_26*
 RLcp32_319-OMcp32_320*RLcp32_221-OMcp32_321*RLcp32_222-OMcp32_322*RLcp32_223-OMcp32_323*RLcp32_224-OMcp32_36*RLcp32_219;
    VIcp32_224 = qd[2]-OMcp32_120*RLcp32_321-OMcp32_121*RLcp32_322-OMcp32_122*RLcp32_323-OMcp32_123*RLcp32_324-OMcp32_16*
 RLcp32_319+OMcp32_320*RLcp32_121+OMcp32_321*RLcp32_122+OMcp32_322*RLcp32_123+OMcp32_323*RLcp32_124+OMcp32_36*RLcp32_119;
    VIcp32_324 = qd[3]+OMcp32_120*RLcp32_221+OMcp32_121*RLcp32_222+OMcp32_122*RLcp32_223+OMcp32_123*RLcp32_224+OMcp32_16*
 RLcp32_219-OMcp32_220*RLcp32_121-OMcp32_221*RLcp32_122-OMcp32_222*RLcp32_123-OMcp32_223*RLcp32_124-OMcp32_26*RLcp32_119;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp32_124;
    sens->P[2] = POcp32_224;
    sens->P[3] = POcp32_324;
    sens->R[1][1] = ROcp32_124;
    sens->R[1][2] = ROcp32_224;
    sens->R[1][3] = ROcp32_324;
    sens->R[2][1] = ROcp32_424;
    sens->R[2][2] = ROcp32_524;
    sens->R[2][3] = ROcp32_624;
    sens->R[3][1] = ROcp32_723;
    sens->R[3][2] = ROcp32_823;
    sens->R[3][3] = ROcp32_923;
    sens->V[1] = VIcp32_124;
    sens->V[2] = VIcp32_224;
    sens->V[3] = VIcp32_324;
    sens->OM[1] = OMcp32_124;
    sens->OM[2] = OMcp32_224;
    sens->OM[3] = OMcp32_324;
 
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
    OMcp33_120 = OMcp33_16+ROcp33_16*qd[19]+ROcp33_419*qd[20];
    OMcp33_220 = OMcp33_26+ROcp33_26*qd[19]+ROcp33_519*qd[20];
    OMcp33_320 = OMcp33_36+ROcp33_36*qd[19]+ROcp33_619*qd[20];
    RLcp33_121 = ROcp33_720*s->dpt[3][36];
    RLcp33_221 = ROcp33_820*s->dpt[3][36];
    RLcp33_321 = ROcp33_920*s->dpt[3][36];
    OMcp33_121 = OMcp33_120+ROcp33_720*qd[21];
    OMcp33_221 = OMcp33_220+ROcp33_820*qd[21];
    OMcp33_321 = OMcp33_320+ROcp33_920*qd[21];

// = = Block_1_0_0_34_0_5 = = 
 
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
    OMcp33_122 = OMcp33_121+ROcp33_421*qd[22];
    OMcp33_222 = OMcp33_221+ROcp33_521*qd[22];
    OMcp33_322 = OMcp33_321+ROcp33_621*qd[22];
    RLcp33_123 = ROcp33_421*s->dpt[2][43];
    RLcp33_223 = ROcp33_521*s->dpt[2][43];
    RLcp33_323 = ROcp33_621*s->dpt[2][43];
    OMcp33_123 = OMcp33_122+ROcp33_122*qd[23];
    OMcp33_223 = OMcp33_222+ROcp33_222*qd[23];
    OMcp33_323 = OMcp33_322+ROcp33_322*qd[23];
    RLcp33_124 = ROcp33_723*s->dpt[3][45];
    RLcp33_224 = ROcp33_823*s->dpt[3][45];
    RLcp33_324 = ROcp33_923*s->dpt[3][45];
    OMcp33_124 = OMcp33_123+ROcp33_723*qd[24];
    OMcp33_224 = OMcp33_223+ROcp33_823*qd[24];
    OMcp33_324 = OMcp33_323+ROcp33_923*qd[24];
    RLcp33_125 = ROcp33_723*s->dpt[3][48];
    RLcp33_225 = ROcp33_823*s->dpt[3][48];
    RLcp33_325 = ROcp33_923*s->dpt[3][48];
    POcp33_125 = RLcp33_119+RLcp33_121+RLcp33_122+RLcp33_123+RLcp33_124+RLcp33_125+q[1];
    POcp33_225 = RLcp33_219+RLcp33_221+RLcp33_222+RLcp33_223+RLcp33_224+RLcp33_225+q[2];
    POcp33_325 = RLcp33_319+RLcp33_321+RLcp33_322+RLcp33_323+RLcp33_324+RLcp33_325+q[3];
    OMcp33_125 = OMcp33_124+ROcp33_424*qd[25];
    OMcp33_225 = OMcp33_224+ROcp33_524*qd[25];
    OMcp33_325 = OMcp33_324+ROcp33_624*qd[25];
    VIcp33_125 = qd[1]+OMcp33_220*RLcp33_321+OMcp33_221*RLcp33_322+OMcp33_222*RLcp33_323+OMcp33_223*RLcp33_324+OMcp33_224*
 RLcp33_325+OMcp33_26*RLcp33_319-OMcp33_320*RLcp33_221-OMcp33_321*RLcp33_222-OMcp33_322*RLcp33_223-OMcp33_323*RLcp33_224-
 OMcp33_324*RLcp33_225-OMcp33_36*RLcp33_219;
    VIcp33_225 = qd[2]-OMcp33_120*RLcp33_321-OMcp33_121*RLcp33_322-OMcp33_122*RLcp33_323-OMcp33_123*RLcp33_324-OMcp33_124*
 RLcp33_325-OMcp33_16*RLcp33_319+OMcp33_320*RLcp33_121+OMcp33_321*RLcp33_122+OMcp33_322*RLcp33_123+OMcp33_323*RLcp33_124+
 OMcp33_324*RLcp33_125+OMcp33_36*RLcp33_119;
    VIcp33_325 = qd[3]+OMcp33_120*RLcp33_221+OMcp33_121*RLcp33_222+OMcp33_122*RLcp33_223+OMcp33_123*RLcp33_224+OMcp33_124*
 RLcp33_225+OMcp33_16*RLcp33_219-OMcp33_220*RLcp33_121-OMcp33_221*RLcp33_122-OMcp33_222*RLcp33_123-OMcp33_223*RLcp33_124-
 OMcp33_224*RLcp33_125-OMcp33_26*RLcp33_119;

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp33_125;
    sens->P[2] = POcp33_225;
    sens->P[3] = POcp33_325;
    sens->R[1][1] = ROcp33_125;
    sens->R[1][2] = ROcp33_225;
    sens->R[1][3] = ROcp33_325;
    sens->R[2][1] = ROcp33_424;
    sens->R[2][2] = ROcp33_524;
    sens->R[2][3] = ROcp33_624;
    sens->R[3][1] = ROcp33_725;
    sens->R[3][2] = ROcp33_825;
    sens->R[3][3] = ROcp33_925;
    sens->V[1] = VIcp33_125;
    sens->V[2] = VIcp33_225;
    sens->V[3] = VIcp33_325;
    sens->OM[1] = OMcp33_125;
    sens->OM[2] = OMcp33_225;
    sens->OM[3] = OMcp33_325;
 
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
    OMcp34_120 = OMcp34_16+ROcp34_16*qd[19]+ROcp34_419*qd[20];
    OMcp34_220 = OMcp34_26+ROcp34_26*qd[19]+ROcp34_519*qd[20];
    OMcp34_320 = OMcp34_36+ROcp34_36*qd[19]+ROcp34_619*qd[20];
    RLcp34_121 = ROcp34_720*s->dpt[3][36];
    RLcp34_221 = ROcp34_820*s->dpt[3][36];
    RLcp34_321 = ROcp34_920*s->dpt[3][36];
    OMcp34_121 = OMcp34_120+ROcp34_720*qd[21];
    OMcp34_221 = OMcp34_220+ROcp34_820*qd[21];
    OMcp34_321 = OMcp34_320+ROcp34_920*qd[21];

// = = Block_1_0_0_35_0_6 = = 
 
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
    OMcp34_126 = OMcp34_121+ROcp34_421*qd[26];
    OMcp34_226 = OMcp34_221+ROcp34_521*qd[26];
    OMcp34_326 = OMcp34_321+ROcp34_621*qd[26];
    RLcp34_127 = ROcp34_421*s->dpt[2][52];
    RLcp34_227 = ROcp34_521*s->dpt[2][52];
    RLcp34_327 = ROcp34_621*s->dpt[2][52];
    OMcp34_127 = OMcp34_126+ROcp34_126*qd[27];
    OMcp34_227 = OMcp34_226+ROcp34_226*qd[27];
    OMcp34_327 = OMcp34_326+ROcp34_326*qd[27];
    RLcp34_128 = ROcp34_727*s->dpt[3][54];
    RLcp34_228 = ROcp34_827*s->dpt[3][54];
    RLcp34_328 = ROcp34_927*s->dpt[3][54];
    POcp34_128 = RLcp34_119+RLcp34_121+RLcp34_126+RLcp34_127+RLcp34_128+q[1];
    POcp34_228 = RLcp34_219+RLcp34_221+RLcp34_226+RLcp34_227+RLcp34_228+q[2];
    POcp34_328 = RLcp34_319+RLcp34_321+RLcp34_326+RLcp34_327+RLcp34_328+q[3];
    OMcp34_128 = OMcp34_127+ROcp34_727*qd[28];
    OMcp34_228 = OMcp34_227+ROcp34_827*qd[28];
    OMcp34_328 = OMcp34_327+ROcp34_927*qd[28];
    VIcp34_128 = qd[1]+OMcp34_220*RLcp34_321+OMcp34_221*RLcp34_326+OMcp34_226*RLcp34_327+OMcp34_227*RLcp34_328+OMcp34_26*
 RLcp34_319-OMcp34_320*RLcp34_221-OMcp34_321*RLcp34_226-OMcp34_326*RLcp34_227-OMcp34_327*RLcp34_228-OMcp34_36*RLcp34_219;
    VIcp34_228 = qd[2]-OMcp34_120*RLcp34_321-OMcp34_121*RLcp34_326-OMcp34_126*RLcp34_327-OMcp34_127*RLcp34_328-OMcp34_16*
 RLcp34_319+OMcp34_320*RLcp34_121+OMcp34_321*RLcp34_126+OMcp34_326*RLcp34_127+OMcp34_327*RLcp34_128+OMcp34_36*RLcp34_119;
    VIcp34_328 = qd[3]+OMcp34_120*RLcp34_221+OMcp34_121*RLcp34_226+OMcp34_126*RLcp34_227+OMcp34_127*RLcp34_228+OMcp34_16*
 RLcp34_219-OMcp34_220*RLcp34_121-OMcp34_221*RLcp34_126-OMcp34_226*RLcp34_127-OMcp34_227*RLcp34_128-OMcp34_26*RLcp34_119;

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp34_128;
    sens->P[2] = POcp34_228;
    sens->P[3] = POcp34_328;
    sens->R[1][1] = ROcp34_128;
    sens->R[1][2] = ROcp34_228;
    sens->R[1][3] = ROcp34_328;
    sens->R[2][1] = ROcp34_428;
    sens->R[2][2] = ROcp34_528;
    sens->R[2][3] = ROcp34_628;
    sens->R[3][1] = ROcp34_727;
    sens->R[3][2] = ROcp34_827;
    sens->R[3][3] = ROcp34_927;
    sens->V[1] = VIcp34_128;
    sens->V[2] = VIcp34_228;
    sens->V[3] = VIcp34_328;
    sens->OM[1] = OMcp34_128;
    sens->OM[2] = OMcp34_228;
    sens->OM[3] = OMcp34_328;
 
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
    OMcp35_120 = OMcp35_16+ROcp35_16*qd[19]+ROcp35_419*qd[20];
    OMcp35_220 = OMcp35_26+ROcp35_26*qd[19]+ROcp35_519*qd[20];
    OMcp35_320 = OMcp35_36+ROcp35_36*qd[19]+ROcp35_619*qd[20];
    RLcp35_121 = ROcp35_720*s->dpt[3][36];
    RLcp35_221 = ROcp35_820*s->dpt[3][36];
    RLcp35_321 = ROcp35_920*s->dpt[3][36];
    OMcp35_121 = OMcp35_120+ROcp35_720*qd[21];
    OMcp35_221 = OMcp35_220+ROcp35_820*qd[21];
    OMcp35_321 = OMcp35_320+ROcp35_920*qd[21];

// = = Block_1_0_0_36_0_6 = = 
 
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
    OMcp35_126 = OMcp35_121+ROcp35_421*qd[26];
    OMcp35_226 = OMcp35_221+ROcp35_521*qd[26];
    OMcp35_326 = OMcp35_321+ROcp35_621*qd[26];
    RLcp35_127 = ROcp35_421*s->dpt[2][52];
    RLcp35_227 = ROcp35_521*s->dpt[2][52];
    RLcp35_327 = ROcp35_621*s->dpt[2][52];
    OMcp35_127 = OMcp35_126+ROcp35_126*qd[27];
    OMcp35_227 = OMcp35_226+ROcp35_226*qd[27];
    OMcp35_327 = OMcp35_326+ROcp35_326*qd[27];
    RLcp35_128 = ROcp35_727*s->dpt[3][54];
    RLcp35_228 = ROcp35_827*s->dpt[3][54];
    RLcp35_328 = ROcp35_927*s->dpt[3][54];
    OMcp35_128 = OMcp35_127+ROcp35_727*qd[28];
    OMcp35_228 = OMcp35_227+ROcp35_827*qd[28];
    OMcp35_328 = OMcp35_327+ROcp35_927*qd[28];
    RLcp35_129 = ROcp35_727*s->dpt[3][57];
    RLcp35_229 = ROcp35_827*s->dpt[3][57];
    RLcp35_329 = ROcp35_927*s->dpt[3][57];
    POcp35_129 = RLcp35_119+RLcp35_121+RLcp35_126+RLcp35_127+RLcp35_128+RLcp35_129+q[1];
    POcp35_229 = RLcp35_219+RLcp35_221+RLcp35_226+RLcp35_227+RLcp35_228+RLcp35_229+q[2];
    POcp35_329 = RLcp35_319+RLcp35_321+RLcp35_326+RLcp35_327+RLcp35_328+RLcp35_329+q[3];
    OMcp35_129 = OMcp35_128+ROcp35_428*qd[29];
    OMcp35_229 = OMcp35_228+ROcp35_528*qd[29];
    OMcp35_329 = OMcp35_328+ROcp35_628*qd[29];
    VIcp35_129 = qd[1]+OMcp35_220*RLcp35_321+OMcp35_221*RLcp35_326+OMcp35_226*RLcp35_327+OMcp35_227*RLcp35_328+OMcp35_228*
 RLcp35_329+OMcp35_26*RLcp35_319-OMcp35_320*RLcp35_221-OMcp35_321*RLcp35_226-OMcp35_326*RLcp35_227-OMcp35_327*RLcp35_228-
 OMcp35_328*RLcp35_229-OMcp35_36*RLcp35_219;
    VIcp35_229 = qd[2]-OMcp35_120*RLcp35_321-OMcp35_121*RLcp35_326-OMcp35_126*RLcp35_327-OMcp35_127*RLcp35_328-OMcp35_128*
 RLcp35_329-OMcp35_16*RLcp35_319+OMcp35_320*RLcp35_121+OMcp35_321*RLcp35_126+OMcp35_326*RLcp35_127+OMcp35_327*RLcp35_128+
 OMcp35_328*RLcp35_129+OMcp35_36*RLcp35_119;
    VIcp35_329 = qd[3]+OMcp35_120*RLcp35_221+OMcp35_121*RLcp35_226+OMcp35_126*RLcp35_227+OMcp35_127*RLcp35_228+OMcp35_128*
 RLcp35_229+OMcp35_16*RLcp35_219-OMcp35_220*RLcp35_121-OMcp35_221*RLcp35_126-OMcp35_226*RLcp35_127-OMcp35_227*RLcp35_128-
 OMcp35_228*RLcp35_129-OMcp35_26*RLcp35_119;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp35_129;
    sens->P[2] = POcp35_229;
    sens->P[3] = POcp35_329;
    sens->R[1][1] = ROcp35_129;
    sens->R[1][2] = ROcp35_229;
    sens->R[1][3] = ROcp35_329;
    sens->R[2][1] = ROcp35_428;
    sens->R[2][2] = ROcp35_528;
    sens->R[2][3] = ROcp35_628;
    sens->R[3][1] = ROcp35_729;
    sens->R[3][2] = ROcp35_829;
    sens->R[3][3] = ROcp35_929;
    sens->V[1] = VIcp35_129;
    sens->V[2] = VIcp35_229;
    sens->V[3] = VIcp35_329;
    sens->OM[1] = OMcp35_129;
    sens->OM[2] = OMcp35_229;
    sens->OM[3] = OMcp35_329;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

