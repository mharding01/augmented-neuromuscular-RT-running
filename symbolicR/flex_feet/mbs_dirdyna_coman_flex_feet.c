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
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 18264
//
//	==> Generation Time :  0.380 seconds
//	==> Post-Processing :  0.450 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[33][33];
// double c[33];
{ 
 
#include "mbs_dirdyna_coman_flex_feet.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

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

// = = Block_0_1_0_0_0_1 = = 
 
// Forward Kinematics 

  AlF24 = -s->g[3]*S4;
  AlF34 = -s->g[3]*C4;
  OM15 = qd[4]*C5;
  OpF15 = -qd[4]*qd[5]*S5;
  OpF35 = qd[4]*qd[5]*C5;
  AlF15 = -AlF34*S5;
  AlF35 = AlF34*C5;
  AlM15_2 = S4*S5;
  AlM35_2 = -S4*C5;
  AlM15_3 = -C4*S5;
  AlM35_3 = C4*C5;
  OM16 = qd[5]*S6+OM15*C6;
  OM26 = qd[5]*C6-OM15*S6;
  OM36 = qd[6]+qd[4]*S5;
  OpF16 = -(qd[6]*OM15*S6-C6*(OpF15+qd[5]*qd[6]));
  OpF26 = -(qd[6]*OM15*C6+S6*(OpF15+qd[5]*qd[6]));
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BeF26 = BS26-OpF35;
  BeF36 = BS36+OpF26;
  BeF46 = BS26+OpF35;
  BeF66 = BS66-OpF16;
  BeF76 = BS36-OpF26;
  BeF86 = BS66+OpF16;
  AlF16 = AlF15*C6+AlF24*S6;
  AlF26 = -(AlF15*S6-AlF24*C6);
  AlM16_1 = C5*C6;
  AlM26_1 = -C5*S6;
  AlM16_2 = AlM15_2*C6+C4*S6;
  AlM26_2 = -(AlM15_2*S6-C4*C6);
  AlM16_3 = AlM15_3*C6+S4*S6;
  AlM26_3 = -(AlM15_3*S6-S4*C6);
  OpM16_4 = C5*C6;
  OpM26_4 = -C5*S6;

// = = Block_0_1_0_1_0_2 = = 
 
// Forward Kinematics 

  OM17 = OM16*C7-OM36*S7;
  OM27 = qd[7]+OM26;
  OM37 = OM16*S7+OM36*C7;
  OpF17 = C7*(OpF16-qd[7]*OM36)-S7*(OpF35+qd[7]*OM16);
  OpF37 = C7*(OpF35+qd[7]*OM16)+S7*(OpF16-qd[7]*OM36);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BeF27 = BS27-OpF37;
  BeF87 = BS67+OpF17;
  AlF17 = C7*(AlF16+BeF26*s->dpt[2][2])-S7*(AlF35+BeF86*s->dpt[2][2]);
  AlF27 = AlF26+BS56*s->dpt[2][2];
  AlF37 = C7*(AlF35+BeF86*s->dpt[2][2])+S7*(AlF16+BeF26*s->dpt[2][2]);
  AlM17_1 = AlM16_1*C7-S5*S7;
  AlM37_1 = AlM16_1*S7+S5*C7;
  AlM17_2 = AlM16_2*C7-AlM35_2*S7;
  AlM37_2 = AlM16_2*S7+AlM35_2*C7;
  AlM17_3 = AlM16_3*C7-AlM35_3*S7;
  AlM37_3 = AlM16_3*S7+AlM35_3*C7;
  OpM17_4 = OpM16_4*C7-S5*S7;
  OpM37_4 = OpM16_4*S7+S5*C7;
  AlM17_4 = -s->dpt[2][2]*(OpM16_4*S7+S5*C7);
  AlM37_4 = s->dpt[2][2]*(OpM16_4*C7-S5*S7);
  OpM17_5 = S6*C7;
  OpM37_5 = S6*S7;
  AlM17_5 = -s->dpt[2][2]*S6*S7;
  AlM37_5 = s->dpt[2][2]*S6*C7;
  AlM17_6 = -s->dpt[2][2]*C7;
  AlM37_6 = -s->dpt[2][2]*S7;
  OM18 = qd[8]+OM17;
  OM28 = OM27*C8+OM37*S8;
  OM38 = -(OM27*S8-OM37*C8);
  OpF28 = C8*(OpF26+qd[8]*OM37)+S8*(OpF37-qd[8]*OM27);
  OpF38 = C8*(OpF37-qd[8]*OM27)-S8*(OpF26+qd[8]*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  BS98 = -(OM18*OM18+OM28*OM28);
  BeF38 = BS38+OpF28;
  BeF68 = BS68-OpF17;
  AlF18 = AlF17+BeF27*s->dpt[2][6];
  AlF28 = C8*(AlF27+BS57*s->dpt[2][6])+S8*(AlF37+BeF87*s->dpt[2][6]);
  AlF38 = C8*(AlF37+BeF87*s->dpt[2][6])-S8*(AlF27+BS57*s->dpt[2][6]);
  AlM28_1 = AlM26_1*C8+AlM37_1*S8;
  AlM38_1 = -(AlM26_1*S8-AlM37_1*C8);
  AlM28_2 = AlM26_2*C8+AlM37_2*S8;
  AlM38_2 = -(AlM26_2*S8-AlM37_2*C8);
  AlM28_3 = AlM26_3*C8+AlM37_3*S8;
  AlM38_3 = -(AlM26_3*S8-AlM37_3*C8);
  OpM28_4 = OpM26_4*C8+OpM37_4*S8;
  OpM38_4 = -(OpM26_4*S8-OpM37_4*C8);
  AlM18_4 = AlM17_4-OpM37_4*s->dpt[2][6];
  AlM28_4 = S8*(AlM37_4+OpM17_4*s->dpt[2][6]);
  AlM38_4 = C8*(AlM37_4+OpM17_4*s->dpt[2][6]);
  OpM28_5 = OpM37_5*S8+C6*C8;
  OpM38_5 = OpM37_5*C8-C6*S8;
  AlM18_5 = AlM17_5-OpM37_5*s->dpt[2][6];
  AlM28_5 = S8*(AlM37_5+OpM17_5*s->dpt[2][6]);
  AlM38_5 = C8*(AlM37_5+OpM17_5*s->dpt[2][6]);
  OpM28_6 = C7*S8;
  OpM38_6 = C7*C8;
  AlM18_6 = AlM17_6-s->dpt[2][6]*C7;
  AlM28_6 = S8*(AlM37_6-s->dpt[2][6]*S7);
  AlM38_6 = C8*(AlM37_6-s->dpt[2][6]*S7);
  OM19 = OM18*C9+OM28*S9;
  OM29 = -(OM18*S9-OM28*C9);
  OM39 = qd[9]+OM38;
  OpF19 = C9*(OpF17+qd[9]*OM28)+S9*(OpF28-qd[9]*OM18);
  OpF29 = C9*(OpF28-qd[9]*OM18)-S9*(OpF17+qd[9]*OM28);
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BeF39 = BS39+OpF29;
  BeF69 = BS69-OpF19;
  AlF19 = C9*(AlF18+BeF38*s->dpt[3][8])+S9*(AlF28+BeF68*s->dpt[3][8]);
  AlF29 = C9*(AlF28+BeF68*s->dpt[3][8])-S9*(AlF18+BeF38*s->dpt[3][8]);
  AlF39 = AlF38+BS98*s->dpt[3][8];
  AlM19_1 = AlM17_1*C9+AlM28_1*S9;
  AlM29_1 = -(AlM17_1*S9-AlM28_1*C9);
  AlM19_2 = AlM17_2*C9+AlM28_2*S9;
  AlM29_2 = -(AlM17_2*S9-AlM28_2*C9);
  AlM19_3 = AlM17_3*C9+AlM28_3*S9;
  AlM29_3 = -(AlM17_3*S9-AlM28_3*C9);
  OpM19_4 = OpM17_4*C9+OpM28_4*S9;
  OpM29_4 = -(OpM17_4*S9-OpM28_4*C9);
  AlM19_4 = C9*(AlM18_4+OpM28_4*s->dpt[3][8])+S9*(AlM28_4-OpM17_4*s->dpt[3][8]);
  AlM29_4 = C9*(AlM28_4-OpM17_4*s->dpt[3][8])-S9*(AlM18_4+OpM28_4*s->dpt[3][8]);
  OpM19_5 = OpM17_5*C9+OpM28_5*S9;
  OpM29_5 = -(OpM17_5*S9-OpM28_5*C9);
  AlM19_5 = C9*(AlM18_5+OpM28_5*s->dpt[3][8])+S9*(AlM28_5-OpM17_5*s->dpt[3][8]);
  AlM29_5 = C9*(AlM28_5-OpM17_5*s->dpt[3][8])-S9*(AlM18_5+OpM28_5*s->dpt[3][8]);
  OpM19_6 = OpM28_6*S9-S7*C9;
  OpM29_6 = OpM28_6*C9+S7*S9;
  AlM19_6 = C9*(AlM18_6+OpM28_6*s->dpt[3][8])+S9*(AlM28_6+s->dpt[3][8]*S7);
  AlM29_6 = C9*(AlM28_6+s->dpt[3][8]*S7)-S9*(AlM18_6+OpM28_6*s->dpt[3][8]);
  OpM19_7 = C8*S9;
  OpM29_7 = C8*C9;
  AlM19_7 = s->dpt[3][8]*C8*C9;
  AlM29_7 = -s->dpt[3][8]*C8*S9;
  AlM19_8 = -s->dpt[3][8]*S9;
  AlM29_8 = -s->dpt[3][8]*C9;
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd[10]+OM29;
  OM310 = OM19*S10+OM39*C10;
  OpF110 = C10*(OpF19-qd[10]*OM39)-S10*(OpF38+qd[10]*OM19);
  OpF310 = C10*(OpF38+qd[10]*OM19)+S10*(OpF19-qd[10]*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  BS910 = -(OM110*OM110+OM210*OM210);
  BeF310 = BS310+OpF29;
  BeF610 = BS610-OpF110;
  AlF110 = C10*(AlF19+BeF39*s->dpt[3][11])-S10*(AlF39+BS99*s->dpt[3][11]);
  AlF210 = AlF29+BeF69*s->dpt[3][11];
  AlF310 = C10*(AlF39+BS99*s->dpt[3][11])+S10*(AlF19+BeF39*s->dpt[3][11]);
  AlM110_1 = AlM19_1*C10-AlM38_1*S10;
  AlM310_1 = AlM19_1*S10+AlM38_1*C10;
  AlM110_2 = AlM19_2*C10-AlM38_2*S10;
  AlM310_2 = AlM19_2*S10+AlM38_2*C10;
  AlM110_3 = AlM19_3*C10-AlM38_3*S10;
  AlM310_3 = AlM19_3*S10+AlM38_3*C10;
  OpM110_4 = OpM19_4*C10-OpM38_4*S10;
  OpM310_4 = OpM19_4*S10+OpM38_4*C10;
  AlM110_4 = -(AlM38_4*S10-C10*(AlM19_4+OpM29_4*s->dpt[3][11]));
  AlM210_4 = AlM29_4-OpM19_4*s->dpt[3][11];
  AlM310_4 = AlM38_4*C10+S10*(AlM19_4+OpM29_4*s->dpt[3][11]);
  OpM110_5 = OpM19_5*C10-OpM38_5*S10;
  OpM310_5 = OpM19_5*S10+OpM38_5*C10;
  AlM110_5 = -(AlM38_5*S10-C10*(AlM19_5+OpM29_5*s->dpt[3][11]));
  AlM210_5 = AlM29_5-OpM19_5*s->dpt[3][11];
  AlM310_5 = AlM38_5*C10+S10*(AlM19_5+OpM29_5*s->dpt[3][11]);
  OpM110_6 = OpM19_6*C10-OpM38_6*S10;
  OpM310_6 = OpM19_6*S10+OpM38_6*C10;
  AlM110_6 = -(AlM38_6*S10-C10*(AlM19_6+OpM29_6*s->dpt[3][11]));
  AlM210_6 = AlM29_6-OpM19_6*s->dpt[3][11];
  AlM310_6 = AlM38_6*C10+S10*(AlM19_6+OpM29_6*s->dpt[3][11]);
  OpM110_7 = OpM19_7*C10+S10*S8;
  OpM310_7 = OpM19_7*S10-C10*S8;
  AlM110_7 = C10*(AlM19_7+OpM29_7*s->dpt[3][11]);
  AlM210_7 = AlM29_7-OpM19_7*s->dpt[3][11];
  AlM310_7 = S10*(AlM19_7+OpM29_7*s->dpt[3][11]);
  OpM110_8 = C10*C9;
  OpM310_8 = S10*C9;
  AlM110_8 = C10*(AlM19_8-s->dpt[3][11]*S9);
  AlM210_8 = AlM29_8-s->dpt[3][11]*C9;
  AlM310_8 = S10*(AlM19_8-s->dpt[3][11]*S9);
  OM111 = qd[11]+OM110;
  OM211 = OM210*C11+OM310*S11;
  OM311 = -(OM210*S11-OM310*C11);
  OpF211 = C11*(OpF29+qd[11]*OM310)+S11*(OpF310-qd[11]*OM210);
  OpF311 = C11*(OpF310-qd[11]*OM210)-S11*(OpF29+qd[11]*OM310);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS611 = OM211*OM311;
  AlF111 = AlF110+BeF310*s->dpt[3][14];
  AlF211 = C11*(AlF210+BeF610*s->dpt[3][14])+S11*(AlF310+BS910*s->dpt[3][14]);
  AlF311 = C11*(AlF310+BS910*s->dpt[3][14])-S11*(AlF210+BeF610*s->dpt[3][14]);
  AlM211_1 = AlM29_1*C11+AlM310_1*S11;
  AlM311_1 = -(AlM29_1*S11-AlM310_1*C11);
  AlM211_2 = AlM29_2*C11+AlM310_2*S11;
  AlM311_2 = -(AlM29_2*S11-AlM310_2*C11);
  AlM211_3 = AlM29_3*C11+AlM310_3*S11;
  AlM311_3 = -(AlM29_3*S11-AlM310_3*C11);
  OpM211_4 = OpM29_4*C11+OpM310_4*S11;
  OpM311_4 = -(OpM29_4*S11-OpM310_4*C11);
  AlM111_4 = AlM110_4+OpM29_4*s->dpt[3][14];
  AlM211_4 = AlM310_4*S11+C11*(AlM210_4-OpM110_4*s->dpt[3][14]);
  AlM311_4 = AlM310_4*C11-S11*(AlM210_4-OpM110_4*s->dpt[3][14]);
  OpM211_5 = OpM29_5*C11+OpM310_5*S11;
  OpM311_5 = -(OpM29_5*S11-OpM310_5*C11);
  AlM111_5 = AlM110_5+OpM29_5*s->dpt[3][14];
  AlM211_5 = AlM310_5*S11+C11*(AlM210_5-OpM110_5*s->dpt[3][14]);
  AlM311_5 = AlM310_5*C11-S11*(AlM210_5-OpM110_5*s->dpt[3][14]);
  OpM211_6 = OpM29_6*C11+OpM310_6*S11;
  OpM311_6 = -(OpM29_6*S11-OpM310_6*C11);
  AlM111_6 = AlM110_6+OpM29_6*s->dpt[3][14];
  AlM211_6 = AlM310_6*S11+C11*(AlM210_6-OpM110_6*s->dpt[3][14]);
  AlM311_6 = AlM310_6*C11-S11*(AlM210_6-OpM110_6*s->dpt[3][14]);
  OpM211_7 = OpM29_7*C11+OpM310_7*S11;
  OpM311_7 = -(OpM29_7*S11-OpM310_7*C11);
  AlM111_7 = AlM110_7+OpM29_7*s->dpt[3][14];
  AlM211_7 = AlM310_7*S11+C11*(AlM210_7-OpM110_7*s->dpt[3][14]);
  AlM311_7 = AlM310_7*C11-S11*(AlM210_7-OpM110_7*s->dpt[3][14]);
  OpM211_8 = OpM310_8*S11-C11*S9;
  OpM311_8 = OpM310_8*C11+S11*S9;
  AlM111_8 = AlM110_8-s->dpt[3][14]*S9;
  AlM211_8 = AlM310_8*S11+C11*(AlM210_8-OpM110_8*s->dpt[3][14]);
  AlM311_8 = AlM310_8*C11-S11*(AlM210_8-OpM110_8*s->dpt[3][14]);
  OpM211_9 = C10*S11;
  OpM311_9 = C10*C11;
  AlM211_9 = s->dpt[3][14]*S10*C11;
  AlM311_9 = -s->dpt[3][14]*S10*S11;
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OpF112 = C12*(OpF110-qd[12]*OM311)-S12*(OpF311+qd[12]*OM111);
  OpF312 = C12*(OpF311+qd[12]*OM111)+S12*(OpF110-qd[12]*OM311);
  BS112 = -(OM212*OM212+OM312*OM312);
  BS312 = OM112*OM312;
  BS912 = -(OM112*OM112+OM212*OM212);
  BeF312 = BS312+OpF211;
  BeF412 = OpF312+OM112*OM212;
  BeF612 = OM212*OM312-OpF112;
  BeF712 = BS312-OpF211;
  AlF112 = AlF111*C12-AlF311*S12;
  AlF312 = AlF111*S12+AlF311*C12;
  AlM112_1 = AlM110_1*C12-AlM311_1*S12;
  AlM312_1 = AlM110_1*S12+AlM311_1*C12;
  AlM112_2 = AlM110_2*C12-AlM311_2*S12;
  AlM312_2 = AlM110_2*S12+AlM311_2*C12;
  AlM112_3 = AlM110_3*C12-AlM311_3*S12;
  AlM312_3 = AlM110_3*S12+AlM311_3*C12;
  OpM112_4 = OpM110_4*C12-OpM311_4*S12;
  OpM312_4 = OpM110_4*S12+OpM311_4*C12;
  AlM112_4 = AlM111_4*C12-AlM311_4*S12;
  AlM312_4 = AlM111_4*S12+AlM311_4*C12;
  OpM112_5 = OpM110_5*C12-OpM311_5*S12;
  OpM312_5 = OpM110_5*S12+OpM311_5*C12;
  AlM112_5 = AlM111_5*C12-AlM311_5*S12;
  AlM312_5 = AlM111_5*S12+AlM311_5*C12;
  OpM112_6 = OpM110_6*C12-OpM311_6*S12;
  OpM312_6 = OpM110_6*S12+OpM311_6*C12;
  AlM112_6 = AlM111_6*C12-AlM311_6*S12;
  AlM312_6 = AlM111_6*S12+AlM311_6*C12;
  OpM112_7 = OpM110_7*C12-OpM311_7*S12;
  OpM312_7 = OpM110_7*S12+OpM311_7*C12;
  AlM112_7 = AlM111_7*C12-AlM311_7*S12;
  AlM312_7 = AlM111_7*S12+AlM311_7*C12;
  OpM112_8 = OpM110_8*C12-OpM311_8*S12;
  OpM312_8 = OpM110_8*S12+OpM311_8*C12;
  AlM112_8 = AlM111_8*C12-AlM311_8*S12;
  AlM312_8 = AlM111_8*S12+AlM311_8*C12;
  OpM112_9 = -(OpM311_9*S12+S10*C12);
  OpM312_9 = OpM311_9*C12-S10*S12;
  AlM112_9 = -AlM311_9*S12;
  AlM312_9 = AlM311_9*C12;
  OpM112_10 = S11*S12;
  OpM312_10 = -S11*C12;
  AlM112_10 = s->dpt[3][14]*C12;
  AlM312_10 = s->dpt[3][14]*S12;

// = = Block_0_1_0_1_0_5 = = 
 
// Forward Kinematics 

  OM115 = OM16*C15-OM36*S15;
  OM215 = qd[15]+OM26;
  OM315 = OM16*S15+OM36*C15;
  OpF115 = C15*(OpF16-qd[15]*OM36)-S15*(OpF35+qd[15]*OM16);
  OpF315 = C15*(OpF35+qd[15]*OM16)+S15*(OpF16-qd[15]*OM36);
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS515 = -(OM115*OM115+OM315*OM315);
  BS615 = OM215*OM315;
  BeF215 = BS215-OpF315;
  BeF815 = BS615+OpF115;
  AlF115 = C15*(AlF16+BeF26*s->dpt[2][3])-S15*(AlF35+BeF86*s->dpt[2][3]);
  AlF215 = AlF26+BS56*s->dpt[2][3];
  AlF315 = C15*(AlF35+BeF86*s->dpt[2][3])+S15*(AlF16+BeF26*s->dpt[2][3]);
  AlM115_1 = AlM16_1*C15-S15*S5;
  AlM315_1 = AlM16_1*S15+C15*S5;
  AlM115_2 = AlM16_2*C15-AlM35_2*S15;
  AlM315_2 = AlM16_2*S15+AlM35_2*C15;
  AlM115_3 = AlM16_3*C15-AlM35_3*S15;
  AlM315_3 = AlM16_3*S15+AlM35_3*C15;
  OpM115_4 = OpM16_4*C15-S15*S5;
  OpM315_4 = OpM16_4*S15+C15*S5;
  AlM115_4 = -s->dpt[2][3]*(OpM16_4*S15+C15*S5);
  AlM315_4 = s->dpt[2][3]*(OpM16_4*C15-S15*S5);
  OpM115_5 = C15*S6;
  OpM315_5 = S15*S6;
  AlM115_5 = -s->dpt[2][3]*S15*S6;
  AlM315_5 = s->dpt[2][3]*C15*S6;
  AlM115_6 = -s->dpt[2][3]*C15;
  AlM315_6 = -s->dpt[2][3]*S15;
  OM116 = qd[16]+OM115;
  OM216 = OM215*C16+OM315*S16;
  OM316 = -(OM215*S16-OM315*C16);
  OpF216 = C16*(OpF26+qd[16]*OM315)+S16*(OpF315-qd[16]*OM215);
  OpF316 = C16*(OpF315-qd[16]*OM215)-S16*(OpF26+qd[16]*OM315);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BeF316 = BS316+OpF216;
  BeF616 = BS616-OpF115;
  AlF116 = AlF115+BeF215*s->dpt[2][24];
  AlF216 = C16*(AlF215+BS515*s->dpt[2][24])+S16*(AlF315+BeF815*s->dpt[2][24]);
  AlF316 = C16*(AlF315+BeF815*s->dpt[2][24])-S16*(AlF215+BS515*s->dpt[2][24]);
  AlM216_1 = AlM26_1*C16+AlM315_1*S16;
  AlM316_1 = -(AlM26_1*S16-AlM315_1*C16);
  AlM216_2 = AlM26_2*C16+AlM315_2*S16;
  AlM316_2 = -(AlM26_2*S16-AlM315_2*C16);
  AlM216_3 = AlM26_3*C16+AlM315_3*S16;
  AlM316_3 = -(AlM26_3*S16-AlM315_3*C16);
  OpM216_4 = OpM26_4*C16+OpM315_4*S16;
  OpM316_4 = -(OpM26_4*S16-OpM315_4*C16);
  AlM116_4 = AlM115_4-OpM315_4*s->dpt[2][24];
  AlM216_4 = S16*(AlM315_4+OpM115_4*s->dpt[2][24]);
  AlM316_4 = C16*(AlM315_4+OpM115_4*s->dpt[2][24]);
  OpM216_5 = OpM315_5*S16+C16*C6;
  OpM316_5 = OpM315_5*C16-S16*C6;
  AlM116_5 = AlM115_5-OpM315_5*s->dpt[2][24];
  AlM216_5 = S16*(AlM315_5+OpM115_5*s->dpt[2][24]);
  AlM316_5 = C16*(AlM315_5+OpM115_5*s->dpt[2][24]);
  OpM216_6 = C15*S16;
  OpM316_6 = C15*C16;
  AlM116_6 = AlM115_6-s->dpt[2][24]*C15;
  AlM216_6 = S16*(AlM315_6-s->dpt[2][24]*S15);
  AlM316_6 = C16*(AlM315_6-s->dpt[2][24]*S15);
  OM117 = OM116*C17+OM216*S17;
  OM217 = -(OM116*S17-OM216*C17);
  OM317 = qd[17]+OM316;
  OpF117 = C17*(OpF115+qd[17]*OM216)+S17*(OpF216-qd[17]*OM116);
  OpF217 = C17*(OpF216-qd[17]*OM116)-S17*(OpF115+qd[17]*OM216);
  BS217 = OM117*OM217;
  BS317 = OM117*OM317;
  BS617 = OM217*OM317;
  BS917 = -(OM117*OM117+OM217*OM217);
  BeF317 = BS317+OpF217;
  BeF617 = BS617-OpF117;
  AlF117 = C17*(AlF116+BeF316*s->dpt[3][26])+S17*(AlF216+BeF616*s->dpt[3][26]);
  AlF217 = C17*(AlF216+BeF616*s->dpt[3][26])-S17*(AlF116+BeF316*s->dpt[3][26]);
  AlF317 = AlF316+BS916*s->dpt[3][26];
  AlM117_1 = AlM115_1*C17+AlM216_1*S17;
  AlM217_1 = -(AlM115_1*S17-AlM216_1*C17);
  AlM117_2 = AlM115_2*C17+AlM216_2*S17;
  AlM217_2 = -(AlM115_2*S17-AlM216_2*C17);
  AlM117_3 = AlM115_3*C17+AlM216_3*S17;
  AlM217_3 = -(AlM115_3*S17-AlM216_3*C17);
  OpM117_4 = OpM115_4*C17+OpM216_4*S17;
  OpM217_4 = -(OpM115_4*S17-OpM216_4*C17);
  AlM117_4 = C17*(AlM116_4+OpM216_4*s->dpt[3][26])+S17*(AlM216_4-OpM115_4*s->dpt[3][26]);
  AlM217_4 = C17*(AlM216_4-OpM115_4*s->dpt[3][26])-S17*(AlM116_4+OpM216_4*s->dpt[3][26]);
  OpM117_5 = OpM115_5*C17+OpM216_5*S17;
  OpM217_5 = -(OpM115_5*S17-OpM216_5*C17);
  AlM117_5 = C17*(AlM116_5+OpM216_5*s->dpt[3][26])+S17*(AlM216_5-OpM115_5*s->dpt[3][26]);
  AlM217_5 = C17*(AlM216_5-OpM115_5*s->dpt[3][26])-S17*(AlM116_5+OpM216_5*s->dpt[3][26]);
  OpM117_6 = OpM216_6*S17-S15*C17;
  OpM217_6 = OpM216_6*C17+S15*S17;
  AlM117_6 = C17*(AlM116_6+OpM216_6*s->dpt[3][26])+S17*(AlM216_6+s->dpt[3][26]*S15);
  AlM217_6 = C17*(AlM216_6+s->dpt[3][26]*S15)-S17*(AlM116_6+OpM216_6*s->dpt[3][26]);
  OpM117_15 = C16*S17;
  OpM217_15 = C16*C17;
  AlM117_15 = s->dpt[3][26]*C16*C17;
  AlM217_15 = -s->dpt[3][26]*C16*S17;
  AlM117_16 = -s->dpt[3][26]*S17;
  AlM217_16 = -s->dpt[3][26]*C17;
  OM118 = OM117*C18-OM317*S18;
  OM218 = qd[18]+OM217;
  OM318 = OM117*S18+OM317*C18;
  OpF118 = C18*(OpF117-qd[18]*OM317)-S18*(OpF316+qd[18]*OM117);
  OpF318 = C18*(OpF316+qd[18]*OM117)+S18*(OpF117-qd[18]*OM317);
  BS218 = OM118*OM218;
  BS318 = OM118*OM318;
  BS618 = OM218*OM318;
  BS918 = -(OM118*OM118+OM218*OM218);
  BeF318 = BS318+OpF217;
  BeF618 = BS618-OpF118;
  AlF118 = C18*(AlF117+BeF317*s->dpt[3][29])-S18*(AlF317+BS917*s->dpt[3][29]);
  AlF218 = AlF217+BeF617*s->dpt[3][29];
  AlF318 = C18*(AlF317+BS917*s->dpt[3][29])+S18*(AlF117+BeF317*s->dpt[3][29]);
  AlM118_1 = AlM117_1*C18-AlM316_1*S18;
  AlM318_1 = AlM117_1*S18+AlM316_1*C18;
  AlM118_2 = AlM117_2*C18-AlM316_2*S18;
  AlM318_2 = AlM117_2*S18+AlM316_2*C18;
  AlM118_3 = AlM117_3*C18-AlM316_3*S18;
  AlM318_3 = AlM117_3*S18+AlM316_3*C18;
  OpM118_4 = OpM117_4*C18-OpM316_4*S18;
  OpM318_4 = OpM117_4*S18+OpM316_4*C18;
  AlM118_4 = -(AlM316_4*S18-C18*(AlM117_4+OpM217_4*s->dpt[3][29]));
  AlM218_4 = AlM217_4-OpM117_4*s->dpt[3][29];
  AlM318_4 = AlM316_4*C18+S18*(AlM117_4+OpM217_4*s->dpt[3][29]);
  OpM118_5 = OpM117_5*C18-OpM316_5*S18;
  OpM318_5 = OpM117_5*S18+OpM316_5*C18;
  AlM118_5 = -(AlM316_5*S18-C18*(AlM117_5+OpM217_5*s->dpt[3][29]));
  AlM218_5 = AlM217_5-OpM117_5*s->dpt[3][29];
  AlM318_5 = AlM316_5*C18+S18*(AlM117_5+OpM217_5*s->dpt[3][29]);
  OpM118_6 = OpM117_6*C18-OpM316_6*S18;
  OpM318_6 = OpM117_6*S18+OpM316_6*C18;
  AlM118_6 = -(AlM316_6*S18-C18*(AlM117_6+OpM217_6*s->dpt[3][29]));
  AlM218_6 = AlM217_6-OpM117_6*s->dpt[3][29];
  AlM318_6 = AlM316_6*C18+S18*(AlM117_6+OpM217_6*s->dpt[3][29]);
  OpM118_15 = OpM117_15*C18+S16*S18;
  OpM318_15 = OpM117_15*S18-S16*C18;
  AlM118_15 = C18*(AlM117_15+OpM217_15*s->dpt[3][29]);
  AlM218_15 = AlM217_15-OpM117_15*s->dpt[3][29];
  AlM318_15 = S18*(AlM117_15+OpM217_15*s->dpt[3][29]);
  OpM118_16 = C17*C18;
  OpM318_16 = C17*S18;
  AlM118_16 = C18*(AlM117_16-s->dpt[3][29]*S17);
  AlM218_16 = AlM217_16-s->dpt[3][29]*C17;
  AlM318_16 = S18*(AlM117_16-s->dpt[3][29]*S17);
  OM119 = qd[19]+OM118;
  OM219 = OM218*C19+OM318*S19;
  OM319 = -(OM218*S19-OM318*C19);
  OpF219 = C19*(OpF217+qd[19]*OM318)+S19*(OpF318-qd[19]*OM218);
  OpF319 = C19*(OpF318-qd[19]*OM218)-S19*(OpF217+qd[19]*OM318);
  BS219 = OM119*OM219;
  BS319 = OM119*OM319;
  BS619 = OM219*OM319;
  AlF119 = AlF118+BeF318*s->dpt[3][32];
  AlF219 = C19*(AlF218+BeF618*s->dpt[3][32])+S19*(AlF318+BS918*s->dpt[3][32]);
  AlF319 = C19*(AlF318+BS918*s->dpt[3][32])-S19*(AlF218+BeF618*s->dpt[3][32]);
  AlM219_1 = AlM217_1*C19+AlM318_1*S19;
  AlM319_1 = -(AlM217_1*S19-AlM318_1*C19);
  AlM219_2 = AlM217_2*C19+AlM318_2*S19;
  AlM319_2 = -(AlM217_2*S19-AlM318_2*C19);
  AlM219_3 = AlM217_3*C19+AlM318_3*S19;
  AlM319_3 = -(AlM217_3*S19-AlM318_3*C19);
  OpM219_4 = OpM217_4*C19+OpM318_4*S19;
  OpM319_4 = -(OpM217_4*S19-OpM318_4*C19);
  AlM119_4 = AlM118_4+OpM217_4*s->dpt[3][32];
  AlM219_4 = AlM318_4*S19+C19*(AlM218_4-OpM118_4*s->dpt[3][32]);
  AlM319_4 = AlM318_4*C19-S19*(AlM218_4-OpM118_4*s->dpt[3][32]);
  OpM219_5 = OpM217_5*C19+OpM318_5*S19;
  OpM319_5 = -(OpM217_5*S19-OpM318_5*C19);
  AlM119_5 = AlM118_5+OpM217_5*s->dpt[3][32];
  AlM219_5 = AlM318_5*S19+C19*(AlM218_5-OpM118_5*s->dpt[3][32]);
  AlM319_5 = AlM318_5*C19-S19*(AlM218_5-OpM118_5*s->dpt[3][32]);
  OpM219_6 = OpM217_6*C19+OpM318_6*S19;
  OpM319_6 = -(OpM217_6*S19-OpM318_6*C19);
  AlM119_6 = AlM118_6+OpM217_6*s->dpt[3][32];
  AlM219_6 = AlM318_6*S19+C19*(AlM218_6-OpM118_6*s->dpt[3][32]);
  AlM319_6 = AlM318_6*C19-S19*(AlM218_6-OpM118_6*s->dpt[3][32]);
  OpM219_15 = OpM217_15*C19+OpM318_15*S19;
  OpM319_15 = -(OpM217_15*S19-OpM318_15*C19);
  AlM119_15 = AlM118_15+OpM217_15*s->dpt[3][32];
  AlM219_15 = AlM318_15*S19+C19*(AlM218_15-OpM118_15*s->dpt[3][32]);
  AlM319_15 = AlM318_15*C19-S19*(AlM218_15-OpM118_15*s->dpt[3][32]);
  OpM219_16 = OpM318_16*S19-S17*C19;
  OpM319_16 = OpM318_16*C19+S17*S19;
  AlM119_16 = AlM118_16-s->dpt[3][32]*S17;
  AlM219_16 = AlM318_16*S19+C19*(AlM218_16-OpM118_16*s->dpt[3][32]);
  AlM319_16 = AlM318_16*C19-S19*(AlM218_16-OpM118_16*s->dpt[3][32]);
  OpM219_17 = C18*S19;
  OpM319_17 = C18*C19;
  AlM219_17 = s->dpt[3][32]*S18*C19;
  AlM319_17 = -s->dpt[3][32]*S18*S19;
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM219;
  OM320 = OM119*S20+OM319*C20;
  OpF120 = C20*(OpF118-qd[20]*OM319)-S20*(OpF319+qd[20]*OM119);
  OpF320 = C20*(OpF319+qd[20]*OM119)+S20*(OpF118-qd[20]*OM319);
  BS120 = -(OM220*OM220+OM320*OM320);
  BS320 = OM120*OM320;
  BS920 = -(OM120*OM120+OM220*OM220);
  BeF320 = BS320+OpF219;
  BeF420 = OpF320+OM120*OM220;
  BeF620 = OM220*OM320-OpF120;
  BeF720 = BS320-OpF219;
  AlF120 = AlF119*C20-AlF319*S20;
  AlF320 = AlF119*S20+AlF319*C20;
  AlM120_1 = AlM118_1*C20-AlM319_1*S20;
  AlM320_1 = AlM118_1*S20+AlM319_1*C20;
  AlM120_2 = AlM118_2*C20-AlM319_2*S20;
  AlM320_2 = AlM118_2*S20+AlM319_2*C20;
  AlM120_3 = AlM118_3*C20-AlM319_3*S20;
  AlM320_3 = AlM118_3*S20+AlM319_3*C20;
  OpM120_4 = OpM118_4*C20-OpM319_4*S20;
  OpM320_4 = OpM118_4*S20+OpM319_4*C20;
  AlM120_4 = AlM119_4*C20-AlM319_4*S20;
  AlM320_4 = AlM119_4*S20+AlM319_4*C20;
  OpM120_5 = OpM118_5*C20-OpM319_5*S20;
  OpM320_5 = OpM118_5*S20+OpM319_5*C20;
  AlM120_5 = AlM119_5*C20-AlM319_5*S20;
  AlM320_5 = AlM119_5*S20+AlM319_5*C20;
  OpM120_6 = OpM118_6*C20-OpM319_6*S20;
  OpM320_6 = OpM118_6*S20+OpM319_6*C20;
  AlM120_6 = AlM119_6*C20-AlM319_6*S20;
  AlM320_6 = AlM119_6*S20+AlM319_6*C20;
  OpM120_15 = OpM118_15*C20-OpM319_15*S20;
  OpM320_15 = OpM118_15*S20+OpM319_15*C20;
  AlM120_15 = AlM119_15*C20-AlM319_15*S20;
  AlM320_15 = AlM119_15*S20+AlM319_15*C20;
  OpM120_16 = OpM118_16*C20-OpM319_16*S20;
  OpM320_16 = OpM118_16*S20+OpM319_16*C20;
  AlM120_16 = AlM119_16*C20-AlM319_16*S20;
  AlM320_16 = AlM119_16*S20+AlM319_16*C20;
  OpM120_17 = -(OpM319_17*S20+S18*C20);
  OpM320_17 = OpM319_17*C20-S18*S20;
  AlM120_17 = -AlM319_17*S20;
  AlM320_17 = AlM319_17*C20;
  OpM120_18 = S19*S20;
  OpM320_18 = -S19*C20;
  AlM120_18 = s->dpt[3][32]*C20;
  AlM320_18 = s->dpt[3][32]*S20;

// = = Block_0_1_0_1_0_8 = = 
 
// Forward Kinematics 

  OM123 = qd[23]+OM16;
  OM223 = OM26*C23+OM36*S23;
  OM323 = -(OM26*S23-OM36*C23);
  OpF223 = C23*(OpF26+qd[23]*OM36)+S23*(OpF35-qd[23]*OM26);
  OpF323 = C23*(OpF35-qd[23]*OM26)-S23*(OpF26+qd[23]*OM36);
  BS223 = OM123*OM223;
  BS323 = OM123*OM323;
  BS623 = OM223*OM323;
  AlF123 = AlF16+BS16*s->dpt[1][4]+BeF36*s->dpt[3][4];
  AlF223 = C23*(AlF26+BeF46*s->dpt[1][4]+BeF66*s->dpt[3][4])+S23*(AlF35+BS96*s->dpt[3][4]+BeF76*s->dpt[1][4]);
  AlF323 = C23*(AlF35+BS96*s->dpt[3][4]+BeF76*s->dpt[1][4])-S23*(AlF26+BeF46*s->dpt[1][4]+BeF66*s->dpt[3][4]);
  AlM223_1 = AlM26_1*C23+S23*S5;
  AlM323_1 = -(AlM26_1*S23-C23*S5);
  AlM223_2 = AlM26_2*C23+AlM35_2*S23;
  AlM323_2 = -(AlM26_2*S23-AlM35_2*C23);
  AlM223_3 = AlM26_3*C23+AlM35_3*S23;
  AlM323_3 = -(AlM26_3*S23-AlM35_3*C23);
  OpM223_4 = OpM26_4*C23+S23*S5;
  OpM323_4 = -(OpM26_4*S23-C23*S5);
  AlM123_4 = OpM26_4*s->dpt[3][4];
  AlM223_4 = -(OpM26_4*s->dpt[1][4]*S23+C23*(OpM16_4*s->dpt[3][4]-s->dpt[1][4]*S5));
  AlM323_4 = -(OpM26_4*s->dpt[1][4]*C23-S23*(OpM16_4*s->dpt[3][4]-s->dpt[1][4]*S5));
  OpM223_5 = C23*C6;
  OpM323_5 = -S23*C6;
  AlM123_5 = s->dpt[3][4]*C6;
  AlM223_5 = -(s->dpt[1][4]*S23*C6+s->dpt[3][4]*C23*S6);
  AlM323_5 = -(s->dpt[1][4]*C23*C6-s->dpt[3][4]*S23*S6);
  AlM223_6 = s->dpt[1][4]*C23;
  AlM323_6 = -s->dpt[1][4]*S23;
  OM124 = OM123*C24-OM323*S24;
  OM224 = qd[24]+OM223;
  OM324 = OM123*S24+OM323*C24;
  OpF124 = C24*(OpF16-qd[24]*OM323)-S24*(OpF323+qd[24]*OM123);
  OpF324 = C24*(OpF323+qd[24]*OM123)+S24*(OpF16-qd[24]*OM323);
  BS224 = OM124*OM224;
  BS324 = OM124*OM324;
  BS624 = OM224*OM324;
  BS924 = -(OM124*OM124+OM224*OM224);
  BeF324 = BS324+OpF223;
  BeF624 = BS624-OpF124;
  AlF124 = AlF123*C24-AlF323*S24;
  AlF324 = AlF123*S24+AlF323*C24;
  AlM124_1 = AlM16_1*C24-AlM323_1*S24;
  AlM324_1 = AlM16_1*S24+AlM323_1*C24;
  AlM124_2 = AlM16_2*C24-AlM323_2*S24;
  AlM324_2 = AlM16_2*S24+AlM323_2*C24;
  AlM124_3 = AlM16_3*C24-AlM323_3*S24;
  AlM324_3 = AlM16_3*S24+AlM323_3*C24;
  OpM124_4 = OpM16_4*C24-OpM323_4*S24;
  OpM324_4 = OpM16_4*S24+OpM323_4*C24;
  AlM124_4 = AlM123_4*C24-AlM323_4*S24;
  AlM324_4 = AlM123_4*S24+AlM323_4*C24;
  OpM124_5 = -(OpM323_5*S24-C24*S6);
  OpM324_5 = OpM323_5*C24+S24*S6;
  AlM124_5 = AlM123_5*C24-AlM323_5*S24;
  AlM324_5 = AlM123_5*S24+AlM323_5*C24;
  OpM124_6 = -C23*S24;
  OpM324_6 = C23*C24;
  AlM124_6 = -AlM323_6*S24;
  AlM324_6 = AlM323_6*C24;
  OM125 = OM124*C25+OM224*S25;
  OM225 = -(OM124*S25-OM224*C25);
  OM325 = qd[25]+OM324;
  OpF125 = C25*(OpF124+qd[25]*OM224)+S25*(OpF223-qd[25]*OM124);
  OpF225 = C25*(OpF223-qd[25]*OM124)-S25*(OpF124+qd[25]*OM224);
  BS125 = -(OM225*OM225+OM325*OM325);
  BS225 = OM125*OM225;
  BS325 = OM125*OM325;
  BS525 = -(OM125*OM125+OM325*OM325);
  BS625 = OM225*OM325;
  BS925 = -(OM125*OM125+OM225*OM225);
  BeF225 = BS225-OpF324;
  BeF325 = BS325+OpF225;
  BeF425 = BS225+OpF324;
  BeF625 = BS625-OpF125;
  BeF725 = BS325-OpF225;
  BeF825 = BS625+OpF125;
  AlF125 = C25*(AlF124+BeF324*s->dpt[3][44])+S25*(AlF223+BeF624*s->dpt[3][44]);
  AlF225 = C25*(AlF223+BeF624*s->dpt[3][44])-S25*(AlF124+BeF324*s->dpt[3][44]);
  AlF325 = AlF324+BS924*s->dpt[3][44];
  AlM125_1 = AlM124_1*C25+AlM223_1*S25;
  AlM225_1 = -(AlM124_1*S25-AlM223_1*C25);
  AlM125_2 = AlM124_2*C25+AlM223_2*S25;
  AlM225_2 = -(AlM124_2*S25-AlM223_2*C25);
  AlM125_3 = AlM124_3*C25+AlM223_3*S25;
  AlM225_3 = -(AlM124_3*S25-AlM223_3*C25);
  OpM125_4 = OpM124_4*C25+OpM223_4*S25;
  OpM225_4 = -(OpM124_4*S25-OpM223_4*C25);
  AlM125_4 = C25*(AlM124_4+OpM223_4*s->dpt[3][44])+S25*(AlM223_4-OpM124_4*s->dpt[3][44]);
  AlM225_4 = C25*(AlM223_4-OpM124_4*s->dpt[3][44])-S25*(AlM124_4+OpM223_4*s->dpt[3][44]);
  OpM125_5 = OpM124_5*C25+OpM223_5*S25;
  OpM225_5 = -(OpM124_5*S25-OpM223_5*C25);
  AlM125_5 = C25*(AlM124_5+OpM223_5*s->dpt[3][44])+S25*(AlM223_5-OpM124_5*s->dpt[3][44]);
  AlM225_5 = C25*(AlM223_5-OpM124_5*s->dpt[3][44])-S25*(AlM124_5+OpM223_5*s->dpt[3][44]);
  OpM125_6 = OpM124_6*C25+S23*S25;
  OpM225_6 = -(OpM124_6*S25-S23*C25);
  AlM125_6 = C25*(AlM124_6+s->dpt[3][44]*S23)+S25*(AlM223_6-OpM124_6*s->dpt[3][44]);
  AlM225_6 = C25*(AlM223_6-OpM124_6*s->dpt[3][44])-S25*(AlM124_6+s->dpt[3][44]*S23);
  OpM125_23 = C24*C25;
  OpM225_23 = -C24*S25;
  AlM125_23 = -s->dpt[3][44]*C24*S25;
  AlM225_23 = -s->dpt[3][44]*C24*C25;
  AlM225_24 = -s->dpt[3][44]*S25;

// = = Block_0_1_0_2_0_9 = = 
 
// Forward Kinematics 

  OM126 = OM125*C26-OM325*S26;
  OM226 = qd[26]+OM225;
  OM326 = OM125*S26+OM325*C26;
  OpF126 = C26*(OpF125-qd[26]*OM325)-S26*(OpF324+qd[26]*OM125);
  OpF326 = C26*(OpF324+qd[26]*OM125)+S26*(OpF125-qd[26]*OM325);
  BS226 = OM126*OM226;
  BS326 = OM126*OM326;
  BS526 = -(OM126*OM126+OM326*OM326);
  BS626 = OM226*OM326;
  BeF226 = BS226-OpF326;
  BeF826 = BS626+OpF126;
  AlF126 = C26*(AlF125+BS125*s->dpt[1][48]+BeF225*s->dpt[2][48]+BeF325*s->dpt[3][48])-S26*(AlF325+BS925*s->dpt[3][48]+
 BeF725*s->dpt[1][48]+BeF825*s->dpt[2][48]);
  AlF226 = AlF225+BS525*s->dpt[2][48]+BeF425*s->dpt[1][48]+BeF625*s->dpt[3][48];
  AlF326 = C26*(AlF325+BS925*s->dpt[3][48]+BeF725*s->dpt[1][48]+BeF825*s->dpt[2][48])+S26*(AlF125+BS125*s->dpt[1][48]+
 BeF225*s->dpt[2][48]+BeF325*s->dpt[3][48]);
  AlM126_1 = AlM125_1*C26-AlM324_1*S26;
  AlM326_1 = AlM125_1*S26+AlM324_1*C26;
  AlM126_2 = AlM125_2*C26-AlM324_2*S26;
  AlM326_2 = AlM125_2*S26+AlM324_2*C26;
  AlM126_3 = AlM125_3*C26-AlM324_3*S26;
  AlM326_3 = AlM125_3*S26+AlM324_3*C26;
  OpM126_4 = OpM125_4*C26-OpM324_4*S26;
  OpM326_4 = OpM125_4*S26+OpM324_4*C26;
  AlM126_4 = C26*(AlM125_4+OpM225_4*s->dpt[3][48]-OpM324_4*s->dpt[2][48])-S26*(AlM324_4+OpM125_4*s->dpt[2][48]-OpM225_4*
 s->dpt[1][48]);
  AlM226_4 = AlM225_4-OpM125_4*s->dpt[3][48]+OpM324_4*s->dpt[1][48];
  AlM326_4 = C26*(AlM324_4+OpM125_4*s->dpt[2][48]-OpM225_4*s->dpt[1][48])+S26*(AlM125_4+OpM225_4*s->dpt[3][48]-OpM324_4*
 s->dpt[2][48]);
  OpM126_5 = OpM125_5*C26-OpM324_5*S26;
  OpM326_5 = OpM125_5*S26+OpM324_5*C26;
  AlM126_5 = C26*(AlM125_5+OpM225_5*s->dpt[3][48]-OpM324_5*s->dpt[2][48])-S26*(AlM324_5+OpM125_5*s->dpt[2][48]-OpM225_5*
 s->dpt[1][48]);
  AlM226_5 = AlM225_5-OpM125_5*s->dpt[3][48]+OpM324_5*s->dpt[1][48];
  AlM326_5 = C26*(AlM324_5+OpM125_5*s->dpt[2][48]-OpM225_5*s->dpt[1][48])+S26*(AlM125_5+OpM225_5*s->dpt[3][48]-OpM324_5*
 s->dpt[2][48]);
  OpM126_6 = OpM125_6*C26-OpM324_6*S26;
  OpM326_6 = OpM125_6*S26+OpM324_6*C26;
  AlM126_6 = C26*(AlM125_6+OpM225_6*s->dpt[3][48]-OpM324_6*s->dpt[2][48])-S26*(AlM324_6+OpM125_6*s->dpt[2][48]-OpM225_6*
 s->dpt[1][48]);
  AlM226_6 = AlM225_6-OpM125_6*s->dpt[3][48]+OpM324_6*s->dpt[1][48];
  AlM326_6 = C26*(AlM324_6+OpM125_6*s->dpt[2][48]-OpM225_6*s->dpt[1][48])+S26*(AlM125_6+OpM225_6*s->dpt[3][48]-OpM324_6*
 s->dpt[2][48]);
  OpM126_23 = OpM125_23*C26-S24*S26;
  OpM326_23 = OpM125_23*S26+S24*C26;
  AlM126_23 = C26*(AlM125_23+OpM225_23*s->dpt[3][48]-s->dpt[2][48]*S24)-S26*(OpM125_23*s->dpt[2][48]-OpM225_23*
 s->dpt[1][48]);
  AlM226_23 = AlM225_23-OpM125_23*s->dpt[3][48]+s->dpt[1][48]*S24;
  AlM326_23 = C26*(OpM125_23*s->dpt[2][48]-OpM225_23*s->dpt[1][48])+S26*(AlM125_23+OpM225_23*s->dpt[3][48]-s->dpt[2][48]
 *S24);
  OpM126_24 = S25*C26;
  OpM326_24 = S25*S26;
  AlM126_24 = C25*C26*(s->dpt[3][44]+s->dpt[3][48])+S26*(s->dpt[1][48]*C25-s->dpt[2][48]*S25);
  AlM226_24 = AlM225_24-s->dpt[3][48]*S25;
  AlM326_24 = C25*S26*(s->dpt[3][44]+s->dpt[3][48])-C26*(s->dpt[1][48]*C25-s->dpt[2][48]*S25);
  AlM126_25 = -s->dpt[2][48]*C26;
  OM127 = qd[27]+OM126;
  OM227 = OM226*C27+OM326*S27;
  OM327 = -(OM226*S27-OM326*C27);
  OpF227 = C27*(OpF225+qd[27]*OM326)+S27*(OpF326-qd[27]*OM226);
  OpF327 = C27*(OpF326-qd[27]*OM226)-S27*(OpF225+qd[27]*OM326);
  BS227 = OM127*OM227;
  BS327 = OM127*OM327;
  BS627 = OM227*OM327;
  BS927 = -(OM127*OM127+OM227*OM227);
  BeF327 = BS327+OpF227;
  BeF627 = BS627-OpF126;
  AlF127 = AlF126+BeF226*s->dpt[2][51];
  AlF227 = C27*(AlF226+BS526*s->dpt[2][51])+S27*(AlF326+BeF826*s->dpt[2][51]);
  AlF327 = C27*(AlF326+BeF826*s->dpt[2][51])-S27*(AlF226+BS526*s->dpt[2][51]);
  AlM227_1 = AlM225_1*C27+AlM326_1*S27;
  AlM327_1 = -(AlM225_1*S27-AlM326_1*C27);
  AlM227_2 = AlM225_2*C27+AlM326_2*S27;
  AlM327_2 = -(AlM225_2*S27-AlM326_2*C27);
  AlM227_3 = AlM225_3*C27+AlM326_3*S27;
  AlM327_3 = -(AlM225_3*S27-AlM326_3*C27);
  OpM227_4 = OpM225_4*C27+OpM326_4*S27;
  OpM327_4 = -(OpM225_4*S27-OpM326_4*C27);
  AlM127_4 = AlM126_4-OpM326_4*s->dpt[2][51];
  AlM227_4 = AlM226_4*C27+S27*(AlM326_4+OpM126_4*s->dpt[2][51]);
  AlM327_4 = -(AlM226_4*S27-C27*(AlM326_4+OpM126_4*s->dpt[2][51]));
  OpM227_5 = OpM225_5*C27+OpM326_5*S27;
  OpM327_5 = -(OpM225_5*S27-OpM326_5*C27);
  AlM127_5 = AlM126_5-OpM326_5*s->dpt[2][51];
  AlM227_5 = AlM226_5*C27+S27*(AlM326_5+OpM126_5*s->dpt[2][51]);
  AlM327_5 = -(AlM226_5*S27-C27*(AlM326_5+OpM126_5*s->dpt[2][51]));
  OpM227_6 = OpM225_6*C27+OpM326_6*S27;
  OpM327_6 = -(OpM225_6*S27-OpM326_6*C27);
  AlM127_6 = AlM126_6-OpM326_6*s->dpt[2][51];
  AlM227_6 = AlM226_6*C27+S27*(AlM326_6+OpM126_6*s->dpt[2][51]);
  AlM327_6 = -(AlM226_6*S27-C27*(AlM326_6+OpM126_6*s->dpt[2][51]));
  OpM227_23 = OpM225_23*C27+OpM326_23*S27;
  OpM327_23 = -(OpM225_23*S27-OpM326_23*C27);
  AlM127_23 = AlM126_23-OpM326_23*s->dpt[2][51];
  AlM227_23 = AlM226_23*C27+S27*(AlM326_23+OpM126_23*s->dpt[2][51]);
  AlM327_23 = -(AlM226_23*S27-C27*(AlM326_23+OpM126_23*s->dpt[2][51]));
  OpM227_24 = OpM326_24*S27+C25*C27;
  OpM327_24 = OpM326_24*C27-C25*S27;
  AlM127_24 = AlM126_24-OpM326_24*s->dpt[2][51];
  AlM227_24 = AlM226_24*C27+S27*(AlM326_24+OpM126_24*s->dpt[2][51]);
  AlM327_24 = -(AlM226_24*S27-C27*(AlM326_24+OpM126_24*s->dpt[2][51]));
  OpM227_25 = C26*S27;
  OpM327_25 = C26*C27;
  AlM127_25 = AlM126_25-s->dpt[2][51]*C26;
  AlM227_25 = s->dpt[1][48]*C27-S26*S27*(s->dpt[2][48]+s->dpt[2][51]);
  AlM327_25 = -(s->dpt[1][48]*S27+S26*C27*(s->dpt[2][48]+s->dpt[2][51]));
  OM128 = OM127*C28+OM227*S28;
  OM228 = -(OM127*S28-OM227*C28);
  OM328 = qd[28]+OM327;
  OpF128 = C28*(OpF126+qd[28]*OM227)+S28*(OpF227-qd[28]*OM127);
  OpF228 = C28*(OpF227-qd[28]*OM127)-S28*(OpF126+qd[28]*OM227);
  BS228 = OM128*OM228;
  BS328 = OM128*OM328;
  BS628 = OM228*OM328;
  BS928 = -(OM128*OM128+OM228*OM228);
  BeF328 = BS328+OpF228;
  BeF628 = BS628-OpF128;
  AlF128 = C28*(AlF127+BeF327*s->dpt[3][53])+S28*(AlF227+BeF627*s->dpt[3][53]);
  AlF228 = C28*(AlF227+BeF627*s->dpt[3][53])-S28*(AlF127+BeF327*s->dpt[3][53]);
  AlF328 = AlF327+BS927*s->dpt[3][53];
  AlM128_1 = AlM126_1*C28+AlM227_1*S28;
  AlM228_1 = -(AlM126_1*S28-AlM227_1*C28);
  AlM128_2 = AlM126_2*C28+AlM227_2*S28;
  AlM228_2 = -(AlM126_2*S28-AlM227_2*C28);
  AlM128_3 = AlM126_3*C28+AlM227_3*S28;
  AlM228_3 = -(AlM126_3*S28-AlM227_3*C28);
  OpM128_4 = OpM126_4*C28+OpM227_4*S28;
  OpM228_4 = -(OpM126_4*S28-OpM227_4*C28);
  AlM128_4 = C28*(AlM127_4+OpM227_4*s->dpt[3][53])+S28*(AlM227_4-OpM126_4*s->dpt[3][53]);
  AlM228_4 = C28*(AlM227_4-OpM126_4*s->dpt[3][53])-S28*(AlM127_4+OpM227_4*s->dpt[3][53]);
  OpM128_5 = OpM126_5*C28+OpM227_5*S28;
  OpM228_5 = -(OpM126_5*S28-OpM227_5*C28);
  AlM128_5 = C28*(AlM127_5+OpM227_5*s->dpt[3][53])+S28*(AlM227_5-OpM126_5*s->dpt[3][53]);
  AlM228_5 = C28*(AlM227_5-OpM126_5*s->dpt[3][53])-S28*(AlM127_5+OpM227_5*s->dpt[3][53]);
  OpM128_6 = OpM126_6*C28+OpM227_6*S28;
  OpM228_6 = -(OpM126_6*S28-OpM227_6*C28);
  AlM128_6 = C28*(AlM127_6+OpM227_6*s->dpt[3][53])+S28*(AlM227_6-OpM126_6*s->dpt[3][53]);
  AlM228_6 = C28*(AlM227_6-OpM126_6*s->dpt[3][53])-S28*(AlM127_6+OpM227_6*s->dpt[3][53]);
  OpM128_23 = OpM126_23*C28+OpM227_23*S28;
  OpM228_23 = -(OpM126_23*S28-OpM227_23*C28);
  AlM128_23 = C28*(AlM127_23+OpM227_23*s->dpt[3][53])+S28*(AlM227_23-OpM126_23*s->dpt[3][53]);
  AlM228_23 = C28*(AlM227_23-OpM126_23*s->dpt[3][53])-S28*(AlM127_23+OpM227_23*s->dpt[3][53]);
  OpM128_24 = OpM126_24*C28+OpM227_24*S28;
  OpM228_24 = -(OpM126_24*S28-OpM227_24*C28);
  AlM128_24 = C28*(AlM127_24+OpM227_24*s->dpt[3][53])+S28*(AlM227_24-OpM126_24*s->dpt[3][53]);
  AlM228_24 = C28*(AlM227_24-OpM126_24*s->dpt[3][53])-S28*(AlM127_24+OpM227_24*s->dpt[3][53]);
  OpM128_25 = OpM227_25*S28-S26*C28;
  OpM228_25 = OpM227_25*C28+S26*S28;
  AlM128_25 = C28*(AlM127_25+OpM227_25*s->dpt[3][53])+S28*(AlM227_25+s->dpt[3][53]*S26);
  AlM228_25 = C28*(AlM227_25+s->dpt[3][53]*S26)-S28*(AlM127_25+OpM227_25*s->dpt[3][53]);
  OpM128_26 = C27*S28;
  OpM228_26 = C27*C28;
  AlM128_26 = s->dpt[3][53]*C27*C28;
  AlM228_26 = -s->dpt[3][53]*C27*S28;
  AlM128_27 = -s->dpt[3][53]*S28;
  AlM228_27 = -s->dpt[3][53]*C28;
  OM129 = OM128*C29-OM328*S29;
  OM229 = qd[29]+OM228;
  OM329 = OM128*S29+OM328*C29;
  OpF129 = C29*(OpF128-qd[29]*OM328)-S29*(OpF327+qd[29]*OM128);
  OpF329 = C29*(OpF327+qd[29]*OM128)+S29*(OpF128-qd[29]*OM328);
  BS229 = OM129*OM229;
  BS329 = OM129*OM329;
  BS629 = OM229*OM329;
  OpM129_4 = OpM128_4*C29-OpM327_4*S29;
  OpM329_4 = OpM128_4*S29+OpM327_4*C29;
  OpM129_5 = OpM128_5*C29-OpM327_5*S29;
  OpM329_5 = OpM128_5*S29+OpM327_5*C29;
  OpM129_6 = OpM128_6*C29-OpM327_6*S29;
  OpM329_6 = OpM128_6*S29+OpM327_6*C29;
  OpM129_23 = OpM128_23*C29-OpM327_23*S29;
  OpM329_23 = OpM128_23*S29+OpM327_23*C29;
  OpM129_24 = OpM128_24*C29-OpM327_24*S29;
  OpM329_24 = OpM128_24*S29+OpM327_24*C29;
  OpM129_25 = OpM128_25*C29-OpM327_25*S29;
  OpM329_25 = OpM128_25*S29+OpM327_25*C29;
  OpM129_26 = OpM128_26*C29+S27*S29;
  OpM329_26 = OpM128_26*S29-S27*C29;
  OpM129_27 = C28*C29;
  OpM329_27 = C28*S29;

// = = Block_0_1_0_2_0_10 = = 
 
// Forward Kinematics 

  OM130 = OM125*C30-OM325*S30;
  OM230 = qd[30]+OM225;
  OM330 = OM125*S30+OM325*C30;
  OpF130 = C30*(OpF125-qd[30]*OM325)-S30*(OpF324+qd[30]*OM125);
  OpF330 = C30*(OpF324+qd[30]*OM125)+S30*(OpF125-qd[30]*OM325);
  BS230 = OM130*OM230;
  BS330 = OM130*OM330;
  BS530 = -(OM130*OM130+OM330*OM330);
  BS630 = OM230*OM330;
  BeF230 = BS230-OpF330;
  BeF830 = BS630+OpF130;
  AlF130 = C30*(AlF125+BS125*s->dpt[1][49]+BeF225*s->dpt[2][49]+BeF325*s->dpt[3][49])-S30*(AlF325+BS925*s->dpt[3][49]+
 BeF725*s->dpt[1][49]+BeF825*s->dpt[2][49]);
  AlF230 = AlF225+BS525*s->dpt[2][49]+BeF425*s->dpt[1][49]+BeF625*s->dpt[3][49];
  AlF330 = C30*(AlF325+BS925*s->dpt[3][49]+BeF725*s->dpt[1][49]+BeF825*s->dpt[2][49])+S30*(AlF125+BS125*s->dpt[1][49]+
 BeF225*s->dpt[2][49]+BeF325*s->dpt[3][49]);
  AlM130_1 = AlM125_1*C30-AlM324_1*S30;
  AlM330_1 = AlM125_1*S30+AlM324_1*C30;
  AlM130_2 = AlM125_2*C30-AlM324_2*S30;
  AlM330_2 = AlM125_2*S30+AlM324_2*C30;
  AlM130_3 = AlM125_3*C30-AlM324_3*S30;
  AlM330_3 = AlM125_3*S30+AlM324_3*C30;
  OpM130_4 = OpM125_4*C30-OpM324_4*S30;
  OpM330_4 = OpM125_4*S30+OpM324_4*C30;
  AlM130_4 = C30*(AlM125_4+OpM225_4*s->dpt[3][49]-OpM324_4*s->dpt[2][49])-S30*(AlM324_4+OpM125_4*s->dpt[2][49]-OpM225_4*
 s->dpt[1][49]);
  AlM230_4 = AlM225_4-OpM125_4*s->dpt[3][49]+OpM324_4*s->dpt[1][49];
  AlM330_4 = C30*(AlM324_4+OpM125_4*s->dpt[2][49]-OpM225_4*s->dpt[1][49])+S30*(AlM125_4+OpM225_4*s->dpt[3][49]-OpM324_4*
 s->dpt[2][49]);
  OpM130_5 = OpM125_5*C30-OpM324_5*S30;
  OpM330_5 = OpM125_5*S30+OpM324_5*C30;
  AlM130_5 = C30*(AlM125_5+OpM225_5*s->dpt[3][49]-OpM324_5*s->dpt[2][49])-S30*(AlM324_5+OpM125_5*s->dpt[2][49]-OpM225_5*
 s->dpt[1][49]);
  AlM230_5 = AlM225_5-OpM125_5*s->dpt[3][49]+OpM324_5*s->dpt[1][49];
  AlM330_5 = C30*(AlM324_5+OpM125_5*s->dpt[2][49]-OpM225_5*s->dpt[1][49])+S30*(AlM125_5+OpM225_5*s->dpt[3][49]-OpM324_5*
 s->dpt[2][49]);
  OpM130_6 = OpM125_6*C30-OpM324_6*S30;
  OpM330_6 = OpM125_6*S30+OpM324_6*C30;
  AlM130_6 = C30*(AlM125_6+OpM225_6*s->dpt[3][49]-OpM324_6*s->dpt[2][49])-S30*(AlM324_6+OpM125_6*s->dpt[2][49]-OpM225_6*
 s->dpt[1][49]);
  AlM230_6 = AlM225_6-OpM125_6*s->dpt[3][49]+OpM324_6*s->dpt[1][49];
  AlM330_6 = C30*(AlM324_6+OpM125_6*s->dpt[2][49]-OpM225_6*s->dpt[1][49])+S30*(AlM125_6+OpM225_6*s->dpt[3][49]-OpM324_6*
 s->dpt[2][49]);
  OpM130_23 = OpM125_23*C30-S24*S30;
  OpM330_23 = OpM125_23*S30+S24*C30;
  AlM130_23 = C30*(AlM125_23+OpM225_23*s->dpt[3][49]-s->dpt[2][49]*S24)-S30*(OpM125_23*s->dpt[2][49]-OpM225_23*
 s->dpt[1][49]);
  AlM230_23 = AlM225_23-OpM125_23*s->dpt[3][49]+s->dpt[1][49]*S24;
  AlM330_23 = C30*(OpM125_23*s->dpt[2][49]-OpM225_23*s->dpt[1][49])+S30*(AlM125_23+OpM225_23*s->dpt[3][49]-s->dpt[2][49]
 *S24);
  OpM130_24 = S25*C30;
  OpM330_24 = S25*S30;
  AlM130_24 = C25*C30*(s->dpt[3][44]+s->dpt[3][49])+S30*(s->dpt[1][49]*C25-s->dpt[2][49]*S25);
  AlM230_24 = AlM225_24-s->dpt[3][49]*S25;
  AlM330_24 = C25*S30*(s->dpt[3][44]+s->dpt[3][49])-C30*(s->dpt[1][49]*C25-s->dpt[2][49]*S25);
  AlM130_25 = -s->dpt[2][49]*C30;
  OM131 = qd[31]+OM130;
  OM231 = OM230*C31+OM330*S31;
  OM331 = -(OM230*S31-OM330*C31);
  OpF231 = C31*(OpF225+qd[31]*OM330)+S31*(OpF330-qd[31]*OM230);
  OpF331 = C31*(OpF330-qd[31]*OM230)-S31*(OpF225+qd[31]*OM330);
  BS231 = OM131*OM231;
  BS331 = OM131*OM331;
  BS631 = OM231*OM331;
  BS931 = -(OM131*OM131+OM231*OM231);
  BeF331 = BS331+OpF231;
  BeF631 = BS631-OpF130;
  AlF131 = AlF130+BeF230*s->dpt[2][60];
  AlF231 = C31*(AlF230+BS530*s->dpt[2][60])+S31*(AlF330+BeF830*s->dpt[2][60]);
  AlF331 = C31*(AlF330+BeF830*s->dpt[2][60])-S31*(AlF230+BS530*s->dpt[2][60]);
  AlM231_1 = AlM225_1*C31+AlM330_1*S31;
  AlM331_1 = -(AlM225_1*S31-AlM330_1*C31);
  AlM231_2 = AlM225_2*C31+AlM330_2*S31;
  AlM331_2 = -(AlM225_2*S31-AlM330_2*C31);
  AlM231_3 = AlM225_3*C31+AlM330_3*S31;
  AlM331_3 = -(AlM225_3*S31-AlM330_3*C31);
  OpM231_4 = OpM225_4*C31+OpM330_4*S31;
  OpM331_4 = -(OpM225_4*S31-OpM330_4*C31);
  AlM131_4 = AlM130_4-OpM330_4*s->dpt[2][60];
  AlM231_4 = AlM230_4*C31+S31*(AlM330_4+OpM130_4*s->dpt[2][60]);
  AlM331_4 = -(AlM230_4*S31-C31*(AlM330_4+OpM130_4*s->dpt[2][60]));
  OpM231_5 = OpM225_5*C31+OpM330_5*S31;
  OpM331_5 = -(OpM225_5*S31-OpM330_5*C31);
  AlM131_5 = AlM130_5-OpM330_5*s->dpt[2][60];
  AlM231_5 = AlM230_5*C31+S31*(AlM330_5+OpM130_5*s->dpt[2][60]);
  AlM331_5 = -(AlM230_5*S31-C31*(AlM330_5+OpM130_5*s->dpt[2][60]));
  OpM231_6 = OpM225_6*C31+OpM330_6*S31;
  OpM331_6 = -(OpM225_6*S31-OpM330_6*C31);
  AlM131_6 = AlM130_6-OpM330_6*s->dpt[2][60];
  AlM231_6 = AlM230_6*C31+S31*(AlM330_6+OpM130_6*s->dpt[2][60]);
  AlM331_6 = -(AlM230_6*S31-C31*(AlM330_6+OpM130_6*s->dpt[2][60]));
  OpM231_23 = OpM225_23*C31+OpM330_23*S31;
  OpM331_23 = -(OpM225_23*S31-OpM330_23*C31);
  AlM131_23 = AlM130_23-OpM330_23*s->dpt[2][60];
  AlM231_23 = AlM230_23*C31+S31*(AlM330_23+OpM130_23*s->dpt[2][60]);
  AlM331_23 = -(AlM230_23*S31-C31*(AlM330_23+OpM130_23*s->dpt[2][60]));
  OpM231_24 = OpM330_24*S31+C25*C31;
  OpM331_24 = OpM330_24*C31-C25*S31;
  AlM131_24 = AlM130_24-OpM330_24*s->dpt[2][60];
  AlM231_24 = AlM230_24*C31+S31*(AlM330_24+OpM130_24*s->dpt[2][60]);
  AlM331_24 = -(AlM230_24*S31-C31*(AlM330_24+OpM130_24*s->dpt[2][60]));
  OpM231_25 = C30*S31;
  OpM331_25 = C30*C31;
  AlM131_25 = AlM130_25-s->dpt[2][60]*C30;
  AlM231_25 = s->dpt[1][49]*C31-S30*S31*(s->dpt[2][49]+s->dpt[2][60]);
  AlM331_25 = -(s->dpt[1][49]*S31+S30*C31*(s->dpt[2][49]+s->dpt[2][60]));
  OM132 = OM131*C32+OM231*S32;
  OM232 = -(OM131*S32-OM231*C32);
  OM332 = qd[32]+OM331;
  OpF132 = C32*(OpF130+qd[32]*OM231)+S32*(OpF231-qd[32]*OM131);
  OpF232 = C32*(OpF231-qd[32]*OM131)-S32*(OpF130+qd[32]*OM231);
  BS232 = OM132*OM232;
  BS332 = OM132*OM332;
  BS632 = OM232*OM332;
  BS932 = -(OM132*OM132+OM232*OM232);
  BeF332 = BS332+OpF232;
  BeF632 = BS632-OpF132;
  AlF132 = C32*(AlF131+BeF331*s->dpt[3][62])+S32*(AlF231+BeF631*s->dpt[3][62]);
  AlF232 = C32*(AlF231+BeF631*s->dpt[3][62])-S32*(AlF131+BeF331*s->dpt[3][62]);
  AlF332 = AlF331+BS931*s->dpt[3][62];
  AlM132_1 = AlM130_1*C32+AlM231_1*S32;
  AlM232_1 = -(AlM130_1*S32-AlM231_1*C32);
  AlM132_2 = AlM130_2*C32+AlM231_2*S32;
  AlM232_2 = -(AlM130_2*S32-AlM231_2*C32);
  AlM132_3 = AlM130_3*C32+AlM231_3*S32;
  AlM232_3 = -(AlM130_3*S32-AlM231_3*C32);
  OpM132_4 = OpM130_4*C32+OpM231_4*S32;
  OpM232_4 = -(OpM130_4*S32-OpM231_4*C32);
  AlM132_4 = C32*(AlM131_4+OpM231_4*s->dpt[3][62])+S32*(AlM231_4-OpM130_4*s->dpt[3][62]);
  AlM232_4 = C32*(AlM231_4-OpM130_4*s->dpt[3][62])-S32*(AlM131_4+OpM231_4*s->dpt[3][62]);
  OpM132_5 = OpM130_5*C32+OpM231_5*S32;
  OpM232_5 = -(OpM130_5*S32-OpM231_5*C32);
  AlM132_5 = C32*(AlM131_5+OpM231_5*s->dpt[3][62])+S32*(AlM231_5-OpM130_5*s->dpt[3][62]);
  AlM232_5 = C32*(AlM231_5-OpM130_5*s->dpt[3][62])-S32*(AlM131_5+OpM231_5*s->dpt[3][62]);
  OpM132_6 = OpM130_6*C32+OpM231_6*S32;
  OpM232_6 = -(OpM130_6*S32-OpM231_6*C32);
  AlM132_6 = C32*(AlM131_6+OpM231_6*s->dpt[3][62])+S32*(AlM231_6-OpM130_6*s->dpt[3][62]);
  AlM232_6 = C32*(AlM231_6-OpM130_6*s->dpt[3][62])-S32*(AlM131_6+OpM231_6*s->dpt[3][62]);
  OpM132_23 = OpM130_23*C32+OpM231_23*S32;
  OpM232_23 = -(OpM130_23*S32-OpM231_23*C32);
  AlM132_23 = C32*(AlM131_23+OpM231_23*s->dpt[3][62])+S32*(AlM231_23-OpM130_23*s->dpt[3][62]);
  AlM232_23 = C32*(AlM231_23-OpM130_23*s->dpt[3][62])-S32*(AlM131_23+OpM231_23*s->dpt[3][62]);
  OpM132_24 = OpM130_24*C32+OpM231_24*S32;
  OpM232_24 = -(OpM130_24*S32-OpM231_24*C32);
  AlM132_24 = C32*(AlM131_24+OpM231_24*s->dpt[3][62])+S32*(AlM231_24-OpM130_24*s->dpt[3][62]);
  AlM232_24 = C32*(AlM231_24-OpM130_24*s->dpt[3][62])-S32*(AlM131_24+OpM231_24*s->dpt[3][62]);
  OpM132_25 = OpM231_25*S32-S30*C32;
  OpM232_25 = OpM231_25*C32+S30*S32;
  AlM132_25 = C32*(AlM131_25+OpM231_25*s->dpt[3][62])+S32*(AlM231_25+s->dpt[3][62]*S30);
  AlM232_25 = C32*(AlM231_25+s->dpt[3][62]*S30)-S32*(AlM131_25+OpM231_25*s->dpt[3][62]);
  OpM132_30 = C31*S32;
  OpM232_30 = C31*C32;
  AlM132_30 = s->dpt[3][62]*C31*C32;
  AlM232_30 = -s->dpt[3][62]*C31*S32;
  AlM132_31 = -s->dpt[3][62]*S32;
  AlM232_31 = -s->dpt[3][62]*C32;
  OM133 = OM132*C33-OM332*S33;
  OM233 = qd[33]+OM232;
  OM333 = OM132*S33+OM332*C33;
  OpF133 = C33*(OpF132-qd[33]*OM332)-S33*(OpF331+qd[33]*OM132);
  OpF333 = C33*(OpF331+qd[33]*OM132)+S33*(OpF132-qd[33]*OM332);
  BS233 = OM133*OM233;
  BS333 = OM133*OM333;
  BS633 = OM233*OM333;
  OpM133_4 = OpM132_4*C33-OpM331_4*S33;
  OpM333_4 = OpM132_4*S33+OpM331_4*C33;
  OpM133_5 = OpM132_5*C33-OpM331_5*S33;
  OpM333_5 = OpM132_5*S33+OpM331_5*C33;
  OpM133_6 = OpM132_6*C33-OpM331_6*S33;
  OpM333_6 = OpM132_6*S33+OpM331_6*C33;
  OpM133_23 = OpM132_23*C33-OpM331_23*S33;
  OpM333_23 = OpM132_23*S33+OpM331_23*C33;
  OpM133_24 = OpM132_24*C33-OpM331_24*S33;
  OpM333_24 = OpM132_24*S33+OpM331_24*C33;
  OpM133_25 = OpM132_25*C33-OpM331_25*S33;
  OpM333_25 = OpM132_25*S33+OpM331_25*C33;
  OpM133_30 = OpM132_30*C33+S31*S33;
  OpM333_30 = OpM132_30*S33-S31*C33;
  OpM133_31 = C32*C33;
  OpM333_31 = C32*S33;

// = = Block_0_2_0_1_0_3 = = 
 
// Backward Dynamics 

  FA113 = -(s->frc[1][13]-s->m[13]*(AlF112+(2.0)*qd[13]*OM212+BS112*s->dpt[1][20]+BeF312*Dz133));
  FA213 = -(s->frc[2][13]-s->m[13]*(AlF211-(2.0)*qd[13]*OM112+BeF412*s->dpt[1][20]+BeF612*Dz133));
  FA313 = -(s->frc[3][13]-s->m[13]*(AlF312+BS912*Dz133+BeF712*s->dpt[1][20]));
  FB113_1 = s->m[13]*AlM112_1;
  FB213_1 = s->m[13]*AlM211_1;
  FB313_1 = s->m[13]*AlM312_1;
  FB113_2 = s->m[13]*AlM112_2;
  FB213_2 = s->m[13]*AlM211_2;
  FB313_2 = s->m[13]*AlM312_2;
  FB113_3 = s->m[13]*AlM112_3;
  FB213_3 = s->m[13]*AlM211_3;
  FB313_3 = s->m[13]*AlM312_3;
  FB113_4 = s->m[13]*(AlM112_4+Dz133*OpM211_4);
  FB213_4 = s->m[13]*(AlM211_4-Dz133*OpM112_4+OpM312_4*s->dpt[1][20]);
  FB313_4 = s->m[13]*(AlM312_4-OpM211_4*s->dpt[1][20]);
  FB113_5 = s->m[13]*(AlM112_5+Dz133*OpM211_5);
  FB213_5 = s->m[13]*(AlM211_5-Dz133*OpM112_5+OpM312_5*s->dpt[1][20]);
  FB313_5 = s->m[13]*(AlM312_5-OpM211_5*s->dpt[1][20]);
  FB113_6 = s->m[13]*(AlM112_6+Dz133*OpM211_6);
  FB213_6 = s->m[13]*(AlM211_6-Dz133*OpM112_6+OpM312_6*s->dpt[1][20]);
  FB313_6 = s->m[13]*(AlM312_6-OpM211_6*s->dpt[1][20]);
  FB113_7 = s->m[13]*(AlM112_7+Dz133*OpM211_7);
  FB213_7 = s->m[13]*(AlM211_7-Dz133*OpM112_7+OpM312_7*s->dpt[1][20]);
  FB313_7 = s->m[13]*(AlM312_7-OpM211_7*s->dpt[1][20]);
  FB113_8 = s->m[13]*(AlM112_8+Dz133*OpM211_8);
  FB213_8 = s->m[13]*(AlM211_8-Dz133*OpM112_8+OpM312_8*s->dpt[1][20]);
  FB313_8 = s->m[13]*(AlM312_8-OpM211_8*s->dpt[1][20]);
  FB113_9 = s->m[13]*(AlM112_9+Dz133*OpM211_9);
  FB213_9 = s->m[13]*(AlM211_9-Dz133*OpM112_9+OpM312_9*s->dpt[1][20]);
  FB313_9 = s->m[13]*(AlM312_9-OpM211_9*s->dpt[1][20]);
  FB113_10 = s->m[13]*(AlM112_10+Dz133*C11);
  FB213_10 = -s->m[13]*(Dz133*OpM112_10-OpM312_10*s->dpt[1][20]);
  FB313_10 = s->m[13]*(AlM312_10-s->dpt[1][20]*C11);
  FB213_11 = s->m[13]*(s->dpt[1][20]*S12-C12*(q[13]+s->dpt[3][20]));
  FB313_12 = -s->m[13]*s->dpt[1][20];

// = = Block_0_2_0_1_0_4 = = 
 
// Backward Dynamics 

  FA114 = -(s->frc[1][14]-s->m[14]*(AlF112+(2.0)*qd[14]*OM212+BS112*s->dpt[1][21]+BeF312*Dz143));
  FA214 = -(s->frc[2][14]-s->m[14]*(AlF211-(2.0)*qd[14]*OM112+BeF412*s->dpt[1][21]+BeF612*Dz143));
  FA314 = -(s->frc[3][14]-s->m[14]*(AlF312+BS912*Dz143+BeF712*s->dpt[1][21]));
  FB114_1 = s->m[14]*AlM112_1;
  FB214_1 = s->m[14]*AlM211_1;
  FB314_1 = s->m[14]*AlM312_1;
  FB114_2 = s->m[14]*AlM112_2;
  FB214_2 = s->m[14]*AlM211_2;
  FB314_2 = s->m[14]*AlM312_2;
  FB114_3 = s->m[14]*AlM112_3;
  FB214_3 = s->m[14]*AlM211_3;
  FB314_3 = s->m[14]*AlM312_3;
  FB114_4 = s->m[14]*(AlM112_4+Dz143*OpM211_4);
  FB214_4 = s->m[14]*(AlM211_4-Dz143*OpM112_4+OpM312_4*s->dpt[1][21]);
  FB314_4 = s->m[14]*(AlM312_4-OpM211_4*s->dpt[1][21]);
  FB114_5 = s->m[14]*(AlM112_5+Dz143*OpM211_5);
  FB214_5 = s->m[14]*(AlM211_5-Dz143*OpM112_5+OpM312_5*s->dpt[1][21]);
  FB314_5 = s->m[14]*(AlM312_5-OpM211_5*s->dpt[1][21]);
  FB114_6 = s->m[14]*(AlM112_6+Dz143*OpM211_6);
  FB214_6 = s->m[14]*(AlM211_6-Dz143*OpM112_6+OpM312_6*s->dpt[1][21]);
  FB314_6 = s->m[14]*(AlM312_6-OpM211_6*s->dpt[1][21]);
  FB114_7 = s->m[14]*(AlM112_7+Dz143*OpM211_7);
  FB214_7 = s->m[14]*(AlM211_7-Dz143*OpM112_7+OpM312_7*s->dpt[1][21]);
  FB314_7 = s->m[14]*(AlM312_7-OpM211_7*s->dpt[1][21]);
  FB114_8 = s->m[14]*(AlM112_8+Dz143*OpM211_8);
  FB214_8 = s->m[14]*(AlM211_8-Dz143*OpM112_8+OpM312_8*s->dpt[1][21]);
  FB314_8 = s->m[14]*(AlM312_8-OpM211_8*s->dpt[1][21]);
  FB114_9 = s->m[14]*(AlM112_9+Dz143*OpM211_9);
  FB214_9 = s->m[14]*(AlM211_9-Dz143*OpM112_9+OpM312_9*s->dpt[1][21]);
  FB314_9 = s->m[14]*(AlM312_9-OpM211_9*s->dpt[1][21]);
  FB114_10 = s->m[14]*(AlM112_10+Dz143*C11);
  FB214_10 = -s->m[14]*(Dz143*OpM112_10-OpM312_10*s->dpt[1][21]);
  FB314_10 = s->m[14]*(AlM312_10-s->dpt[1][21]*C11);
  FB214_11 = s->m[14]*(s->dpt[1][21]*S12-C12*(q[14]+s->dpt[3][21]));
  FB314_12 = -s->m[14]*s->dpt[1][21];

// = = Block_0_2_0_1_0_6 = = 
 
// Backward Dynamics 

  FA121 = -(s->frc[1][21]-s->m[21]*(AlF120+(2.0)*qd[21]*OM220+BS120*s->dpt[1][38]+BeF320*Dz213));
  FA221 = -(s->frc[2][21]-s->m[21]*(AlF219-(2.0)*qd[21]*OM120+BeF420*s->dpt[1][38]+BeF620*Dz213));
  FA321 = -(s->frc[3][21]-s->m[21]*(AlF320+BS920*Dz213+BeF720*s->dpt[1][38]));
  FB121_1 = s->m[21]*AlM120_1;
  FB221_1 = s->m[21]*AlM219_1;
  FB321_1 = s->m[21]*AlM320_1;
  FB121_2 = s->m[21]*AlM120_2;
  FB221_2 = s->m[21]*AlM219_2;
  FB321_2 = s->m[21]*AlM320_2;
  FB121_3 = s->m[21]*AlM120_3;
  FB221_3 = s->m[21]*AlM219_3;
  FB321_3 = s->m[21]*AlM320_3;
  FB121_4 = s->m[21]*(AlM120_4+Dz213*OpM219_4);
  FB221_4 = s->m[21]*(AlM219_4-Dz213*OpM120_4+OpM320_4*s->dpt[1][38]);
  FB321_4 = s->m[21]*(AlM320_4-OpM219_4*s->dpt[1][38]);
  FB121_5 = s->m[21]*(AlM120_5+Dz213*OpM219_5);
  FB221_5 = s->m[21]*(AlM219_5-Dz213*OpM120_5+OpM320_5*s->dpt[1][38]);
  FB321_5 = s->m[21]*(AlM320_5-OpM219_5*s->dpt[1][38]);
  FB121_6 = s->m[21]*(AlM120_6+Dz213*OpM219_6);
  FB221_6 = s->m[21]*(AlM219_6-Dz213*OpM120_6+OpM320_6*s->dpt[1][38]);
  FB321_6 = s->m[21]*(AlM320_6-OpM219_6*s->dpt[1][38]);
  FB121_15 = s->m[21]*(AlM120_15+Dz213*OpM219_15);
  FB221_15 = s->m[21]*(AlM219_15-Dz213*OpM120_15+OpM320_15*s->dpt[1][38]);
  FB321_15 = s->m[21]*(AlM320_15-OpM219_15*s->dpt[1][38]);
  FB121_16 = s->m[21]*(AlM120_16+Dz213*OpM219_16);
  FB221_16 = s->m[21]*(AlM219_16-Dz213*OpM120_16+OpM320_16*s->dpt[1][38]);
  FB321_16 = s->m[21]*(AlM320_16-OpM219_16*s->dpt[1][38]);
  FB121_17 = s->m[21]*(AlM120_17+Dz213*OpM219_17);
  FB221_17 = s->m[21]*(AlM219_17-Dz213*OpM120_17+OpM320_17*s->dpt[1][38]);
  FB321_17 = s->m[21]*(AlM320_17-OpM219_17*s->dpt[1][38]);
  FB121_18 = s->m[21]*(AlM120_18+Dz213*C19);
  FB221_18 = -s->m[21]*(Dz213*OpM120_18-OpM320_18*s->dpt[1][38]);
  FB321_18 = s->m[21]*(AlM320_18-s->dpt[1][38]*C19);
  FB221_19 = s->m[21]*(s->dpt[1][38]*S20-C20*(q[21]+s->dpt[3][38]));
  FB321_20 = -s->m[21]*s->dpt[1][38];

// = = Block_0_2_0_1_0_7 = = 
 
// Backward Dynamics 

  FA122 = -(s->frc[1][22]-s->m[22]*(AlF120+(2.0)*qd[22]*OM220+BS120*s->dpt[1][39]+BeF320*Dz223));
  FA222 = -(s->frc[2][22]-s->m[22]*(AlF219-(2.0)*qd[22]*OM120+BeF420*s->dpt[1][39]+BeF620*Dz223));
  FA322 = -(s->frc[3][22]-s->m[22]*(AlF320+BS920*Dz223+BeF720*s->dpt[1][39]));
  FB122_1 = s->m[22]*AlM120_1;
  FB222_1 = s->m[22]*AlM219_1;
  FB322_1 = s->m[22]*AlM320_1;
  FB122_2 = s->m[22]*AlM120_2;
  FB222_2 = s->m[22]*AlM219_2;
  FB322_2 = s->m[22]*AlM320_2;
  FB122_3 = s->m[22]*AlM120_3;
  FB222_3 = s->m[22]*AlM219_3;
  FB322_3 = s->m[22]*AlM320_3;
  FB122_4 = s->m[22]*(AlM120_4+Dz223*OpM219_4);
  FB222_4 = s->m[22]*(AlM219_4-Dz223*OpM120_4+OpM320_4*s->dpt[1][39]);
  FB322_4 = s->m[22]*(AlM320_4-OpM219_4*s->dpt[1][39]);
  FB122_5 = s->m[22]*(AlM120_5+Dz223*OpM219_5);
  FB222_5 = s->m[22]*(AlM219_5-Dz223*OpM120_5+OpM320_5*s->dpt[1][39]);
  FB322_5 = s->m[22]*(AlM320_5-OpM219_5*s->dpt[1][39]);
  FB122_6 = s->m[22]*(AlM120_6+Dz223*OpM219_6);
  FB222_6 = s->m[22]*(AlM219_6-Dz223*OpM120_6+OpM320_6*s->dpt[1][39]);
  FB322_6 = s->m[22]*(AlM320_6-OpM219_6*s->dpt[1][39]);
  FB122_15 = s->m[22]*(AlM120_15+Dz223*OpM219_15);
  FB222_15 = s->m[22]*(AlM219_15-Dz223*OpM120_15+OpM320_15*s->dpt[1][39]);
  FB322_15 = s->m[22]*(AlM320_15-OpM219_15*s->dpt[1][39]);
  FB122_16 = s->m[22]*(AlM120_16+Dz223*OpM219_16);
  FB222_16 = s->m[22]*(AlM219_16-Dz223*OpM120_16+OpM320_16*s->dpt[1][39]);
  FB322_16 = s->m[22]*(AlM320_16-OpM219_16*s->dpt[1][39]);
  FB122_17 = s->m[22]*(AlM120_17+Dz223*OpM219_17);
  FB222_17 = s->m[22]*(AlM219_17-Dz223*OpM120_17+OpM320_17*s->dpt[1][39]);
  FB322_17 = s->m[22]*(AlM320_17-OpM219_17*s->dpt[1][39]);
  FB122_18 = s->m[22]*(AlM120_18+Dz223*C19);
  FB222_18 = -s->m[22]*(Dz223*OpM120_18-OpM320_18*s->dpt[1][39]);
  FB322_18 = s->m[22]*(AlM320_18-s->dpt[1][39]*C19);
  FB222_19 = s->m[22]*(s->dpt[1][39]*S20-C20*(q[22]+s->dpt[3][39]));
  FB322_20 = -s->m[22]*s->dpt[1][39];

// = = Block_0_2_0_1_0_9 = = 
 
// Backward Dynamics 

  FA129 = -(s->frc[1][29]+s->m[29]*(s->l[1][29]*(OM229*OM229+OM329*OM329)-s->l[2][29]*(BS229-OpF329)-s->l[3][29]*(BS329+
 OpF228)-C29*(AlF128+BeF328*s->dpt[3][56])+S29*(AlF328+BS928*s->dpt[3][56])));
  FA229 = -(s->frc[2][29]-s->m[29]*(AlF228+BeF628*s->dpt[3][56]+s->l[1][29]*(BS229+OpF329)-s->l[2][29]*(OM129*OM129+
 OM329*OM329)+s->l[3][29]*(BS629-OpF129)));
  FA329 = -(s->frc[3][29]-s->m[29]*(s->l[1][29]*(BS329-OpF228)+s->l[2][29]*(BS629+OpF129)-s->l[3][29]*(OM129*OM129+OM229
 *OM229)+C29*(AlF328+BS928*s->dpt[3][56])+S29*(AlF128+BeF328*s->dpt[3][56])));
  CF129 = -(s->trq[1][29]-s->In[1][29]*OpF129+FA229*s->l[3][29]-FA329*s->l[2][29]+OM229*OM329*(s->In[5][29]-s->In[9][29]
 ));
  CF229 = -(s->trq[2][29]-s->In[5][29]*OpF228-FA129*s->l[3][29]+FA329*s->l[1][29]-OM129*OM329*(s->In[1][29]-s->In[9][29]
 ));
  CF329 = -(s->trq[3][29]-s->In[9][29]*OpF329+FA129*s->l[2][29]-FA229*s->l[1][29]+OM129*OM229*(s->In[1][29]-s->In[5][29]
 ));
  FB129_1 = s->m[29]*(AlM128_1*C29-AlM327_1*S29);
  FB229_1 = s->m[29]*AlM228_1;
  FB329_1 = s->m[29]*(AlM128_1*S29+AlM327_1*C29);
  CM129_1 = -(FB229_1*s->l[3][29]-FB329_1*s->l[2][29]);
  CM229_1 = FB129_1*s->l[3][29]-FB329_1*s->l[1][29];
  CM329_1 = -(FB129_1*s->l[2][29]-FB229_1*s->l[1][29]);
  FB129_2 = s->m[29]*(AlM128_2*C29-AlM327_2*S29);
  FB229_2 = s->m[29]*AlM228_2;
  FB329_2 = s->m[29]*(AlM128_2*S29+AlM327_2*C29);
  CM129_2 = -(FB229_2*s->l[3][29]-FB329_2*s->l[2][29]);
  CM229_2 = FB129_2*s->l[3][29]-FB329_2*s->l[1][29];
  CM329_2 = -(FB129_2*s->l[2][29]-FB229_2*s->l[1][29]);
  FB129_3 = s->m[29]*(AlM128_3*C29-AlM327_3*S29);
  FB229_3 = s->m[29]*AlM228_3;
  FB329_3 = s->m[29]*(AlM128_3*S29+AlM327_3*C29);
  CM129_3 = -(FB229_3*s->l[3][29]-FB329_3*s->l[2][29]);
  CM229_3 = FB129_3*s->l[3][29]-FB329_3*s->l[1][29];
  CM329_3 = -(FB129_3*s->l[2][29]-FB229_3*s->l[1][29]);
  FB129_4 = -s->m[29]*(AlM327_4*S29-OpM228_4*s->l[3][29]+OpM329_4*s->l[2][29]-C29*(AlM128_4+OpM228_4*s->dpt[3][56]));
  FB229_4 = s->m[29]*(AlM228_4-OpM128_4*s->dpt[3][56]-OpM129_4*s->l[3][29]+OpM329_4*s->l[1][29]);
  FB329_4 = s->m[29]*(AlM327_4*C29+OpM129_4*s->l[2][29]-OpM228_4*s->l[1][29]+S29*(AlM128_4+OpM228_4*s->dpt[3][56]));
  CM129_4 = s->In[1][29]*OpM129_4-FB229_4*s->l[3][29]+FB329_4*s->l[2][29];
  CM229_4 = s->In[5][29]*OpM228_4+FB129_4*s->l[3][29]-FB329_4*s->l[1][29];
  CM329_4 = s->In[9][29]*OpM329_4-FB129_4*s->l[2][29]+FB229_4*s->l[1][29];
  FB129_5 = -s->m[29]*(AlM327_5*S29-OpM228_5*s->l[3][29]+OpM329_5*s->l[2][29]-C29*(AlM128_5+OpM228_5*s->dpt[3][56]));
  FB229_5 = s->m[29]*(AlM228_5-OpM128_5*s->dpt[3][56]-OpM129_5*s->l[3][29]+OpM329_5*s->l[1][29]);
  FB329_5 = s->m[29]*(AlM327_5*C29+OpM129_5*s->l[2][29]-OpM228_5*s->l[1][29]+S29*(AlM128_5+OpM228_5*s->dpt[3][56]));
  CM129_5 = s->In[1][29]*OpM129_5-FB229_5*s->l[3][29]+FB329_5*s->l[2][29];
  CM229_5 = s->In[5][29]*OpM228_5+FB129_5*s->l[3][29]-FB329_5*s->l[1][29];
  CM329_5 = s->In[9][29]*OpM329_5-FB129_5*s->l[2][29]+FB229_5*s->l[1][29];
  FB129_6 = -s->m[29]*(AlM327_6*S29-OpM228_6*s->l[3][29]+OpM329_6*s->l[2][29]-C29*(AlM128_6+OpM228_6*s->dpt[3][56]));
  FB229_6 = s->m[29]*(AlM228_6-OpM128_6*s->dpt[3][56]-OpM129_6*s->l[3][29]+OpM329_6*s->l[1][29]);
  FB329_6 = s->m[29]*(AlM327_6*C29+OpM129_6*s->l[2][29]-OpM228_6*s->l[1][29]+S29*(AlM128_6+OpM228_6*s->dpt[3][56]));
  CM129_6 = s->In[1][29]*OpM129_6-FB229_6*s->l[3][29]+FB329_6*s->l[2][29];
  CM229_6 = s->In[5][29]*OpM228_6+FB129_6*s->l[3][29]-FB329_6*s->l[1][29];
  CM329_6 = s->In[9][29]*OpM329_6-FB129_6*s->l[2][29]+FB229_6*s->l[1][29];
  FB129_23 = -s->m[29]*(AlM327_23*S29-OpM228_23*s->l[3][29]+OpM329_23*s->l[2][29]-C29*(AlM128_23+OpM228_23*s->dpt[3][56]
 ));
  FB229_23 = s->m[29]*(AlM228_23-OpM128_23*s->dpt[3][56]-OpM129_23*s->l[3][29]+OpM329_23*s->l[1][29]);
  FB329_23 = s->m[29]*(AlM327_23*C29+OpM129_23*s->l[2][29]-OpM228_23*s->l[1][29]+S29*(AlM128_23+OpM228_23*s->dpt[3][56])
 );
  CM129_23 = s->In[1][29]*OpM129_23-FB229_23*s->l[3][29]+FB329_23*s->l[2][29];
  CM229_23 = s->In[5][29]*OpM228_23+FB129_23*s->l[3][29]-FB329_23*s->l[1][29];
  CM329_23 = s->In[9][29]*OpM329_23-FB129_23*s->l[2][29]+FB229_23*s->l[1][29];
  FB129_24 = -s->m[29]*(AlM327_24*S29-OpM228_24*s->l[3][29]+OpM329_24*s->l[2][29]-C29*(AlM128_24+OpM228_24*s->dpt[3][56]
 ));
  FB229_24 = s->m[29]*(AlM228_24-OpM128_24*s->dpt[3][56]-OpM129_24*s->l[3][29]+OpM329_24*s->l[1][29]);
  FB329_24 = s->m[29]*(AlM327_24*C29+OpM129_24*s->l[2][29]-OpM228_24*s->l[1][29]+S29*(AlM128_24+OpM228_24*s->dpt[3][56])
 );
  CM129_24 = s->In[1][29]*OpM129_24-FB229_24*s->l[3][29]+FB329_24*s->l[2][29];
  CM229_24 = s->In[5][29]*OpM228_24+FB129_24*s->l[3][29]-FB329_24*s->l[1][29];
  CM329_24 = s->In[9][29]*OpM329_24-FB129_24*s->l[2][29]+FB229_24*s->l[1][29];
  FB129_25 = -s->m[29]*(AlM327_25*S29-OpM228_25*s->l[3][29]+OpM329_25*s->l[2][29]-C29*(AlM128_25+OpM228_25*s->dpt[3][56]
 ));
  FB229_25 = s->m[29]*(AlM228_25-OpM128_25*s->dpt[3][56]-OpM129_25*s->l[3][29]+OpM329_25*s->l[1][29]);
  FB329_25 = s->m[29]*(AlM327_25*C29+OpM129_25*s->l[2][29]-OpM228_25*s->l[1][29]+S29*(AlM128_25+OpM228_25*s->dpt[3][56])
 );
  CM129_25 = s->In[1][29]*OpM129_25-FB229_25*s->l[3][29]+FB329_25*s->l[2][29];
  CM229_25 = s->In[5][29]*OpM228_25+FB129_25*s->l[3][29]-FB329_25*s->l[1][29];
  CM329_25 = s->In[9][29]*OpM329_25-FB129_25*s->l[2][29]+FB229_25*s->l[1][29];
  FB129_26 = s->m[29]*(OpM228_26*s->l[3][29]-OpM329_26*s->l[2][29]+C29*(AlM128_26+OpM228_26*s->dpt[3][56]));
  FB229_26 = s->m[29]*(AlM228_26-OpM128_26*s->dpt[3][56]-OpM129_26*s->l[3][29]+OpM329_26*s->l[1][29]);
  FB329_26 = s->m[29]*(OpM129_26*s->l[2][29]-OpM228_26*s->l[1][29]+S29*(AlM128_26+OpM228_26*s->dpt[3][56]));
  CM129_26 = s->In[1][29]*OpM129_26-FB229_26*s->l[3][29]+FB329_26*s->l[2][29];
  CM229_26 = s->In[5][29]*OpM228_26+FB129_26*s->l[3][29]-FB329_26*s->l[1][29];
  CM329_26 = s->In[9][29]*OpM329_26-FB129_26*s->l[2][29]+FB229_26*s->l[1][29];
  FB129_27 = s->m[29]*(C29*(AlM128_27-s->dpt[3][56]*S28)-OpM329_27*s->l[2][29]-s->l[3][29]*S28);
  FB229_27 = s->m[29]*(AlM228_27-OpM129_27*s->l[3][29]+OpM329_27*s->l[1][29]-s->dpt[3][56]*C28);
  FB329_27 = s->m[29]*(OpM129_27*s->l[2][29]+s->l[1][29]*S28+S29*(AlM128_27-s->dpt[3][56]*S28));
  CM129_27 = s->In[1][29]*OpM129_27-FB229_27*s->l[3][29]+FB329_27*s->l[2][29];
  CM229_27 = -(s->In[5][29]*S28-FB129_27*s->l[3][29]+FB329_27*s->l[1][29]);
  CM329_27 = s->In[9][29]*OpM329_27-FB129_27*s->l[2][29]+FB229_27*s->l[1][29];
  FB129_28 = -s->m[29]*s->l[2][29]*C29;
  FB229_28 = s->m[29]*(s->l[1][29]*C29+s->l[3][29]*S29);
  FB329_28 = -s->m[29]*s->l[2][29]*S29;
  CM229_28 = FB129_28*s->l[3][29]-FB329_28*s->l[1][29];
  CM229_29 = s->In[5][29]+s->m[29]*s->l[1][29]*s->l[1][29]+s->m[29]*s->l[3][29]*s->l[3][29];
  FA128 = -(s->frc[1][28]-s->m[28]*(AlF128+BeF328*s->l[3][28]-s->l[1][28]*(OM228*OM228+OM328*OM328)+s->l[2][28]*(BS228-
 OpF327)));
  FA228 = -(s->frc[2][28]-s->m[28]*(AlF228+BeF628*s->l[3][28]+s->l[1][28]*(BS228+OpF327)-s->l[2][28]*(OM128*OM128+OM328*
 OM328)));
  FA328 = -(s->frc[3][28]-s->m[28]*(AlF328+BS928*s->l[3][28]+s->l[1][28]*(BS328-OpF228)+s->l[2][28]*(BS628+OpF128)));
  FF128 = FA128+FA129*C29+FA329*S29;
  FF228 = FA228+FA229;
  CF128 = -(s->trq[1][28]-s->In[1][28]*OpF128-s->In[2][28]*OpF228-s->In[3][28]*OpF327-CF129*C29-CF329*S29+FA228*
 s->l[3][28]+FA229*s->dpt[3][56]-FA328*s->l[2][28]-OM228*(s->In[3][28]*OM128+s->In[9][28]*OM328)+OM328*(s->In[2][28]*OM128+
 s->In[5][28]*OM228));
  CF228 = -(s->trq[2][28]-CF229-s->In[2][28]*OpF128-s->In[5][28]*OpF228-FA128*s->l[3][28]+FA328*s->l[1][28]+OM128*(
 s->In[3][28]*OM128+s->In[9][28]*OM328)-OM328*(s->In[1][28]*OM128+s->In[2][28]*OM228+s->In[3][28]*OM328)-s->dpt[3][56]*(FA129
 *C29+FA329*S29));
  CF328 = -(s->trq[3][28]-s->In[3][28]*OpF128-s->In[9][28]*OpF327+CF129*S29-CF329*C29+FA128*s->l[2][28]-FA228*
 s->l[1][28]-OM128*(s->In[2][28]*OM128+s->In[5][28]*OM228)+OM228*(s->In[1][28]*OM128+s->In[2][28]*OM228+s->In[3][28]*OM328));
  FB128_1 = s->m[28]*AlM128_1;
  FB228_1 = s->m[28]*AlM228_1;
  FB328_1 = s->m[28]*AlM327_1;
  FM128_1 = FB128_1+FB129_1*C29+FB329_1*S29;
  FM228_1 = FB228_1+FB229_1;
  CM128_1 = CM129_1*C29+CM329_1*S29-FB228_1*s->l[3][28]-FB229_1*s->dpt[3][56]+FB328_1*s->l[2][28];
  CM228_1 = CM229_1+FB128_1*s->l[3][28]-FB328_1*s->l[1][28]+s->dpt[3][56]*(FB129_1*C29+FB329_1*S29);
  CM328_1 = -(CM129_1*S29-CM329_1*C29+FB128_1*s->l[2][28]-FB228_1*s->l[1][28]);
  FB128_2 = s->m[28]*AlM128_2;
  FB228_2 = s->m[28]*AlM228_2;
  FB328_2 = s->m[28]*AlM327_2;
  FM128_2 = FB128_2+FB129_2*C29+FB329_2*S29;
  FM228_2 = FB228_2+FB229_2;
  CM128_2 = CM129_2*C29+CM329_2*S29-FB228_2*s->l[3][28]-FB229_2*s->dpt[3][56]+FB328_2*s->l[2][28];
  CM228_2 = CM229_2+FB128_2*s->l[3][28]-FB328_2*s->l[1][28]+s->dpt[3][56]*(FB129_2*C29+FB329_2*S29);
  CM328_2 = -(CM129_2*S29-CM329_2*C29+FB128_2*s->l[2][28]-FB228_2*s->l[1][28]);
  FB128_3 = s->m[28]*AlM128_3;
  FB228_3 = s->m[28]*AlM228_3;
  FB328_3 = s->m[28]*AlM327_3;
  FM128_3 = FB128_3+FB129_3*C29+FB329_3*S29;
  FM228_3 = FB228_3+FB229_3;
  CM128_3 = CM129_3*C29+CM329_3*S29-FB228_3*s->l[3][28]-FB229_3*s->dpt[3][56]+FB328_3*s->l[2][28];
  CM228_3 = CM229_3+FB128_3*s->l[3][28]-FB328_3*s->l[1][28]+s->dpt[3][56]*(FB129_3*C29+FB329_3*S29);
  CM328_3 = -(CM129_3*S29-CM329_3*C29+FB128_3*s->l[2][28]-FB228_3*s->l[1][28]);
  FB128_4 = s->m[28]*(AlM128_4+OpM228_4*s->l[3][28]-OpM327_4*s->l[2][28]);
  FB228_4 = s->m[28]*(AlM228_4-OpM128_4*s->l[3][28]+OpM327_4*s->l[1][28]);
  FB328_4 = s->m[28]*(AlM327_4+OpM128_4*s->l[2][28]-OpM228_4*s->l[1][28]);
  FM128_4 = FB128_4+FB129_4*C29+FB329_4*S29;
  FM228_4 = FB228_4+FB229_4;
  CM128_4 = s->In[1][28]*OpM128_4+s->In[2][28]*OpM228_4+s->In[3][28]*OpM327_4+CM129_4*C29+CM329_4*S29-FB228_4*
 s->l[3][28]-FB229_4*s->dpt[3][56]+FB328_4*s->l[2][28];
  CM228_4 = CM229_4+s->In[2][28]*OpM128_4+s->In[5][28]*OpM228_4+FB128_4*s->l[3][28]-FB328_4*s->l[1][28]+s->dpt[3][56]*(
 FB129_4*C29+FB329_4*S29);
  CM328_4 = s->In[3][28]*OpM128_4+s->In[9][28]*OpM327_4-CM129_4*S29+CM329_4*C29-FB128_4*s->l[2][28]+FB228_4*s->l[1][28];
  FB128_5 = s->m[28]*(AlM128_5+OpM228_5*s->l[3][28]-OpM327_5*s->l[2][28]);
  FB228_5 = s->m[28]*(AlM228_5-OpM128_5*s->l[3][28]+OpM327_5*s->l[1][28]);
  FB328_5 = s->m[28]*(AlM327_5+OpM128_5*s->l[2][28]-OpM228_5*s->l[1][28]);
  FM128_5 = FB128_5+FB129_5*C29+FB329_5*S29;
  FM228_5 = FB228_5+FB229_5;
  CM128_5 = s->In[1][28]*OpM128_5+s->In[2][28]*OpM228_5+s->In[3][28]*OpM327_5+CM129_5*C29+CM329_5*S29-FB228_5*
 s->l[3][28]-FB229_5*s->dpt[3][56]+FB328_5*s->l[2][28];
  CM228_5 = CM229_5+s->In[2][28]*OpM128_5+s->In[5][28]*OpM228_5+FB128_5*s->l[3][28]-FB328_5*s->l[1][28]+s->dpt[3][56]*(
 FB129_5*C29+FB329_5*S29);
  CM328_5 = s->In[3][28]*OpM128_5+s->In[9][28]*OpM327_5-CM129_5*S29+CM329_5*C29-FB128_5*s->l[2][28]+FB228_5*s->l[1][28];
  FB128_6 = s->m[28]*(AlM128_6+OpM228_6*s->l[3][28]-OpM327_6*s->l[2][28]);
  FB228_6 = s->m[28]*(AlM228_6-OpM128_6*s->l[3][28]+OpM327_6*s->l[1][28]);
  FB328_6 = s->m[28]*(AlM327_6+OpM128_6*s->l[2][28]-OpM228_6*s->l[1][28]);
  FM128_6 = FB128_6+FB129_6*C29+FB329_6*S29;
  FM228_6 = FB228_6+FB229_6;
  CM128_6 = s->In[1][28]*OpM128_6+s->In[2][28]*OpM228_6+s->In[3][28]*OpM327_6+CM129_6*C29+CM329_6*S29-FB228_6*
 s->l[3][28]-FB229_6*s->dpt[3][56]+FB328_6*s->l[2][28];
  CM228_6 = CM229_6+s->In[2][28]*OpM128_6+s->In[5][28]*OpM228_6+FB128_6*s->l[3][28]-FB328_6*s->l[1][28]+s->dpt[3][56]*(
 FB129_6*C29+FB329_6*S29);
  CM328_6 = s->In[3][28]*OpM128_6+s->In[9][28]*OpM327_6-CM129_6*S29+CM329_6*C29-FB128_6*s->l[2][28]+FB228_6*s->l[1][28];
  FB128_23 = s->m[28]*(AlM128_23+OpM228_23*s->l[3][28]-OpM327_23*s->l[2][28]);
  FB228_23 = s->m[28]*(AlM228_23-OpM128_23*s->l[3][28]+OpM327_23*s->l[1][28]);
  FB328_23 = s->m[28]*(AlM327_23+OpM128_23*s->l[2][28]-OpM228_23*s->l[1][28]);
  FM128_23 = FB128_23+FB129_23*C29+FB329_23*S29;
  FM228_23 = FB228_23+FB229_23;
  CM128_23 = s->In[1][28]*OpM128_23+s->In[2][28]*OpM228_23+s->In[3][28]*OpM327_23+CM129_23*C29+CM329_23*S29-FB228_23*
 s->l[3][28]-FB229_23*s->dpt[3][56]+FB328_23*s->l[2][28];
  CM228_23 = CM229_23+s->In[2][28]*OpM128_23+s->In[5][28]*OpM228_23+FB128_23*s->l[3][28]-FB328_23*s->l[1][28]+
 s->dpt[3][56]*(FB129_23*C29+FB329_23*S29);
  CM328_23 = s->In[3][28]*OpM128_23+s->In[9][28]*OpM327_23-CM129_23*S29+CM329_23*C29-FB128_23*s->l[2][28]+FB228_23*
 s->l[1][28];
  FB128_24 = s->m[28]*(AlM128_24+OpM228_24*s->l[3][28]-OpM327_24*s->l[2][28]);
  FB228_24 = s->m[28]*(AlM228_24-OpM128_24*s->l[3][28]+OpM327_24*s->l[1][28]);
  FB328_24 = s->m[28]*(AlM327_24+OpM128_24*s->l[2][28]-OpM228_24*s->l[1][28]);
  FM128_24 = FB128_24+FB129_24*C29+FB329_24*S29;
  FM228_24 = FB228_24+FB229_24;
  CM128_24 = s->In[1][28]*OpM128_24+s->In[2][28]*OpM228_24+s->In[3][28]*OpM327_24+CM129_24*C29+CM329_24*S29-FB228_24*
 s->l[3][28]-FB229_24*s->dpt[3][56]+FB328_24*s->l[2][28];
  CM228_24 = CM229_24+s->In[2][28]*OpM128_24+s->In[5][28]*OpM228_24+FB128_24*s->l[3][28]-FB328_24*s->l[1][28]+
 s->dpt[3][56]*(FB129_24*C29+FB329_24*S29);
  CM328_24 = s->In[3][28]*OpM128_24+s->In[9][28]*OpM327_24-CM129_24*S29+CM329_24*C29-FB128_24*s->l[2][28]+FB228_24*
 s->l[1][28];
  FB128_25 = s->m[28]*(AlM128_25+OpM228_25*s->l[3][28]-OpM327_25*s->l[2][28]);
  FB228_25 = s->m[28]*(AlM228_25-OpM128_25*s->l[3][28]+OpM327_25*s->l[1][28]);
  FB328_25 = s->m[28]*(AlM327_25+OpM128_25*s->l[2][28]-OpM228_25*s->l[1][28]);
  FM128_25 = FB128_25+FB129_25*C29+FB329_25*S29;
  FM228_25 = FB228_25+FB229_25;
  CM128_25 = s->In[1][28]*OpM128_25+s->In[2][28]*OpM228_25+s->In[3][28]*OpM327_25+CM129_25*C29+CM329_25*S29-FB228_25*
 s->l[3][28]-FB229_25*s->dpt[3][56]+FB328_25*s->l[2][28];
  CM228_25 = CM229_25+s->In[2][28]*OpM128_25+s->In[5][28]*OpM228_25+FB128_25*s->l[3][28]-FB328_25*s->l[1][28]+
 s->dpt[3][56]*(FB129_25*C29+FB329_25*S29);
  CM328_25 = s->In[3][28]*OpM128_25+s->In[9][28]*OpM327_25-CM129_25*S29+CM329_25*C29-FB128_25*s->l[2][28]+FB228_25*
 s->l[1][28];
  FB128_26 = s->m[28]*(AlM128_26+OpM228_26*s->l[3][28]+s->l[2][28]*S27);
  FB228_26 = s->m[28]*(AlM228_26-OpM128_26*s->l[3][28]-s->l[1][28]*S27);
  FB328_26 = s->m[28]*(OpM128_26*s->l[2][28]-OpM228_26*s->l[1][28]);
  FM128_26 = FB128_26+FB129_26*C29+FB329_26*S29;
  FM228_26 = FB228_26+FB229_26;
  CM128_26 = s->In[1][28]*OpM128_26+s->In[2][28]*OpM228_26-s->In[3][28]*S27+CM129_26*C29+CM329_26*S29-FB228_26*
 s->l[3][28]-FB229_26*s->dpt[3][56]+FB328_26*s->l[2][28];
  CM228_26 = CM229_26+s->In[2][28]*OpM128_26+s->In[5][28]*OpM228_26+FB128_26*s->l[3][28]-FB328_26*s->l[1][28]+
 s->dpt[3][56]*(FB129_26*C29+FB329_26*S29);
  CM328_26 = s->In[3][28]*OpM128_26-s->In[9][28]*S27-CM129_26*S29+CM329_26*C29-FB128_26*s->l[2][28]+FB228_26*s->l[1][28];
  FB128_27 = s->m[28]*(AlM128_27-s->l[3][28]*S28);
  FB228_27 = s->m[28]*(AlM228_27-s->l[3][28]*C28);
  FB328_27 = s->m[28]*(s->l[1][28]*S28+s->l[2][28]*C28);
  CM328_27 = s->In[3][28]*C28-CM129_27*S29+CM329_27*C29-FB128_27*s->l[2][28]+FB228_27*s->l[1][28];
  CM328_28 = s->In[9][28]+s->m[28]*s->l[1][28]*s->l[1][28]+s->m[28]*s->l[2][28]*s->l[2][28]+C29*(s->In[9][29]*C29-
 FB129_28*s->l[2][29]+FB229_28*s->l[1][29])+S29*(s->In[1][29]*S29+FB229_28*s->l[3][29]-FB329_28*s->l[2][29]);
  FA127 = -(s->frc[1][27]-s->m[27]*(AlF127+BeF327*s->l[3][27]-s->l[1][27]*(OM227*OM227+OM327*OM327)+s->l[2][27]*(BS227-
 OpF327)));
  FA227 = -(s->frc[2][27]-s->m[27]*(AlF227+BeF627*s->l[3][27]+s->l[1][27]*(BS227+OpF327)-s->l[2][27]*(OM127*OM127+OM327*
 OM327)));
  FA327 = -(s->frc[3][27]-s->m[27]*(AlF327+BS927*s->l[3][27]+s->l[1][27]*(BS327-OpF227)+s->l[2][27]*(BS627+OpF126)));
  FF127 = FA127+FF128*C28-FF228*S28;
  FF227 = FA227+FF128*S28+FF228*C28;
  FF327 = FA327+FA328-FA129*S29+FA329*C29;
  CF127 = -(s->trq[1][27]-s->In[1][27]*OpF126-s->In[3][27]*OpF327-CF128*C28+CF228*S28+FA227*s->l[3][27]-FA327*
 s->l[2][27]-OM227*(s->In[3][27]*OM127-s->In[5][27]*OM327+s->In[9][27]*OM327)+s->dpt[3][53]*(FF128*S28+FF228*C28));
  CF227 = -(s->trq[2][27]-s->In[5][27]*OpF227-CF128*S28-CF228*C28-FA127*s->l[3][27]+FA327*s->l[1][27]+OM127*(
 s->In[3][27]*OM127+s->In[9][27]*OM327)-OM327*(s->In[1][27]*OM127+s->In[3][27]*OM327)-s->dpt[3][53]*(FF128*C28-FF228*S28));
  CF327 = -(s->trq[3][27]-CF328-s->In[3][27]*OpF126-s->In[9][27]*OpF327+FA127*s->l[2][27]-FA227*s->l[1][27]+OM227*(
 s->In[1][27]*OM127+s->In[3][27]*OM327-s->In[5][27]*OM127));
  FB127_1 = s->m[27]*AlM126_1;
  FB227_1 = s->m[27]*AlM227_1;
  FB327_1 = s->m[27]*AlM327_1;
  FM127_1 = FB127_1+FM128_1*C28-FM228_1*S28;
  FM227_1 = FB227_1+FM128_1*S28+FM228_1*C28;
  FM327_1 = FB327_1+FB328_1-FB129_1*S29+FB329_1*C29;
  CM127_1 = CM128_1*C28-CM228_1*S28-FB227_1*s->l[3][27]+FB327_1*s->l[2][27]-s->dpt[3][53]*(FM128_1*S28+FM228_1*C28);
  CM227_1 = CM128_1*S28+CM228_1*C28+FB127_1*s->l[3][27]-FB327_1*s->l[1][27]+s->dpt[3][53]*(FM128_1*C28-FM228_1*S28);
  CM327_1 = CM328_1-FB127_1*s->l[2][27]+FB227_1*s->l[1][27];
  FB127_2 = s->m[27]*AlM126_2;
  FB227_2 = s->m[27]*AlM227_2;
  FB327_2 = s->m[27]*AlM327_2;
  FM127_2 = FB127_2+FM128_2*C28-FM228_2*S28;
  FM227_2 = FB227_2+FM128_2*S28+FM228_2*C28;
  FM327_2 = FB327_2+FB328_2-FB129_2*S29+FB329_2*C29;
  CM127_2 = CM128_2*C28-CM228_2*S28-FB227_2*s->l[3][27]+FB327_2*s->l[2][27]-s->dpt[3][53]*(FM128_2*S28+FM228_2*C28);
  CM227_2 = CM128_2*S28+CM228_2*C28+FB127_2*s->l[3][27]-FB327_2*s->l[1][27]+s->dpt[3][53]*(FM128_2*C28-FM228_2*S28);
  CM327_2 = CM328_2-FB127_2*s->l[2][27]+FB227_2*s->l[1][27];
  FB127_3 = s->m[27]*AlM126_3;
  FB227_3 = s->m[27]*AlM227_3;
  FB327_3 = s->m[27]*AlM327_3;
  FM127_3 = FB127_3+FM128_3*C28-FM228_3*S28;
  FM227_3 = FB227_3+FM128_3*S28+FM228_3*C28;
  FM327_3 = FB327_3+FB328_3-FB129_3*S29+FB329_3*C29;
  CM127_3 = CM128_3*C28-CM228_3*S28-FB227_3*s->l[3][27]+FB327_3*s->l[2][27]-s->dpt[3][53]*(FM128_3*S28+FM228_3*C28);
  CM227_3 = CM128_3*S28+CM228_3*C28+FB127_3*s->l[3][27]-FB327_3*s->l[1][27]+s->dpt[3][53]*(FM128_3*C28-FM228_3*S28);
  CM327_3 = CM328_3-FB127_3*s->l[2][27]+FB227_3*s->l[1][27];
  FB127_4 = s->m[27]*(AlM127_4+OpM227_4*s->l[3][27]-OpM327_4*s->l[2][27]);
  FB227_4 = s->m[27]*(AlM227_4-OpM126_4*s->l[3][27]+OpM327_4*s->l[1][27]);
  FB327_4 = s->m[27]*(AlM327_4+OpM126_4*s->l[2][27]-OpM227_4*s->l[1][27]);
  FM127_4 = FB127_4+FM128_4*C28-FM228_4*S28;
  FM227_4 = FB227_4+FM128_4*S28+FM228_4*C28;
  FM327_4 = FB327_4+FB328_4-FB129_4*S29+FB329_4*C29;
  CM127_4 = s->In[1][27]*OpM126_4+s->In[3][27]*OpM327_4+CM128_4*C28-CM228_4*S28-FB227_4*s->l[3][27]+FB327_4*s->l[2][27]-
 s->dpt[3][53]*(FM128_4*S28+FM228_4*C28);
  CM227_4 = s->In[5][27]*OpM227_4+CM128_4*S28+CM228_4*C28+FB127_4*s->l[3][27]-FB327_4*s->l[1][27]+s->dpt[3][53]*(FM128_4
 *C28-FM228_4*S28);
  CM327_4 = CM328_4+s->In[3][27]*OpM126_4+s->In[9][27]*OpM327_4-FB127_4*s->l[2][27]+FB227_4*s->l[1][27];
  FB127_5 = s->m[27]*(AlM127_5+OpM227_5*s->l[3][27]-OpM327_5*s->l[2][27]);
  FB227_5 = s->m[27]*(AlM227_5-OpM126_5*s->l[3][27]+OpM327_5*s->l[1][27]);
  FB327_5 = s->m[27]*(AlM327_5+OpM126_5*s->l[2][27]-OpM227_5*s->l[1][27]);
  FM127_5 = FB127_5+FM128_5*C28-FM228_5*S28;
  FM227_5 = FB227_5+FM128_5*S28+FM228_5*C28;
  FM327_5 = FB327_5+FB328_5-FB129_5*S29+FB329_5*C29;
  CM127_5 = s->In[1][27]*OpM126_5+s->In[3][27]*OpM327_5+CM128_5*C28-CM228_5*S28-FB227_5*s->l[3][27]+FB327_5*s->l[2][27]-
 s->dpt[3][53]*(FM128_5*S28+FM228_5*C28);
  CM227_5 = s->In[5][27]*OpM227_5+CM128_5*S28+CM228_5*C28+FB127_5*s->l[3][27]-FB327_5*s->l[1][27]+s->dpt[3][53]*(FM128_5
 *C28-FM228_5*S28);
  CM327_5 = CM328_5+s->In[3][27]*OpM126_5+s->In[9][27]*OpM327_5-FB127_5*s->l[2][27]+FB227_5*s->l[1][27];
  FB127_6 = s->m[27]*(AlM127_6+OpM227_6*s->l[3][27]-OpM327_6*s->l[2][27]);
  FB227_6 = s->m[27]*(AlM227_6-OpM126_6*s->l[3][27]+OpM327_6*s->l[1][27]);
  FB327_6 = s->m[27]*(AlM327_6+OpM126_6*s->l[2][27]-OpM227_6*s->l[1][27]);
  FM127_6 = FB127_6+FM128_6*C28-FM228_6*S28;
  FM227_6 = FB227_6+FM128_6*S28+FM228_6*C28;
  FM327_6 = FB327_6+FB328_6-FB129_6*S29+FB329_6*C29;
  CM127_6 = s->In[1][27]*OpM126_6+s->In[3][27]*OpM327_6+CM128_6*C28-CM228_6*S28-FB227_6*s->l[3][27]+FB327_6*s->l[2][27]-
 s->dpt[3][53]*(FM128_6*S28+FM228_6*C28);
  CM227_6 = s->In[5][27]*OpM227_6+CM128_6*S28+CM228_6*C28+FB127_6*s->l[3][27]-FB327_6*s->l[1][27]+s->dpt[3][53]*(FM128_6
 *C28-FM228_6*S28);
  CM327_6 = CM328_6+s->In[3][27]*OpM126_6+s->In[9][27]*OpM327_6-FB127_6*s->l[2][27]+FB227_6*s->l[1][27];
  FB127_23 = s->m[27]*(AlM127_23+OpM227_23*s->l[3][27]-OpM327_23*s->l[2][27]);
  FB227_23 = s->m[27]*(AlM227_23-OpM126_23*s->l[3][27]+OpM327_23*s->l[1][27]);
  FB327_23 = s->m[27]*(AlM327_23+OpM126_23*s->l[2][27]-OpM227_23*s->l[1][27]);
  FM127_23 = FB127_23+FM128_23*C28-FM228_23*S28;
  FM227_23 = FB227_23+FM128_23*S28+FM228_23*C28;
  FM327_23 = FB327_23+FB328_23-FB129_23*S29+FB329_23*C29;
  CM127_23 = s->In[1][27]*OpM126_23+s->In[3][27]*OpM327_23+CM128_23*C28-CM228_23*S28-FB227_23*s->l[3][27]+FB327_23*
 s->l[2][27]-s->dpt[3][53]*(FM128_23*S28+FM228_23*C28);
  CM227_23 = s->In[5][27]*OpM227_23+CM128_23*S28+CM228_23*C28+FB127_23*s->l[3][27]-FB327_23*s->l[1][27]+s->dpt[3][53]*(
 FM128_23*C28-FM228_23*S28);
  CM327_23 = CM328_23+s->In[3][27]*OpM126_23+s->In[9][27]*OpM327_23-FB127_23*s->l[2][27]+FB227_23*s->l[1][27];
  FB127_24 = s->m[27]*(AlM127_24+OpM227_24*s->l[3][27]-OpM327_24*s->l[2][27]);
  FB227_24 = s->m[27]*(AlM227_24-OpM126_24*s->l[3][27]+OpM327_24*s->l[1][27]);
  FB327_24 = s->m[27]*(AlM327_24+OpM126_24*s->l[2][27]-OpM227_24*s->l[1][27]);
  FM127_24 = FB127_24+FM128_24*C28-FM228_24*S28;
  FM227_24 = FB227_24+FM128_24*S28+FM228_24*C28;
  FM327_24 = FB327_24+FB328_24-FB129_24*S29+FB329_24*C29;
  CM127_24 = s->In[1][27]*OpM126_24+s->In[3][27]*OpM327_24+CM128_24*C28-CM228_24*S28-FB227_24*s->l[3][27]+FB327_24*
 s->l[2][27]-s->dpt[3][53]*(FM128_24*S28+FM228_24*C28);
  CM227_24 = s->In[5][27]*OpM227_24+CM128_24*S28+CM228_24*C28+FB127_24*s->l[3][27]-FB327_24*s->l[1][27]+s->dpt[3][53]*(
 FM128_24*C28-FM228_24*S28);
  CM327_24 = CM328_24+s->In[3][27]*OpM126_24+s->In[9][27]*OpM327_24-FB127_24*s->l[2][27]+FB227_24*s->l[1][27];
  FB127_25 = s->m[27]*(AlM127_25+OpM227_25*s->l[3][27]-OpM327_25*s->l[2][27]);
  FB227_25 = s->m[27]*(AlM227_25+OpM327_25*s->l[1][27]+s->l[3][27]*S26);
  FB327_25 = s->m[27]*(AlM327_25-OpM227_25*s->l[1][27]-s->l[2][27]*S26);
  FM127_25 = FB127_25+FM128_25*C28-FM228_25*S28;
  FM227_25 = FB227_25+FM128_25*S28+FM228_25*C28;
  FM327_25 = FB327_25+FB328_25-FB129_25*S29+FB329_25*C29;
  CM127_25 = CM128_25*C28-CM228_25*S28-s->dpt[3][53]*(FM128_25*S28+FM228_25*C28)-s->In[1][27]*S26+s->In[3][27]*OpM327_25
 -FB227_25*s->l[3][27]+FB327_25*s->l[2][27];
  CM227_25 = s->In[5][27]*OpM227_25+CM128_25*S28+CM228_25*C28+FB127_25*s->l[3][27]-FB327_25*s->l[1][27]+s->dpt[3][53]*(
 FM128_25*C28-FM228_25*S28);
  CM327_25 = CM328_25-s->In[3][27]*S26+s->In[9][27]*OpM327_25-FB127_25*s->l[2][27]+FB227_25*s->l[1][27];
  FB127_26 = s->m[27]*(s->l[2][27]*S27+s->l[3][27]*C27);
  FB227_26 = -s->m[27]*s->l[1][27]*S27;
  FB327_26 = -s->m[27]*s->l[1][27]*C27;
  CM127_26 = CM128_26*C28-CM228_26*S28-s->dpt[3][53]*(FM128_26*S28+FM228_26*C28)-s->In[3][27]*S27-FB227_26*s->l[3][27]+
 FB327_26*s->l[2][27];
  CM127_27 = s->In[1][27]+s->m[27]*s->l[2][27]*s->l[2][27]+s->m[27]*s->l[3][27]*s->l[3][27]-s->dpt[3][53]*(C28*(FB228_27
 +FB229_27)+S28*(FB128_27+FB129_27*C29+FB329_27*S29))+C28*(s->In[1][28]*C28-s->In[2][28]*S28+CM129_27*C29+CM329_27*S29-
 FB228_27*s->l[3][28]-FB229_27*s->dpt[3][56]+FB328_27*s->l[2][28])-S28*(CM229_27+s->In[2][28]*C28-s->In[5][28]*S28+FB128_27*
 s->l[3][28]-FB328_27*s->l[1][28]+s->dpt[3][56]*(FB129_27*C29+FB329_27*S29));
  FA126 = -(s->frc[1][26]-s->m[26]*(AlF126+BeF226*s->l[2][26]-s->l[1][26]*(OM226*OM226+OM326*OM326)+s->l[3][26]*(BS326+
 OpF225)));
  FA226 = -(s->frc[2][26]-s->m[26]*(AlF226+BS526*s->l[2][26]+s->l[1][26]*(BS226+OpF326)+s->l[3][26]*(BS626-OpF126)));
  FA326 = -(s->frc[3][26]-s->m[26]*(AlF326+BeF826*s->l[2][26]+s->l[1][26]*(BS326-OpF225)-s->l[3][26]*(OM126*OM126+OM226*
 OM226)));
  FF126 = FA126+FF127;
  FF226 = FA226+FF227*C27-FF327*S27;
  FF326 = FA326+FF227*S27+FF327*C27;
  CF126 = -(s->trq[1][26]-CF127-s->In[1][26]*OpF126-s->In[2][26]*OpF225+FA226*s->l[3][26]-FA326*s->l[2][26]-OM226*(
 s->In[6][26]*OM226+s->In[9][26]*OM326)+OM326*(s->In[2][26]*OM126+s->In[5][26]*OM226+s->In[6][26]*OM326)-s->dpt[2][51]*(FF227
 *S27+FF327*C27));
  CF226 = -(s->trq[2][26]-s->In[2][26]*OpF126-s->In[5][26]*OpF225-s->In[6][26]*OpF326-CF227*C27+CF327*S27-FA126*
 s->l[3][26]+FA326*s->l[1][26]+OM126*(s->In[6][26]*OM226+s->In[9][26]*OM326)-OM326*(s->In[1][26]*OM126+s->In[2][26]*OM226));
  CF326 = -(s->trq[3][26]-s->In[6][26]*OpF225-s->In[9][26]*OpF326-CF227*S27-CF327*C27+FA126*s->l[2][26]-FA226*
 s->l[1][26]+FF127*s->dpt[2][51]-OM126*(s->In[2][26]*OM126+s->In[5][26]*OM226+s->In[6][26]*OM326)+OM226*(s->In[1][26]*OM126+
 s->In[2][26]*OM226));
  FB126_1 = s->m[26]*AlM126_1;
  FB226_1 = s->m[26]*AlM225_1;
  FB326_1 = s->m[26]*AlM326_1;
  FM126_1 = FB126_1+FM127_1;
  FM226_1 = FB226_1+FM227_1*C27-FM327_1*S27;
  FM326_1 = FB326_1+FM227_1*S27+FM327_1*C27;
  CM126_1 = CM127_1-FB226_1*s->l[3][26]+FB326_1*s->l[2][26]+s->dpt[2][51]*(FM227_1*S27+FM327_1*C27);
  CM226_1 = CM227_1*C27-CM327_1*S27+FB126_1*s->l[3][26]-FB326_1*s->l[1][26];
  CM326_1 = CM227_1*S27+CM327_1*C27-FB126_1*s->l[2][26]+FB226_1*s->l[1][26]-FM127_1*s->dpt[2][51];
  FB126_2 = s->m[26]*AlM126_2;
  FB226_2 = s->m[26]*AlM225_2;
  FB326_2 = s->m[26]*AlM326_2;
  FM126_2 = FB126_2+FM127_2;
  FM226_2 = FB226_2+FM227_2*C27-FM327_2*S27;
  FM326_2 = FB326_2+FM227_2*S27+FM327_2*C27;
  CM126_2 = CM127_2-FB226_2*s->l[3][26]+FB326_2*s->l[2][26]+s->dpt[2][51]*(FM227_2*S27+FM327_2*C27);
  CM226_2 = CM227_2*C27-CM327_2*S27+FB126_2*s->l[3][26]-FB326_2*s->l[1][26];
  CM326_2 = CM227_2*S27+CM327_2*C27-FB126_2*s->l[2][26]+FB226_2*s->l[1][26]-FM127_2*s->dpt[2][51];
  FB126_3 = s->m[26]*AlM126_3;
  FB226_3 = s->m[26]*AlM225_3;
  FB326_3 = s->m[26]*AlM326_3;
  FM126_3 = FB126_3+FM127_3;
  FM226_3 = FB226_3+FM227_3*C27-FM327_3*S27;
  FM326_3 = FB326_3+FM227_3*S27+FM327_3*C27;
  CM126_3 = CM127_3-FB226_3*s->l[3][26]+FB326_3*s->l[2][26]+s->dpt[2][51]*(FM227_3*S27+FM327_3*C27);
  CM226_3 = CM227_3*C27-CM327_3*S27+FB126_3*s->l[3][26]-FB326_3*s->l[1][26];
  CM326_3 = CM227_3*S27+CM327_3*C27-FB126_3*s->l[2][26]+FB226_3*s->l[1][26]-FM127_3*s->dpt[2][51];
  FB126_4 = s->m[26]*(AlM126_4+OpM225_4*s->l[3][26]-OpM326_4*s->l[2][26]);
  FB226_4 = s->m[26]*(AlM226_4-OpM126_4*s->l[3][26]+OpM326_4*s->l[1][26]);
  FB326_4 = s->m[26]*(AlM326_4+OpM126_4*s->l[2][26]-OpM225_4*s->l[1][26]);
  FM126_4 = FB126_4+FM127_4;
  FM226_4 = FB226_4+FM227_4*C27-FM327_4*S27;
  FM326_4 = FB326_4+FM227_4*S27+FM327_4*C27;
  CM126_4 = CM127_4+s->In[1][26]*OpM126_4+s->In[2][26]*OpM225_4-FB226_4*s->l[3][26]+FB326_4*s->l[2][26]+s->dpt[2][51]*(
 FM227_4*S27+FM327_4*C27);
  CM226_4 = s->In[2][26]*OpM126_4+s->In[5][26]*OpM225_4+s->In[6][26]*OpM326_4+CM227_4*C27-CM327_4*S27+FB126_4*
 s->l[3][26]-FB326_4*s->l[1][26];
  CM326_4 = s->In[6][26]*OpM225_4+s->In[9][26]*OpM326_4+CM227_4*S27+CM327_4*C27-FB126_4*s->l[2][26]+FB226_4*s->l[1][26]-
 FM127_4*s->dpt[2][51];
  FB126_5 = s->m[26]*(AlM126_5+OpM225_5*s->l[3][26]-OpM326_5*s->l[2][26]);
  FB226_5 = s->m[26]*(AlM226_5-OpM126_5*s->l[3][26]+OpM326_5*s->l[1][26]);
  FB326_5 = s->m[26]*(AlM326_5+OpM126_5*s->l[2][26]-OpM225_5*s->l[1][26]);
  FM126_5 = FB126_5+FM127_5;
  FM226_5 = FB226_5+FM227_5*C27-FM327_5*S27;
  FM326_5 = FB326_5+FM227_5*S27+FM327_5*C27;
  CM126_5 = CM127_5+s->In[1][26]*OpM126_5+s->In[2][26]*OpM225_5-FB226_5*s->l[3][26]+FB326_5*s->l[2][26]+s->dpt[2][51]*(
 FM227_5*S27+FM327_5*C27);
  CM226_5 = s->In[2][26]*OpM126_5+s->In[5][26]*OpM225_5+s->In[6][26]*OpM326_5+CM227_5*C27-CM327_5*S27+FB126_5*
 s->l[3][26]-FB326_5*s->l[1][26];
  CM326_5 = s->In[6][26]*OpM225_5+s->In[9][26]*OpM326_5+CM227_5*S27+CM327_5*C27-FB126_5*s->l[2][26]+FB226_5*s->l[1][26]-
 FM127_5*s->dpt[2][51];
  FB126_6 = s->m[26]*(AlM126_6+OpM225_6*s->l[3][26]-OpM326_6*s->l[2][26]);
  FB226_6 = s->m[26]*(AlM226_6-OpM126_6*s->l[3][26]+OpM326_6*s->l[1][26]);
  FB326_6 = s->m[26]*(AlM326_6+OpM126_6*s->l[2][26]-OpM225_6*s->l[1][26]);
  FM126_6 = FB126_6+FM127_6;
  FM226_6 = FB226_6+FM227_6*C27-FM327_6*S27;
  FM326_6 = FB326_6+FM227_6*S27+FM327_6*C27;
  CM126_6 = CM127_6+s->In[1][26]*OpM126_6+s->In[2][26]*OpM225_6-FB226_6*s->l[3][26]+FB326_6*s->l[2][26]+s->dpt[2][51]*(
 FM227_6*S27+FM327_6*C27);
  CM226_6 = s->In[2][26]*OpM126_6+s->In[5][26]*OpM225_6+s->In[6][26]*OpM326_6+CM227_6*C27-CM327_6*S27+FB126_6*
 s->l[3][26]-FB326_6*s->l[1][26];
  CM326_6 = s->In[6][26]*OpM225_6+s->In[9][26]*OpM326_6+CM227_6*S27+CM327_6*C27-FB126_6*s->l[2][26]+FB226_6*s->l[1][26]-
 FM127_6*s->dpt[2][51];
  FB126_23 = s->m[26]*(AlM126_23+OpM225_23*s->l[3][26]-OpM326_23*s->l[2][26]);
  FB226_23 = s->m[26]*(AlM226_23-OpM126_23*s->l[3][26]+OpM326_23*s->l[1][26]);
  FB326_23 = s->m[26]*(AlM326_23+OpM126_23*s->l[2][26]-OpM225_23*s->l[1][26]);
  FM126_23 = FB126_23+FM127_23;
  FM226_23 = FB226_23+FM227_23*C27-FM327_23*S27;
  FM326_23 = FB326_23+FM227_23*S27+FM327_23*C27;
  CM126_23 = CM127_23+s->In[1][26]*OpM126_23+s->In[2][26]*OpM225_23-FB226_23*s->l[3][26]+FB326_23*s->l[2][26]+
 s->dpt[2][51]*(FM227_23*S27+FM327_23*C27);
  CM226_23 = s->In[2][26]*OpM126_23+s->In[5][26]*OpM225_23+s->In[6][26]*OpM326_23+CM227_23*C27-CM327_23*S27+FB126_23*
 s->l[3][26]-FB326_23*s->l[1][26];
  CM326_23 = s->In[6][26]*OpM225_23+s->In[9][26]*OpM326_23+CM227_23*S27+CM327_23*C27-FB126_23*s->l[2][26]+FB226_23*
 s->l[1][26]-FM127_23*s->dpt[2][51];
  FB126_24 = s->m[26]*(AlM126_24-OpM326_24*s->l[2][26]+s->l[3][26]*C25);
  FB226_24 = s->m[26]*(AlM226_24-OpM126_24*s->l[3][26]+OpM326_24*s->l[1][26]);
  FB326_24 = s->m[26]*(AlM326_24+OpM126_24*s->l[2][26]-s->l[1][26]*C25);
  FM126_24 = FB126_24+FM127_24;
  FM226_24 = FB226_24+FM227_24*C27-FM327_24*S27;
  FM326_24 = FB326_24+FM227_24*S27+FM327_24*C27;
  CM126_24 = CM127_24+s->In[1][26]*OpM126_24+s->In[2][26]*C25-FB226_24*s->l[3][26]+FB326_24*s->l[2][26]+s->dpt[2][51]*(
 FM227_24*S27+FM327_24*C27);
  CM226_24 = s->In[2][26]*OpM126_24+s->In[5][26]*C25+s->In[6][26]*OpM326_24+CM227_24*C27-CM327_24*S27+FB126_24*
 s->l[3][26]-FB326_24*s->l[1][26];
  CM326_24 = s->In[6][26]*C25+s->In[9][26]*OpM326_24+CM227_24*S27+CM327_24*C27-FB126_24*s->l[2][26]+FB226_24*s->l[1][26]
 -FM127_24*s->dpt[2][51];
  FB126_25 = s->m[26]*(AlM126_25-s->l[2][26]*C26);
  FB226_25 = s->m[26]*(s->dpt[1][48]+s->l[1][26]*C26+s->l[3][26]*S26);
  FB326_25 = -s->m[26]*S26*(s->dpt[2][48]+s->l[2][26]);
  CM226_25 = CM227_25*C27-CM327_25*S27-s->In[2][26]*S26+s->In[6][26]*C26+FB126_25*s->l[3][26]-FB326_25*s->l[1][26];
  CM226_26 = s->In[5][26]+s->m[26]*s->l[1][26]*s->l[1][26]+s->m[26]*s->l[3][26]*s->l[3][26]+C27*(s->In[5][27]*C27+
 CM128_26*S28+CM228_26*C28+FB127_26*s->l[3][27]-FB327_26*s->l[1][27]+s->dpt[3][53]*(FM128_26*C28-FM228_26*S28))-S27*(CM328_26
 -s->In[9][27]*S27-FB127_26*s->l[2][27]+FB227_26*s->l[1][27]);

// = = Block_0_2_0_1_0_10 = = 
 
// Backward Dynamics 

  FA133 = -(s->frc[1][33]+s->m[33]*(s->l[1][33]*(OM233*OM233+OM333*OM333)-s->l[2][33]*(BS233-OpF333)-s->l[3][33]*(BS333+
 OpF232)-C33*(AlF132+BeF332*s->dpt[3][65])+S33*(AlF332+BS932*s->dpt[3][65])));
  FA233 = -(s->frc[2][33]-s->m[33]*(AlF232+BeF632*s->dpt[3][65]+s->l[1][33]*(BS233+OpF333)-s->l[2][33]*(OM133*OM133+
 OM333*OM333)+s->l[3][33]*(BS633-OpF133)));
  FA333 = -(s->frc[3][33]-s->m[33]*(s->l[1][33]*(BS333-OpF232)+s->l[2][33]*(BS633+OpF133)-s->l[3][33]*(OM133*OM133+OM233
 *OM233)+C33*(AlF332+BS932*s->dpt[3][65])+S33*(AlF132+BeF332*s->dpt[3][65])));
  CF133 = -(s->trq[1][33]-s->In[1][33]*OpF133-s->In[2][33]*OpF232+FA233*s->l[3][33]-FA333*s->l[2][33]-OM233*(
 s->In[6][33]*OM233+s->In[9][33]*OM333)+OM333*(s->In[2][33]*OM133+s->In[5][33]*OM233+s->In[6][33]*OM333));
  CF233 = -(s->trq[2][33]-s->In[2][33]*OpF133-s->In[5][33]*OpF232-s->In[6][33]*OpF333-FA133*s->l[3][33]+FA333*
 s->l[1][33]+OM133*(s->In[6][33]*OM233+s->In[9][33]*OM333)-OM333*(s->In[1][33]*OM133+s->In[2][33]*OM233));
  CF333 = -(s->trq[3][33]-s->In[6][33]*OpF232-s->In[9][33]*OpF333+FA133*s->l[2][33]-FA233*s->l[1][33]-OM133*(
 s->In[2][33]*OM133+s->In[5][33]*OM233+s->In[6][33]*OM333)+OM233*(s->In[1][33]*OM133+s->In[2][33]*OM233));
  FB133_1 = s->m[33]*(AlM132_1*C33-AlM331_1*S33);
  FB233_1 = s->m[33]*AlM232_1;
  FB333_1 = s->m[33]*(AlM132_1*S33+AlM331_1*C33);
  CM133_1 = -(FB233_1*s->l[3][33]-FB333_1*s->l[2][33]);
  CM233_1 = FB133_1*s->l[3][33]-FB333_1*s->l[1][33];
  CM333_1 = -(FB133_1*s->l[2][33]-FB233_1*s->l[1][33]);
  FB133_2 = s->m[33]*(AlM132_2*C33-AlM331_2*S33);
  FB233_2 = s->m[33]*AlM232_2;
  FB333_2 = s->m[33]*(AlM132_2*S33+AlM331_2*C33);
  CM133_2 = -(FB233_2*s->l[3][33]-FB333_2*s->l[2][33]);
  CM233_2 = FB133_2*s->l[3][33]-FB333_2*s->l[1][33];
  CM333_2 = -(FB133_2*s->l[2][33]-FB233_2*s->l[1][33]);
  FB133_3 = s->m[33]*(AlM132_3*C33-AlM331_3*S33);
  FB233_3 = s->m[33]*AlM232_3;
  FB333_3 = s->m[33]*(AlM132_3*S33+AlM331_3*C33);
  CM133_3 = -(FB233_3*s->l[3][33]-FB333_3*s->l[2][33]);
  CM233_3 = FB133_3*s->l[3][33]-FB333_3*s->l[1][33];
  CM333_3 = -(FB133_3*s->l[2][33]-FB233_3*s->l[1][33]);
  FB133_4 = -s->m[33]*(AlM331_4*S33-OpM232_4*s->l[3][33]+OpM333_4*s->l[2][33]-C33*(AlM132_4+OpM232_4*s->dpt[3][65]));
  FB233_4 = s->m[33]*(AlM232_4-OpM132_4*s->dpt[3][65]-OpM133_4*s->l[3][33]+OpM333_4*s->l[1][33]);
  FB333_4 = s->m[33]*(AlM331_4*C33+OpM133_4*s->l[2][33]-OpM232_4*s->l[1][33]+S33*(AlM132_4+OpM232_4*s->dpt[3][65]));
  CM133_4 = s->In[1][33]*OpM133_4+s->In[2][33]*OpM232_4-FB233_4*s->l[3][33]+FB333_4*s->l[2][33];
  CM233_4 = s->In[2][33]*OpM133_4+s->In[5][33]*OpM232_4+s->In[6][33]*OpM333_4+FB133_4*s->l[3][33]-FB333_4*s->l[1][33];
  CM333_4 = s->In[6][33]*OpM232_4+s->In[9][33]*OpM333_4-FB133_4*s->l[2][33]+FB233_4*s->l[1][33];
  FB133_5 = -s->m[33]*(AlM331_5*S33-OpM232_5*s->l[3][33]+OpM333_5*s->l[2][33]-C33*(AlM132_5+OpM232_5*s->dpt[3][65]));
  FB233_5 = s->m[33]*(AlM232_5-OpM132_5*s->dpt[3][65]-OpM133_5*s->l[3][33]+OpM333_5*s->l[1][33]);
  FB333_5 = s->m[33]*(AlM331_5*C33+OpM133_5*s->l[2][33]-OpM232_5*s->l[1][33]+S33*(AlM132_5+OpM232_5*s->dpt[3][65]));
  CM133_5 = s->In[1][33]*OpM133_5+s->In[2][33]*OpM232_5-FB233_5*s->l[3][33]+FB333_5*s->l[2][33];
  CM233_5 = s->In[2][33]*OpM133_5+s->In[5][33]*OpM232_5+s->In[6][33]*OpM333_5+FB133_5*s->l[3][33]-FB333_5*s->l[1][33];
  CM333_5 = s->In[6][33]*OpM232_5+s->In[9][33]*OpM333_5-FB133_5*s->l[2][33]+FB233_5*s->l[1][33];
  FB133_6 = -s->m[33]*(AlM331_6*S33-OpM232_6*s->l[3][33]+OpM333_6*s->l[2][33]-C33*(AlM132_6+OpM232_6*s->dpt[3][65]));
  FB233_6 = s->m[33]*(AlM232_6-OpM132_6*s->dpt[3][65]-OpM133_6*s->l[3][33]+OpM333_6*s->l[1][33]);
  FB333_6 = s->m[33]*(AlM331_6*C33+OpM133_6*s->l[2][33]-OpM232_6*s->l[1][33]+S33*(AlM132_6+OpM232_6*s->dpt[3][65]));
  CM133_6 = s->In[1][33]*OpM133_6+s->In[2][33]*OpM232_6-FB233_6*s->l[3][33]+FB333_6*s->l[2][33];
  CM233_6 = s->In[2][33]*OpM133_6+s->In[5][33]*OpM232_6+s->In[6][33]*OpM333_6+FB133_6*s->l[3][33]-FB333_6*s->l[1][33];
  CM333_6 = s->In[6][33]*OpM232_6+s->In[9][33]*OpM333_6-FB133_6*s->l[2][33]+FB233_6*s->l[1][33];
  FB133_23 = -s->m[33]*(AlM331_23*S33-OpM232_23*s->l[3][33]+OpM333_23*s->l[2][33]-C33*(AlM132_23+OpM232_23*s->dpt[3][65]
 ));
  FB233_23 = s->m[33]*(AlM232_23-OpM132_23*s->dpt[3][65]-OpM133_23*s->l[3][33]+OpM333_23*s->l[1][33]);
  FB333_23 = s->m[33]*(AlM331_23*C33+OpM133_23*s->l[2][33]-OpM232_23*s->l[1][33]+S33*(AlM132_23+OpM232_23*s->dpt[3][65])
 );
  CM133_23 = s->In[1][33]*OpM133_23+s->In[2][33]*OpM232_23-FB233_23*s->l[3][33]+FB333_23*s->l[2][33];
  CM233_23 = s->In[2][33]*OpM133_23+s->In[5][33]*OpM232_23+s->In[6][33]*OpM333_23+FB133_23*s->l[3][33]-FB333_23*
 s->l[1][33];
  CM333_23 = s->In[6][33]*OpM232_23+s->In[9][33]*OpM333_23-FB133_23*s->l[2][33]+FB233_23*s->l[1][33];
  FB133_24 = -s->m[33]*(AlM331_24*S33-OpM232_24*s->l[3][33]+OpM333_24*s->l[2][33]-C33*(AlM132_24+OpM232_24*s->dpt[3][65]
 ));
  FB233_24 = s->m[33]*(AlM232_24-OpM132_24*s->dpt[3][65]-OpM133_24*s->l[3][33]+OpM333_24*s->l[1][33]);
  FB333_24 = s->m[33]*(AlM331_24*C33+OpM133_24*s->l[2][33]-OpM232_24*s->l[1][33]+S33*(AlM132_24+OpM232_24*s->dpt[3][65])
 );
  CM133_24 = s->In[1][33]*OpM133_24+s->In[2][33]*OpM232_24-FB233_24*s->l[3][33]+FB333_24*s->l[2][33];
  CM233_24 = s->In[2][33]*OpM133_24+s->In[5][33]*OpM232_24+s->In[6][33]*OpM333_24+FB133_24*s->l[3][33]-FB333_24*
 s->l[1][33];
  CM333_24 = s->In[6][33]*OpM232_24+s->In[9][33]*OpM333_24-FB133_24*s->l[2][33]+FB233_24*s->l[1][33];
  FB133_25 = -s->m[33]*(AlM331_25*S33-OpM232_25*s->l[3][33]+OpM333_25*s->l[2][33]-C33*(AlM132_25+OpM232_25*s->dpt[3][65]
 ));
  FB233_25 = s->m[33]*(AlM232_25-OpM132_25*s->dpt[3][65]-OpM133_25*s->l[3][33]+OpM333_25*s->l[1][33]);
  FB333_25 = s->m[33]*(AlM331_25*C33+OpM133_25*s->l[2][33]-OpM232_25*s->l[1][33]+S33*(AlM132_25+OpM232_25*s->dpt[3][65])
 );
  CM133_25 = s->In[1][33]*OpM133_25+s->In[2][33]*OpM232_25-FB233_25*s->l[3][33]+FB333_25*s->l[2][33];
  CM233_25 = s->In[2][33]*OpM133_25+s->In[5][33]*OpM232_25+s->In[6][33]*OpM333_25+FB133_25*s->l[3][33]-FB333_25*
 s->l[1][33];
  CM333_25 = s->In[6][33]*OpM232_25+s->In[9][33]*OpM333_25-FB133_25*s->l[2][33]+FB233_25*s->l[1][33];
  FB133_30 = s->m[33]*(OpM232_30*s->l[3][33]-OpM333_30*s->l[2][33]+C33*(AlM132_30+OpM232_30*s->dpt[3][65]));
  FB233_30 = s->m[33]*(AlM232_30-OpM132_30*s->dpt[3][65]-OpM133_30*s->l[3][33]+OpM333_30*s->l[1][33]);
  FB333_30 = s->m[33]*(OpM133_30*s->l[2][33]-OpM232_30*s->l[1][33]+S33*(AlM132_30+OpM232_30*s->dpt[3][65]));
  CM133_30 = s->In[1][33]*OpM133_30+s->In[2][33]*OpM232_30-FB233_30*s->l[3][33]+FB333_30*s->l[2][33];
  CM233_30 = s->In[2][33]*OpM133_30+s->In[5][33]*OpM232_30+s->In[6][33]*OpM333_30+FB133_30*s->l[3][33]-FB333_30*
 s->l[1][33];
  CM333_30 = s->In[6][33]*OpM232_30+s->In[9][33]*OpM333_30-FB133_30*s->l[2][33]+FB233_30*s->l[1][33];
  FB133_31 = s->m[33]*(C33*(AlM132_31-s->dpt[3][65]*S32)-OpM333_31*s->l[2][33]-s->l[3][33]*S32);
  FB233_31 = s->m[33]*(AlM232_31-OpM133_31*s->l[3][33]+OpM333_31*s->l[1][33]-s->dpt[3][65]*C32);
  FB333_31 = s->m[33]*(OpM133_31*s->l[2][33]+s->l[1][33]*S32+S33*(AlM132_31-s->dpt[3][65]*S32));
  CM133_31 = s->In[1][33]*OpM133_31-s->In[2][33]*S32-FB233_31*s->l[3][33]+FB333_31*s->l[2][33];
  CM233_31 = s->In[2][33]*OpM133_31-s->In[5][33]*S32+s->In[6][33]*OpM333_31+FB133_31*s->l[3][33]-FB333_31*s->l[1][33];
  CM333_31 = -(s->In[6][33]*S32-s->In[9][33]*OpM333_31+FB133_31*s->l[2][33]-FB233_31*s->l[1][33]);
  FB133_32 = -s->m[33]*s->l[2][33]*C33;
  FB233_32 = s->m[33]*(s->l[1][33]*C33+s->l[3][33]*S33);
  FB333_32 = -s->m[33]*s->l[2][33]*S33;
  CM233_32 = -(s->In[2][33]*S33-s->In[6][33]*C33-FB133_32*s->l[3][33]+FB333_32*s->l[1][33]);
  CM233_33 = s->In[5][33]+s->m[33]*s->l[1][33]*s->l[1][33]+s->m[33]*s->l[3][33]*s->l[3][33];
  FA132 = -(s->frc[1][32]-s->m[32]*(AlF132+BeF332*s->l[3][32]-s->l[1][32]*(OM232*OM232+OM332*OM332)+s->l[2][32]*(BS232-
 OpF331)));
  FA232 = -(s->frc[2][32]-s->m[32]*(AlF232+BeF632*s->l[3][32]+s->l[1][32]*(BS232+OpF331)-s->l[2][32]*(OM132*OM132+OM332*
 OM332)));
  FA332 = -(s->frc[3][32]-s->m[32]*(AlF332+BS932*s->l[3][32]+s->l[1][32]*(BS332-OpF232)+s->l[2][32]*(BS632+OpF132)));
  FF132 = FA132+FA133*C33+FA333*S33;
  FF232 = FA232+FA233;
  CF132 = -(s->trq[1][32]-s->In[1][32]*OpF132-s->In[3][32]*OpF331-CF133*C33-CF333*S33+FA232*s->l[3][32]+FA233*
 s->dpt[3][65]-FA332*s->l[2][32]-OM232*(s->In[3][32]*OM132+s->In[6][32]*OM232+s->In[9][32]*OM332)+OM332*(s->In[5][32]*OM232+
 s->In[6][32]*OM332));
  CF232 = -(s->trq[2][32]-CF233-s->In[5][32]*OpF232-s->In[6][32]*OpF331-FA132*s->l[3][32]+FA332*s->l[1][32]+OM132*(
 s->In[3][32]*OM132+s->In[6][32]*OM232+s->In[9][32]*OM332)-OM332*(s->In[1][32]*OM132+s->In[3][32]*OM332)-s->dpt[3][65]*(FA133
 *C33+FA333*S33));
  CF332 = -(s->trq[3][32]-s->In[3][32]*OpF132-s->In[6][32]*OpF232-s->In[9][32]*OpF331+CF133*S33-CF333*C33+FA132*
 s->l[2][32]-FA232*s->l[1][32]-OM132*(s->In[5][32]*OM232+s->In[6][32]*OM332)+OM232*(s->In[1][32]*OM132+s->In[3][32]*OM332));
  FB132_1 = s->m[32]*AlM132_1;
  FB232_1 = s->m[32]*AlM232_1;
  FB332_1 = s->m[32]*AlM331_1;
  FM132_1 = FB132_1+FB133_1*C33+FB333_1*S33;
  FM232_1 = FB232_1+FB233_1;
  CM132_1 = CM133_1*C33+CM333_1*S33-FB232_1*s->l[3][32]-FB233_1*s->dpt[3][65]+FB332_1*s->l[2][32];
  CM232_1 = CM233_1+FB132_1*s->l[3][32]-FB332_1*s->l[1][32]+s->dpt[3][65]*(FB133_1*C33+FB333_1*S33);
  CM332_1 = -(CM133_1*S33-CM333_1*C33+FB132_1*s->l[2][32]-FB232_1*s->l[1][32]);
  FB132_2 = s->m[32]*AlM132_2;
  FB232_2 = s->m[32]*AlM232_2;
  FB332_2 = s->m[32]*AlM331_2;
  FM132_2 = FB132_2+FB133_2*C33+FB333_2*S33;
  FM232_2 = FB232_2+FB233_2;
  CM132_2 = CM133_2*C33+CM333_2*S33-FB232_2*s->l[3][32]-FB233_2*s->dpt[3][65]+FB332_2*s->l[2][32];
  CM232_2 = CM233_2+FB132_2*s->l[3][32]-FB332_2*s->l[1][32]+s->dpt[3][65]*(FB133_2*C33+FB333_2*S33);
  CM332_2 = -(CM133_2*S33-CM333_2*C33+FB132_2*s->l[2][32]-FB232_2*s->l[1][32]);
  FB132_3 = s->m[32]*AlM132_3;
  FB232_3 = s->m[32]*AlM232_3;
  FB332_3 = s->m[32]*AlM331_3;
  FM132_3 = FB132_3+FB133_3*C33+FB333_3*S33;
  FM232_3 = FB232_3+FB233_3;
  CM132_3 = CM133_3*C33+CM333_3*S33-FB232_3*s->l[3][32]-FB233_3*s->dpt[3][65]+FB332_3*s->l[2][32];
  CM232_3 = CM233_3+FB132_3*s->l[3][32]-FB332_3*s->l[1][32]+s->dpt[3][65]*(FB133_3*C33+FB333_3*S33);
  CM332_3 = -(CM133_3*S33-CM333_3*C33+FB132_3*s->l[2][32]-FB232_3*s->l[1][32]);
  FB132_4 = s->m[32]*(AlM132_4+OpM232_4*s->l[3][32]-OpM331_4*s->l[2][32]);
  FB232_4 = s->m[32]*(AlM232_4-OpM132_4*s->l[3][32]+OpM331_4*s->l[1][32]);
  FB332_4 = s->m[32]*(AlM331_4+OpM132_4*s->l[2][32]-OpM232_4*s->l[1][32]);
  FM132_4 = FB132_4+FB133_4*C33+FB333_4*S33;
  FM232_4 = FB232_4+FB233_4;
  CM132_4 = s->In[1][32]*OpM132_4+s->In[3][32]*OpM331_4+CM133_4*C33+CM333_4*S33-FB232_4*s->l[3][32]-FB233_4*
 s->dpt[3][65]+FB332_4*s->l[2][32];
  CM232_4 = CM233_4+s->In[5][32]*OpM232_4+s->In[6][32]*OpM331_4+FB132_4*s->l[3][32]-FB332_4*s->l[1][32]+s->dpt[3][65]*(
 FB133_4*C33+FB333_4*S33);
  CM332_4 = s->In[3][32]*OpM132_4+s->In[6][32]*OpM232_4+s->In[9][32]*OpM331_4-CM133_4*S33+CM333_4*C33-FB132_4*
 s->l[2][32]+FB232_4*s->l[1][32];
  FB132_5 = s->m[32]*(AlM132_5+OpM232_5*s->l[3][32]-OpM331_5*s->l[2][32]);
  FB232_5 = s->m[32]*(AlM232_5-OpM132_5*s->l[3][32]+OpM331_5*s->l[1][32]);
  FB332_5 = s->m[32]*(AlM331_5+OpM132_5*s->l[2][32]-OpM232_5*s->l[1][32]);
  FM132_5 = FB132_5+FB133_5*C33+FB333_5*S33;
  FM232_5 = FB232_5+FB233_5;
  CM132_5 = s->In[1][32]*OpM132_5+s->In[3][32]*OpM331_5+CM133_5*C33+CM333_5*S33-FB232_5*s->l[3][32]-FB233_5*
 s->dpt[3][65]+FB332_5*s->l[2][32];
  CM232_5 = CM233_5+s->In[5][32]*OpM232_5+s->In[6][32]*OpM331_5+FB132_5*s->l[3][32]-FB332_5*s->l[1][32]+s->dpt[3][65]*(
 FB133_5*C33+FB333_5*S33);
  CM332_5 = s->In[3][32]*OpM132_5+s->In[6][32]*OpM232_5+s->In[9][32]*OpM331_5-CM133_5*S33+CM333_5*C33-FB132_5*
 s->l[2][32]+FB232_5*s->l[1][32];
  FB132_6 = s->m[32]*(AlM132_6+OpM232_6*s->l[3][32]-OpM331_6*s->l[2][32]);
  FB232_6 = s->m[32]*(AlM232_6-OpM132_6*s->l[3][32]+OpM331_6*s->l[1][32]);
  FB332_6 = s->m[32]*(AlM331_6+OpM132_6*s->l[2][32]-OpM232_6*s->l[1][32]);
  FM132_6 = FB132_6+FB133_6*C33+FB333_6*S33;
  FM232_6 = FB232_6+FB233_6;
  CM132_6 = s->In[1][32]*OpM132_6+s->In[3][32]*OpM331_6+CM133_6*C33+CM333_6*S33-FB232_6*s->l[3][32]-FB233_6*
 s->dpt[3][65]+FB332_6*s->l[2][32];
  CM232_6 = CM233_6+s->In[5][32]*OpM232_6+s->In[6][32]*OpM331_6+FB132_6*s->l[3][32]-FB332_6*s->l[1][32]+s->dpt[3][65]*(
 FB133_6*C33+FB333_6*S33);
  CM332_6 = s->In[3][32]*OpM132_6+s->In[6][32]*OpM232_6+s->In[9][32]*OpM331_6-CM133_6*S33+CM333_6*C33-FB132_6*
 s->l[2][32]+FB232_6*s->l[1][32];
  FB132_23 = s->m[32]*(AlM132_23+OpM232_23*s->l[3][32]-OpM331_23*s->l[2][32]);
  FB232_23 = s->m[32]*(AlM232_23-OpM132_23*s->l[3][32]+OpM331_23*s->l[1][32]);
  FB332_23 = s->m[32]*(AlM331_23+OpM132_23*s->l[2][32]-OpM232_23*s->l[1][32]);
  FM132_23 = FB132_23+FB133_23*C33+FB333_23*S33;
  FM232_23 = FB232_23+FB233_23;
  CM132_23 = s->In[1][32]*OpM132_23+s->In[3][32]*OpM331_23+CM133_23*C33+CM333_23*S33-FB232_23*s->l[3][32]-FB233_23*
 s->dpt[3][65]+FB332_23*s->l[2][32];
  CM232_23 = CM233_23+s->In[5][32]*OpM232_23+s->In[6][32]*OpM331_23+FB132_23*s->l[3][32]-FB332_23*s->l[1][32]+
 s->dpt[3][65]*(FB133_23*C33+FB333_23*S33);
  CM332_23 = s->In[3][32]*OpM132_23+s->In[6][32]*OpM232_23+s->In[9][32]*OpM331_23-CM133_23*S33+CM333_23*C33-FB132_23*
 s->l[2][32]+FB232_23*s->l[1][32];
  FB132_24 = s->m[32]*(AlM132_24+OpM232_24*s->l[3][32]-OpM331_24*s->l[2][32]);
  FB232_24 = s->m[32]*(AlM232_24-OpM132_24*s->l[3][32]+OpM331_24*s->l[1][32]);
  FB332_24 = s->m[32]*(AlM331_24+OpM132_24*s->l[2][32]-OpM232_24*s->l[1][32]);
  FM132_24 = FB132_24+FB133_24*C33+FB333_24*S33;
  FM232_24 = FB232_24+FB233_24;
  CM132_24 = s->In[1][32]*OpM132_24+s->In[3][32]*OpM331_24+CM133_24*C33+CM333_24*S33-FB232_24*s->l[3][32]-FB233_24*
 s->dpt[3][65]+FB332_24*s->l[2][32];
  CM232_24 = CM233_24+s->In[5][32]*OpM232_24+s->In[6][32]*OpM331_24+FB132_24*s->l[3][32]-FB332_24*s->l[1][32]+
 s->dpt[3][65]*(FB133_24*C33+FB333_24*S33);
  CM332_24 = s->In[3][32]*OpM132_24+s->In[6][32]*OpM232_24+s->In[9][32]*OpM331_24-CM133_24*S33+CM333_24*C33-FB132_24*
 s->l[2][32]+FB232_24*s->l[1][32];
  FB132_25 = s->m[32]*(AlM132_25+OpM232_25*s->l[3][32]-OpM331_25*s->l[2][32]);
  FB232_25 = s->m[32]*(AlM232_25-OpM132_25*s->l[3][32]+OpM331_25*s->l[1][32]);
  FB332_25 = s->m[32]*(AlM331_25+OpM132_25*s->l[2][32]-OpM232_25*s->l[1][32]);
  FM132_25 = FB132_25+FB133_25*C33+FB333_25*S33;
  FM232_25 = FB232_25+FB233_25;
  CM132_25 = s->In[1][32]*OpM132_25+s->In[3][32]*OpM331_25+CM133_25*C33+CM333_25*S33-FB232_25*s->l[3][32]-FB233_25*
 s->dpt[3][65]+FB332_25*s->l[2][32];
  CM232_25 = CM233_25+s->In[5][32]*OpM232_25+s->In[6][32]*OpM331_25+FB132_25*s->l[3][32]-FB332_25*s->l[1][32]+
 s->dpt[3][65]*(FB133_25*C33+FB333_25*S33);
  CM332_25 = s->In[3][32]*OpM132_25+s->In[6][32]*OpM232_25+s->In[9][32]*OpM331_25-CM133_25*S33+CM333_25*C33-FB132_25*
 s->l[2][32]+FB232_25*s->l[1][32];
  FB132_30 = s->m[32]*(AlM132_30+OpM232_30*s->l[3][32]+s->l[2][32]*S31);
  FB232_30 = s->m[32]*(AlM232_30-OpM132_30*s->l[3][32]-s->l[1][32]*S31);
  FB332_30 = s->m[32]*(OpM132_30*s->l[2][32]-OpM232_30*s->l[1][32]);
  FM132_30 = FB132_30+FB133_30*C33+FB333_30*S33;
  FM232_30 = FB232_30+FB233_30;
  CM132_30 = s->In[1][32]*OpM132_30-s->In[3][32]*S31+CM133_30*C33+CM333_30*S33-FB232_30*s->l[3][32]-FB233_30*
 s->dpt[3][65]+FB332_30*s->l[2][32];
  CM232_30 = CM233_30+s->In[5][32]*OpM232_30-s->In[6][32]*S31+FB132_30*s->l[3][32]-FB332_30*s->l[1][32]+s->dpt[3][65]*(
 FB133_30*C33+FB333_30*S33);
  CM332_30 = s->In[3][32]*OpM132_30+s->In[6][32]*OpM232_30-s->In[9][32]*S31-CM133_30*S33+CM333_30*C33-FB132_30*
 s->l[2][32]+FB232_30*s->l[1][32];
  FB132_31 = s->m[32]*(AlM132_31-s->l[3][32]*S32);
  FB232_31 = s->m[32]*(AlM232_31-s->l[3][32]*C32);
  FB332_31 = s->m[32]*(s->l[1][32]*S32+s->l[2][32]*C32);
  CM332_31 = s->In[3][32]*C32-s->In[6][32]*S32-CM133_31*S33+CM333_31*C33-FB132_31*s->l[2][32]+FB232_31*s->l[1][32];
  CM332_32 = s->In[9][32]+s->m[32]*s->l[1][32]*s->l[1][32]+s->m[32]*s->l[2][32]*s->l[2][32]+C33*(s->In[9][33]*C33-
 FB133_32*s->l[2][33]+FB233_32*s->l[1][33])+S33*(s->In[1][33]*S33+FB233_32*s->l[3][33]-FB333_32*s->l[2][33]);
  FA131 = -(s->frc[1][31]-s->m[31]*(AlF131+BeF331*s->l[3][31]-s->l[1][31]*(OM231*OM231+OM331*OM331)+s->l[2][31]*(BS231-
 OpF331)));
  FA231 = -(s->frc[2][31]-s->m[31]*(AlF231+BeF631*s->l[3][31]+s->l[1][31]*(BS231+OpF331)-s->l[2][31]*(OM131*OM131+OM331*
 OM331)));
  FA331 = -(s->frc[3][31]-s->m[31]*(AlF331+BS931*s->l[3][31]+s->l[1][31]*(BS331-OpF231)+s->l[2][31]*(BS631+OpF130)));
  FF131 = FA131+FF132*C32-FF232*S32;
  FF231 = FA231+FF132*S32+FF232*C32;
  FF331 = FA331+FA332-FA133*S33+FA333*C33;
  CF131 = -(s->trq[1][31]-s->In[1][31]*OpF130-s->In[2][31]*OpF231-s->In[3][31]*OpF331-CF132*C32+CF232*S32+FA231*
 s->l[3][31]-FA331*s->l[2][31]-OM231*(s->In[3][31]*OM131+s->In[6][31]*OM231+s->In[9][31]*OM331)+OM331*(s->In[2][31]*OM131+
 s->In[5][31]*OM231+s->In[6][31]*OM331)+s->dpt[3][62]*(FF132*S32+FF232*C32));
  CF231 = -(s->trq[2][31]-s->In[2][31]*OpF130-s->In[5][31]*OpF231-s->In[6][31]*OpF331-CF132*S32-CF232*C32-FA131*
 s->l[3][31]+FA331*s->l[1][31]+OM131*(s->In[3][31]*OM131+s->In[6][31]*OM231+s->In[9][31]*OM331)-OM331*(s->In[1][31]*OM131+
 s->In[2][31]*OM231+s->In[3][31]*OM331)-s->dpt[3][62]*(FF132*C32-FF232*S32));
  CF331 = -(s->trq[3][31]-CF332-s->In[3][31]*OpF130-s->In[6][31]*OpF231-s->In[9][31]*OpF331+FA131*s->l[2][31]-FA231*
 s->l[1][31]-OM131*(s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331)+OM231*(s->In[1][31]*OM131+s->In[2][31]*OM231+
 s->In[3][31]*OM331));
  FB131_1 = s->m[31]*AlM130_1;
  FB231_1 = s->m[31]*AlM231_1;
  FB331_1 = s->m[31]*AlM331_1;
  FM131_1 = FB131_1+FM132_1*C32-FM232_1*S32;
  FM231_1 = FB231_1+FM132_1*S32+FM232_1*C32;
  FM331_1 = FB331_1+FB332_1-FB133_1*S33+FB333_1*C33;
  CM131_1 = CM132_1*C32-CM232_1*S32-FB231_1*s->l[3][31]+FB331_1*s->l[2][31]-s->dpt[3][62]*(FM132_1*S32+FM232_1*C32);
  CM231_1 = CM132_1*S32+CM232_1*C32+FB131_1*s->l[3][31]-FB331_1*s->l[1][31]+s->dpt[3][62]*(FM132_1*C32-FM232_1*S32);
  CM331_1 = CM332_1-FB131_1*s->l[2][31]+FB231_1*s->l[1][31];
  FB131_2 = s->m[31]*AlM130_2;
  FB231_2 = s->m[31]*AlM231_2;
  FB331_2 = s->m[31]*AlM331_2;
  FM131_2 = FB131_2+FM132_2*C32-FM232_2*S32;
  FM231_2 = FB231_2+FM132_2*S32+FM232_2*C32;
  FM331_2 = FB331_2+FB332_2-FB133_2*S33+FB333_2*C33;
  CM131_2 = CM132_2*C32-CM232_2*S32-FB231_2*s->l[3][31]+FB331_2*s->l[2][31]-s->dpt[3][62]*(FM132_2*S32+FM232_2*C32);
  CM231_2 = CM132_2*S32+CM232_2*C32+FB131_2*s->l[3][31]-FB331_2*s->l[1][31]+s->dpt[3][62]*(FM132_2*C32-FM232_2*S32);
  CM331_2 = CM332_2-FB131_2*s->l[2][31]+FB231_2*s->l[1][31];
  FB131_3 = s->m[31]*AlM130_3;
  FB231_3 = s->m[31]*AlM231_3;
  FB331_3 = s->m[31]*AlM331_3;
  FM131_3 = FB131_3+FM132_3*C32-FM232_3*S32;
  FM231_3 = FB231_3+FM132_3*S32+FM232_3*C32;
  FM331_3 = FB331_3+FB332_3-FB133_3*S33+FB333_3*C33;
  CM131_3 = CM132_3*C32-CM232_3*S32-FB231_3*s->l[3][31]+FB331_3*s->l[2][31]-s->dpt[3][62]*(FM132_3*S32+FM232_3*C32);
  CM231_3 = CM132_3*S32+CM232_3*C32+FB131_3*s->l[3][31]-FB331_3*s->l[1][31]+s->dpt[3][62]*(FM132_3*C32-FM232_3*S32);
  CM331_3 = CM332_3-FB131_3*s->l[2][31]+FB231_3*s->l[1][31];
  FB131_4 = s->m[31]*(AlM131_4+OpM231_4*s->l[3][31]-OpM331_4*s->l[2][31]);
  FB231_4 = s->m[31]*(AlM231_4-OpM130_4*s->l[3][31]+OpM331_4*s->l[1][31]);
  FB331_4 = s->m[31]*(AlM331_4+OpM130_4*s->l[2][31]-OpM231_4*s->l[1][31]);
  FM131_4 = FB131_4+FM132_4*C32-FM232_4*S32;
  FM231_4 = FB231_4+FM132_4*S32+FM232_4*C32;
  FM331_4 = FB331_4+FB332_4-FB133_4*S33+FB333_4*C33;
  CM131_4 = s->In[1][31]*OpM130_4+s->In[2][31]*OpM231_4+s->In[3][31]*OpM331_4+CM132_4*C32-CM232_4*S32-FB231_4*
 s->l[3][31]+FB331_4*s->l[2][31]-s->dpt[3][62]*(FM132_4*S32+FM232_4*C32);
  CM231_4 = s->In[2][31]*OpM130_4+s->In[5][31]*OpM231_4+s->In[6][31]*OpM331_4+CM132_4*S32+CM232_4*C32+FB131_4*
 s->l[3][31]-FB331_4*s->l[1][31]+s->dpt[3][62]*(FM132_4*C32-FM232_4*S32);
  CM331_4 = CM332_4+s->In[3][31]*OpM130_4+s->In[6][31]*OpM231_4+s->In[9][31]*OpM331_4-FB131_4*s->l[2][31]+FB231_4*
 s->l[1][31];
  FB131_5 = s->m[31]*(AlM131_5+OpM231_5*s->l[3][31]-OpM331_5*s->l[2][31]);
  FB231_5 = s->m[31]*(AlM231_5-OpM130_5*s->l[3][31]+OpM331_5*s->l[1][31]);
  FB331_5 = s->m[31]*(AlM331_5+OpM130_5*s->l[2][31]-OpM231_5*s->l[1][31]);
  FM131_5 = FB131_5+FM132_5*C32-FM232_5*S32;
  FM231_5 = FB231_5+FM132_5*S32+FM232_5*C32;
  FM331_5 = FB331_5+FB332_5-FB133_5*S33+FB333_5*C33;
  CM131_5 = s->In[1][31]*OpM130_5+s->In[2][31]*OpM231_5+s->In[3][31]*OpM331_5+CM132_5*C32-CM232_5*S32-FB231_5*
 s->l[3][31]+FB331_5*s->l[2][31]-s->dpt[3][62]*(FM132_5*S32+FM232_5*C32);
  CM231_5 = s->In[2][31]*OpM130_5+s->In[5][31]*OpM231_5+s->In[6][31]*OpM331_5+CM132_5*S32+CM232_5*C32+FB131_5*
 s->l[3][31]-FB331_5*s->l[1][31]+s->dpt[3][62]*(FM132_5*C32-FM232_5*S32);
  CM331_5 = CM332_5+s->In[3][31]*OpM130_5+s->In[6][31]*OpM231_5+s->In[9][31]*OpM331_5-FB131_5*s->l[2][31]+FB231_5*
 s->l[1][31];
  FB131_6 = s->m[31]*(AlM131_6+OpM231_6*s->l[3][31]-OpM331_6*s->l[2][31]);
  FB231_6 = s->m[31]*(AlM231_6-OpM130_6*s->l[3][31]+OpM331_6*s->l[1][31]);
  FB331_6 = s->m[31]*(AlM331_6+OpM130_6*s->l[2][31]-OpM231_6*s->l[1][31]);
  FM131_6 = FB131_6+FM132_6*C32-FM232_6*S32;
  FM231_6 = FB231_6+FM132_6*S32+FM232_6*C32;
  FM331_6 = FB331_6+FB332_6-FB133_6*S33+FB333_6*C33;
  CM131_6 = s->In[1][31]*OpM130_6+s->In[2][31]*OpM231_6+s->In[3][31]*OpM331_6+CM132_6*C32-CM232_6*S32-FB231_6*
 s->l[3][31]+FB331_6*s->l[2][31]-s->dpt[3][62]*(FM132_6*S32+FM232_6*C32);
  CM231_6 = s->In[2][31]*OpM130_6+s->In[5][31]*OpM231_6+s->In[6][31]*OpM331_6+CM132_6*S32+CM232_6*C32+FB131_6*
 s->l[3][31]-FB331_6*s->l[1][31]+s->dpt[3][62]*(FM132_6*C32-FM232_6*S32);
  CM331_6 = CM332_6+s->In[3][31]*OpM130_6+s->In[6][31]*OpM231_6+s->In[9][31]*OpM331_6-FB131_6*s->l[2][31]+FB231_6*
 s->l[1][31];
  FB131_23 = s->m[31]*(AlM131_23+OpM231_23*s->l[3][31]-OpM331_23*s->l[2][31]);
  FB231_23 = s->m[31]*(AlM231_23-OpM130_23*s->l[3][31]+OpM331_23*s->l[1][31]);
  FB331_23 = s->m[31]*(AlM331_23+OpM130_23*s->l[2][31]-OpM231_23*s->l[1][31]);
  FM131_23 = FB131_23+FM132_23*C32-FM232_23*S32;
  FM231_23 = FB231_23+FM132_23*S32+FM232_23*C32;
  FM331_23 = FB331_23+FB332_23-FB133_23*S33+FB333_23*C33;
  CM131_23 = s->In[1][31]*OpM130_23+s->In[2][31]*OpM231_23+s->In[3][31]*OpM331_23+CM132_23*C32-CM232_23*S32-FB231_23*
 s->l[3][31]+FB331_23*s->l[2][31]-s->dpt[3][62]*(FM132_23*S32+FM232_23*C32);
  CM231_23 = s->In[2][31]*OpM130_23+s->In[5][31]*OpM231_23+s->In[6][31]*OpM331_23+CM132_23*S32+CM232_23*C32+FB131_23*
 s->l[3][31]-FB331_23*s->l[1][31]+s->dpt[3][62]*(FM132_23*C32-FM232_23*S32);
  CM331_23 = CM332_23+s->In[3][31]*OpM130_23+s->In[6][31]*OpM231_23+s->In[9][31]*OpM331_23-FB131_23*s->l[2][31]+FB231_23
 *s->l[1][31];
  FB131_24 = s->m[31]*(AlM131_24+OpM231_24*s->l[3][31]-OpM331_24*s->l[2][31]);
  FB231_24 = s->m[31]*(AlM231_24-OpM130_24*s->l[3][31]+OpM331_24*s->l[1][31]);
  FB331_24 = s->m[31]*(AlM331_24+OpM130_24*s->l[2][31]-OpM231_24*s->l[1][31]);
  FM131_24 = FB131_24+FM132_24*C32-FM232_24*S32;
  FM231_24 = FB231_24+FM132_24*S32+FM232_24*C32;
  FM331_24 = FB331_24+FB332_24-FB133_24*S33+FB333_24*C33;
  CM131_24 = s->In[1][31]*OpM130_24+s->In[2][31]*OpM231_24+s->In[3][31]*OpM331_24+CM132_24*C32-CM232_24*S32-FB231_24*
 s->l[3][31]+FB331_24*s->l[2][31]-s->dpt[3][62]*(FM132_24*S32+FM232_24*C32);
  CM231_24 = s->In[2][31]*OpM130_24+s->In[5][31]*OpM231_24+s->In[6][31]*OpM331_24+CM132_24*S32+CM232_24*C32+FB131_24*
 s->l[3][31]-FB331_24*s->l[1][31]+s->dpt[3][62]*(FM132_24*C32-FM232_24*S32);
  CM331_24 = CM332_24+s->In[3][31]*OpM130_24+s->In[6][31]*OpM231_24+s->In[9][31]*OpM331_24-FB131_24*s->l[2][31]+FB231_24
 *s->l[1][31];
  FB131_25 = s->m[31]*(AlM131_25+OpM231_25*s->l[3][31]-OpM331_25*s->l[2][31]);
  FB231_25 = s->m[31]*(AlM231_25+OpM331_25*s->l[1][31]+s->l[3][31]*S30);
  FB331_25 = s->m[31]*(AlM331_25-OpM231_25*s->l[1][31]-s->l[2][31]*S30);
  FM131_25 = FB131_25+FM132_25*C32-FM232_25*S32;
  FM231_25 = FB231_25+FM132_25*S32+FM232_25*C32;
  FM331_25 = FB331_25+FB332_25-FB133_25*S33+FB333_25*C33;
  CM131_25 = CM132_25*C32-CM232_25*S32-s->dpt[3][62]*(FM132_25*S32+FM232_25*C32)-s->In[1][31]*S30+s->In[2][31]*OpM231_25
 +s->In[3][31]*OpM331_25-FB231_25*s->l[3][31]+FB331_25*s->l[2][31];
  CM231_25 = CM132_25*S32+CM232_25*C32+s->dpt[3][62]*(FM132_25*C32-FM232_25*S32)-s->In[2][31]*S30+s->In[5][31]*OpM231_25
 +s->In[6][31]*OpM331_25+FB131_25*s->l[3][31]-FB331_25*s->l[1][31];
  CM331_25 = CM332_25-s->In[3][31]*S30+s->In[6][31]*OpM231_25+s->In[9][31]*OpM331_25-FB131_25*s->l[2][31]+FB231_25*
 s->l[1][31];
  FB131_30 = s->m[31]*(s->l[2][31]*S31+s->l[3][31]*C31);
  FB231_30 = -s->m[31]*s->l[1][31]*S31;
  FB331_30 = -s->m[31]*s->l[1][31]*C31;
  CM131_30 = s->In[2][31]*C31-s->In[3][31]*S31+CM132_30*C32-CM232_30*S32-FB231_30*s->l[3][31]+FB331_30*s->l[2][31]-
 s->dpt[3][62]*(FM132_30*S32+FM232_30*C32);
  CM131_31 = s->In[1][31]+s->m[31]*s->l[2][31]*s->l[2][31]+s->m[31]*s->l[3][31]*s->l[3][31]-s->dpt[3][62]*(C32*(FB232_31
 +FB233_31)+S32*(FB132_31+FB133_31*C33+FB333_31*S33))+C32*(s->In[1][32]*C32+CM133_31*C33+CM333_31*S33-FB232_31*s->l[3][32]-
 FB233_31*s->dpt[3][65]+FB332_31*s->l[2][32])-S32*(CM233_31-s->In[5][32]*S32+FB132_31*s->l[3][32]-FB332_31*s->l[1][32]+
 s->dpt[3][65]*(FB133_31*C33+FB333_31*S33));
  FA130 = -(s->frc[1][30]-s->m[30]*(AlF130+BeF230*s->l[2][30]-s->l[1][30]*(OM230*OM230+OM330*OM330)+s->l[3][30]*(BS330+
 OpF225)));
  FA230 = -(s->frc[2][30]-s->m[30]*(AlF230+BS530*s->l[2][30]+s->l[1][30]*(BS230+OpF330)+s->l[3][30]*(BS630-OpF130)));
  FA330 = -(s->frc[3][30]-s->m[30]*(AlF330+BeF830*s->l[2][30]+s->l[1][30]*(BS330-OpF225)-s->l[3][30]*(OM130*OM130+OM230*
 OM230)));
  FF130 = FA130+FF131;
  FF230 = FA230+FF231*C31-FF331*S31;
  FF330 = FA330+FF231*S31+FF331*C31;
  CF130 = -(s->trq[1][30]-CF131-s->In[1][30]*OpF130+FA230*s->l[3][30]-FA330*s->l[2][30]+OM230*OM330*(s->In[5][30]-
 s->In[9][30])-s->dpt[2][60]*(FF231*S31+FF331*C31));
  CF230 = -(s->trq[2][30]-s->In[5][30]*OpF225-CF231*C31+CF331*S31-FA130*s->l[3][30]+FA330*s->l[1][30]-OM130*OM330*(
 s->In[1][30]-s->In[9][30]));
  CF330 = -(s->trq[3][30]-s->In[9][30]*OpF330-CF231*S31-CF331*C31+FA130*s->l[2][30]-FA230*s->l[1][30]+FF131*
 s->dpt[2][60]+OM130*OM230*(s->In[1][30]-s->In[5][30]));
  FB130_1 = s->m[30]*AlM130_1;
  FB230_1 = s->m[30]*AlM225_1;
  FB330_1 = s->m[30]*AlM330_1;
  FM130_1 = FB130_1+FM131_1;
  FM230_1 = FB230_1+FM231_1*C31-FM331_1*S31;
  FM330_1 = FB330_1+FM231_1*S31+FM331_1*C31;
  CM130_1 = CM131_1-FB230_1*s->l[3][30]+FB330_1*s->l[2][30]+s->dpt[2][60]*(FM231_1*S31+FM331_1*C31);
  CM230_1 = CM231_1*C31-CM331_1*S31+FB130_1*s->l[3][30]-FB330_1*s->l[1][30];
  CM330_1 = CM231_1*S31+CM331_1*C31-FB130_1*s->l[2][30]+FB230_1*s->l[1][30]-FM131_1*s->dpt[2][60];
  FB130_2 = s->m[30]*AlM130_2;
  FB230_2 = s->m[30]*AlM225_2;
  FB330_2 = s->m[30]*AlM330_2;
  FM130_2 = FB130_2+FM131_2;
  FM230_2 = FB230_2+FM231_2*C31-FM331_2*S31;
  FM330_2 = FB330_2+FM231_2*S31+FM331_2*C31;
  CM130_2 = CM131_2-FB230_2*s->l[3][30]+FB330_2*s->l[2][30]+s->dpt[2][60]*(FM231_2*S31+FM331_2*C31);
  CM230_2 = CM231_2*C31-CM331_2*S31+FB130_2*s->l[3][30]-FB330_2*s->l[1][30];
  CM330_2 = CM231_2*S31+CM331_2*C31-FB130_2*s->l[2][30]+FB230_2*s->l[1][30]-FM131_2*s->dpt[2][60];
  FB130_3 = s->m[30]*AlM130_3;
  FB230_3 = s->m[30]*AlM225_3;
  FB330_3 = s->m[30]*AlM330_3;
  FM130_3 = FB130_3+FM131_3;
  FM230_3 = FB230_3+FM231_3*C31-FM331_3*S31;
  FM330_3 = FB330_3+FM231_3*S31+FM331_3*C31;
  CM130_3 = CM131_3-FB230_3*s->l[3][30]+FB330_3*s->l[2][30]+s->dpt[2][60]*(FM231_3*S31+FM331_3*C31);
  CM230_3 = CM231_3*C31-CM331_3*S31+FB130_3*s->l[3][30]-FB330_3*s->l[1][30];
  CM330_3 = CM231_3*S31+CM331_3*C31-FB130_3*s->l[2][30]+FB230_3*s->l[1][30]-FM131_3*s->dpt[2][60];
  FB130_4 = s->m[30]*(AlM130_4+OpM225_4*s->l[3][30]-OpM330_4*s->l[2][30]);
  FB230_4 = s->m[30]*(AlM230_4-OpM130_4*s->l[3][30]+OpM330_4*s->l[1][30]);
  FB330_4 = s->m[30]*(AlM330_4+OpM130_4*s->l[2][30]-OpM225_4*s->l[1][30]);
  FM130_4 = FB130_4+FM131_4;
  FM230_4 = FB230_4+FM231_4*C31-FM331_4*S31;
  FM330_4 = FB330_4+FM231_4*S31+FM331_4*C31;
  CM130_4 = CM131_4+s->In[1][30]*OpM130_4-FB230_4*s->l[3][30]+FB330_4*s->l[2][30]+s->dpt[2][60]*(FM231_4*S31+FM331_4*C31
 );
  CM230_4 = s->In[5][30]*OpM225_4+CM231_4*C31-CM331_4*S31+FB130_4*s->l[3][30]-FB330_4*s->l[1][30];
  CM330_4 = s->In[9][30]*OpM330_4+CM231_4*S31+CM331_4*C31-FB130_4*s->l[2][30]+FB230_4*s->l[1][30]-FM131_4*s->dpt[2][60];
  FB130_5 = s->m[30]*(AlM130_5+OpM225_5*s->l[3][30]-OpM330_5*s->l[2][30]);
  FB230_5 = s->m[30]*(AlM230_5-OpM130_5*s->l[3][30]+OpM330_5*s->l[1][30]);
  FB330_5 = s->m[30]*(AlM330_5+OpM130_5*s->l[2][30]-OpM225_5*s->l[1][30]);
  FM130_5 = FB130_5+FM131_5;
  FM230_5 = FB230_5+FM231_5*C31-FM331_5*S31;
  FM330_5 = FB330_5+FM231_5*S31+FM331_5*C31;
  CM130_5 = CM131_5+s->In[1][30]*OpM130_5-FB230_5*s->l[3][30]+FB330_5*s->l[2][30]+s->dpt[2][60]*(FM231_5*S31+FM331_5*C31
 );
  CM230_5 = s->In[5][30]*OpM225_5+CM231_5*C31-CM331_5*S31+FB130_5*s->l[3][30]-FB330_5*s->l[1][30];
  CM330_5 = s->In[9][30]*OpM330_5+CM231_5*S31+CM331_5*C31-FB130_5*s->l[2][30]+FB230_5*s->l[1][30]-FM131_5*s->dpt[2][60];
  FB130_6 = s->m[30]*(AlM130_6+OpM225_6*s->l[3][30]-OpM330_6*s->l[2][30]);
  FB230_6 = s->m[30]*(AlM230_6-OpM130_6*s->l[3][30]+OpM330_6*s->l[1][30]);
  FB330_6 = s->m[30]*(AlM330_6+OpM130_6*s->l[2][30]-OpM225_6*s->l[1][30]);
  FM130_6 = FB130_6+FM131_6;
  FM230_6 = FB230_6+FM231_6*C31-FM331_6*S31;
  FM330_6 = FB330_6+FM231_6*S31+FM331_6*C31;
  CM130_6 = CM131_6+s->In[1][30]*OpM130_6-FB230_6*s->l[3][30]+FB330_6*s->l[2][30]+s->dpt[2][60]*(FM231_6*S31+FM331_6*C31
 );
  CM230_6 = s->In[5][30]*OpM225_6+CM231_6*C31-CM331_6*S31+FB130_6*s->l[3][30]-FB330_6*s->l[1][30];
  CM330_6 = s->In[9][30]*OpM330_6+CM231_6*S31+CM331_6*C31-FB130_6*s->l[2][30]+FB230_6*s->l[1][30]-FM131_6*s->dpt[2][60];
  FB130_23 = s->m[30]*(AlM130_23+OpM225_23*s->l[3][30]-OpM330_23*s->l[2][30]);
  FB230_23 = s->m[30]*(AlM230_23-OpM130_23*s->l[3][30]+OpM330_23*s->l[1][30]);
  FB330_23 = s->m[30]*(AlM330_23+OpM130_23*s->l[2][30]-OpM225_23*s->l[1][30]);
  FM130_23 = FB130_23+FM131_23;
  FM230_23 = FB230_23+FM231_23*C31-FM331_23*S31;
  FM330_23 = FB330_23+FM231_23*S31+FM331_23*C31;
  CM130_23 = CM131_23+s->In[1][30]*OpM130_23-FB230_23*s->l[3][30]+FB330_23*s->l[2][30]+s->dpt[2][60]*(FM231_23*S31+
 FM331_23*C31);
  CM230_23 = s->In[5][30]*OpM225_23+CM231_23*C31-CM331_23*S31+FB130_23*s->l[3][30]-FB330_23*s->l[1][30];
  CM330_23 = s->In[9][30]*OpM330_23+CM231_23*S31+CM331_23*C31-FB130_23*s->l[2][30]+FB230_23*s->l[1][30]-FM131_23*
 s->dpt[2][60];
  FB130_24 = s->m[30]*(AlM130_24-OpM330_24*s->l[2][30]+s->l[3][30]*C25);
  FB230_24 = s->m[30]*(AlM230_24-OpM130_24*s->l[3][30]+OpM330_24*s->l[1][30]);
  FB330_24 = s->m[30]*(AlM330_24+OpM130_24*s->l[2][30]-s->l[1][30]*C25);
  FM130_24 = FB130_24+FM131_24;
  FM230_24 = FB230_24+FM231_24*C31-FM331_24*S31;
  FM330_24 = FB330_24+FM231_24*S31+FM331_24*C31;
  CM130_24 = CM131_24+s->In[1][30]*OpM130_24-FB230_24*s->l[3][30]+FB330_24*s->l[2][30]+s->dpt[2][60]*(FM231_24*S31+
 FM331_24*C31);
  CM230_24 = s->In[5][30]*C25+CM231_24*C31-CM331_24*S31+FB130_24*s->l[3][30]-FB330_24*s->l[1][30];
  CM330_24 = s->In[9][30]*OpM330_24+CM231_24*S31+CM331_24*C31-FB130_24*s->l[2][30]+FB230_24*s->l[1][30]-FM131_24*
 s->dpt[2][60];
  FB130_25 = s->m[30]*(AlM130_25-s->l[2][30]*C30);
  FB230_25 = s->m[30]*(s->dpt[1][49]+s->l[1][30]*C30+s->l[3][30]*S30);
  FB330_25 = -s->m[30]*S30*(s->dpt[2][49]+s->l[2][30]);
  CM230_25 = CM231_25*C31-CM331_25*S31+FB130_25*s->l[3][30]-FB330_25*s->l[1][30];
  CM230_30 = s->In[5][30]+s->m[30]*s->l[1][30]*s->l[1][30]+s->m[30]*s->l[3][30]*s->l[3][30]+C31*(s->In[5][31]*C31-
 s->In[6][31]*S31+CM132_30*S32+CM232_30*C32+FB131_30*s->l[3][31]-FB331_30*s->l[1][31]+s->dpt[3][62]*(FM132_30*C32-FM232_30*
 S32))-S31*(CM332_30+s->In[6][31]*C31-s->In[9][31]*S31-FB131_30*s->l[2][31]+FB231_30*s->l[1][31]);

// = = Block_0_2_0_2_0_2 = = 
 
// Backward Dynamics 

  FA112 = -(s->frc[1][12]-s->m[12]*(AlF112+BS112*s->l[1][12]+BeF312*s->l[3][12]));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211+BeF412*s->l[1][12]+BeF612*s->l[3][12]));
  FA312 = -(s->frc[3][12]-s->m[12]*(AlF312+BS912*s->l[3][12]+BeF712*s->l[1][12]));
  FF112 = FA112+FA113+FA114;
  FF312 = FA312+FA313+FA314;
  CF112 = -(s->trq[1][12]+s->trq[1][13]+s->trq[1][14]-s->In[1][12]*OpF112-s->In[1][13]*OpF112-s->In[1][14]*OpF112+Dz133*
 FA213+Dz143*FA214+FA212*s->l[3][12]+OM212*OM312*(s->In[5][12]-s->In[9][12])+OM212*OM312*(s->In[5][13]-s->In[9][13])+OM212*
 OM312*(s->In[5][14]-s->In[9][14]));
  CF212 = -(s->trq[2][12]+s->trq[2][13]+s->trq[2][14]-s->In[5][12]*OpF211-s->In[5][13]*OpF211-s->In[5][14]*OpF211-Dz133*
 FA113-Dz143*FA114-FA112*s->l[3][12]+FA312*s->l[1][12]+FA313*s->dpt[1][20]+FA314*s->dpt[1][21]-OM112*OM312*(s->In[1][12]-
 s->In[9][12])-OM112*OM312*(s->In[1][13]-s->In[9][13])-OM112*OM312*(s->In[1][14]-s->In[9][14]));
  CF312 = -(s->trq[3][12]+s->trq[3][13]+s->trq[3][14]-s->In[9][12]*OpF312-s->In[9][13]*OpF312-s->In[9][14]*OpF312-FA212*
 s->l[1][12]-FA213*s->dpt[1][20]-FA214*s->dpt[1][21]+OM112*OM212*(s->In[1][12]-s->In[5][12])+OM112*OM212*(s->In[1][13]-
 s->In[5][13])+OM112*OM212*(s->In[1][14]-s->In[5][14]));
  FB112_1 = s->m[12]*AlM112_1;
  FB212_1 = s->m[12]*AlM211_1;
  FB312_1 = s->m[12]*AlM312_1;
  FM112_1 = FB112_1+FB113_1+FB114_1;
  FM312_1 = FB312_1+FB313_1+FB314_1;
  CM112_1 = -(Dz133*FB213_1+Dz143*FB214_1+FB212_1*s->l[3][12]);
  CM212_1 = Dz133*FB113_1+Dz143*FB114_1+FB112_1*s->l[3][12]-FB312_1*s->l[1][12]-FB313_1*s->dpt[1][20]-FB314_1*
 s->dpt[1][21];
  CM312_1 = FB212_1*s->l[1][12]+FB213_1*s->dpt[1][20]+FB214_1*s->dpt[1][21];
  FB112_2 = s->m[12]*AlM112_2;
  FB212_2 = s->m[12]*AlM211_2;
  FB312_2 = s->m[12]*AlM312_2;
  FM112_2 = FB112_2+FB113_2+FB114_2;
  FM312_2 = FB312_2+FB313_2+FB314_2;
  CM112_2 = -(Dz133*FB213_2+Dz143*FB214_2+FB212_2*s->l[3][12]);
  CM212_2 = Dz133*FB113_2+Dz143*FB114_2+FB112_2*s->l[3][12]-FB312_2*s->l[1][12]-FB313_2*s->dpt[1][20]-FB314_2*
 s->dpt[1][21];
  CM312_2 = FB212_2*s->l[1][12]+FB213_2*s->dpt[1][20]+FB214_2*s->dpt[1][21];
  FB112_3 = s->m[12]*AlM112_3;
  FB212_3 = s->m[12]*AlM211_3;
  FB312_3 = s->m[12]*AlM312_3;
  FM112_3 = FB112_3+FB113_3+FB114_3;
  FM312_3 = FB312_3+FB313_3+FB314_3;
  CM112_3 = -(Dz133*FB213_3+Dz143*FB214_3+FB212_3*s->l[3][12]);
  CM212_3 = Dz133*FB113_3+Dz143*FB114_3+FB112_3*s->l[3][12]-FB312_3*s->l[1][12]-FB313_3*s->dpt[1][20]-FB314_3*
 s->dpt[1][21];
  CM312_3 = FB212_3*s->l[1][12]+FB213_3*s->dpt[1][20]+FB214_3*s->dpt[1][21];
  FB112_4 = s->m[12]*(AlM112_4+OpM211_4*s->l[3][12]);
  FB212_4 = s->m[12]*(AlM211_4-OpM112_4*s->l[3][12]+OpM312_4*s->l[1][12]);
  FB312_4 = s->m[12]*(AlM312_4-OpM211_4*s->l[1][12]);
  FM112_4 = FB112_4+FB113_4+FB114_4;
  FM312_4 = FB312_4+FB313_4+FB314_4;
  CM112_4 = s->In[1][13]*OpM112_4-Dz133*FB213_4-Dz143*FB214_4-FB212_4*s->l[3][12]+OpM112_4*(s->In[1][12]+s->In[1][14]);
  CM212_4 = s->In[5][13]*OpM211_4+Dz133*FB113_4+Dz143*FB114_4+FB112_4*s->l[3][12]-FB312_4*s->l[1][12]-FB313_4*
 s->dpt[1][20]-FB314_4*s->dpt[1][21]+OpM211_4*(s->In[5][12]+s->In[5][14]);
  CM312_4 = s->In[9][13]*OpM312_4+FB212_4*s->l[1][12]+FB213_4*s->dpt[1][20]+FB214_4*s->dpt[1][21]+OpM312_4*(s->In[9][12]
 +s->In[9][14]);
  FB112_5 = s->m[12]*(AlM112_5+OpM211_5*s->l[3][12]);
  FB212_5 = s->m[12]*(AlM211_5-OpM112_5*s->l[3][12]+OpM312_5*s->l[1][12]);
  FB312_5 = s->m[12]*(AlM312_5-OpM211_5*s->l[1][12]);
  FM112_5 = FB112_5+FB113_5+FB114_5;
  FM312_5 = FB312_5+FB313_5+FB314_5;
  CM112_5 = s->In[1][13]*OpM112_5-Dz133*FB213_5-Dz143*FB214_5-FB212_5*s->l[3][12]+OpM112_5*(s->In[1][12]+s->In[1][14]);
  CM212_5 = s->In[5][13]*OpM211_5+Dz133*FB113_5+Dz143*FB114_5+FB112_5*s->l[3][12]-FB312_5*s->l[1][12]-FB313_5*
 s->dpt[1][20]-FB314_5*s->dpt[1][21]+OpM211_5*(s->In[5][12]+s->In[5][14]);
  CM312_5 = s->In[9][13]*OpM312_5+FB212_5*s->l[1][12]+FB213_5*s->dpt[1][20]+FB214_5*s->dpt[1][21]+OpM312_5*(s->In[9][12]
 +s->In[9][14]);
  FB112_6 = s->m[12]*(AlM112_6+OpM211_6*s->l[3][12]);
  FB212_6 = s->m[12]*(AlM211_6-OpM112_6*s->l[3][12]+OpM312_6*s->l[1][12]);
  FB312_6 = s->m[12]*(AlM312_6-OpM211_6*s->l[1][12]);
  FM112_6 = FB112_6+FB113_6+FB114_6;
  FM312_6 = FB312_6+FB313_6+FB314_6;
  CM112_6 = s->In[1][13]*OpM112_6-Dz133*FB213_6-Dz143*FB214_6-FB212_6*s->l[3][12]+OpM112_6*(s->In[1][12]+s->In[1][14]);
  CM212_6 = s->In[5][13]*OpM211_6+Dz133*FB113_6+Dz143*FB114_6+FB112_6*s->l[3][12]-FB312_6*s->l[1][12]-FB313_6*
 s->dpt[1][20]-FB314_6*s->dpt[1][21]+OpM211_6*(s->In[5][12]+s->In[5][14]);
  CM312_6 = s->In[9][13]*OpM312_6+FB212_6*s->l[1][12]+FB213_6*s->dpt[1][20]+FB214_6*s->dpt[1][21]+OpM312_6*(s->In[9][12]
 +s->In[9][14]);
  FB112_7 = s->m[12]*(AlM112_7+OpM211_7*s->l[3][12]);
  FB212_7 = s->m[12]*(AlM211_7-OpM112_7*s->l[3][12]+OpM312_7*s->l[1][12]);
  FB312_7 = s->m[12]*(AlM312_7-OpM211_7*s->l[1][12]);
  FM112_7 = FB112_7+FB113_7+FB114_7;
  FM312_7 = FB312_7+FB313_7+FB314_7;
  CM112_7 = s->In[1][13]*OpM112_7-Dz133*FB213_7-Dz143*FB214_7-FB212_7*s->l[3][12]+OpM112_7*(s->In[1][12]+s->In[1][14]);
  CM212_7 = s->In[5][13]*OpM211_7+Dz133*FB113_7+Dz143*FB114_7+FB112_7*s->l[3][12]-FB312_7*s->l[1][12]-FB313_7*
 s->dpt[1][20]-FB314_7*s->dpt[1][21]+OpM211_7*(s->In[5][12]+s->In[5][14]);
  CM312_7 = s->In[9][13]*OpM312_7+FB212_7*s->l[1][12]+FB213_7*s->dpt[1][20]+FB214_7*s->dpt[1][21]+OpM312_7*(s->In[9][12]
 +s->In[9][14]);
  FB112_8 = s->m[12]*(AlM112_8+OpM211_8*s->l[3][12]);
  FB212_8 = s->m[12]*(AlM211_8-OpM112_8*s->l[3][12]+OpM312_8*s->l[1][12]);
  FB312_8 = s->m[12]*(AlM312_8-OpM211_8*s->l[1][12]);
  FM112_8 = FB112_8+FB113_8+FB114_8;
  FM312_8 = FB312_8+FB313_8+FB314_8;
  CM112_8 = s->In[1][13]*OpM112_8-Dz133*FB213_8-Dz143*FB214_8-FB212_8*s->l[3][12]+OpM112_8*(s->In[1][12]+s->In[1][14]);
  CM212_8 = s->In[5][13]*OpM211_8+Dz133*FB113_8+Dz143*FB114_8+FB112_8*s->l[3][12]-FB312_8*s->l[1][12]-FB313_8*
 s->dpt[1][20]-FB314_8*s->dpt[1][21]+OpM211_8*(s->In[5][12]+s->In[5][14]);
  CM312_8 = s->In[9][13]*OpM312_8+FB212_8*s->l[1][12]+FB213_8*s->dpt[1][20]+FB214_8*s->dpt[1][21]+OpM312_8*(s->In[9][12]
 +s->In[9][14]);
  FB112_9 = s->m[12]*(AlM112_9+OpM211_9*s->l[3][12]);
  FB212_9 = s->m[12]*(AlM211_9-OpM112_9*s->l[3][12]+OpM312_9*s->l[1][12]);
  FB312_9 = s->m[12]*(AlM312_9-OpM211_9*s->l[1][12]);
  FM112_9 = FB112_9+FB113_9+FB114_9;
  FM312_9 = FB312_9+FB313_9+FB314_9;
  CM112_9 = s->In[1][13]*OpM112_9-Dz133*FB213_9-Dz143*FB214_9-FB212_9*s->l[3][12]+OpM112_9*(s->In[1][12]+s->In[1][14]);
  CM212_9 = s->In[5][13]*OpM211_9+Dz133*FB113_9+Dz143*FB114_9+FB112_9*s->l[3][12]-FB312_9*s->l[1][12]-FB313_9*
 s->dpt[1][20]-FB314_9*s->dpt[1][21]+OpM211_9*(s->In[5][12]+s->In[5][14]);
  CM312_9 = s->In[9][13]*OpM312_9+FB212_9*s->l[1][12]+FB213_9*s->dpt[1][20]+FB214_9*s->dpt[1][21]+OpM312_9*(s->In[9][12]
 +s->In[9][14]);
  FB112_10 = s->m[12]*(AlM112_10+s->l[3][12]*C11);
  FB212_10 = -s->m[12]*(OpM112_10*s->l[3][12]-OpM312_10*s->l[1][12]);
  FB312_10 = s->m[12]*(AlM312_10-s->l[1][12]*C11);
  CM112_10 = s->In[1][13]*OpM112_10-Dz133*FB213_10-Dz143*FB214_10-FB212_10*s->l[3][12]+OpM112_10*(s->In[1][12]+
 s->In[1][14]);
  CM212_10 = s->In[5][13]*C11+Dz133*FB113_10+Dz143*FB114_10+FB112_10*s->l[3][12]-FB312_10*s->l[1][12]-FB313_10*
 s->dpt[1][20]-FB314_10*s->dpt[1][21]+C11*(s->In[5][12]+s->In[5][14]);
  CM312_10 = s->In[9][13]*OpM312_10+FB212_10*s->l[1][12]+FB213_10*s->dpt[1][20]+FB214_10*s->dpt[1][21]+OpM312_10*(
 s->In[9][12]+s->In[9][14]);
  FB212_11 = s->m[12]*(s->l[1][12]*S12-s->l[3][12]*C12);
  CM212_12 = s->In[5][12]+s->In[5][13]+s->In[5][14]+s->m[12]*s->l[1][12]*s->l[1][12]+s->m[12]*s->l[3][12]*s->l[3][12]+
 s->m[13]*Dz133*Dz133+s->m[14]*Dz143*Dz143-FB313_12*s->dpt[1][20]-FB314_12*s->dpt[1][21];
  FA111 = -(s->frc[1][11]-s->m[11]*(AlF111-s->l[1][11]*(OM211*OM211+OM311*OM311)+s->l[2][11]*(BS211-OpF311)+s->l[3][11]*
 (BS311+OpF211)));
  FA211 = -(s->frc[2][11]-s->m[11]*(AlF211+s->l[1][11]*(BS211+OpF311)-s->l[2][11]*(OM111*OM111+OM311*OM311)+s->l[3][11]*
 (BS611-OpF110)));
  FA311 = -(s->frc[3][11]-s->m[11]*(AlF311+s->l[1][11]*(BS311-OpF211)+s->l[2][11]*(BS611+OpF110)-s->l[3][11]*(OM111*
 OM111+OM211*OM211)));
  FF111 = FA111+FF112*C12+FF312*S12;
  FF211 = FA211+FA212+FA213+FA214;
  FF311 = FA311-FF112*S12+FF312*C12;
  CF111 = -(s->trq[1][11]-s->In[1][11]*OpF110-s->In[2][11]*OpF211-s->In[3][11]*OpF311-CF112*C12-CF312*S12+FA211*
 s->l[3][11]-FA311*s->l[2][11]-OM211*(s->In[3][11]*OM111+s->In[9][11]*OM311)+OM311*(s->In[2][11]*OM111+s->In[5][11]*OM211));
  CF211 = -(s->trq[2][11]-CF212-s->In[2][11]*OpF110-s->In[5][11]*OpF211-FA111*s->l[3][11]+FA311*s->l[1][11]+OM111*(
 s->In[3][11]*OM111+s->In[9][11]*OM311)-OM311*(s->In[1][11]*OM111+s->In[2][11]*OM211+s->In[3][11]*OM311));
  CF311 = -(s->trq[3][11]-s->In[3][11]*OpF110-s->In[9][11]*OpF311+CF112*S12-CF312*C12+FA111*s->l[2][11]-FA211*
 s->l[1][11]-OM111*(s->In[2][11]*OM111+s->In[5][11]*OM211)+OM211*(s->In[1][11]*OM111+s->In[2][11]*OM211+s->In[3][11]*OM311));
  FB111_1 = s->m[11]*AlM110_1;
  FB211_1 = s->m[11]*AlM211_1;
  FB311_1 = s->m[11]*AlM311_1;
  FM111_1 = FB111_1+FM112_1*C12+FM312_1*S12;
  FM211_1 = FB211_1+FB212_1+FB213_1+FB214_1;
  FM311_1 = FB311_1-FM112_1*S12+FM312_1*C12;
  CM111_1 = CM112_1*C12+CM312_1*S12-FB211_1*s->l[3][11]+FB311_1*s->l[2][11];
  CM211_1 = CM212_1+FB111_1*s->l[3][11]-FB311_1*s->l[1][11];
  CM311_1 = -(CM112_1*S12-CM312_1*C12+FB111_1*s->l[2][11]-FB211_1*s->l[1][11]);
  FB111_2 = s->m[11]*AlM110_2;
  FB211_2 = s->m[11]*AlM211_2;
  FB311_2 = s->m[11]*AlM311_2;
  FM111_2 = FB111_2+FM112_2*C12+FM312_2*S12;
  FM211_2 = FB211_2+FB212_2+FB213_2+FB214_2;
  FM311_2 = FB311_2-FM112_2*S12+FM312_2*C12;
  CM111_2 = CM112_2*C12+CM312_2*S12-FB211_2*s->l[3][11]+FB311_2*s->l[2][11];
  CM211_2 = CM212_2+FB111_2*s->l[3][11]-FB311_2*s->l[1][11];
  CM311_2 = -(CM112_2*S12-CM312_2*C12+FB111_2*s->l[2][11]-FB211_2*s->l[1][11]);
  FB111_3 = s->m[11]*AlM110_3;
  FB211_3 = s->m[11]*AlM211_3;
  FB311_3 = s->m[11]*AlM311_3;
  FM111_3 = FB111_3+FM112_3*C12+FM312_3*S12;
  FM211_3 = FB211_3+FB212_3+FB213_3+FB214_3;
  FM311_3 = FB311_3-FM112_3*S12+FM312_3*C12;
  CM111_3 = CM112_3*C12+CM312_3*S12-FB211_3*s->l[3][11]+FB311_3*s->l[2][11];
  CM211_3 = CM212_3+FB111_3*s->l[3][11]-FB311_3*s->l[1][11];
  CM311_3 = -(CM112_3*S12-CM312_3*C12+FB111_3*s->l[2][11]-FB211_3*s->l[1][11]);
  FB111_4 = s->m[11]*(AlM111_4+OpM211_4*s->l[3][11]-OpM311_4*s->l[2][11]);
  FB211_4 = s->m[11]*(AlM211_4-OpM110_4*s->l[3][11]+OpM311_4*s->l[1][11]);
  FB311_4 = s->m[11]*(AlM311_4+OpM110_4*s->l[2][11]-OpM211_4*s->l[1][11]);
  FM111_4 = FB111_4+FM112_4*C12+FM312_4*S12;
  FM211_4 = FB211_4+FB212_4+FB213_4+FB214_4;
  FM311_4 = FB311_4-FM112_4*S12+FM312_4*C12;
  CM111_4 = s->In[1][11]*OpM110_4+s->In[2][11]*OpM211_4+s->In[3][11]*OpM311_4+CM112_4*C12+CM312_4*S12-FB211_4*
 s->l[3][11]+FB311_4*s->l[2][11];
  CM211_4 = CM212_4+s->In[2][11]*OpM110_4+s->In[5][11]*OpM211_4+FB111_4*s->l[3][11]-FB311_4*s->l[1][11];
  CM311_4 = s->In[3][11]*OpM110_4+s->In[9][11]*OpM311_4-CM112_4*S12+CM312_4*C12-FB111_4*s->l[2][11]+FB211_4*s->l[1][11];
  FB111_5 = s->m[11]*(AlM111_5+OpM211_5*s->l[3][11]-OpM311_5*s->l[2][11]);
  FB211_5 = s->m[11]*(AlM211_5-OpM110_5*s->l[3][11]+OpM311_5*s->l[1][11]);
  FB311_5 = s->m[11]*(AlM311_5+OpM110_5*s->l[2][11]-OpM211_5*s->l[1][11]);
  FM111_5 = FB111_5+FM112_5*C12+FM312_5*S12;
  FM211_5 = FB211_5+FB212_5+FB213_5+FB214_5;
  FM311_5 = FB311_5-FM112_5*S12+FM312_5*C12;
  CM111_5 = s->In[1][11]*OpM110_5+s->In[2][11]*OpM211_5+s->In[3][11]*OpM311_5+CM112_5*C12+CM312_5*S12-FB211_5*
 s->l[3][11]+FB311_5*s->l[2][11];
  CM211_5 = CM212_5+s->In[2][11]*OpM110_5+s->In[5][11]*OpM211_5+FB111_5*s->l[3][11]-FB311_5*s->l[1][11];
  CM311_5 = s->In[3][11]*OpM110_5+s->In[9][11]*OpM311_5-CM112_5*S12+CM312_5*C12-FB111_5*s->l[2][11]+FB211_5*s->l[1][11];
  FB111_6 = s->m[11]*(AlM111_6+OpM211_6*s->l[3][11]-OpM311_6*s->l[2][11]);
  FB211_6 = s->m[11]*(AlM211_6-OpM110_6*s->l[3][11]+OpM311_6*s->l[1][11]);
  FB311_6 = s->m[11]*(AlM311_6+OpM110_6*s->l[2][11]-OpM211_6*s->l[1][11]);
  FM111_6 = FB111_6+FM112_6*C12+FM312_6*S12;
  FM211_6 = FB211_6+FB212_6+FB213_6+FB214_6;
  FM311_6 = FB311_6-FM112_6*S12+FM312_6*C12;
  CM111_6 = s->In[1][11]*OpM110_6+s->In[2][11]*OpM211_6+s->In[3][11]*OpM311_6+CM112_6*C12+CM312_6*S12-FB211_6*
 s->l[3][11]+FB311_6*s->l[2][11];
  CM211_6 = CM212_6+s->In[2][11]*OpM110_6+s->In[5][11]*OpM211_6+FB111_6*s->l[3][11]-FB311_6*s->l[1][11];
  CM311_6 = s->In[3][11]*OpM110_6+s->In[9][11]*OpM311_6-CM112_6*S12+CM312_6*C12-FB111_6*s->l[2][11]+FB211_6*s->l[1][11];
  FB111_7 = s->m[11]*(AlM111_7+OpM211_7*s->l[3][11]-OpM311_7*s->l[2][11]);
  FB211_7 = s->m[11]*(AlM211_7-OpM110_7*s->l[3][11]+OpM311_7*s->l[1][11]);
  FB311_7 = s->m[11]*(AlM311_7+OpM110_7*s->l[2][11]-OpM211_7*s->l[1][11]);
  FM111_7 = FB111_7+FM112_7*C12+FM312_7*S12;
  FM211_7 = FB211_7+FB212_7+FB213_7+FB214_7;
  FM311_7 = FB311_7-FM112_7*S12+FM312_7*C12;
  CM111_7 = s->In[1][11]*OpM110_7+s->In[2][11]*OpM211_7+s->In[3][11]*OpM311_7+CM112_7*C12+CM312_7*S12-FB211_7*
 s->l[3][11]+FB311_7*s->l[2][11];
  CM211_7 = CM212_7+s->In[2][11]*OpM110_7+s->In[5][11]*OpM211_7+FB111_7*s->l[3][11]-FB311_7*s->l[1][11];
  CM311_7 = s->In[3][11]*OpM110_7+s->In[9][11]*OpM311_7-CM112_7*S12+CM312_7*C12-FB111_7*s->l[2][11]+FB211_7*s->l[1][11];
  FB111_8 = s->m[11]*(AlM111_8+OpM211_8*s->l[3][11]-OpM311_8*s->l[2][11]);
  FB211_8 = s->m[11]*(AlM211_8-OpM110_8*s->l[3][11]+OpM311_8*s->l[1][11]);
  FB311_8 = s->m[11]*(AlM311_8+OpM110_8*s->l[2][11]-OpM211_8*s->l[1][11]);
  FM111_8 = FB111_8+FM112_8*C12+FM312_8*S12;
  FM211_8 = FB211_8+FB212_8+FB213_8+FB214_8;
  FM311_8 = FB311_8-FM112_8*S12+FM312_8*C12;
  CM111_8 = s->In[1][11]*OpM110_8+s->In[2][11]*OpM211_8+s->In[3][11]*OpM311_8+CM112_8*C12+CM312_8*S12-FB211_8*
 s->l[3][11]+FB311_8*s->l[2][11];
  CM211_8 = CM212_8+s->In[2][11]*OpM110_8+s->In[5][11]*OpM211_8+FB111_8*s->l[3][11]-FB311_8*s->l[1][11];
  CM311_8 = s->In[3][11]*OpM110_8+s->In[9][11]*OpM311_8-CM112_8*S12+CM312_8*C12-FB111_8*s->l[2][11]+FB211_8*s->l[1][11];
  FB111_9 = s->m[11]*(OpM211_9*s->l[3][11]-OpM311_9*s->l[2][11]);
  FB211_9 = s->m[11]*(AlM211_9+OpM311_9*s->l[1][11]+s->l[3][11]*S10);
  FB311_9 = s->m[11]*(AlM311_9-OpM211_9*s->l[1][11]-s->l[2][11]*S10);
  CM111_9 = CM112_9*C12+CM312_9*S12-s->In[1][11]*S10+s->In[2][11]*OpM211_9+s->In[3][11]*OpM311_9-FB211_9*s->l[3][11]+
 FB311_9*s->l[2][11];
  CM211_9 = CM212_9-s->In[2][11]*S10+s->In[5][11]*OpM211_9+FB111_9*s->l[3][11]-FB311_9*s->l[1][11];
  CM311_9 = -(s->In[3][11]*S10-s->In[9][11]*OpM311_9+CM112_9*S12-CM312_9*C12+FB111_9*s->l[2][11]-FB211_9*s->l[1][11]);
  FB111_10 = s->m[11]*(s->dpt[3][14]+s->l[2][11]*S11+s->l[3][11]*C11);
  FB211_10 = -s->m[11]*s->l[1][11]*S11;
  FB311_10 = -s->m[11]*s->l[1][11]*C11;
  CM111_10 = s->In[2][11]*C11-s->In[3][11]*S11+CM112_10*C12+CM312_10*S12-FB211_10*s->l[3][11]+FB311_10*s->l[2][11];
  CM111_11 = s->In[1][11]+s->m[11]*s->l[2][11]*s->l[2][11]+s->m[11]*s->l[3][11]*s->l[3][11]+C12*(s->In[1][13]*C12-Dz133*
 FB213_11-Dz143*FB214_11-FB212_11*s->l[3][12]+C12*(s->In[1][12]+s->In[1][14]))+S12*(s->In[9][13]*S12+FB212_11*s->l[1][12]+
 FB213_11*s->dpt[1][20]+FB214_11*s->dpt[1][21]+S12*(s->In[9][12]+s->In[9][14]));
  FA110 = -(s->frc[1][10]-s->m[10]*(AlF110+BeF310*s->l[3][10]-s->l[1][10]*(OM210*OM210+OM310*OM310)+s->l[2][10]*(BS210-
 OpF310)));
  FA210 = -(s->frc[2][10]-s->m[10]*(AlF210+BeF610*s->l[3][10]+s->l[1][10]*(BS210+OpF310)-s->l[2][10]*(OM110*OM110+OM310*
 OM310)));
  FA310 = -(s->frc[3][10]-s->m[10]*(AlF310+BS910*s->l[3][10]+s->l[1][10]*(BS310-OpF29)+s->l[2][10]*(BS610+OpF110)));
  FF110 = FA110+FF111;
  FF210 = FA210+FF211*C11-FF311*S11;
  FF310 = FA310+FF211*S11+FF311*C11;
  CF110 = -(s->trq[1][10]-CF111-s->In[1][10]*OpF110-s->In[2][10]*OpF29-s->In[3][10]*OpF310+FA210*s->l[3][10]-FA310*
 s->l[2][10]-OM210*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)+OM310*(s->In[2][10]*OM110+s->In[5][10]*OM210+
 s->In[6][10]*OM310)+s->dpt[3][14]*(FF211*C11-FF311*S11));
  CF210 = -(s->trq[2][10]-s->In[2][10]*OpF110-s->In[5][10]*OpF29-s->In[6][10]*OpF310-CF211*C11+CF311*S11-FA110*
 s->l[3][10]+FA310*s->l[1][10]-FF111*s->dpt[3][14]+OM110*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)-OM310*(
 s->In[1][10]*OM110+s->In[2][10]*OM210+s->In[3][10]*OM310));
  CF310 = -(s->trq[3][10]-s->In[3][10]*OpF110-s->In[6][10]*OpF29-s->In[9][10]*OpF310-CF211*S11-CF311*C11+FA110*
 s->l[2][10]-FA210*s->l[1][10]-OM110*(s->In[2][10]*OM110+s->In[5][10]*OM210+s->In[6][10]*OM310)+OM210*(s->In[1][10]*OM110+
 s->In[2][10]*OM210+s->In[3][10]*OM310));
  FB110_1 = s->m[10]*AlM110_1;
  FB210_1 = s->m[10]*AlM29_1;
  FB310_1 = s->m[10]*AlM310_1;
  FM110_1 = FB110_1+FM111_1;
  FM210_1 = FB210_1+FM211_1*C11-FM311_1*S11;
  FM310_1 = FB310_1+FM211_1*S11+FM311_1*C11;
  CM110_1 = CM111_1-FB210_1*s->l[3][10]+FB310_1*s->l[2][10]-s->dpt[3][14]*(FM211_1*C11-FM311_1*S11);
  CM210_1 = CM211_1*C11-CM311_1*S11+FB110_1*s->l[3][10]-FB310_1*s->l[1][10]+FM111_1*s->dpt[3][14];
  CM310_1 = CM211_1*S11+CM311_1*C11-FB110_1*s->l[2][10]+FB210_1*s->l[1][10];
  FB110_2 = s->m[10]*AlM110_2;
  FB210_2 = s->m[10]*AlM29_2;
  FB310_2 = s->m[10]*AlM310_2;
  FM110_2 = FB110_2+FM111_2;
  FM210_2 = FB210_2+FM211_2*C11-FM311_2*S11;
  FM310_2 = FB310_2+FM211_2*S11+FM311_2*C11;
  CM110_2 = CM111_2-FB210_2*s->l[3][10]+FB310_2*s->l[2][10]-s->dpt[3][14]*(FM211_2*C11-FM311_2*S11);
  CM210_2 = CM211_2*C11-CM311_2*S11+FB110_2*s->l[3][10]-FB310_2*s->l[1][10]+FM111_2*s->dpt[3][14];
  CM310_2 = CM211_2*S11+CM311_2*C11-FB110_2*s->l[2][10]+FB210_2*s->l[1][10];
  FB110_3 = s->m[10]*AlM110_3;
  FB210_3 = s->m[10]*AlM29_3;
  FB310_3 = s->m[10]*AlM310_3;
  FM110_3 = FB110_3+FM111_3;
  FM210_3 = FB210_3+FM211_3*C11-FM311_3*S11;
  FM310_3 = FB310_3+FM211_3*S11+FM311_3*C11;
  CM110_3 = CM111_3-FB210_3*s->l[3][10]+FB310_3*s->l[2][10]-s->dpt[3][14]*(FM211_3*C11-FM311_3*S11);
  CM210_3 = CM211_3*C11-CM311_3*S11+FB110_3*s->l[3][10]-FB310_3*s->l[1][10]+FM111_3*s->dpt[3][14];
  CM310_3 = CM211_3*S11+CM311_3*C11-FB110_3*s->l[2][10]+FB210_3*s->l[1][10];
  FB110_4 = s->m[10]*(AlM110_4+OpM29_4*s->l[3][10]-OpM310_4*s->l[2][10]);
  FB210_4 = s->m[10]*(AlM210_4-OpM110_4*s->l[3][10]+OpM310_4*s->l[1][10]);
  FB310_4 = s->m[10]*(AlM310_4+OpM110_4*s->l[2][10]-OpM29_4*s->l[1][10]);
  FM110_4 = FB110_4+FM111_4;
  FM210_4 = FB210_4+FM211_4*C11-FM311_4*S11;
  FM310_4 = FB310_4+FM211_4*S11+FM311_4*C11;
  CM110_4 = CM111_4+s->In[1][10]*OpM110_4+s->In[2][10]*OpM29_4+s->In[3][10]*OpM310_4-FB210_4*s->l[3][10]+FB310_4*
 s->l[2][10]-s->dpt[3][14]*(FM211_4*C11-FM311_4*S11);
  CM210_4 = s->In[2][10]*OpM110_4+s->In[5][10]*OpM29_4+s->In[6][10]*OpM310_4+CM211_4*C11-CM311_4*S11+FB110_4*s->l[3][10]
 -FB310_4*s->l[1][10]+FM111_4*s->dpt[3][14];
  CM310_4 = s->In[3][10]*OpM110_4+s->In[6][10]*OpM29_4+s->In[9][10]*OpM310_4+CM211_4*S11+CM311_4*C11-FB110_4*s->l[2][10]
 +FB210_4*s->l[1][10];
  FB110_5 = s->m[10]*(AlM110_5+OpM29_5*s->l[3][10]-OpM310_5*s->l[2][10]);
  FB210_5 = s->m[10]*(AlM210_5-OpM110_5*s->l[3][10]+OpM310_5*s->l[1][10]);
  FB310_5 = s->m[10]*(AlM310_5+OpM110_5*s->l[2][10]-OpM29_5*s->l[1][10]);
  FM110_5 = FB110_5+FM111_5;
  FM210_5 = FB210_5+FM211_5*C11-FM311_5*S11;
  FM310_5 = FB310_5+FM211_5*S11+FM311_5*C11;
  CM110_5 = CM111_5+s->In[1][10]*OpM110_5+s->In[2][10]*OpM29_5+s->In[3][10]*OpM310_5-FB210_5*s->l[3][10]+FB310_5*
 s->l[2][10]-s->dpt[3][14]*(FM211_5*C11-FM311_5*S11);
  CM210_5 = s->In[2][10]*OpM110_5+s->In[5][10]*OpM29_5+s->In[6][10]*OpM310_5+CM211_5*C11-CM311_5*S11+FB110_5*s->l[3][10]
 -FB310_5*s->l[1][10]+FM111_5*s->dpt[3][14];
  CM310_5 = s->In[3][10]*OpM110_5+s->In[6][10]*OpM29_5+s->In[9][10]*OpM310_5+CM211_5*S11+CM311_5*C11-FB110_5*s->l[2][10]
 +FB210_5*s->l[1][10];
  FB110_6 = s->m[10]*(AlM110_6+OpM29_6*s->l[3][10]-OpM310_6*s->l[2][10]);
  FB210_6 = s->m[10]*(AlM210_6-OpM110_6*s->l[3][10]+OpM310_6*s->l[1][10]);
  FB310_6 = s->m[10]*(AlM310_6+OpM110_6*s->l[2][10]-OpM29_6*s->l[1][10]);
  FM110_6 = FB110_6+FM111_6;
  FM210_6 = FB210_6+FM211_6*C11-FM311_6*S11;
  FM310_6 = FB310_6+FM211_6*S11+FM311_6*C11;
  CM110_6 = CM111_6+s->In[1][10]*OpM110_6+s->In[2][10]*OpM29_6+s->In[3][10]*OpM310_6-FB210_6*s->l[3][10]+FB310_6*
 s->l[2][10]-s->dpt[3][14]*(FM211_6*C11-FM311_6*S11);
  CM210_6 = s->In[2][10]*OpM110_6+s->In[5][10]*OpM29_6+s->In[6][10]*OpM310_6+CM211_6*C11-CM311_6*S11+FB110_6*s->l[3][10]
 -FB310_6*s->l[1][10]+FM111_6*s->dpt[3][14];
  CM310_6 = s->In[3][10]*OpM110_6+s->In[6][10]*OpM29_6+s->In[9][10]*OpM310_6+CM211_6*S11+CM311_6*C11-FB110_6*s->l[2][10]
 +FB210_6*s->l[1][10];
  FB110_7 = s->m[10]*(AlM110_7+OpM29_7*s->l[3][10]-OpM310_7*s->l[2][10]);
  FB210_7 = s->m[10]*(AlM210_7-OpM110_7*s->l[3][10]+OpM310_7*s->l[1][10]);
  FB310_7 = s->m[10]*(AlM310_7+OpM110_7*s->l[2][10]-OpM29_7*s->l[1][10]);
  FM110_7 = FB110_7+FM111_7;
  FM210_7 = FB210_7+FM211_7*C11-FM311_7*S11;
  FM310_7 = FB310_7+FM211_7*S11+FM311_7*C11;
  CM110_7 = CM111_7+s->In[1][10]*OpM110_7+s->In[2][10]*OpM29_7+s->In[3][10]*OpM310_7-FB210_7*s->l[3][10]+FB310_7*
 s->l[2][10]-s->dpt[3][14]*(FM211_7*C11-FM311_7*S11);
  CM210_7 = s->In[2][10]*OpM110_7+s->In[5][10]*OpM29_7+s->In[6][10]*OpM310_7+CM211_7*C11-CM311_7*S11+FB110_7*s->l[3][10]
 -FB310_7*s->l[1][10]+FM111_7*s->dpt[3][14];
  CM310_7 = s->In[3][10]*OpM110_7+s->In[6][10]*OpM29_7+s->In[9][10]*OpM310_7+CM211_7*S11+CM311_7*C11-FB110_7*s->l[2][10]
 +FB210_7*s->l[1][10];
  FB110_8 = s->m[10]*(AlM110_8-OpM310_8*s->l[2][10]-s->l[3][10]*S9);
  FB210_8 = s->m[10]*(AlM210_8-OpM110_8*s->l[3][10]+OpM310_8*s->l[1][10]);
  FB310_8 = s->m[10]*(AlM310_8+OpM110_8*s->l[2][10]+s->l[1][10]*S9);
  FM110_8 = FB110_8+FM111_8;
  FM210_8 = FB210_8+FM211_8*C11-FM311_8*S11;
  FM310_8 = FB310_8+FM211_8*S11+FM311_8*C11;
  CM110_8 = CM111_8+s->In[1][10]*OpM110_8-s->In[2][10]*S9+s->In[3][10]*OpM310_8-FB210_8*s->l[3][10]+FB310_8*s->l[2][10]-
 s->dpt[3][14]*(FM211_8*C11-FM311_8*S11);
  CM210_8 = s->In[2][10]*OpM110_8-s->In[5][10]*S9+s->In[6][10]*OpM310_8+CM211_8*C11-CM311_8*S11+FB110_8*s->l[3][10]-
 FB310_8*s->l[1][10]+FM111_8*s->dpt[3][14];
  CM310_8 = s->In[3][10]*OpM110_8-s->In[6][10]*S9+s->In[9][10]*OpM310_8+CM211_8*S11+CM311_8*C11-FB110_8*s->l[2][10]+
 FB210_8*s->l[1][10];
  FB110_9 = -s->m[10]*s->l[2][10]*C10;
  FB210_9 = s->m[10]*(s->l[1][10]*C10+s->l[3][10]*S10);
  FB310_9 = -s->m[10]*s->l[2][10]*S10;
  CM210_9 = CM211_9*C11-CM311_9*S11+s->dpt[3][14]*(FB111_9+FM112_9*C12+FM312_9*S12)-s->In[2][10]*S10+s->In[6][10]*C10+
 FB110_9*s->l[3][10]-FB310_9*s->l[1][10];
  CM210_10 = s->In[5][10]+s->m[10]*s->l[1][10]*s->l[1][10]+s->m[10]*s->l[3][10]*s->l[3][10]+s->dpt[3][14]*(FB111_10+C12*
 (FB112_10+FB113_10+FB114_10)+S12*(FB312_10+FB313_10+FB314_10))+C11*(CM212_10+s->In[5][11]*C11+FB111_10*s->l[3][11]-FB311_10*
 s->l[1][11])+S11*(s->In[9][11]*S11+CM112_10*S12-CM312_10*C12+FB111_10*s->l[2][11]-FB211_10*s->l[1][11]);
  FA19 = -(s->frc[1][9]-s->m[9]*(AlF19+BeF39*s->l[3][9]-s->l[1][9]*(OM29*OM29+OM39*OM39)+s->l[2][9]*(BS29-OpF38)));
  FA29 = -(s->frc[2][9]-s->m[9]*(AlF29+BeF69*s->l[3][9]+s->l[1][9]*(BS29+OpF38)-s->l[2][9]*(OM19*OM19+OM39*OM39)));
  FA39 = -(s->frc[3][9]-s->m[9]*(AlF39+BS99*s->l[3][9]+s->l[1][9]*(BS39-OpF29)+s->l[2][9]*(BS69+OpF19)));
  FF19 = FA19+FF110*C10+FF310*S10;
  FF29 = FA29+FF210;
  CF19 = -(s->trq[1][9]-s->In[1][9]*OpF19-s->In[2][9]*OpF29-s->In[3][9]*OpF38-CF110*C10-CF310*S10+FA29*s->l[3][9]-FA39*
 s->l[2][9]+FF210*s->dpt[3][11]-OM29*(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)+OM39*(s->In[2][9]*OM19+s->In[5][9]*
 OM29+s->In[6][9]*OM39));
  CF29 = -(s->trq[2][9]-CF210-s->In[2][9]*OpF19-s->In[5][9]*OpF29-s->In[6][9]*OpF38-FA19*s->l[3][9]+FA39*s->l[1][9]+OM19
 *(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)-OM39*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*OM39)-
 s->dpt[3][11]*(FF110*C10+FF310*S10));
  CF39 = -(s->trq[3][9]-s->In[3][9]*OpF19-s->In[6][9]*OpF29-s->In[9][9]*OpF38+CF110*S10-CF310*C10+FA19*s->l[2][9]-FA29*
 s->l[1][9]-OM19*(s->In[2][9]*OM19+s->In[5][9]*OM29+s->In[6][9]*OM39)+OM29*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*
 OM39));
  FB19_1 = s->m[9]*AlM19_1;
  FB29_1 = s->m[9]*AlM29_1;
  FB39_1 = s->m[9]*AlM38_1;
  FM19_1 = FB19_1+FM110_1*C10+FM310_1*S10;
  FM29_1 = FB29_1+FM210_1;
  CM19_1 = CM110_1*C10+CM310_1*S10-FB29_1*s->l[3][9]+FB39_1*s->l[2][9]-FM210_1*s->dpt[3][11];
  CM29_1 = CM210_1+FB19_1*s->l[3][9]-FB39_1*s->l[1][9]+s->dpt[3][11]*(FM110_1*C10+FM310_1*S10);
  CM39_1 = -(CM110_1*S10-CM310_1*C10+FB19_1*s->l[2][9]-FB29_1*s->l[1][9]);
  FB19_2 = s->m[9]*AlM19_2;
  FB29_2 = s->m[9]*AlM29_2;
  FB39_2 = s->m[9]*AlM38_2;
  FM19_2 = FB19_2+FM110_2*C10+FM310_2*S10;
  FM29_2 = FB29_2+FM210_2;
  CM19_2 = CM110_2*C10+CM310_2*S10-FB29_2*s->l[3][9]+FB39_2*s->l[2][9]-FM210_2*s->dpt[3][11];
  CM29_2 = CM210_2+FB19_2*s->l[3][9]-FB39_2*s->l[1][9]+s->dpt[3][11]*(FM110_2*C10+FM310_2*S10);
  CM39_2 = -(CM110_2*S10-CM310_2*C10+FB19_2*s->l[2][9]-FB29_2*s->l[1][9]);
  FB19_3 = s->m[9]*AlM19_3;
  FB29_3 = s->m[9]*AlM29_3;
  FB39_3 = s->m[9]*AlM38_3;
  FM19_3 = FB19_3+FM110_3*C10+FM310_3*S10;
  FM29_3 = FB29_3+FM210_3;
  CM19_3 = CM110_3*C10+CM310_3*S10-FB29_3*s->l[3][9]+FB39_3*s->l[2][9]-FM210_3*s->dpt[3][11];
  CM29_3 = CM210_3+FB19_3*s->l[3][9]-FB39_3*s->l[1][9]+s->dpt[3][11]*(FM110_3*C10+FM310_3*S10);
  CM39_3 = -(CM110_3*S10-CM310_3*C10+FB19_3*s->l[2][9]-FB29_3*s->l[1][9]);
  FB19_4 = s->m[9]*(AlM19_4+OpM29_4*s->l[3][9]-OpM38_4*s->l[2][9]);
  FB29_4 = s->m[9]*(AlM29_4-OpM19_4*s->l[3][9]+OpM38_4*s->l[1][9]);
  FB39_4 = s->m[9]*(AlM38_4+OpM19_4*s->l[2][9]-OpM29_4*s->l[1][9]);
  FM19_4 = FB19_4+FM110_4*C10+FM310_4*S10;
  FM29_4 = FB29_4+FM210_4;
  CM19_4 = s->In[1][9]*OpM19_4+s->In[2][9]*OpM29_4+s->In[3][9]*OpM38_4+CM110_4*C10+CM310_4*S10-FB29_4*s->l[3][9]+FB39_4*
 s->l[2][9]-FM210_4*s->dpt[3][11];
  CM29_4 = CM210_4+s->In[2][9]*OpM19_4+s->In[5][9]*OpM29_4+s->In[6][9]*OpM38_4+FB19_4*s->l[3][9]-FB39_4*s->l[1][9]+
 s->dpt[3][11]*(FM110_4*C10+FM310_4*S10);
  CM39_4 = s->In[3][9]*OpM19_4+s->In[6][9]*OpM29_4+s->In[9][9]*OpM38_4-CM110_4*S10+CM310_4*C10-FB19_4*s->l[2][9]+FB29_4*
 s->l[1][9];
  FB19_5 = s->m[9]*(AlM19_5+OpM29_5*s->l[3][9]-OpM38_5*s->l[2][9]);
  FB29_5 = s->m[9]*(AlM29_5-OpM19_5*s->l[3][9]+OpM38_5*s->l[1][9]);
  FB39_5 = s->m[9]*(AlM38_5+OpM19_5*s->l[2][9]-OpM29_5*s->l[1][9]);
  FM19_5 = FB19_5+FM110_5*C10+FM310_5*S10;
  FM29_5 = FB29_5+FM210_5;
  CM19_5 = s->In[1][9]*OpM19_5+s->In[2][9]*OpM29_5+s->In[3][9]*OpM38_5+CM110_5*C10+CM310_5*S10-FB29_5*s->l[3][9]+FB39_5*
 s->l[2][9]-FM210_5*s->dpt[3][11];
  CM29_5 = CM210_5+s->In[2][9]*OpM19_5+s->In[5][9]*OpM29_5+s->In[6][9]*OpM38_5+FB19_5*s->l[3][9]-FB39_5*s->l[1][9]+
 s->dpt[3][11]*(FM110_5*C10+FM310_5*S10);
  CM39_5 = s->In[3][9]*OpM19_5+s->In[6][9]*OpM29_5+s->In[9][9]*OpM38_5-CM110_5*S10+CM310_5*C10-FB19_5*s->l[2][9]+FB29_5*
 s->l[1][9];
  FB19_6 = s->m[9]*(AlM19_6+OpM29_6*s->l[3][9]-OpM38_6*s->l[2][9]);
  FB29_6 = s->m[9]*(AlM29_6-OpM19_6*s->l[3][9]+OpM38_6*s->l[1][9]);
  FB39_6 = s->m[9]*(AlM38_6+OpM19_6*s->l[2][9]-OpM29_6*s->l[1][9]);
  FM19_6 = FB19_6+FM110_6*C10+FM310_6*S10;
  FM29_6 = FB29_6+FM210_6;
  CM19_6 = s->In[1][9]*OpM19_6+s->In[2][9]*OpM29_6+s->In[3][9]*OpM38_6+CM110_6*C10+CM310_6*S10-FB29_6*s->l[3][9]+FB39_6*
 s->l[2][9]-FM210_6*s->dpt[3][11];
  CM29_6 = CM210_6+s->In[2][9]*OpM19_6+s->In[5][9]*OpM29_6+s->In[6][9]*OpM38_6+FB19_6*s->l[3][9]-FB39_6*s->l[1][9]+
 s->dpt[3][11]*(FM110_6*C10+FM310_6*S10);
  CM39_6 = s->In[3][9]*OpM19_6+s->In[6][9]*OpM29_6+s->In[9][9]*OpM38_6-CM110_6*S10+CM310_6*C10-FB19_6*s->l[2][9]+FB29_6*
 s->l[1][9];
  FB19_7 = s->m[9]*(AlM19_7+OpM29_7*s->l[3][9]+s->l[2][9]*S8);
  FB29_7 = s->m[9]*(AlM29_7-OpM19_7*s->l[3][9]-s->l[1][9]*S8);
  FB39_7 = s->m[9]*(OpM19_7*s->l[2][9]-OpM29_7*s->l[1][9]);
  FM19_7 = FB19_7+FM110_7*C10+FM310_7*S10;
  FM29_7 = FB29_7+FM210_7;
  CM19_7 = s->In[1][9]*OpM19_7+s->In[2][9]*OpM29_7-s->In[3][9]*S8+CM110_7*C10+CM310_7*S10-FB29_7*s->l[3][9]+FB39_7*
 s->l[2][9]-FM210_7*s->dpt[3][11];
  CM29_7 = CM210_7+s->In[2][9]*OpM19_7+s->In[5][9]*OpM29_7-s->In[6][9]*S8+FB19_7*s->l[3][9]-FB39_7*s->l[1][9]+
 s->dpt[3][11]*(FM110_7*C10+FM310_7*S10);
  CM39_7 = s->In[3][9]*OpM19_7+s->In[6][9]*OpM29_7-s->In[9][9]*S8-CM110_7*S10+CM310_7*C10-FB19_7*s->l[2][9]+FB29_7*
 s->l[1][9];
  FB19_8 = s->m[9]*(AlM19_8-s->l[3][9]*S9);
  FB29_8 = s->m[9]*(AlM29_8-s->l[3][9]*C9);
  FB39_8 = s->m[9]*(s->l[1][9]*S9+s->l[2][9]*C9);
  CM39_8 = s->In[3][9]*C9-s->In[6][9]*S9-CM110_8*S10+CM310_8*C10-FB19_8*s->l[2][9]+FB29_8*s->l[1][9];
  CM39_9 = s->In[9][9]+s->m[9]*s->l[1][9]*s->l[1][9]+s->m[9]*s->l[2][9]*s->l[2][9]-C10*(s->In[3][10]*S10-s->In[9][10]*
 C10-CM211_9*S11-CM311_9*C11+FB110_9*s->l[2][10]-FB210_9*s->l[1][10])-S10*(CM111_9-s->In[1][10]*S10+s->In[3][10]*C10-FB210_9*
 s->l[3][10]+FB310_9*s->l[2][10]-s->dpt[3][14]*(C11*(FB211_9+FB212_9+FB213_9+FB214_9)-S11*(FB311_9-FM112_9*S12+FM312_9*C12)));
  FA18 = -(s->frc[1][8]-s->m[8]*(AlF18+BeF38*s->l[3][8]-s->l[1][8]*(OM28*OM28+OM38*OM38)+s->l[2][8]*(BS28-OpF38)));
  FA28 = -(s->frc[2][8]-s->m[8]*(AlF28+BeF68*s->l[3][8]+s->l[1][8]*(BS28+OpF38)-s->l[2][8]*(OM18*OM18+OM38*OM38)));
  FA38 = -(s->frc[3][8]-s->m[8]*(AlF38+BS98*s->l[3][8]+s->l[1][8]*(BS38-OpF28)+s->l[2][8]*(BS68+OpF17)));
  FF18 = FA18+FF19*C9-FF29*S9;
  FF28 = FA28+FF19*S9+FF29*C9;
  FF38 = FA38+FA39-FF110*S10+FF310*C10;
  CF18 = -(s->trq[1][8]-s->In[1][8]*OpF17-CF19*C9+CF29*S9+FA28*s->l[3][8]-FA38*s->l[2][8]-OM28*(s->In[6][8]*OM28+
 s->In[9][8]*OM38)+OM38*(s->In[5][8]*OM28+s->In[6][8]*OM38)+s->dpt[3][8]*(FF19*S9+FF29*C9));
  CF28 = -(s->trq[2][8]-s->In[5][8]*OpF28-s->In[6][8]*OpF38-CF19*S9-CF29*C9-FA18*s->l[3][8]+FA38*s->l[1][8]-OM18*(
 s->In[1][8]*OM38-s->In[6][8]*OM28-s->In[9][8]*OM38)-s->dpt[3][8]*(FF19*C9-FF29*S9));
  CF38 = -(s->trq[3][8]-CF39-s->In[6][8]*OpF28-s->In[9][8]*OpF38+FA18*s->l[2][8]-FA28*s->l[1][8]+OM18*(s->In[1][8]*OM28-
 s->In[5][8]*OM28-s->In[6][8]*OM38));
  FB18_1 = s->m[8]*AlM17_1;
  FB28_1 = s->m[8]*AlM28_1;
  FB38_1 = s->m[8]*AlM38_1;
  FM18_1 = FB18_1+FM19_1*C9-FM29_1*S9;
  FM28_1 = FB28_1+FM19_1*S9+FM29_1*C9;
  FM38_1 = FB38_1+FB39_1-FM110_1*S10+FM310_1*C10;
  CM18_1 = CM19_1*C9-CM29_1*S9-FB28_1*s->l[3][8]+FB38_1*s->l[2][8]-s->dpt[3][8]*(FM19_1*S9+FM29_1*C9);
  CM28_1 = CM19_1*S9+CM29_1*C9+FB18_1*s->l[3][8]-FB38_1*s->l[1][8]+s->dpt[3][8]*(FM19_1*C9-FM29_1*S9);
  CM38_1 = CM39_1-FB18_1*s->l[2][8]+FB28_1*s->l[1][8];
  FB18_2 = s->m[8]*AlM17_2;
  FB28_2 = s->m[8]*AlM28_2;
  FB38_2 = s->m[8]*AlM38_2;
  FM18_2 = FB18_2+FM19_2*C9-FM29_2*S9;
  FM28_2 = FB28_2+FM19_2*S9+FM29_2*C9;
  FM38_2 = FB38_2+FB39_2-FM110_2*S10+FM310_2*C10;
  CM18_2 = CM19_2*C9-CM29_2*S9-FB28_2*s->l[3][8]+FB38_2*s->l[2][8]-s->dpt[3][8]*(FM19_2*S9+FM29_2*C9);
  CM28_2 = CM19_2*S9+CM29_2*C9+FB18_2*s->l[3][8]-FB38_2*s->l[1][8]+s->dpt[3][8]*(FM19_2*C9-FM29_2*S9);
  CM38_2 = CM39_2-FB18_2*s->l[2][8]+FB28_2*s->l[1][8];
  FB18_3 = s->m[8]*AlM17_3;
  FB28_3 = s->m[8]*AlM28_3;
  FB38_3 = s->m[8]*AlM38_3;
  FM18_3 = FB18_3+FM19_3*C9-FM29_3*S9;
  FM28_3 = FB28_3+FM19_3*S9+FM29_3*C9;
  FM38_3 = FB38_3+FB39_3-FM110_3*S10+FM310_3*C10;
  CM18_3 = CM19_3*C9-CM29_3*S9-FB28_3*s->l[3][8]+FB38_3*s->l[2][8]-s->dpt[3][8]*(FM19_3*S9+FM29_3*C9);
  CM28_3 = CM19_3*S9+CM29_3*C9+FB18_3*s->l[3][8]-FB38_3*s->l[1][8]+s->dpt[3][8]*(FM19_3*C9-FM29_3*S9);
  CM38_3 = CM39_3-FB18_3*s->l[2][8]+FB28_3*s->l[1][8];
  FB18_4 = s->m[8]*(AlM18_4+OpM28_4*s->l[3][8]-OpM38_4*s->l[2][8]);
  FB28_4 = s->m[8]*(AlM28_4-OpM17_4*s->l[3][8]+OpM38_4*s->l[1][8]);
  FB38_4 = s->m[8]*(AlM38_4+OpM17_4*s->l[2][8]-OpM28_4*s->l[1][8]);
  FM18_4 = FB18_4+FM19_4*C9-FM29_4*S9;
  FM28_4 = FB28_4+FM19_4*S9+FM29_4*C9;
  FM38_4 = FB38_4+FB39_4-FM110_4*S10+FM310_4*C10;
  CM18_4 = s->In[1][8]*OpM17_4+CM19_4*C9-CM29_4*S9-FB28_4*s->l[3][8]+FB38_4*s->l[2][8]-s->dpt[3][8]*(FM19_4*S9+FM29_4*C9
 );
  CM28_4 = s->In[5][8]*OpM28_4+s->In[6][8]*OpM38_4+CM19_4*S9+CM29_4*C9+FB18_4*s->l[3][8]-FB38_4*s->l[1][8]+s->dpt[3][8]*
 (FM19_4*C9-FM29_4*S9);
  CM38_4 = CM39_4+s->In[6][8]*OpM28_4+s->In[9][8]*OpM38_4-FB18_4*s->l[2][8]+FB28_4*s->l[1][8];
  FB18_5 = s->m[8]*(AlM18_5+OpM28_5*s->l[3][8]-OpM38_5*s->l[2][8]);
  FB28_5 = s->m[8]*(AlM28_5-OpM17_5*s->l[3][8]+OpM38_5*s->l[1][8]);
  FB38_5 = s->m[8]*(AlM38_5+OpM17_5*s->l[2][8]-OpM28_5*s->l[1][8]);
  FM18_5 = FB18_5+FM19_5*C9-FM29_5*S9;
  FM28_5 = FB28_5+FM19_5*S9+FM29_5*C9;
  FM38_5 = FB38_5+FB39_5-FM110_5*S10+FM310_5*C10;
  CM18_5 = s->In[1][8]*OpM17_5+CM19_5*C9-CM29_5*S9-FB28_5*s->l[3][8]+FB38_5*s->l[2][8]-s->dpt[3][8]*(FM19_5*S9+FM29_5*C9
 );
  CM28_5 = s->In[5][8]*OpM28_5+s->In[6][8]*OpM38_5+CM19_5*S9+CM29_5*C9+FB18_5*s->l[3][8]-FB38_5*s->l[1][8]+s->dpt[3][8]*
 (FM19_5*C9-FM29_5*S9);
  CM38_5 = CM39_5+s->In[6][8]*OpM28_5+s->In[9][8]*OpM38_5-FB18_5*s->l[2][8]+FB28_5*s->l[1][8];
  FB18_6 = s->m[8]*(AlM18_6+OpM28_6*s->l[3][8]-OpM38_6*s->l[2][8]);
  FB28_6 = s->m[8]*(AlM28_6+OpM38_6*s->l[1][8]+s->l[3][8]*S7);
  FB38_6 = s->m[8]*(AlM38_6-OpM28_6*s->l[1][8]-s->l[2][8]*S7);
  FM18_6 = FB18_6+FM19_6*C9-FM29_6*S9;
  FM28_6 = FB28_6+FM19_6*S9+FM29_6*C9;
  FM38_6 = FB38_6+FB39_6-FM110_6*S10+FM310_6*C10;
  CM18_6 = CM19_6*C9-CM29_6*S9-s->dpt[3][8]*(FM19_6*S9+FM29_6*C9)-s->In[1][8]*S7-FB28_6*s->l[3][8]+FB38_6*s->l[2][8];
  CM28_6 = s->In[5][8]*OpM28_6+s->In[6][8]*OpM38_6+CM19_6*S9+CM29_6*C9+FB18_6*s->l[3][8]-FB38_6*s->l[1][8]+s->dpt[3][8]*
 (FM19_6*C9-FM29_6*S9);
  CM38_6 = CM39_6+s->In[6][8]*OpM28_6+s->In[9][8]*OpM38_6-FB18_6*s->l[2][8]+FB28_6*s->l[1][8];
  FB18_7 = s->m[8]*(s->l[2][8]*S8+s->l[3][8]*C8);
  FB28_7 = -s->m[8]*s->l[1][8]*S8;
  FB38_7 = -s->m[8]*s->l[1][8]*C8;
  CM18_7 = CM19_7*C9-CM29_7*S9-FB28_7*s->l[3][8]+FB38_7*s->l[2][8]-s->dpt[3][8]*(FM19_7*S9+FM29_7*C9);
  CM18_8 = s->In[1][8]+s->m[8]*s->l[2][8]*s->l[2][8]+s->m[8]*s->l[3][8]*s->l[3][8]-s->dpt[3][8]*(C9*(FB29_8+FM210_8)+S9*
 (FB19_8+FM110_8*C10+FM310_8*S10))+C9*(s->In[1][9]*C9-s->In[2][9]*S9+CM110_8*C10+CM310_8*S10-FB29_8*s->l[3][9]+FB39_8*
 s->l[2][9]-FM210_8*s->dpt[3][11])-S9*(CM210_8+s->In[2][9]*C9-s->In[5][9]*S9+FB19_8*s->l[3][9]-FB39_8*s->l[1][9]+
 s->dpt[3][11]*(FM110_8*C10+FM310_8*S10));
  FA17 = -(s->frc[1][7]-s->m[7]*(AlF17+BeF27*s->l[2][7]-s->l[1][7]*(OM27*OM27+OM37*OM37)+s->l[3][7]*(BS37+OpF26)));
  FA27 = -(s->frc[2][7]-s->m[7]*(AlF27+BS57*s->l[2][7]+s->l[1][7]*(BS27+OpF37)+s->l[3][7]*(BS67-OpF17)));
  FA37 = -(s->frc[3][7]-s->m[7]*(AlF37+BeF87*s->l[2][7]+s->l[1][7]*(BS37-OpF26)-s->l[3][7]*(OM17*OM17+OM27*OM27)));
  FF17 = FA17+FF18;
  FF37 = FA37+FF28*S8+FF38*C8;
  CF17 = -(s->trq[1][7]-CF18-s->In[1][7]*OpF17+FA27*s->l[3][7]-FA37*s->l[2][7]+OM27*OM37*(s->In[5][7]-s->In[9][7])-
 s->dpt[2][6]*(FF28*S8+FF38*C8));
  CF27 = -(s->trq[2][7]-s->In[5][7]*OpF26-CF28*C8+CF38*S8-FA17*s->l[3][7]+FA37*s->l[1][7]-OM17*OM37*(s->In[1][7]-
 s->In[9][7]));
  CF37 = -(s->trq[3][7]-s->In[9][7]*OpF37-CF28*S8-CF38*C8+FA17*s->l[2][7]-FA27*s->l[1][7]+FF18*s->dpt[2][6]+OM17*OM27*(
 s->In[1][7]-s->In[5][7]));
  FB17_1 = s->m[7]*AlM17_1;
  FB27_1 = s->m[7]*AlM26_1;
  FB37_1 = s->m[7]*AlM37_1;
  FM17_1 = FB17_1+FM18_1;
  FM37_1 = FB37_1+FM28_1*S8+FM38_1*C8;
  CM17_1 = CM18_1-FB27_1*s->l[3][7]+FB37_1*s->l[2][7]+s->dpt[2][6]*(FM28_1*S8+FM38_1*C8);
  CM27_1 = CM28_1*C8-CM38_1*S8+FB17_1*s->l[3][7]-FB37_1*s->l[1][7];
  CM37_1 = CM28_1*S8+CM38_1*C8-FB17_1*s->l[2][7]+FB27_1*s->l[1][7]-FM18_1*s->dpt[2][6];
  FB17_2 = s->m[7]*AlM17_2;
  FB27_2 = s->m[7]*AlM26_2;
  FB37_2 = s->m[7]*AlM37_2;
  FM17_2 = FB17_2+FM18_2;
  FM37_2 = FB37_2+FM28_2*S8+FM38_2*C8;
  CM17_2 = CM18_2-FB27_2*s->l[3][7]+FB37_2*s->l[2][7]+s->dpt[2][6]*(FM28_2*S8+FM38_2*C8);
  CM27_2 = CM28_2*C8-CM38_2*S8+FB17_2*s->l[3][7]-FB37_2*s->l[1][7];
  CM37_2 = CM28_2*S8+CM38_2*C8-FB17_2*s->l[2][7]+FB27_2*s->l[1][7]-FM18_2*s->dpt[2][6];
  FB17_3 = s->m[7]*AlM17_3;
  FB27_3 = s->m[7]*AlM26_3;
  FB37_3 = s->m[7]*AlM37_3;
  FM17_3 = FB17_3+FM18_3;
  FM37_3 = FB37_3+FM28_3*S8+FM38_3*C8;
  CM17_3 = CM18_3-FB27_3*s->l[3][7]+FB37_3*s->l[2][7]+s->dpt[2][6]*(FM28_3*S8+FM38_3*C8);
  CM27_3 = CM28_3*C8-CM38_3*S8+FB17_3*s->l[3][7]-FB37_3*s->l[1][7];
  CM37_3 = CM28_3*S8+CM38_3*C8-FB17_3*s->l[2][7]+FB27_3*s->l[1][7]-FM18_3*s->dpt[2][6];
  FB17_4 = s->m[7]*(AlM17_4+OpM26_4*s->l[3][7]-OpM37_4*s->l[2][7]);
  FB27_4 = -s->m[7]*(OpM17_4*s->l[3][7]-OpM37_4*s->l[1][7]);
  FB37_4 = s->m[7]*(AlM37_4+OpM17_4*s->l[2][7]-OpM26_4*s->l[1][7]);
  FM17_4 = FB17_4+FM18_4;
  FM37_4 = FB37_4+FM28_4*S8+FM38_4*C8;
  CM17_4 = CM18_4+s->In[1][7]*OpM17_4-FB27_4*s->l[3][7]+FB37_4*s->l[2][7]+s->dpt[2][6]*(FM28_4*S8+FM38_4*C8);
  CM27_4 = s->In[5][7]*OpM26_4+CM28_4*C8-CM38_4*S8+FB17_4*s->l[3][7]-FB37_4*s->l[1][7];
  CM37_4 = s->In[9][7]*OpM37_4+CM28_4*S8+CM38_4*C8-FB17_4*s->l[2][7]+FB27_4*s->l[1][7]-FM18_4*s->dpt[2][6];
  FB17_5 = s->m[7]*(AlM17_5-OpM37_5*s->l[2][7]+s->l[3][7]*C6);
  FB27_5 = -s->m[7]*(OpM17_5*s->l[3][7]-OpM37_5*s->l[1][7]);
  FB37_5 = s->m[7]*(AlM37_5+OpM17_5*s->l[2][7]-s->l[1][7]*C6);
  FM17_5 = FB17_5+FM18_5;
  FM37_5 = FB37_5+FM28_5*S8+FM38_5*C8;
  CM17_5 = CM18_5+s->In[1][7]*OpM17_5-FB27_5*s->l[3][7]+FB37_5*s->l[2][7]+s->dpt[2][6]*(FM28_5*S8+FM38_5*C8);
  CM27_5 = s->In[5][7]*C6+CM28_5*C8-CM38_5*S8+FB17_5*s->l[3][7]-FB37_5*s->l[1][7];
  CM37_5 = s->In[9][7]*OpM37_5+CM28_5*S8+CM38_5*C8-FB17_5*s->l[2][7]+FB27_5*s->l[1][7]-FM18_5*s->dpt[2][6];
  FB17_6 = s->m[7]*(AlM17_6-s->l[2][7]*C7);
  FB27_6 = s->m[7]*(s->l[1][7]*C7+s->l[3][7]*S7);
  FB37_6 = s->m[7]*(AlM37_6-s->l[2][7]*S7);
  CM27_6 = CM28_6*C8-CM38_6*S8+FB17_6*s->l[3][7]-FB37_6*s->l[1][7];
  CM27_7 = s->In[5][7]+s->m[7]*s->l[1][7]*s->l[1][7]+s->m[7]*s->l[3][7]*s->l[3][7]+C8*(s->In[5][8]*C8-s->In[6][8]*S8+
 CM19_7*S9+CM29_7*C9+FB18_7*s->l[3][8]-FB38_7*s->l[1][8]+s->dpt[3][8]*(FM19_7*C9-FM29_7*S9))-S8*(CM39_7+s->In[6][8]*C8-
 s->In[9][8]*S8-FB18_7*s->l[2][8]+FB28_7*s->l[1][8]);

// = = Block_0_2_0_2_0_5 = = 
 
// Backward Dynamics 

  FA120 = -(s->frc[1][20]-s->m[20]*(AlF120+BS120*s->l[1][20]+BeF320*s->l[3][20]));
  FA220 = -(s->frc[2][20]-s->m[20]*(AlF219+BeF420*s->l[1][20]+BeF620*s->l[3][20]));
  FA320 = -(s->frc[3][20]-s->m[20]*(AlF320+BS920*s->l[3][20]+BeF720*s->l[1][20]));
  FF120 = FA120+FA121+FA122;
  FF320 = FA320+FA321+FA322;
  CF120 = -(s->trq[1][20]+s->trq[1][21]+s->trq[1][22]-s->In[1][20]*OpF120-s->In[1][21]*OpF120-s->In[1][22]*OpF120+Dz213*
 FA221+Dz223*FA222+FA220*s->l[3][20]+OM220*OM320*(s->In[5][20]-s->In[9][20])+OM220*OM320*(s->In[5][21]-s->In[9][21])+OM220*
 OM320*(s->In[5][22]-s->In[9][22]));
  CF220 = -(s->trq[2][20]+s->trq[2][21]+s->trq[2][22]-s->In[5][20]*OpF219-s->In[5][21]*OpF219-s->In[5][22]*OpF219-Dz213*
 FA121-Dz223*FA122-FA120*s->l[3][20]+FA320*s->l[1][20]+FA321*s->dpt[1][38]+FA322*s->dpt[1][39]-OM120*OM320*(s->In[1][20]-
 s->In[9][20])-OM120*OM320*(s->In[1][21]-s->In[9][21])-OM120*OM320*(s->In[1][22]-s->In[9][22]));
  CF320 = -(s->trq[3][20]+s->trq[3][21]+s->trq[3][22]-s->In[9][20]*OpF320-s->In[9][21]*OpF320-s->In[9][22]*OpF320-FA220*
 s->l[1][20]-FA221*s->dpt[1][38]-FA222*s->dpt[1][39]+OM120*OM220*(s->In[1][20]-s->In[5][20])+OM120*OM220*(s->In[1][21]-
 s->In[5][21])+OM120*OM220*(s->In[1][22]-s->In[5][22]));
  FB120_1 = s->m[20]*AlM120_1;
  FB220_1 = s->m[20]*AlM219_1;
  FB320_1 = s->m[20]*AlM320_1;
  FM120_1 = FB120_1+FB121_1+FB122_1;
  FM320_1 = FB320_1+FB321_1+FB322_1;
  CM120_1 = -(Dz213*FB221_1+Dz223*FB222_1+FB220_1*s->l[3][20]);
  CM220_1 = Dz213*FB121_1+Dz223*FB122_1+FB120_1*s->l[3][20]-FB320_1*s->l[1][20]-FB321_1*s->dpt[1][38]-FB322_1*
 s->dpt[1][39];
  CM320_1 = FB220_1*s->l[1][20]+FB221_1*s->dpt[1][38]+FB222_1*s->dpt[1][39];
  FB120_2 = s->m[20]*AlM120_2;
  FB220_2 = s->m[20]*AlM219_2;
  FB320_2 = s->m[20]*AlM320_2;
  FM120_2 = FB120_2+FB121_2+FB122_2;
  FM320_2 = FB320_2+FB321_2+FB322_2;
  CM120_2 = -(Dz213*FB221_2+Dz223*FB222_2+FB220_2*s->l[3][20]);
  CM220_2 = Dz213*FB121_2+Dz223*FB122_2+FB120_2*s->l[3][20]-FB320_2*s->l[1][20]-FB321_2*s->dpt[1][38]-FB322_2*
 s->dpt[1][39];
  CM320_2 = FB220_2*s->l[1][20]+FB221_2*s->dpt[1][38]+FB222_2*s->dpt[1][39];
  FB120_3 = s->m[20]*AlM120_3;
  FB220_3 = s->m[20]*AlM219_3;
  FB320_3 = s->m[20]*AlM320_3;
  FM120_3 = FB120_3+FB121_3+FB122_3;
  FM320_3 = FB320_3+FB321_3+FB322_3;
  CM120_3 = -(Dz213*FB221_3+Dz223*FB222_3+FB220_3*s->l[3][20]);
  CM220_3 = Dz213*FB121_3+Dz223*FB122_3+FB120_3*s->l[3][20]-FB320_3*s->l[1][20]-FB321_3*s->dpt[1][38]-FB322_3*
 s->dpt[1][39];
  CM320_3 = FB220_3*s->l[1][20]+FB221_3*s->dpt[1][38]+FB222_3*s->dpt[1][39];
  FB120_4 = s->m[20]*(AlM120_4+OpM219_4*s->l[3][20]);
  FB220_4 = s->m[20]*(AlM219_4-OpM120_4*s->l[3][20]+OpM320_4*s->l[1][20]);
  FB320_4 = s->m[20]*(AlM320_4-OpM219_4*s->l[1][20]);
  FM120_4 = FB120_4+FB121_4+FB122_4;
  FM320_4 = FB320_4+FB321_4+FB322_4;
  CM120_4 = s->In[1][21]*OpM120_4-Dz213*FB221_4-Dz223*FB222_4-FB220_4*s->l[3][20]+OpM120_4*(s->In[1][20]+s->In[1][22]);
  CM220_4 = s->In[5][21]*OpM219_4+Dz213*FB121_4+Dz223*FB122_4+FB120_4*s->l[3][20]-FB320_4*s->l[1][20]-FB321_4*
 s->dpt[1][38]-FB322_4*s->dpt[1][39]+OpM219_4*(s->In[5][20]+s->In[5][22]);
  CM320_4 = s->In[9][21]*OpM320_4+FB220_4*s->l[1][20]+FB221_4*s->dpt[1][38]+FB222_4*s->dpt[1][39]+OpM320_4*(s->In[9][20]
 +s->In[9][22]);
  FB120_5 = s->m[20]*(AlM120_5+OpM219_5*s->l[3][20]);
  FB220_5 = s->m[20]*(AlM219_5-OpM120_5*s->l[3][20]+OpM320_5*s->l[1][20]);
  FB320_5 = s->m[20]*(AlM320_5-OpM219_5*s->l[1][20]);
  FM120_5 = FB120_5+FB121_5+FB122_5;
  FM320_5 = FB320_5+FB321_5+FB322_5;
  CM120_5 = s->In[1][21]*OpM120_5-Dz213*FB221_5-Dz223*FB222_5-FB220_5*s->l[3][20]+OpM120_5*(s->In[1][20]+s->In[1][22]);
  CM220_5 = s->In[5][21]*OpM219_5+Dz213*FB121_5+Dz223*FB122_5+FB120_5*s->l[3][20]-FB320_5*s->l[1][20]-FB321_5*
 s->dpt[1][38]-FB322_5*s->dpt[1][39]+OpM219_5*(s->In[5][20]+s->In[5][22]);
  CM320_5 = s->In[9][21]*OpM320_5+FB220_5*s->l[1][20]+FB221_5*s->dpt[1][38]+FB222_5*s->dpt[1][39]+OpM320_5*(s->In[9][20]
 +s->In[9][22]);
  FB120_6 = s->m[20]*(AlM120_6+OpM219_6*s->l[3][20]);
  FB220_6 = s->m[20]*(AlM219_6-OpM120_6*s->l[3][20]+OpM320_6*s->l[1][20]);
  FB320_6 = s->m[20]*(AlM320_6-OpM219_6*s->l[1][20]);
  FM120_6 = FB120_6+FB121_6+FB122_6;
  FM320_6 = FB320_6+FB321_6+FB322_6;
  CM120_6 = s->In[1][21]*OpM120_6-Dz213*FB221_6-Dz223*FB222_6-FB220_6*s->l[3][20]+OpM120_6*(s->In[1][20]+s->In[1][22]);
  CM220_6 = s->In[5][21]*OpM219_6+Dz213*FB121_6+Dz223*FB122_6+FB120_6*s->l[3][20]-FB320_6*s->l[1][20]-FB321_6*
 s->dpt[1][38]-FB322_6*s->dpt[1][39]+OpM219_6*(s->In[5][20]+s->In[5][22]);
  CM320_6 = s->In[9][21]*OpM320_6+FB220_6*s->l[1][20]+FB221_6*s->dpt[1][38]+FB222_6*s->dpt[1][39]+OpM320_6*(s->In[9][20]
 +s->In[9][22]);
  FB120_15 = s->m[20]*(AlM120_15+OpM219_15*s->l[3][20]);
  FB220_15 = s->m[20]*(AlM219_15-OpM120_15*s->l[3][20]+OpM320_15*s->l[1][20]);
  FB320_15 = s->m[20]*(AlM320_15-OpM219_15*s->l[1][20]);
  FM120_15 = FB120_15+FB121_15+FB122_15;
  FM320_15 = FB320_15+FB321_15+FB322_15;
  CM120_15 = s->In[1][21]*OpM120_15-Dz213*FB221_15-Dz223*FB222_15-FB220_15*s->l[3][20]+OpM120_15*(s->In[1][20]+
 s->In[1][22]);
  CM220_15 = s->In[5][21]*OpM219_15+Dz213*FB121_15+Dz223*FB122_15+FB120_15*s->l[3][20]-FB320_15*s->l[1][20]-FB321_15*
 s->dpt[1][38]-FB322_15*s->dpt[1][39]+OpM219_15*(s->In[5][20]+s->In[5][22]);
  CM320_15 = s->In[9][21]*OpM320_15+FB220_15*s->l[1][20]+FB221_15*s->dpt[1][38]+FB222_15*s->dpt[1][39]+OpM320_15*(
 s->In[9][20]+s->In[9][22]);
  FB120_16 = s->m[20]*(AlM120_16+OpM219_16*s->l[3][20]);
  FB220_16 = s->m[20]*(AlM219_16-OpM120_16*s->l[3][20]+OpM320_16*s->l[1][20]);
  FB320_16 = s->m[20]*(AlM320_16-OpM219_16*s->l[1][20]);
  FM120_16 = FB120_16+FB121_16+FB122_16;
  FM320_16 = FB320_16+FB321_16+FB322_16;
  CM120_16 = s->In[1][21]*OpM120_16-Dz213*FB221_16-Dz223*FB222_16-FB220_16*s->l[3][20]+OpM120_16*(s->In[1][20]+
 s->In[1][22]);
  CM220_16 = s->In[5][21]*OpM219_16+Dz213*FB121_16+Dz223*FB122_16+FB120_16*s->l[3][20]-FB320_16*s->l[1][20]-FB321_16*
 s->dpt[1][38]-FB322_16*s->dpt[1][39]+OpM219_16*(s->In[5][20]+s->In[5][22]);
  CM320_16 = s->In[9][21]*OpM320_16+FB220_16*s->l[1][20]+FB221_16*s->dpt[1][38]+FB222_16*s->dpt[1][39]+OpM320_16*(
 s->In[9][20]+s->In[9][22]);
  FB120_17 = s->m[20]*(AlM120_17+OpM219_17*s->l[3][20]);
  FB220_17 = s->m[20]*(AlM219_17-OpM120_17*s->l[3][20]+OpM320_17*s->l[1][20]);
  FB320_17 = s->m[20]*(AlM320_17-OpM219_17*s->l[1][20]);
  FM120_17 = FB120_17+FB121_17+FB122_17;
  FM320_17 = FB320_17+FB321_17+FB322_17;
  CM120_17 = s->In[1][21]*OpM120_17-Dz213*FB221_17-Dz223*FB222_17-FB220_17*s->l[3][20]+OpM120_17*(s->In[1][20]+
 s->In[1][22]);
  CM220_17 = s->In[5][21]*OpM219_17+Dz213*FB121_17+Dz223*FB122_17+FB120_17*s->l[3][20]-FB320_17*s->l[1][20]-FB321_17*
 s->dpt[1][38]-FB322_17*s->dpt[1][39]+OpM219_17*(s->In[5][20]+s->In[5][22]);
  CM320_17 = s->In[9][21]*OpM320_17+FB220_17*s->l[1][20]+FB221_17*s->dpt[1][38]+FB222_17*s->dpt[1][39]+OpM320_17*(
 s->In[9][20]+s->In[9][22]);
  FB120_18 = s->m[20]*(AlM120_18+s->l[3][20]*C19);
  FB220_18 = -s->m[20]*(OpM120_18*s->l[3][20]-OpM320_18*s->l[1][20]);
  FB320_18 = s->m[20]*(AlM320_18-s->l[1][20]*C19);
  CM120_18 = s->In[1][21]*OpM120_18-Dz213*FB221_18-Dz223*FB222_18-FB220_18*s->l[3][20]+OpM120_18*(s->In[1][20]+
 s->In[1][22]);
  CM220_18 = s->In[5][21]*C19+Dz213*FB121_18+Dz223*FB122_18+FB120_18*s->l[3][20]-FB320_18*s->l[1][20]-FB321_18*
 s->dpt[1][38]-FB322_18*s->dpt[1][39]+C19*(s->In[5][20]+s->In[5][22]);
  CM320_18 = s->In[9][21]*OpM320_18+FB220_18*s->l[1][20]+FB221_18*s->dpt[1][38]+FB222_18*s->dpt[1][39]+OpM320_18*(
 s->In[9][20]+s->In[9][22]);
  FB220_19 = s->m[20]*(s->l[1][20]*S20-s->l[3][20]*C20);
  CM220_20 = s->In[5][20]+s->In[5][21]+s->In[5][22]+s->m[20]*s->l[1][20]*s->l[1][20]+s->m[20]*s->l[3][20]*s->l[3][20]+
 s->m[21]*Dz213*Dz213+s->m[22]*Dz223*Dz223-FB321_20*s->dpt[1][38]-FB322_20*s->dpt[1][39];
  FA119 = -(s->frc[1][19]-s->m[19]*(AlF119-s->l[1][19]*(OM219*OM219+OM319*OM319)+s->l[2][19]*(BS219-OpF319)+s->l[3][19]*
 (BS319+OpF219)));
  FA219 = -(s->frc[2][19]-s->m[19]*(AlF219+s->l[1][19]*(BS219+OpF319)-s->l[2][19]*(OM119*OM119+OM319*OM319)+s->l[3][19]*
 (BS619-OpF118)));
  FA319 = -(s->frc[3][19]-s->m[19]*(AlF319+s->l[1][19]*(BS319-OpF219)+s->l[2][19]*(BS619+OpF118)-s->l[3][19]*(OM119*
 OM119+OM219*OM219)));
  FF119 = FA119+FF120*C20+FF320*S20;
  FF219 = FA219+FA220+FA221+FA222;
  FF319 = FA319-FF120*S20+FF320*C20;
  CF119 = -(s->trq[1][19]-s->In[1][19]*OpF118-s->In[3][19]*OpF319-CF120*C20-CF320*S20+FA219*s->l[3][19]-FA319*
 s->l[2][19]-OM219*(s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)+OM319*(s->In[5][19]*OM219+s->In[6][19]*OM319));
  CF219 = -(s->trq[2][19]-CF220-s->In[5][19]*OpF219-s->In[6][19]*OpF319-FA119*s->l[3][19]+FA319*s->l[1][19]+OM119*(
 s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)-OM319*(s->In[1][19]*OM119+s->In[3][19]*OM319));
  CF319 = -(s->trq[3][19]-s->In[3][19]*OpF118-s->In[6][19]*OpF219-s->In[9][19]*OpF319+CF120*S20-CF320*C20+FA119*
 s->l[2][19]-FA219*s->l[1][19]-OM119*(s->In[5][19]*OM219+s->In[6][19]*OM319)+OM219*(s->In[1][19]*OM119+s->In[3][19]*OM319));
  FB119_1 = s->m[19]*AlM118_1;
  FB219_1 = s->m[19]*AlM219_1;
  FB319_1 = s->m[19]*AlM319_1;
  FM119_1 = FB119_1+FM120_1*C20+FM320_1*S20;
  FM219_1 = FB219_1+FB220_1+FB221_1+FB222_1;
  FM319_1 = FB319_1-FM120_1*S20+FM320_1*C20;
  CM119_1 = CM120_1*C20+CM320_1*S20-FB219_1*s->l[3][19]+FB319_1*s->l[2][19];
  CM219_1 = CM220_1+FB119_1*s->l[3][19]-FB319_1*s->l[1][19];
  CM319_1 = -(CM120_1*S20-CM320_1*C20+FB119_1*s->l[2][19]-FB219_1*s->l[1][19]);
  FB119_2 = s->m[19]*AlM118_2;
  FB219_2 = s->m[19]*AlM219_2;
  FB319_2 = s->m[19]*AlM319_2;
  FM119_2 = FB119_2+FM120_2*C20+FM320_2*S20;
  FM219_2 = FB219_2+FB220_2+FB221_2+FB222_2;
  FM319_2 = FB319_2-FM120_2*S20+FM320_2*C20;
  CM119_2 = CM120_2*C20+CM320_2*S20-FB219_2*s->l[3][19]+FB319_2*s->l[2][19];
  CM219_2 = CM220_2+FB119_2*s->l[3][19]-FB319_2*s->l[1][19];
  CM319_2 = -(CM120_2*S20-CM320_2*C20+FB119_2*s->l[2][19]-FB219_2*s->l[1][19]);
  FB119_3 = s->m[19]*AlM118_3;
  FB219_3 = s->m[19]*AlM219_3;
  FB319_3 = s->m[19]*AlM319_3;
  FM119_3 = FB119_3+FM120_3*C20+FM320_3*S20;
  FM219_3 = FB219_3+FB220_3+FB221_3+FB222_3;
  FM319_3 = FB319_3-FM120_3*S20+FM320_3*C20;
  CM119_3 = CM120_3*C20+CM320_3*S20-FB219_3*s->l[3][19]+FB319_3*s->l[2][19];
  CM219_3 = CM220_3+FB119_3*s->l[3][19]-FB319_3*s->l[1][19];
  CM319_3 = -(CM120_3*S20-CM320_3*C20+FB119_3*s->l[2][19]-FB219_3*s->l[1][19]);
  FB119_4 = s->m[19]*(AlM119_4+OpM219_4*s->l[3][19]-OpM319_4*s->l[2][19]);
  FB219_4 = s->m[19]*(AlM219_4-OpM118_4*s->l[3][19]+OpM319_4*s->l[1][19]);
  FB319_4 = s->m[19]*(AlM319_4+OpM118_4*s->l[2][19]-OpM219_4*s->l[1][19]);
  FM119_4 = FB119_4+FM120_4*C20+FM320_4*S20;
  FM219_4 = FB219_4+FB220_4+FB221_4+FB222_4;
  FM319_4 = FB319_4-FM120_4*S20+FM320_4*C20;
  CM119_4 = s->In[1][19]*OpM118_4+s->In[3][19]*OpM319_4+CM120_4*C20+CM320_4*S20-FB219_4*s->l[3][19]+FB319_4*s->l[2][19];
  CM219_4 = CM220_4+s->In[5][19]*OpM219_4+s->In[6][19]*OpM319_4+FB119_4*s->l[3][19]-FB319_4*s->l[1][19];
  CM319_4 = s->In[3][19]*OpM118_4+s->In[6][19]*OpM219_4+s->In[9][19]*OpM319_4-CM120_4*S20+CM320_4*C20-FB119_4*
 s->l[2][19]+FB219_4*s->l[1][19];
  FB119_5 = s->m[19]*(AlM119_5+OpM219_5*s->l[3][19]-OpM319_5*s->l[2][19]);
  FB219_5 = s->m[19]*(AlM219_5-OpM118_5*s->l[3][19]+OpM319_5*s->l[1][19]);
  FB319_5 = s->m[19]*(AlM319_5+OpM118_5*s->l[2][19]-OpM219_5*s->l[1][19]);
  FM119_5 = FB119_5+FM120_5*C20+FM320_5*S20;
  FM219_5 = FB219_5+FB220_5+FB221_5+FB222_5;
  FM319_5 = FB319_5-FM120_5*S20+FM320_5*C20;
  CM119_5 = s->In[1][19]*OpM118_5+s->In[3][19]*OpM319_5+CM120_5*C20+CM320_5*S20-FB219_5*s->l[3][19]+FB319_5*s->l[2][19];
  CM219_5 = CM220_5+s->In[5][19]*OpM219_5+s->In[6][19]*OpM319_5+FB119_5*s->l[3][19]-FB319_5*s->l[1][19];
  CM319_5 = s->In[3][19]*OpM118_5+s->In[6][19]*OpM219_5+s->In[9][19]*OpM319_5-CM120_5*S20+CM320_5*C20-FB119_5*
 s->l[2][19]+FB219_5*s->l[1][19];
  FB119_6 = s->m[19]*(AlM119_6+OpM219_6*s->l[3][19]-OpM319_6*s->l[2][19]);
  FB219_6 = s->m[19]*(AlM219_6-OpM118_6*s->l[3][19]+OpM319_6*s->l[1][19]);
  FB319_6 = s->m[19]*(AlM319_6+OpM118_6*s->l[2][19]-OpM219_6*s->l[1][19]);
  FM119_6 = FB119_6+FM120_6*C20+FM320_6*S20;
  FM219_6 = FB219_6+FB220_6+FB221_6+FB222_6;
  FM319_6 = FB319_6-FM120_6*S20+FM320_6*C20;
  CM119_6 = s->In[1][19]*OpM118_6+s->In[3][19]*OpM319_6+CM120_6*C20+CM320_6*S20-FB219_6*s->l[3][19]+FB319_6*s->l[2][19];
  CM219_6 = CM220_6+s->In[5][19]*OpM219_6+s->In[6][19]*OpM319_6+FB119_6*s->l[3][19]-FB319_6*s->l[1][19];
  CM319_6 = s->In[3][19]*OpM118_6+s->In[6][19]*OpM219_6+s->In[9][19]*OpM319_6-CM120_6*S20+CM320_6*C20-FB119_6*
 s->l[2][19]+FB219_6*s->l[1][19];
  FB119_15 = s->m[19]*(AlM119_15+OpM219_15*s->l[3][19]-OpM319_15*s->l[2][19]);
  FB219_15 = s->m[19]*(AlM219_15-OpM118_15*s->l[3][19]+OpM319_15*s->l[1][19]);
  FB319_15 = s->m[19]*(AlM319_15+OpM118_15*s->l[2][19]-OpM219_15*s->l[1][19]);
  FM119_15 = FB119_15+FM120_15*C20+FM320_15*S20;
  FM219_15 = FB219_15+FB220_15+FB221_15+FB222_15;
  FM319_15 = FB319_15-FM120_15*S20+FM320_15*C20;
  CM119_15 = s->In[1][19]*OpM118_15+s->In[3][19]*OpM319_15+CM120_15*C20+CM320_15*S20-FB219_15*s->l[3][19]+FB319_15*
 s->l[2][19];
  CM219_15 = CM220_15+s->In[5][19]*OpM219_15+s->In[6][19]*OpM319_15+FB119_15*s->l[3][19]-FB319_15*s->l[1][19];
  CM319_15 = s->In[3][19]*OpM118_15+s->In[6][19]*OpM219_15+s->In[9][19]*OpM319_15-CM120_15*S20+CM320_15*C20-FB119_15*
 s->l[2][19]+FB219_15*s->l[1][19];
  FB119_16 = s->m[19]*(AlM119_16+OpM219_16*s->l[3][19]-OpM319_16*s->l[2][19]);
  FB219_16 = s->m[19]*(AlM219_16-OpM118_16*s->l[3][19]+OpM319_16*s->l[1][19]);
  FB319_16 = s->m[19]*(AlM319_16+OpM118_16*s->l[2][19]-OpM219_16*s->l[1][19]);
  FM119_16 = FB119_16+FM120_16*C20+FM320_16*S20;
  FM219_16 = FB219_16+FB220_16+FB221_16+FB222_16;
  FM319_16 = FB319_16-FM120_16*S20+FM320_16*C20;
  CM119_16 = s->In[1][19]*OpM118_16+s->In[3][19]*OpM319_16+CM120_16*C20+CM320_16*S20-FB219_16*s->l[3][19]+FB319_16*
 s->l[2][19];
  CM219_16 = CM220_16+s->In[5][19]*OpM219_16+s->In[6][19]*OpM319_16+FB119_16*s->l[3][19]-FB319_16*s->l[1][19];
  CM319_16 = s->In[3][19]*OpM118_16+s->In[6][19]*OpM219_16+s->In[9][19]*OpM319_16-CM120_16*S20+CM320_16*C20-FB119_16*
 s->l[2][19]+FB219_16*s->l[1][19];
  FB119_17 = s->m[19]*(OpM219_17*s->l[3][19]-OpM319_17*s->l[2][19]);
  FB219_17 = s->m[19]*(AlM219_17+OpM319_17*s->l[1][19]+s->l[3][19]*S18);
  FB319_17 = s->m[19]*(AlM319_17-OpM219_17*s->l[1][19]-s->l[2][19]*S18);
  CM119_17 = CM120_17*C20+CM320_17*S20-s->In[1][19]*S18+s->In[3][19]*OpM319_17-FB219_17*s->l[3][19]+FB319_17*s->l[2][19];
  CM219_17 = CM220_17+s->In[5][19]*OpM219_17+s->In[6][19]*OpM319_17+FB119_17*s->l[3][19]-FB319_17*s->l[1][19];
  CM319_17 = -(s->In[3][19]*S18-s->In[6][19]*OpM219_17-s->In[9][19]*OpM319_17+CM120_17*S20-CM320_17*C20+FB119_17*
 s->l[2][19]-FB219_17*s->l[1][19]);
  FB119_18 = s->m[19]*(s->dpt[3][32]+s->l[2][19]*S19+s->l[3][19]*C19);
  FB219_18 = -s->m[19]*s->l[1][19]*S19;
  FB319_18 = -s->m[19]*s->l[1][19]*C19;
  CM119_18 = CM120_18*C20+CM320_18*S20-s->In[3][19]*S19-FB219_18*s->l[3][19]+FB319_18*s->l[2][19];
  CM119_19 = s->In[1][19]+s->m[19]*s->l[2][19]*s->l[2][19]+s->m[19]*s->l[3][19]*s->l[3][19]+C20*(s->In[1][21]*C20-Dz213*
 FB221_19-Dz223*FB222_19-FB220_19*s->l[3][20]+C20*(s->In[1][20]+s->In[1][22]))+S20*(s->In[9][21]*S20+FB220_19*s->l[1][20]+
 FB221_19*s->dpt[1][38]+FB222_19*s->dpt[1][39]+S20*(s->In[9][20]+s->In[9][22]));
  FA118 = -(s->frc[1][18]-s->m[18]*(AlF118+BeF318*s->l[3][18]-s->l[1][18]*(OM218*OM218+OM318*OM318)+s->l[2][18]*(BS218-
 OpF318)));
  FA218 = -(s->frc[2][18]-s->m[18]*(AlF218+BeF618*s->l[3][18]+s->l[1][18]*(BS218+OpF318)-s->l[2][18]*(OM118*OM118+OM318*
 OM318)));
  FA318 = -(s->frc[3][18]-s->m[18]*(AlF318+BS918*s->l[3][18]+s->l[1][18]*(BS318-OpF217)+s->l[2][18]*(BS618+OpF118)));
  FF118 = FA118+FF119;
  FF218 = FA218+FF219*C19-FF319*S19;
  FF318 = FA318+FF219*S19+FF319*C19;
  CF118 = -(s->trq[1][18]-CF119-s->In[1][18]*OpF118-s->In[3][18]*OpF318+FA218*s->l[3][18]-FA318*s->l[2][18]-OM218*(
 s->In[3][18]*OM118-s->In[5][18]*OM318+s->In[9][18]*OM318)+s->dpt[3][32]*(FF219*C19-FF319*S19));
  CF218 = -(s->trq[2][18]-s->In[5][18]*OpF217-CF219*C19+CF319*S19-FA118*s->l[3][18]+FA318*s->l[1][18]-FF119*
 s->dpt[3][32]+OM118*(s->In[3][18]*OM118+s->In[9][18]*OM318)-OM318*(s->In[1][18]*OM118+s->In[3][18]*OM318));
  CF318 = -(s->trq[3][18]-s->In[3][18]*OpF118-s->In[9][18]*OpF318-CF219*S19-CF319*C19+FA118*s->l[2][18]-FA218*
 s->l[1][18]+OM218*(s->In[1][18]*OM118+s->In[3][18]*OM318-s->In[5][18]*OM118));
  FB118_1 = s->m[18]*AlM118_1;
  FB218_1 = s->m[18]*AlM217_1;
  FB318_1 = s->m[18]*AlM318_1;
  FM118_1 = FB118_1+FM119_1;
  FM218_1 = FB218_1+FM219_1*C19-FM319_1*S19;
  FM318_1 = FB318_1+FM219_1*S19+FM319_1*C19;
  CM118_1 = CM119_1-FB218_1*s->l[3][18]+FB318_1*s->l[2][18]-s->dpt[3][32]*(FM219_1*C19-FM319_1*S19);
  CM218_1 = CM219_1*C19-CM319_1*S19+FB118_1*s->l[3][18]-FB318_1*s->l[1][18]+FM119_1*s->dpt[3][32];
  CM318_1 = CM219_1*S19+CM319_1*C19-FB118_1*s->l[2][18]+FB218_1*s->l[1][18];
  FB118_2 = s->m[18]*AlM118_2;
  FB218_2 = s->m[18]*AlM217_2;
  FB318_2 = s->m[18]*AlM318_2;
  FM118_2 = FB118_2+FM119_2;
  FM218_2 = FB218_2+FM219_2*C19-FM319_2*S19;
  FM318_2 = FB318_2+FM219_2*S19+FM319_2*C19;
  CM118_2 = CM119_2-FB218_2*s->l[3][18]+FB318_2*s->l[2][18]-s->dpt[3][32]*(FM219_2*C19-FM319_2*S19);
  CM218_2 = CM219_2*C19-CM319_2*S19+FB118_2*s->l[3][18]-FB318_2*s->l[1][18]+FM119_2*s->dpt[3][32];
  CM318_2 = CM219_2*S19+CM319_2*C19-FB118_2*s->l[2][18]+FB218_2*s->l[1][18];
  FB118_3 = s->m[18]*AlM118_3;
  FB218_3 = s->m[18]*AlM217_3;
  FB318_3 = s->m[18]*AlM318_3;
  FM118_3 = FB118_3+FM119_3;
  FM218_3 = FB218_3+FM219_3*C19-FM319_3*S19;
  FM318_3 = FB318_3+FM219_3*S19+FM319_3*C19;
  CM118_3 = CM119_3-FB218_3*s->l[3][18]+FB318_3*s->l[2][18]-s->dpt[3][32]*(FM219_3*C19-FM319_3*S19);
  CM218_3 = CM219_3*C19-CM319_3*S19+FB118_3*s->l[3][18]-FB318_3*s->l[1][18]+FM119_3*s->dpt[3][32];
  CM318_3 = CM219_3*S19+CM319_3*C19-FB118_3*s->l[2][18]+FB218_3*s->l[1][18];
  FB118_4 = s->m[18]*(AlM118_4+OpM217_4*s->l[3][18]-OpM318_4*s->l[2][18]);
  FB218_4 = s->m[18]*(AlM218_4-OpM118_4*s->l[3][18]+OpM318_4*s->l[1][18]);
  FB318_4 = s->m[18]*(AlM318_4+OpM118_4*s->l[2][18]-OpM217_4*s->l[1][18]);
  FM118_4 = FB118_4+FM119_4;
  FM218_4 = FB218_4+FM219_4*C19-FM319_4*S19;
  FM318_4 = FB318_4+FM219_4*S19+FM319_4*C19;
  CM118_4 = CM119_4+s->In[1][18]*OpM118_4+s->In[3][18]*OpM318_4-FB218_4*s->l[3][18]+FB318_4*s->l[2][18]-s->dpt[3][32]*(
 FM219_4*C19-FM319_4*S19);
  CM218_4 = s->In[5][18]*OpM217_4+CM219_4*C19-CM319_4*S19+FB118_4*s->l[3][18]-FB318_4*s->l[1][18]+FM119_4*s->dpt[3][32];
  CM318_4 = s->In[3][18]*OpM118_4+s->In[9][18]*OpM318_4+CM219_4*S19+CM319_4*C19-FB118_4*s->l[2][18]+FB218_4*s->l[1][18];
  FB118_5 = s->m[18]*(AlM118_5+OpM217_5*s->l[3][18]-OpM318_5*s->l[2][18]);
  FB218_5 = s->m[18]*(AlM218_5-OpM118_5*s->l[3][18]+OpM318_5*s->l[1][18]);
  FB318_5 = s->m[18]*(AlM318_5+OpM118_5*s->l[2][18]-OpM217_5*s->l[1][18]);
  FM118_5 = FB118_5+FM119_5;
  FM218_5 = FB218_5+FM219_5*C19-FM319_5*S19;
  FM318_5 = FB318_5+FM219_5*S19+FM319_5*C19;
  CM118_5 = CM119_5+s->In[1][18]*OpM118_5+s->In[3][18]*OpM318_5-FB218_5*s->l[3][18]+FB318_5*s->l[2][18]-s->dpt[3][32]*(
 FM219_5*C19-FM319_5*S19);
  CM218_5 = s->In[5][18]*OpM217_5+CM219_5*C19-CM319_5*S19+FB118_5*s->l[3][18]-FB318_5*s->l[1][18]+FM119_5*s->dpt[3][32];
  CM318_5 = s->In[3][18]*OpM118_5+s->In[9][18]*OpM318_5+CM219_5*S19+CM319_5*C19-FB118_5*s->l[2][18]+FB218_5*s->l[1][18];
  FB118_6 = s->m[18]*(AlM118_6+OpM217_6*s->l[3][18]-OpM318_6*s->l[2][18]);
  FB218_6 = s->m[18]*(AlM218_6-OpM118_6*s->l[3][18]+OpM318_6*s->l[1][18]);
  FB318_6 = s->m[18]*(AlM318_6+OpM118_6*s->l[2][18]-OpM217_6*s->l[1][18]);
  FM118_6 = FB118_6+FM119_6;
  FM218_6 = FB218_6+FM219_6*C19-FM319_6*S19;
  FM318_6 = FB318_6+FM219_6*S19+FM319_6*C19;
  CM118_6 = CM119_6+s->In[1][18]*OpM118_6+s->In[3][18]*OpM318_6-FB218_6*s->l[3][18]+FB318_6*s->l[2][18]-s->dpt[3][32]*(
 FM219_6*C19-FM319_6*S19);
  CM218_6 = s->In[5][18]*OpM217_6+CM219_6*C19-CM319_6*S19+FB118_6*s->l[3][18]-FB318_6*s->l[1][18]+FM119_6*s->dpt[3][32];
  CM318_6 = s->In[3][18]*OpM118_6+s->In[9][18]*OpM318_6+CM219_6*S19+CM319_6*C19-FB118_6*s->l[2][18]+FB218_6*s->l[1][18];
  FB118_15 = s->m[18]*(AlM118_15+OpM217_15*s->l[3][18]-OpM318_15*s->l[2][18]);
  FB218_15 = s->m[18]*(AlM218_15-OpM118_15*s->l[3][18]+OpM318_15*s->l[1][18]);
  FB318_15 = s->m[18]*(AlM318_15+OpM118_15*s->l[2][18]-OpM217_15*s->l[1][18]);
  FM118_15 = FB118_15+FM119_15;
  FM218_15 = FB218_15+FM219_15*C19-FM319_15*S19;
  FM318_15 = FB318_15+FM219_15*S19+FM319_15*C19;
  CM118_15 = CM119_15+s->In[1][18]*OpM118_15+s->In[3][18]*OpM318_15-FB218_15*s->l[3][18]+FB318_15*s->l[2][18]-
 s->dpt[3][32]*(FM219_15*C19-FM319_15*S19);
  CM218_15 = s->In[5][18]*OpM217_15+CM219_15*C19-CM319_15*S19+FB118_15*s->l[3][18]-FB318_15*s->l[1][18]+FM119_15*
 s->dpt[3][32];
  CM318_15 = s->In[3][18]*OpM118_15+s->In[9][18]*OpM318_15+CM219_15*S19+CM319_15*C19-FB118_15*s->l[2][18]+FB218_15*
 s->l[1][18];
  FB118_16 = s->m[18]*(AlM118_16-OpM318_16*s->l[2][18]-s->l[3][18]*S17);
  FB218_16 = s->m[18]*(AlM218_16-OpM118_16*s->l[3][18]+OpM318_16*s->l[1][18]);
  FB318_16 = s->m[18]*(AlM318_16+OpM118_16*s->l[2][18]+s->l[1][18]*S17);
  FM118_16 = FB118_16+FM119_16;
  FM218_16 = FB218_16+FM219_16*C19-FM319_16*S19;
  FM318_16 = FB318_16+FM219_16*S19+FM319_16*C19;
  CM118_16 = CM119_16+s->In[1][18]*OpM118_16+s->In[3][18]*OpM318_16-FB218_16*s->l[3][18]+FB318_16*s->l[2][18]-
 s->dpt[3][32]*(FM219_16*C19-FM319_16*S19);
  CM218_16 = CM219_16*C19-CM319_16*S19+FM119_16*s->dpt[3][32]-s->In[5][18]*S17+FB118_16*s->l[3][18]-FB318_16*s->l[1][18];
  CM318_16 = s->In[3][18]*OpM118_16+s->In[9][18]*OpM318_16+CM219_16*S19+CM319_16*C19-FB118_16*s->l[2][18]+FB218_16*
 s->l[1][18];
  FB118_17 = -s->m[18]*s->l[2][18]*C18;
  FB218_17 = s->m[18]*(s->l[1][18]*C18+s->l[3][18]*S18);
  FB318_17 = -s->m[18]*s->l[2][18]*S18;
  CM218_17 = CM219_17*C19-CM319_17*S19+FB118_17*s->l[3][18]-FB318_17*s->l[1][18]+s->dpt[3][32]*(FB119_17+FM120_17*C20+
 FM320_17*S20);
  CM218_18 = s->In[5][18]+s->m[18]*s->l[1][18]*s->l[1][18]+s->m[18]*s->l[3][18]*s->l[3][18]+s->dpt[3][32]*(FB119_18+C20*
 (FB120_18+FB121_18+FB122_18)+S20*(FB320_18+FB321_18+FB322_18))+C19*(CM220_18+s->In[5][19]*C19-s->In[6][19]*S19+FB119_18*
 s->l[3][19]-FB319_18*s->l[1][19])-S19*(s->In[6][19]*C19-s->In[9][19]*S19-CM120_18*S20+CM320_18*C20-FB119_18*s->l[2][19]+
 FB219_18*s->l[1][19]);
  FA117 = -(s->frc[1][17]-s->m[17]*(AlF117+BeF317*s->l[3][17]-s->l[1][17]*(OM217*OM217+OM317*OM317)+s->l[2][17]*(BS217-
 OpF316)));
  FA217 = -(s->frc[2][17]-s->m[17]*(AlF217+BeF617*s->l[3][17]+s->l[1][17]*(BS217+OpF316)-s->l[2][17]*(OM117*OM117+OM317*
 OM317)));
  FA317 = -(s->frc[3][17]-s->m[17]*(AlF317+BS917*s->l[3][17]+s->l[1][17]*(BS317-OpF217)+s->l[2][17]*(BS617+OpF117)));
  FF117 = FA117+FF118*C18+FF318*S18;
  FF217 = FA217+FF218;
  CF117 = -(s->trq[1][17]-s->In[1][17]*OpF117-s->In[3][17]*OpF316-CF118*C18-CF318*S18+FA217*s->l[3][17]-FA317*
 s->l[2][17]+FF218*s->dpt[3][29]-OM217*(s->In[3][17]*OM117-s->In[5][17]*OM317+s->In[9][17]*OM317));
  CF217 = -(s->trq[2][17]-CF218-s->In[5][17]*OpF217-FA117*s->l[3][17]+FA317*s->l[1][17]+OM117*(s->In[3][17]*OM117+
 s->In[9][17]*OM317)-OM317*(s->In[1][17]*OM117+s->In[3][17]*OM317)-s->dpt[3][29]*(FF118*C18+FF318*S18));
  CF317 = -(s->trq[3][17]-s->In[3][17]*OpF117-s->In[9][17]*OpF316+CF118*S18-CF318*C18+FA117*s->l[2][17]-FA217*
 s->l[1][17]+OM217*(s->In[1][17]*OM117+s->In[3][17]*OM317-s->In[5][17]*OM117));
  FB117_1 = s->m[17]*AlM117_1;
  FB217_1 = s->m[17]*AlM217_1;
  FB317_1 = s->m[17]*AlM316_1;
  FM117_1 = FB117_1+FM118_1*C18+FM318_1*S18;
  FM217_1 = FB217_1+FM218_1;
  CM117_1 = CM118_1*C18+CM318_1*S18-FB217_1*s->l[3][17]+FB317_1*s->l[2][17]-FM218_1*s->dpt[3][29];
  CM217_1 = CM218_1+FB117_1*s->l[3][17]-FB317_1*s->l[1][17]+s->dpt[3][29]*(FM118_1*C18+FM318_1*S18);
  CM317_1 = -(CM118_1*S18-CM318_1*C18+FB117_1*s->l[2][17]-FB217_1*s->l[1][17]);
  FB117_2 = s->m[17]*AlM117_2;
  FB217_2 = s->m[17]*AlM217_2;
  FB317_2 = s->m[17]*AlM316_2;
  FM117_2 = FB117_2+FM118_2*C18+FM318_2*S18;
  FM217_2 = FB217_2+FM218_2;
  CM117_2 = CM118_2*C18+CM318_2*S18-FB217_2*s->l[3][17]+FB317_2*s->l[2][17]-FM218_2*s->dpt[3][29];
  CM217_2 = CM218_2+FB117_2*s->l[3][17]-FB317_2*s->l[1][17]+s->dpt[3][29]*(FM118_2*C18+FM318_2*S18);
  CM317_2 = -(CM118_2*S18-CM318_2*C18+FB117_2*s->l[2][17]-FB217_2*s->l[1][17]);
  FB117_3 = s->m[17]*AlM117_3;
  FB217_3 = s->m[17]*AlM217_3;
  FB317_3 = s->m[17]*AlM316_3;
  FM117_3 = FB117_3+FM118_3*C18+FM318_3*S18;
  FM217_3 = FB217_3+FM218_3;
  CM117_3 = CM118_3*C18+CM318_3*S18-FB217_3*s->l[3][17]+FB317_3*s->l[2][17]-FM218_3*s->dpt[3][29];
  CM217_3 = CM218_3+FB117_3*s->l[3][17]-FB317_3*s->l[1][17]+s->dpt[3][29]*(FM118_3*C18+FM318_3*S18);
  CM317_3 = -(CM118_3*S18-CM318_3*C18+FB117_3*s->l[2][17]-FB217_3*s->l[1][17]);
  FB117_4 = s->m[17]*(AlM117_4+OpM217_4*s->l[3][17]-OpM316_4*s->l[2][17]);
  FB217_4 = s->m[17]*(AlM217_4-OpM117_4*s->l[3][17]+OpM316_4*s->l[1][17]);
  FB317_4 = s->m[17]*(AlM316_4+OpM117_4*s->l[2][17]-OpM217_4*s->l[1][17]);
  FM117_4 = FB117_4+FM118_4*C18+FM318_4*S18;
  FM217_4 = FB217_4+FM218_4;
  CM117_4 = s->In[1][17]*OpM117_4+s->In[3][17]*OpM316_4+CM118_4*C18+CM318_4*S18-FB217_4*s->l[3][17]+FB317_4*s->l[2][17]-
 FM218_4*s->dpt[3][29];
  CM217_4 = CM218_4+s->In[5][17]*OpM217_4+FB117_4*s->l[3][17]-FB317_4*s->l[1][17]+s->dpt[3][29]*(FM118_4*C18+FM318_4*S18
 );
  CM317_4 = s->In[3][17]*OpM117_4+s->In[9][17]*OpM316_4-CM118_4*S18+CM318_4*C18-FB117_4*s->l[2][17]+FB217_4*s->l[1][17];
  FB117_5 = s->m[17]*(AlM117_5+OpM217_5*s->l[3][17]-OpM316_5*s->l[2][17]);
  FB217_5 = s->m[17]*(AlM217_5-OpM117_5*s->l[3][17]+OpM316_5*s->l[1][17]);
  FB317_5 = s->m[17]*(AlM316_5+OpM117_5*s->l[2][17]-OpM217_5*s->l[1][17]);
  FM117_5 = FB117_5+FM118_5*C18+FM318_5*S18;
  FM217_5 = FB217_5+FM218_5;
  CM117_5 = s->In[1][17]*OpM117_5+s->In[3][17]*OpM316_5+CM118_5*C18+CM318_5*S18-FB217_5*s->l[3][17]+FB317_5*s->l[2][17]-
 FM218_5*s->dpt[3][29];
  CM217_5 = CM218_5+s->In[5][17]*OpM217_5+FB117_5*s->l[3][17]-FB317_5*s->l[1][17]+s->dpt[3][29]*(FM118_5*C18+FM318_5*S18
 );
  CM317_5 = s->In[3][17]*OpM117_5+s->In[9][17]*OpM316_5-CM118_5*S18+CM318_5*C18-FB117_5*s->l[2][17]+FB217_5*s->l[1][17];
  FB117_6 = s->m[17]*(AlM117_6+OpM217_6*s->l[3][17]-OpM316_6*s->l[2][17]);
  FB217_6 = s->m[17]*(AlM217_6-OpM117_6*s->l[3][17]+OpM316_6*s->l[1][17]);
  FB317_6 = s->m[17]*(AlM316_6+OpM117_6*s->l[2][17]-OpM217_6*s->l[1][17]);
  FM117_6 = FB117_6+FM118_6*C18+FM318_6*S18;
  FM217_6 = FB217_6+FM218_6;
  CM117_6 = s->In[1][17]*OpM117_6+s->In[3][17]*OpM316_6+CM118_6*C18+CM318_6*S18-FB217_6*s->l[3][17]+FB317_6*s->l[2][17]-
 FM218_6*s->dpt[3][29];
  CM217_6 = CM218_6+s->In[5][17]*OpM217_6+FB117_6*s->l[3][17]-FB317_6*s->l[1][17]+s->dpt[3][29]*(FM118_6*C18+FM318_6*S18
 );
  CM317_6 = s->In[3][17]*OpM117_6+s->In[9][17]*OpM316_6-CM118_6*S18+CM318_6*C18-FB117_6*s->l[2][17]+FB217_6*s->l[1][17];
  FB117_15 = s->m[17]*(AlM117_15+OpM217_15*s->l[3][17]+s->l[2][17]*S16);
  FB217_15 = s->m[17]*(AlM217_15-OpM117_15*s->l[3][17]-s->l[1][17]*S16);
  FB317_15 = s->m[17]*(OpM117_15*s->l[2][17]-OpM217_15*s->l[1][17]);
  FM117_15 = FB117_15+FM118_15*C18+FM318_15*S18;
  FM217_15 = FB217_15+FM218_15;
  CM117_15 = s->In[1][17]*OpM117_15-s->In[3][17]*S16+CM118_15*C18+CM318_15*S18-FB217_15*s->l[3][17]+FB317_15*s->l[2][17]
 -FM218_15*s->dpt[3][29];
  CM217_15 = CM218_15+s->In[5][17]*OpM217_15+FB117_15*s->l[3][17]-FB317_15*s->l[1][17]+s->dpt[3][29]*(FM118_15*C18+
 FM318_15*S18);
  CM317_15 = s->In[3][17]*OpM117_15-s->In[9][17]*S16-CM118_15*S18+CM318_15*C18-FB117_15*s->l[2][17]+FB217_15*s->l[1][17];
  FB117_16 = s->m[17]*(AlM117_16-s->l[3][17]*S17);
  FB217_16 = s->m[17]*(AlM217_16-s->l[3][17]*C17);
  FB317_16 = s->m[17]*(s->l[1][17]*S17+s->l[2][17]*C17);
  CM317_16 = s->In[3][17]*C17-CM118_16*S18+CM318_16*C18-FB117_16*s->l[2][17]+FB217_16*s->l[1][17];
  CM317_17 = s->In[9][17]+s->m[17]*s->l[1][17]*s->l[1][17]+s->m[17]*s->l[2][17]*s->l[2][17]-C18*(s->In[3][18]*S18-
 s->In[9][18]*C18-CM219_17*S19-CM319_17*C19+FB118_17*s->l[2][18]-FB218_17*s->l[1][18])-S18*(CM119_17-s->In[1][18]*S18+
 s->In[3][18]*C18-FB218_17*s->l[3][18]+FB318_17*s->l[2][18]-s->dpt[3][32]*(C19*(FB219_17+FB220_17+FB221_17+FB222_17)-S19*(
 FB319_17-FM120_17*S20+FM320_17*C20)));
  FA116 = -(s->frc[1][16]-s->m[16]*(AlF116+BeF316*s->l[3][16]-s->l[1][16]*(OM216*OM216+OM316*OM316)+s->l[2][16]*(BS216-
 OpF316)));
  FA216 = -(s->frc[2][16]-s->m[16]*(AlF216+BeF616*s->l[3][16]+s->l[1][16]*(BS216+OpF316)-s->l[2][16]*(OM116*OM116+OM316*
 OM316)));
  FA316 = -(s->frc[3][16]-s->m[16]*(AlF316+BS916*s->l[3][16]+s->l[1][16]*(BS316-OpF216)+s->l[2][16]*(BS616+OpF115)));
  FF116 = FA116+FF117*C17-FF217*S17;
  FF216 = FA216+FF117*S17+FF217*C17;
  FF316 = FA316+FA317-FF118*S18+FF318*C18;
  CF116 = -(s->trq[1][16]-s->In[1][16]*OpF115-s->In[2][16]*OpF216-CF117*C17+CF217*S17+FA216*s->l[3][16]-FA316*
 s->l[2][16]+OM316*(s->In[2][16]*OM116+s->In[5][16]*OM216-s->In[9][16]*OM216)+s->dpt[3][26]*(FF117*S17+FF217*C17));
  CF216 = -(s->trq[2][16]-s->In[2][16]*OpF115-s->In[5][16]*OpF216-CF117*S17-CF217*C17-FA116*s->l[3][16]+FA316*
 s->l[1][16]-OM316*(s->In[1][16]*OM116+s->In[2][16]*OM216-s->In[9][16]*OM116)-s->dpt[3][26]*(FF117*C17-FF217*S17));
  CF316 = -(s->trq[3][16]-CF317-s->In[9][16]*OpF316+FA116*s->l[2][16]-FA216*s->l[1][16]-OM116*(s->In[2][16]*OM116+
 s->In[5][16]*OM216)+OM216*(s->In[1][16]*OM116+s->In[2][16]*OM216));
  FB116_1 = s->m[16]*AlM115_1;
  FB216_1 = s->m[16]*AlM216_1;
  FB316_1 = s->m[16]*AlM316_1;
  FM116_1 = FB116_1+FM117_1*C17-FM217_1*S17;
  FM216_1 = FB216_1+FM117_1*S17+FM217_1*C17;
  FM316_1 = FB316_1+FB317_1-FM118_1*S18+FM318_1*C18;
  CM116_1 = CM117_1*C17-CM217_1*S17-FB216_1*s->l[3][16]+FB316_1*s->l[2][16]-s->dpt[3][26]*(FM117_1*S17+FM217_1*C17);
  CM216_1 = CM117_1*S17+CM217_1*C17+FB116_1*s->l[3][16]-FB316_1*s->l[1][16]+s->dpt[3][26]*(FM117_1*C17-FM217_1*S17);
  CM316_1 = CM317_1-FB116_1*s->l[2][16]+FB216_1*s->l[1][16];
  FB116_2 = s->m[16]*AlM115_2;
  FB216_2 = s->m[16]*AlM216_2;
  FB316_2 = s->m[16]*AlM316_2;
  FM116_2 = FB116_2+FM117_2*C17-FM217_2*S17;
  FM216_2 = FB216_2+FM117_2*S17+FM217_2*C17;
  FM316_2 = FB316_2+FB317_2-FM118_2*S18+FM318_2*C18;
  CM116_2 = CM117_2*C17-CM217_2*S17-FB216_2*s->l[3][16]+FB316_2*s->l[2][16]-s->dpt[3][26]*(FM117_2*S17+FM217_2*C17);
  CM216_2 = CM117_2*S17+CM217_2*C17+FB116_2*s->l[3][16]-FB316_2*s->l[1][16]+s->dpt[3][26]*(FM117_2*C17-FM217_2*S17);
  CM316_2 = CM317_2-FB116_2*s->l[2][16]+FB216_2*s->l[1][16];
  FB116_3 = s->m[16]*AlM115_3;
  FB216_3 = s->m[16]*AlM216_3;
  FB316_3 = s->m[16]*AlM316_3;
  FM116_3 = FB116_3+FM117_3*C17-FM217_3*S17;
  FM216_3 = FB216_3+FM117_3*S17+FM217_3*C17;
  FM316_3 = FB316_3+FB317_3-FM118_3*S18+FM318_3*C18;
  CM116_3 = CM117_3*C17-CM217_3*S17-FB216_3*s->l[3][16]+FB316_3*s->l[2][16]-s->dpt[3][26]*(FM117_3*S17+FM217_3*C17);
  CM216_3 = CM117_3*S17+CM217_3*C17+FB116_3*s->l[3][16]-FB316_3*s->l[1][16]+s->dpt[3][26]*(FM117_3*C17-FM217_3*S17);
  CM316_3 = CM317_3-FB116_3*s->l[2][16]+FB216_3*s->l[1][16];
  FB116_4 = s->m[16]*(AlM116_4+OpM216_4*s->l[3][16]-OpM316_4*s->l[2][16]);
  FB216_4 = s->m[16]*(AlM216_4-OpM115_4*s->l[3][16]+OpM316_4*s->l[1][16]);
  FB316_4 = s->m[16]*(AlM316_4+OpM115_4*s->l[2][16]-OpM216_4*s->l[1][16]);
  FM116_4 = FB116_4+FM117_4*C17-FM217_4*S17;
  FM216_4 = FB216_4+FM117_4*S17+FM217_4*C17;
  FM316_4 = FB316_4+FB317_4-FM118_4*S18+FM318_4*C18;
  CM116_4 = s->In[1][16]*OpM115_4+s->In[2][16]*OpM216_4+CM117_4*C17-CM217_4*S17-FB216_4*s->l[3][16]+FB316_4*s->l[2][16]-
 s->dpt[3][26]*(FM117_4*S17+FM217_4*C17);
  CM216_4 = s->In[2][16]*OpM115_4+s->In[5][16]*OpM216_4+CM117_4*S17+CM217_4*C17+FB116_4*s->l[3][16]-FB316_4*s->l[1][16]+
 s->dpt[3][26]*(FM117_4*C17-FM217_4*S17);
  CM316_4 = CM317_4+s->In[9][16]*OpM316_4-FB116_4*s->l[2][16]+FB216_4*s->l[1][16];
  FB116_5 = s->m[16]*(AlM116_5+OpM216_5*s->l[3][16]-OpM316_5*s->l[2][16]);
  FB216_5 = s->m[16]*(AlM216_5-OpM115_5*s->l[3][16]+OpM316_5*s->l[1][16]);
  FB316_5 = s->m[16]*(AlM316_5+OpM115_5*s->l[2][16]-OpM216_5*s->l[1][16]);
  FM116_5 = FB116_5+FM117_5*C17-FM217_5*S17;
  FM216_5 = FB216_5+FM117_5*S17+FM217_5*C17;
  FM316_5 = FB316_5+FB317_5-FM118_5*S18+FM318_5*C18;
  CM116_5 = s->In[1][16]*OpM115_5+s->In[2][16]*OpM216_5+CM117_5*C17-CM217_5*S17-FB216_5*s->l[3][16]+FB316_5*s->l[2][16]-
 s->dpt[3][26]*(FM117_5*S17+FM217_5*C17);
  CM216_5 = s->In[2][16]*OpM115_5+s->In[5][16]*OpM216_5+CM117_5*S17+CM217_5*C17+FB116_5*s->l[3][16]-FB316_5*s->l[1][16]+
 s->dpt[3][26]*(FM117_5*C17-FM217_5*S17);
  CM316_5 = CM317_5+s->In[9][16]*OpM316_5-FB116_5*s->l[2][16]+FB216_5*s->l[1][16];
  FB116_6 = s->m[16]*(AlM116_6+OpM216_6*s->l[3][16]-OpM316_6*s->l[2][16]);
  FB216_6 = s->m[16]*(AlM216_6+OpM316_6*s->l[1][16]+s->l[3][16]*S15);
  FB316_6 = s->m[16]*(AlM316_6-OpM216_6*s->l[1][16]-s->l[2][16]*S15);
  FM116_6 = FB116_6+FM117_6*C17-FM217_6*S17;
  FM216_6 = FB216_6+FM117_6*S17+FM217_6*C17;
  FM316_6 = FB316_6+FB317_6-FM118_6*S18+FM318_6*C18;
  CM116_6 = CM117_6*C17-CM217_6*S17-s->dpt[3][26]*(FM117_6*S17+FM217_6*C17)-s->In[1][16]*S15+s->In[2][16]*OpM216_6-
 FB216_6*s->l[3][16]+FB316_6*s->l[2][16];
  CM216_6 = CM117_6*S17+CM217_6*C17+s->dpt[3][26]*(FM117_6*C17-FM217_6*S17)-s->In[2][16]*S15+s->In[5][16]*OpM216_6+
 FB116_6*s->l[3][16]-FB316_6*s->l[1][16];
  CM316_6 = CM317_6+s->In[9][16]*OpM316_6-FB116_6*s->l[2][16]+FB216_6*s->l[1][16];
  FB116_15 = s->m[16]*(s->l[2][16]*S16+s->l[3][16]*C16);
  FB216_15 = -s->m[16]*s->l[1][16]*S16;
  FB316_15 = -s->m[16]*s->l[1][16]*C16;
  CM116_15 = s->In[2][16]*C16+CM117_15*C17-CM217_15*S17-FB216_15*s->l[3][16]+FB316_15*s->l[2][16]-s->dpt[3][26]*(
 FM117_15*S17+FM217_15*C17);
  CM116_16 = s->In[1][16]+s->m[16]*s->l[2][16]*s->l[2][16]+s->m[16]*s->l[3][16]*s->l[3][16]-s->dpt[3][26]*(C17*(FB217_16
 +FM218_16)+S17*(FB117_16+FM118_16*C18+FM318_16*S18))+C17*(s->In[1][17]*C17+CM118_16*C18+CM318_16*S18-FB217_16*s->l[3][17]+
 FB317_16*s->l[2][17]-FM218_16*s->dpt[3][29])-S17*(CM218_16-s->In[5][17]*S17+FB117_16*s->l[3][17]-FB317_16*s->l[1][17]+
 s->dpt[3][29]*(FM118_16*C18+FM318_16*S18));
  FA115 = -(s->frc[1][15]-s->m[15]*(AlF115+BeF215*s->l[2][15]-s->l[1][15]*(OM215*OM215+OM315*OM315)+s->l[3][15]*(BS315+
 OpF26)));
  FA215 = -(s->frc[2][15]-s->m[15]*(AlF215+BS515*s->l[2][15]+s->l[1][15]*(BS215+OpF315)+s->l[3][15]*(BS615-OpF115)));
  FA315 = -(s->frc[3][15]-s->m[15]*(AlF315+BeF815*s->l[2][15]+s->l[1][15]*(BS315-OpF26)-s->l[3][15]*(OM115*OM115+OM215*
 OM215)));
  FF115 = FA115+FF116;
  FF315 = FA315+FF216*S16+FF316*C16;
  CF115 = -(s->trq[1][15]-CF116-s->In[1][15]*OpF115-s->In[2][15]*OpF26+FA215*s->l[3][15]-FA315*s->l[2][15]-OM215*(
 s->In[6][15]*OM215+s->In[9][15]*OM315)+OM315*(s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315)-s->dpt[2][24]*(FF216
 *S16+FF316*C16));
  CF215 = -(s->trq[2][15]-s->In[2][15]*OpF115-s->In[5][15]*OpF26-s->In[6][15]*OpF315-CF216*C16+CF316*S16-FA115*
 s->l[3][15]+FA315*s->l[1][15]+OM115*(s->In[6][15]*OM215+s->In[9][15]*OM315)-OM315*(s->In[1][15]*OM115+s->In[2][15]*OM215));
  CF315 = -(s->trq[3][15]-s->In[6][15]*OpF26-s->In[9][15]*OpF315-CF216*S16-CF316*C16+FA115*s->l[2][15]-FA215*s->l[1][15]
 +FF116*s->dpt[2][24]-OM115*(s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315)+OM215*(s->In[1][15]*OM115+s->In[2][15]
 *OM215));
  FB115_1 = s->m[15]*AlM115_1;
  FB215_1 = s->m[15]*AlM26_1;
  FB315_1 = s->m[15]*AlM315_1;
  FM115_1 = FB115_1+FM116_1;
  FM315_1 = FB315_1+FM216_1*S16+FM316_1*C16;
  CM115_1 = CM116_1-FB215_1*s->l[3][15]+FB315_1*s->l[2][15]+s->dpt[2][24]*(FM216_1*S16+FM316_1*C16);
  CM215_1 = CM216_1*C16-CM316_1*S16+FB115_1*s->l[3][15]-FB315_1*s->l[1][15];
  CM315_1 = CM216_1*S16+CM316_1*C16-FB115_1*s->l[2][15]+FB215_1*s->l[1][15]-FM116_1*s->dpt[2][24];
  FB115_2 = s->m[15]*AlM115_2;
  FB215_2 = s->m[15]*AlM26_2;
  FB315_2 = s->m[15]*AlM315_2;
  FM115_2 = FB115_2+FM116_2;
  FM315_2 = FB315_2+FM216_2*S16+FM316_2*C16;
  CM115_2 = CM116_2-FB215_2*s->l[3][15]+FB315_2*s->l[2][15]+s->dpt[2][24]*(FM216_2*S16+FM316_2*C16);
  CM215_2 = CM216_2*C16-CM316_2*S16+FB115_2*s->l[3][15]-FB315_2*s->l[1][15];
  CM315_2 = CM216_2*S16+CM316_2*C16-FB115_2*s->l[2][15]+FB215_2*s->l[1][15]-FM116_2*s->dpt[2][24];
  FB115_3 = s->m[15]*AlM115_3;
  FB215_3 = s->m[15]*AlM26_3;
  FB315_3 = s->m[15]*AlM315_3;
  FM115_3 = FB115_3+FM116_3;
  FM315_3 = FB315_3+FM216_3*S16+FM316_3*C16;
  CM115_3 = CM116_3-FB215_3*s->l[3][15]+FB315_3*s->l[2][15]+s->dpt[2][24]*(FM216_3*S16+FM316_3*C16);
  CM215_3 = CM216_3*C16-CM316_3*S16+FB115_3*s->l[3][15]-FB315_3*s->l[1][15];
  CM315_3 = CM216_3*S16+CM316_3*C16-FB115_3*s->l[2][15]+FB215_3*s->l[1][15]-FM116_3*s->dpt[2][24];
  FB115_4 = s->m[15]*(AlM115_4+OpM26_4*s->l[3][15]-OpM315_4*s->l[2][15]);
  FB215_4 = -s->m[15]*(OpM115_4*s->l[3][15]-OpM315_4*s->l[1][15]);
  FB315_4 = s->m[15]*(AlM315_4+OpM115_4*s->l[2][15]-OpM26_4*s->l[1][15]);
  FM115_4 = FB115_4+FM116_4;
  FM315_4 = FB315_4+FM216_4*S16+FM316_4*C16;
  CM115_4 = CM116_4+s->In[1][15]*OpM115_4+s->In[2][15]*OpM26_4-FB215_4*s->l[3][15]+FB315_4*s->l[2][15]+s->dpt[2][24]*(
 FM216_4*S16+FM316_4*C16);
  CM215_4 = s->In[2][15]*OpM115_4+s->In[5][15]*OpM26_4+s->In[6][15]*OpM315_4+CM216_4*C16-CM316_4*S16+FB115_4*s->l[3][15]
 -FB315_4*s->l[1][15];
  CM315_4 = s->In[6][15]*OpM26_4+s->In[9][15]*OpM315_4+CM216_4*S16+CM316_4*C16-FB115_4*s->l[2][15]+FB215_4*s->l[1][15]-
 FM116_4*s->dpt[2][24];
  FB115_5 = s->m[15]*(AlM115_5-OpM315_5*s->l[2][15]+s->l[3][15]*C6);
  FB215_5 = -s->m[15]*(OpM115_5*s->l[3][15]-OpM315_5*s->l[1][15]);
  FB315_5 = s->m[15]*(AlM315_5+OpM115_5*s->l[2][15]-s->l[1][15]*C6);
  FM115_5 = FB115_5+FM116_5;
  FM315_5 = FB315_5+FM216_5*S16+FM316_5*C16;
  CM115_5 = CM116_5+s->In[1][15]*OpM115_5+s->In[2][15]*C6-FB215_5*s->l[3][15]+FB315_5*s->l[2][15]+s->dpt[2][24]*(FM216_5
 *S16+FM316_5*C16);
  CM215_5 = s->In[2][15]*OpM115_5+s->In[5][15]*C6+s->In[6][15]*OpM315_5+CM216_5*C16-CM316_5*S16+FB115_5*s->l[3][15]-
 FB315_5*s->l[1][15];
  CM315_5 = s->In[6][15]*C6+s->In[9][15]*OpM315_5+CM216_5*S16+CM316_5*C16-FB115_5*s->l[2][15]+FB215_5*s->l[1][15]-
 FM116_5*s->dpt[2][24];
  FB115_6 = s->m[15]*(AlM115_6-s->l[2][15]*C15);
  FB215_6 = s->m[15]*(s->l[1][15]*C15+s->l[3][15]*S15);
  FB315_6 = s->m[15]*(AlM315_6-s->l[2][15]*S15);
  CM215_6 = CM216_6*C16-CM316_6*S16-s->In[2][15]*S15+s->In[6][15]*C15+FB115_6*s->l[3][15]-FB315_6*s->l[1][15];
  CM215_15 = s->In[5][15]+s->m[15]*s->l[1][15]*s->l[1][15]+s->m[15]*s->l[3][15]*s->l[3][15]+C16*(s->In[5][16]*C16+
 CM117_15*S17+CM217_15*C17+FB116_15*s->l[3][16]-FB316_15*s->l[1][16]+s->dpt[3][26]*(FM117_15*C17-FM217_15*S17))-S16*(CM317_15
 -s->In[9][16]*S16-FB116_15*s->l[2][16]+FB216_15*s->l[1][16]);

// = = Block_0_2_0_2_0_8 = = 
 
// Backward Dynamics 

  FA125 = -(s->frc[1][25]-s->m[25]*(AlF125+BS125*s->l[1][25]+BeF225*s->l[2][25]+BeF325*s->l[3][25]));
  FA225 = -(s->frc[2][25]-s->m[25]*(AlF225+BS525*s->l[2][25]+BeF425*s->l[1][25]+BeF625*s->l[3][25]));
  FA325 = -(s->frc[3][25]-s->m[25]*(AlF325+BS925*s->l[3][25]+BeF725*s->l[1][25]+BeF825*s->l[2][25]));
  FF125 = FA125+FF126*C26+FF130*C30+FF326*S26+FF330*S30;
  FF225 = FA225+FF226+FF230;
  CF125 = -(s->trq[1][25]-s->In[1][25]*OpF125-CF126*C26-CF130*C30-CF326*S26-CF330*S30+FA225*s->l[3][25]-FA325*
 s->l[2][25]+FF226*s->dpt[3][48]+FF230*s->dpt[3][49]-OM225*(s->In[6][25]*OM225+s->In[9][25]*OM325)+OM325*(s->In[5][25]*OM225+
 s->In[6][25]*OM325)+s->dpt[2][48]*(FF126*S26-FF326*C26)+s->dpt[2][49]*(FF130*S30-FF330*C30));
  CF225 = -(s->trq[2][25]-CF226-CF230-s->In[5][25]*OpF225-s->In[6][25]*OpF324-FA125*s->l[3][25]+FA325*s->l[1][25]-OM125*
 (s->In[1][25]*OM325-s->In[6][25]*OM225-s->In[9][25]*OM325)-s->dpt[1][48]*(FF126*S26-FF326*C26)-s->dpt[1][49]*(FF130*S30-
 FF330*C30)-s->dpt[3][48]*(FF126*C26+FF326*S26)-s->dpt[3][49]*(FF130*C30+FF330*S30));
  CF325 = -(s->trq[3][25]-s->In[6][25]*OpF225-s->In[9][25]*OpF324+CF126*S26+CF130*S30-CF326*C26-CF330*C30+FA125*
 s->l[2][25]-FA225*s->l[1][25]-FF226*s->dpt[1][48]-FF230*s->dpt[1][49]-OM125*(s->In[6][25]*OM325-OM225*(s->In[1][25]-
 s->In[5][25]))+s->dpt[2][48]*(FF126*C26+FF326*S26)+s->dpt[2][49]*(FF130*C30+FF330*S30));
  FB125_1 = s->m[25]*AlM125_1;
  FB225_1 = s->m[25]*AlM225_1;
  FB325_1 = s->m[25]*AlM324_1;
  FM125_1 = FB125_1+FM126_1*C26+FM130_1*C30+FM326_1*S26+FM330_1*S30;
  FM225_1 = FB225_1+FM226_1+FM230_1;
  CM125_1 = CM126_1*C26+CM130_1*C30+CM326_1*S26+CM330_1*S30-FB225_1*s->l[3][25]+FB325_1*s->l[2][25]-FM226_1*
 s->dpt[3][48]-FM230_1*s->dpt[3][49]-s->dpt[2][48]*(FM126_1*S26-FM326_1*C26)-s->dpt[2][49]*(FM130_1*S30-FM330_1*C30);
  CM225_1 = CM226_1+CM230_1+FB125_1*s->l[3][25]-FB325_1*s->l[1][25]+s->dpt[1][48]*(FM126_1*S26-FM326_1*C26)+
 s->dpt[1][49]*(FM130_1*S30-FM330_1*C30)+s->dpt[3][48]*(FM126_1*C26+FM326_1*S26)+s->dpt[3][49]*(FM130_1*C30+FM330_1*S30);
  CM325_1 = -(CM126_1*S26+CM130_1*S30-CM326_1*C26-CM330_1*C30+FB125_1*s->l[2][25]-FB225_1*s->l[1][25]-FM226_1*
 s->dpt[1][48]-FM230_1*s->dpt[1][49]+s->dpt[2][48]*(FM126_1*C26+FM326_1*S26)+s->dpt[2][49]*(FM130_1*C30+FM330_1*S30));
  FB125_2 = s->m[25]*AlM125_2;
  FB225_2 = s->m[25]*AlM225_2;
  FB325_2 = s->m[25]*AlM324_2;
  FM125_2 = FB125_2+FM126_2*C26+FM130_2*C30+FM326_2*S26+FM330_2*S30;
  FM225_2 = FB225_2+FM226_2+FM230_2;
  CM125_2 = CM126_2*C26+CM130_2*C30+CM326_2*S26+CM330_2*S30-FB225_2*s->l[3][25]+FB325_2*s->l[2][25]-FM226_2*
 s->dpt[3][48]-FM230_2*s->dpt[3][49]-s->dpt[2][48]*(FM126_2*S26-FM326_2*C26)-s->dpt[2][49]*(FM130_2*S30-FM330_2*C30);
  CM225_2 = CM226_2+CM230_2+FB125_2*s->l[3][25]-FB325_2*s->l[1][25]+s->dpt[1][48]*(FM126_2*S26-FM326_2*C26)+
 s->dpt[1][49]*(FM130_2*S30-FM330_2*C30)+s->dpt[3][48]*(FM126_2*C26+FM326_2*S26)+s->dpt[3][49]*(FM130_2*C30+FM330_2*S30);
  CM325_2 = -(CM126_2*S26+CM130_2*S30-CM326_2*C26-CM330_2*C30+FB125_2*s->l[2][25]-FB225_2*s->l[1][25]-FM226_2*
 s->dpt[1][48]-FM230_2*s->dpt[1][49]+s->dpt[2][48]*(FM126_2*C26+FM326_2*S26)+s->dpt[2][49]*(FM130_2*C30+FM330_2*S30));
  FB125_3 = s->m[25]*AlM125_3;
  FB225_3 = s->m[25]*AlM225_3;
  FB325_3 = s->m[25]*AlM324_3;
  FM125_3 = FB125_3+FM126_3*C26+FM130_3*C30+FM326_3*S26+FM330_3*S30;
  FM225_3 = FB225_3+FM226_3+FM230_3;
  CM125_3 = CM126_3*C26+CM130_3*C30+CM326_3*S26+CM330_3*S30-FB225_3*s->l[3][25]+FB325_3*s->l[2][25]-FM226_3*
 s->dpt[3][48]-FM230_3*s->dpt[3][49]-s->dpt[2][48]*(FM126_3*S26-FM326_3*C26)-s->dpt[2][49]*(FM130_3*S30-FM330_3*C30);
  CM225_3 = CM226_3+CM230_3+FB125_3*s->l[3][25]-FB325_3*s->l[1][25]+s->dpt[1][48]*(FM126_3*S26-FM326_3*C26)+
 s->dpt[1][49]*(FM130_3*S30-FM330_3*C30)+s->dpt[3][48]*(FM126_3*C26+FM326_3*S26)+s->dpt[3][49]*(FM130_3*C30+FM330_3*S30);
  CM325_3 = -(CM126_3*S26+CM130_3*S30-CM326_3*C26-CM330_3*C30+FB125_3*s->l[2][25]-FB225_3*s->l[1][25]-FM226_3*
 s->dpt[1][48]-FM230_3*s->dpt[1][49]+s->dpt[2][48]*(FM126_3*C26+FM326_3*S26)+s->dpt[2][49]*(FM130_3*C30+FM330_3*S30));
  FB125_4 = s->m[25]*(AlM125_4+OpM225_4*s->l[3][25]-OpM324_4*s->l[2][25]);
  FB225_4 = s->m[25]*(AlM225_4-OpM125_4*s->l[3][25]+OpM324_4*s->l[1][25]);
  FB325_4 = s->m[25]*(AlM324_4+OpM125_4*s->l[2][25]-OpM225_4*s->l[1][25]);
  FM125_4 = FB125_4+FM126_4*C26+FM130_4*C30+FM326_4*S26+FM330_4*S30;
  FM225_4 = FB225_4+FM226_4+FM230_4;
  CM125_4 = s->In[1][25]*OpM125_4+CM126_4*C26+CM130_4*C30+CM326_4*S26+CM330_4*S30-FB225_4*s->l[3][25]+FB325_4*
 s->l[2][25]-FM226_4*s->dpt[3][48]-FM230_4*s->dpt[3][49]-s->dpt[2][48]*(FM126_4*S26-FM326_4*C26)-s->dpt[2][49]*(FM130_4*S30-
 FM330_4*C30);
  CM225_4 = CM226_4+CM230_4+s->In[5][25]*OpM225_4+s->In[6][25]*OpM324_4+FB125_4*s->l[3][25]-FB325_4*s->l[1][25]+
 s->dpt[1][48]*(FM126_4*S26-FM326_4*C26)+s->dpt[1][49]*(FM130_4*S30-FM330_4*C30)+s->dpt[3][48]*(FM126_4*C26+FM326_4*S26)+
 s->dpt[3][49]*(FM130_4*C30+FM330_4*S30);
  CM325_4 = s->In[6][25]*OpM225_4+s->In[9][25]*OpM324_4-CM126_4*S26-CM130_4*S30+CM326_4*C26+CM330_4*C30-FB125_4*
 s->l[2][25]+FB225_4*s->l[1][25]+FM226_4*s->dpt[1][48]+FM230_4*s->dpt[1][49]-s->dpt[2][48]*(FM126_4*C26+FM326_4*S26)-
 s->dpt[2][49]*(FM130_4*C30+FM330_4*S30);
  FB125_5 = s->m[25]*(AlM125_5+OpM225_5*s->l[3][25]-OpM324_5*s->l[2][25]);
  FB225_5 = s->m[25]*(AlM225_5-OpM125_5*s->l[3][25]+OpM324_5*s->l[1][25]);
  FB325_5 = s->m[25]*(AlM324_5+OpM125_5*s->l[2][25]-OpM225_5*s->l[1][25]);
  FM125_5 = FB125_5+FM126_5*C26+FM130_5*C30+FM326_5*S26+FM330_5*S30;
  FM225_5 = FB225_5+FM226_5+FM230_5;
  CM125_5 = s->In[1][25]*OpM125_5+CM126_5*C26+CM130_5*C30+CM326_5*S26+CM330_5*S30-FB225_5*s->l[3][25]+FB325_5*
 s->l[2][25]-FM226_5*s->dpt[3][48]-FM230_5*s->dpt[3][49]-s->dpt[2][48]*(FM126_5*S26-FM326_5*C26)-s->dpt[2][49]*(FM130_5*S30-
 FM330_5*C30);
  CM225_5 = CM226_5+CM230_5+s->In[5][25]*OpM225_5+s->In[6][25]*OpM324_5+FB125_5*s->l[3][25]-FB325_5*s->l[1][25]+
 s->dpt[1][48]*(FM126_5*S26-FM326_5*C26)+s->dpt[1][49]*(FM130_5*S30-FM330_5*C30)+s->dpt[3][48]*(FM126_5*C26+FM326_5*S26)+
 s->dpt[3][49]*(FM130_5*C30+FM330_5*S30);
  CM325_5 = s->In[6][25]*OpM225_5+s->In[9][25]*OpM324_5-CM126_5*S26-CM130_5*S30+CM326_5*C26+CM330_5*C30-FB125_5*
 s->l[2][25]+FB225_5*s->l[1][25]+FM226_5*s->dpt[1][48]+FM230_5*s->dpt[1][49]-s->dpt[2][48]*(FM126_5*C26+FM326_5*S26)-
 s->dpt[2][49]*(FM130_5*C30+FM330_5*S30);
  FB125_6 = s->m[25]*(AlM125_6+OpM225_6*s->l[3][25]-OpM324_6*s->l[2][25]);
  FB225_6 = s->m[25]*(AlM225_6-OpM125_6*s->l[3][25]+OpM324_6*s->l[1][25]);
  FB325_6 = s->m[25]*(AlM324_6+OpM125_6*s->l[2][25]-OpM225_6*s->l[1][25]);
  FM125_6 = FB125_6+FM126_6*C26+FM130_6*C30+FM326_6*S26+FM330_6*S30;
  FM225_6 = FB225_6+FM226_6+FM230_6;
  CM125_6 = s->In[1][25]*OpM125_6+CM126_6*C26+CM130_6*C30+CM326_6*S26+CM330_6*S30-FB225_6*s->l[3][25]+FB325_6*
 s->l[2][25]-FM226_6*s->dpt[3][48]-FM230_6*s->dpt[3][49]-s->dpt[2][48]*(FM126_6*S26-FM326_6*C26)-s->dpt[2][49]*(FM130_6*S30-
 FM330_6*C30);
  CM225_6 = CM226_6+CM230_6+s->In[5][25]*OpM225_6+s->In[6][25]*OpM324_6+FB125_6*s->l[3][25]-FB325_6*s->l[1][25]+
 s->dpt[1][48]*(FM126_6*S26-FM326_6*C26)+s->dpt[1][49]*(FM130_6*S30-FM330_6*C30)+s->dpt[3][48]*(FM126_6*C26+FM326_6*S26)+
 s->dpt[3][49]*(FM130_6*C30+FM330_6*S30);
  CM325_6 = s->In[6][25]*OpM225_6+s->In[9][25]*OpM324_6-CM126_6*S26-CM130_6*S30+CM326_6*C26+CM330_6*C30-FB125_6*
 s->l[2][25]+FB225_6*s->l[1][25]+FM226_6*s->dpt[1][48]+FM230_6*s->dpt[1][49]-s->dpt[2][48]*(FM126_6*C26+FM326_6*S26)-
 s->dpt[2][49]*(FM130_6*C30+FM330_6*S30);
  FB125_23 = s->m[25]*(AlM125_23+OpM225_23*s->l[3][25]-s->l[2][25]*S24);
  FB225_23 = s->m[25]*(AlM225_23-OpM125_23*s->l[3][25]+s->l[1][25]*S24);
  FB325_23 = s->m[25]*(OpM125_23*s->l[2][25]-OpM225_23*s->l[1][25]);
  FM125_23 = FB125_23+FM126_23*C26+FM130_23*C30+FM326_23*S26+FM330_23*S30;
  FM225_23 = FB225_23+FM226_23+FM230_23;
  CM125_23 = s->In[1][25]*OpM125_23+CM126_23*C26+CM130_23*C30+CM326_23*S26+CM330_23*S30-FB225_23*s->l[3][25]+FB325_23*
 s->l[2][25]-FM226_23*s->dpt[3][48]-FM230_23*s->dpt[3][49]-s->dpt[2][48]*(FM126_23*S26-FM326_23*C26)-s->dpt[2][49]*(FM130_23*
 S30-FM330_23*C30);
  CM225_23 = CM226_23+CM230_23+s->In[5][25]*OpM225_23+s->In[6][25]*S24+FB125_23*s->l[3][25]-FB325_23*s->l[1][25]+
 s->dpt[1][48]*(FM126_23*S26-FM326_23*C26)+s->dpt[1][49]*(FM130_23*S30-FM330_23*C30)+s->dpt[3][48]*(FM126_23*C26+FM326_23*S26
 )+s->dpt[3][49]*(FM130_23*C30+FM330_23*S30);
  CM325_23 = s->In[6][25]*OpM225_23+s->In[9][25]*S24-CM126_23*S26-CM130_23*S30+CM326_23*C26+CM330_23*C30-FB125_23*
 s->l[2][25]+FB225_23*s->l[1][25]+FM226_23*s->dpt[1][48]+FM230_23*s->dpt[1][49]-s->dpt[2][48]*(FM126_23*C26+FM326_23*S26)-
 s->dpt[2][49]*(FM130_23*C30+FM330_23*S30);
  FB125_24 = s->m[25]*C25*(s->dpt[3][44]+s->l[3][25]);
  FB225_24 = s->m[25]*(AlM225_24-s->l[3][25]*S25);
  FB325_24 = -s->m[25]*(s->l[1][25]*C25-s->l[2][25]*S25);
  CM325_24 = s->In[6][25]*C25-CM126_24*S26-CM130_24*S30+CM326_24*C26+CM330_24*C30-FB125_24*s->l[2][25]+FB225_24*
 s->l[1][25]+FM226_24*s->dpt[1][48]+FM230_24*s->dpt[1][49]-s->dpt[2][48]*(FM126_24*C26+FM326_24*S26)-s->dpt[2][49]*(FM130_24*
 C30+FM330_24*S30);
  CM325_25 = s->In[9][25]+s->m[25]*s->l[1][25]*s->l[1][25]+s->m[25]*s->l[2][25]*s->l[2][25]+s->dpt[1][48]*(FB226_25+
 FM227_25*C27-FM327_25*S27)+s->dpt[1][49]*(FB230_25+FM231_25*C31-FM331_25*S31)-s->dpt[2][48]*(C26*(FB126_25+FM127_25)+S26*(
 FB326_25+FM227_25*S27+FM327_25*C27))-s->dpt[2][49]*(C30*(FB130_25+FM131_25)+S30*(FB330_25+FM231_25*S31+FM331_25*C31))+C26*(
 s->In[9][26]*C26+CM227_25*S27+CM327_25*C27-FB126_25*s->l[2][26]+FB226_25*s->l[1][26]-FM127_25*s->dpt[2][51])-S26*(CM127_25-
 s->In[1][26]*S26-FB226_25*s->l[3][26]+FB326_25*s->l[2][26]+s->dpt[2][51]*(FM227_25*S27+FM327_25*C27))+C30*(s->In[9][30]*C30+
 CM231_25*S31+CM331_25*C31-FB130_25*s->l[2][30]+FB230_25*s->l[1][30]-FM131_25*s->dpt[2][60])-S30*(CM131_25-s->In[1][30]*S30-
 FB230_25*s->l[3][30]+FB330_25*s->l[2][30]+s->dpt[2][60]*(FM231_25*S31+FM331_25*C31));
  FA124 = -(s->frc[1][24]-s->m[24]*(AlF124+BeF324*s->l[3][24]-s->l[1][24]*(OM224*OM224+OM324*OM324)+s->l[2][24]*(BS224-
 OpF324)));
  FA224 = -(s->frc[2][24]-s->m[24]*(AlF223+BeF624*s->l[3][24]+s->l[1][24]*(BS224+OpF324)-s->l[2][24]*(OM124*OM124+OM324*
 OM324)));
  FA324 = -(s->frc[3][24]-s->m[24]*(AlF324+BS924*s->l[3][24]+s->l[1][24]*(BS324-OpF223)+s->l[2][24]*(BS624+OpF124)));
  FF124 = FA124+FF125*C25-FF225*S25;
  FF324 = FA324+FA325-FF126*S26-FF130*S30+FF326*C26+FF330*C30;
  CF124 = -(s->trq[1][24]-s->In[1][24]*OpF124-s->In[3][24]*OpF324-CF125*C25+CF225*S25+FA224*s->l[3][24]-FA324*
 s->l[2][24]-OM224*(s->In[3][24]*OM124-s->In[5][24]*OM324+s->In[9][24]*OM324)+s->dpt[3][44]*(FF125*S25+FF225*C25));
  CF224 = -(s->trq[2][24]-s->In[5][24]*OpF223-CF125*S25-CF225*C25-FA124*s->l[3][24]+FA324*s->l[1][24]+OM124*(
 s->In[3][24]*OM124+s->In[9][24]*OM324)-OM324*(s->In[1][24]*OM124+s->In[3][24]*OM324)-s->dpt[3][44]*(FF125*C25-FF225*S25));
  CF324 = -(s->trq[3][24]-CF325-s->In[3][24]*OpF124-s->In[9][24]*OpF324+FA124*s->l[2][24]-FA224*s->l[1][24]+OM224*(
 s->In[1][24]*OM124+s->In[3][24]*OM324-s->In[5][24]*OM124));
  FB124_1 = s->m[24]*AlM124_1;
  FB224_1 = s->m[24]*AlM223_1;
  FB324_1 = s->m[24]*AlM324_1;
  FM124_1 = FB124_1+FM125_1*C25-FM225_1*S25;
  FM324_1 = FB324_1+FB325_1-FM126_1*S26-FM130_1*S30+FM326_1*C26+FM330_1*C30;
  CM124_1 = CM125_1*C25-CM225_1*S25-FB224_1*s->l[3][24]+FB324_1*s->l[2][24]-s->dpt[3][44]*(FM125_1*S25+FM225_1*C25);
  CM224_1 = CM125_1*S25+CM225_1*C25+FB124_1*s->l[3][24]-FB324_1*s->l[1][24]+s->dpt[3][44]*(FM125_1*C25-FM225_1*S25);
  CM324_1 = CM325_1-FB124_1*s->l[2][24]+FB224_1*s->l[1][24];
  FB124_2 = s->m[24]*AlM124_2;
  FB224_2 = s->m[24]*AlM223_2;
  FB324_2 = s->m[24]*AlM324_2;
  FM124_2 = FB124_2+FM125_2*C25-FM225_2*S25;
  FM324_2 = FB324_2+FB325_2-FM126_2*S26-FM130_2*S30+FM326_2*C26+FM330_2*C30;
  CM124_2 = CM125_2*C25-CM225_2*S25-FB224_2*s->l[3][24]+FB324_2*s->l[2][24]-s->dpt[3][44]*(FM125_2*S25+FM225_2*C25);
  CM224_2 = CM125_2*S25+CM225_2*C25+FB124_2*s->l[3][24]-FB324_2*s->l[1][24]+s->dpt[3][44]*(FM125_2*C25-FM225_2*S25);
  CM324_2 = CM325_2-FB124_2*s->l[2][24]+FB224_2*s->l[1][24];
  FB124_3 = s->m[24]*AlM124_3;
  FB224_3 = s->m[24]*AlM223_3;
  FB324_3 = s->m[24]*AlM324_3;
  FM124_3 = FB124_3+FM125_3*C25-FM225_3*S25;
  FM324_3 = FB324_3+FB325_3-FM126_3*S26-FM130_3*S30+FM326_3*C26+FM330_3*C30;
  CM124_3 = CM125_3*C25-CM225_3*S25-FB224_3*s->l[3][24]+FB324_3*s->l[2][24]-s->dpt[3][44]*(FM125_3*S25+FM225_3*C25);
  CM224_3 = CM125_3*S25+CM225_3*C25+FB124_3*s->l[3][24]-FB324_3*s->l[1][24]+s->dpt[3][44]*(FM125_3*C25-FM225_3*S25);
  CM324_3 = CM325_3-FB124_3*s->l[2][24]+FB224_3*s->l[1][24];
  FB124_4 = s->m[24]*(AlM124_4+OpM223_4*s->l[3][24]-OpM324_4*s->l[2][24]);
  FB224_4 = s->m[24]*(AlM223_4-OpM124_4*s->l[3][24]+OpM324_4*s->l[1][24]);
  FB324_4 = s->m[24]*(AlM324_4+OpM124_4*s->l[2][24]-OpM223_4*s->l[1][24]);
  FM124_4 = FB124_4+FM125_4*C25-FM225_4*S25;
  FM324_4 = FB324_4+FB325_4-FM126_4*S26-FM130_4*S30+FM326_4*C26+FM330_4*C30;
  CM124_4 = s->In[1][24]*OpM124_4+s->In[3][24]*OpM324_4+CM125_4*C25-CM225_4*S25-FB224_4*s->l[3][24]+FB324_4*s->l[2][24]-
 s->dpt[3][44]*(FM125_4*S25+FM225_4*C25);
  CM224_4 = s->In[5][24]*OpM223_4+CM125_4*S25+CM225_4*C25+FB124_4*s->l[3][24]-FB324_4*s->l[1][24]+s->dpt[3][44]*(FM125_4
 *C25-FM225_4*S25);
  CM324_4 = CM325_4+s->In[3][24]*OpM124_4+s->In[9][24]*OpM324_4-FB124_4*s->l[2][24]+FB224_4*s->l[1][24];
  FB124_5 = s->m[24]*(AlM124_5+OpM223_5*s->l[3][24]-OpM324_5*s->l[2][24]);
  FB224_5 = s->m[24]*(AlM223_5-OpM124_5*s->l[3][24]+OpM324_5*s->l[1][24]);
  FB324_5 = s->m[24]*(AlM324_5+OpM124_5*s->l[2][24]-OpM223_5*s->l[1][24]);
  FM124_5 = FB124_5+FM125_5*C25-FM225_5*S25;
  FM324_5 = FB324_5+FB325_5-FM126_5*S26-FM130_5*S30+FM326_5*C26+FM330_5*C30;
  CM124_5 = s->In[1][24]*OpM124_5+s->In[3][24]*OpM324_5+CM125_5*C25-CM225_5*S25-FB224_5*s->l[3][24]+FB324_5*s->l[2][24]-
 s->dpt[3][44]*(FM125_5*S25+FM225_5*C25);
  CM224_5 = s->In[5][24]*OpM223_5+CM125_5*S25+CM225_5*C25+FB124_5*s->l[3][24]-FB324_5*s->l[1][24]+s->dpt[3][44]*(FM125_5
 *C25-FM225_5*S25);
  CM324_5 = CM325_5+s->In[3][24]*OpM124_5+s->In[9][24]*OpM324_5-FB124_5*s->l[2][24]+FB224_5*s->l[1][24];
  FB124_6 = s->m[24]*(AlM124_6-OpM324_6*s->l[2][24]+s->l[3][24]*S23);
  FB224_6 = s->m[24]*(AlM223_6-OpM124_6*s->l[3][24]+OpM324_6*s->l[1][24]);
  FB324_6 = s->m[24]*(AlM324_6+OpM124_6*s->l[2][24]-s->l[1][24]*S23);
  CM124_6 = s->In[1][24]*OpM124_6+s->In[3][24]*OpM324_6+CM125_6*C25-CM225_6*S25-FB224_6*s->l[3][24]+FB324_6*s->l[2][24]-
 s->dpt[3][44]*(FM125_6*S25+FM225_6*C25);
  CM224_6 = s->In[5][24]*S23+CM125_6*S25+CM225_6*C25+FB124_6*s->l[3][24]-FB324_6*s->l[1][24]+s->dpt[3][44]*(FM125_6*C25-
 FM225_6*S25);
  CM324_6 = CM325_6+s->In[3][24]*OpM124_6+s->In[9][24]*OpM324_6-FB124_6*s->l[2][24]+FB224_6*s->l[1][24];
  FB124_23 = -s->m[24]*s->l[2][24]*S24;
  FB224_23 = s->m[24]*(s->l[1][24]*S24-s->l[3][24]*C24);
  FB324_23 = s->m[24]*s->l[2][24]*C24;
  CM224_23 = CM125_23*S25+CM225_23*C25+FB124_23*s->l[3][24]-FB324_23*s->l[1][24]+s->dpt[3][44]*(FM125_23*C25-FM225_23*
 S25);
  CM224_24 = s->In[5][24]+s->m[24]*s->l[1][24]*s->l[1][24]+s->m[24]*s->l[3][24]*s->l[3][24]+s->dpt[3][44]*(C25*(FB125_24
 +FM126_24*C26+FM130_24*C30+FM326_24*S26+FM330_24*S30)-S25*(FB225_24+FM226_24+FM230_24))+C25*(CM226_24+CM230_24+s->In[5][25]*
 C25+FB125_24*s->l[3][25]-FB325_24*s->l[1][25]+s->dpt[1][48]*(FM126_24*S26-FM326_24*C26)+s->dpt[1][49]*(FM130_24*S30-FM330_24
 *C30)+s->dpt[3][48]*(FM126_24*C26+FM326_24*S26)+s->dpt[3][49]*(FM130_24*C30+FM330_24*S30))+S25*(s->In[1][25]*S25+CM126_24*
 C26+CM130_24*C30+CM326_24*S26+CM330_24*S30-FB225_24*s->l[3][25]+FB325_24*s->l[2][25]-FM226_24*s->dpt[3][48]-FM230_24*
 s->dpt[3][49]-s->dpt[2][48]*(FM126_24*S26-FM326_24*C26)-s->dpt[2][49]*(FM130_24*S30-FM330_24*C30));
  FA123 = -(s->frc[1][23]-s->m[23]*(AlF123-s->l[1][23]*(OM223*OM223+OM323*OM323)+s->l[2][23]*(BS223-OpF323)+s->l[3][23]*
 (BS323+OpF223)));
  FA223 = -(s->frc[2][23]-s->m[23]*(AlF223+s->l[1][23]*(BS223+OpF323)-s->l[2][23]*(OM123*OM123+OM323*OM323)+s->l[3][23]*
 (BS623-OpF16)));
  FA323 = -(s->frc[3][23]-s->m[23]*(AlF323+s->l[1][23]*(BS323-OpF223)+s->l[2][23]*(BS623+OpF16)-s->l[3][23]*(OM123*OM123
 +OM223*OM223)));
  FF123 = FA123+FF124*C24+FF324*S24;
  FF223 = FA223+FA224+FF125*S25+FF225*C25;
  FF323 = FA323-FF124*S24+FF324*C24;
  CF123 = -(s->trq[1][23]-s->In[1][23]*OpF16-CF124*C24-CF324*S24+FA223*s->l[3][23]-FA323*s->l[2][23]-OM223*(s->In[6][23]
 *OM223+s->In[9][23]*OM323)+OM323*(s->In[5][23]*OM223+s->In[6][23]*OM323));
  CF223 = -(s->trq[2][23]-CF224-s->In[5][23]*OpF223-s->In[6][23]*OpF323-FA123*s->l[3][23]+FA323*s->l[1][23]-OM123*(
 s->In[1][23]*OM323-s->In[6][23]*OM223-s->In[9][23]*OM323));
  CF323 = -(s->trq[3][23]-s->In[6][23]*OpF223-s->In[9][23]*OpF323+CF124*S24-CF324*C24+FA123*s->l[2][23]-FA223*
 s->l[1][23]-OM123*(s->In[6][23]*OM323-OM223*(s->In[1][23]-s->In[5][23])));
  FB123_1 = s->m[23]*AlM16_1;
  FB223_1 = s->m[23]*AlM223_1;
  FB323_1 = s->m[23]*AlM323_1;
  FM123_1 = FB123_1+FM124_1*C24+FM324_1*S24;
  FM223_1 = FB223_1+FB224_1+FM125_1*S25+FM225_1*C25;
  FM323_1 = FB323_1-FM124_1*S24+FM324_1*C24;
  CM123_1 = CM124_1*C24+CM324_1*S24-FB223_1*s->l[3][23]+FB323_1*s->l[2][23];
  CM223_1 = CM224_1+FB123_1*s->l[3][23]-FB323_1*s->l[1][23];
  CM323_1 = -(CM124_1*S24-CM324_1*C24+FB123_1*s->l[2][23]-FB223_1*s->l[1][23]);
  FB123_2 = s->m[23]*AlM16_2;
  FB223_2 = s->m[23]*AlM223_2;
  FB323_2 = s->m[23]*AlM323_2;
  FM123_2 = FB123_2+FM124_2*C24+FM324_2*S24;
  FM223_2 = FB223_2+FB224_2+FM125_2*S25+FM225_2*C25;
  FM323_2 = FB323_2-FM124_2*S24+FM324_2*C24;
  CM123_2 = CM124_2*C24+CM324_2*S24-FB223_2*s->l[3][23]+FB323_2*s->l[2][23];
  CM223_2 = CM224_2+FB123_2*s->l[3][23]-FB323_2*s->l[1][23];
  CM323_2 = -(CM124_2*S24-CM324_2*C24+FB123_2*s->l[2][23]-FB223_2*s->l[1][23]);
  FB123_3 = s->m[23]*AlM16_3;
  FB223_3 = s->m[23]*AlM223_3;
  FB323_3 = s->m[23]*AlM323_3;
  FM123_3 = FB123_3+FM124_3*C24+FM324_3*S24;
  FM223_3 = FB223_3+FB224_3+FM125_3*S25+FM225_3*C25;
  FM323_3 = FB323_3-FM124_3*S24+FM324_3*C24;
  CM123_3 = CM124_3*C24+CM324_3*S24-FB223_3*s->l[3][23]+FB323_3*s->l[2][23];
  CM223_3 = CM224_3+FB123_3*s->l[3][23]-FB323_3*s->l[1][23];
  CM323_3 = -(CM124_3*S24-CM324_3*C24+FB123_3*s->l[2][23]-FB223_3*s->l[1][23]);
  FB123_4 = s->m[23]*(AlM123_4+OpM223_4*s->l[3][23]-OpM323_4*s->l[2][23]);
  FB223_4 = s->m[23]*(AlM223_4-OpM16_4*s->l[3][23]+OpM323_4*s->l[1][23]);
  FB323_4 = s->m[23]*(AlM323_4+OpM16_4*s->l[2][23]-OpM223_4*s->l[1][23]);
  FM223_4 = FB223_4+FB224_4+FM125_4*S25+FM225_4*C25;
  FM323_4 = FB323_4-FM124_4*S24+FM324_4*C24;
  CM123_4 = s->In[1][23]*OpM16_4+CM124_4*C24+CM324_4*S24-FB223_4*s->l[3][23]+FB323_4*s->l[2][23];
  CM223_4 = CM224_4+s->In[5][23]*OpM223_4+s->In[6][23]*OpM323_4+FB123_4*s->l[3][23]-FB323_4*s->l[1][23];
  CM323_4 = s->In[6][23]*OpM223_4+s->In[9][23]*OpM323_4-CM124_4*S24+CM324_4*C24-FB123_4*s->l[2][23]+FB223_4*s->l[1][23];
  FB123_5 = s->m[23]*(AlM123_5+OpM223_5*s->l[3][23]-OpM323_5*s->l[2][23]);
  FB223_5 = s->m[23]*(AlM223_5+OpM323_5*s->l[1][23]-s->l[3][23]*S6);
  FB323_5 = s->m[23]*(AlM323_5-OpM223_5*s->l[1][23]+s->l[2][23]*S6);
  FM223_5 = FB223_5+FB224_5+FM125_5*S25+FM225_5*C25;
  FM323_5 = FB323_5-FM124_5*S24+FM324_5*C24;
  CM123_5 = s->In[1][23]*S6+CM124_5*C24+CM324_5*S24-FB223_5*s->l[3][23]+FB323_5*s->l[2][23];
  CM223_5 = CM224_5+s->In[5][23]*OpM223_5+s->In[6][23]*OpM323_5+FB123_5*s->l[3][23]-FB323_5*s->l[1][23];
  CM323_5 = s->In[6][23]*OpM223_5+s->In[9][23]*OpM323_5-CM124_5*S24+CM324_5*C24-FB123_5*s->l[2][23]+FB223_5*s->l[1][23];
  FB123_6 = -s->m[23]*(s->l[2][23]*C23-s->l[3][23]*S23);
  FB223_6 = s->m[23]*C23*(s->dpt[1][4]+s->l[1][23]);
  FB323_6 = s->m[23]*(AlM323_6-s->l[1][23]*S23);
  CM123_6 = CM124_6*C24+CM324_6*S24-FB223_6*s->l[3][23]+FB323_6*s->l[2][23];
  CM123_23 = s->In[1][23]+s->m[23]*s->l[2][23]*s->l[2][23]+s->m[23]*s->l[3][23]*s->l[3][23]+C24*(s->In[1][24]*C24+
 s->In[3][24]*S24+CM125_23*C25-CM225_23*S25-FB224_23*s->l[3][24]+FB324_23*s->l[2][24]-s->dpt[3][44]*(FM125_23*S25+FM225_23*
 C25))+S24*(CM325_23+s->In[3][24]*C24+s->In[9][24]*S24-FB124_23*s->l[2][24]+FB224_23*s->l[1][24]);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6]+BeF26*s->l[2][6]+BeF36*s->l[3][6]));
  FA26 = -(s->frc[2][6]-s->m[6]*(AlF26+BS56*s->l[2][6]+BeF46*s->l[1][6]+BeF66*s->l[3][6]));
  FA36 = -(s->frc[3][6]-s->m[6]*(AlF35+BS96*s->l[3][6]+BeF76*s->l[1][6]+BeF86*s->l[2][6]));
  FF16 = FA16+FF123+FF115*C15+FF17*C7+FF315*S15+FF37*S7;
  FF26 = FA215+FA26+FA27+FF216*C16+FF223*C23+FF28*C8-FF316*S16-FF323*S23-FF38*S8;
  FF36 = FA36-FF115*S15-FF17*S7+FF223*S23+FF315*C15+FF323*C23+FF37*C7;
  CF16 = -(s->trq[1][6]-CF123-s->In[1][6]*OpF16-s->In[3][6]*OpF35-CF115*C15-CF17*C7-CF315*S15-CF37*S7+FA26*s->l[3][6]-
 FA36*s->l[2][6]-OM26*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)+OM36*(s->In[5][6]*OM26+s->In[6][6]*OM36)+
 s->dpt[2][2]*(FF17*S7-FF37*C7)+s->dpt[2][3]*(FF115*S15-FF315*C15)+s->dpt[3][4]*(FF223*C23-FF323*S23));
  CF26 = -(s->trq[2][6]-CF215-CF27-s->In[5][6]*OpF26-s->In[6][6]*OpF35-CF223*C23+CF323*S23-FA16*s->l[3][6]+FA36*
 s->l[1][6]-FF123*s->dpt[3][4]+OM16*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)-OM36*(s->In[1][6]*OM16+s->In[3][6]*
 OM36)+s->dpt[1][4]*(FF223*S23+FF323*C23));
  CF36 = -(s->trq[3][6]-s->In[3][6]*OpF16-s->In[6][6]*OpF26-s->In[9][6]*OpF35+CF115*S15+CF17*S7-CF223*S23-CF315*C15-
 CF323*C23-CF37*C7+FA16*s->l[2][6]-FA26*s->l[1][6]-OM16*(s->In[5][6]*OM26+s->In[6][6]*OM36)+OM26*(s->In[1][6]*OM16+
 s->In[3][6]*OM36)-s->dpt[1][4]*(FF223*C23-FF323*S23)+s->dpt[2][2]*(FF17*C7+FF37*S7)+s->dpt[2][3]*(FF115*C15+FF315*S15));
  FB16_1 = s->m[6]*AlM16_1;
  FB26_1 = s->m[6]*AlM26_1;
  FB36_1 = s->m[6]*S5;
  FM16_1 = FB16_1+FM123_1+FM115_1*C15+FM17_1*C7+FM315_1*S15+FM37_1*S7;
  FM26_1 = FB215_1+FB26_1+FB27_1+FM216_1*C16+FM223_1*C23+FM28_1*C8-FM316_1*S16-FM323_1*S23-FM38_1*S8;
  FM36_1 = FB36_1-FM115_1*S15-FM17_1*S7+FM223_1*S23+FM315_1*C15+FM323_1*C23+FM37_1*C7;
  CM16_1 = CM123_1+CM115_1*C15+CM17_1*C7+CM315_1*S15+CM37_1*S7-FB26_1*s->l[3][6]+FB36_1*s->l[2][6]-s->dpt[2][2]*(FM17_1*
 S7-FM37_1*C7)-s->dpt[2][3]*(FM115_1*S15-FM315_1*C15)-s->dpt[3][4]*(FM223_1*C23-FM323_1*S23);
  CM26_1 = CM215_1+CM27_1+CM223_1*C23-CM323_1*S23+FB16_1*s->l[3][6]-FB36_1*s->l[1][6]+FM123_1*s->dpt[3][4]-s->dpt[1][4]*
 (FM223_1*S23+FM323_1*C23);
  CM36_1 = CM223_1*S23+CM323_1*C23+s->dpt[1][4]*(FM223_1*C23-FM323_1*S23)-CM115_1*S15-CM17_1*S7+CM315_1*C15+CM37_1*C7-
 FB16_1*s->l[2][6]+FB26_1*s->l[1][6]-s->dpt[2][2]*(FM17_1*C7+FM37_1*S7)-s->dpt[2][3]*(FM115_1*C15+FM315_1*S15);
  FB16_2 = s->m[6]*AlM16_2;
  FB26_2 = s->m[6]*AlM26_2;
  FB36_2 = s->m[6]*AlM35_2;
  FM16_2 = FB16_2+FM123_2+FM115_2*C15+FM17_2*C7+FM315_2*S15+FM37_2*S7;
  FM26_2 = FB215_2+FB26_2+FB27_2+FM216_2*C16+FM223_2*C23+FM28_2*C8-FM316_2*S16-FM323_2*S23-FM38_2*S8;
  CM16_2 = CM123_2+CM115_2*C15+CM17_2*C7+CM315_2*S15+CM37_2*S7-FB26_2*s->l[3][6]+FB36_2*s->l[2][6]-s->dpt[2][2]*(FM17_2*
 S7-FM37_2*C7)-s->dpt[2][3]*(FM115_2*S15-FM315_2*C15)-s->dpt[3][4]*(FM223_2*C23-FM323_2*S23);
  CM26_2 = CM215_2+CM27_2+CM223_2*C23-CM323_2*S23+FB16_2*s->l[3][6]-FB36_2*s->l[1][6]+FM123_2*s->dpt[3][4]-s->dpt[1][4]*
 (FM223_2*S23+FM323_2*C23);
  CM36_2 = CM223_2*S23+CM323_2*C23+s->dpt[1][4]*(FM223_2*C23-FM323_2*S23)-CM115_2*S15-CM17_2*S7+CM315_2*C15+CM37_2*C7-
 FB16_2*s->l[2][6]+FB26_2*s->l[1][6]-s->dpt[2][2]*(FM17_2*C7+FM37_2*S7)-s->dpt[2][3]*(FM115_2*C15+FM315_2*S15);
  FB16_3 = s->m[6]*AlM16_3;
  FB26_3 = s->m[6]*AlM26_3;
  FB36_3 = s->m[6]*AlM35_3;
  FM16_3 = FB16_3+FM123_3+FM115_3*C15+FM17_3*C7+FM315_3*S15+FM37_3*S7;
  FM26_3 = FB215_3+FB26_3+FB27_3+FM216_3*C16+FM223_3*C23+FM28_3*C8-FM316_3*S16-FM323_3*S23-FM38_3*S8;
  CM16_3 = CM123_3+CM115_3*C15+CM17_3*C7+CM315_3*S15+CM37_3*S7-FB26_3*s->l[3][6]+FB36_3*s->l[2][6]-s->dpt[2][2]*(FM17_3*
 S7-FM37_3*C7)-s->dpt[2][3]*(FM115_3*S15-FM315_3*C15)-s->dpt[3][4]*(FM223_3*C23-FM323_3*S23);
  CM26_3 = CM215_3+CM27_3+CM223_3*C23-CM323_3*S23+FB16_3*s->l[3][6]-FB36_3*s->l[1][6]+FM123_3*s->dpt[3][4]-s->dpt[1][4]*
 (FM223_3*S23+FM323_3*C23);
  CM36_3 = CM223_3*S23+CM323_3*C23+s->dpt[1][4]*(FM223_3*C23-FM323_3*S23)-CM115_3*S15-CM17_3*S7+CM315_3*C15+CM37_3*C7-
 FB16_3*s->l[2][6]+FB26_3*s->l[1][6]-s->dpt[2][2]*(FM17_3*C7+FM37_3*S7)-s->dpt[2][3]*(FM115_3*C15+FM315_3*S15);
  FB16_4 = s->m[6]*(OpM26_4*s->l[3][6]-s->l[2][6]*S5);
  FB26_4 = -s->m[6]*(OpM16_4*s->l[3][6]-s->l[1][6]*S5);
  FB36_4 = s->m[6]*(OpM16_4*s->l[2][6]-OpM26_4*s->l[1][6]);
  CM16_4 = CM123_4+s->In[1][6]*OpM16_4+s->In[3][6]*S5+CM115_4*C15+CM17_4*C7+CM315_4*S15+CM37_4*S7-FB26_4*s->l[3][6]+
 FB36_4*s->l[2][6]-s->dpt[2][2]*(FM17_4*S7-FM37_4*C7)-s->dpt[2][3]*(FM115_4*S15-FM315_4*C15)-s->dpt[3][4]*(FM223_4*C23-
 FM323_4*S23);
  CM26_4 = CM215_4+CM27_4+s->In[5][6]*OpM26_4+s->In[6][6]*S5+CM223_4*C23-CM323_4*S23+FB16_4*s->l[3][6]-FB36_4*s->l[1][6]
 -s->dpt[1][4]*(FM223_4*S23+FM323_4*C23)+s->dpt[3][4]*(FB123_4+FM124_4*C24+FM324_4*S24);
  CM36_4 = s->In[3][6]*OpM16_4+s->In[6][6]*OpM26_4+s->In[9][6]*S5-CM115_4*S15-CM17_4*S7+CM223_4*S23+CM315_4*C15+CM323_4*
 C23+CM37_4*C7-FB16_4*s->l[2][6]+FB26_4*s->l[1][6]+s->dpt[1][4]*(FM223_4*C23-FM323_4*S23)-s->dpt[2][2]*(FM17_4*C7+FM37_4*S7)-
 s->dpt[2][3]*(FM115_4*C15+FM315_4*S15);
  FB16_5 = s->m[6]*s->l[3][6]*C6;
  FB26_5 = -s->m[6]*s->l[3][6]*S6;
  FB36_5 = -s->m[6]*(s->l[1][6]*C6-s->l[2][6]*S6);
  CM36_5 = s->In[3][6]*S6+s->In[6][6]*C6-CM115_5*S15-CM17_5*S7+CM223_5*S23+CM315_5*C15+CM323_5*C23+CM37_5*C7-FB16_5*
 s->l[2][6]+FB26_5*s->l[1][6]+s->dpt[1][4]*(FM223_5*C23-FM323_5*S23)-s->dpt[2][2]*(FM17_5*C7+FM37_5*S7)-s->dpt[2][3]*(FM115_5
 *C15+FM315_5*S15);
  CM36_6 = s->In[9][6]+s->m[6]*s->l[1][6]*s->l[1][6]+s->m[6]*s->l[2][6]*s->l[2][6]+s->dpt[1][4]*(C23*(FB223_6+FB224_6+
 FM125_6*S25+FM225_6*C25)-S23*(FB323_6+C24*(FB324_6+FB325_6-FM126_6*S26-FM130_6*S30+FM326_6*C26+FM330_6*C30)-S24*(FB124_6+
 FM125_6*C25-FM225_6*S25)))-s->dpt[2][2]*(C7*(FB17_6+FM18_6)+S7*(FB37_6+FM28_6*S8+FM38_6*C8))-s->dpt[2][3]*(C15*(FB115_6+
 FM116_6)+S15*(FB315_6+FM216_6*S16+FM316_6*C16))+C15*(s->In[9][15]*C15+CM216_6*S16+CM316_6*C16-FB115_6*s->l[2][15]+FB215_6*
 s->l[1][15]-FM116_6*s->dpt[2][24])-S15*(CM116_6-s->In[1][15]*S15-FB215_6*s->l[3][15]+FB315_6*s->l[2][15]+s->dpt[2][24]*(
 FM216_6*S16+FM316_6*C16))+C23*(s->In[6][23]*S23+s->In[9][23]*C23-CM124_6*S24+CM324_6*C24-FB123_6*s->l[2][23]+FB223_6*
 s->l[1][23])+S23*(CM224_6+s->In[5][23]*S23+s->In[6][23]*C23+FB123_6*s->l[3][23]-FB323_6*s->l[1][23])+C7*(s->In[9][7]*C7+
 CM28_6*S8+CM38_6*C8-FB17_6*s->l[2][7]+FB27_6*s->l[1][7]-FM18_6*s->dpt[2][6])-S7*(CM18_6-s->In[1][7]*S7-FB27_6*s->l[3][7]+
 FB37_6*s->l[2][7]+s->dpt[2][6]*(FM28_6*S8+FM38_6*C8));
  FF5_16 = FF16*C6-FF26*S6;
  FF5_26 = FF16*S6+FF26*C6;
  CF5_26 = CF16*S6+CF26*C6;
  FM51_16 = FM16_1*C6-FM26_1*S6;
  FM51_26 = FM16_1*S6+FM26_1*C6;
  CM51_26 = CM16_1*S6+CM26_1*C6;
  FM52_26 = FM16_2*S6+FM26_2*C6;
  CM52_26 = CM16_2*S6+CM26_2*C6;
  CM53_26 = CM16_3*S6+CM26_3*C6;
  CM54_26 = CM16_4*S6+CM26_4*C6;
  CM55_26 = C6*(CM215_5+CM27_5+s->In[5][6]*C6+CM223_5*C23-CM323_5*S23+FB16_5*s->l[3][6]-FB36_5*s->l[1][6]-s->dpt[1][4]*(
 FM223_5*S23+FM323_5*C23)+s->dpt[3][4]*(FB123_5+FM124_5*C24+FM324_5*S24))+S6*(CM123_5+s->In[1][6]*S6+CM115_5*C15+CM17_5*C7+
 CM315_5*S15+CM37_5*S7-FB26_5*s->l[3][6]+FB36_5*s->l[2][6]-s->dpt[2][2]*(FM17_5*S7-FM37_5*C7)-s->dpt[2][3]*(FM115_5*S15-
 FM315_5*C15)-s->dpt[3][4]*(FM223_5*C23-FM323_5*S23));
  FF4_15 = FF36*S5+FF5_16*C5;
  FF4_35 = FF36*C5-FF5_16*S5;
  CF4_15 = CF36*S5+C5*(CF16*C6-CF26*S6);
  FM41_15 = FM36_1*S5+FM51_16*C5;
  FM41_35 = FM36_1*C5-FM51_16*S5;
  CM41_15 = CM36_1*S5+C5*(CM16_1*C6-CM26_1*S6);
  FM42_35 = C5*(FB36_2-FM115_2*S15-FM17_2*S7+FM223_2*S23+FM315_2*C15+FM323_2*C23+FM37_2*C7)-S5*(FM16_2*C6-FM26_2*S6);
  CM42_15 = CM36_2*S5+C5*(CM16_2*C6-CM26_2*S6);
  CM43_15 = CM36_3*S5+C5*(CM16_3*C6-CM26_3*S6);
  CM44_15 = CM36_4*S5+C5*(CM16_4*C6-CM26_4*S6);
  FF3_24 = -(FF4_35*S4-FF5_26*C4);
  FF3_34 = FF4_35*C4+FF5_26*S4;
  FM31_24 = -(FM41_35*S4-FM51_26*C4);
  FM31_34 = FM41_35*C4+FM51_26*S4;
  FM32_24 = -(FM42_35*S4-FM52_26*C4);
  FM32_34 = FM42_35*C4+FM52_26*S4;
  FM33_34 = C4*(C5*(FB36_3-FM115_3*S15-FM17_3*S7+FM223_3*S23+FM315_3*C15+FM323_3*C23+FM37_3*C7)-S5*(FM16_3*C6-FM26_3*S6)
 )+S4*(FM16_3*S6+FM26_3*C6);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  M[1][1] = FM41_15;
  M[1][2] = FM31_24;
  M[1][3] = FM31_34;
  M[1][4] = CM41_15;
  M[1][5] = CM51_26;
  M[1][6] = CM36_1;
  M[1][7] = CM27_1;
  M[1][8] = CM18_1;
  M[1][9] = CM39_1;
  M[1][10] = CM210_1;
  M[1][11] = CM111_1;
  M[1][12] = CM212_1;
  M[1][13] = FB313_1;
  M[1][14] = FB314_1;
  M[1][15] = CM215_1;
  M[1][16] = CM116_1;
  M[1][17] = CM317_1;
  M[1][18] = CM218_1;
  M[1][19] = CM119_1;
  M[1][20] = CM220_1;
  M[1][21] = FB321_1;
  M[1][22] = FB322_1;
  M[1][23] = CM123_1;
  M[1][24] = CM224_1;
  M[1][25] = CM325_1;
  M[1][26] = CM226_1;
  M[1][27] = CM127_1;
  M[1][28] = CM328_1;
  M[1][29] = CM229_1;
  M[1][30] = CM230_1;
  M[1][31] = CM131_1;
  M[1][32] = CM332_1;
  M[1][33] = CM233_1;
  M[2][1] = FM31_24;
  M[2][2] = FM32_24;
  M[2][3] = FM32_34;
  M[2][4] = CM42_15;
  M[2][5] = CM52_26;
  M[2][6] = CM36_2;
  M[2][7] = CM27_2;
  M[2][8] = CM18_2;
  M[2][9] = CM39_2;
  M[2][10] = CM210_2;
  M[2][11] = CM111_2;
  M[2][12] = CM212_2;
  M[2][13] = FB313_2;
  M[2][14] = FB314_2;
  M[2][15] = CM215_2;
  M[2][16] = CM116_2;
  M[2][17] = CM317_2;
  M[2][18] = CM218_2;
  M[2][19] = CM119_2;
  M[2][20] = CM220_2;
  M[2][21] = FB321_2;
  M[2][22] = FB322_2;
  M[2][23] = CM123_2;
  M[2][24] = CM224_2;
  M[2][25] = CM325_2;
  M[2][26] = CM226_2;
  M[2][27] = CM127_2;
  M[2][28] = CM328_2;
  M[2][29] = CM229_2;
  M[2][30] = CM230_2;
  M[2][31] = CM131_2;
  M[2][32] = CM332_2;
  M[2][33] = CM233_2;
  M[3][1] = FM31_34;
  M[3][2] = FM32_34;
  M[3][3] = FM33_34;
  M[3][4] = CM43_15;
  M[3][5] = CM53_26;
  M[3][6] = CM36_3;
  M[3][7] = CM27_3;
  M[3][8] = CM18_3;
  M[3][9] = CM39_3;
  M[3][10] = CM210_3;
  M[3][11] = CM111_3;
  M[3][12] = CM212_3;
  M[3][13] = FB313_3;
  M[3][14] = FB314_3;
  M[3][15] = CM215_3;
  M[3][16] = CM116_3;
  M[3][17] = CM317_3;
  M[3][18] = CM218_3;
  M[3][19] = CM119_3;
  M[3][20] = CM220_3;
  M[3][21] = FB321_3;
  M[3][22] = FB322_3;
  M[3][23] = CM123_3;
  M[3][24] = CM224_3;
  M[3][25] = CM325_3;
  M[3][26] = CM226_3;
  M[3][27] = CM127_3;
  M[3][28] = CM328_3;
  M[3][29] = CM229_3;
  M[3][30] = CM230_3;
  M[3][31] = CM131_3;
  M[3][32] = CM332_3;
  M[3][33] = CM233_3;
  M[4][1] = CM41_15;
  M[4][2] = CM42_15;
  M[4][3] = CM43_15;
  M[4][4] = CM44_15;
  M[4][5] = CM54_26;
  M[4][6] = CM36_4;
  M[4][7] = CM27_4;
  M[4][8] = CM18_4;
  M[4][9] = CM39_4;
  M[4][10] = CM210_4;
  M[4][11] = CM111_4;
  M[4][12] = CM212_4;
  M[4][13] = FB313_4;
  M[4][14] = FB314_4;
  M[4][15] = CM215_4;
  M[4][16] = CM116_4;
  M[4][17] = CM317_4;
  M[4][18] = CM218_4;
  M[4][19] = CM119_4;
  M[4][20] = CM220_4;
  M[4][21] = FB321_4;
  M[4][22] = FB322_4;
  M[4][23] = CM123_4;
  M[4][24] = CM224_4;
  M[4][25] = CM325_4;
  M[4][26] = CM226_4;
  M[4][27] = CM127_4;
  M[4][28] = CM328_4;
  M[4][29] = CM229_4;
  M[4][30] = CM230_4;
  M[4][31] = CM131_4;
  M[4][32] = CM332_4;
  M[4][33] = CM233_4;
  M[5][1] = CM51_26;
  M[5][2] = CM52_26;
  M[5][3] = CM53_26;
  M[5][4] = CM54_26;
  M[5][5] = CM55_26;
  M[5][6] = CM36_5;
  M[5][7] = CM27_5;
  M[5][8] = CM18_5;
  M[5][9] = CM39_5;
  M[5][10] = CM210_5;
  M[5][11] = CM111_5;
  M[5][12] = CM212_5;
  M[5][13] = FB313_5;
  M[5][14] = FB314_5;
  M[5][15] = CM215_5;
  M[5][16] = CM116_5;
  M[5][17] = CM317_5;
  M[5][18] = CM218_5;
  M[5][19] = CM119_5;
  M[5][20] = CM220_5;
  M[5][21] = FB321_5;
  M[5][22] = FB322_5;
  M[5][23] = CM123_5;
  M[5][24] = CM224_5;
  M[5][25] = CM325_5;
  M[5][26] = CM226_5;
  M[5][27] = CM127_5;
  M[5][28] = CM328_5;
  M[5][29] = CM229_5;
  M[5][30] = CM230_5;
  M[5][31] = CM131_5;
  M[5][32] = CM332_5;
  M[5][33] = CM233_5;
  M[6][1] = CM36_1;
  M[6][2] = CM36_2;
  M[6][3] = CM36_3;
  M[6][4] = CM36_4;
  M[6][5] = CM36_5;
  M[6][6] = CM36_6;
  M[6][7] = CM27_6;
  M[6][8] = CM18_6;
  M[6][9] = CM39_6;
  M[6][10] = CM210_6;
  M[6][11] = CM111_6;
  M[6][12] = CM212_6;
  M[6][13] = FB313_6;
  M[6][14] = FB314_6;
  M[6][15] = CM215_6;
  M[6][16] = CM116_6;
  M[6][17] = CM317_6;
  M[6][18] = CM218_6;
  M[6][19] = CM119_6;
  M[6][20] = CM220_6;
  M[6][21] = FB321_6;
  M[6][22] = FB322_6;
  M[6][23] = CM123_6;
  M[6][24] = CM224_6;
  M[6][25] = CM325_6;
  M[6][26] = CM226_6;
  M[6][27] = CM127_6;
  M[6][28] = CM328_6;
  M[6][29] = CM229_6;
  M[6][30] = CM230_6;
  M[6][31] = CM131_6;
  M[6][32] = CM332_6;
  M[6][33] = CM233_6;
  M[7][1] = CM27_1;
  M[7][2] = CM27_2;
  M[7][3] = CM27_3;
  M[7][4] = CM27_4;
  M[7][5] = CM27_5;
  M[7][6] = CM27_6;
  M[7][7] = CM27_7;
  M[7][8] = CM18_7;
  M[7][9] = CM39_7;
  M[7][10] = CM210_7;
  M[7][11] = CM111_7;
  M[7][12] = CM212_7;
  M[7][13] = FB313_7;
  M[7][14] = FB314_7;
  M[8][1] = CM18_1;
  M[8][2] = CM18_2;
  M[8][3] = CM18_3;
  M[8][4] = CM18_4;
  M[8][5] = CM18_5;
  M[8][6] = CM18_6;
  M[8][7] = CM18_7;
  M[8][8] = CM18_8;
  M[8][9] = CM39_8;
  M[8][10] = CM210_8;
  M[8][11] = CM111_8;
  M[8][12] = CM212_8;
  M[8][13] = FB313_8;
  M[8][14] = FB314_8;
  M[9][1] = CM39_1;
  M[9][2] = CM39_2;
  M[9][3] = CM39_3;
  M[9][4] = CM39_4;
  M[9][5] = CM39_5;
  M[9][6] = CM39_6;
  M[9][7] = CM39_7;
  M[9][8] = CM39_8;
  M[9][9] = CM39_9;
  M[9][10] = CM210_9;
  M[9][11] = CM111_9;
  M[9][12] = CM212_9;
  M[9][13] = FB313_9;
  M[9][14] = FB314_9;
  M[10][1] = CM210_1;
  M[10][2] = CM210_2;
  M[10][3] = CM210_3;
  M[10][4] = CM210_4;
  M[10][5] = CM210_5;
  M[10][6] = CM210_6;
  M[10][7] = CM210_7;
  M[10][8] = CM210_8;
  M[10][9] = CM210_9;
  M[10][10] = CM210_10;
  M[10][11] = CM111_10;
  M[10][12] = CM212_10;
  M[10][13] = FB313_10;
  M[10][14] = FB314_10;
  M[11][1] = CM111_1;
  M[11][2] = CM111_2;
  M[11][3] = CM111_3;
  M[11][4] = CM111_4;
  M[11][5] = CM111_5;
  M[11][6] = CM111_6;
  M[11][7] = CM111_7;
  M[11][8] = CM111_8;
  M[11][9] = CM111_9;
  M[11][10] = CM111_10;
  M[11][11] = CM111_11;
  M[12][1] = CM212_1;
  M[12][2] = CM212_2;
  M[12][3] = CM212_3;
  M[12][4] = CM212_4;
  M[12][5] = CM212_5;
  M[12][6] = CM212_6;
  M[12][7] = CM212_7;
  M[12][8] = CM212_8;
  M[12][9] = CM212_9;
  M[12][10] = CM212_10;
  M[12][12] = CM212_12;
  M[12][13] = FB313_12;
  M[12][14] = FB314_12;
  M[13][1] = FB313_1;
  M[13][2] = FB313_2;
  M[13][3] = FB313_3;
  M[13][4] = FB313_4;
  M[13][5] = FB313_5;
  M[13][6] = FB313_6;
  M[13][7] = FB313_7;
  M[13][8] = FB313_8;
  M[13][9] = FB313_9;
  M[13][10] = FB313_10;
  M[13][12] = FB313_12;
  M[13][13] = s->m[13];
  M[14][1] = FB314_1;
  M[14][2] = FB314_2;
  M[14][3] = FB314_3;
  M[14][4] = FB314_4;
  M[14][5] = FB314_5;
  M[14][6] = FB314_6;
  M[14][7] = FB314_7;
  M[14][8] = FB314_8;
  M[14][9] = FB314_9;
  M[14][10] = FB314_10;
  M[14][12] = FB314_12;
  M[14][14] = s->m[14];
  M[15][1] = CM215_1;
  M[15][2] = CM215_2;
  M[15][3] = CM215_3;
  M[15][4] = CM215_4;
  M[15][5] = CM215_5;
  M[15][6] = CM215_6;
  M[15][15] = CM215_15;
  M[15][16] = CM116_15;
  M[15][17] = CM317_15;
  M[15][18] = CM218_15;
  M[15][19] = CM119_15;
  M[15][20] = CM220_15;
  M[15][21] = FB321_15;
  M[15][22] = FB322_15;
  M[16][1] = CM116_1;
  M[16][2] = CM116_2;
  M[16][3] = CM116_3;
  M[16][4] = CM116_4;
  M[16][5] = CM116_5;
  M[16][6] = CM116_6;
  M[16][15] = CM116_15;
  M[16][16] = CM116_16;
  M[16][17] = CM317_16;
  M[16][18] = CM218_16;
  M[16][19] = CM119_16;
  M[16][20] = CM220_16;
  M[16][21] = FB321_16;
  M[16][22] = FB322_16;
  M[17][1] = CM317_1;
  M[17][2] = CM317_2;
  M[17][3] = CM317_3;
  M[17][4] = CM317_4;
  M[17][5] = CM317_5;
  M[17][6] = CM317_6;
  M[17][15] = CM317_15;
  M[17][16] = CM317_16;
  M[17][17] = CM317_17;
  M[17][18] = CM218_17;
  M[17][19] = CM119_17;
  M[17][20] = CM220_17;
  M[17][21] = FB321_17;
  M[17][22] = FB322_17;
  M[18][1] = CM218_1;
  M[18][2] = CM218_2;
  M[18][3] = CM218_3;
  M[18][4] = CM218_4;
  M[18][5] = CM218_5;
  M[18][6] = CM218_6;
  M[18][15] = CM218_15;
  M[18][16] = CM218_16;
  M[18][17] = CM218_17;
  M[18][18] = CM218_18;
  M[18][19] = CM119_18;
  M[18][20] = CM220_18;
  M[18][21] = FB321_18;
  M[18][22] = FB322_18;
  M[19][1] = CM119_1;
  M[19][2] = CM119_2;
  M[19][3] = CM119_3;
  M[19][4] = CM119_4;
  M[19][5] = CM119_5;
  M[19][6] = CM119_6;
  M[19][15] = CM119_15;
  M[19][16] = CM119_16;
  M[19][17] = CM119_17;
  M[19][18] = CM119_18;
  M[19][19] = CM119_19;
  M[20][1] = CM220_1;
  M[20][2] = CM220_2;
  M[20][3] = CM220_3;
  M[20][4] = CM220_4;
  M[20][5] = CM220_5;
  M[20][6] = CM220_6;
  M[20][15] = CM220_15;
  M[20][16] = CM220_16;
  M[20][17] = CM220_17;
  M[20][18] = CM220_18;
  M[20][20] = CM220_20;
  M[20][21] = FB321_20;
  M[20][22] = FB322_20;
  M[21][1] = FB321_1;
  M[21][2] = FB321_2;
  M[21][3] = FB321_3;
  M[21][4] = FB321_4;
  M[21][5] = FB321_5;
  M[21][6] = FB321_6;
  M[21][15] = FB321_15;
  M[21][16] = FB321_16;
  M[21][17] = FB321_17;
  M[21][18] = FB321_18;
  M[21][20] = FB321_20;
  M[21][21] = s->m[21];
  M[22][1] = FB322_1;
  M[22][2] = FB322_2;
  M[22][3] = FB322_3;
  M[22][4] = FB322_4;
  M[22][5] = FB322_5;
  M[22][6] = FB322_6;
  M[22][15] = FB322_15;
  M[22][16] = FB322_16;
  M[22][17] = FB322_17;
  M[22][18] = FB322_18;
  M[22][20] = FB322_20;
  M[22][22] = s->m[22];
  M[23][1] = CM123_1;
  M[23][2] = CM123_2;
  M[23][3] = CM123_3;
  M[23][4] = CM123_4;
  M[23][5] = CM123_5;
  M[23][6] = CM123_6;
  M[23][23] = CM123_23;
  M[23][24] = CM224_23;
  M[23][25] = CM325_23;
  M[23][26] = CM226_23;
  M[23][27] = CM127_23;
  M[23][28] = CM328_23;
  M[23][29] = CM229_23;
  M[23][30] = CM230_23;
  M[23][31] = CM131_23;
  M[23][32] = CM332_23;
  M[23][33] = CM233_23;
  M[24][1] = CM224_1;
  M[24][2] = CM224_2;
  M[24][3] = CM224_3;
  M[24][4] = CM224_4;
  M[24][5] = CM224_5;
  M[24][6] = CM224_6;
  M[24][23] = CM224_23;
  M[24][24] = CM224_24;
  M[24][25] = CM325_24;
  M[24][26] = CM226_24;
  M[24][27] = CM127_24;
  M[24][28] = CM328_24;
  M[24][29] = CM229_24;
  M[24][30] = CM230_24;
  M[24][31] = CM131_24;
  M[24][32] = CM332_24;
  M[24][33] = CM233_24;
  M[25][1] = CM325_1;
  M[25][2] = CM325_2;
  M[25][3] = CM325_3;
  M[25][4] = CM325_4;
  M[25][5] = CM325_5;
  M[25][6] = CM325_6;
  M[25][23] = CM325_23;
  M[25][24] = CM325_24;
  M[25][25] = CM325_25;
  M[25][26] = CM226_25;
  M[25][27] = CM127_25;
  M[25][28] = CM328_25;
  M[25][29] = CM229_25;
  M[25][30] = CM230_25;
  M[25][31] = CM131_25;
  M[25][32] = CM332_25;
  M[25][33] = CM233_25;
  M[26][1] = CM226_1;
  M[26][2] = CM226_2;
  M[26][3] = CM226_3;
  M[26][4] = CM226_4;
  M[26][5] = CM226_5;
  M[26][6] = CM226_6;
  M[26][23] = CM226_23;
  M[26][24] = CM226_24;
  M[26][25] = CM226_25;
  M[26][26] = CM226_26;
  M[26][27] = CM127_26;
  M[26][28] = CM328_26;
  M[26][29] = CM229_26;
  M[27][1] = CM127_1;
  M[27][2] = CM127_2;
  M[27][3] = CM127_3;
  M[27][4] = CM127_4;
  M[27][5] = CM127_5;
  M[27][6] = CM127_6;
  M[27][23] = CM127_23;
  M[27][24] = CM127_24;
  M[27][25] = CM127_25;
  M[27][26] = CM127_26;
  M[27][27] = CM127_27;
  M[27][28] = CM328_27;
  M[27][29] = CM229_27;
  M[28][1] = CM328_1;
  M[28][2] = CM328_2;
  M[28][3] = CM328_3;
  M[28][4] = CM328_4;
  M[28][5] = CM328_5;
  M[28][6] = CM328_6;
  M[28][23] = CM328_23;
  M[28][24] = CM328_24;
  M[28][25] = CM328_25;
  M[28][26] = CM328_26;
  M[28][27] = CM328_27;
  M[28][28] = CM328_28;
  M[28][29] = CM229_28;
  M[29][1] = CM229_1;
  M[29][2] = CM229_2;
  M[29][3] = CM229_3;
  M[29][4] = CM229_4;
  M[29][5] = CM229_5;
  M[29][6] = CM229_6;
  M[29][23] = CM229_23;
  M[29][24] = CM229_24;
  M[29][25] = CM229_25;
  M[29][26] = CM229_26;
  M[29][27] = CM229_27;
  M[29][28] = CM229_28;
  M[29][29] = CM229_29;
  M[30][1] = CM230_1;
  M[30][2] = CM230_2;
  M[30][3] = CM230_3;
  M[30][4] = CM230_4;
  M[30][5] = CM230_5;
  M[30][6] = CM230_6;
  M[30][23] = CM230_23;
  M[30][24] = CM230_24;
  M[30][25] = CM230_25;
  M[30][30] = CM230_30;
  M[30][31] = CM131_30;
  M[30][32] = CM332_30;
  M[30][33] = CM233_30;
  M[31][1] = CM131_1;
  M[31][2] = CM131_2;
  M[31][3] = CM131_3;
  M[31][4] = CM131_4;
  M[31][5] = CM131_5;
  M[31][6] = CM131_6;
  M[31][23] = CM131_23;
  M[31][24] = CM131_24;
  M[31][25] = CM131_25;
  M[31][30] = CM131_30;
  M[31][31] = CM131_31;
  M[31][32] = CM332_31;
  M[31][33] = CM233_31;
  M[32][1] = CM332_1;
  M[32][2] = CM332_2;
  M[32][3] = CM332_3;
  M[32][4] = CM332_4;
  M[32][5] = CM332_5;
  M[32][6] = CM332_6;
  M[32][23] = CM332_23;
  M[32][24] = CM332_24;
  M[32][25] = CM332_25;
  M[32][30] = CM332_30;
  M[32][31] = CM332_31;
  M[32][32] = CM332_32;
  M[32][33] = CM233_32;
  M[33][1] = CM233_1;
  M[33][2] = CM233_2;
  M[33][3] = CM233_3;
  M[33][4] = CM233_4;
  M[33][5] = CM233_5;
  M[33][6] = CM233_6;
  M[33][23] = CM233_23;
  M[33][24] = CM233_24;
  M[33][25] = CM233_25;
  M[33][30] = CM233_30;
  M[33][31] = CM233_31;
  M[33][32] = CM233_32;
  M[33][33] = CM233_33;
  c[1] = FF4_15;
  c[2] = FF3_24;
  c[3] = FF3_34;
  c[4] = CF4_15;
  c[5] = CF5_26;
  c[6] = CF36;
  c[7] = CF27;
  c[8] = CF18;
  c[9] = CF39;
  c[10] = CF210;
  c[11] = CF111;
  c[12] = CF212;
  c[13] = FA313;
  c[14] = FA314;
  c[15] = CF215;
  c[16] = CF116;
  c[17] = CF317;
  c[18] = CF218;
  c[19] = CF119;
  c[20] = CF220;
  c[21] = FA321;
  c[22] = FA322;
  c[23] = CF123;
  c[24] = CF224;
  c[25] = CF325;
  c[26] = CF226;
  c[27] = CF127;
  c[28] = CF328;
  c[29] = CF229;
  c[30] = CF230;
  c[31] = CF131;
  c[32] = CF332;
  c[33] = CF233;

// ====== END Task 0 ====== 


}
 

