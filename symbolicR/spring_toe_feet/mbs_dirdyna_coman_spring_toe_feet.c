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
//	==> Generation Date : Tue May 24 15:23:11 2016
//
//	==> Project name : coman_spring_toe_feet
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 18820
//
//	==> Generation Time :  0.200 seconds
//	==> Post-Processing :  0.250 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[31][31];
// double c[31];
{ 
 
#include "mbs_dirdyna_coman_spring_toe_feet.h" 
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
 
// Trigonometric Variables  

  S12p13 = C12*S13+S12*C13;
  C12p13 = C12*C13-S12*S13;
 
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
  OM113 = OM112*C13-OM312*S13;
  OM213 = qd[13]+OM212;
  OM313 = OM112*S13+OM312*C13;
  OpF113 = C13*(OpF112-qd[13]*OM312)-S13*(OpF312+qd[13]*OM112);
  OpF313 = C13*(OpF312+qd[13]*OM112)+S13*(OpF112-qd[13]*OM312);
  BS313 = OM113*OM313;
  OpM113_4 = OpM112_4*C13-OpM312_4*S13;
  OpM313_4 = OpM112_4*S13+OpM312_4*C13;
  OpM113_5 = OpM112_5*C13-OpM312_5*S13;
  OpM313_5 = OpM112_5*S13+OpM312_5*C13;
  OpM113_6 = OpM112_6*C13-OpM312_6*S13;
  OpM313_6 = OpM112_6*S13+OpM312_6*C13;
  OpM113_7 = OpM112_7*C13-OpM312_7*S13;
  OpM313_7 = OpM112_7*S13+OpM312_7*C13;
  OpM113_8 = OpM112_8*C13-OpM312_8*S13;
  OpM313_8 = OpM112_8*S13+OpM312_8*C13;
  OpM113_9 = OpM112_9*C13-OpM312_9*S13;
  OpM313_9 = OpM112_9*S13+OpM312_9*C13;
  OpM113_10 = S12p13*S11;
  OpM313_10 = -C12p13*S11;

// = = Block_0_1_0_1_0_3 = = 
 
// Trigonometric Variables  

  S19p20 = C19*S20+S19*C20;
  C19p20 = C19*C20-S19*S20;
 
// Forward Kinematics 

  OM114 = OM16*C14-OM36*S14;
  OM214 = qd[14]+OM26;
  OM314 = OM16*S14+OM36*C14;
  OpF114 = C14*(OpF16-qd[14]*OM36)-S14*(OpF35+qd[14]*OM16);
  OpF314 = C14*(OpF35+qd[14]*OM16)+S14*(OpF16-qd[14]*OM36);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS514 = -(OM114*OM114+OM314*OM314);
  BS614 = OM214*OM314;
  BeF214 = BS214-OpF314;
  BeF814 = BS614+OpF114;
  AlF114 = C14*(AlF16+BeF26*s->dpt[2][3])-S14*(AlF35+BeF86*s->dpt[2][3]);
  AlF214 = AlF26+BS56*s->dpt[2][3];
  AlF314 = C14*(AlF35+BeF86*s->dpt[2][3])+S14*(AlF16+BeF26*s->dpt[2][3]);
  AlM114_1 = AlM16_1*C14-S14*S5;
  AlM314_1 = AlM16_1*S14+C14*S5;
  AlM114_2 = AlM16_2*C14-AlM35_2*S14;
  AlM314_2 = AlM16_2*S14+AlM35_2*C14;
  AlM114_3 = AlM16_3*C14-AlM35_3*S14;
  AlM314_3 = AlM16_3*S14+AlM35_3*C14;
  OpM114_4 = OpM16_4*C14-S14*S5;
  OpM314_4 = OpM16_4*S14+C14*S5;
  AlM114_4 = -s->dpt[2][3]*(OpM16_4*S14+C14*S5);
  AlM314_4 = s->dpt[2][3]*(OpM16_4*C14-S14*S5);
  OpM114_5 = C14*S6;
  OpM314_5 = S14*S6;
  AlM114_5 = -s->dpt[2][3]*S14*S6;
  AlM314_5 = s->dpt[2][3]*C14*S6;
  AlM114_6 = -s->dpt[2][3]*C14;
  AlM314_6 = -s->dpt[2][3]*S14;
  OM115 = qd[15]+OM114;
  OM215 = OM214*C15+OM314*S15;
  OM315 = -(OM214*S15-OM314*C15);
  OpF215 = C15*(OpF26+qd[15]*OM314)+S15*(OpF314-qd[15]*OM214);
  OpF315 = C15*(OpF314-qd[15]*OM214)-S15*(OpF26+qd[15]*OM314);
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS615 = OM215*OM315;
  BS915 = -(OM115*OM115+OM215*OM215);
  BeF315 = BS315+OpF215;
  BeF615 = BS615-OpF114;
  AlF115 = AlF114+BeF214*s->dpt[2][22];
  AlF215 = C15*(AlF214+BS514*s->dpt[2][22])+S15*(AlF314+BeF814*s->dpt[2][22]);
  AlF315 = C15*(AlF314+BeF814*s->dpt[2][22])-S15*(AlF214+BS514*s->dpt[2][22]);
  AlM215_1 = AlM26_1*C15+AlM314_1*S15;
  AlM315_1 = -(AlM26_1*S15-AlM314_1*C15);
  AlM215_2 = AlM26_2*C15+AlM314_2*S15;
  AlM315_2 = -(AlM26_2*S15-AlM314_2*C15);
  AlM215_3 = AlM26_3*C15+AlM314_3*S15;
  AlM315_3 = -(AlM26_3*S15-AlM314_3*C15);
  OpM215_4 = OpM26_4*C15+OpM314_4*S15;
  OpM315_4 = -(OpM26_4*S15-OpM314_4*C15);
  AlM115_4 = AlM114_4-OpM314_4*s->dpt[2][22];
  AlM215_4 = S15*(AlM314_4+OpM114_4*s->dpt[2][22]);
  AlM315_4 = C15*(AlM314_4+OpM114_4*s->dpt[2][22]);
  OpM215_5 = OpM314_5*S15+C15*C6;
  OpM315_5 = OpM314_5*C15-S15*C6;
  AlM115_5 = AlM114_5-OpM314_5*s->dpt[2][22];
  AlM215_5 = S15*(AlM314_5+OpM114_5*s->dpt[2][22]);
  AlM315_5 = C15*(AlM314_5+OpM114_5*s->dpt[2][22]);
  OpM215_6 = C14*S15;
  OpM315_6 = C14*C15;
  AlM115_6 = AlM114_6-s->dpt[2][22]*C14;
  AlM215_6 = S15*(AlM314_6-s->dpt[2][22]*S14);
  AlM315_6 = C15*(AlM314_6-s->dpt[2][22]*S14);
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]+OM315;
  OpF116 = C16*(OpF114+qd[16]*OM215)+S16*(OpF215-qd[16]*OM115);
  OpF216 = C16*(OpF215-qd[16]*OM115)-S16*(OpF114+qd[16]*OM215);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BeF316 = BS316+OpF216;
  BeF616 = BS616-OpF116;
  AlF116 = C16*(AlF115+BeF315*s->dpt[3][24])+S16*(AlF215+BeF615*s->dpt[3][24]);
  AlF216 = C16*(AlF215+BeF615*s->dpt[3][24])-S16*(AlF115+BeF315*s->dpt[3][24]);
  AlF316 = AlF315+BS915*s->dpt[3][24];
  AlM116_1 = AlM114_1*C16+AlM215_1*S16;
  AlM216_1 = -(AlM114_1*S16-AlM215_1*C16);
  AlM116_2 = AlM114_2*C16+AlM215_2*S16;
  AlM216_2 = -(AlM114_2*S16-AlM215_2*C16);
  AlM116_3 = AlM114_3*C16+AlM215_3*S16;
  AlM216_3 = -(AlM114_3*S16-AlM215_3*C16);
  OpM116_4 = OpM114_4*C16+OpM215_4*S16;
  OpM216_4 = -(OpM114_4*S16-OpM215_4*C16);
  AlM116_4 = C16*(AlM115_4+OpM215_4*s->dpt[3][24])+S16*(AlM215_4-OpM114_4*s->dpt[3][24]);
  AlM216_4 = C16*(AlM215_4-OpM114_4*s->dpt[3][24])-S16*(AlM115_4+OpM215_4*s->dpt[3][24]);
  OpM116_5 = OpM114_5*C16+OpM215_5*S16;
  OpM216_5 = -(OpM114_5*S16-OpM215_5*C16);
  AlM116_5 = C16*(AlM115_5+OpM215_5*s->dpt[3][24])+S16*(AlM215_5-OpM114_5*s->dpt[3][24]);
  AlM216_5 = C16*(AlM215_5-OpM114_5*s->dpt[3][24])-S16*(AlM115_5+OpM215_5*s->dpt[3][24]);
  OpM116_6 = OpM215_6*S16-S14*C16;
  OpM216_6 = OpM215_6*C16+S14*S16;
  AlM116_6 = C16*(AlM115_6+OpM215_6*s->dpt[3][24])+S16*(AlM215_6+s->dpt[3][24]*S14);
  AlM216_6 = C16*(AlM215_6+s->dpt[3][24]*S14)-S16*(AlM115_6+OpM215_6*s->dpt[3][24]);
  OpM116_14 = C15*S16;
  OpM216_14 = C15*C16;
  AlM116_14 = s->dpt[3][24]*C15*C16;
  AlM216_14 = -s->dpt[3][24]*C15*S16;
  AlM116_15 = -s->dpt[3][24]*S16;
  AlM216_15 = -s->dpt[3][24]*C16;
  OM117 = OM116*C17-OM316*S17;
  OM217 = qd[17]+OM216;
  OM317 = OM116*S17+OM316*C17;
  OpF117 = C17*(OpF116-qd[17]*OM316)-S17*(OpF315+qd[17]*OM116);
  OpF317 = C17*(OpF315+qd[17]*OM116)+S17*(OpF116-qd[17]*OM316);
  BS217 = OM117*OM217;
  BS317 = OM117*OM317;
  BS617 = OM217*OM317;
  BS917 = -(OM117*OM117+OM217*OM217);
  BeF317 = BS317+OpF216;
  BeF617 = BS617-OpF117;
  AlF117 = C17*(AlF116+BeF316*s->dpt[3][27])-S17*(AlF316+BS916*s->dpt[3][27]);
  AlF217 = AlF216+BeF616*s->dpt[3][27];
  AlF317 = C17*(AlF316+BS916*s->dpt[3][27])+S17*(AlF116+BeF316*s->dpt[3][27]);
  AlM117_1 = AlM116_1*C17-AlM315_1*S17;
  AlM317_1 = AlM116_1*S17+AlM315_1*C17;
  AlM117_2 = AlM116_2*C17-AlM315_2*S17;
  AlM317_2 = AlM116_2*S17+AlM315_2*C17;
  AlM117_3 = AlM116_3*C17-AlM315_3*S17;
  AlM317_3 = AlM116_3*S17+AlM315_3*C17;
  OpM117_4 = OpM116_4*C17-OpM315_4*S17;
  OpM317_4 = OpM116_4*S17+OpM315_4*C17;
  AlM117_4 = -(AlM315_4*S17-C17*(AlM116_4+OpM216_4*s->dpt[3][27]));
  AlM217_4 = AlM216_4-OpM116_4*s->dpt[3][27];
  AlM317_4 = AlM315_4*C17+S17*(AlM116_4+OpM216_4*s->dpt[3][27]);
  OpM117_5 = OpM116_5*C17-OpM315_5*S17;
  OpM317_5 = OpM116_5*S17+OpM315_5*C17;
  AlM117_5 = -(AlM315_5*S17-C17*(AlM116_5+OpM216_5*s->dpt[3][27]));
  AlM217_5 = AlM216_5-OpM116_5*s->dpt[3][27];
  AlM317_5 = AlM315_5*C17+S17*(AlM116_5+OpM216_5*s->dpt[3][27]);
  OpM117_6 = OpM116_6*C17-OpM315_6*S17;
  OpM317_6 = OpM116_6*S17+OpM315_6*C17;
  AlM117_6 = -(AlM315_6*S17-C17*(AlM116_6+OpM216_6*s->dpt[3][27]));
  AlM217_6 = AlM216_6-OpM116_6*s->dpt[3][27];
  AlM317_6 = AlM315_6*C17+S17*(AlM116_6+OpM216_6*s->dpt[3][27]);
  OpM117_14 = OpM116_14*C17+S15*S17;
  OpM317_14 = OpM116_14*S17-S15*C17;
  AlM117_14 = C17*(AlM116_14+OpM216_14*s->dpt[3][27]);
  AlM217_14 = AlM216_14-OpM116_14*s->dpt[3][27];
  AlM317_14 = S17*(AlM116_14+OpM216_14*s->dpt[3][27]);
  OpM117_15 = C16*C17;
  OpM317_15 = C16*S17;
  AlM117_15 = C17*(AlM116_15-s->dpt[3][27]*S16);
  AlM217_15 = AlM216_15-s->dpt[3][27]*C16;
  AlM317_15 = S17*(AlM116_15-s->dpt[3][27]*S16);
  OM118 = qd[18]+OM117;
  OM218 = OM217*C18+OM317*S18;
  OM318 = -(OM217*S18-OM317*C18);
  OpF218 = C18*(OpF216+qd[18]*OM317)+S18*(OpF317-qd[18]*OM217);
  OpF318 = C18*(OpF317-qd[18]*OM217)-S18*(OpF216+qd[18]*OM317);
  BS218 = OM118*OM218;
  BS318 = OM118*OM318;
  BS618 = OM218*OM318;
  AlF118 = AlF117+BeF317*s->dpt[3][30];
  AlF218 = C18*(AlF217+BeF617*s->dpt[3][30])+S18*(AlF317+BS917*s->dpt[3][30]);
  AlF318 = C18*(AlF317+BS917*s->dpt[3][30])-S18*(AlF217+BeF617*s->dpt[3][30]);
  AlM218_1 = AlM216_1*C18+AlM317_1*S18;
  AlM318_1 = -(AlM216_1*S18-AlM317_1*C18);
  AlM218_2 = AlM216_2*C18+AlM317_2*S18;
  AlM318_2 = -(AlM216_2*S18-AlM317_2*C18);
  AlM218_3 = AlM216_3*C18+AlM317_3*S18;
  AlM318_3 = -(AlM216_3*S18-AlM317_3*C18);
  OpM218_4 = OpM216_4*C18+OpM317_4*S18;
  OpM318_4 = -(OpM216_4*S18-OpM317_4*C18);
  AlM118_4 = AlM117_4+OpM216_4*s->dpt[3][30];
  AlM218_4 = AlM317_4*S18+C18*(AlM217_4-OpM117_4*s->dpt[3][30]);
  AlM318_4 = AlM317_4*C18-S18*(AlM217_4-OpM117_4*s->dpt[3][30]);
  OpM218_5 = OpM216_5*C18+OpM317_5*S18;
  OpM318_5 = -(OpM216_5*S18-OpM317_5*C18);
  AlM118_5 = AlM117_5+OpM216_5*s->dpt[3][30];
  AlM218_5 = AlM317_5*S18+C18*(AlM217_5-OpM117_5*s->dpt[3][30]);
  AlM318_5 = AlM317_5*C18-S18*(AlM217_5-OpM117_5*s->dpt[3][30]);
  OpM218_6 = OpM216_6*C18+OpM317_6*S18;
  OpM318_6 = -(OpM216_6*S18-OpM317_6*C18);
  AlM118_6 = AlM117_6+OpM216_6*s->dpt[3][30];
  AlM218_6 = AlM317_6*S18+C18*(AlM217_6-OpM117_6*s->dpt[3][30]);
  AlM318_6 = AlM317_6*C18-S18*(AlM217_6-OpM117_6*s->dpt[3][30]);
  OpM218_14 = OpM216_14*C18+OpM317_14*S18;
  OpM318_14 = -(OpM216_14*S18-OpM317_14*C18);
  AlM118_14 = AlM117_14+OpM216_14*s->dpt[3][30];
  AlM218_14 = AlM317_14*S18+C18*(AlM217_14-OpM117_14*s->dpt[3][30]);
  AlM318_14 = AlM317_14*C18-S18*(AlM217_14-OpM117_14*s->dpt[3][30]);
  OpM218_15 = OpM317_15*S18-S16*C18;
  OpM318_15 = OpM317_15*C18+S16*S18;
  AlM118_15 = AlM117_15-s->dpt[3][30]*S16;
  AlM218_15 = AlM317_15*S18+C18*(AlM217_15-OpM117_15*s->dpt[3][30]);
  AlM318_15 = AlM317_15*C18-S18*(AlM217_15-OpM117_15*s->dpt[3][30]);
  OpM218_16 = C17*S18;
  OpM318_16 = C17*C18;
  AlM218_16 = s->dpt[3][30]*S17*C18;
  AlM318_16 = -s->dpt[3][30]*S17*S18;
  OM119 = OM118*C19-OM318*S19;
  OM219 = qd[19]+OM218;
  OM319 = OM118*S19+OM318*C19;
  OpF119 = C19*(OpF117-qd[19]*OM318)-S19*(OpF318+qd[19]*OM118);
  OpF319 = C19*(OpF318+qd[19]*OM118)+S19*(OpF117-qd[19]*OM318);
  BS119 = -(OM219*OM219+OM319*OM319);
  BS319 = OM119*OM319;
  BS919 = -(OM119*OM119+OM219*OM219);
  BeF319 = BS319+OpF218;
  BeF419 = OpF319+OM119*OM219;
  BeF619 = OM219*OM319-OpF119;
  BeF719 = BS319-OpF218;
  AlF119 = AlF118*C19-AlF318*S19;
  AlF319 = AlF118*S19+AlF318*C19;
  AlM119_1 = AlM117_1*C19-AlM318_1*S19;
  AlM319_1 = AlM117_1*S19+AlM318_1*C19;
  AlM119_2 = AlM117_2*C19-AlM318_2*S19;
  AlM319_2 = AlM117_2*S19+AlM318_2*C19;
  AlM119_3 = AlM117_3*C19-AlM318_3*S19;
  AlM319_3 = AlM117_3*S19+AlM318_3*C19;
  OpM119_4 = OpM117_4*C19-OpM318_4*S19;
  OpM319_4 = OpM117_4*S19+OpM318_4*C19;
  AlM119_4 = AlM118_4*C19-AlM318_4*S19;
  AlM319_4 = AlM118_4*S19+AlM318_4*C19;
  OpM119_5 = OpM117_5*C19-OpM318_5*S19;
  OpM319_5 = OpM117_5*S19+OpM318_5*C19;
  AlM119_5 = AlM118_5*C19-AlM318_5*S19;
  AlM319_5 = AlM118_5*S19+AlM318_5*C19;
  OpM119_6 = OpM117_6*C19-OpM318_6*S19;
  OpM319_6 = OpM117_6*S19+OpM318_6*C19;
  AlM119_6 = AlM118_6*C19-AlM318_6*S19;
  AlM319_6 = AlM118_6*S19+AlM318_6*C19;
  OpM119_14 = OpM117_14*C19-OpM318_14*S19;
  OpM319_14 = OpM117_14*S19+OpM318_14*C19;
  AlM119_14 = AlM118_14*C19-AlM318_14*S19;
  AlM319_14 = AlM118_14*S19+AlM318_14*C19;
  OpM119_15 = OpM117_15*C19-OpM318_15*S19;
  OpM319_15 = OpM117_15*S19+OpM318_15*C19;
  AlM119_15 = AlM118_15*C19-AlM318_15*S19;
  AlM319_15 = AlM118_15*S19+AlM318_15*C19;
  OpM119_16 = -(OpM318_16*S19+S17*C19);
  OpM319_16 = OpM318_16*C19-S17*S19;
  AlM119_16 = -AlM318_16*S19;
  AlM319_16 = AlM318_16*C19;
  OpM119_17 = S18*S19;
  OpM319_17 = -S18*C19;
  AlM119_17 = s->dpt[3][30]*C19;
  AlM319_17 = s->dpt[3][30]*S19;
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM219;
  OM320 = OM119*S20+OM319*C20;
  OpF120 = C20*(OpF119-qd[20]*OM319)-S20*(OpF319+qd[20]*OM119);
  OpF320 = C20*(OpF319+qd[20]*OM119)+S20*(OpF119-qd[20]*OM319);
  BS320 = OM120*OM320;
  OpM120_4 = OpM119_4*C20-OpM319_4*S20;
  OpM320_4 = OpM119_4*S20+OpM319_4*C20;
  OpM120_5 = OpM119_5*C20-OpM319_5*S20;
  OpM320_5 = OpM119_5*S20+OpM319_5*C20;
  OpM120_6 = OpM119_6*C20-OpM319_6*S20;
  OpM320_6 = OpM119_6*S20+OpM319_6*C20;
  OpM120_14 = OpM119_14*C20-OpM319_14*S20;
  OpM320_14 = OpM119_14*S20+OpM319_14*C20;
  OpM120_15 = OpM119_15*C20-OpM319_15*S20;
  OpM320_15 = OpM119_15*S20+OpM319_15*C20;
  OpM120_16 = OpM119_16*C20-OpM319_16*S20;
  OpM320_16 = OpM119_16*S20+OpM319_16*C20;
  OpM120_17 = S19p20*S18;
  OpM320_17 = -C19p20*S18;

// = = Block_0_1_0_1_0_4 = = 
 
// Forward Kinematics 

  OM121 = qd[21]+OM16;
  OM221 = OM26*C21+OM36*S21;
  OM321 = -(OM26*S21-OM36*C21);
  OpF221 = C21*(OpF26+qd[21]*OM36)+S21*(OpF35-qd[21]*OM26);
  OpF321 = C21*(OpF35-qd[21]*OM26)-S21*(OpF26+qd[21]*OM36);
  BS221 = OM121*OM221;
  BS321 = OM121*OM321;
  BS621 = OM221*OM321;
  AlF121 = AlF16+BS16*s->dpt[1][4]+BeF36*s->dpt[3][4];
  AlF221 = C21*(AlF26+BeF46*s->dpt[1][4]+BeF66*s->dpt[3][4])+S21*(AlF35+BS96*s->dpt[3][4]+BeF76*s->dpt[1][4]);
  AlF321 = C21*(AlF35+BS96*s->dpt[3][4]+BeF76*s->dpt[1][4])-S21*(AlF26+BeF46*s->dpt[1][4]+BeF66*s->dpt[3][4]);
  AlM221_1 = AlM26_1*C21+S21*S5;
  AlM321_1 = -(AlM26_1*S21-C21*S5);
  AlM221_2 = AlM26_2*C21+AlM35_2*S21;
  AlM321_2 = -(AlM26_2*S21-AlM35_2*C21);
  AlM221_3 = AlM26_3*C21+AlM35_3*S21;
  AlM321_3 = -(AlM26_3*S21-AlM35_3*C21);
  OpM221_4 = OpM26_4*C21+S21*S5;
  OpM321_4 = -(OpM26_4*S21-C21*S5);
  AlM121_4 = OpM26_4*s->dpt[3][4];
  AlM221_4 = -(OpM26_4*s->dpt[1][4]*S21+C21*(OpM16_4*s->dpt[3][4]-s->dpt[1][4]*S5));
  AlM321_4 = -(OpM26_4*s->dpt[1][4]*C21-S21*(OpM16_4*s->dpt[3][4]-s->dpt[1][4]*S5));
  OpM221_5 = C21*C6;
  OpM321_5 = -S21*C6;
  AlM121_5 = s->dpt[3][4]*C6;
  AlM221_5 = -(s->dpt[1][4]*S21*C6+s->dpt[3][4]*C21*S6);
  AlM321_5 = -(s->dpt[1][4]*C21*C6-s->dpt[3][4]*S21*S6);
  AlM221_6 = s->dpt[1][4]*C21;
  AlM321_6 = -s->dpt[1][4]*S21;
  OM122 = OM121*C22-OM321*S22;
  OM222 = qd[22]+OM221;
  OM322 = OM121*S22+OM321*C22;
  OpF122 = C22*(OpF16-qd[22]*OM321)-S22*(OpF321+qd[22]*OM121);
  OpF322 = C22*(OpF321+qd[22]*OM121)+S22*(OpF16-qd[22]*OM321);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS622 = OM222*OM322;
  BS922 = -(OM122*OM122+OM222*OM222);
  BeF322 = BS322+OpF221;
  BeF622 = BS622-OpF122;
  AlF122 = AlF121*C22-AlF321*S22;
  AlF322 = AlF121*S22+AlF321*C22;
  AlM122_1 = AlM16_1*C22-AlM321_1*S22;
  AlM322_1 = AlM16_1*S22+AlM321_1*C22;
  AlM122_2 = AlM16_2*C22-AlM321_2*S22;
  AlM322_2 = AlM16_2*S22+AlM321_2*C22;
  AlM122_3 = AlM16_3*C22-AlM321_3*S22;
  AlM322_3 = AlM16_3*S22+AlM321_3*C22;
  OpM122_4 = OpM16_4*C22-OpM321_4*S22;
  OpM322_4 = OpM16_4*S22+OpM321_4*C22;
  AlM122_4 = AlM121_4*C22-AlM321_4*S22;
  AlM322_4 = AlM121_4*S22+AlM321_4*C22;
  OpM122_5 = -(OpM321_5*S22-C22*S6);
  OpM322_5 = OpM321_5*C22+S22*S6;
  AlM122_5 = AlM121_5*C22-AlM321_5*S22;
  AlM322_5 = AlM121_5*S22+AlM321_5*C22;
  OpM122_6 = -C21*S22;
  OpM322_6 = C21*C22;
  AlM122_6 = -AlM321_6*S22;
  AlM322_6 = AlM321_6*C22;
  OM123 = OM122*C23+OM222*S23;
  OM223 = -(OM122*S23-OM222*C23);
  OM323 = qd[23]+OM322;
  OpF123 = C23*(OpF122+qd[23]*OM222)+S23*(OpF221-qd[23]*OM122);
  OpF223 = C23*(OpF221-qd[23]*OM122)-S23*(OpF122+qd[23]*OM222);
  BS123 = -(OM223*OM223+OM323*OM323);
  BS223 = OM123*OM223;
  BS323 = OM123*OM323;
  BS523 = -(OM123*OM123+OM323*OM323);
  BS623 = OM223*OM323;
  BS923 = -(OM123*OM123+OM223*OM223);
  BeF223 = BS223-OpF322;
  BeF323 = BS323+OpF223;
  BeF423 = BS223+OpF322;
  BeF623 = BS623-OpF123;
  BeF723 = BS323-OpF223;
  BeF823 = BS623+OpF123;
  AlF123 = C23*(AlF122+BeF322*s->dpt[3][40])+S23*(AlF221+BeF622*s->dpt[3][40]);
  AlF223 = C23*(AlF221+BeF622*s->dpt[3][40])-S23*(AlF122+BeF322*s->dpt[3][40]);
  AlF323 = AlF322+BS922*s->dpt[3][40];
  AlM123_1 = AlM122_1*C23+AlM221_1*S23;
  AlM223_1 = -(AlM122_1*S23-AlM221_1*C23);
  AlM123_2 = AlM122_2*C23+AlM221_2*S23;
  AlM223_2 = -(AlM122_2*S23-AlM221_2*C23);
  AlM123_3 = AlM122_3*C23+AlM221_3*S23;
  AlM223_3 = -(AlM122_3*S23-AlM221_3*C23);
  OpM123_4 = OpM122_4*C23+OpM221_4*S23;
  OpM223_4 = -(OpM122_4*S23-OpM221_4*C23);
  AlM123_4 = C23*(AlM122_4+OpM221_4*s->dpt[3][40])+S23*(AlM221_4-OpM122_4*s->dpt[3][40]);
  AlM223_4 = C23*(AlM221_4-OpM122_4*s->dpt[3][40])-S23*(AlM122_4+OpM221_4*s->dpt[3][40]);
  OpM123_5 = OpM122_5*C23+OpM221_5*S23;
  OpM223_5 = -(OpM122_5*S23-OpM221_5*C23);
  AlM123_5 = C23*(AlM122_5+OpM221_5*s->dpt[3][40])+S23*(AlM221_5-OpM122_5*s->dpt[3][40]);
  AlM223_5 = C23*(AlM221_5-OpM122_5*s->dpt[3][40])-S23*(AlM122_5+OpM221_5*s->dpt[3][40]);
  OpM123_6 = OpM122_6*C23+S21*S23;
  OpM223_6 = -(OpM122_6*S23-S21*C23);
  AlM123_6 = C23*(AlM122_6+s->dpt[3][40]*S21)+S23*(AlM221_6-OpM122_6*s->dpt[3][40]);
  AlM223_6 = C23*(AlM221_6-OpM122_6*s->dpt[3][40])-S23*(AlM122_6+s->dpt[3][40]*S21);
  OpM123_21 = C22*C23;
  OpM223_21 = -C22*S23;
  AlM123_21 = -s->dpt[3][40]*C22*S23;
  AlM223_21 = -s->dpt[3][40]*C22*C23;
  AlM223_22 = -s->dpt[3][40]*S23;

// = = Block_0_1_0_2_0_5 = = 
 
// Forward Kinematics 

  OM124 = OM123*C24-OM323*S24;
  OM224 = qd[24]+OM223;
  OM324 = OM123*S24+OM323*C24;
  OpF124 = C24*(OpF123-qd[24]*OM323)-S24*(OpF322+qd[24]*OM123);
  OpF324 = C24*(OpF322+qd[24]*OM123)+S24*(OpF123-qd[24]*OM323);
  BS224 = OM124*OM224;
  BS324 = OM124*OM324;
  BS524 = -(OM124*OM124+OM324*OM324);
  BS624 = OM224*OM324;
  BeF224 = BS224-OpF324;
  BeF824 = BS624+OpF124;
  AlF124 = C24*(AlF123+BS123*s->dpt[1][44]+BeF223*s->dpt[2][44]+BeF323*s->dpt[3][44])-S24*(AlF323+BS923*s->dpt[3][44]+
 BeF723*s->dpt[1][44]+BeF823*s->dpt[2][44]);
  AlF224 = AlF223+BS523*s->dpt[2][44]+BeF423*s->dpt[1][44]+BeF623*s->dpt[3][44];
  AlF324 = C24*(AlF323+BS923*s->dpt[3][44]+BeF723*s->dpt[1][44]+BeF823*s->dpt[2][44])+S24*(AlF123+BS123*s->dpt[1][44]+
 BeF223*s->dpt[2][44]+BeF323*s->dpt[3][44]);
  AlM124_1 = AlM123_1*C24-AlM322_1*S24;
  AlM324_1 = AlM123_1*S24+AlM322_1*C24;
  AlM124_2 = AlM123_2*C24-AlM322_2*S24;
  AlM324_2 = AlM123_2*S24+AlM322_2*C24;
  AlM124_3 = AlM123_3*C24-AlM322_3*S24;
  AlM324_3 = AlM123_3*S24+AlM322_3*C24;
  OpM124_4 = OpM123_4*C24-OpM322_4*S24;
  OpM324_4 = OpM123_4*S24+OpM322_4*C24;
  AlM124_4 = C24*(AlM123_4+OpM223_4*s->dpt[3][44]-OpM322_4*s->dpt[2][44])-S24*(AlM322_4+OpM123_4*s->dpt[2][44]-OpM223_4*
 s->dpt[1][44]);
  AlM224_4 = AlM223_4-OpM123_4*s->dpt[3][44]+OpM322_4*s->dpt[1][44];
  AlM324_4 = C24*(AlM322_4+OpM123_4*s->dpt[2][44]-OpM223_4*s->dpt[1][44])+S24*(AlM123_4+OpM223_4*s->dpt[3][44]-OpM322_4*
 s->dpt[2][44]);
  OpM124_5 = OpM123_5*C24-OpM322_5*S24;
  OpM324_5 = OpM123_5*S24+OpM322_5*C24;
  AlM124_5 = C24*(AlM123_5+OpM223_5*s->dpt[3][44]-OpM322_5*s->dpt[2][44])-S24*(AlM322_5+OpM123_5*s->dpt[2][44]-OpM223_5*
 s->dpt[1][44]);
  AlM224_5 = AlM223_5-OpM123_5*s->dpt[3][44]+OpM322_5*s->dpt[1][44];
  AlM324_5 = C24*(AlM322_5+OpM123_5*s->dpt[2][44]-OpM223_5*s->dpt[1][44])+S24*(AlM123_5+OpM223_5*s->dpt[3][44]-OpM322_5*
 s->dpt[2][44]);
  OpM124_6 = OpM123_6*C24-OpM322_6*S24;
  OpM324_6 = OpM123_6*S24+OpM322_6*C24;
  AlM124_6 = C24*(AlM123_6+OpM223_6*s->dpt[3][44]-OpM322_6*s->dpt[2][44])-S24*(AlM322_6+OpM123_6*s->dpt[2][44]-OpM223_6*
 s->dpt[1][44]);
  AlM224_6 = AlM223_6-OpM123_6*s->dpt[3][44]+OpM322_6*s->dpt[1][44];
  AlM324_6 = C24*(AlM322_6+OpM123_6*s->dpt[2][44]-OpM223_6*s->dpt[1][44])+S24*(AlM123_6+OpM223_6*s->dpt[3][44]-OpM322_6*
 s->dpt[2][44]);
  OpM124_21 = OpM123_21*C24-S22*S24;
  OpM324_21 = OpM123_21*S24+S22*C24;
  AlM124_21 = C24*(AlM123_21+OpM223_21*s->dpt[3][44]-s->dpt[2][44]*S22)-S24*(OpM123_21*s->dpt[2][44]-OpM223_21*
 s->dpt[1][44]);
  AlM224_21 = AlM223_21-OpM123_21*s->dpt[3][44]+s->dpt[1][44]*S22;
  AlM324_21 = C24*(OpM123_21*s->dpt[2][44]-OpM223_21*s->dpt[1][44])+S24*(AlM123_21+OpM223_21*s->dpt[3][44]-s->dpt[2][44]
 *S22);
  OpM124_22 = S23*C24;
  OpM324_22 = S23*S24;
  AlM124_22 = C23*C24*(s->dpt[3][40]+s->dpt[3][44])+S24*(s->dpt[1][44]*C23-s->dpt[2][44]*S23);
  AlM224_22 = AlM223_22-s->dpt[3][44]*S23;
  AlM324_22 = C23*S24*(s->dpt[3][40]+s->dpt[3][44])-C24*(s->dpt[1][44]*C23-s->dpt[2][44]*S23);
  AlM124_23 = -s->dpt[2][44]*C24;
  OM125 = qd[25]+OM124;
  OM225 = OM224*C25+OM324*S25;
  OM325 = -(OM224*S25-OM324*C25);
  OpF225 = C25*(OpF223+qd[25]*OM324)+S25*(OpF324-qd[25]*OM224);
  OpF325 = C25*(OpF324-qd[25]*OM224)-S25*(OpF223+qd[25]*OM324);
  BS225 = OM125*OM225;
  BS325 = OM125*OM325;
  BS625 = OM225*OM325;
  BS925 = -(OM125*OM125+OM225*OM225);
  BeF325 = BS325+OpF225;
  BeF625 = BS625-OpF124;
  AlF125 = AlF124+BeF224*s->dpt[2][47];
  AlF225 = C25*(AlF224+BS524*s->dpt[2][47])+S25*(AlF324+BeF824*s->dpt[2][47]);
  AlF325 = C25*(AlF324+BeF824*s->dpt[2][47])-S25*(AlF224+BS524*s->dpt[2][47]);
  AlM225_1 = AlM223_1*C25+AlM324_1*S25;
  AlM325_1 = -(AlM223_1*S25-AlM324_1*C25);
  AlM225_2 = AlM223_2*C25+AlM324_2*S25;
  AlM325_2 = -(AlM223_2*S25-AlM324_2*C25);
  AlM225_3 = AlM223_3*C25+AlM324_3*S25;
  AlM325_3 = -(AlM223_3*S25-AlM324_3*C25);
  OpM225_4 = OpM223_4*C25+OpM324_4*S25;
  OpM325_4 = -(OpM223_4*S25-OpM324_4*C25);
  AlM125_4 = AlM124_4-OpM324_4*s->dpt[2][47];
  AlM225_4 = AlM224_4*C25+S25*(AlM324_4+OpM124_4*s->dpt[2][47]);
  AlM325_4 = -(AlM224_4*S25-C25*(AlM324_4+OpM124_4*s->dpt[2][47]));
  OpM225_5 = OpM223_5*C25+OpM324_5*S25;
  OpM325_5 = -(OpM223_5*S25-OpM324_5*C25);
  AlM125_5 = AlM124_5-OpM324_5*s->dpt[2][47];
  AlM225_5 = AlM224_5*C25+S25*(AlM324_5+OpM124_5*s->dpt[2][47]);
  AlM325_5 = -(AlM224_5*S25-C25*(AlM324_5+OpM124_5*s->dpt[2][47]));
  OpM225_6 = OpM223_6*C25+OpM324_6*S25;
  OpM325_6 = -(OpM223_6*S25-OpM324_6*C25);
  AlM125_6 = AlM124_6-OpM324_6*s->dpt[2][47];
  AlM225_6 = AlM224_6*C25+S25*(AlM324_6+OpM124_6*s->dpt[2][47]);
  AlM325_6 = -(AlM224_6*S25-C25*(AlM324_6+OpM124_6*s->dpt[2][47]));
  OpM225_21 = OpM223_21*C25+OpM324_21*S25;
  OpM325_21 = -(OpM223_21*S25-OpM324_21*C25);
  AlM125_21 = AlM124_21-OpM324_21*s->dpt[2][47];
  AlM225_21 = AlM224_21*C25+S25*(AlM324_21+OpM124_21*s->dpt[2][47]);
  AlM325_21 = -(AlM224_21*S25-C25*(AlM324_21+OpM124_21*s->dpt[2][47]));
  OpM225_22 = OpM324_22*S25+C23*C25;
  OpM325_22 = OpM324_22*C25-C23*S25;
  AlM125_22 = AlM124_22-OpM324_22*s->dpt[2][47];
  AlM225_22 = AlM224_22*C25+S25*(AlM324_22+OpM124_22*s->dpt[2][47]);
  AlM325_22 = -(AlM224_22*S25-C25*(AlM324_22+OpM124_22*s->dpt[2][47]));
  OpM225_23 = C24*S25;
  OpM325_23 = C24*C25;
  AlM125_23 = AlM124_23-s->dpt[2][47]*C24;
  AlM225_23 = s->dpt[1][44]*C25-S24*S25*(s->dpt[2][44]+s->dpt[2][47]);
  AlM325_23 = -(s->dpt[1][44]*S25+S24*C25*(s->dpt[2][44]+s->dpt[2][47]));
  OM126 = OM125*C26+OM225*S26;
  OM226 = -(OM125*S26-OM225*C26);
  OM326 = qd[26]+OM325;
  OpF126 = C26*(OpF124+qd[26]*OM225)+S26*(OpF225-qd[26]*OM125);
  OpF226 = C26*(OpF225-qd[26]*OM125)-S26*(OpF124+qd[26]*OM225);
  BS226 = OM126*OM226;
  BS326 = OM126*OM326;
  BS626 = OM226*OM326;
  BS926 = -(OM126*OM126+OM226*OM226);
  BeF326 = BS326+OpF226;
  BeF626 = BS626-OpF126;
  AlF126 = C26*(AlF125+BeF325*s->dpt[3][49])+S26*(AlF225+BeF625*s->dpt[3][49]);
  AlF226 = C26*(AlF225+BeF625*s->dpt[3][49])-S26*(AlF125+BeF325*s->dpt[3][49]);
  AlF326 = AlF325+BS925*s->dpt[3][49];
  AlM126_1 = AlM124_1*C26+AlM225_1*S26;
  AlM226_1 = -(AlM124_1*S26-AlM225_1*C26);
  AlM126_2 = AlM124_2*C26+AlM225_2*S26;
  AlM226_2 = -(AlM124_2*S26-AlM225_2*C26);
  AlM126_3 = AlM124_3*C26+AlM225_3*S26;
  AlM226_3 = -(AlM124_3*S26-AlM225_3*C26);
  OpM126_4 = OpM124_4*C26+OpM225_4*S26;
  OpM226_4 = -(OpM124_4*S26-OpM225_4*C26);
  AlM126_4 = C26*(AlM125_4+OpM225_4*s->dpt[3][49])+S26*(AlM225_4-OpM124_4*s->dpt[3][49]);
  AlM226_4 = C26*(AlM225_4-OpM124_4*s->dpt[3][49])-S26*(AlM125_4+OpM225_4*s->dpt[3][49]);
  OpM126_5 = OpM124_5*C26+OpM225_5*S26;
  OpM226_5 = -(OpM124_5*S26-OpM225_5*C26);
  AlM126_5 = C26*(AlM125_5+OpM225_5*s->dpt[3][49])+S26*(AlM225_5-OpM124_5*s->dpt[3][49]);
  AlM226_5 = C26*(AlM225_5-OpM124_5*s->dpt[3][49])-S26*(AlM125_5+OpM225_5*s->dpt[3][49]);
  OpM126_6 = OpM124_6*C26+OpM225_6*S26;
  OpM226_6 = -(OpM124_6*S26-OpM225_6*C26);
  AlM126_6 = C26*(AlM125_6+OpM225_6*s->dpt[3][49])+S26*(AlM225_6-OpM124_6*s->dpt[3][49]);
  AlM226_6 = C26*(AlM225_6-OpM124_6*s->dpt[3][49])-S26*(AlM125_6+OpM225_6*s->dpt[3][49]);
  OpM126_21 = OpM124_21*C26+OpM225_21*S26;
  OpM226_21 = -(OpM124_21*S26-OpM225_21*C26);
  AlM126_21 = C26*(AlM125_21+OpM225_21*s->dpt[3][49])+S26*(AlM225_21-OpM124_21*s->dpt[3][49]);
  AlM226_21 = C26*(AlM225_21-OpM124_21*s->dpt[3][49])-S26*(AlM125_21+OpM225_21*s->dpt[3][49]);
  OpM126_22 = OpM124_22*C26+OpM225_22*S26;
  OpM226_22 = -(OpM124_22*S26-OpM225_22*C26);
  AlM126_22 = C26*(AlM125_22+OpM225_22*s->dpt[3][49])+S26*(AlM225_22-OpM124_22*s->dpt[3][49]);
  AlM226_22 = C26*(AlM225_22-OpM124_22*s->dpt[3][49])-S26*(AlM125_22+OpM225_22*s->dpt[3][49]);
  OpM126_23 = OpM225_23*S26-S24*C26;
  OpM226_23 = OpM225_23*C26+S24*S26;
  AlM126_23 = C26*(AlM125_23+OpM225_23*s->dpt[3][49])+S26*(AlM225_23+s->dpt[3][49]*S24);
  AlM226_23 = C26*(AlM225_23+s->dpt[3][49]*S24)-S26*(AlM125_23+OpM225_23*s->dpt[3][49]);
  OpM126_24 = C25*S26;
  OpM226_24 = C25*C26;
  AlM126_24 = s->dpt[3][49]*C25*C26;
  AlM226_24 = -s->dpt[3][49]*C25*S26;
  AlM126_25 = -s->dpt[3][49]*S26;
  AlM226_25 = -s->dpt[3][49]*C26;
  OM127 = OM126*C27-OM326*S27;
  OM227 = qd[27]+OM226;
  OM327 = OM126*S27+OM326*C27;
  OpF127 = C27*(OpF126-qd[27]*OM326)-S27*(OpF325+qd[27]*OM126);
  OpF327 = C27*(OpF325+qd[27]*OM126)+S27*(OpF126-qd[27]*OM326);
  BS227 = OM127*OM227;
  BS327 = OM127*OM327;
  BS627 = OM227*OM327;
  OpM127_4 = OpM126_4*C27-OpM325_4*S27;
  OpM327_4 = OpM126_4*S27+OpM325_4*C27;
  OpM127_5 = OpM126_5*C27-OpM325_5*S27;
  OpM327_5 = OpM126_5*S27+OpM325_5*C27;
  OpM127_6 = OpM126_6*C27-OpM325_6*S27;
  OpM327_6 = OpM126_6*S27+OpM325_6*C27;
  OpM127_21 = OpM126_21*C27-OpM325_21*S27;
  OpM327_21 = OpM126_21*S27+OpM325_21*C27;
  OpM127_22 = OpM126_22*C27-OpM325_22*S27;
  OpM327_22 = OpM126_22*S27+OpM325_22*C27;
  OpM127_23 = OpM126_23*C27-OpM325_23*S27;
  OpM327_23 = OpM126_23*S27+OpM325_23*C27;
  OpM127_24 = OpM126_24*C27+S25*S27;
  OpM327_24 = OpM126_24*S27-S25*C27;
  OpM127_25 = C26*C27;
  OpM327_25 = C26*S27;

// = = Block_0_1_0_2_0_6 = = 
 
// Forward Kinematics 

  OM128 = OM123*C28-OM323*S28;
  OM228 = qd[28]+OM223;
  OM328 = OM123*S28+OM323*C28;
  OpF128 = C28*(OpF123-qd[28]*OM323)-S28*(OpF322+qd[28]*OM123);
  OpF328 = C28*(OpF322+qd[28]*OM123)+S28*(OpF123-qd[28]*OM323);
  BS228 = OM128*OM228;
  BS328 = OM128*OM328;
  BS528 = -(OM128*OM128+OM328*OM328);
  BS628 = OM228*OM328;
  BeF228 = BS228-OpF328;
  BeF828 = BS628+OpF128;
  AlF128 = C28*(AlF123+BS123*s->dpt[1][45]+BeF223*s->dpt[2][45]+BeF323*s->dpt[3][45])-S28*(AlF323+BS923*s->dpt[3][45]+
 BeF723*s->dpt[1][45]+BeF823*s->dpt[2][45]);
  AlF228 = AlF223+BS523*s->dpt[2][45]+BeF423*s->dpt[1][45]+BeF623*s->dpt[3][45];
  AlF328 = C28*(AlF323+BS923*s->dpt[3][45]+BeF723*s->dpt[1][45]+BeF823*s->dpt[2][45])+S28*(AlF123+BS123*s->dpt[1][45]+
 BeF223*s->dpt[2][45]+BeF323*s->dpt[3][45]);
  AlM128_1 = AlM123_1*C28-AlM322_1*S28;
  AlM328_1 = AlM123_1*S28+AlM322_1*C28;
  AlM128_2 = AlM123_2*C28-AlM322_2*S28;
  AlM328_2 = AlM123_2*S28+AlM322_2*C28;
  AlM128_3 = AlM123_3*C28-AlM322_3*S28;
  AlM328_3 = AlM123_3*S28+AlM322_3*C28;
  OpM128_4 = OpM123_4*C28-OpM322_4*S28;
  OpM328_4 = OpM123_4*S28+OpM322_4*C28;
  AlM128_4 = C28*(AlM123_4+OpM223_4*s->dpt[3][45]-OpM322_4*s->dpt[2][45])-S28*(AlM322_4+OpM123_4*s->dpt[2][45]-OpM223_4*
 s->dpt[1][45]);
  AlM228_4 = AlM223_4-OpM123_4*s->dpt[3][45]+OpM322_4*s->dpt[1][45];
  AlM328_4 = C28*(AlM322_4+OpM123_4*s->dpt[2][45]-OpM223_4*s->dpt[1][45])+S28*(AlM123_4+OpM223_4*s->dpt[3][45]-OpM322_4*
 s->dpt[2][45]);
  OpM128_5 = OpM123_5*C28-OpM322_5*S28;
  OpM328_5 = OpM123_5*S28+OpM322_5*C28;
  AlM128_5 = C28*(AlM123_5+OpM223_5*s->dpt[3][45]-OpM322_5*s->dpt[2][45])-S28*(AlM322_5+OpM123_5*s->dpt[2][45]-OpM223_5*
 s->dpt[1][45]);
  AlM228_5 = AlM223_5-OpM123_5*s->dpt[3][45]+OpM322_5*s->dpt[1][45];
  AlM328_5 = C28*(AlM322_5+OpM123_5*s->dpt[2][45]-OpM223_5*s->dpt[1][45])+S28*(AlM123_5+OpM223_5*s->dpt[3][45]-OpM322_5*
 s->dpt[2][45]);
  OpM128_6 = OpM123_6*C28-OpM322_6*S28;
  OpM328_6 = OpM123_6*S28+OpM322_6*C28;
  AlM128_6 = C28*(AlM123_6+OpM223_6*s->dpt[3][45]-OpM322_6*s->dpt[2][45])-S28*(AlM322_6+OpM123_6*s->dpt[2][45]-OpM223_6*
 s->dpt[1][45]);
  AlM228_6 = AlM223_6-OpM123_6*s->dpt[3][45]+OpM322_6*s->dpt[1][45];
  AlM328_6 = C28*(AlM322_6+OpM123_6*s->dpt[2][45]-OpM223_6*s->dpt[1][45])+S28*(AlM123_6+OpM223_6*s->dpt[3][45]-OpM322_6*
 s->dpt[2][45]);
  OpM128_21 = OpM123_21*C28-S22*S28;
  OpM328_21 = OpM123_21*S28+S22*C28;
  AlM128_21 = C28*(AlM123_21+OpM223_21*s->dpt[3][45]-s->dpt[2][45]*S22)-S28*(OpM123_21*s->dpt[2][45]-OpM223_21*
 s->dpt[1][45]);
  AlM228_21 = AlM223_21-OpM123_21*s->dpt[3][45]+s->dpt[1][45]*S22;
  AlM328_21 = C28*(OpM123_21*s->dpt[2][45]-OpM223_21*s->dpt[1][45])+S28*(AlM123_21+OpM223_21*s->dpt[3][45]-s->dpt[2][45]
 *S22);
  OpM128_22 = S23*C28;
  OpM328_22 = S23*S28;
  AlM128_22 = C23*C28*(s->dpt[3][40]+s->dpt[3][45])+S28*(s->dpt[1][45]*C23-s->dpt[2][45]*S23);
  AlM228_22 = AlM223_22-s->dpt[3][45]*S23;
  AlM328_22 = C23*S28*(s->dpt[3][40]+s->dpt[3][45])-C28*(s->dpt[1][45]*C23-s->dpt[2][45]*S23);
  AlM128_23 = -s->dpt[2][45]*C28;
  OM129 = qd[29]+OM128;
  OM229 = OM228*C29+OM328*S29;
  OM329 = -(OM228*S29-OM328*C29);
  OpF229 = C29*(OpF223+qd[29]*OM328)+S29*(OpF328-qd[29]*OM228);
  OpF329 = C29*(OpF328-qd[29]*OM228)-S29*(OpF223+qd[29]*OM328);
  BS229 = OM129*OM229;
  BS329 = OM129*OM329;
  BS629 = OM229*OM329;
  BS929 = -(OM129*OM129+OM229*OM229);
  BeF329 = BS329+OpF229;
  BeF629 = BS629-OpF128;
  AlF129 = AlF128+BeF228*s->dpt[2][56];
  AlF229 = C29*(AlF228+BS528*s->dpt[2][56])+S29*(AlF328+BeF828*s->dpt[2][56]);
  AlF329 = C29*(AlF328+BeF828*s->dpt[2][56])-S29*(AlF228+BS528*s->dpt[2][56]);
  AlM229_1 = AlM223_1*C29+AlM328_1*S29;
  AlM329_1 = -(AlM223_1*S29-AlM328_1*C29);
  AlM229_2 = AlM223_2*C29+AlM328_2*S29;
  AlM329_2 = -(AlM223_2*S29-AlM328_2*C29);
  AlM229_3 = AlM223_3*C29+AlM328_3*S29;
  AlM329_3 = -(AlM223_3*S29-AlM328_3*C29);
  OpM229_4 = OpM223_4*C29+OpM328_4*S29;
  OpM329_4 = -(OpM223_4*S29-OpM328_4*C29);
  AlM129_4 = AlM128_4-OpM328_4*s->dpt[2][56];
  AlM229_4 = AlM228_4*C29+S29*(AlM328_4+OpM128_4*s->dpt[2][56]);
  AlM329_4 = -(AlM228_4*S29-C29*(AlM328_4+OpM128_4*s->dpt[2][56]));
  OpM229_5 = OpM223_5*C29+OpM328_5*S29;
  OpM329_5 = -(OpM223_5*S29-OpM328_5*C29);
  AlM129_5 = AlM128_5-OpM328_5*s->dpt[2][56];
  AlM229_5 = AlM228_5*C29+S29*(AlM328_5+OpM128_5*s->dpt[2][56]);
  AlM329_5 = -(AlM228_5*S29-C29*(AlM328_5+OpM128_5*s->dpt[2][56]));
  OpM229_6 = OpM223_6*C29+OpM328_6*S29;
  OpM329_6 = -(OpM223_6*S29-OpM328_6*C29);
  AlM129_6 = AlM128_6-OpM328_6*s->dpt[2][56];
  AlM229_6 = AlM228_6*C29+S29*(AlM328_6+OpM128_6*s->dpt[2][56]);
  AlM329_6 = -(AlM228_6*S29-C29*(AlM328_6+OpM128_6*s->dpt[2][56]));
  OpM229_21 = OpM223_21*C29+OpM328_21*S29;
  OpM329_21 = -(OpM223_21*S29-OpM328_21*C29);
  AlM129_21 = AlM128_21-OpM328_21*s->dpt[2][56];
  AlM229_21 = AlM228_21*C29+S29*(AlM328_21+OpM128_21*s->dpt[2][56]);
  AlM329_21 = -(AlM228_21*S29-C29*(AlM328_21+OpM128_21*s->dpt[2][56]));
  OpM229_22 = OpM328_22*S29+C23*C29;
  OpM329_22 = OpM328_22*C29-C23*S29;
  AlM129_22 = AlM128_22-OpM328_22*s->dpt[2][56];
  AlM229_22 = AlM228_22*C29+S29*(AlM328_22+OpM128_22*s->dpt[2][56]);
  AlM329_22 = -(AlM228_22*S29-C29*(AlM328_22+OpM128_22*s->dpt[2][56]));
  OpM229_23 = C28*S29;
  OpM329_23 = C28*C29;
  AlM129_23 = AlM128_23-s->dpt[2][56]*C28;
  AlM229_23 = s->dpt[1][45]*C29-S28*S29*(s->dpt[2][45]+s->dpt[2][56]);
  AlM329_23 = -(s->dpt[1][45]*S29+S28*C29*(s->dpt[2][45]+s->dpt[2][56]));
  OM130 = OM129*C30+OM229*S30;
  OM230 = -(OM129*S30-OM229*C30);
  OM330 = qd[30]+OM329;
  OpF130 = C30*(OpF128+qd[30]*OM229)+S30*(OpF229-qd[30]*OM129);
  OpF230 = C30*(OpF229-qd[30]*OM129)-S30*(OpF128+qd[30]*OM229);
  BS230 = OM130*OM230;
  BS330 = OM130*OM330;
  BS630 = OM230*OM330;
  BS930 = -(OM130*OM130+OM230*OM230);
  BeF330 = BS330+OpF230;
  BeF630 = BS630-OpF130;
  AlF130 = C30*(AlF129+BeF329*s->dpt[3][58])+S30*(AlF229+BeF629*s->dpt[3][58]);
  AlF230 = C30*(AlF229+BeF629*s->dpt[3][58])-S30*(AlF129+BeF329*s->dpt[3][58]);
  AlF330 = AlF329+BS929*s->dpt[3][58];
  AlM130_1 = AlM128_1*C30+AlM229_1*S30;
  AlM230_1 = -(AlM128_1*S30-AlM229_1*C30);
  AlM130_2 = AlM128_2*C30+AlM229_2*S30;
  AlM230_2 = -(AlM128_2*S30-AlM229_2*C30);
  AlM130_3 = AlM128_3*C30+AlM229_3*S30;
  AlM230_3 = -(AlM128_3*S30-AlM229_3*C30);
  OpM130_4 = OpM128_4*C30+OpM229_4*S30;
  OpM230_4 = -(OpM128_4*S30-OpM229_4*C30);
  AlM130_4 = C30*(AlM129_4+OpM229_4*s->dpt[3][58])+S30*(AlM229_4-OpM128_4*s->dpt[3][58]);
  AlM230_4 = C30*(AlM229_4-OpM128_4*s->dpt[3][58])-S30*(AlM129_4+OpM229_4*s->dpt[3][58]);
  OpM130_5 = OpM128_5*C30+OpM229_5*S30;
  OpM230_5 = -(OpM128_5*S30-OpM229_5*C30);
  AlM130_5 = C30*(AlM129_5+OpM229_5*s->dpt[3][58])+S30*(AlM229_5-OpM128_5*s->dpt[3][58]);
  AlM230_5 = C30*(AlM229_5-OpM128_5*s->dpt[3][58])-S30*(AlM129_5+OpM229_5*s->dpt[3][58]);
  OpM130_6 = OpM128_6*C30+OpM229_6*S30;
  OpM230_6 = -(OpM128_6*S30-OpM229_6*C30);
  AlM130_6 = C30*(AlM129_6+OpM229_6*s->dpt[3][58])+S30*(AlM229_6-OpM128_6*s->dpt[3][58]);
  AlM230_6 = C30*(AlM229_6-OpM128_6*s->dpt[3][58])-S30*(AlM129_6+OpM229_6*s->dpt[3][58]);
  OpM130_21 = OpM128_21*C30+OpM229_21*S30;
  OpM230_21 = -(OpM128_21*S30-OpM229_21*C30);
  AlM130_21 = C30*(AlM129_21+OpM229_21*s->dpt[3][58])+S30*(AlM229_21-OpM128_21*s->dpt[3][58]);
  AlM230_21 = C30*(AlM229_21-OpM128_21*s->dpt[3][58])-S30*(AlM129_21+OpM229_21*s->dpt[3][58]);
  OpM130_22 = OpM128_22*C30+OpM229_22*S30;
  OpM230_22 = -(OpM128_22*S30-OpM229_22*C30);
  AlM130_22 = C30*(AlM129_22+OpM229_22*s->dpt[3][58])+S30*(AlM229_22-OpM128_22*s->dpt[3][58]);
  AlM230_22 = C30*(AlM229_22-OpM128_22*s->dpt[3][58])-S30*(AlM129_22+OpM229_22*s->dpt[3][58]);
  OpM130_23 = OpM229_23*S30-S28*C30;
  OpM230_23 = OpM229_23*C30+S28*S30;
  AlM130_23 = C30*(AlM129_23+OpM229_23*s->dpt[3][58])+S30*(AlM229_23+s->dpt[3][58]*S28);
  AlM230_23 = C30*(AlM229_23+s->dpt[3][58]*S28)-S30*(AlM129_23+OpM229_23*s->dpt[3][58]);
  OpM130_28 = C29*S30;
  OpM230_28 = C29*C30;
  AlM130_28 = s->dpt[3][58]*C29*C30;
  AlM230_28 = -s->dpt[3][58]*C29*S30;
  AlM130_29 = -s->dpt[3][58]*S30;
  AlM230_29 = -s->dpt[3][58]*C30;
  OM131 = OM130*C31-OM330*S31;
  OM231 = qd[31]+OM230;
  OM331 = OM130*S31+OM330*C31;
  OpF131 = C31*(OpF130-qd[31]*OM330)-S31*(OpF329+qd[31]*OM130);
  OpF331 = C31*(OpF329+qd[31]*OM130)+S31*(OpF130-qd[31]*OM330);
  BS231 = OM131*OM231;
  BS331 = OM131*OM331;
  BS631 = OM231*OM331;
  OpM131_4 = OpM130_4*C31-OpM329_4*S31;
  OpM331_4 = OpM130_4*S31+OpM329_4*C31;
  OpM131_5 = OpM130_5*C31-OpM329_5*S31;
  OpM331_5 = OpM130_5*S31+OpM329_5*C31;
  OpM131_6 = OpM130_6*C31-OpM329_6*S31;
  OpM331_6 = OpM130_6*S31+OpM329_6*C31;
  OpM131_21 = OpM130_21*C31-OpM329_21*S31;
  OpM331_21 = OpM130_21*S31+OpM329_21*C31;
  OpM131_22 = OpM130_22*C31-OpM329_22*S31;
  OpM331_22 = OpM130_22*S31+OpM329_22*C31;
  OpM131_23 = OpM130_23*C31-OpM329_23*S31;
  OpM331_23 = OpM130_23*S31+OpM329_23*C31;
  OpM131_28 = OpM130_28*C31+S29*S31;
  OpM331_28 = OpM130_28*S31-S29*C31;
  OpM131_29 = C30*C31;
  OpM331_29 = C30*S31;

// = = Block_0_2_0_1_0_5 = = 
 
// Backward Dynamics 

  FA127 = -(s->frc[1][27]+s->m[27]*(s->l[1][27]*(OM227*OM227+OM327*OM327)-s->l[2][27]*(BS227-OpF327)-s->l[3][27]*(BS327+
 OpF226)-C27*(AlF126+BeF326*s->dpt[3][52])+S27*(AlF326+BS926*s->dpt[3][52])));
  FA227 = -(s->frc[2][27]-s->m[27]*(AlF226+BeF626*s->dpt[3][52]+s->l[1][27]*(BS227+OpF327)-s->l[2][27]*(OM127*OM127+
 OM327*OM327)+s->l[3][27]*(BS627-OpF127)));
  FA327 = -(s->frc[3][27]-s->m[27]*(s->l[1][27]*(BS327-OpF226)+s->l[2][27]*(BS627+OpF127)-s->l[3][27]*(OM127*OM127+OM227
 *OM227)+C27*(AlF326+BS926*s->dpt[3][52])+S27*(AlF126+BeF326*s->dpt[3][52])));
  CF127 = -(s->trq[1][27]-s->In[1][27]*OpF127+FA227*s->l[3][27]-FA327*s->l[2][27]+OM227*OM327*(s->In[5][27]-s->In[9][27]
 ));
  CF227 = -(s->trq[2][27]-s->In[5][27]*OpF226-FA127*s->l[3][27]+FA327*s->l[1][27]-OM127*OM327*(s->In[1][27]-s->In[9][27]
 ));
  CF327 = -(s->trq[3][27]-s->In[9][27]*OpF327+FA127*s->l[2][27]-FA227*s->l[1][27]+OM127*OM227*(s->In[1][27]-s->In[5][27]
 ));
  FB127_1 = s->m[27]*(AlM126_1*C27-AlM325_1*S27);
  FB227_1 = s->m[27]*AlM226_1;
  FB327_1 = s->m[27]*(AlM126_1*S27+AlM325_1*C27);
  CM127_1 = -(FB227_1*s->l[3][27]-FB327_1*s->l[2][27]);
  CM227_1 = FB127_1*s->l[3][27]-FB327_1*s->l[1][27];
  CM327_1 = -(FB127_1*s->l[2][27]-FB227_1*s->l[1][27]);
  FB127_2 = s->m[27]*(AlM126_2*C27-AlM325_2*S27);
  FB227_2 = s->m[27]*AlM226_2;
  FB327_2 = s->m[27]*(AlM126_2*S27+AlM325_2*C27);
  CM127_2 = -(FB227_2*s->l[3][27]-FB327_2*s->l[2][27]);
  CM227_2 = FB127_2*s->l[3][27]-FB327_2*s->l[1][27];
  CM327_2 = -(FB127_2*s->l[2][27]-FB227_2*s->l[1][27]);
  FB127_3 = s->m[27]*(AlM126_3*C27-AlM325_3*S27);
  FB227_3 = s->m[27]*AlM226_3;
  FB327_3 = s->m[27]*(AlM126_3*S27+AlM325_3*C27);
  CM127_3 = -(FB227_3*s->l[3][27]-FB327_3*s->l[2][27]);
  CM227_3 = FB127_3*s->l[3][27]-FB327_3*s->l[1][27];
  CM327_3 = -(FB127_3*s->l[2][27]-FB227_3*s->l[1][27]);
  FB127_4 = -s->m[27]*(AlM325_4*S27-OpM226_4*s->l[3][27]+OpM327_4*s->l[2][27]-C27*(AlM126_4+OpM226_4*s->dpt[3][52]));
  FB227_4 = s->m[27]*(AlM226_4-OpM126_4*s->dpt[3][52]-OpM127_4*s->l[3][27]+OpM327_4*s->l[1][27]);
  FB327_4 = s->m[27]*(AlM325_4*C27+OpM127_4*s->l[2][27]-OpM226_4*s->l[1][27]+S27*(AlM126_4+OpM226_4*s->dpt[3][52]));
  CM127_4 = s->In[1][27]*OpM127_4-FB227_4*s->l[3][27]+FB327_4*s->l[2][27];
  CM227_4 = s->In[5][27]*OpM226_4+FB127_4*s->l[3][27]-FB327_4*s->l[1][27];
  CM327_4 = s->In[9][27]*OpM327_4-FB127_4*s->l[2][27]+FB227_4*s->l[1][27];
  FB127_5 = -s->m[27]*(AlM325_5*S27-OpM226_5*s->l[3][27]+OpM327_5*s->l[2][27]-C27*(AlM126_5+OpM226_5*s->dpt[3][52]));
  FB227_5 = s->m[27]*(AlM226_5-OpM126_5*s->dpt[3][52]-OpM127_5*s->l[3][27]+OpM327_5*s->l[1][27]);
  FB327_5 = s->m[27]*(AlM325_5*C27+OpM127_5*s->l[2][27]-OpM226_5*s->l[1][27]+S27*(AlM126_5+OpM226_5*s->dpt[3][52]));
  CM127_5 = s->In[1][27]*OpM127_5-FB227_5*s->l[3][27]+FB327_5*s->l[2][27];
  CM227_5 = s->In[5][27]*OpM226_5+FB127_5*s->l[3][27]-FB327_5*s->l[1][27];
  CM327_5 = s->In[9][27]*OpM327_5-FB127_5*s->l[2][27]+FB227_5*s->l[1][27];
  FB127_6 = -s->m[27]*(AlM325_6*S27-OpM226_6*s->l[3][27]+OpM327_6*s->l[2][27]-C27*(AlM126_6+OpM226_6*s->dpt[3][52]));
  FB227_6 = s->m[27]*(AlM226_6-OpM126_6*s->dpt[3][52]-OpM127_6*s->l[3][27]+OpM327_6*s->l[1][27]);
  FB327_6 = s->m[27]*(AlM325_6*C27+OpM127_6*s->l[2][27]-OpM226_6*s->l[1][27]+S27*(AlM126_6+OpM226_6*s->dpt[3][52]));
  CM127_6 = s->In[1][27]*OpM127_6-FB227_6*s->l[3][27]+FB327_6*s->l[2][27];
  CM227_6 = s->In[5][27]*OpM226_6+FB127_6*s->l[3][27]-FB327_6*s->l[1][27];
  CM327_6 = s->In[9][27]*OpM327_6-FB127_6*s->l[2][27]+FB227_6*s->l[1][27];
  FB127_21 = -s->m[27]*(AlM325_21*S27-OpM226_21*s->l[3][27]+OpM327_21*s->l[2][27]-C27*(AlM126_21+OpM226_21*s->dpt[3][52]
 ));
  FB227_21 = s->m[27]*(AlM226_21-OpM126_21*s->dpt[3][52]-OpM127_21*s->l[3][27]+OpM327_21*s->l[1][27]);
  FB327_21 = s->m[27]*(AlM325_21*C27+OpM127_21*s->l[2][27]-OpM226_21*s->l[1][27]+S27*(AlM126_21+OpM226_21*s->dpt[3][52])
 );
  CM127_21 = s->In[1][27]*OpM127_21-FB227_21*s->l[3][27]+FB327_21*s->l[2][27];
  CM227_21 = s->In[5][27]*OpM226_21+FB127_21*s->l[3][27]-FB327_21*s->l[1][27];
  CM327_21 = s->In[9][27]*OpM327_21-FB127_21*s->l[2][27]+FB227_21*s->l[1][27];
  FB127_22 = -s->m[27]*(AlM325_22*S27-OpM226_22*s->l[3][27]+OpM327_22*s->l[2][27]-C27*(AlM126_22+OpM226_22*s->dpt[3][52]
 ));
  FB227_22 = s->m[27]*(AlM226_22-OpM126_22*s->dpt[3][52]-OpM127_22*s->l[3][27]+OpM327_22*s->l[1][27]);
  FB327_22 = s->m[27]*(AlM325_22*C27+OpM127_22*s->l[2][27]-OpM226_22*s->l[1][27]+S27*(AlM126_22+OpM226_22*s->dpt[3][52])
 );
  CM127_22 = s->In[1][27]*OpM127_22-FB227_22*s->l[3][27]+FB327_22*s->l[2][27];
  CM227_22 = s->In[5][27]*OpM226_22+FB127_22*s->l[3][27]-FB327_22*s->l[1][27];
  CM327_22 = s->In[9][27]*OpM327_22-FB127_22*s->l[2][27]+FB227_22*s->l[1][27];
  FB127_23 = -s->m[27]*(AlM325_23*S27-OpM226_23*s->l[3][27]+OpM327_23*s->l[2][27]-C27*(AlM126_23+OpM226_23*s->dpt[3][52]
 ));
  FB227_23 = s->m[27]*(AlM226_23-OpM126_23*s->dpt[3][52]-OpM127_23*s->l[3][27]+OpM327_23*s->l[1][27]);
  FB327_23 = s->m[27]*(AlM325_23*C27+OpM127_23*s->l[2][27]-OpM226_23*s->l[1][27]+S27*(AlM126_23+OpM226_23*s->dpt[3][52])
 );
  CM127_23 = s->In[1][27]*OpM127_23-FB227_23*s->l[3][27]+FB327_23*s->l[2][27];
  CM227_23 = s->In[5][27]*OpM226_23+FB127_23*s->l[3][27]-FB327_23*s->l[1][27];
  CM327_23 = s->In[9][27]*OpM327_23-FB127_23*s->l[2][27]+FB227_23*s->l[1][27];
  FB127_24 = s->m[27]*(OpM226_24*s->l[3][27]-OpM327_24*s->l[2][27]+C27*(AlM126_24+OpM226_24*s->dpt[3][52]));
  FB227_24 = s->m[27]*(AlM226_24-OpM126_24*s->dpt[3][52]-OpM127_24*s->l[3][27]+OpM327_24*s->l[1][27]);
  FB327_24 = s->m[27]*(OpM127_24*s->l[2][27]-OpM226_24*s->l[1][27]+S27*(AlM126_24+OpM226_24*s->dpt[3][52]));
  CM127_24 = s->In[1][27]*OpM127_24-FB227_24*s->l[3][27]+FB327_24*s->l[2][27];
  CM227_24 = s->In[5][27]*OpM226_24+FB127_24*s->l[3][27]-FB327_24*s->l[1][27];
  CM327_24 = s->In[9][27]*OpM327_24-FB127_24*s->l[2][27]+FB227_24*s->l[1][27];
  FB127_25 = s->m[27]*(C27*(AlM126_25-s->dpt[3][52]*S26)-OpM327_25*s->l[2][27]-s->l[3][27]*S26);
  FB227_25 = s->m[27]*(AlM226_25-OpM127_25*s->l[3][27]+OpM327_25*s->l[1][27]-s->dpt[3][52]*C26);
  FB327_25 = s->m[27]*(OpM127_25*s->l[2][27]+s->l[1][27]*S26+S27*(AlM126_25-s->dpt[3][52]*S26));
  CM127_25 = s->In[1][27]*OpM127_25-FB227_25*s->l[3][27]+FB327_25*s->l[2][27];
  CM227_25 = -(s->In[5][27]*S26-FB127_25*s->l[3][27]+FB327_25*s->l[1][27]);
  CM327_25 = s->In[9][27]*OpM327_25-FB127_25*s->l[2][27]+FB227_25*s->l[1][27];
  FB127_26 = -s->m[27]*s->l[2][27]*C27;
  FB227_26 = s->m[27]*(s->l[1][27]*C27+s->l[3][27]*S27);
  FB327_26 = -s->m[27]*s->l[2][27]*S27;
  CM227_26 = FB127_26*s->l[3][27]-FB327_26*s->l[1][27];
  CM227_27 = s->In[5][27]+s->m[27]*s->l[1][27]*s->l[1][27]+s->m[27]*s->l[3][27]*s->l[3][27];
  FA126 = -(s->frc[1][26]-s->m[26]*(AlF126+BeF326*s->l[3][26]-s->l[1][26]*(OM226*OM226+OM326*OM326)+s->l[2][26]*(BS226-
 OpF325)));
  FA226 = -(s->frc[2][26]-s->m[26]*(AlF226+BeF626*s->l[3][26]+s->l[1][26]*(BS226+OpF325)-s->l[2][26]*(OM126*OM126+OM326*
 OM326)));
  FA326 = -(s->frc[3][26]-s->m[26]*(AlF326+BS926*s->l[3][26]+s->l[1][26]*(BS326-OpF226)+s->l[2][26]*(BS626+OpF126)));
  FF126 = FA126+FA127*C27+FA327*S27;
  FF226 = FA226+FA227;
  CF126 = -(s->trq[1][26]-s->In[1][26]*OpF126-s->In[2][26]*OpF226-s->In[3][26]*OpF325-CF127*C27-CF327*S27+FA226*
 s->l[3][26]+FA227*s->dpt[3][52]-FA326*s->l[2][26]-OM226*(s->In[3][26]*OM126+s->In[9][26]*OM326)+OM326*(s->In[2][26]*OM126+
 s->In[5][26]*OM226));
  CF226 = -(s->trq[2][26]-CF227-s->In[2][26]*OpF126-s->In[5][26]*OpF226-FA126*s->l[3][26]+FA326*s->l[1][26]+OM126*(
 s->In[3][26]*OM126+s->In[9][26]*OM326)-OM326*(s->In[1][26]*OM126+s->In[2][26]*OM226+s->In[3][26]*OM326)-s->dpt[3][52]*(FA127
 *C27+FA327*S27));
  CF326 = -(s->trq[3][26]-s->In[3][26]*OpF126-s->In[9][26]*OpF325+CF127*S27-CF327*C27+FA126*s->l[2][26]-FA226*
 s->l[1][26]-OM126*(s->In[2][26]*OM126+s->In[5][26]*OM226)+OM226*(s->In[1][26]*OM126+s->In[2][26]*OM226+s->In[3][26]*OM326));
  FB126_1 = s->m[26]*AlM126_1;
  FB226_1 = s->m[26]*AlM226_1;
  FB326_1 = s->m[26]*AlM325_1;
  FM126_1 = FB126_1+FB127_1*C27+FB327_1*S27;
  FM226_1 = FB226_1+FB227_1;
  CM126_1 = CM127_1*C27+CM327_1*S27-FB226_1*s->l[3][26]-FB227_1*s->dpt[3][52]+FB326_1*s->l[2][26];
  CM226_1 = CM227_1+FB126_1*s->l[3][26]-FB326_1*s->l[1][26]+s->dpt[3][52]*(FB127_1*C27+FB327_1*S27);
  CM326_1 = -(CM127_1*S27-CM327_1*C27+FB126_1*s->l[2][26]-FB226_1*s->l[1][26]);
  FB126_2 = s->m[26]*AlM126_2;
  FB226_2 = s->m[26]*AlM226_2;
  FB326_2 = s->m[26]*AlM325_2;
  FM126_2 = FB126_2+FB127_2*C27+FB327_2*S27;
  FM226_2 = FB226_2+FB227_2;
  CM126_2 = CM127_2*C27+CM327_2*S27-FB226_2*s->l[3][26]-FB227_2*s->dpt[3][52]+FB326_2*s->l[2][26];
  CM226_2 = CM227_2+FB126_2*s->l[3][26]-FB326_2*s->l[1][26]+s->dpt[3][52]*(FB127_2*C27+FB327_2*S27);
  CM326_2 = -(CM127_2*S27-CM327_2*C27+FB126_2*s->l[2][26]-FB226_2*s->l[1][26]);
  FB126_3 = s->m[26]*AlM126_3;
  FB226_3 = s->m[26]*AlM226_3;
  FB326_3 = s->m[26]*AlM325_3;
  FM126_3 = FB126_3+FB127_3*C27+FB327_3*S27;
  FM226_3 = FB226_3+FB227_3;
  CM126_3 = CM127_3*C27+CM327_3*S27-FB226_3*s->l[3][26]-FB227_3*s->dpt[3][52]+FB326_3*s->l[2][26];
  CM226_3 = CM227_3+FB126_3*s->l[3][26]-FB326_3*s->l[1][26]+s->dpt[3][52]*(FB127_3*C27+FB327_3*S27);
  CM326_3 = -(CM127_3*S27-CM327_3*C27+FB126_3*s->l[2][26]-FB226_3*s->l[1][26]);
  FB126_4 = s->m[26]*(AlM126_4+OpM226_4*s->l[3][26]-OpM325_4*s->l[2][26]);
  FB226_4 = s->m[26]*(AlM226_4-OpM126_4*s->l[3][26]+OpM325_4*s->l[1][26]);
  FB326_4 = s->m[26]*(AlM325_4+OpM126_4*s->l[2][26]-OpM226_4*s->l[1][26]);
  FM126_4 = FB126_4+FB127_4*C27+FB327_4*S27;
  FM226_4 = FB226_4+FB227_4;
  CM126_4 = s->In[1][26]*OpM126_4+s->In[2][26]*OpM226_4+s->In[3][26]*OpM325_4+CM127_4*C27+CM327_4*S27-FB226_4*
 s->l[3][26]-FB227_4*s->dpt[3][52]+FB326_4*s->l[2][26];
  CM226_4 = CM227_4+s->In[2][26]*OpM126_4+s->In[5][26]*OpM226_4+FB126_4*s->l[3][26]-FB326_4*s->l[1][26]+s->dpt[3][52]*(
 FB127_4*C27+FB327_4*S27);
  CM326_4 = s->In[3][26]*OpM126_4+s->In[9][26]*OpM325_4-CM127_4*S27+CM327_4*C27-FB126_4*s->l[2][26]+FB226_4*s->l[1][26];
  FB126_5 = s->m[26]*(AlM126_5+OpM226_5*s->l[3][26]-OpM325_5*s->l[2][26]);
  FB226_5 = s->m[26]*(AlM226_5-OpM126_5*s->l[3][26]+OpM325_5*s->l[1][26]);
  FB326_5 = s->m[26]*(AlM325_5+OpM126_5*s->l[2][26]-OpM226_5*s->l[1][26]);
  FM126_5 = FB126_5+FB127_5*C27+FB327_5*S27;
  FM226_5 = FB226_5+FB227_5;
  CM126_5 = s->In[1][26]*OpM126_5+s->In[2][26]*OpM226_5+s->In[3][26]*OpM325_5+CM127_5*C27+CM327_5*S27-FB226_5*
 s->l[3][26]-FB227_5*s->dpt[3][52]+FB326_5*s->l[2][26];
  CM226_5 = CM227_5+s->In[2][26]*OpM126_5+s->In[5][26]*OpM226_5+FB126_5*s->l[3][26]-FB326_5*s->l[1][26]+s->dpt[3][52]*(
 FB127_5*C27+FB327_5*S27);
  CM326_5 = s->In[3][26]*OpM126_5+s->In[9][26]*OpM325_5-CM127_5*S27+CM327_5*C27-FB126_5*s->l[2][26]+FB226_5*s->l[1][26];
  FB126_6 = s->m[26]*(AlM126_6+OpM226_6*s->l[3][26]-OpM325_6*s->l[2][26]);
  FB226_6 = s->m[26]*(AlM226_6-OpM126_6*s->l[3][26]+OpM325_6*s->l[1][26]);
  FB326_6 = s->m[26]*(AlM325_6+OpM126_6*s->l[2][26]-OpM226_6*s->l[1][26]);
  FM126_6 = FB126_6+FB127_6*C27+FB327_6*S27;
  FM226_6 = FB226_6+FB227_6;
  CM126_6 = s->In[1][26]*OpM126_6+s->In[2][26]*OpM226_6+s->In[3][26]*OpM325_6+CM127_6*C27+CM327_6*S27-FB226_6*
 s->l[3][26]-FB227_6*s->dpt[3][52]+FB326_6*s->l[2][26];
  CM226_6 = CM227_6+s->In[2][26]*OpM126_6+s->In[5][26]*OpM226_6+FB126_6*s->l[3][26]-FB326_6*s->l[1][26]+s->dpt[3][52]*(
 FB127_6*C27+FB327_6*S27);
  CM326_6 = s->In[3][26]*OpM126_6+s->In[9][26]*OpM325_6-CM127_6*S27+CM327_6*C27-FB126_6*s->l[2][26]+FB226_6*s->l[1][26];
  FB126_21 = s->m[26]*(AlM126_21+OpM226_21*s->l[3][26]-OpM325_21*s->l[2][26]);
  FB226_21 = s->m[26]*(AlM226_21-OpM126_21*s->l[3][26]+OpM325_21*s->l[1][26]);
  FB326_21 = s->m[26]*(AlM325_21+OpM126_21*s->l[2][26]-OpM226_21*s->l[1][26]);
  FM126_21 = FB126_21+FB127_21*C27+FB327_21*S27;
  FM226_21 = FB226_21+FB227_21;
  CM126_21 = s->In[1][26]*OpM126_21+s->In[2][26]*OpM226_21+s->In[3][26]*OpM325_21+CM127_21*C27+CM327_21*S27-FB226_21*
 s->l[3][26]-FB227_21*s->dpt[3][52]+FB326_21*s->l[2][26];
  CM226_21 = CM227_21+s->In[2][26]*OpM126_21+s->In[5][26]*OpM226_21+FB126_21*s->l[3][26]-FB326_21*s->l[1][26]+
 s->dpt[3][52]*(FB127_21*C27+FB327_21*S27);
  CM326_21 = s->In[3][26]*OpM126_21+s->In[9][26]*OpM325_21-CM127_21*S27+CM327_21*C27-FB126_21*s->l[2][26]+FB226_21*
 s->l[1][26];
  FB126_22 = s->m[26]*(AlM126_22+OpM226_22*s->l[3][26]-OpM325_22*s->l[2][26]);
  FB226_22 = s->m[26]*(AlM226_22-OpM126_22*s->l[3][26]+OpM325_22*s->l[1][26]);
  FB326_22 = s->m[26]*(AlM325_22+OpM126_22*s->l[2][26]-OpM226_22*s->l[1][26]);
  FM126_22 = FB126_22+FB127_22*C27+FB327_22*S27;
  FM226_22 = FB226_22+FB227_22;
  CM126_22 = s->In[1][26]*OpM126_22+s->In[2][26]*OpM226_22+s->In[3][26]*OpM325_22+CM127_22*C27+CM327_22*S27-FB226_22*
 s->l[3][26]-FB227_22*s->dpt[3][52]+FB326_22*s->l[2][26];
  CM226_22 = CM227_22+s->In[2][26]*OpM126_22+s->In[5][26]*OpM226_22+FB126_22*s->l[3][26]-FB326_22*s->l[1][26]+
 s->dpt[3][52]*(FB127_22*C27+FB327_22*S27);
  CM326_22 = s->In[3][26]*OpM126_22+s->In[9][26]*OpM325_22-CM127_22*S27+CM327_22*C27-FB126_22*s->l[2][26]+FB226_22*
 s->l[1][26];
  FB126_23 = s->m[26]*(AlM126_23+OpM226_23*s->l[3][26]-OpM325_23*s->l[2][26]);
  FB226_23 = s->m[26]*(AlM226_23-OpM126_23*s->l[3][26]+OpM325_23*s->l[1][26]);
  FB326_23 = s->m[26]*(AlM325_23+OpM126_23*s->l[2][26]-OpM226_23*s->l[1][26]);
  FM126_23 = FB126_23+FB127_23*C27+FB327_23*S27;
  FM226_23 = FB226_23+FB227_23;
  CM126_23 = s->In[1][26]*OpM126_23+s->In[2][26]*OpM226_23+s->In[3][26]*OpM325_23+CM127_23*C27+CM327_23*S27-FB226_23*
 s->l[3][26]-FB227_23*s->dpt[3][52]+FB326_23*s->l[2][26];
  CM226_23 = CM227_23+s->In[2][26]*OpM126_23+s->In[5][26]*OpM226_23+FB126_23*s->l[3][26]-FB326_23*s->l[1][26]+
 s->dpt[3][52]*(FB127_23*C27+FB327_23*S27);
  CM326_23 = s->In[3][26]*OpM126_23+s->In[9][26]*OpM325_23-CM127_23*S27+CM327_23*C27-FB126_23*s->l[2][26]+FB226_23*
 s->l[1][26];
  FB126_24 = s->m[26]*(AlM126_24+OpM226_24*s->l[3][26]+s->l[2][26]*S25);
  FB226_24 = s->m[26]*(AlM226_24-OpM126_24*s->l[3][26]-s->l[1][26]*S25);
  FB326_24 = s->m[26]*(OpM126_24*s->l[2][26]-OpM226_24*s->l[1][26]);
  FM126_24 = FB126_24+FB127_24*C27+FB327_24*S27;
  FM226_24 = FB226_24+FB227_24;
  CM126_24 = s->In[1][26]*OpM126_24+s->In[2][26]*OpM226_24-s->In[3][26]*S25+CM127_24*C27+CM327_24*S27-FB226_24*
 s->l[3][26]-FB227_24*s->dpt[3][52]+FB326_24*s->l[2][26];
  CM226_24 = CM227_24+s->In[2][26]*OpM126_24+s->In[5][26]*OpM226_24+FB126_24*s->l[3][26]-FB326_24*s->l[1][26]+
 s->dpt[3][52]*(FB127_24*C27+FB327_24*S27);
  CM326_24 = s->In[3][26]*OpM126_24-s->In[9][26]*S25-CM127_24*S27+CM327_24*C27-FB126_24*s->l[2][26]+FB226_24*s->l[1][26];
  FB126_25 = s->m[26]*(AlM126_25-s->l[3][26]*S26);
  FB226_25 = s->m[26]*(AlM226_25-s->l[3][26]*C26);
  FB326_25 = s->m[26]*(s->l[1][26]*S26+s->l[2][26]*C26);
  CM326_25 = s->In[3][26]*C26-CM127_25*S27+CM327_25*C27-FB126_25*s->l[2][26]+FB226_25*s->l[1][26];
  CM326_26 = s->In[9][26]+s->m[26]*s->l[1][26]*s->l[1][26]+s->m[26]*s->l[2][26]*s->l[2][26]+C27*(s->In[9][27]*C27-
 FB127_26*s->l[2][27]+FB227_26*s->l[1][27])+S27*(s->In[1][27]*S27+FB227_26*s->l[3][27]-FB327_26*s->l[2][27]);
  FA125 = -(s->frc[1][25]-s->m[25]*(AlF125+BeF325*s->l[3][25]-s->l[1][25]*(OM225*OM225+OM325*OM325)+s->l[2][25]*(BS225-
 OpF325)));
  FA225 = -(s->frc[2][25]-s->m[25]*(AlF225+BeF625*s->l[3][25]+s->l[1][25]*(BS225+OpF325)-s->l[2][25]*(OM125*OM125+OM325*
 OM325)));
  FA325 = -(s->frc[3][25]-s->m[25]*(AlF325+BS925*s->l[3][25]+s->l[1][25]*(BS325-OpF225)+s->l[2][25]*(BS625+OpF124)));
  FF125 = FA125+FF126*C26-FF226*S26;
  FF225 = FA225+FF126*S26+FF226*C26;
  FF325 = FA325+FA326-FA127*S27+FA327*C27;
  CF125 = -(s->trq[1][25]-s->In[1][25]*OpF124-s->In[3][25]*OpF325-CF126*C26+CF226*S26+FA225*s->l[3][25]-FA325*
 s->l[2][25]-OM225*(s->In[3][25]*OM125-s->In[5][25]*OM325+s->In[9][25]*OM325)+s->dpt[3][49]*(FF126*S26+FF226*C26));
  CF225 = -(s->trq[2][25]-s->In[5][25]*OpF225-CF126*S26-CF226*C26-FA125*s->l[3][25]+FA325*s->l[1][25]+OM125*(
 s->In[3][25]*OM125+s->In[9][25]*OM325)-OM325*(s->In[1][25]*OM125+s->In[3][25]*OM325)-s->dpt[3][49]*(FF126*C26-FF226*S26));
  CF325 = -(s->trq[3][25]-CF326-s->In[3][25]*OpF124-s->In[9][25]*OpF325+FA125*s->l[2][25]-FA225*s->l[1][25]+OM225*(
 s->In[1][25]*OM125+s->In[3][25]*OM325-s->In[5][25]*OM125));
  FB125_1 = s->m[25]*AlM124_1;
  FB225_1 = s->m[25]*AlM225_1;
  FB325_1 = s->m[25]*AlM325_1;
  FM125_1 = FB125_1+FM126_1*C26-FM226_1*S26;
  FM225_1 = FB225_1+FM126_1*S26+FM226_1*C26;
  FM325_1 = FB325_1+FB326_1-FB127_1*S27+FB327_1*C27;
  CM125_1 = CM126_1*C26-CM226_1*S26-FB225_1*s->l[3][25]+FB325_1*s->l[2][25]-s->dpt[3][49]*(FM126_1*S26+FM226_1*C26);
  CM225_1 = CM126_1*S26+CM226_1*C26+FB125_1*s->l[3][25]-FB325_1*s->l[1][25]+s->dpt[3][49]*(FM126_1*C26-FM226_1*S26);
  CM325_1 = CM326_1-FB125_1*s->l[2][25]+FB225_1*s->l[1][25];
  FB125_2 = s->m[25]*AlM124_2;
  FB225_2 = s->m[25]*AlM225_2;
  FB325_2 = s->m[25]*AlM325_2;
  FM125_2 = FB125_2+FM126_2*C26-FM226_2*S26;
  FM225_2 = FB225_2+FM126_2*S26+FM226_2*C26;
  FM325_2 = FB325_2+FB326_2-FB127_2*S27+FB327_2*C27;
  CM125_2 = CM126_2*C26-CM226_2*S26-FB225_2*s->l[3][25]+FB325_2*s->l[2][25]-s->dpt[3][49]*(FM126_2*S26+FM226_2*C26);
  CM225_2 = CM126_2*S26+CM226_2*C26+FB125_2*s->l[3][25]-FB325_2*s->l[1][25]+s->dpt[3][49]*(FM126_2*C26-FM226_2*S26);
  CM325_2 = CM326_2-FB125_2*s->l[2][25]+FB225_2*s->l[1][25];
  FB125_3 = s->m[25]*AlM124_3;
  FB225_3 = s->m[25]*AlM225_3;
  FB325_3 = s->m[25]*AlM325_3;
  FM125_3 = FB125_3+FM126_3*C26-FM226_3*S26;
  FM225_3 = FB225_3+FM126_3*S26+FM226_3*C26;
  FM325_3 = FB325_3+FB326_3-FB127_3*S27+FB327_3*C27;
  CM125_3 = CM126_3*C26-CM226_3*S26-FB225_3*s->l[3][25]+FB325_3*s->l[2][25]-s->dpt[3][49]*(FM126_3*S26+FM226_3*C26);
  CM225_3 = CM126_3*S26+CM226_3*C26+FB125_3*s->l[3][25]-FB325_3*s->l[1][25]+s->dpt[3][49]*(FM126_3*C26-FM226_3*S26);
  CM325_3 = CM326_3-FB125_3*s->l[2][25]+FB225_3*s->l[1][25];
  FB125_4 = s->m[25]*(AlM125_4+OpM225_4*s->l[3][25]-OpM325_4*s->l[2][25]);
  FB225_4 = s->m[25]*(AlM225_4-OpM124_4*s->l[3][25]+OpM325_4*s->l[1][25]);
  FB325_4 = s->m[25]*(AlM325_4+OpM124_4*s->l[2][25]-OpM225_4*s->l[1][25]);
  FM125_4 = FB125_4+FM126_4*C26-FM226_4*S26;
  FM225_4 = FB225_4+FM126_4*S26+FM226_4*C26;
  FM325_4 = FB325_4+FB326_4-FB127_4*S27+FB327_4*C27;
  CM125_4 = s->In[1][25]*OpM124_4+s->In[3][25]*OpM325_4+CM126_4*C26-CM226_4*S26-FB225_4*s->l[3][25]+FB325_4*s->l[2][25]-
 s->dpt[3][49]*(FM126_4*S26+FM226_4*C26);
  CM225_4 = s->In[5][25]*OpM225_4+CM126_4*S26+CM226_4*C26+FB125_4*s->l[3][25]-FB325_4*s->l[1][25]+s->dpt[3][49]*(FM126_4
 *C26-FM226_4*S26);
  CM325_4 = CM326_4+s->In[3][25]*OpM124_4+s->In[9][25]*OpM325_4-FB125_4*s->l[2][25]+FB225_4*s->l[1][25];
  FB125_5 = s->m[25]*(AlM125_5+OpM225_5*s->l[3][25]-OpM325_5*s->l[2][25]);
  FB225_5 = s->m[25]*(AlM225_5-OpM124_5*s->l[3][25]+OpM325_5*s->l[1][25]);
  FB325_5 = s->m[25]*(AlM325_5+OpM124_5*s->l[2][25]-OpM225_5*s->l[1][25]);
  FM125_5 = FB125_5+FM126_5*C26-FM226_5*S26;
  FM225_5 = FB225_5+FM126_5*S26+FM226_5*C26;
  FM325_5 = FB325_5+FB326_5-FB127_5*S27+FB327_5*C27;
  CM125_5 = s->In[1][25]*OpM124_5+s->In[3][25]*OpM325_5+CM126_5*C26-CM226_5*S26-FB225_5*s->l[3][25]+FB325_5*s->l[2][25]-
 s->dpt[3][49]*(FM126_5*S26+FM226_5*C26);
  CM225_5 = s->In[5][25]*OpM225_5+CM126_5*S26+CM226_5*C26+FB125_5*s->l[3][25]-FB325_5*s->l[1][25]+s->dpt[3][49]*(FM126_5
 *C26-FM226_5*S26);
  CM325_5 = CM326_5+s->In[3][25]*OpM124_5+s->In[9][25]*OpM325_5-FB125_5*s->l[2][25]+FB225_5*s->l[1][25];
  FB125_6 = s->m[25]*(AlM125_6+OpM225_6*s->l[3][25]-OpM325_6*s->l[2][25]);
  FB225_6 = s->m[25]*(AlM225_6-OpM124_6*s->l[3][25]+OpM325_6*s->l[1][25]);
  FB325_6 = s->m[25]*(AlM325_6+OpM124_6*s->l[2][25]-OpM225_6*s->l[1][25]);
  FM125_6 = FB125_6+FM126_6*C26-FM226_6*S26;
  FM225_6 = FB225_6+FM126_6*S26+FM226_6*C26;
  FM325_6 = FB325_6+FB326_6-FB127_6*S27+FB327_6*C27;
  CM125_6 = s->In[1][25]*OpM124_6+s->In[3][25]*OpM325_6+CM126_6*C26-CM226_6*S26-FB225_6*s->l[3][25]+FB325_6*s->l[2][25]-
 s->dpt[3][49]*(FM126_6*S26+FM226_6*C26);
  CM225_6 = s->In[5][25]*OpM225_6+CM126_6*S26+CM226_6*C26+FB125_6*s->l[3][25]-FB325_6*s->l[1][25]+s->dpt[3][49]*(FM126_6
 *C26-FM226_6*S26);
  CM325_6 = CM326_6+s->In[3][25]*OpM124_6+s->In[9][25]*OpM325_6-FB125_6*s->l[2][25]+FB225_6*s->l[1][25];
  FB125_21 = s->m[25]*(AlM125_21+OpM225_21*s->l[3][25]-OpM325_21*s->l[2][25]);
  FB225_21 = s->m[25]*(AlM225_21-OpM124_21*s->l[3][25]+OpM325_21*s->l[1][25]);
  FB325_21 = s->m[25]*(AlM325_21+OpM124_21*s->l[2][25]-OpM225_21*s->l[1][25]);
  FM125_21 = FB125_21+FM126_21*C26-FM226_21*S26;
  FM225_21 = FB225_21+FM126_21*S26+FM226_21*C26;
  FM325_21 = FB325_21+FB326_21-FB127_21*S27+FB327_21*C27;
  CM125_21 = s->In[1][25]*OpM124_21+s->In[3][25]*OpM325_21+CM126_21*C26-CM226_21*S26-FB225_21*s->l[3][25]+FB325_21*
 s->l[2][25]-s->dpt[3][49]*(FM126_21*S26+FM226_21*C26);
  CM225_21 = s->In[5][25]*OpM225_21+CM126_21*S26+CM226_21*C26+FB125_21*s->l[3][25]-FB325_21*s->l[1][25]+s->dpt[3][49]*(
 FM126_21*C26-FM226_21*S26);
  CM325_21 = CM326_21+s->In[3][25]*OpM124_21+s->In[9][25]*OpM325_21-FB125_21*s->l[2][25]+FB225_21*s->l[1][25];
  FB125_22 = s->m[25]*(AlM125_22+OpM225_22*s->l[3][25]-OpM325_22*s->l[2][25]);
  FB225_22 = s->m[25]*(AlM225_22-OpM124_22*s->l[3][25]+OpM325_22*s->l[1][25]);
  FB325_22 = s->m[25]*(AlM325_22+OpM124_22*s->l[2][25]-OpM225_22*s->l[1][25]);
  FM125_22 = FB125_22+FM126_22*C26-FM226_22*S26;
  FM225_22 = FB225_22+FM126_22*S26+FM226_22*C26;
  FM325_22 = FB325_22+FB326_22-FB127_22*S27+FB327_22*C27;
  CM125_22 = s->In[1][25]*OpM124_22+s->In[3][25]*OpM325_22+CM126_22*C26-CM226_22*S26-FB225_22*s->l[3][25]+FB325_22*
 s->l[2][25]-s->dpt[3][49]*(FM126_22*S26+FM226_22*C26);
  CM225_22 = s->In[5][25]*OpM225_22+CM126_22*S26+CM226_22*C26+FB125_22*s->l[3][25]-FB325_22*s->l[1][25]+s->dpt[3][49]*(
 FM126_22*C26-FM226_22*S26);
  CM325_22 = CM326_22+s->In[3][25]*OpM124_22+s->In[9][25]*OpM325_22-FB125_22*s->l[2][25]+FB225_22*s->l[1][25];
  FB125_23 = s->m[25]*(AlM125_23+OpM225_23*s->l[3][25]-OpM325_23*s->l[2][25]);
  FB225_23 = s->m[25]*(AlM225_23+OpM325_23*s->l[1][25]+s->l[3][25]*S24);
  FB325_23 = s->m[25]*(AlM325_23-OpM225_23*s->l[1][25]-s->l[2][25]*S24);
  FM125_23 = FB125_23+FM126_23*C26-FM226_23*S26;
  FM225_23 = FB225_23+FM126_23*S26+FM226_23*C26;
  FM325_23 = FB325_23+FB326_23-FB127_23*S27+FB327_23*C27;
  CM125_23 = CM126_23*C26-CM226_23*S26-s->dpt[3][49]*(FM126_23*S26+FM226_23*C26)-s->In[1][25]*S24+s->In[3][25]*OpM325_23
 -FB225_23*s->l[3][25]+FB325_23*s->l[2][25];
  CM225_23 = s->In[5][25]*OpM225_23+CM126_23*S26+CM226_23*C26+FB125_23*s->l[3][25]-FB325_23*s->l[1][25]+s->dpt[3][49]*(
 FM126_23*C26-FM226_23*S26);
  CM325_23 = CM326_23-s->In[3][25]*S24+s->In[9][25]*OpM325_23-FB125_23*s->l[2][25]+FB225_23*s->l[1][25];
  FB125_24 = s->m[25]*(s->l[2][25]*S25+s->l[3][25]*C25);
  FB225_24 = -s->m[25]*s->l[1][25]*S25;
  FB325_24 = -s->m[25]*s->l[1][25]*C25;
  CM125_24 = CM126_24*C26-CM226_24*S26-s->dpt[3][49]*(FM126_24*S26+FM226_24*C26)-s->In[3][25]*S25-FB225_24*s->l[3][25]+
 FB325_24*s->l[2][25];
  CM125_25 = s->In[1][25]+s->m[25]*s->l[2][25]*s->l[2][25]+s->m[25]*s->l[3][25]*s->l[3][25]-s->dpt[3][49]*(C26*(FB226_25
 +FB227_25)+S26*(FB126_25+FB127_25*C27+FB327_25*S27))+C26*(s->In[1][26]*C26-s->In[2][26]*S26+CM127_25*C27+CM327_25*S27-
 FB226_25*s->l[3][26]-FB227_25*s->dpt[3][52]+FB326_25*s->l[2][26])-S26*(CM227_25+s->In[2][26]*C26-s->In[5][26]*S26+FB126_25*
 s->l[3][26]-FB326_25*s->l[1][26]+s->dpt[3][52]*(FB127_25*C27+FB327_25*S27));
  FA124 = -(s->frc[1][24]-s->m[24]*(AlF124+BeF224*s->l[2][24]-s->l[1][24]*(OM224*OM224+OM324*OM324)+s->l[3][24]*(BS324+
 OpF223)));
  FA224 = -(s->frc[2][24]-s->m[24]*(AlF224+BS524*s->l[2][24]+s->l[1][24]*(BS224+OpF324)+s->l[3][24]*(BS624-OpF124)));
  FA324 = -(s->frc[3][24]-s->m[24]*(AlF324+BeF824*s->l[2][24]+s->l[1][24]*(BS324-OpF223)-s->l[3][24]*(OM124*OM124+OM224*
 OM224)));
  FF124 = FA124+FF125;
  FF224 = FA224+FF225*C25-FF325*S25;
  FF324 = FA324+FF225*S25+FF325*C25;
  CF124 = -(s->trq[1][24]-CF125-s->In[1][24]*OpF124-s->In[2][24]*OpF223+FA224*s->l[3][24]-FA324*s->l[2][24]-OM224*(
 s->In[6][24]*OM224+s->In[9][24]*OM324)+OM324*(s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324)-s->dpt[2][47]*(FF225
 *S25+FF325*C25));
  CF224 = -(s->trq[2][24]-s->In[2][24]*OpF124-s->In[5][24]*OpF223-s->In[6][24]*OpF324-CF225*C25+CF325*S25-FA124*
 s->l[3][24]+FA324*s->l[1][24]+OM124*(s->In[6][24]*OM224+s->In[9][24]*OM324)-OM324*(s->In[1][24]*OM124+s->In[2][24]*OM224));
  CF324 = -(s->trq[3][24]-s->In[6][24]*OpF223-s->In[9][24]*OpF324-CF225*S25-CF325*C25+FA124*s->l[2][24]-FA224*
 s->l[1][24]+FF125*s->dpt[2][47]-OM124*(s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324)+OM224*(s->In[1][24]*OM124+
 s->In[2][24]*OM224));
  FB124_1 = s->m[24]*AlM124_1;
  FB224_1 = s->m[24]*AlM223_1;
  FB324_1 = s->m[24]*AlM324_1;
  FM124_1 = FB124_1+FM125_1;
  FM224_1 = FB224_1+FM225_1*C25-FM325_1*S25;
  FM324_1 = FB324_1+FM225_1*S25+FM325_1*C25;
  CM124_1 = CM125_1-FB224_1*s->l[3][24]+FB324_1*s->l[2][24]+s->dpt[2][47]*(FM225_1*S25+FM325_1*C25);
  CM224_1 = CM225_1*C25-CM325_1*S25+FB124_1*s->l[3][24]-FB324_1*s->l[1][24];
  CM324_1 = CM225_1*S25+CM325_1*C25-FB124_1*s->l[2][24]+FB224_1*s->l[1][24]-FM125_1*s->dpt[2][47];
  FB124_2 = s->m[24]*AlM124_2;
  FB224_2 = s->m[24]*AlM223_2;
  FB324_2 = s->m[24]*AlM324_2;
  FM124_2 = FB124_2+FM125_2;
  FM224_2 = FB224_2+FM225_2*C25-FM325_2*S25;
  FM324_2 = FB324_2+FM225_2*S25+FM325_2*C25;
  CM124_2 = CM125_2-FB224_2*s->l[3][24]+FB324_2*s->l[2][24]+s->dpt[2][47]*(FM225_2*S25+FM325_2*C25);
  CM224_2 = CM225_2*C25-CM325_2*S25+FB124_2*s->l[3][24]-FB324_2*s->l[1][24];
  CM324_2 = CM225_2*S25+CM325_2*C25-FB124_2*s->l[2][24]+FB224_2*s->l[1][24]-FM125_2*s->dpt[2][47];
  FB124_3 = s->m[24]*AlM124_3;
  FB224_3 = s->m[24]*AlM223_3;
  FB324_3 = s->m[24]*AlM324_3;
  FM124_3 = FB124_3+FM125_3;
  FM224_3 = FB224_3+FM225_3*C25-FM325_3*S25;
  FM324_3 = FB324_3+FM225_3*S25+FM325_3*C25;
  CM124_3 = CM125_3-FB224_3*s->l[3][24]+FB324_3*s->l[2][24]+s->dpt[2][47]*(FM225_3*S25+FM325_3*C25);
  CM224_3 = CM225_3*C25-CM325_3*S25+FB124_3*s->l[3][24]-FB324_3*s->l[1][24];
  CM324_3 = CM225_3*S25+CM325_3*C25-FB124_3*s->l[2][24]+FB224_3*s->l[1][24]-FM125_3*s->dpt[2][47];
  FB124_4 = s->m[24]*(AlM124_4+OpM223_4*s->l[3][24]-OpM324_4*s->l[2][24]);
  FB224_4 = s->m[24]*(AlM224_4-OpM124_4*s->l[3][24]+OpM324_4*s->l[1][24]);
  FB324_4 = s->m[24]*(AlM324_4+OpM124_4*s->l[2][24]-OpM223_4*s->l[1][24]);
  FM124_4 = FB124_4+FM125_4;
  FM224_4 = FB224_4+FM225_4*C25-FM325_4*S25;
  FM324_4 = FB324_4+FM225_4*S25+FM325_4*C25;
  CM124_4 = CM125_4+s->In[1][24]*OpM124_4+s->In[2][24]*OpM223_4-FB224_4*s->l[3][24]+FB324_4*s->l[2][24]+s->dpt[2][47]*(
 FM225_4*S25+FM325_4*C25);
  CM224_4 = s->In[2][24]*OpM124_4+s->In[5][24]*OpM223_4+s->In[6][24]*OpM324_4+CM225_4*C25-CM325_4*S25+FB124_4*
 s->l[3][24]-FB324_4*s->l[1][24];
  CM324_4 = s->In[6][24]*OpM223_4+s->In[9][24]*OpM324_4+CM225_4*S25+CM325_4*C25-FB124_4*s->l[2][24]+FB224_4*s->l[1][24]-
 FM125_4*s->dpt[2][47];
  FB124_5 = s->m[24]*(AlM124_5+OpM223_5*s->l[3][24]-OpM324_5*s->l[2][24]);
  FB224_5 = s->m[24]*(AlM224_5-OpM124_5*s->l[3][24]+OpM324_5*s->l[1][24]);
  FB324_5 = s->m[24]*(AlM324_5+OpM124_5*s->l[2][24]-OpM223_5*s->l[1][24]);
  FM124_5 = FB124_5+FM125_5;
  FM224_5 = FB224_5+FM225_5*C25-FM325_5*S25;
  FM324_5 = FB324_5+FM225_5*S25+FM325_5*C25;
  CM124_5 = CM125_5+s->In[1][24]*OpM124_5+s->In[2][24]*OpM223_5-FB224_5*s->l[3][24]+FB324_5*s->l[2][24]+s->dpt[2][47]*(
 FM225_5*S25+FM325_5*C25);
  CM224_5 = s->In[2][24]*OpM124_5+s->In[5][24]*OpM223_5+s->In[6][24]*OpM324_5+CM225_5*C25-CM325_5*S25+FB124_5*
 s->l[3][24]-FB324_5*s->l[1][24];
  CM324_5 = s->In[6][24]*OpM223_5+s->In[9][24]*OpM324_5+CM225_5*S25+CM325_5*C25-FB124_5*s->l[2][24]+FB224_5*s->l[1][24]-
 FM125_5*s->dpt[2][47];
  FB124_6 = s->m[24]*(AlM124_6+OpM223_6*s->l[3][24]-OpM324_6*s->l[2][24]);
  FB224_6 = s->m[24]*(AlM224_6-OpM124_6*s->l[3][24]+OpM324_6*s->l[1][24]);
  FB324_6 = s->m[24]*(AlM324_6+OpM124_6*s->l[2][24]-OpM223_6*s->l[1][24]);
  FM124_6 = FB124_6+FM125_6;
  FM224_6 = FB224_6+FM225_6*C25-FM325_6*S25;
  FM324_6 = FB324_6+FM225_6*S25+FM325_6*C25;
  CM124_6 = CM125_6+s->In[1][24]*OpM124_6+s->In[2][24]*OpM223_6-FB224_6*s->l[3][24]+FB324_6*s->l[2][24]+s->dpt[2][47]*(
 FM225_6*S25+FM325_6*C25);
  CM224_6 = s->In[2][24]*OpM124_6+s->In[5][24]*OpM223_6+s->In[6][24]*OpM324_6+CM225_6*C25-CM325_6*S25+FB124_6*
 s->l[3][24]-FB324_6*s->l[1][24];
  CM324_6 = s->In[6][24]*OpM223_6+s->In[9][24]*OpM324_6+CM225_6*S25+CM325_6*C25-FB124_6*s->l[2][24]+FB224_6*s->l[1][24]-
 FM125_6*s->dpt[2][47];
  FB124_21 = s->m[24]*(AlM124_21+OpM223_21*s->l[3][24]-OpM324_21*s->l[2][24]);
  FB224_21 = s->m[24]*(AlM224_21-OpM124_21*s->l[3][24]+OpM324_21*s->l[1][24]);
  FB324_21 = s->m[24]*(AlM324_21+OpM124_21*s->l[2][24]-OpM223_21*s->l[1][24]);
  FM124_21 = FB124_21+FM125_21;
  FM224_21 = FB224_21+FM225_21*C25-FM325_21*S25;
  FM324_21 = FB324_21+FM225_21*S25+FM325_21*C25;
  CM124_21 = CM125_21+s->In[1][24]*OpM124_21+s->In[2][24]*OpM223_21-FB224_21*s->l[3][24]+FB324_21*s->l[2][24]+
 s->dpt[2][47]*(FM225_21*S25+FM325_21*C25);
  CM224_21 = s->In[2][24]*OpM124_21+s->In[5][24]*OpM223_21+s->In[6][24]*OpM324_21+CM225_21*C25-CM325_21*S25+FB124_21*
 s->l[3][24]-FB324_21*s->l[1][24];
  CM324_21 = s->In[6][24]*OpM223_21+s->In[9][24]*OpM324_21+CM225_21*S25+CM325_21*C25-FB124_21*s->l[2][24]+FB224_21*
 s->l[1][24]-FM125_21*s->dpt[2][47];
  FB124_22 = s->m[24]*(AlM124_22-OpM324_22*s->l[2][24]+s->l[3][24]*C23);
  FB224_22 = s->m[24]*(AlM224_22-OpM124_22*s->l[3][24]+OpM324_22*s->l[1][24]);
  FB324_22 = s->m[24]*(AlM324_22+OpM124_22*s->l[2][24]-s->l[1][24]*C23);
  FM124_22 = FB124_22+FM125_22;
  FM224_22 = FB224_22+FM225_22*C25-FM325_22*S25;
  FM324_22 = FB324_22+FM225_22*S25+FM325_22*C25;
  CM124_22 = CM125_22+s->In[1][24]*OpM124_22+s->In[2][24]*C23-FB224_22*s->l[3][24]+FB324_22*s->l[2][24]+s->dpt[2][47]*(
 FM225_22*S25+FM325_22*C25);
  CM224_22 = s->In[2][24]*OpM124_22+s->In[5][24]*C23+s->In[6][24]*OpM324_22+CM225_22*C25-CM325_22*S25+FB124_22*
 s->l[3][24]-FB324_22*s->l[1][24];
  CM324_22 = s->In[6][24]*C23+s->In[9][24]*OpM324_22+CM225_22*S25+CM325_22*C25-FB124_22*s->l[2][24]+FB224_22*s->l[1][24]
 -FM125_22*s->dpt[2][47];
  FB124_23 = s->m[24]*(AlM124_23-s->l[2][24]*C24);
  FB224_23 = s->m[24]*(s->dpt[1][44]+s->l[1][24]*C24+s->l[3][24]*S24);
  FB324_23 = -s->m[24]*S24*(s->dpt[2][44]+s->l[2][24]);
  CM224_23 = CM225_23*C25-CM325_23*S25-s->In[2][24]*S24+s->In[6][24]*C24+FB124_23*s->l[3][24]-FB324_23*s->l[1][24];
  CM224_24 = s->In[5][24]+s->m[24]*s->l[1][24]*s->l[1][24]+s->m[24]*s->l[3][24]*s->l[3][24]+C25*(s->In[5][25]*C25+
 CM126_24*S26+CM226_24*C26+FB125_24*s->l[3][25]-FB325_24*s->l[1][25]+s->dpt[3][49]*(FM126_24*C26-FM226_24*S26))-S25*(CM326_24
 -s->In[9][25]*S25-FB125_24*s->l[2][25]+FB225_24*s->l[1][25]);

// = = Block_0_2_0_1_0_6 = = 
 
// Backward Dynamics 

  FA131 = -(s->frc[1][31]+s->m[31]*(s->l[1][31]*(OM231*OM231+OM331*OM331)-s->l[2][31]*(BS231-OpF331)-s->l[3][31]*(BS331+
 OpF230)-C31*(AlF130+BeF330*s->dpt[3][61])+S31*(AlF330+BS930*s->dpt[3][61])));
  FA231 = -(s->frc[2][31]-s->m[31]*(AlF230+BeF630*s->dpt[3][61]+s->l[1][31]*(BS231+OpF331)-s->l[2][31]*(OM131*OM131+
 OM331*OM331)+s->l[3][31]*(BS631-OpF131)));
  FA331 = -(s->frc[3][31]-s->m[31]*(s->l[1][31]*(BS331-OpF230)+s->l[2][31]*(BS631+OpF131)-s->l[3][31]*(OM131*OM131+OM231
 *OM231)+C31*(AlF330+BS930*s->dpt[3][61])+S31*(AlF130+BeF330*s->dpt[3][61])));
  CF131 = -(s->trq[1][31]-s->In[1][31]*OpF131-s->In[2][31]*OpF230+FA231*s->l[3][31]-FA331*s->l[2][31]-OM231*(
 s->In[6][31]*OM231+s->In[9][31]*OM331)+OM331*(s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331));
  CF231 = -(s->trq[2][31]-s->In[2][31]*OpF131-s->In[5][31]*OpF230-s->In[6][31]*OpF331-FA131*s->l[3][31]+FA331*
 s->l[1][31]+OM131*(s->In[6][31]*OM231+s->In[9][31]*OM331)-OM331*(s->In[1][31]*OM131+s->In[2][31]*OM231));
  CF331 = -(s->trq[3][31]-s->In[6][31]*OpF230-s->In[9][31]*OpF331+FA131*s->l[2][31]-FA231*s->l[1][31]-OM131*(
 s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331)+OM231*(s->In[1][31]*OM131+s->In[2][31]*OM231));
  FB131_1 = s->m[31]*(AlM130_1*C31-AlM329_1*S31);
  FB231_1 = s->m[31]*AlM230_1;
  FB331_1 = s->m[31]*(AlM130_1*S31+AlM329_1*C31);
  CM131_1 = -(FB231_1*s->l[3][31]-FB331_1*s->l[2][31]);
  CM231_1 = FB131_1*s->l[3][31]-FB331_1*s->l[1][31];
  CM331_1 = -(FB131_1*s->l[2][31]-FB231_1*s->l[1][31]);
  FB131_2 = s->m[31]*(AlM130_2*C31-AlM329_2*S31);
  FB231_2 = s->m[31]*AlM230_2;
  FB331_2 = s->m[31]*(AlM130_2*S31+AlM329_2*C31);
  CM131_2 = -(FB231_2*s->l[3][31]-FB331_2*s->l[2][31]);
  CM231_2 = FB131_2*s->l[3][31]-FB331_2*s->l[1][31];
  CM331_2 = -(FB131_2*s->l[2][31]-FB231_2*s->l[1][31]);
  FB131_3 = s->m[31]*(AlM130_3*C31-AlM329_3*S31);
  FB231_3 = s->m[31]*AlM230_3;
  FB331_3 = s->m[31]*(AlM130_3*S31+AlM329_3*C31);
  CM131_3 = -(FB231_3*s->l[3][31]-FB331_3*s->l[2][31]);
  CM231_3 = FB131_3*s->l[3][31]-FB331_3*s->l[1][31];
  CM331_3 = -(FB131_3*s->l[2][31]-FB231_3*s->l[1][31]);
  FB131_4 = -s->m[31]*(AlM329_4*S31-OpM230_4*s->l[3][31]+OpM331_4*s->l[2][31]-C31*(AlM130_4+OpM230_4*s->dpt[3][61]));
  FB231_4 = s->m[31]*(AlM230_4-OpM130_4*s->dpt[3][61]-OpM131_4*s->l[3][31]+OpM331_4*s->l[1][31]);
  FB331_4 = s->m[31]*(AlM329_4*C31+OpM131_4*s->l[2][31]-OpM230_4*s->l[1][31]+S31*(AlM130_4+OpM230_4*s->dpt[3][61]));
  CM131_4 = s->In[1][31]*OpM131_4+s->In[2][31]*OpM230_4-FB231_4*s->l[3][31]+FB331_4*s->l[2][31];
  CM231_4 = s->In[2][31]*OpM131_4+s->In[5][31]*OpM230_4+s->In[6][31]*OpM331_4+FB131_4*s->l[3][31]-FB331_4*s->l[1][31];
  CM331_4 = s->In[6][31]*OpM230_4+s->In[9][31]*OpM331_4-FB131_4*s->l[2][31]+FB231_4*s->l[1][31];
  FB131_5 = -s->m[31]*(AlM329_5*S31-OpM230_5*s->l[3][31]+OpM331_5*s->l[2][31]-C31*(AlM130_5+OpM230_5*s->dpt[3][61]));
  FB231_5 = s->m[31]*(AlM230_5-OpM130_5*s->dpt[3][61]-OpM131_5*s->l[3][31]+OpM331_5*s->l[1][31]);
  FB331_5 = s->m[31]*(AlM329_5*C31+OpM131_5*s->l[2][31]-OpM230_5*s->l[1][31]+S31*(AlM130_5+OpM230_5*s->dpt[3][61]));
  CM131_5 = s->In[1][31]*OpM131_5+s->In[2][31]*OpM230_5-FB231_5*s->l[3][31]+FB331_5*s->l[2][31];
  CM231_5 = s->In[2][31]*OpM131_5+s->In[5][31]*OpM230_5+s->In[6][31]*OpM331_5+FB131_5*s->l[3][31]-FB331_5*s->l[1][31];
  CM331_5 = s->In[6][31]*OpM230_5+s->In[9][31]*OpM331_5-FB131_5*s->l[2][31]+FB231_5*s->l[1][31];
  FB131_6 = -s->m[31]*(AlM329_6*S31-OpM230_6*s->l[3][31]+OpM331_6*s->l[2][31]-C31*(AlM130_6+OpM230_6*s->dpt[3][61]));
  FB231_6 = s->m[31]*(AlM230_6-OpM130_6*s->dpt[3][61]-OpM131_6*s->l[3][31]+OpM331_6*s->l[1][31]);
  FB331_6 = s->m[31]*(AlM329_6*C31+OpM131_6*s->l[2][31]-OpM230_6*s->l[1][31]+S31*(AlM130_6+OpM230_6*s->dpt[3][61]));
  CM131_6 = s->In[1][31]*OpM131_6+s->In[2][31]*OpM230_6-FB231_6*s->l[3][31]+FB331_6*s->l[2][31];
  CM231_6 = s->In[2][31]*OpM131_6+s->In[5][31]*OpM230_6+s->In[6][31]*OpM331_6+FB131_6*s->l[3][31]-FB331_6*s->l[1][31];
  CM331_6 = s->In[6][31]*OpM230_6+s->In[9][31]*OpM331_6-FB131_6*s->l[2][31]+FB231_6*s->l[1][31];
  FB131_21 = -s->m[31]*(AlM329_21*S31-OpM230_21*s->l[3][31]+OpM331_21*s->l[2][31]-C31*(AlM130_21+OpM230_21*s->dpt[3][61]
 ));
  FB231_21 = s->m[31]*(AlM230_21-OpM130_21*s->dpt[3][61]-OpM131_21*s->l[3][31]+OpM331_21*s->l[1][31]);
  FB331_21 = s->m[31]*(AlM329_21*C31+OpM131_21*s->l[2][31]-OpM230_21*s->l[1][31]+S31*(AlM130_21+OpM230_21*s->dpt[3][61])
 );
  CM131_21 = s->In[1][31]*OpM131_21+s->In[2][31]*OpM230_21-FB231_21*s->l[3][31]+FB331_21*s->l[2][31];
  CM231_21 = s->In[2][31]*OpM131_21+s->In[5][31]*OpM230_21+s->In[6][31]*OpM331_21+FB131_21*s->l[3][31]-FB331_21*
 s->l[1][31];
  CM331_21 = s->In[6][31]*OpM230_21+s->In[9][31]*OpM331_21-FB131_21*s->l[2][31]+FB231_21*s->l[1][31];
  FB131_22 = -s->m[31]*(AlM329_22*S31-OpM230_22*s->l[3][31]+OpM331_22*s->l[2][31]-C31*(AlM130_22+OpM230_22*s->dpt[3][61]
 ));
  FB231_22 = s->m[31]*(AlM230_22-OpM130_22*s->dpt[3][61]-OpM131_22*s->l[3][31]+OpM331_22*s->l[1][31]);
  FB331_22 = s->m[31]*(AlM329_22*C31+OpM131_22*s->l[2][31]-OpM230_22*s->l[1][31]+S31*(AlM130_22+OpM230_22*s->dpt[3][61])
 );
  CM131_22 = s->In[1][31]*OpM131_22+s->In[2][31]*OpM230_22-FB231_22*s->l[3][31]+FB331_22*s->l[2][31];
  CM231_22 = s->In[2][31]*OpM131_22+s->In[5][31]*OpM230_22+s->In[6][31]*OpM331_22+FB131_22*s->l[3][31]-FB331_22*
 s->l[1][31];
  CM331_22 = s->In[6][31]*OpM230_22+s->In[9][31]*OpM331_22-FB131_22*s->l[2][31]+FB231_22*s->l[1][31];
  FB131_23 = -s->m[31]*(AlM329_23*S31-OpM230_23*s->l[3][31]+OpM331_23*s->l[2][31]-C31*(AlM130_23+OpM230_23*s->dpt[3][61]
 ));
  FB231_23 = s->m[31]*(AlM230_23-OpM130_23*s->dpt[3][61]-OpM131_23*s->l[3][31]+OpM331_23*s->l[1][31]);
  FB331_23 = s->m[31]*(AlM329_23*C31+OpM131_23*s->l[2][31]-OpM230_23*s->l[1][31]+S31*(AlM130_23+OpM230_23*s->dpt[3][61])
 );
  CM131_23 = s->In[1][31]*OpM131_23+s->In[2][31]*OpM230_23-FB231_23*s->l[3][31]+FB331_23*s->l[2][31];
  CM231_23 = s->In[2][31]*OpM131_23+s->In[5][31]*OpM230_23+s->In[6][31]*OpM331_23+FB131_23*s->l[3][31]-FB331_23*
 s->l[1][31];
  CM331_23 = s->In[6][31]*OpM230_23+s->In[9][31]*OpM331_23-FB131_23*s->l[2][31]+FB231_23*s->l[1][31];
  FB131_28 = s->m[31]*(OpM230_28*s->l[3][31]-OpM331_28*s->l[2][31]+C31*(AlM130_28+OpM230_28*s->dpt[3][61]));
  FB231_28 = s->m[31]*(AlM230_28-OpM130_28*s->dpt[3][61]-OpM131_28*s->l[3][31]+OpM331_28*s->l[1][31]);
  FB331_28 = s->m[31]*(OpM131_28*s->l[2][31]-OpM230_28*s->l[1][31]+S31*(AlM130_28+OpM230_28*s->dpt[3][61]));
  CM131_28 = s->In[1][31]*OpM131_28+s->In[2][31]*OpM230_28-FB231_28*s->l[3][31]+FB331_28*s->l[2][31];
  CM231_28 = s->In[2][31]*OpM131_28+s->In[5][31]*OpM230_28+s->In[6][31]*OpM331_28+FB131_28*s->l[3][31]-FB331_28*
 s->l[1][31];
  CM331_28 = s->In[6][31]*OpM230_28+s->In[9][31]*OpM331_28-FB131_28*s->l[2][31]+FB231_28*s->l[1][31];
  FB131_29 = s->m[31]*(C31*(AlM130_29-s->dpt[3][61]*S30)-OpM331_29*s->l[2][31]-s->l[3][31]*S30);
  FB231_29 = s->m[31]*(AlM230_29-OpM131_29*s->l[3][31]+OpM331_29*s->l[1][31]-s->dpt[3][61]*C30);
  FB331_29 = s->m[31]*(OpM131_29*s->l[2][31]+s->l[1][31]*S30+S31*(AlM130_29-s->dpt[3][61]*S30));
  CM131_29 = s->In[1][31]*OpM131_29-s->In[2][31]*S30-FB231_29*s->l[3][31]+FB331_29*s->l[2][31];
  CM231_29 = s->In[2][31]*OpM131_29-s->In[5][31]*S30+s->In[6][31]*OpM331_29+FB131_29*s->l[3][31]-FB331_29*s->l[1][31];
  CM331_29 = -(s->In[6][31]*S30-s->In[9][31]*OpM331_29+FB131_29*s->l[2][31]-FB231_29*s->l[1][31]);
  FB131_30 = -s->m[31]*s->l[2][31]*C31;
  FB231_30 = s->m[31]*(s->l[1][31]*C31+s->l[3][31]*S31);
  FB331_30 = -s->m[31]*s->l[2][31]*S31;
  CM231_30 = -(s->In[2][31]*S31-s->In[6][31]*C31-FB131_30*s->l[3][31]+FB331_30*s->l[1][31]);
  CM231_31 = s->In[5][31]+s->m[31]*s->l[1][31]*s->l[1][31]+s->m[31]*s->l[3][31]*s->l[3][31];
  FA130 = -(s->frc[1][30]-s->m[30]*(AlF130+BeF330*s->l[3][30]-s->l[1][30]*(OM230*OM230+OM330*OM330)+s->l[2][30]*(BS230-
 OpF329)));
  FA230 = -(s->frc[2][30]-s->m[30]*(AlF230+BeF630*s->l[3][30]+s->l[1][30]*(BS230+OpF329)-s->l[2][30]*(OM130*OM130+OM330*
 OM330)));
  FA330 = -(s->frc[3][30]-s->m[30]*(AlF330+BS930*s->l[3][30]+s->l[1][30]*(BS330-OpF230)+s->l[2][30]*(BS630+OpF130)));
  FF130 = FA130+FA131*C31+FA331*S31;
  FF230 = FA230+FA231;
  CF130 = -(s->trq[1][30]-s->In[1][30]*OpF130-s->In[3][30]*OpF329-CF131*C31-CF331*S31+FA230*s->l[3][30]+FA231*
 s->dpt[3][61]-FA330*s->l[2][30]-OM230*(s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)+OM330*(s->In[5][30]*OM230+
 s->In[6][30]*OM330));
  CF230 = -(s->trq[2][30]-CF231-s->In[5][30]*OpF230-s->In[6][30]*OpF329-FA130*s->l[3][30]+FA330*s->l[1][30]+OM130*(
 s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)-OM330*(s->In[1][30]*OM130+s->In[3][30]*OM330)-s->dpt[3][61]*(FA131
 *C31+FA331*S31));
  CF330 = -(s->trq[3][30]-s->In[3][30]*OpF130-s->In[6][30]*OpF230-s->In[9][30]*OpF329+CF131*S31-CF331*C31+FA130*
 s->l[2][30]-FA230*s->l[1][30]-OM130*(s->In[5][30]*OM230+s->In[6][30]*OM330)+OM230*(s->In[1][30]*OM130+s->In[3][30]*OM330));
  FB130_1 = s->m[30]*AlM130_1;
  FB230_1 = s->m[30]*AlM230_1;
  FB330_1 = s->m[30]*AlM329_1;
  FM130_1 = FB130_1+FB131_1*C31+FB331_1*S31;
  FM230_1 = FB230_1+FB231_1;
  CM130_1 = CM131_1*C31+CM331_1*S31-FB230_1*s->l[3][30]-FB231_1*s->dpt[3][61]+FB330_1*s->l[2][30];
  CM230_1 = CM231_1+FB130_1*s->l[3][30]-FB330_1*s->l[1][30]+s->dpt[3][61]*(FB131_1*C31+FB331_1*S31);
  CM330_1 = -(CM131_1*S31-CM331_1*C31+FB130_1*s->l[2][30]-FB230_1*s->l[1][30]);
  FB130_2 = s->m[30]*AlM130_2;
  FB230_2 = s->m[30]*AlM230_2;
  FB330_2 = s->m[30]*AlM329_2;
  FM130_2 = FB130_2+FB131_2*C31+FB331_2*S31;
  FM230_2 = FB230_2+FB231_2;
  CM130_2 = CM131_2*C31+CM331_2*S31-FB230_2*s->l[3][30]-FB231_2*s->dpt[3][61]+FB330_2*s->l[2][30];
  CM230_2 = CM231_2+FB130_2*s->l[3][30]-FB330_2*s->l[1][30]+s->dpt[3][61]*(FB131_2*C31+FB331_2*S31);
  CM330_2 = -(CM131_2*S31-CM331_2*C31+FB130_2*s->l[2][30]-FB230_2*s->l[1][30]);
  FB130_3 = s->m[30]*AlM130_3;
  FB230_3 = s->m[30]*AlM230_3;
  FB330_3 = s->m[30]*AlM329_3;
  FM130_3 = FB130_3+FB131_3*C31+FB331_3*S31;
  FM230_3 = FB230_3+FB231_3;
  CM130_3 = CM131_3*C31+CM331_3*S31-FB230_3*s->l[3][30]-FB231_3*s->dpt[3][61]+FB330_3*s->l[2][30];
  CM230_3 = CM231_3+FB130_3*s->l[3][30]-FB330_3*s->l[1][30]+s->dpt[3][61]*(FB131_3*C31+FB331_3*S31);
  CM330_3 = -(CM131_3*S31-CM331_3*C31+FB130_3*s->l[2][30]-FB230_3*s->l[1][30]);
  FB130_4 = s->m[30]*(AlM130_4+OpM230_4*s->l[3][30]-OpM329_4*s->l[2][30]);
  FB230_4 = s->m[30]*(AlM230_4-OpM130_4*s->l[3][30]+OpM329_4*s->l[1][30]);
  FB330_4 = s->m[30]*(AlM329_4+OpM130_4*s->l[2][30]-OpM230_4*s->l[1][30]);
  FM130_4 = FB130_4+FB131_4*C31+FB331_4*S31;
  FM230_4 = FB230_4+FB231_4;
  CM130_4 = s->In[1][30]*OpM130_4+s->In[3][30]*OpM329_4+CM131_4*C31+CM331_4*S31-FB230_4*s->l[3][30]-FB231_4*
 s->dpt[3][61]+FB330_4*s->l[2][30];
  CM230_4 = CM231_4+s->In[5][30]*OpM230_4+s->In[6][30]*OpM329_4+FB130_4*s->l[3][30]-FB330_4*s->l[1][30]+s->dpt[3][61]*(
 FB131_4*C31+FB331_4*S31);
  CM330_4 = s->In[3][30]*OpM130_4+s->In[6][30]*OpM230_4+s->In[9][30]*OpM329_4-CM131_4*S31+CM331_4*C31-FB130_4*
 s->l[2][30]+FB230_4*s->l[1][30];
  FB130_5 = s->m[30]*(AlM130_5+OpM230_5*s->l[3][30]-OpM329_5*s->l[2][30]);
  FB230_5 = s->m[30]*(AlM230_5-OpM130_5*s->l[3][30]+OpM329_5*s->l[1][30]);
  FB330_5 = s->m[30]*(AlM329_5+OpM130_5*s->l[2][30]-OpM230_5*s->l[1][30]);
  FM130_5 = FB130_5+FB131_5*C31+FB331_5*S31;
  FM230_5 = FB230_5+FB231_5;
  CM130_5 = s->In[1][30]*OpM130_5+s->In[3][30]*OpM329_5+CM131_5*C31+CM331_5*S31-FB230_5*s->l[3][30]-FB231_5*
 s->dpt[3][61]+FB330_5*s->l[2][30];
  CM230_5 = CM231_5+s->In[5][30]*OpM230_5+s->In[6][30]*OpM329_5+FB130_5*s->l[3][30]-FB330_5*s->l[1][30]+s->dpt[3][61]*(
 FB131_5*C31+FB331_5*S31);
  CM330_5 = s->In[3][30]*OpM130_5+s->In[6][30]*OpM230_5+s->In[9][30]*OpM329_5-CM131_5*S31+CM331_5*C31-FB130_5*
 s->l[2][30]+FB230_5*s->l[1][30];
  FB130_6 = s->m[30]*(AlM130_6+OpM230_6*s->l[3][30]-OpM329_6*s->l[2][30]);
  FB230_6 = s->m[30]*(AlM230_6-OpM130_6*s->l[3][30]+OpM329_6*s->l[1][30]);
  FB330_6 = s->m[30]*(AlM329_6+OpM130_6*s->l[2][30]-OpM230_6*s->l[1][30]);
  FM130_6 = FB130_6+FB131_6*C31+FB331_6*S31;
  FM230_6 = FB230_6+FB231_6;
  CM130_6 = s->In[1][30]*OpM130_6+s->In[3][30]*OpM329_6+CM131_6*C31+CM331_6*S31-FB230_6*s->l[3][30]-FB231_6*
 s->dpt[3][61]+FB330_6*s->l[2][30];
  CM230_6 = CM231_6+s->In[5][30]*OpM230_6+s->In[6][30]*OpM329_6+FB130_6*s->l[3][30]-FB330_6*s->l[1][30]+s->dpt[3][61]*(
 FB131_6*C31+FB331_6*S31);
  CM330_6 = s->In[3][30]*OpM130_6+s->In[6][30]*OpM230_6+s->In[9][30]*OpM329_6-CM131_6*S31+CM331_6*C31-FB130_6*
 s->l[2][30]+FB230_6*s->l[1][30];
  FB130_21 = s->m[30]*(AlM130_21+OpM230_21*s->l[3][30]-OpM329_21*s->l[2][30]);
  FB230_21 = s->m[30]*(AlM230_21-OpM130_21*s->l[3][30]+OpM329_21*s->l[1][30]);
  FB330_21 = s->m[30]*(AlM329_21+OpM130_21*s->l[2][30]-OpM230_21*s->l[1][30]);
  FM130_21 = FB130_21+FB131_21*C31+FB331_21*S31;
  FM230_21 = FB230_21+FB231_21;
  CM130_21 = s->In[1][30]*OpM130_21+s->In[3][30]*OpM329_21+CM131_21*C31+CM331_21*S31-FB230_21*s->l[3][30]-FB231_21*
 s->dpt[3][61]+FB330_21*s->l[2][30];
  CM230_21 = CM231_21+s->In[5][30]*OpM230_21+s->In[6][30]*OpM329_21+FB130_21*s->l[3][30]-FB330_21*s->l[1][30]+
 s->dpt[3][61]*(FB131_21*C31+FB331_21*S31);
  CM330_21 = s->In[3][30]*OpM130_21+s->In[6][30]*OpM230_21+s->In[9][30]*OpM329_21-CM131_21*S31+CM331_21*C31-FB130_21*
 s->l[2][30]+FB230_21*s->l[1][30];
  FB130_22 = s->m[30]*(AlM130_22+OpM230_22*s->l[3][30]-OpM329_22*s->l[2][30]);
  FB230_22 = s->m[30]*(AlM230_22-OpM130_22*s->l[3][30]+OpM329_22*s->l[1][30]);
  FB330_22 = s->m[30]*(AlM329_22+OpM130_22*s->l[2][30]-OpM230_22*s->l[1][30]);
  FM130_22 = FB130_22+FB131_22*C31+FB331_22*S31;
  FM230_22 = FB230_22+FB231_22;
  CM130_22 = s->In[1][30]*OpM130_22+s->In[3][30]*OpM329_22+CM131_22*C31+CM331_22*S31-FB230_22*s->l[3][30]-FB231_22*
 s->dpt[3][61]+FB330_22*s->l[2][30];
  CM230_22 = CM231_22+s->In[5][30]*OpM230_22+s->In[6][30]*OpM329_22+FB130_22*s->l[3][30]-FB330_22*s->l[1][30]+
 s->dpt[3][61]*(FB131_22*C31+FB331_22*S31);
  CM330_22 = s->In[3][30]*OpM130_22+s->In[6][30]*OpM230_22+s->In[9][30]*OpM329_22-CM131_22*S31+CM331_22*C31-FB130_22*
 s->l[2][30]+FB230_22*s->l[1][30];
  FB130_23 = s->m[30]*(AlM130_23+OpM230_23*s->l[3][30]-OpM329_23*s->l[2][30]);
  FB230_23 = s->m[30]*(AlM230_23-OpM130_23*s->l[3][30]+OpM329_23*s->l[1][30]);
  FB330_23 = s->m[30]*(AlM329_23+OpM130_23*s->l[2][30]-OpM230_23*s->l[1][30]);
  FM130_23 = FB130_23+FB131_23*C31+FB331_23*S31;
  FM230_23 = FB230_23+FB231_23;
  CM130_23 = s->In[1][30]*OpM130_23+s->In[3][30]*OpM329_23+CM131_23*C31+CM331_23*S31-FB230_23*s->l[3][30]-FB231_23*
 s->dpt[3][61]+FB330_23*s->l[2][30];
  CM230_23 = CM231_23+s->In[5][30]*OpM230_23+s->In[6][30]*OpM329_23+FB130_23*s->l[3][30]-FB330_23*s->l[1][30]+
 s->dpt[3][61]*(FB131_23*C31+FB331_23*S31);
  CM330_23 = s->In[3][30]*OpM130_23+s->In[6][30]*OpM230_23+s->In[9][30]*OpM329_23-CM131_23*S31+CM331_23*C31-FB130_23*
 s->l[2][30]+FB230_23*s->l[1][30];
  FB130_28 = s->m[30]*(AlM130_28+OpM230_28*s->l[3][30]+s->l[2][30]*S29);
  FB230_28 = s->m[30]*(AlM230_28-OpM130_28*s->l[3][30]-s->l[1][30]*S29);
  FB330_28 = s->m[30]*(OpM130_28*s->l[2][30]-OpM230_28*s->l[1][30]);
  FM130_28 = FB130_28+FB131_28*C31+FB331_28*S31;
  FM230_28 = FB230_28+FB231_28;
  CM130_28 = s->In[1][30]*OpM130_28-s->In[3][30]*S29+CM131_28*C31+CM331_28*S31-FB230_28*s->l[3][30]-FB231_28*
 s->dpt[3][61]+FB330_28*s->l[2][30];
  CM230_28 = CM231_28+s->In[5][30]*OpM230_28-s->In[6][30]*S29+FB130_28*s->l[3][30]-FB330_28*s->l[1][30]+s->dpt[3][61]*(
 FB131_28*C31+FB331_28*S31);
  CM330_28 = s->In[3][30]*OpM130_28+s->In[6][30]*OpM230_28-s->In[9][30]*S29-CM131_28*S31+CM331_28*C31-FB130_28*
 s->l[2][30]+FB230_28*s->l[1][30];
  FB130_29 = s->m[30]*(AlM130_29-s->l[3][30]*S30);
  FB230_29 = s->m[30]*(AlM230_29-s->l[3][30]*C30);
  FB330_29 = s->m[30]*(s->l[1][30]*S30+s->l[2][30]*C30);
  CM330_29 = s->In[3][30]*C30-s->In[6][30]*S30-CM131_29*S31+CM331_29*C31-FB130_29*s->l[2][30]+FB230_29*s->l[1][30];
  CM330_30 = s->In[9][30]+s->m[30]*s->l[1][30]*s->l[1][30]+s->m[30]*s->l[2][30]*s->l[2][30]+C31*(s->In[9][31]*C31-
 FB131_30*s->l[2][31]+FB231_30*s->l[1][31])+S31*(s->In[1][31]*S31+FB231_30*s->l[3][31]-FB331_30*s->l[2][31]);
  FA129 = -(s->frc[1][29]-s->m[29]*(AlF129+BeF329*s->l[3][29]-s->l[1][29]*(OM229*OM229+OM329*OM329)+s->l[2][29]*(BS229-
 OpF329)));
  FA229 = -(s->frc[2][29]-s->m[29]*(AlF229+BeF629*s->l[3][29]+s->l[1][29]*(BS229+OpF329)-s->l[2][29]*(OM129*OM129+OM329*
 OM329)));
  FA329 = -(s->frc[3][29]-s->m[29]*(AlF329+BS929*s->l[3][29]+s->l[1][29]*(BS329-OpF229)+s->l[2][29]*(BS629+OpF128)));
  FF129 = FA129+FF130*C30-FF230*S30;
  FF229 = FA229+FF130*S30+FF230*C30;
  FF329 = FA329+FA330-FA131*S31+FA331*C31;
  CF129 = -(s->trq[1][29]-s->In[1][29]*OpF128-s->In[2][29]*OpF229-s->In[3][29]*OpF329-CF130*C30+CF230*S30+FA229*
 s->l[3][29]-FA329*s->l[2][29]-OM229*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)+OM329*(s->In[2][29]*OM129+
 s->In[5][29]*OM229+s->In[6][29]*OM329)+s->dpt[3][58]*(FF130*S30+FF230*C30));
  CF229 = -(s->trq[2][29]-s->In[2][29]*OpF128-s->In[5][29]*OpF229-s->In[6][29]*OpF329-CF130*S30-CF230*C30-FA129*
 s->l[3][29]+FA329*s->l[1][29]+OM129*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)-OM329*(s->In[1][29]*OM129+
 s->In[2][29]*OM229+s->In[3][29]*OM329)-s->dpt[3][58]*(FF130*C30-FF230*S30));
  CF329 = -(s->trq[3][29]-CF330-s->In[3][29]*OpF128-s->In[6][29]*OpF229-s->In[9][29]*OpF329+FA129*s->l[2][29]-FA229*
 s->l[1][29]-OM129*(s->In[2][29]*OM129+s->In[5][29]*OM229+s->In[6][29]*OM329)+OM229*(s->In[1][29]*OM129+s->In[2][29]*OM229+
 s->In[3][29]*OM329));
  FB129_1 = s->m[29]*AlM128_1;
  FB229_1 = s->m[29]*AlM229_1;
  FB329_1 = s->m[29]*AlM329_1;
  FM129_1 = FB129_1+FM130_1*C30-FM230_1*S30;
  FM229_1 = FB229_1+FM130_1*S30+FM230_1*C30;
  FM329_1 = FB329_1+FB330_1-FB131_1*S31+FB331_1*C31;
  CM129_1 = CM130_1*C30-CM230_1*S30-FB229_1*s->l[3][29]+FB329_1*s->l[2][29]-s->dpt[3][58]*(FM130_1*S30+FM230_1*C30);
  CM229_1 = CM130_1*S30+CM230_1*C30+FB129_1*s->l[3][29]-FB329_1*s->l[1][29]+s->dpt[3][58]*(FM130_1*C30-FM230_1*S30);
  CM329_1 = CM330_1-FB129_1*s->l[2][29]+FB229_1*s->l[1][29];
  FB129_2 = s->m[29]*AlM128_2;
  FB229_2 = s->m[29]*AlM229_2;
  FB329_2 = s->m[29]*AlM329_2;
  FM129_2 = FB129_2+FM130_2*C30-FM230_2*S30;
  FM229_2 = FB229_2+FM130_2*S30+FM230_2*C30;
  FM329_2 = FB329_2+FB330_2-FB131_2*S31+FB331_2*C31;
  CM129_2 = CM130_2*C30-CM230_2*S30-FB229_2*s->l[3][29]+FB329_2*s->l[2][29]-s->dpt[3][58]*(FM130_2*S30+FM230_2*C30);
  CM229_2 = CM130_2*S30+CM230_2*C30+FB129_2*s->l[3][29]-FB329_2*s->l[1][29]+s->dpt[3][58]*(FM130_2*C30-FM230_2*S30);
  CM329_2 = CM330_2-FB129_2*s->l[2][29]+FB229_2*s->l[1][29];
  FB129_3 = s->m[29]*AlM128_3;
  FB229_3 = s->m[29]*AlM229_3;
  FB329_3 = s->m[29]*AlM329_3;
  FM129_3 = FB129_3+FM130_3*C30-FM230_3*S30;
  FM229_3 = FB229_3+FM130_3*S30+FM230_3*C30;
  FM329_3 = FB329_3+FB330_3-FB131_3*S31+FB331_3*C31;
  CM129_3 = CM130_3*C30-CM230_3*S30-FB229_3*s->l[3][29]+FB329_3*s->l[2][29]-s->dpt[3][58]*(FM130_3*S30+FM230_3*C30);
  CM229_3 = CM130_3*S30+CM230_3*C30+FB129_3*s->l[3][29]-FB329_3*s->l[1][29]+s->dpt[3][58]*(FM130_3*C30-FM230_3*S30);
  CM329_3 = CM330_3-FB129_3*s->l[2][29]+FB229_3*s->l[1][29];
  FB129_4 = s->m[29]*(AlM129_4+OpM229_4*s->l[3][29]-OpM329_4*s->l[2][29]);
  FB229_4 = s->m[29]*(AlM229_4-OpM128_4*s->l[3][29]+OpM329_4*s->l[1][29]);
  FB329_4 = s->m[29]*(AlM329_4+OpM128_4*s->l[2][29]-OpM229_4*s->l[1][29]);
  FM129_4 = FB129_4+FM130_4*C30-FM230_4*S30;
  FM229_4 = FB229_4+FM130_4*S30+FM230_4*C30;
  FM329_4 = FB329_4+FB330_4-FB131_4*S31+FB331_4*C31;
  CM129_4 = s->In[1][29]*OpM128_4+s->In[2][29]*OpM229_4+s->In[3][29]*OpM329_4+CM130_4*C30-CM230_4*S30-FB229_4*
 s->l[3][29]+FB329_4*s->l[2][29]-s->dpt[3][58]*(FM130_4*S30+FM230_4*C30);
  CM229_4 = s->In[2][29]*OpM128_4+s->In[5][29]*OpM229_4+s->In[6][29]*OpM329_4+CM130_4*S30+CM230_4*C30+FB129_4*
 s->l[3][29]-FB329_4*s->l[1][29]+s->dpt[3][58]*(FM130_4*C30-FM230_4*S30);
  CM329_4 = CM330_4+s->In[3][29]*OpM128_4+s->In[6][29]*OpM229_4+s->In[9][29]*OpM329_4-FB129_4*s->l[2][29]+FB229_4*
 s->l[1][29];
  FB129_5 = s->m[29]*(AlM129_5+OpM229_5*s->l[3][29]-OpM329_5*s->l[2][29]);
  FB229_5 = s->m[29]*(AlM229_5-OpM128_5*s->l[3][29]+OpM329_5*s->l[1][29]);
  FB329_5 = s->m[29]*(AlM329_5+OpM128_5*s->l[2][29]-OpM229_5*s->l[1][29]);
  FM129_5 = FB129_5+FM130_5*C30-FM230_5*S30;
  FM229_5 = FB229_5+FM130_5*S30+FM230_5*C30;
  FM329_5 = FB329_5+FB330_5-FB131_5*S31+FB331_5*C31;
  CM129_5 = s->In[1][29]*OpM128_5+s->In[2][29]*OpM229_5+s->In[3][29]*OpM329_5+CM130_5*C30-CM230_5*S30-FB229_5*
 s->l[3][29]+FB329_5*s->l[2][29]-s->dpt[3][58]*(FM130_5*S30+FM230_5*C30);
  CM229_5 = s->In[2][29]*OpM128_5+s->In[5][29]*OpM229_5+s->In[6][29]*OpM329_5+CM130_5*S30+CM230_5*C30+FB129_5*
 s->l[3][29]-FB329_5*s->l[1][29]+s->dpt[3][58]*(FM130_5*C30-FM230_5*S30);
  CM329_5 = CM330_5+s->In[3][29]*OpM128_5+s->In[6][29]*OpM229_5+s->In[9][29]*OpM329_5-FB129_5*s->l[2][29]+FB229_5*
 s->l[1][29];
  FB129_6 = s->m[29]*(AlM129_6+OpM229_6*s->l[3][29]-OpM329_6*s->l[2][29]);
  FB229_6 = s->m[29]*(AlM229_6-OpM128_6*s->l[3][29]+OpM329_6*s->l[1][29]);
  FB329_6 = s->m[29]*(AlM329_6+OpM128_6*s->l[2][29]-OpM229_6*s->l[1][29]);
  FM129_6 = FB129_6+FM130_6*C30-FM230_6*S30;
  FM229_6 = FB229_6+FM130_6*S30+FM230_6*C30;
  FM329_6 = FB329_6+FB330_6-FB131_6*S31+FB331_6*C31;
  CM129_6 = s->In[1][29]*OpM128_6+s->In[2][29]*OpM229_6+s->In[3][29]*OpM329_6+CM130_6*C30-CM230_6*S30-FB229_6*
 s->l[3][29]+FB329_6*s->l[2][29]-s->dpt[3][58]*(FM130_6*S30+FM230_6*C30);
  CM229_6 = s->In[2][29]*OpM128_6+s->In[5][29]*OpM229_6+s->In[6][29]*OpM329_6+CM130_6*S30+CM230_6*C30+FB129_6*
 s->l[3][29]-FB329_6*s->l[1][29]+s->dpt[3][58]*(FM130_6*C30-FM230_6*S30);
  CM329_6 = CM330_6+s->In[3][29]*OpM128_6+s->In[6][29]*OpM229_6+s->In[9][29]*OpM329_6-FB129_6*s->l[2][29]+FB229_6*
 s->l[1][29];
  FB129_21 = s->m[29]*(AlM129_21+OpM229_21*s->l[3][29]-OpM329_21*s->l[2][29]);
  FB229_21 = s->m[29]*(AlM229_21-OpM128_21*s->l[3][29]+OpM329_21*s->l[1][29]);
  FB329_21 = s->m[29]*(AlM329_21+OpM128_21*s->l[2][29]-OpM229_21*s->l[1][29]);
  FM129_21 = FB129_21+FM130_21*C30-FM230_21*S30;
  FM229_21 = FB229_21+FM130_21*S30+FM230_21*C30;
  FM329_21 = FB329_21+FB330_21-FB131_21*S31+FB331_21*C31;
  CM129_21 = s->In[1][29]*OpM128_21+s->In[2][29]*OpM229_21+s->In[3][29]*OpM329_21+CM130_21*C30-CM230_21*S30-FB229_21*
 s->l[3][29]+FB329_21*s->l[2][29]-s->dpt[3][58]*(FM130_21*S30+FM230_21*C30);
  CM229_21 = s->In[2][29]*OpM128_21+s->In[5][29]*OpM229_21+s->In[6][29]*OpM329_21+CM130_21*S30+CM230_21*C30+FB129_21*
 s->l[3][29]-FB329_21*s->l[1][29]+s->dpt[3][58]*(FM130_21*C30-FM230_21*S30);
  CM329_21 = CM330_21+s->In[3][29]*OpM128_21+s->In[6][29]*OpM229_21+s->In[9][29]*OpM329_21-FB129_21*s->l[2][29]+FB229_21
 *s->l[1][29];
  FB129_22 = s->m[29]*(AlM129_22+OpM229_22*s->l[3][29]-OpM329_22*s->l[2][29]);
  FB229_22 = s->m[29]*(AlM229_22-OpM128_22*s->l[3][29]+OpM329_22*s->l[1][29]);
  FB329_22 = s->m[29]*(AlM329_22+OpM128_22*s->l[2][29]-OpM229_22*s->l[1][29]);
  FM129_22 = FB129_22+FM130_22*C30-FM230_22*S30;
  FM229_22 = FB229_22+FM130_22*S30+FM230_22*C30;
  FM329_22 = FB329_22+FB330_22-FB131_22*S31+FB331_22*C31;
  CM129_22 = s->In[1][29]*OpM128_22+s->In[2][29]*OpM229_22+s->In[3][29]*OpM329_22+CM130_22*C30-CM230_22*S30-FB229_22*
 s->l[3][29]+FB329_22*s->l[2][29]-s->dpt[3][58]*(FM130_22*S30+FM230_22*C30);
  CM229_22 = s->In[2][29]*OpM128_22+s->In[5][29]*OpM229_22+s->In[6][29]*OpM329_22+CM130_22*S30+CM230_22*C30+FB129_22*
 s->l[3][29]-FB329_22*s->l[1][29]+s->dpt[3][58]*(FM130_22*C30-FM230_22*S30);
  CM329_22 = CM330_22+s->In[3][29]*OpM128_22+s->In[6][29]*OpM229_22+s->In[9][29]*OpM329_22-FB129_22*s->l[2][29]+FB229_22
 *s->l[1][29];
  FB129_23 = s->m[29]*(AlM129_23+OpM229_23*s->l[3][29]-OpM329_23*s->l[2][29]);
  FB229_23 = s->m[29]*(AlM229_23+OpM329_23*s->l[1][29]+s->l[3][29]*S28);
  FB329_23 = s->m[29]*(AlM329_23-OpM229_23*s->l[1][29]-s->l[2][29]*S28);
  FM129_23 = FB129_23+FM130_23*C30-FM230_23*S30;
  FM229_23 = FB229_23+FM130_23*S30+FM230_23*C30;
  FM329_23 = FB329_23+FB330_23-FB131_23*S31+FB331_23*C31;
  CM129_23 = CM130_23*C30-CM230_23*S30-s->dpt[3][58]*(FM130_23*S30+FM230_23*C30)-s->In[1][29]*S28+s->In[2][29]*OpM229_23
 +s->In[3][29]*OpM329_23-FB229_23*s->l[3][29]+FB329_23*s->l[2][29];
  CM229_23 = CM130_23*S30+CM230_23*C30+s->dpt[3][58]*(FM130_23*C30-FM230_23*S30)-s->In[2][29]*S28+s->In[5][29]*OpM229_23
 +s->In[6][29]*OpM329_23+FB129_23*s->l[3][29]-FB329_23*s->l[1][29];
  CM329_23 = CM330_23-s->In[3][29]*S28+s->In[6][29]*OpM229_23+s->In[9][29]*OpM329_23-FB129_23*s->l[2][29]+FB229_23*
 s->l[1][29];
  FB129_28 = s->m[29]*(s->l[2][29]*S29+s->l[3][29]*C29);
  FB229_28 = -s->m[29]*s->l[1][29]*S29;
  FB329_28 = -s->m[29]*s->l[1][29]*C29;
  CM129_28 = s->In[2][29]*C29-s->In[3][29]*S29+CM130_28*C30-CM230_28*S30-FB229_28*s->l[3][29]+FB329_28*s->l[2][29]-
 s->dpt[3][58]*(FM130_28*S30+FM230_28*C30);
  CM129_29 = s->In[1][29]+s->m[29]*s->l[2][29]*s->l[2][29]+s->m[29]*s->l[3][29]*s->l[3][29]-s->dpt[3][58]*(C30*(FB230_29
 +FB231_29)+S30*(FB130_29+FB131_29*C31+FB331_29*S31))+C30*(s->In[1][30]*C30+CM131_29*C31+CM331_29*S31-FB230_29*s->l[3][30]-
 FB231_29*s->dpt[3][61]+FB330_29*s->l[2][30])-S30*(CM231_29-s->In[5][30]*S30+FB130_29*s->l[3][30]-FB330_29*s->l[1][30]+
 s->dpt[3][61]*(FB131_29*C31+FB331_29*S31));
  FA128 = -(s->frc[1][28]-s->m[28]*(AlF128+BeF228*s->l[2][28]-s->l[1][28]*(OM228*OM228+OM328*OM328)+s->l[3][28]*(BS328+
 OpF223)));
  FA228 = -(s->frc[2][28]-s->m[28]*(AlF228+BS528*s->l[2][28]+s->l[1][28]*(BS228+OpF328)+s->l[3][28]*(BS628-OpF128)));
  FA328 = -(s->frc[3][28]-s->m[28]*(AlF328+BeF828*s->l[2][28]+s->l[1][28]*(BS328-OpF223)-s->l[3][28]*(OM128*OM128+OM228*
 OM228)));
  FF128 = FA128+FF129;
  FF228 = FA228+FF229*C29-FF329*S29;
  FF328 = FA328+FF229*S29+FF329*C29;
  CF128 = -(s->trq[1][28]-CF129-s->In[1][28]*OpF128+FA228*s->l[3][28]-FA328*s->l[2][28]+OM228*OM328*(s->In[5][28]-
 s->In[9][28])-s->dpt[2][56]*(FF229*S29+FF329*C29));
  CF228 = -(s->trq[2][28]-s->In[5][28]*OpF223-CF229*C29+CF329*S29-FA128*s->l[3][28]+FA328*s->l[1][28]-OM128*OM328*(
 s->In[1][28]-s->In[9][28]));
  CF328 = -(s->trq[3][28]-s->In[9][28]*OpF328-CF229*S29-CF329*C29+FA128*s->l[2][28]-FA228*s->l[1][28]+FF129*
 s->dpt[2][56]+OM128*OM228*(s->In[1][28]-s->In[5][28]));
  FB128_1 = s->m[28]*AlM128_1;
  FB228_1 = s->m[28]*AlM223_1;
  FB328_1 = s->m[28]*AlM328_1;
  FM128_1 = FB128_1+FM129_1;
  FM228_1 = FB228_1+FM229_1*C29-FM329_1*S29;
  FM328_1 = FB328_1+FM229_1*S29+FM329_1*C29;
  CM128_1 = CM129_1-FB228_1*s->l[3][28]+FB328_1*s->l[2][28]+s->dpt[2][56]*(FM229_1*S29+FM329_1*C29);
  CM228_1 = CM229_1*C29-CM329_1*S29+FB128_1*s->l[3][28]-FB328_1*s->l[1][28];
  CM328_1 = CM229_1*S29+CM329_1*C29-FB128_1*s->l[2][28]+FB228_1*s->l[1][28]-FM129_1*s->dpt[2][56];
  FB128_2 = s->m[28]*AlM128_2;
  FB228_2 = s->m[28]*AlM223_2;
  FB328_2 = s->m[28]*AlM328_2;
  FM128_2 = FB128_2+FM129_2;
  FM228_2 = FB228_2+FM229_2*C29-FM329_2*S29;
  FM328_2 = FB328_2+FM229_2*S29+FM329_2*C29;
  CM128_2 = CM129_2-FB228_2*s->l[3][28]+FB328_2*s->l[2][28]+s->dpt[2][56]*(FM229_2*S29+FM329_2*C29);
  CM228_2 = CM229_2*C29-CM329_2*S29+FB128_2*s->l[3][28]-FB328_2*s->l[1][28];
  CM328_2 = CM229_2*S29+CM329_2*C29-FB128_2*s->l[2][28]+FB228_2*s->l[1][28]-FM129_2*s->dpt[2][56];
  FB128_3 = s->m[28]*AlM128_3;
  FB228_3 = s->m[28]*AlM223_3;
  FB328_3 = s->m[28]*AlM328_3;
  FM128_3 = FB128_3+FM129_3;
  FM228_3 = FB228_3+FM229_3*C29-FM329_3*S29;
  FM328_3 = FB328_3+FM229_3*S29+FM329_3*C29;
  CM128_3 = CM129_3-FB228_3*s->l[3][28]+FB328_3*s->l[2][28]+s->dpt[2][56]*(FM229_3*S29+FM329_3*C29);
  CM228_3 = CM229_3*C29-CM329_3*S29+FB128_3*s->l[3][28]-FB328_3*s->l[1][28];
  CM328_3 = CM229_3*S29+CM329_3*C29-FB128_3*s->l[2][28]+FB228_3*s->l[1][28]-FM129_3*s->dpt[2][56];
  FB128_4 = s->m[28]*(AlM128_4+OpM223_4*s->l[3][28]-OpM328_4*s->l[2][28]);
  FB228_4 = s->m[28]*(AlM228_4-OpM128_4*s->l[3][28]+OpM328_4*s->l[1][28]);
  FB328_4 = s->m[28]*(AlM328_4+OpM128_4*s->l[2][28]-OpM223_4*s->l[1][28]);
  FM128_4 = FB128_4+FM129_4;
  FM228_4 = FB228_4+FM229_4*C29-FM329_4*S29;
  FM328_4 = FB328_4+FM229_4*S29+FM329_4*C29;
  CM128_4 = CM129_4+s->In[1][28]*OpM128_4-FB228_4*s->l[3][28]+FB328_4*s->l[2][28]+s->dpt[2][56]*(FM229_4*S29+FM329_4*C29
 );
  CM228_4 = s->In[5][28]*OpM223_4+CM229_4*C29-CM329_4*S29+FB128_4*s->l[3][28]-FB328_4*s->l[1][28];
  CM328_4 = s->In[9][28]*OpM328_4+CM229_4*S29+CM329_4*C29-FB128_4*s->l[2][28]+FB228_4*s->l[1][28]-FM129_4*s->dpt[2][56];
  FB128_5 = s->m[28]*(AlM128_5+OpM223_5*s->l[3][28]-OpM328_5*s->l[2][28]);
  FB228_5 = s->m[28]*(AlM228_5-OpM128_5*s->l[3][28]+OpM328_5*s->l[1][28]);
  FB328_5 = s->m[28]*(AlM328_5+OpM128_5*s->l[2][28]-OpM223_5*s->l[1][28]);
  FM128_5 = FB128_5+FM129_5;
  FM228_5 = FB228_5+FM229_5*C29-FM329_5*S29;
  FM328_5 = FB328_5+FM229_5*S29+FM329_5*C29;
  CM128_5 = CM129_5+s->In[1][28]*OpM128_5-FB228_5*s->l[3][28]+FB328_5*s->l[2][28]+s->dpt[2][56]*(FM229_5*S29+FM329_5*C29
 );
  CM228_5 = s->In[5][28]*OpM223_5+CM229_5*C29-CM329_5*S29+FB128_5*s->l[3][28]-FB328_5*s->l[1][28];
  CM328_5 = s->In[9][28]*OpM328_5+CM229_5*S29+CM329_5*C29-FB128_5*s->l[2][28]+FB228_5*s->l[1][28]-FM129_5*s->dpt[2][56];
  FB128_6 = s->m[28]*(AlM128_6+OpM223_6*s->l[3][28]-OpM328_6*s->l[2][28]);
  FB228_6 = s->m[28]*(AlM228_6-OpM128_6*s->l[3][28]+OpM328_6*s->l[1][28]);
  FB328_6 = s->m[28]*(AlM328_6+OpM128_6*s->l[2][28]-OpM223_6*s->l[1][28]);
  FM128_6 = FB128_6+FM129_6;
  FM228_6 = FB228_6+FM229_6*C29-FM329_6*S29;
  FM328_6 = FB328_6+FM229_6*S29+FM329_6*C29;
  CM128_6 = CM129_6+s->In[1][28]*OpM128_6-FB228_6*s->l[3][28]+FB328_6*s->l[2][28]+s->dpt[2][56]*(FM229_6*S29+FM329_6*C29
 );
  CM228_6 = s->In[5][28]*OpM223_6+CM229_6*C29-CM329_6*S29+FB128_6*s->l[3][28]-FB328_6*s->l[1][28];
  CM328_6 = s->In[9][28]*OpM328_6+CM229_6*S29+CM329_6*C29-FB128_6*s->l[2][28]+FB228_6*s->l[1][28]-FM129_6*s->dpt[2][56];
  FB128_21 = s->m[28]*(AlM128_21+OpM223_21*s->l[3][28]-OpM328_21*s->l[2][28]);
  FB228_21 = s->m[28]*(AlM228_21-OpM128_21*s->l[3][28]+OpM328_21*s->l[1][28]);
  FB328_21 = s->m[28]*(AlM328_21+OpM128_21*s->l[2][28]-OpM223_21*s->l[1][28]);
  FM128_21 = FB128_21+FM129_21;
  FM228_21 = FB228_21+FM229_21*C29-FM329_21*S29;
  FM328_21 = FB328_21+FM229_21*S29+FM329_21*C29;
  CM128_21 = CM129_21+s->In[1][28]*OpM128_21-FB228_21*s->l[3][28]+FB328_21*s->l[2][28]+s->dpt[2][56]*(FM229_21*S29+
 FM329_21*C29);
  CM228_21 = s->In[5][28]*OpM223_21+CM229_21*C29-CM329_21*S29+FB128_21*s->l[3][28]-FB328_21*s->l[1][28];
  CM328_21 = s->In[9][28]*OpM328_21+CM229_21*S29+CM329_21*C29-FB128_21*s->l[2][28]+FB228_21*s->l[1][28]-FM129_21*
 s->dpt[2][56];
  FB128_22 = s->m[28]*(AlM128_22-OpM328_22*s->l[2][28]+s->l[3][28]*C23);
  FB228_22 = s->m[28]*(AlM228_22-OpM128_22*s->l[3][28]+OpM328_22*s->l[1][28]);
  FB328_22 = s->m[28]*(AlM328_22+OpM128_22*s->l[2][28]-s->l[1][28]*C23);
  FM128_22 = FB128_22+FM129_22;
  FM228_22 = FB228_22+FM229_22*C29-FM329_22*S29;
  FM328_22 = FB328_22+FM229_22*S29+FM329_22*C29;
  CM128_22 = CM129_22+s->In[1][28]*OpM128_22-FB228_22*s->l[3][28]+FB328_22*s->l[2][28]+s->dpt[2][56]*(FM229_22*S29+
 FM329_22*C29);
  CM228_22 = s->In[5][28]*C23+CM229_22*C29-CM329_22*S29+FB128_22*s->l[3][28]-FB328_22*s->l[1][28];
  CM328_22 = s->In[9][28]*OpM328_22+CM229_22*S29+CM329_22*C29-FB128_22*s->l[2][28]+FB228_22*s->l[1][28]-FM129_22*
 s->dpt[2][56];
  FB128_23 = s->m[28]*(AlM128_23-s->l[2][28]*C28);
  FB228_23 = s->m[28]*(s->dpt[1][45]+s->l[1][28]*C28+s->l[3][28]*S28);
  FB328_23 = -s->m[28]*S28*(s->dpt[2][45]+s->l[2][28]);
  CM228_23 = CM229_23*C29-CM329_23*S29+FB128_23*s->l[3][28]-FB328_23*s->l[1][28];
  CM228_28 = s->In[5][28]+s->m[28]*s->l[1][28]*s->l[1][28]+s->m[28]*s->l[3][28]*s->l[3][28]+C29*(s->In[5][29]*C29-
 s->In[6][29]*S29+CM130_28*S30+CM230_28*C30+FB129_28*s->l[3][29]-FB329_28*s->l[1][29]+s->dpt[3][58]*(FM130_28*C30-FM230_28*
 S30))-S29*(CM330_28+s->In[6][29]*C29-s->In[9][29]*S29-FB129_28*s->l[2][29]+FB229_28*s->l[1][29]);

// = = Block_0_2_0_2_0_2 = = 
 
// Backward Dynamics 

  FA113 = -(s->frc[1][13]+s->m[13]*(s->l[1][13]*(OM213*OM213+OM313*OM313)-s->l[3][13]*(BS313+OpF211)-C13*(AlF112+BS112*
 s->dpt[1][20]+BeF312*s->dpt[3][20])+S13*(AlF312+BS912*s->dpt[3][20]+BeF712*s->dpt[1][20])));
  FA213 = -(s->frc[2][13]-s->m[13]*(AlF211+BeF412*s->dpt[1][20]+BeF612*s->dpt[3][20]+s->l[1][13]*(OpF313+OM113*OM213)-
 s->l[3][13]*(OpF113-OM213*OM313)));
  FA313 = -(s->frc[3][13]-s->m[13]*(s->l[1][13]*(BS313-OpF211)-s->l[3][13]*(OM113*OM113+OM213*OM213)+C13*(AlF312+BS912*
 s->dpt[3][20]+BeF712*s->dpt[1][20])+S13*(AlF112+BS112*s->dpt[1][20]+BeF312*s->dpt[3][20])));
  CF113 = -(s->trq[1][13]-s->In[1][13]*OpF113+FA213*s->l[3][13]+OM213*OM313*(s->In[5][13]-s->In[9][13]));
  CF213 = -(s->trq[2][13]-s->In[5][13]*OpF211-FA113*s->l[3][13]+FA313*s->l[1][13]-OM113*OM313*(s->In[1][13]-s->In[9][13]
 ));
  CF313 = -(s->trq[3][13]-s->In[9][13]*OpF313-FA213*s->l[1][13]+OM113*OM213*(s->In[1][13]-s->In[5][13]));
  FB113_1 = s->m[13]*(AlM112_1*C13-AlM312_1*S13);
  FB213_1 = s->m[13]*AlM211_1;
  FB313_1 = s->m[13]*(AlM112_1*S13+AlM312_1*C13);
  CM113_1 = -FB213_1*s->l[3][13];
  CM213_1 = FB113_1*s->l[3][13]-FB313_1*s->l[1][13];
  CM313_1 = FB213_1*s->l[1][13];
  FB113_2 = s->m[13]*(AlM112_2*C13-AlM312_2*S13);
  FB213_2 = s->m[13]*AlM211_2;
  FB313_2 = s->m[13]*(AlM112_2*S13+AlM312_2*C13);
  CM113_2 = -FB213_2*s->l[3][13];
  CM213_2 = FB113_2*s->l[3][13]-FB313_2*s->l[1][13];
  CM313_2 = FB213_2*s->l[1][13];
  FB113_3 = s->m[13]*(AlM112_3*C13-AlM312_3*S13);
  FB213_3 = s->m[13]*AlM211_3;
  FB313_3 = s->m[13]*(AlM112_3*S13+AlM312_3*C13);
  CM113_3 = -FB213_3*s->l[3][13];
  CM213_3 = FB113_3*s->l[3][13]-FB313_3*s->l[1][13];
  CM313_3 = FB213_3*s->l[1][13];
  FB113_4 = s->m[13]*(OpM211_4*s->l[3][13]+C13*(AlM112_4+OpM211_4*s->dpt[3][20])-S13*(AlM312_4-OpM211_4*s->dpt[1][20]));
  FB213_4 = s->m[13]*(AlM211_4-OpM112_4*s->dpt[3][20]-OpM113_4*s->l[3][13]+OpM312_4*s->dpt[1][20]+OpM313_4*s->l[1][13]);
  FB313_4 = s->m[13]*(C13*(AlM312_4-OpM211_4*s->dpt[1][20])+S13*(AlM112_4+OpM211_4*s->dpt[3][20])-OpM211_4*s->l[1][13]);
  CM113_4 = s->In[1][13]*OpM113_4-FB213_4*s->l[3][13];
  CM213_4 = s->In[5][13]*OpM211_4+FB113_4*s->l[3][13]-FB313_4*s->l[1][13];
  CM313_4 = s->In[9][13]*OpM313_4+FB213_4*s->l[1][13];
  FB113_5 = s->m[13]*(OpM211_5*s->l[3][13]+C13*(AlM112_5+OpM211_5*s->dpt[3][20])-S13*(AlM312_5-OpM211_5*s->dpt[1][20]));
  FB213_5 = s->m[13]*(AlM211_5-OpM112_5*s->dpt[3][20]-OpM113_5*s->l[3][13]+OpM312_5*s->dpt[1][20]+OpM313_5*s->l[1][13]);
  FB313_5 = s->m[13]*(C13*(AlM312_5-OpM211_5*s->dpt[1][20])+S13*(AlM112_5+OpM211_5*s->dpt[3][20])-OpM211_5*s->l[1][13]);
  CM113_5 = s->In[1][13]*OpM113_5-FB213_5*s->l[3][13];
  CM213_5 = s->In[5][13]*OpM211_5+FB113_5*s->l[3][13]-FB313_5*s->l[1][13];
  CM313_5 = s->In[9][13]*OpM313_5+FB213_5*s->l[1][13];
  FB113_6 = s->m[13]*(OpM211_6*s->l[3][13]+C13*(AlM112_6+OpM211_6*s->dpt[3][20])-S13*(AlM312_6-OpM211_6*s->dpt[1][20]));
  FB213_6 = s->m[13]*(AlM211_6-OpM112_6*s->dpt[3][20]-OpM113_6*s->l[3][13]+OpM312_6*s->dpt[1][20]+OpM313_6*s->l[1][13]);
  FB313_6 = s->m[13]*(C13*(AlM312_6-OpM211_6*s->dpt[1][20])+S13*(AlM112_6+OpM211_6*s->dpt[3][20])-OpM211_6*s->l[1][13]);
  CM113_6 = s->In[1][13]*OpM113_6-FB213_6*s->l[3][13];
  CM213_6 = s->In[5][13]*OpM211_6+FB113_6*s->l[3][13]-FB313_6*s->l[1][13];
  CM313_6 = s->In[9][13]*OpM313_6+FB213_6*s->l[1][13];
  FB113_7 = s->m[13]*(OpM211_7*s->l[3][13]+C13*(AlM112_7+OpM211_7*s->dpt[3][20])-S13*(AlM312_7-OpM211_7*s->dpt[1][20]));
  FB213_7 = s->m[13]*(AlM211_7-OpM112_7*s->dpt[3][20]-OpM113_7*s->l[3][13]+OpM312_7*s->dpt[1][20]+OpM313_7*s->l[1][13]);
  FB313_7 = s->m[13]*(C13*(AlM312_7-OpM211_7*s->dpt[1][20])+S13*(AlM112_7+OpM211_7*s->dpt[3][20])-OpM211_7*s->l[1][13]);
  CM113_7 = s->In[1][13]*OpM113_7-FB213_7*s->l[3][13];
  CM213_7 = s->In[5][13]*OpM211_7+FB113_7*s->l[3][13]-FB313_7*s->l[1][13];
  CM313_7 = s->In[9][13]*OpM313_7+FB213_7*s->l[1][13];
  FB113_8 = s->m[13]*(OpM211_8*s->l[3][13]+C13*(AlM112_8+OpM211_8*s->dpt[3][20])-S13*(AlM312_8-OpM211_8*s->dpt[1][20]));
  FB213_8 = s->m[13]*(AlM211_8-OpM112_8*s->dpt[3][20]-OpM113_8*s->l[3][13]+OpM312_8*s->dpt[1][20]+OpM313_8*s->l[1][13]);
  FB313_8 = s->m[13]*(C13*(AlM312_8-OpM211_8*s->dpt[1][20])+S13*(AlM112_8+OpM211_8*s->dpt[3][20])-OpM211_8*s->l[1][13]);
  CM113_8 = s->In[1][13]*OpM113_8-FB213_8*s->l[3][13];
  CM213_8 = s->In[5][13]*OpM211_8+FB113_8*s->l[3][13]-FB313_8*s->l[1][13];
  CM313_8 = s->In[9][13]*OpM313_8+FB213_8*s->l[1][13];
  FB113_9 = s->m[13]*(OpM211_9*s->l[3][13]+C13*(AlM112_9+OpM211_9*s->dpt[3][20])-S13*(AlM312_9-OpM211_9*s->dpt[1][20]));
  FB213_9 = s->m[13]*(AlM211_9-OpM112_9*s->dpt[3][20]-OpM113_9*s->l[3][13]+OpM312_9*s->dpt[1][20]+OpM313_9*s->l[1][13]);
  FB313_9 = s->m[13]*(C13*(AlM312_9-OpM211_9*s->dpt[1][20])+S13*(AlM112_9+OpM211_9*s->dpt[3][20])-OpM211_9*s->l[1][13]);
  CM113_9 = s->In[1][13]*OpM113_9-FB213_9*s->l[3][13];
  CM213_9 = s->In[5][13]*OpM211_9+FB113_9*s->l[3][13]-FB313_9*s->l[1][13];
  CM313_9 = s->In[9][13]*OpM313_9+FB213_9*s->l[1][13];
  FB113_10 = s->m[13]*(s->l[3][13]*C11+C13*(AlM112_10+s->dpt[3][20]*C11)-S13*(AlM312_10-s->dpt[1][20]*C11));
  FB213_10 = -s->m[13]*(OpM112_10*s->dpt[3][20]+OpM113_10*s->l[3][13]-OpM312_10*s->dpt[1][20]-OpM313_10*s->l[1][13]);
  FB313_10 = s->m[13]*(C13*(AlM312_10-s->dpt[1][20]*C11)+S13*(AlM112_10+s->dpt[3][20]*C11)-s->l[1][13]*C11);
  CM113_10 = s->In[1][13]*OpM113_10-FB213_10*s->l[3][13];
  CM213_10 = s->In[5][13]*C11+FB113_10*s->l[3][13]-FB313_10*s->l[1][13];
  CM313_10 = s->In[9][13]*OpM313_10+FB213_10*s->l[1][13];
  FB213_11 = s->m[13]*(s->dpt[1][20]*S12-s->dpt[3][20]*C12+s->l[1][13]*S12p13-s->l[3][13]*C12p13);
  CM113_11 = s->In[1][13]*C12p13-FB213_11*s->l[3][13];
  CM313_11 = s->In[9][13]*S12p13+FB213_11*s->l[1][13];
  FB113_12 = s->m[13]*(s->l[3][13]+s->dpt[1][20]*S13+s->dpt[3][20]*C13);
  FB313_12 = -s->m[13]*(s->l[1][13]+s->dpt[1][20]*C13-s->dpt[3][20]*S13);
  CM213_12 = s->In[5][13]+FB113_12*s->l[3][13]-FB313_12*s->l[1][13];
  CM213_13 = s->In[5][13]+s->m[13]*s->l[1][13]*s->l[1][13]+s->m[13]*s->l[3][13]*s->l[3][13];
  FA112 = -(s->frc[1][12]-s->m[12]*(AlF112+BS112*s->l[1][12]+BeF312*s->l[3][12]));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211+BeF412*s->l[1][12]+BeF612*s->l[3][12]));
  FA312 = -(s->frc[3][12]-s->m[12]*(AlF312+BS912*s->l[3][12]+BeF712*s->l[1][12]));
  FF112 = FA112+FA113*C13+FA313*S13;
  FF312 = FA312-FA113*S13+FA313*C13;
  CF112 = -(s->trq[1][12]-s->In[1][12]*OpF112-CF113*C13-CF313*S13+FA212*s->l[3][12]+FA213*s->dpt[3][20]+OM212*OM312*(
 s->In[5][12]-s->In[9][12]));
  CF212 = -(s->trq[2][12]-CF213-s->In[5][12]*OpF211-FA112*s->l[3][12]+FA312*s->l[1][12]-OM112*OM312*(s->In[1][12]-
 s->In[9][12])-s->dpt[1][20]*(FA113*S13-FA313*C13)-s->dpt[3][20]*(FA113*C13+FA313*S13));
  CF312 = -(s->trq[3][12]-s->In[9][12]*OpF312+CF113*S13-CF313*C13-FA212*s->l[1][12]-FA213*s->dpt[1][20]+OM112*OM212*(
 s->In[1][12]-s->In[5][12]));
  FB112_1 = s->m[12]*AlM112_1;
  FB212_1 = s->m[12]*AlM211_1;
  FB312_1 = s->m[12]*AlM312_1;
  FM112_1 = FB112_1+FB113_1*C13+FB313_1*S13;
  FM312_1 = FB312_1-FB113_1*S13+FB313_1*C13;
  CM112_1 = CM113_1*C13+CM313_1*S13-FB212_1*s->l[3][12]-FB213_1*s->dpt[3][20];
  CM212_1 = CM213_1+FB112_1*s->l[3][12]-FB312_1*s->l[1][12]+s->dpt[1][20]*(FB113_1*S13-FB313_1*C13)+s->dpt[3][20]*(
 FB113_1*C13+FB313_1*S13);
  CM312_1 = -(CM113_1*S13-CM313_1*C13-FB212_1*s->l[1][12]-FB213_1*s->dpt[1][20]);
  FB112_2 = s->m[12]*AlM112_2;
  FB212_2 = s->m[12]*AlM211_2;
  FB312_2 = s->m[12]*AlM312_2;
  FM112_2 = FB112_2+FB113_2*C13+FB313_2*S13;
  FM312_2 = FB312_2-FB113_2*S13+FB313_2*C13;
  CM112_2 = CM113_2*C13+CM313_2*S13-FB212_2*s->l[3][12]-FB213_2*s->dpt[3][20];
  CM212_2 = CM213_2+FB112_2*s->l[3][12]-FB312_2*s->l[1][12]+s->dpt[1][20]*(FB113_2*S13-FB313_2*C13)+s->dpt[3][20]*(
 FB113_2*C13+FB313_2*S13);
  CM312_2 = -(CM113_2*S13-CM313_2*C13-FB212_2*s->l[1][12]-FB213_2*s->dpt[1][20]);
  FB112_3 = s->m[12]*AlM112_3;
  FB212_3 = s->m[12]*AlM211_3;
  FB312_3 = s->m[12]*AlM312_3;
  FM112_3 = FB112_3+FB113_3*C13+FB313_3*S13;
  FM312_3 = FB312_3-FB113_3*S13+FB313_3*C13;
  CM112_3 = CM113_3*C13+CM313_3*S13-FB212_3*s->l[3][12]-FB213_3*s->dpt[3][20];
  CM212_3 = CM213_3+FB112_3*s->l[3][12]-FB312_3*s->l[1][12]+s->dpt[1][20]*(FB113_3*S13-FB313_3*C13)+s->dpt[3][20]*(
 FB113_3*C13+FB313_3*S13);
  CM312_3 = -(CM113_3*S13-CM313_3*C13-FB212_3*s->l[1][12]-FB213_3*s->dpt[1][20]);
  FB112_4 = s->m[12]*(AlM112_4+OpM211_4*s->l[3][12]);
  FB212_4 = s->m[12]*(AlM211_4-OpM112_4*s->l[3][12]+OpM312_4*s->l[1][12]);
  FB312_4 = s->m[12]*(AlM312_4-OpM211_4*s->l[1][12]);
  FM112_4 = FB112_4+FB113_4*C13+FB313_4*S13;
  FM312_4 = FB312_4-FB113_4*S13+FB313_4*C13;
  CM112_4 = s->In[1][12]*OpM112_4+CM113_4*C13+CM313_4*S13-FB212_4*s->l[3][12]-FB213_4*s->dpt[3][20];
  CM212_4 = CM213_4+s->In[5][12]*OpM211_4+FB112_4*s->l[3][12]-FB312_4*s->l[1][12]+s->dpt[1][20]*(FB113_4*S13-FB313_4*C13
 )+s->dpt[3][20]*(FB113_4*C13+FB313_4*S13);
  CM312_4 = s->In[9][12]*OpM312_4-CM113_4*S13+CM313_4*C13+FB212_4*s->l[1][12]+FB213_4*s->dpt[1][20];
  FB112_5 = s->m[12]*(AlM112_5+OpM211_5*s->l[3][12]);
  FB212_5 = s->m[12]*(AlM211_5-OpM112_5*s->l[3][12]+OpM312_5*s->l[1][12]);
  FB312_5 = s->m[12]*(AlM312_5-OpM211_5*s->l[1][12]);
  FM112_5 = FB112_5+FB113_5*C13+FB313_5*S13;
  FM312_5 = FB312_5-FB113_5*S13+FB313_5*C13;
  CM112_5 = s->In[1][12]*OpM112_5+CM113_5*C13+CM313_5*S13-FB212_5*s->l[3][12]-FB213_5*s->dpt[3][20];
  CM212_5 = CM213_5+s->In[5][12]*OpM211_5+FB112_5*s->l[3][12]-FB312_5*s->l[1][12]+s->dpt[1][20]*(FB113_5*S13-FB313_5*C13
 )+s->dpt[3][20]*(FB113_5*C13+FB313_5*S13);
  CM312_5 = s->In[9][12]*OpM312_5-CM113_5*S13+CM313_5*C13+FB212_5*s->l[1][12]+FB213_5*s->dpt[1][20];
  FB112_6 = s->m[12]*(AlM112_6+OpM211_6*s->l[3][12]);
  FB212_6 = s->m[12]*(AlM211_6-OpM112_6*s->l[3][12]+OpM312_6*s->l[1][12]);
  FB312_6 = s->m[12]*(AlM312_6-OpM211_6*s->l[1][12]);
  FM112_6 = FB112_6+FB113_6*C13+FB313_6*S13;
  FM312_6 = FB312_6-FB113_6*S13+FB313_6*C13;
  CM112_6 = s->In[1][12]*OpM112_6+CM113_6*C13+CM313_6*S13-FB212_6*s->l[3][12]-FB213_6*s->dpt[3][20];
  CM212_6 = CM213_6+s->In[5][12]*OpM211_6+FB112_6*s->l[3][12]-FB312_6*s->l[1][12]+s->dpt[1][20]*(FB113_6*S13-FB313_6*C13
 )+s->dpt[3][20]*(FB113_6*C13+FB313_6*S13);
  CM312_6 = s->In[9][12]*OpM312_6-CM113_6*S13+CM313_6*C13+FB212_6*s->l[1][12]+FB213_6*s->dpt[1][20];
  FB112_7 = s->m[12]*(AlM112_7+OpM211_7*s->l[3][12]);
  FB212_7 = s->m[12]*(AlM211_7-OpM112_7*s->l[3][12]+OpM312_7*s->l[1][12]);
  FB312_7 = s->m[12]*(AlM312_7-OpM211_7*s->l[1][12]);
  FM112_7 = FB112_7+FB113_7*C13+FB313_7*S13;
  FM312_7 = FB312_7-FB113_7*S13+FB313_7*C13;
  CM112_7 = s->In[1][12]*OpM112_7+CM113_7*C13+CM313_7*S13-FB212_7*s->l[3][12]-FB213_7*s->dpt[3][20];
  CM212_7 = CM213_7+s->In[5][12]*OpM211_7+FB112_7*s->l[3][12]-FB312_7*s->l[1][12]+s->dpt[1][20]*(FB113_7*S13-FB313_7*C13
 )+s->dpt[3][20]*(FB113_7*C13+FB313_7*S13);
  CM312_7 = s->In[9][12]*OpM312_7-CM113_7*S13+CM313_7*C13+FB212_7*s->l[1][12]+FB213_7*s->dpt[1][20];
  FB112_8 = s->m[12]*(AlM112_8+OpM211_8*s->l[3][12]);
  FB212_8 = s->m[12]*(AlM211_8-OpM112_8*s->l[3][12]+OpM312_8*s->l[1][12]);
  FB312_8 = s->m[12]*(AlM312_8-OpM211_8*s->l[1][12]);
  FM112_8 = FB112_8+FB113_8*C13+FB313_8*S13;
  FM312_8 = FB312_8-FB113_8*S13+FB313_8*C13;
  CM112_8 = s->In[1][12]*OpM112_8+CM113_8*C13+CM313_8*S13-FB212_8*s->l[3][12]-FB213_8*s->dpt[3][20];
  CM212_8 = CM213_8+s->In[5][12]*OpM211_8+FB112_8*s->l[3][12]-FB312_8*s->l[1][12]+s->dpt[1][20]*(FB113_8*S13-FB313_8*C13
 )+s->dpt[3][20]*(FB113_8*C13+FB313_8*S13);
  CM312_8 = s->In[9][12]*OpM312_8-CM113_8*S13+CM313_8*C13+FB212_8*s->l[1][12]+FB213_8*s->dpt[1][20];
  FB112_9 = s->m[12]*(AlM112_9+OpM211_9*s->l[3][12]);
  FB212_9 = s->m[12]*(AlM211_9-OpM112_9*s->l[3][12]+OpM312_9*s->l[1][12]);
  FB312_9 = s->m[12]*(AlM312_9-OpM211_9*s->l[1][12]);
  FM112_9 = FB112_9+FB113_9*C13+FB313_9*S13;
  FM312_9 = FB312_9-FB113_9*S13+FB313_9*C13;
  CM112_9 = s->In[1][12]*OpM112_9+CM113_9*C13+CM313_9*S13-FB212_9*s->l[3][12]-FB213_9*s->dpt[3][20];
  CM212_9 = CM213_9+s->In[5][12]*OpM211_9+FB112_9*s->l[3][12]-FB312_9*s->l[1][12]+s->dpt[1][20]*(FB113_9*S13-FB313_9*C13
 )+s->dpt[3][20]*(FB113_9*C13+FB313_9*S13);
  CM312_9 = s->In[9][12]*OpM312_9-CM113_9*S13+CM313_9*C13+FB212_9*s->l[1][12]+FB213_9*s->dpt[1][20];
  FB112_10 = s->m[12]*(AlM112_10+s->l[3][12]*C11);
  FB212_10 = -s->m[12]*(OpM112_10*s->l[3][12]-OpM312_10*s->l[1][12]);
  FB312_10 = s->m[12]*(AlM312_10-s->l[1][12]*C11);
  CM112_10 = s->In[1][12]*OpM112_10+CM113_10*C13+CM313_10*S13-FB212_10*s->l[3][12]-FB213_10*s->dpt[3][20];
  CM212_10 = CM213_10+s->In[5][12]*C11+FB112_10*s->l[3][12]-FB312_10*s->l[1][12]+s->dpt[1][20]*(FB113_10*S13-FB313_10*
 C13)+s->dpt[3][20]*(FB113_10*C13+FB313_10*S13);
  CM312_10 = s->In[9][12]*OpM312_10-CM113_10*S13+CM313_10*C13+FB212_10*s->l[1][12]+FB213_10*s->dpt[1][20];
  FB212_11 = s->m[12]*(s->l[1][12]*S12-s->l[3][12]*C12);
  CM212_12 = s->In[5][12]+CM213_12+s->m[12]*s->l[1][12]*s->l[1][12]+s->m[12]*s->l[3][12]*s->l[3][12]+s->dpt[1][20]*(
 FB113_12*S13-FB313_12*C13)+s->dpt[3][20]*(FB113_12*C13+FB313_12*S13);
  FA111 = -(s->frc[1][11]-s->m[11]*(AlF111-s->l[1][11]*(OM211*OM211+OM311*OM311)+s->l[2][11]*(BS211-OpF311)+s->l[3][11]*
 (BS311+OpF211)));
  FA211 = -(s->frc[2][11]-s->m[11]*(AlF211+s->l[1][11]*(BS211+OpF311)-s->l[2][11]*(OM111*OM111+OM311*OM311)+s->l[3][11]*
 (BS611-OpF110)));
  FA311 = -(s->frc[3][11]-s->m[11]*(AlF311+s->l[1][11]*(BS311-OpF211)+s->l[2][11]*(BS611+OpF110)-s->l[3][11]*(OM111*
 OM111+OM211*OM211)));
  FF111 = FA111+FF112*C12+FF312*S12;
  FF211 = FA211+FA212+FA213;
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
  FM211_1 = FB211_1+FB212_1+FB213_1;
  FM311_1 = FB311_1-FM112_1*S12+FM312_1*C12;
  CM111_1 = CM112_1*C12+CM312_1*S12-FB211_1*s->l[3][11]+FB311_1*s->l[2][11];
  CM211_1 = CM212_1+FB111_1*s->l[3][11]-FB311_1*s->l[1][11];
  CM311_1 = -(CM112_1*S12-CM312_1*C12+FB111_1*s->l[2][11]-FB211_1*s->l[1][11]);
  FB111_2 = s->m[11]*AlM110_2;
  FB211_2 = s->m[11]*AlM211_2;
  FB311_2 = s->m[11]*AlM311_2;
  FM111_2 = FB111_2+FM112_2*C12+FM312_2*S12;
  FM211_2 = FB211_2+FB212_2+FB213_2;
  FM311_2 = FB311_2-FM112_2*S12+FM312_2*C12;
  CM111_2 = CM112_2*C12+CM312_2*S12-FB211_2*s->l[3][11]+FB311_2*s->l[2][11];
  CM211_2 = CM212_2+FB111_2*s->l[3][11]-FB311_2*s->l[1][11];
  CM311_2 = -(CM112_2*S12-CM312_2*C12+FB111_2*s->l[2][11]-FB211_2*s->l[1][11]);
  FB111_3 = s->m[11]*AlM110_3;
  FB211_3 = s->m[11]*AlM211_3;
  FB311_3 = s->m[11]*AlM311_3;
  FM111_3 = FB111_3+FM112_3*C12+FM312_3*S12;
  FM211_3 = FB211_3+FB212_3+FB213_3;
  FM311_3 = FB311_3-FM112_3*S12+FM312_3*C12;
  CM111_3 = CM112_3*C12+CM312_3*S12-FB211_3*s->l[3][11]+FB311_3*s->l[2][11];
  CM211_3 = CM212_3+FB111_3*s->l[3][11]-FB311_3*s->l[1][11];
  CM311_3 = -(CM112_3*S12-CM312_3*C12+FB111_3*s->l[2][11]-FB211_3*s->l[1][11]);
  FB111_4 = s->m[11]*(AlM111_4+OpM211_4*s->l[3][11]-OpM311_4*s->l[2][11]);
  FB211_4 = s->m[11]*(AlM211_4-OpM110_4*s->l[3][11]+OpM311_4*s->l[1][11]);
  FB311_4 = s->m[11]*(AlM311_4+OpM110_4*s->l[2][11]-OpM211_4*s->l[1][11]);
  FM111_4 = FB111_4+FM112_4*C12+FM312_4*S12;
  FM211_4 = FB211_4+FB212_4+FB213_4;
  FM311_4 = FB311_4-FM112_4*S12+FM312_4*C12;
  CM111_4 = s->In[1][11]*OpM110_4+s->In[2][11]*OpM211_4+s->In[3][11]*OpM311_4+CM112_4*C12+CM312_4*S12-FB211_4*
 s->l[3][11]+FB311_4*s->l[2][11];
  CM211_4 = CM212_4+s->In[2][11]*OpM110_4+s->In[5][11]*OpM211_4+FB111_4*s->l[3][11]-FB311_4*s->l[1][11];
  CM311_4 = s->In[3][11]*OpM110_4+s->In[9][11]*OpM311_4-CM112_4*S12+CM312_4*C12-FB111_4*s->l[2][11]+FB211_4*s->l[1][11];
  FB111_5 = s->m[11]*(AlM111_5+OpM211_5*s->l[3][11]-OpM311_5*s->l[2][11]);
  FB211_5 = s->m[11]*(AlM211_5-OpM110_5*s->l[3][11]+OpM311_5*s->l[1][11]);
  FB311_5 = s->m[11]*(AlM311_5+OpM110_5*s->l[2][11]-OpM211_5*s->l[1][11]);
  FM111_5 = FB111_5+FM112_5*C12+FM312_5*S12;
  FM211_5 = FB211_5+FB212_5+FB213_5;
  FM311_5 = FB311_5-FM112_5*S12+FM312_5*C12;
  CM111_5 = s->In[1][11]*OpM110_5+s->In[2][11]*OpM211_5+s->In[3][11]*OpM311_5+CM112_5*C12+CM312_5*S12-FB211_5*
 s->l[3][11]+FB311_5*s->l[2][11];
  CM211_5 = CM212_5+s->In[2][11]*OpM110_5+s->In[5][11]*OpM211_5+FB111_5*s->l[3][11]-FB311_5*s->l[1][11];
  CM311_5 = s->In[3][11]*OpM110_5+s->In[9][11]*OpM311_5-CM112_5*S12+CM312_5*C12-FB111_5*s->l[2][11]+FB211_5*s->l[1][11];
  FB111_6 = s->m[11]*(AlM111_6+OpM211_6*s->l[3][11]-OpM311_6*s->l[2][11]);
  FB211_6 = s->m[11]*(AlM211_6-OpM110_6*s->l[3][11]+OpM311_6*s->l[1][11]);
  FB311_6 = s->m[11]*(AlM311_6+OpM110_6*s->l[2][11]-OpM211_6*s->l[1][11]);
  FM111_6 = FB111_6+FM112_6*C12+FM312_6*S12;
  FM211_6 = FB211_6+FB212_6+FB213_6;
  FM311_6 = FB311_6-FM112_6*S12+FM312_6*C12;
  CM111_6 = s->In[1][11]*OpM110_6+s->In[2][11]*OpM211_6+s->In[3][11]*OpM311_6+CM112_6*C12+CM312_6*S12-FB211_6*
 s->l[3][11]+FB311_6*s->l[2][11];
  CM211_6 = CM212_6+s->In[2][11]*OpM110_6+s->In[5][11]*OpM211_6+FB111_6*s->l[3][11]-FB311_6*s->l[1][11];
  CM311_6 = s->In[3][11]*OpM110_6+s->In[9][11]*OpM311_6-CM112_6*S12+CM312_6*C12-FB111_6*s->l[2][11]+FB211_6*s->l[1][11];
  FB111_7 = s->m[11]*(AlM111_7+OpM211_7*s->l[3][11]-OpM311_7*s->l[2][11]);
  FB211_7 = s->m[11]*(AlM211_7-OpM110_7*s->l[3][11]+OpM311_7*s->l[1][11]);
  FB311_7 = s->m[11]*(AlM311_7+OpM110_7*s->l[2][11]-OpM211_7*s->l[1][11]);
  FM111_7 = FB111_7+FM112_7*C12+FM312_7*S12;
  FM211_7 = FB211_7+FB212_7+FB213_7;
  FM311_7 = FB311_7-FM112_7*S12+FM312_7*C12;
  CM111_7 = s->In[1][11]*OpM110_7+s->In[2][11]*OpM211_7+s->In[3][11]*OpM311_7+CM112_7*C12+CM312_7*S12-FB211_7*
 s->l[3][11]+FB311_7*s->l[2][11];
  CM211_7 = CM212_7+s->In[2][11]*OpM110_7+s->In[5][11]*OpM211_7+FB111_7*s->l[3][11]-FB311_7*s->l[1][11];
  CM311_7 = s->In[3][11]*OpM110_7+s->In[9][11]*OpM311_7-CM112_7*S12+CM312_7*C12-FB111_7*s->l[2][11]+FB211_7*s->l[1][11];
  FB111_8 = s->m[11]*(AlM111_8+OpM211_8*s->l[3][11]-OpM311_8*s->l[2][11]);
  FB211_8 = s->m[11]*(AlM211_8-OpM110_8*s->l[3][11]+OpM311_8*s->l[1][11]);
  FB311_8 = s->m[11]*(AlM311_8+OpM110_8*s->l[2][11]-OpM211_8*s->l[1][11]);
  FM111_8 = FB111_8+FM112_8*C12+FM312_8*S12;
  FM211_8 = FB211_8+FB212_8+FB213_8;
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
  CM111_11 = s->In[1][11]+s->m[11]*s->l[2][11]*s->l[2][11]+s->m[11]*s->l[3][11]*s->l[3][11]+C12*(s->In[1][12]*C12+
 CM113_11*C13+CM313_11*S13-FB212_11*s->l[3][12]-FB213_11*s->dpt[3][20])+S12*(s->In[9][12]*S12-CM113_11*S13+CM313_11*C13+
 FB212_11*s->l[1][12]+FB213_11*s->dpt[1][20]);
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
 (FB112_10+FB113_10*C13+FB313_10*S13)+S12*(FB312_10-FB113_10*S13+FB313_10*C13))+C11*(CM212_10+s->In[5][11]*C11+FB111_10*
 s->l[3][11]-FB311_10*s->l[1][11])+S11*(s->In[9][11]*S11+CM112_10*S12-CM312_10*C12+FB111_10*s->l[2][11]-FB211_10*s->l[1][11]);
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
 s->l[3][10]+FB310_9*s->l[2][10]-s->dpt[3][14]*(C11*(FB211_9+FB212_9+FB213_9)-S11*(FB311_9-FM112_9*S12+FM312_9*C12)));
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

// = = Block_0_2_0_2_0_3 = = 
 
// Backward Dynamics 

  FA120 = -(s->frc[1][20]+s->m[20]*(s->l[1][20]*(OM220*OM220+OM320*OM320)-s->l[3][20]*(BS320+OpF218)-C20*(AlF119+BS119*
 s->dpt[1][36]+BeF319*s->dpt[3][36])+S20*(AlF319+BS919*s->dpt[3][36]+BeF719*s->dpt[1][36])));
  FA220 = -(s->frc[2][20]-s->m[20]*(AlF218+BeF419*s->dpt[1][36]+BeF619*s->dpt[3][36]+s->l[1][20]*(OpF320+OM120*OM220)-
 s->l[3][20]*(OpF120-OM220*OM320)));
  FA320 = -(s->frc[3][20]-s->m[20]*(s->l[1][20]*(BS320-OpF218)-s->l[3][20]*(OM120*OM120+OM220*OM220)+C20*(AlF319+BS919*
 s->dpt[3][36]+BeF719*s->dpt[1][36])+S20*(AlF119+BS119*s->dpt[1][36]+BeF319*s->dpt[3][36])));
  CF120 = -(s->trq[1][20]-s->In[1][20]*OpF120+FA220*s->l[3][20]+OM220*OM320*(s->In[5][20]-s->In[9][20]));
  CF220 = -(s->trq[2][20]-s->In[5][20]*OpF218-FA120*s->l[3][20]+FA320*s->l[1][20]-OM120*OM320*(s->In[1][20]-s->In[9][20]
 ));
  CF320 = -(s->trq[3][20]-s->In[9][20]*OpF320-FA220*s->l[1][20]+OM120*OM220*(s->In[1][20]-s->In[5][20]));
  FB120_1 = s->m[20]*(AlM119_1*C20-AlM319_1*S20);
  FB220_1 = s->m[20]*AlM218_1;
  FB320_1 = s->m[20]*(AlM119_1*S20+AlM319_1*C20);
  CM120_1 = -FB220_1*s->l[3][20];
  CM220_1 = FB120_1*s->l[3][20]-FB320_1*s->l[1][20];
  CM320_1 = FB220_1*s->l[1][20];
  FB120_2 = s->m[20]*(AlM119_2*C20-AlM319_2*S20);
  FB220_2 = s->m[20]*AlM218_2;
  FB320_2 = s->m[20]*(AlM119_2*S20+AlM319_2*C20);
  CM120_2 = -FB220_2*s->l[3][20];
  CM220_2 = FB120_2*s->l[3][20]-FB320_2*s->l[1][20];
  CM320_2 = FB220_2*s->l[1][20];
  FB120_3 = s->m[20]*(AlM119_3*C20-AlM319_3*S20);
  FB220_3 = s->m[20]*AlM218_3;
  FB320_3 = s->m[20]*(AlM119_3*S20+AlM319_3*C20);
  CM120_3 = -FB220_3*s->l[3][20];
  CM220_3 = FB120_3*s->l[3][20]-FB320_3*s->l[1][20];
  CM320_3 = FB220_3*s->l[1][20];
  FB120_4 = s->m[20]*(OpM218_4*s->l[3][20]+C20*(AlM119_4+OpM218_4*s->dpt[3][36])-S20*(AlM319_4-OpM218_4*s->dpt[1][36]));
  FB220_4 = s->m[20]*(AlM218_4-OpM119_4*s->dpt[3][36]-OpM120_4*s->l[3][20]+OpM319_4*s->dpt[1][36]+OpM320_4*s->l[1][20]);
  FB320_4 = s->m[20]*(C20*(AlM319_4-OpM218_4*s->dpt[1][36])+S20*(AlM119_4+OpM218_4*s->dpt[3][36])-OpM218_4*s->l[1][20]);
  CM120_4 = s->In[1][20]*OpM120_4-FB220_4*s->l[3][20];
  CM220_4 = s->In[5][20]*OpM218_4+FB120_4*s->l[3][20]-FB320_4*s->l[1][20];
  CM320_4 = s->In[9][20]*OpM320_4+FB220_4*s->l[1][20];
  FB120_5 = s->m[20]*(OpM218_5*s->l[3][20]+C20*(AlM119_5+OpM218_5*s->dpt[3][36])-S20*(AlM319_5-OpM218_5*s->dpt[1][36]));
  FB220_5 = s->m[20]*(AlM218_5-OpM119_5*s->dpt[3][36]-OpM120_5*s->l[3][20]+OpM319_5*s->dpt[1][36]+OpM320_5*s->l[1][20]);
  FB320_5 = s->m[20]*(C20*(AlM319_5-OpM218_5*s->dpt[1][36])+S20*(AlM119_5+OpM218_5*s->dpt[3][36])-OpM218_5*s->l[1][20]);
  CM120_5 = s->In[1][20]*OpM120_5-FB220_5*s->l[3][20];
  CM220_5 = s->In[5][20]*OpM218_5+FB120_5*s->l[3][20]-FB320_5*s->l[1][20];
  CM320_5 = s->In[9][20]*OpM320_5+FB220_5*s->l[1][20];
  FB120_6 = s->m[20]*(OpM218_6*s->l[3][20]+C20*(AlM119_6+OpM218_6*s->dpt[3][36])-S20*(AlM319_6-OpM218_6*s->dpt[1][36]));
  FB220_6 = s->m[20]*(AlM218_6-OpM119_6*s->dpt[3][36]-OpM120_6*s->l[3][20]+OpM319_6*s->dpt[1][36]+OpM320_6*s->l[1][20]);
  FB320_6 = s->m[20]*(C20*(AlM319_6-OpM218_6*s->dpt[1][36])+S20*(AlM119_6+OpM218_6*s->dpt[3][36])-OpM218_6*s->l[1][20]);
  CM120_6 = s->In[1][20]*OpM120_6-FB220_6*s->l[3][20];
  CM220_6 = s->In[5][20]*OpM218_6+FB120_6*s->l[3][20]-FB320_6*s->l[1][20];
  CM320_6 = s->In[9][20]*OpM320_6+FB220_6*s->l[1][20];
  FB120_14 = s->m[20]*(OpM218_14*s->l[3][20]+C20*(AlM119_14+OpM218_14*s->dpt[3][36])-S20*(AlM319_14-OpM218_14*
 s->dpt[1][36]));
  FB220_14 = s->m[20]*(AlM218_14-OpM119_14*s->dpt[3][36]-OpM120_14*s->l[3][20]+OpM319_14*s->dpt[1][36]+OpM320_14*
 s->l[1][20]);
  FB320_14 = s->m[20]*(C20*(AlM319_14-OpM218_14*s->dpt[1][36])+S20*(AlM119_14+OpM218_14*s->dpt[3][36])-OpM218_14*
 s->l[1][20]);
  CM120_14 = s->In[1][20]*OpM120_14-FB220_14*s->l[3][20];
  CM220_14 = s->In[5][20]*OpM218_14+FB120_14*s->l[3][20]-FB320_14*s->l[1][20];
  CM320_14 = s->In[9][20]*OpM320_14+FB220_14*s->l[1][20];
  FB120_15 = s->m[20]*(OpM218_15*s->l[3][20]+C20*(AlM119_15+OpM218_15*s->dpt[3][36])-S20*(AlM319_15-OpM218_15*
 s->dpt[1][36]));
  FB220_15 = s->m[20]*(AlM218_15-OpM119_15*s->dpt[3][36]-OpM120_15*s->l[3][20]+OpM319_15*s->dpt[1][36]+OpM320_15*
 s->l[1][20]);
  FB320_15 = s->m[20]*(C20*(AlM319_15-OpM218_15*s->dpt[1][36])+S20*(AlM119_15+OpM218_15*s->dpt[3][36])-OpM218_15*
 s->l[1][20]);
  CM120_15 = s->In[1][20]*OpM120_15-FB220_15*s->l[3][20];
  CM220_15 = s->In[5][20]*OpM218_15+FB120_15*s->l[3][20]-FB320_15*s->l[1][20];
  CM320_15 = s->In[9][20]*OpM320_15+FB220_15*s->l[1][20];
  FB120_16 = s->m[20]*(OpM218_16*s->l[3][20]+C20*(AlM119_16+OpM218_16*s->dpt[3][36])-S20*(AlM319_16-OpM218_16*
 s->dpt[1][36]));
  FB220_16 = s->m[20]*(AlM218_16-OpM119_16*s->dpt[3][36]-OpM120_16*s->l[3][20]+OpM319_16*s->dpt[1][36]+OpM320_16*
 s->l[1][20]);
  FB320_16 = s->m[20]*(C20*(AlM319_16-OpM218_16*s->dpt[1][36])+S20*(AlM119_16+OpM218_16*s->dpt[3][36])-OpM218_16*
 s->l[1][20]);
  CM120_16 = s->In[1][20]*OpM120_16-FB220_16*s->l[3][20];
  CM220_16 = s->In[5][20]*OpM218_16+FB120_16*s->l[3][20]-FB320_16*s->l[1][20];
  CM320_16 = s->In[9][20]*OpM320_16+FB220_16*s->l[1][20];
  FB120_17 = s->m[20]*(s->l[3][20]*C18+C20*(AlM119_17+s->dpt[3][36]*C18)-S20*(AlM319_17-s->dpt[1][36]*C18));
  FB220_17 = -s->m[20]*(OpM119_17*s->dpt[3][36]+OpM120_17*s->l[3][20]-OpM319_17*s->dpt[1][36]-OpM320_17*s->l[1][20]);
  FB320_17 = s->m[20]*(C20*(AlM319_17-s->dpt[1][36]*C18)+S20*(AlM119_17+s->dpt[3][36]*C18)-s->l[1][20]*C18);
  CM120_17 = s->In[1][20]*OpM120_17-FB220_17*s->l[3][20];
  CM220_17 = s->In[5][20]*C18+FB120_17*s->l[3][20]-FB320_17*s->l[1][20];
  CM320_17 = s->In[9][20]*OpM320_17+FB220_17*s->l[1][20];
  FB220_18 = s->m[20]*(s->dpt[1][36]*S19-s->dpt[3][36]*C19+s->l[1][20]*S19p20-s->l[3][20]*C19p20);
  CM120_18 = s->In[1][20]*C19p20-FB220_18*s->l[3][20];
  CM320_18 = s->In[9][20]*S19p20+FB220_18*s->l[1][20];
  FB120_19 = s->m[20]*(s->l[3][20]+s->dpt[1][36]*S20+s->dpt[3][36]*C20);
  FB320_19 = -s->m[20]*(s->l[1][20]+s->dpt[1][36]*C20-s->dpt[3][36]*S20);
  CM220_19 = s->In[5][20]+FB120_19*s->l[3][20]-FB320_19*s->l[1][20];
  CM220_20 = s->In[5][20]+s->m[20]*s->l[1][20]*s->l[1][20]+s->m[20]*s->l[3][20]*s->l[3][20];
  FA119 = -(s->frc[1][19]-s->m[19]*(AlF119+BS119*s->l[1][19]+BeF319*s->l[3][19]));
  FA219 = -(s->frc[2][19]-s->m[19]*(AlF218+BeF419*s->l[1][19]+BeF619*s->l[3][19]));
  FA319 = -(s->frc[3][19]-s->m[19]*(AlF319+BS919*s->l[3][19]+BeF719*s->l[1][19]));
  FF119 = FA119+FA120*C20+FA320*S20;
  FF319 = FA319-FA120*S20+FA320*C20;
  CF119 = -(s->trq[1][19]-s->In[1][19]*OpF119-CF120*C20-CF320*S20+FA219*s->l[3][19]+FA220*s->dpt[3][36]+OM219*OM319*(
 s->In[5][19]-s->In[9][19]));
  CF219 = -(s->trq[2][19]-CF220-s->In[5][19]*OpF218-FA119*s->l[3][19]+FA319*s->l[1][19]-OM119*OM319*(s->In[1][19]-
 s->In[9][19])-s->dpt[1][36]*(FA120*S20-FA320*C20)-s->dpt[3][36]*(FA120*C20+FA320*S20));
  CF319 = -(s->trq[3][19]-s->In[9][19]*OpF319+CF120*S20-CF320*C20-FA219*s->l[1][19]-FA220*s->dpt[1][36]+OM119*OM219*(
 s->In[1][19]-s->In[5][19]));
  FB119_1 = s->m[19]*AlM119_1;
  FB219_1 = s->m[19]*AlM218_1;
  FB319_1 = s->m[19]*AlM319_1;
  FM119_1 = FB119_1+FB120_1*C20+FB320_1*S20;
  FM319_1 = FB319_1-FB120_1*S20+FB320_1*C20;
  CM119_1 = CM120_1*C20+CM320_1*S20-FB219_1*s->l[3][19]-FB220_1*s->dpt[3][36];
  CM219_1 = CM220_1+FB119_1*s->l[3][19]-FB319_1*s->l[1][19]+s->dpt[1][36]*(FB120_1*S20-FB320_1*C20)+s->dpt[3][36]*(
 FB120_1*C20+FB320_1*S20);
  CM319_1 = -(CM120_1*S20-CM320_1*C20-FB219_1*s->l[1][19]-FB220_1*s->dpt[1][36]);
  FB119_2 = s->m[19]*AlM119_2;
  FB219_2 = s->m[19]*AlM218_2;
  FB319_2 = s->m[19]*AlM319_2;
  FM119_2 = FB119_2+FB120_2*C20+FB320_2*S20;
  FM319_2 = FB319_2-FB120_2*S20+FB320_2*C20;
  CM119_2 = CM120_2*C20+CM320_2*S20-FB219_2*s->l[3][19]-FB220_2*s->dpt[3][36];
  CM219_2 = CM220_2+FB119_2*s->l[3][19]-FB319_2*s->l[1][19]+s->dpt[1][36]*(FB120_2*S20-FB320_2*C20)+s->dpt[3][36]*(
 FB120_2*C20+FB320_2*S20);
  CM319_2 = -(CM120_2*S20-CM320_2*C20-FB219_2*s->l[1][19]-FB220_2*s->dpt[1][36]);
  FB119_3 = s->m[19]*AlM119_3;
  FB219_3 = s->m[19]*AlM218_3;
  FB319_3 = s->m[19]*AlM319_3;
  FM119_3 = FB119_3+FB120_3*C20+FB320_3*S20;
  FM319_3 = FB319_3-FB120_3*S20+FB320_3*C20;
  CM119_3 = CM120_3*C20+CM320_3*S20-FB219_3*s->l[3][19]-FB220_3*s->dpt[3][36];
  CM219_3 = CM220_3+FB119_3*s->l[3][19]-FB319_3*s->l[1][19]+s->dpt[1][36]*(FB120_3*S20-FB320_3*C20)+s->dpt[3][36]*(
 FB120_3*C20+FB320_3*S20);
  CM319_3 = -(CM120_3*S20-CM320_3*C20-FB219_3*s->l[1][19]-FB220_3*s->dpt[1][36]);
  FB119_4 = s->m[19]*(AlM119_4+OpM218_4*s->l[3][19]);
  FB219_4 = s->m[19]*(AlM218_4-OpM119_4*s->l[3][19]+OpM319_4*s->l[1][19]);
  FB319_4 = s->m[19]*(AlM319_4-OpM218_4*s->l[1][19]);
  FM119_4 = FB119_4+FB120_4*C20+FB320_4*S20;
  FM319_4 = FB319_4-FB120_4*S20+FB320_4*C20;
  CM119_4 = s->In[1][19]*OpM119_4+CM120_4*C20+CM320_4*S20-FB219_4*s->l[3][19]-FB220_4*s->dpt[3][36];
  CM219_4 = CM220_4+s->In[5][19]*OpM218_4+FB119_4*s->l[3][19]-FB319_4*s->l[1][19]+s->dpt[1][36]*(FB120_4*S20-FB320_4*C20
 )+s->dpt[3][36]*(FB120_4*C20+FB320_4*S20);
  CM319_4 = s->In[9][19]*OpM319_4-CM120_4*S20+CM320_4*C20+FB219_4*s->l[1][19]+FB220_4*s->dpt[1][36];
  FB119_5 = s->m[19]*(AlM119_5+OpM218_5*s->l[3][19]);
  FB219_5 = s->m[19]*(AlM218_5-OpM119_5*s->l[3][19]+OpM319_5*s->l[1][19]);
  FB319_5 = s->m[19]*(AlM319_5-OpM218_5*s->l[1][19]);
  FM119_5 = FB119_5+FB120_5*C20+FB320_5*S20;
  FM319_5 = FB319_5-FB120_5*S20+FB320_5*C20;
  CM119_5 = s->In[1][19]*OpM119_5+CM120_5*C20+CM320_5*S20-FB219_5*s->l[3][19]-FB220_5*s->dpt[3][36];
  CM219_5 = CM220_5+s->In[5][19]*OpM218_5+FB119_5*s->l[3][19]-FB319_5*s->l[1][19]+s->dpt[1][36]*(FB120_5*S20-FB320_5*C20
 )+s->dpt[3][36]*(FB120_5*C20+FB320_5*S20);
  CM319_5 = s->In[9][19]*OpM319_5-CM120_5*S20+CM320_5*C20+FB219_5*s->l[1][19]+FB220_5*s->dpt[1][36];
  FB119_6 = s->m[19]*(AlM119_6+OpM218_6*s->l[3][19]);
  FB219_6 = s->m[19]*(AlM218_6-OpM119_6*s->l[3][19]+OpM319_6*s->l[1][19]);
  FB319_6 = s->m[19]*(AlM319_6-OpM218_6*s->l[1][19]);
  FM119_6 = FB119_6+FB120_6*C20+FB320_6*S20;
  FM319_6 = FB319_6-FB120_6*S20+FB320_6*C20;
  CM119_6 = s->In[1][19]*OpM119_6+CM120_6*C20+CM320_6*S20-FB219_6*s->l[3][19]-FB220_6*s->dpt[3][36];
  CM219_6 = CM220_6+s->In[5][19]*OpM218_6+FB119_6*s->l[3][19]-FB319_6*s->l[1][19]+s->dpt[1][36]*(FB120_6*S20-FB320_6*C20
 )+s->dpt[3][36]*(FB120_6*C20+FB320_6*S20);
  CM319_6 = s->In[9][19]*OpM319_6-CM120_6*S20+CM320_6*C20+FB219_6*s->l[1][19]+FB220_6*s->dpt[1][36];
  FB119_14 = s->m[19]*(AlM119_14+OpM218_14*s->l[3][19]);
  FB219_14 = s->m[19]*(AlM218_14-OpM119_14*s->l[3][19]+OpM319_14*s->l[1][19]);
  FB319_14 = s->m[19]*(AlM319_14-OpM218_14*s->l[1][19]);
  FM119_14 = FB119_14+FB120_14*C20+FB320_14*S20;
  FM319_14 = FB319_14-FB120_14*S20+FB320_14*C20;
  CM119_14 = s->In[1][19]*OpM119_14+CM120_14*C20+CM320_14*S20-FB219_14*s->l[3][19]-FB220_14*s->dpt[3][36];
  CM219_14 = CM220_14+s->In[5][19]*OpM218_14+FB119_14*s->l[3][19]-FB319_14*s->l[1][19]+s->dpt[1][36]*(FB120_14*S20-
 FB320_14*C20)+s->dpt[3][36]*(FB120_14*C20+FB320_14*S20);
  CM319_14 = s->In[9][19]*OpM319_14-CM120_14*S20+CM320_14*C20+FB219_14*s->l[1][19]+FB220_14*s->dpt[1][36];
  FB119_15 = s->m[19]*(AlM119_15+OpM218_15*s->l[3][19]);
  FB219_15 = s->m[19]*(AlM218_15-OpM119_15*s->l[3][19]+OpM319_15*s->l[1][19]);
  FB319_15 = s->m[19]*(AlM319_15-OpM218_15*s->l[1][19]);
  FM119_15 = FB119_15+FB120_15*C20+FB320_15*S20;
  FM319_15 = FB319_15-FB120_15*S20+FB320_15*C20;
  CM119_15 = s->In[1][19]*OpM119_15+CM120_15*C20+CM320_15*S20-FB219_15*s->l[3][19]-FB220_15*s->dpt[3][36];
  CM219_15 = CM220_15+s->In[5][19]*OpM218_15+FB119_15*s->l[3][19]-FB319_15*s->l[1][19]+s->dpt[1][36]*(FB120_15*S20-
 FB320_15*C20)+s->dpt[3][36]*(FB120_15*C20+FB320_15*S20);
  CM319_15 = s->In[9][19]*OpM319_15-CM120_15*S20+CM320_15*C20+FB219_15*s->l[1][19]+FB220_15*s->dpt[1][36];
  FB119_16 = s->m[19]*(AlM119_16+OpM218_16*s->l[3][19]);
  FB219_16 = s->m[19]*(AlM218_16-OpM119_16*s->l[3][19]+OpM319_16*s->l[1][19]);
  FB319_16 = s->m[19]*(AlM319_16-OpM218_16*s->l[1][19]);
  FM119_16 = FB119_16+FB120_16*C20+FB320_16*S20;
  FM319_16 = FB319_16-FB120_16*S20+FB320_16*C20;
  CM119_16 = s->In[1][19]*OpM119_16+CM120_16*C20+CM320_16*S20-FB219_16*s->l[3][19]-FB220_16*s->dpt[3][36];
  CM219_16 = CM220_16+s->In[5][19]*OpM218_16+FB119_16*s->l[3][19]-FB319_16*s->l[1][19]+s->dpt[1][36]*(FB120_16*S20-
 FB320_16*C20)+s->dpt[3][36]*(FB120_16*C20+FB320_16*S20);
  CM319_16 = s->In[9][19]*OpM319_16-CM120_16*S20+CM320_16*C20+FB219_16*s->l[1][19]+FB220_16*s->dpt[1][36];
  FB119_17 = s->m[19]*(AlM119_17+s->l[3][19]*C18);
  FB219_17 = -s->m[19]*(OpM119_17*s->l[3][19]-OpM319_17*s->l[1][19]);
  FB319_17 = s->m[19]*(AlM319_17-s->l[1][19]*C18);
  CM119_17 = s->In[1][19]*OpM119_17+CM120_17*C20+CM320_17*S20-FB219_17*s->l[3][19]-FB220_17*s->dpt[3][36];
  CM219_17 = CM220_17+s->In[5][19]*C18+FB119_17*s->l[3][19]-FB319_17*s->l[1][19]+s->dpt[1][36]*(FB120_17*S20-FB320_17*
 C20)+s->dpt[3][36]*(FB120_17*C20+FB320_17*S20);
  CM319_17 = s->In[9][19]*OpM319_17-CM120_17*S20+CM320_17*C20+FB219_17*s->l[1][19]+FB220_17*s->dpt[1][36];
  FB219_18 = s->m[19]*(s->l[1][19]*S19-s->l[3][19]*C19);
  CM219_19 = s->In[5][19]+CM220_19+s->m[19]*s->l[1][19]*s->l[1][19]+s->m[19]*s->l[3][19]*s->l[3][19]+s->dpt[1][36]*(
 FB120_19*S20-FB320_19*C20)+s->dpt[3][36]*(FB120_19*C20+FB320_19*S20);
  FA118 = -(s->frc[1][18]-s->m[18]*(AlF118-s->l[1][18]*(OM218*OM218+OM318*OM318)+s->l[2][18]*(BS218-OpF318)+s->l[3][18]*
 (BS318+OpF218)));
  FA218 = -(s->frc[2][18]-s->m[18]*(AlF218+s->l[1][18]*(BS218+OpF318)-s->l[2][18]*(OM118*OM118+OM318*OM318)+s->l[3][18]*
 (BS618-OpF117)));
  FA318 = -(s->frc[3][18]-s->m[18]*(AlF318+s->l[1][18]*(BS318-OpF218)+s->l[2][18]*(BS618+OpF117)-s->l[3][18]*(OM118*
 OM118+OM218*OM218)));
  FF118 = FA118+FF119*C19+FF319*S19;
  FF218 = FA218+FA219+FA220;
  FF318 = FA318-FF119*S19+FF319*C19;
  CF118 = -(s->trq[1][18]-s->In[1][18]*OpF117-s->In[3][18]*OpF318-CF119*C19-CF319*S19+FA218*s->l[3][18]-FA318*
 s->l[2][18]-OM218*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)+OM318*(s->In[5][18]*OM218+s->In[6][18]*OM318));
  CF218 = -(s->trq[2][18]-CF219-s->In[5][18]*OpF218-s->In[6][18]*OpF318-FA118*s->l[3][18]+FA318*s->l[1][18]+OM118*(
 s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)-OM318*(s->In[1][18]*OM118+s->In[3][18]*OM318));
  CF318 = -(s->trq[3][18]-s->In[3][18]*OpF117-s->In[6][18]*OpF218-s->In[9][18]*OpF318+CF119*S19-CF319*C19+FA118*
 s->l[2][18]-FA218*s->l[1][18]-OM118*(s->In[5][18]*OM218+s->In[6][18]*OM318)+OM218*(s->In[1][18]*OM118+s->In[3][18]*OM318));
  FB118_1 = s->m[18]*AlM117_1;
  FB218_1 = s->m[18]*AlM218_1;
  FB318_1 = s->m[18]*AlM318_1;
  FM118_1 = FB118_1+FM119_1*C19+FM319_1*S19;
  FM218_1 = FB218_1+FB219_1+FB220_1;
  FM318_1 = FB318_1-FM119_1*S19+FM319_1*C19;
  CM118_1 = CM119_1*C19+CM319_1*S19-FB218_1*s->l[3][18]+FB318_1*s->l[2][18];
  CM218_1 = CM219_1+FB118_1*s->l[3][18]-FB318_1*s->l[1][18];
  CM318_1 = -(CM119_1*S19-CM319_1*C19+FB118_1*s->l[2][18]-FB218_1*s->l[1][18]);
  FB118_2 = s->m[18]*AlM117_2;
  FB218_2 = s->m[18]*AlM218_2;
  FB318_2 = s->m[18]*AlM318_2;
  FM118_2 = FB118_2+FM119_2*C19+FM319_2*S19;
  FM218_2 = FB218_2+FB219_2+FB220_2;
  FM318_2 = FB318_2-FM119_2*S19+FM319_2*C19;
  CM118_2 = CM119_2*C19+CM319_2*S19-FB218_2*s->l[3][18]+FB318_2*s->l[2][18];
  CM218_2 = CM219_2+FB118_2*s->l[3][18]-FB318_2*s->l[1][18];
  CM318_2 = -(CM119_2*S19-CM319_2*C19+FB118_2*s->l[2][18]-FB218_2*s->l[1][18]);
  FB118_3 = s->m[18]*AlM117_3;
  FB218_3 = s->m[18]*AlM218_3;
  FB318_3 = s->m[18]*AlM318_3;
  FM118_3 = FB118_3+FM119_3*C19+FM319_3*S19;
  FM218_3 = FB218_3+FB219_3+FB220_3;
  FM318_3 = FB318_3-FM119_3*S19+FM319_3*C19;
  CM118_3 = CM119_3*C19+CM319_3*S19-FB218_3*s->l[3][18]+FB318_3*s->l[2][18];
  CM218_3 = CM219_3+FB118_3*s->l[3][18]-FB318_3*s->l[1][18];
  CM318_3 = -(CM119_3*S19-CM319_3*C19+FB118_3*s->l[2][18]-FB218_3*s->l[1][18]);
  FB118_4 = s->m[18]*(AlM118_4+OpM218_4*s->l[3][18]-OpM318_4*s->l[2][18]);
  FB218_4 = s->m[18]*(AlM218_4-OpM117_4*s->l[3][18]+OpM318_4*s->l[1][18]);
  FB318_4 = s->m[18]*(AlM318_4+OpM117_4*s->l[2][18]-OpM218_4*s->l[1][18]);
  FM118_4 = FB118_4+FM119_4*C19+FM319_4*S19;
  FM218_4 = FB218_4+FB219_4+FB220_4;
  FM318_4 = FB318_4-FM119_4*S19+FM319_4*C19;
  CM118_4 = s->In[1][18]*OpM117_4+s->In[3][18]*OpM318_4+CM119_4*C19+CM319_4*S19-FB218_4*s->l[3][18]+FB318_4*s->l[2][18];
  CM218_4 = CM219_4+s->In[5][18]*OpM218_4+s->In[6][18]*OpM318_4+FB118_4*s->l[3][18]-FB318_4*s->l[1][18];
  CM318_4 = s->In[3][18]*OpM117_4+s->In[6][18]*OpM218_4+s->In[9][18]*OpM318_4-CM119_4*S19+CM319_4*C19-FB118_4*
 s->l[2][18]+FB218_4*s->l[1][18];
  FB118_5 = s->m[18]*(AlM118_5+OpM218_5*s->l[3][18]-OpM318_5*s->l[2][18]);
  FB218_5 = s->m[18]*(AlM218_5-OpM117_5*s->l[3][18]+OpM318_5*s->l[1][18]);
  FB318_5 = s->m[18]*(AlM318_5+OpM117_5*s->l[2][18]-OpM218_5*s->l[1][18]);
  FM118_5 = FB118_5+FM119_5*C19+FM319_5*S19;
  FM218_5 = FB218_5+FB219_5+FB220_5;
  FM318_5 = FB318_5-FM119_5*S19+FM319_5*C19;
  CM118_5 = s->In[1][18]*OpM117_5+s->In[3][18]*OpM318_5+CM119_5*C19+CM319_5*S19-FB218_5*s->l[3][18]+FB318_5*s->l[2][18];
  CM218_5 = CM219_5+s->In[5][18]*OpM218_5+s->In[6][18]*OpM318_5+FB118_5*s->l[3][18]-FB318_5*s->l[1][18];
  CM318_5 = s->In[3][18]*OpM117_5+s->In[6][18]*OpM218_5+s->In[9][18]*OpM318_5-CM119_5*S19+CM319_5*C19-FB118_5*
 s->l[2][18]+FB218_5*s->l[1][18];
  FB118_6 = s->m[18]*(AlM118_6+OpM218_6*s->l[3][18]-OpM318_6*s->l[2][18]);
  FB218_6 = s->m[18]*(AlM218_6-OpM117_6*s->l[3][18]+OpM318_6*s->l[1][18]);
  FB318_6 = s->m[18]*(AlM318_6+OpM117_6*s->l[2][18]-OpM218_6*s->l[1][18]);
  FM118_6 = FB118_6+FM119_6*C19+FM319_6*S19;
  FM218_6 = FB218_6+FB219_6+FB220_6;
  FM318_6 = FB318_6-FM119_6*S19+FM319_6*C19;
  CM118_6 = s->In[1][18]*OpM117_6+s->In[3][18]*OpM318_6+CM119_6*C19+CM319_6*S19-FB218_6*s->l[3][18]+FB318_6*s->l[2][18];
  CM218_6 = CM219_6+s->In[5][18]*OpM218_6+s->In[6][18]*OpM318_6+FB118_6*s->l[3][18]-FB318_6*s->l[1][18];
  CM318_6 = s->In[3][18]*OpM117_6+s->In[6][18]*OpM218_6+s->In[9][18]*OpM318_6-CM119_6*S19+CM319_6*C19-FB118_6*
 s->l[2][18]+FB218_6*s->l[1][18];
  FB118_14 = s->m[18]*(AlM118_14+OpM218_14*s->l[3][18]-OpM318_14*s->l[2][18]);
  FB218_14 = s->m[18]*(AlM218_14-OpM117_14*s->l[3][18]+OpM318_14*s->l[1][18]);
  FB318_14 = s->m[18]*(AlM318_14+OpM117_14*s->l[2][18]-OpM218_14*s->l[1][18]);
  FM118_14 = FB118_14+FM119_14*C19+FM319_14*S19;
  FM218_14 = FB218_14+FB219_14+FB220_14;
  FM318_14 = FB318_14-FM119_14*S19+FM319_14*C19;
  CM118_14 = s->In[1][18]*OpM117_14+s->In[3][18]*OpM318_14+CM119_14*C19+CM319_14*S19-FB218_14*s->l[3][18]+FB318_14*
 s->l[2][18];
  CM218_14 = CM219_14+s->In[5][18]*OpM218_14+s->In[6][18]*OpM318_14+FB118_14*s->l[3][18]-FB318_14*s->l[1][18];
  CM318_14 = s->In[3][18]*OpM117_14+s->In[6][18]*OpM218_14+s->In[9][18]*OpM318_14-CM119_14*S19+CM319_14*C19-FB118_14*
 s->l[2][18]+FB218_14*s->l[1][18];
  FB118_15 = s->m[18]*(AlM118_15+OpM218_15*s->l[3][18]-OpM318_15*s->l[2][18]);
  FB218_15 = s->m[18]*(AlM218_15-OpM117_15*s->l[3][18]+OpM318_15*s->l[1][18]);
  FB318_15 = s->m[18]*(AlM318_15+OpM117_15*s->l[2][18]-OpM218_15*s->l[1][18]);
  FM118_15 = FB118_15+FM119_15*C19+FM319_15*S19;
  FM218_15 = FB218_15+FB219_15+FB220_15;
  FM318_15 = FB318_15-FM119_15*S19+FM319_15*C19;
  CM118_15 = s->In[1][18]*OpM117_15+s->In[3][18]*OpM318_15+CM119_15*C19+CM319_15*S19-FB218_15*s->l[3][18]+FB318_15*
 s->l[2][18];
  CM218_15 = CM219_15+s->In[5][18]*OpM218_15+s->In[6][18]*OpM318_15+FB118_15*s->l[3][18]-FB318_15*s->l[1][18];
  CM318_15 = s->In[3][18]*OpM117_15+s->In[6][18]*OpM218_15+s->In[9][18]*OpM318_15-CM119_15*S19+CM319_15*C19-FB118_15*
 s->l[2][18]+FB218_15*s->l[1][18];
  FB118_16 = s->m[18]*(OpM218_16*s->l[3][18]-OpM318_16*s->l[2][18]);
  FB218_16 = s->m[18]*(AlM218_16+OpM318_16*s->l[1][18]+s->l[3][18]*S17);
  FB318_16 = s->m[18]*(AlM318_16-OpM218_16*s->l[1][18]-s->l[2][18]*S17);
  CM118_16 = CM119_16*C19+CM319_16*S19-s->In[1][18]*S17+s->In[3][18]*OpM318_16-FB218_16*s->l[3][18]+FB318_16*s->l[2][18];
  CM218_16 = CM219_16+s->In[5][18]*OpM218_16+s->In[6][18]*OpM318_16+FB118_16*s->l[3][18]-FB318_16*s->l[1][18];
  CM318_16 = -(s->In[3][18]*S17-s->In[6][18]*OpM218_16-s->In[9][18]*OpM318_16+CM119_16*S19-CM319_16*C19+FB118_16*
 s->l[2][18]-FB218_16*s->l[1][18]);
  FB118_17 = s->m[18]*(s->dpt[3][30]+s->l[2][18]*S18+s->l[3][18]*C18);
  FB218_17 = -s->m[18]*s->l[1][18]*S18;
  FB318_17 = -s->m[18]*s->l[1][18]*C18;
  CM118_17 = CM119_17*C19+CM319_17*S19-s->In[3][18]*S18-FB218_17*s->l[3][18]+FB318_17*s->l[2][18];
  CM118_18 = s->In[1][18]+s->m[18]*s->l[2][18]*s->l[2][18]+s->m[18]*s->l[3][18]*s->l[3][18]+C19*(s->In[1][19]*C19+
 CM120_18*C20+CM320_18*S20-FB219_18*s->l[3][19]-FB220_18*s->dpt[3][36])+S19*(s->In[9][19]*S19-CM120_18*S20+CM320_18*C20+
 FB219_18*s->l[1][19]+FB220_18*s->dpt[1][36]);
  FA117 = -(s->frc[1][17]-s->m[17]*(AlF117+BeF317*s->l[3][17]-s->l[1][17]*(OM217*OM217+OM317*OM317)+s->l[2][17]*(BS217-
 OpF317)));
  FA217 = -(s->frc[2][17]-s->m[17]*(AlF217+BeF617*s->l[3][17]+s->l[1][17]*(BS217+OpF317)-s->l[2][17]*(OM117*OM117+OM317*
 OM317)));
  FA317 = -(s->frc[3][17]-s->m[17]*(AlF317+BS917*s->l[3][17]+s->l[1][17]*(BS317-OpF216)+s->l[2][17]*(BS617+OpF117)));
  FF117 = FA117+FF118;
  FF217 = FA217+FF218*C18-FF318*S18;
  FF317 = FA317+FF218*S18+FF318*C18;
  CF117 = -(s->trq[1][17]-CF118-s->In[1][17]*OpF117-s->In[3][17]*OpF317+FA217*s->l[3][17]-FA317*s->l[2][17]-OM217*(
 s->In[3][17]*OM117-s->In[5][17]*OM317+s->In[9][17]*OM317)+s->dpt[3][30]*(FF218*C18-FF318*S18));
  CF217 = -(s->trq[2][17]-s->In[5][17]*OpF216-CF218*C18+CF318*S18-FA117*s->l[3][17]+FA317*s->l[1][17]-FF118*
 s->dpt[3][30]+OM117*(s->In[3][17]*OM117+s->In[9][17]*OM317)-OM317*(s->In[1][17]*OM117+s->In[3][17]*OM317));
  CF317 = -(s->trq[3][17]-s->In[3][17]*OpF117-s->In[9][17]*OpF317-CF218*S18-CF318*C18+FA117*s->l[2][17]-FA217*
 s->l[1][17]+OM217*(s->In[1][17]*OM117+s->In[3][17]*OM317-s->In[5][17]*OM117));
  FB117_1 = s->m[17]*AlM117_1;
  FB217_1 = s->m[17]*AlM216_1;
  FB317_1 = s->m[17]*AlM317_1;
  FM117_1 = FB117_1+FM118_1;
  FM217_1 = FB217_1+FM218_1*C18-FM318_1*S18;
  FM317_1 = FB317_1+FM218_1*S18+FM318_1*C18;
  CM117_1 = CM118_1-FB217_1*s->l[3][17]+FB317_1*s->l[2][17]-s->dpt[3][30]*(FM218_1*C18-FM318_1*S18);
  CM217_1 = CM218_1*C18-CM318_1*S18+FB117_1*s->l[3][17]-FB317_1*s->l[1][17]+FM118_1*s->dpt[3][30];
  CM317_1 = CM218_1*S18+CM318_1*C18-FB117_1*s->l[2][17]+FB217_1*s->l[1][17];
  FB117_2 = s->m[17]*AlM117_2;
  FB217_2 = s->m[17]*AlM216_2;
  FB317_2 = s->m[17]*AlM317_2;
  FM117_2 = FB117_2+FM118_2;
  FM217_2 = FB217_2+FM218_2*C18-FM318_2*S18;
  FM317_2 = FB317_2+FM218_2*S18+FM318_2*C18;
  CM117_2 = CM118_2-FB217_2*s->l[3][17]+FB317_2*s->l[2][17]-s->dpt[3][30]*(FM218_2*C18-FM318_2*S18);
  CM217_2 = CM218_2*C18-CM318_2*S18+FB117_2*s->l[3][17]-FB317_2*s->l[1][17]+FM118_2*s->dpt[3][30];
  CM317_2 = CM218_2*S18+CM318_2*C18-FB117_2*s->l[2][17]+FB217_2*s->l[1][17];
  FB117_3 = s->m[17]*AlM117_3;
  FB217_3 = s->m[17]*AlM216_3;
  FB317_3 = s->m[17]*AlM317_3;
  FM117_3 = FB117_3+FM118_3;
  FM217_3 = FB217_3+FM218_3*C18-FM318_3*S18;
  FM317_3 = FB317_3+FM218_3*S18+FM318_3*C18;
  CM117_3 = CM118_3-FB217_3*s->l[3][17]+FB317_3*s->l[2][17]-s->dpt[3][30]*(FM218_3*C18-FM318_3*S18);
  CM217_3 = CM218_3*C18-CM318_3*S18+FB117_3*s->l[3][17]-FB317_3*s->l[1][17]+FM118_3*s->dpt[3][30];
  CM317_3 = CM218_3*S18+CM318_3*C18-FB117_3*s->l[2][17]+FB217_3*s->l[1][17];
  FB117_4 = s->m[17]*(AlM117_4+OpM216_4*s->l[3][17]-OpM317_4*s->l[2][17]);
  FB217_4 = s->m[17]*(AlM217_4-OpM117_4*s->l[3][17]+OpM317_4*s->l[1][17]);
  FB317_4 = s->m[17]*(AlM317_4+OpM117_4*s->l[2][17]-OpM216_4*s->l[1][17]);
  FM117_4 = FB117_4+FM118_4;
  FM217_4 = FB217_4+FM218_4*C18-FM318_4*S18;
  FM317_4 = FB317_4+FM218_4*S18+FM318_4*C18;
  CM117_4 = CM118_4+s->In[1][17]*OpM117_4+s->In[3][17]*OpM317_4-FB217_4*s->l[3][17]+FB317_4*s->l[2][17]-s->dpt[3][30]*(
 FM218_4*C18-FM318_4*S18);
  CM217_4 = s->In[5][17]*OpM216_4+CM218_4*C18-CM318_4*S18+FB117_4*s->l[3][17]-FB317_4*s->l[1][17]+FM118_4*s->dpt[3][30];
  CM317_4 = s->In[3][17]*OpM117_4+s->In[9][17]*OpM317_4+CM218_4*S18+CM318_4*C18-FB117_4*s->l[2][17]+FB217_4*s->l[1][17];
  FB117_5 = s->m[17]*(AlM117_5+OpM216_5*s->l[3][17]-OpM317_5*s->l[2][17]);
  FB217_5 = s->m[17]*(AlM217_5-OpM117_5*s->l[3][17]+OpM317_5*s->l[1][17]);
  FB317_5 = s->m[17]*(AlM317_5+OpM117_5*s->l[2][17]-OpM216_5*s->l[1][17]);
  FM117_5 = FB117_5+FM118_5;
  FM217_5 = FB217_5+FM218_5*C18-FM318_5*S18;
  FM317_5 = FB317_5+FM218_5*S18+FM318_5*C18;
  CM117_5 = CM118_5+s->In[1][17]*OpM117_5+s->In[3][17]*OpM317_5-FB217_5*s->l[3][17]+FB317_5*s->l[2][17]-s->dpt[3][30]*(
 FM218_5*C18-FM318_5*S18);
  CM217_5 = s->In[5][17]*OpM216_5+CM218_5*C18-CM318_5*S18+FB117_5*s->l[3][17]-FB317_5*s->l[1][17]+FM118_5*s->dpt[3][30];
  CM317_5 = s->In[3][17]*OpM117_5+s->In[9][17]*OpM317_5+CM218_5*S18+CM318_5*C18-FB117_5*s->l[2][17]+FB217_5*s->l[1][17];
  FB117_6 = s->m[17]*(AlM117_6+OpM216_6*s->l[3][17]-OpM317_6*s->l[2][17]);
  FB217_6 = s->m[17]*(AlM217_6-OpM117_6*s->l[3][17]+OpM317_6*s->l[1][17]);
  FB317_6 = s->m[17]*(AlM317_6+OpM117_6*s->l[2][17]-OpM216_6*s->l[1][17]);
  FM117_6 = FB117_6+FM118_6;
  FM217_6 = FB217_6+FM218_6*C18-FM318_6*S18;
  FM317_6 = FB317_6+FM218_6*S18+FM318_6*C18;
  CM117_6 = CM118_6+s->In[1][17]*OpM117_6+s->In[3][17]*OpM317_6-FB217_6*s->l[3][17]+FB317_6*s->l[2][17]-s->dpt[3][30]*(
 FM218_6*C18-FM318_6*S18);
  CM217_6 = s->In[5][17]*OpM216_6+CM218_6*C18-CM318_6*S18+FB117_6*s->l[3][17]-FB317_6*s->l[1][17]+FM118_6*s->dpt[3][30];
  CM317_6 = s->In[3][17]*OpM117_6+s->In[9][17]*OpM317_6+CM218_6*S18+CM318_6*C18-FB117_6*s->l[2][17]+FB217_6*s->l[1][17];
  FB117_14 = s->m[17]*(AlM117_14+OpM216_14*s->l[3][17]-OpM317_14*s->l[2][17]);
  FB217_14 = s->m[17]*(AlM217_14-OpM117_14*s->l[3][17]+OpM317_14*s->l[1][17]);
  FB317_14 = s->m[17]*(AlM317_14+OpM117_14*s->l[2][17]-OpM216_14*s->l[1][17]);
  FM117_14 = FB117_14+FM118_14;
  FM217_14 = FB217_14+FM218_14*C18-FM318_14*S18;
  FM317_14 = FB317_14+FM218_14*S18+FM318_14*C18;
  CM117_14 = CM118_14+s->In[1][17]*OpM117_14+s->In[3][17]*OpM317_14-FB217_14*s->l[3][17]+FB317_14*s->l[2][17]-
 s->dpt[3][30]*(FM218_14*C18-FM318_14*S18);
  CM217_14 = s->In[5][17]*OpM216_14+CM218_14*C18-CM318_14*S18+FB117_14*s->l[3][17]-FB317_14*s->l[1][17]+FM118_14*
 s->dpt[3][30];
  CM317_14 = s->In[3][17]*OpM117_14+s->In[9][17]*OpM317_14+CM218_14*S18+CM318_14*C18-FB117_14*s->l[2][17]+FB217_14*
 s->l[1][17];
  FB117_15 = s->m[17]*(AlM117_15-OpM317_15*s->l[2][17]-s->l[3][17]*S16);
  FB217_15 = s->m[17]*(AlM217_15-OpM117_15*s->l[3][17]+OpM317_15*s->l[1][17]);
  FB317_15 = s->m[17]*(AlM317_15+OpM117_15*s->l[2][17]+s->l[1][17]*S16);
  FM117_15 = FB117_15+FM118_15;
  FM217_15 = FB217_15+FM218_15*C18-FM318_15*S18;
  FM317_15 = FB317_15+FM218_15*S18+FM318_15*C18;
  CM117_15 = CM118_15+s->In[1][17]*OpM117_15+s->In[3][17]*OpM317_15-FB217_15*s->l[3][17]+FB317_15*s->l[2][17]-
 s->dpt[3][30]*(FM218_15*C18-FM318_15*S18);
  CM217_15 = CM218_15*C18-CM318_15*S18+FM118_15*s->dpt[3][30]-s->In[5][17]*S16+FB117_15*s->l[3][17]-FB317_15*s->l[1][17];
  CM317_15 = s->In[3][17]*OpM117_15+s->In[9][17]*OpM317_15+CM218_15*S18+CM318_15*C18-FB117_15*s->l[2][17]+FB217_15*
 s->l[1][17];
  FB117_16 = -s->m[17]*s->l[2][17]*C17;
  FB217_16 = s->m[17]*(s->l[1][17]*C17+s->l[3][17]*S17);
  FB317_16 = -s->m[17]*s->l[2][17]*S17;
  CM217_16 = CM218_16*C18-CM318_16*S18+FB117_16*s->l[3][17]-FB317_16*s->l[1][17]+s->dpt[3][30]*(FB118_16+FM119_16*C19+
 FM319_16*S19);
  CM217_17 = s->In[5][17]+s->m[17]*s->l[1][17]*s->l[1][17]+s->m[17]*s->l[3][17]*s->l[3][17]+s->dpt[3][30]*(FB118_17+C19*
 (FB119_17+FB120_17*C20+FB320_17*S20)+S19*(FB319_17-FB120_17*S20+FB320_17*C20))+C18*(CM219_17+s->In[5][18]*C18-s->In[6][18]*
 S18+FB118_17*s->l[3][18]-FB318_17*s->l[1][18])-S18*(s->In[6][18]*C18-s->In[9][18]*S18-CM119_17*S19+CM319_17*C19-FB118_17*
 s->l[2][18]+FB218_17*s->l[1][18]);
  FA116 = -(s->frc[1][16]-s->m[16]*(AlF116+BeF316*s->l[3][16]-s->l[1][16]*(OM216*OM216+OM316*OM316)+s->l[2][16]*(BS216-
 OpF315)));
  FA216 = -(s->frc[2][16]-s->m[16]*(AlF216+BeF616*s->l[3][16]+s->l[1][16]*(BS216+OpF315)-s->l[2][16]*(OM116*OM116+OM316*
 OM316)));
  FA316 = -(s->frc[3][16]-s->m[16]*(AlF316+BS916*s->l[3][16]+s->l[1][16]*(BS316-OpF216)+s->l[2][16]*(BS616+OpF116)));
  FF116 = FA116+FF117*C17+FF317*S17;
  FF216 = FA216+FF217;
  CF116 = -(s->trq[1][16]-s->In[1][16]*OpF116-s->In[3][16]*OpF315-CF117*C17-CF317*S17+FA216*s->l[3][16]-FA316*
 s->l[2][16]+FF217*s->dpt[3][27]-OM216*(s->In[3][16]*OM116-s->In[5][16]*OM316+s->In[9][16]*OM316));
  CF216 = -(s->trq[2][16]-CF217-s->In[5][16]*OpF216-FA116*s->l[3][16]+FA316*s->l[1][16]+OM116*(s->In[3][16]*OM116+
 s->In[9][16]*OM316)-OM316*(s->In[1][16]*OM116+s->In[3][16]*OM316)-s->dpt[3][27]*(FF117*C17+FF317*S17));
  CF316 = -(s->trq[3][16]-s->In[3][16]*OpF116-s->In[9][16]*OpF315+CF117*S17-CF317*C17+FA116*s->l[2][16]-FA216*
 s->l[1][16]+OM216*(s->In[1][16]*OM116+s->In[3][16]*OM316-s->In[5][16]*OM116));
  FB116_1 = s->m[16]*AlM116_1;
  FB216_1 = s->m[16]*AlM216_1;
  FB316_1 = s->m[16]*AlM315_1;
  FM116_1 = FB116_1+FM117_1*C17+FM317_1*S17;
  FM216_1 = FB216_1+FM217_1;
  CM116_1 = CM117_1*C17+CM317_1*S17-FB216_1*s->l[3][16]+FB316_1*s->l[2][16]-FM217_1*s->dpt[3][27];
  CM216_1 = CM217_1+FB116_1*s->l[3][16]-FB316_1*s->l[1][16]+s->dpt[3][27]*(FM117_1*C17+FM317_1*S17);
  CM316_1 = -(CM117_1*S17-CM317_1*C17+FB116_1*s->l[2][16]-FB216_1*s->l[1][16]);
  FB116_2 = s->m[16]*AlM116_2;
  FB216_2 = s->m[16]*AlM216_2;
  FB316_2 = s->m[16]*AlM315_2;
  FM116_2 = FB116_2+FM117_2*C17+FM317_2*S17;
  FM216_2 = FB216_2+FM217_2;
  CM116_2 = CM117_2*C17+CM317_2*S17-FB216_2*s->l[3][16]+FB316_2*s->l[2][16]-FM217_2*s->dpt[3][27];
  CM216_2 = CM217_2+FB116_2*s->l[3][16]-FB316_2*s->l[1][16]+s->dpt[3][27]*(FM117_2*C17+FM317_2*S17);
  CM316_2 = -(CM117_2*S17-CM317_2*C17+FB116_2*s->l[2][16]-FB216_2*s->l[1][16]);
  FB116_3 = s->m[16]*AlM116_3;
  FB216_3 = s->m[16]*AlM216_3;
  FB316_3 = s->m[16]*AlM315_3;
  FM116_3 = FB116_3+FM117_3*C17+FM317_3*S17;
  FM216_3 = FB216_3+FM217_3;
  CM116_3 = CM117_3*C17+CM317_3*S17-FB216_3*s->l[3][16]+FB316_3*s->l[2][16]-FM217_3*s->dpt[3][27];
  CM216_3 = CM217_3+FB116_3*s->l[3][16]-FB316_3*s->l[1][16]+s->dpt[3][27]*(FM117_3*C17+FM317_3*S17);
  CM316_3 = -(CM117_3*S17-CM317_3*C17+FB116_3*s->l[2][16]-FB216_3*s->l[1][16]);
  FB116_4 = s->m[16]*(AlM116_4+OpM216_4*s->l[3][16]-OpM315_4*s->l[2][16]);
  FB216_4 = s->m[16]*(AlM216_4-OpM116_4*s->l[3][16]+OpM315_4*s->l[1][16]);
  FB316_4 = s->m[16]*(AlM315_4+OpM116_4*s->l[2][16]-OpM216_4*s->l[1][16]);
  FM116_4 = FB116_4+FM117_4*C17+FM317_4*S17;
  FM216_4 = FB216_4+FM217_4;
  CM116_4 = s->In[1][16]*OpM116_4+s->In[3][16]*OpM315_4+CM117_4*C17+CM317_4*S17-FB216_4*s->l[3][16]+FB316_4*s->l[2][16]-
 FM217_4*s->dpt[3][27];
  CM216_4 = CM217_4+s->In[5][16]*OpM216_4+FB116_4*s->l[3][16]-FB316_4*s->l[1][16]+s->dpt[3][27]*(FM117_4*C17+FM317_4*S17
 );
  CM316_4 = s->In[3][16]*OpM116_4+s->In[9][16]*OpM315_4-CM117_4*S17+CM317_4*C17-FB116_4*s->l[2][16]+FB216_4*s->l[1][16];
  FB116_5 = s->m[16]*(AlM116_5+OpM216_5*s->l[3][16]-OpM315_5*s->l[2][16]);
  FB216_5 = s->m[16]*(AlM216_5-OpM116_5*s->l[3][16]+OpM315_5*s->l[1][16]);
  FB316_5 = s->m[16]*(AlM315_5+OpM116_5*s->l[2][16]-OpM216_5*s->l[1][16]);
  FM116_5 = FB116_5+FM117_5*C17+FM317_5*S17;
  FM216_5 = FB216_5+FM217_5;
  CM116_5 = s->In[1][16]*OpM116_5+s->In[3][16]*OpM315_5+CM117_5*C17+CM317_5*S17-FB216_5*s->l[3][16]+FB316_5*s->l[2][16]-
 FM217_5*s->dpt[3][27];
  CM216_5 = CM217_5+s->In[5][16]*OpM216_5+FB116_5*s->l[3][16]-FB316_5*s->l[1][16]+s->dpt[3][27]*(FM117_5*C17+FM317_5*S17
 );
  CM316_5 = s->In[3][16]*OpM116_5+s->In[9][16]*OpM315_5-CM117_5*S17+CM317_5*C17-FB116_5*s->l[2][16]+FB216_5*s->l[1][16];
  FB116_6 = s->m[16]*(AlM116_6+OpM216_6*s->l[3][16]-OpM315_6*s->l[2][16]);
  FB216_6 = s->m[16]*(AlM216_6-OpM116_6*s->l[3][16]+OpM315_6*s->l[1][16]);
  FB316_6 = s->m[16]*(AlM315_6+OpM116_6*s->l[2][16]-OpM216_6*s->l[1][16]);
  FM116_6 = FB116_6+FM117_6*C17+FM317_6*S17;
  FM216_6 = FB216_6+FM217_6;
  CM116_6 = s->In[1][16]*OpM116_6+s->In[3][16]*OpM315_6+CM117_6*C17+CM317_6*S17-FB216_6*s->l[3][16]+FB316_6*s->l[2][16]-
 FM217_6*s->dpt[3][27];
  CM216_6 = CM217_6+s->In[5][16]*OpM216_6+FB116_6*s->l[3][16]-FB316_6*s->l[1][16]+s->dpt[3][27]*(FM117_6*C17+FM317_6*S17
 );
  CM316_6 = s->In[3][16]*OpM116_6+s->In[9][16]*OpM315_6-CM117_6*S17+CM317_6*C17-FB116_6*s->l[2][16]+FB216_6*s->l[1][16];
  FB116_14 = s->m[16]*(AlM116_14+OpM216_14*s->l[3][16]+s->l[2][16]*S15);
  FB216_14 = s->m[16]*(AlM216_14-OpM116_14*s->l[3][16]-s->l[1][16]*S15);
  FB316_14 = s->m[16]*(OpM116_14*s->l[2][16]-OpM216_14*s->l[1][16]);
  FM116_14 = FB116_14+FM117_14*C17+FM317_14*S17;
  FM216_14 = FB216_14+FM217_14;
  CM116_14 = s->In[1][16]*OpM116_14-s->In[3][16]*S15+CM117_14*C17+CM317_14*S17-FB216_14*s->l[3][16]+FB316_14*s->l[2][16]
 -FM217_14*s->dpt[3][27];
  CM216_14 = CM217_14+s->In[5][16]*OpM216_14+FB116_14*s->l[3][16]-FB316_14*s->l[1][16]+s->dpt[3][27]*(FM117_14*C17+
 FM317_14*S17);
  CM316_14 = s->In[3][16]*OpM116_14-s->In[9][16]*S15-CM117_14*S17+CM317_14*C17-FB116_14*s->l[2][16]+FB216_14*s->l[1][16];
  FB116_15 = s->m[16]*(AlM116_15-s->l[3][16]*S16);
  FB216_15 = s->m[16]*(AlM216_15-s->l[3][16]*C16);
  FB316_15 = s->m[16]*(s->l[1][16]*S16+s->l[2][16]*C16);
  CM316_15 = s->In[3][16]*C16-CM117_15*S17+CM317_15*C17-FB116_15*s->l[2][16]+FB216_15*s->l[1][16];
  CM316_16 = s->In[9][16]+s->m[16]*s->l[1][16]*s->l[1][16]+s->m[16]*s->l[2][16]*s->l[2][16]-C17*(s->In[3][17]*S17-
 s->In[9][17]*C17-CM218_16*S18-CM318_16*C18+FB117_16*s->l[2][17]-FB217_16*s->l[1][17])-S17*(CM118_16-s->In[1][17]*S17+
 s->In[3][17]*C17-FB217_16*s->l[3][17]+FB317_16*s->l[2][17]-s->dpt[3][30]*(C18*(FB218_16+FB219_16+FB220_16)-S18*(FB318_16-
 FM119_16*S19+FM319_16*C19)));
  FA115 = -(s->frc[1][15]-s->m[15]*(AlF115+BeF315*s->l[3][15]-s->l[1][15]*(OM215*OM215+OM315*OM315)+s->l[2][15]*(BS215-
 OpF315)));
  FA215 = -(s->frc[2][15]-s->m[15]*(AlF215+BeF615*s->l[3][15]+s->l[1][15]*(BS215+OpF315)-s->l[2][15]*(OM115*OM115+OM315*
 OM315)));
  FA315 = -(s->frc[3][15]-s->m[15]*(AlF315+BS915*s->l[3][15]+s->l[1][15]*(BS315-OpF215)+s->l[2][15]*(BS615+OpF114)));
  FF115 = FA115+FF116*C16-FF216*S16;
  FF215 = FA215+FF116*S16+FF216*C16;
  FF315 = FA315+FA316-FF117*S17+FF317*C17;
  CF115 = -(s->trq[1][15]-s->In[1][15]*OpF114-s->In[2][15]*OpF215-CF116*C16+CF216*S16+FA215*s->l[3][15]-FA315*
 s->l[2][15]+OM315*(s->In[2][15]*OM115+s->In[5][15]*OM215-s->In[9][15]*OM215)+s->dpt[3][24]*(FF116*S16+FF216*C16));
  CF215 = -(s->trq[2][15]-s->In[2][15]*OpF114-s->In[5][15]*OpF215-CF116*S16-CF216*C16-FA115*s->l[3][15]+FA315*
 s->l[1][15]-OM315*(s->In[1][15]*OM115+s->In[2][15]*OM215-s->In[9][15]*OM115)-s->dpt[3][24]*(FF116*C16-FF216*S16));
  CF315 = -(s->trq[3][15]-CF316-s->In[9][15]*OpF315+FA115*s->l[2][15]-FA215*s->l[1][15]-OM115*(s->In[2][15]*OM115+
 s->In[5][15]*OM215)+OM215*(s->In[1][15]*OM115+s->In[2][15]*OM215));
  FB115_1 = s->m[15]*AlM114_1;
  FB215_1 = s->m[15]*AlM215_1;
  FB315_1 = s->m[15]*AlM315_1;
  FM115_1 = FB115_1+FM116_1*C16-FM216_1*S16;
  FM215_1 = FB215_1+FM116_1*S16+FM216_1*C16;
  FM315_1 = FB315_1+FB316_1-FM117_1*S17+FM317_1*C17;
  CM115_1 = CM116_1*C16-CM216_1*S16-FB215_1*s->l[3][15]+FB315_1*s->l[2][15]-s->dpt[3][24]*(FM116_1*S16+FM216_1*C16);
  CM215_1 = CM116_1*S16+CM216_1*C16+FB115_1*s->l[3][15]-FB315_1*s->l[1][15]+s->dpt[3][24]*(FM116_1*C16-FM216_1*S16);
  CM315_1 = CM316_1-FB115_1*s->l[2][15]+FB215_1*s->l[1][15];
  FB115_2 = s->m[15]*AlM114_2;
  FB215_2 = s->m[15]*AlM215_2;
  FB315_2 = s->m[15]*AlM315_2;
  FM115_2 = FB115_2+FM116_2*C16-FM216_2*S16;
  FM215_2 = FB215_2+FM116_2*S16+FM216_2*C16;
  FM315_2 = FB315_2+FB316_2-FM117_2*S17+FM317_2*C17;
  CM115_2 = CM116_2*C16-CM216_2*S16-FB215_2*s->l[3][15]+FB315_2*s->l[2][15]-s->dpt[3][24]*(FM116_2*S16+FM216_2*C16);
  CM215_2 = CM116_2*S16+CM216_2*C16+FB115_2*s->l[3][15]-FB315_2*s->l[1][15]+s->dpt[3][24]*(FM116_2*C16-FM216_2*S16);
  CM315_2 = CM316_2-FB115_2*s->l[2][15]+FB215_2*s->l[1][15];
  FB115_3 = s->m[15]*AlM114_3;
  FB215_3 = s->m[15]*AlM215_3;
  FB315_3 = s->m[15]*AlM315_3;
  FM115_3 = FB115_3+FM116_3*C16-FM216_3*S16;
  FM215_3 = FB215_3+FM116_3*S16+FM216_3*C16;
  FM315_3 = FB315_3+FB316_3-FM117_3*S17+FM317_3*C17;
  CM115_3 = CM116_3*C16-CM216_3*S16-FB215_3*s->l[3][15]+FB315_3*s->l[2][15]-s->dpt[3][24]*(FM116_3*S16+FM216_3*C16);
  CM215_3 = CM116_3*S16+CM216_3*C16+FB115_3*s->l[3][15]-FB315_3*s->l[1][15]+s->dpt[3][24]*(FM116_3*C16-FM216_3*S16);
  CM315_3 = CM316_3-FB115_3*s->l[2][15]+FB215_3*s->l[1][15];
  FB115_4 = s->m[15]*(AlM115_4+OpM215_4*s->l[3][15]-OpM315_4*s->l[2][15]);
  FB215_4 = s->m[15]*(AlM215_4-OpM114_4*s->l[3][15]+OpM315_4*s->l[1][15]);
  FB315_4 = s->m[15]*(AlM315_4+OpM114_4*s->l[2][15]-OpM215_4*s->l[1][15]);
  FM115_4 = FB115_4+FM116_4*C16-FM216_4*S16;
  FM215_4 = FB215_4+FM116_4*S16+FM216_4*C16;
  FM315_4 = FB315_4+FB316_4-FM117_4*S17+FM317_4*C17;
  CM115_4 = s->In[1][15]*OpM114_4+s->In[2][15]*OpM215_4+CM116_4*C16-CM216_4*S16-FB215_4*s->l[3][15]+FB315_4*s->l[2][15]-
 s->dpt[3][24]*(FM116_4*S16+FM216_4*C16);
  CM215_4 = s->In[2][15]*OpM114_4+s->In[5][15]*OpM215_4+CM116_4*S16+CM216_4*C16+FB115_4*s->l[3][15]-FB315_4*s->l[1][15]+
 s->dpt[3][24]*(FM116_4*C16-FM216_4*S16);
  CM315_4 = CM316_4+s->In[9][15]*OpM315_4-FB115_4*s->l[2][15]+FB215_4*s->l[1][15];
  FB115_5 = s->m[15]*(AlM115_5+OpM215_5*s->l[3][15]-OpM315_5*s->l[2][15]);
  FB215_5 = s->m[15]*(AlM215_5-OpM114_5*s->l[3][15]+OpM315_5*s->l[1][15]);
  FB315_5 = s->m[15]*(AlM315_5+OpM114_5*s->l[2][15]-OpM215_5*s->l[1][15]);
  FM115_5 = FB115_5+FM116_5*C16-FM216_5*S16;
  FM215_5 = FB215_5+FM116_5*S16+FM216_5*C16;
  FM315_5 = FB315_5+FB316_5-FM117_5*S17+FM317_5*C17;
  CM115_5 = s->In[1][15]*OpM114_5+s->In[2][15]*OpM215_5+CM116_5*C16-CM216_5*S16-FB215_5*s->l[3][15]+FB315_5*s->l[2][15]-
 s->dpt[3][24]*(FM116_5*S16+FM216_5*C16);
  CM215_5 = s->In[2][15]*OpM114_5+s->In[5][15]*OpM215_5+CM116_5*S16+CM216_5*C16+FB115_5*s->l[3][15]-FB315_5*s->l[1][15]+
 s->dpt[3][24]*(FM116_5*C16-FM216_5*S16);
  CM315_5 = CM316_5+s->In[9][15]*OpM315_5-FB115_5*s->l[2][15]+FB215_5*s->l[1][15];
  FB115_6 = s->m[15]*(AlM115_6+OpM215_6*s->l[3][15]-OpM315_6*s->l[2][15]);
  FB215_6 = s->m[15]*(AlM215_6+OpM315_6*s->l[1][15]+s->l[3][15]*S14);
  FB315_6 = s->m[15]*(AlM315_6-OpM215_6*s->l[1][15]-s->l[2][15]*S14);
  FM115_6 = FB115_6+FM116_6*C16-FM216_6*S16;
  FM215_6 = FB215_6+FM116_6*S16+FM216_6*C16;
  FM315_6 = FB315_6+FB316_6-FM117_6*S17+FM317_6*C17;
  CM115_6 = CM116_6*C16-CM216_6*S16-s->dpt[3][24]*(FM116_6*S16+FM216_6*C16)-s->In[1][15]*S14+s->In[2][15]*OpM215_6-
 FB215_6*s->l[3][15]+FB315_6*s->l[2][15];
  CM215_6 = CM116_6*S16+CM216_6*C16+s->dpt[3][24]*(FM116_6*C16-FM216_6*S16)-s->In[2][15]*S14+s->In[5][15]*OpM215_6+
 FB115_6*s->l[3][15]-FB315_6*s->l[1][15];
  CM315_6 = CM316_6+s->In[9][15]*OpM315_6-FB115_6*s->l[2][15]+FB215_6*s->l[1][15];
  FB115_14 = s->m[15]*(s->l[2][15]*S15+s->l[3][15]*C15);
  FB215_14 = -s->m[15]*s->l[1][15]*S15;
  FB315_14 = -s->m[15]*s->l[1][15]*C15;
  CM115_14 = s->In[2][15]*C15+CM116_14*C16-CM216_14*S16-FB215_14*s->l[3][15]+FB315_14*s->l[2][15]-s->dpt[3][24]*(
 FM116_14*S16+FM216_14*C16);
  CM115_15 = s->In[1][15]+s->m[15]*s->l[2][15]*s->l[2][15]+s->m[15]*s->l[3][15]*s->l[3][15]-s->dpt[3][24]*(C16*(FB216_15
 +FM217_15)+S16*(FB116_15+FM117_15*C17+FM317_15*S17))+C16*(s->In[1][16]*C16+CM117_15*C17+CM317_15*S17-FB216_15*s->l[3][16]+
 FB316_15*s->l[2][16]-FM217_15*s->dpt[3][27])-S16*(CM217_15-s->In[5][16]*S16+FB116_15*s->l[3][16]-FB316_15*s->l[1][16]+
 s->dpt[3][27]*(FM117_15*C17+FM317_15*S17));
  FA114 = -(s->frc[1][14]-s->m[14]*(AlF114+BeF214*s->l[2][14]-s->l[1][14]*(OM214*OM214+OM314*OM314)+s->l[3][14]*(BS314+
 OpF26)));
  FA214 = -(s->frc[2][14]-s->m[14]*(AlF214+BS514*s->l[2][14]+s->l[1][14]*(BS214+OpF314)+s->l[3][14]*(BS614-OpF114)));
  FA314 = -(s->frc[3][14]-s->m[14]*(AlF314+BeF814*s->l[2][14]+s->l[1][14]*(BS314-OpF26)-s->l[3][14]*(OM114*OM114+OM214*
 OM214)));
  FF114 = FA114+FF115;
  FF314 = FA314+FF215*S15+FF315*C15;
  CF114 = -(s->trq[1][14]-CF115-s->In[1][14]*OpF114-s->In[2][14]*OpF26+FA214*s->l[3][14]-FA314*s->l[2][14]-OM214*(
 s->In[6][14]*OM214+s->In[9][14]*OM314)+OM314*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)-s->dpt[2][22]*(FF215
 *S15+FF315*C15));
  CF214 = -(s->trq[2][14]-s->In[2][14]*OpF114-s->In[5][14]*OpF26-s->In[6][14]*OpF314-CF215*C15+CF315*S15-FA114*
 s->l[3][14]+FA314*s->l[1][14]+OM114*(s->In[6][14]*OM214+s->In[9][14]*OM314)-OM314*(s->In[1][14]*OM114+s->In[2][14]*OM214));
  CF314 = -(s->trq[3][14]-s->In[6][14]*OpF26-s->In[9][14]*OpF314-CF215*S15-CF315*C15+FA114*s->l[2][14]-FA214*s->l[1][14]
 +FF115*s->dpt[2][22]-OM114*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)+OM214*(s->In[1][14]*OM114+s->In[2][14]
 *OM214));
  FB114_1 = s->m[14]*AlM114_1;
  FB214_1 = s->m[14]*AlM26_1;
  FB314_1 = s->m[14]*AlM314_1;
  FM114_1 = FB114_1+FM115_1;
  FM314_1 = FB314_1+FM215_1*S15+FM315_1*C15;
  CM114_1 = CM115_1-FB214_1*s->l[3][14]+FB314_1*s->l[2][14]+s->dpt[2][22]*(FM215_1*S15+FM315_1*C15);
  CM214_1 = CM215_1*C15-CM315_1*S15+FB114_1*s->l[3][14]-FB314_1*s->l[1][14];
  CM314_1 = CM215_1*S15+CM315_1*C15-FB114_1*s->l[2][14]+FB214_1*s->l[1][14]-FM115_1*s->dpt[2][22];
  FB114_2 = s->m[14]*AlM114_2;
  FB214_2 = s->m[14]*AlM26_2;
  FB314_2 = s->m[14]*AlM314_2;
  FM114_2 = FB114_2+FM115_2;
  FM314_2 = FB314_2+FM215_2*S15+FM315_2*C15;
  CM114_2 = CM115_2-FB214_2*s->l[3][14]+FB314_2*s->l[2][14]+s->dpt[2][22]*(FM215_2*S15+FM315_2*C15);
  CM214_2 = CM215_2*C15-CM315_2*S15+FB114_2*s->l[3][14]-FB314_2*s->l[1][14];
  CM314_2 = CM215_2*S15+CM315_2*C15-FB114_2*s->l[2][14]+FB214_2*s->l[1][14]-FM115_2*s->dpt[2][22];
  FB114_3 = s->m[14]*AlM114_3;
  FB214_3 = s->m[14]*AlM26_3;
  FB314_3 = s->m[14]*AlM314_3;
  FM114_3 = FB114_3+FM115_3;
  FM314_3 = FB314_3+FM215_3*S15+FM315_3*C15;
  CM114_3 = CM115_3-FB214_3*s->l[3][14]+FB314_3*s->l[2][14]+s->dpt[2][22]*(FM215_3*S15+FM315_3*C15);
  CM214_3 = CM215_3*C15-CM315_3*S15+FB114_3*s->l[3][14]-FB314_3*s->l[1][14];
  CM314_3 = CM215_3*S15+CM315_3*C15-FB114_3*s->l[2][14]+FB214_3*s->l[1][14]-FM115_3*s->dpt[2][22];
  FB114_4 = s->m[14]*(AlM114_4+OpM26_4*s->l[3][14]-OpM314_4*s->l[2][14]);
  FB214_4 = -s->m[14]*(OpM114_4*s->l[3][14]-OpM314_4*s->l[1][14]);
  FB314_4 = s->m[14]*(AlM314_4+OpM114_4*s->l[2][14]-OpM26_4*s->l[1][14]);
  FM114_4 = FB114_4+FM115_4;
  FM314_4 = FB314_4+FM215_4*S15+FM315_4*C15;
  CM114_4 = CM115_4+s->In[1][14]*OpM114_4+s->In[2][14]*OpM26_4-FB214_4*s->l[3][14]+FB314_4*s->l[2][14]+s->dpt[2][22]*(
 FM215_4*S15+FM315_4*C15);
  CM214_4 = s->In[2][14]*OpM114_4+s->In[5][14]*OpM26_4+s->In[6][14]*OpM314_4+CM215_4*C15-CM315_4*S15+FB114_4*s->l[3][14]
 -FB314_4*s->l[1][14];
  CM314_4 = s->In[6][14]*OpM26_4+s->In[9][14]*OpM314_4+CM215_4*S15+CM315_4*C15-FB114_4*s->l[2][14]+FB214_4*s->l[1][14]-
 FM115_4*s->dpt[2][22];
  FB114_5 = s->m[14]*(AlM114_5-OpM314_5*s->l[2][14]+s->l[3][14]*C6);
  FB214_5 = -s->m[14]*(OpM114_5*s->l[3][14]-OpM314_5*s->l[1][14]);
  FB314_5 = s->m[14]*(AlM314_5+OpM114_5*s->l[2][14]-s->l[1][14]*C6);
  FM114_5 = FB114_5+FM115_5;
  FM314_5 = FB314_5+FM215_5*S15+FM315_5*C15;
  CM114_5 = CM115_5+s->In[1][14]*OpM114_5+s->In[2][14]*C6-FB214_5*s->l[3][14]+FB314_5*s->l[2][14]+s->dpt[2][22]*(FM215_5
 *S15+FM315_5*C15);
  CM214_5 = s->In[2][14]*OpM114_5+s->In[5][14]*C6+s->In[6][14]*OpM314_5+CM215_5*C15-CM315_5*S15+FB114_5*s->l[3][14]-
 FB314_5*s->l[1][14];
  CM314_5 = s->In[6][14]*C6+s->In[9][14]*OpM314_5+CM215_5*S15+CM315_5*C15-FB114_5*s->l[2][14]+FB214_5*s->l[1][14]-
 FM115_5*s->dpt[2][22];
  FB114_6 = s->m[14]*(AlM114_6-s->l[2][14]*C14);
  FB214_6 = s->m[14]*(s->l[1][14]*C14+s->l[3][14]*S14);
  FB314_6 = s->m[14]*(AlM314_6-s->l[2][14]*S14);
  CM214_6 = CM215_6*C15-CM315_6*S15-s->In[2][14]*S14+s->In[6][14]*C14+FB114_6*s->l[3][14]-FB314_6*s->l[1][14];
  CM214_14 = s->In[5][14]+s->m[14]*s->l[1][14]*s->l[1][14]+s->m[14]*s->l[3][14]*s->l[3][14]+C15*(s->In[5][15]*C15+
 CM116_14*S16+CM216_14*C16+FB115_14*s->l[3][15]-FB315_14*s->l[1][15]+s->dpt[3][24]*(FM116_14*C16-FM216_14*S16))-S15*(CM316_14
 -s->In[9][15]*S15-FB115_14*s->l[2][15]+FB215_14*s->l[1][15]);

// = = Block_0_2_0_2_0_4 = = 
 
// Backward Dynamics 

  FA123 = -(s->frc[1][23]-s->m[23]*(AlF123+BS123*s->l[1][23]+BeF223*s->l[2][23]+BeF323*s->l[3][23]));
  FA223 = -(s->frc[2][23]-s->m[23]*(AlF223+BS523*s->l[2][23]+BeF423*s->l[1][23]+BeF623*s->l[3][23]));
  FA323 = -(s->frc[3][23]-s->m[23]*(AlF323+BS923*s->l[3][23]+BeF723*s->l[1][23]+BeF823*s->l[2][23]));
  FF123 = FA123+FF124*C24+FF128*C28+FF324*S24+FF328*S28;
  FF223 = FA223+FF224+FF228;
  CF123 = -(s->trq[1][23]-s->In[1][23]*OpF123-CF124*C24-CF128*C28-CF324*S24-CF328*S28+FA223*s->l[3][23]-FA323*
 s->l[2][23]+FF224*s->dpt[3][44]+FF228*s->dpt[3][45]-OM223*(s->In[6][23]*OM223+s->In[9][23]*OM323)+OM323*(s->In[5][23]*OM223+
 s->In[6][23]*OM323)+s->dpt[2][44]*(FF124*S24-FF324*C24)+s->dpt[2][45]*(FF128*S28-FF328*C28));
  CF223 = -(s->trq[2][23]-CF224-CF228-s->In[5][23]*OpF223-s->In[6][23]*OpF322-FA123*s->l[3][23]+FA323*s->l[1][23]-OM123*
 (s->In[1][23]*OM323-s->In[6][23]*OM223-s->In[9][23]*OM323)-s->dpt[1][44]*(FF124*S24-FF324*C24)-s->dpt[1][45]*(FF128*S28-
 FF328*C28)-s->dpt[3][44]*(FF124*C24+FF324*S24)-s->dpt[3][45]*(FF128*C28+FF328*S28));
  CF323 = -(s->trq[3][23]-s->In[6][23]*OpF223-s->In[9][23]*OpF322+CF124*S24+CF128*S28-CF324*C24-CF328*C28+FA123*
 s->l[2][23]-FA223*s->l[1][23]-FF224*s->dpt[1][44]-FF228*s->dpt[1][45]-OM123*(s->In[6][23]*OM323-OM223*(s->In[1][23]-
 s->In[5][23]))+s->dpt[2][44]*(FF124*C24+FF324*S24)+s->dpt[2][45]*(FF128*C28+FF328*S28));
  FB123_1 = s->m[23]*AlM123_1;
  FB223_1 = s->m[23]*AlM223_1;
  FB323_1 = s->m[23]*AlM322_1;
  FM123_1 = FB123_1+FM124_1*C24+FM128_1*C28+FM324_1*S24+FM328_1*S28;
  FM223_1 = FB223_1+FM224_1+FM228_1;
  CM123_1 = CM124_1*C24+CM128_1*C28+CM324_1*S24+CM328_1*S28-FB223_1*s->l[3][23]+FB323_1*s->l[2][23]-FM224_1*
 s->dpt[3][44]-FM228_1*s->dpt[3][45]-s->dpt[2][44]*(FM124_1*S24-FM324_1*C24)-s->dpt[2][45]*(FM128_1*S28-FM328_1*C28);
  CM223_1 = CM224_1+CM228_1+FB123_1*s->l[3][23]-FB323_1*s->l[1][23]+s->dpt[1][44]*(FM124_1*S24-FM324_1*C24)+
 s->dpt[1][45]*(FM128_1*S28-FM328_1*C28)+s->dpt[3][44]*(FM124_1*C24+FM324_1*S24)+s->dpt[3][45]*(FM128_1*C28+FM328_1*S28);
  CM323_1 = -(CM124_1*S24+CM128_1*S28-CM324_1*C24-CM328_1*C28+FB123_1*s->l[2][23]-FB223_1*s->l[1][23]-FM224_1*
 s->dpt[1][44]-FM228_1*s->dpt[1][45]+s->dpt[2][44]*(FM124_1*C24+FM324_1*S24)+s->dpt[2][45]*(FM128_1*C28+FM328_1*S28));
  FB123_2 = s->m[23]*AlM123_2;
  FB223_2 = s->m[23]*AlM223_2;
  FB323_2 = s->m[23]*AlM322_2;
  FM123_2 = FB123_2+FM124_2*C24+FM128_2*C28+FM324_2*S24+FM328_2*S28;
  FM223_2 = FB223_2+FM224_2+FM228_2;
  CM123_2 = CM124_2*C24+CM128_2*C28+CM324_2*S24+CM328_2*S28-FB223_2*s->l[3][23]+FB323_2*s->l[2][23]-FM224_2*
 s->dpt[3][44]-FM228_2*s->dpt[3][45]-s->dpt[2][44]*(FM124_2*S24-FM324_2*C24)-s->dpt[2][45]*(FM128_2*S28-FM328_2*C28);
  CM223_2 = CM224_2+CM228_2+FB123_2*s->l[3][23]-FB323_2*s->l[1][23]+s->dpt[1][44]*(FM124_2*S24-FM324_2*C24)+
 s->dpt[1][45]*(FM128_2*S28-FM328_2*C28)+s->dpt[3][44]*(FM124_2*C24+FM324_2*S24)+s->dpt[3][45]*(FM128_2*C28+FM328_2*S28);
  CM323_2 = -(CM124_2*S24+CM128_2*S28-CM324_2*C24-CM328_2*C28+FB123_2*s->l[2][23]-FB223_2*s->l[1][23]-FM224_2*
 s->dpt[1][44]-FM228_2*s->dpt[1][45]+s->dpt[2][44]*(FM124_2*C24+FM324_2*S24)+s->dpt[2][45]*(FM128_2*C28+FM328_2*S28));
  FB123_3 = s->m[23]*AlM123_3;
  FB223_3 = s->m[23]*AlM223_3;
  FB323_3 = s->m[23]*AlM322_3;
  FM123_3 = FB123_3+FM124_3*C24+FM128_3*C28+FM324_3*S24+FM328_3*S28;
  FM223_3 = FB223_3+FM224_3+FM228_3;
  CM123_3 = CM124_3*C24+CM128_3*C28+CM324_3*S24+CM328_3*S28-FB223_3*s->l[3][23]+FB323_3*s->l[2][23]-FM224_3*
 s->dpt[3][44]-FM228_3*s->dpt[3][45]-s->dpt[2][44]*(FM124_3*S24-FM324_3*C24)-s->dpt[2][45]*(FM128_3*S28-FM328_3*C28);
  CM223_3 = CM224_3+CM228_3+FB123_3*s->l[3][23]-FB323_3*s->l[1][23]+s->dpt[1][44]*(FM124_3*S24-FM324_3*C24)+
 s->dpt[1][45]*(FM128_3*S28-FM328_3*C28)+s->dpt[3][44]*(FM124_3*C24+FM324_3*S24)+s->dpt[3][45]*(FM128_3*C28+FM328_3*S28);
  CM323_3 = -(CM124_3*S24+CM128_3*S28-CM324_3*C24-CM328_3*C28+FB123_3*s->l[2][23]-FB223_3*s->l[1][23]-FM224_3*
 s->dpt[1][44]-FM228_3*s->dpt[1][45]+s->dpt[2][44]*(FM124_3*C24+FM324_3*S24)+s->dpt[2][45]*(FM128_3*C28+FM328_3*S28));
  FB123_4 = s->m[23]*(AlM123_4+OpM223_4*s->l[3][23]-OpM322_4*s->l[2][23]);
  FB223_4 = s->m[23]*(AlM223_4-OpM123_4*s->l[3][23]+OpM322_4*s->l[1][23]);
  FB323_4 = s->m[23]*(AlM322_4+OpM123_4*s->l[2][23]-OpM223_4*s->l[1][23]);
  FM123_4 = FB123_4+FM124_4*C24+FM128_4*C28+FM324_4*S24+FM328_4*S28;
  FM223_4 = FB223_4+FM224_4+FM228_4;
  CM123_4 = s->In[1][23]*OpM123_4+CM124_4*C24+CM128_4*C28+CM324_4*S24+CM328_4*S28-FB223_4*s->l[3][23]+FB323_4*
 s->l[2][23]-FM224_4*s->dpt[3][44]-FM228_4*s->dpt[3][45]-s->dpt[2][44]*(FM124_4*S24-FM324_4*C24)-s->dpt[2][45]*(FM128_4*S28-
 FM328_4*C28);
  CM223_4 = CM224_4+CM228_4+s->In[5][23]*OpM223_4+s->In[6][23]*OpM322_4+FB123_4*s->l[3][23]-FB323_4*s->l[1][23]+
 s->dpt[1][44]*(FM124_4*S24-FM324_4*C24)+s->dpt[1][45]*(FM128_4*S28-FM328_4*C28)+s->dpt[3][44]*(FM124_4*C24+FM324_4*S24)+
 s->dpt[3][45]*(FM128_4*C28+FM328_4*S28);
  CM323_4 = s->In[6][23]*OpM223_4+s->In[9][23]*OpM322_4-CM124_4*S24-CM128_4*S28+CM324_4*C24+CM328_4*C28-FB123_4*
 s->l[2][23]+FB223_4*s->l[1][23]+FM224_4*s->dpt[1][44]+FM228_4*s->dpt[1][45]-s->dpt[2][44]*(FM124_4*C24+FM324_4*S24)-
 s->dpt[2][45]*(FM128_4*C28+FM328_4*S28);
  FB123_5 = s->m[23]*(AlM123_5+OpM223_5*s->l[3][23]-OpM322_5*s->l[2][23]);
  FB223_5 = s->m[23]*(AlM223_5-OpM123_5*s->l[3][23]+OpM322_5*s->l[1][23]);
  FB323_5 = s->m[23]*(AlM322_5+OpM123_5*s->l[2][23]-OpM223_5*s->l[1][23]);
  FM123_5 = FB123_5+FM124_5*C24+FM128_5*C28+FM324_5*S24+FM328_5*S28;
  FM223_5 = FB223_5+FM224_5+FM228_5;
  CM123_5 = s->In[1][23]*OpM123_5+CM124_5*C24+CM128_5*C28+CM324_5*S24+CM328_5*S28-FB223_5*s->l[3][23]+FB323_5*
 s->l[2][23]-FM224_5*s->dpt[3][44]-FM228_5*s->dpt[3][45]-s->dpt[2][44]*(FM124_5*S24-FM324_5*C24)-s->dpt[2][45]*(FM128_5*S28-
 FM328_5*C28);
  CM223_5 = CM224_5+CM228_5+s->In[5][23]*OpM223_5+s->In[6][23]*OpM322_5+FB123_5*s->l[3][23]-FB323_5*s->l[1][23]+
 s->dpt[1][44]*(FM124_5*S24-FM324_5*C24)+s->dpt[1][45]*(FM128_5*S28-FM328_5*C28)+s->dpt[3][44]*(FM124_5*C24+FM324_5*S24)+
 s->dpt[3][45]*(FM128_5*C28+FM328_5*S28);
  CM323_5 = s->In[6][23]*OpM223_5+s->In[9][23]*OpM322_5-CM124_5*S24-CM128_5*S28+CM324_5*C24+CM328_5*C28-FB123_5*
 s->l[2][23]+FB223_5*s->l[1][23]+FM224_5*s->dpt[1][44]+FM228_5*s->dpt[1][45]-s->dpt[2][44]*(FM124_5*C24+FM324_5*S24)-
 s->dpt[2][45]*(FM128_5*C28+FM328_5*S28);
  FB123_6 = s->m[23]*(AlM123_6+OpM223_6*s->l[3][23]-OpM322_6*s->l[2][23]);
  FB223_6 = s->m[23]*(AlM223_6-OpM123_6*s->l[3][23]+OpM322_6*s->l[1][23]);
  FB323_6 = s->m[23]*(AlM322_6+OpM123_6*s->l[2][23]-OpM223_6*s->l[1][23]);
  FM123_6 = FB123_6+FM124_6*C24+FM128_6*C28+FM324_6*S24+FM328_6*S28;
  FM223_6 = FB223_6+FM224_6+FM228_6;
  CM123_6 = s->In[1][23]*OpM123_6+CM124_6*C24+CM128_6*C28+CM324_6*S24+CM328_6*S28-FB223_6*s->l[3][23]+FB323_6*
 s->l[2][23]-FM224_6*s->dpt[3][44]-FM228_6*s->dpt[3][45]-s->dpt[2][44]*(FM124_6*S24-FM324_6*C24)-s->dpt[2][45]*(FM128_6*S28-
 FM328_6*C28);
  CM223_6 = CM224_6+CM228_6+s->In[5][23]*OpM223_6+s->In[6][23]*OpM322_6+FB123_6*s->l[3][23]-FB323_6*s->l[1][23]+
 s->dpt[1][44]*(FM124_6*S24-FM324_6*C24)+s->dpt[1][45]*(FM128_6*S28-FM328_6*C28)+s->dpt[3][44]*(FM124_6*C24+FM324_6*S24)+
 s->dpt[3][45]*(FM128_6*C28+FM328_6*S28);
  CM323_6 = s->In[6][23]*OpM223_6+s->In[9][23]*OpM322_6-CM124_6*S24-CM128_6*S28+CM324_6*C24+CM328_6*C28-FB123_6*
 s->l[2][23]+FB223_6*s->l[1][23]+FM224_6*s->dpt[1][44]+FM228_6*s->dpt[1][45]-s->dpt[2][44]*(FM124_6*C24+FM324_6*S24)-
 s->dpt[2][45]*(FM128_6*C28+FM328_6*S28);
  FB123_21 = s->m[23]*(AlM123_21+OpM223_21*s->l[3][23]-s->l[2][23]*S22);
  FB223_21 = s->m[23]*(AlM223_21-OpM123_21*s->l[3][23]+s->l[1][23]*S22);
  FB323_21 = s->m[23]*(OpM123_21*s->l[2][23]-OpM223_21*s->l[1][23]);
  FM123_21 = FB123_21+FM124_21*C24+FM128_21*C28+FM324_21*S24+FM328_21*S28;
  FM223_21 = FB223_21+FM224_21+FM228_21;
  CM123_21 = s->In[1][23]*OpM123_21+CM124_21*C24+CM128_21*C28+CM324_21*S24+CM328_21*S28-FB223_21*s->l[3][23]+FB323_21*
 s->l[2][23]-FM224_21*s->dpt[3][44]-FM228_21*s->dpt[3][45]-s->dpt[2][44]*(FM124_21*S24-FM324_21*C24)-s->dpt[2][45]*(FM128_21*
 S28-FM328_21*C28);
  CM223_21 = CM224_21+CM228_21+s->In[5][23]*OpM223_21+s->In[6][23]*S22+FB123_21*s->l[3][23]-FB323_21*s->l[1][23]+
 s->dpt[1][44]*(FM124_21*S24-FM324_21*C24)+s->dpt[1][45]*(FM128_21*S28-FM328_21*C28)+s->dpt[3][44]*(FM124_21*C24+FM324_21*S24
 )+s->dpt[3][45]*(FM128_21*C28+FM328_21*S28);
  CM323_21 = s->In[6][23]*OpM223_21+s->In[9][23]*S22-CM124_21*S24-CM128_21*S28+CM324_21*C24+CM328_21*C28-FB123_21*
 s->l[2][23]+FB223_21*s->l[1][23]+FM224_21*s->dpt[1][44]+FM228_21*s->dpt[1][45]-s->dpt[2][44]*(FM124_21*C24+FM324_21*S24)-
 s->dpt[2][45]*(FM128_21*C28+FM328_21*S28);
  FB123_22 = s->m[23]*C23*(s->dpt[3][40]+s->l[3][23]);
  FB223_22 = s->m[23]*(AlM223_22-s->l[3][23]*S23);
  FB323_22 = -s->m[23]*(s->l[1][23]*C23-s->l[2][23]*S23);
  CM323_22 = s->In[6][23]*C23-CM124_22*S24-CM128_22*S28+CM324_22*C24+CM328_22*C28-FB123_22*s->l[2][23]+FB223_22*
 s->l[1][23]+FM224_22*s->dpt[1][44]+FM228_22*s->dpt[1][45]-s->dpt[2][44]*(FM124_22*C24+FM324_22*S24)-s->dpt[2][45]*(FM128_22*
 C28+FM328_22*S28);
  CM323_23 = s->In[9][23]+s->m[23]*s->l[1][23]*s->l[1][23]+s->m[23]*s->l[2][23]*s->l[2][23]+s->dpt[1][44]*(FB224_23+
 FM225_23*C25-FM325_23*S25)+s->dpt[1][45]*(FB228_23+FM229_23*C29-FM329_23*S29)-s->dpt[2][44]*(C24*(FB124_23+FM125_23)+S24*(
 FB324_23+FM225_23*S25+FM325_23*C25))-s->dpt[2][45]*(C28*(FB128_23+FM129_23)+S28*(FB328_23+FM229_23*S29+FM329_23*C29))+C24*(
 s->In[9][24]*C24+CM225_23*S25+CM325_23*C25-FB124_23*s->l[2][24]+FB224_23*s->l[1][24]-FM125_23*s->dpt[2][47])-S24*(CM125_23-
 s->In[1][24]*S24-FB224_23*s->l[3][24]+FB324_23*s->l[2][24]+s->dpt[2][47]*(FM225_23*S25+FM325_23*C25))+C28*(s->In[9][28]*C28+
 CM229_23*S29+CM329_23*C29-FB128_23*s->l[2][28]+FB228_23*s->l[1][28]-FM129_23*s->dpt[2][56])-S28*(CM129_23-s->In[1][28]*S28-
 FB228_23*s->l[3][28]+FB328_23*s->l[2][28]+s->dpt[2][56]*(FM229_23*S29+FM329_23*C29));
  FA122 = -(s->frc[1][22]-s->m[22]*(AlF122+BeF322*s->l[3][22]-s->l[1][22]*(OM222*OM222+OM322*OM322)+s->l[2][22]*(BS222-
 OpF322)));
  FA222 = -(s->frc[2][22]-s->m[22]*(AlF221+BeF622*s->l[3][22]+s->l[1][22]*(BS222+OpF322)-s->l[2][22]*(OM122*OM122+OM322*
 OM322)));
  FA322 = -(s->frc[3][22]-s->m[22]*(AlF322+BS922*s->l[3][22]+s->l[1][22]*(BS322-OpF221)+s->l[2][22]*(BS622+OpF122)));
  FF122 = FA122+FF123*C23-FF223*S23;
  FF322 = FA322+FA323-FF124*S24-FF128*S28+FF324*C24+FF328*C28;
  CF122 = -(s->trq[1][22]-s->In[1][22]*OpF122-s->In[3][22]*OpF322-CF123*C23+CF223*S23+FA222*s->l[3][22]-FA322*
 s->l[2][22]-OM222*(s->In[3][22]*OM122-s->In[5][22]*OM322+s->In[9][22]*OM322)+s->dpt[3][40]*(FF123*S23+FF223*C23));
  CF222 = -(s->trq[2][22]-s->In[5][22]*OpF221-CF123*S23-CF223*C23-FA122*s->l[3][22]+FA322*s->l[1][22]+OM122*(
 s->In[3][22]*OM122+s->In[9][22]*OM322)-OM322*(s->In[1][22]*OM122+s->In[3][22]*OM322)-s->dpt[3][40]*(FF123*C23-FF223*S23));
  CF322 = -(s->trq[3][22]-CF323-s->In[3][22]*OpF122-s->In[9][22]*OpF322+FA122*s->l[2][22]-FA222*s->l[1][22]+OM222*(
 s->In[1][22]*OM122+s->In[3][22]*OM322-s->In[5][22]*OM122));
  FB122_1 = s->m[22]*AlM122_1;
  FB222_1 = s->m[22]*AlM221_1;
  FB322_1 = s->m[22]*AlM322_1;
  FM122_1 = FB122_1+FM123_1*C23-FM223_1*S23;
  FM322_1 = FB322_1+FB323_1-FM124_1*S24-FM128_1*S28+FM324_1*C24+FM328_1*C28;
  CM122_1 = CM123_1*C23-CM223_1*S23-FB222_1*s->l[3][22]+FB322_1*s->l[2][22]-s->dpt[3][40]*(FM123_1*S23+FM223_1*C23);
  CM222_1 = CM123_1*S23+CM223_1*C23+FB122_1*s->l[3][22]-FB322_1*s->l[1][22]+s->dpt[3][40]*(FM123_1*C23-FM223_1*S23);
  CM322_1 = CM323_1-FB122_1*s->l[2][22]+FB222_1*s->l[1][22];
  FB122_2 = s->m[22]*AlM122_2;
  FB222_2 = s->m[22]*AlM221_2;
  FB322_2 = s->m[22]*AlM322_2;
  FM122_2 = FB122_2+FM123_2*C23-FM223_2*S23;
  FM322_2 = FB322_2+FB323_2-FM124_2*S24-FM128_2*S28+FM324_2*C24+FM328_2*C28;
  CM122_2 = CM123_2*C23-CM223_2*S23-FB222_2*s->l[3][22]+FB322_2*s->l[2][22]-s->dpt[3][40]*(FM123_2*S23+FM223_2*C23);
  CM222_2 = CM123_2*S23+CM223_2*C23+FB122_2*s->l[3][22]-FB322_2*s->l[1][22]+s->dpt[3][40]*(FM123_2*C23-FM223_2*S23);
  CM322_2 = CM323_2-FB122_2*s->l[2][22]+FB222_2*s->l[1][22];
  FB122_3 = s->m[22]*AlM122_3;
  FB222_3 = s->m[22]*AlM221_3;
  FB322_3 = s->m[22]*AlM322_3;
  FM122_3 = FB122_3+FM123_3*C23-FM223_3*S23;
  FM322_3 = FB322_3+FB323_3-FM124_3*S24-FM128_3*S28+FM324_3*C24+FM328_3*C28;
  CM122_3 = CM123_3*C23-CM223_3*S23-FB222_3*s->l[3][22]+FB322_3*s->l[2][22]-s->dpt[3][40]*(FM123_3*S23+FM223_3*C23);
  CM222_3 = CM123_3*S23+CM223_3*C23+FB122_3*s->l[3][22]-FB322_3*s->l[1][22]+s->dpt[3][40]*(FM123_3*C23-FM223_3*S23);
  CM322_3 = CM323_3-FB122_3*s->l[2][22]+FB222_3*s->l[1][22];
  FB122_4 = s->m[22]*(AlM122_4+OpM221_4*s->l[3][22]-OpM322_4*s->l[2][22]);
  FB222_4 = s->m[22]*(AlM221_4-OpM122_4*s->l[3][22]+OpM322_4*s->l[1][22]);
  FB322_4 = s->m[22]*(AlM322_4+OpM122_4*s->l[2][22]-OpM221_4*s->l[1][22]);
  FM122_4 = FB122_4+FM123_4*C23-FM223_4*S23;
  FM322_4 = FB322_4+FB323_4-FM124_4*S24-FM128_4*S28+FM324_4*C24+FM328_4*C28;
  CM122_4 = s->In[1][22]*OpM122_4+s->In[3][22]*OpM322_4+CM123_4*C23-CM223_4*S23-FB222_4*s->l[3][22]+FB322_4*s->l[2][22]-
 s->dpt[3][40]*(FM123_4*S23+FM223_4*C23);
  CM222_4 = s->In[5][22]*OpM221_4+CM123_4*S23+CM223_4*C23+FB122_4*s->l[3][22]-FB322_4*s->l[1][22]+s->dpt[3][40]*(FM123_4
 *C23-FM223_4*S23);
  CM322_4 = CM323_4+s->In[3][22]*OpM122_4+s->In[9][22]*OpM322_4-FB122_4*s->l[2][22]+FB222_4*s->l[1][22];
  FB122_5 = s->m[22]*(AlM122_5+OpM221_5*s->l[3][22]-OpM322_5*s->l[2][22]);
  FB222_5 = s->m[22]*(AlM221_5-OpM122_5*s->l[3][22]+OpM322_5*s->l[1][22]);
  FB322_5 = s->m[22]*(AlM322_5+OpM122_5*s->l[2][22]-OpM221_5*s->l[1][22]);
  FM122_5 = FB122_5+FM123_5*C23-FM223_5*S23;
  FM322_5 = FB322_5+FB323_5-FM124_5*S24-FM128_5*S28+FM324_5*C24+FM328_5*C28;
  CM122_5 = s->In[1][22]*OpM122_5+s->In[3][22]*OpM322_5+CM123_5*C23-CM223_5*S23-FB222_5*s->l[3][22]+FB322_5*s->l[2][22]-
 s->dpt[3][40]*(FM123_5*S23+FM223_5*C23);
  CM222_5 = s->In[5][22]*OpM221_5+CM123_5*S23+CM223_5*C23+FB122_5*s->l[3][22]-FB322_5*s->l[1][22]+s->dpt[3][40]*(FM123_5
 *C23-FM223_5*S23);
  CM322_5 = CM323_5+s->In[3][22]*OpM122_5+s->In[9][22]*OpM322_5-FB122_5*s->l[2][22]+FB222_5*s->l[1][22];
  FB122_6 = s->m[22]*(AlM122_6-OpM322_6*s->l[2][22]+s->l[3][22]*S21);
  FB222_6 = s->m[22]*(AlM221_6-OpM122_6*s->l[3][22]+OpM322_6*s->l[1][22]);
  FB322_6 = s->m[22]*(AlM322_6+OpM122_6*s->l[2][22]-s->l[1][22]*S21);
  CM122_6 = s->In[1][22]*OpM122_6+s->In[3][22]*OpM322_6+CM123_6*C23-CM223_6*S23-FB222_6*s->l[3][22]+FB322_6*s->l[2][22]-
 s->dpt[3][40]*(FM123_6*S23+FM223_6*C23);
  CM222_6 = s->In[5][22]*S21+CM123_6*S23+CM223_6*C23+FB122_6*s->l[3][22]-FB322_6*s->l[1][22]+s->dpt[3][40]*(FM123_6*C23-
 FM223_6*S23);
  CM322_6 = CM323_6+s->In[3][22]*OpM122_6+s->In[9][22]*OpM322_6-FB122_6*s->l[2][22]+FB222_6*s->l[1][22];
  FB122_21 = -s->m[22]*s->l[2][22]*S22;
  FB222_21 = s->m[22]*(s->l[1][22]*S22-s->l[3][22]*C22);
  FB322_21 = s->m[22]*s->l[2][22]*C22;
  CM222_21 = CM123_21*S23+CM223_21*C23+FB122_21*s->l[3][22]-FB322_21*s->l[1][22]+s->dpt[3][40]*(FM123_21*C23-FM223_21*
 S23);
  CM222_22 = s->In[5][22]+s->m[22]*s->l[1][22]*s->l[1][22]+s->m[22]*s->l[3][22]*s->l[3][22]+s->dpt[3][40]*(C23*(FB123_22
 +FM124_22*C24+FM128_22*C28+FM324_22*S24+FM328_22*S28)-S23*(FB223_22+FM224_22+FM228_22))+C23*(CM224_22+CM228_22+s->In[5][23]*
 C23+FB123_22*s->l[3][23]-FB323_22*s->l[1][23]+s->dpt[1][44]*(FM124_22*S24-FM324_22*C24)+s->dpt[1][45]*(FM128_22*S28-FM328_22
 *C28)+s->dpt[3][44]*(FM124_22*C24+FM324_22*S24)+s->dpt[3][45]*(FM128_22*C28+FM328_22*S28))+S23*(s->In[1][23]*S23+CM124_22*
 C24+CM128_22*C28+CM324_22*S24+CM328_22*S28-FB223_22*s->l[3][23]+FB323_22*s->l[2][23]-FM224_22*s->dpt[3][44]-FM228_22*
 s->dpt[3][45]-s->dpt[2][44]*(FM124_22*S24-FM324_22*C24)-s->dpt[2][45]*(FM128_22*S28-FM328_22*C28));
  FA121 = -(s->frc[1][21]-s->m[21]*(AlF121-s->l[1][21]*(OM221*OM221+OM321*OM321)+s->l[2][21]*(BS221-OpF321)+s->l[3][21]*
 (BS321+OpF221)));
  FA221 = -(s->frc[2][21]-s->m[21]*(AlF221+s->l[1][21]*(BS221+OpF321)-s->l[2][21]*(OM121*OM121+OM321*OM321)+s->l[3][21]*
 (BS621-OpF16)));
  FA321 = -(s->frc[3][21]-s->m[21]*(AlF321+s->l[1][21]*(BS321-OpF221)+s->l[2][21]*(BS621+OpF16)-s->l[3][21]*(OM121*OM121
 +OM221*OM221)));
  FF121 = FA121+FF122*C22+FF322*S22;
  FF221 = FA221+FA222+FF123*S23+FF223*C23;
  FF321 = FA321-FF122*S22+FF322*C22;
  CF121 = -(s->trq[1][21]-s->In[1][21]*OpF16-CF122*C22-CF322*S22+FA221*s->l[3][21]-FA321*s->l[2][21]-OM221*(s->In[6][21]
 *OM221+s->In[9][21]*OM321)+OM321*(s->In[5][21]*OM221+s->In[6][21]*OM321));
  CF221 = -(s->trq[2][21]-CF222-s->In[5][21]*OpF221-s->In[6][21]*OpF321-FA121*s->l[3][21]+FA321*s->l[1][21]-OM121*(
 s->In[1][21]*OM321-s->In[6][21]*OM221-s->In[9][21]*OM321));
  CF321 = -(s->trq[3][21]-s->In[6][21]*OpF221-s->In[9][21]*OpF321+CF122*S22-CF322*C22+FA121*s->l[2][21]-FA221*
 s->l[1][21]-OM121*(s->In[6][21]*OM321-OM221*(s->In[1][21]-s->In[5][21])));
  FB121_1 = s->m[21]*AlM16_1;
  FB221_1 = s->m[21]*AlM221_1;
  FB321_1 = s->m[21]*AlM321_1;
  FM121_1 = FB121_1+FM122_1*C22+FM322_1*S22;
  FM221_1 = FB221_1+FB222_1+FM123_1*S23+FM223_1*C23;
  FM321_1 = FB321_1-FM122_1*S22+FM322_1*C22;
  CM121_1 = CM122_1*C22+CM322_1*S22-FB221_1*s->l[3][21]+FB321_1*s->l[2][21];
  CM221_1 = CM222_1+FB121_1*s->l[3][21]-FB321_1*s->l[1][21];
  CM321_1 = -(CM122_1*S22-CM322_1*C22+FB121_1*s->l[2][21]-FB221_1*s->l[1][21]);
  FB121_2 = s->m[21]*AlM16_2;
  FB221_2 = s->m[21]*AlM221_2;
  FB321_2 = s->m[21]*AlM321_2;
  FM121_2 = FB121_2+FM122_2*C22+FM322_2*S22;
  FM221_2 = FB221_2+FB222_2+FM123_2*S23+FM223_2*C23;
  FM321_2 = FB321_2-FM122_2*S22+FM322_2*C22;
  CM121_2 = CM122_2*C22+CM322_2*S22-FB221_2*s->l[3][21]+FB321_2*s->l[2][21];
  CM221_2 = CM222_2+FB121_2*s->l[3][21]-FB321_2*s->l[1][21];
  CM321_2 = -(CM122_2*S22-CM322_2*C22+FB121_2*s->l[2][21]-FB221_2*s->l[1][21]);
  FB121_3 = s->m[21]*AlM16_3;
  FB221_3 = s->m[21]*AlM221_3;
  FB321_3 = s->m[21]*AlM321_3;
  FM121_3 = FB121_3+FM122_3*C22+FM322_3*S22;
  FM221_3 = FB221_3+FB222_3+FM123_3*S23+FM223_3*C23;
  FM321_3 = FB321_3-FM122_3*S22+FM322_3*C22;
  CM121_3 = CM122_3*C22+CM322_3*S22-FB221_3*s->l[3][21]+FB321_3*s->l[2][21];
  CM221_3 = CM222_3+FB121_3*s->l[3][21]-FB321_3*s->l[1][21];
  CM321_3 = -(CM122_3*S22-CM322_3*C22+FB121_3*s->l[2][21]-FB221_3*s->l[1][21]);
  FB121_4 = s->m[21]*(AlM121_4+OpM221_4*s->l[3][21]-OpM321_4*s->l[2][21]);
  FB221_4 = s->m[21]*(AlM221_4-OpM16_4*s->l[3][21]+OpM321_4*s->l[1][21]);
  FB321_4 = s->m[21]*(AlM321_4+OpM16_4*s->l[2][21]-OpM221_4*s->l[1][21]);
  FM221_4 = FB221_4+FB222_4+FM123_4*S23+FM223_4*C23;
  FM321_4 = FB321_4-FM122_4*S22+FM322_4*C22;
  CM121_4 = s->In[1][21]*OpM16_4+CM122_4*C22+CM322_4*S22-FB221_4*s->l[3][21]+FB321_4*s->l[2][21];
  CM221_4 = CM222_4+s->In[5][21]*OpM221_4+s->In[6][21]*OpM321_4+FB121_4*s->l[3][21]-FB321_4*s->l[1][21];
  CM321_4 = s->In[6][21]*OpM221_4+s->In[9][21]*OpM321_4-CM122_4*S22+CM322_4*C22-FB121_4*s->l[2][21]+FB221_4*s->l[1][21];
  FB121_5 = s->m[21]*(AlM121_5+OpM221_5*s->l[3][21]-OpM321_5*s->l[2][21]);
  FB221_5 = s->m[21]*(AlM221_5+OpM321_5*s->l[1][21]-s->l[3][21]*S6);
  FB321_5 = s->m[21]*(AlM321_5-OpM221_5*s->l[1][21]+s->l[2][21]*S6);
  FM221_5 = FB221_5+FB222_5+FM123_5*S23+FM223_5*C23;
  FM321_5 = FB321_5-FM122_5*S22+FM322_5*C22;
  CM121_5 = s->In[1][21]*S6+CM122_5*C22+CM322_5*S22-FB221_5*s->l[3][21]+FB321_5*s->l[2][21];
  CM221_5 = CM222_5+s->In[5][21]*OpM221_5+s->In[6][21]*OpM321_5+FB121_5*s->l[3][21]-FB321_5*s->l[1][21];
  CM321_5 = s->In[6][21]*OpM221_5+s->In[9][21]*OpM321_5-CM122_5*S22+CM322_5*C22-FB121_5*s->l[2][21]+FB221_5*s->l[1][21];
  FB121_6 = -s->m[21]*(s->l[2][21]*C21-s->l[3][21]*S21);
  FB221_6 = s->m[21]*C21*(s->dpt[1][4]+s->l[1][21]);
  FB321_6 = s->m[21]*(AlM321_6-s->l[1][21]*S21);
  CM121_6 = CM122_6*C22+CM322_6*S22-FB221_6*s->l[3][21]+FB321_6*s->l[2][21];
  CM121_21 = s->In[1][21]+s->m[21]*s->l[2][21]*s->l[2][21]+s->m[21]*s->l[3][21]*s->l[3][21]+C22*(s->In[1][22]*C22+
 s->In[3][22]*S22+CM123_21*C23-CM223_21*S23-FB222_21*s->l[3][22]+FB322_21*s->l[2][22]-s->dpt[3][40]*(FM123_21*S23+FM223_21*
 C23))+S22*(CM323_21+s->In[3][22]*C22+s->In[9][22]*S22-FB122_21*s->l[2][22]+FB222_21*s->l[1][22]);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6]+BeF26*s->l[2][6]+BeF36*s->l[3][6]));
  FA26 = -(s->frc[2][6]-s->m[6]*(AlF26+BS56*s->l[2][6]+BeF46*s->l[1][6]+BeF66*s->l[3][6]));
  FA36 = -(s->frc[3][6]-s->m[6]*(AlF35+BS96*s->l[3][6]+BeF76*s->l[1][6]+BeF86*s->l[2][6]));
  FF16 = FA16+FF121+FF114*C14+FF17*C7+FF314*S14+FF37*S7;
  FF26 = FA214+FA26+FA27+FF215*C15+FF221*C21+FF28*C8-FF315*S15-FF321*S21-FF38*S8;
  FF36 = FA36-FF114*S14-FF17*S7+FF221*S21+FF314*C14+FF321*C21+FF37*C7;
  CF16 = -(s->trq[1][6]-CF121-s->In[1][6]*OpF16-s->In[3][6]*OpF35-CF114*C14-CF17*C7-CF314*S14-CF37*S7+FA26*s->l[3][6]-
 FA36*s->l[2][6]-OM26*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)+OM36*(s->In[5][6]*OM26+s->In[6][6]*OM36)+
 s->dpt[2][2]*(FF17*S7-FF37*C7)+s->dpt[2][3]*(FF114*S14-FF314*C14)+s->dpt[3][4]*(FF221*C21-FF321*S21));
  CF26 = -(s->trq[2][6]-CF214-CF27-s->In[5][6]*OpF26-s->In[6][6]*OpF35-CF221*C21+CF321*S21-FA16*s->l[3][6]+FA36*
 s->l[1][6]-FF121*s->dpt[3][4]+OM16*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)-OM36*(s->In[1][6]*OM16+s->In[3][6]*
 OM36)+s->dpt[1][4]*(FF221*S21+FF321*C21));
  CF36 = -(s->trq[3][6]-s->In[3][6]*OpF16-s->In[6][6]*OpF26-s->In[9][6]*OpF35+CF114*S14+CF17*S7-CF221*S21-CF314*C14-
 CF321*C21-CF37*C7+FA16*s->l[2][6]-FA26*s->l[1][6]-OM16*(s->In[5][6]*OM26+s->In[6][6]*OM36)+OM26*(s->In[1][6]*OM16+
 s->In[3][6]*OM36)-s->dpt[1][4]*(FF221*C21-FF321*S21)+s->dpt[2][2]*(FF17*C7+FF37*S7)+s->dpt[2][3]*(FF114*C14+FF314*S14));
  FB16_1 = s->m[6]*AlM16_1;
  FB26_1 = s->m[6]*AlM26_1;
  FB36_1 = s->m[6]*S5;
  FM16_1 = FB16_1+FM121_1+FM114_1*C14+FM17_1*C7+FM314_1*S14+FM37_1*S7;
  FM26_1 = FB214_1+FB26_1+FB27_1+FM215_1*C15+FM221_1*C21+FM28_1*C8-FM315_1*S15-FM321_1*S21-FM38_1*S8;
  FM36_1 = FB36_1-FM114_1*S14-FM17_1*S7+FM221_1*S21+FM314_1*C14+FM321_1*C21+FM37_1*C7;
  CM16_1 = CM121_1+CM114_1*C14+CM17_1*C7+CM314_1*S14+CM37_1*S7-FB26_1*s->l[3][6]+FB36_1*s->l[2][6]-s->dpt[2][2]*(FM17_1*
 S7-FM37_1*C7)-s->dpt[2][3]*(FM114_1*S14-FM314_1*C14)-s->dpt[3][4]*(FM221_1*C21-FM321_1*S21);
  CM26_1 = CM214_1+CM27_1+CM221_1*C21-CM321_1*S21+FB16_1*s->l[3][6]-FB36_1*s->l[1][6]+FM121_1*s->dpt[3][4]-s->dpt[1][4]*
 (FM221_1*S21+FM321_1*C21);
  CM36_1 = CM221_1*S21+CM321_1*C21+s->dpt[1][4]*(FM221_1*C21-FM321_1*S21)-CM114_1*S14-CM17_1*S7+CM314_1*C14+CM37_1*C7-
 FB16_1*s->l[2][6]+FB26_1*s->l[1][6]-s->dpt[2][2]*(FM17_1*C7+FM37_1*S7)-s->dpt[2][3]*(FM114_1*C14+FM314_1*S14);
  FB16_2 = s->m[6]*AlM16_2;
  FB26_2 = s->m[6]*AlM26_2;
  FB36_2 = s->m[6]*AlM35_2;
  FM16_2 = FB16_2+FM121_2+FM114_2*C14+FM17_2*C7+FM314_2*S14+FM37_2*S7;
  FM26_2 = FB214_2+FB26_2+FB27_2+FM215_2*C15+FM221_2*C21+FM28_2*C8-FM315_2*S15-FM321_2*S21-FM38_2*S8;
  CM16_2 = CM121_2+CM114_2*C14+CM17_2*C7+CM314_2*S14+CM37_2*S7-FB26_2*s->l[3][6]+FB36_2*s->l[2][6]-s->dpt[2][2]*(FM17_2*
 S7-FM37_2*C7)-s->dpt[2][3]*(FM114_2*S14-FM314_2*C14)-s->dpt[3][4]*(FM221_2*C21-FM321_2*S21);
  CM26_2 = CM214_2+CM27_2+CM221_2*C21-CM321_2*S21+FB16_2*s->l[3][6]-FB36_2*s->l[1][6]+FM121_2*s->dpt[3][4]-s->dpt[1][4]*
 (FM221_2*S21+FM321_2*C21);
  CM36_2 = CM221_2*S21+CM321_2*C21+s->dpt[1][4]*(FM221_2*C21-FM321_2*S21)-CM114_2*S14-CM17_2*S7+CM314_2*C14+CM37_2*C7-
 FB16_2*s->l[2][6]+FB26_2*s->l[1][6]-s->dpt[2][2]*(FM17_2*C7+FM37_2*S7)-s->dpt[2][3]*(FM114_2*C14+FM314_2*S14);
  FB16_3 = s->m[6]*AlM16_3;
  FB26_3 = s->m[6]*AlM26_3;
  FB36_3 = s->m[6]*AlM35_3;
  FM16_3 = FB16_3+FM121_3+FM114_3*C14+FM17_3*C7+FM314_3*S14+FM37_3*S7;
  FM26_3 = FB214_3+FB26_3+FB27_3+FM215_3*C15+FM221_3*C21+FM28_3*C8-FM315_3*S15-FM321_3*S21-FM38_3*S8;
  CM16_3 = CM121_3+CM114_3*C14+CM17_3*C7+CM314_3*S14+CM37_3*S7-FB26_3*s->l[3][6]+FB36_3*s->l[2][6]-s->dpt[2][2]*(FM17_3*
 S7-FM37_3*C7)-s->dpt[2][3]*(FM114_3*S14-FM314_3*C14)-s->dpt[3][4]*(FM221_3*C21-FM321_3*S21);
  CM26_3 = CM214_3+CM27_3+CM221_3*C21-CM321_3*S21+FB16_3*s->l[3][6]-FB36_3*s->l[1][6]+FM121_3*s->dpt[3][4]-s->dpt[1][4]*
 (FM221_3*S21+FM321_3*C21);
  CM36_3 = CM221_3*S21+CM321_3*C21+s->dpt[1][4]*(FM221_3*C21-FM321_3*S21)-CM114_3*S14-CM17_3*S7+CM314_3*C14+CM37_3*C7-
 FB16_3*s->l[2][6]+FB26_3*s->l[1][6]-s->dpt[2][2]*(FM17_3*C7+FM37_3*S7)-s->dpt[2][3]*(FM114_3*C14+FM314_3*S14);
  FB16_4 = s->m[6]*(OpM26_4*s->l[3][6]-s->l[2][6]*S5);
  FB26_4 = -s->m[6]*(OpM16_4*s->l[3][6]-s->l[1][6]*S5);
  FB36_4 = s->m[6]*(OpM16_4*s->l[2][6]-OpM26_4*s->l[1][6]);
  CM16_4 = CM121_4+s->In[1][6]*OpM16_4+s->In[3][6]*S5+CM114_4*C14+CM17_4*C7+CM314_4*S14+CM37_4*S7-FB26_4*s->l[3][6]+
 FB36_4*s->l[2][6]-s->dpt[2][2]*(FM17_4*S7-FM37_4*C7)-s->dpt[2][3]*(FM114_4*S14-FM314_4*C14)-s->dpt[3][4]*(FM221_4*C21-
 FM321_4*S21);
  CM26_4 = CM214_4+CM27_4+s->In[5][6]*OpM26_4+s->In[6][6]*S5+CM221_4*C21-CM321_4*S21+FB16_4*s->l[3][6]-FB36_4*s->l[1][6]
 -s->dpt[1][4]*(FM221_4*S21+FM321_4*C21)+s->dpt[3][4]*(FB121_4+FM122_4*C22+FM322_4*S22);
  CM36_4 = s->In[3][6]*OpM16_4+s->In[6][6]*OpM26_4+s->In[9][6]*S5-CM114_4*S14-CM17_4*S7+CM221_4*S21+CM314_4*C14+CM321_4*
 C21+CM37_4*C7-FB16_4*s->l[2][6]+FB26_4*s->l[1][6]+s->dpt[1][4]*(FM221_4*C21-FM321_4*S21)-s->dpt[2][2]*(FM17_4*C7+FM37_4*S7)-
 s->dpt[2][3]*(FM114_4*C14+FM314_4*S14);
  FB16_5 = s->m[6]*s->l[3][6]*C6;
  FB26_5 = -s->m[6]*s->l[3][6]*S6;
  FB36_5 = -s->m[6]*(s->l[1][6]*C6-s->l[2][6]*S6);
  CM36_5 = s->In[3][6]*S6+s->In[6][6]*C6-CM114_5*S14-CM17_5*S7+CM221_5*S21+CM314_5*C14+CM321_5*C21+CM37_5*C7-FB16_5*
 s->l[2][6]+FB26_5*s->l[1][6]+s->dpt[1][4]*(FM221_5*C21-FM321_5*S21)-s->dpt[2][2]*(FM17_5*C7+FM37_5*S7)-s->dpt[2][3]*(FM114_5
 *C14+FM314_5*S14);
  CM36_6 = s->In[9][6]+s->m[6]*s->l[1][6]*s->l[1][6]+s->m[6]*s->l[2][6]*s->l[2][6]+s->dpt[1][4]*(C21*(FB221_6+FB222_6+
 FM123_6*S23+FM223_6*C23)-S21*(FB321_6+C22*(FB322_6+FB323_6-FM124_6*S24-FM128_6*S28+FM324_6*C24+FM328_6*C28)-S22*(FB122_6+
 FM123_6*C23-FM223_6*S23)))-s->dpt[2][2]*(C7*(FB17_6+FM18_6)+S7*(FB37_6+FM28_6*S8+FM38_6*C8))-s->dpt[2][3]*(C14*(FB114_6+
 FM115_6)+S14*(FB314_6+FM215_6*S15+FM315_6*C15))+C14*(s->In[9][14]*C14+CM215_6*S15+CM315_6*C15-FB114_6*s->l[2][14]+FB214_6*
 s->l[1][14]-FM115_6*s->dpt[2][22])-S14*(CM115_6-s->In[1][14]*S14-FB214_6*s->l[3][14]+FB314_6*s->l[2][14]+s->dpt[2][22]*(
 FM215_6*S15+FM315_6*C15))+C21*(s->In[6][21]*S21+s->In[9][21]*C21-CM122_6*S22+CM322_6*C22-FB121_6*s->l[2][21]+FB221_6*
 s->l[1][21])+S21*(CM222_6+s->In[5][21]*S21+s->In[6][21]*C21+FB121_6*s->l[3][21]-FB321_6*s->l[1][21])+C7*(s->In[9][7]*C7+
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
  CM55_26 = C6*(CM214_5+CM27_5+s->In[5][6]*C6+CM221_5*C21-CM321_5*S21+FB16_5*s->l[3][6]-FB36_5*s->l[1][6]-s->dpt[1][4]*(
 FM221_5*S21+FM321_5*C21)+s->dpt[3][4]*(FB121_5+FM122_5*C22+FM322_5*S22))+S6*(CM121_5+s->In[1][6]*S6+CM114_5*C14+CM17_5*C7+
 CM314_5*S14+CM37_5*S7-FB26_5*s->l[3][6]+FB36_5*s->l[2][6]-s->dpt[2][2]*(FM17_5*S7-FM37_5*C7)-s->dpt[2][3]*(FM114_5*S14-
 FM314_5*C14)-s->dpt[3][4]*(FM221_5*C21-FM321_5*S21));
  FF4_15 = FF36*S5+FF5_16*C5;
  FF4_35 = FF36*C5-FF5_16*S5;
  CF4_15 = CF36*S5+C5*(CF16*C6-CF26*S6);
  FM41_15 = FM36_1*S5+FM51_16*C5;
  FM41_35 = FM36_1*C5-FM51_16*S5;
  CM41_15 = CM36_1*S5+C5*(CM16_1*C6-CM26_1*S6);
  FM42_35 = C5*(FB36_2-FM114_2*S14-FM17_2*S7+FM221_2*S21+FM314_2*C14+FM321_2*C21+FM37_2*C7)-S5*(FM16_2*C6-FM26_2*S6);
  CM42_15 = CM36_2*S5+C5*(CM16_2*C6-CM26_2*S6);
  CM43_15 = CM36_3*S5+C5*(CM16_3*C6-CM26_3*S6);
  CM44_15 = CM36_4*S5+C5*(CM16_4*C6-CM26_4*S6);
  FF3_24 = -(FF4_35*S4-FF5_26*C4);
  FF3_34 = FF4_35*C4+FF5_26*S4;
  FM31_24 = -(FM41_35*S4-FM51_26*C4);
  FM31_34 = FM41_35*C4+FM51_26*S4;
  FM32_24 = -(FM42_35*S4-FM52_26*C4);
  FM32_34 = FM42_35*C4+FM52_26*S4;
  FM33_34 = C4*(C5*(FB36_3-FM114_3*S14-FM17_3*S7+FM221_3*S21+FM314_3*C14+FM321_3*C21+FM37_3*C7)-S5*(FM16_3*C6-FM26_3*S6)
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
  M[1][13] = CM213_1;
  M[1][14] = CM214_1;
  M[1][15] = CM115_1;
  M[1][16] = CM316_1;
  M[1][17] = CM217_1;
  M[1][18] = CM118_1;
  M[1][19] = CM219_1;
  M[1][20] = CM220_1;
  M[1][21] = CM121_1;
  M[1][22] = CM222_1;
  M[1][23] = CM323_1;
  M[1][24] = CM224_1;
  M[1][25] = CM125_1;
  M[1][26] = CM326_1;
  M[1][27] = CM227_1;
  M[1][28] = CM228_1;
  M[1][29] = CM129_1;
  M[1][30] = CM330_1;
  M[1][31] = CM231_1;
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
  M[2][13] = CM213_2;
  M[2][14] = CM214_2;
  M[2][15] = CM115_2;
  M[2][16] = CM316_2;
  M[2][17] = CM217_2;
  M[2][18] = CM118_2;
  M[2][19] = CM219_2;
  M[2][20] = CM220_2;
  M[2][21] = CM121_2;
  M[2][22] = CM222_2;
  M[2][23] = CM323_2;
  M[2][24] = CM224_2;
  M[2][25] = CM125_2;
  M[2][26] = CM326_2;
  M[2][27] = CM227_2;
  M[2][28] = CM228_2;
  M[2][29] = CM129_2;
  M[2][30] = CM330_2;
  M[2][31] = CM231_2;
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
  M[3][13] = CM213_3;
  M[3][14] = CM214_3;
  M[3][15] = CM115_3;
  M[3][16] = CM316_3;
  M[3][17] = CM217_3;
  M[3][18] = CM118_3;
  M[3][19] = CM219_3;
  M[3][20] = CM220_3;
  M[3][21] = CM121_3;
  M[3][22] = CM222_3;
  M[3][23] = CM323_3;
  M[3][24] = CM224_3;
  M[3][25] = CM125_3;
  M[3][26] = CM326_3;
  M[3][27] = CM227_3;
  M[3][28] = CM228_3;
  M[3][29] = CM129_3;
  M[3][30] = CM330_3;
  M[3][31] = CM231_3;
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
  M[4][13] = CM213_4;
  M[4][14] = CM214_4;
  M[4][15] = CM115_4;
  M[4][16] = CM316_4;
  M[4][17] = CM217_4;
  M[4][18] = CM118_4;
  M[4][19] = CM219_4;
  M[4][20] = CM220_4;
  M[4][21] = CM121_4;
  M[4][22] = CM222_4;
  M[4][23] = CM323_4;
  M[4][24] = CM224_4;
  M[4][25] = CM125_4;
  M[4][26] = CM326_4;
  M[4][27] = CM227_4;
  M[4][28] = CM228_4;
  M[4][29] = CM129_4;
  M[4][30] = CM330_4;
  M[4][31] = CM231_4;
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
  M[5][13] = CM213_5;
  M[5][14] = CM214_5;
  M[5][15] = CM115_5;
  M[5][16] = CM316_5;
  M[5][17] = CM217_5;
  M[5][18] = CM118_5;
  M[5][19] = CM219_5;
  M[5][20] = CM220_5;
  M[5][21] = CM121_5;
  M[5][22] = CM222_5;
  M[5][23] = CM323_5;
  M[5][24] = CM224_5;
  M[5][25] = CM125_5;
  M[5][26] = CM326_5;
  M[5][27] = CM227_5;
  M[5][28] = CM228_5;
  M[5][29] = CM129_5;
  M[5][30] = CM330_5;
  M[5][31] = CM231_5;
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
  M[6][13] = CM213_6;
  M[6][14] = CM214_6;
  M[6][15] = CM115_6;
  M[6][16] = CM316_6;
  M[6][17] = CM217_6;
  M[6][18] = CM118_6;
  M[6][19] = CM219_6;
  M[6][20] = CM220_6;
  M[6][21] = CM121_6;
  M[6][22] = CM222_6;
  M[6][23] = CM323_6;
  M[6][24] = CM224_6;
  M[6][25] = CM125_6;
  M[6][26] = CM326_6;
  M[6][27] = CM227_6;
  M[6][28] = CM228_6;
  M[6][29] = CM129_6;
  M[6][30] = CM330_6;
  M[6][31] = CM231_6;
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
  M[7][13] = CM213_7;
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
  M[8][13] = CM213_8;
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
  M[9][13] = CM213_9;
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
  M[10][13] = CM213_10;
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
  M[12][13] = CM213_12;
  M[13][1] = CM213_1;
  M[13][2] = CM213_2;
  M[13][3] = CM213_3;
  M[13][4] = CM213_4;
  M[13][5] = CM213_5;
  M[13][6] = CM213_6;
  M[13][7] = CM213_7;
  M[13][8] = CM213_8;
  M[13][9] = CM213_9;
  M[13][10] = CM213_10;
  M[13][12] = CM213_12;
  M[13][13] = CM213_13;
  M[14][1] = CM214_1;
  M[14][2] = CM214_2;
  M[14][3] = CM214_3;
  M[14][4] = CM214_4;
  M[14][5] = CM214_5;
  M[14][6] = CM214_6;
  M[14][14] = CM214_14;
  M[14][15] = CM115_14;
  M[14][16] = CM316_14;
  M[14][17] = CM217_14;
  M[14][18] = CM118_14;
  M[14][19] = CM219_14;
  M[14][20] = CM220_14;
  M[15][1] = CM115_1;
  M[15][2] = CM115_2;
  M[15][3] = CM115_3;
  M[15][4] = CM115_4;
  M[15][5] = CM115_5;
  M[15][6] = CM115_6;
  M[15][14] = CM115_14;
  M[15][15] = CM115_15;
  M[15][16] = CM316_15;
  M[15][17] = CM217_15;
  M[15][18] = CM118_15;
  M[15][19] = CM219_15;
  M[15][20] = CM220_15;
  M[16][1] = CM316_1;
  M[16][2] = CM316_2;
  M[16][3] = CM316_3;
  M[16][4] = CM316_4;
  M[16][5] = CM316_5;
  M[16][6] = CM316_6;
  M[16][14] = CM316_14;
  M[16][15] = CM316_15;
  M[16][16] = CM316_16;
  M[16][17] = CM217_16;
  M[16][18] = CM118_16;
  M[16][19] = CM219_16;
  M[16][20] = CM220_16;
  M[17][1] = CM217_1;
  M[17][2] = CM217_2;
  M[17][3] = CM217_3;
  M[17][4] = CM217_4;
  M[17][5] = CM217_5;
  M[17][6] = CM217_6;
  M[17][14] = CM217_14;
  M[17][15] = CM217_15;
  M[17][16] = CM217_16;
  M[17][17] = CM217_17;
  M[17][18] = CM118_17;
  M[17][19] = CM219_17;
  M[17][20] = CM220_17;
  M[18][1] = CM118_1;
  M[18][2] = CM118_2;
  M[18][3] = CM118_3;
  M[18][4] = CM118_4;
  M[18][5] = CM118_5;
  M[18][6] = CM118_6;
  M[18][14] = CM118_14;
  M[18][15] = CM118_15;
  M[18][16] = CM118_16;
  M[18][17] = CM118_17;
  M[18][18] = CM118_18;
  M[19][1] = CM219_1;
  M[19][2] = CM219_2;
  M[19][3] = CM219_3;
  M[19][4] = CM219_4;
  M[19][5] = CM219_5;
  M[19][6] = CM219_6;
  M[19][14] = CM219_14;
  M[19][15] = CM219_15;
  M[19][16] = CM219_16;
  M[19][17] = CM219_17;
  M[19][19] = CM219_19;
  M[19][20] = CM220_19;
  M[20][1] = CM220_1;
  M[20][2] = CM220_2;
  M[20][3] = CM220_3;
  M[20][4] = CM220_4;
  M[20][5] = CM220_5;
  M[20][6] = CM220_6;
  M[20][14] = CM220_14;
  M[20][15] = CM220_15;
  M[20][16] = CM220_16;
  M[20][17] = CM220_17;
  M[20][19] = CM220_19;
  M[20][20] = CM220_20;
  M[21][1] = CM121_1;
  M[21][2] = CM121_2;
  M[21][3] = CM121_3;
  M[21][4] = CM121_4;
  M[21][5] = CM121_5;
  M[21][6] = CM121_6;
  M[21][21] = CM121_21;
  M[21][22] = CM222_21;
  M[21][23] = CM323_21;
  M[21][24] = CM224_21;
  M[21][25] = CM125_21;
  M[21][26] = CM326_21;
  M[21][27] = CM227_21;
  M[21][28] = CM228_21;
  M[21][29] = CM129_21;
  M[21][30] = CM330_21;
  M[21][31] = CM231_21;
  M[22][1] = CM222_1;
  M[22][2] = CM222_2;
  M[22][3] = CM222_3;
  M[22][4] = CM222_4;
  M[22][5] = CM222_5;
  M[22][6] = CM222_6;
  M[22][21] = CM222_21;
  M[22][22] = CM222_22;
  M[22][23] = CM323_22;
  M[22][24] = CM224_22;
  M[22][25] = CM125_22;
  M[22][26] = CM326_22;
  M[22][27] = CM227_22;
  M[22][28] = CM228_22;
  M[22][29] = CM129_22;
  M[22][30] = CM330_22;
  M[22][31] = CM231_22;
  M[23][1] = CM323_1;
  M[23][2] = CM323_2;
  M[23][3] = CM323_3;
  M[23][4] = CM323_4;
  M[23][5] = CM323_5;
  M[23][6] = CM323_6;
  M[23][21] = CM323_21;
  M[23][22] = CM323_22;
  M[23][23] = CM323_23;
  M[23][24] = CM224_23;
  M[23][25] = CM125_23;
  M[23][26] = CM326_23;
  M[23][27] = CM227_23;
  M[23][28] = CM228_23;
  M[23][29] = CM129_23;
  M[23][30] = CM330_23;
  M[23][31] = CM231_23;
  M[24][1] = CM224_1;
  M[24][2] = CM224_2;
  M[24][3] = CM224_3;
  M[24][4] = CM224_4;
  M[24][5] = CM224_5;
  M[24][6] = CM224_6;
  M[24][21] = CM224_21;
  M[24][22] = CM224_22;
  M[24][23] = CM224_23;
  M[24][24] = CM224_24;
  M[24][25] = CM125_24;
  M[24][26] = CM326_24;
  M[24][27] = CM227_24;
  M[25][1] = CM125_1;
  M[25][2] = CM125_2;
  M[25][3] = CM125_3;
  M[25][4] = CM125_4;
  M[25][5] = CM125_5;
  M[25][6] = CM125_6;
  M[25][21] = CM125_21;
  M[25][22] = CM125_22;
  M[25][23] = CM125_23;
  M[25][24] = CM125_24;
  M[25][25] = CM125_25;
  M[25][26] = CM326_25;
  M[25][27] = CM227_25;
  M[26][1] = CM326_1;
  M[26][2] = CM326_2;
  M[26][3] = CM326_3;
  M[26][4] = CM326_4;
  M[26][5] = CM326_5;
  M[26][6] = CM326_6;
  M[26][21] = CM326_21;
  M[26][22] = CM326_22;
  M[26][23] = CM326_23;
  M[26][24] = CM326_24;
  M[26][25] = CM326_25;
  M[26][26] = CM326_26;
  M[26][27] = CM227_26;
  M[27][1] = CM227_1;
  M[27][2] = CM227_2;
  M[27][3] = CM227_3;
  M[27][4] = CM227_4;
  M[27][5] = CM227_5;
  M[27][6] = CM227_6;
  M[27][21] = CM227_21;
  M[27][22] = CM227_22;
  M[27][23] = CM227_23;
  M[27][24] = CM227_24;
  M[27][25] = CM227_25;
  M[27][26] = CM227_26;
  M[27][27] = CM227_27;
  M[28][1] = CM228_1;
  M[28][2] = CM228_2;
  M[28][3] = CM228_3;
  M[28][4] = CM228_4;
  M[28][5] = CM228_5;
  M[28][6] = CM228_6;
  M[28][21] = CM228_21;
  M[28][22] = CM228_22;
  M[28][23] = CM228_23;
  M[28][28] = CM228_28;
  M[28][29] = CM129_28;
  M[28][30] = CM330_28;
  M[28][31] = CM231_28;
  M[29][1] = CM129_1;
  M[29][2] = CM129_2;
  M[29][3] = CM129_3;
  M[29][4] = CM129_4;
  M[29][5] = CM129_5;
  M[29][6] = CM129_6;
  M[29][21] = CM129_21;
  M[29][22] = CM129_22;
  M[29][23] = CM129_23;
  M[29][28] = CM129_28;
  M[29][29] = CM129_29;
  M[29][30] = CM330_29;
  M[29][31] = CM231_29;
  M[30][1] = CM330_1;
  M[30][2] = CM330_2;
  M[30][3] = CM330_3;
  M[30][4] = CM330_4;
  M[30][5] = CM330_5;
  M[30][6] = CM330_6;
  M[30][21] = CM330_21;
  M[30][22] = CM330_22;
  M[30][23] = CM330_23;
  M[30][28] = CM330_28;
  M[30][29] = CM330_29;
  M[30][30] = CM330_30;
  M[30][31] = CM231_30;
  M[31][1] = CM231_1;
  M[31][2] = CM231_2;
  M[31][3] = CM231_3;
  M[31][4] = CM231_4;
  M[31][5] = CM231_5;
  M[31][6] = CM231_6;
  M[31][21] = CM231_21;
  M[31][22] = CM231_22;
  M[31][23] = CM231_23;
  M[31][28] = CM231_28;
  M[31][29] = CM231_29;
  M[31][30] = CM231_30;
  M[31][31] = CM231_31;
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
  c[13] = CF213;
  c[14] = CF214;
  c[15] = CF115;
  c[16] = CF316;
  c[17] = CF217;
  c[18] = CF118;
  c[19] = CF219;
  c[20] = CF220;
  c[21] = CF121;
  c[22] = CF222;
  c[23] = CF323;
  c[24] = CF224;
  c[25] = CF125;
  c[26] = CF326;
  c[27] = CF227;
  c[28] = CF228;
  c[29] = CF129;
  c[30] = CF330;
  c[31] = CF231;

// ====== END Task 0 ====== 


}
 

