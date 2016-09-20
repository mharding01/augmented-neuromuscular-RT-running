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
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 4310
//
//	==> Generation Time :  0.050 seconds
//	==> Post-Processing :  0.040 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[31];
{ 
 
#include "mbs_invdyna_coman_spring_toe_short_feet.h" 
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

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA33 = qdd[3]-s->g[3];
  ALPHA24 = qdd[2]*C4+ALPHA33*S4;
  ALPHA34 = -(qdd[2]*S4-ALPHA33*C4);
  OM15 = qd[4]*C5;
  OMp15 = -(qd[4]*qd[5]*S5-qdd[4]*C5);
  ALPHA15 = qdd[1]*C5-ALPHA34*S5;
  ALPHA35 = qdd[1]*S5+ALPHA34*C5;
  OM16 = qd[5]*S6+OM15*C6;
  OM26 = qd[5]*C6-OM15*S6;
  OM36 = qd[6]+qd[4]*S5;
  OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15);
  OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6]);
  OMp36 = qdd[6]+qd[4]*qd[5]*C5+qdd[4]*S5;
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BETA26 = BS26-OMp36;
  BETA36 = BS36+OMp26;
  BETA46 = BS26+OMp36;
  BETA66 = BS66-OMp16;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA15*C6+ALPHA24*S6;
  ALPHA26 = -(ALPHA15*S6-ALPHA24*C6);
  OM17 = OM16*C7-OM36*S7;
  OM27 = qd[7]+OM26;
  OM37 = OM16*S7+OM36*C7;
  OMp17 = C7*(OMp16-qd[7]*OM36)-S7*(OMp36+qd[7]*OM16);
  OMp27 = qdd[7]+OMp26;
  OMp37 = C7*(OMp36+qd[7]*OM16)+S7*(OMp16-qd[7]*OM36);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BETA27 = BS27-OMp37;
  BETA87 = BS67+OMp17;
  ALPHA17 = C7*(ALPHA16+BETA26*s->dpt[2][2])-S7*(ALPHA35+BETA86*s->dpt[2][2]);
  ALPHA27 = ALPHA26+BS56*s->dpt[2][2];
  ALPHA37 = C7*(ALPHA35+BETA86*s->dpt[2][2])+S7*(ALPHA16+BETA26*s->dpt[2][2]);
  OM18 = qd[8]+OM17;
  OM28 = OM27*C8+OM37*S8;
  OM38 = -(OM27*S8-OM37*C8);
  OMp18 = qdd[8]+OMp17;
  OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27);
  OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  BS98 = -(OM18*OM18+OM28*OM28);
  BETA38 = BS38+OMp28;
  BETA68 = BS68-OMp18;
  ALPHA18 = ALPHA17+BETA27*s->dpt[2][6];
  ALPHA28 = C8*(ALPHA27+BS57*s->dpt[2][6])+S8*(ALPHA37+BETA87*s->dpt[2][6]);
  ALPHA38 = C8*(ALPHA37+BETA87*s->dpt[2][6])-S8*(ALPHA27+BS57*s->dpt[2][6]);
  OM19 = OM18*C9+OM28*S9;
  OM29 = -(OM18*S9-OM28*C9);
  OM39 = qd[9]+OM38;
  OMp19 = C9*(OMp18+qd[9]*OM28)+S9*(OMp28-qd[9]*OM18);
  OMp29 = C9*(OMp28-qd[9]*OM18)-S9*(OMp18+qd[9]*OM28);
  OMp39 = qdd[9]+OMp38;
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BETA39 = BS39+OMp29;
  BETA69 = BS69-OMp19;
  ALPHA19 = C9*(ALPHA18+BETA38*s->dpt[3][8])+S9*(ALPHA28+BETA68*s->dpt[3][8]);
  ALPHA29 = C9*(ALPHA28+BETA68*s->dpt[3][8])-S9*(ALPHA18+BETA38*s->dpt[3][8]);
  ALPHA39 = ALPHA38+BS98*s->dpt[3][8];
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd[10]+OM29;
  OM310 = OM19*S10+OM39*C10;
  OMp110 = C10*(OMp19-qd[10]*OM39)-S10*(OMp39+qd[10]*OM19);
  OMp210 = qdd[10]+OMp29;
  OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  BS910 = -(OM110*OM110+OM210*OM210);
  BETA310 = BS310+OMp210;
  BETA610 = BS610-OMp110;
  ALPHA110 = C10*(ALPHA19+BETA39*s->dpt[3][11])-S10*(ALPHA39+BS99*s->dpt[3][11]);
  ALPHA210 = ALPHA29+BETA69*s->dpt[3][11];
  ALPHA310 = C10*(ALPHA39+BS99*s->dpt[3][11])+S10*(ALPHA19+BETA39*s->dpt[3][11]);
  OM111 = qd[11]+OM110;
  OM211 = OM210*C11+OM310*S11;
  OM311 = -(OM210*S11-OM310*C11);
  OMp111 = qdd[11]+OMp110;
  OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210);
  OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS611 = OM211*OM311;
  ALPHA111 = ALPHA110+BETA310*s->dpt[3][14];
  ALPHA211 = C11*(ALPHA210+BETA610*s->dpt[3][14])+S11*(ALPHA310+BS910*s->dpt[3][14]);
  ALPHA311 = C11*(ALPHA310+BS910*s->dpt[3][14])-S11*(ALPHA210+BETA610*s->dpt[3][14]);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111);
  OMp212 = qdd[12]+OMp211;
  OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311);
  BS112 = -(OM212*OM212+OM312*OM312);
  BS312 = OM112*OM312;
  BS912 = -(OM112*OM112+OM212*OM212);
  BETA312 = BS312+OMp212;
  BETA412 = OMp312+OM112*OM212;
  BETA612 = OM212*OM312-OMp112;
  BETA712 = BS312-OMp212;
  ALPHA112 = ALPHA111*C12-ALPHA311*S12;
  ALPHA312 = ALPHA111*S12+ALPHA311*C12;
  OM113 = OM112*C13-OM312*S13;
  OM213 = qd[13]+OM212;
  OM313 = OM112*S13+OM312*C13;
  OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112);
  OMp213 = qdd[13]+OMp212;
  OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312);
  BS313 = OM113*OM313;
  OM114 = OM16*C14-OM36*S14;
  OM214 = qd[14]+OM26;
  OM314 = OM16*S14+OM36*C14;
  OMp114 = C14*(OMp16-qd[14]*OM36)-S14*(OMp36+qd[14]*OM16);
  OMp214 = qdd[14]+OMp26;
  OMp314 = C14*(OMp36+qd[14]*OM16)+S14*(OMp16-qd[14]*OM36);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS514 = -(OM114*OM114+OM314*OM314);
  BS614 = OM214*OM314;
  BETA214 = BS214-OMp314;
  BETA814 = BS614+OMp114;
  ALPHA114 = C14*(ALPHA16+BETA26*s->dpt[2][3])-S14*(ALPHA35+BETA86*s->dpt[2][3]);
  ALPHA214 = ALPHA26+BS56*s->dpt[2][3];
  ALPHA314 = C14*(ALPHA35+BETA86*s->dpt[2][3])+S14*(ALPHA16+BETA26*s->dpt[2][3]);
  OM115 = qd[15]+OM114;
  OM215 = OM214*C15+OM314*S15;
  OM315 = -(OM214*S15-OM314*C15);
  OMp115 = qdd[15]+OMp114;
  OMp215 = C15*(OMp214+qd[15]*OM314)+S15*(OMp314-qd[15]*OM214);
  OMp315 = C15*(OMp314-qd[15]*OM214)-S15*(OMp214+qd[15]*OM314);
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS615 = OM215*OM315;
  BS915 = -(OM115*OM115+OM215*OM215);
  BETA315 = BS315+OMp215;
  BETA615 = BS615-OMp115;
  ALPHA115 = ALPHA114+BETA214*s->dpt[2][22];
  ALPHA215 = C15*(ALPHA214+BS514*s->dpt[2][22])+S15*(ALPHA314+BETA814*s->dpt[2][22]);
  ALPHA315 = C15*(ALPHA314+BETA814*s->dpt[2][22])-S15*(ALPHA214+BS514*s->dpt[2][22]);
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]+OM315;
  OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115);
  OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215);
  OMp316 = qdd[16]+OMp315;
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BETA316 = BS316+OMp216;
  BETA616 = BS616-OMp116;
  ALPHA116 = C16*(ALPHA115+BETA315*s->dpt[3][24])+S16*(ALPHA215+BETA615*s->dpt[3][24]);
  ALPHA216 = C16*(ALPHA215+BETA615*s->dpt[3][24])-S16*(ALPHA115+BETA315*s->dpt[3][24]);
  ALPHA316 = ALPHA315+BS915*s->dpt[3][24];
  OM117 = OM116*C17-OM316*S17;
  OM217 = qd[17]+OM216;
  OM317 = OM116*S17+OM316*C17;
  OMp117 = C17*(OMp116-qd[17]*OM316)-S17*(OMp316+qd[17]*OM116);
  OMp217 = qdd[17]+OMp216;
  OMp317 = C17*(OMp316+qd[17]*OM116)+S17*(OMp116-qd[17]*OM316);
  BS217 = OM117*OM217;
  BS317 = OM117*OM317;
  BS617 = OM217*OM317;
  BS917 = -(OM117*OM117+OM217*OM217);
  BETA317 = BS317+OMp217;
  BETA617 = BS617-OMp117;
  ALPHA117 = C17*(ALPHA116+BETA316*s->dpt[3][27])-S17*(ALPHA316+BS916*s->dpt[3][27]);
  ALPHA217 = ALPHA216+BETA616*s->dpt[3][27];
  ALPHA317 = C17*(ALPHA316+BS916*s->dpt[3][27])+S17*(ALPHA116+BETA316*s->dpt[3][27]);
  OM118 = qd[18]+OM117;
  OM218 = OM217*C18+OM317*S18;
  OM318 = -(OM217*S18-OM317*C18);
  OMp118 = qdd[18]+OMp117;
  OMp218 = C18*(OMp217+qd[18]*OM317)+S18*(OMp317-qd[18]*OM217);
  OMp318 = C18*(OMp317-qd[18]*OM217)-S18*(OMp217+qd[18]*OM317);
  BS218 = OM118*OM218;
  BS318 = OM118*OM318;
  BS618 = OM218*OM318;
  ALPHA118 = ALPHA117+BETA317*s->dpt[3][30];
  ALPHA218 = C18*(ALPHA217+BETA617*s->dpt[3][30])+S18*(ALPHA317+BS917*s->dpt[3][30]);
  ALPHA318 = C18*(ALPHA317+BS917*s->dpt[3][30])-S18*(ALPHA217+BETA617*s->dpt[3][30]);
  OM119 = OM118*C19-OM318*S19;
  OM219 = qd[19]+OM218;
  OM319 = OM118*S19+OM318*C19;
  OMp119 = C19*(OMp118-qd[19]*OM318)-S19*(OMp318+qd[19]*OM118);
  OMp219 = qdd[19]+OMp218;
  OMp319 = C19*(OMp318+qd[19]*OM118)+S19*(OMp118-qd[19]*OM318);
  BS119 = -(OM219*OM219+OM319*OM319);
  BS319 = OM119*OM319;
  BS919 = -(OM119*OM119+OM219*OM219);
  BETA319 = BS319+OMp219;
  BETA419 = OMp319+OM119*OM219;
  BETA619 = OM219*OM319-OMp119;
  BETA719 = BS319-OMp219;
  ALPHA119 = ALPHA118*C19-ALPHA318*S19;
  ALPHA319 = ALPHA118*S19+ALPHA318*C19;
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM219;
  OM320 = OM119*S20+OM319*C20;
  OMp120 = C20*(OMp119-qd[20]*OM319)-S20*(OMp319+qd[20]*OM119);
  OMp220 = qdd[20]+OMp219;
  OMp320 = C20*(OMp319+qd[20]*OM119)+S20*(OMp119-qd[20]*OM319);
  BS320 = OM120*OM320;
  OM121 = qd[21]+OM16;
  OM221 = OM26*C21+OM36*S21;
  OM321 = -(OM26*S21-OM36*C21);
  OMp121 = qdd[21]+OMp16;
  OMp221 = C21*(OMp26+qd[21]*OM36)+S21*(OMp36-qd[21]*OM26);
  OMp321 = C21*(OMp36-qd[21]*OM26)-S21*(OMp26+qd[21]*OM36);
  BS221 = OM121*OM221;
  BS321 = OM121*OM321;
  BS621 = OM221*OM321;
  ALPHA121 = ALPHA16+BETA36*s->dpt[3][4]+BS16*s->dpt[1][4];
  ALPHA221 = C21*(ALPHA26+BETA46*s->dpt[1][4]+BETA66*s->dpt[3][4])+S21*(ALPHA35+BETA76*s->dpt[1][4]+BS96*s->dpt[3][4]);
  ALPHA321 = C21*(ALPHA35+BETA76*s->dpt[1][4]+BS96*s->dpt[3][4])-S21*(ALPHA26+BETA46*s->dpt[1][4]+BETA66*s->dpt[3][4]);
  OM122 = OM121*C22-OM321*S22;
  OM222 = qd[22]+OM221;
  OM322 = OM121*S22+OM321*C22;
  OMp122 = C22*(OMp121-qd[22]*OM321)-S22*(OMp321+qd[22]*OM121);
  OMp222 = qdd[22]+OMp221;
  OMp322 = C22*(OMp321+qd[22]*OM121)+S22*(OMp121-qd[22]*OM321);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS622 = OM222*OM322;
  BS922 = -(OM122*OM122+OM222*OM222);
  BETA322 = BS322+OMp222;
  BETA622 = BS622-OMp122;
  ALPHA122 = ALPHA121*C22-ALPHA321*S22;
  ALPHA322 = ALPHA121*S22+ALPHA321*C22;
  OM123 = OM122*C23+OM222*S23;
  OM223 = -(OM122*S23-OM222*C23);
  OM323 = qd[23]+OM322;
  OMp123 = C23*(OMp122+qd[23]*OM222)+S23*(OMp222-qd[23]*OM122);
  OMp223 = C23*(OMp222-qd[23]*OM122)-S23*(OMp122+qd[23]*OM222);
  OMp323 = qdd[23]+OMp322;
  BS123 = -(OM223*OM223+OM323*OM323);
  BS223 = OM123*OM223;
  BS323 = OM123*OM323;
  BS523 = -(OM123*OM123+OM323*OM323);
  BS623 = OM223*OM323;
  BS923 = -(OM123*OM123+OM223*OM223);
  BETA223 = BS223-OMp323;
  BETA323 = BS323+OMp223;
  BETA423 = BS223+OMp323;
  BETA623 = BS623-OMp123;
  BETA723 = BS323-OMp223;
  BETA823 = BS623+OMp123;
  ALPHA123 = C23*(ALPHA122+BETA322*s->dpt[3][40])+S23*(ALPHA221+BETA622*s->dpt[3][40]);
  ALPHA223 = C23*(ALPHA221+BETA622*s->dpt[3][40])-S23*(ALPHA122+BETA322*s->dpt[3][40]);
  ALPHA323 = ALPHA322+BS922*s->dpt[3][40];
  OM124 = OM123*C24-OM323*S24;
  OM224 = qd[24]+OM223;
  OM324 = OM123*S24+OM323*C24;
  OMp124 = C24*(OMp123-qd[24]*OM323)-S24*(OMp323+qd[24]*OM123);
  OMp224 = qdd[24]+OMp223;
  OMp324 = C24*(OMp323+qd[24]*OM123)+S24*(OMp123-qd[24]*OM323);
  BS224 = OM124*OM224;
  BS324 = OM124*OM324;
  BS524 = -(OM124*OM124+OM324*OM324);
  BS624 = OM224*OM324;
  BETA224 = BS224-OMp324;
  BETA824 = BS624+OMp124;
  ALPHA124 = C24*(ALPHA123+BETA223*s->dpt[2][44]+BETA323*s->dpt[3][44]+BS123*s->dpt[1][44])-S24*(ALPHA323+BETA723*
 s->dpt[1][44]+BETA823*s->dpt[2][44]+BS923*s->dpt[3][44]);
  ALPHA224 = ALPHA223+BETA423*s->dpt[1][44]+BETA623*s->dpt[3][44]+BS523*s->dpt[2][44];
  ALPHA324 = C24*(ALPHA323+BETA723*s->dpt[1][44]+BETA823*s->dpt[2][44]+BS923*s->dpt[3][44])+S24*(ALPHA123+BETA223*
 s->dpt[2][44]+BETA323*s->dpt[3][44]+BS123*s->dpt[1][44]);
  OM125 = qd[25]+OM124;
  OM225 = OM224*C25+OM324*S25;
  OM325 = -(OM224*S25-OM324*C25);
  OMp125 = qdd[25]+OMp124;
  OMp225 = C25*(OMp224+qd[25]*OM324)+S25*(OMp324-qd[25]*OM224);
  OMp325 = C25*(OMp324-qd[25]*OM224)-S25*(OMp224+qd[25]*OM324);
  BS225 = OM125*OM225;
  BS325 = OM125*OM325;
  BS625 = OM225*OM325;
  BS925 = -(OM125*OM125+OM225*OM225);
  BETA325 = BS325+OMp225;
  BETA625 = BS625-OMp125;
  ALPHA125 = ALPHA124+BETA224*s->dpt[2][47];
  ALPHA225 = C25*(ALPHA224+BS524*s->dpt[2][47])+S25*(ALPHA324+BETA824*s->dpt[2][47]);
  ALPHA325 = C25*(ALPHA324+BETA824*s->dpt[2][47])-S25*(ALPHA224+BS524*s->dpt[2][47]);
  OM126 = OM125*C26+OM225*S26;
  OM226 = -(OM125*S26-OM225*C26);
  OM326 = qd[26]+OM325;
  OMp126 = C26*(OMp125+qd[26]*OM225)+S26*(OMp225-qd[26]*OM125);
  OMp226 = C26*(OMp225-qd[26]*OM125)-S26*(OMp125+qd[26]*OM225);
  OMp326 = qdd[26]+OMp325;
  BS226 = OM126*OM226;
  BS326 = OM126*OM326;
  BS626 = OM226*OM326;
  BS926 = -(OM126*OM126+OM226*OM226);
  BETA326 = BS326+OMp226;
  BETA626 = BS626-OMp126;
  ALPHA126 = C26*(ALPHA125+BETA325*s->dpt[3][49])+S26*(ALPHA225+BETA625*s->dpt[3][49]);
  ALPHA226 = C26*(ALPHA225+BETA625*s->dpt[3][49])-S26*(ALPHA125+BETA325*s->dpt[3][49]);
  ALPHA326 = ALPHA325+BS925*s->dpt[3][49];
  OM127 = OM126*C27-OM326*S27;
  OM227 = qd[27]+OM226;
  OM327 = OM126*S27+OM326*C27;
  OMp127 = C27*(OMp126-qd[27]*OM326)-S27*(OMp326+qd[27]*OM126);
  OMp227 = qdd[27]+OMp226;
  OMp327 = C27*(OMp326+qd[27]*OM126)+S27*(OMp126-qd[27]*OM326);
  BS227 = OM127*OM227;
  BS327 = OM127*OM327;
  BS627 = OM227*OM327;
  OM128 = OM123*C28-OM323*S28;
  OM228 = qd[28]+OM223;
  OM328 = OM123*S28+OM323*C28;
  OMp128 = C28*(OMp123-qd[28]*OM323)-S28*(OMp323+qd[28]*OM123);
  OMp228 = qdd[28]+OMp223;
  OMp328 = C28*(OMp323+qd[28]*OM123)+S28*(OMp123-qd[28]*OM323);
  BS228 = OM128*OM228;
  BS328 = OM128*OM328;
  BS528 = -(OM128*OM128+OM328*OM328);
  BS628 = OM228*OM328;
  BETA228 = BS228-OMp328;
  BETA828 = BS628+OMp128;
  ALPHA128 = C28*(ALPHA123+BETA223*s->dpt[2][45]+BETA323*s->dpt[3][45]+BS123*s->dpt[1][45])-S28*(ALPHA323+BETA723*
 s->dpt[1][45]+BETA823*s->dpt[2][45]+BS923*s->dpt[3][45]);
  ALPHA228 = ALPHA223+BETA423*s->dpt[1][45]+BETA623*s->dpt[3][45]+BS523*s->dpt[2][45];
  ALPHA328 = C28*(ALPHA323+BETA723*s->dpt[1][45]+BETA823*s->dpt[2][45]+BS923*s->dpt[3][45])+S28*(ALPHA123+BETA223*
 s->dpt[2][45]+BETA323*s->dpt[3][45]+BS123*s->dpt[1][45]);
  OM129 = qd[29]+OM128;
  OM229 = OM228*C29+OM328*S29;
  OM329 = -(OM228*S29-OM328*C29);
  OMp129 = qdd[29]+OMp128;
  OMp229 = C29*(OMp228+qd[29]*OM328)+S29*(OMp328-qd[29]*OM228);
  OMp329 = C29*(OMp328-qd[29]*OM228)-S29*(OMp228+qd[29]*OM328);
  BS229 = OM129*OM229;
  BS329 = OM129*OM329;
  BS629 = OM229*OM329;
  BS929 = -(OM129*OM129+OM229*OM229);
  BETA329 = BS329+OMp229;
  BETA629 = BS629-OMp129;
  ALPHA129 = ALPHA128+BETA228*s->dpt[2][56];
  ALPHA229 = C29*(ALPHA228+BS528*s->dpt[2][56])+S29*(ALPHA328+BETA828*s->dpt[2][56]);
  ALPHA329 = C29*(ALPHA328+BETA828*s->dpt[2][56])-S29*(ALPHA228+BS528*s->dpt[2][56]);
  OM130 = OM129*C30+OM229*S30;
  OM230 = -(OM129*S30-OM229*C30);
  OM330 = qd[30]+OM329;
  OMp130 = C30*(OMp129+qd[30]*OM229)+S30*(OMp229-qd[30]*OM129);
  OMp230 = C30*(OMp229-qd[30]*OM129)-S30*(OMp129+qd[30]*OM229);
  OMp330 = qdd[30]+OMp329;
  BS230 = OM130*OM230;
  BS330 = OM130*OM330;
  BS630 = OM230*OM330;
  BS930 = -(OM130*OM130+OM230*OM230);
  BETA330 = BS330+OMp230;
  BETA630 = BS630-OMp130;
  ALPHA130 = C30*(ALPHA129+BETA329*s->dpt[3][58])+S30*(ALPHA229+BETA629*s->dpt[3][58]);
  ALPHA230 = C30*(ALPHA229+BETA629*s->dpt[3][58])-S30*(ALPHA129+BETA329*s->dpt[3][58]);
  ALPHA330 = ALPHA329+BS929*s->dpt[3][58];
  OM131 = OM130*C31-OM330*S31;
  OM231 = qd[31]+OM230;
  OM331 = OM130*S31+OM330*C31;
  OMp131 = C31*(OMp130-qd[31]*OM330)-S31*(OMp330+qd[31]*OM130);
  OMp231 = qdd[31]+OMp230;
  OMp331 = C31*(OMp330+qd[31]*OM130)+S31*(OMp130-qd[31]*OM330);
  BS231 = OM131*OM231;
  BS331 = OM131*OM331;
  BS631 = OM231*OM331;
 
// Backward Dynamics 

  Fs131 = -(s->frc[1][31]+s->m[31]*(s->l[1][31]*(OM231*OM231+OM331*OM331)-s->l[2][31]*(BS231-OMp331)-s->l[3][31]*(BS331+
 OMp231)-C31*(ALPHA130+BETA330*s->dpt[3][61])+S31*(ALPHA330+BS930*s->dpt[3][61])));
  Fs231 = -(s->frc[2][31]-s->m[31]*(ALPHA230+BETA630*s->dpt[3][61]+s->l[1][31]*(BS231+OMp331)-s->l[2][31]*(OM131*OM131+
 OM331*OM331)+s->l[3][31]*(BS631-OMp131)));
  Fs331 = -(s->frc[3][31]-s->m[31]*(s->l[1][31]*(BS331-OMp231)+s->l[2][31]*(BS631+OMp131)-s->l[3][31]*(OM131*OM131+OM231
 *OM231)+C31*(ALPHA330+BS930*s->dpt[3][61])+S31*(ALPHA130+BETA330*s->dpt[3][61])));
  Cq131 = -(s->trq[1][31]-s->In[1][31]*OMp131-s->In[2][31]*OMp231+Fs231*s->l[3][31]-Fs331*s->l[2][31]-OM231*(
 s->In[6][31]*OM231+s->In[9][31]*OM331)+OM331*(s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331));
  Cq231 = -(s->trq[2][31]-s->In[2][31]*OMp131-s->In[5][31]*OMp231-s->In[6][31]*OMp331-Fs131*s->l[3][31]+Fs331*
 s->l[1][31]+OM131*(s->In[6][31]*OM231+s->In[9][31]*OM331)-OM331*(s->In[1][31]*OM131+s->In[2][31]*OM231));
  Cq331 = -(s->trq[3][31]-s->In[6][31]*OMp231-s->In[9][31]*OMp331+Fs131*s->l[2][31]-Fs231*s->l[1][31]-OM131*(
 s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331)+OM231*(s->In[1][31]*OM131+s->In[2][31]*OM231));
  Fs130 = -(s->frc[1][30]-s->m[30]*(ALPHA130+BETA330*s->l[3][30]-s->l[1][30]*(OM230*OM230+OM330*OM330)+s->l[2][30]*(
 BS230-OMp330)));
  Fs230 = -(s->frc[2][30]-s->m[30]*(ALPHA230+BETA630*s->l[3][30]+s->l[1][30]*(BS230+OMp330)-s->l[2][30]*(OM130*OM130+
 OM330*OM330)));
  Fs330 = -(s->frc[3][30]-s->m[30]*(ALPHA330+BS930*s->l[3][30]+s->l[1][30]*(BS330-OMp230)+s->l[2][30]*(BS630+OMp130)));
  Fq130 = Fs130+Fs131*C31+Fs331*S31;
  Fq230 = Fs230+Fs231;
  Cq130 = -(s->trq[1][30]-s->In[1][30]*OMp130-s->In[3][30]*OMp330-Cq131*C31-Cq331*S31+Fs230*s->l[3][30]+Fs231*
 s->dpt[3][61]-Fs330*s->l[2][30]-OM230*(s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)+OM330*(s->In[5][30]*OM230+
 s->In[6][30]*OM330));
  Cq230 = -(s->trq[2][30]-Cq231-s->In[5][30]*OMp230-s->In[6][30]*OMp330-Fs130*s->l[3][30]+Fs330*s->l[1][30]+OM130*(
 s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)-OM330*(s->In[1][30]*OM130+s->In[3][30]*OM330)-s->dpt[3][61]*(Fs131
 *C31+Fs331*S31));
  Cq330 = -(s->trq[3][30]-s->In[3][30]*OMp130-s->In[6][30]*OMp230-s->In[9][30]*OMp330+Cq131*S31-Cq331*C31+Fs130*
 s->l[2][30]-Fs230*s->l[1][30]-OM130*(s->In[5][30]*OM230+s->In[6][30]*OM330)+OM230*(s->In[1][30]*OM130+s->In[3][30]*OM330));
  Fs129 = -(s->frc[1][29]-s->m[29]*(ALPHA129+BETA329*s->l[3][29]-s->l[1][29]*(OM229*OM229+OM329*OM329)+s->l[2][29]*(
 BS229-OMp329)));
  Fs229 = -(s->frc[2][29]-s->m[29]*(ALPHA229+BETA629*s->l[3][29]+s->l[1][29]*(BS229+OMp329)-s->l[2][29]*(OM129*OM129+
 OM329*OM329)));
  Fs329 = -(s->frc[3][29]-s->m[29]*(ALPHA329+BS929*s->l[3][29]+s->l[1][29]*(BS329-OMp229)+s->l[2][29]*(BS629+OMp129)));
  Fq129 = Fs129+Fq130*C30-Fq230*S30;
  Fq229 = Fs229+Fq130*S30+Fq230*C30;
  Fq329 = Fs329+Fs330-Fs131*S31+Fs331*C31;
  Cq129 = -(s->trq[1][29]-s->In[1][29]*OMp129-s->In[2][29]*OMp229-s->In[3][29]*OMp329-Cq130*C30+Cq230*S30+Fs229*
 s->l[3][29]-Fs329*s->l[2][29]-OM229*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)+OM329*(s->In[2][29]*OM129+
 s->In[5][29]*OM229+s->In[6][29]*OM329)+s->dpt[3][58]*(Fq130*S30+Fq230*C30));
  Cq229 = -(s->trq[2][29]-s->In[2][29]*OMp129-s->In[5][29]*OMp229-s->In[6][29]*OMp329-Cq130*S30-Cq230*C30-Fs129*
 s->l[3][29]+Fs329*s->l[1][29]+OM129*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)-OM329*(s->In[1][29]*OM129+
 s->In[2][29]*OM229+s->In[3][29]*OM329)-s->dpt[3][58]*(Fq130*C30-Fq230*S30));
  Cq329 = -(s->trq[3][29]-Cq330-s->In[3][29]*OMp129-s->In[6][29]*OMp229-s->In[9][29]*OMp329+Fs129*s->l[2][29]-Fs229*
 s->l[1][29]-OM129*(s->In[2][29]*OM129+s->In[5][29]*OM229+s->In[6][29]*OM329)+OM229*(s->In[1][29]*OM129+s->In[2][29]*OM229+
 s->In[3][29]*OM329));
  Fs128 = -(s->frc[1][28]-s->m[28]*(ALPHA128+BETA228*s->l[2][28]-s->l[1][28]*(OM228*OM228+OM328*OM328)+s->l[3][28]*(
 BS328+OMp228)));
  Fs228 = -(s->frc[2][28]-s->m[28]*(ALPHA228+BS528*s->l[2][28]+s->l[1][28]*(BS228+OMp328)+s->l[3][28]*(BS628-OMp128)));
  Fs328 = -(s->frc[3][28]-s->m[28]*(ALPHA328+BETA828*s->l[2][28]+s->l[1][28]*(BS328-OMp228)-s->l[3][28]*(OM128*OM128+
 OM228*OM228)));
  Fq128 = Fq129+Fs128;
  Fq228 = Fs228+Fq229*C29-Fq329*S29;
  Fq328 = Fs328+Fq229*S29+Fq329*C29;
  Cq128 = -(s->trq[1][28]-Cq129-s->In[1][28]*OMp128+Fs228*s->l[3][28]-Fs328*s->l[2][28]+OM228*OM328*(s->In[5][28]-
 s->In[9][28])-s->dpt[2][56]*(Fq229*S29+Fq329*C29));
  Cq228 = -(s->trq[2][28]-s->In[5][28]*OMp228-Cq229*C29+Cq329*S29-Fs128*s->l[3][28]+Fs328*s->l[1][28]-OM128*OM328*(
 s->In[1][28]-s->In[9][28]));
  Cq328 = -(s->trq[3][28]-s->In[9][28]*OMp328-Cq229*S29-Cq329*C29+Fq129*s->dpt[2][56]+Fs128*s->l[2][28]-Fs228*
 s->l[1][28]+OM128*OM228*(s->In[1][28]-s->In[5][28]));
  Fs127 = -(s->frc[1][27]+s->m[27]*(s->l[1][27]*(OM227*OM227+OM327*OM327)-s->l[2][27]*(BS227-OMp327)-s->l[3][27]*(BS327+
 OMp227)-C27*(ALPHA126+BETA326*s->dpt[3][52])+S27*(ALPHA326+BS926*s->dpt[3][52])));
  Fs227 = -(s->frc[2][27]-s->m[27]*(ALPHA226+BETA626*s->dpt[3][52]+s->l[1][27]*(BS227+OMp327)-s->l[2][27]*(OM127*OM127+
 OM327*OM327)+s->l[3][27]*(BS627-OMp127)));
  Fs327 = -(s->frc[3][27]-s->m[27]*(s->l[1][27]*(BS327-OMp227)+s->l[2][27]*(BS627+OMp127)-s->l[3][27]*(OM127*OM127+OM227
 *OM227)+C27*(ALPHA326+BS926*s->dpt[3][52])+S27*(ALPHA126+BETA326*s->dpt[3][52])));
  Cq127 = -(s->trq[1][27]-s->In[1][27]*OMp127+Fs227*s->l[3][27]-Fs327*s->l[2][27]+OM227*OM327*(s->In[5][27]-s->In[9][27]
 ));
  Cq227 = -(s->trq[2][27]-s->In[5][27]*OMp227-Fs127*s->l[3][27]+Fs327*s->l[1][27]-OM127*OM327*(s->In[1][27]-s->In[9][27]
 ));
  Cq327 = -(s->trq[3][27]-s->In[9][27]*OMp327+Fs127*s->l[2][27]-Fs227*s->l[1][27]+OM127*OM227*(s->In[1][27]-s->In[5][27]
 ));
  Fs126 = -(s->frc[1][26]-s->m[26]*(ALPHA126+BETA326*s->l[3][26]-s->l[1][26]*(OM226*OM226+OM326*OM326)+s->l[2][26]*(
 BS226-OMp326)));
  Fs226 = -(s->frc[2][26]-s->m[26]*(ALPHA226+BETA626*s->l[3][26]+s->l[1][26]*(BS226+OMp326)-s->l[2][26]*(OM126*OM126+
 OM326*OM326)));
  Fs326 = -(s->frc[3][26]-s->m[26]*(ALPHA326+BS926*s->l[3][26]+s->l[1][26]*(BS326-OMp226)+s->l[2][26]*(BS626+OMp126)));
  Fq126 = Fs126+Fs127*C27+Fs327*S27;
  Fq226 = Fs226+Fs227;
  Cq126 = -(s->trq[1][26]-s->In[1][26]*OMp126-s->In[2][26]*OMp226-s->In[3][26]*OMp326-Cq127*C27-Cq327*S27+Fs226*
 s->l[3][26]+Fs227*s->dpt[3][52]-Fs326*s->l[2][26]-OM226*(s->In[3][26]*OM126+s->In[9][26]*OM326)+OM326*(s->In[2][26]*OM126+
 s->In[5][26]*OM226));
  Cq226 = -(s->trq[2][26]-Cq227-s->In[2][26]*OMp126-s->In[5][26]*OMp226-Fs126*s->l[3][26]+Fs326*s->l[1][26]+OM126*(
 s->In[3][26]*OM126+s->In[9][26]*OM326)-OM326*(s->In[1][26]*OM126+s->In[2][26]*OM226+s->In[3][26]*OM326)-s->dpt[3][52]*(Fs127
 *C27+Fs327*S27));
  Cq326 = -(s->trq[3][26]-s->In[3][26]*OMp126-s->In[9][26]*OMp326+Cq127*S27-Cq327*C27+Fs126*s->l[2][26]-Fs226*
 s->l[1][26]-OM126*(s->In[2][26]*OM126+s->In[5][26]*OM226)+OM226*(s->In[1][26]*OM126+s->In[2][26]*OM226+s->In[3][26]*OM326));
  Fs125 = -(s->frc[1][25]-s->m[25]*(ALPHA125+BETA325*s->l[3][25]-s->l[1][25]*(OM225*OM225+OM325*OM325)+s->l[2][25]*(
 BS225-OMp325)));
  Fs225 = -(s->frc[2][25]-s->m[25]*(ALPHA225+BETA625*s->l[3][25]+s->l[1][25]*(BS225+OMp325)-s->l[2][25]*(OM125*OM125+
 OM325*OM325)));
  Fs325 = -(s->frc[3][25]-s->m[25]*(ALPHA325+BS925*s->l[3][25]+s->l[1][25]*(BS325-OMp225)+s->l[2][25]*(BS625+OMp125)));
  Fq125 = Fs125+Fq126*C26-Fq226*S26;
  Fq225 = Fs225+Fq126*S26+Fq226*C26;
  Fq325 = Fs325+Fs326-Fs127*S27+Fs327*C27;
  Cq125 = -(s->trq[1][25]-s->In[1][25]*OMp125-s->In[3][25]*OMp325-Cq126*C26+Cq226*S26+Fs225*s->l[3][25]-Fs325*
 s->l[2][25]-OM225*(s->In[3][25]*OM125-s->In[5][25]*OM325+s->In[9][25]*OM325)+s->dpt[3][49]*(Fq126*S26+Fq226*C26));
  Cq225 = -(s->trq[2][25]-s->In[5][25]*OMp225-Cq126*S26-Cq226*C26-Fs125*s->l[3][25]+Fs325*s->l[1][25]+OM125*(
 s->In[3][25]*OM125+s->In[9][25]*OM325)-OM325*(s->In[1][25]*OM125+s->In[3][25]*OM325)-s->dpt[3][49]*(Fq126*C26-Fq226*S26));
  Cq325 = -(s->trq[3][25]-Cq326-s->In[3][25]*OMp125-s->In[9][25]*OMp325+Fs125*s->l[2][25]-Fs225*s->l[1][25]+OM225*(
 s->In[1][25]*OM125+s->In[3][25]*OM325-s->In[5][25]*OM125));
  Fs124 = -(s->frc[1][24]-s->m[24]*(ALPHA124+BETA224*s->l[2][24]-s->l[1][24]*(OM224*OM224+OM324*OM324)+s->l[3][24]*(
 BS324+OMp224)));
  Fs224 = -(s->frc[2][24]-s->m[24]*(ALPHA224+BS524*s->l[2][24]+s->l[1][24]*(BS224+OMp324)+s->l[3][24]*(BS624-OMp124)));
  Fs324 = -(s->frc[3][24]-s->m[24]*(ALPHA324+BETA824*s->l[2][24]+s->l[1][24]*(BS324-OMp224)-s->l[3][24]*(OM124*OM124+
 OM224*OM224)));
  Fq124 = Fq125+Fs124;
  Fq224 = Fs224+Fq225*C25-Fq325*S25;
  Fq324 = Fs324+Fq225*S25+Fq325*C25;
  Cq124 = -(s->trq[1][24]-Cq125-s->In[1][24]*OMp124-s->In[2][24]*OMp224+Fs224*s->l[3][24]-Fs324*s->l[2][24]-OM224*(
 s->In[6][24]*OM224+s->In[9][24]*OM324)+OM324*(s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324)-s->dpt[2][47]*(Fq225
 *S25+Fq325*C25));
  Cq224 = -(s->trq[2][24]-s->In[2][24]*OMp124-s->In[5][24]*OMp224-s->In[6][24]*OMp324-Cq225*C25+Cq325*S25-Fs124*
 s->l[3][24]+Fs324*s->l[1][24]+OM124*(s->In[6][24]*OM224+s->In[9][24]*OM324)-OM324*(s->In[1][24]*OM124+s->In[2][24]*OM224));
  Cq324 = -(s->trq[3][24]-s->In[6][24]*OMp224-s->In[9][24]*OMp324-Cq225*S25-Cq325*C25+Fq125*s->dpt[2][47]+Fs124*
 s->l[2][24]-Fs224*s->l[1][24]-OM124*(s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324)+OM224*(s->In[1][24]*OM124+
 s->In[2][24]*OM224));
  Fs123 = -(s->frc[1][23]-s->m[23]*(ALPHA123+BETA223*s->l[2][23]+BETA323*s->l[3][23]+BS123*s->l[1][23]));
  Fs223 = -(s->frc[2][23]-s->m[23]*(ALPHA223+BETA423*s->l[1][23]+BETA623*s->l[3][23]+BS523*s->l[2][23]));
  Fs323 = -(s->frc[3][23]-s->m[23]*(ALPHA323+BETA723*s->l[1][23]+BETA823*s->l[2][23]+BS923*s->l[3][23]));
  Fq123 = Fs123+Fq124*C24+Fq128*C28+Fq324*S24+Fq328*S28;
  Fq223 = Fq224+Fq228+Fs223;
  Cq123 = -(s->trq[1][23]-s->In[1][23]*OMp123-Cq124*C24-Cq128*C28-Cq324*S24-Cq328*S28+Fq224*s->dpt[3][44]+Fq228*
 s->dpt[3][45]+Fs223*s->l[3][23]-Fs323*s->l[2][23]-OM223*(s->In[6][23]*OM223+s->In[9][23]*OM323)+OM323*(s->In[5][23]*OM223+
 s->In[6][23]*OM323)+s->dpt[2][44]*(Fq124*S24-Fq324*C24)+s->dpt[2][45]*(Fq128*S28-Fq328*C28));
  Cq223 = -(s->trq[2][23]-Cq224-Cq228-s->In[5][23]*OMp223-s->In[6][23]*OMp323-Fs123*s->l[3][23]+Fs323*s->l[1][23]-OM123*
 (s->In[1][23]*OM323-s->In[6][23]*OM223-s->In[9][23]*OM323)-s->dpt[1][44]*(Fq124*S24-Fq324*C24)-s->dpt[1][45]*(Fq128*S28-
 Fq328*C28)-s->dpt[3][44]*(Fq124*C24+Fq324*S24)-s->dpt[3][45]*(Fq128*C28+Fq328*S28));
  Cq323 = -(s->trq[3][23]-s->In[6][23]*OMp223-s->In[9][23]*OMp323+Cq124*S24+Cq128*S28-Cq324*C24-Cq328*C28-Fq224*
 s->dpt[1][44]-Fq228*s->dpt[1][45]+Fs123*s->l[2][23]-Fs223*s->l[1][23]+OM123*(s->In[1][23]*OM223-s->In[5][23]*OM223-
 s->In[6][23]*OM323)+s->dpt[2][44]*(Fq124*C24+Fq324*S24)+s->dpt[2][45]*(Fq128*C28+Fq328*S28));
  Fs122 = -(s->frc[1][22]-s->m[22]*(ALPHA122+BETA322*s->l[3][22]-s->l[1][22]*(OM222*OM222+OM322*OM322)+s->l[2][22]*(
 BS222-OMp322)));
  Fs222 = -(s->frc[2][22]-s->m[22]*(ALPHA221+BETA622*s->l[3][22]+s->l[1][22]*(BS222+OMp322)-s->l[2][22]*(OM122*OM122+
 OM322*OM322)));
  Fs322 = -(s->frc[3][22]-s->m[22]*(ALPHA322+BS922*s->l[3][22]+s->l[1][22]*(BS322-OMp222)+s->l[2][22]*(BS622+OMp122)));
  Fq122 = Fs122+Fq123*C23-Fq223*S23;
  Fq322 = Fs322+Fs323-Fq124*S24-Fq128*S28+Fq324*C24+Fq328*C28;
  Cq122 = -(s->trq[1][22]-s->In[1][22]*OMp122-s->In[3][22]*OMp322-Cq123*C23+Cq223*S23+Fs222*s->l[3][22]-Fs322*
 s->l[2][22]-OM222*(s->In[3][22]*OM122-s->In[5][22]*OM322+s->In[9][22]*OM322)+s->dpt[3][40]*(Fq123*S23+Fq223*C23));
  Cq222 = -(s->trq[2][22]-s->In[5][22]*OMp222-Cq123*S23-Cq223*C23-Fs122*s->l[3][22]+Fs322*s->l[1][22]+OM122*(
 s->In[3][22]*OM122+s->In[9][22]*OM322)-OM322*(s->In[1][22]*OM122+s->In[3][22]*OM322)-s->dpt[3][40]*(Fq123*C23-Fq223*S23));
  Cq322 = -(s->trq[3][22]-Cq323-s->In[3][22]*OMp122-s->In[9][22]*OMp322+Fs122*s->l[2][22]-Fs222*s->l[1][22]+OM222*(
 s->In[1][22]*OM122+s->In[3][22]*OM322-s->In[5][22]*OM122));
  Fs121 = -(s->frc[1][21]-s->m[21]*(ALPHA121-s->l[1][21]*(OM221*OM221+OM321*OM321)+s->l[2][21]*(BS221-OMp321)+
 s->l[3][21]*(BS321+OMp221)));
  Fs221 = -(s->frc[2][21]-s->m[21]*(ALPHA221+s->l[1][21]*(BS221+OMp321)-s->l[2][21]*(OM121*OM121+OM321*OM321)+
 s->l[3][21]*(BS621-OMp121)));
  Fs321 = -(s->frc[3][21]-s->m[21]*(ALPHA321+s->l[1][21]*(BS321-OMp221)+s->l[2][21]*(BS621+OMp121)-s->l[3][21]*(OM121*
 OM121+OM221*OM221)));
  Fq121 = Fs121+Fq122*C22+Fq322*S22;
  Fq221 = Fs221+Fs222+Fq123*S23+Fq223*C23;
  Fq321 = Fs321-Fq122*S22+Fq322*C22;
  Cq121 = -(s->trq[1][21]-s->In[1][21]*OMp121-Cq122*C22-Cq322*S22+Fs221*s->l[3][21]-Fs321*s->l[2][21]-OM221*(
 s->In[6][21]*OM221+s->In[9][21]*OM321)+OM321*(s->In[5][21]*OM221+s->In[6][21]*OM321));
  Cq221 = -(s->trq[2][21]-Cq222-s->In[5][21]*OMp221-s->In[6][21]*OMp321-Fs121*s->l[3][21]+Fs321*s->l[1][21]-OM121*(
 s->In[1][21]*OM321-s->In[6][21]*OM221-s->In[9][21]*OM321));
  Cq321 = -(s->trq[3][21]-s->In[6][21]*OMp221-s->In[9][21]*OMp321+Cq122*S22-Cq322*C22+Fs121*s->l[2][21]-Fs221*
 s->l[1][21]+OM121*(s->In[1][21]*OM221-s->In[5][21]*OM221-s->In[6][21]*OM321));
  Fs120 = -(s->frc[1][20]+s->m[20]*(s->l[1][20]*(OM220*OM220+OM320*OM320)-s->l[3][20]*(BS320+OMp220)-C20*(ALPHA119+
 BETA319*s->dpt[3][36]+BS119*s->dpt[1][36])+S20*(ALPHA319+BETA719*s->dpt[1][36]+BS919*s->dpt[3][36])));
  Fs220 = -(s->frc[2][20]-s->m[20]*(ALPHA218+BETA419*s->dpt[1][36]+BETA619*s->dpt[3][36]+s->l[1][20]*(OMp320+OM120*OM220
 )-s->l[3][20]*(OMp120-OM220*OM320)));
  Fs320 = -(s->frc[3][20]-s->m[20]*(s->l[1][20]*(BS320-OMp220)-s->l[3][20]*(OM120*OM120+OM220*OM220)+C20*(ALPHA319+
 BETA719*s->dpt[1][36]+BS919*s->dpt[3][36])+S20*(ALPHA119+BETA319*s->dpt[3][36]+BS119*s->dpt[1][36])));
  Cq120 = -(s->trq[1][20]-s->In[1][20]*OMp120+Fs220*s->l[3][20]+OM220*OM320*(s->In[5][20]-s->In[9][20]));
  Cq220 = -(s->trq[2][20]-s->In[5][20]*OMp220-Fs120*s->l[3][20]+Fs320*s->l[1][20]-OM120*OM320*(s->In[1][20]-s->In[9][20]
 ));
  Cq320 = -(s->trq[3][20]-s->In[9][20]*OMp320-Fs220*s->l[1][20]+OM120*OM220*(s->In[1][20]-s->In[5][20]));
  Fs119 = -(s->frc[1][19]-s->m[19]*(ALPHA119+BETA319*s->l[3][19]+BS119*s->l[1][19]));
  Fs219 = -(s->frc[2][19]-s->m[19]*(ALPHA218+BETA419*s->l[1][19]+BETA619*s->l[3][19]));
  Fs319 = -(s->frc[3][19]-s->m[19]*(ALPHA319+BETA719*s->l[1][19]+BS919*s->l[3][19]));
  Fq119 = Fs119+Fs120*C20+Fs320*S20;
  Fq319 = Fs319-Fs120*S20+Fs320*C20;
  Cq119 = -(s->trq[1][19]-s->In[1][19]*OMp119-Cq120*C20-Cq320*S20+Fs219*s->l[3][19]+Fs220*s->dpt[3][36]+OM219*OM319*(
 s->In[5][19]-s->In[9][19]));
  Cq219 = -(s->trq[2][19]-Cq220-s->In[5][19]*OMp219-Fs119*s->l[3][19]+Fs319*s->l[1][19]-OM119*OM319*(s->In[1][19]-
 s->In[9][19])-s->dpt[1][36]*(Fs120*S20-Fs320*C20)-s->dpt[3][36]*(Fs120*C20+Fs320*S20));
  Cq319 = -(s->trq[3][19]-s->In[9][19]*OMp319+Cq120*S20-Cq320*C20-Fs219*s->l[1][19]-Fs220*s->dpt[1][36]+OM119*OM219*(
 s->In[1][19]-s->In[5][19]));
  Fs118 = -(s->frc[1][18]-s->m[18]*(ALPHA118-s->l[1][18]*(OM218*OM218+OM318*OM318)+s->l[2][18]*(BS218-OMp318)+
 s->l[3][18]*(BS318+OMp218)));
  Fs218 = -(s->frc[2][18]-s->m[18]*(ALPHA218+s->l[1][18]*(BS218+OMp318)-s->l[2][18]*(OM118*OM118+OM318*OM318)+
 s->l[3][18]*(BS618-OMp118)));
  Fs318 = -(s->frc[3][18]-s->m[18]*(ALPHA318+s->l[1][18]*(BS318-OMp218)+s->l[2][18]*(BS618+OMp118)-s->l[3][18]*(OM118*
 OM118+OM218*OM218)));
  Fq118 = Fs118+Fq119*C19+Fq319*S19;
  Fq218 = Fs218+Fs219+Fs220;
  Fq318 = Fs318-Fq119*S19+Fq319*C19;
  Cq118 = -(s->trq[1][18]-s->In[1][18]*OMp118-s->In[3][18]*OMp318-Cq119*C19-Cq319*S19+Fs218*s->l[3][18]-Fs318*
 s->l[2][18]-OM218*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)+OM318*(s->In[5][18]*OM218+s->In[6][18]*OM318));
  Cq218 = -(s->trq[2][18]-Cq219-s->In[5][18]*OMp218-s->In[6][18]*OMp318-Fs118*s->l[3][18]+Fs318*s->l[1][18]+OM118*(
 s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)-OM318*(s->In[1][18]*OM118+s->In[3][18]*OM318));
  Cq318 = -(s->trq[3][18]-s->In[3][18]*OMp118-s->In[6][18]*OMp218-s->In[9][18]*OMp318+Cq119*S19-Cq319*C19+Fs118*
 s->l[2][18]-Fs218*s->l[1][18]-OM118*(s->In[5][18]*OM218+s->In[6][18]*OM318)+OM218*(s->In[1][18]*OM118+s->In[3][18]*OM318));
  Fs117 = -(s->frc[1][17]-s->m[17]*(ALPHA117+BETA317*s->l[3][17]-s->l[1][17]*(OM217*OM217+OM317*OM317)+s->l[2][17]*(
 BS217-OMp317)));
  Fs217 = -(s->frc[2][17]-s->m[17]*(ALPHA217+BETA617*s->l[3][17]+s->l[1][17]*(BS217+OMp317)-s->l[2][17]*(OM117*OM117+
 OM317*OM317)));
  Fs317 = -(s->frc[3][17]-s->m[17]*(ALPHA317+BS917*s->l[3][17]+s->l[1][17]*(BS317-OMp217)+s->l[2][17]*(BS617+OMp117)));
  Fq117 = Fq118+Fs117;
  Fq217 = Fs217+Fq218*C18-Fq318*S18;
  Fq317 = Fs317+Fq218*S18+Fq318*C18;
  Cq117 = -(s->trq[1][17]-Cq118-s->In[1][17]*OMp117-s->In[3][17]*OMp317+Fs217*s->l[3][17]-Fs317*s->l[2][17]-OM217*(
 s->In[3][17]*OM117-s->In[5][17]*OM317+s->In[9][17]*OM317)+s->dpt[3][30]*(Fq218*C18-Fq318*S18));
  Cq217 = -(s->trq[2][17]-s->In[5][17]*OMp217-Cq218*C18+Cq318*S18-Fq118*s->dpt[3][30]-Fs117*s->l[3][17]+Fs317*
 s->l[1][17]+OM117*(s->In[3][17]*OM117+s->In[9][17]*OM317)-OM317*(s->In[1][17]*OM117+s->In[3][17]*OM317));
  Cq317 = -(s->trq[3][17]-s->In[3][17]*OMp117-s->In[9][17]*OMp317-Cq218*S18-Cq318*C18+Fs117*s->l[2][17]-Fs217*
 s->l[1][17]+OM217*(s->In[1][17]*OM117+s->In[3][17]*OM317-s->In[5][17]*OM117));
  Fs116 = -(s->frc[1][16]-s->m[16]*(ALPHA116+BETA316*s->l[3][16]-s->l[1][16]*(OM216*OM216+OM316*OM316)+s->l[2][16]*(
 BS216-OMp316)));
  Fs216 = -(s->frc[2][16]-s->m[16]*(ALPHA216+BETA616*s->l[3][16]+s->l[1][16]*(BS216+OMp316)-s->l[2][16]*(OM116*OM116+
 OM316*OM316)));
  Fs316 = -(s->frc[3][16]-s->m[16]*(ALPHA316+BS916*s->l[3][16]+s->l[1][16]*(BS316-OMp216)+s->l[2][16]*(BS616+OMp116)));
  Fq116 = Fs116+Fq117*C17+Fq317*S17;
  Fq216 = Fq217+Fs216;
  Cq116 = -(s->trq[1][16]-s->In[1][16]*OMp116-s->In[3][16]*OMp316-Cq117*C17-Cq317*S17+Fq217*s->dpt[3][27]+Fs216*
 s->l[3][16]-Fs316*s->l[2][16]-OM216*(s->In[3][16]*OM116-s->In[5][16]*OM316+s->In[9][16]*OM316));
  Cq216 = -(s->trq[2][16]-Cq217-s->In[5][16]*OMp216-Fs116*s->l[3][16]+Fs316*s->l[1][16]+OM116*(s->In[3][16]*OM116+
 s->In[9][16]*OM316)-OM316*(s->In[1][16]*OM116+s->In[3][16]*OM316)-s->dpt[3][27]*(Fq117*C17+Fq317*S17));
  Cq316 = -(s->trq[3][16]-s->In[3][16]*OMp116-s->In[9][16]*OMp316+Cq117*S17-Cq317*C17+Fs116*s->l[2][16]-Fs216*
 s->l[1][16]+OM216*(s->In[1][16]*OM116+s->In[3][16]*OM316-s->In[5][16]*OM116));
  Fs115 = -(s->frc[1][15]-s->m[15]*(ALPHA115+BETA315*s->l[3][15]-s->l[1][15]*(OM215*OM215+OM315*OM315)+s->l[2][15]*(
 BS215-OMp315)));
  Fs215 = -(s->frc[2][15]-s->m[15]*(ALPHA215+BETA615*s->l[3][15]+s->l[1][15]*(BS215+OMp315)-s->l[2][15]*(OM115*OM115+
 OM315*OM315)));
  Fs315 = -(s->frc[3][15]-s->m[15]*(ALPHA315+BS915*s->l[3][15]+s->l[1][15]*(BS315-OMp215)+s->l[2][15]*(BS615+OMp115)));
  Fq115 = Fs115+Fq116*C16-Fq216*S16;
  Fq215 = Fs215+Fq116*S16+Fq216*C16;
  Fq315 = Fs315+Fs316-Fq117*S17+Fq317*C17;
  Cq115 = -(s->trq[1][15]-s->In[1][15]*OMp115-s->In[2][15]*OMp215-Cq116*C16+Cq216*S16+Fs215*s->l[3][15]-Fs315*
 s->l[2][15]+OM315*(s->In[2][15]*OM115+s->In[5][15]*OM215-s->In[9][15]*OM215)+s->dpt[3][24]*(Fq116*S16+Fq216*C16));
  Cq215 = -(s->trq[2][15]-s->In[2][15]*OMp115-s->In[5][15]*OMp215-Cq116*S16-Cq216*C16-Fs115*s->l[3][15]+Fs315*
 s->l[1][15]-OM315*(s->In[1][15]*OM115+s->In[2][15]*OM215-s->In[9][15]*OM115)-s->dpt[3][24]*(Fq116*C16-Fq216*S16));
  Cq315 = -(s->trq[3][15]-Cq316-s->In[9][15]*OMp315+Fs115*s->l[2][15]-Fs215*s->l[1][15]-OM115*(s->In[2][15]*OM115+
 s->In[5][15]*OM215)+OM215*(s->In[1][15]*OM115+s->In[2][15]*OM215));
  Fs114 = -(s->frc[1][14]-s->m[14]*(ALPHA114+BETA214*s->l[2][14]-s->l[1][14]*(OM214*OM214+OM314*OM314)+s->l[3][14]*(
 BS314+OMp214)));
  Fs214 = -(s->frc[2][14]-s->m[14]*(ALPHA214+BS514*s->l[2][14]+s->l[1][14]*(BS214+OMp314)+s->l[3][14]*(BS614-OMp114)));
  Fs314 = -(s->frc[3][14]-s->m[14]*(ALPHA314+BETA814*s->l[2][14]+s->l[1][14]*(BS314-OMp214)-s->l[3][14]*(OM114*OM114+
 OM214*OM214)));
  Fq114 = Fq115+Fs114;
  Fq314 = Fs314+Fq215*S15+Fq315*C15;
  Cq114 = -(s->trq[1][14]-Cq115-s->In[1][14]*OMp114-s->In[2][14]*OMp214+Fs214*s->l[3][14]-Fs314*s->l[2][14]-OM214*(
 s->In[6][14]*OM214+s->In[9][14]*OM314)+OM314*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)-s->dpt[2][22]*(Fq215
 *S15+Fq315*C15));
  Cq214 = -(s->trq[2][14]-s->In[2][14]*OMp114-s->In[5][14]*OMp214-s->In[6][14]*OMp314-Cq215*C15+Cq315*S15-Fs114*
 s->l[3][14]+Fs314*s->l[1][14]+OM114*(s->In[6][14]*OM214+s->In[9][14]*OM314)-OM314*(s->In[1][14]*OM114+s->In[2][14]*OM214));
  Cq314 = -(s->trq[3][14]-s->In[6][14]*OMp214-s->In[9][14]*OMp314-Cq215*S15-Cq315*C15+Fq115*s->dpt[2][22]+Fs114*
 s->l[2][14]-Fs214*s->l[1][14]-OM114*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)+OM214*(s->In[1][14]*OM114+
 s->In[2][14]*OM214));
  Fs113 = -(s->frc[1][13]+s->m[13]*(s->l[1][13]*(OM213*OM213+OM313*OM313)-s->l[3][13]*(BS313+OMp213)-C13*(ALPHA112+
 BETA312*s->dpt[3][20]+BS112*s->dpt[1][20])+S13*(ALPHA312+BETA712*s->dpt[1][20]+BS912*s->dpt[3][20])));
  Fs213 = -(s->frc[2][13]-s->m[13]*(ALPHA211+BETA412*s->dpt[1][20]+BETA612*s->dpt[3][20]+s->l[1][13]*(OMp313+OM113*OM213
 )-s->l[3][13]*(OMp113-OM213*OM313)));
  Fs313 = -(s->frc[3][13]-s->m[13]*(s->l[1][13]*(BS313-OMp213)-s->l[3][13]*(OM113*OM113+OM213*OM213)+C13*(ALPHA312+
 BETA712*s->dpt[1][20]+BS912*s->dpt[3][20])+S13*(ALPHA112+BETA312*s->dpt[3][20]+BS112*s->dpt[1][20])));
  Cq113 = -(s->trq[1][13]-s->In[1][13]*OMp113+Fs213*s->l[3][13]+OM213*OM313*(s->In[5][13]-s->In[9][13]));
  Cq213 = -(s->trq[2][13]-s->In[5][13]*OMp213-Fs113*s->l[3][13]+Fs313*s->l[1][13]-OM113*OM313*(s->In[1][13]-s->In[9][13]
 ));
  Cq313 = -(s->trq[3][13]-s->In[9][13]*OMp313-Fs213*s->l[1][13]+OM113*OM213*(s->In[1][13]-s->In[5][13]));
  Fs112 = -(s->frc[1][12]-s->m[12]*(ALPHA112+BETA312*s->l[3][12]+BS112*s->l[1][12]));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA211+BETA412*s->l[1][12]+BETA612*s->l[3][12]));
  Fs312 = -(s->frc[3][12]-s->m[12]*(ALPHA312+BETA712*s->l[1][12]+BS912*s->l[3][12]));
  Fq112 = Fs112+Fs113*C13+Fs313*S13;
  Fq312 = Fs312-Fs113*S13+Fs313*C13;
  Cq112 = -(s->trq[1][12]-s->In[1][12]*OMp112-Cq113*C13-Cq313*S13+Fs212*s->l[3][12]+Fs213*s->dpt[3][20]+OM212*OM312*(
 s->In[5][12]-s->In[9][12]));
  Cq212 = -(s->trq[2][12]-Cq213-s->In[5][12]*OMp212-Fs112*s->l[3][12]+Fs312*s->l[1][12]-OM112*OM312*(s->In[1][12]-
 s->In[9][12])-s->dpt[1][20]*(Fs113*S13-Fs313*C13)-s->dpt[3][20]*(Fs113*C13+Fs313*S13));
  Cq312 = -(s->trq[3][12]-s->In[9][12]*OMp312+Cq113*S13-Cq313*C13-Fs212*s->l[1][12]-Fs213*s->dpt[1][20]+OM112*OM212*(
 s->In[1][12]-s->In[5][12]));
  Fs111 = -(s->frc[1][11]-s->m[11]*(ALPHA111-s->l[1][11]*(OM211*OM211+OM311*OM311)+s->l[2][11]*(BS211-OMp311)+
 s->l[3][11]*(BS311+OMp211)));
  Fs211 = -(s->frc[2][11]-s->m[11]*(ALPHA211+s->l[1][11]*(BS211+OMp311)-s->l[2][11]*(OM111*OM111+OM311*OM311)+
 s->l[3][11]*(BS611-OMp111)));
  Fs311 = -(s->frc[3][11]-s->m[11]*(ALPHA311+s->l[1][11]*(BS311-OMp211)+s->l[2][11]*(BS611+OMp111)-s->l[3][11]*(OM111*
 OM111+OM211*OM211)));
  Fq111 = Fs111+Fq112*C12+Fq312*S12;
  Fq211 = Fs211+Fs212+Fs213;
  Fq311 = Fs311-Fq112*S12+Fq312*C12;
  Cq111 = -(s->trq[1][11]-s->In[1][11]*OMp111-s->In[2][11]*OMp211-s->In[3][11]*OMp311-Cq112*C12-Cq312*S12+Fs211*
 s->l[3][11]-Fs311*s->l[2][11]-OM211*(s->In[3][11]*OM111+s->In[9][11]*OM311)+OM311*(s->In[2][11]*OM111+s->In[5][11]*OM211));
  Cq211 = -(s->trq[2][11]-Cq212-s->In[2][11]*OMp111-s->In[5][11]*OMp211-Fs111*s->l[3][11]+Fs311*s->l[1][11]+OM111*(
 s->In[3][11]*OM111+s->In[9][11]*OM311)-OM311*(s->In[1][11]*OM111+s->In[2][11]*OM211+s->In[3][11]*OM311));
  Cq311 = -(s->trq[3][11]-s->In[3][11]*OMp111-s->In[9][11]*OMp311+Cq112*S12-Cq312*C12+Fs111*s->l[2][11]-Fs211*
 s->l[1][11]-OM111*(s->In[2][11]*OM111+s->In[5][11]*OM211)+OM211*(s->In[1][11]*OM111+s->In[2][11]*OM211+s->In[3][11]*OM311));
  Fs110 = -(s->frc[1][10]-s->m[10]*(ALPHA110+BETA310*s->l[3][10]-s->l[1][10]*(OM210*OM210+OM310*OM310)+s->l[2][10]*(
 BS210-OMp310)));
  Fs210 = -(s->frc[2][10]-s->m[10]*(ALPHA210+BETA610*s->l[3][10]+s->l[1][10]*(BS210+OMp310)-s->l[2][10]*(OM110*OM110+
 OM310*OM310)));
  Fs310 = -(s->frc[3][10]-s->m[10]*(ALPHA310+BS910*s->l[3][10]+s->l[1][10]*(BS310-OMp210)+s->l[2][10]*(BS610+OMp110)));
  Fq110 = Fq111+Fs110;
  Fq210 = Fs210+Fq211*C11-Fq311*S11;
  Fq310 = Fs310+Fq211*S11+Fq311*C11;
  Cq110 = -(s->trq[1][10]-Cq111-s->In[1][10]*OMp110-s->In[2][10]*OMp210-s->In[3][10]*OMp310+Fs210*s->l[3][10]-Fs310*
 s->l[2][10]-OM210*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)+OM310*(s->In[2][10]*OM110+s->In[5][10]*OM210+
 s->In[6][10]*OM310)+s->dpt[3][14]*(Fq211*C11-Fq311*S11));
  Cq210 = -(s->trq[2][10]-s->In[2][10]*OMp110-s->In[5][10]*OMp210-s->In[6][10]*OMp310-Cq211*C11+Cq311*S11-Fq111*
 s->dpt[3][14]-Fs110*s->l[3][10]+Fs310*s->l[1][10]+OM110*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)-OM310*(
 s->In[1][10]*OM110+s->In[2][10]*OM210+s->In[3][10]*OM310));
  Cq310 = -(s->trq[3][10]-s->In[3][10]*OMp110-s->In[6][10]*OMp210-s->In[9][10]*OMp310-Cq211*S11-Cq311*C11+Fs110*
 s->l[2][10]-Fs210*s->l[1][10]-OM110*(s->In[2][10]*OM110+s->In[5][10]*OM210+s->In[6][10]*OM310)+OM210*(s->In[1][10]*OM110+
 s->In[2][10]*OM210+s->In[3][10]*OM310));
  Fs19 = -(s->frc[1][9]-s->m[9]*(ALPHA19+BETA39*s->l[3][9]-s->l[1][9]*(OM29*OM29+OM39*OM39)+s->l[2][9]*(BS29-OMp39)));
  Fs29 = -(s->frc[2][9]-s->m[9]*(ALPHA29+BETA69*s->l[3][9]+s->l[1][9]*(BS29+OMp39)-s->l[2][9]*(OM19*OM19+OM39*OM39)));
  Fs39 = -(s->frc[3][9]-s->m[9]*(ALPHA39+BS99*s->l[3][9]+s->l[1][9]*(BS39-OMp29)+s->l[2][9]*(BS69+OMp19)));
  Fq19 = Fs19+Fq110*C10+Fq310*S10;
  Fq29 = Fq210+Fs29;
  Cq19 = -(s->trq[1][9]-s->In[1][9]*OMp19-s->In[2][9]*OMp29-s->In[3][9]*OMp39-Cq110*C10-Cq310*S10+Fq210*s->dpt[3][11]+
 Fs29*s->l[3][9]-Fs39*s->l[2][9]-OM29*(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)+OM39*(s->In[2][9]*OM19+s->In[5][9]
 *OM29+s->In[6][9]*OM39));
  Cq29 = -(s->trq[2][9]-Cq210-s->In[2][9]*OMp19-s->In[5][9]*OMp29-s->In[6][9]*OMp39-Fs19*s->l[3][9]+Fs39*s->l[1][9]+OM19
 *(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)-OM39*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*OM39)-
 s->dpt[3][11]*(Fq110*C10+Fq310*S10));
  Cq39 = -(s->trq[3][9]-s->In[3][9]*OMp19-s->In[6][9]*OMp29-s->In[9][9]*OMp39+Cq110*S10-Cq310*C10+Fs19*s->l[2][9]-Fs29*
 s->l[1][9]-OM19*(s->In[2][9]*OM19+s->In[5][9]*OM29+s->In[6][9]*OM39)+OM29*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*
 OM39));
  Fs18 = -(s->frc[1][8]-s->m[8]*(ALPHA18+BETA38*s->l[3][8]-s->l[1][8]*(OM28*OM28+OM38*OM38)+s->l[2][8]*(BS28-OMp38)));
  Fs28 = -(s->frc[2][8]-s->m[8]*(ALPHA28+BETA68*s->l[3][8]+s->l[1][8]*(BS28+OMp38)-s->l[2][8]*(OM18*OM18+OM38*OM38)));
  Fs38 = -(s->frc[3][8]-s->m[8]*(ALPHA38+BS98*s->l[3][8]+s->l[1][8]*(BS38-OMp28)+s->l[2][8]*(BS68+OMp18)));
  Fq18 = Fs18+Fq19*C9-Fq29*S9;
  Fq28 = Fs28+Fq19*S9+Fq29*C9;
  Fq38 = Fs38+Fs39-Fq110*S10+Fq310*C10;
  Cq18 = -(s->trq[1][8]-s->In[1][8]*OMp18-Cq19*C9+Cq29*S9+Fs28*s->l[3][8]-Fs38*s->l[2][8]-OM28*(s->In[6][8]*OM28+
 s->In[9][8]*OM38)+OM38*(s->In[5][8]*OM28+s->In[6][8]*OM38)+s->dpt[3][8]*(Fq19*S9+Fq29*C9));
  Cq28 = -(s->trq[2][8]-s->In[5][8]*OMp28-s->In[6][8]*OMp38-Cq19*S9-Cq29*C9-Fs18*s->l[3][8]+Fs38*s->l[1][8]-OM18*(
 s->In[1][8]*OM38-s->In[6][8]*OM28-s->In[9][8]*OM38)-s->dpt[3][8]*(Fq19*C9-Fq29*S9));
  Cq38 = -(s->trq[3][8]-Cq39-s->In[6][8]*OMp28-s->In[9][8]*OMp38+Fs18*s->l[2][8]-Fs28*s->l[1][8]+OM18*(s->In[1][8]*OM28-
 s->In[5][8]*OM28-s->In[6][8]*OM38));
  Fs17 = -(s->frc[1][7]-s->m[7]*(ALPHA17+BETA27*s->l[2][7]-s->l[1][7]*(OM27*OM27+OM37*OM37)+s->l[3][7]*(BS37+OMp27)));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA27+BS57*s->l[2][7]+s->l[1][7]*(BS27+OMp37)+s->l[3][7]*(BS67-OMp17)));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA37+BETA87*s->l[2][7]+s->l[1][7]*(BS37-OMp27)-s->l[3][7]*(OM17*OM17+OM27*OM27)));
  Fq17 = Fq18+Fs17;
  Fq37 = Fs37+Fq28*S8+Fq38*C8;
  Cq17 = -(s->trq[1][7]-Cq18-s->In[1][7]*OMp17+Fs27*s->l[3][7]-Fs37*s->l[2][7]+OM27*OM37*(s->In[5][7]-s->In[9][7])-
 s->dpt[2][6]*(Fq28*S8+Fq38*C8));
  Cq27 = -(s->trq[2][7]-s->In[5][7]*OMp27-Cq28*C8+Cq38*S8-Fs17*s->l[3][7]+Fs37*s->l[1][7]-OM17*OM37*(s->In[1][7]-
 s->In[9][7]));
  Cq37 = -(s->trq[3][7]-s->In[9][7]*OMp37-Cq28*S8-Cq38*C8+Fq18*s->dpt[2][6]+Fs17*s->l[2][7]-Fs27*s->l[1][7]+OM17*OM27*(
 s->In[1][7]-s->In[5][7]));
  Fs16 = -(s->frc[1][6]-s->m[6]*(ALPHA16+BETA26*s->l[2][6]+BETA36*s->l[3][6]+BS16*s->l[1][6]));
  Fs26 = -(s->frc[2][6]-s->m[6]*(ALPHA26+BETA46*s->l[1][6]+BETA66*s->l[3][6]+BS56*s->l[2][6]));
  Fs36 = -(s->frc[3][6]-s->m[6]*(ALPHA35+BETA76*s->l[1][6]+BETA86*s->l[2][6]+BS96*s->l[3][6]));
  Fq16 = Fq121+Fs16+Fq114*C14+Fq17*C7+Fq314*S14+Fq37*S7;
  Fq26 = Fs214+Fs26+Fs27+Fq215*C15+Fq221*C21+Fq28*C8-Fq315*S15-Fq321*S21-Fq38*S8;
  Fq36 = Fs36-Fq114*S14-Fq17*S7+Fq221*S21+Fq314*C14+Fq321*C21+Fq37*C7;
  Cq16 = -(s->trq[1][6]-Cq121-s->In[1][6]*OMp16-s->In[3][6]*OMp36-Cq114*C14-Cq17*C7-Cq314*S14-Cq37*S7+Fs26*s->l[3][6]-
 Fs36*s->l[2][6]-OM26*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)+OM36*(s->In[5][6]*OM26+s->In[6][6]*OM36)+
 s->dpt[2][2]*(Fq17*S7-Fq37*C7)+s->dpt[2][3]*(Fq114*S14-Fq314*C14)+s->dpt[3][4]*(Fq221*C21-Fq321*S21));
  Cq26 = -(s->trq[2][6]-Cq214-Cq27-s->In[5][6]*OMp26-s->In[6][6]*OMp36-Cq221*C21+Cq321*S21-Fq121*s->dpt[3][4]-Fs16*
 s->l[3][6]+Fs36*s->l[1][6]+OM16*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)-OM36*(s->In[1][6]*OM16+s->In[3][6]*OM36
 )+s->dpt[1][4]*(Fq221*S21+Fq321*C21));
  Cq36 = -(s->trq[3][6]-s->In[3][6]*OMp16-s->In[6][6]*OMp26-s->In[9][6]*OMp36+Cq114*S14+Cq17*S7-Cq221*S21-Cq314*C14-
 Cq321*C21-Cq37*C7+Fs16*s->l[2][6]-Fs26*s->l[1][6]-OM16*(s->In[5][6]*OM26+s->In[6][6]*OM36)+OM26*(s->In[1][6]*OM16+
 s->In[3][6]*OM36)-s->dpt[1][4]*(Fq221*C21-Fq321*S21)+s->dpt[2][2]*(Fq17*C7+Fq37*S7)+s->dpt[2][3]*(Fq114*C14+Fq314*S14));
  Fq15 = Fq16*C6-Fq26*S6;
  Fq25 = Fq16*S6+Fq26*C6;
  Cq25 = Cq16*S6+Cq26*C6;
  Fq14 = Fq15*C5+Fq36*S5;
  Fq34 = -(Fq15*S5-Fq36*C5);
  Cq14 = Cq36*S5+C5*(Cq16*C6-Cq26*S6);
  Fq23 = Fq25*C4-Fq34*S4;
  Fq33 = Fq25*S4+Fq34*C4;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Fq14;
  Qq[2] = Fq23;
  Qq[3] = Fq33;
  Qq[4] = Cq14;
  Qq[5] = Cq25;
  Qq[6] = Cq36;
  Qq[7] = Cq27;
  Qq[8] = Cq18;
  Qq[9] = Cq39;
  Qq[10] = Cq210;
  Qq[11] = Cq111;
  Qq[12] = Cq212;
  Qq[13] = Cq213;
  Qq[14] = Cq214;
  Qq[15] = Cq115;
  Qq[16] = Cq316;
  Qq[17] = Cq217;
  Qq[18] = Cq118;
  Qq[19] = Cq219;
  Qq[20] = Cq220;
  Qq[21] = Cq121;
  Qq[22] = Cq222;
  Qq[23] = Cq323;
  Qq[24] = Cq224;
  Qq[25] = Cq125;
  Qq[26] = Cq326;
  Qq[27] = Cq227;
  Qq[28] = Cq228;
  Qq[29] = Cq129;
  Qq[30] = Cq330;
  Qq[31] = Cq231;

// ====== END Task 0 ====== 


}
 

