
#ifndef MODAL_h
#define MODAL_h

//#include "project_info.h" 
//#include "cmake_config.h"

#include "useful_functions.h"

#include "mbs_xml_reader.h"
#include "mbs_part.h"
#include "mbs_equil.h"
#include "mbs_data.h"
#include "MBSfun.h"

//#include "mstr.h"

typedef struct MODAL_option_strct
{
    double time; // defaut = 0.0
    double lintol; // defaut = 1e-3
    double relincr; // defaut = 1e-2
    double absincr; // defaut = 1e-3
    double mode_ampl; // defaut = 0.2
    double equitol; // defaut = 1e-6
    int save_mat; // no = 0, yes = 1, defaut = 0
    int save_result; // no = 0, yes = 1, defaut = 0
    int save_anim; // no = 0, yes = 1, defaut = 0
    int renamefile; // no = 0, yes = 1, defaut = 0
    int verbose; // no = 0, yes = 1, defaut = 1
    int clearmbsglobal; // inout = 1, out = 2, none = 3, all = 4, defaut = 1

    int n_ignored_qu;
    int *ignored_qu;

} MODAL_option_strct;

typedef struct MODAL_lpk_strct
{
    int *diverge_ind; // size n_x
    int n_diverge;
    double maxcomp;

    double *x_equil;
    double *x_mid;
    double *x_ext;

    double *F_equil;
    double *F_mid;
    double *F_ext; 

    double **GK_prem;
    double **GK_comp;

} MODAL_lpk_strct;

typedef struct MODAL_mode_strct
{
    int type; // 1=unoscillating_stable; 2=rigid; 3=unoscillating_unstable; 4=oscillating-damped; 5=oscillating-undamped; 6=oscillating-unstable
    
    double a; 
    double b;
    
    double ksi; 
    double nu_0; 
    double nu;

    double *eigen_vector_r;
    double *eigen_vector_phi;

} MODAL_mode_strct;

typedef struct MODAL_gen_strct
{
    MODAL_option_strct *options; 

    double *q_save;
    double *qd_save;

    double **Mr; 
    double **Bvuc;

    int n_x;

    MODAL_lpk_strct *lpk;

    double **Mr_xx;

    double **Kr; 
    double **Gr;

    double **B;

    double *eigen_value_a;
    double *eigen_value_b;
    double **eigen_matrix_r;
    double **eigen_matrix_phi;

    int n_mode;
    MODAL_mode_strct **mode_list; 
    int *sorted_mode_ind;


} MODAL_gen_strct;



int MODAL_run_modal(MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct *modal_gen_strct,    MbsAux *mbs_aux, MbsData *mbs_data);


MODAL_option_strct* init_MODAL_option_strct();
void free_MODAL_option_strct(MODAL_option_strct* modal_option_strct);

MODAL_lpk_strct* init_MODAL_lpk_strct(int n_x);
void free_MODAL_lpk_strct(MODAL_lpk_strct* modal_lpk_strct, int n_x);

void init_MODAL_gen_strct_SD(MODAL_gen_strct* modal_gen_strct, int n_x);
void free_MODAL_gen_strct_SD(MODAL_gen_strct* modal_gen_strct, int n_x);

MODAL_gen_strct* init_MODAL_gen_strct(MDS_gen_strct*  mds_gen_strct, MbsPart *mbs_part);
void free_MODAL_gen_strct(MODAL_gen_strct* modal_gen_strct, MbsPart *mbs_part);

//void MODAL_get_options_from_user(MODAL_option_strct *options);

#endif
