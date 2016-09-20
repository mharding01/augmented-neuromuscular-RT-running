
#include "mbs_data.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// Those utility functions should be move elsewhere
// We should use function of useful_functions. h but signature is not the same ... we need to merge that all!

void print_vector(char* prefix, int n, double *vec){
    int i;
    printf(" %s = [", prefix);
    for(i=1; i <= n ; i++){
       printf(" %lf ", vec[i]);
    }
    printf("]\n");
}

void print_intvector(char* prefix, int n, int *vec){
    int i;
    printf(" %s = [", prefix);
    for(i=1; i <= n ; i++){
       printf(" %d ", vec[i]);
    }
    printf("]\n");
}

void print_matrix(char* prefix, int r, int c, double **mat){
    int i, j;
    printf(" %s = [", prefix);
    for(i=1; i <= r ; i++){
        for(j=1; j <= c ; j++){
            printf(" %lf ", mat[i][j]);
        }
        printf(" \n        ");
    }
    printf("]\n");
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */





void mbs_print_data(MbsData* s){
    printf("\n MbsData \n");

    printf(" nbody  . .  = %d \n", s->nbody);
    printf(" njoint . .  = %d \n", s->njoint);
    printf(" npt  . . .  = %d \n", s->npt);
    printf(" nqu  . . .  = %d \n", s->nqu);
    printf(" nqc  . . .  = %d \n", s->nqc);
    printf(" nqa  . . .  = %d \n", s->nqa);
    printf(" nqv  . . .  = %d \n", s->nqv);
    printf(" nhu  . . .  = %d \n", s->nhu);
    printf(" nqlocked .  = %d \n", s->nqlocked);
    printf(" nqdriven .  = %d \n", s->nqdriven);
    //printf(" Nloopc . .  = %d \n", s->Nloopc);
    printf(" Nuserc . .  = %d \n", s->Nuserc);
    printf(" Ncons  . .  = %d \n", s->Ncons);
    printf(" Nlink  . .  = %d \n", s->Nlink);
    printf(" Nlink3D  .  = %d \n", s->Nlink3D);
    printf(" Nsensor  .  = %d \n", s->Nsensor);
    printf(" Nxfrc  . .  = %d \n", s->Nxfrc);
    //printf(" Nwheel . .  = %d \n", s->Nwheel);
    printf(" Nuser_model = %d \n", s->Nuser_model);
    printf(" Nux . . . . = %d \n", s->Nux);

    print_vector("g",  3, s->g);

    print_vector("q  ", s->njoint, s->q);
    print_vector("qd ", s->njoint, s->qd);
    print_vector("qdd", s->njoint, s->qdd);
    print_vector("Qq",  s->njoint, s->Qq);

    printf("\n");
    print_intvector("qc  ", s->nqc, s->qc);
    print_intvector("qu  ", s->nqu, s->qu);
    print_intvector("qa  ", s->nqa, s->qa);
    print_intvector("qv  ", s->nqv, s->qv);
    print_intvector("hu  ", s->nhu, s->hu);

    print_matrix("dpt ", 3, s->npt, s->dpt);
    print_vector("m",  s->njoint, s->m);
    print_matrix("In ", 9, s->njoint, s->In);
    print_matrix("l ", 3, s->njoint, s->l);

    /*printUserModel(s->usrMod);

    printUserIO(s->userIO);*/
}
