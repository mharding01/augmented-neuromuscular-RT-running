
#include "mbs_part.h"
#include "mbs_project_interface.h"
#include "gsl/gsl_linalg.h"

void readmatrix (double** matrix,int* n);                   //function to read a matrix
void printmatrix (double** matrix,int* n);              //function to print a matrix
void update_initzeros(int* initzeros,double** matrix,int* n);   //function to update the initzeros array
void arrange_matrix(double** matrix,int* n,int* initzeros);     //function to arrange matrix
void scale_matrix(double** matrix,int* n,int* initzeros);       //Function to scale matrix

//----------------------------------------------------------------
//                  Main function
//---------------------------------------------------------------

int rank_double_tab(double** matrix_in, int x, int y)
{
    int i,j,n,m;
    int rank;

    gsl_matrix * A;
    gsl_matrix * V;
    gsl_vector * S;
    gsl_vector * work;

    if(x>y)
    {
        m = x;
        n = y;
    }
    else
    {
        m = y;
        n = x;
    }

    A = gsl_matrix_alloc(m, n);
    V = gsl_matrix_alloc(n, n);
    S = gsl_vector_alloc(n);
    work = gsl_vector_alloc(n);

    for(i=0; i<x; i++)
    {
        for(j=0; j<y; j++)
        {
            if(x>y)
            {
                gsl_matrix_set(A, i, j, matrix_in[i][j]);
            }
            else
            {
                gsl_matrix_set(A, j, i, matrix_in[i][j]);
            }
        }
    }

    gsl_linalg_SV_decomp(A,V,S,work);

    rank = 0;
    for(i=0; i<n; i++)
    {
        if(fabs(gsl_vector_get(S,i))> 1e-30)
        {
            rank ++;
        }
    }

    gsl_matrix_free(A);
    gsl_matrix_free(V);
    gsl_vector_free(S);
    gsl_vector_free(work);

    return rank;
}

int rank_double_tab2(double** matrix_in, int x, int y)
{



    double **matrix;

    /*
        int n[2], rank;


    readmatrix(matrix, n);

    printf("\tThe matrix you have entered is shown below\n\n");
    printmatrix(matrix,n);

    rank =  rank_double_vec(matrix, n[0], n[1]);
    */

    int n[2], i, retest=1, grp, p, r, j, t, q, rank;
    int* initzeros;

    matrix = get_double_tab(x,y);
    copy_double_tab(matrix_in, matrix,x, y);

    n[0] = x;
    n[1] = y;
    initzeros = get_int_vec(y);

    /*
    printf("\tThe matrix you have entered is shown below %d , %d\n\n", x, y);
    printmatrix(matrix,n);
    //*/

    update_initzeros(initzeros,matrix,n);
    arrange_matrix(matrix,n,initzeros);

    /*printf("\tThe matrix after arrange  %d , %d\n\n", x, y);
    printmatrix(matrix,n);



    if(matrix[0][0]==0)
    {
    printf("\n\tError: Invalid Marix \n\n");
    }

    */

    update_initzeros(initzeros,matrix,n);
    scale_matrix(matrix,n,initzeros);

    while(retest==1)
    {
        grp=0;
        for(i=0;i<n[0];++i)
        {
            p=0;
            while(initzeros[i+p]==initzeros[i+p+1]&&(i+p+1)<n[0])
            {
                grp=grp+1;
                p=p+1;
            }

            if(grp!=0)
            {
                while(grp!=0)
                {
                    for(j=0;j<n[1];++j)
                    {
                        matrix[i+grp][j]=matrix[i+grp][j]-matrix[i][j];
                    }
                    grp=grp-1;
                }
            break;
            }
        }

        update_initzeros(initzeros,matrix,n);
        arrange_matrix(matrix,n,initzeros);
        update_initzeros(initzeros,matrix,n);
        scale_matrix(matrix,n,initzeros);

        retest=0;
        for(r=0;r<n[0];++r)
        {
            if(initzeros[r]==initzeros[r+1]&&r+1<n[0])
            {
                if(initzeros[r]!=n[1])
                retest=1;
            }
        }
    }

    //printf("\n\n\t Reduced matrix is as shown below\n\n");
    //printmatrix(matrix,n);

    rank =0;
    for (i=0;i<n[0];++i)
    {
        if (initzeros[i]!=n[1])
        {
            ++rank;
        }
    }

    free_double_tab(matrix, n[0]);

    //printf("\n Rank Of The Matrix = %d\n\n", rank);

    return rank;
}


//----------------------------------------------------------------
//           Function definition to read a matrix
//----------------------------------------------------------------

void readmatrix (double** mat,int* n)
{
    int i,j;

    printf("\tEnter the no: rows in the matrix : ");
    scanf("%d",&n[0]);
    printf("\tEnter the no: columns in the matrix : ");
    scanf("%d",&n[1]);
    printf("\tEnter The Matrix Elements Row wise  ");
    scanf("\n");

    for(i=0;i<n[0];++i)
    {
        for(j=0;j<n[1];++j)
        {
            scanf("%lf",&mat[i][j]);
        }
    }
}

//----------------------------------------------------------------
//               Function definition to print a matrix
//----------------------------------------------------------------


void printmatrix (double** mat,int* n)
{
    int i,j;
    for(i=0;i<n[0];++i)
    {
        for(j=0;j<n[1];++j)
        {
            printf("  %f",mat[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}


//------------------------------------------------------------------
//               Function definition to Update initzeros array
//-----------------------------------------------------------------

void update_initzeros(int* initzeros, double** matrix,int* n)
{
    int zcount, i, j;
    for(i=0;i<n[0];++i)
    {
        zcount=0;
        for(j=0; (matrix[i][j]==0) && (j<n[1]); ++j)
        {
            ++zcount;
        }
        initzeros[i]=zcount;
    }
}


//-------------------------------------------------------------------
//               Function definition to arrange matrix
//-------------------------------------------------------------------

void arrange_matrix(double** matrix,int* n,int* initzeros)
{
    int l,reqrow,i,k,lastrow,tempvar,large;
    double* rowtemp;
    rowtemp = get_double_vec(n[1]);
    lastrow=n[0]-1;

    for(l=0;l<n[0];++l)
    {
        large=initzeros[0];
        for(i=0;i<n[0];++i)
        {
            if(large<=initzeros[i])
            {
                large=initzeros[i];
                reqrow=i;
            }
        }

        initzeros[reqrow]=-1;
        tempvar=initzeros[reqrow];
        initzeros[reqrow]=initzeros[lastrow];
        initzeros[lastrow]=tempvar;

        for(k=0;k<n[1];++k)
        {
            rowtemp[k]=matrix[lastrow][k];
        }
        for(k=0;k<n[1];++k)
        {
            matrix[lastrow][k]=matrix[reqrow][k];
        }
        for(k=0;k<n[1];++k)
        {
            matrix[reqrow][k]=rowtemp[k];
        }
        lastrow=lastrow-1;
    }
}

//---------------------------------------------------------------------------
//                        Function definition to scale a matrix
//---------------------------------------------------------------------------


void scale_matrix(double** matrix,int* n,int* initzeros)
{
    int i,j;
    double divisor;
    for(i=0; i<n[0]; ++i)
    {
        divisor=matrix[i][initzeros[i]];

        for(j=initzeros[i];j<n[1];++j)
        {
            matrix[i][j]=matrix[i][j]/divisor;
        }
    }
}

void PART_vec_perm(int* a ,int* ipm, int irk)
{
    int i, i1, z;

    for(i=0; i<irk; i++)
    {
        i1 = ipm[i];
        if( i1 != i)
        {
            z = a[i];
            a[i] = a[i1];
            a[i1] = z;
        }
    }
}

void PART_lutot(double** a, int nl, int nc, int rowperm, int* irk, int* ierr, int* row_perm, int* col_perm)
{
    int i, j, iter, indl, indc, ilmax, il, ic, it1;
    double p,z;
    double epslu = 1.0e-15;
    int *ipl, *ipc;
    *irk = 0;

    ipl = get_int_vec(nl);
    ipc = get_int_vec(nc); ////////caution

    for (iter=0; iter<nl; iter++)
    {
        //   Maximum pivot search : element (il, ic)
        if (rowperm)
        {
            ilmax = nl;
        }
        else
        {
            ilmax = iter+1;
        }

        //printf("\n ilmax = %d\n", ilmax);
        p = 0.0;
        for(j=iter; j<nc; j++)
        {
            //printf("\n iter = %d\n", iter);
            for(i=iter; i<ilmax; i++)
            {
                z = a[i][j];
                if(fabs(z) > fabs(p))
                {
                    p = z;
                    il = i;
                    ic = j;
                }
            }
        }

        //   Case of a singular matrix (redundant constraints)

        f0123_int_vec(row_perm, nl);
        f0123_int_vec(col_perm, nc);

        if(fabs(p) < epslu )
        {
            *ierr = -1;
            PART_vec_perm(row_perm, ipl, *irk);
            PART_vec_perm(col_perm, ipc, *irk);
        }
        else
        {
            //   Update of the rank and the permutation vectors

            *irk = iter+1;
            ipl[iter] = il;
            ipc[iter] = ic;

            //   Permutation of rows iter and il
            if(il > iter)
            {
                for(j=0; j<nc; j++)
                {
                    z = a[iter][j];
                    a[iter][j] = a[il][j];
                    a[il][j] = z;
                }
            }
            //   Permutation of columns iter and ic
            if(ic > iter)
            {
                for(i=0; i<nl; i++)
                {
                    z = a[i][iter];
                    a[i][iter] = a[i][ic];
                    a[i][ic] = z;
                }
            }

            //   Non pathological case : the process ends at iter = nl or nc
            if((iter == nl) || (iter == nc))
            {
                *ierr = 0;
                PART_vec_perm(row_perm, ipl, *irk);
                PART_vec_perm(col_perm, ipc, *irk);
             }
            else
            {
                //   Transformation of the sub-matrix
                it1 = iter + 1;
                for(i = it1; i<nl; i++){
                    z = a[i][iter]/p;
                    a[i][iter] = z;
                    for(j = it1; j<nc; j++)
                    {
                        a[i][j] = a[i][j] - z * a[iter][j];
                    }
                }
            }
        }
    }
    free_int_vec(ipl);
    free_int_vec(ipc);
}

double frand(void)
{
    return(rand()/(double)RAND_MAX );
}

void PART_calc_hJ(MbsData* mbs_data, double* h, double** J)
{
    int i,j;

    double* h_loopc = get_double_vec(mbs_data->Nloopc +1);
    double** J_loopc = get_double_tab(mbs_data->Nloopc +1, mbs_data->njoint+1);
    double* h_userc= get_double_vec(mbs_data->Nuserc +1);
    double** J_userc = get_double_tab(mbs_data->Nuserc +1, mbs_data->njoint+1);

    if(mbs_data->Nloopc)
    {
        mbs_cons_hJ(h_loopc, J_loopc, mbs_data, mbs_data->tsim);
        for (i=0; i<mbs_data->Nloopc; i++)
        {
            h[i] = h_loopc[i+1];
            for (j=0; j<mbs_data->njoint; j++)
            {
                J[i][j] =J_loopc[i+1][j+1];
            }
        }
    }
    if(mbs_data->Nuserc)
    {
        user_cons_hJ(h_userc, J_userc, mbs_data, mbs_data->tsim);
        for (i=0; i<mbs_data->Nuserc; i++)
        {
            h[mbs_data->Nloopc+i] = h_userc[i+1];
            for (j=0; j<mbs_data->njoint; j++)
            {
                J[mbs_data->Nloopc+i][j] =J_userc[i+1][j+1];
            }
        }
    }

    free_double_vec(h_loopc);
    free_double_tab(J_loopc, mbs_data->Nloopc +1);
    free_double_vec(h_userc);
    free_double_tab(J_userc, mbs_data->Nuserc +1);
}


void PART_coord_part(MbsData* mbs_data, MbsPart*  mbs_part, int* ind_u_des, int n_u_des, int* ind_c, int n_c, int first_partitioning , int* err)
{
    int i, j, nl_JAC, nc_JAC, Real_Rank, prsnt,n_vu, ivu, irk, ierr;

    double* q_rand, * q_safe;
    double* h_;
    double** Jac, **Jv;

    double hjfd;

    int *ind_u_des_com,  *ind_vu, *row_perm, *col_perm;
    int *ind_aux, *ind_irk, *ind_u_nsort ,*ind_hv_nsort;

    q_rand = get_double_vec(mbs_data->njoint);
    q_safe = get_double_vec(mbs_data->njoint);

    nl_JAC = mbs_data->Nloopc + mbs_data->Nuserc;
    nc_JAC = mbs_data->njoint;

    h_ = get_double_vec(nl_JAC);
    Jac = get_double_tab(nl_JAC, nc_JAC);

    ind_u_des_com = get_int_vec(n_u_des + n_c);
    n_vu = mbs_data->njoint - (n_u_des + n_c);
    ind_vu = get_int_vec(n_vu);

    *err = 0;

    // Compute the real rank of the Jacobian matrix

    for(i=0; i<mbs_data->njoint; i++)
    {
        q_rand[i] = frand();
    }
    copy_double_vec(&(mbs_data->q[1]), q_safe, mbs_data->njoint);
    copy_double_vec(q_rand, &(mbs_data->q[1]), mbs_data->njoint);

    PART_calc_hJ(mbs_data, h_, Jac);

    Real_Rank = rank_double_tab(Jac, nl_JAC, nc_JAC);                   // calcul du rang reel de Jac pour q random

    copy_double_vec(q_safe, &(mbs_data->q[1]), mbs_data->njoint);

    //  Proposition de partitionnement pour un u_des fourni (vide), (partiel, plein, valide ou non)

    conc_int_vec (ind_u_des, n_u_des, ind_c, n_c, ind_u_des_com);    // mise en commun des u_des et des c

    ivu=0; // variables (v et u) a partitionner //old = setdiff in matlab
    for(i=0; i<mbs_data->njoint; i++)
    {
        prsnt = 0;
        for(j=0; j<(n_u_des + n_c) ;j++)
        {
            if(i == ind_u_des_com[j])
            {
                prsnt = 1;
            }
        }
        if(prsnt == 0)
        {
            ind_vu[ivu] = i;
            ivu++;
        }
    }

    if( first_partitioning == 1)// Distrub the 0 the first time only (for the second one, the system is closed)
    {
        for(i=0; i<n_vu; i++)// random perturbation  (1e-5) of the null values (v,u_non_des): avoid singularities
            {
            if (fabs(mbs_data->q[ind_vu[i]+1]) < 1e-6)
            {
                mbs_data->q[ind_vu[i]+1] = 1e-5*frand();
            }
        }
    }

    PART_calc_hJ(mbs_data, h_, Jac);

    row_perm = get_int_vec(nl_JAC);
    col_perm = get_int_vec(n_vu);
    Jv = get_double_tab(nl_JAC, n_vu);

    // Pivoted LU Factorisation

    slctc_double_tab(Jac, nl_JAC, nc_JAC, Jv, n_vu, ind_vu);

    PART_lutot(Jv, nl_JAC, n_vu, mbs_part->options->rowperm, &irk, &ierr, row_perm, col_perm);

    // Check the user choice for independent coordinates

    if (irk < Real_Rank)   // choix mauvais - erreur
    {
        printf("<<<<<ok : %d>>>>>\n",irk );
        mbs_part->ind_qu = NULL; mbs_part->ind_qv = NULL; mbs_part->ind_hu = NULL; mbs_part->ind_hv = NULL;
        mbs_part->n_qu = 0;
        mbs_part->n_qv = 0;
        mbs_part->n_hu = 0;
        mbs_part->n_hv = 0;
        printf(">>PART>>\n");
        printf(">>PART>> mbs_coord_part: wrong choice for the independent variable set - Jv matrix singular\n");
        printf(">>PART>>\n");
        *err = -1;
    }
    else                 // choix valide
    {
        ind_aux = get_int_vec(n_vu);
        ind_irk = get_int_vec(irk);
        f0123_int_vec(ind_irk, irk);

        slct_int_vec(ind_vu, n_vu, col_perm, n_vu, ind_aux);

        mbs_part->n_qv = irk;
        if (mbs_part->ind_qv != NULL)
        {
            free_int_vec(mbs_part->ind_qv);
        }
        mbs_part->ind_qv = get_int_vec(mbs_part->n_qv);

        slct_int_vec(ind_aux, 0, ind_irk, irk, mbs_part->ind_qv); // Liste definitive des v


        if (n_vu > irk)    // additional independent variabes : les dernières                                      //?? n_vu = lenth (ind_aux)
        {
            ind_u_nsort = get_int_vec(n_u_des+(n_vu-irk));
            copy_int_vec(ind_u_des, ind_u_nsort,n_u_des);
            for(i=0; i<(n_vu-irk); i++)
            {
                ind_u_nsort[n_u_des + i] = ind_aux[irk+i];
            }
            mbs_part->n_qu = n_u_des +(n_vu-irk);
        }
        else
        {
            mbs_part->n_qu = n_u_des;
            ind_u_nsort = get_int_vec(mbs_part->n_qu);
            copy_int_vec(ind_u_des, ind_u_nsort,mbs_part->n_qu);         // no additional independent variable
        }

        if (mbs_part->ind_qu != NULL)
        {
            free_int_vec(mbs_part->ind_qu);
        }
        mbs_part->ind_qu = get_int_vec(mbs_part->n_qu);
        sort_int_vec(ind_u_nsort, mbs_part->ind_qu, mbs_part->n_qu);
        free_int_vec(ind_u_nsort);

        for(i=0; i<mbs_part->n_qu; i++)
        {
            mbs_data->q[mbs_part->ind_qu[i]+1] = q_safe[mbs_part->ind_qu[i]]; // +1 for mbs_data convention
        }

        if (irk == nl_JAC)                   // Case : no redundant constraints
        {
            mbs_part->n_hu = nl_JAC;
            if (mbs_part->ind_hu != NULL)
            {
                free_int_vec(mbs_part->ind_hu);
            }
            mbs_part->ind_hu = get_int_vec(mbs_part->n_hu);
            copy_int_vec(row_perm, mbs_part->ind_hu, mbs_part->n_hu);

            mbs_part->ind_hv = NULL;
            mbs_part->n_hv = 0;
        }
        else      // Case : redundant constraints
        {
            mbs_part->n_hu = irk;
            if (mbs_part->ind_hu != NULL)
            {
                free_int_vec(mbs_part->ind_hu);
            }
            mbs_part->ind_hu = get_int_vec(mbs_part->n_hu);
            copy_int_vec(row_perm, mbs_part->ind_hu, mbs_part->n_hu);

            mbs_part->n_hv = nl_JAC-irk;
            ind_hv_nsort = get_int_vec(mbs_part->n_hv);

            copy_int_vec(&(row_perm[irk]), ind_hv_nsort, mbs_part->n_hv);
            if (mbs_part->ind_hv != NULL)
            {
                free_int_vec(mbs_part->ind_hv);
            }
            mbs_part->ind_hv = get_int_vec(mbs_part->n_hv);
            sort_int_vec(ind_hv_nsort, mbs_part->ind_hv, mbs_part->n_hv);

            free_int_vec(ind_hv_nsort);
        }
        free_int_vec(ind_aux);
        free_int_vec(ind_irk );
    }

    free_int_vec(row_perm);
    free_int_vec(col_perm);
    free_double_tab(Jv, nl_JAC);

    free_double_vec(q_rand);
    free_double_vec(q_safe);
    free_double_vec(h_);
    free_double_tab(Jac, nl_JAC);
    free_int_vec(ind_u_des_com);
    free_int_vec(ind_vu);



    // copy data from mbs_part to mbs_data for calling close_loop correctly
    mbs_data->nhu = mbs_part->n_hu;
    for(i=0; i<mbs_data->nhu; i++){
        mbs_data->hu[i+1]=mbs_part->ind_hu[i]+1;
    }
    mbs_data->nqv = mbs_part->n_qv;
    for(i=0; i<mbs_data->nqv; i++){
        mbs_data->qv[i+1]=mbs_part->ind_qv[i]+1;
    }


}


int mbs_run_part(MbsPart*  mbs_part, MbsData* mbs_data)
{

    int i;
    int err, nbiter;
    int* ind_c = NULL;
    int* ind_u_des = NULL;


    MbsAux *mbs_aux; //change it !

    if(mbs_data->Ncons == 0) // change it !
    {
        if(mbs_part->options->verbose)
        {
            printf(">>PART>> no coordinate partitioning required for this system !\n");
        }
        mbs_data->DonePart = 1;
        return mbs_data->DonePart;
    }
    if(mbs_part->options->verbose)
    {
        printf(">>PART>> PART_run start\n");
    }

    // sort qu and qc

    ind_c = get_int_vec(mbs_data->nqc);
    ind_u_des = get_int_vec(mbs_data->nqu);

    sort_int_vec(mbs_data->qc+1, ind_c, mbs_data->nqc);
    sort_int_vec(mbs_data->qu+1, ind_u_des, mbs_data->nqu);
    // switch to index strating at 0  (instead of 1)
    for (i=0; i<mbs_data->nqc; i++){
        ind_c[i]-=1;
    }
    for (i=0; i<mbs_data->nqu; i++){
        ind_u_des[i]-=1;
    }


    // call user_drivenJoints

    user_DrivenJoints(mbs_data, mbs_data->tsim);
    /*
    for ( i=0; i<100; i++)
    {
        printf("%f\n",frand());
    }*/
    /*
    print_int_vec(mds_gen_strct->bodytree->qu, s->nqu);
    print_int_vec(ind_u_des, s->nqu);
    */

    //  First partitioning

    if (mbs_part->options->verbose)
    {
        printf(">>PART>> ***** Coordinate partitioning *****\n");
    }

    PART_coord_part(mbs_data, mbs_part, ind_u_des, mbs_data->nqu, ind_c, mbs_data->nqc, 1, &err); // d'abord on cherche un choix valable



    if( err == -1)
    {
        printf(">>PART>>\n");
        printf(">>PART>> ***** mbs_run_part : Coordinate partitioning interrupted : *****\n");
        printf(">>PART>>\n");
        //myproject_part = MBS_part;
        mbs_data->DonePart = 0;
        return mbs_data->DonePart;
    }

    //  Fermeture
    if (mbs_part->options->verbose)
    {
        printf(">>PART>> ***** Geometrical Constraint solution *****\n");
    }

    mbs_aux = initMbsAux(mbs_data);
    nbiter = mbs_close_geo(mbs_data, mbs_aux); // on essaie de fermer
    freeMbsAux(mbs_aux, mbs_data);


    if (nbiter == -1)
    {
        printf(">>PART>>\n");
        printf(">>PART>> ***** mbs_run_part : impossible to close the MBS : *****\n");
        printf(">>PART>>    -> try with other desired independent variables u\n");
        printf(">>PART>>    and/or\n");
        printf(">>PART>>    -> try with other initial values for q (u, v)\n");
        printf(">>PART>>    -> ... but have a look at the proposed {u,v} partition\n");
        printf(">>PART>>\n");
    }
    else
    {
        if (mbs_part->options->verbose)
        {
        printf("***** Constaints solved after %d iterations *****\n", nbiter);
        }
        copy_double_vec(mbs_data->q+1, mbs_part->q_closed ,mbs_data->njoint);

    // Second partitioning on the closed structure (possible refinment still on the basis of ind_u_des)

        PART_coord_part(mbs_data, mbs_part, ind_u_des, mbs_data->nqu, ind_c, mbs_data->nqc, 0, &err);
        if (err == -1)   // not very likely but should be considered ...
        {
            printf(">>PART>>\n");
            printf(">>PART>> ***** mbs_run_part : Second coordinate partitioning interrupted : *****\n");
            printf(">>PART>>\n");
            //myproject_part = MBS_part;
            mbs_data->DonePart = 0;
            return mbs_data->DonePart;
        }
        else
        {
        mbs_data->DonePart = 1;
        }
    }

    // Stockage des resultats dans la structure mbs_part et s//tockage
    /*
    mbs_part.nu = length(ind_u);
    mbs_part.ind_u = ind_u;
    mbs_part.nv = length(ind_v);
    mbs_part.ind_v = ind_v;
    mbs_part.nhu = length(ind_hu); // JFC : 15/01/2008 : ajout
    mbs_part.hu = ind_hu;
    mbs_part.nhv =  length(ind_hv); // PF : 09/04/2009 : ajout
    mbs_part.hv = ind_hv;
    */

    // Affectation global=>Workspace

    //myproject_part = MBS_part;

    if (1)
    {
        mbs_data->nqu = mbs_part->n_qu;
        for(i=0; i<mbs_part->n_qu; i++)
        {
            mbs_data->qu[i+1] = mbs_part->ind_qu[i] +1;
        }
        mbs_data->nqv = mbs_part->n_qv;
        for(i=0; i<mbs_part->n_qv; i++)
        {
            mbs_data->qv[i+1] = mbs_part->ind_qv[i] +1;
        }
        mbs_data->nhu = mbs_part->n_hu;
        for(i=0; i<mbs_part->n_hu; i++)
        {
            mbs_data->hu[i+1] = mbs_part->ind_hu[i] +1;
        }
        /*mbs_data->nhv = mbs_part->n_hv;
        for(i=0; i<mbs_part->n_hv; i++)
        {
            mbs_data->hv[i+1] = mbs_part->ind_hv[i] +1;
        }*/
    }

    if (mbs_part->options->verbose)
    {
        printf(">>PART>> Partitionning result :\n");

        printf("nqu :  %d\n",mbs_part->n_qu);
        printf("qu :  ");print_int_vec(mbs_data->qu+1, mbs_part->n_qu);
        printf("\n");
        printf("nqv :  %d\n",mbs_part->n_qv);
        printf("qv :  ");print_int_vec(mbs_data->qv+1, mbs_part->n_qv);
        printf("\n");
        printf("nhu :  %d\n",mbs_part->n_hu);
        printf("hu :  ");print_int_vec(mbs_data->hu+1, mbs_part->n_hu);
        printf("\n");
//      printf("nhv :  %d\n",mbs_part->n_hv);
//      printf("hv :  ");print_int_vec(mbs_part->ind_hv, mbs_part->n_hv);
//      printf("\n");

        printf(">>PART>> ***** mbs_run_part end *****\n");
    }

    free_int_vec(ind_u_des);
    free_int_vec(ind_c);

    return mbs_data->DonePart;
}



////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Init and free functions for PART structures  ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


MbsPartOptions* mbs_new_part_options(void)
{
    MbsPartOptions *part_option_strct;

    part_option_strct = (MbsPartOptions*) malloc(sizeof(MbsPartOptions));

    part_option_strct->rowperm = 0;
    part_option_strct->visualise = 0;
    part_option_strct->treshold = 1e-9;
    part_option_strct->drivers = 0;
    part_option_strct->verbose = 1;
    part_option_strct->clearmbsglobal = 1;

    return part_option_strct;
}

void mbs_delete_part_options(MbsPartOptions* opt)
{
    free(opt);
}

MbsPart* mbs_new_part(MbsData*  mbs_data)
{
    MbsPart *mbs_part;
    
    mbs_part = (MbsPart*) malloc(sizeof(MbsPart));

    // Initialize options
    mbs_part->options = mbs_new_part_options();

    mbs_part->n_qu = 0;
    mbs_part->ind_qu = NULL;

    mbs_part->n_qv = 0;
    mbs_part->ind_qv = NULL;

    mbs_part->n_hu = 0;
    mbs_part->ind_hu = NULL;

    mbs_part->n_hv = 0;
    mbs_part->ind_hv = NULL;

    mbs_part->q_closed = get_double_vec(mbs_data->njoint);

    return mbs_part;
}
void mbs_delete_part(MbsPart* mbs_part)
{
    mbs_delete_part_options(mbs_part->options);

    free_int_vec(mbs_part->ind_qu);
    free_int_vec(mbs_part->ind_qv);
    free_int_vec(mbs_part->ind_hu);
    free_int_vec(mbs_part->ind_hv);

    free_double_vec(mbs_part->q_closed);

    free(mbs_part);
}
