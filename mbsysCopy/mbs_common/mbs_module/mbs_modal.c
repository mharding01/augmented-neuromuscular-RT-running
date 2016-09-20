#include "mbs_modal.h"

#include "gsl/gsl_math.h"
#include "gsl/gsl_eigen.h"


int MODAL_eig(double **in_matrix, int n, double *eigen_value_a, double *eigen_value_b, double **eigen_matrix_r, double **eigen_matrix_phi)
{
    int i,j;
    double max_r, phi_0;
    int ind_max_r; 

    gsl_matrix * matrix = gsl_matrix_alloc(n, n);

    gsl_vector_complex * r_vec = gsl_vector_complex_alloc(n);
    gsl_matrix_complex * r_mat = gsl_matrix_complex_alloc(n, n);

    gsl_complex cplx;

    gsl_eigen_nonsymmv_workspace * w =  gsl_eigen_nonsymmv_alloc (n);
    
    for (i=0; i<n; i++)
    {
        for(j=0; j<n; j++)
        {
            gsl_matrix_set(matrix, i, j, in_matrix[i][j]);
        }
    }

    gsl_eigen_nonsymmv (matrix, r_vec, r_mat,  w);

    for (i=0; i<n; i++)
    {
        cplx =  gsl_vector_complex_get (r_vec,i);
        eigen_value_a[i] = GSL_REAL(cplx);
        eigen_value_b[i] = GSL_IMAG(cplx);

        max_r=0.0; 
        for(j=0; j<n; j++)
        {
            cplx = gsl_matrix_complex_get(r_mat,j,i);
            eigen_matrix_r[j][i] = sqrt(GSL_REAL(cplx) * GSL_REAL(cplx) + GSL_IMAG(cplx) * GSL_IMAG(cplx));
            eigen_matrix_phi[j][i] = atan2(GSL_IMAG(cplx), GSL_REAL(cplx));
            if (eigen_matrix_r[j][i]>max_r)
            {
                max_r = eigen_matrix_r[j][i];
                ind_max_r = j;
            }
        }
        phi_0 = eigen_matrix_phi[ind_max_r][i];
        for(j=0; j<n; j++)
        {
             eigen_matrix_r[j][i] = eigen_matrix_r[j][i]/max_r; 
             eigen_matrix_phi[j][i] = eigen_matrix_phi[j][i]-phi_0;
        }
    }

    gsl_matrix_free(matrix);
    gsl_vector_complex_free(r_vec); 
    gsl_matrix_complex_free(r_mat); 

    gsl_eigen_nonsymmv_free (w);

    return EXIT_SUCCESS;
}

int MODAL_fct_modal(double *x, int n_x, double *F_x, MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct *modal_gen_strct,    MbsAux *mbs_aux, MbsData *mbs_data, int posvit)
{
    int fail = 0; 
    int i; 

    int ind_x;
    double *Fr_uc;
    Fr_uc = get_double_vec(mbs_part->n_qu + mds_gen_strct->bodytree->n_qc);


    ind_x = 0;
    for(i=0; i<mbs_part->n_qu; i++)
    {
        //equil_gen_strct->xeq_ptr[i][0] = x[i];
        if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[i]) == -1)
        {
            if (posvit == 1)
            {
                mbs_data->q[mbs_part->ind_qu[i]+1] = x[ind_x];
            }
            else 
            {
                mbs_data->qd[mbs_part->ind_qu[i]+1] = x[ind_x];
            }
            ind_x++;
        }
    }

    fail = compute_Fr_uc(Fr_uc, mds_gen_strct,mbs_part, mbs_aux, mbs_data );

    ind_x = 0;
    for(i=0; i<mbs_part->n_qu; i++)
    {
        if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[i]) == -1)
        {
            F_x[ind_x] = Fr_uc[i];
            ind_x++;
        }
    }

    free_double_vec(Fr_uc);
    return fail; 
}

int MODAL_linearipk(double **GK, MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct *modal_gen_strct, MbsAux *mbs_aux, MbsData *mbs_data, int posvit) 
{
    int i, j, k; // fctlin,qoqp_equi,fctlin_equi,posvit,epsmod,delta_ua,delta_ur,linpa
    int ind_x;
    int rep, compt, test, li; 

    int lin_fail = 0;

    double max_GK_k, max_GK_prem_div_k;
    double increment;

    modal_gen_strct->lpk->n_diverge = 0;
    modal_gen_strct->lpk->maxcomp = 0.0;

    ind_x = 0;
    for(i=0; i<mbs_part->n_qu; i++)
    {
        //equil_gen_strct->xeq_ptr[i][0] = x[i];
        if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[i]) == -1)
        {
            if (posvit == 1)
            {
                modal_gen_strct->lpk->x_equil[ind_x] = mbs_data->q[mbs_part->ind_qu[i] +1];
            }
            else 
            {
                modal_gen_strct->lpk->x_equil[ind_x] = mbs_data->qd[mbs_part->ind_qu[i] +1];
            }
            ind_x++;
        }
    }

    for(k=0; k<modal_gen_strct->n_x; k++)// boucle sur la matrice
    {    
        for(i=0; i<modal_gen_strct->n_x; i++)
        {
            for(j=0; j<modal_gen_strct->n_x; j++)
            {
                modal_gen_strct->lpk->GK_comp[i][j] = 100.0;
            }
        }
        rep = 1;
        compt = 1;
        copy_double_vec(modal_gen_strct->lpk->x_equil, modal_gen_strct->lpk->x_ext, modal_gen_strct->n_x);

        //print_double_vec(modal_gen_strct->lpk->x_equil,  modal_gen_strct->n_x);

        copy_double_vec(modal_gen_strct->q_save, &(mbs_data->q[1]), modal_gen_strct->n_x);
        copy_double_vec(modal_gen_strct->qd_save, &(mbs_data->qd[1]), modal_gen_strct->n_x);

    
    //   Calcul de l'incrément idéal
    
        if (fabs(modal_gen_strct->lpk->x_ext[k]*modal_gen_strct->options->relincr) < modal_gen_strct->options->absincr)
        {
            if (modal_gen_strct->lpk->x_ext[k] == 0.0)
            {
                increment = modal_gen_strct->options->absincr;
            }
            else
            {
                increment = sign(modal_gen_strct->lpk->x_ext[k]) * modal_gen_strct->options->absincr;
            }
        }
        else
        {
            increment = modal_gen_strct->lpk->x_ext[k] * modal_gen_strct->options->relincr;
        }

    //   Parabolic fitting 

    //   1. Point extreme

        modal_gen_strct->lpk->x_ext[k] = modal_gen_strct->lpk->x_equil[k] + increment;
        lin_fail = MODAL_fct_modal(modal_gen_strct->lpk->x_ext, modal_gen_strct->n_x, modal_gen_strct->lpk->F_ext, mds_gen_strct, mbs_part, modal_gen_strct, mbs_aux, mbs_data, posvit);

        copy_double_vec(modal_gen_strct->lpk->x_ext, modal_gen_strct->lpk->x_mid, modal_gen_strct->n_x );

        while (rep)
        {

    //   2. Point milieu
            li = 0;
            modal_gen_strct->lpk->x_mid[k] = modal_gen_strct->lpk->x_equil[k] + increment/2;

            lin_fail = MODAL_fct_modal(modal_gen_strct->lpk->x_mid,modal_gen_strct->n_x, modal_gen_strct->lpk->F_mid, mds_gen_strct, mbs_part, modal_gen_strct, mbs_aux, mbs_data, posvit);

            for(i=0; i<modal_gen_strct->n_x; i++)
            {
                GK[i][k] = (-modal_gen_strct->lpk->F_ext[i] + 4.0 * modal_gen_strct->lpk->F_mid[i] - 3.0 * 0.0) / increment;//modal_gen_strct->lpk->F_equil[i]
                                
                if(compt == 1)
                {
                    modal_gen_strct->lpk->GK_prem[i][k] = GK[i][k];
                }
                if (fabs(modal_gen_strct->lpk->F_ext[i]) < modal_gen_strct->options->equitol && fabs(modal_gen_strct->lpk->F_mid[i]) < modal_gen_strct->options->equitol)
                {
                    li++;
                    GK[i][k] = 0.0;
                }

            }
            if (li == modal_gen_strct->n_x)
            {
                if (modal_gen_strct->options->verbose)
                {
                    printf("Column %d/%d    (no influence)\n",k,modal_gen_strct->n_x);
                }
                rep = 0;
            }
            else
            {
                max_GK_k = 0.0; 
                test = 1;
                for (i=0; i<modal_gen_strct->n_x; i++)
                {
                    max_GK_k = MAX(max_GK_k,fabs(GK[i][k]));
                }
                for (i=0; i<modal_gen_strct->n_x; i++)
                {
                    if (fabs(GK[i][k] / max_GK_k) > 1e-6 && fabs(GK[i][k]) > 1e-8)
                    {
                        if (fabs((modal_gen_strct->lpk->GK_comp[i][k]-GK[i][k])/GK[i][k]) > modal_gen_strct->options->lintol)
                        {
                            test = 0;
                        }
                    }
                }
                if (test)
                {
                    compt++;
                    for (i=0; i<modal_gen_strct->n_x; i++)
                    {
                        modal_gen_strct->lpk->GK_comp[i][k] = GK[i][k];
                    }
                    increment = increment / 2.0;
                    copy_double_vec(modal_gen_strct->lpk->F_mid,modal_gen_strct->lpk->F_ext,i<modal_gen_strct->n_x);
                    if (compt > 10)
                    {
                        if (modal_gen_strct->options->verbose)
                        {
                            printf("Column %d/%d    (waiting...)\n",k,modal_gen_strct->n_x);
                        }
                        modal_gen_strct->lpk->diverge_ind[modal_gen_strct->lpk->n_diverge]=k;
                        modal_gen_strct->lpk->n_diverge++;
                        rep = 0;
                    }
                }
                else
                {
                    if (modal_gen_strct->lpk->maxcomp < max_GK_k)
                    {
                        modal_gen_strct->lpk->maxcomp = max_GK_k;
                    }
                    if (modal_gen_strct->options->verbose)
                    {
                        printf("Column %d/%d    (%d iterations)\n",k,modal_gen_strct->n_x, compt);
                    }
                    rep = 0;
                }
            }
        }
    }

    //    colonnes en attente : on se base sur maxcomp pour éventuellement les accepter

    if(modal_gen_strct->lpk->n_diverge > 0)
    {
        if (modal_gen_strct->options->verbose)
        {
            printf(">>MBS>> coming back to the waiting columns\n");
        }
        for(k=0; k<modal_gen_strct->lpk->n_diverge; k++)
        {
            max_GK_prem_div_k = 0.0;
            for (i=0; i<modal_gen_strct->n_x; i++)
            {
                max_GK_prem_div_k = MAX(max_GK_prem_div_k, fabs(modal_gen_strct->lpk->GK_prem[i][modal_gen_strct->lpk->diverge_ind[k]]));
            }
            if (max_GK_prem_div_k < 1e-5 * modal_gen_strct->lpk->maxcomp)
            {
                for (i=0; i<modal_gen_strct->n_x; i++)
                {
                    GK[i][modal_gen_strct->lpk->diverge_ind[k]] = 0.0;
                }
                if (modal_gen_strct->options->verbose)
                {
                    printf(">>MBS>> column ',num2str(diverge[k]),'/',num2str(nbrl),'    has converged\n");
                }
            }
            else
            {
                printf(">>MBS>> column ',num2str(diverge[k]),'/',num2str(nbrl),'   no convergence after 5 iterations\n");
                printf(">>MBS>> Restart with other parameters !\n");
                lin_fail = 1;
                return lin_fail;
            }
        }
    }
    lin_fail = 0;
    return lin_fail;
}

int MODAL_copy_x_mode( double** eigen_matrix_r, double** eigen_matrix_phi, int n_x, int ind, double *eigen_vector_r, double *eigen_vector_phi)
{
    int i;
    double r_max, phi_r_max; 
    int ind_r_max;
    r_max = 0.0;
    for(i=0; i<n_x; i++)
    {
        if(fabs(eigen_matrix_r[n_x+i][ind])>r_max)
        {
            r_max = fabs(eigen_matrix_r[n_x+i][ind]);
            ind_r_max = i;
        }
    }
    r_max = eigen_matrix_r[n_x+ind_r_max][ind];
    phi_r_max = eigen_matrix_phi[n_x+ind_r_max][ind];
    for(i=0; i<n_x; i++)
    {
        eigen_vector_r[i] = eigen_matrix_r[n_x+i][ind] / r_max;
        eigen_vector_phi[i] = eigen_matrix_phi[n_x+i][ind] - phi_r_max;
    }
    
    return 0; // signature should be changed if no return is needed
}



int MODAL_write_result(MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct* modal_gen_strct, char* filename)
{
    int i,j,k, ind_x; 
    char *q_type_char = "x";

    double q_v_cur_a, q_v_cur_b; 

    FILE  *fidR;
    
    // File declaration
    fidR = NULL; // internal filename

    // Opening file
    fidR = fopen(filename, "wt"); 

    // Fill the file
    if(fidR == NULL)
    {
        printf("error: cannot open file '%s'\n", filename);
        return EXIT_FAILURE;
    }  
    fprintf(fidR,"\n-------------------------------------------------------\n");
    fprintf(fidR,"Project            : %s\n","mbs_data->name");
    fprintf(fidR,"Date            : %s", get_time_machine()); 
    fprintf(fidR,"Process            : Modal Analysis\n");
    fprintf(fidR,"Computation        : Eigenmodes\n");
    fprintf(fidR,"-------------------------------------------------------\n");

    fprintf(fidR,"|---------------------------------------------------------------------------------------|\n");
    fprintf(fidR,"|    Eigenvalues                            |                                                |\n");
    fprintf(fidR,"|    #            re                im        |    nu(Hz)            ksi(%%)            nu0(Hz)        |\n");
    fprintf(fidR,"|---------------------------------------------------------------------------------------|\n");

    for(i=0; i<modal_gen_strct->n_mode; i++)
    {
        fprintf(fidR,"|    %d        %f        %f    |    %f        %f        %f    |\n",i+1, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->a, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->b, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->nu/(2* M_PI), modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->ksi *100.0, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->nu_0 /(2* M_PI)  );
    }
    fprintf(fidR,"|---------------------------------------------------------------------------------------|\n");

    fprintf(fidR," Details about modes: \n");
    fprintf(fidR," --------------------\n\n");

    for(i=0; i<modal_gen_strct->n_mode; i++)
    {
        fprintf(fidR," --------------------------------------------------------------------------------\n");
        fprintf(fidR," Eigenmode %d, (Eigen value # %d = %f + %f i )\n", i+1, i+1, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->a, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->b);
        switch(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->type)
        {
            case 1: 
                fprintf(fidR," Type : unoscillating_stable\n");
            break;
            case 2: 
                fprintf(fidR," Type : rigid\n");
            break;
            case 3: 
                fprintf(fidR," Type : unoscillating_unstable\n");
            break;
            case 4: 
                fprintf(fidR," Type : oscillating-damped\n");
            break;
            case 5: 
                fprintf(fidR," Type : oscillating-undamped\n");
            break;
            case 6: 
                fprintf(fidR," Type : oscillating-unstable\n");
            break;
        }
        fprintf(fidR," --------------------------------------------------------------------------------\n");

        fprintf(fidR," q-id        q-name                        q-type        q-r                q-phi\n");
        q_type_char = "u";
        ind_x=0;
        for(j=0; j<mbs_part->n_qu; j++)
        {
            fprintf(fidR," %d", mbs_part->ind_qu[j]+1); 
            fprintf(fidR,"            %-25s", mds_gen_strct->bodytree->joint_list[mbs_part->ind_qu[j]]->name );
            if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[j]) == -1)
            {
                fprintf(fidR,"    %s            %f        %f\n", q_type_char, modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x], modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x] );
                ind_x++;
            }
            else 
            {
                fprintf(fidR,"    %s-%s            %f        %f\n", q_type_char,"i", 0.0, 0.0 );
            }
        }
        q_type_char = "v";
        for(j=0; j<mbs_part->n_qv; j++)
        {
            fprintf(fidR," %d", mbs_part->ind_qv[j]+1); 
            fprintf(fidR,"            %-25s", mds_gen_strct->bodytree->joint_list[mbs_part->ind_qv[j]]->name );
            q_v_cur_a = 0.0; 
            q_v_cur_b = 0.0;
            ind_x=0;
            for(k=0; k< mbs_part->n_qu; k++)
            {
                if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[k]) == -1)
                {
                    q_v_cur_a += modal_gen_strct->Bvuc[j][k] * modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x] * cos(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x]); 
                    q_v_cur_b += modal_gen_strct->Bvuc[j][k] * modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x] * sin(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x]);
                    ind_x++;
                }
            }
            // need paul to know if c par of Bvuc is needed

            fprintf(fidR,"    %s            %f        %f\n", q_type_char, sqrt(q_v_cur_a*q_v_cur_a + q_v_cur_b*q_v_cur_b), atan2(q_v_cur_b, q_v_cur_a ));
        }
        fprintf(fidR,"\n\n\n");
    }
    fclose(fidR);

    return EXIT_SUCCESS;
}

int MODAL_save_anim(MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct* modal_gen_strct, char* filepath)
{
    int i, j, k, l, ind_x; 

    char filename[100]; // caution 
    FILE  *fidR;
    double *q_print, *q_mode_r, *q_mode_phi;
    int N, n_cycle;

    double t, delta_t, tf;
    double q_v_cur_a, q_v_cur_b, alpha;
    
    q_mode_r = get_double_vec(mds_gen_strct->bodytree->n_joint);
    q_mode_phi = get_double_vec(mds_gen_strct->bodytree->n_joint); 
    q_print = get_double_vec(mds_gen_strct->bodytree->n_joint);

    for(i=0; i<modal_gen_strct->n_mode; i++)
    {
        sprintf(filename, "%smode_%d.anim", filepath, i+1);
        //printf(filename);
        // File declaration
        fidR = NULL; // internal filename

        // Opening file
        fidR = fopen(filename, "wt"); 

        // Fill the file
        if(fidR == NULL)
        {
            printf("error: cannot open file '%s'\n", filename);
            return EXIT_FAILURE;
        }  

        t = 0.0;
        switch(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->type)
        {
            case 1: 
                alpha = 5.0;
                delta_t = 0.001;
                tf = 2.0; 
            break;
            case 2: 
                alpha = 0.0;
                delta_t = 0.001;
                tf = 2.0; 
            break;
            case 3: 
                alpha = 5.0;
                delta_t = 0.001;
                tf = 2.0; 
            break;
            case 4:
            case 5:
            case 6:
                alpha = 2.0 * M_PI;
                delta_t = 0.001;
                tf = 10.0; 
            break;
        }
        ind_x = 0;
        for(j=0; j<mbs_part->n_qu; j++)
        {
            if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[j]) == -1)
            {
                q_mode_r[mbs_part->ind_qu[j]] = modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x];
                q_mode_phi[mbs_part->ind_qu[j]] = modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x];
                ind_x++;
            }
            else
            {
                q_mode_r[mbs_part->ind_qu[j]] = 0.0;
                q_mode_phi[mbs_part->ind_qu[j]] = 0.0;
            }
        }
        for(j=0; j<mbs_part->n_qv; j++)
        {
            q_v_cur_a = 0.0; 
            q_v_cur_b = 0.0;
            ind_x = 0;
            for(k=0; k< mbs_part->n_qu; k++)
            {
                if (find_int_vec(modal_gen_strct->options->ignored_qu,modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[k]) == -1)
                {
                    q_v_cur_a += modal_gen_strct->Bvuc[j][k] * modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x] * cos(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x]); 
                    q_v_cur_b += modal_gen_strct->Bvuc[j][k] * modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_r[ind_x] * sin(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->eigen_vector_phi[ind_x]);
                    ind_x++;
                }
            }
            q_mode_r[mbs_part->ind_qv[j]] = sqrt(q_v_cur_a*q_v_cur_a + q_v_cur_b*q_v_cur_b);
            q_mode_phi[mbs_part->ind_qv[j]] = atan2(q_v_cur_b, q_v_cur_a );
        }
        for(j=0; j<mds_gen_strct->bodytree->n_joint; j++)
        {
            q_mode_r[j] *= modal_gen_strct->options->mode_ampl;
        }
        while(t < tf)
        {    
            for(k=0; k<mds_gen_strct->bodytree->n_joint; k++)
            {
                switch(modal_gen_strct->mode_list[modal_gen_strct->sorted_mode_ind[i]]->type)
                {
                    case 1: 
                        q_print[k] = modal_gen_strct->q_save[k] + q_mode_r[k] * cos(q_mode_phi[k]) * exp(-alpha * t);
                    break;
                    case 2: 
                        q_print[k] = modal_gen_strct->q_save[k] + q_mode_r[k] * t;
                    break;
                    case 3:
                        q_print[k] = modal_gen_strct->q_save[k] + q_mode_r[k] * cos(q_mode_phi[k]) * exp(alpha * (t-tf));
                    break;
                    case 4:
                    case 5:
                    case 6:
                        q_print[k] = modal_gen_strct->q_save[k] + q_mode_r[k] * cos(q_mode_phi[k] + alpha * t);
                    break;
                }
            }
            fprintf(fidR,"% 12.8f", t);
            for(k=0; k<mds_gen_strct->bodytree->n_joint; k++)
            {
                fprintf(fidR,"  %12.8f", q_print[k]);
            }
            fprintf(fidR,"\n");

            t += delta_t;
        }
        fclose(fidR);
    }
    free_double_vec(q_mode_r);
    free_double_vec(q_mode_phi);
    free_double_vec(q_print);
    
    
    return 0; // signature should be changed if no return is needed
}

int MODAL_run_modal(MDS_gen_strct *mds_gen_strct, MbsPart *mbs_part, MODAL_gen_strct *modal_gen_strct,    MbsAux *mbs_aux, MbsData *mbs_data)
{
    int i, j, ind_x, ind_mode, ind_x_1, ind_x_2; 
    int posvit, lin_fail;

    double d; 
    double *sol_1, *sol_2; 
    int *ind_prov;

    double min_a, min_b; 
    int ind_cur, next_type;


    mbs_data->process = 2;

    // Test de prerequis
     
    if (mbs_data->DoneEquil == 0)
    {
        printf(">>MBS>> mbs_run_modal : please perform an equilibrium at first !\n");
        return EXIT_FAILURE;
    }
    if (mbs_part->n_qu == 0)
    {
        printf(">>MBS>> mbs_run_modal : no independent variable - irrelevant process !\n");
        return EXIT_FAILURE;
    }

    if (modal_gen_strct->options->verbose)
    {
        printf(">>MBS>> ***** mbs_run_modal begin *****\n");
    }

    copy_double_vec(&(mbs_data->q[1]), modal_gen_strct->q_save, mds_gen_strct->bodytree->n_joint);
    copy_double_vec(&(mbs_data->qd[1]), modal_gen_strct->qd_save, mds_gen_strct->bodytree->n_joint);


    // 2. Mass matrix computation

    if (modal_gen_strct->options->verbose)
    {
        printf(">>MBS>> mbs_modal : Computation of the mass matrix Mr\n");
    }

    dirdynared(mbs_aux,mbs_data);

    ind_x = 0;
    for(i=0; i<mbs_part->n_qu; i++)
    {
        if (find_int_vec(modal_gen_strct->options->ignored_qu, modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[i]) == -1)
        {
            modal_gen_strct->lpk->F_equil[ind_x] = mbs_aux->Fr[mbs_part->ind_qu[i]+1] ;
            ind_x++;
        }
    }
    /*
    printf(" -------------------\n "); 
    print_double_vec(modal_gen_strct->lpk->F_equil, mbs_part->n_qu);
    printf(" -------------------\n "); 
    */
    if (mbs_data->DoneEquil != 1)
    {
        printf(">>MBS>> Mass matrix computation not successfull\n");
        //return EXIT_FAILURE;
    }

    // Result
    for(i=0; i<mbs_part->n_qu; i++)
    {
        for(j=i; j<mbs_part->n_qu; j++)
        {
            modal_gen_strct->Mr[i][j] = mbs_aux->Mr[i+1][j+1];
            modal_gen_strct->Mr[j][i] = mbs_aux->Mr[i+1][j+1]; // take account the symetrie because cholc(call in dyrdynared) destroy the lower part of Mr ! 
        }
    }
    ind_x_1 = 0; 
    for(i=0; i<mbs_part->n_qu; i++)
    {
        if (find_int_vec(modal_gen_strct->options->ignored_qu, modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[i]) == -1)
        {
            ind_x_2 = 0;
            for(j=0; j<mbs_part->n_qu; j++)
            {
                if (find_int_vec(modal_gen_strct->options->ignored_qu, modal_gen_strct->options->n_ignored_qu, mbs_part->ind_qu[j]) == -1)
                {
                    modal_gen_strct->Mr_xx[ind_x_1][ind_x_2] = modal_gen_strct->Mr[i][j];
                    ind_x_2++;
                }
            }
            ind_x_1++;
        }
    }

    for(i=0; i<mbs_part->n_qv; i++)
    {
        for(j=0; j<(mbs_part->n_qu + mds_gen_strct->bodytree->n_qc); j++)
        {
            modal_gen_strct->Bvuc[i][j] = mbs_aux->Bvuc[i+1][j+1];
        }
    }

    //  3. System linearization

    copy_double_vec(modal_gen_strct->q_save, &(mbs_data->q[1]), mds_gen_strct->bodytree->n_joint);
    copy_double_vec(modal_gen_strct->qd_save, &(mbs_data->qd[1]), mds_gen_strct->bodytree->n_joint);

    //    Kr = delta Fr/delta u

    posvit = 1;
    if(modal_gen_strct->options->verbose)
    {
        printf(">>MBS>> mbs_modal : Computation of the tangent matrix Kr\n");
    }
    lin_fail = MODAL_linearipk(modal_gen_strct->Kr, mds_gen_strct, mbs_part, modal_gen_strct,    mbs_aux, mbs_data, posvit);

    copy_double_vec(modal_gen_strct->q_save, &(mbs_data->q[1]), mds_gen_strct->bodytree->n_joint);
    copy_double_vec(modal_gen_strct->qd_save, &(mbs_data->qd[1]), mds_gen_strct->bodytree->n_joint);

    //    Gr = delta Fr/delta ud
    
    posvit = 2;
    if(modal_gen_strct->options->verbose)
    {
        printf(">>MBS>> mbs_modal : Computation of the tangent matrix Gr\n");
    }
    lin_fail = MODAL_linearipk(modal_gen_strct->Gr, mds_gen_strct, mbs_part, modal_gen_strct,    mbs_aux, mbs_data, posvit);

    copy_double_vec(modal_gen_strct->q_save, &(mbs_data->q[1]), mds_gen_strct->bodytree->n_joint);
    copy_double_vec(modal_gen_strct->qd_save, &(mbs_data->qd[1]), mds_gen_strct->bodytree->n_joint);

    save_double_tab(modal_gen_strct->Kr, modal_gen_strct->n_x, modal_gen_strct->n_x, "Kr_debug.txt"); 
    save_double_tab(modal_gen_strct->Gr, modal_gen_strct->n_x, modal_gen_strct->n_x, "Gr_debug.txt"); 

    if(lin_fail > 0)
    { 
        return EXIT_FAILURE;
    }
     
    //  4. Modal analysis

    //  Eigen system
    sol_1 = get_double_vec(modal_gen_strct->n_x);
    sol_2 = get_double_vec(modal_gen_strct->n_x);

    ind_prov = get_int_vec(modal_gen_strct->n_x);

    ludcmp_2(modal_gen_strct->Mr_xx, modal_gen_strct->n_x, ind_prov , &d);

    for(i=0; i<modal_gen_strct->n_x; i++)
    {
        for(j=0; j<modal_gen_strct->n_x; j++)
        {
            sol_1[j] = modal_gen_strct->Gr[j][i];
            sol_2[j] = modal_gen_strct->Kr[j][i];
        }
        lubksb_2(modal_gen_strct->Mr_xx, modal_gen_strct->n_x, ind_prov , sol_1);
        lubksb_2(modal_gen_strct->Mr_xx, modal_gen_strct->n_x, ind_prov , sol_2);

        for(j=0; j<modal_gen_strct->n_x; j++)
        {
            modal_gen_strct->B[j][i] = -sol_1[j];// = modal_gen_strct->Kr[j][i];
            modal_gen_strct->B[j][i+modal_gen_strct->n_x] = -sol_2[j];//sol_2[j] = modal_gen_strct->Gr[j][i];
        }
        modal_gen_strct->B[modal_gen_strct->n_x + i][i] = 1.0;
    }

    free_double_vec(sol_1);
    free_double_vec(sol_2);
    free_int_vec(ind_prov);

    MODAL_eig(modal_gen_strct->B, 2 * modal_gen_strct->n_x, modal_gen_strct->eigen_value_a, modal_gen_strct->eigen_value_b, modal_gen_strct->eigen_matrix_r, modal_gen_strct->eigen_matrix_phi);

    //save_double_tab(modal_gen_strct->B, 2 * modal_gen_strct->n_x, 2 * modal_gen_strct->n_x, "B.txt"); 

    ind_mode = 0;
    for(i=0; i<2*modal_gen_strct->n_x; i++)
    {
        if(fabs(modal_gen_strct->eigen_value_b[i])<1e-6)
        {
            if(modal_gen_strct->eigen_value_a[i]<-1e-6)
            {
                modal_gen_strct->mode_list[ind_mode]->type = 1;
            }
            else if(modal_gen_strct->eigen_value_a[i]<1e-6)
            {
                modal_gen_strct->mode_list[ind_mode]->type = 2;
            }
            else
            {
                modal_gen_strct->mode_list[ind_mode]->type = 3;
            }
        }
        else
        {
            if(modal_gen_strct->eigen_value_a[i]<-1e-6)
            {
                modal_gen_strct->mode_list[ind_mode]->type = 4;
                modal_gen_strct->mode_list[ind_mode]->ksi = cos(atan(modal_gen_strct->eigen_value_b[i]/modal_gen_strct->eigen_value_a[i]));
                modal_gen_strct->mode_list[ind_mode]->nu_0 = sqrt(modal_gen_strct->eigen_value_a[i]*modal_gen_strct->eigen_value_a[i] + modal_gen_strct->eigen_value_b[i]*modal_gen_strct->eigen_value_b[i]);
                modal_gen_strct->mode_list[ind_mode]->nu = modal_gen_strct->mode_list[ind_mode]->nu_0 * sqrt(1.0-modal_gen_strct->mode_list[ind_mode]->ksi*modal_gen_strct->mode_list[ind_mode]->ksi);    
            }
            else if(modal_gen_strct->eigen_value_a[i]<1e-6)
            {
                modal_gen_strct->mode_list[ind_mode]->type = 5;
                modal_gen_strct->mode_list[ind_mode]->ksi = 0.0;
                modal_gen_strct->mode_list[ind_mode]->nu_0 = fabs(modal_gen_strct->eigen_value_b[i]);
                modal_gen_strct->mode_list[ind_mode]->nu = fabs(modal_gen_strct->eigen_value_b[i]);
            }
            else
            {
                modal_gen_strct->mode_list[ind_mode]->type = 6;
            }
        }
        modal_gen_strct->mode_list[ind_mode]->a = modal_gen_strct->eigen_value_a[i];
        modal_gen_strct->mode_list[ind_mode]->b = modal_gen_strct->eigen_value_b[i];
        MODAL_copy_x_mode( modal_gen_strct->eigen_matrix_r, modal_gen_strct->eigen_matrix_phi, modal_gen_strct->n_x, i, modal_gen_strct->mode_list[ind_mode]->eigen_vector_r, modal_gen_strct->mode_list[ind_mode]->eigen_vector_phi);
        ind_mode++;
    }
    modal_gen_strct->n_mode = ind_mode;

    ind_mode = 0;
    for(i=0; i<modal_gen_strct->n_mode; i++)
    {
        modal_gen_strct->sorted_mode_ind[i] = -1;
    }

    for(i=1; i<=6; i++) // type
    {
        next_type = 0;
        while(next_type != 1)
        {
            min_a = DBL_MAX;
            min_b = DBL_MAX;
            ind_cur = -1;
            for(j=0; j<modal_gen_strct->n_mode; j++) //
            {
                if(modal_gen_strct->mode_list[j]->type == i && find_int_vec(modal_gen_strct->sorted_mode_ind, ind_mode, j) == -1)
                {
                    if(modal_gen_strct->mode_list[j]->a < min_a)
                    {
                        min_a = modal_gen_strct->mode_list[j]->a;
                        min_b = modal_gen_strct->mode_list[j]->b;
                        ind_cur = j;
                    }
                    else if(modal_gen_strct->mode_list[j]->a == min_a)
                    {
                        if(modal_gen_strct->mode_list[j]->b <= min_b)
                        {
                            min_b = modal_gen_strct->mode_list[j]->b;
                            ind_cur = j;
                        }
                    }
                }
            }
            if(ind_cur == -1)
            {
                next_type = 1;
            }
            else
            {
                modal_gen_strct->sorted_mode_ind[ind_mode] = ind_cur;
                ind_mode++;
            }
        }
    }
    if(modal_gen_strct->options->save_mat)
    {
        save_double_tab(modal_gen_strct->Kr, modal_gen_strct->n_x, modal_gen_strct->n_x, "/../../resultsR/Kr.txt"); 
        save_double_tab(modal_gen_strct->Gr, modal_gen_strct->n_x, modal_gen_strct->n_x, "/../../resultsR/Gr.txt"); 
        save_double_tab(modal_gen_strct->B, 2 * modal_gen_strct->n_x, 2 * modal_gen_strct->n_x, "/../../resultsR/B.txt");
    }
    if(modal_gen_strct->options->save_result)
    {
        MODAL_write_result(mds_gen_strct, mbs_part, modal_gen_strct, "/../../resultsR/modal_result.txt");
    }
    if(modal_gen_strct->options->save_anim)
    {
        MODAL_save_anim(mds_gen_strct, mbs_part, modal_gen_strct, "/../../animationR/");
    }

    if (modal_gen_strct->options->verbose) 
    {
        printf(">>MBS>> ***** mbs_run_modal end *****\n"); 
    }

    mbs_data->process = 0;
    mbs_data->DoneModal = 1;
    return 0;  // function signature should be change if no return is needed
}


MODAL_option_strct* init_MODAL_option_strct(void)
{
    MODAL_option_strct *modal_option_strct; 

    modal_option_strct = (MODAL_option_strct*) malloc(sizeof(MODAL_option_strct));

    modal_option_strct->time = 0.0;
    modal_option_strct->lintol = 1e-3;
    modal_option_strct->relincr = 1e-2;
    modal_option_strct->absincr = 1e-3;
    modal_option_strct->mode_ampl = 0.2;
    modal_option_strct->equitol = 1e-6;
    modal_option_strct->save_mat = 0;
    modal_option_strct->save_result = 0;
    modal_option_strct->save_anim = 0;
    modal_option_strct->renamefile = 0;
    modal_option_strct->verbose = 1;
    modal_option_strct->clearmbsglobal = 1;

    modal_option_strct->n_ignored_qu = 0;
    modal_option_strct->ignored_qu = NULL;

    return modal_option_strct;
}
void free_MODAL_option_strct(MODAL_option_strct* modal_option_strct)
{
    free_int_vec(modal_option_strct->ignored_qu);

    free(modal_option_strct);
}

MODAL_lpk_strct* init_MODAL_lpk_strct(int n_x)
{
    MODAL_lpk_strct *modal_lpk_strct; 

    modal_lpk_strct = (MODAL_lpk_strct*) malloc(sizeof(MODAL_lpk_strct));

    modal_lpk_strct->diverge_ind = get_int_vec(n_x);
    modal_lpk_strct->n_diverge = 0;
    modal_lpk_strct->maxcomp = 0.0;

    modal_lpk_strct->x_equil = get_double_vec(n_x);
    modal_lpk_strct->x_mid = get_double_vec(n_x);
    modal_lpk_strct->x_ext = get_double_vec(n_x);

    modal_lpk_strct->F_equil = get_double_vec(n_x);
    modal_lpk_strct->F_mid = get_double_vec(n_x);
    modal_lpk_strct->F_ext = get_double_vec(n_x);

    modal_lpk_strct->GK_comp = get_double_tab(n_x, n_x);
    modal_lpk_strct->GK_prem= get_double_tab(n_x, n_x);

    return modal_lpk_strct;
}
void free_MODAL_lpk_strct(MODAL_lpk_strct* modal_lpk_strct, int n_x)
{
    free_int_vec(modal_lpk_strct->diverge_ind);

    free_double_vec(modal_lpk_strct->x_equil);
    free_double_vec(modal_lpk_strct->x_mid);
    free_double_vec(modal_lpk_strct->x_ext);

    free_double_vec(modal_lpk_strct->F_equil);
    free_double_vec(modal_lpk_strct->F_mid);
    free_double_vec(modal_lpk_strct->F_ext);

    free_double_tab(modal_lpk_strct->GK_comp, n_x);
    free_double_tab(modal_lpk_strct->GK_prem, n_x);

    free(modal_lpk_strct);
}

MODAL_mode_strct* init_MODAL_mode_strct(int n_x)
{ 
    MODAL_mode_strct *modal_mode_strct; 

    modal_mode_strct = (MODAL_mode_strct*) malloc(sizeof(MODAL_mode_strct));

    modal_mode_strct->type = 0;

    modal_mode_strct->a = 0.0; 
    modal_mode_strct->b = 0.0;

    modal_mode_strct->ksi = 0.0;
    modal_mode_strct->nu_0 = 0.0;
    modal_mode_strct->nu = 0.0;

    modal_mode_strct->eigen_vector_r = get_double_vec(n_x); 
    modal_mode_strct->eigen_vector_phi = get_double_vec(n_x); 

    return modal_mode_strct;
}
void free_MODAL_mode_strct(MODAL_mode_strct* modal_mode_strct)
{
    free_double_vec(modal_mode_strct->eigen_vector_r);
    free_double_vec(modal_mode_strct->eigen_vector_phi);

    free(modal_mode_strct);
}

void init_MODAL_gen_strct_SD(MODAL_gen_strct* modal_gen_strct, int n_x)
{
    int i;
    modal_gen_strct->Mr_xx = get_double_tab(n_x, n_x);

    modal_gen_strct->Kr = get_double_tab(n_x, n_x);
    modal_gen_strct->Gr = get_double_tab(n_x, n_x);

    modal_gen_strct->B = get_double_tab(2 * n_x, 2 * n_x); 

    modal_gen_strct->eigen_value_a = get_double_vec(2*n_x);
    modal_gen_strct->eigen_value_b = get_double_vec(2*n_x);

    modal_gen_strct->eigen_matrix_r = get_double_tab(2*n_x, 2*n_x);
    modal_gen_strct->eigen_matrix_phi = get_double_tab(2*n_x, 2*n_x);

    modal_gen_strct->n_mode = 0;
    modal_gen_strct->mode_list = (MODAL_mode_strct**) malloc(2*n_x*sizeof(MODAL_mode_strct*));
    for(i=0; i<2*n_x; i++)
    {
        modal_gen_strct->mode_list[i] = init_MODAL_mode_strct(n_x);
    }
    modal_gen_strct->sorted_mode_ind = get_int_vec(2*n_x);
}

MODAL_gen_strct* init_MODAL_gen_strct(MDS_gen_strct*  mds_gen_strct, MbsPart *mbs_part)
{ 
    MODAL_gen_strct *modal_gen_strct; 

    modal_gen_strct = (MODAL_gen_strct*) malloc(sizeof(MODAL_gen_strct));

    modal_gen_strct->options = init_MODAL_option_strct();
//    MODAL_get_options_from_user(modal_gen_strct->options);

    modal_gen_strct->q_save = get_double_vec(mds_gen_strct->bodytree->n_joint);
    modal_gen_strct->qd_save = get_double_vec(mds_gen_strct->bodytree->n_joint);

    modal_gen_strct->Mr = get_double_tab(mbs_part->n_qu, mds_gen_strct->bodytree->n_qu);
    modal_gen_strct->Bvuc = get_double_tab(mbs_part->n_qv, mbs_part->n_qu + mds_gen_strct->bodytree->n_qc);

    modal_gen_strct->n_x = mbs_part->n_qu - modal_gen_strct->options->n_ignored_qu;

    init_MODAL_gen_strct_SD(modal_gen_strct, modal_gen_strct->n_x);
    modal_gen_strct->lpk = init_MODAL_lpk_strct(modal_gen_strct->n_x);

    return modal_gen_strct;
}
void free_MODAL_gen_strct_SD(MODAL_gen_strct* modal_gen_strct, int n_x)
{
    int i;
    free_double_tab(modal_gen_strct->Mr_xx, n_x);

    free_double_tab(modal_gen_strct->Kr, n_x);
    free_double_tab(modal_gen_strct->Gr, n_x);

    free_double_tab(modal_gen_strct->B, 2 * n_x); 

    free_double_vec(modal_gen_strct->eigen_value_a); 
    free_double_vec(modal_gen_strct->eigen_value_b); 

    free_double_tab(modal_gen_strct->eigen_matrix_r, 2*n_x);
    free_double_tab(modal_gen_strct->eigen_matrix_phi, 2*n_x);

    for(i=0; i<2*n_x; i++)
    {
         free_MODAL_mode_strct(modal_gen_strct->mode_list[i]);
    }
    free(modal_gen_strct->mode_list);

    free_int_vec(modal_gen_strct->sorted_mode_ind);
}
void free_MODAL_gen_strct(MODAL_gen_strct* modal_gen_strct, MbsPart *mbs_part)
{
    if(modal_gen_strct != NULL)
    {
        free_MODAL_option_strct(modal_gen_strct->options);

        free_double_vec(modal_gen_strct->q_save);
        free_double_vec(modal_gen_strct->qd_save);

        free_double_tab(modal_gen_strct->Mr, mbs_part->n_qu); // caution with partitionning
        free_double_tab(modal_gen_strct->Bvuc, mbs_part->n_qv);

        free_MODAL_lpk_strct(modal_gen_strct->lpk, modal_gen_strct->n_x);
        free_MODAL_gen_strct_SD(modal_gen_strct, modal_gen_strct->n_x);

        free(modal_gen_strct);
    }
}
