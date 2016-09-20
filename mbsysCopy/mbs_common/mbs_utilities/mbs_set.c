/**
 * @file mbs_set.c
 *
 *
 * Creation date: 11/06/2015
 * @author Olivier Lantsoght (based on the work of other from CEREM)
 *
 *
 * (c) Universite catholique de Louvain
 */

#include "mbs_set.h"
#include <stdio.h>
#include <stdlib.h>

void mbs_set_nb_userc(MbsData *mbs_data, int Nuserc)
{
    int i;

    mbs_data->Nuserc = Nuserc;
    mbs_data->Ncons  = mbs_data->Nloopc + mbs_data->Nuserc;
    mbs_data->nhu    = mbs_data->Ncons;

    // re-initialize hu vector
    if(mbs_data->hu!=NULL)
    {
        free(mbs_data->hu);
    }
    mbs_data->hu = (int*) malloc((mbs_data->nhu+1)*sizeof(int));
    mbs_data->hu[0] = mbs_data->nhu;
    for (i=1; i<=mbs_data->nhu; i++)
    {
        mbs_data->hu[i] = i;
    }
}

/*! \brief set a variable to independent partition
 * 
 * \param[in,out] mbs_data Robotran structure
 * \param[in] qu variable to set to independent
 *
 * This function must be calle between 'mbs_load' and 'mbs_new_part' in 'main.c'
 */
void mbs_set_qu(MbsData *mbs_data, int qu)
{
    int new_nqu, new_nqv, new_nqc;
    int new_nqdriven, new_nqlocked;

    // add element
    mbs_data->qu = add_mbs_q_elem(mbs_data->qu, mbs_data->nqu, qu, &new_nqu);

    // remove element
    mbs_data->qv = remove_mbs_q_elem(mbs_data->qv, mbs_data->nqv, qu, &new_nqv);
    mbs_data->qc = remove_mbs_q_elem(mbs_data->qc, mbs_data->nqc, qu, &new_nqc);
    mbs_data->qdriven = remove_mbs_q_elem(mbs_data->qdriven, mbs_data->nqdriven, qu, &new_nqdriven);
    mbs_data->qlocked = remove_mbs_q_elem(mbs_data->qlocked, mbs_data->nqlocked, qu, &new_nqlocked);

    // update and check the vector size
    udpate_nq(mbs_data, new_nqu, new_nqv, new_nqc, new_nqdriven, new_nqlocked);
}

/*! \brief set a variable to dependent partition
 * 
 * \param[in,out] mbs_data Robotran structure
 * \param[in] qv variable to set to dependent
 *
 * This function must be calle between 'mbs_load' and 'mbs_new_part' in 'main.c'
 */
void mbs_set_qv(MbsData *mbs_data, int qv)
{
    int new_nqu, new_nqv, new_nqc;
    int new_nqdriven, new_nqlocked;

    // add element
    mbs_data->qv = add_mbs_q_elem(mbs_data->qv, mbs_data->nqv, qv, &new_nqv);

    // remove element
    mbs_data->qu = remove_mbs_q_elem(mbs_data->qu, mbs_data->nqu, qv, &new_nqu);
    mbs_data->qc = remove_mbs_q_elem(mbs_data->qc, mbs_data->nqc, qv, &new_nqc);
    mbs_data->qdriven = remove_mbs_q_elem(mbs_data->qdriven, mbs_data->nqdriven, qv, &new_nqdriven);
    mbs_data->qlocked = remove_mbs_q_elem(mbs_data->qlocked, mbs_data->nqlocked, qv, &new_nqlocked);

    // update and check the vector size
    udpate_nq(mbs_data, new_nqu, new_nqv, new_nqc, new_nqdriven, new_nqlocked);
}

/*! \brief set a variable to driven partition
 * 
 * \param[in,out] mbs_data Robotran structure
 * \param[in] qdriven variable to set to driven
 *
 * This function must be calle between 'mbs_load' and 'mbs_new_part' in 'main.c'
 */
void mbs_set_qdriven(MbsData *mbs_data, int qdriven)
{
    int new_nqu, new_nqv, new_nqc;
    int new_nqdriven, new_nqlocked;

    // add element
    mbs_data->qc = add_mbs_q_elem(mbs_data->qc, mbs_data->nqc, qdriven, &new_nqc);
    mbs_data->qdriven = add_mbs_q_elem(mbs_data->qdriven, mbs_data->nqdriven, qdriven, &new_nqdriven);

    // remove element
    mbs_data->qu = remove_mbs_q_elem(mbs_data->qu, mbs_data->nqu, qdriven, &new_nqu);
    mbs_data->qv = remove_mbs_q_elem(mbs_data->qv, mbs_data->nqv, qdriven, &new_nqv);
    mbs_data->qlocked = remove_mbs_q_elem(mbs_data->qlocked, mbs_data->nqlocked, qdriven, &new_nqlocked);

    // update and check the vector size
    udpate_nq(mbs_data, new_nqu, new_nqv, new_nqc, new_nqdriven, new_nqlocked);
}

/*! \brief set a variable to locked partition
 * 
 * \param[in,out] mbs_data Robotran structure
 * \param[in] qlocked variable to set to locked
 *
 * This function must be calle between 'mbs_load' and 'mbs_new_part' in 'main.c'
 */
void mbs_set_qlocked(MbsData *mbs_data, int qlocked)
{
    int new_nqu, new_nqv, new_nqc;
    int new_nqdriven, new_nqlocked;

    // add element
    mbs_data->qc = add_mbs_q_elem(mbs_data->qc, mbs_data->nqc, qlocked, &new_nqc);
    mbs_data->qlocked = add_mbs_q_elem(mbs_data->qlocked, mbs_data->nqlocked, qlocked, &new_nqlocked);

    // remove element
    mbs_data->qu = remove_mbs_q_elem(mbs_data->qu, mbs_data->nqu, qlocked, &new_nqu);
    mbs_data->qv = remove_mbs_q_elem(mbs_data->qv, mbs_data->nqv, qlocked, &new_nqv);
    mbs_data->qdriven = remove_mbs_q_elem(mbs_data->qdriven, mbs_data->nqdriven, qlocked, &new_nqdriven);

    // update and check the vector size
    udpate_nq(mbs_data, new_nqu, new_nqv, new_nqc, new_nqdriven, new_nqlocked);
}

/*! \brief update the nq variables
 * 
 * \param[in,out] mbs_data Robotran structure
 */
void udpate_nq(MbsData *mbs_data, int new_nqu, int new_nqv, int new_nqc, int new_nqdriven, int new_nqlocked)
{
    mbs_data->nqu = new_nqu;
    mbs_data->nqv = new_nqv;
    mbs_data->nqc = new_nqc;
    mbs_data->nqdriven = new_nqdriven;
    mbs_data->nqlocked = new_nqlocked;

    // check
    if (mbs_data->nqdriven + mbs_data->nqlocked != mbs_data->nqc)
    {
        printf("Error: nqdriven (%d) + nqlocked (%d) != nqc (%d) !\n", mbs_data->nqdriven, mbs_data->nqlocked, mbs_data->nqc);
        exit(EXIT_FAILURE);
    }

    if (mbs_data->nqu + mbs_data->nqv + mbs_data->nqc != mbs_data->njoint)
    {
        printf("Error: nqu (%d) + nqv (%d) + nqc (%d) != njoint (%d) !\n", mbs_data->nqu, mbs_data->nqv, mbs_data->nqc, mbs_data->njoint);
        exit(EXIT_FAILURE);
    }
}

/*! \brief add a new index in a q vector
 * 
 * \param[in,out] q_vec old q vector
 * \param[in] nq number of elements of the old vector
 * \param[in] new_q new index to add
 * \param[out] new_nq new 'nq' value
 * \return new allocated vector or the same vector if the new element cannot be added
 *
 * the old vector memory is released (except when returning the same vector)
 */
int* add_mbs_q_elem(int *q_vec, int nq, int new_q, int *new_nq)
{
    int i, index;
    int *new_vec;

    // nothing yet in this vector
    if (!nq)
    {
        // safety
        if (q_vec != NULL)
        {
            free(q_vec);
        }

        // create new vector
        new_vec = (int*) calloc(2,sizeof(int));

        new_vec[0] = 1;
        new_vec[1] = new_q;

        *new_nq = 1;

        return new_vec;
    }

    index = -1;
    
    // loop on all vector elements
    for(i=1; i<=nq; i++)
    {
        // safety
        if (q_vec[i] == new_q)
        {
            printf("Warning: index %d already in this vector\n", new_q);
            *new_nq = nq;
            return q_vec;
        }
        else if (q_vec[i] > new_q) // new place for index found
        {
            index = i;
            break;
        }
    }

    // create new vector
    new_vec = (int*) calloc(nq+2,sizeof(int));

    new_vec[0] = nq+1;

    *new_nq = nq+1;

    // add at the end
    if (index == -1)
    {
        for(i=1; i<=nq; i++)
        {
            new_vec[i] = q_vec[i];
        }

        new_vec[nq+1] = new_q;
    }
    else // add before the end
    {
        for(i=1; i<index; i++)
        {
            new_vec[i] = q_vec[i];
        }

        new_vec[index] = new_q;

        for(i=index+1; i<=nq+1; i++)
        {
            new_vec[i] = q_vec[i-1];
        }
    }

    free(q_vec);

    return new_vec;
}

/*! \brief remove an old index in a q vector
 * 
 * \param[in,out] q_vec old q vector
 * \param[in] nq number of elements of the old vector
 * \param[in] old_q old index to remove
 * \param[out] new_nq new 'nq' value
 * \return new allocated vector or the same vector if the old element to remove cannot be found
 *
 * the old vector memory is released (except when returning the same vector)
 */
int* remove_mbs_q_elem(int *q_vec, int nq, int old_q, int *new_nq)
{
    int i, index;
    int *new_vec;

    // vector not allocated
    if (!nq)
    {
        *new_nq = 0;

        return NULL;
    }

    index = -1;

    // loop on all vector elements
    for(i=1; i<=nq; i++)
    {
        if (q_vec[i] == old_q)
        {
            index = i;
            break;
        }
    }

    // nothing to remove in this vector
    if (index == -1)
    {
        *new_nq = nq;
        return q_vec;
    }

    // create new vector
    new_vec = (int*) calloc(nq,sizeof(int));

    new_vec[0] = nq-1;

    *new_nq = nq-1;

    // fill vector
    for(i=1; i<index; i++)
    {
        new_vec[i] = q_vec[i];
    }

    for(i=index; i<=nq-1; i++)
    {
        new_vec[i] = q_vec[i+1];
    }

    free(q_vec);

    return new_vec;
}

/*! \brief print the vectors qu, qv, qc, qdriven and qlocked
 *
 * \param[in] mbs_data Robotran structure
 */
void print_mbs_q_all(MbsData *mbs_data)
{
    printf("qu    ->   ");
    print_mbs_q_vec(mbs_data->qu, mbs_data->nqu);
    printf("qv    ->   ");
    print_mbs_q_vec(mbs_data->qv, mbs_data->nqv);
    printf("qc    ->   ");
    print_mbs_q_vec(mbs_data->qc, mbs_data->nqc);
    printf("qdriven -> ");
    print_mbs_q_vec(mbs_data->qdriven, mbs_data->nqdriven);
    printf("qlocked -> ");
    print_mbs_q_vec(mbs_data->qlocked, mbs_data->nqlocked);
}

/*! \brief print a vector of indexes with first elem in brackets
 *
 * \param[in] q_vec vector to print
 * \param[in] nq size of the vector - 1
 */
void print_mbs_q_vec(int *q_vec, int nq)
{
    int i;

    if (nq)
    {
        printf("nb: %d - q: [%d]", nq, q_vec[0]);

        for(i=1; i<=nq; i++)
        {
            printf(" ; %d", q_vec[i]);
        }

        printf("\n");
    }
    else
    {
        printf("NULL\n");
    }
}
