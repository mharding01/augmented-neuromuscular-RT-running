/**
 * @file mbs_set.h
 *
 *
 * Creation date: 11/06/2015
 * @author Olivier Lantsoght (based on the work of other from CEREM)
 *
 *
 * (c) Universite catholique de Louvain
 */

#ifndef MBS_SET_h
#define MBS_SET_h

#include "mbs_data.h"

#ifdef __cplusplus
extern "C" {
#endif
     /**
      * Set the number of user constraints and update the related structures
      *
      * @param[in,out] mbs_data Robotran structure
      * @param[in] Nuserc number of user constraints
      */
    void mbs_set_nb_userc(MbsData *mbs_data, int Nuserc);

    void mbs_set_qu(MbsData *mbs_data, int qu);
    void mbs_set_qv(MbsData *mbs_data, int qv);
    void mbs_set_qdriven(MbsData *mbs_data, int qdriven);
    void mbs_set_qlocked(MbsData *mbs_data, int qlocked);

    void print_mbs_q_all(MbsData *mbs_data);
    void print_mbs_q_vec(int *q_vec, int nq);
#ifdef __cplusplus
}
#endif

int* add_mbs_q_elem(int *q_vec, int nq, int new_q, int *new_nq);
int* remove_mbs_q_elem(int *q_vec, int nq, int old_q, int *new_nq);
void udpate_nq(MbsData *mbs_data, int new_nqu, int new_nqv, int new_nqc, int new_nqdriven, int new_nqlocked);

#endif
