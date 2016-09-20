/* --------------------------------------------------------
 * This code was generated automatically by MBsysC modules.
 * MBsysC modules are distributed as part of the ROBOTRAN 
 * software. They provides functionalities for dealing with
 * symbolic equations generated by ROBOTRAN. 
 *
 * More info on www.robotran.be 
 *
 * Universite catholique de Louvain, Belgium 
 *
 * Last update : Tue May 19 13:31:40 2015
 * --------------------------------------------------------
 *
 */
#include "user_model.h"
#include "mbs_xml_reader.h"

// ============================================================ //

/*! \brief generate new UserModel structure
 * 
 * \return initialized UserModel structure
 */
UserModel* mbs_new_user_model() 
{
	UserModel* um;
	um = (UserModel*)malloc(sizeof(UserModel));

	return um;
}

/*! \brief release memory for the UserModel structure
 * 
 * \param[out] um UserModel structure to release
 */
void mbs_delete_user_model(UserModel* um) 
{
	free(um);
}

/*! \brief loas user model from MBS
 * 
 * \param[in,out] gen MDS generation structure
 * \param[in,out] um UserModel structure
 */
void mbs_load_user_model_xml(MDS_gen_strct* gen, UserModel* um) 
{

}

// ============================================================ //
 
