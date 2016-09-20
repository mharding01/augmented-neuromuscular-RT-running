#ifndef mbs_load_xml_h
#define mbs_load_xml_h

#include "useful_functions.h"
#include "mbs_xml_reader.h"
#include "mbs_data.h"

#ifdef __cplusplus
extern "C" {
#endif
 /**
  * Load the data from the given *.mbs file (data in the xml 
  * format). The memory of a new MbsData is allocated.                                          
  */
MbsData* mbs_load(const char* mbs_filename);

 /**
  * Free the memory used by the given MbsData structure.
  */
void mbs_delete_data(MbsData *s);
#ifdef __cplusplus
}
#endif

 /**
  * Load the user model data 
  */
void mbs_load_user_model_xml(MDS_gen_strct* gen, UserModel* ums);

 /**
  * Retrieve a MbsData structure from the given MDS_gen_strct
  */
MbsData* MDS_create_MBSdataStruct(MDS_gen_strct* mds_gen_strct);


#endif