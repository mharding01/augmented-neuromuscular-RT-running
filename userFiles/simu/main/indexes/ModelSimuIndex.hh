/*! 
 * \author Nicolas Van der Noot
 * \file ModelSimuIndex.hh
 * \brief ModelSimuIndex class
 */
#ifndef _MODEL_SIMU_INDEX_HH_
#define _MODEL_SIMU_INDEX_HH_

#include <vector>
#include "SimuOptions.h"
#include "mbs_data.h"
#include "SimuIndex.hh"

/*! \brief indexes corresponding to the ones used in the chosen model
 *
 * In comparison, "SimuIndex.hh" provides all the indexes, even the ones not used
 */
class ModelSimuIndex
{
	public:
		ModelSimuIndex(MbsData *mbs_data, SimuOptions *options);
		~ModelSimuIndex();

		int get_nb_joints() const { return joint_indexes.size(); }

		int get_joint_indexes(int i) const { return joint_indexes[i]; }
		int get_body_indexes(int i)  const { return body_indexes[i];  }
		int get_Ssens_indexes(int i) const { return Ssens_indexes[i]; }
		int get_Fsens_indexes(int i) const { return Fsens_indexes[i]; }
		int get_link_indexes(int i)  const { return link_indexes[i];  }

		int get_inv_joint_indexes(int i) const { return inv_joint_indexes[i]; }
		int get_inv_body_indexes(int i)  const { return inv_body_indexes[i];  }
		int get_inv_Ssens_indexes(int i) const { return inv_Ssens_indexes[i]; }
		int get_inv_Fsens_indexes(int i) const { return inv_Fsens_indexes[i]; }
		int get_inv_link_indexes(int i)  const { return inv_link_indexes[i];  }

		int get_mbs_joint_indexes(int i) const { return mbs_joint_indexes[i]; }
		int get_mbs_body_indexes(int i)  const { return mbs_body_indexes[i];  }
		int get_mbs_Ssens_indexes(int i) const { return mbs_Ssens_indexes[i]; }
		int get_mbs_Fsens_indexes(int i) const { return mbs_Fsens_indexes[i]; }
		int get_mbs_link_indexes(int i)  const { return mbs_link_indexes[i];  }

		int get_inv_mbs_joint_indexes(int i) const { return inv_mbs_joint_indexes[i]; }
		int get_inv_mbs_body_indexes(int i)  const { return inv_mbs_body_indexes[i];  }
		int get_inv_mbs_Ssens_indexes(int i) const { return inv_mbs_Ssens_indexes[i]; }
		int get_inv_mbs_Fsens_indexes(int i) const { return inv_mbs_Fsens_indexes[i]; }
		int get_inv_mbs_link_indexes(int i)  const { return inv_mbs_link_indexes[i];  }

		/// get the MBS index for the joint with the index 'index'
		int get_mbs_jt(int index)   const { return get_mbs_joint_indexes(get_inv_joint_indexes(index)); }
		int get_mbs_bd(int index)   const { return get_mbs_body_indexes(get_inv_body_indexes(index)); }
		int get_mbs_S(int index)    const { return get_mbs_Ssens_indexes(get_inv_Ssens_indexes(index)); }
		int get_mbs_F(int index)    const { return get_mbs_Fsens_indexes(get_inv_Fsens_indexes(index)); }
		int get_mbs_link(int index) const { return get_mbs_link_indexes(get_inv_link_indexes(index)); }

		int get_ind_F(int mbs_index) const { return get_Fsens_indexes(get_inv_mbs_Fsens_indexes(mbs_index)); }

		// get the name for MBS index
		const char* get_name_mbs_jt(int index)   const { return SimuJointIndex::get_index_name(get_joint_indexes(get_inv_mbs_joint_indexes(index))); }
		const char* get_name_mbs_bd(int index)   const { return SimuBodyIndex::get_index_name(get_body_indexes(get_inv_mbs_body_indexes(index))); }
		const char* get_name_mbs_S(int index)    const { return SimuSsensIndex::get_index_name(get_Ssens_indexes(get_inv_mbs_Ssens_indexes(index))); }
		const char* get_name_mbs_F(int index)    const { return SimuFsensIndex::get_index_name(get_Fsens_indexes(get_inv_mbs_Fsens_indexes(index))); }
		const char* get_name_mbs_link(int index) const { return SimuLinkIndex::get_index_name(get_link_indexes(get_inv_mbs_link_indexes(index))); }

	private:
		std::vector<int> joint_indexes; ///< joint indexes
		std::vector<int> body_indexes;  ///< body indexes
		std::vector<int> Ssens_indexes; ///< S sensors indexes
		std::vector<int> Fsens_indexes; ///< F sensors indexes
		std::vector<int> link_indexes;  ///< link indexes

		std::vector<int> inv_joint_indexes; ///< inverse of the 'joint_indexes' vector
		std::vector<int> inv_body_indexes;  ///< inverse of the 'body_indexes' vector
		std::vector<int> inv_Ssens_indexes; ///< inverse of the 'Ssens_indexes' vector
		std::vector<int> inv_Fsens_indexes; ///< inverse of the 'Fsens_indexes' vector
		std::vector<int> inv_link_indexes;  ///< inverse of the 'link_indexes' vector

		std::vector<int> mbs_joint_indexes; ///< joint indexes (MBS model)
		std::vector<int> mbs_body_indexes;  ///< body indexes (MBS model)
		std::vector<int> mbs_Ssens_indexes; ///< S sensors indexes (MBS model)
		std::vector<int> mbs_Fsens_indexes; ///< F sensors indexes (MBS model)
		std::vector<int> mbs_link_indexes;  ///< link indexes (MBS model)

		std::vector<int> inv_mbs_joint_indexes; ///< inverse of the 'mbs_joint_indexes' vector
		std::vector<int> inv_mbs_body_indexes;  ///< inverse of the 'mbs_body_indexes' vector
		std::vector<int> inv_mbs_Ssens_indexes; ///< inverse of the 'mbs_Ssens_indexes' vector
		std::vector<int> inv_mbs_Fsens_indexes; ///< inverse of the 'mbs_Fsens_indexes' vector
		std::vector<int> inv_mbs_link_indexes;  ///< inverse of the 'mbs_link_indexes' vector

		int coman_model; ///< COMAN model
		MbsData *mbs_data;    ///< Robotran structure
		SimuOptions *options; ///< simulation options

		// function prototypes
		int get_mbs_joint_index(int joint_id);
		int get_mbs_body_index(int body_id);
		int get_mbs_Ssens_index(int Ssens_id);
		int get_mbs_Fsens_index(int Fsens_id);
		int get_mbs_link_index(int link_id);
		void check_sizes();
		void mbs_indexes();
		void inverse_create();
		
		void generate_inverse(std::vector<int>& index_vec, std::vector<int>& inv_index_vec);

		void joint_add(int simu_id) { joint_indexes.push_back(simu_id); }
		void body_add(int simu_id)  { body_indexes.push_back(simu_id); }
		void Ssens_add(int simu_id) { Ssens_indexes.push_back(simu_id); }
		void Fsens_add(int simu_id) { Fsens_indexes.push_back(simu_id); }
		void link_add(int simu_id)  { link_indexes.push_back(simu_id); }
};

#endif
