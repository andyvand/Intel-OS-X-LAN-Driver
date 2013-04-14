/*
 *  e1000.c
 *  AppleIntelE1000e
 *
 *
 */


#include "e1000.h"

static struct {
	u16 devid;
	u16 boardid;
} e1000e_pci_tbl[] = {
	{ E1000_DEV_ID_82571EB_COPPER, board_82571 },
	{ E1000_DEV_ID_82571EB_FIBER, board_82571 },
	{ E1000_DEV_ID_82571EB_QUAD_COPPER, board_82571 },
	{ E1000_DEV_ID_82571EB_QUAD_COPPER_LP, board_82571 },
	{ E1000_DEV_ID_82571EB_QUAD_FIBER, board_82571 },
	{ E1000_DEV_ID_82571EB_SERDES, board_82571 },
	{ E1000_DEV_ID_82571EB_SERDES_DUAL, board_82571 },
	{ E1000_DEV_ID_82571EB_SERDES_QUAD, board_82571 },
	{ E1000_DEV_ID_82571PT_QUAD_COPPER, board_82571 },
	
	{ E1000_DEV_ID_82572EI, board_82572 },
	{ E1000_DEV_ID_82572EI_COPPER, board_82572 },
	{ E1000_DEV_ID_82572EI_FIBER, board_82572 },
	{ E1000_DEV_ID_82572EI_SERDES, board_82572 },
	
	{ E1000_DEV_ID_82573E, board_82573 },
	{ E1000_DEV_ID_82573E_IAMT, board_82573 },
	{ E1000_DEV_ID_82573L, board_82573 },
	
	{ E1000_DEV_ID_82574L, board_82574 },
	{ E1000_DEV_ID_82574LA, board_82574 },
	{ E1000_DEV_ID_82583V, board_82583 },
	
	{ E1000_DEV_ID_80003ES2LAN_COPPER_DPT, board_80003es2lan },
	{ E1000_DEV_ID_80003ES2LAN_COPPER_SPT, board_80003es2lan },
	{ E1000_DEV_ID_80003ES2LAN_SERDES_DPT, board_80003es2lan },
	{ E1000_DEV_ID_80003ES2LAN_SERDES_SPT, board_80003es2lan },
	
	{ E1000_DEV_ID_ICH8_IFE, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IFE_G, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IFE_GT, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IGP_AMT, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IGP_C, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IGP_M, board_ich8lan },
	{ E1000_DEV_ID_ICH8_IGP_M_AMT, board_ich8lan },
	{ E1000_DEV_ID_ICH8_82567V_3, board_ich8lan },
	
	{ E1000_DEV_ID_ICH9_IFE, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IFE_G, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IFE_GT, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IGP_AMT, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IGP_C, board_ich9lan },
	{ E1000_DEV_ID_ICH9_BM, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IGP_M, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IGP_M_AMT, board_ich9lan },
	{ E1000_DEV_ID_ICH9_IGP_M_V, board_ich9lan },
	
	{ E1000_DEV_ID_ICH10_R_BM_LM, board_ich9lan },
	{ E1000_DEV_ID_ICH10_R_BM_LF, board_ich9lan },
	{ E1000_DEV_ID_ICH10_R_BM_V, board_ich9lan },
	
	{ E1000_DEV_ID_ICH10_D_BM_LM, board_ich10lan },
	{ E1000_DEV_ID_ICH10_D_BM_LF, board_ich10lan },
	{ E1000_DEV_ID_ICH10_D_BM_V, board_ich10lan },
	
	{ E1000_DEV_ID_PCH_M_HV_LM, board_pchlan },
	{ E1000_DEV_ID_PCH_M_HV_LC, board_pchlan },
	{ E1000_DEV_ID_PCH_D_HV_DM, board_pchlan },
	{ E1000_DEV_ID_PCH_D_HV_DC, board_pchlan },
	
	{ E1000_DEV_ID_PCH2_LV_LM, board_pch2lan },
	{ E1000_DEV_ID_PCH2_LV_V, board_pch2lan },

	{ E1000_DEV_ID_PCH_LPT_I217_LM, board_pch_lpt},
	{ E1000_DEV_ID_PCH_LPT_I217_V, board_pch_lpt},
	{ E1000_DEV_ID_PCH_LPTLP_I218_LM, board_pch_lpt},
	{ E1000_DEV_ID_PCH_LPTLP_I218_V, board_pch_lpt},

	{ 0, 0 }	/* terminate list */
};

static const struct e1000_info *e1000_info_tbl[] = {
	[board_82571] = &e1000_82571_info,
	[board_82572] = &e1000_82572_info,
	[board_82573] = &e1000_82573_info,
	[board_82574] = &e1000_82574_info,
	[board_82583] = &e1000_82583_info,
	[board_80003es2lan] = &e1000_es2_info,
	[board_ich8lan] = &e1000_ich8_info,
	[board_ich9lan] = &e1000_ich9_info,
	[board_ich10lan] = &e1000_ich10_info,
	[board_pchlan] = &e1000_pch_info,
	[board_pch2lan] = &e1000_pch2_info,
	[board_pch_lpt] = &e1000_pch_lpt_info,
};


static u16 find_device_type( u16 pci_dev_id ){
	for(int k = 0; k < sizeof(e1000e_pci_tbl)/sizeof(e1000e_pci_tbl[0]); k++ ){
		if(e1000e_pci_tbl[k].devid == pci_dev_id )
			return e1000e_pci_tbl[k].boardid;
	}
	return 0;
}

const struct e1000_info* e1000_probe( u16 pci_dev_id )
{
	u16 board_id = find_device_type(pci_dev_id);
	return e1000_info_tbl[board_id];
}

s32 e1000_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	return -E1000_ERR_CONFIG;
}

// Mutex used in ich8lan.c
IOLock* swflag_mutex;
IOLock* nvm_mutex;

