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

	{ 0, 0 }	/* terminate list */
};


static s32 e1000_get_variants_82571(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	static int global_quad_port_a; /* global port a indication */
	struct pci_dev *pdev = adapter->pdev;
	int is_port_b = er32(STATUS) & E1000_STATUS_FUNC_1;
	
	/* tag quad port adapters first, it's used below */
	switch (pdev->device) {
		case E1000_DEV_ID_82571EB_QUAD_COPPER:
		case E1000_DEV_ID_82571EB_QUAD_FIBER:
		case E1000_DEV_ID_82571EB_QUAD_COPPER_LP:
		case E1000_DEV_ID_82571PT_QUAD_COPPER:
			adapter->flags |= FLAG_IS_QUAD_PORT;
			/* mark the first port */
			if (global_quad_port_a == 0)
				adapter->flags |= FLAG_IS_QUAD_PORT_A;
			/* Reset for multiple quad port adapters */
			global_quad_port_a++;
			if (global_quad_port_a == 4)
				global_quad_port_a = 0;
			break;
		default:
			break;
	}
	
	switch (adapter->hw.mac.type) {
		case e1000_82571:
			/* these dual ports don't have WoL on port B at all */
			if (((pdev->device == E1000_DEV_ID_82571EB_FIBER) ||
				 (pdev->device == E1000_DEV_ID_82571EB_SERDES) ||
				 (pdev->device == E1000_DEV_ID_82571EB_COPPER)) &&
				(is_port_b))
				adapter->flags &= ~FLAG_HAS_WOL;
			/* quad ports only support WoL on port A */
			if (adapter->flags & FLAG_IS_QUAD_PORT &&
				(!(adapter->flags & FLAG_IS_QUAD_PORT_A)))
				adapter->flags &= ~FLAG_HAS_WOL;
			/* Does not support WoL on any port */
			if (pdev->device == E1000_DEV_ID_82571EB_SERDES_QUAD)
				adapter->flags &= ~FLAG_HAS_WOL;
			break;
			
		case e1000_82573:
			if (pdev->device == E1000_DEV_ID_82573L) {
				adapter->flags |= FLAG_HAS_JUMBO_FRAMES;
				adapter->max_hw_frame_size = DEFAULT_JUMBO;
			}
			break;
			
		default:
			break;
	}
	
	return 0;
}

static struct e1000_info e1000_82571_info = {
	.mac = e1000_82571,
	.flags = FLAG_HAS_HW_VLAN_FILTER
    | FLAG_HAS_JUMBO_FRAMES | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD | FLAG_HAS_SMART_POWER_DOWN | FLAG_RESET_OVERWRITES_LAA	/* errata */
    | FLAG_TARC_SPEED_MODE_BIT	/* errata */
    | FLAG_APME_CHECK_PORT_B,
	.flags2 = FLAG2_DISABLE_ASPM_L1	/* errata 13 */
    | FLAG2_DMA_BURST,
	.pba = 38,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_82571,
	.get_variants = e1000_get_variants_82571,
};

static struct e1000_info e1000_82572_info = {
	.mac = e1000_82572,
	.flags = FLAG_HAS_HW_VLAN_FILTER
    | FLAG_HAS_JUMBO_FRAMES | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD | FLAG_TARC_SPEED_MODE_BIT,	/* errata */
	.flags2 = FLAG2_DISABLE_ASPM_L1	/* errata 13 */
    | FLAG2_DMA_BURST,
	.pba = 38,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_82571,
	.get_variants = e1000_get_variants_82571,
};

static struct e1000_info e1000_82573_info = {
	.mac = e1000_82573,
	.flags = FLAG_HAS_HW_VLAN_FILTER | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_SMART_POWER_DOWN | FLAG_HAS_AMT | FLAG_HAS_SWSM_ON_LOAD,
	.flags2 = FLAG2_DISABLE_ASPM_L1 | FLAG2_DISABLE_ASPM_L0S,
	.pba = 20,
	.max_hw_frame_size = ETH_FRAME_LEN + ETH_FCS_LEN,
	.init_ops = e1000_init_function_pointers_82571,
	.get_variants = e1000_get_variants_82571,
};

static struct e1000_info e1000_82574_info = {
	.mac = e1000_82574,
	.flags = FLAG_HAS_HW_VLAN_FILTER
    | FLAG_HAS_MSIX
    | FLAG_HAS_JUMBO_FRAMES | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_SMART_POWER_DOWN
    | FLAG_HAS_AMT | FLAG_HAS_CTRLEXT_ON_LOAD,
	.flags2 = FLAG2_CHECK_PHY_HANG
	| FLAG2_DISABLE_ASPM_L0S
	| FLAG2_DISABLE_ASPM_L1 | FLAG2_NO_DISABLE_RX | FLAG2_DMA_BURST,
	.pba = 32,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_82571,
	.get_variants = e1000_get_variants_82571,
};

static struct e1000_info e1000_82583_info = {
	.mac = e1000_82583,
	.flags = FLAG_HAS_HW_VLAN_FILTER | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_SMART_POWER_DOWN
    | FLAG_HAS_AMT | FLAG_HAS_JUMBO_FRAMES | FLAG_HAS_CTRLEXT_ON_LOAD,
	.flags2 = FLAG2_DISABLE_ASPM_L0S | FLAG2_NO_DISABLE_RX,
	.pba = 32,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_82571,
	.get_variants = e1000_get_variants_82571,
};

static struct e1000_info e1000_es2_info = {
	.mac = e1000_80003es2lan,
	.flags = FLAG_HAS_HW_VLAN_FILTER
    | FLAG_HAS_JUMBO_FRAMES | FLAG_HAS_WOL | FLAG_APME_IN_CTRL3
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD | FLAG_RX_NEEDS_RESTART	/* errata */
    | FLAG_TARC_SET_BIT_ZERO	/* errata */
    | FLAG_APME_CHECK_PORT_B | FLAG_DISABLE_FC_PAUSE_TIME,	/* errata */
	.flags2 = FLAG2_DMA_BURST,
	.pba = 38,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_80003es2lan,
	.get_variants = NULL,
};


static s32 e1000_get_variants_ich8lan(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
    
	/*
	 * Disable Jumbo Frame support on parts with Intel 10/100 PHY or
	 * on parts with MACsec enabled in NVM (reflected in CTRL_EXT).
	 */
	if ((adapter->hw.phy.type == e1000_phy_ife) ||
	    ((adapter->hw.mac.type >= e1000_pch2lan) &&
	     (!(er32(CTRL_EXT) & E1000_CTRL_EXT_LSECCK)))) {
            adapter->flags &= ~FLAG_HAS_JUMBO_FRAMES;
            adapter->max_hw_frame_size = ETH_FRAME_LEN + ETH_FCS_LEN;
            
            hw->mac.ops.blink_led = NULL;
        }
    
	if ((adapter->hw.mac.type == e1000_ich8lan) &&
	    (adapter->hw.phy.type != e1000_phy_ife))
		adapter->flags |= FLAG_LSC_GIG_SPEED_DROP;
    
	/* Enable workaround for 82579 w/ ME enabled */
	if ((adapter->hw.mac.type == e1000_pch2lan) &&
		(er32(FWSM) & E1000_ICH_FWSM_FW_VALID))
		adapter->flags2 |= FLAG2_PCIM2PCI_ARBITER_WA;
    
	return 0;
}

static struct e1000_info e1000_ich8_info = {
	.mac = e1000_ich8lan,
	.flags = FLAG_HAS_WOL | FLAG_IS_ICH
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD
    | FLAG_HAS_AMT | FLAG_HAS_FLASH | FLAG_APME_IN_WUC,
	.pba = 8,
	.max_hw_frame_size = ETH_FRAME_LEN + ETH_FCS_LEN,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_ich9_info = {
	.mac = e1000_ich9lan,
	.flags = FLAG_HAS_JUMBO_FRAMES | FLAG_IS_ICH | FLAG_HAS_WOL
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD
    | FLAG_HAS_AMT | FLAG_HAS_FLASH | FLAG_APME_IN_WUC,
	.pba = 18,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_ich10_info = {
	.mac = e1000_ich10lan,
	.flags = FLAG_HAS_JUMBO_FRAMES | FLAG_IS_ICH | FLAG_HAS_WOL
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD
    | FLAG_HAS_AMT | FLAG_HAS_FLASH | FLAG_APME_IN_WUC,
	.pba = 18,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_pch_info = {
	.mac = e1000_pchlan,
	.flags = FLAG_IS_ICH | FLAG_HAS_WOL
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD | FLAG_HAS_AMT | FLAG_HAS_FLASH | FLAG_HAS_JUMBO_FRAMES | FLAG_DISABLE_FC_PAUSE_TIME	/* errata */
    | FLAG_APME_IN_WUC,
	.flags2 = FLAG2_HAS_PHY_STATS,
	.pba = 26,
	.max_hw_frame_size = 4096,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_pch2_info = {
	.mac = e1000_pch2lan,
	.flags = FLAG_IS_ICH | FLAG_HAS_WOL
#ifndef HAVE_NDO_SET_FEATURES
    | FLAG_RX_CSUM_ENABLED
#endif
    | FLAG_HAS_CTRLEXT_ON_LOAD
    | FLAG_HAS_AMT
    | FLAG_HAS_FLASH | FLAG_HAS_JUMBO_FRAMES | FLAG_APME_IN_WUC,
	.flags2 = FLAG2_HAS_PHY_STATS | FLAG2_HAS_EEE,
	.pba = 26,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_pch_lpt_info = {
	.mac = e1000_pch_lpt,
	.flags = FLAG_IS_ICH | FLAG_HAS_WOL
#ifndef HAVE_NDO_SET_FEATURES
	| FLAG_RX_CSUM_ENABLED
#endif
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH | FLAG_HAS_JUMBO_FRAMES | FLAG_APME_IN_WUC,
	.flags2 = FLAG2_HAS_PHY_STATS | FLAG2_HAS_EEE,
	.pba = 26,
	.max_hw_frame_size = DEFAULT_JUMBO,
	.init_ops = e1000_init_function_pointers_ich8lan,
	.get_variants = e1000_get_variants_ich8lan,
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

