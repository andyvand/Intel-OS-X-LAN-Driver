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

	{ 0 }	/* terminate list */
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
		case e1000_82574:
		case e1000_82583:
#if	0
			/* Disable ASPM L0s due to hardware errata */
			e1000e_disable_aspm(adapter->pdev, PCIE_LINK_STATE_L0S);
#endif

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
	.mac			= e1000_82571,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_RESET_OVERWRITES_LAA /* errata */
	| FLAG_TARC_SPEED_MODE_BIT /* errata */
	| FLAG_APME_CHECK_PORT_B,
	.flags2			= FLAG2_DISABLE_ASPM_L1, /* errata 13 */
	.pba			= 38,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_82571,
	.get_variants		= e1000_get_variants_82571,
};

static struct e1000_info e1000_82572_info = {
	.mac			= e1000_82572,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_TARC_SPEED_MODE_BIT, /* errata */
	.flags2			= FLAG2_DISABLE_ASPM_L1, /* errata 13 */
	.pba			= 38,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_82571,
	.get_variants		= e1000_get_variants_82571,
};

static struct e1000_info e1000_82573_info = {
	.mac			= e1000_82573,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_SWSM_ON_LOAD,
	.pba			= 20,
	.max_hw_frame_size	= ETH_FRAME_LEN + ETH_FCS_LEN,
	.init_ops		= e1000_init_function_pointers_82571,
	.get_variants		= e1000_get_variants_82571,
};

static struct e1000_info e1000_82574_info = {
	.mac			= e1000_82574,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
#ifdef CONFIG_E1000E_MSIX
	| FLAG_HAS_MSIX
#endif
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG2_CHECK_PHY_HANG, /*errata */
	.pba			= 32,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_82571,
	.get_variants		= e1000_get_variants_82571,
};

static struct e1000_info e1000_82583_info = {
	.mac			= e1000_82583,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_CTRLEXT_ON_LOAD,
	.pba			= 32,
	.max_hw_frame_size	= ETH_FRAME_LEN + ETH_FCS_LEN,
	.init_ops		= e1000_init_function_pointers_82571,
	.get_variants		= e1000_get_variants_82571,
};

static struct e1000_info e1000_es2_info = {
	.mac			= e1000_80003es2lan,
	.flags			= FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_RX_NEEDS_RESTART /* errata */
	| FLAG_TARC_SET_BIT_ZERO /* errata */
	| FLAG_APME_CHECK_PORT_B
	| FLAG_DISABLE_FC_PAUSE_TIME /* errata */
	| FLAG_TIPG_MEDIUM_FOR_80003ESLAN,
	.pba			= 38,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_80003es2lan,
	.get_variants		= NULL,
};

static s32 e1000_get_variants_ich8lan(struct e1000_adapter *adapter)
{
	if (adapter->hw.phy.type == e1000_phy_ife) {
		adapter->flags &= ~FLAG_HAS_JUMBO_FRAMES;
		adapter->max_hw_frame_size = ETH_FRAME_LEN + ETH_FCS_LEN;
	}
	
	if ((adapter->hw.mac.type == e1000_ich8lan) &&
	    (adapter->hw.phy.type == e1000_phy_igp_3))
		adapter->flags |= FLAG_LSC_GIG_SPEED_DROP;
	
	return 0;
}

static struct e1000_info e1000_ich8_info = {
	.mac			= e1000_ich8lan,
	.flags			= FLAG_HAS_WOL
	| FLAG_IS_ICH
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	.pba			= 8,
	.max_hw_frame_size	= ETH_FRAME_LEN + ETH_FCS_LEN,
	.init_ops		= e1000_init_function_pointers_ich8lan,
	.get_variants		= e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_ich9_info = {
	.mac			= e1000_ich9lan,
	.flags			= FLAG_HAS_JUMBO_FRAMES
	| FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_ERT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	.pba			= 10,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_ich8lan,
	.get_variants		= e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_ich10_info = {
	.mac			= e1000_ich10lan,
	.flags			= FLAG_HAS_JUMBO_FRAMES
	| FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_ERT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	.pba			= 10,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_ich8lan,
	.get_variants		= e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_pch_info = {
	.mac			= e1000_pchlan,
	.flags			= FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_DISABLE_FC_PAUSE_TIME /* errata */
	| FLAG_APME_IN_WUC,
	.flags2			= FLAG2_HAS_PHY_STATS,
	.pba			= 26,
	.max_hw_frame_size	= 4096,
	.init_ops		= e1000_init_function_pointers_ich8lan,
	.get_variants		= e1000_get_variants_ich8lan,
};

static struct e1000_info e1000_pch2_info = {
	.mac			= e1000_pch2lan,
	.flags			= FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_APME_IN_WUC,
	.flags2			= FLAG2_HAS_PHY_STATS
	| FLAG2_HAS_EEE,
	.pba			= 18,
	.max_hw_frame_size	= DEFAULT_JUMBO,
	.init_ops		= e1000_init_function_pointers_ich8lan,
	.get_variants		= e1000_get_variants_ich8lan,
};

static const struct e1000_info *e1000_info_tbl[] = {
	[board_82571]		= &e1000_82571_info,
	[board_82572]		= &e1000_82572_info,
	[board_82573]		= &e1000_82573_info,
	[board_82574]		= &e1000_82574_info,
	[board_82583]		= &e1000_82583_info,
	[board_80003es2lan]	= &e1000_es2_info,
	[board_ich8lan]		= &e1000_ich8_info,
	[board_ich9lan]		= &e1000_ich9_info,
	[board_ich10lan]	= &e1000_ich10_info,
	[board_pchlan]		= &e1000_pch_info,
	[board_pch2lan]		= &e1000_pch2_info,
};

#if	0
struct e1000_reg_info {
	u32 ofs;
	char *name;
};

#define E1000_RDFH	0x02410 /* Rx Data FIFO Head - RW */
#define E1000_RDFT	0x02418 /* Rx Data FIFO Tail - RW */
#define E1000_RDFHS	0x02420 /* Rx Data FIFO Head Saved - RW */
#define E1000_RDFTS	0x02428 /* Rx Data FIFO Tail Saved - RW */
#define E1000_RDFPC	0x02430 /* Rx Data FIFO Packet Count - RW */

#define E1000_TDFH	0x03410 /* Tx Data FIFO Head - RW */
#define E1000_TDFT	0x03418 /* Tx Data FIFO Tail - RW */
#define E1000_TDFHS	0x03420 /* Tx Data FIFO Head Saved - RW */
#define E1000_TDFTS	0x03428 /* Tx Data FIFO Tail Saved - RW */
#define E1000_TDFPC	0x03430 /* Tx Data FIFO Packet Count - RW */

static const struct e1000_reg_info e1000_reg_info_tbl[] = {
	
	/* General Registers */
	{E1000_CTRL, "CTRL"},
	{E1000_STATUS, "STATUS"},
	{E1000_CTRL_EXT, "CTRL_EXT"},
	
	/* Interrupt Registers */
	{E1000_ICR, "ICR"},
	
	/* RX Registers */
	{E1000_RCTL, "RCTL"},
	{E1000_RDLEN(0), "RDLEN"},
	{E1000_RDH(0), "RDH"},
	{E1000_RDT(0), "RDT"},
	{E1000_RDTR, "RDTR"},
	{E1000_RXDCTL(0), "RXDCTL"},
	{E1000_ERT, "ERT"},
	{E1000_RDBAL(0), "RDBAL"},
	{E1000_RDBAH(0), "RDBAH"},
	{E1000_RDFH, "RDFH"},
	{E1000_RDFT, "RDFT"},
	{E1000_RDFHS, "RDFHS"},
	{E1000_RDFTS, "RDFTS"},
	{E1000_RDFPC, "RDFPC"},
	
	/* TX Registers */
	{E1000_TCTL, "TCTL"},
	{E1000_TDBAL(0), "TDBAL"},
	{E1000_TDBAH(0), "TDBAH"},
	{E1000_TDLEN(0), "TDLEN"},
	{E1000_TDH(0), "TDH"},
	{E1000_TDT(0), "TDT"},
	{E1000_TIDV, "TIDV"},
	{E1000_TXDCTL(0), "TXDCTL"},
	{E1000_TADV, "TADV"},
	{E1000_TARC(0), "TARC"},
	{E1000_TDFH, "TDFH"},
	{E1000_TDFT, "TDFT"},
	{E1000_TDFHS, "TDFHS"},
	{E1000_TDFTS, "TDFTS"},
	{E1000_TDFPC, "TDFPC"},
	
	/* List Terminator */
	{}
};

/*
 * e1000_regdump - register printout routine
 */
static void e1000_regdump(struct e1000_hw *hw, struct e1000_reg_info *reginfo)
{
	int n = 0;
	char rname[16];
	u32 regs[8];
	
	switch (reginfo->ofs) {
		case E1000_RXDCTL(0):
			for (n = 0; n < 2; n++)
				regs[n] = __er32(hw, E1000_RXDCTL(n));
			break;
		case E1000_TXDCTL(0):
			for (n = 0; n < 2; n++)
				regs[n] = __er32(hw, E1000_TXDCTL(n));
			break;
		case E1000_TARC(0):
			for (n = 0; n < 2; n++)
				regs[n] = __er32(hw, E1000_TARC(n));
			break;
		default:
			printk(KERN_INFO "%-15s %08x\n",
				   reginfo->name, __er32(hw, reginfo->ofs));
			return;
	}
	
	snprintf(rname, 16, "%s%s", reginfo->name, "[0-1]");
	printk(KERN_INFO "%-15s ", rname);
	for (n = 0; n < 2; n++)
		printk(KERN_CONT "%08x ", regs[n]);
	printk(KERN_CONT "\n");
}


/*
 * e1000e_dump - Print registers, tx-ring and rx-ring
 */
static void e1000e_dump(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_reg_info *reginfo;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_tx_desc *tx_desc;
	struct my_u0 { u64 a; u64 b; } *u0;
	struct e1000_buffer *buffer_info;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	union e1000_rx_desc_packet_split *rx_desc_ps;
	struct e1000_rx_desc *rx_desc;
	struct my_u1 { u64 a; u64 b; u64 c; u64 d; } *u1;
	u32 staterr;
	int i = 0;
	
	if (!netif_msg_hw(adapter))
		return;
	
	/* Print netdevice Info */
	if (netdev) {
		dev_info(pci_dev_to_dev(adapter->pdev), "Net device Info\n");
		printk(KERN_INFO "Device Name     state            "
			   "trans_start      last_rx\n");
		printk(KERN_INFO "%-15s %016lX %016lX %016lX\n",
			   netdev->name,
			   netdev->state,
			   netdev->trans_start,
			   netdev->last_rx);
	}
	
	/* Print Registers */
	dev_info(pci_dev_to_dev(adapter->pdev), "Register Dump\n");
	printk(KERN_INFO " Register Name   Value\n");
	for (reginfo = (struct e1000_reg_info *)e1000_reg_info_tbl;
	     reginfo->name; reginfo++) {
		e1000_regdump(hw, reginfo);
	}
	
	/* Print TX Ring Summary */
	if (!netdev || !netif_running(netdev))
		goto exit;
	
	dev_info(pci_dev_to_dev(adapter->pdev), "TX Rings Summary\n");
	printk(KERN_INFO "Queue [NTU] [NTC] [bi(ntc)->dma  ]"
		   " leng ntw timestamp\n");
	buffer_info = &tx_ring->buffer_info[tx_ring->next_to_clean];
	printk(KERN_INFO " %5d %5X %5X %016llX %04X %3X %016llX\n",
		   0, tx_ring->next_to_use, tx_ring->next_to_clean,
		   (u64)buffer_info->dma,
		   buffer_info->length,
		   buffer_info->next_to_watch,
		   (u64)buffer_info->time_stamp);
	
	/* Print TX Rings */
	if (!netif_msg_tx_done(adapter))
		goto rx_ring_summary;
	
	dev_info(pci_dev_to_dev(adapter->pdev), "TX Rings Dump\n");
	
	/* Transmit Descriptor Formats - DEXT[29] is 0 (Legacy) or 1 (Extended)
	 *
	 * Legacy Transmit Descriptor
	 *   +--------------------------------------------------------------+
	 * 0 |         Buffer Address [63:0] (Reserved on Write Back)       |
	 *   +--------------------------------------------------------------+
	 * 8 | Special  |    CSS     | Status |  CMD    |  CSO   |  Length  |
	 *   +--------------------------------------------------------------+
	 *   63       48 47        36 35    32 31     24 23    16 15        0
	 *
	 * Extended Context Descriptor (DTYP=0x0) for TSO or checksum offload
	 *   63      48 47    40 39       32 31             16 15    8 7      0
	 *   +----------------------------------------------------------------+
	 * 0 |  TUCSE  | TUCS0  |   TUCSS   |     IPCSE       | IPCS0 | IPCSS |
	 *   +----------------------------------------------------------------+
	 * 8 |   MSS   | HDRLEN | RSV | STA | TUCMD | DTYP |      PAYLEN      |
	 *   +----------------------------------------------------------------+
	 *   63      48 47    40 39 36 35 32 31   24 23  20 19                0
	 *
	 * Extended Data Descriptor (DTYP=0x1)
	 *   +----------------------------------------------------------------+
	 * 0 |                     Buffer Address [63:0]                      |
	 *   +----------------------------------------------------------------+
	 * 8 | VLAN tag |  POPTS  | Rsvd | Status | Command | DTYP |  DTALEN  |
	 *   +----------------------------------------------------------------+
	 *   63       48 47     40 39  36 35    32 31     24 23  20 19        0
	 */
	printk(KERN_INFO "Tl[desc]     [address 63:0  ] [SpeCssSCmCsLen]"
		   " [bi->dma       ] leng  ntw timestamp        bi->skb "
		   "<-- Legacy format\n");
	printk(KERN_INFO "Tc[desc]     [Ce CoCsIpceCoS] [MssHlRSCm0Plen]"
		   " [bi->dma       ] leng  ntw timestamp        bi->skb "
		   "<-- Ext Context format\n");
	printk(KERN_INFO "Td[desc]     [address 63:0  ] [VlaPoRSCm1Dlen]"
		   " [bi->dma       ] leng  ntw timestamp        bi->skb "
		   "<-- Ext Data format\n");
	for (i = 0; tx_ring->desc && (i < tx_ring->count); i++) {
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];
		u0 = (struct my_u0 *)tx_desc;
		printk(KERN_INFO "T%c[0x%03X]    %016llX %016llX %016llX "
			   "%04X  %3X %016llX %p",
		       (!(le64_to_cpu(u0->b) & (1<<29)) ? 'l' :
				((le64_to_cpu(u0->b) & (1<<20)) ? 'd' : 'c')), i,
		       le64_to_cpu(u0->a), le64_to_cpu(u0->b),
		       (u64)buffer_info->dma, buffer_info->length,
		       buffer_info->next_to_watch, (u64)buffer_info->time_stamp,
		       buffer_info->skb);
		if (i == tx_ring->next_to_use && i == tx_ring->next_to_clean)
			printk(KERN_CONT " NTC/U\n");
		else if (i == tx_ring->next_to_use)
			printk(KERN_CONT " NTU\n");
		else if (i == tx_ring->next_to_clean)
			printk(KERN_CONT " NTC\n");
		else
			printk(KERN_CONT "\n");
		
		if (netif_msg_pktdata(adapter) && buffer_info->dma != 0)
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS,
						   16, 1, phys_to_virt(buffer_info->dma),
						   buffer_info->length, true);
	}
	
	/* Print RX Rings Summary */
rx_ring_summary:
	dev_info(pci_dev_to_dev(adapter->pdev), "RX Rings Summary\n");
	printk(KERN_INFO "Queue [NTU] [NTC]\n");
	printk(KERN_INFO " %5d %5X %5X\n", 0,
		   rx_ring->next_to_use, rx_ring->next_to_clean);
	
	/* Print RX Rings */
	if (!netif_msg_rx_status(adapter))
		goto exit;
	
	dev_info(pci_dev_to_dev(adapter->pdev), "RX Rings Dump\n");
	switch (adapter->rx_ps_pages) {
		case 1:
		case 2:
		case 3:
			/* [Extended] Packet Split Receive Descriptor Format
			 *
			 *    +-----------------------------------------------------+
			 *  0 |                Buffer Address 0 [63:0]              |
			 *    +-----------------------------------------------------+
			 *  8 |                Buffer Address 1 [63:0]              |
			 *    +-----------------------------------------------------+
			 * 16 |                Buffer Address 2 [63:0]              |
			 *    +-----------------------------------------------------+
			 * 24 |                Buffer Address 3 [63:0]              |
			 *    +-----------------------------------------------------+
			 */
			printk(KERN_INFO "R  [desc]      [buffer 0 63:0 ] "
				   "[buffer 1 63:0 ] "
				   "[buffer 2 63:0 ] [buffer 3 63:0 ] [bi->dma       ] "
				   "[bi->skb] <-- Ext Pkt Split format\n");
			/* [Extended] Receive Descriptor (Write-Back) Format
			 *
			 *   63       48 47    32 31     13 12    8 7    4 3        0
			 *   +------------------------------------------------------+
			 * 0 | Packet   | IP     |  Rsvd   | MRQ   | Rsvd | MRQ RSS |
			 *   | Checksum | Ident  |         | Queue |      |  Type   |
			 *   +------------------------------------------------------+
			 * 8 | VLAN Tag | Length | Extended Error | Extended Status |
			 *   +------------------------------------------------------+
			 *   63       48 47    32 31            20 19               0
			 */
			printk(KERN_INFO "RWB[desc]      [ck ipid mrqhsh] "
				   "[vl   l0 ee  es] "
				   "[ l3  l2  l1 hs] [reserved      ] ---------------- "
				   "[bi->skb] <-- Ext Rx Write-Back format\n");
			for (i = 0; i < rx_ring->count; i++) {
				buffer_info = &rx_ring->buffer_info[i];
				rx_desc_ps = E1000_RX_DESC_PS(*rx_ring, i);
				u1 = (struct my_u1 *)rx_desc_ps;
				staterr =
				le32_to_cpu(rx_desc_ps->wb.middle.status_error);
				if (staterr & E1000_RXD_STAT_DD) {
					/* Descriptor Done */
					printk(KERN_INFO "RWB[0x%03X]     %016llX "
						   "%016llX %016llX %016llX "
						   "---------------- %p", i,
						   le64_to_cpu(u1->a),
						   le64_to_cpu(u1->b),
						   le64_to_cpu(u1->c),
						   le64_to_cpu(u1->d),
						   buffer_info->skb);
				} else {
					printk(KERN_INFO "R  [0x%03X]     %016llX "
						   "%016llX %016llX %016llX %016llX %p", i,
						   le64_to_cpu(u1->a),
						   le64_to_cpu(u1->b),
						   le64_to_cpu(u1->c),
						   le64_to_cpu(u1->d),
						   (u64)buffer_info->dma,
						   buffer_info->skb);
					
					if (netif_msg_pktdata(adapter))
						print_hex_dump(KERN_INFO, "",
									   DUMP_PREFIX_ADDRESS, 16, 1,
									   phys_to_virt(buffer_info->dma),
									   adapter->rx_ps_bsize0, true);
				}
				
				if (i == rx_ring->next_to_use)
					printk(KERN_CONT " NTU\n");
				else if (i == rx_ring->next_to_clean)
					printk(KERN_CONT " NTC\n");
				else
					printk(KERN_CONT "\n");
			}
			break;
		default:
		case 0:
			/* Legacy Receive Descriptor Format
			 *
			 * +-----------------------------------------------------+
			 * |                Buffer Address [63:0]                |
			 * +-----------------------------------------------------+
			 * | VLAN Tag | Errors | Status 0 | Packet csum | Length |
			 * +-----------------------------------------------------+
			 * 63       48 47    40 39      32 31         16 15      0
			 */
			printk(KERN_INFO "Rl[desc]     [address 63:0  ] "
				   "[vl er S cks ln] [bi->dma       ] [bi->skb] "
				   "<-- Legacy format\n");
			for (i = 0; rx_ring->desc && (i < rx_ring->count); i++) {
				rx_desc = E1000_RX_DESC(*rx_ring, i);
				buffer_info = &rx_ring->buffer_info[i];
				u0 = (struct my_u0 *)rx_desc;
				printk(KERN_INFO "Rl[0x%03X]    %016llX %016llX "
					   "%016llX %p",
					   i, le64_to_cpu(u0->a), le64_to_cpu(u0->b),
					   (u64)buffer_info->dma, buffer_info->skb);
				if (i == rx_ring->next_to_use)
					printk(KERN_CONT " NTU\n");
				else if (i == rx_ring->next_to_clean)
					printk(KERN_CONT " NTC\n");
				else
					printk(KERN_CONT "\n");
				
				if (netif_msg_pktdata(adapter))
					print_hex_dump(KERN_INFO, "",
								   DUMP_PREFIX_ADDRESS,
								   16, 1, phys_to_virt(buffer_info->dma),
								   adapter->rx_buffer_len, true);
			}
	}
	
exit:
	return;
}


#endif

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


/*
 * This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */

#define E1000_MAX_NIC 32

#define OPTION_UNSET   -1
#define OPTION_DISABLED 0
#define OPTION_ENABLED  1

#define COPYBREAK_DEFAULT 256
unsigned int copybreak = COPYBREAK_DEFAULT;

/*
 * All parameters are treated the same, as an integer array of values.
 * This macro just reduces the need to repeat the same declaration code
 * over and over (plus this helps to avoid typo bugs).
 */

#define E1000_PARAM_INIT { [0 ... E1000_MAX_NIC] = OPTION_UNSET }
#define E1000_PARAM(X, desc) \


/*
 * Transmit Interrupt Delay in units of 1.024 microseconds
 * Tx interrupt delay needs to typically be set to something non zero
 *
 * Valid Range: 0-65535
 */
E1000_PARAM(TxIntDelay, "Transmit Interrupt Delay");
#define DEFAULT_TIDV 8
#define MAX_TXDELAY 0xFFFF
#define MIN_TXDELAY 0

/*
 * Transmit Absolute Interrupt Delay in units of 1.024 microseconds
 *
 * Valid Range: 0-65535
 */
E1000_PARAM(TxAbsIntDelay, "Transmit Absolute Interrupt Delay");
#define DEFAULT_TADV 32
#define MAX_TXABSDELAY 0xFFFF
#define MIN_TXABSDELAY 0

/*
 * Receive Interrupt Delay in units of 1.024 microseconds
 * hardware will likely hang if you set this to anything but zero.
 *
 * Valid Range: 0-65535
 */
E1000_PARAM(RxIntDelay, "Receive Interrupt Delay");
#define DEFAULT_RDTR 0
#define MAX_RXDELAY 0xFFFF
#define MIN_RXDELAY 0

/*
 * Receive Absolute Interrupt Delay in units of 1.024 microseconds
 *
 * Valid Range: 0-65535
 */
E1000_PARAM(RxAbsIntDelay, "Receive Absolute Interrupt Delay");
#define DEFAULT_RADV 8
#define MAX_RXABSDELAY 0xFFFF
#define MIN_RXABSDELAY 0

/*
 * Interrupt Throttle Rate (interrupts/sec)
 *
 * Valid Range: 100-100000 (0=off, 1=dynamic, 3=dynamic conservative)
 */
E1000_PARAM(InterruptThrottleRate, "Interrupt Throttling Rate");
#define DEFAULT_ITR 3
#define MAX_ITR 100000
#define MIN_ITR 100

#ifdef CONFIG_E1000E_MSIX
/* IntMode (Interrupt Mode)
 *
 * Valid Range: 0 - 2
 *
 * Default Value: 2 (MSI-X)
 */
E1000_PARAM(IntMode, "Interrupt Mode");
#define MAX_INTMODE	2
#define MIN_INTMODE	0

#endif /* CONFIG_E1000E_MSIX */
/*
 * Enable Smart Power Down of the PHY
 *
 * Valid Range: 0, 1
 *
 * Default Value: 0 (disabled)
 */
E1000_PARAM(SmartPowerDownEnable, "Enable PHY smart power down");

/*
 * Enable Kumeran Lock Loss workaround
 *
 * Valid Range: 0, 1
 *
 * Default Value: 1 (enabled)
 */
E1000_PARAM(KumeranLockLoss, "Enable Kumeran lock loss workaround");

/*
 * Enable CRC Stripping
 *
 * Valid Range: 0, 1
 *
 * Default Value: 1 (enabled)
 */
E1000_PARAM(CrcStripping, "Enable CRC Stripping, disable if your BMC needs " \
			"the CRC");

/*
 * Enable/disable EEE (a.k.a. IEEE802.3az)
 *
 * Valid Range: 0, 1
 *
 * Default Value: 1
 */
E1000_PARAM(EEE, "Enable/disable on parts that support the feature");

struct e1000_option {
	enum { enable_option, range_option, list_option } type;
	const char *name;
	const char *err;
	int def;
	union {
		struct { /* range_option info */
			int min;
			int max;
		} r;
		struct { /* list_option info */
			int nr;
			struct e1000_opt_list { int i; char *str; } *p;
		} l;
	} arg;
};

static int e1000_validate_option(unsigned int *value,
										   const struct e1000_option *opt,
										   struct e1000_adapter *adapter)
{
	if (*value == OPTION_UNSET) {
		*value = opt->def;
		return 0;
	}
	
	switch (opt->type) {
		case enable_option:
			switch (*value) {
				case OPTION_ENABLED:
					e_info("%s Enabled\n", opt->name);
					return 0;
				case OPTION_DISABLED:
					e_info("%s Disabled\n", opt->name);
					return 0;
			}
			break;
		case range_option:
			if (*value >= opt->arg.r.min && *value <= opt->arg.r.max) {
				e_info("%s set to %i\n", opt->name, *value);
				return 0;
			}
			break;
		case list_option: {
			int i;
			struct e1000_opt_list *ent;
			
			for (i = 0; i < opt->arg.l.nr; i++) {
				ent = &opt->arg.l.p[i];
				if (*value == ent->i) {
					if (ent->str[0] != '\0')
						e_info("%s\n", ent->str);
					return 0;
				}
			}
		}
			break;
		default:
			BUG();
	}
	
	e_info("Invalid %s value specified (%i) %s\n", opt->name, *value,
	       opt->err);
	*value = opt->def;
	return -1;
}

/**
 * e1000e_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 **/
void e1000e_check_options(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int bd = adapter->bd_number;
	
	if (bd >= E1000_MAX_NIC) {
		e_notice("Warning: no configuration for board #%i\n", bd);
		e_notice("Using defaults for all values\n");
	}
	
	{ /* Transmit Interrupt Delay */
		static const struct e1000_option opt = {
			.type = range_option,
			.name = "Transmit Interrupt Delay",
			.err  = "using default of "
			__MODULE_STRING(DEFAULT_TIDV),
			.def  = DEFAULT_TIDV,
			.arg  = { .r = { .min = MIN_TXDELAY,
				.max = MAX_TXDELAY } }
		};
		
		adapter->tx_int_delay = opt.def;
	}
	{ /* Transmit Absolute Interrupt Delay */
		static const struct e1000_option opt = {
			.type = range_option,
			.name = "Transmit Absolute Interrupt Delay",
			.err  = "using default of "
			__MODULE_STRING(DEFAULT_TADV),
			.def  = DEFAULT_TADV,
			.arg  = { .r = { .min = MIN_TXABSDELAY,
				.max = MAX_TXABSDELAY } }
		};
		
		adapter->tx_abs_int_delay = opt.def;
	}
	{ /* Receive Interrupt Delay */
		static struct e1000_option opt = {
			.type = range_option,
			.name = "Receive Interrupt Delay",
			.err  = "using default of "
			__MODULE_STRING(DEFAULT_RDTR),
			.def  = DEFAULT_RDTR,
			.arg  = { .r = { .min = MIN_RXDELAY,
				.max = MAX_RXDELAY } }
		};
		
		adapter->rx_int_delay = opt.def;
	}
	{ /* Receive Absolute Interrupt Delay */
		static const struct e1000_option opt = {
			.type = range_option,
			.name = "Receive Absolute Interrupt Delay",
			.err  = "using default of "
			__MODULE_STRING(DEFAULT_RADV),
			.def  = DEFAULT_RADV,
			.arg  = { .r = { .min = MIN_RXABSDELAY,
				.max = MAX_RXABSDELAY } }
		};
		
		adapter->rx_abs_int_delay = opt.def;
	}
	{ /* Interrupt Throttling Rate */
		static const struct e1000_option opt = {
			.type = range_option,
			.name = "Interrupt Throttling Rate (ints/sec)",
			.err  = "using default of "
			__MODULE_STRING(DEFAULT_ITR),
			.def  = DEFAULT_ITR,
			.arg  = { .r = { .min = MIN_ITR,
				.max = MAX_ITR } }
		};
		
		adapter->itr_setting = opt.def;
		adapter->itr = 20000;
	}
	{ /* CRC Stripping */
		adapter->flags2 |= FLAG2_CRC_STRIPPING;
	}
	{ /* Kumeran Lock Loss Workaround */
		static const struct e1000_option opt = {
			.type = enable_option,
			.name = "Kumeran Lock Loss Workaround",
			.err  = "defaulting to Enabled",
			.def  = OPTION_ENABLED
		};
		
		if (hw->mac.type == e1000_ich8lan)
			e1000e_set_kmrn_lock_loss_workaround_ich8lan(hw, opt.def);
	}
	{ /* EEE for parts supporting the feature */
		static const struct e1000_option opt = {
			.type = enable_option,
			.name = "EEE Support",
			.err  = "defaulting to Enabled",
			.def  = OPTION_ENABLED
		};
		
		if (adapter->flags2 & FLAG2_HAS_EEE) {
			/* Currently only supported on 82579 */
			hw->dev_spec.ich8lan.eee_disable = !opt.def;
		}
	}
}

// Mutex used in ich8lan.c
IOLock* swflag_mutex;
IOLock* nvm_mutex;

