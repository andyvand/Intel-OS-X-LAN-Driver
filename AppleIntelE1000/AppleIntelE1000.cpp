#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/assert.h>

extern "C" {
#include <sys/kpi_mbuf.h>
#include <net/ethernet.h>
}

extern "C" {
#include "e1000.h"
}

#include "AppleIntelE1000.h"


#define TBDS_PER_TCB 12
#define super IOEthernetController
#define COPYBREAK_DEFAULT 256

static inline void RELEASE(OSObject* x)
{
	if(x != NULL) {
		x->release();
		x = NULL;
	} 
}

static uint32_t jiffies()
{
#if defined(MAC_OS_X_VERSION_10_6)
	clock_sec_t seconds;
	clock_usec_t microsecs;
#else
	uint32_t seconds;
	uint32_t microsecs;
#endif
	clock_get_system_microtime(&seconds, &microsecs);
	return  seconds * 100 + microsecs / 10000; // 10 ms
}

/**
 * e1000_check_64k_bound - check that memory doesn't cross 64kB boundary
 * @adapter: address of board private structure
 * @start: address of beginning of memory
 * @len: length of memory
 **/
static bool e1000_check_64k_bound(struct e1000_adapter *adapter,
								  void *start, unsigned long len)
{
	unsigned long begin = (unsigned long) start;
	unsigned long end = begin + len;
	
	/* First rev 82545 and 82546 need to not allow any memory
	 * write location to cross 64k boundary due to errata 23 */
	if (adapter->hw.mac.type == e1000_82545 ||
	    adapter->hw.mac.type == e1000_82546) {
		return ((begin ^ (end - 1)) >> 16) != 0 ? false : true;
	}
	
	return true;
}

/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static void e1000_irq_disable(struct e1000_adapter *adapter)
{
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, ~0);
	E1000_WRITE_FLUSH(&adapter->hw);
	//synchronize_irq(adapter->pdev->irq);
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

static void e1000_irq_enable(struct e1000_adapter *adapter)
{
	E1000_WRITE_REG(&adapter->hw, E1000_IMS, IMS_ENABLE_MASK);
	E1000_WRITE_FLUSH(&adapter->hw);
}

#ifdef NETIF_F_HW_VLAN_TX

static void e1000_update_mng_vlan(struct e1000_adapter *adapter)
{
	u16 vid = adapter->hw.mng_cookie.vlan_id;
	u16 old_vid = adapter->mng_vlan_id;
	if (adapter->vlgrp) {
		if (!vlan_group_get_device(adapter->vlgrp, vid)) {
			if (adapter->hw.mng_cookie.status &
				E1000_MNG_DHCP_COOKIE_STATUS_VLAN) {
				e1000_vlan_rx_add_vid(netdev, vid);
				adapter->mng_vlan_id = vid;
			} else {
				adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
			}
			
			if ((old_vid != (u16)E1000_MNG_VLAN_NONE) &&
				(vid != old_vid) &&
			    !vlan_group_get_device(adapter->vlgrp, old_vid))
				e1000_vlan_rx_kill_vid(netdev, old_vid);
		} else {
			adapter->mng_vlan_id = vid;
		}
	}
}
#endif

/**
 * e1000_alloc_rings - Allocate memory for all rings
 * @adapter: board private structure to initialize
 **/
static int e1000_alloc_rings(struct e1000_adapter *adapter)
{
	adapter->tx_ring = (struct e1000_tx_ring *)IOMalloc(sizeof(struct e1000_tx_ring));
	if (!adapter->tx_ring)
		return -ENOMEM;
	bzero(adapter->tx_ring, sizeof(struct e1000_tx_ring));
	
	adapter->rx_ring = (struct e1000_rx_ring *)IOMalloc(sizeof(struct e1000_rx_ring));
	if (!adapter->rx_ring) {
		IOFree(adapter->tx_ring, sizeof(struct e1000_tx_ring));
		return -ENOMEM;
	}
	bzero(adapter->rx_ring, sizeof(struct e1000_rx_ring));

	return E1000_SUCCESS;
}

/**
 * e1000_sw_init - Initialize general software structures (struct e1000_adapter)
 * @adapter: board private structure to initialize
 *
 * e1000_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int e1000_sw_init(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	
	adapter->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;
	adapter->min_frame_size = ETH_ZLEN + ETH_FCS_LEN;
	
	hw->fc.requested_mode = e1000_fc_default;
	
	/* Initialize the hardware-specific values */
	if (e1000_setup_init_funcs(hw, false)) {
		DPRINTK(PROBE, ERR, "Hardware Initialization Failure\n");
		return -EIO;
	}
	
	if (e1000_alloc_rings(adapter)) {
		DPRINTK(PROBE, ERR, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}
	
	/* Explicitly disable IRQ since the NIC can be in any state. */
	e1000_irq_disable(adapter);
	
	//spin_lock_init(&adapter->stats_lock);
	
	//set_bit(__E1000_DOWN, &adapter->state);
	return 0;
}

static void e1000_init_manageability(struct e1000_adapter *adapter)
{
	if (adapter->en_mng_pt) {
		u32 manc = E1000_READ_REG(&adapter->hw, E1000_MANC);
		
		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);
		
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}



static void e1000_release_manageability(struct e1000_adapter *adapter)
{
	if (adapter->en_mng_pt) {
		u32 manc = E1000_READ_REG(&adapter->hw, E1000_MANC);
		
		/* re-enable hardware interception of ARP */
		manc |= E1000_MANC_ARP_EN;
		
		/* This is asymmetric with init_manageability, as we want to
		 * ensure that MNG2HOST filters are still enabled after this
		 * driver is unloaded as other host drivers such as PXE also
		 * may require these filters. */
		
		/* XXX stop the hardware watchdog ? */
		
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

void e1000_reset(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_fc_info *fc = &adapter->hw.fc;
	u32 pba = 0, tx_space, min_tx_space, min_rx_space;
	bool legacy_pba_adjust = false;
	u16 hwm;
	
	/* Repartition Pba for greater than 9k mtu
	 * To take effect CTRL.RST is required.
	 */
	
	switch (mac->type) {
		case e1000_82542:
		case e1000_82543:
		case e1000_82544:
		case e1000_82540:
		case e1000_82541:
		case e1000_82541_rev_2:
			legacy_pba_adjust = true;
			pba = E1000_PBA_48K;
			break;
		case e1000_82545:
		case e1000_82545_rev_3:
		case e1000_82546:
		case e1000_82546_rev_3:
			pba = E1000_PBA_48K;
			break;
		case e1000_82547:
		case e1000_82547_rev_2:
			legacy_pba_adjust = true;
			pba = E1000_PBA_30K;
			break;
		case e1000_undefined:
		case e1000_num_macs:
			break;
	}
	
	if (legacy_pba_adjust == true) {
		if (adapter->max_frame_size > E1000_RXBUFFER_8192)
			pba -= 8; /* allocate more FIFO for Tx */
		
		if (mac->type == e1000_82547) {
			adapter->tx_fifo_head = 0;
			adapter->tx_head_addr = pba << E1000_TX_HEAD_ADDR_SHIFT;
			adapter->tx_fifo_size =
			(E1000_PBA_40K - pba) << E1000_PBA_BYTES_SHIFT;
			atomic_set(&adapter->tx_fifo_stall, 0);
		}
	} else if (adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) {
		/* adjust PBA for jumbo frames */
		E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);
		
		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB. */
		pba = E1000_READ_REG(&adapter->hw, E1000_PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the tx fifo also stores 16 bytes of information about the tx
		 * but don't include ethernet FCS because hardware appends it */
		min_tx_space = (adapter->max_frame_size +
		                sizeof(struct e1000_tx_desc) -
		                ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;
		
		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation */
		if (tx_space < min_tx_space &&
		    ((min_tx_space - tx_space) < pba)) {
			pba = pba - (min_tx_space - tx_space);
			
			/* PCI/PCIx hardware has PBA alignment constraints */
			switch (mac->type) {
				case e1000_82545 ... e1000_82546_rev_3:
					pba &= ~(E1000_PBA_8K - 1);
					break;
				default:
					break;
			}
			
			/* if short on rx space, rx wins and must trump tx
			 * adjustment or use Early Receive if available */
			if (pba < min_rx_space) {
				pba = min_rx_space;
			}
		}
	}
	
	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);
	
	/* flow control settings */
	/* The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus the early receive size (for parts
	 *   with ERT support assuming ERT set to E1000_ERT_2048), or
	 * - the full Rx FIFO size minus one full frame */
	hwm = min(((pba << 10) * 9 / 10),
			  ((pba << 10) - adapter->max_frame_size));
	
	fc->high_water = hwm & 0xFFF8;	/* 8-byte granularity */
	fc->low_water = fc->high_water - 8;
	
	fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = 1;
	fc->current_mode = fc->requested_mode;
	
	/* Allow time for pending master requests to run */
	e1000_reset_hw(&adapter->hw);
	
	if (mac->type >= e1000_82544)
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, 0);
	
	if (e1000_init_hw(&adapter->hw))
		DPRINTK(PROBE, ERR, "Hardware Error\n");
#ifdef NETIF_F_HW_VLAN_TX
	e1000_update_mng_vlan(adapter);
#endif
	/* if (adapter->hwflags & HWFLAGS_PHY_PWR_BIT) { */
	if (mac->type >= e1000_82544 &&
	    mac->type <= e1000_82547_rev_2 &&
	    mac->autoneg == 1 &&
	    adapter->hw.phy.autoneg_advertised == ADVERTISE_1000_FULL) {
		u32 ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
		/* clear phy power management bit if we are in gig only mode,
		 * which if enabled will attempt negotiation to 100Mb, which
		 * can cause a loss of link at power off or driver unload */
		ctrl &= ~E1000_CTRL_SWDPIN3;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	}
	
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERNET_IEEE_VLAN_TYPE);
	
	e1000_reset_adaptive(&adapter->hw);
	e1000_get_phy_info(&adapter->hw);
	
	e1000_release_manageability(adapter);
}

/**
 * e1000_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in e1000_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * e1000_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int e1000_probe(struct e1000_adapter *adapter, UInt32* features)
{
	static int cards_found = 0;
	static int global_quad_port_a = 0; /* global ksp3 port a indication */
	int err;
	//int i
	int pci_using_dac = 0;
	u16 eeprom_data = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;

	
	/* setup the private structure */
	if ((err = e1000_sw_init(adapter)))
		goto err_sw_init;
	
	err = -EIO;
	if ((err = e1000_init_mac_params(&adapter->hw)))
		goto err_hw_init;
	
	if ((err = e1000_init_nvm_params(&adapter->hw)))
		goto err_hw_init;
	
	if ((err = e1000_init_phy_params(&adapter->hw)))
		goto err_hw_init;
	
	e1000_get_bus_info(&adapter->hw);
	
	e1000_init_script_state_82541(&adapter->hw, true);
	e1000_set_tbi_compatibility_82543(&adapter->hw, true);
	
	adapter->hw.phy.autoneg_wait_to_complete = false;
	adapter->hw.mac.adaptive_ifs = true;
	
	/* Copper options */
	
	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		adapter->hw.phy.mdix = AUTO_ALL_MODES;
		adapter->hw.phy.disable_polarity_correction = false;
		adapter->hw.phy.ms_type = E1000_MASTER_SLAVE;
	}
	
	if (e1000_check_reset_block(&adapter->hw))
		DEBUGOUT( "PHY reset is blocked due to SOL/IDER session.\n");
	
#ifdef MAX_SKB_FRAGS
	if (adapter->hw.mac.type >= e1000_82543) {
#ifdef NETIF_F_HW_VLAN_TX
		*features = NETIF_F_SG |
		NETIF_F_HW_CSUM |
		NETIF_F_HW_VLAN_TX |
		NETIF_F_HW_VLAN_RX |
		NETIF_F_HW_VLAN_FILTER;
#else
		*features = NETIF_F_SG | NETIF_F_HW_CSUM;
#endif
	}
	
#ifdef NETIF_F_TSO
	if ((adapter->hw.mac.type >= e1000_82544) &&
		(adapter->hw.mac.type != e1000_82547)) {
		adapter->flags |= E1000_FLAG_HAS_TSO;
		*features |= NETIF_F_TSO;
	}
	
#endif
	if (pci_using_dac)
		*features |= NETIF_F_HIGHDMA;
	
#endif
	
	/* Hardware features, flags and workarounds */
	if (adapter->hw.mac.type >= e1000_82540) {
		adapter->flags |= E1000_FLAG_HAS_SMBUS;
		adapter->flags |= E1000_FLAG_HAS_INTR_MODERATION;
	}
	
	if (adapter->hw.mac.type == e1000_82543)
		adapter->flags |= E1000_FLAG_BAD_TX_CARRIER_STATS_FD;
	
	adapter->en_mng_pt = e1000_enable_mng_pass_thru(&adapter->hw);
	
	/* before reading the NVM, reset the controller to
	 * put the device in a known good starting state */
	
	e1000_reset_hw(&adapter->hw);
	
	/* make sure we don't intercept ARP packets until we're up */
	e1000_release_manageability(adapter);
	
	/* make sure the NVM is good */
	
	if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
		IOLog( "The NVM Checksum Is Not Valid\n");
		err = -EIO;
		goto err_eeprom;
	}
	
	/* copy the MAC address out of the NVM */
	
	if (e1000_read_mac_addr(&adapter->hw)){
		IOLog("NVM Read Error\n");
	}

	e1000_check_options(adapter);
	
	/* Initial Wake on LAN setting
	 * If APM wake is enabled in the EEPROM,
	 * enable the ACPI Magic Packet filter
	 */
	
	switch (adapter->hw.mac.type) {
		case e1000_82542:
		case e1000_82543:
			break;
		case e1000_82544:
			e1000_read_nvm(&adapter->hw,
						   NVM_INIT_CONTROL2_REG, 1, &eeprom_data);
			eeprom_apme_mask = E1000_EEPROM_82544_APM;
			break;
		case e1000_82546:
		case e1000_82546_rev_3:
			if (adapter->hw.bus.func == 1) {
				e1000_read_nvm(&adapter->hw,
							   NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
				break;
			}
			/* Fall Through */
		default:
			e1000_read_nvm(&adapter->hw,
						   NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
			break;
	}
	if (eeprom_data & eeprom_apme_mask)
		adapter->eeprom_wol |= E1000_WUFC_MAG;
	
	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port */
	switch (adapter->hw.device_id) {
		case E1000_DEV_ID_82546GB_PCIE:
			adapter->eeprom_wol = 0;
			break;
		case E1000_DEV_ID_82546EB_FIBER:
		case E1000_DEV_ID_82546GB_FIBER:
			/* Wake events only supported on port A for dual fiber
			 * regardless of eeprom setting */
			if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
				E1000_STATUS_FUNC_1)
				adapter->eeprom_wol = 0;
			break;
		case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
			/* if quad port adapter, disable WoL on all but port A */
			if (global_quad_port_a != 0)
				adapter->eeprom_wol = 0;
			else
				adapter->flags |= E1000_FLAG_QUAD_PORT_A;
			/* Reset for multiple quad port adapters */
			if (++global_quad_port_a == 4)
				global_quad_port_a = 0;
			break;
	}
	
	/* initialize the wol settings based on the eeprom settings */
	adapter->wol = adapter->eeprom_wol;
	
	/* print bus type/speed/width info */
	{
		struct e1000_hw *hw = &adapter->hw;
		IOLog("(PCI%s:%s:%s) ",
				((hw->bus.type == e1000_bus_type_pcix) ? "-X" :
				 (hw->bus.type == e1000_bus_type_pci_express ? " Express":"")),
				((hw->bus.speed == e1000_bus_speed_2500) ? "2.5Gb/s" :
				 (hw->bus.speed == e1000_bus_speed_133) ? "133MHz" :
				 (hw->bus.speed == e1000_bus_speed_120) ? "120MHz" :
				 (hw->bus.speed == e1000_bus_speed_100) ? "100MHz" :
				 (hw->bus.speed == e1000_bus_speed_66) ? "66MHz" : "33MHz"),
				((hw->bus.width == e1000_bus_width_64) ? "64-bit" :
				 (hw->bus.width == e1000_bus_width_pcie_x4) ? "Width x4" :
				 (hw->bus.width == e1000_bus_width_pcie_x1) ? "Width x1" :
				 "32-bit"));
	}
	
	
	/* reset the hardware with the new settings */
	e1000_reset(adapter);

#if	0
	/* tell the stack to leave us alone until e1000_open() is called */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	
	strcpy(netdev->name, "eth%d");
	err = register_netdev(netdev);
	if (err)
		goto err_register;
	
#endif
	IOLog("Intel(R) PRO/1000 Network Connection\n");
	cards_found++;
	return 0;
	
err_register:
err_hw_init:
err_eeprom:
	if (!e1000_check_reset_block(&adapter->hw))
		e1000_phy_hw_reset(&adapter->hw);
#if	0
	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);
#endif
	IOFree(adapter->tx_ring, sizeof(*adapter->tx_ring));
	IOFree(adapter->rx_ring, sizeof(*adapter->rx_ring));
err_sw_init:
	//iounmap(adapter->hw.hw_addr);
err_ioremap:
	//free_netdev(netdev);
err_alloc_etherdev:
	//pci_release_regions(pdev);
err_pci_reg:
err_dma:
	//pci_disable_device(pdev);
	IOLog("e1000_probe() failed.\n");
	return err;
}

/**
 * e1000_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 * @tx_ring:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
static int e1000_setup_tx_resources(struct e1000_adapter *adapter,
                                    struct e1000_tx_ring *tx_ring)
{
	int size;
	
	size = sizeof(struct e1000_buffer) * tx_ring->count;
	tx_ring->buffer_info = (struct e1000_buffer*)IOMallocPageable(size, PAGE_SIZE);	// 4K sligned
	if (!tx_ring->buffer_info) {
		DPRINTK(PROBE, ERR,
				"Unable to allocate memory for the transmit descriptor ring (IOMallocPageable) %u\n", tx_ring->count);
		return -ENOMEM;
	}
	memset(tx_ring->buffer_info, 0, size);
	
	/* round up to nearest 4K */
	
	tx_ring->size = tx_ring->count * sizeof(struct e1000_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);
	
	{
		tx_ring->pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
							kIODirectionInOut | kIOMemoryPhysicallyContiguous, 
							tx_ring->size,
							0xFFFF0000UL);	// 32-bits, 64K-aligned
		if(tx_ring->pool){
			tx_ring->pool->prepare();
			tx_ring->desc = (void*)tx_ring->pool->getBytesNoCopy();
			tx_ring->dma = (void*)tx_ring->pool->getPhysicalAddress();
		}
	}
	if (!tx_ring->desc) {
	setup_tx_desc_die:
		IOFreePageable(tx_ring->buffer_info,size);
		DPRINTK(PROBE, ERR,
				"Unable to allocate memory for the transmit descriptor ring (IOBufferMemoryDescriptor) %u\n", tx_ring->size);
		return -ENOMEM;
	}
#if	0
	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, tx_ring->desc, tx_ring->size)) {
		void *olddesc = tx_ring->desc;
		dma_addr_t olddma = tx_ring->dma;
		DPRINTK(TX_ERR, ERR, "tx_ring align check failed: %u bytes "
				"at %p\n", tx_ring->size, tx_ring->desc);
		/* Try again, without freeing the previous */
		tx_ring->desc = pci_alloc_consistent(pdev, tx_ring->size,
		                                     &tx_ring->dma);
		/* Failed allocation, critical failure */
		if (!tx_ring->desc) {
			pci_free_consistent(pdev, tx_ring->size, olddesc,
			                    olddma);
			goto setup_tx_desc_die;
		}
		
		if (!e1000_check_64k_bound(adapter, tx_ring->desc,
		                           tx_ring->size)) {
			/* give up */
			pci_free_consistent(pdev, tx_ring->size, tx_ring->desc,
								tx_ring->dma);
			pci_free_consistent(pdev, tx_ring->size, olddesc,
			                    olddma);
			DPRINTK(PROBE, ERR,
					"Unable to allocate aligned memory "
					"for the transmit descriptor ring\n");
			vfree(tx_ring->buffer_info);
			return -ENOMEM;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, tx_ring->size, olddesc,
			                    olddma);
		}
	}
#endif
	memset(tx_ring->desc, 0, tx_ring->size);
	
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	
	return 0;
}

/**
 * e1000_setup_all_tx_resources - wrapper to allocate Tx resources
 * @adapter: board private structure
 *
 * this allocates tx resources, return 0 on success, negative
 * on failure
 **/
int e1000_setup_all_tx_resources(struct e1000_adapter *adapter)
{
	int err = 0;
	
	err = e1000_setup_tx_resources(adapter, adapter->tx_ring);
	if (err)
		DPRINTK(PROBE, ERR, "Allocation for Tx Queue failed\n");
	
	return err;
}

/**
 * e1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void e1000_configure_tx(struct e1000_adapter *adapter)
{
	u64 tdba;
	struct e1000_hw *hw = &adapter->hw;
	u32 tdlen, tctl, tipg;
	u32 ipgr1, ipgr2;
	
	/* Setup the HW Tx Head and Tail descriptor pointers */
	tdba = (IOPhysicalAddress)adapter->tx_ring->dma;
	tdlen = adapter->tx_ring->count * sizeof(struct e1000_tx_desc);
	E1000_WRITE_REG(hw, E1000_TDBAL(0), (tdba & 0x00000000ffffffffULL));
	E1000_WRITE_REG(hw, E1000_TDBAH(0), (tdba >> 32));
	E1000_WRITE_REG(hw, E1000_TDLEN(0), tdlen);
	E1000_WRITE_REG(hw, E1000_TDH(0), 0);
	E1000_WRITE_REG(hw, E1000_TDT(0), 0);
	adapter->tx_ring->tdh = E1000_REGISTER(hw, E1000_TDH(0));
	adapter->tx_ring->tdt = E1000_REGISTER(hw, E1000_TDT(0));
	
	
	/* Set the default values for the Tx Inter Packet Gap timer */
	if (adapter->hw.mac.type <= e1000_82547_rev_2 &&
	    (hw->phy.media_type == e1000_media_type_fiber ||
	     hw->phy.media_type == e1000_media_type_internal_serdes))
		tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
	else
		tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
	
	switch (hw->mac.type) {
		case e1000_82542:
			tipg = DEFAULT_82542_TIPG_IPGT;
			ipgr1 = DEFAULT_82542_TIPG_IPGR1;
			ipgr2 = DEFAULT_82542_TIPG_IPGR2;
			break;
		default:
			ipgr1 = DEFAULT_82543_TIPG_IPGR1;
			ipgr2 = DEFAULT_82543_TIPG_IPGR2;
			break;
	}
	tipg |= ipgr1 << E1000_TIPG_IPGR1_SHIFT;
	tipg |= ipgr2 << E1000_TIPG_IPGR2_SHIFT;
	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	
	/* Set the Tx Interrupt Delay register */
	
	E1000_WRITE_REG(hw, E1000_TIDV, adapter->tx_int_delay);
	if (adapter->flags & E1000_FLAG_HAS_INTR_MODERATION)
		E1000_WRITE_REG(hw, E1000_TADV, adapter->tx_abs_int_delay);
	
	/* Program the Transmit Control Register */
	
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
	(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
	
	e1000_config_collision_dist(hw);
	
	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter->txd_cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS;
	
	/* only set IDE if we are delaying interrupts using the timers */
	if (adapter->tx_int_delay)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;
	
	if (hw->mac.type < e1000_82543)
		adapter->txd_cmd |= E1000_TXD_CMD_RPS;
	else
		adapter->txd_cmd |= E1000_TXD_CMD_RS;
	
	/* Cache if we're 82544 running in PCI-X because we'll
	 * need this to apply a workaround later in the send path. */
	if (hw->mac.type == e1000_82544 &&
	    hw->bus.type == e1000_bus_type_pcix)
		adapter->pcix_82544 = 1;
	
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
	
}

/**
 * e1000_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: board private structure
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_setup_rx_resources(struct e1000_adapter *adapter,
                                    struct e1000_rx_ring *rx_ring)
{
	int size, desc_len;
	
	size = sizeof(struct e1000_rx_buffer) * rx_ring->count;
	rx_ring->buffer_info = (struct e1000_rx_buffer*)IOMallocPageable(size, PAGE_SIZE);
	if (!rx_ring->buffer_info) {
		DPRINTK(PROBE, ERR,
				"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(rx_ring->buffer_info, 0, size);
	
	desc_len = sizeof(struct e1000_rx_desc);
	
	/* Round up to nearest 4K */
	
	rx_ring->size = rx_ring->count * desc_len;
	rx_ring->size = ALIGN(rx_ring->size, 4096);
	
	rx_ring->pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,kIODirectionInOut | kIOMemoryPhysicallyContiguous, 
								rx_ring->size,
								0xFFFF0000UL);	// 32-bits, 64K-aligned
	if(rx_ring->pool){
		rx_ring->pool->prepare();
		rx_ring->desc = (void*)rx_ring->pool->getBytesNoCopy();
		rx_ring->dma = (void*)rx_ring->pool->getPhysicalAddress();
	}
	
	if (!rx_ring->desc) {
		IOLog("Unable to allocate memory for the receive descriptor ring\n");
setup_rx_desc_die:
		IOFreePageable(rx_ring->buffer_info, size);
		return -ENOMEM;
	}

#if	0
	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, rx_ring->desc, rx_ring->size)) {
		void *olddesc = rx_ring->desc;
		dma_addr_t olddma = rx_ring->dma;
		DPRINTK(RX_ERR, ERR, "rx_ring align check failed: %u bytes "
				"at %p\n", rx_ring->size, rx_ring->desc);
		/* Try again, without freeing the previous */
		rx_ring->desc = pci_alloc_consistent(pdev, rx_ring->size,
		                                     &rx_ring->dma);
		/* Failed allocation, critical failure */
		if (!rx_ring->desc) {
			pci_free_consistent(pdev, rx_ring->size, olddesc,
			                    olddma);
			DPRINTK(PROBE, ERR,
					"Unable to allocate memory "
					"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		}
		
		if (!e1000_check_64k_bound(adapter, rx_ring->desc,
		                           rx_ring->size)) {
			/* give up */
			pci_free_consistent(pdev, rx_ring->size, rx_ring->desc,
			                    rx_ring->dma);
			pci_free_consistent(pdev, rx_ring->size, olddesc,
			                    olddma);
			DPRINTK(PROBE, ERR,
					"Unable to allocate aligned memory "
					"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, rx_ring->size, olddesc,
			                    olddma);
		}
	}
#endif
	memset(rx_ring->desc, 0, rx_ring->size);
	
	/* set up ring defaults */
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	rx_ring->rx_skb_top = NULL;
	rx_ring->adapter = adapter;
	
	return 0;
}

/**
 * e1000_setup_all_rx_resources - wrapper to allocate Rx resources
 * @adapter: board private structure
 *
 * this allocates rx resources, return 0 on success, negative on failure
 **/
int e1000_setup_all_rx_resources(struct e1000_adapter *adapter)
{
	int err = 0;
	
	err = e1000_setup_rx_resources(adapter, adapter->rx_ring);
	if (err)
		DPRINTK(PROBE, ERR, "Allocation for Rx Queue failed\n");
	
	return err;
}

/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/

static void e1000_setup_rctl(struct e1000_adapter *adapter)
{
	u32 rctl;
	
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
	E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
	(adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
	
	/* disable the stripping of CRC because it breaks
	 * BMC firmware connected over SMBUS
	 if (adapter->hw.mac.type > e1000_82543)
	 rctl |= E1000_RCTL_SECRC;
	 */
	
	if (e1000_tbi_sbp_enabled_82543(&adapter->hw))
		rctl |= E1000_RCTL_SBP;
	else
		rctl &= ~E1000_RCTL_SBP;
	
	if(adapter->max_frame_size <= ETH_DATA_LEN + ETH_HLEN + ETH_FCS_LEN )
		rctl &= ~E1000_RCTL_LPE;
	else
		rctl |= E1000_RCTL_LPE;
	
	/* Setup buffer sizes */
	rctl &= ~E1000_RCTL_SZ_4096;
	rctl |= E1000_RCTL_BSEX;
	switch (adapter->rx_buffer_len) {
		case E1000_RXBUFFER_2048:
		default:
			rctl |= E1000_RCTL_SZ_2048;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case E1000_RXBUFFER_4096:
			rctl |= E1000_RCTL_SZ_4096;
			break;
		case E1000_RXBUFFER_8192:
			rctl |= E1000_RCTL_SZ_8192;
			break;
		case E1000_RXBUFFER_16384:
			rctl |= E1000_RCTL_SZ_16384;
			break;
	}
	
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
}


/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/
static void e1000_alloc_rx_buffers(struct e1000_adapter *adapter,
                                   struct e1000_rx_ring *rx_ring,
                                   int cleaned_count)
{
	AppleIntelE1000* netdev = (AppleIntelE1000*)adapter->netdev;
	struct e1000_rx_desc *rx_desc;
	struct e1000_rx_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i;
	unsigned int bufsz = adapter->rx_buffer_len + NET_IP_ALIGN;
	
	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];
	
	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {	// recycle
			mbuf_setlen(skb, 0);
		} else {
			skb = netdev->allocatePacket(bufsz);
			if (unlikely(!skb)) {
				/* Better luck next round */
				adapter->alloc_rx_buff_failed++;
				break;
			}
			
			/* Fix for errata 23, can't cross 64kB boundary */
			if (!e1000_check_64k_bound(adapter, mbuf_data(skb), bufsz)) {
				mbuf_t oldskb = skb;
				IOLog("skb align check failed: %u bytes \n", bufsz);
				/* Try again, without freeing the previous */
				skb = netdev->allocatePacket(bufsz);
				/* Failed allocation, critical failure */
				if (!skb) {
					netdev->freePacket(oldskb);
					adapter->alloc_rx_buff_failed++;
					break;
				}
				
				if (!e1000_check_64k_bound(adapter, mbuf_data(skb), bufsz)) {
					/* give up */
					netdev->freePacket(skb);
					netdev->freePacket(oldskb);
					adapter->alloc_rx_buff_failed++;
					break; /* while !buffer_info->skb */
				}
				
				/* Use new allocation */
				netdev->freePacket(oldskb);
			}
			/* Make buffer alignment 2 beyond a 16 byte boundary
			 * this will result in a 16 byte aligned IP header after
			 * the 14 byte MAC header is removed
			 */
			//skb_reserve(skb, NET_IP_ALIGN);
			
			buffer_info->skb = skb;
			buffer_info->dma = 0;	// invalidate
		}

		// get physical info
		if(buffer_info->dma == 0){	// if invalid
			buffer_info->dma = (void*)netdev->mapRxSingle(skb);
			
			/* Fix for errata 23, can't cross 64kB boundary */
			if (!e1000_check_64k_bound(adapter,
									   (void *)(unsigned long)buffer_info->dma,
									   adapter->rx_buffer_len)) {
				DPRINTK(RX_ERR, ERR,
						"dma align check failed: %u bytes at %p\n",
						adapter->rx_buffer_len,
						(void *)(unsigned long)buffer_info->dma);
				netdev->freePacket(skb);
				buffer_info->skb = NULL;
				//pci_unmap_single(pdev, buffer_info->dma, adapter->rx_buffer_len, PCI_DMA_FROMDEVICE);
				buffer_info->dma = 0;
				
				adapter->alloc_rx_buff_failed++;
				break; /* while !buffer_info->skb */
			}
		}
		rx_desc = E1000_RX_DESC(*rx_ring, i);
		rx_desc->buffer_addr = cpu_to_le64((UInt64)buffer_info->dma);
		
		if (unlikely(++i == rx_ring->count))
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}
	
	if (likely(rx_ring->next_to_use != i)) {
		rx_ring->next_to_use = i;
		if (unlikely(i-- == 0))
			i = (rx_ring->count - 1);
		
		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64). */
		//wmb();
		writel(i, adapter->hw.hw_addr + rx_ring->rdt);
	}
}


/**
 * e1000_clean_rx_irq - Send received data up the network stack; legacy
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter,
							   struct e1000_rx_ring *rx_ring)
{
	AppleIntelE1000* netdev = (AppleIntelE1000*)adapter->netdev;
	struct e1000_rx_desc *rx_desc, *next_rxd;
	struct e1000_rx_buffer *buffer_info, *next_buffer;
	u32 length;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = false;
	unsigned int total_rx_bytes=0, total_rx_packets=0;
	
	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];
	
	while (rx_desc->status & E1000_RXD_STAT_DD) {
		mbuf_t skb;
		u8 status;
		
		status = rx_desc->status;
		skb = buffer_info->skb;
		// buffer_info->skb = NULL; -> postpone
		
		prefetch(skb->data - NET_IP_ALIGN);
		
		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);
		
		next_buffer = &rx_ring->buffer_info[i];
		
		cleaned = true;
		cleaned_count++;
		// pci_unmap_single(pdev, buffer_info->dma, adapter->rx_buffer_len, PCI_DMA_FROMDEVICE);
		//buffer_info->dma = 0;
		
		length = le16_to_cpu(rx_desc->length);
		
		/* !EOP means multiple descriptors were used to store a single
		 * packet, if thats the case we need to toss it.  In fact, we
		 * to toss every packet with the EOP bit clear and the next
		 * frame that _does_ have the EOP bit set, as it is by
		 * definition only a frame fragment
		 */
		if (unlikely(!(status & E1000_RXD_STAT_EOP)))
			adapter->flags |= E1000_FLAG_IS_DISCARDING;
		
		if (adapter->flags & E1000_FLAG_IS_DISCARDING) {
			/* All receives must fit into a single buffer */
			E1000_DBG("AppleE1000: Receive packet consumed multiple buffers\n");
			/* recycle */
			//buffer_info->skb = skb;
			if (status & E1000_RXD_STAT_EOP)
				adapter->flags &= ~E1000_FLAG_IS_DISCARDING;
			goto next_desc;
		}
		
		if (unlikely(rx_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK)) {
			netdev->e1000_rx_err(1);
			u8 last_byte = *((u8*)mbuf_data(skb) + length - 1);
			if (TBI_ACCEPT(&adapter->hw, status,
						   rx_desc->errors, length, last_byte,
						   adapter->min_frame_size,
						   adapter->max_frame_size)) {
				spin_lock_irqsave(&adapter->stats_lock, irq_flags);
				e1000_tbi_adjust_stats_82543(&adapter->hw,
											 &adapter->stats,
											 length, (u8*)mbuf_data(skb),
											 adapter->max_frame_size);
				spin_unlock_irqrestore(&adapter->stats_lock, irq_flags);
				length--;
			} else {
				/* recycle */
				//buffer_info->skb = skb;
				goto next_desc;
			}
		}
		
		/* adjust length to remove Ethernet CRC, this must be
		 * done after the TBI_ACCEPT workaround above */
		length -= 4;
		
		/* probably a little skewed due to removing CRC */
		total_rx_bytes += length;
		total_rx_packets++;
		
		/* code added for copybreak, this should improve
		 * performance for small packets with large amounts
		 * of reassembly being done in the stack */
		if (length < COPYBREAK_DEFAULT) {
			mbuf_t skb_in = netdev->copyPacket(skb);
			if (skb_in) {
				skb = skb_in;
			} else {
				/* else just continue with the old one */
				buffer_info->skb = NULL;
			}
		} else {
			buffer_info->skb = NULL;
		}

		/* end copybreak code */
		//mbuf_adjustlen(skb, length); // done in inputPacket.
		
		/* Receive Checksum Offload */
		netdev->e1000_rx_checksum(skb, (u32)(status) | ((u32)(rx_desc->errors) << 24));
		
		//skb->protocol = eth_type_trans(skb, netdev);
		
		netdev->e1000_receive_skb(skb, length, status, rx_desc->special);
		
		//netdev->last_rx = jiffies;
		
next_desc:
		rx_desc->status = 0;
		
		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= E1000_RX_BUFFER_WRITE)) {
			adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);
			cleaned_count = 0;
		}
		
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	rx_ring->next_to_clean = i;
	
	cleaned_count = E1000_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);
	
	adapter->total_rx_packets += total_rx_packets;
	adapter->total_rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_packets += total_rx_packets;
	return cleaned;
}

/**
 * e1000_configure_rx - Configure 8254x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void e1000_configure_rx(struct e1000_adapter *adapter)
{
	u64 rdba;
	struct e1000_hw *hw = &adapter->hw;
	u32 rdlen, rctl, rxcsum;
	{
		rdlen = adapter->rx_ring->count *
		sizeof(struct e1000_rx_desc);
		adapter->clean_rx = e1000_clean_rx_irq;
		adapter->alloc_rx_buf = e1000_alloc_rx_buffers;
	}
	
	/* disable receives while setting up the descriptors */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* do not flush or delay here, causes some strange problem
	 * on SERDES connected SFP modules */
	
	/* set the Receive Delay Timer Register */
	E1000_WRITE_REG(hw, E1000_RDTR, adapter->rx_int_delay);
	
	if (adapter->flags & E1000_FLAG_HAS_INTR_MODERATION) {
		E1000_WRITE_REG(hw, E1000_RADV, adapter->rx_abs_int_delay);
		if (adapter->itr_setting != 0)
			E1000_WRITE_REG(hw, E1000_ITR,
							1000000000 / (adapter->itr * 256));
	}
	
	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring */
	rdba = (IOPhysicalAddress)adapter->rx_ring->dma;
	E1000_WRITE_REG(hw, E1000_RDBAL(0), (rdba & 0x00000000ffffffffULL));
	E1000_WRITE_REG(hw, E1000_RDBAH(0), (rdba >> 32));
	E1000_WRITE_REG(hw, E1000_RDLEN(0), rdlen);
	E1000_WRITE_REG(hw, E1000_RDH(0), 0);
	E1000_WRITE_REG(hw, E1000_RDT(0), 0);
	adapter->rx_ring->rdh = E1000_REGISTER(hw, E1000_RDH(0));
	adapter->rx_ring->rdt = E1000_REGISTER(hw, E1000_RDT(0));
	
	if (hw->mac.type >= e1000_82543) {
		/* Enable 82543 Receive Checksum Offload for TCP and UDP */
		rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
		if (adapter->rx_csum == true) {
			rxcsum |= E1000_RXCSUM_TUOFL;
		} else {
			rxcsum &= ~E1000_RXCSUM_TUOFL;
			/* don't need to clear IPPCSE as it defaults to 0 */
		}
		E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);
	}
	
	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
}

static void e1000_unmap_and_free_tx_resource(struct e1000_adapter *adapter,
                                             struct e1000_buffer *buffer_info)
{
	if (buffer_info->dma) {
#if	0
		if (buffer_info->mapped_as_page)
			pci_unmap_page(adapter->pdev,
						   buffer_info->dma,
						   buffer_info->length,
						   PCI_DMA_TODEVICE);
		else
			pci_unmap_single(adapter->pdev,
							 buffer_info->dma,
							 buffer_info->length,
							 PCI_DMA_TODEVICE);
#endif
		buffer_info->dma = 0;
	}
	if (buffer_info->skb) {
		AppleIntelE1000* netdev = (AppleIntelE1000*)adapter->netdev;
		netdev->freePacket(buffer_info->skb);
		buffer_info->skb = NULL;
	}
	/* buffer_info must be completely set up in the transmit path */
}


/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 * @tx_ring: ring to be cleaned
 **/
static void e1000_clean_tx_ring(struct e1000_adapter *adapter,
                                struct e1000_tx_ring *tx_ring)
{
	struct e1000_buffer *buffer_info;
	unsigned long size;
	unsigned int i;
	
	/* Free all the Tx ring sk_buffs */
	
	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_info[i];
		e1000_unmap_and_free_tx_resource(adapter, buffer_info);
	}
	
	size = sizeof(struct e1000_buffer) * tx_ring->count;
	memset(tx_ring->buffer_info, 0, size);
	
	/* Zero out the descriptor ring */
	
	memset(tx_ring->desc, 0, tx_ring->size);
	
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	tx_ring->last_tx_tso = 0;
	
	writel(0, adapter->hw.hw_addr + tx_ring->tdh);
	writel(0, adapter->hw.hw_addr + tx_ring->tdt);
}

/**
 * e1000_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void e1000_clean_all_tx_rings(struct e1000_adapter *adapter)
{
	e1000_clean_tx_ring(adapter, adapter->tx_ring);
}

/**
 * e1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
static void e1000_free_tx_resources(struct e1000_adapter *adapter,
                                    struct e1000_tx_ring *tx_ring)
{
	e1000_clean_tx_ring(adapter, tx_ring);
	
	IOFreePageable(tx_ring->buffer_info, sizeof(struct e1000_buffer) * tx_ring->count);
	tx_ring->buffer_info = NULL;
	
	tx_ring->pool->complete();
	tx_ring->pool->release();
	tx_ring->desc = NULL;
}

/**
 * e1000_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void e1000_free_all_tx_resources(struct e1000_adapter *adapter)
{
	e1000_free_tx_resources(adapter, adapter->tx_ring);
}

/**
 * e1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 * @rx_ring: ring to free buffers from
 **/
static void e1000_clean_rx_ring(struct e1000_adapter *adapter,
                                struct e1000_rx_ring *rx_ring)
{
	AppleIntelE1000* netdev = (AppleIntelE1000*)adapter->netdev;
	struct e1000_rx_buffer *buffer_info;
	unsigned long size;
	unsigned int i;
	
	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		if (buffer_info->dma &&
		    adapter->clean_rx == e1000_clean_rx_irq) {
#if	0
			pci_unmap_single(pdev, buffer_info->dma,
			                 adapter->rx_buffer_len,
			                 PCI_DMA_FROMDEVICE);
#endif
		}
		
		buffer_info->dma = 0;
		if (buffer_info->page) {
			//put_page(buffer_info->page);
			buffer_info->page = NULL;
		}
		if (buffer_info->skb) {
			netdev->freePacket(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}
	
	
	size = sizeof(struct e1000_rx_buffer) * rx_ring->count;
	memset(rx_ring->buffer_info, 0, size);
	
	/* Zero out the descriptor ring */
	
	memset(rx_ring->desc, 0, rx_ring->size);
	
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	adapter->flags &= ~E1000_FLAG_IS_DISCARDING;
	
	writel(0, adapter->hw.hw_addr + rx_ring->rdh);
	writel(0, adapter->hw.hw_addr + rx_ring->rdt);
}

/**
 * e1000_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void e1000_clean_all_rx_rings(struct e1000_adapter *adapter)
{
	e1000_clean_rx_ring(adapter, adapter->rx_ring);
}

/**
 * e1000_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
static void e1000_free_rx_resources(struct e1000_adapter *adapter,
                                    struct e1000_rx_ring *rx_ring)
{
	e1000_clean_rx_ring(adapter, rx_ring);
	
	IOFreePageable(rx_ring->buffer_info, sizeof(struct e1000_rx_buffer) * rx_ring->count);
	rx_ring->buffer_info = NULL;
	
	rx_ring->pool->complete();
	rx_ring->pool->release();
	rx_ring->desc = NULL;
}

/**
 * e1000_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
void e1000_free_all_rx_resources(struct e1000_adapter *adapter)
{
	e1000_free_rx_resources(adapter, adapter->rx_ring);
}

/**
 * e1000_configure - configure the hardware for RX and TX
 * @adapter: private board structure
 **/
static void e1000_configure(struct e1000_adapter *adapter)
{
	struct e1000_rx_ring *ring = adapter->rx_ring;
	AppleIntelE1000* netdev = (AppleIntelE1000*)adapter->netdev;
	
	netdev->e1000_set_multi();
#ifdef NETIF_F_HW_VLAN_TX
	e1000_restore_vlan(adapter);
#endif
	e1000_init_manageability(adapter);
	
	e1000_configure_tx(adapter);
	e1000_setup_rctl(adapter);
	e1000_configure_rx(adapter);
	/* call E1000_DESC_UNUSED which always leaves
	 * at least 1 descriptor unused to make sure
	 * next_to_use != next_to_clean */
	adapter->alloc_rx_buf(adapter, ring, E1000_DESC_UNUSED(ring));
	
	//adapter->tx_queue_len = netdev->tx_queue_len;
}

int e1000_up(struct e1000_adapter *adapter)
{
	/* hardware has been reset, we need to reload some things */
	e1000_configure(adapter);
	
	//clear_bit(__E1000_DOWN, &adapter->state);
	
	e1000_irq_enable(adapter);
	
	/* fire a link change interrupt to start the watchdog */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_LSC);
	return 0;
}

void e1000_down(struct e1000_adapter *adapter)
{
	u32 tctl, rctl;
	
	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer */
	//set_bit(__E1000_DOWN, &adapter->state);
	
	/* disable receives in the hardware */
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */
	
	//netif_tx_disable(netdev);
	
	/* disable transmits in the hardware */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);
	/* flush both disables and wait for them to finish */
	E1000_WRITE_FLUSH(&adapter->hw);
	msleep(10);
	
	e1000_irq_disable(adapter);
#if	0
	del_timer_sync(&adapter->tx_fifo_stall_timer);
	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_info_timer);
	
	//netdev->tx_queue_len = adapter->tx_queue_len;
	netif_carrier_off(netdev);
#endif
	adapter->link_speed = 0;
	adapter->link_duplex = 0;

	e1000_reset(adapter);
	e1000_clean_all_tx_rings(adapter);
	e1000_clean_all_rx_rings(adapter);
}

/**
 * e1000_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int e1000_open(struct e1000_adapter *adapter)
{
	int err;
	
	/* allocate transmit descriptors */
	err = e1000_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;
	
	/* allocate receive descriptors */
	err = e1000_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;
	
	if (adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_up_phy(&adapter->hw);
	
#ifdef NETIF_F_HW_VLAN_TX
	adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
	if ((adapter->hw.mng_cookie.status &
	     E1000_MNG_DHCP_COOKIE_STATUS_VLAN)) {
		e1000_update_mng_vlan(adapter);
	}
#endif
	
#if		0
	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.  */
	e1000_configure(adapter);
	
	err = e1000_request_irq(adapter);
	if (err)
		goto err_req_irq;
	/* From here on the code is the same as e1000_up() */
	//clear_bit(__E1000_DOWN, &adapter->state);
	
	e1000_irq_enable(adapter);
	
	/* fire a link status change interrupt to start the watchdog */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_LSC);
#endif
	
	return E1000_SUCCESS;
	
err_req_irq:
	/* Power down the PHY so no link is implied when interface is down *
	 * The PHY cannot be powered down if any of the following is true *
	 * (a) WoL is enabled
	 * (b) AMT is active
	 * (c) SoL/IDER session is active */
	if (!adapter->wol && adapter->hw.mac.type >= e1000_82540 &&
		adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_down_phy(&adapter->hw);
	e1000_free_all_rx_resources(adapter);
err_setup_rx:
	e1000_free_all_tx_resources(adapter);
err_setup_tx:
	e1000_reset(adapter);
	
	return err;
}



/* The 82542 2.0 (revision 2) needs to have the receive unit in reset
 * and memory write and invalidate disabled for certain operations
 */
static void e1000_enter_82542_rst(struct e1000_adapter *adapter)
{
	u32 rctl;
	
	if (adapter->hw.mac.type != e1000_82542)
		return;
	if (adapter->hw.revision_id != E1000_REVISION_2)
		return;
	
	e1000_pci_clear_mwi(&adapter->hw);
	
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	rctl |= E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);

#if		0
	struct net_device *netdev = adapter->netdev;
	if (netif_running(netdev))
		e1000_clean_all_rx_rings(adapter);
#endif
}

static void e1000_leave_82542_rst(struct e1000_adapter *adapter)
{
	u32 rctl;
	
	if (adapter->hw.mac.type != e1000_82542)
		return;
	if (adapter->hw.revision_id != E1000_REVISION_2)
		return;
	
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	rctl &= ~E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);
	
	if (adapter->hw.bus.pci_cmd_word & PCI_COMMAND_INVALIDATE)
		e1000_pci_set_mwi(&adapter->hw);
#if	0
	struct net_device *netdev = adapter->netdev;
	if (netif_running(netdev)) {
		/* No need to loop, because 82542 supports only 1 queue */
		struct e1000_rx_ring *ring = adapter->rx_ring;
		e1000_configure_rx(adapter);
		adapter->alloc_rx_buf(adapter, ring, E1000_DESC_UNUSED(ring));
	}
#endif
}

/**
 * e1000_smartspeed - Workaround for SmartSpeed on 82541 and 82547 controllers.
 * @adapter:
 **/
static void e1000_smartspeed(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_phy_info *phy = &adapter->hw.phy;
	u16 phy_status;
	u16 phy_ctrl;
	
	if ((phy->type != e1000_phy_igp) || !mac->autoneg ||
	    !(phy->autoneg_advertised & ADVERTISE_1000_FULL))
		return;
	
	if (adapter->smartspeed == 0) {
		/* If Master/Slave config fault is asserted twice,
		 * we assume back-to-back */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT)) return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT)) return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_ctrl);
		if (phy_ctrl & CR_1000T_MS_ENABLE) {
			phy_ctrl &= ~CR_1000T_MS_ENABLE;
			e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL,
								phy_ctrl);
			adapter->smartspeed++;
			if (!e1000_phy_setup_autoneg(&adapter->hw) &&
				!e1000_read_phy_reg(&adapter->hw, PHY_CONTROL,
									&phy_ctrl)) {
					phy_ctrl |= (MII_CR_AUTO_NEG_EN |
								 MII_CR_RESTART_AUTO_NEG);
					e1000_write_phy_reg(&adapter->hw, PHY_CONTROL,
										phy_ctrl);
				}
		}
		return;
	} else if (adapter->smartspeed == E1000_SMARTSPEED_DOWNSHIFT) {
		/* If still no link, perhaps using 2/3 pair cable */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_ctrl);
		phy_ctrl |= CR_1000T_MS_ENABLE;
		e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL, phy_ctrl);
		if (!e1000_phy_setup_autoneg(&adapter->hw) &&
			!e1000_read_phy_reg(&adapter->hw, PHY_CONTROL, &phy_ctrl)) {
			phy_ctrl |= (MII_CR_AUTO_NEG_EN |
						 MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(&adapter->hw, PHY_CONTROL, phy_ctrl);
		}
	}
	/* Restart process after E1000_SMARTSPEED_MAX iterations */
	if (adapter->smartspeed++ == E1000_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}

/**
 * e1000_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int e1000_close(struct e1000_adapter *adapter)
{
	
	//WARN_ON(test_bit(__E1000_RESETTING, &adapter->state));
	e1000_down(adapter);
	/* Power down the PHY so no link is implied when interface is down *
	 * The PHY cannot be powered down if any of the following is true *
	 * (a) WoL is enabled
	 * (b) AMT is active
	 * (c) SoL/IDER session is active */
	if (!adapter->wol && adapter->hw.mac.type >= e1000_82540 &&
		adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_down_phy(&adapter->hw);
	//e1000_free_irq(adapter);
	
	e1000_free_all_tx_resources(adapter);
	e1000_free_all_rx_resources(adapter);
	
#ifdef NETIF_F_HW_VLAN_TX
	/* kill manageability vlan ID if supported, but not if a vlan with
	 * the same ID is registered on the host OS (let 8021q kill it) */
	if ((adapter->hw.mng_cookie.status &
		 E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
		!(adapter->vlgrp &&
		  vlan_group_get_device(adapter->vlgrp, adapter->mng_vlan_id))) {
			e1000_vlan_rx_kill_vid(netdev, adapter->mng_vlan_id);
		}
#endif
	
	return 0;
}

/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 *
 * the return value indicates if transmit processing was completed
 **/
static bool e1000_clean_tx_irq(struct e1000_adapter *adapter,
                               struct e1000_tx_ring *tx_ring)
{
	//E1000_DBG("AppleIntelE1000::e1000_clean_tx_irq()\n");
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
	bool cleaned = false;
	bool retval = true;
	unsigned int total_tx_bytes=0, total_tx_packets=0;
	
	
	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);
	
	while (eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) {
		for (cleaned = false; !cleaned; ) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);
			
			if (cleaned) {
				struct sk_buff *skb = buffer_info->skb;
				//E1000_DBG("AppleIntelE1000::e1000_clean_tx_irq(%p), cleand\n", skb );
#ifdef NETIF_F_TSO
				unsigned int segs, bytecount;
				segs = skb_shinfo(skb)->gso_segs ?: 1;
				/* multiply data chunks by size of headers */
				bytecount = ((segs - 1) * skb_headlen(skb)) +
				skb->len;
				total_tx_packets += segs;
				total_tx_bytes += bytecount;
#else
				total_tx_packets++;
				total_tx_bytes += mbuf_len(skb);
#endif
			}
			e1000_unmap_and_free_tx_resource(adapter, buffer_info);
			tx_desc->upper.data = 0;
			E1000_TX_DESC_INC(tx_ring,i);
		}
		
		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
	}
	
	tx_ring->next_to_clean = i;
	
#define TX_WAKE_THRESHOLD 32
#if	0
	if (unlikely(cleaned && netif_carrier_ok(netdev) &&
				 E1000_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		//smp_mb();
		
		if (netif_queue_stopped(netdev) &&
		    !(test_bit(__E1000_DOWN, &adapter->state))) {
			netif_wake_queue(netdev);
			++adapter->restart_queue;
		}
	}
#endif
	if (adapter->detect_tx_hung) {
		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i */
		adapter->detect_tx_hung = false;
#if	0
		if (tx_ring->buffer_info[eop].dma &&
		    time_after(jiffies(), tx_ring->buffer_info[eop].time_stamp +
		               (adapter->tx_timeout_factor * HZ))
		    && !(E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		         E1000_STATUS_TXOFF)) {
				
				/* detected Tx unit hang */
				IOLog("Detected Tx Unit Hang\n");
				netdev->stopQueue();
			}
#endif
	}
	adapter->total_tx_bytes += total_tx_bytes;
	adapter->total_tx_packets += total_tx_packets;
	adapter->net_stats.tx_bytes += total_tx_bytes;
	adapter->net_stats.tx_packets += total_tx_packets;
	return retval;
}

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

/**
 * e1000_update_itr - update the dynamic ITR value based on statistics
 * @adapter: pointer to adapter
 * @itr_setting: current adapter->itr
 * @packets: the number of packets during this measurement interval
 * @bytes: the number of bytes during this measurement interval
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 *      this functionality is controlled by the InterruptThrottleRate module
 *      parameter (see e1000_param.c)
 **/
static unsigned int e1000_update_itr(struct e1000_adapter *adapter,
                                     u16 itr_setting, int packets,
                                     int bytes)
{
	unsigned int retval = itr_setting;
	
	if (unlikely(!(adapter->flags & E1000_FLAG_HAS_INTR_MODERATION)))
		goto update_itr_done;
	
	if (packets == 0)
		goto update_itr_done;
	
	switch (itr_setting) {
		case lowest_latency:
			/* handle TSO and jumbo frames */
			if (bytes/packets > 8000)
				retval = bulk_latency;
			else if ((packets < 5) && (bytes > 512)) {
				retval = low_latency;
			}
			break;
		case low_latency:  /* 50 usec aka 20000 ints/s */
			if (bytes > 10000) {
				/* this if handles the TSO accounting */
				if (bytes/packets > 8000) {
					retval = bulk_latency;
				} else if ((packets < 10) || ((bytes/packets) > 1200)) {
					retval = bulk_latency;
				} else if ((packets > 35)) {
					retval = lowest_latency;
				}
			} else if (bytes/packets > 2000) {
				retval = bulk_latency;
			} else if (packets <= 2 && bytes < 512) {
				retval = lowest_latency;
			}
			break;
		case bulk_latency: /* 250 usec aka 4000 ints/s */
			if (bytes > 25000) {
				if (packets > 35) {
					retval = low_latency;
				}
			} else if (bytes < 6000) {
				retval = low_latency;
			}
			break;
	}
	
update_itr_done:
	return retval;
}

static void e1000_set_itr(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 current_itr;
	u32 new_itr = adapter->itr;
	
	if (unlikely(!(adapter->flags & E1000_FLAG_HAS_INTR_MODERATION)))
		return;
	
	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (unlikely(adapter->link_speed != SPEED_1000)) {
		current_itr = 0;
		new_itr = 4000;
		goto set_itr_now;
	}
	
	adapter->tx_itr = e1000_update_itr(adapter,
									   adapter->tx_itr,
									   adapter->total_tx_packets,
									   adapter->total_tx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->tx_itr == lowest_latency)
		adapter->tx_itr = low_latency;
	
	adapter->rx_itr = e1000_update_itr(adapter,
									   adapter->rx_itr,
									   adapter->total_rx_packets,
									   adapter->total_rx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->rx_itr == lowest_latency)
		adapter->rx_itr = low_latency;
	
	current_itr = max(adapter->rx_itr, adapter->tx_itr);
	
	switch (current_itr) {
			/* counts and packets in update_itr are dependent on these numbers */
		case lowest_latency:
			new_itr = 70000;
			break;
		case low_latency:
			new_itr = 20000; /* aka hwitr = ~200 */
			break;
		case bulk_latency:
			new_itr = 4000;
			break;
		default:
			break;
	}
	
set_itr_now:
	if (new_itr != adapter->itr) {
		/* this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing */
		new_itr = new_itr > adapter->itr ?
		min(adapter->itr + (new_itr >> 2), new_itr) :
		new_itr;
		adapter->itr = new_itr;
		E1000_WRITE_REG(hw, E1000_ITR, 1000000000 / (new_itr * 256));
	}
	
	return;
}

#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16

static int e1000_tso(struct e1000_adapter *adapter,
                     struct e1000_tx_ring *tx_ring, struct sk_buff *skb)
{
#ifdef NETIF_F_TSO
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	u32 cmd_length = 0;
	u16 ipcse = 0, tucse, mss;
	u8 ipcss, ipcso, tucss, tucso, hdr_len;
	int err;
	
	if (skb_is_gso(skb)) {
		if (skb_header_cloned(skb)) {
			err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
			if (err)
				return err;
		}
		
		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		mss = skb_shinfo(skb)->gso_size;
		if (skb->protocol == htons(ETH_P_IP)) {
			struct iphdr *iph = ip_hdr(skb);
			iph->tot_len = 0;
			iph->check = 0;
			tcp_hdr(skb)->check = ~csum_tcpudp_magic(iph->saddr,
													 iph->daddr, 0,
													 IPPROTO_TCP,
													 0);
			cmd_length = E1000_TXD_CMD_IP;
			ipcse = skb_transport_offset(skb) - 1;
#ifdef NETIF_F_TSO6
		} else if (skb_shinfo(skb)->gso_type == SKB_GSO_TCPV6) {
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check =
			~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
							 &ipv6_hdr(skb)->daddr,
							 0, IPPROTO_TCP, 0);
			ipcse = 0;
#endif
		}
		ipcss = skb_network_offset(skb);
		ipcso = (void *)&(ip_hdr(skb)->check) - (void *)skb->data;
		tucss = skb_transport_offset(skb);
		tucso = (void *)&(tcp_hdr(skb)->check) - (void *)skb->data;
		tucse = 0;
		
		cmd_length |= (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE |
					   E1000_TXD_CMD_TCP | (skb->len - (hdr_len)));
		
		i = tx_ring->next_to_use;
		context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];
		
		context_desc->lower_setup.ip_fields.ipcss  = ipcss;
		context_desc->lower_setup.ip_fields.ipcso  = ipcso;
		context_desc->lower_setup.ip_fields.ipcse  = cpu_to_le16(ipcse);
		context_desc->upper_setup.tcp_fields.tucss = tucss;
		context_desc->upper_setup.tcp_fields.tucso = tucso;
		context_desc->upper_setup.tcp_fields.tucse = cpu_to_le16(tucse);
		context_desc->tcp_seg_setup.fields.mss     = cpu_to_le16(mss);
		context_desc->tcp_seg_setup.fields.hdr_len = hdr_len;
		context_desc->cmd_and_length = cpu_to_le32(cmd_length);
		
		buffer_info->time_stamp = jiffies;
		buffer_info->next_to_watch = i;
		
		E1000_TX_DESC_INC(tx_ring,i);
		tx_ring->next_to_use = i;
		
		return true;
	}
#endif
	
	return false;
}



static void e1000_tx_queue(struct e1000_adapter *adapter,
                           struct e1000_tx_ring *tx_ring,
                           int tx_flags, int count)
{
	//E1000_DBG("AppleIntelE1000::e1000_tx_queue(%d)\n", count);
	struct e1000_tx_desc *tx_desc = NULL;
	struct e1000_buffer *buffer_info;
	u32 txd_upper = 0, txd_lower = E1000_TXD_CMD_IFCS;
	unsigned int i;
	
	if (likely(tx_flags & E1000_TX_FLAGS_TSO)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D |
		E1000_TXD_CMD_TSE;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
		
		if (likely(tx_flags & E1000_TX_FLAGS_IPV4))
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
	}
	
	if (likely(tx_flags & E1000_TX_FLAGS_CSUM)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}
	
	if (unlikely(tx_flags & E1000_TX_FLAGS_VLAN)) {
		txd_lower |= E1000_TXD_CMD_VLE;
		txd_upper |= (tx_flags & E1000_TX_FLAGS_VLAN_MASK);
	}
	
	i = tx_ring->next_to_use;
	
	while (count--) {
		buffer_info = &tx_ring->buffer_info[i];
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		tx_desc->buffer_addr = cpu_to_le64((UInt64)buffer_info->dma);
		tx_desc->lower.data =
		cpu_to_le32(txd_lower | buffer_info->length);
		tx_desc->upper.data = cpu_to_le32(txd_upper);
		E1000_TX_DESC_INC(tx_ring,i);
	}
	
	tx_desc->lower.data |= cpu_to_le32(adapter->txd_cmd);
	
	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64). */
	//wmb();
	
	tx_ring->next_to_use = i;
	writel(i, adapter->hw.hw_addr + tx_ring->tdt);
	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems */
	//mmiowb();
}


#define E1000_FIFO_HDR			0x10
#define E1000_82547_PAD_LEN		0x3E0

/**
 * 82547 workaround to avoid controller hang in half-duplex environment.
 * The workaround is to avoid queuing a large packet that would span
 * the internal Tx FIFO ring boundary by notifying the stack to resend
 * the packet at a later time.  This gives the Tx FIFO an opportunity to
 * flush all packets.  When that occurs, we reset the Tx FIFO pointers
 * to the beginning of the Tx FIFO.
 **/
static int e1000_82547_fifo_workaround(struct e1000_adapter *adapter,
                                       struct sk_buff *skb)
{
	u32 fifo_space = adapter->tx_fifo_size - adapter->tx_fifo_head;
	u32 skb_fifo_len = mbuf_len(skb) + E1000_FIFO_HDR;
	
	skb_fifo_len = ALIGN(skb_fifo_len, E1000_FIFO_HDR);
	
	if (adapter->link_duplex != HALF_DUPLEX)
		goto no_fifo_stall_required;
	
	if (atomic_read(&adapter->tx_fifo_stall))
		return 1;
	
	if (skb_fifo_len >= (E1000_82547_PAD_LEN + fifo_space)) {
		atomic_set(&adapter->tx_fifo_stall, 1);
		return 1;
	}
	
no_fifo_stall_required:
	adapter->tx_fifo_head += skb_fifo_len;
	if (adapter->tx_fifo_head >= adapter->tx_fifo_size)
		adapter->tx_fifo_head -= adapter->tx_fifo_size;
	return 0;
}

bool e1000_has_link(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = false;
	s32 ret_val = 0;
	
	/* get_link_status is set on LSC (link status) interrupt or
	 * rx sequence error interrupt.  get_link_status will stay
	 * false until the e1000_check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
		case e1000_media_type_copper:
			if (hw->mac.get_link_status) {
				ret_val = e1000_check_for_link(hw);
				link_active = !hw->mac.get_link_status;
			} else {
				link_active = true;
			}
			break;
		case e1000_media_type_fiber:
			ret_val = e1000_check_for_link(hw);
			link_active = !!(E1000_READ_REG(hw, E1000_STATUS) &
							 E1000_STATUS_LU);
			break;
		case e1000_media_type_internal_serdes:
			ret_val = e1000_check_for_link(hw);
			link_active = adapter->hw.mac.serdes_has_link;
			break;
		default:
		case e1000_media_type_unknown:
			break;
	}
	
	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (E1000_READ_REG(&adapter->hw, E1000_CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		DPRINTK(LINK, INFO,
				"Gigabit has been disabled, downgrading speed\n");
	}
	
	return link_active;
}

static void set_hw_address(struct e1000_adapter *adapter)
{
	/* 82542 2.0 needs to be in reset to write receive address registers */

	if (adapter->hw.mac.type == e1000_82542)
		e1000_enter_82542_rst(adapter);

	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	if (adapter->hw.mac.type == e1000_82542)
		e1000_leave_82542_rst(adapter);
}

/////////////////
/////////////////
/////////////////

#pragma mark IOKit Framework
 
OSDefineMetaClassAndStructors(AppleIntelE1000, super);


void AppleIntelE1000::free()
{
	E1000_DBG("AppleIntelE1000::free()\n");
	
	RELEASE(netif);
	RELEASE(interruptSource);
	RELEASE(watchdogSource);
	
	RELEASE(workLoop);
	RELEASE(pciDevice);
	RELEASE(mediumDict);
	
	int i;
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		RELEASE(mediumTable[i]);
	}
	
	RELEASE(csrPCIAddress);
	RELEASE(flashPCIAddress);
	
	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	
	super::free();
}

bool AppleIntelE1000::init(OSDictionary *properties)
{
	E1000_DBG("bool AppleIntelE1000::Init(OSDictionary *properties).\n");
	
	if (super::init(properties) == false) 
		return false;
	int i;
	
	enabledForNetif = false;
	interruptEnabled = false;
	
	pciDevice = NULL;
	mediumDict = NULL;
	csrPCIAddress = NULL;
	flashPCIAddress = NULL;
	interruptSource = NULL;
	watchdogSource = NULL;
	netif = NULL;
	
	transmitQueue = NULL;
	preLinkStatus = 0;
	rxMbufCursor = NULL;
	txMbufCursor = NULL;
	
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		mediumTable[i] = NULL;
	}
	
	suspend = false;
	return true;
}

void AppleIntelE1000::stop(IOService* provider)
{
	E1000_DBG("AppleIntelE1000::stop(IOService * provider)\n");
	
	e1000_close(&adapter);

	if(workLoop){
		if (watchdogSource) {
			workLoop->removeEventSource(watchdogSource);
			watchdogSource->release();
			watchdogSource = NULL;
		}
		
		if (interruptSource) {
			workLoop->removeEventSource(interruptSource);
			interruptSource->release();
			interruptSource = NULL;
		}
	}
	
	// netif->terminate();
	// detachInterface((IONetworkInterface *)netif);
	
	super::stop(provider);
}

bool AppleIntelE1000::start(IOService* provider)
{
	E1000_DBG("AppleIntelE1000::start(IOService * provider)\n");
    bool success = false;
	
	if (super::start(provider) == false) {
		E1000_DBG("supper::start failed.\n");
		return false;
	}
	pciDevice = OSDynamicCast(IOPCIDevice, provider);
	if (pciDevice == NULL)
		return false;
	
	
	pciDevice->retain();
	if (pciDevice->open(this) == false)
		return false;
		
	adapter.hw.vendor_id = pciDevice->configRead16(kIOPCIConfigVendorID);
	adapter.hw.device_id = pciDevice->configRead16(kIOPCIConfigDeviceID);
	adapter.hw.subsystem_vendor_id = pciDevice->configRead16(kIOPCIConfigSubSystemVendorID);
	adapter.hw.subsystem_device_id = pciDevice->configRead16(kIOPCIConfigSubSystemID);
	adapter.hw.revision_id = pciDevice->configRead8(kIOPCIConfigRevisionID);
	adapter.hw.bus.pci_cmd_word = pciDevice->configRead16(kIOPCIConfigCommand);

	
	// adapter.hw.device_id will be used later
	IOLog("vendor:device: 0x%x:0x%x.\n", adapter.hw.vendor_id, adapter.hw.device_id);
	
	do {
		if(!initEventSources(provider)){
			break;
		}
#if	0
		/* allocate transmit descriptors */
		if (e1000_setup_ring_resources(adapter.tx_ring))
			break;
		/* allocate receive descriptors */
		if (e1000_setup_ring_resources(adapter.rx_ring))
			break;
#endif
		
		csrPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
		if (csrPCIAddress == NULL) {
			E1000_DBG("csrPCIAddress.\n");
			break;
		}
		flashPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
		if (flashPCIAddress == NULL) {
			E1000_DBG("flashPCIAddress.\n");
			break;
		}
		
		adapter.hw.hw_addr = (__uint8_t*)csrPCIAddress->getVirtualAddress();
		adapter.hw.flash_address = (__uint8_t*)flashPCIAddress->getVirtualAddress();

		adapter.netdev = this;
		adapter.pdev = pciDevice;
		adapter.hw.back = &adapter;
		
		// Init PCI config space:
        if ( false == initPCIConfigSpace( pciDevice ) )
            break;

		e1000_set_mtu(ETH_DATA_LEN);

		if(e1000_probe(&adapter, &features))
			break;

		set_hw_address(&adapter);
#if	0
		// Timer work.....
		init_timer(&adapter->tx_fifo_stall_timer);
		adapter->tx_fifo_stall_timer.function = &e1000_82547_tx_fifo_stall;
		adapter->tx_fifo_stall_timer.data = (unsigned long) adapter;
		
		init_timer(&adapter->watchdog_timer);
		adapter->watchdog_timer.function = &e1000_watchdog;
		adapter->watchdog_timer.data = (unsigned long) adapter;
		
		init_timer(&adapter->phy_info_timer);
		adapter->phy_info_timer.function = &e1000_update_phy_info;
		adapter->phy_info_timer.data = (unsigned long) adapter;
		
		INIT_WORK(&adapter->reset_task, e1000_reset_task);
		INIT_WORK(&adapter->watchdog_task, e1000_watchdog_task);
#endif
		/// end e1000_probe
		/// begin e1000_open
		if(e1000_open(&adapter))
			break;
		/// end e1000_open

		//Explicitly disable IRQ since the NIC can be in any state.
		E1000_WRITE_REG(&adapter.hw, E1000_IMC, ~0);
		E1000_READ_REG(&adapter.hw, E1000_STATUS);
		//sw_init end

		// reset the hardware with the new settings
		addNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);
		addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex,
						 10 * MBit, MEDIUM_INDEX_10HD);
		addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex,
						 10 * MBit, MEDIUM_INDEX_10FD);
		addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex,
						 100 * MBit, MEDIUM_INDEX_100HD);
		addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex,
						 100 * MBit, MEDIUM_INDEX_100FD);
		addNetworkMedium(kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex,
						 1000 * MBit, MEDIUM_INDEX_1000FD);
		
		if (!publishMediumDictionary(mediumDict)) {
			E1000_DBG("publishMediumDictionary.\n");
			break;
		}
		
		success = true;
	} while(false);
	
	// Close our provider, it will be re-opened on demand when
	// our enable() is called by a client.
	pciDevice->close(this);
	
	do {
		if ( false == success )
			break;
		
		// Allocate and attach an IOEthernetInterface instance.
		if (attachInterface((IONetworkInterface **)(&netif), true) == false) {
			E1000_DBG("Failed to attach data link layer.\n");
			break;
		}
		
		E1000_DBG("start success.\n");
		success = true;
	} while(false);
	
	return success;
}

//--------------------------------------------------------------------------
// Update PCI command register to enable the IO mapped PCI memory range,
// and bus-master interface.

bool AppleIntelE1000::initPCIConfigSpace( IOPCIDevice *pciDevice )
{
	UInt16	reg16;
	
	reg16 = pciDevice->configRead16( kIOPCIConfigCommand );
	
	reg16 |= ( kIOPCICommandBusMaster  |
			  kIOPCICommandMemorySpace |
			  kIOPCICommandMemWrInvalidate );
	reg16 &= ~kIOPCICommandIOSpace;  // disable I/O space
	pciDevice->configWrite16( kIOPCIConfigCommand, reg16 );
	
	return true;
}/* end initPCIConfigSpace */

//---------------------------------------------------------------------------
bool AppleIntelE1000::initEventSources( IOService* provider )
{
	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop* myWorkLoop = (IOWorkLoop *) getWorkLoop();
	if (myWorkLoop == NULL) {
		E1000_DBG(" myWorkLoop is NULL.\n");
		return false;
	}
	
	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL) {
		E1000_DBG("getOutputQueue failed.\n");
		return false;
	}
	transmitQueue->setCapacity(NUM_RING_FRAME);
	
	interruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
								this,
								&AppleIntelE1000::interruptHandler,
								&AppleIntelE1000::interruptFilter,
								provider);
	
	if (!interruptSource ||
		(myWorkLoop->addEventSource(interruptSource) != kIOReturnSuccess)) {
		E1000_DBG("workloop add eventsource interrupt source.\n");
		return false;
	}
	
	// This is important. If the interrupt line is shared with other devices,
	// then the interrupt vector will be enabled only if all corresponding
	// interrupt event sources are enabled. To avoid masking interrupts for
	// other devices that are sharing the interrupt line, the event source
	// is enabled immediately.
	interruptSource->enable();
	
	// Register a timer event source. This is used as a watchdog timer.
	//
	watchdogSource = IOTimerEventSource::timerEventSource(this, &AppleIntelE1000::timeoutHandler );
	if (!watchdogSource || (myWorkLoop->addEventSource(watchdogSource) != kIOReturnSuccess)) {
		E1000_DBG("watchdogSource create failed.\n");
		return false;
	}
	
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
	if (mediumDict == NULL) {
		E1000_DBG("mediumDict .\n");
		return false;
	}
	
	return true;
}

//---------------------------------------------------------------------------
IOReturn AppleIntelE1000::enable(IONetworkInterface * netif)
{
	E1000_DBG("AppleIntelE1000::enable().\n");
	if (enabledForNetif) {
		return kIOReturnSuccess;
	}
	
	//pciDevice->open(this);
	e1000_power_up_phy(&adapter.hw);

	
	if(suspend){
		suspend = false;
		// do something.
		e1000_reset(&adapter);
		E1000_WRITE_REG(&adapter.hw, E1000_WUS, ~0);
		
		e1000_init_manageability(&adapter);
	}
	
	e1000_up(&adapter);
	interruptEnabled = true;
	
	if (selectMedium(getSelectedMedium()) != kIOReturnSuccess)
		return kIOReturnIOError;
	
	watchdogSource->setTimeoutMS(1000);
	
	transmitQueue->start();
	
	workLoop->enableAllInterrupts();
	
	enabledForNetif = true;
	return kIOReturnSuccess; 
}

IOReturn AppleIntelE1000::disable(IONetworkInterface * netif)
{
	E1000_DBG("AppleIntelE1000::disable().\n");
	if(!enabledForNetif)
		return kIOReturnSuccess;
	enabledForNetif = false;
		
	workLoop->disableAllInterrupts();
	
	if (transmitQueue) {
		transmitQueue->stop();
		transmitQueue->flush();
	}
	
	//stop watchdog timer
	watchdogSource->cancelTimeout();		// Stop the timer event source.
	
	/* disable receives in the hardware */
	e1000_down(&adapter);
	e1000_power_down_phy(&adapter.hw);
	
	
	setLinkStatus( kIONetworkLinkValid );	// Valid sans kIONetworkLinkActive
	
	//pciDevice->close(this);
	
	return kIOReturnSuccess;
}


UInt32 AppleIntelE1000::outputPacket(mbuf_t skb, void * param)
{
	//E1000_DBG("AppleIntelE1000::outputPacket(%p)\n", skb );
	
	struct e1000_tx_ring *tx_ring = adapter.tx_ring;
	//unsigned int max_txd_pwr = adapter.tx_desc_pwr;
	struct e1000_buffer *buffer_info;

	IOPhysicalSegment tx_segments[TBDS_PER_TCB];
	int count = 0;
	unsigned int i, j, first;
	unsigned int tx_flags = 0;
	
	if (enabledForNetif == false) {             // drop the packet.
		//E1000_DBG("not enabledForNetif in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	
	if (mbuf_len(skb) <= 0) {
		//E1000_DBG("skb <=0 in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	/* On PCIX HW, there have been rare TXHangs caused by custom apps
	 * sending frames with skb->len == 16 (macdest+macsrc+protocol+2bytes).
	 * Such frames are not transmitted by registered protocols, so are
	 * only a problem for experimental code.
	 * Pad all 16 byte packets with an additional byte to work-around this
	 * problem case.
	 */
	if (unlikely(mbuf_len(skb) == 16)) {
		E1000_DBG("skb adjusted in outputPacket.\n");
		mbuf_adjustlen(skb,1);
		//skb_put(skb,1);
	}

	if (unlikely(adapter.hw.mac.type == e1000_82547)) {
		if (unlikely(e1000_82547_fifo_workaround(&adapter, skb))) {
			transmitQueue->stop();
			return kIOReturnOutputDropped;
		}
	}
	
	
#ifdef NETIF_F_HW_VLAN_TX
	if (unlikely(adapter->vlgrp && vlan_tx_tag_present(skb))) {
		tx_flags |= E1000_TX_FLAGS_VLAN;
		tx_flags |= (vlan_tx_tag_get(skb) << E1000_TX_FLAGS_VLAN_SHIFT);
	}
#endif
	
	first = tx_ring->next_to_use;
	
	int tso = e1000_tso(&adapter, tx_ring, skb);
	if (tso < 0) {
		freePacket(skb);
		return kIOReturnOutputDropped;
	}

	if (likely(tso)) {
		if (likely(adapter.hw.mac.type != e1000_82544))
			tx_ring->last_tx_tso = 1;
		tx_flags |= E1000_TX_FLAGS_TSO;
	} else if (likely(e1000_tx_csum(tx_ring, skb))){
		tx_flags |= E1000_TX_FLAGS_CSUM;
		tx_flags |= E1000_TX_FLAGS_IPV4;	//
	}
	struct ether_header* eh;
	eh = (struct ether_header*)mbuf_data(skb);
	if (likely(eh->ether_type == htons(ETHERTYPE_IP)))
		tx_flags |= E1000_TX_FLAGS_IPV4;
	
	count = txMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &tx_segments[0], TBDS_PER_TCB);
	if (count == 0) {
		E1000_DBG("failed to getphysicalsegment in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	netStats->outputPackets++;

	// e1000_tx_map
	j = adapter.tx_ring->next_to_use;
	
	for (i = 0; i < count; i++) {
		buffer_info = &tx_ring->buffer_info[j];
		
		buffer_info->next_to_watch = j;
		buffer_info->skb = NULL;
		buffer_info->time_stamp = jiffies();
		buffer_info->dma = (void*)tx_segments[i].location;
		buffer_info->length = tx_segments[i].length;
		//buffer_info->mapped_as_page = false;
		E1000_TX_DESC_INC(tx_ring,j);
	}	
	//OSWriteLittleInt32((UInt8*)adapter.hw.hw_addr + adapter.tx_ring->tail, 0, j);
	//OSSynchronizeIO();

	E1000_TX_DESC_DEC(tx_ring,j);
	tx_ring->buffer_info[j].skb = skb;
	tx_ring->buffer_info[first].next_to_watch = j;

	e1000_tx_queue(&adapter, tx_ring, tx_flags, count);
	
	/* Make sure there is space in the ring for the next send. */
	// e1000_maybe_stop_tx(netdev, MAX_SKB_FRAGS + 2);
	return kIOReturnOutputSuccess;
}

void AppleIntelE1000::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	E1000_DBG("AppleIntelE1000::getPacketBufferConstraints.\n");
	constraints->alignStart = kIOPacketBufferAlign2;
	constraints->alignLength = kIOPacketBufferAlign1;
	return;
}

IOOutputQueue * AppleIntelE1000::createOutputQueue()
{
	//E1000_DBG("AppleIntelE1000::createOutputQueue().\n");
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

const OSString * AppleIntelE1000::newVendorString() const
{
	E1000_DBG("const OSString * AppleIntelE1000::newVendorString() const.\n");
	return OSString::withCString("Intel");
}

const OSString * AppleIntelE1000::newModelString() const
{
	E1000_DBG("const OSString * AppleIntelE1000::newModelString() const.\n");
	return OSString::withCString(getName());
}

IOReturn AppleIntelE1000::selectMedium(const IONetworkMedium * medium)
{
	E1000_DBG("IOReturn AppleIntelE1000::selectMedium(const IONetworkMedium * medium).\n");
	bool link;
	
	link = e1000_has_link(&adapter);
	
	if (OSDynamicCast(IONetworkMedium, medium) == 0) {
		// Defaults to Auto.
		medium = mediumTable[MEDIUM_INDEX_AUTO];
	}
	
	
	if (link) {
		adapter.hw.mac.ops.get_link_up_info(&adapter.hw,
											&adapter.link_speed,
											&adapter.link_duplex);
		switch(adapter.link_speed) {
			case SPEED_1000:
				medium = mediumTable[MEDIUM_INDEX_1000FD];
				break;
			case SPEED_100:
				if (adapter.link_duplex == FULL_DUPLEX) {
					medium = mediumTable[MEDIUM_INDEX_100FD];
				} else {
					medium = mediumTable[MEDIUM_INDEX_100HD];
				}
				break;
			case SPEED_10:
				if (adapter.link_duplex == FULL_DUPLEX) {
					medium = mediumTable[MEDIUM_INDEX_10FD];
				} else {
					medium = mediumTable[MEDIUM_INDEX_10HD];
				}
				break;
			default:
				break;
		}
	}
	
	if (medium) {
		if (!setCurrentMedium(medium)) {
			E1000_DBG("setCurrentMedium error.\n");
		}
	}
	
	return ( medium ? kIOReturnSuccess : kIOReturnIOError );
}

bool AppleIntelE1000::configureInterface(IONetworkInterface * interface)
{
	E1000_DBG("AppleIntelE1000::configureInterface.\n");
	
	IONetworkData * data = NULL;
	
	if (super::configureInterface(interface) == false) {
		E1000_DBG("IOEthernetController::confiugureInterface failed.\n"); 
		return false;
	}
	
	// Get the generic network statistics structure.
	data = interface->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		E1000_DBG("netif getParameter NetworkStatsKey failed."); 
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = interface->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		E1000_DBG("netif getParameter kIOEthernetStatsKey failed."); 
		return false;
	}
	
	return true;
}

bool AppleIntelE1000::createWorkLoop()
{
	E1000_DBG("AppleIntelE1000::createWorkLoop().\n");
	workLoop = IOWorkLoop::workLoop();	
	return (workLoop !=  NULL);
}

IOWorkLoop * AppleIntelE1000::getWorkLoop() const
{
	E1000_DBG("AppleIntelE1000::getWorkLoop() const.\n");
	return workLoop;
}

//-----------------------------------------------------------------------
// Methods inherited from IOEthernetController.
//-----------------------------------------------------------------------

IOReturn AppleIntelE1000::getHardwareAddress(IOEthernetAddress * addr)
{
	E1000_DBG("getHardwareAddress %02x:%02x:%02x:%02x:%02x:%02x.\n",
		  adapter.hw.mac.addr[0],adapter.hw.mac.addr[1],adapter.hw.mac.addr[2],
		  adapter.hw.mac.addr[3],adapter.hw.mac.addr[4],adapter.hw.mac.addr[5] );
	memcpy(addr->bytes, adapter.hw.mac.addr, kIOEthernetAddressSize);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000::setHardwareAddress(const IOEthernetAddress * addr)
{
	E1000_DBG("setHardwareAddress %02x:%02x:%02x:%02x:%02x:%02x.\n",
		   addr->bytes[0],addr->bytes[1],addr->bytes[2],
		   addr->bytes[3],addr->bytes[4],addr->bytes[5] );
	memcpy(adapter.hw.mac.addr, addr->bytes, kIOEthernetAddressSize);

	set_hw_address(&adapter);

	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000::setPromiscuousMode(bool active)
{
	E1000_DBG("AppleIntelE1000::setPromiscuousMode(%d).\n", active);
	promiscusMode = active;
	e1000_set_multi();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000::setMulticastMode(bool active)
{
	E1000_DBG("AppleIntelE1000::setMulticastMode(%d).\n", active);
	multicastMode = active;
	e1000_set_multi();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	E1000_DBG("AppleIntelE1000::setMulticastList(%p, %u).\n", addrs, (unsigned int)count);
	e1000_update_mc_addr_list(&adapter.hw,(u8*)addrs,count);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily == kChecksumFamilyTCPIP ) {
		if( ! isOutput ) {
			*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			return kIOReturnSuccess;
		}
	}
	*checksumMask = 0;
	return kIOReturnUnsupported;
}

//-----------------------------------------------------------------------
// e1000e private functions
//-----------------------------------------------------------------------
bool AppleIntelE1000::addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) {
		E1000_DBG("Couldn't allocate medium.\n");
		return false;
	}
	
	if (!IONetworkMedium::addMedium(mediumDict, medium)) {
		E1000_DBG("Couldn't add medium.\n");
		return false;
	}
	
	mediumTable[index] = medium;
	medium->release();
	return true;
}

void AppleIntelE1000::interruptOccurred(IOInterruptEventSource * src)
{
	struct e1000_hw* hw = &adapter.hw;
	
	if (interruptReason & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = true;
	}
	/* Writing IMC and IMS is needed for 82547.
	 * Due to Hub Link bus being occupied, an interrupt
	 * de-assertion message is not able to be sent.
	 * When an interrupt assertion message is generated later,
	 * two messages are re-ordered and sent out.
	 * That causes APIC to think 82547 is in de-assertion
	 * state, while 82547 is in assertion state, resulting
	 * in dead lock. Writing IMC forces 82547 into
	 * de-assertion state.
	 */
	if (hw->mac.type == e1000_82547 || hw->mac.type == e1000_82547_rev_2)
		E1000_WRITE_REG(hw, E1000_IMC, ~0);
	
	adapter.total_tx_bytes = 0;
	adapter.total_rx_bytes = 0;
	adapter.total_tx_packets = 0;
	adapter.total_rx_packets = 0;

	if(adapter.clean_rx(&adapter, adapter.rx_ring)){
		int count = netif->flushInputQueue();
		netStats->inputPackets += count;
		//E1000_DBG("AppleIntelE1000::interruptOccurred,flushInputQueue = %d\n",count);
	}
	if(e1000_clean_tx_irq(&adapter, adapter.tx_ring)){
		transmitQueue->service();
	}

	if (likely(adapter.itr_setting & 3))
		e1000_set_itr(&adapter);

	if (hw->mac.type == e1000_82547 || hw->mac.type == e1000_82547_rev_2)
		e1000_irq_enable(&adapter);
}

void AppleIntelE1000::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	AppleIntelE1000 * me = (AppleIntelE1000 *) target;
	me->interruptOccurred(src);
}

bool AppleIntelE1000::interruptFilter(OSObject * target, IOFilterInterruptEventSource * src )
{
	AppleIntelE1000 * me = (AppleIntelE1000 *) target;
	struct e1000_hw* hw = &me->adapter.hw;
	u32 icr = E1000_READ_REG(hw, E1000_ICR);
	
	if (!me->interruptEnabled)
		return false;
	
	if (!icr)
		return false;  /* Not our interrupt */
	/*
	 * Interrupt Auto-Mask...upon reading ICR,
	 * interrupts are masked.  No need for the
	 * IMC write
	 */
	
	me->interruptReason = icr;
	return true;
}



void AppleIntelE1000::timeoutOccurred(IOTimerEventSource* src)
{
	watchdogSource->setTimeoutMS(3000);

	// update phy info at first
	e1000_get_phy_info(&adapter.hw);

	// then update link
	UInt32 link, tctl;
	link = e1000_has_link(&adapter);
	
	//E1000_DBG("timeout link:%d, preLinkStatus:%d.\n", link, preLinkStatus);
	if (link && link == preLinkStatus) {
		goto link_up;
	}
	
	if (link) {
		u32 ctrl;
		bool txb2b = 1;
#ifdef SIOCGMIIPHY
		/* update snapshot of PHY registers on LSC */
		e1000_phy_read_status(adapter);
#endif
		e1000_get_speed_and_duplex(&adapter.hw,
								   &adapter.link_speed,
								   &adapter.link_duplex);

		ctrl = E1000_READ_REG(&adapter.hw, E1000_CTRL);
		IOLog("NIC Link is Up %d Mbps %s, "
				"Flow Control: %s\n",
				adapter.link_speed,
				adapter.link_duplex == FULL_DUPLEX ?
				"Full Duplex" : "Half Duplex",
				((ctrl & E1000_CTRL_TFCE) && (ctrl &
				E1000_CTRL_RFCE)) ? "RX/TX" : ((ctrl &
				E1000_CTRL_RFCE) ? "RX" : ((ctrl & E1000_CTRL_TFCE) ? "TX" : "None" )));
		/* tweak tx_queue_len according to speed/duplex
		 * and adjust the timeout factor */
		//netdev->tx_queue_len = adapter->tx_queue_len;
		adapter.tx_timeout_factor = 1;
		switch (adapter.link_speed) {
			case SPEED_10:
				txb2b = 0;
				//netdev->tx_queue_len = 10;
				adapter.tx_timeout_factor = 16;
				break;
			case SPEED_100:
				txb2b = 0;
				//netdev->tx_queue_len = 100;
				/* maybe add some timeout factor ? */
				break;
		}

#ifdef NETIF_F_TSO
		/* disable TSO for pcie and 10/100 speeds, to avoid
		 * some hardware issues */
		if (!(adapter.flags & E1000_FLAG_TSO_FORCE) &&
			adapter.hw.bus.type == e1000_bus_type_pci_express){
			switch (adapter.link_speed) {
				case SPEED_10:
				case SPEED_100:
					IOLog("10/100 speed: disabling TSO\n");
					//netdev->features &= ~NETIF_F_TSO;
#ifdef NETIF_F_TSO6
					//netdev->features &= ~NETIF_F_TSO6;
#endif
					break;
				case SPEED_1000:
					//netdev->features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
					//netdev->features |= NETIF_F_TSO6;
#endif
					break;
				default:
					/* oops */
					break;
			}
		}
#endif
		/* enable transmits in the hardware, need to do this
		 * after setting TARC0 */
		tctl = E1000_READ_REG(&adapter.hw, E1000_TCTL);
		tctl |= E1000_TCTL_EN;
		E1000_WRITE_REG(&adapter.hw, E1000_TCTL, tctl);
		
		//netif_carrier_on(netdev);

		adapter.smartspeed = 0;
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), adapter.link_speed * MBit);
		if (transmitQueue) {
			transmitQueue->start();
		}
		
	} else {
		
		if (link != preLinkStatus) {
			adapter.link_speed = 0;
			adapter.link_duplex = 0;
			IOLog("Link is Down\n");
			
			transmitQueue->stop();
			setLinkStatus(kIONetworkLinkValid, 0);
			
		}
		e1000_smartspeed(&adapter);
	}
	
link_up:
	// e1000_update_stats(adapter);
	// e1000e_update_adaptive(&adapter->hw);
	
	/* Cause software interrupt to ensure Rx ring is cleaned */

	/* Force detection of hung controller every watchdog period */
	adapter.detect_tx_hung = true;
	preLinkStatus = link;


	// fifo stall
	if (atomic_read(&adapter.tx_fifo_stall)) {
		if ((E1000_READ_REG(&adapter.hw, E1000_TDT(0)) ==
			 E1000_READ_REG(&adapter.hw, E1000_TDH(0))) &&
			(E1000_READ_REG(&adapter.hw, E1000_TDFT) ==
			 E1000_READ_REG(&adapter.hw, E1000_TDFH)) &&
			(E1000_READ_REG(&adapter.hw, E1000_TDFTS) ==
			 E1000_READ_REG(&adapter.hw, E1000_TDFHS))) {
				tctl = E1000_READ_REG(&adapter.hw, E1000_TCTL);
				E1000_WRITE_REG(&adapter.hw, E1000_TCTL,
								tctl & ~E1000_TCTL_EN);
				E1000_WRITE_REG(&adapter.hw, E1000_TDFT,
								adapter.tx_head_addr);
				E1000_WRITE_REG(&adapter.hw, E1000_TDFH,
								adapter.tx_head_addr);
				E1000_WRITE_REG(&adapter.hw, E1000_TDFTS,
								adapter.tx_head_addr);
				E1000_WRITE_REG(&adapter.hw, E1000_TDFHS,
								adapter.tx_head_addr);
				E1000_WRITE_REG(&adapter.hw, E1000_TCTL, tctl);
				E1000_WRITE_FLUSH(&adapter.hw);
				
				adapter.tx_fifo_head = 0;
				atomic_set(&adapter.tx_fifo_stall, 0);
			}
	}
}

void AppleIntelE1000::timeoutHandler(OSObject * target, IOTimerEventSource * src)
{
	//E1000_DBG("void AppleIntelE1000::timeoutHandler(OSObject * target, IOTimerEventSource * src)\n");
	AppleIntelE1000* me = (AppleIntelE1000*) target;
	me->timeoutOccurred(src);
	
}



/**
 * e1000_set_multi - Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/
void AppleIntelE1000::e1000_set_multi()
{
	struct e1000_hw *hw = &adapter.hw;
	u32 rctl;
	
	/* Check for Promiscuous and All Multicast modes */
	
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	
	if (promiscusMode) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		
		/* disable VLAN filters */
		rctl &= ~E1000_RCTL_VFE;
	} else {
		if (multicastMode) {
			rctl |= E1000_RCTL_MPE;
			rctl &= ~E1000_RCTL_UPE;
		} else {
			rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
		}
#ifdef NETIF_F_HW_VLAN_TX
		/* enable VLAN filters if there's a VLAN */
		if (adapter.vlgrp)
			rctl |= E1000_RCTL_VFE;
#endif
	}
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	
	/* 82542 2.0 needs to be in reset to write receive address registers */
	
	if (hw->mac.type == e1000_82542)
		e1000_enter_82542_rst(&adapter);
}


bool AppleIntelE1000::e1000_tx_csum( struct e1000_tx_ring *tx_ring,  mbuf_t skb)
{
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	u8 css = ETHER_HDR_LEN;
	u8 offset = 0;
	u32 cmd_len = E1000_TXD_CMD_DEXT;


	UInt32 checksumDemanded;
	getChecksumDemand(skb, kChecksumFamilyTCPIP, &checksumDemanded);
	if((checksumDemanded & (kChecksumIP|kChecksumTCP|kChecksumUDP)) == 0){
		return false;
	}

	struct ether_header* eh;
	eh = (struct ether_header*)mbuf_data(skb);
	switch (htons(eh->ether_type)) {
	case ETHERTYPE_IP:
		{
			struct ip* ih = (struct ip*)(eh+1);
			if (ih->ip_p == IPPROTO_TCP){
				cmd_len |= E1000_TXD_CMD_TCP;
				offset = offsetof(struct tcphdr, th_sum);
			} else {
				offset = offsetof(struct udphdr, uh_sum);
			}
		}
		css += sizeof(struct ip);
		break;
	case ETHERTYPE_IPV6:
		/* XXX not handling all IPV6 headers */
		{
			struct ip6_hdr* ih = (struct ip6_hdr*)(eh+1);
			if (ih->ip6_nxt == IPPROTO_TCP){
				cmd_len |= E1000_TXD_CMD_TCP;
				offset = offsetof(struct tcphdr, th_sum);
			} else {
				offset = offsetof(struct udphdr, uh_sum);
			}
		}
		css += sizeof(struct ip6_hdr);
		break;
	default:
		break;
	}
	
	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
	
	context_desc->lower_setup.ip_config = 0;
	context_desc->upper_setup.tcp_fields.tucss = css;
	context_desc->upper_setup.tcp_fields.tucso = css + offset;

	context_desc->upper_setup.tcp_fields.tucse = 0;
	context_desc->tcp_seg_setup.data = 0;
	context_desc->cmd_and_length = cpu_to_le32(cmd_len);
	
	buffer_info->time_stamp = jiffies();
	buffer_info->next_to_watch = i;
	
	E1000_TX_DESC_INC(tx_ring,i);
	tx_ring->next_to_use = i;
	
	return true;
}


IOReturn AppleIntelE1000::registerWithPolicyMaker ( IOService * policyMaker )
{
	E1000_DBG("void AppleIntelE1000::registerWithPolicyMaker()\n");
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	powerState = 1;
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}

IOReturn AppleIntelE1000::setPowerState( unsigned long powerStateOrdinal,
										 IOService *policyMaker )
{
	E1000_DBG("AppleIntelE1000::setPowerState(%d)\n",(int)powerStateOrdinal);
	if (powerState == powerStateOrdinal)
		return IOPMAckImplied;
	powerState = powerStateOrdinal;
	
	if(powerState == 1){	//
		;
	} else {
		suspend = true;
	}
	/* acknowledge the completion of our power state change */
#if	0
	policyMaker->acknowledgeSetPowerState();	
#endif
	
    return IOPMAckImplied;
}

IOReturn AppleIntelE1000::getMaxPacketSize (UInt32 *maxSize) const {
	if (maxSize){
		if( adapter.hw.mac.type == e1000_82542 || 
		   adapter.hw.mac.type == e1000_undefined ) {
			*maxSize = ETH_DATA_LEN;
		} else {
			*maxSize = MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);
		}
	}
	E1000_DBG("AppleIntelE1000::getMaxPacketSize(%d)\n",(int)*maxSize);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000::getMinPacketSize (UInt32 *minSize) const {
	if(minSize)
		*minSize = ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN;
	
	return kIOReturnSuccess;
}

/**
 * e1000_set_mtu - Change the Maximum Transfer Unit
 **/

void AppleIntelE1000::e1000_set_mtu(UInt32 maxSize){
	UInt32 max_frame = maxSize + ETH_HLEN + ETH_FCS_LEN;
	
	adapter.max_frame_size = max_frame;
	IOLog("changing MTU from %d to %d\n", (int)mtu, (int)maxSize);
	mtu = maxSize;
	
	/*
	 * NOTE: netdev_alloc_skb reserves 16 bytes, and typically NET_IP_ALIGN
	 * means we reserve 2 more, this pushes us to allocate from the next
	 * larger slab size.
	 * i.e. RXBUFFER_2048 --> size-4096 slab
	 * However with the new *_jumbo_rx* routines, jumbo receives will use
	 * fragmented skbs
	 */
	
	if (max_frame <= 256)
		adapter.rx_buffer_len = 256;
	else if (max_frame <= 512)
		adapter.rx_buffer_len = 512;
	else if (max_frame <= 1024)
		adapter.rx_buffer_len = 1024;
	else if (max_frame <= 2048)
		adapter.rx_buffer_len = 2048;
	else if (max_frame <= 4096)
		adapter.rx_buffer_len = 4096;
	else if (max_frame <= 8192)
		adapter.rx_buffer_len = 8192;
	else if (max_frame <= 16384)
		adapter.rx_buffer_len = 16384;
	
	/* adjust allocation if LPE protects us, and we aren't using SBP */
	if ((max_frame == ETH_FRAME_LEN + ETH_FCS_LEN) ||
		(max_frame == ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN))
		adapter.rx_buffer_len = ETH_FRAME_LEN + VLAN_HLEN
		+ ETH_FCS_LEN;
	
	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, 1);
	txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, TBDS_PER_TCB);
}

IOReturn AppleIntelE1000::setMaxPacketSize (UInt32 maxSize){
	if(maxSize == mtu)
		return kIOReturnSuccess;
	
	//if (netif_running(netdev))
	//	e1000e_down(adapter);
	
	e1000_set_mtu(maxSize);
	
	e1000_reset(&adapter);
	
	return kIOReturnSuccess;
}

IOPhysicalAddress AppleIntelE1000::mapRxSingle(mbuf_t m)
{
	struct IOPhysicalSegment vector;
	UInt count;
	
	count = rxMbufCursor->getPhysicalSegments(m, &vector, 1);
	if (!count)
		return 0;
	return vector.location;
}

/**
 * e1000_rx_checksum - Receive Checksum Offload for 82543
 * @adapter:     board private structure
 * @status_err:  receive descriptor status and error fields
 * @csum:        receive descriptor csum field
 * @sk_buff:     socket buffer with received data
 **/
void AppleIntelE1000::e1000_rx_checksum(mbuf_t skb, u32 status_err)
{
	u16 status = (u16)status_err;
	u8 errors = (u8)(status_err >> 24);
	//skb->ip_summed = CHECKSUM_NONE;
	
	/* 82543 or newer only */
	if (unlikely(adapter.hw.mac.type < e1000_82543)) return;
	/* Ignore Checksum bit is set */
	if (unlikely(status & E1000_RXD_STAT_IXSM)) return;
	/* TCP/UDP checksum error bit is set */
	if (unlikely(errors & E1000_RXD_ERR_TCPE)) {
		/* let the stack verify checksum errors */
		adapter.hw_csum_err++;
		return;
	}
	/* TCP/UDP Checksum has not been calculated */
	if (adapter.hw.mac.type <= e1000_82547_rev_2) {
		if (!(status & E1000_RXD_STAT_TCPCS))
			return;
	} else {
		if (!(status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)))
			return;
	}
	/* It must be a TCP or UDP packet with a valid checksum */
	UInt32 ckResult = 0;
	if (status & E1000_RXD_STAT_IPCS)
		ckResult |= kChecksumIP;
	if (status & E1000_RXD_STAT_UDPCS)
		ckResult |= kChecksumUDP;
	if (status & E1000_RXD_STAT_TCPCS)
		ckResult |= kChecksumTCP;
	if(ckResult){
		setChecksumResult(skb, kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, ckResult);
	}
	adapter.hw_csum_good++;
}

/**
 * e1000_receive_skb - helper function to handle rx indications
 * @adapter: board private structure
 * @status: descriptor status field as written by hardware
 * @vlan: descriptor vlan field as written by hardware (no le/be conversion)
 * @skb: pointer to sk_buff to be indicated to stack
 **/
void AppleIntelE1000::e1000_receive_skb( mbuf_t skb, UInt32 len, u8 status,  u16 vlan )
{
#ifdef NETIF_F_HW_VLAN_TX
	if (unlikely(adapter->vlgrp && (status & E1000_RXD_STAT_VP))) {
		vlan_hwaccel_rx(skb, adapter->vlgrp,
		                le16_to_cpu(vlan) & E1000_RXD_SPC_VLAN_MASK);
	} else {
		netif_rx(skb);
	}
#else
	//netif->inputPacket(skb, mbuf_len(skb));
	netif->inputPacket(skb, len, IONetworkInterface::kInputOptionQueuePacket);
	//netStats->inputPackets++;
#endif
}

void AppleIntelE1000::stopQueue()
{
	IOLog("AppleIntelE1000::stopQueue()\n");
	transmitQueue->stop();
}

void AppleIntelE1000::e1000_rx_err( int count )
{
	netStats->inputErrors += count;
}

void AppleIntelE1000::e1000_tx_err( int count ){
	netStats->outputErrors += count;
}



void e1000_pci_set_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)hw->back;
	IOPCIDevice* pdev = (IOPCIDevice*)adapter->pdev;
	
	UInt16 reg16 = pdev->configRead16( kIOPCIConfigCommand );
	reg16 |= kIOPCICommandMemWrInvalidate; 
	pdev->configWrite16(kIOPCIConfigCommand, reg16);
	
}

void e1000_pci_clear_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)hw->back;
	IOPCIDevice* pdev = (IOPCIDevice*)adapter->pdev;
	
	UInt16 reg16 = pdev->configRead16( kIOPCIConfigCommand );
	reg16 &= ~kIOPCICommandMemWrInvalidate; 
	pdev->configWrite16(kIOPCIConfigCommand, reg16);
}

void e1000_read_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)hw->back;
	IOPCIDevice* pdev = (IOPCIDevice*)adapter->pdev;
	
	*value = pdev->configRead16(reg);
}

void e1000_write_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)hw->back;
	IOPCIDevice* pdev = (IOPCIDevice*)adapter->pdev;
	
	pdev->configWrite16(reg, *value);
}

s32 e1000_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = (struct e1000_adapter*)hw->back;
	IOPCIDevice* pdev = (IOPCIDevice*)adapter->pdev;
	u16 cap_offset;
	
	cap_offset = pdev->findPCICapability(kIOPCIPCIExpressCapability);
	if (!cap_offset)
		return -E1000_ERR_CONFIG;

	*value = pdev->configRead16(cap_offset + reg);
	
	return E1000_SUCCESS;
}

