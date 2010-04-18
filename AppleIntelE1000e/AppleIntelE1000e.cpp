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

#include "AppleIntelE1000e.h"


#define TBDS_PER_TCB 12
#define super IOEthernetController

static inline void RELEASE(OSObject* x)
{
	if(x != NULL) {
		x->release();
		x = NULL;
	} 
}

static void init_mutex(){
	nvm_mutex = IOLockAlloc();
	swflag_mutex = IOLockAlloc();
}

static void release_mutex(){
	if(nvm_mutex){
		IOLockFree(nvm_mutex);
		nvm_mutex = NULL;
	}
	
	if(swflag_mutex){
		IOLockFree(swflag_mutex);
		swflag_mutex = NULL;
	}
}

static inline void clear_desc(IOBufferMemoryDescriptor* desc, size_t size)
{
	bzero(desc->getBytesNoCopy(), size);
}

static void e1000_free_ring_resources(e1000_ring* ring)
{
	if (ring->desc) {
        ring->desc->complete();
        ring->desc->release();
		ring->desc = NULL;
	}
	bzero(ring, sizeof(e1000_ring));
}

/**
 * e1000_setup_ring_resources - allocate Tx/Rx resources (Descriptors)
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static int e1000_setup_ring_resources(struct e1000_ring* ring)
{
	IOBufferMemoryDescriptor* desc;
	IOPhysicalAddress dma;
	
	desc = IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
													   SIZE_RING_DESC, PAGE_SIZE );
	if (desc == NULL) {
		e_dbg("No memory for ring desc.\n");
		return -1;
	}
	dma = desc->getPhysicalAddress();
	
	clear_desc( desc, SIZE_RING_DESC);
	ring->dma = dma;
	ring->desc = desc;
	
	e_dbg("ring desc success: size->%d, virtual:%p, phy:%p.\n", SIZE_RING_DESC, desc->getBytesNoCopy(), (void*)dma);
	
	return 0;
}

/**
 * e1000_get_hw_control - get control of the h/w from f/w
 * @adapter: address of board private structure
 *
 * e1000_get_hw_control sets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded. For AMT version (only with 82573)
 * of the f/w this means that the network i/f is open.
 **/
static void e1000_get_hw_control(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_ext;
	u32 swsm;
	
	/* Let firmware know the driver has taken over */
	if (adapter->flags & FLAG_HAS_SWSM_ON_LOAD) {
		swsm = er32(SWSM);
		ew32(SWSM, swsm | E1000_SWSM_DRV_LOAD);
	} else if (adapter->flags & FLAG_HAS_CTRLEXT_ON_LOAD) {
		ctrl_ext = er32(CTRL_EXT);
		ew32(CTRL_EXT, ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	}
}

/**
 * e1000_release_hw_control - release control of the h/w to f/w
 * @adapter: address of board private structure
 *
 * e1000_release_hw_control resets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded. For AMT version (only with 82573) i
 * of the f/w this means that the network i/f is closed.
 *
 **/
static void e1000_release_hw_control(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_ext;
	u32 swsm;
	
	/* Let firmware taken over control of h/w */
	if (adapter->flags & FLAG_HAS_SWSM_ON_LOAD) {
		swsm = er32(SWSM);
		ew32(SWSM, swsm & ~E1000_SWSM_DRV_LOAD);
	} else if (adapter->flags & FLAG_HAS_CTRLEXT_ON_LOAD) {
		ctrl_ext = er32(CTRL_EXT);
		ew32(CTRL_EXT, ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
	}
}

/**
 * e1000e_power_up_phy - restore link in case the phy was powered down
 * @adapter: address of board private structure
 *
 * The phy may be powered down to save power and turn off link when the
 * driver is unloaded and wake on lan is not enabled (among others)
 * *** this routine MUST be followed by a call to e1000e_reset ***
 **/
void e1000e_power_up_phy(struct e1000_adapter *adapter)
{
	if (adapter->hw.phy.ops.power_up)
		adapter->hw.phy.ops.power_up(&adapter->hw);
	
	adapter->hw.mac.ops.setup_link(&adapter->hw);
}

/**
 * e1000_power_down_phy - Power down the PHY
 *
 * Power down the PHY so no link is implied when interface is down.
 * The PHY cannot be powered down if management or WoL is active.
 */
static void e1000_power_down_phy(struct e1000_adapter *adapter)
{
	/* WoL is enabled */
	if (adapter->wol)
		return;
	
	if (adapter->hw.phy.ops.power_down)
		adapter->hw.phy.ops.power_down(&adapter->hw);
}

/**
 *  e1000_update_mc_addr_list - Update Multicast addresses
 *  @hw: pointer to the HW structure
 *  @mc_addr_list: array of multicast addresses to program
 *  @mc_addr_count: number of multicast addresses to program
 *
 *  Updates the Multicast Table Array.
 *  The caller must have a packed mc_addr_list of multicast addresses.
 **/
static void e1000_update_mc_addr_list(struct e1000_hw *hw, u8 *mc_addr_list,
									  u32 mc_addr_count)
{
	hw->mac.ops.update_mc_addr_list(hw, mc_addr_list, mc_addr_count);
}

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
 *      while increasing bulk throughput.  This functionality is controlled
 *      by the InterruptThrottleRate module parameter.
 **/
static unsigned int e1000_update_itr(struct e1000_adapter *adapter,
									 u16 itr_setting, int packets,
									 int bytes)
{
	unsigned int retval = itr_setting;
	
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
	
	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (adapter->link_speed != SPEED_1000) {
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
		/*
		 * this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing
		 */
		new_itr = new_itr > adapter->itr ?
		min(adapter->itr + (new_itr >> 2), new_itr) :
		new_itr;
		adapter->itr = new_itr;
#ifdef CONFIG_E1000E_MSIX
		adapter->rx_ring->itr_val = new_itr;
		if (adapter->msix_entries)
			adapter->rx_ring->set_itr = 1;
		else
			ew32(ITR, 1000000000 / (new_itr * 256));
#else
		ew32(ITR, 1000000000 / (new_itr * 256));
#endif
	}
}

#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc->getBytesNoCopy()))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)

/////////////////


OSDefineMetaClassAndStructors(AppleIntelE1000e, super);


void AppleIntelE1000e::free()
{
	e_dbg("void AppleIntelE1000e::free()\n");
	
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
	
	// for ich8lan
	release_mutex();
	
	super::free();
}

bool AppleIntelE1000e::init(OSDictionary *properties)
{
	e_dbg("bool AppleIntelE1000e::Init(OSDictionary *properties).\n");
	
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
	
	// static allocate
	bzero(rx_buffer_info,sizeof(rx_buffer_info));
	bzero(tx_buffer_info,sizeof(tx_buffer_info));
	bzero(&rx_ring,sizeof(rx_ring));
	bzero(&tx_ring,sizeof(tx_ring));
	rx_ring.buffer_info = rx_buffer_info;
	tx_ring.buffer_info = tx_buffer_info;
	tx_ring.count = rx_ring.count = NUM_RING_FRAME;
	adapter.rx_ring = &rx_ring;
	adapter.tx_ring = &tx_ring;
	
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		mediumTable[i] = NULL;
	}

	// for ich8lan
	init_mutex();

	suspend = false;
	return true;
}

void AppleIntelE1000e::stop(IOService* provider)
{
	e_dbg("AppleIntelE1000e::stop(IOService * provider)\n");

	e1000_free_tx_resources();
	e1000_free_rx_resources();

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

bool AppleIntelE1000e::start(IOService* provider)
{
	e_dbg("AppleIntelE1000e::start(IOService * provider)\n");
    bool success = false;
	
	if (super::start(provider) == false) {
		e_dbg("supper::start failed.\n");
		return false;
	}
	pciDevice = OSDynamicCast(IOPCIDevice, provider);
	if (pciDevice == NULL)
		return false;
	

	pciDevice->retain();
	if (pciDevice->open(this) == false)
		return false;
	
	do {
		if(!initEventSources(provider)){
			break;
		}

		/* allocate transmit descriptors */
		if (e1000_setup_ring_resources(adapter.tx_ring))
			break;
		/* allocate receive descriptors */
		if (e1000_setup_ring_resources(adapter.rx_ring))
			break;
		
		// adapter.hw.device_id will be used later
		adapter.hw.device_id = pciDevice->configRead16(kIOPCIConfigDeviceID);
		IOLog("vendor:device: 0x%x:0x%x.\n", pciDevice->configRead16(kIOPCIConfigVendorID), adapter.hw.device_id);
		
		csrPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
		if (csrPCIAddress == NULL) {
			e_dbg("csrPCIAddress.\n");
			break;
		}
		flashPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
		if (flashPCIAddress == NULL) {
			e_dbg("flashPCIAddress.\n");
			break;
		}
		
		csrCPUAddress = csrPCIAddress->getVirtualAddress();
		flashCPUAddress = flashPCIAddress->getVirtualAddress();

		// Init PCI config space:
        if ( false == initPCIConfigSpace( pciDevice ) )
            break;
		

		adapter.ei = e1000_probe(adapter.hw.device_id);
		if (adapter.ei == NULL) {
			break;
		}
		
		adapter.pba = adapter.ei->pba;
		adapter.flags = adapter.ei->flags;
		adapter.flags2 = adapter.ei->flags2;
		adapter.hw.adapter = &adapter;
		adapter.hw.mac.type = adapter.ei->mac;
		adapter.max_hw_frame_size = adapter.ei->max_hw_frame_size;
		// adapter.msg_enable = (1 << NETIF_MSG_DRV | NETIF_MSG_PROBE) - 1;
		
		adapter.hw.hw_addr = (__uint8_t*)csrCPUAddress;
		adapter.hw.flash_address = (__uint8_t*)flashCPUAddress;
		
		e1000_set_mtu(ETH_DATA_LEN);	// this call allocates rxMbufCursor/txMbufCursor
		
		adapter.ei->init_ops(&adapter.hw);
		adapter.hw.mac.ops.init_params(&adapter.hw);
		adapter.hw.nvm.ops.init_params(&adapter.hw);
		adapter.hw.phy.ops.init_params(&adapter.hw);
		
		
		/* setup adapter struct */
		if (adapter.ei->get_variants) {
			if (adapter.ei->get_variants(&adapter))
				break;
		}
		
		//Explicitly disable IRQ since the NIC can be in any state.
		E1000_WRITE_REG(&adapter.hw, E1000_IMC, ~0);
		E1000_READ_REG(&adapter.hw, E1000_STATUS);
		//sw_init end
		
		adapter.hw.bus.width = e1000_bus_width_unknown; // dont care		
		
		adapter.hw.phy.autoneg_wait_to_complete = 0;	
		/* Copper options */
		if (adapter.hw.phy.media_type == e1000_media_type_copper) {
			adapter.hw.phy.mdix = AUTO_ALL_MODES;
			adapter.hw.phy.disable_polarity_correction = 0;
			adapter.hw.phy.ms_type = e1000_ms_hw_default;
		}
		
		if (adapter.hw.phy.ops.check_reset_block &&
			adapter.hw.phy.ops.check_reset_block(&adapter.hw))
			e_info("PHY reset is blocked due to SOL/IDER session.\n");
		
		adapter.hw.mac.ops.reset_hw(&adapter.hw);
		
		int i;
		for (i = 0; i < 2; i++) {
			if (adapter.hw.nvm.ops.validate(&adapter.hw) >= 0)
				break;
		}
		if (i == 2) {
			e_dbg("The NVM Checksum Is Not Valid\n");
			break;
		}

		/* copy the MAC address out of the NVM */
		if (e1000e_read_mac_addr(&adapter.hw))
			e_dbg("NVM Read Error while reading MAC address\n");
		
		// Initialize link parameters. User can change them with ethtool
		adapter.hw.mac.autoneg = 1;
		adapter.fc_autoneg = 1;
		adapter.hw.fc.requested_mode = e1000_fc_default;
		adapter.hw.fc.current_mode = e1000_fc_default;
		adapter.hw.phy.autoneg_advertised = 0x2f;
		
		// reset the hardware with the new settings
		e1000e_reset();
		
		
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
			e_dbg("publishMediumDictionary.\n");
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
			e_dbg("Failed to attach data link layer.\n");
			break;
		}
		
		e_dbg("start success.\n");
		success = true;
	} while(false);
	
	return success;
}

//--------------------------------------------------------------------------
// Update PCI command register to enable the IO mapped PCI memory range,
// and bus-master interface.

bool AppleIntelE1000e::initPCIConfigSpace( IOPCIDevice *pciDevice )
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
bool AppleIntelE1000e::initEventSources( IOService* provider )
{
	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop* myWorkLoop = (IOWorkLoop *) getWorkLoop();
	if (myWorkLoop == NULL) {
		e_dbg(" myWorkLoop is NULL.\n");
		return false;
	}

	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL) {
		e_dbg("getOutputQueue failed.\n");
		return false;
	}
	transmitQueue->setCapacity(NUM_RING_FRAME);
	
	interruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
							this,
							&AppleIntelE1000e::interruptHandler,
							&AppleIntelE1000e::interruptFilter,
							provider);
	
	if (!interruptSource ||
		(myWorkLoop->addEventSource(interruptSource) != kIOReturnSuccess)) {
		e_dbg("workloop add eventsource interrupt source.\n");
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
	watchdogSource = IOTimerEventSource::timerEventSource(this, &AppleIntelE1000e::timeoutHandler );
	if (!watchdogSource || (myWorkLoop->addEventSource(watchdogSource) != kIOReturnSuccess)) {
		e_dbg("watchdogSource create failed.\n");
		return false;
	}
	
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
	if (mediumDict == NULL) {
		e_dbg("mediumDict .\n");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
IOReturn AppleIntelE1000e::enable(IONetworkInterface * netif)
{
	e_dbg("AppleIntelE1000e::enable().\n");
	if (enabledForNetif) {
		e_info("Netif has been enabled already.\n");
		return kIOReturnSuccess;
	}
	
	//pciDevice->open(this);

	e1000e_power_up_phy(&adapter);

#if		1
	if(suspend){
		suspend = false;
		/* report the system wakeup cause from S3/S4 */
		if (adapter.flags2 & FLAG2_HAS_PHY_WAKEUP) {
			u16 phy_data;
			
			e1e_rphy(&adapter.hw, BM_WUS, &phy_data);
			if (phy_data) {
				e_info("PHY Wakeup cause - %s\n",
					   phy_data & E1000_WUS_EX ? "Unicast Packet" :
					   phy_data & E1000_WUS_MC ? "Multicast Packet" :
					   phy_data & E1000_WUS_BC ? "Broadcast Packet" :
					   phy_data & E1000_WUS_MAG ? "Magic Packet" :
					   phy_data & E1000_WUS_LNKC ? "Link Status "
					   " Change" : "other");
			}
			e1e_wphy(&adapter.hw, BM_WUS, ~0);
		} else {
			struct e1000_hw *hw = &adapter.hw;
			u32 wus = er32(WUS);
			if (wus) {
				e_info("MAC Wakeup cause - %s\n",
					   wus & E1000_WUS_EX ? "Unicast Packet" :
					   wus & E1000_WUS_MC ? "Multicast Packet" :
					   wus & E1000_WUS_BC ? "Broadcast Packet" :
					   wus & E1000_WUS_MAG ? "Magic Packet" :
					   wus & E1000_WUS_LNKC ? "Link Status Change" :
					   "other");
			}
			ew32(WUS, ~0);
		}
		
		e1000e_reset();

		e1000_init_manageability();
	}
#endif
	adapter.mng_vlan_id = E1000_MNG_VLAN_NONE;
	
	/*
	 * If AMT is enabled, let the firmware know that the network
	 * interface is now open
	 */
	if (adapter.flags & FLAG_HAS_AMT)
		e1000_get_hw_control(&adapter);

	/*
	 * before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.
	 */
	e1000_configure();
	
	// Create and register an interrupt event source. The provider will
	// take care of the low-level interrupt registration stuff.
	//
	// enable interrupt
	e1000_irq_enable();
	
	if (selectMedium(getSelectedMedium()) != kIOReturnSuccess)
		return kIOReturnIOError;

	watchdogSource->setTimeoutMS(1000);
	
	transmitQueue->start();
	
	adapter.hw.mac.get_link_status = true;

	/* fire a link status change interrupt to start the watchdog */
	E1000_WRITE_REG(&adapter.hw, E1000_ICS, E1000_ICS_LSC);

	workLoop->enableAllInterrupts();

	enabledForNetif = true;
	return kIOReturnSuccess; 

err_setup_rx:
	e1000_free_tx_resources();
err_setup_tx:
	e1000e_reset();
	
	return kIOReturnIOError;
}

IOReturn AppleIntelE1000e::disable(IONetworkInterface * netif)
{
	e_dbg("AppleIntelE1000e::disable().\n");
	if(!enabledForNetif)
		return kIOReturnSuccess;
	enabledForNetif = false;
	
	struct e1000_hw* hw = &adapter.hw;
	UInt32 rctl, tctl;

	/* disable irq */
	e1000_irq_disable();
	workLoop->disableAllInterrupts();
	
	if (transmitQueue) {
		transmitQueue->stop();
		transmitQueue->flush();
	}

	//stop watchdog timer
	watchdogSource->cancelTimeout();		// Stop the timer event source.

	/* disable receives in the hardware */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */
	
	/* disable transmits in the hardware */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);

	/* flush both disables and wait for them to finish */
	e1e_flush();
	E1000_READ_REG(hw, E1000_STATUS);
	msleep(10);

	adapter.link_speed = 0;
	adapter.link_duplex = 0;

	e1000e_reset();

	e1000_clean_tx_ring();
	e1000_clean_rx_ring();
	
	e1000_power_down_phy(&adapter);
	
	//e1000_free_irq(&adapter);
	
#ifdef NETIF_F_HW_VLAN_TX
	/*
	 * kill manageability vlan ID if supported, but not if a vlan with
	 * the same ID is registered on the host OS (let 8021q kill it)
	 */
	if ((adapter->hw.mng_cookie.status &
		 E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
		!(adapter->vlgrp &&
		  vlan_group_get_device(adapter->vlgrp, adapter->mng_vlan_id)))
		e1000_vlan_rx_kill_vid(netdev, adapter->mng_vlan_id);

#endif
	
	/*
	 * If AMT is enabled, let the firmware know that the network
	 * interface is now closed
	 */
	if (adapter.flags & FLAG_HAS_AMT)
		e1000_release_hw_control(&adapter);

	setLinkStatus( kIONetworkLinkValid );	// Valid sans kIONetworkLinkActive

	//pciDevice->close(this);

	return kIOReturnSuccess;
}


UInt32 AppleIntelE1000e::outputPacket(mbuf_t skb, void * param)
{
	// e_dbg("AppleIntelE1000e::outputPacket()\n");
	
	struct e1000_buffer* buffer_info = NULL;
	struct e1000_tx_desc* tx_desc = NULL;
	struct e1000_tx_desc* tx_desc_array = NULL;
	struct e1000_hw* hw = &adapter.hw;
	
	IOPhysicalSegment tx_segments[TBDS_PER_TCB];
	int count = 0;
	unsigned int i, j, first;
	UInt32 txd_upper = 0, txd_lower = 0;
	bool txCRC = false;
	
	if (enabledForNetif == false) {             // drop the packet.
		//e_dbg("not enabledForNetif in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	
	if (mbuf_len(skb) <= 0) {
		//e_dbg("skb <=0 in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	
	if (e1000_tx_csum(skb)) {
		txCRC = true;
	}
	txCRC = false;
	
	count = txMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &tx_segments[0], TBDS_PER_TCB);
	//count = txMbufCursor->getPhysicalSegments(skb, &tx_segments[0]);
	if (count == 0) {
		//e_dbg("failed to getphysicalsegment in outputPacket.\n");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	netStats->outputPackets++;
	
	if (0) {
		IOLog("enter TX count:%d, len:%04x, next_to_use:%d, next_to_clean:%d, head:%d, tail:%d.\n",
		      count, (unsigned int)mbuf_len(skb), adapter.tx_ring->next_to_use, adapter.tx_ring->next_to_clean, E1000_READ_REG(hw, adapter.tx_ring->head), E1000_READ_REG(hw, adapter.tx_ring->tail));
	}
	
	
	if (txCRC == true) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}
	
	tx_desc_array = (e1000_tx_desc*)(adapter.tx_ring->desc->getBytesNoCopy());
	j = first = adapter.tx_ring->next_to_use;
	
	for (i = 0; i < count; i++) {
		buffer_info = &adapter.tx_ring->buffer_info[j];
		tx_desc = &tx_desc_array[j];
		
		buffer_info->next_to_watch = j;
		buffer_info->skb = NULL;
		
		tx_desc->buffer_addr = tx_segments[i].location;
		tx_desc->lower.data = txd_lower | (tx_segments[i].length);
		tx_desc->upper.data = txd_upper;
		
		j++;
		if (j >= adapter.tx_ring->count)
			j = 0;
	}
	tx_desc->lower.data |= adapter.txd_cmd;
	
	adapter.tx_ring->next_to_use = j;
	OSWriteLittleInt32((UInt8*)adapter.hw.hw_addr + adapter.tx_ring->tail, 0, j);
	OSSynchronizeIO();
	
	if (j == 0)
		j = adapter.tx_ring->count - 1;
	else
		j--;
	
	adapter.tx_ring->buffer_info[j].skb = skb;
	adapter.tx_ring->buffer_info[first].next_to_watch = j;
	IODelay(10);
	
	if (0) {
		IOLog("in TX lower:%08x, upper:%08x.\n", tx_desc->lower.data, tx_desc->upper.data);
	}
	//writel(i, adapter.hw.hw_addr + adapter.tx_ring->tail);
	
	/* Make sure there is space in the ring for the next send. */
	// e1000_maybe_stop_tx(netdev, MAX_SKB_FRAGS + 2);
	return kIOReturnOutputSuccess;
}

void AppleIntelE1000e::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	e_dbg("AppleIntelE1000e::getPacketBufferConstraints.\n");
	constraints->alignStart = kIOPacketBufferAlign2;
	constraints->alignLength = kIOPacketBufferAlign1;
	return;
}

IOOutputQueue * AppleIntelE1000e::createOutputQueue()
{
	//e_dbg("AppleIntelE1000e::createOutputQueue().\n");
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

const OSString * AppleIntelE1000e::newVendorString() const
{
	e_dbg("const OSString * AppleIntelE1000e::newVendorString() const.\n");
	return OSString::withCString("Intel");
}

const OSString * AppleIntelE1000e::newModelString() const
{
	e_dbg("const OSString * AppleIntelE1000e::newModelString() const.\n");
	return OSString::withCString(getName());
}

IOReturn AppleIntelE1000e::selectMedium(const IONetworkMedium * medium)
{
	e_dbg("IOReturn AppleIntelE1000e::selectMedium(const IONetworkMedium * medium).\n");
	bool link;
	
	link = e1000e_setup_copper_link(&adapter.hw);
	
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
			e_dbg("setCurrentMedium error.\n");
		}
	}
	
	return ( medium ? kIOReturnSuccess : kIOReturnIOError );
}

bool AppleIntelE1000e::configureInterface(IONetworkInterface * interface)
{
	e_dbg("AppleIntelE1000e::configureInterface.\n");
	
	IONetworkData * data = NULL;
	
	if (super::configureInterface(interface) == false) {
		e_dbg("IOEthernetController::confiugureInterface failed.\n"); 
		return false;
	}
	
	// Get the generic network statistics structure.
	data = interface->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		e_dbg("netif getParameter NetworkStatsKey failed."); 
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = interface->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		e_dbg("netif getParameter kIOEthernetStatsKey failed."); 
		return false;
	}
	
	return true;
}

bool AppleIntelE1000e::createWorkLoop()
{
	e_dbg("AppleIntelE1000e::createWorkLoop().\n");
	workLoop = IOWorkLoop::workLoop();	
	return (workLoop !=  NULL);
}

IOWorkLoop * AppleIntelE1000e::getWorkLoop() const
{
	e_dbg("AppleIntelE1000e::getWorkLoop() const.\n");
	return workLoop;
}

//-----------------------------------------------------------------------
// Methods inherited from IOEthernetController.
//-----------------------------------------------------------------------

IOReturn AppleIntelE1000e::getHardwareAddress(IOEthernetAddress * addr)
{
	e_dbg("getHardwareAddress %02x:%02x:%02x:%02x:%02x:%02x.\n",
		   adapter.hw.mac.addr[0],adapter.hw.mac.addr[1],adapter.hw.mac.addr[2],
		   adapter.hw.mac.addr[3],adapter.hw.mac.addr[4],adapter.hw.mac.addr[5] );
	memcpy(addr->bytes, adapter.hw.mac.addr, kIOEthernetAddressSize);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::setHardwareAddress(const IOEthernetAddress * addr)
{
	e_info("setHardwareAddress %02x:%02x:%02x:%02x:%02x:%02x.\n",
		addr->bytes[0],addr->bytes[1],addr->bytes[2],
		addr->bytes[3],addr->bytes[4],addr->bytes[5] );
	memcpy(adapter.hw.mac.addr, addr->bytes, kIOEthernetAddressSize);
	
	e1000e_rar_set(&adapter.hw, adapter.hw.mac.addr, 0);
	
	if (adapter.flags & FLAG_RESET_OVERWRITES_LAA) {
		/* activate the work around */
		e1000e_set_laa_state_82571(&adapter.hw, 1);
		
		/*
		 * Hold a copy of the LAA in RAR[14] This is done so that
		 * between the time RAR[0] gets clobbered  and the time it
		 * gets fixed (in e1000_watchdog), the actual LAA is in one
		 * of the RARs and no incoming packets directed to this port
		 * are dropped. Eventually the LAA will be in RAR[0] and
		 * RAR[14]
		 */
		e1000e_rar_set(&adapter.hw,
					   adapter.hw.mac.addr,
					   adapter.hw.mac.rar_entry_count - 1);
	}
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setPromiscuousMode(bool active)
{
	e_dbg("AppleIntelE1000e::setPromiscuousMode(%d).\n", active);
	promiscusMode = active;
	e1000_set_multi();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setMulticastMode(bool active)
{
	e_dbg("AppleIntelE1000e::setMulticastMode(%d).\n", active);
	multicastMode = active;
	e1000_set_multi();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	e_dbg("AppleIntelE1000e::setMulticastList(%p, %u).\n", addrs, (unsigned int)count);
	e1000_update_mc_addr_list(&adapter.hw,(u8*)addrs,count);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily != kChecksumFamilyTCPIP ) {
		IOLog("AppleIntelE1000e: Operating system wants information for unknown checksum family.\n");
		return kIOReturnUnsupported;
	} else {
		if( !isOutput ) {
			*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
		} else {
			//*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			*checksumMask = 0;
		}
		return kIOReturnSuccess;
	}
}

//-----------------------------------------------------------------------
// e1000e private functions
//-----------------------------------------------------------------------
bool AppleIntelE1000e::addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) {
		e_dbg("Couldn't allocate medium.\n");
		return false;
	}
	
	if (!IONetworkMedium::addMedium(mediumDict, medium)) {
		e_dbg("Couldn't add medium.\n");
		return false;
	}
	
	mediumTable[index] = medium;
	medium->release();
	return true;
}

void AppleIntelE1000e::interruptOccurred(IOInterruptEventSource * src)
{
	int i;
	struct e1000_hw* hw = &adapter.hw;
	
	if (interruptReason & E1000_ICR_LSC) {
		hw->mac.get_link_status = true;
		/*
		 * ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers
		 */
		
		if ((adapter.flags & FLAG_LSC_GIG_SPEED_DROP) &&
			(!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)))
			e1000e_gig_downshift_workaround_ich8lan(hw);
	#if	0
		/*
		 * 80003ES2LAN workaround--
		 * For packet buffer work-around on link down event;
		 * disable receives here in the ISR and
		 * reset adapter in watchdog
		 */
		if (adapter.flags & FLAG_RX_NEEDS_RESTART) {
			/* disable receives */
			rctl = er32(RCTL);
			ew32(RCTL, rctl & ~E1000_RCTL_EN);
			adapter.flags |= FLAG_RX_RESTART_NOW;
		}
		/* guard against interrupt when we're going down */
	#endif
	}
	
	adapter.total_tx_bytes = 0;
	adapter.total_rx_bytes = 0;
	adapter.total_tx_packets = 0;
	adapter.total_rx_packets = 0;
	
	for (i = 0; i < E1000_MAX_INTR; i++) {
		// int rx_cleaned = adapter->clean_rx(adapter);
		int rx_cleaned = e1000_clean_rx_irq();
		int tx_cleaned_complete = e1000_clean_tx_irq();
		if (!rx_cleaned && tx_cleaned_complete)
			break;
	}
	
	if (adapter.itr_setting & 3)
		e1000_set_itr(&adapter);
}

void AppleIntelE1000e::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	AppleIntelE1000e * me = (AppleIntelE1000e *) target;
	me->interruptOccurred(src);
}

bool AppleIntelE1000e::interruptFilter(OSObject * target, IOFilterInterruptEventSource * src )
{
	AppleIntelE1000e * me = (AppleIntelE1000e *) target;
	struct e1000_hw* hw = &me->adapter.hw;
	u32 icr;

	if (!me->interruptEnabled)
		return false;
	
	icr = er32(ICR);

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



void AppleIntelE1000e::e1000_init_manageability()
{
	struct e1000_hw *hw = &adapter.hw;
	u32 manc, manc2h;
	
	if (!(adapter.flags & FLAG_MNG_PT_ENABLED))
		return;
	
	manc = er32(MANC);
	
	/*
	 * enable receiving management packets to the host. this will probably
	 * generate destination unreachable messages from the host OS, but
	 * the packets will be handled on SMBUS
	 */
	manc |= E1000_MANC_EN_MNG2HOST;
	manc2h = er32(MANC2H);
#define E1000_MNG2HOST_PORT_623 (1 << 5)
#define E1000_MNG2HOST_PORT_664 (1 << 6)
	manc2h |= E1000_MNG2HOST_PORT_623;
	manc2h |= E1000_MNG2HOST_PORT_664;
	ew32(MANC2H, manc2h);
	ew32(MANC, manc);
}

/**
 * e1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
void AppleIntelE1000e::e1000_configure_tx()
{
	struct e1000_hw *hw = &adapter.hw;
	struct e1000_ring *tx_ring = adapter.tx_ring;
	UInt32 tctl, tipg;
	UInt32 ipgr1, ipgr2;
	
	/* Setup the HW Tx Head and Tail descriptor pointers */
	
	u64 tdba = (u64)tx_ring->dma;
	u32 tdlen = tx_ring->count * sizeof(struct e1000_tx_desc);
	ew32(TDBAL(0), (tdba & DMA_BIT_MASK(32)));
	ew32(TDBAH(0), (tdba >> 32));
	ew32(TDLEN(0), tdlen);
	ew32(TDH(0), 0);
	ew32(TDT(0), 0);
	
	tx_ring->head = E1000_TDH(0);
	tx_ring->tail = E1000_TDT(0);
	
	/* Set the default values for the Tx Inter Packet Gap timer */
	tipg = DEFAULT_82543_TIPG_IPGT_COPPER;          /*  8  */
	ipgr1 = DEFAULT_82543_TIPG_IPGR1;               /*  8  */
	ipgr2 = DEFAULT_82543_TIPG_IPGR2;               /*  6  */
	
	tipg |= ipgr1 << E1000_TIPG_IPGR1_SHIFT;
	tipg |= ipgr2 << E1000_TIPG_IPGR2_SHIFT;
	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	
	/* Set the Tx Interrupt Delay register */
	//E1000_WRITE_REG(hw, E1000_TIDV, adapter.tx_int_delay);
	/* Tx irq moderation */
	//E1000_WRITE_REG(hw, E1000_TADV, adapter.tx_abs_int_delay);
	
	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
	(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
	
	hw->mac.ops.config_collision_dist(hw);
	
	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter.txd_cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS;
	
	/* only set IDE if we are delaying interrupts using the timers */
	//if (adapter.tx_int_delay)
	//adapter.txd_cmd |= E1000_TXD_CMD_IDE;
	
	/* enable Report Status bit */
	adapter.txd_cmd |= E1000_TXD_CMD_RS;
	
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
	
	//adapter.tx_queue_len = adapter.netdev->tx_queue_len;
}

/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
#define PAGE_USE_COUNT(S) (((S) >> PAGE_SHIFT) + \
(((S) & (PAGE_SIZE - 1)) ? 1 : 0))
void AppleIntelE1000e::e1000_setup_rctl()
{
	struct e1000_hw *hw = &adapter.hw;
	u32 rctl, rfctl;
	u32 psrctl = 0;
	u32 pages = 0;
	
	/* Program MC offset vector base */
	rctl = er32(RCTL);
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
	E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
	(adapter.hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
	
	/* Do not Store bad packets */
	rctl &= ~E1000_RCTL_SBP;
	
	/* Enable Long Packet receive */
	if (mtu <= ETH_DATA_LEN)
		rctl &= ~E1000_RCTL_LPE;
	else
		rctl |= E1000_RCTL_LPE;
	
	/* Some systems expect that the CRC is included in SMBUS traffic.  The
	 * hardware strips the CRC before sending to both SMBUS (BMC) and to
	 * host memory when this is enabled */
	if (adapter.flags2 & FLAG2_CRC_STRIPPING)
		rctl |= E1000_RCTL_SECRC;
	
	/* Workaround Si errata on 82577 PHY - configure IPG for jumbos */
	if ((hw->phy.type == e1000_phy_82577) && (rctl & E1000_RCTL_LPE)) {
		u16 phy_data;
		
		e1e_rphy(hw, PHY_REG(770, 26), &phy_data);
		phy_data &= 0xfff8;
		phy_data |= (1 << 2);
		e1e_wphy(hw, PHY_REG(770, 26), phy_data);
		
		e1e_rphy(hw, 22, &phy_data);
		phy_data &= 0x0fff;
		phy_data |= (1 << 14);
		e1e_wphy(hw, 0x10, 0x2823);
		e1e_wphy(hw, 0x11, 0x0003);
		e1e_wphy(hw, 22, phy_data);
	}
	
	/* Setup buffer sizes */
	rctl &= ~E1000_RCTL_SZ_4096;
	rctl |= E1000_RCTL_BSEX;
	switch (adapter.rx_buffer_len) {
		case 256:
			rctl |= E1000_RCTL_SZ_256;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case 512:
			rctl |= E1000_RCTL_SZ_512;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case 1024:
			rctl |= E1000_RCTL_SZ_1024;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case 2048:
		default:
			rctl |= E1000_RCTL_SZ_2048;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case 4096:
			rctl |= E1000_RCTL_SZ_4096;
			break;
		case 8192:
			rctl |= E1000_RCTL_SZ_8192;
			break;
		case 16384:
			rctl |= E1000_RCTL_SZ_16384;
			break;
	}
	
	/*
	 * 82571 and greater support packet-split where the protocol
	 * header is placed in skb->data and the packet data is
	 * placed in pages hanging off of skb_shinfo(skb)->nr_frags.
	 * In the case of a non-split, skb->data is linearly filled,
	 * followed by the page buffers.  Therefore, skb->data is
	 * sized to hold the largest protocol header.
	 *
	 * allocations using alloc_page take too long for regular MTU
	 * so only enable packet split for jumbo frames
	 *
	 * Using pages when the page size is greater than 16k wastes
	 * a lot of memory, since we allocate 3 pages at all times
	 * per packet.
	 */
#if	1
	// not supported (yet)
	pages = 0;
#else
	pages = PAGE_USE_COUNT(mtu);
#endif	
	if (!(adapter.flags & FLAG_IS_ICH) && (pages <= 3) &&
	    (PAGE_SIZE <= 16384) && (rctl & E1000_RCTL_LPE))
		adapter.rx_ps_pages = pages;
	else
		adapter.rx_ps_pages = 0;
	if (adapter.rx_ps_pages) {
		/* Configure extra packet-split registers */
		rfctl = er32(RFCTL);
		rfctl |= E1000_RFCTL_EXTEN;
		/*
		 * disable packet split support for IPv6 extension headers,
		 * because some malformed IPv6 headers can hang the Rx
		 */
		rfctl |= (E1000_RFCTL_IPV6_EX_DIS |
				  E1000_RFCTL_NEW_IPV6_EXT_DIS);
		
		ew32(RFCTL, rfctl);
		
		/* Enable Packet split descriptors */
		rctl |= E1000_RCTL_DTYP_PS;
		
		psrctl |= adapter.rx_ps_bsize0 >> E1000_PSRCTL_BSIZE0_SHIFT;
		
		switch (adapter.rx_ps_pages) {
			case 3:
				psrctl |= PAGE_SIZE << E1000_PSRCTL_BSIZE3_SHIFT;
			case 2:
				psrctl |= PAGE_SIZE << E1000_PSRCTL_BSIZE2_SHIFT;
			case 1:
				psrctl |= PAGE_SIZE >> E1000_PSRCTL_BSIZE1_SHIFT;
				break;
		}
		
		ew32(PSRCTL, psrctl);
	}
	
	ew32(RCTL, rctl);

	/* just started the receive unit, no need to restart */
	adapter.flags &= ~FLAG_RX_RESTART_NOW;
}

/**
 * e1000_configure_rx - Configure Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void AppleIntelE1000e::e1000_configure_rx()
{
	struct e1000_hw *hw = &adapter.hw;
	struct e1000_ring *rx_ring = adapter.rx_ring;
	u64 rdba;
	u32 rdlen, rctl, rxcsum, ctrl_ext;
	
	if (adapter.rx_ps_pages) {
		/* this is a 32 byte descriptor */
		e_dbg("32 byte descriptor.\n");
		rdlen = rx_ring->count *
		sizeof(union e1000_rx_desc_packet_split);
	} else {
		rdlen = rx_ring->count * sizeof(struct e1000_rx_desc);
	}
	
	/* disable receives while setting up the descriptors */
	rctl = er32(RCTL);
	ew32(RCTL, rctl & ~E1000_RCTL_EN);
	e1e_flush();
	msleep(10);
	
	/* set the Receive Delay Timer Register */
	ew32(RDTR, adapter.rx_int_delay);
	
	/* irq moderation */
	ew32(RADV, adapter.rx_abs_int_delay);
	if (adapter.itr_setting != 0)
		ew32(ITR, 1000000000 / (adapter.itr * 256));
	
	ctrl_ext = er32(CTRL_EXT);
	ew32(CTRL_EXT, ctrl_ext);
	e1e_flush();
	
	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	rdba = (u64)rx_ring->dma;
	ew32(RDBAL(0), (rdba & DMA_BIT_MASK(32)));
	ew32(RDBAH(0), (rdba >> 32));
	ew32(RDLEN(0), rdlen);
	ew32(RDH(0), 0);
	ew32(RDT(0), 0);
	rx_ring->head = E1000_RDH(0);
	rx_ring->tail = E1000_RDT(0);
	
	/* Enable Receive Checksum Offload for TCP and UDP */
	rxcsum = er32(RXCSUM);
	if (adapter.flags & FLAG_RX_CSUM_ENABLED) {
		rxcsum |= E1000_RXCSUM_TUOFL;
		
		/*
		 * IPv4 payload checksum for UDP fragments must be
		 * used in conjunction with packet-split.
		 */
		if (adapter.rx_ps_pages)
			rxcsum |= E1000_RXCSUM_IPPCSE;
	} else {
		rxcsum &= ~E1000_RXCSUM_TUOFL;
		/* no need to clear IPPCSE as it defaults to 0 */
	}
	ew32(RXCSUM, rxcsum);
	
	/*
	 * Enable early receives on supported devices, only takes effect when
	 * packet size is equal or larger than the specified value (in 8 byte
	 * units), e.g. using jumbo frames when setting to E1000_ERT_2048
	 */
	if (adapter.flags & FLAG_HAS_ERT) {
		if (mtu > ETH_DATA_LEN) {
			u32 rxdctl = er32(RXDCTL(0));
			ew32(RXDCTL(0), rxdctl | 0x3);
			ew32(ERT, E1000_ERT_2048 | (1 << 13));
			/*
			 * With jumbo frames and early-receive enabled,
			 * excessive C-state transition latencies result in
			 * dropped transactions.
			 */
#if	0
			pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
									  adapter.netdev->name, 55);
#endif
		} else {
#if	0
			pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
									  adapter.netdev->name,
									  PM_QOS_DEFAULT_VALUE);
#endif
		}
	}
	
	/* Enable Receives */
	ew32(RCTL, rctl);
}


/**
 * e1000_desc_unused - calculate if we have unused descriptors
 **/
int AppleIntelE1000e::e1000_desc_unused(struct e1000_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;
		
	return ring->count + ring->next_to_clean - ring->next_to_use - 1;
}


/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/
void AppleIntelE1000e::e1000_alloc_rx_buffers()
{
	struct e1000_rx_desc *rx_desc, *rx_desc_array;
	struct e1000_buffer *buffer_info;
	struct IOPhysicalSegment vector;
	
	mbuf_t skb;
	UInt32 count;
	
	UInt32 i;
	
	rx_desc_array = (e1000_rx_desc*)(adapter.rx_ring->desc->getBytesNoCopy());

	for (i = 0; i < adapter.rx_ring->count; i++) {
		
		buffer_info = &adapter.rx_ring->buffer_info[i];
		rx_desc = &rx_desc_array[i];
		
		//kIOEthernetMaxPacketSize
		skb = allocatePacket(adapter.rx_buffer_len);
		if (skb == NULL) {
			IOLog("allocatePacket failed in alloc_rx_buffer.\n");
			break;
		}
		
		//mbuf_adj(skb, 0x12);
		count = rxMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &vector, 1);
		if (count == 0) {
			IOLog("getPhysicalSegments failed in alloc_rx_buffer.\n");
			freePacket(skb);
			break;
		}
		buffer_info->skb = skb;
		rx_desc->buffer_addr = vector.location;
		//IOLog("index:%d, addr:%08x.\n", i, vector.location);
	}
	
	adapter.rx_ring->next_to_use = adapter.rx_ring->count - 1;
	E1000_WRITE_REG(&adapter.hw, adapter.rx_ring->tail, adapter.rx_ring->count - 2);
}


bool AppleIntelE1000e::e1000_clean_rx_irq()
{
	struct e1000_rx_desc *rx_desc, *next_rxd, *rx_desc_array;
	struct e1000_buffer *buffer_info, *next_buffer;
	struct e1000_hw* hw = &adapter.hw;
	
	UInt32 length;
	UInt32 head, tail;
	unsigned int i;
	bool cleaned = 0;
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	
	
	i = adapter.rx_ring->next_to_clean % adapter.rx_ring->count;
	rx_desc_array = (e1000_rx_desc*)(adapter.rx_ring->desc->getBytesNoCopy());
	rx_desc = &rx_desc_array[i];
	buffer_info = &adapter.rx_ring->buffer_info[i];
	head = E1000_READ_REG(hw, adapter.rx_ring->head);
	tail = E1000_READ_REG(hw, adapter.rx_ring->tail);
	
	while ((rx_desc->status & E1000_RXD_STAT_DD)) {
		mbuf_t skb;
		UInt8 status;
		UInt32 ckResult;
		struct IOPhysicalSegment vector;
		
		status = rx_desc->status;
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		i++;
		if (i >= adapter.rx_ring->count)
			i = 0;
		
		next_rxd = &rx_desc_array[i];
		next_buffer = &adapter.rx_ring->buffer_info[i];
		
		cleaned = true;
		length = le16_to_cpu(rx_desc->length);
#if	0
		if (rx_desc->errors){
			e_dbg("rx_desc->errors.\n");
			goto next_desc;
		}
#endif
		/* !EOP means multiple descriptors were used to store a single
		 * packet, also make sure the frame isn't just CRC only */
		if (!(status & E1000_RXD_STAT_EOP) || (length <= 4)) {
			/* All receives must fit into a single buffer */
			e_dbg("Receive packet consumed multiple buffers\n");
			/* recycle */
			buffer_info->skb = skb;
			goto next_desc;
		}

		if (rx_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK) {
			/* recycle */
			buffer_info->skb = skb;
			goto next_desc;
		}
		
		/* adjust length to remove Ethernet CRC */
		if (!(adapter.flags2 & FLAG2_CRC_STRIPPING))
			length -= 4;
		
		total_rx_bytes += length;
		total_rx_packets++;

		// skb_put(skb, length);

		/* Receive Checksum Offload */
		ckResult = 0;
		if (status & E1000_RXD_STAT_IPCS) {
			ckResult |= kChecksumIP;
		}
		if (status & E1000_RXD_STAT_UDPCS) {
			ckResult |= kChecksumUDP;
		}
		if (status & E1000_RXD_STAT_TCPCS) {
			ckResult |= kChecksumTCP;
		} 
		setChecksumResult(skb, kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, ckResult);
		netif->inputPacket(skb, length);
		netStats->inputPackets++;
		buffer_info->skb = allocatePacket(adapter.rx_buffer_len);
		//mbuf_adj(buffer_info->skb, 0x12);
		rxMbufCursor->getPhysicalSegmentsWithCoalesce(buffer_info->skb, &vector, 1);
		rx_desc->buffer_addr = vector.location;
		
	next_desc:
		rx_desc->status = 0;
		rx_desc->errors = 0;
		rx_desc->length = 0;
		
		// alloc
		adapter.rx_ring->next_to_use++;
		if (adapter.rx_ring->next_to_use >= adapter.rx_ring->count) {
			adapter.rx_ring->next_to_use = 0; 
			E1000_WRITE_REG(hw, adapter.rx_ring->tail, adapter.rx_ring->count - 1);
		} else {
			E1000_WRITE_REG(hw, adapter.rx_ring->tail, adapter.rx_ring->next_to_use - 1);
		}
		// alloc end
		
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	adapter.rx_ring->next_to_clean = i;
	
	/*
	 if (cleaned)
	 netif->flushInputQueue();
	 */
	
	return cleaned;
}

/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 *
 * the return value indicates if there is more work to do (later)
 **/
bool AppleIntelE1000e::e1000_clean_tx_irq()
{
	struct e1000_tx_desc *tx_desc, *eop_desc, *tx_desc_array;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
	bool cleaned = 0, retval = 1;
	bool do_service = false;
	
	i = adapter.tx_ring->next_to_clean;
	eop = adapter.tx_ring->buffer_info[i].next_to_watch;
	tx_desc_array = (e1000_tx_desc*)(adapter.tx_ring->desc->getBytesNoCopy());
	eop_desc = &tx_desc_array[eop];
	
	
	if (0) {		
		IOLog("DDING: clean_tx_irq: next_to_clean->%d, eop->%d.\n", i, eop);
	}
	
	while (eop_desc->upper.data & (E1000_TXD_STAT_DD)) {
		for (cleaned = 0; !cleaned; ) {
			tx_desc = &tx_desc_array[i];
			buffer_info = &adapter.tx_ring->buffer_info[i];
			
			cleaned = (i == eop);
			
			if (cleaned) {
				do_service = true;
			}
			
			if (buffer_info->skb) {
				freePacket(buffer_info->skb);
				buffer_info->skb = NULL;
			}
			
			tx_desc->upper.data = 0;
			
			i++;
			if (i >= adapter.tx_ring->count)
				i = 0;
		}
		
		eop = adapter.tx_ring->buffer_info[i].next_to_watch;
		eop_desc = &tx_desc_array[eop];
	}
	
	adapter.tx_ring->next_to_clean = i;
	
	
	if (do_service) {
		transmitQueue->service();
	}
	
	return retval;
}


void AppleIntelE1000e::e1000e_enable_receives()
{
	/* make sure the receive unit is started */
	if ((adapter.flags & FLAG_RX_NEEDS_RESTART) &&
	    (adapter.flags & FLAG_RX_RESTART_NOW)) {
		struct e1000_hw *hw = &adapter.hw;
		u32 rctl = er32(RCTL);
		ew32(RCTL, rctl | E1000_RCTL_EN);
		adapter.flags &= ~FLAG_RX_RESTART_NOW;
	}
}

bool AppleIntelE1000e::e1000_has_link()
{
	struct e1000_hw *hw = &adapter.hw;
	bool link_active = 0;
	SInt32 ret_val = 0;
	
	/*
	 * get_link_status is set on LSC (link status) interrupt or
	 * Rx sequence error interrupt.  get_link_status will stay
	 * false until the check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			ret_val = hw->mac.ops.check_for_link(hw);
			link_active = !hw->mac.get_link_status;
		} else {
			link_active = 1;
		}
		break;
	case e1000_media_type_fiber:
		ret_val = hw->mac.ops.check_for_link(hw);
		link_active = !!(er32(STATUS) & E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		ret_val = hw->mac.ops.check_for_link(hw);
		link_active = adapter.hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}
	
	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (er32(CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		e_info("Gigabit has been disabled, downgrading speed\n");
	}

	return link_active;
}


/**
 * e1000e_reset - bring the hardware into a known good state
 *
 * This function boots the hardware and enables some settings that
 * require a configuration cycle of the hardware - those cannot be
 * set/changed during runtime. After reset the device needs to be
 * properly configured for Rx, Tx etc.
 */
void AppleIntelE1000e::e1000e_reset()
{
	struct e1000_mac_info *mac = &adapter.hw.mac;
	struct e1000_fc_info *fc = &adapter.hw.fc;
	struct e1000_hw *hw = &adapter.hw;
	u32 tx_space, min_tx_space, min_rx_space;
	u32 pba = adapter.pba;
	u16 hwm;
	
	/* reset Packet Buffer Allocation to default */
	ew32(PBA, pba);
	
	if (adapter.max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) {
		/*
		 * To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB.
		 */
		pba = er32(PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/*
		 * the Tx fifo also stores 16 bytes of information about the tx
		 * but don't include ethernet FCS because hardware appends it
		 */
		min_tx_space = (adapter.max_frame_size +
						sizeof(struct e1000_tx_desc) -
						ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter.max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;
		
		/*
		 * If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation
		 */
		if ((tx_space < min_tx_space) &&
		    ((min_tx_space - tx_space) < pba)) {
			pba -= min_tx_space - tx_space;
			
			/*
			 * if short on Rx space, Rx wins and must trump tx
			 * adjustment or use Early Receive if available
			 */
			if ((pba < min_rx_space) &&
			    (!(adapter.flags & FLAG_HAS_ERT)))
			/* ERT enabled in e1000_configure_rx */
				pba = min_rx_space;
		}
		
		ew32(PBA, pba);
	}
	
	
	/*
	 * flow control settings
	 *
	 * The high water mark must be low enough to fit two full frames
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus the early receive size (for parts
	 *   with ERT support assuming ERT set to E1000_ERT_2048), or
	 * - the full Rx FIFO size minus two full frames
	 */
	if (hw->mac.type == e1000_pchlan) {
		/*
		 * Workaround PCH LOM adapter hangs with certain network
		 * loads.  If hangs persist, try disabling Tx flow control.
		 */
		if (mtu > ETH_DATA_LEN) {
			fc->high_water = 0x3500;
			fc->low_water  = 0x1500;
		} else {
			fc->high_water = 0x5000;
			fc->low_water  = 0x3000;
		}
	} else {
		if ((adapter.flags & FLAG_HAS_ERT) &&
		    (mtu > ETH_DATA_LEN))
			hwm = min(((pba << 10) * 9 / 10),
					  ((pba << 10) - (E1000_ERT_2048 << 3)));
		else
			hwm = min(((pba << 10) * 9 / 10),
					  ((pba << 10) - (2*adapter.max_frame_size)));
		
		fc->high_water = hwm & E1000_FCRTH_RTH; /* 8-byte granularity */
		fc->low_water = (fc->high_water - (2*adapter.max_frame_size));
		fc->low_water &= E1000_FCRTL_RTL; /* 8-byte granularity */
	}
	
	if (adapter.flags & FLAG_DISABLE_FC_PAUSE_TIME)
		fc->pause_time = 0xFFFF;
	else
		fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = 1;
	fc->current_mode = fc->requested_mode;
	
	/* Allow time for pending master requests to run */
	mac->ops.reset_hw(hw);
	
	/*
	 * For parts with AMT enabled, let the firmware know
	 * that the network interface is in control
	 */
	if (adapter.flags & FLAG_HAS_AMT)
		e1000_get_hw_control(&adapter);
	
	ew32(WUC, 0);
	if (adapter.flags2 & FLAG2_HAS_PHY_WAKEUP)
		e1e_wphy(&adapter.hw, BM_WUC, 0);
	
	if (mac->ops.init_hw(hw))
		e_err("Hardware Error\n");
	
	/* additional part of the flow-control workaround above */
	if (hw->mac.type == e1000_pchlan)
		ew32(FCRTV_PCH, 0x1000);
	
#ifdef NETIF_F_HW_VLAN_TX
	e1000_update_mng_vlan(adapter);
	
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	ew32(VET, ETH_P_8021Q);
	
#endif
	e1000e_reset_adaptive(hw);
	e1000_get_phy_info(hw);
	
	if ((adapter.flags & FLAG_HAS_SMART_POWER_DOWN) &&
	    !(adapter.flags & FLAG_SMART_POWER_DOWN)) {
		u16 phy_data = 0;
		/*
		 * speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed
		 */
		e1e_rphy(hw, IGP02E1000_PHY_POWER_MGMT, &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1e_wphy(hw, IGP02E1000_PHY_POWER_MGMT, phy_data);
	}
}


/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 **/
void AppleIntelE1000e::e1000_clean_tx_ring()
{
	struct e1000_hw* hw = &adapter.hw;
	
	bzero(tx_buffer_info, sizeof(tx_buffer_info));
	
	if (adapter.rx_ring->desc) {
		clear_desc(adapter.tx_ring->desc, SIZE_RING_DESC);
	}
	
	adapter.tx_ring->next_to_use = 0;
	adapter.tx_ring->next_to_clean = 0;
	
	E1000_WRITE_REG(hw, adapter.tx_ring->head, 0);
	E1000_WRITE_REG(hw, adapter.tx_ring->tail, 0);

	//writel(0, adapter.hw.hw_addr + tx_ring->head);
	//writel(0, adapter.hw.hw_addr + tx_ring->tail);
}


/**
 * e1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void AppleIntelE1000e::e1000_free_tx_resources()
{
	e1000_free_ring_resources(adapter.tx_ring);
}


/**
 * e1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 **/
void AppleIntelE1000e::e1000_clean_rx_ring()
{
	struct e1000_hw* hw = &adapter.hw;
	unsigned int i;
	
	for (i = 0; i < adapter.rx_ring->count; i++) {
		if (adapter.rx_ring->buffer_info[i].skb != NULL) {
			freePacket(adapter.rx_ring->buffer_info[i].skb);
			adapter.rx_ring->buffer_info[i].skb = NULL;
		}
	}
	
	if (adapter.rx_ring->desc) {
		clear_desc(adapter.rx_ring->desc, SIZE_RING_DESC);
	}
	
	adapter.rx_ring->next_to_use = 0;
	adapter.rx_ring->next_to_clean = 0;
	
	E1000_WRITE_REG(hw, adapter.rx_ring->head, 0);
	E1000_WRITE_REG(hw, adapter.rx_ring->tail, 0);
	
	//writel(0, adapter.hw.hw_addr + rx_ring->head);
	//writel(0, adapter.hw.hw_addr + rx_ring->tail);
}


/**
 * e1000_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/

void AppleIntelE1000e::e1000_free_rx_resources()
{
	e1000_free_ring_resources(adapter.rx_ring);
}

void AppleIntelE1000e::timeoutOccurred(IOTimerEventSource* src)
{
	watchdogSource->setTimeoutMS(3000);
	
	struct e1000_mac_info *mac = &adapter.hw.mac;
	struct e1000_hw *hw = &adapter.hw;
	UInt32 link, tctl;
	
	link = e1000_has_link();
	
	//e_dbg("timeout link:%d, preLinkStatus:%d.\n", link, preLinkStatus);
	if (link && link == preLinkStatus) {
		//e1000e_enable_receives();
		goto link_up;
	}
	
#ifdef NETIF_F_HW_VLAN_TX
	if ((e1000e_enable_tx_pkt_filtering(hw)) &&
	    (adapter->mng_vlan_id != adapter->hw.mng_cookie.vlan_id))
		e1000_update_mng_vlan(adapter);
	
#endif
	if (link) {
		mac->ops.get_link_up_info(&adapter.hw,
								  &adapter.link_speed,
								  &adapter.link_duplex);
		e1000_print_link_info();
		/*
		 * On supported PHYs, check for duplex mismatch only
		 * if link has autonegotiated at 10/100 half
		 */
		if ((hw->phy.type == e1000_phy_igp_3 ||
		     hw->phy.type == e1000_phy_bm) &&
		    (hw->mac.autoneg == true) &&
		    (adapter.link_speed == SPEED_10 ||
		     adapter.link_speed == SPEED_100) &&
		    (adapter.link_duplex == HALF_DUPLEX)) {
			UInt16 autoneg_exp;
			
			hw->phy.ops.read_reg(hw, PHY_AUTONEG_EXP,
								 &autoneg_exp);
			
			if (!(autoneg_exp & NWAY_ER_LP_NWAY_CAPS))
				e_info("Autonegotiated half duplex but"
				       " link partner cannot autoneg. "
				       " Try forcing full duplex if "
				       "link gets many collisions.");
		}
		
		/*
		 * enable transmits in the hardware, need to do this
		 * after setting TARC(0)
		 */
		tctl = E1000_READ_REG(hw, E1000_TCTL);
		tctl |= E1000_TCTL_EN;
		E1000_WRITE_REG(hw, E1000_TCTL, tctl);
		e1000e_enable_receives();
		E1000_READ_REG(hw, E1000_STATUS);
		UInt32 speed = 1000 * MBit;
		if(adapter.link_speed == SPEED_100)
			speed = 100 * MBit;
		else if(adapter.link_speed == SPEED_10)
			speed = 10 * MBit;
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), speed);
		if (transmitQueue) {
			transmitQueue->start();
		}
		
	} else {
		
		if (link != preLinkStatus) {
			adapter.link_speed = 0;
			adapter.link_duplex = 0;
			e_info("Link is Down\n");
			
			transmitQueue->stop();
			setLinkStatus(kIONetworkLinkValid, 0);
			
		}
	}
	
link_up:
	// e1000e_update_stats(adapter);
	// e1000e_update_adaptive(&adapter->hw);

	/* Cause software interrupt to ensure Rx ring is cleaned */
#ifdef CONFIG_E1000E_MSIX
	if (adapter->msix_entries)
		ew32(ICS, adapter->rx_ring->ims_val);
	else
		ew32(ICS, E1000_ICS_RXDMT0);
#else
	ew32(ICS, E1000_ICS_RXDMT0);
#endif
	/* Force detection of hung controller every watchdog period */
	adapter.detect_tx_hung = 1;
	
	/*
	 * With 82571 controllers, LAA may be overwritten due to controller
	 * reset from the other port. Set the appropriate LAA in RAR[0]
	 */
	if (e1000e_get_laa_state_82571(hw))
		e1000e_rar_set(hw, adapter.hw.mac.addr, 0);
	
	preLinkStatus = link;
}

void AppleIntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)
{
	//e_dbg("void AppleIntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)\n");
	AppleIntelE1000e* me = (AppleIntelE1000e*) target;
	me->timeoutOccurred(src);
	
}

void AppleIntelE1000e::e1000_print_link_info()
{
	struct e1000_hw *hw = &adapter.hw;
	UInt32 ctrl = E1000_READ_REG(hw, E1000_CTRL);
	
	e_info("Link is Up %d Mbps %s, Flow Control: %s\n",
		   adapter.link_speed,
		   (adapter.link_duplex == FULL_DUPLEX) ?
	       "Full Duplex" : "Half Duplex",
		   ((ctrl & E1000_CTRL_TFCE) && (ctrl & E1000_CTRL_RFCE)) ?
	       "RX/TX" :
		   ((ctrl & E1000_CTRL_RFCE) ? "RX" :
			((ctrl & E1000_CTRL_TFCE) ? "TX" : "None" )));
}

void AppleIntelE1000e::e1000_set_multi()
{
	struct e1000_hw *hw = &adapter.hw;
	u32 rctl;
	
	/* Check for Promiscuous and All Multicast modes */
	
	rctl = er32(RCTL);

	if (promiscusMode) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		rctl &= ~E1000_RCTL_VFE;
	} else {
		if (multicastMode) {
			rctl |= E1000_RCTL_MPE;
			rctl &= ~E1000_RCTL_UPE;
		} else {
			rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
		}
		if (adapter.flags & FLAG_HAS_HW_VLAN_FILTER)
			rctl |= E1000_RCTL_VFE;
	}
	
	ew32(RCTL, rctl);
}

/**
 * e1000_configure - configure the hardware for Rx and Tx
 * @adapter: private board structure
 **/
void AppleIntelE1000e::e1000_configure()
{
	setMulticastList(NULL,0);
#ifdef NETIF_F_HW_VLAN_TX
	e1000_restore_vlan(&adapter);
#endif
	e1000_init_manageability();
	e1000_configure_tx();
	e1000_setup_rctl();
	e1000_configure_rx();
	e1000_alloc_rx_buffers();
}

/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 **/
void AppleIntelE1000e::e1000_irq_disable()
{
	struct e1000_hw *hw = &adapter.hw;

	interruptEnabled = false;

	ew32(IMC, ~0);
#ifdef CONFIG_E1000E_MSIX
	if (adapter.msix_entries)
		ew32(EIAC_82574, 0);
#endif /* CONFIG_E1000E_MSIX */
	e1e_flush();
	// synchronize_irq(adapter.pdev->irq);
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 **/
void AppleIntelE1000e::e1000_irq_enable()
{
	struct e1000_hw *hw = &adapter.hw;

#ifdef CONFIG_E1000E_MSIX
	if (adapter.msix_entries) {
		ew32(EIAC_82574, adapter.eiac_mask & E1000_EIAC_MASK_82574);
		ew32(IMS, adapter.eiac_mask | E1000_IMS_OTHER | E1000_IMS_LSC);
	} else {
		ew32(IMS, IMS_ENABLE_MASK);
	}
#else
	ew32(IMS, IMS_ENABLE_MASK);
#endif /* CONFIG_E1000E_MSIX */
	e1e_flush();

	interruptEnabled = true;
}


bool AppleIntelE1000e::e1000_tx_csum(mbuf_t skb)
{
	struct e1000_ring *tx_ring = adapter.tx_ring;
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	u8 css, cso;
	u32 cmd_len = E1000_TXD_CMD_DEXT;
	

	UInt32 checksumDemanded;
	getChecksumDemand(skb, kChecksumFamilyTCPIP, &checksumDemanded);
	if((checksumDemanded & (kChecksumIP|kChecksumTCP|kChecksumUDP)) == 0)
		return false;

	if(	checksumDemanded & kChecksumTCP )
		cmd_len |= E1000_TXD_CMD_TCP;

	
	css = ETHER_HDR_LEN;
	struct ip* iphdr = (struct ip*)((u8*)mbuf_data(skb) + ETHER_HDR_LEN);
	css += iphdr->ip_hl << 2;
	//IOLog("IP Header Len = %d\n", css - ETHER_HDR_LEN);
	if(checksumDemanded & kChecksumTCP)
		cso = css + offsetof(struct tcphdr, th_sum);
	else if(checksumDemanded & kChecksumUDP)
		cso = css + offsetof(struct udphdr, uh_sum);
	
	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
	
	context_desc->lower_setup.ip_config = 0;
	context_desc->upper_setup.tcp_fields.tucss = css;
	context_desc->upper_setup.tcp_fields.tucso = cso;
	context_desc->upper_setup.tcp_fields.tucse = 0;
	context_desc->tcp_seg_setup.data = 0;
	context_desc->cmd_and_length = cpu_to_le32(cmd_len);
#if defined(MAC_OS_X_VERSION_10_6)
	clock_sec_t seconds;
	clock_usec_t microsecs;
#else
	uint32_t seconds;
	uint32_t microsecs;
#endif
	clock_get_system_microtime(&seconds, &microsecs);
	buffer_info->time_stamp =  seconds * 100 + microsecs / 10000; // 10 ms
	buffer_info->next_to_watch = i;
	
	i++;
	if (i >= tx_ring->count)
		i = 0;
	tx_ring->next_to_use = i;
	
	return true;
}

IOReturn AppleIntelE1000e::registerWithPolicyMaker ( IOService * policyMaker )
{
	e_dbg("void AppleIntelE1000e::registerWithPolicyMaker()\n");
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	powerState = 1;
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}

IOReturn AppleIntelE1000e::setPowerState( unsigned long powerStateOrdinal,
								IOService *policyMaker )
{
	e_dbg("void AppleIntelE1000e::setPowerState(%d)\n",(int)powerStateOrdinal);
	if (powerState == powerStateOrdinal)
		return IOPMAckImplied;
	powerState = powerStateOrdinal;

	if(powerState == 1){	//
		;
	} else {
		suspend = true;
		//e1000_power_down_phy(&adapter);
	}
	/* acknowledge the completion of our power state change */
#if	0
	policyMaker->acknowledgeSetPowerState();	
#endif

    return IOPMAckImplied;
}

IOReturn AppleIntelE1000e::getMaxPacketSize (UInt32 *maxSize) const {
	if (maxSize)
		*maxSize = adapter.max_hw_frame_size - (ETH_HLEN + ETH_FCS_LEN);
	
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::getMinPacketSize (UInt32 *minSize) const {
	if(minSize)
		*minSize = ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN;
	
	return kIOReturnSuccess;
}

/**
 * e1000_set_mtu - Change the Maximum Transfer Unit
 **/

void AppleIntelE1000e::e1000_set_mtu(UInt32 maxSize){
	UInt32 max_frame = maxSize + ETH_HLEN + ETH_FCS_LEN;

	adapter.max_frame_size = max_frame;
	e_info("changing MTU from %d to %d\n", (int)mtu, (int)maxSize);
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

	adapter.min_frame_size = ETHER_MIN_LEN;

	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, 1);
	txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, TBDS_PER_TCB);
}

IOReturn AppleIntelE1000e::setMaxPacketSize (UInt32 maxSize){
	if(maxSize == mtu)
		return kIOReturnSuccess;

	//if (netif_running(netdev))
	//	e1000e_down(adapter);
	
	e1000_set_mtu(maxSize);
	
	e1000e_reset();
	
	return kIOReturnSuccess;
}

