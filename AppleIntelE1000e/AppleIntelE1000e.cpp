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

#define USE_RX_IP_CHECKSUM	1
#define USE_RX_UDP_CHECKSUM	1
#define USE_TX_IP_CHECKSUM	0
#define USE_TX_UDP_CHECKSUM	1

#define TBDS_PER_TCB 12
#define super IOEthernetController

#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_NO_FCS		0x00000010
#define E1000_TX_FLAGS_HWTSTAMP		0x00000020
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16

static int cards_found = 0;

#define	RELEASE(x)	if(x){x->release();x=NULL;}

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
#define	HZ	100
#define	time_after(a,b)	((a)>(b))

/**
 * e1000_desc_unused - calculate if we have unused descriptors
 **/
static int e1000_desc_unused(struct e1000_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;
	
	return ring->count + ring->next_to_clean - ring->next_to_use - 1;
}

#ifdef HAVE_HW_TIME_STAMP
/**
 * e1000e_systim_to_hwtstamp - convert system time value to hw time stamp
 * @adapter: board private structure
 * @hwtstamps: time stamp structure to update
 * @systim: unsigned 64bit system time value.
 *
 * Convert the system time value stored in the RX/TXSTMP registers into a
 * hwtstamp which can be used by the upper level time stamping functions.
 *
 * The 'systim_lock' spinlock is used to protect the consistency of the
 * system time value. This is needed because reading the 64 bit time
 * value involves reading two 32 bit registers. The first read latches the
 * value.
 **/
static void e1000e_systim_to_hwtstamp(struct e1000_adapter *adapter,
									  struct skb_shared_hwtstamps *hwtstamps,
									  u64 systim)
{
	u64 ns;
	unsigned long flags;
	
	spin_lock_irqsave(&adapter->systim_lock, flags);
	ns = timecounter_cyc2time(&adapter->tc, systim);
	spin_unlock_irqrestore(&adapter->systim_lock, flags);
	
	memset(hwtstamps, 0, sizeof(*hwtstamps));
	hwtstamps->hwtstamp = ns_to_ktime(ns);
}

/**
 * e1000e_rx_hwtstamp - utility function which checks for Rx time stamp
 * @adapter: board private structure
 * @status: descriptor extended error and status field
 * @skb: particular skb to include time stamp
 *
 * If the time stamp is valid, convert it into the timecounter ns value
 * and store that result into the shhwtstamps structure which is passed
 * up the network stack.
 **/
static void e1000e_rx_hwtstamp(struct e1000_adapter *adapter, u32 status,
							   struct sk_buff *skb)
{
	struct e1000_hw *hw = &adapter->hw;
	u64 rxstmp;
	
	if (!(adapter->flags & FLAG_HAS_HW_TIMESTAMP) ||
	    !(status & E1000_RXDEXT_STATERR_TST) ||
	    !(er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_VALID))
		return;
	
	/* The Rx time stamp registers contain the time stamp.  No other
	 * received packet will be time stamped until the Rx time stamp
	 * registers are read.  Because only one packet can be time stamped
	 * at a time, the register values must belong to this packet and
	 * therefore none of the other additional attributes need to be
	 * compared.
	 */
	rxstmp = (u64)er32(RXSTMPL);
	rxstmp |= (u64)er32(RXSTMPH) << 32;
	e1000e_systim_to_hwtstamp(adapter, skb_hwtstamps(skb), rxstmp);
	
	adapter->flags2 &= ~FLAG2_CHECK_RX_HWTSTAMP;
}
#endif /* HAVE_HW_TIME_STAMP */


static void e1000e_disable_aspm(IOPCIDevice *pci_dev, u16 state)
{
	u8 pos;
	u16 reg16;
	u16 aspm_ctl = 0;
	
	if (state & PCIE_LINK_STATE_L0S)
		aspm_ctl |= PCI_EXP_LNKCTL_ASPM_L0S;
	if (state & PCIE_LINK_STATE_L1)
		aspm_ctl |= PCI_EXP_LNKCTL_ASPM_L1;
	
	/* Both device and parent should have the same ASPM setting.
	 * Disable ASPM in downstream component first and then upstream.
	 */
	pci_dev->findPCICapability(kIOPCIPCIExpressCapability, &pos);
	reg16 = pci_dev->configRead16(pos + PCI_EXP_LNKCTL);
	reg16 &= ~aspm_ctl;
	pci_dev->configWrite16(pos + PCI_EXP_LNKCTL, reg16);
#if	0
	if (!pdev->bus->self)
		return;
	
	pos = pci_pcie_cap(pdev->bus->self);
	pci_read_config_word(pdev->bus->self, pos + PCI_EXP_LNKCTL, &reg16);
	reg16 &= ~aspm_ctl;
	pci_write_config_word(pdev->bus->self, pos + PCI_EXP_LNKCTL, reg16);
#endif
}

// Not Used
static void e1000_power_off(IOPCIDevice *pdev, bool sleep, bool wake)
{
#if	0
	if (sleep && wake) {
		pci_prepare_to_sleep(pdev);
		return;
	}
	
	pci_wake_from_d3(pdev, wake);
	pci_set_power_state(pdev, PCI_D3hot);
#endif
}

// Not Used
static void e1000_complete_shutdown(e1000_adapter *adapter, IOPCIDevice *pdev, bool sleep, bool wake)
{
	
	/* The pci-e switch on some quad port adapters will report a
	 * correctable error when the MAC transitions from D0 to D3.  To
	 * prevent this we need to mask off the correctable errors on the
	 * downstream port of the pci-e switch.
	 */
	if (adapter->flags & FLAG_IS_QUAD_PORT) {
		u8 pos;
		pdev->findPCICapability(kIOPCIPCIExpressCapability, &pos);
		u16 devctl;
		
		devctl = pdev->configRead16(pos + PCI_EXP_LNKCTL);
		pdev->configWrite16(pos + PCI_EXP_DEVCTL, (devctl & ~PCI_EXP_DEVCTL_CERE));

		
		e1000_power_off(pdev, sleep, wake);
		
		pdev->configWrite16(pos + PCI_EXP_DEVCTL, devctl);
	} else {
		e1000_power_off(pdev, sleep, wake);
	}
}


/**
 * e1000_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 **/
static int e1000_alloc_queues(struct e1000_adapter *adapter)
{
	adapter->tx_ring = (e1000_ring*)IOMalloc(sizeof(struct e1000_ring));
	if (adapter->tx_ring){
        bzero(adapter->tx_ring, sizeof(e1000_ring));
        adapter->tx_ring->count = adapter->tx_ring_count;
        adapter->tx_ring->adapter = adapter;
		adapter->rx_ring = (e1000_ring*)IOMalloc(sizeof(struct e1000_ring));
		if (adapter->rx_ring){
			bzero(adapter->rx_ring, sizeof(e1000_ring));
            adapter->rx_ring->count = adapter->rx_ring_count;
            adapter->rx_ring->adapter = adapter;
			return 0;
		}
		IOFree(adapter->tx_ring, sizeof(e1000_ring));
	}
	return -ENOMEM;
}

/**
* e1000_configure_msix - Configure MSI-X hardware
*
* e1000_configure_msix sets up the hardware to properly
* generate MSI-X interrupts.
**/
static void e1000_configure_msix(struct e1000_adapter *adapter)
{
}

void e1000e_reset_interrupt_capability(struct e1000_adapter *adapter)
{
}

/**
 * e1000e_set_interrupt_capability - set MSI or MSI-X if supported
 *
 * Attempt to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
void e1000e_set_interrupt_capability(struct e1000_adapter *adapter)
{
	adapter->int_mode = E1000E_INT_MODE_LEGACY;
	adapter->msix_entries = NULL;
	/* store the number of vectors being used */
	adapter->num_vectors = 1;
}


/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 **/
static void e1000_irq_disable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	
	ew32(IMC, ~0);
	if (adapter->msix_entries)
		ew32(EIAC_82574, 0);
	e1e_flush();
	
	if (adapter->msix_entries) {
		int i;
		for (i = 0; i < adapter->num_vectors; i++)
			synchronize_irq(adapter->msix_entries[i].vector);
	} else {
		synchronize_irq(adapter->pdev->irq);
	}
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 **/
static void e1000_irq_enable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	if (adapter->msix_entries) {
		ew32(EIAC_82574, adapter->eiac_mask & E1000_EIAC_MASK_82574);
		ew32(IMS, adapter->eiac_mask | E1000_IMS_OTHER | E1000_IMS_LSC);
	} else if (hw->mac.type == e1000_pch_lpt) {
		ew32(IMS, IMS_ENABLE_MASK | E1000_IMS_ECCER);
	} else {
		ew32(IMS, IMS_ENABLE_MASK);
	}
	e1e_flush();
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
	struct net_device *netdev = adapter->netdev;
	
	adapter->rx_buffer_len = ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN;
	adapter->rx_ps_bsize0 = 128;
	adapter->max_frame_size = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	adapter->min_frame_size = ETH_ZLEN + ETH_FCS_LEN;
	
	e1000e_set_interrupt_capability(adapter);
	
	if (e1000_alloc_queues(adapter))
		return -ENOMEM;
	
#ifdef HAVE_HW_TIME_STAMP
	/* Setup hardware time stamping cyclecounter */
	if (adapter->flags & FLAG_HAS_HW_TIMESTAMP) {
		adapter->cc.read = e1000e_cyclecounter_read;
		adapter->cc.mask = CLOCKSOURCE_MASK(64);
		adapter->cc.mult = 1;
		/* cc.shift set in e1000e_get_base_tininca() */
		
		spin_lock_init(&adapter->systim_lock);
		INIT_WORK(&adapter->tx_hwtstamp_work, e1000e_tx_hwtstamp_work);
	}
#endif /* HAVE_HW_TIME_STAMP */

	/* Explicitly disable IRQ since the NIC can be in any state. */
	e1000_irq_disable(adapter);
	
	set_bit(__E1000_DOWN, &adapter->state);
	return 0;
}

static void e1000_eeprom_checks(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int ret_val;
	u16 buf = 0;
	
	if (hw->mac.type != e1000_82573)
		return;
	
	ret_val = e1000_read_nvm(hw, NVM_INIT_CONTROL2_REG, 1, &buf);
	le16_to_cpus(&buf);
	if (!ret_val && (!(buf & (1 << 0)))) {
		/* Deep Smart Power Down (DSPD) */
		IOLog("Warning: detected DSPD enabled in EEPROM\n");
	}
}

/**
 * @e1000_alloc_ring - allocate memory for a ring structure
 **/
static int e1000_alloc_ring_dma(struct e1000_adapter *adapter,
								struct e1000_ring *ring)
{
	ring->pool = IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
													   ring->size, PAGE_SIZE );

	if (!ring->pool)
		return -ENOMEM;

	ring->pool->prepare();
	ring->desc = ring->pool->getBytesNoCopy();
	ring->dma = ring->pool->getPhysicalAddress();

	bzero(ring->desc, ring->size);
	
	return 0;
}

/**
 * e1000e_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
int e1000e_setup_tx_resources(struct e1000_ring *tx_ring)
{
	struct e1000_adapter *adapter = tx_ring->adapter;
	int err = -ENOMEM, size;
	
	size = sizeof(struct e1000_buffer) * tx_ring->count;
	tx_ring->buffer_info = (e1000_buffer*)IOMalloc(size);
	if (!tx_ring->buffer_info)
		goto err;
	bzero(tx_ring->buffer_info, size);
	
	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(struct e1000_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);
	
	err = e1000_alloc_ring_dma(adapter, tx_ring);
	if (err)
		goto err;
	
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	
	return 0;
err:
	IOFree(tx_ring->buffer_info, size);
	e_err("Unable to allocate memory for the transmit descriptor ring\n");
	return err;
}

/**
 * e1000e_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: board private structure
 *
 * Returns 0 on success, negative on failure
 **/
int e1000e_setup_rx_resources(struct e1000_ring *rx_ring)
{
	struct e1000_adapter *adapter = rx_ring->adapter;
	struct e1000_buffer *buffer_info;
	int i, size, desc_len, err = -ENOMEM;
	
	size = sizeof(struct e1000_buffer) * rx_ring->count;
	rx_ring->buffer_info = (e1000_buffer*)IOMalloc(size);
	if (!rx_ring->buffer_info)
		goto err;
	bzero(rx_ring->buffer_info, size);
	
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		buffer_info->ps_pages = (e1000_ps_page*)IOMalloc(PS_PAGE_BUFFERS * sizeof(struct e1000_ps_page));
		if (!buffer_info->ps_pages)
			goto err_pages;
		bzero(buffer_info->ps_pages, PS_PAGE_BUFFERS * sizeof(struct e1000_ps_page));
	}
	
	desc_len = sizeof(union e1000_rx_desc_packet_split);
	
	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * desc_len;
	rx_ring->size = ALIGN(rx_ring->size, 4096);
	
	err = e1000_alloc_ring_dma(adapter, rx_ring);
	if (err)
		goto err_pages;
	
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	rx_ring->rx_skb_top = NULL;
	
	return 0;
	
err_pages:
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		IOFree(buffer_info->ps_pages, PS_PAGE_BUFFERS * sizeof(struct e1000_ps_page));
	}
err:
	IOFree(rx_ring->buffer_info, size);
	e_err("Unable to allocate memory for the receive descriptor ring\n");
	return err;
}



/**
 * e1000e_get_hw_control - get control of the h/w from f/w
 * @adapter: address of board private structure
 *
 * e1000e_get_hw_control sets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded. For AMT version (only with 82573)
 * of the f/w this means that the network i/f is open.
 **/
void e1000e_get_hw_control(struct e1000_adapter *adapter)
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
 * e1000e_release_hw_control - release control of the h/w to f/w
 * @adapter: address of board private structure
 *
 * e1000e_release_hw_control resets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded. For AMT version (only with 82573) i
 * of the f/w this means that the network i/f is closed.
 *
 **/
void e1000e_release_hw_control(struct e1000_adapter *adapter)
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
 * e1000e_reset - bring the hardware into a known good state
 *
 * This function boots the hardware and enables some settings that
 * require a configuration cycle of the hardware - those cannot be
 * set/changed during runtime. After reset the device needs to be
 * properly configured for Rx, Tx etc.
 */
void e1000e_reset(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_fc_info *fc = &adapter->hw.fc;
	struct e1000_hw *hw = &adapter->hw;
	u32 tx_space, min_tx_space, min_rx_space;
	u32 pba = adapter->pba;
	u16 hwm;
	
	/* reset Packet Buffer Allocation to default */
	ew32(PBA, pba);
	
	if (adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) {
		/* To maintain wire speed transmits, the Tx FIFO should be
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
		/* the Tx fifo also stores 16 bytes of information about the Tx
		 * but don't include ethernet FCS because hardware appends it
		 */
		min_tx_space = (adapter->max_frame_size +
						sizeof(struct e1000_tx_desc) - ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;
		
		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation
		 */
		if ((tx_space < min_tx_space) &&
		    ((min_tx_space - tx_space) < pba)) {
			pba -= min_tx_space - tx_space;
			
			/* if short on Rx space, Rx wins and must trump Tx
			 * adjustment
			 */
			if (pba < min_rx_space)
				pba = min_rx_space;
		}
		
		ew32(PBA, pba);
	}
	
	
	/* flow control settings
	 *
	 * The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus one full frame
	 */
	if (adapter->flags & FLAG_DISABLE_FC_PAUSE_TIME)
		fc->pause_time = 0xFFFF;
	else
		fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = true;
	fc->current_mode = fc->requested_mode;

	switch (hw->mac.type) {
		case e1000_ich9lan:
		case e1000_ich10lan:
			if (adapter->netdev->mtu > ETH_DATA_LEN) {
				pba = 14;
				ew32(PBA, pba);
				fc->high_water = 0x2800;
				fc->low_water = fc->high_water - 8;
				break;
			}
			/* fall-through */
		default:
			hwm = min(((pba << 10) * 9 / 10),
					  ((pba << 10) - adapter->max_frame_size));
			
			fc->high_water = hwm & E1000_FCRTH_RTH; /* 8-byte granularity */
			fc->low_water = fc->high_water - 8;
			break;
		case e1000_pchlan:
			/* Workaround PCH LOM adapter hangs with certain network
			 * loads.  If hangs persist, try disabling Tx flow control.
			 */
			if (adapter->netdev->mtu > ETH_DATA_LEN) {
				fc->high_water = 0x3500;
				fc->low_water  = 0x1500;
			} else {
				fc->high_water = 0x5000;
				fc->low_water  = 0x3000;
			}
			fc->refresh_time = 0x1000;
			break;
		case e1000_pch2lan:
		case e1000_pch_lpt:
			fc->refresh_time = 0x0400;

            if (adapter->netdev->mtu > ETH_DATA_LEN) {
				fc->high_water = 0x05C20;
				fc->low_water = 0x05048;
				fc->pause_time = 0x0650;
            }

			fc->high_water = ((pba << 10) * 9 / 10) & E1000_FCRTH_RTH;
			fc->low_water = ((pba << 10) * 8 / 10) & E1000_FCRTL_RTL;
			break;
	}

	/* Alignment of Tx data is on an arbitrary byte boundary with the
	 * maximum size per Tx descriptor limited only to the transmit
	 * allocation of the packet buffer minus 96 bytes with an upper
	 * limit of 24KB due to receive synchronization limitations.
	 */
	adapter->tx_fifo_limit = min_t(u32, ((er32(PBA) >> 16) << 10) - 96,
								   24 << 10);

	/* Disable Adaptive Interrupt Moderation if 2 full packets cannot
	 * fit in receive buffer.
	 */
	if (adapter->itr_setting & 0x3) {
		if ((adapter->max_frame_size * 2) > (pba << 10)) {
			if (!(adapter->flags2 & FLAG2_DISABLE_AIM)) {
				IOLog("Interrupt Throttle Rate turned off\n");
				adapter->flags2 |= FLAG2_DISABLE_AIM;
				e1000e_write_itr(adapter, 0);
			}
		} else if (adapter->flags2 & FLAG2_DISABLE_AIM) {
			IOLog( "Interrupt Throttle Rate turned on\n");
			adapter->flags2 &= ~FLAG2_DISABLE_AIM;
			adapter->itr = 20000;
			e1000e_write_itr(adapter, adapter->itr);
		}
	}

	/* Allow time for pending master requests to run */
	mac->ops.reset_hw(hw);
	
	/* For parts with AMT enabled, let the firmware know
	 * that the network interface is in control
	 */
	if (adapter->flags & FLAG_HAS_AMT)
		e1000e_get_hw_control(adapter);
	
	ew32(WUC, 0);
	if (adapter->flags2 & FLAG2_HAS_PHY_WAKEUP)
		e1e_wphy(&adapter->hw, BM_WUC, 0);
	
	if (mac->ops.init_hw(hw))
		e_err("Hardware Error\n");
	
#ifdef NETIF_F_HW_VLAN_TX
	e1000_update_mng_vlan(adapter);
	
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	ew32(VET, ETH_P_8021Q);
	
#endif
	e1000e_reset_adaptive(hw);

#ifdef HAVE_HW_TIME_STAMP
	/* initialize systim and reset the ns time counter */
	e1000e_config_hwtstamp(adapter);
#endif
	
	/* Set EEE advertisement as appropriate */
	if (adapter->flags2 & FLAG2_HAS_EEE) {
		s32 ret_val;
		u16 adv_addr;
		
		switch (hw->phy.type) {
			case e1000_phy_82579:
				adv_addr = I82579_EEE_ADVERTISEMENT;
				break;
			case e1000_phy_i217:
				adv_addr = I217_EEE_ADVERTISEMENT;
				break;
			default:
				e_info("Invalid PHY type setting EEE advertisement\n");
				return;
		}
		
		ret_val = hw->phy.ops.acquire(hw);
		if (ret_val) {
			e_info("EEE advertisement - unable to acquire PHY\n");
			return;
		}
		
		if (!adapter->eee_advert)
			dev_info(pci_dev_to_dev(adapter->pdev),
					 "Disabling EEE advertisement\n");
		
		e1000_write_emi_reg_locked(hw, adv_addr, adapter->eee_advert);
		
		hw->phy.ops.release(hw);
	}
	
#if 0
	if (!netif_running(adapter->netdev) &&
	    !test_bit(__E1000_TESTING, &adapter->state)) {
		e1000_power_down_phy(adapter);
		return;
	}
#endif
    
	e1000_get_phy_info(hw);
	
	if ((adapter->flags & FLAG_HAS_SMART_POWER_DOWN) &&
	    !(adapter->flags & FLAG_SMART_POWER_DOWN)) {
		u16 phy_data = 0;
		/* speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed
		 */
		e1e_rphy(hw, IGP02E1000_PHY_POWER_MGMT, &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1e_wphy(hw, IGP02E1000_PHY_POWER_MGMT, phy_data);
	}
}


/**
 * e1000e_write_mc_addr_list - write multicast addresses to MTA
 * @netdev: network interface device structure
 *
 * Writes multicast address list to the MTA hash table.
 * Returns: -ENOMEM on failure
 *                0 on no addresses written
 *                X on writing X addresses to MTA
 */
static int e1000e_write_mc_addr_list(struct e1000_hw *hw, u8 *mc_addr_list,
									  u32 mc_addr_count)
{
	hw->mac.ops.update_mc_addr_list(hw, mc_addr_list, mc_addr_count);
	return 0;
}

#ifdef HAVE_SET_RX_MODE
/**
 * e1000e_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int e1000e_write_uc_addr_list(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	unsigned int rar_entries = hw->mac.rar_entry_count;
	int count = 0;
	
	/* save a rar entry for our hardware address */
	rar_entries--;
	
	/* save a rar entry for the LAA workaround */
	if (adapter->flags & FLAG_RESET_OVERWRITES_LAA)
		rar_entries--;
	
	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > rar_entries)
		return -ENOMEM;
	
	if (!netdev_uc_empty(netdev) && rar_entries) {
#ifdef NETDEV_HW_ADDR_T_UNICAST
		struct netdev_hw_addr *ha;
#else
		struct dev_mc_list *ha;
#endif
		
		/* write the addresses in reverse order to avoid write
		 * combining
		 */
		netdev_for_each_uc_addr(ha, netdev) {
			if (!rar_entries)
				break;
#ifdef NETDEV_HW_ADDR_T_UNICAST
			hw->mac.ops.rar_set(hw, ha->addr, rar_entries--);
#else
			hw->mac.ops.rar_set(hw, ha->da_addr, rar_entries--);
#endif
			count++;
		}
	}
	
	/* zero out the remaining RAR entries not used above */
	for (; rar_entries > 0; rar_entries--) {
		ew32(RAH(rar_entries), 0);
		ew32(RAL(rar_entries), 0);
	}
	e1e_flush();
	
	return count;
}

#endif /* HAVE_SET_RX_MODE */

/**
 * e1000e_vlan_strip_enable - helper to disable HW VLAN stripping
 * @adapter: board private structure to initialize
 **/
static void e1000e_vlan_strip_disable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl;
	
	/* disable VLAN tag insert/strip */
	ctrl = er32(CTRL);
	ctrl &= ~E1000_CTRL_VME;
	ew32(CTRL, ctrl);
}

/**
 * e1000e_vlan_strip_enable - helper to enable HW VLAN stripping
 * @adapter: board private structure to initialize
 **/
static void e1000e_vlan_strip_enable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl;
	
	/* enable VLAN tag insert/strip */
	ctrl = er32(CTRL);
	ctrl |= E1000_CTRL_VME;
	ew32(CTRL, ctrl);
}

static void e1000e_update_rdt_wa(struct e1000_ring *rx_ring, unsigned int i)
{
	struct e1000_adapter *adapter = rx_ring->adapter;
	struct e1000_hw *hw = &adapter->hw;
	s32 ret_val = __ew32_prepare(hw);

	writel(i, rx_ring->tail);
	
	if (unlikely(!ret_val && (i != readl(rx_ring->tail)))) {
		u32 rctl = er32(RCTL);
		ew32(RCTL, rctl & ~E1000_RCTL_EN);
		e_err("ME firmware caused invalid RDT - resetting\n");
		schedule_work(&adapter->reset_task);
	}
}

static void e1000e_update_tdt_wa(struct e1000_ring *tx_ring, unsigned int i)
{
	struct e1000_adapter *adapter = tx_ring->adapter;
	struct e1000_hw *hw = &adapter->hw;
	s32 ret_val = __ew32_prepare(hw);
	
	writel(i, tx_ring->tail);
	
	if (unlikely(!ret_val && (i != readl(tx_ring->tail)))) {
		u32 tctl = er32(TCTL);
		ew32(TCTL, tctl & ~E1000_TCTL_EN);
		e_err("ME firmware caused invalid TDT - resetting\n");
		schedule_work(&adapter->reset_task);
	}
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
static unsigned int e1000_update_itr(u16 itr_setting, int packets, int bytes)
{
	unsigned int retval = itr_setting;
	
	if (packets == 0)
		return itr_setting;
	
	switch (itr_setting) {
		case lowest_latency:
			/* handle TSO and jumbo frames */
			if (bytes/packets > 8000)
				retval = bulk_latency;
			else if ((packets < 5) && (bytes > 512))
				retval = low_latency;
			break;
		case low_latency:  /* 50 usec aka 20000 ints/s */
			if (bytes > 10000) {
				/* this if handles the TSO accounting */
				if (bytes/packets > 8000)
					retval = bulk_latency;
				else if ((packets < 10) || ((bytes/packets) > 1200))
					retval = bulk_latency;
				else if ((packets > 35))
					retval = lowest_latency;
			} else if (bytes/packets > 2000) {
				retval = bulk_latency;
			} else if (packets <= 2 && bytes < 512) {
				retval = lowest_latency;
			}
			break;
		case bulk_latency: /* 250 usec aka 4000 ints/s */
			if (bytes > 25000) {
				if (packets > 35)
					retval = low_latency;
			} else if (bytes < 6000) {
				retval = low_latency;
			}
			break;
	}
	
	return retval;
}

static void e1000_set_itr(struct e1000_adapter *adapter)
{
	u16 current_itr;
	u32 new_itr = adapter->itr;
	
	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (adapter->link_speed != SPEED_1000) {
		current_itr = 0;
		new_itr = 4000;
		goto set_itr_now;
	}
	
	if (adapter->flags2 & FLAG2_DISABLE_AIM) {
		new_itr = 0;
		goto set_itr_now;
	}

	adapter->tx_itr = e1000_update_itr(adapter->tx_itr,
									   adapter->total_tx_packets,
									   adapter->total_tx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->tx_itr == lowest_latency)
		adapter->tx_itr = low_latency;
	
	adapter->rx_itr = e1000_update_itr(adapter->rx_itr,
									   adapter->total_rx_packets,
									   adapter->total_rx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->rx_itr == lowest_latency)
		adapter->rx_itr = low_latency;
	
	current_itr = max(adapter->rx_itr, adapter->tx_itr);
	
	/* counts and packets in update_itr are dependent on these numbers */
	switch (current_itr) {
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
		 * increasing
		 */
		new_itr = new_itr > adapter->itr ?
		min(adapter->itr + (new_itr >> 2), new_itr) :
		new_itr;
		adapter->itr = new_itr;
		adapter->rx_ring->itr_val = new_itr;
		if (adapter->msix_entries)
			adapter->rx_ring->set_itr = 1;
		else
			e1000e_write_itr(adapter, new_itr);
	}
}

/**
 * e1000e_write_itr - write the ITR value to the appropriate registers
 * @adapter: address of board private structure
 * @itr: new ITR value to program
 *
 * e1000e_write_itr determines if the adapter is in MSI-X mode
 * and, if so, writes the EITR registers with the ITR value.
 * Otherwise, it writes the ITR value into the ITR register.
 * Re-configure OBFF based on the new ITR value.
 **/
void e1000e_write_itr(struct e1000_adapter *adapter, u32 itr)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 new_itr = itr ? 1000000000 / (itr * 256) : 0;
	
	if (adapter->msix_entries) {
		int vector;
		
		for (vector = 0; vector < adapter->num_vectors; vector++)
			writel(new_itr, (u8*)hw->hw_addr + E1000_EITR_82574(vector));
	} else {
		ew32(ITR, new_itr);
	}
	
	if (hw->mac.type == e1000_pch_lpt)
		hw->mac.ops.set_obff_timer(hw, new_itr);
}

static int e1000_tx_map(struct e1000_adapter *adapter, mbuf_t skb, unsigned int first, IOMbufNaturalMemoryCursor * txMbufCursor )
{
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_buffer *buffer_info;
	unsigned int count = 0, i;
	unsigned int f, bytecount, segs;

	unsigned int nr_frags;
	IOPhysicalSegment tx_segments[TBDS_PER_TCB];
	nr_frags = txMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &tx_segments[0], TBDS_PER_TCB);

	
	i = tx_ring->next_to_use;
	
	bytecount = 0;
	for(f = 0; f < nr_frags; f++) {
		buffer_info = &tx_ring->buffer_info[i];
		buffer_info->length = tx_segments[f].length;
		buffer_info->time_stamp = jiffies();
		buffer_info->next_to_watch = i;
		bytecount += tx_segments[f].length;
		buffer_info->dma = tx_segments[f].location;
		buffer_info->mapped_as_page = false;

		count++;
		
		i++;
		if (i == tx_ring->count)
			i = 0;
	}
	
	if (i == 0)
		i = tx_ring->count - 1;
	else
		i--;
	
#ifdef NETIF_F_TSO
	segs = skb_shinfo(skb)->gso_segs ?: 1;
#else
	segs = 1;
#endif
	/* multiply data chunks by size of headers */
	//bytecount = ((segs - 1) * mbuf_pkthdr_len(skb)) +mbuf_len(skb);

	tx_ring->buffer_info[i].skb = skb;
	tx_ring->buffer_info[i].segs = segs;
	tx_ring->buffer_info[i].bytecount = bytecount;
	tx_ring->buffer_info[first].next_to_watch = i;
	
	return count;
	
}


static void e1000_tx_queue(struct e1000_ring *tx_ring, int tx_flags, int count)
{
	struct e1000_adapter *adapter = tx_ring->adapter;
	struct e1000_tx_desc *tx_desc = NULL;
	struct e1000_buffer *buffer_info;
	u32 txd_upper = 0, txd_lower = E1000_TXD_CMD_IFCS;
	unsigned int i;
	
	if (tx_flags & E1000_TX_FLAGS_TSO) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D |
		E1000_TXD_CMD_TSE;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
		
		if (tx_flags & E1000_TX_FLAGS_IPV4)
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
	}
	
	if (tx_flags & E1000_TX_FLAGS_CSUM) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}
	
	if (tx_flags & E1000_TX_FLAGS_VLAN) {
		txd_lower |= E1000_TXD_CMD_VLE;
		txd_upper |= (tx_flags & E1000_TX_FLAGS_VLAN_MASK);
	}
	
	if (unlikely(tx_flags & E1000_TX_FLAGS_NO_FCS))
		txd_lower &= ~(E1000_TXD_CMD_IFCS);
	
#ifdef HAVE_HW_TIME_STAMP
	if (unlikely(tx_flags & E1000_TX_FLAGS_HWTSTAMP)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_EXTCMD_TSTAMP;
	}
#endif
	
	i = tx_ring->next_to_use;
	
	do {
		buffer_info = &tx_ring->buffer_info[i];
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		tx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);
		tx_desc->lower.data =
		cpu_to_le32(txd_lower | buffer_info->length);
		tx_desc->upper.data = cpu_to_le32(txd_upper);
		
		i++;
		if (i == tx_ring->count)
			i = 0;
	} while (--count > 0);
	
	tx_desc->lower.data |= cpu_to_le32(adapter->txd_cmd);
	
	/* txd_cmd re-enables FCS, so we'll re-disable it here as desired. */
	if (unlikely(tx_flags & E1000_TX_FLAGS_NO_FCS))
		tx_desc->lower.data &= ~(cpu_to_le32(E1000_TXD_CMD_IFCS));
	
	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64).
	 */
	wmb();
	
	tx_ring->next_to_use = i;

	if ((adapter->flags2 & FLAG2_NEEDS_OBFF_WORKAROUND) &&
	    !test_bit(__E1000_OBFF_DISABLED, &adapter->state)) {
		struct e1000_hw *hw = &adapter->hw;
		u32 svcr = er32(SVCR);
		
		ew32(SVCR, svcr & ~E1000_SVCR_OFF_EN);
		set_bit(__E1000_OBFF_DISABLED, &adapter->state);
	}
	
	if (adapter->flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
		e1000e_update_tdt_wa(tx_ring, i);
	else
		writel(i, tx_ring->tail);
	
	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems
	 */
	mmiowb();
}

#define MINIMUM_DHCP_PACKET_SIZE 282
static int e1000_transfer_dhcp_info(struct e1000_adapter *adapter,
									struct sk_buff *skb)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 length, offset;
	
#ifdef NETIF_F_HW_VLAN_TX
	if (vlan_tx_tag_present(skb) &&
	    !((vlan_tx_tag_get(skb) == adapter->hw.mng_cookie.vlan_id) &&
	      (adapter->hw.mng_cookie.status &
	       E1000_MNG_DHCP_COOKIE_STATUS_VLAN)))
		return 0;
	
#endif
	if (mbuf_pkthdr_len(skb) <= MINIMUM_DHCP_PACKET_SIZE)
		return 0;
	
	ether_header * ehdr = (ether_header *)mbuf_data(skb);
	if (ehdr->ether_type != htons(ETHERTYPE_IP))
		return 0;
	
	{
		const struct ip *ip = (struct ip *)((u8 *)ehdr + 14);
		struct udphdr *udp;
		
		if (ip->ip_p != IPPROTO_UDP)
			return 0;
		
		udp = (struct udphdr *)((u8 *)ip + (ip->ip_hl << 2));
		if (ntohs(udp->uh_dport) != 67)
			return 0;
		
		offset = (u8 *)udp + 8 - (u8*)ehdr;
		length = mbuf_pkthdr_len(skb) - offset;
		return e1000e_mng_write_dhcp_info(hw, (u8 *)udp + 8, length);
	}
	
	return 0;
}

static void e1000e_check_82574_phy_workaround(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	
	/* With 82574 controllers, PHY needs to be checked periodically
	 * for hung state and reset, if two calls return true
	 */
	if (e1000_check_phy_82574(hw)) 
		adapter->phy_hang_count++;
	else
		adapter->phy_hang_count = 0;
	
	if (adapter->phy_hang_count > 1) {
		adapter->phy_hang_count = 0;
		schedule_work(&adapter->reset_task);
	}
}

static void e1000e_flush_descriptors(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
    
	if (!(adapter->flags2 & FLAG2_DMA_BURST))
		return;
    
	/* flush pending descriptor writebacks to memory */
	ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
	ew32(RDTR, adapter->rx_int_delay | E1000_RDTR_FPD);
    
	/* execute the writes immediately */
	e1e_flush();

	/* due to rare timing issues, write to TIDV/RDTR again to ensure the
	 * write is successful
	 */
	ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
	ew32(RDTR, adapter->rx_int_delay | E1000_RDTR_FPD);
	
	/* execute the writes immediately */
	e1e_flush();
}

static int e1000_init_phy_wakeup(struct e1000_adapter *adapter, u32 wufc)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 i, mac_reg;
	u16 phy_reg, wuc_enable;
	int retval;
	
	/* copy MAC RARs to PHY RARs */
	e1000_copy_rx_addrs_to_phy_ich8lan(hw);
	
	retval = hw->phy.ops.acquire(hw);
	if (retval) {
		e_err("Could not acquire PHY\n");
		return retval;
	}
	
	/* Enable access to wakeup registers on and set page to BM_WUC_PAGE */
	retval = e1000_enable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
	if (retval)
		goto release;
	
	/* copy MAC MTA to PHY MTA - only needed for pchlan */
	for (i = 0; i < adapter->hw.mac.mta_reg_count; i++) {
		mac_reg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
		hw->phy.ops.write_reg_page(hw, BM_MTA(i),
								   (u16)(mac_reg & 0xFFFF));
		hw->phy.ops.write_reg_page(hw, BM_MTA(i) + 1,
								   (u16)((mac_reg >> 16) & 0xFFFF));
	}
	
	/* configure PHY Rx Control register */
	hw->phy.ops.read_reg_page(&adapter->hw, BM_RCTL, &phy_reg);
	mac_reg = er32(RCTL);
	if (mac_reg & E1000_RCTL_UPE)
		phy_reg |= BM_RCTL_UPE;
	if (mac_reg & E1000_RCTL_MPE)
		phy_reg |= BM_RCTL_MPE;
	phy_reg &= ~(BM_RCTL_MO_MASK);
	if (mac_reg & E1000_RCTL_MO_3)
		phy_reg |= (((mac_reg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT)
					<< BM_RCTL_MO_SHIFT);
	if (mac_reg & E1000_RCTL_BAM)
		phy_reg |= BM_RCTL_BAM;
	if (mac_reg & E1000_RCTL_PMCF)
		phy_reg |= BM_RCTL_PMCF;
	mac_reg = er32(CTRL);
	if (mac_reg & E1000_CTRL_RFCE)
		phy_reg |= BM_RCTL_RFCE;
	hw->phy.ops.write_reg_page(&adapter->hw, BM_RCTL, phy_reg);
	
	/* enable PHY wakeup in MAC register */
	ew32(WUFC, wufc);
	ew32(WUC, E1000_WUC_PHY_WAKE | E1000_WUC_PME_EN);
	
	/* configure and enable PHY wakeup in PHY registers */
	hw->phy.ops.write_reg_page(&adapter->hw, BM_WUFC, wufc);
	hw->phy.ops.write_reg_page(&adapter->hw, BM_WUC, E1000_WUC_PME_EN);
	
	/* activate PHY wakeup */
	wuc_enable |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
	retval = e1000_disable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
	if (retval)
		e_err("Could not set PHY Host Wakeup bit\n");
release:
	hw->phy.ops.release(hw);
	
	return retval;
}

static void e1000e_downshift_workaround(struct e1000_adapter *adapter)
{
	if (test_bit(__E1000_DOWN, &adapter->state))
		return;
	
	e1000e_gig_downshift_workaround_ich8lan(&adapter->hw);
}

#ifdef SIOCGMIIPHY
/**
 * e1000_phy_read_status - Update the PHY register status snapshot
 * @adapter: board private structure
 **/
static void e1000_phy_read_status(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_phy_regs *phy = &adapter->phy_regs;
	
	if ((er32(STATUS) & E1000_STATUS_LU) &&
	    (adapter->hw.phy.media_type == e1000_media_type_copper)) {
		int ret_val;
		
		ret_val = e1e_rphy(hw, MII_BMCR, &phy->bmcr);
		ret_val |= e1e_rphy(hw, MII_BMSR, &phy->bmsr);
		ret_val |= e1e_rphy(hw, MII_ADVERTISE, &phy->advertise);
		ret_val |= e1e_rphy(hw, MII_LPA, &phy->lpa);
		ret_val |= e1e_rphy(hw, MII_EXPANSION, &phy->expansion);
		ret_val |= e1e_rphy(hw, MII_CTRL1000, &phy->ctrl1000);
		ret_val |= e1e_rphy(hw, MII_STAT1000, &phy->stat1000);
		ret_val |= e1e_rphy(hw, MII_ESTATUS, &phy->estatus);
		if (ret_val)
			e_warn("Error reading PHY register\n");
	} else {
		/* Do not read PHY registers if link is not up
		 * Set values to typical power-on defaults
		 */
		phy->bmcr = (BMCR_SPEED1000 | BMCR_ANENABLE | BMCR_FULLDPLX);
		phy->bmsr = (BMSR_100FULL | BMSR_100HALF | BMSR_10FULL |
					 BMSR_10HALF | BMSR_ESTATEN | BMSR_ANEGCAPABLE |
					 BMSR_ERCAP);
		phy->advertise = (ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
						  ADVERTISE_ALL | ADVERTISE_CSMA);
		phy->lpa = 0;
		phy->expansion = EXPANSION_ENABLENPAGE;
		phy->ctrl1000 = ADVERTISE_1000FULL;
		phy->stat1000 = 0;
		phy->estatus = (ESTATUS_1000_TFULL | ESTATUS_1000_THALF);
	}
}

#endif /* SIOCGMIIPHY */
static void e1000_print_link_info(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl = er32(CTRL);
	
	/* Link status message must follow this format for user tools */
	/* *INDENT-OFF* */
	e_info("%s NIC Link is Up %d Mbps %s Duplex, Flow Control: %s\n",
			"e1000e", adapter->link_speed,
			adapter->link_duplex == FULL_DUPLEX ? "Full" : "Half",
			(ctrl & E1000_CTRL_TFCE) && (ctrl & E1000_CTRL_RFCE) ? "Rx/Tx" :
			(ctrl & E1000_CTRL_RFCE) ? "Rx" :
			(ctrl & E1000_CTRL_TFCE) ? "Tx" : "None");
	/* *INDENT-ON* */
}

static bool e1000e_has_link(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = false;
	s32 ret_val = 0;
	
	/* get_link_status is set on LSC (link status) interrupt or
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
				link_active = true;
			}
			break;
		case e1000_media_type_fiber:
			ret_val = hw->mac.ops.check_for_link(hw);
			link_active = !!(er32(STATUS) & E1000_STATUS_LU);
			break;
		case e1000_media_type_internal_serdes:
			ret_val = hw->mac.ops.check_for_link(hw);
			link_active = adapter->hw.mac.serdes_has_link;
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
	struct e1000_adapter *adapter = &priv_adapter;
	
	if (super::init(properties) == false) 
		return false;
		
	enabledForNetif = false;
	
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
	
	netdev_features = 0;
	// for ich8lan
	init_mutex();

    adapter->node = -1;
	adapter->pdev = &priv_pdev;
	adapter->netdev = &priv_netdev;
	
	return true;
}

void AppleIntelE1000e::stop(IOService* provider)
{
	e_dbg("AppleIntelE1000e::stop(IOService * provider)\n");
	struct e1000_adapter *adapter = &priv_adapter;

#if 0
	e1000e_ptp_remove(adapter);
#endif

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
#ifdef HAVE_HW_TIME_STAMP
	if (adapter->flags & FLAG_HAS_HW_TIMESTAMP) {
		cancel_work_sync(&adapter->tx_hwtstamp_work);
		if (adapter->tx_hwtstamp_skb) {
			dev_kfree_skb_any(adapter->tx_hwtstamp_skb);
			adapter->tx_hwtstamp_skb = NULL;
		}
	}
#endif

	e1000_power_down_phy(adapter);
#if 1		// WOL
	bool wake;
	__e1000_shutdown( &wake, false);
#endif
	
	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	e1000e_release_hw_control(adapter);
	
	e1000e_reset_interrupt_capability(adapter);
	IOFree(adapter->tx_ring, sizeof(e1000_ring));
	IOFree(adapter->rx_ring, sizeof(e1000_ring));
	
	super::stop(provider);
}

bool AppleIntelE1000e::start(IOService* provider)
{
	e_dbg("AppleIntelE1000e::start(IOService * provider)\n");
    bool success = false;
	int err, i;
	u16 eeprom_data = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;

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
		
		// adapter.hw.device_id will be used later
		adapter->pdev->provider = pciDevice;
		adapter->pdev->device = pciDevice->configRead16(kIOPCIConfigDeviceID);
		IOLog("vendor:device: 0x%x:0x%x.\n", pciDevice->configRead16(kIOPCIConfigVendorID), adapter->pdev->device);
		
		csrPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
		if (csrPCIAddress == NULL) {
			e_dbg("csrPCIAddress.\n");
			break;
		}
		
		// Init PCI config space:
        if ( false == initPCIConfigSpace( pciDevice ) )
            break;
		

		const e1000_info* ei = e1000_probe(adapter->pdev->device);
		if (ei == NULL) {
			break;
		}
        {
            u16 aspm_disable_flag = 0;
            if (ei->flags2 & FLAG2_DISABLE_ASPM_L0S)
                aspm_disable_flag = PCIE_LINK_STATE_L0S;
            if (ei->flags2 & FLAG2_DISABLE_ASPM_L1)
                aspm_disable_flag |= PCIE_LINK_STATE_L1;
            if (aspm_disable_flag)
                e1000e_disable_aspm(pciDevice, aspm_disable_flag);
        }
		
		adapter->ei = ei;
		adapter->pba = ei->pba;
		adapter->flags = ei->flags;
		adapter->flags2 = ei->flags2;
		adapter->hw.adapter = adapter;
		adapter->hw.mac.type = ei->mac;
		adapter->max_hw_frame_size = ei->max_hw_frame_size;
		// adapter->msg_enable = (1 << NETIF_MSG_DRV | NETIF_MSG_PROBE) - 1;
		
		adapter->hw.hw_addr = (u8*)(csrPCIAddress->getVirtualAddress());
		if (adapter->flags & FLAG_HAS_FLASH){
			flashPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
			if (flashPCIAddress == NULL) {
				e_dbg("flashPCIAddress.\n");
				break;
			}
			adapter->hw.flash_address = (u8*)(flashPCIAddress->getVirtualAddress());
		}
		adapter->bd_number = cards_found++;

		e1000e_check_options(adapter);

		adapter->min_frame_size = ETHER_MIN_LEN;
		e1000_change_mtu(ETH_DATA_LEN);	// this call allocates rxMbufCursor/txMbufCursor

		/* setup adapter struct */
		err = e1000_sw_init(adapter);
		if(err)
			break;

		memcpy(&hw->mac.ops, ei->mac_ops, sizeof(hw->mac.ops));
		memcpy(&hw->nvm.ops, ei->nvm_ops, sizeof(hw->nvm.ops));
		memcpy(&hw->phy.ops, ei->phy_ops, sizeof(hw->phy.ops));
		
		if (ei->get_variants) {
			err = ei->get_variants(adapter);
			if (err)
				break;
		}
		
		adapter->hw.mac.ops.get_bus_info(&adapter->hw);
		
		adapter->hw.phy.autoneg_wait_to_complete = 0;	

		/* Copper options */
		if (adapter->hw.phy.media_type == e1000_media_type_copper) {
			adapter->hw.phy.mdix = AUTO_ALL_MODES;
			adapter->hw.phy.disable_polarity_correction = 0;
			adapter->hw.phy.ms_type = e1000_ms_hw_default;
		}
		
		if (hw->phy.ops.check_reset_block && hw->phy.ops.check_reset_block(hw))
			e_info("PHY reset is blocked due to SOL/IDER session.\n");

		if (e1000e_enable_mng_pass_thru(&adapter->hw))
			adapter->flags |= FLAG_MNG_PT_ENABLED;
		
		/* before reading the NVM, reset the controller to
		 * put the device in a known good starting state
		 */
		adapter->hw.mac.ops.reset_hw(&adapter->hw);
		
		/* systems with ASPM and others may see the checksum fail on the first
		 * attempt. Let's give it a few tries
		 */
		for (i = 0;; i++) {
			if (e1000_validate_nvm_checksum(&adapter->hw) >= 0)
				break;
			if (i == 2) {
				e_err("The NVM Checksum Is Not Valid\n");
				err = -EIO;
				break;
			}
		}
		if(err)
			break;

		e1000_eeprom_checks(adapter);

		/* copy the MAC address */
		if (e1000e_read_mac_addr(&adapter->hw))
			e_dbg("NVM Read Error while reading MAC address\n");
		
		
		// Initialize link parameters. User can change them with ethtool
		adapter->hw.mac.autoneg = 1;
		adapter->fc_autoneg = true;
		adapter->hw.fc.requested_mode = e1000_fc_default;
		adapter->hw.fc.current_mode = e1000_fc_default;
		adapter->hw.phy.autoneg_advertised = 0x2f;
		
		/* ring size defaults */
		adapter->rx_ring->count = E1000_DEFAULT_RXD;
		adapter->tx_ring->count = E1000_DEFAULT_TXD;
		
		/* Initial Wake on LAN setting - If APM wake is enabled in
		 * the EEPROM, enable the ACPI Magic Packet filter
		 */
		if (adapter->flags & FLAG_APME_IN_WUC) {
			/* APME bit in EEPROM is mapped to WUC.APME */
			eeprom_data = er32(WUC);
			eeprom_apme_mask = E1000_WUC_APME;
            if ((hw->mac.type > e1000_ich10lan) &&
                (eeprom_data & E1000_WUC_PHY_WAKE))
				adapter->flags2 |= FLAG2_HAS_PHY_WAKEUP;
		} else if (adapter->flags & FLAG_APME_IN_CTRL3) {
			if (adapter->flags & FLAG_APME_CHECK_PORT_B &&
				(adapter->hw.bus.func == 1))
				e1000_read_nvm(&adapter->hw, NVM_INIT_CONTROL3_PORT_B,
							   1, &eeprom_data);
			else
				e1000_read_nvm(&adapter->hw, NVM_INIT_CONTROL3_PORT_A,
							   1, &eeprom_data);
		}
		
		/* fetch WoL from EEPROM */
		if (eeprom_data & eeprom_apme_mask)
			adapter->eeprom_wol |= E1000_WUFC_MAG;
		
		/* now that we have the eeprom settings, apply the special cases
		 * where the eeprom may be wrong or the board simply won't support
		 * wake on lan on a particular port
		 */
		if (!(adapter->flags & FLAG_HAS_WOL))
			adapter->eeprom_wol = 0;
		
		/* initialize the wol settings based on the eeprom settings */
		adapter->wol = adapter->eeprom_wol;
		e_info("AppleIntelE1000e:WOL = %d\n", (int)adapter->wol);
		//device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);
		
		/* save off EEPROM version number */
		e1000_read_nvm(&adapter->hw, 5, 1, &adapter->eeprom_vers);
		
		/* reset the hardware with the new settings */
		e1000e_reset(adapter);

		/* If the controller has AMT, do not set DRV_LOAD until the interface
		 * is up.  For all other cases, let the f/w know that the h/w is now
		 * under the control of the driver.
		 */
		if (!(adapter->flags & FLAG_HAS_AMT))
			e1000e_get_hw_control(adapter);
#if 0
		/* init PTP hardware clock */
		e1000e_ptp_init(adapter);
#endif
		
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

	interruptSource = IOInterruptEventSource::interruptEventSource(this,&AppleIntelE1000e::interruptHandler,provider);

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

	struct e1000_adapter *adapter = &priv_adapter;
	int err;
	
	pciDevice->open(this);

	if (selectMedium(getSelectedMedium()) != kIOReturnSuccess)
		return kIOReturnIOError;

	/* allocate transmit descriptors */
	err = e1000e_setup_tx_resources(adapter->tx_ring);
	if (err)
		goto err_setup_tx;
	
	/* allocate receive descriptors */
	err = e1000e_setup_rx_resources(adapter->rx_ring);
	if (err)
		goto err_setup_rx;
	
	/* If AMT is enabled, let the firmware know that the network
	 * interface is now open
	 */
	if (adapter->flags & FLAG_HAS_AMT){
		e1000e_get_hw_control(adapter);
		e1000e_reset(adapter);
	}
	
	e1000e_power_up_phy(adapter);
	

#ifdef NETIF_F_HW_VLAN_TX
	adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
	if ((adapter->hw.mng_cookie.status &
		 E1000_MNG_DHCP_COOKIE_STATUS_VLAN))
		e1000_update_mng_vlan(adapter);
	
#endif
	/* DMA latency requirement to workaround jumbo issue */
#if 0
	if (adapter->hw.mac.type == e1000_pch2lan)
        ;
#endif
    /* From here on the code is the same as e1000e_up() */
	e1000e_up();

	enabledForNetif = true;
	return kIOReturnSuccess; 

err_setup_rx:
	e1000e_free_tx_resources();
err_setup_tx:
	e1000e_reset(adapter);
	
	return kIOReturnIOError;
}

IOReturn AppleIntelE1000e::disable(IONetworkInterface * netif)
{
	e_dbg("AppleIntelE1000e::disable().\n");

	struct e1000_adapter *adapter = &priv_adapter;

	if(!enabledForNetif)
		return kIOReturnSuccess;
	enabledForNetif = false;

	e1000e_down();	
	
	//e1000_free_irq(adapter);
	e1000_power_down_phy(adapter);
	
	e1000e_free_tx_resources();
	e1000e_free_rx_resources();
	
#ifdef NETIF_F_HW_VLAN_TX
	/* kill manageability vlan ID if supported, but not if a vlan with
	 * the same ID is registered on the host OS (let 8021q kill it)
	 */
	if ((adapter->hw.mng_cookie.status &
		 E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
		!(adapter->vlgrp &&
		  vlan_group_get_device(adapter->vlgrp, adapter->mng_vlan_id)))
		e1000_vlan_rx_kill_vid(netdev, adapter->mng_vlan_id);
	
#endif
	/* If AMT is enabled, let the firmware know that the network
	 * interface is now closed
	 */
	if (adapter->flags & FLAG_HAS_AMT)
		e1000e_release_hw_control(adapter);
#if 0
	if (adapter->hw.mac.type == e1000_pch2lan)
        ;
#endif

	pciDevice->close(this);

	return kIOReturnSuccess;
}

void AppleIntelE1000e::e1000e_up()
{
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	
	/* hardware has been reset, we need to reload some things */
	e1000_configure();
	
	clear_bit(__E1000_DOWN, &adapter->state);
	
	if (adapter->msix_entries)
		e1000_configure_msix(adapter);
	e1000_irq_enable(adapter);

	watchdogSource->setTimeoutMS(200);
	
	//transmitQueue->start();
	workLoop->enableAllInterrupts();
	
	/* fire a link change interrupt to start the watchdog */
	if (adapter->msix_entries)
		ew32(ICS, E1000_ICS_LSC | E1000_ICR_OTHER);
	else
		ew32(ICS, E1000_ICS_LSC);
	
}

void AppleIntelE1000e::e1000e_down()
{
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u32 tctl, rctl;
	
	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer
	 */
	set_bit(__E1000_DOWN, &adapter->state);
	
	/* disable receives in the hardware */
	rctl = er32(RCTL);
	if (!(adapter->flags2 & FLAG2_NO_DISABLE_RX))
		ew32(RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */

	if (transmitQueue) {
		transmitQueue->stop();
		transmitQueue->flush();
	}
	
	/* disable transmits in the hardware */
	tctl = er32(TCTL);
	tctl &= ~E1000_TCTL_EN;
	ew32(TCTL, tctl);

	/* flush both disables and wait for them to finish */
	e1e_flush();
	usleep_range(10000, 20000);
	
#ifdef CONFIG_E1000E_NAPI
	napi_disable(&adapter->napi);
#endif
	e1000_irq_disable(adapter);
	workLoop->disableAllInterrupts();
	
	
	//stop watchdog timer
	watchdogSource->cancelTimeout();		// Stop the timer event source.
	
	setLinkStatus( kIONetworkLinkValid );	// Valid sans kIONetworkLinkActive
	preLinkStatus = 0;
    
	e1000e_flush_descriptors(adapter);
	e1000_clean_tx_ring();
	e1000_clean_rx_ring();
    

	adapter->link_speed = 0;
	adapter->link_duplex = 0;
	
	e1000e_reset(adapter);

	/* TODO: for power management, we could drop the link and
	 * pci_disable_device here.
	 */
}

#if 0
void e1000e_reinit_locked(struct e1000_adapter *adapter)
{
	might_sleep();
	while (test_and_set_bit(__E1000_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	e1000e_down(adapter);
	e1000e_up(adapter);
	clear_bit(__E1000_RESETTING, &adapter->state);
}
#endif

#ifdef HAVE_HW_TIME_STAMP
/**
 * e1000e_cyclecounter_read - read raw cycle counter (used by time counter)
 * @cc: cyclecounter structure
 **/
static cycle_t e1000e_cyclecounter_read(const struct cyclecounter *cc)
{
	struct e1000_adapter *adapter = container_of(cc, struct e1000_adapter,
												 cc);
	struct e1000_hw *hw = &adapter->hw;
	cycle_t systim;
	
	/* latch SYSTIMH on read of SYSTIML */
	systim = (cycle_t)er32(SYSTIML);
	systim |= (cycle_t)er32(SYSTIMH) << 32;
	
	return systim;
}
#endif /* HAVE_HW_TIME_STAMP */

UInt32 AppleIntelE1000e::outputPacket(mbuf_t skb, void * param)
{
	//e_dbg("AppleIntelE1000e::outputPacket()\n");

	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	unsigned int first;
	unsigned int tx_flags = 0;
	int count = 0;
	
	if (enabledForNetif == false) {             // drop the packet.
		e_dbg("not enabledForNetif in outputPacket.\n");
		return kIOReturnOutputDropped;
	}

	if (adapter->hw.mac.tx_pkt_filtering)
		e1000_transfer_dhcp_info(adapter, skb);

	if (e1000_desc_unused(adapter->tx_ring) < TBDS_PER_TCB+2){
		stalled = true;
		return kIOReturnOutputStall;
	}
	
	if (mbuf_pkthdr_len(skb) <= 0) {
		IOLog("skb <=0 in outputPacket.\n");
		return kIOReturnOutputDropped;
	}

	UInt32 vlan;
	if (getVlanTagDemand(skb,&vlan)) {
		tx_flags |= E1000_TX_FLAGS_VLAN;
		tx_flags |= (vlan << E1000_TX_FLAGS_VLAN_SHIFT);
	}
	
    if (e1000_tx_csum(skb)) {
		tx_flags |= E1000_TX_FLAGS_CSUM;
	}

	first = tx_ring->next_to_use;
	count = e1000_tx_map(adapter, skb, first, txMbufCursor);
	
	if (count <= 0) {
		IOLog("failed to getphysicalsegment in outputPacket.\n");
		return kIOReturnOutputDropped;
	}
#ifdef HAVE_HW_TIME_STAMP
#endif

	e1000_tx_queue(tx_ring, tx_flags, count);
	
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
	e1000_adapter *adapter = &priv_adapter;
	
	link = e1000e_setup_copper_link(&adapter->hw);
	
	if (OSDynamicCast(IONetworkMedium, medium) == 0) {
		// Defaults to Auto.
		medium = mediumTable[MEDIUM_INDEX_AUTO];
	}
	
	
	if (link) {
		adapter->hw.mac.ops.get_link_up_info(&adapter->hw,
											&adapter->link_speed,
											&adapter->link_duplex);
		switch(adapter->link_speed) {
			case SPEED_1000:
				medium = mediumTable[MEDIUM_INDEX_1000FD];
				break;
			case SPEED_100:
				if (adapter->link_duplex == FULL_DUPLEX) {
					medium = mediumTable[MEDIUM_INDEX_100FD];
				} else {
					medium = mediumTable[MEDIUM_INDEX_100HD];
				}
				break;
			case SPEED_10:
				if (adapter->link_duplex == FULL_DUPLEX) {
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
	memcpy(addr->bytes, priv_adapter.hw.mac.addr, kIOEthernetAddressSize);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::setHardwareAddress(const IOEthernetAddress * addr)
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	e_info("setHardwareAddress %02x:%02x:%02x:%02x:%02x:%02x.\n",
		addr->bytes[0],addr->bytes[1],addr->bytes[2],
		addr->bytes[3],addr->bytes[4],addr->bytes[5] );
	memcpy(adapter->hw.mac.addr, addr->bytes, kIOEthernetAddressSize);
	
	hw->mac.ops.rar_set(&adapter->hw, adapter->hw.mac.addr, 0);
	
	if (adapter->flags & FLAG_RESET_OVERWRITES_LAA) {
		/* activate the work around */
		e1000e_set_laa_state_82571(&adapter->hw, 1);
		
		/* Hold a copy of the LAA in RAR[14] This is done so that
		 * between the time RAR[0] gets clobbered  and the time it
		 * gets fixed (in e1000_watchdog), the actual LAA is in one
		 * of the RARs and no incoming packets directed to this port
		 * are dropped. Eventually the LAA will be in RAR[0] and
		 * RAR[14]
		 */
		hw->mac.ops.rar_set(&adapter->hw,
					   adapter->hw.mac.addr,
					   adapter->hw.mac.rar_entry_count - 1);
	}
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setPromiscuousMode(bool active)
{
	e_dbg("AppleIntelE1000e::setPromiscuousMode(%d).\n", active);
	promiscusMode = active;
	e1000e_set_rx_mode();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setMulticastMode(bool active)
{
	e_dbg("AppleIntelE1000e::setMulticastMode(%d).\n", active);
	multicastMode = active;
	e1000e_set_rx_mode();
	return kIOReturnSuccess;
}
IOReturn AppleIntelE1000e::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	e1000_adapter *adapter = &priv_adapter;
	mcCount = count;
	e1000e_write_mc_addr_list(&adapter->hw,(u8*)addrs,count);
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily != kChecksumFamilyTCPIP ) {
		IOLog("AppleIntelE1000e: Operating system wants information for unknown checksum family.\n");
		return kIOReturnUnsupported;
	} else {
		UInt32 mask = 0;
		if( !isOutput ) {
			mask |= kChecksumTCP;
#if USE_RX_UDP_CHECKSUM
			mask |= kChecksumUDP;
#endif
#if USE_RX_IP_CHECKSUM
			mask |= kChecksumIP;
#endif
		} else {
			mask |= kChecksumTCP;
#if USE_TX_UDP_CHECKSUM
			mask |= kChecksumUDP;
#endif
#if USE_TX_IP_CHECKSUM
			mask |= kChecksumIP;
#endif
		}
		*checksumMask = mask;
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
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw* hw = &adapter->hw;
	u32 rctl, icr = er32(ICR);
	
	if(!enabledForNetif)
		return;

	if (!icr || test_bit(__E1000_DOWN, &adapter->state))
		return;  /* Not our interrupt */
	
	if (icr & E1000_ICR_LSC) {
		hw->mac.get_link_status = true;
		/* ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers
		 */
		
		if ((adapter->flags & FLAG_LSC_GIG_SPEED_DROP) &&
		    (!(er32(STATUS) & E1000_STATUS_LU)))
			e1000e_downshift_workaround(adapter);

		/* 80003ES2LAN workaround--
		 * For packet buffer work-around on link down event;
		 * disable receives here in the ISR and
		 * reset adapter in watchdog
		 */
		if (adapter->flags & FLAG_RX_NEEDS_RESTART) {
			/* disable receives */
			rctl = er32(RCTL);
			ew32(RCTL, rctl & ~E1000_RCTL_EN);
			adapter->flags |= FLAG_RESTART_NOW;
		}
		/* guard against interrupt when we're going down */
	}
	
	/* Reset on uncorrectable ECC error */
	if ((icr & E1000_ICR_ECCER) && (hw->mac.type == e1000_pch_lpt)) {
		u32 pbeccsts = er32(PBECCSTS);
		
		adapter->corr_errors +=
		pbeccsts & E1000_PBECCSTS_CORR_ERR_CNT_MASK;
		adapter->uncorr_errors +=
		(pbeccsts & E1000_PBECCSTS_UNCORR_ERR_CNT_MASK) >>
		E1000_PBECCSTS_UNCORR_ERR_CNT_SHIFT;
		
		/* Do the reset outside of interrupt context */
		schedule_work(&adapter->reset_task);
		
		/* return immediately since reset is imminent */
		return;
	}

	adapter->total_tx_bytes = 0;
	adapter->total_rx_bytes = 0;
	adapter->total_tx_packets = 0;
	adapter->total_rx_packets = 0;

	for (int i = 0; i < E1000_MAX_INTR; i++) {
		// int rx_cleaned = adapter->clean_rx(adapter);
		int rx_cleaned = this->clean_rx_irq();
		if(rx_cleaned){
			netStats->inputPackets += netif->flushInputQueue();
		}
		int tx_cleaned_complete = e1000_clean_tx_irq();
		if (!rx_cleaned && tx_cleaned_complete)
			break;
	}
	transmitQueue->service();

	if (likely(adapter->itr_setting & 3))
		e1000_set_itr(adapter);
}

void AppleIntelE1000e::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	AppleIntelE1000e * me = (AppleIntelE1000e *) target;
	me->interruptOccurred(src);
}


void AppleIntelE1000e::e1000_init_manageability_pt()
{
	struct e1000_hw *hw = &priv_adapter.hw;
	u32 manc, manc2h, mdef, i, j;
	
	if (!(priv_adapter.flags & FLAG_MNG_PT_ENABLED))
		return;
	
	manc = er32(MANC);
	
	/* enable receiving management packets to the host. this will probably
	 * generate destination unreachable messages from the host OS, but
	 * the packets will be handled on SMBUS
	 */
	manc |= E1000_MANC_EN_MNG2HOST;
	manc2h = er32(MANC2H);
	
	switch (hw->mac.type) {
		default:
			manc2h |= (E1000_MANC2H_PORT_623 | E1000_MANC2H_PORT_664);
			break;
		case e1000_82574:
		case e1000_82583:
			/* Check if IPMI pass-through decision filter already exists;
			 * if so, enable it.
			 */
			for (i = 0, j = 0; i < 8; i++) {
				mdef = er32(MDEF(i));
				
				/* Ignore filters with anything other than IPMI ports */
				if (mdef & ~(E1000_MDEF_PORT_623 | E1000_MDEF_PORT_664))
					continue;
				
				/* Enable this decision filter in MANC2H */
				if (mdef)
					manc2h |= (1 << i);
				
				j |= mdef;
			}
			
			if (j == (E1000_MDEF_PORT_623 | E1000_MDEF_PORT_664))
				break;
			
			/* Create new decision filter in an empty filter */
			for (i = 0, j = 0; i < 8; i++)
				if (er32(MDEF(i)) == 0) {
					ew32(MDEF(i), (E1000_MDEF_PORT_623 |
								   E1000_MDEF_PORT_664));
					manc2h |= (1 << 1);
					j++;
					break;
				}
			
			if (!j)
				e_warn("Unable to create IPMI pass-through filter\n");
			break;
	}
	
	ew32(MANC2H, manc2h);
	ew32(MANC, manc);
}


/**
 * e1000_configure_tx - Configure Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
void AppleIntelE1000e::e1000_configure_tx()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	u64 tdba;
	u32 tdlen, tarc;
	
	/* Setup the HW Tx Head and Tail descriptor pointers */
	tdba = tx_ring->dma;
	tdlen = tx_ring->count * sizeof(struct e1000_tx_desc);
	ew32(TDBAL(0), (tdba & DMA_BIT_MASK(32)));
	ew32(TDBAH(0), (tdba >> 32));
	ew32(TDLEN(0), tdlen);
	ew32(TDH(0), 0);
	ew32(TDT(0), 0);
	tx_ring->head = (u8*)adapter->hw.hw_addr + E1000_TDH(0);
	tx_ring->tail = (u8*)adapter->hw.hw_addr + E1000_TDT(0);
		
	/* Set the Tx Interrupt Delay register */
	ew32(TIDV, adapter->tx_int_delay);
	/* Tx irq moderation */
	ew32(TADV, adapter->tx_abs_int_delay);

	if (adapter->flags2 & FLAG2_DMA_BURST) {
		u32 txdctl = er32(TXDCTL(0));
		txdctl &= ~(E1000_TXDCTL_PTHRESH | E1000_TXDCTL_HTHRESH |
                    E1000_TXDCTL_WTHRESH);
		/* set up some performance related parameters to encourage the
		 * hardware to use the bus more efficiently in bursts, depends
		 * on the tx_int_delay to be enabled,
		 * wthresh = 1 ==> burst write a cacheline (64 bytes) at a time
		 * hthresh = 1 ==> prefetch when one or more available
		 * pthresh = 0x1f ==> prefetch if internal cache 31 or less
		 * BEWARE: this seems to work but should be considered first if
		 * there are Tx hangs or other Tx related bugs
		 */
		txdctl |= E1000_TXDCTL_DMA_BURST_ENABLE;
		ew32(TXDCTL(0), txdctl);
	}
	/* erratum work around: set txdctl the same for both queues */
	ew32(TXDCTL(1), er32(TXDCTL(0)));

	if (adapter->flags & FLAG_TARC_SPEED_MODE_BIT) {
		tarc = er32(TARC(0));
		/* set the speed mode bit, we'll clear it if we're not at
		 * gigabit link later
		 */
#define SPEED_MODE_BIT (1 << 21)
		tarc |= SPEED_MODE_BIT;
		ew32(TARC(0), tarc);
	}
	
	/* errata: program both queues to unweighted RR */
	if (adapter->flags & FLAG_TARC_SET_BIT_ZERO) {
		tarc = er32(TARC(0));
		tarc |= 1;
		ew32(TARC(0), tarc);
		tarc = er32(TARC(1));
		tarc |= 1;
		ew32(TARC(1), tarc);
	}
	
	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter->txd_cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS;
	
	/* only set IDE if we are delaying interrupts using the timers */
	if (adapter->tx_int_delay)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;
	
	/* enable Report Status bit */
	adapter->txd_cmd |= E1000_TXD_CMD_RS;
	
	hw->mac.ops.config_collision_dist(hw);
}

/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
#if	1
// no support
#define PAGE_USE_COUNT(S)	0
#else
#define PAGE_USE_COUNT(S) (((S) >> PAGE_SHIFT) + \
(((S) & (PAGE_SIZE - 1)) ? 1 : 0))
#endif
void AppleIntelE1000e::e1000_setup_rctl()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl, rfctl;
	u32 pages = 0;

	/* Workaround Si errata on PCHx - configure jumbo frame flow */
	if (hw->mac.type >= e1000_pch2lan) {
		s32 ret_val;
        
		if (adapter->netdev->mtu > ETH_DATA_LEN)
			ret_val = e1000_lv_jumbo_workaround_ich8lan(hw, true);
		else
			ret_val = e1000_lv_jumbo_workaround_ich8lan(hw, false);
        
		if (ret_val)
			e_dbg("failed to enable jumbo frame workaround mode\n");
	}

	/* Program MC offset vector base */
	rctl = er32(RCTL);
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
	E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
	(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
	
	/* Do not Store bad packets */
	rctl &= ~E1000_RCTL_SBP;
	
	/* Enable Long Packet receive */
	if (adapter->netdev->mtu <= ETH_DATA_LEN)
		rctl &= ~E1000_RCTL_LPE;
	else
		rctl |= E1000_RCTL_LPE;

	/* Some systems expect that the CRC is included in SMBUS traffic.  The
	 * hardware strips the CRC before sending to both SMBUS (BMC) and to
	 * host memory when this is enabled */
	if (adapter->flags2 & FLAG2_CRC_STRIPPING)
		rctl |= E1000_RCTL_SECRC;
	
	/* Workaround Si errata on 82577/82578 - configure IPG for jumbos */
	if ((hw->mac.type == e1000_pchlan) && (rctl & E1000_RCTL_LPE)) {
		u32 mac_data;
		u16 phy_data;
        
		e1e_rphy(hw, PHY_REG(770, 26), &phy_data);
		phy_data &= 0xfff8;
		phy_data |= (1 << 2);
		e1e_wphy(hw, PHY_REG(770, 26), phy_data);
        
		mac_data = er32(FFLT_DBG);
		mac_data |= (1 << 17);
		ew32(FFLT_DBG, mac_data);
        
		if (hw->phy.type == e1000_phy_82577) {
			e1e_rphy(hw, 22, &phy_data);
			phy_data &= 0x0fff;
			phy_data |= (1 << 14);
			e1e_wphy(hw, 0x10, 0x2823);
			e1e_wphy(hw, 0x11, 0x0003);
			e1e_wphy(hw, 22, phy_data);
		}
	}
	
	
	/* Setup buffer sizes */
	rctl &= ~E1000_RCTL_SZ_4096;
	rctl |= E1000_RCTL_BSEX;
	switch (adapter->rx_buffer_len) {
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

	/* Enable Extended Status in all Receive Descriptors */
	rfctl = er32(RFCTL);
	rfctl |= E1000_RFCTL_EXTEN;
	ew32(RFCTL, rfctl);

	/* 82571 and greater support packet-split where the protocol
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
	pages = PAGE_USE_COUNT(adapter->netdev->mtu);
	if ((pages <= 3) && (PAGE_SIZE <= 16384) && (rctl & E1000_RCTL_LPE))
		adapter->rx_ps_pages = pages;
	else
		adapter->rx_ps_pages = 0;
	
	if (adapter->rx_ps_pages) {
		u32 psrctl = 0;

		/* Enable Packet split descriptors */
		rctl |= E1000_RCTL_DTYP_PS;
		
		psrctl |= adapter->rx_ps_bsize0 >> E1000_PSRCTL_BSIZE0_SHIFT;
		
		switch (adapter->rx_ps_pages) {
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
	
	/* This is useful for sniffing bad packets. */
	if (netdev_features & NETIF_F_RXALL) {
		/* UPE and MPE will be handled by normal PROMISC logic
		 * in e1000e_set_rx_mode */
		rctl |= (E1000_RCTL_SBP |	/* Receive bad packets */
				 E1000_RCTL_BAM |	/* RX All Bcast Pkts */
				 E1000_RCTL_PMCF);	/* RX All MAC Ctrl Pkts */
		
		rctl &= ~(E1000_RCTL_VFE |	/* Disable VLAN filter */
				  E1000_RCTL_DPF |	/* Allow filtered pause */
				  E1000_RCTL_CFIEN);	/* Dis VLAN CFIEN Filter */
		/* Do not mess with E1000_CTRL_VME, it affects transmit as well,
		 * and that breaks VLANs.
		 */
	}
	
	ew32(RCTL, rctl);

	/* just started the receive unit, no need to restart */
	adapter->flags &= ~FLAG_RESTART_NOW;
}

/**
 * e1000_configure_rx - Configure Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void AppleIntelE1000e::e1000_configure_rx()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	u64 rdba;
	u32 rdlen, rctl, rxcsum, ctrl_ext;
	
	if (adapter->rx_ps_pages) {
		/* this is a 32 byte descriptor */
		e_dbg("32 byte descriptor.\n");
		rdlen = rx_ring->count *
			sizeof(union e1000_rx_desc_packet_split);
	} else {
		rdlen = rx_ring->count * sizeof(union e1000_rx_desc_extended);
	}
	
	/* disable receives while setting up the descriptors */
	rctl = er32(RCTL);
	if (!(adapter->flags2 & FLAG2_NO_DISABLE_RX))
		ew32(RCTL, rctl & ~E1000_RCTL_EN);
	e1e_flush();
	usleep_range(10000, 20000);

	if (adapter->flags2 & FLAG2_DMA_BURST) {
		/* set the writeback threshold (only takes effect if the RDTR
		 * is set). set GRAN=1 and write back up to 0x4 worth, and
		 * enable prefetching of 0x20 Rx descriptors
		 * granularity = 01
		 * wthresh = 04,
		 * hthresh = 04,
		 * pthresh = 0x20
		 */
		ew32(RXDCTL(0), E1000_RXDCTL_DMA_BURST_ENABLE);
		ew32(RXDCTL(1), E1000_RXDCTL_DMA_BURST_ENABLE);
        
		/* override the delay timers for enabling bursting, only if
		 * the value was not set by the user via module options
		 */
		if (adapter->rx_int_delay == DEFAULT_RDTR)
			adapter->rx_int_delay = BURST_RDTR;
		if (adapter->rx_abs_int_delay == DEFAULT_RADV)
			adapter->rx_abs_int_delay = BURST_RADV;
	}

	/* set the Receive Delay Timer Register */
	ew32(RDTR, adapter->rx_int_delay);
	
	/* irq moderation */
	ew32(RADV, adapter->rx_abs_int_delay);
	if ((adapter->itr_setting != 0) && (adapter->itr != 0))
		e1000e_write_itr(adapter, adapter->itr);
	
	ctrl_ext = er32(CTRL_EXT);
	ew32(CTRL_EXT, ctrl_ext);
	e1e_flush();
	
	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	rdba = (u64)rx_ring->dma;
	ew32(RDBAL(0), (rdba & DMA_BIT_MASK(32)));
	ew32(RDBAH(0), (rdba >> 32));
	ew32(RDLEN(0), rdlen);
	ew32(RDH(0), 0);
	ew32(RDT(0), 0);
	rx_ring->head = (u8*)adapter->hw.hw_addr + E1000_RDH(0);
	rx_ring->tail = (u8*)adapter->hw.hw_addr + E1000_RDT(0);
	
	/* Enable Receive Checksum Offload for TCP and UDP */
	rxcsum = er32(RXCSUM);
#ifdef HAVE_NDO_SET_FEATURES
	if (adapter->netdev->features & NETIF_F_RXCSUM)
#else
	if (adapter->flags & FLAG_RX_CSUM_ENABLED)
#endif
		rxcsum |= E1000_RXCSUM_TUOFL;
	else
		rxcsum &= ~E1000_RXCSUM_TUOFL;
	ew32(RXCSUM, rxcsum);
	
	/* With jumbo frames, excessive C-state transition
	 * latencies result in dropped transactions.
	 */
	if (adapter->netdev->mtu > ETH_DATA_LEN) {
		if (adapter->flags & FLAG_IS_ICH) {
			u32 rxdctl = er32(RXDCTL(0));
			ew32(RXDCTL(0), rxdctl | 0x3);
		}
#if 0
		u32 lat =
		((er32(PBA) & E1000_PBA_RXA_MASK) * 1024 -
		 adapter->max_frame_size) * 8 / 1000;
		
#ifdef HAVE_PM_QOS_REQUEST_ACTIVE
		pm_qos_update_request(&adapter->netdev->pm_qos_req, lat);
#elif defined(HAVE_PM_QOS_REQUEST_LIST)
		pm_qos_update_request(adapter->netdev->pm_qos_req, lat);
#else
		pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
								  adapter->netdev->name, lat);
#endif
	} else {
#ifdef HAVE_PM_QOS_REQUEST_ACTIVE
		pm_qos_update_request(&adapter->netdev->pm_qos_req,
							  PM_QOS_DEFAULT_VALUE);
#elif defined(HAVE_PM_QOS_REQUEST_LIST)
		pm_qos_update_request(adapter->netdev->pm_qos_req,
							  PM_QOS_DEFAULT_VALUE);
#else
		pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
								  adapter->netdev->name,
								  PM_QOS_DEFAULT_VALUE);
#endif
#endif
		}
	
	/* Enable Receives */
	ew32(RCTL, rctl);
}




/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/
void AppleIntelE1000e::e1000_alloc_rx_buffers(int cleaned_count)
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	union e1000_rx_desc_extended *rx_desc;
	struct e1000_buffer *buffer_info;
	mbuf_t skb;
	unsigned int i;
	unsigned int bufsz = adapter->rx_buffer_len;

	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];
	
	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {	// recycle
			//skb_trim(skb, 0);
		} else {
			skb = allocatePacket( bufsz);
			if (!skb) {
				/* Better luck next round */
				adapter->alloc_rx_buff_failed++;
				break;
			}
			buffer_info->skb = skb;
			buffer_info->dma = 0;
		}
		if(buffer_info->dma == 0){
			UInt32 count;
			struct IOPhysicalSegment vector;
			count = rxMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &vector, 1);
			if (count == 0) {
				IOLog("getPhysicalSegments failed in alloc_rx_buffer.\n");
				adapter->rx_dma_failed++;
				freePacket(skb);
				buffer_info->skb = NULL;
				break;
			}
			buffer_info->dma = vector.location;
		}

		rx_desc = E1000_RX_DESC_EXT(*rx_ring, i);
		rx_desc->read.buffer_addr = cpu_to_le64(buffer_info->dma);

		if (unlikely(!(i & (E1000_RX_BUFFER_WRITE - 1)))) {
			/* Force memory writes to complete before letting h/w
			 * know there are new descriptors to fetch.  (Only
			 * applicable for weak-ordered memory model archs,
			 * such as IA-64).
			 */
			wmb();
			if (adapter->flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
				e1000e_update_rdt_wa(rx_ring, i);
			else
				writel(i, rx_ring->tail);
		}
		
		i++;
		if (i == rx_ring->count)
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}
	
	rx_ring->next_to_use = i;
	
}


/**
 * e1000_alloc_jumbo_rx_buffers - Replace used jumbo receive buffers
 * @adapter: address of board private structure
 * @cleaned_count: number of buffers to allocate this pass
 **/

void AppleIntelE1000e::e1000_alloc_jumbo_rx_buffers( int cleaned_count )
{
	e1000_adapter *adapter = &priv_adapter;
	union e1000_rx_desc_extended *rx_desc;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	struct e1000_buffer *buffer_info;
	struct e1000_ps_page *ps_page;
	unsigned int i;
	
	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];
	
	while (cleaned_count--) {
		ps_page = &buffer_info->ps_pages[0];
		rx_desc = E1000_RX_DESC_EXT(*rx_ring, i);
		if (!ps_page->page) {

			ps_page->pool = IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
																			adapter->rx_buffer_len, PAGE_SIZE );
			if (!ps_page->pool) {
				adapter->alloc_rx_buff_failed++;
				goto no_buffers;
			}
			ps_page->pool->prepare();
			ps_page->page = (page*)ps_page->pool->getBytesNoCopy();
			ps_page->dma = ps_page->pool->getPhysicalAddress();
			/* Refresh the desc even if buffer_addrs
			 * didn't change because each write-back
			 * erases this info.
			 */
			rx_desc->read.buffer_addr = cpu_to_le64(ps_page->dma);
		}

		if (unlikely(!(i & (E1000_RX_BUFFER_WRITE - 1)))) {
			/* Force memory writes to complete before letting h/w
			 * know there are new descriptors to fetch.  (Only
			 * applicable for weak-ordered memory model archs,
			 * such as IA-64).
			 */
			wmb();
			if (adapter->flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
				e1000e_update_rdt_wa(rx_ring, i);
			else
				writel(i, rx_ring->tail);
		}
		
		i++;
		if (i == rx_ring->count)
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}
	
no_buffers:
	rx_ring->next_to_use = i;
}


bool AppleIntelE1000e::e1000_clean_rx_irq()
{
	e1000_adapter *adapter = &priv_adapter;
	//struct e1000_hw* hw = &adapter->hw;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	union e1000_rx_desc_extended *rx_desc, *next_rxd;
	struct e1000_buffer *buffer_info, *next_buffer;
	u32 length, staterr;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = 0;
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	
	
	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC_EXT(*rx_ring, i);
	staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
	buffer_info = &rx_ring->buffer_info[i];
	
	while (staterr & E1000_RXD_STAT_DD) {
		mbuf_t skb;
		
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		//prefetch(skb->data - NET_IP_ALIGN);

		i++;
		if (i == rx_ring->count)
			i = 0;
		next_rxd = E1000_RX_DESC_EXT(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];
		
		cleaned = 1;
		cleaned_count++;
		buffer_info->dma = 0;

		length = le16_to_cpu(rx_desc->wb.upper.length);

		/* !EOP means multiple descriptors were used to store a single
		 * packet, if that's the case we need to toss it.  In fact, we
		 * need to toss every packet with the EOP bit clear and the
		 * next frame that _does_ have the EOP bit set, as it is by
		 * definition only a frame fragment
		 */
		if (unlikely(!(staterr & E1000_RXD_STAT_EOP)))
			adapter->flags2 |= FLAG2_IS_DISCARDING;

		if (adapter->flags2 & FLAG2_IS_DISCARDING) {
			/* All receives must fit into a single buffer */
			e_dbg("Receive packet consumed multiple buffers\n");
			/* recycle */
			buffer_info->skb = skb;
			if (staterr & E1000_RXD_STAT_EOP)
				adapter->flags2 &= ~FLAG2_IS_DISCARDING;
		} else if (unlikely((staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK) &&
						 !(netdev_features & NETIF_F_RXALL))) {
			/* recycle */
			buffer_info->skb = skb;
		} else if(length > 4) {
			/* adjust length to remove Ethernet CRC */
			if (!(adapter->flags2 & FLAG2_CRC_STRIPPING)) {
				/* If configured to store CRC, don't subtract FCS,
				 * but keep the FCS bytes out of the total_rx_bytes
				 * counter
				 */
				if (netdev_features & NETIF_F_RXFCS)
					total_rx_bytes -= 4;
				else
					length -= 4;
			}
			
			total_rx_bytes += length;
			total_rx_packets++;
			
			/* code added for copybreak, this should improve
			 * performance for small packets with large amounts
			 * of reassembly being done in the stack
			 */
			if (length < copybreak) {
				mbuf_t new_skb = copyPacket(skb, length);
				if (new_skb) {
					buffer_info->skb = skb;
					skb = new_skb;
				}
			} else {
				;
			}
			/* end copybreak code */
			
			/* Receive Checksum Offload */
			e1000_rx_checksum(skb, staterr);

			e1000_receive_skb(skb, length, staterr, rx_desc->wb.upper.vlan);
		}
next_desc:		
		rx_desc->wb.upper.status_error &= cpu_to_le32(~0xFF);

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= E1000_RX_BUFFER_WRITE) {
			this->alloc_rx_buf(cleaned_count);
			cleaned_count = 0;
		}
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;

		staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
	}
	rx_ring->next_to_clean = i;
	
	cleaned_count = e1000_desc_unused(rx_ring);
	if (cleaned_count)
		this->alloc_rx_buf(cleaned_count);

	adapter->total_rx_bytes += total_rx_bytes;
	adapter->total_rx_packets += total_rx_packets;

	return cleaned;
}


bool AppleIntelE1000e::e1000_clean_jumbo_rx_irq()
{
	struct e1000_adapter *adapter = &priv_adapter;
	//struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	union e1000_rx_desc_extended *rx_desc, *next_rxd;
	struct e1000_buffer *buffer_info, *next_buffer;
	struct e1000_ps_page *ps_page;
	mbuf_t skb;
	u32 length, staterr;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = false;
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	
	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC_EXT(*rx_ring, i);
	staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
	buffer_info = &rx_ring->buffer_info[i];
	
	while (staterr & E1000_RXD_STAT_DD) {

		i++;
		if (i == rx_ring->count)
			i = 0;
		next_rxd = E1000_RX_DESC_EXT(*rx_ring, i);
		
		next_buffer = &rx_ring->buffer_info[i];
		
		cleaned = true;
		cleaned_count++;
		
		/* see !EOP comment in other rx routine */
		if (!(staterr & E1000_RXD_STAT_EOP))
			adapter->flags2 |= FLAG2_IS_DISCARDING;
		
		if (adapter->flags2 & FLAG2_IS_DISCARDING) {
			e_dbg("Receive packet consumed multiple buffers\n");
			if (staterr & E1000_RXD_STAT_EOP)
				adapter->flags2 &= ~FLAG2_IS_DISCARDING;
			goto next_desc;
		}
		
		if (staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK) {
			goto next_desc;
		}
		
		length = le16_to_cpu(rx_desc->wb.upper.length);

		if (!length) {
			e_dbg("Last part of the packet spanning multiple "
			      "descriptors\n");
			goto next_desc;
		}

		/* strip the ethernet crc, problem is we're using pages now so
		 * this whole operation can get a little cpu intensive
		 */
		if (!(adapter->flags2 & FLAG2_CRC_STRIPPING))
			length -=4;
		
		skb = allocatePacket(length);
		if(skb == NULL){
			goto next_desc;
		}

		/* Good Receive */
		ps_page = &buffer_info->ps_pages[0];
		mbuf_copyback(skb, 0, length, ps_page->page, MBUF_WAITOK);
		
		total_rx_bytes += length;
		total_rx_packets++;
		
		e1000_rx_checksum(skb, staterr);
		
		e1000_receive_skb(skb, length, staterr, rx_desc->wb.upper.vlan);
		
	next_desc:
		rx_desc->wb.upper.status_error &= cpu_to_le32(~0xFF);
		
		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= E1000_RX_BUFFER_WRITE) {
			this->alloc_rx_buf(cleaned_count);
			cleaned_count = 0;
		}
		
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
		
		staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
	}
	rx_ring->next_to_clean = i;
	
	cleaned_count = e1000_desc_unused(rx_ring);
	if (cleaned_count)
		this->alloc_rx_buf(cleaned_count);
	
	adapter->total_rx_bytes += total_rx_bytes;
	adapter->total_rx_packets += total_rx_packets;
#ifdef HAVE_NDO_GET_STATS64
#elif HAVE_NETDEV_STATS_IN_NETDEV
	netdev->stats.rx_bytes += total_rx_bytes;
	netdev->stats.rx_packets += total_rx_packets;
#else
	adapter->net_stats.rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_packets += total_rx_packets;
#endif
	return cleaned;
}

void AppleIntelE1000e::e1000_print_hw_hang()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	unsigned int i = tx_ring->next_to_clean;
	unsigned int eop = tx_ring->buffer_info[i].next_to_watch;
	struct e1000_tx_desc *eop_desc = E1000_TX_DESC(*tx_ring, eop);
	struct e1000_hw *hw = &adapter->hw;
	u16 phy_status, phy_1000t_status, phy_ext_status;
	u16 pci_status;
	
	if (test_bit(__E1000_DOWN, &adapter->state))
		return;
	
	if (!adapter->tx_hang_recheck && (adapter->flags2 & FLAG2_DMA_BURST)) {
		/* May be block on write-back, flush and detect again
		 * flush pending descriptor writebacks to memory
		 */
		ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
		/* execute the writes immediately */
		e1e_flush();
		/* Due to rare timing issues, write to TIDV again to ensure
		 * the write is successful
		 */
		ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
		/* execute the writes immediately */
		e1e_flush();
		adapter->tx_hang_recheck = true;
		return;
	}
	/* Real hang detected */
	adapter->tx_hang_recheck = false;
//	netif_stop_queue(netdev);

	e1e_rphy(hw, MII_BMSR, &phy_status);
	e1e_rphy(hw, MII_STAT1000, &phy_1000t_status);
	e1e_rphy(hw, MII_ESTATUS, &phy_ext_status);
	
	pci_status = pciDevice->configRead16( kIOPCIConfigStatus );
	
	/* detected Hardware unit hang */
	e_err("Detected Hardware Unit Hang:\n"
	      "  TDH                  <%x>\n"
	      "  TDT                  <%x>\n"
	      "  next_to_use          <%x>\n"
	      "  next_to_clean        <%x>\n"
	      "buffer_info[next_to_clean]:\n"
	      "  time_stamp           <%lx>\n"
	      "  next_to_watch        <%x>\n"
	      "  next_to_watch.status <%x>\n"
	      "MAC Status             <%x>\n"
	      "PHY Status             <%x>\n"
	      "PHY 1000BASE-T Status  <%x>\n"
	      "PHY Extended Status    <%x>\n"
	      "PCI Status             <%x>\n",
	      readl(tx_ring->head),
	      readl(tx_ring->tail),
	      tx_ring->next_to_use,
	      tx_ring->next_to_clean,
	      tx_ring->buffer_info[eop].time_stamp,
	      eop,
	      eop_desc->upper.fields.status,
	      er32(STATUS),
	      phy_status, phy_1000t_status, phy_ext_status, pci_status);
#if	0
	/* Suggest workaround for known h/w issue */
	if ((hw->mac.type == e1000_pchlan) && (er32(CTRL) & E1000_CTRL_TFCE))
		e_err("Try turning off Tx pause (flow control) via ethtool\n");
#endif
}

#ifdef HAVE_HW_TIME_STAMP
/**
 * e1000e_tx_hwtstamp_work - check for Tx time stamp
 * @work: pointer to work struct
 *
 * This work function polls the TSYNCTXCTL valid bit to determine when a
 * timestamp has been taken for the current stored skb.  The timestamp must
 * be for this skb because only one such packet is allowed in the queue.
 */
static void e1000e_tx_hwtstamp_work(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work, struct e1000_adapter,
												 tx_hwtstamp_work);
	struct e1000_hw *hw = &adapter->hw;
	
	if (!adapter->tx_hwtstamp_skb)
		return;
	
	if (er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_VALID) {
		struct skb_shared_hwtstamps shhwtstamps;
		u64 txstmp;
		
		txstmp = er32(TXSTMPL);
		txstmp |= (u64)er32(TXSTMPH) << 32;
		
		e1000e_systim_to_hwtstamp(adapter, &shhwtstamps, txstmp);
		
		skb_tstamp_tx(adapter->tx_hwtstamp_skb, &shhwtstamps);
		dev_kfree_skb_any(adapter->tx_hwtstamp_skb);
		adapter->tx_hwtstamp_skb = NULL;
	} else {
		/* reschedule to check later */
		schedule_work(&adapter->tx_hwtstamp_work);
	}
}
#endif /* HAVE_HW_TIME_STAMP */

/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
bool AppleIntelE1000e::e1000_clean_tx_irq()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
	unsigned int count = 0;
	unsigned int total_tx_bytes = 0, total_tx_packets = 0;
	unsigned int bytes_compl = 0, pkts_compl = 0;
	
	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);
	
	while ((eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) &&
	       (count < tx_ring->count)) {
		bool cleaned = false;
		//rmb();
		for (; !cleaned; count++) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);
			
			if (cleaned) {
				total_tx_packets++;
				total_tx_bytes += buffer_info->bytecount;
				if (buffer_info->skb) {
					bytes_compl += mbuf_pkthdr_len(buffer_info->skb);
					pkts_compl++;
				}
				netStats->outputPackets++;
			}
				
			e1000_put_txbuf(buffer_info);
			tx_desc->upper.data = 0;
			
			i++;
			if (i >= adapter->tx_ring->count)
				i = 0;
		}
		
		if (i == tx_ring->next_to_use)
			break;
		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
	}
	
	tx_ring->next_to_clean = i;

#define TX_WAKE_THRESHOLD 32
	if (count &&
	    e1000_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		if (stalled &&
		    !(test_bit(__E1000_DOWN, &adapter->state))) {
			transmitQueue->start();
			++adapter->restart_queue;
		}
	}

	if (adapter->detect_tx_hung) {
		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i
		 */
		adapter->detect_tx_hung = false;
		if (tx_ring->buffer_info[i].time_stamp &&
		    time_after(jiffies(), tx_ring->buffer_info[i].time_stamp
					   + (adapter->tx_timeout_factor * HZ)) &&
			!(er32(STATUS) & E1000_STATUS_TXOFF))
			e1000_print_hw_hang();
		else
			adapter->tx_hang_recheck = false;
	}
	adapter->total_tx_bytes += total_tx_bytes;
	adapter->total_tx_packets += total_tx_packets;
#ifdef HAVE_NETDEV_STATS_IN_NETDEV
	netdev->stats.tx_bytes += total_tx_bytes;
	netdev->stats.tx_packets += total_tx_packets;
#else
	adapter->net_stats.tx_bytes += total_tx_bytes;
	adapter->net_stats.tx_packets += total_tx_packets;
#endif
	return count < tx_ring->count;
}


void AppleIntelE1000e::e1000e_enable_receives()
{
	e1000_adapter *adapter = &priv_adapter;
	/* make sure the receive unit is started */
	if ((adapter->flags & FLAG_RX_NEEDS_RESTART) &&
	    (adapter->flags & FLAG_RESTART_NOW)) {
		struct e1000_hw *hw = &adapter->hw;
		u32 rctl = er32(RCTL);
		ew32(RCTL, rctl | E1000_RCTL_EN);
		adapter->flags &= ~FLAG_RESTART_NOW;
	}
}


void AppleIntelE1000e::e1000_put_txbuf(e1000_buffer *buffer_info)
{
	if (buffer_info->skb) {
		freePacket(buffer_info->skb);
		buffer_info->skb = NULL;
	}
	buffer_info->time_stamp = 0;
}


/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 **/
void AppleIntelE1000e::e1000_clean_tx_ring()
{
	struct e1000_adapter *adapter = &priv_adapter;	
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_buffer *buffer_info;
	unsigned long size;
	unsigned int i;
	
	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_info[i];
		e1000_put_txbuf(buffer_info);
	}
	
#ifdef CONFIG_BQL
	netdev_reset_queue(adapter->netdev);
#endif
	size = sizeof(struct e1000_buffer) * tx_ring->count;
	bzero(tx_ring->buffer_info, size);
	
	bzero(tx_ring->desc, tx_ring->size);
	
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	
	writel(0, tx_ring->head);
	if (tx_ring->adapter->flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
		e1000e_update_tdt_wa(tx_ring, 0);
	else
		writel(0, tx_ring->tail);
}


/**
 * e1000e_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void AppleIntelE1000e::e1000e_free_tx_resources()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	
	e1000_clean_tx_ring();
	
	vm_size_t size = sizeof(struct e1000_buffer) * tx_ring->count;
	IOFree(tx_ring->buffer_info,size);
	tx_ring->buffer_info = NULL;
	
	tx_ring->pool->complete();
	tx_ring->pool->release();

	tx_ring->pool = NULL;
	tx_ring->desc = NULL;
}

/**
 * e1000e_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/

void AppleIntelE1000e::e1000e_free_rx_resources()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	int i;
	
	e1000_clean_rx_ring();
	
	for (i = 0; i < rx_ring->count; i++) {
		IOFree(rx_ring->buffer_info[i].ps_pages, PS_PAGE_BUFFERS * sizeof(struct e1000_ps_page));
	}

	vm_size_t size = sizeof(struct e1000_buffer) * rx_ring->count;
	IOFree(rx_ring->buffer_info, size);
	rx_ring->buffer_info = NULL;

	rx_ring->pool->complete();
	rx_ring->pool->release();
	rx_ring->pool = NULL;

	rx_ring->desc = NULL;
}



/**
 * e1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 **/
void AppleIntelE1000e::e1000_clean_rx_ring()
{
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *rx_ring = adapter->rx_ring;
	struct e1000_buffer *buffer_info;
	struct e1000_ps_page *ps_page;
	unsigned int i, j;
	
	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		if (buffer_info->dma) {
			buffer_info->dma = 0;
		}
		
		if (buffer_info->page) {
			//put_page(buffer_info->page);
			buffer_info->page = NULL;
		}
		
		if (buffer_info->skb) {
			freePacket(buffer_info->skb);
			buffer_info->skb = NULL;
		}
		
		for (j = 0; j < PS_PAGE_BUFFERS; j++) {
			ps_page = &buffer_info->ps_pages[j];
			if (!ps_page->page)
				break;
			ps_page->dma = 0;

			ps_page->pool->complete();
			ps_page->pool->release();
			ps_page->pool = 0;
			//put_page(ps_page->page);
			ps_page->page = NULL;
		}
	}
	
#ifdef CONFIG_E1000E_NAPI
	/* there also may be some cached data from a chained receive */
	if (rx_ring->rx_skb_top) {
		dev_kfree_skb(rx_ring->rx_skb_top);
		rx_ring->rx_skb_top = NULL;
	}
#endif
	
	/* Zero out the descriptor ring */
	bzero(rx_ring->desc, rx_ring->size);
	
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	adapter->flags2 &= ~FLAG2_IS_DISCARDING;
	
	writel(0, rx_ring->head);
	if (rx_ring->adapter->flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
		e1000e_update_rdt_wa(rx_ring, 0);
	else
		writel(0, rx_ring->tail);
}


void AppleIntelE1000e::timeoutOccurred(IOTimerEventSource* src)
{
	if(!enabledForNetif)
		return;

	watchdogSource->setTimeoutMS(200);
	
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_phy_info *phy = &adapter->hw.phy;
	struct e1000_hw *hw = &adapter->hw;
	UInt32 link, tctl;
	
	link = e1000e_has_link(adapter);
	
	//e_dbg("timeout link:%d, preLinkStatus:%d.\n", (int)link, (int)preLinkStatus);
	if (link && link == preLinkStatus) {
		e1000e_enable_receives();
		
		if ((adapter->flags2 & FLAG2_NEEDS_OBFF_WORKAROUND) &&
		    test_bit(__E1000_OBFF_DISABLED, &adapter->state) &&
		    (er32(TDH(0)) == er32(TDT(0)))) {
			u32 svcr = er32(SVCR);
			
			/* Enable OBFF when there is nothing to transmit */
			ew32(SVCR, svcr | E1000_SVCR_OFF_EN);
			clear_bit(__E1000_OBFF_DISABLED, &adapter->state);
		}
		
		goto link_up;
	}
	
#ifdef NETIF_F_HW_VLAN_TX
	if ((e1000e_enable_tx_pkt_filtering(hw)) &&
	    (adapter->mng_vlan_id != adapter->hw.mng_cookie.vlan_id))
		e1000_update_mng_vlan(adapter);
	
#endif
	if (link) {
		mac->ops.get_link_up_info(hw,
								  &adapter->link_speed,
								  &adapter->link_duplex);
		e1000_print_link_info(adapter);
		
		/* check if SmartSpeed worked */
		e1000e_check_downshift(hw);
		if (phy->speed_downgraded)
			netdev_warn(netdev,
					    "Link Speed was downgraded by SmartSpeed\n");
		
		/* On supported PHYs, check for duplex mismatch only
		 * if link has autonegotiated at 10/100 half
		 */
		if ((hw->phy.type == e1000_phy_igp_3 ||
		     hw->phy.type == e1000_phy_bm) &&
		    (hw->mac.autoneg == true) &&
		    (adapter->link_speed == SPEED_10 ||
		     adapter->link_speed == SPEED_100) &&
		    (adapter->link_duplex == HALF_DUPLEX)) {
			UInt16 autoneg_exp;
			
			e1e_rphy(hw, MII_EXPANSION, &autoneg_exp);
			
			if (!(autoneg_exp & EXPANSION_NWAY))
				e_info("Autonegotiated half duplex but"
				       " link partner cannot autoneg. "
				       " Try forcing full duplex if "
				       "link gets many collisions.");
		}

		/* adjust timeout factor according to speed/duplex */
		adapter->tx_timeout_factor = 1;
		switch (adapter->link_speed) {
			case SPEED_10:
				adapter->tx_timeout_factor = 16;
				break;
			case SPEED_100:
				adapter->tx_timeout_factor = 10;
				break;
		}
		
		/* enable transmits in the hardware, need to do this
		 * after setting TARC(0)
		 */
		tctl = er32(TCTL);
		tctl |= E1000_TCTL_EN;
		ew32(TCTL, tctl);

		/* Perform any post-link-up configuration before
		 * reporting link up.
		 */
		if (phy->ops.cfg_on_link_up)
			phy->ops.cfg_on_link_up(hw);

		e1000e_enable_receives();
		//E1000_READ_REG(hw, E1000_STATUS);
		UInt32 speed = 1000 * MBit;
		if(adapter->link_speed == SPEED_100)
			speed = 100 * MBit;
		else if(adapter->link_speed == SPEED_10)
			speed = 10 * MBit;
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), speed);
		transmitQueue->start();
	} else {
		
		if (link != preLinkStatus) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			e_info("Link is Down\n");
			
			/* The link is lost so the controller stops DMA.
			 * If there is queued Tx work that cannot be done
			 * or if on an 8000ES2LAN which requires a Rx packet
			 * buffer work-around on link down event, reset the
			 * controller to flush the Tx/Rx packet buffers.
			 * (Do the reset outside of interrupt context).
			 */
			transmitQueue->stop();
			setLinkStatus(kIONetworkLinkValid, 0);
			
		}
	}
	
link_up:
	// e1000e_update_stats(adapter);
	if (adapter->flags & FLAG_RESTART_NOW) {
		schedule_work(&adapter->reset_task);
		/* return immediately since reset is imminent */
		return;
	}
	// e1000e_update_adaptive(&adapter->hw);

	/* Simple mode for Interrupt Throttle Rate (ITR) */
	if (adapter->itr_setting == 4) {
		/*
		 * Symmetric Tx/Rx gets a reduced ITR=2000;
		 * Total asymmetrical Tx or Rx gets ITR=8000;
		 * everyone else is between 2000-8000.
		 */
		u32 goc = (adapter->gotc + adapter->gorc) / 10000;
		u32 dif = (adapter->gotc > adapter->gorc ?
				   adapter->gotc - adapter->gorc :
				   adapter->gorc - adapter->gotc) / 10000;
		u32 itr = goc > 0 ? (dif * 6000 / goc + 2000) : 8000;
		
		e1000e_write_itr(adapter, itr);
	}
	
	/* Cause software interrupt to ensure Rx ring is cleaned */
	if (adapter->msix_entries)
		ew32(ICS, adapter->rx_ring->ims_val);
	else
		ew32(ICS, E1000_ICS_RXDMT0);

	/* flush pending descriptors to memory before detecting Tx hang */
	e1000e_flush_descriptors(adapter);

	/* Force detection of hung controller every watchdog period */
	adapter->detect_tx_hung = true;

	/* With 82571 controllers, LAA may be overwritten due to controller
	 * reset from the other port. Set the appropriate LAA in RAR[0]
	 */
	if (e1000e_get_laa_state_82571(hw))
		hw->mac.ops.rar_set(hw, adapter->hw.mac.addr, 0);
	
	if (adapter->flags2 & FLAG2_CHECK_PHY_HANG)
		e1000e_check_82574_phy_workaround(adapter);

#ifdef HAVE_HW_TIME_STAMP
	/* Clear valid timestamp stuck in RXSTMPL/H due to a Rx error */
	if (adapter->hwtstamp_config.rx_filter != HWTSTAMP_FILTER_NONE) {
		if ((adapter->flags2 & FLAG2_CHECK_RX_HWTSTAMP) &&
		    (er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_VALID)) {
			er32(RXSTMPH);
			adapter->rx_hwtstamp_cleared++;
		} else {
			adapter->flags2 |= FLAG2_CHECK_RX_HWTSTAMP;
		}
	}
#endif
#if 0
	/* Reset the timer */
	if (!test_bit(__E1000_DOWN, &adapter->state))
		mod_timer(&adapter->watchdog_timer,
				  round_jiffies(jiffies + 2 * HZ));
#endif
	preLinkStatus = link;
}

void AppleIntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)
{
	//e_dbg("void AppleIntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)\n");
	AppleIntelE1000e* me = (AppleIntelE1000e*) target;
	me->timeoutOccurred(src);
	
}


void AppleIntelE1000e::e1000e_set_rx_mode()
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;
	
	/* Check for Promiscuous and All Multicast modes */
	
	rctl = er32(RCTL);

	/* clear the affected bits */
	rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);

	if (promiscusMode) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		rctl &= ~E1000_RCTL_VFE;
#ifdef HAVE_VLAN_RX_REGISTER
		rctl &= ~E1000_RCTL_VFE;
#else
		/* Do not hardware filter VLANs in promisc mode */
		//e1000e_vlan_filter_disable(adapter);
#endif
	} else {
		
		if (multicastMode) {
			rctl |= E1000_RCTL_MPE;
		} else {
			/* Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscuous mode so
			 * that we can at least receive multicast traffic
			 */
		}
#ifdef HAVE_VLAN_RX_REGISTER
		if (adapter->flags & FLAG_HAS_HW_VLAN_FILTER)
			rctl |= E1000_RCTL_VFE;
#else
		//e1000e_vlan_filter_enable(adapter);
#endif
#ifdef HAVE_SET_RX_MODE
		/* Write addresses to available RAR registers, if there is not
		 * sufficient space to store all the addresses then enable
		 * unicast promiscuous mode
		 */
		count = e1000e_write_uc_addr_list(netdev);
		if (count < 0)
			rctl |= E1000_RCTL_UPE;
#endif /* HAVE_SET_RX_MODE */
	}
	
	ew32(RCTL, rctl);
}
// place holder
#ifdef NETIF_F_RXHASH
static void e1000e_setup_rss_hash(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mrqc, rxcsum;
	int i;
	static const u32 rsskey[10] = {
		0xda565a6d, 0xc20e5b25, 0x3d256741, 0xb08fa343, 0xcb2bcad0,
		0xb4307bae, 0xa32dcb77, 0x0cf23080, 0x3bb7426a, 0xfa01acbe
	};
	
	/* Fill out hash function seed */
	for (i = 0; i < 10; i++)
		ew32(RSSRK(i), rsskey[i]);
	
	/* Direct all traffic to queue 0 */
	for (i = 0; i < 32; i++)
		ew32(RETA(i), 0);
	
	/* Disable raw packet checksumming so that RSS hash is placed in
	 * descriptor on writeback.
	 */
	rxcsum = er32(RXCSUM);
	rxcsum |= E1000_RXCSUM_PCSD;
	
	ew32(RXCSUM, rxcsum);
	
	mrqc = (E1000_MRQC_RSS_FIELD_IPV4 |
			E1000_MRQC_RSS_FIELD_IPV4_TCP |
			E1000_MRQC_RSS_FIELD_IPV6 |
			E1000_MRQC_RSS_FIELD_IPV6_TCP |
			E1000_MRQC_RSS_FIELD_IPV6_TCP_EX);
	
	ew32(MRQC, mrqc);
}

#endif /* NETIF_F_RXHASH */
#ifdef HAVE_HW_TIME_STAMP
/**
 * e1000e_get_base_timinca - get default SYSTIM time increment attributes
 * @adapter: board private structure
 * @timinca: pointer to returned time increment attributes
 *
 * Get attributes for incrementing the System Time Register SYSTIML/H at
 * the default base frequency, and set the cyclecounter shift value.
 **/
#ifndef HAVE_PTP_1588_CLOCK
static
#endif
s32 e1000e_get_base_timinca(struct e1000_adapter *adapter, u32 *timinca)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 incvalue, incperiod, shift;
	
	/* Make sure clock is enabled on I217 before checking the frequency */
	if ((hw->mac.type == e1000_pch_lpt) &&
	    !(er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_ENABLED) &&
	    !(er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_ENABLED)) {
		u32 fextnvm7 = er32(FEXTNVM7);
		
		if (!(fextnvm7 & (1 << 0))) {
			ew32(FEXTNVM7, fextnvm7 | (1 << 0));
			e1e_flush();
		}
	}
	
	switch (hw->mac.type) {
		case e1000_pch2lan:
		case e1000_pch_lpt:
			/* On I217, the clock frequency is 25MHz or 96MHz as
			 * indicated by the System Clock Frequency Indication
			 */
			if ((hw->mac.type != e1000_pch_lpt) ||
				(er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_SYSCFI)) {
				/* Stable 96MHz frequency */
				incperiod = INCPERIOD_96MHz;
				incvalue = INCVALUE_96MHz;
				shift = INCVALUE_SHIFT_96MHz;
				adapter->cc.shift = shift + INCPERIOD_SHIFT_96MHz;
				break;
			}
			/* fall-through */
		case e1000_82574:
		case e1000_82583:
			/* Stable 25MHz frequency */
			incperiod = INCPERIOD_25MHz;
			incvalue = INCVALUE_25MHz;
			shift = INCVALUE_SHIFT_25MHz;
			adapter->cc.shift = shift;
			break;
		default:
			return -EINVAL;
	}
	
	*timinca = ((incperiod << E1000_TIMINCA_INCPERIOD_SHIFT) |
				((incvalue << shift) & E1000_TIMINCA_INCVALUE_MASK));
	
	return 0;
}

/**
 * e1000e_config_hwtstamp - configure the hwtstamp registers and enable/disable
 * @adapter: board private structure
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't cause any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware filters.
 * Not all combinations are supported, in particular event type has to be
 * specified. Matching the kind of event packet is not supported, with the
 * exception of "all V2 events regardless of level 2 or 4".
 **/
static int e1000e_config_hwtstamp(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct hwtstamp_config *config = &adapter->hwtstamp_config;
	u32 tsync_tx_ctl = E1000_TSYNCTXCTL_ENABLED;
	u32 tsync_rx_ctl = E1000_TSYNCRXCTL_ENABLED;
#ifdef HAVE_PTP_1588_CLOCK
	u32 rxmtrl = 0;
	u16 rxudp = 0;
	bool is_l4 = false;
	bool is_l2 = false;
#endif
	u32 regval;
	s32 ret_val;
	
	if (!(adapter->flags & FLAG_HAS_HW_TIMESTAMP))
		return -EINVAL;
	
	/* flags reserved for future extensions - must be zero */
	if (config->flags)
		return -EINVAL;
	
	switch (config->tx_type) {
		case HWTSTAMP_TX_OFF:
			tsync_tx_ctl = 0;
			break;
		case HWTSTAMP_TX_ON:
			break;
		default:
			return -ERANGE;
	}
	
	switch (config->rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			tsync_rx_ctl = 0;
			break;
#ifdef HAVE_PTP_1588_CLOCK
		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
			rxmtrl = E1000_RXMTRL_PTP_V1_SYNC_MESSAGE;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
			rxmtrl = E1000_RXMTRL_PTP_V1_DELAY_REQ_MESSAGE;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
			/* Also time stamps V2 L2 Path Delay Request/Response */
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_V2;
			rxmtrl = E1000_RXMTRL_PTP_V2_SYNC_MESSAGE;
			is_l2 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
			/* Also time stamps V2 L2 Path Delay Request/Response. */
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_V2;
			rxmtrl = E1000_RXMTRL_PTP_V2_DELAY_REQ_MESSAGE;
			is_l2 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
			/* Hardware cannot filter just V2 L4 Sync messages;
			 * fall-through to V2 (both L2 and L4) Sync.
			 */
		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			/* Also time stamps V2 Path Delay Request/Response. */
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
			rxmtrl = E1000_RXMTRL_PTP_V2_SYNC_MESSAGE;
			is_l2 = true;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			/* Hardware cannot filter just V2 L4 Delay Request messages;
			 * fall-through to V2 (both L2 and L4) Delay Request.
			 */
		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			/* Also time stamps V2 Path Delay Request/Response. */
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
			rxmtrl = E1000_RXMTRL_PTP_V2_DELAY_REQ_MESSAGE;
			is_l2 = true;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
			/* Hardware cannot filter just V2 L4 or L2 Event messages;
			 * fall-through to all V2 (both L2 and L4) Events.
			 */
		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_EVENT_V2;
			config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			is_l2 = true;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
			/* For V1, the hardware can only filter Sync messages or
			 * Delay Request messages but not both so fall-through to
			 * time stamp all packets.
			 */
#endif /* HAVE_PTP_1588_CLOCK */
		case HWTSTAMP_FILTER_ALL:
#ifdef HAVE_PTP_1588_CLOCK
			is_l2 = true;
			is_l4 = true;
#endif
			tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_ALL;
			config->rx_filter = HWTSTAMP_FILTER_ALL;
			break;
		default:
			return -ERANGE;
	}
	
	/* enable/disable Tx h/w time stamping */
	regval = er32(TSYNCTXCTL);
	regval &= ~E1000_TSYNCTXCTL_ENABLED;
	regval |= tsync_tx_ctl;
	ew32(TSYNCTXCTL, regval);
	if ((er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_ENABLED) !=
	    (regval & E1000_TSYNCTXCTL_ENABLED)) {
		e_err("Timesync Tx Control register not set as expected\n");
		return -EAGAIN;
	}
	
	/* enable/disable Rx h/w time stamping */
	regval = er32(TSYNCRXCTL);
	regval &= ~(E1000_TSYNCRXCTL_ENABLED | E1000_TSYNCRXCTL_TYPE_MASK);
	regval |= tsync_rx_ctl;
	ew32(TSYNCRXCTL, regval);
	if ((er32(TSYNCRXCTL) & (E1000_TSYNCRXCTL_ENABLED |
							 E1000_TSYNCRXCTL_TYPE_MASK)) !=
	    (regval & (E1000_TSYNCRXCTL_ENABLED |
				   E1000_TSYNCRXCTL_TYPE_MASK))) {
		e_err("Timesync Rx Control register not set as expected\n");
		return -EAGAIN;
	}
#ifdef HAVE_PTP_1588_CLOCK
	/* L2: define ethertype filter for time stamped packets */
	if (is_l2)
		rxmtrl |= ETH_P_1588;
	
	/* define which PTP packets get time stamped */
	ew32(RXMTRL, rxmtrl);
	
	/* Filter by destination port */
	if (is_l4) {
		rxudp = PTP_EV_PORT;
		cpu_to_be16s(&rxudp);
	}
	ew32(RXUDP, rxudp);
	
	e1e_flush();
#endif
	
	/* Clear TSYNCRXCTL_VALID & TSYNCTXCTL_VALID bit */
	er32(RXSTMPH);
	er32(TXSTMPH);
	
	/* Get and set the System Time Register SYSTIM base frequency */
	ret_val = e1000e_get_base_timinca(adapter, &regval);
	if (ret_val)
		return ret_val;
	ew32(TIMINCA, regval);
	
	/* reset the ns time counter */
	timecounter_init(&adapter->tc, &adapter->cc,
					 ktime_to_ns(ktime_get_real()));
	
	return 0;
}
#endif /* HAVE_HW_TIME_STAMP */


/**
 * e1000_configure - configure the hardware for Rx and Tx
 * @adapter: private board structure
 **/
void AppleIntelE1000e::e1000_configure()
{
	e1000_adapter *adapter = &priv_adapter;
	adapter->flags &= ~FLAG_MSI_ENABLED;

	setMulticastList(NULL,0);
	e1000e_vlan_strip_enable(adapter);
#ifdef NETIF_F_HW_VLAN_TX
	e1000_restore_vlan(adapter);
#endif
	e1000_init_manageability_pt();

	e1000_configure_tx();
	e1000_setup_rctl();
	e1000_configure_rx();

	this->alloc_rx_buf(e1000_desc_unused(adapter->rx_ring));
}
	
/**
 * e1000_receive_skb - helper function to handle Rx indications
 * @adapter: board private structure
 * @staterr: descriptor extended error and status field as written by hardware
 * @vlan: descriptor vlan field as written by hardware (no le/be conversion)
 * @skb: pointer to sk_buff to be indicated to stack
 **/
void AppleIntelE1000e::e1000_receive_skb(mbuf_t skb, u32 length, u32 staterr, __le16 vlan)
{
	
#ifdef HAVE_HW_TIME_STAMP
	e1000e_rx_hwtstamp(adapter, staterr, skb);
#endif
	
    if(staterr & E1000_RXD_STAT_VP){
        u16 tag = le16_to_cpu(vlan);
        if(tag){
            setVlanTag(skb, tag);
        }
    }
	
	netStats->inputPackets += netif->inputPacket(skb, length, IONetworkInterface::kInputOptionQueuePacket);
}

void AppleIntelE1000e::e1000_rx_checksum(mbuf_t skb, u32 status_err)
{
	struct e1000_adapter *adapter = &priv_adapter;
	u16 status = (u16)status_err;
	u8 errors = (u8)(status_err >> 24);

	/* Rx checksum disabled */
#ifdef HAVE_NDO_SET_FEATURES
	if (!(adapter->netdev->features & NETIF_F_RXCSUM))
#else
        if (!(adapter->flags & FLAG_RX_CSUM_ENABLED))
#endif
            return;

	/* Ignore Checksum bit is set */
	if (status & E1000_RXD_STAT_IXSM)
		return;

	/* TCP/UDP checksum error bit or IP checksum error bit is set */
	if (errors & (E1000_RXD_ERR_TCPE | E1000_RXD_ERR_IPE)) {
		/* let the stack verify checksum errors */
		adapter->hw_csum_err++;
		return;
	}

	/* TCP/UDP Checksum has not been calculated */
	if (!(status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)))
		return;

	UInt32 ckMask = kChecksumTCP;
	UInt32 ckResult = 0;
#if USE_RX_UDP_CHECKSUM
	ckMask |= kChecksumUDP;
	if (status & E1000_RXD_STAT_UDPCS) {
		ckResult |= kChecksumUDP;
	}
#endif
	if (status & E1000_RXD_STAT_TCPCS) {
		/* TCP checksum is good */
		ckResult |= kChecksumTCP;
	}
	setChecksumResult(skb, kChecksumFamilyTCPIP, ckMask, ckResult);
}


bool AppleIntelE1000e::e1000_tx_csum(mbuf_t skb)
{
#if 1
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	u32 txd_cmd = 0;

	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);

	
	UInt32 checksumDemanded;
	getChecksumDemand(skb, kChecksumFamilyTCPIP, &checksumDemanded);
	if(checksumDemanded & kChecksumTCP){
		context_desc->lower_setup.ip_config = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		context_desc->upper_setup.tcp_config = E1000_TXD_POPTS_TXSM << 8;
	} else if(checksumDemanded & kChecksumUDP){
		context_desc->lower_setup.ip_config = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		context_desc->upper_setup.tcp_config = E1000_TXD_POPTS_TXSM << 8;
	} else {
		return false;
	}

	/* If we reach this point, the checksum offload context
	 * needs to be reset.
	 */
	context_desc->lower_setup.ip_fields.ipcss = ETHER_HDR_LEN;
	context_desc->lower_setup.ip_fields.ipcso =
	ETHER_HDR_LEN + offsetof(struct ip, ip_sum);
	context_desc->lower_setup.ip_fields.ipcse =
		cpu_to_le16(ETHER_HDR_LEN + sizeof(struct ip) - 1);
	
	context_desc->upper_setup.tcp_fields.tucss =
	ETHER_HDR_LEN + sizeof(struct ip);
	context_desc->upper_setup.tcp_fields.tucse = cpu_to_le16(0);
	
	if (checksumDemanded & kChecksumTCP) {
		context_desc->upper_setup.tcp_fields.tucso =
		ETHER_HDR_LEN + sizeof(struct ip) +
		offsetof(struct tcphdr, th_sum);
		txd_cmd |= E1000_TXD_CMD_TCP;
	} else {
		context_desc->upper_setup.tcp_fields.tucso =
		ETHER_HDR_LEN + sizeof(struct ip) +
		offsetof(struct udphdr, uh_sum);
	}
	
	context_desc->tcp_seg_setup.data = cpu_to_le32(0);
	context_desc->cmd_and_length = cpu_to_le32(adapter->txd_cmd | E1000_TXD_CMD_DEXT);

	buffer_info->next_to_watch = i;
	
	i++;
	if (i >= tx_ring->count)
		i = 0;
	tx_ring->next_to_use = i;
#else
	struct e1000_adapter *adapter = &priv_adapter;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;

	struct ip *ip = NULL;
	u32 ip_hlen, hdr_len;
	int ehdrlen;
	u32 cmd = E1000_TXD_CMD_DEXT;
	
	UInt32 checksumDemanded;
	getChecksumDemand(skb, kChecksumFamilyTCPIP, &checksumDemanded);
	if((checksumDemanded & (kChecksumIP|kChecksumTCP|kChecksumUDP)) == 0)
		return false;

	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
	
	// from FreeBSD
	ehdrlen = ETHER_HDR_LEN;
	ip = (struct ip *)((u8*)mbuf_data(skb) + ehdrlen);
	ip_hlen = ip->ip_hl << 2;
	hdr_len = ehdrlen + ip_hlen;

	context_desc->lower_setup.ip_config = 0;
#if USE_TX_IP_CHECKSUM
	if(	checksumDemanded & kChecksumIP ){
		/*
		 * Start offset for header checksum calculation.
		 * End offset for header checksum calculation.
		 * Offset of place to put the checksum.
		 */
		context_desc->lower_setup.ip_fields.ipcss = ehdrlen;
		context_desc->lower_setup.ip_fields.ipcse = cpu_to_le16(ehdrlen + ip_hlen);
		context_desc->lower_setup.ip_fields.ipcso = ehdrlen + offsetof(struct ip, ip_sum);
		cmd |= E1000_TXD_CMD_IP;
	}
#endif
	//context_desc->upper_setup.tcp_config = 0;
	if(	checksumDemanded & kChecksumTCP ){
		/*
		 * Start offset for payload checksum calculation.
		 * End offset for payload checksum calculation.
		 * Offset of place to put the checksum.
		 */
		context_desc->upper_setup.tcp_fields.tucss = hdr_len;
		context_desc->upper_setup.tcp_fields.tucso = hdr_len + offsetof(struct tcphdr, th_sum);
		cmd |= E1000_TXD_CMD_TCP;
	}
#if USE_TX_UDP_CHECKSUM
	if(	checksumDemanded & kChecksumUDP ){
		/*
		 * Start offset for header checksum calculation.
		 * End offset for header checksum calculation.
		 * Offset of place to put the checksum.
		 */
		context_desc->upper_setup.tcp_fields.tucss = hdr_len;
		context_desc->upper_setup.tcp_fields.tucso = hdr_len + offsetof(struct udphdr, uh_sum);
	}
#endif

	context_desc->upper_setup.tcp_fields.tucse = 0;
	context_desc->tcp_seg_setup.data = 0;
	context_desc->cmd_and_length = cpu_to_le32(cmd);
	
	buffer_info->next_to_watch = i;
	
	i++;
	if (i >= tx_ring->count)
		i = 0;
	tx_ring->next_to_use = i;
	
#endif
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

	/* acknowledge the completion of our power state change */
    return IOPMAckImplied;
}

IOReturn AppleIntelE1000e::getMaxPacketSize (UInt32 *maxSize) const {
	const e1000_adapter *adapter = &priv_adapter;
	if (maxSize)
		*maxSize = adapter->max_hw_frame_size - (ETH_HLEN + ETH_FCS_LEN);
	
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::getMinPacketSize (UInt32 *minSize) const {
	if(minSize)
		*minSize = ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN;
	
	return kIOReturnSuccess;
}

/**
 * e1000_change_mtu - Change the Maximum Transfer Unit
 **/

void AppleIntelE1000e::e1000_change_mtu(UInt32 new_mtu){
	UInt32 max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;
	e1000_adapter *adapter = &priv_adapter;


	/* Jumbo frame workaround on 82579 and newer requires CRC be stripped */
	if ((adapter->hw.mac.type >= e1000_pch2lan) &&
	    !(adapter->flags2 & FLAG2_CRC_STRIPPING) &&
	    (new_mtu > ETH_DATA_LEN)) {
		e_err("Jumbo Frames not supported on this device when CRC stripping is disabled.\n");
		return;
	}

	adapter->max_frame_size = max_frame;
	e_info("changing MTU from %d to %d\n", (int)priv_netdev.mtu, (int)new_mtu);
	priv_netdev.mtu = new_mtu;
	
	/* NOTE: netdev_alloc_skb reserves 16 bytes, and typically NET_IP_ALIGN
	 * means we reserve 2 more, this pushes us to allocate from the next
	 * larger slab size.
	 * i.e. RXBUFFER_2048 --> size-4096 slab
	 * However with the new *_jumbo_rx* routines, jumbo receives will use
	 * fragmented skbs
	 */
	
	if (max_frame <= 2048)
		adapter->rx_buffer_len = 2048;
	else if (max_frame <= 4096)
		adapter->rx_buffer_len = 4096;
	else if (max_frame <= 8192)
		adapter->rx_buffer_len = 8192;
	else if (max_frame <= 16384)
		adapter->rx_buffer_len = 16384;
	
	/* adjust allocation if LPE protects us, and we aren't using SBP */
	if ((max_frame == ETH_FRAME_LEN + ETH_FCS_LEN) ||
		(max_frame == ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN))
		adapter->rx_buffer_len = ETH_FRAME_LEN + VLAN_HLEN
		+ ETH_FCS_LEN;

	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(adapter->rx_buffer_len, 1);
	txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(adapter->rx_buffer_len, TBDS_PER_TCB);
}

IOReturn AppleIntelE1000e::setMaxPacketSize (UInt32 maxSize){

	if(maxSize != priv_netdev.mtu){

		if(enabledForNetif){
			disable(netif);
			
			e1000_change_mtu(maxSize);
			
			enable(netif);
		} else {
			e1000_change_mtu(maxSize);
			e1000e_reset(&priv_adapter);
		}
	}
	return kIOReturnSuccess;
}

IOReturn AppleIntelE1000e::setWakeOnMagicPacket(bool active)
{
	e1000_adapter *adapter = &priv_adapter;
	e_dbg("AppleIntelE1000e::setWakeOnMagicPacket(%s), WOL support = %s\n",
		  active?"TRUE":"FALSE",
		  adapter->eeprom_wol?"YES":"NO");
	if(active){
       if (adapter->eeprom_wol == 0)
           return kIOReturnUnsupported;   
		adapter->wol = adapter->eeprom_wol;
	} else {
		adapter->wol = 0;
	}
	return kIOReturnSuccess;   
}

IOReturn AppleIntelE1000e::getPacketFilters(const OSSymbol * group, UInt32 * filters) const {
	if(group == gIOEthernetWakeOnLANFilterGroup){
		*filters = kIOEthernetWakeOnMagicPacket;
		return kIOReturnSuccess;   
	}
#if defined(MAC_OS_X_VERSION_10_6)
	if(group == gIOEthernetDisabledWakeOnLANFilterGroup){
		*filters = 0;
		return kIOReturnSuccess;   
	}
#endif
	return super::getPacketFilters(group, filters);
}


int AppleIntelE1000e::__e1000_shutdown(bool *enable_wake, bool runtime)
{
	e1000_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl, ctrl_ext, rctl, status;
	/* Runtime suspend should only enable wakeup for link changes */
	u32 wufc = runtime ? E1000_WUFC_LNKC : adapter->wol;
	int retval = 0;
#if 0	
	netif_device_detach(netdev);
	
	if (netif_running(netdev)) {
		int count = E1000_CHECK_RESET_COUNT;
		
		while (test_bit(__E1000_RESETTING, &adapter->state) && count--)
			usleep_range(10000, 20000);
		
		WARN_ON(test_bit(__E1000_RESETTING, &adapter->state));
		e1000e_down(adapter);
		e1000_free_irq(adapter);
	}
#endif
	e1000e_reset_interrupt_capability(adapter);
#if	0
	retval = pci_save_state(pdev);
	if (retval)
		return retval;
#endif
	status = er32(STATUS);
	if (status & E1000_STATUS_LU)
		wufc &= ~E1000_WUFC_LNKC;
	
	if (wufc) {
		e1000_setup_rctl();
		e1000e_set_rx_mode();
		
		/* turn on all-multi mode if wake on multicast is enabled */
		if (wufc & E1000_WUFC_MC) {
			rctl = er32(RCTL);
			rctl |= E1000_RCTL_MPE;
			ew32(RCTL, rctl);
		}
		
		ctrl = er32(CTRL);
		ctrl |= E1000_CTRL_ADVD3WUC;
		if (!(adapter->flags2 & FLAG2_HAS_PHY_WAKEUP))
			ctrl |= E1000_CTRL_EN_PHY_PWR_MGMT;
		ew32(CTRL, ctrl);
		
		if (adapter->hw.phy.media_type == e1000_media_type_fiber ||
		    adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes) {
			/* keep the laser running in D3 */
			ctrl_ext = er32(CTRL_EXT);
			ctrl_ext |= E1000_CTRL_EXT_SDP3_DATA;
			ew32(CTRL_EXT, ctrl_ext);
		}
		
		if (adapter->flags & FLAG_IS_ICH)
			e1000_suspend_workarounds_ich8lan(&adapter->hw);
		
		/* Allow time for pending master requests to run */
		e1000e_disable_pcie_master(&adapter->hw);
		
		if (adapter->flags2 & FLAG2_HAS_PHY_WAKEUP) {
			/* enable wakeup by the PHY */
			retval = e1000_init_phy_wakeup(adapter, wufc);
			if (retval)
				return retval;
		} else {
			/* enable wakeup by the MAC */
			ew32(WUFC, wufc);
			ew32(WUC, E1000_WUC_PME_EN);
		}
	} else {
		ew32(WUC, 0);
		ew32(WUFC, 0);
	}
	
	*enable_wake = !!wufc;
	
	/* make sure adapter isn't asleep if manageability is enabled */
	if ((adapter->flags & FLAG_MNG_PT_ENABLED) ||
	    (hw->mac.ops.check_mng_mode(hw)))
		*enable_wake = true;
	
	if (adapter->hw.phy.type == e1000_phy_igp_3)
		e1000e_igp3_phy_powerdown_workaround_ich8lan(&adapter->hw);
	
	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	e1000e_release_hw_control(adapter);
	
	//pci_disable_device(pdev);
	
	return 0;
}



bool AppleIntelE1000e::clean_rx_irq()
{
	if(priv_adapter.rx_buffer_len > PAGE_SIZE)
		return e1000_clean_jumbo_rx_irq();

	return e1000_clean_rx_irq();
}

void AppleIntelE1000e::alloc_rx_buf(int cleaned_count)
{
	if(priv_adapter.rx_buffer_len > PAGE_SIZE)
		e1000_alloc_jumbo_rx_buffers(cleaned_count);
	else
		e1000_alloc_rx_buffers(cleaned_count);
}


int AppleIntelE1000e::getIntOption( int def, const char *name )
{
	int val = def;
	OSNumber* numObj = OSDynamicCast( OSNumber, getProperty(name) );
	if ( numObj ){
		val = (int)numObj->unsigned32BitValue();
	}
	return val;
}

UInt32 AppleIntelE1000e::getFeatures() const {
	return kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan;
}


extern "C" {
	int pci_read_config_word(pci_dev *dev, int where, u16 *val)
	{
		
		IOPCIDevice* pDev = (IOPCIDevice*)(dev->provider);
		*val = pDev->configRead16(where);
		return 0;
	}
	u16 pci_find_capability(pci_dev *dev, u16 val)
	{
		IOPCIDevice* pDev = (IOPCIDevice*)(dev->provider);
		u8 pos;
		pDev->findPCICapability(kIOPCIPCIExpressCapability, &pos);
		return (u16)pos;
	}
}


