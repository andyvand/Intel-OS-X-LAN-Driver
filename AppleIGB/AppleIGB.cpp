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
#include "igb.h"
}

#include "AppleIGB.h"



#define	RELEASE(x)	{if(x)x->release();x=NULL;}


static void* kzalloc(size_t size)
{
	void* p = IOMalloc(size);
	if(p){
		bzero(p, size);
	} else {
		IOLog("kzalloc: failed size = %d\n", (int)size );
	}
	return p;
}

static void* kcalloc(size_t num, size_t size)
{
	void* p = IOMalloc(num * size);
	if(p){
		bzero(p, num * size);
	} else {
		IOLog("kcalloc: failed num = %d, size = %d\n", (int)num, (int)size );
	}
	return p;
}

static void kfree(void* p, size_t size)
{
	IOFree(p, size);
}


static void* vzalloc(size_t size)
{
	void* p = IOMallocPageable(size, PAGE_SIZE);
	if(p){
		bzero(p, size);
	} else {
	IOLog("vzalloc: failed size = %d\n", (int)size );
	}
	return p;
}

static void vfree(void* p, size_t size)
{
	IOFreePageable(p, size);
}

static void netif_carrier_off(IOEthernetController* netdev){
	((AppleIGB*)netdev)->setCarrier(false);
}

static void netif_carrier_on(IOEthernetController* netdev){
	((AppleIGB*)netdev)->setCarrier(true);
}


static void netif_tx_start_all_queues(IOEthernetController* netdev){
	((AppleIGB*)netdev)->startTxQueue();
}

static void netif_tx_wake_all_queues(IOEthernetController* netdev){
	((AppleIGB*)netdev)->startTxQueue();
}


static void netif_tx_stop_all_queues(IOEthernetController* netdev){
	((AppleIGB*)netdev)->stopTxQueue();
}

static igb_adapter* netdev_priv(IOEthernetController* netdev)
{
	return ((AppleIGB*)netdev)->adapter();
}

static int netif_running(IOEthernetController* netdev)
{
	return ((AppleIGB*)netdev)->running();
}

static int netif_queue_stopped(IOEthernetController* netdev)
{
	return !netif_running(netdev);
}

static int netif_carrier_ok(IOEthernetController* netdev)
{
	return ((AppleIGB*)netdev)->carrier();
}

static void netif_wake_queue(IOEthernetController* netdev)
{
	netif_tx_wake_all_queues(netdev);
}

static void netif_stop_queue(IOEthernetController* netdev)
{
	netif_tx_stop_all_queues(netdev);
}

static inline bool netif_device_attach(IOEthernetController* netdev)
{
	return ((AppleIGB*)netdev)->netifAttach();
}

static inline void netif_device_detach(IOEthernetController* netdev)
{
	((AppleIGB*)netdev)->netifDetach();
}

static mbuf_t netdev_alloc_skb_ip_align(IOEthernetController* netdev, u16 rx_buffer_len)
{
	return netdev->allocatePacket(rx_buffer_len);
}

static dma_addr_t dma_map_single(IOEthernetController* netdev, mbuf_t skb )
{
	return ((AppleIGB*)netdev)->mapSingle(skb);
}

static __be16 vlan_get_protocol(struct sk_buff *skb)
{
	iphdr* p = (iphdr*)mbuf_pkthdr_header(skb);
	return p->ip_p;
}

#define	jiffies	_jiffies()
static uint32_t _jiffies()
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
#define	HZ	250

static int time_after(uint32_t a, uint32_t b)
{
	if(a > b)
		return 1;
	return 0;
}

#define schedule_work(a)	(*(a))->setTimeoutMS(1)

static int pci_enable_device_mem(IOPCIDevice *dev)
{
	if(dev->setMemoryEnable(true))
		return 0;
	return -EINVAL;
}


#define	skb_record_rx_queue(skb,n)

#define PCI_MSI_FLAGS           2       /* Various flags */
#define PCI_MSI_FLAGS_QMASK    0x0e    /* Maximum queue size available */
static int pci_enable_msi_block(IOPCIDevice *dev )
{
	unsigned int nvec = 1;
	int status, maxvec;
	u16 msgctl;

	u8 pos;

	if (dev->findPCICapability(kIOPCIMSICapability, &pos) == 0)
		return -EINVAL;
	msgctl = dev->configRead16(pos+PCI_MSI_FLAGS);
	maxvec = 1 << ((msgctl & PCI_MSI_FLAGS_QMASK) >> 1);
	if (nvec > maxvec)
		return maxvec;
	
#if 0
	status = pci_msi_check_device(dev, nvec, kIOPCIMSICapability);
	if (status)
		return status;
	
	/* Check whether driver already requested MSI-X irqs */
	if (dev->msix_enabled) {
		return -EINVAL;
	}
#endif
#if 0
	// OS specific chain
	status = msi_capability_init(dev, nvec);
#endif
	return status;
}

void igb_reset(struct igb_adapter *);
static int igb_setup_all_tx_resources(struct igb_adapter *);
static int igb_setup_all_rx_resources(struct igb_adapter *);
static void igb_free_all_tx_resources(struct igb_adapter *);
static void igb_free_all_rx_resources(struct igb_adapter *);
static void igb_setup_mrqc(struct igb_adapter *);
void igb_update_stats(struct igb_adapter *);
//static int igb_probe(IOPCIDevice*, const struct pci_device_id *);
//static void __devexit igb_remove(IOPCIDevice *pdev);
#ifdef HAVE_HW_TIME_STAMP
static void igb_init_hw_timer(struct igb_adapter *adapter);
#endif
static int igb_sw_init(struct igb_adapter *);
static int igb_open(IOEthernetController*);
static int igb_close(IOEthernetController*);
static void igb_configure_tx(struct igb_adapter *);
static void igb_configure_rx(struct igb_adapter *);
static void igb_clean_all_tx_rings(struct igb_adapter *);
static void igb_clean_all_rx_rings(struct igb_adapter *);
static void igb_clean_tx_ring(struct igb_adapter *,struct igb_ring *);
static void igb_set_rx_mode(IOEthernetController*);
static void igb_update_phy_info(unsigned long);
static void igb_watchdog(unsigned long);
static void igb_watchdog_task(struct work_struct *);
static void igb_dma_err_task(struct igb_adapter *adapter,IOTimerEventSource * src);
static void igb_dma_err_timer(unsigned long data);
static netdev_tx_t igb_xmit_frame(struct sk_buff *skb, IOEthernetController*);
static struct net_device_stats *igb_get_stats(IOEthernetController*);
static int igb_change_mtu(IOEthernetController*, int);
void igb_full_sync_mac_table(struct igb_adapter *adapter);
static int igb_set_mac(IOEthernetController*, void *);
static void igb_set_uta(struct igb_adapter *adapter);
static irqreturn_t igb_intr(int irq, void *);
static irqreturn_t igb_intr_msi(int irq, void *);
static irqreturn_t igb_msix_other(int irq, void *);
static irqreturn_t igb_msix_ring(int irq, void *);
#ifdef IGB_DCA
static void igb_update_dca(struct igb_q_vector *);
static void igb_setup_dca(struct igb_adapter *);
#endif /* IGB_DCA */
static int igb_poll(struct igb_q_vector *, int);
static bool igb_clean_tx_irq(struct igb_q_vector *);
static bool igb_clean_rx_irq(struct igb_q_vector *, int);
static int igb_ioctl(IOEthernetController*, struct ifreq *, int cmd);
static void igb_tx_timeout(IOEthernetController*);
static void igb_reset_task(struct work_struct *);
#ifdef HAVE_VLAN_RX_REGISTER
static void igb_vlan_mode(IOEthernetController*, struct vlan_group *);
#endif
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
static int igb_vlan_rx_add_vid(struct net_device *, u16);
static int igb_vlan_rx_kill_vid(struct net_device *, u16);
#else
static void igb_vlan_rx_add_vid(struct net_device *, u16);
static void igb_vlan_rx_kill_vid(struct net_device *, u16);
#endif
static void igb_restore_vlan(struct igb_adapter *);
void igb_rar_set(struct igb_adapter *adapter, u32 index);
static void igb_ping_all_vfs(struct igb_adapter *);
static void igb_msg_task(struct igb_adapter *);
static void igb_vmm_control(struct igb_adapter *);
static int igb_set_vf_mac(struct igb_adapter *, int, unsigned char *);
static void igb_restore_vf_multicasts(struct igb_adapter *adapter);
static void igb_process_mdd_event(struct igb_adapter *);
#ifdef IFLA_VF_MAX
static int igb_ndo_set_vf_mac( IOEthernetController* netdev, int vf, u8 *mac);
static int igb_ndo_set_vf_vlan(IOEthernetController *netdev,
							   int vf, u16 vlan, u8 qos);
static int igb_ndo_set_vf_bw(IOEthernetController *netdev, int vf, int tx_rate);
static int igb_ndo_get_vf_config(IOEthernetController *netdev, int vf,
								 struct ifla_vf_info *ivi);
static void igb_check_vf_rate_limit(struct igb_adapter *);
#endif
static int igb_vf_configure(struct igb_adapter *adapter, int vf);
static int igb_check_vf_assignment(struct igb_adapter *adapter);
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
static int igb_find_enabled_vfs(struct igb_adapter *adapter);
#endif
#ifdef CONFIG_PM
#ifdef HAVE_SYSTEM_SLEEP_PM_OPS
static int igb_suspend(struct device *dev);
static int igb_resume(struct device *dev);
#ifdef CONFIG_PM_RUNTIME
static int igb_runtime_suspend(struct device *dev);
static int igb_runtime_resume(struct device *dev);
static int igb_runtime_idle(struct device *dev);
#endif /* CONFIG_PM_RUNTIME */
static const struct dev_pm_ops igb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(igb_suspend, igb_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(igb_runtime_suspend, igb_runtime_resume,
					   igb_runtime_idle)
#endif /* CONFIG_PM_RUNTIME */
};
#endif /* HAVE_SYSTEM_SLEEP_PM_OPS */
#endif
#ifndef USE_REBOOT_NOTIFIER
static void igb_shutdown(IOPCIDevice*);
#else
static int igb_notify_reboot(struct notifier_block *, unsigned long, void *);
static struct notifier_block igb_notifier_reboot = {
	.notifier_call	= igb_notify_reboot,
	.next		= NULL,
	.priority	= 0
};
#endif
#ifdef IGB_DCA
static int igb_notify_dca(struct notifier_block *, unsigned long, void *);
static struct notifier_block dca_notifier = {
	.notifier_call	= igb_notify_dca,
	.next		= NULL,
	.priority	= 0
};
#endif

#ifdef HAVE_PCI_ERS
static pci_ers_result_t igb_io_error_detected(IOPCIDevice *,
											  pci_channel_state_t);
static pci_ers_result_t igb_io_slot_reset(IOPCIDevice *);
static void igb_io_resume(IOPCIDevice *);

static struct pci_error_handlers igb_err_handler = {
	.error_detected = igb_io_error_detected,
	.slot_reset = igb_io_slot_reset,
	.resume = igb_io_resume,
};
#endif

static void igb_init_fw(struct igb_adapter *adapter);
static void igb_init_dmac(struct igb_adapter *adapter, u32 pba);


static void igb_vfta_set(struct igb_adapter *adapter, u32 vid, bool add)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_host_mng_dhcp_cookie *mng_cookie = &hw->mng_cookie;
	u32 index = (vid >> E1000_VFTA_ENTRY_SHIFT) & E1000_VFTA_ENTRY_MASK;
	u32 mask = 1 << (vid & E1000_VFTA_ENTRY_BIT_SHIFT_MASK);
	u32 vfta;
	
	/*
	 * if this is the management vlan the only option is to add it in so
	 * that the management pass through will continue to work
	 */
	if ((mng_cookie->status & E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
	    (vid == mng_cookie->vlan_id))
		add = TRUE;
	
	vfta = adapter->shadow_vfta[index];
	
	if (add)
		vfta |= mask;
	else
		vfta &= ~mask;
	
	e1000_write_vfta(hw, index, vfta);
	adapter->shadow_vfta[index] = vfta;
}

#ifdef HAVE_HW_TIME_STAMP
/**
 * igb_read_clock - read raw cycle counter (to be used by time counter)
 */
static cycle_t igb_read_clock(const struct cyclecounter *tc)
{
	struct igb_adapter *adapter =
	container_of(tc, struct igb_adapter, cycles);
	struct e1000_hw *hw = &adapter->hw;
	u64 stamp = 0;
	int shift = 0;
	
	/*
	 * The timestamp latches on lowest register read. For the 82580
	 * the lowest register is SYSTIMR instead of SYSTIML.  However we never
	 * adjusted TIMINCA so SYSTIMR will just read as all 0s so ignore it.
	 */
	if (hw->mac.type >= e1000_82580) {
		stamp = E1000_READ_REG(hw, E1000_SYSTIMR) >> 8;
		shift = IGB_82580_TSYNC_SHIFT;
	}
	
	stamp |= (u64)E1000_READ_REG(hw, E1000_SYSTIML) << shift;
	stamp |= (u64)E1000_READ_REG(hw, E1000_SYSTIMH) << (shift + 32);
	return stamp;
}

#endif /* SIOCSHWTSTAMP */



#define Q_IDX_82576(i) (((i & 0x1) << 3) + (i >> 1))
/**
 * igb_cache_ring_register - Descriptor ring to register mapping
 * @adapter: board private structure to initialize
 *
 * Once we know the feature-set enabled for the device, we'll cache
 * the register offset the descriptor ring is assigned to.
 **/
static void igb_cache_ring_register(struct igb_adapter *adapter)
{
	int i = 0, j = 0;
	u32 rbase_offset = adapter->vfs_allocated_count;
	
	switch (adapter->hw.mac.type) {
		case e1000_82576:
			/* The queues are allocated for virtualization such that VF 0
			 * is allocated queues 0 and 8, VF 1 queues 1 and 9, etc.
			 * In order to avoid collision we start at the first free queue
			 * and continue consuming queues in the same sequence
			 */
			if ((adapter->rss_queues > 1) && adapter->vmdq_pools) {
				for (; i < adapter->rss_queues; i++)
					adapter->rx_ring[i]->reg_idx = rbase_offset +
					Q_IDX_82576(i);
			}
		case e1000_82575:
		case e1000_82580:
		case e1000_i350:
		default:
			for (; i < adapter->num_rx_queues; i++)
				adapter->rx_ring[i]->reg_idx = rbase_offset + i;
			for (; j < adapter->num_tx_queues; j++)
				adapter->tx_ring[j]->reg_idx = rbase_offset + j;
			break;
	}
}

static void igb_free_queues(struct igb_adapter *adapter)
{
	int i;
	
	for (i = 0; i < adapter->num_tx_queues; i++) {
		kfree(adapter->tx_ring[i], sizeof(igb_ring));
		adapter->tx_ring[i] = NULL;
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		kfree(adapter->rx_ring[i], sizeof(igb_ring));
		adapter->rx_ring[i] = NULL;
	}
	adapter->num_rx_queues = 0;
	adapter->num_tx_queues = 0;
}

/**
 * igb_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 * We allocate one ring per queue at run-time since we don't know the
 * number of queues at compile-time.
 **/
static int igb_alloc_queues(struct igb_adapter *adapter)
{
	struct igb_ring *ring;
	int i;
	
	for (i = 0; i < adapter->num_tx_queues; i++) {
		ring = (igb_ring*)kzalloc(sizeof(struct igb_ring));
		if (!ring)
			goto err;
		ring->count = adapter->tx_ring_count;
		ring->queue_index = i;
		//ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;
		ring->numa_node = adapter->node;
		/* For 82575, context index must be unique per ring. */
		if (adapter->hw.mac.type == e1000_82575)
			ring->flags |= (1<<IGB_RING_FLAG_TX_CTX_IDX);
		adapter->tx_ring[i] = ring;
	}
	
	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = (igb_ring*)kzalloc(sizeof(struct igb_ring));
		if (!ring)
			goto err;
		ring->count = adapter->rx_ring_count;
		ring->queue_index = i;
		//ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;
		ring->numa_node = adapter->node;
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
		ring->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;
#endif
#ifndef HAVE_NDO_SET_FEATURES
		/* enable rx checksum */
		set_bit(IGB_RING_FLAG_RX_CSUM, &ring->flags);
		
#endif
		/* set flag indicating ring supports SCTP checksum offload */
		if (adapter->hw.mac.type >= e1000_82576)
			set_bit(IGB_RING_FLAG_RX_SCTP_CSUM, &ring->flags);

		/* On i350, loopback VLAN packets have the tag byte-swapped. */
		if (adapter->hw.mac.type == e1000_i350)
			set_bit(IGB_RING_FLAG_RX_LB_VLAN_BSWAP, &ring->flags);
		
		adapter->rx_ring[i] = ring;
	}
	
	igb_cache_ring_register(adapter);
	
	return E1000_SUCCESS;
	
err:
	igb_free_queues(adapter);
	
	return -ENOMEM;
}

static void igb_configure_lli(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 port;
	
	/* LLI should only be enabled for MSI-X or MSI interrupts */
	if (!adapter->msix_entries && !(adapter->flags & IGB_FLAG_HAS_MSI))
		return;
	
	if (adapter->lli_port) {
		/* use filter 0 for port */
		port = htons((u16)adapter->lli_port);
		E1000_WRITE_REG(hw, E1000_IMIR(0),
						(port | E1000_IMIR_PORT_IM_EN));
		E1000_WRITE_REG(hw, E1000_IMIREXT(0),
						(E1000_IMIREXT_SIZE_BP | E1000_IMIREXT_CTRL_BP));
	}
	
	if (adapter->flags & IGB_FLAG_LLI_PUSH) {
		/* use filter 1 for push flag */
		E1000_WRITE_REG(hw, E1000_IMIR(1),
						(E1000_IMIR_PORT_BP | E1000_IMIR_PORT_IM_EN));
		E1000_WRITE_REG(hw, E1000_IMIREXT(1),
						(E1000_IMIREXT_SIZE_BP | E1000_IMIREXT_CTRL_PSH));
	}
	
	if (adapter->lli_size) {
		/* use filter 2 for size */
		E1000_WRITE_REG(hw, E1000_IMIR(2),
						(E1000_IMIR_PORT_BP | E1000_IMIR_PORT_IM_EN));
		E1000_WRITE_REG(hw, E1000_IMIREXT(2),
						(adapter->lli_size | E1000_IMIREXT_CTRL_BP));
	}
	
}

/**
 *  igb_write_ivar - configure ivar for given MSI-X vector
 *  @hw: pointer to the HW structure
 *  @msix_vector: vector number we are allocating to a given ring
 *  @index: row index of IVAR register to write within IVAR table
 *  @offset: column offset of in IVAR, should be multiple of 8
 *
 *  This function is intended to handle the writing of the IVAR register
 *  for adapters 82576 and newer.  The IVAR table consists of 2 columns,
 *  each containing an cause allocation for an Rx and Tx ring, and a
 *  variable number of rows depending on the number of queues supported.
 **/
static void igb_write_ivar(struct e1000_hw *hw, int msix_vector,
						   int index, int offset)
{
	u32 ivar = E1000_READ_REG_ARRAY(hw, E1000_IVAR0, index);
	
	/* clear any bits that are currently set */
	ivar &= ~((u32)0xFF << offset);
	
	/* write vector and valid bit */
	ivar |= (msix_vector | E1000_IVAR_VALID) << offset;
	
	E1000_WRITE_REG_ARRAY(hw, E1000_IVAR0, index, ivar);
}

#define IGB_N0_QUEUE -1
static void igb_assign_vector(struct igb_q_vector *q_vector, int msix_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;
	int rx_queue = IGB_N0_QUEUE;
	int tx_queue = IGB_N0_QUEUE;
	u32 msixbm = 0;
	
	if (q_vector->rx.ring)
		rx_queue = q_vector->rx.ring->reg_idx;
	if (q_vector->tx.ring)
		tx_queue = q_vector->tx.ring->reg_idx;
	
	switch (hw->mac.type) {
		case e1000_82575:
			/* The 82575 assigns vectors using a bitmask, which matches the
			 bitmask for the EICR/EIMS/EIMC registers.  To assign one
			 or more queues to a vector, we write the appropriate bits
			 into the MSIXBM register for that vector. */
			if (rx_queue > IGB_N0_QUEUE)
				msixbm = E1000_EICR_RX_QUEUE0 << rx_queue;
			if (tx_queue > IGB_N0_QUEUE)
				msixbm |= E1000_EICR_TX_QUEUE0 << tx_queue;
			if (!adapter->msix_entries && msix_vector == 0)
				msixbm |= E1000_EIMS_OTHER;
			E1000_WRITE_REG_ARRAY(hw, E1000_MSIXBM(0), msix_vector, msixbm);
			q_vector->eims_value = msixbm;
			break;
		case e1000_82576:
			/*
			 * 82576 uses a table that essentially consists of 2 columns
			 * with 8 rows.  The ordering is column-major so we use the
			 * lower 3 bits as the row index, and the 4th bit as the 
			 * column offset.
			 */
			if (rx_queue > IGB_N0_QUEUE)
				igb_write_ivar(hw, msix_vector,
							   rx_queue & 0x7,
							   (rx_queue & 0x8) << 1);
			if (tx_queue > IGB_N0_QUEUE)
				igb_write_ivar(hw, msix_vector,
							   tx_queue & 0x7,
							   ((tx_queue & 0x8) << 1) + 8);
			q_vector->eims_value = 1 << msix_vector;
			break;
		case e1000_82580:
		case e1000_i350:
			/*
			 * On 82580 and newer adapters the scheme is similar to 82576
			 * however instead of ordering column-major we have things
			 * ordered row-major.  So we traverse the table by using
			 * bit 0 as the column offset, and the remaining bits as the
			 * row index.
			 */
			if (rx_queue > IGB_N0_QUEUE)
				igb_write_ivar(hw, msix_vector,
							   rx_queue >> 1,
							   (rx_queue & 0x1) << 4);
			if (tx_queue > IGB_N0_QUEUE)
				igb_write_ivar(hw, msix_vector,
							   tx_queue >> 1,
							   ((tx_queue & 0x1) << 4) + 8);
			q_vector->eims_value = 1 << msix_vector;
			break;
		default:
			BUG();
			break;
	}
	
	/* add q_vector eims value to global eims_enable_mask */
	adapter->eims_enable_mask |= q_vector->eims_value;
	
	/* configure q_vector to set itr on first interrupt */
	q_vector->set_itr = 1;
}

/**
 * igb_configure_msix - Configure MSI-X hardware
 *
 * igb_configure_msix sets up the hardware to properly
 * generate MSI-X interrupts.
 **/
static void igb_configure_msix(struct igb_adapter *adapter)
{
	u32 tmp;
	int i, vector = 0;
	struct e1000_hw *hw = &adapter->hw;
	
	adapter->eims_enable_mask = 0;
	
	/* set vector for other causes, i.e. link changes */
	switch (hw->mac.type) {
		case e1000_82575:
			tmp = E1000_READ_REG(hw, E1000_CTRL_EXT);
			/* enable MSI-X PBA support*/
			tmp |= E1000_CTRL_EXT_PBA_CLR;
			
			/* Auto-Mask interrupts upon ICR read. */
			tmp |= E1000_CTRL_EXT_EIAME;
			tmp |= E1000_CTRL_EXT_IRCA;
			
			E1000_WRITE_REG(hw, E1000_CTRL_EXT, tmp);
			
			/* enable msix_other interrupt */
			E1000_WRITE_REG_ARRAY(hw, E1000_MSIXBM(0), vector++,
								  E1000_EIMS_OTHER);
			adapter->eims_other = E1000_EIMS_OTHER;
			
			break;
			
		case e1000_82576:
		case e1000_82580:
		case e1000_i350:
			/* Turn on MSI-X capability first, or our settings
			 * won't stick.  And it will take days to debug. */
			E1000_WRITE_REG(hw, E1000_GPIE, E1000_GPIE_MSIX_MODE |
							E1000_GPIE_PBA | E1000_GPIE_EIAME |
							E1000_GPIE_NSICR);
			
			/* enable msix_other interrupt */
			adapter->eims_other = 1 << vector;
			tmp = (vector++ | E1000_IVAR_VALID) << 8;
			
			E1000_WRITE_REG(hw, E1000_IVAR_MISC, tmp);
			break;
		default:
			/* do nothing, since nothing else supports MSI-X */
			break;
	} /* switch (hw->mac.type) */
	
	adapter->eims_enable_mask |= adapter->eims_other;
	
	for (i = 0; i < adapter->num_q_vectors; i++)
		igb_assign_vector(adapter->q_vector[i], vector++);
	
	E1000_WRITE_FLUSH(hw);
}

/**
 * igb_request_msix - Initialize MSI-X interrupts
 *
 * igb_request_msix allocates MSI-X vectors and requests interrupts from the
 * kernel.
 **/
static int igb_request_msix(struct igb_adapter *adapter)
{
#ifdef __APPLE__
	return -1;
#else
	IOEthernetController* netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	int i, err = 0, vector = 0;
	
	err = request_irq(adapter->msix_entries[vector].vector,
	                  &igb_msix_other, 0, netdev->name, adapter);
	if (err)
		goto out;
	vector++;
	
	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct igb_q_vector *q_vector = adapter->q_vector[i];
		
		q_vector->itr_register = hw->hw_addr + E1000_EITR(vector);
		
		if (q_vector->rx.ring && q_vector->tx.ring)
			sprintf(q_vector->name, "%s-TxRx-%u", netdev->name,
			        q_vector->rx.ring->queue_index);
		else if (q_vector->tx.ring)
			sprintf(q_vector->name, "%s-tx-%u", netdev->name,
			        q_vector->tx.ring->queue_index);
		else if (q_vector->rx.ring)
			sprintf(q_vector->name, "%s-rx-%u", netdev->name,
			        q_vector->rx.ring->queue_index);
		else
			sprintf(q_vector->name, "%s-unused", netdev->name);
		
		err = request_irq(adapter->msix_entries[vector].vector,
		                  igb_msix_ring, 0, q_vector->name,
		                  q_vector);
		if (err)
			goto out;
		vector++;
	}
	
	igb_configure_msix(adapter);
	return 0;
out:
	return err;
#endif
}

static void igb_reset_interrupt_capability(struct igb_adapter *adapter)
{
	if (adapter->msix_entries) {
		//pci_disable_msix(adapter->pdev);
		kfree(adapter->msix_entries,sizeof(struct msix_entry));
		adapter->msix_entries = NULL;
	} else if (adapter->flags & IGB_FLAG_HAS_MSI) {
		//pci_disable_msi(adapter->pdev);
	}
}

/**
 * igb_free_q_vectors - Free memory allocated for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * This function frees the memory allocated to the q_vectors.  In addition if
 * NAPI is enabled it will delete any references to the NAPI struct prior
 * to freeing the q_vector.
 **/
static void igb_free_q_vectors(struct igb_adapter *adapter)
{
	int v_idx;
	
	for (v_idx = 0; v_idx < adapter->num_q_vectors; v_idx++) {
		struct igb_q_vector *q_vector = adapter->q_vector[v_idx];
		adapter->q_vector[v_idx] = NULL;
		if (!q_vector)
			continue;
		//netif_napi_del(&q_vector->napi);
		kfree(q_vector,sizeof(igb_q_vector));
	}
	adapter->num_q_vectors = 0;
}

/**
 * igb_clear_interrupt_scheme - reset the device to a state of no interrupts
 *
 * This function resets the device so that it has 0 rx queues, tx queues, and
 * MSI-X interrupts allocated.
 */
static void igb_clear_interrupt_scheme(struct igb_adapter *adapter)
{
	igb_free_queues(adapter);
	igb_free_q_vectors(adapter);
	igb_reset_interrupt_capability(adapter);
}

/**
 * igb_process_mdd_event
 * @adapter - board private structure
 *
 * Identify a malicious VF, disable the VF TX/RX queues and log a message.
 */
static void igb_process_mdd_event(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 lvmmc, vfte, vfre, mdfb;
	u8 vf_queue;
	
	lvmmc = E1000_READ_REG(hw, E1000_LVMMC);
	vf_queue = lvmmc >> 29;
	
	/* VF index cannot be bigger or equal to VFs allocated */
	if (vf_queue >= adapter->vfs_allocated_count)
		return;
#ifndef __APPLE__
	netdev_info(adapter->netdev,
	            "VF %d misbehaved. VF queues are disabled. "
	            "VM misbehavior code is 0x%x\n", vf_queue, lvmmc);
#endif

	/* Disable VFTE and VFRE related bits */
	vfte = E1000_READ_REG(hw, E1000_VFTE);
	vfte &= ~(1 << vf_queue);
	E1000_WRITE_REG(hw, E1000_VFTE, vfte);
	
	vfre = E1000_READ_REG(hw, E1000_VFRE);
	vfre &= ~(1 << vf_queue);
	E1000_WRITE_REG(hw, E1000_VFRE, vfre);
	
	/* Disable MDFB related bit */
	mdfb = E1000_READ_REG(hw, E1000_MDFB);
	mdfb &= ~(1 << vf_queue);
	E1000_WRITE_REG(hw, E1000_MDFB, mdfb);
	
	/* Reset the specific VF */
	E1000_WRITE_REG(hw, E1000_VTCTRL(vf_queue), E1000_VTCTRL_RST);
}

/**
 * igb_disable_mdd
 * @adapter - board private structure
 *
 * Disable MDD behavior in the HW
 **/
static void igb_disable_mdd(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 reg;
	
	if (hw->mac.type != e1000_i350)
		return;
	
	reg = E1000_READ_REG(hw, E1000_DTXCTL);
	reg &= (~E1000_DTXCTL_MDP_EN);
	E1000_WRITE_REG(hw, E1000_DTXCTL, reg);
}

/**
 * igb_enable_mdd
 * @adapter - board private structure
 *
 * Enable the HW to detect malicious driver and sends an interrupt to
 * the driver. 
 * 
 * Only available on i350 device
 **/
static void igb_enable_mdd(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 reg;
	
	if (hw->mac.type != e1000_i350)
		return;
	
	reg = E1000_READ_REG(hw, E1000_DTXCTL);
	reg |= E1000_DTXCTL_MDP_EN;
	E1000_WRITE_REG(hw, E1000_DTXCTL, reg);
}

/**
 * igb_reset_sriov_capability - disable SR-IOV if enabled
 *
 * Attempt to disable single root IO virtualization capabilites present in the
 * kernel.
 **/
static void igb_reset_sriov_capability(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	struct e1000_hw *hw = &adapter->hw;
	
	/* reclaim resources allocated to VFs */
	if (adapter->vf_data) {
		if (!igb_check_vf_assignment(adapter)) {
			/* disable iov and allow time for transactions to clear */
			//pci_disable_sriov(pdev);
			msleep(500);
			//dev_info(pci_dev_to_dev(pdev), "IOV Disabled\n");
		}
		
		/* Disable Malicious Driver Detection */
		igb_disable_mdd(adapter);
		
		/* free vf data storage */
		kfree(adapter->vf_data,sizeof(struct vf_data_storage));
		adapter->vf_data = NULL;
		
		/* switch rings back to PF ownership */
		E1000_WRITE_REG(hw, E1000_IOVCTL, E1000_IOVCTL_REUSE_VFQ);
		E1000_WRITE_FLUSH(hw);
		msleep(100);
		
	}
	
	adapter->vfs_allocated_count = 0;
}

/**
 * igb_set_sriov_capability - setup SR-IOV if supported
 *
 * Attempt to enable single root IO virtualization capabilites present in the
 * kernel.
 **/
static void igb_set_sriov_capability(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	int old_vfs = 0;
#ifndef __APPLE__
	int i;
#endif
	
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
	old_vfs = igb_find_enabled_vfs(adapter);
#endif
	if (old_vfs) {
		IOLog(	"%d pre-allocated VFs found - override "
				 "max_vfs setting of %d\n", old_vfs,
				adapter->vfs_allocated_count);
		adapter->vfs_allocated_count = old_vfs;
 	}
	/* no VFs requested, do nothing */
	if (!adapter->vfs_allocated_count)
		return;
	
	/* allocate vf data storage */
	adapter->vf_data = (vf_data_storage*)kcalloc(adapter->vfs_allocated_count,
	                           sizeof(struct vf_data_storage));
	
	if (adapter->vf_data) {
#ifndef __APPLE__
		if (!old_vfs) {
			if (pci_enable_sriov(pdev,
								 adapter->vfs_allocated_count))
				goto err_out;
		}
		for (i = 0; i < adapter->vfs_allocated_count; i++)
			igb_vf_configure(adapter, i);
		
		/* DMA Coalescing is not supported in IOV mode. */
		if (adapter->hw.mac.type >= e1000_i350)
			adapter->dmac = IGB_DMAC_DISABLE;
		if (adapter->hw.mac.type < e1000_i350)
			adapter->flags |= IGB_FLAG_DETECT_BAD_DMA;
		return;
#endif
		
		kfree(adapter->vf_data, sizeof(struct vf_data_storage));
		adapter->vf_data = NULL;
	}
	
err_out:
	kfree(adapter->vf_data, sizeof(struct vf_data_storage));
	adapter->vf_data = NULL;
	adapter->vfs_allocated_count = 0;
	IOLog("Failed to initialize SR-IOV virtualization\n");
}

/**
 * igb_set_interrupt_capability - set MSI or MSI-X if supported
 *
 * Attempt to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static void igb_set_interrupt_capability(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	//int err;
	int numvecs;
	
	/* Number of supported queues. */
	adapter->num_rx_queues = adapter->rss_queues;
	
	if (adapter->vmdq_pools > 1)
		adapter->num_rx_queues += adapter->vmdq_pools - 1;
	
#ifdef HAVE_TX_MQ
	if (adapter->vmdq_pools)
		adapter->num_tx_queues = adapter->vmdq_pools;
	else
		adapter->num_tx_queues = adapter->num_rx_queues;
#else
	adapter->num_tx_queues = max_t(u32, 1, adapter->vmdq_pools);
#endif
	
	switch (adapter->int_mode) {
		case IGB_INT_MODE_MSIX:
			/* start with one vector for every rx queue */
			numvecs = adapter->num_rx_queues;
			
			/* if tx handler is seperate add 1 for every tx queue */
			if (!(adapter->flags & IGB_FLAG_QUEUE_PAIRS))
				numvecs += adapter->num_tx_queues;
			
			/* store the number of vectors reserved for queues */
			adapter->num_q_vectors = numvecs;
			
			/* add 1 vector for link status interrupts */
			numvecs++;
#ifndef __APPLE__
			adapter->msix_entries = (msix_entry*)kcalloc(numvecs, sizeof(struct msix_entry));
			if (adapter->msix_entries) {
				for (i = 0; i < numvecs; i++)
					adapter->msix_entries[i].entry = i;

				err = pci_enable_msix(pdev,
									  adapter->msix_entries, numvecs);
				if (err == 0)
					break;
			}
			/* MSI-X failed, so fall through and try MSI */
			dev_warn(pci_dev_to_dev(pdev), "Failed to initialize MSI-X interrupts. "
					 "Falling back to MSI interrupts.\n");
#endif
			igb_reset_interrupt_capability(adapter);
		case IGB_INT_MODE_MSI:
#ifndef __APPLE__
			if (!pci_enable_msi(pdev))
				adapter->flags |= IGB_FLAG_HAS_MSI;
			else
				dev_warn(pci_dev_to_dev(pdev), "Failed to initialize MSI "
						 "interrupts.  Falling back to legacy "
						 "interrupts.\n");
#endif
			/* Fall through */
		case IGB_INT_MODE_LEGACY:
			/* disable advanced features and set number of queues to 1 */
			igb_reset_sriov_capability(adapter);
			adapter->vmdq_pools = 0;
			adapter->rss_queues = 1;
			adapter->flags |= IGB_FLAG_QUEUE_PAIRS;
			adapter->num_rx_queues = 1;
			adapter->num_tx_queues = 1;
			adapter->num_q_vectors = 1;
			/* Don't do anything; this is system default */
			break;
	}
	
#ifdef HAVE_TX_MQ
	/* Notify the stack of the (possibly) reduced Tx Queue count. */
#ifdef CONFIG_NETDEVICES_MULTIQUEUE
	adapter->netdev->egress_subqueue_count = adapter->num_tx_queues;
#else
	adapter->netdev->real_num_tx_queues =
	(adapter->vmdq_pools ? 1 : adapter->num_tx_queues);
#endif /* CONFIG_NETDEVICES_MULTIQUEUE */
#endif /* HAVE_TX_MQ */
}

/**
 * igb_alloc_q_vectors - Allocate memory for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * We allocate one q_vector per queue interrupt.  If allocation fails we
 * return -ENOMEM.
 **/
static int igb_alloc_q_vectors(struct igb_adapter *adapter)
{
	struct igb_q_vector *q_vector;
	struct e1000_hw *hw = &adapter->hw;
	int v_idx;
	
	for (v_idx = 0; v_idx < adapter->num_q_vectors; v_idx++) {
		q_vector = (igb_q_vector*)kzalloc(sizeof(struct igb_q_vector));
		if (!q_vector)
			goto err_out;

		q_vector->adapter = adapter;
		q_vector->itr_register = hw->hw_addr + E1000_EITR(0);
		q_vector->itr_val = IGB_START_ITR;
		//netif_napi_add(adapter->netdev, &q_vector->napi, igb_poll, 64);
		adapter->q_vector[v_idx] = q_vector;
	}
	
	return 0;
	
err_out:
	igb_free_q_vectors(adapter);
	return -ENOMEM;
}

static void igb_map_rx_ring_to_vector(struct igb_adapter *adapter,
                                      int ring_idx, int v_idx)
{
	struct igb_q_vector *q_vector = adapter->q_vector[v_idx];
	
	q_vector->rx.ring = adapter->rx_ring[ring_idx];
	q_vector->rx.ring->q_vector = q_vector;
	q_vector->rx.count++;
	q_vector->itr_val = adapter->rx_itr_setting;
	if (q_vector->itr_val && q_vector->itr_val <= 3)
		q_vector->itr_val = IGB_START_ITR;
}

static void igb_map_tx_ring_to_vector(struct igb_adapter *adapter,
                                      int ring_idx, int v_idx)
{
	struct igb_q_vector *q_vector = adapter->q_vector[v_idx];
	
	q_vector->tx.ring = adapter->tx_ring[ring_idx];
	q_vector->tx.ring->q_vector = q_vector;
	q_vector->tx.count++;
	q_vector->itr_val = adapter->tx_itr_setting;
	q_vector->tx.work_limit = adapter->tx_work_limit;
	if (q_vector->itr_val && q_vector->itr_val <= 3)
		q_vector->itr_val = IGB_START_ITR;
}

/**
 * igb_map_ring_to_vector - maps allocated queues to vectors
 *
 * This function maps the recently allocated queues to vectors.
 **/
static int igb_map_ring_to_vector(struct igb_adapter *adapter)
{
	int i;
	int v_idx = 0;
	
	if ((adapter->num_q_vectors < adapter->num_rx_queues) ||
	    (adapter->num_q_vectors < adapter->num_tx_queues))
		return -ENOMEM;
	
	if (adapter->num_q_vectors >=
	    (adapter->num_rx_queues + adapter->num_tx_queues)) {
		for (i = 0; i < adapter->num_rx_queues; i++)
			igb_map_rx_ring_to_vector(adapter, i, v_idx++);
		for (i = 0; i < adapter->num_tx_queues; i++)
			igb_map_tx_ring_to_vector(adapter, i, v_idx++);
	} else {
		for (i = 0; i < adapter->num_rx_queues; i++) {
			if (i < adapter->num_tx_queues)
				igb_map_tx_ring_to_vector(adapter, i, v_idx);
			igb_map_rx_ring_to_vector(adapter, i, v_idx++);
		}
		for (; i < adapter->num_tx_queues; i++)
			igb_map_tx_ring_to_vector(adapter, i, v_idx++);
	}
	return 0;
}

/**
 * igb_init_interrupt_scheme - initialize interrupts, allocate queues/vectors
 *
 * This function initializes the interrupts and allocates all of the queues.
 **/
static int igb_init_interrupt_scheme(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	int err;
	
	igb_set_interrupt_capability(adapter);
	
	err = igb_alloc_q_vectors(adapter);
	if (err) {
		IOLog( "Unable to allocate memory for vectors\n");
		goto err_alloc_q_vectors;
	}
	
	err = igb_alloc_queues(adapter);
	if (err) {
		IOLog( "Unable to allocate memory for queues\n");
		goto err_alloc_queues;
	}
	
	err = igb_map_ring_to_vector(adapter);
	if (err) {
		IOLog( "Invalid q_vector to ring mapping\n");
		goto err_map_queues;
	}
	
	
	return 0;
err_map_queues:
	igb_free_queues(adapter);
err_alloc_queues:
	igb_free_q_vectors(adapter);
err_alloc_q_vectors:
	igb_reset_interrupt_capability(adapter);
	return err;
}

/**
 * igb_request_irq - initialize interrupts
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static int igb_request_irq(struct igb_adapter *adapter)
{
	int err = 0;
	
	if (adapter->msix_entries) {
		err = igb_request_msix(adapter);
		if (!err)
			goto request_done;
		/* fall back to MSI */
		igb_clear_interrupt_scheme(adapter);
		igb_reset_sriov_capability(adapter);
#ifndef __APPLE__
		if (!pci_enable_msi(pdev))
			adapter->flags |= IGB_FLAG_HAS_MSI;
#endif
		igb_free_all_tx_resources(adapter);
		igb_free_all_rx_resources(adapter);
		adapter->num_tx_queues = 1;
		adapter->num_rx_queues = 1;
		adapter->num_q_vectors = 1;
		err = igb_alloc_q_vectors(adapter);
		if (err) {
			IOLog( "Unable to allocate memory for vectors\n");
			goto request_done;
		}
		err = igb_alloc_queues(adapter);
		if (err) {
			IOLog( "Unable to allocate memory for queues\n");
			igb_free_q_vectors(adapter);
			goto request_done;
		}
		igb_setup_all_tx_resources(adapter);
		igb_setup_all_rx_resources(adapter);
	}
	
	igb_assign_vector(adapter->q_vector[0], 0);
	
#ifndef __APPLE__
	if (adapter->flags & IGB_FLAG_HAS_MSI) {
		err = request_irq(pdev->irq, &igb_intr_msi, 0,
						  netdev->name, adapter);
		if (!err)
			goto request_done;
		
		/* fall back to legacy interrupts */
		igb_reset_interrupt_capability(adapter);
		adapter->flags &= ~IGB_FLAG_HAS_MSI;
	}
	err = request_irq(pdev->irq, &igb_intr, IRQF_SHARED,
					  netdev->name, adapter);
#endif	
	
	if (err)
		IOLog("Error %d getting interrupt\n",
				err);
	
request_done:
	return err;
}

static void igb_free_irq(struct igb_adapter *adapter)
{
#ifndef __APPLE__
	if (adapter->msix_entries) {
		int vector = 0, i;
		
		free_irq(adapter->msix_entries[vector++].vector, adapter);
		
		for (i = 0; i < adapter->num_q_vectors; i++)
			free_irq(adapter->msix_entries[vector++].vector,
			         adapter->q_vector[i]);
	} else {
		free_irq(adapter->pdev->irq, adapter);
	}
#endif
}

/**
 * igb_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static void igb_irq_disable(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	
	/*
	 * we need to be careful when disabling interrupts.  The VFs are also
	 * mapped into these registers and so clearing the bits can cause
	 * issues on the VF drivers so we only need to clear what we set
	 */
	if (adapter->msix_entries) {
		u32 regval = E1000_READ_REG(hw, E1000_EIAM);
		E1000_WRITE_REG(hw, E1000_EIAM, regval & ~adapter->eims_enable_mask);
		E1000_WRITE_REG(hw, E1000_EIMC, adapter->eims_enable_mask);
		regval = E1000_READ_REG(hw, E1000_EIAC);
		E1000_WRITE_REG(hw, E1000_EIAC, regval & ~adapter->eims_enable_mask);
	}
	
	E1000_WRITE_REG(hw, E1000_IAM, 0);
	E1000_WRITE_REG(hw, E1000_IMC, ~0);
	E1000_WRITE_FLUSH(hw);
#ifndef __APPLE__
	if (adapter->msix_entries) {
		int vector = 0, i;
		
		synchronize_irq(adapter->msix_entries[vector++].vector);
		
		for (i = 0; i < adapter->num_q_vectors; i++)
			synchronize_irq(adapter->msix_entries[vector++].vector);
	} else {
		synchronize_irq(adapter->pdev->irq);
	}
#endif
}

/**
 * igb_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/
static void igb_irq_enable(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	
	if (adapter->msix_entries) {
		u32 ims = E1000_IMS_LSC | E1000_IMS_DOUTSYNC | E1000_IMS_DRSTA;
		u32 regval = E1000_READ_REG(hw, E1000_EIAC);
		E1000_WRITE_REG(hw, E1000_EIAC, regval | adapter->eims_enable_mask);
		regval = E1000_READ_REG(hw, E1000_EIAM);
		E1000_WRITE_REG(hw, E1000_EIAM, regval | adapter->eims_enable_mask);
		E1000_WRITE_REG(hw, E1000_EIMS, adapter->eims_enable_mask);
		if (adapter->vfs_allocated_count) {
			E1000_WRITE_REG(hw, E1000_MBVFIMR, 0xFF);
			ims |= E1000_IMS_VMMB;
			/* For I350 device only enable MDD interrupts*/
			if ((adapter->mdd) &&
			    (adapter->hw.mac.type == e1000_i350))
				ims |= E1000_IMS_MDDET;
		}
		E1000_WRITE_REG(hw, E1000_IMS, ims);
	} else {
		E1000_WRITE_REG(hw, E1000_IMS, IMS_ENABLE_MASK |
						E1000_IMS_DRSTA);
		E1000_WRITE_REG(hw, E1000_IAM, IMS_ENABLE_MASK |
						E1000_IMS_DRSTA);
	}
}

static void igb_update_mng_vlan(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 vid = adapter->hw.mng_cookie.vlan_id;
	u16 old_vid = adapter->mng_vlan_id;
	
	if (hw->mng_cookie.status & E1000_MNG_DHCP_COOKIE_STATUS_VLAN) {
		/* add VID to filter table */
		igb_vfta_set(adapter, vid, TRUE);
		adapter->mng_vlan_id = vid;
	} else {
		adapter->mng_vlan_id = IGB_MNG_VLAN_NONE;
	}
	
	if ((old_vid != (u16)IGB_MNG_VLAN_NONE) &&
	    (vid != old_vid) &&
#ifdef HAVE_VLAN_RX_REGISTER
	    !vlan_group_get_device(adapter->vlgrp, old_vid)) {
#else
	    !test_bit(old_vid, adapter->active_vlans)) {
#endif
			/* remove VID from filter table */
			igb_vfta_set(adapter, old_vid, FALSE);
		}
	}
	
	/**
	 * igb_release_hw_control - release control of the h/w to f/w
	 * @adapter: address of board private structure
	 *
	 * igb_release_hw_control resets CTRL_EXT:DRV_LOAD bit.
	 * For ASF and Pass Through versions of f/w this means that the
	 * driver is no longer loaded.
	 *
	 **/
	static void igb_release_hw_control(struct igb_adapter *adapter)
	{
		struct e1000_hw *hw = &adapter->hw;
		u32 ctrl_ext;
		
		/* Let firmware take over control of h/w */
		ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
		E1000_WRITE_REG(hw, E1000_CTRL_EXT,
						ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
	}
	
	/**
	 * igb_get_hw_control - get control of the h/w from f/w
	 * @adapter: address of board private structure
	 *
	 * igb_get_hw_control sets CTRL_EXT:DRV_LOAD bit.
	 * For ASF and Pass Through versions of f/w this means that
	 * the driver is loaded.
	 *
	 **/
	static void igb_get_hw_control(struct igb_adapter *adapter)
	{
		struct e1000_hw *hw = &adapter->hw;
		u32 ctrl_ext;
		
		/* Let firmware know the driver has taken over */
		ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
		E1000_WRITE_REG(hw, E1000_CTRL_EXT,
						ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	}
	
	/**
	 * igb_configure - configure the hardware for RX and TX
	 * @adapter: private board structure
	 **/
	static void igb_configure(struct igb_adapter *adapter)
	{
		IOEthernetController* netdev = adapter->netdev;
		int i;
		
		igb_get_hw_control(adapter);
		igb_set_rx_mode(netdev);
		
		igb_restore_vlan(adapter);
		
		igb_setup_tctl(adapter);
		igb_setup_mrqc(adapter);
		igb_setup_rctl(adapter);
		
		igb_configure_tx(adapter);
		igb_configure_rx(adapter);
		
		e1000_rx_fifo_flush_82575(&adapter->hw);
#ifdef CONFIG_NETDEVICES_MULTIQUEUE
		
		if (adapter->num_tx_queues > 1)
			netdev->features |= NETIF_F_MULTI_QUEUE;
		else
			netdev->features &= ~NETIF_F_MULTI_QUEUE;
#endif
		
		/* call igb_desc_unused which always leaves
		 * at least 1 descriptor unused to make sure
		 * next_to_use != next_to_clean */
		for (i = 0; i < adapter->num_rx_queues; i++) {
			struct igb_ring *ring = adapter->rx_ring[i];
			igb_alloc_rx_buffers(ring, igb_desc_unused(ring));
		}
	}
	
/**
 * igb_power_up_link - Power up the phy/serdes link
 * @adapter: address of board private structure
 **/
void igb_power_up_link(struct igb_adapter *adapter)
{
	if (adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_up_phy(&adapter->hw);
	else
		e1000_power_up_fiber_serdes_link(&adapter->hw);

	e1000_phy_hw_reset(&adapter->hw);
}
	
/**
 * igb_power_down_link - Power down the phy/serdes link
 * @adapter: address of board private structure
 */
static void igb_power_down_link(struct igb_adapter *adapter)
{
	if (adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_down_phy(&adapter->hw);
	else
		e1000_shutdown_fiber_serdes_link(&adapter->hw);
}
	
	/**
	 * igb_up - Open the interface and prepare it to handle traffic
	 * @adapter: board private structure
	 **/
	int igb_up(struct igb_adapter *adapter)
	{
		struct e1000_hw *hw = &adapter->hw;
		//int i;
		
		/* hardware has been reset, we need to reload some things */
		igb_configure(adapter);
		
		clear_bit(__IGB_DOWN, &adapter->state);
#ifndef __APPLE__
		for (i = 0; i < adapter->num_q_vectors; i++)
			napi_enable(&(adapter->q_vector[i]->napi));
#endif
		if (adapter->msix_entries)
			igb_configure_msix(adapter);
		else
			igb_assign_vector(adapter->q_vector[0], 0);
		
		igb_configure_lli(adapter);
		
		/* Clear any pending interrupts. */
		E1000_READ_REG(hw, E1000_ICR);
		igb_irq_enable(adapter);
		
		/* notify VFs that reset has been completed */
		if (adapter->vfs_allocated_count) {
			u32 reg_data = E1000_READ_REG(hw, E1000_CTRL_EXT);
			reg_data |= E1000_CTRL_EXT_PFRSTD;
			E1000_WRITE_REG(hw, E1000_CTRL_EXT, reg_data);
		}
		
		netif_tx_start_all_queues(adapter->netdev);
		
		if (adapter->flags & IGB_FLAG_DETECT_BAD_DMA)
			schedule_work(&adapter->dma_err_task);
		/* start the watchdog. */
		hw->mac.get_link_status = 1;
		schedule_work(&adapter->watchdog_task);
		
		return 0;
	}
	
	void igb_down(struct igb_adapter *adapter)
	{
		IOEthernetController* netdev = adapter->netdev;
		struct e1000_hw *hw = &adapter->hw;
		u32 tctl, rctl;
		//int i;
		
		/* signal that we're down so the interrupt handler does not
		 * reschedule our watchdog timer */
		set_bit(__IGB_DOWN, &adapter->state);
		
		/* disable receives in the hardware */
		rctl = E1000_READ_REG(hw, E1000_RCTL);
		E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
		/* flush and sleep below */
		netif_tx_stop_all_queues(netdev);

		/* disable transmits in the hardware */
		tctl = E1000_READ_REG(hw, E1000_TCTL);
		tctl &= ~E1000_TCTL_EN;
		E1000_WRITE_REG(hw, E1000_TCTL, tctl);
		/* flush both disables and wait for them to finish */
		E1000_WRITE_FLUSH(hw);
		usleep_range(10000, 20000);
		
#ifndef __APPLE__
		for (i = 0; i < adapter->num_q_vectors; i++)
			napi_disable(&(adapter->q_vector[i]->napi));
#endif
		igb_irq_disable(adapter);

#ifdef __APPLE__

#else
		del_timer_sync(&adapter->watchdog_timer);
		if (adapter->flags & IGB_FLAG_DETECT_BAD_DMA)
			del_timer_sync(&adapter->dma_err_timer);
		del_timer_sync(&adapter->phy_info_timer);
#endif
		netif_carrier_off(netdev);

		/* record the stats before reset*/
		igb_update_stats(adapter);
		
		adapter->link_speed = 0;
		adapter->link_duplex = 0;
		
#ifdef HAVE_PCI_ERS
		if (!pci_channel_offline(adapter->pdev))
			igb_reset(adapter);
#else
		igb_reset(adapter);
#endif
		igb_clean_all_tx_rings(adapter);
		igb_clean_all_rx_rings(adapter);
#ifdef IGB_DCA
		
		/* since we reset the hardware DCA settings were cleared */
		igb_setup_dca(adapter);
#endif
}
	
void igb_reinit_locked(struct igb_adapter *adapter)
{
	WARN_ON(in_interrupt());
	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	igb_down(adapter);
	igb_up(adapter);
	clear_bit(__IGB_RESETTING, &adapter->state);
}

void igb_reset(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_mac_info *mac = &hw->mac;
	struct e1000_fc_info *fc = &hw->fc;
	u32 pba = 0, tx_space, min_tx_space, min_rx_space;
	u16 hwm;
		
	/* Repartition Pba for greater than 9k mtu
	 * To take effect CTRL.RST is required.
	 */
	switch (mac->type) {
		case e1000_i350:
		case e1000_82580:
			pba = E1000_READ_REG(hw, E1000_RXPBS);
			pba = e1000_rxpbs_adjust_82580(pba);
			break;
		case e1000_82576:
			pba = E1000_READ_REG(hw, E1000_RXPBS);
			pba &= E1000_RXPBS_SIZE_MASK_82576;
			break;
		case e1000_82575:
		default:
			pba = E1000_PBA_34K;
			break;
	}
	
	if ((adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) &&
		(mac->type < e1000_82576)) {
		/* adjust PBA for jumbo frames */
		E1000_WRITE_REG(hw, E1000_PBA, pba);
		
		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB. */
		pba = E1000_READ_REG(hw, E1000_PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the tx fifo also stores 16 bytes of information about the tx
		 * but don't include ethernet FCS because hardware appends it */
		min_tx_space = (adapter->max_frame_size +
						sizeof(union e1000_adv_tx_desc) -
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
			
			/* if short on rx space, rx wins and must trump tx
			 * adjustment */
			if (pba < min_rx_space)
				pba = min_rx_space;
		}
		E1000_WRITE_REG(hw, E1000_PBA, pba);
	}
	
	/* flow control settings */
	/* The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, or
	 * - the full Rx FIFO size minus one full frame */
	hwm = min(((pba << 10) * 9 / 10),
			  ((pba << 10) - 2 * adapter->max_frame_size));
	
	fc->high_water = hwm & 0xFFF0;	/* 16-byte granularity */
	fc->low_water = fc->high_water - 16;
	fc->pause_time = 0xFFFF;
	fc->send_xon = 1;
	fc->current_mode = fc->requested_mode;
	
	/* disable receive for all VFs and wait one second */
	if (adapter->vfs_allocated_count) {
		int i;
		/*
		 * Clear all flags except indication that the PF has set
		 * the VF MAC addresses administratively
		 */
		for (i = 0 ; i < adapter->vfs_allocated_count; i++)
			adapter->vf_data[i].flags &= IGB_VF_FLAG_PF_SET_MAC;
		
		/* ping all the active vfs to let them know we are going down */
		igb_ping_all_vfs(adapter);
		
		/* disable transmits and receives */
		E1000_WRITE_REG(hw, E1000_VFRE, 0);
		E1000_WRITE_REG(hw, E1000_VFTE, 0);
	}
	
	/* Allow time for pending master requests to run */
	e1000_reset_hw(hw);
	E1000_WRITE_REG(hw, E1000_WUC, 0);
	
	if (e1000_init_hw(hw))
		IOLog( "Hardware Error\n");
	
	igb_init_dmac(adapter, pba);
	/* Re-initialize the thermal sensor on i350 devices. */
	if (mac->type == e1000_i350 && hw->bus.func == 0) {
		/*
		 * If present, re-initialize the external thermal sensor
		 * interface.
		 */
		if (adapter->ets)
			e1000_set_i2c_bb(hw);
		e1000_init_thermal_sensor_thresh(hw);
	}
	if (!netif_running(adapter->netdev))
		igb_power_down_link(adapter);
	
	igb_update_mng_vlan(adapter);
	
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	E1000_WRITE_REG(hw, E1000_VET, ETHERNET_IEEE_VLAN_TYPE);
	
	e1000_get_phy_info(hw);
}
	
#ifdef HAVE_NDO_SET_FEATURES
static netdev_features_t igb_fix_features(struct net_device *netdev,
										  netdev_features_t features)
{
	/*
	 * Since there is no support for separate tx vlan accel
	 * enabled make sure tx flag is cleared if rx is.
	 */
	if (!(features & NETIF_F_HW_VLAN_RX))
		features &= ~NETIF_F_HW_VLAN_TX;
		
	/* If Rx checksum is disabled, then LRO should also be disabled */
	if (!(features & NETIF_F_RXCSUM))
		features &= ~NETIF_F_LRO;
	
	return features;
}
	
static int igb_set_features(struct net_device *netdev,
							netdev_features_t features)
{
	u32 changed = netdev->features() ^ features;
	
	if (changed & NETIF_F_HW_VLAN_RX)
		igb_vlan_mode(netdev, features);
	
	return 0;
}
	
#endif /* HAVE_NDO_SET_FEATURES */

	
#ifdef HAVE_HW_TIME_STAMP
/**
 * igb_init_hw_timer - Initialize hardware timer used with IEEE 1588 timestamp
 * @adapter: board private structure to initialize
 *
 * igb_init_hw_timer initializes the function pointer and values for the hw
 * timer found in hardware.
 **/
static void igb_init_hw_timer(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	switch (hw->mac.type) {
	case e1000_i350:
	case e1000_82580:
		memset(&adapter->cycles, 0, sizeof(adapter->cycles));
		adapter->cycles.read = igb_read_clock;
		adapter->cycles.mask = CLOCKSOURCE_MASK(64);
		adapter->cycles.mult = 1;
		/*
		 * The 82580 timesync updates the system timer every 8ns by 8ns
		 * and the value cannot be shifted.  Instead we need to shift
		 * the registers to generate a 64bit timer value.  As a result
		 * SYSTIMR/L/H, TXSTMPL/H, RXSTMPL/H all have to be shifted by
		 * 24 in order to generate a larger value for synchronization.
		 */
		adapter->cycles.shift = IGB_82580_TSYNC_SHIFT;
		/* disable system timer temporarily by setting bit 31 */
		E1000_WRITE_REG(hw, E1000_TSAUXC, 0x80000000);
		E1000_WRITE_FLUSH(hw);

		/* Set registers so that rollover occurs soon to test this. */
		E1000_WRITE_REG(hw, E1000_SYSTIMR, 0x00000000);
		E1000_WRITE_REG(hw, E1000_SYSTIML, 0x80000000);
		E1000_WRITE_REG(hw, E1000_SYSTIMH, 0x000000FF);
		E1000_WRITE_FLUSH(hw);

		/* enable system timer by clearing bit 31 */
		E1000_WRITE_REG(hw, E1000_TSAUXC, 0x0);
		E1000_WRITE_FLUSH(hw);

		timecounter_init(&adapter->clock,
				 &adapter->cycles,
				 ktime_to_ns(ktime_get_real()));
		/*
		 * Synchronize our NIC clock against system wall clock. NIC
		 * time stamp reading requires ~3us per sample, each sample
		 * was pretty stable even under load => only require 10
		 * samples for each offset comparison.
		 */
		memset(&adapter->compare, 0, sizeof(adapter->compare));
		adapter->compare.source = &adapter->clock;
		adapter->compare.target = ktime_get_real;
		adapter->compare.num_samples = 10;
		timecompare_update(&adapter->compare, 0);
		break;
	case e1000_82576:
		/*
		 * Initialize hardware timer: we keep it running just in case
		 * that some program needs it later on.
		 */
		memset(&adapter->cycles, 0, sizeof(adapter->cycles));
		adapter->cycles.read = igb_read_clock;
		adapter->cycles.mask = CLOCKSOURCE_MASK(64);
		adapter->cycles.mult = 1;
		/**
		 * Scale the NIC clock cycle by a large factor so that
		 * relatively small clock corrections can be added or
		 * subtracted at each clock tick. The drawbacks of a large
		 * factor are a) that the clock register overflows more quickly
		 * (not such a big deal) and b) that the increment per tick has
		 * to fit into 24 bits.  As a result we need to use a shift of
		 * 19 so we can fit a value of 16 into the TIMINCA register.
		 */
		adapter->cycles.shift = IGB_82576_TSYNC_SHIFT;
		E1000_WRITE_REG(hw, E1000_TIMINCA,
		                (1 << E1000_TIMINCA_16NS_SHIFT) |
		                (16 << IGB_82576_TSYNC_SHIFT));

		/* Set registers so that rollover occurs soon to test this. */
		E1000_WRITE_REG(hw, E1000_SYSTIML, 0x00000000);
		E1000_WRITE_REG(hw, E1000_SYSTIMH, 0xFF800000);
		E1000_WRITE_FLUSH(hw);

		timecounter_init(&adapter->clock,
				 &adapter->cycles,
				 ktime_to_ns(ktime_get_real()));
		/*
		 * Synchronize our NIC clock against system wall clock. NIC
		 * time stamp reading requires ~3us per sample, each sample
		 * was pretty stable even under load => only require 10
		 * samples for each offset comparison.
		 */
		memset(&adapter->compare, 0, sizeof(adapter->compare));
		adapter->compare.source = &adapter->clock;
		adapter->compare.target = ktime_get_real;
		adapter->compare.num_samples = 10;
		timecompare_update(&adapter->compare, 0);
		break;
	case e1000_82575:
		/* 82575 does not support timesync */
	default:
		break;
	}
}

#endif /* HAVE_HW_TIME_STAMP */
/**
 * igb_sw_init - Initialize general software structures (struct igb_adapter)
 * @adapter: board private structure to initialize
 *
 * igb_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int __devinit igb_sw_init(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	IOEthernetController *netdev = adapter->netdev;
	IOPCIDevice *pdev = adapter->pdev;

	/* PCI config space info */

	hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
	hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
	
	hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
	hw->subsystem_device_id = pdev->configRead16(kIOPCIConfigSubSystemID);

	hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);

	hw->bus.pci_cmd_word = pdev->configRead16(kIOPCIConfigCommand);

	/* set default ring sizes */
	adapter->tx_ring_count = IGB_DEFAULT_TXD;
	adapter->rx_ring_count = IGB_DEFAULT_RXD;

	/* set default work limits */
	adapter->tx_work_limit = IGB_DEFAULT_TX_WORK;

	adapter->max_frame_size = ((AppleIGB*)netdev)->mtu() + ETH_HLEN + ETH_FCS_LEN +
					      VLAN_HLEN;

	/* Initialize the hardware-specific values */
	if (e1000_setup_init_funcs(hw, TRUE)) {
		IOLog( "Hardware Initialization Failure\n");
		return -EIO;
	}

	igb_check_options(adapter);

	adapter->mac_table = (igb_mac_addr*)kzalloc(sizeof(struct igb_mac_addr) * hw->mac.rar_entry_count);

	/* Setup and initialize a copy of the hw vlan table array */
	adapter->shadow_vfta = (u32 *)kzalloc(sizeof(u32) * E1000_VFTA_ENTRIES);

	/* These calls may decrease the number of queues */
	igb_set_sriov_capability(adapter);

	if (igb_init_interrupt_scheme(adapter)) {
		IOLog( "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	/* Explicitly disable IRQ since the NIC can be in any state. */
	igb_irq_disable(adapter);

	set_bit(__IGB_DOWN, &adapter->state);
	return 0;
}

/**
 * igb_open - Called when a network interface is made active
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
static int igb_open(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	int err;
	//int i;

	/* disallow open during test */
	if (test_bit(__IGB_TESTING, &adapter->state))
		return -EBUSY;

	netif_carrier_off(netdev);

	/* allocate transmit descriptors */
	err = igb_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = igb_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;

	igb_power_up_link(adapter);

	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.  */
	igb_configure(adapter);

	err = igb_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* From here on the code is the same as igb_up() */
	clear_bit(__IGB_DOWN, &adapter->state);
#ifndef __APPLE__
	for (i = 0; i < adapter->num_q_vectors; i++)
		napi_enable(&(adapter->q_vector[i]->napi));
#endif
	igb_configure_lli(adapter);

	/* Clear any pending interrupts. */
	E1000_READ_REG(hw, E1000_ICR);

	igb_irq_enable(adapter);

	/* notify VFs that reset has been completed */
	if (adapter->vfs_allocated_count) {
		u32 reg_data = E1000_READ_REG(hw, E1000_CTRL_EXT);
		reg_data |= E1000_CTRL_EXT_PFRSTD;
		E1000_WRITE_REG(hw, E1000_CTRL_EXT, reg_data);
	}

	netif_tx_start_all_queues(netdev);

	if (adapter->flags & IGB_FLAG_DETECT_BAD_DMA)
		schedule_work(&adapter->dma_err_task);

	/* start the watchdog. */
	hw->mac.get_link_status = 1;
	schedule_work(&adapter->watchdog_task);

	return E1000_SUCCESS;

err_req_irq:
	igb_release_hw_control(adapter);
	igb_power_down_link(adapter);
	igb_free_all_rx_resources(adapter);
err_setup_rx:
	igb_free_all_tx_resources(adapter);
err_setup_tx:
	igb_reset(adapter);

	return err;
}

/**
 * igb_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the driver's control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int igb_close(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	WARN_ON(test_bit(__IGB_RESETTING, &adapter->state));
	igb_down(adapter);

	/*CMW test */
	igb_release_hw_control(adapter);
	
	igb_free_irq(adapter);

	igb_free_all_tx_resources(adapter);
	igb_free_all_rx_resources(adapter);

	return 0;
}

/**
 * igb_setup_tx_resources - allocate Tx resources (Descriptors)
 * @tx_ring: tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
int igb_setup_tx_resources(struct igb_ring *tx_ring)
{
	//struct device *dev = tx_ring->dev;
	//int orig_node = dev_to_node(dev);
	int size;

	size = sizeof(struct igb_tx_buffer) * tx_ring->count;
	tx_ring->tx_buffer_info = (igb_tx_buffer*)vzalloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;

	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(union e1000_adv_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	//set_dev_node(dev, orig_node);
#ifdef __APPLE__
	tx_ring->pool= IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task,
							kIODirectionInOut | kIOMemoryPhysicallyContiguous,
							(vm_size_t)(tx_ring->size), PAGE_SIZE );
	
	if (!tx_ring->pool)
		goto err;
	tx_ring->pool->prepare();
	tx_ring->desc = tx_ring->pool->getBytesNoCopy();
	tx_ring->dma = tx_ring->pool->getPhysicalAddress();
#else
	tx_ring->desc = dma_alloc_coherent(dev,
						   tx_ring->size,
						   &tx_ring->dma,
						   GFP_KERNEL);
	
	if (!tx_ring->desc)
		goto err;
#endif

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	vfree(tx_ring->tx_buffer_info, size);
	IOLog("Unable to allocate memory for the transmit descriptor ring\n");
	return -ENOMEM;
}

/**
 * igb_setup_all_tx_resources - wrapper to allocate Tx resources
 *				  (Descriptors) for all queues
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static int igb_setup_all_tx_resources(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	int i, err = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		err = igb_setup_tx_resources(adapter->tx_ring[i]);
		if (err) {
			IOLog(
				"Allocation for Tx Queue %u failed\n", i);
			for (i--; i >= 0; i--)
				igb_free_tx_resources(adapter,adapter->tx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 * igb_setup_tctl - configure the transmit control registers
 * @adapter: Board private structure
 **/
void igb_setup_tctl(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 tctl;

	/* disable queue 0 which is enabled by default on 82575 and 82576 */
	E1000_WRITE_REG(hw, E1000_TXDCTL(0), 0);

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
		(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);

	e1000_config_collision_dist(hw);

	/* Enable transmits */
	tctl |= E1000_TCTL_EN;

	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
}

/**
 * igb_configure_tx_ring - Configure transmit ring after Reset
 * @adapter: board private structure
 * @ring: tx ring to configure
 *
 * Configure a transmit ring after a reset.
 **/
void igb_configure_tx_ring(struct igb_adapter *adapter,
                           struct igb_ring *ring)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 txdctl = 0;
	u64 tdba = ring->dma;
	int reg_idx = ring->reg_idx;

	/* disable the queue */
	E1000_WRITE_REG(hw, E1000_TXDCTL(reg_idx), 0);
	E1000_WRITE_FLUSH(hw);
	mdelay(10);

	E1000_WRITE_REG(hw, E1000_TDLEN(reg_idx),
	                ring->count * sizeof(union e1000_adv_tx_desc));
	E1000_WRITE_REG(hw, E1000_TDBAL(reg_idx),
	                tdba & 0x00000000ffffffffULL);
	E1000_WRITE_REG(hw, E1000_TDBAH(reg_idx), tdba >> 32);

	ring->tail = hw->hw_addr + E1000_TDT(reg_idx);
	E1000_WRITE_REG(hw, E1000_TDH(reg_idx), 0);
	writel(0, ring->tail);

	txdctl |= IGB_TX_PTHRESH;
	txdctl |= IGB_TX_HTHRESH << 8;
	txdctl |= IGB_TX_WTHRESH << 16;

	txdctl |= E1000_TXDCTL_QUEUE_ENABLE;
	E1000_WRITE_REG(hw, E1000_TXDCTL(reg_idx), txdctl);
}

/**
 * igb_configure_tx - Configure transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void igb_configure_tx(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		igb_configure_tx_ring(adapter, adapter->tx_ring[i]);
}

/**
 * igb_setup_rx_resources - allocate Rx resources (Descriptors)
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
int igb_setup_rx_resources(struct igb_ring *rx_ring)
{
	//struct device *dev = rx_ring->dev;
	//int orig_node = dev_to_node(dev);
	int size, desc_len;

	size = sizeof(struct igb_rx_buffer) * rx_ring->count;
	rx_ring->rx_buffer_info = (igb_rx_buffer*)vzalloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;

	desc_len = sizeof(union e1000_adv_rx_desc);

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * desc_len;
	rx_ring->size = ALIGN(rx_ring->size, 4096);
#ifdef __APPLE__
	rx_ring->pool= IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task,
								kIODirectionInOut | kIOMemoryPhysicallyContiguous,
								(vm_size_t)(rx_ring->size), PAGE_SIZE );
	
	if (!rx_ring->pool)
		goto err;
	rx_ring->pool->prepare();
	rx_ring->desc = rx_ring->pool->getBytesNoCopy();
	rx_ring->dma = rx_ring->pool->getPhysicalAddress();
#else
	rx_ring->desc = dma_alloc_coherent(dev,
						   rx_ring->size,
						   &rx_ring->dma,
						   GFP_KERNEL);

	if (!rx_ring->desc)
		goto err;
#endif

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;

err:
	vfree(rx_ring->rx_buffer_info,size);
	rx_ring->rx_buffer_info = NULL;
	IOLog("Unable to allocate memory for the receive descriptor"
		" ring\n");
	return -ENOMEM;
}

/**
 * igb_setup_all_rx_resources - wrapper to allocate Rx resources
 *				  (Descriptors) for all queues
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static int igb_setup_all_rx_resources(struct igb_adapter *adapter)
{
	//IOPCIDevice *pdev = adapter->pdev;
	int i, err = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		err = igb_setup_rx_resources(adapter->rx_ring[i]);
		if (err) {
			IOLog(
				"Allocation for Rx Queue %u failed\n", i);
			for (i--; i >= 0; i--)
				igb_free_rx_resources(adapter, adapter->rx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 * igb_setup_mrqc - configure the multiple receive queue control registers
 * @adapter: Board private structure
 **/
static void igb_setup_mrqc(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mrqc, rxcsum;
	u32 j, num_rx_queues, shift = 0, shift2 = 0;
	union e1000_reta {
		u32 dword;
		u8  bytes[4];
	} reta;
	static const u8 rsshash[40] = {
		0x6d, 0x5a, 0x56, 0xda, 0x25, 0x5b, 0x0e, 0xc2, 0x41, 0x67,
		0x25, 0x3d, 0x43, 0xa3, 0x8f, 0xb0, 0xd0, 0xca, 0x2b, 0xcb,
		0xae, 0x7b, 0x30, 0xb4,	0x77, 0xcb, 0x2d, 0xa3, 0x80, 0x30,
		0xf2, 0x0c, 0x6a, 0x42, 0xb7, 0x3b, 0xbe, 0xac, 0x01, 0xfa };

	/* Fill out hash function seeds */
	for (j = 0; j < 10; j++) {
		u32 rsskey = rsshash[(j * 4)];
		rsskey |= rsshash[(j * 4) + 1] << 8;
		rsskey |= rsshash[(j * 4) + 2] << 16;
		rsskey |= rsshash[(j * 4) + 3] << 24;
		E1000_WRITE_REG_ARRAY(hw, E1000_RSSRK(0), j, rsskey);
	}

	num_rx_queues = adapter->rss_queues;

	if (adapter->vfs_allocated_count || adapter->vmdq_pools) {
		/* 82575 and 82576 supports 2 RSS queues for VMDq */
		switch (hw->mac.type) {
		case e1000_i350:
		case e1000_82580:
			num_rx_queues = 1;
			shift = 0;
			break;
		case e1000_82576:
			shift = 3;
			num_rx_queues = 2;
			break;
		case e1000_82575:
			shift = 2;
			shift2 = 6;
		default:
			break;
		}
	} else {
		if (hw->mac.type == e1000_82575)
			shift = 6;
	}

	for (j = 0; j < (32 * 4); j++) {
		reta.bytes[j & 3] = (j % num_rx_queues) << shift;
		if (shift2)
			reta.bytes[j & 3] |= num_rx_queues << shift2;
		if ((j & 3) == 3)
			E1000_WRITE_REG(hw, E1000_RETA(j >> 2), reta.dword);
	}

	/*
	 * Disable raw packet checksumming so that RSS hash is placed in
	 * descriptor on writeback.  No need to enable TCP/UDP/IP checksum
	 * offloads as they are enabled by default
	 */
	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	rxcsum |= E1000_RXCSUM_PCSD;

	if (adapter->hw.mac.type >= e1000_82576)
		/* Enable Receive Checksum Offload for SCTP */
		rxcsum |= E1000_RXCSUM_CRCOFL;

	/* Don't need to set TUOFL or IPOFL, they default to 1 */
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

	/* If VMDq is enabled then we set the appropriate mode for that, else
	 * we default to RSS so that an RSS hash is calculated per packet even
	 * if we are only using one queue */
	if (adapter->vfs_allocated_count || adapter->vmdq_pools) {
		if (hw->mac.type > e1000_82575) {
			/* Set the default pool for the PF's first queue */
			u32 vtctl = E1000_READ_REG(hw, E1000_VT_CTL);
			vtctl &= ~(E1000_VT_CTL_DEFAULT_POOL_MASK |
				   E1000_VT_CTL_DISABLE_DEF_POOL);
			vtctl |= adapter->vfs_allocated_count <<
				E1000_VT_CTL_DEFAULT_POOL_SHIFT;
			E1000_WRITE_REG(hw, E1000_VT_CTL, vtctl);
		} else if (adapter->rss_queues > 1) {
			/* set default queue for pool 1 to queue 2 */
			E1000_WRITE_REG(hw, E1000_VT_CTL,
				        adapter->rss_queues << 7);
		}
		if (adapter->rss_queues > 1)
			mrqc = E1000_MRQC_ENABLE_VMDQ_RSS_2Q;
		else
			mrqc = E1000_MRQC_ENABLE_VMDQ;
	} else {
		mrqc = E1000_MRQC_ENABLE_RSS_4Q;
	}

	igb_vmm_control(adapter);

	/*
	 * Generate RSS hash based on TCP port numbers and/or
	 * IPv4/v6 src and dst addresses since UDP cannot be
	 * hashed reliably due to IP fragmentation
	 */
	mrqc |= E1000_MRQC_RSS_FIELD_IPV4 |
		E1000_MRQC_RSS_FIELD_IPV4_TCP |
		E1000_MRQC_RSS_FIELD_IPV6 |
		E1000_MRQC_RSS_FIELD_IPV6_TCP |
		E1000_MRQC_RSS_FIELD_IPV6_TCP_EX;

	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

/**
 * igb_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
void igb_setup_rctl(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;

	rctl = E1000_READ_REG(hw, E1000_RCTL);

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl &= ~(E1000_RCTL_LBM_TCVR | E1000_RCTL_LBM_MAC);

	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/*
	 * enable stripping of CRC. It's unlikely this will break BMC
	 * redirection as it did with e1000. Newer features require
	 * that the HW strips the CRC.
	 */
	rctl |= E1000_RCTL_SECRC;

	/* disable store bad packets and clear size bits. */
	rctl &= ~(E1000_RCTL_SBP | E1000_RCTL_SZ_256);

	/* enable LPE to prevent packets larger than max_frame_size */
	rctl |= E1000_RCTL_LPE;

	/* disable queue 0 to prevent tail write w/o re-config */
	E1000_WRITE_REG(hw, E1000_RXDCTL(0), 0);

	/* Attention!!!  For SR-IOV PF driver operations you must enable
	 * queue drop for all VF and PF queues to prevent head of line blocking
	 * if an un-trusted VF does not provide descriptors to hardware.
	 */
	if (adapter->vfs_allocated_count) {
		/* set all queue drop enable bits */
		E1000_WRITE_REG(hw, E1000_QDE, ALL_QUEUES);
	}

	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
}

static inline int igb_set_vf_rlpml(struct igb_adapter *adapter, int size,
                                   int vfn)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 vmolr;

	/* if it isn't the PF check to see if VFs are enabled and
	 * increase the size to support vlan tags */
	if (vfn < adapter->vfs_allocated_count &&
	    adapter->vf_data[vfn].vlans_enabled)
		size += VLAN_HLEN;

	vmolr = E1000_READ_REG(hw, E1000_VMOLR(vfn));
	vmolr &= ~E1000_VMOLR_RLPML_MASK;
	vmolr |= size | E1000_VMOLR_LPE;
	E1000_WRITE_REG(hw, E1000_VMOLR(vfn), vmolr);

	return 0;
}

/**
 * igb_rlpml_set - set maximum receive packet size
 * @adapter: board private structure
 *
 * Configure maximum receivable packet size.
 **/
static void igb_rlpml_set(struct igb_adapter *adapter)
{
	u32 max_frame_size = adapter->max_frame_size;
	struct e1000_hw *hw = &adapter->hw;
	u16 pf_id = adapter->vfs_allocated_count;

	if (adapter->vmdq_pools && hw->mac.type != e1000_82575) {
		int i;
		for (i = 0; i < adapter->vmdq_pools; i++)
			igb_set_vf_rlpml(adapter, max_frame_size, pf_id + i);
		/*
		 * If we're in VMDQ or SR-IOV mode, then set global RLPML
		 * to our max jumbo frame size, in case we need to enable
		 * jumbo frames on one of the rings later.
		 * This will not pass over-length frames into the default
		 * queue because it's gated by the VMOLR.RLPML.
		 */
		max_frame_size = MAX_JUMBO_FRAME_SIZE;
	}
	/* Set VF RLPML for the PF device. */
	if (adapter->vfs_allocated_count)
		igb_set_vf_rlpml(adapter, max_frame_size, pf_id);

	E1000_WRITE_REG(hw, E1000_RLPML, max_frame_size);
}

static inline void igb_set_vf_vlan_strip(struct igb_adapter *adapter,
					int vfn, bool enable)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 val;
	void __iomem *reg;

	if (hw->mac.type < e1000_82576)
		return;

	if (hw->mac.type == e1000_i350)
		reg = hw->hw_addr + E1000_DVMOLR(vfn);
	else
		reg = hw->hw_addr + E1000_VMOLR(vfn);

	val = readl(reg);
	if (enable)
		val |= E1000_VMOLR_STRVLAN;
	else
		val &= ~(E1000_VMOLR_STRVLAN);
	writel(val, reg);
}
static inline void igb_set_vmolr(struct igb_adapter *adapter,
				 int vfn, bool aupe)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 vmolr;

	/*
	 * This register exists only on 82576 and newer so if we are older then
	 * we should exit and do nothing
	 */
	if (hw->mac.type < e1000_82576)
		return;

	vmolr = E1000_READ_REG(hw, E1000_VMOLR(vfn));

	if (aupe)
		vmolr |= E1000_VMOLR_AUPE;        /* Accept untagged packets */
	else
		vmolr &= ~(E1000_VMOLR_AUPE); /* Tagged packets ONLY */

	/* clear all bits that might not be set */
	vmolr &= ~E1000_VMOLR_RSSE;

	if (adapter->rss_queues > 1 && vfn == adapter->vfs_allocated_count)
		vmolr |= E1000_VMOLR_RSSE; /* enable RSS */

	vmolr |= E1000_VMOLR_BAM;	   /* Accept broadcast */
	vmolr |= E1000_VMOLR_LPE; 	   /* Accept long packets */

	E1000_WRITE_REG(hw, E1000_VMOLR(vfn), vmolr);
}

/**
 * igb_configure_rx_ring - Configure a receive ring after Reset
 * @adapter: board private structure
 * @ring: receive ring to be configured
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void igb_configure_rx_ring(struct igb_adapter *adapter,
                           struct igb_ring *ring)
{
	struct e1000_hw *hw = &adapter->hw;
	u64 rdba = ring->dma;
	int reg_idx = ring->reg_idx;
	u32 srrctl = 0, rxdctl = 0;

	/* disable the queue */
	E1000_WRITE_REG(hw, E1000_RXDCTL(reg_idx), 0);

	/* Set DMA base address registers */
	E1000_WRITE_REG(hw, E1000_RDBAL(reg_idx),
	                rdba & 0x00000000ffffffffULL);
	E1000_WRITE_REG(hw, E1000_RDBAH(reg_idx), rdba >> 32);
	E1000_WRITE_REG(hw, E1000_RDLEN(reg_idx),
	               ring->count * sizeof(union e1000_adv_rx_desc));

	/* initialize head and tail */
	ring->tail = hw->hw_addr + E1000_RDT(reg_idx);
	E1000_WRITE_REG(hw, E1000_RDH(reg_idx), 0);
	writel(0, ring->tail);

	/* set descriptor configuration */
#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
	srrctl = IGB_RX_HDR_LEN << E1000_SRRCTL_BSIZEHDRSIZE_SHIFT;
#if (PAGE_SIZE / 2) > IGB_RXBUFFER_16384
	srrctl |= IGB_RXBUFFER_16384 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
#else
	srrctl |= (PAGE_SIZE / 2) >> E1000_SRRCTL_BSIZEPKT_SHIFT;
#endif
	srrctl |= E1000_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS;
#else /* CONFIG_IGB_DISABLE_PACKET_SPLIT */
	srrctl = ALIGN(ring->rx_buffer_len, 1024) >>
	         E1000_SRRCTL_BSIZEPKT_SHIFT;
	srrctl |= E1000_SRRCTL_DESCTYPE_ADV_ONEBUF;
#endif /* CONFIG_IGB_DISABLE_PACKET_SPLIT */
#ifdef IGB_PER_PKT_TIMESTAMP
	if (hw->mac.type >= e1000_82580)
		srrctl |= E1000_SRRCTL_TIMESTAMP;
#endif
	/*
	 * We should set the drop enable bit if:
	 *  SR-IOV is enabled
	 *   or
	 *  Flow Control is disabled and number of RX queues > 1
	 *
	 *  This allows us to avoid head of line blocking for security
	 *  and performance reasons.
	 */
	if (adapter->vfs_allocated_count ||
	    (adapter->num_rx_queues > 1 &&
	     (hw->fc.requested_mode == e1000_fc_none ||
	      hw->fc.requested_mode == e1000_fc_rx_pause)))
		srrctl |= E1000_SRRCTL_DROP_EN;

	E1000_WRITE_REG(hw, E1000_SRRCTL(reg_idx), srrctl);

	/* set filtering for VMDQ pools */
	igb_set_vmolr(adapter, reg_idx & 0x7, true);

	rxdctl |= IGB_RX_PTHRESH;
	rxdctl |= IGB_RX_HTHRESH << 8;
	rxdctl |= IGB_RX_WTHRESH << 16;

	/* enable receive descriptor fetching */
	rxdctl |= E1000_RXDCTL_QUEUE_ENABLE;
	E1000_WRITE_REG(hw, E1000_RXDCTL(reg_idx), rxdctl);
}

/**
 * igb_configure_rx - Configure receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void igb_configure_rx(struct igb_adapter *adapter)
{
	int i;

	/* set UTA to appropriate mode */
	igb_set_uta(adapter);

	igb_full_sync_mac_table(adapter);
	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring */
	for (i = 0; i < adapter->num_rx_queues; i++)
		igb_configure_rx_ring(adapter, adapter->rx_ring[i]);
}

/**
 * igb_free_tx_resources - Free Tx Resources per Queue
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
void igb_free_tx_resources(struct igb_adapter *adapter, struct igb_ring *tx_ring)
{
	igb_clean_tx_ring(adapter, tx_ring);

	vfree(tx_ring->tx_buffer_info,sizeof(struct igb_tx_buffer) * tx_ring->count);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

#ifdef __APPLE__
	tx_ring->pool->complete();
	tx_ring->pool->release();
	tx_ring->pool = NULL;
#else
	dma_free_coherent(tx_ring->dev, tx_ring->size,
			  tx_ring->desc, tx_ring->dma);
#endif

	tx_ring->desc = NULL;
}

/**
 * igb_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void igb_free_all_tx_resources(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		igb_free_tx_resources(adapter, adapter->tx_ring[i]);
}

void igb_unmap_and_free_tx_resource(struct igb_adapter *adapter,
					struct igb_ring *ring,
				    struct igb_tx_buffer *tx_buffer)
{
#ifdef __APPLE__
	if (tx_buffer->skb) {
		adapter->netdev->freePacket(tx_buffer->skb);
	}
#else
	if (tx_buffer->skb) {
		adapter->netdev->freePacket(tx_buffer->skb);
		if (dma_unmap_len(tx_buffer, len))
			dma_unmap_single(ring->dev,
			                 dma_unmap_addr(tx_buffer, dma),
			                 dma_unmap_len(tx_buffer, len),
			                 DMA_TO_DEVICE);
	} else if (dma_unmap_len(tx_buffer, len)) {
		dma_unmap_page(ring->dev,
		               dma_unmap_addr(tx_buffer, dma),
		               dma_unmap_len(tx_buffer, len),
		               DMA_TO_DEVICE);
	}
#endif
	tx_buffer->next_to_watch = NULL;
	tx_buffer->skb = NULL;
	dma_unmap_len_set(tx_buffer, len, 0);
	/* buffer_info must be completely set up in the transmit path */
}

/**
 * igb_clean_tx_ring - Free Tx Buffers
 * @tx_ring: ring to be cleaned
 **/
static void igb_clean_tx_ring(struct igb_adapter *adapter, struct igb_ring *tx_ring)
{
	struct igb_tx_buffer *buffer_info;
	unsigned long size;
	u16 i;

	if (!tx_ring->tx_buffer_info)
		return;
	/* Free all the Tx ring sk_buffs */

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->tx_buffer_info[i];
		igb_unmap_and_free_tx_resource(adapter, tx_ring, buffer_info);
	}

#ifdef CONFIG_BQL
	netdev_tx_reset_queue(txring_txq(tx_ring));
#endif /* CONFIG_BQL */
	
	size = sizeof(struct igb_tx_buffer) * tx_ring->count;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

/**
 * igb_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void igb_clean_all_tx_rings(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		igb_clean_tx_ring(adapter,adapter->tx_ring[i]);
}

/**
 * igb_free_rx_resources - Free Rx Resources
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
void igb_free_rx_resources(struct igb_adapter *adapter, struct igb_ring *rx_ring)
{
	igb_clean_rx_ring(adapter, rx_ring);

	vfree(rx_ring->rx_buffer_info,sizeof(struct igb_rx_buffer) * rx_ring->count);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;
#ifdef __APPLE__
	if(rx_ring->pool){
		rx_ring->pool->complete();
		rx_ring->pool->release();
		rx_ring->pool = NULL;
	}
#else
	dma_free_coherent(rx_ring->dev, rx_ring->size,
			  rx_ring->desc, rx_ring->dma);
#endif

	rx_ring->desc = NULL;
}

/**
 * igb_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
static void igb_free_all_rx_resources(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		igb_free_rx_resources(adapter, adapter->rx_ring[i]);
}

/**
 * igb_clean_rx_ring - Free Rx Buffers per Queue
 * @rx_ring: ring to free buffers from
 **/
void igb_clean_rx_ring(struct igb_adapter *adapter, struct igb_ring *rx_ring)
{
	unsigned long size;
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
	//const int bufsz = rx_ring->rx_buffer_len;
#else
	const int bufsz = IGB_RX_HDR_LEN;
#endif
	u16 i;

	if (!rx_ring->rx_buffer_info)
		return;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		struct igb_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];
#ifdef __APPLE__
		if (buffer_info->pool) {
			if(buffer_info->dma)
				buffer_info->pool->complete();
			buffer_info->pool->release();
			buffer_info->pool = NULL;
		}
		if (buffer_info->skb) {
			adapter->netdev->freePacket(buffer_info->skb);
			buffer_info->skb = NULL;
		}
#else
		if (buffer_info->dma) {
			dma_unmap_single(rx_ring->dev,
			                 buffer_info->dma,
					 bufsz,
					 DMA_FROM_DEVICE);
			buffer_info->dma = 0;
		}

		if (buffer_info->skb) {
			adapter->netdev->freePacket(buffer_info->skb);
			buffer_info->skb = NULL;
		}
#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
		if (buffer_info->page_dma) {
			dma_unmap_page(rx_ring->dev,
			               buffer_info->page_dma,
				       PAGE_SIZE / 2,
				       DMA_FROM_DEVICE);
			buffer_info->page_dma = 0;
		}
		if (buffer_info->page) {
			put_page(buffer_info->page);
			buffer_info->page = NULL;
			buffer_info->page_offset = 0;
		}
#endif
#endif	// __APPLE__
	}

	size = sizeof(struct igb_rx_buffer) * rx_ring->count;
	memset(rx_ring->rx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

/**
 * igb_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void igb_clean_all_rx_rings(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		igb_clean_rx_ring(adapter, adapter->rx_ring[i]);
}

#ifndef __APPLE__
/**
 * igb_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int igb_set_mac(IOEthernetController *netdev, void *p)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	igb_del_mac_filter(adapter, hw->mac.addr,
			   adapter->vfs_allocated_count);
	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(hw->mac.addr, addr->sa_data, netdev->addr_len);

	/* set the correct pool for the new PF MAC address in entry 0 */
	return igb_add_mac_filter(adapter, hw->mac.addr,
	                   adapter->vfs_allocated_count);
}
#endif

#ifndef __APPLE__
/**
 * igb_write_mc_addr_list - write multicast addresses to MTA
 * @netdev: network interface device structure
 *
 * Writes multicast address list to the MTA hash table.
 * Returns: -ENOMEM on failure
 *                0 on no addresses written
 *                X on writing X addresses to MTA
 **/
int igb_write_mc_addr_list(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *ha;
#else
	struct dev_mc_list *ha;
#endif
	u8  *mta_list;
	int i, count;
	count = netdev_mc_count(netdev);

	if (!count) {
		e1000_update_mc_addr_list(hw, NULL, 0);
		return 0;
	}
	mta_list = kzalloc(count * 6, GFP_ATOMIC);
	if (!mta_list)
		return -ENOMEM;

	/* The shared function expects a packed array of only addresses. */
	i = 0;
	netdev_for_each_mc_addr(ha, netdev)
#ifdef NETDEV_HW_ADDR_T_MULTICAST
		memcpy(mta_list + (i++ * ETH_ALEN), ha->addr, ETH_ALEN);
#else
		memcpy(mta_list + (i++ * ETH_ALEN), ha->dmi_addr, ETH_ALEN);
#endif
	e1000_update_mc_addr_list(hw, mta_list, i);
	kfree(mta_list);

	return count;
}
#endif

void igb_rar_set(struct igb_adapter *adapter, u32 index)
{
	u32 rar_low, rar_high;
	struct e1000_hw *hw = &adapter->hw;
	u8 *addr = adapter->mac_table[index].addr;
	/* HW expects these in little endian so we reverse the byte order
	 * from network order (big endian) to little endian
	 */
	rar_low = ((u32) addr[0] | ((u32) addr[1] << 8) |
	          ((u32) addr[2] << 16) | ((u32) addr[3] << 24));
	rar_high = ((u32) addr[4] | ((u32) addr[5] << 8));

	/* Indicate to hardware the Address is Valid. */
	if (adapter->mac_table[index].state & IGB_MAC_STATE_IN_USE)
		rar_high |= E1000_RAH_AV;

	if (hw->mac.type == e1000_82575)
		rar_high |= E1000_RAH_POOL_1 * adapter->mac_table[index].queue;
	else
		rar_high |= E1000_RAH_POOL_1 << adapter->mac_table[index].queue;

	E1000_WRITE_REG(hw, E1000_RAL(index), rar_low);
	E1000_WRITE_FLUSH(hw);
	E1000_WRITE_REG(hw, E1000_RAH(index), rar_high);
	E1000_WRITE_FLUSH(hw);
}

void igb_full_sync_mac_table(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;
	for (i = 0; i < hw->mac.rar_entry_count; i++) {
			igb_rar_set(adapter, i);
	}
}

void igb_sync_mac_table(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;
	for (i = 0; i < hw->mac.rar_entry_count; i++) {
		if (adapter->mac_table[i].state & IGB_MAC_STATE_MODIFIED)
			igb_rar_set(adapter, i);
		adapter->mac_table[i].state &= ~(IGB_MAC_STATE_MODIFIED);
	}
}

int igb_available_rars(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i, count = 0;

	for (i = 0; i < hw->mac.rar_entry_count; i++) {
		if (adapter->mac_table[i].state == 0)
			count++;
	}
	return count;
}

#ifdef HAVE_SET_RX_MODE
/**
 * igb_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int igb_write_uc_addr_list(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	unsigned int vfn = adapter->vfs_allocated_count;
	int count = 0;

	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > igb_available_rars(adapter))
		return -ENOMEM;
	if (!netdev_uc_empty(netdev)) {
#ifdef NETDEV_HW_ADDR_T_UNICAST
		struct netdev_hw_addr *ha;
#else
		struct dev_mc_list *ha;
#endif
		netdev_for_each_uc_addr(ha, netdev) {
#ifdef NETDEV_HW_ADDR_T_UNICAST
			igb_del_mac_filter(adapter, ha->addr, vfn);
			igb_add_mac_filter(adapter, ha->addr, vfn);
#else
			igb_del_mac_filter(adapter, ha->da_addr, vfn);
			igb_add_mac_filter(adapter, ha->da_addr, vfn);
#endif
			count++;
		}
	}
	return count;
}

#endif /* HAVE_SET_RX_MODE */
/**
 * igb_set_rx_mode - Secondary Unicast, Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_rx_mode entry point is called whenever the unicast or multicast
 * address lists or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast,
 * promiscuous mode, and all-multi behavior.
 **/
static void igb_set_rx_mode(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	unsigned int vfn = adapter->vfs_allocated_count;
	u32 rctl, vmolr = 0;
	//int count;

	/* Check for Promiscuous and All Multicast modes */
	rctl = E1000_READ_REG(hw, E1000_RCTL);

	/* clear the effected bits */
	rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE | E1000_RCTL_VFE);

	if (((AppleIGB*)netdev)->flags() & IFF_PROMISC) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		vmolr |= (E1000_VMOLR_ROPE | E1000_VMOLR_MPME);
	} else {
		if (((AppleIGB*)netdev)->flags() & IFF_ALLMULTI) {
			rctl |= E1000_RCTL_MPE;
			vmolr |= E1000_VMOLR_MPME;
		} else {
			/*
			 * Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscuous mode so
			 * that we can at least receive multicast traffic
			 */
			vmolr |= E1000_VMOLR_ROMPE;
		}
#ifdef HAVE_SET_RX_MODE
		/*
		 * Write addresses to available RAR registers, if there is not
		 * sufficient space to store all the addresses then enable
		 * unicast promiscuous mode
		 */
		count = igb_write_uc_addr_list(netdev);
		if (count < 0) {
			rctl |= E1000_RCTL_UPE;
			vmolr |= E1000_VMOLR_ROPE;
		}
#endif /* HAVE_SET_RX_MODE */
		rctl |= E1000_RCTL_VFE;
	}
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	/*
	 * In order to support SR-IOV and eventually VMDq it is necessary to set
	 * the VMOLR to enable the appropriate modes.  Without this workaround
	 * we will have issues with VLAN tag stripping not being done for frames
	 * that are only arriving because we are the default pool
	 */
	if (hw->mac.type < e1000_82576)
		return;

	vmolr |= E1000_READ_REG(hw, E1000_VMOLR(vfn)) &
	         ~(E1000_VMOLR_ROPE | E1000_VMOLR_MPME | E1000_VMOLR_ROMPE);
	E1000_WRITE_REG(hw, E1000_VMOLR(vfn), vmolr);
	igb_restore_vf_multicasts(adapter);
}

static void igb_check_wvbr(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 wvbr = 0;

	switch (hw->mac.type) {
	case e1000_82576:
	case e1000_i350:
		if (!(wvbr = E1000_READ_REG(hw, E1000_WVBR)))
			return;
		break;
	default:
		break;
	}

	adapter->wvbr |= wvbr;
}

#define IGB_STAGGERED_QUEUE_OFFSET 8

static void igb_spoof_check(struct igb_adapter *adapter)
{
	int j;

	if (!adapter->wvbr)
		return;

	for(j = 0; j < adapter->vfs_allocated_count; j++) {
		if (adapter->wvbr & (1 << j) ||
		    adapter->wvbr & (1 << (j + IGB_STAGGERED_QUEUE_OFFSET))) {
			DPRINTK(DRV, WARNING,
				"Spoof event(s) detected on VF %d\n", j);
			adapter->wvbr &=
				~((1 << j) |
				  (1 << (j + IGB_STAGGERED_QUEUE_OFFSET)));
		}
	}
}

/* Need to wait a few seconds after link up to get diagnostic information from
 * the phy */
static void igb_update_phy_info(unsigned long data)
{
	struct igb_adapter *adapter = (struct igb_adapter *) data;
	e1000_get_phy_info(&adapter->hw);
}

/**
 * igb_has_link - check shared code for link and determine up/down
 * @adapter: pointer to driver private info
 **/
bool igb_has_link(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = FALSE;

	/* get_link_status is set on LSC (link status) interrupt or
	 * rx sequence error interrupt.  get_link_status will stay
	 * false until the e1000_check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (!hw->mac.get_link_status)
			return true;
	case e1000_media_type_internal_serdes:
		e1000_check_for_link(hw);
		link_active = !hw->mac.get_link_status;
		break;
	case e1000_media_type_unknown:
	default:
		break;
	}

	return link_active;
}

static void igb_dma_err_task(struct igb_adapter *adapter,IOTimerEventSource * src)
{
		int vf;
		struct e1000_hw *hw = &adapter->hw;
		u32 hgptc;
		u32 ciaa, ciad;
		
		hgptc = E1000_READ_REG(hw, E1000_HGPTC);
		if (hgptc) /* If incrementing then no need for the check below */
			goto dma_timer_reset;
		/*
		 * Check to see if a bad DMA write target from an errant or
		 * malicious VF has caused a PCIe error.  If so then we can
		 * issue a VFLR to the offending VF(s) and then resume without
		 * requesting a full slot reset.
		 */
		
		for (vf = 0; vf < adapter->vfs_allocated_count; vf++) {
			ciaa = (vf << 16) | 0x80000000;
			/* 32 bit read so align, we really want status at offset 6 */
			ciaa |= PCI_COMMAND;
			E1000_WRITE_REG(hw, E1000_CIAA, ciaa);
			ciad = E1000_READ_REG(hw, E1000_CIAD);
			ciaa &= 0x7FFFFFFF;
			/* disable debug mode asap after reading data */
			E1000_WRITE_REG(hw, E1000_CIAA, ciaa);
			/* Get the upper 16 bits which will be the PCI status reg */
			ciad >>= 16;
			if (ciad & (PCI_STATUS_REC_MASTER_ABORT |
						PCI_STATUS_REC_TARGET_ABORT |
						PCI_STATUS_SIG_SYSTEM_ERROR)) {
				IOLog("VF %d suffered error\n", vf);
				/* Issue VFLR */
				ciaa = (vf << 16) | 0x80000000;
				ciaa |= 0xA8;
				E1000_WRITE_REG(hw, E1000_CIAA, ciaa);
				ciad = 0x00008000;  /* VFLR */
				E1000_WRITE_REG(hw, E1000_CIAD, ciad);
				ciaa &= 0x7FFFFFFF;
				E1000_WRITE_REG(hw, E1000_CIAA, ciaa);
			}
		}
dma_timer_reset:
	/* Reset the timer */
	if (!test_bit(__IGB_DOWN, &adapter->state))
		src->setTimeoutMS(100);
}
	
	
	
enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

/**
 * igb_update_ring_itr - update the dynamic ITR value based on packet size
 *
 *      Stores a new ITR value based on strictly on packet size.  This
 *      algorithm is less sophisticated than that used in igb_update_itr,
 *      due to the difficulty of synchronizing statistics across multiple
 *      receive rings.  The divisors and thresholds used by this function
 *      were determined based on theoretical maximum wire speed and testing
 *      data, in order to minimize response time while increasing bulk
 *      throughput.
 *      This functionality is controlled by the InterruptThrottleRate module
 *      parameter (see igb_param.c)
 *      NOTE:  This function is called only when operating in a multiqueue
 *             receive environment.
 * @q_vector: pointer to q_vector
 **/
static void igb_update_ring_itr(struct igb_q_vector *q_vector)
{
	int new_val = q_vector->itr_val;
	int avg_wire_size = 0;
	struct igb_adapter *adapter = q_vector->adapter;
	unsigned int packets;

	/* For non-gigabit speeds, just fix the interrupt rate at 4000
	 * ints/sec - ITR timer value of 120 ticks.
	 */
	if (adapter->link_speed != SPEED_1000) {
		new_val = IGB_4K_ITR;
		goto set_itr_val;
	}

	packets = q_vector->rx.total_packets;
	if (packets)
		avg_wire_size = q_vector->rx.total_bytes / packets;

	packets = q_vector->tx.total_packets;
	if (packets)
		avg_wire_size = max_t(u32, avg_wire_size,
		                      q_vector->tx.total_bytes / packets);

	/* if avg_wire_size isn't set no work was done */
	if (!avg_wire_size)
		goto clear_counts;

	/* Add 24 bytes to size to account for CRC, preamble, and gap */
	avg_wire_size += 24;

	/* Don't starve jumbo frames */
	avg_wire_size = min(avg_wire_size, 3000);

	/* Give a little boost to mid-size frames */
	if ((avg_wire_size > 300) && (avg_wire_size < 1200))
		new_val = avg_wire_size / 3;
	else
		new_val = avg_wire_size / 2;

	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (new_val < IGB_20K_ITR &&
	    ((q_vector->rx.ring && adapter->rx_itr_setting == 3) ||
	     (!q_vector->rx.ring && adapter->tx_itr_setting == 3)))
		new_val = IGB_20K_ITR;

set_itr_val:
	if (new_val != q_vector->itr_val) {
		q_vector->itr_val = new_val;
		q_vector->set_itr = 1;
	}
clear_counts:
	q_vector->rx.total_bytes = 0;
	q_vector->rx.total_packets = 0;
	q_vector->tx.total_bytes = 0;
	q_vector->tx.total_packets = 0;
}

/**
 * igb_update_itr - update the dynamic ITR value based on statistics
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 *      this functionality is controlled by the InterruptThrottleRate module
 *      parameter (see igb_param.c)
 *      NOTE:  These calculations are only valid when operating in a single-
 *             queue environment.
 * @q_vector: pointer to q_vector
 * @ring_container: ring info to update the itr for
 **/
static void igb_update_itr(struct igb_q_vector *q_vector,
			   struct igb_ring_container *ring_container)
{
	unsigned int packets = ring_container->total_packets;
	unsigned int bytes = ring_container->total_bytes;
	u8 itrval = ring_container->itr;

	/* no packets, exit with status unchanged */
	if (packets == 0)
		return;

	switch (itrval) {
	case lowest_latency:
		/* handle TSO and jumbo frames */
		if (bytes/packets > 8000)
			itrval = bulk_latency;
		else if ((packets < 5) && (bytes > 512))
			itrval = low_latency;
		break;
	case low_latency:  /* 50 usec aka 20000 ints/s */
		if (bytes > 10000) {
			/* this if handles the TSO accounting */
			if (bytes/packets > 8000) {
				itrval = bulk_latency;
			} else if ((packets < 10) || ((bytes/packets) > 1200)) {
				itrval = bulk_latency;
			} else if ((packets > 35)) {
				itrval = lowest_latency;
			}
		} else if (bytes/packets > 2000) {
			itrval = bulk_latency;
		} else if (packets <= 2 && bytes < 512) {
			itrval = lowest_latency;
		}
		break;
	case bulk_latency: /* 250 usec aka 4000 ints/s */
		if (bytes > 25000) {
			if (packets > 35)
				itrval = low_latency;
		} else if (bytes < 1500) {
			itrval = low_latency;
		}
		break;
	}

	/* clear work counters since we have the values we need */
	ring_container->total_bytes = 0;
	ring_container->total_packets = 0;

	/* write updated itr to ring container */
	ring_container->itr = itrval;
}

static void igb_set_itr(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	u32 new_itr = q_vector->itr_val;
	u8 current_itr = 0;

	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (adapter->link_speed != SPEED_1000) {
		current_itr = 0;
		new_itr = IGB_4K_ITR;
		goto set_itr_now;
	}

	igb_update_itr(q_vector, &q_vector->tx);
	igb_update_itr(q_vector, &q_vector->rx);

	current_itr = max(q_vector->rx.itr, q_vector->tx.itr);

	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (current_itr == lowest_latency &&
	    ((q_vector->rx.ring && adapter->rx_itr_setting == 3) ||
	     (!q_vector->rx.ring && adapter->tx_itr_setting == 3)))
		current_itr = low_latency;

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = IGB_70K_ITR; /* 70,000 ints/sec */
		break;
	case low_latency:
		new_itr = IGB_20K_ITR; /* 20,000 ints/sec */
		break;
	case bulk_latency:
		new_itr = IGB_4K_ITR;  /* 4,000 ints/sec */
		break;
	default:
		break;
	}

set_itr_now:
	if (new_itr != q_vector->itr_val) {
		/* this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing */
		new_itr = new_itr > q_vector->itr_val ?
		             max((new_itr * q_vector->itr_val) /
		                 (new_itr + (q_vector->itr_val >> 2)),
				 new_itr) :
			     new_itr;
		/* Don't write the value here; it resets the adapter's
		 * internal timer, and causes us to delay far longer than
		 * we should between interrupts.  Instead, we write the ITR
		 * value at the beginning of the next interrupt so the timing
		 * ends up being correct.
		 */
		q_vector->itr_val = new_itr;
		q_vector->set_itr = 1;
	}
}

void igb_tx_ctxtdesc(struct igb_ring *tx_ring, u32 vlan_macip_lens,
		     u32 type_tucmd, u32 mss_l4len_idx)
{
	struct e1000_adv_tx_context_desc *context_desc;
	u16 i = tx_ring->next_to_use;

	context_desc = IGB_TX_CTXTDESC(tx_ring, i);

	i++;
	tx_ring->next_to_use = (i < tx_ring->count) ? i : 0;

	/* set bits to identify this as an advanced context descriptor */
	type_tucmd |= E1000_TXD_CMD_DEXT | E1000_ADVTXD_DTYP_CTXT;

	/* For 82575, context index must be unique per ring. */
	if (test_bit(IGB_RING_FLAG_TX_CTX_IDX, &tx_ring->flags))
		mss_l4len_idx |= tx_ring->reg_idx << 4;

	context_desc->vlan_macip_lens	= cpu_to_le32(vlan_macip_lens);
	context_desc->seqnum_seed	= 0;
	context_desc->type_tucmd_mlhl	= cpu_to_le32(type_tucmd);
	context_desc->mss_l4len_idx	= cpu_to_le32(mss_l4len_idx);
}

static int igb_tso(struct igb_ring *tx_ring,
		   struct igb_tx_buffer *first,
		   u8 *hdr_len)
{
		return 0;
}

// copy for accessing c++ constants

static void igb_tx_csum(struct igb_ring *tx_ring, struct igb_tx_buffer *first)
{
	struct sk_buff *skb = first->skb;
	u32 vlan_macip_lens = 0;
	u32 mss_l4len_idx = 0;
	u32 type_tucmd = 0;

#ifdef __APPLE__

	UInt32 checksumDemanded;
	tx_ring->netdev->getChecksumDemand(skb, IONetworkController::kChecksumFamilyTCPIP, &checksumDemanded);
	checksumDemanded &= (IONetworkController::kChecksumIP|IONetworkController::kChecksumTCP|IONetworkController::kChecksumUDP);
#if     0
    IOLog("igb_tx_csum: %s%s%s\n",
          (checksumDemanded&IONetworkController::kChecksumIP)?"IP ":"",
          (checksumDemanded&IONetworkController::kChecksumTCP)?"TCP ":"",
          (checksumDemanded&IONetworkController::kChecksumUDP)?"UDP ":""
          );
#endif
	int  ehdrlen = ETHER_HDR_LEN;
	if(checksumDemanded == 0){
		if (!(first->tx_flags & IGB_TX_FLAGS_VLAN))
			return;
	} else {
		int  ip_hlen;
		u8* packet;
		
		vlan_macip_lens = type_tucmd = mss_l4len_idx = 0;
		
		/* Set the ether header length */
		packet = (u8*)mbuf_data(skb) + ehdrlen;
		
		if(checksumDemanded & (IONetworkController::kChecksumTCPIPv6|IONetworkController::kChecksumUDPIPv6)){
			// IPv6
			ip_hlen = sizeof(struct ip6_hdr);
			type_tucmd |= E1000_ADVTXD_TUCMD_IPV6;
		} else {
			struct ip *ip = (struct ip *)((u8*)mbuf_data(skb) + ehdrlen);
			ip_hlen = ip->ip_hl << 2;
			type_tucmd |= E1000_ADVTXD_TUCMD_IPV4;
		}
		vlan_macip_lens |= ip_hlen;
		
		if(checksumDemanded & IONetworkController::kChecksumTCP){
			type_tucmd |= E1000_ADVTXD_TUCMD_L4T_TCP;
			struct tcphdr* tcph = (struct tcphdr*)(packet + ip_hlen);
			mss_l4len_idx = (tcph->th_off << 2) << E1000_ADVTXD_L4LEN_SHIFT;
		} else if(checksumDemanded & IONetworkController::kChecksumUDP){
			type_tucmd |= E1000_ADVTXD_TUCMD_L4T_UDP;
			mss_l4len_idx = sizeof(struct udphdr) << E1000_ADVTXD_L4LEN_SHIFT;
		}
		
		first->tx_flags |= IGB_TX_FLAGS_CSUM;
	}
	vlan_macip_lens |= ehdrlen << E1000_ADVTXD_MACLEN_SHIFT;


#else	// __APPLE__
	
	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		if (!(first->tx_flags & IGB_TX_FLAGS_VLAN))
			return;
	} else {
		u8 l4_hdr = 0;
		switch (first->protocol) {
		case __constant_htons(ETH_P_IP):
			vlan_macip_lens |= skb_network_header_len(skb);
			type_tucmd |= E1000_ADVTXD_TUCMD_IPV4;
			l4_hdr = ip_hdr(skb)->protocol;
			break;
#ifdef NETIF_F_IPV6_CSUM
		case __constant_htons(ETH_P_IPV6):
			vlan_macip_lens |= skb_network_header_len(skb);
			l4_hdr = ipv6_hdr(skb)->nexthdr;
			break;
#endif
		default:
			if (unlikely(net_ratelimit())) {
				dev_warn(tx_ring->dev,
				 "partial checksum but proto=%x!\n",
				 first->protocol);
			}
			break;
		}

		switch (l4_hdr) {
		case IPPROTO_TCP:
			type_tucmd |= E1000_ADVTXD_TUCMD_L4T_TCP;
			mss_l4len_idx = tcp_hdrlen(skb) <<
					E1000_ADVTXD_L4LEN_SHIFT;
			break;
#ifdef HAVE_SCTP
		case IPPROTO_SCTP:
			type_tucmd |= E1000_ADVTXD_TUCMD_L4T_SCTP;
			mss_l4len_idx = sizeof(struct sctphdr) <<
					E1000_ADVTXD_L4LEN_SHIFT;
			break;
#endif
		case IPPROTO_UDP:
			mss_l4len_idx = sizeof(struct udphdr) <<
					E1000_ADVTXD_L4LEN_SHIFT;
			break;
		default:
			if (unlikely(net_ratelimit())) {
				dev_warn(tx_ring->dev,
				 "partial checksum but l4 proto=%x!\n",
				 l4_hdr);
			}
			break;
		}

		/* update TX checksum flag */
		first->tx_flags |= IGB_TX_FLAGS_CSUM;
	}

	vlan_macip_lens |= skb_network_offset(skb) << E1000_ADVTXD_MACLEN_SHIFT;
#endif
	vlan_macip_lens |= first->tx_flags & IGB_TX_FLAGS_VLAN_MASK;
	
	igb_tx_ctxtdesc(tx_ring, vlan_macip_lens, type_tucmd, mss_l4len_idx);
}

static __le32 igb_tx_cmd_type(u32 tx_flags)
{
	/* set type for advanced descriptor with frame checksum insertion */
	__le32 cmd_type = cpu_to_le32(E1000_ADVTXD_DTYP_DATA |
				      E1000_ADVTXD_DCMD_IFCS |
				      E1000_ADVTXD_DCMD_DEXT);

	/* set HW vlan bit if vlan is present */
	if (tx_flags & IGB_TX_FLAGS_VLAN)
		cmd_type |= cpu_to_le32(E1000_ADVTXD_DCMD_VLE);

	/* set timestamp bit if present */
	if (tx_flags & IGB_TX_FLAGS_TSTAMP)
		cmd_type |= cpu_to_le32(E1000_ADVTXD_MAC_TSTAMP);

	/* set segmentation bits for TSO */
	if (tx_flags & IGB_TX_FLAGS_TSO)
		cmd_type |= cpu_to_le32(E1000_ADVTXD_DCMD_TSE);

	return cmd_type;
}

static void igb_tx_olinfo_status(struct igb_ring *tx_ring,
				 union e1000_adv_tx_desc *tx_desc,
				 u32 tx_flags, unsigned int paylen)
{
	u32 olinfo_status = paylen << E1000_ADVTXD_PAYLEN_SHIFT;

	/* 82575 requires a unique index per ring if any offload is enabled */
	if ((tx_flags & (IGB_TX_FLAGS_CSUM | IGB_TX_FLAGS_VLAN)) &&
	    test_bit(IGB_RING_FLAG_TX_CTX_IDX, &tx_ring->flags))
		olinfo_status |= tx_ring->reg_idx << 4;

	/* insert L4 checksum */
	if (tx_flags & IGB_TX_FLAGS_CSUM) {
		olinfo_status |= E1000_TXD_POPTS_TXSM << 8;

		/* insert IPv4 checksum */
		if (tx_flags & IGB_TX_FLAGS_IPV4)
			olinfo_status |= E1000_TXD_POPTS_IXSM << 8;
	}

	tx_desc->read.olinfo_status = cpu_to_le32(olinfo_status);
}

/*
 * The largest size we can write to the descriptor is 65535.  In order to
 * maintain a power of two alignment we have to limit ourselves to 32K.
 */
#define IGB_MAX_TXD_PWR	15
#define IGB_MAX_DATA_PER_TXD	(1<<IGB_MAX_TXD_PWR)

static void igb_tx_map(struct igb_ring *tx_ring,
		       struct igb_tx_buffer *first,
		       const u8 hdr_len)
{
	struct sk_buff *skb = first->skb;
	struct igb_tx_buffer *tx_buffer;
	union e1000_adv_tx_desc *tx_desc;
	dma_addr_t dma;
#ifdef __APPLE__
	unsigned int size;
	unsigned int paylen = mbuf_pkthdr_len(skb);
	UInt32 k,count;
	struct IOPhysicalSegment vec[MAX_SKB_FRAGS];
	count = tx_ring->netdev->txCursor()->getPhysicalSegmentsWithCoalesce(skb, vec, MAX_SKB_FRAGS);
	if(count == 0)
		return;

#else	// __APPLE__
#ifdef MAX_SKB_FRAGS
	struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[0];
	unsigned int data_len = skb->data_len;
#endif
	unsigned int size = skb_headlen(skb);
	unsigned int paylen = skb->len - hdr_len;
#endif	// __APPLE__
	__le32 cmd_type;
	u32 tx_flags = first->tx_flags;
	u16 i = tx_ring->next_to_use;

	tx_desc = IGB_TX_DESC(tx_ring, i);

	igb_tx_olinfo_status(tx_ring, tx_desc, tx_flags, paylen);
	cmd_type = igb_tx_cmd_type(tx_flags);

#ifdef __APPLE__
	dma = vec[0].location;
	size = vec[0].length;
	k = 1;
#else
	dma = dma_map_single(tx_ring->dev, skb->data, size, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_ring->dev, dma))
		goto dma_error;
#endif

	/* record length, and DMA address */
	dma_unmap_len_set(first, len, size);
	dma_unmap_addr_set(first, dma, dma);
	tx_desc->read.buffer_addr = cpu_to_le64(dma);

#ifdef MAX_SKB_FRAGS
	for (;;) {
#endif
		while (unlikely(size > IGB_MAX_DATA_PER_TXD)) {
			tx_desc->read.cmd_type_len =
				cmd_type | cpu_to_le32(IGB_MAX_DATA_PER_TXD);

			i++;
			tx_desc++;
			if (i == tx_ring->count) {
				tx_desc = IGB_TX_DESC(tx_ring, 0);
				i = 0;
			}

			dma += IGB_MAX_DATA_PER_TXD;
			size -= IGB_MAX_DATA_PER_TXD;

			tx_desc->read.olinfo_status = 0;
			tx_desc->read.buffer_addr = cpu_to_le64(dma);
		}

#ifdef MAX_SKB_FRAGS
	#ifdef __APPLE__
		if(k >= count)
			break;
	#else
		if (likely(!data_len))
			break;
	#endif

		tx_desc->read.cmd_type_len = cmd_type | cpu_to_le32(size);

		i++;
		tx_desc++;
		if (i == tx_ring->count) {
			tx_desc = IGB_TX_DESC(tx_ring, 0);
			i = 0;
		}

	#ifdef __APPLE__
		dma = vec[k].location;
		size = vec[k].length;
		k++;
	#else
		size = skb_frag_size(frag);
		data_len -= size;

		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, size,
							   DMA_TO_DEVICE);
		if (dma_mapping_error(tx_ring->dev, dma))
			goto dma_error;
	#endif

		tx_buffer = &tx_ring->tx_buffer_info[i];
		dma_unmap_len_set(tx_buffer, len, size);
		dma_unmap_addr_set(tx_buffer, dma, dma);
		
		tx_desc->read.olinfo_status = 0;
		tx_desc->read.buffer_addr = cpu_to_le64(dma);

	#ifndef __APPLE__
		frag++;
	#endif
	}

#endif /* MAX_SKB_FRAGS */
#ifdef CONFIG_BQL
	netdev_tx_sent_queue(txring_txq(tx_ring), first->bytecount);
#endif /* CONFIG_BQL */
	
	/* write last descriptor with RS and EOP bits */
	cmd_type |= cpu_to_le32(size) | cpu_to_le32(IGB_TXD_DCMD);
	tx_desc->read.cmd_type_len = cmd_type;

	/* set the timestamp */
	first->time_stamp = jiffies;

	/*
	 * Force memory writes to complete before letting h/w know there
	 * are new descriptors to fetch.  (Only applicable for weak-ordered
	 * memory model archs, such as IA-64).
	 *
	 * We also need this memory barrier to make certain all of the
	 * status bits have been updated before next_to_watch is written.
	 */
	wmb();

	/* set next_to_watch value indicating a packet is present */
	first->next_to_watch = tx_desc;

	i++;
	if (i == tx_ring->count)
		i = 0;

	tx_ring->next_to_use = i;

	writel(i, tx_ring->tail);

	/* we need this if more than one processor can write to our tail
	 * at a time, it syncronizes IO on IA64/Altix systems */
	mmiowb();

#ifndef __APPLE__
	return;

dma_error:
	IOLog( "TX DMA map failed\n");

	/* clear dma mappings for failed tx_buffer_info map */
	for (;;) {
		tx_buffer = &tx_ring->tx_buffer_info[i];
		igb_unmap_and_free_tx_resource(adapter, tx_ring, tx_buffer);
		if (tx_buffer == first)
			break;
		if (i == 0)
			i = tx_ring->count;
		i--;
	}

	tx_ring->next_to_use = i;
#endif
}

static int __igb_maybe_stop_tx(struct igb_ring *tx_ring, const u16 size)
{
	IOEthernetController *netdev = netdev_ring(tx_ring);

#ifndef __APPLE__
	if (netif_is_multiqueue(netdev))
		netif_stop_subqueue(netdev, ring_queue_index(tx_ring));
	else
#endif
		netif_stop_queue(netdev);

	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it. */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (igb_desc_unused(tx_ring) < size)
		return -EBUSY;

	/* A reprieve! */
#ifndef __APPLE__
	if (netif_is_multiqueue(netdev))
		netif_wake_subqueue(netdev, ring_queue_index(tx_ring));
	else
#endif
		netif_wake_queue(netdev);

	tx_ring->tx_stats.restart_queue++;
	return 0;
}

static inline int igb_maybe_stop_tx(struct igb_ring *tx_ring, const u16 size)
{
	if (igb_desc_unused(tx_ring) >= size)
		return 0;
	return __igb_maybe_stop_tx(tx_ring, size);
}

#ifndef __APPLE__	// see outputPacket()
netdev_tx_t igb_xmit_frame_ring(struct igb_adapter *adapter,struct sk_buff *skb,
				struct igb_ring *tx_ring)
{
	struct igb_tx_buffer *first;
	int tso;
	u32 tx_flags = 0;
	__be16 protocol = vlan_get_protocol(skb);
	u8 hdr_len = 0;
	/* need: 1 descriptor per page,
	 *       + 2 desc gap to keep tail from touching head,
	 *       + 1 desc for skb->data,
	 *       + 1 desc for context descriptor,
	 * otherwise try next time */
	if (igb_maybe_stop_tx(tx_ring, skb_shinfo(skb)->nr_frags + 4)) {
		/* this is a hard error */
		return NETDEV_TX_BUSY;
	}
	/* record the location of the first descriptor for this packet */
	first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];
	first->skb = skb;
	first->bytecount = mbuf_pkthdr_len(skb);
	first->gso_segs = 1;

#ifdef HAVE_HW_TIME_STAMP
	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		tx_flags |= IGB_TX_FLAGS_TSTAMP;
	}
#endif
	if (vlan_tx_tag_present(skb)) {
		tx_flags |= IGB_TX_FLAGS_VLAN;
		tx_flags |= (vlan_tx_tag_get(skb) << IGB_TX_FLAGS_VLAN_SHIFT);
	}

	/* record initial flags and protocol */
	first->tx_flags = tx_flags;
	first->protocol = protocol;
	
	tso = igb_tso(tx_ring, first, &hdr_len);
	if (tso < 0)
		goto out_drop;
	else if (!tso)
		igb_tx_csum(tx_ring, first);

	igb_tx_map(tx_ring, first, hdr_len);

#ifndef HAVE_TRANS_START_IN_QUEUE
	//netdev_ring(tx_ring)->trans_start = jiffies;

#endif
	/* Make sure there is space in the ring for the next send. */
	igb_maybe_stop_tx(tx_ring, MAX_SKB_FRAGS + 4);

	return NETDEV_TX_OK;

out_drop:
	igb_unmap_and_free_tx_resource(adapter, tx_ring, first);

	return NETDEV_TX_OK;
}
#endif

#ifdef HAVE_TX_MQ
static inline struct igb_ring *igb_tx_queue_mapping(struct igb_adapter *adapter,
                                                    struct sk_buff *skb)
{
	unsigned int r_idx = skb->queue_mapping;

	if (r_idx >= adapter->num_tx_queues)
		r_idx = r_idx % adapter->num_tx_queues;

	return adapter->tx_ring[r_idx];
}
#else
#define igb_tx_queue_mapping(_adapter, _skb) (_adapter)->tx_ring[0]
#endif

#ifndef __APPLE__	// see outputPacket()
static netdev_tx_t igb_xmit_frame(struct sk_buff *skb,
                                  IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	if (test_bit(__IGB_DOWN, &adapter->state)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (skb->len <= 0) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/*
	 * The minimum packet size with TCTL.PSP set is 17 so pad the skb
	 * in order to meet this minimum size requirement.
	 */
	if (skb->len < 17) {
		if (skb_padto(skb, 17))
			return NETDEV_TX_OK;
		skb->len = 17;
	}

	return igb_xmit_frame_ring(skb, igb_tx_queue_mapping(adapter, skb));
}
#endif

/**
 * igb_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
static void igb_tx_timeout(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	/* Do the reset outside of interrupt context */
	adapter->tx_timeout_count++;

	if (hw->mac.type >= e1000_82580)
		hw->dev_spec._82575.global_device_reset = true;

	schedule_work(&adapter->reset_task);
	E1000_WRITE_REG(hw, E1000_EICS,
			(adapter->eims_enable_mask & ~adapter->eims_other));
}

#ifndef	__APPLE__
static void igb_reset_task(struct work_struct *work)
{
	struct igb_adapter *adapter;
	adapter = container_of(work, struct igb_adapter, reset_task);

	igb_reinit_locked(adapter);
}
#endif

/**
 * igb_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are updated here and also from the timer callback.
 **/
static struct net_device_stats *igb_get_stats(IOEthernetController *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	
	if (!test_bit(__IGB_RESETTING, &adapter->state))
		igb_update_stats(adapter);
	
#ifdef HAVE_NETDEV_STATS_IN_NETDEV
	/* only return the current stats */
	return &netdev->stats;
#else
	/* only return the current stats */
	return &adapter->net_stats;
#endif /* HAVE_NETDEV_STATS_IN_NETDEV */
}

/**
 * igb_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int igb_change_mtu(IOEthernetController *netdev, int new_mtu)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
	u32 rx_buffer_len, i;
#endif

	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		usleep_range(1000, 2000);

	/* igb_down has a dependency on max_frame_size */
	adapter->max_frame_size = max_frame;

#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
#ifdef IGB_PER_PKT_TIMESTAMP
	if (adapter->hw.mac.type >= e1000_82580)
		max_frame += IGB_TS_HDR_LEN;

#endif
	/*
	 * RLPML prevents us from receiving a frame larger than max_frame so
	 * it is safe to just set the rx_buffer_len to max_frame without the
	 * risk of an skb over panic.
	 */
	if (max_frame <= MAXIMUM_ETHERNET_VLAN_SIZE)
		rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;
	else
		rx_buffer_len = max_frame;

#endif
	if (netif_running(netdev))
		igb_down(adapter);


#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
	for (i = 0; i < adapter->num_rx_queues; i++)
		adapter->rx_ring[i]->rx_buffer_len = rx_buffer_len;

#endif
	if (netif_running(netdev))
		igb_up(adapter);
	else
		igb_reset(adapter);

	clear_bit(__IGB_RESETTING, &adapter->state);

	return 0;
}

/**
 * igb_update_stats - Update the board statistics counters
 * @adapter: board private structure
 **/

void igb_update_stats(struct igb_adapter *adapter)
{
    IONetworkStats * net_stats = adapter->netdev->getNetStats();
    IOEthernetStats * ether_stats = adapter->netdev->getEtherStats();
	struct e1000_hw *hw = &adapter->hw;
#ifdef HAVE_PCI_ERS
	IOPCIDevice *pdev = adapter->pdev;
#endif
	u32 reg, mpc;
	u16 phy_tmp;
	int i;
	u64 bytes, packets;

#define PHY_IDLE_ERROR_COUNT_MASK 0x00FF

	/*
	 * Prevent stats update while adapter is being reset, or if the pci
	 * connection is down.
	 */
	if (adapter->link_speed == 0)
		return;
#ifdef HAVE_PCI_ERS
	if (pci_channel_offline(pdev))
		return;

#endif
	bytes = 0;
	packets = 0;
	for (i = 0; i < adapter->num_rx_queues; i++) {
		u32 rqdpc_tmp = E1000_READ_REG(hw, E1000_RQDPC(i)) & 0x0FFF;
		struct igb_ring *ring = adapter->rx_ring[i];
		ring->rx_stats.drops += rqdpc_tmp;
		ether_stats->dot3RxExtraEntry.overruns += rqdpc_tmp;

		bytes += ring->rx_stats.bytes;
		packets += ring->rx_stats.packets;
	}

	//net_stats->rx_bytes = bytes;
	net_stats->inputPackets = packets;

	bytes = 0;
	packets = 0;
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igb_ring *ring = adapter->tx_ring[i];
		bytes += ring->tx_stats.bytes;
		packets += ring->tx_stats.packets;
	}
	//net_stats->tx_bytes = bytes;
	net_stats->outputPackets = packets;

	/* read stats registers */
	adapter->stats.crcerrs += E1000_READ_REG(hw, E1000_CRCERRS);
	adapter->stats.gprc += E1000_READ_REG(hw, E1000_GPRC);
	adapter->stats.gorc += E1000_READ_REG(hw, E1000_GORCL);
	E1000_READ_REG(hw, E1000_GORCH); /* clear GORCL */
	adapter->stats.bprc += E1000_READ_REG(hw, E1000_BPRC);
	adapter->stats.mprc += E1000_READ_REG(hw, E1000_MPRC);
	adapter->stats.roc += E1000_READ_REG(hw, E1000_ROC);

	adapter->stats.prc64 += E1000_READ_REG(hw, E1000_PRC64);
	adapter->stats.prc127 += E1000_READ_REG(hw, E1000_PRC127);
	adapter->stats.prc255 += E1000_READ_REG(hw, E1000_PRC255);
	adapter->stats.prc511 += E1000_READ_REG(hw, E1000_PRC511);
	adapter->stats.prc1023 += E1000_READ_REG(hw, E1000_PRC1023);
	adapter->stats.prc1522 += E1000_READ_REG(hw, E1000_PRC1522);
	adapter->stats.symerrs += E1000_READ_REG(hw, E1000_SYMERRS);
	adapter->stats.sec += E1000_READ_REG(hw, E1000_SEC);

	mpc = E1000_READ_REG(hw, E1000_MPC);
	adapter->stats.mpc += mpc;
	ether_stats->dot3RxExtraEntry.overruns += mpc;
	adapter->stats.scc += E1000_READ_REG(hw, E1000_SCC);
	adapter->stats.ecol += E1000_READ_REG(hw, E1000_ECOL);
	adapter->stats.mcc += E1000_READ_REG(hw, E1000_MCC);
	adapter->stats.latecol += E1000_READ_REG(hw, E1000_LATECOL);
	adapter->stats.dc += E1000_READ_REG(hw, E1000_DC);
	adapter->stats.rlec += E1000_READ_REG(hw, E1000_RLEC);
	adapter->stats.xonrxc += E1000_READ_REG(hw, E1000_XONRXC);
	adapter->stats.xontxc += E1000_READ_REG(hw, E1000_XONTXC);
	adapter->stats.xoffrxc += E1000_READ_REG(hw, E1000_XOFFRXC);
	adapter->stats.xofftxc += E1000_READ_REG(hw, E1000_XOFFTXC);
	adapter->stats.fcruc += E1000_READ_REG(hw, E1000_FCRUC);
	adapter->stats.gptc += E1000_READ_REG(hw, E1000_GPTC);
	adapter->stats.gotc += E1000_READ_REG(hw, E1000_GOTCL);
	E1000_READ_REG(hw, E1000_GOTCH); /* clear GOTCL */
	adapter->stats.rnbc += E1000_READ_REG(hw, E1000_RNBC);
	adapter->stats.ruc += E1000_READ_REG(hw, E1000_RUC);
	adapter->stats.rfc += E1000_READ_REG(hw, E1000_RFC);
	adapter->stats.rjc += E1000_READ_REG(hw, E1000_RJC);
	adapter->stats.tor += E1000_READ_REG(hw, E1000_TORH);
	adapter->stats.tot += E1000_READ_REG(hw, E1000_TOTH);
	adapter->stats.tpr += E1000_READ_REG(hw, E1000_TPR);

	adapter->stats.ptc64 += E1000_READ_REG(hw, E1000_PTC64);
	adapter->stats.ptc127 += E1000_READ_REG(hw, E1000_PTC127);
	adapter->stats.ptc255 += E1000_READ_REG(hw, E1000_PTC255);
	adapter->stats.ptc511 += E1000_READ_REG(hw, E1000_PTC511);
	adapter->stats.ptc1023 += E1000_READ_REG(hw, E1000_PTC1023);
	adapter->stats.ptc1522 += E1000_READ_REG(hw, E1000_PTC1522);

	adapter->stats.mptc += E1000_READ_REG(hw, E1000_MPTC);
	adapter->stats.bptc += E1000_READ_REG(hw, E1000_BPTC);

	adapter->stats.tpt += E1000_READ_REG(hw, E1000_TPT);
	adapter->stats.colc += E1000_READ_REG(hw, E1000_COLC);

	adapter->stats.algnerrc += E1000_READ_REG(hw, E1000_ALGNERRC);
	/* read internal phy sepecific stats */
	reg = E1000_READ_REG(hw, E1000_CTRL_EXT);
	if (!(reg & E1000_CTRL_EXT_LINK_MODE_MASK)) {
		adapter->stats.rxerrc += E1000_READ_REG(hw, E1000_RXERRC);
		adapter->stats.tncrs += E1000_READ_REG(hw, E1000_TNCRS);
	}

	adapter->stats.tsctc += E1000_READ_REG(hw, E1000_TSCTC);
	adapter->stats.tsctfc += E1000_READ_REG(hw, E1000_TSCTFC);

	adapter->stats.iac += E1000_READ_REG(hw, E1000_IAC);
	adapter->stats.icrxoc += E1000_READ_REG(hw, E1000_ICRXOC);
	adapter->stats.icrxptc += E1000_READ_REG(hw, E1000_ICRXPTC);
	adapter->stats.icrxatc += E1000_READ_REG(hw, E1000_ICRXATC);
	adapter->stats.ictxptc += E1000_READ_REG(hw, E1000_ICTXPTC);
	adapter->stats.ictxatc += E1000_READ_REG(hw, E1000_ICTXATC);
	adapter->stats.ictxqec += E1000_READ_REG(hw, E1000_ICTXQEC);
	adapter->stats.ictxqmtc += E1000_READ_REG(hw, E1000_ICTXQMTC);
	adapter->stats.icrxdmtc += E1000_READ_REG(hw, E1000_ICRXDMTC);

	/* Fill out the OS statistics structure */
	//net_stats->multicast = adapter->stats.mprc;
	net_stats->collisions = adapter->stats.colc;

	/* Rx Errors */

	/* RLEC on some newer hardware can be incorrect so build
	 * our own version based on RUC and ROC */
	net_stats->inputErrors = adapter->stats.rxerrc +
		adapter->stats.crcerrs + adapter->stats.algnerrc +
		adapter->stats.ruc + adapter->stats.roc +
		adapter->stats.cexterr;
	ether_stats->dot3StatsEntry.frameTooLongs = adapter->stats.roc;
	ether_stats->dot3RxExtraEntry.frameTooShorts = adapter->stats.ruc;
	ether_stats->dot3StatsEntry.fcsErrors = adapter->stats.crcerrs;
	ether_stats->dot3StatsEntry.alignmentErrors = adapter->stats.algnerrc;
	ether_stats->dot3StatsEntry.missedFrames = adapter->stats.mpc;

	/* Tx Errors */
	net_stats->outputErrors = adapter->stats.ecol +
			       adapter->stats.latecol;
	ether_stats->dot3StatsEntry.deferredTransmissions = adapter->stats.ecol;
	ether_stats->dot3StatsEntry.lateCollisions = adapter->stats.latecol;
	ether_stats->dot3StatsEntry.carrierSenseErrors = adapter->stats.tncrs;

	/* Tx Dropped needs to be maintained elsewhere */

	/* Phy Stats */
	if (hw->phy.media_type == e1000_media_type_copper) {
		if ((adapter->link_speed == SPEED_1000) &&
		   (!e1000_read_phy_reg(hw, PHY_1000T_STATUS, &phy_tmp))) {
			phy_tmp &= PHY_IDLE_ERROR_COUNT_MASK;
			adapter->phy_stats.idle_errors += phy_tmp;
		}
	}

	/* Management Stats */
	adapter->stats.mgptc += E1000_READ_REG(hw, E1000_MGTPTC);
	adapter->stats.mgprc += E1000_READ_REG(hw, E1000_MGTPRC);
	if (hw->mac.type > e1000_82580) {
		adapter->stats.o2bgptc += E1000_READ_REG(hw, E1000_O2BGPTC);
		adapter->stats.o2bspc += E1000_READ_REG(hw, E1000_O2BSPC);
		adapter->stats.b2ospc += E1000_READ_REG(hw, E1000_B2OSPC);
		adapter->stats.b2ogprc += E1000_READ_REG(hw, E1000_B2OGPRC);
	}
}

#ifndef __APPLE__
static irqreturn_t igb_msix_other(int irq, void *data)
{
	struct igb_adapter *adapter = data;
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = E1000_READ_REG(hw, E1000_ICR);
	/* reading ICR causes bit 31 of EICR to be cleared */

	if (icr & E1000_ICR_DRSTA)
		schedule_work(&adapter->reset_task);

	if (icr & E1000_ICR_DOUTSYNC) {
		/* HW is reporting DMA is out of sync */
		adapter->stats.doosync++;
		/* The DMA Out of Sync is also indication of a spoof event
		 * in IOV mode. Check the Wrong VM Behavior register to
		 * see if it is really a spoof event. */
		igb_check_wvbr(adapter);
	}

	/* Check for a mailbox event */
	if (icr & E1000_ICR_VMMB)
		igb_msg_task(adapter);

	if (icr & E1000_ICR_LSC) {
		hw->mac.get_link_status = 1;
		/* guard against interrupt when we're going down */
		if (!test_bit(__IGB_DOWN, &adapter->state))
			mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

	/* Check for MDD event */
	if (icr & E1000_ICR_MDDET)
		igb_process_mdd_event(adapter);

	E1000_WRITE_REG(hw, E1000_EIMS, adapter->eims_other);

	return IRQ_HANDLED;
}
#endif

static void igb_write_itr(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	u32 itr_val = q_vector->itr_val & 0x7FFC;

	if (!q_vector->set_itr)
		return;

	if (!itr_val)
		itr_val = 0x4;

	if (adapter->hw.mac.type == e1000_82575)
		itr_val |= itr_val << 16;
	else
		itr_val |= E1000_EITR_CNT_IGNR;

	writel(itr_val, q_vector->itr_register);
	q_vector->set_itr = 0;
}

#ifndef __APPLE__
static irqreturn_t igb_msix_ring(int irq, void *data)
{
	struct igb_q_vector *q_vector = data;

	/* Write the ITR value calculated from the previous interrupt. */
	igb_write_itr(q_vector);

	napi_schedule(&q_vector->napi);

	return IRQ_HANDLED;
}
#endif

#ifdef IGB_DCA
static void igb_update_dca(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;
	int cpu = get_cpu();

	if (q_vector->cpu == cpu)
		goto out_no_update;

	if (q_vector->tx.ring) {
		int q = q_vector->tx.ring->reg_idx;
		u32 dca_txctrl = E1000_READ_REG(hw, E1000_DCA_TXCTRL(q));
		if (hw->mac.type == e1000_82575) {
			dca_txctrl &= ~E1000_DCA_TXCTRL_CPUID_MASK;
			dca_txctrl |= dca3_get_tag(&adapter->pdev->dev, cpu);
		} else {
			dca_txctrl &= ~E1000_DCA_TXCTRL_CPUID_MASK_82576;
			dca_txctrl |= dca3_get_tag(&adapter->pdev->dev, cpu) <<
			              E1000_DCA_TXCTRL_CPUID_SHIFT_82576;
		}
		dca_txctrl |= E1000_DCA_TXCTRL_DESC_DCA_EN;
		E1000_WRITE_REG(hw, E1000_DCA_TXCTRL(q), dca_txctrl);
	}
	if (q_vector->rx.ring) {
		int q = q_vector->rx.ring->reg_idx;
		u32 dca_rxctrl = E1000_READ_REG(hw, E1000_DCA_RXCTRL(q));
		if (hw->mac.type == e1000_82575) {
			dca_rxctrl &= ~E1000_DCA_RXCTRL_CPUID_MASK;
			dca_rxctrl |= dca3_get_tag(&adapter->pdev->dev, cpu);
		} else {
			dca_rxctrl &= ~E1000_DCA_RXCTRL_CPUID_MASK_82576;
			dca_rxctrl |= dca3_get_tag(&adapter->pdev->dev, cpu) <<
			              E1000_DCA_RXCTRL_CPUID_SHIFT_82576;
		}
		dca_rxctrl |= E1000_DCA_RXCTRL_DESC_DCA_EN;
		dca_rxctrl |= E1000_DCA_RXCTRL_HEAD_DCA_EN;
		dca_rxctrl |= E1000_DCA_RXCTRL_DATA_DCA_EN;
		E1000_WRITE_REG(hw, E1000_DCA_RXCTRL(q), dca_rxctrl);
	}
	q_vector->cpu = cpu;
out_no_update:
	put_cpu();
}

static void igb_setup_dca(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;

	if (!(adapter->flags & IGB_FLAG_DCA_ENABLED))
		return;

	/* Always use CB2 mode, difference is masked in the CB driver. */
	E1000_WRITE_REG(hw, E1000_DCA_CTRL, E1000_DCA_CTRL_DCA_MODE_CB2);

	for (i = 0; i < adapter->num_q_vectors; i++) {
		adapter->q_vector[i]->cpu = -1;
		igb_update_dca(adapter->q_vector[i]);
	}
}

static int __igb_notify_dca(struct device *dev, void *data)
{
	IOEthernetController *netdev = dev_get_drvdata(dev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	IOPCIDevice *pdev = adapter->pdev;
	struct e1000_hw *hw = &adapter->hw;
	unsigned long event = *(unsigned long *)data;

	switch (event) {
	case DCA_PROVIDER_ADD:
		/* if already enabled, don't do it again */
		if (adapter->flags & IGB_FLAG_DCA_ENABLED)
			break;
		if (dca_add_requester(dev) == E1000_SUCCESS) {
			adapter->flags |= IGB_FLAG_DCA_ENABLED;
			dev_info(pci_dev_to_dev(pdev), "DCA enabled\n");
			igb_setup_dca(adapter);
			break;
		}
		/* Fall Through since DCA is disabled. */
	case DCA_PROVIDER_REMOVE:
		if (adapter->flags & IGB_FLAG_DCA_ENABLED) {
			/* without this a class_device is left
			 * hanging around in the sysfs model */
			dca_remove_requester(dev);
			dev_info(pci_dev_to_dev(pdev), "DCA disabled\n");
			adapter->flags &= ~IGB_FLAG_DCA_ENABLED;
			E1000_WRITE_REG(hw, E1000_DCA_CTRL, E1000_DCA_CTRL_DCA_DISABLE);
		}
		break;
	}

	return E1000_SUCCESS;
}

static int igb_notify_dca(struct notifier_block *nb, unsigned long event,
                          void *p)
{
	int ret_val;

	ret_val = driver_for_each_device(&igb_driver.driver, NULL, &event,
	                                 __igb_notify_dca);

	return ret_val ? NOTIFY_BAD : NOTIFY_DONE;
}
#endif /* IGB_DCA */

	static int igb_vf_configure(struct igb_adapter *adapter, int vf)
	{
		unsigned char mac_addr[ETH_ALEN];
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
		struct pci_dev *pdev = adapter->pdev;
		struct e1000_hw *hw = &adapter->hw;
		struct pci_dev *pvfdev;
		unsigned int device_id;
		u16 thisvf_devfn;
#endif
		
		random_ether_addr(mac_addr);
		igb_set_vf_mac(adapter, vf, mac_addr);
		
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
		switch (adapter->hw.mac.type) {
			case e1000_82576:
				device_id = IGB_82576_VF_DEV_ID;
				/* VF Stride for 82576 is 2 */
				thisvf_devfn = (pdev->devfn + 0x80 + (vf << 1)) |
				(pdev->devfn & 1);
				break;
			case e1000_i350:
				device_id = IGB_I350_VF_DEV_ID;
				/* VF Stride for I350 is 4 */
				thisvf_devfn = (pdev->devfn + 0x80 + (vf << 2)) |
				(pdev->devfn & 3);
				break;
			default:
				device_id = 0;
				thisvf_devfn = 0;
				break;
		}
		
		pvfdev = pci_get_device(hw->vendor_id, device_id, NULL);
		while (pvfdev) {
			if (pvfdev->devfn == thisvf_devfn)
				break;
			pvfdev = pci_get_device(hw->vendor_id,
									device_id, pvfdev);
		}
		
		if (pvfdev)
			adapter->vf_data[vf].vfdev = pvfdev;
		else
			dev_err(&pdev->dev,
					"Couldn't find pci dev ptr for VF %4.4x\n",
					thisvf_devfn);
		return pvfdev != NULL;
#else
		return true;
#endif
	}
	
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
	static int igb_find_enabled_vfs(struct igb_adapter *adapter)
	{
		struct e1000_hw *hw = &adapter->hw;
		struct pci_dev *pdev = adapter->pdev;
		struct pci_dev *pvfdev;
		u16 vf_devfn = 0;
		u16 vf_stride;
		unsigned int device_id;
		int vfs_found = 0;
		
		switch (adapter->hw.mac.type) {
			case e1000_82576:
				device_id = IGB_82576_VF_DEV_ID;
				/* VF Stride for 82576 is 2 */
				vf_stride = 2;
				break;
			case e1000_i350:
				device_id = IGB_I350_VF_DEV_ID;
				/* VF Stride for I350 is 4 */
				vf_stride = 4;
				break;
			default:
				device_id = 0;
				vf_stride = 0;
				break;
		}
		
		vf_devfn = pdev->devfn + 0x80;
		pvfdev = pci_get_device(hw->vendor_id, device_id, NULL);
		while (pvfdev) {
			if (pvfdev->devfn == vf_devfn)
				vfs_found++;
			vf_devfn += vf_stride;
			pvfdev = pci_get_device(hw->vendor_id,
									device_id, pvfdev);
		}
		
		return vfs_found;
	}
#endif
	
	static int igb_check_vf_assignment(struct igb_adapter *adapter)
	{
#ifdef HAVE_PCI_DEV_FLAGS_ASSIGNED
		int i;
		for (i = 0; i < adapter->vfs_allocated_count; i++) {
			if (adapter->vf_data[i].vfdev) {
				if (adapter->vf_data[i].vfdev->dev_flags &
					PCI_DEV_FLAGS_ASSIGNED)
					return true;
			}
		}
#endif
		return false;
	}
	
static void igb_ping_all_vfs(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ping;
	int i;

	for (i = 0 ; i < adapter->vfs_allocated_count; i++) {
		ping = E1000_PF_CONTROL_MSG;
		if (adapter->vf_data[i].flags & IGB_VF_FLAG_CTS)
			ping |= E1000_VT_MSGTYPE_CTS;
		e1000_write_mbx(hw, &ping, 1, i);
	}
}

/**
 *  igb_mta_set_ - Set multicast filter table address
 *  @adapter: pointer to the adapter structure
 *  @hash_value: determines the MTA register and bit to set
 *
 *  The multicast table address is a register array of 32-bit registers.
 *  The hash_value is used to determine what register the bit is in, the
 *  current value is read, the new bit is OR'd in and the new value is
 *  written back into the register.
 **/
void igb_mta_set(struct igb_adapter *adapter, u32 hash_value)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 hash_bit, hash_reg, mta;

	/*
	 * The MTA is a register array of 32-bit registers. It is
	 * treated like an array of (32*mta_reg_count) bits.  We want to
	 * set bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The (hw->mac.mta_reg_count - 1) serves as a
	 * mask to bits 31:5 of the hash value which gives us the
	 * register we're modifying.  The hash bit within that register
	 * is determined by the lower 5 bits of the hash value.
	 */
	hash_reg = (hash_value >> 5) & (hw->mac.mta_reg_count - 1);
	hash_bit = hash_value & 0x1F;

	mta = E1000_READ_REG_ARRAY(hw, E1000_MTA, hash_reg);

	mta |= (1 << hash_bit);

	E1000_WRITE_REG_ARRAY(hw, E1000_MTA, hash_reg, mta);
	E1000_WRITE_FLUSH(hw);
}

static int igb_set_vf_promisc(struct igb_adapter *adapter, u32 *msgbuf, u32 vf)
{

	struct e1000_hw *hw = &adapter->hw;
	u32 vmolr = E1000_READ_REG(hw, E1000_VMOLR(vf));
	struct vf_data_storage *vf_data = &adapter->vf_data[vf];

	vf_data->flags &= ~(IGB_VF_FLAG_UNI_PROMISC |
	                    IGB_VF_FLAG_MULTI_PROMISC);
	vmolr &= ~(E1000_VMOLR_ROPE | E1000_VMOLR_ROMPE | E1000_VMOLR_MPME);

#ifdef IGB_ENABLE_VF_PROMISC
	if (*msgbuf & E1000_VF_SET_PROMISC_UNICAST) {
		vmolr |= E1000_VMOLR_ROPE;
		vf_data->flags |= IGB_VF_FLAG_UNI_PROMISC;
		*msgbuf &= ~E1000_VF_SET_PROMISC_UNICAST;
	}
#endif
	if (*msgbuf & E1000_VF_SET_PROMISC_MULTICAST) {
		vmolr |= E1000_VMOLR_MPME;
		vf_data->flags |= IGB_VF_FLAG_MULTI_PROMISC;
		*msgbuf &= ~E1000_VF_SET_PROMISC_MULTICAST;
	} else {
		/*
		 * if we have hashes and we are clearing a multicast promisc
		 * flag we need to write the hashes to the MTA as this step
		 * was previously skipped
		 */
		if (vf_data->num_vf_mc_hashes > 30) {
			vmolr |= E1000_VMOLR_MPME;
		} else if (vf_data->num_vf_mc_hashes) {
			int j;
			vmolr |= E1000_VMOLR_ROMPE;
			for (j = 0; j < vf_data->num_vf_mc_hashes; j++)
				igb_mta_set(adapter, vf_data->vf_mc_hashes[j]);
		}
	}

	E1000_WRITE_REG(hw, E1000_VMOLR(vf), vmolr);

	/* there are flags left unprocessed, likely not supported */
	if (*msgbuf & E1000_VT_MSGINFO_MASK)
		return -EINVAL;

	return 0;

}

static int igb_set_vf_multicasts(struct igb_adapter *adapter,
				  u32 *msgbuf, u32 vf)
{
	int n = (msgbuf[0] & E1000_VT_MSGINFO_MASK) >> E1000_VT_MSGINFO_SHIFT;
	u16 *hash_list = (u16 *)&msgbuf[1];
	struct vf_data_storage *vf_data = &adapter->vf_data[vf];
	int i;

	/* salt away the number of multicast addresses assigned
	 * to this VF for later use to restore when the PF multi cast
	 * list changes
	 */
	vf_data->num_vf_mc_hashes = n;

	/* only up to 30 hash values supported */
	if (n > 30)
		n = 30;

	/* store the hashes for later use */
	for (i = 0; i < n; i++)
		vf_data->vf_mc_hashes[i] = hash_list[i];

	/* Flush and reset the mta with the new values */
	igb_set_rx_mode(adapter->netdev);

	return 0;
}

static void igb_restore_vf_multicasts(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct vf_data_storage *vf_data;
	int i, j;

	for (i = 0; i < adapter->vfs_allocated_count; i++) {
		u32 vmolr = E1000_READ_REG(hw, E1000_VMOLR(i));
		vmolr &= ~(E1000_VMOLR_ROMPE | E1000_VMOLR_MPME);

		vf_data = &adapter->vf_data[i];

		if ((vf_data->num_vf_mc_hashes > 30) ||
		    (vf_data->flags & IGB_VF_FLAG_MULTI_PROMISC)) {
			vmolr |= E1000_VMOLR_MPME;
		} else if (vf_data->num_vf_mc_hashes) {
			vmolr |= E1000_VMOLR_ROMPE;
			for (j = 0; j < vf_data->num_vf_mc_hashes; j++)
				igb_mta_set(adapter, vf_data->vf_mc_hashes[j]);
		}
		E1000_WRITE_REG(hw, E1000_VMOLR(i), vmolr);
	}
}

static void igb_clear_vf_vfta(struct igb_adapter *adapter, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 pool_mask, reg, vid;
	u16 vlan_default;
	int i;

	pool_mask = 1 << (E1000_VLVF_POOLSEL_SHIFT + vf);

	/* Find the vlan filter for this id */
	for (i = 0; i < E1000_VLVF_ARRAY_SIZE; i++) {
		reg = E1000_READ_REG(hw, E1000_VLVF(i));

		/* remove the vf from the pool */
		reg &= ~pool_mask;

		/* if pool is empty then remove entry from vfta */
		if (!(reg & E1000_VLVF_POOLSEL_MASK) &&
		    (reg & E1000_VLVF_VLANID_ENABLE)) {
			reg = 0;
			vid = reg & E1000_VLVF_VLANID_MASK;
			igb_vfta_set(adapter, vid, FALSE);
		}

		E1000_WRITE_REG(hw, E1000_VLVF(i), reg);
	}

	adapter->vf_data[vf].vlans_enabled = 0;

	vlan_default = adapter->vf_data[vf].default_vf_vlan_id;
	if (vlan_default)
		igb_vlvf_set(adapter, vlan_default, true, vf);
}

s32 igb_vlvf_set(struct igb_adapter *adapter, u32 vid, bool add, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 reg, i;

	/* The vlvf table only exists on 82576 hardware and newer */
	if (hw->mac.type < e1000_82576)
		return -1;

	/* we only need to do this if VMDq is enabled */
	if (!adapter->vmdq_pools)
		return -1;

	/* Find the vlan filter for this id */
	for (i = 0; i < E1000_VLVF_ARRAY_SIZE; i++) {
		reg = E1000_READ_REG(hw, E1000_VLVF(i));
		if ((reg & E1000_VLVF_VLANID_ENABLE) &&
		    vid == (reg & E1000_VLVF_VLANID_MASK))
			break;
	}

	if (add) {
		if (i == E1000_VLVF_ARRAY_SIZE) {
			/* Did not find a matching VLAN ID entry that was
			 * enabled.  Search for a free filter entry, i.e.
			 * one without the enable bit set
			 */
			for (i = 0; i < E1000_VLVF_ARRAY_SIZE; i++) {
				reg = E1000_READ_REG(hw, E1000_VLVF(i));
				if (!(reg & E1000_VLVF_VLANID_ENABLE))
					break;
			}
		}
		if (i < E1000_VLVF_ARRAY_SIZE) {
			/* Found an enabled/available entry */
			reg |= 1 << (E1000_VLVF_POOLSEL_SHIFT + vf);

			/* if !enabled we need to set this up in vfta */
			if (!(reg & E1000_VLVF_VLANID_ENABLE)) {
				/* add VID to filter table */
				igb_vfta_set(adapter, vid, TRUE);
				reg |= E1000_VLVF_VLANID_ENABLE;
			}
			reg &= ~E1000_VLVF_VLANID_MASK;
			reg |= vid;
			E1000_WRITE_REG(hw, E1000_VLVF(i), reg);

			/* do not modify RLPML for PF devices */
			if (vf >= adapter->vfs_allocated_count)
				return E1000_SUCCESS;

			if (!adapter->vf_data[vf].vlans_enabled) {
				u32 size;
				reg = E1000_READ_REG(hw, E1000_VMOLR(vf));
				size = reg & E1000_VMOLR_RLPML_MASK;
				size += 4;
				reg &= ~E1000_VMOLR_RLPML_MASK;
				reg |= size;
				E1000_WRITE_REG(hw, E1000_VMOLR(vf), reg);
			}

			adapter->vf_data[vf].vlans_enabled++;
		}
	} else {
		if (i < E1000_VLVF_ARRAY_SIZE) {
			/* remove vf from the pool */
			reg &= ~(1 << (E1000_VLVF_POOLSEL_SHIFT + vf));
			/* if pool is empty then remove entry from vfta */
			if (!(reg & E1000_VLVF_POOLSEL_MASK)) {
				reg = 0;
				igb_vfta_set(adapter, vid, FALSE);
			}
			E1000_WRITE_REG(hw, E1000_VLVF(i), reg);

			/* do not modify RLPML for PF devices */
			if (vf >= adapter->vfs_allocated_count)
				return E1000_SUCCESS;

			adapter->vf_data[vf].vlans_enabled--;
			if (!adapter->vf_data[vf].vlans_enabled) {
				u32 size;
				reg = E1000_READ_REG(hw, E1000_VMOLR(vf));
				size = reg & E1000_VMOLR_RLPML_MASK;
				size -= 4;
				reg &= ~E1000_VMOLR_RLPML_MASK;
				reg |= size;
				E1000_WRITE_REG(hw, E1000_VMOLR(vf), reg);
			}
		}
	}
	return E1000_SUCCESS;
}

#ifdef IFLA_VF_MAX
static void igb_set_vmvir(struct igb_adapter *adapter, u32 vid, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;

	if (vid)
		E1000_WRITE_REG(hw, E1000_VMVIR(vf), (vid | E1000_VMVIR_VLANA_DEFAULT));
	else
		E1000_WRITE_REG(hw, E1000_VMVIR(vf), 0);
}

static int igb_ndo_set_vf_vlan(IOEthernetController *netdev,
			       int vf, u16 vlan, u8 qos)
{
	int err = 0;
	struct igb_adapter *adapter = netdev_priv(netdev);

	/* VLAN IDs accepted range 0-4094 */
	if ((vf >= adapter->vfs_allocated_count) || (vlan > VLAN_VID_MASK-1) || (qos > 7))
		return -EINVAL;
	if (vlan || qos) {
		err = igb_vlvf_set(adapter, vlan, !!vlan, vf);
		if (err)
			goto out;
		igb_set_vmvir(adapter, vlan | (qos << VLAN_PRIO_SHIFT), vf);
		igb_set_vmolr(adapter, vf, !vlan);
		adapter->vf_data[vf].pf_vlan = vlan;
		adapter->vf_data[vf].pf_qos = qos;
		igb_set_vf_vlan_strip(adapter, vf, true); 
		dev_info(&adapter->pdev->dev,
			 "Setting VLAN %d, QOS 0x%x on VF %d\n", vlan, qos, vf);
		if (test_bit(__IGB_DOWN, &adapter->state)) {
			dev_warn(&adapter->pdev->dev,
				 "The VF VLAN has been set,"
				 " but the PF device is not up.\n");
			dev_warn(&adapter->pdev->dev,
				 "Bring the PF device up before"
				 " attempting to use the VF device.\n");
		}
	} else {
		if (adapter->vf_data[vf].pf_vlan)
			dev_info(&adapter->pdev->dev,
					 "Clearing VLAN on VF %d\n", vf);
		igb_vlvf_set(adapter, adapter->vf_data[vf].pf_vlan,
				   false, vf);
		igb_set_vmvir(adapter, vlan, vf);
		igb_set_vmolr(adapter, vf, true);
		igb_set_vf_vlan_strip(adapter, vf, false); 
		adapter->vf_data[vf].pf_vlan = 0;
		adapter->vf_data[vf].pf_qos = 0;
       }
out:
       return err;
}
#endif

static int igb_set_vf_vlan(struct igb_adapter *adapter, u32 *msgbuf, u32 vf)
{
	int add = (msgbuf[0] & E1000_VT_MSGINFO_MASK) >> E1000_VT_MSGINFO_SHIFT;
	int vid = (msgbuf[1] & E1000_VLVF_VLANID_MASK);

	if (vid)
		igb_set_vf_vlan_strip(adapter, vf, true);
	else
		igb_set_vf_vlan_strip(adapter, vf, false);

	return igb_vlvf_set(adapter, vid, add, vf);
}

static inline void igb_vf_reset(struct igb_adapter *adapter, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;

	/* clear flags except flag that the PF has set the MAC */
	adapter->vf_data[vf].flags &= IGB_VF_FLAG_PF_SET_MAC;
	adapter->vf_data[vf].last_nack = jiffies;

	/* reset offloads to defaults */
	igb_set_vmolr(adapter, vf, true);

	/* reset vlans for device */
	igb_clear_vf_vfta(adapter, vf);
#ifdef IFLA_VF_MAX
	if (adapter->vf_data[vf].pf_vlan)
		igb_ndo_set_vf_vlan(adapter->netdev, vf,
				    adapter->vf_data[vf].pf_vlan,
				    adapter->vf_data[vf].pf_qos);
	else
		igb_clear_vf_vfta(adapter, vf);
#endif

	/* reset multicast table array for vf */
	adapter->vf_data[vf].num_vf_mc_hashes = 0;

	/* Flush and reset the mta with the new values */
	igb_set_rx_mode(adapter->netdev);
	
	/* 
	 * Reset the VFs TDWBAL and TDWBAH registers which are not
	 * cleared by a VFLR
	 */
	E1000_WRITE_REG(hw, E1000_TDWBAH(vf), 0);
	E1000_WRITE_REG(hw, E1000_TDWBAL(vf), 0);
	if (hw->mac.type == e1000_82576) {
		E1000_WRITE_REG(hw, E1000_TDWBAH(IGB_MAX_VF_FUNCTIONS + vf), 0);
		E1000_WRITE_REG(hw, E1000_TDWBAL(IGB_MAX_VF_FUNCTIONS + vf), 0);
	}
}

static void igb_vf_reset_event(struct igb_adapter *adapter, u32 vf)
{
	unsigned char *vf_mac = adapter->vf_data[vf].vf_mac_addresses;

	/* generate a new mac address as we were hotplug removed/added */
	if (!(adapter->vf_data[vf].flags & IGB_VF_FLAG_PF_SET_MAC))
		random_ether_addr(vf_mac);
    
	/* process remaining reset events */
	igb_vf_reset(adapter, vf);
}

static void igb_vf_reset_msg(struct igb_adapter *adapter, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;
	unsigned char *vf_mac = adapter->vf_data[vf].vf_mac_addresses;
	u32 reg, msgbuf[3];
	u8 *addr = (u8 *)(&msgbuf[1]);

	/* process all the same items cleared in a function level reset */
	igb_vf_reset(adapter, vf);

	/* set vf mac address */
	igb_del_mac_filter(adapter, vf_mac, vf);
	igb_add_mac_filter(adapter, vf_mac, vf);

	/* enable transmit and receive for vf */
	reg = E1000_READ_REG(hw, E1000_VFTE);
	E1000_WRITE_REG(hw, E1000_VFTE, reg | (1 << vf));
	reg = E1000_READ_REG(hw, E1000_VFRE);
	E1000_WRITE_REG(hw, E1000_VFRE, reg | (1 << vf));

	adapter->vf_data[vf].flags |= IGB_VF_FLAG_CTS;

	/* reply to reset with ack and vf mac address */
	msgbuf[0] = E1000_VF_RESET | E1000_VT_MSGTYPE_ACK;
	memcpy(addr, vf_mac, 6);
	e1000_write_mbx(hw, msgbuf, 3, vf);
}

static int igb_set_vf_mac_addr(struct igb_adapter *adapter, u32 *msg, int vf)
{
	/*
	 * The VF MAC Address is stored in a packed array of bytes
	 * starting at the second 32 bit word of the msg array
	 */
	unsigned char *addr = (unsigned char *)&msg[1];
	int err = -1;

	if (is_valid_ether_addr(addr))
		err = igb_set_vf_mac(adapter, vf, addr);

	return err;
}

static void igb_rcv_ack_from_vf(struct igb_adapter *adapter, u32 vf)
{
	struct e1000_hw *hw = &adapter->hw;
	struct vf_data_storage *vf_data = &adapter->vf_data[vf];
	u32 msg = E1000_VT_MSGTYPE_NACK;

	/* if device isn't clear to send it shouldn't be reading either */
	if (!(vf_data->flags & IGB_VF_FLAG_CTS) &&
	    time_after(jiffies, vf_data->last_nack + (2 * HZ))) {
		e1000_write_mbx(hw, &msg, 1, vf);
		vf_data->last_nack = jiffies;
	}
}

static void igb_rcv_msg_from_vf(struct igb_adapter *adapter, u32 vf)
{
	u32 msgbuf[E1000_VFMAILBOX_SIZE];
	struct e1000_hw *hw = &adapter->hw;
	struct vf_data_storage *vf_data = &adapter->vf_data[vf];
	s32 retval;

	retval = e1000_read_mbx(hw, msgbuf, E1000_VFMAILBOX_SIZE, vf);

	if (retval) {
		IOLog( "Error receiving message from VF\n");
		return;
	}

	/* this is a message we already processed, do nothing */
	if (msgbuf[0] & (E1000_VT_MSGTYPE_ACK | E1000_VT_MSGTYPE_NACK))
		return;

	/*
	 * until the vf completes a reset it should not be
	 * allowed to start any configuration.
	 */

	if (msgbuf[0] == E1000_VF_RESET) {
		igb_vf_reset_msg(adapter, vf);
		return;
	}

	if (!(vf_data->flags & IGB_VF_FLAG_CTS)) {
		msgbuf[0] = E1000_VT_MSGTYPE_NACK;
		if (time_after(jiffies, vf_data->last_nack + (2 * HZ))) {
			e1000_write_mbx(hw, msgbuf, 1, vf);
			vf_data->last_nack = jiffies;
		}
		return;
	}

	switch ((msgbuf[0] & 0xFFFF)) {
	case E1000_VF_SET_MAC_ADDR:
		retval = -EINVAL;
#ifndef IGB_DISABLE_VF_MAC_SET
		if (!(vf_data->flags & IGB_VF_FLAG_PF_SET_MAC))
			retval = igb_set_vf_mac_addr(adapter, msgbuf, vf);
		else
			DPRINTK(DRV, INFO,
				"VF %d attempted to override administratively "
				"set MAC address\nReload the VF driver to "
				"resume operations\n", vf);
#endif
		break;
	case E1000_VF_SET_PROMISC:
		retval = igb_set_vf_promisc(adapter, msgbuf, vf);
		break;
	case E1000_VF_SET_MULTICAST:
		retval = igb_set_vf_multicasts(adapter, msgbuf, vf);
		break;
	case E1000_VF_SET_LPE:
		retval = igb_set_vf_rlpml(adapter, msgbuf[1], vf);
		break;
	case E1000_VF_SET_VLAN:
		retval = -1;
#ifdef IFLA_VF_MAX
		if (vf_data->pf_vlan)
			DPRINTK(DRV, INFO,
				"VF %d attempted to override administratively "
				"set VLAN tag\nReload the VF driver to "
				"resume operations\n", vf);
		else
#endif
			retval = igb_set_vf_vlan(adapter, msgbuf, vf);
		break;
	default:
		IOLog( "Unhandled Msg %08x\n", msgbuf[0]);
		retval = -E1000_ERR_MBX;
		break;
	}

	/* notify the VF of the results of what it sent us */
	if (retval)
		msgbuf[0] |= E1000_VT_MSGTYPE_NACK;
	else
		msgbuf[0] |= E1000_VT_MSGTYPE_ACK;

	msgbuf[0] |= E1000_VT_MSGTYPE_CTS;

	e1000_write_mbx(hw, msgbuf, 1, vf);
}

static void igb_msg_task(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 vf;

	for (vf = 0; vf < adapter->vfs_allocated_count; vf++) {
		/* process any reset requests */
		if (!e1000_check_for_rst(hw, vf))
			igb_vf_reset_event(adapter, vf);

		/* process any messages pending */
		if (!e1000_check_for_msg(hw, vf))
			igb_rcv_msg_from_vf(adapter, vf);

		/* process any acks */
		if (!e1000_check_for_ack(hw, vf))
			igb_rcv_ack_from_vf(adapter, vf);
	}
}

/**
 *  igb_set_uta - Set unicast filter table address
 *  @adapter: board private structure
 *
 *  The unicast table address is a register array of 32-bit registers.
 *  The table is meant to be used in a way similar to how the MTA is used
 *  however due to certain limitations in the hardware it is necessary to
 *  set all the hash bits to 1 and use the VMOLR ROPE bit as a promiscuous
 *  enable bit to allow vlan tag stripping when promiscuous mode is enabled
 **/
static void igb_set_uta(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;

	/* The UTA table only exists on 82576 hardware and newer */
	if (hw->mac.type < e1000_82576)
		return;

	/* we only need to do this if VMDq is enabled */
	if (!adapter->vmdq_pools)
		return;

	for (i = 0; i < hw->mac.uta_reg_count; i++)
		E1000_WRITE_REG_ARRAY(hw, E1000_UTA, i, ~0);
}

/**
 * igb_intr_msi - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t igb_intr_msi(int irq, void *data)
{
#ifndef __APPLE__
	struct igb_adapter *adapter = data;
	struct igb_q_vector *q_vector = adapter->q_vector[0];
	struct e1000_hw *hw = &adapter->hw;
	/* read ICR disables interrupts using IAM */
	u32 icr = E1000_READ_REG(hw, E1000_ICR);

	igb_write_itr(q_vector);

	if (icr & E1000_ICR_DRSTA)
		schedule_work(&adapter->reset_task);

	if (icr & E1000_ICR_DOUTSYNC) {
		/* HW is reporting DMA is out of sync */
		adapter->stats.doosync++;
	}

	if (icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = 1;
		if (!test_bit(__IGB_DOWN, &adapter->state))
			mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

	napi_schedule(&q_vector->napi);
#endif
	return IRQ_HANDLED;
}

/**
 * igb_intr - Legacy Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t igb_intr(int irq, void *data)
{
#ifndef __APPLE__
	struct igb_adapter *adapter = data;
	struct igb_q_vector *q_vector = adapter->q_vector[0];
	struct e1000_hw *hw = &adapter->hw;
	/* Interrupt Auto-Mask...upon reading ICR, interrupts are masked.  No
	 * need for the IMC write */
	u32 icr = E1000_READ_REG(hw, E1000_ICR);

	/* IMS will not auto-mask if INT_ASSERTED is not set, and if it is
	 * not set, then the adapter didn't send an interrupt */
	if (!(icr & E1000_ICR_INT_ASSERTED))
		return IRQ_NONE;

	igb_write_itr(q_vector);

	if (icr & E1000_ICR_DRSTA)
		schedule_work(&adapter->reset_task);

	if (icr & E1000_ICR_DOUTSYNC) {
		/* HW is reporting DMA is out of sync */
		adapter->stats.doosync++;
	}

	if (icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = 1;
		/* guard against interrupt when we're going down */
		if (!test_bit(__IGB_DOWN, &adapter->state))
			mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

	napi_schedule(&q_vector->napi);
#endif
	return IRQ_HANDLED;
}

void igb_ring_irq_enable(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;

	if ((q_vector->rx.ring && (adapter->rx_itr_setting & 3)) ||
	    (!q_vector->rx.ring && (adapter->tx_itr_setting & 3))) {
		if ((adapter->num_q_vectors == 1) && !adapter->vf_data)
			igb_set_itr(q_vector);
		else
			igb_update_ring_itr(q_vector);
	}

	if (!test_bit(__IGB_DOWN, &adapter->state)) {
		if (adapter->msix_entries)
			E1000_WRITE_REG(hw, E1000_EIMS, q_vector->eims_value);
		else
			igb_irq_enable(adapter);
	}
}

/**
 * igb_poll - NAPI Rx polling callback
 * @napi: napi polling structure
 * @budget: count of how many packets we should handle
 **/
static int igb_poll(struct igb_q_vector *q_vector, int budget)
{
	bool clean_complete = true;

#ifdef IGB_DCA
	if (q_vector->adapter->flags & IGB_FLAG_DCA_ENABLED)
		igb_update_dca(q_vector);
#endif
	if (q_vector->tx.ring)
		clean_complete = igb_clean_tx_irq(q_vector);

	if (q_vector->rx.ring)
		clean_complete &= igb_clean_rx_irq(q_vector, budget);

#ifndef HAVE_NETDEV_NAPI_LIST
	/* if netdev is disabled we need to stop polling */
	if (!netif_running(q_vector->adapter->netdev))
		clean_complete = true;

#endif
	/* If all work not completed, return budget and keep polling */
	if (!clean_complete)
		return budget;

	/* If not enough Rx work done, exit the polling mode */
#ifndef __APPLE__
	napi_complete(napi);
#endif
	igb_ring_irq_enable(q_vector);
	return 0;
}

#ifdef HAVE_HW_TIME_STAMP
/**
 * igb_systim_to_hwtstamp - convert system time value to hw timestamp
 * @adapter: board private structure
 * @shhwtstamps: timestamp structure to update
 * @regval: unsigned 64bit system time value.
 *
 * We need to convert the system time value stored in the RX/TXSTMP registers
 * into a hwtstamp which can be used by the upper level timestamping functions
 */
static void igb_systim_to_hwtstamp(struct igb_adapter *adapter,
                                   struct skb_shared_hwtstamps *shhwtstamps,
                                   u64 regval)
{
	u64 ns;

	/*
	 * The 82580 starts with 1ns at bit 0 in RX/TXSTMPL, shift this up to
	 * 24 to match clock shift we setup earlier.
	 */
	if (adapter->hw.mac.type >= e1000_82580)
		regval <<= IGB_82580_TSYNC_SHIFT;

	ns = timecounter_cyc2time(&adapter->clock, regval);
	
	/*
	 * force a timecompare_update here (even if less than a second
	 * has passed) in order to prevent the case when ptpd or other
	 * software jumps the clock offset. othwerise there is a small
	 * window when the timestamp would be based on previous skew
	 * and invalid results would be pushed to the network stack.
	 */
	timecompare_update(&adapter->compare, 0);
	memset(shhwtstamps, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);
	shhwtstamps->syststamp = timecompare_transform(&adapter->compare, ns);
}

/**
 * igb_tx_hwtstamp - utility function which checks for TX time stamp
 * @q_vector: pointer to q_vector containing needed info
 * @buffer: pointer to igb_tx_buffer structure
 *
 * If we were asked to do hardware stamping and such a time stamp is
 * available, then it must have been for this skb here because we only
 * allow only one such packet into the queue.
 */
static void igb_tx_hwtstamp(struct igb_q_vector *q_vector,
			    struct igb_tx_buffer *buffer_info)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;
	struct skb_shared_hwtstamps shhwtstamps;
	u64 regval;

	/* if skb does not support hw timestamp or TX stamp not valid exit */
	if (likely(!(buffer_info->tx_flags & IGB_TX_FLAGS_TSTAMP)) ||
	    !(E1000_READ_REG(hw, E1000_TSYNCTXCTL) & E1000_TSYNCTXCTL_VALID))
		return;

	regval = E1000_READ_REG(hw, E1000_TXSTMPL);
	regval |= (u64)E1000_READ_REG(hw, E1000_TXSTMPH) << 32;

	igb_systim_to_hwtstamp(adapter, &shhwtstamps, regval);
	skb_tstamp_tx(buffer_info->skb, &shhwtstamps);
}

#endif
/**
 * igb_clean_tx_irq - Reclaim resources after transmit completes
 * @q_vector: pointer to q_vector containing needed info
 * returns TRUE if ring is completely cleaned
 **/
static bool igb_clean_tx_irq(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct igb_ring *tx_ring = q_vector->tx.ring;
	struct igb_tx_buffer *tx_buffer;
	union e1000_adv_tx_desc *tx_desc, *eop_desc;
	unsigned int total_bytes = 0, total_packets = 0;
	unsigned int budget = q_vector->tx.work_limit;
	unsigned int i = tx_ring->next_to_clean;

	if (test_bit(__IGB_DOWN, &adapter->state))
		return true;

	tx_buffer = &tx_ring->tx_buffer_info[i];
	tx_desc = IGB_TX_DESC(tx_ring, i);
	i -= tx_ring->count;

	for (; budget; budget--) {
		eop_desc = tx_buffer->next_to_watch;

		/* prevent any other reads prior to eop_desc */
		rmb();

		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* if DD is not set pending work has not been completed */
		if (!(eop_desc->wb.status & cpu_to_le32(E1000_TXD_STAT_DD)))
			break;

		/* clear next_to_watch to prevent false hangs */
		tx_buffer->next_to_watch = NULL;

		/* update the statistics for this packet */
		total_bytes += tx_buffer->bytecount;
		total_packets += tx_buffer->gso_segs;

#ifdef HAVE_HW_TIME_STAMP
		/* retrieve hardware timestamp */
		igb_tx_hwtstamp(q_vector, tx_buffer);

#endif
		/* free the skb */
		adapter->netdev->freePacket(tx_buffer->skb);

		/* unmap skb header data */
#ifndef	__APPLE__
		dma_unmap_single(tx_ring->dev,
		                 dma_unmap_addr(tx_buffer, dma),
		                 dma_unmap_len(tx_buffer, len),
		                 DMA_TO_DEVICE);
#endif
		/* clear tx_buffer data */
		tx_buffer->skb = NULL;
		dma_unmap_len_set(tx_buffer, len, 0);
		
		/* clear last DMA location and unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= tx_ring->count;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = IGB_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
#ifdef	__APPLE__
			dma_unmap_len_set(tx_buffer, len, 0);
#else
			if (dma_unmap_len(tx_buffer, len)) {
				dma_unmap_page(tx_ring->dev,
				               dma_unmap_addr(tx_buffer, dma),
				               dma_unmap_len(tx_buffer, len),
				               DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buffer, len, 0);
			}
#endif
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		tx_desc++;
		i++;
		if (unlikely(!i)) {
			i -= tx_ring->count;
			tx_buffer = tx_ring->tx_buffer_info;
			tx_desc = IGB_TX_DESC(tx_ring, 0);
		}
	}

#ifdef CONFIG_BQL
	netdev_tx_completed_queue(txring_txq(tx_ring),
							  total_packets, total_bytes);
#endif /* CONFIG_BQL */

	i += tx_ring->count;
	tx_ring->next_to_clean = i;
	tx_ring->tx_stats.bytes += total_bytes;
	tx_ring->tx_stats.packets += total_packets;
	q_vector->tx.total_bytes += total_bytes;
	q_vector->tx.total_packets += total_packets;

	if (test_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags)) {
		struct e1000_hw *hw = &adapter->hw;

		eop_desc = tx_buffer->next_to_watch;

		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i */
		clear_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags);
		if (eop_desc &&
		    time_after(jiffies, tx_buffer->time_stamp +
		               (adapter->tx_timeout_factor * HZ))
		    && !(E1000_READ_REG(hw, E1000_STATUS) &
		         E1000_STATUS_TXOFF)) {

			/* detected Tx unit hang */
			IOLog(
				"Detected Tx Unit Hang\n"
				"  Tx Queue             <%d>\n"
				"  TDH                  <%x>\n"
				"  TDT                  <%x>\n"
				"  next_to_use          <%x>\n"
				"  next_to_clean        <%x>\n"
				"buffer_info[next_to_clean]\n"
				"  time_stamp           <%lx>\n"
				"  next_to_watch        <%p>\n"
				"  desc.status          <%x>\n",
				tx_ring->queue_index,
				E1000_READ_REG(hw, E1000_TDH(tx_ring->reg_idx)),
				readl(tx_ring->tail),
				tx_ring->next_to_use,
				tx_ring->next_to_clean,
				tx_buffer->time_stamp,
				eop_desc,
				eop_desc->wb.status);
#ifdef	__APPLE__
				netif_stop_queue(netdev_ring(tx_ring));
#else
				if (netif_is_multiqueue(netdev_ring(tx_ring)))
				netif_stop_subqueue(netdev_ring(tx_ring),
						    ring_queue_index(tx_ring));
			else
				netif_stop_queue(netdev_ring(tx_ring));
#endif
			/* we are about to reset, no point in enabling stuff */
			return true;
		}
	}

	if (unlikely(total_packets &&
		     netif_carrier_ok(netdev_ring(tx_ring)) &&
		     igb_desc_unused(tx_ring) >= IGB_TX_QUEUE_WAKE)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
#ifdef	__APPLE__
		if (netif_queue_stopped(netdev_ring(tx_ring)) &&
			!(test_bit(__IGB_DOWN, &adapter->state))) {
			netif_wake_queue(netdev_ring(tx_ring));
			tx_ring->tx_stats.restart_queue++;
		}
#else
		if (netif_is_multiqueue(netdev_ring(tx_ring))) {
			if (__netif_subqueue_stopped(netdev_ring(tx_ring),
						     ring_queue_index(tx_ring)) &&
			    !(test_bit(__IGB_DOWN, &adapter->state))) {
				netif_wake_subqueue(netdev_ring(tx_ring),
						    ring_queue_index(tx_ring));
				tx_ring->tx_stats.restart_queue++;
			}
		} else {
			if (netif_queue_stopped(netdev_ring(tx_ring)) &&
			    !(test_bit(__IGB_DOWN, &adapter->state))) {
				netif_wake_queue(netdev_ring(tx_ring));
				tx_ring->tx_stats.restart_queue++;
			}
		}
#endif
	}

	return !!budget;
}

#ifdef HAVE_VLAN_RX_REGISTER
/**
 * igb_receive_skb - helper function to handle rx indications
 * @q_vector: structure containing interrupt and ring information
 * @skb: packet to send up
 **/
static void igb_receive_skb(struct igb_q_vector *q_vector,
                            struct sk_buff *skb)
{
#ifdef	__APPLE__
	q_vector->adapter->netdev->receive(skb);
#else
	struct vlan_group **vlgrp = netdev_priv(skb->dev);

	if (IGB_CB(skb)->vid) {
		if (*vlgrp) {
			vlan_gro_receive(&q_vector->napi, *vlgrp,
					 IGB_CB(skb)->vid, skb);
		} else {
			dev_kfree_skb_any(skb);
		}
	} else {
		napi_gro_receive(&q_vector->napi, skb);
	}
#endif
}

#endif /* HAVE_VLAN_RX_REGISTER */
static inline void igb_rx_checksum(struct igb_ring *ring,
				   union e1000_adv_rx_desc *rx_desc,
				   struct sk_buff *skb)
{
	//skb_checksum_none_assert(skb);

	/* Ignore Checksum bit is set */
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_IXSM))
		return;

	/* Rx checksum disabled via ethtool */
#ifdef HAVE_NDO_SET_FEATURES
	if (!(netdev_ring(ring)->features & NETIF_F_RXCSUM))
#else
	if (!test_bit(IGB_RING_FLAG_RX_CSUM, &ring->flags))
#endif
		return;

	/* TCP/UDP checksum error bit is set */
	if (igb_test_staterr(rx_desc,
	    		     E1000_RXDEXT_STATERR_TCPE |
			     E1000_RXDEXT_STATERR_IPE)) {
		/*
		 * work around errata with sctp packets where the TCPE aka
		 * L4E bit is set incorrectly on 64 byte (60 byte w/o crc)
		 * packets, (aka let the stack check the crc32c)
		 */
		if (!((mbuf_pkthdr_len(skb) == 60) &&
		      test_bit(IGB_RING_FLAG_RX_SCTP_CSUM, &ring->flags)))
			ring->rx_stats.csum_err++;

		/* let the stack verify checksum errors */
		return;
	}
	/* It must be a TCP or UDP packet with a valid checksum */
#ifdef	__APPLE__
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_IPCS))
		ring->netdev->rxChecksumOK(skb, IONetworkController::kChecksumIP);
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_TCPCS))
		ring->netdev->rxChecksumOK(skb, IONetworkController::kChecksumTCP|IONetworkController::kChecksumUDP);
#else
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_TCPCS |
						 E1000_RXD_STAT_UDPCS))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
}

#ifdef NETIF_F_RXHASH
static inline void igb_rx_hash(struct igb_ring *ring,
			       union e1000_adv_rx_desc *rx_desc,
			       struct sk_buff *skb)
{
	if (((AppleIGB*)netdev_ring(ring))->features() & NETIF_F_RXHASH)
		skb->rxhash = le32_to_cpu(rx_desc->wb.lower.hi_dword.rss);
}

#endif
#ifdef HAVE_HW_TIME_STAMP
static void igb_rx_hwtstamp(struct igb_q_vector *q_vector,
			    union e1000_adv_rx_desc *rx_desc,
                            struct sk_buff *skb)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;
	u64 regval;

	if (!igb_test_staterr(rx_desc, E1000_RXDADV_STAT_TSIP |
				       E1000_RXDADV_STAT_TS))
		return;

	/*
	 * If this bit is set, then the RX registers contain the time stamp. No
	 * other packet will be time stamped until we read these registers, so
	 * read the registers to make them available again. Because only one
	 * packet can be time stamped at a time, we know that the register
	 * values must belong to this one here and therefore we don't need to
	 * compare any of the additional attributes stored for it.
	 *
	 * If nothing went wrong, then it should have a skb_shared_tx that we
	 * can turn into a skb_shared_hwtstamps.
	 */
	if (igb_test_staterr(rx_desc, E1000_RXDADV_STAT_TSIP)) {
		u32 *stamp = (u32 *)skb->data;
		regval = le32_to_cpu(*(stamp + 2));
		regval |= (u64)le32_to_cpu(*(stamp + 3)) << 32;
		skb_pull(skb, IGB_TS_HDR_LEN);
	} else {
		if(!(E1000_READ_REG(hw, E1000_TSYNCRXCTL) & E1000_TSYNCRXCTL_VALID))
			return;

		regval = E1000_READ_REG(hw, E1000_RXSTMPL);
		regval |= (u64)E1000_READ_REG(hw, E1000_RXSTMPH) << 32;
	}

	igb_systim_to_hwtstamp(adapter, skb_hwtstamps(skb), regval);
}
#endif
#ifdef __APPLE__
static u16 igb_rx_vlan(struct igb_ring *ring,
			union e1000_adv_rx_desc *rx_desc,
			struct sk_buff *skb)
{
	u16 vid = 0;
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_VP)) {
		if (igb_test_staterr(rx_desc, E1000_RXDEXT_STATERR_LB) &&
			test_bit(IGB_RING_FLAG_RX_LB_VLAN_BSWAP, &ring->flags))
			vid = be16_to_cpu(rx_desc->wb.upper.vlan);
		else
			vid = le16_to_cpu(rx_desc->wb.upper.vlan);
	}
	return vid;
}
#else /* __APPLE__ */
static void igb_rx_vlan(struct igb_ring *ring,
			union e1000_adv_rx_desc *rx_desc,
			struct sk_buff *skb)
{
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_VP)) {
		u16 vid = 0;
		if (igb_test_staterr(rx_desc, E1000_RXDEXT_STATERR_LB) &&
		    test_bit(IGB_RING_FLAG_RX_LB_VLAN_BSWAP, &ring->flags))
			vid = be16_to_cpu(rx_desc->wb.upper.vlan);
		else
			vid = le16_to_cpu(rx_desc->wb.upper.vlan);
#ifdef HAVE_VLAN_RX_REGISTER
		IGB_CB(skb)->vid = vid;
	} else {
		IGB_CB(skb)->vid = 0;
#else
		__vlan_hwaccel_put_tag(skb, vid);
#endif
	}
}
#endif

#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
static inline u16 igb_get_hlen(union e1000_adv_rx_desc *rx_desc)
{
	/* HW will not DMA in data larger than the given buffer, even if it
	 * parses the (NFS, of course) header to be larger.  In that case, it
	 * fills the header buffer and spills the rest into the page.
	 */
	u16 hlen = (le16_to_cpu(rx_desc->wb.lower.lo_dword.hs_rss.hdr_info) &
	           E1000_RXDADV_HDRBUFLEN_MASK) >> E1000_RXDADV_HDRBUFLEN_SHIFT;
	if (hlen > IGB_RX_HDR_LEN)
		hlen = IGB_RX_HDR_LEN;
	return hlen;
}

#endif

static bool igb_clean_rx_irq(struct igb_q_vector *q_vector, int budget)
{
	struct igb_ring *rx_ring = q_vector->rx.ring;
	union e1000_adv_rx_desc *rx_desc;
	unsigned int total_bytes = 0, total_packets = 0;
	u16 cleaned_count = igb_desc_unused(rx_ring);
	u16 i = rx_ring->next_to_clean;
#ifdef __APPLE__
	u16 vid;
#endif

	rx_desc = IGB_RX_DESC(rx_ring, i);

	while (igb_test_staterr(rx_desc, E1000_RXD_STAT_DD)) {
		struct igb_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];
		struct sk_buff *skb = buffer_info->skb;
		union e1000_adv_rx_desc *next_rxd;

		buffer_info->skb = NULL;
		prefetch(skb->data);

		i++;
		if (i == rx_ring->count)
			i = 0;

		next_rxd = IGB_RX_DESC(rx_ring, i);
		prefetch(next_rxd);

		/*
		 * This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * RXD_STAT_DD bit is set
		 */
		rmb();

#ifdef	__APPLE__
		u16 length = le16_to_cpu(rx_desc->wb.upper.length);
		if(buffer_info->pool == NULL){	// skb direct
			// skb is already there
		} else {
			skb = rx_ring->netdev->allocatePacket(length);
			if(skb){
				mbuf_copyback(skb, 0, length,
							  buffer_info->pool->getBytesNoCopy(),
							  MBUF_WAITOK);
			}
			buffer_info->pool->complete();
		}
		buffer_info->dma = 0;
#else	/* __APPLE__ */
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
		__skb_put(skb, le16_to_cpu(rx_desc->wb.upper.length));
		dma_unmap_single(rx_ring->dev, buffer_info->dma,
				 rx_ring->rx_buffer_len,
				 DMA_FROM_DEVICE);
		buffer_info->dma = 0;
#else	/* CONFIG_IGB_DISABLE_PACKET_SPLIT */
		if (!skb_is_nonlinear(skb)) {
			__skb_put(skb, igb_get_hlen(rx_desc));
			dma_unmap_single(rx_ring->dev, buffer_info->dma,
			                 IGB_RX_HDR_LEN,
							 DMA_FROM_DEVICE);
			buffer_info->dma = 0;
		}
		
		if (rx_desc->wb.upper.length) {
			u16 length = le16_to_cpu(rx_desc->wb.upper.length);

			skb_fill_page_desc(skb, skb_shinfo(skb)->nr_frags,
							   buffer_info->page,
							   buffer_info->page_offset,
							   length);
			
			if ((page_count(buffer_info->page) != 1) ||
			    (page_to_nid(buffer_info->page) != current_node))
				buffer_info->page = NULL;
			else
				get_page(buffer_info->page);
			dma_unmap_page(rx_ring->dev, buffer_info->page_dma,
				       PAGE_SIZE / 2, DMA_FROM_DEVICE);
			buffer_info->page_dma = 0;
		}
		if (!igb_test_staterr(rx_desc, E1000_RXD_STAT_EOP)) {
			struct igb_rx_buffer *next_buffer;
			next_buffer = &rx_ring->rx_buffer_info[i];
			buffer_info->skb = next_buffer->skb;
			buffer_info->dma = next_buffer->dma;
			next_buffer->skb = skb;
			next_buffer->dma = 0;
			goto next_desc;
		}
		
#endif /* CONFIG_IGB_DISABLE_PACKET_SPLIT */
#endif	/* __APPLE__ */
		if (igb_test_staterr(rx_desc,
				     E1000_RXDEXT_ERR_FRAME_ERR_MASK)) {
			((AppleIGB*)netdev_ring(rx_ring))->receiveError(skb);
			goto next_desc;
		}

#ifdef HAVE_HW_TIME_STAMP
		igb_rx_hwtstamp(q_vector, rx_desc, skb);
#endif
#ifdef NETIF_F_RXHASH
		igb_rx_hash(rx_ring, rx_desc, skb);
#endif
		igb_rx_checksum(rx_ring, rx_desc, skb);
		vid = igb_rx_vlan(rx_ring, rx_desc, skb);

		total_bytes += mbuf_pkthdr_len(skb);
		total_packets++;

		//skb->protocol = eth_type_trans(skb, netdev_ring(rx_ring));

#ifdef HAVE_VLAN_RX_REGISTER
		igb_receive_skb(q_vector, skb);
#else
		((AppleIGB*)netdev_ring(rx_ring))->receive(skb, (UInt32)vid);
#endif

#ifndef NETIF_F_GRO
		//netdev_ring(rx_ring)->last_rx = jiffies;

#endif
		budget--;
next_desc:
		cleaned_count++;

		if (!budget)
			break;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= IGB_RX_BUFFER_WRITE) {
			igb_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
	}

	rx_ring->next_to_clean = i;
	rx_ring->rx_stats.packets += total_packets;
	rx_ring->rx_stats.bytes += total_bytes;
	q_vector->rx.total_packets += total_packets;
	q_vector->rx.total_bytes += total_bytes;

	if (cleaned_count)
		igb_alloc_rx_buffers(rx_ring, cleaned_count);

	return !!budget;
}


static bool igb_alloc_mapped_skb(struct igb_ring *rx_ring,
				 struct igb_rx_buffer *bi)
{
#ifdef	__APPLE__
	dma_addr_t dma;

	if(rx_ring->rx_buffer_len <= 2048){	// direct receive
		struct sk_buff *skb = bi->skb;
		
		if (likely(!skb)) {
			skb = netdev_alloc_skb_ip_align(netdev_ring(rx_ring),
											rx_ring->rx_buffer_len);
			bi->skb = skb;
			if (!skb) {
				rx_ring->rx_stats.alloc_failed++;
				return false;
			}
			
			/* initialize skb for ring */
			skb_record_rx_queue(skb, ring_queue_index(rx_ring));
		}
		struct IOPhysicalSegment vec[2];
		UInt32 count = netdev_ring(rx_ring)->rxCursor()->getPhysicalSegmentsWithCoalesce(skb, vec, 1);
		if(count == 0){
			rx_ring->rx_stats.alloc_failed++;
			netdev_ring(rx_ring)->freePacket(skb);
			bi->skb = NULL;
			return false;
		}
		dma = vec[0].location;
	} else {
		// actually, skb is not allocated - alloc/copy on receive.
		IOBufferMemoryDescriptor *pool = bi->pool;
		dma = bi->dma;
		if (dma){
			IOLog("igb_alloc_mapped_skb: dma is still active\n");
			return true;
		}
		
		if (pool) {
			//IOLog("igb_alloc_mapped_skb: resuse pool\n");
		} else {
			//IOLog("igb_alloc_mapped_skb: new pool, size = %d\n", (int)rx_ring->rx_buffer_len);
			pool = IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task,
															   kIODirectionInOut | kIOMemoryPhysicallyContiguous,
															   (vm_size_t)(rx_ring->rx_buffer_len), PAGE_SIZE/2 );
			bi->pool = pool;
			if (unlikely(!pool)) {
				rx_ring->rx_stats.alloc_failed++;
				return false;
			}
		}
		pool->prepare();
		dma = pool->getPhysicalAddress();
	}
#else	// __APPLE__
	struct sk_buff *skb = bi->skb;
	dma_addr_t dma = bi->dma;

	if (dma)
		return true;

	if (likely(!skb)) {
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
		skb = netdev_alloc_skb_ip_align(netdev_ring(rx_ring),
										rx_ring->rx_buffer_len);
#else
		skb = netdev_alloc_skb_ip_align(netdev_ring(rx_ring),
						IGB_RX_HDR_LEN);
#endif
		bi->skb = skb;
		if (!skb) {
			rx_ring->rx_stats.alloc_failed++;
			return false;
		}

		/* initialize skb for ring */
		skb_record_rx_queue(skb, ring_queue_index(rx_ring));
	}

#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
	dma = dma_map_single(rx_ring->dev, skb->data,
			     rx_ring->rx_buffer_len, DMA_FROM_DEVICE);
#else
	dma = dma_map_single(rx_ring->dev, skb->data,
			     IGB_RX_HDR_LEN, DMA_FROM_DEVICE);
#endif

	if (dma_mapping_error(rx_ring->dev, dma)) {
		rx_ring->rx_stats.alloc_failed++;
		return false;
	}

#endif	// __APPLE__
	bi->dma = dma;
	return true;
}

#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
static bool igb_alloc_mapped_page(struct igb_ring *rx_ring,
				  struct igb_rx_buffer *bi)
{
	IOBufferMemoryDescriptor *page = bi->page;
	dma_addr_t page_dma = bi->page_dma;
	unsigned int page_offset = bi->page_offset ^ (PAGE_SIZE / 2);

	if (page_dma)
		return true;

	if (!page) {
		page = IOBufferMemoryDescriptor::inTaskWithOptions( kernel_task,
													kIODirectionInOut | kIOMemoryPhysicallyContiguous,
													PAGE_SIZE/2, PAGE_SIZE/2 );
		bi->page = page;
		if (unlikely(!page)) {
			rx_ring->rx_stats.alloc_failed++;
			return false;
		}
	}
	page->prepare();
	page_dma = page->getPhysicalAddress();

	bi->page_dma = page_dma;
	bi->page_offset = page_offset;
	return true;
}

#endif /* CONFIG_IGB_DISABLE_PACKET_SPLIT */
/**
 * igb_alloc_rx_buffers - Replace used receive buffers; packet split
 * @adapter: address of board private structure
 **/
void igb_alloc_rx_buffers(struct igb_ring *rx_ring, u16 cleaned_count)
{
	union e1000_adv_rx_desc *rx_desc;
	struct igb_rx_buffer *bi;
	u16 i = rx_ring->next_to_use;

	rx_desc = IGB_RX_DESC(rx_ring, i);
	bi = &rx_ring->rx_buffer_info[i];
	i -= rx_ring->count;

	while (cleaned_count--) {
		if (!igb_alloc_mapped_skb(rx_ring, bi))
			break;

		/* Refresh the desc even if buffer_addrs didn't change
		 * because each write-back erases this info. */
#ifdef CONFIG_IGB_DISABLE_PACKET_SPLIT
		rx_desc->read.pkt_addr = cpu_to_le64(bi->dma);
#else
		rx_desc->read.hdr_addr = cpu_to_le64(bi->dma);

		if (!igb_alloc_mapped_page(rx_ring, bi))
			break;

		rx_desc->read.pkt_addr = cpu_to_le64(bi->page_dma);

#endif /* CONFIG_IGB_DISABLE_PACKET_SPLIT */
		rx_desc++;
		bi++;
		i++;
		if (unlikely(!i)) {
			rx_desc = IGB_RX_DESC(rx_ring, 0);
			bi = rx_ring->rx_buffer_info;
			i -= rx_ring->count;
		}

		/* clear the hdr_addr for the next_to_use descriptor */
		rx_desc->read.hdr_addr = 0;
	}

	i += rx_ring->count;

	if (rx_ring->next_to_use != i) {
		rx_ring->next_to_use = i;

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64). */
		wmb();
		writel(i, rx_ring->tail);
	}
}

#ifdef SIOCGMIIPHY
/**
 * igb_mii_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int igb_mii_ioctl(IOEthernetController *netdev, struct ifreq *ifr, int cmd)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);

	if (adapter->hw.phy.media_type != e1000_media_type_copper)
		return -EOPNOTSUPP;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = adapter->hw.phy.addr;
		break;
	case SIOCGMIIREG:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (e1000_read_phy_reg(&adapter->hw, data->reg_num & 0x1F,
				   &data->val_out))
			return -EIO;
		break;
	case SIOCSMIIREG:
	default:
		return -EOPNOTSUPP;
	}
	return E1000_SUCCESS;
}

#endif
#ifdef HAVE_HW_TIME_STAMP
/**
 * igb_hwtstamp_ioctl - control hardware time stamping
 * @netdev:
 * @ifreq:
 * @cmd:
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't case any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware
 * filters. Not all combinations are supported, in particular event
 * type has to be specified. Matching the kind of event packet is
 * not supported, with the exception of "all V2 events regardless of
 * level 2 or 4".
 *
 **/
static int igb_hwtstamp_ioctl(IOEthernetController *netdev,
			      struct ifreq *ifr, int cmd)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct hwtstamp_config config;
	u32 tsync_tx_ctl = E1000_TSYNCTXCTL_ENABLED;
	u32 tsync_rx_ctl = E1000_TSYNCRXCTL_ENABLED;
	u32 tsync_rx_cfg = 0;
	bool is_l4 = false;
	bool is_l2 = false;
	u32 regval;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		tsync_tx_ctl = 0;
	case HWTSTAMP_TX_ON:
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		tsync_rx_ctl = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_ALL:
		/*
		 * register TSYNCRXCFG must be set, therefore it is not
		 * possible to time stamp both Sync and Delay_Req messages
		 * => fall back to time stamping all packets
		 */
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_ALL;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
		tsync_rx_cfg = E1000_TSYNCRXCFG_PTP_V1_SYNC_MESSAGE;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
		tsync_rx_cfg = E1000_TSYNCRXCFG_PTP_V1_DELAY_REQ_MESSAGE;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
		tsync_rx_cfg = E1000_TSYNCRXCFG_PTP_V2_SYNC_MESSAGE;
		is_l2 = true;
		is_l4 = true;
		config.rx_filter = HWTSTAMP_FILTER_SOME;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
		tsync_rx_cfg = E1000_TSYNCRXCFG_PTP_V2_DELAY_REQ_MESSAGE;
		is_l2 = true;
		is_l4 = true;
		config.rx_filter = HWTSTAMP_FILTER_SOME;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_EVENT_V2;
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		is_l2 = true;
		is_l4 = true;
		break;
	default:
		return -ERANGE;
	}

	if (hw->mac.type == e1000_82575) {
		if (tsync_rx_ctl | tsync_tx_ctl)
			return -EINVAL;
		return 0;
	}

#ifdef IGB_PER_PKT_TIMESTAMP
	/*
	 * Per-packet timestamping only works if all packets are
	 * timestamped, so enable timestamping in all packets as
	 * long as one rx filter was configured.
	 */
	if ((hw->mac.type >= e1000_82580) && tsync_rx_ctl) {
		tsync_rx_ctl = E1000_TSYNCRXCTL_ENABLED;
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_ALL;
	}
#endif

	/* enable/disable TX */
	regval = E1000_READ_REG(hw, E1000_TSYNCTXCTL);
	regval &= ~E1000_TSYNCTXCTL_ENABLED;
	regval |= tsync_tx_ctl;
	E1000_WRITE_REG(hw, E1000_TSYNCTXCTL, regval);

	/* enable/disable RX */
	regval = E1000_READ_REG(hw, E1000_TSYNCRXCTL);
	regval &= ~(E1000_TSYNCRXCTL_ENABLED | E1000_TSYNCRXCTL_TYPE_MASK);
	regval |= tsync_rx_ctl;
	E1000_WRITE_REG(hw, E1000_TSYNCRXCTL, regval);

	/* define which PTP packets are time stamped */
	E1000_WRITE_REG(hw, E1000_TSYNCRXCFG, tsync_rx_cfg);

	/* define ethertype filter for timestamped packets */
	if (is_l2)
		E1000_WRITE_REG(hw, E1000_ETQF(3),
		                (E1000_ETQF_FILTER_ENABLE | /* enable filter */
		                 E1000_ETQF_1588 | /* enable timestamping */
		                 ETH_P_1588));     /* 1588 eth protocol type */
	else
		E1000_WRITE_REG(hw, E1000_ETQF(3), 0);

#define PTP_PORT 319
	/* L4 Queue Filter[3]: filter by destination port and protocol */
	if (is_l4) {
		u32 ftqf = (IPPROTO_UDP /* UDP */
			| E1000_FTQF_VF_BP /* VF not compared */
			| E1000_FTQF_1588_TIME_STAMP /* Enable Timestamping */
			| E1000_FTQF_MASK); /* mask all inputs */
		ftqf &= ~E1000_FTQF_MASK_PROTO_BP; /* enable protocol check */

		E1000_WRITE_REG(hw, E1000_IMIR(3), htons(PTP_PORT));
		E1000_WRITE_REG(hw, E1000_IMIREXT(3),
				(E1000_IMIREXT_SIZE_BP | E1000_IMIREXT_CTRL_BP));
		if (hw->mac.type == e1000_82576) {
			/* enable source port check */
			E1000_WRITE_REG(hw, E1000_SPQF(3), htons(PTP_PORT));
			ftqf &= ~E1000_FTQF_MASK_SOURCE_PORT_BP;
		}
		E1000_WRITE_REG(hw, E1000_FTQF(3), ftqf);
	} else {
		E1000_WRITE_REG(hw, E1000_FTQF(3), E1000_FTQF_MASK);
	}
	E1000_WRITE_FLUSH(hw);

	adapter->hwtstamp_config = config;

	/* clear TX/RX time stamp registers, just to be sure */
	regval = E1000_READ_REG(hw, E1000_TXSTMPH);
	regval = E1000_READ_REG(hw, E1000_RXSTMPH);

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

#endif

#ifndef	__APPLE__
/**
 * igb_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int igb_ioctl(IOEthernetController *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
#ifdef SIOCGMIIPHY
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return igb_mii_ioctl(netdev, ifr, cmd);
#endif
#ifdef HAVE_HW_TIME_STAMP
	case SIOCSHWTSTAMP:
		return igb_hwtstamp_ioctl(netdev, ifr, cmd);
#endif
#ifdef ETHTOOL_OPS_COMPAT
	case SIOCETHTOOL:
		return ethtool_ioctl(ifr);
#endif
	default:
		return -EOPNOTSUPP;
	}
}
#endif

s32 e1000_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = (igb_adapter*)hw->back;
	u8 cap_offset;

	if (0 == adapter->pdev->findPCICapability(kIOPCIPCIExpressCapability, &cap_offset))
		return -E1000_ERR_CONFIG;

	*value = adapter->pdev->configRead16(cap_offset + reg);

	return E1000_SUCCESS;
}

s32 e1000_write_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = (igb_adapter*)hw->back;
	u8 cap_offset;

	if (0 == adapter->pdev->findPCICapability(kIOPCIPCIExpressCapability, &cap_offset))
		return -E1000_ERR_CONFIG;
	
	adapter->pdev->configWrite16(cap_offset + reg, *value);

	return E1000_SUCCESS;
}

#ifdef HAVE_VLAN_RX_REGISTER
static void igb_vlan_mode(IOEthernetController *netdev, struct vlan_group *vlgrp)
#else
void igb_vlan_mode(IOEthernetController *netdev, u32 features)
#endif
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl, rctl;
	int i;
#ifdef HAVE_VLAN_RX_REGISTER
	bool enable = !!vlgrp;

	igb_irq_disable(adapter);

	adapter->vlgrp = vlgrp;

	if (!test_bit(__IGB_DOWN, &adapter->state))
		igb_irq_enable(adapter);
#else
	bool enable = !!(features & NETIF_F_HW_VLAN_RX);
#endif

	if (enable) {
		/* enable VLAN tag insert/strip */
		ctrl = E1000_READ_REG(hw, E1000_CTRL);
		ctrl |= E1000_CTRL_VME;
		E1000_WRITE_REG(hw, E1000_CTRL, ctrl);

		/* Disable CFI check */
		rctl = E1000_READ_REG(hw, E1000_RCTL);
		rctl &= ~E1000_RCTL_CFIEN;
		E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	} else {
		/* disable VLAN tag insert/strip */
		ctrl = E1000_READ_REG(hw, E1000_CTRL);
		ctrl &= ~E1000_CTRL_VME;
		E1000_WRITE_REG(hw, E1000_CTRL, ctrl);
	}

	for (i = 0; i < adapter->vmdq_pools; i++) {
		igb_set_vf_vlan_strip(adapter,
				      adapter->vfs_allocated_count + i,
				      enable);
	}

	igb_rlpml_set(adapter);
}

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
static int igb_vlan_rx_add_vid(IOEthernetController *netdev, u16 vid)
#else
static void igb_vlan_rx_add_vid(IOEthernetController *netdev, u16 vid)
#endif
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int pf_id = adapter->vfs_allocated_count;

	/* attempt to add filter to vlvf array */
	igb_vlvf_set(adapter, vid, TRUE, pf_id);

	/* add the filter since PF can receive vlans w/o entry in vlvf */
	igb_vfta_set(adapter, vid, TRUE);

#ifndef HAVE_VLAN_RX_REGISTER

	set_bit(vid, adapter->active_vlans);
#endif
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
		return 0;
#endif
}

static void igb_vlan_rx_kill_vid(IOEthernetController *netdev, u16 vid)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int pf_id = adapter->vfs_allocated_count;
	s32 err;

#ifdef HAVE_VLAN_RX_REGISTER
	igb_irq_disable(adapter);

	vlan_group_set_device(adapter->vlgrp, vid, NULL);

	if (!test_bit(__IGB_DOWN, &adapter->state))
		igb_irq_enable(adapter);

#endif /* HAVE_VLAN_RX_REGISTER */
	/* remove vlan from VLVF table array */
	err = igb_vlvf_set(adapter, vid, FALSE, pf_id);

	/* if vid was not present in VLVF just remove it from table */
	if (err)
		igb_vfta_set(adapter, vid, FALSE);
#ifndef HAVE_VLAN_RX_REGISTER

	clear_bit(vid, adapter->active_vlans);
#endif
}

static void igb_restore_vlan(struct igb_adapter *adapter)
{
#ifdef HAVE_VLAN_RX_REGISTER
	igb_vlan_mode(adapter->netdev, adapter->vlgrp);

	if (adapter->vlgrp) {
		u16 vid;
		for (vid = 0; vid < VLAN_N_VID; vid++) {
			if (!vlan_group_get_device(adapter->vlgrp, vid))
				continue;
			igb_vlan_rx_add_vid(adapter->netdev, vid);
		}
	}
#else
	u16 vid;

	igb_vlan_mode(adapter->netdev, adapter->netdev->features());

#ifdef	__APPLE__
	vid = 0;
	for(int k = 0; k < sizeof(BITS_TO_LONGS(VLAN_N_VID)); k++){
		unsigned long t = adapter->active_vlans[k];
		// little endian
		for( int j = 0; j < BITS_PER_LONG; j++ ){
			if(t & (1<<j)){
				igb_vlan_rx_add_vid(adapter->netdev, vid);
			}
			vid++;
		}
	}
#else
	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID)
		igb_vlan_rx_add_vid(adapter->netdev, vid);
#endif
#endif
}

int igb_set_spd_dplx(struct igb_adapter *adapter, u16 spddplx)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;

	mac->autoneg = 0;

	/* Fiber NIC's only allow 1000 gbps Full duplex */
	if ((adapter->hw.phy.media_type == e1000_media_type_internal_serdes ) &&
		spddplx != (SPEED_1000 + DUPLEX_FULL)) {
		IOLog(
		        "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}

	switch (spddplx) {
	case SPEED_10 + DUPLEX_HALF:
		mac->forced_speed_duplex = ADVERTISE_10_HALF;
		break;
	case SPEED_10 + DUPLEX_FULL:
		mac->forced_speed_duplex = ADVERTISE_10_FULL;
		break;
	case SPEED_100 + DUPLEX_HALF:
		mac->forced_speed_duplex = ADVERTISE_100_HALF;
		break;
	case SPEED_100 + DUPLEX_FULL:
		mac->forced_speed_duplex = ADVERTISE_100_FULL;
		break;
	case SPEED_1000 + DUPLEX_FULL:
		mac->autoneg = 1;
		adapter->hw.phy.autoneg_advertised = ADVERTISE_1000_FULL;
		break;
	case SPEED_1000 + DUPLEX_HALF: /* not supported */
	default:
		IOLog( "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}
	return 0;
}
	
#ifndef	__APPLE__
static int __igb_shutdown(struct pci_dev *pdev, bool *enable_wake,
							bool runtime)
{
	IOEthernetController *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl, rctl, status;
	u32 wufc = runtime ? E1000_WUFC_LNKC : adapter->wol;
#ifdef CONFIG_PM
	int retval = 0;
#endif

	netif_device_detach(netdev);

	if (netif_running(netdev))
		__igb_close(netdev, true);

	igb_clear_interrupt_scheme(adapter);

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;
#endif

	status = E1000_READ_REG(hw, E1000_STATUS);
	if (status & E1000_STATUS_LU)
		wufc &= ~E1000_WUFC_LNKC;

	if (wufc) {
		igb_setup_rctl(adapter);
		igb_set_rx_mode(netdev);

		/* turn on all-multi mode if wake on multicast is enabled */
		if (wufc & E1000_WUFC_MC) {
			rctl = E1000_READ_REG(hw, E1000_RCTL);
			rctl |= E1000_RCTL_MPE;
			E1000_WRITE_REG(hw, E1000_RCTL, rctl);
		}

		ctrl = E1000_READ_REG(hw, E1000_CTRL);
		/* phy power management enable */
		#define E1000_CTRL_EN_PHY_PWR_MGMT 0x00200000
		ctrl |= E1000_CTRL_ADVD3WUC;
		E1000_WRITE_REG(hw, E1000_CTRL, ctrl);

		/* Allow time for pending master requests to run */
		e1000_disable_pcie_master(hw);

		E1000_WRITE_REG(hw, E1000_WUC, E1000_WUC_PME_EN);
		E1000_WRITE_REG(hw, E1000_WUFC, wufc);
	} else {
		E1000_WRITE_REG(hw, E1000_WUC, 0);
		E1000_WRITE_REG(hw, E1000_WUFC, 0);
	}

	*enable_wake = wufc || adapter->en_mng_pt;
	if (!*enable_wake)
		igb_power_down_link(adapter);
	else
		igb_power_up_link(adapter);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant. */
	igb_release_hw_control(adapter);

	pci_disable_device(pdev);

	return 0;
}
#endif

#ifdef CONFIG_PM
#ifdef HAVE_SYSTEM_SLEEP_PM_OPS
static int igb_suspend(IOPCIDevice *pdev, pm_message_t state)
{
	int retval;
	bool wake;
	struct pci_dev *pdev = to_pci_dev(dev);

	retval = __igb_shutdown(pdev, &wake, 0);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}

static int igb_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	IOEthernetController *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	err = pci_enable_device_mem(pdev);
	if (err) {
		IOLog( "igb: Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

#ifdef CONFIG_PM_RUNTIME
	if (!rtnl_is_locked()) {
		/*
		 * shut up ASSERT_RTNL() warning in
		 * netif_set_real_num_tx/rx_queues.
		 */
		rtnl_lock();
		err = igb_init_interrupt_scheme(adapter);
		rtnl_unlock();
	} else {
		err = igb_init_interrupt_scheme(adapter);
	}
	if (err) {
#else
	if (igb_init_interrupt_scheme(adapter)) {
#endif /* CONFIG_PM_RUNTIME */
			IOLog( "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	igb_reset(adapter);

	/* let the f/w know that the h/w is now under the control of the
	 * driver. */
	igb_get_hw_control(adapter);

	E1000_WRITE_REG(hw, E1000_WUS, ~0);

	if (netdev->flags & IFF_UP) {
		err = __igb_open(netdev, true);
		if (err)
			return err;
	}

	netif_device_attach(netdev);

	return 0;
}
#ifdef CONFIG_PM_RUNTIME
	static int igb_runtime_idle(struct device *dev)
	{
		struct pci_dev *pdev = to_pci_dev(dev);
		struct net_device *netdev = pci_get_drvdata(pdev);
		struct igb_adapter *adapter = netdev_priv(netdev);
		
		if (!igb_has_link(adapter))
			pm_schedule_suspend(dev, MSEC_PER_SEC * 5);
		
		return -EBUSY;
	}
	
	static int igb_runtime_suspend(struct device *dev)
	{
		struct pci_dev *pdev = to_pci_dev(dev);
		int retval;
		bool wake;
		
		retval = __igb_shutdown(pdev, &wake, 1);
		if (retval)
			return retval;
		
		if (wake) {
			pci_prepare_to_sleep(pdev);
		} else {
			pci_wake_from_d3(pdev, false);
			pci_set_power_state(pdev, PCI_D3hot);
		}
		
		return 0;
	}
	
	static int igb_runtime_resume(struct device *dev)
	{
		return igb_resume(dev);
	}
#endif /* CONFIG_PM_RUNTIME */
#endif /* HAVE_SYSTEM_SLEEP_PM_OPS */
#endif	/* CONFIG_PM */

#ifndef	__APPLE__
#ifdef USE_REBOOT_NOTIFIER
/* only want to do this for 2.4 kernels? */
static int igb_notify_reboot(struct notifier_block *nb, unsigned long event,
                             void *p)
{
	IOPCIDevice *pdev = NULL;
	bool wake;

	switch (event) {
	case SYS_DOWN:
	case SYS_HALT:
	case SYS_POWER_OFF:
		while ((pdev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, pdev))) {
			if (pci_dev_driver(pdev) == &igb_driver) {
				__igb_shutdown(pdev, &wake, 0);
				if (event == SYS_POWER_OFF) {
					pci_wake_from_d3(pdev, wake);
					pci_set_power_state(pdev, PCI_D3hot);
				}
			}
		}
	}
	return NOTIFY_DONE;
}
#else
static void igb_shutdown(IOPCIDevice *pdev)
{
	bool wake;

	__igb_shutdown(pdev, &wake, 0);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}
#endif /* USE_REBOOT_NOTIFIER */
#endif

#ifdef HAVE_PCI_ERS
#define E1000_DEV_ID_82576_VF 0x10CA
/**
 * igb_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t igb_io_error_detected(IOPCIDevice *pdev,
					      pci_channel_state_t state)
{
	IOEthernetController *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);

#ifdef CONFIG_PCI_IOV
	struct pci_dev *bdev, *vfdev;
	u32 dw0, dw1, dw2, dw3;
	int vf, pos;
	u16 req_id, pf_func;
	
	if (!(adapter->flags & IGB_FLAG_DETECT_BAD_DMA))
		goto skip_bad_vf_detection;
	
	bdev = pdev->bus->self;
	while (bdev && (bdev->pcie_type != PCI_EXP_TYPE_ROOT_PORT))
		bdev = bdev->bus->self;
	
	if (!bdev)
		goto skip_bad_vf_detection;
	
	pos = pci_find_ext_capability(bdev, PCI_EXT_CAP_ID_ERR);
	if (!pos)
		goto skip_bad_vf_detection;
	
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG, &dw0);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 4, &dw1);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 8, &dw2);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 12, &dw3);
	
	req_id = dw1 >> 16;
	/* On the 82576 if bit 7 of the requestor ID is set then it's a VF */
	if (!(req_id & 0x0080))
		goto skip_bad_vf_detection;
	
	pf_func = req_id & 0x01;
	if ((pf_func & 1) == (pdev->devfn & 1)) {
		
		vf = (req_id & 0x7F) >> 1;
		dev_err(pci_dev_to_dev(pdev),
				"VF %d has caused a PCIe error\n", vf);
		dev_err(pci_dev_to_dev(pdev),
				"TLP: dw0: %8.8x\tdw1: %8.8x\tdw2: "
				"%8.8x\tdw3: %8.8x\n",
				dw0, dw1, dw2, dw3);
		
		/* Find the pci device of the offending VF */
		vfdev = pci_get_device(PCI_VENDOR_ID_INTEL,
							   E1000_DEV_ID_82576_VF, NULL);
		while (vfdev) {
			if (vfdev->devfn == (req_id & 0xFF))
				break;
			vfdev = pci_get_device(PCI_VENDOR_ID_INTEL,
								   E1000_DEV_ID_82576_VF, vfdev);
		}
		/*
 		 * There's a slim chance the VF could have been hot plugged,
 		 * so if it is no longer present we don't need to issue the
 		 * VFLR.  Just clean up the AER in that case.
 		 */
		if (vfdev) {
			dev_err(pci_dev_to_dev(pdev),
					"Issuing VFLR to VF %d\n", vf);
			pci_write_config_dword(vfdev, 0xA8, 0x00008000);
		}
		
		pci_cleanup_aer_uncorrect_error_status(pdev);
	}
	
	/*
 	 * Even though the error may have occurred on the other port
 	 * we still need to increment the vf error reference count for
 	 * both ports because the I/O resume function will be called
 	 * for both of them.
 	 */
	adapter->vferr_refcount++;
	
	return PCI_ERS_RESULT_RECOVERED;
	
skip_bad_vf_detection:
#endif /* CONFIG_PCI_IOV */
	
	netif_device_detach(netdev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (netif_running(netdev))
		igb_down(adapter);
	pci_disable_device(pdev);

	/* Request a slot slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * igb_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot. Implementation
 * resembles the first-half of the igb_resume routine.
 */
static pci_ers_result_t igb_io_slot_reset(IOPCIDevice *pdev)
{
	IOEthernetController *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	pci_ers_result_t result;

	if (pci_enable_device_mem(pdev)) {
		IOLog( "Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pci_set_master(pdev);
		pci_restore_state(pdev);
		pci_save_state(pdev);

		pci_enable_wake(pdev, PCI_D3hot, 0);
		pci_enable_wake(pdev, PCI_D3cold, 0);

		schedule_work(&adapter->reset_task);
		E1000_WRITE_REG(hw, E1000_WUS, ~0);
		result = PCI_ERS_RESULT_RECOVERED;
	}

	pci_cleanup_aer_uncorrect_error_status(pdev);

	return result;
}

/**
 * igb_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation. Implementation resembles the
 * second-half of the igb_resume routine.
 */
static void igb_io_resume(IOPCIDevice *pdev)
{
	IOEthernetController *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);

	if (adapter->vferr_refcount) {
		dev_info(pci_dev_to_dev(pdev), "Resuming after VF err\n");
		adapter->vferr_refcount--;
		return;
	}

	if (netif_running(netdev)) {
		if (igb_up(adapter)) {
			IOLog( "igb_up failed after reset\n");
			return;
		}
	}

	netif_device_attach(netdev);

	/* let the f/w know that the h/w is now under the control of the
	 * driver. */
	igb_get_hw_control(adapter);
}

#endif /* HAVE_PCI_ERS */

int igb_add_mac_filter(struct igb_adapter *adapter, u8 *addr, u16 queue)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;

	if (is_zero_ether_addr(addr))
		return 0;

	for (i = 0; i < hw->mac.rar_entry_count; i++) {
		if (adapter->mac_table[i].state & IGB_MAC_STATE_IN_USE)
			continue;
		adapter->mac_table[i].state = (IGB_MAC_STATE_MODIFIED |
						   IGB_MAC_STATE_IN_USE);
		memcpy(adapter->mac_table[i].addr, addr, ETH_ALEN);
		adapter->mac_table[i].queue = queue;
		igb_sync_mac_table(adapter);
		return 0;
	}
	return -ENOMEM;
}
int igb_del_mac_filter(struct igb_adapter *adapter, u8* addr, u16 queue)
{
	/* search table for addr, if found, set to 0 and sync */
	int i;
	struct e1000_hw *hw = &adapter->hw;

	if (is_zero_ether_addr(addr))
		return 0;
	for (i = 0; i < hw->mac.rar_entry_count; i++) {
		if (!compare_ether_addr(addr, adapter->mac_table[i].addr) &&
		    adapter->mac_table[i].queue == queue) {
			adapter->mac_table[i].state = IGB_MAC_STATE_MODIFIED;
			memset(adapter->mac_table[i].addr, 0, ETH_ALEN);
			adapter->mac_table[i].queue = 0;
			igb_sync_mac_table(adapter);
			return 0;
		}
	}
	return -ENOMEM;
}
static int igb_set_vf_mac(struct igb_adapter *adapter,
                          int vf, unsigned char *mac_addr)
{
	igb_del_mac_filter(adapter, adapter->vf_data[vf].vf_mac_addresses, vf);
	memcpy(adapter->vf_data[vf].vf_mac_addresses, mac_addr, ETH_ALEN);

	igb_add_mac_filter(adapter, mac_addr, vf);

	return 0;
}

#ifdef IFLA_VF_MAX
static int igb_ndo_set_vf_mac(IOEthernetController *netdev, int vf, u8 *mac)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	if (!is_valid_ether_addr(mac) || (vf >= adapter->vfs_allocated_count))
		return -EINVAL;
	adapter->vf_data[vf].flags |= IGB_VF_FLAG_PF_SET_MAC;
	dev_info(&adapter->pdev->dev, "setting MAC %pM on VF %d\n", mac, vf);
	dev_info(&adapter->pdev->dev, "Reload the VF driver to make this"
				      " change effective.\n");
	if (test_bit(__IGB_DOWN, &adapter->state)) {
		dev_warn(&adapter->pdev->dev, "The VF MAC address has been set,"
			 " but the PF device is not up.\n");
		dev_warn(&adapter->pdev->dev, "Bring the PF device up before"
			 " attempting to use the VF device.\n");
	}
	return igb_set_vf_mac(adapter, vf, mac);
}

static int igb_link_mbps(int internal_link_speed)
{
	switch (internal_link_speed) {
	case SPEED_100:
		return 100;
	case SPEED_1000:
		return 1000;
	default:
		return 0;
	}
}

static void igb_set_vf_rate_limit(struct e1000_hw *hw, int vf, int tx_rate,
			int link_speed)
{
	int rf_dec, rf_int;
	u32 bcnrc_val;

	if (tx_rate != 0) {
		/* Calculate the rate factor values to set */
		rf_int = link_speed / tx_rate;
		rf_dec = (link_speed - (rf_int * tx_rate));
		rf_dec = (rf_dec * (1<<E1000_RTTBCNRC_RF_INT_SHIFT)) / tx_rate;

		bcnrc_val = E1000_RTTBCNRC_RS_ENA;
		bcnrc_val |= ((rf_int<<E1000_RTTBCNRC_RF_INT_SHIFT) &
				E1000_RTTBCNRC_RF_INT_MASK);
		bcnrc_val |= (rf_dec & E1000_RTTBCNRC_RF_DEC_MASK);
	} else {
		bcnrc_val = 0;
	}

	E1000_WRITE_REG(hw, E1000_RTTDQSEL, vf); /* vf X uses queue X */
	/*
	 * Set global transmit compensation time to the MMW_SIZE in RTTBCNRM
	 * register. MMW_SIZE=0x014 if 9728-byte jumbo is supported.
	 */
	E1000_WRITE_REG(hw, E1000_RTTBCNRM(0), 0x14);
	E1000_WRITE_REG(hw, E1000_RTTBCNRC, bcnrc_val);
}

static void igb_check_vf_rate_limit(struct igb_adapter *adapter)
{
	int actual_link_speed, i;
	bool reset_rate = false;

	/* VF TX rate limit was not set */
	if ((adapter->vf_rate_link_speed == 0) || 
		(adapter->hw.mac.type != e1000_82576))
		return;

	actual_link_speed = igb_link_mbps(adapter->link_speed);
	if (actual_link_speed != adapter->vf_rate_link_speed) {
		reset_rate = true;
		adapter->vf_rate_link_speed = 0;
		dev_info(&adapter->pdev->dev,
		"Link speed has been changed. VF Transmit rate is disabled\n");
	}

	for (i = 0; i < adapter->vfs_allocated_count; i++) {
		if (reset_rate)
			adapter->vf_data[i].tx_rate = 0;

		igb_set_vf_rate_limit(&adapter->hw, i,
			adapter->vf_data[i].tx_rate, actual_link_speed);
	}
}

static int igb_ndo_set_vf_bw(IOEthernetController *netdev, int vf, int tx_rate)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	int actual_link_speed;
	
	if (hw->mac.type != e1000_82576)
		return -EOPNOTSUPP;

	actual_link_speed = igb_link_mbps(adapter->link_speed);
	if ((vf >= adapter->vfs_allocated_count) ||
		(!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)) ||
		(tx_rate < 0) || (tx_rate > actual_link_speed))
		return -EINVAL;

	adapter->vf_rate_link_speed = actual_link_speed;
	adapter->vf_data[vf].tx_rate = (u16)tx_rate;
	igb_set_vf_rate_limit(hw, vf, tx_rate, actual_link_speed);

	return 0;
}

static int igb_ndo_get_vf_config(IOEthernetController *netdev,
				 int vf, struct ifla_vf_info *ivi)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	if (vf >= adapter->vfs_allocated_count)
		return -EINVAL;
	ivi->vf = vf;
	memcpy(&ivi->mac, adapter->vf_data[vf].vf_mac_addresses, ETH_ALEN);
	ivi->tx_rate = adapter->vf_data[vf].tx_rate;
	ivi->vlan = adapter->vf_data[vf].pf_vlan;
	ivi->qos = adapter->vf_data[vf].pf_qos;
	return 0;
}
#endif
static void igb_vmm_control(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 reg;

	switch (hw->mac.type) {
	case e1000_82575:
	default:
		/* replication is not supported for 82575 */
		return;
	case e1000_82576:
		/* notify HW that the MAC is adding vlan tags */
		reg = E1000_READ_REG(hw, E1000_DTXCTL);
		reg |= (E1000_DTXCTL_VLAN_ADDED |
			E1000_DTXCTL_SPOOF_INT);
		E1000_WRITE_REG(hw, E1000_DTXCTL, reg);
	case e1000_82580:
		/* enable replication vlan tag stripping */
		reg = E1000_READ_REG(hw, E1000_RPLOLR);
		reg |= E1000_RPLOLR_STRVLAN;
		E1000_WRITE_REG(hw, E1000_RPLOLR, reg);
	case e1000_i350:
		/* none of the above registers are supported by i350 */
		break;
	}

	/* Enable Malicious Driver Detection */
	if ((hw->mac.type == e1000_i350) && (adapter->vfs_allocated_count) &&
	    (adapter->mdd))
		igb_enable_mdd(adapter);

	/* enable replication and loopback support */
	e1000_vmdq_set_loopback_pf(hw, adapter->vfs_allocated_count ||
				   adapter->vmdq_pools);

	e1000_vmdq_set_anti_spoofing_pf(hw, adapter->vfs_allocated_count ||
					adapter->vmdq_pools,
					adapter->vfs_allocated_count);
	e1000_vmdq_set_replication_pf(hw, adapter->vfs_allocated_count ||
				      adapter->vmdq_pools);
}

static void igb_init_fw(struct igb_adapter *adapter) 
{
	struct e1000_fw_drv_info fw_cmd;
	struct e1000_hw *hw = &adapter->hw;
	int i;
	u16 mask;
	
	mask = E1000_SWFW_PHY0_SM;
	
	if (!hw->mac.ops.acquire_swfw_sync(hw, mask)) {
		for (i = 0; i <= FW_MAX_RETRIES; i++) {
			E1000_WRITE_REG(hw, E1000_FWSTS, E1000_FWSTS_FWRI);
			fw_cmd.hdr.cmd = FW_CMD_DRV_INFO;
			fw_cmd.hdr.buf_len = FW_CMD_DRV_INFO_LEN;
			fw_cmd.hdr.cmd_or_resp.cmd_resv = FW_CMD_RESERVED;
			fw_cmd.port_num = hw->bus.func;
			fw_cmd.drv_version = FW_FAMILY_DRV_VER;
			fw_cmd.hdr.checksum = 0;
			fw_cmd.hdr.checksum = e1000_calculate_checksum((u8 *)&fw_cmd,
			                                           (FW_HDR_LEN +
			                                            fw_cmd.hdr.buf_len));
			 e1000_host_interface_command(hw, (u8*)&fw_cmd,
			                             sizeof(fw_cmd));
			if (fw_cmd.hdr.cmd_or_resp.ret_status == FW_STATUS_SUCCESS)
				break;
		}
	} else
		IOLog( "Unable to get semaphore, firmware init failed.\n");
	hw->mac.ops.release_swfw_sync(hw, mask);
}

static void igb_init_dmac(struct igb_adapter *adapter, u32 pba)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 dmac_thr;
	u16 hwm;

	if (hw->mac.type > e1000_82580) {
		if (adapter->dmac != IGB_DMAC_DISABLE) {
			u32 reg;

			/* force threshold to 0.  */
			E1000_WRITE_REG(hw, E1000_DMCTXTH, 0);

			/*
			 * DMA Coalescing high water mark needs to be greater
			 * than the Rx threshold. Set hwm to PBA - max frame
			 * size in 16B units, capping it at PBA - 6KB.
			 */
			hwm = 64 * pba - adapter->max_frame_size / 16;
			if (hwm < 64 * (pba - 6))
				hwm = 64 * (pba - 6);
			reg = E1000_READ_REG(hw, E1000_FCRTC);
			reg &= ~E1000_FCRTC_RTH_COAL_MASK;
			reg |= ((hwm << E1000_FCRTC_RTH_COAL_SHIFT)
					& E1000_FCRTC_RTH_COAL_MASK);
			E1000_WRITE_REG(hw, E1000_FCRTC, reg);
			
			/* 
			 * Set the DMA Coalescing Rx threshold to PBA - 2 * max
 			 * frame size, capping it at PBA - 10KB.
 			 */
			dmac_thr = pba - adapter->max_frame_size / 512;
			if (dmac_thr < pba - 10)
				dmac_thr = pba - 10;
			reg = E1000_READ_REG(hw, E1000_DMACR);
			reg &= ~E1000_DMACR_DMACTHR_MASK;
			reg |= ((dmac_thr << E1000_DMACR_DMACTHR_SHIFT)
				& E1000_DMACR_DMACTHR_MASK);

			/* transition to L0x or L1 if available..*/
			reg |= (E1000_DMACR_DMAC_EN | E1000_DMACR_DMAC_LX_MASK);

			/* watchdog timer= msec values in 32usec intervals */
			reg |= ((adapter->dmac) >> 5);
			E1000_WRITE_REG(hw, E1000_DMACR, reg);

			/* no lower threshold to disable coalescing(smart fifb)-UTRESH=0*/
			E1000_WRITE_REG(hw, E1000_DMCRTRH, 0);

			/*
			 * This sets the time to wait before requesting transition to
			 * low power state to number of usecs needed to receive 1 512
			 * byte frame at gigabit line rate
			 */
			reg = (IGB_DMCTLX_DCFLUSH_DIS | 0x4);

			E1000_WRITE_REG(hw, E1000_DMCTLX, reg);

			/* free space in tx packet buffer to wake from DMA coal */
			E1000_WRITE_REG(hw, E1000_DMCTXTH, (IGB_MIN_TXPBSIZE -
				(IGB_TX_BUF_4096 + adapter->max_frame_size)) >> 6);

			/* make low power state decision controlled by DMA coal */
			reg = E1000_READ_REG(hw, E1000_PCIEMISC);
			reg &= ~E1000_PCIEMISC_LX_DECISION;
			E1000_WRITE_REG(hw, E1000_PCIEMISC, reg);
		} /* endif adapter->dmac is not disabled */
	} else if (hw->mac.type == e1000_82580) {
		u32 reg = E1000_READ_REG(hw, E1000_PCIEMISC);
		E1000_WRITE_REG(hw, E1000_PCIEMISC,
		                reg & ~E1000_PCIEMISC_LX_DECISION);
		E1000_WRITE_REG(hw, E1000_DMACR, 0);
	}
}
/* igb_main.c */


OSDefineMetaClassAndStructors(AppleIGB, super);


void AppleIGB::free()
{
	RELEASE(workLoop);
	RELEASE(csrPCIAddress);
	RELEASE(pdev);

	RELEASE(mediumDict);
	
	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	
	super::free();
}

bool AppleIGB::init(OSDictionary *properties)
{
	struct igb_adapter *adapter = &priv_adapter;
	
	if (super::init(properties) == false) 
		return false;
		
	enabledForNetif = false;

	pdev = NULL;
	mediumDict = NULL;
	csrPCIAddress = NULL;
	interruptSource = NULL;
	watchdogSource = NULL;
	netif = NULL;
	
	transmitQueue = NULL;
	preLinkStatus = 0;
	rxMbufCursor = NULL;
	txMbufCursor = NULL;
	
    adapter->node = -1;
	
	_mtu = 1500;
	return true;
}

// follows after igb_remove()
void AppleIGB::stop(IOService* provider)
{
	struct igb_adapter *adapter = &priv_adapter;
	
	detachInterface(netif);
	RELEASE(netif);
	/* flush_scheduled work may reschedule our watchdog task, so
	 * explicitly disable watchdog tasks from being rescheduled  */
	if(workLoop){
		if (watchdogSource) {
			workLoop->removeEventSource(watchdogSource);
			RELEASE(watchdogSource);
		}
		if (resetSource) {
			workLoop->removeEventSource(resetSource);
			RELEASE(resetSource);
		}
		if (dmaErrSource) {
			workLoop->removeEventSource(dmaErrSource);
			RELEASE(dmaErrSource);
		}
		
		if (interruptSource) {
			workLoop->removeEventSource(interruptSource);
			RELEASE(interruptSource);
		}
	}
	set_bit(__IGB_DOWN, &adapter->state);

#ifdef IGB_DCA
	if (adapter->flags & IGB_FLAG_DCA_ENABLED) {
		IOLog("DCA disabled\n");
		dca_remove_requester(&pdev->dev);
		adapter->flags &= ~IGB_FLAG_DCA_ENABLED;
		E1000_WRITE_REG(hw, E1000_DCA_CTRL, E1000_DCA_CTRL_DCA_DISABLE);
	}
#endif
	
	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant. */
	igb_release_hw_control(adapter);
#if	0
	unregister_netdev(netdev);
#endif
	
	igb_clear_interrupt_scheme(adapter);
	igb_reset_sriov_capability(adapter);
	
#if	0
	iounmap(hw->hw_addr);
	if (hw->flash_address)
		iounmap(hw->flash_address);
	pci_release_selected_regions(pdev,
	                             pci_select_bars(pdev, IORESOURCE_MEM));
#endif	
	kfree(adapter->mac_table, sizeof(struct igb_mac_addr));
	kfree(adapter->rx_ring, sizeof(struct igb_ring));
#if	0
	free_netdev(netdev);
	
	pci_disable_pcie_error_reporting(pdev);
	
	pci_disable_device(pdev);
#endif	
	
	super::stop(provider);
}

	
// igb_probe
bool AppleIGB::start(IOService* provider)
{
    bool success = false;
	
	struct igb_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u16 eeprom_data = 0;
	u8 pba_str[E1000_PBANUM_LENGTH];
	s32 ret_val;
	int err;
	static SInt8 global_quad_port_a; /* global quad port a indication */
	static SInt8 cards_found;
	
	if (super::start(provider) == false) {
		return false;
	}

	pdev = OSDynamicCast(IOPCIDevice, provider);
	if (pdev == NULL)
		return false;
	
	pdev->retain();
	if (pdev->open(this) == false)
		return false;
	
	if(!initEventSources(provider)){
		return false;
	}
	
	csrPCIAddress = pdev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (csrPCIAddress == NULL) {
		return false;
	}
	
	{
		UInt16	reg16;
		reg16	= pdev->configRead16( kIOPCIConfigCommand );
		reg16  &= ~kIOPCICommandIOSpace;
		reg16	|= ( kIOPCICommandBusMaster
					|    kIOPCICommandMemorySpace
					|	 kIOPCICommandMemWrInvalidate );
		
		pdev->configWrite16( kIOPCIConfigCommand, reg16 );
		
		// pdev->setMemoryEnable(true);
	}

	do {
#ifndef HAVE_ASPM_QUIRKS
		/* 82575 requires that the pci-e link partner disable the L0s state */
		switch (pdev->configRead16(kIOPCIConfigDeviceID)) {
			case E1000_DEV_ID_82575EB_COPPER:
			case E1000_DEV_ID_82575EB_FIBER_SERDES:
			case E1000_DEV_ID_82575GB_QUAD_COPPER:
				// I do not know how to implement this. 
				// pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S);
			default:
				break;
		}
#endif /* HAVE_ASPM_QUIRKS */
#if	0
		err = pci_request_selected_regions(pdev,
										   pci_select_bars(pdev,
														   IORESOURCE_MEM),
										   igb_driver_name);
#endif
		// pdev->setBusMasterEnable(true);
		
		//SET_MODULE_OWNER(netdev);
		//SET_NETDEV_DEV(netdev, &pdev->dev);
		
		//pci_set_drvdata(pdev, netdev);
		adapter->netdev = this;
		adapter->pdev = pdev;
		hw->back = adapter;
		adapter->port_num = hw->bus.func;
		//adapter->msg_enable = (1 << debug) - 1;
		
		hw->hw_addr = (u8*)(csrPCIAddress->getVirtualAddress());
		
#ifdef HAVE_NET_DEVICE_OPS
		//netdev->netdev_ops = &igb_netdev_ops;
#else /* HAVE_NET_DEVICE_OPS */
		//netdev->open = &igb_open;
		//netdev->stop = &igb_close;
		//netdev->get_stats = &igb_get_stats;
#ifdef HAVE_SET_RX_MODE
		//netdev->set_rx_mode = &igb_set_rx_mode;
#endif
		//netdev->set_multicast_list = &igb_set_rx_mode;
		//netdev->set_mac_address = &igb_set_mac;
		//netdev->change_mtu = &igb_change_mtu;
		//netdev->do_ioctl = &igb_ioctl;
#ifdef HAVE_TX_TIMEOUT
		//netdev->tx_timeout = &igb_tx_timeout;
#endif
		//netdev->vlan_rx_register = igb_vlan_mode;
		//netdev->vlan_rx_add_vid = igb_vlan_rx_add_vid;
		//netdev->vlan_rx_kill_vid = igb_vlan_rx_kill_vid;
		//netdev->hard_start_xmit = &igb_xmit_frame;
#endif /* HAVE_NET_DEVICE_OPS */
		//igb_set_ethtool_ops(netdev);
#ifdef HAVE_TX_TIMEOUT
		//netdev->watchdog_timeo = 5 * HZ;
#endif
		
		//strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

		adapter->bd_number = OSIncrementAtomic8(&cards_found);
		
		/* setup the private structure */
		err = igb_sw_init(adapter);
		if (err)
			goto err_sw_init;
		
		e1000_get_bus_info(hw);

		hw->phy.autoneg_wait_to_complete = FALSE;
		hw->mac.adaptive_ifs = FALSE;
		
		/* Copper options */
		if (hw->phy.media_type == e1000_media_type_copper) {
#ifdef ETH_TP_MDI_X
			hw->phy.mdix = ETH_TP_MDI_INVALID;
#else
			hw->phy.mdix = AUTO_ALL_MODES;
#endif /* ETH_TP_MDI_X */
			hw->phy.disable_polarity_correction = FALSE;
			hw->phy.ms_type = e1000_ms_hw_default;
		}
		
		if (e1000_check_reset_block(hw))
			IOLog("PHY reset is blocked due to SOL/IDER session.\n");
		
		/*
		 * features is initialized to 0 in allocation, it might have bits
		 * set by igb_sw_init so we should use an or instead of an
		 * assignment.
		 */
		_features |= NETIF_F_SG |
		NETIF_F_IP_CSUM |
#ifdef NETIF_F_IPV6_CSUM
		NETIF_F_IPV6_CSUM |
#endif
#ifdef NETIF_F_RXHASH
		NETIF_F_RXHASH |
#endif
#ifdef HAVE_NDO_SET_FEATURES
		NETIF_F_RXCSUM |
#endif
		NETIF_F_HW_VLAN_RX |
		NETIF_F_HW_VLAN_TX;
		
#ifdef HAVE_NDO_SET_FEATURES
		/* copy netdev features into list of user selectable features */
		netdev->hw_features |= _features;
#else
#ifdef NETIF_F_GRO
		
		/* this is only needed on kernels prior to 2.6.39 */
		_features |= NETIF_F_GRO;
#endif
#endif
		
		/* set this bit last since it cannot be part of hw_features */
		_features |= NETIF_F_HW_VLAN_FILTER;
		
		
		if (hw->mac.type >= e1000_82576)
			_features |= NETIF_F_SCTP_CSUM;
		
		adapter->en_mng_pt = e1000_enable_mng_pass_thru(hw);
		
		/* before reading the NVM, reset the controller to put the device in a
		 * known good starting state */
		e1000_reset_hw(hw);
		
		/* make sure the NVM is good */
		if (e1000_validate_nvm_checksum(hw) < 0) {
			IOLog( "The NVM Checksum Is Not"
				  " Valid\n");
			goto err_eeprom;
		}
		
		/* copy the MAC address out of the NVM */
		if (e1000_read_mac_addr(hw))
			IOLog( "NVM Read Error\n");
		
		if (!is_valid_ether_addr(hw->mac.addr)) {
			IOLog( "Invalid MAC Address\n");
			goto err_eeprom;
		}
		
		memcpy(&adapter->mac_table[0].addr, hw->mac.addr, ETH_ALEN);
		
		adapter->mac_table[0].queue = adapter->vfs_allocated_count;
		adapter->mac_table[0].state = (IGB_MAC_STATE_DEFAULT | IGB_MAC_STATE_IN_USE);
		igb_rar_set(adapter, 0);
		
		/* get firmware version for ethtool -i */
		e1000_read_nvm(&adapter->hw, 5, 1, &adapter->fw_version);

		adapter->watchdog_task = watchdogSource;
		if (adapter->flags & IGB_FLAG_DETECT_BAD_DMA)
			adapter->dma_err_task = dmaErrSource;
		adapter->reset_task = resetSource;
		
		rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(_mtu + ETH_HLEN + ETH_FCS_LEN, 1);
		txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(_mtu + ETH_HLEN + ETH_FCS_LEN, MAX_SKB_FRAGS);
		
		/* Initialize link properties that are user-changeable */
		adapter->fc_autoneg = true;
		hw->mac.autoneg = true;
		hw->phy.autoneg_advertised = 0x2f;
		
		hw->fc.requested_mode = e1000_fc_default;
		hw->fc.current_mode = e1000_fc_default;
		
		e1000_validate_mdi_setting(hw);
		
		/* Initial Wake on LAN setting If APM wake is enabled in the EEPROM,
		 * enable the ACPI Magic Packet filter
		 */
		
		if (hw->bus.func == 0)
			e1000_read_nvm(hw, NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		else if (hw->mac.type >= e1000_82580)
			hw->nvm.ops.read(hw, NVM_INIT_CONTROL3_PORT_A +
							 NVM_82580_LAN_FUNC_OFFSET(hw->bus.func), 1,
							 &eeprom_data);
		else if (hw->bus.func == 1)
			e1000_read_nvm(hw, NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
		
		if (eeprom_data & IGB_EEPROM_APME)
			adapter->eeprom_wol |= E1000_WUFC_MAG;
		
		/* now that we have the eeprom settings, apply the special cases where
		 * the eeprom may be wrong or the board simply won't support wake on
		 * lan on a particular port */
		switch (pdev->configRead16(kIOPCIConfigDeviceID)) {
			case E1000_DEV_ID_82575GB_QUAD_COPPER:
				adapter->eeprom_wol = 0;
				break;
			case E1000_DEV_ID_82575EB_FIBER_SERDES:
			case E1000_DEV_ID_82576_FIBER:
			case E1000_DEV_ID_82576_SERDES:
				/* Wake events only supported on port A for dual fiber
				 * regardless of eeprom setting */
				if (E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_FUNC_1)
					adapter->eeprom_wol = 0;
				break;
			case E1000_DEV_ID_82576_QUAD_COPPER:
			case E1000_DEV_ID_82576_QUAD_COPPER_ET2:
				/* if quad port adapter, disable WoL on all but port A */
				if (global_quad_port_a != 0)
					adapter->eeprom_wol = 0;
				else
					adapter->flags |= IGB_FLAG_QUAD_PORT_A;
				/* Reset for multiple quad port adapters */
				if (++global_quad_port_a == 4)
					global_quad_port_a = 0;
				break;
		}
		
		/* initialize the wol settings based on the eeprom settings */
		adapter->wol = adapter->eeprom_wol;
#if	0
		device_set_wakeup_enable(pci_dev_to_dev(adapter->pdev), adapter->wol);
#endif
		/* reset the hardware with the new settings */
		igb_reset(adapter);
		
		/* let the f/w know that the h/w is now under the control of the
		 * driver. */
		igb_get_hw_control(adapter);
		
		/* carrier off reporting is important to ethtool even BEFORE open */
		netif_carrier_off(this);
		
#ifdef IGB_DCA
		if (dca_add_requester(&pdev->dev) == E1000_SUCCESS) {
			adapter->flags |= IGB_FLAG_DCA_ENABLED;
			dev_info(pci_dev_to_dev(pdev), "DCA enabled\n");
			igb_setup_dca(adapter);
		}
		
#endif
#ifdef HAVE_HW_TIME_STAMP
		/* do hw tstamp init after resetting */
		igb_init_hw_timer(adapter);
		
#endif
		IOLog("Intel(R) Gigabit Ethernet Network Connection\n");
		/* print bus type/speed/width info */
		IOLog("%s: (PCIe:%s:%s) ",
			  "AppleIGB",
			  ((hw->bus.speed == e1000_bus_speed_2500) ? "2.5GT/s" :
			   (hw->bus.speed == e1000_bus_speed_5000) ? "5.0GT/s" :
			   "unknown"),
			  ((hw->bus.width == e1000_bus_width_pcie_x4) ? "Width x4" :
			   (hw->bus.width == e1000_bus_width_pcie_x2) ? "Width x2" :
			   (hw->bus.width == e1000_bus_width_pcie_x1) ? "Width x1" :
			   "unknown"));
		IOLog("MAC: %2x:%2x:%2x:%2x:%2x:%2x ",
			  hw->mac.addr[0],hw->mac.addr[1],hw->mac.addr[2],
			  hw->mac.addr[3],hw->mac.addr[4],hw->mac.addr[5]);
		ret_val = e1000_read_pba_string(hw, pba_str, E1000_PBANUM_LENGTH);
		IOLog("PBA No: %s\n", ret_val ? "Unknown": (char*)pba_str);

		/* Initialize the thermal sensor on i350 devices. */
		if (hw->mac.type == e1000_i350 && hw->bus.func == 0) {
			u16 ets_word;
			
			/*
			 * Read the NVM to determine if this i350 device supports an
			 * external thermal sensor.
			 */
			e1000_read_nvm(hw, NVM_ETS_CFG, 1, &ets_word);
			if (ets_word != 0x0000 && ets_word != 0xFFFF)
				adapter->ets = true;
			else
				adapter->ets = false;
#ifdef IGB_SYSFS
			igb_sysfs_init(adapter);
#else
#ifdef IGB_PROCFS
			igb_procfs_init(adapter);
#endif /* IGB_PROCFS */
#endif /* IGB_SYSFS */
		} else {
			adapter->ets = false;
		}

		switch (hw->mac.type) {
			case e1000_i350:
				/* Enable EEE for internal copper PHY devices */
				if (hw->phy.media_type == e1000_media_type_copper)
					e1000_set_eee_i350(hw);
				
				/* send driver version info to firmware */
				igb_init_fw(adapter);
				break;
			default:
				break;
		}
		IOLog(
			  "Using %s interrupts. %d rx queue(s), %d tx queue(s)\n",
			  adapter->msix_entries ? "MSI-X" :
			  (adapter->flags & IGB_FLAG_HAS_MSI) ? "MSI" : "legacy",
			  adapter->num_rx_queues, adapter->num_tx_queues);
		
		//cards_found++; -> atomic
		//pm_runtime_put_noidle(&pdev->dev);
		success = true;
		break;
		
		//igb_release_hw_control(adapter);
	err_eeprom:
		if (!e1000_check_reset_block(hw))
			e1000_phy_hw_reset(hw);

	err_sw_init:
		igb_clear_interrupt_scheme(adapter);
		igb_reset_sriov_capability(adapter);
		RELEASE(csrPCIAddress);	// iounmap(hw->hw_addr);
	} while(false);


	// Close our provider, it will be re-opened on demand when
	// our enable() is called by a client.
	pdev->close(this);
	
	do {
		if ( false == success )
			break;
		
		success = false;

		// Allocate and attach an IOEthernetInterface instance.
		if (attachInterface((IONetworkInterface**)&netif, false) == false) {
			break;
		}

		netif->registerService();
		success = true;
	} while(false);

	return success;
}


//---------------------------------------------------------------------------
bool AppleIGB::initEventSources( IOService* provider )
{
	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop* myWorkLoop = getWorkLoop();
	if (myWorkLoop == NULL) {
		return false;
	}

	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL) {
		return false;
	}
	transmitQueue->setCapacity(IGB_DEFAULT_TXD);

	interruptSource = IOInterruptEventSource::interruptEventSource(this,&AppleIGB::interruptHandler,provider);

	if (!interruptSource ||
		(myWorkLoop->addEventSource(interruptSource) != kIOReturnSuccess)) {
		return false;
	}

	watchdogSource = IOTimerEventSource::timerEventSource(this, &AppleIGB::watchdogHandler );
	getWorkLoop()->addEventSource(watchdogSource);

	resetSource = IOTimerEventSource::timerEventSource(this, &AppleIGB::resetHandler );
	getWorkLoop()->addEventSource(resetSource);

	dmaErrSource = IOTimerEventSource::timerEventSource(this, &AppleIGB::resetHandler );
	getWorkLoop()->addEventSource(dmaErrSource);
	
	// This is important. If the interrupt line is shared with other devices,
	// then the interrupt vector will be enabled only if all corresponding
	// interrupt event sources are enabled. To avoid masking interrupts for
	// other devices that are sharing the interrupt line, the event source
	// is enabled immediately.
	interruptSource->enable();
	
	
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
	if (mediumDict == NULL) {
		return false;
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
		return false;
	}
	
	return true;
}

//---------------------------------------------------------------------------
IOReturn AppleIGB::enable(IONetworkInterface * netif)
{
	if(!enabledForNetif){
		pdev->open(this);
		
		if (selectMedium(getSelectedMedium()) != kIOReturnSuccess)
			return kIOReturnIOError;
		
		if(igb_open(this))
			return kIOReturnIOError;
		
		enabledForNetif = true;
	}
	return kIOReturnSuccess; 

}

IOReturn AppleIGB::disable(IONetworkInterface * netif)
{
	if(enabledForNetif){
		enabledForNetif = false;

		igb_close(this);

		pdev->close(this);
	}
	return kIOReturnSuccess;
}



// corresponds to igb_xmit_frame
UInt32 AppleIGB::outputPacket(mbuf_t skb, void * param)
{
	struct igb_adapter *adapter = &priv_adapter;
    UInt32 rc = kIOReturnOutputSuccess;
	
	if (enabledForNetif == false) {             // drop the packet.
		return kIOReturnOutputDropped;
	}

	if (test_bit(__IGB_DOWN, &adapter->state)) {
		freePacket(skb);
		return kIOReturnOutputSuccess;
	}
	
	if (mbuf_pkthdr_len(skb) <= 0) {
		freePacket(skb);
		return kIOReturnOutputSuccess;
	}
	
	/*
	 * The minimum packet size with TCTL.PSP set is 17 so pad the skb
	 * in order to meet this minimum size requirement.
	 */
	// not applied to Mac OS X
	// igb_xmit_frame_ring is inlined here
	do {
        struct igb_ring *tx_ring = igb_tx_queue_mapping(adapter, skb);
        struct igb_tx_buffer *first;
        int tso;
        u32 tx_flags = 0;
        u8 hdr_len = 0;
        /* need: 1 descriptor per page,
         *       + 2 desc gap to keep tail from touching head,
         *       + 1 desc for skb->data,
         *       + 1 desc for context descriptor,
         * otherwise try next time */
        if (igb_maybe_stop_tx(tx_ring, MAX_SKB_FRAGS + 4)) {
            /* this is a hard error */
            rc = kIOReturnOutputDropped;
            break;
        }
        /* record the location of the first descriptor for this packet */
        first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];
        first->skb = skb;
        first->bytecount = mbuf_pkthdr_len(skb);
        first->gso_segs = 1;
        
        UInt32 vlan;
        if(getVlanTagDemand(skb,&vlan)){
			//IOLog("vlan = %d\n",(int)vlan);
            tx_flags |= IGB_TX_FLAGS_VLAN;
            tx_flags |= (vlan << IGB_TX_FLAGS_VLAN_SHIFT);
        }
        
        /* record initial flags and protocol */
        first->tx_flags = tx_flags;
        
        tso = igb_tso(tx_ring, first, &hdr_len);
        if (tso < 0){
            igb_unmap_and_free_tx_resource(adapter, tx_ring, first);
        } else {
			if (!tso)
				igb_tx_csum(tx_ring, first);
			igb_tx_map(tx_ring, first, hdr_len);
            
			/* Make sure there is space in the ring for the next send. */
			igb_maybe_stop_tx(tx_ring, MAX_SKB_FRAGS + 4);
        }
    } while(false);

    if(rc != kIOReturnOutputSuccess)
		netStats->outputErrors += 1;
	return rc;
}

void AppleIGB::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	constraints->alignStart = kIOPacketBufferAlign2;
	constraints->alignLength = kIOPacketBufferAlign1;
	return;
}

IOOutputQueue * AppleIGB::createOutputQueue()
{
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

const OSString * AppleIGB::newVendorString() const
{
	return OSString::withCString("Intel");
}

const OSString * AppleIGB::newModelString() const
{
	static struct  {
		UInt16 id;
		const char* name;
	} decieModelNames[] = 
	{
		{ E1000_DEV_ID_I350_COPPER, "i350 Copper"},
		{ E1000_DEV_ID_I350_FIBER, "i350 Fiber"},
		{ E1000_DEV_ID_I350_SERDES, "i350 SerDes"},
		{ E1000_DEV_ID_I350_SGMII, "i350 SGMII"},
		{ E1000_DEV_ID_82580_COPPER, "82580 Copper"},
		{ E1000_DEV_ID_82580_FIBER, "82580 Fiber"},
		{ E1000_DEV_ID_82580_QUAD_FIBER, "82580 Quad Fiber"},
		{ E1000_DEV_ID_82580_SERDES, "82580 SerDes"},
		{ E1000_DEV_ID_82580_SGMII, "82580 SGMII"},
		{ E1000_DEV_ID_82580_COPPER_DUAL, "82580 Dual Copper"},
		{ E1000_DEV_ID_DH89XXCC_SGMII, "DH89XXCC SGMII"},
		{ E1000_DEV_ID_DH89XXCC_SERDES, "DH89XXCC SerDes"},
		{ E1000_DEV_ID_DH89XXCC_BACKPLANE, "DH89XXCC Backplane"},
		{ E1000_DEV_ID_DH89XXCC_SFP, "DH89XXCC SFP"},
		{ E1000_DEV_ID_82576, "82576"},
		{ E1000_DEV_ID_82576_NS, "82576 NS"},
		{ E1000_DEV_ID_82576_NS_SERDES, "82576 NS SerDes"},
		{ E1000_DEV_ID_82576_FIBER, "82576 Fiber"},
		{ E1000_DEV_ID_82576_SERDES, "82576 SerDes"},
		{ E1000_DEV_ID_82576_SERDES_QUAD, "82576 Quad SerDes"},
		{ E1000_DEV_ID_82576_QUAD_COPPER_ET2, "82576 Quad Copper ET2"},
		{ E1000_DEV_ID_82576_QUAD_COPPER, "82576 Quad Copper"},
		{ E1000_DEV_ID_82575EB_COPPER, "82575EB Copper"},
		{ E1000_DEV_ID_82575EB_FIBER_SERDES, "82575EB Fiber SerDes"},
		{ E1000_DEV_ID_82575GB_QUAD_COPPER, "82575 Quad Copper"},
	};
	int k;
	for( k = 0; k < sizeof(decieModelNames)/sizeof(decieModelNames[0]); k++){
		if(priv_adapter.hw.device_id == decieModelNames[k].id )
			return OSString::withCString(decieModelNames[k].name);
	}
	return OSString::withCString("Unknown");
}

IOReturn AppleIGB::selectMedium(const IONetworkMedium * medium)
{
	bool link;
	igb_adapter *adapter = &priv_adapter;
	
	//link = e1000e_setup_copper_link(&adapter->hw);
	
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
			IOLog("AppleIGB::setCurrentMedium error.\n");
		}
	}
	
	return ( medium ? kIOReturnSuccess : kIOReturnIOError );
}

bool AppleIGB::configureInterface(IONetworkInterface * interface)
{
	
	IONetworkData * data = NULL;
	
	if (super::configureInterface(interface) == false) {
		IOLog("IOEthernetController::confiugureInterface failed.\n"); 
		return false;
	}
	
	// Get the generic network statistics structure.
	data = interface->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		IOLog("netif getParameter NetworkStatsKey failed."); 
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = interface->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		IOLog("netif getParameter kIOEthernetStatsKey failed."); 
		return false;
	}
	
	return true;
}

bool AppleIGB::createWorkLoop()
{
	workLoop = IOWorkLoop::workLoop();	
	return (workLoop !=  NULL);
}

IOWorkLoop * AppleIGB::getWorkLoop() const
{
	return workLoop;
}

//-----------------------------------------------------------------------
// Methods inherited from IOEthernetController.
//-----------------------------------------------------------------------

IOReturn AppleIGB::getHardwareAddress(IOEthernetAddress * addr)
{
	memcpy(addr->bytes, priv_adapter.hw.mac.addr, kIOEthernetAddressSize);
	return kIOReturnSuccess;
}

// corresponds to igb_set_mac
IOReturn AppleIGB::setHardwareAddress(const IOEthernetAddress * addr)
{
	igb_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;

	

	
	igb_del_mac_filter(adapter, hw->mac.addr,
					   adapter->vfs_allocated_count);
	memcpy(adapter->hw.mac.addr, addr->bytes, kIOEthernetAddressSize);
	
	/* set the correct pool for the new PF MAC address in entry 0 */
	igb_add_mac_filter(adapter, hw->mac.addr,
							  adapter->vfs_allocated_count);
	
	return kIOReturnSuccess;
}
IOReturn AppleIGB::setPromiscuousMode(bool active)
{
	if(active)
		iff_flags |= IFF_PROMISC;
	else
		iff_flags &= ~IFF_PROMISC;

	igb_set_rx_mode(this);
	return kIOReturnSuccess;
}
	
IOReturn AppleIGB::setMulticastMode(bool active)
{
	if(active)
		iff_flags |= IFF_ALLMULTI;
	else
		iff_flags &= IFF_ALLMULTI;
	
	igb_set_rx_mode(this);
	return kIOReturnSuccess;
}

// corresponds to igb_write_mc_addr_list	
IOReturn AppleIGB::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	igb_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;

	if (!count) {
		e1000_update_mc_addr_list(hw, NULL, 0);
		return 0;
	}
	
	/* The shared function expects a packed array of only addresses. */
	e1000_update_mc_addr_list(hw, (u8*)addrs, count);
	
	return kIOReturnSuccess;
}

IOReturn AppleIGB::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	*checksumMask = 0;
	if( checksumFamily != kChecksumFamilyTCPIP ) {
		IOLog("AppleIGB: Operating system wants information for unknown checksum family.\n");
		return kIOReturnUnsupported;
	}
	
	if( !isOutput ) {
		*checksumMask = kChecksumTCP | kChecksumUDP | kChecksumIP;
	} else {
		*checksumMask = kChecksumTCP | kChecksumUDP;
	}
	return kIOReturnSuccess;
}

//-----------------------------------------------------------------------
// e1000e private functions
//-----------------------------------------------------------------------
bool AppleIGB::addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) {
		IOLog("Couldn't allocate medium.\n");
		return false;
	}
	
	if (!IONetworkMedium::addMedium(mediumDict, medium)) {
		IOLog("Couldn't add medium.\n");
		return false;
	}
	
	mediumTable[index] = medium;
	medium->release();
	return true;
}

// corresponds to igb-intr
void AppleIGB::interruptOccurred(IOInterruptEventSource * src, int count)
{
	struct igb_adapter *adapter = &priv_adapter;
	struct igb_q_vector *q_vector = adapter->q_vector[0];
	struct e1000_hw *hw = &adapter->hw;
	/* Interrupt Auto-Mask...upon reading ICR, interrupts are masked.  No
	 * need for the IMC write */
	u32 icr = E1000_READ_REG(hw, E1000_ICR);
	
	/* IMS will not auto-mask if INT_ASSERTED is not set, and if it is
	 * not set, then the adapter didn't send an interrupt */
	if (!(icr & E1000_ICR_INT_ASSERTED))
		return;
	
	igb_write_itr(q_vector);
	
	if (icr & E1000_ICR_DRSTA)
		igb_reinit_locked(adapter);
	
	if (icr & E1000_ICR_DOUTSYNC) {
		/* HW is reporting DMA is out of sync */
		adapter->stats.doosync++;
	}
	
	if (icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = 1;
		/* guard against interrupt when we're going down */
		if (!test_bit(__IGB_DOWN, &adapter->state))
			watchdogSource->setTimeoutMS(1);
	}
	
	igb_poll(q_vector, 64);
}

void AppleIGB::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	AppleIGB * me = (AppleIGB *) target;
	me->interruptOccurred(src, count);
}


// corresponds to igb_watchdog_task	
void AppleIGB::watchdogTask()
{
	struct igb_adapter *adapter = &priv_adapter;
	struct e1000_hw *hw = &adapter->hw;
	u32 link;
	int i;
	u32 thstat, ctrl_ext;
	
	
	link = igb_has_link(adapter);
	if (link) {
		/* Cancel scheduled suspend requests. */
//		pm_runtime_resume(netdev->dev.parent);
		
		if (!netif_carrier_ok(this)) {
			u32 ctrl;
			e1000_get_speed_and_duplex(hw,
			                           &adapter->link_speed,
			                           &adapter->link_duplex);
			
			ctrl = E1000_READ_REG(hw, E1000_CTRL);
			/* Links status message must follow this format */
			IOLog("igb: Link is Up %d Mbps %s, "
				   "Flow Control: %s\n",
			       adapter->link_speed,
			       adapter->link_duplex == FULL_DUPLEX ?
				   "Full Duplex" : "Half Duplex",
			       ((ctrl & E1000_CTRL_TFCE) &&
			        (ctrl & E1000_CTRL_RFCE)) ? "RX/TX":
			       ((ctrl & E1000_CTRL_RFCE) ?  "RX" :
					((ctrl & E1000_CTRL_TFCE) ?  "TX" : "None")));
			/* adjust timeout factor according to speed/duplex */
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
				case SPEED_10:
					adapter->tx_timeout_factor = 14;
					break;
				case SPEED_100:
					/* maybe add some timeout factor ? */
					break;
			}
			
			netif_carrier_on(this);
			netif_tx_wake_all_queues(this);
			
			igb_ping_all_vfs(adapter);
#ifdef IFLA_VF_MAX
			igb_check_vf_rate_limit(adapter);
#endif /* IFLA_VF_MAX */
			
			/* link state has changed, schedule phy info update */
			if (!test_bit(__IGB_DOWN, &adapter->state))
				updatePhyInfoTask();
		}
	} else {
		if (netif_carrier_ok(this)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			/* check for thermal sensor event on i350 */
			if (hw->mac.type == e1000_i350) {
				thstat = E1000_READ_REG(hw, E1000_THSTAT);
				ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
				if ((hw->phy.media_type ==
					 e1000_media_type_copper) &&
					!(ctrl_ext &
					  E1000_CTRL_EXT_LINK_MODE_SGMII)) {
						if (thstat & E1000_THSTAT_PWR_DOWN) {
							IOLog("igb: The "
								   "network adapter was stopped "
								  "because it overheated.\n");
						}
						if (thstat & E1000_THSTAT_LINK_THROTTLE) {
							IOLog("igb: The network "
								   "adapter supported "
								   "link speed "
								   "was downshifted "
								   "because it "
								   "overheated.\n" );
						}
					}
			}
			
			/* Links status message must follow this format */
			IOLog("igb: NIC Link is Down\n");
			netif_carrier_off(this);
			netif_tx_stop_all_queues(this);
			
			igb_ping_all_vfs(adapter);
			
			/* link state has changed, schedule phy info update */
			if (!test_bit(__IGB_DOWN, &adapter->state))
				updatePhyInfoTask();
		}
	}
	
	igb_update_stats(adapter);
	
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igb_ring *tx_ring = adapter->tx_ring[i];
		if (!netif_carrier_ok(this)) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context). */
			if (igb_desc_unused(tx_ring) + 1 < tx_ring->count) {
				adapter->tx_timeout_count++;
				schedule_work(&adapter->reset_task);
				/* return immediately since reset is imminent */
				return;
			}
		}
		
		/* Force detection of hung controller every watchdog period */
		set_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags);
	}
	
	/* Cause software interrupt to ensure rx ring is cleaned */
	if (adapter->msix_entries) {
		u32 eics = 0;
		for (i = 0; i < adapter->num_q_vectors; i++)
			eics |= adapter->q_vector[i]->eims_value;
		E1000_WRITE_REG(hw, E1000_EICS, eics);
	} else {
		E1000_WRITE_REG(hw, E1000_ICS, E1000_ICS_RXDMT0);
	}
	
	igb_spoof_check(adapter);
	
	/* Reset the timer */
	if (!test_bit(__IGB_DOWN, &adapter->state))
		watchdogSource->setTimeoutMS(200);
}

// corresponds to igb_update_phy_info
void AppleIGB::updatePhyInfoTask()
{
}
	
void AppleIGB::watchdogHandler(OSObject * target, IOTimerEventSource * src)
{
	AppleIGB* me = (AppleIGB*) target;
	me->watchdogTask();
	me->watchdogSource->setTimeoutMS(200);
}
	
void AppleIGB::resetHandler(OSObject * target, IOTimerEventSource * src)
{
	AppleIGB* me = (AppleIGB*) target;
	
	if(src == me->resetSource)
		igb_reinit_locked(&me->priv_adapter);
	else if(src == me->dmaErrSource)
		igb_dma_err_task(&me->priv_adapter,src);
}


IOReturn AppleIGB::registerWithPolicyMaker ( IOService * policyMaker )
{
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	powerState = 1;
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}

IOReturn AppleIGB::setPowerState( unsigned long powerStateOrdinal,
								IOService *policyMaker )
{
	if (powerState == powerStateOrdinal)
		return IOPMAckImplied;
	powerState = powerStateOrdinal;

	/* acknowledge the completion of our power state change */
    return IOPMAckImplied;
}

IOReturn AppleIGB::getMaxPacketSize (UInt32 *maxSize) const {
	if (maxSize)
		*maxSize = 9238;  // or mtu = 9216 ?

	return kIOReturnSuccess;
}

IOReturn AppleIGB::getMinPacketSize (UInt32 *minSize) const {
	if(minSize)
		*minSize = ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN;
	
	return kIOReturnSuccess;
}


IOReturn AppleIGB::setMaxPacketSize (UInt32 maxSize){

	if(maxSize != _mtu){
        _mtu = maxSize;
        igb_change_mtu(this,maxSize);

        RELEASE(rxMbufCursor);
        RELEASE(txMbufCursor);

        UInt32 max_frame = _mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
        rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, 1);
        txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(max_frame, MAX_SKB_FRAGS);
	}
	return kIOReturnSuccess;
}

IOReturn AppleIGB::setWakeOnMagicPacket(bool active)
{
	igb_adapter *adapter = &priv_adapter;
	if(active){
       if (adapter->eeprom_wol == 0)
           return kIOReturnUnsupported;   
		adapter->wol = adapter->eeprom_wol;
	} else {
		adapter->wol = 0;
	}
	return kIOReturnSuccess;   
}

IOReturn AppleIGB::getPacketFilters(const OSSymbol * group, UInt32 * filters) const {
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

UInt32 AppleIGB::getFeatures() const {
	return kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan;
}
    
    

/**
 * Linux porting helpers
 **/


void AppleIGB::startTxQueue()
{
	transmitQueue->start();
}

void AppleIGB::stopTxQueue()
{
	transmitQueue->stop();
	transmitQueue->flush();
}

void AppleIGB::rxChecksumOK( mbuf_t skb, UInt32 flag )
{
	setChecksumResult(skb, kChecksumFamilyTCPIP, flag, flag );
}

dma_addr_t AppleIGB::mapSingle(mbuf_t skb)
{
	struct IOPhysicalSegment vector;
	if (rxMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &vector, 1) == 0)
		return NULL;
	return vector.location;
}
	
bool AppleIGB::carrier()
{
	return (preLinkStatus & kIONetworkLinkActive) != 0;

}
	
void AppleIGB::setCarrier(bool stat)
{
	if(stat){
		preLinkStatus = kIONetworkLinkValid | kIONetworkLinkActive;
		UInt64 speed = 1000 * MBit;
		switch (priv_adapter.link_speed) {
			case SPEED_10:
				speed = 10 * MBit;
				break;
			case SPEED_100:
				speed = 100 * MBit;
				break;
		}
		setLinkStatus(preLinkStatus, getCurrentMedium(), speed);
	} else {
		preLinkStatus = kIONetworkLinkValid;
		setLinkStatus(preLinkStatus);
	}
	
}
	
void AppleIGB::receive(mbuf_t skb, UInt32 vlanTag)
{
	if(vlanTag){
		//IOLog("receive: vlan = %d\n",(int)vlanTag);
		setVlanTag(skb, vlanTag);
	}
	netif->inputPacket(skb);
}

void AppleIGB::receiveError(mbuf_t skb)
{
	freePacket(skb);
	netStats->inputErrors += 1;
}

bool AppleIGB::netifAttach()
{
	return attachInterface((IONetworkInterface **)(&netif), true);
}

void AppleIGB::netifDetach()
{
	detachInterface(netif);
	RELEASE(netif);
}



