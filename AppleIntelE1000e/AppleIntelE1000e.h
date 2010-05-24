#ifndef __INTEL_E1000E_H__
#define __INTEL_E1000E_H__

#define MBit 1000000

enum {
	eePowerStateOff = 0,
	eePowerStateOn,
	eePowerStateCount
};

enum {
	kFiveSeconds = 5000000
};


enum
{
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_10HD,
	MEDIUM_INDEX_10FD,
	MEDIUM_INDEX_100HD,
	MEDIUM_INDEX_100FD,
	MEDIUM_INDEX_1000FD,
	MEDIUM_INDEX_COUNT
};

enum 
{
	kActivationLevelNone = 0,  /* adapter shut off */
	kActivationLevelKDP,       /* adapter partially up to support KDP */
	kActivationLevelBSD        /* adapter fully up to support KDP and BSD */
};

enum
{
	kFullInitialization = 0,
	kResetChip          = 1
};

#define	MAX_RX_SIZE	(kIOEthernetAddressSize+kIOEthernetMaxPacketSize)
#define	SIZE_RING_DESC	PAGE_SIZE
#define NUM_RING_FRAME	256

class AppleIntelE1000e: public IOEthernetController
{
	
	OSDeclareDefaultStructors(AppleIntelE1000e);
	
public:
	// --------------------------------------------------
	// IOService (or its superclass) methods.
	// --------------------------------------------------
	
	virtual bool start(IOService * provider);
	virtual void stop(IOService * provider);
	virtual bool init(OSDictionary *properties);
	virtual void free();
	
	// --------------------------------------------------
	// Power Management Support
	// --------------------------------------------------
	virtual IOReturn registerWithPolicyMaker(IOService* policyMaker);
    virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService *policyMaker );
	
	// --------------------------------------------------
	// IONetworkController methods.
	// --------------------------------------------------
	
	virtual IOReturn enable(IONetworkInterface * netif);
	virtual IOReturn disable(IONetworkInterface * netif);
	
	virtual UInt32 outputPacket(mbuf_t m, void * param);
	
	virtual void getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const;
	
	virtual IOOutputQueue * createOutputQueue();
	
	virtual const OSString * newVendorString() const;
	virtual const OSString * newModelString() const;
	
	virtual IOReturn selectMedium(const IONetworkMedium * medium);
	virtual bool configureInterface(IONetworkInterface * interface);
	
	virtual bool createWorkLoop();
	virtual IOWorkLoop * getWorkLoop() const;
	
	//-----------------------------------------------------------------------
	// Methods inherited from IOEthernetController.
	//-----------------------------------------------------------------------
	
	virtual IOReturn getHardwareAddress(IOEthernetAddress * addr);
	virtual IOReturn setHardwareAddress(const IOEthernetAddress * addr);
	virtual IOReturn setPromiscuousMode(bool active);
	virtual IOReturn setMulticastMode(bool active);
	virtual IOReturn setMulticastList(IOEthernetAddress * addrs, UInt32 count);
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
	virtual IOReturn setMaxPacketSize (UInt32 maxSize);
	virtual IOReturn getMaxPacketSize (UInt32 *maxSize) const;
	virtual IOReturn getMinPacketSize (UInt32 *minSize) const;
	
private:
	IOWorkLoop * workLoop;
	IOPCIDevice * pciDevice;
	OSDictionary * mediumDict;
	IONetworkMedium * mediumTable[MEDIUM_INDEX_COUNT];
	IOOutputQueue * transmitQueue;
	
	IOInterruptEventSource * interruptSource;
	IOTimerEventSource * watchdogSource;
	
	IOEthernetInterface * netif;
	IONetworkStats * netStats;
	IOEthernetStats * etherStats;
    
	IOMemoryMap * csrPCIAddress;
	IOMemoryMap * flashPCIAddress;
	IOVirtualAddress csrCPUAddress;
	IOVirtualAddress flashCPUAddress;
	
	IOMbufNaturalMemoryCursor * rxMbufCursor;
	IOMbufNaturalMemoryCursor * txMbufCursor;
	
	bool interruptEnabled;
	bool enabledForNetif;
	bool promiscusMode;
	bool multicastMode;
	UInt32 preLinkStatus;
	UInt32 mtu;
	UInt32 powerState;

	bool resumeState;
	bool suspend;
	
	struct e1000_adapter adapter;
	struct pci_dev priv_pdev;
	
	struct e1000_ring rx_ring;
	struct e1000_ring tx_ring;
	struct e1000_buffer rx_buffer_info[NUM_RING_FRAME];
	struct e1000_buffer tx_buffer_info[NUM_RING_FRAME];
private:
	void interruptOccurred(IOInterruptEventSource * src);
	void timeoutOccurred(IOTimerEventSource* src);
	bool addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);

	bool initEventSources( IOService* provider );
	bool initPCIConfigSpace(IOPCIDevice* provider);

	void e1000_print_link_info();
	void e1000_free_rx_resources();
	void e1000_clean_rx_ring();
	void e1000_free_tx_resources();
	void e1000_clean_tx_ring();
	void e1000e_reset();
	bool e1000e_has_link();
	void e1000e_enable_receives();
	bool e1000_clean_tx_irq();
	bool e1000_clean_rx_irq();
	void e1000_alloc_rx_buffers();
	int  e1000_desc_unused(struct e1000_ring *ring);
	void e1000_configure_rx();
	void e1000_setup_rctl();
	void e1000_init_manageability_pt();
	void e1000_configure_tx();
	void e1000_set_multi();
	bool e1000_tx_csum(mbuf_t skb);
	void e1000_irq_enable();
	void e1000_irq_disable();
	void e1000_configure();
	void e1000_set_mtu(UInt32 maxSize);
	
public:
	static void interruptHandler(OSObject * target,
								 IOInterruptEventSource * src,
								 int count );
	
	static bool interruptFilter(OSObject * target ,
								IOFilterInterruptEventSource * src);
	
	static void timeoutHandler(OSObject * target, IOTimerEventSource * src);
	UInt32 interruptReason;
	
};


#endif //__INTEL_E1000E_H__
