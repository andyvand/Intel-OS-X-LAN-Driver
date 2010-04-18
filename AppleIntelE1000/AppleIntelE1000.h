#ifndef __INTEL_E1000_H__
#define __INTEL_E1000_H__

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

class AppleIntelE1000: public IOEthernetController
{
	
	OSDeclareDefaultStructors(AppleIntelE1000);
	
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
	
	IOMbufNaturalMemoryCursor * rxMbufCursor;
	IOMbufNaturalMemoryCursor * txMbufCursor;
	
	bool interruptEnabled;
	bool enabledForNetif;
	bool promiscusMode;
	bool multicastMode;
	UInt32 preLinkStatus;
	UInt32 mtu;
	UInt32 powerState;
	
	bool suspend;
	UInt32 features;
	
	struct e1000_adapter adapter;
	
private:
	void interruptOccurred(IOInterruptEventSource * src);
	void timeoutOccurred(IOTimerEventSource* src);
	bool addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);
	
	bool initEventSources( IOService* provider );
	bool initPCIConfigSpace(IOPCIDevice* provider);
	
	void e1000_clean_rx_ring();
public:
	void e1000_set_multi();
	void e1000_set_mtu(UInt32 maxSize);
	IOPhysicalAddress mapRxSingle(mbuf_t);
	void e1000_rx_checksum(mbuf_t, u32);
	bool e1000_tx_csum(struct e1000_tx_ring *tx_ring,  mbuf_t skb);
	void e1000_receive_skb( mbuf_t, UInt32 len, u8 status, u16 vlan );
	void e1000_rx_err( int  );
	void e1000_tx_err( int  );
	void stopQueue();

public:
	static void interruptHandler(OSObject * target, IOInterruptEventSource * src, int count );
	
	static bool interruptFilter(OSObject * target , IOFilterInterruptEventSource * src);
	
	static void timeoutHandler(OSObject * target, IOTimerEventSource * src);
	UInt32 interruptReason;
	
};


#endif //__INTEL_E1000_H__
