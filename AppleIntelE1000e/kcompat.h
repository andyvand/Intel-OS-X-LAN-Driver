/*******************************************************************************

  Macros to compile Intel PRO/1000 Linux driver almost-as-is for Mac OS X.
 
*******************************************************************************/

#ifndef _KCOMPAT_H_
#define _KCOMPAT_H_

#define	s64	__int64_t
#define	s32	__int32_t
#define	s16	__int16_t
#define	s8	__int8_t
#define	u64	__uint64_t
#define	u32	__uint32_t
#define	u16	__uint16_t
#define	u8	__uint8_t

#ifndef __le16
#define __le16 __uint16_t
#endif
#ifndef __le32
#define __le32 __uint32_t
#endif
#ifndef __le64
#define __le64 __uint64_t
#endif
#ifndef __be16
#define __be16 __uint16_t
#endif
#ifndef __be32
#define __be32 __uint32_t
#endif
#ifndef __be64
#define __be64 __uint64_t
#endif

#define	sk_buff	__mbuf

#define	__iomem
#define __devinit

#define	dma_addr_t	IOPhysicalAddress

#define	____cacheline_aligned_in_smp

#define true 1
#define false 0

#define min_t(type,x,y) \
	({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })

#define cpu_to_le16(x)	OSSwapHostToLittleConstInt16(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)
#define	cpu_to_le64(x)	OSSwapHostToLittleConstInt64(x)
#define	le16_to_cpu(x)	OSSwapLittleToHostInt16(x)
#define	le32_to_cpu(x)	OSSwapLittleToHostInt32(x)
#if		defined(__BIG_ENDIAN__)
#define	le16_to_cpus(x)	(*(x)=le16_to_cpu(*(x)))
#else
#define	le16_to_cpus(x)
#endif

#define	writel(val, reg)	_OSWriteInt32(reg, 0, val)
#define	writew(val, reg)	_OSWriteInt16(reg, 0, val)
#define	readl(reg)	_OSReadInt32(reg, 0)
#define	readw(reg)	_OSReadInt16(reg, 0)

#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))

#define BITS_PER_LONG   32

#define BITS_TO_LONGS(bits) \
(((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)

/* GFP_ATOMIC means both !wait (__GFP_WAIT not set) and use emergency pool */
#define GFP_ATOMIC      0

struct net_device {
	u32 mtu;
};
struct pci_dev {
	u16 vendor;
	u16 device;
};

struct net_device_stats {
	unsigned long	rx_packets;				/* total packets received       */
	unsigned long	tx_packets;				/* total packets transmitted    */
	unsigned long	rx_bytes;				/* total bytes received         */
	unsigned long	tx_bytes;				/* total bytes transmitted      */
	unsigned long	rx_errors;				/* bad packets received         */
	unsigned long	tx_errors;				/* packet transmit problems     */
	unsigned long	rx_dropped;				/* no space in linux buffers    */
	unsigned long	tx_dropped;				/* no space available in linux  */
	unsigned long	multicast;				/* multicast packets received   */
	unsigned long	collisions;

	/* detailed rx_errors: */
	unsigned long	rx_length_errors;
	unsigned long	rx_over_errors;			/* receiver ring buff overflow  */
	unsigned long	rx_crc_errors;			/* recved pkt with crc error    */
	unsigned long	rx_frame_errors;		/* recv'd frame alignment error */
	unsigned long	rx_fifo_errors;			/* recv'r fifo overrun          */
	unsigned long	rx_missed_errors;		/* receiver missed packet       */

	/* detailed tx_errors */
	unsigned long	tx_aborted_errors;
	unsigned long	tx_carrier_errors;
	unsigned long	tx_fifo_errors;
	unsigned long	tx_heartbeat_errors;
	unsigned long	tx_window_errors;

	/* for cslip etc */
	unsigned long	rx_compressed;
	unsigned long	tx_compressed;
};

struct list_head {
	struct list_head *next, *prev;
};

struct timer_list {
	struct list_head entry;
	unsigned long expires;

	//spinlock_t lock;
	unsigned long magic;

	void (*function)(unsigned long);
	unsigned long data;

	//struct tvec_t_base_s *base;
};

struct work_struct {
	unsigned long pending;
	struct list_head entry;
	void (*func)(void *);
	void *data;
	void *wq_data;
	struct timer_list timer;
};

#define IFNAMSIZ        16

#define ETH_ALEN		6			/* Octets in one ethernet addr   */
#define ETH_HLEN		14			/* Total octets in header.       */
#define ETH_ZLEN		60			/* Min. octets in frame sans FCS */
#define ETH_DATA_LEN	1500		/* Max. octets in payload        */
#define ETH_FRAME_LEN	1514		/* Max. octets in frame sans FCS */
#define ETH_FCS_LEN		4			/* Octets in the FCS             */

#define VLAN_HLEN		4			/* The additional bytes (on top of the Ethernet header) that VLAN requires. */
#define VLAN_ETH_ALEN	6			/* Octets in one ethernet addr   */
#define VLAN_ETH_HLEN	18			/* Total octets in header.       */
#define VLAN_ETH_ZLEN	64			/* Min. octets in frame sans FCS */
#define VLAN_N_VID              4096

#define NET_IP_ALIGN	2
#ifndef NETIF_F_RXFCS
#define NETIF_F_RXFCS	0
#endif /* NETIF_F_RXFCS */
#ifndef NETIF_F_RXALL
#define NETIF_F_RXALL	0
#endif /* NETIF_F_RXALL */

#define	PCI_EXP_DEVCTL	8
#define	PCI_EXP_DEVCTL_CERE	0x0001	/* Correctable Error Reporting En. */
#define	PCI_EXP_LNKCTL	16
#define PCIE_LINK_STATE_L0S     1
#define PCIE_LINK_STATE_L1 2

#define MAX_NUMNODES 1
#define first_online_node 0
#define node_online(node) ((node) == 0)
#define ether_crc_le(length, data) _kc_ether_crc_le(length, data)
#ifndef is_zero_ether_addr
#define is_zero_ether_addr _kc_is_zero_ether_addr
static inline int _kc_is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}
#endif
#ifndef is_multicast_ether_addr
#define is_multicast_ether_addr _kc_is_multicast_ether_addr
static inline int _kc_is_multicast_ether_addr(const u8 *addr)
{
	return addr[0] & 0x01;
}
#endif /* is_multicast_ether_addr */

static inline unsigned int _kc_ether_crc_le(int length, unsigned char *data)
{
	unsigned int crc = 0xffffffff;  /* Initial value. */
	while(--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 8; --bit >= 0; current_octet >>= 1) {
			if ((crc ^ current_octet) & 1) {
				crc >>= 1;
				crc ^= 0xedb88320U;
			} else
				crc >>= 1;
		}
	}
	return crc;
}

#define	EIO		5
#define	ENOMEM	12
#define	EBUSY	16
/*****************************************************************************/
#define msleep(x)	IOSleep(x)
#define udelay(x)	IODelay(x)

#define mdelay(x)	for(int i = 0; i < x; i++ )udelay(1000)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define usleep_range(min, max)	msleep(DIV_ROUND_UP(min, 1000))	

#define schedule_work(a)

/*****************************************************************************/

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/************** Ugly macros to compile ich8lan.c *****************************/

#define	DEFINE_MUTEX(x)	void x##_dummy(){}
#define	mutex_lock(x)	IOLockLock(*x)
#define	mutex_unlock(x)	IOLockUnlock(*x)

#ifndef __cplusplus
typedef void IOBufferMemoryDescriptor;
#endif

#define	prefetch(x)
#define	unlikely(x)	(x)
#define	likely(x)	(x)
#define	BUG()
#define	wmb()	OSSynchronizeIO()
#define	mmiowb()	OSSynchronizeIO()

#define	__MODULE_STRING(s)	"x"
#define synchronize_irq(x) 

static inline int test_bit(int nr, const volatile unsigned long * addr) {
	return (*addr & (1<<nr)) != 0;
}

static inline void set_bit(int nr, volatile unsigned long * addr) {
	*addr |= (1 << nr);
}

static inline void clear_bit(int nr, volatile unsigned long * addr) {
	*addr &= ~(1 << nr);
}

static inline int test_and_set_bit(int nr, volatile unsigned long * addr) {
	int rc = test_bit(nr,addr);
	set_bit(nr,addr);
	return rc;
}


#endif /* _KCOMPAT_H_ */
