/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _ALX_H_
#define _ALX_H_

#if 0
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_vlan.h>
#include <linux/mii.h>
#include <linux/aer.h>
#include <linux/bitops.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>
#include <linux/mdio.h>
#endif

#include "alx_hw.h"

/* specific error info */
#define ALX_ERR_SUCCESS          0x0000
#define ALX_ERR_ALOAD            0x0001
#define ALX_ERR_RSTMAC           0x0002
#define ALX_ERR_PARM             0x0003
#define ALX_ERR_MIIBUSY          0x0004


/* Transmit Packet Descriptor, contains 4 32-bit words.
 *
 *   31               16               0
 *   +----------------+----------------+
 *   |    vlan-tag    |   buf length   |
 *   +----------------+----------------+
 *   |              Word 1             |
 *   +----------------+----------------+
 *   |      Word 2: buf addr lo        |
 *   +----------------+----------------+
 *   |      Word 3: buf addr hi        |
 *   +----------------+----------------+
 *
 * Word 2 and 3 combine to form a 64-bit buffer address
 *
 * Word 1 has three forms, depending on the state of bit 8/12/13:
 * if bit8 =='1', the definition is just for custom checksum offload.
 * if bit8 == '0' && bit12 == '1' && bit13 == '1', the *FIRST* descriptor
 *     for the skb is special for LSO V2, Word 2 become total skb length ,
 *     Word 3 is meaningless.
 * other condition, the definition is for general skb or ip/tcp/udp
 *     checksum or LSO(TSO) offload.
 *
 * Here is the depiction:
 *
 *   0-+                                  0-+
 *   1 |                                  1 |
 *   2 |                                  2 |
 *   3 |    Payload offset                3 |    L4 header offset
 *   4 |        (7:0)                     4 |        (7:0)
 *   5 |                                  5 |
 *   6 |                                  6 |
 *   7-+                                  7-+
 *   8      Custom csum enable = 1        8      Custom csum enable = 0
 *   9      General IPv4 checksum         9      General IPv4 checksum
 *   10     General TCP checksum          10     General TCP checksum
 *   11     General UDP checksum          11     General UDP checksum
 *   12     Large Send Segment enable     12     Large Send Segment enable
 *   13     Large Send Segment type       13     Large Send Segment type
 *   14     VLAN tagged                   14     VLAN tagged
 *   15     Insert VLAN tag               15     Insert VLAN tag
 *   16     IPv4 packet                   16     IPv4 packet
 *   17     Ethernet frame type           17     Ethernet frame type
 *   18-+                                 18-+
 *   19 |                                 19 |
 *   20 |                                 20 |
 *   21 |   Custom csum offset            21 |
 *   22 |       (25:18)                   22 |
 *   23 |                                 23 |   MSS (30:18)
 *   24 |                                 24 |
 *   25-+                                 25 |
 *   26-+                                 26 |
 *   27 |                                 27 |
 *   28 |   Reserved                      28 |
 *   29 |                                 29 |
 *   30-+                                 30-+
 *   31     End of packet                 31     End of packet
 */

struct tpd_desc {
	__le32 word0;
	__le32 word1;
	union {
		__le64 addr;
		struct {
			__le32 pkt_len;
			__le32 resvd;
		} l;
	} adrl;
} __packed;

/* tpd word 0 */
#define TPD_BUFLEN_MASK			0xFFFF
#define TPD_BUFLEN_SHIFT		0
#define TPD_VLTAG_MASK			0xFFFF
#define TPD_VLTAG_SHIFT			16

/* tpd word 1 */
#define TPD_CXSUMSTART_MASK		0x00FF
#define TPD_CXSUMSTART_SHIFT		0
#define TPD_L4HDROFFSET_MASK		0x00FF
#define TPD_L4HDROFFSET_SHIFT		0
#define TPD_CXSUM_EN_MASK		0x0001
#define TPD_CXSUM_EN_SHIFT		8
#define TPD_IP_XSUM_MASK		0x0001
#define TPD_IP_XSUM_SHIFT		9
#define TPD_TCP_XSUM_MASK		0x0001
#define TPD_TCP_XSUM_SHIFT		10
#define TPD_UDP_XSUM_MASK		0x0001
#define TPD_UDP_XSUm_SHIFT		11
#define TPD_LSO_EN_MASK			0x0001
#define TPD_LSO_EN_SHIFT		12
#define TPD_LSO_V2_MASK			0x0001
#define TPD_LSO_V2_SHIFT		13
#define TPD_VLTAGGED_MASK		0x0001
#define TPD_VLTAGGED_SHIFT		14
#define TPD_INS_VLTAG_MASK		0x0001
#define TPD_INS_VLTAG_SHIFT		15
#define TPD_IPV4_MASK			0x0001
#define TPD_IPV4_SHIFT			16
#define TPD_ETHTYPE_MASK		0x0001
#define TPD_ETHTYPE_SHIFT		17
#define TPD_CXSUMOFFSET_MASK		0x00FF
#define TPD_CXSUMOFFSET_SHIFT		18
#define TPD_MSS_MASK			0x1FFF
#define TPD_MSS_SHIFT			18
#define TPD_EOP_MASK			0x0001
#define TPD_EOP_SHIFT			31

#define DESC_GET(_x, _name) ((_x) >> _name##SHIFT & _name##MASK)

/* Receive Free Descriptor */
struct rfd_desc {
	__le64 addr;		/* data buffer address, length is
				 * declared in register --- every
				 * buffer has the same size
				 */
} __packed;

/* Receive Return Descriptor, contains 4 32-bit words.
 *
 *   31               16               0
 *   +----------------+----------------+
 *   |              Word 0             |
 *   +----------------+----------------+
 *   |     Word 1: RSS Hash value      |
 *   +----------------+----------------+
 *   |              Word 2             |
 *   +----------------+----------------+
 *   |              Word 3             |
 *   +----------------+----------------+
 *
 * Word 0 depiction         &            Word 2 depiction:
 *
 *   0--+                                 0--+
 *   1  |                                 1  |
 *   2  |                                 2  |
 *   3  |                                 3  |
 *   4  |                                 4  |
 *   5  |                                 5  |
 *   6  |                                 6  |
 *   7  |    IP payload checksum          7  |     VLAN tag
 *   8  |         (15:0)                  8  |      (15:0)
 *   9  |                                 9  |
 *   10 |                                 10 |
 *   11 |                                 11 |
 *   12 |                                 12 |
 *   13 |                                 13 |
 *   14 |                                 14 |
 *   15-+                                 15-+
 *   16-+                                 16-+
 *   17 |     Number of RFDs              17 |
 *   18 |        (19:16)                  18 |
 *   19-+                                 19 |     Protocol ID
 *   20-+                                 20 |      (23:16)
 *   21 |                                 21 |
 *   22 |                                 22 |
 *   23 |                                 23-+
 *   24 |                                 24 |     Reserved
 *   25 |     Start index of RFD-ring     25-+
 *   26 |         (31:20)                 26 |     RSS Q-num (27:25)
 *   27 |                                 27-+
 *   28 |                                 28-+
 *   29 |                                 29 |     RSS Hash algorithm
 *   30 |                                 30 |      (31:28)
 *   31-+                                 31-+
 *
 * Word 3 depiction:
 *
 *   0--+
 *   1  |
 *   2  |
 *   3  |
 *   4  |
 *   5  |
 *   6  |
 *   7  |    Packet length (include FCS)
 *   8  |         (13:0)
 *   9  |
 *   10 |
 *   11 |
 *   12 |
 *   13-+
 *   14      L4 Header checksum error
 *   15      IPv4 checksum error
 *   16      VLAN tagged
 *   17-+
 *   18 |    Protocol ID (19:17)
 *   19-+
 *   20      Receive error summary
 *   21      FCS(CRC) error
 *   22      Frame alignment error
 *   23      Truncated packet
 *   24      Runt packet
 *   25      Incomplete packet due to insufficient rx-desc
 *   26      Broadcast packet
 *   27      Multicast packet
 *   28      Ethernet type (EII or 802.3)
 *   29      FIFO overflow
 *   30      Length error (for 802.3, length field mismatch with actual len)
 *   31      Updated, indicate to driver that this RRD is refreshed.
 */

struct rrd_desc {
	__le32 word0;
	__le32 rss_hash;
	__le32 word2;
	__le32 word3;
} __packed;

/* rrd word 0 */
#define RRD_XSUM_MASK		0xFFFF
#define RRD_XSUM_SHIFT		0
#define RRD_NOR_MASK		0x000F
#define RRD_NOR_SHIFT		16
#define RRD_SI_MASK		0x0FFF
#define RRD_SI_SHIFT		20

/* rrd word 2 */
#define RRD_VLTAG_MASK		0xFFFF
#define RRD_VLTAG_SHIFT		0
#define RRD_PID_MASK		0x00FF
#define RRD_PID_SHIFT		16
#define RRD_PID_NONIP		0	/* non-ip packet */
#define RRD_PID_IPV4		1	/* ipv4(only) */
#define RRD_PID_IPV6TCP		2	/* tcp/ipv6 */
#define RRD_PID_IPV4TCP		3	/* tcp/ipv4 */
#define RRD_PID_IPV6UDP		4	/* udp/ipv6 */
#define RRD_PID_IPV4UDP		5	/* udp/ipv4 */
#define RRD_PID_IPV6		6	/* ipv6(only) */
#define RRD_PID_LLDP		7	/* LLDP packet */
#define RRD_PID_1588		8	/* 1588 packet */
#define RRD_RSSQ_MASK		0x0007
#define RRD_RSSQ_SHIFT		25
#define RRD_RSSALG_MASK		0x000F
#define RRD_RSSALG_SHIFT	28
#define RRD_RSSALG_TCPV6	0x1	/* TCP(IPV6) hash algorithm */
#define RRD_RSSALG_IPV6		0x2	/* IPV6 hash algorithm */
#define RRD_RSSALG_TCPV4	0x4	/* TCP(IPV4) hash algorithm */
#define RRD_RSSALG_IPV4		0x8	/* IPV4 hash algorithm */

/* rrd word 3 */
#define RRD_PKTLEN_MASK		0x3FFF
#define RRD_PKTLEN_SHIFT	0
#define RRD_ERR_L4_MASK		0x0001
#define RRD_ERR_L4_SHIFT	14
#define RRD_ERR_IPV4_MASK	0x0001
#define RRD_ERR_IPV4_SHIFT	15
#define RRD_VLTAGGED_MASK	0x0001
#define RRD_VLTAGGED_SHIFT	16
#define RRD_OLD_PID_MASK	0x0007
#define RRD_OLD_PID_SHIFT	17
#define RRD_ERR_RES_MASK	0x0001
#define RRD_ERR_RES_SHIFT	20
#define RRD_ERR_FCS_MASK	0x0001
#define RRD_ERR_FCS_SHIFT	21
#define RRD_ERR_FAE_MASK	0x0001
#define RRD_ERR_FAE_SHIFT	22
#define RRD_ERR_TRUNC_MASK	0x0001
#define RRD_ERR_TRUNC_SHIFT	23
#define RRD_ERR_RUNT_MASK	0x0001
#define RRD_ERR_RUNT_SHIFT	24
#define RRD_ERR_ICMP_MASK	0x0001
#define RRD_ERR_ICMP_SHIFT	25
#define RRD_BCAST_MASK		0x0001
#define RRD_BCAST_SHIFT		26
#define RRD_MCAST_MASK		0x0001
#define RRD_MCAST_SHIFT		27
#define RRD_ETHTYPE_MASK	0x0001
#define RRD_ETHTYPE_SHIFT	28
#define RRD_ERR_FIFOV_MASK	0x0001
#define RRD_ERR_FIFOV_SHIFT	29
#define RRD_ERR_LEN_MASK	0x0001
#define RRD_ERR_LEN_SHIFT	30
#define RRD_UPDATED_MASK	0x0001
#define RRD_UPDATED_SHIFT	31


/* Statistics counters collected by the MAC */
struct alx_hw_stats {
	/* rx */
	unsigned long rx_ok;
	unsigned long rx_bcast;
	unsigned long rx_mcast;
	unsigned long rx_pause;
	unsigned long rx_ctrl;
	unsigned long rx_fcs_err;
	unsigned long rx_len_err;
	unsigned long rx_byte_cnt;
	unsigned long rx_runt;
	unsigned long rx_frag;
	unsigned long rx_sz_64B;
	unsigned long rx_sz_127B;
	unsigned long rx_sz_255B;
	unsigned long rx_sz_511B;
	unsigned long rx_sz_1023B;
	unsigned long rx_sz_1518B;
	unsigned long rx_sz_max;
	unsigned long rx_ov_sz;
	unsigned long rx_ov_rxf;
	unsigned long rx_ov_rrd;
	unsigned long rx_align_err;
	unsigned long rx_bc_byte_cnt;
	unsigned long rx_mc_byte_cnt;
	unsigned long rx_err_addr;

	/* tx */
	unsigned long tx_ok;
	unsigned long tx_bcast;
	unsigned long tx_mcast;
	unsigned long tx_pause;
	unsigned long tx_exc_defer;
	unsigned long tx_ctrl;
	unsigned long tx_defer;
	unsigned long tx_byte_cnt;
	unsigned long tx_sz_64B;
	unsigned long tx_sz_127B;
	unsigned long tx_sz_255B;
	unsigned long tx_sz_511B;
	unsigned long tx_sz_1023B;
	unsigned long tx_sz_1518B;
	unsigned long tx_sz_max;
	unsigned long tx_single_col;
	unsigned long tx_multi_col;
	unsigned long tx_late_col;
	unsigned long tx_abort_col;
	unsigned long tx_underrun;
	unsigned long tx_trd_eop;
	unsigned long tx_len_err;
	unsigned long tx_trunc;
	unsigned long tx_bc_byte_cnt;
	unsigned long tx_mc_byte_cnt;
	unsigned long update;
};

#define SPEED_0			0
#define HALF_DUPLEX		1
#define FULL_DUPLEX		2
#define ALX_MAX_SETUP_LNK_CYCLE	50

#define ALX_SPEED_TO_ETHADV(_speed) (\
(_speed) == SPEED_1000 + FULL_DUPLEX ? ADVERTISED_1000baseT_Full :	\
(_speed) == SPEED_100 + FULL_DUPLEX ? ADVERTISED_100baseT_Full :	\
(_speed) == SPEED_100 + HALF_DUPLEX ? ADVERTISED_10baseT_Half :		\
(_speed) == SPEED_10 + FULL_DUPLEX ? ADVERTISED_10baseT_Full :		\
(_speed) == SPEED_10 + HALF_DUPLEX ? ADVERTISED_10baseT_Half :		\
0)


#define ALX_DEF_RXBUF_SIZE	1536
#define ALX_MAX_JUMBO_PKT_SIZE	(9*1024)
#define ALX_MAX_TSO_PKT_SIZE	(7*1024)
#define ALX_MAX_FRAME_SIZE	ALX_MAX_JUMBO_PKT_SIZE
#define ALX_MIN_FRAME_SIZE	68
#define ALX_RAW_MTU(_mtu)	(_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN)

#define ALX_MAX_RX_QUEUES	8	/* for RSS */
#define ALX_MAX_TX_QUEUES	4	/* multiple tx queues */
#define ALX_MAX_HANDLED_INTRS	5

#define ALX_ISR_MISC		(\
	ALX_ISR_PCIE_LNKDOWN | \
	ALX_ISR_PHY | \
	ALX_ISR_DMAW | \
	ALX_ISR_DMAR | \
	ALX_ISR_SMB | \
	ALX_ISR_MANU | \
	ALX_ISR_TIMER | \
	ALX_ISR_RXF_OV | \
	ALX_ISR_TXF_UR | \
	ALX_ISR_RFD_UR)

#define ALX_ISR_FATAL	(\
	ALX_ISR_PCIE_LNKDOWN | \
	 ALX_ISR_DMAW | \
	 ALX_ISR_DMAR)

#define ALX_ISR_ALERT	(\
	ALX_ISR_RXF_OV | \
	ALX_ISR_TXF_UR | \
	ALX_ISR_RFD_UR)

#define ALX_ISR_ALL_QUEUES (\
	ALX_ISR_TX_Q0 | \
	ALX_ISR_TX_Q1 | \
	ALX_ISR_TX_Q2 | \
	ALX_ISR_TX_Q3 | \
	ALX_ISR_RX_Q0 | \
	ALX_ISR_RX_Q1 | \
	ALX_ISR_RX_Q2 | \
	ALX_ISR_RX_Q3 | \
	ALX_ISR_RX_Q4 | \
	ALX_ISR_RX_Q5 | \
	ALX_ISR_RX_Q6 | \
	ALX_ISR_RX_Q7)

#define ALX_MAX_MSIX_INTRS	16	/* maximum interrupt vectors for msix */
#define ALX_WATCHDOG_TIME   (5 * HZ)

/*
 * alx_ring_header is a single, contiguous block of memory space
 * used by the three descriptor rings (tpd, rfd, rrd)
 */
struct alx_ring_header {
	void        *desc;      /* virt addr */
	dma_addr_t   dma;       /* phy addr */
	A_UINT32          size;      /* length in bytes */
};

/*
 * alx_buffer wraps around a pointer to a socket buffer
 * so a DMA physical address can be stored along with the skb
 */
struct alx_buffer {
	struct sk_buff *skb;		/* socket buffer */
	DEFINE_DMA_UNMAP_ADDR(dma);	/* DMA address */
	DEFINE_DMA_UNMAP_LEN(size);	/* buffer size */
	A_UINT16		flags;		/* information of this buffer */
};
#define ALX_BUF_TX_FIRSTFRAG	0x1

/* rx queue */
struct alx_rx_queue {
	struct net_device *netdev;
	struct device *dev;		/* device pointer for dma operation */
	struct rrd_desc *rrd_hdr;	/* rrd ring virtual addr */
	dma_addr_t rrd_dma;		/* rrd ring physical addr */
	struct rfd_desc *rfd_hdr;	/* rfd ring virtual addr */
	dma_addr_t rfd_dma;		/* rfd ring physical addr */
	struct alx_buffer *bf_info;	/* info for rx-skbs */

	A_UINT16 count;			/* number of ring elements */
	A_UINT16 pidx;			/* rfd producer index */
	A_UINT16 cidx;			/* rfd consumer index */
	A_UINT16 rrd_cidx;
	A_UINT16 p_reg;			/* register saving producer index */
	A_UINT16 c_reg;			/* register saving consumer index */
	A_UINT16 qidx;			/* queue index */
	unsigned long flag;

	struct sk_buff_head list;
};
#define ALX_RQ_USING		1
#define ALX_RX_ALLOC_THRESH	32
/* tx queue */
struct alx_tx_queue {
	struct net_device *netdev;
	struct device *dev;		/* device pointer for dma operation */
	struct tpd_desc *tpd_hdr;	/* tpd ring virtual addr */
	dma_addr_t tpd_dma;		/* tpd ring physical addr */
	struct alx_buffer *bf_info;	/* info for tx-skbs pending on HW */
	A_UINT16 count;			/* number of ring elements  */
	A_UINT16 pidx;			/* producer index */
	atomic_t cidx;			/* consumer index */
	A_UINT16 p_reg;			/* register saving producer index */
	A_UINT16 c_reg;			/* register saving consumer index */
	A_UINT16 qidx;			/* queue index */
};

#define ALX_TX_WAKEUP_THRESH(_tq) ((_tq)->count / 4)
#define ALX_DEFAULT_TX_WORK		128

struct alx_napi {
	struct napi_struct	napi;
	struct alx_adapter	*adpt;
	struct alx_rx_queue	*rxq;
	struct alx_tx_queue	*txq;
	int			vec_idx;
	A_UINT32			vec_mask;
	char			irq_lbl[IFNAMSIZ];
};

enum ALX_FLAGS {
	ALX_FLAG_CAP_GIGA = 0,		/* support gigabit speed */
	ALX_FLAG_CAP_PTP,		/* support 1588 */
	ALX_FLAG_CAP_AZ,		/* support EEE */
	ALX_FLAG_CAP_L0S,		/* support ASPM L0S */
	ALX_FLAG_CAP_L1,		/* support ASPM L1 */
	ALX_FLAG_CAP_SWOI,		/* support SWOI feature */
	ALX_FLAG_CAP_RSS,		/* support RSS */
	ALX_FLAG_CAP_MSIX,		/* support MSI-X */
	ALX_FLAG_CAP_MTQ,		/* support Multi-TX-Q */
	ALX_FLAG_CAP_MRQ,		/* support Multi-RX-Q */
	ALX_FLAG_USING_MSIX,		/* using MSI-X */
	ALX_FLAG_USING_MSI,		/* using MSI */
	ALX_FLAG_RESETING,		/* hw is in reset process */
	ALX_FLAG_TESTING,		/* self testing */
	ALX_FLAG_HALT,			/* hw is down */
	ALX_FLAG_FPGA,			/* FPGA, not ASIC */
	ALX_FLAG_TASK_PENDING,		/* work is pending */
	ALX_FLAG_TASK_CHK_LINK,		/* check PHY link */
	ALX_FLAG_TASK_RESET,		/* reset whole chip */
	ALX_FLAG_TASK_UPDATE_SMB,	/* update SMB */

	ALX_FLAG_NUMBER_OF_FLAGS,
};

/*
 *board specific private data structure
 */
struct alx_adapter {
	A_UINT8 __iomem *hw_addr;		/* memory mapped PCI base address */

	A_UINT8 mac_addr[ETH_ALEN];		/* current mac address */
	A_UINT8 perm_addr[ETH_ALEN];		/* permanent mac address */

	struct net_device	*netdev;
	struct pci_dev		*pdev;

	A_UINT16 bd_number;			/* board number;*/

	unsigned int		nr_vec;		/* totally msix vectors */
	struct msix_entry	*msix_ent;	/* msix entries */

	/* all descriptor memory */
	struct alx_ring_header	ring_header;
	int			tx_ringsz;
	int			rx_ringsz;
	int			rxbuf_size;

	struct alx_napi		*qnapi[8];
	int			nr_txq;		/* number of napi for TX-Q */
	int			nr_rxq;		/* number of napi for RX-Q */
	int			nr_napi;	/* total napi for TX-Q/RX-Q  */
	A_UINT16			mtu;		/* MTU */
	A_UINT16			imt;		/* interrupt moderation timer */
	A_UINT8			dma_chnl;	/* number of DMA channels */
	A_UINT8			max_dma_chnl;
	A_UINT32			rx_ctrl;	/* main rx control */
	A_UINT32			mc_hash[2];	/* multicast addr hash table */

	A_UINT8			rss_key[40];	/* RSS hash algorithm key */
	A_UINT32			rss_idt[32];	/* RSS indirection table */
	A_UINT16			rss_idt_size;	/* RSS indirection table size */
	A_UINT8			rss_hash_type;	/* RSS hash type */

	A_UINT32			wrr[ALX_MAX_TX_QUEUES];	/* weight round robin
							 * for multiple-tx-Q */
	A_UINT32			wrr_ctrl;	/* prioirty control */

	A_UINT32			imask;		/* interrupt mask for ALX_IMR */
	A_UINT32			smb_timer;	/* statistic counts refresh
						 * timeout, million-seconds */
	spinlock_t		smb_lock;	/* lock for updating stats */

	HAL_BOOL			link_up;	/* link up flag */
	A_UINT16			link_speed;	/* current link speed */
	A_UINT8			link_duplex;	/* current link duplex */

	A_UINT32			adv_cfg;	/* auto-neg advertisement
						 * or force mode config
						 */
	A_UINT8			flowctrl;	/* flow control */

	struct work_struct	task;		/* any delayed work */
	struct net_device_stats net_stats;	/* statistics counters */
	struct alx_hw_stats	hw_stats;	/* statistics counters,
						 * same order with hw
						 */
	A_UINT32			sleep_ctrl;	/* control used when sleep */
	atomic_t		irq_sem;	/* interrupt sync */
	A_UINT16			msg_enable;	/* msg level */

	DECLARE_BITMAP(flags, ALX_FLAG_NUMBER_OF_FLAGS);

	spinlock_t		mdio_lock;	/* used for MII bus access */
	struct mdio_if_info	mdio;
	A_UINT16			phy_id[2];

	HAL_BOOL			lnk_patch;	/* PHY link patch flag */
	HAL_BOOL			hib_patch;	/* PHY hibernation patch flag */
};

#define ALX_VID(_a)	((_a)->pdev->vendor)
#define ALX_DID(_a)	((_a)->pdev->device)
#define ALX_SUB_VID(_a)	((_a)->pdev->subsystem_vendor)
#define ALX_SUB_DID(_a)	((_a)->pdev->subsystem_device)
#define ALX_REVID(_a)	((_a)->pdev->revision >> ALX_PCI_REVID_SHIFT)
#define ALX_WITH_CR(_a)	((_a)->pdev->revision & 1)

#define ALX_FLAG(_adpt, _FLAG) (\
	test_bit(ALX_FLAG_##_FLAG, (_adpt)->flags))
#define ALX_FLAG_SET(_adpt, _FLAG) (\
	set_bit(ALX_FLAG_##_FLAG, (_adpt)->flags))
#define ALX_FLAG_CLEAR(_adpt, _FLAG) (\
	clear_bit(ALX_FLAG_##_FLAG, (_adpt)->flags))

/* flush regs */
#define ALX_MEM_FLUSH(s) (readl((s)->hw_addr))

/* needed by alx_ethtool.c */
extern void alx_reinit(struct alx_adapter *adpt);
extern void __devinit alx_set_ethtool_ops(struct net_device *dev);
extern char alx_drv_name[];


#endif /* _ALX_H_ */
