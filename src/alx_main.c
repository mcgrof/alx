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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_vlan.h>
#include <linux/mii.h>
#include <linux/aer.h>

#include "alx.h"

char alx_drv_name[] = "alx";
static const char alx_drv_desc[] =
"Qualcomm Atheros(R) AR816x/AR817x PCI-E Ethernet Network Driver";

/* alx_pci_tbl - PCI Device ID Table
 *
 * Wildcard entries (PCI_ANY_ID) should come last
 * Last entry must be all 0s
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, private data (not used) }
 */
#define ALX_ETHER_DEVICE(device_id) {\
	PCI_DEVICE(ALX_VENDOR_ID, device_id)}
static DEFINE_PCI_DEVICE_TABLE(alx_pci_tbl) = {
	ALX_ETHER_DEVICE(ALX_DEV_ID_AR8161),
	ALX_ETHER_DEVICE(ALX_DEV_ID_AR8162),
	ALX_ETHER_DEVICE(ALX_DEV_ID_AR8171),
	ALX_ETHER_DEVICE(ALX_DEV_ID_AR8172),
	{0,}
};

MODULE_DEVICE_TABLE(pci, alx_pci_tbl);
MODULE_AUTHOR("Qualcomm Corporation, <nic-devel@qualcomm.com>");
MODULE_DESCRIPTION("Qualcomm Atheros Gigabit Ethernet Driver");
MODULE_LICENSE("Dual BSD/GPL");


static int alx_poll(struct napi_struct *napi, int budget);
static irqreturn_t alx_msix_ring(int irq, void *data);
static irqreturn_t alx_intr_msix_misc(int irq, void *data);
static irqreturn_t alx_intr_msi(int irq, void *data);
static irqreturn_t alx_intr_legacy(int irq, void *data);
static void alx_init_ring_ptrs(struct alx_adapter *adpt);

static inline void alx_schedule_work(struct alx_adapter *adpt)
{
	if (!ALX_FLAG(adpt, HALT) &&
	    !ALX_FLAG(adpt, TASK_PENDING)) {
		ALX_FLAG_SET(adpt, TASK_PENDING);
		schedule_work(&adpt->task);
	}
}

static inline void alx_cancel_work(struct alx_adapter *adpt)
{
	ALX_FLAG_CLEAR(adpt, TASK_PENDING);
	cancel_work_sync(&adpt->task);
}

void alx_add_mc_addr(struct alx_adapter *adpt, u8 *addr)
{
	u32 crc32, bit, reg;

	crc32 = ether_crc(ETH_ALEN, addr);

	/* The HASH Table  is a register array of 2 32-bit registers.
	 * It is treated like an array of 64 bits.  We want to set
	 * bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The register is determined by the
	 * upper 7 bits of the hash value and the bit within that
	 * register are determined by the lower 5 bits of the value.
	 */
	reg = (crc32 >> 31) & 0x1;
	bit = (crc32 >> 26) & 0x1F;

	adpt->mc_hash[reg] |= (0x1 << bit);
}

static void __alx_set_rx_mode(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct netdev_hw_addr *ha;

	/* comoute mc addresses' hash value ,and put it into hash table */
	netdev_for_each_mc_addr(ha, netdev)
		alx_add_mc_addr(adpt, ha->addr);

	ALX_MEM_W32(adpt, ALX_HASH_TBL0, adpt->mc_hash[0]);
	ALX_MEM_W32(adpt, ALX_HASH_TBL1, adpt->mc_hash[1]);

	/* check for Promiscuous and All Multicast modes */
	adpt->rx_ctrl &= ~(ALX_MAC_CTRL_MULTIALL_EN | ALX_MAC_CTRL_PROMISC_EN);
	if (netdev->flags & IFF_PROMISC)
		adpt->rx_ctrl |= ALX_MAC_CTRL_PROMISC_EN;
	if (netdev->flags & IFF_ALLMULTI)
		adpt->rx_ctrl |= ALX_MAC_CTRL_MULTIALL_EN;

	ALX_MEM_W32(adpt, ALX_MAC_CTRL, adpt->rx_ctrl);
}

/* alx_set_rx_mode - Multicast and Promiscuous mode set */
static void alx_set_rx_mode(struct net_device *netdev)
{
	__alx_set_rx_mode(netdev);
}


/* alx_set_mac - Change the Ethernet Address of the NIC */
static int alx_set_mac_address(struct net_device *netdev, void *data)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct sockaddr *addr = data;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (netdev->addr_assign_type & NET_ADDR_RANDOM)
		netdev->addr_assign_type ^= NET_ADDR_RANDOM;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(adpt->mac_addr, addr->sa_data, netdev->addr_len);
	alx_set_macaddr(adpt, adpt->mac_addr);

	return 0;
}

static void alx_free_napis(struct alx_adapter *adpt)
{
	struct alx_napi *np;
	int i;

	for (i = 0; i < adpt->nr_napi; i++) {
		np = adpt->qnapi[i];
		if (!np)
			continue;

		netif_napi_del(&np->napi);
		kfree(np->txq);
		np->txq = NULL;
		kfree(np->rxq);
		np->rxq = NULL;
		adpt->qnapi[i] = NULL;
	}
}
static u16 tx_pidx_reg[] = {ALX_TPD_PRI0_PIDX, ALX_TPD_PRI1_PIDX,
			    ALX_TPD_PRI2_PIDX, ALX_TPD_PRI3_PIDX};
static u16 tx_cidx_reg[] = {ALX_TPD_PRI0_CIDX, ALX_TPD_PRI1_CIDX,
			    ALX_TPD_PRI2_CIDX, ALX_TPD_PRI3_CIDX};
static u32 tx_vect_mask[] = {ALX_ISR_TX_Q0, ALX_ISR_TX_Q1,
			     ALX_ISR_TX_Q2, ALX_ISR_TX_Q3};
static u32 rx_vect_mask[] = {ALX_ISR_RX_Q0, ALX_ISR_RX_Q1,
			     ALX_ISR_RX_Q2, ALX_ISR_RX_Q3,
			     ALX_ISR_RX_Q4, ALX_ISR_RX_Q5,
			     ALX_ISR_RX_Q6, ALX_ISR_RX_Q7};
static int alx_alloc_napis(struct alx_adapter *adpt)
{
	struct alx_napi		*np;
	struct alx_rx_queue	*rxq;
	struct alx_tx_queue	*txq;
	int i;

	adpt->imask &= ~ALX_ISR_ALL_QUEUES;

	/* alloc alx_napi */
	for (i = 0; i < adpt->nr_napi; i++) {
		np = kzalloc(sizeof(struct alx_napi), GFP_KERNEL);
		if (!np)
			goto err_out;

		np->adpt = adpt;
		netif_napi_add(adpt->netdev, &np->napi, alx_poll, 64);
		adpt->qnapi[i] = np;
	}

	/* alloc tx queue */
	for (i = 0; i < adpt->nr_txq; i++) {
		np = adpt->qnapi[i];
		txq = kzalloc(sizeof(struct alx_tx_queue), GFP_KERNEL);
		if (!txq)
			goto err_out;
		np->txq = txq;
		txq->p_reg = tx_pidx_reg[i];
		txq->c_reg = tx_cidx_reg[i];
		txq->count = adpt->tx_ringsz;
		txq->qidx = (u16)i;
		np->vec_mask |= tx_vect_mask[i];
		adpt->imask |= tx_vect_mask[i];
	}

	/* alloc rx queue */
	for (i = 0; i < adpt->nr_rxq; i++) {
		np = adpt->qnapi[i];
		rxq = kzalloc(sizeof(struct alx_rx_queue), GFP_KERNEL);
		if (!rxq)
			goto err_out;
		np->rxq = rxq;
		rxq->p_reg = ALX_RFD_PIDX;
		rxq->c_reg = ALX_RFD_CIDX;
		rxq->count = adpt->rx_ringsz;
		rxq->qidx = (u16)i;
		np->vec_mask |= rx_vect_mask[i];
		adpt->imask |= rx_vect_mask[i];
	}

	return 0;

err_out:
	alx_free_napis(adpt);
	return -ENOMEM;
}

static int alx_alloc_rings(struct alx_adapter *adpt)
{
	struct alx_buffer *bf;
	u8 *desc;
	dma_addr_t  dma;
	int i, size, hw_rxrng, offset;

	/* alx_buffer */
	size = sizeof(struct alx_buffer) * adpt->nr_txq * adpt->tx_ringsz +
	       sizeof(struct alx_buffer) * adpt->nr_rxq * adpt->rx_ringsz;

	bf = vzalloc(size);
	if (!bf)
		goto err_out;

	/* physical rx rings */
	hw_rxrng  = ALX_FLAG(adpt, CAP_MRQ) ? adpt->nr_rxq : 1;
	size = sizeof(struct tpd_desc) * adpt->tx_ringsz * adpt->nr_txq +
	       (sizeof(struct rrd_desc) + sizeof(struct rfd_desc)) *
	       adpt->rx_ringsz * hw_rxrng +
	       adpt->nr_txq * 8 +	/* tx align */
	       hw_rxrng * 8;		/* rx align */
	desc = dma_alloc_coherent(&adpt->pdev->dev, size, &dma, GFP_KERNEL);
	if (!desc)
		goto err_out;

	memset(desc, 0, size);
	adpt->ring_header.desc = desc;
	adpt->ring_header.dma = dma;
	adpt->ring_header.size = size;

	size = sizeof(struct tpd_desc) * adpt->tx_ringsz;
	for (i = 0; i < adpt->nr_txq; i++) {
		offset = ALIGN(dma, 8) - dma;
		desc += offset;
		dma += offset;
		adpt->qnapi[i]->txq->netdev = adpt->netdev;
		adpt->qnapi[i]->txq->dev = &adpt->pdev->dev;
		adpt->qnapi[i]->txq->tpd_hdr = (struct tpd_desc *)desc;
		adpt->qnapi[i]->txq->tpd_dma = dma;
		adpt->qnapi[i]->txq->count = adpt->tx_ringsz;
		adpt->qnapi[i]->txq->bf_info = bf;
		desc += size;
		dma += size;
		bf += adpt->tx_ringsz;
	}
	size = sizeof(struct rrd_desc) * adpt->rx_ringsz;
	for (i = 0; i < hw_rxrng; i++) {
		offset = ALIGN(dma, 8) - dma;
		desc += offset;
		dma += offset;
		adpt->qnapi[i]->rxq->netdev = adpt->netdev;
		adpt->qnapi[i]->rxq->dev = &adpt->pdev->dev;
		adpt->qnapi[i]->rxq->rrd_hdr = (struct rrd_desc *)desc;
		adpt->qnapi[i]->rxq->rrd_dma = dma;
		adpt->qnapi[i]->rxq->count = adpt->rx_ringsz;
		desc += size;
		dma += size;
	}
	size = sizeof(struct rfd_desc) * adpt->rx_ringsz;
	for (i = 0; i < hw_rxrng; i++) {
		offset = ALIGN(dma, 8) - dma;
		desc += offset;
		dma += offset;
		adpt->qnapi[i]->rxq->rfd_hdr = (struct rfd_desc *)desc;
		adpt->qnapi[i]->rxq->rfd_dma = dma;
		desc += size;
		dma += size;
	}
	for (i = 0; i < adpt->nr_rxq; i++) {
		adpt->qnapi[i]->rxq->bf_info = bf;
		bf += adpt->rx_ringsz;
	}

	return 0;

err_out:
	if (bf)
		vfree(bf);

	return -ENOMEM;
}

static void alx_free_rings(struct alx_adapter *adpt)
{
	struct alx_buffer *bf;
	struct alx_napi *np;

	/* alx_buffer header is in the 1st tpdq->bf_info */
	np = adpt->qnapi[0];
	if (np) {
		bf = np->txq->bf_info;
		if (bf) {
			vfree(bf);
			np->txq->bf_info = NULL;
		}
	}
	if (adpt->ring_header.desc) {
		dma_free_coherent(&adpt->pdev->dev,
				  adpt->ring_header.size,
				  adpt->ring_header.desc,
				  adpt->ring_header.dma);
		adpt->ring_header.desc = NULL;
	}
}

static int alx_alloc_rxring_buf(struct alx_adapter *adpt,
				struct alx_rx_queue *rxq)
{
	struct sk_buff *skb;
	struct alx_buffer *cur_buf;
	struct rfd_desc *rfd;
	dma_addr_t dma;
	u16 cur, next, count = 0;

	next = cur = rxq->pidx;
	if (++next == rxq->count)
		next = 0;
	cur_buf = rxq->bf_info + cur;
	rfd = rxq->rfd_hdr + cur;

	while (!cur_buf->skb && next != rxq->cidx) {
		skb = dev_alloc_skb(adpt->rxbuf_size);
		if (unlikely(!skb)) {
			netdev_warn(adpt->netdev, "alloc skb failed\n");
			break;
		}
		skb_reserve(skb, NET_IP_ALIGN);
		dma = dma_map_single(rxq->dev,
				     skb->data,
				     adpt->rxbuf_size,
				     DMA_FROM_DEVICE);
		if (dma_mapping_error(rxq->dev, dma)) {
			netdev_warn(adpt->netdev, "mapping rx-buffer failed\n");
			dev_kfree_skb(skb);
			break;
		}
		cur_buf->skb = skb;
		dma_unmap_len_set(cur_buf, size, adpt->rxbuf_size);
		dma_unmap_addr_set(cur_buf, dma, dma);
		rfd->addr = cpu_to_le64(dma);

		cur = next;
		if (++next == rxq->count)
			next = 0;
		cur_buf = rxq->bf_info + cur;
		rfd = rxq->rfd_hdr + cur;
		count++;
	}

	if (count) {
		wmb();
		rxq->pidx = cur;
		ALX_MEM_W16(adpt, rxq->p_reg, (u16)cur);
	}

	return count;
}

static void alx_free_rxring_buf(struct alx_rx_queue *rxq)
{
	struct alx_buffer *cur_buf;
	u16 i;

	if (rxq == NULL)
		return;

	for (i = 0; i < rxq->count; i++) {
		cur_buf = rxq->bf_info + i;
		if (cur_buf->skb) {
			dma_unmap_single(rxq->dev,
					 dma_unmap_addr(cur_buf, dma),
					 dma_unmap_len(cur_buf, size),
					 DMA_FROM_DEVICE);
			dev_kfree_skb(cur_buf->skb);
			cur_buf->skb = NULL;
			dma_unmap_len_set(cur_buf, size, 0);
			dma_unmap_addr_set(cur_buf, dma, 0);
		}
	}
	rxq->pidx = 0;
	rxq->cidx = 0;
	rxq->rrd_cidx = 0;
}

static int alx_setup_all_ring_resources(struct alx_adapter *adpt)
{
	int err, i;

	err = alx_alloc_napis(adpt);
	if (err)
		goto out;
	err = alx_alloc_rings(adpt);
	if (err)
		goto out;

	/* set ring headers to HW register */
	alx_init_ring_ptrs(adpt);

	for (i = 0; i < adpt->nr_rxq; i++) {
		err = alx_alloc_rxring_buf(adpt, adpt->qnapi[i]->rxq);
		if (!err) {
			err = -ENOMEM;
			goto out;
		}
	}
out:
	if (unlikely(err)) {
		dev_err(&adpt->pdev->dev,
			"setup_all_ring_resources fail %d\n",
			err);
	}
	return err;
}

static void alx_txbuf_unmap_and_free(struct alx_tx_queue *txq, int entry)
{
	struct alx_buffer *txb = txq->bf_info + entry;

	if (dma_unmap_len(txb, size) &&
	    txb->flags & ALX_BUF_TX_FIRSTFRAG) {
		dma_unmap_single(txq->dev,
				 dma_unmap_addr(txb, dma),
				 dma_unmap_len(txb, size),
				 DMA_TO_DEVICE);
		txb->flags &= ~ALX_BUF_TX_FIRSTFRAG;
	} else if (dma_unmap_len(txb, size)) {
		dma_unmap_page(txq->dev,
			       dma_unmap_addr(txb, dma),
			       dma_unmap_len(txb, size),
			       DMA_TO_DEVICE);
	}
	if (txb->skb) {
		dev_kfree_skb_any(txb->skb);
		txb->skb = NULL;
	}
	dma_unmap_len_set(txb, size, 0);
}

static void alx_free_txring_buf(struct alx_tx_queue *txq)
{
	int i;

	if (!txq->bf_info)
		return;

	for (i = 0; i < txq->count; i++)
		alx_txbuf_unmap_and_free(txq, i);

	memset(txq->bf_info, 0, txq->count * sizeof(struct alx_buffer));
	memset(txq->tpd_hdr, 0, txq->count * sizeof(struct tpd_desc));
	txq->pidx = 0;
	atomic_set(&txq->cidx, 0);

	netdev_tx_reset_queue(netdev_get_tx_queue(txq->netdev, txq->qidx));
}

/* free up pending skb for tx/rx */
static void alx_free_all_rings_buf(struct alx_adapter *adpt)
{
	int i;

	for (i = 0; i < adpt->nr_txq; i++)
		alx_free_txring_buf(adpt->qnapi[i]->txq);

	for (i = 0; i < adpt->nr_rxq; i++)
		alx_free_rxring_buf(adpt->qnapi[i]->rxq);
}

static void alx_free_all_ring_resources(struct alx_adapter *adpt)
{
	alx_free_all_rings_buf(adpt);
	alx_free_rings(adpt);
	alx_free_napis(adpt);
}

static inline int alx_tpd_avail(struct alx_tx_queue *txq)
{
	u16 cidx = atomic_read(&txq->cidx);

	return txq->pidx >= cidx ?
		txq->count + cidx - txq->pidx - 1 :
		cidx - txq->pidx - 1;
}



static bool alx_clean_tx_irq(struct alx_tx_queue *txq)
{
	struct alx_adapter *adpt = netdev_priv(txq->netdev);
	struct netdev_queue *netque;
	u16 hw_cidx, sw_cidx;
	unsigned int total_bytes = 0, total_packets = 0;
	int budget = ALX_DEFAULT_TX_WORK;

	if (ALX_FLAG(adpt, HALT))
		return true;

	netque = netdev_get_tx_queue(adpt->netdev, txq->qidx);
	sw_cidx = atomic_read(&txq->cidx);

	ALX_MEM_R16(adpt, txq->c_reg, &hw_cidx);

	netdev_info(adpt->netdev,
		    "TX[Preg:%x]: consumer = 0x%x, hw-consumer = 0x%x\n",
		    txq->p_reg, sw_cidx, hw_cidx);

	while (sw_cidx != hw_cidx && budget > 0) {
		struct sk_buff *skb;

		skb = txq->bf_info[sw_cidx].skb;
		if (skb) {
			total_bytes += skb->len;
			total_packets++;
			budget--;
		}
		alx_txbuf_unmap_and_free(txq, sw_cidx);

		if (++sw_cidx == txq->count)
			sw_cidx = 0;
	}
	atomic_set(&txq->cidx, sw_cidx);

	netdev_tx_completed_queue(netque, total_packets, total_bytes);

	if (unlikely(netif_tx_queue_stopped(netque) &&
		     netif_carrier_ok(adpt->netdev) &&
		     alx_tpd_avail(txq) > ALX_TX_WAKEUP_THRESH(txq) &&
		     !ALX_FLAG(adpt, HALT))) {
		netif_tx_wake_queue(netque);
	}

	return sw_cidx == hw_cidx;
}

static inline bool alx_skb_dequeue_n(struct alx_rx_queue *rxq, int max_pkts,
				     struct sk_buff_head *list)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);
	bool use_lock = !ALX_FLAG(adpt, CAP_MRQ);
	bool empty;
	struct sk_buff *skb;
	int count = 0;

	if (use_lock)
		spin_lock(&rxq->list.lock);

	while (count < max_pkts) {
		skb = __skb_dequeue(&rxq->list);
		if (skb) {
			__skb_queue_tail(list, skb);
			count++;
		} else
			break;
	}

	empty = count == 0;

	if (use_lock)
		spin_unlock(&rxq->list.lock);

	return empty;
}

static inline void alx_skb_queue_tail(struct alx_rx_queue *rxq,
				      struct sk_buff *skb)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);
	bool use_lock = !ALX_FLAG(adpt, CAP_MRQ);

	if (use_lock)
		spin_lock(&rxq->list.lock);

	__skb_queue_tail(&rxq->list, skb);

	if (use_lock)
		spin_unlock(&rxq->list.lock);
}

static bool alx_dispatch_skb(struct alx_rx_queue *rxq)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);
	struct rrd_desc *rrd;
	struct alx_buffer *rxb;
	struct sk_buff *skb;
	u16 length, rfd_cleaned = 0;
	struct alx_rx_queue *tmp_rxq;
	int qnum;

	if (test_and_set_bit(ALX_RQ_USING, &rxq->flag))
		return false;

	while (1) {
		rrd = rxq->rrd_hdr + rxq->rrd_cidx;
		if (!(rrd->word3 & (1 << RRD_UPDATED_SHIFT)))
			break;
		rrd->word3 &= ~(1 << RRD_UPDATED_SHIFT);

		if (unlikely(FIELD_GETX(rrd->word0, RRD_SI) != rxq->cidx ||
			     FIELD_GETX(rrd->word0, RRD_NOR) != 1)) {
			netdev_err(adpt->netdev,
				   "wrong rrd-SI/NOR packet! rrd->word0 is %08x\n",
				   rrd->word0);
			/* reset chip */
			ALX_FLAG_SET(adpt, TASK_RESET);
			alx_schedule_work(adpt);
			return true;
		}
		rxb = rxq->bf_info + rxq->cidx;
		dma_unmap_single(rxq->dev,
				 dma_unmap_addr(rxb, dma),
				 dma_unmap_len(rxb, size),
				 DMA_FROM_DEVICE);
		dma_unmap_len_set(rxb, size, 0);
		skb = rxb->skb;
		rxb->skb = NULL;

		if (unlikely(rrd->word3 & (1 << RRD_ERR_RES_SHIFT) ||
			     rrd->word3 & (1 << RRD_ERR_LEN_SHIFT))) {
			netdev_warn(adpt->netdev,
				   "wrong packet! rrd->word3 is %08x\n",
				   rrd->word3);
			rrd->word3 = 0;
			dev_kfree_skb_any(skb);
			goto next_pkt;
		}
		length = FIELD_GETX(rrd->word3, RRD_PKTLEN) - ETH_FCS_LEN;
		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, adpt->netdev);
		/* checksum */
		skb_checksum_none_assert(skb);
		if (adpt->netdev->features & NETIF_F_RXCSUM) {
			switch (FIELD_GETX(rrd->word2, RRD_PID)) {
			case RRD_PID_IPV6UDP:
			case RRD_PID_IPV4UDP:
			case RRD_PID_IPV4TCP:
			case RRD_PID_IPV6TCP:
				if (rrd->word3 & ((1 << RRD_ERR_L4_SHIFT) |
						  (1 << RRD_ERR_IPV4_SHIFT))) {
					netdev_warn(
						adpt->netdev,
						"rx-chksum error, w2=%X\n",
						rrd->word2);
					break;
				}
				skb->ip_summed = CHECKSUM_UNNECESSARY;
				break;
			}
		}
		/* vlan tag */
		if (rrd->word3 & (1 << RRD_VLTAGGED_SHIFT)) {
			u16 tag = ntohs(FIELD_GETX(rrd->word2, RRD_VLTAG));
			__vlan_hwaccel_put_tag(skb, ntohs(tag));
		}
		qnum = FIELD_GETX(rrd->word2, RRD_RSSQ) % adpt->nr_rxq;
		tmp_rxq = ALX_FLAG(adpt, CAP_MRQ) ?
				rxq : adpt->qnapi[qnum]->rxq;
		alx_skb_queue_tail(tmp_rxq, skb);

next_pkt:

		if (++rxq->cidx == rxq->count)
			rxq->cidx = 0;
		if (++rxq->rrd_cidx == rxq->count)
			rxq->rrd_cidx = 0;

		if (++rfd_cleaned > ALX_RX_ALLOC_THRESH)
			rfd_cleaned -= alx_alloc_rxring_buf(adpt, rxq);
	}

	if (rfd_cleaned)
		alx_alloc_rxring_buf(adpt, rxq);

	clear_bit(ALX_RQ_USING, &rxq->flag);

	return true;
}

static inline struct alx_rx_queue *alx_hw_rxq(struct alx_rx_queue *rxq)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);

	return ALX_FLAG(adpt, CAP_MRQ) ? rxq : adpt->qnapi[0]->rxq;
}

static void __alx_configure_rss(struct alx_adapter *adpt)
{
	u32 ctrl;
	int i;

	ALX_MEM_R32(adpt, ALX_RXQ0, &ctrl);

	if (adpt->nr_rxq > 1) { /* RSS enable */

		for (i = 0; i < sizeof(adpt->rss_key); i++) {
			/* rss key should be saved in chip with
			 * reversed order.
			 */
			int j = sizeof(adpt->rss_key) - i - 1;

			ALX_MEM_W8(adpt, ALX_RSS_KEY0 + j, adpt->rss_key[i]);
		}

		for (i = 0; i < ARRAY_SIZE(adpt->rss_idt); i++)
			ALX_MEM_W32(adpt, ALX_RSS_IDT_TBL0, adpt->rss_idt[i]);

		FIELD_SET32(ctrl, ALX_RXQ0_RSS_HSTYP, adpt->rss_hash_type);
		FIELD_SET32(ctrl, ALX_RXQ0_RSS_MODE, ALX_RXQ0_RSS_MODE_MQMI);
		FIELD_SET32(ctrl, ALX_RXQ0_IDT_TBL_SIZE, adpt->rss_idt_size);
		ctrl |= ALX_RXQ0_RSS_HASH_EN;
	} else { /* RSS disable */
		ctrl &= ~ALX_RXQ0_RSS_HASH_EN;
	}

	ALX_MEM_W32(adpt, ALX_RXQ0, ctrl);
}

static inline struct napi_struct *alx_rxq_to_napi(
	struct alx_rx_queue *rxq)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);

	return &adpt->qnapi[rxq->qidx]->napi;
}

static bool alx_clean_rx_irq(struct alx_rx_queue *rxq, int budget)
{
	struct sk_buff_head list;
	int empty;
	bool collect;

	__skb_queue_head_init(&list);
	collect = alx_dispatch_skb(alx_hw_rxq(rxq));
	empty = alx_skb_dequeue_n(rxq, budget, &list);
	if (!skb_queue_empty(&list)) {
		struct napi_struct *napi;
		struct sk_buff *skb;

		napi = alx_rxq_to_napi(rxq);
		while (!skb_queue_empty(&list)) {
			skb = __skb_dequeue(&list);
			napi_gro_receive(napi, skb);
		}
	}

	return collect && empty;
}

static int alx_request_msix(struct alx_adapter *adpt)
{
	struct net_device *netdev = adpt->netdev;
	int i, err;
	int vec;

	err = request_irq(adpt->msix_ent[0].vector,
		alx_intr_msix_misc, 0, netdev->name, adpt);
	if (err)
		goto out;

	vec = 1;
	for (i = 0; i < adpt->nr_napi; i++) {
		struct alx_napi *np = adpt->qnapi[i];

		if (np->txq && np->rxq)
			sprintf(np->irq_lbl, "%s-TR-%u", netdev->name, i);
		else if (np->txq)
			sprintf(np->irq_lbl, "%s-T-%u", netdev->name, i);
		else
			sprintf(np->irq_lbl, "%s-R-%u", netdev->name, i);

		np->vec_idx = vec;
		err = request_irq(adpt->msix_ent[vec].vector,
			alx_msix_ring, 0, np->irq_lbl, np);
		if (err) {
			for (i--, vec--; i >= 0; i--) {
				np = adpt->qnapi[i];
				free_irq(adpt->msix_ent[vec].vector, np);
			}
			free_irq(adpt->msix_ent[0].vector, adpt);
			goto out;
		}
		vec++;
	}

out:
	return err;
}

static void alx_disable_msix(struct alx_adapter *adpt)
{
	if (adpt->msix_ent) {
		pci_disable_msix(adpt->pdev);
		kfree(adpt->msix_ent);
		adpt->msix_ent = NULL;
	}
	ALX_FLAG_CLEAR(adpt, USING_MSIX);
}

static void alx_mask_msix(struct alx_adapter *adpt, int index, bool mask)
{
	u32 reg, val;

	reg = ALX_MSIX_ENTRY_BASE + index * PCI_MSIX_ENTRY_SIZE +
		   PCI_MSIX_ENTRY_VECTOR_CTRL;

	val = mask ? PCI_MSIX_ENTRY_CTRL_MASKBIT : 0;

	ALX_MEM_W32(adpt, reg, val);
	ALX_MEM_FLUSH(adpt);
}

static void alx_disable_msi(struct alx_adapter *adpt)
{
	if (ALX_FLAG(adpt, USING_MSI)) {
		pci_disable_msi(adpt->pdev);
		ALX_FLAG_CLEAR(adpt, USING_MSI);
	}
}

static void alx_disable_advanced_intr(struct alx_adapter *adpt)
{
	alx_disable_msix(adpt);
	alx_disable_msi(adpt);
}

static void alx_irq_enable(struct alx_adapter *adpt)
{
	int i;

	if (!atomic_dec_and_test(&adpt->irq_sem))
		return;

	/* level-1 interrupt switch */
	ALX_MEM_W32(adpt, ALX_ISR, 0);
	ALX_MEM_W32(adpt, ALX_IMR, adpt->imask);
	ALX_MEM_FLUSH(adpt);

	if (!ALX_FLAG(adpt, USING_MSIX))
		return;

	/* enable all individual MSIX IRQs */
	for (i = 0; i < adpt->nr_vec; i++)
		alx_mask_msix(adpt, i, false);
}

static void alx_irq_disable(struct alx_adapter *adpt)
{
	int i;

	atomic_inc(&adpt->irq_sem);

	ALX_MEM_W32(adpt, ALX_ISR, ALX_ISR_DIS);
	ALX_MEM_W32(adpt, ALX_IMR, 0);
	ALX_MEM_FLUSH(adpt);

	if (ALX_FLAG(adpt, USING_MSIX)) {
		for (i = 0; i < adpt->nr_vec; i++) {
			alx_mask_msix(adpt, i, true);
			synchronize_irq(adpt->msix_ent[i].vector);
		}
	} else {
		synchronize_irq(adpt->pdev->irq);
	}
}

static int alx_request_irq(struct alx_adapter *adpt)
{
	struct pci_dev *pdev = adpt->pdev;
	int err;
	u32 msi_ctrl;

	adpt->imask = ALX_ISR_MISC;
	msi_ctrl = FIELDX(ALX_MSI_RETRANS_TM, adpt->imt >> 1);

	if (ALX_FLAG(adpt, USING_MSIX)) {
		ALX_MEM_W32(adpt, ALX_MSI_RETRANS_TIMER, msi_ctrl);
		err = alx_request_msix(adpt);
		if (!err)
			goto out;
		/* fall back to MSI or legacy interrupt mode,
		 * re-alloc all resources
		 */
		alx_free_all_ring_resources(adpt);
		alx_disable_msix(adpt);
		adpt->nr_rxq = 1;
		adpt->nr_txq = 1;
		adpt->nr_napi = 1;
		adpt->nr_vec = 1;
		__alx_configure_rss(adpt); /* disable RSS */
		netif_set_real_num_tx_queues(adpt->netdev, adpt->nr_txq);
		netif_set_real_num_rx_queues(adpt->netdev, adpt->nr_rxq);
		if (!pci_enable_msi(pdev))
			ALX_FLAG_SET(adpt, USING_MSI);

		err = alx_setup_all_ring_resources(adpt);
		if (err) {
			dev_err(&pdev->dev,
				"Unable to allocate enough memory err=%d\n",
				err);
			goto out;
		}
	}

	adpt->imask = ALX_ISR_MISC | ALX_ISR_TX_Q0 | ALX_ISR_RX_Q0;

	if (ALX_FLAG(adpt, USING_MSI)) {
		ALX_MEM_W32(adpt, ALX_MSI_RETRANS_TIMER,
			    msi_ctrl | ALX_MSI_MASK_SEL_LINE);
		err = request_irq(pdev->irq, alx_intr_msi, 0,
				  adpt->netdev->name, adpt);
		if (!err)
			goto out;
		/* fall back to legacy interrupt */
		alx_disable_msi(adpt);
	}

	ALX_MEM_W32(adpt, ALX_MSI_RETRANS_TIMER, 0);
	err = request_irq(pdev->irq, alx_intr_legacy, IRQF_SHARED,
			  adpt->netdev->name, adpt);

	if (err)
		dev_err(&pdev->dev,
			"request shared irq failed, err = %d\n",
			err);

out:
	if (unlikely(err))
		dev_err(&pdev->dev, "register IRQ fail %d\n", err);

	return err;
}

static void alx_free_irq(struct alx_adapter *adpt)
{
	struct pci_dev *pdev = adpt->pdev;
	int i, vec;

	if (ALX_FLAG(adpt, USING_MSIX)) {
		free_irq(adpt->msix_ent[0].vector, adpt);
		vec = 1;
		for (i = 0; i < adpt->nr_napi; i++, vec++)
			free_irq(adpt->msix_ent[vec].vector, adpt->qnapi[i]);
	} else {
		free_irq(pdev->irq, adpt);
	}
	alx_disable_advanced_intr(adpt);
}


static int __devinit alx_identify_hw(struct alx_adapter *adpt)
{
	int rev = ALX_REVID(adpt);
	int err = -EINVAL;

	switch (ALX_DID(adpt)) {
	case ALX_DEV_ID_AR8161:
	case ALX_DEV_ID_AR8162:
	case ALX_DEV_ID_AR8171:
	case ALX_DEV_ID_AR8172:
		if (rev > ALX_REV_B0)
			break;
		err = 0;
		ALX_FLAG_SET(adpt, CAP_L0S);
		ALX_FLAG_SET(adpt, CAP_L1);
		adpt->max_dma_chnl = rev == ALX_REV_B0 ? 4 : 2;
		break;
	}

	if (!err && ALX_DID(adpt) & 1) { /* id-bit0: gigabit flag */
		ALX_FLAG_SET(adpt, CAP_GIGA);
	}

	return err;
}

static bool __devinit alx_get_phy_id(struct alx_adapter *adpt)
{
	/* prtad is fixed to 0 for all of chips */
	if (mdio45_probe(&adpt->mdio, 0))
		return false;

	if (alx_read_phy_reg(adpt, MII_PHYSID1, &adpt->phy_id[0]) ||
	    alx_read_phy_reg(adpt, MII_PHYSID2, &adpt->phy_id[1]))
		return false;

	return true;
}

/* some issues are relavant to specific platforms
 * we assign those patches for the chip by pci device id
 * vendor id, subsystem id and revision number
 */
struct alx_platform_patch {
	u16 pci_did;				/* pci device id */
	u8  pci_rev;				/* pci revision id */
	u16 subsystem_vid;
	u16 subsystem_did;
	u32 pflag;				/* patch flag */
#define ALX_PF_LINK		0x00001		/* PHY link issue */
#define ALX_PF_HIB		0x00002		/* Hibernatation issue */
#define ALX_PF_ANY_REV		0x10000		/* not care revision number */
};

static const struct alx_platform_patch plats[] __devinitdata = {
{0x1091, 0x00, 0x1969, 0x0091, 0x1001},
{0},
};

static void __devinit alx_patch_assign(struct alx_adapter *adpt)
{
	int i = 0;

	while (plats[i].pci_did != 0) {
		if (plats[i].pci_did == ALX_DID(adpt) &&
		    plats[i].subsystem_vid == ALX_SUB_VID(adpt) &&
		    plats[i].subsystem_did == ALX_SUB_DID(adpt) &&
		    (plats[i].pflag & ALX_PF_ANY_REV ||
		     plats[i].pci_rev == adpt->pdev->revision)) {
			if (plats[i].pflag & ALX_PF_LINK)
				adpt->lnk_patch = true;
			if (plats[i].pflag & ALX_PF_HIB)
				adpt->hib_patch = true;
		}
		i++;
	}
}

static const u8 def_rss_key[40] __devinitdata = {
	0xE2, 0x91, 0xD7, 0x3D, 0x18, 0x05, 0xEC, 0x6C,
	0x2A, 0x94, 0xB3, 0x0D, 0xA5, 0x4F, 0x2B, 0xEC,
	0xEA, 0x49, 0xAF, 0x7C, 0xE2, 0x14, 0xAD, 0x3D,
	0xB8, 0x55, 0xAA, 0xBE, 0x6A, 0x3E, 0x67, 0xEA,
	0x14, 0x36, 0x4D, 0x17, 0x3B, 0xED, 0x20, 0x0D,
};

static void alx_init_def_rss_idt(struct alx_adapter *adpt)
{
	int i, x, y;
	u32 val;

	for (i = 0; i < adpt->rss_idt_size; i++) {
		val = ethtool_rxfh_indir_default(i, adpt->nr_rxq);
		x = i >> 3;
		y = i * 4 & 0x1F;
		adpt->rss_idt[x] &= ~(0xF << y);
		adpt->rss_idt[x] |= (val & 0xF) << y;
	}
}

/* alx_init_adapter -
 *    initialize general software structure (struct alx_adapter).
 *    fields are inited based on PCI device information.
 */
static int __devinit alx_init_sw(struct alx_adapter *adpt)
{
	struct pci_dev	*pdev = adpt->pdev;
	int i, err;

	err = alx_identify_hw(adpt);
	if (err) {
		dev_err(&pdev->dev, "unrecognize the chip, aborting\n");
		return err;
	}

	/* assign patch flag for specific platforms */
	alx_patch_assign(adpt);

	memcpy(adpt->rss_key, def_rss_key, sizeof(def_rss_key));
	adpt->rss_idt_size = 0x100;
	adpt->rss_hash_type = ALX_RSS_HASH_TYPE_ALL;
	adpt->smb_timer = 400;		/* trigger stats interrupt per 400ms */
	adpt->mtu = adpt->netdev->mtu;
	adpt->rxbuf_size = ALIGN(ALX_RAW_MTU(adpt->mtu), 8);
	adpt->tx_ringsz = 256;
	adpt->rx_ringsz = 512;
	adpt->sleep_ctrl = ALX_SLEEP_WOL_MAGIC | ALX_SLEEP_WOL_PHY;
	adpt->imt = 200;	/* 200us */
	adpt->dma_chnl = adpt->max_dma_chnl;
	adpt->link_up = false;	/* PHY init status */
	adpt->link_duplex = 0;
	adpt->link_speed = SPEED_0;
	adpt->adv_cfg = ADVERTISED_Autoneg |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_1000baseT_Full;
	adpt->flowctrl = ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX;
	adpt->wrr_ctrl = ALX_WRR_PRI_RESTRICT_NONE;
	for (i = 0; i < ARRAY_SIZE(adpt->wrr); i++)
		adpt->wrr[i] = 4;

	adpt->rx_ctrl = ALX_MAC_CTRL_WOLSPED_SWEN |
			ALX_MAC_CTRL_MHASH_ALG_HI5B |
			FIELDX(ALX_MAC_CTRL_PRMBLEN, 7);

	atomic_set(&adpt->irq_sem, 1);
	ALX_FLAG_SET(adpt, HALT);

	return err;
}

static netdev_features_t alx_fix_features(struct net_device *netdev,
					  netdev_features_t features)
{
	/*
	 * Since there is no support for separate rx/tx vlan accel
	 * enable/disable make sure tx flag is always in same state as rx.
	 */
	if (features & NETIF_F_HW_VLAN_RX)
		features |= NETIF_F_HW_VLAN_TX;
	else
		features &= ~NETIF_F_HW_VLAN_TX;

	if (netdev->mtu > ALX_MAX_TSO_PKT_SIZE)
		features &= ~(NETIF_F_TSO | NETIF_F_TSO6);

	return features;
}


static int alx_set_features(struct net_device *netdev,
			    netdev_features_t features)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	netdev_features_t changed = netdev->features ^ features;

	if (!(changed & NETIF_F_HW_VLAN_RX))
		return 0;

	if (features & NETIF_F_HW_VLAN_RX)
		adpt->rx_ctrl |= ALX_MAC_CTRL_VLANSTRIP;
	else
		adpt->rx_ctrl &= ~ALX_MAC_CTRL_VLANSTRIP;

	ALX_MEM_W32(adpt, ALX_MAC_CTRL, adpt->rx_ctrl);

	return 0;
}

/* alx_change_mtu - Change the Maximum Transfer Unit */
static int alx_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	int old_mtu   = netdev->mtu;
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;

	if ((max_frame < ALX_MIN_FRAME_SIZE) ||
	    (max_frame > ALX_MAX_FRAME_SIZE)) {
		netdev_err(netdev, "invalid MTU setting\n");
		return -EINVAL;
	}
	/* set MTU */
	if (old_mtu != new_mtu && netif_running(netdev)) {
		netdev_info(adpt->netdev,
			   "changing MTU from %d to %d\n",
			   netdev->mtu, new_mtu);
		netdev->mtu = new_mtu;
		adpt->rxbuf_size = new_mtu > ALX_DEF_RXBUF_SIZE ?
				   ALIGN(max_frame, 8) : ALX_DEF_RXBUF_SIZE;
		netdev_update_features(netdev);
		alx_reinit(adpt);
	}

	return 0;
}

/* configure hardware everything except:
 *  1. interrupt vectors
 *  2. enable control for rx modules
 */
static void alx_configure(struct alx_adapter *adpt)
{
	u32 val, raw_mtu, max_payload;
	u16 val16;
	u8 chip_rev = ALX_REVID(adpt);

	/* mac address */
	alx_set_macaddr(adpt, adpt->mac_addr);

#if 0
	/* clear WoL setting/status */
	ALX_MEM_R32(hw, ALX_WOL0, &val);
	ALX_MEM_W32(hw, ALX_WOL0, 0);
#endif
	/* clk gating */
	ALX_MEM_W32(adpt, ALX_CLK_GATE, ALX_CLK_GATE_ALL_A0);

	/* idle timeout to switch clk_125M */
	if (chip_rev == ALX_REV_B0) {
		ALX_MEM_W32(adpt, ALX_IDLE_DECISN_TIMER,
			ALX_IDLE_DECISN_TIMER_DEF);
	}

	/* stats refresh timeout */
	ALX_MEM_W32(adpt, ALX_SMB_TIMER, adpt->smb_timer * 500UL);

	/* intr moduration */
	ALX_MEM_R32(adpt, ALX_MASTER, &val);
	val = val | ALX_MASTER_IRQMOD2_EN |
		    ALX_MASTER_IRQMOD1_EN |
		    ALX_MASTER_SYSALVTIMER_EN;  /* sysalive */
	ALX_MEM_W32(adpt, ALX_MASTER, val);
	ALX_MEM_W32(adpt, ALX_IRQ_MODU_TIMER,
		    FIELDX(ALX_IRQ_MODU_TIMER1, adpt->imt >> 1));
	/* intr re-trig timeout */
	ALX_MEM_W32(adpt, ALX_INT_RETRIG, ALX_INT_RETRIG_TO);
	/* tpd threshold to trig int */
	ALX_MEM_W32(adpt, ALX_TINT_TPD_THRSHLD, adpt->tx_ringsz / 3);
	ALX_MEM_W32(adpt, ALX_TINT_TIMER, adpt->imt);

	/* mtu */
	raw_mtu = adpt->mtu + ETH_HLEN;
	ALX_MEM_W32(adpt, ALX_MTU, raw_mtu + 8); /* crc + vlan */
	if (raw_mtu > ALX_MTU_JUMBO_TH)
		adpt->rx_ctrl &= ~ALX_MAC_CTRL_FAST_PAUSE;

	/* txq */
	if ((raw_mtu + 8) < ALX_TXQ1_JUMBO_TSO_TH)
		val = (raw_mtu + 8 + 7) >> 3; /* 7 for QWORD align */
	else
		val = ALX_TXQ1_JUMBO_TSO_TH >> 3;
	ALX_MEM_W32(adpt, ALX_TXQ1, val | ALX_TXQ1_ERRLGPKT_DROP_EN);
	max_payload = pcie_get_readrq(adpt->pdev) >> 8;
	/*
	 * if BIOS had changed the default dma read max length,
	 * restore it to default value
	 */
	if (max_payload < ALX_DEV_CTRL_MAXRRS_MIN)
		pcie_set_readrq(adpt->pdev, 128 << ALX_DEV_CTRL_MAXRRS_MIN);

	val = FIELDX(ALX_TXQ0_TPD_BURSTPREF, ALX_TXQ_TPD_BURSTPREF_DEF) |
	      ALX_TXQ0_MODE_ENHANCE |
	      ALX_TXQ0_LSO_8023_EN |
	      ALX_TXQ0_SUPT_IPOPT |
	      FIELDX(ALX_TXQ0_TXF_BURST_PREF, ALX_TXQ_TXF_BURST_PREF_DEF);
	ALX_MEM_W32(adpt, ALX_TXQ0, val);
	val = FIELDX(ALX_HQTPD_Q1_NUMPREF, ALX_TXQ_TPD_BURSTPREF_DEF) |
	      FIELDX(ALX_HQTPD_Q2_NUMPREF, ALX_TXQ_TPD_BURSTPREF_DEF) |
	      FIELDX(ALX_HQTPD_Q3_NUMPREF, ALX_TXQ_TPD_BURSTPREF_DEF) |
	      ALX_HQTPD_BURST_EN;
	ALX_MEM_W32(adpt, ALX_HQTPD, val);

	/* rxq, flow control */
	ALX_MEM_R32(adpt, ALX_SRAM5, &val);
	val = FIELD_GETX(val, ALX_SRAM_RXF_LEN) << 3; /* bytes */
	if (val > ALX_SRAM_RXF_LEN_8K) {
		val16 = ALX_MTU_STD_ALGN >> 3;
		val = (val - ALX_RXQ2_RXF_FLOW_CTRL_RSVD) >> 3;
	} else {
		val16 = ALX_MTU_STD_ALGN >> 3;
		val = (val - ALX_MTU_STD_ALGN) >> 3;
	}
	ALX_MEM_W32(adpt, ALX_RXQ2,
		    FIELDX(ALX_RXQ2_RXF_XOFF_THRESH, val16) |
		    FIELDX(ALX_RXQ2_RXF_XON_THRESH, val));
	val = FIELDX(ALX_RXQ0_NUM_RFD_PREF, ALX_RXQ0_NUM_RFD_PREF_DEF) |
	      FIELDX(ALX_RXQ0_RSS_MODE, ALX_RXQ0_RSS_MODE_DIS) |
	      FIELDX(ALX_RXQ0_IDT_TBL_SIZE, ALX_RXQ0_IDT_TBL_SIZE_DEF) |
	      ALX_RXQ0_RSS_HSTYP_ALL |
	      ALX_RXQ0_RSS_HASH_EN |
	      ALX_RXQ0_IPV6_PARSE_EN;
	if (ALX_FLAG(adpt, CAP_GIGA)) {
		FIELD_SET32(val, ALX_RXQ0_ASPM_THRESH,
			    ALX_RXQ0_ASPM_THRESH_100M);
	}
	ALX_MEM_W32(adpt, ALX_RXQ0, val);

	/* DMA */
	ALX_MEM_R32(adpt, ALX_DMA, &val);
	val = FIELDX(ALX_DMA_RORDER_MODE, ALX_DMA_RORDER_MODE_OUT) |
	      ALX_DMA_RREQ_PRI_DATA |
	      FIELDX(ALX_DMA_RREQ_BLEN, max_payload) |
	      FIELDX(ALX_DMA_WDLY_CNT, ALX_DMA_WDLY_CNT_DEF) |
	      FIELDX(ALX_DMA_RDLY_CNT, ALX_DMA_RDLY_CNT_DEF) |
	      FIELDX(ALX_DMA_RCHNL_SEL, adpt->dma_chnl - 1);
	ALX_MEM_W32(adpt, ALX_DMA, val);

	/* multi-tx-q weight */
	if (ALX_FLAG(adpt, CAP_MTQ)) {
		val = FIELDX(ALX_WRR_PRI, adpt->wrr_ctrl) |
		      FIELDX(ALX_WRR_PRI0, adpt->wrr[0]) |
		      FIELDX(ALX_WRR_PRI1, adpt->wrr[1]) |
		      FIELDX(ALX_WRR_PRI2, adpt->wrr[2]) |
		      FIELDX(ALX_WRR_PRI3, adpt->wrr[3]);
		ALX_MEM_W32(adpt, ALX_WRR, val);
	}

	/* multi-rx-q */
	__alx_configure_rss(adpt);

	/* rx mode */
	__alx_set_rx_mode(adpt->netdev);
}

static int __alx_open(struct alx_adapter *adpt)
{
	int i, err;

	netif_carrier_off(adpt->netdev);

	/* allocate all memory resources */
	err = alx_setup_all_ring_resources(adpt);
	if (err)
		goto err_out;

	/* make hardware ready before allocate interrupt */
	alx_configure(adpt);

	err = alx_request_irq(adpt);
	if (err)
		goto err_out;

	ALX_FLAG_CLEAR(adpt, HALT);

	for (i = 0; i < adpt->nr_napi; i++)
		napi_enable(&(adpt->qnapi[i]->napi));

	/* clear old interrupts */
	ALX_MEM_W32(adpt, ALX_ISR, (u32)~ALX_ISR_DIS);

	alx_irq_enable(adpt);

	netif_tx_start_all_queues(adpt->netdev);

	ALX_FLAG_SET(adpt, TASK_CHK_LINK);
	alx_schedule_work(adpt);
	return 0;

err_out:

	alx_free_all_ring_resources(adpt);
	alx_disable_advanced_intr(adpt);
	return err;
}

static void alx_halt(struct alx_adapter *adpt)
{
	struct net_device *netdev = adpt->netdev;
	int i;

	ALX_FLAG_SET(adpt, HALT);

	alx_cancel_work(adpt);
	/* stop TX/RX */
	alx_stop_mac(adpt);
	/* disable L0S/L1 */
	alx_enable_aspm(adpt, false, false);

	netif_carrier_off(netdev);
	for (i = 0; i < adpt->nr_napi; i++)
		napi_disable(&(adpt->qnapi[i]->napi));

	netif_tx_disable(netdev);

	alx_irq_disable(adpt);

	alx_free_all_rings_buf(adpt);
}

static void alx_activate(struct alx_adapter *adpt)
{
	int i;

	for (i = 0; i < adpt->nr_rxq; i++)
		alx_alloc_rxring_buf(adpt, adpt->qnapi[i]->rxq);

	/* hardware setting lost, restore it */
	alx_configure(adpt);

	ALX_FLAG_CLEAR(adpt, HALT);

	for (i = 0; i < adpt->nr_napi; i++)
		napi_enable(&(adpt->qnapi[i]->napi));

	/* clear old interrupts */
	ALX_MEM_W32(adpt, ALX_ISR, (u32)~ALX_ISR_DIS);

	alx_irq_enable(adpt);

	netif_tx_wake_all_queues(adpt->netdev);

	ALX_FLAG_SET(adpt, TASK_CHK_LINK);
	alx_schedule_work(adpt);
}

static void __alx_stop(struct alx_adapter *adpt)
{
	alx_halt(adpt);

	alx_free_irq(adpt);

	alx_free_all_ring_resources(adpt);
}

static bool alx_enable_msix(struct alx_adapter *adpt)
{
	int nr_txq, nr_rxq, vec_req;
	int i, err;

	nr_txq = min_t(int, num_online_cpus(), ALX_MAX_TX_QUEUES);
	nr_rxq = min_t(int, num_online_cpus(), ALX_MAX_RX_QUEUES);
	nr_rxq = rounddown_pow_of_two(nr_rxq);
	/* one more vector for PHY link change & timer & other events */
	vec_req = max_t(int, nr_txq, nr_rxq) + 1;

	if (vec_req  <= 2) {
		netdev_info(adpt->netdev,
			    "cpu core num is less, MSI-X isn't necessary\n");
		return false; /* try msi */
	}

	adpt->msix_ent = kcalloc(vec_req,
				 sizeof(struct msix_entry),
				 GFP_KERNEL);
	if (!adpt->msix_ent) {
		netdev_info(adpt->netdev, "can't alloc msix entries\n");
		return false;
	}
	for (i = 0; i < vec_req; i++)
		adpt->msix_ent[i].entry = i;

	err = pci_enable_msix(adpt->pdev, adpt->msix_ent, vec_req);
	if (err) {
		kfree(adpt->msix_ent);
		adpt->msix_ent = NULL;
		netdev_info(adpt->netdev, "can't enable MSI-X interrupt\n");
		return false;
	}

	adpt->nr_txq = nr_txq;
	adpt->nr_rxq = nr_rxq;
	adpt->nr_vec = vec_req;
	adpt->nr_napi = vec_req - 1;

	return true;
}

static void alx_init_intr(struct alx_adapter *adpt)
{
	if ((ALX_FLAG(adpt, CAP_MTQ) || ALX_FLAG(adpt, CAP_RSS)) &&
	    ALX_FLAG(adpt, CAP_MSIX)) {
		if (alx_enable_msix(adpt))
			ALX_FLAG_SET(adpt, USING_MSIX);
	}
	if (!ALX_FLAG(adpt, USING_MSIX)) {
		adpt->nr_txq = ALX_FLAG(adpt, CAP_MTQ) ?
			ALX_MAX_TX_QUEUES : 1;
		adpt->nr_rxq = 1;
		adpt->nr_napi = 1;
		adpt->nr_vec = 1;
	}

	netif_set_real_num_tx_queues(adpt->netdev, adpt->nr_txq);
	netif_set_real_num_rx_queues(adpt->netdev, adpt->nr_rxq);
}

static void alx_init_ring_ptrs(struct alx_adapter *adpt)
{
	struct alx_napi *np;
	int i, tx_idx, rx_idx;
	u32 addr_hi;
	u16 txring_header_reg[] = {ALX_TPD_PRI0_ADDR_LO, ALX_TPD_PRI1_ADDR_LO,
				   ALX_TPD_PRI2_ADDR_LO, ALX_TPD_PRI3_ADDR_LO};
	u16 rfdring_header_reg[] = {ALX_RFD_ADDR_LO};
	u16 rrdring_header_reg[] = {ALX_RRD_ADDR_LO};

	tx_idx = 0;
	rx_idx = 0;
	for (i = 0; i < adpt->nr_napi; i++) {
		np = adpt->qnapi[i];
		if (np->rxq) {
			np->rxq->pidx = 0;
			np->rxq->cidx = 0;
			np->rxq->rrd_cidx = 0;
			if (!ALX_FLAG(adpt, CAP_MRQ) && rx_idx == 0) {
				ALX_MEM_W32(adpt, rfdring_header_reg[0],
					    np->rxq->rfd_dma);
				ALX_MEM_W32(adpt, rrdring_header_reg[0],
					    np->rxq->rrd_dma);
			}
			rx_idx++;
		}
		if (np->txq) {
			np->txq->pidx = 0;
			atomic_set(&np->txq->cidx, 0);
			ALX_MEM_W32(adpt, txring_header_reg[tx_idx],
				np->txq->tpd_dma);
			tx_idx++;
		}
	}

	addr_hi = ((u64)adpt->ring_header.dma) >> 32;
	ALX_MEM_W32(adpt, ALX_TX_BASE_ADDR_HI, addr_hi);
	ALX_MEM_W32(adpt, ALX_RX_BASE_ADDR_HI, addr_hi);
	ALX_MEM_W32(adpt, ALX_TPD_RING_SZ, adpt->tx_ringsz);
	ALX_MEM_W32(adpt, ALX_RRD_RING_SZ, adpt->rx_ringsz);
	ALX_MEM_W32(adpt, ALX_RFD_RING_SZ, adpt->rx_ringsz);

	/* load these ptrs into chip internal */
	ALX_MEM_W32(adpt, ALX_SRAM9, ALX_SRAM_LOAD_PTR);
}

static void alx_show_speed(struct net_device *dev, u16 speed)
{
	char *desc;

	desc = speed == SPEED_1000 + FULL_DUPLEX ?
		"1 Gbps Duplex Full" :
		speed == SPEED_100 + FULL_DUPLEX ?
		"100 Mbps Duplex Full" :
		speed == SPEED_100 + HALF_DUPLEX ?
		"100 Mbps Duplex Half" :
		speed == SPEED_10 + FULL_DUPLEX ?
		"10 Mbps Duplex Full" :
		speed == SPEED_10 + HALF_DUPLEX ?
		"10 Mbps Duplex Half" :
		"Unknown speed";
	netdev_info(dev, "NIC Link Up: %s\n", desc);
}

static void alx_check_link(struct alx_adapter *adpt)
{
	u16 speed, old_speed;
	bool link_up;
	int i, err;

	if (ALX_FLAG(adpt, HALT))
		return;

	err = alx_get_phy_link(adpt, &link_up, &speed);
	if (err)
		goto out;

	/* open interrutp mask */
	adpt->imask |= ALX_ISR_PHY;
	ALX_MEM_W32(adpt, ALX_IMR, adpt->imask);

	if (!link_up && !adpt->link_up)
		goto out;

	old_speed = adpt->link_speed + adpt->link_duplex;

	if (link_up) {
		if (adpt->link_up && old_speed == speed)
			goto out; /* same speed */

		alx_show_speed(adpt->netdev, speed);
		adpt->link_duplex = speed % 10;
		adpt->link_speed = speed - adpt->link_duplex;
		adpt->link_up = true;
		alx_post_phy_link(adpt, adpt->link_speed,
				  ALX_FLAG(adpt, CAP_AZ));

		alx_enable_aspm(adpt, ALX_FLAG(adpt, CAP_L0S),
			ALX_FLAG(adpt, CAP_L1));
		alx_start_mac(adpt);
		/* link kept, just speed changed */
		if (adpt->link_up)
			goto out;
		/* link changed from 'down' to 'up' */
		netif_tx_wake_all_queues(adpt->netdev);
		netif_carrier_on(adpt->netdev);
		goto out;
	}

	/* link changed from 'up' to 'down' */
	netif_carrier_off(adpt->netdev);
	adpt->link_up = false;
	adpt->link_speed = SPEED_0;
	netdev_info(adpt->netdev, "NIC Link Down\n");

	alx_stop_mac(adpt);
	alx_enable_aspm(adpt, false, ALX_FLAG(adpt, CAP_L1));
	alx_post_phy_link(adpt, SPEED_0, ALX_FLAG(adpt, CAP_AZ));

	/* free any pending tx skbs */
	netif_tx_disable(adpt->netdev);
	for (i = 0; i < adpt->nr_txq; i++)
		alx_free_txring_buf(adpt->qnapi[i]->txq);

out:
	if (err) {
		ALX_FLAG_SET(adpt, TASK_RESET);
		alx_schedule_work(adpt);
	}
}

/* alx_open - Called when a network interface is made active */
static int alx_open(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	int err;

	/* decide interrupt mode, some resources allocation depend on it */
	alx_init_intr(adpt);

	/* init rss indirection table */
	alx_init_def_rss_idt(adpt);

	err = __alx_open(adpt);

	return err;
}

/* alx_stop - Disables a network interface */
static int alx_stop(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	WARN_ON(ALX_FLAG(adpt, RESETING));

	__alx_stop(adpt);

	return 0;
}

static int alx_select_powersaving_speed(struct alx_adapter *adpt, u16 *speed)
{
	int i, err;
	u16 spd, lpa;
	bool linkup;

	err = alx_get_phy_link(adpt, &linkup, &spd);
	if (err)
		goto out;

	if (!linkup) {
		*speed = SPEED_0;
		goto out;
	}

	err = alx_read_phy_reg(adpt, MII_LPA, &lpa);
	if (err)
		goto out;

	if (!(lpa & LPA_LPACK)) { /* force mode */
		*speed = spd;
		goto out;
	}
	if (lpa & LPA_100FULL)
		*speed = SPEED_100 + FULL_DUPLEX;
	else if (lpa & LPA_100HALF)
		*speed = SPEED_100 + HALF_DUPLEX;
	else if (lpa & LPA_10FULL)
		*speed = SPEED_10 + FULL_DUPLEX;
	else
		*speed = SPEED_10 + HALF_DUPLEX;

	if (*speed != spd) { /* restart PHY link phase */

		err = alx_setup_speed_duplex(adpt,
			ALX_SPEED_TO_ETHADV(*speed) | ADVERTISED_Autoneg,
			ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX);
		if (err)
			goto out;

		/* wait for linkup */
		for (i = 0; i < ALX_MAX_SETUP_LNK_CYCLE; i++) {
			u16 speed2;
			bool link_on;

			msleep(100);
			err = alx_get_phy_link(adpt, &link_on, &speed2);
			if (err)
				goto out;
			if (link_on)
				break;
		}
		if (i == ALX_MAX_SETUP_LNK_CYCLE) {
			netdev_err(adpt->netdev,
				   "config PHY speed/duplex failed,err=%d\n",
				   err);
			err = -EIO;
			goto out;
		}
	}

out:
	return err;
}

static int __alx_shutdown(struct pci_dev *pdev, bool *wol_en)
{
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	struct net_device *netdev = adpt->netdev;
	int err;
	u16 speed;

	netif_device_detach(netdev);
	if (netif_running(netdev))
		__alx_stop(adpt);

#ifdef CONFIG_PM_SLEEP
	err = pci_save_state(pdev);
	if (err)
		goto out;
#endif

	err = alx_select_powersaving_speed(adpt, &speed);
	if (!err)
		err = alx_clear_phy_intr(adpt);
	if (!err)
		err = alx_config_wol(adpt);
	if (!err)
		err = alx_pre_suspend(adpt, speed);
	if (err)
		goto out;

	*wol_en = false;
	if (adpt->sleep_ctrl & ALX_SLEEP_ACTIVE) {
		device_set_wakeup_enable(&pdev->dev, true);
		*wol_en = true;
	}

	pci_disable_device(pdev);

out:
	return err;
}

static void alx_shutdown(struct pci_dev *pdev)
{
	int err;
	bool wol_en;

	err = __alx_shutdown(pdev, &wol_en);
	if (likely(!err)) {
		pci_wake_from_d3(pdev, wol_en);
		pci_set_power_state(pdev, PCI_D3hot);
	} else {
		dev_err(&pdev->dev, "shutdown fail %d\n", err);
	}
}


#ifdef CONFIG_PM_SLEEP
static int alx_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int err;
	bool wol_en;

	err = __alx_shutdown(pdev, &wol_en);
	if (unlikely(err)) {
		dev_err(&pdev->dev, "shutdown fail in suspend %d\n", err);
		goto out;
	}
	if (wol_en) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

out:
	return err;
}

static int alx_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	struct net_device *netdev = adpt->netdev;
	int err;

	if (!netif_running(netdev))
		return 0;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	adpt->link_up = false;
	adpt->link_speed = SPEED_0;

	alx_reset_pcie(adpt);
	alx_reset_phy(adpt, !adpt->hib_patch);
	err = alx_reset_mac(adpt);
	if (err) {
		dev_err(&pdev->dev, "resume:reset_mac fail %d\n", err);
		return err;
	}
	err = alx_setup_speed_duplex(adpt, adpt->adv_cfg, adpt->flowctrl);
	if (err) {
		dev_err(&pdev->dev ,
			"resume:setup_speed_duplex fail %d\n",
			err);
		return err;
	}

	if (netif_running(netdev)) {
		err = __alx_open(adpt);
		if (err)
			return err;
	}

	netif_device_attach(netdev);

	return err;
}
#endif



/* alx_update_hw_stats - Update the board statistics counters. */
static void alx_update_hw_stats(struct alx_adapter *adpt)
{
	u16 reg;
	u32 data;
	unsigned long *p;

	if (ALX_FLAG(adpt, HALT) || ALX_FLAG(adpt, RESETING))
		return;

	/* RX stats */
	reg = ALX_RX_STATS_BIN;
	p = &adpt->hw_stats.rx_ok;
	while (reg <=  ALX_RX_STATS_END) {
		ALX_MEM_R32(adpt, reg, &data);
		*p++ += data;
		reg += 4;
	}

	/* TX stats */
	reg = ALX_TX_STATS_BIN;
	p = &adpt->hw_stats.tx_ok;
	while (reg <= ALX_TX_STATS_END) {
		ALX_MEM_R32(adpt, reg, &data);
		*p++ += data;
		reg += 4;
	}
}

/* alx_get_stats - Get System Network Statistics
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 */
static struct net_device_stats *alx_get_stats(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct net_device_stats *net_stats = &netdev->stats;
	struct alx_hw_stats *hw_stats = &adpt->hw_stats;

	spin_lock(&adpt->smb_lock);

	alx_update_hw_stats(adpt);

	net_stats->tx_packets = hw_stats->tx_ok;
	net_stats->tx_bytes   = hw_stats->tx_byte_cnt;
	net_stats->rx_packets = hw_stats->rx_ok;
	net_stats->rx_bytes   = hw_stats->rx_byte_cnt;
	net_stats->multicast  = hw_stats->rx_mcast;
	net_stats->collisions = hw_stats->tx_single_col +
				hw_stats->tx_multi_col * 2 +
				hw_stats->tx_late_col + hw_stats->tx_abort_col;

	net_stats->rx_errors  = hw_stats->rx_frag + hw_stats->rx_fcs_err +
				hw_stats->rx_len_err + hw_stats->rx_ov_sz +
				hw_stats->rx_ov_rrd + hw_stats->rx_align_err;

	net_stats->rx_fifo_errors   = hw_stats->rx_ov_rxf;
	net_stats->rx_length_errors = hw_stats->rx_len_err;
	net_stats->rx_crc_errors    = hw_stats->rx_fcs_err;
	net_stats->rx_frame_errors  = hw_stats->rx_align_err;
	net_stats->rx_over_errors   = hw_stats->rx_ov_rrd + hw_stats->rx_ov_rxf;

	net_stats->rx_missed_errors = hw_stats->rx_ov_rrd + hw_stats->rx_ov_rxf;

	net_stats->tx_errors = hw_stats->tx_late_col + hw_stats->tx_abort_col +
			       hw_stats->tx_underrun + hw_stats->tx_trunc;

	net_stats->tx_aborted_errors = hw_stats->tx_abort_col;
	net_stats->tx_fifo_errors    = hw_stats->tx_underrun;
	net_stats->tx_window_errors  = hw_stats->tx_late_col;

	spin_unlock(&adpt->smb_lock);

	return net_stats;
}

static void alx_update_stats(struct alx_adapter *adpt)
{
	spin_lock(&adpt->smb_lock);
	alx_update_hw_stats(adpt);
	spin_unlock(&adpt->smb_lock);
}

void alx_reinit(struct alx_adapter *adpt)
{
	WARN_ON(in_interrupt());

	while (test_and_set_bit(ALX_FLAG_RESETING, adpt->flags))
		msleep(20);

	if (ALX_FLAG(adpt, HALT))
		return;

	alx_halt(adpt);
	alx_activate(adpt);

	ALX_FLAG_CLEAR(adpt, RESETING);
}

/* alx_task - manages and runs subtasks */
static void alx_task(struct work_struct *work)
{
	struct alx_adapter *adpt = container_of(work, struct alx_adapter, task);

	/* don't support reentrance */
	while (test_and_set_bit(ALX_FLAG_TASK_PENDING, adpt->flags))
		msleep(20);

	if (ALX_FLAG(adpt, HALT))
		goto out;

	if (!test_and_clear_bit(ALX_FLAG_TASK_RESET, adpt->flags))
		alx_reinit(adpt);

	if (!test_and_clear_bit(ALX_FLAG_TASK_UPDATE_SMB, adpt->flags))
		alx_update_stats(adpt);

	if (!test_and_clear_bit(ALX_FLAG_TASK_CHK_LINK, adpt->flags))
		alx_check_link(adpt);

out:
	ALX_FLAG_CLEAR(adpt, TASK_PENDING);
}


static irqreturn_t alx_msix_ring(int irq, void *data)
{
	struct alx_napi *np = data;
	struct alx_adapter *adpt = np->adpt;
	u32 intr;

	/* mask interrupt to ACK chip */
	alx_mask_msix(adpt, np->vec_idx, true);
	ALX_MEM_R32(adpt, ALX_ISR, &intr);
	intr &= ~np->vec_mask;

	if (!ALX_FLAG(adpt, HALT))
		napi_schedule(&np->napi);

	/* clear interrutp status */
	ALX_MEM_W32(adpt, ALX_ISR, intr);

	return IRQ_HANDLED;
}

static inline bool alx_handle_intr_misc(struct alx_adapter *adpt, u32 intr)
{
	if (unlikely(intr & ALX_ISR_FATAL)) {
		ALX_FLAG_SET(adpt, TASK_RESET);
		alx_schedule_work(adpt);
		return true;
	}

	if (intr & ALX_ISR_ALERT)
		netdev_warn(adpt->netdev, "interrutp alert :%x\n", intr);

	if (intr & ALX_ISR_SMB) {
		ALX_FLAG_SET(adpt, TASK_UPDATE_SMB);
		alx_schedule_work(adpt);
	}

	if (intr & ALX_ISR_PHY) {
		/* suppress PHY interrupt, because the source
		 * is from PHY internal. only the internal status
		 * is cleared, the interrupt status could be cleared.
		 */
		adpt->imask &= ~ALX_ISR_PHY;
		ALX_MEM_W32(adpt, ALX_IMR, adpt->imask);
		ALX_FLAG_SET(adpt, TASK_CHK_LINK);
		alx_schedule_work(adpt);
	}

	return false;
}

static irqreturn_t alx_intr_msix_misc(int irq, void *data)
{
	struct alx_adapter *adpt = data;
	u32 intr;

	/* mask interrupt to ACK chip */
	alx_mask_msix(adpt, 0, true);

	/* read interrupt status */
	ALX_MEM_R32(adpt, ALX_ISR, &intr);
	intr &= (adpt->imask & ~ALX_ISR_ALL_QUEUES);

	if (alx_handle_intr_misc(adpt, intr))
		return IRQ_HANDLED;

	/* clear interrupt status */
	ALX_MEM_W32(adpt, ALX_ISR, intr);

	/* enable interrupt again */
	if (!ALX_FLAG(adpt, HALT))
		alx_mask_msix(adpt, 0, false);

	return IRQ_HANDLED;
}

static inline irqreturn_t alx_intr_1(struct alx_adapter *adpt, u32 intr)
{
	/* ACK interrupt */
	ALX_MEM_W32(adpt, ALX_ISR, ALX_ISR_DIS);
	intr &= adpt->imask;

	if (alx_handle_intr_misc(adpt, intr))
		return IRQ_HANDLED;

	if (intr & (ALX_ISR_TX_Q0 | ALX_ISR_RX_Q0)) {
		napi_schedule(&adpt->qnapi[0]->napi);
		/* mask rx/tx interrupt, enable them when napi complete */
		ALX_MEM_W32(adpt, ALX_IMR, adpt->imask & ~ALX_ISR_ALL_QUEUES);
	}

	/* clear interrupt status */
	ALX_MEM_W32(adpt, ALX_ISR, intr & ~ALX_ISR_DIS);

	return IRQ_HANDLED;
}


static irqreturn_t alx_intr_msi(int irq, void *data)
{
	struct alx_adapter *adpt = data;
	u32 intr;

	/* read interrupt status */
	ALX_MEM_R32(adpt, ALX_ISR, &intr);

	return alx_intr_1(adpt, intr);
}

static irqreturn_t alx_intr_legacy(int irq, void *data)
{
	struct alx_adapter *adpt = data;
	u32 intr;

	/* read interrupt status */
	ALX_MEM_R32(adpt, ALX_ISR, &intr);
	if (intr & ALX_ISR_DIS || 0 == (intr & adpt->imask))
		return IRQ_NONE;

	return alx_intr_1(adpt, intr);
}


static int alx_poll(struct napi_struct *napi, int budget)
{
	struct alx_napi *np = container_of(napi, struct alx_napi, napi);
	struct alx_adapter *adpt = np->adpt;
	bool complete = true;

	if (np->txq)
		complete = alx_clean_tx_irq(np->txq);
	if (np->rxq)
		complete &= alx_clean_rx_irq(np->rxq, budget);

	if (!complete)
		return budget;

	/* rx-packet finished, exit the polling mode */
	napi_complete(&np->napi);

	/* enable interrupt */
	if (!ALX_FLAG(adpt, HALT)) {
		if (ALX_FLAG(adpt, USING_MSIX))
			alx_mask_msix(adpt, np->vec_idx, false);
		else
			ALX_MEM_W32(adpt, ALX_IMR, adpt->imask);
		ALX_MEM_FLUSH(adpt);
	}

	return 0;
}

static inline struct alx_tx_queue *alx_tx_queue_mapping(
			struct alx_adapter *adpt,
			struct sk_buff *skb)
{
	int index = skb_get_queue_mapping(skb);

	if (index >= adpt->nr_txq)
		index = index % adpt->nr_txq;

	return adpt->qnapi[index]->txq;
}

static inline int alx_tpd_req(struct sk_buff *skb)
{
	int num;

	num = skb_shinfo(skb)->nr_frags + 1;
	if (skb_is_gso(skb) && skb_shinfo(skb)->gso_type & SKB_GSO_TCPV6)
		num++;

	return num;
}

static int alx_tx_csum(struct sk_buff *skb, struct tpd_desc *first)
{
	u8 cso, css;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	cso = skb_checksum_start_offset(skb);
	if (cso & 0x1)
		return -1; /* drop it */

	css = cso + skb->csum_offset;
	first->word1 |= FIELDX(TPD_CXSUMSTART, cso >> 1);
	first->word1 |= FIELDX(TPD_CXSUMOFFSET, css >> 1);
	first->word1 |= 1 << TPD_CXSUM_EN_SHIFT;

	return 1;
}

static int alx_tso(struct sk_buff *skb, struct tpd_desc *first)
{
	int hdr_len;
	int err;

	if (!skb_is_gso(skb))
		return 0;

	if (skb_header_cloned(skb)) {
		err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (unlikely(err))
			return err;
	}

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph;

		iph = ip_hdr(skb);
		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		iph->check = 0;
		tcp_hdr(skb)->check = ~csum_tcpudp_magic(
						iph->saddr,
						iph->daddr,
						0, IPPROTO_TCP, 0);
		first->word1 |= 1 << TPD_IPV4_SHIFT;
		first->word1 |= FIELDX(TPD_L4HDROFFSET,
				       skb_transport_offset(skb));
		if (unlikely(skb->len == hdr_len)) {
			/* no tcp payload */
			first->word1 |= 1 << TPD_IP_XSUM_SHIFT;
			first->word1 |= 1 << TPD_TCP_XSUM_SHIFT;
			return 0;
		}
		first->word1 |= 1 << TPD_LSO_EN_SHIFT;
		first->word1 |= FIELDX(TPD_MSS, skb_shinfo(skb)->gso_size);
	} else if (skb_is_gso_v6(skb)) {
		struct ipv6hdr *ip6h;

		ip6h = ipv6_hdr(skb);
		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		ip6h->payload_len = 0;
		tcp_hdr(skb)->check = ~csum_ipv6_magic(
						&ip6h->saddr,
						&ip6h->daddr,
						0, IPPROTO_TCP, 0);
		first->word1 |= FIELDX(TPD_L4HDROFFSET,
				       skb_transport_offset(skb));
		if (unlikely(skb->len == hdr_len)) {
			/* no tcp payload */
			ip6h->payload_len = skb->len -
				((unsigned char *)ip6h - skb->data) -
				sizeof(struct ipv6hdr);
			first->word1 |= 1 << TPD_IP_XSUM_SHIFT;
			first->word1 |= 1 << TPD_TCP_XSUM_SHIFT;
			return 0;
		}
		/* for LSOv2, the 1st TPD just provides packet length */
		first->adrl.l.pkt_len = skb->len;
		first->word1 |= 1 << TPD_LSO_EN_SHIFT;
		first->word1 |= 1 << TPD_LSO_V2_SHIFT;
		first->word1 |= FIELDX(TPD_MSS, skb_shinfo(skb)->gso_size);
	}

	return 1;
}

static int alx_tx_map(struct alx_tx_queue *txq, struct sk_buff *skb)
{
	struct tpd_desc *tpd, *first_tpd;
	struct alx_buffer *buf, *first_buf;
	dma_addr_t dma;
	u16 producer, maplen, f;

	producer = txq->pidx;

	first_tpd = txq->tpd_hdr + producer;
	first_buf = txq->bf_info + producer;
	tpd = first_tpd;
	buf = first_buf;
	if (tpd->word1 & (1 << TPD_LSO_V2_SHIFT)) {
		if (++producer == txq->count)
			producer = 0;
		tpd = txq->tpd_hdr + producer;
		buf = txq->bf_info + producer;
		tpd->word0 = first_tpd->word0;
		tpd->word1 = first_tpd->word1;
	}
	maplen = skb_headlen(skb);
	dma = dma_map_single(txq->dev, skb->data, maplen, DMA_TO_DEVICE);
	if (dma_mapping_error(txq->dev, dma))
		goto err_dma;

	dma_unmap_len_set(buf, size, maplen);
	dma_unmap_addr_set(buf, dma, dma);

	tpd->adrl.addr = cpu_to_le64(dma);
	tpd->word0 |= FIELDX(TPD_BUFLEN, maplen);

	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		struct skb_frag_struct *frag;

		frag = &skb_shinfo(skb)->frags[f];
		if (++producer == txq->count)
			producer = 0;
		tpd = txq->tpd_hdr + producer;
		buf = txq->bf_info + producer;
		tpd->word0 = first_tpd->word0;
		tpd->word1 = first_tpd->word1;
		maplen = skb_frag_size(frag);
		dma = skb_frag_dma_map(txq->dev, frag, 0,
				       maplen, DMA_TO_DEVICE);
		if (dma_mapping_error(txq->dev, dma))
			goto err_dma;
		dma_unmap_len_set(buf, size, maplen);
		dma_unmap_addr_set(buf, dma, dma);

		tpd->adrl.addr = cpu_to_le64(dma);
		tpd->word0 |= FIELDX(TPD_BUFLEN, maplen);
	}
	/* last TPD */
	tpd->word1 |= 1 << TPD_EOP_SHIFT;

	if (++producer == txq->count)
		producer = 0;

	first_buf->flags |= ALX_BUF_TX_FIRSTFRAG;
	buf->skb = skb;
	txq->pidx = producer;

	return 0;

err_dma:

	for (f = txq->pidx; f != producer;) {
		alx_txbuf_unmap_and_free(txq, f);
		if (++f == txq->count)
			f = 0;
	}
	return -1;
}

static netdev_tx_t alx_start_xmit_ring(struct alx_tx_queue *txq,
				       struct sk_buff *skb)
{
	struct alx_adapter *adpt;
	struct netdev_queue *netque;
	struct tpd_desc *first;
	int budget, tpdreq;

	adpt = netdev_priv(txq->netdev);
	netque = netdev_get_tx_queue(txq->netdev, skb_get_queue_mapping(skb));

	tpdreq = alx_tpd_req(skb);
	budget = alx_tpd_avail(txq);

	if (unlikely(budget < tpdreq)) {
		if (!netif_tx_queue_stopped(netque)) {
			netif_tx_stop_queue(netque);

			/* TX reclaim might have plenty of free TPD
			 * but see tx_queue is active (because its
			 * judement doesn't acquire tx-spin-lock,
			 * this situation cause the TX-queue stop and
			 * never be wakeup.
			 * try one more time
			 */
			budget = alx_tpd_avail(txq);
			if (budget >= tpdreq) {
				netif_tx_wake_queue(netque);
				goto tx_conti;
			}
			netdev_err(adpt->netdev,
				   "TPD Ring is full when queue awake!\n");
		}
		return NETDEV_TX_BUSY;
	}

tx_conti:

	first = txq->tpd_hdr + txq->pidx;
	memset(first, 0, sizeof(struct tpd_desc));

	/* NOTE, chip only supports single-VLAN insertion (81-00-TAG) */
	if (vlan_tx_tag_present(skb)) {
		first->word1 |= 1 << TPD_INS_VLTAG_SHIFT;
		first->word0 |= FIELDX(TPD_VLTAG, htons(vlan_tx_tag_get(skb)));
	}
	if (skb->protocol == htons(ETH_P_8021Q))
		first->word1 |= 1 << TPD_VLTAGGED_SHIFT;
	if (skb_network_offset(skb) != ETH_HLEN)
		first->word1 |= 1 << TPD_ETHTYPE_SHIFT;
	if (alx_tx_csum(skb, first) < 0)
		goto drop;
	if (alx_tso(skb, first) < 0)
		goto drop;
	if (alx_tx_map(txq, skb) < 0)
		goto drop;

	/* refresh produce idx on HW */
	wmb();
	ALX_MEM_W16(adpt, txq->p_reg, txq->pidx);

	netdev_info(adpt->netdev,
		    "TX[Preg:%X]: producer = 0x%x, consumer = 0x%x\n",
		    txq->p_reg, txq->pidx, atomic_read(&txq->cidx));

	return NETDEV_TX_OK;

drop:
	memset(first, 0, sizeof(struct tpd_desc));
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static netdev_tx_t alx_start_xmit(struct sk_buff *skb,
				  struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	if (ALX_FLAG(adpt, HALT)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (skb->len <= 0) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	return alx_start_xmit_ring(alx_tx_queue_mapping(adpt, skb), skb);
}

static void alx_tx_timeout(struct net_device *dev)
{
	struct alx_adapter *adpt = netdev_priv(dev);

	netdev_err(adpt->netdev, "tx timeout, reset\n");
	// ???? alx_dump_state(adpt);

	ALX_FLAG_SET(adpt, TASK_RESET);
	alx_schedule_work(adpt);
}

static int alx_mdio_read(struct net_device *netdev,
			 int prtad, int devad, u16 addr)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	u16 val;
	int err;

	if (prtad != adpt->mdio.prtad)
		return -EINVAL;

	if (devad != MDIO_DEVAD_NONE)
		err = alx_read_phy_ext(adpt, devad, addr, &val);
	else
		err = alx_read_phy_reg(adpt, addr, &val);

	return err ? -EIO : val;
}

static int alx_mdio_write(struct net_device *netdev,
			  int prtad, int devad, u16 addr, u16 val)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	int err;

	if (prtad != adpt->mdio.prtad)
		return -EINVAL;

	if (devad != MDIO_DEVAD_NONE)
		err = alx_write_phy_ext(adpt, devad, addr, val);
	else
		err = alx_write_phy_reg(adpt, addr, val);

	return err ? -EIO : 0;
}

static int alx_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	if (!netif_running(netdev))
		return -EAGAIN;

	return mdio_mii_ioctl(&adpt->mdio, if_mii(ifr), cmd);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void alx_poll_controller(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	int i;

	if (ALX_FLAG(adpt, HALT))
		return;

	if (ALX_FLAG(adpt, USING_MSIX)) {
		alx_intr_msix_misc(0, adpt);
		for (i = 0; i < adpt->nr_napi; i++)
			alx_msix_ring(0, adpt->qnapi[i]);
	} else if (ALX_FLAG(adpt, USING_MSI))
		alx_intr_msi(0, adpt);
	else
		alx_intr_legacy(0, adpt);

}
#endif

static const struct net_device_ops alx_netdev_ops = {
	.ndo_open               = alx_open,
	.ndo_stop               = alx_stop,
	.ndo_start_xmit         = alx_start_xmit,
	.ndo_get_stats          = alx_get_stats,
	.ndo_set_rx_mode        = alx_set_rx_mode,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_set_mac_address    = alx_set_mac_address,
	.ndo_change_mtu         = alx_change_mtu,
	.ndo_do_ioctl           = alx_ioctl,
	.ndo_tx_timeout         = alx_tx_timeout,
	.ndo_fix_features	= alx_fix_features,
	.ndo_set_features	= alx_set_features,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller    = alx_poll_controller,
#endif
};

/* alx_probe - Device Initialization Routine */
static int __devinit
alx_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct alx_adapter *adpt = NULL;
	bool phy_cfged;
	int bars, pm_cap, err;
	static int cards_found;

	/* enable device (incl. PCI PM wakeup and hotplug setup) */
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot enable PCI device memory\n");
		return err;
	}

	/* The alx chip can DMA to 64-bit addresses, but it uses a single
	 * shared register for the high 32 bits, so only a single, aligned,
	 * 4 GB physical address range can be used at a time.
	 */
	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)) &&
	    !dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64))) {
		dev_info(&pdev->dev, "DMA to 64-BIT addresses\n");
	} else {
		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(&pdev->dev,
						    DMA_BIT_MASK(32));
			if (err) {
				dev_err(&pdev->dev,
					"No usable DMA config, aborting\n");
				goto err_dma_mask;
			}
		}
	}

	/* obtain PCI resources */
	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	err = pci_request_selected_regions(pdev, bars, alx_drv_name);
	if (err) {
		dev_err(&pdev->dev,
			"pci_request_selected_regions failed(bars:%d)\n", bars);
		goto err_pci_region;
	}

	pci_enable_pcie_error_reporting(pdev);
	pci_set_master(pdev);

	/* find PM capability */
	pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
	if (pm_cap == 0) {
		dev_err(&pdev->dev,
			"Can't find power management capability, aborting\n");
		err = -EIO;
		goto err_pm;
	}
	err = pci_set_power_state(pdev, PCI_D0);
	if (err) {
		dev_err(&pdev->dev, "switch to D0 status failed, aborting\n");
		goto err_pm;
	}

	/* netdev zeroed in init_etherdev */
	netdev = alloc_etherdev_mqs(sizeof(struct alx_adapter),
				    ALX_MAX_TX_QUEUES,
				    ALX_MAX_RX_QUEUES);
	if (!netdev) {
		dev_err(&pdev->dev, "etherdev_mq alloc failed\n");
		err = -ENOMEM;
		goto err_alloc_ethdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, adpt);
	adpt = netdev_priv(netdev);
	adpt->netdev = netdev;
	adpt->pdev = pdev;
	adpt->msg_enable = 0;
	adpt->bd_number = cards_found;

	adpt->hw_addr = pci_ioremap_bar(pdev, 0);
	if (!adpt->hw_addr) {
		dev_err(&pdev->dev, "cannot map device registers\n");
		err = -EIO;
		goto err_iomap;
	}

	netdev->netdev_ops = &alx_netdev_ops;
	alx_set_ethtool_ops(netdev);
	netdev->irq  = pdev->irq;
	netdev->watchdog_timeo = ALX_WATCHDOG_TIME;

	/* init alx_adapte structure */
	err = alx_init_sw(adpt);
	if (err) {
		dev_err(&pdev->dev, "net device private data init failed\n");
		goto err_init_sw;
	}

	/* reset pcie */
	alx_reset_pcie(adpt);

	/* check if phy already configed by ohter driver */
	phy_cfged = alx_phy_configed(adpt);

	/* reset PHY to a known stable status */
	if (!phy_cfged)
		alx_reset_phy(adpt, !adpt->hib_patch);

	/* reset mac/dma controller */
	err = alx_reset_mac(adpt);
	if (err) {
		dev_err(&pdev->dev, "MAC Reset failed, error = %d\n", err);
		err = -EIO;
		goto err_rst_mac;
	}

	/* setup link to put it in a known good starting state */
	if (!phy_cfged) {
		err = alx_setup_speed_duplex(adpt,
			adpt->adv_cfg, adpt->flowctrl);
		if (err) {
			dev_err(&pdev->dev,
				"config PHY speed/duplex failed,err=%d\n",
				err);
			err = -EIO;
			goto err_setup_link;
		}
	}

	netdev->hw_features = NETIF_F_SG	 |
			      NETIF_F_HW_CSUM	 |
			      NETIF_F_HW_VLAN_RX |
			      NETIF_F_TSO        |
			      NETIF_F_TSO6;
	netdev->features = netdev->hw_features | NETIF_F_HW_VLAN_TX;

	/* read permanent mac addr from register or eFuse */
	if (alx_get_perm_macaddr(adpt, adpt->perm_addr)) {
		dev_warn(&pdev->dev, "invalid perm-address, use random one\n");
		eth_hw_addr_random(netdev);
		memcpy(adpt->perm_addr, netdev->dev_addr, netdev->addr_len);
	}
	/* using permanent address as current address */
	memcpy(adpt->mac_addr, adpt->perm_addr, ETH_ALEN);
	memcpy(netdev->dev_addr, adpt->mac_addr, ETH_ALEN);
	memcpy(netdev->perm_addr, adpt->perm_addr, ETH_ALEN);

	/* PHY mdio */
	adpt->mdio.prtad = MDIO_PRTAD_NONE;
	adpt->mdio.mmds = 0;
	adpt->mdio.dev = netdev;
	adpt->mdio.mode_support =
		MDIO_SUPPORTS_C45 | MDIO_SUPPORTS_C22 | MDIO_EMULATE_C22;
	adpt->mdio.mdio_read = alx_mdio_read;
	adpt->mdio.mdio_write = alx_mdio_write;
	if (!alx_get_phy_id(adpt)) {
		dev_err(&pdev->dev, "identify PHY failed\n");
		err = -EIO;
		goto err_id_phy;
	}

	INIT_WORK(&adpt->task, alx_task);

	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "register netdevice failed\n");
		goto err_register_netdev;
	}

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

	device_set_wakeup_enable(&pdev->dev, adpt->sleep_ctrl);
	cards_found++;

	dev_info(&pdev->dev,
		 "alx(%pM): Qualcomm Atheros Ethernet Network Connection\n",
		 netdev->dev_addr);

	return 0;

err_id_phy:
err_register_netdev:
err_setup_link:
err_rst_mac:
err_init_sw:
	iounmap(adpt->hw_addr);
err_iomap:
	free_netdev(netdev);
err_alloc_ethdev:
err_pm:
	pci_release_selected_regions(pdev, bars);
err_pci_region:
err_dma_mask:
	pci_disable_device(pdev);
	dev_err(&pdev->dev, "error when probe device, error = %d\n", err);
	return err;
}

/* alx_remove - Device Removal Routine */
static void __devexit alx_remove(struct pci_dev *pdev)
{
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	struct net_device *netdev = adpt->netdev;

	ALX_FLAG_SET(adpt, HALT);
	alx_cancel_work(adpt);

	/* restore permanent mac address */
	alx_set_macaddr(adpt, adpt->perm_addr);

	unregister_netdev(netdev);
	iounmap(adpt->hw_addr);
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));

	dev_info(&pdev->dev, "dev removed\n");
	free_netdev(netdev);
	pci_disable_pcie_error_reporting(pdev);
	pci_disable_device(pdev);
}

/* alx_pci_error_detected */
static pci_ers_result_t alx_pci_error_detected(struct pci_dev *pdev,
					       pci_channel_state_t state)
{
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	struct net_device *netdev = adpt->netdev;
	pci_ers_result_t rc = PCI_ERS_RESULT_NEED_RESET;

	rtnl_lock();

	if (netif_running(netdev)) {
		netif_device_detach(netdev);
		alx_halt(adpt);
	}
	if (state == pci_channel_io_perm_failure)
		rc = PCI_ERS_RESULT_DISCONNECT;
	else
		pci_disable_device(pdev);

	rtnl_unlock();

	return rc;
}

/* alx_pci_error_slot_reset */
static pci_ers_result_t alx_pci_error_slot_reset(struct pci_dev *pdev)
{
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	pci_ers_result_t rc = PCI_ERS_RESULT_DISCONNECT;

	rtnl_lock();

	if (pci_enable_device(pdev)) {
		dev_err(&pdev->dev, "Re-enable PCI device after reset fail\n");
		goto out;
	}

	pci_set_master(pdev);
	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	alx_reset_pcie(adpt);
	if (!alx_reset_mac(adpt))
		rc = PCI_ERS_RESULT_RECOVERED;
out:
	pci_cleanup_aer_uncorrect_error_status(pdev);

	rtnl_unlock();

	return rc;
}

/* alx_pci_error_resume */
static void alx_pci_error_resume(struct pci_dev *pdev)
{
	struct alx_adapter *adpt = pci_get_drvdata(pdev);
	struct net_device *netdev = adpt->netdev;

	rtnl_lock();

	if (netif_running(netdev)) {
		alx_activate(adpt);
		netif_device_attach(netdev);
	}

	rtnl_unlock();
}


static struct pci_error_handlers alx_err_handler = {
	.error_detected = alx_pci_error_detected,
	.slot_reset     = alx_pci_error_slot_reset,
	.resume         = alx_pci_error_resume,
};

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(alx_pm_ops, alx_suspend, alx_resume);
#define ALX_PM_OPS      (&alx_pm_ops)
#else
#define ALX_PM_OPS      NULL
#endif

static struct pci_driver alx_driver = {
	.name        = alx_drv_name,
	.id_table    = alx_pci_tbl,
	.probe       = alx_probe,
	.remove      = __devexit_p(alx_remove),
	.shutdown    = alx_shutdown,
	.err_handler = &alx_err_handler,
	.driver.pm   = ALX_PM_OPS,
};


static int __init alx_init_module(void)
{
	pr_info("%s\n", alx_drv_desc);
	return pci_register_driver(&alx_driver);
}
module_init(alx_init_module);


static void __exit alx_exit_module(void)
{
	pci_unregister_driver(&alx_driver);
}
module_exit(alx_exit_module);
