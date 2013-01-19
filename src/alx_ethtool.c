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

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mdio.h>

#include "alx_reg.h"
#include "alx_hw.h"
#include "alx.h"

static int alx_get_settings(struct net_device *netdev,
			    struct ethtool_cmd *ecmd)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;

	ecmd->supported = (SUPPORTED_10baseT_Half  |
			   SUPPORTED_10baseT_Full  |
			   SUPPORTED_100baseT_Half |
			   SUPPORTED_100baseT_Full |
			   SUPPORTED_Autoneg       |
			   SUPPORTED_TP);
	if (ALX_CAP(hw, GIGA))
		ecmd->supported |= SUPPORTED_1000baseT_Full;

	ecmd->advertising = ADVERTISED_TP;
	if (hw->adv_cfg & ADVERTISED_Autoneg)
		ecmd->advertising |= hw->adv_cfg;

	ecmd->port = PORT_TP;
	ecmd->phy_address = 0;
	ecmd->autoneg = (hw->adv_cfg & ADVERTISED_Autoneg) ?
		AUTONEG_ENABLE : AUTONEG_DISABLE;
	ecmd->transceiver = XCVR_INTERNAL;

	if (hw->link_up) {
		ethtool_cmd_speed_set(ecmd, hw->link_speed);
		ecmd->duplex = hw->link_duplex;
	} else {
		ethtool_cmd_speed_set(ecmd, -1);
		ecmd->duplex = -1;
	}

	return 0;
}

static int alx_set_settings(struct net_device *netdev,
			    struct ethtool_cmd *ecmd)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;
	u32 adv_cfg;
	int err = 0;

	while (test_and_set_bit(ALX_FLAG_RESETING, &adpt->flags))
		msleep(20);

	if (ecmd->autoneg == AUTONEG_ENABLE) {
		if (ecmd->advertising & ADVERTISED_1000baseT_Half) {
			dev_warn(&adpt->pdev->dev, "1000M half is invalid\n");
			ALX_FLAG_CLEAR(adpt, RESETING);
			return -EINVAL;
		}
		adv_cfg = ecmd->advertising | ADVERTISED_Autoneg;
	} else {
		int speed = ethtool_cmd_speed(ecmd);

		switch (speed + ecmd->duplex) {
		case SPEED_10 + DUPLEX_HALF:
			adv_cfg = ADVERTISED_10baseT_Half;
			break;
		case SPEED_10 + DUPLEX_FULL:
			adv_cfg = ADVERTISED_10baseT_Full;
			break;
		case SPEED_100 + DUPLEX_HALF:
			adv_cfg = ADVERTISED_100baseT_Half;
			break;
		case SPEED_100 + DUPLEX_FULL:
			adv_cfg = ADVERTISED_100baseT_Full;
			break;
		default:
			err = -EINVAL;
			break;
		}
	}

	if (!err) {
		hw->adv_cfg = adv_cfg;
		err = alx_setup_speed_duplex(hw, adv_cfg, hw->flowctrl);
		if (err) {
			dev_warn(&adpt->pdev->dev,
				 "config PHY speed/duplex failed,err=%d\n",
				 err);
			err = -EIO;
		}
	}

	ALX_FLAG_CLEAR(adpt, RESETING);

	return err;
}

static void alx_get_pauseparam(struct net_device *netdev,
			       struct ethtool_pauseparam *pause)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;

	if (hw->flowctrl & ALX_FC_ANEG &&
	    hw->adv_cfg & ADVERTISED_Autoneg)
		pause->autoneg = AUTONEG_ENABLE;
	else
		pause->autoneg = AUTONEG_DISABLE;

	if (hw->flowctrl & ALX_FC_TX)
		pause->tx_pause = 1;
	else
		pause->tx_pause = 0;

	if (hw->flowctrl & ALX_FC_RX)
		pause->rx_pause = 1;
	else
		pause->rx_pause = 0;
}


static int alx_set_pauseparam(struct net_device *netdev,
			      struct ethtool_pauseparam *pause)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;
	int err = 0;
	bool reconfig_phy = false;
	u8 fc = 0;

	if (pause->tx_pause)
		fc |= ALX_FC_TX;
	if (pause->rx_pause)
		fc |= ALX_FC_RX;
	if (pause->autoneg)
		fc |= ALX_FC_ANEG;

	while (test_and_set_bit(ALX_FLAG_RESETING, &adpt->flags))
		msleep(20);

	/* restart auto-neg for auto-mode */
	if (hw->adv_cfg & ADVERTISED_Autoneg) {
		if (!((fc ^ hw->flowctrl) & ALX_FC_ANEG))
			reconfig_phy = true;
		if (fc & hw->flowctrl & ALX_FC_ANEG &&
		    (fc ^ hw->flowctrl) & (ALX_FC_RX | ALX_FC_TX))
			reconfig_phy = true;
	}

	if (reconfig_phy) {
		err = alx_setup_speed_duplex(hw, hw->adv_cfg, fc);
		if (err) {
			dev_warn(&adpt->pdev->dev,
				 "config PHY flow control failed,err=%d\n",
				 err);
			err = -EIO;
		}
	}

	/* flow control on mac */
	if ((fc ^ hw->flowctrl) & (ALX_FC_RX | ALX_FC_TX))
		alx_cfg_mac_fc(hw, fc);

	hw->flowctrl = fc;

	ALX_FLAG_CLEAR(adpt, RESETING);

	return err;
}

static u32 alx_get_msglevel(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	return adpt->msg_enable;
}

static void alx_set_msglevel(struct net_device *netdev, u32 data)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	adpt->msg_enable = data;
}

static const u32 hw_regs[] = {
	ALX_DEV_CAP, ALX_DEV_CTRL, ALX_LNK_CAP, ALX_LNK_CTRL,
	ALX_UE_SVRT, ALX_EFLD, ALX_SLD, ALX_PPHY_MISC1,
	ALX_PPHY_MISC2, ALX_PDLL_TRNS1,
	ALX_TLEXTN_STATS, ALX_EFUSE_CTRL, ALX_EFUSE_DATA, ALX_SPI_OP1,
	ALX_SPI_OP2, ALX_SPI_OP3, ALX_EF_CTRL, ALX_EF_ADDR,
	ALX_EF_DATA, ALX_SPI_ID,
	ALX_SPI_CFG_START, ALX_PMCTRL, ALX_LTSSM_CTRL, ALX_MASTER,
	ALX_MANU_TIMER, ALX_IRQ_MODU_TIMER, ALX_PHY_CTRL, ALX_MAC_STS,
	ALX_MDIO, ALX_MDIO_EXTN,
	ALX_PHY_STS, ALX_BIST0, ALX_BIST1, ALX_SERDES,
	ALX_LED_CTRL, ALX_LED_PATN, ALX_LED_PATN2, ALX_SYSALV,
	ALX_PCIERR_INST, ALX_LPI_DECISN_TIMER,
	ALX_LPI_CTRL, ALX_LPI_WAIT, ALX_HRTBT_VLAN, ALX_HRTBT_CTRL,
	ALX_RXPARSE, ALX_MAC_CTRL, ALX_GAP, ALX_STAD1,
	ALX_LED_CTRL, ALX_HASH_TBL0,
	ALX_HASH_TBL1, ALX_HALFD, ALX_DMA, ALX_WOL0,
	ALX_WOL1, ALX_WOL2, ALX_WRR, ALX_HQTPD,
	ALX_CPUMAP1, ALX_CPUMAP2,
	ALX_MISC, ALX_RX_BASE_ADDR_HI, ALX_RFD_ADDR_LO, ALX_RFD_RING_SZ,
	ALX_RFD_BUF_SZ, ALX_RRD_ADDR_LO, ALX_RRD_RING_SZ,
	ALX_RFD_PIDX, ALX_RFD_CIDX, ALX_RXQ0,
	ALX_RXQ1, ALX_RXQ2, ALX_RXQ3, ALX_TX_BASE_ADDR_HI,
	ALX_TPD_PRI0_ADDR_LO, ALX_TPD_PRI1_ADDR_LO,
	ALX_TPD_PRI2_ADDR_LO, ALX_TPD_PRI3_ADDR_LO,
	ALX_TPD_PRI0_PIDX, ALX_TPD_PRI1_PIDX,
	ALX_TPD_PRI2_PIDX, ALX_TPD_PRI3_PIDX, ALX_TPD_PRI0_CIDX,
	ALX_TPD_PRI1_CIDX, ALX_TPD_PRI2_CIDX, ALX_TPD_PRI3_CIDX,
	ALX_TPD_RING_SZ, ALX_TXQ0, ALX_TXQ1, ALX_TXQ2,
	ALX_MSI_MAP_TBL1, ALX_MSI_MAP_TBL2, ALX_MSI_ID_MAP,
	ALX_MSIX_MASK, ALX_MSIX_PENDING,
	ALX_RSS_HASH_VAL, ALX_RSS_HASH_FLAG, ALX_RSS_BASE_CPU_NUM,
};

static int alx_get_regs_len(struct net_device *netdev)
{
	return (ARRAY_SIZE(hw_regs) + 0x20) * 4;
}

static void alx_get_regs(struct net_device *netdev,
			 struct ethtool_regs *regs, void *buff)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;
	u32 *p = buff;
	int i;

	regs->version = (ALX_DID(hw) << 16) | (ALX_REVID(hw) << 8) | 1;

	memset(buff, 0, (ARRAY_SIZE(hw_regs) + 0x20) * 4);

	for (i = 0; i < ARRAY_SIZE(hw_regs); i++, p++)
		ALX_MEM_R32(hw, hw_regs[i], p);

	/* last 0x20 for PHY register */
	for (i = 0; i < 0x20; i++) {
		alx_read_phy_reg(hw, i, (u16 *)p);
		p++;
	}
}

static void alx_get_wol(struct net_device *netdev,
			struct ethtool_wolinfo *wol)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;

	wol->supported = WAKE_MAGIC | WAKE_PHY;
	wol->wolopts = 0;

	if (hw->sleep_ctrl & ALX_SLEEP_WOL_MAGIC)
		wol->wolopts |= WAKE_MAGIC;
	if (hw->sleep_ctrl & ALX_SLEEP_WOL_PHY)
		wol->wolopts |= WAKE_PHY;

	netif_info(adpt, wol, adpt->netdev,
		   "wolopts = %x\n",
		   wol->wolopts);
}

static int alx_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;

	if (wol->wolopts & (WAKE_ARP | WAKE_MAGICSECURE |
			    WAKE_UCAST | WAKE_BCAST | WAKE_MCAST))
		return -EOPNOTSUPP;

	hw->sleep_ctrl = 0;

	if (wol->wolopts & WAKE_MAGIC)
		hw->sleep_ctrl |= ALX_SLEEP_WOL_MAGIC;
	if (wol->wolopts & WAKE_PHY)
		hw->sleep_ctrl |= ALX_SLEEP_WOL_PHY;

	netdev_info(adpt->netdev, "wol-ctrl=%X\n", hw->sleep_ctrl);

	device_set_wakeup_enable(&adpt->pdev->dev, hw->sleep_ctrl);

	return 0;
}


static int alx_nway_reset(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	if (netif_running(netdev))
		alx_reinit(adpt);

	return 0;
}

static const char alx_gstrings_test[][ETH_GSTRING_LEN] = {
	"register test  (offline)",
	"memory test    (offline)",
	"interrupt test (offline)",
	"loopback test  (offline)",
	"link test      (offline)"
};
#define ALX_TEST_LEN (sizeof(alx_gstrings_test) / ETH_GSTRING_LEN)

/* private flags */
#define ALX_ETH_PF_LNK_10MH	BIT(0)
#define ALX_ETH_PF_LNK_10MF	BIT(1)
#define ALX_ETH_PF_LNK_100MH	BIT(2)
#define ALX_ETH_PF_LNK_100MF	BIT(3)
#define ALX_ETH_PF_LNK_1000MF	BIT(4)
#define ALX_ETH_PF_LNK_MASK	(\
	ALX_ETH_PF_LNK_10MH |\
	ALX_ETH_PF_LNK_10MF |\
	ALX_ETH_PF_LNK_100MH |\
	ALX_ETH_PF_LNK_100MF |\
	ALX_ETH_PF_LNK_1000MF)

static const char alx_gstrings_stats[][ETH_GSTRING_LEN] = {
	"rx_packets",
	"rx_bcast_packets",
	"rx_mcast_packets",
	"rx_pause_packets",
	"rx_ctrl_packets",
	"rx_fcs_errors",
	"rx_length_errors",
	"rx_bytes",
	"rx_runt_packets",
	"rx_fragments",
	"rx_64B_or_less_packets",
	"rx_65B_to_127B_packets",
	"rx_128B_to_255B_packets",
	"rx_256B_to_511B_packets",
	"rx_512B_to_1023B_packets",
	"rx_1024B_to_1518B_packets",
	"rx_1519B_to_mtu_packets",
	"rx_oversize_packets",
	"rx_rxf_ov_drop_packets",
	"rx_rrd_ov_drop_packets",
	"rx_align_errors",
	"rx_bcast_bytes",
	"rx_mcast_bytes",
	"rx_address_errors",
	"tx_packets",
	"tx_bcast_packets",
	"tx_mcast_packets",
	"tx_pause_packets",
	"tx_exc_defer_packets",
	"tx_ctrl_packets",
	"tx_defer_packets",
	"tx_bytes",
	"tx_64B_or_less_packets",
	"tx_65B_to_127B_packets",
	"tx_128B_to_255B_packets",
	"tx_256B_to_511B_packets",
	"tx_512B_to_1023B_packets",
	"tx_1024B_to_1518B_packets",
	"tx_1519B_to_mtu_packets",
	"tx_single_collision",
	"tx_multiple_collisions",
	"tx_late_collision",
	"tx_abort_collision",
	"tx_underrun",
	"tx_trd_eop",
	"tx_length_errors",
	"tx_trunc_packets",
	"tx_bcast_bytes",
	"tx_mcast_bytes",
	"tx_update",
};

#define ALX_STATS_LEN (sizeof(alx_gstrings_stats) / ETH_GSTRING_LEN)

static void alx_get_strings(struct net_device *netdev, u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_TEST:
		memcpy(buf, &alx_gstrings_test, sizeof(alx_gstrings_test));
		break;
	case ETH_SS_STATS:
		memcpy(buf, &alx_gstrings_stats, sizeof(alx_gstrings_stats));
		break;
	}
}

static int alx_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ALX_STATS_LEN;
	case ETH_SS_TEST:
		return ALX_TEST_LEN;
	default:
		return -ENOTSUPP;
	}
}

struct alx_reg_attr {
	u16 reg;
	u32 ro_mask;
	u32 rw_mask;
	u32 rc_mask;
	u32 rst_val;
	u8  rst_affect;
};

struct alx_reg_attr ar816x_regs_a[] = {
	{0x1400, 0xffff80E0,	0x4D00,		0x0,        0x40020000, 0},
	{0x1404, 0x0,		0xffffffff,     0x0,        0x0,        1},
	{0x1408, 0x0,		0xffffffff,     0x0,        0x0,        1},
	{0x140c, 0xFFFF0000,	0x0,            0x0,        0xffff3800, 0},
	{0x1410, 0xffffffff,	0x0,            0x0,        0x0000,     0},
	{0x1414, 0x0,		0x0,            0x0,        0x0,        1},
	{0x141C, 0xfffffffe,	0x0,            0x0,        0x0,        1},
	{0x1420, 0xfffffffe,	0x0,            0x0,        0x0,        1},
	{0x1484, 0x0,		0x7f7f7f7f,     0x0,        0x60405060, 1},
	{0x1490, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1494, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1498, 0x0,		0xffff3ff,      0x0,        0x07a1f037, 1},
	{0x149C, 0xffff0000,	0xffff,         0x0,        0x600,      1},
	{0x14a0, 0x808078c0,	0x7f803f,       0x7f000700, 0x0,        1},
	{0x14a4, 0x0,		0xFFFFFFFF,     0x0,        0x0,        1},
	{0x14a8, 0xFF000000,	0x00FFFFFF,     0x0,        0x0,        1},
	{0x1540, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1544, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1550, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1560, 0xFFFFF000,	0xfff,          0x0,        0x0,        0},
	{0x1564, 0xFFFF0000,	0xffff,         0x0,        0x0,        0},
	{0x1568, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1578, 0xFFFFF000,	0xfff,          0x0,        0x0,        0},
	{0x157C, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1580, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1584, 0xFFFF0000,	0xffff,         0x0,        0x0,        0},
	{0x1588, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1590, 0xFF00,	0xFFFF00DF,     0x0,        0x01000045, 1},
	{0x1594, 0xFFFFF800,	0x7FF,          0x0,        191,        1},
	{0x15A0, 0x200E0040,	0x5FF1FFBF,     0x0,        0x40810083, 1},
	{0x15A4, 0xFFFFF000,	0xFFF,          0x0,        0x1210,     1},
	{0x15A8, 0xF000F000,	0x0FFF0FFF,     0x0,        0x02E003C0, 1},
	{0x15AC, 0xF000,	0xFFFF0FFF,     0x0,        0x0100,     1},
	{0x15C4, 0xFF000000,	0xFFFFFF,       0x0,        0x0,        1},
	{0x15C8, 0xFFFF0000,	0xFFFF,         0x0,        0x0100,     1},
	{0x15E0, 0xFFFFF000,	0xFFF,          0x0,        0x0,        1},
	{0x15F0, 0x0,		0xFFFFFFFF,     0x0,        0x0,        1},
	{0x15F4, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x15F8, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x15FC, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x1700, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1704, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1708, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x170c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1710, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1714, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1718, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x171c, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1720, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1724, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1728, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x172c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1730, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1734, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1738, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x173c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1740, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1744, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1748, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x174c, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x1750, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1754, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1758, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x175c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1760, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1764, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1768, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x176c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1770, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x1774, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1778, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x177c, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1780, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1784, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1788, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x178c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1790, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1794, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1798, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x179c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a0, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a4, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a8, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17ac, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b0, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b4, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b8, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17bc, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x17c0, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0xffff, 0, 0, 0, 0, 0},
};

struct alx_reg_attr ar816x_regs_b[] = {
	{0x1400, 0xffff80E0,	0x4D00,		0x0,        0x40020000, 0},
	{0x1404, 0x0,		0xffffffff,     0x0,        0x0,        1},
	{0x1408, 0x0,		0xffffffff,     0x0,        0x0,        1},
	{0x140c, 0xFFFF0000,	0x0,            0x0,        0xffff3800, 0},
	{0x1410, 0xffffffff,	0x0,            0x0,        0x0000,     0},
	{0x1414, 0x0,		0x0,            0x0,        0x0,        1},
	{0x141C, 0xfffffffe,	0x0,            0x0,        0x0,        1},
	{0x1420, 0xfffffffe,	0x0,            0x0,        0x0,        1},
	{0x1484, 0x0,		0x7f7f7f7f,     0x0,        0x60405018, 1},
	{0x1490, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1494, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1498, 0x0,		0xffff3ff,      0x0,        0x07a1f037, 1},
	{0x149C, 0xffff0000,	0xffff,         0x0,        0x600,      1},
	{0x14a0, 0x808078c0,	0x7f803f,       0x7f000700, 0x0,        1},
	{0x14a4, 0x0,		0xFFFFFFFF,     0x0,        0x0,        1},
	{0x14a8, 0xFF000000,	0x00FFFFFF,     0x0,        0x0,        1},
	{0x1540, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1544, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1550, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1560, 0xFFFFF000,	0xfff,          0x0,        0x0,        0},
	{0x1564, 0xFFFF0000,	0xffff,         0x0,        0x0,        0},
	{0x1568, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1578, 0xFFFFF000,	0xfff,          0x0,        0x0,        0},
	{0x157C, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1580, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1584, 0xFFFF0000,	0xffff,         0x0,        0x0,        0},
	{0x1588, 0x0,		0xffffffff,     0x0,        0x0,        0},
	{0x1590, 0xFF00,	0xFFFF00DF,     0x0,        0x01000045, 1},
	{0x1594, 0xFFFFF800,	0x7FF,          0x0,        191,        1},
	{0x15A0, 0x200E0040,	0x5FF1FFBF,     0x0,        0x40810083, 1},
	{0x15A4, 0xFFFFF000,	0xFFF,          0x0,        0x1210,     1},
	{0x15A8, 0xF000F000,	0x0FFF0FFF,     0x0,        0x02E003C0, 1},
	{0x15AC, 0xF000,	0xFFFF0FFF,     0x0,        0x0100,     1},
	{0x15C4, 0xFF000000,	0xFFFFFF,       0x0,        0x0,        1},
	{0x15C8, 0xFFFF0000,	0xFFFF,         0x0,        0x0100,     1},
	{0x15E0, 0xFFFFF000,	0xFFF,          0x0,        0x0,        1},
	{0x15F0, 0x0,		0xFFFFFFFF,     0x0,        0x0,        1},
	{0x15F4, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x15F8, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x15FC, 0xFFFFFFFF,	0x0,            0x0,        0x0,        1},
	{0x1700, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1704, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1708, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x170c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1710, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1714, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1718, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x171c, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1720, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1724, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1728, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x172c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1730, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1734, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1738, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x173c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1740, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1744, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1748, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x174c, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x1750, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1754, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1758, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x175c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1760, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1764, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1768, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x176c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1770, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x1774, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1778, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x177c, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x1780, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1784, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1788, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x178c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1790, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1794, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x1798, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x179c, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a0, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a4, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17a8, 0xffffffff,	0x0,            0xffffff,   0x0,        1},
	{0x17ac, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b0, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b4, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17b8, 0xffffffff,	0x0,            0xffff,     0x0,        1},
	{0x17bc, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0x17c0, 0xffffffff,	0x0,            0xffffffff, 0x0,        1},
	{0xffff, 0, 0, 0, 0, 0},
};

static int alx_diag_register(struct alx_adapter *adpt, u64 *data)
{
	struct alx_hw *hw = &adpt->hw;
	struct alx_reg_attr *preg, *oreg;
	u32 val, old;

	switch (ALX_DID(hw)) {
	case ALX_DEV_ID_AR8161:
	case ALX_DEV_ID_AR8162:
	case ALX_DEV_ID_AR8171:
	case ALX_DEV_ID_AR8172:
		oreg = ALX_REV_B0 == ALX_REVID(hw) ?
			ar816x_regs_b : ar816x_regs_a;
		break;
	default:
		/* unknow type */
		*data = 1;
		return -EIO;
	}

	/* issue a MAC-reset */
	ALX_MEM_W32(hw, ALX_MASTER, ALX_MASTER_DMA_MAC_RST);
	msleep(50);

	/* check reset value */
	preg = oreg;
	while (preg->reg != 0xffff) {
		if (preg->rst_affect) {
			ALX_MEM_R32(hw, preg->reg, &val);
			if (val != preg->rst_val) {
				netif_err(adpt, hw, adpt->netdev,
					  "register %X, hard-rst:%X, read-val:%X\n",
					  preg->reg, preg->rst_val, val);
				*data = 2;
				return -EIO;
			}
		}
		preg++;
	}

	/* check read-clear/read-write attribute */
	preg = oreg;

	while (preg->reg != 0xffff) {
		ALX_MEM_R32(hw, preg->reg, &old);

		/* read clear */
		if (preg->rc_mask) {
			u32 v2;

			msleep(20);
			ALX_MEM_R32(hw, preg->reg, &v2);
			if ((v2 & preg->rc_mask) != 0) {
				netif_err(adpt, hw, adpt->netdev,
					  "register %X, RC-mask:%X, Old:%X, New:%X\n",
					  preg->reg, preg->rc_mask, old, v2);
				*data = 3;
				return -EIO;
			}
		}

		/* read/write */
		ALX_MEM_W32(hw, preg->reg, 0xffffffff & preg->rw_mask);
		ALX_MEM_FLUSH(hw);
		ALX_MEM_R32(hw, preg->reg, &val);
		if ((val & preg->rw_mask) != preg->rw_mask) {
			netif_err(adpt, hw, adpt->netdev,
				  "register %X, RW-mask:%X, val-1:%X\n",
					  preg->reg, preg->rw_mask, val);
			*data = 4;
			return -EIO;
		}
		ALX_MEM_W32(hw, preg->reg, 0);
		ALX_MEM_FLUSH(hw);
		ALX_MEM_R32(hw, preg->reg, &val);
		if ((val & preg->rw_mask) != 0) {
			netif_err(adpt, hw, adpt->netdev,
				  "register %X, RW-mask:%X, val-0:%X\n",
					  preg->reg, preg->rw_mask, val);
			*data = 4;
			return -EIO;
		}

		/* restore */
		ALX_MEM_W32(hw, preg->reg, old);

		preg++;
	}

	return 0;
}

static int alx_diag_sram(struct alx_adapter *adpt, u64 *data)
{
	struct alx_hw *hw = &adpt->hw;
	u32 ret[2];
	int i, err;

	err = alx_reset_mac(hw);
	if (err) {
		netif_err(adpt, hw, adpt->netdev, "reset_mac fail %d\n", err);
		*data = 1;
		goto out;
	}
	/* issue bist command */
	ALX_MEM_W32(hw, ALX_BIST0, ALX_BIST0_START);
	ALX_MEM_W32(hw, ALX_BIST1, ALX_BIST1_START);

	/* wait for 100ms */
	ret[1] = ret[0] = 0;
	for (i = 0; i < 5; i++) {
		msleep(20);
		ALX_MEM_R32(hw, ALX_BIST0, &ret[0]);
		ALX_MEM_R32(hw, ALX_BIST1, &ret[1]);
		if (ret[0] & ALX_BIST0_START  || ret[1] & ALX_BIST1_START)
			continue;
		else
			break;
	}

	for (i = 0; i < 2; i++) {
		if (ret[i] & ALX_BIST0_START) {
			netif_err(adpt, hw, adpt->netdev,
				  "sram(%d) bist not complete(%X)!\n",
				  i, ret[i]);
			*data = 2;
			err = -EIO;
			goto out;
		}
		if (ret[i] & ALX_BIST0_FAIL) {
			netif_err(adpt, hw, adpt->netdev,
				  "sram(%d) bist fail(%X)!\n",
				  i, ret[i]);
			*data = 3;
			err = -EIO;
			goto out;
		}
	}
out:
	return err;
}

static int alx_diag_reset(struct alx_adapter *adpt)
{
	struct alx_hw *hw = &adpt->hw;
	int err;

	alx_reset_pcie(hw);
	alx_reset_phy(hw, !hw->hib_patch);
	err = alx_reset_mac(hw);
	if (!err)
		err = alx_setup_speed_duplex(hw, hw->adv_cfg, hw->flowctrl);

	if (err) {
		netif_err(adpt, hw, adpt->netdev, "alx_diag_reset err %X\n",
			  err);
	}

	return err;
}

static int alx_diag_link(struct alx_adapter *adpt, u64 *data)
{
	struct alx_hw *hw = &adpt->hw;
	u32 flags, ethadv;
	u16 speed;
	u8 fc;
	int i, err;

	ethadv = ADVERTISED_Autoneg;
	flags = adpt->eth_pflags & ALX_ETH_PF_LNK_MASK;
	if (flags == 0)
		flags = ALX_ETH_PF_LNK_MASK;
	if (flags & ALX_ETH_PF_LNK_10MH)
		ethadv |= ADVERTISED_10baseT_Half;
	if (flags & ALX_ETH_PF_LNK_10MF)
		ethadv |= ADVERTISED_10baseT_Full;
	if (flags & ALX_ETH_PF_LNK_100MH)
		ethadv |= ADVERTISED_100baseT_Half;
	if (flags & ALX_ETH_PF_LNK_100MF)
		ethadv |= ADVERTISED_100baseT_Full;
	if (flags & ALX_ETH_PF_LNK_1000MF)
		ethadv |= ADVERTISED_1000baseT_Full;

	fc = ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX;

	alx_reset_phy(hw, !hw->hib_patch);
	err = alx_setup_speed_duplex(hw, ethadv, fc);
	if (err) {
		netif_err(adpt, hw, adpt->netdev,
			  "config PHY speed/duplex failed, adv=%X,err=%d\n",
			  ethadv, err);
			  *data = 1;
			goto out;
	}

	/* wait for linkup */
	for (i = 0; i < ALX_MAX_SETUP_LNK_CYCLE; i++) {
		bool link_up;

		msleep(100);
		err = alx_get_phy_link(hw, &link_up, &speed);
		if (err) {
			netif_err(adpt, hw, adpt->netdev,
				  "get PHY speed/duplex failed,err=%d\n",
				  err);
			*data = 2;
			goto out;
		}
		if (link_up)
			break;
	}
	if (i == ALX_MAX_SETUP_LNK_CYCLE) {
		err = ALX_LINK_TIMEOUT;
		netif_err(adpt, hw, adpt->netdev,
			  "get PHY speed/duplex timeout.\n");
		*data = 3;
		goto out;
	}

	netif_info(adpt, hw, adpt->netdev, "link:%s\n", speed_desc(speed));

out:
	return err;
}

static int alx_diag_interrupt(struct alx_adapter *adpt, u64 *data)
{

	return 0;
}


static int alx_diag_loopback(struct alx_adapter *adpt, u64 *data, bool phy_lpbk)
{
	return 0;
}

static void alx_self_test(struct net_device *netdev,
			  struct ethtool_test *etest,
			  u64 *data)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	bool if_running = netif_running(netdev);
	bool phy_lpback = etest->flags & ETH_TEST_FL_EXTERNAL_LB;

	ALX_FLAG_SET(adpt, TESTING);
	memset(data, 0, sizeof(u64) * ALX_TEST_LEN);

	if (if_running)
		dev_close(netdev);

	if (etest->flags == ETH_TEST_FL_OFFLINE) {
		netif_info(adpt, hw, adpt->netdev,  "offline test start...\n");

		if (alx_diag_register(adpt, &data[0]))
			etest->flags |= ETH_TEST_FL_FAILED;

		if (alx_diag_sram(adpt, &data[1]))
			etest->flags |= ETH_TEST_FL_FAILED;

		if (alx_diag_interrupt(adpt, &data[2]))
			etest->flags |= ETH_TEST_FL_FAILED;

		if (phy_lpback)
			etest->flags |= ETH_TEST_FL_EXTERNAL_LB_DONE;
		if (alx_diag_loopback(adpt, &data[3], phy_lpback))
			etest->flags |= ETH_TEST_FL_FAILED;

	} else {
		netif_info(adpt, hw, adpt->netdev,  "online test start...\n");

		if (alx_diag_link(adpt, &data[4]))
			etest->flags |= ETH_TEST_FL_FAILED;
	}

	ALX_FLAG_CLEAR(adpt, TESTING);
	alx_diag_reset(adpt);

	if (if_running)
		dev_open(netdev);
}

static void alx_get_ethtool_stats(struct net_device *netdev,
				  struct ethtool_stats *estats, u64 *data)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;

	spin_lock(&adpt->smb_lock);

	__alx_update_hw_stats(hw);
	memcpy(data, &hw->stats, sizeof(hw->stats));

	spin_unlock(&adpt->smb_lock);
}

static u32 alx_get_priv_flags(struct net_device *netdev)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	return adpt->eth_pflags;
}

static int alx_set_priv_flags(struct net_device *netdev, u32 flags)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	adpt->eth_pflags = flags;

	return 0;
}

static void alx_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *drvinfo)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	strlcpy(drvinfo->driver, alx_drv_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, alx_drv_version, sizeof(drvinfo->version));
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, pci_name(adpt->pdev),
		sizeof(drvinfo->bus_info));
	drvinfo->n_stats = ALX_STATS_LEN;
	drvinfo->testinfo_len = ALX_TEST_LEN;
	drvinfo->n_priv_flags = 5;
	drvinfo->regdump_len = alx_get_regs_len(netdev);
	drvinfo->eedump_len = 0;
}

static const struct ethtool_ops alx_ethtool_ops = {
	.get_settings    = alx_get_settings,
	.set_settings    = alx_set_settings,
	.get_pauseparam  = alx_get_pauseparam,
	.set_pauseparam  = alx_set_pauseparam,
	.get_drvinfo     = alx_get_drvinfo,
	.get_regs_len    = alx_get_regs_len,
	.get_regs        = alx_get_regs,
	.get_wol         = alx_get_wol,
	.set_wol         = alx_set_wol,
	.get_msglevel    = alx_get_msglevel,
	.set_msglevel    = alx_set_msglevel,
	.nway_reset      = alx_nway_reset,
	.get_link        = ethtool_op_get_link,
	.get_strings	 = alx_get_strings,
	.get_sset_count	 = alx_get_sset_count,
	.get_ethtool_stats = alx_get_ethtool_stats,
	.self_test	 = alx_self_test,
	.get_priv_flags	 = alx_get_priv_flags,
	.set_priv_flags	 = alx_set_priv_flags,
};

void __devinit alx_set_ethtool_ops(struct net_device *dev)
{
	SET_ETHTOOL_OPS(dev, &alx_ethtool_ops);
}

