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
	return (ARRAY_SIZE(hw_regs) + 1) * 4;
}

static void alx_get_regs(struct net_device *netdev,
			 struct ethtool_regs *regs, void *buff)
{
	struct alx_adapter *adpt = netdev_priv(netdev);
	struct alx_hw *hw = &adpt->hw;
	u32 *p = buff;
	int i;

	regs->version = (ALX_DID(hw) << 16) | (ALX_REVID(hw) << 8) | 1;

	memset(buff, 0, (ARRAY_SIZE(hw_regs) + 1) * 4);

	for (i = 0; i < ARRAY_SIZE(hw_regs); i++, p++)
		ALX_MEM_R32(hw, hw_regs[i], p);

	/* last one for PHY Link Status */
	alx_read_phy_reg(hw, MII_BMSR, (u16 *)p);
}

static void alx_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *drvinfo)
{
	struct alx_adapter *adpt = netdev_priv(netdev);

	strlcpy(drvinfo->driver, alx_drv_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, pci_name(adpt->pdev),
		sizeof(drvinfo->bus_info));
	drvinfo->n_stats = 0;
	drvinfo->testinfo_len = 0;
	drvinfo->regdump_len = alx_get_regs_len(netdev);
	drvinfo->eedump_len = 0;
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
};

void __devinit alx_set_ethtool_ops(struct net_device *dev)
{
	SET_ETHTOOL_OPS(dev, &alx_ethtool_ops);
}

