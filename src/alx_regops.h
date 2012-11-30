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

#ifndef	__ALX_REGOPS_H__
#define	__ALX_REGOPS_H__

/* write to 8bit register via pci memory space */
#define ALX_MEM_W8(s, reg, val) (writeb((val), ((s)->hw_addr + reg)))

/* read from 8bit register via pci memory space */
#define ALX_MEM_R8(s, reg, pdat) (\
		*(A_UINT8 *)(pdat) = readb((s)->hw_addr + reg))

/* write to 16bit register via pci memory space */
#define ALX_MEM_W16(s, reg, val) (writew((val), ((s)->hw_addr + reg)))

/* read from 16bit register via pci memory space */
#define ALX_MEM_R16(s, reg, pdat) (\
		*(A_UINT16 *)(pdat) = readw((s)->hw_addr + reg))

/* write to 32bit register via pci memory space */
#define ALX_MEM_W32(s, reg, val) (writel((val), ((s)->hw_addr + reg)))

/* read from 32bit register via pci memory space */
#define ALX_MEM_R32(s, reg, pdat) (\
		*(A_UINT32 *)(pdat) = readl((s)->hw_addr + reg))

/* read from 16bit register via pci config space */
#define ALX_CFG_R16(s, reg, pdat) (\
	pci_read_config_word((s)->pdev, (reg), (pdat)))

/* write to 16bit register via pci config space */
#define ALX_CFG_W16(s, reg, val) (\
	pci_write_config_word((s)->pdev, (reg), (val)))

/* flush regs */
#define ALX_MEM_FLUSH(s) (readl((s)->hw_addr))


#endif /* __ALX_REGOPS_H__ */
