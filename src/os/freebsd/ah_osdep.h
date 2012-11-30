#ifndef	__AH_OSDEP_H__
#define	__AH_OSDEP_H__

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/linker_set.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <sys/types.h>

/*
 * FreeBSD-specific typedefs.
 */
typedef	int	HAL_BOOL;
/* XXX TODO: true/false as typedefs? */
#define	AH_FALSE	0
#define	AH_TRUE		1

#define	BIT(x)		(1 << (x))

typedef	uint8_t		A_UINT8;
typedef	uint16_t	A_UINT16;
typedef	uint32_t	A_UINT32;

/* XXX TODO: make the below types A_xxx */
typedef	uint16_t	__le16;
typedef	uint32_t	__le32;
typedef	uint64_t	__le64;

typedef	uint16_t	__be16;
typedef	uint32_t	__be32;
typedef	uint64_t	__be64;

#define	be32_to_cpu(x)	be32toh(x)
#define	be16_to_cpu(x)	be16toh(x)

#define	cpu_to_be32(x)	htobe32(x)
#define	cpu_to_be16(x)	htobe16(x)

/* XXX not likely right for 64 bit platforms! */
typedef	uint32_t	A_DMA_ADDR;

/* XXX also, and this requires atomic _function calls_ to drive */
typedef	uint32_t	A_ATOMIC;

#if 0
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
#endif

/* XXX undo this */
#define	__iomem

#define	ALX_MEM_W8(s, reg, val)
#define	ALX_MEM_R8(s, reg, pdat)
#define	ALX_MEM_W16(s, reg, val)
#define	ALX_MEM_R16(s, reg, pdat)
#define	ALX_MEM_W32(s, reg, val)
#define	ALX_MEM_R32(s, reg, pdat)

#define	ALX_CFG_R16(s, reg, pdat)
#define	ALX_CFG_W16(s, reg, val)


/* Sleep functions */
#define	OS_UDELAY(s)	DELAY(s)
#define	OS_MDELAY(s)	DELAY(s)

/* Spinlocks */
typedef struct mtx	A_SPINLOCK;
#define	OS_SPIN_LOCK(s)	mtx_lock_spin(s)
#define	OS_SPIN_UNLOCK(s)	mtx_unlock_spin(s)
/* XXX init, free */

#endif	/* __AH_OSDEP_H__ */
