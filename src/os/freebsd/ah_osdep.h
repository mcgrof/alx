#ifndef	__AH_OSDEP_H__
#define	__AH_OSDEP_H__

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/linker_set.h>

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

#endif	/* __AH_OSDEP_H__ */
