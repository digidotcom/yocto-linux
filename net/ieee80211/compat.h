/*
 * Header file to maintain compatibility among different kernel versions.
 *
 * Copyright (c) 2004-2006  Zhu Yi <yi.zhu@intel.com>, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. See README and COPYING for
 * more details.
 */

#include <linux/version.h>
#include <linux/if_ether.h>	/* ETH_ALEN */
#include <linux/wireless.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8)
#define        __iomem
#define        __le32          u32
#endif

#ifndef        NETDEV_TX_OK
#define        NETDEV_TX_OK            0
#endif

#ifndef ARPHRD_IEEE80211_RADIOTAP
#define ARPHRD_IEEE80211_RADIOTAP 803  /* IEEE 802.11 + radiotap header */
#endif

#ifndef __bitwise /* if __leXX is not defined */
typedef __u16 __le16;
typedef __u64 __le64;
#endif

#ifndef DEFINE_SPINLOCK
#define DEFINE_SPINLOCK(s)	spinlock_t s = SPIN_LOCK_UNLOCKED
#endif

#ifndef WIRELESS_SPY
#define WIRELESS_SPY		/* enable iwspy support */
#endif

#ifndef __nocast
#define __nocast
#endif

#ifndef NETDEV_TX_BUSY
#define NETDEV_TX_BUSY 1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
typedef unsigned gfp_t;
#endif

/* WE compatibility macros */
#if WIRELESS_EXT < 17
#define IW_QUAL_QUAL_UPDATED    0x01    /* Value was updated since last read */
#define IW_QUAL_LEVEL_UPDATED   0x02
#define IW_QUAL_NOISE_UPDATED   0x04
#define IW_QUAL_ALL_UPDATED     0x07
#define IW_QUAL_QUAL_INVALID    0x10    /* Driver doesn't provide value */
#define IW_QUAL_LEVEL_INVALID   0x20
#define IW_QUAL_NOISE_INVALID   0x40
#define IW_QUAL_ALL_INVALID     0x70
#endif

#if WIRELESS_EXT < 19
#define IW_QUAL_DBM             0x08    /* Level + Noise are dBm */
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
static inline int is_multicast_ether_addr(const u8 *addr)
{
       return addr[0] & 0x01;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
static inline int is_broadcast_ether_addr(const u8 *addr)
{
        return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) == 0xff;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
static inline void *kzalloc(size_t size, unsigned __nocast flags)
{
	void *ret = kmalloc(size, flags);
	if (ret)
		memset(ret, 0, size);
	return ret;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
static inline unsigned compare_ether_addr(const u8 *_a, const u8 *_b)
{
	const u16 *a = (const u16 *) _a;
	const u16 *b = (const u16 *) _b;

	BUILD_BUG_ON(ETH_ALEN != 6);
	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) != 0;
}
#endif
