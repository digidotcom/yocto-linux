/*
 *  wifi/digi_wi_g.c
 *
 *  Copyright (C) 2007 by Digi International Inc.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version2  as published by
 *  the Free Software Foundation.
*/
/*
 *  !Revision:   $Revision: 1.147 $
 *  !Author:     Bernd Westermann/Markus Pietrek/Miriam Ruiz
 *  !Descr:
 *  !References: [1] 802.11G_Programming_Guide_1.0
 *               [2] /net/m/ISO_STANDARDS/802.11g-2003.pdf
 *               [3] Wireless LANs: 802.11 WLAN Technologie und praktische
 *                   Umsetzung im Detail
*/

#include <linux/module.h>
#include <linux/platform_device.h> /* platform_get */
#include <linux/init.h>		/* __init */
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ioport.h>	/* request_region */
#include <linux/delay.h>	/* udelay */
#include <linux/crc32.h>
#include <linux/vmalloc.h>      /* vmalloc */
#include <linux/random.h>       /* get_random_bytes */
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/irq.h>		/* NO_IRQ */
#include <mach/regs-mem.h>
#include <mach/regs-sys-common.h>

/* for supported_rates */
#include <linux/../../net/ieee80211/softmac/ieee80211softmac_priv.h>
#undef assert

#include "digi_wi_g.h"

#define REMOVE_ME

#ifdef CONFIG_DIGI_WI_G_DEBUG
# define DBG_INIT          0x0001
# define DBG_MINOR         0x0002
# define DBG_RX            0x0004
# define DBG_TX            0x0008
# define DBG_INT           0x0010
# define DBG_INTERFACE     0x0020
# define DBG_LINK          0x0040
# define DBG_SECURITY      0x0080
# define DBG_UPDATE_RATE   0x0100
# define DBG_TIMEOUT       0x0200
# define DBG_TIME          0x0800
# define DBG_MAJOR         0x1000
# define DBG_HW_IO         0x2000
# define DBG_ERROR         0x4000
# define DBG_ERROR_CRIT    0x8000

static unsigned int dw_dbg_level = DBG_ERROR_CRIT;

# define DBG(flag, format, ...)          \
	do { \
		if ((dw_dbg_level & (flag)) == (flag))           \
			printk(KERN_INFO format "\n", ##__VA_ARGS__); \
	} while (0)
# define DBG_FN(flag)                       \
	do {      \
		if ((dw_dbg_level & (flag)) == (flag))       \
			printk(KERN_INFO DRIVER_NAME "@%s:line %d\n", __func__, __LINE__); \
	} while (0)
# define DBG_EXEC(expr) \
	do { \
		expr; \
	} while (0)

# define ASSERT(expr) \
	do {                          \
		if (!(expr)) {                               \
			ERROR("Assertion failed! line %d %s",     \
				__LINE__,#expr);           \
		}                                                       \
	} while (0)
# define REQUIRE_LOCKED_ANY(lock) \
	do { \
		if (unlikely(!spin_is_locked(&lock))) {      \
			ERROR(#lock " should be locked\n"); \
			dump_stack();                             \
		}                                                 \
	} while (0)

# define REQUIRE_UNLOCKED_ANY(lock) \
	do { \
		if (unlikely(spin_is_locked(&lock))) {      \
			ERROR(#lock " should be unlocked\n"); \
			dump_stack();                             \
		}                                                 \
	} while (0)

# define REQUIRE_LOCKED(priv)   REQUIRE_LOCKED_ANY(priv->lock)
# define REQUIRE_UNLOCKED(priv) REQUIRE_UNLOCKED_ANY(priv->lock)

/* for timing collection */
enum {
	INT = 0,
	INT_DONE,
	INT_TASKLET,
	INT_TASKLET_DONE,
	RX_FIFO,
	RX_AES_FIFO,
	TX_FIFO,
	TX_AES_FIFO,
	RX_TASKLET,
	RX_TASKLET_DONE,
	RX_FRAME_TO_STACK,
	RX_FRAME_TO_STACK_DONE,
	RX_PROCESS_FRAME,
	RX_PROCESS_FRAME_RX,
	RX_PROCESS_FRAME_DONE,
	RX_DECRYPT,
	RX_DECRYPT_DONE,
	RX_OVERRUN,
	RX_WORK,
	RX_WORK_DONE,
	RX_PAUSE,
	START_XMIT,
	START_XMIT_DONE,
	TX_TASKLET,
	TX_TASKLET_DONE,
	TX_SEND,
	INTERMEDIATE,
	INTERMEDIATE2,
	SET_CHANNEL,
};

# define NAME(x) [x] = #x

static const char* ns_hperf_type_names[] = {
	NAME(INT),
	NAME(INT_DONE),
	NAME(INT_TASKLET),
	NAME(INT_TASKLET_DONE),
	NAME(RX_FIFO),
	NAME(RX_AES_FIFO),
	NAME(TX_FIFO),
	NAME(TX_AES_FIFO),
	NAME(RX_TASKLET),
	NAME(RX_TASKLET_DONE),
	NAME(RX_FRAME_TO_STACK),
	NAME(RX_FRAME_TO_STACK_DONE),
	NAME(RX_PROCESS_FRAME),
	NAME(RX_PROCESS_FRAME_RX),
	NAME(RX_PROCESS_FRAME_DONE),
	NAME(RX_DECRYPT),
	NAME(RX_DECRYPT_DONE),
	NAME(RX_OVERRUN),
	NAME(RX_WORK),
	NAME(RX_WORK_DONE),
	NAME(RX_PAUSE),
	NAME(START_XMIT),
	NAME(START_XMIT_DONE),
	NAME(TX_TASKLET),
	NAME(TX_TASKLET_DONE),
	NAME(TX_SEND),
	NAME(INTERMEDIATE),
	NAME(INTERMEDIATE2),
	NAME(SET_CHANNEL),
};

#define NS_HPERF_SIZE 2048

/* ns_hperf_type_names needs to be define first before including */
//# include <asm/arch-ns9xxx/ns9xxx_hperf.h>

/* only in debug case we are interested in revision */
# define REVISION " $Revision: 1.147 $"
#else /* CONFIG_DIGI_WI_G_DEBUG */
# define DBG(flag, format, ...)		do {} while (0)
# define DBG_FN(flag)			do {} while (0)
# define DBG_EXEC(flag)			do {} while (0)
# define ASSERT(expr)			do {} while (0)
# define REQUIRE_LOCKED(lock)		do {} while (0)
# define REQUIRE_LOCKED_ANY(lock)	do {} while (0)
# define REQUIRE_UNLOCKED(lock)		do {} while (0)
# define REQUIRE_UNLOCKED_ANY(lock)	do {} while (0)
# define REVISION ""
#endif /* CONFIG_DIGI_WI_G_DEBUG */

#define CLEAR(x)			memset(&(x), 0, sizeof((x)))
#define dw_iosetbits32(offs, mask)	dw_iowrite32(dw_ioread32(offs)|(mask),(offs))
#define dw_iocleanbits32(offs, mask)	dw_iowrite32(dw_ioread32(offs)&~(mask),(offs))

#define BASIC_RATE_MASK(x) ((x) & ((unsigned char) ~IEEE80211_BASIC_RATE_MASK))
#define BASIC_RATE(x) ((x) | IEEE80211_BASIC_RATE_MASK)

#define ERROR(format, ...)	printk(KERN_ERR "*** ERROR " DRIVER_NAME\
					" @ %s: " format "\n",	\
					__func__,		\
					##__VA_ARGS__)
#define ERRORL(format, ...)	printkl(KERN_ERR "*** ERROR " DRIVER_NAME\
					" @ %s: " format "\n",	\
					__func__,		\
					##__VA_ARGS__)
#define WARNL(format, ...)	printkl(KERN_INFO DRIVER_NAME\
					" @ %s: " format "\n",	\
					__func__,		\
					##__VA_ARGS__)
#define to_dev(pdev)		platform_get_drvdata(pdev)

#define rate_is_enough(priv)	((priv)->rate.tx_data_any >= 4)
/* at least 3/4 are acknowledged */
#define rate_is_success(priv)	\
	(4 * (priv)->rate.tx_data_ack > 3 * (priv)->rate.tx_data_any)
/* less than 1/4 are not acknowledged */
#define rate_is_failure(priv)	\
	(4 * (priv)->rate.tx_data_ack < (priv)->rate.tx_data_any)

#define MAC_GROUP		0x01  /* broadcast or multicast address */
#define IS_GROUP_ADDR(addr)	(addr[ 0 ] & MAC_GROUP)

#define IS_FRAME(fc, ftype, stype) \
	(((fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) == (ftype | stype))
#define IS_DATA(fc)		IS_FRAME(fc, IEEE80211_FTYPE_DATA, IEEE80211_STYPE_DATA)
#define IS_MGMT(fc)		IS_FRAME(fc, IEEE80211_FTYPE_MGMT, 0 /* any */)


#define ACK_SIZE	14 /* ACK frame size */

#define USE_SHORTPRE(fi, rate)	\
	(((rate) != IEEE80211_CCK_RATE_1MB) && fi->use_short_preamble)
/* from Net+OS: mac_rate.c */
/* not including SIFS and PLCP preamble/header */
#define	LENGTH(bytes, rate)	((16 * (bytes) + (rate) - 1) / (rate))
/* Length (in usecs) of SIFS and PLCP preamble/header. */
#define	PRE_LEN( fi, rate)	(USE_SHORTPRE(fi, rate) ? 106 : 202)
/* Duration (in usecs) of an OFDM frame at rate (in 500kbps units)
 * including SIFS and PLCP preamble/header */
#define	OFDM_DUR(bytes, rate)	(36 + 4 * ((4 * (bytes) + (rate) + 10) / (rate)))

typedef struct {
	u8  bps;	/* bit rate in 500kbps units */
	u8  ofdm_code;	/* ofdm rate code, 0 if not ofdm */
	u16 ack_len;	/* duration of ack or cts in usecs */
} rate_info_t;

typedef int dw_rate_index_t;

/* separate maintained static because of polling performance */
static void* __iomem vbase = NULL;

static void dw_cw_set(dw_priv_t* priv, u16 fc);
static void dw_rx_fifo_error(dw_priv_t* priv);
static int  dw_rx_frame_get_length(const dw_frame_rx_t* frame);
static int  dw_rx_frame_is_duplicate(dw_priv_t* priv,
		const struct ieee80211_hdr_3addr* hdr);
static void dw_rx_frame_give_to_stack(dw_priv_t* priv, dw_frame_rx_t* frame);
static void dw_rx_frame_fetch(dw_priv_t* priv);
static void dw_rx_tasklet_handler(unsigned long data);
static void dw_tx_set_plcp(dw_frame_tx_info_t* fi, int fragment, int rate);
static int dw_tx_frame_prepare(dw_priv_t* priv,
		dw_frame_tx_info_t* fi, struct ieee80211_txb* txb);
static void dw_tx_fragment_send(dw_priv_t* priv, dw_frame_tx_t* frame);
static void dw_tx_tasklet_handler(unsigned long data);
static irqreturn_t dw_int(int irq, void *dev_id);
static void dw_tx_reset(dw_priv_t* priv);
static void dw_tx_wait_for_idle_and_pause(dw_priv_t* priv);
static void dw_tx_continue_queue(dw_priv_t* priv);
static int dw_ieee80211_hard_start_xmit(struct ieee80211_txb* txb,
		struct net_device* dev, int priority);
static void dw_update_status_led(dw_priv_t* priv);
static void dw_rate_reset(dw_priv_t* priv);
static void dw_rate_update(dw_priv_t* priv);
static void dw_management_timer(u_long a);

/* initialization stuff */
static int  __init dw_hw_init_card(struct net_device* dev);
static int  __init dw_start_dev(struct platform_device* pdev);
static int  __init dw_probe( struct platform_device* pdev);
static void dw_stop_dev(struct platform_device* pdev);
static int  dw_remove(struct platform_device* pdev);
static void dw_release_device(struct device* dev);

static void dw_set_channel(dw_priv_t*priv, u8 channel);

/* softmac/ieee interface stuff */
static void dw_softmac_txrates_change(struct net_device* dev, u32 changes);
static int dw_wx_set_encode(struct net_device* dev,
		struct iw_request_info* info,
		union iwreq_data* data, char* extra);
static int dw_wx_set_encodeext(struct net_device* dev,
		struct iw_request_info* info, union iwreq_data* data,
		char* extra);
static void dw_softmac_notify_authenticated(
	struct net_device* dev, int event_type, void* context);
static void dw_softmac_set_chan(struct net_device* dev, u8 channel);
static void dw_softmac_set_bssid_filter(
	struct net_device *dev, const u8* bssid);
static void dw_ieee80211_set_security(struct net_device* dev,
		struct ieee80211_security* sec);
static int dw_geo_init(dw_priv_t* priv);
static int dw_open(struct net_device* dev);
static int dw_close(struct net_device* dev);
static void dw_set_multicast_list(struct net_device* dev);

/* ********** Local Variables ********** */

static const char* dw_version =
	"WiFi: " DRIVER_NAME " driver " COMPILE_TIME REVISION;

/* define the resources the driver will use */
static struct resource dw_mem = {
	.name  = DRIVER_NAME,
	.start = MAC_BASE_PHYS,
	.end   = MAC_BASE_PHYS + MAC_BASE_SIZE,
	.flags = IORESOURCE_MEM,
};

/* describes the device */
static struct platform_device dw_device = {
	.id     = -1,
	.name   = DRIVER_NAME,/* must be equal to platform-driver.driver.name*/
	.resource = &dw_mem,
	.dev = {
		.release = dw_release_device,
	},
};

/* describes the driver */
static struct platform_driver dw_driver = {
	.probe  = dw_probe,
	.remove = dw_remove,
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

/* RF transceiver frequency divider for each channel */
#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
static const struct {
	u16 integer;
	u16 fraction;
} freq_table[] = {
	{ 0, 0 },
	{ 0x6847, 0x0999 }, /* 1 (2412 MHz) */
	{ 0x6847, 0x099b }, /* 2 (2417 MHz) */
	{ 0x6867, 0x0998 }, /* 3 */
	{ 0x6867, 0x099a }, /* 4 */
	{ 0x6867, 0x0999 }, /* 5 */
	{ 0x6867, 0x099b }, /* 6 */
	{ 0x6857, 0x0998 }, /* 7 */
	{ 0x6857, 0x099a }, /* 8 */
	{ 0x6857, 0x0999 }, /* 9 */
	{ 0x6857, 0x099b }, /* 10 */
	{ 0x6877, 0x0998 }, /* 11 */
	{ 0x6877, 0x099a }, /* 12 */
	{ 0x6877, 0x0999 }, /* 13 (2472 MHz) */
	{ 0x684f, 0x0ccc }, /* 14 (2484 MHz) */
};
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
static const struct {
	u16 integer;
	u16 fraction;
} freq_table[] = {
	{ 0, 0 },
	{ 0x04c7, 0x0999 }, /* 1 (2412 MHz) */
	{ 0x04c7, 0x099b }, /* 2 (2417 MHz) */
	{ 0x04e7, 0x0998 }, /* 3 */
	{ 0x04e7, 0x099a }, /* 4 */
	{ 0x04e7, 0x0999 }, /* 5 */
	{ 0x04e7, 0x099b }, /* 6 */
	{ 0x04d7, 0x0998 }, /* 7 */
	{ 0x04d7, 0x099a }, /* 8 */
	{ 0x04d7, 0x0999 }, /* 9 */
	{ 0x04d7, 0x099b }, /* 10 */
	{ 0x04f7, 0x0998 }, /* 11 */
	{ 0x04f7, 0x099a }, /* 12 */
	{ 0x04f7, 0x0999 }, /* 13 (2472 MHz) */
	{ 0x04cf, 0x0ccc }, /* 14 (2484 MHz) */
};
#endif
/* see [2] 10.4.42. */
static const u8 dw_rates[ RATES_SUPPORTED ] = {
	BASIC_RATE(IEEE80211_CCK_RATE_1MB),
	BASIC_RATE(IEEE80211_CCK_RATE_2MB),
	BASIC_RATE(IEEE80211_CCK_RATE_5MB),
	/* we need to give 11MB also to AP, as otherwise we are not
	 * authenticated or softmac ignores it because rates don't match */
	BASIC_RATE(IEEE80211_CCK_RATE_11MB),
	IEEE80211_OFDM_RATE_6MB,
	IEEE80211_OFDM_RATE_9MB,
	IEEE80211_OFDM_RATE_12MB,
	IEEE80211_OFDM_RATE_18MB,
	IEEE80211_OFDM_RATE_24MB,
	IEEE80211_OFDM_RATE_36MB,
	IEEE80211_OFDM_RATE_48MB,
	IEEE80211_OFDM_RATE_54MB,
};

/* basic rate will be calculated */
#define MK_CCK(rate, ofdm) \
	{ .bps = rate, .ofdm_code = ofdm, .ack_len = LENGTH(ACK_SIZE, rate) }
#define MK_OFDM(rate, ofdm) \
	{ .bps = rate, .ofdm_code = ofdm, .ack_len = OFDM_DUR(ACK_SIZE, rate) }
/* they need to be ordered in their bitrate, because softmac returns us
 * ap_ri.rate this way, and rates_info uses the indexes. */
static const rate_info_t rates_info[ RATES_SUPPORTED ] = {
	MK_CCK( IEEE80211_CCK_RATE_1MB,    0  ),
	MK_CCK( IEEE80211_CCK_RATE_2MB,    0  ),
	MK_CCK( IEEE80211_CCK_RATE_5MB,    0  ),
	MK_OFDM(IEEE80211_OFDM_RATE_6MB,   0xb),
	MK_OFDM(IEEE80211_OFDM_RATE_9MB,   0xf),
	MK_CCK( IEEE80211_CCK_RATE_11MB,   0  ),
	MK_OFDM(IEEE80211_OFDM_RATE_12MB,  0xa),
	MK_OFDM(IEEE80211_OFDM_RATE_18MB,  0xe),
	MK_OFDM(IEEE80211_OFDM_RATE_24MB,  0x9),
	MK_OFDM(IEEE80211_OFDM_RATE_36MB,  0xd),
	MK_OFDM(IEEE80211_OFDM_RATE_48MB,  0x8),
	MK_OFDM(IEEE80211_OFDM_RATE_54MB,  0xc),
};
#undef MK_OFDM
#undef MK_CCK

#ifdef CONFIG_DIGI_WI_G_HW_ENCRYPTION
# define DW_SW_AES_DEFAULT 0
#else
# define DW_SW_AES_DEFAULT 1
#endif  /* CONFIG_DIGI_WI_G_HW_ENCRYPTION */

static unsigned int dw_sw_aes  = DW_SW_AES_DEFAULT;
static unsigned int dw_cfg_vco = 0;  /* use built-in */

/* Tables for the conversion of the signal strenght to dBm */
/* Map LNA value to gain value */
static const unsigned char lnatable[] =	{0, 0, 23, 39};
/* Map high gain values to dBm */
static const char gaintable[] = {-82, -84, -85, -86, -87, -89, -90, -92, -94, -98};

/* ********** Local Functions ********** */

/* ********** inline stuff ********** */

/**
 * dw_to_48 - converts to numbers to an 48bit number
 */
static inline u48 dw_to_48(u32 n1, u16 n0)
{
	return (((u64) n1) << 16) | n0;
}

/**
 * dw_48_inc - increments a 48bit number, wrapping aroung on 1<<48
 */
static inline void dw_inc_48(u48* n)
{
	(*n)++;
	*n &= ((u64) 1 << 48) - 1;
}

/**
 * dw_ioread32 - reads from memory mapped FPGA
 */
static inline u32 dw_ioread32(u32 offs)
{
	u32 val = ioread32(vbase + offs);

	DBG(DBG_HW_IO, "R %04x = %x", offs, val);

	return val;
}

/**
 * dw_write32 - writes to memory mapped FPGA
 */
static inline void dw_iowrite32(u32 val, u32 offs)
{
	DBG(DBG_HW_IO, "W %04x = %x", offs, val);
	iowrite32(val, vbase + offs);
}

/**
 * dw_hw_get_status - returns the FPGA's status
 */
static inline u32 dw_hw_get_status(dw_priv_t* priv)
{
	REQUIRE_LOCKED(priv);

	return dw_ioread32(HW_GEN_STATUS);
}

/**
 * dw_hw_memcpy_to - copies memory to FPGA @ offs
 */
static inline void dw_hw_memcpy_to(u32 offs, const void* _src, int len)
{
	u32* src = (u32 *) _src;

	for (; len > 0; len -= 4, offs += 4)
		dw_iowrite32(cpu_to_be32(*src++), offs);
}

/**
 * dw_hw_memcpy_from - copies memory from FPGA @ offs
 */
static inline void dw_hw_memcpy_from(void* _dst, u32 offs, int len)
{
	u32* dst = (u32 *) _dst;

	for (; len > 0; len -= 4, offs += 4)
		*dst++ = be32_to_cpu(dw_ioread32(offs));
}

static inline void dw_hw_write_rf(u8 addr, u32 data)
{
	dw_iowrite32((((u32) addr) << 20) | data, HW_SPI_DATA);

	udelay(10);
}

/**
 * dw_hw_read_fifo - read's the FIFO's and swaps data
 *
 * no check for underrun is performed
 */
static inline void dw_hw_read_fifo(void * _dst, int _len)
{
	u32* dst = (u32 *) _dst;
	int len = _len;

	if (dw_ioread32(HW_GEN_STATUS) & STAT_RXFE)
		printk("Reading from an EMPTY RX FIFO\n");

	for (; len >= 32; len -= 32) {
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
	}

	for (; len > 0; len -= 4)
		*dst++ = be32_to_cpu(dw_ioread32(HW_DATA_FIFO));
}

/**
 * dw_hw_write_fifo - writes swapped data to FIFO
 *
 * no check for overrun is performed
 */
static inline void dw_hw_write_fifo(const void* _src, int _len)
{
	const u32* src = (const u32 *) _src;
	int len = _len;

	for (; len >= 32; len -= 32) {
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
	}

	for (; len > 0; len -= 4)
		dw_iowrite32(cpu_to_be32(*src++), HW_DATA_FIFO);
}

/**
 * dw_hw_aes_read_fifo - read's the AES FIFO's and swaps data
 *
 */
static inline void dw_hw_aes_read_fifo(void* _dst, int len)
{
	int timeout = AES_BUSY_TIMEOUT;
	u32* dst = (u32 *) _dst;

	while (len > 0) {
		if (!(dw_ioread32(HW_RSSI_AES) & AES_EMPTY)) {
			*dst++ = be32_to_cpu(dw_ioread32(HW_AES_FIFO));
			len -= 4;
		} else {
			/* !TODO. No calibration. When interrupts are enabled, use jiffies */
			if (!timeout) {
				ERROR("Timeout on read AES FIFO @ %i", len);
				break;
			}
			timeout--;
		}
	} /* while (len > 0) */
}

/**
 * dw_hw_aes_read_fifo_noswap - read's the AES FIFO's
 *
 */
static inline void dw_hw_aes_read_fifo_noswap(void* _dst, int len)
{
	int timeout = AES_BUSY_TIMEOUT;
	u32* dst = (u32 *) _dst;

	while (len > 0) {
		if (!(dw_ioread32(HW_RSSI_AES) & AES_EMPTY)) {
			*dst++ = dw_ioread32(HW_AES_FIFO);
			len -= 4;
		} else {
			/* !TODO: No calibration. When interrupts are enabled, use jiffies */
			if (!timeout) {
				ERROR("Timeout on read AES FIFO @ %i", len);
				break;
			}
			timeout--;
		}
	} /* while (len > 0) */
}

/**
 * dw_hw_aes_write_fifo - writes swapped data to AES FIFO
 *
 * no check for overrun is performed
 */
static inline void dw_hw_aes_write_fifo(const void* _src, int len)
{
	const u32* src = (const u32 *) _src;
	int timeout = AES_BUSY_TIMEOUT;

	while (len > 0) {
		if (!(dw_ioread32(HW_RSSI_AES) & AES_FULL)) {
			dw_iowrite32(cpu_to_be32(*src++), HW_AES_FIFO);
			len -= 4;
		} else {
			/* !TODO: No calibration. When interrupts are enabled, use jiffies */
			if (!timeout) {
				ERROR("Timeout on write AES FIFO");
				break;
			}
			timeout--;
		}
	} /* while (len > 0) */
}

/**
 * dw_hw_aes_write_fifo_noswap - writes to AES FIFO
 *
 * no check for overrun is performed
 */
static inline void dw_hw_aes_write_fifo_noswap(const void* _src, int len)
{
	const u32* src = (const u32 *) _src;
	int timeout = AES_BUSY_TIMEOUT;

	while (len > 0) {
		if (!(dw_ioread32(HW_RSSI_AES) & AES_FULL)) {
			dw_iowrite32(*src++, HW_AES_FIFO);
			len -= 4;
		} else {
			/* !TODO: No calibration. When interrupts are enabled, use jiffies */
			if (!timeout) {
				ERROR("Timeout on write AES FIFO");
				break;
			}
			timeout--;
		}
	} /* while (len > 0) */
}

/**
 * dw_hw_aes_wait - waits until AES is finished
 *
 * @return: 0 on timeout, otherwise > 1
 */
static inline int dw_hw_aes_wait(void)
{
	int timeout = AES_BUSY_TIMEOUT;

	/* !TODO: redesign it to run with interrupts enabled, then use jiffies */
	DBG_FN(DBG_TIMEOUT);

	while (timeout && (dw_ioread32(HW_RSSI_AES) & AES_BUSY)) {
		timeout--;
		ndelay(1);
	}

	if (!timeout)
		ERROR("Timedout on AES");

	return timeout;
}

/**
 * dw_channel_to_freq_a - calculates frequency out of channel (for 802.11a)
 */
static inline int dw_channel_to_freq_a(u8 channel)
{
	return (5000 + (5 * channel));
}

/**
 * dw_channel_to_freq_bg - calculates frequency out of channel (for 802.11b/g)
 */
static inline int dw_channel_to_freq_bg(u8 channel)
{
	int freq;

	if (14 == channel)
		freq = 2484;
	else
		freq = 2407 + (5 * channel);

	return freq;
}

static inline void dw_set_led_on(int led, u8 on)
{
	gpio_set_value(led, !on);
}

/**
 * dw_list_move - moves a list from src and appends it to dst
 */
static inline void dw_list_move(struct list_head* dst, struct list_head* src)
{
	struct list_head* cursor;
	struct list_head* next;

	list_for_each_safe(cursor, next, src)
		list_move_tail(cursor, dst);
}

static inline void dw_hw_set_vco(int channel)
{
	u32 vco = dw_cfg_vco;

	if (!vco)
#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
		vco = 0x46662;
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
		vco = 0x3020;
#else
		BUG();
#endif
	dw_hw_write_rf(3, vco);
}

/**
 * dw_rate_info - returns the index to rates_basic/rates_info
 *
 * bitrate is in 500kbps
 */
static inline dw_rate_index_t dw_rate_info_index(int bitrate)
{
	dw_rate_index_t i = 0;

	while (i < ARRAY_SIZE(rates_info)) {
		if (rates_info[ i ].bps == bitrate)
			return i;
		i++;
	}

	ERROR("Unsupported rate %i\n", bitrate);

	return 0;
}

/**
 * dw_rx_pause - pauses the receiver
 *
 * This will lead probably to FIFO overruns. In this case, the FPGA will not
 * send the acknowledgment, so the sender will try again.
 * And this gives the stack time to clear the queue.
 */
static inline void dw_rx_pause(dw_priv_t* priv, char pause)
{
	DBG_FN(DBG_RX | DBG_MINOR);
	REQUIRE_LOCKED(priv);

	priv->rx.pause = pause;

	if (pause)
		dw_iocleanbits32(HW_INTR_MASK, INTR_RXFIFO);
	else
		dw_iosetbits32(HW_INTR_MASK, INTR_RXFIFO);
}

/* ********** now the not inline stuff ********** */

/***********************************************************************
 * @Function: dw_ccmp_get_data_tx
 * @Return:
 * @Descr: Get AES encryption data for a frame
 ***********************************************************************/
static void dw_ccmp_get_data_tx(dw_priv_t *priv, struct ieee80211_hdr_3addr * hdr,
		const dw_fragment_tx_t* frag, ccmp_data_t* data,
		u8* extiv, int dlen)
{
	ccmp_key_t *key   = frag->crypt.key;
	u8 *bp;

	DBG_FN(DBG_TX);

	/* Increment packet number */
	dw_inc_48(&key->tx_pn);

	CLEAR(*data);

	memset(extiv, 0, EXTIV_SIZE);

	SET16(extiv, key->tx_pn & 0xffff);
	extiv[3] = priv->ieee->tx_keyidx << 6 | EXT_IV;

	SET32(&extiv[4], key->tx_pn >> 16);

	/* Set up CCM initial block for MIC IV */
	data->init[0] = 0x59;
	data->init[1] = 0;
	memcpy (&data->init[2], hdr->addr2, ETH_ALEN);
	data->init[8]  = extiv[7];
	data->init[9]  = extiv[6];
	data->init[10] = extiv[5];
	data->init[11] = extiv[4];
	data->init[12] = extiv[1];
	data->init[13] = extiv[0];
	data->init[14] = dlen >> 8;
	data->init[15] = dlen;

	/* Set up MIC header blocks */
	bp = (u8 *) &hdr->frame_ctl;

	data->header[0] = 0;
	data->header[1] = 22;
	data->header[2] = bp[0] & 0xcf;
	data->header[3] = bp[1] & 0xd7;
	memcpy(&data->header[4], hdr->addr1, 3*ETH_ALEN);
	data->header[22] = WLAN_GET_SEQ_FRAG(le16_to_cpu(hdr->seq_ctl));
	data->header[23] = 0;
	memset (&data->header[24], 0, 8);
}

/***********************************************************************
 * @Function: dw_ccmp_get_data_rx
 * @Return:
 * @Descr: Get AES encryption data for a frame
 ***********************************************************************/
static int dw_ccmp_get_data_rx(dw_priv_t *priv,
		const struct ieee80211_hdr_3addr * hdr,
		int dlen, ccmp_data_t *data)
{
	ccmp_key_t *key;
	u8 *bp;

	DBG_FN(DBG_RX);

	/* Not encrypted */
	if (dlen < 0 || !(hdr->payload[3] & EXT_IV)) {
		return 0;
	}

	/* Key not set */
	key = &priv->aeskeys[hdr->payload[3] >> 6];
	if (!key->valid) {
		return 0;
	}

	CLEAR(*data);

	/* Set up CCM initial block for MIC IV */
	data->init[0] = 0x59;
	data->init[1] = 0;
	memcpy (data->init+2, hdr->addr2, ETH_ALEN);

	/* extiv */
	data->init[8]  = hdr->payload[7];
	data->init[9]  = hdr->payload[6];
	data->init[10] = hdr->payload[5];
	data->init[11] = hdr->payload[4];
	data->init[12] = hdr->payload[1];
	data->init[13] = hdr->payload[0];
	data->init[14] = dlen >> 8;
	data->init[15] = dlen;

	/* Set up MIC header blocks */
	bp = (u8 *) &hdr->frame_ctl;

	data->header[0] = 0;
	data->header[1] = 22;
	data->header[2] = bp[0] & 0xcf;
	data->header[3] = bp[1] & 0xd7;
	memcpy (data->header+4, hdr->addr1, 3*ETH_ALEN);
	data->header[22] = WLAN_GET_SEQ_FRAG(le16_to_cpu(hdr->seq_ctl));
	data->header[23] = 0;
	memset (data->header+24, 0, 8);

	return 1;
}

/***********************************************************************
 * Function: dump_hex_buffer
 * Return: nothing
 * Descr: prints a buffer hexadecimal and with character if printable
 ***********************************************************************/
static void dump_hex_buffer(const void* buffer, const int len)
{
	const unsigned char* hexbuffer = (const unsigned char*)buffer;
	int i;
	const int colcount = 16;
	const int colnum = 4;
	const int colcut = colcount / colnum;

	for (i = 0; i < len; i += colcount) {
		/* print one row*/
		int j, rowlen;

		if (i+colcount <= len)
			rowlen = colcount;
		else
			rowlen = len - i;

		printk("%08X  ", (int) hexbuffer);
		printk("    ");

		/* print hexadecimal representation */
		for (j = 0; j < rowlen; j++) {
			printk("%02X ", *(hexbuffer+j));
			if ((j + 1) % colcut == 0)
				/* additional separator*/
				printk(" ");
		}

		for (j = rowlen; j < colcount; j++)
			printk("   ");

		if (rowlen != colcount)
			for (j = 0; j <= (colcount - rowlen - 1) / colcut; j++)
				printk(" ");

		printk("  ");

		/* print character representation row */
		for (j=0; j < rowlen; j++) {
			unsigned char c = *(hexbuffer+j);
			if (c < 32 || c > 127)
				c = '.';

			printk("%c", c);
		}
		printk("\n");
		hexbuffer += colcount;
	}
}

/**
 * dw_plcp_get_bitrate_cck - returns the bitrate of HW's internal rate code
 *
 * retval is rate_code/5. But division is 1us slower than switch (5us to 4us)
 */
static int dw_plcp_get_bitrate_cck(int rate_code)
{
	volatile int ret;

	DBG_FN(DBG_RX | DBG_MINOR);

	switch(rate_code) {
		case 0x0A: ret = IEEE80211_CCK_RATE_1MB;  break;
		case 0x14: ret = IEEE80211_CCK_RATE_2MB;  break;
		case 0x37: ret = IEEE80211_CCK_RATE_5MB;  break;
		case 0x6E: ret = IEEE80211_CCK_RATE_11MB; break;
		default:
//			ERROR("Unknown rate_code %i cck, using 1MB\n", rate_code);
			ret = IEEE80211_CCK_RATE_1MB;  break;
	}

	return ret;
}

/**
 * dw_plcp_get_ratecode_cck - returns the HW's internal rate code from bitrate
 *
 * retval is 5*bitrate. But switch is 1us slower than multiplication (2us to 3us)
 */
static int dw_plcp_get_ratecode_cck(int bitrate)
{
	DBG_FN(DBG_TX | DBG_MINOR);

	/* no check for unsupported bitrates, but upper layer should have handled it */
	return 5 * bitrate;
}

/**
 * dw_plcp_get_bitrate_ofdm - returns the bitrate of HW's internal rate code.
 *
 * see [1], 2.5.2
 */
static int dw_plcp_get_bitrate_ofdm(int rate_code)
{
	int ret;

	DBG_FN(DBG_RX | DBG_MINOR);

	switch(rate_code) {
		case 0xB: ret = IEEE80211_OFDM_RATE_6MB;  break;
		case 0xF: ret = IEEE80211_OFDM_RATE_9MB;  break;
		case 0xA: ret = IEEE80211_OFDM_RATE_12MB; break;
		case 0xE: ret = IEEE80211_OFDM_RATE_18MB; break;
		case 0x9: ret = IEEE80211_OFDM_RATE_24MB; break;
		case 0xD: ret = IEEE80211_OFDM_RATE_36MB; break;
		case 0x8: ret = IEEE80211_OFDM_RATE_48MB; break;
		case 0xC: ret = IEEE80211_OFDM_RATE_54MB; break;
		default:
			ERROR("Unknown rate_code %i ofdm, using 6MB\n", rate_code);
			ret = IEEE80211_OFDM_RATE_6MB;  break;
	}

	return ret;
}

/**
 * dw_plcp_get_ratecode_ofdm - returns the HW's internal rate code from bitrate
 *
 * see [1], 2.5.2
 */
static int dw_plcp_get_ratecode_ofdm(int bitrate)
{
	DBG_FN(DBG_TX | DBG_MINOR);

	return rates_info[ dw_rate_info_index(bitrate) ].ofdm_code;
}

/**
 * dw_cw_set - updates the contention window depending on the frame type
 */
static void dw_cw_set(dw_priv_t* priv, u16 fc)
{
	int cw;

	DBG_FN(DBG_INIT);
	REQUIRE_LOCKED(priv);

	if (fc & IEEE80211_STYPE_BEACON)
		cw = 2 * CW_MIN + 1;
	else if (fc & IEEE80211_STYPE_PROBE_RESP)
		cw = CW_MIN;
	else
		cw = priv->cw;

	/* Set backoff timer */
	if (cw) {
		u16 rand;
		get_random_bytes(&rand, 2);

		/* Pick a random time up to cw, convert to usecs */
		cw = 10 * (rand & cw);
		if (fc & IEEE80211_STYPE_BEACON)
			dw_iowrite32(cw, HW_BEACON_BO);
		else
			dw_iowrite32(cw, HW_BACKOFF);
	}
}

/**
 * RF transceiver transmitter gain for each power level.
 * This is the 0-15 power level, bit reversed.
 */
static unsigned char powerTable[] = {
	0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
	0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
};

/**
 * This is used to set the transmit power.  Values can range
 * from 0-15, where 8 is the default and I am told provides about
 * 12dBm (16mW) output power.
 */
static int dw_set_tx_power(struct dw_priv *priv, unsigned char value)
{
#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
	if (value > DW_TX_POWER_MAX_DBM)
		value = DW_TX_POWER_MAX_DBM;

	dw_hw_write_rf(5, 0x19e40 | powerTable[value]);
	priv->tx_power = value;
	return 1;
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
	if (value < 0)
		value = 0;
	// Map max value (15) to 14 to avoid hardware problems
	else if (value > 14)
		value = 14;
#ifdef _UNDEFINED_
	dw_hw_write_rf(5, 0x09e40 | powerTable[value]);
	priv->tx_power = value;
	return 1;
#else
	dw_hw_write_rf(5, 0x09ee0);
	priv->tx_power = 0;
	return -EIO;
#endif

#endif // defined(CONFIG_DIGI_WI_G_UBEC_JD)
}

/**
 * dw_rx_fifo_error - reports a fifo error and resets it
 */
static void dw_rx_fifo_error(dw_priv_t* priv)
{
	DBG_FN(DBG_RX);
	REQUIRE_LOCKED(priv);
	REQUIRE_LOCKED_ANY(priv->ieee->lock);

	/* the reason of the error should have been reported already */

	/* give a pulse */
	dw_iosetbits32(HW_GEN_CONTROL, GEN_RXFIFORST);
	wmb();
	dw_iocleanbits32(HW_GEN_CONTROL, GEN_RXFIFORST);

	priv->wstats.discard.misc++;
}

/**
 * dw_rx_frame_get_length - determines the frame length from hardware values
 *
 * @return frame length in bytes or -1 on failure
 */
static int dw_rx_frame_get_length(const dw_frame_rx_t* frame)
{
	const dw_hw_hdr_rx_t* hdr = &frame->hdr;
	int len = 0;

	DBG_FN(DBG_RX);

	if (MOD_OFDM == hdr->mod.mod_type) {
		const dw_hw_ofdm_t* ofdm = &hdr->plcp.ofdm;
		/* switch and no table because check for bad data from FPGA */
		switch(ofdm->rate) {
		case 0xB:
		case 0xF:
		case 0xA:
		case 0xE:
		case 0x9:
		case 0xD:
		case 0x8:
		case 0xC:
			break;
		default:
			ERROR("Wrong rate %i", ofdm->rate);
			goto error;
		}
		len = ofdm->length;
	} else {
		const dw_hw_pskcck_t* pskcck = &hdr->plcp.pskcck;
		int service;

		/* service check */
		service  = pskcck->service;
		service &= ~SERVICE_LOCKED;
		service &= ~SERVICE_MODSEL;
		service &= ~SERVICE_LENEXT;

		/* Use a switch to avoid doing a divide operation.*/
		len = pskcck->length;
		switch(pskcck->signal) {
		case 10:
			len /= 8;
			break;
		case 20:
			len /= 4;
			break;
		case 55:
			len = (11 * len) / 16;
			break;
		case 110:
			len = (11 * len) / 8;
			if (pskcck->service & SERVICE_LENEXT)
			len--;
			break;
		default:
			ERRORL("Signal not defined %i %i", pskcck->signal,
			pskcck->length);

			/* !TODO: Remove me when solved  */
			dump_hex_buffer(hdr, sizeof(*hdr));
			goto error;
		}
	}

	/* check length for integrity?  */
	if (unlikely(len > HW_RX_FIFO_SIZE)) {
		ERRORL("Wrong size %i", len);
		goto error;
	}

	return len;

error:
	return -1;
}

/**
 * dw_rx_frame_is_duplicate - checks whether the frame has been already
 * received. This is not done by hardware.
 *
 * @return 1 if duplicate otherwise 1
 */
static int dw_rx_frame_is_duplicate(dw_priv_t* priv,
		const struct ieee80211_hdr_3addr* hdr)
{
	struct list_head* it;
	dw_duplicate_t*   sender = NULL;
	int               is_duplicate = 0;

	REQUIRE_LOCKED(priv);

	/* do we had anything from that sender already? */
	list_for_each(it, &priv->rx.dups.known.list) {
                /* addr2 is sender */
                dw_duplicate_t* entry = list_entry(it, dw_duplicate_t, list);
                if (!memcmp(entry->src, hdr->addr2, ARRAY_SIZE(entry->src))) {
                        /* found sender */
                        sender = entry;
                        break;
                }
        } /* list_for_each */

        if (NULL == sender) {
                /* create an entry for the new sender */
                struct list_head* new;

                if (unlikely(list_empty(&priv->rx.dups.free.list)))
                        /* the last one in known list was the first
                         * added and so may possible be the oldest.
                         * Using jiffies is probably not necessary */
                        new = priv->rx.dups.known.list.prev;
                else
                        new = priv->rx.dups.free.list.next;

                /* move it to head of known entries */
                list_del(new);
                list_add(new, &priv->rx.dups.known.list);

                sender = list_entry(new, dw_duplicate_t, list);

                memcpy(sender->src, hdr->addr2, ARRAY_SIZE(sender->src));
        } else {
                /* did we receive the frame already? */
                u16 fc;

                fc = le16_to_cpu(hdr->frame_ctl);
                if ((fc & IEEE80211_FCTL_RETRY) &&
                    (sender->seq_ctl == hdr->seq_ctl))
                        /* we did see already the sequence number */
                        is_duplicate = 1;
        } /* if (NULL == sender) */

        if (!is_duplicate)
                /* update sequence control field */
                sender->seq_ctl = hdr->seq_ctl;

        return is_duplicate;
}

/**
 * dw_rx_frame_decrypt - decrypts a frame when necessary.
 *
 * skb->data contains undecrypted data on return
 */
#ifndef REMOVE_ME
static void dw_rx_frame_decrypt(dw_priv_t* priv, struct sk_buff* skb)
{
        DBG_FN(DBG_RX | DBG_MINOR);
}
#endif

/**
 * dw_rx_frame_give_to_stack - give the kernel/user the data
 */
static void dw_rx_frame_give_to_stack(dw_priv_t* priv, dw_frame_rx_t* frame)
{
        static struct ieee80211_rx_stats stats;
	int gain; /* For received signal strength to dBm conversion */

        DBG_FN(DBG_RX | DBG_MAJOR);

        /* processing needs time, but we want to be able to stil copy frames
         * from Rx FIFO. Therefore we are unlocked. */
        REQUIRE_UNLOCKED(priv);

#ifndef REMOVE_ME
        dw_rx_frame_decrypt(priv, frame->skb);
#endif

        /* update stats */
        CLEAR(stats);
        stats.mask =
                IEEE80211_STATMASK_RSSI   |
                IEEE80211_STATMASK_SIGNAL |
                IEEE80211_STATMASK_RATE;
        stats.mac_time = jiffies;
        stats.rate = ((MOD_OFDM == frame->hdr.mod.mod_type) ?
                       dw_plcp_get_bitrate_ofdm(frame->hdr.plcp.ofdm.rate) :
                       dw_plcp_get_bitrate_cck(frame->hdr.plcp.pskcck.signal));
        stats.freq = IEEE80211_24GHZ_BAND;
        stats.len  = frame->skb->len;


	/* Convert received signal strength to dBm */
	gain = lnatable[frame->hdr.mod.rssi_lna] + 2 * frame->hdr.mod.rssi_vga;
	if (gain > 96)
		stats.signal = -98;
	else if (gain > 86)
		stats.signal = gaintable[gain-87];
	else
		stats.signal = 5 - gain;

	/* RSSI is used only internally to determine best network and not reported back */

        /* RSSI (Received Signal Strength Indication) is a measurement of the
         * power present in a received radio signal. In an IEEE 802.11 system
         * RSSI is the received signal strength in a wireless environment, in
         * arbitrary units. RSSI measurements will vary from 0 to 255 depending
         * on the vendor. It consists of a one byte integer value. A value of 1
         * will indicate the minimum signal strength detectable by the wireless
         * card, while 0 indicates no signal. The value has a maximum of
         * RSSI_Max. - http://en.wikipedia.org/wiki/RSSI
         *
         * See also: http://www.ces.clemson.edu/linux/dbm-rssi.shtml
         */
	stats.rssi   = MAC_RSSI_MAX
		       - 15 * (frame->hdr.mod.rssi_lna - 1)
		       - 2 * (frame->hdr.mod.rssi_vga);

        /* we don't know it here, but ieee80211 stack will look into it with
           our patch in ieee80211_rx.c before
           + if (255 == stats->received_channel)
           + stats->received_channel = network->channel;
           memcpy(&network->stats, stats, sizeof(network->stats));
        */
        stats.received_channel = 255;

        /* put it on stack */
        ieee80211_rx_any(priv->ieee, frame->skb, &stats);

        /* allocate next buffer */
        frame->skb = dev_alloc_skb(DW_MTU);
        if (unlikely(NULL == frame->skb)) {
                dw_iocleanbits32(HW_GEN_CONTROL, GEN_RXEN);
                panic(DRIVER_NAME ": Out of memory\n");
        }
}

#define SPYBUF_SIZE	2312
/**
 * dw_rx_frame_fetch - Copies a frame from FIFO
 */
static void dw_rx_frame_fetch(dw_priv_t* priv)
{
        struct list_head* element;
        dw_frame_rx_t*    frame;
        int len;
        int ignore_frame = 0;
        u16 fc;
	char *spy_buf;
	unsigned int spy_len = 0;

        DBG_FN(DBG_RX | DBG_INT);
        REQUIRE_LOCKED(priv);

        if (unlikely(list_empty(&priv->rx.queue.free.list))) {
                ERROR("Frame received, but queue empty. Receiver should be paused");
                len = 0xffff;   /* for DBG_MARK */

                /* nowhere to store. Reset FIFO */
                spin_lock(&priv->ieee->lock);
                priv->ieee->stats.rx_over_errors++;
                dw_rx_fifo_error(priv);
                spin_unlock(&priv->ieee->lock);

		return;
        }

        /* use a free skb and fill it with the frame */
        element = priv->rx.queue.free.list.next;
        frame   = list_entry(element, dw_frame_rx_t, list);

        spy_buf = kmalloc(SPYBUF_SIZE, GFP_KERNEL);
	memset(spy_buf, 0, sizeof(spy_buf)); spy_len = 0;

        /* copy frame header, swapped, we need it's header for length */
        dw_hw_read_fifo(&frame->hdr, sizeof(frame->hdr));
	memcpy(spy_buf + spy_len, &frame->hdr, sizeof(frame->hdr)); spy_len += sizeof(frame->hdr);

        len = dw_rx_frame_get_length(frame);

        if ((len >= sizeof(struct ieee80211_hdr)) && (len <= DW_MTU)) {
                /* read data and if necessary decrypt it */
                size_t remaining_len = len;
                /*  */
                size_t delta_decrypt_header = sizeof(struct ieee80211_hdr_3addr) + EXTIV_SIZE - sizeof(struct ieee80211_hdr);

                struct ieee80211_hdr* hdr = (struct ieee80211_hdr*) frame->skb->data;
                struct ieee80211_hdr_3addr* hdr3 = (struct ieee80211_hdr_3addr*) hdr;
                char* data = frame->skb->data;


                /* reads data and keeps track of remaining_len */
#define READ_FIFO(to_read) do { \
		dw_hw_read_fifo(data, to_read); \
		memcpy(spy_buf + spy_len, data, to_read); spy_len += to_read; \
		data += to_read; \
		remaining_len -= to_read; \
		ASSERT(remaining_len >= 0); } while (0)

                /* read header */
                READ_FIFO(sizeof(struct ieee80211_hdr));

                fc = le16_to_cpu(hdr->frame_ctl);

                if (!dw_sw_aes &&
                    (fc & IEEE80211_FCTL_PROTECTED) &&
                    (remaining_len > delta_decrypt_header)) {
                        /* there is a encrypted frame present */
                        ccmp_data_t cdata;
                        ccmp_key_t* key;
                        size_t data_len;
                        int index;
                        u48 pn;

                        /* get frame headers and init vector from rx FIFO */
                        READ_FIFO(delta_decrypt_header);

                        /* get key index from message */
                        index = (hdr3->payload[ 3 ] >> 6) & (WEP_KEYS - 1);
                        key   = &priv->aeskeys[ index ];

                        /* get packet number from IV and check for replay.
                           packet number must be greated or equal than
                           expected one. Takes care of wrap around. */
                        pn = dw_to_48(GET32(&hdr3->payload[ 4 ]),
                                       GET16(&hdr3->payload[ 0 ]));

                        data_len = remaining_len - IEEE80211_FCS_LEN - MIC_SIZE;

                        if (key->valid &&  /* we know the key */
                            (pn - key->rx_pn >= 0) &&
                            dw_ccmp_get_data_rx(priv, hdr3, data_len, &cdata)) {
                                /* payload doesn't include MIC or CCMP */
                                len -= CCMP_SIZE;

                                /* !TODO. Convert AES_wait into a non-busy
                                   polling function */
                                /* retreive and decrypt encoded payload data */
                                dw_hw_aes_wait();

                                /* configure mode and key */
                                dw_iowrite32(HW_AES_MODE_1 | (index & 0xf),
                                              HW_AES_MODE);

                                /* this read puts AES into decrypt mode */
                                dw_ioread32(HW_AES_MODE);

                                /* write key and init vector to AES engine */
                                dw_hw_aes_write_fifo(&cdata, sizeof(cdata));

                                /* get decrypted payload data.
                                 * We will overvwrite the CCMP header
                                 * previously read. But the stack doesn't want
                                 * to see it anyway because it expects unencrypted data */
                                dw_hw_aes_read_fifo(hdr3->payload, data_len);

                                /* wait for MIC calculation to finish.
                                   !TODO: convert to non-busy */
                                dw_hw_aes_wait();

                                if (dw_ioread32(HW_RSSI_AES) & AES_MIC) {
                                        /* frame was ok */
                                        dw_inc_48(&key->rx_pn);
                                        /* the stack should not convert it */
                                        hdr->frame_ctl = cpu_to_le16(fc & ~IEEE80211_FCTL_PROTECTED);
                                } else {
                                        ignore_frame = 1;
                                        spin_lock(&priv->ieee->lock);
                                        priv->ieee->ieee_stats.rx_discards_undecryptable++;
                                        spin_unlock(&priv->ieee->lock);
                                        ERROR("Wrong MIC");
                                }
                        } else {
                                /* TKIP decrypted? Le'ts handle it by SW */
                                READ_FIFO(remaining_len);
                        }
#undef READ_FIFO
                } else {
                        /* retrieve remaining unencrypted data.
                           We read FCS, but stack will ignore it. */
                        dw_hw_read_fifo(data, remaining_len);
			memcpy(spy_buf + spy_len, data, remaining_len); spy_len += remaining_len;
                }
        } else {
                if (len > DW_MTU)
		{
                        ERROR("Oversized frame with %i bytes, ignoring it\n", len);

		}

                spin_lock(&priv->ieee->lock);
                priv->ieee->stats.rx_length_errors++;
                dw_rx_fifo_error(priv);
                spin_unlock(&priv->ieee->lock);

                ignore_frame = 1;
        }
        kfree(spy_buf);

        if (!ignore_frame) {
                /* process frame */

                int skblen  = len - IEEE80211_FCS_LEN;/* need no FCS further */
                int ignore  = 0;
                int control = ((fc & IEEE80211_FCTL_FTYPE) == IEEE80211_FTYPE_CTL);

                switch(fc & IEEE80211_FCTL_FTYPE) {
                    case IEEE80211_FTYPE_MGMT:  /* no break */
                    case IEEE80211_FTYPE_DATA:
                        break;

                    case IEEE80211_FTYPE_CTL:   /* no break */
                    default:
                        ignore = 1;
                        break;
                }

		if (!ignore &&
                    (len >= sizeof(struct ieee80211_hdr_3addr)) &&
                    dw_rx_frame_is_duplicate(priv,
                                              (const struct ieee80211_hdr_3addr*) frame->skb->data))
                        ignore = 1;

                if (!ignore) {
                        /* the layer ignores the above frames, so we don't
                         * need to alloc/free skb's for them. CTL are
                         * ignored anyway and not freed by layer (bug)?
                         * -> out of memory
                         *
                         * We have read the whole frame because a CTL frame is
                         * typically 10 Bytes large, the header 4 Bytes.
                         * Not much gain for reading only header, but a loose
                         * for all other frames and still some overhead of
                         * calculation  */
                        skb_put(frame->skb, skblen);

                        if (IS_DATA(fc))
                                priv->rate.have_activity = 1;

                        list_move_tail(element, &priv->rx.queue.filled.list);

                        if (unlikely(list_empty(&priv->rx.queue.free.list)))
                                /* no room to store any longer. So, FIFO may
                                 * overrun, but in this case, no 802.11
                                 * acknowledgements are transmitted and the
                                 * frame is repeatedly sent => less errors */

                                dw_rx_pause(priv, 1);
                } else {
                        if (priv->tx.last_was_data && control &&
                            ((fc & IEEE80211_FCTL_STYPE) == IEEE80211_STYPE_ACK))
                        {
                                priv->tx.data_pending_ack = NULL;
                                priv->rate.tx_data_ack++;
                        }

                } /* fc */
        } else {
                spin_lock(&priv->ieee->lock);
                priv->ieee->stats.rx_dropped++;
                spin_unlock(&priv->ieee->lock);
        }
}

/**
 * dw_rx_tasklet_handler - Read's all frames in fifo
 */
static void dw_rx_tasklet_handler(unsigned long data)
{
        dw_priv_t*        priv = (dw_priv_t*) data;
        struct list_head  frames;
        struct list_head* cursor;
        unsigned long     flags;

        DBG_FN(DBG_RX | DBG_INT);

        /* move the filled list to our context, so the tasklet is able to
         * continue mostly unlocked. */
        INIT_LIST_HEAD(&frames);

        dw_lock(priv, flags);
        dw_list_move(&frames, &priv->rx.queue.filled.list);
        dw_unlock(priv, flags);

        list_for_each(cursor, &frames) {
                dw_frame_rx_t* frame = list_entry(cursor, dw_frame_rx_t, list);
                dw_rx_frame_give_to_stack(priv, frame);
        }

        /* now move the processed frame list that is now empty again back to
           free  */
        dw_lock(priv, flags);
        dw_list_move(&priv->rx.queue.free.list, &frames);
        dw_rx_pause(priv, 0);
        dw_unlock(priv, flags);
}

/**
 * dw_tx_reset - reset's the Tx FIFO when a timeout is detected
 */
static void dw_tx_reset(dw_priv_t* priv)
{
        DBG_FN(DBG_TX | DBG_ERROR);
        REQUIRE_LOCKED(priv);

        ERRORL("TX Reset and retrying transmission");

        /* reset FIFO */

        dw_iosetbits32(HW_GEN_CONTROL, GEN_TXFIFORST);
        wmb();
        dw_iocleanbits32(HW_GEN_CONTROL, GEN_TXFIFORST);

        priv->tx.data_pending_ack = NULL;

        /* retransmit it */

        if (likely(!list_empty(&priv->tx.queued))) {
                dw_frame_tx_t* frame = list_entry(priv->tx.queued.prev, dw_frame_tx_t, list);
                dw_tx_fragment_send(priv, frame);
        } else {
                ERROR("Tx Queue is empty, but we have a tx timeout????");
        }
}

/**
 * dw_tx_wait_for_idle_and_pause - on return, no frame is processed any more by
 * Tx
 *
 * After the return it is safe to configure the transmitter
 */
static void dw_tx_wait_for_idle_and_pause(dw_priv_t* priv)
{
        unsigned long flags;
        int sleep;

        DBG_FN(DBG_TX);
        REQUIRE_UNLOCKED(priv);

        /* no more new transmits until dw_tx_continue_queue */
        dw_lock(priv, flags);
        priv->tx.pause = 1;
        sleep          = priv->tx.pending;
        netif_stop_queue(priv->dev);
        dw_unlock(priv, flags);

        if (sleep)
                /* wait for queue to be emptied.
                 * we mustn't be locked, because we can sleep. But that's not a
                 * problem. If the interrupt handler is faster, it will
                 * already have freed semaphore, so we run through */
                down(&priv->tx.pause_sem);
}

static void dw_tx_continue_queue(dw_priv_t* priv)
{
        unsigned long flags;

        DBG_FN(DBG_TX);
        REQUIRE_UNLOCKED(priv);

        dw_lock(priv, flags);

        priv->tx.pause = 0;
        netif_wake_queue(priv->dev);
        if (!list_empty(&priv->tx.queued)) {
                /* transmitter is paused, so nothing is pending. Send next
                   queued entry */
                dw_frame_tx_t* frame = list_entry(priv->tx.queued.prev, dw_frame_tx_t, list);
                dw_tx_fragment_send(priv, frame);
        }

        dw_unlock(priv, flags);
}


/**
 * dw_tx_set_plcp - set's PLCP Header of the frame.
 */
static void dw_tx_set_plcp(dw_frame_tx_info_t* fi, int fragment, int rate)
{
        dw_fragment_tx_t* frag = &fi->frags[ fragment ];
        dw_hw_hdr_tx_t* hdr    = &frag->hdr;
        size_t          length = frag->phys_len;

        DBG_FN(DBG_TX);

        CLEAR(*hdr);
        /* FIFO length in words of complete frame with header,
           rounded up. FCS is added automatically */
        hdr->mod.length = (sizeof(*hdr) + length + 3) / 4;

        /* FCS length is required for signal */
        length += IEEE80211_FCS_LEN;
        if (!ieee80211_is_cck_rate(rate)) {
                hdr->mod.mod_type     = MOD_OFDM;
                hdr->plcp.ofdm.rate   = dw_plcp_get_ratecode_ofdm(rate);
                hdr->plcp.ofdm.length = length;
                hdr->plcp.ofdm.raw32  = cpu_to_le32(hdr->plcp.ofdm.raw32);
        } else {
                int signal = dw_plcp_get_ratecode_cck(rate);
                hdr->mod.mod_type        = MOD_PSKCCK;
                hdr->plcp.pskcck.signal  = signal;
                hdr->plcp.pskcck.service = SERVICE_LOCKED;

                /* convert length from bytes to microseconds */
                switch(signal) {
                    case 10:  length *= 8; break;
                    case 20:  length *= 4; break;
                    case 55:  length = (16 * length + 10) / 11; break;
                    case 110:
                        length = (8 * length + 10) / 11;
                        /* set length extension bit if needed */
                        if ((11 * length) / 8 > ( frag->phys_len + IEEE80211_FCS_LEN))
                                hdr->plcp.pskcck.service |= SERVICE_LENEXT;
                        break;
                    default:
                        ERRORL("Unsupported signal/rate %i/%i", signal,rate);
                        break;
                }
                hdr->plcp.pskcck.length = cpu_to_le16(length);
                hdr->plcp.pskcck.raw32  = cpu_to_le32(hdr->plcp.pskcck.raw32);
        }
        hdr->mod.raw32 = cpu_to_le32(hdr->mod.raw32);
}

/**
 * dw_tx_ack_duration -
 *
 * @return the duration for the acknowledgement of our data package in us
 */
static int dw_tx_ack_duration(
        dw_priv_t* priv,
        const dw_frame_tx_info_t* fi,
        dw_rate_index_t index)
{
        const rate_info_t* rate_info = &rates_info[ priv->tx.basics[ index ] ];
        /* ack/crts is sent at equal or lower basic rate */
        int dur = rate_info->ack_len;

        REQUIRE_LOCKED(priv);

        /* add psk/cck preamble time */
        if (!rate_info->ofdm_code)
                dur += PRE_LEN(fi, rate_info->bps);

        return dur;
}

/**
 * dw_tx_duration -
 *
 * PLCP must be set.
 * @return the duration for the frame in us
 */
static int dw_tx_duration(
        dw_priv_t* priv,
        const dw_frame_tx_info_t* fi,
        int fragment,
        int rate)
{
        int index = dw_rate_info_index(rate);
        int dur;

        /* frame duration */
        if (rates_info[ index ].ofdm_code)
                dur = OFDM_DUR(fi->frags[ fragment ].phys_len, rate);
        else
                dur = PRE_LEN(fi, rates_info[ index ].bps) + le16_to_cpu(fi->frags[ fragment ].hdr.plcp.pskcck.length);

        return dur + dw_tx_ack_duration(priv, fi, index);
}


/**
 * dw_tx_frame_prepare - prepare everything that is needed to send linux frame
 *
 * Determine PLCP, rate, sequence/fragment number
 *
 * @return 0 on failure. Frame needs to be dropped
 */
static int dw_tx_frame_prepare(
        dw_priv_t* priv,
        dw_frame_tx_info_t* fi,
        struct ieee80211_txb* txb)
{
        struct ieee80211_hdr_3addr* hdr3        = NULL;
        const struct ieee80211_hdr_1addr* hdr1  = NULL;
        struct ieee80211_hdr* hdr  = (struct ieee80211_hdr *)txb->fragments[ 0 ]->data;
        int fc = le16_to_cpu(hdr->frame_ctl);
        int i;
        int rate;
        int rate_last_fragment;
        int rate_index;
        int duration;

        REQUIRE_LOCKED(priv);

        if (txb->fragments[ 0 ]->len >= sizeof(struct ieee80211_hdr_3addr))
                /* we need it for sequence number */
                 hdr3 = (struct ieee80211_hdr_3addr*) hdr;
        if (txb->fragments[ 0 ]->len >= sizeof(struct ieee80211_hdr_1addr))
                /* we need it for group addressing */
                 hdr1 = (struct ieee80211_hdr_1addr*) hdr;

        CLEAR(*fi);
        fi->txb     = txb;
        fi->is_data = IS_DATA(fc);

        /* the stack sets encrypted whenenver the frame needs to be encrypted,
         * but set's FCTL_PROTECTED only when it has encrypted it itself. This
         * leaves CCMP for us.
         * encrypted can also be set to 1 if we don't provide hardware. */
        fi->use_hw_encryption = fi->txb->encrypted && !(fc & IEEE80211_FCTL_PROTECTED);
        if (fi->use_hw_encryption) {
                fc |= IEEE80211_FCTL_PROTECTED;
                hdr->frame_ctl = cpu_to_le16(fc);
        }

        /* determine bit rate for fragment */
        spin_lock(&priv->softmac->lock);
        rate = rate_last_fragment = priv->softmac->txrates.default_rate;

        if (IS_MGMT(fc))
                rate_last_fragment = dw_rates[ 0 ];
        else if ((NULL != hdr3) && is_multicast_ether_addr(hdr3->addr1))
                rate = rate_last_fragment = priv->softmac->txrates.mcast_rate;
        else if (fc & IEEE80211_FCTL_PROTECTED)
                /* send all but the last at broadcast_rate */
                rate_last_fragment = priv->softmac->txrates.mcast_rate;
        spin_unlock(&priv->softmac->lock);

        /* generate sequence number, encryption and plcp */
        for (i = 0; i < fi->txb->nr_frags; i++) {
                dw_fragment_tx_t* frag = &fi->frags[ i ];
                struct sk_buff*   skb  = txb->fragments[ i ];
                int last_frag = (i == fi->txb->nr_frags - 1);
                int rate_frag = BASIC_RATE_MASK(last_frag ? rate_last_fragment : rate);

                if (txb->fragments[ i ]->len >= sizeof(struct ieee80211_hdr_3addr)) {
                        int fragment = (fi->txb->rts_included ? (i - 1) : i);
                        /* it holds a sequence/fragment number */
                        struct ieee80211_hdr_3addr* frag_hdr3 = (struct ieee80211_hdr_3addr*) txb->fragments[ i ]->data;
                        /* see ieee80211.h: WLAN_GET_SEQ_FRAG(seq) */
                        frag_hdr3->seq_ctl = cpu_to_le16((atomic_read(&priv->tx.seq_nr) << 4) |
                                                         (fragment & 0xf));
                }

                fi->use_short_preamble =
                        (rate_frag != IEEE80211_CCK_RATE_1MB) && priv->softmac->bssinfo.short_preamble;

                frag->phys_len = skb->len;

                /* provide encryption. Requires sequence number for MIC */
                if (fi->use_hw_encryption) {
                        frag->crypt.key_index = priv->ieee->tx_keyidx;
                        frag->crypt.key = &priv->aeskeys[ frag->crypt.key_index ];

                        /* Key not set */
                        if (!frag->crypt.key->valid)
                                goto error;

                        /* EXTIV and MIC are automatically appended */
                        frag->phys_len += CCMP_SIZE;
                }

                /* requires fragment length with encryption */
                dw_tx_set_plcp(fi, i, rate_frag);
        } /* plcp */

        /* add duration, for initial fragments */
        rate_index = dw_rate_info_index(BASIC_RATE_MASK(rate));
        for (i = 0; i < fi->txb->nr_frags - 1; i++) {
                struct ieee80211_hdr* frag_hdr  = (struct ieee80211_hdr *)txb->fragments[ i ]->data;

                /* ack of this frame + data of next frame */
                duration = dw_tx_ack_duration(priv, fi, rate_index) +
                        dw_tx_duration(priv, fi, i + 1, rate);

                frag_hdr->duration_id = cpu_to_le16(duration);
        }

        /* set duration for final or last fragment */
        if ((NULL != hdr1) && IS_GROUP_ADDR(hdr1->addr1))
                duration = 0;
        else
                duration = dw_tx_ack_duration(priv, fi,
                                               dw_rate_info_index(BASIC_RATE_MASK(rate_last_fragment)));

        hdr->duration_id = cpu_to_le16(duration);

        if (NULL != hdr3)
                /* no need for cutting it at 0xffff, will be done on
                   assignment to seq_ctl */
                atomic_inc(&priv->tx.seq_nr);

        return 1;

error:
        return 0;
}

/**
 * dw_tx_fragment_send - copies a fragment to FIFO and sets all registers.
 */
static void dw_tx_fragment_send(
        dw_priv_t* priv,
        dw_frame_tx_t* frame)
{
        const dw_fragment_tx_t* frag  = &frame->s.frags[ priv->tx.fragment ];
        const struct sk_buff* skb     = frame->s.txb->fragments[ priv->tx.fragment ];
        struct ieee80211_hdr* hdr  = (struct ieee80211_hdr *)skb->data;
        struct ieee80211_hdr_3addr* hdr3 = (struct ieee80211_hdr_3addr*) hdr;
        int hw_gen_ctrl = dw_ioread32(HW_GEN_CONTROL);
        u16 fc          = le16_to_cpu(hdr->frame_ctl);

        DBG_FN(DBG_TX);
        REQUIRE_LOCKED(priv);

        //if (priv->tx.retries) {
        if (priv->tx.times_sent) {
                /* Add RETRY flag, sequence number is already set */
                fc |= IEEE80211_FCTL_RETRY;
                hdr->frame_ctl = cpu_to_le16(fc);
                if (priv->cw < CW_MAX)
                        /* code starts always with 2^n-1. Therefore, we still
                         * always have 2^n-1 and never overrun CW_MAX. */
                        priv->cw = priv->cw * 2 + 1;

                priv->wstats.discard.retries++;
        } else {
                priv->cw = CW_MIN;
        }

        dw_cw_set(priv, fc);

        if (frame->s.use_short_preamble)
                hw_gen_ctrl |= GEN_SHPRE;
        else
                /* may be set from a previous frame */
                hw_gen_ctrl &= ~GEN_SHPRE;

        priv->tx.last_was_data = frame->s.is_data;
        if (frame->s.is_data) {
                priv->rate.have_activity = 1;
                priv->rate.tx_data_any++;
        }

        /* sent frame, either bye AES or unencrypted engine */
        if (!frame->s.use_hw_encryption) {
        /* Prevent transmitting until all data have been written into the tx fifo */
                dw_iowrite32(hw_gen_ctrl | GEN_TXHOLD, HW_GEN_CONTROL);
                dw_hw_write_fifo(&frag->hdr, sizeof(frag->hdr));
                dw_hw_write_fifo(skb->data,  skb->len);
                dw_iowrite32(hw_gen_ctrl, HW_GEN_CONTROL);
        } else {
                ccmp_data_t cdata;
                u8 extiv[ EXTIV_SIZE ];

                dw_ccmp_get_data_tx(priv, hdr3, frag, &cdata, extiv, skb->len - sizeof(*hdr3));

                /* frame ctl is part of data used for MIC. */
                dw_hw_aes_wait();

                /* prevent transmitting until encrypted data is ready */
                dw_iosetbits32(HW_GEN_CONTROL, GEN_TXHOLD);

                /* write MAC, Frame Header and IV FIFO */
                dw_hw_write_fifo(&frag->hdr, sizeof(frag->hdr) );
                dw_hw_write_fifo(hdr3, sizeof(*hdr3) );
                dw_hw_write_fifo(&extiv, sizeof(extiv));

                /* configure mode and key */
                dw_iowrite32(HW_AES_MODE_1 | (frag->crypt.key_index & 0xf),
                              HW_AES_MODE);

                /* write init block to AES FIFO */
                dw_hw_aes_write_fifo(&cdata, sizeof(cdata));

                /* start transmit */
                dw_iocleanbits32(HW_GEN_CONTROL, GEN_TXHOLD);

                /* write plaintext data to AES FIFO */
                dw_hw_aes_write_fifo(hdr3->payload, skb->len - sizeof(*hdr3));
        }
        priv->dev->trans_start = jiffies;
        priv->tx.pending       = 1;
        priv->tx.times_sent++;

	if (priv->ieee->iw_mode != IW_MODE_ADHOC) {
		if (((*(char *)(skb->data)) & 0x0C) == 0x08) {	/* If it is a data frame */
			priv->tx.data_pending_ack = (void *)frag;
			priv->tx.jiffies_pending_ack = jiffies;
		}
	}
}

/**
 * dw_tx_tasklet_handler - One fragment has been completed.
 *
 * is only called when entries are in tx.queued list
 */
static void dw_tx_tasklet_handler(unsigned long data)
{
        dw_priv_t*     priv        = (dw_priv_t*) data;
        dw_frame_tx_t* frame       = NULL;
        unsigned int   retry_frame = 0;
        unsigned long  flags;

        DBG_FN(DBG_TX | DBG_INT);

        dw_lock(priv, flags);

        /* check if there are further fragments left */
        if (list_empty(&priv->tx.queued) && !priv->tx.timeout)
                /* Spurious Interrupt. May happen if we slow everything down
                 * with debug messages */
                goto out;

        spin_lock(&priv->ieee->lock);  /* interrupts already disabled */

        frame = list_entry(priv->tx.queued.next, dw_frame_tx_t, list);
        if (priv->tx.timeout) {
                int max_retries =
                        ((frame->s.txb->fragments[ priv->tx.fragment ]->len >= priv->ieee->rts) ?
                          priv->long_retry_limit :
                          priv->short_retry_limit);

                priv->tx.retries++;
                priv->tx.timeout = 0;

                if (priv->tx.retries <= max_retries) {
                        retry_frame = 1;
                } else {
                        priv->tx.data_pending_ack = NULL;
                        priv->ieee->ieee_stats.tx_retry_limit_exceeded++;
                }
        } /* if (priv->tx.timeout) */

        if (!retry_frame && priv->tx.data_pending_ack == NULL) {
                /* send next fragment */
                priv->tx.fragment++;
                if (priv->tx.fragment == frame->s.txb->nr_frags) {
                        /* frame is complete. Free resources */
                        ieee80211_txb_free(frame->s.txb);
                        frame->s.txb = NULL;

                        /* queue entry can be reused */
                        list_move_tail(priv->tx.queued.next, &priv->tx.free);
                        if (!list_empty(&priv->tx.queued))
                                /* take next frame's first fragment */
                                frame = list_entry(priv->tx.queued.next, dw_frame_tx_t, list);
                        else
                                frame = NULL;

                        priv->tx.fragment = 0;
                        priv->tx.retries = 0;
                        priv->tx.times_sent = 0;

                        if (!priv->tx.pause &&
                            netif_queue_stopped(priv->dev))
                                /* we have space again */
                                netif_wake_queue(priv->dev);

                }
        }

        if (priv->tx.data_pending_ack && time_after(jiffies, priv->tx.jiffies_pending_ack + ACK_TIMEOUT)) {
                priv->tx.data_pending_ack = NULL;
        }

        spin_unlock(&priv->ieee->lock);

        if (NULL != frame)
        {
                dw_tx_fragment_send(priv, frame);
        } else {
                /* no more frames */
                priv->tx.pending = 0;
        }

        if (priv->tx.pause && list_empty(&priv->tx.queued))
                /* nothing is being processed any longer.
                   Awake listener */
                up(&priv->tx.pause_sem);

out:
        dw_unlock(priv, flags);
}

/* Supported rates info elements */
static const u8 ratesA[]  = { DW_ELEM_SUPRATES, 8, DW_RATE_BASIC+12, 18, DW_RATE_BASIC+24, 36, DW_RATE_BASIC+48, 72, 96, 108 };
static const u8 ratesB[]  = { DW_ELEM_SUPRATES, 4, DW_RATE_BASIC+2, DW_RATE_BASIC+4, 11, 22 };
static const u8 ratesG[]  = { DW_ELEM_SUPRATES, 8, DW_RATE_BASIC+2, DW_RATE_BASIC+4, 11, 22, 12, 18, 24, 36 };
static const u8 ratesGx[] = { DW_ELEM_EXTSUPRATES, 4, 48, 72, 96, 108 };

static void dw_beacon_set_plcp(dw_hw_hdr_tx_t * hdr, int rate, size_t phys_len)
{
        size_t length = phys_len;
        memset(hdr, 0, sizeof(*hdr));

	/**
	 * Length in words, including Frame Header and PLCP Header, and excluding FCS, rounded up
	 * FCS is added automatically
	 */
        hdr->mod.length = (phys_len - FCS_SIZE + 3) / 4;

        /* FCS length is required for signal */
        length += IEEE80211_FCS_LEN;
        if (!ieee80211_is_cck_rate(rate)) {
                hdr->mod.mod_type     = MOD_OFDM;
                hdr->plcp.ofdm.rate   = dw_plcp_get_ratecode_ofdm(rate);
                hdr->plcp.ofdm.length = length;
                hdr->plcp.ofdm.raw32  = cpu_to_le32(hdr->plcp.ofdm.raw32);
        } else {
                int signal = dw_plcp_get_ratecode_cck(rate);
                hdr->mod.mod_type        = MOD_PSKCCK;
                hdr->plcp.pskcck.signal  = signal;
                hdr->plcp.pskcck.service = SERVICE_LOCKED;

                /* convert length from bytes to microseconds */
                switch(signal) {
                    case 10:  length *= 8; break;
                    case 20:  length *= 4; break;
                    case 55:  length = (16 * length + 10) / 11; break;
                    case 110:
                        length = (8 * length + 10) / 11;
                        /* set length extension bit if needed */
                        if ((11 * length) / 8 > ( phys_len + IEEE80211_FCS_LEN))
                                hdr->plcp.pskcck.service |= SERVICE_LENEXT;
                        break;
                    default:
                        ERRORL("Unsupported signal/rate %i/%i", signal, rate);
                        break;
                }
                hdr->plcp.pskcck.length = cpu_to_le16(length);
                hdr->plcp.pskcck.raw32  = cpu_to_le32(hdr->plcp.pskcck.raw32);
        }
        hdr->mod.raw32 = cpu_to_le32(hdr->mod.raw32);
}

/**
 * Store supported rates elements into a buffer
 * @param bp Pointer into buffer
 * @param elem Element to store: DW_ELEM_SUPRATES, DW_ELEM_EXTSUPRATES, or 0 for both
 * @param channel Channel number
 * @return Updated buffer pointer
 */
static u8 *dw_beacon_set_rates(u8 *bp, int elem, int channel)
{
	const u8 *sr;

	if (DW_CHAN_5G (channel))
		sr = ratesA;
	//else if (OPT_BONLY)
	//	sr = ratesB;
	else
		sr = ratesG;

	/* Store up to 8 supported rates */
	if (elem != DW_ELEM_EXTSUPRATES) {
		memcpy (bp, sr, sr[1]+2);
		bp += bp[1] + 2;
	}

	/* Store remaining extended supported rates */
	if (elem != DW_ELEM_SUPRATES && sr == ratesG) {
		memcpy (bp, ratesGx, ratesGx[1]+2);
		bp += bp[1] + 2;
	}

	return bp;
}

/**
 * Create beacon and probe response frames to send in an IBSS
 * @param interval Beacon interval in TU
 * @return 1 if success, 0 if error
 */
static void dw_beacon_make_beacon(dw_priv_t * priv, int interval)
{
    dw_beacon_frame * bcnFrame = &priv->beacon_frame;
    u16 atimWindow = 0; /* ATIM window size, 0 if none */

    u8 *bp = bcnFrame->body;
    u16 bss_caps = 0, caps;

    u8 bss_addr[ ETH_ALEN ];

    priv->beacon_body_length = 0;

    bss_caps = DW_CAP_IBSS;
    //if (!(macParams.encrypt & WLN_ENCR_OPEN))
    //    bss_caps |= CAP_PRIVACY;
    //if (OPT_SHORTPRE)
    //    bss_caps |= CAP_SHORTPRE;

    memcpy(bss_addr, priv->adhoc.bssid, ETH_ALEN);

    /* Init beacon MAC header */
    memset (bcnFrame, 0, sizeof (dw_beacon_frame));
    bcnFrame->fc = IEEE80211_STYPE_BEACON;
    memset (bcnFrame->addr1, 0xff, ETH_ALEN);
    DW_SET_ADDR (bcnFrame->addr2, priv->dev->dev_addr); /* station MAC address */
    DW_SET_ADDR (bcnFrame->addr3, bss_addr); /* BSS to associate with */

    /** Set fixed params
     * Timestamp is set by hardware */
    SET16 (&bp[8], interval);

    /* Set capabilities */
    caps = bss_caps & (DW_CAP_ESS|DW_CAP_IBSS|DW_CAP_PRIVACY);
    /* Use short preamble if allowed in BSS and params and rate > 1 mbps. */
    /* caps |= DW_CAP_SHORTPRE; */
    SET16 (&bp[10], caps);
    bp += 12;

    /* Set SSID */
    bp[0] = DW_ELEM_SSID;
    bp[1] = priv->softmac->associnfo.req_essid.len;
    memcpy (&bp[2], priv->softmac->associnfo.req_essid.data, priv->softmac->associnfo.req_essid.len);
    bp += bp[1] + 2;

    /* Set supported rates */
    bp = dw_beacon_set_rates(bp, DW_ELEM_SUPRATES, priv->adhoc.channel);

    /* Set channel number */
    if (!DW_CHAN_5G (priv->adhoc.channel)) {
        bp[0] = DW_ELEM_DSPARAM;
        bp[1] = 1;
        bp[2] = priv->adhoc.channel;
        bp += bp[1] + 2;
    }

    /* Set IBSS ATIM window */
    bp[0] = DW_ELEM_IBSSPARAM;
    bp[1] = 2;
    SET16 (&bp[2], atimWindow);
    bp += bp[1] + 2;

    /* Set ERP info. */
    //if (!DW_CHAN_5G (priv->adhoc.channel) && !(OPT_BONLY)))
    {
        bp[0] = DW_ELEM_ERPINFO;
        bp[1] = 1;
        bp[2] = 0;
        bp += bp[1] + 2;
    }

    /* Set extended supported rates */
    bp = dw_beacon_set_rates(bp, DW_ELEM_EXTSUPRATES, priv->adhoc.channel);
    priv->beacon_body_length = ((u8 *)bp - (u8 *)bcnFrame->body);
    dw_beacon_set_plcp(&bcnFrame->hwHdr, IEEE80211_CCK_RATE_1MB , priv->beacon_body_length + sizeof(dw_beacon_frame) - BEACON_BODY_SIZE + FCS_SIZE);
    bcnFrame->hwHdr.plcp.pskcck.raw32 = cpu_to_be32(0x0a046802);
    priv->beacon_ready = 1;
}

static void dw_beacon_start_IBSS(struct dw_priv *priv)
{
	int atim = 0;

	dw_beacon_make_beacon(priv, DW_BEACON_INT);
	/* If starting IBSS, set beacon and ATIM intervals */
	dw_iowrite32(atim | (DW_BEACON_INT << 16), HW_CFP_ATIM);
	dw_cw_set(priv, IEEE80211_STYPE_BEACON);
	/* Write beacon frame to beacon buffer */
	dw_iosetbits32(HW_GEN_CONTROL, GEN_BEACEN);
	dw_hw_write_fifo((void *)&priv->beacon_frame, priv->beacon_body_length + sizeof(dw_beacon_frame) - BEACON_BODY_SIZE);
	dw_iocleanbits32(HW_GEN_CONTROL, GEN_BEACEN);
	/* Set interrupt mask to enable TBTT and ATIM interrupts */
	dw_iosetbits32(HW_INTR_MASK, INTR_TBTT|INTR_ATIM);
	/* Enable IBSS mode */
	dw_iosetbits32(HW_MAC_CONTROL, CTRL_IBSS|CTRL_BEACONTX);
}

/**
 * dw_beacon_associate - does whatever is necessary for association
 */
static void dw_beacon_associate(struct ieee80211softmac_device *mac,
	struct ieee80211_assoc_response *resp,
	struct ieee80211softmac_network *net)
{
	u16 cap = 0;
	u8 erp_value = net->erp_value;

	if (resp != NULL)
		cap = le16_to_cpu(resp->capability);
	mac->associnfo.associating = 0;
	mac->bssinfo.supported_rates = net->supported_rates;
	ieee80211softmac_recalc_txrates(mac);

	mac->associnfo.associated = 1;

	if (resp != NULL)
		mac->associnfo.short_preamble_available =
		(cap & WLAN_CAPABILITY_SHORT_PREAMBLE) != 0;
	ieee80211softmac_process_erp(mac, erp_value);

	if (mac->set_bssid_filter)
		mac->set_bssid_filter(mac->dev, net->bssid);
	memcpy(mac->ieee->bssid, net->bssid, ETH_ALEN);
	netif_carrier_on(mac->dev);

	if (resp != NULL)
		mac->association_id = le16_to_cpup(&resp->aid);
}

static void dw_beacon_start(struct work_struct *work)
{
	dw_priv_t *priv = container_of((struct delayed_work *)work, struct dw_priv, beacon_work);
	unsigned long flags;
	int channel;
	struct ieee80211softmac_network *net;
	struct ieee80211softmac_device *mac = priv->softmac;

	if (IW_MODE_ADHOC != priv->ieee->iw_mode
		|| mac->associnfo.associating
		|| mac->scanning
		|| priv->beacon_ready
	) return;

	if (!mac->associnfo.req_essid.len
		|| *mac->associnfo.req_essid.data == '\0'
	) return;

	channel = priv->adhoc.channel;
	if (!channel) channel = priv->channel;
	if (!channel) channel = DW_IBSS_DEFAULT_CHANNEL;

	memset(&priv->adhoc, 0, sizeof(priv->adhoc));
	get_random_bytes(priv->adhoc.bssid, ETH_ALEN); // Select a random BSSID
	priv->adhoc.bssid[0] &= ~DW_MAC_GROUP;  // clear group bit
	priv->adhoc.bssid[0] |=  DW_MAC_LOCAL;  // set local bit
	priv->adhoc.channel = channel;
	priv->adhoc.mode=IW_MODE_ADHOC;
	priv->adhoc.essid.len = mac->associnfo.req_essid.len;
	memcpy(priv->adhoc.essid.data, mac->associnfo.req_essid.data, IW_ESSID_MAX_SIZE + 1);

	net = ieee80211softmac_get_network_by_essid_locked(mac, &mac->associnfo.associate_essid);
	if (!net) {
		net = &priv->adhoc;

		mac->set_channel(mac->dev, net->channel);
		if (mac->set_bssid_filter)
			mac->set_bssid_filter(mac->dev, net->bssid);

		spin_lock_irqsave(&mac->lock, flags);
		dw_beacon_associate(mac, NULL, net);
		ieee80211softmac_call_events_locked(mac, IEEE80211SOFTMAC_EVENT_ASSOCIATED, net);
		spin_unlock_irqrestore(&mac->lock, flags);

		mac->associnfo.scan_retry = IEEE80211SOFTMAC_ASSOC_SCAN_RETRY_LIMIT;
		mac->associnfo.bssvalid = 1;
		mac->associnfo.channel = net->channel;
		memcpy(mac->associnfo.bssid, net->bssid, ETH_ALEN);
		mac->associnfo.associate_essid.len = net->essid.len;
		memcpy(mac->associnfo.associate_essid.data, net->essid.data, IW_ESSID_MAX_SIZE + 1);
	} else {
		memcpy(priv->adhoc.bssid, net->bssid, ETH_ALEN);
		priv->adhoc.essid.len = net->essid.len;
		memcpy(priv->adhoc.essid.data, net->essid.data, IW_ESSID_MAX_SIZE + 1);
	}

	dw_beacon_start_IBSS(priv);
}

/**
 * dw_int - interrupt handler for WiFi FPGA
 */
static irqreturn_t dw_int(int irq, void *dev_id)
{
	dw_priv_t* priv = dev_id;
	u32 status;

	DBG_FN(DBG_INT);

	/* acknowledge interrupt */
	spin_lock(&priv->lock);
	status = dw_ioread32(HW_INTR_STATUS);
	dw_iowrite32(status, HW_INTR_STATUS);

	if (status & INTR_RXFIFO) {
		int frame_received = 0;
		/* process all frames */
		while (!priv->rx.pause &&
			(!(dw_hw_get_status(priv) & STAT_RXFE))) {
			frame_received = 1;
			dw_rx_frame_fetch(priv);
		}

		if (frame_received)
			/* the FIFO is prone to run full, do it with high
			priority  */
			tasklet_hi_schedule(&priv->rx.tasklet);

		DBG_EXEC(status &= ~INTR_RXFIFO);
	} /* if (status & INTR_RXFIFO) */

	if (status & INTR_TIMEOUT) {
		priv->tx.timeout = 1;
		DBG_EXEC(status &= ~INTR_TIMEOUT);
	}

	if (unlikely(status & INTR_ABORT)) {
		priv->tx.timeout = 1;
		/* retransmit it */
		DBG_EXEC(status &= ~INTR_ABORT);
	}

        if (status & INTR_TXEND) {
                tasklet_schedule(&priv->tx.tasklet);
                DBG_EXEC(status &= ~INTR_TXEND);
        }

	if (status & (INTR_TBTT | INTR_ATIM)) {
		/* is emitted by hardware even if masked out :-(. */

		/** Beacon frame is already in beacon buffer.
		 * Only set backoff timer here. */
		dw_cw_set(priv, IEEE80211_STYPE_BEACON);
		DBG_EXEC(status &= ~(INTR_TBTT | INTR_ATIM));
	}

	if (unlikely(status & INTR_RXOVERRUN)) {
		DBG(DBG_RX, "Receiver overrun");
		spin_lock(&priv->ieee->lock);
		priv->ieee->stats.rx_over_errors++;
		spin_unlock(&priv->ieee->lock);
		DBG_EXEC(status &= ~INTR_RXOVERRUN);
	}

#ifdef CONFIG_DIGI_WI_G_DEBUG
	/* when not developing/debugging, we don't run extra tests. */
	if (unlikely(status))
		/* check that we had everything handled*/
		ERRORL("Unhandled interrupt 0x%08x, mask is 0x%08x", status,
				dw_ioread32(HW_INTR_MASK));
#endif  /* CONFIG_DIGI_WI_G_DEBUG */

	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int dw_ieee80211_handle_beacon (
	struct net_device * dev,
	struct ieee80211_beacon * beacon,
	struct ieee80211_network * network)
{	
	dw_priv_t* priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	if ( !priv->softmac->associnfo.associated && 
	     !priv->softmac->associnfo.associating &&
	     IW_MODE_INFRA == priv->ieee->iw_mode &&
	     priv->jiffies_last_beacon + RESCAN_TIMEOUT < jiffies &&
	     (strncmp(priv->softmac->associnfo.req_essid.data, 
	     beacon->info_element->data, beacon->info_element->len) == 0) &&
	     priv->reconnection_attempts &&
	     priv->reconnection_attempts < MAX_RECONNECTION_ATTEMPTS ) {
		dw_lock(priv, flags);
		dw_set_channel(priv, network->channel);
		ieee80211softmac_try_reassoc(priv->softmac);
		priv->jiffies_last_beacon = jiffies;
		priv->reconnection_attempts++;
		dw_unlock(priv, flags);
	}

	if ( priv->softmac->associnfo.associated &&
	     IW_MODE_INFRA == priv->ieee->iw_mode &&
	     memcmp(network->bssid, beacon->header.addr3, ETH_ALEN) == 0 ) {
		priv->jiffies_last_beacon = jiffies;
		priv->reconnection_attempts = 0;
	}
	return ieee80211softmac_handle_beacon(dev, beacon, network);
}

/* Allocate a management frame */
static u8 * ieee80211softmac_alloc_mgt(u32 size)
{
	u8 * data;

	/* Add the header and FCS to the size */
	size = size + IEEE80211_3ADDR_LEN;
	if (size > IEEE80211_DATA_LEN)
		return NULL;
	/* Allocate the frame */
	data = kzalloc(size, GFP_ATOMIC);
	return data;
}

static void
ieee80211softmac_hdr_2addr(struct ieee80211softmac_device *mac,
	struct ieee80211_hdr_2addr *header, u32 type, u8 *dest)
{
	/* Fill in the frame control flags */
	header->frame_ctl = cpu_to_le16(type);
	/* Control packets always have WEP turned off */
	if (type > IEEE80211_STYPE_CFENDACK && type < IEEE80211_STYPE_PSPOLL)
		header->frame_ctl |= mac->ieee->sec.level ? cpu_to_le16(IEEE80211_FCTL_PROTECTED) : 0;

	/* Fill in the duration */
	header->duration_id = 0;
	/* FIXME: How do I find this?
	 * calculate. But most drivers just fill in 0 (except if it's a station id of course) */

	/* Fill in the Destination Address */
	if (dest == NULL)
		memset(header->addr1, 0xFF, ETH_ALEN);
	else
		memcpy(header->addr1, dest, ETH_ALEN);
	/* Fill in the Source Address */
	memcpy(header->addr2, mac->ieee->dev->dev_addr, ETH_ALEN);
}

static void
ieee80211softmac_hdr_3addr(struct ieee80211softmac_device *mac,
	struct ieee80211_hdr_3addr *header, u32 type, u8 *dest, u8 *bssid)
{
	/* This is common with 2addr, so use that instead */
	ieee80211softmac_hdr_2addr(mac, (struct ieee80211_hdr_2addr *)header, type, dest);

	/* Fill in the BSS ID */
	if (bssid == NULL)
		memset(header->addr3, 0xFF, ETH_ALEN);
	else
		memcpy(header->addr3, bssid, ETH_ALEN);

	/* Fill in the sequence # */
	/* FIXME: I need to add this to the softmac struct
	 * shouldn't the sequence number be in ieee80211? */
}

static int dw_ieee80211_handle_probe_request (
	struct net_device * dev,
	struct ieee80211_probe_request * req,
	struct ieee80211_rx_stats * stats)
{
	dw_priv_t* priv = ieee80211softmac_priv(dev);

	if (priv->softmac->associnfo.associate_essid.len == req->info_element->len &&
		memcmp(priv->softmac->associnfo.associate_essid.data, req->info_element->data, priv->softmac->associnfo.associate_essid.len) == 0) {
		struct ieee80211_probe_response *pkt = NULL;
		struct ieee80211softmac_device *mac = priv->softmac;
		struct ieee80211softmac_network *net = &priv->adhoc;
		u8 *data;
		u32 pkt_size = 0;
		int encrypt_mpdu = 0;

		pkt = (struct ieee80211_probe_response *)ieee80211softmac_alloc_mgt(priv->beacon_body_length);

		if (unlikely(pkt == NULL)) {
			printk("Error, packet is nonexistant or 0 length\n");
			return -ENOMEM;
		}

		ieee80211softmac_hdr_3addr(mac, &(pkt->header), IEEE80211_STYPE_PROBE_RESP, net->bssid, priv->adhoc.bssid);
		data = (u8 *)pkt->info_element;
		memcpy(pkt->info_element, priv->beacon_frame.body, priv->beacon_body_length);
		data += priv->beacon_body_length;
		pkt_size = (data - (u8 *)pkt);
		ieee80211_tx_frame(priv->ieee, (struct ieee80211_hdr *)pkt, IEEE80211_3ADDR_LEN, pkt_size, encrypt_mpdu);
		kfree(pkt);
	}

	return 0;
}

/**
 * dw_ieee80211_hard_start_xmit - prepares and sends a frame or put it in tx
 * queue for transmission when current frames are completed.
 */
static int dw_ieee80211_hard_start_xmit(
	struct ieee80211_txb* txb,
	struct net_device*    dev,
	int priority)
{
	dw_priv_t*      priv  = ieee80211softmac_priv(dev);
	dw_frame_tx_t*  frame = NULL;
	int             err   = -ENOMEM;
	unsigned long   flags;
	unsigned long   ieeeflags;
	int             i;
	dw_frame_tx_info_t info;

	DBG_FN(DBG_TX);

	/* sanity checks of txb */
	BUG_ON(!txb->nr_frags);  /* nothing to do */
	if (txb->nr_frags >= TX_MAX_FRAGS) {
		/* we are lazy and use a fixed-size array */
		ERROR("Too many fragments, dropping it: %i", txb->nr_frags);
		goto error;
	}

	/* Check if the device queue is big enough for every fragment. If not,
	 * drop the whole packet. */
	for (i = 0; i < txb->nr_frags; i++) {
		if (unlikely(txb->fragments[ i ]->len >
				(HW_TX_FIFO_SIZE + sizeof(dw_hw_hdr_tx_t)))) {
			ERROR("PIO Tx Device queue too small, dropping it");

			goto error;
		}
	}

	if (netif_queue_stopped(dev)) {
		/* apperently, softmac doesn't honor netif_queue_stopped,
		   at least for control messages. Block it ourself. */
		err = -EBUSY;
		goto error;
	}

	/* take the next free queue entry and provide it's data */
	dw_lock(priv, flags);

	/* prepare everything. */
	if (!dw_tx_frame_prepare(priv, &info, txb))
		goto error_unlock;

	frame    = list_entry(priv->tx.free.next, dw_frame_tx_t, list);
	frame->s = info;

	/* put it to tx queue*/
	if (list_empty(&priv->tx.queued) && !priv->tx.pending) {
		/* no transmission is running yet, so the queue won't be
		   processed by int handler. Write it directly to FIFO
		   start with first fragment. */
		priv->tx.retries = 0;
		priv->tx.times_sent = 0;
		dw_tx_fragment_send(priv, frame);
	}
	/* int handler is locked, so we can manage queue list after sending */
	list_move_tail(priv->tx.free.next, &priv->tx.queued);

	if (list_empty(&priv->tx.free)) {
		/* Driver starts always with free entries.
		   But now they are all in use. Wait until we have some free
		   entries */
		netif_stop_queue(dev);
	}

	dw_unlock(priv, flags);

	return 0;

error_unlock:
	dw_unlock(priv, flags);

error:
	/* txb is free'd on !0 return */
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	priv->ieee->stats.tx_dropped++;
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	return err;
}

/**
 * dw_update_status_led - updates the LED depending on association info
 */
static void dw_update_status_led(dw_priv_t* priv)
{
	static int reduce_rate = 0;
	reduce_rate++;          /* slow it down to 500ms, don't divide */

	if (reduce_rate == 9) {
		static int count = 0;
		reduce_rate = 0;

		count++;
		spin_lock(&priv->ieee->lock);
		spin_lock(&priv->softmac->lock);
		if (priv->softmac->associnfo.associated) {
			if (IW_MODE_ADHOC == priv->ieee->iw_mode)
				/* slow blink, 7/8 on */
				dw_set_led_on(PIN_LED, count & 7);
			else
				dw_set_led_on(PIN_LED, 1);  /* solid on */
		} else
			dw_set_led_on(PIN_LED, count & 1);  /* faster blink */
		spin_unlock(&priv->softmac->lock);
		spin_unlock(&priv->ieee->lock);
	}
}

/**
 * dw_rate_reset - resets rate statistics
 */
static void dw_rate_reset(dw_priv_t* priv)
{
	int i;

	DBG_FN(DBG_INTERFACE | DBG_UPDATE_RATE);
	REQUIRE_LOCKED(priv);

	CLEAR(priv->rate);

	priv->rate.success_threshold = THRESH_MIN;

	/* find index in ap_ri that matches default_rate */
	for (i = 0; i < priv->ap_ri.count; i++) {
		if (priv->softmac->txrates.default_rate == BASIC_RATE_MASK(priv->ap_ri.rates[ i ])) {
			priv->rate.index = i;
			break;
		}
	}
}

/**
 * dw_rate_update - selects a better rate depending on frame error count
 *
 * taken from wifi_mac:mac_rate.c:UpdateRate
 */
static void dw_rate_update(dw_priv_t* priv)
{
	int changed = 0;

	DBG_FN(DBG_UPDATE_RATE);
	REQUIRE_LOCKED(priv);

	if (atomic_read(&priv->fix_rate) || !rate_is_enough(priv))
		/* user requested fix rate or not enough data*/
		return;

	spin_lock(&priv->softmac->lock);
	if (!priv->softmac->associnfo.associated)
		goto out;

	if (rate_is_success(priv)) {
		/* try to increase rate */
		priv->rate.success++;
		if ((priv->rate.success >= priv->rate.success_threshold) &&
		    (priv->rate.index < (priv->ap_ri.count - 1))) {
			/* we are successfull long enough */
			priv->rate.recovery = 1;
			priv->rate.success  = 0;
			priv->rate.index++;
			changed = 1;
		} else
			priv->rate.recovery = 0;
	} else if (rate_is_failure(priv)) {
		/* decrease rate */
		if (priv->rate.index > 0) {
			if (priv->rate.recovery) {
				/* errors resulted from a rate increase.
				   double successfull intervals needed to
				   incrase next time */
				priv->rate.success_threshold *= 2;
				if (priv->rate.success_threshold > THRESH_MAX)
					priv->rate.success_threshold = THRESH_MAX;
			} else
				priv->rate.success_threshold = THRESH_MIN;

			priv->rate.index--;
			changed = 1;
		}

		priv->rate.success  = 0;
		priv->rate.recovery = 0;
	}

	if (changed) {
		priv->softmac->txrates.default_rate = BASIC_RATE_MASK(priv->ap_ri.rates[ priv->rate.index ]);
		DBG(DBG_UPDATE_RATE, "Rate changed to %i",
		     priv->softmac->txrates.default_rate);
	}

	/* reset counter so we work on the next time slice */
	priv->rate.tx_data_any = priv->rate.tx_data_ack = 0;

out:
	spin_unlock(&priv->softmac->lock);
}

/**
 * dw_management_timer - performs all periodic background non tx/rx work
 */
static void dw_management_timer(unsigned long a)
{
	dw_priv_t*    priv = (dw_priv_t*) a;
	unsigned long flags;

	DBG_FN(DBG_UPDATE_RATE);

	dw_lock(priv, flags);

	switch (priv->ieee->iw_mode) {
		case IW_MODE_ADHOC:
			if (!priv->softmac->scanning && !priv->beacon_ready) {
				schedule_delayed_work(&priv->beacon_work, 0);
			}
			break;
		case IW_MODE_INFRA:
			if (priv->softmac->associnfo.associated && priv->jiffies_last_beacon + BEACON_TIMEOUT < jiffies)
			{
				priv->softmac->associnfo.associated = 0;
				ieee80211softmac_start_scan(priv->softmac);
				priv->jiffies_last_beacon = jiffies;
				ieee80211softmac_try_reassoc(priv->softmac);
				priv->reconnection_attempts = 1;
			} else if (
				!priv->softmac->associnfo.associated
				&& !priv->softmac->associnfo.associating
				&& priv->reconnection_attempts
				&& priv->softmac->associnfo.req_essid.len
				&& priv->softmac->associnfo.req_essid.data
				&& *priv->softmac->associnfo.req_essid.data
				&& priv->jiffies_last_beacon + RESCAN_TIMEOUT < jiffies)
			{
				if (priv->reconnection_attempts < MAX_RECONNECTION_ATTEMPTS) {
					priv->softmac->associnfo.associated = 0;
					ieee80211softmac_start_scan(priv->softmac);
					priv->jiffies_last_beacon = jiffies;
					ieee80211softmac_try_reassoc(priv->softmac);
					priv->reconnection_attempts++;
				} else {
					ieee80211softmac_disassoc(priv->softmac);
					memset(priv->softmac->associnfo.bssid, 0, ETH_ALEN);
					priv->reconnection_attempts = 0;
				}
			}
			break;
		default:
			break;
	}

	dw_update_status_led(priv);

	if (priv->rate.have_activity) {
		priv->rate.have_activity = 0;
		/* blink when data arrives */
		dw_set_led_on(PIN_ACTIVITY_LED,
			       priv->activity.counter & 1);

		priv->activity.counter++;
	} else
		/* turn it off */
		dw_set_led_on(PIN_ACTIVITY_LED, 0);

	priv->rate.counter++;
	if (!(priv->rate.counter % MANAGEMENT_TICKS_FOR_UPDATE))
		/* we don't need to update rate each LED timer event */
		dw_rate_update(priv);

	if (priv->tx.pending && time_after(jiffies, priv->dev->trans_start + TX_TIMEOUT)) {
		/* we can't use tx_timeout. It only works when
		 * netif_carrier_ok, but that is set only when associated.
		 * But we could loose association and run into a tx
		 * hanger. Therefore we;d never get associated, and never
		 * reset.So do it homemade. */
		dw_tx_reset(priv);
	}

	mod_timer(&priv->management_timer, jiffies + MANAGEMENT_JIFFIES);
	dw_unlock(priv, flags);
}

/**
 * dw_hw_init_card - initializes card and memory settings
 */
static int __init dw_hw_init_card(struct net_device* dev)
{
	dw_priv_t*    priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	u32 firmware_id;
	u32 ctrl;
	u32 mac_status;

	DBG_FN(DBG_INIT);

	dw_lock(priv, flags);

	/* Configure PIO pins */
	gpio_direction_output(PIN_LED, 0);
	gpio_direction_output(PIN_ACTIVITY_LED, 0);
	dw_set_led_on(PIN_LED, 0);
	dw_set_led_on(PIN_ACTIVITY_LED, 0);
	gpio_configure_ns9360(PIN_INTR, 0, 1, 0);

	/* CS2 is used for wireless baseband registers.
	   Map to MAC_BASE address, 32 bit width, 8 wait states.*/
	iowrite32(0x82, MEM_DMCONF(2));
	iowrite32(0, MEM_SMWED(2));
	iowrite32(2, MEM_SMOED(2));
	iowrite32(8, MEM_SMRD(2));
	iowrite32(0, MEM_SMPMRD(2));
	iowrite32(4, MEM_SMWD(2));
	iowrite32(2, MEM_SWT(2));

	iowrite32(MAC_BASE_PHYS, SYS_SMCSSMB(2));
	iowrite32(MAC_MASK, SYS_SMCSSMM(2));

	/* Init baseband processor and MAC assist */

	ctrl = GEN_INIT | GEN_ANTDIV;

	/* we can't do a softreset here */

	/* go back to normal state */

	dw_iowrite32(ctrl,       HW_GEN_CONTROL);
	dw_iowrite32(0,          HW_MAC_CONTROL);
	dw_iowrite32(0,          HW_INTR_MASK);
	/* Mike's recommendation for antenna diversity and antenna map value.
	   Transmit only on primary antenna, receive on diversity */
	dw_iowrite32(0x88000000, HW_RSSI_AES);

	/* get MAC from fpga */
	dw_hw_memcpy_from(dev->dev_addr, HW_STAID0, ETH_ALEN);

	/* Initialize RF tranceiver */
#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
	dw_hw_write_rf(7, 0x27ffe); /* TEST register */
	dw_hw_write_rf(6, 0xf81ac); /* Filter Register */
	dw_set_tx_power(priv, DW_TX_POWER_DEFAULT); /* Transmitter Gain */
	dw_hw_write_rf(4, 0x0002b); /* Receiver Gain */

	dw_hw_set_vco(0);     /* no channel selected yet */

	dw_hw_write_rf(0, 0x25f9c); /* Mode Control - Calibrate Filter */
	udelay(10);
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
	dw_hw_set_vco(0);     /* no channel selected yet */
	dw_hw_write_rf(4, 0x0007b); /* Receiver Gain */
	dw_set_tx_power(priv, DW_TX_POWER_DEFAULT); /* Transmitter Gain */
	dw_hw_write_rf(6, 0xf81ac); /* Filter Register */
	dw_hw_write_rf(7, 0x3fffe); /* TEST register */

	dw_hw_write_rf(0, 0x27fdc); /* Mode Control - Calibrate Filter */
	udelay(10);
	dw_hw_write_rf(0, 0x27fd4); /* Mode Control - RX/TX mode */
#endif

	/* Firmware version */
	firmware_id = dw_ioread32(HW_VERSION);
	mac_status = dw_ioread32(HW_MAC_STATUS);
	printk(KERN_DEBUG DRIVER_NAME ": FPGA  HW version: %i.%02i  FW Version: %i.%02i\n",
		(firmware_id >> 8) & 0xff, firmware_id & 0xff,
		(mac_status >> 24) & 0xff, (mac_status >> 16) & 0xff);

	dw_unlock(priv, flags);

	return 1;
}

/**
 * dw_reset_rx_dups - resets the duplicate list and forgets the senders
 */
static void dw_reset_rx_dups(dw_priv_t* priv)
{
	unsigned long flags;
	int i;

	REQUIRE_UNLOCKED(priv);

	dw_lock(priv, flags);

	CLEAR(priv->rx.dups);

	/* for detecting duplicates */
	INIT_LIST_HEAD(&priv->rx.dups.known.list);
	INIT_LIST_HEAD(&priv->rx.dups.free.list);
	for (i = 0; i < ARRAY_SIZE(priv->rx.dups.entries); i++) {
		struct list_head* entry = &priv->rx.dups.entries[ i ].list;
		INIT_LIST_HEAD(entry);
		list_add_tail(entry, &priv->rx.dups.free.list);
	} /* for (i = 0) */

	dw_unlock(priv, flags);
}

/**
 * dw_setup_rx_queue - creates the list structure
 *
 * @return: -ENOMEM on no failures otherwise 0
 */
static int __init dw_setup_rx_queue(dw_priv_t* priv)
{
        dw_frame_rx_t* tmp = NULL;
        size_t aligned_size;
        size_t buffer_size;
        int    i;

        INIT_LIST_HEAD(&priv->rx.queue.free.list);
        INIT_LIST_HEAD(&priv->rx.queue.filled.list);

        aligned_size = ((char*) &tmp[ 1 ]) - ((char*) &tmp[ 0 ]);
        buffer_size  = aligned_size * RX_QUEUE_SIZE;
        priv->rx.queue.buffer =
                (dw_frame_rx_t*) vmalloc(buffer_size);
        if (NULL == priv->rx.queue.buffer)
                goto error;

        memset(priv->rx.queue.buffer, 0, buffer_size);

        /* create frame skb and list for moving unswapped frames */
        for (i = 0; i < RX_QUEUE_SIZE; i++) {
                dw_frame_rx_t*    frame = &priv->rx.queue.buffer[ i ];
                struct list_head* entry = &frame->list;

                frame->skb = dev_alloc_skb(DW_MTU);
                if (NULL == frame->skb) {
                        for (; i > 0; i--) {
                                dev_kfree_skb(frame->skb);
                                frame->skb = NULL;
                        }

                        goto error_skb;
                }

                INIT_LIST_HEAD(entry);
                list_add(entry, &priv->rx.queue.free.list);
        }

        return 0;

error_skb:
        vfree(priv->rx.queue.buffer);
        priv->rx.queue.buffer = NULL;
error:
        ERROR("No Memory for Rx Queue");
        return -ENOMEM;
}

/**
 * dw_start_dev - starts resource
 */
static int dw_start_dev(struct platform_device* pdev)
{
	struct net_device* dev  = to_dev(pdev);
	struct dw_priv*    priv = ieee80211softmac_priv(dev);
	struct list_head*  cursor;
	int err = -ENODEV;
	int i;

	DBG_FN(DBG_INIT);

	CLEAR(*priv);

	spin_lock_init(&priv->lock);
	priv->dev         = dev;
	priv->short_retry_limit = DW_DEFAULT_SHORT_RETRY_LIMIT;
	priv->long_retry_limit  = DW_DEFAULT_LONG_RETRY_LIMIT;
	snprintf(priv->nick, IW_ESSID_MAX_SIZE, "Digi Wireless b/g");

	atomic_set(&priv->fix_rate, 0);

	/* the stack may later reduce the number of supported rates if the
	 * access point can't provide them */
	ASSERT(ARRAY_SIZE(dw_rates) <= ARRAY_SIZE(priv->ap_ri.rates));
	for (i = 0; i < ARRAY_SIZE(dw_rates); i++)
		priv->ap_ri.rates[ i ] = dw_rates[ i ];
	priv->ap_ri.count = i;

	/* rx */
	err = dw_setup_rx_queue(priv);
	if (err < 0)
		goto error_rx_queue;

	tasklet_init(&priv->rx.tasklet, dw_rx_tasklet_handler,
		      (unsigned long) priv);
	tasklet_disable(&priv->rx.tasklet);  /* will be enabled in open */

	/* tx */
	atomic_set(&priv->tx.seq_nr, 0);
	/* first down should sleep */
	init_MUTEX_LOCKED(&priv->tx.pause_sem);
	INIT_LIST_HEAD(&priv->tx.queued);
	INIT_LIST_HEAD(&priv->tx.free);
	for (i = 0; i < ARRAY_SIZE(priv->tx.frames); i++) {
		struct list_head* entry = &priv->tx.frames[ i ].list;
		INIT_LIST_HEAD(entry);
		list_add_tail(entry, &priv->tx.free);
	}

	tasklet_init(&priv->tx.tasklet, dw_tx_tasklet_handler,
			(unsigned long) priv);
	tasklet_disable(&priv->tx.tasklet);

	init_timer(&priv->management_timer);
	priv->management_timer.function = dw_management_timer;
	priv->management_timer.data     = (unsigned long) priv;

	if (!dw_hw_init_card(dev))
		goto error_init;

	/* card is setup, now we can register the irq */
	err = request_irq(dev->irq, dw_int, 0, dev->name,
			priv);
	if (err) {
		ERROR("register interrupt %d failed, err %d", dev->irq, err);
		goto error_irq;
	}
	disable_irq(dev->irq);

	priv->softmac = ieee80211_priv(dev);
	priv->softmac->set_channel      = dw_softmac_set_chan;
	priv->softmac->set_bssid_filter = dw_softmac_set_bssid_filter;
	priv->softmac->txrates_change   = dw_softmac_txrates_change;
	priv->ieee = netdev_priv(dev);
	priv->ieee->modulation = IEEE80211_OFDM_MODULATION | IEEE80211_CCK_MODULATION;
	priv->ieee->hard_start_xmit = dw_ieee80211_hard_start_xmit;
	priv->ieee->set_security    = dw_ieee80211_set_security;
	priv->ieee->handle_beacon   = dw_ieee80211_handle_beacon;
	priv->ieee->handle_probe_request = dw_ieee80211_handle_probe_request;
	priv->ieee->iw_mode         = IW_MODE_INFRA;
	priv->ieee->freq_band       = IEEE80211_24GHZ_BAND;
	priv->ieee->rts             = DW_MAX_RTS_THRESHOLD;
	priv->ieee->perfect_rssi    = MAC_RSSI_MAX;
	priv->ieee->worst_rssi      = 0;
	priv->ieee->config         |= CFG_IEEE80211_RTS;

	if (dw_geo_init(priv) < 0)
		goto error_init2;

	if (register_netdev(dev))
		goto error_init2;

	ieee80211softmac_notify(dev, IEEE80211SOFTMAC_EVENT_AUTHENTICATED,
			dw_softmac_notify_authenticated, NULL);

	return 0;

error_init2:
	free_irq(dev->irq, priv);
error_irq:
error_init:
	list_for_each(cursor, &priv->rx.queue.free.list) {
		dw_frame_rx_t* frame = list_entry(cursor, dw_frame_rx_t, list);
		dev_kfree_skb(frame->skb);
		frame->skb = NULL;
	}
	vfree(priv->rx.queue.buffer);
	priv->rx.queue.buffer = NULL;

error_rx_queue:
	return err;
}

/**
 * dw_stop_dev - stops device
 */
static void dw_stop_dev(struct platform_device* pdev)
{
	struct net_device* dev  = to_dev(pdev);
	struct dw_priv*    priv = ieee80211softmac_priv(dev);
	unsigned long      flags;
	struct list_head*  it;

	DBG_FN(DBG_INIT);

	dw_lock(priv, flags);
	dw_iowrite32(0, HW_MAC_CONTROL);
	dw_unlock(priv, flags);

	list_for_each(it, &priv->rx.queue.free.list) {
		dw_frame_rx_t* frame = list_entry(it, dw_frame_rx_t, list);
		dev_kfree_skb(frame->skb);
		frame->skb = NULL;
	}
	vfree(priv->rx.queue.buffer);

	unregister_netdev(dev);

	dw_set_led_on(PIN_LED, 0);
	dw_set_led_on(PIN_ACTIVITY_LED, 0);

	gpio_free(PIN_LED);
	gpio_free(PIN_ACTIVITY_LED);
	gpio_free(PIN_INTR);
}

/**
 * dw_remove - stops device and removes all resources
 */
static int dw_remove(struct platform_device* pdev)
{
	struct net_device* dev = to_dev(pdev);

	DBG_FN(DBG_INIT);

	dw_stop_dev(pdev);

	/* unmap resources */
	if (0 != dev->base_addr) {
		iounmap(vbase);
		vbase = NULL;
	}
	free_irq(dev->irq, ieee80211softmac_priv(dev));

	free_ieee80211softmac(dev);
	dev = NULL;

	platform_set_drvdata(pdev, NULL);

	release_resource(pdev->resource);

	return 0;
}

/**
 * dw_release_device - dummy function
 */
static void dw_release_device(struct device* dev)
{
	DBG_FN(DBG_INIT);

	/* nothing to do. But we need a !NULL function */
}

/**
 * dw_softmac_txrates_change - adjusts rate to allowed values.
 *
 * Select a better on.
 */
static void dw_softmac_txrates_change(struct net_device* dev, u32 changes)
{
	dw_priv_t* priv       = ieee80211softmac_priv(dev);
	int        was_locked = spin_is_locked(&priv->lock);
	unsigned long flags   = 0;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	if (!was_locked)
		/* we are called either directly from open/user interface,
		   or from our tasklet context when processing a frame.*/
		dw_lock(priv, flags);

	/* new rate */
	CLEAR(priv->rate);

	dw_rate_reset(priv);

	if (!was_locked)
		dw_unlock(priv, flags);
}

/**
 * dw_wx_set_encode - changes the encoding
 *
 * Used by wireless tools <= 28 for WEP-1. Not AES
 */
static int dw_wx_set_encode(struct net_device* dev,
		struct iw_request_info* info, union iwreq_data* data,
		char* extra)
{
	dw_priv_t*    priv = ieee80211softmac_priv(dev);
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);

	priv->ieee->host_encrypt      = 1;
	priv->ieee->host_encrypt_msdu = 1;
	priv->ieee->host_decrypt      = 1;
	priv->ieee->host_mc_decrypt   = 1;

	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	return ieee80211_wx_set_encode(priv->ieee, info, data, extra);
}

/**
 * dw_wx_set_encodeext - changes the encoding
 *
 * Only used by wpa_supplicant or wireless_tool >= 29
 */
static int dw_wx_set_encodeext(struct net_device* dev,
	struct iw_request_info* info, union iwreq_data* data,
	char* extra)
{
	dw_priv_t*            priv = ieee80211softmac_priv(dev);
	struct iw_encode_ext* ext = (struct iw_encode_ext*) extra;
	unsigned long ieeeflags;
	int           host = (dw_sw_aes || (IW_ENCODE_ALG_CCMP != ext->alg));

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);

	priv->ieee->host_encrypt      = host;
	priv->ieee->host_encrypt_msdu = host;
	priv->ieee->host_decrypt      = host;
	priv->ieee->host_mc_decrypt   = host;

	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	/* wx_set_encodeext will initialize itself depending on host_encrypt.
	 * After wx_set_encodeext, it will be of no further use. */
	return ieee80211_wx_set_encodeext(priv->ieee, info, data, extra);
}

/**
 * dw_softmac_notify_authenticated - called when authenticated
 *
 * Resets statistics
 */
static void dw_softmac_notify_authenticated(struct net_device* dev,
		int event_type,  void* context)
{
	dw_priv_t*    priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE);

	dw_lock(priv, flags);
	dw_rate_reset(priv);
	dw_unlock(priv, flags);
}

/***********************************************************************
 * @Function: dw_ieee80211_get_network_by_bssid
 * @Return:
 * @Descr: Get a network from the list by BSSID with locking
 ***********************************************************************/
struct ieee80211_network *
dw_ieee80211_get_network_by_bssid(struct dw_priv *priv, u8 *bssid)
{
	unsigned long ieeeflags;
	struct ieee80211_network *ieee_net = NULL, *tmp_net;
	struct list_head *list_ptr;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	list_for_each(list_ptr, &priv->ieee->network_list) {
		tmp_net = list_entry(list_ptr, struct ieee80211_network, list);
		if (!memcmp(tmp_net->bssid, bssid, ETH_ALEN)) {
			if (ieee_net==NULL) {
				ieee_net = tmp_net;
			} else {
				if (tmp_net->last_scanned > ieee_net->last_scanned)
					ieee_net = tmp_net;
			}
		}
	}
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	return ieee_net;
}

/* CCMP encryption data */
static const u32 te4[256] = {
	0x63636363U, 0x7c7c7c7cU, 0x77777777U, 0x7b7b7b7bU,
	0xf2f2f2f2U, 0x6b6b6b6bU, 0x6f6f6f6fU, 0xc5c5c5c5U,
	0x30303030U, 0x01010101U, 0x67676767U, 0x2b2b2b2bU,
	0xfefefefeU, 0xd7d7d7d7U, 0xababababU, 0x76767676U,
	0xcacacacaU, 0x82828282U, 0xc9c9c9c9U, 0x7d7d7d7dU,
	0xfafafafaU, 0x59595959U, 0x47474747U, 0xf0f0f0f0U,
	0xadadadadU, 0xd4d4d4d4U, 0xa2a2a2a2U, 0xafafafafU,
	0x9c9c9c9cU, 0xa4a4a4a4U, 0x72727272U, 0xc0c0c0c0U,
	0xb7b7b7b7U, 0xfdfdfdfdU, 0x93939393U, 0x26262626U,
	0x36363636U, 0x3f3f3f3fU, 0xf7f7f7f7U, 0xccccccccU,
	0x34343434U, 0xa5a5a5a5U, 0xe5e5e5e5U, 0xf1f1f1f1U,
	0x71717171U, 0xd8d8d8d8U, 0x31313131U, 0x15151515U,
	0x04040404U, 0xc7c7c7c7U, 0x23232323U, 0xc3c3c3c3U,
	0x18181818U, 0x96969696U, 0x05050505U, 0x9a9a9a9aU,
	0x07070707U, 0x12121212U, 0x80808080U, 0xe2e2e2e2U,
	0xebebebebU, 0x27272727U, 0xb2b2b2b2U, 0x75757575U,
	0x09090909U, 0x83838383U, 0x2c2c2c2cU, 0x1a1a1a1aU,
	0x1b1b1b1bU, 0x6e6e6e6eU, 0x5a5a5a5aU, 0xa0a0a0a0U,
	0x52525252U, 0x3b3b3b3bU, 0xd6d6d6d6U, 0xb3b3b3b3U,
	0x29292929U, 0xe3e3e3e3U, 0x2f2f2f2fU, 0x84848484U,
	0x53535353U, 0xd1d1d1d1U, 0x00000000U, 0xededededU,
	0x20202020U, 0xfcfcfcfcU, 0xb1b1b1b1U, 0x5b5b5b5bU,
	0x6a6a6a6aU, 0xcbcbcbcbU, 0xbebebebeU, 0x39393939U,
	0x4a4a4a4aU, 0x4c4c4c4cU, 0x58585858U, 0xcfcfcfcfU,
	0xd0d0d0d0U, 0xefefefefU, 0xaaaaaaaaU, 0xfbfbfbfbU,
	0x43434343U, 0x4d4d4d4dU, 0x33333333U, 0x85858585U,
	0x45454545U, 0xf9f9f9f9U, 0x02020202U, 0x7f7f7f7fU,
	0x50505050U, 0x3c3c3c3cU, 0x9f9f9f9fU, 0xa8a8a8a8U,
	0x51515151U, 0xa3a3a3a3U, 0x40404040U, 0x8f8f8f8fU,
	0x92929292U, 0x9d9d9d9dU, 0x38383838U, 0xf5f5f5f5U,
	0xbcbcbcbcU, 0xb6b6b6b6U, 0xdadadadaU, 0x21212121U,
	0x10101010U, 0xffffffffU, 0xf3f3f3f3U, 0xd2d2d2d2U,
	0xcdcdcdcdU, 0x0c0c0c0cU, 0x13131313U, 0xececececU,
	0x5f5f5f5fU, 0x97979797U, 0x44444444U, 0x17171717U,
	0xc4c4c4c4U, 0xa7a7a7a7U, 0x7e7e7e7eU, 0x3d3d3d3dU,
	0x64646464U, 0x5d5d5d5dU, 0x19191919U, 0x73737373U,
	0x60606060U, 0x81818181U, 0x4f4f4f4fU, 0xdcdcdcdcU,
	0x22222222U, 0x2a2a2a2aU, 0x90909090U, 0x88888888U,
	0x46464646U, 0xeeeeeeeeU, 0xb8b8b8b8U, 0x14141414U,
	0xdedededeU, 0x5e5e5e5eU, 0x0b0b0b0bU, 0xdbdbdbdbU,
	0xe0e0e0e0U, 0x32323232U, 0x3a3a3a3aU, 0x0a0a0a0aU,
	0x49494949U, 0x06060606U, 0x24242424U, 0x5c5c5c5cU,
	0xc2c2c2c2U, 0xd3d3d3d3U, 0xacacacacU, 0x62626262U,
	0x91919191U, 0x95959595U, 0xe4e4e4e4U, 0x79797979U,
	0xe7e7e7e7U, 0xc8c8c8c8U, 0x37373737U, 0x6d6d6d6dU,
	0x8d8d8d8dU, 0xd5d5d5d5U, 0x4e4e4e4eU, 0xa9a9a9a9U,
	0x6c6c6c6cU, 0x56565656U, 0xf4f4f4f4U, 0xeaeaeaeaU,
	0x65656565U, 0x7a7a7a7aU, 0xaeaeaeaeU, 0x08080808U,
	0xbabababaU, 0x78787878U, 0x25252525U, 0x2e2e2e2eU,
	0x1c1c1c1cU, 0xa6a6a6a6U, 0xb4b4b4b4U, 0xc6c6c6c6U,
	0xe8e8e8e8U, 0xddddddddU, 0x74747474U, 0x1f1f1f1fU,
	0x4b4b4b4bU, 0xbdbdbdbdU, 0x8b8b8b8bU, 0x8a8a8a8aU,
	0x70707070U, 0x3e3e3e3eU, 0xb5b5b5b5U, 0x66666666U,
	0x48484848U, 0x03030303U, 0xf6f6f6f6U, 0x0e0e0e0eU,
	0x61616161U, 0x35353535U, 0x57575757U, 0xb9b9b9b9U,
	0x86868686U, 0xc1c1c1c1U, 0x1d1d1d1dU, 0x9e9e9e9eU,
	0xe1e1e1e1U, 0xf8f8f8f8U, 0x98989898U, 0x11111111U,
	0x69696969U, 0xd9d9d9d9U, 0x8e8e8e8eU, 0x94949494U,
	0x9b9b9b9bU, 0x1e1e1e1eU, 0x87878787U, 0xe9e9e9e9U,
	0xcecececeU, 0x55555555U, 0x28282828U, 0xdfdfdfdfU,
	0x8c8c8c8cU, 0xa1a1a1a1U, 0x89898989U, 0x0d0d0d0dU,
	0xbfbfbfbfU, 0xe6e6e6e6U, 0x42424242U, 0x68686868U,
	0x41414141U, 0x99999999U, 0x2d2d2d2dU, 0x0f0f0f0fU,
	0xb0b0b0b0U, 0x54545454U, 0xbbbbbbbbU, 0x16161616U,
};

static const u32 rcon[] = {
	/* for 128-bit blocks, Rijndael never uses more than 10 rcon values */
	0x01000000, 0x02000000,
	0x04000000, 0x08000000,
	0x10000000, 0x20000000,
	0x40000000, 0x80000000,
	0x1B000000, 0x36000000,
};

/***********************************************************************
 * @Function: dw_aes_set_encrypt_key
 * @Return:
 * @Descr: Expands the cipher key into the encryption key schedule
 ***********************************************************************/
static int dw_aes_set_encrypt_key(const unsigned char * user_key,
		const int bits, struct aes_key_st * key)
{
	u32 *rk;
	int i = 0;
	u32 temp;

        DBG_FN(DBG_SECURITY);

	if (!user_key || !key)
		return -1;
	if (bits != 128 && bits != 192 && bits != 256)
		return -2;

	rk = (u32*)key->rd_key;

	rk[0] = GETU32(user_key    );
	rk[1] = GETU32(user_key +  4);
	rk[2] = GETU32(user_key +  8);
	rk[3] = GETU32(user_key + 12);
	if (bits == 128) {
		while (1) {
			temp  = rk[3];
			rk[4] = rk[0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[5] = rk[1] ^ rk[4];
			rk[6] = rk[2] ^ rk[5];
			rk[7] = rk[3] ^ rk[6];
			if (++i == 10) {
				return 0;
			}
			rk += 4;
		}
	}
	rk[4] = GETU32(user_key + 16);
	rk[5] = GETU32(user_key + 20);
	if (bits == 192) {
		while (1) {
			temp = rk[ 5];
			rk[ 6] = rk[ 0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[ 7] = rk[ 1] ^ rk[ 6];
			rk[ 8] = rk[ 2] ^ rk[ 7];
			rk[ 9] = rk[ 3] ^ rk[ 8];
			if (++i == 8) {
				return 0;
			}
			rk[10] = rk[ 4] ^ rk[ 9];
			rk[11] = rk[ 5] ^ rk[10];
			rk += 6;
		}
	}
	rk[6] = GETU32(user_key + 24);
	rk[7] = GETU32(user_key + 28);
	if (bits == 256) {
		while (1) {
			temp = rk[ 7];
			rk[ 8] = rk[ 0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[ 9] = rk[ 1] ^ rk[ 8];
			rk[10] = rk[ 2] ^ rk[ 9];
			rk[11] = rk[ 3] ^ rk[10];
			if (++i == 7) {
				return 0;
			}
			temp = rk[11];
			rk[12] = rk[ 4] ^
				(te4[(temp >> 24)       ] & 0xff000000) ^
				(te4[(temp >> 16) & 0xff] & 0x00ff0000) ^
				(te4[(temp >>  8) & 0xff] & 0x0000ff00) ^
				(te4[(temp     ) & 0xff] & 0x000000ff);
			rk[13] = rk[ 5] ^ rk[12];
			rk[14] = rk[ 6] ^ rk[13];
			rk[15] = rk[ 7] ^ rk[14];

			rk += 8;
		}
	}
	return 0;
}


/***********************************************************************
 * @Function: dw_ccmp_set_key
 * @Return:
 * @Descr:
 ***********************************************************************/
static u8 dw_ccmp_set_key(struct dw_priv *priv, int id, u8 *data, int len, u8 *seq)
{
	ccmp_key_t *key;

	DBG_FN(DBG_SECURITY);

	key = &priv->aeskeys[id];

	if (len == CCMP_KEY_SIZE)
	{
		CLEAR(*key);
		key->valid = 1;
		/* Set sequence counter if given */
		if (seq)
			key->rx_pn  = dw_to_48(GET32(seq+2), GET16(seq));

		/* Store only AES key schedule for this key */
		dw_aes_set_encrypt_key(data, AES_BITS, &key->rk);
	}
	else if (len == 0) {
		key->valid = 0;
	}
	else
		return 0;

	return 1;
}


/***********************************************************************
 * @Function: dw_send_key
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_send_key(int index, unsigned long *key)
{
	DBG_FN(DBG_SECURITY);

	if (!dw_hw_aes_wait())
		goto error;

	/* Set key load mode */
	dw_iowrite32(0x14 | index, HW_AES_MODE);

	/* Write key to hw */
	dw_hw_aes_write_fifo_noswap(key, 4*44);

	return 0;

error:
	return -1;
}


/***********************************************************************
 * @Function: dw_send_tx_key
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_send_tx_key(struct dw_priv *priv, int index)
{
	ccmp_key_t *key;

	DBG_FN(DBG_SECURITY);

	if (!(priv->ieee->sec.flags & (1 << index)))
		return 0;

	/* expand key */
	if (0 == dw_ccmp_set_key(priv, index, priv->ieee->sec.keys[index], priv->ieee->sec.key_sizes[index], 0)) {
		printk(KERN_ERR "Error while expanding tx Key!!!\n");
		return -1;
	}
	key = &priv->aeskeys[index];

	return dw_send_key(index, key->rk.rd_key);
}


/***********************************************************************
 * @Function: dw_send_keys
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_send_keys(struct dw_priv *priv)
{
	ccmp_key_t *key;
	int i, len, err = 0;

	DBG_FN(DBG_SECURITY);
	REQUIRE_LOCKED(priv);

	/* Note: AES keys cannot be set for multiple times.
	 * Only set it at the first time. */
	for (i = 0; i < 4; i++) {
		key = &priv->aeskeys[i];
		len = priv->ieee->sec.key_sizes[i];
		if (!(priv->ieee->sec.flags & (1 << i))) {
			continue;
		}

		/* expand key */
		if (0 == dw_ccmp_set_key(priv, i, priv->ieee->sec.keys[i], len, 0)) {
			printk(KERN_ERR "Error while expanding Key!!!\n");
			continue;
		}

		if (dw_send_key(i, key->rk.rd_key)<0)
			err = -1;
	}
	return err;
}


/***********************************************************************
 * @Function: dw_set_hwcrypto_keys
 * @Return:
 * @Descr:
 ***********************************************************************/
static void dw_set_hwcrypto_keys(struct dw_priv *priv)
{
	int err = 0;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	switch (priv->ieee->sec.level) {
	case SEC_LEVEL_3:
		if (priv->ieee->sec.flags & SEC_ACTIVE_KEY)
			if (dw_send_tx_key(priv, priv->ieee->sec.active_key) < 0)
				err = -1;

		if (!dw_sw_aes)
			if (dw_send_keys(priv) < 0)
				err = -1;
		break;
	default:
		break;
	}
	if (err < 0)
		printk(KERN_ERR "Error while sending keys to hardware!!!\n");
}


/***********************************************************************
 * @Function: dw_ieee80211_set_security
 * @Return:
 * @Descr: set_security() callback in struct ieee80211_device
 ***********************************************************************/
static void dw_ieee80211_set_security(struct net_device *dev,
					   struct ieee80211_security *sec)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int i, flag;
	unsigned long flags;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	for (i = 0; i < 4; i++) {
		flag = 1 << i;
		if (sec->flags & (flag)) {
			priv->ieee->sec.encode_alg[i] = sec->encode_alg[i];
			priv->ieee->sec.key_sizes[i] = sec->key_sizes[i];
			if (sec->key_sizes[i] == 0)
				priv->ieee->sec.flags &= ~(flag);
			else {
				memcpy(priv->ieee->sec.keys[i], sec->keys[i],
				       sec->key_sizes[i]);
				priv->ieee->sec.flags |= (flag);
			}
		} else if (sec->level != SEC_LEVEL_1)
			priv->ieee->sec.flags &= ~(flag);
	}

	if (sec->flags & SEC_ACTIVE_KEY) {
		if (sec->active_key <= 3) {
			priv->ieee->sec.active_key = sec->active_key;
			priv->ieee->sec.flags |= SEC_ACTIVE_KEY;
		} else
			priv->ieee->sec.flags &= ~SEC_ACTIVE_KEY;
	} else
		priv->ieee->sec.flags &= ~SEC_ACTIVE_KEY;

	if ((sec->flags & SEC_AUTH_MODE) &&
	    (priv->ieee->sec.auth_mode != sec->auth_mode)) {
		priv->ieee->sec.auth_mode = sec->auth_mode;
		priv->ieee->sec.flags |= SEC_AUTH_MODE;
	}

	if (sec->flags & SEC_ENABLED && priv->ieee->sec.enabled != sec->enabled) {
		priv->ieee->sec.flags |= SEC_ENABLED;
		priv->ieee->sec.enabled = sec->enabled;
	}

	if (sec->flags & SEC_ENCRYPT)
		priv->ieee->sec.encrypt = sec->encrypt;

	if (sec->flags & SEC_LEVEL && priv->ieee->sec.level != sec->level) {
		priv->ieee->sec.level = sec->level;
		priv->ieee->sec.flags |= SEC_LEVEL;
	}

	if (!priv->ieee->host_encrypt && (sec->flags & SEC_ENCRYPT))
		dw_set_hwcrypto_keys(priv);

	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	dw_unlock(priv, flags);
}


/***********************************************************************
 * @Function: dw_softmac_set_bssid_filter
 * @Return:
 * @Descr: Set BSS mode and IDs (irq)
 ***********************************************************************/
static void dw_softmac_set_bssid_filter(struct net_device *dev, const u8 *bssid)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	struct ieee80211softmac_network *net;
	int i;
	int was_locked = spin_is_locked(&priv->lock);
	unsigned long flags = 0;
	int last_basic = 0;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	if (!was_locked)
		dw_lock(priv, flags);

	priv->beacon_ready = 0;

	/* Set BSSID in hardware */
	dw_hw_memcpy_to(HW_BSSID0, bssid, ETH_ALEN);

	if (priv->softmac->associnfo.associated) {
		/* Set SSID in hardware */
		dw_iowrite32(priv->softmac->associnfo.associate_essid.len |
				0x000e0000, HW_SSID_LEN);
		/*TODO set rates*/

		/* get ratesinfo from assocnetwork */
		net = ieee80211softmac_get_network_by_essid_locked(priv->softmac, &priv->softmac->associnfo.associate_essid);

		if (net) {
			/* provide only the subset of the rates we support */
			priv->ap_ri.count = 0;

			for (i=0; i < net->supported_rates.count; i++) {
				int j = 0;
				for (j = 0; j < ARRAY_SIZE(dw_rates); j++) {
					if ((net->supported_rates.rates[ i ] & 0x7F) == (dw_rates[ j ] & 0x7F)) {
						priv->ap_ri.rates[ priv->ap_ri.count++ ] = net->supported_rates.rates[ i ];

						break;
					}
				}
			}
		}


		dw_hw_memcpy_to(HW_SSID, priv->softmac->associnfo.associate_essid.data, priv->softmac->associnfo.associate_essid.len);
		/* Disable IBSS mode */
		//dw_iocleanbits32(HW_MAC_CONTROL, CTRL_IBSS | CTRL_BEACONTX);
	} else {
		/* reset it */
		for (i = 0; i < ARRAY_SIZE(dw_rates); i++)
			priv->ap_ri.rates[ i ] = dw_rates[ i ];
		priv->ap_ri.count = i;
	}

	/* determine equal or lower basic rate for the rate being used.
	   The acknowledge of AP is send with
	   api_ri.rates[ basics[ data_rate_index ] ]*/
	for (i = 0; i < ARRAY_SIZE(priv->tx.basics); i++) {
		if ((priv->ap_ri.rates[ i ] & IEEE80211_BASIC_RATE_MASK) == IEEE80211_BASIC_RATE_MASK)
			last_basic = i;
		priv->tx.basics[ i ] = last_basic;
	}

	priv->softmac->txrates.default_rate = BASIC_RATE_MASK(priv->ap_ri.rates[ priv->rate.index ]);
	DBG(DBG_UPDATE_RATE, "Rate set to %i", priv->softmac->txrates.default_rate);
	priv->rate.tx_data_any = priv->rate.tx_data_ack = 0; // reset counter so we work on the next time slice

	if (!was_locked)
		dw_unlock(priv, flags);

}

/***********************************************************************
 * @Function: dw_set_channel
 * @Return:
 * @Descr: Select a channel
 ***********************************************************************/
static void dw_set_channel(dw_priv_t* priv, u8 channel)
{
	DBG_FN(DBG_INTERFACE | DBG_MINOR);
	REQUIRE_LOCKED(priv);

	dw_iocleanbits32(HW_GEN_CONTROL, GEN_RXEN);
	priv->beacon_ready = 0;

	/* Set frequency divider for channel */
	dw_hw_write_rf (1, freq_table[channel].integer);
	dw_hw_write_rf (2, freq_table[channel].fraction);

#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
	/* Filter  calibration */
	dw_hw_write_rf (0, 0x25f9c);
	udelay (10);

	/* VCO calibration */
	dw_hw_write_rf (0, 0x25f9a);
	udelay (80);

	/* Mode Control - RX/TX mode */
	dw_hw_write_rf (0, 0x25f94);

	/* Allow the trannsceiver to settle in the new mode of operation */
	udelay (10);
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
	/* VCO Calibration */
	dw_hw_write_rf (0, 0x27fda);
	udelay(40);

	/* Set frequency divider for channel again */
	dw_hw_write_rf (1, freq_table[channel].integer);
	dw_hw_write_rf (2, freq_table[channel].fraction);

	/* Mode Control - RX/TX mode */
	dw_hw_write_rf (0, 0x27fd4);

	dw_hw_set_vco(channel);
#endif
	priv->channel = channel;
	dw_iosetbits32(HW_GEN_CONTROL, GEN_RXEN);
}

/***********************************************************************
 * @Function: dw_softmac_set_chan
 * @Return:
 * @Descr: Select a channel)
 ***********************************************************************/
static void dw_softmac_set_chan(struct net_device* dev, u8 channel)
{
	dw_priv_t*    priv = ieee80211softmac_priv(dev);
	//struct ieee80211softmac_device *mac = ieee80211_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	//if (IW_MODE_ADHOC == priv->ieee->iw_mode && !mac->scanning)
	//    printk("Selected channel = %d\n", channel);

	/* we really don't want to change the channel while there is something
	 * in the queue for a different channel */
	dw_tx_wait_for_idle_and_pause(priv);

	dw_lock(priv, flags);
	dw_set_channel( priv, channel);
	dw_unlock(priv, flags);

	dw_tx_continue_queue(priv);
}


/***********************************************************************
 * @Function: dw_geo_init
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_geo_init(dw_priv_t *priv)
{
	struct ieee80211_geo *geo;
	struct ieee80211_channel *chan;
	int i;
	u8 channel;
	const char *iso_country;

	DBG_FN(DBG_INIT);

	geo = kzalloc(sizeof(*geo), GFP_KERNEL);
	if (!geo)
		return -ENOMEM;

	iso_country = "XX";

	for (i = 0, channel = IEEE80211_52GHZ_MIN_CHANNEL;
	channel <= IEEE80211_52GHZ_MAX_CHANNEL; channel++) {
		chan = &geo->a[i++];
		chan->freq = dw_channel_to_freq_a(channel);
		chan->channel = channel;
	}
	geo->a_channels = i;
	for (i = 0, channel = IEEE80211_24GHZ_MIN_CHANNEL;
		channel <= IEEE80211_24GHZ_MAX_CHANNEL; channel++) {
		chan = &geo->bg[i++];
		chan->freq = dw_channel_to_freq_bg(channel);
		chan->channel = channel;
	}
	geo->bg_channels = i;
	memcpy(geo->name, iso_country, 2);
	if (0 /*TODO: Outdoor use only */)
		geo->name[2] = 'O';
	else if (0 /*TODO: Indoor use only */)
		geo->name[2] = 'I';
	else
		geo->name[2] = ' ';
	geo->name[3] = '\0';

	ieee80211_set_geo(priv->ieee, geo);
	kfree(geo);

	return 1;
}


/***********************************************************************
 * @Function: dw_get_name
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_get_name(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	strcpy(data->name, "IEEE 802.11b/g");
	return 0;
}

/***********************************************************************
 * @Function: dw_wx_get_encode
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_encode(struct net_device *dev,
				   struct iw_request_info *info,
				   union iwreq_data *data,
				   char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int err;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	err = ieee80211_wx_get_encode(priv->ieee, info, data, extra);

	return err;
}

/***********************************************************************
 * @Function: dw_wx_get_encodeext
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_encodeext(struct net_device *dev,
		struct iw_request_info *info, union iwreq_data *data,
		char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int err;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	err = ieee80211_wx_get_encodeext(priv->ieee, info, data, extra);

	return err;
}


/***********************************************************************
 * @Function: dw_wx_set_channelfreq
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_channelfreq(struct net_device *dev,
				      struct iw_request_info *info,
				      union iwreq_data *data,
				      char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	u8 channel;
	int err = -EINVAL;
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_tx_wait_for_idle_and_pause(priv);

	dw_lock(priv, flags);

	if ((data->freq.m >= 0) && (data->freq.m <= 1000)) {
		channel = data->freq.m;
	} else {
		channel = ieee80211_freq_to_channel(priv->ieee, data->freq.m);
	}
	if (ieee80211_is_valid_channel(priv->ieee, channel)) {
		dw_set_channel(priv, channel);
		err = 0;
	}

	dw_unlock(priv, flags);

	dw_tx_continue_queue(priv);

	return err;
}


/***********************************************************************
 * @Function: dw_wx_get_channelfreq
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_channelfreq(struct net_device *dev,
		struct iw_request_info *info, union iwreq_data *data,
		char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	data->freq.e = 1;
	data->freq.m = ieee80211_channel_to_freq(priv->ieee, priv->channel) * 100000;
	data->freq.flags = IW_FREQ_FIXED;
	dw_unlock(priv, flags);
	return 0;
}

/***********************************************************************
 * @Function: dw_wx_set_mode
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_mode(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);

	priv->ieee->iw_mode = data->mode;

	if (IW_MODE_ADHOC == priv->ieee->iw_mode) {
		if (priv->channel)
			priv->adhoc.channel = priv->channel;
	} else {
		dw_iocleanbits32(HW_MAC_CONTROL, CTRL_IBSS|CTRL_BEACONTX); /* Disable IBSS mode */
	}

	priv->beacon_ready = 0;
	priv->beacon_body_length=0;

	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	return 0;
}


/***********************************************************************
 * @Function: dw_wx_get_mode
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_mode(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	data->mode = priv->ieee->iw_mode;
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	return 0;
}


/***********************************************************************
 * @Function: dw_wx_get_rangeparams
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_rangeparams(struct net_device *dev,
		struct iw_request_info *info, union iwreq_data *data,
		char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	struct iw_range *range = (struct iw_range *)extra;
	const struct ieee80211_geo *geo;
	int i, j, level;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	data->data.length = sizeof(*range);
	memset(range, 0, sizeof(*range));

	/*TODO: What about 802.11b? Have to meassure!!! */
	/* 54Mb/s == ~27Mb/s payload throughput (802.11g) */
	range->throughput = 27 * 1000 * 1000;
	range->min_nwid = 0x0000;
	range->max_nwid = 0x0020;

	range->max_qual.qual = 100;
	range->max_qual.level = MAC_RSSI_MAX;
	range->max_qual.noise = 0;
	range->max_qual.updated =
		IW_QUAL_QUAL_UPDATED | IW_QUAL_LEVEL_UPDATED;

	range->avg_qual.qual    = 0;
	range->avg_qual.level   = 0;
	range->avg_qual.noise   = 0;
	range->avg_qual.updated = IW_QUAL_NOISE_INVALID;

	range->min_rts = DW_MIN_RTS_THRESHOLD;
	range->max_rts = DW_MAX_RTS_THRESHOLD;
	range->min_frag = MIN_FRAG_THRESHOLD;
	range->max_frag = MAX_FRAG_THRESHOLD;

	range->encoding_size[0] = 5;
	range->encoding_size[1] = 13;
	range->num_encoding_sizes = 2;
	range->max_encoding_tokens = WEP_KEYS;

	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 19;

	range->enc_capa = IW_ENC_CAPA_WPA |
			  IW_ENC_CAPA_WPA2 |
			  IW_ENC_CAPA_CIPHER_TKIP |
			  IW_ENC_CAPA_CIPHER_CCMP;

	for (i = 0; i < ARRAY_SIZE(dw_rates); i++)
		range->bitrate[ i ] = RATE_IN_BS(BASIC_RATE_MASK(dw_rates[ i ]));
	range->num_bitrates = i;

	geo = ieee80211_get_geo(priv->ieee);
	range->num_channels = geo->bg_channels;
	j = 0;
	for (i = 0; i < geo->bg_channels; i++) {
		if (j == IW_MAX_FREQUENCIES)
			break;
		range->freq[j].i = j + 1;
		range->freq[j].m = geo->bg[i].freq * 100000;
		range->freq[j].e = 1;
		j++;
	}
	range->num_frequency = j;

	/* retry limit capabilities */
	range->retry_capa = IW_RETRY_LIMIT | IW_RETRY_LIFETIME;
	range->retry_flags = IW_RETRY_LIMIT;
	range->r_time_flags = IW_RETRY_LIFETIME;

	/* I don't know the range.*/
	range->min_retry = 1;
	range->max_retry = 65535;
	range->min_r_time = 1024;
	range->max_r_time = 65535 * 1024;

	/* txpower is supported in dBm's */
//	if (priv->ieee->iw_mode == IW_MODE_ADHOC) {
		range->txpower_capa = IW_TXPOW_DBM;
		range->num_txpower = IW_MAX_TXPOWER;
		for (i = 0, level = (DW_TX_POWER_MAX_DBM * 16);
				i < IW_MAX_TXPOWER;
				i++, level -=
				((DW_TX_POWER_MAX_DBM -
				DW_TX_POWER_MIN_DBM) * 16) / (IW_MAX_TXPOWER - 1))
			range->txpower[i] = level / 16;
//	} else {
//		range->txpower_capa = 0;
//		range->num_txpower = 0;
//	}
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);

	return 0;
}


/***********************************************************************
 * @Function: dw_get_wireless_stats
 * @Return:
 * @Descr: Get wireless statistics.  Called by /proc/net/wireless and by SIOCGIWSTATS
 ***********************************************************************/
static struct iw_statistics *dw_get_wireless_stats(struct net_device *dev)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	struct ieee80211softmac_device *mac = ieee80211_priv(dev);
	struct iw_statistics *wstats;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	wstats = &priv->wstats;
	if (!mac->associnfo.associated) {
		wstats->miss.beacon = 0;
		wstats->discard.retries = 0;
		wstats->discard.nwid = 0;
		wstats->discard.code = 0;
		wstats->discard.fragment = 0;
		wstats->discard.misc = 0;
		wstats->qual.qual = 0;
		wstats->qual.level = 0;
		wstats->qual.noise = 0;
		wstats->qual.updated = IW_QUAL_NOISE_INVALID |
			IW_QUAL_QUAL_INVALID | IW_QUAL_LEVEL_INVALID;
	} else {
		spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
		/* fill in the real statistics when iface associated */
		wstats->qual.qual =
					(100 *
					(priv->ieee->perfect_rssi - priv->ieee->worst_rssi) *
					(priv->ieee->perfect_rssi - priv->ieee->worst_rssi) -
					(priv->ieee->perfect_rssi - priv->ieee->networks->stats.rssi) *
					(15 * (priv->ieee->perfect_rssi - priv->ieee->worst_rssi) +
					62 * (priv->ieee->perfect_rssi -
					priv->ieee->networks->stats.rssi))) /
					((priv->ieee->perfect_rssi -
					priv->ieee->worst_rssi) * (priv->ieee->perfect_rssi -
					priv->ieee->worst_rssi));
		/*
		 * !TODO. Seems they are not calculated. Take
		 * one from the last frame */
		wstats->qual.level = priv->ieee->networks->stats.signal;
		wstats->qual.noise = priv->ieee->networks->stats.noise;
		wstats->qual.updated = IW_QUAL_QUAL_UPDATED | IW_QUAL_LEVEL_UPDATED;
		wstats->discard.code = priv->ieee->ieee_stats.rx_discards_undecryptable;
		wstats->discard.retries = priv->ieee->ieee_stats.tx_retry_limit_exceeded;
		wstats->discard.nwid = priv->ieee->ieee_stats.tx_discards_wrong_sa;
		wstats->discard.fragment = priv->ieee->ieee_stats.rx_fragments;
		wstats->discard.misc = priv->wstats.discard.misc;
		wstats->miss.beacon = 0;	/* FIXME */
		spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	}
	return wstats;
}


/***********************************************************************
 * @Function: dw_wx_set_nick
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_nick(struct net_device *dev,
			       struct iw_request_info *info,
			       union iwreq_data *data,
			       char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	size_t len;
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	len =  min((size_t)data->data.length, (size_t)IW_ESSID_MAX_SIZE);
	memcpy(priv->nick, extra, len);
	priv->nick[len] = '\0';
	dw_unlock(priv, flags);

	return 0;
}


/***********************************************************************
 * @Function: dw_wx_get_nick
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_nick(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	size_t len;
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	len = strlen(priv->nick) + 1;
	memcpy(extra, priv->nick, len);
	data->data.length = (__u16)len;
	dw_unlock(priv, flags);

	return 0;
}


/***********************************************************************
 * @Function: dw_wx_set_rts
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_rts(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int err = -EINVAL;
	unsigned long flags;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	if (data->rts.disabled) {
		priv->ieee->rts = DW_MAX_RTS_THRESHOLD;
		err = 0;
	} else {
		if (data->rts.value >= DW_MIN_RTS_THRESHOLD &&
		    data->rts.value <= DW_MAX_RTS_THRESHOLD) {
			priv->ieee->rts = data->rts.value;
			err = 0;
		}
	}
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	dw_unlock(priv, flags);

	return err;
}


/***********************************************************************
 * @Function: dw_wx_get_rts
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_rts(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *data, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	data->rts.value = priv->ieee->rts;
	data->rts.fixed = 0;
	data->rts.disabled = (priv->ieee->rts == DW_MAX_RTS_THRESHOLD);
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	dw_unlock(priv, flags);

	return 0;
}


/***********************************************************************
 * @Function: dw_wx_set_rate
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_rate(struct net_device *dev,
			      struct iw_request_info *info,
			      union iwreq_data *data,
			      char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	s32 in_rate = data->bitrate.value;
	int err = 0;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);

	if (in_rate == -1)
		/* auto */
		atomic_set(&priv->fix_rate, 0);
	else
		atomic_set(&priv->fix_rate, 1);

	err = ieee80211softmac_wx_set_rate(dev, info, data, extra);

	dw_unlock(priv, flags);

	return err;
}


/***********************************************************************
 * @Function: dw_wx_set_frag
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_frag(struct net_device *dev,
			       struct iw_request_info *info,
			       union iwreq_data *data,
			       char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int err = -EINVAL;
	unsigned long flags;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	if (data->frag.disabled) {
		priv->ieee->fts = MAX_FRAG_THRESHOLD;
		err = 0;
	} else {
		if (data->frag.value >= MIN_FRAG_THRESHOLD &&
		    data->frag.value <= MAX_FRAG_THRESHOLD) {
			priv->ieee->fts = data->frag.value & ~0x1;
			err = 0;
		}
	}
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	dw_unlock(priv, flags);

	return err;
}


/***********************************************************************
 * @Function: dw_wx_get_frag
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_frag(struct net_device *dev,
			       struct iw_request_info *info,
			       union iwreq_data *data,
			       char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	unsigned long ieeeflags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	spin_lock_irqsave(&priv->ieee->lock, ieeeflags);
	data->frag.value = priv->ieee->fts;
	data->frag.fixed = 0;
	data->frag.disabled = (priv->ieee->fts == MAX_FRAG_THRESHOLD);
	spin_unlock_irqrestore(&priv->ieee->lock, ieeeflags);
	dw_unlock(priv, flags);

	return 0;
}

/***********************************************************************
 * @Function: dw_wx_set_xmitpower
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_xmitpower(struct net_device *dev,
				    struct iw_request_info *info,
				    union iwreq_data *data,
				    char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	int err = 0, value;
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	if ((data->txpower.flags & IW_TXPOW_TYPE) != IW_TXPOW_DBM) return -EINVAL;
	if (data->txpower.fixed == 0) return -EINVAL;
	if (data->txpower.value < DW_TX_POWER_MIN_DBM || data->txpower.value > DW_TX_POWER_MAX_DBM) return -EINVAL;

	value = data->txpower.value;
	dw_lock(priv, flags);
	err = dw_set_tx_power(priv, value);
	dw_unlock(priv, flags);
	return 0;
}


/***********************************************************************
 * @Function: dw_wx_get_xmitpower
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_xmitpower(struct net_device *dev,
		struct iw_request_info *info, union iwreq_data *data,
		char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	data->txpower.fixed = 1;
	data->txpower.value = priv->tx_power;
	data->txpower.flags = IW_TXPOW_DBM;
	return 0;
}


/***********************************************************************
 * @Function: dw_wx_set_retry
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_set_retry(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *wrqu, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	if (wrqu->retry.flags & IW_RETRY_LIFETIME || wrqu->retry.disabled)
		return -EINVAL;

	if (!(wrqu->retry.flags & IW_RETRY_LIMIT))
		return 0;

	if (wrqu->retry.value < 0 || wrqu->retry.value > 255)
		return -EINVAL;

	dw_lock(priv, flags);
	if (wrqu->retry.flags & IW_RETRY_MIN)
		priv->short_retry_limit = (u8) wrqu->retry.value;
	else if (wrqu->retry.flags & IW_RETRY_MAX)
		priv->long_retry_limit = (u8) wrqu->retry.value;
	else {
		priv->short_retry_limit = (u8) wrqu->retry.value;
		priv->long_retry_limit = (u8) wrqu->retry.value;
	}

	dw_unlock(priv, flags);
	return 0;
}


/***********************************************************************
 * @Function: dw_wx_get_retry
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_wx_get_retry(struct net_device *dev, struct iw_request_info *info,
		union iwreq_data *wrqu, char *extra)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	wrqu->retry.disabled = 0;

	if ((wrqu->retry.flags & IW_RETRY_TYPE) == IW_RETRY_LIFETIME) {
		dw_unlock(priv, flags);
		return -EINVAL;
	}

	if (wrqu->retry.flags & IW_RETRY_MAX) {
		wrqu->retry.flags = IW_RETRY_LIMIT | IW_RETRY_MAX;
		wrqu->retry.value = priv->long_retry_limit;
	} else if (wrqu->retry.flags & IW_RETRY_MIN) {
		wrqu->retry.flags = IW_RETRY_LIMIT | IW_RETRY_MIN;
		wrqu->retry.value = priv->short_retry_limit;
	} else {
		wrqu->retry.flags = IW_RETRY_LIMIT;
		wrqu->retry.value = priv->short_retry_limit;
	}
	dw_unlock(priv, flags);

	return 0;
}

#ifdef WX
# undef WX
#endif
#define WX(ioctl)  [(ioctl) - SIOCSIWCOMMIT]
static const iw_handler dw_wx_handlers[] = {
	/* Wireless Identification */
	WX(SIOCGIWNAME)		= dw_get_name,
	/* Basic operations */
	WX(SIOCSIWFREQ)		= dw_wx_set_channelfreq,
	WX(SIOCGIWFREQ)		= dw_wx_get_channelfreq,
	WX(SIOCSIWMODE)		= dw_wx_set_mode,
	WX(SIOCGIWMODE)		= dw_wx_get_mode,
	/* Informative stuff */
	WX(SIOCGIWRANGE)	= dw_wx_get_rangeparams,
	/* Spy support (statistics per MAC address - used for Mobile IP support) */
	WX(SIOCSIWSPY)		= iw_handler_set_spy,
	WX(SIOCGIWSPY)		= iw_handler_get_spy,
	WX(SIOCSIWTHRSPY)	= iw_handler_set_thrspy,
	WX(SIOCGIWTHRSPY)	= iw_handler_get_thrspy,
	/* Access Point manipulation */
	WX(SIOCSIWAP)		= ieee80211softmac_wx_set_wap,
	WX(SIOCGIWAP)		= ieee80211softmac_wx_get_wap,
	WX(SIOCSIWSCAN)		= ieee80211softmac_wx_trigger_scan,
	WX(SIOCGIWSCAN)		= ieee80211softmac_wx_get_scan_results,
	/* 802.11 specific support */
	WX(SIOCSIWESSID)	= ieee80211softmac_wx_set_essid,
	WX(SIOCGIWESSID)	= ieee80211softmac_wx_get_essid,
	WX(SIOCSIWNICKN)	= dw_wx_set_nick,
	WX(SIOCGIWNICKN)	= dw_wx_get_nick,
	/* Other parameters */
	WX(SIOCSIWRATE)		= dw_wx_set_rate,
	WX(SIOCGIWRATE)		= ieee80211softmac_wx_get_rate,
	WX(SIOCSIWRTS)		= dw_wx_set_rts,
	WX(SIOCGIWRTS)		= dw_wx_get_rts,
	WX(SIOCSIWFRAG)		= dw_wx_set_frag,
	WX(SIOCGIWFRAG)		= dw_wx_get_frag,
	WX(SIOCSIWTXPOW)	= dw_wx_set_xmitpower,
	WX(SIOCGIWTXPOW)	= dw_wx_get_xmitpower,
	WX(SIOCSIWRETRY)	= dw_wx_set_retry,
	WX(SIOCGIWRETRY)	= dw_wx_get_retry,
	/* Encoding */
	WX(SIOCSIWENCODE)	= dw_wx_set_encode,
	WX(SIOCGIWENCODE)	= dw_wx_get_encode,
	WX(SIOCSIWENCODEEXT)	= dw_wx_set_encodeext,
	WX(SIOCGIWENCODEEXT)	= dw_wx_get_encodeext,
	/* Power saving */
/*TODO	WX(SIOCSIWPOWER)	= dw_wx_set_power, */
/*TODO	WX(SIOCGIWPOWER)	= dw_wx_get_power, */
	WX(SIOCSIWGENIE)	= ieee80211softmac_wx_set_genie,
	WX(SIOCGIWGENIE)	= ieee80211softmac_wx_get_genie,
	WX(SIOCSIWMLME)		= ieee80211softmac_wx_set_mlme,
	WX(SIOCSIWAUTH)		= ieee80211_wx_set_auth,
	WX(SIOCGIWAUTH)		= ieee80211_wx_get_auth,
};
#undef WX

#ifdef NEED_PRIVATE_HANDLER
# include "dw_priv_handler.c"
#else
static const iw_handler dw_priv_wx_handlers[] = {};
static const struct iw_priv_args dw_priv_wx_args[] = {};
#endif

const struct iw_handler_def dw_wx_handlers_def = {
	.standard		= dw_wx_handlers,
	.num_standard		= ARRAY_SIZE(dw_wx_handlers),
	.num_private		= ARRAY_SIZE(dw_priv_wx_handlers),
	.num_private_args	= ARRAY_SIZE(dw_priv_wx_args),
	.private		= dw_priv_wx_handlers,
	.private_args		= dw_priv_wx_args,
	.get_wireless_stats	= dw_get_wireless_stats,
};

/***********************************************************************
 * @Function: dw_close
 * @Return:
 * @Descr:
 ***********************************************************************/
static int dw_close(struct net_device* dev)
{
	struct dw_priv* priv = ieee80211softmac_priv(dev);
	unsigned long   flags;
	struct list_head*  it;

	DBG_FN(DBG_INIT);

	ieee80211softmac_stop(dev);

	/* don't disable anything before softmac_stop. Maybe SoftMAC will run a
	 * disassoc cycle somewhat later. */
	del_timer_sync(&priv->management_timer);

	cancel_delayed_work(&priv->beacon_work);

	disable_irq(dev->irq);

	tasklet_disable(&priv->rx.tasklet);
	/* from this point, rx queue.filled is empty, and it is not paused */
	tasklet_disable(&priv->tx.tasklet);

	dw_lock(priv, flags);

	priv->tx.data_pending_ack = NULL;

	/* delete unsend frames */
	list_for_each(it, &priv->tx.queued) {
		dw_frame_tx_t* frame = list_entry(it, dw_frame_tx_t, list);
		ieee80211_txb_free(frame->s.txb);
		frame->s.txb = NULL;
	}
	dw_list_move(&priv->tx.free, &priv->tx.queued);

	dw_iocleanbits32(HW_GEN_CONTROL, GEN_RXEN);
	dw_iowrite32(0, HW_INTR_MASK);
	dw_iowrite32(dw_ioread32(HW_INTR_STATUS), HW_INTR_STATUS);

	dw_set_led_on(PIN_LED, 0);
	dw_set_led_on(PIN_ACTIVITY_LED, 0);

	dw_unlock(priv, flags);

	return 0;
}

/***********************************************************************
 * @Function: dw_open
 * @Return:
 * @Descr: init/reinit
 ***********************************************************************/
static int dw_open(struct net_device* dev)
{
	struct dw_priv* priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INIT);

	dw_reset_rx_dups(priv);

	priv->tx.data_pending_ack = NULL;

	priv->jiffies_last_beacon = jiffies;
	priv->reconnection_attempts = 0;

	tasklet_enable(&priv->rx.tasklet);
	tasklet_enable(&priv->tx.tasklet);

	ieee80211softmac_start(dev);

	ieee80211softmac_set_rates(dev, ARRAY_SIZE(dw_rates), (u8*) dw_rates); /* cast away const */

	/* softmac sets user_rate to 24MB, but we start much slower to work
	   always */
	priv->softmac->txrates.default_rate = priv->softmac->txrates.user_rate = IEEE80211_CCK_RATE_1MB;

	dw_lock(priv, flags);

	priv->beacon_ready = 0;
	INIT_DELAYED_WORK(&priv->beacon_work, dw_beacon_start);
	priv->beacon_body_length=0;
	priv->adhoc.channel = DW_IBSS_DEFAULT_CHANNEL;

	/* acknowledge pending interrupts */
	dw_iowrite32(dw_ioread32(HW_INTR_STATUS), HW_INTR_STATUS);
	dw_iowrite32(INTR_ALL,       HW_INTR_MASK);
	dw_iowrite32(HW_AES_MODE_1,  HW_AES_MODE );
	mod_timer(&priv->management_timer, jiffies + MANAGEMENT_JIFFIES);

	enable_irq(dev->irq);

	dw_iosetbits32(HW_GEN_CONTROL, GEN_RXEN);

	dw_unlock(priv, flags);

	return 0;
}

/***********************************************************************
 * @Function: dw_set_multicast_list
 * @Return:
 * @Descr:
 ***********************************************************************/
static void dw_set_multicast_list(struct net_device* dev)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);
	unsigned long flags;

	DBG_FN(DBG_INTERFACE | DBG_MINOR);

	dw_lock(priv, flags);
	if (dev->flags & IFF_PROMISC) {
		dw_iosetbits32(HW_MAC_CONTROL, CTRL_PROMISC);
	} else {
		dw_iocleanbits32(HW_MAC_CONTROL, CTRL_PROMISC);
	}
	dw_unlock(priv, flags);
}

/***********************************************************************
 * @Function: dw_net_get_stats
 * @Return:
 * @Descr:
 ***********************************************************************/
static struct net_device_stats * dw_net_get_stats(struct net_device *dev)
{
	struct dw_priv *priv = ieee80211softmac_priv(dev);

	DBG_FN(DBG_INTERFACE);

	return &priv->ieee->stats;
}

static const struct net_device_ops dw_netdev_ops = {
	.ndo_open               = dw_open,
	.ndo_stop               = dw_close,
	.ndo_start_xmit		= ieee80211_xmit,
	.ndo_set_multicast_list = dw_set_multicast_list,
	.ndo_get_stats		= dw_net_get_stats,
};

/**
 * dw_probe - probe for resources and for chip being present
 *
 * @return <0 on failure
 */
static int __devinit dw_probe(struct platform_device* pdev)
{
	struct net_device* dev = NULL;
	int err = -ENODEV;

	DBG_FN(DBG_INIT);

	/* Create the network device object. */
	dev = alloc_ieee80211softmac(sizeof(dw_priv_t));
	if (NULL == dev) {
		ERROR(":  Couldn't alloc_ieee80211softmac ");
		err = -ENOMEM;
		goto error_alloc;
	}
	platform_set_drvdata(pdev, dev);
	dev->irq  = INTR_ID;

	if (dev_alloc_name(dev, "wlan%d") < 0) {
		ERROR("Couldn't get name");
		goto error_name;
	}

	/* allocate resources and map memory */
	if (gpio_request(PIN_LED, DRIVER_NAME) != 0) {
		ERROR("GPIO %i already in use", PIN_LED);
		goto error_pin_led;
	}
	if (gpio_request(PIN_ACTIVITY_LED, DRIVER_NAME) != 0) {
		ERROR("GPIO %i already in use", PIN_ACTIVITY_LED);
		goto error_pin_activity_led;
	}
	if (gpio_request(PIN_INTR, DRIVER_NAME) != 0) {
		ERROR("GPIO %i already in use", PIN_INTR);
		goto error_pin_intr;
	}

	err = request_resource(&iomem_resource, pdev->resource);
	if (err) {
		ERROR("Memory already in used: 0x%08x...0x%08x",
				pdev->resource->start, pdev->resource->end);
		goto error_res;
	}

	vbase = ioremap(pdev->resource->start,
			pdev->resource->end - pdev->resource->start);
	if (NULL == vbase) {
                ERROR("ioremap failed");
                err = -ENOMEM;
                goto error_remap;
        }
        dev->base_addr = (int) vbase;

        /* test if fpga is programmed */
        if (dw_ioread32(HW_VERSION) == 0xffffffff) {
                ERROR("FPGA not present");
                goto error_fpga;
        }

        /* all resources available, initializw remaining stuff */
        dev->wireless_handlers  = &dw_wx_handlers_def;
        dev->netdev_ops = &dw_netdev_ops;

        /* initialize hardware/private stuff */
        err = dw_start_dev(pdev);
        if (err)
                goto error;

        return 0;

error_fpga:
        iounmap(vbase);
        vbase = NULL;
error_remap:
        release_resource(pdev->resource);
error_res:
        gpio_free(PIN_INTR);
error_pin_intr:
        gpio_free(PIN_ACTIVITY_LED);
error_pin_activity_led:
        gpio_free(PIN_LED);
error_pin_led:
error_name:
        /* the IRQ hasn't been setup, so unwind a part and don't go to
         * dw_remove */
        free_ieee80211softmac(dev);
error_alloc:
        return err;

error:
        /* dev memory is setup, so we can use the remove function */
        dw_remove(pdev);

        return err;
}

/***********************************************************************
 * @Function: dw_init_module
 * @Return: 0 if successfull; -ENODEV if interrupt or device is not available
 * @Descr: initialize module
 ***********************************************************************/
static int __init dw_init_module(void)
{
	int err;

        DBG_FN(DBG_INIT);

	printk(KERN_INFO "%s\n", dw_version);

        if (dw_sw_aes)
                printk(KERN_NOTICE "AES encoding/decoding is done in software\n");

        err = platform_device_register(&dw_device);
        if (err) {
                ERROR("Device Register Failed");
                goto error;
        }

        err = platform_driver_register(&dw_driver);
        if (err) {
                ERROR("Driver Register Failed");
                goto error;
        }

        return 0;

error:
        return err;
}


/***********************************************************************
 * @Function: dw_cleanup_module
 * @Return: nothing
 * @Descr: deinitializes module. Actually will never be called due to
 * MOD_INC_USE_COUNT > 1. Will be fixed if hardware watchdog can be disabled.
 ***********************************************************************/
static void __exit dw_cleanup_module(void)
{
        DBG_FN(DBG_INIT);

	platform_driver_unregister(&dw_driver);
	platform_device_unregister(&dw_device);
}

#ifndef MODULE
/**
 * dw_sw_cmdline - parses kernel commandline
 *
 * @return always 1
 */
static int __init dw_sw_cmdline(char* arg)
{
        if ((NULL != arg) && *arg)
                dw_sw_aes = simple_strtoul(arg, NULL, 10);

        return 1;
}
#endif  /* MODULE */

module_init(dw_init_module);
module_exit(dw_cleanup_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bernd Westermann");
MODULE_AUTHOR("Markus Pietrek");
MODULE_AUTHOR("Miriam Ruiz");
MODULE_DESCRIPTION("WiFi driver for ConnectCore Wi-9C");

module_param(dw_sw_aes, int, 0x00);
MODULE_PARM_DESC(dw_sw_aes, "If set, AES encoding/decoding is done in software.");

module_param(dw_cfg_vco, int, 0x00);
MODULE_PARM_DESC(dw_cfg_vco, "Config Value of VCO (0 use built-in)");

#ifdef CONFIG_DIGI_WI_G_DEBUG
module_param(dw_dbg_level, int, 0x00);
MODULE_PARM_DESC(dw_dbg_level, "debug output level");
#endif /* CONFIG_DIGI_WI_G_DEBUG */

#ifndef MODULE
__setup("dw_sw_aes=", dw_sw_cmdline);
#endif /* MODULE */
