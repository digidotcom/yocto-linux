/*******************************************************************************
 * digi_wi_g.h
 *
 * Support for DIGI Wireless Module.
 *
 * $Id: digi_wi_g.h,v 1.64 2008-02-13 11:02:34 mruiz Exp $
 * @Author: Bernd Westermann
 * @References: [1] NET+OS code mac_hw_wi9c.c
 *
 * Copyright (C) 2007 by FS Forth-Systeme GmbH
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *******************************************************************************
 *  History:
 *     14/03/06	Initial Version
 *
 *******************************************************************************/

#ifndef FS_DIGI_WI_G_DRIVER_H
#define FS_DIGI_WI_G_DRIVER_H

#include <linux/netdevice.h>
#include <net/ieee80211.h>
#include <net/ieee80211softmac.h>
#include <net/ieee80211softmac_wx.h>

#include <net/iw_handler.h>     /* New driver API */
#include <mach/regs-bbu.h>
#include <mach/hardware.h>	/* BBUS_CLK_FREQ */
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>

#ifdef dprintkl
#undef dprintkl
#endif
#ifdef dprintk
#undef dprintk
#endif


#define DRIVER_MAJOR 	0
#define DRIVER_MINOR 	60
#define DRIVER_NAME  	"digi_wi_g"

/* for LEDs, update rate etc. */
/* LED time is 50ms */
#define MANAGEMENT_TIME_MS       50
#define	MANAGEMENT_JIFFIES	 ( ( MANAGEMENT_TIME_MS * HZ ) / 1000 )
/* update rate only every 500ms */
#define MANAGEMENT_TICKS_FOR_UPDATE ( 500 / MANAGEMENT_TIME_MS )
#define TX_TIMEOUT		(4*HZ/2)
#define ACK_TIMEOUT		(HZ/2)
#define BEACON_TIMEOUT	(HZ)
#define RESCAN_TIMEOUT	(15*HZ)

#define MAX_RECONNECTION_ATTEMPTS 5

/* Min/max successful intervals to increase rate */
#define	THRESH_MIN		1
#define	THRESH_MAX		10

#define TX_QUEUE_SIZE            8  /* not tuned yet */
#define TX_MAX_FRAGS            16
#define RX_QUEUE_SIZE           44  /* one ICMP frame */

#define RX_DUP_SENDER_SIZE      16  /* somewhat lower than MAX_NETWORK_COUNT
                                     * because in worst case we need to scan
                                     * RX_DUP_SENDER_SIZE until we found the
                                     * active one */

#define DW_PIO_MAXTXPACKETS      4

#define DW_DEFAULT_SHORT_RETRY_LIMIT	7
#define DW_DEFAULT_LONG_RETRY_LIMIT	4
#define AES_BUSY_TIMEOUT	1000000 /* in ns */

#define printkl(f, x...)  do { if (printk_ratelimit()) printk(f ,##x); } while (0)

#ifdef CONFIG_DIGI_WI_G_DEBUG
# define assert(expr) \
	do {	\
		if (unlikely(!(expr))) {	\
		printk(KERN_ERR PFX "ASSERTION FAILED (%s) \nat: %s:%d:%s()\n",	\
			#expr, __FILE__, __LINE__, __FUNCTION__);	\
		}						\
	} while (0)
# define dprintkl		printkl
# define dprintk(f, x...)  do { printk(f ,##x); } while (0)
# define COMPILE_TIME " from ("__TIME__ ") "
#else
# define assert(expr)	do { /* nothing */ } while (0)
# define dprintkl(f, x...)	do { /* nothing */ } while (0)
# define dprintk(f, x...)  do { /* nothing */ } while (0)
# define COMPILE_TIME ""
#endif  /* CONFIG_DIGI_WI_G_DEBUG */

#define	DIGI_WIRELESS_G_REG_PHYS	NS9XXX_CSxSTAT_PHYS(2)

/* Hardware register defines */
#define	MAC_BASE_PHYS	DIGI_WIRELESS_G_REG_PHYS
#define MAC_BASE_SIZE	0xe0000/*100U*/			/* Register segment size */
#define MAC_MASK	0xffffc001			/* Size mask and enable bit */

/* Baseband control registers */
#define	HW_VERSION		(0x00)	/* Version */
#define	HW_GEN_CONTROL		(0x04)	/* General control */
#define	HW_GEN_STATUS		(0x08)	/* General status */
#define	HW_RSSI_AES		(0x0c)	/* RSSI and AES status */
#define	HW_INTR_MASK		(0x10)	/* Interrupt mask */
#define	HW_INTR_STATUS		(0x14)	/* Interrupt status */
#define	HW_SPI_DATA		(0x18)	/* RF SPI data register */
#define	HW_SPI_CONTROL		(0x1c)	/* RF SPI control register */
#define	HW_DATA_FIFO		(0x20)	/* Data FIFO */
#define	HW_AES_FIFO		(0x30)	/* AES FIFO */
#define	HW_AES_MODE		(0x38)	/* AES mode */

/* MAC control registers */
#define	HW_STAID0		(0x40)	/* Station ID (6 bytes) */
#define	HW_STAID1		(0x44)
#define	HW_BSSID0		(0x48)	/* BSS ID (6 bytes) */
#define	HW_BSSID1		(0x4c)
#define	HW_SSID_LEN		(0x50)	/* SSID length (8 bits) */
#define	HW_BACKOFF		(0x54)	/* Backoff period (16 bits) */
#define	HW_LISTEN		(0x58)	/* Listen interval (16 bits), CFP (8 bits), DTIM (8 bits) */
#define	HW_CFP_ATIM		(0x5c)	/* CFP max duration/ATIM period (16 bits), beacon interval (16 bits) */
#define	HW_MAC_STATUS		(0x60)	/* MAC status (8 bits) */
#define	HW_MAC_CONTROL		(0x64)	/* MAC control (8 bits) */
#define	HW_REMAIN_BO		(0x68)	/* Remaining backoff (16 bits) */
#define	HW_BEACON_BO		(0x6c)	/* Beacon backoff (16 bits), beacon mask (8 bits) */
#define	HW_SSID			(0x80)	/* Service set ID (32 bytes) */

/* FIFO sizes in bytes */
#define	HW_TX_FIFO_SIZE	1792
#define	HW_RX_FIFO_SIZE	2048


/* General control register bits */
#define	GEN_RXEN	0x00000001	/* Receive enable */
#define	GEN_ANTDIV	0x00000002	/* Antenna diversity */
#define	GEN_ANTSEL	0x00000004	/* Antenna select */
#define	GEN_5GEN	0x00000008	/* 5 GHz band enable */
#define	GEN_SHPRE	0x00000010	/* Transmit short preamble */
#define	GEN_RXFIFORST	0x00000020	/* Receive FIFO reset */
#define	GEN_TXFIFORST	0x00000040	/* Transmit FIFO reset */
#define	GEN_TXHOLD	0x00000080	/* Transmit FIFO hold */
#define	GEN_BEACEN	0x00000100	/* Beacon enable */
#define	GEN_BST		0x00001000	/* Boot status */
#define	GEN_CLKEN	0x00002000	/* Clock enable */
#define	GEN_TXFIFOEMPTY	0x00004000	/* Transmit FIFO empty */
#define	GEN_TXFIFOFULL	0x00008000	/* Transmit FIFO full */

#if defined(CONFIG_DIGI_WI_G_UBEC_JD)
# define GEN_INIT	0x377a0000	/* Initial state */
#elif defined(CONFIG_DIGI_WI_G_UBEC_HC)
# define GEN_INIT	0x37700000	/* Initial state */
#else
# error "You need to choose an UBEC Transceiver Revision"
#endif

/* General status register bits */
#define	STAT_RXFE	0x00000010	/* Receive FIFO empty */
#define SFT_RESET       0x04000000      /* WiFi Baseband soft reset */
/* AES status register bits */
#define	AES_EMPTY	0x00010000	/* AES receive FIFO empty */
#define	AES_FULL	0x00020000	/* AES transmit FIFO full */
#define	AES_BUSY	0x00040000	/* AES engine busy */
#define	AES_MIC		0x00080000	/* AES MIC correct */

/* Interrupt mask and status register bits */
#define INTR_RXFIFO	0x00000001	/* Receive FIFO not empty */
#define INTR_TXEND	0x00000002	/* Transmit complete */
#define INTR_TIMEOUT	0x00000004	/* CTS/ACK receive timeout */
#define INTR_ABORT	0x00000008	/* CTS transmit abort */
#define INTR_TBTT	0x00000010	/* Beacon transmission time */
#define INTR_ATIM	0x00000020	/* ATIM interval end */
#define INTR_RXOVERRUN	0x00000040	/* Receive FIFO overrun */
#define INTR_ALL	( INTR_RXFIFO    | \
                          INTR_TIMEOUT   | \
                          INTR_RXOVERRUN | \
                          INTR_TXEND )

/* MAC control register bits */
#define	CTRL_TXREQ	0x00000001	/* Transmit request */
#define	CTRL_AUTOTXDIS	0x00000002	/* Auto-transmit disable */
#define	CTRL_BEACONTX	0x00000004	/* Beacon transmit enable */
#define	CTRL_PROMISC	0x00000008	/* Promiscuous mode */
#define	CTRL_IBSS	0x00000010	/* IBBS mode */

#define HW_AES_MODE_0   0x00000000
#define HW_AES_MODE_1   0x00000010

/* BOOTMUX bit */
#define	BOOTMUX_LOW	0x00000001	/* Bootmux,bit 0, must go low after load */

/* PIO pins */
#define PIN_INIT		58		/* FPGA program init */
#define PIN_INTR		65		/* Interrupt (same as done) */
#define PIN_LED			67		/* Link status LED */
#define PIN_ACTIVITY_LED	66		/* Link activity LED */
#define PIN_BOOTMUX		66		/* Enables serial ports */

#define INTR_ID		IRQ_NS9XXX_ETHPHY	/* Interrupt ID for PIN_INTR */

/* Get/set/clear a PIO pin */
#define PIO_SET(pin)	ns9xxx_gpio_setpin(pin, 1)
#define PIO_CLR(pin)	ns9xxx_gpio_setpin(pin, 0)

#define	CW_MIN		31	/* Min contention window size */
#define	CW_MAX		1023	/* Max contention window size */

/* Maximum rxsignal strength we expect to see (this is in dB) */
#define MAC_RSSI_MAX 0x4f

/* the CCK/OFDM base is 2 * Mb/s */
#define RATE_IN_BS(x) ((x) * 500000)

/* see email from Mike Schaffner from 16.03.07 (RE: Verification of
 * initialization to mpietrek and hbujanda */
#define VCO_DEFAULT_CHANNEL_12 	0x7160
#define VCO_DEFAULT		0x7020

#define RATES_SUPPORTED		12

#define AES_MAXNR 14

#define CCMP_KEY_SIZE	16			/* CCMP key size */
#define AES_BLOCK_SIZE	16
#define	EXTIV_SIZE	8			/* IV and extended IV size */
#define	MIC_SIZE	8			/* Message integrity check size */
#define	CCMP_SIZE	(EXTIV_SIZE+MIC_SIZE)	/* Total CCMP size */
#define FCS_SIZE 4 // FCS (CRC-32) size
#define	DATA_SIZE	28			/* Data frame header+FCS size */
/* Key ID byte in data frame body */
#define	EXT_IV		0x20	/* Extended IV is present */

#define DW_TX_POWER_MIN_DBM 0
#define DW_TX_POWER_MAX_DBM 15
#define DW_TX_POWER_DEFAULT 10

typedef u64 u48;

struct aes_key_st {
    unsigned long rd_key[4 *(AES_MAXNR + 1)];
};

typedef union {
        struct {
                u8	signal;		/* (rate in 100 kbps) */
                u8	service;	/* Service: OR of SERVICE_xxx */
                u16	length;		/* Length in usecs (needs byte swap) */
        };
        u8  raw[4];
        u32 raw32;
}  __attribute__((__packed__)) dw_hw_pskcck_t;

typedef union {
        struct {
                unsigned rate		:4;	/* Data rate */
                unsigned reserved	:1;
                unsigned length		:12;	/* Length in bytes */
                unsigned parity		:1;	/* Even parity bit */
                unsigned _pad   	:14;	/* Service field */
        };
        u8  raw[4];
        u32 raw32;
} __attribute__((__packed__)) dw_hw_ofdm_t;

/* modulation header (rx ) */
typedef union {
        struct {
                unsigned mod_type : 8;
                unsigned rssi_vga : 5;
                unsigned rssi_lna : 2;
                unsigned ant	  : 1;
                unsigned freq_off : 16;	/* Frequency offset (needs byte swap) */
        };
        u32 raw32;
} __attribute__((__packed__)) dw_hw_mod_rx_t;

/* modulation header ( tx ) */
typedef union {
        struct {
                unsigned int mod_type :8;
                unsigned int length   :9;
                unsigned int pad      :15;
        };
        u32 raw32;
} __attribute__((__packed__)) dw_hw_mod_tx_t;

typedef union {
        dw_hw_pskcck_t pskcck;
        dw_hw_ofdm_t   ofdm;
} __attribute__((__packed__)) dw_hw_plcp_t;

typedef struct {
        dw_hw_mod_tx_t mod;
        dw_hw_plcp_t   plcp;
} __attribute__((__packed__)) dw_hw_hdr_tx_t;

typedef struct {
        dw_hw_mod_rx_t mod;
        dw_hw_plcp_t   plcp;
} __attribute__((__packed__)) dw_hw_hdr_rx_t;

/* CCMP key data */

typedef struct {
	u8	init[AES_BLOCK_SIZE];
	u8	header[2*AES_BLOCK_SIZE];
} __attribute__ ((__packed__)) ccmp_data_t;

typedef struct {
	u8	valid;		/* TRUE if key is valid */
        u48     tx_pn;          /* transmit packet number */
        u48     rx_pn;          /* next receive packet number */
	struct aes_key_st	rk;	/* AES key schedule */
} ccmp_key_t;

/* stores the necessary data to detect duplicate frames */
typedef struct {
        struct list_head list;
        u8  		src[ ETH_ALEN ];  /* addr2/sender  */
        u16 		seq_ctl;  /* seq_ctl of last received frame */
} dw_duplicate_t;

typedef struct {
        struct list_head list;
        struct sk_buff*  skb;  /* message data */
        dw_hw_hdr_rx_t   hdr;
} dw_frame_rx_t;

typedef struct {
        dw_frame_rx_t  free;
        dw_frame_rx_t  filled;
        dw_frame_rx_t* buffer;
} dw_frame_rx_queue_t;

typedef struct {
        dw_hw_hdr_tx_t hdr;

        /* physical length of data, including CCMP. Is not included skb->len if
         * hardware encryption is performed. */
        size_t phys_len;

        struct {
                ccmp_key_t* key;
                int         key_index;
        } crypt;
} dw_fragment_tx_t;

typedef struct {
        struct ieee80211_txb* txb;
        dw_fragment_tx_t      frags[ TX_MAX_FRAGS ];

        u8 is_data            : 1;
        u8 use_hw_encryption  : 1;
        u8 use_short_preamble : 1;
} dw_frame_tx_info_t;

typedef struct {
        struct list_head   list;
        dw_frame_tx_info_t s;
} dw_frame_tx_t;

// IBSS

#define	DW_BEACON_INT		100		// IBSS Beacon interval (in TU)

//
// 802.11 MIB constants
//
#define	DW_SHORT_RETRY_LIMIT	7		// Small frame transmit retry limit
#define	DW_LONG_RETRY_LIMIT	4		// Large frame transmit retry limit

#define	DW_TU				1024L/1000	// Time unit (in msecs)
#define	DW_MAX_TX_LIFETIME		(512*TU)	// Transmit lifetime limit (in msecs)
#define	DW_MAX_RX_LIFETIME		(512*TU)	// Receive lifetime limit (in msecs)

// Max number of fragments
#define	DW_MAX_FRAGS			16

// Frame header modulation type field
#define	DW_MOD_PSKCCK		0x00	// PSK/CCK modulation
#define	DW_MOD_OFDM			0xee	// OFDM modulation

// PSK/CCK PLCP service field bits
#define	DW_SERVICE_LOCKED		0x04	// Locked clocks
#define	DW_SERVICE_MODSEL		0x08	// Modulation selection
#define	DW_SERVICE_LENEXT		0x80	// Length extension

// MAC type field values
#define	DW_TYPE_ASSOC_REQ	0x00	// Association request
#define	DW_TYPE_ASSOC_RESP	0x10	// Association response
#define	DW_TYPE_REASSOC_REQ	0x20	// Reassociation request
#define	DW_TYPE_REASSOC_RESP	0x30	// Reassociation response
#define	DW_TYPE_PROBE_REQ	0x40	// Probe request
#define	DW_TYPE_PROBE_RESP	0x50	// Probe response

#define	DW_TYPE_BEACON		0x80	// Beacon
#define	DW_TYPE_ATIM			0x90	// Annoucement traffice indication
#define	DW_TYPE_DISASSOC		0xa0	// Disassociation
#define	DW_TYPE_AUTH			0xb0	// Authentication
#define	DW_TYPE_DEAUTH		0xc0	// Deauthentication

#define	DW_TYPE_RTS			0xb4	// Request to send
#define	DW_TYPE_CTS			0xc4	// Clear to send
#define	DW_TYPE_ACK			0xd4	// Acknowledgement
#define	DW_TYPE_PSPOLL		0xa4	// Power Save(PS)-Poll

#define	DW_TYPE_DATA			0x08	// Data

// TRUE if buf is data or management frame
#define	DW_IS_DATA(buf)			(((buf)->macHdr.fc.type & 0xcf) == TYPE_DATA)
#define	DW_IS_MGMT(buf)		(((buf)->macHdr.fc.type & 0x0f) == 0)

// MAC address macros
#define	DW_MAC_GROUP			0x01	// Broadcast or multicast address
#define	DW_MAC_LOCAL			0x02	// Locally administered address

#define	DW_EQUAL_ADDR(a1, a2)	(memcmp (a1, a2, ETH_ALEN) == 0)
#define	DW_SET_ADDR(a1, a2)		(memcpy (a1, a2, ETH_ALEN))

// Authentication algorithm number field values
#define	DW_AUTH_OPEN			0x00	// Open system
#define	DW_AUTH_SHAREDKEY		0x01	// Shared key
#define	DW_AUTH_LEAP			0x80	// LEAP

// Capability information field bits
#define	DW_CAP_ESS			0x0001	// Extended service set (infrastructure)
#define	DW_CAP_IBSS			0x0002	// Independent BSS (ad hoc)
#define	DW_CAP_POLLABLE		0x0004	// Contention free pollable
#define	DW_CAP_POLLREQ		0x0008	// Contention free poll request
#define	DW_CAP_PRIVACY		0x0010	// Privacy (WEP) required
#define	DW_CAP_SHORTPRE		0x0020	// Short preambles allowed
#define	DW_CAP_PBCC			0x0040	// PBCC modulation allowed
#define	DW_CAP_AGILITY			0x0080	// Channel agility in use
#define	DW_CAP_SHORTSLOT		0x0400	// Short slot time in use
#define	DW_CAP_DSSSOFDM		0x2000	// DSSS-OFDM in use

// Status code field values
#define	DW_STAT_SUCCESS		0

// Reason code field values
#define	DW_REAS_NOLONGERVALID	2
#define	DW_REAS_DEAUTH_LEAVING	3
#define	DW_REAS_INACTIVITY		4
#define	DW_REAS_INCORRECT_FRAME_UNAUTH 6
#define	DW_REAS_INCORRECT_FRAME_UNASSO 7

// Information element IDs
#define	DW_ELEM_SSID			0		// Service set ID
#define	DW_ELEM_SUPRATES		1		// Supported rates
#define	DW_ELEM_DSPARAM		3		// DS parameter set
#define	DW_ELEM_IBSSPARAM		6		// IBSS parameter set
#define	DW_ELEM_COUNTRY      	7		// Country information
#define	DW_ELEM_CHALLENGE		16		// Challenge text
#define	DW_ELEM_ERPINFO		42		// Extended rate PHY info
#define	DW_ELEM_RSN			48		// Robust security network (WPA2)
#define	DW_ELEM_EXTSUPRATES	50		// Extended supported rates
#define	DW_ELEM_VENDOR		221		// Vendor extension (WPA)

// 802.11d related defines
// minimum length field value in country information elelment
#define	DW_COUNTRY_INFO_MIN_LEN   6

// Supported rates bits
#define	DW_RATE_BASIC			0x80	// Bit set if basic rate

// TRUE if channel number in 5 GHz band
#define	DW_CHAN_5G(chan)		((chan) > 14)

// ERP info bits
#define	DW_ERP_NONERP			0x01	// Non-ERP present
#define	DW_ERP_USEPROTECT		0x02	// Use protection
#define	DW_ERP_BARKER			0x04	// Barker (long) preamble mode

// Key ID byte in data frame body
#define	DW_EXT_IV				0x20	// Extended IV is present

// Correct CRC-32 check value
#define	DW_GOOD_CRC32			0x2144df1c

#pragma pack()

#define BEACON_BODY_SIZE 64

// MAC buffer, including complete MAC frame
typedef struct {
	dw_hw_hdr_tx_t hwHdr; // Frame and PLCP headers
	u16 fc; // Frame control
	u16 duration; // Duration/ID (needs byte swap)
	u8 addr1[ ETH_ALEN ]; // Address 1
	u8 addr2[ ETH_ALEN ]; // Address 2
	u8 addr3[ ETH_ALEN ]; // Address 3
	u16 seq_ctl; // Sequence control fields
	u8 body[BEACON_BODY_SIZE];
} __attribute__ ((packed)) dw_beacon_frame;

// Length (in usecs) of a MAC frame of bytes at rate (in 500kbps units)
// not including SIFS and PLCP preamble/header
#define	DW_LENGTH_uS(bytes, rate)		((16*(bytes)+(rate)-1)/(rate))

// Length (in usecs) of SIFS and PLCP preamble/header.
#define	DW_PRE_LEN_uS(rate)			(USE_SHORTPRE(rate) ? 106 : 202)

// Duration (in usecs) of an OFDM frame at rate (in 500kbps units)
// including SIFS and PLCP preamble/header
#define	DW_OFDM_DUR(bytes, rate)	(36 + 4*((4*(bytes)+(rate)+10)/(rate)))

// Information on each supported rate
typedef struct {
	u8	bps; // Bit rate in 500kbps units
	u8	ofdmCode; // OFDM rate code, 0 if not OFDM
	u16	ackLen; // Duration of ACK or CTS in usecs
} RateInfo;

#define DW_IBSS_DEFAULT_CHANNEL 6

// End of IBSS

typedef struct dw_priv {
	struct net_device*              dev;
	struct ieee80211_device*        ieee;
	struct ieee80211softmac_device* softmac;

	struct iw_statistics wstats;

	spinlock_t lock;

	struct timer_list management_timer;

	/* Additional information, specific to the 80211 cores. */
	/* Driver status flags. */
	u32 recovery:1;	/* TRUE if interval follows rate increase */
	/* Interrupt Service Routine tasklet (bottom-half) */
	u8 channel;
	/* Informational stuff. */
	char nick[IW_ESSID_MAX_SIZE + 1];
	int short_retry_limit;
	int long_retry_limit;
	ccmp_key_t aeskeys[WEP_KEYS];
	int cw;		/* Contention window size */
	int success;		/* Successful intervals */
	int success_threshold;	/* Successful intervals needed to increase rate */
        atomic_t fix_rate;

        struct ieee80211softmac_ratesinfo ap_ri;

        struct {
                int counter;
        } activity;

        struct {
                int index;
                int counter;
                int recovery;
                int tx_data_any;
                int tx_data_ack;
                int have_activity;
                int success;
                int success_threshold;
        } rate;

        struct {
                struct tasklet_struct tasklet;
                dw_frame_rx_queue_t   queue;
                char                  pause;

                /* maintains the list of received frames */
                struct {
                        dw_duplicate_t known;
                        dw_duplicate_t free;
                        dw_duplicate_t entries[ RX_DUP_SENDER_SIZE ];
                } dups;
        } rx;

        struct {
                struct tasklet_struct tasklet;
                dw_frame_tx_t         frames[ TX_QUEUE_SIZE ];
                struct list_head      queued;
                struct list_head      free;
                atomic_t              seq_nr;
                int                   timeout;
                int                   times_sent;
                int                   retries;
                int                   fragment;
                char                  pending;
                char                  pause;
                struct semaphore      pause_sem;
                char                  last_was_data;
                int                   basics[ RATES_SUPPORTED ];
                void *                data_pending_ack;
                unsigned long         jiffies_pending_ack;
        } tx;

	unsigned char tx_power;

	int beacon_ready;
	struct delayed_work beacon_work;
	int beacon_body_length;
	dw_beacon_frame beacon_frame;
	struct ieee80211softmac_network adhoc;

	unsigned long jiffies_last_beacon;
	int reconnection_attempts;
}  dw_priv_t;

/* dw_(un)lock() protect struct dw_private.
 */
#define dw_lock(priv, flags)	spin_lock_irqsave(&(priv)->lock, flags)
#define dw_unlock(priv, flags)	spin_unlock_irqrestore(&(priv)->lock, flags)

/* Frame header modulation type field */
#define	MOD_PSKCCK	0x00	/* PSK/CCK modulation */
#define	MOD_OFDM	0xee	/* OFDM modulation */

/* PLCP service field bits */
#define SERVICE_LOCKED	0x04    /* Locked clocks */
#define SERVICE_MODSEL	0x08    /* Modulation selection */
#define SERVICE_LENEXT	0x80    /* Length extension */

/* Threshold values. */
#define DW_MIN_RTS_THRESHOLD		1U
#define DW_MAX_RTS_THRESHOLD		2304U
/* 1536 is default for most routers, but our FIFO is larger, so we could accept
 * more data, e.g. for improvements when doing AES etc. This has been reported
 * being done with Apple iTunes */
#define DW_MTU				2048

#define	AES_BITS	128	/* 128 bit keys, 10 rounds */

# define GETU32(pt) (((u32)(pt)[0] << 24) ^ ((u32)(pt)[1] << 16) ^ ((u32)(pt)[2] <<  8) ^ ((u32)(pt)[3]))
/* Get 16 bits at byte pointer */
#define	GET16(bp)		((bp)[0] | ((bp)[1] << 8))
/* Get 32 bits at byte pointer */
#define	GET32(bp)		((bp)[0] | ((bp)[1] << 8) | ((bp)[2] << 16) | ((bp)[3] << 24))
/* Store 16 bits at byte pointer */
#define	SET16(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8; }
/* Store 32 bits at byte pointer */
#define	SET32(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8;  \
				(bp)[2] = (data) >> 16; \
				(bp)[3] = (data) >> 24; }

#endif /* FS_DIGI_WI_G_DRIVER_H */
