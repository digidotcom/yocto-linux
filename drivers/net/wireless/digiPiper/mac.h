#ifndef DIGI_MAC_H_
#define DIGI_MAC_H_

enum baseband_control_regs {
	BB_VERSION = 0x00,
	BB_GENERAL_CTL = 0x04,
	BB_GENERAL_STAT = 0x08,
	BB_RSSI = 0x0c,
	BB_IRQ_MASK = 0x10,
	BB_IRQ_STAT = 0x14,
	BB_SPI_DATA = 0x18,
	BB_SPI_CTRL = 0x1c,
	BB_DATA_FIFO = 0x20,
	BB_TRACK_CONTROL = 0x28,
	BB_CONF_2 = 0x2c,
	BB_AES_FIFO = 0x30,
	BB_AES_CTL = 0x38,
	BB_OUTPUT_CONTROL = 0x3c
};

#define BB_VERSION_MASK(v)		((v) & 0xffff)

#define BB_GENERAL_CTL_RX_EN		(1<<0)
#define BB_GENERAL_CTL_ANT_DIV		(1<<1)
#define BB_GENERAL_CTL_ANT_SEL		(1<<2)
#define	BB_GENERAL_CTL_GEN_5GEN		(1<<3)		// 5 GHz band enable
#define BB_GENERAL_CTL_SH_PRE       (1<<4)
#define BB_GENERAL_CTL_RXFIFORST    (1<<5)
#define BB_GENERAL_CTL_TXFIFORST    (1<<6)
#define BB_GENERAL_CTL_TX_HOLD		(1<<7)
#define BB_GENERAL_CTL_BEACON_EN	(1<<8)
#define BB_GENERAL_CTL_FW_LOAD_ENABLE   (1 << 9)
#define BB_GENERAL_CTL_DSP_LOAD_ENABLE  (1 << 10)
#define BB_GENERAL_CTL_MAC_ASSIST_ENABLE (1 << 11)
#define BB_GENERAL_CTL_TX_FIFO_FULL	 (1<<15)
#define BB_GENERAL_CTL_TX_FIFO_EMPTY (1<<14)
/* TODO: verify max gain value for Piper and Wi9p*/
#define BB_GENERAL_CTL_MAX_GAIN(g)	(((g) & 0x7f)<<16)
#define BB_GENERAL_CTL_PWR_UP		(1<<24)
#define BB_GENERAL_CTL_ADC_CLK_EN	(1<<25)
#define BB_GENERAL_CTL_BOOT_STAT	(1<<28)
#define BB_GENERAL_CTL_CLK_EN		(1<<29)
#define BB_GENERAL_CTL_SPI_RST		(1<<30)

#define BB_GENERAL_CTL_MAX_GAIN_MASK (0x007F0000)
#define BB_GENERAL_CTL_DEFAULT_MAX_GAIN_A	(0x00790000)
#define BB_GENERAL_CTL_DEFAULT_MAX_GAIN_BG	(0x007c0000)

#if defined(CONFIG_PIPER_WIFI)
#if 0
#define BB_GENERAL_CTL_INIT		(BB_GENERAL_CTL_MAX_GAIN(0x7a) | \
		BB_GENERAL_CTL_PWR_UP | BB_GENERAL_CTL_ADC_CLK_EN | \
		BB_GENERAL_CTL_BOOT_STAT | BB_GENERAL_CTL_CLK_EN)
#define BB_GENERAL_CTL_RESET		(BB_GENERAL_CTL_MAX_GAIN(0x7f) | \
		BB_GENERAL_CTL_ADC_CLK_EN | BB_GENERAL_CTL_BOOT_STAT | \
		BB_GENERAL_CTL_SPI_RST)
#else
#define BB_GENERAL_CTL_INIT		(BB_GENERAL_CTL_MAX_GAIN(0x7a)

#define BB_GENERAL_CTL_RESET		(BB_GENERAL_CTL_MAX_GAIN(0x7f) | \
                                BB_GENERAL_CTL_SPI_RST
#endif
#else
#define BB_GENERAL_CTL_INIT		(BB_GENERAL_CTL_MAX_GAIN(0x7a) | \
		BB_GENERAL_CTL_PWR_UP | BB_GENERAL_CTL_ADC_CLK_EN | \
		BB_GENERAL_CTL_BOOT_STAT | BB_GENERAL_CTL_CLK_EN)

#define BB_GENERAL_CTL_RESET		(BB_GENERAL_CTL_MAX_GAIN(0x7f) | \
		BB_GENERAL_CTL_ADC_CLK_EN | BB_GENERAL_CTL_BOOT_STAT | \
		BB_GENERAL_CTL_SPI_RST)
#endif
#define BB_RSSI_LED			    (1<<8)
#define BB_RSSI_EAS_FIFO_EMPTY  (1 << 16)
#define BB_RSSI_EAS_FIFO_FULL   (1 << 17)
#define BB_RSSI_EAS_BUSY        (1 << 18)
#define BB_RSSI_EAS_MIC         (1 << 19)
#define BB_RSSI_ANT_MASK		(0xff<<24)
#ifdef CONFIG_MACH_CCW9P9215JS
#define BB_RSSI_ANT_NO_DIV_MAP	    (0x96000000)
#define BB_RSSI_ANT_DIV_MAP         (0x1E000000)
#else
#define BB_RSSI_ANT_NO_DIV_MAP	    (0x69000000)
#define BB_RSSI_ANT_DIV_MAP         (0xE1000000)
#endif
#define BB_GENERAL_STAT_RESET		(1<<30)
/*
 * STAT_B_EN is a constant that defines a bit in the Wireless Controller FPGA Baseband Control Register
 * for General Status, which enables the PSK/CCK receiver baseband circuitry (802.11b receiver).
 * STAT_A_EN is a constant that defines a bit in the Wireless Controller FPGA Baseband Control Register
 * for General Status, which enables the OFDM receive baseband circuitry (802.11a receiver).
 */

#define BB_GENERAL_STAT_B_EN		0x10000000      // B EN (PSK/CCK)
#define BB_GENERAL_STAT_A_EN		0x20000000      // A EN (OFDM)
#define BB_GENERAL_STAT_RX_FIFO_EMPTY   (1 << 4)
#define BB_GENERAL_STAT_DC_DIS      (1 << 24)
#define BB_GENERAL_STAT_SRC_DIS		(1 << 16)
#define BB_GENERAL_STAT_SPRD_DIS    (1 << 17)
#define BB_GENERAL_STAT_DLL_DIS		(1 << 18)
#define TRACK_TX_B_GAIN_MASK	0xff000000		// Mask word for B_TX_GAIN
#define TRACK_TX_B_GAIN_NORMAL	0xA0000000		// normal setting for B_TX_GAIN
#define TRACK_BG_BAND           0x00430000     // Tracking constant for 802.11 b/g frequency band
#define TRACK_CONSTANT_MASK		0x00ff0000		// mask for tracking constant
#define TRACK_4920_4980_A_BAND  0x00210000     // Tracking constant for 802.11 a sub-frequency band
#define TRACK_5150_5350_A_BAND  0x001F0000     // Tracking constant for 802.11 a sub-frequency band
#define TRACK_5470_5725_A_BAND  0x001D0000     // Tracking constant for 802.11 a sub-frequency band
#define TRACK_5725_5825_A_BAND  0x001C0000     // Tracking constant for 802.11 a sub-frequency band


#define BB_IRQ_MASK_RX_FIFO		(1<<0)
#define BB_IRQ_MASK_TX_FIFO_EMPTY	(1<<1)
#define BB_IRQ_MASK_TIMEOUT		(1<<2)
#define BB_IRQ_MASK_TX_ABORT		(1<<3)
#define BB_IRQ_MASK_TBTT		(1<<4)
#define BB_IRQ_MASK_ATIM		(1<<5)
#define BB_IRQ_MASK_RX_OVERRUN		(1<<6)

#define BB_AES_CTL_KEY_LOAD		(1<<2)
#define BB_AES_CTL_AES_MODE		(1<<4)

enum mac_control_regs {
	MAC_STA_ID0 = 0x40,
	MAC_STA_ID1 = 0x44,
	MAC_BSS_ID0 = 0x48,
	MAC_BSS_ID1 = 0x4c,
	MAC_SSID_LEN = 0x50,	/* OFDM_BRS, PSK_BRS, TX_CTL, SSID_LEN */
	MAC_BACKOFF = 0x54,	/* actually 0x56; 2 low order bytes are empty */
	MAC_DTIM_PERIOD = 0x58,
	/*MAC_CFP_PERIOD = 0x59,*/
	/*MAC_LISTEN_INTERVAL = 0x5a,*/
	MAC_CFP_ATIM = 0x5c,	/* beacon interval, CFP/ATIM duration */
	MAC_STATUS = 0x60,
	/*MAC_TXP_TIMING = 0x62,*/
	/*MAC_STATUS = 0x63,*/
	MAC_CTL = 0x64,		/* MAC_AES_KEY_DIS (8 bits), MAC_CTL (8 bits) */
	MAC_MEASURE = 0x68,	/* actually 0x69 */
	/*MAC_REMAIN_BO = 0x6a,*/
	MAC_BEACON_FILT = 0x6c,	/* actally 0x6d */
	/*MAC_BEACON_BO = 0x6e,*/
	MAC_STA2_ID0 = 0xb0,
	MAC_STA2_ID1 = 0xb4,
	MAC_STA3_ID0 = 0xb8,
	MAC_STA3_ID1 = 0xbc,

	MAC_EEPROM_CTL = 0xf0,
	MAC_EEPROM_DATA = 0xf8,

	MAC_SSID = 0x80,

	BEACON_FIFO = 0x85,     /* dummy value used to select data fifo for beacon load */
};


#define MAC_SSID_LEN_MASK           (0x000000ff)
#define MAC_REVISION_MASK(v)		(((v) >> 16) & 0xffff)

#define MAC_BEACON_INTERVAL_SHIFT   (16)
#define MAC_BEACON_INTERVAL_MASK    (0xffff0000)

#define MAC_ATIM_PERIOD_MASK        (0x0000ffff)

#define MAC_LISTEN_INTERVAL_MASK    (0x0000ffff)

#define MAC_DTIM_PERIOD_SHIFT       (24)
#define MAC_DTIM_PERIOD_MASK        (0xff000000)

#define MAC_DTIM_CFP_SHIFT          (16)
#define MAC_DTIM_CFP_MASK           (0x00ff0000)

#define MAC_OFDM_BRS_MASK           (0xff000000)
#define MAC_OFDM_BRS_SHIFT          (24)
#define MAC_PSK_BRS_MASK            (0x000f0000)
#define MAC_PSK_BRS_SHIFT           (16)

#define MAC_BEACON_BACKOFF_MASK     (0x0000ffff)

#define MAC_BRS_MASK                (MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK)

#define MAC_CTL_TX_REQ          (1)
#define MAC_CTL_BEACON_TX		(1<<2)
#define MAC_CTL_IBSS			(1<<4)
#define MAC_CTL_AES_DISABLE		(1<<5)
#define MAC_CTL_MAC_FLTR		(1<<6)
#define MAC_CTL_KEY0_DISABLE		(1<<8)
#define MAC_CTL_KEY1_DISABLE		(1<<9)
#define MAC_CTL_KEY2_DISABLE		(1<<10)
#define MAC_CTL_KEY3_DISABLE		(1<<11)

#define MAC_EEPROM_CTL_WAIT_MS		21

/*
 * RX packets look something like:
 * <custom bus protocol header(s)> - protocol-dependent
 * <rx_frame_hdr> - 4 bytes
 * <plcp; either psk_cck_hdr or ofdm_hdr> - 4 bytes
 * <mac header> - dealt with by the mac80211 stack, not the driver
 * <data>
 *
 * TX packets are similar:
 * <custom bus protocol header(s)>
 * <tx_frame_hdr> - 4 bytes
 * <plcp; either psk_cck_hdr or ofdm_hdr> - 4 bytes
 * <mac header>
 * <data>
 */



// MAC type field values
#define	TYPE_ASSOC_REQ		0x00	// Association request
#define	TYPE_ASSOC_RESP		0x10	// Association response
#define	TYPE_REASSOC_REQ	0x20	// Reassociation request
#define	TYPE_REASSOC_RESP	0x30	// Reassociation response
#define	TYPE_PROBE_REQ		0x40	// Probe request
#define	TYPE_PROBE_RESP		0x50	// Probe response

#define	TYPE_BEACON		0x80	// Beacon
#define	TYPE_ATIM		0x90	// Annoucement traffice indication
#define	TYPE_DISASSOC		0xa0	// Disassociation
#define	TYPE_AUTH		0xb0	// Authentication
#define	TYPE_DEAUTH		0xc0	// Deauthentication
#define TYPE_ACTION		0xd0    // Action

#define TYPE_RTS		0xb4	// Request to send
#define TYPE_CTS		0xc4	// Clear to send
#define TYPE_ACK		0xd4	// Acknowledgement
#define TYPE_PSPOLL		0xa4	// Power Save(PS)-Poll

#define TYPE_DATA		0x08	// Data
#define TYPE_NULL_DATA		0x48    // Null Data

struct tx_frame_hdr {
#if 1
	unsigned int modulation_type:8;
	unsigned int length:9;
	unsigned int pad:15;
#else
	uint8_t modulation_type;
	__le16 length:9;
	unsigned int pad:15;
#endif
} __attribute__((packed));


#define MOD_TYPE_PSKCCK		0x00
#define MOD_TYPE_OFDM		0xee

struct psk_cck_hdr {
	uint8_t signal;		/* x100Kbps */
	uint8_t service;
	__le16 length;		/* usecs */
} __attribute__((packed));

/* PSK/CCK PLCP service field bits */
#define PLCP_SERVICE_LOCKED	0x04	/* locked clocks */
#define PLCP_SERVICE_MODSEL	0x08	/* modulation selection */
#define PLCP_SERVICE_LENEXT	0x80	/* length extension */

struct ofdm_hdr {
	unsigned int rate:4;
	unsigned int pad_a:1;
	unsigned int length:12;	/* in bytes */
	unsigned int parity:1;
	unsigned int pad_b:14;
} __attribute__((packed));


struct rx_frame_hdr {
	uint8_t modulation_type;
	unsigned int rssi_variable_gain_attenuator:5;
	unsigned int rssi_low_noise_amp:2;
	unsigned int antenna:1;
	__be16 freq_offset;
	union
	{
	    struct psk_cck_hdr psk;
	    struct ofdm_hdr ofdm;
	} mod;
} __attribute__((packed));

typedef struct
{
	unsigned type		:8;		// Type, subtype, version
	unsigned toDS		:1;		// To distribution service (AP)
	unsigned fromDS		:1;		// From distribution service (AP)
	unsigned moreFrag	:1;		// More fragments
	unsigned retry		:1;		// Retransmission
	unsigned pwrMgt		:1;		// Power management state
	unsigned moreData	:1;		// More data buffered
	unsigned protectd	:1;		// Encrypted
	unsigned order		:1;		// Strictly ordered
} frameControlFieldType_t;

#define WLN_ADDR_SIZE       (6)

typedef unsigned char MacAddr[WLN_ADDR_SIZE];


#define	PACKED_H
#define	PACKED_F	__attribute__ ((packed))

typedef PACKED_H struct {
	unsigned type		:8;		// Type, subtype, version
#ifdef BIG_ENDIAN
	unsigned order		:1;		// Strictly ordered
	unsigned protected	:1;		// Encrypted
	unsigned moreData	:1;		// More data buffered
	unsigned pwrMgt		:1;		// Power management state
	unsigned retry		:1;		// Retransmission
	unsigned moreFrag	:1;		// More fragments
	unsigned fromDS		:1;		// From distribution service (AP)
	unsigned toDS		:1;		// To distribution service (AP)
#else
	unsigned toDS		:1;		// To distribution service (AP)
	unsigned fromDS		:1;		// From distribution service (AP)
	unsigned moreFrag	:1;		// More fragments
	unsigned retry		:1;		// Retransmission
	unsigned pwrMgt		:1;		// Power management state
	unsigned moreData	:1;		// More data buffered
	unsigned protected	:1;		// Encrypted
	unsigned order		:1;		// Strictly ordered
#endif
} PACKED_F FrameControl_t;


// Sequence control field
// Need to swap bytes on BIG_ENDIAN
typedef PACKED_H struct {
#ifdef BIG_ENDIAN
	unsigned seq		:12;	// Sequence number
	unsigned frag		:4;		// Fragment number
#else
	unsigned frag		:4;		// Fragment number
	unsigned seq		:12;	// Sequence number
#endif
} PACKED_F SeqControl;

// Union of sequence control types
typedef PACKED_H union {
	SeqControl sq;				// Sequence control fields
	unsigned short sq16;	    // Sequence control as 16-bit int (needs byte swap)
} PACKED_F SeqControlU;



typedef PACKED_H struct {
	FrameControl_t fc;			// Frame control
	unsigned short	duration;   // Duration/ID (needs byte swap)
	MacAddr	addr1;				// Address 1
	MacAddr	addr2;				// Address 2
	MacAddr	addr3;				// Address 3
	SeqControlU squ;			// Sequence control fields
} PACKED_F _80211HeaderType;

typedef PACKED_H struct {
	FrameControl_t fc;			// Frame control
	unsigned short aid;   		// association identifier
	MacAddr	addr1;				// Address 1
	MacAddr	addr2;				// Address 2
} PACKED_F _80211PSPollType;

#define _80211_HEADER_LENGTH	(sizeof(_80211HeaderType))
#define TX_HEADER_LENGTH	(sizeof(struct ofdm_hdr) + sizeof(struct tx_frame_hdr))
/* FIFO sizes in bytes */
#define TX_FIFO_SIZE            1792
#define RX_FIFO_SIZE            2048

#define RATE_MASK_BASIC		0x0153
#define RATE_MASK_OFDM		0x0ff0
#define RATE_MASK_PSK_CCK	0x000f

#define BEACON_INT		100	/* in TU */


#define	DEFAULT_CW_MIN		32			// Min contention window size
#define	DEFAULT_CW_MAX		1024		// Max contention window size

#define ASLOT_TIME		20
#endif
