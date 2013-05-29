#ifndef __PIPER_H_
#define __PIPER_H_

#include <linux/completion.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>
#include <linux/i2c.h>
#include "mac.h"


/* #define WANT_DEBUG */
#ifdef WANT_DEBUG
#define digi_dbg(fmt, arg...) \
    printk(KERN_ERR PIPER_DRIVER_NAME ": %s - " fmt, __func__, ##arg)
#else
#define digi_dbg(fmt, arg...) \
	do { } while (0)
#endif

#define ERROR(x)		printk(KERN_ALERT x)

/* Debug levels */
#define DSILENT			0xffff
#define DERROR			0xfff0
#define DWARNING		0xffe0
#define DNORMAL			0xffd0
#define DVERBOSE		0xffc0
#define DVVERBOSE		0xffb0
#define DALL			0xffa0


#define PIPER_DRIVER_NAME	"piper"
#define DRV_VERS		"0.1"

/* Useful defines for AES */
#define	PIPER_EXTIV_SIZE	8	/* IV and extended IV size */
#define	MIC_SIZE		8	/* Message integrity check size */
#define ICV_SIZE		4
#define	DATA_SIZE		28	/* Data frame header+FCS size */
#define	CCMP_SIZE		(PIPER_EXTIV_SIZE + MIC_SIZE)	/* Total CCMP size */
#define EXPANDED_KEY_LENGTH	(176)	/* length of expanded AES key */
#define PIPER_MAX_KEYS		(4)
#define AES_BLOB_LENGTH		(48)	/* length of AES IV and headers */

/* Calibration constants */
#define WCD_MAGIC		"WCALDATA"
#define WCD_MAX_CAL_POINTS	(8)
#define WCD_CHANNELS_BG		(14)
#define WCD_CHANNELS_A		(35)
#define WCD_B_CURVE_INDEX       (0)
#define WCD_G_CURVE_INDEX       (1)

/*
 * Set this #define to receive frames in the ISR.  This may improve
 * performance under heavy load at the expense of interrupt latency.
 */
#define WANT_TO_RECEIVE_FRAMES_IN_ISR	(0)


typedef u64 u48;

/*
 * This enum lists the possible LED states.
 */
enum led_states {
    led_shutdown,
    led_adhoc,
    led_not_associated,
    led_associated
};

/* Available leds */
enum wireless_led {
    STATUS_LED,
    ACTIVITY_LED,
};

#define WCD_HW_REV_MASK			0xf000
#define WCD_HW_REV_PROTOTYPE	0x0000
#define WCD_HW_REV_PILOT		0x1000
#define WCD_HW_REV_A			0x2000
#define WCD_PLATFORM_MASK		0x0ff0
#define WCD_CCW9P_PLATFORM		0x0010
#define WCD_CCW9M_PLATFORM		0x0020


typedef struct nv_wcd_header {
    char magic_string[8];	/* WCALDATA */
    char ver_major;		/* Major version in ascii */
    char ver_minor;		/* Minor version in ascii */
    u16 hw_platform;		/* Hardware Platform used for calibration */
    u8 numcalpoints;		/* Number of points per curve */
    u8 padding[107];		/* Reserved for future use */
    u32 wcd_len;		/* Total length of the data section */
    u32 wcd_crc;		/* Data section crc32 */
} nv_wcd_header_t;

typedef struct wcd_point {
    s16 out_power;		/* Output Power */
    u16 adc_val;		/* Measured ADC val */
    u8 power_index;		/* Airoha Power Index */
    u8 reserved[3];		/* For future use */
} wcd_point_t;

typedef struct wcd_curve {
    u8 max_adc_value;		/* maximum allowed ADC value for this curve */
    u8 reserved[3];		/* Resered for future use */
    /* Calibration curve points */
    wcd_point_t points[WCD_MAX_CAL_POINTS];
} wcd_curve_t;

typedef struct wcd_data {
    nv_wcd_header_t header;
    wcd_curve_t cal_curves_bg[WCD_CHANNELS_BG][2];
    wcd_curve_t cal_curves_a[WCD_CHANNELS_A];
} wcd_data_t;

typedef enum {
    op_write,
    op_or,
    op_and
} reg_op_t;

enum antenna_select {
    ANTENNA_BOTH = 0,
    ANTENNA_1,
    ANTENNA_2,
};

typedef struct {
    bool loaded;
    bool enabled;
    bool weSentLastOne;
} piperBeaconInfo_t;

typedef enum {
    RECEIVED_ACK,
    TX_COMPLETE,
    OUT_OF_RETRIES,
    TX_NOT_DONE
} tx_result_t;


/* Structure that holds the information we need to support H/W AES encryption */
struct piperKeyInfo {
    bool valid;			/* indicates if this record is valid */
    u8 addr[ETH_ALEN];		/* MAC address associated with key */
    u32 expandedKey[EXPANDED_KEY_LENGTH / sizeof(u32)];
    u48 txPn;			/* packet number for transmit */
    u48 rxPn;			/* expected receive packet number */
};

/* rf */
struct digi_rf_ops {
    const char *name;
    void (*init) (struct ieee80211_hw *, int);
    int (*stop) (struct ieee80211_hw *);
    int (*set_chan) (struct ieee80211_hw *, int chan);
    int (*set_chan_no_rx) (struct ieee80211_hw *, int chan);
    int (*set_pwr) (struct ieee80211_hw *, uint8_t val);
    void (*set_pwr_index) (struct ieee80211_hw *, unsigned int val);
    void (*power_on) (struct ieee80211_hw *, bool want_power_on);
    void (*getOfdmBrs) (int channel, u64 brsBitMask, unsigned int *ofdm,
			unsigned int *psk);
    enum ieee80211_band (*getBand) (int);
    int (*getFrequency) (int);
    void (*set_hw_info)(struct ieee80211_hw *, int channel, u16 hw_platform);
    const struct ieee80211_rate *(*getRate) (unsigned int);
    int channelChangeTime;
    s8 maxSignal;
    struct ieee80211_supported_band *bands;
    u8 n_bands;
    unsigned int hw_revision;
    unsigned int hw_platform;
    const struct ieee80211_rate *(*getMaxRate)(unsigned int hw_platform,
                                         unsigned int hw_revision,
                                         unsigned int channel_index);
};

struct piper_stats {
    u32 rx_overruns;
    u32 tx_complete_count;
    u32 tx_start_count;
    u32 tx_total_tetries;
    u32 tx_retry_count[IEEE80211_TX_MAX_RATES];
    u32 tx_retry_index;
//    struct ieee80211_tx_queue_stats tx_queue;
    struct ieee80211_low_level_stats ll_stats;
    spinlock_t lock;
};

enum piper_ps_mode {
    PS_MODE_LOW_POWER,
    PS_MODE_FULL_POWER
};

enum piper_ps_state {
    PS_STATE_WAIT_FOR_BEACON,
    PS_STATE_WAIT_FOR_STOP_TRANSMIT_EVENT,
    PS_STATE_WAIT_FOR_TRANSMITTER_DONE,
    PS_STATE_WAIT_FOR_WAKEUP_ALARM,
    PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT
};

struct piper_priv;

struct piper_ps {
    u32 beacon_int;
    u16 aid;
	volatile unsigned int scan_timer;
    unsigned int sleep_time;
    struct timer_list timer;
    enum piper_ps_mode mode;
    enum piper_ps_state state;
    unsigned int this_event;
    spinlock_t lock;
    volatile bool power_management;
    volatile bool poweredDown;
    volatile bool rxTaskletRunning;
    volatile bool allowTransmits;
    volatile bool stopped_tx_queues;
    volatile unsigned int frames_pending;
};

typedef void (*tx_skb_return_cb_t)(struct ieee80211_hw *hw,
				 struct sk_buff *skb);

struct piper_queue {
	struct sk_buff *skb;
	tx_skb_return_cb_t skb_return_cb;
};

#define PIPER_TX_QUEUE_SIZE			(16)
#define NEXT_TX_QUEUE_INDEX(x)		((x+1) & 0xf)

struct piper_priv {
    const char *drv_name;
    char debug_cmd[32];
    u32 version;
    struct piper_pdata *pdata;
    struct ieee80211_hw *hw;
    struct ieee80211_key_conf txKeyInfo;
    struct ieee80211_cts ctsFrame;
    struct ieee80211_rts rtsFrame;
    struct ieee80211_rate *calibrationTxRate;
    struct piper_stats pstats;
    struct tasklet_struct rx_tasklet;
    struct tasklet_struct tx_tasklet;
    spinlock_t tx_tasklet_lock;
    bool tx_tasklet_running;
    spinlock_t tx_queue_lock;
    struct piper_queue tx_queue[PIPER_TX_QUEUE_SIZE];
    unsigned int tx_queue_head;
    unsigned int tx_queue_tail;
    unsigned int tx_queue_count;
    bool expectingAck;
    struct timer_list tx_timer;
    struct timer_list led_timer;
    enum led_states led_state;
    struct piper_ps ps;
    bool power_save_was_on_when_suspended;
    struct access_ops *ac;
    spinlock_t aesLock;
    struct digi_rf_ops *rf;
    void *__iomem vbase;
    int irq;
    tx_result_t tx_result;
    int tx_signal_strength;

    /* Function callbacks */
    int (*init_hw) (struct piper_priv *, enum ieee80211_band);
    int (*deinit_hw) (struct piper_priv *);
    void (*set_irq_mask_bit) (struct piper_priv *, u32);
    void (*clear_irq_mask_bit) (struct piper_priv *, u32);
     u16(*get_next_beacon_backoff) (void);
    int (*load_beacon) (struct piper_priv *, u8 *, u32);
    int (*rand) (void);
    void (*tx_calib_cb) (struct piper_priv *);
    int (*set_antenna) (struct piper_priv *, enum antenna_select);
    int (*set_tracking_constant) (struct piper_priv * piperp,
				  unsigned megahertz);
    void (*adjust_max_agc) (struct piper_priv * piperp, unsigned int rssi,
			    _80211HeaderType * header);

    /* General settings */
    enum nl80211_iftype if_type;
    bool areWeAssociated;
    bool is_radio_on;
    bool use_short_preamble;
    int channel;
    int tx_power;
    u8 bssid[ETH_ALEN];
    bool tx_cts;
    bool tx_rts;
    enum antenna_select antenna;
    int power_duty;

    /* AES stuff */
    bool use_hw_aes;
    u32 aes_key_count;
    struct piperKeyInfo key[PIPER_MAX_KEYS];
    u32 tx_aes_key;
    u32 tx_aes_blob[AES_BLOB_LENGTH / sizeof(u32)];

    /* IBSS */
    piperBeaconInfo_t beacon;
};

struct access_ops {
    spinlock_t reg_lock;
    int (*wr_reg) (struct piper_priv *, u8 reg, u32 val, reg_op_t op);
     u32(*rd_reg) (struct piper_priv *, u8 reg);
    int (*wr_fifo) (struct piper_priv *, u8 addr, u8 * buf, int len);
    int (*rd_fifo) (struct piper_priv *, u8 addr, u8 * buf, int len);
};

struct piper_pdata {
    u8 macaddr[6];
    int rst_gpio;
    int irq_gpio;
    int status_led_gpio;
    int rf_transceiver;
    wcd_data_t wcd;
    int i2c_adapter_num;
    struct piper_priv *piperp;

    /* Platform callbacks */
    void (*reset) (struct piper_priv *, int);
    int (*init) (struct piper_priv *);
    int (*late_init) (struct piper_priv *);
    void (*set_led) (struct piper_priv *, enum wireless_led, int);
    void (*early_resume) (struct piper_priv *);
};

/* main */
int piper_alloc_hw(struct piper_priv **priv, size_t priv_sz);
void piper_free_hw(struct piper_priv *priv);
int piper_register_hw(struct piper_priv *priv, struct device *dev,
		      struct digi_rf_ops *rf);
void piper_unregister_hw(struct piper_priv *priv);
irqreturn_t piper_irq_handler(int irq, void *dev_id);
void packet_tx_done(struct piper_priv *piperp,
		    tx_result_t result, int singalstrength);
void piper_rx_tasklet(unsigned long context);
void piper_tx_tasklet(unsigned long context);
bool piper_prepare_aes_datablob(struct piper_priv *digi,
				unsigned int keyIndex, u8 * aesBlob,
				u8 * frame, u32 length, bool isTransmit);
void piper_load_mac_firmware(struct piper_priv *piperp);
void piper_load_dsp_firmware(struct piper_priv *piperp);
int piper_spike_suppression(struct piper_priv *piperp, bool retry);
void piper_reset_mac(struct piper_priv *piperp);
void piper_set_macaddr(struct piper_priv *piperp);
int piper_hw_tx_private(struct ieee80211_hw *hw, struct sk_buff *skb, tx_skb_return_cb_t fn);
void piper_empty_tx_queue(struct piper_priv *piperp);
int piper_tx_enqueue(struct piper_priv *piperp, struct sk_buff *skb, tx_skb_return_cb_t skb_return_cb);
struct sk_buff *piper_tx_getqueue(struct piper_priv *piperp);
bool piper_tx_queue_half_full(struct piper_priv *piperp);
void piper_set_macaddr(struct piper_priv *piperp);
void piper_MacEnterActiveMode(struct piper_priv *piperp, bool want_spike_suppression);
int piper_MacEnterSleepMode(struct piper_priv *piperp, bool force);
void piper_sendNullDataFrame(struct piper_priv *piperp, bool isPowerSaveOn);
void piper_ps_rx_task_exiting(struct piper_priv *piperp);
void piper_ps_scan_event(struct piper_priv *piperp);

/*
 * Defines for debugging function dumpRegisters
 */
#define MAIN_REGS           (1)
#define MAC_REGS            (2)
#define RF_REGS             (4)
#define FRAME_BUFFER_REGS   (8)
#define CTRL_STATUS_REGS    (0x10)
#define FIFO_REGS           (0x20)
#define IRQ_REGS            (0x40)
#define ALL_REGS            (0xf)

void digiWifiDumpRegisters(struct piper_priv *digi, unsigned int regs);
void digiWifiDumpSkb(struct sk_buff *skb);

extern void digiWifiDumpWordsAdd(unsigned int word);
extern void digiWifiDumpWordsDump(void);
extern void digiWifiDumpWordsReset(void);

#endif				/* __PIPER_H_ */
