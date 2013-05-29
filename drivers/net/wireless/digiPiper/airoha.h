#ifndef DIGI_RF_AH7230_H_
#define DIGI_RF_AH7230_H_

/* 0 is reserved for unknown transceiver */
#define RF_AIROHA_7230		(1)
#define RF_AIROHA_2236		(2)

#define SPI_INIT_AIROHA		(0x00000018)	/* AIROHA-specific SPI length */
#define	SPI_INIT_AIROHA2236	(0x00000014)	/* AIROHA 2236-specific SPI length */
#define	GEN_INIT_AIROHA_24GHZ	(0x31720005)	/*  Initial state; 2.4GHZ_PA_ON= active low; bit 25 */
#define	GEN_INIT_AIROHA_50GHZ	(0x33760008)	/* Initial state; 5.0GHZ_PA_ON= active high; bit 25 */

#define AIROHA_LOWEST_PSK_RATE_INDEX	(0)
#define AIROHA_LOWEST_OFDM_RATE_INDEX	(4)
#define AIROHA_55_MBPS_RATE_INDEX		(2)

/*
 * Subtract this number from the channel index to index into
 * the 802.11a channel array.
 */
#define BAND_A_OFFSET		(17)

struct digi_rf_freq {
	uint16_t integer;
	uint16_t fract;
};

extern struct digi_rf_ops al7230_rf_ops;

#endif
