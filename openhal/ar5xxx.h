/*
 * Copyright (c) 2004-2007 Reyk Floeter <reyk@openbsd.org>
 * Copyright (c) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
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
 *
 * $Id$
 */

/*
 * HAL interface for Atheros Wireless LAN devices.
 *
 * ar5k is a free replacement of the binary-only HAL used by some drivers
 * for Atheros chipsets. While using a different ABI, it tries to be
 * source-compatible with the original (non-free) HAL interface.
 *
 * Many thanks to various contributors who supported the development of
 * ar5k with hard work and useful information. And, of course, for all the
 * people who encouraged me to continue this work which has been based
 * on my initial approach found on http://team.vantronix.net/ar5k/.
 */

#ifndef _AR5K_H
#define _AR5K_H

/*Os dependent definitions*/
#include "ah_osdep.h"

/*Regulatory domain & Channel definitions*/
#include "ieee80211_regdomain.h"

/*Options*/
#include "opt_ah.h"

/*
 *Translation for MadWiFi combatibility
 *(damn this is changed AGAIN in if_ath.pci :P)
 */
#include "translation.h"

/*Use with MadWiFi/net80211*/
#include "stack_net80211.h"


/****************************\
  GENERIC DRIVER DEFINITIONS
\****************************/

/*
 * C doesn't support boolean ;-(
 * TODO: See if there is a bool definition somewere else
 * in the kernel, we shouldn't redefine it if it does...
 */
#ifndef TRUE
#define	TRUE	1
#endif
#ifndef FALSE
#define	FALSE	0
#endif
typedef u_int8_t AR5K_BOOL;

/*
 * Error codes reported from HAL to the driver
 */
typedef enum {
	AR5K_OK		= 0,	/* Everything went O.K.*/
	AR5K_ENOMEM	= 1,	/* Unable to allocate memory for ath_hal*/
	AR5K_EIO	= 2,	/* Hardware I/O Error*/
	AR5K_EELOCKED	= 3,	/* Unable to access EEPROM*/
	AR5K_EEBADSUM	= 4,	/* Invalid EEPROM checksum*/
	AR5K_EEREAD	= 5,	/* Unable to get device caps from EEPROM */
	AR5K_EEBADMAC	= 6,	/* Unable to read MAC address from EEPROM */
	AR5K_EINVAL	= 7,	/* Invalid parameter to function */
	AR5K_ENOTSUPP	= 8,	/* Hardware revision not supported */
	AR5K_EINPROGRESS= 9,	/* Unexpected error ocured during process */
} AR5K_STATUS;

/*
 * Some tuneable values (these should be changeable by the user)
 */
#define AR5K_TUNE_DMA_BEACON_RESP		2
#define AR5K_TUNE_SW_BEACON_RESP		10
#define AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF	0
#define AR5K_TUNE_RADAR_ALERT			FALSE
#define AR5K_TUNE_MIN_TX_FIFO_THRES		1
#define AR5K_TUNE_MAX_TX_FIFO_THRES		((MAX_PDU_LENGTH / 64) + 1)
#define AR5K_TUNE_RSSI_THRES			1792
#define AR5K_TUNE_REGISTER_TIMEOUT		20000
#define AR5K_TUNE_REGISTER_DWELL_TIME		20000
#define AR5K_TUNE_BEACON_INTERVAL		100
#define AR5K_TUNE_AIFS				2
#define AR5K_TUNE_AIFS_11B			2
#define AR5K_TUNE_AIFS_XR			0
#define AR5K_TUNE_CWMIN				15
#define AR5K_TUNE_CWMIN_11B			31
#define AR5K_TUNE_CWMIN_XR			3
#define AR5K_TUNE_CWMAX				1023
#define AR5K_TUNE_CWMAX_11B			1023
#define AR5K_TUNE_CWMAX_XR			7
#define AR5K_TUNE_NOISE_FLOOR			-72
#define AR5K_TUNE_MAX_TXPOWER			60
#define AR5K_TUNE_DEFAULT_TXPOWER		30
#define AR5K_TUNE_TPC_TXPOWER			TRUE
#define AR5K_TUNE_ANT_DIVERSITY			TRUE
#define AR5K_TUNE_HWTXTRIES			4

/* token to use for aifs, cwmin, cwmax in MadWiFi */
#define	AR5K_TXQ_USEDEFAULT	((u_int32_t) -1)

#define IEEE80211_ADDR_LEN      6       /* size of 802.11 address */
#define ETHER_ADDR_LEN          6       /* length of an Ethernet address */
static const u_char etherbroadcastaddr[ETHER_ADDR_LEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//#define etherbroadcastaddr 0xff




/*****************************\
  GENERIC CHIPSET DEFINITIONS
\*****************************/

/* MAC Chips*/
enum ar5k_version {
	AR5K_AR5210	= 0,
	AR5K_AR5211	= 1,
	AR5K_AR5212	= 2,
};

/*PHY Chips*/
enum ar5k_radio {
	AR5K_AR5110	= 0,
	AR5K_AR5111	= 1,
	AR5K_AR5112	= 2,
};

/*
 * Common silicon revision/version values
 */
enum ar5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_REV,
	AR5K_VERSION_RAD,
	AR5K_VERSION_DEV
};

struct ar5k_srev_name {
	const char		*sr_name;
	enum ar5k_srev_type	sr_type;
	u_int			sr_val;
};

#define AR5K_SREV_NAME	{						\
	{ "5210",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5210 },	\
	{ "5311",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311 },	\
	{ "5311a",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311A },\
	{ "5311b",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311B },\
	{ "5211",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5211 },	\
	{ "5212",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5212 },	\
	{ "5213",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213 },	\
	{ "xxxx",	AR5K_VERSION_VER,	AR5K_SREV_UNKNOWN },	\
	{ "5110",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },	\
	{ "5111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },	\
	{ "2111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },	\
	{ "5112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },	\
	{ "5112a",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },	\
	{ "2112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },	\
	{ "2112a",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },	\
	{ "xxxx",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },	\
	{ "2413",	AR5K_VERSION_DEV,	AR5K_DEVID_AR2413 },	\
 	{ "5413",	AR5K_VERSION_DEV,	AR5K_DEVID_AR5413 },	\
 	{ "5424",	AR5K_VERSION_DEV,	AR5K_DEVID_AR5424 },	\
 	{ "xxxx",	AR5K_VERSION_DEV,	AR5K_SREV_UNKNOWN }	\
}

#define AR5K_SREV_UNKNOWN	0xffff

#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_UNSUPP	0x60

#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_UNSUPP	0x50




/****************\
  TX DEFINITIONS
\****************/

/*
 * Tx Descriptor
 */
struct ath_tx_status {
	u_int16_t	ts_seqnum;
	u_int16_t	ts_tstamp;
	u_int8_t	ts_status;
	u_int8_t	ts_rate;
	int8_t		ts_rssi;
	u_int8_t	ts_shortretry;
	u_int8_t	ts_longretry;
	u_int8_t	ts_virtcol;
	u_int8_t	ts_antenna;
};

#define AR5K_TXSTAT_ALTRATE	0x80
#define AR5K_TXERR_XRETRY	0x01
#define AR5K_TXERR_FILT		0x02
#define AR5K_TXERR_FIFO		0x04

/*
 * Queue types used to classify tx queues.
 */
typedef enum {
	AR5K_TX_QUEUE_INACTIVE = 0,/*This queue is not used -see ath_hal_releasetxqueue*/
	AR5K_TX_QUEUE_DATA,	  /*A normal data queue*/
	AR5K_TX_QUEUE_XR_DATA,	  /*An XR-data queue*/
	AR5K_TX_QUEUE_BEACON,	  /*The beacon queue*/
	AR5K_TX_QUEUE_CAB,	  /*The ater-beacon queue*/
	AR5K_TX_QUEUE_UAPSD,	  /*Unscheduled Automatic Power Save Delivery queue*/
} AR5K_TX_QUEUE;

#define	AR5K_NUM_TX_QUEUES	10

/*
 * Queue syb-types to classify normal data queues.
 * These are the 4 Access Categories as defined in
 * WME spec. 0 is the lowest priority and 4 is the
 * highest. Normal data that hasn't been classified
 * goes to the Best Effort AC.
 */
typedef enum {
	AR5K_WME_AC_BK = 0,	/*Background traffic*/
	AR5K_WME_AC_BE, 	/*Best-effort (normal) traffic)*/
	AR5K_WME_AC_VI, 	/*Video traffic*/
	AR5K_WME_AC_VO, 	/*Voice traffic*/
} AR5K_TX_QUEUE_SUBTYPE;

/*
 * Queue ID numbers as returned by the HAL, each number
 * represents a hw queue. If hw does not support hw queues
 * (eg 5210/5211) all data goes in one queue. These match
 * d80211 definitions (net80211/MadWiFi don't use them).
 */
typedef enum {
	AR5K_TX_QUEUE_ID_DATA_MIN = 0, /*IEEE80211_TX_QUEUE_DATA0*/
	AR5K_TX_QUEUE_ID_DATA_MAX = 4, /*IEEE80211_TX_QUEUE_DATA4*/
	AR5K_TX_QUEUE_ID_DATA_SVP = 5, /*IEEE80211_TX_QUEUE_SVP - Spectralink Voice Protocol*/
	AR5K_TX_QUEUE_ID_CAB	  = 6, /*IEEE80211_TX_QUEUE_AFTER_BEACON*/
	AR5K_TX_QUEUE_ID_BEACON	  = 7, /*IEEE80211_TX_QUEUE_BEACON*/
	AR5K_TX_QUEUE_ID_UAPSD	  = 8,
	AR5K_TX_QUEUE_ID_XR_DATA  = 9,
} AR5K_TX_QUEUE_ID;


/*
 * Flags to set hw queue's parameters...
 */
#define AR5K_TXQ_FLAG_TXINT_ENABLE		0x0001	/* Enable TXOK and TXERR interrupts -not used- */
#define AR5K_TXQ_FLAG_TXDESCINT_ENABLE		0x0002	/* Enable TXDESC interrupt -not implemented- */
#define AR5K_TXQ_FLAG_BACKOFF_DISABLE		0x0004	/* Disable random post-backoff */
#define AR5K_TXQ_FLAG_COMPRESSION_ENABLE	0x0008	/* Enable hw compression -not implemented-*/
#define AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE	0x0010	/* Enable ready time expiry policy (?)*/
#define AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE	0x0020	/* Enable backoff while bursting */
#define AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS		0x0040	/* Disable backoff while bursting */
#define AR5K_TXQ_FLAG_TXEOLINT_ENABLE		0x0080	/* Enable TXEOL interrupt -not implemented-*/

/*
 * A struct to hold tx queue's parameters
 */
typedef struct {
	AR5K_TX_QUEUE			tqi_type;	/* See AR5K_TX_QUEUE */
	AR5K_TX_QUEUE_SUBTYPE		tqi_subtype;	/* See AR5K_TX_QUEUE_SUBTYPE */
	u_int16_t			tqi_flags;	/* Tx queue flags (see above) */
	u_int32_t			tqi_aifs;	/* Arbitrated Interframe Space */
	int32_t				tqi_cw_min;	/* Minimum Contention Window */
	int32_t				tqi_cw_max;	/* Maximum Contention Window */
	u_int32_t			tqi_cbr_period; /* Constant bit rate period */
	u_int32_t			tqi_cbr_overflow_limit;
	u_int32_t			tqi_burst_time;
	u_int32_t			tqi_ready_time; /* Not used */
	u_int32_t			tqi_comp_buffer;/* Compression Buffer's phys addr */
} AR5K_TXQ_INFO;

/*
 * Transmit packet types.
 * These are not fully used inside OpenHAL yet
 */
typedef enum {
	AR5K_PKT_TYPE_NORMAL 		= 0,
	AR5K_PKT_TYPE_ATIM 		= 1,
	AR5K_PKT_TYPE_PSPOLL 		= 2,
	AR5K_PKT_TYPE_BEACON 		= 3,
	AR5K_PKT_TYPE_PROBE_RESP 	= 4,
	AR5K_PKT_TYPE_PIFS 		= 5,
} AR5K_PKT_TYPE;

/*
 * TX power and TPC settings
 */
#define AR5K_TXPOWER_OFDM(_r, _v)	(			\
	((0 & 1) << ((_v) + 6)) |				\
	(((hal->ah_txpower.txp_rates[(_r)]) & 0x3f) << (_v))	\
)

#define AR5K_TXPOWER_CCK(_r, _v)	(			\
	(hal->ah_txpower.txp_rates[(_r)] & 0x3f) << (_v)	\
)

/*
 * Used to compute TX times
 */
#define AR5K_CCK_SIFS_TIME		10
#define AR5K_CCK_PREAMBLE_BITS		144
#define AR5K_CCK_PLCP_BITS		48

#define AR5K_OFDM_SIFS_TIME		16
#define AR5K_OFDM_PREAMBLE_TIME		20
#define AR5K_OFDM_PLCP_BITS		22
#define AR5K_OFDM_SYMBOL_TIME		4

#define AR5K_TURBO_SIFS_TIME		8
#define AR5K_TURBO_PREAMBLE_TIME	14
#define AR5K_TURBO_PLCP_BITS		22
#define AR5K_TURBO_SYMBOL_TIME		4

#define AR5K_XR_SIFS_TIME		16
#define AR5K_XR_PLCP_BITS		22
#define AR5K_XR_SYMBOL_TIME		4

/* CCK */
#define AR5K_CCK_NUM_BITS(_frmlen) (_frmlen << 3)

#define AR5K_CCK_PHY_TIME(_sp) (_sp ?					\
	((AR5K_CCK_PREAMBLE_BITS + AR5K_CCK_PLCP_BITS) >> 1) :		\
	(AR5K_CCK_PREAMBLE_BITS + AR5K_CCK_PLCP_BITS))

#define AR5K_CCK_TX_TIME(_kbps, _frmlen, _sp)				\
	AR5K_CCK_PHY_TIME(_sp) +					\
	((AR5K_CCK_NUM_BITS(_frmlen) * 1000) / _kbps) +		\
	AR5K_CCK_SIFS_TIME

/* OFDM */
#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_OFDM_SYMBOL_TIME) / 1000)

#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_OFDM_NUM_BITS(_frmlen), AR5K_OFDM_NUM_BITS_PER_SYM(_kbps))

#define AR5K_OFDM_TX_TIME(_kbps, _frmlen)				\
	AR5K_OFDM_PREAMBLE_TIME + AR5K_OFDM_SIFS_TIME +			\
	(AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_OFDM_SYMBOL_TIME)

/* TURBO */
#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_BITS_PER_SYM(_kbps) (((_kbps << 1) *		\
	AR5K_TURBO_SYMBOL_TIME) / 1000)

#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_TURBO_NUM_BITS(_frmlen),				\
	AR5K_TURBO_NUM_BITS_PER_SYM(_kbps))

#define AR5K_TURBO_TX_TIME(_kbps, _frmlen)				\
	AR5K_TURBO_PREAMBLE_TIME + AR5K_TURBO_SIFS_TIME +		\
	(AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_TURBO_SYMBOL_TIME)

/* eXtendent Range (?)*/
#define AR5K_XR_PREAMBLE_TIME(_kbps) (((_kbps) < 1000) ? 173 : 76)

#define AR5K_XR_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_XR_SYMBOL_TIME) / 1000)

#define AR5K_XR_NUM_BITS(_frmlen) (AR5K_XR_PLCP_BITS + (_frmlen << 3))

#define AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_XR_NUM_BITS(_frmlen), AR5K_XR_NUM_BITS_PER_SYM(_kbps))

#define AR5K_XR_TX_TIME(_kbps, _frmlen)				\
	AR5K_XR_PREAMBLE_TIME(_kbps) + AR5K_XR_SIFS_TIME +		\
	(AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_XR_SYMBOL_TIME)




/****************\
  RX DEFINITIONS
\****************/

/*
 * Rx Descriptor
 */
struct ath_rx_status {
	u_int16_t	rs_datalen;
	u_int16_t	rs_tstamp;
	u_int8_t	rs_status;
	u_int8_t	rs_phyerr;
	int8_t		rs_rssi;
	u_int8_t	rs_keyix;
	u_int8_t	rs_rate;
	u_int8_t	rs_antenna;
	u_int8_t	rs_more;
};

#define AR5K_RXERR_CRC		0x01
#define AR5K_RXERR_PHY		0x02
#define AR5K_RXERR_FIFO		0x04
#define AR5K_RXERR_DECRYPT	0x08
#define AR5K_RXERR_MIC		0x10
#define AR5K_RXKEYIX_INVALID	((u_int8_t) - 1)
#define AR5K_TXKEYIX_INVALID	((u_int32_t) - 1)

/*
 * RX filters
 * Most of them are not yet used inside OpenHAL
 */
#define	AR5K_RX_FILTER_UCAST 		0x00000001	/* Don't filter unicast frames */
#define	AR5K_RX_FILTER_MCAST 		0x00000002	/* Don't filter multicast frames */
#define	AR5K_RX_FILTER_BCAST 		0x00000004	/* Don't filter broadcast frames */
#define	AR5K_RX_FILTER_CONTROL 		0x00000008	/* Don't filter control frames */
#define	AR5K_RX_FILTER_BEACON 		0x00000010	/* Don't filter beacon frames */
#define	AR5K_RX_FILTER_PROM 		0x00000020	/* Set promiscuous mode */
#define	AR5K_RX_FILTER_XRPOLL 		0x00000040	/* Don't filter XR poll frame */
#define	AR5K_RX_FILTER_PROBEREQ 	0x00000080	/* Don't filter probe requests */
#define	AR5K_RX_FILTER_PHYERR 		0x00000100	/* Don't filter phy errors */
#define	AR5K_RX_FILTER_PHYRADAR 	0x00000200	/* Don't filter phy radar errors*/

typedef struct {
	u_int32_t	ackrcv_bad;
	u_int32_t	rts_bad;
	u_int32_t	rts_good;
	u_int32_t	fcs_bad;
	u_int32_t	beacons;
} AR5K_MIB_STATS;




/**************************\
 BEACON TIMERS DEFINITIONS
\**************************/

#define AR5K_BEACON_PERIOD	0x0000ffff
#define AR5K_BEACON_ENA		0x00800000 /*enable beacon xmit*/
#define AR5K_BEACON_RESET_TSF	0x01000000 /*force a TSF reset*/

/*
 * Per-station beacon timer state.
 */
typedef struct {
	u_int32_t	bs_next_beacon;
	u_int32_t	bs_next_dtim;
	u_int32_t	bs_interval;		/*in TU's -see net80211/ieee80211_var.h-
						can also include the above flags*/
	u_int8_t	bs_dtim_period;
	u_int8_t	bs_cfp_period;
	u_int16_t	bs_cfp_max_duration;	/*if non-zero hw is setup to coexist with
						a Point Coordination Function capable AP*/
	u_int16_t	bs_cfp_du_remain;
	u_int16_t	bs_tim_offset;
	u_int16_t	bs_sleep_duration;
	u_int16_t	bs_bmiss_threshold;
	u_int32_t  	bs_cfp_next;
} AR5K_BEACON_STATE;




/********************\
  COMMON DEFINITIONS
\********************/

/*
 * Atheros descriptor
 */
struct ath_desc {
	u_int32_t	ds_link;
	u_int32_t	ds_data;
	u_int32_t	ds_ctl0;
	u_int32_t	ds_ctl1;
	u_int32_t	ds_hw[4];

	union {
		struct ath_rx_status rx;
		struct ath_tx_status tx;
	} ds_us;

#define ds_rxstat ds_us.rx
#define ds_txstat ds_us.tx

} __packed;

#define AR5K_RXDESC_INTREQ	0x0020

#define AR5K_TXDESC_CLRDMASK	0x0001
#define AR5K_TXDESC_NOACK	0x0002
#define AR5K_TXDESC_RTSENA	0x0004
#define AR5K_TXDESC_CTSENA	0x0008
#define AR5K_TXDESC_INTREQ	0x0010
#define AR5K_TXDESC_VEOL	0x0020

/*
 * 802.11 operating modes...
 */
#define AR5K_MODE_11A  	0x01
#define AR5K_MODE_11B  	0x02
#define AR5K_MODE_11G  	0x04
#define AR5K_MODE_TURBO	0x08
#define AR5K_MODE_108G 	0x16
#define AR5K_MODE_XR	0x32
#define AR5K_MODE_ALL 	(AR5K_MODE_11A|	\
			AR5K_MODE_11B|	\
			AR5K_MODE_11G|	\
			AR5K_MODE_TURBO|\
			AR5K_MODE_108G|	\
			AR5K_MODE_XR)

/*
 * Channel definitions
 */
typedef struct {
	u_int16_t	freq;		/* setting in Mhz */
	u_int16_t	channel_flags;
	u_int8_t	private_flags;	/* not used in OpenHAL yet*/
} AR5K_CHANNEL;

#define AR5K_SLOT_TIME_9	396
#define AR5K_SLOT_TIME_20	880
#define AR5K_SLOT_TIME_MAX	0xffff

/* channel_flags */
#define	CHANNEL_CW_INT	0x0008	/* Contention Window interference detected */
#define	CHANNEL_TURBO	0x0010	/* Turbo Channel */
#define	CHANNEL_CCK	0x0020	/* CCK channel */
#define	CHANNEL_OFDM	0x0040	/* OFDM channel */
#define	CHANNEL_2GHZ	0x0080	/* 2GHz channel. */
#define	CHANNEL_5GHZ	0x0100	/* 5GHz channel */
#define	CHANNEL_PASSIVE	0x0200	/* Only passive scan allowed */
#define	CHANNEL_DYN	0x0400	/* Dynamic CCK-OFDM channel (for g operation)*/
#define	CHANNEL_XR	0x0800	/* XR channel */

#define	CHANNEL_A	(CHANNEL_5GHZ|CHANNEL_OFDM)
#define	CHANNEL_B	(CHANNEL_2GHZ|CHANNEL_CCK)
#define	CHANNEL_PUREG	(CHANNEL_2GHZ|CHANNEL_OFDM)
//#ifdef notdef
#define	CHANNEL_G	(CHANNEL_2GHZ|CHANNEL_DYN)
//#else
//#define	CHANNEL_G	(CHANNEL_2GHZ|CHANNEL_OFDM)
//#endif
#define	CHANNEL_T	(CHANNEL_5GHZ|CHANNEL_OFDM|CHANNEL_TURBO)
#define	CHANNEL_TG	(CHANNEL_2GHZ|CHANNEL_OFDM|CHANNEL_TURBO)
#define	CHANNEL_108A	CHANNEL_T
#define	CHANNEL_108G	CHANNEL_TG
#define	CHANNEL_X	(CHANNEL_5GHZ|CHANNEL_OFDM|CHANNEL_XR)

#define	CHANNEL_ALL 	(CHANNEL_OFDM|CHANNEL_CCK| CHANNEL_2GHZ |\
			 CHANNEL_5GHZ | CHANNEL_TURBO)

#define	CHANNEL_ALL_NOTURBO 	(CHANNEL_ALL &~ CHANNEL_TURBO)
#define CHANNEL_MODES	CHANNEL_ALL

/*
 * Used internaly in OpenHAL (ar5211.c/ar5212.c
 * for reset_tx_queue). Also see struct AR5K_CHANNEL.
 */
#define IS_CHAN_XR(_c) \
        ((_c.channel_flags & CHANNEL_XR) != 0)

#define IS_CHAN_B(_c) \
        ((_c.channel_flags & CHANNEL_B) != 0)

typedef enum {
	AR5K_CHIP_5GHZ = CHANNEL_5GHZ,
	AR5K_CHIP_2GHZ = CHANNEL_2GHZ,
} AR5K_CHIP;

/*
 * The following structure will be used to map 2GHz channels to
 * 5GHz Atheros channels.
 */
struct ar5k_athchan_2ghz {
	u_int32_t	a2_flags;
	u_int16_t	a2_athchan;
};

/*
 * Rate definitions
 */

#define AR5K_MAX_RATES	32 /*max number of rates on the rate table*/

typedef struct {
	u_int8_t	valid;		/* Valid for rate control */
	u_int32_t	modulation;
	u_int16_t	rate_kbps;		
	u_int8_t	rate_code;	/* Rate mapping for h/w descriptors */
	u_int8_t	dot11_rate;
	u_int8_t	control_rate;
	u_int16_t	lp_ack_duration;/* long preamble ACK duration */
	u_int16_t	sp_ack_duration;/* short preamble ACK duration*/
} AR5K_RATE;

typedef struct {
	u_int16_t	rate_count;				
	u_int8_t	rate_code_to_index[AR5K_MAX_RATES];	/* Back-mapping */
	AR5K_RATE	rates[AR5K_MAX_RATES];
} AR5K_RATE_TABLE;

/*
 * Rate tables...
 */
#define AR5K_RATES_11A { 8, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 6, 4, 2, 0,	\
	7, 5, 3, 1, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, MODULATION_OFDM, 6000, 11, 140, 0 },		\
	{ 1, MODULATION_OFDM, 9000, 15, 18, 0 },		\
	{ 1, MODULATION_OFDM, 12000, 10, 152, 2 },		\
	{ 1, MODULATION_OFDM, 18000, 14, 36, 2 },		\
	{ 1, MODULATION_OFDM, 24000, 9, 176, 4 },		\
	{ 1, MODULATION_OFDM, 36000, 13, 72, 4 },		\
	{ 1, MODULATION_OFDM, 48000, 8, 96, 4 },		\
	{ 1, MODULATION_OFDM, 54000, 12, 108, 4 } }		\
}

#define AR5K_RATES_11B { 4, {						\
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,	\
	3, 2, 1, 0, 255, 255, 255, 255 }, {				\
	{ 1, MODULATION_CCK, 1000, 27, 130, 0 },	\
	{ 1, MODULATION_CCK, 2000, 26, 132, 1 },	\
	{ 1, MODULATION_CCK, 5500, 25, 139, 1 },	\
	{ 1, MODULATION_CCK, 11000, 24, 150, 1 } }	\
}

#define AR5K_RATES_11G { 12, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 10, 8, 6, 4,	\
	11, 9, 7, 5, 255, 255, 255, 255, 255, 255, 255, 255,	\
	3, 2, 1, 0, 255, 255, 255, 255 }, {			\
	{ 1, MODULATION_CCK, 1000, 27, 2, 0 },		\
	{ 1, MODULATION_CCK, 2000, 26, 4, 1 },		\
	{ 1, MODULATION_CCK, 5500, 25, 11, 1 },		\
	{ 1, MODULATION_CCK, 11000, 24, 22, 1 },	\
	{ 0, MODULATION_OFDM, 6000, 11, 12, 4 },	\
	{ 0, MODULATION_OFDM, 9000, 15, 18, 4 },	\
	{ 1, MODULATION_OFDM, 12000, 10, 24, 6 },	\
	{ 1, MODULATION_OFDM, 18000, 14, 36, 6 },	\
	{ 1, MODULATION_OFDM, 24000, 9, 48, 8 },	\
	{ 1, MODULATION_OFDM, 36000, 13, 72, 8 },	\
	{ 1, MODULATION_OFDM, 48000, 8, 96, 8 },	\
	{ 1, MODULATION_OFDM, 54000, 12, 108, 8 } }	\
}

#define AR5K_RATES_TURBO { 8, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 6, 4, 2, 0,	\
	7, 5, 3, 1, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, MODULATION_TURBO, 6000, 11, 140, 0 },	\
	{ 1, MODULATION_TURBO, 9000, 15, 18, 0 },	\
	{ 1, MODULATION_TURBO, 12000, 10, 152, 2 },	\
	{ 1, MODULATION_TURBO, 18000, 14, 36, 2 },	\
	{ 1, MODULATION_TURBO, 24000, 9, 176, 4 },	\
	{ 1, MODULATION_TURBO, 36000, 13, 72, 4 },	\
	{ 1, MODULATION_TURBO, 48000, 8, 96, 4 },	\
	{ 1, MODULATION_TURBO, 54000, 12, 108, 4 } }	\
}

#define AR5K_RATES_XR { 12, {					\
	255, 3, 1, 255, 255, 255, 2, 0, 10, 8, 6, 4,		\
	11, 9, 7, 5, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, MODULATION_XR, 500, 7, 129, 0 },		\
	{ 1, MODULATION_XR, 1000, 2, 139, 1 },		\
	{ 1, MODULATION_XR, 2000, 6, 150, 2 },		\
	{ 1, MODULATION_XR, 3000, 1, 150, 3 },		\
	{ 1, MODULATION_OFDM, 6000, 11, 140, 4 },	\
	{ 1, MODULATION_OFDM, 9000, 15, 18, 4 },	\
	{ 1, MODULATION_OFDM, 12000, 10, 152, 6 },	\
	{ 1, MODULATION_OFDM, 18000, 14, 36, 6 },	\
	{ 1, MODULATION_OFDM, 24000, 9, 176, 8 },	\
	{ 1, MODULATION_OFDM, 36000, 13, 72, 8 },	\
	{ 1, MODULATION_OFDM, 48000, 8, 96, 8 },	\
	{ 1, MODULATION_OFDM, 54000, 12, 108, 8 } }	\
}

/*
 * Crypto definitions
 */

/* key types */
typedef enum {
	AR5K_CIPHER_WEP		= 0,
	AR5K_CIPHER_AES_OCB	= 1,
	AR5K_CIPHER_AES_CCM	= 2,
	AR5K_CIPHER_CKIP	= 3,
	AR5K_CIPHER_TKIP	= 4,
	AR5K_CIPHER_CLR		= 5,	/* no encryption */
	AR5K_CIPHER_MIC		= 127	/* used for Message 
					   Integrity Code */
} AR5K_CIPHER;

#define AR5K_KEYVAL_LENGTH_40	5
#define AR5K_KEYVAL_LENGTH_104	13
#define AR5K_KEYVAL_LENGTH_128	16
#define AR5K_KEYVAL_LENGTH_MAX	AR5K_KEYVAL_LENGTH_128

typedef struct {
	int		wk_len;		/* key's length */
	u_int8_t	wk_key[AR5K_KEYVAL_LENGTH_MAX];
	u_int8_t	wk_type;	/* see above */
	u_int8_t	wk_mic[8];	/* TKIP MIC key */
} AR5K_KEYVAL;



/***********************\
 HW RELATED DEFINITIONS
\***********************/

/*
 * Misc definitions
 */
#define	AR5K_RSSI_EP_MULTIPLIER	(1<<7)

#define AR5K_ASSERT_ENTRY(_e, _s) do {		\
	if (_e >= _s)				\
		return (FALSE);			\
} while (0)


typedef struct {
	u_int32_t	ns_avgbrssi;	/* average beacon rssi */
	u_int32_t	ns_avgrssi;	/* average data rssi */
	u_int32_t	ns_avgtxrssi;	/* average tx rssi */
} AR5K_NODE_STATS;

typedef enum {
	AR5K_ANT_VARIABLE	= 0,	/* variable by programming */
	AR5K_ANT_FIXED_A	= 1,	/* fixed to 11a frequencies */
	AR5K_ANT_FIXED_B	= 2,	/* fixed to 11b frequencies */
	AR5K_ANT_MAX		= 3,
} AR5K_ANT_SETTING;

/*
 * HAL interrupt abstraction
 */

/*
 * These are maped to take advantage of some common bits
 * between the MAC chips, to be able to set intr properties
 * easier. Some of them are not used yet inside OpenHAL.
 */
typedef enum {
	AR5K_INT_RX	= 0x00000001,
	AR5K_INT_RXDESC	= 0x00000002,
	AR5K_INT_RXNOFRM = 0x00000008,
	AR5K_INT_RXEOL	= 0x00000010,
	AR5K_INT_RXORN	= 0x00000020,
	AR5K_INT_TX	= 0x00000040,
	AR5K_INT_TXDESC	= 0x00000080,
	AR5K_INT_TXURN	= 0x00000800,
	AR5K_INT_MIB	= 0x00001000,
	AR5K_INT_RXPHY	= 0x00004000,
	AR5K_INT_RXKCM	= 0x00008000,
	AR5K_INT_SWBA	= 0x00010000,
	AR5K_INT_BMISS	= 0x00040000,
	AR5K_INT_BNR	= 0x00100000,
	AR5K_INT_GPIO	= 0x01000000,
	AR5K_INT_FATAL	= 0x40000000,
	AR5K_INT_GLOBAL	= 0x80000000,

	/*A sum of all the common bits*/
	AR5K_INT_COMMON  = AR5K_INT_RXNOFRM
			| AR5K_INT_RXDESC
			| AR5K_INT_RXEOL
			| AR5K_INT_RXORN
			| AR5K_INT_TXURN
			| AR5K_INT_TXDESC
			| AR5K_INT_MIB
			| AR5K_INT_RXPHY
			| AR5K_INT_RXKCM
			| AR5K_INT_SWBA
			| AR5K_INT_BMISS
			| AR5K_INT_GPIO,
	AR5K_INT_NOCARD	= 0xffffffff /*Declare that the card 
				       has been removed*/
} AR5K_INT;

/*
 * Power management
 */
typedef enum {
	AR5K_PM_UNDEFINED = 0,
	AR5K_PM_AUTO,
	AR5K_PM_AWAKE,
	AR5K_PM_FULL_SLEEP,
	AR5K_PM_NETWORK_SLEEP,
} AR5K_POWER_MODE;


/*
 * LED states
 */
typedef int AR5K_LED_STATE;

/*
 * These match net80211 definitions (not used in
 * d80211).
 */
#define AR5K_LED_INIT	0 /*IEEE80211_S_INIT*/
#define AR5K_LED_SCAN	1 /*IEEE80211_S_SCAN*/
#define AR5K_LED_AUTH	2 /*IEEE80211_S_AUTH*/
#define AR5K_LED_ASSOC	3 /*IEEE80211_S_ASSOC*/
#define AR5K_LED_RUN	4 /*IEEE80211_S_RUN*/

/* GPIO-controlled software LED */
#define AR5K_SOFTLED_PIN	0
#define AR5K_SOFTLED_ON		0
#define AR5K_SOFTLED_OFF	1

/*
 * Gain settings
 */
typedef enum {
	AR5K_RFGAIN_INACTIVE = 0,
	AR5K_RFGAIN_READ_REQUESTED,
	AR5K_RFGAIN_NEED_CHANGE,
} AR5K_RFGAIN;

#define AR5K_GAIN_CRN_FIX_BITS_5111		4
#define AR5K_GAIN_CRN_FIX_BITS_5112		7
#define AR5K_GAIN_CRN_MAX_FIX_BITS		AR5K_GAIN_CRN_FIX_BITS_5112
#define AR5K_GAIN_DYN_ADJUST_HI_MARGIN		15
#define AR5K_GAIN_DYN_ADJUST_LO_MARGIN		20
#define AR5K_GAIN_CCK_PROBE_CORR		5
#define AR5K_GAIN_CCK_OFDM_GAIN_DELTA		15
#define AR5K_GAIN_STEP_COUNT			10
#define AR5K_GAIN_PARAM_TX_CLIP			0
#define AR5K_GAIN_PARAM_PD_90			1
#define AR5K_GAIN_PARAM_PD_84			2
#define AR5K_GAIN_PARAM_GAIN_SEL		3
#define AR5K_GAIN_PARAM_MIX_ORN			0
#define AR5K_GAIN_PARAM_PD_138			1
#define AR5K_GAIN_PARAM_PD_137			2
#define AR5K_GAIN_PARAM_PD_136			3
#define AR5K_GAIN_PARAM_PD_132			4
#define AR5K_GAIN_PARAM_PD_131			5
#define AR5K_GAIN_PARAM_PD_130			6
#define AR5K_GAIN_CHECK_ADJUST(_g) 		\
	((_g)->g_current <= (_g)->g_low || (_g)->g_current >= (_g)->g_high)

struct ar5k_gain_opt_step {
	int16_t				gos_param[AR5K_GAIN_CRN_MAX_FIX_BITS];
	int32_t				gos_gain;
};

struct ar5k_gain_opt {
	u_int32_t			go_default;
	u_int32_t			go_steps_count;
	const struct ar5k_gain_opt_step	go_step[AR5K_GAIN_STEP_COUNT];
};

struct ar5k_gain {
	u_int32_t			g_step_idx;
	u_int32_t			g_current;
	u_int32_t			g_target;
	u_int32_t			g_low;
	u_int32_t			g_high;
	u_int32_t			g_f_corr;
	u_int32_t			g_active;
	const struct ar5k_gain_opt_step	*g_step;
};

/*
 * Gain optimization tables...
 */
#define AR5K_AR5111_GAIN_OPT	{		\
	4,					\
	9,					\
	{					\
		{ { 4, 1, 1, 1 }, 6 },		\
		{ { 4, 0, 1, 1 }, 4 },		\
		{ { 3, 1, 1, 1 }, 3 },		\
		{ { 4, 0, 0, 1 }, 1 },		\
		{ { 4, 1, 1, 0 }, 0 },		\
		{ { 4, 0, 1, 0 }, -2 },		\
		{ { 3, 1, 1, 0 }, -3 },		\
		{ { 4, 0, 0, 0 }, -4 },		\
		{ { 2, 1, 1, 0 }, -6 }		\
	}					\
}

#define AR5K_AR5112_GAIN_OPT	{			\
	1,						\
	8,						\
	{						\
		{ { 3, 0, 0, 0, 0, 0, 0 }, 6 },		\
		{ { 2, 0, 0, 0, 0, 0, 0 }, 0 },		\
		{ { 1, 0, 0, 0, 0, 0, 0 }, -3 },	\
		{ { 0, 0, 0, 0, 0, 0, 0 }, -6 },	\
		{ { 0, 1, 1, 0, 0, 0, 0 }, -8 },	\
		{ { 0, 1, 1, 0, 1, 1, 0 }, -10 },	\
		{ { 0, 1, 0, 1, 1, 1, 0 }, -13 },	\
		{ { 0, 1, 0, 1, 1, 0, 1 }, -16 },	\
	}						\
}

/*
 * Common ar5xxx EEPROM data registers
 */
#define AR5K_EEPROM_MAGIC		0x003d
#define AR5K_EEPROM_MAGIC_VALUE		0x5aa5
#define AR5K_EEPROM_PROTECT		0x003f
#define AR5K_EEPROM_PROTECT_RD_0_31	0x0001
#define AR5K_EEPROM_PROTECT_WR_0_31	0x0002
#define AR5K_EEPROM_PROTECT_RD_32_63	0x0004
#define AR5K_EEPROM_PROTECT_WR_32_63	0x0008
#define AR5K_EEPROM_PROTECT_RD_64_127	0x0010
#define AR5K_EEPROM_PROTECT_WR_64_127	0x0020
#define AR5K_EEPROM_PROTECT_RD_128_191	0x0040
#define AR5K_EEPROM_PROTECT_WR_128_191	0x0080
#define AR5K_EEPROM_PROTECT_RD_192_207	0x0100
#define AR5K_EEPROM_PROTECT_WR_192_207	0x0200
#define AR5K_EEPROM_PROTECT_RD_208_223	0x0400
#define AR5K_EEPROM_PROTECT_WR_208_223	0x0800
#define AR5K_EEPROM_PROTECT_RD_224_239	0x1000
#define AR5K_EEPROM_PROTECT_WR_224_239	0x2000
#define AR5K_EEPROM_PROTECT_RD_240_255	0x4000
#define AR5K_EEPROM_PROTECT_WR_240_255	0x8000
#define AR5K_EEPROM_REG_DOMAIN		0x00bf
#define AR5K_EEPROM_INFO_BASE		0x00c0
#define AR5K_EEPROM_INFO_MAX		(0x400 - AR5K_EEPROM_INFO_BASE)
#define AR5K_EEPROM_INFO_CKSUM		0xffff
#define AR5K_EEPROM_INFO(_n)		(AR5K_EEPROM_INFO_BASE + (_n))

#define AR5K_EEPROM_VERSION		AR5K_EEPROM_INFO(1)
#define AR5K_EEPROM_VERSION_3_0		0x3000
#define AR5K_EEPROM_VERSION_3_1		0x3001
#define AR5K_EEPROM_VERSION_3_2		0x3002
#define AR5K_EEPROM_VERSION_3_3		0x3003
#define AR5K_EEPROM_VERSION_3_4		0x3004
#define AR5K_EEPROM_VERSION_4_0		0x4000
#define AR5K_EEPROM_VERSION_4_1		0x4001
#define AR5K_EEPROM_VERSION_4_2		0x4002
#define AR5K_EEPROM_VERSION_4_3		0x4003
#define AR5K_EEPROM_VERSION_4_6		0x4006
#define AR5K_EEPROM_VERSION_4_7		0x3007

#define AR5K_EEPROM_MODE_11A		0
#define AR5K_EEPROM_MODE_11B		1
#define AR5K_EEPROM_MODE_11G		2

#define AR5K_EEPROM_HDR			AR5K_EEPROM_INFO(2)
#define AR5K_EEPROM_HDR_11A(_v)		(((_v) >> AR5K_EEPROM_MODE_11A) & 0x1)
#define AR5K_EEPROM_HDR_11B(_v)		(((_v) >> AR5K_EEPROM_MODE_11B) & 0x1)
#define AR5K_EEPROM_HDR_11G(_v)		(((_v) >> AR5K_EEPROM_MODE_11G) & 0x1)
#define AR5K_EEPROM_HDR_T_2GHZ_DIS(_v)	(((_v) >> 3) & 0x1)
#define AR5K_EEPROM_HDR_T_5GHZ_DBM(_v)	(((_v) >> 4) & 0x7f)
#define AR5K_EEPROM_HDR_DEVICE(_v)	(((_v) >> 11) & 0x7)
#define AR5K_EEPROM_HDR_T_5GHZ_DIS(_v)	(((_v) >> 15) & 0x1)
#define AR5K_EEPROM_HDR_RFKILL(_v)	(((_v) >> 14) & 0x1)

#define AR5K_EEPROM_RFKILL_GPIO_SEL	0x0000001c
#define AR5K_EEPROM_RFKILL_GPIO_SEL_S	2
#define AR5K_EEPROM_RFKILL_POLARITY	0x00000002
#define AR5K_EEPROM_RFKILL_POLARITY_S	1

/* Newer EEPROMs are using a different offset */
#define AR5K_EEPROM_OFF(_v, _v3_0, _v3_3) \
	(((_v) >= AR5K_EEPROM_VERSION_3_3) ? _v3_3 : _v3_0)

#define AR5K_EEPROM_ANT_GAIN(_v)	AR5K_EEPROM_OFF(_v, 0x00c4, 0x00c3)
#define AR5K_EEPROM_ANT_GAIN_5GHZ(_v)	((int8_t)(((_v) >> 8) & 0xff))
#define AR5K_EEPROM_ANT_GAIN_2GHZ(_v)	((int8_t)((_v) & 0xff))

#define AR5K_EEPROM_MODES_11A(_v)	AR5K_EEPROM_OFF(_v, 0x00c5, 0x00d4)
#define AR5K_EEPROM_MODES_11B(_v)	AR5K_EEPROM_OFF(_v, 0x00d0, 0x00f2)
#define AR5K_EEPROM_MODES_11G(_v)	AR5K_EEPROM_OFF(_v, 0x00da, 0x010d)
#define AR5K_EEPROM_CTL(_v)		AR5K_EEPROM_OFF(_v, 0x00e4, 0x0128)

/* Since 3.1 */
#define AR5K_EEPROM_OBDB0_2GHZ	0x00ec
#define AR5K_EEPROM_OBDB1_2GHZ	0x00ed

/* Misc values available since EEPROM 4.0 */
#define AR5K_EEPROM_MISC0		0x00c4
#define AR5K_EEPROM_EARSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_EEMAP(_v)		(((_v) >> 14) & 0x3)
#define AR5K_EEPROM_MISC1		0x00c5
#define AR5K_EEPROM_TARGET_PWRSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_HAS32KHZCRYSTAL(_v)	(((_v) >> 14) & 0x1)

/* Some EEPROM defines */
#define AR5K_EEPROM_EEP_SCALE		100
#define AR5K_EEPROM_EEP_DELTA		10
#define AR5K_EEPROM_N_MODES		3
#define AR5K_EEPROM_N_5GHZ_CHAN		10
#define AR5K_EEPROM_N_2GHZ_CHAN		3
#define AR5K_EEPROM_MAX_CHAN		10
#define AR5K_EEPROM_N_PCDAC		11
#define AR5K_EEPROM_N_TEST_FREQ		8
#define AR5K_EEPROM_N_EDGES		8
#define AR5K_EEPROM_N_INTERCEPTS	11
#define AR5K_EEPROM_FREQ_M(_v)		AR5K_EEPROM_OFF(_v, 0x7f, 0xff)
#define AR5K_EEPROM_PCDAC_M		0x3f
#define AR5K_EEPROM_PCDAC_START		1
#define AR5K_EEPROM_PCDAC_STOP		63
#define AR5K_EEPROM_PCDAC_STEP		1
#define AR5K_EEPROM_NON_EDGE_M		0x40
#define AR5K_EEPROM_CHANNEL_POWER	8
#define AR5K_EEPROM_N_OBDB		4
#define AR5K_EEPROM_OBDB_DIS		0xffff
#define AR5K_EEPROM_CHANNEL_DIS		0xff
#define AR5K_EEPROM_SCALE_OC_DELTA(_x)	(((_x) * 2) / 10)
#define AR5K_EEPROM_N_CTLS(_v)		AR5K_EEPROM_OFF(_v, 16, 32)
#define AR5K_EEPROM_MAX_CTLS		32
#define AR5K_EEPROM_N_XPD_PER_CHANNEL	4
#define AR5K_EEPROM_N_XPD0_POINTS	4
#define AR5K_EEPROM_N_XPD3_POINTS	3
#define AR5K_EEPROM_N_INTERCEPT_10_2GHZ	35
#define AR5K_EEPROM_N_INTERCEPT_10_5GHZ	55
#define AR5K_EEPROM_POWER_M		0x3f
#define AR5K_EEPROM_POWER_MIN		0
#define AR5K_EEPROM_POWER_MAX		3150
#define AR5K_EEPROM_POWER_STEP		50
#define AR5K_EEPROM_POWER_TABLE_SIZE	64
#define AR5K_EEPROM_N_POWER_LOC_11B	4
#define AR5K_EEPROM_N_POWER_LOC_11G	6
#define AR5K_EEPROM_I_GAIN		10
#define AR5K_EEPROM_CCK_OFDM_DELTA	15
#define AR5K_EEPROM_N_IQ_CAL		2

struct ar5k_eeprom_info {
	u_int16_t	ee_magic;
	u_int16_t	ee_protect;
	u_int16_t	ee_regdomain;
	u_int16_t	ee_version;
	u_int16_t	ee_header;
	u_int16_t	ee_ant_gain;
	u_int16_t	ee_misc0;
	u_int16_t	ee_misc1;
	u_int16_t	ee_cck_ofdm_gain_delta;
	u_int16_t	ee_cck_ofdm_power_delta;
	u_int16_t	ee_scaled_cck_delta;
	u_int16_t	ee_tx_clip;
	u_int16_t	ee_pwd_84;
	u_int16_t	ee_pwd_90;
	u_int16_t	ee_gain_select;

	u_int16_t	ee_i_cal[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_q_cal[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_fixed_bias[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_turbo_max_power[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xr_power[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_switch_settling[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_ant_tx_rx[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_ant_control[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_PCDAC];
	u_int16_t	ee_ob[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_OBDB];
	u_int16_t	ee_db[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_OBDB];
	u_int16_t	ee_tx_end2xlna_enable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_tx_end2xpa_disable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_tx_frm2xpa_enable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_thr_62[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xlna_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xpd[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_x_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_i_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_margin_tx_rx[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_false_detect[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_cal_pier[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_2GHZ_CHAN];
	u_int16_t	ee_channel[AR5K_EEPROM_N_MODES][AR5K_EEPROM_MAX_CHAN];

	u_int16_t	ee_ctls;
	u_int16_t	ee_ctl[AR5K_EEPROM_MAX_CTLS];

	int16_t		ee_noise_floor_thr[AR5K_EEPROM_N_MODES];
	int8_t		ee_adc_desired_size[AR5K_EEPROM_N_MODES];
	int8_t		ee_pga_desired_size[AR5K_EEPROM_N_MODES];
};

/*
 * AR5k register access
 */

/*O.S. dependent functions are located in ah_osdep.h*/
#define AR5K_REG_SM(_val, _flags)					\
	(((_val) << _flags##_S) & (_flags))

#define AR5K_REG_MS(_val, _flags)					\
	(((_val) & (_flags)) >> _flags##_S)

#define AR5K_REG_WRITE_BITS(_reg, _flags, _val)				\
	AR5K_REG_WRITE(_reg, (AR5K_REG_READ(_reg) &~ (_flags)) |	\
	    (((_val) << _flags##_S) & (_flags)))

#define AR5K_REG_MASKED_BITS(_reg, _flags, _mask)			\
	AR5K_REG_WRITE(_reg, (AR5K_REG_READ(_reg) & (_mask)) | (_flags))

#define AR5K_REG_ENABLE_BITS(_reg, _flags)				\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) | (_flags))

#define AR5K_REG_DISABLE_BITS(_reg, _flags)				\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) &~ (_flags))

#define AR5K_PHY_WRITE(_reg, _val)					\
	AR5K_REG_WRITE(hal->ah_phy + ((_reg) << 2), _val)

#define AR5K_PHY_READ(_reg)						\
	AR5K_REG_READ(hal->ah_phy + ((_reg) << 2))

#define AR5K_REG_WAIT(_i)						\
	if (_i % 64)							\
		AR5K_DELAY(1);

#define AR5K_EEPROM_READ(_o, _v)	{				\
	if ((ret = hal->ah_eeprom_read(hal, (_o),			\
		 &(_v))) != 0)						\
		return (ret);						\
}

#define AR5K_EEPROM_READ_HDR(_o, _v)					\
	AR5K_EEPROM_READ(_o, hal->ah_capabilities.cap_eeprom._v);	\

/* Read status of selected queue */
#define AR5K_REG_READ_Q(_reg, _queue)					\
	(AR5K_REG_READ(_reg) & (1 << _queue))				\

#define AR5K_REG_WRITE_Q(_reg, _queue)					\
	AR5K_REG_WRITE(_reg, (1 << _queue))

#define AR5K_Q_ENABLE_BITS(_reg, _queue) do {				\
	_reg |= 1 << _queue;						\
} while (0)

#define AR5K_Q_DISABLE_BITS(_reg, _queue) do {				\
	_reg &= ~(1 << _queue);						\
} while (0)

/*
 * Unaligned little endian access
 */
#define AR5K_LE_READ_2(_p)						\
	(((const u_int8_t *)(_p))[0] | (((const u_int8_t *)(_p))[1] << 8))
#define AR5K_LE_READ_4(_p) \
	(((const u_int8_t *)(_p))[0] |					\
	(((const u_int8_t *)(_p))[1] << 8) |				\
	(((const u_int8_t *)(_p))[2] << 16) |				\
	(((const u_int8_t *)(_p))[3] << 24))
#define AR5K_LE_WRITE_2(_p, _val) \
	((((u_int8_t *)(_p))[0] = ((u_int32_t)(_val) & 0xff)),		\
	(((u_int8_t *)(_p))[1] = (((u_int32_t)(_val) >> 8) & 0xff)))
#define AR5K_LE_WRITE_4(_p, _val)					\
	((((u_int8_t *)(_p))[0] = ((u_int32_t)(_val) & 0xff)),		\
	(((u_int8_t *)(_p))[1] = (((u_int32_t)(_val) >> 8) & 0xff)),	\
	(((u_int8_t *)(_p))[2] = (((u_int32_t)(_val) >> 16) & 0xff)),	\
	(((u_int8_t *)(_p))[3] = (((u_int32_t)(_val) >> 24) & 0xff)))

/*
 * Initial register values
 */

/*
 * Common initial register values
 */
#define AR5K_INIT_MODE				CHANNEL_B

#define AR5K_INIT_TX_LATENCY			502
#define AR5K_INIT_USEC				39
#define AR5K_INIT_USEC_TURBO			79
#define AR5K_INIT_USEC_32			31
#define AR5K_INIT_CARR_SENSE_EN			1
#define AR5K_INIT_PROG_IFS			920
#define AR5K_INIT_PROG_IFS_TURBO		960
#define AR5K_INIT_EIFS				3440
#define AR5K_INIT_EIFS_TURBO			6880
#define AR5K_INIT_SLOT_TIME			396
#define AR5K_INIT_SLOT_TIME_TURBO		480
#define AR5K_INIT_ACK_CTS_TIMEOUT		1024
#define AR5K_INIT_ACK_CTS_TIMEOUT_TURBO		0x08000800
#define AR5K_INIT_SIFS				560
#define AR5K_INIT_SIFS_TURBO			480
#define AR5K_INIT_SH_RETRY			10
#define AR5K_INIT_LG_RETRY			AR5K_INIT_SH_RETRY
#define AR5K_INIT_SSH_RETRY			32
#define AR5K_INIT_SLG_RETRY			AR5K_INIT_SSH_RETRY
#define AR5K_INIT_TX_RETRY			10
#define AR5K_INIT_TOPS				8
#define AR5K_INIT_RXNOFRM			8
#define AR5K_INIT_RPGTO				0
#define AR5K_INIT_TXNOFRM			0
#define AR5K_INIT_BEACON_PERIOD			65535
#define AR5K_INIT_TIM_OFFSET			0
#define AR5K_INIT_BEACON_EN			0
#define AR5K_INIT_RESET_TSF			0

#define AR5K_INIT_TRANSMIT_LATENCY		(			\
	(AR5K_INIT_TX_LATENCY << 14) | (AR5K_INIT_USEC_32 << 7) |	\
	(AR5K_INIT_USEC)						\
)
#define AR5K_INIT_TRANSMIT_LATENCY_TURBO	(			\
	(AR5K_INIT_TX_LATENCY << 14) | (AR5K_INIT_USEC_32 << 7) |	\
	(AR5K_INIT_USEC_TURBO)						\
)
#define AR5K_INIT_PROTO_TIME_CNTRL		(			\
	(AR5K_INIT_CARR_SENSE_EN << 26) | (AR5K_INIT_EIFS << 12) |	\
	(AR5K_INIT_PROG_IFS)						\
)
#define AR5K_INIT_PROTO_TIME_CNTRL_TURBO	(			\
	(AR5K_INIT_CARR_SENSE_EN << 26) | (AR5K_INIT_EIFS_TURBO << 12) |\
	(AR5K_INIT_PROG_IFS_TURBO)					\
)
#define AR5K_INIT_BEACON_CONTROL		(			\
	(AR5K_INIT_RESET_TSF << 24) | (AR5K_INIT_BEACON_EN << 23) |	\
	(AR5K_INIT_TIM_OFFSET << 16) | (AR5K_INIT_BEACON_PERIOD)	\
)

/*
 * Non - common initial register values
 */
struct ar5k_ini {
	u_int16_t	ini_register;
	u_int32_t	ini_value;

	enum {
		AR5K_INI_WRITE = 0,
		AR5K_INI_READ = 1,
	} ini_mode;
};

#define AR5K_INI_VAL_11A		0
#define AR5K_INI_VAL_11A_TURBO		1
#define AR5K_INI_VAL_11B		2
#define AR5K_INI_VAL_11G		3
#define AR5K_INI_VAL_11G_TURBO		4
#define AR5K_INI_VAL_XR			0
#define AR5K_INI_VAL_MAX		5

#define AR5K_INI_PHY_5111		0
#define AR5K_INI_PHY_5112		1
#define AR5K_INI_PHY_511X		1

#define AR5K_AR5111_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS
#define AR5K_AR5112_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS

struct ar5k_ini_rf {
	u_int8_t	rf_bank;
	u_int16_t	rf_register;
	u_int32_t	rf_value[5];
};

#define AR5K_AR5111_INI_RF	{						\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00380000, 0x00380000, 0x00380000, 0x00380000, 0x00380000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x000000c0, 0x00000080, 0x00000080 } },	\
	{ 0, 0x989c,								\
	    { 0x000400f9, 0x000400f9, 0x000400ff, 0x000400fd, 0x000400fd } },	\
	{ 0, 0x98d4,								\
	    { 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 } },	\
	{ 1, 0x98d4,								\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d4,								\
	    { 0x00000010, 0x00000014, 0x00000010, 0x00000010, 0x00000014 } },	\
	{ 3, 0x98d8,								\
	    { 0x00601068, 0x00601068, 0x00601068, 0x00601068, 0x00601068 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x10000000, 0x10000000, 0x10000000, 0x10000000, 0x10000000 } },	\
	{ 6, 0x989c,								\
	    { 0x04000000, 0x04000000, 0x04000000, 0x04000000, 0x04000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x0a000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x003800c0, 0x00380080, 0x023800c0, 0x003800c0, 0x003800c0 } },	\
	{ 6, 0x989c,								\
	    { 0x00020006, 0x00020006, 0x00000006, 0x00020006, 0x00020006 } },	\
	{ 6, 0x989c,								\
	    { 0x00000089, 0x00000089, 0x00000089, 0x00000089, 0x00000089 } },	\
	{ 6, 0x989c,								\
	    { 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0 } },	\
	{ 6, 0x989c,								\
	    { 0x00040007, 0x00040007, 0x00040007, 0x00040007, 0x00040007 } },	\
	{ 6, 0x98d4,								\
	    { 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a } },	\
	{ 7, 0x989c,								\
	    { 0x00000040, 0x00000048, 0x00000040, 0x00000040, 0x00000040 } },	\
	{ 7, 0x989c,								\
	    { 0x00000010, 0x00000010, 0x00000010, 0x00000010, 0x00000010 } },	\
	{ 7, 0x989c,								\
	    { 0x00000008, 0x00000008, 0x00000008, 0x00000008, 0x00000008 } },	\
	{ 7, 0x989c,								\
	    { 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f } },	\
	{ 7, 0x989c,								\
	    { 0x000000f1, 0x000000f1, 0x00000061, 0x000000f1, 0x000000f1 } },	\
	{ 7, 0x989c,								\
	    { 0x0000904f, 0x0000904f, 0x0000904c, 0x0000904f, 0x0000904f } },	\
	{ 7, 0x989c,								\
	    { 0x0000125a, 0x0000125a, 0x0000129a, 0x0000125a, 0x0000125a } },	\
	{ 7, 0x98cc,								\
	    { 0x0000000e, 0x0000000e, 0x0000000f, 0x0000000e, 0x0000000e } },	\
}

#define AR5K_AR5112_INI_RF	{						\
	{ 1, 0x98d4,								\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d0,								\
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },	\
	{ 3, 0x98dc,								\
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },	\
	{ 6, 0x989c,								\
	    { 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000 } },	\
	{ 6, 0x989c,								\
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00660000, 0x00660000, 0x00660000, 0x00660000, 0x00660000 } },	\
	{ 6, 0x989c,								\
	    { 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0x00730000, 0x00730000, 0x00730000, 0x00730000, 0x00730000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00600000, 0x00600000, 0x00600000, 0x00600000, 0x00600000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00840000, 0x00840000, 0x00840000, 0x00840000, 0x00840000 } },	\
	{ 6, 0x989c,								\
	    { 0x00640000, 0x00640000, 0x00640000, 0x00640000, 0x00640000 } },	\
	{ 6, 0x989c,								\
	    { 0x00200000, 0x00200000, 0x00200000, 0x00200000, 0x00200000 } },	\
	{ 6, 0x989c,								\
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },	\
	{ 6, 0x989c,								\
	    { 0x00250000, 0x00250000, 0x00250000, 0x00250000, 0x00250000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x00510000, 0x00510000, 0x00510000, 0x00510000, 0x00510000 } },	\
	{ 6, 0x989c,								\
	    { 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000 } },	\
	{ 6, 0x989c,								\
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000 } },	\
	{ 6, 0x989c,								\
	    { 0x00400000, 0x00400000, 0x00400000, 0x00400000, 0x00400000 } },	\
	{ 6, 0x989c,								\
	    { 0x03090000, 0x03090000, 0x03090000, 0x03090000, 0x03090000 } },	\
	{ 6, 0x989c,								\
	    { 0x06000000, 0x06000000, 0x06000000, 0x06000000, 0x06000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000000b0, 0x000000b0, 0x000000a8, 0x000000a8, 0x000000a8 } },	\
	{ 6, 0x989c,								\
	    { 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e } },	\
	{ 6, 0x989c,								\
	    { 0x006c4a41, 0x006c4a41, 0x006c4af1, 0x006c4a61, 0x006c4a61 } },	\
	{ 6, 0x989c,								\
	    { 0x0050892a, 0x0050892a, 0x0050892b, 0x0050892b, 0x0050892b } },	\
	{ 6, 0x989c,								\
	    { 0x00842400, 0x00842400, 0x00842400, 0x00842400, 0x00842400 } },	\
	{ 6, 0x989c,								\
	    { 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200 } },	\
	{ 6, 0x98d0,								\
	    { 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c } },	\
	{ 7, 0x989c,								\
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },	\
	{ 7, 0x989c,								\
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },	\
	{ 7, 0x989c,								\
	    { 0x0000000a, 0x0000000a, 0x00000012, 0x00000012, 0x00000012 } },	\
	{ 7, 0x989c,								\
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },	\
	{ 7, 0x989c,								\
	    { 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1 } },	\
	{ 7, 0x989c,								\
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },	\
	{ 7, 0x989c,								\
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },	\
	{ 7, 0x989c,								\
	    { 0x00000022, 0x00000022, 0x00000022, 0x00000022, 0x00000022 } },	\
	{ 7, 0x989c,								\
	    { 0x00000092, 0x00000092, 0x00000092, 0x00000092, 0x00000092 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },	\
	{ 7, 0x989c,								\
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },	\
	{ 7, 0x989c,								\
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },	\
	{ 7, 0x98c4,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
	}
 	 
#define AR5K_AR5112A_INI_RF     {						\
	{ 1, 0x98d4,								\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d0,								\
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },	\
	{ 3, 0x98dc,								\
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },	\
	{ 6, 0x989c,								\
	    { 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000 } },	\
	{ 6, 0x989c,								\
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00010000, 0x00010000, 0x00010000, 0x00010000, 0x00010000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00180000, 0x00180000, 0x00180000, 0x00180000, 0x00180000 } },	\
	{ 6, 0x989c,								\
	    { 0x00600000, 0x00600000, 0x006e0000, 0x006e0000, 0x006e0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000 } },	\
	{ 6, 0x989c,								\
	    { 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000 } },	\
	{ 6, 0x989c,								\
	    { 0x04480000, 0x04480000, 0x04480000, 0x04480000, 0x04480000 } },	\
	{ 6, 0x989c,								\
	    { 0x00220000, 0x00220000, 0x00220000, 0x00220000, 0x00220000 } },	\
	{ 6, 0x989c,								\
	    { 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00190000, 0x00190000, 0x00190000, 0x00190000, 0x00190000 } },	\
	{ 6, 0x989c,								\
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },	\
	{ 6, 0x989c,								\
	    { 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000 } },	\
	{ 6, 0x989c,								\
	    { 0x00990000, 0x00990000, 0x00990000, 0x00990000, 0x00990000 } },	\
	{ 6, 0x989c,								\
	    { 0x00500000, 0x00500000, 0x00500000, 0x00500000, 0x00500000 } },	\
	{ 6, 0x989c,								\
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000 } },	\
	{ 6, 0x989c,								\
	    { 0x01740000, 0x01740000, 0x01740000, 0x01740000, 0x01740000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x86280000, 0x86280000, 0x86280000, 0x86280000, 0x86280000 } },	\
	{ 6, 0x989c,								\
	    { 0x31840000, 0x31840000, 0x31840000, 0x31840000, 0x31840000 } },	\
	{ 6, 0x989c,								\
	    { 0x00020080, 0x00020080, 0x00020080, 0x00020080, 0x00020080 } },	\
	{ 6, 0x989c,								\
	    { 0x00080009, 0x00080009, 0x00080009, 0x00080009, 0x00080009 } },	\
	{ 6, 0x989c,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2 } },	\
	{ 6, 0x989c,								\
	    { 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084 } },	\
	{ 6, 0x989c,								\
	    { 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4 } },	\
	{ 6, 0x989c,								\
	    { 0x00119220, 0x00119220, 0x00119220, 0x00119220, 0x00119220 } },	\
	{ 6, 0x989c,								\
	    { 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800 } },	\
	{ 6, 0x98d8,								\
	    { 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230 } },	\
	{ 7, 0x989c,								\
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },	\
	{ 7, 0x989c,								\
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },	\
	{ 7, 0x989c,								\
	    { 0x00000012, 0x00000012, 0x00000012, 0x00000012, 0x00000012 } },	\
	{ 7, 0x989c,								\
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9 } },	\
	{ 7, 0x989c,								\
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },	\
	{ 7, 0x989c,								\
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },	\
	{ 7, 0x989c,								\
	    { 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2 } },	\
	{ 7, 0x989c,								\
	    { 0x00000052, 0x00000052, 0x00000052, 0x00000052, 0x00000052 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },	\
	{ 7, 0x989c,								\
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },	\
	{ 7, 0x989c,								\
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },	\
	{ 7, 0x98c4,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
}

struct ar5k_ini_rfgain {
	u_int16_t	rfg_register;
	u_int32_t	rfg_value[2][2];

#define AR5K_INI_RFGAIN_5GHZ	0
#define AR5K_INI_RFGAIN_2GHZ	1
};

#define AR5K_INI_RFGAIN	{							\
	{ 0x9a00, {								\
		{ 0x000001a9, 0x00000000 }, { 0x00000007, 0x00000007 } } },	\
	{ 0x9a04, {								\
		{ 0x000001e9, 0x00000040 }, { 0x00000047, 0x00000047 } } },	\
	{ 0x9a08, {								\
		{ 0x00000029, 0x00000080 }, { 0x00000087, 0x00000087 } } },	\
	{ 0x9a0c, {								\
		{ 0x00000069, 0x00000150 }, { 0x000001a0, 0x000001a0 } } },	\
	{ 0x9a10, {								\
		{ 0x00000199, 0x00000190 }, { 0x000001e0, 0x000001e0 } } },	\
	{ 0x9a14, {								\
		{ 0x000001d9, 0x000001d0 }, { 0x00000020, 0x00000020 } } },	\
	{ 0x9a18, {								\
		{ 0x00000019, 0x00000010 }, { 0x00000060, 0x00000060 } } },	\
	{ 0x9a1c, {								\
		{ 0x00000059, 0x00000044 }, { 0x000001a1, 0x000001a1 } } },	\
	{ 0x9a20, {								\
		{ 0x00000099, 0x00000084 }, { 0x000001e1, 0x000001e1 } } },	\
	{ 0x9a24, {								\
		{ 0x000001a5, 0x00000148 }, { 0x00000021, 0x00000021 } } },	\
	{ 0x9a28, {								\
		{ 0x000001e5, 0x00000188 }, { 0x00000061, 0x00000061 } } },	\
	{ 0x9a2c, {								\
		{ 0x00000025, 0x000001c8 }, { 0x00000162, 0x00000162 } } },	\
	{ 0x9a30, {								\
		{ 0x000001c8, 0x00000014 }, { 0x000001a2, 0x000001a2 } } },	\
	{ 0x9a34, {								\
		{ 0x00000008, 0x00000042 }, { 0x000001e2, 0x000001e2 } } },	\
	{ 0x9a38, {								\
		{ 0x00000048, 0x00000082 }, { 0x00000022, 0x00000022 } } },	\
	{ 0x9a3c, {								\
		{ 0x00000088, 0x00000178 }, { 0x00000062, 0x00000062 } } },	\
	{ 0x9a40, {								\
		{ 0x00000198, 0x000001b8 }, { 0x00000163, 0x00000163 } } },	\
	{ 0x9a44, {								\
		{ 0x000001d8, 0x000001f8 }, { 0x000001a3, 0x000001a3 } } },	\
	{ 0x9a48, {								\
		{ 0x00000018, 0x00000012 }, { 0x000001e3, 0x000001e3 } } },	\
	{ 0x9a4c, {								\
		{ 0x00000058, 0x00000052 }, { 0x00000023, 0x00000023 } } },	\
	{ 0x9a50, {								\
		{ 0x00000098, 0x00000092 }, { 0x00000063, 0x00000063 } } },	\
	{ 0x9a54, {								\
		{ 0x000001a4, 0x0000017c }, { 0x00000184, 0x00000184 } } },	\
	{ 0x9a58, {								\
		{ 0x000001e4, 0x000001bc }, { 0x000001c4, 0x000001c4 } } },	\
	{ 0x9a5c, {								\
		{ 0x00000024, 0x000001fc }, { 0x00000004, 0x00000004 } } },	\
	{ 0x9a60, {								\
		{ 0x00000064, 0x0000000a }, { 0x000001ea, 0x0000000b } } },	\
	{ 0x9a64, {								\
		{ 0x000000a4, 0x0000004a }, { 0x0000002a, 0x0000004b } } },	\
	{ 0x9a68, {								\
		{ 0x000000e4, 0x0000008a }, { 0x0000006a, 0x0000008b } } },	\
	{ 0x9a6c, {								\
		{ 0x0000010a, 0x0000015a }, { 0x000000aa, 0x000001ac } } },	\
	{ 0x9a70, {								\
		{ 0x0000014a, 0x0000019a }, { 0x000001ab, 0x000001ec } } },	\
	{ 0x9a74, {								\
		{ 0x0000018a, 0x000001da }, { 0x000001eb, 0x0000002c } } },	\
	{ 0x9a78, {								\
		{ 0x000001ca, 0x0000000e }, { 0x0000002b, 0x00000012 } } },	\
	{ 0x9a7c, {								\
		{ 0x0000000a, 0x0000004e }, { 0x0000006b, 0x00000052 } } },	\
	{ 0x9a80, {								\
		{ 0x0000004a, 0x0000008e }, { 0x000000ab, 0x00000092 } } },	\
	{ 0x9a84, {								\
		{ 0x0000008a, 0x0000015e }, { 0x000001ac, 0x00000193 } } },	\
	{ 0x9a88, {								\
		{ 0x000001ba, 0x0000019e }, { 0x000001ec, 0x000001d3 } } },	\
	{ 0x9a8c, {								\
		{ 0x000001fa, 0x000001de }, { 0x0000002c, 0x00000013 } } },	\
	{ 0x9a90, {								\
		{ 0x0000003a, 0x00000009 }, { 0x0000003a, 0x00000053 } } },	\
	{ 0x9a94, {								\
		{ 0x0000007a, 0x00000049 }, { 0x0000007a, 0x00000093 } } },	\
	{ 0x9a98, {								\
		{ 0x00000186, 0x00000089 }, { 0x000000ba, 0x00000194 } } },	\
	{ 0x9a9c, {								\
		{ 0x000001c6, 0x00000179 }, { 0x000001bb, 0x000001d4 } } },	\
	{ 0x9aa0, {								\
		{ 0x00000006, 0x000001b9 }, { 0x000001fb, 0x00000014 } } },	\
	{ 0x9aa4, {								\
		{ 0x00000046, 0x000001f9 }, { 0x0000003b, 0x0000003a } } },	\
	{ 0x9aa8, {								\
		{ 0x00000086, 0x00000039 }, { 0x0000007b, 0x0000007a } } },	\
	{ 0x9aac, {								\
		{ 0x000000c6, 0x00000079 }, { 0x000000bb, 0x000000ba } } },	\
	{ 0x9ab0, {								\
		{ 0x000000c6, 0x000000b9 }, { 0x000001bc, 0x000001bb } } },	\
	{ 0x9ab4, {								\
		{ 0x000000c6, 0x000001bd }, { 0x000001fc, 0x000001fb } } },	\
	{ 0x9ab8, {								\
		{ 0x000000c6, 0x000001fd }, { 0x0000003c, 0x0000003b } } },	\
	{ 0x9abc, {								\
		{ 0x000000c6, 0x0000003d }, { 0x0000007c, 0x0000007b } } },	\
	{ 0x9ac0, {								\
		{ 0x000000c6, 0x0000007d }, { 0x000000bc, 0x000000bb } } },	\
	{ 0x9ac4, {								\
		{ 0x000000c6, 0x000000bd }, { 0x000000fc, 0x000001bc } } },	\
	{ 0x9ac8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000001fc } } },	\
	{ 0x9acc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x0000003c } } },	\
	{ 0x9ad0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x0000007c } } },	\
	{ 0x9ad4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000bc } } },	\
	{ 0x9ad8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9adc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9aec, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9afc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
}

/*
 * Chipset capabilities -see ath_hal_getcapability-
 * get_capability function is not yet fully implemented
 * in OpenHAL so most of these don't work yet...
 */
typedef enum {
	AR5K_CAP_REG_DMN		= 0,	/* Used to get current reg. domain id */
	AR5K_CAP_CIPHER			= 1,	/* Can handle encryption */
	AR5K_CAP_TKIP_MIC		= 2,	/* Can handle TKIP MIC in hardware */
	AR5K_CAP_TKIP_SPLIT		= 3,	/* TKIP uses split keys */
	AR5K_CAP_PHYCOUNTERS		= 4,	/* PHY error counters */
	AR5K_CAP_DIVERSITY		= 5,	/* Supports fast diversity */
	AR5K_CAP_NUM_TXQUEUES		= 6,	/* Used to get max number of hw txqueues */
	AR5K_CAP_VEOL			= 7,	/* Supports virtual EOL */
	AR5K_CAP_COMPRESSION		= 8,	/* Supports compression */
	AR5K_CAP_BURST			= 9,	/* Supports packet bursting */
	AR5K_CAP_FASTFRAME		= 10,	/* Supports fast frames */
	AR5K_CAP_TXPOW			= 11,	/* Used to get global tx power limit */
	AR5K_CAP_TPC			= 12,	/* Can do per-packet tx power control (needed for 802.11a) */
	AR5K_CAP_BSSIDMASK		= 13,	/* Supports bssid mask */
	AR5K_CAP_MCAST_KEYSRCH		= 14,	/* Supports multicast key search */
	AR5K_CAP_TSF_ADJUST		= 15,	/* Supports beacon tsf adjust */
	AR5K_CAP_XR			= 16,	/* Supports XR mode */
	AR5K_CAP_WME_TKIPMIC 		= 17,	/* Supports TKIP MIC when using WMM */
	AR5K_CAP_CHAN_HALFRATE 		= 18,	/* Supports half rate channels */
	AR5K_CAP_CHAN_QUARTERRATE 	= 19,	/* Supports quarter rate channels */
	AR5K_CAP_RFSILENT		= 20,	/* Supports RFsilent */
} AR5K_CAPABILITY_TYPE;

typedef struct {
	/*
	 * Supported PHY modes
	 * (ie. CHANNEL_A, CHANNEL_B, ...)
	 */
	u_int16_t	cap_mode;

	/*
	 * Frequency range (without regulation restrictions)
	 */
	struct {
		u_int16_t	range_2ghz_min;
		u_int16_t	range_2ghz_max;
		u_int16_t	range_5ghz_min;
		u_int16_t	range_5ghz_max;
	} cap_range;

	/*
	 * Active regulation domain settings
	 */
	struct {
		ieee80211_regdomain_t	reg_current;
		ieee80211_regdomain_t	reg_hw;
	} cap_regdomain;

	/*
	 * Values stored in the EEPROM (some of them...)
	 */
	struct ar5k_eeprom_info	cap_eeprom;

	/*
	 * Queue information
	 */
	struct {
		u_int8_t	q_tx_num;
	} cap_queues;
} ar5k_capabilities_t;




/***************************************\
  HARDWARE ABSTRACTION LAYER STRUCTURE
\***************************************/

/*
 * Regulation stuff
 */
typedef enum ieee80211_countrycode AR5K_CTRY_CODE;

/* Default regulation domain if stored value EEPROM value is invalid */
#define AR5K_TUNE_REGDOMAIN	DMN_FCC2_FCCA	/* Canada */
#define AR5K_TUNE_CTRY		CTRY_DEFAULT

/*
 * Misc defines
 */
#define AR5K_ABI_VERSION		0x04090901 /* YYMMDDnn */

#define AR5K_ELEMENTS(_array)	(sizeof(_array) / sizeof(_array[0]))

typedef struct ath_hal * (ar5k_attach_t)
	(u_int16_t, AR5K_SOFTC, AR5K_BUS_TAG, AR5K_BUS_HANDLE, AR5K_STATUS *);

typedef AR5K_BOOL (ar5k_rfgain_t)
	(struct ath_hal *, AR5K_CHANNEL *, u_int);

/*
 * HAL Functions that have different implementations for each chipset...
 */
#define AR5K_HAL_FUNCTION(_hal, _n, _f)	(_hal)->ah_##_f = ar5k_##_n##_##_f
#define AR5K_HAL_FUNCTIONS(_t, _n, _a) \
	_t const AR5K_RATE_TABLE *(_a _n##_get_rate_table)(struct ath_hal *, u_int mode);	\
	_t void (_a _n##_detach)(struct ath_hal *);						\
	/* Reset functions */									\
	_t AR5K_BOOL (_a _n##_reset)(struct ath_hal *, AR5K_OPMODE, AR5K_CHANNEL *,		\
				AR5K_BOOL change_channel, AR5K_STATUS *status);			\
	_t void (_a _n##_set_opmode)(struct ath_hal *);					\
	_t AR5K_BOOL (_a _n##_calibrate)(struct ath_hal*, AR5K_CHANNEL *);			\
	/* Transmit functions */								\
	_t AR5K_BOOL (_a _n##_update_tx_triglevel)(struct ath_hal*, AR5K_BOOL level);		\
	_t int (_a _n##_setup_tx_queue)(struct ath_hal *, AR5K_TX_QUEUE, AR5K_TXQ_INFO *);	\
	_t AR5K_BOOL (_a _n##_setup_tx_queueprops)(struct ath_hal *, int queue,			\
				const AR5K_TXQ_INFO *);						\
	_t AR5K_BOOL (_a _n##_release_tx_queue)(struct ath_hal *, u_int queue);			\
	_t AR5K_BOOL (_a _n##_reset_tx_queue)(struct ath_hal *, u_int queue);			\
	_t u_int32_t (_a _n##_get_tx_buf)(struct ath_hal *, u_int queue);			\
	_t AR5K_BOOL (_a _n##_put_tx_buf)(struct ath_hal *, u_int, u_int32_t phys_addr);	\
	_t AR5K_BOOL (_a _n##_tx_start)(struct ath_hal *, u_int queue);				\
	_t AR5K_BOOL (_a _n##_stop_tx_dma)(struct ath_hal *, u_int queue);			\
	_t AR5K_BOOL (_a _n##_setup_tx_desc)(struct ath_hal *, struct ath_desc *,		\
				u_int packet_length, u_int header_length, AR5K_PKT_TYPE type,	\
				u_int txPower, u_int tx_rate0, u_int tx_tries0, u_int key_index,\
				u_int antenna_mode, u_int flags, u_int rtscts_rate,		\
				u_int rtscts_duration);						\
	_t AR5K_BOOL (_a _n##_setup_xtx_desc)(struct ath_hal *, struct ath_desc *,		\
				u_int tx_rate1, u_int tx_tries1, u_int tx_rate2,		\
				u_int tx_tries2,u_int tx_rate3, u_int tx_tries3);		\
	_t AR5K_BOOL (_a _n##_fill_tx_desc)(struct ath_hal *, struct ath_desc *, u_int segLen,	\
				AR5K_BOOL firstSeg, AR5K_BOOL lastSeg, const struct ath_desc *);\
	_t AR5K_STATUS (_a _n##_proc_tx_desc)(struct ath_hal *, struct ath_desc *);		\
	_t AR5K_BOOL (_a _n##_has_veol)(struct ath_hal *);					\
	/* Receive Functions */									\
	_t u_int32_t (_a _n##_get_rx_buf)(struct ath_hal*);					\
	_t void (_a _n##_put_rx_buf)(struct ath_hal*, u_int32_t rxdp);				\
	_t void (_a _n##_start_rx)(struct ath_hal*);						\
	_t AR5K_BOOL (_a _n##_stop_rx_dma)(struct ath_hal*);					\
	_t void (_a _n##_start_rx_pcu)(struct ath_hal*);					\
	_t void (_a _n##_stop_pcu_recv)(struct ath_hal*);					\
	_t void (_a _n##_set_mcast_filter)(struct ath_hal*, u_int32_t filter0,			\
				u_int32_t filter1);						\
	_t AR5K_BOOL (_a _n##_set_mcast_filterindex)(struct ath_hal*, u_int32_t index);	\
	_t AR5K_BOOL (_a _n##_clear_mcast_filter_idx)(struct ath_hal*,u_int32_t index);	\
	_t u_int32_t (_a _n##_get_rx_filter)(struct ath_hal*);					\
	_t void (_a _n##_set_rx_filter)(struct ath_hal*, u_int32_t);				\
	_t AR5K_BOOL (_a _n##_setup_rx_desc)(struct ath_hal *, struct ath_desc *,		\
				u_int32_t size,	u_int flags);					\
	_t AR5K_STATUS (_a _n##_proc_rx_desc)(struct ath_hal *, struct ath_desc *,		\
				u_int32_t phyAddr, struct ath_desc *next);			\
	_t void (_a _n##_set_rx_signal)(struct ath_hal *, const AR5K_NODE_STATS *);		\
	/* Misc Functions */									\
	_t void (_a _n##_dump_state)(struct ath_hal *);					\
	_t AR5K_BOOL (_a _n##_get_diag_state)(struct ath_hal *, int request,const void *args,	\
				u_int32_t argsize, void **result, u_int32_t *resultsize);	\
	_t void (_a _n##_get_lladdr)(struct ath_hal *, u_int8_t *);				\
	_t AR5K_BOOL (_a _n##_set_lladdr)(struct ath_hal *, const u_int8_t*);			\
	_t AR5K_BOOL (_a _n##_set_regdomain)(struct ath_hal*, u_int16_t, AR5K_STATUS *);	\
	_t void (_a _n##_set_ledstate)(struct ath_hal*, AR5K_LED_STATE);			\
	_t void (_a _n##_set_associd)(struct ath_hal*, const u_int8_t *bssid,			\
				u_int16_t assocId);						\
	_t AR5K_BOOL (_a _n##_set_gpio_input)(struct ath_hal *, u_int32_t gpio);		\
	_t AR5K_BOOL (_a _n##_set_gpio_output)(struct ath_hal *, u_int32_t gpio);		\
	_t u_int32_t (_a _n##_get_gpio)(struct ath_hal *, u_int32_t gpio);			\
	_t AR5K_BOOL (_a _n##_set_gpio)(struct ath_hal *, u_int32_t gpio, u_int32_t val);	\
	_t void (_a _n##_set_gpio_intr)(struct ath_hal*, u_int, u_int32_t);			\
	_t u_int32_t (_a _n##_get_tsf32)(struct ath_hal*);					\
	_t u_int64_t (_a _n##_get_tsf64)(struct ath_hal*);					\
	_t void (_a _n##_reset_tsf)(struct ath_hal*);						\
	_t u_int16_t (_a _n##_get_regdomain)(struct ath_hal*);					\
	_t AR5K_BOOL (_a _n##_detect_card_present)(struct ath_hal*);				\
	_t void (_a _n##_update_mib_counters)(struct ath_hal*, AR5K_MIB_STATS*);		\
	_t AR5K_RFGAIN (_a _n##_get_rf_gain)(struct ath_hal*);					\
	_t AR5K_BOOL (_a _n##_set_slot_time)(struct ath_hal*, u_int);				\
	_t u_int (_a _n##_get_slot_time)(struct ath_hal*);					\
	_t AR5K_BOOL (_a _n##_set_ack_timeout)(struct ath_hal *, u_int);			\
	_t u_int (_a _n##_get_ack_timeout)(struct ath_hal*);					\
	_t AR5K_BOOL (_a _n##_set_cts_timeout)(struct ath_hal*, u_int);			\
	_t u_int (_a _n##_get_cts_timeout)(struct ath_hal*);					\
	/* Key Cache Functions */								\
	_t AR5K_BOOL (_a _n##_is_cipher_supported)(struct ath_hal*, AR5K_CIPHER);		\
	_t u_int32_t (_a _n##_get_keycache_size)(struct ath_hal*);				\
	_t AR5K_BOOL (_a _n##_reset_key)(struct ath_hal*, u_int16_t);				\
	_t AR5K_BOOL (_a _n##_is_key_valid)(struct ath_hal *, u_int16_t);			\
	_t AR5K_BOOL (_a _n##_set_key)(struct ath_hal*, u_int16_t, const AR5K_KEYVAL *,		\
				const u_int8_t *, int);						\
	_t AR5K_BOOL (_a _n##_set_key_lladdr)(struct ath_hal*, u_int16_t, const u_int8_t *);	\
	/* Power Management Functions */							\
	_t AR5K_BOOL (_a _n##_set_power)(struct ath_hal*, AR5K_POWER_MODE mode,		\
				AR5K_BOOL set_chip, u_int16_t sleep_duration);			\
	_t AR5K_POWER_MODE (_a _n##_get_power_mode)(struct ath_hal*);				\
	_t AR5K_BOOL (_a _n##_query_pspoll_support)(struct ath_hal*);				\
	_t AR5K_BOOL (_a _n##_init_pspoll)(struct ath_hal*);					\
	_t AR5K_BOOL (_a _n##_enable_pspoll)(struct ath_hal *, u_int8_t *, u_int16_t);		\
	_t AR5K_BOOL (_a _n##_disable_pspoll)(struct ath_hal *);				\
	/* Beacon Management Functions */							\
	_t void (_a _n##_init_beacon)(struct ath_hal *, u_int32_t nexttbtt, u_int32_t intval);	\
	_t void (_a _n##_set_beacon_timers)(struct ath_hal *, const AR5K_BEACON_STATE *);	\
	_t void (_a _n##_reset_beacon)(struct ath_hal *);					\
	_t AR5K_BOOL (_a _n##_wait_for_beacon)(struct ath_hal *, AR5K_BUS_ADDR);		\
	/* Interrupt functions */								\
	_t AR5K_BOOL (_a _n##_is_intr_pending)(struct ath_hal *);				\
	_t AR5K_BOOL (_a _n##_get_isr)(struct ath_hal *, u_int32_t *);				\
	_t u_int32_t (_a _n##_get_intr)(struct ath_hal *);					\
	_t AR5K_INT (_a _n##_set_intr)(struct ath_hal *, AR5K_INT);				\
	/* Chipset functions (ar5k-specific, non-HAL) */					\
	_t AR5K_BOOL (_a _n##_get_capabilities)(struct ath_hal *);				\
	_t void (_a _n##_radar_alert)(struct ath_hal *, AR5K_BOOL enable);			\
	_t AR5K_BOOL (_a _n##_eeprom_is_busy)(struct ath_hal *);				\
	_t int (_a _n##_eeprom_read)(struct ath_hal *, u_int32_t offset, u_int16_t *data);	\
	_t int (_a _n##_eeprom_write)(struct ath_hal *, u_int32_t offset, u_int16_t data);	\
	/* Functions not found in OpenBSD */							\
	_t AR5K_BOOL (_a _n##_get_tx_queueprops)(struct ath_hal *, int, AR5K_TXQ_INFO *);	\
	_t AR5K_STATUS (_a _n##_get_capability)(struct ath_hal *, AR5K_CAPABILITY_TYPE,		\
				u_int32_t, u_int32_t *);					\
	_t u_int32_t  (_a _n##_num_tx_pending)(struct ath_hal *, u_int);			\
	_t AR5K_BOOL (_a _n##_phy_disable)(struct ath_hal *);					\
	_t void (_a _n##_set_pcu_config)(struct ath_hal *);					\
	_t AR5K_BOOL (_a _n##_set_txpower_limit)(struct ath_hal *, u_int);			\
	_t void (_a _n##_set_def_antenna)(struct ath_hal *, u_int);				\
	_t u_int  (_a _n ##_get_def_antenna)(struct ath_hal *);					\
	_t AR5K_BOOL (_a _n ##_set_bssid_mask)(struct ath_hal *, const u_int8_t*);		\
	/*Totaly unimplemented*/								\
	_t AR5K_BOOL (_a _n##_set_capability)(struct ath_hal *, AR5K_CAPABILITY_TYPE, u_int32_t,\
				u_int32_t,AR5K_STATUS *) ;					\
	_t void (_a _n##_proc_mib_event)(struct ath_hal *, const AR5K_NODE_STATS *) ;		\
	_t void (_a _n##_get_tx_inter_queue)(struct ath_hal *, u_int32_t *);


#define AR5K_MAX_GPIO		10
#define AR5K_MAX_RF_BANKS	8

struct ath_hal {
	u_int32_t		ah_magic;
	u_int16_t		ah_device;
	u_int16_t		ah_sub_vendor;

	AR5K_SOFTC		ah_sc;
	bus_space_tag_t		ah_st;
	bus_space_handle_t 	ah_sh;
	AR5K_CTRY_CODE		ah_country_code;

	AR5K_INT			ah_imr;

	AR5K_OPMODE		ah_op_mode;
	AR5K_POWER_MODE		ah_power_mode;
	AR5K_CHANNEL		ah_current_channel;
	AR5K_BOOL		ah_turbo;
	AR5K_BOOL		ah_calibration;
	AR5K_BOOL		ah_running;
	AR5K_BOOL		ah_single_chip;
	AR5K_RFGAIN		ah_rf_gain;

	AR5K_RATE_TABLE		ah_rt_11a;
	AR5K_RATE_TABLE		ah_rt_11b;
	AR5K_RATE_TABLE		ah_rt_11g;
	AR5K_RATE_TABLE		ah_rt_turbo;
	AR5K_RATE_TABLE		ah_rt_xr;

	u_int32_t		ah_mac_srev;
	u_int16_t		ah_mac_version;
	u_int16_t		ah_mac_revision;
	u_int16_t		ah_phy_revision;
	u_int16_t		ah_radio_5ghz_revision;
	u_int16_t		ah_radio_2ghz_revision;

	enum ar5k_version	ah_version;
	enum ar5k_radio		ah_radio;
	u_int32_t		ah_phy;

	AR5K_BOOL		ah_5ghz;
	AR5K_BOOL		ah_2ghz;

#define ah_regdomain		ah_capabilities.cap_regdomain.reg_current
#define ah_regdomain_hw	ah_capabilities.cap_regdomain.reg_hw
#define ah_modes		ah_capabilities.cap_mode
#define ah_ee_version		ah_capabilities.cap_eeprom.ee_version

	u_int32_t		ah_atim_window;
	u_int32_t		ah_aifs;
	u_int32_t		ah_cw_min;
	u_int32_t		ah_cw_max;
	AR5K_BOOL		ah_software_retry;
	u_int32_t		ah_limit_tx_retries;

	u_int32_t		ah_antenna[AR5K_EEPROM_N_MODES][AR5K_ANT_MAX];
	AR5K_BOOL		ah_ant_diversity;

	u_int8_t		ah_sta_id[IEEE80211_ADDR_LEN];
	u_int8_t		ah_bssid[IEEE80211_ADDR_LEN];

	u_int32_t		ah_gpio[AR5K_MAX_GPIO];
	int			ah_gpio_npins;

	ar5k_capabilities_t	ah_capabilities;

	AR5K_TXQ_INFO		ah_txq[AR5K_NUM_TX_QUEUES];
	u_int32_t		ah_txq_interrupts;

	u_int32_t		*ah_rf_banks;
	size_t			ah_rf_banks_size;
	struct ar5k_gain	ah_gain;
	u_int32_t		ah_offset[AR5K_MAX_RF_BANKS];

	struct {
		u_int16_t	txp_pcdac[AR5K_EEPROM_POWER_TABLE_SIZE];
		u_int16_t	txp_rates[AR5K_MAX_RATES];
		int16_t		txp_min, txp_max;
		AR5K_BOOL	txp_tpc;
		int16_t		txp_ofdm;
	} ah_txpower;

	struct {
		AR5K_BOOL	r_enabled;
		int		r_last_alert;
		AR5K_CHANNEL	r_last_channel;
	} ah_radar;

	/*
	 * Function pointers
	 */
	AR5K_HAL_FUNCTIONS(, ah, *);

};

/*
 * Prototypes -functions common for all chipsets-
 */


const char		*ath_hal_probe(u_int16_t, u_int16_t);
struct ath_hal		*ath_hal_attach(u_int16_t device, AR5K_SOFTC sc, AR5K_BUS_TAG,
					AR5K_BUS_HANDLE, AR5K_STATUS *);
u_int16_t		 ath_hal_computetxtime(struct ath_hal *, const AR5K_RATE_TABLE *,
					u_int32_t, u_int16_t, AR5K_BOOL);
u_int			 ath_hal_mhz2ieee(u_int, u_int);
u_int			 ath_hal_ieee2mhz(u_int, u_int);
AR5K_BOOL		 ath_hal_init_channels(struct ath_hal *, AR5K_CHANNEL *,
					u_int, u_int *, AR5K_CTRY_CODE, u_int16_t, 
					AR5K_BOOL, AR5K_BOOL);
const char		*ar5k_printver(enum ar5k_srev_type, u_int32_t);
void			 ar5k_radar_alert(struct ath_hal *);
ieee80211_regdomain_t	 ar5k_regdomain_to_ieee(u_int16_t);
u_int16_t		 ar5k_regdomain_from_ieee(ieee80211_regdomain_t);
u_int16_t		 ar5k_get_regdomain(struct ath_hal *);
u_int32_t		 ar5k_bitswap(u_int32_t, u_int);
u_int			 ar5k_clocktoh(u_int, AR5K_BOOL);
u_int			 ar5k_htoclock(u_int, AR5K_BOOL);
void			 ar5k_rt_copy(AR5K_RATE_TABLE *, const AR5K_RATE_TABLE *);
AR5K_BOOL		 ar5k_register_timeout(struct ath_hal *, u_int32_t, u_int32_t,
					u_int32_t, AR5K_BOOL);
int			 ar5k_eeprom_init(struct ath_hal *);
int			 ar5k_eeprom_read_mac(struct ath_hal *, u_int8_t *);
AR5K_BOOL		 ar5k_eeprom_regulation_domain(struct ath_hal *, AR5K_BOOL,
					ieee80211_regdomain_t *);
AR5K_BOOL		 ar5k_channel(struct ath_hal *, AR5K_CHANNEL *);
AR5K_BOOL		 ar5k_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int);
u_int32_t		 ar5k_rfregs_gainf_corr(struct ath_hal *);
AR5K_BOOL		 ar5k_rfregs_gain_readback(struct ath_hal *);
int32_t			 ar5k_rfregs_gain_adjust(struct ath_hal *);
AR5K_BOOL		 ar5k_rfgain(struct ath_hal *, u_int, u_int);
void			 ar5k_txpower_table(struct ath_hal *, AR5K_CHANNEL *, int16_t);

/*added*/
extern	u_int  ath_hal_getwirelessmodes(struct ath_hal*, AR5K_CTRY_CODE);
void ath_hal_detach(struct ath_hal *ah);
struct ath_hal * _ath_hal_attach(u_int16_t devid, AR5K_SOFTC sc, AR5K_BUS_TAG t,
					AR5K_BUS_HANDLE h, void* s);
#endif /* _AR5K_H */
