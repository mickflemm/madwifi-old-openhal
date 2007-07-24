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

struct ath5k_gain_opt_step {
	int16_t				gos_param[AR5K_GAIN_CRN_MAX_FIX_BITS];
	int32_t				gos_gain;
};

struct ath5k_gain_opt {
	u_int32_t			go_default;
	u_int32_t			go_steps_count;
	const struct ath5k_gain_opt_step	go_step[AR5K_GAIN_STEP_COUNT];
};

struct ath5k_gain {
	u_int32_t			g_step_idx;
	u_int32_t			g_current;
	u_int32_t			g_target;
	u_int32_t			g_low;
	u_int32_t			g_high;
	u_int32_t			g_f_corr;
	u_int32_t			g_active;
	const struct ath5k_gain_opt_step	*g_step;
};

/*
 * Gain optimization tables...
 */
#define AR5K_RF5111_GAIN_OPT	{		\
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

#define AR5K_RF5112_GAIN_OPT	{			\
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
 * HW SPECIFIC STRUCTS
 */

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

/* Struct to hold EEPROM calibration data */
struct ath5k_eeprom_info {
	u_int16_t	ee_magic;		/* Magic Number */
	u_int16_t	ee_protect;		/* Protection bits (ath5kreg.h) */
	u_int16_t	ee_regdomain;		/* Regulatory Domain */
	u_int16_t	ee_version;		/* EEPROM Revision */
	u_int16_t	ee_header;		/* EEPROM Header (ath5kreg.h,get_capabilities) */
	u_int16_t	ee_ant_gain;		/* Antenna Gain (ath5kreg.h) */
	u_int16_t	ee_misc0;
	u_int16_t	ee_misc1;
	u_int16_t	ee_cck_ofdm_gain_delta;	/* CCK to OFDM gain delta */
	u_int16_t	ee_cck_ofdm_power_delta;/* CCK to OFDM power delta */
	u_int16_t	ee_scaled_cck_delta;

	/* Used for tx thermal adjustment (eeprom_init, rfregs) */
	u_int16_t	ee_tx_clip;
	u_int16_t	ee_pwd_84;
	u_int16_t	ee_pwd_90;
	u_int16_t	ee_gain_select;

	/* RF Calibration settings (reset, rfregs) */
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
	u_int16_t	ee_false_detect[AR5K_EEPROM_N_MODES];	/* Unused */
	u_int16_t	ee_cal_pier[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_2GHZ_CHAN]; /* Unused */
	u_int16_t	ee_channel[AR5K_EEPROM_N_MODES][AR5K_EEPROM_MAX_CHAN];	/* Empty ! */

	/* Conformance test limits (Unused) */
	u_int16_t	ee_ctls;
	u_int16_t	ee_ctl[AR5K_EEPROM_MAX_CTLS];

	/* Noise Floor Calibration settings */
	int16_t		ee_noise_floor_thr[AR5K_EEPROM_N_MODES];
	int8_t		ee_adc_desired_size[AR5K_EEPROM_N_MODES];
	int8_t		ee_pga_desired_size[AR5K_EEPROM_N_MODES];
};

/*
 * Internal HW RX/TX descriptor structures
 * (rX: reserved fields possibily used by future versions of the ar5k chipset)
 */

/*
 * Common rx control descriptor
 */
struct ath5k_rx_desc {

	/* RX control word 0 */
	u_int32_t	rx_control_0;

#define AR5K_DESC_RX_CTL0			0x00000000

	/* RX control word 1 */
	u_int32_t	rx_control_1;

#define AR5K_DESC_RX_CTL1_BUF_LEN		0x00000fff
#define AR5K_DESC_RX_CTL1_INTREQ		0x00002000

} __packed;

/*
 * 5210/5211 rx status descriptor
 */
struct ath5k_hw_old_rx_status {

	/*`RX status word 0`*/
	u_int32_t	rx_status_0;

#define AR5K_OLD_RX_DESC_STATUS0_DATA_LEN		0x00000fff
#define AR5K_OLD_RX_DESC_STATUS0_MORE			0x00001000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE		0x00078000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE_S		15
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL		0x07f80000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL_S	19
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA	0x38000000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA_S	27

	/* RX status word 1 */
	u_int32_t	rx_status_1;

#define AR5K_OLD_RX_DESC_STATUS1_DONE			0x00000001
#define AR5K_OLD_RX_DESC_STATUS1_FRAME_RECEIVE_OK	0x00000002
#define AR5K_OLD_RX_DESC_STATUS1_CRC_ERROR		0x00000004
#define AR5K_OLD_RX_DESC_STATUS1_FIFO_OVERRUN		0x00000008
#define AR5K_OLD_RX_DESC_STATUS1_DECRYPT_CRC_ERROR	0x00000010
#define AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR		0x000000e0
#define AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR_S		5
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_VALID	0x00000100
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX		0x00007e00
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_S		9
#define AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP	0x0fff8000
#define AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP_S	15
#define AR5K_OLD_RX_DESC_STATUS1_KEY_CACHE_MISS		0x10000000

} __packed;

/*
 * 5212 rx status descriptor
 */
struct ath5k_hw_new_rx_status {

	/* RX status word 0 */
	u_int32_t	rx_status_0;

#define AR5K_NEW_RX_DESC_STATUS0_DATA_LEN		0x00000fff
#define AR5K_NEW_RX_DESC_STATUS0_MORE			0x00001000
#define AR5K_NEW_RX_DESC_STATUS0_DECOMP_CRC_ERROR	0x00002000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE		0x000f8000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE_S		15
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL		0x0ff00000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL_S	20
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA	0xf0000000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA_S	28

	/* RX status word 1 */
	u_int32_t	rx_status_1;

#define AR5K_NEW_RX_DESC_STATUS1_DONE			0x00000001
#define AR5K_NEW_RX_DESC_STATUS1_FRAME_RECEIVE_OK	0x00000002
#define AR5K_NEW_RX_DESC_STATUS1_CRC_ERROR		0x00000004
#define AR5K_NEW_RX_DESC_STATUS1_DECRYPT_CRC_ERROR	0x00000008
#define AR5K_NEW_RX_DESC_STATUS1_PHY_ERROR		0x00000010
#define AR5K_NEW_RX_DESC_STATUS1_MIC_ERROR		0x00000020
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_VALID	0x00000100
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX		0x0000fe00
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_S		9
#define AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP	0x7fff0000
#define AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP_S	16
#define AR5K_NEW_RX_DESC_STATUS1_KEY_CACHE_MISS		0x80000000
} __packed;

/*
 * 5212 rx error descriptor
 */
struct ath5k_hw_rx_error {

	/* RX error word 0 */
	u_int32_t	rx_error_0;

#define AR5K_RX_DESC_ERROR0			0x00000000

	/* RX error word 1 */
	u_int32_t	rx_error_1;

#define AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE	0x0000ff00
#define AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE_S	8

} __packed;

#define AR5K_DESC_RX_PHY_ERROR_NONE		0x00
#define AR5K_DESC_RX_PHY_ERROR_TIMING		0x20
#define AR5K_DESC_RX_PHY_ERROR_PARITY		0x40
#define AR5K_DESC_RX_PHY_ERROR_RATE		0x60
#define AR5K_DESC_RX_PHY_ERROR_LENGTH		0x80
#define AR5K_DESC_RX_PHY_ERROR_64QAM		0xa0
#define AR5K_DESC_RX_PHY_ERROR_SERVICE		0xc0
#define AR5K_DESC_RX_PHY_ERROR_TRANSMITOVR	0xe0

/*
 * 5210/5211 2-word tx control descriptor
 */
struct ath5k_hw_2w_tx_desc {

	/* TX control word 0 */
	u_int32_t	tx_control_0;

#define AR5K_2W_TX_DESC_CTL0_FRAME_LEN		0x00000fff
#define AR5K_2W_TX_DESC_CTL0_HEADER_LEN		0x0003f000 /*[5210 ?]*/
#define AR5K_2W_TX_DESC_CTL0_HEADER_LEN_S	12
#define AR5K_2W_TX_DESC_CTL0_XMIT_RATE		0x003c0000
#define AR5K_2W_TX_DESC_CTL0_XMIT_RATE_S	18
#define AR5K_2W_TX_DESC_CTL0_RTSENA		0x00400000
#define AR5K_2W_TX_DESC_CTL0_CLRDMASK		0x01000000
#define AR5K_2W_TX_DESC_CTL0_LONG_PACKET	0x00800000 /*[5210]*/
#define AR5K_2W_TX_DESC_CTL0_VEOL		0x00800000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL0_FRAME_TYPE		0x1c000000 /*[5210]*/
#define AR5K_2W_TX_DESC_CTL0_FRAME_TYPE_S	26
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5210	0x02000000
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5211	0x1e000000
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT	(hal->ah_version == AR5K_AR5210 ? \
						AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5210 : \
						AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5211)
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_S	25
#define AR5K_2W_TX_DESC_CTL0_INTREQ		0x20000000
#define AR5K_2W_TX_DESC_CTL0_ENCRYPT_KEY_VALID	0x40000000

	/* TX control word 1 */
	u_int32_t	tx_control_1;

#define AR5K_2W_TX_DESC_CTL1_BUF_LEN		0x00000fff
#define AR5K_2W_TX_DESC_CTL1_MORE		0x00001000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5210	0x0007e000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5211	0x000fe000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX	(hal->ah_version == AR5K_AR5210 ? \
						AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5210 : \
						AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5211)
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_S	13
#define AR5K_2W_TX_DESC_CTL1_FRAME_TYPE		0x00700000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL1_FRAME_TYPE_S	20
#define AR5K_2W_TX_DESC_CTL1_NOACK		0x00800000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL1_RTS_DURATION	0xfff80000 /*[5210 ?]*/

} __packed;

#define AR5K_AR5210_TX_DESC_FRAME_TYPE_NORMAL   0x00
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_ATIM     0x04
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_PSPOLL   0x08
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_NO_DELAY 0x0c
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_PIFS     0x10

/*
 * 5212 4-word tx control descriptor
 */
struct ath5k_hw_4w_tx_desc {

	/* TX control word 0 */
	u_int32_t	tx_control_0;

#define AR5K_4W_TX_DESC_CTL0_FRAME_LEN		0x00000fff
#define AR5K_4W_TX_DESC_CTL0_XMIT_POWER		0x003f0000
#define AR5K_4W_TX_DESC_CTL0_XMIT_POWER_S	16
#define AR5K_4W_TX_DESC_CTL0_RTSENA		0x00400000
#define AR5K_4W_TX_DESC_CTL0_VEOL		0x00800000
#define AR5K_4W_TX_DESC_CTL0_CLRDMASK		0x01000000
#define AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT	0x1e000000
#define AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT_S	25
#define AR5K_4W_TX_DESC_CTL0_INTREQ		0x20000000
#define AR5K_4W_TX_DESC_CTL0_ENCRYPT_KEY_VALID	0x40000000
#define AR5K_4W_TX_DESC_CTL0_CTSENA		0x80000000

	/* TX control word 1 */
	u_int32_t	tx_control_1;

#define AR5K_4W_TX_DESC_CTL1_BUF_LEN		0x00000fff
#define AR5K_4W_TX_DESC_CTL1_MORE		0x00001000
#define AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX	0x000fe000
#define AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_S	13
#define AR5K_4W_TX_DESC_CTL1_FRAME_TYPE		0x00f00000
#define AR5K_4W_TX_DESC_CTL1_FRAME_TYPE_S	20
#define AR5K_4W_TX_DESC_CTL1_NOACK		0x01000000
#define AR5K_4W_TX_DESC_CTL1_COMP_PROC		0x06000000
#define AR5K_4W_TX_DESC_CTL1_COMP_PROC_S	25
#define AR5K_4W_TX_DESC_CTL1_COMP_IV_LEN	0x18000000
#define AR5K_4W_TX_DESC_CTL1_COMP_IV_LEN_S	27
#define AR5K_4W_TX_DESC_CTL1_COMP_ICV_LEN	0x60000000
#define AR5K_4W_TX_DESC_CTL1_COMP_ICV_LEN_S	29

	/* TX control word 2 */
	u_int32_t	tx_control_2;

#define AR5K_4W_TX_DESC_CTL2_RTS_DURATION		0x00007fff
#define AR5K_4W_TX_DESC_CTL2_DURATION_UPDATE_ENABLE	0x00008000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0		0x000f0000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0_S		16
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1		0x00f00000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1_S		20
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2		0x0f000000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2_S		24
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3		0xf0000000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3_S		28

	/* TX control word 3 */
	u_int32_t	tx_control_3;

#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE0		0x0000001f
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE1		0x000003e0
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE1_S	5
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE2		0x00007c00
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE2_S	10
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE3		0x000f8000
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE3_S	15
#define AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE	0x01f00000
#define AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE_S	20

} __packed;


/*
 * Common tx status descriptor
 */
struct ath5k_hw_tx_status {

	/* TX status word 0 */
	u_int32_t	tx_status_0;

#define AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK	0x00000001
#define AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES	0x00000002
#define AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN	0x00000004
#define AR5K_DESC_TX_STATUS0_FILTERED		0x00000008
/*???
#define AR5K_DESC_TX_STATUS0_RTS_FAIL_COUNT	0x000000f0
#define AR5K_DESC_TX_STATUS0_RTS_FAIL_COUNT_S	4
*/
#define AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT	0x000000f0
#define AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT_S	4
/*???
#define AR5K_DESC_TX_STATUS0_DATA_FAIL_COUNT	0x00000f00
#define AR5K_DESC_TX_STATUS0_DATA_FAIL_COUNT_S	8
*/
#define AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT	0x00000f00
#define AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT_S	8
#define AR5K_DESC_TX_STATUS0_VIRT_COLL_COUNT	0x0000f000
#define AR5K_DESC_TX_STATUS0_VIRT_COLL_COUNT_S	12
#define AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP	0xffff0000
#define AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP_S	16

	/* TX status word 1 */
	u_int32_t	tx_status_1;

#define AR5K_DESC_TX_STATUS1_DONE		0x00000001
#define AR5K_DESC_TX_STATUS1_SEQ_NUM		0x00001ffe
#define AR5K_DESC_TX_STATUS1_SEQ_NUM_S		1
#define AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH	0x001fe000
#define AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH_S	13
#define AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX	0x00600000
#define AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX_S	21
#define AR5K_DESC_TX_STATUS1_COMP_SUCCESS	0x00800000
#define AR5K_DESC_TX_STATUS1_XMIT_ANTENNA	0x01000000

} __packed;



/*
 * AR5K REGISTER ACCESS
 */

/*Swap RX/TX Descriptor for big endian archs*/
#if defined(__BIG_ENDIAN)
#define AR5K_INIT_CFG	(		\
	AR5K_CFG_SWTD | AR5K_CFG_SWRD	\
)
#else
#define AR5K_INIT_CFG	0x00000000
#endif

#define AR5K_REG_READ(_reg)	ath5k_hw_reg_read(hal, _reg)

#define AR5K_REG_WRITE(_reg, _val)	ath5k_hw_reg_write(hal, _val, _reg)

#define AR5K_REG_SM(_val, _flags)					\
	(((_val) << _flags##_S) & (_flags))

#define AR5K_REG_MS(_val, _flags)					\
	(((_val) & (_flags)) >> _flags##_S)

/* Some registers can hold multiple values of interest. For this
 * reason when we want to write to these registers we must first
 * retrieve the values which we do not want to clear (lets call this 
 * old_data) and then set the register with this and our new_value: 
 * ( old_data | new_value) */
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
		udelay(1);

#define AR5K_EEPROM_READ(_o, _v)	{				\
	if ((ret = ath5k_hw_eeprom_read(hal, (_o),			\
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
#define AR5K_LE_READ_2	ath5k_hw_read_unaligned_16
#define AR5K_LE_READ_4	ath5k_hw_read_unaligned_32
#define AR5K_LE_WRITE_2	ath5k_hw_write_unaligned_16
#define AR5K_LE_WRITE_4	ath5k_hw_write_unaligned_32

#define AR5K_LOW_ID(_a)(				\
(_a)[0] | (_a)[1] << 8 | (_a)[2] << 16 | (_a)[3] << 24	\
)

#define AR5K_HIGH_ID(_a)	((_a)[4] | (_a)[5] << 8)



/*
 * INITIAL REGISTER VALUES
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
 * Non-common initial register values which have to be loaded into the
 * card at boot time and after each reset.
 */


/*
 * RF REGISTERS
 */

/* Register dumps are done per operation mode */
#define AR5K_INI_VAL_11A		0
#define AR5K_INI_VAL_11A_TURBO		1
#define AR5K_INI_VAL_11B		2
#define AR5K_INI_VAL_11G		3
#define AR5K_INI_VAL_11G_TURBO		4
#define AR5K_INI_VAL_XR			0
#define AR5K_INI_VAL_MAX		5

#define AR5K_RF5111_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS
#define AR5K_RF5112_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS


/* Struct to hold initial RF register values (RF Banks)*/
struct ath5k_ini_rf {
	u_int8_t	rf_bank;	/* check out ath5kreg.h */
	u_int16_t	rf_register;	/* register address */
	u_int32_t	rf_value[5];	/* register value for
					   different modes (see avove) */
};

/* RF5111 mode-specific init registers */
#define AR5K_RF5111_INI_RF	{						\
	{ 0, 0x989c,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
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

/* RF5112 mode-specific init registers */
#define AR5K_RF5112_INI_RF	{						\
	{ 1, 0x98d4,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
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

/* RF5112A mode-specific init registers */ 	 
#define AR5K_RF5112A_INI_RF     {						\
	{ 1, 0x98d4,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
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

/*
 * Mode-specific RF Gain table (64bytes) for RF5111/5112
 * (RF5110 only comes with AR5210 and only supports a/turbo a 
 * mode so initial RF Gain values are included in AR5K_AR5210_INI)
 */
struct ath5k_ini_rfgain {
	u_int16_t	rfg_register;		/* RF Gain register address */
	u_int32_t	rfg_value[2];		/* Register value [freq (see below)] */

#define AR5K_INI_RFGAIN_5GHZ	0
#define AR5K_INI_RFGAIN_2GHZ	1
};

/* Initial RF Gain settings for RF5111 */
#define AR5K_RF5111_INI_RFGAIN	{				\
	/*			      5Ghz	2Ghz	*/	\
	{ AR5K_RF_GAIN(0),	{ 0x000001a9, 0x00000000 } },	\
	{ AR5K_RF_GAIN(1),	{ 0x000001e9, 0x00000040 } },	\
	{ AR5K_RF_GAIN(2),	{ 0x00000029, 0x00000080 } },	\
	{ AR5K_RF_GAIN(3),	{ 0x00000069, 0x00000150 } },	\
	{ AR5K_RF_GAIN(4),	{ 0x00000199, 0x00000190 } },	\
	{ AR5K_RF_GAIN(5),	{ 0x000001d9, 0x000001d0 } },	\
	{ AR5K_RF_GAIN(6),	{ 0x00000019, 0x00000010 } },	\
	{ AR5K_RF_GAIN(7),	{ 0x00000059, 0x00000044 } },	\
	{ AR5K_RF_GAIN(8),	{ 0x00000099, 0x00000084 } },	\
	{ AR5K_RF_GAIN(9),	{ 0x000001a5, 0x00000148 } },	\
	{ AR5K_RF_GAIN(10),	{ 0x000001e5, 0x00000188 } },	\
	{ AR5K_RF_GAIN(11),	{ 0x00000025, 0x000001c8 } },	\
	{ AR5K_RF_GAIN(12),	{ 0x000001c8, 0x00000014 } },	\
	{ AR5K_RF_GAIN(13),	{ 0x00000008, 0x00000042 } },	\
	{ AR5K_RF_GAIN(14),	{ 0x00000048, 0x00000082 } },	\
	{ AR5K_RF_GAIN(15),	{ 0x00000088, 0x00000178 } },	\
	{ AR5K_RF_GAIN(16),	{ 0x00000198, 0x000001b8 } },	\
	{ AR5K_RF_GAIN(17),	{ 0x000001d8, 0x000001f8 } },	\
	{ AR5K_RF_GAIN(18),	{ 0x00000018, 0x00000012 } },	\
	{ AR5K_RF_GAIN(19),	{ 0x00000058, 0x00000052 } },	\
	{ AR5K_RF_GAIN(20),	{ 0x00000098, 0x00000092 } },	\
	{ AR5K_RF_GAIN(21),	{ 0x000001a4, 0x0000017c } },	\
	{ AR5K_RF_GAIN(22),	{ 0x000001e4, 0x000001bc } },	\
	{ AR5K_RF_GAIN(23),	{ 0x00000024, 0x000001fc } },	\
	{ AR5K_RF_GAIN(24),	{ 0x00000064, 0x0000000a } },	\
	{ AR5K_RF_GAIN(25),	{ 0x000000a4, 0x0000004a } },	\
	{ AR5K_RF_GAIN(26),	{ 0x000000e4, 0x0000008a } },	\
	{ AR5K_RF_GAIN(27),	{ 0x0000010a, 0x0000015a } },	\
	{ AR5K_RF_GAIN(28),	{ 0x0000014a, 0x0000019a } },	\
	{ AR5K_RF_GAIN(29),	{ 0x0000018a, 0x000001da } },	\
	{ AR5K_RF_GAIN(30),	{ 0x000001ca, 0x0000000e } },	\
	{ AR5K_RF_GAIN(31),	{ 0x0000000a, 0x0000004e } },	\
	{ AR5K_RF_GAIN(32),	{ 0x0000004a, 0x0000008e } },	\
	{ AR5K_RF_GAIN(33),	{ 0x0000008a, 0x0000015e } },	\
	{ AR5K_RF_GAIN(34),	{ 0x000001ba, 0x0000019e } },	\
	{ AR5K_RF_GAIN(35),	{ 0x000001fa, 0x000001de } },	\
	{ AR5K_RF_GAIN(36),	{ 0x0000003a, 0x00000009 } },	\
	{ AR5K_RF_GAIN(37),	{ 0x0000007a, 0x00000049 } },	\
	{ AR5K_RF_GAIN(38),	{ 0x00000186, 0x00000089 } },	\
	{ AR5K_RF_GAIN(39),	{ 0x000001c6, 0x00000179 } },	\
	{ AR5K_RF_GAIN(40),	{ 0x00000006, 0x000001b9 } },	\
	{ AR5K_RF_GAIN(41),	{ 0x00000046, 0x000001f9 } },	\
	{ AR5K_RF_GAIN(42),	{ 0x00000086, 0x00000039 } },	\
	{ AR5K_RF_GAIN(43),	{ 0x000000c6, 0x00000079 } },	\
	{ AR5K_RF_GAIN(44),	{ 0x000000c6, 0x000000b9 } },	\
	{ AR5K_RF_GAIN(45),	{ 0x000000c6, 0x000001bd } },	\
	{ AR5K_RF_GAIN(46),	{ 0x000000c6, 0x000001fd } },	\
	{ AR5K_RF_GAIN(47),	{ 0x000000c6, 0x0000003d } },	\
	{ AR5K_RF_GAIN(48),	{ 0x000000c6, 0x0000007d } },	\
	{ AR5K_RF_GAIN(49),	{ 0x000000c6, 0x000000bd } },	\
	{ AR5K_RF_GAIN(50),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(51),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(52),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(53),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(54),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(55),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(56),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(57),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(58),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(59),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(60),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(61),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(62),	{ 0x000000c6, 0x000000fd } },	\
	{ AR5K_RF_GAIN(63),	{ 0x000000c6, 0x000000fd } },	\
}

/* Initial RF Gain settings for RF5112 */
#define AR5K_RF5112_INI_RFGAIN	{				\
	/*			      5Ghz	2Ghz	*/	\
	{ AR5K_RF_GAIN(0),	{ 0x00000007, 0x00000007 } },	\
	{ AR5K_RF_GAIN(1),	{ 0x00000047, 0x00000047 } },	\
	{ AR5K_RF_GAIN(2),	{ 0x00000087, 0x00000087 } },	\
	{ AR5K_RF_GAIN(3),	{ 0x000001a0, 0x000001a0 } },	\
	{ AR5K_RF_GAIN(4),	{ 0x000001e0, 0x000001e0 } },	\
	{ AR5K_RF_GAIN(5),	{ 0x00000020, 0x00000020 } },	\
	{ AR5K_RF_GAIN(6),	{ 0x00000060, 0x00000060 } },	\
	{ AR5K_RF_GAIN(7),	{ 0x000001a1, 0x000001a1 } },	\
	{ AR5K_RF_GAIN(8),	{ 0x000001e1, 0x000001e1 } },	\
	{ AR5K_RF_GAIN(9),	{ 0x00000021, 0x00000021 } },	\
	{ AR5K_RF_GAIN(10),	{ 0x00000061, 0x00000061 } },	\
	{ AR5K_RF_GAIN(11),	{ 0x00000162, 0x00000162 } },	\
	{ AR5K_RF_GAIN(12),	{ 0x000001a2, 0x000001a2 } },	\
	{ AR5K_RF_GAIN(13),	{ 0x000001e2, 0x000001e2 } },	\
	{ AR5K_RF_GAIN(14),	{ 0x00000022, 0x00000022 } },	\
	{ AR5K_RF_GAIN(15),	{ 0x00000062, 0x00000062 } },	\
	{ AR5K_RF_GAIN(16),	{ 0x00000163, 0x00000163 } },	\
	{ AR5K_RF_GAIN(17),	{ 0x000001a3, 0x000001a3 } },	\
	{ AR5K_RF_GAIN(18),	{ 0x000001e3, 0x000001e3 } },	\
	{ AR5K_RF_GAIN(19),	{ 0x00000023, 0x00000023 } },	\
	{ AR5K_RF_GAIN(20),	{ 0x00000063, 0x00000063 } },	\
	{ AR5K_RF_GAIN(21),	{ 0x00000184, 0x00000184 } },	\
	{ AR5K_RF_GAIN(22),	{ 0x000001c4, 0x000001c4 } },	\
	{ AR5K_RF_GAIN(23),	{ 0x00000004, 0x00000004 } },	\
	{ AR5K_RF_GAIN(24),	{ 0x000001ea, 0x0000000b } },	\
	{ AR5K_RF_GAIN(25),	{ 0x0000002a, 0x0000004b } },	\
	{ AR5K_RF_GAIN(26),	{ 0x0000006a, 0x0000008b } },	\
	{ AR5K_RF_GAIN(27),	{ 0x000000aa, 0x000001ac } },	\
	{ AR5K_RF_GAIN(28),	{ 0x000001ab, 0x000001ec } },	\
	{ AR5K_RF_GAIN(29),	{ 0x000001eb, 0x0000002c } },	\
	{ AR5K_RF_GAIN(30),	{ 0x0000002b, 0x00000012 } },	\
	{ AR5K_RF_GAIN(31),	{ 0x0000006b, 0x00000052 } },	\
	{ AR5K_RF_GAIN(32),	{ 0x000000ab, 0x00000092 } },	\
	{ AR5K_RF_GAIN(33),	{ 0x000001ac, 0x00000193 } },	\
	{ AR5K_RF_GAIN(34),	{ 0x000001ec, 0x000001d3 } },	\
	{ AR5K_RF_GAIN(35),	{ 0x0000002c, 0x00000013 } },	\
	{ AR5K_RF_GAIN(36),	{ 0x0000003a, 0x00000053 } },	\
	{ AR5K_RF_GAIN(37),	{ 0x0000007a, 0x00000093 } },	\
	{ AR5K_RF_GAIN(38),	{ 0x000000ba, 0x00000194 } },	\
	{ AR5K_RF_GAIN(39),	{ 0x000001bb, 0x000001d4 } },	\
	{ AR5K_RF_GAIN(40),	{ 0x000001fb, 0x00000014 } },	\
	{ AR5K_RF_GAIN(41),	{ 0x0000003b, 0x0000003a } },	\
	{ AR5K_RF_GAIN(42),	{ 0x0000007b, 0x0000007a } },	\
	{ AR5K_RF_GAIN(43),	{ 0x000000bb, 0x000000ba } },	\
	{ AR5K_RF_GAIN(44),	{ 0x000001bc, 0x000001bb } },	\
	{ AR5K_RF_GAIN(45),	{ 0x000001fc, 0x000001fb } },	\
	{ AR5K_RF_GAIN(46),	{ 0x0000003c, 0x0000003b } },	\
	{ AR5K_RF_GAIN(47),	{ 0x0000007c, 0x0000007b } },	\
	{ AR5K_RF_GAIN(48),	{ 0x000000bc, 0x000000bb } },	\
	{ AR5K_RF_GAIN(49),	{ 0x000000fc, 0x000001bc } },	\
	{ AR5K_RF_GAIN(50),	{ 0x000000fc, 0x000001fc } },	\
	{ AR5K_RF_GAIN(51),	{ 0x000000fc, 0x0000003c } },	\
	{ AR5K_RF_GAIN(52),	{ 0x000000fc, 0x0000007c } },	\
	{ AR5K_RF_GAIN(53),	{ 0x000000fc, 0x000000bc } },	\
	{ AR5K_RF_GAIN(54),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(55),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(56),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(57),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(58),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(59),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(60),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(61),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(62),	{ 0x000000fc, 0x000000fc } },	\
	{ AR5K_RF_GAIN(63),	{ 0x000000fc, 0x000000fc } },	\
}



/*
 * MAC/PHY REGISTERS
 */

/*
 * Mode-independed initial register writes
 */

struct ath5k_ini {
	u_int16_t	ini_register;
	u_int32_t	ini_value;

	enum {
		AR5K_INI_WRITE = 0,	/* Default */
		AR5K_INI_READ = 1,	/* Cleared on read */
	} ini_mode;
};

/* Initial register settings for AR5210 */
#define AR5K_AR5210_INI {		\
	/* PCU and MAC registers */	\
	{ AR5K_NOQCU_TXDP0,	0 },	\
	{ AR5K_NOQCU_TXDP1,	0 },	\
	{ AR5K_RXDP,		0 },	\
	{ AR5K_CR,		0 },	\
	{ AR5K_ISR,		0, AR5K_INI_READ },	\
	{ AR5K_IMR,		0 },	\
	{ AR5K_IER,		AR5K_IER_DISABLE },	\
	{ AR5K_BSR,		0, AR5K_INI_READ },	\
	{ AR5K_TXCFG,		AR5K_DMASIZE_128B },	\
	{ AR5K_RXCFG,		AR5K_DMASIZE_128B },	\
	{ AR5K_CFG,		AR5K_INIT_CFG },	\
	{ AR5K_TOPS,		AR5K_INIT_TOPS },	\
	{ AR5K_RXNOFRM,		AR5K_INIT_RXNOFRM },	\
	{ AR5K_RPGTO,		AR5K_INIT_RPGTO },	\
	{ AR5K_TXNOFRM,		AR5K_INIT_TXNOFRM },	\
	{ AR5K_SFR,		0 },	\
	{ AR5K_MIBC,		0 },	\
	{ AR5K_MISC,		0 },	\
	{ AR5K_RX_FILTER_5210,	0 },	\
	{ AR5K_MCAST_FILTER0_5210, 0 },	\
	{ AR5K_MCAST_FILTER1_5210, 0 },	\
	{ AR5K_TX_MASK0,	0 },	\
	{ AR5K_TX_MASK1,	0 },	\
	{ AR5K_CLR_TMASK,	0 },	\
	{ AR5K_TRIG_LVL,	AR5K_TUNE_MIN_TX_FIFO_THRES },	\
	{ AR5K_DIAG_SW_5210,	0 },	\
	{ AR5K_RSSI_THR,	AR5K_TUNE_RSSI_THRES },	\
	{ AR5K_TSF_L32_5210,	0 },	\
	{ AR5K_TIMER0_5210,	0 },	\
	{ AR5K_TIMER1_5210,	0xffffffff },	\
	{ AR5K_TIMER2_5210,	0xffffffff },	\
	{ AR5K_TIMER3_5210,	1 },	\
	{ AR5K_CFP_DUR_5210,	0 },	\
	{ AR5K_CFP_PERIOD_5210,	0 },	\
	/* PHY registers */		\
	{ AR5K_PHY(0),	0x00000047 },	\
	{ AR5K_PHY_AGC,	0x00000000 },	\
	{ AR5K_PHY(3),	0x09848ea6 },	\
	{ AR5K_PHY(4),	0x3d32e000 },	\
	{ AR5K_PHY(5),	0x0000076b },	\
	{ AR5K_PHY_ACT,	AR5K_PHY_ACT_DISABLE },	\
	{ AR5K_PHY(8),	0x02020200 },	\
	{ AR5K_PHY(9),	0x00000e0e },	\
	{ AR5K_PHY(10),	0x0a020201 },	\
	{ AR5K_PHY(11),	0x00036ffc },	\
	{ AR5K_PHY(12),	0x00000000 },	\
	{ AR5K_PHY(13),	0x00000e0e },	\
	{ AR5K_PHY(14),	0x00000007 },	\
	{ AR5K_PHY(15),	0x00020100 },	\
	{ AR5K_PHY(16),	0x89630000 },	\
	{ AR5K_PHY(17),	0x1372169c },	\
	{ AR5K_PHY(18),	0x0018b633 },	\
	{ AR5K_PHY(19),	0x1284613c },	\
	{ AR5K_PHY(20),	0x0de8b8e0 },	\
	{ AR5K_PHY(21),	0x00074859 },	\
	{ AR5K_PHY(22),	0x7e80beba },	\
	{ AR5K_PHY(23),	0x313a665e },	\
	{ AR5K_PHY_AGCCTL, 0x00001d08 },\
	{ AR5K_PHY(25),	0x0001ce00 },	\
	{ AR5K_PHY(26),	0x409a4190 },	\
	{ AR5K_PHY(28),	0x0000000f },	\
	{ AR5K_PHY(29),	0x00000080 },	\
	{ AR5K_PHY(30),	0x00000004 },	\
	{ AR5K_PHY(31),	0x00000018 }, 	/* 0x987c */	\
	{ AR5K_PHY(64),	0x00000000 }, 	/* 0x9900 */	\
	{ AR5K_PHY(65),	0x00000000 },	\
	{ AR5K_PHY(66),	0x00000000 },	\
	{ AR5K_PHY(67),	0x00800000 },	\
	{ AR5K_PHY(68),	0x00000003 },	\
	/* BB gain table (64bytes) */	\
	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000020 },	\
	{ AR5K_BB_GAIN(2), 0x00000010 },	\
	{ AR5K_BB_GAIN(3), 0x00000030 },	\
	{ AR5K_BB_GAIN(4), 0x00000008 },	\
	{ AR5K_BB_GAIN(5), 0x00000028 },	\
	{ AR5K_BB_GAIN(6), 0x00000028 },	\
	{ AR5K_BB_GAIN(7), 0x00000004 },	\
	{ AR5K_BB_GAIN(8), 0x00000024 },	\
	{ AR5K_BB_GAIN(9), 0x00000014 },	\
	{ AR5K_BB_GAIN(10), 0x00000034 },	\
	{ AR5K_BB_GAIN(11), 0x0000000c },	\
	{ AR5K_BB_GAIN(12), 0x0000002c },	\
	{ AR5K_BB_GAIN(13), 0x00000002 },	\
	{ AR5K_BB_GAIN(14), 0x00000022 },	\
	{ AR5K_BB_GAIN(15), 0x00000012 },	\
	{ AR5K_BB_GAIN(16), 0x00000032 },	\
	{ AR5K_BB_GAIN(17), 0x0000000a },	\
	{ AR5K_BB_GAIN(18), 0x0000002a },	\
	{ AR5K_BB_GAIN(19), 0x00000001 },	\
	{ AR5K_BB_GAIN(20), 0x00000021 },	\
	{ AR5K_BB_GAIN(21), 0x00000011 },	\
	{ AR5K_BB_GAIN(22), 0x00000031 },	\
	{ AR5K_BB_GAIN(23), 0x00000009 },	\
	{ AR5K_BB_GAIN(24), 0x00000029 },	\
	{ AR5K_BB_GAIN(25), 0x00000005 },	\
	{ AR5K_BB_GAIN(26), 0x00000025 },	\
	{ AR5K_BB_GAIN(27), 0x00000015 },	\
	{ AR5K_BB_GAIN(28), 0x00000035 },	\
	{ AR5K_BB_GAIN(29), 0x0000000d },	\
	{ AR5K_BB_GAIN(30), 0x0000002d },	\
	{ AR5K_BB_GAIN(31), 0x00000003 },	\
	{ AR5K_BB_GAIN(32), 0x00000023 },	\
	{ AR5K_BB_GAIN(33), 0x00000013 },	\
	{ AR5K_BB_GAIN(34), 0x00000033 },	\
	{ AR5K_BB_GAIN(35), 0x0000000b },	\
	{ AR5K_BB_GAIN(36), 0x0000002b },	\
	{ AR5K_BB_GAIN(37), 0x00000007 },	\
	{ AR5K_BB_GAIN(38), 0x00000027 },	\
	{ AR5K_BB_GAIN(39), 0x00000017 },	\
	{ AR5K_BB_GAIN(40), 0x00000037 },	\
	{ AR5K_BB_GAIN(41), 0x0000000f },	\
	{ AR5K_BB_GAIN(42), 0x0000002f },	\
	{ AR5K_BB_GAIN(43), 0x0000002f },	\
	{ AR5K_BB_GAIN(44), 0x0000002f },	\
	{ AR5K_BB_GAIN(45), 0x0000002f },	\
	{ AR5K_BB_GAIN(46), 0x0000002f },	\
	{ AR5K_BB_GAIN(47), 0x0000002f },	\
	{ AR5K_BB_GAIN(48), 0x0000002f },	\
	{ AR5K_BB_GAIN(49), 0x0000002f },	\
	{ AR5K_BB_GAIN(50), 0x0000002f },	\
	{ AR5K_BB_GAIN(51), 0x0000002f },	\
	{ AR5K_BB_GAIN(52), 0x0000002f },	\
	{ AR5K_BB_GAIN(53), 0x0000002f },	\
	{ AR5K_BB_GAIN(54), 0x0000002f },	\
	{ AR5K_BB_GAIN(55), 0x0000002f },	\
	{ AR5K_BB_GAIN(56), 0x0000002f },	\
	{ AR5K_BB_GAIN(57), 0x0000002f },	\
	{ AR5K_BB_GAIN(58), 0x0000002f },	\
	{ AR5K_BB_GAIN(59), 0x0000002f },	\
	{ AR5K_BB_GAIN(60), 0x0000002f },	\
	{ AR5K_BB_GAIN(61), 0x0000002f },	\
	{ AR5K_BB_GAIN(62), 0x0000002f },	\
	{ AR5K_BB_GAIN(63), 0x0000002f },	\
	/* 5110 RF gain table (64btes) */	\
	{ AR5K_RF_GAIN(0), 0x0000001d },	\
	{ AR5K_RF_GAIN(1), 0x0000005d },	\
	{ AR5K_RF_GAIN(2), 0x0000009d },	\
	{ AR5K_RF_GAIN(3), 0x000000dd },	\
	{ AR5K_RF_GAIN(4), 0x0000011d },	\
	{ AR5K_RF_GAIN(5), 0x00000021 },	\
	{ AR5K_RF_GAIN(6), 0x00000061 },	\
	{ AR5K_RF_GAIN(7), 0x000000a1 },	\
	{ AR5K_RF_GAIN(8), 0x000000e1 },	\
	{ AR5K_RF_GAIN(9), 0x00000031 },	\
	{ AR5K_RF_GAIN(10), 0x00000071 },	\
	{ AR5K_RF_GAIN(11), 0x000000b1 },	\
	{ AR5K_RF_GAIN(12), 0x0000001c },	\
	{ AR5K_RF_GAIN(13), 0x0000005c },	\
	{ AR5K_RF_GAIN(14), 0x00000029 },	\
	{ AR5K_RF_GAIN(15), 0x00000069 },	\
	{ AR5K_RF_GAIN(16), 0x000000a9 },	\
	{ AR5K_RF_GAIN(17), 0x00000020 },	\
	{ AR5K_RF_GAIN(18), 0x00000019 },	\
	{ AR5K_RF_GAIN(19), 0x00000059 },	\
	{ AR5K_RF_GAIN(20), 0x00000099 },	\
	{ AR5K_RF_GAIN(21), 0x00000030 },	\
	{ AR5K_RF_GAIN(22), 0x00000005 },	\
	{ AR5K_RF_GAIN(23), 0x00000025 },	\
	{ AR5K_RF_GAIN(24), 0x00000065 },	\
	{ AR5K_RF_GAIN(25), 0x000000a5 },	\
	{ AR5K_RF_GAIN(26), 0x00000028 },	\
	{ AR5K_RF_GAIN(27), 0x00000068 },	\
	{ AR5K_RF_GAIN(28), 0x0000001f },	\
	{ AR5K_RF_GAIN(29), 0x0000001e },	\
	{ AR5K_RF_GAIN(30), 0x00000018 },	\
	{ AR5K_RF_GAIN(31), 0x00000058 },	\
	{ AR5K_RF_GAIN(32), 0x00000098 },	\
	{ AR5K_RF_GAIN(33), 0x00000003 },	\
	{ AR5K_RF_GAIN(34), 0x00000004 },	\
	{ AR5K_RF_GAIN(35), 0x00000044 },	\
	{ AR5K_RF_GAIN(36), 0x00000084 },	\
	{ AR5K_RF_GAIN(37), 0x00000013 },	\
	{ AR5K_RF_GAIN(38), 0x00000012 },	\
	{ AR5K_RF_GAIN(39), 0x00000052 },	\
	{ AR5K_RF_GAIN(40), 0x00000092 },	\
	{ AR5K_RF_GAIN(41), 0x000000d2 },	\
	{ AR5K_RF_GAIN(42), 0x0000002b },	\
	{ AR5K_RF_GAIN(43), 0x0000002a },	\
	{ AR5K_RF_GAIN(44), 0x0000006a },	\
	{ AR5K_RF_GAIN(45), 0x000000aa },	\
	{ AR5K_RF_GAIN(46), 0x0000001b },	\
	{ AR5K_RF_GAIN(47), 0x0000001a },	\
	{ AR5K_RF_GAIN(48), 0x0000005a },	\
	{ AR5K_RF_GAIN(49), 0x0000009a },	\
	{ AR5K_RF_GAIN(50), 0x000000da },	\
	{ AR5K_RF_GAIN(51), 0x00000006 },	\
	{ AR5K_RF_GAIN(52), 0x00000006 },	\
	{ AR5K_RF_GAIN(53), 0x00000006 },	\
	{ AR5K_RF_GAIN(54), 0x00000006 },	\
	{ AR5K_RF_GAIN(55), 0x00000006 },	\
	{ AR5K_RF_GAIN(56), 0x00000006 },	\
	{ AR5K_RF_GAIN(57), 0x00000006 },	\
	{ AR5K_RF_GAIN(58), 0x00000006 },	\
	{ AR5K_RF_GAIN(59), 0x00000006 },	\
	{ AR5K_RF_GAIN(60), 0x00000006 },	\
	{ AR5K_RF_GAIN(61), 0x00000006 },	\
	{ AR5K_RF_GAIN(62), 0x00000006 },	\
	{ AR5K_RF_GAIN(63), 0x00000006 },	\
	/* PHY activation */			\
	{ AR5K_PHY(53), 0x00000020 },		\
	{ AR5K_PHY(51), 0x00000004 },		\
	{ AR5K_PHY(50), 0x00060106 },		\
	{ AR5K_PHY(39), 0x0000006d },		\
	{ AR5K_PHY(48), 0x00000000 },		\
	{ AR5K_PHY(52), 0x00000014 },		\
	{ AR5K_PHY_ACT, AR5K_PHY_ACT_ENABLE },	\
}


/* Initial register settings for AR5211 */
#define AR5K_AR5211_INI {			\
	{ AR5K_RXDP,		0x00000000 },	\
	{ AR5K_RTSD0,		0x84849c9c },	\
	{ AR5K_RTSD1,		0x7c7c7c7c },	\
	{ AR5K_RXCFG,		0x00000005 },	\
	{ AR5K_MIBC,		0x00000000 },	\
	{ AR5K_TOPS,		0x00000008 },	\
	{ AR5K_RXNOFRM,		0x00000008 },	\
	{ AR5K_TXNOFRM,		0x00000010 },	\
	{ AR5K_RPGTO,		0x00000000 },	\
	{ AR5K_RFCNT,		0x0000001f },	\
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },	\
	{ AR5K_DCU_FP,		0x00000000 },	\
	{ AR5K_STA_ID1,		0x00000000 },	\
	{ AR5K_BSS_ID0,		0x00000000 },	\
	{ AR5K_BSS_ID1,		0x00000000 },	\
	{ AR5K_RSSI_THR,	0x00000000 },	\
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },	\
	{ AR5K_TIMER0_5211,	0x00000030 },	\
	{ AR5K_TIMER1_5211,	0x0007ffff },	\
	{ AR5K_TIMER2_5211,	0x01ffffff },	\
	{ AR5K_TIMER3_5211,	0x00000031 },	\
	{ AR5K_CFP_DUR_5211,	0x00000000 },	\
	{ AR5K_RX_FILTER_5211,	0x00000000 },	\
	{ AR5K_MCAST_FILTER0_5211, 0x00000000 },\
	{ AR5K_MCAST_FILTER1_5211, 0x00000002 },\
	{ AR5K_DIAG_SW_5211,	0x00000000 },	\
	{ AR5K_ADDAC_TEST,	0x00000000 },	\
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },	\
        /* PHY registers */			\
	{ AR5K_PHY_AGC,	0x00000000 },		\
	{ AR5K_PHY(3),	0x2d849093 },		\
	{ AR5K_PHY(4),	0x7d32e000 },		\
	{ AR5K_PHY(5),	0x00000f6b },		\
	{ AR5K_PHY_ACT,	0x00000000 },		\
	{ AR5K_PHY(11),	0x00026ffe },		\
	{ AR5K_PHY(12),	0x00000000 },		\
	{ AR5K_PHY(15),	0x00020100 },		\
	{ AR5K_PHY(16),	0x206a017a },		\
	{ AR5K_PHY(19),	0x1284613c },		\
	{ AR5K_PHY(21),	0x00000859 },		\
	{ AR5K_PHY(26),	0x409a4190 },	/* 0x9868 */	\
	{ AR5K_PHY(27),	0x050cb081 },		\
	{ AR5K_PHY(28),	0x0000000f },		\
	{ AR5K_PHY(29),	0x00000080 },		\
	{ AR5K_PHY(30),	0x0000000c },		\
	{ AR5K_PHY(64),	0x00000000 },		\
	{ AR5K_PHY(65),	0x00000000 },		\
	{ AR5K_PHY(66),	0x00000000 },		\
	{ AR5K_PHY(67),	0x00800000 },		\
	{ AR5K_PHY(68),	0x00000001 },		\
	{ AR5K_PHY(71),	0x0000092a },		\
	{ AR5K_PHY_IQ,	0x00000000 },		\
	{ AR5K_PHY(73),	0x00058a05 },		\
	{ AR5K_PHY(74),	0x00000001 },		\
	{ AR5K_PHY(75),	0x00000000 },		\
	{ AR5K_PHY_PAPD_PROBE, 0x00000000 },	\
	{ AR5K_PHY(77),	0x00000000 },	/* 0x9934 */	\
	{ AR5K_PHY(78),	0x00000000 },	/* 0x9938 */	\
	{ AR5K_PHY(79),	0x0000003f },	/* 0x993c */	\
	{ AR5K_PHY(80),	0x00000004 },		\
	{ AR5K_PHY(82),	0x00000000 },		\
	{ AR5K_PHY(83),	0x00000000 },		\
	{ AR5K_PHY(84),	0x00000000 },		\
	{ AR5K_PHY_RADAR, 0x5d50f14c },		\
	{ AR5K_PHY(86),	0x00000018 },		\
	{ AR5K_PHY(87),	0x004b6a8e },		\
	/* Power table (32bytes) */			\
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x06ff05ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x07ff07ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x08ff08ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x09ff09ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x0aff0aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x0bff0bff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x0cff0cff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x0dff0dff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x0fff0eff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x12ff12ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x14ff13ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x16ff15ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x19ff17ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x1bff1aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x1eff1dff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x23ff20ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x27ff25ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x2cff29ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x31ff2fff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x37ff34ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(31), 0x3aff3aff },	\
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },	\
	{ AR5K_PHY(642), 0x503e4646 },		\
	{ AR5K_PHY_GAIN_2GHZ, 0x6480416c },	\
	{ AR5K_PHY(644), 0x0199a003 },		\
	{ AR5K_PHY(645), 0x044cd610 },		\
	{ AR5K_PHY(646), 0x13800040 },		\
	{ AR5K_PHY(647), 0x1be00060 },		\
	{ AR5K_PHY(648), 0x0c53800a },		\
	{ AR5K_PHY(649), 0x0014df3b },		\
	{ AR5K_PHY(650), 0x000001b5 },		\
	{ AR5K_PHY(651), 0x00000020 },		\
}

/* Initial register settings for AR5212 */
#define AR5K_AR5212_INI {			\
	{ AR5K_RXDP,		0x00000000 },	\
	{ AR5K_RXCFG,		0x00000005 },	\
	{ AR5K_MIBC,		0x00000000 },	\
	{ AR5K_TOPS,		0x00000008 },	\
	{ AR5K_RXNOFRM,		0x00000008 },	\
	{ AR5K_TXNOFRM,		0x00000010 },	\
	{ AR5K_RPGTO,		0x00000000 },	\
	{ AR5K_RFCNT,		0x0000001f },	\
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },	\
	{ AR5K_DCU_FP,		0x00000000 },	\
	{ AR5K_DCU_TXP,		0x00000000 },	\
	{ AR5K_DCU_TX_FILTER,	0x00000000 },	\
	/* Unknown table */			\
	{ 0x1078, 0x00000000 },			\
	{ 0x10b8, 0x00000000 },			\
	{ 0x10f8, 0x00000000 },			\
	{ 0x1138, 0x00000000 },			\
	{ 0x1178, 0x00000000 },			\
	{ 0x11b8, 0x00000000 },			\
	{ 0x11f8, 0x00000000 },			\
	{ 0x1238, 0x00000000 },			\
	{ 0x1278, 0x00000000 },			\
	{ 0x12b8, 0x00000000 },			\
	{ 0x12f8, 0x00000000 },			\
	{ 0x1338, 0x00000000 },			\
	{ 0x1378, 0x00000000 },			\
	{ 0x13b8, 0x00000000 },			\
	{ 0x13f8, 0x00000000 },			\
	{ 0x1438, 0x00000000 },			\
	{ 0x1478, 0x00000000 },			\
	{ 0x14b8, 0x00000000 },			\
	{ 0x14f8, 0x00000000 },			\
	{ 0x1538, 0x00000000 },			\
	{ 0x1578, 0x00000000 },			\
	{ 0x15b8, 0x00000000 },			\
	{ 0x15f8, 0x00000000 },			\
	{ 0x1638, 0x00000000 },			\
	{ 0x1678, 0x00000000 },			\
	{ 0x16b8, 0x00000000 },			\
	{ 0x16f8, 0x00000000 },			\
	{ 0x1738, 0x00000000 },			\
	{ 0x1778, 0x00000000 },			\
	{ 0x17b8, 0x00000000 },			\
	{ 0x17f8, 0x00000000 },			\
	{ 0x103c, 0x00000000 },			\
	{ 0x107c, 0x00000000 },			\
	{ 0x10bc, 0x00000000 },			\
	{ 0x10fc, 0x00000000 },			\
	{ 0x113c, 0x00000000 },			\
	{ 0x117c, 0x00000000 },			\
	{ 0x11bc, 0x00000000 },			\
	{ 0x11fc, 0x00000000 },			\
	{ 0x123c, 0x00000000 },			\
	{ 0x127c, 0x00000000 },			\
	{ 0x12bc, 0x00000000 },			\
	{ 0x12fc, 0x00000000 },			\
	{ 0x133c, 0x00000000 },			\
	{ 0x137c, 0x00000000 },			\
	{ 0x13bc, 0x00000000 },			\
	{ 0x13fc, 0x00000000 },			\
	{ 0x143c, 0x00000000 },			\
	{ 0x147c, 0x00000000 },			\
	{ AR5K_STA_ID1,		0x00000000 },	\
	{ AR5K_BSS_ID0,		0x00000000 },	\
	{ AR5K_BSS_ID1,		0x00000000 },	\
	{ AR5K_RSSI_THR,	0x00000000 },	\
	{ AR5K_BEACON_5211,	0x00000000 },	\
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },	\
	{ AR5K_TIMER0_5211,	0x00000030 },	\
	{ AR5K_TIMER1_5211,	0x0007ffff },	\
	{ AR5K_TIMER2_5211,	0x01ffffff },	\
	{ AR5K_TIMER3_5211,	0x00000031 },	\
	{ AR5K_CFP_DUR_5211,	0x00000000 },	\
	{ AR5K_RX_FILTER_5211,	0x00000000 },	\
	{ AR5K_DIAG_SW_5211,	0x00000000 },	\
	{ AR5K_ADDAC_TEST,	0x00000000 },	\
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },	\
	{ 0x805c, 0xffffc7ff },			\
	{ 0x8080, 0x00000000 },			\
	{ AR5K_NAV_5211,	0x00000000 },	\
	{ AR5K_RTS_OK_5211,	0x00000000 },	\
	{ AR5K_RTS_FAIL_5211,	0x00000000 },	\
	{ AR5K_ACK_FAIL_5211,	0x00000000 },	\
	{ AR5K_FCS_FAIL_5211,	0x00000000 },	\
	{ AR5K_BEACON_CNT_5211,	0x00000000 },	\
	{ AR5K_XRMODE,		0x2a82301a },	\
	{ AR5K_XRDELAY,		0x05dc01e0 },	\
	{ AR5K_XRTIMEOUT,	0x1f402710 },	\
	{ AR5K_XRCHIRP,		0x01f40000 },	\
	{ AR5K_XRSTOMP,		0x00001e1c },	\
	{ AR5K_SLEEP0,		0x0002aaaa },	\
	{ AR5K_SLEEP1,		0x02005555 },	\
	{ AR5K_SLEEP2,		0x00000000 },	\
	{ AR5K_BSS_IDM0,	0xffffffff },	\
	{ AR5K_BSS_IDM1,	0x0000ffff },	\
	{ AR5K_TXPC,		0x00000000 },	\
	{ AR5K_PROFCNT_TX,	0x00000000 },	\
	{ AR5K_PROFCNT_RX,	0x00000000 },	\
	{ AR5K_PROFCNT_RXCLR,	0x00000000 },	\
	{ AR5K_PROFCNT_CYCLE,	0x00000000 },	\
	{ 0x80fc, 0x00000088 },			\
	{ AR5K_RATE_DUR(0),	0x00000000 },	\
	{ AR5K_RATE_DUR(1),	0x0000008c },	\
	{ AR5K_RATE_DUR(2),	0x000000e4 },	\
	{ AR5K_RATE_DUR(3),	0x000002d5 },	\
	{ AR5K_RATE_DUR(4),	0x00000000 },	\
	{ AR5K_RATE_DUR(5),	0x00000000 },	\
	{ AR5K_RATE_DUR(6),	0x000000a0 },	\
	{ AR5K_RATE_DUR(7),	0x000001c9 },	\
	{ AR5K_RATE_DUR(8),	0x0000002c },	\
	{ AR5K_RATE_DUR(9),	0x0000002c },	\
	{ AR5K_RATE_DUR(10),	0x00000030 },	\
	{ AR5K_RATE_DUR(11),	0x0000003c },	\
	{ AR5K_RATE_DUR(12),	0x0000002c },	\
	{ AR5K_RATE_DUR(13),	0x0000002c },	\
	{ AR5K_RATE_DUR(14),	0x00000030 },	\
	{ AR5K_RATE_DUR(15),	0x0000003c },	\
	{ AR5K_RATE_DUR(16),	0x00000000 },	\
	{ AR5K_RATE_DUR(17),	0x00000000 },	\
	{ AR5K_RATE_DUR(18),	0x00000000 },	\
	{ AR5K_RATE_DUR(19),	0x00000000 },	\
	{ AR5K_RATE_DUR(20),	0x00000000 },	\
	{ AR5K_RATE_DUR(21),	0x00000000 },	\
	{ AR5K_RATE_DUR(22),	0x00000000 },	\
	{ AR5K_RATE_DUR(23),	0x00000000 },	\
	{ AR5K_RATE_DUR(24),	0x000000d5 },	\
	{ AR5K_RATE_DUR(25),	0x000000df },	\
	{ AR5K_RATE_DUR(26),	0x00000102 },	\
	{ AR5K_RATE_DUR(27),	0x0000013a },	\
	{ AR5K_RATE_DUR(28),	0x00000075 },	\
	{ AR5K_RATE_DUR(29),	0x0000007f },	\
	{ AR5K_RATE_DUR(30),	0x000000a2 },	\
	{ AR5K_RATE_DUR(31),	0x00000000 },	\
	{ 0x8100, 0x00010002},			\
	{ AR5K_TSF_PARM,	0x00000001 },	\
	{ 0x8108, 0x000000c0 },			\
	{ AR5K_PHY_ERR_FIL,	0x00000000 },	\
	{ 0x8110, 0x00000168 },			\
	{ 0x8114, 0x00000000 },			\
	/* Some kind of table			\
	 * also notice ...03<-02<-01<-00) */	\
	{ 0x87c0, 0x03020100 },			\
	{ 0x87c4, 0x07060504 },			\
	{ 0x87c8, 0x0b0a0908 },			\
	{ 0x87cc, 0x0f0e0d0c },			\
	{ 0x87d0, 0x13121110 },			\
	{ 0x87d4, 0x17161514 },			\
	{ 0x87d8, 0x1b1a1918 },			\
	{ 0x87dc, 0x1f1e1d1c },			\
	/* loop ? */				\
	{ 0x87e0, 0x03020100 },			\
	{ 0x87e4, 0x07060504 },			\
	{ 0x87e8, 0x0b0a0908 },			\
	{ 0x87ec, 0x0f0e0d0c },			\
	{ 0x87f0, 0x13121110 },			\
	{ 0x87f4, 0x17161514 },			\
	{ 0x87f8, 0x1b1a1918 },			\
	{ 0x87fc, 0x1f1e1d1c },			\
	/* PHY registers */			\
	{ AR5K_PHY_AGC,	0x00000000 },		\
	{ AR5K_PHY(3),	0xad848e19 },		\
	{ AR5K_PHY(4),	0x7d28e000 },		\
	{ AR5K_PHY_TIMING_3, 0x9c0a9f6b },	\
	{ AR5K_PHY_ACT,	0x00000000 },		\
	{ AR5K_PHY(11),	0x00022ffe },		\
	{ AR5K_PHY(15),	0x00020100 },		\
	{ AR5K_PHY(16),	0x206a017a },		\
	{ AR5K_PHY(19),	0x1284613c },		\
	{ AR5K_PHY(21),	0x00000859 },		\
	{ AR5K_PHY(64),	0x00000000 },		\
	{ AR5K_PHY(65),	0x00000000 },		\
	{ AR5K_PHY(66),	0x00000000 },		\
	{ AR5K_PHY(67),	0x00800000 },		\
	{ AR5K_PHY(68),	0x00000001 },		\
	{ AR5K_PHY(71),	0x0000092a },		\
	{ AR5K_PHY_IQ,	0x05100000 },		\
	{ AR5K_PHY(74), 0x00000001 },		\
	{ AR5K_PHY(75), 0x00000004 },		\
	{ AR5K_PHY_TXPOWER_RATE1, 0x1e1f2022 },	\
	{ AR5K_PHY_TXPOWER_RATE2, 0x0a0b0c0d },	\
	{ AR5K_PHY_TXPOWER_RATE_MAX, 0x0000003f },\
	{ AR5K_PHY(80), 0x00000004 },		\
	{ AR5K_PHY(82), 0x9280b212 },		\
	{ AR5K_PHY_RADAR, 0x5d50e188 },		\
	{ AR5K_PHY(86),	0x000000ff },		\
	{ AR5K_PHY(87),	0x004b6a8e },		\
	{ AR5K_PHY(90),	0x000003ce },		\
	{ AR5K_PHY(92),	0x192fb515 },		\
	{ AR5K_PHY(93),	0x00000000 },		\
	{ AR5K_PHY(94),	0x00000001 },		\
	{ AR5K_PHY(95),	0x00000000 },		\
	/* Power table (32bytes) */		\
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(31),0x10ff10ff },	\
	{ AR5K_PHY(644), 0x0080a333 },		\
	{ AR5K_PHY(645), 0x00206c10 },		\
	{ AR5K_PHY(646), 0x009c4060 },		\
	{ AR5K_PHY(647), 0x1483800a },		\
	{ AR5K_PHY(648), 0x01831061 },		\
	{ AR5K_PHY(649), 0x00000400 },		\
	{ AR5K_PHY(650), 0x000001b5 },		\
	{ AR5K_PHY(651), 0x00000000 },		\
	{ AR5K_PHY_TXPOWER_RATE3, 0x20202020 },	\
	{ AR5K_PHY_TXPOWER_RATE2, 0x20202020 },	\
	{ AR5K_PHY(655), 0x13c889af },		\
	{ AR5K_PHY(656), 0x38490a20 },		\
	{ AR5K_PHY(657), 0x00007bb6 },		\
	{ AR5K_PHY(658), 0x0fff3ffc },		\
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },	\
}

/*
 * Initial BaseBand Gain settings for RF5111/5112
 * (only AR5210 comes with RF5110 so initial
 * BB Gain settings are included in AR5K_AR5210_INI)
 */

/* RF5111 Initial BaseBand Gain settings */
#define AR5K_RF5111_INI_BBGAIN {		\
	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000020 },	\
	{ AR5K_BB_GAIN(2), 0x00000010 },	\
	{ AR5K_BB_GAIN(3), 0x00000030 },	\
	{ AR5K_BB_GAIN(4), 0x00000008 },	\
	{ AR5K_BB_GAIN(5), 0x00000028 },	\
	{ AR5K_BB_GAIN(6), 0x00000004 },	\
	{ AR5K_BB_GAIN(7), 0x00000024 },	\
	{ AR5K_BB_GAIN(8), 0x00000014 },	\
	{ AR5K_BB_GAIN(9), 0x00000034 },	\
	{ AR5K_BB_GAIN(10), 0x0000000c },	\
	{ AR5K_BB_GAIN(11), 0x0000002c },	\
	{ AR5K_BB_GAIN(12), 0x00000002 },	\
	{ AR5K_BB_GAIN(13), 0x00000022 },	\
	{ AR5K_BB_GAIN(14), 0x00000012 },	\
	{ AR5K_BB_GAIN(15), 0x00000032 },	\
	{ AR5K_BB_GAIN(16), 0x0000000a },	\
	{ AR5K_BB_GAIN(17), 0x0000002a },	\
	{ AR5K_BB_GAIN(18), 0x00000006 },	\
	{ AR5K_BB_GAIN(19), 0x00000026 },	\
	{ AR5K_BB_GAIN(20), 0x00000016 },	\
	{ AR5K_BB_GAIN(21), 0x00000036 },	\
	{ AR5K_BB_GAIN(22), 0x0000000e },	\
	{ AR5K_BB_GAIN(23), 0x0000002e },	\
	{ AR5K_BB_GAIN(24), 0x00000001 },	\
	{ AR5K_BB_GAIN(25), 0x00000021 },	\
	{ AR5K_BB_GAIN(26), 0x00000011 },	\
	{ AR5K_BB_GAIN(27), 0x00000031 },	\
	{ AR5K_BB_GAIN(28), 0x00000009 },	\
	{ AR5K_BB_GAIN(29), 0x00000029 },	\
	{ AR5K_BB_GAIN(30), 0x00000005 },	\
	{ AR5K_BB_GAIN(31), 0x00000025 },	\
	{ AR5K_BB_GAIN(32), 0x00000015 },	\
	{ AR5K_BB_GAIN(33), 0x00000035 },	\
	{ AR5K_BB_GAIN(34), 0x0000000d },	\
	{ AR5K_BB_GAIN(35), 0x0000002d },	\
	{ AR5K_BB_GAIN(36), 0x00000003 },	\
	{ AR5K_BB_GAIN(37), 0x00000023 },	\
	{ AR5K_BB_GAIN(38), 0x00000013 },	\
	{ AR5K_BB_GAIN(39), 0x00000033 },	\
	{ AR5K_BB_GAIN(40), 0x0000000b },	\
	{ AR5K_BB_GAIN(41), 0x0000002b },	\
	{ AR5K_BB_GAIN(42), 0x0000002b },	\
	{ AR5K_BB_GAIN(43), 0x0000002b },	\
	{ AR5K_BB_GAIN(44), 0x0000002b },	\
	{ AR5K_BB_GAIN(45), 0x0000002b },	\
	{ AR5K_BB_GAIN(46), 0x0000002b },	\
	{ AR5K_BB_GAIN(47), 0x0000002b },	\
	{ AR5K_BB_GAIN(48), 0x0000002b },	\
	{ AR5K_BB_GAIN(49), 0x0000002b },	\
	{ AR5K_BB_GAIN(50), 0x0000002b },	\
	{ AR5K_BB_GAIN(51), 0x0000002b },	\
	{ AR5K_BB_GAIN(52), 0x0000002b },	\
	{ AR5K_BB_GAIN(53), 0x0000002b },	\
	{ AR5K_BB_GAIN(54), 0x0000002b },	\
	{ AR5K_BB_GAIN(55), 0x0000002b },	\
	{ AR5K_BB_GAIN(56), 0x0000002b },	\
	{ AR5K_BB_GAIN(57), 0x0000002b },	\
	{ AR5K_BB_GAIN(58), 0x0000002b },	\
	{ AR5K_BB_GAIN(59), 0x0000002b },	\
	{ AR5K_BB_GAIN(60), 0x0000002b },	\
	{ AR5K_BB_GAIN(61), 0x0000002b },	\
	{ AR5K_BB_GAIN(62), 0x00000002 },	\
	{ AR5K_BB_GAIN(63), 0x00000016 },	\
}

/* RF 5112 Initial BaseBand Gain settings */
#define AR5K_RF5112_INI_BBGAIN {		\
 	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000001 },	\
	{ AR5K_BB_GAIN(2), 0x00000002 },	\
	{ AR5K_BB_GAIN(3), 0x00000003 },	\
	{ AR5K_BB_GAIN(4), 0x00000004 },	\
	{ AR5K_BB_GAIN(5), 0x00000005 },	\
	{ AR5K_BB_GAIN(6), 0x00000008 },	\
	{ AR5K_BB_GAIN(7), 0x00000009 },	\
	{ AR5K_BB_GAIN(8), 0x0000000a },	\
	{ AR5K_BB_GAIN(9), 0x0000000b },	\
	{ AR5K_BB_GAIN(10), 0x0000000c },	\
	{ AR5K_BB_GAIN(11), 0x0000000d },	\
	{ AR5K_BB_GAIN(12), 0x00000010 },	\
	{ AR5K_BB_GAIN(13), 0x00000011 },	\
	{ AR5K_BB_GAIN(14), 0x00000012 },	\
	{ AR5K_BB_GAIN(15), 0x00000013 },	\
	{ AR5K_BB_GAIN(16), 0x00000014 },	\
	{ AR5K_BB_GAIN(17), 0x00000015 },	\
	{ AR5K_BB_GAIN(18), 0x00000018 },	\
	{ AR5K_BB_GAIN(19), 0x00000019 },	\
	{ AR5K_BB_GAIN(20), 0x0000001a },	\
	{ AR5K_BB_GAIN(21), 0x0000001b },	\
	{ AR5K_BB_GAIN(22), 0x0000001c },	\
	{ AR5K_BB_GAIN(23), 0x0000001d },	\
	{ AR5K_BB_GAIN(24), 0x00000020 },	\
	{ AR5K_BB_GAIN(25), 0x00000021 },	\
	{ AR5K_BB_GAIN(26), 0x00000022 },	\
	{ AR5K_BB_GAIN(27), 0x00000023 },	\
	{ AR5K_BB_GAIN(28), 0x00000024 },	\
	{ AR5K_BB_GAIN(29), 0x00000025 },	\
	{ AR5K_BB_GAIN(30), 0x00000028 },	\
	{ AR5K_BB_GAIN(31), 0x00000029 },	\
	{ AR5K_BB_GAIN(32), 0x0000002a },	\
	{ AR5K_BB_GAIN(33), 0x0000002b },	\
	{ AR5K_BB_GAIN(34), 0x0000002c },	\
	{ AR5K_BB_GAIN(35), 0x0000002d },	\
	{ AR5K_BB_GAIN(36), 0x00000030 },	\
	{ AR5K_BB_GAIN(37), 0x00000031 },	\
	{ AR5K_BB_GAIN(38), 0x00000032 },	\
	{ AR5K_BB_GAIN(39), 0x00000033 },	\
	{ AR5K_BB_GAIN(40), 0x00000034 },	\
	{ AR5K_BB_GAIN(41), 0x00000035 },	\
	{ AR5K_BB_GAIN(42), 0x00000035 },	\
	{ AR5K_BB_GAIN(43), 0x00000035 },	\
	{ AR5K_BB_GAIN(44), 0x00000035 },	\
	{ AR5K_BB_GAIN(45), 0x00000035 },	\
	{ AR5K_BB_GAIN(46), 0x00000035 },	\
	{ AR5K_BB_GAIN(47), 0x00000035 },	\
	{ AR5K_BB_GAIN(48), 0x00000035 },	\
	{ AR5K_BB_GAIN(49), 0x00000035 },	\
	{ AR5K_BB_GAIN(50), 0x00000035 },	\
	{ AR5K_BB_GAIN(51), 0x00000035 },	\
	{ AR5K_BB_GAIN(52), 0x00000035 },	\
	{ AR5K_BB_GAIN(53), 0x00000035 },	\
	{ AR5K_BB_GAIN(54), 0x00000035 },	\
	{ AR5K_BB_GAIN(55), 0x00000035 },	\
	{ AR5K_BB_GAIN(56), 0x00000035 },	\
	{ AR5K_BB_GAIN(57), 0x00000035 },	\
	{ AR5K_BB_GAIN(58), 0x00000035 },	\
	{ AR5K_BB_GAIN(59), 0x00000035 },	\
	{ AR5K_BB_GAIN(60), 0x00000035 },	\
	{ AR5K_BB_GAIN(61), 0x00000035 },	\
	{ AR5K_BB_GAIN(62), 0x00000010 },	\
	{ AR5K_BB_GAIN(63), 0x0000001a },	\
}


/*
 * Mode specific initial register values
 */

struct ath5k_ini_mode {
	u_int16_t	mode_register;
	u_int32_t	mode_value[5];
};

/* Initial mode-specific settings for AR5211
 * XXX: how about gTurbo ? RF5111 supports it, how about AR5211 ? */
#define AR5K_AR5211_INI_MODE {								\
	{ AR5K_TXCFG,									\
	/*         mode a/XR  mode aTurbo   mode b   mode g(OFDM?) mode gTurbo (N/A) */	\
 		{ 0x00000017, 0x00000017, 0x00000017, 0x00000017, 0x00000017 } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(0),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(1),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(2),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(3),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(4),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(5),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(6),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(7),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(8),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(9),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_DCU_GBL_IFS_SLOT,							\
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x00000168, 0x00000168 } },	\
	{ AR5K_DCU_GBL_IFS_SIFS,							\
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000230, 0x00000230 } },	\
	{ AR5K_DCU_GBL_IFS_EIFS,							\
		{ 0x00000d98, 0x00001180, 0x00001f48, 0x00000d98, 0x00000d98 } },	\
	{ AR5K_DCU_GBL_IFS_MISC,							\
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000a0e0, 0x0000a0e0 } },	\
	{ AR5K_TIME_OUT,								\
		{ 0x04000400, 0x08000800, 0x20003000, 0x04000400, 0x04000400 } },	\
	{ AR5K_USEC_5211,								\
		{ 0x0e8d8fa7, 0x0e8d8fcf, 0x01608f95, 0x0e8d8fa7, 0x0e8d8fa7 } },	\
	{ AR5K_PHY_TURBO,								\
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0x9820,									\
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },	\
	{ 0x9824,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9828,									\
		{ 0x0a020001, 0x0a020001, 0x05010000, 0x0a020001, 0x0a020001 } },	\
	{ 0x9834,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9838,									\
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },	\
	{ 0x9844,									\
		{ 0x1372169c, 0x137216a5, 0x137216a8, 0x1372169c, 0x1372169c } },	\
	{ 0x9848,									\
		{ 0x0018ba67, 0x0018ba67, 0x0018ba69, 0x0018ba69, 0x0018ba69 } },	\
	{ 0x9850,									\
		{ 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0 } },	\
	{ AR5K_PHY_SIG,									\
		{ 0x7e800d2e, 0x7e800d2e, 0x7ec00d2e, 0x7e800d2e, 0x7e800d2e } },	\
	{ AR5K_PHY_AGCCOARSE,								\
		{ 0x31375d5e, 0x31375d5e, 0x313a5d5e, 0x31375d5e, 0x31375d5e } },	\
	{ AR5K_PHY_AGCCTL,								\
		{ 0x0000bd10, 0x0000bd10, 0x0000bd38, 0x0000bd10, 0x0000bd10 } },	\
	{ AR5K_PHY_NF,									\
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },	\
	{ AR5K_PHY_RX_DELAY,								\
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002710, 0x00002710 } },	\
	{ 0x9918,									\
		{ 0x00000190, 0x00000190, 0x00000084, 0x00000190, 0x00000190 } },	\
	{ AR5K_PHY_FRAME_CTL_5211,							\
		{ 0x6fe01020, 0x6fe01020, 0x6fe00920, 0x6fe01020, 0x6fe01020 } },	\
	{ AR5K_PHY_PCDAC_TXPOWER(0),							\
		{ 0x05ff14ff, 0x05ff14ff, 0x05ff14ff, 0x05ff19ff, 0x05ff19ff } },	\
	{ AR5K_RF_BUFFER_CONTROL_4,							\
		{ 0x00000010, 0x00000014, 0x00000010, 0x00000010, 0x00000010 } },	\
}

/* Initial mode-specific settings for AR5212 */
#define AR5K_AR5212_INI_MODE {								\
	{ AR5K_TXCFG,									\
	/*         mode a/XR  mode aTurbo   mode b   mode g (DYN) mode gTurbo */	\
		{ 0x00008107, 0x00008107, 0x00008107, 0x00008107, 0x00008107 } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(0),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(1),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(2),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(3),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(4),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(5),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(6),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(7),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(8),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_QUEUE_DFS_LOCAL_IFS(9),							\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },	\
	{ AR5K_DCU_GBL_IFS_SIFS,							\
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000160, 0x000001e0 } },	\
	{ AR5K_DCU_GBL_IFS_SLOT,							\
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x0000018c, 0x000001e0 } },	\
	{ AR5K_DCU_GBL_IFS_EIFS,							\
		{ 0x00000e60, 0x00001180, 0x00001f1c, 0x00003e38, 0x00001180 } },	\
	{ AR5K_DCU_GBL_IFS_MISC,							\
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000b0e0, 0x00014068 } },	\
	{ AR5K_TIME_OUT,								\
		{ 0x03e803e8, 0x06e006e0, 0x04200420, 0x08400840, 0x06e006e0 } },	\
}

/* Initial mode-specific settings for AR5212 + RF5111 */
#define AR5K_AR5212_RF5111_INI_MODE {							\
	{ AR5K_USEC_5211,								\
	/*         mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
		{ 0x128d8fa7, 0x09880fcf, 0x04e00f95, 0x128d8fab, 0x09880fcf } },	\
	{ AR5K_PHY_TURBO,								\
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 } },	\
	{ 0x9820,									\
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },	\
	{ 0x9824,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9828,									\
		{ 0x0a020001, 0x0a020001, 0x05010100, 0x0a020001, 0x0a020001 } },	\
	{ 0x9834,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9838,									\
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },	\
	{ 0x9844,									\
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 } },	\
	{ 0x9848,									\
		{ 0x0018da5a, 0x0018da5a, 0x0018ca69, 0x0018ca69, 0x0018ca69 } },	\
	{ 0x9850,									\
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 } },	\
	{ AR5K_PHY_SIG,									\
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e } },	\
	{ AR5K_PHY_AGCCOARSE,								\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137615e } },	\
	{ AR5K_PHY_AGCCTL,								\
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 } },	\
	{ AR5K_PHY_NF,									\
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },	\
	{ AR5K_PHY_ADCSAT,								\
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 } },	\
	{ 0x986c,									\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb080, 0x050cb080 } },	\
	{ AR5K_PHY_RX_DELAY,								\
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002af8, 0x00002710 } },	\
	{ 0x9918,									\
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 } },	\
	{ 0x9924,									\
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 } },	\
	{ AR5K_PHY_FRAME_CTL_5211,							\
		{ 0xffb81020, 0xffb81020, 0xffb80d20, 0xffb81020, 0xffb81020 } },	\
	{ AR5K_PHY_PCDAC_TXPOWER(0),							\
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff } },	\
	{ 0xa230,									\
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 } },	\
	{ 0xa208,									\
		{ 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 } },	\
}

/* Initial mode-specific settings for AR5212 + RF5112 */
#define AR5K_AR5212_RF5112_INI_MODE {							\
	{ AR5K_USEC_5211,								\
	/*         mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
		{ 0x128d93a7, 0x098813cf, 0x04e01395, 0x128d93ab, 0x098813cf } },	\
	{ AR5K_PHY_TURBO,								\
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 } },	\
	{ 0x9820,									\
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },	\
	{ 0x9824,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9828,									\
		{ 0x0a020001, 0x0a020001, 0x05020100, 0x0a020001, 0x0a020001 } },	\
	{ 0x9834,									\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9838,									\
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },	\
	{ 0x9844,									\
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 } },	\
	{ 0x9848,									\
		{ 0x0018da6d, 0x0018da6d, 0x0018ca75, 0x0018ca75, 0x0018ca75 } },	\
	{ 0x9850,									\
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 } },	\
	{ AR5K_PHY_SIG,									\
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e } },	\
	{ AR5K_PHY_AGCCOARSE,								\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e } },	\
	{ AR5K_PHY_AGCCTL,								\
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 } },	\
	{ AR5K_PHY_NF,									\
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },	\
	{ AR5K_PHY_ADCSAT,								\
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 } },	\
	{ 0x986c,									\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081 } },	\
	{ AR5K_PHY_RX_DELAY,								\
		{ 0x000007d0, 0x000007d0, 0x0000044c, 0x00000898, 0x000007d0 } },	\
	{ 0x9918,									\
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 } },	\
	{ 0x9924,									\
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 } },	\
	{ AR5K_PHY_FRAME_CTL_5211,							\
		{ 0xffb81020, 0xffb81020, 0xffb80d10, 0xffb81010, 0xffb81010 } },	\
	{ AR5K_PHY_PCDAC_TXPOWER(0),							\
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff } },	\
	{ 0xa230,									\
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 } },	\
	{ AR5K_PHY_CCKTXCTL,								\
		{ 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 } },	\
	{ 0xa208,									\
		{ 0xd6be6788, 0xd6be6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 } },	\
	{ AR5K_PHY_GAIN_2GHZ,								\
		{ 0x642c0140, 0x642c0140, 0x6442c160, 0x6442c160, 0x6442c160 } },	\
}

