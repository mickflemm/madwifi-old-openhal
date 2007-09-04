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
 * (Please have a look at ar5xxx.h for further information)
 */

#include "ah_devid.h"
#include "ath5kreg.h"
#include "ath5k.h"


/* 
 * Known pci ids
 */

static const struct {
	u_int16_t	vendor;
	u_int16_t	device;
	u_int8_t	mac_version;
} ath5k_known_products[] = {
	/*
	 * From pcidevs_data.h
	 */
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5210, 		AR5K_AR5210},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5210_AP, 		AR5K_AR5210},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5210_DEFAULT, 	AR5K_AR5210},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5211,		AR5K_AR5211},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5211_DEFAULT,	AR5K_AR5211},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5311,		AR5K_AR5211},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5211_FPGA11B, 	AR5K_AR5211},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5211_LEGACY, 	AR5K_AR5211},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212,		AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_DEFAULT, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_FPGA, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_IBM, 	AR5K_AR5212},
	{ PCI_VENDOR_3COM,    	PCI_PRODUCT_3COM_3CRDAG675, 		AR5K_AR5212},
	{ PCI_VENDOR_3COM2,   	PCI_PRODUCT_3COM2_3CRPAG175, 		AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_REV2, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_REV7, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_REV8, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0014, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0015, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0016, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0017, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0018, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5212_0019, 	AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR2413, 		AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5413, 		AR5K_AR5212},
	{ PCI_VENDOR_ATHEROS, 	PCI_PRODUCT_ATHEROS_AR5424, 		AR5K_AR5212},
};

/*Rate tables*/
static const AR5K_RATE_TABLE ath5k_rt_11a = AR5K_RATES_11A;
static const AR5K_RATE_TABLE ath5k_rt_11b = AR5K_RATES_11B;
static const AR5K_RATE_TABLE ath5k_rt_11g = AR5K_RATES_11G;
static const AR5K_RATE_TABLE ath5k_rt_turbo = AR5K_RATES_TURBO;
static const AR5K_RATE_TABLE ath5k_rt_xr = AR5K_RATES_XR;

/*
 * Supported channels
 */
static const struct
ieee80211_regchannel ath5k_5ghz_channels[] = IEEE80211_CHANNELS_5GHZ;
static const struct
ieee80211_regchannel ath5k_2ghz_channels[] = IEEE80211_CHANNELS_2GHZ;

/*
 * Initial register dumps
 */

/*
 * MAC/PHY Settings
 */
/* Common for all modes */
static const struct ath5k_ini ar5210_ini[] = AR5K_AR5210_INI;
static const struct ath5k_ini ar5211_ini[] = AR5K_AR5211_INI;
static const struct ath5k_ini ar5212_ini[] = AR5K_AR5212_INI;
/* Mode-specific settings */
static const struct ath5k_ini_mode ar5211_ini_mode[] = AR5K_AR5211_INI_MODE;
static const struct ath5k_ini_mode ar5212_ini_mode[] = AR5K_AR5212_INI_MODE;
static const struct ath5k_ini_mode ar5212_rf5111_ini_mode[] = AR5K_AR5212_RF5111_INI_MODE;
static const struct ath5k_ini_mode ar5212_rf5112_ini_mode[] = AR5K_AR5212_RF5112_INI_MODE;
/* RF Initial BB gain settings */
static const struct ath5k_ini rf5111_ini_bbgain[] = AR5K_RF5111_INI_BBGAIN;
static const struct ath5k_ini rf5112_ini_bbgain[] = AR5K_RF5112_INI_BBGAIN;


/*
 * RF Settings
 */
/* RF Banks */
static const struct ath5k_ini_rf rf5111_rf[] = AR5K_RF5111_INI_RF;
static const struct ath5k_ini_rf rf5112_rf[] = AR5K_RF5112_INI_RF;
static const struct ath5k_ini_rf rf5112a_rf[] = AR5K_RF5112A_INI_RF;
/* Initial mode-specific RF gain table for 5111/5112 */
static const struct ath5k_ini_rfgain rf5111_ini_rfgain[] = AR5K_RF5111_INI_RFGAIN;
static const struct ath5k_ini_rfgain rf5112_ini_rfgain[] = AR5K_RF5112_INI_RFGAIN;
/* Initial gain optimization tables */
static const struct ath5k_gain_opt rf5111_gain_opt = AR5K_RF5111_GAIN_OPT;
static const struct ath5k_gain_opt rf5112_gain_opt = AR5K_RF5112_GAIN_OPT;


/*
 * Enable to overwrite the country code (use "00" for debug)
 */
#if 0
#define COUNTRYCODE "00"
#endif

/*******************\
  General Functions
\*******************/

/*
 * Perform a lookup if the device is supported by the HAL
 * and return the chip name.
 * TODO:Left here for combatibility, change it in ath5k
 */
const char *
ath_hal_probe(u_int16_t vendor, u_int16_t device)
{
	int i;

	/*
	 * Perform a linear search on the table of supported devices
	 */
	for (i = 0; i < AR5K_ELEMENTS(ath5k_known_products); i++) {
		if (vendor == ath5k_known_products[i].vendor &&
		    device == ath5k_known_products[i].device){
			switch (ath5k_known_products[i].mac_version) {
				case AR5K_AR5210:
					return("AR5210");
				case AR5K_AR5211:
					return("AR5211");
				case AR5K_AR5212:
					return("AR5212");
				default:
					return ("");
			}
		}
	}

	return (NULL);
}

/*
 * Calculate transmition time of a frame
 * TODO: Left here for combatibility, change it in ath5k
 */
u_int16_t /*TODO: Is this really hardware dependent ?*/
ath_hal_computetxtime(struct ath_hal *hal, const AR5K_RATE_TABLE *rates,
    u_int32_t frame_length, u_int16_t rate_index, AR5K_BOOL short_preamble)
{
	const AR5K_RATE *rate;
	u_int32_t value;

	AR5K_ASSERT_ENTRY(rate_index, rates->rate_count);

	/*
	 * Get rate by index
	 */
	rate = &rates->rates[rate_index];

	/*
	 * Calculate the transmission time by operation (PHY) mode
	 */
	switch (rate->modulation) {
	case MODULATION_CCK:
		/*
		 * CCK / DS mode (802.11b)
		 */
		value = AR5K_CCK_TX_TIME(rate->rate_kbps, frame_length,
		    (short_preamble && (rate->modulation == MODULATION_CCK_SP)));
		break;

	case MODULATION_OFDM:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 */
		if (AR5K_OFDM_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return (0);
		value = AR5K_OFDM_TX_TIME(rate->rate_kbps, frame_length);
		break;

	case MODULATION_TURBO:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 * Atheros "Turbo Mode" (doubled rates)
		 */
		if (AR5K_TURBO_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return (0);
		value = AR5K_TURBO_TX_TIME(rate->rate_kbps, frame_length);
		break;

	case MODULATION_XR:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 * Atheros "eXtended Range" (XR)
		 */
		if (AR5K_XR_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return (0);
		value = AR5K_XR_TX_TIME(rate->rate_kbps, frame_length);
		break;

	default:
		return (0);
	}

	return (value);
}

/*
 * Return the supported 802.11 operation modes
 * TODO:Left here for combatibility, change it in ath5k
 */
u_int/*TODO:Fix this*/
ath_hal_getwirelessmodes(struct ath_hal *hal, AR5K_CTRY_CODE country) 
{
	switch(hal->ah_version){
	case AR5K_AR5212:
		return (AR5K_MODE_11A|AR5K_MODE_11B|AR5K_MODE_11G);
	case AR5K_AR5211:
		return (AR5K_MODE_11A|AR5K_MODE_11B|AR5K_MODE_11G);
	default :
		return(AR5K_MODE_11A);
	}
}

/*
 * Functions used internaly
 */

static u_int32_t
ath5k_hw_bitswap(u_int32_t val, u_int bits)
{
	u_int32_t retval = 0, bit, i;

	for (i = 0; i < bits; i++) {
		bit = (val >> i) & 1;
		retval = (retval << 1) | bit;
	}

	return (retval);
}

inline u_int
ath5k_hw_htoclock(u_int usec, AR5K_BOOL turbo)
{
	return (turbo == TRUE ? (usec * 80) : (usec * 40));
}

inline u_int
ath5k_hw_clocktoh(u_int clock, AR5K_BOOL turbo)
{
	return (turbo == TRUE ? (clock / 80) : (clock / 40));
}

/*
 * Copy a rate table to a new one
 */
inline void
ath5k_hw_rtcopy(AR5K_RATE_TABLE *dst, const AR5K_RATE_TABLE *src)
{
	memset(dst, 0, sizeof(AR5K_RATE_TABLE));
	dst->rate_count = src->rate_count;
	memcpy(dst->rates, src->rates, sizeof(dst->rates));
}

/*
 * Get the rate table for a specific operation mode
 */
const AR5K_RATE_TABLE *
ath5k_hw_get_rate_table(struct ath_hal *hal, u_int mode)
{

	AR5K_TRACE;

	switch (mode) {
	case AR5K_MODE_11A:
		return (&hal->ah_rt_11a);
	case AR5K_MODE_TURBO:
		return (&hal->ah_rt_turbo);
	case AR5K_MODE_11B:
		return (&hal->ah_rt_11b);
	case AR5K_MODE_11G:
		return (&hal->ah_rt_11g);
	case AR5K_MODE_XR:
		return (&hal->ah_rt_xr);
	default:
		return (NULL);
	}

	return (NULL);
}

/*
 * Read from a device register
 */
static inline u32 ath5k_hw_reg_read(struct ath_hal *hal, u16 reg)
{
	return readl(hal->ah_sh + reg);
}

/*
 * Write to a device register
 */
static inline void ath5k_hw_reg_write(struct ath_hal *hal, u32 val, u16 reg)
{
	writel(val, hal->ah_sh + reg);
}

static inline __u16 ath5k_hw_unaligned_read_16(__le16 *p)
{
	return le16_to_cpu(get_unaligned(p));
}

static inline void ath5k_hw_unaligned_write_16(__u16 v, __le16* p)
{
	put_unaligned(cpu_to_le16(v), p);
}

static inline __u32 ath5k_hw_unaligned_read_32(__le32 *p)
{
	return le32_to_cpu(get_unaligned(p));
}

static inline void ath5k_hw_unaligned_write_32(__u32 v, __le32 *p)
{
	put_unaligned(cpu_to_le32(v), p);
}

/*
 * Check if a register write has been completed
 */
static AR5K_BOOL
ath5k_hw_register_timeout(struct ath_hal *hal, u_int32_t reg, u_int32_t flag,
    u_int32_t val, AR5K_BOOL is_set)
{
	int i;
	u_int32_t data;

	for (i = AR5K_TUNE_REGISTER_TIMEOUT; i > 0; i--) {
		data = AR5K_REG_READ(reg);
		if ((is_set == TRUE) && (data & flag))
			break;
		else if ((data & flag) == val)
			break;
		udelay(15);
	}

	if (i <= 0)
		return (FALSE);

	return (TRUE);
}

/*
 * Write initial register dump
 */
static void
ath5k_hw_ini_registers(struct ath_hal *hal, int size,
		const struct ath5k_ini *ini_regs, AR5K_BOOL change_channel)
{
	int i;

	/* Write initial registers */
	for (i = 0; i < size ; i++) {
		/* On channel change there is 
		 * no need to mess with PCU */
		if (change_channel == TRUE &&
		    ini_regs[i].ini_register >= AR5K_PCU_MIN &&
		    ini_regs[i].ini_register <= AR5K_PCU_MAX)
			continue;

		switch (ini_regs[i].ini_mode) {
		case AR5K_INI_READ:
			/* Cleared on read */
			AR5K_REG_READ(ini_regs[i].ini_register);
			break;
		case AR5K_INI_WRITE:
		default:
			AR5K_REG_WAIT(i);
			AR5K_REG_WRITE(ini_regs[i].ini_register,
					ini_regs[i].ini_value);
		}
	}
}

static void
ath5k_hw_ini_mode_registers(struct ath_hal *hal, int size,
		const struct ath5k_ini_mode *ini_mode, u_int8_t mode)
{
	int i;

	for (i = 0; i < size; i++) {
		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE((u_int32_t)ini_mode[i].mode_register,
		    ini_mode[i].mode_value[mode]);
	}

}

/***************************************\
	Attach/Detach Functions
\***************************************/

/*
 * Check if the device is supported and initialize the needed structs
 */
struct ath_hal *
ath5k_hw_init(u_int16_t device, AR5K_SOFTC sc, AR5K_BUS_TAG st,
		AR5K_BUS_HANDLE sh, AR5K_STATUS *status)
{
	struct ath_hal *hal = NULL;
	u_int8_t mac[ETH_ALEN];
	u_int8_t mac_version = 255; /* Initialize this to something else than ath5k_version */
	int i;
	u_int32_t srev;
	*status = AR5K_EINVAL;

	/*
	 * Check if device is a known one
	 */
	for (i = 0; i < AR5K_ELEMENTS(ath5k_known_products); i++) {
		if (device == ath5k_known_products[i].device)
			mac_version = ath5k_known_products[i].mac_version;
	}

	/* If there wasn't a match, the device is not supported */
	if (mac_version == 255) {
		*status = AR5K_ENOTSUPP;
		AR5K_PRINTF("device not supported: 0x%04x\n", device);
		return (NULL);
	}

	/* If we passed the test malloc a hal struct */
	if ((hal = kmalloc(sizeof(struct ath_hal), GFP_KERNEL)) == NULL) {
		*status = AR5K_ENOMEM;
		AR5K_PRINT("out of memory\n");
		return (NULL);
	}

	/*Initialize it*/
	memset(hal, 0, sizeof(struct ath_hal));

	hal->ah_sc = sc;
	hal->ah_st = st;
	hal->ah_sh = sh;
	hal->ah_device = device;
	hal->ah_sub_vendor = 0; /* XXX unknown?! */

	/*
	 * HAL information
	 */

	/* Regulation Stuff */
	hal->ah_country_code = AR5K_TUNE_CTRY;
	ath5k_get_regdomain(hal);

	hal->ah_op_mode = AR5K_M_STA;
	hal->ah_radar.r_enabled = AR5K_TUNE_RADAR_ALERT;
	hal->ah_turbo = FALSE;
	hal->ah_txpower.txp_tpc = AR5K_TUNE_TPC_TXPOWER;
	hal->ah_imr = 0;
	hal->ah_atim_window = 0;
	hal->ah_aifs = AR5K_TUNE_AIFS;
	hal->ah_cw_min = AR5K_TUNE_CWMIN;
	hal->ah_limit_tx_retries = AR5K_INIT_TX_RETRY;
	hal->ah_software_retry = FALSE;
	hal->ah_ant_diversity = AR5K_TUNE_ANT_DIVERSITY;

	/*
	 * Set the mac revision based on the pci id
	 */
	hal->ah_version	= mac_version;

	if (hal->ah_version == AR5K_AR5212)
		hal->ah_magic = AR5K_EEPROM_MAGIC_5212;
	else if (hal->ah_version == AR5K_AR5211)
		hal->ah_magic = AR5K_EEPROM_MAGIC_5211;

	/* Get MAC revision */
	srev = AR5K_REG_READ(AR5K_SREV);
	hal->ah_mac_srev = srev;
	hal->ah_mac_version = AR5K_REG_MS(srev, AR5K_SREV_VER);
	hal->ah_mac_revision = AR5K_REG_MS(srev, AR5K_SREV_REV);

	/* Return on unsupported devices */
	if((srev >= AR5K_SREV_VER_AR5416)){
		printk(KERN_ERR "ath_hal: Device not supported (0x%x)\n", srev);
		*status = AR5K_ENOTSUPP;
		goto failed;
	}

	switch (srev) {
		case AR5K_SREV_VER_AR2424:
		case AR5K_SREV_VER_AR5424:
		case AR5K_SREV_VER_AR5413:
		case AR5K_SREV_VER_AR5414:
			/*
			 * Known single chip solutions
			 */
			hal->ah_single_chip = TRUE;
			break;
		default:
			/*
			 * Multi chip solutions
			 */
			hal->ah_single_chip = FALSE;
			break;
	}

	/* Bring device out of sleep and reset it's units */
	if (ath5k_hw_nic_wakeup(hal, AR5K_INIT_MODE, TRUE) != TRUE)
		goto failed;

	/* Get PHY and RADIO revisions */
	hal->ah_phy_revision = 
		AR5K_REG_READ(AR5K_PHY_CHIP_ID) & 0x00ffffffff;
	hal->ah_radio_5ghz_revision =
		ath5k_hw_radio_revision(hal, AR5K_CHIP_5GHZ);

	if (hal->ah_version == AR5K_AR5210) {
		hal->ah_radio_2ghz_revision = 0;
	} else {
		hal->ah_radio_2ghz_revision = 
			ath5k_hw_radio_revision(hal, AR5K_CHIP_2GHZ);
	}

	/* Single chip radio */
	if (hal->ah_radio_2ghz_revision == hal->ah_radio_5ghz_revision)
		hal->ah_radio_2ghz_revision = 0;

	/* Identify the radio chip*/
	if (hal->ah_version == AR5K_AR5210)
		hal->ah_radio = AR5K_RF5110;
	else
		hal->ah_radio = hal->ah_radio_5ghz_revision < AR5K_SREV_RAD_5112 ?
							AR5K_RF5111 : AR5K_RF5112;

	hal->ah_phy = AR5K_PHY(0);

	/* Set MAC to bcast: ff:ff:ff:ff:ff:ff, this is using 'mac' as a 
 	 * temporary variable for setting our BSSID. Right bellow we update 
 	 * it with ath5k_hw_get_lladdr() */
	memset(mac, 0xff, ETH_ALEN);
	ath5k_hw_set_associd(hal, mac, 0);

	ath5k_hw_get_lladdr(hal, mac);
	ath5k_hw_set_opmode(hal);
		
#ifdef AR5K_DEBUG
	ath5k_hw_dump_state(hal);
#endif

	/*
	 * Get card capabilities, values, ...
	 */

	if (ath5k_hw_eeprom_init(hal) != 0) {
		*status = AR5K_EELOCKED;
		AR5K_PRINT("unable to init EEPROM\n");
		goto failed;
	}

	/* Get misc capabilities */
	if (ath5k_hw_get_capabilities(hal) != TRUE) {
		*status = AR5K_EEREAD;
		AR5K_PRINTF("unable to get device capabilities: 0x%04x\n",
		    device);
		goto failed;
	}

	/* Get MAC address */
	if ((*status = ath5k_hw_eeprom_read_mac(hal, mac)) != 0) {
		*status = AR5K_EEBADMAC;
		AR5K_PRINTF("unable to read address from EEPROM: 0x%04x\n",
		    device);
		goto failed;
	}

	ath5k_hw_set_lladdr(hal, mac);

	/* Get rate tables */
	if (hal->ah_capabilities.cap_mode & AR5K_MODE_11A)
		ath5k_hw_rtcopy(&hal->ah_rt_11a, &ath5k_rt_11a);
	if (hal->ah_capabilities.cap_mode & AR5K_MODE_11B)
		ath5k_hw_rtcopy(&hal->ah_rt_11b, &ath5k_rt_11b);
	if (hal->ah_capabilities.cap_mode & AR5K_MODE_11G)
		ath5k_hw_rtcopy(&hal->ah_rt_11g, &ath5k_rt_11g);
	if (hal->ah_capabilities.cap_mode & AR5K_MODE_TURBO)
		ath5k_hw_rtcopy(&hal->ah_rt_turbo, &ath5k_rt_turbo);
	if (hal->ah_capabilities.cap_mode & AR5K_MODE_XR)
		ath5k_hw_rtcopy(&hal->ah_rt_xr, &ath5k_rt_xr);

	/* Initialize the gain optimization values */
	/*For RF5111*/
	if (hal->ah_radio == AR5K_RF5111) {
		hal->ah_gain.g_step_idx = rf5111_gain_opt.go_default;
		hal->ah_gain.g_step =
		    &rf5111_gain_opt.go_step[hal->ah_gain.g_step_idx];
		hal->ah_gain.g_low = 20;
		hal->ah_gain.g_high = 35;
		hal->ah_gain.g_active = 1;
	/*For RF5112*/
	} else if (hal->ah_radio == AR5K_RF5112) {
		hal->ah_gain.g_step_idx = rf5112_gain_opt.go_default;
		hal->ah_gain.g_step =
		    &rf5111_gain_opt.go_step[hal->ah_gain.g_step_idx];
		hal->ah_gain.g_low = 20;
		hal->ah_gain.g_high = 85;
		hal->ah_gain.g_active = 1;
	}

	*status = AR5K_OK;

	printk(KERN_INFO "ath_hal: MAC revision: %s (0x%x)\n",
		ath5k_hw_get_part_name(AR5K_VERSION_VER,hal->ah_mac_srev),
					hal->ah_mac_srev);
	if((AR5K_MODE_11B & hal->ah_capabilities.cap_mode) &&
	(AR5K_MODE_11A & hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath_hal: PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_5ghz_revision),
						hal->ah_radio_5ghz_revision);
	}
	if((AR5K_MODE_11B & hal->ah_capabilities.cap_mode) &&
	!(AR5K_MODE_11A & hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath_hal: 2Ghz PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_2ghz_revision),
						hal->ah_radio_2ghz_revision);
	}
	if(!(AR5K_MODE_11B & hal->ah_capabilities.cap_mode) &&
	(AR5K_MODE_11A & hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath_hal: 5Ghz PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_5ghz_revision),
						hal->ah_radio_5ghz_revision);
	}
	printk(KERN_INFO "ath_hal: EEPROM version: %x.%x\n",
		(hal->ah_ee_version & 0xF000) >> 12, hal->ah_ee_version & 0xFFF);

	return (hal);

 failed:
	kfree(hal);
	return (NULL);
}

/*
 * Bring up MAC + PHY Chips
 */
AR5K_BOOL
ath5k_hw_nic_wakeup(struct ath_hal *hal, u_int16_t flags, AR5K_BOOL initial)
{
	u_int32_t turbo, mode, clock;

	turbo = 0;
	mode = 0;
	clock = 0;

	AR5K_TRACE;

	if (hal->ah_version != AR5K_AR5210) {
		/*
		 * Get channel mode flags
		 */
 
		if (hal->ah_radio >= AR5K_RF5112) {
			mode = AR5K_PHY_MODE_RAD_RF5112;
			clock = AR5K_PHY_PLL_RF5112;
		} else {
			mode = AR5K_PHY_MODE_RAD_RF5111;	/*Zero*/
			clock = AR5K_PHY_PLL_RF5111;		/*Zero*/
		}

		if (flags & CHANNEL_2GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_2GHZ;
			clock |= AR5K_PHY_PLL_44MHZ;

			if (flags & CHANNEL_CCK) {
				mode |= AR5K_PHY_MODE_MOD_CCK;
			} else if (flags & CHANNEL_OFDM) {
				/* XXX: Dynamic OFDM/CCK is not supported by the AR5211
				 * so we set MOD_OFDM for plain g (no CCK headers)
				 * operation. We need to test this, 5211 might
				 * support ofdm-only g after all, there are also
				 * initial register values in the code for g 
				 * mode (see ath5k_hw.h). */
				if (hal->ah_version == AR5K_AR5211) {
					mode |= AR5K_PHY_MODE_MOD_OFDM;
				} else {
					mode |= AR5K_PHY_MODE_MOD_DYN;
				}
			} else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return (FALSE);
			}
		} else if (flags & CHANNEL_5GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_5GHZ;
			clock |= AR5K_PHY_PLL_40MHZ;
			if (flags & CHANNEL_OFDM) {
				mode |= AR5K_PHY_MODE_MOD_OFDM;
			} else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return (FALSE);
			}
		} else {
			AR5K_PRINT("invalid radio frequency mode\n");
			return (FALSE);
		}

		if (flags & CHANNEL_TURBO) {
			turbo = AR5K_PHY_TURBO_MODE |
			    AR5K_PHY_TURBO_SHORT;
		}
	}

	/*
	 * Reset and wakeup the device
	 */

	else {
		if (initial == TRUE) {
			/* ...reset hardware */
			if (ath5k_hw_nic_reset(hal,
				AR5K_RESET_CTL_PCI) == FALSE) {
				AR5K_PRINT("failed to reset the PCI chipset\n");
				return (FALSE);
			}

			udelay(1000);
		}

		/* ...wakeup */
		if (ath5k_hw_set_power(hal,
			AR5K_PM_AWAKE, TRUE, 0) == FALSE) {
			AR5K_PRINT("failed to resume the MAC Chip\n");
			return (FALSE);
		}

		/* ...enable Atheros turbo mode if requested */
		if (flags & CHANNEL_TURBO)
			AR5K_REG_WRITE(AR5K_PHY_TURBO, AR5K_PHY_TURBO_MODE);

		/* ...reset chipset */
		if (ath5k_hw_nic_reset(hal, AR5K_RESET_CTL_CHIP) == FALSE) {
			AR5K_PRINT("failed to reset the AR5210 chipset\n");
			return (FALSE);
		}

		udelay(1000);
	}

	/* ...reset chipset and PCI device */
	if (hal->ah_single_chip == FALSE &&
	ath5k_hw_nic_reset(hal,AR5K_RESET_CTL_CHIP | AR5K_RESET_CTL_PCI) == FALSE) {
		AR5K_PRINT("failed to reset the MAC Chip + PCI\n");
		return (FALSE);
	}

	if (hal->ah_version == AR5K_AR5210)
		udelay(2300);

	/* ...wakeup */
	if (ath5k_hw_set_power(hal,
		AR5K_PM_AWAKE, TRUE, 0) == FALSE) {
		AR5K_PRINT("failed to resume the MAC Chip\n");
		return (FALSE);
	}

	/* ...final warm reset */
	if (ath5k_hw_nic_reset(hal, 0) == FALSE) {
		AR5K_PRINT("failed to warm reset the MAC Chip\n");
		return (FALSE);
	}

	if (hal->ah_version != AR5K_AR5210){
		/* ...set the PHY operating mode */
		AR5K_REG_WRITE(AR5K_PHY_PLL, clock);
		udelay(300);

		AR5K_REG_WRITE(AR5K_PHY_MODE, mode);
		AR5K_REG_WRITE(AR5K_PHY_TURBO, turbo);
	}

	return (TRUE);
}

/*
 * Get the PHY Chip revision
 */
u_int16_t
ath5k_hw_radio_revision(struct ath_hal *hal, AR5K_CHIP chip)
{
	int i;
	u_int32_t srev;
	u_int16_t ret;

	AR5K_TRACE;

	/*
	 * Set the radio chip access register
	 */
	switch (chip) {
	case AR5K_CHIP_2GHZ:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_2GHZ);
		break;
	case AR5K_CHIP_5GHZ:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);
		break;
	default:
		return (0);
	}

	udelay(2000);

	/* ...wait until PHY is ready and read the selected radio revision */
	AR5K_REG_WRITE(AR5K_PHY(0x34), 0x00001c16);

	for (i = 0; i < 8; i++)
		AR5K_REG_WRITE(AR5K_PHY(0x20), 0x00010000);

	if (hal->ah_version == AR5K_AR5210) {
		srev = AR5K_REG_READ(AR5K_PHY(256) >> 28) & 0xf;

		ret = (u_int16_t) ath5k_hw_bitswap(srev, 4) + 1;
	} else {
		srev = (AR5K_REG_READ(AR5K_PHY(0x100)) >> 24) & 0xff;

		ret = (u_int16_t) ath5k_hw_bitswap(((srev & 0xf0) >> 4) | ((srev & 0x0f) << 4), 8);
	}

	/* Reset to the 5GHz mode */
	AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);

	return (ret);
}

/*
 * Free the hal struct
 */
void
ath5k_hw_detach(struct ath_hal *hal)
{
	AR5K_TRACE;

	if (hal->ah_rf_banks != NULL)
		kfree(hal->ah_rf_banks);

	/*
	 * Free HAL structure, assume interrupts are down
	 */
	kfree(hal);
}




/*******************************\
	Reset Functions
\*******************************/

/*
 * Main reset function
 */
AR5K_BOOL
ath5k_hw_reset(struct ath_hal *hal, AR5K_OPMODE op_mode, AR5K_CHANNEL *channel,
    AR5K_BOOL change_channel, AR5K_STATUS *status)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int8_t mac[ETH_ALEN];
	u_int32_t data, noise_floor, s_seq, s_ant, s_led[3];
	u_int i, mode, freq, ee_mode, ant[2];
	const AR5K_RATE_TABLE *rt;

	AR5K_TRACE;

	*status = AR5K_OK;
	s_seq = 0;
	s_ant = 1;
	ee_mode = 0;
	freq = 0;
	mode = 0;

	/*
	 * Save some registers before a reset
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		if (change_channel == TRUE) {
			/*Sequence number for queue 0 -do this for all queues ?*/
			s_seq = AR5K_REG_READ(AR5K_QUEUE_DFS_SEQNUM(0));
			/*Default antenna*/
			s_ant = AR5K_REG_READ(AR5K_DEFAULT_ANTENNA);
		}
	}

	/*GPIOs*/
	s_led[0] = AR5K_REG_READ(AR5K_PCICFG) &	AR5K_PCICFG_LEDSTATE;
	s_led[1] = AR5K_REG_READ(AR5K_GPIOCR);
	s_led[2] = AR5K_REG_READ(AR5K_GPIODO);

	if (change_channel == TRUE && hal->ah_rf_banks != NULL)
		ath5k_hw_get_rf_gain(hal);


	/*Wakeup the device*/
	if (ath5k_hw_nic_wakeup(hal, channel->channel_flags, FALSE) == FALSE) {
		*status = AR5K_EIO;
		return (FALSE);
	}

	/*
	 * Initialize operating mode
	 */
	hal->ah_op_mode = op_mode;

	/*
	 * 5111/5112 Settings
	 * 5210 only comes with RF5110
	 */
	if (hal->ah_version != AR5K_AR5210) {
		if ((hal->ah_radio != AR5K_RF5111) &&
		(hal->ah_radio != AR5K_RF5112)) {
			AR5K_PRINTF("invalid phy radio: %u\n", hal->ah_radio);
			*status = AR5K_EINVAL;
			return (FALSE);
		}

		switch (channel->channel_flags & CHANNEL_MODES) {
		case CHANNEL_A:
			mode = AR5K_INI_VAL_11A;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		case CHANNEL_B:
			mode = AR5K_INI_VAL_11B;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11B;
			break;
		/*Is this ok on 5211 too ?*/
		case CHANNEL_G:
			mode = AR5K_INI_VAL_11G;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			break;
		case CHANNEL_T:
			mode = AR5K_INI_VAL_11A_TURBO;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		/*Is this ok on 5211 too ?*/
		case CHANNEL_TG:
			mode = AR5K_INI_VAL_11G_TURBO;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			break;
		case CHANNEL_XR:
			if (hal->ah_version == AR5K_AR5211) {
				AR5K_PRINTF("XR mode not available on 5211");
				return (FALSE);
			}
			mode = AR5K_INI_VAL_XR;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		default:
			AR5K_PRINTF("invalid channel: %d\n", channel->freq);
			*status = AR5K_EINVAL;
			return (FALSE);
		}

		/* PHY access enable */
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);

	}

	/*
	 * Write initial mode-specific settings
	 */
	/*For 5212*/
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_ini_mode_registers(hal, AR5K_ELEMENTS(ar5212_ini_mode),
						ar5212_ini_mode, mode);
		if (hal->ah_radio == AR5K_RF5111) {
			ath5k_hw_ini_mode_registers(hal, AR5K_ELEMENTS(ar5212_rf5111_ini_mode),
							ar5212_rf5111_ini_mode, mode);
		} else if (hal->ah_radio == AR5K_RF5112) {
			ath5k_hw_ini_mode_registers(hal, AR5K_ELEMENTS(ar5212_rf5112_ini_mode),
							ar5212_rf5112_ini_mode, mode);
		}
	}
	/*For 5211*/
	if (hal->ah_version == AR5K_AR5211) {
		ath5k_hw_ini_mode_registers(hal, AR5K_ELEMENTS(ar5211_ini_mode),
						ar5211_ini_mode, mode);
	}
	/* For 5210 mode settings check out ath5k_hw_reset_tx_queue */

	/*
	 * Write initial settings common for all modes
	 */
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(ar5212_ini),
					ar5212_ini, change_channel);
		if (hal->ah_radio == AR5K_RF5112) {
			AR5K_REG_WRITE(AR5K_PHY_PAPD_PROBE,
					AR5K_PHY_PAPD_PROBE_INI_5112);
			ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(rf5112_ini_bbgain),
						rf5112_ini_bbgain, change_channel);
		} else if (hal->ah_radio == AR5K_RF5111) {
			AR5K_REG_WRITE( AR5K_PHY_GAIN_2GHZ,
					AR5K_PHY_GAIN_2GHZ_INI_5111); 
			AR5K_REG_WRITE( AR5K_PHY_PAPD_PROBE,
					AR5K_PHY_PAPD_PROBE_INI_5111 );
			ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(rf5111_ini_bbgain),
						rf5111_ini_bbgain, change_channel);
		}
	} else if (hal->ah_version == AR5K_AR5211) {
		ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(ar5211_ini),
					ar5211_ini, change_channel);
		/* AR5211 only comes with 5111 */
		ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(rf5111_ini_bbgain),
					rf5111_ini_bbgain, change_channel);
	} else if (hal->ah_version == AR5K_AR5210) {
		ath5k_hw_ini_registers(hal, AR5K_ELEMENTS(ar5210_ini),
					ar5210_ini, change_channel);
	}


	/*
	 * 5211/5212 Specific
	 */
	if (hal->ah_version != AR5K_AR5210) {
		/*
		 * Write initial RF gain settings
		 * This should work for both 5111/5112
		 */
		if (ath5k_hw_rfgain(hal, freq) == FALSE) {
			*status = AR5K_EIO;
			return (FALSE);
		}

		udelay(1000);

		/*
		 * Set rate duration table on 5212
		 */
		if (hal->ah_version == AR5K_AR5212) {

			/*For 802.11b*/
			if (!(channel->channel_flags & CHANNEL_B)) {

				/*Get rate table for this operation mode*/
				rt = ath5k_hw_get_rate_table(hal, AR5K_MODE_11B);

				/*Write rate duration table*/
				for (i = 0; i < rt->rate_count; i++) {
					data = AR5K_RATE_DUR(rt->rates[i].rate_code);
					AR5K_REG_WRITE(data,
					    ath_hal_computetxtime(hal, rt, 14,
					    rt->rates[i].control_rate, FALSE));
					if (HAS_SHPREAMBLE(i)) {
						AR5K_REG_WRITE(data +
						    (AR5K_SET_SHORT_PREAMBLE << 2),
						    ath_hal_computetxtime(hal, rt, 14,
						    rt->rates[i].control_rate, FALSE));
					}
				}
	
			} else {
			/*For 802.11a/g Turbo/XR mode (AR5K_MODE_XR here is O.K. for both a/g - OFDM)*/

				/*Get rate table for this operation mode*/
				rt = ath5k_hw_get_rate_table(hal,
				    channel->channel_flags & CHANNEL_TURBO ?
				    AR5K_MODE_TURBO : AR5K_MODE_XR);

				/*Write rate duration table*/
				for (i = 0; i < rt->rate_count; i++) {
					AR5K_REG_WRITE(AR5K_RATE_DUR(rt->rates[i].rate_code),
					    ath_hal_computetxtime(hal, rt, 14,
					    rt->rates[i].control_rate, FALSE));
				}

			}
		}

		/* Fix for first revision of the RF5112 RF chipset */
		if (hal->ah_radio >= AR5K_RF5112 &&
			hal->ah_radio_5ghz_revision < AR5K_SREV_RAD_5112A) {
				AR5K_REG_WRITE(AR5K_PHY_CCKTXCTL,
				AR5K_PHY_CCKTXCTL_WORLD);
			if (channel->channel_flags & CHANNEL_A)
				data = 0xffb81020;
			else
				data = 0xffb80d20;
			AR5K_REG_WRITE(AR5K_PHY_FRAME_CTL, data);
		}

		/*
		 * Set TX power (XXX use txpower from net80211)
		 */
		if (ath5k_hw_txpower(hal, channel,
			AR5K_TUNE_DEFAULT_TXPOWER) == FALSE) {
			*status = AR5K_EIO;
			return (FALSE);
		}

		/*
		 * Write RF registers
		 * TODO:Does this work on 5211 (5111) ?
		 */
		if (ath5k_hw_rfregs(hal, channel, mode) == FALSE) {
			*status = AR5K_EINPROGRESS;
			return (FALSE);
		}

		/*
		 * Configure additional registers
		 */

		/* Write OFDM timings on 5212*/
		if (hal->ah_version == AR5K_AR5212) {
			if (channel->channel_flags & CHANNEL_OFDM) {
				u_int32_t coef_scaled, coef_exp, coef_man, ds_coef_exp,
				    ds_coef_man, clock;

				clock = channel->channel_flags & CHANNEL_T ? 80 : 40;
				coef_scaled = ((5 * (clock << 24)) / 2) / channel->freq;

				for (coef_exp = 31; coef_exp > 0; coef_exp--)
					if ((coef_scaled >> coef_exp) & 0x1)
						break;

				if (!coef_exp) {
					*status = AR5K_EINVAL;
					return (FALSE);
				}

				coef_exp = 14 - (coef_exp - 24);
				coef_man = coef_scaled + (1 << (24 - coef_exp - 1));
				ds_coef_man = coef_man >> (24 - coef_exp);
				ds_coef_exp = coef_exp - 16;

				AR5K_REG_WRITE_BITS(AR5K_PHY_TIMING_3,
				    AR5K_PHY_TIMING_3_DSC_MAN, ds_coef_man);
				AR5K_REG_WRITE_BITS(AR5K_PHY_TIMING_3,
				    AR5K_PHY_TIMING_3_DSC_EXP, ds_coef_exp);
			}
		}

		/* Enable/disable 802.11b mode on 5111
		(enable 2111 frequency converter + CCK) */
		if (hal->ah_radio == AR5K_RF5111) {
			if (channel->channel_flags & CHANNEL_B)
				AR5K_REG_ENABLE_BITS(AR5K_TXCFG,
				    AR5K_TXCFG_B_MODE);
			else
				AR5K_REG_DISABLE_BITS(AR5K_TXCFG,
				    AR5K_TXCFG_B_MODE);
		}

		/* Set antenna mode */
		AR5K_REG_MASKED_BITS(AR5K_PHY(0x44),
		    hal->ah_antenna[ee_mode][0], 0xfffffc06);

			if (freq == AR5K_INI_RFGAIN_2GHZ)
				ant[0] = ant[1] = AR5K_ANT_FIXED_B;
			else
				ant[0] = ant[1] = AR5K_ANT_FIXED_A;


		AR5K_REG_WRITE(AR5K_PHY_ANT_SWITCH_TABLE_0,
		    hal->ah_antenna[ee_mode][ant[0]]);
		AR5K_REG_WRITE(AR5K_PHY_ANT_SWITCH_TABLE_1,
		    hal->ah_antenna[ee_mode][ant[1]]);

		/* Commit values from EEPROM */
		if (hal->ah_radio == AR5K_RF5111)
			AR5K_REG_WRITE_BITS(AR5K_PHY_FRAME_CTL,
			    AR5K_PHY_FRAME_CTL_TX_CLIP, ee->ee_tx_clip);

		AR5K_REG_WRITE(AR5K_PHY(0x5a),
		    AR5K_PHY_NF_SVAL(ee->ee_noise_floor_thr[ee_mode]));

		AR5K_REG_MASKED_BITS(AR5K_PHY(0x11),
		    (ee->ee_switch_settling[ee_mode] << 7) & 0x3f80, 0xffffc07f);
		AR5K_REG_MASKED_BITS(AR5K_PHY(0x12),
		    (ee->ee_ant_tx_rx[ee_mode] << 12) & 0x3f000, 0xfffc0fff);
		AR5K_REG_MASKED_BITS(AR5K_PHY(0x14),
		    (ee->ee_adc_desired_size[ee_mode] & 0x00ff) |
		    ((ee->ee_pga_desired_size[ee_mode] << 8) & 0xff00), 0xffff0000);

		AR5K_REG_WRITE(AR5K_PHY(0x0d),
		    (ee->ee_tx_end2xpa_disable[ee_mode] << 24) |
		    (ee->ee_tx_end2xpa_disable[ee_mode] << 16) |
		    (ee->ee_tx_frm2xpa_enable[ee_mode] << 8) |
		    (ee->ee_tx_frm2xpa_enable[ee_mode]));

		AR5K_REG_MASKED_BITS(AR5K_PHY(0x0a),
		    ee->ee_tx_end2xlna_enable[ee_mode] << 8, 0xffff00ff);
		AR5K_REG_MASKED_BITS(AR5K_PHY(0x19),
		    (ee->ee_thr_62[ee_mode] << 12) & 0x7f000, 0xfff80fff);
		AR5K_REG_MASKED_BITS(AR5K_PHY(0x49), 4, 0xffffff01);

		AR5K_REG_ENABLE_BITS(AR5K_PHY_IQ,
		    AR5K_PHY_IQ_CORR_ENABLE |
		    (ee->ee_i_cal[ee_mode] << AR5K_PHY_IQ_CORR_Q_I_COFF_S) |
		    ee->ee_q_cal[ee_mode]);

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
			AR5K_REG_WRITE_BITS(AR5K_PHY_GAIN_2GHZ,
			    AR5K_PHY_GAIN_2GHZ_MARGIN_TXRX,
			    ee->ee_margin_tx_rx[ee_mode]);
		}

	} else {
		udelay(1000);
		/* Disable phy and wait */
		AR5K_REG_WRITE(AR5K_PHY_ACT, AR5K_PHY_ACT_DISABLE);
		udelay(1000);
	}

	/*
	 * Restore saved values
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		AR5K_REG_WRITE(AR5K_QUEUE_DFS_SEQNUM(0), s_seq);
		AR5K_REG_WRITE(AR5K_DEFAULT_ANTENNA, s_ant);
	}
	AR5K_REG_ENABLE_BITS(AR5K_PCICFG, s_led[0]);
	AR5K_REG_WRITE(AR5K_GPIOCR, s_led[1]);
	AR5K_REG_WRITE(AR5K_GPIODO, s_led[2]);

	/*
	 * Misc
	 */
	memset(mac, 0xff, ETH_ALEN);
	ath5k_hw_set_associd(hal, mac, 0);
	ath5k_hw_set_opmode(hal);
	/*PISR/SISR Not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		AR5K_REG_WRITE(AR5K_PISR, 0xffffffff);
		/* XXX: AR5K_RSSI_THR has masks and shifts defined for it, so 
 		 * direct write using AR5K_REG_WRITE seems wrong. Test with:
		 * AR5K_REG_WRITE_BITS(AR5K_RSSI_THR,
		 *   AR5K_RSSI_THR_BMISS, AR5K_TUNE_RSSI_THRES);
		 * with different variables and check results compared
		 * to AR5K_REG_WRITE()  */
		AR5K_REG_WRITE(AR5K_RSSI_THR, AR5K_TUNE_RSSI_THRES);
	}

	/*
	 * Set Rx/Tx DMA Configuration
	 *(passing dma size not available on 5210)
	 */
	if (hal->ah_version != AR5K_AR5210) {
		AR5K_REG_WRITE_BITS(AR5K_TXCFG, AR5K_TXCFG_SDMAMR,
				AR5K_DMASIZE_512B | AR5K_TXCFG_DMASIZE);
		AR5K_REG_WRITE_BITS(AR5K_RXCFG, AR5K_RXCFG_SDMAMW,
				AR5K_DMASIZE_512B);
	}

	/*
	 * Set channel and calibrate the PHY
	 */
	if (ath5k_hw_channel(hal, channel) == FALSE) {
		*status = AR5K_EIO;
		return (FALSE);
	}

	/*
	 * Enable the PHY and wait until completion
	 */
	AR5K_REG_WRITE(AR5K_PHY_ACT, AR5K_PHY_ACT_ENABLE);

	/*
	 * 5111/5112 Specific
	 */
	if (hal->ah_version != AR5K_AR5210) {
		data = AR5K_REG_READ(AR5K_PHY_RX_DELAY) & AR5K_PHY_RX_DELAY_M;
		data = (channel->channel_flags & CHANNEL_CCK) ?
		    ((data << 2) / 22) : (data / 10);

		udelay(100 + data);
	} else {
		udelay(1000);
	}

	/*
	 * Enable calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_CAL);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
		AR5K_PHY_AGCCTL_CAL, 0, FALSE) == FALSE) {
		AR5K_PRINTF("calibration timeout (%uMHz)\n",
		    channel->freq);
		return (FALSE);
	}

	/*
 	 * Enable noise floor calibration and wait until completion
 	 */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_NF);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
		AR5K_PHY_AGCCTL_NF, 0, FALSE) == FALSE) {
		AR5K_PRINTF("noise floor calibration timeout (%uMHz)\n",
				channel->freq);
		return (FALSE);
	}

	/* Wait until the noise floor is calibrated and read the value */
	for (i = 20; i > 0; i--) {
		udelay(1000);
		noise_floor = AR5K_REG_READ(AR5K_PHY_NF);

		if (AR5K_PHY_NF_RVAL(noise_floor) &
		AR5K_PHY_NF_ACTIVE)
			noise_floor = AR5K_PHY_NF_AVAL(noise_floor);
	
		if (noise_floor <= AR5K_TUNE_NOISE_FLOOR)
			break;
	}

	if (noise_floor > AR5K_TUNE_NOISE_FLOOR) {
		AR5K_PRINTF("noise floor calibration failed (%uMHz)\n",
			channel->freq);
		return (FALSE);
	}
	
	hal->ah_calibration = FALSE;

	if (!(channel->channel_flags & CHANNEL_B)) {
		hal->ah_calibration = TRUE;
		AR5K_REG_WRITE_BITS(AR5K_PHY_IQ,
		    AR5K_PHY_IQ_CAL_NUM_LOG_MAX, 15);
		AR5K_REG_ENABLE_BITS(AR5K_PHY_IQ,
		    AR5K_PHY_IQ_RUN);
	}

	/*
	 * Reset queues and start beacon timers at the end of the reset routine
	 */
	for (i = 0; i < hal->ah_capabilities.cap_queues.q_tx_num; i++) {
		/*No QCU on 5210*/
		if (hal->ah_version != AR5K_AR5210)
			AR5K_REG_WRITE_Q(AR5K_QUEUE_QCUMASK(i), i);

		if (ath5k_hw_reset_tx_queue(hal, i) == FALSE) {
			AR5K_PRINTF("failed to reset TX queue #%d\n", i);
			*status = AR5K_EINVAL;
			return (FALSE);
		}
	}

	/* Pre-enable interrupts on 5211/5212*/
	if (hal->ah_version != AR5K_AR5210) {
		ath5k_hw_set_intr(hal, AR5K_INT_RX | AR5K_INT_TX | AR5K_INT_FATAL);
	}

	/*
	 * Set RF kill flags if supported by the device (read from the EEPROM)
	 * Disable gpio_intr for now since it results system hang.
	 * TODO: Handle this in ath_intr
	 */
#if 0
	if (AR5K_EEPROM_HDR_RFKILL(hal->ah_capabilities.cap_eeprom.ee_header)) {
		ath5k_hw_set_gpio_input(hal, 0);
		if ((hal->ah_gpio[0] = ath5k_hw_get_gpio(hal, 0)) == 0)
			ath5k_hw_set_gpio_intr(hal, 0, 1);
		else
			ath5k_hw_set_gpio_intr(hal, 0, 0);
	}
#endif

	/*
	 * Set the 32MHz reference clock on 5212 phy clock sleep register
	 */
	if (hal->ah_version == AR5K_AR5212) {
		AR5K_REG_WRITE(AR5K_PHY_SCR, AR5K_PHY_SCR_32MHZ);
		AR5K_REG_WRITE(AR5K_PHY_SLMT, AR5K_PHY_SLMT_32MHZ);
		AR5K_REG_WRITE(AR5K_PHY_SCAL, AR5K_PHY_SCAL_32MHZ);
		AR5K_REG_WRITE(AR5K_PHY_SCLOCK, AR5K_PHY_SCLOCK_32MHZ);
		AR5K_REG_WRITE(AR5K_PHY_SDELAY, AR5K_PHY_SDELAY_32MHZ);
		AR5K_REG_WRITE(AR5K_PHY_SPENDING, hal->ah_radio == AR5K_RF5111 ?
				AR5K_PHY_SPENDING_RF5111 : AR5K_PHY_SPENDING_RF5112);
	}

	/* 
	 * Disable beacons and reset the register
	 */
	AR5K_REG_DISABLE_BITS(AR5K_BEACON,
	    AR5K_BEACON_ENABLE | AR5K_BEACON_RESET_TSF);

	return (TRUE);
}

/*
 * Reset chipset
 */
AR5K_BOOL
ath5k_hw_nic_reset(struct ath_hal *hal, u_int32_t val)
{
	AR5K_BOOL ret = FALSE;
	u_int32_t mask = val ? val : ~0;

	AR5K_TRACE;

	/* Read-and-clear RX Descriptor Pointer*/
	AR5K_REG_READ(AR5K_RXDP);

	/*
	 * Reset the device and wait until success
	 */
	AR5K_REG_WRITE(AR5K_RESET_CTL, val);

	/* Wait at least 128 PCI clocks */
	udelay(15);

	if (hal->ah_version == AR5K_AR5210) {
		val &= AR5K_RESET_CTL_CHIP;
		mask &= AR5K_RESET_CTL_CHIP;
	} else {
		val &=
		    AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;

		mask &=
		    AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;
	}

	ret = ath5k_hw_register_timeout(hal, AR5K_RESET_CTL, mask, val, FALSE);

	/*
	 * Reset configuration register (for hw byte-swap)
	 */
	if ((val & AR5K_RESET_CTL_PCU) == 0)
		AR5K_REG_WRITE(AR5K_CFG, AR5K_INIT_CFG);

	return (ret);
}

/*
 * Power management functions
 */

/*
 * Sleep control
 */
AR5K_BOOL
ath5k_hw_set_power(struct ath_hal *hal, AR5K_POWER_MODE mode,
    AR5K_BOOL set_chip, u_int16_t sleep_duration)
{
	u_int32_t staid;
	int i;

	AR5K_TRACE;
	staid = AR5K_REG_READ(AR5K_STA_ID1);

	switch (mode) {
	case AR5K_PM_AUTO:
		staid &= ~AR5K_STA_ID1_DEFAULT_ANTENNA;
		/* fallthrough */
	case AR5K_PM_NETWORK_SLEEP:
		if (set_chip == TRUE) {
			AR5K_REG_WRITE(AR5K_SLEEP_CTL,
			    AR5K_SLEEP_CTL_SLE | sleep_duration);
		}
		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_FULL_SLEEP:
		if (set_chip == TRUE) {
			AR5K_REG_WRITE(AR5K_SLEEP_CTL,
			    AR5K_SLEEP_CTL_SLE_SLP);
		}
		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_AWAKE:
		if (set_chip == FALSE)
			goto commit;

		AR5K_REG_WRITE(AR5K_SLEEP_CTL, AR5K_SLEEP_CTL_SLE_WAKE);

		for (i = 5000; i > 0; i--) {
			/* Check if the chip did wake up */
			if ((AR5K_REG_READ(AR5K_PCICFG) &
			    AR5K_PCICFG_SPWR_DN) == 0)
				break;

			/* Wait a bit and retry */
			udelay(200);
			AR5K_REG_WRITE(AR5K_SLEEP_CTL,
			    AR5K_SLEEP_CTL_SLE_WAKE);
		}

		/* Fail if the chip didn't wake up */
		if (i <= 0)
			return (FALSE);

		staid &= ~AR5K_STA_ID1_PWR_SV;
		break;

	default:
		return (FALSE);
	}

 commit:
	hal->ah_power_mode = mode;

	AR5K_REG_WRITE(AR5K_STA_ID1, staid);

	return (TRUE);
}

/*
 * Get power mode (sleep state)
 * TODO:Remove ?
 */
AR5K_POWER_MODE
ath5k_hw_get_power_mode(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (hal->ah_power_mode);
}




/***********************\
  DMA Related Functions
\***********************/

/*
 * Receive functions
 */

/*
 * Start DMA receive
 */
void
ath5k_hw_start_rx(struct ath_hal *hal)
{
	AR5K_TRACE;
	AR5K_REG_WRITE(AR5K_CR, AR5K_CR_RXE);
}

/*
 * Stop DMA receive
 */
AR5K_BOOL
ath5k_hw_stop_rx_dma(struct ath_hal *hal)
{
	int i;

	AR5K_TRACE;
	AR5K_REG_WRITE(AR5K_CR, AR5K_CR_RXD);

	/*
	 * It may take some time to disable the DMA receive unit
	 */
	for (i = 2000;
	     i > 0 && (AR5K_REG_READ(AR5K_CR) & AR5K_CR_RXE) != 0;
	     i--)
		udelay(10);

	return (i > 0 ? TRUE : FALSE);
}

/*
 * Get the address of the RX Descriptor
 */
u_int32_t
ath5k_hw_get_rx_buf(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_RXDP));
}

/*
 * Set the address of the RX Descriptor
 */
void
ath5k_hw_put_rx_buf(struct ath_hal *hal, u_int32_t phys_addr)
{
	AR5K_TRACE;

	/*TODO:Shouldn't we check if RX is enabled first ?*/
	AR5K_REG_WRITE(AR5K_RXDP, phys_addr);
}

/*
 * Transmit functions
 */

/*
 * Start DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
AR5K_BOOL
ath5k_hw_tx_start(struct ath_hal *hal, u_int queue)
{
	u_int32_t tx_queue;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return (FALSE);

	if (hal->ah_version == AR5K_AR5210) {

		tx_queue = AR5K_REG_READ(AR5K_CR);

		/*
		 * Set the queue by type on 5210
		 */
		switch (hal->ah_txq[queue].tqi_type) {
			case AR5K_TX_QUEUE_DATA:
				tx_queue |= AR5K_CR_TXE0 & ~AR5K_CR_TXD0;
				break;
			case AR5K_TX_QUEUE_BEACON:
				tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
				AR5K_REG_WRITE(AR5K_BSR,
						AR5K_BCR_TQ1V | 
						AR5K_BCR_BDMAE);
				break;
			case AR5K_TX_QUEUE_CAB:
				tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
				AR5K_REG_WRITE(AR5K_BSR,
						AR5K_BCR_TQ1FV |
						AR5K_BCR_TQ1V |
						AR5K_BCR_BDMAE);
				break;
			default:
				return (FALSE);
		}
		/* Start queue */
		AR5K_REG_WRITE(AR5K_CR, tx_queue);
	} else {
		/* Return if queue is disabled */
		if (AR5K_REG_READ_Q(AR5K_QCU_TXD, queue))
			return (FALSE);

		/* Start queue */
		AR5K_REG_WRITE_Q(AR5K_QCU_TXE, queue);
	}

	return (TRUE);
}

/*
 * Stop DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
AR5K_BOOL
ath5k_hw_stop_tx_dma(struct ath_hal *hal, u_int queue)
{
	int i = 100, pending;
	u_int32_t tx_queue;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return (FALSE);

	if (hal->ah_version == AR5K_AR5210) {
		tx_queue = AR5K_REG_READ(AR5K_CR);

		/*
		 * Set by queue type
		 */
		switch (hal->ah_txq[queue].tqi_type) {
			case AR5K_TX_QUEUE_DATA:
				tx_queue |= AR5K_CR_TXD0 & ~AR5K_CR_TXE0;
				break;
			case AR5K_TX_QUEUE_BEACON:
			case AR5K_TX_QUEUE_CAB:
				/* XXX Fix me... */
				tx_queue |= AR5K_CR_TXD1 & ~AR5K_CR_TXD1;
				AR5K_REG_WRITE(AR5K_BSR, 0);
				break;
			default:
				return (FALSE);
		}

		/* Stop queue */
		AR5K_REG_WRITE(AR5K_CR, tx_queue);
	} else {
		/*
		 * Schedule TX disable and wait until queue is empty
		 */
		AR5K_REG_WRITE_Q(AR5K_QCU_TXD, queue);

		/*Check for pending frames*/
		do {
			pending = AR5K_REG_READ(AR5K_QUEUE_STATUS(queue)) &
						AR5K_QCU_STS_FRMPENDCNT;
			udelay(100);
		} while (--i && pending);

		/* Clear register */
		AR5K_REG_WRITE(AR5K_QCU_TXD, 0);
	}

	/*TODO: Check for success else return false*/
	return (TRUE);
}

/*
 * Get the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */ 
u_int32_t
ath5k_hw_get_tx_buf(struct ath_hal *hal, u_int queue)
{
	u_int16_t tx_reg;
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Get the transmit queue descriptor pointer from the selected queue
	 */
	/*5210 doesn't have QCU*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (hal->ah_txq[queue].tqi_type) {
			case AR5K_TX_QUEUE_DATA:
				tx_reg = AR5K_NOQCU_TXDP0;
				break;
			case AR5K_TX_QUEUE_BEACON:
			case AR5K_TX_QUEUE_CAB:
				tx_reg = AR5K_NOQCU_TXDP1;
				break;
			default:
				return (0xffffffff);
		}
	} else {
		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	return (AR5K_REG_READ(tx_reg));
}

/*
 * Set the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */
AR5K_BOOL
ath5k_hw_put_tx_buf(struct ath_hal *hal, u_int queue, u_int32_t phys_addr)
{
	u_int16_t tx_reg;
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Set the transmit queue descriptor pointer register by type
	 * on 5210
	 */
	if (hal->ah_version == AR5K_AR5210) {
		switch (hal->ah_txq[queue].tqi_type) {
			case AR5K_TX_QUEUE_DATA:
				tx_reg = AR5K_NOQCU_TXDP0;
				break;
			case AR5K_TX_QUEUE_BEACON:
			case AR5K_TX_QUEUE_CAB:
				tx_reg = AR5K_NOQCU_TXDP1;
				break;
			default:
				return (FALSE);
		}
	} else {
		/*
		 * Set the transmit queue descriptor pointer for 
		 * the selected queue on QCU for 5211+
		 * (this won't work if the queue is still active)
		 */
		if (AR5K_REG_READ_Q(AR5K_QCU_TXE, queue))
			return (FALSE);

		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	/* Set descriptor pointer */
	AR5K_REG_WRITE(tx_reg, phys_addr);

	return (TRUE);
}

/*
 * Update tx trigger level
 */
AR5K_BOOL
ath5k_hw_update_tx_triglevel(struct ath_hal *hal, AR5K_BOOL increase)
{
	u_int32_t trigger_level, imr;
	AR5K_BOOL status = FALSE;
	AR5K_TRACE;

	/*
	 * Disable interrupts by setting the mask
	 */
	imr = ath5k_hw_set_intr(hal, hal->ah_imr & ~AR5K_INT_GLOBAL);

	/*TODO: Boundary check on trigger_level*/
	trigger_level = AR5K_REG_MS(AR5K_REG_READ(AR5K_TXCFG),
			AR5K_TXCFG_TXFULL);

	if (increase == FALSE) {
		if (--trigger_level < AR5K_TUNE_MIN_TX_FIFO_THRES)
			goto done;
	} else
		trigger_level +=
			((AR5K_TUNE_MAX_TX_FIFO_THRES - trigger_level) / 2);

	/*
	 * Update trigger level on success
	 */
	if (hal->ah_version == AR5K_AR5210)
		AR5K_REG_WRITE(AR5K_TRIG_LVL, trigger_level);
	else
		AR5K_REG_WRITE_BITS(AR5K_TXCFG,
				AR5K_TXCFG_TXFULL, trigger_level);

	status = TRUE;

 done:
	/*
	 * Restore interrupt mask
	 */
	ath5k_hw_set_intr(hal, imr);

	return (status);
}

/*
 * Interrupt handling
 */

/*
 * Check if we have pending interrupts
 */
AR5K_BOOL
ath5k_hw_is_intr_pending(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (AR5K_REG_READ(AR5K_INTPEND) == TRUE ? TRUE : FALSE);
}

/*
 * Get interrupt mask (ISR)
 */
AR5K_BOOL
ath5k_hw_get_isr(struct ath_hal *hal, u_int32_t *interrupt_mask)
{
	u_int32_t data;

	AR5K_TRACE;

	/*
	 * Read interrupt status from the Interrupt Status register
	 * on 5210
	 */
	if (hal->ah_version == AR5K_AR5210) {
		if ((data = AR5K_REG_READ(AR5K_ISR)) == AR5K_INT_NOCARD) {
			*interrupt_mask = data;
			return (FALSE);
		}
	}

	/*
	 * Read interrupt status from the Read-And-Clear shadow register
	 */
	data = AR5K_REG_READ(AR5K_RAC_PISR);

	/*
	 * Get abstract interrupt mask (HAL-compatible)
	 */
	*interrupt_mask = (data & AR5K_INT_COMMON) & hal->ah_imr;

	if (data == AR5K_INT_NOCARD)
		return (FALSE);

	if (data & (AR5K_ISR_RXOK | AR5K_ISR_RXERR))
		*interrupt_mask |= AR5K_INT_RX;

	if (data & (AR5K_ISR_TXOK | AR5K_ISR_TXERR))
		*interrupt_mask |= AR5K_INT_TX;

	if (hal->ah_version != AR5K_AR5210) {
		/*HIU = Host Interface Unit (PCI etc)*/
		if (data & (AR5K_ISR_HIUERR))
			*interrupt_mask |= AR5K_INT_FATAL;

		/*Beacon Not Ready*/
		if (data & (AR5K_ISR_BNR))
			*interrupt_mask |= AR5K_INT_BNR;
	}

	/*
	 * Special interrupt handling (not caught by the driver)
	 */
	if (((*interrupt_mask) & AR5K_ISR_RXPHY) &&
	    hal->ah_radar.r_enabled == TRUE)
		ath5k_radar_alert(hal);

	/* 
	 * XXX: BMISS interrupts may occur after association. 
	 * I found this on 5210 code but it needs testing
	 */
#if 0
	interrupt_mask &= ~AR5K_INT_BMISS;
#endif

	/*
	 * In case we didn't handle anything,
	 * print the register value.
	 */
	if (*interrupt_mask == 0)
		AR5K_PRINTF("0x%08x\n", data);

	return (TRUE);
}

/*
 * Return the interrupt mask stored previously
 * TODO: Remove ?
 */
u_int32_t
ath5k_hw_get_intr(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (hal->ah_imr);
}

/*
 * Set interrupt mask
 */
AR5K_INT
ath5k_hw_set_intr(struct ath_hal *hal, AR5K_INT new_mask)
{
	AR5K_INT old_mask, int_mask;

	/*
	 * Disable card interrupts to prevent any race conditions
	 * (they will be re-enabled afterwards).
	 */
	AR5K_REG_WRITE(AR5K_IER, AR5K_IER_DISABLE);

	old_mask = hal->ah_imr;

	/*
	 * Add additional, chipset-dependent interrupt mask flags
	 * and write them to the IMR (interrupt mask register).
	 */
	int_mask = new_mask & AR5K_INT_COMMON;

	if (new_mask & AR5K_INT_RX)
		int_mask |=
			AR5K_IMR_RXOK |
			AR5K_IMR_RXERR |
			AR5K_IMR_RXORN |
			AR5K_IMR_RXDESC;

	if (new_mask & AR5K_INT_TX)
		int_mask |=
			AR5K_IMR_TXOK |
			AR5K_IMR_TXERR |
			AR5K_IMR_TXDESC |
			AR5K_IMR_TXURN;

	if (hal->ah_version != AR5K_AR5210) {
		if (new_mask & AR5K_INT_FATAL) {
			int_mask |= AR5K_IMR_HIUERR;
			AR5K_REG_ENABLE_BITS(AR5K_SIMR2,
					AR5K_SIMR2_MCABT |
					AR5K_SIMR2_SSERR |
					AR5K_SIMR2_DPERR);
		}
	}

	AR5K_REG_WRITE(AR5K_PIMR, int_mask);

	/* Store new interrupt mask */
	hal->ah_imr = new_mask;

	/* ..re-enable interrupts */
	AR5K_REG_WRITE(AR5K_IER, AR5K_IER_ENABLE);

	return (old_mask);
}

/*
 * Enalbe HW radar detection
 */
void
ath5k_hw_radar_alert(struct ath_hal *hal, AR5K_BOOL enable)
{

	AR5K_TRACE;
	/*
	 * Enable radar detection
	 */

	/*Disable interupts*/
	AR5K_REG_WRITE(AR5K_IER, AR5K_IER_DISABLE);

	/*
	 * Set the RXPHY interrupt to be able to detect
	 * possible radar activity.
	 */
	if (hal->ah_version == AR5K_AR5210) {
		if (enable == TRUE) {
			AR5K_REG_ENABLE_BITS(AR5K_IMR,
					AR5K_IMR_RXPHY);
		} else {
			AR5K_REG_DISABLE_BITS(AR5K_IMR,
					AR5K_IMR_RXPHY);
		}
	} else {
		/*Also set AR5K_PHY_RADAR register on 5111/5112*/
		if (enable == TRUE) {
			AR5K_REG_WRITE(AR5K_PHY_RADAR,
				AR5K_PHY_RADAR_ENABLE);
			AR5K_REG_ENABLE_BITS(AR5K_PIMR,
					AR5K_IMR_RXPHY);
		} else {
			AR5K_REG_WRITE(AR5K_PHY_RADAR,
				AR5K_PHY_RADAR_DISABLE);
			AR5K_REG_DISABLE_BITS(AR5K_PIMR,
					AR5K_IMR_RXPHY);
		}
	}

	/*Re-enable interrupts*/
	AR5K_REG_WRITE(AR5K_IER, AR5K_IER_ENABLE);
}




/*************************\
  EEPROM access functions
\*************************/

/*
 * Check if eeprom is busy
 */
AR5K_BOOL
ath5k_hw_eeprom_is_busy(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (AR5K_REG_READ(AR5K_CFG) & AR5K_CFG_EEBS ?
		TRUE : FALSE);
}

/*
 * Read from eeprom
 */
int
ath5k_hw_eeprom_read(struct ath_hal *hal, u_int32_t offset, u_int16_t *data)
{
	u_int32_t status, timeout;

	AR5K_TRACE;
	/*
	 * Initialize EEPROM access
	 */
	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_EEAE);
		(void)AR5K_REG_READ(AR5K_EEPROM_BASE + (4 * offset));
	} else {
		AR5K_REG_WRITE(AR5K_EEPROM_BASE, offset);
		AR5K_REG_ENABLE_BITS(AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_READ);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = AR5K_REG_READ(AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_RDDONE) {
			if (status & AR5K_EEPROM_STAT_RDERR)
				return (EIO);
			*data = (u_int16_t)
			    (AR5K_REG_READ(AR5K_EEPROM_DATA) & 0xffff);
			return (0);
		}
		udelay(15);
	}

	return (ETIMEDOUT);
}

/*
 * Write to eeprom - currently disabled, use at your own risk
 */
int
ath5k_hw_eeprom_write(struct ath_hal *hal, u_int32_t offset, u_int16_t data)
{
#if 0
	u_int32_t status, timeout;

	AR5K_TRACE;

	/*
	 * Initialize eeprom access
	 */

	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_EEAE);
	} else {
		AR5K_REG_ENABLE_BITS(AR5K_EEPROM_CMD, AR5K_EEPROM_CMD_RESET);
	}

	/*
	 * Write data to data register
	 */

	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_WRITE(AR5K_EEPROM_BASE + (4 * offset), data);
	} else {
		AR5K_REG_WRITE(AR5K_EEPROM_BASE, offset);
		AR5K_REG_WRITE(AR5K_EEPROM_DATA, data);
		AR5K_REG_ENABLE_BITS(AR5K_EEPROM_CMD, AR5K_EEPROM_CMD_WRITE);
	}

	/*
	 * Check status
	 */

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = AR5K_REG_READ(AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_WRDONE) {
			if (status & AR5K_EEPROM_STAT_WRERR)
				return (EIO);
			return (0);
		}
		udelay(15);
	}
#endif
	AR5K_PRINTF("EEPROM Write is disabled!");
	return (ETIMEDOUT);
}

/*
 * Translate binary channel representation in EEPROM to frequency
 */
u_int16_t
ath5k_hw_eeprom_bin2freq(struct ath_hal *hal, u_int16_t bin, u_int mode)
{
	u_int16_t val;

	if (bin == AR5K_EEPROM_CHANNEL_DIS)
		return (bin);

	if (mode == AR5K_EEPROM_MODE_11A) {
		if (hal->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = (5 * bin) + 4800;
		else
			val = bin > 62 ?
			    (10 * 62) + (5 * (bin - 62)) + 5100 :
			    (bin * 10) + 5100;
	} else {
		if (hal->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = bin + 2300;
		else
			val = bin + 2400;
	}

	return (val);
}

/*
 * Read antenna infos from eeprom
 */
int
ath5k_hw_eeprom_read_ants(struct ath_hal *hal, u_int32_t *offset, u_int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int32_t o = *offset;
	u_int16_t val;
	int ret, i = 0;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_switch_settling[mode]	= (val >> 8) & 0x7f;
	ee->ee_ant_tx_rx[mode]		= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	= (val >> 10) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 4) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 2) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 14) & 0x3;
	ee->ee_ant_control[mode][i++]	= (val >> 8) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	/* Get antenna modes */
	hal->ah_antenna[mode][0] =
	    (ee->ee_ant_control[mode][0] << 4) | 0x1;
	hal->ah_antenna[mode][AR5K_ANT_FIXED_A] =
	     ee->ee_ant_control[mode][1] 	|
	    (ee->ee_ant_control[mode][2] << 6) 	|
	    (ee->ee_ant_control[mode][3] << 12) |
	    (ee->ee_ant_control[mode][4] << 18) |
	    (ee->ee_ant_control[mode][5] << 24);
	hal->ah_antenna[mode][AR5K_ANT_FIXED_B] =
	     ee->ee_ant_control[mode][6] 	|
	    (ee->ee_ant_control[mode][7] << 6) 	|
	    (ee->ee_ant_control[mode][8] << 12) |
	    (ee->ee_ant_control[mode][9] << 18) |
	    (ee->ee_ant_control[mode][10] << 24);

	/* return new offset */
	*offset = o;

	return (0);
}

/*
 * Read some mode-specific values from EEPROM for phy calibration
 */
int
ath5k_hw_eeprom_read_modes(struct ath_hal *hal, u_int32_t *offset, u_int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int32_t o = *offset;
	u_int16_t val;
	int ret;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xlna_enable[mode]	= (val >> 8) & 0xff;
	ee->ee_thr_62[mode]		= val & 0xff;

	if (hal->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_thr_62[mode] =
		    mode == AR5K_EEPROM_MODE_11A ? 15 : 28;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xpa_disable[mode]	= (val >> 8) & 0xff;
	ee->ee_tx_frm2xpa_enable[mode]	= val & 0xff;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_pga_desired_size[mode]	= (val >> 8) & 0xff;

	if ((val & 0xff) & 0x80)
		ee->ee_noise_floor_thr[mode] = -((((val & 0xff) ^ 0xff)) + 1);
	else
		ee->ee_noise_floor_thr[mode] = val & 0xff;

	if (hal->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_noise_floor_thr[mode] =
		    mode == AR5K_EEPROM_MODE_11A ? -54 : -1;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_xlna_gain[mode]		= (val >> 5) & 0xff;
	ee->ee_x_gain[mode]		= (val >> 1) & 0xf;
	ee->ee_xpd[mode]		= val & 0x1;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0)
		ee->ee_fixed_bias[mode] = (val >> 13) & 0x1;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(o++, val);
		ee->ee_false_detect[mode] = (val >> 6) & 0x7f;

		if (mode == AR5K_EEPROM_MODE_11A)
			ee->ee_xr_power[mode] = val & 0x3f;
		else {
			ee->ee_ob[mode][0] = val & 0x7;
			ee->ee_db[mode][0] = (val >> 3) & 0x7;
		}
	}

	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_4) {
		ee->ee_i_gain[mode] = AR5K_EEPROM_I_GAIN;
		ee->ee_cck_ofdm_power_delta = AR5K_EEPROM_CCK_OFDM_DELTA;
	} else {
		ee->ee_i_gain[mode] = (val >> 13) & 0x7;

		AR5K_EEPROM_READ(o++, val);
		ee->ee_i_gain[mode] |= (val << 3) & 0x38;

		if (mode == AR5K_EEPROM_MODE_11G)
			ee->ee_cck_ofdm_power_delta = (val >> 3) & 0xff;
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0 &&
	    mode == AR5K_EEPROM_MODE_11A) {
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_6 &&
	    mode == AR5K_EEPROM_MODE_11G)
		ee->ee_scaled_cck_delta = (val >> 11) & 0x1f;

	/* return new offset */
	*offset = o;

	return (0);
}

/*
 * Initialize eeprom & capabilities structs
 */
int
ath5k_hw_eeprom_init(struct ath_hal *hal)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int32_t offset;
	u_int16_t val;
	int ret, i;
	u_int mode;

	/* Initial TX thermal adjustment values */
	ee->ee_tx_clip = 4;
	ee->ee_pwd_84 = ee->ee_pwd_90 = 1;
	ee->ee_gain_select = 1;

	/*
	 * Read values from EEPROM and store them in the capability structure
	 */
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MAGIC, ee_magic);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_PROTECT, ee_protect);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_REG_DOMAIN, ee_regdomain);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_VERSION, ee_version);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_HDR, ee_header);

	/* Return if we have an old EEPROM */
	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_0)
		return (0);

#ifdef notyet
	/*
	 * Validate the checksum of the EEPROM date. There are some
	 * devices with invalid EEPROMs.
	 */
	for (cksum = 0, offset = 0; offset < AR5K_EEPROM_INFO_MAX; offset++) {
		AR5K_EEPROM_READ(AR5K_EEPROM_INFO(offset), val);
		cksum ^= val;
	}
	if (cksum != AR5K_EEPROM_INFO_CKSUM) {
		AR5K_PRINTF("Invalid EEPROM checksum 0x%04x\n", cksum);
		return (AR5K_EEBADSUM);
	}
#endif

	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_ANT_GAIN(hal->ah_ee_version),
	    ee_ant_gain);

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC0, ee_misc0);
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC1, ee_misc1);
	}

	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB0_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11B][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11B][0] = (val >> 3) & 0x7;

		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB1_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11G][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11G][0] = (val >> 3) & 0x7;
	}

	/*
	 * Get conformance test limit values
	 */
	offset = AR5K_EEPROM_CTL(hal->ah_ee_version);
	ee->ee_ctls = AR5K_EEPROM_N_CTLS(hal->ah_ee_version);

	for (i = 0; i < ee->ee_ctls; i++) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_ctl[i] = (val >> 8) & 0xff;
		ee->ee_ctl[i + 1] = val & 0xff;
	}

	/*
	 * Get values for 802.11a (5GHz)
	 */
	mode = AR5K_EEPROM_MODE_11A;

	ee->ee_turbo_max_power[mode] =
	    AR5K_EEPROM_HDR_T_5GHZ_DBM(ee->ee_header);

	offset = AR5K_EEPROM_MODES_11A(hal->ah_ee_version);

	if ((ret = ath5k_hw_eeprom_read_ants(hal, &offset, mode)) != 0)
		return (ret);

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (int8_t)((val >> 8) & 0xff);
	ee->ee_ob[mode][3]		= (val >> 5) & 0x7;
	ee->ee_db[mode][3]		= (val >> 2) & 0x7;
	ee->ee_ob[mode][2]		= (val << 1) & 0x7;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_ob[mode][2]		|= (val >> 15) & 0x1;
	ee->ee_db[mode][2]		= (val >> 12) & 0x7;
	ee->ee_ob[mode][1]		= (val >> 9) & 0x7;
	ee->ee_db[mode][1]		= (val >> 6) & 0x7;
	ee->ee_ob[mode][0]		= (val >> 3) & 0x7;
	ee->ee_db[mode][0]		= val & 0x7;

	if ((ret = ath5k_hw_eeprom_read_modes(hal, &offset, mode)) != 0)
		return (ret);

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_margin_tx_rx[mode] = val & 0x3f;
	}

	/*
	 * Get values for 802.11b (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11B;
	offset = AR5K_EEPROM_MODES_11B(hal->ah_ee_version);

	if ((ret = ath5k_hw_eeprom_read_ants(hal, &offset, mode)) != 0)
		return (ret);

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (int8_t)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	if ((ret = ath5k_hw_eeprom_read_modes(hal, &offset, mode)) != 0)
		return (ret);

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
		    ath5k_hw_eeprom_bin2freq(hal, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
		    ath5k_hw_eeprom_bin2freq(hal, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
		    ath5k_hw_eeprom_bin2freq(hal, val & 0xff, mode);
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
		ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;
	}

	/*
	 * Get values for 802.11g (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11G;
	offset = AR5K_EEPROM_MODES_11G(hal->ah_ee_version);

	if ((ret = ath5k_hw_eeprom_read_ants(hal, &offset, mode)) != 0)
		return (ret);

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (int8_t)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	if ((ret = ath5k_hw_eeprom_read_modes(hal, &offset, mode)) != 0)
		return (ret);

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
		    ath5k_hw_eeprom_bin2freq(hal, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
		    ath5k_hw_eeprom_bin2freq(hal, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_turbo_max_power[mode] = val & 0x7f;
		ee->ee_xr_power[mode] = (val >> 7) & 0x3f;

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
		    ath5k_hw_eeprom_bin2freq(hal, val & 0xff, mode);

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
			ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;
		}

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_2) {
			AR5K_EEPROM_READ(offset++, val);
			ee->ee_cck_ofdm_gain_delta = val & 0xff;
		}
	}

	/*
	 * Read 5GHz EEPROM channels
	 */

	return (0);
}

/*
 * Read the MAC address from eeprom
 * TODO: Update ath5kregs.h to include these offsets
 */
int
ath5k_hw_eeprom_read_mac(struct ath_hal *hal, u_int8_t *mac)
{
	u_int32_t total, offset;
	u_int16_t data;
	int octet;
	u_int8_t mac_d[ETH_ALEN];

	memset(mac, 0, ETH_ALEN);
	memset(&mac_d, 0, ETH_ALEN);

	if (ath5k_hw_eeprom_read(hal, 0x20, &data) != 0)
		return (AR5K_EIO);

	for (offset = 0x1f, octet = 0, total = 0;
	     offset >= 0x1d; offset--) {
		if (ath5k_hw_eeprom_read(hal, offset, &data) != 0)
			return (AR5K_EIO);

		total += data;
		mac_d[octet + 1] = data & 0xff;
		mac_d[octet] = data >> 8;
		octet += 2;
	}

	memcpy(mac, mac_d, ETH_ALEN);

	if ((!total) || total == (3 * 0xffff))
		return (AR5K_EINVAL);

	return (0);
}

/*
 * Read/Write refulatory domain
 */
AR5K_BOOL
ath5k_hw_eeprom_regulation_domain(struct ath_hal *hal, AR5K_BOOL write,
    ieee80211_regdomain_t *regdomain)
{
	u_int16_t ee_regdomain;

	/* Read current value */
	if (write != TRUE) {
		ee_regdomain = hal->ah_capabilities.cap_eeprom.ee_regdomain;
		*regdomain = ath5k_regdomain_to_ieee(ee_regdomain);
		return (TRUE);
	}

	ee_regdomain = ath5k_regdomain_from_ieee(*regdomain);

	/* Try to write a new value */
	if (hal->ah_capabilities.cap_eeprom.ee_protect &
	    AR5K_EEPROM_PROTECT_WR_128_191)
		return (FALSE);
	if (ath5k_hw_eeprom_write(hal, AR5K_EEPROM_REG_DOMAIN,
	    ee_regdomain) != 0)
		return (FALSE);

	hal->ah_capabilities.cap_eeprom.ee_regdomain = ee_regdomain;

	return (TRUE);
}

/*
 * Use the above to write a new regulatory domain
 */
AR5K_BOOL
ath5k_hw_set_regdomain(struct ath_hal *hal, u_int16_t regdomain,
    AR5K_STATUS *status)
{
	ieee80211_regdomain_t ieee_regdomain;

	ieee_regdomain = ath5k_regdomain_to_ieee(regdomain);

	if (ath5k_hw_eeprom_regulation_domain(hal, TRUE,
		&ieee_regdomain) == TRUE) {
		*status = AR5K_OK;
		return (TRUE);
	}

	*status = AR5K_EIO;

	return (FALSE);
}

/*
 * Fill the capabilities struct
 */
AR5K_BOOL
ath5k_hw_get_capabilities(struct ath_hal *hal)
{
	u_int16_t ee_header;

	AR5K_TRACE;
	/* Capabilities stored in the EEPROM */
	ee_header = hal->ah_capabilities.cap_eeprom.ee_header;

	if (hal->ah_version == AR5K_AR5210) {
		/*
		 * Set radio capabilities
		 * (The AR5110 only supports the middle 5GHz band)
		 */
		hal->ah_capabilities.cap_range.range_5ghz_min = 5120;
		hal->ah_capabilities.cap_range.range_5ghz_max = 5430;
		hal->ah_capabilities.cap_range.range_2ghz_min = 0;
		hal->ah_capabilities.cap_range.range_2ghz_max = 0;

		/* Set supported modes */
		hal->ah_capabilities.cap_mode = AR5K_MODE_11A | AR5K_MODE_TURBO;
	} else {
		/*
		 * XXX The tranceiver supports frequencies from 4920 to 6100GHz
		 * XXX and from 2312 to 2732GHz. There are problems with the current
		 * XXX ieee80211 implementation because the IEEE channel mapping
		 * XXX does not support negative channel numbers (2312MHz is channel
		 * XXX -19). Of course, this doesn't matter because these channels
		 * XXX are out of range but some regulation domains like MKK (Japan)
		 * XXX will support frequencies somewhere around 4.8GHz.
		 */

		/*
		 * Set radio capabilities
		 */

		if (AR5K_EEPROM_HDR_11A(ee_header)) {
			hal->ah_capabilities.cap_range.range_5ghz_min = 5005; /* 4920 */
			hal->ah_capabilities.cap_range.range_5ghz_max = 6100;
	
			/* Set supported modes */
			hal->ah_capabilities.cap_mode = AR5K_MODE_11A | AR5K_MODE_TURBO | 
					(hal->ah_version == AR5K_AR5212 ? AR5K_MODE_XR : 0);
		}
	
		/* Enable  802.11b if a 2GHz capable radio (2111/5112) is connected */
		if (AR5K_EEPROM_HDR_11B(ee_header) || AR5K_EEPROM_HDR_11G(ee_header)) {
			hal->ah_capabilities.cap_range.range_2ghz_min = 2412; /* 2312 */
			hal->ah_capabilities.cap_range.range_2ghz_max = 2732;
	
			if (AR5K_EEPROM_HDR_11B(ee_header))
				hal->ah_capabilities.cap_mode |= AR5K_MODE_11B;

			if (AR5K_EEPROM_HDR_11G(ee_header))
				hal->ah_capabilities.cap_mode |= AR5K_MODE_11G;

		}
	}

	/* GPIO */
	hal->ah_gpio_npins = AR5K_NUM_GPIO;

	/* Set number of supported TX queues */
	if (hal->ah_version == AR5K_AR5210)
		hal->ah_capabilities.cap_queues.q_tx_num = AR5K_NUM_TX_QUEUES_NOQCU;
	else
		hal->ah_capabilities.cap_queues.q_tx_num = AR5K_NUM_TX_QUEUES;

	return (TRUE);
}




/*********************************\
  Protocol Control Unit Functions
\*********************************/

/*
 * Set Operation mode
 */
void
ath5k_hw_set_opmode(struct ath_hal *hal)
{
	u_int32_t pcu_reg, beacon_reg, low_id, high_id;

	pcu_reg = 0;
	beacon_reg = 0;

	AR5K_TRACE;

	switch (hal->ah_op_mode) {
	case AR5K_M_IBSS:
		pcu_reg |= AR5K_STA_ID1_ADHOC |
			AR5K_STA_ID1_DESC_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?AR5K_STA_ID1_NO_PSPOLL :0);

		beacon_reg |= AR5K_BCR_ADHOC;
		break;

	case AR5K_M_HOSTAP:
		pcu_reg |= AR5K_STA_ID1_AP |
			AR5K_STA_ID1_RTS_DEF_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?AR5K_STA_ID1_NO_PSPOLL :0);

		beacon_reg |= AR5K_BCR_AP;
		break;

	case AR5K_M_STA:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?AR5K_STA_ID1_PWR_SV :0);
	case AR5K_M_MONITOR:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?AR5K_STA_ID1_NO_PSPOLL :0);
		break;

	default:
		return;
	}

	/*
	 * Set PCU registers
	 */
	low_id = AR5K_LOW_ID(hal->ah_sta_id);
	high_id = AR5K_HIGH_ID(hal->ah_sta_id);
	AR5K_REG_WRITE(AR5K_STA_ID0, low_id);
	AR5K_REG_WRITE(AR5K_STA_ID1, pcu_reg | high_id);

	/*
	 * Set Beacon Control Register on 5210
	 */
	if (hal->ah_version == AR5K_AR5210)
		AR5K_REG_WRITE(AR5K_BCR, beacon_reg);

	return;
}

void /*TODO: Get rid of this, clean up the driver code, only set_opmode is needed*/
ath5k_hw_set_pcu_config(struct ath_hal *hal)
{
	AR5K_TRACE;
	ath5k_hw_set_opmode(hal);
	return;
}

/*
 * BSSID Functions
 */

/*
 * Get station id
 */
void
ath5k_hw_get_lladdr(struct ath_hal *hal, u_int8_t *mac)
{
	AR5K_TRACE;
	memcpy(mac, hal->ah_sta_id, ETH_ALEN);
}

/*
 * Set station id
 */
AR5K_BOOL
ath5k_hw_set_lladdr(struct ath_hal *hal, const u_int8_t *mac)
{
	u_int32_t low_id, high_id;

	AR5K_TRACE;
	/* Set new station ID */
	memcpy(hal->ah_sta_id, mac, ETH_ALEN);

	low_id = AR5K_LOW_ID(mac);
	high_id = AR5K_HIGH_ID(mac);

	AR5K_REG_WRITE(AR5K_STA_ID0, low_id);
	AR5K_REG_WRITE(AR5K_STA_ID1, high_id);

	return (TRUE);
}

/*
 * Set BSSID
 */
void
ath5k_hw_set_associd(struct ath_hal *hal, const u_int8_t *bssid,
    u_int16_t assoc_id)
{
	u_int32_t low_id, high_id;
	u_int16_t tim_offset = 0;

	/*
	 * Set simple BSSID mask on 5212
	 */
	if (hal->ah_version == AR5K_AR5212) {
		AR5K_REG_WRITE(AR5K_BSS_IDM0, 0xfffffff);
		AR5K_REG_WRITE(AR5K_BSS_IDM1, 0xfffffff);
	}

	/*
	 * Set BSSID which triggers the "SME Join" operation
	 */
	low_id = AR5K_LOW_ID(bssid);
	high_id = AR5K_HIGH_ID(bssid);
	AR5K_REG_WRITE(AR5K_BSS_ID0, low_id);
	AR5K_REG_WRITE(AR5K_BSS_ID1, high_id |
	    ((assoc_id & 0x3fff) << AR5K_BSS_ID1_AID_S));
	memcpy(&hal->ah_bssid, bssid, ETH_ALEN);

	if (assoc_id == 0) {
		ath5k_hw_disable_pspoll(hal);
		return;
	}

	AR5K_REG_WRITE_BITS(AR5K_BEACON, AR5K_BEACON_TIM,
	    tim_offset ? tim_offset + 4 : 0);

	ath5k_hw_enable_pspoll(hal, NULL, 0);
}

/*
 * Set BSSID mask on 5212
 */
AR5K_BOOL
ath5k_hw_set_bssid_mask(struct ath_hal *hal, const u_int8_t* mask)
{
	u_int32_t low_id, high_id;
	AR5K_TRACE;

	if (hal->ah_version == AR5K_AR5212) {

		low_id = AR5K_LOW_ID(mask);
		high_id = AR5K_HIGH_ID(mask);

		AR5K_REG_WRITE(AR5K_BSS_IDM0, low_id); 
		AR5K_REG_WRITE(AR5K_BSS_IDM1, high_id); 

		return (TRUE); 
	} else
		return (FALSE);
}

/*
 * Receive start/stop functions
 */

/*
 * Start receive on PCU
 */
void
ath5k_hw_start_rx_pcu(struct ath_hal *hal)
{
	AR5K_TRACE;
	AR5K_REG_DISABLE_BITS(AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * Stop receive on PCU
 */
void
ath5k_hw_stop_pcu_recv(struct ath_hal *hal)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * RX Filter functions
 */

/*
 * Set multicast filter
 */
void
ath5k_hw_set_mcast_filter(struct ath_hal *hal, u_int32_t filter0,
    u_int32_t filter1)
{
	AR5K_TRACE;
	/* Set the multicat filter */
	AR5K_REG_WRITE(AR5K_MCAST_FILTER0, filter0);
	AR5K_REG_WRITE(AR5K_MCAST_FILTER1, filter1);
}

/*
 * Set multicast filter by index
 */
AR5K_BOOL
ath5k_hw_set_mcast_filterindex(struct ath_hal *hal, u_int32_t index)
{

	AR5K_TRACE;
	if (index >= 64)
	    return (FALSE);
	else if (index >= 32)
	    AR5K_REG_ENABLE_BITS(AR5K_MCAST_FILTER1,
		(1 << (index - 32)));
	else
	    AR5K_REG_ENABLE_BITS(AR5K_MCAST_FILTER0,
		(1 << index));

	return (TRUE);
}

/*
 * Clear Multicast filter by index
 */
AR5K_BOOL
ath5k_hw_clear_mcast_filter_idx(struct ath_hal *hal, u_int32_t index)
{

	AR5K_TRACE;
	if (index >= 64)
	    return (FALSE);
	else if (index >= 32)
	    AR5K_REG_DISABLE_BITS(AR5K_MCAST_FILTER1,
		(1 << (index - 32)));
	else
	    AR5K_REG_DISABLE_BITS(AR5K_MCAST_FILTER0,
		(1 << index));

	return (TRUE);
}

/*
 * Get current rx filter
 */
u_int32_t
ath5k_hw_get_rx_filter(struct ath_hal *hal)
{
	u_int32_t data, filter = 0;

	AR5K_TRACE;
	filter = AR5K_REG_READ(AR5K_RX_FILTER);

	/*Radar detection for 5212*/
	if (hal->ah_version == AR5K_AR5212) {
		data = AR5K_REG_READ(AR5K_PHY_ERR_FIL);

		if (data & AR5K_PHY_ERR_FIL_RADAR)
			filter |= AR5K_RX_FILTER_PHYRADAR;
		if (data & (AR5K_PHY_ERR_FIL_OFDM |
			AR5K_PHY_ERR_FIL_CCK))
			filter |= AR5K_RX_FILTER_PHYERR;
	}

	return (filter);
}

/*
 * Set rx filter
 */
void
ath5k_hw_set_rx_filter(struct ath_hal *hal, u_int32_t filter)
{
	u_int32_t data = 0;

	AR5K_TRACE;

	/* Set PHY error filter register on 5212*/
	if (hal->ah_version == AR5K_AR5212) {
		if (filter & AR5K_RX_FILTER_PHYRADAR)
			data |= AR5K_PHY_ERR_FIL_RADAR;
		if (filter & AR5K_RX_FILTER_PHYERR)
			data |= AR5K_PHY_ERR_FIL_OFDM |
				AR5K_PHY_ERR_FIL_CCK;
	}

	/*
	 * The AR5210 uses promiscous mode to detect radar activity
	 */
	if ((hal->ah_version == AR5K_AR5210) && 
			(filter & AR5K_RX_FILTER_PHYRADAR)) {
		filter &= ~AR5K_RX_FILTER_PHYRADAR;
		filter |= AR5K_RX_FILTER_PROM;
	}

	/*Zero length DMA*/
	if (data)
		AR5K_REG_ENABLE_BITS(AR5K_RXCFG,
				AR5K_RXCFG_ZLFDMA);
	else
		AR5K_REG_DISABLE_BITS(AR5K_RXCFG,
				AR5K_RXCFG_ZLFDMA);

	/*Write RX Filter register*/
	AR5K_REG_WRITE(AR5K_RX_FILTER, filter & 0xff);

	/*Write PHY error filter register on 5212*/
	if (hal->ah_version == AR5K_AR5212)
		AR5K_REG_WRITE(AR5K_PHY_ERR_FIL, data);

}

/*
 * Beacon related functions
 */

/*
 * Get a 32bit TSF
 */
u_int32_t
ath5k_hw_get_tsf32(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (AR5K_REG_READ(AR5K_TSF_L32));
}

/*
 * Get the full 64bit TSF
 */
u_int64_t
ath5k_hw_get_tsf64(struct ath_hal *hal)
{
	u_int64_t tsf = AR5K_REG_READ(AR5K_TSF_U32);
	AR5K_TRACE;

	return (AR5K_REG_READ(AR5K_TSF_L32) | (tsf << 32));
}

/*
 * Force a TSF reset
 */
void
ath5k_hw_reset_tsf(struct ath_hal *hal)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(AR5K_BEACON,
	    AR5K_BEACON_RESET_TSF);
}

/*
 * Initialize beacon timers
 */
void
ath5k_hw_init_beacon(struct ath_hal *hal, u_int32_t next_beacon,
    u_int32_t interval)
{
	u_int32_t timer1, timer2, timer3;

	AR5K_TRACE;
	/*
	 * Set the additional timers by mode
	 */
	switch (hal->ah_op_mode) {
	case AR5K_M_STA:
		if (hal->ah_version == AR5K_AR5210) {
			timer1 = 0xffffffff;
			timer2 = 0xffffffff;
		} else {
			timer1 = 0x0000ffff;
			timer2 = 0x0007ffff;
		}
		break;

	default:
		timer1 = (next_beacon - AR5K_TUNE_DMA_BEACON_RESP) <<
		    0x00000003;
		timer2 = (next_beacon - AR5K_TUNE_SW_BEACON_RESP) <<
		    0x00000003;
	}

	timer3 = next_beacon +
	    (hal->ah_atim_window ? hal->ah_atim_window : 1);

	/*
	 * Set the beacon register and enable all timers.
	 * (next beacon, DMA beacon, software beacon, ATIM window time)
	 */
	AR5K_REG_WRITE(AR5K_TIMER0, next_beacon);
	AR5K_REG_WRITE(AR5K_TIMER1, timer1);
	AR5K_REG_WRITE(AR5K_TIMER2, timer2);
	AR5K_REG_WRITE(AR5K_TIMER3, timer3);

	AR5K_REG_WRITE(AR5K_BEACON, interval &
	    (AR5K_BEACON_PERIOD | AR5K_BEACON_RESET_TSF |
	    AR5K_BEACON_ENABLE));
}

/*
 * Set beacon timers
 */
void
ath5k_hw_set_beacon_timers(struct ath_hal *hal, const AR5K_BEACON_STATE *state)
{
	u_int32_t cfp_period, next_cfp, dtim, interval, next_beacon;

 	/* 
	 * TODO: should be changed through *state
	 * review AR5K_BEACON_STATE struct
	 *
	 * XXX: These are used for cfp period bellow, are they
	 * ok ? Is it O.K. for tsf here to be 0 or should we use
	 * get_tsf ?
	 */
	u_int32_t dtim_count = 0; /* XXX */
	u_int32_t cfp_count = 0; /* XXX */
	u_int32_t tsf = 0; /* XXX */

	AR5K_TRACE;
	/* Return on an invalid beacon state */
	if (state->bs_interval < 1)
		return;

	interval = state->bs_interval;
	dtim = state->bs_dtim_period;

	/*
	 * PCF support?
	 */
	if (state->bs_cfp_period > 0) {
		/* 
		 * Enable PCF mode and set the CFP 
		 * (Contention Free Period) and timer registers
		 */
		cfp_period = state->bs_cfp_period * state->bs_dtim_period *
		    state->bs_interval;
		next_cfp = (cfp_count * state->bs_dtim_period + dtim_count) *
		    state->bs_interval;

		AR5K_REG_ENABLE_BITS(AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
		AR5K_REG_WRITE(AR5K_CFP_PERIOD, cfp_period);
		AR5K_REG_WRITE(AR5K_CFP_DUR, state->bs_cfp_max_duration);
		AR5K_REG_WRITE(AR5K_TIMER2,
		    (tsf + (next_cfp == 0 ? cfp_period : next_cfp)) << 3);
	} else {
		/* Disable PCF mode */
		AR5K_REG_DISABLE_BITS(AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
	}

	/*
	 * Enable the beacon timer register
	 */
	AR5K_REG_WRITE(AR5K_TIMER0, state->bs_next_beacon);

	/*
	 * Start the beacon timers
	 */
	AR5K_REG_WRITE(AR5K_BEACON,
		(AR5K_REG_READ(AR5K_BEACON) &~
		(AR5K_BEACON_PERIOD | AR5K_BEACON_TIM)) |
		AR5K_REG_SM(state->bs_tim_offset ? state->bs_tim_offset + 4 : 0,
		AR5K_BEACON_TIM) | AR5K_REG_SM(state->bs_interval,
		AR5K_BEACON_PERIOD));

	/*
	 * Write new beacon miss threshold, if it appears to be valid
	 * XXX: Figure out right values for min <= bs_bmiss_threshold <= max
	 * and return if its not in range. We can test this by reading value and
	 * setting value to a largest value and seeing which values register.
	 */

	AR5K_REG_WRITE_BITS(AR5K_RSSI_THR,
	    AR5K_RSSI_THR_BMISS, state->bs_bmiss_threshold);

	/*
	 * Set sleep control register
	 * XXX: Didn't find this in 5210 code but since this register
	 * exists also in ar5k's 5210 headers i leave it as common code.
	 */
	AR5K_REG_WRITE_BITS(AR5K_SLEEP_CTL, AR5K_SLEEP_CTL_SLDUR,
			(state->bs_sleep_duration - 3) << 3);

	/*
	 * Set enhanced sleep registers on 5212
	 */
	if (hal->ah_version == AR5K_AR5212) {
		if ((state->bs_sleep_duration > state->bs_interval) &&
		(roundup(state->bs_sleep_duration, interval) ==
		state->bs_sleep_duration))
			interval = state->bs_sleep_duration;

		if (state->bs_sleep_duration > dtim &&
		(dtim == 0 || roundup(state->bs_sleep_duration, dtim) ==
		state->bs_sleep_duration))
			dtim = state->bs_sleep_duration;

		if (interval > dtim)
			return;
	
		next_beacon = interval == dtim ?
			state->bs_next_dtim: state->bs_next_beacon;

		AR5K_REG_WRITE(AR5K_SLEEP0,
			AR5K_REG_SM((state->bs_next_dtim - 3) << 3,
			AR5K_SLEEP0_NEXT_DTIM) |
			AR5K_REG_SM(10, AR5K_SLEEP0_CABTO) |
			AR5K_SLEEP0_ENH_SLEEP_EN |
			AR5K_SLEEP0_ASSUME_DTIM);

		AR5K_REG_WRITE(AR5K_SLEEP1,
			AR5K_REG_SM((next_beacon - 3) << 3,
			AR5K_SLEEP1_NEXT_TIM) |
			AR5K_REG_SM(10, AR5K_SLEEP1_BEACON_TO));

		AR5K_REG_WRITE(AR5K_SLEEP2,
			AR5K_REG_SM(interval, AR5K_SLEEP2_TIM_PER) |
			AR5K_REG_SM(dtim, AR5K_SLEEP2_DTIM_PER));
	}
}

/*
 * Reset beacon timers
 */
void
ath5k_hw_reset_beacon(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*
	 * Disable beacon timer
	 */
	AR5K_REG_WRITE(AR5K_TIMER0, 0);

	/*
	 * Disable some beacon register values
	 */
	AR5K_REG_DISABLE_BITS(AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA | 
				AR5K_STA_ID1_PCF);
	AR5K_REG_WRITE(AR5K_BEACON, AR5K_BEACON_PERIOD);
}

/*
 * Wait for beacon queue to finish
 * TODO: This function's name is misleading, rename
 */
AR5K_BOOL
ath5k_hw_wait_for_beacon(struct ath_hal *hal, AR5K_BUS_ADDR phys_addr)
{
	AR5K_BOOL ret;
	int i;

	AR5K_TRACE;

	/* 5210 doesn't have QCU*/
	if (hal->ah_version == AR5K_AR5210) {
		/*
		 * Wait for beaconn queue to finish by checking
		 * Control Register and Beacon Status Register.
		 */
		for (i = (AR5K_TUNE_BEACON_INTERVAL / 2); i > 0 &&
		(AR5K_REG_READ(AR5K_BSR) & AR5K_BSR_TXQ1F) != 0 &&
		(AR5K_REG_READ(AR5K_CR) & AR5K_CR_TXE1 ) != 0; i--);

		/* Timeout... */
		if (i <= 0) {
			/*
			 * Re-schedule the beacon queue
			 */
			AR5K_REG_WRITE(AR5K_NOQCU_TXDP1, (u_int32_t)phys_addr);
			AR5K_REG_WRITE(AR5K_BCR, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE);

			return (FALSE);
		}
		ret= TRUE;

	} else {
	/*5211/5212*/
		ret = ath5k_hw_register_timeout(hal,
			AR5K_QUEUE_STATUS(AR5K_TX_QUEUE_ID_BEACON),
			AR5K_QCU_STS_FRMPENDCNT, 0, FALSE);

		if (AR5K_REG_READ_Q(AR5K_QCU_TXE, AR5K_TX_QUEUE_ID_BEACON))
			return (FALSE);
	}

	return (ret);
}

/*
 * Update mib counters (statistics)
 */
void
ath5k_hw_update_mib_counters(struct ath_hal *hal, AR5K_MIB_STATS *statistics)
{
	AR5K_TRACE;
	/* Read-And-Clear */
	statistics->ackrcv_bad += AR5K_REG_READ(AR5K_ACK_FAIL);
	statistics->rts_bad += AR5K_REG_READ(AR5K_RTS_FAIL);
	statistics->rts_good += AR5K_REG_READ(AR5K_RTS_OK);
	statistics->fcs_bad += AR5K_REG_READ(AR5K_FCS_FAIL);
	statistics->beacons += AR5K_REG_READ(AR5K_BEACON_CNT);

	/* Reset profile count registers on 5212*/
	if (hal->ah_version == AR5K_AR5212) {
		AR5K_REG_WRITE(AR5K_PROFCNT_TX, 0);
		AR5K_REG_WRITE(AR5K_PROFCNT_RX, 0);
		AR5K_REG_WRITE(AR5K_PROFCNT_RXCLR, 0);
		AR5K_REG_WRITE(AR5K_PROFCNT_CYCLE, 0);
	}
}

void /*Unimplemented*/
ath5k_hw_proc_mib_event(struct ath_hal *hal, const AR5K_NODE_STATS *stats) 
{
	AR5K_TRACE;
	return;
}

/*
 * ACK/CTS Timeouts
 */

/*
 * Set ACK timeout on PCU
 */
AR5K_BOOL
ath5k_hw_set_ack_timeout(struct ath_hal *hal, u_int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_ACK),
	    hal->ah_turbo) <= timeout)
		return (FALSE);

	AR5K_REG_WRITE_BITS(AR5K_TIME_OUT, AR5K_TIME_OUT_ACK,
	    ath5k_hw_htoclock(timeout, hal->ah_turbo));

	return (TRUE);
}

/*
 * Read the ACK timeout from PCU
 */
u_int
ath5k_hw_get_ack_timeout(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (ath5k_hw_clocktoh(AR5K_REG_MS(AR5K_REG_READ(AR5K_TIME_OUT),
	    AR5K_TIME_OUT_ACK), hal->ah_turbo));
}

/*
 * Set CTS timeout on PCU
 */
AR5K_BOOL
ath5k_hw_set_cts_timeout(struct ath_hal *hal, u_int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_CTS),
	    hal->ah_turbo) <= timeout)
		return (FALSE);

	AR5K_REG_WRITE_BITS(AR5K_TIME_OUT, AR5K_TIME_OUT_CTS,
	    ath5k_hw_htoclock(timeout, hal->ah_turbo));

	return (TRUE);
}

/*
 * Read CTS timeout from PCU
 */
u_int
ath5k_hw_get_cts_timeout(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (ath5k_hw_clocktoh(AR5K_REG_MS(AR5K_REG_READ(AR5K_TIME_OUT),
	    AR5K_TIME_OUT_CTS), hal->ah_turbo));
}

/*
 * Key table (WEP) functions
 */

/*
 * Return which ciphers are supported by hw
 */
AR5K_BOOL
ath5k_hw_is_cipher_supported(struct ath_hal *hal, AR5K_CIPHER cipher)
{
	AR5K_TRACE;
	/*
	 * Only WEP for now
	 */
	if (cipher == AR5K_CIPHER_WEP)
		return (TRUE);

	return (FALSE);
}

/*
 * Get key cache size
 */
u_int32_t
ath5k_hw_get_keycache_size(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (AR5K_KEYCACHE_SIZE);
}

/*
 * Reset encryption key
 */
AR5K_BOOL
ath5k_hw_reset_key(struct ath_hal *hal, u_int16_t entry)
{
	int i;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	for (i = 0; i < AR5K_KEYCACHE_SIZE; i++)
		AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(entry, i), 0);

	/* Set NULL encryption on non-5210*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_WRITE(AR5K_KEYTABLE_TYPE(entry),
				AR5K_KEYTABLE_TYPE_NULL);

	return (FALSE); /*????*/
}

/*
 * Check if a key entry is valid
 */
AR5K_BOOL
ath5k_hw_is_key_valid(struct ath_hal *hal, u_int16_t entry)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/*
	 * Check the validation flag at the end of the entry
	 */
	if (AR5K_REG_READ(AR5K_KEYTABLE_MAC1(entry)) &
	    AR5K_KEYTABLE_VALID)
		return (TRUE);

	return (FALSE);
}

/*
 * Set encryption key
 */
AR5K_BOOL
ath5k_hw_set_key(struct ath_hal *hal, u_int16_t entry,
    const AR5K_KEYVAL *keyval, const u_int8_t *mac, int xor_notused)
{
	int i;
	int keytype;
	__le32 key_v[5];

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	memset(&key_v, 0, sizeof(key_v));

	switch (keyval->wk_len) {
	case AR5K_KEYVAL_LENGTH_40:
		memcpy(&key_v[0], &keyval->wk_key[0], 5);
		keytype = AR5K_KEYTABLE_TYPE_40;
		break;

	case AR5K_KEYVAL_LENGTH_104:
		memcpy(&key_v[0], &keyval->wk_key[0], 6);
		memcpy(&key_v[2], &keyval->wk_key[6], 6);
		memcpy(&key_v[4], &keyval->wk_key[12], 1);
		keytype = AR5K_KEYTABLE_TYPE_104;
		break;

	case AR5K_KEYVAL_LENGTH_128:
		memcpy(&key_v[0], &keyval->wk_key[0], 6);
		memcpy(&key_v[2], &keyval->wk_key[6], 6);
		memcpy(&key_v[4], &keyval->wk_key[12], 4);
		keytype = AR5K_KEYTABLE_TYPE_128;
		break;

	default:
		/* Unsupported key length (not WEP40/104/128) */
		return (FALSE);
	}

	for (i = 0; i < AR5K_ELEMENTS(key_v); i++)
		AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(entry, i),
			       le32_to_cpu(key_v[i]));

	AR5K_REG_WRITE(AR5K_KEYTABLE_TYPE(entry), keytype);

	return (ath5k_hw_set_key_lladdr(hal, entry, mac));
}

AR5K_BOOL
ath5k_hw_set_key_lladdr(struct ath_hal *hal, u_int16_t entry,
    const u_int8_t *mac)
{
	u_int32_t low_id, high_id;

	AR5K_TRACE;
	 /* Invalid entry (key table overflow) */
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/* MAC may be NULL if it's a broadcast key. In this case no need to 
 	 * to compute AR5K_LOW_ID and AR5K_HIGH_ID as we already know it. */
	if(unlikely(mac == NULL)) {
		low_id = 0xffffffff;
		high_id = 0xffff | AR5K_KEYTABLE_VALID;
	}
	else {
		low_id = AR5K_LOW_ID(mac);
		high_id = AR5K_HIGH_ID(mac) | AR5K_KEYTABLE_VALID;
	}

	AR5K_REG_WRITE(AR5K_KEYTABLE_MAC0(entry), low_id);
	AR5K_REG_WRITE(AR5K_KEYTABLE_MAC1(entry), high_id);

	return (TRUE);
}




/********************************************\
Queue Control Unit, DFS Control Unit Functions
\********************************************/

/*
 * Initialize a transmit queue
 */
int
ath5k_hw_setup_tx_queue(struct ath_hal *hal, AR5K_TX_QUEUE queue_type,
			AR5K_TXQ_INFO *queue_info)
{
	u_int queue;
	AR5K_TRACE;

	/*
	 * Get queue by type
	 */
	/*5210 only has 2 queues*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (queue_type) {
			case AR5K_TX_QUEUE_DATA:
				queue = AR5K_TX_QUEUE_ID_NOQCU_DATA;
				break;
			case AR5K_TX_QUEUE_BEACON:
			case AR5K_TX_QUEUE_CAB:
				queue = AR5K_TX_QUEUE_ID_NOQCU_BEACON;
				break;
			default:
				return (-1);
		}
	} else {
		switch (queue_type) {
			case AR5K_TX_QUEUE_DATA:
				for (queue = AR5K_TX_QUEUE_ID_DATA_MIN;
					hal->ah_txq[queue].tqi_type != 
					AR5K_TX_QUEUE_INACTIVE; queue++) {

					if (queue > AR5K_TX_QUEUE_ID_DATA_MAX)
						return (-1);
				}
				break;
			case AR5K_TX_QUEUE_UAPSD:
				queue = AR5K_TX_QUEUE_ID_UAPSD;
				break;
			case AR5K_TX_QUEUE_BEACON:
				queue = AR5K_TX_QUEUE_ID_BEACON;
				break;
			case AR5K_TX_QUEUE_CAB:
				queue = AR5K_TX_QUEUE_ID_CAB;
				break;
			case AR5K_TX_QUEUE_XR_DATA:
				if (hal->ah_version != AR5K_AR5212)
					AR5K_PRINTF("XR data queues only supported in 5212!");
				queue = AR5K_TX_QUEUE_ID_XR_DATA;
				break;
			default:
				return (-1);
		}	
	}

	/*
	 * Setup internal queue structure
	 */
	memset(&hal->ah_txq[queue], 0, sizeof(AR5K_TXQ_INFO));
	hal->ah_txq[queue].tqi_type = queue_type;

	if (queue_info != NULL) {
		queue_info->tqi_type = queue_type;
		if (ath5k_hw_setup_tx_queueprops(hal, queue, queue_info)
		    != TRUE)
			return (-1);
	}
	/*
	 * We use ah_txq_interrupts to hold a temp value for
	 * the Secondary interrupt mask registers on 5211+
	 * check out ath5k_hw_reset_tx_queue
	 */
	AR5K_Q_ENABLE_BITS(hal->ah_txq_interrupts, queue);

	return (queue);
}

/*
 * Setup a transmit queue
 */
AR5K_BOOL
ath5k_hw_setup_tx_queueprops(struct ath_hal *hal, int queue,
				const AR5K_TXQ_INFO *queue_info)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE) 
		return (FALSE);

	memcpy(&hal->ah_txq[queue], queue_info, sizeof(AR5K_TXQ_INFO));

	/*XXX: Is this supported on 5210 ?*/
	if ((queue_info->tqi_type == AR5K_TX_QUEUE_DATA && 
		((queue_info->tqi_subtype == AR5K_WME_AC_VI) ||
		(queue_info->tqi_subtype == AR5K_WME_AC_VO))) ||
		queue_info->tqi_type == AR5K_TX_QUEUE_UAPSD)
		hal->ah_txq[queue].tqi_flags |=
			AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS;

	return (TRUE);
}

/*
 * Get properties for a specific transmit queue
 */
AR5K_BOOL
ath5k_hw_get_tx_queueprops(struct ath_hal *hal, int queue, AR5K_TXQ_INFO *queue_info)
{
	AR5K_TRACE;
	memcpy(queue_info, &hal->ah_txq[queue], sizeof(AR5K_TXQ_INFO));
	return (TRUE);
}

/*
 * Set a transmit queue inactive
 */
AR5K_BOOL
ath5k_hw_release_tx_queue(struct ath_hal *hal, u_int queue)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* This queue will be skipped in further operations */
	hal->ah_txq[queue].tqi_type = AR5K_TX_QUEUE_INACTIVE;
	/*For SIMR setup*/
	AR5K_Q_DISABLE_BITS(hal->ah_txq_interrupts, queue);

	return (FALSE); /*???*/
}

/*
 * Set DFS params for a transmit queue
 */
AR5K_BOOL
ath5k_hw_reset_tx_queue(struct ath_hal *hal, u_int queue)
{
	u_int32_t cw_min, cw_max, retry_lg, retry_sh;
	AR5K_TXQ_INFO *tq = &hal->ah_txq[queue];

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	tq = &hal->ah_txq[queue];

	if (tq->tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return (TRUE);

	if (hal->ah_version == AR5K_AR5210) {
		/* Only handle data queues, others will be ignored */
		if (tq->tqi_type != AR5K_TX_QUEUE_DATA)
			return (TRUE);

		/* Set Slot time */
		AR5K_REG_WRITE(AR5K_SLOT_TIME,
			hal->ah_turbo == TRUE ?
			AR5K_INIT_SLOT_TIME_TURBO :
			AR5K_INIT_SLOT_TIME);
		/* Set ACK_CTS timeout */
		AR5K_REG_WRITE(AR5K_SLOT_TIME,
			hal->ah_turbo == TRUE ?
			AR5K_INIT_ACK_CTS_TIMEOUT_TURBO :
			AR5K_INIT_ACK_CTS_TIMEOUT);
		/* Set Transmit Latency */
		AR5K_REG_WRITE(AR5K_USEC_5210,
			hal->ah_turbo == TRUE ?
			AR5K_INIT_TRANSMIT_LATENCY_TURBO :
			AR5K_INIT_TRANSMIT_LATENCY);
		/* Set IFS0 */
		if (hal->ah_turbo == TRUE ){
			 AR5K_REG_WRITE(AR5K_IFS0,
				((AR5K_INIT_SIFS_TURBO + 					\
				(hal->ah_aifs + tq->tqi_aifs) * AR5K_INIT_SLOT_TIME_TURBO)	\
				<< AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS_TURBO);
		} else {
			AR5K_REG_WRITE(AR5K_IFS0,
				((AR5K_INIT_SIFS + 					\
				(hal->ah_aifs + tq->tqi_aifs) * AR5K_INIT_SLOT_TIME)	\
				<< AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS);
		}
		/* Set IFS1 */
		AR5K_REG_WRITE(AR5K_IFS1,
			hal->ah_turbo == TRUE ?
			AR5K_INIT_PROTO_TIME_CNTRL_TURBO :
			AR5K_INIT_PROTO_TIME_CNTRL);
		/* Set PHY register 0x9844 (??) */
		AR5K_REG_WRITE(AR5K_PHY(17),
			hal->ah_turbo == TRUE ?
			((AR5K_REG_READ(AR5K_PHY(17)) & ~0x7F) | 0x38) :
			((AR5K_REG_READ(AR5K_PHY(17)) & ~0x7F) | 0x1C) );
		/* Set Frame Control Register */
		AR5K_REG_WRITE(AR5K_PHY_FRAME_CTL_5210,
			hal->ah_turbo == TRUE ?
			(AR5K_PHY_FRAME_CTL_INI |	\
			AR5K_PHY_TURBO_MODE |		\
			AR5K_PHY_TURBO_SHORT |		\
			0x2020) :
			(AR5K_PHY_FRAME_CTL_INI | 0x1020));

	}

	/*
	 * Calculate cwmin/max by channel mode
	 */
	cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN;
	cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX;
	hal->ah_aifs = AR5K_TUNE_AIFS;
	/*XR is only supported on 5212*/
	if (IS_CHAN_XR(hal->ah_current_channel)
			&& (hal->ah_version == AR5K_AR5212)) {
		cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN_XR;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX_XR;
		hal->ah_aifs = AR5K_TUNE_AIFS_XR;
	/*B mode is not supported on 5210*/
	} else if (IS_CHAN_B(hal->ah_current_channel)
			&& (hal->ah_version != AR5K_AR5210)) {
		cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN_11B;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX_11B;
		hal->ah_aifs = AR5K_TUNE_AIFS_11B;
	}

	cw_min = 1;
	while (cw_min < hal->ah_cw_min)
		cw_min = (cw_min << 1) | 1;

	cw_min = tq->tqi_cw_min < 0 ?
	    (cw_min >> (-tq->tqi_cw_min)) :
	    ((cw_min << tq->tqi_cw_min) + (1 << tq->tqi_cw_min) - 1);
	cw_max = tq->tqi_cw_max < 0 ?
	    (cw_max >> (-tq->tqi_cw_max)) :
	    ((cw_max << tq->tqi_cw_max) + (1 << tq->tqi_cw_max) - 1);

	/*
	 * Calculate and set retry limits
	 */
	if (hal->ah_software_retry == TRUE) {
		/* XXX Need to test this */
		retry_lg = hal->ah_limit_tx_retries;
		retry_sh = retry_lg =
		    retry_lg > AR5K_DCU_RETRY_LMT_SH_RETRY ?
		    AR5K_DCU_RETRY_LMT_SH_RETRY : retry_lg;
	} else {
		retry_lg = AR5K_INIT_LG_RETRY;
		retry_sh = AR5K_INIT_SH_RETRY;
	}

	/*No QCU/DCU [5210]*/
	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_WRITE(AR5K_NODCU_RETRY_LMT,
			(cw_min << AR5K_NODCU_RETRY_LMT_CW_MIN_S)
			| AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
				AR5K_NODCU_RETRY_LMT_SLG_RETRY)
			| AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
				AR5K_NODCU_RETRY_LMT_SSH_RETRY)
			| AR5K_REG_SM(retry_lg, AR5K_NODCU_RETRY_LMT_LG_RETRY)
			| AR5K_REG_SM(retry_sh, AR5K_NODCU_RETRY_LMT_SH_RETRY));
	} else {
		/*QCU/DCU [5211+]*/
		AR5K_REG_WRITE(AR5K_QUEUE_DFS_RETRY_LIMIT(queue),
			AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
				AR5K_DCU_RETRY_LMT_SLG_RETRY) |
			AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
				AR5K_DCU_RETRY_LMT_SSH_RETRY) |
			AR5K_REG_SM(retry_lg, AR5K_DCU_RETRY_LMT_LG_RETRY) |
			AR5K_REG_SM(retry_sh, AR5K_DCU_RETRY_LMT_SH_RETRY));

	/*===Rest is also for QCU/DCU only [5211+]===*/

		/*
		 * Set initial content window (cw_min/cw_max)
		 * and arbitrated interframe space (aifs)...
		 */
		AR5K_REG_WRITE(AR5K_QUEUE_DFS_LOCAL_IFS(queue),
			AR5K_REG_SM(cw_min, AR5K_DCU_LCL_IFS_CW_MIN) |
			AR5K_REG_SM(cw_max, AR5K_DCU_LCL_IFS_CW_MAX) |
			AR5K_REG_SM(hal->ah_aifs + tq->tqi_aifs,
			AR5K_DCU_LCL_IFS_AIFS));

		/*
		 * Set misc registers
		 */
		AR5K_REG_WRITE(AR5K_QUEUE_MISC(queue),
			AR5K_QCU_MISC_DCU_EARLY);

		if (tq->tqi_cbr_period) {
			AR5K_REG_WRITE(AR5K_QUEUE_CBRCFG(queue),
				AR5K_REG_SM(tq->tqi_cbr_period,
					AR5K_QCU_CBRCFG_INTVAL) |
				AR5K_REG_SM(tq->tqi_cbr_overflow_limit,
					AR5K_QCU_CBRCFG_ORN_THRES));
			AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_CBR);
			if (tq->tqi_cbr_overflow_limit)
				AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_CBR_THRES_ENABLE);
		}

		if (tq->tqi_ready_time) {
			AR5K_REG_WRITE(AR5K_QUEUE_RDYTIMECFG(queue),
				AR5K_REG_SM(tq->tqi_ready_time,
					AR5K_QCU_RDYTIMECFG_INTVAL) |
				AR5K_QCU_RDYTIMECFG_ENABLE);
		}

		if (tq->tqi_burst_time) {
			AR5K_REG_WRITE(AR5K_QUEUE_DFS_CHANNEL_TIME(queue),
				AR5K_REG_SM(tq->tqi_burst_time,
				AR5K_DCU_CHAN_TIME_DUR) |
				AR5K_DCU_CHAN_TIME_ENABLE);

			if (tq->tqi_flags & AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE) {
				AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_TXE);
			}
		}

		if (tq->tqi_flags & AR5K_TXQ_FLAG_BACKOFF_DISABLE) {
			AR5K_REG_WRITE(AR5K_QUEUE_DFS_MISC(queue),
			    AR5K_DCU_MISC_POST_FR_BKOFF_DIS);
		}

		if (tq->tqi_flags & AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE) {
			AR5K_REG_WRITE(AR5K_QUEUE_DFS_MISC(queue),
			    AR5K_DCU_MISC_BACKOFF_FRAG);
		}

		/*
		 * Set registers by queue type
		 */
		switch (tq->tqi_type) {
			case AR5K_TX_QUEUE_BEACON:
				AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_FRSHED_DBA_GT |
					AR5K_QCU_MISC_CBREXP_BCN |
					AR5K_QCU_MISC_BCN_ENABLE);

				AR5K_REG_ENABLE_BITS(AR5K_QUEUE_DFS_MISC(queue),
					(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
					AR5K_DCU_MISC_ARBLOCK_CTL_S) |
					AR5K_DCU_MISC_POST_FR_BKOFF_DIS |
					AR5K_DCU_MISC_BCN_ENABLE);

				AR5K_REG_WRITE(AR5K_QUEUE_RDYTIMECFG(queue),
					((AR5K_TUNE_BEACON_INTERVAL -
					(AR5K_TUNE_SW_BEACON_RESP - 
					AR5K_TUNE_DMA_BEACON_RESP) -
					AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF) * 1024) |
					AR5K_QCU_RDYTIMECFG_ENABLE);
				break;

		case AR5K_TX_QUEUE_CAB:
			AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_DBA_GT |
				AR5K_QCU_MISC_CBREXP |
				AR5K_QCU_MISC_CBREXP_BCN);

			AR5K_REG_ENABLE_BITS(AR5K_QUEUE_DFS_MISC(queue),
				(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
				AR5K_DCU_MISC_ARBLOCK_CTL_S));
			break;

		case AR5K_TX_QUEUE_UAPSD:
			AR5K_REG_ENABLE_BITS(AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_CBREXP);
			break;

		case AR5K_TX_QUEUE_DATA:
		default:
			break;
		}

		/*
		 * Enable tx queue in the secondary interrupt mask registers
		 */
		AR5K_REG_WRITE(AR5K_SIMR0,
			AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_SIMR0_QCU_TXOK) |
			AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_SIMR0_QCU_TXDESC));
		AR5K_REG_WRITE(AR5K_SIMR1,
			AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_SIMR1_QCU_TXERR));
		AR5K_REG_WRITE(AR5K_SIMR2,
			AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_SIMR2_QCU_TXURN));
	}	

	return (TRUE);
}

/*
 * Get number of pending frames
 * for a specific queue [5211+]
 */
u_int32_t
ath5k_hw_num_tx_pending(struct ath_hal *hal, u_int queue) {
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return (FALSE);

	/* XXX: How about AR5K_CFG_TXCNT ? */
	if (hal->ah_version == AR5K_AR5210)
		return (FALSE);

	return (AR5K_QUEUE_STATUS(queue) & AR5K_QCU_STS_FRMPENDCNT);
}

/*
 * Set slot time
 */
AR5K_BOOL
ath5k_hw_set_slot_time(struct ath_hal *hal, u_int slot_time)
{
	AR5K_TRACE;
	if (slot_time < AR5K_SLOT_TIME_9 || slot_time > AR5K_SLOT_TIME_MAX)
		return (FALSE);

	if (hal->ah_version == AR5K_AR5210)
		AR5K_REG_WRITE(AR5K_SLOT_TIME,
			ath5k_hw_htoclock(slot_time, hal->ah_turbo));
	else
		AR5K_REG_WRITE(AR5K_DCU_GBL_IFS_SLOT, slot_time);

	return (TRUE);
}

/*
 * Get slot time
 */
u_int
ath5k_hw_get_slot_time(struct ath_hal *hal)
{
	AR5K_TRACE;
	if (hal->ah_version == AR5K_AR5210)
		return (ath5k_hw_clocktoh(AR5K_REG_READ(AR5K_SLOT_TIME) &
		    0xffff, hal->ah_turbo));
	else
		return (AR5K_REG_READ(AR5K_DCU_GBL_IFS_SLOT) & 0xffff);
}




/******************************\
 Hardware Descriptor Functions
\******************************/

/*
 * TX Descriptor
 */

/*
 * Initialize the 2-word tx descriptor on 5210/5211
 */
AR5K_BOOL
ath5k_hw_setup_2word_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int packet_length, u_int header_length, AR5K_PKT_TYPE type, u_int tx_power,
    u_int tx_rate0, u_int tx_tries0, u_int key_index, u_int antenna_mode,
    u_int flags, u_int rtscts_rate, u_int rtscts_duration)
{
	u_int32_t frame_type;
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc*)&desc->ds_ctl0;

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return (FALSE);

	/* Initialize control descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;

	/* Setup control descriptor */

	/*Verify packet length*/
	if ((tx_desc->tx_control_0 = (packet_length &
			AR5K_2W_TX_DESC_CTL0_FRAME_LEN)) != packet_length)
		return (FALSE);
	/*
	 * Verify header length
	 * XXX: I only found that on 5210 code, does it work on 5211 ?
	 */
	if (hal->ah_version == AR5K_AR5210)
		if ((tx_desc->tx_control_0 = (header_length &
				AR5K_2W_TX_DESC_CTL0_HEADER_LEN)) != header_length)
			return (FALSE);

	/*Diferences between 5210-5211*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (type) {
			case AR5K_PKT_TYPE_BEACON:
			case AR5K_PKT_TYPE_PROBE_RESP:
				frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_NO_DELAY;
			case AR5K_PKT_TYPE_PIFS:
				frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_PIFS;
			default:
				frame_type = type /*<< 2 ?*/;
		}

		tx_desc->tx_control_0 =
			AR5K_REG_SM(frame_type, AR5K_2W_TX_DESC_CTL0_FRAME_TYPE)|
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE);
	} else {
		tx_desc->tx_control_0 |=
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE) |
			AR5K_REG_SM(antenna_mode, AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT);
		tx_desc->tx_control_1 =
			AR5K_REG_SM(type, AR5K_2W_TX_DESC_CTL1_FRAME_TYPE);
	}
#define _TX_FLAGS(_c, _flag)						\
	if (flags & AR5K_TXDESC_##_flag)				\
		tx_desc->tx_control_##_c |=				\
			AR5K_2W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |=
			AR5K_2W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |=
			AR5K_REG_SM(key_index,
			AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS Duration [5210 ?]
	 */
	if ((hal->ah_version == AR5K_AR5210) &&
			(flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA))) {
		tx_desc->tx_control_1 |=
			rtscts_duration & AR5K_2W_TX_DESC_CTL1_RTS_DURATION;
	}

	return (TRUE);
}

/*
 * Initialize the 4-word tx descriptor on 5212
 */
AR5K_BOOL
ath5k_hw_setup_4word_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
	u_int packet_length, u_int header_length, AR5K_PKT_TYPE type, u_int tx_power,
	u_int tx_rate0, u_int tx_tries0, u_int key_index, u_int antenna_mode,
	u_int flags, u_int rtscts_rate, u_int rtscts_duration)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;

	AR5K_TRACE;

	tx_desc = (struct ath5k_hw_4w_tx_desc*)&desc->ds_ctl0;

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return (FALSE);

	/* Initialize status descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;
	tx_desc->tx_control_2 = 0;
	tx_desc->tx_control_3 = 0;

	/* Setup status descriptor */
	if ((tx_desc->tx_control_0 = (packet_length &
			AR5K_4W_TX_DESC_CTL0_FRAME_LEN)) != packet_length)
		return (FALSE);

	tx_desc->tx_control_0 |=
		AR5K_REG_SM(tx_power, AR5K_4W_TX_DESC_CTL0_XMIT_POWER) |
		AR5K_REG_SM(antenna_mode, AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT);
	tx_desc->tx_control_1 =
		AR5K_REG_SM(type, AR5K_4W_TX_DESC_CTL1_FRAME_TYPE);
	tx_desc->tx_control_2 =
		AR5K_REG_SM(tx_tries0 + AR5K_TUNE_HWTXTRIES,
		AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0);
	tx_desc->tx_control_3 =
		tx_rate0 & AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;

#define _TX_FLAGS(_c, _flag)			\
	if (flags & AR5K_TXDESC_##_flag)	\
		tx_desc->tx_control_##_c |=	\
			AR5K_4W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(0, CTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |=
			AR5K_4W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |=
			AR5K_REG_SM(key_index,
		    		AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS
	 */
	if (flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)) {
		if ((flags & AR5K_TXDESC_RTSENA) &&
		    (flags & AR5K_TXDESC_CTSENA))
			return (FALSE);
		tx_desc->tx_control_2 |=
		    rtscts_duration & AR5K_4W_TX_DESC_CTL2_RTS_DURATION;
		tx_desc->tx_control_3 |=
		    AR5K_REG_SM(rtscts_rate,
		    AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE);
	}

	return (TRUE);
}

/*
 * Initialize a 4-word XR tx descriptor on 5212
 */
AR5K_BOOL
ath5k_hw_setup_xr_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int tx_rate1, u_int tx_tries1, u_int tx_rate2, u_int tx_tries2,
    u_int tx_rate3, u_int tx_tries3)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;

	if (hal->ah_version == AR5K_AR5212) {
		tx_desc = (struct ath5k_hw_4w_tx_desc*)&desc->ds_ctl0;

#define _XTX_TRIES(_n)							\
	if (tx_tries##_n) {						\
		tx_desc->tx_control_2 |=				\
		    AR5K_REG_SM(tx_tries##_n,				\
		    AR5K_4W_TX_DESC_CTL2_XMIT_TRIES##_n);		\
		tx_desc->tx_control_3 |=				\
		    AR5K_REG_SM(tx_rate##_n,				\
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE##_n);		\
	}

		_XTX_TRIES(1);
		_XTX_TRIES(2);
		_XTX_TRIES(3);

#undef _XTX_TRIES

		return (TRUE);
	}

	return(FALSE);
}

/*
 * Fill the 2-word tx descriptor on 5210/5211
 */
AR5K_BOOL
ath5k_hw_fill_2word_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int segment_length, AR5K_BOOL first_segment, AR5K_BOOL last_segment, const struct ath_desc *last_desc)
{
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc*)&desc->ds_ctl0;

	/* Clear status descriptor */
	memset(desc->ds_hw, 0, sizeof(desc->ds_hw));

	/* Validate segment length and initialize the descriptor */
	if ((tx_desc->tx_control_1 = (segment_length &
			AR5K_2W_TX_DESC_CTL1_BUF_LEN)) != segment_length)
		return (FALSE);

	if (first_segment != TRUE)
		tx_desc->tx_control_0 &= ~AR5K_2W_TX_DESC_CTL0_FRAME_LEN;

	if (last_segment != TRUE)
		tx_desc->tx_control_1 |= AR5K_2W_TX_DESC_CTL1_MORE;

	return (TRUE);
}

/*
 * Fill the 4-word tx descriptor on 5212
 * XXX: Added an argument *last_desc -need revision
 */
AR5K_BOOL
ath5k_hw_fill_4word_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
	u_int segment_length, AR5K_BOOL first_segment, AR5K_BOOL last_segment,
	const struct ath_desc *last_desc)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;
	struct ath5k_hw_tx_status *tx_status;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc*)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status*)&desc->ds_hw[2];

	/* Clear status descriptor */
	memset(tx_status, 0, sizeof(struct ath5k_hw_tx_status));

	/* Validate segment length and initialize the descriptor */
	if ((tx_desc->tx_control_1 = (segment_length &
			AR5K_4W_TX_DESC_CTL1_BUF_LEN)) != segment_length)
		return (FALSE);

	if (first_segment != TRUE)
		tx_desc->tx_control_0 &= ~AR5K_4W_TX_DESC_CTL0_FRAME_LEN;

	if (last_segment != TRUE)
		tx_desc->tx_control_1 |= AR5K_4W_TX_DESC_CTL1_MORE;

	return (TRUE);
}

/*
 * Proccess the tx status descriptor on 5210/5211
 */
AR5K_STATUS
ath5k_hw_proc_2word_tx_status(struct ath_hal *hal, struct ath_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc*)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status*)&desc->ds_hw[0];

	/* No frame has been send or error */
	if ((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	/*TODO: desc->ds_us.tx.ts_virtcol + test*/
	desc->ds_us.tx.ts_seqnum =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = 1;
	desc->ds_us.tx.ts_status = 0;
	desc->ds_us.tx.ts_rate =
	    AR5K_REG_MS(tx_desc->tx_control_0,
	    AR5K_2W_TX_DESC_CTL0_XMIT_RATE);

	if ((tx_status->tx_status_0 &
			AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0) {
		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return (AR5K_OK);
}

/*
 * Proccess a tx descriptor on 5212
 */
AR5K_STATUS
ath5k_hw_proc_4word_tx_status(struct ath_hal *hal, struct ath_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_4w_tx_desc *tx_desc;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc*)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status*)&desc->ds_hw[2];

	/* No frame has been send or error */
	if ((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	desc->ds_us.tx.ts_seqnum =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = (tx_status->tx_status_1 &
	    AR5K_DESC_TX_STATUS1_XMIT_ANTENNA) ? 2 : 1;
	desc->ds_us.tx.ts_status = 0;

	switch (AR5K_REG_MS(tx_status->tx_status_1,
		AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX)) {
	case 0:
		desc->ds_us.tx.ts_rate = tx_desc->tx_control_3 &
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;
		break;
	case 1:
		desc->ds_us.tx.ts_rate =
		    AR5K_REG_MS(tx_desc->tx_control_3,
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE1);
		desc->ds_us.tx.ts_longretry +=
		    AR5K_REG_MS(tx_desc->tx_control_2,
		    AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1);
		break;
	case 2:
		desc->ds_us.tx.ts_rate =
		    AR5K_REG_MS(tx_desc->tx_control_3,
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE2);
		desc->ds_us.tx.ts_longretry +=
		    AR5K_REG_MS(tx_desc->tx_control_2,
		    AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2);
		break;
	case 3:
		desc->ds_us.tx.ts_rate =
		    AR5K_REG_MS(tx_desc->tx_control_3,
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE3);
		desc->ds_us.tx.ts_longretry +=
		    AR5K_REG_MS(tx_desc->tx_control_2,
		    AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3);
		break;
	}

	if ((tx_status->tx_status_0 &
	    AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0) {
		if (tx_status->tx_status_0 &
		    AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 &
		    AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 &
		    AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return (AR5K_OK);
}

/*
 * RX Descriptor
 */

/*
 * Initialize an rx descriptor
 */
AR5K_BOOL
ath5k_hw_setup_rx_desc(struct ath_hal *hal, struct ath_desc *desc,
			u_int32_t size, u_int flags)
{
	struct ath5k_rx_desc *rx_desc;

	AR5K_TRACE;
	rx_desc = (struct ath5k_rx_desc*)&desc->ds_ctl0;

	/*
	 *Clear ds_hw 
	 * If we don't clean the status descriptor,
	 * while scanning we get too many results, 
	 * most of them virtual, after some secs 
	 * of scanning system hangs. M.F.
	*/
	memset(desc->ds_hw, 0, sizeof(desc->ds_hw));

	/*Initialize rx descriptor*/
	rx_desc->rx_control_0 = 0;
	rx_desc->rx_control_1 = 0;

	/*Setup descriptor*/
	if ((rx_desc->rx_control_1 = (size &
	    AR5K_DESC_RX_CTL1_BUF_LEN)) != size)
		return (FALSE);

	if (flags & AR5K_RXDESC_INTREQ)
		rx_desc->rx_control_1 |= AR5K_DESC_RX_CTL1_INTREQ;

	return (TRUE);
}

/*
 * Proccess the rx status descriptor on 5210/5211
 */
AR5K_STATUS
ath5k_hw_proc_old_rx_status(struct ath_hal *hal, struct ath_desc *desc,
			u_int32_t phys_addr, struct ath_desc *next)
{
	struct ath5k_hw_old_rx_status *rx_status;

	rx_status = (struct ath5k_hw_old_rx_status*)&desc->ds_hw[0];

	/* No frame received / not ready */
	if ((rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
	    AR5K_OLD_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi =
	    AR5K_REG_MS(rx_status->rx_status_0,
	    	AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate =
	    AR5K_REG_MS(rx_status->rx_status_0,
	    	AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
	    AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
	    AR5K_OLD_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp =
	    AR5K_REG_MS(rx_status->rx_status_1,
	    	AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 &
	    AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix =
		    AR5K_REG_MS(rx_status->rx_status_1,
		    	AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 &
	AR5K_OLD_RX_DESC_STATUS1_FRAME_RECEIVE_OK) == 0) {
		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_FIFO_OVERRUN)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_FIFO;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_status->rx_status_1,
					AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR);
		}

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;
	}

	return (AR5K_OK);
}

/*
 * Proccess the rx status descriptor on 5212
 */
AR5K_STATUS
ath5k_hw_proc_new_rx_status(struct ath_hal *hal, struct ath_desc *desc,
			u_int32_t phys_addr, struct ath_desc *next)
{
	struct ath5k_hw_new_rx_status *rx_status;
	struct ath5k_hw_rx_error *rx_err;

	AR5K_TRACE;
	rx_status = (struct ath5k_hw_new_rx_status*)&desc->ds_hw[0];

	/* Overlay on error */
	rx_err = (struct ath5k_hw_rx_error*)&desc->ds_hw[0];

	/* No frame received / not ready */
	if ((rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi =
		AR5K_REG_MS(rx_status->rx_status_0,
			AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate =
		AR5K_REG_MS(rx_status->rx_status_0,
			AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp =
		AR5K_REG_MS(rx_status->rx_status_1,
			AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 &
			AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix =
			AR5K_REG_MS(rx_status->rx_status_1,
				AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 &
			AR5K_NEW_RX_DESC_STATUS1_FRAME_RECEIVE_OK) == 0) {
		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_err->rx_error_1,
					AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE);
		}

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_MIC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_MIC;
	}

	return (AR5K_OK);
}




/****************\
  GPIO Functions
\****************/

/*
 * Set led state
 */
void
ath5k_hw_set_ledstate(struct ath_hal *hal, AR5K_LED_STATE state)
{
	u_int32_t led;
	/*5210 has different led mode handling*/
	u_int32_t led_5210;

	AR5K_TRACE;

	/*Reset led status*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_DISABLE_BITS(AR5K_PCICFG,
			AR5K_PCICFG_LEDMODE |  AR5K_PCICFG_LED);
	else
		AR5K_REG_DISABLE_BITS(AR5K_PCICFG,
				AR5K_PCICFG_LED);

	/*
	 * Some blinking values, define at your wish
	 */
	switch (state) {
	case AR5K_LED_SCAN:
	case AR5K_LED_AUTH:
		led = AR5K_PCICFG_LEDMODE_PROP |
			AR5K_PCICFG_LED_PEND;
		led_5210 = AR5K_PCICFG_LED_PEND|
			AR5K_PCICFG_LED_BCTL;
		break;

	case AR5K_LED_INIT:
		led = AR5K_PCICFG_LEDMODE_PROP |
			AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;

	case AR5K_LED_ASSOC:
	case AR5K_LED_RUN:
		led = AR5K_PCICFG_LEDMODE_PROP |
			AR5K_PCICFG_LED_ASSOC;
		led_5210 = AR5K_PCICFG_LED_ASSOC;
		break;

	default:
		led = AR5K_PCICFG_LEDMODE_PROM |
			AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;
	}

	/*Write new status to the register*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, led);
	else
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, led_5210);
}

/*
 * Set GPIO outputs
 */
AR5K_BOOL
ath5k_hw_set_gpio_output(struct ath_hal *hal, u_int32_t gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return (FALSE);

	AR5K_REG_WRITE(AR5K_GPIOCR,
	    (AR5K_REG_READ(AR5K_GPIOCR) &~ AR5K_GPIOCR_OUT(gpio))
	    | AR5K_GPIOCR_OUT(gpio));

	return (TRUE);
}

/*
 * Set GPIO inputs
 */
AR5K_BOOL
ath5k_hw_set_gpio_input(struct ath_hal *hal, u_int32_t gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return (FALSE);

	AR5K_REG_WRITE(AR5K_GPIOCR,
	    (AR5K_REG_READ(AR5K_GPIOCR) &~ AR5K_GPIOCR_OUT(gpio))
	    | AR5K_GPIOCR_IN(gpio));

	return (TRUE);
}

/*
 * Get GPIO state
 */
u_int32_t
ath5k_hw_get_gpio(struct ath_hal *hal, u_int32_t gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return (0xffffffff);

	/* GPIO input magic */
	return (((AR5K_REG_READ(AR5K_GPIODI) &
	    AR5K_GPIODI_M) >> gpio) & 0x1);
}

/*
 * Set GPIO state
 */
AR5K_BOOL
ath5k_hw_set_gpio(struct ath_hal *hal, u_int32_t gpio, u_int32_t val)
{
	u_int32_t data;
	AR5K_TRACE;

	if (gpio > AR5K_NUM_GPIO)
		return (FALSE);

	/* GPIO output magic */
	data =  AR5K_REG_READ(AR5K_GPIODO);

	data &= ~(1 << gpio);
	data |= (val&1) << gpio;

	AR5K_REG_WRITE(AR5K_GPIODO, data);

	return (TRUE);
}

/*
 * Initialize the GPIO interrupt (RFKill switch)
 */
void
ath5k_hw_set_gpio_intr(struct ath_hal *hal, u_int gpio,
    u_int32_t interrupt_level)
{
	u_int32_t data;

	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return;

	/*
	 * Set the GPIO interrupt
	 */
	data = (AR5K_REG_READ(AR5K_GPIOCR) &
	    ~(AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_SELH |
	    AR5K_GPIOCR_INT_ENA | AR5K_GPIOCR_OUT(gpio))) |
	    (AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_ENA);

	AR5K_REG_WRITE(AR5K_GPIOCR,
	    interrupt_level ? data : (data | AR5K_GPIOCR_INT_SELH));

	hal->ah_imr |= AR5K_IMR_GPIO;

	/* Enable GPIO interrupts */
	AR5K_REG_ENABLE_BITS(AR5K_PIMR, AR5K_IMR_GPIO);
}




/*********************************\
 Regulatory Domain/Channels Setup
\*********************************/

/*
 * Following 2 functions come from net80211 
 * TODO: These do not belong here, they have nothing
 * to do with hw. I left them here temporarily for
 * combatibility.
 * M.F.
 */

/*
 * Convert MHz frequency to IEEE channel number.
 */
u_int
ath_hal_mhz2ieee(u_int freq, u_int flags)
{
	if (flags & CHANNEL_2GHZ) {	/* 2GHz band */
		if (freq == 2484)		/* Japan */
			return 14;
		/* don't number non-IEEE channels unless we do channel tests */
		if ((freq >= 2412) && (freq < 2484))
			return (freq - 2407) / 5;
		if (CHAN_DEBUG == 1) /* 15-26 */
			return ((freq - 2512)/20) + 15;
		return 0;
	} else if (flags & CHANNEL_5GHZ)	{	/* 5Ghz band */
		/* don't number non-IEEE channels unless we do channel tests */
		if (((freq >= 5150) && (freq <= 5825))|| CHAN_DEBUG == 1)
			return (freq - 5000) / 5;
		return 0;
	} else
		/* something is fishy, don't do anything */
		return 0;
}

/*
 * Convert IEEE channel number to MHz frequency.
 */
u_int
ath_hal_ieee2mhz(u_int chan, u_int flags)
{
	if (flags & CHANNEL_2GHZ) {	/* 2GHz band */
		if (chan == 14)
			return 2484;
		if (chan < 14)
			return 2407 + chan * 5;
		else
			return 2512 + ((chan - 15) * 20);
	} else if (flags & CHANNEL_5GHZ) /* 5Ghz band */
		return 5000 + (chan * 5);
	else {					/* either, guess */
		if (chan == 14)
			return 2484;
		if (chan < 14)			/* 0-13 */
			return 2407 + chan * 5;
		if (chan < 27)			/* 15-26 */
			return 2512 + ((chan - 15) * 20);
		return 5000 + (chan * 5);
	}
}

/*
 * Check if a channel is supported
 */
AR5K_BOOL
ath5k_check_channel(struct ath_hal *hal, u_int16_t freq, u_int flags)
{
	/* Check if the channel is in our supported range */
	if (flags & CHANNEL_2GHZ) {
		if ((freq >= hal->ah_capabilities.cap_range.range_2ghz_min) &&
		    (freq <= hal->ah_capabilities.cap_range.range_2ghz_max))
			return (TRUE);
	} else if (flags & CHANNEL_5GHZ) 
		if ((freq >= hal->ah_capabilities.cap_range.range_5ghz_min) &&
		    (freq <= hal->ah_capabilities.cap_range.range_5ghz_max))
			return (TRUE);

	return (FALSE);
}

/*
 * Initialize channels array
 * TODO: Do this in the driver, only check_channel is hw related
 * also left here temporarily for combatibility.
 */
AR5K_BOOL
ath_hal_init_channels(struct ath_hal *hal, AR5K_CHANNEL *channels,
    u_int max_channels, u_int *channels_size, AR5K_CTRY_CODE country, u_int16_t mode,
    AR5K_BOOL outdoor, AR5K_BOOL extended)
{
	u_int i, c;
	u_int32_t domain_current;
	u_int domain_5ghz, domain_2ghz;
	AR5K_CHANNEL *all_channels;
	AR5K_CTRY_CODE country_current;

	/* Allocate and initialize channel array */
	if ((all_channels = kmalloc(sizeof(AR5K_CHANNEL) * max_channels,
	    GFP_KERNEL)) == NULL)
		return (FALSE);
	memset(all_channels, 0, sizeof(AR5K_CHANNEL) * max_channels);

	i = c = 0;
	domain_current = hal->ah_regdomain;
	hal->ah_country_code = country;
	country_current = hal->ah_country_code;

	/*
	 * In debugging mode, enable all channels supported by the chipset
	 */
	if (domain_current == DMN_DEFAULT || CHAN_DEBUG == 1) {
		int min, max, freq;
		u_int flags;

		min = 1; /* 2GHz channel 1 -2412Mhz */
		max = 26;/* 2GHz channel 26 (non-ieee) -2732Mhz */

		flags = CHANNEL_B | CHANNEL_G;

 debugchan:
		for (i = min; (i <= max) && (c < max_channels); i++) {
			freq = ath_hal_ieee2mhz(i, flags);
			if (ath5k_check_channel(hal, freq, flags) == FALSE)
				continue;
			all_channels[c].freq = freq;
			all_channels[c].channel_flags = flags;
			c++;
		}

		/* If is there to protect from infinite loop */
		if (flags & CHANNEL_2GHZ) {
/* ath_hal_mhz2ieee returns 1 for IEEE80211_CHANNELS_5GHZ_MIN 
for loop starts from 1 and all channels are marked as 5GHz M.F.*/
//			min = ath_hal_mhz2ieee(IEEE80211_CHANNELS_5GHZ_MIN,
//			    CHANNEL_5GHZ);
/* Continue from where we stoped, skip last 2GHz channel */
			min = max + 1;
			max = ath_hal_mhz2ieee(IEEE80211_CHANNELS_5GHZ_MAX,
						CHANNEL_5GHZ);
			flags = CHANNEL_A | CHANNEL_T | CHANNEL_XR;
			goto debugchan;
		}

		goto done;
	}

	domain_5ghz = ieee80211_regdomain2flag(domain_current,
	    IEEE80211_CHANNELS_5GHZ_MIN);
	domain_2ghz = ieee80211_regdomain2flag(domain_current,
	    IEEE80211_CHANNELS_2GHZ_MIN);

	/*
	 * Create channel list based on chipset capabilities, regulation domain
	 * and mode. 5GHz...
	 */
	for (i = 0; (hal->ah_capabilities.cap_range.range_5ghz_max > 0) &&
		 (i < AR5K_ELEMENTS(ath5k_5ghz_channels)) &&
		 (c < max_channels); i++) {
		/* Check if channel is supported by the chipset */
		if (ath5k_check_channel(hal,
		    ath5k_5ghz_channels[i].rc_channel,
		    CHANNEL_5GHZ) == FALSE)
			continue;

		/* Match regulation domain */
		if ((IEEE80211_DMN(ath5k_5ghz_channels[i].rc_domain) &
			IEEE80211_DMN(domain_5ghz)) == 0)
			continue;

		/* Match modes */
		if (ath5k_5ghz_channels[i].rc_mode & CHANNEL_TURBO)
			all_channels[c].channel_flags = CHANNEL_T;
		else if (ath5k_5ghz_channels[i].rc_mode &
		    CHANNEL_OFDM)
			all_channels[c].channel_flags = CHANNEL_A;
		else
			continue;

		/* Write channel and increment counter */
		all_channels[c++].freq = ath5k_5ghz_channels[i].rc_channel;
	}

	/*
	 * ...and 2GHz.
	 */
	for (i = 0; (hal->ah_capabilities.cap_range.range_2ghz_max > 0) &&
			(i < AR5K_ELEMENTS(ath5k_2ghz_channels)) &&
			(c < max_channels); i++) {
		
		/* Check if channel is supported by the chipset */
		if (ath5k_check_channel(hal,
		    ath5k_2ghz_channels[i].rc_channel,
		    CHANNEL_2GHZ) == FALSE)
			continue;

		/* Match regulation domain */
		if ((IEEE80211_DMN(ath5k_2ghz_channels[i].rc_domain) &
			IEEE80211_DMN(domain_2ghz)) == 0)
			continue;

		/* Match modes */
		if ((hal->ah_capabilities.cap_mode & AR5K_MODE_11B) &&
		   (ath5k_2ghz_channels[i].rc_mode & CHANNEL_CCK))
			all_channels[c].channel_flags = CHANNEL_B;

		if ((hal->ah_capabilities.cap_mode & AR5K_MODE_11G) &&
		   (ath5k_2ghz_channels[i].rc_mode & CHANNEL_OFDM)) {
			all_channels[c].channel_flags |= CHANNEL_G;
/*			if (ath5k_2ghz_channels[i].rc_mode &
			    CHANNEL_TURBO)
				all_channels[c].channel_flags |= CHANNEL_TG;*/
		}

		/* Write channel and increment counter */
		all_channels[c++].freq = ath5k_2ghz_channels[i].rc_channel;
	}

 done:
	memcpy(channels, all_channels, sizeof(AR5K_CHANNEL) * max_channels);
	*channels_size = c;
	kfree(all_channels);
	return (TRUE);
}

/*
 * Regdomain stuff, these also don't belong here etc
 */

u_int16_t
ath5k_regdomain_from_ieee(ieee80211_regdomain_t ieee)
{
	u_int32_t regdomain = (u_int32_t)ieee;

	/*
	 * Use the default regulation domain if the value is empty
	 * or not supported by the net80211 regulation code.
	 */
	if (ieee80211_regdomain2flag(regdomain,
	    IEEE80211_CHANNELS_5GHZ_MIN) == DMN_DEBUG)
		return ((u_int16_t)AR5K_TUNE_REGDOMAIN);

	/* It is supported, just return the value */
	return (regdomain);
}

ieee80211_regdomain_t
ath5k_regdomain_to_ieee(u_int16_t regdomain)
{
	ieee80211_regdomain_t ieee = (ieee80211_regdomain_t)regdomain;

	return (ieee);
}

u_int16_t
ath5k_get_regdomain(struct ath_hal *hal)
{
	u_int16_t regdomain;
	ieee80211_regdomain_t ieee_regdomain;
#ifdef COUNTRYCODE
	u_int16_t code;
#endif

	ath5k_hw_eeprom_regulation_domain(hal, FALSE, &ieee_regdomain);
	hal->ah_capabilities.cap_regdomain.reg_hw = ieee_regdomain;

#ifdef COUNTRYCODE
	/*
	 * Get the regulation domain by country code. This will ignore
	 * the settings found in the EEPROM.
	 */
	code = ieee80211_name2countrycode(COUNTRYCODE);
	ieee_regdomain = ieee80211_countrycode2regdomain(code);
#endif

	regdomain = ath5k_regdomain_from_ieee(ieee_regdomain);
	hal->ah_capabilities.cap_regdomain.reg_current = regdomain;

	return (regdomain);
}

u_int16_t /*TODO:Get rid of this*/
ath5k_hw_get_regdomain(struct ath_hal *hal)
{
	AR5K_TRACE;
	return (ath5k_get_regdomain(hal));
}




/*************************\
  PHY/RF access functions
\*************************/

/*
 * Set a channel on the radio chip
 */
AR5K_BOOL
ath5k_hw_channel(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	AR5K_BOOL ret;

	/*
	 * Check bounds supported by the PHY
	 * (don't care about regulation restrictions at this point)
	 */
	if ((channel->freq < hal->ah_capabilities.cap_range.range_2ghz_min ||
	    channel->freq > hal->ah_capabilities.cap_range.range_2ghz_max) &&
	    (channel->freq < hal->ah_capabilities.cap_range.range_5ghz_min ||
	    channel->freq > hal->ah_capabilities.cap_range.range_5ghz_max)) {
		AR5K_PRINTF("channel out of supported range (%u MHz)\n",
		    channel->freq);
		return (FALSE);
	}

	/*
	 * Set the channel and wait
	 */
	if (hal->ah_radio == AR5K_RF5110)
		ret = ath5k_hw_rf5110_channel(hal, channel);
	else if (hal->ah_radio == AR5K_RF5111)
		ret = ath5k_hw_rf5111_channel(hal, channel);
	else
		ret = ath5k_hw_rf5112_channel(hal, channel);

	if (ret == FALSE)
		return (ret);

	hal->ah_current_channel.freq = channel->freq;
	hal->ah_current_channel.channel_flags = channel->channel_flags;
	hal->ah_turbo = channel->channel_flags == CHANNEL_T ?
	    TRUE : FALSE;

	return (TRUE);
}

/*
 * Convertion needed for RF5110
 */
u_int32_t
ath5k_hw_rf5110_chan2athchan(AR5K_CHANNEL *channel)
{
	u_int32_t athchan;

	/*
	 * Convert IEEE channel/MHz to an internal channel value used
	 * by the AR5210 chipset. This has not been verified with
	 * newer chipsets like the AR5212A who have a completely
	 * different RF/PHY part.
	 */
	athchan = (ath5k_hw_bitswap((ath_hal_mhz2ieee(channel->freq,
	    channel->channel_flags) - 24) / 2, 5) << 1) |
	    (1 << 6) | 0x1;

	return (athchan);
}

/*
 * Set channel on RF5110
 */
AR5K_BOOL
ath5k_hw_rf5110_channel(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	u_int32_t data;

	/*
	 * Set the channel and wait
	 */
	data = ath5k_hw_rf5110_chan2athchan(channel);
	AR5K_REG_WRITE(AR5K_RF_BUFFER, data);
	AR5K_REG_WRITE(AR5K_RF_BUFFER_CONTROL_0, 0);
	udelay(1000);

	return (TRUE);
}

/*
 * Convertion needed for 5111
 */
AR5K_BOOL
ath5k_hw_rf5111_chan2athchan(u_int ieee, struct ath5k_athchan_2ghz *athchan)
{
	int channel;

	/* Cast this value to catch negative channel numbers (>= -19) */ 
	channel = (int)ieee;

	/*
	 * Map 2GHz IEEE channel to 5GHz Atheros channel
	 */
	if (channel <= 13) {
		athchan->a2_athchan = 115 + channel;
		athchan->a2_flags = 0x46;
	} else if (channel == 14) {
		athchan->a2_athchan = 124;
		athchan->a2_flags = 0x44;
	} else if (channel >= 15 && channel <= 26) {
		athchan->a2_athchan = ((channel - 14) * 4) + 132;
		athchan->a2_flags = 0x46;
	} else
		return (FALSE);

	return (TRUE);
}

/*
 * Set channel on 5111
 */
AR5K_BOOL
ath5k_hw_rf5111_channel(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	u_int ieee_channel, ath_channel;
	u_int32_t data0, data1, clock;
	struct ath5k_athchan_2ghz ath_channel_2ghz;

	/*
	 * Set the channel on the RF5111 radio
	 */
	data0 = data1 = 0;
	ath_channel = ieee_channel = ath_hal_mhz2ieee(channel->freq,
	    channel->channel_flags);

	if (channel->channel_flags & CHANNEL_2GHZ) {
		/* Map 2GHz channel to 5GHz Atheros channel ID */
		if (ath5k_hw_rf5111_chan2athchan(ieee_channel,
			&ath_channel_2ghz) == FALSE)
			return (FALSE);

		ath_channel = ath_channel_2ghz.a2_athchan;
		data0 = ((ath5k_hw_bitswap(ath_channel_2ghz.a2_flags, 8) & 0xff)
		    << 5) | (1 << 4);
	}

	if (ath_channel < 145 || !(ath_channel & 1)) {
		clock = 1;
		data1 = ((ath5k_hw_bitswap(ath_channel - 24, 8) & 0xff) << 2)
		    | (clock << 1) | (1 << 10) | 1;
	} else {
		clock = 0;
		data1 = ((ath5k_hw_bitswap((ath_channel - 24) / 2, 8) & 0xff) << 2)
		    | (clock << 1) | (1 << 10) | 1;
	}

	AR5K_REG_WRITE(AR5K_RF_BUFFER, (data1 & 0xff) | ((data0 & 0xff) << 8));
	AR5K_REG_WRITE(AR5K_RF_BUFFER_CONTROL_3, ((data1 >> 8) & 0xff) | (data0 & 0xff00));

	return (TRUE);
}

/*
 * Set channel on 5112
 */
AR5K_BOOL
ath5k_hw_rf5112_channel(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	u_int32_t data, data0, data1, data2;
	u_int16_t c;

	data = data0 = data1 = data2 = 0;
	c = channel->freq;

	/*
	 * Set the channel on the RF5112 or newer
	 */
	if (c < 4800) {
		if (!((c - 2224) % 5)) {
			data0 = ((2 * (c - 704)) - 3040) / 10;
			data1 = 1;
		} else if (!((c - 2192) % 5)) {
			data0 = ((2 * (c - 672)) - 3040) / 10;
			data1 = 0;
		} else
			return (FALSE);

		data0 = ath5k_hw_bitswap((data0 << 2) & 0xff, 8);
	} else {
		if (!(c % 20) && c >= 5120) {
			data0 = ath5k_hw_bitswap(((c - 4800) / 20 << 2), 8);
			data2 = ath5k_hw_bitswap(3, 2);
		} else if (!(c % 10)) {
			data0 = ath5k_hw_bitswap(((c - 4800) / 10 << 1), 8);
			data2 = ath5k_hw_bitswap(2, 2);
		} else if (!(c % 5)) {
			data0 = ath5k_hw_bitswap((c - 4800) / 5, 8);
			data2 = ath5k_hw_bitswap(1, 2);
		} else
			return (FALSE);
	}

	data = (data0 << 4) | (data1 << 1) | (data2 << 2) | 0x1001;

	AR5K_REG_WRITE(AR5K_RF_BUFFER, data & 0xff);
	AR5K_REG_WRITE(AR5K_RF_BUFFER_CONTROL_5, (data >> 8) & 0x7f);

	return (TRUE);
}

/*
 * Perform a PHY calibration
 */
AR5K_BOOL
ath5k_hw_phy_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel){

	AR5K_BOOL ret;

	if (hal->ah_radio == AR5K_RF5110)
		ret = ath5k_hw_rf5110_calibrate(hal,channel);
	else
		ret = ath5k_hw_rf511x_calibrate(hal,channel);

	return (ret);
}
/*
 * Perform a PHY calibration on RF5110
 */
AR5K_BOOL
ath5k_hw_rf5110_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	AR5K_BOOL ret = TRUE;
	u_int32_t phy_sig, phy_agc, phy_sat, beacon, noise_floor;
	u_int i;

#define AGC_DISABLE	{			\
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGC,	\
		AR5K_PHY_AGC_DISABLE);		\
	udelay(10);				\
}

#define AGC_ENABLE	{			\
	AR5K_REG_DISABLE_BITS(AR5K_PHY_AGC,	\
	    AR5K_PHY_AGC_DISABLE);		\
}

	/*
	 * Disable beacons and RX/TX queues, wait
	 */
	AR5K_REG_ENABLE_BITS(AR5K_DIAG_SW_5210,
	    AR5K_DIAG_SW_DIS_TX | AR5K_DIAG_SW_DIS_RX_5210);
	beacon = AR5K_REG_READ(AR5K_BEACON_5210);
	AR5K_REG_WRITE(AR5K_BEACON_5210, beacon & ~AR5K_BEACON_ENABLE);

	udelay(2300);

	/*
	 * Set the channel (with AGC turned off)
	 */
	AGC_DISABLE;
	ret = ath5k_hw_channel(hal, channel);

	/*
	 * Activate PHY and wait
	 */
	AR5K_REG_WRITE(AR5K_PHY_ACT, AR5K_PHY_ACT_ENABLE);
	udelay(1000);

	AGC_ENABLE;

	if (ret == FALSE)
		return (ret);

	/*
	 * Calibrate the radio chip
	 */

	/* Remember normal state */
	phy_sig = AR5K_REG_READ(AR5K_PHY_SIG);
	phy_agc = AR5K_REG_READ(AR5K_PHY_AGCCOARSE);
	phy_sat = AR5K_REG_READ(AR5K_PHY_ADCSAT);

	/* Update radio registers */
	AR5K_REG_WRITE(AR5K_PHY_SIG,
		(phy_sig & ~(AR5K_PHY_SIG_FIRPWR)) |
		AR5K_REG_SM(-1, AR5K_PHY_SIG_FIRPWR));

	AR5K_REG_WRITE(AR5K_PHY_AGCCOARSE,
		(phy_agc & ~(AR5K_PHY_AGCCOARSE_HI |
			AR5K_PHY_AGCCOARSE_LO)) |
		AR5K_REG_SM(-1, AR5K_PHY_AGCCOARSE_HI) |
		AR5K_REG_SM(-127, AR5K_PHY_AGCCOARSE_LO));

	AR5K_REG_WRITE(AR5K_PHY_ADCSAT,
		(phy_sat & ~(AR5K_PHY_ADCSAT_ICNT |
			AR5K_PHY_ADCSAT_THR)) |
		AR5K_REG_SM(2, AR5K_PHY_ADCSAT_ICNT) |
		AR5K_REG_SM(12, AR5K_PHY_ADCSAT_THR));

	udelay(20);

	AGC_DISABLE;
	AR5K_REG_WRITE(AR5K_PHY_RFSTG, AR5K_PHY_RFSTG_DISABLE);
	AGC_ENABLE;

	udelay(1000);

	/*
	 * Enable calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_CAL);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
		AR5K_PHY_AGCCTL_CAL, 0, FALSE) == FALSE) {
		AR5K_PRINTF("calibration timeout (%uMHz)\n",
			channel->freq);
		ret = FALSE;
	}

	/* Reset to normal state */
	AR5K_REG_WRITE(AR5K_PHY_SIG, phy_sig);
	AR5K_REG_WRITE(AR5K_PHY_AGCCOARSE, phy_agc);
	AR5K_REG_WRITE(AR5K_PHY_ADCSAT, phy_sat);

	if (ret == FALSE)
		return (FALSE);

	/*
 	 * Enable noise floor calibration and wait until completion
 	 */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_NF);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
		AR5K_PHY_AGCCTL_NF, 0, FALSE) == FALSE) {
		AR5K_PRINTF("noise floor calibration timeout (%uMHz)\n",
				channel->freq);
		return (FALSE);
	}

	/* Wait until the noise floor is calibrated */
	for (i = 20; i > 0; i--) {
		udelay(1000);
		noise_floor = AR5K_REG_READ(AR5K_PHY_NF);

		if (AR5K_PHY_NF_RVAL(noise_floor) &
				AR5K_PHY_NF_ACTIVE)
			noise_floor = AR5K_PHY_NF_AVAL(noise_floor);
	
		if (noise_floor <= AR5K_TUNE_NOISE_FLOOR)
			break;
	}

	if (noise_floor > AR5K_TUNE_NOISE_FLOOR) {
		AR5K_PRINTF("noise floor calibration failed (%uMHz)\n",
			channel->freq);
		return (FALSE);
	}


	/*
	 * Re-enable RX/TX and beacons
	 */
	AR5K_REG_DISABLE_BITS(AR5K_DIAG_SW_5210,
		AR5K_DIAG_SW_DIS_TX | AR5K_DIAG_SW_DIS_RX_5210);
	AR5K_REG_WRITE(AR5K_BEACON_5210, beacon);

#undef AGC_ENABLE
#undef AGC_DISABLE

	return (TRUE);
}

/*
 * Perform a PHY calibration on RF5111/5112
 * -Fix BPSK/QAM Constellation (I/Q correction)
 * -Calculate Noise Floor
 */
AR5K_BOOL
ath5k_hw_rf511x_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	u_int32_t i_pwr, q_pwr;
	int32_t iq_corr, i_coff, i_coffd, q_coff, q_coffd;
	AR5K_TRACE;

	if (hal->ah_calibration == FALSE ||
	    AR5K_REG_READ(AR5K_PHY_IQ) & AR5K_PHY_IQ_RUN)
		goto done;

	hal->ah_calibration = FALSE;

	iq_corr = AR5K_REG_READ(AR5K_PHY_IQRES_CAL_CORR);
	i_pwr = AR5K_REG_READ(AR5K_PHY_IQRES_CAL_PWR_I);
	q_pwr = AR5K_REG_READ(AR5K_PHY_IQRES_CAL_PWR_Q);
	i_coffd = ((i_pwr >> 1) + (q_pwr >> 1)) >> 7;
	q_coffd = q_pwr >> 6;

	if (i_coffd == 0 || q_coffd == 0)
		goto done;

	i_coff = ((-iq_corr) / i_coffd) & 0x3f;
	q_coff = (((int32_t)i_pwr / q_coffd) - 64) & 0x1f;

	/* Commit new IQ value */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_IQ,
		AR5K_PHY_IQ_CORR_ENABLE |
		((u_int32_t)q_coff) |
		((u_int32_t)i_coff << AR5K_PHY_IQ_CORR_Q_I_COFF_S));

 done:
	/* Start noise floor calibration */
	AR5K_REG_ENABLE_BITS(AR5K_PHY_AGCCTL,
		AR5K_PHY_AGCCTL_NF);

	/* Request RF gain */
	if (channel->channel_flags & CHANNEL_5GHZ) {
		AR5K_REG_WRITE(AR5K_PHY_PAPD_PROBE,
			AR5K_REG_SM(hal->ah_txpower.txp_max,
			AR5K_PHY_PAPD_PROBE_TXPOWER) |
			AR5K_PHY_PAPD_PROBE_TX_NEXT);
		hal->ah_rf_gain = AR5K_RFGAIN_READ_REQUESTED;
	}

	return (TRUE);
}

AR5K_BOOL
ath5k_hw_phy_disable(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	AR5K_REG_WRITE(AR5K_PHY_ACT, AR5K_PHY_ACT_DISABLE);
	return (TRUE);
}

void /*TODO:Boundary check*/
ath5k_hw_set_def_antenna(struct ath_hal *hal, u_int ant)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_WRITE(AR5K_DEFAULT_ANTENNA, ant);

	return;
}

u_int
ath5k_hw_get_def_antenna(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	if (hal->ah_version != AR5K_AR5210)
		return AR5K_REG_READ(AR5K_DEFAULT_ANTENNA);

	return (FALSE); /*XXX: What do we return for 5210 ?*/
}

/*
 * Used to modify RF Banks before writing them to AR5K_RF_BUFFER
 */
u_int
ath5k_hw_rfregs_op(u_int32_t *rf, u_int32_t offset, u_int32_t reg, u_int32_t bits,
    u_int32_t first, u_int32_t col, AR5K_BOOL set)
{
	u_int32_t mask, entry, last, data, shift, position;
	int32_t left;
	int i;

	data = 0;

	if (rf == NULL)
		/* should not happen */
		return (0);

	if (!(col <= 3 && bits <= 32 && first + bits <= 319)) {
		AR5K_PRINTF("invalid values at offset %u\n", offset);
		return (0);
	}

	entry = ((first - 1) / 8) + offset;
	position = (first - 1) % 8;

	if (set == TRUE)
		data = ath5k_hw_bitswap(reg, bits);

	for (i = shift = 0, left = bits; left > 0; position = 0, entry++, i++) {
		last = (position + left > 8) ? 8 : position + left;
		mask = (((1 << last) - 1) ^ ((1 << position) - 1)) <<
		    (col * 8);

		if (set == TRUE) {
			rf[entry] &= ~mask;
			rf[entry] |= ((data << position) << (col * 8)) & mask;
			data >>= (8 - position);
		} else {
			data = (((rf[entry] & mask) >> (col * 8)) >>
			    position) << shift;
			shift += last - position;
		}

		left -= 8 - position;
	}

	data = set == TRUE ? 1 : ath5k_hw_bitswap(data, bits);

	return (data);
}

u_int32_t
ath5k_hw_rfregs_gainf_corr(struct ath_hal *hal)
{
	u_int32_t mix, step;
	u_int32_t *rf;

	if (hal->ah_rf_banks == NULL)
		return (0);

	rf = hal->ah_rf_banks;
	hal->ah_gain.g_f_corr = 0;

	if (ath5k_hw_rfregs_op(rf, hal->ah_offset[7], 0, 1, 36, 0, FALSE) != 1)
		return (0);

	step = ath5k_hw_rfregs_op(rf, hal->ah_offset[7], 0, 4, 32, 0, FALSE);
	mix = hal->ah_gain.g_step->gos_param[0];

	switch (mix) {
	case 3:
		hal->ah_gain.g_f_corr = step * 2;
		break;
	case 2:
		hal->ah_gain.g_f_corr = (step - 5) * 2;
		break;
	case 1:
		hal->ah_gain.g_f_corr = step;
		break;
	default:
		hal->ah_gain.g_f_corr = 0;
		break;
	}

	return (hal->ah_gain.g_f_corr);
}

AR5K_BOOL
ath5k_hw_rfregs_gain_readback(struct ath_hal *hal)
{
	u_int32_t step, mix, level[4];
	u_int32_t *rf;

	if (hal->ah_rf_banks == NULL)
		return (0);

	rf = hal->ah_rf_banks;

	if (hal->ah_radio == AR5K_RF5111) {
		step = ath5k_hw_rfregs_op(rf, hal->ah_offset[7],
		    0, 6, 37, 0, FALSE);
		level[0] = 0;
		level[1] = (step == 0x3f) ? 0x32 : step + 4;
		level[2] = (step != 0x3f) ? 0x40 : level[0];
		level[3] = level[2] + 0x32;

		hal->ah_gain.g_high = level[3] -
		    (step == 0x3f ? AR5K_GAIN_DYN_ADJUST_HI_MARGIN : -5);
		hal->ah_gain.g_low = level[0] +
		    (step == 0x3f ? AR5K_GAIN_DYN_ADJUST_LO_MARGIN : 0);
	} else {
		mix = ath5k_hw_rfregs_op(rf, hal->ah_offset[7],
		    0, 1, 36, 0, FALSE);
		level[0] = level[2] = 0;

		if (mix == 1) {
			level[1] = level[3] = 83;
		} else {
			level[1] = level[3] = 107;
			hal->ah_gain.g_high = 55;
		}
	}

	return ((hal->ah_gain.g_current >= level[0] &&
	    hal->ah_gain.g_current <= level[1]) ||
	    (hal->ah_gain.g_current >= level[2] &&
	    hal->ah_gain.g_current <= level[3]));
}

int32_t
ath5k_hw_rfregs_gain_adjust(struct ath_hal *hal)
{
	int ret = 0;
	const struct ath5k_gain_opt *go;

	go = hal->ah_radio == AR5K_RF5111 ?
	    &rf5111_gain_opt : &rf5112_gain_opt;

	hal->ah_gain.g_step = &go->go_step[hal->ah_gain.g_step_idx];

	if (hal->ah_gain.g_current >= hal->ah_gain.g_high) {
		if (hal->ah_gain.g_step_idx == 0)
			return (-1);
		for (hal->ah_gain.g_target = hal->ah_gain.g_current;
		    hal->ah_gain.g_target >=  hal->ah_gain.g_high &&
		    hal->ah_gain.g_step_idx > 0;
		    hal->ah_gain.g_step =
		    &go->go_step[hal->ah_gain.g_step_idx]) {
			hal->ah_gain.g_target -= 2 *
			    (go->go_step[--(hal->ah_gain.g_step_idx)].gos_gain -
			    hal->ah_gain.g_step->gos_gain);
		}

		ret = 1;
		goto done;
	}

	if (hal->ah_gain.g_current <= hal->ah_gain.g_low) {
		if (hal->ah_gain.g_step_idx == (go->go_steps_count - 1))
			return (-2);
		for (hal->ah_gain.g_target = hal->ah_gain.g_current;
		    hal->ah_gain.g_target <=  hal->ah_gain.g_low &&
		    hal->ah_gain.g_step_idx < (go->go_steps_count - 1);
		    hal->ah_gain.g_step =
		    &go->go_step[hal->ah_gain.g_step_idx]) {
			hal->ah_gain.g_target -= 2 *
			    (go->go_step[++(hal->ah_gain.g_step_idx)].gos_gain -
			    hal->ah_gain.g_step->gos_gain);
		}

		ret = 2;
		goto done;
	}

 done:
#ifdef AR5K_DEBUG
	AR5K_PRINTF("ret %d, gain step %u, current gain %u, target gain %u\n",
	    ret,
	    hal->ah_gain.g_step_idx,
	    hal->ah_gain.g_current,
	    hal->ah_gain.g_target);
#endif

	return (ret);
}

/*
 * Initialize RF
 */
AR5K_BOOL
ath5k_hw_rfregs(struct ath_hal *hal, AR5K_CHANNEL *channel, u_int mode)
{
	ath5k_rfgain_t *func = NULL;
	AR5K_BOOL ret;

	if (hal->ah_radio == AR5K_RF5111) {
		hal->ah_rf_banks_size = sizeof(rf5111_rf);
		func = ath5k_hw_rf5111_rfregs;
	} else if (hal->ah_radio == AR5K_RF5112) {
		if (hal->ah_radio_5ghz_revision >= AR5K_SREV_RAD_5112A)
			hal->ah_rf_banks_size = sizeof(rf5112a_rf);
		else
		hal->ah_rf_banks_size = sizeof(rf5112_rf);
		func = ath5k_hw_rf5112_rfregs;
	} else
		return (FALSE);

	if (hal->ah_rf_banks == NULL) {
		/* XXX do extra checks? */
		if ((hal->ah_rf_banks = kmalloc(hal->ah_rf_banks_size,
		    GFP_KERNEL)) == NULL) {
			AR5K_PRINT("out of memory\n");
			return (FALSE);
		}
	}

	ret = (func)(hal, channel, mode);

	if (ret == TRUE)
		hal->ah_rf_gain = AR5K_RFGAIN_INACTIVE;

	return (ret);
}

/*
 * Read EEPROM Calibration data, modify RF Banks and Initialize RF5111
 */
AR5K_BOOL
ath5k_hw_rf5111_rfregs(struct ath_hal *hal, AR5K_CHANNEL *channel, u_int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	const u_int rf_size = AR5K_ELEMENTS(rf5111_rf);
	u_int32_t *rf;
	int i, obdb = -1, bank = -1;
	u_int32_t ee_mode;

	AR5K_ASSERT_ENTRY(mode, AR5K_INI_VAL_MAX);

	rf = hal->ah_rf_banks;

	/* Copy values to modify them */
	for (i = 0; i < rf_size; i++) {
		if (rf5111_rf[i].rf_bank >=
		    AR5K_RF5111_INI_RF_MAX_BANKS) {
			AR5K_PRINT("invalid bank\n");
			return (FALSE);
		}

		if (bank != rf5111_rf[i].rf_bank) {
			bank = rf5111_rf[i].rf_bank;
			hal->ah_offset[bank] = i;
		}

		rf[i] = rf5111_rf[i].rf_value[mode];
	}

	/* Modify bank 0 */
	if (channel->channel_flags & CHANNEL_2GHZ) {
		if (channel->channel_flags & CHANNEL_B)
			ee_mode = AR5K_EEPROM_MODE_11B;
		else
			ee_mode = AR5K_EEPROM_MODE_11G;
		obdb = 0;

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[0],
			ee->ee_ob[ee_mode][obdb], 3, 119, 0, TRUE))
			return (FALSE);

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[0],
			ee->ee_ob[ee_mode][obdb], 3, 122, 0, TRUE))
			return (FALSE);

		obdb = 1;
	/* Modify bank 6 */
	} else {
		/* For 11a, Turbo and XR */
		ee_mode = AR5K_EEPROM_MODE_11A;
		obdb = channel->freq >= 5725 ? 3 :
		    (channel->freq >= 5500 ? 2 :
			(channel->freq >= 5260 ? 1 :
			    (channel->freq > 4000 ? 0 : -1)));

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_pwd_84, 1, 51, 3, TRUE))
			return (FALSE);

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_pwd_90, 1, 45, 3, TRUE))
			return (FALSE);
	}

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
		!ee->ee_xpd[ee_mode], 1, 95, 0, TRUE))
		return (FALSE);

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
		ee->ee_x_gain[ee_mode], 4, 96, 0, TRUE))
		return (FALSE);

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
		obdb >= 0 ? ee->ee_ob[ee_mode][obdb] : 0, 3, 104, 0, TRUE))
		return (FALSE);

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
		obdb >= 0 ? ee->ee_db[ee_mode][obdb] : 0, 3, 107, 0, TRUE))
		return (FALSE);

	/* Modify bank 7 */
	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[7],
		ee->ee_i_gain[ee_mode], 6, 29, 0, TRUE))
		return (FALSE);

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[7],
		ee->ee_xpd[ee_mode], 1, 4, 0, TRUE))
		return (FALSE);

	/* Write RF values */
	for (i = 0; i < rf_size; i++) {
		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE(rf5111_rf[i].rf_register, rf[i]);
	}

	return (TRUE);
}

/*
 * Read EEPROM Calibration data, modify RF Banks and Initialize RF5112
 */
AR5K_BOOL
ath5k_hw_rf5112_rfregs(struct ath_hal *hal, AR5K_CHANNEL *channel, u_int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int rf_size;
	u_int32_t *rf;
	int i, obdb = -1, bank = -1;
	u_int32_t ee_mode;
	const struct ath5k_ini_rf *rf_ini;

	AR5K_ASSERT_ENTRY(mode, AR5K_INI_VAL_MAX);

	rf = hal->ah_rf_banks;

	if (hal->ah_radio_5ghz_revision >= AR5K_SREV_RAD_5112A) {
		rf_ini = rf5112a_rf;
		rf_size = AR5K_ELEMENTS(rf5112a_rf);
	} else {
		rf_ini = rf5112_rf;
		rf_size = AR5K_ELEMENTS(rf5112_rf);
	}

	/* Copy values to modify them */
	for (i = 0; i < rf_size; i++) {
		if (rf_ini[i].rf_bank >=
		    AR5K_RF5112_INI_RF_MAX_BANKS) {
			AR5K_PRINT("invalid bank\n");
			return (FALSE);
		}

		if (bank != rf_ini[i].rf_bank) {
			bank = rf_ini[i].rf_bank;
			hal->ah_offset[bank] = i;
		}

		rf[i] = rf_ini[i].rf_value[mode];
	}

	/* Modify bank 6 */
	if (channel->channel_flags & CHANNEL_2GHZ) {
		if (channel->channel_flags & CHANNEL_B)
			ee_mode = AR5K_EEPROM_MODE_11B;
		else
			ee_mode = AR5K_EEPROM_MODE_11G;
		obdb = 0;

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_ob[ee_mode][obdb], 3, 287, 0, TRUE))
			return (FALSE);

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_ob[ee_mode][obdb], 3, 290, 0, TRUE))
			return (FALSE);
	} else {
		/* For 11a, Turbo and XR */
		ee_mode = AR5K_EEPROM_MODE_11A;
		obdb = channel->freq >= 5725 ? 3 :
		    (channel->freq >= 5500 ? 2 :
			(channel->freq >= 5260 ? 1 :
			    (channel->freq > 4000 ? 0 : -1)));

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_ob[ee_mode][obdb], 3, 279, 0, TRUE))
			return (FALSE);

		if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
			ee->ee_ob[ee_mode][obdb], 3, 282, 0, TRUE))
			return (FALSE);
	}

#ifdef notyet
	ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
	    ee->ee_x_gain[ee_mode], 2, 270, 0, TRUE);
	ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
	    ee->ee_x_gain[ee_mode], 2, 257, 0, TRUE);
#endif

	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[6],
		ee->ee_xpd[ee_mode], 1, 302, 0, TRUE))
		return (FALSE);

	/* Modify bank 7 */
	if (!ath5k_hw_rfregs_op(rf, hal->ah_offset[7],
		ee->ee_i_gain[ee_mode], 6, 14, 0, TRUE))
		return (FALSE);

	/* Write RF values */
	for (i = 0; i < rf_size; i++)
		AR5K_REG_WRITE(rf_ini[i].rf_register, rf[i]);

	return (TRUE);
}

AR5K_BOOL
ath5k_hw_rfgain(struct ath_hal *hal, u_int freq)
{
	int i;
	struct ath5k_ini_rfgain *ath5k_rfg;

	switch (hal->ah_radio) {
	case AR5K_RF5111:
		ath5k_rfg = (struct ath5k_ini_rfgain*) &rf5111_ini_rfgain;
		break;
	case AR5K_RF5112:
		ath5k_rfg = (struct ath5k_ini_rfgain*) &rf5112_ini_rfgain;
		break;
	default:
		return (FALSE);
	}

	switch (freq) {
	case AR5K_INI_RFGAIN_2GHZ:
	case AR5K_INI_RFGAIN_5GHZ:
		break;
	default:
		return (FALSE);
	}

	for (i = 0; i < AR5K_ELEMENTS(ath5k_rfg); i++) {
		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE((u_int32_t)ath5k_rfg[i].rfg_register,
		    ath5k_rfg[i].rfg_value[freq]);
	}

	return (TRUE);
}

AR5K_RFGAIN
ath5k_hw_get_rf_gain(struct ath_hal *hal)
{
	u_int32_t data, type;

	AR5K_TRACE;

	if ((hal->ah_rf_banks == NULL) || (!hal->ah_gain.g_active) 
			|| (hal->ah_version <= AR5K_AR5211))
		return (AR5K_RFGAIN_INACTIVE);

	if (hal->ah_rf_gain != AR5K_RFGAIN_READ_REQUESTED)
		goto done;

	data = AR5K_REG_READ(AR5K_PHY_PAPD_PROBE);

	if (!(data & AR5K_PHY_PAPD_PROBE_TX_NEXT)) {
		hal->ah_gain.g_current =
			data >> AR5K_PHY_PAPD_PROBE_GAINF_S;
		type = AR5K_REG_MS(data, AR5K_PHY_PAPD_PROBE_TYPE);

		if (type == AR5K_PHY_PAPD_PROBE_TYPE_CCK)
			hal->ah_gain.g_current += AR5K_GAIN_CCK_PROBE_CORR;

		if (hal->ah_radio == AR5K_RF5112) {
			ath5k_hw_rfregs_gainf_corr(hal);
			hal->ah_gain.g_current =
				hal->ah_gain.g_current >= hal->ah_gain.g_f_corr ?
				(hal->ah_gain.g_current - hal->ah_gain.g_f_corr) :
				0;
		}

		if (ath5k_hw_rfregs_gain_readback(hal) &&
		AR5K_GAIN_CHECK_ADJUST(&hal->ah_gain) &&
		ath5k_hw_rfregs_gain_adjust(hal))
			hal->ah_rf_gain = AR5K_RFGAIN_NEED_CHANGE;
	}

 done:
	return (hal->ah_rf_gain);
}

/*
 * TX power setup
 */

/*
 * Initialize the tx power table (not fully implemented)
 */
void
ath5k_hw_txpower_table(struct ath_hal *hal, AR5K_CHANNEL *channel, int16_t max_power)
{
	u_int16_t txpower, *rates;
	int i, min, max, n;

	rates = hal->ah_txpower.txp_rates;

	txpower = AR5K_TUNE_DEFAULT_TXPOWER * 2;
	if (max_power > txpower)
		txpower = max_power > AR5K_TUNE_MAX_TXPOWER ?
		    AR5K_TUNE_MAX_TXPOWER : max_power;

	for (i = 0; i < AR5K_MAX_RATES; i++)
		rates[i] = txpower;

	/* XXX setup target powers by rate */

	hal->ah_txpower.txp_min = rates[7];
	hal->ah_txpower.txp_max = rates[0];
	hal->ah_txpower.txp_ofdm = rates[0];

	/* Calculate the power table */
	n = AR5K_ELEMENTS(hal->ah_txpower.txp_pcdac);
	min = AR5K_EEPROM_PCDAC_START;
	max = AR5K_EEPROM_PCDAC_STOP;
	for (i = 0; i < n; i += AR5K_EEPROM_PCDAC_STEP)
		hal->ah_txpower.txp_pcdac[i] =
#ifdef notyet
		min + ((i * (max - min)) / n);
#else
		min;
#endif
}

/*
 * Set transmition power
 */
AR5K_BOOL /*O.K. - txpower_table is unimplemented so this doesn't work*/
ath5k_hw_txpower(struct ath_hal *hal, AR5K_CHANNEL *channel, u_int txpower)
{
	AR5K_BOOL tpc = hal->ah_txpower.txp_tpc;
	int i;

	AR5K_TRACE;
	if (txpower > AR5K_TUNE_MAX_TXPOWER) {
		AR5K_PRINTF("invalid tx power: %u\n", txpower);
		return (FALSE);
	}

	/* Reset TX power values */
	memset(&hal->ah_txpower, 0, sizeof(hal->ah_txpower));
	hal->ah_txpower.txp_tpc = tpc;

	/* Initialize TX power table */
	ath5k_hw_txpower_table(hal, channel, txpower);

	/* 
	 * Write TX power values
	 */
	for (i = 0; i < (AR5K_EEPROM_POWER_TABLE_SIZE / 2); i++) {
		AR5K_REG_WRITE(AR5K_PHY_PCDAC_TXPOWER(i),
		      ((((hal->ah_txpower.txp_pcdac[(i << 1) + 1] << 8) | 0xff) & 0xffff) << 16) 
		    | ((((hal->ah_txpower.txp_pcdac[(i << 1)    ] << 8) | 0xff) & 0xffff)      )
		    );
	}

	AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE1,
	    AR5K_TXPOWER_OFDM(3, 24) | AR5K_TXPOWER_OFDM(2, 16)
	    | AR5K_TXPOWER_OFDM(1, 8) | AR5K_TXPOWER_OFDM(0, 0));

	AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE2,
	    AR5K_TXPOWER_OFDM(7, 24) | AR5K_TXPOWER_OFDM(6, 16)
	    | AR5K_TXPOWER_OFDM(5, 8) | AR5K_TXPOWER_OFDM(4, 0));

	AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE3,
	    AR5K_TXPOWER_CCK(10, 24) | AR5K_TXPOWER_CCK(9, 16)
	    | AR5K_TXPOWER_CCK(15, 8) | AR5K_TXPOWER_CCK(8, 0));

	AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE4,
	    AR5K_TXPOWER_CCK(14, 24) | AR5K_TXPOWER_CCK(13, 16)
	    | AR5K_TXPOWER_CCK(12, 8) | AR5K_TXPOWER_CCK(11, 0));

	if (hal->ah_txpower.txp_tpc == TRUE) {
		AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE_MAX,
		    AR5K_PHY_TXPOWER_RATE_MAX_TPC_ENABLE |
		    AR5K_TUNE_MAX_TXPOWER);
	} else {
		AR5K_REG_WRITE(AR5K_PHY_TXPOWER_RATE_MAX,
		    AR5K_PHY_TXPOWER_RATE_MAX |
		    AR5K_TUNE_MAX_TXPOWER);
	}

	return (TRUE);
}

AR5K_BOOL
ath5k_hw_set_txpower_limit(struct ath_hal *hal, u_int power)
{
	/*Just a try M.F.*/
	AR5K_CHANNEL *channel = &hal->ah_current_channel;

	AR5K_TRACE;
	AR5K_PRINTF("changing txpower to %d\n",power);
	return (ath5k_hw_txpower(hal, channel, power));
}




/****************\
  Misc functions
\****************/

void /*O.K.*/
ath5k_hw_dump_state(struct ath_hal *hal)
{
#ifdef AR5K_DEBUG
#define AR5K_PRINT_REGISTER(_x)						\
	AR5K_PRINTF("(%s: %08x) ", #_x, AR5K_REG_READ(AR5K_##_x));

	AR5K_PRINT("MAC registers:\n");
	AR5K_PRINT_REGISTER(CR);
	AR5K_PRINT_REGISTER(CFG);
	AR5K_PRINT_REGISTER(IER);
	AR5K_PRINT_REGISTER(TXCFG);
	AR5K_PRINT_REGISTER(RXCFG);
	AR5K_PRINT_REGISTER(MIBC);
	AR5K_PRINT_REGISTER(TOPS);
	AR5K_PRINT_REGISTER(RXNOFRM);
	AR5K_PRINT_REGISTER(RPGTO);
	AR5K_PRINT_REGISTER(RFCNT);
	AR5K_PRINT_REGISTER(MISC);
	AR5K_PRINT_REGISTER(PISR);
	AR5K_PRINT_REGISTER(SISR0);
	AR5K_PRINT_REGISTER(SISR1);
	AR5K_PRINT_REGISTER(SISR3);
	AR5K_PRINT_REGISTER(SISR4);
	AR5K_PRINT_REGISTER(DCM_ADDR);
	AR5K_PRINT_REGISTER(DCM_DATA);
	AR5K_PRINT_REGISTER(DCCFG);
	AR5K_PRINT_REGISTER(CCFG);
	AR5K_PRINT_REGISTER(CCFG_CUP);
	AR5K_PRINT_REGISTER(CPC0);
	AR5K_PRINT_REGISTER(CPC1);
	AR5K_PRINT_REGISTER(CPC2);
	AR5K_PRINT_REGISTER(CPCORN);
	AR5K_PRINT_REGISTER(QCU_TXE);
	AR5K_PRINT_REGISTER(QCU_TXD);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SIFS);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SLOT);
	AR5K_PRINT_REGISTER(DCU_FP);
	AR5K_PRINT_REGISTER(DCU_TXP);
	AR5K_PRINT_REGISTER(DCU_TX_FILTER);
	AR5K_PRINT_REGISTER(RC);
	AR5K_PRINT_REGISTER(SCR);
	AR5K_PRINT_REGISTER(INTPEND);
	AR5K_PRINT_REGISTER(PCICFG);
	AR5K_PRINT_REGISTER(GPIOCR);
	AR5K_PRINT_REGISTER(GPIODO);
	AR5K_PRINT_REGISTER(SREV);
	AR5K_PRINT_REGISTER(EEPROM_BASE);
	AR5K_PRINT_REGISTER(EEPROM_DATA);
	AR5K_PRINT_REGISTER(EEPROM_CMD);
	AR5K_PRINT_REGISTER(EEPROM_CFG);
	AR5K_PRINT_REGISTER(PCU_MIN);
	AR5K_PRINT_REGISTER(STA_ID0);
	AR5K_PRINT_REGISTER(STA_ID1);
	AR5K_PRINT_REGISTER(BSS_ID0);
	AR5K_PRINT_REGISTER(SLOT_TIME);
	AR5K_PRINT_REGISTER(TIME_OUT);
	AR5K_PRINT_REGISTER(RSSI_THR);
	AR5K_PRINT_REGISTER(BEACON);
	AR5K_PRINT_REGISTER(CFP_PERIOD);
	AR5K_PRINT_REGISTER(TIMER0);
	AR5K_PRINT_REGISTER(TIMER2);
	AR5K_PRINT_REGISTER(TIMER3);
	AR5K_PRINT_REGISTER(CFP_DUR);
	AR5K_PRINT_REGISTER(MCAST_FILTER0);
	AR5K_PRINT_REGISTER(MCAST_FILTER1);
	AR5K_PRINT_REGISTER(DIAG_SW);
	AR5K_PRINT_REGISTER(TSF_U32);
	AR5K_PRINT_REGISTER(ADDAC_TEST);
	AR5K_PRINT_REGISTER(DEFAULT_ANTENNA);
	AR5K_PRINT_REGISTER(LAST_TSTP);
	AR5K_PRINT_REGISTER(NAV);
	AR5K_PRINT_REGISTER(RTS_OK);
	AR5K_PRINT_REGISTER(ACK_FAIL);
	AR5K_PRINT_REGISTER(FCS_FAIL);
	AR5K_PRINT_REGISTER(BEACON_CNT);
	AR5K_PRINT_REGISTER(TSF_PARM);
	AR5K_PRINT_REGISTER(RATE_DUR_0);
	AR5K_PRINT_REGISTER(KEYTABLE_0);
	AR5K_PRINT("\n");

	AR5K_PRINT("PHY registers:\n");
	AR5K_PRINT_REGISTER(PHY_TURBO);
	AR5K_PRINT_REGISTER(PHY_AGC);
	AR5K_PRINT_REGISTER(PHY_TIMING_3);
	AR5K_PRINT_REGISTER(PHY_CHIP_ID);
	AR5K_PRINT_REGISTER(PHY_AGCCTL);
	AR5K_PRINT_REGISTER(PHY_NF);
	AR5K_PRINT_REGISTER(PHY_SCR);
	AR5K_PRINT_REGISTER(PHY_SLMT);
	AR5K_PRINT_REGISTER(PHY_SCAL);
	AR5K_PRINT_REGISTER(PHY_RX_DELAY);
	AR5K_PRINT_REGISTER(PHY_IQ);
	AR5K_PRINT_REGISTER(PHY_PAPD_PROBE);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE1);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE2);
	AR5K_PRINT_REGISTER(PHY_FC);
	AR5K_PRINT_REGISTER(PHY_RADAR);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_0);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_1);
	AR5K_PRINT("\n");
#endif
}

AR5K_BOOL /*what about VEOL cap ?*/
ath5k_hw_has_veol(struct ath_hal *hal)
{
	return (TRUE);
}

void /*Unimplemented*/
ath5k_hw_get_tx_inter_queue(struct ath_hal *hal, u_int32_t *i)
{
	AR5K_TRACE;
	/* XXX */
	return;
}

void /*Added AR5K_NODE_STATS argument*/
ath5k_hw_set_rx_signal_monitor(struct ath_hal *hal, const AR5K_NODE_STATS *stats)
{
	AR5K_TRACE;
	/* Signal state monitoring is not yet supported */
}

AR5K_BOOL /*Added arguments*/
ath5k_hw_get_diag_state(struct ath_hal *hal, int request, 
const void *args, u_int32_t argsize, void **result, u_int32_t *resultsize)
{
	AR5K_TRACE;
	/*
	 * We'll ignore this right now. This seems to be some kind of an obscure
	 * debugging interface for the binary-only HAL.
	 */
	return (FALSE);
}

AR5K_BOOL /*TODO:Is this realy needed ? We have get_isr that will return 0xfff.. on removal*/
ath5k_hw_detect_card_present(struct ath_hal *hal)
{
	u_int16_t magic;
	AR5K_TRACE;
	/*
	 * Checking the EEPROM's magic value could be an indication
	 * if the card is still present. I didn't find another suitable
	 * way to do this.
	 */
	if (ath5k_hw_eeprom_read(hal, AR5K_EEPROM_MAGIC, &magic) != 0)
		return (FALSE);

	return (magic == AR5K_EEPROM_MAGIC_VALUE ? TRUE : FALSE);
}

AR5K_STATUS
ath5k_hw_get_capability(struct ath_hal *hal, AR5K_CAPABILITY_TYPE cap_type,
			   u_int32_t capability, u_int32_t *result) 
{
	AR5K_TRACE;

	switch (cap_type) {
	case AR5K_CAP_REG_DMN:
		if (result){
			*result = ath5k_get_regdomain(hal);
			goto yes;
		}
	case AR5K_CAP_CIPHER: 
		switch (capability) {
			case AR5K_CIPHER_WEP: 
				goto yes;
			default:
				goto no;
		}
	case AR5K_CAP_NUM_TXQUEUES: 
		if (result) {
			if (hal->ah_version == AR5K_AR5210)
				*result = AR5K_NUM_TX_QUEUES_NOQCU;
			else
				*result = AR5K_NUM_TX_QUEUES;
			goto yes;
		}
	case AR5K_CAP_VEOL:
		goto yes;
	case AR5K_CAP_COMPRESSION:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_BURST:
		goto yes;
	case AR5K_CAP_TPC:
		goto yes;
	case AR5K_CAP_BSSIDMASK:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_XR:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	default: 
		goto no;
	}

 no:
	return (AR5K_EINVAL);
 yes:
	return AR5K_OK;
	
}

AR5K_BOOL
ath5k_hw_set_capability(struct ath_hal *hal, AR5K_CAPABILITY_TYPE cap_type,
			   u_int32_t capability, u_int32_t setting, AR5K_STATUS *status) 
{

	AR5K_TRACE;
	if (status)
		*status = AR5K_OK;

	return (FALSE);
}

AR5K_BOOL
ath5k_hw_query_pspoll_support(struct ath_hal *hal)
{
	AR5K_TRACE;
	if (hal->ah_version == AR5K_AR5210)
		return(TRUE);

	return (FALSE);
}

AR5K_BOOL
ath5k_hw_init_pspoll(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*
	 * Not used
	 */
	return (FALSE);
}

AR5K_BOOL
ath5k_hw_enable_pspoll(struct ath_hal *hal, u_int8_t *bssid,
    u_int16_t assoc_id)
{
	AR5K_TRACE;
	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_DISABLE_BITS(AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL |
			AR5K_STA_ID1_DEFAULT_ANTENNA);
		return (TRUE);
	}

	return (FALSE);
}

AR5K_BOOL
ath5k_hw_disable_pspoll(struct ath_hal *hal)
{
	AR5K_TRACE;
	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL |
			AR5K_STA_ID1_DEFAULT_ANTENNA);
		return (TRUE);
	}

	return (FALSE);
}

const char *
ath5k_hw_get_part_name(enum ath5k_srev_type type, u_int32_t val)
{
	struct ath5k_srev_name names[] = AR5K_SREV_NAME;
	const char *name = "xxxxx";
	int i;

	for (i = 0; i < AR5K_ELEMENTS(names); i++) {
		if (names[i].sr_type != type ||
		    names[i].sr_val == AR5K_SREV_UNKNOWN)
			continue;
		if ((val & 0xff) < names[i + 1].sr_val) {
			name = names[i].sr_name;
			break;
		}
	}

	return (name);
}

void /*O.K. - TODO: Implement this in if_ath.c (ath_intr)*/
ath5k_radar_alert(struct ath_hal *hal)
{
	/*
	 * Limit ~1/s
	 */
	
//	if (hal->ah_radar.r_last_channel.freq ==
//	    hal->ah_current_channel.freq &&
//	    tick < (hal->ah_radar.r_last_alert + hz))
		return;

/*	hal->ah_radar.r_last_channel.freq =
	    hal->ah_current_channel.freq;
	hal->ah_radar.r_last_channel.channel_flags =
	    hal->ah_current_channel.channel_flags;
	hal->ah_radar.r_last_alert = tick;

	AR5K_PRINTF("Possible radar activity detected at %u MHz (tick %u)\n",
	    hal->ah_radar.r_last_alert, hal->ah_current_channel.freq);*/
}
