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
 * HAL interface for the Atheros AR5001 Wireless LAN chipset
 * (AR5211 + AR5111).
 */

#include "ar5xxx.h"
#include "ar5211reg.h"
#include "ar5211var.h"

AR5K_BOOL	 ar5k_ar5211_nic_reset(struct ath_hal *, u_int32_t);
AR5K_BOOL	 ar5k_ar5211_nic_wakeup(struct ath_hal *, u_int16_t);
u_int16_t	 ar5k_ar5211_radio_revision(struct ath_hal *, AR5K_CHIP);
void	 ar5k_ar5211_fill(struct ath_hal *);
void	 ar5k_ar5211_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int,
    u_int);

/*
 * Initial register setting for the AR5211
 */
static const struct ar5k_ini ar5211_ini[] =
    AR5K_AR5211_INI;
static const struct ar5k_ar5211_ini_mode ar5211_mode[] =
    AR5K_AR5211_INI_MODE;
static const struct ar5k_ar5211_ini_rf ar5211_rf[] =
    AR5K_AR5211_INI_RF;

AR5K_HAL_FUNCTIONS(extern, ar5k_ar5211,);

void
ar5k_ar5211_fill(struct ath_hal *hal)
{
	hal->ah_magic = AR5K_AR5211_MAGIC;

	/*
	 * Init/Exit functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, get_rate_table);
	AR5K_HAL_FUNCTION(hal, ar5211, detach);

	/*
	 * Reset functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, reset);
	AR5K_HAL_FUNCTION(hal, ar5211, set_opmode);
	AR5K_HAL_FUNCTION(hal, ar5211, calibrate);

	/*
	 * TX functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, update_tx_triglevel);
	AR5K_HAL_FUNCTION(hal, ar5211, setup_tx_queue);
	AR5K_HAL_FUNCTION(hal, ar5211, setup_tx_queueprops);
	AR5K_HAL_FUNCTION(hal, ar5211, release_tx_queue);
	AR5K_HAL_FUNCTION(hal, ar5211, reset_tx_queue);
	AR5K_HAL_FUNCTION(hal, ar5211, get_tx_buf);
	AR5K_HAL_FUNCTION(hal, ar5211, put_tx_buf);
	AR5K_HAL_FUNCTION(hal, ar5211, tx_start);
	AR5K_HAL_FUNCTION(hal, ar5211, stop_tx_dma);
	AR5K_HAL_FUNCTION(hal, ar5211, setup_tx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, setup_xtx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, fill_tx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, proc_tx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, has_veol);

	/*
	 * RX functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, get_rx_buf);
	AR5K_HAL_FUNCTION(hal, ar5211, put_rx_buf);
	AR5K_HAL_FUNCTION(hal, ar5211, start_rx);
	AR5K_HAL_FUNCTION(hal, ar5211, stop_rx_dma);
	AR5K_HAL_FUNCTION(hal, ar5211, start_rx_pcu);
	AR5K_HAL_FUNCTION(hal, ar5211, stop_pcu_recv);
	AR5K_HAL_FUNCTION(hal, ar5211, set_mcast_filter);
	AR5K_HAL_FUNCTION(hal, ar5211, set_mcast_filterindex);
	AR5K_HAL_FUNCTION(hal, ar5211, clear_mcast_filter_idx);
	AR5K_HAL_FUNCTION(hal, ar5211, get_rx_filter);
	AR5K_HAL_FUNCTION(hal, ar5211, set_rx_filter);
	AR5K_HAL_FUNCTION(hal, ar5211, setup_rx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, proc_rx_desc);
	AR5K_HAL_FUNCTION(hal, ar5211, set_rx_signal);

	/*
	 * Misc functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, dump_state);
	AR5K_HAL_FUNCTION(hal, ar5211, get_diag_state);
	AR5K_HAL_FUNCTION(hal, ar5211, get_lladdr);
	AR5K_HAL_FUNCTION(hal, ar5211, set_lladdr);
	AR5K_HAL_FUNCTION(hal, ar5211, set_regdomain);
	AR5K_HAL_FUNCTION(hal, ar5211, set_ledstate);
	AR5K_HAL_FUNCTION(hal, ar5211, set_associd);
	AR5K_HAL_FUNCTION(hal, ar5211, set_gpio_input);
	AR5K_HAL_FUNCTION(hal, ar5211, set_gpio_output);
	AR5K_HAL_FUNCTION(hal, ar5211, get_gpio);
	AR5K_HAL_FUNCTION(hal, ar5211, set_gpio);
	AR5K_HAL_FUNCTION(hal, ar5211, set_gpio_intr);
	AR5K_HAL_FUNCTION(hal, ar5211, get_tsf32);
	AR5K_HAL_FUNCTION(hal, ar5211, get_tsf64);
	AR5K_HAL_FUNCTION(hal, ar5211, reset_tsf);
	AR5K_HAL_FUNCTION(hal, ar5211, get_regdomain);
	AR5K_HAL_FUNCTION(hal, ar5211, detect_card_present);
	AR5K_HAL_FUNCTION(hal, ar5211, update_mib_counters);
	AR5K_HAL_FUNCTION(hal, ar5211, get_rf_gain);
	AR5K_HAL_FUNCTION(hal, ar5211, set_slot_time);
	AR5K_HAL_FUNCTION(hal, ar5211, get_slot_time);
	AR5K_HAL_FUNCTION(hal, ar5211, set_ack_timeout);
	AR5K_HAL_FUNCTION(hal, ar5211, get_ack_timeout);
	AR5K_HAL_FUNCTION(hal, ar5211, set_cts_timeout);
	AR5K_HAL_FUNCTION(hal, ar5211, get_cts_timeout);

	/*
	 * Key table (WEP) functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, is_cipher_supported);
	AR5K_HAL_FUNCTION(hal, ar5211, get_keycache_size);
	AR5K_HAL_FUNCTION(hal, ar5211, reset_key);
	AR5K_HAL_FUNCTION(hal, ar5211, is_key_valid);
	AR5K_HAL_FUNCTION(hal, ar5211, set_key);
	AR5K_HAL_FUNCTION(hal, ar5211, set_key_lladdr);

	/*
	 * Power management functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, set_power);
	AR5K_HAL_FUNCTION(hal, ar5211, get_power_mode);
	AR5K_HAL_FUNCTION(hal, ar5211, query_pspoll_support);
	AR5K_HAL_FUNCTION(hal, ar5211, init_pspoll);
	AR5K_HAL_FUNCTION(hal, ar5211, enable_pspoll);
	AR5K_HAL_FUNCTION(hal, ar5211, disable_pspoll);

	/*
	 * Beacon functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, init_beacon);
	AR5K_HAL_FUNCTION(hal, ar5211, set_beacon_timers);
	AR5K_HAL_FUNCTION(hal, ar5211, reset_beacon);
	AR5K_HAL_FUNCTION(hal, ar5211, wait_for_beacon);

	/*
	 * Interrupt functions
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, is_intr_pending);
	AR5K_HAL_FUNCTION(hal, ar5211, get_isr);
	AR5K_HAL_FUNCTION(hal, ar5211, get_intr);
	AR5K_HAL_FUNCTION(hal, ar5211, set_intr);

	/*
	 * Chipset functions (ar5k-specific, non-HAL)
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, get_capabilities);
	AR5K_HAL_FUNCTION(hal, ar5211, radar_alert);

	/*
	 * EEPROM access
	 */
	AR5K_HAL_FUNCTION(hal, ar5211, eeprom_is_busy);
	AR5K_HAL_FUNCTION(hal, ar5211, eeprom_read);
	AR5K_HAL_FUNCTION(hal, ar5211, eeprom_write);

	/* Functions not found in OpenBSD */
	AR5K_HAL_FUNCTION(hal, ar5211, get_tx_queueprops);
	AR5K_HAL_FUNCTION(hal, ar5211, get_capability);
	AR5K_HAL_FUNCTION(hal, ar5211, num_tx_pending);
	AR5K_HAL_FUNCTION(hal, ar5211, phy_disable);
	AR5K_HAL_FUNCTION(hal, ar5211, set_pcu_config);
	AR5K_HAL_FUNCTION(hal, ar5211, set_txpower_limit);
	AR5K_HAL_FUNCTION(hal, ar5211, set_def_antenna);
	AR5K_HAL_FUNCTION(hal, ar5211, get_def_antenna);
	AR5K_HAL_FUNCTION(hal, ar5211, set_bssid_mask);
	/*Totaly unimplemented*/
	AR5K_HAL_FUNCTION(hal, ar5211, set_capability);
	AR5K_HAL_FUNCTION(hal, ar5211, proc_mib_event);
	AR5K_HAL_FUNCTION(hal, ar5211, get_tx_inter_queue);

}

struct ath_hal * /*Ported & removed an arg from call to set_associd*/
ar5k_ar5211_attach(u_int16_t device, AR5K_SOFTC sc, AR5K_BUS_TAG st,
    AR5K_BUS_HANDLE sh, AR5K_STATUS *status)
{
	struct ath_hal *hal = (struct ath_hal*) sc;
	u_int8_t mac[IEEE80211_ADDR_LEN];
	u_int32_t srev;

	ar5k_ar5211_fill(hal);

	/* Bring device out of sleep and reset it's units */
	if (ar5k_ar5211_nic_wakeup(hal, AR5K_INIT_MODE) != TRUE)
		return (NULL);

	/* Get MAC, PHY and RADIO revisions */
	srev = AR5K_REG_READ(AR5K_AR5211_SREV);
	hal->ah_mac_srev = srev;
	hal->ah_mac_version = AR5K_REG_MS(srev, AR5K_AR5211_SREV_VER);
	hal->ah_mac_revision = AR5K_REG_MS(srev, AR5K_AR5211_SREV_REV);
	hal->ah_phy_revision = AR5K_REG_READ(AR5K_AR5211_PHY_CHIP_ID) &
	    0x00ffffffff;
	hal->ah_radio_5ghz_revision =
	    ar5k_ar5211_radio_revision(hal, AR5K_CHIP_5GHZ);
	hal->ah_radio_2ghz_revision = 0;

	/* Identify the chipset (this has to be done in an early step) */
	hal->ah_version = AR5K_AR5211;
	hal->ah_radio = AR5K_AR5111;
	hal->ah_phy = AR5K_AR5211_PHY(0);

	bcopy(etherbroadcastaddr, mac, IEEE80211_ADDR_LEN);
	ar5k_ar5211_set_associd(hal, mac, 0);
	ar5k_ar5211_get_lladdr(hal, mac);
	ar5k_ar5211_set_opmode(hal);

	return (hal);
}

AR5K_BOOL
ar5k_ar5211_nic_reset(struct ath_hal *hal, u_int32_t val)
{
	AR5K_BOOL ret = FALSE;
	u_int32_t mask = val ? val : ~0;

	/* Read-and-clear */
	AR5K_REG_READ(AR5K_AR5211_RXDP);

	/*
	 * Reset the device and wait until success
	 */
	AR5K_REG_WRITE(AR5K_AR5211_RC, val);

	/* Wait at least 128 PCI clocks */
	AR5K_DELAY(15);

	val &=
	    AR5K_AR5211_RC_PCU | AR5K_AR5211_RC_BB;

	mask &=
	    AR5K_AR5211_RC_PCU | AR5K_AR5211_RC_BB;

	ret = ar5k_register_timeout(hal, AR5K_AR5211_RC, mask, val, FALSE);

	/*
	 * Reset configuration register
	 */
	if ((val & AR5K_AR5211_RC_PCU) == 0)
		AR5K_REG_WRITE(AR5K_AR5211_CFG, AR5K_AR5211_INIT_CFG);

	return (ret);
}

AR5K_BOOL
ar5k_ar5211_nic_wakeup(struct ath_hal *hal, u_int16_t flags)
{
	u_int32_t turbo, mode, clock;

	turbo = 0;
	mode = 0;
	clock = 0;

	/*
	 * Get channel mode flags
	 */

	if (flags & CHANNEL_2GHZ) {
		mode |= AR5K_AR5211_PHY_MODE_FREQ_2GHZ;
		clock |= AR5K_AR5211_PHY_PLL_44MHZ;
	} else if (flags & CHANNEL_5GHZ) {
		mode |= AR5K_AR5211_PHY_MODE_FREQ_5GHZ;
		clock |= AR5K_AR5211_PHY_PLL_40MHZ;
	} else {
		AR5K_PRINT("invalid radio frequency mode\n");
		return (FALSE);
	}

	if ((flags & CHANNEL_CCK) ||
	    (flags & CHANNEL_DYN)) {
		/* Dynamic OFDM/CCK is not supported by the AR5211 */
		mode |= AR5K_AR5211_PHY_MODE_MOD_CCK;
	} else if (flags & CHANNEL_OFDM) {
		mode |= AR5K_AR5211_PHY_MODE_MOD_OFDM;
	} else {
		AR5K_PRINT("invalid radio frequency mode\n");
		return (FALSE);
	}

	if (flags & CHANNEL_TURBO) {
		turbo = AR5K_AR5211_PHY_TURBO_MODE |
		    AR5K_AR5211_PHY_TURBO_SHORT;
	}

	/*
	 * Reset and wakeup the device
	 */

	/* ...reset chipset and PCI device */
	if (ar5k_ar5211_nic_reset(hal,
		AR5K_AR5211_RC_CHIP | AR5K_AR5211_RC_PCI) == FALSE) {
		AR5K_PRINT("failed to reset the AR5211 + PCI chipset\n");
		return (FALSE);
	}

	/* ...wakeup */
	if (ar5k_ar5211_set_power(hal,
		AR5K_PM_AWAKE, TRUE, 0) == FALSE) {
		AR5K_PRINT("failed to resume the AR5211 (again)\n");
		return (FALSE);
	}

	/* ...final warm reset */
	if (ar5k_ar5211_nic_reset(hal, 0) == FALSE) {
		AR5K_PRINT("failed to warm reset the AR5211\n");
		return (FALSE);
	}

	/* ...set the PHY operating mode */
	AR5K_REG_WRITE(AR5K_AR5211_PHY_PLL, clock);
	AR5K_DELAY(300);

	AR5K_REG_WRITE(AR5K_AR5211_PHY_MODE, mode);
	AR5K_REG_WRITE(AR5K_AR5211_PHY_TURBO, turbo);

	return (TRUE);
}

u_int16_t
ar5k_ar5211_radio_revision(struct ath_hal *hal, AR5K_CHIP chip)
{
	int i;
	u_int32_t srev;
	u_int16_t ret;

	/*
	 * Set the radio chip access register
	 */
	switch (chip) {
	case AR5K_CHIP_2GHZ:
		AR5K_REG_WRITE(AR5K_AR5211_PHY(0), AR5K_AR5211_PHY_SHIFT_2GHZ);
		break;
	case AR5K_CHIP_5GHZ:
		AR5K_REG_WRITE(AR5K_AR5211_PHY(0), AR5K_AR5211_PHY_SHIFT_5GHZ);
		break;
	default:
		return (0);
	}

	AR5K_DELAY(2000);

	/* ...wait until PHY is ready and read the selected radio revision */
	AR5K_REG_WRITE(AR5K_AR5211_PHY(0x34), 0x00001c16);

	for (i = 0; i < 8; i++)
		AR5K_REG_WRITE(AR5K_AR5211_PHY(0x20), 0x00010000);
	srev = (AR5K_REG_READ(AR5K_AR5211_PHY(0x100)) >> 24) & 0xff;

	ret = ar5k_bitswap(((srev & 0xf0) >> 4) | ((srev & 0x0f) << 4), 8);

	/* Reset to the 5GHz mode */
	AR5K_REG_WRITE(AR5K_AR5211_PHY(0), AR5K_AR5211_PHY_SHIFT_5GHZ);

	return (ret);
}

const AR5K_RATE_TABLE *
ar5k_ar5211_get_rate_table(struct ath_hal *hal, u_int mode)
{
	switch (mode) {
	case AR5K_MODE_11A:
		return (&hal->ah_rt_11a);
	case AR5K_MODE_TURBO:
		return (&hal->ah_rt_turbo);
	case AR5K_MODE_11B:
		return (&hal->ah_rt_11b);
	case AR5K_MODE_11G:
		return (&hal->ah_rt_11g);
	default:
		return (NULL);
	}

	return (NULL);
}

void /*O.K.*/
ar5k_ar5211_detach(struct ath_hal *hal)
{
	/*
	 * Free HAL structure, assume interrupts are down
	 */
	free(hal, M_DEVBUF);
}

AR5K_BOOL /*New*/
ar5k_ar5211_phy_disable(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	AR5K_REG_WRITE(AR5K_AR5211_PHY_ACTIVE, AR5K_AR5211_PHY_DISABLE);
	return TRUE;
}

AR5K_BOOL /*Ported*/
ar5k_ar5211_reset(struct ath_hal *hal, AR5K_OPMODE op_mode, AR5K_CHANNEL *channel,
    AR5K_BOOL change_channel, AR5K_STATUS *status)
{
	struct ar5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u_int8_t mac[IEEE80211_ADDR_LEN];
	u_int32_t data, s_seq, s_ant, s_led[3];
	u_int i, mode, freq, ee_mode, ant[2];

	/*
	 * Save some registers before a reset
	 */
	if (change_channel == TRUE) {
		s_seq = AR5K_REG_READ(AR5K_AR5211_DCU_SEQNUM(0));
		s_ant = AR5K_REG_READ(AR5K_AR5211_DEFAULT_ANTENNA);
	} else {
		s_seq = 0;
		s_ant = 1;
	}

	s_led[0] = AR5K_REG_READ(AR5K_AR5211_PCICFG) &
	    AR5K_AR5211_PCICFG_LEDSTATE;
	s_led[1] = AR5K_REG_READ(AR5K_AR5211_GPIOCR);
	s_led[2] = AR5K_REG_READ(AR5K_AR5211_GPIODO);

	if (ar5k_ar5211_nic_wakeup(hal, channel->channel_flags) == FALSE)
		return (FALSE);

	/*
	 * Initialize operating mode
	 */
	hal->ah_op_mode = op_mode;

	 switch (channel->channel_flags & CHANNEL_MODES) {
 	 case CHANNEL_A:
		mode = AR5K_INI_VAL_11A;
		freq = AR5K_INI_RFGAIN_5GHZ;
		ee_mode = AR5K_EEPROM_MODE_11A;
		break ;
	case CHANNEL_T:
		mode = AR5K_INI_VAL_11A_TURBO;
		freq = AR5K_INI_RFGAIN_5GHZ;
		ee_mode = AR5K_EEPROM_MODE_11A;
		break;
	case CHANNEL_B: 
		mode = AR5K_INI_VAL_11B;
		freq = AR5K_INI_RFGAIN_2GHZ;
		ee_mode = AR5K_EEPROM_MODE_11B;
		break;
	case CHANNEL_G:
		mode = AR5K_INI_VAL_11G;
		freq = AR5K_INI_RFGAIN_2GHZ;
		ee_mode = AR5K_EEPROM_MODE_11G;
		break;
	default:
		AR5K_PRINTF("invalid channel: %d\n", channel->freq);
		return (FALSE);
	}

	/* PHY access enable */
	AR5K_REG_WRITE(AR5K_AR5211_PHY(0), AR5K_AR5211_PHY_SHIFT_5GHZ);

	/*
	 * Write initial RF registers
	 */
	ar5k_ar5211_rfregs(hal, channel, freq, ee_mode);

	/*
	 * Write initial mode settings
	 */
	for (i = 0; i < AR5K_ELEMENTS(ar5211_mode); i++) {
		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE((u_int32_t)ar5211_mode[i].mode_register,
		    ar5211_mode[i].mode_value[mode]);
	}

	/*
	 * Write initial register settings
	 */
	for (i = 0; i < AR5K_ELEMENTS(ar5211_ini); i++) {
		if (change_channel == TRUE &&
		    ar5211_ini[i].ini_register >= AR5K_AR5211_PCU_MIN &&
		    ar5211_ini[i].ini_register <= AR5K_AR5211_PCU_MAX)
			continue;

		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE((u_int32_t)ar5211_ini[i].ini_register,
		    ar5211_ini[i].ini_value);
	}

	/*
	 * Write initial RF gain settings
	 */
	if (ar5k_rfgain(hal, AR5K_INI_PHY_5111, freq) == FALSE)
		return (FALSE);

	AR5K_DELAY(1000);

	/*
	 * Configure additional registers
	 */

	if (hal->ah_radio == AR5K_AR5111) {
		if (channel->channel_flags & CHANNEL_B)
			AR5K_REG_ENABLE_BITS(AR5K_AR5211_TXCFG,
			    AR5K_AR5211_TXCFG_B_MODE);
		else
			AR5K_REG_DISABLE_BITS(AR5K_AR5211_TXCFG,
			    AR5K_AR5211_TXCFG_B_MODE);
	}

	/* Set antenna mode */
	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x44),
	    hal->ah_antenna[ee_mode][0], 0xfffffc06);

		if (freq == AR5K_INI_RFGAIN_2GHZ)
			ant[0] = ant[1] =AR5K_ANT_FIXED_B;
		else
			ant[0] = ant[1] = AR5K_ANT_FIXED_A;

	AR5K_REG_WRITE(AR5K_AR5211_PHY_ANT_SWITCH_TABLE_0,
	    hal->ah_antenna[ee_mode][ant[0]]);
	AR5K_REG_WRITE(AR5K_AR5211_PHY_ANT_SWITCH_TABLE_1,
	    hal->ah_antenna[ee_mode][ant[1]]);

	/* Commit values from EEPROM */
	AR5K_REG_WRITE_BITS(AR5K_AR5211_PHY_FC,
	    AR5K_AR5211_PHY_FC_TX_CLIP, ee->ee_tx_clip);

	AR5K_REG_WRITE(AR5K_AR5211_PHY(0x5a),
	    AR5K_AR5211_PHY_NF_SVAL(ee->ee_noise_floor_thr[ee_mode]));

	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x11),
	    (ee->ee_switch_settling[ee_mode] << 7) & 0x3f80, 0xffffc07f);
	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x12),
	    (ee->ee_ant_tx_rx[ee_mode] << 12) & 0x3f000, 0xfffc0fff);
	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x14),
	    (ee->ee_adc_desired_size[ee_mode] & 0x00ff) |
	    ((ee->ee_pga_desired_size[ee_mode] << 8) & 0xff00), 0xffff0000);

	AR5K_REG_WRITE(AR5K_AR5211_PHY(0x0d),
	    (ee->ee_tx_end2xpa_disable[ee_mode] << 24) |
	    (ee->ee_tx_end2xpa_disable[ee_mode] << 16) |
	    (ee->ee_tx_frm2xpa_enable[ee_mode] << 8) |
	    (ee->ee_tx_frm2xpa_enable[ee_mode]));

	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x0a),
	    ee->ee_tx_end2xlna_enable[ee_mode] << 8, 0xffff00ff);
	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x19),
	    (ee->ee_thr_62[ee_mode] << 12) & 0x7f000, 0xfff80fff);
	AR5K_REG_MASKED_BITS(AR5K_AR5211_PHY(0x49), 4, 0xffffff01);

	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PHY_IQ,
	    AR5K_AR5211_PHY_IQ_CORR_ENABLE |
	    (ee->ee_i_cal[ee_mode] << AR5K_AR5211_PHY_IQ_CORR_Q_I_COFF_S) |
	    ee->ee_q_cal[ee_mode]);

	/*
	 * Restore saved values
	 */
	AR5K_REG_WRITE(AR5K_AR5211_DCU_SEQNUM(0), s_seq);
	AR5K_REG_WRITE(AR5K_AR5211_DEFAULT_ANTENNA, s_ant);
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PCICFG, s_led[0]);
	AR5K_REG_WRITE(AR5K_AR5211_GPIOCR, s_led[1]);
	AR5K_REG_WRITE(AR5K_AR5211_GPIODO, s_led[2]);

	/*
	 * Misc
	 */
	bcopy(etherbroadcastaddr, mac, IEEE80211_ADDR_LEN);
	ar5k_ar5211_set_associd(hal, mac, 0);
	ar5k_ar5211_set_opmode(hal);
	AR5K_REG_WRITE(AR5K_AR5211_PISR, 0xffffffff);
	AR5K_REG_WRITE(AR5K_AR5211_RSSI_THR, AR5K_TUNE_RSSI_THRES);

	/*
	 * Set Rx/Tx DMA Configuration
	 */
	AR5K_REG_WRITE_BITS(AR5K_AR5211_TXCFG, AR5K_AR5211_TXCFG_SDMAMR,
	    AR5K_AR5211_DMASIZE_512B | AR5K_AR5211_TXCFG_DMASIZE);
	AR5K_REG_WRITE_BITS(AR5K_AR5211_RXCFG, AR5K_AR5211_RXCFG_SDMAMW,
	    AR5K_AR5211_DMASIZE_512B);

	/*
	 * Set channel and calibrate the PHY
	 */
	if (ar5k_channel(hal, channel) == FALSE)
		return (FALSE);

	/*
	 * Enable the PHY and wait until completion
	 */
	AR5K_REG_WRITE(AR5K_AR5211_PHY_ACTIVE, AR5K_AR5211_PHY_ENABLE);

	data = AR5K_REG_READ(AR5K_AR5211_PHY_RX_DELAY) &
	    AR5K_AR5211_PHY_RX_DELAY_M;
	data = (channel->channel_flags & CHANNEL_CCK) ?
	    ((data << 2) / 22) : (data / 10);

	AR5K_DELAY(100 + data);

	/*
	 * Start calibration
	 */
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PHY_AGCCTL,
	    AR5K_AR5211_PHY_AGCCTL_NF |
	    AR5K_AR5211_PHY_AGCCTL_CAL);

	if (channel->channel_flags & CHANNEL_B) {
		hal->ah_calibration = FALSE;
	} else {
		hal->ah_calibration = TRUE;
		AR5K_REG_WRITE_BITS(AR5K_AR5211_PHY_IQ,
		    AR5K_AR5211_PHY_IQ_CAL_NUM_LOG_MAX, 15);
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_PHY_IQ,
		    AR5K_AR5211_PHY_IQ_RUN);
	}

	/*
	 * Reset queues and start beacon timers at the end of the reset routine
	 */
	for (i = 0; i < hal->ah_capabilities.cap_queues.q_tx_num; i++) {
		AR5K_REG_WRITE_Q(AR5K_AR5211_DCU_QCUMASK(i), i);
		if (ar5k_ar5211_reset_tx_queue(hal, i) == FALSE) {
			AR5K_PRINTF("failed to reset TX queue #%d\n", i);
			return (FALSE);
		}
	}

	/* Pre-enable interrupts */
	ar5k_ar5211_set_intr(hal, AR5K_INT_RX | AR5K_INT_TX | AR5K_INT_FATAL);

	/*
	 * Set RF kill flags if supported by the device (read from the EEPROM)
	 */
/*	if (AR5K_EEPROM_HDR_RFKILL(hal->ah_capabilities.cap_eeprom.ee_header)) {
		ar5k_ar5211_set_gpio_input(hal, 0);
		if ((hal->ah_gpio[0] = ar5k_ar5211_get_gpio(hal, 0)) == 0)
			ar5k_ar5211_set_gpio_intr(hal, 0, 1);
		else
			ar5k_ar5211_set_gpio_intr(hal, 0, 0);
	}
*/
	/* 
	 * Disable beacons and reset the register
	 */
	AR5K_REG_DISABLE_BITS(AR5K_AR5211_BEACON,
	    AR5K_AR5211_BEACON_ENABLE | AR5K_AR5211_BEACON_RESET_TSF);

	return (TRUE);
}

void /*New*/
ar5k_ar5211_set_def_antenna(struct ath_hal *hal, u_int ant)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	AR5K_REG_WRITE(AR5K_AR5211_DEFAULT_ANTENNA, ant);
	return;
}

u_int/*New*/
ar5k_ar5211_get_def_antenna(struct ath_hal *hal)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	return AR5K_REG_READ(AR5K_AR5211_DEFAULT_ANTENNA);
}

void
ar5k_ar5211_set_opmode(struct ath_hal *hal)
{
	u_int32_t pcu_reg, low_id, high_id;

	pcu_reg = 0;

	switch (hal->ah_op_mode) {
	case AR5K_M_IBSS:
		pcu_reg |= AR5K_AR5211_STA_ID1_ADHOC |
		    AR5K_AR5211_STA_ID1_DESC_ANTENNA;
		break;

	case AR5K_M_HOSTAP:
		pcu_reg |= AR5K_AR5211_STA_ID1_AP |
		    AR5K_AR5211_STA_ID1_RTS_DEFAULT_ANTENNA;
		break;

	case AR5K_M_STA:
	case AR5K_M_MONITOR:
		pcu_reg |= AR5K_AR5211_STA_ID1_DEFAULT_ANTENNA;
		break;

	default:
		return;
	}

	/*
	 * Set PCU registers
	 */
	low_id = AR5K_LOW_ID(hal->ah_sta_id);
	high_id = AR5K_HIGH_ID(hal->ah_sta_id);
	AR5K_REG_WRITE(AR5K_AR5211_STA_ID0, low_id);
	AR5K_REG_WRITE(AR5K_AR5211_STA_ID1, pcu_reg | high_id);

	return;
}

void /*New*/
ar5k_ar5211_set_pcu_config(struct ath_hal *hal)
{
	AR5K_TRACE;
	ar5k_ar5211_set_opmode(hal);
	return;
}

AR5K_BOOL
ar5k_ar5211_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel)
{
	u_int32_t i_pwr, q_pwr;
	int32_t iq_corr, i_coff, i_coffd, q_coff, q_coffd;

	if (hal->ah_calibration == FALSE ||
	    AR5K_REG_READ(AR5K_AR5211_PHY_IQ) & AR5K_AR5211_PHY_IQ_RUN)
		goto done;

	hal->ah_calibration = FALSE;

	iq_corr = AR5K_REG_READ(AR5K_AR5211_PHY_IQRES_CAL_CORR);
	i_pwr = AR5K_REG_READ(AR5K_AR5211_PHY_IQRES_CAL_PWR_I);
	q_pwr = AR5K_REG_READ(AR5K_AR5211_PHY_IQRES_CAL_PWR_Q);
	i_coffd = ((i_pwr >> 1) + (q_pwr >> 1)) >> 7;
	q_coffd = q_pwr >> 6;

	if (i_coffd == 0 || q_coffd == 0)
		goto done;

	i_coff = ((-iq_corr) / i_coffd) & 0x3f;
	q_coff = (((int32_t)i_pwr / q_coffd) - 64) & 0x1f;

	/* Commit new IQ value */
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PHY_IQ,
	    AR5K_AR5211_PHY_IQ_CORR_ENABLE |
	    ((u_int32_t)q_coff) |
	    ((u_int32_t)i_coff << AR5K_AR5211_PHY_IQ_CORR_Q_I_COFF_S));

 done:
	/* Start noise floor calibration */
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PHY_AGCCTL,
	    AR5K_AR5211_PHY_AGCCTL_NF);

	return (TRUE);
}

/*
 * Transmit functions
 */

AR5K_BOOL
ar5k_ar5211_update_tx_triglevel(struct ath_hal *hal, AR5K_BOOL increase)
{
	u_int32_t trigger_level, imr;
	AR5K_BOOL status = FALSE;

	/*
	 * Disable interrupts by setting the mask
	 */
	imr = ar5k_ar5211_set_intr(hal, hal->ah_imr & ~AR5K_INT_GLOBAL);

	trigger_level = AR5K_REG_MS(AR5K_REG_READ(AR5K_AR5211_TXCFG),
	    AR5K_AR5211_TXCFG_TXFULL);

	if (increase == FALSE) {
		if (--trigger_level < AR5K_TUNE_MIN_TX_FIFO_THRES)
			goto done;
	} else
		trigger_level +=
		    ((AR5K_TUNE_MAX_TX_FIFO_THRES - trigger_level) / 2);

	/*
	 * Update trigger level on success
	 */
	AR5K_REG_WRITE_BITS(AR5K_AR5211_TXCFG,
	    AR5K_AR5211_TXCFG_TXFULL, trigger_level);
	status = TRUE;

 done:
	/*
	 * Restore interrupt mask
	 */
	ar5k_ar5211_set_intr(hal, imr);

	return (status);
}

int
ar5k_ar5211_setup_tx_queue(struct ath_hal *hal, AR5K_TX_QUEUE queue_type,
     AR5K_TXQ_INFO *queue_info)
{
	u_int queue;

	/*
	 * Get queue by type
	 */
	if (queue_type == AR5K_TX_QUEUE_DATA) {
		for (queue = AR5K_TX_QUEUE_ID_DATA_MIN;
		     hal->ah_txq[queue].tqi_type != AR5K_TX_QUEUE_INACTIVE;
		     queue++)
			if (queue > AR5K_TX_QUEUE_ID_DATA_MAX)
				return (-1);
	} else if (queue_type == AR5K_TX_QUEUE_UAPSD) {
		queue = AR5K_TX_QUEUE_ID_UAPSD;
	} else if (queue_type == AR5K_TX_QUEUE_BEACON) {
		queue = AR5K_TX_QUEUE_ID_BEACON;
	} else if (queue_type == AR5K_TX_QUEUE_CAB) {
		queue = AR5K_TX_QUEUE_ID_CAB;
	} else
		return (-1);

	/*
	 * Setup internal queue structure
	 */
	bzero(&hal->ah_txq[queue], sizeof(AR5K_TXQ_INFO));
	hal->ah_txq[queue].tqi_type = queue_type;

	if (queue_info != NULL) {
		queue_info->tqi_type = queue_type;
		if (ar5k_ar5211_setup_tx_queueprops(hal, queue, queue_info)
		    != TRUE)
			return (-1);
	}

	AR5K_Q_ENABLE_BITS(hal->ah_txq_interrupts, queue);

	return (queue);
}

AR5K_BOOL
ar5k_ar5211_setup_tx_queueprops(struct ath_hal *hal, int queue,
    const AR5K_TXQ_INFO *queue_info)
{
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	if (hal->ah_txq[queue].tqi_type != AR5K_TX_QUEUE_INACTIVE)
		return (FALSE);

	bcopy(queue_info, &hal->ah_txq[queue], sizeof(AR5K_TXQ_INFO));

	if ((queue_info->tqi_type == AR5K_TX_QUEUE_DATA && 
		((queue_info->tqi_subtype == AR5K_WME_AC_VI) ||
		(queue_info->tqi_subtype == AR5K_WME_AC_VO))) ||
		queue_info->tqi_type == AR5K_TX_QUEUE_UAPSD)
		hal->ah_txq[queue].tqi_flags |=
		    AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS;

	return (TRUE);
}

AR5K_BOOL /*New*/
ar5k_ar5211_get_tx_queueprops(struct ath_hal *hal, int queue, AR5K_TXQ_INFO *queue_info)
{
	AR5K_TRACE;
	memcpy(queue_info, &hal->ah_txq[queue], sizeof(AR5K_TXQ_INFO));
	return (TRUE);
}

AR5K_BOOL
ar5k_ar5211_release_tx_queue(struct ath_hal *hal, u_int queue)
{
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* This queue will be skipped in further operations */
	hal->ah_txq[queue].tqi_type = AR5K_TX_QUEUE_INACTIVE;
	AR5K_Q_DISABLE_BITS(hal->ah_txq_interrupts, queue);

	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_reset_tx_queue(struct ath_hal *hal, u_int queue)
{
	u_int32_t cw_min, cw_max, retry_lg, retry_sh;
	AR5K_TXQ_INFO *tq;

	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	tq = &hal->ah_txq[queue];

	if (tq->tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return (TRUE);

	/*
	 * Set registers by channel mode
	 */
	if (IS_CHAN_B(hal->ah_current_channel)) {
		hal->ah_cw_min = AR5K_TUNE_CWMIN_11B;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX_11B;
		hal->ah_aifs = AR5K_TUNE_AIFS_11B;
	} else {
		hal->ah_cw_min = AR5K_TUNE_CWMIN;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX;
		hal->ah_aifs = AR5K_TUNE_AIFS;
	}

	/*
	 * Set retry limits
	 */
	if (hal->ah_software_retry == TRUE) {
		/* XXX Need to test this */
		retry_lg = hal->ah_limit_tx_retries;
		retry_sh = retry_lg =
		    retry_lg > AR5K_AR5211_DCU_RETRY_LMT_SH_RETRY ?
		    AR5K_AR5211_DCU_RETRY_LMT_SH_RETRY : retry_lg;
	} else {
		retry_lg = AR5K_INIT_LG_RETRY;
		retry_sh = AR5K_INIT_SH_RETRY;
	}

	AR5K_REG_WRITE(AR5K_AR5211_DCU_RETRY_LMT(queue),
	    AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
	    AR5K_AR5211_DCU_RETRY_LMT_SLG_RETRY) |
	    AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
	    AR5K_AR5211_DCU_RETRY_LMT_SSH_RETRY) |
	    AR5K_REG_SM(retry_lg, AR5K_AR5211_DCU_RETRY_LMT_LG_RETRY) |
	    AR5K_REG_SM(retry_sh, AR5K_AR5211_DCU_RETRY_LMT_SH_RETRY));

	/*
	 * Set initial content window (cw_min/cw_max)
	 */
	cw_min = 1;
	while (cw_min < hal->ah_cw_min)
		cw_min = (cw_min << 1) | 1;

	cw_min = tq->tqi_cw_min < 0 ?
	    (cw_min >> (-tq->tqi_cw_min)) :
	    ((cw_min << tq->tqi_cw_min) + (1 << tq->tqi_cw_min) - 1);
	cw_max = tq->tqi_cw_max < 0 ?
	    (cw_max >> (-tq->tqi_cw_max)) :
	    ((cw_max << tq->tqi_cw_max) + (1 << tq->tqi_cw_max) - 1);

	AR5K_REG_WRITE(AR5K_AR5211_DCU_LCL_IFS(queue),
	    AR5K_REG_SM(cw_min, AR5K_AR5211_DCU_LCL_IFS_CW_MIN) |
	    AR5K_REG_SM(cw_max, AR5K_AR5211_DCU_LCL_IFS_CW_MAX) |
	    AR5K_REG_SM(hal->ah_aifs + tq->tqi_aifs,
	    AR5K_AR5211_DCU_LCL_IFS_AIFS));

	/*
	 * Set misc registers
	 */
	AR5K_REG_WRITE(AR5K_AR5211_QCU_MISC(queue),
	    AR5K_AR5211_QCU_MISC_DCU_EARLY);

	if (tq->tqi_cbr_period) {
		AR5K_REG_WRITE(AR5K_AR5211_QCU_CBRCFG(queue),
		    AR5K_REG_SM(tq->tqi_cbr_period,
		    AR5K_AR5211_QCU_CBRCFG_INTVAL) |
		    AR5K_REG_SM(tq->tqi_cbr_overflow_limit,
		    AR5K_AR5211_QCU_CBRCFG_ORN_THRES));
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
		    AR5K_AR5211_QCU_MISC_FRSHED_CBR);
		if (tq->tqi_cbr_overflow_limit)
			AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
			    AR5K_AR5211_QCU_MISC_CBR_THRES_ENABLE);
	}

	if (tq->tqi_ready_time) {
		AR5K_REG_WRITE(AR5K_AR5211_QCU_RDYTIMECFG(queue),
		    AR5K_REG_SM(tq->tqi_ready_time,
		    AR5K_AR5211_QCU_RDYTIMECFG_INTVAL) |
		    AR5K_AR5211_QCU_RDYTIMECFG_ENABLE);
	}

	if (tq->tqi_burst_time) {
		AR5K_REG_WRITE(AR5K_AR5211_DCU_CHAN_TIME(queue),
		    AR5K_REG_SM(tq->tqi_burst_time,
		    AR5K_AR5211_DCU_CHAN_TIME_DUR) |
		    AR5K_AR5211_DCU_CHAN_TIME_ENABLE);

		if (tq->tqi_flags & AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE) {
			AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
			    AR5K_AR5211_QCU_MISC_TXE);
		}
	}

	if (tq->tqi_flags & AR5K_TXQ_FLAG_BACKOFF_DISABLE) {
		AR5K_REG_WRITE(AR5K_AR5211_DCU_MISC(queue),
		    AR5K_AR5211_DCU_MISC_POST_FR_BKOFF_DIS);
	}

	if (tq->tqi_flags & AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE) {
		AR5K_REG_WRITE(AR5K_AR5211_DCU_MISC(queue),
		    AR5K_AR5211_DCU_MISC_BACKOFF_FRAG);
	}

	/*
	 * Set registers by queue type
	 */
	switch (tq->tqi_type) {
	case AR5K_TX_QUEUE_BEACON:
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
		    AR5K_AR5211_QCU_MISC_FRSHED_DBA_GT |
		    AR5K_AR5211_QCU_MISC_CBREXP_BCN |
		    AR5K_AR5211_QCU_MISC_BCN_ENABLE);

		AR5K_REG_ENABLE_BITS(AR5K_AR5211_DCU_MISC(queue),
		    (AR5K_AR5211_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
		    AR5K_AR5211_DCU_MISC_ARBLOCK_CTL_GLOBAL) |
		    AR5K_AR5211_DCU_MISC_POST_FR_BKOFF_DIS |
		    AR5K_AR5211_DCU_MISC_BCN_ENABLE);

		AR5K_REG_WRITE(AR5K_AR5211_QCU_RDYTIMECFG(queue),
		    ((AR5K_TUNE_BEACON_INTERVAL -
		    (AR5K_TUNE_SW_BEACON_RESP - AR5K_TUNE_DMA_BEACON_RESP) -
		    AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF) * 1024) |
		    AR5K_AR5211_QCU_RDYTIMECFG_ENABLE);
		break;

	case AR5K_TX_QUEUE_CAB:
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
		    AR5K_AR5211_QCU_MISC_FRSHED_DBA_GT |
		    AR5K_AR5211_QCU_MISC_CBREXP |
		    AR5K_AR5211_QCU_MISC_CBREXP_BCN);

		AR5K_REG_ENABLE_BITS(AR5K_AR5211_DCU_MISC(queue),
		    (AR5K_AR5211_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
		    AR5K_AR5211_DCU_MISC_ARBLOCK_CTL_GLOBAL));
		break;

	case AR5K_TX_QUEUE_UAPSD:
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_QCU_MISC(queue),
		    AR5K_AR5211_QCU_MISC_CBREXP);
		break;

	case AR5K_TX_QUEUE_DATA:
	default:
		break;
	}

	/*
	 * Enable tx queue in the secondary interrupt mask registers
	 */
	AR5K_REG_WRITE(AR5K_AR5211_SIMR0,
	    AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_AR5211_SIMR0_QCU_TXOK) |
	    AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_AR5211_SIMR0_QCU_TXDESC));
	AR5K_REG_WRITE(AR5K_AR5211_SIMR1,
	    AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_AR5211_SIMR1_QCU_TXERR));
	AR5K_REG_WRITE(AR5K_AR5211_SIMR2,
	    AR5K_REG_SM(hal->ah_txq_interrupts, AR5K_AR5211_SIMR2_QCU_TXURN));

	return (TRUE);
}

u_int32_t
ar5k_ar5211_get_tx_buf(struct ath_hal *hal, u_int queue)
{
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Get the transmit queue descriptor pointer from the selected queue
	 */
	return (AR5K_REG_READ(AR5K_AR5211_QCU_TXDP(queue)));
}

AR5K_BOOL
ar5k_ar5211_put_tx_buf(struct ath_hal *hal, u_int queue, u_int32_t phys_addr)
{
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Set the transmit queue descriptor pointer for the selected queue
	 * (this won't work if the queue is still active)
	 */
	if (AR5K_REG_READ_Q(AR5K_AR5211_QCU_TXE, queue))
		return (FALSE);

	AR5K_REG_WRITE(AR5K_AR5211_QCU_TXDP(queue), phys_addr);

	return (TRUE);
}

u_int32_t  /*Code from roofnet*/
ar5k_ar5211_num_tx_pending(struct ath_hal *hal, u_int queue) {
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);
	return (AR5K_AR5211_QCU_STS(queue) & AR5K_AR5211_QCU_STS_FRMPENDCNT);
}

AR5K_BOOL
ar5k_ar5211_tx_start(struct ath_hal *hal, u_int queue)
{
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is disabled */
	if (AR5K_REG_READ_Q(AR5K_AR5211_QCU_TXD, queue))
		return (FALSE);

	/* Start queue */
	AR5K_REG_WRITE_Q(AR5K_AR5211_QCU_TXE, queue);

	return (TRUE);
}

AR5K_BOOL /*Fixed delay*/
ar5k_ar5211_stop_tx_dma(struct ath_hal *hal, u_int queue)
{
	int i = 100, pending;

	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Schedule TX disable and wait until queue is empty
	 */
	AR5K_REG_WRITE_Q(AR5K_AR5211_QCU_TXD, queue);

	do {
		pending = AR5K_REG_READ(AR5K_AR5211_QCU_STS(queue)) &
		     AR5K_AR5211_QCU_STS_FRMPENDCNT;
		AR5K_DELAY(100);
	} while (--i && pending);

	/* Clear register */
	AR5K_REG_WRITE(AR5K_AR5211_QCU_TXD, 0);

	return (TRUE);
}

AR5K_BOOL /*O.K. - Initialize tx_desc and clear ds_hw */
ar5k_ar5211_setup_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int packet_length, u_int header_length, AR5K_PKT_TYPE type, u_int tx_power,
    u_int tx_rate0, u_int tx_tries0, u_int key_index, u_int antenna_mode,
    u_int flags, u_int rtscts_rate, u_int rtscts_duration)
{
	struct ar5k_ar5211_tx_desc *tx_desc;

	tx_desc = (struct ar5k_ar5211_tx_desc*)&desc->ds_ctl0;

	/*Clear ds_hw*/
	bzero(desc->ds_hw, sizeof(desc->ds_hw));

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return (FALSE);

	/* Initialize status descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;

	/* Setup status descriptor */

	if ((tx_desc->tx_control_0 = (packet_length &
	    AR5K_AR5211_DESC_TX_CTL0_FRAME_LEN)) != packet_length)
		return (FALSE);

	tx_desc->tx_control_0 |=
	    AR5K_REG_SM(tx_rate0, AR5K_AR5211_DESC_TX_CTL0_XMIT_RATE) |
	    AR5K_REG_SM(antenna_mode, AR5K_AR5211_DESC_TX_CTL0_ANT_MODE_XMIT);
	tx_desc->tx_control_1 =
	    AR5K_REG_SM(type, AR5K_AR5211_DESC_TX_CTL1_FRAME_TYPE);

#define _TX_FLAGS(_c, _flag)						\
	if (flags & AR5K_TXDESC_##_flag)					\
		tx_desc->tx_control_##_c |=				\
			AR5K_AR5211_DESC_TX_CTL##_c##_##_flag

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
		    AR5K_AR5211_DESC_TX_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |=
		    AR5K_REG_SM(key_index,
		    AR5K_AR5211_DESC_TX_CTL1_ENCRYPT_KEY_INDEX);
	}

	return (TRUE);
}

AR5K_BOOL/*Added an argument *last_desc -need revision -don't clear descriptor here*/
ar5k_ar5211_fill_tx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int segment_length, AR5K_BOOL first_segment, AR5K_BOOL last_segment, const struct ath_desc *last_desc)
{
	struct ar5k_ar5211_tx_desc *tx_desc;

	tx_desc = (struct ar5k_ar5211_tx_desc*)&desc->ds_ctl0;

	/* Clear status descriptor */
//	bzero(desc->ds_hw, sizeof(desc->ds_hw));

	/* Validate segment length and initialize the descriptor */
	if ((tx_desc->tx_control_1 = (segment_length &
	    AR5K_AR5211_DESC_TX_CTL1_BUF_LEN)) != segment_length)
		return (FALSE);

	if (first_segment != TRUE)
		tx_desc->tx_control_0 &= ~AR5K_AR5211_DESC_TX_CTL0_FRAME_LEN;

	if (last_segment != TRUE)
		tx_desc->tx_control_1 |= AR5K_AR5211_DESC_TX_CTL1_MORE;

	return (TRUE);
}

AR5K_BOOL
ar5k_ar5211_setup_xtx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int tx_rate1, u_int tx_tries1, u_int tx_rate2, u_int tx_tries2,
    u_int tx_rate3, u_int tx_tries3)
{
	return (FALSE);
}

AR5K_STATUS
ar5k_ar5211_proc_tx_desc(struct ath_hal *hal, struct ath_desc *desc)
{
	struct ar5k_ar5211_tx_status *tx_status;
	struct ar5k_ar5211_tx_desc *tx_desc;

	tx_desc = (struct ar5k_ar5211_tx_desc*)&desc->ds_ctl0;
	tx_status = (struct ar5k_ar5211_tx_status*)&desc->ds_hw[0];

	/* No frame has been send or error */
	if ((tx_status->tx_status_1 & AR5K_AR5211_DESC_TX_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_AR5211_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_AR5211_DESC_TX_STATUS0_RTS_FAIL_COUNT);
	desc->ds_us.tx.ts_longretry =
	    AR5K_REG_MS(tx_status->tx_status_0,
	    AR5K_AR5211_DESC_TX_STATUS0_DATA_FAIL_COUNT);
	desc->ds_us.tx.ts_seqnum =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_AR5211_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi =
	    AR5K_REG_MS(tx_status->tx_status_1,
	    AR5K_AR5211_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = 1;
	desc->ds_us.tx.ts_status = 0;
	desc->ds_us.tx.ts_rate =
	    AR5K_REG_MS(tx_desc->tx_control_0,
	    AR5K_AR5211_DESC_TX_CTL0_XMIT_RATE);

	if ((tx_status->tx_status_0 &
	    AR5K_AR5211_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0) {
		if (tx_status->tx_status_0 &
		    AR5K_AR5211_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 &
		    AR5K_AR5211_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 &
		    AR5K_AR5211_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return (AR5K_OK);
}

AR5K_BOOL
ar5k_ar5211_has_veol(struct ath_hal *hal)
{
	return (TRUE);
}

void /*Unimplemented*/
ar5k_ar5211_get_tx_inter_queue(struct ath_hal *hal, u_int32_t *i)
{
	AR5K_TRACE;
	/* XXX */
	return;
}

/*
 * Receive functions
 */

u_int32_t
ar5k_ar5211_get_rx_buf(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_RXDP));
}

void
ar5k_ar5211_put_rx_buf(struct ath_hal *hal, u_int32_t phys_addr)
{
	AR5K_REG_WRITE(AR5K_AR5211_RXDP, phys_addr);
}

void
ar5k_ar5211_start_rx(struct ath_hal *hal)
{
	AR5K_REG_WRITE(AR5K_AR5211_CR, AR5K_AR5211_CR_RXE);
}

AR5K_BOOL
ar5k_ar5211_stop_rx_dma(struct ath_hal *hal)
{
	int i;

	AR5K_REG_WRITE(AR5K_AR5211_CR, AR5K_AR5211_CR_RXD);

	/*
	 * It may take some time to disable the DMA receive unit
	 */
	for (i = 2000;
	     i > 0 && (AR5K_REG_READ(AR5K_AR5211_CR) & AR5K_AR5211_CR_RXE) != 0;
	     i--)
		AR5K_DELAY(10);

	return (i > 0 ? TRUE : FALSE);
}

void
ar5k_ar5211_start_rx_pcu(struct ath_hal *hal)
{
	AR5K_REG_DISABLE_BITS(AR5K_AR5211_DIAG_SW, AR5K_AR5211_DIAG_SW_DIS_RX);
}

void
ar5k_ar5211_stop_pcu_recv(struct ath_hal *hal)
{
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_DIAG_SW, AR5K_AR5211_DIAG_SW_DIS_RX);
}

void
ar5k_ar5211_set_mcast_filter(struct ath_hal *hal, u_int32_t filter0,
    u_int32_t filter1)
{
	/* Set the multicat filter */
	AR5K_REG_WRITE(AR5K_AR5211_MCAST_FIL0, filter0);
	AR5K_REG_WRITE(AR5K_AR5211_MCAST_FIL1, filter1);
}

AR5K_BOOL
ar5k_ar5211_set_mcast_filterindex(struct ath_hal *hal, u_int32_t index)
{
	if (index >= 64) {
	    return (FALSE);
	} else if (index >= 32) {
	    AR5K_REG_ENABLE_BITS(AR5K_AR5211_MCAST_FIL1,
		(1 << (index - 32)));
	} else {
	    AR5K_REG_ENABLE_BITS(AR5K_AR5211_MCAST_FIL0,
		(1 << index));
	}

	return (TRUE);
}

AR5K_BOOL
ar5k_ar5211_clear_mcast_filter_idx(struct ath_hal *hal, u_int32_t index)
{

	if (index >= 64) {
	    return (FALSE);
	} else if (index >= 32) {
	    AR5K_REG_DISABLE_BITS(AR5K_AR5211_MCAST_FIL1,
		(1 << (index - 32)));
	} else {
	    AR5K_REG_DISABLE_BITS(AR5K_AR5211_MCAST_FIL0,
		(1 << index));
	}

	return (TRUE);
}

u_int32_t
ar5k_ar5211_get_rx_filter(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_RX_FILTER));
}

void
ar5k_ar5211_set_rx_filter(struct ath_hal *hal, u_int32_t filter)
{
	AR5K_REG_WRITE(AR5K_AR5211_RX_FILTER, filter);
}

AR5K_BOOL /*O.K. - Initialize rx_desc and clear ds_hw */
ar5k_ar5211_setup_rx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int32_t size, u_int flags)
{
	struct ar5k_ar5211_rx_desc *rx_desc;

	rx_desc = (struct ar5k_ar5211_rx_desc*)&desc->ds_ctl0;

	/*
	 * Clear ds_hw 
	 * If we don't clean the descriptor, while 
	 * scanning we get too many results, 
	 * most of them virtual, after some secs 
	 * of scanning system halts. M.F.
	 */
	bzero(desc->ds_hw, sizeof(desc->ds_hw));

	/*Initialize rx descriptor*/
	rx_desc->rx_control_0 = 0;
	rx_desc->rx_control_1 = 0;

	/*Setup descriptor*/

	if ((rx_desc->rx_control_1 = (size &
	    AR5K_AR5211_DESC_RX_CTL1_BUF_LEN)) != size)
		return (FALSE);

	if (flags & AR5K_RXDESC_INTREQ)
		rx_desc->rx_control_1 |= AR5K_AR5211_DESC_RX_CTL1_INTREQ;

	return (TRUE);
}

AR5K_STATUS
ar5k_ar5211_proc_rx_desc(struct ath_hal *hal, struct ath_desc *desc,
    u_int32_t phys_addr, struct ath_desc *next)
{
	struct ar5k_ar5211_rx_status *rx_status;

	rx_status = (struct ar5k_ar5211_rx_status*)&desc->ds_hw[0];

	/* No frame received / not ready */
	if ((rx_status->rx_status_1 & AR5K_AR5211_DESC_RX_STATUS1_DONE) == 0)
		return (AR5K_EINPROGRESS);

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
	    AR5K_AR5211_DESC_RX_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi =
	    AR5K_REG_MS(rx_status->rx_status_0,
	    AR5K_AR5211_DESC_RX_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate =
	    AR5K_REG_MS(rx_status->rx_status_0,
	    AR5K_AR5211_DESC_RX_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
	    AR5K_AR5211_DESC_RX_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
	    AR5K_AR5211_DESC_RX_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp =
	    AR5K_REG_MS(rx_status->rx_status_1,
	    AR5K_AR5211_DESC_RX_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 &
	    AR5K_AR5211_DESC_RX_STATUS1_KEY_INDEX_VALID) {
		desc->ds_us.rx.rs_keyix =
		    AR5K_REG_MS(rx_status->rx_status_1,
		    AR5K_AR5211_DESC_RX_STATUS1_KEY_INDEX);
	} else {
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;
	}

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 &
	    AR5K_AR5211_DESC_RX_STATUS1_FRAME_RECEIVE_OK) == 0) {
		if (rx_status->rx_status_1 &
		    AR5K_AR5211_DESC_RX_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
		    AR5K_AR5211_DESC_RX_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
			    AR5K_REG_MS(rx_status->rx_status_1,
			    AR5K_AR5211_DESC_RX_STATUS1_PHY_ERROR);
		}

		if (rx_status->rx_status_1 &
		    AR5K_AR5211_DESC_RX_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;
	}

	return (AR5K_OK);
}

void /*Added AR5K_NODE_STATS argument*/
ar5k_ar5211_set_rx_signal(struct ath_hal *hal, const AR5K_NODE_STATS *stats)
{
	/* Signal state monitoring is not yet supported */
}

/*
 * Misc functions
 */

void
ar5k_ar5211_dump_state(struct ath_hal *hal)
{
#ifdef AR5K_DEBUG
#define AR5K_PRINT_REGISTER(_x)						\
	AR5K_PRINTF("(%s: %08x) ", #_x, AR5K_REG_READ(AR5K_AR5211_##_x));

	AR5K_PRINT("MAC registers:\n");
	AR5K_PRINT_REGISTER(CR);
	AR5K_PRINT_REGISTER(CFG);
	AR5K_PRINT_REGISTER(IER);
	AR5K_PRINT_REGISTER(RTSD0);
	AR5K_PRINT_REGISTER(TXCFG);
	AR5K_PRINT_REGISTER(RXCFG);
	AR5K_PRINT_REGISTER(RXJLA);
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
	AR5K_PRINT_REGISTER(MCAST_FIL0);
	AR5K_PRINT_REGISTER(MCAST_FIL1);
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
	AR5K_PRINT_REGISTER(KEYTABLE_0);
	AR5K_PRINT("\n");

	AR5K_PRINT("PHY registers:\n");
	AR5K_PRINT_REGISTER(PHY_TURBO);
	AR5K_PRINT_REGISTER(PHY_AGC);
	AR5K_PRINT_REGISTER(PHY_CHIP_ID);
	AR5K_PRINT_REGISTER(PHY_AGCCTL);
	AR5K_PRINT_REGISTER(PHY_NF);
	AR5K_PRINT_REGISTER(PHY_RX_DELAY);
	AR5K_PRINT_REGISTER(PHY_IQ);
	AR5K_PRINT_REGISTER(PHY_PAPD_PROBE);
	AR5K_PRINT_REGISTER(PHY_FC);
	AR5K_PRINT_REGISTER(PHY_RADAR);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_0);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_1);
	AR5K_PRINT("\n");
#endif
}

AR5K_BOOL /*Added arguments*/
ar5k_ar5211_get_diag_state(struct ath_hal *hal, int request, const void *args, u_int32_t argsize, void **result, u_int32_t *resultsize)
{
	/*
	 * We'll ignore this right now. This seems to be some kind of an obscure
	 * debugging interface for the binary-only HAL.
	 */
	return (FALSE);
}

void
ar5k_ar5211_get_lladdr(struct ath_hal *hal, u_int8_t *mac)
{
	bcopy(hal->ah_sta_id, mac, IEEE80211_ADDR_LEN);
}

AR5K_BOOL
ar5k_ar5211_set_lladdr(struct ath_hal *hal, const u_int8_t *mac)
{
	u_int32_t low_id, high_id;

	/* Set new station ID */
	bcopy(mac, hal->ah_sta_id, IEEE80211_ADDR_LEN);

	low_id = AR5K_LOW_ID(mac);
	high_id = AR5K_HIGH_ID(mac);

	AR5K_REG_WRITE(AR5K_AR5211_STA_ID0, low_id);
	AR5K_REG_WRITE(AR5K_AR5211_STA_ID1, high_id);

	return (TRUE);
}

AR5K_BOOL
ar5k_ar5211_set_regdomain(struct ath_hal *hal, u_int16_t regdomain,
    AR5K_STATUS *status)
{
	ieee80211_regdomain_t ieee_regdomain;

	ieee_regdomain = ar5k_regdomain_to_ieee(regdomain);

	if (ar5k_eeprom_regulation_domain(hal, TRUE,
		&ieee_regdomain) == TRUE) {
		*status = AR5K_OK;
		return (TRUE);
	}

	*status = EIO;

	return (FALSE);
}

void
ar5k_ar5211_set_ledstate(struct ath_hal *hal, AR5K_LED_STATE state)
{
	u_int32_t led;

	AR5K_REG_DISABLE_BITS(AR5K_AR5211_PCICFG,
	    AR5K_AR5211_PCICFG_LEDMODE |  AR5K_AR5211_PCICFG_LED);

	/*
	 * Some blinking values, define at your wish
	 */
	switch (state) {
	case AR5K_LED_SCAN:
	case AR5K_LED_AUTH:
		led = AR5K_AR5211_PCICFG_LEDMODE_PROP |
		    AR5K_AR5211_PCICFG_LED_PEND;
		break;

	case AR5K_LED_INIT:
		led = AR5K_AR5211_PCICFG_LEDMODE_PROP |
		    AR5K_AR5211_PCICFG_LED_NONE;
		break;

	case AR5K_LED_ASSOC:
	case AR5K_LED_RUN:
		led = AR5K_AR5211_PCICFG_LEDMODE_PROP |
		    AR5K_AR5211_PCICFG_LED_ASSOC;
		break;

	default:
		led = AR5K_AR5211_PCICFG_LEDMODE_PROM |
		    AR5K_AR5211_PCICFG_LED_NONE;
		break;
	}

	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PCICFG, led);
}

void /*Removed argument trim_offset for combatibility -need revision*/
ar5k_ar5211_set_associd(struct ath_hal *hal, const u_int8_t *bssid,
    u_int16_t assoc_id)
{
	u_int32_t low_id, high_id;
	u_int16_t tim_offset = 0;

	/*
	 * Set BSSID which triggers the "SME Join" operation
	 */
	low_id = AR5K_LOW_ID(bssid);
	high_id = AR5K_HIGH_ID(bssid);
	AR5K_REG_WRITE(AR5K_AR5211_BSS_ID0, low_id);
	AR5K_REG_WRITE(AR5K_AR5211_BSS_ID1, high_id |
	    ((assoc_id & 0x3fff) << AR5K_AR5211_BSS_ID1_AID_S));
	bcopy(bssid, hal->ah_bssid, IEEE80211_ADDR_LEN);

	if (assoc_id == 0) {
		ar5k_ar5211_disable_pspoll(hal);
		return;
	}

	AR5K_REG_WRITE(AR5K_AR5211_BEACON,
	    (AR5K_REG_READ(AR5K_AR5211_BEACON) &
	    ~AR5K_AR5211_BEACON_TIM) |
	    (((tim_offset ? tim_offset + 4 : 0) <<
	    AR5K_AR5211_BEACON_TIM_S) &
	    AR5K_AR5211_BEACON_TIM));

	ar5k_ar5211_enable_pspoll(hal, NULL, 0);
}

AR5K_BOOL  /*New*/
ar5k_ar5211_set_bssid_mask(struct ath_hal *hal, const u_int8_t* mask)
{
	/*Not supported in 5211*/
	AR5K_TRACE; 
	return (FALSE); 
}

AR5K_BOOL
ar5k_ar5211_set_gpio_output(struct ath_hal *hal, u_int32_t gpio)
{
	if (gpio > AR5K_AR5211_NUM_GPIO)
		return (FALSE);

	AR5K_REG_WRITE(AR5K_AR5211_GPIOCR,
	    (AR5K_REG_READ(AR5K_AR5211_GPIOCR) &~ AR5K_AR5211_GPIOCR_ALL(gpio))
	    | AR5K_AR5211_GPIOCR_ALL(gpio));

	return (TRUE);
}

AR5K_BOOL
ar5k_ar5211_set_gpio_input(struct ath_hal *hal, u_int32_t gpio)
{
	if (gpio > AR5K_AR5211_NUM_GPIO)
		return (FALSE);

	AR5K_REG_WRITE(AR5K_AR5211_GPIOCR,
	    (AR5K_REG_READ(AR5K_AR5211_GPIOCR) &~ AR5K_AR5211_GPIOCR_ALL(gpio))
	    | AR5K_AR5211_GPIOCR_NONE(gpio));

	return (TRUE);
}

u_int32_t
ar5k_ar5211_get_gpio(struct ath_hal *hal, u_int32_t gpio)
{
	if (gpio > AR5K_AR5211_NUM_GPIO)
		return (0xffffffff);

	/* GPIO input magic */
	return (((AR5K_REG_READ(AR5K_AR5211_GPIODI) &
	    AR5K_AR5211_GPIODI_M) >> gpio) & 0x1);
}

AR5K_BOOL
ar5k_ar5211_set_gpio(struct ath_hal *hal, u_int32_t gpio, u_int32_t val)
{
	u_int32_t data;

	if (gpio > AR5K_AR5211_NUM_GPIO)
		return (FALSE);

	/* GPIO output magic */
	data =  AR5K_REG_READ(AR5K_AR5211_GPIODO);

	data &= ~(1 << gpio);
	data |= (val&1) << gpio;

	AR5K_REG_WRITE(AR5K_AR5211_GPIODO, data);

	return (TRUE);
}

void
ar5k_ar5211_set_gpio_intr(struct ath_hal *hal, u_int gpio,
    u_int32_t interrupt_level)
{
	u_int32_t data;

	if (gpio > AR5K_AR5211_NUM_GPIO)
		return;

	/*
	 * Set the GPIO interrupt
	 */
	data = (AR5K_REG_READ(AR5K_AR5211_GPIOCR) &
	    ~(AR5K_AR5211_GPIOCR_INT_SEL(gpio) | AR5K_AR5211_GPIOCR_INT_SELH |
	    AR5K_AR5211_GPIOCR_INT_ENA | AR5K_AR5211_GPIOCR_ALL(gpio))) |
	    (AR5K_AR5211_GPIOCR_INT_SEL(gpio) | AR5K_AR5211_GPIOCR_INT_ENA);

	AR5K_REG_WRITE(AR5K_AR5211_GPIOCR,
	    interrupt_level ? data : (data | AR5K_AR5211_GPIOCR_INT_SELH));

	hal->ah_imr |= AR5K_AR5211_PIMR_GPIO;

	/* Enable GPIO interrupts */
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_PIMR, AR5K_AR5211_PIMR_GPIO);
}

u_int32_t
ar5k_ar5211_get_tsf32(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_TSF_L32));
}

u_int64_t
ar5k_ar5211_get_tsf64(struct ath_hal *hal)
{
	u_int64_t tsf = AR5K_REG_READ(AR5K_AR5211_TSF_U32);

	return (AR5K_REG_READ(AR5K_AR5211_TSF_L32) | (tsf << 32));
}

void
ar5k_ar5211_reset_tsf(struct ath_hal *hal)
{
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_BEACON,
	    AR5K_AR5211_BEACON_RESET_TSF);
}

u_int16_t
ar5k_ar5211_get_regdomain(struct ath_hal *hal)
{
	return (ar5k_get_regdomain(hal));
}

AR5K_BOOL
ar5k_ar5211_detect_card_present(struct ath_hal *hal)
{
	u_int16_t magic;

	/*
	 * Checking the EEPROM's magic value could be an indication
	 * if the card is still present. I didn't find another suitable
	 * way to do this.
	 */
	if (ar5k_ar5211_eeprom_read(hal, AR5K_EEPROM_MAGIC, &magic) != 0)
		return (FALSE);

	return (magic == AR5K_EEPROM_MAGIC_VALUE ? TRUE : FALSE);
}

void
ar5k_ar5211_update_mib_counters(struct ath_hal *hal, AR5K_MIB_STATS *statistics)
{
	statistics->ackrcv_bad += AR5K_REG_READ(AR5K_AR5211_ACK_FAIL);
	statistics->rts_bad += AR5K_REG_READ(AR5K_AR5211_RTS_FAIL);
	statistics->rts_good += AR5K_REG_READ(AR5K_AR5211_RTS_OK);
	statistics->fcs_bad += AR5K_REG_READ(AR5K_AR5211_FCS_FAIL);
	statistics->beacons += AR5K_REG_READ(AR5K_AR5211_BEACON_CNT);
}

void /*Unimplemented*/
ar5k_ar5211_proc_mib_event(struct ath_hal *hal, const AR5K_NODE_STATS *stats) 
{
	AR5K_TRACE;
	return;
}

AR5K_RFGAIN
ar5k_ar5211_get_rf_gain(struct ath_hal *hal)
{
	return (AR5K_RFGAIN_INACTIVE);
}

AR5K_BOOL
ar5k_ar5211_set_slot_time(struct ath_hal *hal, u_int slot_time)
{
	if (slot_time < AR5K_SLOT_TIME_9 || slot_time > AR5K_SLOT_TIME_MAX)
		return (FALSE);

	AR5K_REG_WRITE(AR5K_AR5211_DCU_GBL_IFS_SLOT, slot_time);

	return (TRUE);
}

u_int
ar5k_ar5211_get_slot_time(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_DCU_GBL_IFS_SLOT) & 0xffff);
}

AR5K_BOOL
ar5k_ar5211_set_ack_timeout(struct ath_hal *hal, u_int timeout)
{
	if (ar5k_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_AR5211_TIME_OUT_ACK),
	    hal->ah_turbo) <= timeout)
		return (FALSE);

	AR5K_REG_WRITE_BITS(AR5K_AR5211_TIME_OUT, AR5K_AR5211_TIME_OUT_ACK,
	    ar5k_htoclock(timeout, hal->ah_turbo));

	return (TRUE);
}

u_int
ar5k_ar5211_get_ack_timeout(struct ath_hal *hal)
{
	return (ar5k_clocktoh(AR5K_REG_MS(AR5K_REG_READ(AR5K_AR5211_TIME_OUT),
	    AR5K_AR5211_TIME_OUT_ACK), hal->ah_turbo));
}

AR5K_BOOL
ar5k_ar5211_set_cts_timeout(struct ath_hal *hal, u_int timeout)
{
	if (ar5k_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_AR5211_TIME_OUT_CTS),
	    hal->ah_turbo) <= timeout)
		return (FALSE);

	AR5K_REG_WRITE_BITS(AR5K_AR5211_TIME_OUT, AR5K_AR5211_TIME_OUT_CTS,
	    ar5k_htoclock(timeout, hal->ah_turbo));

	return (TRUE);
}

u_int
ar5k_ar5211_get_cts_timeout(struct ath_hal *hal)
{
	return (ar5k_clocktoh(AR5K_REG_MS(AR5K_REG_READ(AR5K_AR5211_TIME_OUT),
	    AR5K_AR5211_TIME_OUT_CTS), hal->ah_turbo));
}

AR5K_STATUS /*New*/
ar5k_ar5211_get_capability(struct ath_hal *hal, AR5K_CAPABILITY_TYPE cap_type,
			   u_int32_t capability, u_int32_t *result) 
{
	AR5K_TRACE;

	switch (cap_type) {
	case AR5K_CAP_REG_DMN:
		if (result){
			*result = ar5k_get_regdomain(hal);
			goto yes;
		}
	case AR5K_CAP_CIPHER: 
		switch (capability) {
		case AR5K_CIPHER_WEP: goto yes;
		default:             goto no;
		}
	case AR5K_CAP_NUM_TXQUEUES: 
		if (result) {
			*result = AR5K_AR5211_TX_NUM_QUEUES;
			goto yes;
		}
	case AR5K_CAP_VEOL:
		goto yes;
	case AR5K_CAP_COMPRESSION:
		goto yes;
	case AR5K_CAP_BURST:
		goto yes;
	case AR5K_CAP_TPC:
		goto yes;
	case AR5K_CAP_BSSIDMASK:
		goto yes;
	case AR5K_CAP_XR:
		goto yes;
	default: 
		goto no;
	}

 no:
	return (AR5K_EINVAL);
 yes:
	return AR5K_OK;
	
}

AR5K_BOOL
ar5k_ar5211_set_capability(struct ath_hal *hal, AR5K_CAPABILITY_TYPE cap_type,
			   u_int32_t capability, u_int32_t setting, AR5K_STATUS *status) 
{

	AR5K_TRACE;
	if (status) {
		*status = AR5K_OK;
	}
	return (FALSE);
}

/*
 * Key table (WEP) functions
 */

AR5K_BOOL
ar5k_ar5211_is_cipher_supported(struct ath_hal *hal, AR5K_CIPHER cipher)
{
	/*
	 * The AR5211 only supports WEP
	 */
	if (cipher == AR5K_CIPHER_WEP)
		return (TRUE);

	return (FALSE);
}

u_int32_t
ar5k_ar5211_get_keycache_size(struct ath_hal *hal)
{
	return (AR5K_AR5211_KEYCACHE_SIZE);
}

AR5K_BOOL
ar5k_ar5211_reset_key(struct ath_hal *hal, u_int16_t entry)
{
	int i;

	AR5K_ASSERT_ENTRY(entry, AR5K_AR5211_KEYTABLE_SIZE);

	for (i = 0; i < AR5K_AR5211_KEYCACHE_SIZE; i++)
		AR5K_REG_WRITE(AR5K_AR5211_KEYTABLE_OFF(entry, i), 0);

	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_is_key_valid(struct ath_hal *hal, u_int16_t entry)
{
	AR5K_ASSERT_ENTRY(entry, AR5K_AR5211_KEYTABLE_SIZE);

	/*
	 * Check the validation flag at the end of the entry
	 */
	if (AR5K_REG_READ(AR5K_AR5211_KEYTABLE_MAC1(entry)) &
	    AR5K_AR5211_KEYTABLE_VALID)
		return (TRUE);

	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_set_key(struct ath_hal *hal, u_int16_t entry,
    const AR5K_KEYVAL *keyval, const u_int8_t *mac, int xor_notused)
{
	int i;
	u_int32_t key_v[AR5K_AR5211_KEYCACHE_SIZE - 2];

	AR5K_ASSERT_ENTRY(entry, AR5K_AR5211_KEYTABLE_SIZE);

	bzero(&key_v, sizeof(key_v));

	switch (keyval->wk_len) {
	case AR5K_KEYVAL_LENGTH_40:
		bcopy(keyval->wk_key, &key_v[0], 4);
		bcopy(keyval->wk_key + 4, &key_v[1], 1);
		key_v[5] = AR5K_AR5211_KEYTABLE_TYPE_40;
		break;

	case AR5K_KEYVAL_LENGTH_104:
		bcopy(keyval->wk_key, &key_v[0], 4);
		bcopy(keyval->wk_key + 4, &key_v[1], 2);
		bcopy(keyval->wk_key + 6, &key_v[2], 4);
		bcopy(keyval->wk_key + 10, &key_v[3], 2);
		bcopy(keyval->wk_key + 12, &key_v[4], 1);
		key_v[5] = AR5K_AR5211_KEYTABLE_TYPE_104;
		break;

	case AR5K_KEYVAL_LENGTH_128:
		bcopy(keyval->wk_key, &key_v[0], 4);
		bcopy(keyval->wk_key + 4, &key_v[1], 2);
		bcopy(keyval->wk_key + 6, &key_v[2], 4);
		bcopy(keyval->wk_key + 10, &key_v[3], 2);
		bcopy(keyval->wk_key + 12, &key_v[4], 4);
		key_v[5] = AR5K_AR5211_KEYTABLE_TYPE_128;
		break;

	default:
		/* Unsupported key length (not WEP40/104/128) */
		return (FALSE);
	}

	for (i = 0; i < AR5K_ELEMENTS(key_v); i++)
		AR5K_REG_WRITE(AR5K_AR5211_KEYTABLE_OFF(entry, i), key_v[i]);

	return (ar5k_ar5211_set_key_lladdr(hal, entry, mac));
}

AR5K_BOOL
ar5k_ar5211_set_key_lladdr(struct ath_hal *hal, u_int16_t entry,
    const u_int8_t *mac)
{
	u_int32_t low_id, high_id;
	const u_int8_t *mac_v;

	/*
	 * Invalid entry (key table overflow)
	 */
	AR5K_ASSERT_ENTRY(entry, AR5K_AR5211_KEYTABLE_SIZE);

	/* MAC may be NULL if it's a broadcast key */
	mac_v = mac == NULL ? etherbroadcastaddr : mac;

	low_id = AR5K_LOW_ID(mac_v);
	high_id = AR5K_HIGH_ID(mac_v);
	high_id |= AR5K_AR5211_KEYTABLE_VALID;

	AR5K_REG_WRITE(AR5K_AR5211_KEYTABLE_MAC0(entry), low_id);
	AR5K_REG_WRITE(AR5K_AR5211_KEYTABLE_MAC1(entry), high_id);

	return (TRUE);
}

/*
 * Power management functions
 */

AR5K_BOOL
ar5k_ar5211_set_power(struct ath_hal *hal, AR5K_POWER_MODE mode,
    AR5K_BOOL set_chip, u_int16_t sleep_duration)
{
	u_int32_t staid;
	int i;

	staid = AR5K_REG_READ(AR5K_AR5211_STA_ID1);

	switch (mode) {
	case AR5K_PM_AUTO:
		staid &= ~AR5K_AR5211_STA_ID1_DEFAULT_ANTENNA;
		/* fallthrough */
	case AR5K_PM_NETWORK_SLEEP:
		if (set_chip == TRUE) {
			AR5K_REG_WRITE(AR5K_AR5211_SCR,
			    AR5K_AR5211_SCR_SLE | sleep_duration);
		}
		staid |= AR5K_AR5211_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_FULL_SLEEP:
		if (set_chip == TRUE) {
			AR5K_REG_WRITE(AR5K_AR5211_SCR,
			    AR5K_AR5211_SCR_SLE_SLP);
		}
		staid |= AR5K_AR5211_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_AWAKE:
		if (set_chip == FALSE)
			goto commit;

		AR5K_REG_WRITE(AR5K_AR5211_SCR, AR5K_AR5211_SCR_SLE_WAKE);

		for (i = 5000; i > 0; i--) {
			/* Check if the AR5211 did wake up */
			if ((AR5K_REG_READ(AR5K_AR5211_PCICFG) &
			    AR5K_AR5211_PCICFG_SPWR_DN) == 0)
				break;

			/* Wait a bit and retry */
			AR5K_DELAY(200);
			AR5K_REG_WRITE(AR5K_AR5211_SCR,
			    AR5K_AR5211_SCR_SLE_WAKE);
		}

		/* Fail if the AR5211 didn't wake up */
		if (i <= 0)
			return (FALSE);

		staid &= ~AR5K_AR5211_STA_ID1_PWR_SV;
		break;

	default:
		return (FALSE);
	}

 commit:
	hal->ah_power_mode = mode;

	AR5K_REG_WRITE(AR5K_AR5211_STA_ID1, staid);

	return (TRUE);
}

AR5K_POWER_MODE
ar5k_ar5211_get_power_mode(struct ath_hal *hal)
{
	return (hal->ah_power_mode);
}

AR5K_BOOL
ar5k_ar5211_query_pspoll_support(struct ath_hal *hal)
{
	/* nope */
	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_init_pspoll(struct ath_hal *hal)
{
	/*
	 * Not used on the AR5211
	 */
	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_enable_pspoll(struct ath_hal *hal, u_int8_t *bssid,
    u_int16_t assoc_id)
{
	return (FALSE);
}

AR5K_BOOL
ar5k_ar5211_disable_pspoll(struct ath_hal *hal)
{
	return (FALSE);
}

AR5K_BOOL /*Unimplemented*/
ar5k_ar5211_set_txpower_limit(struct ath_hal *hal, u_int32_t power)
{
//	AR5K_CHANNEL *channel = &hal->ah_current_channel;

	AR5K_TRACE;
	AR5K_PRINTF("changing txpower to %d\n unimplemented ;-(",power);
	return FALSE;
}

/*
 * Beacon functions
 */

void
ar5k_ar5211_init_beacon(struct ath_hal *hal, u_int32_t next_beacon,
    u_int32_t interval)
{
	u_int32_t timer1, timer2, timer3;

	/*
	 * Set the additional timers by mode
	 */
	switch (hal->ah_op_mode) {
	case AR5K_M_STA:
		timer1 = 0x0000ffff;
		timer2 = 0x0007ffff;
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
	 * Enable all timers and set the beacon register
	 * (next beacon, DMA beacon, software beacon, ATIM window time)
	 */
	AR5K_REG_WRITE(AR5K_AR5211_TIMER0, next_beacon);
	AR5K_REG_WRITE(AR5K_AR5211_TIMER1, timer1);
	AR5K_REG_WRITE(AR5K_AR5211_TIMER2, timer2);
	AR5K_REG_WRITE(AR5K_AR5211_TIMER3, timer3);

	AR5K_REG_WRITE(AR5K_AR5211_BEACON, interval &
	    (AR5K_AR5211_BEACON_PERIOD | AR5K_AR5211_BEACON_RESET_TSF |
	    AR5K_AR5211_BEACON_ENABLE));
}

void /*Removed arguments - should be changed through *state - review AR5K_BEACON_STATE struct*/
ar5k_ar5211_set_beacon_timers(struct ath_hal *hal, const AR5K_BEACON_STATE *state)
{
	u_int32_t cfp_period, next_cfp;

	u_int32_t dtim_count = 0; /* XXX */
	u_int32_t cfp_count = 0; /* XXX */
	u_int32_t tsf = 0; /* XXX */

	/* Return on an invalid beacon state */
	if (state->bs_interval < 1)
		return;

	/*
	 * PCF support?
	 */
	if (state->bs_cfp_period > 0) {
		/* Enable CFP mode and set the CFP and timer registers */
		cfp_period = state->bs_cfp_period * state->bs_dtim_period *
		    state->bs_interval;
		next_cfp = (cfp_count * state->bs_dtim_period + dtim_count) *
		    state->bs_interval;

		AR5K_REG_DISABLE_BITS(AR5K_AR5211_STA_ID1,
		    AR5K_AR5211_STA_ID1_DEFAULT_ANTENNA |
		    AR5K_AR5211_STA_ID1_PCF);
		AR5K_REG_WRITE(AR5K_AR5211_CFP_PERIOD, cfp_period);
		AR5K_REG_WRITE(AR5K_AR5211_CFP_DUR, state->bs_cfp_max_duration);
		AR5K_REG_WRITE(AR5K_AR5211_TIMER2,
		    (tsf + (next_cfp == 0 ? cfp_period : next_cfp)) << 3);
	} else {
		/* Disable PCF mode */
		AR5K_REG_DISABLE_BITS(AR5K_AR5211_STA_ID1,
		    AR5K_AR5211_STA_ID1_DEFAULT_ANTENNA |
		    AR5K_AR5211_STA_ID1_PCF);
	}

	/*
	 * Enable the beacon timer register
	 */
	AR5K_REG_WRITE(AR5K_AR5211_TIMER0, state->bs_next_beacon);

	/*
	 * Start the beacon timers
	 */
	AR5K_REG_WRITE(AR5K_AR5211_BEACON,
	    (AR5K_REG_READ(AR5K_AR5211_BEACON) &~
	    (AR5K_AR5211_BEACON_PERIOD | AR5K_AR5211_BEACON_TIM)) |
	    AR5K_REG_SM(state->bs_tim_offset ? state->bs_tim_offset + 4 : 0,
	    AR5K_AR5211_BEACON_TIM) | AR5K_REG_SM(state->bs_interval,
	    AR5K_AR5211_BEACON_PERIOD));

	/*
	 * Write new beacon miss threshold, if it appears to be valid
	 */
	if ((AR5K_AR5211_RSSI_THR_BMISS >> AR5K_AR5211_RSSI_THR_BMISS_S) <
	    state->bs_bmiss_threshold)
		return;

	AR5K_REG_WRITE_BITS(AR5K_AR5211_RSSI_THR_M,
	    AR5K_AR5211_RSSI_THR_BMISS, state->bs_bmiss_threshold);
	AR5K_REG_WRITE_BITS(AR5K_AR5211_SCR, AR5K_AR5211_SCR_SLDUR,
	    (state->bs_sleep_duration - 3) << 3);
}

void
ar5k_ar5211_reset_beacon(struct ath_hal *hal)
{
	/*
	 * Disable beacon timer
	 */
	AR5K_REG_WRITE(AR5K_AR5211_TIMER0, 0);

	/*
	 * Disable some beacon register values
	 */
	AR5K_REG_DISABLE_BITS(AR5K_AR5211_STA_ID1,
	    AR5K_AR5211_STA_ID1_DEFAULT_ANTENNA | AR5K_AR5211_STA_ID1_PCF);
	AR5K_REG_WRITE(AR5K_AR5211_BEACON, AR5K_AR5211_BEACON_PERIOD);
}

AR5K_BOOL
ar5k_ar5211_wait_for_beacon(struct ath_hal *hal, AR5K_BUS_ADDR phys_addr)
{
	AR5K_BOOL ret;

	/*
	 * Wait for beaconn queue to be done
	 */
	ret = ar5k_register_timeout(hal,
	    AR5K_AR5211_QCU_STS(AR5K_TX_QUEUE_ID_BEACON),
	    AR5K_AR5211_QCU_STS_FRMPENDCNT, 0, FALSE);

	if (AR5K_REG_READ_Q(AR5K_AR5211_QCU_TXE, AR5K_TX_QUEUE_ID_BEACON))
		return (FALSE);

	return (ret);
}

/*
 * Interrupt handling
 */

AR5K_BOOL
ar5k_ar5211_is_intr_pending(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_INTPEND) == 0 ? FALSE : TRUE);
}

AR5K_BOOL
ar5k_ar5211_get_isr(struct ath_hal *hal, u_int32_t *interrupt_mask)
{
	u_int32_t data;

	/*
	 * Read interrupt status from the Read-And-Clear shadow register
	 */
	data = AR5K_REG_READ(AR5K_AR5211_RAC_PISR);

	/*
	 * Get abstract interrupt mask (HAL-compatible)
	 */
	*interrupt_mask = (data & AR5K_INT_COMMON) & hal->ah_imr;

	if (data == AR5K_INT_NOCARD)
		return (FALSE);

	if (data & (AR5K_AR5211_PISR_RXOK | AR5K_AR5211_PISR_RXERR))
		*interrupt_mask |= AR5K_INT_RX;

	if (data & (AR5K_AR5211_PISR_TXOK | AR5K_AR5211_PISR_TXERR))
		*interrupt_mask |= AR5K_INT_TX;

	if (data & (AR5K_AR5211_PISR_HIUERR))
		*interrupt_mask |= AR5K_INT_FATAL;

	/*
	 * Special interrupt handling (not caught by the driver)
	 */
	if (((*interrupt_mask) & AR5K_AR5211_PISR_RXPHY) &&
	    hal->ah_radar.r_enabled == TRUE)
		ar5k_radar_alert(hal);

	return (TRUE);
}

u_int32_t
ar5k_ar5211_get_intr(struct ath_hal *hal)
{
	/* Return the interrupt mask stored previously */
	return (hal->ah_imr);
}

AR5K_INT
ar5k_ar5211_set_intr(struct ath_hal *hal, AR5K_INT new_mask)
{
	AR5K_INT old_mask, int_mask;

	/*
	 * Disable card interrupts to prevent any race conditions
	 * (they will be re-enabled afterwards).
	 */
	AR5K_REG_WRITE(AR5K_AR5211_IER, AR5K_AR5211_IER_DISABLE);

	old_mask = hal->ah_imr;

	/*
	 * Add additional, chipset-dependent interrupt mask flags
	 * and write them to the IMR (interrupt mask register).
	 */
	int_mask = new_mask & AR5K_INT_COMMON;

	if (new_mask & AR5K_INT_RX)
		int_mask |=
		    AR5K_AR5211_PIMR_RXOK |
		    AR5K_AR5211_PIMR_RXERR |
		    AR5K_AR5211_PIMR_RXORN |
		    AR5K_AR5211_PIMR_RXDESC;

	if (new_mask & AR5K_INT_TX)
		int_mask |=
		    AR5K_AR5211_PIMR_TXOK |
		    AR5K_AR5211_PIMR_TXERR |
		    AR5K_AR5211_PIMR_TXDESC |
		    AR5K_AR5211_PIMR_TXURN;

	if (new_mask & AR5K_INT_FATAL) {
		int_mask |= AR5K_AR5211_PIMR_HIUERR;
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_SIMR2,
		    AR5K_AR5211_SIMR2_MCABT |
		    AR5K_AR5211_SIMR2_SSERR |
		    AR5K_AR5211_SIMR2_DPERR);
	}

	AR5K_REG_WRITE(AR5K_AR5211_PIMR, int_mask);

	/* Store new interrupt mask */
	hal->ah_imr = new_mask;

	/* ..re-enable interrupts */
	AR5K_REG_WRITE(AR5K_AR5211_IER, AR5K_AR5211_IER_ENABLE);

	return (old_mask);
}

/*
 * Misc internal functions
 */

AR5K_BOOL
ar5k_ar5211_get_capabilities(struct ath_hal *hal)
{
	u_int16_t ee_header;

	/* Capabilities stored in the EEPROM */
	ee_header = hal->ah_capabilities.cap_eeprom.ee_header;

	/*
	 * XXX The AR5211 tranceiver supports frequencies from 4920 to 6100GHz
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
		hal->ah_capabilities.cap_mode = AR5K_MODE_11A | AR5K_MODE_TURBO;
	}

	/* This chip will support 802.11b if the 2GHz radio is connected */
	if (AR5K_EEPROM_HDR_11B(ee_header) || AR5K_EEPROM_HDR_11G(ee_header)) {
		hal->ah_capabilities.cap_range.range_2ghz_min = 2412; /* 2312 */
		hal->ah_capabilities.cap_range.range_2ghz_max = 2732;

		if (AR5K_EEPROM_HDR_11B(ee_header))
			hal->ah_capabilities.cap_mode |= AR5K_MODE_11B;
#if 0
		if (AR5K_EEPROM_HDR_11G(ee_header))
			hal->ah_capabilities.cap_mode |= AR5K_MODE_11G;
#endif
	}

	/* GPIO */
	hal->ah_gpio_npins = AR5K_AR5211_NUM_GPIO;

	/* Set number of supported TX queues */
	hal->ah_capabilities.cap_queues.q_tx_num = AR5K_AR5211_TX_NUM_QUEUES;

	return (TRUE);
}

void
ar5k_ar5211_radar_alert(struct ath_hal *hal, AR5K_BOOL enable)
{
	/*
	 * Enable radar detection
	 */
	AR5K_REG_WRITE(AR5K_AR5211_IER, AR5K_AR5211_IER_DISABLE);

	if (enable == TRUE) {
		AR5K_REG_WRITE(AR5K_AR5211_PHY_RADAR,
		    AR5K_AR5211_PHY_RADAR_ENABLE);
		AR5K_REG_ENABLE_BITS(AR5K_AR5211_PIMR,
		    AR5K_AR5211_PIMR_RXPHY);
	} else {
		AR5K_REG_WRITE(AR5K_AR5211_PHY_RADAR,
		    AR5K_AR5211_PHY_RADAR_DISABLE);
		AR5K_REG_DISABLE_BITS(AR5K_AR5211_PIMR,
		    AR5K_AR5211_PIMR_RXPHY);
	}

	AR5K_REG_WRITE(AR5K_AR5211_IER, AR5K_AR5211_IER_ENABLE);
}

/*
 * EEPROM access functions
 */

AR5K_BOOL
ar5k_ar5211_eeprom_is_busy(struct ath_hal *hal)
{
	return (AR5K_REG_READ(AR5K_AR5211_CFG) & AR5K_AR5211_CFG_EEBS ?
	    TRUE : FALSE);
}

int
ar5k_ar5211_eeprom_read(struct ath_hal *hal, u_int32_t offset, u_int16_t *data)
{
	u_int32_t status, i;

	/*
	 * Initialize EEPROM access
	 */
	AR5K_REG_WRITE(AR5K_AR5211_EEPROM_BASE, (u_int8_t)offset);
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_EEPROM_CMD,
	    AR5K_AR5211_EEPROM_CMD_READ);

	for (i = AR5K_TUNE_REGISTER_TIMEOUT; i > 0; i--) {
		status = AR5K_REG_READ(AR5K_AR5211_EEPROM_STATUS);
		if (status & AR5K_AR5211_EEPROM_STAT_RDDONE) {
			if (status & AR5K_AR5211_EEPROM_STAT_RDERR)
				return (EIO);
			*data = (u_int16_t)
			    (AR5K_REG_READ(AR5K_AR5211_EEPROM_DATA) & 0xffff);
			return (0);
		}
		AR5K_DELAY(15);
	}

	return (ETIMEDOUT);
}

int
ar5k_ar5211_eeprom_write(struct ath_hal *hal, u_int32_t offset, u_int16_t data)
{
	u_int32_t status, timeout;

	/* Enable eeprom access */
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_EEPROM_CMD,
	    AR5K_AR5211_EEPROM_CMD_RESET);
	AR5K_REG_ENABLE_BITS(AR5K_AR5211_EEPROM_CMD,
	    AR5K_AR5211_EEPROM_CMD_WRITE);

	/*
	 * Prime write pump
	 */
	AR5K_REG_WRITE(AR5K_AR5211_EEPROM_BASE, (u_int8_t)offset - 1);

	for (timeout = 10000; timeout > 0; timeout--) {
		AR5K_DELAY(1);
		status = AR5K_REG_READ(AR5K_AR5211_EEPROM_STATUS);
		if (status & AR5K_AR5211_EEPROM_STAT_WRDONE) {
			if (status & AR5K_AR5211_EEPROM_STAT_WRERR)
				return (EIO);
			return (0);
		}
	}

	return (ETIMEDOUT);
}

/*
 * RF register settings
 */

void
ar5k_ar5211_rfregs(struct ath_hal *hal, AR5K_CHANNEL *channel, u_int freq,
    u_int ee_mode)
{
	struct ar5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	struct ar5k_ar5211_ini_rf rf[AR5K_ELEMENTS(ar5211_rf)];
	u_int32_t ob, db, obdb, xpds, xpdp, x_gain;
	u_int i;

	bcopy(ar5211_rf, rf, sizeof(rf));
	obdb = 0;

	if (freq == AR5K_INI_RFGAIN_2GHZ &&
	    hal->ah_ee_version >= AR5K_EEPROM_VERSION_3_1) {
		ob = ar5k_bitswap(ee->ee_ob[ee_mode][0], 3);
		db = ar5k_bitswap(ee->ee_db[ee_mode][0], 3);
		rf[25].rf_value[freq] =
		    ((ob << 6) & 0xc0) | (rf[25].rf_value[freq] & ~0xc0);
		rf[26].rf_value[freq] =
		    (((ob >> 2) & 0x1) | ((db << 1) & 0xe)) |
		    (rf[26].rf_value[freq] & ~0xf);
	}

	if (freq == AR5K_INI_RFGAIN_5GHZ) {
		/* For 11a and Turbo */
		obdb = channel->freq >= 5725 ? 3 :
		    (channel->freq >= 5500 ? 2 :
			(channel->freq >= 5260 ? 1 :
			    (channel->freq > 4000 ? 0 : -1)));
	}

	ob = ee->ee_ob[ee_mode][obdb];
	db = ee->ee_db[ee_mode][obdb];
	x_gain = ee->ee_x_gain[ee_mode];
	xpds = ee->ee_xpd[ee_mode];
	xpdp = !xpds;

	rf[11].rf_value[freq] = (rf[11].rf_value[freq] & ~0xc0) |
		(((ar5k_bitswap(x_gain, 4) << 7) | (xpdp << 6)) & 0xc0);
	rf[12].rf_value[freq] = (rf[12].rf_value[freq] & ~0x7) |
		((ar5k_bitswap(x_gain, 4) >> 1) & 0x7);
	rf[12].rf_value[freq] = (rf[12].rf_value[freq] & ~0x80) |
		((ar5k_bitswap(ob, 3) << 7) & 0x80);
	rf[13].rf_value[freq] = (rf[13].rf_value[freq] & ~0x3) |
		((ar5k_bitswap(ob, 3) >> 1) & 0x3);
	rf[13].rf_value[freq] = (rf[13].rf_value[freq] & ~0x1c) |
		((ar5k_bitswap(db, 3) << 2) & 0x1c);
	rf[17].rf_value[freq] = (rf[17].rf_value[freq] & ~0x8) |
		((xpds << 3) & 0x8);

	for (i = 0; i < AR5K_ELEMENTS(rf); i++) {
		AR5K_REG_WAIT(i);
		AR5K_REG_WRITE((u_int32_t)rf[i].rf_register,
		    rf[i].rf_value[freq]);
	}

	hal->ah_rf_gain = AR5K_RFGAIN_INACTIVE;
}
