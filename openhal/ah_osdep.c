/*
 * Copyright (c) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
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

#include "ah.h"

/*Definitions for module loading/unloading 
 *combatible with 2.4 and 2.6 kernels*/

static char *dev_info = "ath_hal";

MODULE_AUTHOR("Nick Kossifidis");
MODULE_DESCRIPTION("OpenHAL");
MODULE_SUPPORTED_DEVICE("Atheros AR5xxx WLAN cards");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

/*Attach/Dettach to HAL*/

struct ath_hal *
_ath_hal_attach(u_int16_t devid, AR5K_SOFTC sc,
                AR5K_BUS_TAG t, AR5K_BUS_HANDLE h, void* s)
{
        AR5K_STATUS status;
        struct ath_hal *ah = ath5k_hw_init(devid, sc, t, h, &status);

        *(AR5K_STATUS *)s = status;
        if (ah)
#ifndef __MOD_INC_USE_COUNT
		if (!try_module_get(THIS_MODULE)) {
			printk(KERN_WARNING "try_module_get failed\n");
			ath_hal_detach(ah);
			return NULL;
		}
#else
		MOD_INC_USE_COUNT;
#endif

        return ah;
}

void
ath_hal_detach(struct ath_hal *ah)
{
        ath5k_hw_detach(ah);
#ifndef __MOD_INC_USE_COUNT
	module_put(THIS_MODULE);
#else
	MOD_DEC_USE_COUNT;
#endif
}

EXPORT_SYMBOL(ath_hal_probe);
EXPORT_SYMBOL(_ath_hal_attach);
EXPORT_SYMBOL(ath_hal_detach);
EXPORT_SYMBOL(ath_hal_init_channels);
EXPORT_SYMBOL(ath_hal_getwirelessmodes);
EXPORT_SYMBOL(ath_hal_computetxtime);
EXPORT_SYMBOL(ath_hal_mhz2ieee);
EXPORT_SYMBOL(ath_hal_ieee2mhz);

EXPORT_SYMBOL(ath5k_hw_setup_xr_tx_desc);
EXPORT_SYMBOL(ath5k_hw_proc_2word_tx_status);
EXPORT_SYMBOL(ath5k_hw_put_tx_buf);
EXPORT_SYMBOL(ath5k_hw_tx_start);
EXPORT_SYMBOL(ath5k_hw_get_lladdr);
EXPORT_SYMBOL(ath5k_hw_setup_tx_queueprops);
EXPORT_SYMBOL(ath5k_hw_put_rx_buf);
EXPORT_SYMBOL(ath5k_hw_setup_2word_tx_desc);
EXPORT_SYMBOL(ath5k_hw_init_beacon);
EXPORT_SYMBOL(ath5k_hw_set_cts_timeout);
EXPORT_SYMBOL(ath5k_hw_get_ack_timeout);
EXPORT_SYMBOL(ath5k_hw_get_tx_queueprops);
EXPORT_SYMBOL(ath5k_hw_set_key);
EXPORT_SYMBOL(ath5k_hw_set_gpio);
EXPORT_SYMBOL(ath5k_hw_get_slot_time);
EXPORT_SYMBOL(ath5k_hw_setup_4word_tx_desc);
EXPORT_SYMBOL(ath5k_hw_start_rx);
EXPORT_SYMBOL(ath5k_hw_phy_disable);
EXPORT_SYMBOL(ath5k_hw_set_beacon_timers);
EXPORT_SYMBOL(ath5k_hw_set_capability);
EXPORT_SYMBOL(ath5k_hw_release_tx_queue);
EXPORT_SYMBOL(ath5k_hw_get_rate_table);
EXPORT_SYMBOL(ath5k_hw_set_txpower_limit);
EXPORT_SYMBOL(ath5k_hw_get_capability);
EXPORT_SYMBOL(ath5k_hw_phy_calibrate);
EXPORT_SYMBOL(ath5k_hw_reset_tx_queue);
EXPORT_SYMBOL(ath5k_hw_setup_rx_desc);
EXPORT_SYMBOL(ath5k_hw_is_key_valid);
EXPORT_SYMBOL(ath5k_hw_get_rx_filter);
EXPORT_SYMBOL(ath5k_hw_proc_new_rx_status);
EXPORT_SYMBOL(ath5k_hw_set_slot_time);
EXPORT_SYMBOL(ath5k_hw_reset);
EXPORT_SYMBOL(ath5k_hw_set_ack_timeout);
EXPORT_SYMBOL(ath5k_hw_proc_old_rx_status);
EXPORT_SYMBOL(ath5k_hw_get_keycache_size);
EXPORT_SYMBOL(ath5k_hw_get_tx_buf);
EXPORT_SYMBOL(ath5k_hw_get_rf_gain);
EXPORT_SYMBOL(ath5k_hw_proc_mib_event);
EXPORT_SYMBOL(ath5k_hw_fill_4word_tx_desc);
EXPORT_SYMBOL(ath5k_hw_set_def_antenna);
EXPORT_SYMBOL(ath5k_hw_set_power);
EXPORT_SYMBOL(ath5k_hw_get_def_antenna);
EXPORT_SYMBOL(ath5k_hw_stop_pcu_recv);
EXPORT_SYMBOL(ath5k_hw_set_gpio_output);
EXPORT_SYMBOL(ath5k_hw_stop_tx_dma);
EXPORT_SYMBOL(ath5k_hw_get_cts_timeout);
EXPORT_SYMBOL(ath5k_hw_set_intr);
EXPORT_SYMBOL(ath5k_hw_start_rx_pcu);
EXPORT_SYMBOL(ath5k_hw_update_tx_triglevel);
EXPORT_SYMBOL(ath5k_hw_set_key_lladdr);
EXPORT_SYMBOL(ath5k_hw_set_ledstate);
EXPORT_SYMBOL(ath5k_hw_stop_rx_dma);
EXPORT_SYMBOL(ath5k_hw_get_diag_state);
EXPORT_SYMBOL(ath5k_hw_set_mcast_filter);
EXPORT_SYMBOL(ath5k_hw_setup_tx_queue);
EXPORT_SYMBOL(ath5k_hw_set_rx_signal_monitor);
EXPORT_SYMBOL(ath5k_hw_set_opmode);
EXPORT_SYMBOL(ath5k_hw_reset_key);
EXPORT_SYMBOL(ath5k_hw_is_intr_pending);
EXPORT_SYMBOL(ath5k_hw_set_associd);
EXPORT_SYMBOL(ath5k_hw_proc_4word_tx_status);
EXPORT_SYMBOL(ath5k_hw_get_isr);
EXPORT_SYMBOL(ath5k_hw_get_tsf32);
EXPORT_SYMBOL(ath5k_hw_fill_2word_tx_desc);
EXPORT_SYMBOL(ath5k_hw_num_tx_pending);
EXPORT_SYMBOL(ath5k_hw_get_tsf64);
EXPORT_SYMBOL(ath5k_hw_set_rx_filter);
EXPORT_SYMBOL(ath5k_hw_set_lladdr);
EXPORT_SYMBOL(ath5k_hw_get_rx_buf);


static int __init
init_ath_hal(void)
{
	printk(KERN_INFO "%s: OpenHAL loaded (AR5210, AR5211, AR5212, RF5110/1/2)\n", dev_info);
	return (0);
}
module_init(init_ath_hal);

static void __exit
exit_ath_hal(void)
{
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_hal);
