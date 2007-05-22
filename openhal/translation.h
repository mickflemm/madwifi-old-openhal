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

/*
 * Init/Exit functions
 */
#define ah_getRateTable		ah_get_rate_table
//detach

/*
 * Reset functions
 */
//reset
//set_opmode
#define ah_perCalibration	ah_phy_calibrate
#define ah_phyDisable		ah_phy_disable

/*
 * TX functions
 */
#define ah_updateTxTrigLevel	ah_update_tx_triglevel
#define ah_setupTxQueue		ah_setup_tx_queue
#define ah_setTxQueueProps	ah_setup_tx_queueprops
#define ah_getTxQueueProps	ah_get_tx_queueprops
#define ah_releaseTxQueue	ah_release_tx_queue
#define ah_resetTxQueue		ah_reset_tx_queue
#define ah_getTxDP		ah_get_tx_buf
#define ah_setTxDP		ah_put_tx_buf
#define ah_numTxPending	ah_num_tx_pending
#define ah_startTxDma		ah_tx_start
#define ah_stopTxDma		ah_stop_tx_dma
#define ah_setupTxDesc		ah_setup_tx_desc
#define ah_setupXTxDesc		ah_setup_xtx_desc
#define ah_fillTxDesc 		ah_fill_tx_desc
#define ah_procTxDesc		ah_proc_tx_desc
#define ah_getTxIntrQueue	ah_get_tx_inter_queue
//has_veol

/*
 * RX functions
 */
#define ah_getRxDP		ah_get_rx_buf
#define ah_setRxDP		ah_put_rx_buf
#define ah_enableReceive	ah_start_rx
#define ah_stopDmaReceive	ah_stop_rx_dma
#define ah_startPcuReceive	ah_start_rx_pcu
#define ah_stopPcuReceive	ah_stop_pcu_recv
#define ah_setMulticastFilter	ah_set_mcast_filter
#define ah_setMulticastFilterIndex	ah_set_mcast_filterindex
#define ah_clrMulticastFilterIndex	ah_clear_mcast_filter_idx
#define ah_getRxFilter		ah_get_rx_filter
#define ah_setRxFilter		ah_set_rx_filter
#define ah_setupRxDesc		ah_setup_rx_desc
#define ah_procRxDesc		ah_proc_rx_desc
#define ah_rxMonitor		ah_set_rx_signal
#define ah_procMibEvent		ah_proc_mib_event

/*
 * Misc functions
 */
#define ah_getCapability	ah_get_capability
#define ah_setCapability	ah_set_capability
//dump_state
#define ah_getDiagState		ah_get_diag_state
#define ah_getMacAddress	ah_get_lladdr
#define ah_setMacAddress	ah_set_lladdr
#define ah_setBssIdMask		ah_set_bssid_mask
//#define ah_getBssIdMask	ah_get_bssid_mask
#define ah_setRegulatoryDomain	ah_set_regdomain
#define ah_setLedState		ah_set_ledstate
#define ah_writeAssocid		ah_set_associd
#define ah_gpioCfgInput		ah_set_gpio_input
#define ah_gpioCfgOutput	ah_set_gpio_output
#define ah_gpioGet		ah_get_gpio
#define ah_gpioSet		ah_set_gpio
#define ah_gpioSetIntr		ah_set_gpio_intr
#define ah_getTsf32		ah_get_tsf32
#define ah_getTsf64		ah_get_tsf64
#define ah_resetTsf		ah_reset_tsf
//get_regdomain
#define ah_detectCardPresent 	ah_detect_card_present
#define ah_updateMibCounters	ah_update_mib_counters
#define ah_getRfGain		ah_get_rf_gain
#define ah_getDefAntenna	ah_get_def_antenna
#define ah_setDefAntenna	ah_set_def_antenna
#define ah_setSlotTime		ah_set_slot_time
#define ah_getSlotTime		ah_get_slot_time
#define ah_setAckTimeout	ah_set_ack_timeout
#define ah_getAckTimeout	ah_get_ack_timeout
#define ah_setCTSTimeout	ah_set_cts_timeout
#define ah_getCTSTimeout	ah_get_cts_timeout

/*
 * Key table (WEP) functions
 */
//is_cipher_supported
#define ah_getKeyCacheSize	ah_get_keycache_size
#define ah_resetKeyCacheEntry	ah_reset_key
#define ah_isKeyCacheEntryValid	ah_is_key_valid
#define ah_setKeyCacheEntry	ah_set_key
#define ah_setKeyCacheEntryMac	ah_set_key_lladdr

/*
 * Power management functions
 */
#define ah_setPowerMode		ah_set_power
#define ah_getPowerMode		ah_get_power_mode
//query_pspoll_support
#define ah_initPSPoll 		ah_init_pspoll
#define ah_enablePSPoll		ah_enable_pspoll
#define ah_disablePSPoll	ah_disable_pspoll
#define ah_setTxPowerLimit	ah_set_txpower_limit

/*
 * Beacon functions
 */
#define ah_beaconInit			ah_init_beacon
#define ah_setStationBeaconTimers	ah_set_beacon_timers
#define ah_resetStationBeaconTimers	ah_reset_beacon
#define ah_waitForBeaconDone		ah_wait_for_beacon

/*
 * Interrupt functions
 */
#define ah_isInterruptPending		ah_is_intr_pending
#define ah_getPendingInterrupts		ah_get_isr
#define ah_getInterrupts		ah_get_intr
#define ah_setInterrupts		ah_set_intr

/*
 * Chipset functions (ar5k-specific, non-HAL)
 */
//get_capabilities
#define ah_radarlert	ah_radar_alert

/*
 * EEPROM access
 */
//eeprom_is_busy
//eeprom_read
//eeprom_write

#define ah_setPCUConfig		ah_set_pcu_config
