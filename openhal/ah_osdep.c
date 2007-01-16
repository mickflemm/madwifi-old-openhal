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

#ifndef __MOD_INC_USE_COUNT
#define AH_MOD_INC_USE_COUNT(_m)                                        \
        if (!try_module_get(_m)) {                                      \
                printk(KERN_WARNING "try_module_get failed\n");         \
                return NULL;                                            \
        }
#define AH_MOD_DEC_USE_COUNT(_m)        module_put(_m)
#else
#define AH_MOD_INC_USE_COUNT(_m)        MOD_INC_USE_COUNT
#define AH_MOD_DEC_USE_COUNT(_m)        MOD_DEC_USE_COUNT
#endif

static char *dev_info = "ath_hal";

MODULE_AUTHOR("Nick Kossifidis");
MODULE_DESCRIPTION("OpenHAL");
MODULE_SUPPORTED_DEVICE("");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

/*Attach/Dettach to HAL*/

struct ath_hal *
_ath_hal_attach(u_int16_t devid, AR5K_SOFTC sc,
                AR5K_BUS_TAG t, AR5K_BUS_HANDLE h, void* s)
{
        AR5K_STATUS status;
        struct ath_hal *ah = ath_hal_attach(devid, sc, t, h, &status);

        *(AR5K_STATUS *)s = status;
        if (ah)
                AH_MOD_INC_USE_COUNT(THIS_MODULE);
        return ah;
}

void
ath_hal_detach(struct ath_hal *ah)
{
        (*ah->ah_detach)(ah);
        AH_MOD_DEC_USE_COUNT(THIS_MODULE);
}

EXPORT_SYMBOL(ath_hal_probe);
EXPORT_SYMBOL(_ath_hal_attach);
EXPORT_SYMBOL(ath_hal_detach);
EXPORT_SYMBOL(ath_hal_init_channels);
EXPORT_SYMBOL(ath_hal_getwirelessmodes);
EXPORT_SYMBOL(ath_hal_computetxtime);
EXPORT_SYMBOL(ath_hal_mhz2ieee);
EXPORT_SYMBOL(ath_hal_ieee2mhz);

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
