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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/cache.h>
#include <linux/if_arp.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
/*For radar functions*/
//#include <linux/cpufreq.h>

#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <asm/io.h>


typedef void* AR5K_SOFTC;
typedef int AR5K_BUS_TAG;
typedef __iomem void* AR5K_BUS_HANDLE;
typedef u_int32_t AR5K_BUS_ADDR;
#define bus_space_tag_t AR5K_BUS_TAG
#define bus_space_handle_t AR5K_BUS_HANDLE

 /*
 * Linux uses __BIG_ENDIAN and __LITTLE_ENDIAN while BSD uses _foo
 * and an explicit _BYTE_ORDER.  Sorry, BSD got there first--define
 * things in the BSD way...
 */
#define LITTLE_ENDIAN  1234    /* LSB first: i386, vax */
#define BIG_ENDIAN     4321    /* MSB first: 68000, ibm, net */

#if defined(__LITTLE_ENDIAN)
#define BYTE_ORDER     LITTLE_ENDIAN
#elif defined(__BIG_ENDIAN)
#define BYTE_ORDER     BIG_ENDIAN
#else
#error "Please fix asm/byteorder.h"
#endif

#define AR5K_PRINTF(fmt, ...)   printk("%s: " fmt, __func__, ##__VA_ARGS__)
#define AR5K_PRINT(fmt)         printk("%s: " fmt, __func__)
#ifdef AR5K_DEBUG
#define AR5K_TRACE              printk("%s:%d\n", __func__, __LINE__)
#else
#define AR5K_TRACE
#endif
#define AR5K_DELAY(_n)          udelay(_n)
#define malloc(_a, _b, _c) kmalloc(_a, GFP_KERNEL)
#define free(_a, _b) kfree(_a)
#define bcopy(_a, _b, _c)       memcpy(_b, _a, _c)
#define bzero(_a, _b)           memset(_a, 0, _b)

//#define AR5K_REG_WRITE(_reg, _val)      (writel(_val, hal->ah_sh + (_reg)))
//      bus_space_write_4(hal->ah_st, hal->ah_sh, (_reg), (_val))

//#define AR5K_REG_READ(_reg)             (readl(hal->ah_sh + (_reg)))
//      bus_space_read_4(hal->ah_st, hal->ah_sh, (_reg))
