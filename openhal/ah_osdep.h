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
#include <asm/uaccess.h>
#include <asm/io.h>


typedef void* HAL_SOFTC;
typedef int HAL_BUS_TAG;
typedef void* HAL_BUS_HANDLE;
typedef u_int32_t HAL_BUS_ADDR;
#define bus_space_tag_t HAL_BUS_TAG
#define bus_space_handle_t HAL_BUS_HANDLE
//#define hz cpufreq_get(0) / 1000
//#define tick 1000000 / hz

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
#ifndef printf
#define printf                  AR5K_PRINT
#endif

#define AR5K_REG_WRITE(_reg, _val)      (writel(cpu_to_le32(_val), hal->ah_sh + (_reg)))
//      bus_space_write_4(hal->ah_st, hal->ah_sh, (_reg), (_val))

#define AR5K_REG_READ(_reg)             (le32_to_cpu(readl(hal->ah_sh + (_reg))))
//      bus_space_read_4(hal->ah_st, hal->ah_sh, (_reg))
