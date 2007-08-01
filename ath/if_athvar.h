/*-
 * Copyright (c) 2002-2007 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 * $FreeBSD: src/sys/dev/ath/if_athvar.h,v 1.20 2005/01/24 20:31:24 sam Exp $
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_ATHVAR_H
#define _DEV_ATH_ATHVAR_H

#include "ah.h"
#include "../net80211/ieee80211_radiotap.h"
#include "if_athioctl.h"
#include "if_athrate.h"

#include <linux/wireless.h>

#ifndef ARPHRD_IEEE80211_RADIOTAP
#define ARPHRD_IEEE80211_RADIOTAP 803 /* IEEE 802.11 + radiotap header */
#endif /* ARPHRD_IEEE80211_RADIOTAP */

/*
 * Deduce if tasklets are available.  If not then
 * fall back to using the immediate work queue.
 */
#include <linux/interrupt.h>
#ifdef DECLARE_TASKLET			/* native tasklets */
#define tq_struct tasklet_struct
#define ATH_INIT_TQUEUE(a,b,c)		tasklet_init((a),(b),(unsigned long)(c))
#define ATH_SCHEDULE_TQUEUE(a,b)	tasklet_schedule((a))
typedef unsigned long TQUEUE_ARG;
#define mark_bh(a)
#else					/* immediate work queue */
#define ATH_INIT_TQUEUE(a,b,c)		INIT_TQUEUE(a,b,c)
#define ATH_SCHEDULE_TQUEUE(a,b) do {		\
	*(b) |= queue_task((a), &tq_immediate);	\
} while(0)
typedef void *TQUEUE_ARG;
#define	tasklet_disable(t)	do { (void) t; local_bh_disable(); } while (0)
#define	tasklet_enable(t)	do { (void) t; local_bh_enable(); } while (0)
#endif /* !DECLARE_TASKLET */

/*
 * Guess how the interrupt handler should work.
 */
#if !defined(IRQ_NONE)
typedef void irqreturn_t;
#define	IRQ_NONE
#define	IRQ_HANDLED
#endif /* !defined(IRQ_NONE) */

#ifndef SET_MODULE_OWNER
#define	SET_MODULE_OWNER(dev) do {		\
	dev->owner = THIS_MODULE;		\
} while (0)
#endif

#ifndef SET_NETDEV_DEV
#define	SET_NETDEV_DEV(ndev, pdev)
#endif


/*
 * Macro to expand scalars to 64-bit objects
 */
#define	ito64(x) (sizeof(x)==8) ? (((unsigned long long int)(x)) & (0xff)) : \
		 (sizeof(x)==16) ? (((unsigned long long int)(x)) & 0xffff) : \
		 ((sizeof(x)==32) ? (((unsigned long long int)(x)) & 0xffffffff): (unsigned long long int)(x))


/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define	ATH_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, struct file *filp, void *buffer, \
		size_t *lenp)
#define	ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, filp, buffer, lenp)
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) */
#define	ATH_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, struct file *filp, void __user *buffer,\
		size_t *lenp, loff_t *ppos)
#define	ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, filp, buffer, lenp, ppos)
#endif

#define	ATH_TIMEOUT		1000

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU     2290
#define ATH_MIN_MTU     32  

#define	ATH_RXBUF	40		/* number of RX buffers */
#define	ATH_TXBUF	200		/* number of TX buffers */
#define	ATH_TXDESC	1		/* number of descriptors per buffer */
#define ATH_BCBUF       1               /* number of beacon buffers */
#define	ATH_TXMAXTRY	11		/* max number of transmit attempts */
#define	ATH_TXINTR_PERIOD 5		/* max number of batched tx descriptors */

#define ATH_BEACON_AIFS_DEFAULT  0      /* default aifs for ap beacon q */
#define ATH_BEACON_CWMIN_DEFAULT 0      /* default cwmin for ap beacon q */
#define ATH_BEACON_CWMAX_DEFAULT 0      /* default cwmax for ap beacon q */

/*
 * The key cache is used for h/w cipher state and also for
 * tracking station state such as the current tx antenna.
 * We also setup a mapping table between key cache slot indices
 * and station state to short-circuit node lookups on rx.
 * Different parts have different size key caches.  We handle
 * up to ATH_KEYMAX entries (could dynamically allocate state).
 */
#define ATH_KEYMAX      128             /* max key cache size we handle */
#define ATH_KEYBYTES    (ATH_KEYMAX/NBBY)       /* storage space in bytes */

/* driver-specific node state */
struct ath_node {
	struct ieee80211_node an_node;	/* base class */
	u_int8_t	an_tx_mgtrate;	/* h/w rate for management/ctl frames */
	u_int8_t	an_tx_mgtratesp;/* short preamble h/w rate for " " */
	u_int32_t	an_avgrssi;	/* average rssi over all rx frames */
	AR5K_NODE_STATS	an_halstats;	/* rssi statistics used by hal */
	/* variable-length rate control state follows */
};
#define	ATH_NODE(ni)	((struct ath_node *)(ni))
#define	ATH_NODE_CONST(ni)	((const struct ath_node *)(ni))

#define ATH_RSSI_LPF_LEN	10
#define ATH_RSSI_DUMMY_MARKER	0x127
#define ATH_EP_MUL(x, mul)	((x) * (mul))
#define ATH_RSSI_IN(x)		(ATH_EP_MUL((x), AR5K_RSSI_EP_MULTIPLIER))
#define ATH_LPF_RSSI(x, y, len) \
    ((x != ATH_RSSI_DUMMY_MARKER) ? (((x) * ((len) - 1) + (y)) / (len)) : (y))
#define ATH_RSSI_LPF(x, y) do {						\
    if ((y) >= -20)							\
    	x = ATH_LPF_RSSI((x), ATH_RSSI_IN((y)), ATH_RSSI_LPF_LEN);	\
} while (0)

struct ath_buf {
	STAILQ_ENTRY(ath_buf)	bf_list;
	//int			bf_nseg;
	int			bf_flags;	/* tx descriptor flags */
	struct ath_desc		*bf_desc;	/* virtual addr of desc */
	dma_addr_t		bf_daddr;	/* physical addr of desc */
	struct sk_buff		*bf_skb;	/* skbuff for buf */
	dma_addr_t		bf_skbaddr;	/* physical addr of skb data */
	struct ieee80211_node	*bf_node;	/* pointer to the node */
};
typedef STAILQ_HEAD(, ath_buf) ath_bufhead;

struct ath_hal;
struct ath_desc;
struct proc_dir_entry;

/*
 * Data transmit queue state.  One of these exists for each
 * hardware transmit queue.  Packets sent to us from above
 * are assigned to queues based on their priority.  Not all
 * devices support a complete set of hardware transmit queues.
 * For those devices the array sc_ac2q will map multiple
 * priorities to fewer hardware queues (typically all to one
 * hardware queue).
 */
struct ath_txq {
	u_int			axq_qnum;	/* hardware q number */
	u_int			axq_depth;	/* queue depth (stat only) */
	u_int			axq_intrcnt;	/* interrupt count */
	u_int32_t		*axq_link;	/* link ptr in last TX desc */
	STAILQ_HEAD(, ath_buf)	axq_q;		/* transmit queue */
	spinlock_t		axq_lock;	/* lock on q and link */
	/*
	 * State for patching up CTS when bursting.
	 */
	struct	ath_buf		*axq_linkbuf;	/* va of last buffer */
	struct	ath_desc	*axq_lastdsWithCTS;
						/* first desc of last descriptor
						 * that contains CTS 
						 */
	struct	ath_desc	*axq_gatingds;	/* final desc of the gating desc
						 * that determines whether
						 * lastdsWithCTS has been DMA'ed
						 * or not
						 */
};

#define	ATH_TXQ_LOCK_INIT(_sc, _tq)	spin_lock_init(&(_tq)->axq_lock)
#define	ATH_TXQ_LOCK_DESTROY(_tq)	
#define	ATH_TXQ_LOCK(_tq)		spin_lock(&(_tq)->axq_lock)
#define	ATH_TXQ_UNLOCK(_tq)		spin_unlock(&(_tq)->axq_lock)
#define	ATH_TXQ_LOCK_BH(_tq)		spin_lock_bh(&(_tq)->axq_lock)
#define	ATH_TXQ_UNLOCK_BH(_tq)		spin_unlock_bh(&(_tq)->axq_lock)
#define	ATH_TXQ_LOCK_ASSERT(_tq) \
	KASSERT(spin_is_locked(&(_tq)->axq_lock), ("txq not locked!"))

#define ATH_TXQ_INSERT_TAIL(_tq, _elm, _field) do { \
	STAILQ_INSERT_TAIL(&(_tq)->axq_q, (_elm), _field); \
	(_tq)->axq_depth++; \
} while (0)
#define ATH_TXQ_REMOVE_HEAD(_tq, _field) do { \
	STAILQ_REMOVE_HEAD(&(_tq)->axq_q, _field); \
	(_tq)->axq_depth--; \
} while (0)

struct ath_softc {
	struct net_device	sc_dev;		/* NB: must be first */
	void __iomem		*sc_iobase;	/* address of the device */
	struct net_device	sc_rawdev;	/* live monitor device */
	struct semaphore	sc_lock;	/* dev-level lock */
	struct net_device_stats	sc_devstats;	/* device statistics */
	struct ath_stats	sc_stats;	/* private statistics */
	struct ieee80211com	sc_ic;		/* IEEE 802.11 common */
	int			sc_regdomain;
	int			sc_countrycode;
	int			sc_debug;
	void			(*sc_recv_mgmt)(struct ieee80211com *,
					struct sk_buff *,
					struct ieee80211_node *,
					int, int, u_int32_t);
	int			(*sc_newstate)(struct ieee80211com *,
					enum ieee80211_state, int);
	void 			(*sc_node_free)(struct ieee80211_node *);
	void			*sc_bdev;	/* associated bus device */
	struct ath_desc		*sc_desc;	/* TX/RX descriptors */
	size_t			sc_desc_len;	/* size of TX/RX descriptors */
	u_int16_t		sc_cachelsz;	/* cache line size */
	dma_addr_t		sc_desc_daddr;	/* DMA (physical) address */
	struct ath_hal		*sc_ah;		/* Atheros HAL */
	struct ath_ratectrl	*sc_rc;		/* tx rate control support */
	void			(*sc_setdefantenna)(struct ath_softc *, u_int);
	unsigned int		sc_invalid : 1,	/* disable hardware accesses */
				sc_mrretry : 1,	/* multi-rate retry support */
				sc_softled : 1,	/* enable LED gpio status */
				sc_splitmic: 1,	/* split TKIP MIC keys */
				sc_needmib : 1,	/* enable MIB stats intr */
				sc_diversity : 1,/* enable rx diversity */
				sc_lockslottime : 1,/* lock slot time value */
				sc_hasveol : 1,	/* tx VEOL support */
				sc_ledstate: 1,	/* LED on/off state */
				sc_blinking: 1,	/* LED blink operation active */
				sc_endblink: 1,	/* finish LED blink operation */
				sc_mcastkey: 1, /* mcast key cache search */
				sc_hasclrkey:1, /* CLR key supported */
				sc_rawdev_enabled : 1;  /* enable sc_rawdev */
						/* rate tables */
	const AR5K_RATE_TABLE	*sc_rates[IEEE80211_MODE_MAX];
	const AR5K_RATE_TABLE	*sc_currates;	/* current rate table */
	enum ieee80211_phymode	sc_curmode;	/* current phy mode */
	u_int16_t		sc_curtxpow;	/* current tx power limit */
	AR5K_CHANNEL		sc_curchan;	/* current h/w channel */
	u_int8_t		sc_rixmap[256];	/* IEEE to h/w rate table ix */
	struct {
		u_int8_t	ieeerate;	/* IEEE rate */
		u_int8_t	rxflags;	/* radiotap rx flags */
		u_int8_t	txflags;	/* radiotap tx flags */
		u_int16_t	ledon;		/* softled on time */
		u_int16_t	ledoff;		/* softled off time */
	} sc_hwmap[32];				/* h/w rate ix mappings */
	u_int8_t		sc_protrix;	/* protection rate index */
	u_int			sc_txantenna;	/* tx antenna (fixed or auto) */
	AR5K_INT			sc_imask;	/* interrupt mask copy */
	u_int			sc_keymax;	/* size of key cache */
	u_int8_t                sc_keymap[ATH_KEYBYTES];/* key use bit map */
	struct ieee80211_node   *sc_keyixmap[ATH_KEYMAX];/* key ix->node map */

	u_int			sc_ledpin;	/* GPIO pin for driving LED */
	u_int			sc_ledon;	/* pin setting for LED on */
	u_int			sc_ledidle;	/* idle polling interval */
	int			sc_ledevent;	/* time of last LED event */
	u_int8_t		sc_rxrate;	/* current rx rate for LED */
	u_int8_t		sc_txrate;	/* current tx rate for LED */
	u_int16_t		sc_ledoff;	/* off time for current blink */
	struct timer_list	sc_ledtimer;	/* led off timer */
	u_int32_t               sc_rxfilter;

	union {
		struct ath_tx_radiotap_header th;
		u_int8_t	pad[64];
	} u_tx_rt;
	int			sc_tx_th_len;
	union {
		struct ath_rx_radiotap_header th;
		u_int8_t	pad[64];
	} u_rx_rt;
	int			sc_rx_th_len;

	struct tq_struct	sc_fataltq;	/* fatal int tasklet */
	struct tq_struct	sc_radartq;	/* Radar detection */

	int			sc_rxbufsize;	/* rx size based on mtu */
	ath_bufhead		sc_rxbuf;	/* receive buffer */
	u_int32_t		*sc_rxlink;	/* link ptr in last RX desc */
	struct tq_struct	sc_rxtq;	/* rx intr tasklet */
	struct tq_struct	sc_rxorntq;	/* rxorn intr tasklet */
	u_int8_t		sc_defant;	/* current default antenna */
	u_int8_t		sc_rxotherant;	/* rx's on non-default antenna*/

	ath_bufhead		sc_txbuf;	/* transmit buffer */
	spinlock_t		sc_txbuflock;	/* txbuf lock */
	int			sc_tx_timer;	/* transmit timeout */
	u_int			sc_txqsetup;	/* h/w queues setup */
	u_int			sc_txintrperiod;/* tx interrupt batching */
	struct ath_txq		sc_txq[AR5K_NUM_TX_QUEUES];
	struct ath_txq		*sc_ac2q[5];	/* WME AC -> h/w q map */ 
	struct tq_struct	sc_txtq;	/* tx intr tasklet */

	ath_bufhead		sc_bbuf;	/* beacon buffers */
	u_int			sc_bhalq;	/* HAL q for outgoing beacons */
	u_int			sc_bmisscount;	/* missed beacon transmits */
	u_int32_t		sc_ant_tx[8];	/* recent tx frames/antenna */
	struct ath_txq		*sc_cabq;	/* tx q for cab frames */
	struct ath_buf		*sc_bufptr;	/* allocated buffer ptr */
	struct ieee80211_beacon_offsets sc_boff;/* dynamic update state */
	struct tq_struct	sc_bmisstq;	/* bmiss intr tasklet */
	struct tq_struct	sc_bstuckq;	/* stuck beacon processing */
	enum {
		OK,				/* no change needed */
		UPDATE,				/* update pending */
		COMMIT				/* beacon sent, commit change */
	} sc_updateslot;			/* slot time update fsm */

	struct timer_list	sc_cal_ch;	/* calibration timer */
	struct timer_list	sc_scan_ch;	/* AP scan timer */
	struct iw_statistics	sc_iwstats;	/* wireless statistics block */
	struct ctl_table_header	*sc_sysctl_header;
	struct ctl_table	*sc_sysctls;
};

#define	ATH_LOCK_INIT(_sc) \
	init_MUTEX(&(_sc)->sc_lock)
#define	ATH_LOCK_DESTROY(_sc)
#define	ATH_LOCK(_sc)			down(&(_sc)->sc_lock)
#define	ATH_UNLOCK(_sc)			up(&(_sc)->sc_lock)
#define	ATH_LOCK_ASSERT(_sc)
//TODO:	KASSERT(spin_is_locked(&(_sc)->sc_lock), ("buf not locked!"))

#define	ATH_TXQ_SETUP(sc, i)	((sc)->sc_txqsetup & (1<<i))

#define	ATH_TXBUF_LOCK_INIT(_sc)	spin_lock_init(&(_sc)->sc_txbuflock)
#define	ATH_TXBUF_LOCK_DESTROY(_sc)
#define	ATH_TXBUF_LOCK(_sc)		spin_lock(&(_sc)->sc_txbuflock)
#define	ATH_TXBUF_UNLOCK(_sc)		spin_unlock(&(_sc)->sc_txbuflock)
#define	ATH_TXBUF_LOCK_BH(_sc)		spin_lock_bh(&(_sc)->sc_txbuflock)
#define	ATH_TXBUF_UNLOCK_BH(_sc)	spin_unlock_bh(&(_sc)->sc_txbuflock)
#define	ATH_TXBUF_LOCK_ASSERT(_sc) \
	KASSERT(spin_is_locked(&(_sc)->sc_txbuflock), ("txbuf not locked!"))

int	ath_attach(u_int16_t, struct net_device *);
int	ath_detach(struct net_device *);
void	ath_resume(struct net_device *);
void	ath_suspend(struct net_device *);
/*
 *Port r1752 - Starting linux kernel v2.6.19 and later 
 *interrupt handlers are not passed.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19) 
irqreturn_t ath_intr(int, void *); 
#else 
irqreturn_t ath_intr(int, void *, struct pt_regs *regs); 
#endif 
void bus_read_cachesize(struct ath_softc *, u_int8_t *);
int ath_ioctl_ethtool(struct ath_softc *, int, void __user *);
void	ath_sysctl_register(void);
void	ath_sysctl_unregister(void);

/*
 * HAL definitions to comply with local coding convention.
 */
#define	ath_hal_getcapability(_ah, _cap, _param, _result) \
	ath5k_hw_get_capability((_ah), (_cap), (_param), (_result))
#define	ath_hal_setcapability(_ah, _cap, _param, _v, _status) \
	ath5k_hw_set_capability((_ah), (_cap), (_param), (_v), (_status))
#define	ath_hal_ciphersupported(_ah, _cipher) \
	(ath_hal_getcapability(_ah, AR5K_CAP_CIPHER, _cipher, NULL) == AR5K_OK)
#define	ath_hal_getregdomain(_ah, _prd) \
	ath_hal_getcapability(_ah, AR5K_CAP_REG_DMN, 0, (_prd))
#define	ath_hal_getcountrycode(_ah, _pcc) \
	(*(_pcc) = (_ah)->ah_country_code)
#define	ath_hal_tkipsplit(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TKIP_SPLIT, 0, NULL) == AR5K_OK)
#define	ath_hal_hwphycounters(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_PHYCOUNTERS, 0, NULL) == AR5K_OK)
#define	ath_hal_hasdiversity(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_DIVERSITY, 0, NULL) == AR5K_OK)
#define	ath_hal_getdiversity(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_DIVERSITY, 1, NULL) == AR5K_OK)
#define	ath_hal_setdiversity(_ah, _v) \
	ath_hal_setcapability(_ah, AR5K_CAP_DIVERSITY, 1, _v, NULL)
#define	ath_hal_getdiag(_ah, _pv) \
	(ath_hal_getcapability(_ah, AR5K_CAP_DIAG, 0, _pv) == AR5K_OK)
#define	ath_hal_setdiag(_ah, _v) \
	ath_hal_setcapability(_ah, AR5K_CAP_DIAG, 0, _v, NULL)
#define	ath_hal_getnumtxqueues(_ah, _pv) \
	(ath_hal_getcapability(_ah, AR5K_CAP_NUM_TXQUEUES, 0, _pv) == AR5K_OK)
#define	ath_hal_hasveol(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_VEOL, 0, NULL) == AR5K_OK)
#define	ath_hal_hastxpowlimit(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TXPOW, 0, NULL) == AR5K_OK)
/*
#define	ath_hal_settxpowlimit(_ah, _pow) \
	((*(_ah)->ah_setTxPowerLimit)((_ah), (_pow)))
*/
#define	ath_hal_gettxpowlimit(_ah, _ppow) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TXPOW, 1, _ppow) == AR5K_OK)
#define	ath_hal_getmaxtxpow(_ah, _ppow) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TXPOW, 2, _ppow) == AR5K_OK)
#define	ath_hal_gettpscale(_ah, _scale) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TXPOW, 3, _scale) == AR5K_OK)
#define	ath_hal_settpscale(_ah, _v) \
	ath_hal_setcapability(_ah, AR5K_CAP_TXPOW, 3, _v, NULL)
#define	ath_hal_hastpc(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TPC, 0, NULL) == AR5K_OK)
#define	ath_hal_gettpc(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_TPC, 1, NULL) == AR5K_OK)
#define	ath_hal_settpc(_ah, _v) \
	ath_hal_setcapability(_ah, AR5K_CAP_TPC, 1, _v, NULL)
#define	ath_hal_hasbursting(_ah) \
	(ath_hal_getcapability(_ah, AR5K_CAP_BURST, 0, NULL) == AR5K_OK)
#ifdef notyet
#define ath_hal_hasmcastkeysearch(_ah) \
        (ath_hal_getcapability(_ah, AR5K_CAP_MCAST_KEYSRCH, 0, NULL) == AR5K_OK)
#define ath_hal_getmcastkeysearch(_ah) \
        (ath_hal_getcapability(_ah, AR5K_CAP_MCAST_KEYSRCH, 1, NULL) == AR5K_OK)
#else
#define ath_hal_getmcastkeysearch(_ah)  0
#endif

/*
#define	ath_hal_updateCTSForBursting(_ah, _ds, _prevds, _prevdsWithCTS, \
		_gatingds,  _txOpLimit, _ctsDuration) \
	((*(_ah)->ah_updateCTSForBursting)((_ah), (_ds), (_prevds), \
		(_prevdsWithCTS), (_gatingds), (_txOpLimit), (_ctsDuration)))
*/

#endif /* _DEV_ATH_ATHVAR_H */
