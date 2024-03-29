/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
 * $Id$
 */

/*
 * Atsushi Onoe's rate control algorithm.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>

#include <asm/uaccess.h>

#include <net80211/if_media.h>
#include <net80211/ieee80211_var.h>

#include "if_athvar.h"
#include "ah_desc.h"

#include "onoe.h"

#define	ONOE_DEBUG
#ifdef ONOE_DEBUG
#define	DPRINTF(sc, _fmt, ...) do {					\
	if (sc->sc_debug & 0x10)					\
		printk(_fmt, __VA_ARGS__);				\
} while (0)
#else
#define	DPRINTF(sc, _fmt, ...)
#endif

/*
 * Default parameters for the rate control algorithm.  These are
 * all tunable with sysctls.  The rate controller runs periodically
 * (each ath_rateinterval ms) analyzing transmit statistics for each
 * neighbor/station (when operating in station mode this is only the AP).
 * If transmits look to be working well over a sampling period then
 * it gives a "raise rate credit".  If transmits look to not be working
 * well than it deducts a credit.  If the credits cross a threshold then
 * the transmit rate is raised.  Various error conditions force the
 * the transmit rate to be dropped.
 *
 * The decision to issue/deduct a credit is based on the errors and
 * retries accumulated over the sampling period.  ath_rate_raise defines
 * the percent of retransmits for which a credit is issued/deducted.
 * ath_rate_raise_threshold defines the threshold on credits at which
 * the transmit rate is increased.
 *
 * XXX this algorithm is flawed.
 */
static	int ath_rateinterval = 1000;		/* rate ctl interval (ms)  */
static	int ath_rate_raise = 10;		/* add credit threshold */
static	int ath_rate_raise_threshold = 10;	/* rate ctl raise threshold */

static void	ath_ratectl(unsigned long);
static void	ath_rate_update(struct ath_softc *, struct ieee80211_node *,
			int rate);
static void	ath_rate_ctl_start(struct ath_softc *, struct ieee80211_node *);
static void	ath_rate_ctl(void *, struct ieee80211_node *);

void
ath_rate_node_init(struct ath_softc *sc, struct ath_node *an)
{
	/* NB: assumed to be zero'd by caller */
	ath_rate_update(sc, &an->an_node, 0);
}
EXPORT_SYMBOL(ath_rate_node_init);

void
ath_rate_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
}
EXPORT_SYMBOL(ath_rate_node_cleanup);

void
ath_rate_findrate(struct ath_softc *sc, struct ath_node *an,
	int shortPreamble, size_t frameLen,
	u_int8_t *rix, int *try0, u_int8_t *txrate)
{
	struct onoe_node *on = ATH_NODE_ONOE(an);

	*rix = on->on_tx_rix0;
	*try0 = on->on_tx_try0;
	if (shortPreamble)
		*txrate = on->on_tx_rate0sp;
	else
		*txrate = on->on_tx_rate0;
}
EXPORT_SYMBOL(ath_rate_findrate);

void
ath_rate_setupxtxdesc(struct ath_softc *sc, struct ath_node *an,
	struct ath_desc *ds, int shortPreamble, u_int8_t rix)
{
	struct onoe_node *on = ATH_NODE_ONOE(an);

	u_int8_t rate1, rate2, rate3, try1, try2, try3;
	
	if (shortPreamble) {
		rate1 = on->on_tx_rate1sp;
		rate2 = on->on_tx_rate2sp;
		rate3 = on->on_tx_rate3sp;
	} else {
		rate1 = on->on_tx_rate1;
		rate2 = on->on_tx_rate2;
		rate3 = on->on_tx_rate3;
	}
		
	/* setting try to 0 seems to bypass this rate */
	try1 = (rate1 == 0) ? 0 : 2;
	try2 = (rate2 == 0) ? 0 : 2;
	try3 = (rate3 == 0) ? 0 : 2;
	
	ath5k_hw_setup_xr_tx_desc(sc->sc_ah, ds
		, rate1, try1	/* series 1 */
		, rate2, try2	/* series 2 */
		, rate3, try3	/* series 3 */
	);
}
EXPORT_SYMBOL(ath_rate_setupxtxdesc);

void
ath_rate_tx_complete(struct ath_softc *sc,
	struct ath_node *an, const struct ath_desc *ds)
{
	struct onoe_node *on = ATH_NODE_ONOE(an);

	if (ds->ds_txstat.ts_status == 0)
		on->on_tx_ok++;
	else
		on->on_tx_err++;
	on->on_tx_retr += ds->ds_txstat.ts_shortretry
			+ ds->ds_txstat.ts_longretry;
}
EXPORT_SYMBOL(ath_rate_tx_complete);

void
ath_rate_newassoc(struct ath_softc *sc, struct ath_node *an, int isnew)
{
	if (isnew)
		ath_rate_ctl_start(sc, &an->an_node);
}
EXPORT_SYMBOL(ath_rate_newassoc);

static void
ath_rate_update(struct ath_softc *sc, struct ieee80211_node *ni, int rate)
{
	struct ath_node *an = ATH_NODE(ni);
	struct onoe_node *on = ATH_NODE_ONOE(an);
	const AR5K_RATE_TABLE *rt = sc->sc_currates;
	u_int8_t rix;

	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	DPRINTF(sc, "%s: set xmit rate for %s to %dM\n",
	    __func__, ether_sprintf(ni->ni_macaddr),
	    ni->ni_rates.rs_nrates > 0 ?
		(ni->ni_rates.rs_rates[rate] & IEEE80211_RATE_VAL) / 2 : 0);

	ni->ni_txrate = rate;
	/* XXX management/control frames always go at the lowest speed */
	an->an_tx_mgtrate = rt->rates[0].rate_code;
	an->an_tx_mgtratesp = an->an_tx_mgtrate | SHPREAMBLE_FLAG(0);
	/*
	 * Before associating a node has no rate set setup
	 * so we can't calculate any transmit codes to use.
	 * This is ok since we should never be sending anything
	 * but management frames and those always go at the
	 * lowest hardware rate.
	 */
	if (ni->ni_rates.rs_nrates == 0)
		goto done;
	on->on_tx_rix0 = sc->sc_rixmap[
		ni->ni_rates.rs_rates[rate] & IEEE80211_RATE_VAL];
	on->on_tx_rate0 = rt->rates[on->on_tx_rix0].rate_code;
	
	on->on_tx_rate0sp = on->on_tx_rate0 |
		SHPREAMBLE_FLAG(on->on_tx_rix0);
	if (sc->sc_mrretry) {
		/*
		 * Hardware supports multi-rate retry; setup two
		 * step-down retry rates and make the lowest rate
		 * be the ``last chance''.  We use 4, 2, 2, 2 tries
		 * respectively (4 is set here, the rest are fixed
		 * in the xmit routine).
		 */
		on->on_tx_try0 = 1 + 3;		/* 4 tries at rate 0 */
		if (--rate >= 0) {
			rix = sc->sc_rixmap[
				ni->ni_rates.rs_rates[rate]&IEEE80211_RATE_VAL];
			on->on_tx_rate1 = rt->rates[rix].rate_code;
			on->on_tx_rate1sp = on->on_tx_rate1 |
				SHPREAMBLE_FLAG(rix);
		} else {
			on->on_tx_rate1 = on->on_tx_rate1sp = 0;
		}
		if (--rate >= 0) {
			rix = sc->sc_rixmap[
				ni->ni_rates.rs_rates[rate]&IEEE80211_RATE_VAL];
			on->on_tx_rate2 = rt->rates[rix].rate_code;
			on->on_tx_rate2sp = on->on_tx_rate2 |
				SHPREAMBLE_FLAG(rix);
		} else {
			on->on_tx_rate2 = on->on_tx_rate2sp = 0;
		}
		if (rate > 0) {
			/* NB: only do this if we didn't already do it above */
			on->on_tx_rate3 = rt->rates[0].rate_code;
			on->on_tx_rate3sp =
				an->an_tx_mgtrate | SHPREAMBLE_FLAG(0);
		} else {
			on->on_tx_rate3 = on->on_tx_rate3sp = 0;
		}
	} else {
		on->on_tx_try0 = ATH_TXMAXTRY;	/* max tries at rate 0 */
		on->on_tx_rate1 = on->on_tx_rate1sp = 0;
		on->on_tx_rate2 = on->on_tx_rate2sp = 0;
		on->on_tx_rate3 = on->on_tx_rate3sp = 0;
	}
done:
	on->on_tx_ok = on->on_tx_err = on->on_tx_retr = on->on_tx_upper = 0;
}

/*
 * Set the starting transmit rate for a node.
 */
static void
ath_rate_ctl_start(struct ath_softc *sc, struct ieee80211_node *ni)
{
#define	RATE(_ix)	(ni->ni_rates.rs_rates[(_ix)] & IEEE80211_RATE_VAL)
	struct ieee80211com *ic = &sc->sc_ic;
	int srate;

	KASSERT(ni->ni_rates.rs_nrates > 0, ("no rates"));
	if (ic->ic_fixed_rate == -1) {
		/*
		 * No fixed rate is requested. For 11b start with
		 * the highest negotiated rate; otherwise, for 11g
		 * and 11a, we start "in the middle" at 24Mb or 36Mb.
		 */
		srate = ni->ni_rates.rs_nrates - 1;
		if (sc->sc_curmode != IEEE80211_MODE_11B) {
			/*
			 * Scan the negotiated rate set to find the
			 * closest rate.
			 */
			/* NB: the rate set is assumed sorted */
			for (; srate >= 0 && RATE(srate) > 72; srate--)
				;
			KASSERT(srate >= 0, ("bogus rate set"));
		}
	} else {
		/*
		 * A fixed rate is to be used; ic_fixed_rate is an
		 * index into the supported rate set.  Convert this
		 * to the index into the negotiated rate set for
		 * the node.  We know the rate is there because the
		 * rate set is checked when the station associates.
		 */
		const struct ieee80211_rateset *rs =
			&ic->ic_sup_rates[ic->ic_curmode];
		int r = rs->rs_rates[ic->ic_fixed_rate] & IEEE80211_RATE_VAL;
		/* NB: the rate set is assumed sorted */
		srate = ni->ni_rates.rs_nrates - 1;
		for (; srate >= 0 && RATE(srate) != r; srate--)
			;
		KASSERT(srate >= 0,
			("fixed rate %d not in rate set", ic->ic_fixed_rate));
	}
	ath_rate_update(sc, ni, srate);
#undef RATE
}

static void
ath_rate_cb(void *arg, struct ieee80211_node *ni)
{
	struct ath_softc *sc = arg;
	ath_rate_update(sc, ni, 0);
}

/*
 * Reset the rate control state for each 802.11 state transition.
 */
void
ath_rate_newstate(struct ath_softc *sc, enum ieee80211_state state)
{
	struct onoe_softc *osc = (struct onoe_softc *) sc->sc_rc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;

	if (state == IEEE80211_S_INIT) {
		del_timer(&osc->timer);
		return;
	}
	if (ic->ic_opmode == IEEE80211_M_STA) {
		/*
		 * Reset local xmit state; this is really only
		 * meaningful when operating in station mode.
		 */
		ni = ic->ic_bss;
		if (state == IEEE80211_S_RUN) {
			ath_rate_ctl_start(sc, ni);
		} else {
			ath_rate_update(sc, ni, 0);
		}
	} else {
		/*
		 * When operating as a station the node table holds
		 * the AP's that were discovered during scanning.
		 * For any other operating mode we want to reset the
		 * tx rate state of each node.
		 */
		ieee80211_iterate_nodes(&ic->ic_sta, ath_rate_cb, sc);
		ath_rate_update(sc, ic->ic_bss, 0);
	}
	if (ic->ic_fixed_rate == -1 && state == IEEE80211_S_RUN) {
		int interval;
		/*
		 * Start the background rate control thread if we
		 * are not configured to use a fixed xmit rate.
		 */
		interval = ath_rateinterval;
		if (ic->ic_opmode == IEEE80211_M_STA)
			interval /= 2;
		mod_timer(&osc->timer, jiffies + ((HZ * interval) / 1000));
	}
}
EXPORT_SYMBOL(ath_rate_newstate);

/* 
 * Examine and potentially adjust the transmit rate.
 */
static void
ath_rate_ctl(void *arg, struct ieee80211_node *ni)
{
	struct ath_softc *sc = arg;
	struct onoe_node *on = ATH_NODE_ONOE(ATH_NODE(ni));
	struct ieee80211_rateset *rs = &ni->ni_rates;
	int dir = 0, nrate, enough;

	/*
	 * Rate control
	 * XXX: very primitive version.
	 */
	enough = (on->on_tx_ok + on->on_tx_err >= 10);

	/* no packet reached -> down */
	if (on->on_tx_err > 0 && on->on_tx_ok == 0)
		dir = -1;

	/* all packets needs retry in average -> down */
	if (enough && on->on_tx_ok < on->on_tx_retr)
		dir = -1;

	/* no error and less than rate_raise% of packets need retry -> up */
	if (enough && on->on_tx_err == 0 &&
	    on->on_tx_retr < (on->on_tx_ok * ath_rate_raise) / 100)
		dir = 1;

	DPRINTF(sc, "%s: ok %d err %d retr %d upper %d dir %d\n",
		ether_sprintf(ni->ni_macaddr),
		on->on_tx_ok, on->on_tx_err, on->on_tx_retr,
		on->on_tx_upper, dir);

	nrate = ni->ni_txrate;
	switch (dir) {
	case 0:
		if (enough && on->on_tx_upper > 0)
			on->on_tx_upper--;
		break;
	case -1:
		if (nrate > 0) {
			nrate--;
			sc->sc_stats.ast_rate_drop++;
		}
		on->on_tx_upper = 0;
		break;
	case 1:
		/* raise rate if we hit rate_raise_threshold */
		if (++on->on_tx_upper < ath_rate_raise_threshold)
			break;
		on->on_tx_upper = 0;
		if (nrate + 1 < rs->rs_nrates) {
			nrate++;
			sc->sc_stats.ast_rate_raise++;
		}
		break;
	}

	if (nrate != ni->ni_txrate) {
		DPRINTF(sc, "%s: %dM -> %dM (%d ok, %d err, %d retr)\n",
		    __func__,
		    (rs->rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL) / 2,
		    (rs->rs_rates[nrate] & IEEE80211_RATE_VAL) / 2,
		    on->on_tx_ok, on->on_tx_err, on->on_tx_retr);
		ath_rate_update(sc, ni, nrate);
	} else if (enough)
		on->on_tx_ok = on->on_tx_err = on->on_tx_retr = 0;
}

static void
ath_ratectl(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct onoe_softc *osc = (struct onoe_softc *) sc->sc_rc;
	struct ieee80211com *ic = &sc->sc_ic;
	int interval;

	if (dev->flags & IFF_RUNNING) {
		sc->sc_stats.ast_rate_calls++;

		if (ic->ic_opmode == IEEE80211_M_STA)
			ath_rate_ctl(sc, ic->ic_bss);	/* NB: no reference */
		else
			ieee80211_iterate_nodes(&ic->ic_sta, ath_rate_ctl, sc);
	}
	interval = ath_rateinterval;
	if (ic->ic_opmode == IEEE80211_M_STA)
		interval /= 2;
	osc->timer.expires = jiffies + ((HZ * interval) / 1000);
	add_timer(&osc->timer);
}

struct ath_ratectrl *
ath_rate_attach(struct ath_softc *sc)
{
	struct onoe_softc *osc;

	osc = kmalloc(sizeof(struct onoe_softc), GFP_ATOMIC);
	if (osc == NULL)
		return NULL;
	osc->arc.arc_space = sizeof(struct onoe_node);
	init_timer(&osc->timer);
	osc->timer.data = (unsigned long) &sc->sc_dev;
	osc->timer.function = ath_ratectl;

	return &osc->arc;
}
EXPORT_SYMBOL(ath_rate_attach);

void
ath_rate_detach(struct ath_ratectrl *arc)
{
	struct onoe_softc *osc = (struct onoe_softc *) arc;

	del_timer(&osc->timer);
	kfree(osc);
}
EXPORT_SYMBOL(ath_rate_detach);

static	int minrateinterval = 500;		/* 500ms */
static	int maxpercent = 100;			/* 100% */
static	int minpercent = 0;			/* 0% */
static	int maxint = 0x7fffffff;		/* 32-bit big */

#define	CTL_AUTO	-2	/* cannot be CTL_ANY or CTL_NONE */

void
ath_rate_dynamic_sysctl_register(struct ath_softc *sc)
{
}
EXPORT_SYMBOL(ath_rate_dynamic_sysctl_register);

/*
 * Static (i.e. global) sysctls.
 */
enum {
	DEV_ATH		= 9,			/* XXX known by many */
};

static ctl_table ath_rate_static_sysctls[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "interval",
	  .mode		= 0644,
	  .data		= &ath_rateinterval,
	  .maxlen	= sizeof(ath_rateinterval),
	  .extra1	= &minrateinterval,
	  .extra2	= &maxint,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "raise",
	  .mode		= 0644,
	  .data		= &ath_rate_raise,
	  .maxlen	= sizeof(ath_rate_raise),
	  .extra1	= &minpercent,
	  .extra2	= &maxpercent,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "raise_threshold",
	  .mode		= 0644,
	  .data		= &ath_rate_raise_threshold,
	  .maxlen	= sizeof(ath_rate_raise_threshold),
	  .proc_handler	= proc_dointvec
	},
	{ 0 }
};
static ctl_table ath_rate_table[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "rate",
	  .mode		= 0555,
	  .child	= ath_rate_static_sysctls
	}, { 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_rate_table
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_sysctl_header;

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Atsushi Onoe's rate control algorithm for Atheros devices");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static char *version = "1.0";
static char *dev_info = "ath_rate_onoe";

static int __init
init_ath_rate_onoe(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

	ath_sysctl_header = ATH_REGISTER_SYSCTL_TABLE(ath_root_table);
	return (0);
}
module_init(init_ath_rate_onoe);

static void __exit
exit_ath_rate_onoe(void)
{
	if (ath_sysctl_header != NULL)
		unregister_sysctl_table(ath_sysctl_header);

	printk(KERN_INFO "%s: unloaded\n", dev_info);
}
module_exit(exit_ath_rate_onoe);
