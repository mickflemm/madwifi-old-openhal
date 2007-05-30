/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_output.c,v 1.18 2005/01/24 20:50:20 sam Exp $");
__KERNEL_RCSID(0, "$NetBSD: ieee80211_output.c,v 1.9 2003/11/02 00:17:27 dyoung Exp $");

/*
 * IEEE 802.11 output handling.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/if_vlan.h>

#include "if_llc.h"
#include "if_ethersubr.h"
#include "if_media.h"

#include <net80211/ieee80211_var.h>

#ifdef IEEE80211_DEBUG
/*
 * Decide if an outbound management frame should be
 * printed when debugging is enabled.  This filters some
 * of the less interesting frames that come frequently
 * (e.g. beacons).
 */
static __inline int
doprint(struct ieee80211com *ic, int subtype)
{
	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		return (ic->ic_opmode == IEEE80211_M_IBSS);
	}
	return 1;
}
#endif

/*
 * Send a management frame to the specified node.  The node pointer
 * must have a reference as the pointer will be passed to the driver
 * and potentially held for a long time.  If the frame is successfully
 * dispatched to the driver, then it is responsible for freeing the
 * reference (and potentially free'ing up any associated storage).
 */
static int
ieee80211_mgmt_output(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct sk_buff *skb, int type)
{
	struct net_device *dev = ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_cb *cb = (struct ieee80211_cb *)skb->cb;

	KASSERT(ni != NULL, ("null node"));

	/*
	 * We want to pass the node down to the
	 * driver's start routine.  If we don't do so then the start
	 * routine must immediately look it up again and that can
	 * cause a lock order reversal if, for example, this frame
	 * is being sent because the station is being timedout and
	 * the frame being sent is a DEAUTH message. We stuff it in 
	 * the cb structure.
	 */
	cb->ni = ni;

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT | type;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	wh->i_dur = 0;
	*(__le16 *)wh->i_seq =
	    htole16(ni->ni_txseqs[0] << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseqs[0]++;
	/*
	 * Hack.  When sending PROBE_REQ frames while scanning we
	 * explicitly force a broadcast rather than (as before) clobber
	 * ni_macaddr and ni_bssid.  This is stopgap, we need a way
	 * to communicate this directly rather than do something
	 * implicit based on surrounding state.
	 */
	if (type == IEEE80211_FC0_SUBTYPE_PROBE_REQ &&
	    (ic->ic_flags & IEEE80211_F_SCAN)) {
		IEEE80211_ADDR_COPY(wh->i_addr1, dev->broadcast);
		IEEE80211_ADDR_COPY(wh->i_addr2, ic->ic_myaddr);
		IEEE80211_ADDR_COPY(wh->i_addr3, dev->broadcast);
	} else {
		IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
		IEEE80211_ADDR_COPY(wh->i_addr2, ic->ic_myaddr);
		IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);
	}

	if ((cb->flags & M_LINK0) != 0 && ni->ni_challenge != NULL) {
		cb->flags &= ~M_LINK0;
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
			"[%s] encrypting frame (%s)\n",
			ether_sprintf(wh->i_addr1), __func__);
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
	}
#ifdef IEEE80211_DEBUG
	/* avoid printing too many frames */
	if ((ieee80211_msg_debug(ic) && doprint(ic, type)) ||
	    ieee80211_msg_dumppkts(ic)) {
		printf("[%s] send %s on channel %u\n",
		    ether_sprintf(wh->i_addr1),
		    ieee80211_mgt_subtype_name[
			(type & IEEE80211_FC0_SUBTYPE_MASK) >>
				IEEE80211_FC0_SUBTYPE_SHIFT],
		    ieee80211_chan2ieee(ic, ni->ni_chan));
	}
#endif
	IEEE80211_NODE_STAT(ni, tx_mgmt);
	IF_ENQUEUE(&ic->ic_mgtq, skb);
	(*dev->hard_start_xmit)(NULL, dev);
	return 0;
}

/*
 * Send a null data frame to the specified node.
 */
int
ieee80211_send_nulldata(struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct net_device *dev = ic->ic_dev;
	struct sk_buff *skb;
	struct ieee80211_frame *wh;
	struct ieee80211_cb *cb;
	u_int8_t *frm;

	skb = ieee80211_getmgtframe(&frm, 0);

	if (skb == NULL) {
		/* XXX debug msg */
		ic->ic_stats.is_tx_nobuf++;
		return ENOMEM;
	}
	cb  = (struct ieee80211_cb *)skb->cb;
	cb->ni = ieee80211_ref_node(ni);

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA |
		IEEE80211_FC0_SUBTYPE_NODATA;
	wh->i_dur = 0;
	*(__le16 *)wh->i_seq =
	    htole16(ni->ni_txseqs[0] << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseqs[0]++;

	/* XXX WDS */
	wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
	IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, ni->ni_bssid);
	IEEE80211_ADDR_COPY(wh->i_addr3, ic->ic_myaddr);
	skb_trim(skb, sizeof(struct ieee80211_frame));

	IEEE80211_NODE_STAT(ni, tx_data);

	IF_ENQUEUE(&ic->ic_mgtq, skb);		/* cheat */
	(*dev->hard_start_xmit)(NULL, dev);

	return 0;
}

/* 
 * Assign priority to a frame based on any vlan tag assigned
 * to the station and/or any Diffserv setting in an IP header.
 * Finally, if an ACM policy is setup (in station mode) it's
 * applied.
 */
int
ieee80211_classify(struct ieee80211com *ic, struct sk_buff *skb, struct ieee80211_node *ni)
{
	int v_wme_ac, d_wme_ac, ac;
	struct ether_header *eh;
	const struct iphdr *ip;

	if ((ni->ni_flags & IEEE80211_NODE_QOS) == 0) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s] no QOS/WME for this node (%s)\n", __func__, ether_sprintf(ni->ni_macaddr));
		ac = WME_AC_BE;
		goto done;
	}

	/* 
	 * If node has a vlan tag then all traffic
	 * to it must have a matching tag.
	 */
	v_wme_ac = 0;
	if (ni->ni_vlan != 0) {
		 if (ic->ic_vlgrp == NULL || !vlan_tx_tag_present(skb)) {
			IEEE80211_NODE_STAT(ni, tx_novlantag);
			return 1;
		}
		if (vlan_tx_tag_get(skb) != ni->ni_vlan) {
			IEEE80211_NODE_STAT(ni, tx_vlanmismatch);
			return 1;
		}
		/* map vlan priority to AC */
		switch (vlan_get_ingress_priority(skb->dev, ni->ni_vlan)) {
		case 1:
		case 2:
			v_wme_ac = WME_AC_BK;
			break;
		case 0:
		case 3:
			v_wme_ac = WME_AC_BE;
			break;
		case 4:
		case 5:
			v_wme_ac = WME_AC_VI;
			break;
		case 6:
		case 7:
			v_wme_ac = WME_AC_VO;
			break;
		}
	}
	eh = (struct ether_header *) skb->data;
	if (eh->ether_type == __constant_htons(ETHERTYPE_IP)) {
		ip = ip_hdr(skb);

		/*
		 * IP frame, map the DSCP field (corrected version, <<2).
		 */
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "ip=%p,ip->version=%d, ip->length=%d, &(ip->saddr)=%p, &(ip->daddr)=%p\n",
			ip, ip->version, ip->ihl, &(ip->saddr), &(ip->daddr));
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] src: 0x%x dst: 0x%x paket tos: 0x%x\n", 
				__func__, ic->ic_dev->name, ip->saddr, ip->daddr, ip->tos);
		switch (ip->tos) {
		case 0x20:
		case 0x40:
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] sorting packet in WME_AC_BK queue (node %s)\n", 
				__func__, ic->ic_dev->name, ether_sprintf(ni->ni_macaddr));
			d_wme_ac = WME_AC_BK;	/* background */
			break;
		case 0x80:
		case 0xa0:
			d_wme_ac = WME_AC_VI;	/* video */
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] sorting packet in WME_AC_VI queue (node %s)\n", 
				__func__, ic->ic_dev->name, ether_sprintf(ni->ni_macaddr));
			break;
		case 0xc0:			/* voice */
		case 0xe0:
		case 0x88:			/* XXX UPSD */
		case 0xb8:
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] sorting packet in WME_AC_VO queue (node %s)\n", 
				__func__, ic->ic_dev->name, ether_sprintf(ni->ni_macaddr));
			d_wme_ac = WME_AC_VO;
			break;
		default:
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] sorting packet in WME_AC_BE queue (node %s)\n", 
				__func__, ic->ic_dev->name, ether_sprintf(ni->ni_macaddr));
			d_wme_ac = WME_AC_BE;
			break;
		}
	} else {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME, "[%s (%s)] no IP packet, sorting packet in WME_AC_BE queue (node %s)\n", 
			__func__, ic->ic_dev->name, ether_sprintf(ni->ni_macaddr));
		d_wme_ac = WME_AC_BE;
	}
	/*
	 * Use highest priority AC.
	 */
	if (v_wme_ac > d_wme_ac)
		ac = v_wme_ac;
	else
		ac = d_wme_ac;

	/*
	 * Apply ACM policy.
	 */
	if (ic->ic_opmode == IEEE80211_M_STA) {
		static const int acmap[4] = {
			WME_AC_BK,	/* WME_AC_BE */
			WME_AC_BK,	/* WME_AC_BK */
			WME_AC_BE,	/* WME_AC_VI */
			WME_AC_VI,	/* WME_AC_VO */
		};
		while (ac != WME_AC_BK &&
		    ic->ic_wme.wme_wmeBssChanParams.cap_wmeParams[ac].wmep_acm)
			ac = acmap[ac];
	}
done:
	M_WME_SETAC(skb, ac);
	return 0;
}
EXPORT_SYMBOL(ieee80211_classify);

/*
 * Insure there is sufficient headroom and tailroom to
 * encapsulate the 802.11 data frame.  If room isn't
 * already there, reallocate so there is enough space.
 * Drivers and cipher modules assume we have done the
 * necessary work and fail rudely if they don't find
 * the space they need.
 */
static struct sk_buff *
ieee80211_skbhdr_adjust(struct ieee80211com *ic, int hdrsize,
	struct ieee80211_key *key, struct sk_buff *skb)
{
	int need_headroom = hdrsize;
	int need_tailroom = 0;

	if (key != NULL) {
		const struct ieee80211_cipher *cip = key->wk_cipher;
		/*
		 * Adjust for crypto needs.  When hardware crypto is
		 * being used we assume the hardware/driver will deal
		 * with any padding (on the fly, without needing to
		 * expand the frame contents).  When software crypto
		 * is used we need to insure room is available at the
		 * front and back and also for any per-MSDU additions.
		 */
		/* XXX belongs in crypto code? */
		need_headroom += cip->ic_header;
		/* XXX pre-calculate per key */
		if (key->wk_flags & IEEE80211_KEY_SWCRYPT)
			need_tailroom += cip->ic_trailer;
		/* XXX frags */
		if (key->wk_flags & IEEE80211_KEY_SWMIC)
			need_tailroom += cip->ic_miclen;
	}
	/*
	 * We know we are called just after stripping an Ethernet
	 * header and before prepending an LLC header.  This means we 
	 * need to assure the LLC header fits in.
	 */
	need_headroom += sizeof(struct llc);
	skb = skb_unshare(skb, GFP_ATOMIC);
	if (skb == NULL) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_OUTPUT,
		    "%s: cannot unshare for encapsulation\n", __func__);
		ic->ic_stats.is_tx_nobuf++;
	} else if (skb_tailroom(skb) < need_tailroom) {
		int headroom = skb_headroom(skb) < need_headroom ?
			need_headroom - skb_headroom(skb) : 0;
		if (pskb_expand_head(skb, headroom,
			need_tailroom - skb_tailroom(skb), GFP_ATOMIC)) {
			dev_kfree_skb(skb);
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_OUTPUT,
			    "%s: cannot expand storage (tail)\n", __func__);
			ic->ic_stats.is_tx_nobuf++;
			skb = NULL;
		}
	} else if (skb_headroom(skb) < need_headroom) {
		struct sk_buff *tmp = skb;
		skb = skb_realloc_headroom(skb, need_headroom);
		dev_kfree_skb(tmp);
		if (skb == NULL) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_OUTPUT,
			    "%s: cannot expand storage (head)\n", __func__);
			ic->ic_stats.is_tx_nobuf++;
		}
	}
	return skb;
}

#define	KEY_UNDEFINED(k)	((k).wk_cipher == &ieee80211_cipher_none)
/*
 * Return the transmit key to use in sending a unicast frame.
 * If a unicast key is set we use that.  When no unicast key is set
 * we fall back to the default transmit key.
 */ 
static inline struct ieee80211_key *
ieee80211_crypto_getucastkey(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (KEY_UNDEFINED(ni->ni_ucastkey)) {
		if (ic->ic_def_txkey == IEEE80211_KEYIX_NONE ||
		    KEY_UNDEFINED(ic->ic_nw_keys[ic->ic_def_txkey]))
			return NULL;
		return &ic->ic_nw_keys[ic->ic_def_txkey];
	} else {
		return &ni->ni_ucastkey;
	}
}

/*
 * Return the transmit key to use in sending a multicast frame.
 * Multicast traffic always uses the group key which is installed as
 * the default tx key.
 */ 
static inline struct ieee80211_key *
ieee80211_crypto_getmcastkey(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (ic->ic_def_txkey == IEEE80211_KEYIX_NONE ||
	    KEY_UNDEFINED(ic->ic_nw_keys[ic->ic_def_txkey]))
		return NULL;
	return &ic->ic_nw_keys[ic->ic_def_txkey];
}

/*
 * Encapsulate an outbound data frame.  The sk_buff chain is updated.
 * If an error is encountered NULL is returned.  The caller is required
 * to provide a node reference and pullup the ethernet header in the
 * first sk_buff.
 */
struct sk_buff *
ieee80211_encap(struct ieee80211com *ic, struct sk_buff *skb,
	struct ieee80211_node *ni)
{
	struct ether_header eh;
	struct ieee80211_frame *wh = NULL;
	struct ieee80211_frame_addr4 *wh4 = NULL;
	struct ieee80211_key *key;
	struct llc *llc;
	int hdrsize, datalen, addqos;
	struct net_device *wdsdev = ni->ni_wdsdev;
	u_int8_t *i_seq;
	u_int8_t *i_fc;
	struct ieee80211_cb *cb = (struct ieee80211_cb *)skb->cb;

	KASSERT(skb->len >= sizeof(eh), ("no ethernet header!"));
	memcpy(&eh, skb->data, sizeof(struct ether_header));
	skb_pull(skb, sizeof(struct ether_header));

	/*
	 * Insure space for additional headers.  First identify
	 * transmit key to use in calculating any buffer adjustments
	 * required.  This is also used below to do privacy
	 * encapsulation work.  Then calculate the 802.11 header
	 * size and any padding required by the driver.
	 *
	 * Note key may be NULL if we fall back to the default
	 * transmit key and that is not set.  In that case the
	 * buffer may not be expanded as needed by the cipher
	 * routines, but they will/should discard it.
	 */
	if (ic->ic_flags & IEEE80211_F_PRIVACY) {
		if (ic->ic_opmode == IEEE80211_M_STA ||
		    !IEEE80211_IS_MULTICAST(eh.ether_dhost)||wdsdev)
			key = ieee80211_crypto_getucastkey(ic, ni);
		else
			key = ieee80211_crypto_getmcastkey(ic, ni);
		if (key == NULL && eh.ether_type != __constant_htons(ETHERTYPE_PAE)) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_CRYPTO,
			    "[%s] no default transmit key (%s) deftxkey %u\n",
			    ether_sprintf(eh.ether_dhost), __func__,
			    ic->ic_def_txkey);
			ic->ic_stats.is_tx_nodefkey++;
		}
	} else
		key = NULL;
	/* XXX 4-address format */
	/*
	 * XXX Some ap's don't handle QoS-encapsulated EAPOL
	 * frames so suppress use.  This may be an issue if other
	 * ap's require all data frames to be QoS-encapsulated
	 * once negotiated in which case we'll need to make this
	 * configurable.
	 */
	addqos = (ni->ni_flags & IEEE80211_NODE_QOS) &&
		 eh.ether_type != __constant_htons(ETHERTYPE_PAE);
	if (addqos) {
		if(wdsdev) {
			hdrsize = sizeof(struct ieee80211_qosframe_addr4);
		} else {
			hdrsize = sizeof(struct ieee80211_qosframe);
		}
	} else {
		if(wdsdev) {
			hdrsize = sizeof(struct ieee80211_frame_addr4);
		} else {
			hdrsize = sizeof(struct ieee80211_frame);
		}
	}
	if (ic->ic_flags & IEEE80211_F_DATAPAD)
		hdrsize = roundup(hdrsize, sizeof(u_int32_t));
	skb = ieee80211_skbhdr_adjust(ic, hdrsize, key, skb);
	if (skb == NULL) {
		/* NB: ieee80211_skbhdr_adjust handles msgs+statistics */
		goto bad;
	}

	llc = (struct llc *) skb_push(skb, sizeof(struct llc));
	llc->llc_dsap = llc->llc_ssap = LLC_SNAP_LSAP;
	llc->llc_control = LLC_UI;
	llc->llc_snap.org_code[0] = 0;
	llc->llc_snap.org_code[1] = 0;
	llc->llc_snap.org_code[2] = 0;
	llc->llc_snap.ether_type = eh.ether_type;
	datalen = skb->len;		/* NB: w/o 802.11 header */

	if(wdsdev) {
		wh4 = (struct ieee80211_frame_addr4*)skb_push(skb, hdrsize);
		wh4->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
		wh4->i_dur = 0;
		i_seq = &wh4->i_seq[0];
		i_fc = &wh4->i_fc[0];
	} else {
		wh = (struct ieee80211_frame *)skb_push(skb, hdrsize);
		wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
		wh->i_dur = 0;
		i_seq = &wh->i_seq[0];
		i_fc = &wh->i_fc[0];
	}

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		wh->i_fc[1] = IEEE80211_FC1_DIR_TODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_bssid);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
		wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		/*
		 * NB: always use the bssid from ic_bss as the
		 *     neighbor's may be stale after an ibss merge
		 */
		IEEE80211_ADDR_COPY(wh->i_addr3, ic->ic_bss->ni_bssid);
		break;
	case IEEE80211_M_HOSTAP:
		/* bssid is our self */
		if(wdsdev) {
			i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
			IEEE80211_ADDR_COPY(wh4->i_addr2, ni->ni_bssid);
			IEEE80211_ADDR_COPY(wh4->i_addr3, eh.ether_dhost);
			IEEE80211_ADDR_COPY(wh4->i_addr4, eh.ether_shost);
			/* addr1 is wds peer */
			IEEE80211_ADDR_COPY(wh4->i_addr1, ni->ni_macaddr);
		} else {
			i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
			IEEE80211_ADDR_COPY(wh->i_addr2, ni->ni_bssid);
			IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
			IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_shost);
		}
		break;
	case IEEE80211_M_MONITOR:
		goto bad;
	}
	if (cb->flags & M_MORE_DATA)
		wh->i_fc[1] |= IEEE80211_FC1_MORE_DATA;
	if (addqos) {
		struct ieee80211_qosframe *qwh;
		struct ieee80211_qosframe_addr4 *qwh4;
		u_int8_t *i_qos;
		int ac, tid;

		if(wdsdev) {
			qwh4 = (struct ieee80211_qosframe_addr4 *) wh4;
			i_qos = &qwh4->i_qos[0];
		} else {
			qwh = (struct ieee80211_qosframe *) wh;
			i_qos = &qwh->i_qos[0];
		}

		ac = M_WME_GETAC(skb);
		/* map from access class/queue to 11e header priorty value */
		tid = WME_AC_TO_TID(ac);
		i_qos[0] = tid & IEEE80211_QOS_TID;
		if (ic->ic_wme.wme_wmeChanParams.cap_wmeParams[ac].wmep_noackPolicy)
			i_qos[0] |= 1 << IEEE80211_QOS_ACKPOLICY_S;
		i_qos[1] = 0;
		i_fc[0] |= IEEE80211_FC0_SUBTYPE_QOS;

		*(__le16 *)i_seq =
		    htole16(ni->ni_txseqs[tid] << IEEE80211_SEQ_SEQ_SHIFT);
		ni->ni_txseqs[tid]++;
	} else {
		*(__le16 *)i_seq =
		    htole16(ni->ni_txseqs[0] << IEEE80211_SEQ_SEQ_SHIFT);
		ni->ni_txseqs[0]++;
	}
	if (key != NULL) {
		/*
		 * IEEE 802.1X: send EAPOL frames always in the clear.
		 * WPA/WPA2: encrypt EAPOL keys when pairwise keys are set.
		 */
		if (eh.ether_type != __constant_htons(ETHERTYPE_PAE) ||
		    ((ic->ic_flags & IEEE80211_F_WPA) &&
		     (ic->ic_opmode == IEEE80211_M_STA ?
		      !KEY_UNDEFINED(*key) : !KEY_UNDEFINED(ni->ni_ucastkey)))) {
			i_fc[1] |= IEEE80211_FC1_WEP;
			/* XXX do fragmentation */
			if (!ieee80211_crypto_enmic(ic, key, skb, 0)) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_OUTPUT,
				    "[%s] enmic failed, discard frame\n",
				    ether_sprintf(eh.ether_dhost));
				ic->ic_stats.is_crypto_enmicfail++;
				goto bad;
			}
		}
	}

	//TODO: hope remove of ni->ni_inact = ic->ic_inact_run; is ok

	IEEE80211_NODE_STAT(ni, tx_data);
	IEEE80211_NODE_STAT_ADD(ni, tx_bytes, datalen);

	return skb;
bad:
	if (skb != NULL)
		dev_kfree_skb(skb);
	return NULL;
}
EXPORT_SYMBOL(ieee80211_encap);

/*
 * Add a supported rates element id to a frame.
 */
static u_int8_t *
ieee80211_add_rates(u_int8_t *frm, const struct ieee80211_rateset *rs)
{
	int nrates;

	*frm++ = IEEE80211_ELEMID_RATES;
	nrates = rs->rs_nrates;
	if (nrates > IEEE80211_RATE_SIZE)
		nrates = IEEE80211_RATE_SIZE;
	*frm++ = nrates;
	memcpy(frm, rs->rs_rates, nrates);
	return frm + nrates;
}

/*
 * Add an extended supported rates element id to a frame.
 */
static u_int8_t *
ieee80211_add_xrates(u_int8_t *frm, const struct ieee80211_rateset *rs)
{
	/*
	 * Add an extended supported rates element if operating in 11g mode.
	 */
	if (rs->rs_nrates > IEEE80211_RATE_SIZE) {
		int nrates = rs->rs_nrates - IEEE80211_RATE_SIZE;
		*frm++ = IEEE80211_ELEMID_XRATES;
		*frm++ = nrates;
		memcpy(frm, rs->rs_rates + IEEE80211_RATE_SIZE, nrates);
		frm += nrates;
	}
	return frm;
}

/* 
 * Add an ssid elemet to a frame.
 */
static u_int8_t *
ieee80211_add_ssid(u_int8_t *frm, const u_int8_t *ssid, u_int len)
{
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = len;
	memcpy(frm, ssid, len);
	return frm + len;
}

/*
 * Add an erp element to a frame.
 */
static u_int8_t *
ieee80211_add_erp(u_int8_t *frm, struct ieee80211com *ic)
{
	u_int8_t erp;

	*frm++ = IEEE80211_ELEMID_ERP;
	*frm++ = 1;
	erp = 0;
	if (ic->ic_nonerpsta != 0)
		erp |= IEEE80211_ERP_NON_ERP_PRESENT;
	if (ic->ic_flags & IEEE80211_F_USEPROT)
		erp |= IEEE80211_ERP_USE_PROTECTION;
	if (ic->ic_flags & IEEE80211_F_USEBARKER)
		erp |= IEEE80211_ERP_LONG_PREAMBLE;
	*frm++ = erp;
	return frm;
}

static u_int8_t *
ieee80211_setup_wpa_ie(struct ieee80211com *ic, u_int8_t *ie)
{
#define	WPA_OUI_BYTES		0x00, 0x50, 0xf2
#define	ADDSHORT(frm, v) do {			\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v) >> 8;			\
	frm += 2;				\
} while (0)
#define	ADDSELECTOR(frm, sel) do {		\
	memcpy(frm, sel, 4);			\
	frm += 4;				\
} while (0)
	static const u_int8_t oui[4] = { WPA_OUI_BYTES, WPA_OUI_TYPE };
	static const u_int8_t cipher_suite[][4] = {
		{ WPA_OUI_BYTES, WPA_CSE_WEP40 },	/* NB: 40-bit */
		{ WPA_OUI_BYTES, WPA_CSE_TKIP },
		{ 0x00, 0x00, 0x00, 0x00 },		/* XXX WRAP */
		{ WPA_OUI_BYTES, WPA_CSE_CCMP },
		{ 0x00, 0x00, 0x00, 0x00 },		/* XXX CKIP */
		{ WPA_OUI_BYTES, WPA_CSE_NULL },
	};
	static const u_int8_t wep104_suite[4] =
		{ WPA_OUI_BYTES, WPA_CSE_WEP104 };
	static const u_int8_t key_mgt_unspec[4] =
		{ WPA_OUI_BYTES, WPA_ASE_8021X_UNSPEC };
	static const u_int8_t key_mgt_psk[4] =
		{ WPA_OUI_BYTES, WPA_ASE_8021X_PSK };
	const struct ieee80211_rsnparms *rsn = &ic->ic_bss->ni_rsn;
	u_int8_t *frm = ie;
	u_int8_t *selcnt;

	*frm++ = IEEE80211_ELEMID_VENDOR;
	*frm++ = 0;				/* length filled in below */
	memcpy(frm, oui, sizeof(oui));		/* WPA OUI */
	frm += sizeof(oui);
	ADDSHORT(frm, WPA_VERSION);

	/* XXX filter out CKIP */

	/* multicast cipher */
	if (rsn->rsn_mcastcipher == IEEE80211_CIPHER_WEP &&
	    rsn->rsn_mcastkeylen >= 13)
		ADDSELECTOR(frm, wep104_suite);
	else
		ADDSELECTOR(frm, cipher_suite[rsn->rsn_mcastcipher]);

	/* unicast cipher list */
	selcnt = frm;
	ADDSHORT(frm, 0);			/* selector count */
	if (rsn->rsn_ucastcipherset & (1<<IEEE80211_CIPHER_AES_CCM)) {
		selcnt[0]++;
		ADDSELECTOR(frm, cipher_suite[IEEE80211_CIPHER_AES_CCM]);
	}
	if (rsn->rsn_ucastcipherset & (1<<IEEE80211_CIPHER_TKIP)) {
		selcnt[0]++;
		ADDSELECTOR(frm, cipher_suite[IEEE80211_CIPHER_TKIP]);
	}

	/* authenticator selector list */
	selcnt = frm;
	ADDSHORT(frm, 0);			/* selector count */
	if (rsn->rsn_keymgmtset & WPA_ASE_8021X_UNSPEC) {
		selcnt[0]++;
		ADDSELECTOR(frm, key_mgt_unspec);
	}
	if (rsn->rsn_keymgmtset & WPA_ASE_8021X_PSK) {
		selcnt[0]++;
		ADDSELECTOR(frm, key_mgt_psk);
	}

	/* optional capabilities */
	if (rsn->rsn_caps != 0 && rsn->rsn_caps != RSN_CAP_PREAUTH)
		ADDSHORT(frm, rsn->rsn_caps);

	/* calculate element length */
	ie[1] = frm - ie - 2;
	KASSERT(ie[1]+2 <= (int)sizeof(struct ieee80211_ie_wpa),
		("WPA IE too big, %u > %u",
		ie[1]+2, (int)sizeof(struct ieee80211_ie_wpa)));
	return frm;
#undef ADDSHORT
#undef ADDSELECTOR
#undef WPA_OUI_BYTES
}

static u_int8_t *
ieee80211_setup_rsn_ie(struct ieee80211com *ic, u_int8_t *ie)
{
#define	RSN_OUI_BYTES		0x00, 0x0f, 0xac
#define	ADDSHORT(frm, v) do {			\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v) >> 8;			\
	frm += 2;				\
} while (0)
#define	ADDSELECTOR(frm, sel) do {		\
	memcpy(frm, sel, 4);			\
	frm += 4;				\
} while (0)
	static const u_int8_t cipher_suite[][4] = {
		{ RSN_OUI_BYTES, RSN_CSE_WEP40 },	/* NB: 40-bit */
		{ RSN_OUI_BYTES, RSN_CSE_TKIP },
		{ RSN_OUI_BYTES, RSN_CSE_WRAP },
		{ RSN_OUI_BYTES, RSN_CSE_CCMP },
		{ 0x00, 0x00, 0x00, 0x00 },		/* XXX CKIP */
		{ RSN_OUI_BYTES, RSN_CSE_NULL },
	};
	static const u_int8_t wep104_suite[4] =
		{ RSN_OUI_BYTES, RSN_CSE_WEP104 };
	static const u_int8_t key_mgt_unspec[4] =
		{ RSN_OUI_BYTES, RSN_ASE_8021X_UNSPEC };
	static const u_int8_t key_mgt_psk[4] =
		{ RSN_OUI_BYTES, RSN_ASE_8021X_PSK };
	const struct ieee80211_rsnparms *rsn = &ic->ic_bss->ni_rsn;
	u_int8_t *frm = ie;
	u_int8_t *selcnt;

	*frm++ = IEEE80211_ELEMID_RSN;
	*frm++ = 0;				/* length filled in below */
	ADDSHORT(frm, RSN_VERSION);

	/* XXX filter out CKIP */

	/* multicast cipher */
	if (rsn->rsn_mcastcipher == IEEE80211_CIPHER_WEP &&
	    rsn->rsn_mcastkeylen >= 13)
		ADDSELECTOR(frm, wep104_suite);
	else
		ADDSELECTOR(frm, cipher_suite[rsn->rsn_mcastcipher]);

	/* unicast cipher list */
	selcnt = frm;
	ADDSHORT(frm, 0);			/* selector count */
	if (rsn->rsn_ucastcipherset & (1<<IEEE80211_CIPHER_AES_CCM)) {
		selcnt[0]++;
		ADDSELECTOR(frm, cipher_suite[IEEE80211_CIPHER_AES_CCM]);
	}
	if (rsn->rsn_ucastcipherset & (1<<IEEE80211_CIPHER_TKIP)) {
		selcnt[0]++;
		ADDSELECTOR(frm, cipher_suite[IEEE80211_CIPHER_TKIP]);
	}

	/* authenticator selector list */
	selcnt = frm;
	ADDSHORT(frm, 0);			/* selector count */
	if (rsn->rsn_keymgmtset & WPA_ASE_8021X_UNSPEC) {
		selcnt[0]++;
		ADDSELECTOR(frm, key_mgt_unspec);
	}
	if (rsn->rsn_keymgmtset & WPA_ASE_8021X_PSK) {
		selcnt[0]++;
		ADDSELECTOR(frm, key_mgt_psk);
	}

	/* optional capabilities */
	ADDSHORT(frm, rsn->rsn_caps);
	
	/* XXX PMKID */

	/* calculate element length */
	ie[1] = frm - ie - 2;
	KASSERT(ie[1]+2 <= (int)sizeof(struct ieee80211_ie_wpa),
		("RSN IE too big, %u > %u",
		ie[1]+2, (int)sizeof(struct ieee80211_ie_wpa)));
	return frm;
#undef ADDSELECTOR
#undef ADDSHORT
#undef RSN_OUI_BYTES
}

/*
 * Add a WPA/RSN element to a frame.
 */
static u_int8_t *
ieee80211_add_wpa(u_int8_t *frm, struct ieee80211com *ic)
{

	KASSERT(ic->ic_flags & IEEE80211_F_WPA, ("no WPA/RSN!"));
	if (ic->ic_flags & IEEE80211_F_WPA2)
		frm = ieee80211_setup_rsn_ie(ic, frm);
	if (ic->ic_flags & IEEE80211_F_WPA1)
		frm = ieee80211_setup_wpa_ie(ic, frm);
	return frm;
}

#define	WME_OUI_BYTES		0x00, 0x50, 0xf2
/*
 * Add a WME information element to a frame.
 */
static u_int8_t *
ieee80211_add_wme_info(u_int8_t *frm, struct ieee80211_wme_state *wme)
{
	static const struct ieee80211_wme_info info = {
		.wme_id		= IEEE80211_ELEMID_VENDOR,
		.wme_len	= sizeof(struct ieee80211_wme_info) - 2,
		.wme_oui	= { WME_OUI_BYTES },
		.wme_type	= WME_OUI_TYPE,
		.wme_subtype	= WME_INFO_OUI_SUBTYPE,
		.wme_version	= WME_VERSION,
		.wme_info	= 0,
	};
	memcpy(frm, &info, sizeof(info));
	return frm + sizeof(info); 
}

/*
 * Add a WME parameters element to a frame.
 */
static u_int8_t *
ieee80211_add_wme_param(u_int8_t *frm, struct ieee80211_wme_state *wme)
{
#define	SM(_v, _f)	(((_v) << _f##_S) & _f)
#define	ADDSHORT(frm, v) do {			\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v) >> 8;			\
	frm += 2;				\
} while (0)
	/* NB: this works 'cuz a param has an info at the front */
	static const struct ieee80211_wme_info param = {
		.wme_id		= IEEE80211_ELEMID_VENDOR,
		.wme_len	= sizeof(struct ieee80211_wme_param) - 2,
		.wme_oui	= { WME_OUI_BYTES },
		.wme_type	= WME_OUI_TYPE,
		.wme_subtype	= WME_PARAM_OUI_SUBTYPE,
		.wme_version	= WME_VERSION,
	};
	int i;

	memcpy(frm, &param, sizeof(param));
	frm += offsetof(struct ieee80211_wme_info, wme_info);
	*frm++ = wme->wme_bssChanParams.cap_info;	/* AC info */
	*frm++ = 0;					/* reserved field */
	for (i = 0; i < WME_NUM_AC; i++) {
		const struct wmeParams *ac =
		       &wme->wme_bssChanParams.cap_wmeParams[i];
		*frm++ = SM(i, WME_PARAM_ACI)
		       | SM(ac->wmep_acm, WME_PARAM_ACM)
		       | SM(ac->wmep_aifsn, WME_PARAM_AIFSN)
		       ;
		*frm++ = SM(ac->wmep_logcwmax, WME_PARAM_LOGCWMAX)
		       | SM(ac->wmep_logcwmin, WME_PARAM_LOGCWMIN)
		       ;
		ADDSHORT(frm, ac->wmep_txopLimit);
	}
	return frm;
#undef SM
#undef ADDSHORT
}
#undef WME_OUI_BYTES

/*
 * Send a management frame.  The node is for the destination (or ic_bss
 * when in station mode).  Nodes other than ic_bss have their reference
 * count bumped to reflect our use for an indeterminant time.
 */
int
ieee80211_send_mgmt(struct ieee80211com *ic, struct ieee80211_node *ni,
	int type, int arg)
{
#define	senderr(_x, _v)	do { ic->ic_stats._v++; ret = _x; goto bad; } while (0)
	struct sk_buff *skb;
	u_int8_t *frm;
	enum ieee80211_phymode mode;
	u_int16_t capinfo;
	int has_challenge, is_shared_key, ret, timer, status;

	KASSERT(ni != NULL, ("null node"));

	/*
	 * Hold a reference on the node so it doesn't go away until after
	 * the xmit is complete all the way in the driver.  On error we
	 * will remove our reference.
	 */
	IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE,
		"ieee80211_ref_node (%s:%u) %p<%s> refcnt %d\n",
		__func__, __LINE__,
		ni, ether_sprintf(ni->ni_macaddr),
		ieee80211_node_refcnt(ni)+1);
	ieee80211_ref_node(ni);

	timer = 0;
	switch (type) {
	case IEEE80211_FC0_SUBTYPE_PROBE_REQ:
		/*
		 * prreq frame format
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 *	[tlv] user-specified ie's
		 */
		skb = ieee80211_getmgtframe(&frm,
			 2 + IEEE80211_NWID_LEN
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       + (ic->ic_opt_ie != NULL ? ic->ic_opt_ie_len : 0)
		);
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		frm = ieee80211_add_ssid(frm, ic->ic_des_essid, ic->ic_des_esslen);
		mode = ieee80211_chan2mode(ic, ni->ni_chan);
		frm = ieee80211_add_rates(frm, &ic->ic_sup_rates[mode]);
		frm = ieee80211_add_xrates(frm, &ic->ic_sup_rates[mode]);
		if (ic->ic_opt_ie != NULL) {
			memcpy(frm, ic->ic_opt_ie, ic->ic_opt_ie_len);
			frm += ic->ic_opt_ie_len;
		}
		skb_trim(skb, frm - skb->data);

		IEEE80211_NODE_STAT(ni, tx_probereq);
		if (ic->ic_opmode == IEEE80211_M_STA)
			timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		/*
		 * probe response frame format
		 *	[8] time stamp
		 *	[2] beacon interval
		 *	[2] cabability information
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] parameter set (FH/DS)
		 *	[tlv] parameter set (IBSS)
		 *	[tlv] extended rate phy (ERP)
		 *	[tlv] extended supported rates
		 *	[tlv] WPA
		 *	[tlv] WME (optional)
		 */
		skb = ieee80211_getmgtframe(&frm,
			 8
		       + sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + 2 + IEEE80211_NWID_LEN
		       + 2 + IEEE80211_RATE_SIZE
		       + 7	/* max(7,3) */
		       + 6
		       + 3
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       /* XXX !WPA1+WPA2 fits w/o a cluster */
		       + (ic->ic_flags & IEEE80211_F_WPA ?
				2*sizeof(struct ieee80211_ie_wpa) : 0)
		       + sizeof(struct ieee80211_wme_param)
		);
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		memset(frm, 0, 8);	/* timestamp should be filled later */
		frm += 8;
		*(__le16 *)frm = htole16(ic->ic_bss->ni_intval);
		frm += 2;
		if (ic->ic_opmode == IEEE80211_M_IBSS)
			capinfo = IEEE80211_CAPINFO_IBSS;
		else
			capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		*(__le16 *)frm = htole16(capinfo);
		frm += 2;

		frm = ieee80211_add_ssid(frm, ic->ic_bss->ni_essid,
				ic->ic_bss->ni_esslen);
		frm = ieee80211_add_rates(frm, &ni->ni_rates);

		if (ic->ic_phytype == IEEE80211_T_FH) {
                        *frm++ = IEEE80211_ELEMID_FHPARMS;
                        *frm++ = 5;
                        *frm++ = ni->ni_fhdwell & 0x00ff;
                        *frm++ = (ni->ni_fhdwell >> 8) & 0x00ff;
                        *frm++ = IEEE80211_FH_CHANSET(
			    ieee80211_chan2ieee(ic, ni->ni_chan));
                        *frm++ = IEEE80211_FH_CHANPAT(
			    ieee80211_chan2ieee(ic, ni->ni_chan));
                        *frm++ = ni->ni_fhindex;
		} else {
			*frm++ = IEEE80211_ELEMID_DSPARMS;
			*frm++ = 1;
			*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
		}

		if (ic->ic_opmode == IEEE80211_M_IBSS) {
			*frm++ = IEEE80211_ELEMID_IBSSPARMS;
			*frm++ = 2;
			*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
		}
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		if (ic->ic_curmode == IEEE80211_MODE_11G ||
		    ic->ic_curmode == IEEE80211_MODE_TURBO_G)
			frm = ieee80211_add_erp(frm, ic);
		if (ic->ic_flags & IEEE80211_F_WPA)
			frm = ieee80211_add_wpa(frm, ic);
		if (ic->ic_flags & IEEE80211_F_WME)
			frm = ieee80211_add_wme_param(frm, &ic->ic_wme);		
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_AUTH:
		status = arg >> 16;
		arg &= 0xffff;
		has_challenge = ((arg == IEEE80211_AUTH_SHARED_CHALLENGE ||
		    arg == IEEE80211_AUTH_SHARED_RESPONSE) &&
		    ni->ni_challenge != NULL);

		/*
		 * Deduce whether we're doing open authentication or
		 * shared key authentication.  We do the latter if
		 * we're in the middle of a shared key authentication
		 * handshake or if we're initiating an authentication
		 * request and configured to use shared key.
		 */
		is_shared_key = has_challenge ||
		     arg >= IEEE80211_AUTH_SHARED_RESPONSE ||
		     (arg == IEEE80211_AUTH_SHARED_REQUEST &&
		      ic->ic_bss->ni_authmode == IEEE80211_AUTH_SHARED);

		skb = ieee80211_getmgtframe(&frm,
			  3 * sizeof(u_int16_t)
			+ (has_challenge && status == IEEE80211_STATUS_SUCCESS ?
				sizeof(u_int16_t)+IEEE80211_CHALLENGE_LEN : 0)
		);
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		((__le16 *)frm)[0] =
		    (is_shared_key) ? htole16(IEEE80211_AUTH_ALG_SHARED)
		                    : htole16(IEEE80211_AUTH_ALG_OPEN);
		((__le16 *)frm)[1] = htole16(arg);	/* sequence number */
		((__le16 *)frm)[2] = htole16(status);/* status */

		if (has_challenge && status == IEEE80211_STATUS_SUCCESS) {
			((__le16 *)frm)[3] =
			    htole16((IEEE80211_CHALLENGE_LEN << 8) |
			    IEEE80211_ELEMID_CHALLENGE);
			memcpy(&((__le16 *)frm)[4], ni->ni_challenge,
			    IEEE80211_CHALLENGE_LEN);
			skb_trim(skb, 4 * sizeof(u_int16_t) + IEEE80211_CHALLENGE_LEN);
			if (arg == IEEE80211_AUTH_SHARED_RESPONSE) {
				struct ieee80211_cb *cb =
					(struct ieee80211_cb *)skb->cb;
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
				    "[%s] request encrypt frame (%s)\n",
				    ether_sprintf(ni->ni_macaddr), __func__);
				cb->flags |= M_LINK0; /* WEP-encrypt, please */
			}
		} else
			skb_trim(skb, 3 * sizeof(u_int16_t));

		/* XXX not right for shared key */
		if (status == IEEE80211_STATUS_SUCCESS)
			IEEE80211_NODE_STAT(ni, tx_auth);
		else
			IEEE80211_NODE_STAT(ni, tx_auth_fail);

		if (ic->ic_opmode == IEEE80211_M_STA)
			timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_DEAUTH:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
			"[%s] send station deauthenticate (reason %d)\n",
			ether_sprintf(ni->ni_macaddr), arg);
		skb = ieee80211_getmgtframe(&frm, sizeof(u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(__le16 *)frm = htole16(arg);	/* reason */
		skb_trim(skb, sizeof(u_int16_t));

		IEEE80211_NODE_STAT(ni, tx_deauth);
		IEEE80211_NODE_STAT_SET(ni, tx_deauth_code, arg);

		ieee80211_node_unauthorize(ni);	/* port closed */
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_REQ:
	case IEEE80211_FC0_SUBTYPE_REASSOC_REQ:
		/*
		 * asreq frame format
		 *	[2] capability information
		 *	[2] listen interval
		 *	[6*] current AP address (reassoc only)
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 *	[tlv] WME
		 *	[tlv] user-specified ie's
		 */
		skb = ieee80211_getmgtframe(&frm,
			 sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + IEEE80211_ADDR_LEN
		       + 2 + IEEE80211_NWID_LEN
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       + sizeof(struct ieee80211_wme_info)
		       + (ic->ic_opt_ie != NULL ? ic->ic_opt_ie_len : 0)
		);
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		capinfo = 0;
		if (ic->ic_opmode == IEEE80211_M_IBSS)
			capinfo |= IEEE80211_CAPINFO_IBSS;
		else		/* IEEE80211_M_STA */
			capinfo |= IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		/*
		 * NB: Some 11a AP's reject the request when
		 *     short premable is set.
		 */
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) &&
		    (ic->ic_caps & IEEE80211_C_SHSLOT))
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		*(__le16 *)frm = htole16(capinfo);
		frm += 2;

		*(__le16 *)frm = htole16(ic->ic_lintval);
		frm += 2;

		if (type == IEEE80211_FC0_SUBTYPE_REASSOC_REQ) {
			IEEE80211_ADDR_COPY(frm, ic->ic_bss->ni_bssid);
			frm += IEEE80211_ADDR_LEN;
		}

		frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		if ((ic->ic_flags & IEEE80211_F_WME) && ni->ni_wme_ie != NULL)
			frm = ieee80211_add_wme_info(frm, &ic->ic_wme);
		if (ic->ic_opt_ie != NULL) {
			memcpy(frm, ic->ic_opt_ie, ic->ic_opt_ie_len);
			frm += ic->ic_opt_ie_len;
		}
		skb_trim(skb, frm - skb->data);

		timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_RESP:
	case IEEE80211_FC0_SUBTYPE_REASSOC_RESP:
		/*
		 * asreq frame format
		 *	[2] capability information
		 *	[2] status
		 *	[2] association ID
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 *	[tlv] WME (if enabled and STA enabled)
		 */
		skb = ieee80211_getmgtframe(&frm,
			 sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       + sizeof(struct ieee80211_wme_param)
		);
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		*(__le16 *)frm = htole16(capinfo);
		frm += 2;

		*(__le16 *)frm = htole16(arg);	/* status */
		frm += 2;

		if (arg == IEEE80211_STATUS_SUCCESS) {
			*(__le16 *)frm = htole16(ni->ni_associd);
			IEEE80211_NODE_STAT(ni, tx_assoc);
		} else
			IEEE80211_NODE_STAT(ni, tx_assoc_fail);
		frm += 2;

		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		if ((ic->ic_flags & IEEE80211_F_WME) && ni->ni_wme_ie != NULL)
			frm = ieee80211_add_wme_param(frm, &ic->ic_wme);
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_DISASSOC:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
			"[%s] send station disassociate (reason %d)\n",
			ether_sprintf(ni->ni_macaddr), arg);
		skb = ieee80211_getmgtframe(&frm, sizeof(u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(__le16 *)frm = htole16(arg);	/* reason */
		skb_trim(skb, sizeof(u_int16_t));

		IEEE80211_NODE_STAT(ni, tx_disassoc);
		IEEE80211_NODE_STAT_SET(ni, tx_disassoc_code, arg);
		break;

	default:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			"[%s] invalid mgmt frame type %u\n",
			ether_sprintf(ni->ni_macaddr), type);
		senderr(EINVAL, is_tx_unknownmgt);
		/* NOTREACHED */
	}

	ret = ieee80211_mgmt_output(ic, ni, skb, type);
	if (ret == 0) {
		if (timer)
			ic->ic_mgt_timer = timer;
	} else {
bad:
		ieee80211_free_node(ni);
	}
	return ret;
#undef senderr
}

/*
 * Allocate a beacon frame and fillin the appropriate bits.
 */
struct sk_buff *
ieee80211_beacon_alloc(struct ieee80211com *ic, struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo)
{
	struct net_device *dev = ic->ic_dev;
	struct ieee80211_frame *wh;
	struct sk_buff *skb;
	struct ieee80211_cb *cb;
	int pktlen;
	u_int8_t *frm, *efrm;
	u_int16_t capinfo;
	struct ieee80211_rateset *rs;

	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[3] parameter set (DS)
	 *	[tlv] parameter set (IBSS/TIM)
	 *	[tlv] extended rate phy (ERP)
	 *	[tlv] extended supported rates
	 *	[tlv] WME parameters
	 *	[tlv] WPA/RSN parameters
	 * XXX Vendor-specific OIDs (e.g. Atheros)
	 * NB: we allocate the max space required for the TIM bitmap.
	 */
	rs = &ni->ni_rates;
	pktlen =   8					/* time stamp */
		 + sizeof(u_int16_t)			/* beacon interval */
		 + sizeof(u_int16_t)			/* capabilities */
		 + 2 + ni->ni_esslen			/* ssid */
	         + 2 + IEEE80211_RATE_SIZE		/* supported rates */
	         + 2 + 1				/* DS parameters */
		 + 2 + 4 + ic->ic_tim_len		/* DTIM/IBSSPARMS */
		 + 2 + 1				/* ERP */
	         + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		 + (ic->ic_caps & IEEE80211_C_WME ?	/* WME */
			sizeof(struct ieee80211_wme_param) : 0)
		 + (ic->ic_caps & IEEE80211_C_WPA ?	/* WPA 1+2 */
			2*sizeof(struct ieee80211_ie_wpa) : 0)
		 ;
	skb = ieee80211_getmgtframe(&frm, pktlen);
	cb = (struct ieee80211_cb *) skb->cb;

	if (skb == NULL) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			"%s: cannot get buf; size %u\n", __func__, pktlen);
		ic->ic_stats.is_tx_nobuf++;
		return NULL;
	}

	memset(frm, 0, 8);	/* XXX timestamp is set by hardware/driver */
	frm += 8;
	*(__le16 *)frm = htole16(ni->ni_intval);
	frm += 2;
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	bo->bo_caps = (__le16 *)frm;
	*(__le16 *)frm = htole16(capinfo);
	frm += 2;
	*frm++ = IEEE80211_ELEMID_SSID;
	if ((ic->ic_flags & IEEE80211_F_HIDESSID) == 0) {
		*frm++ = ni->ni_esslen;
		memcpy(frm, ni->ni_essid, ni->ni_esslen);
		frm += ni->ni_esslen;
	} else
		*frm++ = 0;
	frm = ieee80211_add_rates(frm, rs);
	if (ic->ic_curmode != IEEE80211_MODE_FH) {
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
	}
	bo->bo_tim = frm;
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
		bo->bo_tim_len = 0;
	} else {
		struct ieee80211_tim_ie *tie = (struct ieee80211_tim_ie *) frm;

		tie->tim_ie = IEEE80211_ELEMID_TIM;
		tie->tim_len = 4;	/* length */
		tie->tim_count = 0;	/* DTIM count */ 
		tie->tim_period = ic->ic_dtim_period;	/* DTIM period */
		tie->tim_bitctl = 0;	/* bitmap control */
		tie->tim_bitmap[0] = 0;	/* Partial Virtual Bitmap */
		frm += sizeof(struct ieee80211_tim_ie);
		bo->bo_tim_len = 1;
	}
	bo->bo_trailer = frm;
	if (ic->ic_flags & IEEE80211_F_WME) {
		bo->bo_wme = frm;
		frm = ieee80211_add_wme_param(frm, &ic->ic_wme);
		ic->ic_flags &= ~IEEE80211_F_WMEUPDATE;
	}
	if (ic->ic_flags & IEEE80211_F_WPA)
		frm = ieee80211_add_wpa(frm, ic);
	if (ic->ic_curmode == IEEE80211_MODE_11G ||
	    ic->ic_curmode == IEEE80211_MODE_TURBO_G)
		frm = ieee80211_add_erp(frm, ic);
	efrm = ieee80211_add_xrates(frm, rs);
	cb->ni = ni;
	bo->bo_trailer_len = efrm - bo->bo_trailer;
	skb_trim(skb, efrm - skb->data);

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	wh->i_dur = 0;
	IEEE80211_ADDR_COPY(wh->i_addr1, dev->broadcast);
	IEEE80211_ADDR_COPY(wh->i_addr2, ic->ic_myaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);
	*(__le16 *)wh->i_seq = 0;

	return skb;
}
EXPORT_SYMBOL(ieee80211_beacon_alloc);

/*
 * Update the dynamic parts of a beacon frame based on the current state.
 */
int
ieee80211_beacon_update(struct ieee80211com *ic, struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo, struct sk_buff *skb0, int mcast)
{
	int len_changed = 0;
	u_int16_t capinfo;
	unsigned long flags;

	IEEE80211_BEACON_LOCK(ic, flags);
	/* XXX faster to recalculate entirely or just changes? */
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	*bo->bo_caps = htole16(capinfo);

	if (ic->ic_flags & IEEE80211_F_WME) {
		struct ieee80211_wme_state *wme = &ic->ic_wme;

		/*
		 * Check for agressive mode change.  When there is
		 * significant high priority traffic in the BSS
		 * throttle back BE traffic by using conservative
		 * parameters.  Otherwise BE uses agressive params
		 * to optimize performance of legacy/non-QoS traffic.
		 */
		if (wme->wme_flags & WME_F_AGGRMODE) {
			if (wme->wme_hipri_traffic >
			    wme->wme_hipri_switch_thresh) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME,
				    "%s: traffic %u, disable aggressive mode\n",
				    __func__, wme->wme_hipri_traffic);
				wme->wme_flags &= ~WME_F_AGGRMODE;
				ieee80211_wme_updateparams_locked(ic);
				wme->wme_hipri_traffic =
					wme->wme_hipri_switch_hysteresis;
			} else
				wme->wme_hipri_traffic = 0;
		} else {
			if (wme->wme_hipri_traffic <=
			    wme->wme_hipri_switch_thresh) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_WME,
				    "%s: traffic %u, enable aggressive mode\n",
				    __func__, wme->wme_hipri_traffic);
				wme->wme_flags |= WME_F_AGGRMODE;
				ieee80211_wme_updateparams_locked(ic);
				wme->wme_hipri_traffic = 0;
			} else
				wme->wme_hipri_traffic =
					wme->wme_hipri_switch_hysteresis;
		}
		if (ic->ic_flags & IEEE80211_F_WMEUPDATE) {
			(void) ieee80211_add_wme_param(bo->bo_wme, wme);
			ic->ic_flags &= ~IEEE80211_F_WMEUPDATE;
		}
	}

	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {	/* NB: no IBSS support*/
		struct ieee80211_tim_ie *tie =
			(struct ieee80211_tim_ie *) bo->bo_tim;
		if (ic->ic_flags & IEEE80211_F_TIMUPDATE) {
			u_int timlen, timoff, i;
			/* 
			 * ATIM/DTIM needs updating.  If it fits in the
			 * current space allocated then just copy in the
			 * new bits.  Otherwise we need to move any trailing
			 * data to make room.  Note that we know there is
			 * contiguous space because ieee80211_beacon_allocate
			 * insures there is space in the mbuf to write a
			 * maximal-size virtual bitmap (based on ic_max_aid).
			 */
			/*
			 * Calculate the bitmap size and offset, copy any
			 * trailer out of the way, and then copy in the
			 * new bitmap and update the information element.
			 * Note that the tim bitmap must contain at least
			 * one byte and any offset must be even.
			 */
			if (ic->ic_ps_pending != 0) {
				timoff = 128;		/* impossibly large */
				for (i = 0; i < ic->ic_tim_len; i++)
					if (ic->ic_tim_bitmap[i]) {
						timoff = i &~ 1;
						break;
					}
				KASSERT(timoff != 128, ("tim bitmap empty!"));
				for (i = ic->ic_tim_len-1; i >= timoff; i--)
					if (ic->ic_tim_bitmap[i])
						break;
				timlen = 1 + (i - timoff);
			} else {
				timoff = 0;
				timlen = 1;
			}
			if (timlen != bo->bo_tim_len) {
				/* copy up/down trailer */
				memmove(tie->tim_bitmap+timlen, bo->bo_trailer,
					bo->bo_trailer_len);
				bo->bo_trailer = tie->tim_bitmap+timlen;
				bo->bo_wme = bo->bo_trailer;
				bo->bo_tim_len = timlen;

				/* update information element */
				tie->tim_len = 3 + timlen;
				tie->tim_bitctl = timoff;
				len_changed = 1;
			}
			memcpy(tie->tim_bitmap, ic->ic_tim_bitmap + timoff,
				bo->bo_tim_len);

			ic->ic_flags &= ~IEEE80211_F_TIMUPDATE;

			IEEE80211_DPRINTF(ic, IEEE80211_MSG_POWER,
				"%s: TIM updated, pending %u, off %u, len %u\n",
				__func__, ic->ic_ps_pending, timoff, timlen);
		}
		/* count down DTIM period */
		if (tie->tim_count == 0)
			tie->tim_count = tie->tim_period - 1;
		else
			tie->tim_count--;
		/* update state for buffered multicast frames on DTIM */
		if (mcast && (tie->tim_count == 0 || tie->tim_period == 1))
			tie->tim_bitctl |= 1;
		else
			tie->tim_bitctl &= ~1;
	}
	IEEE80211_BEACON_UNLOCK(ic, flags);

	return len_changed;
}
EXPORT_SYMBOL(ieee80211_beacon_update);

/*
 * Save an outbound packet for a node in power-save sleep state.
 * The new packet is placed on the node's saved queue, and the TIM
 * is changed, if necessary.
 */
void
ieee80211_pwrsave(struct ieee80211com *ic, struct ieee80211_node *ni, 
		  struct sk_buff *skb)
{
	int qlen, age;

	IEEE80211_NODE_SAVEQ_LOCK(ni);
	if (_IF_QFULL(&ni->ni_savedq)) {
		IEEE80211_NODE_SAVEQ_UNLOCK(ni);
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			"[%s] pwr save q overflow (max size %d)\n",
			ether_sprintf(ni->ni_macaddr), IEEE80211_PS_MAX_QUEUE);
#ifdef IEEE80211_DEBUG
		if (ieee80211_msg_dumppkts(ic))
			ieee80211_dump_pkt((caddr_t) skb->data, skb->len, -1, -1);
#endif
		dev_kfree_skb(skb);
		return;
	}
	/*
	 * Tag the frame with it's expiry time and insert
	 * it in the queue.  The aging interval is 4 times
	 * the listen interval specified by the station. 
	 * Frames that sit around too long are reclaimed
	 * using this information.
	 */
	/* XXX handle overflow? */
	age = ((ni->ni_intval * ic->ic_lintval) << 2) / 1024; /* TU -> secs */
	_IEEE80211_NODE_SAVEQ_ENQUEUE(ni, skb, qlen, age);
	IEEE80211_NODE_SAVEQ_UNLOCK(ni);

	IEEE80211_DPRINTF(ic, IEEE80211_MSG_POWER,
		"[%s] save frame, %u now queued\n",
		ether_sprintf(ni->ni_macaddr), qlen);

	if (qlen == 1)
		ic->ic_set_tim(ni, 1);
}
EXPORT_SYMBOL(ieee80211_pwrsave);
