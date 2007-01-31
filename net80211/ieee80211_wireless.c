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
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * Wireless extensions support for 802.11 common code.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/utsname.h>
#include <linux/if_arp.h>		/* XXX for ARPHRD_ETHER */
#include <net/iw_handler.h>

#include <asm/uaccess.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

#define	IS_UP(_dev) \
	(((_dev)->flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))
#define	IS_UP_AUTO(_ic) \
	(IS_UP((_ic)->ic_dev) && (_ic)->ic_roaming == IEEE80211_ROAMING_AUTO)

static u_int encode_ie(void *, size_t, const u_int8_t *, size_t, 
		       const char *, size_t);
static void set_quality(struct iw_quality *, u_int);

struct read_ap_args {
	int i;
	int mode;
	struct iw_event *iwe;
	char *start;
	char *current_ev;
};

static void
read_ap_result(void *arg, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct read_ap_args *sa = arg;
	struct iw_event *iwe = sa->iwe;
	char *end_buf = sa->start + IW_SCAN_MAX_DATA;
	char *current_val;
	int j;
#if WIRELESS_EXT > 14
	char buf[64*2 + 30];
#endif
	/*
	 * Translate data to WE format.
	 */
	if (sa->current_ev >= end_buf) {
		return;
	}

	if ((sa->mode != 0) ^ (ni->ni_wpa_ie != NULL))
		return;

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = SIOCGIWAP;
	iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		IEEE80211_ADDR_COPY(iwe->u.ap_addr.sa_data, ni->ni_macaddr);
	else
		IEEE80211_ADDR_COPY(iwe->u.ap_addr.sa_data, ni->ni_bssid);
	sa->current_ev = iwe_stream_add_event(sa->current_ev,
		end_buf, iwe, IW_EV_ADDR_LEN);

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = SIOCGIWESSID;
	iwe->u.data.flags = 1;
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		iwe->u.data.length = ic->ic_des_esslen;
		sa->current_ev = iwe_stream_add_point(sa->current_ev,
				end_buf, iwe, ic->ic_des_essid);
	} else {
		iwe->u.data.length = ni->ni_esslen;
		sa->current_ev = iwe_stream_add_point(sa->current_ev,
				end_buf, iwe, ni->ni_essid);
	}

	if (ni->ni_capinfo & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) {
		memset(iwe, 0, sizeof(iwe));
		iwe->cmd = SIOCGIWMODE;
		iwe->u.mode = ni->ni_capinfo & IEEE80211_CAPINFO_ESS ?
			IW_MODE_MASTER : IW_MODE_ADHOC;
		sa->current_ev = iwe_stream_add_event(sa->current_ev,
				end_buf, iwe, IW_EV_UINT_LEN);
	}

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = SIOCGIWFREQ;
	iwe->u.freq.m = ni->ni_chan->ic_freq * 100000;
	iwe->u.freq.e = 1;
	sa->current_ev = iwe_stream_add_event(sa->current_ev,
			end_buf, iwe, IW_EV_FREQ_LEN);

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = IWEVQUAL;
	set_quality(&iwe->u.qual, (*ic->ic_node_getrssi)(ni));
	sa->current_ev = iwe_stream_add_event(sa->current_ev,
		end_buf, iwe, IW_EV_QUAL_LEN);

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = SIOCGIWENCODE;
	if (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY)
		iwe->u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
	else
		iwe->u.data.flags = IW_ENCODE_DISABLED;
	iwe->u.data.length = 0;
	sa->current_ev = iwe_stream_add_point(sa->current_ev, end_buf, iwe, "");

	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = SIOCGIWRATE;
	current_val = sa->current_ev + IW_EV_LCP_LEN;
	for (j = 0; j < ni->ni_rates.rs_nrates; j++) {
		if (ni->ni_rates.rs_rates[j]) {
			iwe->u.bitrate.value = ((ni->ni_rates.rs_rates[j] &
			    IEEE80211_RATE_VAL) / 2) * 1000000;
			current_val = iwe_stream_add_value(sa->current_ev,
				current_val, end_buf, iwe,
				IW_EV_PARAM_LEN);
		}
	}
	/* remove fixed header if no rates were added */
	if ((u_int)(current_val - sa->current_ev) > IW_EV_LCP_LEN)
		sa->current_ev = current_val;

#if WIRELESS_EXT > 14
	memset(iwe, 0, sizeof(iwe));
	iwe->cmd = IWEVCUSTOM;
	snprintf(buf, sizeof(buf), "bcn_int=%d", ni->ni_intval);
	iwe->u.data.length = strlen(buf);
	sa->current_ev = iwe_stream_add_point(sa->current_ev, end_buf, iwe, buf);

	if (ni->ni_wpa_ie != NULL) {
		static const char rsn_leader[] = "rsn_ie=";
		static const char wpa_leader[] = "wpa_ie=";
			memset(iwe, 0, sizeof(iwe));
		iwe->cmd = IWEVCUSTOM;
		if (ni->ni_wpa_ie[0] == IEEE80211_ELEMID_RSN)
			iwe->u.data.length = encode_ie(buf, sizeof(buf),
				ni->ni_wpa_ie, ni->ni_wpa_ie[1]+2,
				rsn_leader, sizeof(rsn_leader)-1);
		else
			iwe->u.data.length = encode_ie(buf, sizeof(buf),
				ni->ni_wpa_ie, ni->ni_wpa_ie[1]+2,
				wpa_leader, sizeof(wpa_leader)-1);
		if (iwe->u.data.length != 0)
			sa->current_ev = iwe_stream_add_point(sa->current_ev, end_buf,
				iwe, buf);
	}
#endif /* WIRELESS_EXT > 14 */
	return;
}

/*
 * Units are in db above the noise floor. That means the
 * rssi values reported in the tx/rx descriptors in the
 * driver are the SNR expressed in db.
 *
 * If you assume that the noise floor is -95, which is an
 * excellent assumption 99.5 % of the time, then you can
 * derive the absolute signal level (i.e. -95 + rssi). 
 * There are some other slight factors to take into account
 * depending on whether the rssi measurement is from 11b,
 * 11g, or 11a.   These differences are at most 2db and
 * can be documented.
 *
 * NB: various calculations are based on the orinoco/wavelan
 *     drivers for compatibility
 */
static void
set_quality(struct iw_quality *iq, u_int rssi)
{
	iq->qual = rssi;
	/* NB: max is 94 because noise is hardcoded to 161 */
	if (iq->qual > 94)
		iq->qual = 94;

	iq->noise = 161;		/* -95dBm */
	iq->level = iq->noise + iq->qual;
	iq->updated = 7;
}

void
ieee80211_iw_getstats(struct ieee80211com *ic, struct iw_statistics *is)
{
#define	NZ(x)	((x) ? (x) : 1)

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		/* use stats from associated ap */
		if (ic->ic_bss && ic->ic_state == IEEE80211_S_RUN)
			set_quality(&is->qual,
				(*ic->ic_node_getrssi)(ic->ic_bss));
		else
			set_quality(&is->qual, 0);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
	case IEEE80211_M_HOSTAP: {
		struct ieee80211_node_table *nt = &ic->ic_sta;
		struct ieee80211_node* ni;
		u_int32_t rssi_samples = 0;
		u_int32_t rssi_total = 0;

		/* average stats from all nodes */
		TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
			rssi_samples++;
			rssi_total += (*ic->ic_node_getrssi)(ni);
		}
		set_quality(&is->qual, rssi_total / NZ(rssi_samples));
		break;
	}
	case IEEE80211_M_MONITOR:
	default:
		/* no stats */
		set_quality(&is->qual, 0);
		break;
	}
	is->status = ic->ic_state;
	is->discard.nwid = ic->ic_stats.is_rx_wrongbss
			 + ic->ic_stats.is_rx_ssidmismatch;
	is->discard.code = ic->ic_stats.is_rx_wepfail
			 + ic->ic_stats.is_rx_decryptcrc;
	is->discard.fragment = 0;
	is->discard.retries = 0;
	is->discard.misc = 0;

	is->miss.beacon = 0;
#undef NZ
}
EXPORT_SYMBOL(ieee80211_iw_getstats);

int
ieee80211_ioctl_giwname(struct ieee80211com *ic,
		   struct iw_request_info *info,
		   char *name, char *extra)
{

	/* XXX should use media status but IFM_AUTO case gets tricky */
	switch (ic->ic_curmode) {
	case IEEE80211_MODE_11A:
		strncpy(name, "IEEE 802.11a", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11B:
		strncpy(name, "IEEE 802.11b", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11G:
		strncpy(name, "IEEE 802.11g", IFNAMSIZ);
		break;
	case IEEE80211_MODE_TURBO_A:
		strncpy(name, "Turbo-A", IFNAMSIZ);
		break;
	case IEEE80211_MODE_TURBO_G:
		strncpy(name, "Turbo-G", IFNAMSIZ);
		break;
	default:
		strncpy(name, "IEEE 802.11", IFNAMSIZ);
		break;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwname);

/*
 * Get a key index from a request.  If nothing is
 * specified in the request we use the current xmit
 * key index.  Otherwise we just convert the index
 * to be base zero.
 */
static int
getiwkeyix(struct ieee80211com *ic, const struct iw_point* erq, int *kix)
{
	int kid;

	kid = erq->flags & IW_ENCODE_INDEX;
	if (kid < 1 || kid > IEEE80211_WEP_NKID) {
		kid = ic->ic_def_txkey;
		if (kid == IEEE80211_KEYIX_NONE)
			kid = 0;
	} else
		--kid;
	if (0 <= kid && kid < IEEE80211_WEP_NKID) {
		*kix = kid;
		return 0;
	} else
		return EINVAL;
}

int
ieee80211_ioctl_siwencode(struct ieee80211com *ic,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *keybuf)
{
	int kid, error = 0;
	int wepchange = 0;
	struct ieee80211_key *k;
	/* 
	 * set key
	 *
	 * New version of iwconfig set the IW_ENCODE_NOKEY flag
	 * when no key is given, but older versions don't,
	 * so we have to check the length too.
	 */
	if (erq->length > 0 && !(erq->flags & IW_ENCODE_NOKEY)) {
		/*
		 * set key contents, set the default transmit key
		 * and enable crypto.
	 	 */
		error = getiwkeyix(ic, erq, &kid);
		if (error)
			return -error;
		if (erq->length > IEEE80211_KEYBUF_SIZE)
			return -EINVAL;
		/* set key contents */
		k = &ic->ic_nw_keys[kid];
		ieee80211_key_update_begin(ic);
		k->wk_keyix = kid;      /* NB: force fixed key id */
		if (ieee80211_crypto_newkey(ic, IEEE80211_CIPHER_WEP,
		    IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV, k)) {
			k->wk_keylen = erq->length;
			/* NB: preserve flags set by newkey */
			k->wk_flags |=
				IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV;
			memcpy(k->wk_key, keybuf, erq->length);
			memset(k->wk_key + erq->length, 0,
				IEEE80211_KEYBUF_SIZE - erq->length);
			if (!ieee80211_crypto_setkey(ic, k, ic->ic_myaddr))
				error = -EINVAL;
		} else {
			error = -EINVAL;
		}
		ieee80211_key_update_end(ic);
		
		if (error == 0) {
			/* set default key & enable privacy */
			ic->ic_def_txkey = kid;
			wepchange = (ic->ic_flags & IEEE80211_F_PRIVACY) == 0;
			ic->ic_flags |= IEEE80211_F_PRIVACY;
			ic->ic_flags |= IEEE80211_F_DROPUNENC;
		}
	}
	/* 
	 * set key index only
	 */
	else if ( (erq->flags & IW_ENCODE_INDEX) > 0) {
		/* 
		 * verify the new key has a non-zero length
		 * and change the default transmit key.
		 */
		error = getiwkeyix(ic, erq, &kid);
		if (error)
			return -error;
		
		if (ic->ic_nw_keys[kid].wk_keylen == 0)
			return -EINVAL;
		
		ic->ic_def_txkey = kid;
	}
	
	/* disable crypto & make sure we allow unencrypted packets again */
        if (erq->flags & IW_ENCODE_DISABLED) {
		wepchange = (ic->ic_flags & IEEE80211_F_PRIVACY) != 0;
		ic->ic_flags &= ~IEEE80211_F_PRIVACY;
		ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
	}
	/* allow unencrypted packets */
	if (erq->flags & IW_ENCODE_OPEN) {
		ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
	}
	/* dont allow unencrypted packets & make sure privacy is enabled */
	if (erq->flags & IW_ENCODE_RESTRICTED) {
		wepchange = (ic->ic_flags & IEEE80211_F_PRIVACY) == 0;
		ic->ic_flags |= IEEE80211_F_PRIVACY;
		ic->ic_flags |= IEEE80211_F_DROPUNENC;
	}
	
	if (error == 0 && IS_UP(ic->ic_dev)) {
		/*
		 * Device is up and running; we must kick it to
		 * effect the change.  If we're enabling/disabling
		 * crypto use then we must re-initialize the device
		 * so the 802.11 state machine is reset.  Otherwise
		 * the key state should have been updated above.
		 */
		if (wepchange && ic->ic_roaming == IEEE80211_ROAMING_AUTO)
			error = -(*ic->ic_init)(ic->ic_dev);
	}
	return error;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwencode);

int
ieee80211_ioctl_giwencode(struct ieee80211com *ic,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *key)
{
	struct ieee80211_key *k;
	int error, kid;

	if (ic->ic_flags & IEEE80211_F_PRIVACY) {
		error = getiwkeyix(ic, erq, &kid);
		if (error != 0)
			return -error;
		k = &ic->ic_nw_keys[kid];
		/* XXX no way to return cipher/key type */

		erq->flags = kid + 1;			/* NB: base 1 */
		if (erq->length > k->wk_keylen)
			erq->length = k->wk_keylen;
		memcpy(key, k->wk_key, erq->length);
		erq->flags |= IW_ENCODE_ENABLED;
	} else {
		erq->length = 0;
		erq->flags = IW_ENCODE_DISABLED;
	}
	if (ic->ic_flags & IEEE80211_F_DROPUNENC)
		erq->flags |= IW_ENCODE_RESTRICTED;
	else
		erq->flags |= IW_ENCODE_OPEN;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwencode);

#ifndef ifr_media
#define	ifr_media	ifr_ifru.ifru_ivalue
#endif

int
ieee80211_ioctl_siwrate(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ifreq ifr;
	int rate;

	if (!ic->ic_media.ifm_cur)
		return -EINVAL;
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~ IFM_TMASK;
	if (rrq->fixed) {
		/* XXX fudge checking rates */
		rate = ieee80211_rate2media(ic, 2 * rrq->value / 1000000,
				ic->ic_curmode);
		if (rate == IFM_AUTO)		/* NB: unknown rate */
			return -EINVAL;
	} else
		rate = IFM_AUTO;
	ifr.ifr_media |= IFM_SUBTYPE(rate);

	return -ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}
EXPORT_SYMBOL(ieee80211_ioctl_siwrate);

int
ieee80211_ioctl_giwrate(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ifmediareq imr;
	int rate;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);

	rrq->fixed = IFM_SUBTYPE(ic->ic_media.ifm_media) != IFM_AUTO;
	/* media status will have the current xmit rate if available */
	rate = ieee80211_media2rate(imr.ifm_active);
	if (rate == -1)		/* IFM_AUTO */
		rate = 0;
	rrq->value = 1000000 * (rate / 2);

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwrate);

int
ieee80211_ioctl_siwsens(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwsens);

int
ieee80211_ioctl_giwsens(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	sens->value = 0;
	sens->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwsens);

int
ieee80211_ioctl_siwrts(struct ieee80211com *ic,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{
	u16 val;

	if (rts->disabled)
		val = IEEE80211_RTS_MAX;
	else if (IEEE80211_RTS_MIN <= rts->value &&
	    rts->value <= IEEE80211_RTS_MAX)
		val = rts->value;
	else
		return -EINVAL;
	if (val != ic->ic_rtsthreshold) {
		ic->ic_rtsthreshold = val;
		if (IS_UP(ic->ic_dev))
			return -(*ic->ic_reset)(ic->ic_dev);
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwrts);

int
ieee80211_ioctl_giwrts(struct ieee80211com *ic,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{

	rts->value = ic->ic_rtsthreshold;
	rts->disabled = (rts->value == IEEE80211_RTS_MAX);
	rts->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwrts);

int
ieee80211_ioctl_siwfrag(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{
	u16 val;

	if (rts->disabled)
		val = 2346;
	else if (rts->value < 256 || rts->value > 2346)
		return -EINVAL;
	else
		val = rts->value & ~0x1; /* even numbers only */
	if (val != ic->ic_fragthreshold) {
		ic->ic_fragthreshold = val;
		if (IS_UP(ic->ic_dev))
			return -(*ic->ic_reset)(ic->ic_dev);
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwfrag);

int
ieee80211_ioctl_giwfrag(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{

	rts->value = ic->ic_fragthreshold;
	rts->disabled = (rts->value == 2346);
	rts->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwfrag);

int
ieee80211_ioctl_siwap(struct ieee80211com *ic,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{
	static const u_int8_t zero_bssid[IEEE80211_ADDR_LEN];

	/* NB: should not be set when in HOSTAP mode */
	if (ic->ic_opmode != IEEE80211_M_STA &&
	    ic->ic_opmode != IEEE80211_M_AHDEMO &&
	    ic->ic_opmode != IEEE80211_M_IBSS)
		return -EINVAL;
	IEEE80211_ADDR_COPY(ic->ic_des_bssid, &ap_addr->sa_data);
	/* looks like a zero address disables */
	if (IEEE80211_ADDR_EQ(ic->ic_des_bssid, zero_bssid))
		ic->ic_flags &= ~IEEE80211_F_DESBSSID;
	else
		ic->ic_flags |= IEEE80211_F_DESBSSID;
	return IS_UP_AUTO(ic) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwap);

int
ieee80211_ioctl_giwap(struct ieee80211com *ic,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{

	if (ic->ic_flags & IEEE80211_F_DESBSSID)
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_des_bssid);
	else
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_bss->ni_bssid);
	ap_addr->sa_family = ARPHRD_ETHER;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwap);

int
ieee80211_ioctl_siwnickn(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{

	if (data->length > IEEE80211_NWID_LEN)
		return -EINVAL;

	memset(ic->ic_nickname, 0, IEEE80211_NWID_LEN);
	memcpy(ic->ic_nickname, nickname, data->length);
	ic->ic_nicknamelen = data->length;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwnickn);

int
ieee80211_ioctl_giwnickn(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{

	if (data->length > ic->ic_nicknamelen + 1)
		data->length = ic->ic_nicknamelen + 1;
	if (data->length > 0) {
		memcpy(nickname, ic->ic_nickname, data->length-1);
		nickname[data->length-1] = '\0';
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwnickn);

static struct ieee80211_channel *
getcurchan(struct ieee80211com *ic)
{
	switch (ic->ic_state) {
	case IEEE80211_S_INIT:
	case IEEE80211_S_SCAN:
		return ic->ic_des_chan;
	default:
		return ic->ic_ibss_chan;
	}
}

static int
cap2cipher(int flag)
{
	switch (flag) {
	case IEEE80211_C_WEP:		return IEEE80211_CIPHER_WEP;
	case IEEE80211_C_AES:		return IEEE80211_CIPHER_AES_OCB;
	case IEEE80211_C_AES_CCM:	return IEEE80211_CIPHER_AES_CCM;
	case IEEE80211_C_CKIP:		return IEEE80211_CIPHER_CKIP;
	case IEEE80211_C_TKIP:		return IEEE80211_CIPHER_TKIP;
	}
	return -1;
}

int
ieee80211_ioctl_siwfreqx(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_freq *freq, char *extra)
{
	struct ieee80211_channel *c;
	int i;
	
	if (freq->e > 1)
		return -EINVAL;
	if (freq->e == 1)
		i = ieee80211_mhz2ieee(freq->m / 100000, 0);
	else
		i = freq->m;
	if (i != 0) {
		if (i > IEEE80211_CHAN_MAX || isclr(ic->ic_chan_active, i))
			return -EINVAL;
		c = &ic->ic_channels[i];
		if (c == getcurchan(ic)) {	/* no change, just return */
			ic->ic_des_chan = c;	/* XXX */
			return 0;
		}
		ic->ic_des_chan = c;
		if (c != IEEE80211_CHAN_ANYC)
			ic->ic_ibss_chan = c;
	} else {
		/*
		 * Intepret channel 0 to mean "no desired channel";
		 * otherwise there's no way to undo fixing the desired
		 * channel.
		 */
		if (ic->ic_des_chan == IEEE80211_CHAN_ANYC)
			return 0;
		ic->ic_des_chan = IEEE80211_CHAN_ANYC;
	}
	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
	else
		return IS_UP_AUTO(ic) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwfreqx);

int
ieee80211_ioctl_giwfreq(struct ieee80211com *ic,
				struct iw_request_info *info,
				struct iw_freq *freq, char *extra)
{

	if (!ic->ic_ibss_chan)
		return -EINVAL;

	freq->m = ic->ic_ibss_chan->ic_freq * 100000;
	freq->e = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwfreq);

int
ieee80211_ioctl_siwessid(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *ssid)
{

	if (data->flags == 0) {		/* ANY */
		memset(ic->ic_des_essid, 0, sizeof(ic->ic_des_essid));
		ic->ic_des_esslen = 0;
	} else {
		if (data->length > sizeof(ic->ic_des_essid))
			data->length = sizeof(ic->ic_des_essid);
		memcpy(ic->ic_des_essid, ssid, data->length);
		ic->ic_des_esslen = data->length;
		/*
		 * Deduct a trailing \0 since iwconfig passes a string
		 * length that includes this.  Unfortunately this means
		 * that specifying a string with multiple trailing \0's
		 * won't be handled correctly.  Not sure there's a good
		 * solution; the API is botched (the length should be
		 * exactly those bytes that are meaningful and not include
		 * extraneous stuff).
		 */
		if (ic->ic_des_esslen > 0 &&
		    ic->ic_des_essid[ic->ic_des_esslen-1] == '\0')
			ic->ic_des_esslen--;
	}
	return IS_UP_AUTO(ic) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwessid);

int
ieee80211_ioctl_giwessid(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *essid)
{

	data->flags = 1;		/* active */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		if (data->length > ic->ic_des_esslen)
			data->length = ic->ic_des_esslen;
		memcpy(essid, ic->ic_des_essid, data->length);
	} else {
		if (ic->ic_des_esslen == 0) {
			if (data->length > ic->ic_bss->ni_esslen)
				data->length = ic->ic_bss->ni_esslen;
			memcpy(essid, ic->ic_bss->ni_essid, data->length);
		} else {
			if (data->length > ic->ic_des_esslen)
				data->length = ic->ic_des_esslen;
			memcpy(essid, ic->ic_des_essid, data->length);
		}
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwessid);

int
ieee80211_ioctl_giwrange(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *extra)
{
	struct ieee80211_node *ni = ic->ic_bss;
	struct iw_range *range = (struct iw_range *) extra;
	struct ieee80211_rateset *rs;
	int i, r;

	data->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	/* TODO: could fill num_txpower and txpower array with
	 * something; however, there are 128 different values.. */

	range->txpower_capa = IW_TXPOW_DBM;

	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_IBSS) {
		range->min_pmp = 1 * 1024;
		range->max_pmp = 65535 * 1024;
		range->min_pmt = 1 * 1024;
		range->max_pmt = 1000 * 1024;
		range->pmp_flags = IW_POWER_PERIOD;
		range->pmt_flags = IW_POWER_TIMEOUT;
		range->pm_capa = IW_POWER_PERIOD | IW_POWER_TIMEOUT |
			IW_POWER_UNICAST_R | IW_POWER_ALL_R;
	}

	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 13;

	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->min_retry = 0;
	range->max_retry = 255;

	range->num_channels = IEEE80211_CHAN_MAX;	/* XXX */

	range->num_frequency = 0;
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++)
		if (isset(ic->ic_chan_active, i)) {
			range->freq[range->num_frequency].i = i;
			range->freq[range->num_frequency].m =
				ic->ic_channels[i].ic_freq * 100000;
			range->freq[range->num_frequency].e = 1;
			if (++range->num_frequency == IW_MAX_FREQUENCIES)
				break;
		}

	/* Max quality is max field value minus noise floor */
	range->max_qual.qual  = 0xff - 161;

	/*
	 * In order to use dBm measurements, 'level' must be lower
	 * than any possible measurement (see iw_print_stats() in
	 * wireless tools).  It's unclear how this is meant to be
	 * done, but setting zero in these values forces dBm and
	 * the actual numbers are not used.
	 */
	range->max_qual.level = 0;
	range->max_qual.noise = 0;

	range->sensitivity = 3;

	range->max_encoding_tokens = IEEE80211_WEP_NKID;
	/* XXX query driver to find out supported key sizes */
	range->num_encoding_sizes = 3;
	range->encoding_size[0] = 5;		/* 40-bit */
	range->encoding_size[1] = 13;		/* 104-bit */
	range->encoding_size[2] = 16;		/* 128-bit */

	/* XXX this only works for station mode */
	rs = &ni->ni_rates;
	range->num_bitrates = rs->rs_nrates;
	if (range->num_bitrates > IW_MAX_BITRATES)
		range->num_bitrates = IW_MAX_BITRATES;
	for (i = 0; i < range->num_bitrates; i++) {
		r = rs->rs_rates[i] & IEEE80211_RATE_VAL;
		range->bitrate[i] = (r * 1000000) / 2;
	}

	/* estimated maximum TCP throughput values (bps) */
	range->throughput = 5500000;

	range->min_rts = 0;
	range->max_rts = 2347;
	range->min_frag = 256;
	range->max_frag = 2346;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwrange);

int
ieee80211_ioctl_siwmode(struct ieee80211com *ic,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ifreq ifr;
	if (!ic->ic_media.ifm_cur)
		return -EINVAL;
	memset(&ifr, 0, sizeof(ifr));
	/* NB: remove any fixed-rate at the same time */
	/* also remove pseudo_ibss (IFM_FLAG0) flag */
	ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~
		(IFM_OMASK | IFM_TMASK | IFM_FLAG0);
	switch (*mode) {
	case IW_MODE_INFRA:
		/* NB: this is the default */
		ic->ic_des_chan = IEEE80211_CHAN_ANYC;
		break;
	case IW_MODE_ADHOC:
		ifr.ifr_media |= IFM_IEEE80211_ADHOC;
		break;
	case IW_MODE_MASTER:
		ifr.ifr_media |= IFM_IEEE80211_HOSTAP;
		break;
#if WIRELESS_EXT >= 15
	case IW_MODE_MONITOR:
		ifr.ifr_media |= IFM_IEEE80211_MONITOR;
		break;
#endif
	default:
		return -EINVAL;
	}
	if (ic->ic_curmode == IEEE80211_MODE_TURBO_G ||
	    ic->ic_curmode == IEEE80211_MODE_TURBO_A) {
		ifr.ifr_media |= IFM_IEEE80211_TURBO;
	}

	return -ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}
EXPORT_SYMBOL(ieee80211_ioctl_siwmode);

int
ieee80211_ioctl_giwmode(struct ieee80211com *ic,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ifmediareq imr;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);

	if (imr.ifm_active & IFM_IEEE80211_HOSTAP)
		*mode = IW_MODE_MASTER;
#if WIRELESS_EXT >= 15
	else if (imr.ifm_active & IFM_IEEE80211_MONITOR)
		*mode = IW_MODE_MONITOR;
#endif
	else if (imr.ifm_active & IFM_IEEE80211_ADHOC)
		*mode = IW_MODE_ADHOC;
	else
		*mode = IW_MODE_INFRA;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwmode);

int
ieee80211_ioctl_siwpower(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *wrq, char *extra)
{

	if (wrq->disabled) {
		if (ic->ic_flags & IEEE80211_F_PMGTON) {
			ic->ic_flags &= ~IEEE80211_F_PMGTON;
			goto done;
		}
		return 0;
	}

	if ((ic->ic_caps & IEEE80211_C_PMGT) == 0)
		return -EOPNOTSUPP;
	switch (wrq->flags & IW_POWER_MODE) {
	case IW_POWER_UNICAST_R:
	case IW_POWER_ALL_R:
	case IW_POWER_ON:
		ic->ic_flags |= IEEE80211_F_PMGTON;
		break;
	default:
		return -EINVAL;
	}
	if (wrq->flags & IW_POWER_TIMEOUT) {
		ic->ic_holdover = wrq->value / 1024;
		ic->ic_flags |= IEEE80211_F_PMGTON;
	}
	if (wrq->flags & IW_POWER_PERIOD) {
		ic->ic_lintval = wrq->value / 1024;
		ic->ic_flags |= IEEE80211_F_PMGTON;
	}
done:
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwpower);

int
ieee80211_ioctl_giwpower(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{

	rrq->disabled = (ic->ic_flags & IEEE80211_F_PMGTON) == 0;
	if (!rrq->disabled) {
		switch (rrq->flags & IW_POWER_TYPE) {
		case IW_POWER_TIMEOUT:
			rrq->flags = IW_POWER_TIMEOUT;
			rrq->value = ic->ic_holdover * 1024;
			break;
		case IW_POWER_PERIOD:
			rrq->flags = IW_POWER_PERIOD;
			rrq->value = ic->ic_lintval * 1024;
			break;
		}
		rrq->flags |= IW_POWER_ALL_R;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwpower);

int
ieee80211_ioctl_siwretry(struct ieee80211com *ic,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{

	if (rrq->disabled) {
		if (ic->ic_flags & IEEE80211_F_SWRETRY) {
			ic->ic_flags &= ~IEEE80211_F_SWRETRY;
			goto done;
		}
		return 0;
	}

	if ((ic->ic_caps & IEEE80211_C_SWRETRY) == 0)
		return -EOPNOTSUPP;
	if (rrq->flags == IW_RETRY_LIMIT) {
		if (rrq->value >= 0) {
			ic->ic_txmin = rrq->value;
			ic->ic_txmax = rrq->value;	/* XXX */
			ic->ic_txlifetime = 0;		/* XXX */
			ic->ic_flags |= IEEE80211_F_SWRETRY;
		} else {
			ic->ic_flags &= ~IEEE80211_F_SWRETRY;
		}
		return 0;
	}
done:
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwretry);

int
ieee80211_ioctl_giwretry(struct ieee80211com *ic,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{

	rrq->disabled = (ic->ic_flags & IEEE80211_F_SWRETRY) == 0;
	if (!rrq->disabled) {
		switch (rrq->flags & IW_RETRY_TYPE) {
		case IW_RETRY_LIFETIME:
			rrq->flags = IW_RETRY_LIFETIME;
			rrq->value = ic->ic_txlifetime * 1024;
			break;
		case IW_RETRY_LIMIT:
			rrq->flags = IW_RETRY_LIMIT;
			switch (rrq->flags & IW_RETRY_MODIFIER) {
			case IW_RETRY_MIN:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = ic->ic_txmin;
				break;
			case IW_RETRY_MAX:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = ic->ic_txmax;
				break;
			}
			break;
		}
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwretry);

int
ieee80211_ioctl_siwtxpow(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{
	int fixed, disabled;

        fixed = (ic->ic_flags & IEEE80211_F_TXPOW_FIXED);
        disabled = (fixed && ic->ic_bss->ni_txpower == 0);

	if (rrq->fixed) {
		if ((ic->ic_caps & IEEE80211_C_TXPMGT) == 0)
			return -EOPNOTSUPP;
                if (!(IEEE80211_TXPOWER_MIN < rrq->value &&
                      rrq->value < IEEE80211_TXPOWER_MAX))
                        return -EINVAL;
		ic->ic_txpowlimit = rrq->value;
		ic->ic_flags |= IEEE80211_F_TXPOW_FIXED;
	} else {
		if (!fixed)		/* no change */
			return 0;
		ic->ic_flags &= ~IEEE80211_F_TXPOW_FIXED;
	}
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwtxpow);

int
ieee80211_ioctl_giwtxpow(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{

	rrq->value = ic->ic_txpowlimit;
        if (!(IEEE80211_TXPOWER_MIN < rrq->value &&
              rrq->value < IEEE80211_TXPOWER_MAX))
    		rrq->value = IEEE80211_TXPOWER_MAX;
	rrq->fixed = (ic->ic_flags & IEEE80211_F_TXPOW_FIXED) != 0;
	rrq->disabled = (rrq->fixed && rrq->value == 0);
	rrq->flags = IW_TXPOW_MWATT;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwtxpow);

/* is obsolete iwlist ap */
int
ieee80211_ioctl_iwaplist(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	struct ieee80211_node_table *nt;
	struct ieee80211_node *ni;
	struct sockaddr addr[IW_MAX_AP];
	struct iw_quality qual[IW_MAX_AP];
	int i;

	i = 0;
	
	if (ic->ic_opmode == IEEE80211_M_STA)
		nt = &ic->ic_scan;
	else
		nt = &ic->ic_sta;
	
	/*
	 * TODO: use ieee80211_iterate_nodes(&ic->ic_scan,func,&args)
	 * also create a func like bsd does: wi_read_ap_result()
	 */
	
	IEEE80211_NODE_LOCK(nt);
	TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
		if (ic->ic_opmode == IEEE80211_M_HOSTAP ||
		    ic->ic_opmode == IEEE80211_M_IBSS) {
			if (ni == ic->ic_bss) /* don't include BSS node */
				continue;
			IEEE80211_ADDR_COPY(addr[i].sa_data, ni->ni_macaddr);
		}
		else
			IEEE80211_ADDR_COPY(addr[i].sa_data, ni->ni_bssid);
		addr[i].sa_family = ARPHRD_ETHER;
		set_quality(&qual[i], (*ic->ic_node_getrssi)(ni));
		if (++i >= IW_MAX_AP)
			break;
	}
	IEEE80211_NODE_UNLOCK(nt);
	data->length = i;
	memcpy(extra, &addr, i*sizeof(addr[0]));
	data->flags = 1;		/* signal quality present (sort of) */
	memcpy(extra + i*sizeof(addr[0]), &qual, i*sizeof(qual[i]));

	return 0;

}
EXPORT_SYMBOL(ieee80211_ioctl_iwaplist);

#ifdef SIOCGIWSCAN
int
ieee80211_ioctl_siwscan(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	u_char *chanlist = ic->ic_chan_active;
	//u_char *chanlist = ic->ic_chan_avail; /* TODO: check */
	int i;

	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		return -EINVAL;

	/*
	 * XXX don't permit a scan to be started unless we
	 * know the device is ready.  For the moment this means
	 * the device is marked up as this is the required to
	 * initialize the hardware.  It would be better to permit
	 * scanning prior to being up but that'll require some
	 * changes to the infrastructure.
	 */
	if (!IS_UP(ic->ic_dev))
		return -ENETDOWN;
	if (ic->ic_state == IEEE80211_S_SCAN &&
	    (ic->ic_flags & (IEEE80211_F_SCAN|IEEE80211_F_ASCAN))) {
		return -EINPROGRESS;
	}
	if (ic->ic_ibss_chan == NULL ||
	    isclr(chanlist, ieee80211_chan2ieee(ic, ic->ic_ibss_chan))) {
		for (i = 0; i <= IEEE80211_CHAN_MAX; i++)
			if (isset(chanlist, i)) {
				ic->ic_ibss_chan = &ic->ic_channels[i];
				goto found;
			}
		return -EINVAL;			/* no active channels */
found:
		;
	}
	if (ic->ic_bss->ni_chan == IEEE80211_CHAN_ANYC ||
	    isclr(chanlist, ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan)))
		ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	memcpy(ic->ic_chan_active, chanlist, sizeof(ic->ic_chan_active));
 	/*
	 * We force the state to INIT before calling ieee80211_new_state
	 * to get ieee80211_begin_scan called.  We really want to scan w/o
	 * altering the current state but that's not possible right now.
	 */
	/* XXX handle proberequest case */
	if (ic->ic_state != IEEE80211_S_INIT)
		ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

	ieee80211_new_state(ic, IEEE80211_S_SCAN, 1);
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwscan);


#if WIRELESS_EXT > 14
/*
 * Encode a WPA or RSN information element as a custom
 * element using the hostap format.
 */
static u_int
encode_ie(void *buf, size_t bufsize,
	const u_int8_t *ie, size_t ielen,
	const char *leader, size_t leader_len)
{
	u_int8_t *p;
	u_int i;

	if (bufsize < leader_len)
		return 0;
	p = buf;
	memcpy(p, leader, leader_len);
	bufsize -= leader_len;
	p += leader_len;
	for (i = 0; i < ielen && bufsize > 2; i++)
		p += sprintf(p, "%02x", ie[i]);
	return (i == ielen ? p - (u_int8_t *)buf : 0);
}
#endif /* WIRELESS_EXT > 14 */

int
ieee80211_ioctl_giwscan(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	char *current_ev = extra;
	struct iw_event iwe;
	struct read_ap_args args;

	/* XXX use generation number and always return current results */
	if ((ic->ic_flags & (IEEE80211_F_SCAN|IEEE80211_F_ASCAN)) &&
	    !(ic->ic_flags & IEEE80211_F_SSCAN)) {
		/*
		 * Still scanning, indicate the caller should try again.
		 */
		return -EAGAIN;
	}
	/*
	 * Do two passes to insure WPA/non-WPA scan candidates
	 * are sorted to the front.  This is a hack to deal with
	 * the wireless extensions capping scan results at
	 * IW_SCAN_MAX_DATA bytes.  In densely populated environments
	 * it's easy to overflow this buffer (especially with WPA/RSN
	 * information elements).  Note this sorting hack does not
	 * guarantee we won't overflow anyway.
	 */
	args.i = 0;
	args.iwe = &iwe;
	args.start = extra;
	args.current_ev = current_ev;
	args.mode = ic->ic_flags & IEEE80211_F_WPA;
	ieee80211_iterate_nodes(&ic->ic_scan, read_ap_result, &args);
	args.mode = args.mode ? 0 : IEEE80211_F_WPA;
	ieee80211_iterate_nodes(&ic->ic_scan, read_ap_result, &args);

	data->length = args.current_ev - extra;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwscan);
#endif /* SIOCGIWSCAN */

static int
cipher2cap(int cipher)
{
	switch (cipher) {
	case IEEE80211_CIPHER_WEP:	return IEEE80211_C_WEP;
	case IEEE80211_CIPHER_AES_OCB:	return IEEE80211_C_AES;
	case IEEE80211_CIPHER_AES_CCM:	return IEEE80211_C_AES_CCM;
	case IEEE80211_CIPHER_CKIP:	return IEEE80211_C_CKIP;
	case IEEE80211_CIPHER_TKIP:	return IEEE80211_C_TKIP;
	}
	return 0;
}

int
ieee80211_ioctl_setparam(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct ieee80211_rsnparms *rsn = &ic->ic_bss->ni_rsn;
	int *i = (int *) extra;
	int param = i[0];		/* parameter id is 1st */
	int value = i[1];		/* NB: most values are TYPE_INT */
	struct ifreq ifr;
	int retv = 0;
	int j, caps;
	const struct ieee80211_authenticator *auth;
	const struct ieee80211_aclator *acl;

	switch (param) {
	case IEEE80211_PARAM_TURBO:
		if (!ic->ic_media.ifm_cur)
			return -EINVAL;
		memset(&ifr, 0, sizeof(ifr));
		ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media;
		if (value)
			ifr.ifr_media |= IFM_IEEE80211_TURBO;
		else
			ifr.ifr_media &= ~IFM_IEEE80211_TURBO;
		retv = ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
		break;
	case IEEE80211_PARAM_MODE:
		if (!ic->ic_media.ifm_cur)
			return -EINVAL;
		memset(&ifr, 0, sizeof(ifr));
		ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~ IFM_MMASK;
		ifr.ifr_media |= IFM_MAKEMODE(value);
		retv = ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
		break;
	case IEEE80211_PARAM_AUTHMODE:
		switch (value) {
		case IEEE80211_AUTH_WPA:	/* WPA */
		case IEEE80211_AUTH_8021X:	/* 802.1x */
		case IEEE80211_AUTH_OPEN:	/* open */
		case IEEE80211_AUTH_SHARED:	/* shared-key */
		case IEEE80211_AUTH_AUTO:	/* auto */
			auth = ieee80211_authenticator_get(value);
			if (auth == NULL)
				return -EINVAL;
			break;
		default:
			return -EINVAL;
		}
		switch (value) {
		case IEEE80211_AUTH_WPA:	/* WPA w/ 802.1x */
			ic->ic_flags |= IEEE80211_F_PRIVACY;
			value = IEEE80211_AUTH_8021X;
			break;
		case IEEE80211_AUTH_OPEN:	/* open */
			ic->ic_flags &= ~(IEEE80211_F_WPA|IEEE80211_F_PRIVACY);
			break;
		case IEEE80211_AUTH_SHARED:	/* shared-key */
		case IEEE80211_AUTH_8021X:	/* 802.1x */
			ic->ic_flags &= ~IEEE80211_F_WPA;
			/* both require a key so mark the PRIVACY capability */
			ic->ic_flags |= IEEE80211_F_PRIVACY;
			break;
		case IEEE80211_AUTH_AUTO:	/* auto */
			ic->ic_flags &= ~IEEE80211_F_WPA;
			/* XXX PRIVACY handling? */
			/* XXX what's the right way to do this? */
			break;
		}
		/* NB: authenticator attach/detach happens on state change */
		ic->ic_bss->ni_authmode = value;
		/* XXX mixed/mode/usage? */
		ic->ic_auth = auth;
		retv = ENETRESET;
		break;
	case IEEE80211_PARAM_PROTMODE:
		if (value > IEEE80211_PROT_RTSCTS)
			return -EINVAL;
		ic->ic_protmode = value;
		/* NB: if not operating in 11g this can wait */
		if (ic->ic_curmode == IEEE80211_MODE_11G ||
		    ic->ic_curmode == IEEE80211_MODE_TURBO_G)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_MCASTCIPHER:
		/* XXX s/w implementations */
		if ((ic->ic_caps & cipher2cap(value)) == 0)
			return -EINVAL;
		rsn->rsn_mcastcipher = value;
		if (ic->ic_flags & IEEE80211_F_WPA)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_MCASTKEYLEN:
		if (!(0 < value && value < IEEE80211_KEYBUF_SIZE))
			return -EINVAL;
		/* XXX no way to verify driver capability */
		rsn->rsn_mcastkeylen = value;
		if (ic->ic_flags & IEEE80211_F_WPA)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_UCASTCIPHERS:
		/*
		 * Convert cipher set to equivalent capabilities.
		 * NB: this logic intentionally ignores unknown and
		 * unsupported ciphers so folks can specify 0xff or
		 * similar and get all available ciphers.
		 */
		caps = 0;
		for (j = 1; j < 32; j++)	/* NB: skip WEP */
			if (value & (1<<j))
				caps |= cipher2cap(j);
		/* XXX s/w implementations */
		caps &= ic->ic_caps;	/* restrict to supported ciphers */
		/* XXX verify ciphers ok for unicast use? */
		/* XXX disallow if running as it'll have no effect */
		rsn->rsn_ucastcipherset = caps;
		if (ic->ic_flags & IEEE80211_F_WPA)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_UCASTCIPHER:
		if ((rsn->rsn_ucastcipherset & cipher2cap(value)) == 0)
			return -EINVAL;
		rsn->rsn_ucastcipher = value;
		break;
	case IEEE80211_PARAM_UCASTKEYLEN:
		if (!(0 < value && value < IEEE80211_KEYBUF_SIZE))
			return -EINVAL;
		/* XXX no way to verify driver capability */
		rsn->rsn_ucastkeylen = value;
		break;
	case IEEE80211_PARAM_KEYMGTALGS:
		/* XXX check */
		rsn->rsn_keymgmtset = value;
		if (ic->ic_flags & IEEE80211_F_WPA)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_RSNCAPS:
		/* XXX check */
		rsn->rsn_caps = value;
		if (ic->ic_flags & IEEE80211_F_WPA)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_WPA:
		if (value > 3)
			return -EINVAL;
		/* XXX verify ciphers available */
		ic->ic_flags &= ~IEEE80211_F_WPA;
		switch (value) {
		case 1:
			ic->ic_flags |= IEEE80211_F_WPA1;
			break;
		case 2:
			ic->ic_flags |= IEEE80211_F_WPA2;
			break;
		case 3:
			ic->ic_flags |= IEEE80211_F_WPA1 | IEEE80211_F_WPA2;
			break;
		}
		retv = ENETRESET;		/* XXX? */
		break;
	case IEEE80211_PARAM_ROAMING:
		if (!(IEEE80211_ROAMING_DEVICE <= value &&
		    value <= IEEE80211_ROAMING_MANUAL))
			return -EINVAL;
		ic->ic_roaming = value;
		break;
	case IEEE80211_PARAM_PRIVACY:
		if (value) {
			/* XXX check for key state? */
			ic->ic_flags |= IEEE80211_F_PRIVACY;
		} else
			ic->ic_flags &= ~IEEE80211_F_PRIVACY;
		break;
	case IEEE80211_PARAM_DROPUNENCRYPTED:
		if (value)
			ic->ic_flags |= IEEE80211_F_DROPUNENC;
		else
			ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
		break;
	case IEEE80211_PARAM_COUNTERMEASURES:
		if (value) {
			if ((ic->ic_flags & IEEE80211_F_WPA) == 0)
				return -EINVAL;
			ic->ic_flags |= IEEE80211_F_COUNTERM;
		} else
			ic->ic_flags &= ~IEEE80211_F_COUNTERM;
		break;
	case IEEE80211_PARAM_DRIVER_CAPS:
		ic->ic_caps = value;		/* NB: for testing */
		break;
	case IEEE80211_PARAM_MACCMD:
		acl = ic->ic_acl;
		switch (value) {
		case IEEE80211_MACCMD_POLICY_OPEN:
		case IEEE80211_MACCMD_POLICY_ALLOW:
		case IEEE80211_MACCMD_POLICY_DENY:
			if (acl == NULL) {
				acl = ieee80211_aclator_get("mac");
				if (acl == NULL || !acl->iac_attach(ic))
					return -EINVAL;
				ic->ic_acl = acl;
			}
			acl->iac_setpolicy(ic, value);
			break;
		case IEEE80211_MACCMD_FLUSH:
			if (acl != NULL)
				acl->iac_flush(ic);
			/* NB: silently ignore when not in use */
			break;
		case IEEE80211_MACCMD_DETACH:
			if (acl != NULL) {
				ic->ic_acl = NULL;
				acl->iac_detach(ic);
			}
			break;
		}
		break;
	case IEEE80211_PARAM_WME:
		if (ic->ic_opmode != IEEE80211_M_STA)
			return -EINVAL;
		if (value)
			ic->ic_flags |= IEEE80211_F_WME;
		else
			ic->ic_flags &= ~IEEE80211_F_WME;
		break;
	case IEEE80211_PARAM_HIDESSID:
		if (value)
			ic->ic_flags |= IEEE80211_F_HIDESSID;
		else
			ic->ic_flags &= ~IEEE80211_F_HIDESSID;
		retv = ENETRESET;
		break;
	case IEEE80211_PARAM_APBRIDGE:
		if (value == 0)
			ic->ic_flags |= IEEE80211_F_NOBRIDGE;
		else
			ic->ic_flags &= ~IEEE80211_F_NOBRIDGE;
		break;
	case IEEE80211_PARAM_INACT:
		ic->ic_inact_run = value / IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_INACT_AUTH:
		ic->ic_inact_auth = value / IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_INACT_INIT:
		ic->ic_inact_init = value / IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_IBSS:
		if (!ic->ic_media.ifm_cur)
			return -EINVAL;
		memset(&ifr, 0, sizeof(ifr));
		/* NB: remove any fixed-rate at the same time */
		ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~
			(IFM_OMASK | IFM_TMASK);
		if (value==0) {
			// IFM_FLAG0 signals adhoc demo mode
			ifr.ifr_media |= (IFM_IEEE80211_ADHOC|IFM_FLAG0);
		}
		else {
			ifr.ifr_media |= IFM_IEEE80211_ADHOC;
			ifr.ifr_media &= ~IFM_FLAG0;
		}
		retv = ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
		break;
	case IEEE80211_PARAM_PUREG:
		if (value)
		 	ic->ic_flags |= IEEE80211_F_PUREG;
		else
			ic->ic_flags &= ~IEEE80211_F_PUREG;
		/* NB: reset only if we're operating on an 11g channel */
		if (ic->ic_curmode == IEEE80211_MODE_11G)
			retv = ENETRESET;
		break;
	case IEEE80211_PARAM_WDSONLY:
		ic->ic_wdsonly = value;
		break;
	case IEEE80211_PARAM_RESET:
		ic->ic_init(ic->ic_dev);
		break;
	default:
		retv = EOPNOTSUPP;
		break;
	}
	if (retv == ENETRESET)
		retv = IS_UP_AUTO(ic) ? (*ic->ic_init)(ic->ic_dev) : 0;
	return -retv;
}
EXPORT_SYMBOL(ieee80211_ioctl_setparam);

int
ieee80211_ioctl_getparam(struct ieee80211com *ic, struct iw_request_info *info,
			void *w, char *extra)
{
	struct ieee80211_rsnparms *rsn = &ic->ic_bss->ni_rsn;
	struct ifmediareq imr;
	int *param = (int *) extra;
	u_int m;

	switch (param[0]) {
	case IEEE80211_PARAM_TURBO:
		(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);
		param[0] = (imr.ifm_active & IFM_IEEE80211_TURBO) != 0;
		break;
	case IEEE80211_PARAM_MODE:
		(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);
		switch (IFM_MODE(imr.ifm_active)) {
		case IFM_IEEE80211_11A:
			param[0] = 1;
			break;
		case IFM_IEEE80211_11B:
			param[0] = 2;
			break;
		case IFM_IEEE80211_11G:
			param[0] = 3;
			break;
		case IFM_IEEE80211_FH:
			param[0] = 4;
			break;
		case IFM_AUTO:
			param[0] = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IEEE80211_PARAM_AUTHMODE:
		if (ic->ic_flags & IEEE80211_F_WPA)
			param[0] = IEEE80211_AUTH_WPA;
		else
			param[0] = ic->ic_bss->ni_authmode;
		break;
	case IEEE80211_PARAM_PROTMODE:
		param[0] = ic->ic_protmode;
		break;
	case IEEE80211_PARAM_MCASTCIPHER:
		param[0] = rsn->rsn_mcastcipher;
		break;
	case IEEE80211_PARAM_MCASTKEYLEN:
		param[0] = rsn->rsn_mcastkeylen;
		break;
	case IEEE80211_PARAM_UCASTCIPHERS:
		param[0] = 0;
		for (m = 0x1; m != 0; m <<= 1)
			if (rsn->rsn_ucastcipherset & m)
				param[0] |= 1<<cap2cipher(m);
		break;
	case IEEE80211_PARAM_UCASTCIPHER:
		param[0] = rsn->rsn_ucastcipher;
		break;
	case IEEE80211_PARAM_UCASTKEYLEN:
		param[0] = rsn->rsn_ucastkeylen;
		break;
	case IEEE80211_PARAM_KEYMGTALGS:
		param[0] = rsn->rsn_keymgmtset;
		break;
	case IEEE80211_PARAM_RSNCAPS:
		param[0] = rsn->rsn_caps;
		break;
	case IEEE80211_PARAM_WPA:
		switch (ic->ic_flags & IEEE80211_F_WPA) {
		case IEEE80211_F_WPA1:
			param[0] = 1;
			break;
		case IEEE80211_F_WPA2:
			param[0] = 2;
			break;
		case IEEE80211_F_WPA1 | IEEE80211_F_WPA2:
			param[0] = 3;
			break;
		default:
			param[0] = 0;
			break;
		}
		break;
	case IEEE80211_PARAM_ROAMING:
		param[0] = ic->ic_roaming;
		break;
	case IEEE80211_PARAM_PRIVACY:
		param[0] = (ic->ic_flags & IEEE80211_F_PRIVACY) != 0;
		break;
	case IEEE80211_PARAM_DROPUNENCRYPTED:
		param[0] = (ic->ic_flags & IEEE80211_F_DROPUNENC) != 0;
		break;
	case IEEE80211_PARAM_COUNTERMEASURES:
		param[0] = (ic->ic_flags & IEEE80211_F_COUNTERM) != 0;
		break;
	case IEEE80211_PARAM_DRIVER_CAPS:
		param[0] = ic->ic_caps;
		break;
	case IEEE80211_PARAM_WME:
		param[0] = (ic->ic_flags & IEEE80211_F_WME) != 0;
		break;
	case IEEE80211_PARAM_HIDESSID:
		param[0] = (ic->ic_flags & IEEE80211_F_HIDESSID) != 0;
		break;
	case IEEE80211_PARAM_APBRIDGE:
		param[0] = (ic->ic_flags & IEEE80211_F_NOBRIDGE) == 0;
		break;
	case IEEE80211_PARAM_INACT:
		param[0] = ic->ic_inact_run * IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_INACT_AUTH:
		param[0] = ic->ic_inact_auth * IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_INACT_INIT:
		param[0] = ic->ic_inact_init * IEEE80211_INACT_WAIT;
		break;
	case IEEE80211_PARAM_IBSS:
		param[0] = (ic->ic_opmode == IEEE80211_M_IBSS);
		break;
	case IEEE80211_PARAM_PUREG:
		param[0] = (ic->ic_flags & IEEE80211_F_PUREG) != 0;
		break;
	case IEEE80211_PARAM_WDSONLY:
		param[0] = ic->ic_wdsonly;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_getparam);

int
ieee80211_ioctl_setoptie(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	union iwreq_data *u = w;
	void *ie;

	/*
	 * NB: Doing this for ap operation could be useful (e.g. for
	 *     WPA and/or WME) except that it typically is worthless
	 *     without being able to intervene when processing
	 *     association response frames--so disallow it for now.
	 */
	if (ic->ic_opmode != IEEE80211_M_STA)
		return -EINVAL;
	/* NB: data.length is validated by the wireless extensions code */
	MALLOC(ie, void *, u->data.length, M_DEVBUF, M_WAITOK);
	if (ie == NULL)
		return -ENOMEM;
	memcpy(ie, extra, u->data.length);
	/* XXX sanity check data? */
	if (ic->ic_opt_ie != NULL)
		FREE(ic->ic_opt_ie, M_DEVBUF);
	ic->ic_opt_ie = ie;
	ic->ic_opt_ie_len = u->data.length;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_setoptie);

int
ieee80211_ioctl_getoptie(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	union iwreq_data *u = w;

	if (ic->ic_opt_ie == NULL)
		return -EINVAL;
	if (u->data.length < ic->ic_opt_ie_len)
		return -EINVAL;
	u->data.length = ic->ic_opt_ie_len;
	memcpy(extra, ic->ic_opt_ie, u->data.length);
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_getoptie);

int
ieee80211_ioctl_setkey(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct ieee80211req_key *ik = (struct ieee80211req_key *)extra;
	struct ieee80211_node *ni;
	struct ieee80211_key *wk;
	u_int16_t kid;
	int error;

	/* NB: cipher support is verified by ieee80211_crypt_newkey */
	/* NB: this also checks ik->ik_keylen > sizeof(wk->wk_key) */
	if (ik->ik_keylen > sizeof(ik->ik_keydata))
		return -E2BIG;
	kid = ik->ik_keyix;
	if (kid == IEEE80211_KEYIX_NONE) {
		/* XXX unicast keys currently must be tx/rx */
		if (ik->ik_flags != (IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV))
			return -EINVAL;
		if (ic->ic_opmode == IEEE80211_M_STA) {
			ni = ic->ic_bss;
			if (!IEEE80211_ADDR_EQ(ik->ik_macaddr, ni->ni_bssid))
				return -EADDRNOTAVAIL;
		} else
			ni = ieee80211_find_node(&ic->ic_sta, ik->ik_macaddr);
		if (ni == NULL)
			return -ENOENT;
		wk = &ni->ni_ucastkey;
	} else {
		if (kid >= IEEE80211_WEP_NKID)
			return -EINVAL;
		wk = &ic->ic_nw_keys[kid];
		ni = NULL;
		ik->ik_flags |= IEEE80211_KEY_GROUP;
	}
	error = 0;
	ieee80211_key_update_begin(ic);
	if (ieee80211_crypto_newkey(ic, ik->ik_type, ik->ik_flags, wk)) {
		wk->wk_keylen = ik->ik_keylen;
		/* NB: MIC presence is implied by cipher type */
		if (wk->wk_keylen > IEEE80211_KEYBUF_SIZE)
			wk->wk_keylen = IEEE80211_KEYBUF_SIZE;
		wk->wk_keyrsc = ik->ik_keyrsc;
		wk->wk_keytsc = 0;			/* new key, reset */
		wk->wk_flags |=
			ik->ik_flags & (IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV);
		memset(wk->wk_key, 0, sizeof(wk->wk_key));
		memcpy(wk->wk_key, ik->ik_keydata, ik->ik_keylen);
		if (!ieee80211_crypto_setkey(ic, wk,
		    ni != NULL ? ni->ni_macaddr : ik->ik_macaddr))
			error = -EIO;
		else if ((ik->ik_flags & IEEE80211_KEY_DEFAULT))
			ic->ic_def_txkey = kid;
	} else
		error = -ENXIO;
	ieee80211_key_update_end(ic);
	if (ni != NULL)
		ieee80211_free_node(ni);
	return error;
}
EXPORT_SYMBOL(ieee80211_ioctl_setkey);

static int
ieee80211_ioctl_getkey(struct ieee80211com *ic, struct iwreq *iwr)
{
	struct ieee80211_node *ni;
	struct ieee80211req_key ik;
	struct ieee80211_key *wk;
	const struct ieee80211_cipher *cip;
	u_int kid;

	if (iwr->u.data.length != sizeof(ik))
		return -EINVAL;
	if (copy_from_user(&ik, iwr->u.data.pointer, sizeof(ik)))
		return -EFAULT;
	kid = ik.ik_keyix;
	if (kid == IEEE80211_KEYIX_NONE) {
		ni = ieee80211_find_node(&ic->ic_sta, ik.ik_macaddr);
		if (ni == NULL)
			return -EINVAL;		/* XXX */
		wk = &ni->ni_ucastkey;
	} else {
		if (kid >= IEEE80211_WEP_NKID)
			return -EINVAL;
		wk = &ic->ic_nw_keys[kid];
		IEEE80211_ADDR_COPY(&ik.ik_macaddr, ic->ic_bss->ni_macaddr);
		ni = NULL;
	}
	cip = wk->wk_cipher;
	ik.ik_type = cip->ic_cipher;
	ik.ik_keylen = wk->wk_keylen;
	ik.ik_flags = wk->wk_flags & (IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV);
	if (wk->wk_keyix == ic->ic_def_txkey)
		ik.ik_flags |= IEEE80211_KEY_DEFAULT;
	if (capable(CAP_NET_ADMIN)) {
		/* NB: only root can read key data */
		ik.ik_keyrsc = wk->wk_keyrsc;
		ik.ik_keytsc = wk->wk_keytsc;
		memcpy(ik.ik_keydata, wk->wk_key, wk->wk_keylen);
		if (cip->ic_cipher == IEEE80211_CIPHER_TKIP) {
			memcpy(ik.ik_keydata+wk->wk_keylen,
				wk->wk_key + IEEE80211_KEYBUF_SIZE,
				IEEE80211_MICBUF_SIZE);
			ik.ik_keylen += IEEE80211_MICBUF_SIZE;
		}
	} else {
		ik.ik_keyrsc = 0;
		ik.ik_keytsc = 0;
		memset(ik.ik_keydata, 0, sizeof(ik.ik_keydata));
	}
	if (ni != NULL)
		ieee80211_free_node(ni);
	return (copy_to_user(iwr->u.data.pointer, &ik, sizeof(ik)) ?
			-EFAULT : 0);
}

int
ieee80211_ioctl_delkey(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct ieee80211req_del_key *dk = (struct ieee80211req_del_key *)extra;
	int kid;

	kid = dk->idk_keyix;
	/* XXX u_int8_t -> u_int16_t */
	if (dk->idk_keyix == (u_int8_t) IEEE80211_KEYIX_NONE) {
		struct ieee80211_node *ni =
			ieee80211_find_node(&ic->ic_sta, dk->idk_macaddr);
		if (ni == NULL) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: node not found\n", __func__);
			return -ENOENT;
		}
		/* XXX error return */
		ieee80211_crypto_delkey(ic, &ni->ni_ucastkey);
		ieee80211_free_node(ni);
	} else {
		if (kid >= IEEE80211_WEP_NKID) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: key id %d >= %d\n", __func__,
				kid, IEEE80211_WEP_NKID);
			return -EINVAL;
		}
		/* XXX error return */
		ieee80211_crypto_delkey(ic, &ic->ic_nw_keys[kid]);
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_delkey);

static void
domlme(void *arg, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211req_mlme *mlme = arg;

	if (ni->ni_associd != 0) {
		IEEE80211_SEND_MGMT(ic, ni,
			mlme->im_op == IEEE80211_MLME_DEAUTH ?
				IEEE80211_FC0_SUBTYPE_DEAUTH :
				IEEE80211_FC0_SUBTYPE_DISASSOC,
			mlme->im_reason);
	}
	ieee80211_node_leave(ic, ni);
}

int
ieee80211_ioctl_setmlme(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct ieee80211req_mlme *mlme = (struct ieee80211req_mlme *)extra;
	struct ieee80211_node *ni;

	switch (mlme->im_op) {
	case IEEE80211_MLME_ASSOC:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
			"%s: assoc\n", __func__);
                if (ic->ic_opmode != IEEE80211_M_STA)
                        return -EINVAL;
                /* XXX must be in S_SCAN state? */

		if (mlme->im_ssid_len != 0) {
			/*
			 * Desired ssid specified; must match both bssid and
			 * ssid to distinguish ap advertising multiple ssid's.
			 */
			ni = ieee80211_find_node_with_ssid(&ic->ic_scan,
				mlme->im_macaddr,
				mlme->im_ssid_len, mlme->im_ssid);
		} else {
			/*
			 * Normal case; just match bssid.
			 */
			ni = ieee80211_find_node(&ic->ic_scan, mlme->im_macaddr);
		}
		if (ni == NULL) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: node not found\n", __func__);
			return -ENOENT;
		}
		if (!ieee80211_sta_join(ic, ni)) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: join failed\n", __func__);
			ieee80211_free_node(ni);
			return -EINVAL;
		}
		break;
	case IEEE80211_MLME_DISASSOC:
	case IEEE80211_MLME_DEAUTH:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
			"%s: %s\n", __func__,
			(mlme->im_op == IEEE80211_MLME_DISASSOC ? "disassoc" : "deauth"));
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			/* XXX not quite right */
			ieee80211_new_state(ic, IEEE80211_S_INIT,
				mlme->im_reason);
			break;
		case IEEE80211_M_HOSTAP:
			/* NB: the broadcast address means do 'em all */
			if (!IEEE80211_ADDR_EQ(mlme->im_macaddr, ic->ic_dev->broadcast)) {
				if ((ni = ieee80211_find_node(&ic->ic_sta,
						mlme->im_macaddr)) == NULL) {
					IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
						"%s: node not found\n", __func__);
					return -ENOENT;
				}
				domlme(mlme, ni);
				ieee80211_free_node(ni);
			} else {
				ieee80211_iterate_nodes(&ic->ic_sta,
						domlme, mlme);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IEEE80211_MLME_AUTHORIZE:
	case IEEE80211_MLME_UNAUTHORIZE:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
			"%s: %sauthorize\n", __func__,
			(mlme->im_op == IEEE80211_MLME_UNAUTHORIZE ? "un" : ""));
		if (ic->ic_opmode != IEEE80211_M_HOSTAP)
			return -EINVAL;
		ni = ieee80211_find_node(&ic->ic_sta, mlme->im_macaddr);
		if (ni == NULL) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: node not found\n", __func__);
			return -ENOENT;
		}
		if (mlme->im_op == IEEE80211_MLME_AUTHORIZE)
			ieee80211_node_authorize(ni);
		else
			ieee80211_node_unauthorize(ni);
		ieee80211_free_node(ni);
		break;
	case IEEE80211_MLME_CLEAR_STATS:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
			"%s: clear stats\n", __func__);
		if (ic->ic_opmode != IEEE80211_M_HOSTAP)
			return -EINVAL;
		ni = ieee80211_find_node(&ic->ic_sta, mlme->im_macaddr);
		if (ni == NULL) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_MLME,
				"%s: node not found\n", __func__);
			return -ENOENT;
		}
		/* clear statistics */
		memset(&ni->ni_stats, 0, sizeof(struct ieee80211_nodestats));
		ieee80211_free_node(ni);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_setmlme);

int
ieee80211_ioctl_addmac(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct sockaddr *sa = (struct sockaddr *)extra;
	const struct ieee80211_aclator *acl = ic->ic_acl;

	if (acl == NULL) {
		acl = ieee80211_aclator_get("mac");
		if (acl == NULL || !acl->iac_attach(ic))
			return -EINVAL;
		ic->ic_acl = acl;
	}
	acl->iac_add(ic, sa->sa_data);
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_addmac);

int
ieee80211_ioctl_delmac(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct sockaddr *sa = (struct sockaddr *)extra;
	const struct ieee80211_aclator *acl = ic->ic_acl;

	if (acl == NULL) {
		acl = ieee80211_aclator_get("mac");
		if (acl == NULL || !acl->iac_attach(ic))
			return -EINVAL;
		ic->ic_acl = acl;
	}
	acl->iac_remove(ic, sa->sa_data);
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_delmac);

int
ieee80211_ioctl_chanlist(struct ieee80211com *ic, struct iw_request_info *info,
			void *w, char *extra)
{
	struct ieee80211req_chanlist *list =
		(struct ieee80211req_chanlist *)extra;
	u_char chanlist[roundup(IEEE80211_CHAN_MAX, NBBY)];
	int i, j;

	memset(chanlist, 0, sizeof(chanlist));
	/*
	 * Since channel 0 is not available for DS, channel 1
	 * is assigned to LSB on WaveLAN.
	 */
	if (ic->ic_phytype == IEEE80211_T_DS)
		i = 1;
	else
		i = 0;
	for (j = 0; i <= IEEE80211_CHAN_MAX; i++, j++) {
		/*
		 * NB: silently discard unavailable channels so users
		 *     can specify 1-255 to get all available channels.
		 */
		if (isset(list->ic_channels, j) && isset(ic->ic_chan_avail, i))
			setbit(chanlist, i);
	}
	if (ic->ic_ibss_chan == NULL ||
	    isclr(chanlist, ieee80211_chan2ieee(ic, ic->ic_ibss_chan))) {
		for (i = 0; i <= IEEE80211_CHAN_MAX; i++)
			if (isset(chanlist, i)) {
				ic->ic_ibss_chan = &ic->ic_channels[i];
				goto found;
			}
		return EINVAL;			/* no active channels */
found:
		;
	}
	memcpy(ic->ic_chan_active, chanlist, sizeof(ic->ic_chan_active));
	if (ic->ic_bss->ni_chan == IEEE80211_CHAN_ANYC ||
	    isclr(chanlist, ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan)))
		ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	return IS_UP_AUTO(ic) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_chanlist);

static int
ieee80211_ioctl_getwpaie(struct ieee80211com *ic, struct iwreq *iwr)
{
	struct ieee80211_node *ni;
	struct ieee80211req_wpaie wpaie;

	if (iwr->u.data.length != sizeof(wpaie))
		return -EINVAL;
	if (copy_from_user(&wpaie, iwr->u.data.pointer, IEEE80211_ADDR_LEN))
		return -EFAULT;
	ni = ieee80211_find_node(&ic->ic_sta, wpaie.wpa_macaddr);
	if (ni == NULL)
		return -EINVAL;		/* XXX */
	memset(wpaie.wpa_ie, 0, sizeof(wpaie.wpa_ie));
	if (ni->ni_wpa_ie != NULL) {
		int ielen = ni->ni_wpa_ie[1] + 2;
		if (ielen > (int)sizeof(wpaie.wpa_ie))
			ielen = sizeof(wpaie.wpa_ie);
		memcpy(wpaie.wpa_ie, ni->ni_wpa_ie, ielen);
	}
	ieee80211_free_node(ni);
	return (copy_to_user(iwr->u.data.pointer, &wpaie, sizeof(wpaie)) ?
			-EFAULT : 0);
}

static int
ieee80211_ioctl_getstastats(struct ieee80211com *ic, struct iwreq *iwr)
{
	struct ieee80211_node *ni;
	u_int8_t macaddr[IEEE80211_ADDR_LEN];
	const int off = offsetof(struct ieee80211req_sta_stats, is_stats);
	int error;

	if (iwr->u.data.length < off)
		return -EINVAL;
	if (copy_from_user(&macaddr, iwr->u.data.pointer, IEEE80211_ADDR_LEN))
		return -EFAULT;
	ni = ieee80211_find_node(&ic->ic_sta, macaddr);
	if (ni == NULL)
		return -EINVAL;		/* XXX */
	/* NB: copy out only the statistics */
	error = copy_to_user(iwr->u.data.pointer + off, &ni->ni_stats,
			iwr->u.data.length - off);
	ieee80211_free_node(ni);
	return (error ? -EFAULT : 0);
}

#define	IW_PRIV_TYPE_OPTIE	IW_PRIV_TYPE_BYTE | IEEE80211_MAX_OPT_IE
#define	IW_PRIV_TYPE_KEY \
	IW_PRIV_TYPE_BYTE | sizeof(struct ieee80211req_key)
#define	IW_PRIV_TYPE_DELKEY \
	IW_PRIV_TYPE_BYTE | sizeof(struct ieee80211req_del_key)
#define	IW_PRIV_TYPE_MLME \
	IW_PRIV_TYPE_BYTE | sizeof(struct ieee80211req_mlme)
#define	IW_PRIV_TYPE_CHANLIST \
	IW_PRIV_TYPE_BYTE | sizeof(struct ieee80211req_chanlist)

static const struct iw_priv_args ieee80211_priv_args[] = {
	/* NB: setoptie & getoptie are !IW_PRIV_SIZE_FIXED */
	{ IEEE80211_IOCTL_SETOPTIE,
	  IW_PRIV_TYPE_OPTIE, 0,			"setoptie" },
	{ IEEE80211_IOCTL_GETOPTIE,
	  0, IW_PRIV_TYPE_OPTIE,			"getoptie" },
	{ IEEE80211_IOCTL_SETKEY,
	  IW_PRIV_TYPE_KEY | IW_PRIV_SIZE_FIXED, 0,	"setkey" },
	{ IEEE80211_IOCTL_DELKEY,
	  IW_PRIV_TYPE_DELKEY | IW_PRIV_SIZE_FIXED, 0,	"delkey" },
	{ IEEE80211_IOCTL_SETMLME,
	  IW_PRIV_TYPE_MLME | IW_PRIV_SIZE_FIXED, 0,	"setmlme" },
	{ IEEE80211_IOCTL_ADDMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"addmac" },
	{ IEEE80211_IOCTL_DELMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"delmac" },
	{ IEEE80211_IOCTL_CHANLIST,
	  IW_PRIV_TYPE_CHANLIST | IW_PRIV_SIZE_FIXED, 0,"chanlist" },
	{ IEEE80211_IOCTL_WDSADD,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"wdsadd" },
	{ IEEE80211_IOCTL_WDSDEL,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"wdsdel" },
#if WIRELESS_EXT >= 12
	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "setparam" },
	/*
	 * These depends on sub-ioctl support which added in version 12.
	 */
	{ IEEE80211_IOCTL_GETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,	"getparam" },

	/* sub-ioctl handlers */
	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "" },
	{ IEEE80211_IOCTL_GETPARAM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "" },

	/* sub-ioctl definitions */
	{ IEEE80211_PARAM_TURBO,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "turbo" },
	{ IEEE80211_PARAM_TURBO,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_turbo" },
	{ IEEE80211_PARAM_MODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mode" },
	{ IEEE80211_PARAM_MODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mode" },
	{ IEEE80211_PARAM_AUTHMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "authmode" },
	{ IEEE80211_PARAM_AUTHMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_authmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "protmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_protmode" },
	{ IEEE80211_PARAM_MCASTCIPHER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mcastcipher" },
	{ IEEE80211_PARAM_MCASTCIPHER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mcastcipher" },
	{ IEEE80211_PARAM_MCASTKEYLEN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mcastkeylen" },
	{ IEEE80211_PARAM_MCASTKEYLEN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mcastkeylen" },
	{ IEEE80211_PARAM_UCASTCIPHERS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastciphers" },
	{ IEEE80211_PARAM_UCASTCIPHERS,
	/*
	 * NB: can't use "get_ucastciphers" 'cuz iwpriv command names
	 *     must be <IFNAMESIZ which is 16.
	 */
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_uciphers" },
	{ IEEE80211_PARAM_UCASTCIPHER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastcipher" },
	{ IEEE80211_PARAM_UCASTCIPHER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ucastcipher" },
	{ IEEE80211_PARAM_UCASTKEYLEN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastkeylen" },
	{ IEEE80211_PARAM_UCASTKEYLEN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ucastkeylen" },
	{ IEEE80211_PARAM_KEYMGTALGS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "keymgtalgs" },
	{ IEEE80211_PARAM_KEYMGTALGS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_keymgtalgs" },
	{ IEEE80211_PARAM_RSNCAPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rsncaps" },
	{ IEEE80211_PARAM_RSNCAPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rsncaps" },
	{ IEEE80211_PARAM_ROAMING,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "roaming" },
	{ IEEE80211_PARAM_ROAMING,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_roaming" },
	{ IEEE80211_PARAM_PRIVACY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "privacy" },
	{ IEEE80211_PARAM_PRIVACY,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_privacy" },
	{ IEEE80211_PARAM_COUNTERMEASURES,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "countermeasures" },
	{ IEEE80211_PARAM_COUNTERMEASURES,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_countermeas" },
	{ IEEE80211_PARAM_DROPUNENCRYPTED,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dropunencrypted" },
	{ IEEE80211_PARAM_DROPUNENCRYPTED,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dropunencry" },
	{ IEEE80211_PARAM_WPA,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wpa" },
	{ IEEE80211_PARAM_WPA,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wpa" },
	{ IEEE80211_PARAM_DRIVER_CAPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "driver_caps" },
	{ IEEE80211_PARAM_DRIVER_CAPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_driver_caps" },
	{ IEEE80211_PARAM_MACCMD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "maccmd" },
	{ IEEE80211_PARAM_WME,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wme" },
	{ IEEE80211_PARAM_WME,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wme" },
	{ IEEE80211_PARAM_HIDESSID,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "hide_ssid" },
	{ IEEE80211_PARAM_HIDESSID,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_hide_ssid" },
	{ IEEE80211_PARAM_APBRIDGE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ap_bridge" },
	{ IEEE80211_PARAM_APBRIDGE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ap_bridge" },
	{ IEEE80211_PARAM_INACT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact" },
	{ IEEE80211_PARAM_INACT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact" },
	{ IEEE80211_PARAM_INACT_AUTH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact_auth" },
	{ IEEE80211_PARAM_INACT_AUTH,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact_auth" },
	{ IEEE80211_PARAM_INACT_INIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact_init" },
	{ IEEE80211_PARAM_INACT_INIT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact_init" },
	{ IEEE80211_PARAM_IBSS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ibss" },
	{ IEEE80211_PARAM_IBSS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ibss" },
	{ IEEE80211_PARAM_PUREG,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "pureg" },
	{ IEEE80211_PARAM_PUREG,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_pureg" },
	{ IEEE80211_PARAM_WDSONLY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wdsonly" },
	{ IEEE80211_PARAM_WDSONLY,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wdsonly" },
	{ IEEE80211_PARAM_RESET,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "reset" },
#endif /* WIRELESS_EXT >= 12 */
};

void
ieee80211_ioctl_iwsetup(struct iw_handler_def *def)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	def->private_args = (struct iw_priv_args *) ieee80211_priv_args;
	def->num_private_args = N(ieee80211_priv_args);
#undef N
}
EXPORT_SYMBOL(ieee80211_ioctl_iwsetup);

/*
 * Handle private ioctl requests.
 */
int
ieee80211_ioctlx(struct ieee80211com *ic, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCG80211STATS:
		return copy_to_user(ifr->ifr_data, &ic->ic_stats,
				sizeof (ic->ic_stats)) ? -EFAULT : 0;
	case IEEE80211_IOCTL_GETKEY:
		return ieee80211_ioctl_getkey(ic, (struct iwreq *) ifr);
	case IEEE80211_IOCTL_GETWPAIE:
		return ieee80211_ioctl_getwpaie(ic, (struct iwreq *) ifr);
	case IEEE80211_IOCTL_GETSTASTATS:
		return ieee80211_ioctl_getstastats(ic, (struct iwreq *) ifr);
	}
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(ieee80211_ioctlx);
