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

/*net80211 definitions needed for use with madwifi*/
#include <net80211/_ieee80211.h>
#include <net80211/ieee80211.h>

#define MAX_PDU_LENGTH		IEEE80211_MAX_LEN
#define MODULATION_CCK		IEEE80211_T_CCK
#define MODULATION_OFDM		IEEE80211_T_OFDM
#define MODULATION_TURBO	IEEE80211_T_TURBO
#define MODULATION_XR 		7 /*XR thingie*/
#define MODULATION_CCK_SP 	8 /*CCK + Shortpreamble*/

#define AR5K_SET_SHORT_PREAMBLE 0x04 /* adding this flag to rate_code
					enables short preamble, see ar5212_reg.h */
#define HAS_SHPREAMBLE(_ix) (rt->rates[_ix].modulation == MODULATION_CCK_SP)
#define SHPREAMBLE_FLAG(_ix) HAS_SHPREAMBLE(_ix)?AR5K_SET_SHORT_PREAMBLE:0

typedef enum {
	AR5K_M_STA	= IEEE80211_M_STA,
	AR5K_M_IBSS	= IEEE80211_M_IBSS,
	AR5K_M_HOSTAP	= IEEE80211_M_HOSTAP,
	AR5K_M_MONITOR	= IEEE80211_M_MONITOR,
}AR5K_OPMODE;
