#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>
#include "radar.h"


/*
 Upon "in-service" radar detection, this code handles the frequency hopping
 by finding a new channel where we haven't detected any radar or have not
 tried to use for 30 minutes
*/

/* 30 minutes */
#define RADAR_CHANNEL_REUSE_LIMIT (HZ * 60 * 30) 

static void
radar_device_reanimate(unsigned long arg)
{
	struct ieee80211com *ic = (struct ieee80211com *)arg;
	struct net_device *dev = ic->ic_dev;

	del_timer(&ic->ic_radar_reanimate);

	printk("%s: reanimating device after radar disturbance\n", dev->name);
	ic->ic_init(dev);
}

void
radar_init(struct ieee80211com *ic)
{
	memset (ic->ic_channelList, 0, sizeof (ic->ic_channelList));
	init_timer(&ic->ic_radar_reanimate);
	ic->ic_radar_reanimate.function = radar_device_reanimate;
	ic->ic_radar_reanimate.data = (unsigned long) ic;
}

struct ieee80211_channel *
radar_handle_interference(struct ieee80211com *ic)
{
    struct ieee80211_channel *c;

    int index = 0;
    int chan;
    u_int32_t firstTimeout = jiffies;;

    /* Mark current channel as having radar on it */
    chan = ic->ic_ibss_chan - ic->ic_channels;

    printk ("%s: Marking channel %d as radar disturbed, time=%lu.\n", __func__, chan, jiffies);

    if (ic->ic_ibss_chan != NULL)
	ic->ic_channelList[chan] = jiffies;

    /* Find next appropriate channel */ 
    while (index < IEEE80211_CHAN_MAX) {
		if (!isclr(ic->ic_chan_active, index) &&
		    (ic->ic_channelList[index] == 0 || 
		    (jiffies - ic->ic_channelList[index]) > RADAR_CHANNEL_REUSE_LIMIT)) {
			printk ("%s: Hopping to channel %d (%u).\n", 
				ic->ic_dev->name, index, ic->ic_channelList[index]);
			break;
		}
		/* channel blocked */
		if((ic->ic_channelList[index] != 0) &&
		    (ic->ic_channelList[index] < firstTimeout)){
			firstTimeout = ic->ic_channelList[index];
		}
		index ++;
    }
    if (index == IEEE80211_CHAN_MAX) {
		/* No channels are availiable */
		printk("%s: all channels blocked, first timeout=%i\n", 
		       ic->ic_dev->name, firstTimeout + RADAR_CHANNEL_REUSE_LIMIT);
		ic->ic_radar_reanimate.expires = firstTimeout + RADAR_CHANNEL_REUSE_LIMIT;
		add_timer(&ic->ic_radar_reanimate);
		return NULL;
    }
    c = &ic->ic_channels[index];

    return c;
}
