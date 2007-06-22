#!/bin/bash

for module in ath_{pci,rate_{amrr,minstrel,onoe,sample}} \
	      wlan_{wep,tkip,ccmp,acl,xauth,scan_{sta,ap}} ath_hal wlan
do
	grep -q ^$module /proc/modules && modprobe -r $module || true
done
