#!/bin/bash

PATTERN='\(ath_.*\|wlan_.*\|wlan\)$'

MAX_TRIES=10

fatal()
{
	echo "FATAL: $1" >&2
	exit 1
}

[ "$UID" = 0 ] || fatal "You must be root to run this script"
[ -r /proc/modules ] || fatal "Cannot read /proc/modules"

tries="$MAX_TRIES"
while [ "$tries" != "0" ]; do
	skipped=0
	cat /proc/modules | while true; do
		read -r name size use_count use_name state trailer || \
			exit "$skipped"

		expr "$name" : "$PATTERN" >/dev/null || continue

		# Compatibility for Linux 2.4.x
		[ "$state" = "" ] && { use_name="-"; state="Live"; }

		if [ "$state" != "Live" ] || [ "$use_count" != "0" ] || \
		   [ "$use_name" != "-" ]; then
			skipped=1
			if [ "$tries" = "1" ]; then
				echo "Cannot unload \"$name\"" >&2
			fi
			continue
		fi

		echo "Unloading \"$name\""
		sync	# to be safe
		/sbin/rmmod "$name" || fatal "cannot unload module \"$name\""
		sync    # to be even safer
	done
	skipped="$?"
	[ "$skipped" = "0"  ] && break
	tries=$(($tries - 1))
done

if [ "$skipped" = "1" ]; then
	echo "Error - some modules could not be unloaded" >&2
	exit 1
fi

exit 0
