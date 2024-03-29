/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
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
 * $FreeBSD: src/tools/tools/ath/80211stats.c,v 1.2 2003/12/07 21:38:28 sam Exp $
 */

/*
 * 80211debug [-i interface] flags
 * (default interface is wlan0).
 */
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include <err.h>

#define	N(a)	(sizeof(a)/sizeof(a[0]))

const char *progname;

#define	IEEE80211_MSG_DEBUG	0x40000000	/* IFF_DEBUG equivalent */
#define	IEEE80211_MSG_DUMPPKTS	0x20000000	/* IFF_LINK2 equivalant */
#define	IEEE80211_MSG_CRYPTO	0x10000000	/* crypto work */
#define	IEEE80211_MSG_INPUT	0x08000000	/* input handling */
#define	IEEE80211_MSG_XRATE	0x04000000	/* rate set handling */
#define	IEEE80211_MSG_ELEMID	0x02000000	/* element id parsing */
#define	IEEE80211_MSG_NODE	0x01000000	/* node handling */
#define	IEEE80211_MSG_ASSOC	0x00800000	/* association handling */
#define	IEEE80211_MSG_AUTH	0x00400000	/* authentication handling */
#define	IEEE80211_MSG_SCAN	0x00200000	/* scanning */
#define	IEEE80211_MSG_OUTPUT	0x00100000	/* output handling */
#define	IEEE80211_MSG_STATE	0x00080000	/* state machine */
#define	IEEE80211_MSG_POWER	0x00040000	/* power save handling */
#define	IEEE80211_MSG_DOT1X	0x00020000	/* 802.1x authenticator */
#define	IEEE80211_MSG_DOT1XSM	0x00010000	/* 802.1x state machine */
#define	IEEE80211_MSG_RADIUS	0x00008000	/* 802.1x radius client */
#define	IEEE80211_MSG_RADDUMP	0x00004000	/* dump 802.1x radius packets */
#define	IEEE80211_MSG_RADKEYS	0x00002000	/* dump 802.1x keys */
#define	IEEE80211_MSG_WPA	0x00001000	/* WPA/RSN protocol */
#define	IEEE80211_MSG_ACL	0x00000800	/* ACL handling */
#define	IEEE80211_MSG_WME	0x00000400	/* WME protocol */

static struct {
	const char	*name;
	u_int		bit;
} flags[] = {
	{ "debug",	IEEE80211_MSG_DEBUG },
	{ "dumppkts",	IEEE80211_MSG_DUMPPKTS },
	{ "crypto",	IEEE80211_MSG_CRYPTO },
	{ "input",	IEEE80211_MSG_INPUT },
	{ "xrate",	IEEE80211_MSG_XRATE },
	{ "elemid",	IEEE80211_MSG_ELEMID },
	{ "node",	IEEE80211_MSG_NODE },
	{ "assoc",	IEEE80211_MSG_ASSOC },
	{ "auth",	IEEE80211_MSG_AUTH },
	{ "scan",	IEEE80211_MSG_SCAN },
	{ "output",	IEEE80211_MSG_OUTPUT },
	{ "state",	IEEE80211_MSG_STATE },
	{ "power",	IEEE80211_MSG_POWER },
	{ "dotx1",	IEEE80211_MSG_DOT1X },
	{ "dot1xsm",	IEEE80211_MSG_DOT1XSM },
	{ "radius",	IEEE80211_MSG_RADIUS },
	{ "raddump",	IEEE80211_MSG_RADDUMP },
	{ "radkeys",	IEEE80211_MSG_RADKEYS },
	{ "wpa",	IEEE80211_MSG_WPA },
	{ "acl",	IEEE80211_MSG_ACL },
	{ "wme",	IEEE80211_MSG_WME },
};

static u_int
getflag(const char *name, int len)
{
	int i;

	for (i = 0; i < N(flags); i++)
		if (strncasecmp(flags[i].name, name, len) == 0)
			return flags[i].bit;
	return 0;
}

static void
usage(void)
{
	int i;

	fprintf(stderr, "usage: %s [-i device] [[+|-]flags]\n", progname);
	fprintf(stderr, "where flags are:\n");
	for (i = 0; i < N(flags); i++)
		printf("%s\n", flags[i].name);
	exit(-1);
}

#ifdef __linux__
static int
sysctlbyname(const char *oid0, void *oldp, size_t *oldlenp, void *newp, size_t newlen)
{
	char oidcopy[1024], *oid;
	char path[1024];
	FILE *fd;

	strncpy(oidcopy, oid0, sizeof(oidcopy));
	oid = oidcopy;
	strcpy(path, "/proc/sys");
	do {
		char *cp, *tp;

		for (cp = oid; *cp != '\0' && *cp != '.'; cp++)
			;
		if (*cp == '.')
			*cp++ = '\0';
		tp = strchr(path, '\0');
		snprintf(tp, sizeof(path) - (tp-path), "/%s", oid);
		oid = cp;
	} while (*oid != '\0');
	if (oldp != NULL) {
		fd = fopen(path, "r");
		if (fd == NULL)
			return -1;
		/* XXX only handle int's */
		if (fscanf(fd, "%u", (int *) oldp) != 1) {
			fclose(fd);
			return -1;
		}
	} else {
		fd = fopen(path, "w");
		if (fd == NULL)
			return -1;
		/* XXX only handle int's */
		(void) fprintf(fd, "%u", *(int *)newp);
	}
	fclose(fd);
	return 0;
}
#endif /* __linux__ */

int
main(int argc, char *argv[])
{
	const char *ifname = "ath0";
	const char *cp, *tp;
	const char *sep;
	int op, i;
	u_int32_t debug, ndebug;
	size_t debuglen;
	char oid[256];

	progname = argv[0];
	if (argc > 1) {
		if (strcmp(argv[1], "-i") == 0) {
			if (argc < 2)
				errx(1, "missing interface name for -i option");
			ifname = argv[2];
			argc -= 2, argv += 2;
		} else if ((strcmp(argv[1], "-?") == 0) ||
		           (strcmp(argv[1], "--help") == 0))
			usage();
	}

#ifdef __linux__
	snprintf(oid, sizeof(oid), "net.wlan.%s.debug", ifname);
#else
	snprintf(oid, sizeof(oid), "net.wlan.%s.debug", ifname+3);
#endif
	debuglen = sizeof(debug);
	if (sysctlbyname(oid, &debug, &debuglen, NULL, 0) < 0)
		err(1, "sysctl-get(%s)", oid);
	ndebug = debug;
	for (; argc > 1; argc--, argv++) {
		cp = argv[1];
		do {
			u_int bit;

			if (*cp == '-') {
				cp++;
				op = -1;
			} else if (*cp == '+') {
				cp++;
				op = 1;
			} else
				op = 0;
			for (tp = cp; *tp != '\0' && *tp != '+' && *tp != '-';)
				tp++;
			bit = getflag(cp, tp-cp);
			if (op < 0)
				ndebug &= ~bit;
			else if (op > 0)
				ndebug |= bit;
			else {
				if (bit == 0) {
					if (isdigit(*cp))
						bit = strtoul(cp, NULL, 0);
					else
						errx(1, "unknown flag %.*s",
							(int)(tp-cp), cp);
				}
				ndebug = bit;
			}
		} while (*(cp = tp) != '\0');
	}
	if (debug != ndebug) {
		printf("%s: 0x%x => ", oid, debug);
		if (sysctlbyname(oid, NULL, NULL, &ndebug, sizeof(ndebug)) < 0)
			err(1, "sysctl-set(%s)", oid);
		printf("0x%x", ndebug);
		debug = ndebug;
	} else
		printf("%s: 0x%x", oid, debug);
	sep = "<";
	for (i = 0; i < N(flags); i++)
		if (debug & flags[i].bit) {
			printf("%s%s", sep, flags[i].name);
			sep = ",";
		}
	printf("%s\n", *sep != '<' ? ">" : "");
	return 0;
}
