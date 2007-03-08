/*-
 * Copyright (c) 2003-2005 Sam Leffler, Errno Consulting
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
 *
 *	$Id$
 */
#ifndef _NET80211_IEEE80211_LINUX_H_
#define _NET80211_IEEE80211_LINUX_H_

/*
 * Beacon locking definitions.
 */
typedef spinlock_t ieee80211_beacon_lock_t;
#define	IEEE80211_BEACON_LOCK_INIT(_ic, _name) \
	spin_lock_init(&(_ic)->ic_beaconlock)
#define	IEEE80211_BEACON_LOCK_DESTROY(_ic) /* not necessary */
#define IEEE80211_BEACON_LOCK(_ic, flags)	spin_lock_irqsave(&(_ic)->ic_beaconlock, flags) 
#define IEEE80211_BEACON_UNLOCK(_ic, flags)	spin_unlock_irqrestore(&(_ic)->ic_beaconlock, flags) 
/* NB: beware, *_is_locked() are boguly defined for UP+!PREEMPT */
#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT)
#define	IEEE80211_BEACON_LOCK_ASSERT(_ic) \
	KASSERT(spin_is_locked(&(_ic)->ic_beaconlock), ("BEACONLOCK not locked!"))
#else
#define	IEEE80211_BEACON_LOCK_ASSERT(_ic)
#endif

/*
 * Node locking definitions.
 * TODO: should i use irqsave?
 */
typedef spinlock_t ieee80211_node_lock_t;
#define	IEEE80211_NODE_LOCK_INIT(_nt, _name) spin_lock_init(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_LOCK_DESTROY(_nt) /* not necessary */
#define	IEEE80211_NODE_LOCK(_nt)	spin_lock(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_UNLOCK(_nt)	spin_unlock(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_LOCK_BH(_nt)	spin_lock_bh(&(_nt)->nt_nodelock)
#define	IEEE80211_NODE_UNLOCK_BH(_nt)	spin_unlock_bh(&(_nt)->nt_nodelock)
/* NB: beware, *_is_locked() are boguly defined for UP+!PREEMPT */
#if (defined(CONFIG_SMP) || defined(CONFIG_PREEMPT)) && defined(spinlock_is_locked)
#define IEEE80211_NODE_LOCK_ASSERT(_nt) \
        KASSERT(spinlock_is_locked(&(_nt)->nt_nodelock), \
                ("802.11 node not locked!"))
#else
#define IEEE80211_NODE_LOCK_ASSERT(_nt)
#endif

/*
 * Node table scangen locking definitions.
 */
typedef spinlock_t ieee80211_scan_lock_t;
#define	IEEE80211_SCAN_LOCK_INIT(_nt, _name) \
	spin_lock_init(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_LOCK_DESTROY(_nt)	/* not necessary */
#define	IEEE80211_SCAN_LOCK(_nt)		spin_lock(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_UNLOCK(_nt)		spin_unlock(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_LOCK_BH(_nt)		spin_lock_bh(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_UNLOCK_BH(_nt)		spin_unlock_bh(&(_nt)->nt_scanlock)
#define	IEEE80211_SCAN_LOCK_ASSERT(_nt)

/*
 * 802.1x state locking definitions.
 */
typedef spinlock_t eapol_lock_t;
#define	EAPOL_LOCK_INIT(_ec, _name)	spin_lock_init(&(_ec)->ec_lock)
#define	EAPOL_LOCK_DESTROY(_ec)		/* not necessary */
#define	EAPOL_LOCK(_ec)			spin_lock_bh(&(_ec)->ec_lock)
#define	EAPOL_UNLOCK(_ec)		spin_unlock_bh(&(_ec)->ec_lock)
/* NB: beware, *_is_locked() are boguly defined for UP+!PREEMPT */
#if (defined(CONFIG_SMP) || defined(CONFIG_PREEMPT))
#define	EAPOL_LOCK_ASSERT(_ec) \
	KASSERT(spin_is_locked(&(_ec)->ec_lock), ("EAPOL not locked!"))
#else
#define	EAPOL_LOCK_ASSERT(_ec)
#endif

/*
 * Per-node power-save queue definitions.
 */
#define IEEE80211_NODE_SAVEQ_INIT(_ni, _name) \
	spin_lock_init(&(_ni)->ni_savedq.lock)

#define IEEE80211_NODE_SAVEQ_LOCK(_ni) \
	spin_lock_bh(&(_ni)->ni_savedq.lock)

#define IEEE80211_NODE_SAVEQ_UNLOCK(_ni) \
	spin_unlock_bh(&(_ni)->ni_savedq.lock)

#define IEEE80211_NODE_SAVEQ_DESTROY(_ni) \
	/* not necessary */

//TODO: is sometimes called without a lock, hope this is o.k.
#define IEEE80211_NODE_SAVEQ_QLEN(_ni) \
	skb_queue_len(&(_ni)->ni_savedq)

#define IEEE80211_NODE_SAVEQ_DEQUEUE(_ni, _skb, _qlen) do {	\
	IEEE80211_NODE_SAVEQ_LOCK(_ni);				\
	(_skb) = __skb_dequeue(&(_ni)->ni_savedq);		\
	(_qlen) = IEEE80211_NODE_SAVEQ_QLEN(_ni);		\
	IEEE80211_NODE_SAVEQ_UNLOCK(_ni);			\
} while (0)

#define IEEE80211_NODE_SAVEQ_DRAIN(_ni, _qlen) do {		\
	IEEE80211_NODE_SAVEQ_LOCK(_ni);				\
	(_qlen) = IEEE80211_NODE_SAVEQ_QLEN(_ni);		\
	__skb_queue_purge(&(_ni)->ni_savedq);			\
	IEEE80211_NODE_SAVEQ_UNLOCK(_ni);			\
} while (0)

#define _IEEE80211_NODE_SAVEQ_DEQUEUE_HEAD(_ni, _skb) \
	(_skb) = __skb_dequeue(&(_ni)->ni_savedq);

//TODO: check regarding age
#define _IEEE80211_NODE_SAVEQ_ENQUEUE(_ni, _skb, _qlen, _age) do {	\
	struct sk_buff *skb0;						\
	if ((skb0 = skb_peek_tail(&(_ni)->ni_savedq)) != NULL)		\
		(_age) -= M_AGE_GET(skb0);				\
	__skb_queue_tail(&(_ni)->ni_savedq, (_skb));			\
	M_AGE_SET((_skb), (_age));					\
	(_qlen) = IEEE80211_NODE_SAVEQ_QLEN(_ni);			\
} while (0)

/*
 * 802.1x MAC ACL database locking definitions.
 */
typedef spinlock_t acl_lock_t;
#define	ACL_LOCK_INIT(_as, _name)	spin_lock_init(&(_as)->as_lock)
#define	ACL_LOCK_DESTROY(_as)		/* not necessary */
#define	ACL_LOCK(_as)			spin_lock_bh(&(_as)->as_lock)
#define	ACL_UNLOCK(_as)			spin_unlock_bh(&(_as)->as_lock)
/* NB: beware, *_is_locked() are boguly defined for UP+!PREEMPT */
#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT)
#define	ACL_LOCK_ASSERT(_as) \
	KASSERT(spin_is_locked(&(_as)->as_lock), ("ACL not locked!"))
#else
#define	ACL_LOCK_ASSERT(_as)
#endif

/*
 * Node reference counting definitions.
 *
 * ieee80211_node_initref	initialize the reference count to 1
 * ieee80211_node_incref	add a reference
 * ieee80211_node_decref	remove a reference
 * ieee80211_node_dectestref	remove a reference and return 1 if this
 *				is the last reference, otherwise 0
 * ieee80211_node_refcnt	reference count for printing (only)
 */
#define ieee80211_node_initref(_ni) \
	atomic_set(&(_ni)->ni_refcnt, 1)
#define ieee80211_node_incref(_ni) \
	atomic_inc(&(_ni)->ni_refcnt)
#define	ieee80211_node_decref(_ni) \
	atomic_dec(&(_ni)->ni_refcnt)
#define	ieee80211_node_dectestref(_ni) \
	atomic_dec_and_test(&(_ni)->ni_refcnt)
#define	ieee80211_node_refcnt(_ni)	(_ni)->ni_refcnt.counter

/*
 * control buffer flags.
 */
#define M_EXT           0x0001  /* has associated external storage */
#define M_PKTHDR        0x0002  /* start of record */
#define M_EOR           0x0004  /* end of record */
#define M_RDONLY        0x0008  /* associated data is marked read-only */
#define M_PROTO1        0x0010  /* protocol-specific */
#define M_PROTO2        0x0020  /* protocol-specific */
#define M_PROTO3        0x0040  /* protocol-specific */
#define M_PROTO4        0x0080  /* protocol-specific */
#define M_PROTO5        0x0100  /* protocol-specific */
#define M_PROTO6        0x4000  /* protocol-specific (avoid M_BCAST conflict) */
#define M_FREELIST      0x8000  /* mbuf is on the free list */

/*
 * pkthdr flags (also stored in cb).
 */
#define M_BCAST         0x0200  /* send/received as link-level broadcast */
#define M_MCAST         0x0400  /* send/received as link-level multicast */
#define M_FRAG          0x0800  /* packet is a fragment of a larger packet */
#define M_FIRSTFRAG     0x1000  /* packet is first fragment */
#define M_LASTFRAG      0x2000  /* packet is last fragment */

#define	M_LINK0		M_PROTO1		/* WEP requested */
#define	M_PWR_SAV	M_PROTO4		/* bypass PS handling */
#define	M_HASFCS	M_PROTO5		/* WEP included? */
#define	M_MORE_DATA	M_PROTO6		/* more data frames to follow */

/*
 * Encode WME access control bits in the PROTO flags.
 * This is safe since it's passed directly in to the
 * driver and there's no chance someone else will clobber
 * them on us.
 */
#define	M_WME_AC_MASK	(M_PROTO2|M_PROTO3)
/* XXX 5 is wrong if M_PROTO* are redefined */
#define	M_WME_AC_SHIFT	5

#define	M_WME_SETAC(skb, ac) \
	(((struct ieee80211_cb *)skb->cb)->flags = \
		(((struct ieee80211_cb *)skb->cb)->flags &~ M_WME_AC_MASK) | \
		((ac) << M_WME_AC_SHIFT))
#define	M_WME_GETAC(skb) \
	((((struct ieee80211_cb *)skb->cb)->flags >> M_WME_AC_SHIFT) & 0x3)

/*
 * Mbufs on the power save queue are tagged with an age and
 * timed out.  We reuse the hardware checksum field in the
 * mbuf packet header to store this data.
 */
#define M_AGE_SET(skb,v)          (((struct ieee80211_cb *)skb->cb)->age = v)
#define M_AGE_GET(skb)            (((struct ieee80211_cb *)skb->cb)->age)
#define M_AGE_SUB(skb,adj)        (((struct ieee80211_cb *)skb->cb)->age -= adj)


#define	le16toh(_x)	le16_to_cpu(_x)
#define	htole16(_x)	cpu_to_le16(_x)
#define	le32toh(_x)	le32_to_cpu(_x)
#define	htole32(_x)	cpu_to_le32(_x)
#define	be32toh(_x)	be32_to_cpu(_x)
#define	htobe32(_x)	cpu_to_be32(_x)
#define	le64toh(_x)	le64_to_cpu(_x)
#define	htole64(_x)	cpu_to_le64(_x)

#define uintptr_t	unsigned long

/*
 * Linux has no equivalents to malloc types so null these out.
 */
#define	MALLOC_DEFINE(type, shortdesc, longdesc)
#define	MALLOC_DECLARE(type)

/*
 * flags to malloc.
 */
#define	M_NOWAIT	0x0001		/* do not block */
#define	M_WAITOK	0x0002		/* ok to block */
#define	M_ZERO		0x0100		/* bzero the allocation */

static inline void *
ieee80211_malloc(size_t size, int flags)
{
	void *p = kmalloc(size, flags & M_NOWAIT ? GFP_ATOMIC : GFP_KERNEL);
	if (p && (flags & M_ZERO))
		memset(p, 0, size);
	return p;
}
#define	MALLOC(_ptr, cast, _size, _type, _flags) \
	((_ptr) = (cast)ieee80211_malloc(_size, _flags))
#define	FREE(addr, type)	kfree((addr))

/*
 * This unlikely to be popular but it dramatically reduces diffs.
 */
#define printf(...) printk(__VA_ARGS__)
struct ieee80211com;
extern	void if_printf(struct net_device *, const char *, ...);
extern	const char *ether_sprintf(const u_int8_t *);

/*
 * Queue write-arounds and support routines.
 */
extern	struct sk_buff *ieee80211_getmgtframe(u_int8_t **frm, u_int pktlen);

#define	IF_ENQUEUE(_q,_skb)	skb_queue_tail(_q,_skb)
#define	IF_DEQUEUE(_q,_skb)	(_skb = skb_dequeue(_q))
#define	IF_DRAIN(_q)		skb_queue_purge(_q)	// with lock
#define _IF_QFULL(_q)		(skb_queue_len(_q) >= IEEE80211_PS_MAX_QUEUE)
#define IF_POLL(_q, _skb)	((_skb) = skb_peek(_q))

extern	void skb_queue_drain(struct sk_buff_head *q);

extern	struct net_device_stats *ieee80211_getstats(struct net_device *);

#ifndef __MOD_INC_USE_COUNT
#define	_MOD_INC_USE(_m, _err)						\
	if (!try_module_get(_m)) {					\
		printk(KERN_WARNING "%s: try_module_get failed\n",	\
			__func__); \
		_err;							\
	}
#define	_MOD_DEC_USE(_m)	module_put(_m)
#else
#define	_MOD_INC_USE(_m, _err)	MOD_INC_USE_COUNT
#define	_MOD_DEC_USE(_m)	MOD_DEC_USE_COUNT
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
static inline u_int64_t
get_jiffies_64(void)
{
	return (u_int64_t) jiffies;		/* XXX not right */
}
#endif

#ifndef CLONE_KERNEL
/*
 * List of flags we want to share for kernel threads,
 * if only because they are not used by them anyway.
 */
#define CLONE_KERNEL	(CLONE_FS | CLONE_FILES | CLONE_SIGHAND)
#endif

#include <linux/mm.h>
#ifndef offset_in_page
#define	offset_in_page(p) ((unsigned long) (p) & ~PAGE_MASK)
#endif

#ifndef module_put_and_exit
#define module_put_and_exit(code) do {	\
	_MOD_DEC_USE(THIS_MODULE);	\
	do_exit(code);			\
} while (0)
#endif

/*
 * Linux uses __BIG_ENDIAN and __LITTLE_ENDIAN while BSD uses _foo
 * and an explicit _BYTE_ORDER.  Sorry, BSD got there first--define
 * things in the BSD way...
 */
#undef _LITTLE_ENDIAN
#define	_LITTLE_ENDIAN	1234	/* LSB first: i386, vax */
#undef _BIG_ENDIAN
#define	_BIG_ENDIAN	4321	/* MSB first: 68000, ibm, net */
#include <asm/byteorder.h>
#if defined(__LITTLE_ENDIAN)
#define	_BYTE_ORDER	_LITTLE_ENDIAN
#elif defined(__BIG_ENDIAN)
#define	_BYTE_ORDER	_BIG_ENDIAN
#else
#error "Please fix asm/byteorder.h"
#endif


/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define	IEEE80211_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, struct file *filp, void *buffer, \
		size_t *lenp)
#define	IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, filp, buffer, lenp)
#else
#define	IEEE80211_SYSCTL_DECL(f, ctl, write, filp, buffer, lenp, ppos) \
	f(ctl_table *ctl, int write, struct file *filp, void __user *buffer,\
		size_t *lenp, loff_t *ppos)
#define	IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos) \
	proc_dointvec(ctl, write, filp, buffer, lenp, ppos)
#endif

extern	void ieee80211_sysctl_register(struct ieee80211com *);
extern	void ieee80211_sysctl_unregister(struct ieee80211com *);

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define IEEE80211_VLAN_TAG_USED 1

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20)
#define	vlan_hwaccel_receive_skb(skb, grp, tag)	vlan_hwaccel_rx(skb, grp, tag)
#endif

#ifndef VLAN_GROUP_ARRAY_PART_LEN
#define vlan_group_set_device(group, vid, dev) do { \
	group->vlan_devices[vid] = dev; \
} while (0);
#endif

extern	void ieee80211_vlan_register(struct ieee80211com *, struct vlan_group*);
extern	void ieee80211_vlan_kill_vid(struct ieee80211com *, unsigned short);
#else
#define IEEE80211_VLAN_TAG_USED 0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define	free_netdev(dev)	kfree(dev)
#endif

struct iw_statistics;
extern	void ieee80211_iw_getstats(struct ieee80211com*, struct iw_statistics*);
struct iw_request_info;
struct iw_point;
extern	int ieee80211_ioctl_giwname(struct ieee80211com *,
		struct iw_request_info *, char *name, char *extra);
extern	int ieee80211_ioctl_siwencode(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_giwencode(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *key);
struct iw_param;
extern	int ieee80211_ioctl_siwrate(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwrate(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_siwsens(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwsens(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_siwrts(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwrts(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_siwfrag(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwfrag(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
struct sockaddr;
extern	int ieee80211_ioctl_siwap(struct ieee80211com *,
		struct iw_request_info *, struct sockaddr *, char *);
extern	int ieee80211_ioctl_giwap(struct ieee80211com *,
		struct iw_request_info *, struct sockaddr *, char *);
extern	int ieee80211_ioctl_siwnickn(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_giwnickn(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
struct iw_freq;
extern	int ieee80211_ioctl_siwfreqx(struct ieee80211com *,
		struct iw_request_info *, struct iw_freq *, char *);
extern	int ieee80211_ioctl_giwfreq(struct ieee80211com *,
		struct iw_request_info *, struct iw_freq *, char *);
extern	int ieee80211_ioctl_siwessid(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_giwessid(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_giwrange(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_siwmode(struct ieee80211com *,
		struct iw_request_info *, __u32 *, char *);
extern	int ieee80211_ioctl_giwmode(struct ieee80211com *,
		struct iw_request_info *, __u32 *, char *);
extern	int ieee80211_ioctl_siwpower(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwpower(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_siwretry(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwretry(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_siwtxpow(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_giwtxpow(struct ieee80211com *,
		struct iw_request_info *, struct iw_param *, char *);
extern	int ieee80211_ioctl_iwaplist(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_siwscan(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);
extern	int ieee80211_ioctl_giwscan(struct ieee80211com *,
		struct iw_request_info *, struct iw_point *, char *);

extern	int ieee80211_ioctl_setparam(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_getparam(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_setoptie(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_getoptie(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_setkey(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_delkey(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_setmlme(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_addmac(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_delmac(struct ieee80211com *,
		struct iw_request_info *, void *, char *);
extern	int ieee80211_ioctl_chanlist(struct ieee80211com *,
		struct iw_request_info *, void *, char *);

extern	void ieee80211_ioctl_iwsetup(struct iw_handler_def *);

#endif /* _NET80211_IEEE80211_LINUX_H_ */
