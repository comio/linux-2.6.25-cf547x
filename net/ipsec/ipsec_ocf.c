/*
 * IPSEC OCF support
 *
 * This code written by David McCullough <dmccullough@cyberguard.com>
 * Copyright (C) 2005 Intel Corporation.  All Rights Reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.  See <http://www.fsf.org/copyleft/gpl.txt>.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>

#define __NO_VERSION__
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */

#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/skbuff.h>

#include <linux/interrupt.h>

#include <net/ip.h>

#include <openswan.h>
#include "openswan/ipsec_sa.h"
#include "openswan/ipsec_rcv.h"
#include "openswan/ipsec_xmit.h"
#include "openswan/ipsec_tunnel.h"
#include "openswan/ipsec_xform.h"
#include "openswan/ipsec_auth.h"
#include "openswan/ipsec_esp.h"
#include "openswan/ipsec_ah.h"
#include "openswan/ipcomp.h"
#include "openswan/ipsec_proto.h"

#include <openswan/pfkeyv2.h>
#include <openswan/pfkey.h>

#include "ipsec_ocf.h"

extern int debug_ocf;
extern int debug_pfkey;
extern int debug_rcv;

static int ocf_cryptodev_selection = CRYPTO_ANYDEVICE;
module_param(ocf_cryptodev_selection, int, 0644);
MODULE_PARM_DESC(ocf_cryptodev_selection, 
                "select OCF driver (anydevice -1, anyhardware -2, "
                "anysoftware -3, 0 and up select specific drivers in OCF");

#define USE_OCF_FORCE_KNOBS
// these are intended to be temporary and for testing [bart]
#ifdef USE_OCF_FORCE_KNOBS
static int ocf_force_ipcomp_cryptodev = CRYPTO_ANYDEVICE;
module_param(ocf_force_ipcomp_cryptodev, int, 0644);
MODULE_PARM_DESC(ocf_force_ipcomp_cryptodev, "OCF device to use for ipcomp (-1 disable)");
static int ocf_force_ipcomp_alg = 0;
module_param(ocf_force_ipcomp_alg, int, 0644);
MODULE_PARM_DESC(ocf_force_ipcomp_alg, "OCF alg to use for ipcomp (0 disable)");
#endif

/*static*/ int ocf_rcv_calls=0;
/*static*/ int ocf_rcv_calls_failed=0;
/*static*/ int ocf_rcv_cb=0;
/*static*/ int ocf_xmit_cb=0;
uint ocf_xmit_calls=0;
uint ocf_xmit_calls_failed=0;
module_param(ocf_rcv_calls, int, 0444);
module_param(ocf_rcv_calls_failed, int, 0444);
module_param(ocf_rcv_cb,    int, 0444);
module_param(ocf_xmit_cb,   int, 0444);
module_param(ocf_xmit_calls, int, 0444);
module_param(ocf_xmit_calls_failed, int, 0444);

/*
 * Tuning parameters,  the settings below appear best for
 * the IXP
 */
#define USE_BATCH 0	/* enable batch mode */
#define USE_CBIMM 1	/* enable immediate callbacks */
#define FORCE_QS  0	/* force use of queues for continuation of state machine */

/*
 * Because some OCF operations are synchronous (ie., software encryption)
 * we need to protect ourselves from distructive re-entry.  All we do
 * is track where we are at and either callback  immediately or Q the
 * callback to avoid conflicts.  This allows us to deal with the fact that
 * OCF doesn't tell us if our crypto operations will be async or sync.
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#define PROCESS_LATER(wq, sm) \
	({ \
		INIT_WORK(&(wq), (void (*)(struct work_struct *))(sm)); \
		schedule_work(&(wq)); \
	})
#else
#define PROCESS_LATER(wq, sm, arg) \
	({ \
		INIT_WORK(&(wq), (void (*)(void *))(sm), (void *)(arg)); \
		schedule_work(&(wq)); \
	})
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#define PROCESS_NOW(wq, sm) \
	({ \
		(*sm)(&(wq)); \
	})
#else
#define PROCESS_NOW(wq, sm, arg) \
	({ \
		(*sm)(arg); \
	})
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#if FORCE_QS == 0
	#define PROCESS_NEXT(wq, sm) \
		if (in_interrupt()) { \
			PROCESS_LATER(wq, sm); \
		} else { \
			PROCESS_NOW(wq, sm); \
		}
#else
	#define PROCESS_NEXT(wq, sm) PROCESS_LATER(wq, sm)
#endif
#else
#if FORCE_QS == 0
	#define PROCESS_NEXT(wq, sm, arg) \
		if (in_interrupt()) { \
			PROCESS_LATER(wq, sm, arg); \
		} else { \
			PROCESS_NOW(wq, sm, arg); \
		}
#else
	#define PROCESS_NEXT(wq, sm, arg) PROCESS_LATER(wq, sm, arg)
#endif
#endif

/*
 * convert openswan values to OCF values
 */

static int
ipsec_ocf_compalg(int compalg)
{
	switch (compalg) {
        case IPCOMP_DEFLATE:    return CRYPTO_DEFLATE_COMP;
        case IPCOMP_LZS:        return CRYPTO_LZS_COMP;

        // ocf does not have these yet...

        //case IPCOMP_OUI:        
        //case IPCOMP_V42BIS:
	}
	return 0;
}


static int
ipsec_ocf_authalg(int authalg)
{
	switch (authalg) {
	case AH_SHA:  return CRYPTO_SHA1_HMAC;
	case AH_MD5:  return CRYPTO_MD5_HMAC;
	}
	return 0;
}


static int
ipsec_ocf_encalg(int encalg)
{
	switch (encalg) {
	case ESP_NULL:      return CRYPTO_NULL_CBC;
	case ESP_DES:       return CRYPTO_DES_CBC;
	case ESP_3DES:      return CRYPTO_3DES_CBC;
	case ESP_AES:       return CRYPTO_AES_CBC;
	case ESP_CAST:      return CRYPTO_CAST_CBC;
	case ESP_BLOWFISH:  return CRYPTO_BLF_CBC;
	}
	return 0;
}

/* 
 * We use this function because sometimes we want to pass a negative offset
 * into skb_put(), this does not work on 64bit platforms because long to
 * unsigned int casting.
 */
static inline unsigned char *
safe_skb_put(struct sk_buff *skb, int extend)
{
        unsigned char *ptr;

        if (extend>0) {
                // increase the size of the packet
                ptr = skb_put(skb, extend);
        } else {
                // shrink the size of the packet
                ptr = skb->tail;
                skb_trim (skb, skb->len + extend);
        }

        return ptr;
}

/*
 * We need to grow the skb to accommodate the expanssion of the ipcomp packet.
 *
 * The following comment comes from the skb_decompress() which does the
 * same...
 *
 * We have no way of knowing the exact length of the resulting
 * decompressed output before we have actually done the decompression.
 * For now, we guess that the packet will not be bigger than the
 * attached ipsec device's mtu or 16260, whichever is biggest.
 * This may be wrong, since the sender's mtu may be bigger yet.
 * XXX This must be dealt with later XXX
 */
static int
ipsec_ocf_ipcomp_copy_expand (struct ipsec_rcv_state *irs)
{
        struct sk_buff *nskb;
        unsigned grow_to, grow_by;
        ptrdiff_t ptr_delta;

        if (!irs->skb)
                return IPSEC_RCV_IPCOMPFAILED;

        grow_to = irs->skb->dev 
                ? (irs->skb->dev->mtu < 16260 
                                ? 16260 
                                : irs->skb->dev->mtu)
                : (65520 - ntohs(irs->ipp->tot_len));
        grow_by = grow_to - irs->skb->len;
        grow_by -= skb_headroom(irs->skb);
        grow_by -= skb_tailroom(irs->skb);

        // it's big enouhg
        if (! grow_by)
                return IPSEC_RCV_OK;

        nskb = skb_copy_expand (irs->skb, 
                        skb_headroom(irs->skb),
                        skb_tailroom(irs->skb) + grow_by,
                        GFP_ATOMIC);
        if (! nskb)
                return IPSEC_RCV_ERRMEMALLOC;

        memcpy (nskb->head, irs->skb->head, skb_headroom(irs->skb));

        nskb->network_header = nskb->data + skb_network_offset(irs->skb);
        nskb->transport_header  = nskb->data + (irs->skb->transport_header - irs->skb->data);

        // update all irs pointers
        ptr_delta = nskb->data - irs->skb->data;
        irs->authenticator = (void*)((char*)irs->authenticator + ptr_delta);
        irs->ipp           = (void*)((char*)irs->ipp           + ptr_delta);

        // flip in the large one
        irs->pre_ipcomp_skb = irs->skb;
        irs->skb = nskb;

        // move the tail up to the end to let OCF know how big the buffer is
        if (grow_by > (irs->skb->end - irs->skb->tail))
                grow_by = irs->skb->end - irs->skb->tail;
        skb_put (irs->skb, grow_by);

        return IPSEC_RCV_OK;
}


/*
 * if we can do the request ops, setup the sessions and return 0
 * otherwise return error with ipsp unchanged
 */

int
ipsec_ocf_sa_init(struct ipsec_sa *ipsp, int authalg, int encalg)
{
	struct cryptoini crie, cria;
        char devicename[CRYPTO_NAME_LEN];
	int error;

	KLIPS_PRINT(debug_ocf, "klips_debug:ipsec_ocf_sa_init(a=0x%x,e=0x%x)\n",
			authalg, encalg);

	if (authalg && ipsp->ips_key_bits_a == 0) {
		KLIPS_PRINT(debug_ocf,
				"klips_debug:ipsec_ocf_sa_init(a=0x%x,e=0x%x) a-key-bits=0\n",
				authalg, encalg);
		/* pretend we are happy with this */
		return 0;
	}

	if (encalg && ipsp->ips_key_bits_e == 0) {
		KLIPS_PRINT(debug_ocf,
				"klips_debug:ipsec_ocf_sa_init(a=0x%x,e=0x%x) e-key-bits=0\n",
				authalg, encalg);
		/* pretend we are happy with this */
		return 0;
	}

	memset(&crie, 0, sizeof(crie));
	memset(&cria, 0, sizeof(cria));

	cria.cri_alg = ipsec_ocf_authalg(authalg);
	cria.cri_klen = ipsp->ips_key_bits_a;
	cria.cri_key  = ipsp->ips_key_a;
	/* ipsec_dmp_block("ocf auth key", cria.cri_key, cria.cri_klen/8); */

	crie.cri_alg = ipsec_ocf_encalg(encalg);
	crie.cri_klen = ipsp->ips_key_bits_e;
	crie.cri_key  = ipsp->ips_key_e;
	/* ipsec_dmp_block("ocf cipher key", crie.cri_key, crie.cri_klen/8); */

	switch (crie.cri_alg) {
	case CRYPTO_AES_CBC:
		ipsp->ips_iv_size = 16;
		break;
	case CRYPTO_DES_CBC:
	case CRYPTO_3DES_CBC:
		ipsp->ips_iv_size = 8;
		break;
	default:
		ipsp->ips_iv_size = 0;
		break;
	}
	ipsp->ips_iv_bits = ipsp->ips_iv_size * 8;
	ipsp->ips_auth_bits = ipsp->ips_key_bits_a;

        // automatically fix stupid mistakes
        if (ocf_cryptodev_selection < CRYPTODEV_SELECTION_MIN)
                ocf_cryptodev_selection = CRYPTO_ANYDEVICE;

	if (authalg && encalg) {
		crie.cri_next = &cria;
		error = crypto_newsession(&ipsp->ocf_cryptoid, &crie,
                                ocf_cryptodev_selection);
	} else if (encalg) {
		error = crypto_newsession(&ipsp->ocf_cryptoid, &crie,
                                ocf_cryptodev_selection);
	} else if (authalg) {
		error = crypto_newsession(&ipsp->ocf_cryptoid, &cria,
                                ocf_cryptodev_selection);
	} else {
		KLIPS_ERROR(debug_ocf, "klips_debug:ipsec_ocf_sa_init: "
				"no authalg or encalg\n");
		return -ENOENT;
	}

	if (error) {
		KLIPS_ERROR(debug_ocf, "klips_debug:ipsec_ocf_sa_init: "
				"crypto_newsession failed 0x%x\n", error);
		return error;
	}

        // get the name of the device we are using from OCF
        memset (devicename, 0, CRYPTO_NAME_LEN);
        crypto_get_sess_devicename(ipsp->ocf_cryptoid, devicename);

        // make sure we don't print out too much garbage if the interface is
        // borken
        devicename[CRYPTO_NAME_LEN-1] = 0;
        KLIPS_PRINT (debug_ocf, "klips_debug:ipsec_ocf_sa_init: "
                        "id=0x%016llx device='%s'\n",
                        ipsp->ocf_cryptoid, devicename);

	/* make sure no ALG stuff bites us */
	if (ipsp->ips_alg_enc)
		printk("We received an ALG initted SA\n");
	ipsp->ips_alg_enc = NULL;

	ipsp->ocf_in_use = 1;
	return 0;
}

/* this function returns a zero upon success */
int 
ipsec_ocf_comp_sa_init(struct ipsec_sa *ipsp, int compalg)
{
	struct cryptoini cric;
        char devicename[CRYPTO_NAME_LEN];
	int error;
        int cryptodevid;


#ifdef USE_OCF_FORCE_KNOBS
        // remove when we have working hardware support for crypto offload
        if (ocf_force_ipcomp_alg) {
                printk ("klips:ocf: forcing ipcomp offload of alg=%d to alg id=%d\n", 
                                compalg, ocf_force_ipcomp_alg);

                compalg = ocf_force_ipcomp_alg;
        }
#endif

	KLIPS_PRINT(debug_ocf, "klips_debug:ipsec_ocf_comp_sa_init(c=0x%x)\n",
			compalg);

	memset(&cric, 0, sizeof(cric));

	cric.cri_alg = ipsec_ocf_compalg(compalg);

        if (! cric.cri_alg) {
		KLIPS_ERROR(debug_ocf, "klips_debug:ipsec_ocf_comp_sa_init: "
				"invalid compalg=%d given\n", compalg);
		return -ENOENT;
        }

        cryptodevid = ocf_cryptodev_selection;

#ifdef USE_OCF_FORCE_KNOBS
        // remove when we have working hardware support for crypto offload
        if (ocf_force_ipcomp_cryptodev != CRYPTO_ANYDEVICE) {
                printk ("klips:ocf: forcing ipcomp offload of alg=%d to crypto id=%d\n", 
                                compalg, ocf_force_ipcomp_cryptodev);

                cryptodevid = ocf_force_ipcomp_cryptodev;
        }
#endif

        // automatically fix stupid mistakes
        if (cryptodevid < CRYPTODEV_SELECTION_MIN)
                cryptodevid = CRYPTO_ANYDEVICE;

        error = crypto_newsession(&ipsp->ocf_cryptoid, &cric,
                        cryptodevid);
	if (error) {
		KLIPS_ERROR(debug_ocf, "klips_debug:ipsec_ocf_comp_sa_init: "
				"crypto_newsession failed 0x%x\n", error);
		return -ENOENT;
	}

        // get the name of the device we are using from OCF
        memset (devicename, 0, CRYPTO_NAME_LEN);
        crypto_get_sess_devicename(ipsp->ocf_cryptoid, devicename);

        // make sure we don't print out too much garbage if the interface is
        // borken
        devicename[CRYPTO_NAME_LEN-1] = 0;
        KLIPS_PRINT (debug_ocf, "klips_debug:ipsec_ocf_comp_sa_init: "
                        "id=0x%016llx device='%s'\n",
                        ipsp->ocf_cryptoid, devicename);

	ipsp->ocf_in_use = 1;
	return 0;
}


int
ipsec_ocf_sa_free(struct ipsec_sa *ipsp)
{
	KLIPS_PRINT(debug_ocf, "klips_debug:ipsec_ocf_sa_free()\n");
	crypto_freesession(ipsp->ocf_cryptoid);
	ipsp->ocf_cryptoid = -1;
	ipsp->ocf_in_use = 0;
	return 1;
}


static int
ipsec_ocf_rcv_cb(struct cryptop *crp)
{
	struct ipsec_rcv_state *irs = (struct ipsec_rcv_state *)crp->crp_opaque;
        struct iphdr *newiph;
        unsigned orig_len, decomp_len;
	struct cryptodesc *crdc=NULL;

	KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv_cb\n");

	ocf_rcv_cb++;

	if (irs == NULL) {
		KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv_cb: "
				"NULL irs in callback\n");
		return 0;
	}

	/*
	 * we must update the state before returning to the state machine.
	 * if we have an error,  terminate the processing by moving to the DONE
	 * state
	 */

        /* by default we will finish processing */
	irs->state = IPSEC_RSM_DONE;

	if (crp->crp_etype) {
                ptrdiff_t ptr_delta;

		KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv_cb: "
				"error in processing 0x%x\n", crp->crp_etype);

                switch(irs->ipsp->ips_said.proto) {
                case IPPROTO_COMP:
                        // need to restore the previous skb
                        ptr_delta = irs->pre_ipcomp_skb->data - irs->skb->data;
                        irs->authenticator = (void*)((char*)irs->authenticator + ptr_delta);
                        irs->ipp           = (void*)((char*)irs->ipp           + ptr_delta);

                        kfree_skb (irs->skb);
                        irs->skb = irs->pre_ipcomp_skb;
                        irs->pre_ipcomp_skb = NULL;

                        // bart - TODO: we should really be failing here, but
                        // ipsec_rsm() gets confused
                        //irs->state = IPSEC_RCV_IPCOMPFAILED;
                        break;

                default:
                        break;
                }

                goto bail;
	}
        
	switch(irs->ipsp->ips_said.proto) {
        case IPPROTO_ESP:
                /* ESP, process it */
		if (ipsec_rcv_esp_post_decrypt(irs) == IPSEC_RCV_OK) {
			/* this one came up good, set next state */
			irs->state = IPSEC_RSM_DECAP_CONT;
                }
                break;

	case IPPROTO_AH:
                /* AH post processing, put back fields we had to zero */
                irs->ipp->ttl      = irs->ttl;
                irs->ipp->check    = irs->check;
                irs->ipp->frag_off = irs->frag_off;
                irs->ipp->tos      = irs->tos;
                irs->state         = IPSEC_RSM_AUTH_CHK;

                /* pull up the IP header again after processing */
                skb_pull(irs->skb, ((unsigned char *)irs->protostuff.ahstuff.ahp) -
                                ((unsigned char *)irs->ipp));

                break;

        case IPPROTO_COMP:
		crdc = crp->crp_desc;

                kfree_skb (irs->pre_ipcomp_skb);
                irs->pre_ipcomp_skb = NULL;

		KLIPS_PRINT(debug_rcv, "comp before adjustments:");
		KLIPS_IP_PRINT(debug_rcv & DB_TN_XMIT, irs->ipp);

                orig_len = irs->skb->len - sizeof (struct ipcomphdr);
                decomp_len = crp->crp_olen - crdc->crd_inject;

                newiph = (struct iphdr*)((char*)irs->ipp + sizeof (struct ipcomphdr));

		KLIPS_PRINT(debug_rcv, "comp results: olen: %u, inject: %u (len=%d) iph->totlen=%u\n",
			    crp->crp_olen, crdc->crd_inject, decomp_len, ntohs(newiph->tot_len));

                // move the ip header to consume room previously taken by
                // the ipcomp header
                skb_pull (irs->skb, sizeof (struct ipcomphdr));
                memmove (newiph, irs->ipp, irs->iphlen);
                irs->ipp = newiph;

                irs->skb->network_header += sizeof (struct ipcomphdr);
                irs->skb->transport_header += sizeof (struct ipcomphdr);

                // adjust the ipp pointer to point to the header we decoded
                //irs->ipp = (void*)((char*)irs->ipp - irs->iphlen;

                irs->ipp->protocol = irs->next_header;
                irs->ipp->tot_len = htons (irs->iphlen + decomp_len);
                irs->ipp->check = 0;
                irs->ipp->check = ip_fast_csum((char *) irs->ipp, irs->ipp->ihl);

		KLIPS_PRINT(debug_rcv, "comp after len adjustments:");
		KLIPS_IP_PRINT(debug_rcv & DB_TN_XMIT, irs->ipp);
 
		/* Update skb length/tail by "putting" the growth */
                safe_skb_put (irs->skb, decomp_len - orig_len);

                // set the new header in the skb
                irs->skb->network_header = (sk_buff_data_t)irs->ipp;

                // relese the backup copy
                if (irs->pre_ipcomp_skb) {
                        kfree_skb (irs->pre_ipcomp_skb);
                        irs->pre_ipcomp_skb = NULL;
                }

                /* IPcomp finished, continue processing */
                irs->state = IPSEC_RSM_DECAP_CONT;
                break;
        }

bail:
	crypto_freereq(crp);
	crp = NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	/* setup the rest of the processing now */
	PROCESS_NOW(irs->workq, ipsec_rsm);
#else
	/* setup the rest of the processing now */
	PROCESS_NOW(irs->workq, ipsec_rsm, irs);
#endif
	return 0;
}

enum ipsec_rcv_value
ipsec_ocf_rcv(struct ipsec_rcv_state *irs)
{
	struct cryptop *crp;
	struct cryptodesc *crde=NULL, *crda=NULL, *crdc=NULL;
	struct ipsec_sa *ipsp;
        int req_count, rc;
	int err;

	KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv\n");

	ipsp = irs->ipsp;
	if (!ipsp) {
		KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: "
				"no SA for rcv processing\n");
		return IPSEC_RCV_SAIDNOTFOUND;
	}

	if (!irs->skb) {
		KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: no skb\n");
		return IPSEC_RCV_SAIDNOTFOUND;
	}

        switch (ipsp->ips_said.proto) {
        case IPPROTO_COMP:
                rc = ipsec_ocf_ipcomp_copy_expand (irs);
                if (rc != IPSEC_RCV_OK) {
                        KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: "
                                        "growing skb for ipcomp failed, rc=%d\n", rc);
                        return rc;
                }
                break;
        case IPPROTO_ESP:
        case IPPROTO_AH:
                break;
        default:
                KLIPS_PRINT(debug_rcv & DB_RX_XF, "klips_debug:ipsec_ocf_rcv: "
                                "bad protocol %d\n", ipsp->ips_said.proto);
                return IPSEC_RCV_BADPROTO;
        }

        req_count = (ipsp->ips_authalg ? 1 : 0)
                  + (ipsp->ips_encalg ? 1 : 0)
                  + (ipsp->ips_compalg ? 1 : 0);
	crp = crypto_getreq(req_count);
	if (!crp) {
		KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: "
				"crypto_getreq(%d) returned NULL\n", req_count);
		return IPSEC_RCV_REALLYBAD;
	}

        /* we currently don't support any chaining across protocols */
        switch(ipsp->ips_said.proto) {
        case IPPROTO_ESP:
		crda = crp->crp_desc;
		crde = crda->crd_next;
                break;
        case IPPROTO_COMP:
                crdc = crp->crp_desc;
                break;
        case IPPROTO_AH:
                crda = crp->crp_desc;
                break;
        }

	if (crda) {
		/* Authentication descriptor */
		crda->crd_alg = ipsec_ocf_authalg(ipsp->ips_authalg);
		if (!crda->crd_alg) {
			KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: "
					"bad auth alg 0x%x\n", ipsp->ips_authalg);
                        goto error_free_crp;
		}

		if (!crde) { /* assuming AH processing */
			/* push the IP header so we can authenticate it */
			skb_push(irs->skb, ((unsigned char *)irs->protostuff.ahstuff.ahp) -
								((unsigned char *)irs->ipp));
		}

		crda->crd_key          = ipsp->ips_key_a;
		crda->crd_klen         = ipsp->ips_key_bits_a;
		crda->crd_inject       = irs->authenticator - irs->skb->data;
		/* Copy the authenticator to check aganinst later */
		memcpy(irs->hash, irs->authenticator, 12);

		if (!crde) { /* assume AH processing */
			/* AH processing, save fields we have to zero */
			irs->ttl           = irs->ipp->ttl;
			irs->check         = irs->ipp->check;
			irs->frag_off      = irs->ipp->frag_off;
			irs->tos           = irs->ipp->tos;
			irs->ipp->ttl      = 0;
			irs->ipp->check    = 0;
			irs->ipp->frag_off = 0;
			irs->ipp->tos      = 0;
			crda->crd_len      = irs->skb->len;
			crda->crd_skip     = ((unsigned char *)irs->ipp) - irs->skb->data;
			memset(irs->authenticator, 0, 12);
		} else {
			crda->crd_len      = irs->ilen;
			crda->crd_skip     =
				((unsigned char *) irs->protostuff.espstuff.espp) -
							irs->skb->data;
			/* clear the authenticator to be sure */
			/* FIXME: don't do this as some drivers actually check this data */
			/* need to work out a cleaner way to ensure we do not see */
			/* the old value from the packet later */
			//memset(irs->authenticator, 0, 12);
		}
	}

	if (crde) {
		crde->crd_alg = ipsec_ocf_encalg(ipsp->ips_encalg);
		if (!crde->crd_alg) {
			KLIPS_PRINT(debug_rcv, "klips_debug:ipsec_ocf_rcv: "
					"bad enc alg 0x%x\n", ipsp->ips_encalg);
                        goto error_free_crp;
		}

		irs->esphlen     = ESP_HEADER_LEN + ipsp->ips_iv_size;
		irs->ilen       -= irs->esphlen;
		crde->crd_skip   = (irs->skb->transport_header - irs->skb->data) + irs->esphlen;
		crde->crd_len    = irs->ilen;
		crde->crd_inject = crde->crd_skip - ipsp->ips_iv_size;
		crde->crd_klen   = ipsp->ips_key_bits_e;
		crde->crd_key    = ipsp->ips_key_e;
	}

        if (crdc) {
                struct ipcomphdr *cmph;
                int compalg = ipsp->ips_compalg;
                /* Decompression descriptor */
#ifdef USE_OCF_FORCE_KNOBS
                // remove when we have working hardware support for crypto offload
                if (ocf_force_ipcomp_alg)
                        compalg = ocf_force_ipcomp_alg;
#endif
                crdc->crd_alg = ipsec_ocf_compalg(compalg);
                if (!crdc->crd_alg) {
                        KLIPS_PRINT(debug_tunnel&DB_TN_XMIT, "klips_debug:ipsec_ocf_rcv: "
                                        "bad decomp alg 0x%x\n",
                                        ipsp->ips_compalg);
                        goto error_free_crp;
                }
                crdc->crd_flags  = 0;
                // this is where the current ipcomp header is
                cmph = (struct ipcomphdr*)((char*)irs->ipp + irs->iphlen);
                // store the nested protocol
                irs->next_header = cmph->ipcomp_nh;
                // start decompressing after ip header and the ipcomp header
                crdc->crd_skip   = ((unsigned char*)irs->ipp) + irs->iphlen 
                                 + sizeof (struct ipcomphdr) - irs->skb->data;
                // decompress all ip data past the ipcomp header
                crdc->crd_len    = ntohs(irs->ipp->tot_len) - irs->iphlen 
                                 - sizeof (struct ipcomphdr);
                // decompress inplace (some hardware can only do inplace)
                crdc->crd_inject = crdc->crd_skip;
        }

	crp->crp_ilen = irs->skb->len; /* Total input length */
	crp->crp_olen = irs->skb->len; /* Total output length */
	crp->crp_flags =
			CRYPTO_F_SKBUF |
#if USE_CBIMM == 1
			CRYPTO_F_CBIMM |
#endif
#if USE_BATCH == 1
			CRYPTO_F_BATCH |
#endif
			0;
	crp->crp_buf = (caddr_t) irs->skb;
	crp->crp_callback = ipsec_ocf_rcv_cb;
	crp->crp_sid = ipsp->ocf_cryptoid;
	crp->crp_opaque = (caddr_t) irs;
	ocf_rcv_calls++;

	err = crypto_dispatch(crp);
	if(err != 0) {
		ocf_rcv_calls_failed++;
		KLIPS_RATEDEBUG(debug_ocf, "crypto_dispatch rcv failure #%u: %u \n", ocf_rcv_calls_failed, err);
		crypto_freereq(crp);
		return IPSEC_RCV_OCFFAIL;
	}
	return(IPSEC_RCV_PENDING);

error_free_crp:
        crypto_freereq(crp);
        return IPSEC_RCV_BADPROTO;
}


static int
ipsec_ocf_xmit_cb(struct cryptop *crp)
{
	struct ipsec_xmit_state *ixs = (struct ipsec_xmit_state *)crp->crp_opaque;
        struct iphdr *newiph;
        struct ipcomphdr *cmph;
        unsigned orig_len, comp_len;
	struct cryptodesc *crdc=NULL;

	KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit_cb\n");

	if (ixs == NULL) {
		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit_cb: "
				"NULL ixs in callback\n");
		return 0;
	}

	/*
	 * we must update the state before returning to the state machine.
	 * if we have an error,  terminate the processing by moving to the DONE
	 * state
	 */

	ixs->state = IPSEC_XSM_DONE; /* assume bad xmit */
	if (crp->crp_etype) {
                ptrdiff_t ptr_delta;

		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit_cb: "
				"error in processing 0x%x\n", crp->crp_etype);

                switch(ixs->ipsp->ips_said.proto) {
                case IPPROTO_COMP:
                        // It's ok for compression to fail... we made a clone
                        // of the packet, so we just revert it now...
                        if (! ixs->pre_ipcomp_skb) {
                                KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, 
                                        "klips_debug:ipsec_ocf_xmit_cb: "
                                        "IPcomp on %d bytes failed, "
                                        "but we have no clone!\n", 
                                        ntohs(ixs->iph->tot_len) - ixs->iphlen);

                                // this is a fail.
                                break;
                        }

                        KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, 
                                "klips_debug:ipsec_ocf_xmit_cb: "
                                "IPcomp on %d bytes failed, "
                                "using backup clone.\n", 
                                ntohs(ixs->iph->tot_len) - ixs->iphlen);

                        ptr_delta = ixs->pre_ipcomp_skb->data - ixs->skb->data;
                        ixs->iph           = (void*)((char*)ixs->iph + ptr_delta);

			/*
			 * can not free it here, because we are under
			 * IRQ, potentially, so queue it for later
			 */
			ipsec_skb_gc_enqueue(ixs->skb);

                        ixs->skb = ixs->pre_ipcomp_skb;
                        ixs->skb->network_header  += ptr_delta;
                        ixs->skb->transport_header   += ptr_delta;
                        ixs->pre_ipcomp_skb = NULL;

                        // this means we don't compress
                        ixs->state = IPSEC_XSM_CONT;
                        break;

                default:
                        break;
                }

                goto bail;
	}
        
	switch(ixs->ipsp->ips_said.proto) {
        case IPPROTO_ESP:
                /* ESP, nothing to do */
                break;

	case IPPROTO_AH:
                /* AH post processing, put back fields we had to zero */
                ixs->iph->ttl      = ixs->ttl;
                ixs->iph->check    = ixs->check;
                ixs->iph->frag_off = ixs->frag_off;
                ixs->iph->tos      = ixs->tos;
                break;

        case IPPROTO_COMP:
                /* IPcomp fill in the header */
		crdc = crp->crp_desc;

		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT,
			    "klips_debug:ipsec_ocf_xmit_cb: "
			    "after <%s%s%s>, SA:%s:\n",
			    IPS_XFORM_NAME(ixs->ipsp),
			    ixs->sa_len ? ixs->sa_txt : " (error)");
		KLIPS_IP_PRINT(debug_tunnel & DB_TN_XMIT, ixs->iph);

                orig_len = ntohs(ixs->iph->tot_len) - ixs->iphlen;
                comp_len = crp->crp_olen - crdc->crd_inject;

                newiph = (struct iphdr*)((char*)ixs->iph - sizeof (struct ipcomphdr));
                cmph = (struct ipcomphdr*)((char*)newiph + ixs->iphlen);

                // move the ip header to make room for the new ipcomp header
                skb_push (ixs->skb, sizeof (struct ipcomphdr));
                memmove (newiph, ixs->iph, ixs->iphlen);
                ixs->iph = newiph;

                ixs->skb->network_header -= sizeof (struct ipcomphdr);
                ixs->skb->transport_header -= sizeof (struct ipcomphdr);

                // now we can fill in the ipcomp header
                cmph->ipcomp_nh = ixs->next_header;
                cmph->ipcomp_flags = 0;
                cmph->ipcomp_cpi = htons((__u16)(ntohl(ixs->ipsp->ips_said.spi) & 0x0000ffff));

                // update the ip header to reflect the compression
                ixs->iph->protocol = IPPROTO_COMP;
                ixs->iph->tot_len = htons(ixs->iphlen 
                                + sizeof(struct ipcomphdr) 
                                + comp_len);
                ixs->iph->check = 0;
                ixs->iph->check = ip_fast_csum((char *) ixs->iph, ixs->iph->ihl);

                /* Update skb length/tail by "unputting" the shrinkage */
                safe_skb_put (ixs->skb, comp_len - orig_len);

                ixs->ipsp->ips_comp_adapt_skip = 0;
                ixs->ipsp->ips_comp_adapt_tries = 0;

                // relese the backup copy
                if (ixs->pre_ipcomp_skb) {
                        kfree_skb (ixs->pre_ipcomp_skb);
                        ixs->pre_ipcomp_skb = NULL;
                }
                
                break;
        }

        /* all good */
        ixs->state = IPSEC_XSM_CONT;

bail:
	crypto_freereq(crp);
	crp = NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	/* setup the rest of the processing now */
	PROCESS_NOW(ixs->workq, ipsec_xsm);
#else
	/* setup the rest of the processing now */
	PROCESS_NOW(ixs->workq, ipsec_xsm, ixs);
#endif
	return 0;
}


enum ipsec_xmit_value
ipsec_ocf_xmit(struct ipsec_xmit_state *ixs)
{
	struct cryptop *crp;
	struct cryptodesc *crde=NULL, *crda=NULL, *crdc=NULL;
	struct ipsec_sa *ipsp;
        int req_count, payload_size;
	int err;

	KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit\n");

	ipsp = ixs->ipsp;
	if (!ipsp) {
		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
				"no SA for rcv processing\n");
		return IPSEC_XMIT_SAIDNOTFOUND;
	}

	if (!ixs->skb) {
		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT,
				"klips_debug:ipsec_ocf_xmit: no skb\n");
		return IPSEC_XMIT_SAIDNOTFOUND;
	}

	switch(ipsp->ips_said.proto) {
        case IPPROTO_COMP:
                // skip packets that have less then 90 bytes of payload to
                // compress
                payload_size = ntohs(ixs->iph->tot_len) - ixs->iphlen;
                if (payload_size < 90) {
                        KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, 
                                        "klips_debug:ipsec_ocf_xmit: "
                                        "skipping IPcomp on packet with "
                                        "%d payload bytes\n", payload_size);
                        return IPSEC_XMIT_OK;
                }
                // there is a chance that we may not compress, and
                // since the compression overwrites the data, we will clone
                // the packet and restore it if we fail to compress
                KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, 
                                "klips_debug:ipsec_ocf_xmit: "
                                "IPcomp on %d bytes can fail, "
                                "duplicating the skb\n", payload_size);
                ixs->pre_ipcomp_skb = skb_copy_expand (ixs->skb,
                                skb_headroom(ixs->skb),
                                skb_tailroom(ixs->skb), GFP_ATOMIC);
                if (! ixs->pre_ipcomp_skb) {
                        // We can either drop the packet, but instead we try
                        // to do the compression as it might succeed.  Should it
                        // fail, the packet will be dropped in the callback.
                        KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, 
                                "klips_debug:ipsec_ocf_xmit: "
                                "skb_clone failed -- ignoring\n");
                }
                break;
        case IPPROTO_ESP:
	case IPPROTO_AH:
                break;
        default:
		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
				"bad protocol %d\n", ipsp->ips_said.proto);
                return IPSEC_XMIT_BADPROTO;
        }

        req_count = (ipsp->ips_authalg ? 1 : 0)
                  + (ipsp->ips_encalg ? 1 : 0)
                  + (ipsp->ips_compalg ? 1 : 0);
	crp = crypto_getreq(req_count);
	if (!crp) {
		KLIPS_PRINT(debug_tunnel & DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
				"crypto_getreq(%d) returned NULL\n", req_count);
		return IPSEC_XMIT_ERRMEMALLOC;
	}

        /* we currently don't support any chaining across protocols */
	switch(ipsp->ips_said.proto) {
        case IPPROTO_ESP:
		crde = crp->crp_desc;
		crda = crde->crd_next;
                break;
        case IPPROTO_COMP:
		crdc = crp->crp_desc;
                break;
	case IPPROTO_AH:
		crda = crp->crp_desc;
                break;
        }

	if (crda) {
		/* Authentication descriptor */
		crda->crd_alg = ipsec_ocf_authalg(ipsp->ips_authalg);
		if (!crda->crd_alg) {
			KLIPS_PRINT(debug_tunnel&DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
					"bad auth alg 0x%x\n", ipsp->ips_authalg);
                        goto error_free_crp;
		}
		if (!crde) { /* assume AH processing */
			/* AH processing, save fields we have to zero */
			crda->crd_skip     = ((unsigned char *) ixs->iph) - ixs->skb->data;
			ixs->ttl           = ixs->iph->ttl;
			ixs->check         = ixs->iph->check;
			ixs->frag_off      = ixs->iph->frag_off;
			ixs->tos           = ixs->iph->tos;
			ixs->iph->ttl      = 0;
			ixs->iph->check    = 0;
			ixs->iph->frag_off = 0;
			ixs->iph->tos      = 0;
			crda->crd_inject   =
				(((struct ahhdr *)(ixs->dat + ixs->iphlen))->ah_data) -
					ixs->skb->data;
			crda->crd_len      = ixs->len - ixs->authlen;
			memset(ixs->skb->data + crda->crd_inject, 0, 12); // DM
		} else {
			crda->crd_skip     = ((unsigned char *) ixs->espp) - ixs->skb->data;
			crda->crd_inject   = ixs->len - ixs->authlen;
			crda->crd_len      = ixs->len - ixs->iphlen - ixs->authlen;
		}
                crp->crp_mac     = ixs->skb->data + crda->crd_inject;
                crp->crp_maclen  = ixs->authlen;
		crda->crd_key    = ipsp->ips_key_a;
		crda->crd_klen   = ipsp->ips_key_bits_a;

		/* ipsec_dmp_block("ocf auth key (as submitted)", crda->crd_key, crda->crd_klen/8); */
	}

	if (crde) {
		/* Encryption descriptor */
		crde->crd_alg = ipsec_ocf_encalg(ipsp->ips_encalg);
		if (!crde->crd_alg) {
			KLIPS_PRINT(debug_tunnel&DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
					"bad enc alg 0x%x\n", ipsp->ips_encalg);
                        goto error_free_crp;
		}
		crde->crd_flags  = CRD_F_ENCRYPT;
		crde->crd_skip   = ixs->idat - ixs->dat;
		crde->crd_len    = ixs->ilen;
		crde->crd_inject = ((unsigned char *) ixs->espp->esp_iv) - ixs->dat;
		crde->crd_klen   = ipsp->ips_key_bits_e;
		crde->crd_key    = ipsp->ips_key_e;
	}

        if (crdc) {
                int compalg = ipsp->ips_compalg;
                /* Compression descriptor */
#ifdef USE_OCF_FORCE_KNOBS
                // remove when we have working hardware support for crypto offload
                if (ocf_force_ipcomp_alg)
                        compalg = ocf_force_ipcomp_alg;
#endif
                crdc->crd_alg = ipsec_ocf_compalg(compalg);
		if (!crdc->crd_alg) {
			KLIPS_PRINT(debug_tunnel&DB_TN_XMIT, "klips_debug:ipsec_ocf_xmit: "
					"bad comp alg 0x%x\n",
                                        ipsp->ips_compalg);
                        goto error_free_crp;
		}
		crdc->crd_flags  = CRD_F_ENCRYPT;
                // store the nested protocol
                ixs->next_header = ixs->iph->protocol;
                // start compressing after ip header
		crdc->crd_skip   = ((unsigned char*)ixs->iph) 
                        + ixs->iphlen - ixs->dat;
                // compress all ip data
		crdc->crd_len    = ntohs(ixs->iph->tot_len) - ixs->iphlen;
                // compress inplace (some hardware can only do inplace)
		crdc->crd_inject = crdc->crd_skip;
        }

	crp->crp_ilen = ixs->skb->len; /* Total input length */
	crp->crp_olen = ixs->skb->len; /* Total output length */
	crp->crp_flags =
			CRYPTO_F_SKBUF |
#if USE_CBIMM == 1
			CRYPTO_F_CBIMM |
#endif
#if USE_BATCH == 1
			CRYPTO_F_BATCH |
#endif
			0;
	crp->crp_buf = (caddr_t) ixs->skb;
	crp->crp_callback = ipsec_ocf_xmit_cb;
	crp->crp_sid = ipsp->ocf_cryptoid;
	crp->crp_opaque = (caddr_t) ixs;
	ocf_xmit_calls++;

	err = crypto_dispatch(crp);
	if(err != 0) {
		ocf_xmit_calls_failed++;
		KLIPS_RATEDEBUG(debug_ocf, "crypto_dispatch xmit failure #%u: %u (ENOMEM=%u)\n", ocf_xmit_calls_failed, err, ENOMEM);
		crypto_freereq(crp);
		return IPSEC_XMIT_OCFFAIL;
	}
	return(IPSEC_XMIT_PENDING);

error_free_crp:
	crypto_freereq(crp);
        return IPSEC_XMIT_BADPROTO;
}




#ifdef CONFIG_KLIPS_AH
static struct ipsec_alg_supported ocf_ah_algs[] = {
  {
	  .ias_name       = "ocf-md5hmac",
	  .ias_id         = AH_MD5,
	  .ias_exttype    = SADB_EXT_SUPPORTED_AUTH,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 128,
	  .ias_keymaxbits = 128,
  },
  {
	  .ias_name       = "ocf-sha1hmac",
	  .ias_id         = AH_SHA,
	  .ias_exttype    = SADB_EXT_SUPPORTED_AUTH,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 160,
	  .ias_keymaxbits = 160,
  },
  {
	  .ias_name       = NULL,
	  .ias_id         = 0,
	  .ias_exttype    = 0,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 0,
	  .ias_keymaxbits = 0,
  }
};
#endif /* CONFIG_KLIPS_AH */

static struct ipsec_alg_supported ocf_esp_algs[] = {
  {
	  .ias_name       = "ocf-md5hmac",
	  .ias_id         = AH_MD5,
	  .ias_exttype    = SADB_EXT_SUPPORTED_AUTH,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 128,
	  .ias_keymaxbits = 128,
  },
  {
	  .ias_name       = "ocf-sha1hmac",
	  .ias_id         = AH_SHA,
	  .ias_exttype    = SADB_EXT_SUPPORTED_AUTH,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 160,
	  .ias_keymaxbits = 160,
  },
  {
	  .ias_name       = "ocf-aes",
	  .ias_id         = ESP_AES,
	  .ias_exttype    = SADB_EXT_SUPPORTED_ENCRYPT,
	  .ias_ivlen      = 16,
	  .ias_keyminbits = 128,
	  .ias_keymaxbits = 256,
  },
  {
	  .ias_name       = "ocf-3des",
	  .ias_id         = ESP_3DES,
	  .ias_exttype    = SADB_EXT_SUPPORTED_ENCRYPT,
	  .ias_ivlen      = 8,
	  .ias_keyminbits = 192,
	  .ias_keymaxbits = 192,
  },
  {
	  .ias_name       = "ocf-des",
	  .ias_id         = ESP_DES,
	  .ias_exttype    = SADB_EXT_SUPPORTED_ENCRYPT,
	  .ias_ivlen      = 8,
	  .ias_keyminbits = 64,
	  .ias_keymaxbits = 64,
  },
  {
	  .ias_name       = NULL,
	  .ias_id         = 0,
	  .ias_exttype    = 0,
	  .ias_ivlen      = 0,
	  .ias_keyminbits = 0,
	  .ias_keymaxbits = 0,
  }
};

static int
ipsec_ocf_check_alg(struct ipsec_alg_supported *s)
{
	struct cryptoini cri;
	int64_t cryptoid;

	memset(&cri, 0, sizeof(cri));
	if (s->ias_exttype == SADB_EXT_SUPPORTED_ENCRYPT)
		cri.cri_alg  = ipsec_ocf_encalg(s->ias_id);
	else
		cri.cri_alg  = ipsec_ocf_authalg(s->ias_id);
	cri.cri_klen     = s->ias_keyminbits;
	cri.cri_key      = "0123456789abcdefghijklmnopqrstuvwxyz";

	if (crypto_newsession(&cryptoid, &cri, 0)) {
		KLIPS_PRINT(debug_ocf, "klips_debug:ipsec_ocf:%s not supported\n",
				s->ias_name);
		return 0;
	}
	crypto_freesession(cryptoid);
	KLIPS_PRINT(debug_ocf, "klips_debug:ipsec_ocf:%s supported\n",
			s->ias_name);
	return 1;
}

void
ipsec_ocf_init(void)
{
	struct ipsec_alg_supported *s;

	for (s = ocf_esp_algs; s->ias_name; s++) {
		if (ipsec_ocf_check_alg(s))
			(void)pfkey_list_insert_supported(s,
					&(pfkey_supported_list[SADB_SATYPE_ESP]));
	}

#ifdef CONFIG_KLIPS_AH
	for (s = ocf_ah_algs; s->ias_name; s++) {
		if (ipsec_ocf_check_alg(s))
			(void)pfkey_list_insert_supported(s,
					&(pfkey_supported_list[SADB_SATYPE_AH]));
	}
#endif

	/* send register event to userspace	*/
	pfkey_register_reply(SADB_SATYPE_ESP, NULL);
	pfkey_register_reply(SADB_SATYPE_AH, NULL);
}

