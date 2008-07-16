/*
 * Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Daniela Sirocchi <r52684@freescale.com>
 * Maintainer: Olivia Yin <r63875@@freescale.com> for MPC83xxE family
 *
 * Description:
 * ocftest.c - DES known-answer test for SEC2 device driver
 *
 * Changelog:
 *
 * July, 2007, Roy Zang <tie-fei.zang@freescale.com>
 * 	- Change For MPC8544DS board SEC driver test
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/dcache.h>
#include <linux/file.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#if 0
#include <crypto/cryptodev.h>
#endif
#if 1
#include <opencrypto/crypto.h>
#include <opencrypto/cryptodev.h>
#endif

#define AES_KEYSIZE     (24)//16 24 32
#define AES_CTXSIZE     (8)
#define AES_MAXTESTSIZE (1024)

static const unsigned char RC4Data1[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
};
static const unsigned char result1[] = { 
	0x75, 0xb7, 0x87, 0x80, 0x99, 0xe0, 0xc5, 0x96
};
static const unsigned char arc4Key1[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
};
static int arc4KeyLen1 = 8;

static const unsigned char RC4Data2[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char result2[] = {
        0x74, 0x94, 0xc2, 0xe7, 0x10, 0x4b, 0x08, 0x79
};
static const unsigned char arc4Key2[] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
};
static int arc4KeyLen2 = 8;

static const unsigned char RC4Data3[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char result3[] = {
        0xde, 0x18, 0x89, 0x41, 0xa3, 0x37, 0x5d, 0x3a
};
static const unsigned char arc4Key3[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static int arc4KeyLen3 = 8;

static const unsigned char RC4Data4[] = {
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
	0x12, 0x34, 0x56, 0x78
};/*len = 28*/
static const unsigned char result4[] = {
	0x66, 0xa0, 0x94, 0x9f, 0x8a, 0xf7, 0xd6, 0x89,
	0x1f, 0x7f, 0x83, 0x2b, 0xa8, 0x33, 0xc0, 0x0c,
	0x89, 0x2e, 0xbe, 0x30, 0x14, 0x3c, 0xe2, 0x87,
	0x40, 0x01, 0x1e, 0xcf
};
static const unsigned char arc4Key4[] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
};
static int arc4KeyLen4 = 8;

static const unsigned char RC4Data5[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
};/*len = 8*/
static const unsigned char result5[] = {
	0x69, 0x72, 0x36, 0x59, 0x1B, 0x52, 0x42, 0xB1
};
static const unsigned char arc4Key5[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static int arc4KeyLen5 = 16;

static int cryptotest_cb(void *op)
{
	struct cryptop *crp = (struct cryptop *) op;

	printk("%s()\n", __FUNCTION__);
	if (crp->crp_etype == EAGAIN) {
		crp->crp_flags &= ~CRYPTO_F_DONE;
#ifdef NOTYET
		/*
		 * DAVIDM I am fairly sure that we should turn this into a batch
		 * request to stop bad karma/lockup, revisit
		 */
		crp->crp_flags |= CRYPTO_F_BATCH;
#endif
		return crypto_dispatch(crp);
	}

	wake_up_interruptible(&crp->crp_waitq);

	return 0;
}

int testArc4(int len, int b, int num)
{
	int status=0;
	int keybytes;
	int ivbytes = 259;
	struct cryptoini cri;
	struct cryptop *crp;
	struct cryptodesc *crd=NULL;
	u_int64_t sid;
	int i;
	unsigned char *arc4DecResult;
	unsigned char *arc4EncResult;
	unsigned char *arc4_key;

	if (len > 1024)
	{
		printk("%s: test data size limit is %d\n", __FUNCTION__, AES_MAXTESTSIZE);
		return -1;
	}

	arc4DecResult = kmalloc(len, GFP_KERNEL);
	arc4EncResult = kmalloc(len, GFP_KERNEL);
	memset(&cri, 0, sizeof(cri));
	printk("%s: arc4DecResult %x,arc4EncResult %x\n", __FUNCTION__, arc4DecResult, arc4EncResult);
	if (b == 0) {
		printk("\n*** Test ARC4 ECB *** %d size, klen %d\n", len, 16);
		keybytes = 16;
		cri.cri_alg = CRYPTO_ARC4;
	}else {
		printk("\n*** Test ARC4 ECB *** %d size, klen %d\n", len, 8);
		keybytes = 8;
		cri.cri_alg = CRYPTO_ARC4;
	}

/*************************************************************/
	/* Encrypto */
	arc4_key = kmalloc(keybytes, GFP_KERNEL);
	if (num == 1){
		memcpy(arc4_key, arc4Key1, keybytes);
		memcpy(arc4EncResult, RC4Data1, len);
	}
	else if (num == 2){
                memcpy(arc4_key, arc4Key2, keybytes);
                memcpy(arc4EncResult, RC4Data2, len);
        }
        else if (num == 3){
                memcpy(arc4_key, arc4Key3, keybytes);
                memcpy(arc4EncResult, RC4Data3, len);
        }
        else if (num == 4){
                memcpy(arc4_key, arc4Key4, keybytes);
                memcpy(arc4EncResult, RC4Data4, len);
        }
        else if (num == 5){
                memcpy(arc4_key, arc4Key5, keybytes);
                memcpy(arc4EncResult, RC4Data5, len);
        }
	else {
		printk("ARC test do not support\n");	
	}

	cri.cri_klen= keybytes*8;
	cri.cri_key = (caddr_t)arc4_key;

	status = crypto_newsession(&sid, &cri, CRYPTO_ANYHARDWARE);
	if (status) {
		printk("%s - newsession %d\n", __FUNCTION__, status);
		return -1;
	}

	crp = crypto_getreq(1);

	if (crp == NULL) {
		printk("%s: ENOMEM\n", __FUNCTION__);
		status = ENOMEM;
		return status;
	}

	crd = crp->crp_desc;
	crd->crd_alg = cri.cri_alg;
	crd->crd_key = cri.cri_key;
	crd->crd_klen = cri.cri_klen;
	crd->crd_len = len;
	crd->crd_skip = 0;
	//memcpy(crd->crd_iv, cri.cri_iv, ivbytes);
	//crd->crd_iv = 0;
	crd->crd_flags |= CRD_F_ENCRYPT | CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT | CRD_ARC4_FIRST;

	crp->crp_buf = (caddr_t)arc4EncResult;
	crp->crp_sid = sid;
	crp->crp_ilen = len;
	crp->crp_flags = CRYPTO_F_CBIMM;
	crp->crp_callback = (int (*) (struct cryptop *)) cryptotest_cb;
	status = crypto_dispatch(crp);
	if (status == 0) {
		printk("%s 1 about to WAIT\n", __FUNCTION__);
		//
		// we really need to wait for driver to complete to maintain
		// state,  luckily interrupts will be remembered
		//
		do {
			status = wait_event_interruptible(crp->crp_waitq,
					((crp->crp_flags & CRYPTO_F_DONE) != 0));
			//
			// we can't break out of this loop or we will leave behind
			// a huge mess,  however,  staying here means if your driver
			// is broken user applications can hang and not be killed.
			// The solution,  fix your driver :-)
			//
			// XXX - MCR says BS, processes should always be killable.
			//
			if (status) {
				schedule();
				status = 0;
			}
		} while ((crp->crp_flags & CRYPTO_F_DONE) == 0);
		printk("%s 1 finished WAITING status=%d\n", __FUNCTION__, status);
	}

	if (crp->crp_etype != 0) {
		status = crp->crp_etype;
		printk("%s error in crp processing %x\n", __FUNCTION__,status);
		return -1;
	}

	/*----------------------------------------------------------------------*/
	if (num == 1) {
		if ((memcmp(result1, arc4EncResult, len)) == 0) {
			printk("*** Test ARC4 ECB Enc Passed ***\n");
			status = 0;
		} else {
			printk("*** Test ARC4 ECB Enc Failed ***\n");
			status = -1;
		}
	}
	else if (num == 2) {
                if ((memcmp(result2, arc4EncResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test ARC4 ECB Enc Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 3) {
                if ((memcmp(result3, arc4EncResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test ARC4 ECB Enc Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 4) {
                if ((memcmp(result4, arc4EncResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test ARC4 ECB Enc Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 5) {
                if ((memcmp(result5, arc4EncResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test ARC4 ECB Enc Failed ***\n");
                        status = -1;
		}
        }

        for(i=1;i<=8;i++){
                printk("0x%x ",*(arc4EncResult+i));
        }
	printk("\n");
	//memcpy(cri.cri_iv, crd->crd_iv, ivbytes);
	/************************8 Decrypto *****************************/
	memcpy(arc4DecResult, arc4EncResult, len);
	//memcpy(crd->crd_iv, cri.cri_iv, ivbytes);
	//crd->crd_iv = NULL;
	crd->crd_next = 0;
	crp->crp_buf = (caddr_t)arc4DecResult;
	crd->crd_flags &= ~CRD_F_ENCRYPT;
	crd->crd_flags |= CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT | CRD_ARC4_FIRST;
	crp->crp_flags = CRYPTO_F_CBIMM;
	status = crypto_dispatch(crp);
	if (status == 0) {
		printk("%s 2 about to WAIT\n", __FUNCTION__);
		do {
			status = wait_event_interruptible(crp->crp_waitq,
					((crp->crp_flags & CRYPTO_F_DONE) != 0));
			if (status) {
				schedule();
				status = 0;
			}
		} while ((crp->crp_flags & CRYPTO_F_DONE) == 0);
		printk("%s 2 finished WAITING error=%d\n", __FUNCTION__, status);
	}

	if (crp->crp_etype != 0) {
		status = crp->crp_etype;
		printk("%s error in crp processing %x\n", __FUNCTION__,status);
		return -1;
	}

	/*compare */
	if (num == 1) {
		if ((memcmp(RC4Data1, arc4DecResult, len)) == 0) {
			printk("*** Test ARC4 ECB Dec Passed ***\n");
			status = 0;
		} else {
			printk("*** Test AES ECB Dec Failed ***\n");
			status = -1;
		}
	}
        else if (num == 2) {
                if ((memcmp(RC4Data2, arc4DecResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Dec Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES ECB Dec Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 3) {
                if ((memcmp(RC4Data3, arc4DecResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Dec Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES ECB Dec Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 4) {
                if ((memcmp(RC4Data4, arc4DecResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Dec Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES ECB Dec Failed ***\n");
                        status = -1;
		}
        }
        else if (num == 5) {
                if ((memcmp(RC4Data5, arc4DecResult, len)) == 0) {
                        printk("*** Test ARC4 ECB Dec Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES ECB Dec Failed ***\n");
                        status = -1;
		}
        }


	for(i=1;i<=8;i++){
		printk("0x%x",*(arc4DecResult+i));
	//	desDecResult++;
	}
	printk("\n");
	for(i=1;i<=8;i++){
                printk("0x%x",RC4Data1[i]);
        //      desDecResult++;
        
        }              

	kfree(arc4DecResult);
	kfree(arc4EncResult);
	kfree(arc4_key);
	crypto_freesession(sid);
	printk("\n*** Test ARC4 ECB Done ***\n");

	return status;
}


/* Top level of test */
int testArc4All(void)
{
	int status=0, nFails=0, nTests=0;

	/* All tests complete, show status and exit */
	if ((status = testArc4(8, 1, 1)) == 0)  
		nTests++;
	else
		printk("testArc4(%d) Failed! %04x\n", ++nFails, status);
/*
        if ((status = testArc4(8, 1, 2)) == 0)  
                nTests++;
        else
                printk("testArc4(%d) Failed! %04x\n", ++nFails, status);

        if ((status = testArc4(8, 1, 3)) == 0)  
                nTests++;
        else
                printk("testArc4(%d) Failed! %04x\n", ++nFails, status);
        if ((status = testArc4(28, 1, 4)) == 0)  
                nTests++;
        else
                printk("testArc4(%d) Failed! %04x\n", ++nFails, status);
        if ((status = testArc4(8, 0, 5)) == 0)  
                nTests++;
        else
                printk("testArc4(%d) Failed! %04x\n", ++nFails, status);
*/

	if (nFails == 0)
		printk("testAll(): All %d Tests Passed\n", nTests);
	else
		printk("Tests Passed %d, Failed %d!\n", nTests, nFails);

	if (nFails)
		return -1;

	return 0;
}

static int __init sec1test_arc4_init(void)
{
	testArc4All();
	return 0;
}

static void __exit sec1test_arc4_exit(void)
{
	return;
}

module_init(sec1test_arc4_init);
module_exit(sec1test_arc4_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC1.x driver test module");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
