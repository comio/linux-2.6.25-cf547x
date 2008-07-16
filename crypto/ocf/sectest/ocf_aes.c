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

static const unsigned char aesData1[] = "Single block msg";
static const unsigned char result1[] = { 
	0xe3, 0x53, 0x77, 0x9c, 0x10, 0x79, 0xae, 0xb8,
	0x27, 0x08, 0x94, 0x2d, 0xbe, 0x77, 0x18, 0x1a 
};

static const unsigned char aesData2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};

static const unsigned char result2[] = { 
	0xd2, 0x96, 0xcd, 0x94, 0xc2, 0xcc, 0xcf, 0x8a,
	0x3a, 0x86, 0x30, 0x28, 0xb5, 0xe1, 0xdc, 0x0a,
	0x75, 0x86, 0x60, 0x2d, 0x25, 0x3c, 0xff, 0xf9,
	0x1b, 0x82, 0x66, 0xbe, 0xa6, 0xd6, 0x1a, 0xb1 
};

static const unsigned char aesKey1[] = {
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
       	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06 
};

static const unsigned char aesKey2[] = {
	0xc2, 0x86, 0x69, 0x6d, 0x88, 0x7c, 0x9a, 0xa0,
	0x61, 0x1b, 0xbb, 0x3e, 0x20, 0x25, 0xa4, 0x5a 
};
static const unsigned char iv_in1[] = { 
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,
        0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41 
};

static const unsigned char iv_in2[] = { 
	0x56, 0x2e, 0x17, 0x99, 0x6d, 0x09, 0x3d, 0x28,
        0xdd, 0xb3, 0xba, 0x69, 0x5a, 0x2e, 0x6f, 0x58 
};

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

int testAes(int len, int b)
{
	int status=0;
	int keybytes;
	int ivbytes = 16;
	struct cryptoini cri;
	struct cryptop *crp;
	struct cryptodesc *crd=NULL;
	u_int64_t sid;
	int i;
	unsigned char *aesDecResult;
	unsigned char *aesEncResult;
	unsigned char *aes_key;

	if (len > 1024)
	{
		printk("%s: test data size limit is %d\n", __FUNCTION__, AES_MAXTESTSIZE);
		return -1;
	}

	aesDecResult = kmalloc(len, GFP_KERNEL);
	aesEncResult = kmalloc(len, GFP_KERNEL);
	memset(&cri, 0, sizeof(cri));
	printk("%s: aesDecResult %x,aesEncResult %x\n", __FUNCTION__, aesDecResult, aesEncResult);
	if (b == 0) {
		printk("\n*** Test AES CBC *** %d size\n", len);
		keybytes = 16;
		cri.cri_alg = CRYPTO_AES_CBC;
	}else {
		printk("\n*** Test AES ECB *** %d size\n", len);
		keybytes = 16;
		cri.cri_alg = CRYPTO_AES_CBC;
	}

/*************************************************************/
	/* Encrypto */
	aes_key = kmalloc(keybytes, GFP_KERNEL);
	if(len == 16) {
		memcpy(aes_key, aesKey1, keybytes);
		memcpy(aesEncResult, aesData1, len);
	}
	else if (len == 32) {
		memcpy(aes_key, aesKey2, keybytes);
                memcpy(aesEncResult, aesData2, len);
	}
	else {
		printk("The test do not support the len \n");
	}
	cri.cri_klen= keybytes*8;
	cri.cri_key = (caddr_t)aes_key;
	if(len == 16) {
		memcpy(cri.cri_iv, iv_in1, ivbytes);
	}
	else if(len == 32) {
        	memcpy(cri.cri_iv, iv_in2, ivbytes);
	}
        else {
                printk("The test do not support the len \n");
        }

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
	memcpy(crd->crd_iv, cri.cri_iv, ivbytes);
	crd->crd_flags |= CRD_F_ENCRYPT | CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT;

	crp->crp_buf = (caddr_t)aesEncResult;
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
        if(len == 16) {
                if ((memcmp(result1, aesEncResult, len)) == 0) {
                        printk("*** Test AES CBC Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES CBC Enc Failed ***\n");
                        status = -1;
                }
        }
        else if(len == 32) {
                if ((memcmp(result2, aesEncResult, len)) == 0) {
                        printk("*** Test AES CBC Enc Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES CBC Enc Failed ***\n");
                        status = -1;
                }
        }
        else {
                printk("The test case do not support the Len\n");
        }

	/* Decrypto */
	memcpy(aesDecResult, aesEncResult, len);
	memcpy(crd->crd_iv, cri.cri_iv, ivbytes);
	crp->crp_buf = (caddr_t)aesDecResult;
	crd->crd_flags &= ~CRD_F_ENCRYPT;
	crd->crd_flags |= CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT;
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
	if(len == 16) {
		if ((memcmp(aesData1, aesDecResult, len)) == 0) {
			printk("*** Test AES CBC Dec Passed ***\n");
			status = 0;
		} else {
			printk("*** Test AES CBC Dec Failed ***\n");
			status = -1;
		}
	}
	else if(len == 32) {
                if ((memcmp(aesData2, aesDecResult, len)) == 0) {
                        printk("*** Test AES CBC Dec Passed ***\n");
                        status = 0;
                } else {
                        printk("*** Test AES CBC Dec Failed ***\n");
                        status = -1;
                }
        }
        else {
		printk("The test case do not support the Len\n");
	}


	for(i=1;i<=8;i++){
		printk("0x%x",*(aesDecResult+i));
	//	desDecResult++;
	}
	printk("\n");
	for(i=1;i<=8;i++){
		if(len == 16) {
                	printk("0x%x",aesData1[i]);
        //      desDecResult++;
                } 
		else if (len == 32) {
                	printk("0x%x",aesData2[i]);
		}
        }              
printk("%s: aesDecResult %x,aesEncResult %x\n",__FUNCTION__,aesDecResult,aesEncResult);
printk("\n*** Test AES CBC 1*\n");
	kfree(aesDecResult);
printk("\n*** Test AES CBC 2*\n");
	kfree(aesEncResult);\
printk("\n*** Test AES CBC 3*\n");
	kfree(aes_key);
	printk("\n*** Test AES CBC 4*\n");
	crypto_freesession(sid);
	printk("\n*** Test AES CBC Done ***\n");

	return status;
}


/* Top level of test */
int testAesAll(void)
{
	int status=0, nFails=0, nTests=0;

	/* All tests complete, show status and exit */
	if ((status = testAes(16, 1)) == 0)  /* DES (0x2.) */
		nTests++;
	else
		printk("testAes(%d) Failed! %04x\n", ++nFails, status);

        if ((status = testAes(32, 0)) == 0)  /* DES (0x2.) */
                nTests++;
        else
                printk("testAes(%d) Failed! %04x\n", ++nFails, status);

	if (nFails == 0)
		printk("testAll(): All %d Tests Passed\n", nTests);
	else
		printk("Tests Passed %d, Failed %d!\n", nTests, nFails);

	if (nFails)
		return -1;

	return 0;
}

static int __init sec1test_aes_init(void)
{
	testAesAll();
	return 0;
}

static void __exit sec1test_aes_exit(void)
{
	return;
}

module_init(sec1test_aes_init);
module_exit(sec1test_aes_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC1.x driver test module");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
