/*     _____                _        __     _
 *    /__   \_ __ __ _  ___| | __ /\ \ \___| |_
 *      / /\/ '__/ _` |/ __| |/ //  \/ / _ \ __|
 *     / /  | | | (_| | (__|   '/ /\  /  __/ |_
 *     \_\  |_|  \__,_|\___|_|\_\_\ \_\\___|\__|
 *
 * Copyright (c) 2016-2017 Trackio International AG
 * All rights reserved.
 *
 */

#ifndef _perso_h_
#define _perso_h_

#if defined(CFG_perso)

void perso_enter (void);

enum {
    RSP_OK		= 0x00,
    RSP_EPARAM		= 0x80,
    RSP_EINTERNAL	= 0x81,
    RSP_ETIMEOUT	= 0x82,
    RSP_WTX		= 0xfe,
    RSP_NOTIMPL		= 0xff,
};

// glue functions
// TBD

#endif

#endif
