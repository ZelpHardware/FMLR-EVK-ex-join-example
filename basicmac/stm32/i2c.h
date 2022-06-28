// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _I2C_H_
#define _I2C_H_

#include "board.h"
#include "oslmic.h"

//////////////////////////////////////////////////////////////////////
// I2C
//////////////////////////////////////////////////////////////////////

#ifdef BRD_I2C

#define I2C_BUSY	1
#define I2C_OK		0
#define I2C_NAK		-1
#define I2C_ABORT	-2
typedef void (*i2c_cb) (int status);
void i2c_xfer (unsigned int addr, unsigned char* buf, unsigned int wlen, unsigned int rlen, i2c_cb cb, ostime_t timeout);
void i2c_xfer_ex (unsigned int addr, unsigned char* buf, unsigned int wlen, unsigned int rlen, ostime_t timeout,
	osjob_t* job, osjobcb_t cb, int* pstatus);
void i2c_abort (void);

#endif

#endif
