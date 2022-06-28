/*    __  __ _____ _____   ____  __  __ _____ _____ ____
 *   |  \/  |_   _|  __ \ / __ \|  \/  |_   _/ ____/ __ \
 *   | \  / | | | | |__) | |  | | \  / | | || |   | |  | |
 *   | |\/| | | | |  _  /| |  | | |\/| | | || |   | |  | |
 *   | |  | |_| |_| | \ \| |__| | |  | |_| || |___| |__| |
 *   |_|  |_|_____|_|  \_\\____/|_|  |_|_____\_____\____/
 *
 * Copyright (c) 2019 Miromico AG
 * All rights reserved.
 */

#include "lmic.h"
#include "hw.h"
#include "i2c.h"

#include "sht21.h"

static struct {
    unsigned char buf[6];

    osjobcb_t cb;

    eDeviceStatus_t* pstatus;
    sht21_data* pdata;
} sht21;

static const uint8_t SHT21_ADDRESS = 0x80;

static const uint8_t CMD_TRIGGER_T_NH   = 0xF3;
static const uint8_t CMD_TRIGGER_RH_NH  = 0xF5;
static const uint8_t CMD_READ_USR_REG   = 0xEF;

static const uint8_t DELAY_MEASURE_T    = 85;
static const uint8_t DELAY_MEASURE_RH   = 29;

static uint8_t sht21_generate_crc(uint8_t* data, uint16_t count);
static inline bool sht21_check_crc(uint8_t* data, uint16_t count);

#define I2C_TIMEOUT   ms2osticks(500)

static void init_func(osjob_t* job) {
    static enum {
        INIT, CFG_DONE
    } state;
    static int i2c_status;

    switch (state) {
    default:
    case INIT:
        sht21.buf[0] = CMD_READ_USR_REG;

        i2c_xfer_ex(SHT21_ADDRESS, sht21.buf, 1, 1, I2C_TIMEOUT, job, init_func,
                    &i2c_status);
        break;

    case CFG_DONE:
        if (i2c_status == I2C_OK) {
            *sht21.pstatus = eDevice_Initialized;
        } else {
            *sht21.pstatus = eDevice_Failed;
        }

        // done -- invoke application callback
        os_setCallback(job, sht21.cb);
        state = INIT;
        return;
    }
    state += 1;
}

static void read_func(osjob_t* job) {
    static enum {
        INIT, TRIGGERED_T, READ_T, DONE_T, TRIGGERED_RH, READ_RH, DONE,
    } state;
    static int i2c_status;

    switch (state) {
    default:
    case INIT: {
        // Trigger temperature measurement
        sht21.buf[0] = CMD_TRIGGER_T_NH;

        i2c_xfer_ex(SHT21_ADDRESS, sht21.buf, 1, 0, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;
    }

    case TRIGGERED_T:
        if (i2c_status != I2C_OK) {
            goto error;
        }

        // wait for conversion to complete
        os_setTimedCallback(job, os_getTime() + ms2osticks(DELAY_MEASURE_T), read_func);
        break;

    case READ_T:
        i2c_xfer_ex(SHT21_ADDRESS, sht21.buf, 0, 3, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;

    case DONE_T: {
        if (i2c_status != I2C_OK || !sht21_check_crc(sht21.buf, 2)) {
            goto error;
        }

        int32_t temp_ticks = (sht21.buf[1] & 0xfc) | ((int32_t)sht21.buf[0] << 8);
        sht21.pdata->temp = (((-4685 * 16384) + 4393 * (int32_t)temp_ticks + 8192) / 16384);

        // Trigger rH measurement
        sht21.buf[0] = CMD_TRIGGER_RH_NH;

        i2c_xfer_ex(SHT21_ADDRESS, sht21.buf, 1, 0, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;
    }

    case TRIGGERED_RH:
        if (i2c_status != I2C_OK) {
            goto error;
        }

        // wait for conversion to complete
        os_setTimedCallback(job, os_getTime() + ms2osticks(DELAY_MEASURE_RH), read_func);
        break;

    case READ_RH:
        i2c_xfer_ex(SHT21_ADDRESS, sht21.buf, 0, 3, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;

    case DONE: {
        if ((i2c_status != I2C_OK) || !sht21_check_crc(sht21.buf, 2)) {
            goto error;
        }
        uint32_t rh_ticks = (sht21.buf[1] & 0xfc) | ((int32_t)sht21.buf[0] << 8);
        sht21.pdata->rh = (((-12 * 32768) + 125 * rh_ticks + 16384) / 32768);
        // all done
        *sht21.pstatus = eDevice_Ok;
        goto done;
    }
    }
    state += 1;
    return;

error:
    *sht21.pstatus = eDevice_Failed;
done:
    state = INIT;
    os_setCallback(job, sht21.cb);
}

static inline bool sht21_check_crc(uint8_t* data, uint16_t count) {
    return sht21_generate_crc(data, count) == data[count];
}

#define POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001

static uint8_t sht21_generate_crc(uint8_t* data, uint16_t count) {
    uint8_t crc = 0;
    uint8_t byteCtr;

    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < count; ++byteCtr) {
        crc ^= (data[byteCtr]);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) { crc = (crc << 1) ^ POLYNOMIAL; }
            else { crc = (crc << 1); }
        }
    }
    return crc;
}

void sht21_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus) {
    sht21.cb = cb;
    sht21.pstatus = pstatus;
    os_setCallback(job, init_func);
}

void sht21_read(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus, sht21_data* pdata) {
    sht21.cb = cb;
    sht21.pstatus = pstatus;
    sht21.pdata = pdata;
    os_setCallback(job, read_func);
}
